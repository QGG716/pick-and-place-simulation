"""Full frozen top-layer native planning batches; no physical execution."""
import argparse
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from time import perf_counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.coupled_batch import BatchState,batch_scene,coupled_binding,CoupledBatchClient,CoupledCycleConnector
from unloading_sim.coupled_lift_checks import check_coupled_state
from unloading_sim.top_layer_lifted import FrozenTopLayer,rank_remaining
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.moveit2_backend import ResidentMoveItClient,box_message


def save(path,data):
    # Repeated native world samples are large. Stream compact JSON instead of
    # building an indented duplicate in memory; preserve the last complete file
    # if the filesystem fills during a later checkpoint. A partial file remains
    # visible as failed-write evidence and is never treated as a checkpoint.
    pending=path.with_suffix(path.suffix+'.partial')
    with pending.open('w',encoding='utf-8',newline='\n') as stream:
        json.dump(data,stream,ensure_ascii=False,allow_nan=False,separators=(',',':'))
        stream.write('\n')
    pending.replace(path)


def source_identity(args):
    if args.development:return dict(source_commit=args.source_commit,development=True)
    if args.source_manifest:
        m=json.loads(args.source_manifest.read_text())
        if m['source_commit']!=args.source_commit:raise ValueError('SOURCE_COMMIT_MISMATCH')
        actual={str(p.relative_to(ROOT)).replace('\\','/') for directory in ('src','tools','ros2','configs')
            for p in (ROOT/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
        if actual!=set(m['sha256']):raise ValueError('SOURCE_FILE_SET_MISMATCH')
        for file,sha in m['sha256'].items():
            p=(ROOT/file).resolve()
            if not p.is_relative_to(ROOT) or hashlib.sha256(p.read_bytes()).hexdigest()!=sha:raise ValueError('SOURCE_FILE_MISMATCH:'+file)
    else:
        if subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()!=args.source_commit:raise ValueError('SOURCE_COMMIT_MISMATCH')
        if subprocess.check_output(['git','diff','HEAD','--','src','tools','ros2','configs'],cwd=ROOT):raise ValueError('DIRTY_SOURCE')
    return dict(source_commit=args.source_commit,development=False)


def run_seed(args,original,built,frozen,seed,out):
    layout=original.policy.layout_validation.layout
    state=BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),
        float(original.receiver.center[2]+original.receiver.half_extents[2]))
    initial=state.root();attempts=[];began=perf_counter();attempt_no=0
    flags=dict(qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    def result(status):
        n=len(state.ledger)
        return dict(schema='m710_coupled_top_layer_batch_v2',seed=seed,status=status,initial=initial,terminal=state.root(),
            completed_count=n,target_count=len(frozen.target_ids),remaining_target_ids=list(frozen.remaining_ids([e['target_id'] for e in state.ledger])),
            release_count=n,nominal_first_contact_count=n,separation_count=n,ideal_drain_count=n,
            ledger=state.ledger,cycles=state.cycles,attempts=attempts,planning_wall_s=perf_counter()-began,
            lower_layer_preserved=all(any(box_message(b)==box_message(o) for b in state.cartons) for o in original.cartons if o.name not in frozen.target_ids),
            hardware_lift_duration_s=None,nominal_drop_assumed=True,ideal_receiver_drain_assumed=True,**flags)
    while frozen.remaining_ids([e['target_id'] for e in state.ledger]):
        scene=batch_scene(original,state,built.robot,layout)
        check=check_coupled_state(layout,state.height,state.cartons,designed_contacts=scene.snapshot['assembly']['designed_contacts'])
        save(out/f'world-{len(state.ledger)}.json',check)
        if not check['passed']:
            attempts.append(dict(failure='COMMITTED_WORLD_GEOMETRY_REJECTED',evidence=check));break
        ranked=rank_remaining(frozen,state.cartons,[e['target_id'] for e in state.ledger],state.q,built.robot,scene.receiver)
        accepted=False
        for ranked_target in ranked:
            target=next(b for b in state.cartons if b.name==ranked_target['target_id'])
            task_started=perf_counter();deadline=task_started+args.task_budget_s
            options=[]
            low=float(original.receiver.center[2]+original.receiver.half_extents[2])
            for option in ranked_target['options']:
                options.append(option)
                if state.height>low+.005:
                    options.append(dict(option,synchronization='CHECKED_PRE_APPROACH_LOW_THEN_PROCESS_HEIGHT',pre_approach_height_m=low))
                    options.append(dict(option,synchronization='CHECKED_LOW_APPROACH_EXTRACTION_THEN_ATTACHED_LIFT',pre_approach_height_m=low,late_receiver_sync=True))
            for option_index,option in enumerate(options):
                if perf_counter()>=deadline:
                    attempts.append(dict(target_id=target.name,complete=False,failure='SHARED_TASK_BUDGET_EXHAUSTED',
                        unsearched_process_options=options[option_index:],unsearched_option_count=len(options)-option_index,
                        classification='SEARCH_INCOMPLETE_NOT_GLOBAL_UNREACHABILITY',wall_s=0.))
                    break
                attempt_no+=1;folder=out/f'attempt-{attempt_no:03d}';folder.mkdir()
                row=dict(target_id=target.name,option=option,root=state.root(),folder=folder.name,complete=False)
                attempts.append(row);started=perf_counter();client=None;c=None
                print(json.dumps(dict(seed=seed,attempt=attempt_no,target=target.name,process=option['template'],completed=len(state.ledger))),flush=True)
                try:
                    binding=coupled_binding(layout,state.cartons,state.height,option['height_m'],target.name,scene.snapshot['assembly']['designed_contacts'])
                    if 'pre_approach_height_m' in option:
                        pre=coupled_binding(layout,state.cartons,state.height,low,target.name,scene.snapshot['assembly']['designed_contacts'])
                        binding=coupled_binding(layout,state.cartons,low,option['height_m'],target.name,scene.snapshot['assembly']['designed_contacts'])
                        binding['pre_approach_transition']=pre
                    os.environ['M710_MOVEIT_SEED']=str(seed)
                    client=CoupledBatchClient(ResidentMoveItClient(args.worker_command,log_path=str(folder/'worker.log')),binding=binding)
                    c=CoupledCycleConnector.from_existing(built,scene,native_client=client,planning_mode='cold_from_scratch',
                        native_budget=dict(task_wall_time_s=args.task_budget_s,stage_wall_time_s=args.stage_budget_s,ipc_timeout_s=args.ipc_timeout_s))
                    c.option=option;c.batch_root=state.root();c.template_provenance=dict(role=dict(role=option['original_role']),template=option['template'])
                    c.budget=replace(c.budget,planning_wall_time_s=args.task_budget_s,candidate_wall_time_s=None,
                        stage_wall_time_s=args.stage_budget_s,postprocess_wall_time_s=args.stage_budget_s)
                    c.stack_carton_names={b.name for b in state.cartons};c.robot_state_validator.stack_carton_names=set(c.stack_carton_names)
                    c.start_planning_request(task_started);c._native_task_started=task_started
                    c._request_deadline_monotonic=deadline;c._deadline_monotonic=deadline;c._final_export_reserve_s=0.;c.boundary_costs={}
                    faces,screen=simulation_contact_targets(scene,target,c.robot.base_transform[:3,3],c.budget.pregrasp_standoff_m)
                    selected=[f for f in faces if f.geometric_face_id==option['face']['geometric_face_id']]
                    row['screening']=screen
                    if not selected:
                        row['failure']='BUSINESS_FACE_NOT_CURRENTLY_EXPOSED';continue
                    if len(selected)!=1:raise ValueError('ONE_UPSTREAM_POSE_PER_FACE_REQUIRED')
                    plan,detail=c.plan_target(selected[0],state.q,scene.all_obstacles,scene.policy.data['suction'],seed)
                    save(folder/'attempt.json',detail)
                    if plan is not None:
                        state.commit(plan);row['complete']=True;accepted=True
                        save(folder/'plan.json',plan)
                        print(json.dumps(dict(seed=seed,completed=len(state.ledger),terminal_q=state.q.tolist(),height=state.height)),flush=True)
                        break
                    row['failure']=detail.get('failure');row['classification']='SEARCH_INCOMPLETE_NOT_GLOBAL_UNREACHABILITY'
                except Exception as exc:
                    row.update(failure=f'{type(exc).__name__}: {exc}',classification='REQUEST_FAILURE_RETAINED')
                    import traceback
                    (folder/'exception.txt').write_text(traceback.format_exc())
                finally:
                    row['wall_s']=perf_counter()-started
                    if client:
                        save(folder/'requests.json',client.records)
                        save(folder/'search-trace.json',dict(model=client.trace_inputs.get('model'),attempts=client.traces))
                        row['prior_calls']=sum('prior' in str(r.get('op')) or r.get('pipeline_id')=='static_prior' for r in client.records)
                        client.close()
                    save(folder/'summary.json',row);save(out/'result.json',result('RUNNING'))
            if accepted:break
        if not accepted:break
    final=result('COMPLETE_PLANNING_ONLY' if len(state.ledger)==len(frozen.target_ids) else 'INCOMPLETE_RETAINED')
    save(out/'result.json',final);return final


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--worker-command',required=True)
    p.add_argument('--worker-binary',type=Path,required=True);p.add_argument('--source-commit',required=True)
    p.add_argument('--source-manifest',type=Path);p.add_argument('--seeds',type=int,nargs='+',default=[71070,71071])
    p.add_argument('--task-budget-s',type=float,default=300.);p.add_argument('--stage-budget-s',type=float,default=90.)
    p.add_argument('--ipc-timeout-s',type=float,default=100.);p.add_argument('--development',action='store_true')
    args=p.parse_args()
    if len(set(args.seeds))!=len(args.seeds) or (len(args.seeds)<2 and not args.development):p.error('at least two independent prefixed seeds')
    if not 0<args.stage_budget_s<args.ipc_timeout_s or args.task_budget_s<=0:p.error('invalid resource limits')
    source=source_identity(args);args.output.mkdir(parents=True,exist_ok=False)
    policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
    original=build_verified_motion_input(policy,ROOT);frozen=FrozenTopLayer.capture(original.cartons)
    built=_build_automatic_trajectory_connector(original,policy.layout_validation.layout.robot())
    if built.connector is None:raise ValueError(built.failure_reason)
    save(args.output/'protocol.json',dict(**source,seeds=args.seeds,selection='FIXED_BEFORE_BATCH_RUNS',
        frozen=frozen.evidence(),worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
        configured_initial_q=original.snapshot['robot']['q_rad'],original_cartons=[box_message(b) for b in original.cartons],
        command=[sys.executable,*sys.argv],pair_profile_enabled=False,task_budget_s=args.task_budget_s,stage_budget_s=args.stage_budget_s))
    rows=[]
    for seed in args.seeds:
        out=args.output/f'seed-{seed}';out.mkdir();rows.append(run_seed(args,original,built.connector,frozen,seed,out))
        save(args.output/'summary.json',dict(**source,runs=rows,qualification_status='NOT_EVALUATED',continuous_swept_proof=False,
            physical_execution='NOT_RUN',execution_ready=False))
    print(json.dumps(dict(completed=[r['completed_count'] for r in rows],targets=len(frozen.target_ids))),flush=True)

if __name__=='__main__':main()
