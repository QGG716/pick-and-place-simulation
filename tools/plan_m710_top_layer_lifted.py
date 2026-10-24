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
from unloading_sim.moveit2_backend import MoveItUnavailable
from unloading_sim.junction_reception import synchronization_options
from unloading_sim.conveyor_lift import ScissorGeometryRangeError


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


def checkpoint_candidate(state,plan,client,folder,out,row,connections,*,initial_cycle_count=0):
    proposed=deepcopy(state);proposed.commit(plan)
    save(folder/'plan.json',plan)
    save(folder/'requests.json',client.records)
    save(folder/'search-trace.json',dict(model=client.trace_inputs.get('model'),attempts=client.traces))
    save(folder/'connection-attempts.json',connections)
    save(folder/'summary.json',dict(row,complete=True))
    required=['attempt.json','plan.json','requests.json','search-trace.json','connection-attempts.json','summary.json']
    if any(not (folder/name).is_file() or (folder/name).stat().st_size==0 for name in required):
        raise ValueError('CHECKPOINT_REQUIRED_EVIDENCE_MISSING')
    save(out/'checkpoint.json',dict(root=proposed.root(),completed_count=len(proposed.ledger),
        current_run_completed_count=len(proposed.cycles)-initial_cycle_count,
        historical_completed_count=len(proposed.ledger)-len(proposed.cycles)+initial_cycle_count,
        plan_file=str(folder/'plan.json'),required_evidence_files=required))
    return proposed


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


def run_seed(args,original,built,frozen,seed,out,*,initial_state=None,target_ids=None):
    layout=original.policy.layout_validation.layout
    state=initial_state or BatchState(frozen,tuple(original.cartons),np.asarray(original.snapshot['robot']['q_rad']),
        float(original.receiver.center[2]+original.receiver.half_extents[2]))
    historical_count=len(state.ledger);historical_ledger=deepcopy(state.ledger)
    initial_cycle_count=len(state.cycles)
    scope=tuple(frozen.target_ids if target_ids is None else target_ids)
    def remaining():
        return [n for n in frozen.remaining_ids([e['target_id'] for e in state.ledger]) if n in scope]
    initial=state.root();attempts=[];began=perf_counter();attempt_no=0
    flags=dict(qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
    def result(status):
        n=len(state.ledger)-historical_count
        return dict(schema='m710_coupled_top_layer_batch_v2',seed=seed,status=status,initial=initial,terminal=state.root(),
            completed_count=n,target_count=len(scope),remaining_target_ids=remaining(),
            historical_completed_count=historical_count,historical_ledger=historical_ledger,
            release_count=n,nominal_first_contact_count=n,separation_count=n,ideal_drain_count=n,
            ledger=state.ledger,cycles=state.cycles[initial_cycle_count:],attempts=attempts,planning_wall_s=perf_counter()-began,
            lower_layer_preserved=all(any(box_message(b)==box_message(o) for b in state.cartons) for o in original.cartons if o.name not in frozen.target_ids),
            hardware_lift_duration_s=None,nominal_drop_assumed=True,ideal_receiver_drain_assumed=True,**flags)
    while remaining():
        scene=batch_scene(original,state,built.robot,layout)
        check=check_coupled_state(layout,state.height,state.cartons,designed_contacts=scene.snapshot['assembly']['designed_contacts'])
        save(out/f'world-{len(state.ledger)}.json',check)
        if not check['passed']:
            attempts.append(dict(failure='COMMITTED_WORLD_GEOMETRY_REJECTED',evidence=check));break
        ranked=rank_remaining(frozen,state.cartons,[e['target_id'] for e in state.ledger],state.q,built.robot,scene.receiver,
            longitudinal=next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'))
        ranked=[r for r in ranked if r['target_id'] in scope]
        accepted=False
        for ranked_target in ranked:
            target=next(b for b in state.cartons if b.name==ranked_target['target_id'])
            task_started=perf_counter();deadline=task_started+args.task_budget_s
            shared_stages={'prefix':min(deadline,task_started+args.stage_budget_s)}
            options=[]
            low=float(getattr(args,'low_top_minimum_m',None) or
                (original.receiver.center[2]+original.receiver.half_extents[2]))
            business_options=ranked_target['options']
            low_top=False
            if getattr(args,'low_top_minimum_m',None) is not None:
                from unloading_sim.low_top_receiver import low_top_option
                option=low_top_option(target,scene.receiver,
                    next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'),
                    frozen,args.low_top_minimum_m,
                    definition=getattr(args,'low_definition','FROZEN_ORIGINAL_HMIN_BELOW_MINIMUM'))
                if option is not None:
                    business_options=[option];low_top=True
            if getattr(args,'junction_only',False):
                business_options=business_options[:1]
                if not business_options or business_options[0]['template']!='FRONT_DIRECT_HMIN' or not business_options[0]['reception']['junction']:
                    raise ValueError('REQUESTED_JUNCTION_FRONT_HMIN_NOT_APPLICABLE')
            from unloading_sim.low_top_receiver import layer_top_option,TOP_TEMPLATES
            layer_permission=getattr(args,'layer_top_permission',None)
            if layer_permission is not None:
                top_option=layer_top_option(target,scene.receiver,
                    next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'),
                    frozen,state.height,layer_permission)
                if top_option is not None:
                    # One new business face. Keep the existing front/side options
                    # and the same task/prefix deadlines; give top the first visit.
                    business_options=([top_option] if getattr(args,'layer_top_only',False)
                                      else [top_option,*business_options])
                elif getattr(args,'layer_top_only',False):
                    raise ValueError('TOP_ONLY_TARGET_OUTSIDE_FROZEN_LAYER_PERMISSION')
            routes_permission=getattr(args,'selected_routes_permission',None)
            if routes_permission is not None:
                from unloading_sim.l01_selected_routes import selected_route_option
                business_options=[selected_route_option(target,scene.receiver,
                    next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'),
                    frozen,state.height,routes_permission)]
            hmax_permission=getattr(args,'l01_hmax_permission',None)
            if hmax_permission is not None:
                from unloading_sim.l01_hmax import options as hmax_options
                business_options=hmax_options(target,scene.receiver,
                    next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal'),
                    frozen,state.height,hmax_permission)
            for option in business_options:
                domain=getattr(args,'receive_height_range',None)
                if domain is not None and not domain[0]-1e-9 <= option['height_m'] <= domain[1]+1e-9:
                    attempts.append(dict(target_id=target.name,option=option,complete=False,
                        failure='RECEIVE_HEIGHT_OUTSIDE_EXISTING_MECHANISM_RANGE',
                        classification='PROCESS_GEOMETRY_REJECTED_BEFORE_NATIVE_SEARCH',
                        receive_height_range_m=list(domain),wall_s=0.))
                    continue
                options.extend([option] if option['template'] in TOP_TEMPLATES or routes_permission is not None or hmax_permission is not None else synchronization_options(option,state.height,low,
                    include_receive_before_approach=getattr(args,'include_receive_before_approach',False)))
            eccentric=getattr(args,'right_front_eccentric',False) and not low_top
            visits=options;total_visits=len(options)
            if eccentric:
                from unloading_sim.right_front_eccentric import offset_intervals,candidate_visits,candidate_deltas
                faces,_=simulation_contact_targets(scene,target,built.robot.base_transform[:3,3],built.budget.pregrasp_standoff_m)
                front=next(t for t in faces if t.face_role=='FRONT')
                intervals=offset_intervals(front,frozen,scene.policy.data['suction'])
                # One approach problem serves both synchronization locations.
                options=[dict(o,synchronization='PREGRASP_OR_CONTACT') if o['synchronization']=='PREGRASP' else o
                         for o in options if o['synchronization']!='CONTACT']
                visits=candidate_visits(options,intervals)
                total_visits=len(options)*sum(1 for _ in candidate_deltas(intervals))
            for option_index,option in enumerate(visits):
                low_top=option['template'] in TOP_TEMPLATES
                if perf_counter()>=min(deadline,shared_stages['prefix']):
                    attempts.append(dict(target_id=target.name,complete=False,
                        failure='SHARED_TASK_BUDGET_EXHAUSTED' if perf_counter()>=deadline else 'SHARED_PREFIX_BUDGET_EXHAUSTED',
                        unsearched_process_options=options[option_index:] if not eccentric else None,
                        unsearched_option_count=total_visits-option_index,
                        classification='SEARCH_INCOMPLETE_NOT_GLOBAL_UNREACHABILITY',wall_s=0.))
                    break
                attempt_no+=1;folder=out/f'attempt-{attempt_no:03d}';folder.mkdir()
                row=dict(target_id=target.name,option=option,root=state.root(),folder=folder.name,complete=False)
                attempts.append(row);started=perf_counter();client=None;c=None;evidence_saved=False;fatal=False
                print(json.dumps(dict(seed=seed,attempt=attempt_no,target=target.name,process=option['template'],
                    completed=len(state.ledger)-historical_count,historical_completed=historical_count)),flush=True)
                try:
                    binding=coupled_binding(layout,state.cartons,state.height,option['height_m'],target.name,scene.snapshot['assembly']['designed_contacts'])
                    if 'pre_approach_height_m' in option:
                        temporary=option['pre_approach_height_m']
                        pre=coupled_binding(layout,state.cartons,state.height,temporary,target.name,scene.snapshot['assembly']['designed_contacts'])
                        binding=coupled_binding(layout,state.cartons,temporary,option['height_m'],target.name,scene.snapshot['assembly']['designed_contacts'])
                        binding['pre_approach_transition']=pre
                    binding['receive_contract']=dict(height_m=option['height_m'],initial_target=box_message(target),
                        junction_front=bool((option.get('reception') or {}).get('junction') and
                                            option['face']['geometric_face_id']=='x-'),third_sync_disabled=True)
                    if hmax_permission is not None:
                        binding['receive_contract']['l01_hmax_permission']=deepcopy(hmax_permission)
                    os.environ['M710_MOVEIT_SEED']=str(seed)
                    client=CoupledBatchClient(ResidentMoveItClient(args.worker_command,log_path=str(folder/'worker.log')),
                        binding=binding,expected_model_identity=getattr(args,'expected_model_identity',None))
                    connector_type=CoupledCycleConnector
                    if low_top:
                        from unloading_sim.low_top_receiver import LowTopReceiverConnector
                        connector_type=LowTopReceiverConnector
                    c=connector_type.from_existing(built,scene,native_client=client,planning_mode='cold_from_scratch',
                        native_budget=dict(task_wall_time_s=args.task_budget_s,stage_wall_time_s=args.stage_budget_s,ipc_timeout_s=args.ipc_timeout_s))
                    c.option=option;c.batch_root=state.root();c.template_provenance=dict(role=dict(role=option['original_role']),template=option['template'])
                    c.layer_top_permission=layer_permission
                    c.selected_routes_permission=routes_permission
                    c.l01_hmax_permission=hmax_permission
                    c.eccentric_frozen_layer=frozen if eccentric else None
                    c.upper_arm_preference_enabled=eccentric or getattr(args,'upper_arm_preference',False)
                    c.empty_approach_direct_first=args.direct_first
                    c.height_stage_deadlines=shared_stages;c.height_prefix_active=True
                    c.height_visit_deadline=(shared_stages['prefix'] if low_top or option.get('finite_side_channel') else
                        min(shared_stages['prefix'],perf_counter()+(10. if eccentric else 20.)))
                    if hmax_permission is not None and option_index==len(options)-1:
                        c.height_visit_deadline=shared_stages['prefix']
                    c.budget=replace(c.budget,planning_wall_time_s=args.task_budget_s,candidate_wall_time_s=None,
                        stage_wall_time_s=args.stage_budget_s,postprocess_wall_time_s=args.stage_budget_s)
                    c.stack_carton_names={b.name for b in state.cartons};c.robot_state_validator.stack_carton_names=set(c.stack_carton_names)
                    c.start_planning_request(task_started);c._native_task_started=task_started
                    c._request_deadline_monotonic=deadline;c._deadline_monotonic=deadline;c._final_export_reserve_s=0.;c.boundary_costs={}
                    side_depth=None
                    if option.get('finite_side_channel') or (hmax_permission is not None and option['face']['geometric_face_id']=='y+'):
                        from unloading_sim.l01_selected_routes import finite_side_exposure
                        row['finite_side_exposure']=finite_side_exposure(c,state.q)
                        side_depth=row['finite_side_exposure']['depth_m']
                    faces,screen=simulation_contact_targets(scene,target,c.robot.base_transform[:3,3],c.budget.pregrasp_standoff_m,
                        side_exposure_depth_m=side_depth)
                    selected=[f for f in faces if f.geometric_face_id==option['face']['geometric_face_id']]
                    row['screening']=screen
                    if not selected:
                        row['failure']='BUSINESS_FACE_NOT_CURRENTLY_EXPOSED';continue
                    if len(selected)!=1:raise ValueError('ONE_UPSTREAM_POSE_PER_FACE_REQUIRED')
                    chosen=selected[0]
                    if eccentric:
                        from unloading_sim.right_front_eccentric import eccentric_target
                        chosen=eccentric_target(chosen,frozen,option['world_y_offset_m'],scene.policy.data['suction'])
                        row['eccentric_domain_intervals_m']=intervals
                        row['candidate_id']=chosen.candidate_id
                        row['target']=chosen.evidence()
                    plan,detail=c.plan_target(chosen,state.q,scene.all_obstacles,scene.policy.data['suction'],seed)
                    save(folder/'attempt.json',detail)
                    if plan is not None:
                        # Save all required native evidence and the proposed
                        # checkpoint BEFORE changing the completed batch root.
                        state=checkpoint_candidate(state,plan,client,folder,out,dict(row,wall_s=perf_counter()-started),
                            c.connection_attempts,initial_cycle_count=initial_cycle_count)
                        row['complete']=True;accepted=True;evidence_saved=True
                        print(json.dumps(dict(seed=seed,completed=len(state.ledger)-historical_count,
                            historical_completed=historical_count,terminal_q=state.q.tolist(),height=state.height)),flush=True)
                        break
                    row['failure']=detail.get('failure');row['classification']='SEARCH_INCOMPLETE_NOT_GLOBAL_UNREACHABILITY'
                except Exception as exc:
                    if isinstance(exc,ScissorGeometryRangeError) and client is None:
                        # A fully specified physical mechanism cannot exist at
                        # this height. No native candidate was dispatched.
                        row.update(failure=str(exc),classification='PROCESS_GEOMETRY_REJECTED_BEFORE_NATIVE_SEARCH',
                            geometry_evidence=exc.evidence,requested_receive_height_m=option['height_m'],
                            requested_temporary_height_m=option.get('pre_approach_height_m'),
                            evidence_folder=row.pop('folder'),native_dispatch_count=0)
                        continue
                    row.update(failure=f'{type(exc).__name__}: {exc}',classification='REQUEST_FAILURE_RETAINED')
                    import traceback
                    (folder/'exception.txt').write_text(traceback.format_exc())
                    recoverable=isinstance(exc,MoveItUnavailable) and str(exc) in {
                        'NATIVE_REQUEST_SHARED_BUDGET_EXHAUSTED','SINGLE_TARGET_STAGE_DEADLINE',
                        'PRE_APPROACH_COUPLED_SWEEP_REJECTED','RECEIVER_SYNC_CANDIDATE_REJECTED','RECEIVER_REGION_STAGE_DEADLINE'}
                    if not recoverable:
                        fatal=True
                        raise
                finally:
                    row['wall_s']=perf_counter()-started
                    if client:
                        try:
                            if not evidence_saved:
                                if c and getattr(c,'_single_active_row',None) and not (folder/'attempt.json').exists():
                                    failed_detail=deepcopy(c._single_active_row)
                                    failed_detail.update(complete=False,request_exception=row.get('failure'))
                                    save(folder/'attempt.json',failed_detail)
                                save(folder/'requests.json',client.records)
                                save(folder/'search-trace.json',dict(model=client.trace_inputs.get('model'),attempts=client.traces))
                                if c:save(folder/'connection-attempts.json',c.connection_attempts)
                            row['prior_calls']=sum('prior' in str(r.get('op')) or r.get('pipeline_id')=='static_prior' for r in client.records)
                        finally:client.close()
                    save(folder/'summary.json',row);save(out/'result.json',result('REQUEST_STOPPED' if fatal else 'RUNNING'))
            if accepted:break
        if not accepted:break
    final=result('COMPLETE_PLANNING_ONLY' if not remaining() else 'INCOMPLETE_RETAINED')
    save(out/'result.json',final);return final


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--worker-command',required=True)
    p.add_argument('--worker-binary',type=Path,required=True);p.add_argument('--source-commit',required=True)
    p.add_argument('--source-manifest',type=Path);p.add_argument('--seeds',type=int,nargs='+',default=[71070,71071])
    p.add_argument('--task-budget-s',type=float,default=300.);p.add_argument('--stage-budget-s',type=float,default=90.)
    p.add_argument('--ipc-timeout-s',type=float,default=100.);p.add_argument('--development',action='store_true')
    p.add_argument('--direct-first',action=argparse.BooleanOptionalAction,default=True)
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
        command=[sys.executable,*sys.argv],pair_profile_enabled=False,task_budget_s=args.task_budget_s,stage_budget_s=args.stage_budget_s,
        approach_recovery_mode='shared_budget',ompl_attempt_quota=None,direct_first=args.direct_first,
        connection_slice_s=6.,native_rrt_request_slice_s=2.,direct_time_share_per_method=.10,
        approach_quality_policy='same_pose_ik_comparison_and_native_rrt_incumbent_v2',
        initial_ik_comparison_max_s=.30,quality_search_max_s=4.,quality_remaining_share=.10,
        search_operation='RESTART',tree_retained=False))
    rows=[]
    for seed in args.seeds:
        out=args.output/f'seed-{seed}';out.mkdir();rows.append(run_seed(args,original,built.connector,frozen,seed,out))
        save(args.output/'summary.json',dict(**source,runs=rows,qualification_status='NOT_EVALUATED',continuous_swept_proof=False,
            physical_execution='NOT_RUN',execution_ready=False))
    print(json.dumps(dict(completed=[r['completed_count'] for r in rows],targets=len(frozen.target_ids))),flush=True)

if __name__=='__main__':main()
