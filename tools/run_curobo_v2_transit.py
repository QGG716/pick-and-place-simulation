"""Run one fixed business TRANSIT request through the baseline or isolated V2.

Historical motion supplies endpoints/attachment only. Every state and edge is
rechecked under the current scene and policy. This tool never launches Isaac.
"""
import argparse
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
from time import perf_counter
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import (load_layout_motion_policy,build_verified_motion_input,
                                               _build_automatic_trajectory_connector)
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowSequencePolicy,RowUnloadingState
from unloading_sim.stage_export import export_request,file_hash
from unloading_sim.stage_backend import run_stage,StageRequest,fingerprint


def write(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8')


def context(motion_path,state_path,config):
    motion=json.loads(Path(motion_path).read_text())
    policy=load_layout_motion_policy(config)
    # Historical paths are input hints, never permission to restore old safety.
    # Only the unused history-directory locator may differ from current config.
    historical_policy=deepcopy(motion['effective_motion_policy'])
    historical_policy['search_strategy']['history']['source']=policy.data['search_strategy']['history']['source']
    if historical_policy!=policy.data:
        raise ValueError('fixture policy differs from current active configuration')
    policy=replace(policy,data=motion['effective_motion_policy'])
    scene=build_verified_motion_input(policy)
    if state_path:
        state=json.loads(Path(state_path).read_text())
        rows=RowUnloadingState(RowSequencePolicy(row_height_fraction=.05))
        rows.rank(scene.cartons,support_graph=scene.support_graph)
        scene=apply_actual_motion_state(scene,state,row_state=rows)
    if scene.snapshot['scene_fingerprint']!=motion['scene_fingerprint']:
        raise ValueError('historical endpoints do not belong to current frozen scene')
    robot=policy.layout_validation.layout.robot()
    built=_build_automatic_trajectory_connector(scene,robot)
    if built.connector is None:
        raise RuntimeError(str(built))
    connector=built.connector
    segment=motion['selected_trajectory_segment']
    q=np.asarray(segment['path'],float)
    target=next(x for x in scene.cartons if x.name==segment['target'])
    rigid=RigidAttachment(np.asarray(segment['contact']['physical_contact_from_box']),target.half_extents,target.name)
    attached=PhysicalContactAttachment(robot,rigid,connector.flange_from_virtual_task_tcp,connector.flange_from_physical_contact)
    if not np.allclose(attached.box_at(q[segment['grasp_index']]).world_from_local,target.world_from_local,atol=1e-7):
        raise ValueError('historical attachment is not pose continuous with current target')
    a,b=segment['stage_ranges']['transit']
    if a!=segment['stage_ranges']['extraction'][1] or b!=segment['stage_ranges']['place'][0]:
        raise ValueError('business TRANSIT boundary mismatch')
    selection=segment['contact']['cup_selection']
    connector.robot_state_validator.commanded_cup_mask=selection['commanded_active_mask']
    connector.robot_state_validator.stack_carton_names={x.name for x in scene.cartons}
    connector.robot_state_validator.contact_target_name=target.name
    return motion,scene,connector,attached,q[a:b+1],target


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--motion',required=True);p.add_argument('--state')
    p.add_argument('--config',default='configs/validation/m710id70_handoff_continuation.yaml')
    p.add_argument('--output',required=True);p.add_argument('--backend',choices=['baseline','curobo_v2'],default='curobo_v2')
    p.add_argument('--fixture',choices=['business','direct_unit'],default='business')
    p.add_argument('--endpoints-only',action='store_true')
    p.add_argument('--gpu-python');p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--warm-runs',type=int,default=1)
    args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    entered=perf_counter()
    motion,scene,connector,attached,historical,target=context(args.motion,args.state,args.config)
    if args.fixture=='direct_unit':
        start=historical[-1].copy();goal=start.copy();goal[2]+=.001
        historical=np.array([start,goal])
    request,bundle=export_request(scene,connector,attached,historical[0],historical[-1],request_id=target.name+'-transit')
    write(out/'request.json',request.to_dict());write(out/'bundle.json',bundle)
    write(out/'provenance.json',dict(source_motion=str(args.motion),source_motion_sha256=file_hash(args.motion),
          source_state=args.state,source_state_sha256=None if args.state is None else file_hash(args.state),
          fixture_kind='business_full_TRANSIT' if args.fixture=='business' else 'synthetic_direct_1mrad_J3_at_preplace', historical_success_inherited=False,
          current_source_hashes={str(x.relative_to(ROOT)):file_hash(x) for x in
            [ROOT/'src/unloading_sim/layout_trajectory.py',ROOT/'src/unloading_sim/pinocchio_backend.py',ROOT/'src/unloading_sim/collision_policy.py']}))
    if args.prepare_only:
        print('PREPARED',out);return
    obstacles=[x for x in scene.all_obstacles if x.name!=target.name]
    state_check=lambda q:connector._state_failure(np.asarray(q,float),obstacles,attachment=attached,stage='transit')
    authority=lambda trajectory:connector._path_failure(trajectory['q'],obstacles,attachment=attached,stage='transit')
    revision=lambda:(request.data['scene_revision'],request.data['scene_fingerprint'])
    process=None
    try:
        if args.backend=='curobo_v2':
            if not args.gpu_python: p.error('--gpu-python required')
            log=(out/'gpu_worker.log').open('w')
            process=subprocess.Popen([args.gpu_python,'-m','unloading_sim.curobo_v2_backend',
                 str((out/'bundle.json').resolve()),str((out/'sphere_cache').resolve())],
                 stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True,bufsize=1)
            native_call_count=0
            def call(command):
                nonlocal native_call_count
                t=perf_counter();process.stdin.write(json.dumps(command)+'\n');process.stdin.flush()
                line=process.stdout.readline()
                if not line: return dict(status='DEPENDENCY_UNAVAILABLE',trajectory=None,error='worker exited; see gpu_worker.log')
                result=json.loads(line);result['ipc_inclusive_s']=perf_counter()-t
                if command.get('op')=='solve':
                    write(out/f'native_candidate_{native_call_count:03d}.json',result)
                    native_call_count+=1
                return result
            # A few fixed states, including an independent J3 perturbation.
            states=[request.data['q_start'],request.data['q_goal'],(historical[0]+np.array([0,0,.01,0,0,0])).tolist()]
            fk=call(dict(op='fk',states=states));write(out/'gpu_model.json',fk)
            if 'fk' in fk:
                errors=[]
                base_inv=np.linalg.inv(connector.robot.base_transform)
                for i,q in enumerate(states):
                    frames=connector.robot.named_link_frames(np.asarray(q))
                    frames['held_carton']=attached.box_at(q).world_from_local
                    for name in ('flange','fanuc_flange','tool0','held_carton'):
                        from unloading_sim.stage_backend import pose_wxyz
                        expected=np.asarray(pose_wxyz(base_inv@frames[name]))
                        got=fk['fk'][name]
                        pos=np.asarray(got['position']).reshape(-1,3)[i]
                        quat=np.asarray(got['quaternion_wxyz']).reshape(-1,4)[i]
                        errors.append(dict(state=i,frame=name,position_m=float(np.max(np.abs(pos-expected[:3]))),
                                           quaternion_error=float(min(np.linalg.norm(quat-expected[3:]),np.linalg.norm(quat+expected[3:])))))
                ok=all(x['position_m']<2e-5 and x['quaternion_error']<2e-5 for x in errors)
                ok=ok and all(x.get('passed',True) for x in fk['model_audit']['inertials'].values())
                write(out/'fk_consistency.json',dict(passed=ok,errors=errors))
                if not ok:
                    write(out/'result.json',dict(status='MODEL_MISMATCH',candidate_generated=False,authority_accepted=False,delivered=False,isaac_execution_completed=False));return
            else:
                write(out/'result.json',fk);return
            if args.endpoints_only:
                native=fk.get('native_endpoints',{})
                write(out/'endpoint_result.json',dict(native=native,
                    authority=[state_check(q) for q in states],stage='TRANSIT',no_planning_attempt=True))
                return
            candidate=lambda req,attempt:call(dict(op='solve',attempt=attempt))
        else:
            # Rerun the original explicit fixed-endpoint RRT baseline; never copy
            # historical success. Its own path checks plus common authority apply.
            def candidate(req,attempt):
                t=perf_counter()
                path,failure,evidence=connector._transit(np.asarray(req.data['q_start']),np.asarray(req.data['q_goal']),
                    obstacles,seed=req.data['seed']+attempt,iteration_budget=600,attachment=attached,stage='transit')
                trajectory=None
                if path:
                    arr=np.asarray(path)
                    # This is a comparison interpolation, not a replay delivery.
                    dt=np.maximum(.025,np.max(np.abs(np.diff(arr,axis=0))/np.asarray(req.data['limits']['velocity']),axis=1))
                    trajectory=dict(q=arr.tolist(),time_s=np.r_[0,np.cumsum(dt)].tolist(),joint_names=req.data['joint_names'],
                                    valid_length=len(arr),interpolation='linear_joint_samples',dq=None,ddq=None)
                return dict(backend=dict(name='existing_rrt_connect',version=None,commit=None),status='CANDIDATE_GENERATED' if path else 'BACKEND_NO_CANDIDATE',
                            trajectory=trajectory,error=failure,evidence=evidence,timings=dict(native_total_s=perf_counter()-t))
        results=[]
        for run in range(1+args.warm_runs):
            result=run_stage(request,candidate,authority,state_check,revision)
            result['run_kind']='cold' if run==0 else 'warm'
            result['timings']['request_entry_to_result_s']=perf_counter()-entered if run==0 else result['timings']['end_to_end_s']
            result['timings']['handoff_s']=None
            result['timings']['timing_scope']='result only; no executable delivery when rejected'
            results.append(result);write(out/'results.json',results);write(out/'result.json',result)
            print(run,result['status'],result['timings'],flush=True)
            if result['status'] in ('START_INVALID','GOAL_INVALID','DEPENDENCY_UNAVAILABLE','MODEL_MISMATCH'):
                break
    finally:
        if process:
            process.stdin.close();process.wait();log.close()

if __name__=='__main__': main()
