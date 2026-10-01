"""Real fixed suffix extraction and directed resident Pilz TCP semantics checks."""
from __future__ import annotations
import argparse
from copy import copy
import json
from pathlib import Path
import sys
from time import perf_counter
import traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.moveit2_backend import MoveItLayoutConnector,model_request,validate_native_result,TASK_TCP_LINK,digest
from unloading_sim.moveit2_tcp import audit_linear_tcp
from unloading_sim.geometry import rotation_matrix_from_rotation_vector
from unloading_sim.history_candidates import compatible_hint,HistoryPolicy
from unloading_sim.history_adaptation import solve_local


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--suite',choices=['suffix','semantics'],required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    start=perf_counter();args.output.parent.mkdir(parents=True,exist_ok=True)
    out=dict(schema='m710_rotating_tcp_v1',suite=args.suite,results=[],isaac='NOT_RUN',native_calls=[])
    def save(): args.output.write_text(json.dumps(out,indent=2,allow_nan=False),encoding='utf-8')
    c=None
    try:
        policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
        scene=build_verified_motion_input(policy,ROOT)
        fixture=json.loads((ROOT/'tests/fixtures/moveit2/frozen_candidate.json').read_text())
        assert fixture['scene_fingerprint']==scene.snapshot['scene_fingerprint']
        old=fixture['selected_trajectory_segment'];built=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
        if built.connector is None: raise RuntimeError(built.failure_reason)
        c=MoveItLayoutConnector.from_existing(built.connector,scene);c.start_planning_request()
        from unloading_sim.conveyor_placement import PlacementPolicy
        c.placement_policy=PlacementPolicy(edge_tolerance_m=float(policy.data['search_strategy']['conveyor_footprint_boundary_tolerance_m']))
        c.stack_carton_names={b.name for b in scene.cartons};c.robot_state_validator.stack_carton_names=frozenset(c.stack_carton_names)
        target=next(b for b in scene.cartons if b.name==old['target']);c.robot_state_validator.contact_target_name=target.name
        obstacles=[b for b in scene.all_obstacles if b.name!=target.name]
        out.update(startup=c.native_startup,fk=c.native_fk,scene_fingerprint=scene.snapshot['scene_fingerprint'],
            policy_fingerprint=policy.policy_fingerprint,cartons=len(scene.cartons),target=target.name,
            tool_shapes=len(c.native_tools),setup_s=perf_counter()-start)
        native_request=c.native.request
        def checkpoint(payload,**kwargs):
            raw=native_request(payload,**kwargs)
            if payload['op']=='plan':
                with args.output.with_suffix('.native.jsonl').open('a',encoding='utf-8') as f:
                    f.write(json.dumps(dict(request=payload,response=raw,authority_status='NOT_YET_CHECKED'),allow_nan=False)+'\n')
                print(raw['status'],raw.get('failure_comments'),raw.get('mtc_plan_s'),flush=True)
            return raw
        c.native.request=checkpoint
        if args.suite=='suffix':
            hint=compatible_hint(fixture,scene,c,target,built.evidence,HistoryPolicy())
            hint['attempt_provenance']={'candidate_id':'71070000fixed_candidate','source':'frozen_candidate.json'}
            contact=solve_local(c,np.asarray(old['contact']['requested_virtual_task_tcp_pose_world']),np.asarray(old['path'][old['grasp_index']]))
            if not contact.success: raise RuntimeError('CONTACT_IK_FAILED:'+contact.message)
            # Extraction must never redo the long prefix checks.
            state_check,path_check=c._state_failure,c._path_failure
            def forbidden(*a,**k): raise AssertionError('EXTRACTION_RAN_HEAVY_PREFIX_CHECK')
            c._state_failure=c._path_failure=forbidden
            requests=c.history_linear_suffix_requests(hint,target,contact.q,scene.all_obstacles)
            early=c.history_capability_check(hint,target,contact.q,scene.all_obstacles)
            c._state_failure,c._path_failure=state_check,path_check
            out['extraction_heavy_checks']=0;out['early_capability']=early;out['suffix_fixtures']=[]
            for r in requests:
                request=c._build_native_request(r['q_start'],None,obstacles,seed=71070,attachment=r['attachment'],stage='transit',goal_pose=r['goal_pose'])
                info={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in r.items() if k!='attachment'}
                info.update(native_request=request,start_pose_world=c.robot.fk(r['q_start']).tolist(),reference_frame='world',
                    flange_from_physical_contact=c.flange_from_physical_contact.tolist(),
                    physical_contact_from_box=r['attachment'].rigid.tcp_from_box.tolist())
                out['suffix_fixtures'].append(info)
            save()
            # One process intent, fixed first existing release alternative. No seed scan.
            r=requests[0];t=perf_counter()
            path,failure,evidence=c._native_plan(r['q_start'],None,obstacles,seed=71070,
                attachment=r['attachment'],stage='transit',goal_pose=r['goal_pose'])
            out['results'].append(dict(case='real_fixed_rotating_suffix',path=[q.tolist() for q in path],failure=failure,evidence=evidence,end_to_end_s=perf_counter()-t))
        else:
            q=policy.layout_validation.initial_q.copy()
            # The fixed full scene and physical tool remain unchanged in all cases.
            for name,translate,rotate in [('constant_orientation',[0,0,.002],[0,0,0]),
                ('offset_pure_rotation',[0,0,0],[0,.01,0]),('offset_translation_rotation',[0,0,.002],[0,.01,0])]:
                pose=c.robot.fk(q);pose[:3,3]+=translate;pose[:3,:3]=rotation_matrix_from_rotation_vector(np.asarray(rotate))@pose[:3,:3]
                t=perf_counter();path,failure,evidence=c._native_plan(q,None,scene.all_obstacles,seed=71070,stage='pregrasp',goal_pose=pose)
                out['results'].append(dict(case=name,failure=failure,evidence=evidence,path=[x.tolist() for x in path],end_to_end_s=perf_counter()-t));save()
            # A second collisionless task frame in the SAME process; never alter physical tool geometry.
            alternate=copy(c);alternate.flange_from_virtual_task_tcp=c.flange_from_virtual_task_tcp.copy();alternate.flange_from_virtual_task_tcp[:3,3]=0
            import os
            init,_,_=model_request(alternate,scene,asset_root=os.environ.get('M710_MOVEIT_ASSET_ROOT'))
            t=perf_counter();startup=c.native.request(init);alternate.native_identity=init['identity']
            fk=lambda values:c.robot.named_link_frames(values)['flange']@alternate.flange_from_virtual_task_tcp
            pose=fk(q);pose[:3,:3]=rotation_matrix_from_rotation_vector(np.array([0,.01,0]))@pose[:3,:3]
            req=alternate._build_native_request(q,None,scene.all_obstacles,seed=71070,stage='pregrasp',goal_pose=pose)
            req.update(pipeline_id='pilz_industrial_motion_planner',planner_id='LIN')
            raw=c.native.request(req,timeout=600);audit=None;authority=None
            if raw['status']=='SUCCESS':
                path=validate_native_result(raw,q,None,c.native_joint_names)
                audit=audit_linear_tcp(path,fk,fk(q),pose,position_tolerance=c.ik['position_tolerance_m'],orientation_tolerance=c.ik['orientation_tolerance_rad'],edge_resolution_rad=c.budget.edge_resolution_rad)
                authority=c._path_failure(path,scene.all_obstacles,stage='pregrasp')
            out['results'].append(dict(case='zero_translation_offset_rotation_second_context',startup=startup,native=raw,audit=audit,authority_failure=authority,end_to_end_s=perf_counter()-t))
            first_init,_,_=model_request(c,scene,asset_root=os.environ.get('M710_MOVEIT_ASSET_ROOT'))
            out['reused_first_context']=c.native.request(first_init);c.check_fk();out['reused_fk']=c.native_fk
            # Unsupported changed TCP must exit before any heavy historical prefix work.
            bound=c.flange_from_virtual_task_tcp.copy();c.flange_from_virtual_task_tcp=alternate.flange_from_virtual_task_tcp.copy()
            out['unsupported_context']=c.capability_check(c.robot.fk(q),c.robot.fk(q),stage='transit',location='before_heavy_checks')
            c.flange_from_virtual_task_tcp=bound
        out['status']='COMPLETED'
    except Exception as e:
        out.update(status='ERROR',error=str(e),traceback=traceback.format_exc());print(out['traceback'],flush=True)
    finally:
        if c is not None: out['native_calls']=c.native_evidence;c.native.close()
        out['end_to_end_s']=perf_counter()-start;save()
    return int(out['status']=='ERROR')

if __name__=='__main__':raise SystemExit(main())
