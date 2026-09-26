"""Small reproducible real-layout backend experiment; no population sweep."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from time import perf_counter
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy, build_verified_motion_input, _build_automatic_trajectory_connector
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.moveit2_backend import MoveItLayoutConnector


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--suite',choices=['stages','task','capability'],default='stages')
    p.add_argument('--backend',choices=['core','moveit2'],default='moveit2')
    p.add_argument('--case', action='append', help='Run only named directed motion cases')
    p.add_argument('--config',default='configs/validation/m710id70_proof_of_concept.yaml')
    p.add_argument('--fixture',default='tests/fixtures/moveit2/frozen_candidate.json')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();started=perf_counter();args.output.parent.mkdir(parents=True,exist_ok=True)
    report=dict(schema='m710_moveit_experiment_v1',suite=args.suite,backend=args.backend,results=[],isaac='NOT_RUN')
    c=None
    def save(): args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    try:
        policy=load_layout_motion_policy(ROOT/args.config)
        scene=build_verified_motion_input(policy,ROOT)
        fixture=json.loads((ROOT/args.fixture).read_text())
        if fixture['scene_fingerprint']!=scene.snapshot['scene_fingerprint']:
            raise ValueError('FIXTURE_SCENE_MISMATCH')
        old=fixture['selected_trajectory_segment'];path=np.asarray(old['path'])
        built=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
        if built.connector is None: raise RuntimeError(built.failure_reason)
        c=built.connector
        if args.backend=='moveit2': c=MoveItLayoutConnector.from_existing(c,scene)
        from unloading_sim.conveyor_placement import PlacementPolicy
        strategy=policy.data["search_strategy"]
        c.placement_policy=PlacementPolicy(maximum_candidates=int(strategy["placement_candidates"]),
            coarse_samples_per_axis=int(strategy["coarse_place_samples_per_axis"]),fine_samples_per_axis=int(strategy["fine_place_samples_per_axis"]),
            contact_tolerance_m=float(policy.data["state_validity"]["contact_tolerance_m"]),
            edge_tolerance_m=float(strategy["conveyor_footprint_boundary_tolerance_m"]),
            occupancy_clearance_m=c.collision_policy.pair_clearance("external",c.collision_margin_m),
            normal_tolerance_rad=np.deg2rad(strategy["placement_normal_tolerance_deg"]),
            process_family_by_support=dict(strategy["surface_process_families"]),
            allowed_families_by_process={k:tuple(v) for k,v in strategy["allowed_placement_families"].items()},
            overlap_process_priority=tuple(strategy["overlap_process_priority"]))
        c.start_planning_request()
        c.stack_carton_names={b.name for b in scene.cartons}
        c.robot_state_validator.stack_carton_names=frozenset(c.stack_carton_names)
        target=next(b for b in scene.cartons if b.name==old['target'])
        c.robot_state_validator.contact_target_name=target.name
        physical=c.physical_from_virtual(c.robot.fk(path[old['grasp_index']]))
        attachment=PhysicalContactAttachment(c.robot,RigidAttachment.capture(physical,target),
            c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
        payload_obstacles=[b for b in scene.all_obstacles if b.name!=target.name]
        report.update(scene_fingerprint=scene.snapshot['scene_fingerprint'],policy_fingerprint=policy.policy_fingerprint,
            obstacle_count=len(scene.all_obstacles),cartons=len(scene.cartons),fixture_provenance=fixture['provenance'],
            setup_s=perf_counter()-started,seed=71070,effective_policy=c.collision_policy.to_mapping())
        if args.backend=='moveit2':report.update(startup=c.native_startup,fk=c.native_fk)
        if args.suite=='stages':
            q=policy.layout_validation.initial_q.copy();goal=q.copy();goal[0]+=.01
            a,b=old['stage_ranges']['pregrasp'];ta,tb=old['stage_ranges']['transit']
            cases=[('empty_short',q,goal,scene.all_obstacles,None,None,'real_layout_small_perturbation'),
                ('empty_fixed_approach',path[a],path[b],scene.all_obstacles,None,None,'frozen_candidate_endpoints'),
                ('loaded_fixed_transit',path[ta],path[tb],payload_obstacles,attachment,None,'frozen_candidate_endpoints')]
            lin=c.robot.fk(q);lin[2,3]+=.002
            cases.append(('linear_fixed_orientation',q,None,scene.all_obstacles,None,lin,'real_layout_small_translation'))
            for index,(name,q0,q1,obstacles,attached,pose,kind) in enumerate(cases):
                if args.case and name not in args.case: continue
                case_started=perf_counter()
                if pose is None:
                    values=c._transit(q0,q1,obstacles,seed=71070+index,iteration_budget=600,
                        attachment=attached,stage='transit' if attached else 'pregrasp')
                else:
                    values=c._cartesian(q0,pose,obstacles,seed=71070+index,stage='pregrasp')
                result,failure,evidence=values
                row=dict(case=name,fixture_kind=kind,q_start=q0.tolist(),q_goal=None if q1 is None else q1.tolist(),
                    goal_pose=None if pose is None else pose.tolist(),path=[q.tolist() for q in result],failure=failure,
                    authoritative_status='PASS' if failure is None else 'FAIL',evidence=evidence,end_to_end_s=perf_counter()-case_started)
                report['results'].append(row);save();print(name,row['authoritative_status'],row['end_to_end_s'],flush=True)
        else:
            from unloading_sim.history_candidates import compatible_hint, HistoryPolicy
            hint=compatible_hint(fixture,scene,c,target,built.evidence,HistoryPolicy())
            hint['attempt_provenance']={'candidate_id':'71070000fixed_candidate','source':'frozen_candidate.json'}
            if args.suite=='capability':
                if args.backend!='moveit2': raise ValueError('capability suite requires moveit2')
                def forbidden(*a,**k): raise AssertionError('EARLY_CAPABILITY_MISSED_HEAVY_CHECK')
                c._path_failure=forbidden;c._state_failure=forbidden
            with c._contact_context():
                task_started=perf_counter()
                result=c.plan(target=target,face=old['face'],requested_virtual_contact=np.asarray(old['contact']['requested_virtual_task_tcp_pose_world']),
                    grasp_candidates=[dict(q_rad=path[old['grasp_index']].tolist(),candidate_id='fixed_candidate')],
                    home_q=policy.layout_validation.initial_q,all_obstacles=scene.all_obstacles,receiver=scene.receiver,
                    support_names=tuple(scene.support_graph.supported_by[target.name]),suction=policy.data['suction'],seed=71070,
                    history_hint=hint)
            report['results'].append(dict(success=result.success,failure=result.failure,segment=result.segment,
                attempts=result.attempts,statistics=result.statistics,end_to_end_s=perf_counter()-task_started,
                scope='one fixed historical candidate, current process reconstruction and authoritative checks; no new candidate sweep'))
        if args.suite=='capability':
            assert not result.success and result.failure['reason']=='UNSUPPORTED_ROTATING_TASK_TCP_LIN'
            assert c.native_evidence==[]
            report['heavy_state_or_edge_checks']=0
            report['capability_checks']=c.capability_evidence
        report['status']='COMPLETED'
    except Exception as exc:
        report.update(status='ERROR',error=str(exc),traceback=traceback.format_exc());print(report['traceback'],flush=True)
    finally:
        if isinstance(c,MoveItLayoutConnector):
            report['native_calls']=c.native_evidence;c.native.close()
        report['end_to_end_s']=perf_counter()-started;save()
    return 1 if report['status']=='ERROR' else 0


if __name__=='__main__':raise SystemExit(main())
