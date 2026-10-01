"""Actual native/authority differential checks; no task search or execution."""
import argparse
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.moveit2_backend import MoveItLayoutConnector


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True);started=perf_counter()
    policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
    scene=build_verified_motion_input(policy,ROOT)
    core=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot()).connector
    c=MoveItLayoutConnector.from_existing(core,scene);c.start_planning_request()
    old=json.loads((ROOT/'tests/fixtures/moveit2/frozen_candidate.json').read_text())['selected_trajectory_segment']
    historical=json.loads((ROOT/'tests/fixtures/moveit2/clearance_rejections.json').read_text())
    assert historical['scene_fingerprint']==scene.snapshot['scene_fingerprint']
    target=next(b for b in scene.cartons if b.name==old['target'])
    c.stack_carton_names={b.name for b in scene.cartons};c.robot_state_validator.stack_carton_names=frozenset(c.stack_carton_names)
    c.robot_state_validator.contact_target_name=target.name
    q_grasp=np.asarray(old['path'][old['grasp_index']]);physical=c.physical_from_virtual(c.robot.fk(q_grasp))
    attachment=PhysicalContactAttachment(c.robot,RigidAttachment.capture(physical,target),c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
    obstacles=[b for b in scene.all_obstacles if b.name!=target.name]
    request=c._build_native_request(historical['q_start'],historical['q_goal'],obstacles,seed=71072,attachment=attachment,stage='transit')
    report=dict(schema='m710_clearance_differential_v1',startup=c.native_startup,request=request,
        source=historical,attachment_provenance='reconstructed by original fixture grasp FK and RigidAttachment.capture; original raw request was not retained',
        endpoints=[],historical_states=[],isaac='NOT_RUN')
    try:
        for label in ('q_start','q_goal'):
            q=np.asarray(historical[label]);native=c.native.request({**request,'op':'validate','q_start':q.tolist()})
            t=perf_counter();failure=c._state_failure(q,obstacles,attachment=attachment,stage='transit')
            report['endpoints'].append(dict(endpoint=label,q=q.tolist(),native=native,authority=failure,authority_s=perf_counter()-t))
        for case in historical['cases']:
            q=np.asarray(case['original_failure']['q_rad']);edge=np.asarray(case['edge']);fraction=case['original_failure']['fraction']
            assert np.array_equal(edge[0]+fraction*(edge[1]-edge[0]),q)
            native=c.native.request({**request,'op':'validate','q_start':q.tolist(),'probe_path':edge.tolist()})
            t=perf_counter();failure=c._state_failure(q,obstacles,attachment=attachment,stage='transit');state_s=perf_counter()-t
            t=perf_counter();edge_failure=c._path_failure(edge,obstacles,attachment=attachment,stage='transit');edge_s=perf_counter()-t
            report['historical_states'].append(dict(original=case,native=native,authority=failure,edge_authority=edge_failure,
                authority_state_s=state_s,authority_edge_s=edge_s,
                distance_difference_m=None if not failure or not native['failure'] else native['failure'].get('surface_distance_m',0)-failure.get('surface_distance_m',0)))
            args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
            assert native['legacy_intersection_valid'] and not native['native_valid'] and failure
            assert native['path_failure'] and edge_failure
            assert set(native['failure']['pair'])==set(failure['pair'])
        # Same real context: invalid endpoints must not reach any planner.
        bad=historical['cases'][0]['original_failure']['q_rad']
        invalid_goal=c.native.request({**request,'op':'plan','pipeline_id':'pilz_industrial_motion_planner',
            'planner_id':'PTP','q_goal':bad})
        invalid_start=c.native.request({**request,'op':'plan','pipeline_id':'ompl',
            'planner_id':'RRTConnectkConfigDefault','q_start':bad})
        assert invalid_goal['status']=='INVALID_GOAL_CLEARANCE' and invalid_start['status']=='INVALID_START_CLEARANCE'
        assert invalid_goal['pipeline_calls']=={} and invalid_start['pipeline_calls']=={}
        report['invalid_endpoints_without_planning']=dict(goal=invalid_goal,start=invalid_start)
        _,failure,evidence=c._native_plan(np.asarray(historical['q_start']),np.asarray(bad),obstacles,
            seed=71072,attachment=attachment,stage='transit')
        assert failure['reason']=='INVALID_GOAL_AUTHORITY'
        report['python_invalid_goal']=dict(failure=failure,evidence=evidence)
        # Diagnostic-only world perturbation: rebuild a new diff then return to
        # the exact original context. Never use this modified world for planning.
        from copy import deepcopy
        from unloading_sim.moveit2_backend import digest
        changed=deepcopy(request['world']);pair=historical['cases'][0]['original_failure']['pair']
        neighbor=next(b for b in changed if b['id']==pair[1])
        neighbor['pose'][2][3]+=10.
        moved=c.native.request({**request,'op':'validate','q_start':bad,'world':changed,'scene_fingerprint':digest(changed)})
        original=c.native.request({**request,'op':'validate','q_start':bad})
        detached=c.native.request({**request,'op':'validate','q_start':bad,'attachment':None})
        assert moved['native_valid'] and not original['native_valid']
        assert original['attached_id']==target.name and detached['attached_id']==''
        assert original['world_count']+1==detached['world_count']
        report['synthetic_context_switch']=dict(scope='diagnostic world perturbation only, not planning evidence',
            moved_neighbor=moved,restored_original=original,detached=detached)
        report['status']='PASS'
    finally:
        c.native.close();report['end_to_end_s']=perf_counter()-started
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('Historical state/edge differential:',report['status'],report['end_to_end_s'])


if __name__=='__main__':main()
