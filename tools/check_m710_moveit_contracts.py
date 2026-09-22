"""Real native scene isolation and fail-closed checks, without execution."""
import json
import os
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.moveit2_backend import MoveItLayoutConnector,box_message,digest,MoveItUnavailable


def main():
    output=Path(sys.argv[1]);output.parent.mkdir(parents=True,exist_ok=True)
    policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
    scene=build_verified_motion_input(policy,ROOT)
    built=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
    c=MoveItLayoutConnector.from_existing(built.connector,scene)
    target=scene.cartons[0];neighbor=scene.cartons[1]
    q=policy.layout_validation.initial_q
    local=np.linalg.inv(c.robot.named_link_frames(q)['flange'])@target.world_from_local
    attached=box_message(target,local);attached['touch_links']=sorted(c.native_compliant)
    # Ownership/count inspection only: this artificial home attachment is NOT
    # claimed as a checked motion, grasp or physical evidence.
    pair=[sorted(c.native_compliant)[0],neighbor.name]
    world=[box_message(b) for b in scene.all_obstacles]
    request=dict(op='inspect',identity=c.native_identity,q_start=q.tolist(),world=world,scene_fingerprint=digest(world),
        allowed_pairs=[pair],attachment=attached,start_velocity=[0.]*6,path_constraints={},inspect_pairs=[pair])
    first=c.native.request(request)
    second=c.native.request({**request,'attachment':None,'allowed_pairs':[]})
    assert first['attached_count']==1 and first['world_count']==len(world)-1 and not first['target_in_world']
    assert first['permissions']==[True] and first['base_attached_count']==0
    assert second['attached_count']==0 and second['world_count']==len(world) and second['permissions']==[False]
    result=dict(schema='m710_native_contract_checks_v1',scope='scene bookkeeping only; artificial attachment is not a planning success',
        attached_diff=first,subsequent_candidate=second,fk=c.native_fk,startup=c.native_startup)
    # A zero-motion synthetic bookkeeping task exercises the real custom MTC
    # stages and attach/release transitions, not full-task motion acceptance.
    stationary=dict(path=[q.tolist(),q.tolist()],durations=[.01],generator='synthetic_bookkeeping_only')
    composition=c.native.request({**request,'op':'compose','attachment':None,
        'allowed_pairs':[['base_link',c.robot_state_validator.base_support_obstacle_name]],
        'stages':[{**stationary,'name':'contact','attach':attached},
                  {**stationary,'name':'place','release':box_message(target)},
                  {**stationary,'name':'withdrawal'}]})
    assert composition['status']=='SUCCESS' and composition['stage_count']==3
    assert composition['attached_objects']==[] and composition['world_count']==len(world)
    result['synthetic_mtc_state_transitions']=composition
    after=c.native.request({**request,'attachment':None,'allowed_pairs':[]})
    assert after['attached_count']==0 and after['world_count']==len(world) and after['permissions']==[False]
    result['after_composition']=after
    # Protocol error terminates this client: no late result can be reused.
    invalid=q.copy();invalid[0]=1e6
    try:c.native.request({**request,'q_start':invalid.tolist()})
    except MoveItUnavailable as exc:
        result['invalid_start']=str(exc)
        assert 'INVALID_START_BOUNDS' in str(exc)
        assert c.native.process.poll() is not None
    else:raise AssertionError('invalid start accepted')
    c.native.close();result['status']='PASS'
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('native scene isolation, FK, invalid start: PASS')


if __name__=='__main__':main()
