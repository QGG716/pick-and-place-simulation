"""20-50 mm release policy, production gates and exact receiver-top ownership."""
from dataclasses import replace
from copy import deepcopy
import numpy as np
import pytest

from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.release_motion import (ReleasePolicy, IDEAL_RECEPTION_RELEASE, ideal_reception_region,
    predict_release, verify_release_prediction, release_flight_envelope)
from unloading_sim.isaac_collision_policy import classify_poc_runtime_pair, declared_receiver_top_contact
from unloading_sim.m710_replay_physics import audit_runtime_short_drop
from unloading_sim.post_landing_transport import begin_ideal_transport, advance_ideal_transport
from unloading_sim.layout_single_carton import (load_layout_motion_policy, build_verified_motion_input,
    _build_automatic_trajectory_connector)
from unloading_sim.planning_profile import DEFAULT_MOTION
from test_adaptive_release_motion import deck, carton
from test_poc_pair_clearance import policy
from test_post_landing_transport import POLICY
from ideal_handoff_fixture import measured_handoff


@pytest.mark.parametrize("height,accepted", [(0,False),(.010,False),(.015,False),(.019999,False),
    (.020,True),(.025,True),(.035,True),(.050,True),(.050001,False)])
def test_lowest_corner_release_boundaries_and_recorded_export_validation(height, accepted):
    box=carton(height, rotation_matrix_from_rpy(0,np.pi/2,0))
    prediction=predict_release(box,[deck()],mode=IDEAL_RECEPTION_RELEASE)
    assert prediction['height_m']==pytest.approx(height)
    assert prediction['accepted'] is accepted
    assert prediction['actual_support'] is None
    if accepted:
        assert verify_release_prediction(dict(actual_box_pose_world=box.world_from_local.tolist(),
            release_prediction=prediction),box.name)['accepted']
    else:
        with pytest.raises(ValueError):
            verify_release_prediction(dict(actual_box_pose_world=box.world_from_local.tolist(),
                release_prediction=prediction),box.name)


def test_height_is_not_bottom_face_coplanarity_but_tilt_and_coverage_remain():
    tilted=carton(.040,rotation_matrix_from_rpy(0,np.deg2rad(4),0))
    assert tilted.corners()[:,2].max()>.650
    assert ideal_reception_region(tilted,[deck()])['accepted']
    assert not ideal_reception_region(carton(.03,rotation_matrix_from_rpy(0,np.deg2rad(6),0)),[deck()])['accepted']
    outside=OBB([1.1,0,.78],[.3,.2,.15],np.eye(3),'box','carton')
    assert not ideal_reception_region(outside,[deck()])['accepted']
    obstacle=OBB([0,0,.76],[.02,.02,.02],np.eye(3),'frame','fixed')
    assert predict_release(carton(),[deck()],mode=IDEAL_RECEPTION_RELEASE,obstacles=[obstacle])['reason']=='IDEAL_RECEPTION_ENVELOPE_COLLISION'


def test_policy_has_one_authority_and_only_interior_nominal_candidates():
    assert ReleasePolicy().ideal_heights()==pytest.approx([.025,.035,.045])
    for values in [dict(maximum_drop_m=.06),dict(ideal_release_min_height_m=.015),
                   dict(ideal_release_height_reserve_m=.02)]:
        with pytest.raises(ValueError):ReleasePolicy(**values).ideal_heights()
    with pytest.raises(ValueError):ideal_reception_region(carton(),[deck()],maximum_drop_m=.04)
    for invalid in (float('nan'),float('inf')):
        with pytest.raises(ValueError):ReleasePolicy(ideal_release_height_reserve_m=invalid)


def runtime_metadata():
    box=carton(.025); receiver=deck()
    prediction=predict_release(box,[receiver],mode=IDEAL_RECEPTION_RELEASE)
    primitive=lambda b:dict(name=b.name,center_m=b.center.tolist(),size_m=(2*b.half_extents).tolist(),
        rotation_matrix=b.rotation.tolist(),category=b.category,dynamic=b.name==box.name)
    return dict(target=box.name,release_prediction=prediction,release_mode=IDEAL_RECEPTION_RELEASE,
        selected_place_support_names=[receiver.name],scene_primitives=[primitive(box),primitive(receiver)])


@pytest.mark.parametrize('height,accepted',[(.019999,False),(.020,True),(.025,True),(.050,True),(.050001,False)])
def test_runtime_checks_actual_height_not_command(height,accepted):
    metadata=runtime_metadata()
    # Command also near the upper bound for that boundary test; actual bounds
    # must pass independently of the existing 3 mm pose-binding gate.
    if height>.04:
        metadata['release_prediction']=predict_release(carton(.049),[deck()],mode=IDEAL_RECEPTION_RELEASE)
    elif height<.024:
        metadata['release_prediction']=predict_release(carton(.022),[deck()],mode=IDEAL_RECEPTION_RELEASE)
    result=audit_runtime_short_drop(metadata,position=carton(height).center,rotation=np.eye(3),
        linear_velocity=[0,0,0],angular_velocity=[0,0,0],current_cartons=[])
    assert result['accepted'] is accepted


def test_approved_same_body_can_descend_below_twenty_without_a_second_release_gate():
    p={**POLICY,'reception_mode':'ideal'}; box=carton(.025)
    record=begin_ideal_transport(box,receiver_name='belt',receivers={'belt':deck()},
        directions={'belt':[-1,0,0]},time_s=0,policy=p,attachment_removed=True,
        top_contact_observed=False,support_geometry_accepted=False,expected_target='box',actual_attachment_observed=True,
        released_handoff=measured_handoff(box,deck(),p,0))
    advance_ideal_transport(record,dt_s=.05,speed_m_s=.3)
    assert record['pose_world'][2][3]==pytest.approx(.760)
    assert record['reception_region']['gap_m']==pytest.approx(.025)
    assert not record['actual_top_contact_observed']
    envelope=release_flight_envelope(box,predict_release(box,[deck()],mode=IDEAL_RECEPTION_RELEASE))
    assert envelope.corners()[:,2].min()==pytest.approx(.6)


@pytest.mark.parametrize('reverse',[False,True])
@pytest.mark.parametrize('actor',['/receiver','/receiver/TopCollision'])
def test_exact_created_top_identity_in_both_orders(actor,reverse):
    args=dict(actor0='/box',actor1=actor,collider0='/box',collider1='/receiver/TopCollision',
        target_path='/box',receiver_top_owners={'/receiver/TopCollision':'/receiver'},
        declared_receivers={'/receiver'},stage='place',attached=True,separation_m=0.,maximum_penetration_m=.001)
    if reverse:
        for field in ('actor','collider'):args[field+'0'],args[field+'1']=args[field+'1'],args[field+'0']
    assert declared_receiver_top_contact(**args)
    assert declared_receiver_top_contact(**{**args,'stage':'transit'}) is None
    assert declared_receiver_top_contact(**{**args,'attached':False})


@pytest.mark.parametrize('collider',['/receiver/StructureCollision','/receiver/Leg','/receiver/Unknown','/robot/Tool'])
def test_side_unknown_or_robot_cannot_inherit_top_permission(collider):
    assert declared_receiver_top_contact(actor0='/box',actor1='/receiver',collider0='/box',collider1=collider,
        target_path='/box',receiver_top_owners={'/receiver/TopCollision':'/receiver'},declared_receivers={'/receiver'},
        stage='place',attached=True,separation_m=0.,maximum_penetration_m=.001) is None


def test_contact_ledger_requires_actual_created_top_collider():
    from unloading_sim.isaac_collision_policy import physical_support_contact_observed
    owners={'/receiver/TopCollision':'/receiver'}
    for collider,expected in [('/receiver/TopCollision',True),('/receiver/StructureCollision',False)]:
        headers={('/box','/receiver','/box',collider)}
        assert bool(physical_support_contact_observed(headers,'/box',{'/receiver'},owners)) is expected


def test_old_measured_transit_gap_still_fails_unchanged_runtime_rule():
    r=classify_poc_runtime_pair(collider0='/box',collider1='/receiver/TopCollision',
        minimum_separation_m=.0049581085331737995,policy=policy(),robot_link_by_collider={},
        owned_tool_colliders=set(),stage='transit')
    assert not r['accepted'] and r['required_pair_clearance_m']==.005


def test_real_production_constructor_consumes_all_reserves_and_rejects_old_release_heights():
    scene=build_verified_motion_input(load_layout_motion_policy(DEFAULT_MOTION))
    c=_build_automatic_trajectory_connector(scene,scene.policy.layout_validation.layout.robot()).connector
    assert c.budget.execution_reserves()=={name:.003 for name in (
        'approach_runtime_clearance_reserve_m','receiver_runtime_clearance_reserve_m',
        'departure_runtime_clearance_reserve_m','extraction_runtime_clearance_reserve_m')}
    assert c.budget.release_policy().ideal_heights()==pytest.approx([.025,.035,.045])
    for height in (0,.010,.015):
        # The real finish entry rejects before any IK, even on an old/history call.
        _,failure,_=c._finish_place_branch(target=None,face=None,requested_virtual_contact=None,home_q=None,
            contact_q=None,physical_contact=None,rigid=None,attachment=None,selection=None,pregrasp=None,
            contact=None,support_release=None,extraction=None,released_tracker=None,payload_obstacles=None,
            placement=None,selected_supports=None,trace={},seed=0,release_height=height)
        assert failure['reason']=='IDEAL_RELEASE_HEIGHT_OUT_OF_BOUNDS'


def test_production_state_edges_and_cache_consume_receiver_reserve():
    from test_search_diagnostics import connector
    from unloading_sim.layout_trajectory import PhysicalContactAttachment
    from unloading_sim.validation_physics import RigidAttachment
    from unloading_sim.pair_clearance import obb_pair_failure
    # Minimal translational robot; the actual payload validator and dense edge
    # checker run unchanged, including the ordinary 5 mm check.
    def validator(q, obstacles, *, payload, **kwargs):
        if payload is not None:
            return next((failure for o in obstacles if (failure:=obb_pair_failure(payload,o,policy(),.005))),None)
    c=connector(validator);c.collision_policy=policy()
    c.budget=replace(c.budget,receiver_runtime_clearance_reserve_m=.003)
    body=OBB([0,0,0],[.1,.1,.1],np.eye(3),'box','carton')
    attachment=PhysicalContactAttachment(c.robot,RigidAttachment(np.eye(4),body.half_extents,body.name),np.eye(4),np.eye(4))
    receiver=OBB([0,0,-.206],[1,1,.1],np.eye(3),'receiver','conveyor')
    failure=c._path_failure([np.array([0,0,.004,0,0,0]),np.zeros(6)],[receiver],attachment=attachment,stage='transit')
    assert failure['reason']=='PLANNING_EXECUTION_RESERVE'
    assert failure['required_pair_clearance_m']==pytest.approx(.008)
    c.budget=replace(c.budget,receiver_runtime_clearance_reserve_m=0.)
    assert c._state_failure(np.zeros(6),[receiver],attachment=attachment,stage='transit') is None
    c.budget=replace(c.budget,receiver_runtime_clearance_reserve_m=.003)
    assert c._state_failure(np.zeros(6),[receiver],attachment=attachment,stage='transit')['reason']=='PLANNING_EXECUTION_RESERVE'
    # Placement contact permission remains independently scoped.
    assert c._state_failure(np.zeros(6),[receiver],attachment=attachment,stage='place') is None


def test_release_runtime_source_orders_actual_gate_before_removal_and_separate_takeover():
    from pathlib import Path
    source=(Path(__file__).resolve().parents[1]/'scripts/isaacsim_fanuc_replay.py').read_text()
    check=source.index('"event": "ideal_release_actual_height_check"')
    remove=source.index('stage.RemovePrim(grasp_joint_path)',check)
    evidence=source.index('"event": "actual_constraint_removed"',remove)
    takeover=source.index('record = begin_ideal_transport(',evidence)
    assert check<remove<evidence<takeover
    import ast
    from types import SimpleNamespace
    branch=next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.If)
        and any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='ideal_takeover_audit'
            for t in x.targets) for x in n.body))
    condition=compile(ast.Expression(branch.test),'production_takeover_condition','eval')
    inputs=dict(ideal_reception_mode=True,release_open_confirmed=True,assumed_reception_state=None,
        stack_clearance_step=SimpleNamespace(hold=False))
    assert eval(condition,inputs)
    inputs['stack_clearance_step'].hold=True
    assert not eval(condition,inputs)


def test_zero_displacement_place_keeps_actual_release_dwell_stage():
    from unloading_sim.m710_replay_physics import resolve_actual_task_stage
    windows=[dict(stage='transit',start_time_s=1.,end_time_s=2.),dict(stage='place',start_time_s=2.,end_time_s=2.),
             dict(stage='withdrawal',start_time_s=2.,end_time_s=3.)]
    args=dict(grasp_commanded=True,grasp_event_time_s=.5,contact_wait_started_s=None,
              attached=True,release_commanded=False,release_event_time_s=2.1)
    assert resolve_actual_task_stage(windows,1.999,**args)=='transit'
    assert resolve_actual_task_stage(windows,2.,**args)=='place'
    assert resolve_actual_task_stage(windows,2.05,**args)=='place'
    assert resolve_actual_task_stage(windows,2.11,**{**args,'attached':False,'release_commanded':True})=='withdrawal'


def test_execution_preflight_policy_rejects_height_and_reserve_mismatch():
    from unloading_sim.m710_execution import _validate_poc_release_policy
    strategy=load_layout_motion_policy(DEFAULT_MOTION).data['search_strategy']
    segment=dict(place=dict(release_prediction=dict(policy=ReleasePolicy().to_mapping())),
        planning_execution_reserves={name:strategy[name] for name in (
            'approach_runtime_clearance_reserve_m','receiver_runtime_clearance_reserve_m',
            'departure_runtime_clearance_reserve_m','extraction_runtime_clearance_reserve_m')})
    _validate_poc_release_policy(segment,strategy)
    changed=deepcopy(segment);changed['place']['release_prediction']['policy']['ideal_release_min_height_m']=.015
    with pytest.raises(ValueError,match='height policy'):_validate_poc_release_policy(changed,strategy)
    changed=deepcopy(segment);changed['planning_execution_reserves']['receiver_runtime_clearance_reserve_m']=0.
    with pytest.raises(ValueError,match='execution reserves'):_validate_poc_release_policy(changed,strategy)


def geometric_connector():
    from test_search_diagnostics import connector
    from unloading_sim.pair_clearance import obb_pair_failure
    def tools(q):
        return [OBB(np.asarray(q)[:3]+[-.010,0,0],[.004]*3,np.eye(3),'rigid_tool','tool')]
    def validator(q, obstacles, **kwargs):
        return next((failure for tool in tools(q) for obstacle in obstacles
                     if (failure:=obb_pair_failure(tool,obstacle,policy(),.005))),None)
    c=connector(validator);c.collision_policy=policy()
    c.robot.clamp=lambda q: np.clip(q,-2.,2.)
    c.ik=dict(load_layout_motion_policy(DEFAULT_MOTION).data['ik'])
    c.tool_collision_obbs_provider=tools
    c.budget=replace(c.budget,proof_of_concept=True,approach_runtime_clearance_reserve_m=.003,
                    receiver_runtime_clearance_reserve_m=.003,departure_runtime_clearance_reserve_m=.003,
                    extraction_runtime_clearance_reserve_m=.003)
    return c


def test_departure_generation_and_residence_retain_actual_tool_reserve():
    from unloading_sim.pair_clearance import obb_surface_distance
    c=geometric_connector();box=carton(.025);start=np.array([-.3,0,.775,0,0,0])
    prediction=predict_release(box,[deck()],mode=IDEAL_RECEPTION_RELEASE)
    path,failure,audit=c._departure(start,box,[deck()],[0,-1,0],seed=3,
        working_normal=[1,0,0],release_prediction=prediction)
    assert failure is None and len(path)>1
    gap=obb_surface_distance(c.tool_collision_obbs_provider(path[-1])[0],release_flight_envelope(box,prediction))
    assert gap>=.008-1e-9
    assert audit['distance_m']==pytest.approx(.0024)


def test_approach_generation_consumes_terminal_reserve_without_inflating_contact():
    c=geometric_connector()
    def fk(q):
        pose=np.eye(4);pose[:3,:3]=rotation_matrix_from_rpy(0,np.pi/2,0);pose[:3,3]=np.asarray(q)[:3]
        return pose
    c.robot.fk=fk
    target=OBB([.1,0,.5],[.1]*3,np.eye(3),'box','carton')
    grasp=np.array([0,0,.5,0,0,0]);start=np.array([-.05,0,.5,0,0,0])
    pre,contact,failure,audit=c._approach(start,grasp,fk(grasp),[target],target,seed=3)
    assert failure is None and contact
    assert audit['attempts'][0]['terminal_distance_m']==pytest.approx(.0084)
    assert np.allclose(contact[-1],grasp,atol=1e-4)


def test_extraction_generation_keeps_single_three_mm_reserve_after_fk():
    from unloading_sim.layout_trajectory import PhysicalContactAttachment
    from unloading_sim.validation_physics import RigidAttachment
    from unloading_sim.pair_clearance import obb_surface_distance
    c=geometric_connector();start=np.zeros(6)
    body=OBB([0,0,0],[.1]*3,np.eye(3),'box','carton')
    neighbor=OBB([.2,0,0],[.1]*3,np.eye(3),'neighbor','carton')
    attachment=PhysicalContactAttachment(c.robot,RigidAttachment(np.eye(4),body.half_extents,body.name),np.eye(4),np.eye(4))
    tracker,failure=c._initial_proximity(body,[neighbor],[])
    assert failure is None
    path,tracker,failure,audit=next(c._extraction_options(start,attachment,[neighbor],tracker,np.array([-1.,0,0]),seed=3))
    assert failure is None and tracker.fully_released
    attempt=audit['attempts'][audit['selected_attempt']]
    assert attempt['runtime_clearance_goal_m']==pytest.approx(.0082)
    assert attempt['runtime_clearance_reserve_m']==.003
    assert obb_surface_distance(attachment.box_at(path[-1]),neighbor)>=.0082-1e-9
