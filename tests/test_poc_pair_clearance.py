from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.collision_policy import SimulationCollisionPolicy, PhysicsCheckedStackTracker
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.pair_clearance import obb_surface_distance, obb_pair_failure, obb_pair_evidence
from unloading_sim.isaac_collision_policy import classify_poc_runtime_pair
from unloading_sim.layout_single_carton import load_layout_motion_policy, build_verified_motion_input, _build_automatic_trajectory_connector
from unloading_sim.planning_profile import DEFAULT_MOTION


def policy():
    return SimulationCollisionPolicy.from_mapping(load_layout_motion_policy(DEFAULT_MOTION).layout_validation.data['collision_policy'])


def box(center, name='box', half=(.1, .1, .1), rotation=None):
    return OBB(center, half, np.eye(3) if rotation is None else rotation, name, 'carton')


@pytest.mark.parametrize('gap,accepted', [(-.001, False), (0., False), (.004, False),
    (.005-2e-9, False), (.005-5e-10, True), (.005, True), (.006, True), (.015, True)])
def test_total_surface_gap_and_runtime_classification(gap, accepted):
    p=policy();a=box([0,0,0], 'a');b=box([.2+gap,0,0], 'b')
    cpu=obb_pair_evidence(a,b,p,stage='transit')
    runtime=classify_poc_runtime_pair(collider0='/a', collider1='/b', minimum_separation_m=gap,
        policy=p, robot_link_by_collider={'/a':'J2_link'}, owned_tool_colliders={}, stage='transit')
    assert cpu['accepted'] == runtime['accepted'] == accepted
    assert cpu['classification'] == runtime['classification']
    assert cpu['required_pair_clearance_m'] == .005
    assert 'penetration_depth_m' not in cpu
    if gap == .015:
        assert obb_pair_failure(a,b,SimulationCollisionPolicy(),.01)
        assert obb_pair_failure(a,b,p,.01) is None  # no hidden addition of old margin


def test_unknown_contact_and_proxy_overlap_fail_closed():
    p=policy();a=box([0,0,0]);b=box([0,0,0], 'tool')
    evidence=obb_pair_evidence(a,b,p,proxy=True)
    assert evidence['classification']=='PROXY_GEOMETRY_INTERSECTION'
    assert not evidence['accepted'] and not evidence['original_cad_intersection_confirmed']
    for distance in (None, float('nan')):
        result=classify_poc_runtime_pair(collider0='/a',collider1='/b',minimum_separation_m=distance,
            policy=p, robot_link_by_collider={},owned_tool_colliders={},stage='transit')
        assert result['classification']=='UNKNOWN' and not result['accepted']


def test_self_gap_uses_zero_not_twenty_mm_and_contact_still_fails():
    from unloading_sim.pinocchio_backend import PinocchioHppFclBackend
    import coal
    backend=object.__new__(PinocchioHppFclBackend);backend.coal=coal
    shape=coal.Box(.2,.2,.2);p=policy()
    for gap in (.015,.004,0.,-.001):
        result=backend._poc_pair_result(shape,coal.Transform3s(np.eye(3),np.zeros(3)),
            shape,coal.Transform3s(np.eye(3),np.array([.2+gap,0,0])),p.pair_clearance('robot_self',.01),
            p,('J2_link','J4_link'),'transit')
        assert result.in_collision == (gap <= 0)
    assert p.wrist_tool_pairs(['owned'])=={('J5_link','owned'),('J6_link','owned')}
    assert ('J4_link','owned') not in p.wrist_tool_pairs(['owned'])


def test_rotated_box_euclidean_features_agree_with_coal():
    import coal
    rng=np.random.default_rng(7102)
    for _ in range(40):
        a=box([0,0,0],rotation=rotation_matrix_from_rpy(*rng.uniform(-2,2,3)))
        b=box(rng.uniform(-.4,.4,3),rotation=rotation_matrix_from_rpy(*rng.uniform(-2,2,3)))
        sa,sb=coal.Box(*(.2*np.ones(3))),coal.Box(*(.2*np.ones(3)))
        ta,tb=coal.Transform3s(a.rotation,a.center),coal.Transform3s(b.rotation,b.center)
        overlap=coal.CollisionResult();coal.collide(sa,ta,sb,tb,coal.CollisionRequest(),overlap)
        expected=0. if overlap.isCollision() else coal.distance(sa,ta,sb,tb,coal.DistanceRequest(),coal.DistanceResult())
        assert obb_surface_distance(a,b)==pytest.approx(expected,abs=2e-8)
    a=box([0,0,0]);b=box([.204,.204,0],rotation=rotation_matrix_from_rpy(0,0,0))
    assert a.intersects_obb(b,margin=.0025)  # both local expansions are not Euclidean distance
    assert obb_surface_distance(a,b)==pytest.approx(np.sqrt(2)*.004)
    assert obb_pair_failure(a,b,policy(),.01) is None


def test_finite_frozen_walls_check_entire_box_including_door_edge():
    scene=build_verified_motion_input(load_layout_motion_policy(DEFAULT_MOTION))
    walls={b.name:b for b in scene.fixed_components if b.name.startswith('trailer_')}
    p=policy()
    for center,reject in [([0,1.15,1.5],True), ([-3.5,1.3,1.5],False), ([-3.25,1.15,1.5],True)]:
        test=box(center)
        assert bool(obb_pair_failure(test,walls['trailer_left_wall'],p,.01))==reject
    for name in ('trailer_right_wall','trailer_ceiling','trailer_closed_end_wall'):
        assert obb_pair_failure(box(walls[name].center),walls[name],p,.01)
    # Actual production plane checker must leave side walls to finite geometry.
    built=_build_automatic_trajectory_connector(scene,scene.policy.layout_validation.layout.robot())
    v=built.connector.robot_state_validator
    q=scene.policy.layout_validation.initial_q
    assert v._plane_failure(q,box([-3.5,1.4,1.5]),'transit') is None
    assert v._plane_failure(q,box([-3.5,1.4,-.1]),'transit')['reason']=='FLOOR_CLEARANCE'


def test_stack_entry_is_5p2mm_and_free_rule_cannot_be_lost():
    from unloading_sim.m710_replay_physics import ActualStackContactMonitor
    p=policy();neighbor=box([0,0,0],'neighbor');start=box([.2,0,0],'target')
    tracker=PhysicsCheckedStackTracker(start,[neighbor],p,.01,.0002)
    assert not tracker.fully_released
    assert tracker.state_failure(box([.206,0,0],'target'),[neighbor]) is None
    assert tracker.fully_released
    assert tracker.state_failure(box([.204,0,0],'target'),[neighbor])['classification']=='CLEARANCE_INSUFFICIENT'


def test_policy_identity_and_legacy_configuration_are_isolated():
    p=policy();old=SimulationCollisionPolicy.from_mapping(load_layout_motion_policy(
        'configs/validation/m710id70_layout_v1_single_carton.yaml').layout_validation.data['collision_policy'])
    assert not old.poc_pair_clearance and old.free_space_clearance_m==.0202
    assert old.pair_clearance('robot_self',.01)==.02
    assert p.fingerprint != old.fingerprint and p.free_space_clearance_m==.0052
    assert SimulationCollisionPolicy.from_mapping(old.to_mapping()).fingerprint==old.fingerprint
    with pytest.raises(ValueError):
        replace(p,required_pair_clearance_m=.01)
    scene=build_verified_motion_input(load_layout_motion_policy(DEFAULT_MOTION))
    built=_build_automatic_trajectory_connector(scene,scene.policy.layout_validation.layout.robot())
    c=built.connector
    a=c._context_identity(scene.all_obstacles,stage='transit')
    c.collision_policy=old
    assert c._context_identity(scene.all_obstacles,stage='transit')!=a


def test_release_latch_uses_fresh_5p2mm_separation_not_contact_envelope():
    from unloading_sim.m710_replay_physics import BoundedTargetCupReleaseClearance
    pair = ('/robot', '/target', '/robot/cup', '/target')
    gate = BoundedTargetCupReleaseClearance(1., required_clearance_m=policy().free_space_clearance_m)
    gate.begin(0.)
    for distance in (None, float('nan'), .004, .005):
        values = {} if distance is None else {pair: distance}
        assert gate.observe(.1, {pair}, target_path='/target', compliant_paths={'/robot/cup'},
                            current_pair_separations=values) is None
        assert gate.pending
    gate.observe(.2, {pair}, target_path='/target', compliant_paths={'/robot/cup'},
                 current_pair_separations={pair: .0052})
    assert not gate.pending  # Positive proximity header remains; no 20 mm LOST requirement.
    gate.observe(.3, {pair}, target_path='/target', compliant_paths={'/robot/cup'},
                 current_pair_separations={pair: .004})
    assert not gate.pending  # Normal runtime pair rule handles recontact; no reopened permission.


def test_preflight_rejects_changed_effective_policy_at_same_config_path():
    from copy import deepcopy
    from unloading_sim.m710_execution import build_m710_execution_preflight
    from unloading_sim.planning_profile import DEFAULT_EXECUTION
    scene = build_verified_motion_input(load_layout_motion_policy(DEFAULT_MOTION))
    data = deepcopy(scene.policy.layout_validation.data)
    data['collision_policy'] = SimulationCollisionPolicy().to_mapping()
    changed = replace(scene, policy=replace(scene.policy,
        layout_validation=replace(scene.policy.layout_validation, data=data)))
    with pytest.raises(ValueError, match='actual motion input collision policy differs'):
        build_m710_execution_preflight(DEFAULT_EXECUTION, motion_input=changed, motion_result={})
