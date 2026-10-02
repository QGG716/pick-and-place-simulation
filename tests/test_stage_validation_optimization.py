"""Focused invariant/bound regressions; these synthetic scenes are not qualification."""
from dataclasses import replace
import numpy as np
import pytest

from unloading_sim.collision_policy import SimulationCollisionPolicy, PhysicsCheckedStackTracker
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.layout_trajectory import ExactM710LayoutStateValidator, LayoutTrajectoryConnector, PhysicalContactAttachment
from unloading_sim.motion_validation import RequestBudget, Status
from unloading_sim.pair_clearance import obb_surface_distance, obb_distance_at_least, obb_aabb_distance_lower, obb_pair_failure
from unloading_sim.robot import CollisionResult
from unloading_sim.validation_physics import RigidAttachment


def policy():
    return SimulationCollisionPolicy(schema='m710_poc_pair_collision_policy_v4',
        required_pair_clearance_m=.005, boundary_mode='finite_frozen_scene_walls',
        free_space_clearance_m=.0052, stack_contact_mode='planner_relaxed_physics_checked')


class Robot:
    dof = 6
    joint_limits = np.tile([-2., 2.], (6, 1))
    base_transform = tip_from_tcp = np.eye(4)
    geometry_revision = 0

    def __init__(self):
        self.tool_collision_local_boxes = np.array([[.206, 0, .103, .2, .2, .2]])
        self.tool_compliant_collision_local_boxes = np.array([[0, 0, 0, .08, .08, .01]])
        self.mesh_failure = False

    def fk(self, q):
        t = np.eye(4)
        t[:3, :3] = rotation_matrix_from_rpy(0, 0, q[1])
        t[:3, 3] = [q[0], 0, 1]
        return t

    def geometric_jacobian(self, q): return np.eye(6)
    def within_limits(self, q): return np.asarray(q).shape == (6,) and np.all(np.abs(q) <= 2)
    def named_link_frames(self, q): return {'flange': self.fk(q)}
    def collision_result(self, q, obstacles, **unused):
        return CollisionResult(self.mesh_failure, 'supplementary_mesh', 'J2_link', 'obstacle')
    def _boxes(self, q, rows, prefix):
        t = self.fk(q)
        return [OBB(t[:3, :3]@r[:3]+t[:3, 3], r[3:]/2, t[:3, :3], prefix+str(i), 'robot')
                for i, r in enumerate(rows)]
    def tool_collision_obbs(self, q): return self._boxes(q, self.tool_collision_local_boxes, 'tool_rigid_')
    def tool_compliant_collision_obbs(self, q): return self._boxes(q, self.tool_compliant_collision_local_boxes, 'cup_')
    def tool_all_physical_obbs(self, q): return self.tool_collision_obbs(q)+self.tool_compliant_collision_obbs(q)


def setup(mode='optimized'):
    r = Robot()
    v = ExactM710LayoutStateValidator(r, r, collision_margin_m=.01, floor_z_m=0.,
        right_wall_y_m=-2., left_wall_y_m=2., robot_world_boxes=lambda q: [],
        collision_policy=policy().to_mapping(), nominal_cup_compression_m=0.)
    v.validation_mode = mode
    v.commanded_cup_mask = (True,)
    v.contact_target_name = 'payload'
    c = LayoutTrajectoryConnector(r, v, flange_from_virtual_task_tcp=np.eye(4),
        flange_from_physical_contact=np.eye(4), ik_policy={}, collision_margin_m=.01,
        contact_tolerance_m=.0002, joint_margin_rad=.001, maximum_jacobian_condition=1e4,
        validator_identity='synthetic_stage_optimization', execution_qualified=True,
        collision_policy=policy().to_mapping())
    c.start_planning_request()
    t = np.eye(4); t[2, 3] = .103
    a = PhysicalContactAttachment(r, RigidAttachment(t, np.full(3, .1), 'payload'), np.eye(4), np.eye(4))
    world = [OBB([4, 0, 1], [.1]*3, np.eye(3), 'neighbor', 'carton')]
    return c, a, world


def run(c, a, world, stage='transit'):
    q = np.zeros(6); end = q.copy(); end[0] = .002; end[1] = .001
    failure = c._path_failure([q, end], world, attachment=a, stage=stage)
    return failure, c.last_stage_validation_profile


def test_cold_invariant_reuse_preserves_grid_and_real_supplementary_checks():
    ref, a, w = setup('reference'); opt, b, v = setup()
    rf, rp = run(ref, a, w); of, op = run(opt, b, v)
    assert rf is of is None
    assert rp['motion']['state_visits'] == op['motion']['state_visits']
    assert rp['motion']['unique_state_visits'] == op['motion']['unique_state_visits']
    assert op['query_counts']['fixed_tool_payload_pair_reuses'] > 0
    assert op['query_counts']['obb_sat_queries'] < rp['query_counts']['obb_sat_queries']
    # A passing fixed pair never substitutes for the independent robot mesh gate.
    opt.start_planning_request(); opt.robot.mesh_failure = True
    failure, _ = run(opt, b, v)
    assert failure['reason'] == 'ROBOT_MESH_COLLISION'


@pytest.mark.parametrize('change', ['attachment', 'payload_shape', 'tool_geometry', 'mask', 'stage', 'scene', 'policy'])
def test_binding_changes_recheck_fixed_pairs(change):
    c, a, w = setup(); assert run(c, a, w)[0] is None
    old = c.last_stage_validation_profile['context_id']
    stage = 'transit'
    if change == 'attachment': a.rigid.tcp_from_box[1, 3] += .001
    elif change == 'payload_shape': a.rigid.half_extents[1] -= .001
    elif change == 'tool_geometry': c.robot.tool_collision_local_boxes[0, 0] += .001
    elif change == 'mask': c.robot_state_validator.commanded_cup_mask = (False,)
    elif change == 'stage': stage = 'extraction'
    elif change == 'scene': w[0].center[0] += .001
    elif change == 'policy':
        c.collision_policy = replace(c.collision_policy, maximum_compliant_cup_additional_compression_m=.004)
        c.robot_state_validator.collision_policy = c.collision_policy
    failure, profile = run(c, a, w, stage)
    assert failure is None
    assert profile['context_id'] != old
    assert profile['query_counts']['fixed_relation_context_builds'] == 1


def test_new_request_clears_fixed_pair_and_state_conclusions():
    c, a, w = setup(); assert run(c, a, w)[0] is None
    assert c.robot_state_validator._fixed_tool_payload
    c.start_planning_request()
    assert not c.robot_state_validator._fixed_tool_payload
    assert not c._state_cache
    assert run(c, a, w)[1]['query_counts']['fixed_relation_context_builds'] == 1


def test_missing_context_does_not_enable_fixed_pair_reuse():
    c, a, w = setup(); q = np.zeros(6)
    assert c._state_failure(q, w, attachment=a, stage='transit') is None
    assert not c.robot_state_validator._fixed_tool_payload


def test_rigid_tool_penetration_and_overcompressed_cup_are_still_rejected():
    c, a, w = setup(); assert run(c, a, w)[0] is None
    a.rigid.tcp_from_box[0, 3] = .02
    failure, _ = run(c, a, w)
    assert failure['reason'] == 'RIGID_TOOL_COLLISION'
    c, a, w = setup(); a.rigid.tcp_from_box[2, 3] = .094
    failure, _ = run(c, a, w)
    assert failure['reason'] == 'RIGID_TOOL_COLLISION'
    assert failure['pair'][0] == 'cup_0'


@pytest.mark.parametrize('gap', [.004986852246589828, .005-2e-9, .005, .005+2e-9, .1])
def test_bound_keeps_five_mm_boundary_and_recorded_rejection(gap):
    a = OBB([0, 0, 1], [.1]*3, np.eye(3), 'payload')
    b = OBB([.2+gap, 0, 1], [.1]*3, np.eye(3), 'neighbor')
    assert obb_aabb_distance_lower(a, b) <= obb_surface_distance(a, b)
    assert obb_distance_at_least(a, b, .0052) == (obb_surface_distance(a, b) >= .0052)
    assert (obb_pair_failure(a, b, policy(), .01) is None) == (gap+1e-9 >= .005)


def test_rotated_bounds_are_conservative_and_unknown_does_not_pass():
    rng = np.random.default_rng(71203)
    for _ in range(20):
        a = OBB(rng.uniform(-1, 1, 3), rng.uniform(.02, .2, 3), rotation_matrix_from_rpy(*rng.uniform(-3, 3, 3)))
        b = OBB(rng.uniform(-1, 1, 3), rng.uniform(.02, .2, 3), rotation_matrix_from_rpy(*rng.uniform(-3, 3, 3)))
        assert obb_aabb_distance_lower(a, b) <= obb_surface_distance(a, b)+1e-12


def test_ordered_extraction_still_rejects_reentry_and_keeps_direction():
    p = policy(); neighbor = OBB([.2, 0, 1], [.1]*3, np.eye(3), 'neighbor', 'carton')
    def target(x): return OBB([x, 0, 1], [.1]*3, np.eye(3), 'payload', 'payload')
    for optimized in (False, True):
        tracker = PhysicsCheckedStackTracker(target(0), [neighbor], p, .01, .0002, optimized=optimized)
        assert not tracker.fully_released
        assert tracker.state_failure(target(-.006), [neighbor]) is None
        assert tracker.fully_released
        failure = tracker.state_failure(target(0), [neighbor])
        assert failure['contact_state'] == 'FREE_SPACE_RULES_RESTORED'
        assert tracker.clone().fully_released
        reverse = PhysicsCheckedStackTracker(target(-.006), [neighbor], p, .01, .0002, optimized=optimized)
        assert reverse.state_failure(target(0), [neighbor])['contact_state'] == 'FREE_SPACE_RULES_RESTORED'


def test_release_retains_payload_obstacle_and_ends_fixed_relation_permission():
    c, a, w = setup(); assert run(c, a, w)[0] is None
    q = np.zeros(6)
    # Released box remains in the receiver scene. A changed robot pose meets it.
    released = a.box_at(q)
    q[0] = -.2
    failure = c._path_failure([q], [*w, released], stage='withdrawal', target_contact=released)
    assert failure['reason'] == 'RIGID_TOOL_COLLISION'
    assert c.last_stage_validation_profile['query_counts'].get('fixed_tool_payload_pair_reuses', 0) == 0


def test_cancellation_budget_and_stale_context_never_become_invalid_cache():
    c, a, w = setup(); v = c._motion_validator(w, attachment=a, stage='transit')
    q = np.zeros(6); end = q.copy(); end[0] = .002
    assert v.check_path([q, end], RequestBudget(cancelled=lambda: True)).status == Status.CANCELLED
    assert v.check_path([q, end], RequestBudget(max_checks=0)).status == Status.INDETERMINATE
    assert not v.states and not v.edges
    w[0].center[0] -= .001
    assert v.check_path([q, end]).status == Status.INDETERMINATE
    assert not v.states and not v.edges
