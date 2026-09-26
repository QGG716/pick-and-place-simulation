from types import SimpleNamespace
from time import perf_counter
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.motion_quality import collinear_indices, path_quality, reversal_counts
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.timing import JointMotionLimits, time_parameterize_joint_path, source_waypoint_times, sample_quintic_knots
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.release_motion import predict_release, SHORT_DROP_RELEASE


def test_release_envelope_contains_rotating_flight_and_vertical_apex():
    from dataclasses import asdict
    from unloading_sim.release_motion import ReleasePolicy, flight_box, release_flight_envelope
    box = OBB([0., 0., .8], [.3, .2, .15], rotation_matrix_from_rpy(.1, -.2, .3), 'payload', 'carton')
    velocity, omega = [.02, -.03, .2], [.05, .03, -.02]
    prediction = dict(flight_time_s=.08, linear_velocity_m_s=velocity,
                      angular_velocity_rad_s=omega, policy=asdict(ReleasePolicy()))
    envelope = release_flight_envelope(box, prediction)
    for t in np.r_[np.linspace(0, .08, 101), .2/9.81]:
        corners = flight_box(box, velocity, omega, t).corners()
        assert np.all(corners >= envelope.center-envelope.half_extents)
        assert np.all(corners <= envelope.center+envelope.half_extents)
    assert release_flight_envelope(box, {**prediction, 'flight_time_s':0.}) is box


def test_bounded_shortcut_checks_interior_and_preserves_process_boundary():
    path = np.array([[0.,0.], [0.,1.], [1.,1.], [1.,0.]])
    blocked = lambda q: not (.2 < q[0] < .8 and q[1] < .5)
    planner = RRTConnectPlanner(-2*np.ones(2), 2*np.ones(2), blocked, edge_resolution=.025)
    reduced, info = planner.bounded_shortcut(path, deadline=perf_counter()+1, protected=[1], attempts=8)
    assert any(np.array_equal(p, path[1]) for p in reduced)
    assert len(reduced) > 2
    assert all(planner.edge_valid(a,b) for a,b in zip(reduced[:-1], reduced[1:]))
    assert info['state_checks'] > 2


def test_exhausted_shortcut_never_publishes_partly_checked_edge():
    path = np.array([[0.,0.], [0.,1.], [1.,1.], [1.,0.]])
    planner = RRTConnectPlanner(-2*np.ones(2), 2*np.ones(2), lambda q: True, edge_resolution=.01)
    reduced, info = planner.bounded_shortcut(path, deadline=perf_counter()+1, state_budget=2)
    np.testing.assert_array_equal(reduced, path)
    assert info['state_checks'] == 2


def test_collinear_samples_pass_with_nonzero_velocity_and_corners_remain():
    path = np.array([[0.,0.],[.25,0.],[.5,0.],[1.,0.],[1.,1.]])
    keep = collinear_indices(path)
    assert keep == [0,3,4]
    assert collinear_indices(path, protected=[2]) == [0,2,3,4]
    limits = JointMotionLimits(np.ones(2), np.ones(2)*2, np.ones(2)*8)
    timed = time_parameterize_joint_path(path[keep], limits)
    times = source_waypoint_times(path, keep, timed)
    q, v = sample_quintic_knots(timed.time_from_start, timed.positions, times)
    np.testing.assert_allclose(q, path, atol=1e-12)
    assert v[2,0] > 0
    np.testing.assert_allclose(v[3], 0, atol=1e-12)
    assert timed.audit(limits)['within_limits']
    for t in np.linspace(0, timed.duration_seconds, 31):
        q, v = sample_quintic_knots(timed.time_from_start, timed.positions, t)
        expected = timed.sample(t)
        np.testing.assert_allclose(q, expected[0], atol=1e-12)
        np.testing.assert_allclose(v, expected[1], atol=1e-12)


def test_so3_crossing_pi_and_hysteresis_do_not_create_false_reversals():
    values = np.array([[3.13],[3.15]])
    result = path_quality(values, fk=lambda q: np.block([
        [rotation_matrix_from_rpy(0,0,q[0]), np.zeros((3,1))], [np.zeros((1,3)), np.ones((1,1))]]))
    assert result['orientation_travel_so3_rad'] == pytest.approx(.02)
    full_turn = path_quality([[0.], [2*np.pi]], fk=lambda q: np.block([
        [rotation_matrix_from_rpy(0,0,q[0]), np.zeros((3,1))], [np.zeros((1,3)), np.ones((1,1))]]))
    assert full_turn['orientation_travel_so3_rad'] == pytest.approx(2*np.pi)
    assert reversal_counts([[0.],[.1],[.095],[.101],[.07]]) == [1]


def test_actual_rest_start_waits_for_measured_velocity_and_times_out():
    from unloading_sim.m710_replay_physics import RestStartGate
    gate = RestStartGate()
    assert gate.evaluate(0., [0.]*6, [.1]*6, [0.]*6)['hold']
    assert gate.evaluate(.1, [0.]*6, [0.]*6, [0.]*6)['hold']
    assert not gate.evaluate(.21, [0.]*6, [0.]*6, [0.]*6)['hold']
    assert RestStartGate().evaluate(2.1, [.1]*6, [0.]*6, [0.]*6)['reason'] == 'ACTUAL_START_SETTLING_TIMEOUT'


def test_runtime_analytic_reference_and_command_samples_are_bound():
    from unloading_sim.m710_replay_physics import replay_command_arrays, sample_joint_reference
    reference = {'interpolation':'C2_piecewise_quintic_rest_to_rest',
                 'timestamps_seconds':[0.,2.], 'positions_rad':[[0.]*6,[1.]*6]}
    times = np.linspace(0,2,9)
    q, _ = sample_quintic_knots([0.,2.], [[0.]*6,[1.]*6], times)
    bundle = {'timestamps_seconds':times.tolist(),'positions_rad':q.tolist(), 'metadata':{'joint_reference':reference}}
    replay_command_arrays(bundle, range(6))
    actual, v = sample_joint_reference(times, q, .35, reference=reference)
    expected, ev = sample_quintic_knots([0.,2.], [[0.]*6,[1.]*6], .35)
    np.testing.assert_allclose(actual, expected)
    np.testing.assert_allclose(v, ev)
    bundle['positions_rad'][1][0] += .001
    with pytest.raises(ValueError, match='disagree'):
        replay_command_arrays(bundle, range(6))


def test_connection_compares_two_feasible_paths_and_selects_lower_soft_cost():
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
    class Stream:
        calls = 0
        def __next__(self):
            self.calls += 1
            return SimpleNamespace(q=np.full(6, .1*self.calls), position_error=0., orientation_error=0.,
                                   search_evidence={'candidate_id':self.calls})
        def evidence(self):
            return {'seeds_attempted':self.calls, 'iterations_consumed':self.calls}
    class Connector(LayoutTrajectoryConnector):
        def _ik_stream(self, *args, **kwargs):
            self.stream = Stream()
            return self.stream
        def _transit(self, start, goal, *args, **kwargs):
            path = [start, np.ones(6), goal] if goal[0] < .15 else [start, goal]
            return path, None, {'planning_iterations_consumed':1}
    from test_layout_trajectory import _Robot
    robot = _Robot()
    connector = Connector(robot, lambda *a, **kw: None,
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4),
        ik_policy={}, collision_margin_m=.01, contact_tolerance_m=.0002, joint_margin_rad=.01,
        maximum_jacobian_condition=10000, validator_identity='fixture', execution_qualified=True)
    selected, path, failure, evidence = connector._connect_pose(np.eye(4), [np.zeros(6)], np.zeros(6), [],
                                                              ik_seed=0, connection_seed=1, stage='pregrasp', purpose="FREE_APPROACH")
    assert failure is None
    np.testing.assert_allclose(selected, .2)
    assert len(path) == 2 and connector.stream.calls == 2
    assert evidence['attempts'][1]['path_quality']['soft_score'] < evidence['attempts'][0]['path_quality']['soft_score']


def test_reuse_tool_rejects_changed_non_target_obstacle_and_current_release_policy():
    file = Path(__file__).resolve().parents[1]/'tools/reuse_m710_motion.py'
    spec = importlib.util.spec_from_file_location('reuse_motion', file)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    box = OBB([0,0,.775], [.3,.2,.15], np.eye(3), 'target', 'carton')
    belt = OBB([0,0,.3], [1,1,.3], np.eye(3), 'belt', 'conveyor')
    prediction = predict_release(box, [belt], mode=SHORT_DROP_RELEASE)
    segment = {'target':'target', 'path': [[0.]*6], 'release_index':0,
        'place': {'actual_box_pose_world':box.world_from_local.tolist(), 'release_prediction':prediction}}
    from unloading_sim.layout_trajectory import LayoutTrajectoryBudget
    connector = SimpleNamespace(budget=LayoutTrajectoryBudget(maximum_drop_m=.05),
                                post_landing_transport={'mode':'ideal_outfeed'})
    far = OBB([3,0,.775], [.1,.1,.1], np.eye(3), 'other', 'carton')
    actual, _ = module.recheck_current_release(segment, connector, [belt,far], box)
    assert actual['verification_scope'] == 'CURRENT_ENVIRONMENT'
    moved = OBB([0,0,.72], far.half_extents, far.rotation, far.name, far.category)
    with pytest.raises(ValueError, match='release prediction'):
        module.recheck_current_release(segment, connector, [belt,moved], box)
    from dataclasses import replace
    connector.budget = replace(connector.budget, maximum_drop_m=.01)
    with pytest.raises(ValueError, match='release prediction'):
        module.recheck_current_release(segment, connector, [belt,far], box)
