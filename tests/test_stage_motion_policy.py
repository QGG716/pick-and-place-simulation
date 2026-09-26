"""Production dispatch with real validation, IK and bounded RRT, no fake success."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.layout_trajectory import LayoutTrajectoryConnector, LayoutTrajectoryBudget, PhysicalContactAttachment
from unloading_sim.motion_validation import Status
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.stage_motion_policy import MotionPurpose, GenerationMethod, VerifiedTemplate
from unloading_sim.validation_physics import RigidAttachment
from test_extraction_boundary_guard import CartesianRobot, fixture as extraction_fixture
from test_lookahead_contact import START_Q, CONTACT_Q, official_scene


def connector():
    robot = CartesianRobot()
    def collision(q, obstacles, **kwargs):
        carriage = OBB(q[:3], [.005]*3, np.eye(3), 'carriage', 'robot')
        for obstacle in obstacles:
            if carriage.intersects_obb(obstacle, margin=.005):
                return dict(reason='ROBOT_COLLISION', pair=[carriage.name, obstacle.name])
    c = LayoutTrajectoryConnector(robot, collision,
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4),
        ik_policy=dict(position_tolerance_m=1e-5, orientation_tolerance_rad=1e-5,
                       random_restarts=0, max_iterations=100, damping=.005,
                       max_step_rad=.2, orientation_weight=.55,
                       candidate_dedup_tolerance_rad=.01, candidate_dedup_tolerance_m=.001),
        collision_margin_m=.005, contact_tolerance_m=.0002, joint_margin_rad=.01,
        maximum_jacobian_condition=1e4, validator_identity='actual_analytic_carriage',
        execution_qualified=True, budget=LayoutTrajectoryBudget(proof_of_concept=True,
            stage_connection_iterations=200, stage_connection_attempts=3))
    c.start_planning_request()
    return c


START = np.array([0., 0., 1., 0., 0., 0.])
GOAL = START + np.array([.4, 0., 0., 0., 0., 0.])


def obstacle():
    return OBB([.2, 0., 1.], [.025, .06, .06], np.eye(3), 'obstacle', 'wall')


def detour():
    offset = np.array([0., .2, 0., 0., 0., 0.])
    return [START+offset, GOAL+offset]


def connect(c, obstacles, **kwargs):
    return c._transit(START, GOAL, obstacles, purpose=MotionPurpose.FREE_APPROACH,
                      stage='pregrasp', seed=44, iteration_budget=200, **kwargs)


@pytest.mark.parametrize('loaded', [False, True])
def test_real_endpoint_ik_direct_has_no_cartesian_or_rrt(loaded, monkeypatch):
    c = connector()
    attachment = (PhysicalContactAttachment(c.robot, RigidAttachment(np.eye(4), np.full(3,.01), 'payload'),
                    np.eye(4), np.eye(4)) if loaded else None)
    def forbidden(*args, **kwargs):
        pytest.fail('valid joint direct must not generate Cartesian candidates or call RRT')
    monkeypatch.setattr(c, '_local_cartesian_transit', forbidden)
    monkeypatch.setattr(c, '_cartesian', forbidden)
    monkeypatch.setattr(RRTConnectPlanner, 'plan', forbidden)
    q, path, failure, trace = c._connect_pose(c.robot.fk(GOAL), [START], START, [],
        purpose='FREE_LOADED_TRANSFER' if loaded else 'FREE_APPROACH',
        stage='transit' if loaded else 'pregrasp', attachment=attachment,
        ik_seed=3, connection_seed=4, candidate_factory=forbidden)
    assert failure is None and len(path) == 2
    assert trace['selected_method'] == 'JOINT_DIRECT'
    assert trace['endpoint_ik_seed_attempts'] > 0
    assert c._statistics['cartesian_samples'] == c._statistics['rrt_iterations_consumed'] == 0
    assert trace['validation_completed']


def test_history_hint_is_rechecked_with_endpoint_bridges_before_rrt(monkeypatch):
    c = connector(); obstacles = [obstacle()]
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('hint succeeds before RRT'))
    path, failure, trace = connect(c, obstacles, candidates=(
        (GenerationMethod.HISTORY_HINT, lambda: (detour(), None, dict(source='TEST_RECORDED_NODES'))),))
    assert failure is None
    assert trace['selected_method'] == 'HISTORY_HINT' and not trace['rrt_called']
    np.testing.assert_array_equal(path[0], START); np.testing.assert_array_equal(path[-1], GOAL)
    assert trace['attempts'][-1]['endpoint_bridges_checked']
    assert c._motion_validator(obstacles, stage='pregrasp').check_path(path).valid


def test_invalid_hint_falls_through_to_real_rrt_and_same_contract():
    c = connector(); obstacles = [obstacle()]
    path, failure, trace = connect(c, obstacles, candidates=(
        (GenerationMethod.HISTORY_HINT, lambda: ([START, GOAL], None, {})),))
    assert failure is None, trace
    assert trace['selected_method'] == 'RRT_CONNECT'
    assert trace['rrt_constructed'] and trace['rrt_called'] and trace['rrt_expanded']
    assert trace['extension_attempts'] > 0 and trace['planning_iterations_consumed'] > 0
    assert trace['direct_edge_validation']['cache_hit']
    v = c._motion_validator(obstacles, stage='pregrasp')
    assert trace['validation_context'] == v.context.context_id
    assert v.check_path(path).valid


@pytest.mark.parametrize('condition', ['invalid_start', 'invalid_goal', 'unknown', 'cancel', 'budget', 'context'])
def test_abnormal_inputs_do_not_become_rrt_collisions(condition, monkeypatch):
    c = connector(); obstacles = []
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('must not call RRT'))
    if condition.startswith('invalid_'):
        center = START[:3] if condition == 'invalid_start' else GOAL[:3]
        obstacles.append(OBB(center, [.02]*3, np.eye(3), 'blocked', 'wall'))
    elif condition == 'unknown':
        c.robot_state_validator = lambda *a, **k: dict(reason='GEOMETRY_UNKNOWN', classification='UNKNOWN')
    elif condition == 'cancel':
        c.cancel_requested = True
    elif condition == 'budget':
        c.validation_budget.max_checks = 0
    else:
        real = c._motion_validator
        def stale(*a, **k):
            v = real(*a, **k); c.collision_margin_m += .001; return v
        monkeypatch.setattr(c, '_motion_validator', stale)
    path, failure, trace = connect(c, obstacles)
    assert not path and failure
    assert not trace['rrt_called']
    if condition.startswith('invalid_'):
        assert trace['invalid_endpoint'] == condition.removeprefix('invalid_')
    else:
        assert trace['validation_status'] in ('INDETERMINATE', 'CANCELLED')


def test_other_ik_direct_gets_opportunity_before_first_rrt(monkeypatch):
    c = connector(); obstacles = [obstacle()]
    goals = [GOAL, GOAL + np.array([0., .4, 0., 0., 0., 0.])]
    class Stream:
        index = 0
        def __next__(self):
            if self.index == len(goals):raise StopIteration
            q = goals[self.index]; self.index += 1
            return SimpleNamespace(q=q, position_error=0., orientation_error=0., search_evidence={})
        def evidence(self):return dict(seeds_attempted=self.index)
    # Isolate IK scheduling: geometric edges and production dispatcher are real.
    stream = Stream(); monkeypatch.setattr(c, '_ik_stream', lambda *a, **k: stream)
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('second direct succeeds'))
    q, path, failure, trace = c._connect_pose(c.robot.fk(GOAL), [START], START, obstacles,
        purpose='FREE_APPROACH', stage='pregrasp', ik_seed=1, connection_seed=2)
    assert failure is None and stream.index == 2
    np.testing.assert_array_equal(q, goals[1])
    assert [a['pass'] for a in trace['attempts']] == ['JOINT_DIRECT', 'JOINT_DIRECT']


@pytest.mark.parametrize('purpose', [p for p in MotionPurpose if 'FREE' not in p.value] + ['UNKNOWN'])
def test_controlled_and_unknown_purposes_cannot_enter_rrt(purpose, monkeypatch):
    c = connector()
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('controlled process searched'))
    with pytest.raises(ValueError):
        c._transit(START, GOAL, [], purpose=purpose, seed=1, iteration_budget=20, stage='transit')


def test_verified_template_receipt_and_new_equal_binding(monkeypatch):
    c = connector(); a = [obstacle()]
    v = c._motion_validator(a, stage='pregrasp')
    receipt = VerifiedTemplate.capture([START, *detour(), GOAL], v, c._validation_request())
    assert receipt and receipt.candidate(v)[0] == GenerationMethod.VERIFIED_TEMPLATE
    b = deepcopy(a); vb = c._motion_validator(b, stage='pregrasp')
    assert vb is not v and receipt.candidate(vb)[0] == GenerationMethod.HISTORY_HINT
    path, failure, trace = connect(c, a, candidates=(receipt.candidate(v),))
    assert failure is None and trace['selected_method'] == 'VERIFIED_TEMPLATE'
    b[0].center[:] = START[:3]
    assert vb.check_motion(START, GOAL).status == Status.INDETERMINATE
    assert v.check_path(receipt.path).valid
    path, failure, trace = connect(c, b, candidates=(receipt.candidate(vb),))
    assert failure and not path and trace['invalid_endpoint'] == 'start'


def test_official_approach_preserves_free_contact_boundary(official_scene, monkeypatch):
    scene, c = official_scene
    c.start_planning_request(); c._deadline_monotonic=None; c.validation_budget.deadline=None
    old = c.ik['random_restarts']; c.ik['random_restarts']=0
    c.stack_carton_names={b.name for b in scene.cartons}
    target = next(b for b in scene.cartons if b.name == 'carton_l07_c02')
    observed=[]; actual = c._cartesian
    def record(*args, **kwargs):
        observed.append((kwargs['purpose'], kwargs.get('target_contact')))
        return actual(*args, **kwargs)
    monkeypatch.setattr(c, '_cartesian', record)
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('nearby free direct is valid'))
    try:
        c._contact_selection(CONTACT_Q, target, 'front', scene.policy.data['suction'])
        prefix, terminal, failure, trace = c._approach(START_Q, CONTACT_Q,
            c.robot.fk(CONTACT_Q), scene.all_obstacles, target, seed=7)
        assert failure is None
        assert trace['selected_mode'] == 'direct'
        assert len(prefix) == 1 and 0 < trace['terminal_contact_start_index'] < len(terminal)-1
        assert observed == [(MotionPurpose.CONTACT_PROCESS, target)]
        assert trace['attempts'][-1]['search']['connection']['purpose'] == 'FREE_APPROACH'
    finally:
        c.ik['random_restarts']=old


def test_unreleased_tracker_cannot_enter_loaded_transfer(monkeypatch):
    c, start, attachment, tracker, neighbor = extraction_fixture()
    monkeypatch.setattr(c, '_connect_pose', lambda *a, **k: pytest.fail('not released'))
    arguments = dict(target=attachment.box_at(start), face='top', requested_virtual_contact=np.eye(4),
        home_q=start, contact_q=start, physical_contact=np.eye(4), rigid=attachment.rigid,
        attachment=attachment, selection={}, pregrasp=[start], contact=[start], support_release=[start],
        extraction=[start], released_tracker=tracker, payload_obstacles=[neighbor], placement=None,
        selected_supports=[], trace={'stages':{}}, seed=0)
    assert c._finish_place_branch(**arguments)[1]['reason'] == 'ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED'
    tracker.fully_released=True
    assert c._finish_place_branch(**arguments)[1]['reason'] == 'EXTRACTION_EXECUTION_RESERVE_NOT_RETAINED'


def test_all_extraction_routes_keep_real_process_validation_and_isolated_trackers(monkeypatch):
    c, start, attachment, tracker, neighbor = extraction_fixture()
    c.budget = replace(c.budget, proof_of_concept=True)
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('process cannot search'))
    routes = []
    for path, branch, failure, evidence in c._extraction_options(
            start, attachment, [neighbor], tracker, np.array([-1., 0., 0.]), seed=71070):
        assert failure is None and branch.fully_released
        assert c._extraction_reserve_failure(path[-1], attachment, [neighbor]) is None
        assert branch is not tracker and not tracker.fully_released
        attempt = evidence['attempts'][-1]
        routes.append(attempt['search']['route'])
        assert all(part['purpose'] == 'EXTRACTION_PROCESS' for part in attempt['search']['parts'])
        if len(routes) == 3:break
    assert routes == ['straight', 'outward_then_turn', 'coupled_lift_turn']


@pytest.mark.parametrize('purpose,stage', [('CONTACT_PROCESS','contact'),
    ('SUPPORT_RELEASE_PROCESS','support-release'), ('EXTRACTION_PROCESS','extraction'),
    ('PLACEMENT_PROCESS','place'), ('DEPARTURE_PROCESS','withdrawal')])
def test_real_process_failure_never_falls_back_to_rrt(purpose, stage, monkeypatch):
    c = connector()
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('controlled failure searched'))
    path, failure, trace = c._cartesian(START, c.robot.fk(GOAL), [obstacle()],
        purpose=purpose, stage=stage, seed=3)
    assert failure is not None and not trace['rrt_called']
    assert trace['selected_method'] == 'PROCESS_WAYPOINT_CANDIDATE'
    assert trace['along_path_ik_calls'] > 0
    c.cancel_requested = True
    _, failure, trace = c._cartesian(START, c.robot.fk(GOAL), [],
        purpose=purpose, stage=stage, seed=3)
    assert failure['validation']['status'] == 'CANCELLED'
    assert trace['along_path_ik_calls'] == 0


def test_motion_subsegments_preserve_exported_stage_and_event_boundaries():
    from test_layout_trajectory import _segment
    from unloading_sim.stage_motion_policy import motion_subsegments, validate_motion_subsegments
    segment = _segment()
    segment['place']['effective_receiver'] = 'conveyor_transverse'
    segment['motion_subsegments'] = motion_subsegments(segment, {'stages':{}})
    validate_motion_subsegments(segment)
    assert segment['motion_subsegments'][0]['path_range'] == [0, 1]
    # Direct approach exports the free prefix inside contact, without granting
    # terminal-contact permission to that prefix.
    del segment['stage_ranges']['pregrasp']
    segment['stage_ranges']['contact'] = [0, 2]
    segment['approach'] = dict(free_connection_end_index=1, terminal_contact_start_index=1)
    segment['motion_subsegments'] = motion_subsegments(segment, {'stages':{}})
    validate_motion_subsegments(segment)
    assert segment['motion_subsegments'][0]['exported_stage'] == 'contact'
    assert segment['motion_subsegments'][1]['path_range'] == [1, 2]
    segment['motion_subsegments'][5]['path_range'][1] = 5
    with pytest.raises(ValueError):validate_motion_subsegments(segment)


def test_extraction_cancellation_stops_before_next_route(monkeypatch):
    c, start, attachment, tracker, neighbor = extraction_fixture()
    c.budget = replace(c.budget, proof_of_concept=True)
    c.cancel_requested = True
    options = list(c._extraction_options(start, attachment, [neighbor], tracker,
                                        np.array([-1., 0., 0.]), seed=2))
    assert len(options) == 1
    assert options[0][2]['validation']['status'] == 'CANCELLED'
    assert not tracker.fully_released
    assert c._statistics['ik_calls'] == 0


def test_ik_endpoint_unknown_preserves_cause_without_rrt(monkeypatch):
    c = connector()
    c.robot_state_validator = lambda q,*a,**k: (
        dict(reason='ENDPOINT_GEOMETRY_UNKNOWN', classification='UNKNOWN') if q[0] > .3 else None)
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('unknown endpoint searched'))
    _, path, failure, trace = c._connect_pose(c.robot.fk(GOAL), [START], START, [],
        purpose='FREE_APPROACH', stage='pregrasp', ik_seed=1, connection_seed=2)
    assert not path and failure['reason'] == 'ENDPOINT_GEOMETRY_UNKNOWN'
    assert failure['validation']['status'] == 'INDETERMINATE'
    assert trace['endpoint_checks'] and not trace['rrt_called']


def test_production_plan_does_not_restart_a_cancelled_or_stale_branch(monkeypatch):
    c = connector(); calls = []
    failure = dict(reason='VALIDATION_CONTEXT_CHANGED', validation=dict(status='INDETERMINATE'))
    def interrupted(**kwargs):
        calls.append(kwargs)
        return None, failure, {}
    monkeypatch.setattr(c, '_plan_branch', interrupted)
    target=OBB([1.,0.,1.],[.1]*3,np.eye(3),'target','carton')
    result=c.plan(target=target,face='front',requested_virtual_contact=c.robot.fk(GOAL),
        grasp_candidates=[dict(q_rad=GOAL),dict(q_rad=GOAL)],home_q=START,
        all_obstacles=[target],receiver=target,support_names=(),suction={},seed=7)
    assert not result.success and len(calls)==1
    assert result.failure==failure
