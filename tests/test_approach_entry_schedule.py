"""Existing entry scheduling with real IK, geometry and production connection."""
import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.stage_motion_policy import failure_status, interrupts_generation
from test_stage_motion_policy import connector, START, GOAL


def scene():
    # The near entry crosses the wall; the existing 100 mm entry goes below it.
    wall = OBB([.2, 0, 1], [.025, .02, .012], np.eye(3), 'wall', 'wall')
    target = OBB([.4, 0, 1.1], [.02]*3, np.eye(3), 'target', 'carton')
    return [wall], target


def test_adaptive_entry_gets_real_direct_opportunity_before_rrt(monkeypatch):
    c = connector(); obstacles, target = scene()
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('adaptive direct succeeds'))
    prefix, terminal, failure, trace = c._approach(START, GOAL, c.robot.fk(GOAL),
        obstacles, target, seed=7)
    assert failure is None, trace
    assert trace['selected_mode'] == 'adaptive_pregrasp'
    assert all(a['connection_pass'] == 'JOINT_DIRECT' for a in trace['attempts'])
    assert trace['attempts'][0]['failure'] is not None
    np.testing.assert_array_equal(prefix[0], START)
    np.testing.assert_array_equal(prefix[-1], terminal[0])
    np.testing.assert_allclose(c.robot.fk(terminal[-1]), c.robot.fk(GOAL), atol=1e-5)
    assert c._motion_validator(obstacles, stage='pregrasp').check_path(prefix).valid
    assert c._motion_validator(obstacles, target_contact=target, stage='contact').check_path(terminal).valid


@pytest.mark.parametrize('kind', ['cancel', 'work', 'context'])
def test_request_stop_between_entries_does_not_search(monkeypatch, kind):
    c = connector(); obstacles, target = scene(); actual = c._transit; calls=[]
    def stop_after_first(*a, **k):
        result = actual(*a, **k); calls.append(result)
        if len(calls) == 1:
            if kind == 'cancel': c.cancel_requested = True
            elif kind == 'work': c.validation_budget.max_checks = c.validation_budget.checks
            else: obstacles[0].center[1] += .001
        return result
    monkeypatch.setattr(c, '_transit', stop_after_first)
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('request invalid'))
    prefix, terminal, failure, trace = c._approach(START, GOAL, c.robot.fk(GOAL),
        obstacles, target, seed=7)
    assert not prefix and not terminal
    assert failure_status(failure) == ('CANCELLED' if kind == 'cancel' else 'INDETERMINATE')
    assert interrupts_generation(failure)


def test_direct_only_pose_defers_search_without_claiming_unreachable(monkeypatch):
    c = connector(); obstacles, _ = scene()
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('deferred'))
    _, path, failure, trace = c._connect_pose(c.robot.fk(GOAL), [START], START, obstacles,
        stage='pregrasp', purpose='FREE_APPROACH', ik_seed=7, connection_seed=8, allow_rrt=False)
    assert not path and failure_status(failure) == 'INDETERMINATE'
    assert not interrupts_generation(failure)
    assert not trace['rrt_called']


def test_clearance_candidate_uses_full_tool_and_free_edges(monkeypatch):
    c = connector()
    stack = OBB([.2, 0, 1], [.025, .02, .012], np.eye(3), 'stack', 'carton')
    c.tool_collision_obbs_provider = lambda q: [OBB(q[:3], [.02]*3, np.eye(3), 'whole_tool', 'tool')]
    obstacles = [stack]
    observed = []; cartesian = c._cartesian
    def record(*a, **k):
        observed.append(k)
        return cartesian(*a, **k)
    monkeypatch.setattr(c, '_cartesian', record)
    monkeypatch.setattr(RRTConnectPlanner, 'plan', lambda *a, **k: pytest.fail('local candidate cannot search'))
    path, failure, trace = c._approach_clearance_candidate(START, GOAL, c.robot.fk(GOAL),
        obstacles, seed=7, sample_budget=240)
    assert failure is None and trace['validation_completed']
    assert trace['full_tool_radius_m'] == pytest.approx(np.sqrt(3)*.02)
    assert observed and all(k['stage'] == 'pregrasp' and k.get('target_contact') is None for k in observed)
    assert trace['cartesian_samples'] <= 240
    assert c._motion_validator(obstacles, stage='pregrasp').check_path(path).valid
    np.testing.assert_array_equal(path[0], START)
    np.testing.assert_allclose(c.robot.fk(path[-1]), c.robot.fk(GOAL), atol=1e-5)
    _, failure, limited = c._approach_clearance_candidate(START, GOAL, c.robot.fk(GOAL),
        obstacles, seed=7, sample_budget=1)
    assert failure_status(failure) == 'INDETERMINATE' and not interrupts_generation(failure)
    assert limited['cartesian_samples'] <= 1
