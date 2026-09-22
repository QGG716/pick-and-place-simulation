"""Controlled-clock regressions of the production connector, not physics claims."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import unloading_sim.layout_trajectory as lt
import unloading_sim.planner as planning
import unloading_sim.motion_quality as quality
import unloading_sim.ik as ik_module
import unloading_sim.motion_validation as validation_module
from test_layout_trajectory import _Robot, _segment


class Clock:
    now = 100.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock(monkeypatch):
    value = Clock()
    for module in (lt, planning, quality, ik_module, validation_module):
        monkeypatch.setattr(module, "perf_counter", value, raising=False)
    return value


def connector(clock, validator=None):
    c = lt.LayoutTrajectoryConnector(_Robot(), validator or (lambda *a, **k: None),
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4),
        ik_policy=dict(position_tolerance_m=.001, orientation_tolerance_rad=.01),
        collision_margin_m=.01, contact_tolerance_m=.0002, joint_margin_rad=.01,
        maximum_jacobian_condition=1e4, validator_identity="synthetic-clock-fixture",
        execution_qualified=True, budget=lt.LayoutTrajectoryBudget(planning_wall_time_s=20))
    c.start_planning_request(clock())
    return c


def transit(c):
    return c._transit(np.zeros(6), np.full(6, .1), [], seed=3,
                      iteration_budget=10, stage="pregrasp")


def test_shortcut_timeout_retains_independent_strict_baseline(clock, monkeypatch):
    c = connector(clock)
    raw = [np.zeros(6), np.full(6, .2), np.full(6, .1)]
    monkeypatch.setattr(planning.RRTConnectPlanner, "plan", lambda *a, **k:
                        planning.PlanResult(True, raw, 1))
    def shortcut(self, path, *, deadline, **kwargs):
        path[1][:] = 1.5  # A working copy must not mutate the retained path.
        clock.now = deadline
        return [path[0], path[-1]], {"termination": "POSTPROCESS_BUDGET_EXHAUSTED"}
    monkeypatch.setattr(planning.RRTConnectPlanner, "bounded_shortcut", shortcut)
    c._deadline_monotonic = clock() + 10
    path, failure, evidence = transit(c)
    assert failure is None, (failure, evidence)
    np.testing.assert_array_equal(path, raw)
    assert raw[1][0] == .2
    assert evidence["validation_level"] == "B_STRICT_LOCAL_CONNECTION"
    assert evidence["fallback_to_verified"]


def test_second_connection_cannot_spend_terminal_contact_budget(clock, monkeypatch):
    c = connector(clock)
    c._deadline_monotonic = clock() + 2
    class Stream:
        calls = 0
        def __next__(self):
            self.calls += 1
            if self.calls == 2:
                clock.now += 5
            return SimpleNamespace(q=np.full(6, .1), search_evidence={},
                                   position_error=0., orientation_error=0.)
        def evidence(self):
            return {"seeds_attempted": self.calls}
    stream = Stream()
    monkeypatch.setattr(c, "_ik_stream", lambda *a, **k: stream)
    terminal_checks = []
    def terminal(start, goal, obstacles, **kwargs):
        terminal_checks.append(c._path_failure([start, start], obstacles,
            target_contact=kwargs["target_contact"], stage="contact"))
        return [start.copy(), start.copy()], terminal_checks[-1], {}
    monkeypatch.setattr(c, "_cartesian", terminal)
    target = lt.OBB([0, 0, 1], [.3, .2, .15], np.eye(3), "target", "carton")
    result = c._approach(np.zeros(6), np.full(6, .1), np.eye(4), [], target, seed=1)
    assert result[2] is None, result
    assert terminal_checks == [None]
    assert stream.calls == 1


def test_zero_budget_does_not_launch_rrt(clock, monkeypatch):
    c = connector(clock)
    c._deadline_monotonic = clock()
    calls = []
    monkeypatch.setattr(planning.RRTConnectPlanner, "plan", lambda *a, **k:
        calls.append(k) or planning.PlanResult(False, [], 0, "time limit reached"))
    assert transit(c)[1] is not None
    assert calls == []


def test_last_validation_call_overruns_deadline(clock):
    def validator(*args, **kwargs):
        clock.now += 3
    c = connector(clock, validator)
    c._deadline_monotonic = clock() + 1
    failure = c._path_failure([np.zeros(6)], [], stage="pregrasp")
    assert failure is not None
    assert failure["stage"] == "pregrasp"


@pytest.mark.parametrize("mode", ["success", "failure", "exception"])
def test_budget_scope_restores_parent(clock, mode):
    c = connector(clock)
    saved = c._deadline_monotonic
    try:
        with c._budget_scope(clock()+1):
            assert c._deadline_monotonic == clock()+1
            with c._budget_scope(clock()+10):
                assert c._deadline_monotonic == clock()+1
            if mode == "exception":
                raise RuntimeError("injected")
            if mode == "failure":
                clock.now += 2
                assert c._deadline_reached()
    except RuntimeError:
        assert mode == "exception"
    assert c._deadline_monotonic == saved


def test_mid_edge_obstacle_rejects_shortcut_retains_original(clock, monkeypatch):
    def validator(q, *args, **kwargs):
        if .03 < q[0] < .07 and q[1] < .04:
            return {"reason": "INTERIOR_OBSTACLE"}
    c = connector(clock, validator)
    raw = [np.zeros(6), np.array([0., .1, 0, 0, 0, 0]),
           np.array([.1, .1, 0, 0, 0, 0]), np.array([.1, 0, 0, 0, 0, 0])]
    monkeypatch.setattr(planning.RRTConnectPlanner, "plan", lambda *a, **k:
                        planning.PlanResult(True, raw, 1))
    # Inject an invalid proposed shortcut; caller's unchanged edge grid rejects it.
    monkeypatch.setattr(planning.RRTConnectPlanner, "bounded_shortcut", lambda self, path, **k:
                        ([path[0], path[-1]], {"termination": "INJECTED_PROPOSAL"}))
    path, failure, evidence = c._transit(raw[0], raw[-1], [], seed=0,
                                        iteration_budget=10, stage="pregrasp")
    assert failure is None
    np.testing.assert_array_equal(path, raw)
    assert evidence["simplification"]["recheck_failure"]["reason"] == "INTERIOR_OBSTACLE"
    assert evidence["fallback_to_verified"]


def test_unverified_original_cannot_be_retained(clock, monkeypatch):
    c = connector(clock, lambda *a, **k: {"reason": "COLLISION"})
    monkeypatch.setattr(planning.RRTConnectPlanner, "plan", lambda *a, **k:
        planning.PlanResult(True, [np.zeros(6), np.full(6, .1)], 1))
    path, failure, evidence = transit(c)
    assert path == [] and failure["reason"] == "COLLISION"
    assert evidence["validation_level"] == "A_UNVERIFIED_GEOMETRY"


@pytest.mark.parametrize("change", ["target", "mask", "scene", "attachment", "policy", "request"])
def test_retention_context_separates_permissions(clock, change):
    c = connector(clock)
    c.robot_state_validator = SimpleNamespace(commanded_cup_mask=[True]+[False]*71,
        contact_target_name="target", stack_carton_names=set())
    target = lt.OBB([0, 0, 1], [.3, .2, .15], np.eye(3), "target", "carton")
    kwargs = dict(obstacles=[target], target_contact=target, stage="contact")
    initial = c._context_identity(**kwargs)
    if change == "target":
        c.robot_state_validator.contact_target_name = "other"
    elif change == "mask":
        c.robot_state_validator.commanded_cup_mask[1] = True
    elif change == "scene":
        kwargs["obstacles"] = []
    elif change == "attachment":
        from unloading_sim.validation_physics import RigidAttachment
        kwargs["attachment"] = SimpleNamespace(rigid=RigidAttachment(np.eye(4),target.half_extents,target.name))
    elif change == "policy":
        c.collision_margin_m += .001
    else:
        c.start_planning_request(clock())
    # Semantic identity is stable across requests; request-owned evidence is reset.
    assert (initial == c._context_identity(**kwargs)) == (change == 'request')
    if change == 'request':
        assert not c._motion_validators and not c._state_cache


def test_context_change_during_optimization_fails_closed(clock, monkeypatch):
    c = connector(clock)
    def shortcut(self, path, **kwargs):
        c.collision_margin_m += .001
        return path, {}
    monkeypatch.setattr(planning.RRTConnectPlanner, "bounded_shortcut", shortcut)
    path, failure, evidence = transit(c)
    assert path == [] and failure["reason"] == "VALIDATION_CONTEXT_CHANGED"


def test_quality_deadline_is_unknown_and_stops_fk_calls(clock):
    count = []
    def fk(q):
        count.append(q)
        clock.now += .6
        return np.eye(4)
    with pytest.raises(quality.QualityDeadline):
        quality.path_quality([np.zeros(6), np.ones(6)], fk=fk, deadline=clock()+1)
    assert len(count) == 2


def task_input():
    segment = _segment()
    target = lt.OBB([0, 0, 1], [.3, .2, .15], np.eye(3), segment["target"], "carton")
    return dict(target=target, face="top", requested_virtual_contact=np.eye(4),
        grasp_candidates=[dict(candidate_id="candidate", q_rad=[0.]*6)],
        home_q=np.zeros(6), all_obstacles=[target], receiver=target,
        support_names=[], suction={}, seed=1)


@pytest.mark.parametrize("mode", ["on_time", "unfinished", "overrun", "exact_boundary"])
def test_complete_task_return_uses_real_completion_not_return_clock(clock, monkeypatch, mode):
    c = connector(clock)
    kwargs = task_input()
    original_contract = lt.validate_layout_trajectory_stage_contract
    def contract(segment):
        original_contract(segment)
        if mode in ("overrun", "exact_boundary"):
            clock.now = c._branch_final_deadline + (1 if mode == "overrun" else 0)
    monkeypatch.setattr(lt, "validate_layout_trajectory_stage_contract", contract)
    def branch(**args):
        segment = _segment()
        clock.now = c._deadline_monotonic - .01
        if mode == "unfinished":
            return segment, None, {}
        failure = c._finalize_task(segment, args["all_obstacles"], args["target"])
        clock.now = max(clock.now, c._deadline_monotonic + .1)
        return (segment if failure is None else None), failure, {}
    monkeypatch.setattr(c, "_plan_branch", branch)
    outcome = c.plan(**kwargs)
    assert outcome.success == (mode == "on_time"), outcome
    assert not outcome.statistics["result_retention"]["execution_ready"]
    if mode == "on_time":
        proof = outcome.segment["validation"]["completion"]
        assert proof["final_validation_completed"] and proof["completed_monotonic"] < 120
    elif mode == "unfinished":
        assert outcome.failure["reason"] == "COMPLETE_TASK_VALIDATION_MISSING_OR_STALE"
    else:
        assert outcome.failure["reason"] == "FINAL_VALIDATION_DEADLINE"
        assert outcome.failure["completion"]["completed_monotonic"] >= 120


def test_final_reserve_available_only_for_complete_candidate(clock, monkeypatch):
    c = connector(clock)
    kwargs = task_input()
    def branch(**args):
        segment = _segment()
        clock.now = 119.25  # Beyond search=119, before request/candidate hard=120.
        failure = c._finalize_task(segment, args["all_obstacles"], args["target"])
        return segment if failure is None else None, failure, {}
    monkeypatch.setattr(c, "_plan_branch", branch)
    assert c.plan(**kwargs).success
    assert c._deadline_monotonic == 119
    assert transit(c)[1] is not None  # No further RRT gets the final reserve.


def test_optional_validation_exception_is_not_success(clock, monkeypatch):
    c = connector(clock)
    saved = c._deadline_monotonic
    def shortcut(*args, **kwargs):
        raise RuntimeError("exact validation unavailable")
    monkeypatch.setattr(planning.RRTConnectPlanner, "bounded_shortcut", shortcut)
    with pytest.raises(RuntimeError, match="exact validation unavailable"):
        transit(c)
    assert c._deadline_monotonic == saved


def test_shortcut_last_sample_overrun_cannot_publish_edge(clock):
    original = np.array([[0., 0.], [0., .1], [.1, .1], [.1, 0.]])
    def valid(q):
        clock.now += .6
        return True
    p = planning.RRTConnectPlanner(-np.ones(2), np.ones(2), valid, edge_resolution=1.)
    path, report = p.bounded_shortcut(original, deadline=clock()+1)
    np.testing.assert_array_equal(path, original)
    assert report["accepted"] == 0


def test_successful_shortcut_is_still_selected_after_both_checks(clock, monkeypatch):
    c = connector(clock)
    raw = [np.zeros(6), np.full(6, .2), np.full(6, .1)]
    monkeypatch.setattr(planning.RRTConnectPlanner, "plan", lambda *a, **k:
                        planning.PlanResult(True, raw, 1))
    path, failure, evidence = transit(c)
    assert failure is None and len(path) == 2
    assert evidence["simplification"]["reason"] == "FULLY_RECHECKED_IMPROVEMENT"
    assert evidence["simplification"]["after"]["soft_score"] < evidence["simplification"]["before"]["soft_score"]
    assert c._statistics["edge_validation_calls"] == 2
    assert len(raw) == 3 and raw[1][0] == .2


@pytest.mark.parametrize("change", ["path", "mask", "request", "target", "stages"])
def test_task_completion_cannot_be_reused_after_context_or_path_changes(clock, change):
    c = connector(clock)
    c.robot_state_validator = SimpleNamespace(commanded_cup_mask=[True]+[False]*71)
    target = task_input()["target"]
    segment = _segment()
    assert c._finalize_task(segment, [target], target) is None
    assert c._task_completed(segment, [target], target)
    if change == "path":
        segment["path"][0][0] += .001
    elif change == "stages":
        segment["stage_ranges"]["transit"][1] -= 1
    elif change == "mask":
        c.robot_state_validator.commanded_cup_mask[1] = True
    elif change == "request":
        c.start_planning_request(clock())
    else:
        target = lt.OBB(target.center, target.half_extents, target.rotation, "different", "carton")
    assert not c._task_completed(segment, [target], target)


def test_lookahead_caps_cannot_borrow_final_reserve(clock, monkeypatch):
    c = connector(clock)
    target = task_input()["target"]
    c.next_contact_provider = lambda _: [dict(target=target,
        requested_virtual_contact=np.eye(4), face="top", suction={})]
    observed = []
    class Stream:
        def __next__(self):
            observed.append(c._deadline_monotonic-clock())
            clock.now = c._deadline_monotonic
            raise StopIteration
        def evidence(self):
            return {}
    monkeypatch.setattr(c, "_ik_stream", lambda *a, **k: Stream())
    saved = c._deadline_monotonic
    for _ in range(4):
        result = c._next_contact_cost(np.zeros(6), target, [target], [target], seed=0)
        assert result["joint_path_length_rad"] is None
        assert c._deadline_monotonic == saved
    assert observed == [4., 4., 4.]
    assert c._lookahead_remaining_s == 0


def test_safe_departure_survives_optional_timeout(clock, monkeypatch):
    c = connector(clock)
    placed = lt.OBB([.03, 0, 1], [.02]*3, np.eye(3), "placed", "carton")
    tool = lt.OBB([0, 0, 1], [.005]*3, np.eye(3), "tool", "tool")
    c.tool_collision_obbs_provider = lambda _: [tool]
    c.next_contact_provider = lambda _: [dict(target=task_input()["target"],
        requested_virtual_contact=np.eye(4))]
    monkeypatch.setattr(c, "_cartesian", lambda start, *a, **k:
        ([start.copy(), start.copy()+.001], None, {}))
    def next_cost(*a, **k):
        clock.now = c._deadline_monotonic
        return {"status": "BOUNDED_NEXT_CONTACT_SEARCH_FAILED", "joint_path_length_rad": None}
    monkeypatch.setattr(c, "_next_contact_cost", next_cost)
    parent = c._deadline_monotonic
    path, failure, evidence = c._departure(np.zeros(6), placed, [], [1., 0, 0], seed=0,
        working_normal=np.array([1., 0, 0]),
        release_prediction={"flight_time_s": 0, "landing_support": {"receiver_names": ["belt"]}})
    assert failure is None and path
    assert evidence["departure_quality"] is None
    assert evidence["compared_safe_departures"][0]["selection_score"] is None
    assert c._deadline_monotonic == parent


@pytest.mark.parametrize("alternative", ["better", "timeout"])
def test_complete_placement_comparison_keeps_verified_task(clock, monkeypatch, alternative):
    """Exercise the real higher-level comparison loop with synthetic stage inputs."""
    c = connector(clock)
    c.budget = replace(c.budget, planning_wall_time_s=100)
    c.start_planning_request(clock())
    target = task_input()["target"]
    belt = lt.OBB([0, 0, .5], [1, 1, .1], np.eye(3), "belt", "conveyor")
    q = np.zeros(6)
    monkeypatch.setattr(c, "_contact_selection", lambda *a, **k: {})
    monkeypatch.setattr(c, "_approach", lambda *a, **k: ([q], [q], None, {}))
    monkeypatch.setattr(c, "_initial_proximity", lambda *a: (object(), None))
    monkeypatch.setattr(c, "_support_release", lambda *a, **k: ([q], None, {}))
    monkeypatch.setattr(c, "_extraction_options", lambda *a, **k: iter([([q], object(), None, {})]))
    placement = SimpleNamespace(payload=target, receiver_names=["belt"], as_dict=lambda: {})
    monkeypatch.setattr(lt, "generate_conveyor_placements", lambda *a, **k: [placement])
    calls = []
    def finish(**kw):
        calls.append(c._deadline_monotonic)
        if len(calls) == 2 and alternative == "timeout":
            clock.now = c._deadline_monotonic
            return None, {"reason": "INJECTED_ALTERNATIVE_TIMEOUT", "stage": "place"}, kw["trace"]
        segment = _segment()
        segment["post_release_safe_residence"] = {"next_contact": {
            "status": "CHECKED_NEXT_CONTACT_CONNECTION", "joint_path_length_rad": .1}}
        segment["place"]["release_prediction"] = {"flight_time_s": .01}
        # Second candidate has exactly the same checked fixture path but a
        # lower known next-connection cost; retain the existing ranking rule.
        if len(calls) == 2:
            segment["post_release_safe_residence"]["next_contact"]["joint_path_length_rad"] = .01
        failure = c._finalize_task(segment, [belt, target], target)
        return segment if failure is None else None, failure, kw["trace"]
    monkeypatch.setattr(c, "_finish_place_branch", finish)
    outer = c._deadline_monotonic
    segment, failure, trace = c._plan_branch(target=target, face="top",
        requested_virtual_contact=np.eye(4), grasp_q=q, home_q=q,
        all_obstacles=[target, belt], receiver=belt, support_names=[], suction={}, seed=0)
    assert len(calls) == 2  # A blanket postprocess cap must not disable this loop.
    assert failure is None and c._task_completed(segment, [belt, target], target)
    expected = .01 if alternative == "better" else .1
    assert segment["post_release_safe_residence"]["next_contact"]["joint_path_length_rad"] == expected
    assert c._deadline_monotonic == outer


def test_contact_failure_never_becomes_complete_approach(clock, monkeypatch):
    c = connector(clock)
    monkeypatch.setattr(c, "_connect_pose", lambda *a, **k:
        (np.zeros(6), [np.zeros(6)], None, {"validation_level": "B_STRICT_LOCAL_CONNECTION"}))
    monkeypatch.setattr(c, "_cartesian", lambda *a, **k:
        ([], {"reason": "CONTACT_COLLISION", "stage": "contact"}, {}))
    result = c._approach(np.zeros(6), np.zeros(6), np.eye(4), [], task_input()["target"], seed=0)
    assert result[0] == result[1] == []
    assert result[2]["reason"] == "CONTACT_COLLISION"
    assert not c._completed_tasks


# Reuse the previously documented actual near-contact input, original official
# meshes and original full-ring checks. Only time/optional optimization is injected.
from test_lookahead_contact import official_scene, CONTACT_Q, START_Q


def test_official_local_connection_contact_and_retention(official_scene, clock, monkeypatch, tmp_path):
    import json
    scene, c = official_scene
    c.ik["random_restarts"] = 0
    c.start_planning_request(clock())
    c.stack_carton_names = {box.name for box in scene.cartons}
    target = next(box for box in scene.cartons if box.name == "carton_l07_c02")
    candidate = dict(target=target, requested_virtual_contact=c.robot.fk(CONTACT_Q),
        face="front", suction=scene.policy.data["suction"], row_id="fixture_highest_row")
    c.next_contact_provider = lambda _: [candidate]
    placed = lt.OBB([-5, 0, 1], [.3, .2, .15], np.eye(3), "previous_carton", "carton")
    original = planning.RRTConnectPlanner.bounded_shortcut
    injected = []
    def spend_optional(self, path, *, deadline, **kwargs):
        result = original(self, path, deadline=deadline, state_budget=1)
        injected.append(dict(start=clock(), finish=deadline, path_nodes=len(path)))
        clock.now = deadline
        return result
    monkeypatch.setattr(planning.RRTConnectPlanner, "bounded_shortcut", spend_optional)
    result = c._next_contact_cost(START_Q, placed, scene.all_obstacles, [placed], seed=71070)
    assert result["status"] == "CHECKED_NEXT_CONTACT_CONNECTION", result
    assert injected and result["validation_level"] == "C_COMPLETE_APPROACH"
    assert sum(result["commanded_active_mask"]) == 40
    assert result["endpoint_checks"][0]["failure"] is None
    assert not result["execution_ready"]
    assert c._lookahead_remaining_s >= 8
    evidence = dict(result=result, time_injection=injected,
        actual_checks="official FK/IK/Coal collision/full seal rings/free and contact edges",
        optional_injection="clock advanced to shortcut deadline after original bounded shortcut",
        policy=c.collision_policy.to_mapping(), isaac_executed=False)
    (tmp_path / "official_budget_retention.json").write_text(json.dumps(evidence, indent=2))
    print("OFFICIAL_BUDGET_RETENTION", json.dumps(evidence))
