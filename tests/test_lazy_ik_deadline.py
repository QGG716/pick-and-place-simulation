"""Deadline propagation through the real lazy stream; no physical execution claims."""
from copy import deepcopy
from dataclasses import replace
import json

import numpy as np
import pytest

import unloading_sim.ik as ik
import unloading_sim.layout_trajectory as lt
from test_feasible_result_budget import clock, connector, task_input


def started_comparison(clock, monkeypatch, *, mode="timeout"):
    checks, calls, streams = [], [], []
    def validator(q, obstacles, **kwargs):
        checks.append(dict(stage=kwargs["stage"], time=clock()))
        return {"reason": "INJECTED_CONTACT_FAILURE"} if mode == "contact_failure" and kwargs["stage"] == "contact" else None
    validator.commanded_cup_mask = [True] + [False]*71
    validator.contact_target_name = "target"
    validator.stack_carton_names = {"neighbor"}
    c = connector(clock, validator)
    c.budget = replace(c.budget, approach_mode="direct")
    c._deadline_monotonic = 106.
    c.ik.update(random_restarts=2, max_iterations=20, damping=.01, max_step_rad=.1,
        candidate_dedup_tolerance_rad=.001, candidate_dedup_tolerance_m=.0001,
        orientation_weight=1.)
    factory = lt.iter_ik_solutions
    def record_stream(*args, **kwargs):
        result = factory(*args, **kwargs)
        assert isinstance(result, ik.IKCandidateStream)
        streams.append(result)
        return result
    monkeypatch.setattr(lt, "iter_ik_solutions", record_stream)
    pregrasp_calls = []
    def solve(robot, target, seed, **kwargs):
        endpoint = kwargs.get("extra_state_valid")
        record = dict(stage="pregrasp" if endpoint is not None else "contact",
            outer_deadline=c._deadline_monotonic,
            solver_deadline=kwargs.get("deadline_monotonic"), started=clock())
        calls.append(record)
        q = np.zeros(6)
        q[:3] = target[:3, 3]
        if endpoint is not None:
            pregrasp_calls.append(record)
            if len(pregrasp_calls) > 1 and mode != "better":
                clock.now = kwargs["deadline_monotonic"]
                record.update(stopped=clock(), reason="shared wall-clock deadline reached")
                return ik.IKResult(False, q, 5, .1, .1, record["reason"])
            q[5] = .4 if len(pregrasp_calls) == 1 else .1
        clock.now += .2 if endpoint is not None else .1
        accepted = endpoint is None or endpoint(q)
        record.update(stopped=clock(), reason="converged" if accepted else "endpoint rejected")
        return ik.IKResult(accepted, q, 1, 0., 0., record["reason"])
    monkeypatch.setattr(ik, "solve_ik", solve)  # Only the timed numerical boundary.
    saved = deepcopy((validator.commanded_cup_mask, validator.contact_target_name,
                      validator.stack_carton_names))
    result = c._approach(np.zeros(6), np.full(6, .1), np.eye(4), [], task_input()["target"], seed=17)
    search = result[3]["attempts"][0]["search"]["connection"]
    evidence = dict(calls=calls, stream_original_deadline=streams[0].kwargs.get("deadline_monotonic"),
        stream_evidence=streams[0].evidence(), connection=search,
        parent_deadline_restored=c._deadline_monotonic,
        terminal_checks=[x for x in checks if x["stage"] == "contact"],
        approach_failure=result[2], approach_completed=result[2] is None and bool(result[1]),
        context_restored=saved == (validator.commanded_cup_mask, validator.contact_target_name,
                                  validator.stack_carton_names),
        first_connection_completed=next(x["completed_monotonic"] for x in c._budget_events
                                       if x["validation_level"] == "B_STRICT_LOCAL_CONNECTION"),
        selected_q=None if not result[1] else result[1][result[3]["free_connection_end_index"]].tolist())
    return result, evidence


def test_started_second_ik_stops_in_optional_window_and_contact_completes(clock, monkeypatch, tmp_path):
    result, evidence = started_comparison(clock, monkeypatch)
    (tmp_path / "lazy_ik_deadline.json").write_text(json.dumps(evidence, indent=2))
    pregrasp = [x for x in evidence["calls"] if x["stage"] == "pregrasp"]
    assert [x["solver_deadline"] for x in pregrasp] == [106., 103.], evidence
    assert pregrasp[1]["outer_deadline"] == 103.
    assert pregrasp[1]["stopped"] == 103.
    assert evidence["stream_original_deadline"] == 106.
    assert evidence["stream_evidence"]["seeds_attempted"] == 2
    assert evidence["stream_evidence"]["termination"] == "PLANNING_WALL_CLOCK_DEADLINE"
    assert evidence["parent_deadline_restored"] == 106.
    assert evidence["terminal_checks"] and evidence["approach_completed"]
    assert evidence["first_connection_completed"] == pytest.approx(100.2)
    assert evidence["context_restored"]
    assert evidence["connection"]["optional_comparison"]["termination"] == "PLANNING_WALL_CLOCK_DEADLINE"
    assert not evidence["connection"]["optional_comparison_skipped"]
    assert result[2] is None


@pytest.mark.parametrize("mode", ["contact_failure", "better"])
def test_started_comparison_preserves_contact_failure_and_better_choice(clock, monkeypatch, mode):
    result, evidence = started_comparison(clock, monkeypatch, mode=mode)
    assert evidence["context_restored"] and evidence["parent_deadline_restored"] == 106.
    assert evidence["terminal_checks"]
    if mode == "contact_failure":
        assert not evidence["approach_completed"]
        assert result[2]["reason"] == "INJECTED_CONTACT_FAILURE"
        assert result[0] == result[1] == []
    else:
        assert evidence["approach_completed"]
        assert evidence["selected_q"][5] == .1
        assert len(evidence["connection"]["attempts"]) == 2


class LinearRobot:
    joint_limits = np.array([[-2., 2.]])

    def clamp(self, q):
        return np.clip(q, -2., 2.)

    def fk(self, q):
        pose = np.eye(4)
        pose[0, 3] = q[0]
        return pose

    def geometric_jacobian(self, q):
        j = np.zeros((6, 1))
        j[0, 0] = 1.
        return j


@pytest.mark.parametrize("late", ["initial_fk", "jacobian", "linear_solve", "final_fk", "endpoint", "final_endpoint"])
def test_real_solver_rejects_last_uninterruptible_call_overrun(clock, monkeypatch, late):
    events = []
    class TimedRobot(LinearRobot):
        fk_calls = 0
        def fk(self, q):
            self.fk_calls += 1
            events.append("fk")
            result = super().fk(q)
            if (late == "initial_fk" and self.fk_calls == 1
                    or late == "final_fk" and self.fk_calls == 2):
                clock.now = 101.25
            return result
        def geometric_jacobian(self, q):
            events.append("jacobian")
            if late == "jacobian":
                clock.now = 101.25
            return super().geometric_jacobian(q)
    solve = np.linalg.solve
    def timed_solve(*args):
        events.append("linear_solve")
        result = solve(*args)
        if late == "linear_solve":
            clock.now = 101.25
        return result
    monkeypatch.setattr(np.linalg, "solve", timed_solve)
    def endpoint(q):
        events.append("endpoint")
        if late in {"endpoint", "final_endpoint"}:
            clock.now = 101.25
        return True
    target = np.eye(4)
    target[0, 3] = 0. if late in {"initial_fk", "endpoint"} else .5
    result = ik.solve_ik(TimedRobot(), target, np.zeros(1), max_iterations=1,
        damping=1e-6, max_step=1., position_tolerance=1e-6,
        extra_state_valid=endpoint, deadline_monotonic=101.)
    assert not result.success, (late, result, events)
    assert result.message.startswith("shared wall-clock deadline")
    assert events[-1] == {"initial_fk": "fk", "final_fk": "fk", "final_endpoint": "endpoint"}.get(late, late)
    assert result.search_evidence["stopped_monotonic"] == 101.25


def test_real_solver_observes_dynamic_tightening_inside_iteration(clock):
    current = [106.]
    class Robot(LinearRobot):
        def geometric_jacobian(self, q):
            current[0] = 100.5
            clock.now = 100.5
            return super().geometric_jacobian(q)
    target = np.eye(4)
    target[0, 3] = .5
    result = ik.solve_ik(Robot(), target, np.zeros(1), deadline_monotonic=106.,
                         deadline_provider=lambda: current[0])
    assert not result.success and result.iterations == 1
    assert result.search_evidence["deadline_monotonic"] == 100.5


def make_stream(c, count=5):
    c.ik.update(random_restarts=0, max_iterations=20, damping=.01, max_step_rad=.1,
        candidate_dedup_tolerance_rad=.001, candidate_dedup_tolerance_m=.0001,
        orientation_weight=1.)
    return c._ik_stream(np.eye(4), [np.full(6, i*.1) for i in range(count)], [],
        seed=1, attachment=None, support_names=(), target_contact=None, stage="pregrasp")


def test_one_next_uses_current_window_for_all_seeds_and_can_resume_without_replay(clock, monkeypatch):
    c = connector(clock)
    c._deadline_monotonic = 106.
    stream = make_stream(c)
    received = []
    def solve(robot, target, seed, **kwargs):
        received.append(kwargs["deadline_monotonic"])
        index = round(seed[0]*10)
        clock.now += .1
        if index == 3:
            clock.now = kwargs["deadline_monotonic"]
        valid = index in {0, 1, 4}
        q = np.zeros(6) if index == 1 else seed.copy()
        return ik.IKResult(valid, q, 2, 0. if valid else .1, 0.,
            "converged" if valid else "shared wall-clock deadline reached" if index == 3 else "rejected")
    monkeypatch.setattr(ik, "solve_ik", solve)
    first = next(stream)
    with c._budget_scope(103.):
        with pytest.raises(StopIteration):
            next(stream)
    assert received == [106., 103., 103., 103.]
    evidence = stream.evidence()
    assert evidence["seeds_attempted"] == 4 and evidence["iterations_consumed"] == 8
    assert evidence["duplicate_candidates"] == 1 and evidence["deduplicated_candidates"] == 1
    assert not evidence["seed_stream_exhausted"]
    assert evidence["termination"] == "PLANNING_WALL_CLOCK_DEADLINE"
    assert stream.kwargs["deadline_monotonic"] == 106.
    second = next(stream)
    np.testing.assert_array_equal(first.q, np.zeros(6))
    np.testing.assert_array_equal(second.q, np.full(6, .4))
    assert received == [106., 103., 103., 103., 106.]
    with pytest.raises(StopIteration):
        next(stream)
    assert stream.evidence()["termination"] == "SEED_STREAM_EXHAUSTED"


@pytest.mark.parametrize("exit_mode", ["normal", "stop", "failure", "exception"])
def test_stream_observes_restored_scope_without_resetting_cursor(clock, monkeypatch, exit_mode):
    c = connector(clock)
    c._deadline_monotonic = 106.
    stream = make_stream(c)
    calls = []
    def solve(robot, target, seed, **kwargs):
        calls.append(seed.tolist())
        if exit_mode == "exception":
            raise RuntimeError("injected numeric failure")
        if exit_mode == "failure":
            clock.now = 101.
            return ik.IKResult(False, seed.copy(), 1, .1, .1, "shared wall-clock deadline reached")
        return ik.IKResult(True, seed.copy(), 1, 0., 0., "converged")
    monkeypatch.setattr(ik, "solve_ik", solve)
    try:
        with c._budget_scope(0. if exit_mode == "stop" else 101.):
            next(stream)
    except StopIteration:
        assert exit_mode in {"stop", "failure"}
    except RuntimeError:
        assert exit_mode == "exception"
    assert c._deadline_monotonic == stream.effective_deadline() == 106.
    assert stream.kwargs["deadline_monotonic"] == 106.
    assert stream.seed_index == len(calls) == (0 if exit_mode == "stop" else 1)
    with c._budget_scope(104.):
        with c._budget_scope(110.):
            assert stream.effective_deadline() == 104.
    with c._budget_scope(None):
        assert stream.effective_deadline() == 106.
    c._request_deadline_monotonic = 105.
    assert stream.effective_deadline() == 105.
    c._deadline_monotonic = None
    c._request_deadline_monotonic = None
    assert stream.effective_deadline() == 106.  # Immutable original parent cap.


@pytest.mark.parametrize("deadline", [None, 0., 99., 101.])
def test_fixed_deadline_callers_remain_compatible(clock, deadline):
    stream = ik.iter_ik_solutions(LinearRobot(), np.eye(4), [np.zeros(1)],
        random_restarts=0, deadline_monotonic=deadline)
    if deadline is not None and deadline <= clock():
        with pytest.raises(StopIteration):
            next(stream)
        assert stream.seed_index == 0
        assert stream.termination == "PLANNING_WALL_CLOCK_DEADLINE"
    else:
        assert next(stream).success and stream.seed_index == 1
