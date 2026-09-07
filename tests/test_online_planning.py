from __future__ import annotations

import json
import hashlib
from pathlib import Path
import zipfile

import pytest

from unloading_sim.online_planning import (
    ContinuousPlanningSession,
    DeterministicAsyncPlanningExecutor,
    ExecutionState,
    PlanStatus,
    PlanningCandidate,
    PlanningPath,
    PlanningRequest,
    PlanningResult,
    PlannerBackend,
    ReplanReason,
    SceneRevision,
    SessionState,
    SynchronousPlanningExecutor,
    normalize_plan_status,
)


class ScriptedBackend(PlannerBackend):
    """Deterministic unit backend: no geometric assumptions enter the tests."""

    def __init__(self, outcomes=None, *, default=PlanStatus.SUCCESS):
        super().__init__(seed=0)
        self.outcomes = {} if outcomes is None else dict(outcomes)
        self.default = default
        self.calls = []

    def plan(self, request, candidate, planning_path):
        self.calls.append((request.request_id, candidate.target_id, candidate.candidate_id, planning_path, self.seed))
        outcome = self.outcomes.get((candidate.candidate_id, planning_path), self.default)
        if isinstance(outcome, Exception):
            raise outcome
        status, latency = (outcome, 0.001) if not isinstance(outcome, tuple) else outcome
        if status is PlanStatus.SUCCESS:
            return PlanningResult.succeeded(
                candidate,
                [request.start_state, tuple(value + 0.01 for value in request.start_state)],
                latency_seconds=latency,
            )
        return PlanningResult.failed(status, candidate, latency_seconds=latency)


def revision(sequence, *cartons):
    return SceneRevision.from_scene({"cartons": list(cartons)}, sequence)


def request(request_id, scene_revision, candidates, *, speculative=False, start=(0.0, 0.0)):
    return PlanningRequest(
        request_id,
        scene_revision,
        start,
        tuple(PlanningCandidate(target, candidate) for target, candidate in candidates),
        seed=17,
        horizon_index=1 if speculative else 0,
        speculative=speculative,
    )


def test_synthetic_feasible_scene_preplans_k_plus_one_and_reuses_matching_revision():
    backend = ScriptedBackend()
    executor = DeterministicAsyncPlanningExecutor()
    session = ContinuousPlanningSession(backend, executor=executor, rolling_horizon=2)
    scene_k = revision(0, "box_0", "box_1")
    predicted_k1 = revision(1, "box_1")

    session.submit(request("k", scene_k, [("box_0", "top")]))
    assert session.state is SessionState.PLANNING
    session.advance()
    plan_k = session.start_execution()
    assert plan_k is not None
    assert session.state is SessionState.EXECUTING

    session.submit_speculative(
        request("k+1", predicted_k1, [("box_1", "top")], speculative=True, start=(0.01, 0.01))
    )
    session.advance()  # Cooperative async planning progresses while k is running.
    assert session.execution.state is ExecutionState.RUNNING
    assert len(session.speculative_plans) == 1

    session.complete_execution(success=True, end_state=(0.01, 0.01), scene_revision=predicted_k1)
    assert session.state is SessionState.READY
    assert not session.speculative_plans
    assert session.ready_plans[0].request.request_id == "k+1"
    assert any(event.kind == "speculative_reused" for event in session.events)


def test_synthetic_scene_change_invalidates_ready_plan_and_replans():
    backend = ScriptedBackend()
    session = ContinuousPlanningSession(backend)
    initial = revision(3, "a", "b")
    changed = revision(4, "a", "b", "unexpected")
    session.submit(request("plan", initial, [("a", "top")]))
    session.run_until_stable()

    session.update_scene(changed)
    assert len(session.invalidated_plans) == 1
    assert session.invalidated_plans[0].invalidated_by is ReplanReason.SCENE_REVISION_CHANGED
    session.run_until_stable()
    assert session.ready_plans[0].planned_revision == changed
    assert session.ready_plans[0].request.replan_reason is ReplanReason.SCENE_REVISION_CHANGED


def test_scene_change_pauses_executing_plan_until_stop_and_validation():
    backend = ScriptedBackend()
    session = ContinuousPlanningSession(backend)
    initial = revision(0, "a")
    changed = revision(1, "a", "intrusion")
    session.submit(request("active", initial, [("a", "top")]))
    session.run_until_stable()
    session.start_execution()

    session.update_scene(changed)

    assert session.execution.state is ExecutionState.PAUSED
    assert session.execution.active_plan.invalidated_by is ReplanReason.SCENE_REVISION_CHANGED
    with pytest.raises(RuntimeError, match="already active"):
        session.start_execution()
    session.stop_invalidated_execution()
    session.run_until_stable()
    assert session.state is SessionState.READY


def test_speculative_plan_is_invalidated_when_observed_scene_differs():
    backend = ScriptedBackend()
    session = ContinuousPlanningSession(backend, rolling_horizon=2)
    scene_k = revision(0, "a", "b")
    predicted = revision(1, "b")
    observed = revision(1, "b", "debris")
    session.submit(request("k", scene_k, [("a", "top")]))
    session.run_until_stable()
    session.start_execution()
    session.submit_speculative(request("k+1", predicted, [("b", "top")], speculative=True))
    session.run_until_stable()

    session.complete_execution(success=True, scene_revision=observed)

    assert any(plan.invalidated_by is ReplanReason.SPECULATIVE_MISMATCH for plan in session.invalidated_plans)
    session.run_until_stable()
    assert session.ready_plans[0].planned_revision == observed


def test_deterministic_unit_scene_uses_fast_warm_cold_and_records_latency():
    outcomes = {
        ("candidate", PlanningPath.FAST): (PlanStatus.NOT_EVALUATED, 0.001),
        ("candidate", PlanningPath.WARM): (PlanStatus.NO_IK, 0.004),
        ("candidate", PlanningPath.COLD): (PlanStatus.SUCCESS, 0.020),
    }
    backend = ScriptedBackend(outcomes)
    session = ContinuousPlanningSession(backend)
    scene = revision(0, "box")
    session.submit(request("paths", scene, [("box", "candidate")]))
    session.run_until_stable()

    assert [call[3] for call in backend.calls] == list(PlanningPath)
    assert session.ready_plans[0].planning_path is PlanningPath.COLD
    report = session.statistics.report()
    assert report["planning_latency"]["count"] == 3
    assert report["planning_latency"]["total_seconds"] == pytest.approx(0.025)
    assert report["planning_latency_by_path"]["FAST"]["max_seconds"] == pytest.approx(0.001)
    assert report["production_throughput"] is None


def test_robot_idle_waiting_for_planner_uses_injected_deterministic_clock():
    class ManualClock:
        value = 0.0

        def __call__(self):
            return self.value

    class ClockedBackend(ScriptedBackend):
        def plan(self, request, candidate, planning_path):
            clock.value += 0.25
            return super().plan(request, candidate, planning_path)

    clock = ManualClock()
    backend = ClockedBackend()
    session = ContinuousPlanningSession(
        backend,
        executor=SynchronousPlanningExecutor(clock),
        clock=clock,
    )
    scene = revision(0, "box")
    session.submit(request("idle", scene, [("box", "top")]))
    session.run_until_stable()

    assert session.statistics.report()["robot_idle_waiting_for_planner_seconds"] == pytest.approx(0.25)


def test_all_candidates_fail_explicitly_blocked_after_candidate_and_target_switches():
    backend = ScriptedBackend(default=PlanStatus.COLLISION)
    session = ContinuousPlanningSession(backend)
    scene = revision(0, "a", "b")
    session.submit(request("blocked", scene, [("a", "a-top"), ("a", "a-side"), ("b", "b-top")]))

    session.run_until_stable()

    assert session.state is SessionState.BLOCKED
    assert not session.ready_plans
    assert len(backend.calls) == 3 * len(PlanningPath)
    decisions = [event.reason for event in session.events if event.kind == "failure_decision"]
    assert "TRY_NEXT_CANDIDATE" in decisions
    assert "TRY_NEXT_TARGET" in decisions
    assert decisions[-1] == "BLOCK"


def test_not_evaluated_waits_for_new_scene_and_backend_exception_enters_recovery():
    scene = revision(0, "a")
    waiting_backend = ScriptedBackend(default=PlanStatus.NOT_EVALUATED)
    waiting = ContinuousPlanningSession(waiting_backend)
    waiting.submit(request("unknown", scene, [("a", "top")]))
    waiting.run_until_stable()
    assert waiting.state is SessionState.WAITING_FOR_SCENE
    waiting_backend.default = PlanStatus.SUCCESS
    waiting.update_scene(revision(1, "a", "new_observation"))
    waiting.run_until_stable()
    assert waiting.state is SessionState.READY

    crashing = ScriptedBackend({("top", PlanningPath.FAST): RuntimeError("planner unavailable")})
    recovery = ContinuousPlanningSession(crashing)
    recovery.submit(request("crash", scene, [("a", "top")]))
    recovery.run_until_stable()
    assert recovery.state is SessionState.RECOVERY


def test_sync_and_async_execution_have_identical_planning_trace():
    scene = revision(0, "box")
    outcomes = {
        ("top", PlanningPath.FAST): PlanStatus.NOT_EVALUATED,
        ("top", PlanningPath.WARM): PlanStatus.SUCCESS,
    }
    traces = []
    for executor in (SynchronousPlanningExecutor(), DeterministicAsyncPlanningExecutor()):
        backend = ScriptedBackend(outcomes)
        session = ContinuousPlanningSession(backend, executor=executor)
        session.submit(request("deterministic", scene, [("box", "top")]))
        session.run_until_stable()
        traces.append([(event.kind, event.reason) for event in session.events])
    assert traces[0] == traces[1]


def test_legacy_failure_status_mapping_is_fail_closed():
    assert normalize_plan_status("PREGRASP_NO_IK") is PlanStatus.NO_IK
    assert normalize_plan_status("PAYLOAD_INITIAL_CLEARANCE_FAILED") is PlanStatus.INITIAL_CLEARANCE_FAILED
    assert normalize_plan_status("ROBOT_COLLISION") is PlanStatus.COLLISION
    assert normalize_plan_status("GRASP_CONSTRAINT_FAILED") is PlanStatus.GRASP_CONSTRAINT_FAILED
    assert normalize_plan_status("unrecognized_new_failure") is PlanStatus.NOT_EVALUATED


class BottomAlternativeEvidenceBackend(PlannerBackend):
    """Integration adapter for the independently recorded V3 successful case."""

    def __init__(self, fixture, evidence_result):
        super().__init__(fixture["request"]["seed"])
        self.fixture = fixture
        self.evidence_result = evidence_result

    def plan(self, request, candidate, planning_path):
        recorded = self.fixture["recorded_result"]
        evidence = self.evidence_result
        if planning_path is not PlanningPath.FAST:
            return PlanningResult.failed(PlanStatus.NOT_EVALUATED, candidate)
        assert evidence["geometric_feasible"] and evidence["failure_reason"] == "OK"
        assert evidence["selected"]["face"] == recorded["face"]
        assert evidence["selected"]["roll_deg"] == recorded["roll_deg"]
        assert evidence["load_status"] == recorded["payload_qualification"]
        trajectory = evidence["selected"]["trajectory"]["q_knots"]
        return PlanningResult.succeeded(
            candidate,
            trajectory,
            latency_seconds=0.0005,
            metadata={"evidence_member": self.fixture["evidence"]["member"]},
        )


def test_v3_bottom_alternative_is_real_m710_integration_fixture_not_acceptance_rate():
    fixture_path = Path(__file__).parent / "fixtures" / "m710id70_v3_bottom_alternative.json"
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    assert fixture["robot_model"] == "fanuc_m710id_70"
    assert "not a continuous-unload or throughput claim" in fixture["scope"]
    assert fixture["recorded_result"]["placement_penetration_m"] <= fixture["recorded_result"]["explicit_contact_tolerance_m"]
    repository = Path(__file__).parents[1]
    archive = repository / fixture["evidence"]["archive"]
    digest = hashlib.sha256()
    with archive.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    assert digest.hexdigest() == fixture["evidence"]["archive_sha256"]
    with zipfile.ZipFile(archive) as evidence_zip:
        evidence = json.loads(evidence_zip.read(fixture["evidence"]["member"]))["result"]
    assert evidence["geometric_feasible"] is True
    assert evidence["selected"]["loaded_tcp_path_m"] == pytest.approx(
        fixture["recorded_result"]["loaded_tcp_path_m"], abs=1e-6
    )

    revision0 = SceneRevision.from_scene(fixture["scene"], 0, source="v3_recorded_evidence")
    data = fixture["request"]
    planning_request = PlanningRequest(
        "v3-bottom-alternative",
        revision0,
        tuple(data["start_state_rad"]),
        (PlanningCandidate(data["target_id"], data["candidate_id"]),),
        seed=data["seed"],
        metadata={"evidence_archive_sha256": fixture["evidence"]["archive_sha256"]},
    )
    session = ContinuousPlanningSession(BottomAlternativeEvidenceBackend(fixture, evidence))
    session.submit(planning_request)
    session.run_until_stable()

    assert session.state is SessionState.READY
    assert session.ready_plans[0].candidate.candidate_id == "top_roll_270"
    assert len(session.ready_plans[0].result.trajectory) == 57
    metrics = session.statistics.report()
    assert metrics["production_throughput"] is None
    assert metrics["planning_latency"]["count"] == 1
