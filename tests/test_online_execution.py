from __future__ import annotations

from dataclasses import FrozenInstanceError, replace

import pytest

from unloading_sim.online_execution import (
    DeterministicSimExecutionBackend,
    ExecutionBackendHealth,
    ExecutionBackendState,
    ExecutionCommandStatus,
    ExecutionFeedback,
    ExecutionFeedbackStatus,
)
from unloading_sim.online_planning import (
    BoundaryMode,
    BackendCapabilities,
    ContinuousPlanningSession,
    MotionBoundaryState,
    PlanArtifactKind,
    PlanStatus,
    PlanningCandidate,
    PlanningRequest,
    PlanningResult,
    PlanningWorldSnapshot,
    PlannerBackend,
    RobotStateRevision,
    SceneRevision,
    SessionState,
)


class FeasibleBackend(PlannerBackend):
    def __init__(self):
        super().__init__()
        self.requests = []

    def plan(self, request, candidate, planning_path):
        del planning_path
        self.requests.append(request)
        return PlanningResult.succeeded(candidate, (request.start_state, (0.2, -0.1)))


class ContinuousBackend(PlannerBackend):
    def __init__(self):
        super().__init__()
        self.capabilities = BackendCapabilities(
            frozenset({path for path in self.capabilities.supported_planning_paths}),
            frozenset({BoundaryMode.CONTINUOUS_BOUNDARY}),
            frozenset({PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY}),
            supports_warm_start=True,
            supports_deterministic_seed=True,
            supports_attached_object=True,
            supports_revalidation=True,
        )

    def plan(self, request, candidate, planning_path):
        del planning_path
        start = request.motion_boundary
        end = MotionBoundaryState(
            (0.2, -0.1),
            (0.2, -0.1),
            (0.01, 0.02),
            start.time_seconds + 1.0,
            BoundaryMode.CONTINUOUS_BOUNDARY,
        )
        return PlanningResult.succeeded(
            candidate,
            (start.q, end.q),
            artifact_kind=PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY,
            expected_start_boundary=start,
            expected_end_boundary=end,
        )


def world() -> PlanningWorldSnapshot:
    scene = {"cartons": ["a"]}
    return PlanningWorldSnapshot(
        SceneRevision.from_scene(scene, 0),
        scene,
        RobotStateRevision(0, (0.0, 0.0), {"mode": "AUTO"}),
        {"id": "tool"},
        None,
        {"x": 0.0},
        {"extension": 0.0},
        {"id": "execution-test"},
    )


def executable_plan():
    snapshot = world()
    session = ContinuousPlanningSession(FeasibleBackend())
    session.submit(
        PlanningRequest(
            "plan",
            snapshot,
            (PlanningCandidate("a", "top"),),
        )
    )
    session.run_until_stable()
    assert session.ready_plans[0].result.status is PlanStatus.SUCCESS
    return session.ready_plans[0]


def test_execution_feedback_is_immutable_and_validates_fields():
    boundary = MotionBoundaryState.stopped((0.0, 0.0))
    metadata = {"nested": ["value"]}
    feedback = ExecutionFeedback(
        1,
        "execution",
        "plan",
        ExecutionFeedbackStatus.RUNNING,
        0.25,
        boundary,
        metadata=metadata,
        observed_at_monotonic_seconds=1.0,
    )
    metadata["nested"].append("changed")

    assert feedback.metadata["nested"] == ("value",)
    with pytest.raises(FrozenInstanceError):
        feedback.progress = 0.5
    with pytest.raises(ValueError, match="progress"):
        ExecutionFeedback(1, "e", "p", ExecutionFeedbackStatus.RUNNING, float("nan"), boundary)
    with pytest.raises(ValueError, match="sequence"):
        ExecutionFeedback(-1, "e", "p", ExecutionFeedbackStatus.RUNNING, 0.0, boundary)
    with pytest.raises(ValueError, match="monotonic"):
        ExecutionFeedback(
            1,
            "e",
            "p",
            ExecutionFeedbackStatus.RUNNING,
            0.0,
            boundary,
            observed_at_monotonic_seconds=-1.0,
        )


def test_stopped_and_succeeded_feedback_contracts_are_explicit():
    stopped = MotionBoundaryState.stopped((0.0, 0.0))
    continuous = MotionBoundaryState(
        (0.0, 0.0),
        (0.1, 0.0),
        (0.0, 0.0),
        0.5,
        BoundaryMode.CONTINUOUS_BOUNDARY,
    )
    with pytest.raises(ValueError, match="STOP_BOUNDARY"):
        ExecutionFeedback(1, "e", "p", ExecutionFeedbackStatus.STOPPED, 0.5, continuous)
    with pytest.raises(ValueError, match="progress 1.0"):
        ExecutionFeedback(1, "e", "p", ExecutionFeedbackStatus.SUCCEEDED, 0.9, stopped)


def test_sim_backend_success_is_deterministic_and_poll_consumes_once():
    plan = executable_plan()
    backend = DeterministicSimExecutionBackend(execution_steps=2)
    command = backend.start(plan)
    assert command.status is ExecutionCommandStatus.ACCEPTED
    assert backend.state is ExecutionBackendState.RUNNING

    feedback = [backend.poll()]
    for _ in range(3):
        assert backend.advance() == 1
        feedback.append(backend.poll())

    assert [item.status for item in feedback] == [
        ExecutionFeedbackStatus.ACCEPTED,
        ExecutionFeedbackStatus.RUNNING,
        ExecutionFeedbackStatus.RUNNING,
        ExecutionFeedbackStatus.SUCCEEDED,
    ]
    assert [item.feedback_sequence for item in feedback] == [1, 2, 3, 4]
    assert [item.progress for item in feedback] == [0.0, 0.0, 0.5, 1.0]
    assert feedback[-1].current_boundary.matches(plan.expected_end_boundary)
    assert backend.poll() is None


@pytest.mark.parametrize(
    "terminal_status",
    [ExecutionFeedbackStatus.FAILED, ExecutionFeedbackStatus.DEVIATED],
)
def test_sim_backend_supports_failed_and_deviated_terminal_feedback(terminal_status):
    backend = DeterministicSimExecutionBackend(execution_steps=1, terminal_status=terminal_status)
    assert backend.start(executable_plan()).accepted
    assert backend.poll().status is ExecutionFeedbackStatus.ACCEPTED
    backend.advance(2)
    feedback = []
    while (item := backend.poll()) is not None:
        feedback.append(item)
    assert feedback[-1].status is terminal_status


def test_sim_backend_stop_is_idempotent_and_returns_complete_stopped_boundary():
    plan = executable_plan()
    backend = DeterministicSimExecutionBackend(execution_steps=3)
    start = backend.start(plan)
    assert backend.poll().status is ExecutionFeedbackStatus.ACCEPTED
    backend.advance(2)
    while backend.poll() is not None:
        pass

    first = backend.request_stop(plan.plan_id, "scene changed")
    second = backend.request_stop(plan.plan_id, "scene changed again")
    assert first.status is ExecutionCommandStatus.ACCEPTED
    assert second.status is ExecutionCommandStatus.ALREADY_STOPPING
    assert first.execution_id == start.execution_id

    backend.advance()
    stopping = backend.poll()
    backend.advance()
    stopped = backend.poll()
    assert stopping.status is ExecutionFeedbackStatus.STOPPING
    assert stopped.status is ExecutionFeedbackStatus.STOPPED
    assert stopped.current_boundary.boundary_mode is BoundaryMode.STOP_BOUNDARY
    assert stopped.current_boundary.qd == (0.0, 0.0)
    assert stopped.current_boundary.qdd == (0.0, 0.0)


def test_sim_backend_rejection_duplicate_start_and_shutdown_are_fail_closed():
    plan = executable_plan()
    rejecting = DeterministicSimExecutionBackend(reject_start=True)
    assert rejecting.start(plan).status is ExecutionCommandStatus.REJECTED

    backend = DeterministicSimExecutionBackend(execution_steps=1)
    assert backend.start(plan).accepted
    backend.advance(2)
    while backend.poll() is not None:
        pass
    assert backend.start(plan).status is ExecutionCommandStatus.REJECTED
    backend.shutdown()
    backend.shutdown()
    assert backend.health is ExecutionBackendHealth.SHUTDOWN
    assert backend.state is ExecutionBackendState.SHUTDOWN
    assert backend.start(plan).status is ExecutionCommandStatus.BACKEND_UNAVAILABLE


def test_complete_execution_records_actual_boundary_and_waits_for_real_scene():
    snapshot = world()
    session = ContinuousPlanningSession(FeasibleBackend())
    session.submit(PlanningRequest("actual", snapshot, (PlanningCandidate("a", "top"),)))
    session.run_until_stable()
    plan = session.start_execution()

    session.complete_execution(success=True, actual_end_boundary=plan.expected_end_boundary)

    assert session.state is SessionState.WAITING_FOR_SCENE
    assert session.last_actual_execution_boundary == plan.expected_end_boundary
    assert session.current_motion_boundary == plan.expected_end_boundary
    assert session.terminal_execution_identity == (None, plan.plan_id)


def test_success_with_actual_continuous_boundary_mismatch_is_execution_deviation():
    snapshot = world()
    start = MotionBoundaryState(
        snapshot.current_q,
        (0.1, 0.0),
        (0.0, 0.0),
        0.0,
        BoundaryMode.CONTINUOUS_BOUNDARY,
    )
    session = ContinuousPlanningSession(ContinuousBackend(), rolling_horizon=2)
    session.submit(
        PlanningRequest(
            "continuous-k",
            snapshot,
            (PlanningCandidate("a", "top"),),
            motion_boundary=start,
        )
    )
    session.run_until_stable()
    plan = session.start_execution(start)
    predicted_scene = {"cartons": ["b"]}
    predicted = PlanningWorldSnapshot(
        SceneRevision.from_scene(predicted_scene, 1),
        predicted_scene,
        RobotStateRevision(1, plan.expected_end_boundary.q, {"mode": "AUTO"}),
        snapshot.tool_attachment,
        snapshot.payload_attachment,
        snapshot.base_state,
        snapshot.conveyor_state,
        snapshot.config_identity,
    )
    session.submit_speculative(
        PlanningRequest(
            "continuous-k+1",
            predicted,
            (PlanningCandidate("b", "top"),),
            speculative=True,
            motion_boundary=plan.expected_end_boundary,
        )
    )
    session.run_until_stable()
    actual = replace(plan.expected_end_boundary, qd=(0.25, -0.1))

    completed = session.complete_execution(success=True, actual_end_boundary=actual)

    assert completed.invalidated_by is not None
    assert session.state is SessionState.RECOVERY
    assert plan.plan_id not in session.successful_completed_plan_ids
    assert not session.speculative_plans
    assert session.last_actual_execution_boundary == actual


def test_stop_acknowledgement_replans_from_complete_stopped_boundary():
    backend = FeasibleBackend()
    snapshot = world()
    session = ContinuousPlanningSession(backend)
    session.submit(PlanningRequest("stop", snapshot, (PlanningCandidate("a", "top"),)))
    session.run_until_stable()
    session.start_execution()
    changed_scene = {"cartons": ["a", "intrusion"]}
    changed = PlanningWorldSnapshot(
        SceneRevision.from_scene(changed_scene, 1),
        changed_scene,
        RobotStateRevision(1, (0.05, -0.02), {"mode": "AUTO"}),
        snapshot.tool_attachment,
        snapshot.payload_attachment,
        snapshot.base_state,
        snapshot.conveyor_state,
        snapshot.config_identity,
    )
    session.update_scene(changed)
    stopped = MotionBoundaryState.stopped((0.05, -0.02), time_seconds=0.4)
    mismatched = replace(
        changed,
        robot_state_revision=RobotStateRevision(2, (0.06, -0.02), {"mode": "AUTO"}),
    )

    with pytest.raises(ValueError, match="match stopped boundary"):
        session.acknowledge_stop(mismatched, stopped)
    session.acknowledge_stop(changed, stopped)
    session.run_until_stable()

    replanned_boundary = backend.requests[-1].motion_boundary
    assert replanned_boundary.matches(stopped)
    assert replanned_boundary.qd == (0.0, 0.0)
    assert replanned_boundary.qdd == (0.0, 0.0)
    assert replanned_boundary.time_seconds == pytest.approx(0.4)
