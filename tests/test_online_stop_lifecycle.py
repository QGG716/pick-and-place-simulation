from __future__ import annotations

from collections import deque

import pytest

from unloading_sim.online_execution import (
    ExecutionBackend,
    ExecutionBackendCapabilities,
    ExecutionBackendHealth,
    ExecutionBackendIdentity,
    ExecutionBackendState,
    ExecutionCommandResult,
    ExecutionCommandStatus,
    ExecutionFeedback,
    ExecutionFeedbackStatus,
)
from unloading_sim.online_planning import (
    BoundaryMode,
    ContinuousPlanningSession,
    ExecutionState,
    MotionBoundaryState,
    PlanArtifactKind,
    PlanningCandidate,
    PlanningRequest,
    PlanningResult,
    PlanningWorldSnapshot,
    PlannerBackend,
    ReplanReason,
    RobotStateRevision,
    SceneRevision,
    SessionState,
)
from unloading_sim.online_runtime import (
    ContinuousPlanningRuntime,
    ObservationAuthority,
    RuntimeIngressStatus,
    RuntimeState,
    RuntimeWatchdogPolicy,
    WorldObservation,
)


class FakeClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def world(sequence: int = 0, cartons=("a",), q=(0.0, 0.0)) -> PlanningWorldSnapshot:
    scene = {"cartons": list(cartons)}
    return PlanningWorldSnapshot(
        SceneRevision.from_scene(scene, sequence, source="stop-lifecycle-test"),
        scene,
        RobotStateRevision(sequence, q, {"mode": "AUTO"}),
        {"id": "tool"},
        None,
        {"x": 0.0},
        {"extension": 0.0},
        {
            "id": "stop-lifecycle-test",
            "robot_model_fingerprint": "robot-stop-test",
            "world_model_fingerprint": "world-stop-test",
        },
    )


def request(request_id: str, snapshot: PlanningWorldSnapshot) -> PlanningRequest:
    return PlanningRequest(
        request_id,
        snapshot,
        (PlanningCandidate(snapshot.scene_snapshot["cartons"][0], "top"),),
    )


def observation(snapshot: PlanningWorldSnapshot, *, sequence: int | None = None) -> WorldObservation:
    return WorldObservation(
        authority=ObservationAuthority.AUTHORITATIVE,
        producer_id="synthetic-perception",
        stream_id="world",
        producer_epoch=0,
        sequence=snapshot.scene_revision.sequence if sequence is None else sequence,
        observed_at_monotonic_seconds=float(snapshot.scene_revision.sequence + 1),
        snapshot=snapshot,
        source="stop lifecycle fixture",
    )


class Planner(PlannerBackend):
    def plan(self, request_, candidate, planning_path):
        del planning_path
        start = request_.motion_boundary
        end = MotionBoundaryState.stopped(
            tuple(value + 0.1 for value in start.q),
            time_seconds=start.time_seconds + 1.0,
        )
        return PlanningResult.succeeded(
            candidate,
            (start.q, end.q),
            expected_start_boundary=start,
            expected_end_boundary=end,
        )


class ScriptedExecutionBackend(ExecutionBackend):
    def __init__(
        self,
        *,
        stop_status: ExecutionCommandStatus = ExecutionCommandStatus.ACCEPTED,
        stop_error: Exception | None = None,
    ) -> None:
        self._identity = ExecutionBackendIdentity("scripted-stop", "1", "1")
        self._capabilities = ExecutionBackendCapabilities(
            frozenset({PlanArtifactKind.GEOMETRIC_PATH}),
            frozenset({BoundaryMode.STOP_BOUNDARY}),
            True,
            True,
            True,
            True,
            True,
            True,
            True,
            False,
        )
        self.feedback = deque()
        self.plan = None
        self.execution_id = "execution-0"
        self.stop_calls = 0
        self.stop_command_id: str | None = None
        self.start_calls = 0
        self.stop_status = stop_status
        self.stop_error = stop_error

    @property
    def identity(self):
        return self._identity

    @property
    def capabilities(self):
        return self._capabilities

    @property
    def health(self):
        return ExecutionBackendHealth.READY

    @property
    def state(self):
        return ExecutionBackendState.IDLE if self.plan is None else ExecutionBackendState.RUNNING

    def start(self, plan_envelope):
        self.start_calls += 1
        self.execution_id = f"execution-{self.start_calls}"
        self.plan = plan_envelope
        return ExecutionCommandResult(
            ExecutionCommandStatus.ACCEPTED,
            f"start-{self.start_calls}",
            self.execution_id,
            plan_envelope.plan_id,
        )

    def poll(self):
        return self.feedback.popleft() if self.feedback else None

    def request_stop(self, plan_id, reason):
        del reason
        self.stop_calls += 1
        if self.stop_error is not None:
            raise self.stop_error
        self.stop_command_id = f"stop-{self.stop_calls}"
        return ExecutionCommandResult(
            self.stop_status,
            self.stop_command_id,
            self.execution_id,
            plan_id,
        )

    def current_boundary(self):
        return None if self.plan is None else self.plan.expected_start_boundary

    def shutdown(self):
        return None

    def emit(self, sequence, status, progress, boundary=None, *, stop_command_id=None):
        assert self.plan is not None
        feedback = ExecutionFeedback(
            sequence,
            self.execution_id,
            self.plan.plan_id,
            status,
            progress,
            boundary or self.plan.expected_start_boundary,
            observed_at_monotonic_seconds=float(sequence),
            feedback_stream_id="scripted-feedback",
            producer_epoch=7,
            stop_command_id=stop_command_id,
        )
        self.feedback.append(feedback)
        return feedback


def running_runtime(*, backend=None, watchdog_policy=None):
    clock = FakeClock()
    backend = backend or ScriptedExecutionBackend()
    session = ContinuousPlanningSession(Planner(), clock=clock)
    runtime = ContinuousPlanningRuntime(
        session,
        backend,
        clock=clock,
        watchdog_policy=watchdog_policy,
    )
    runtime.submit_initial(request("k", world()))
    runtime.step()
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0)
    backend.emit(1, ExecutionFeedbackStatus.RUNNING, 0.2)
    runtime.step()
    assert session.execution.state is ExecutionState.RUNNING
    return runtime, session, backend, clock


def test_case_a_conflicting_observations_cannot_starve_single_stop_dispatch():
    runtime, session, backend, _ = running_runtime()
    changed = world(1, ("a", "intrusion"))
    assert runtime.observe_world(observation(changed, sequence=1)).status is RuntimeIngressStatus.ACCEPTED

    for index in range(3):
        conflict = world(1, ("a", f"conflict-{index}"))
        assert runtime.observe_world(observation(conflict, sequence=1)).status is RuntimeIngressStatus.CONFLICT
        runtime.step()

    snapshot = runtime.snapshot()
    assert backend.stop_calls == 1
    assert session.execution.state is ExecutionState.STOPPING
    assert snapshot["active_execution_id"] == backend.execution_id
    assert snapshot["stop_lifecycle"]["command_state"] == "ACCEPTED"
    assert not snapshot["stop_lifecycle"]["stopped_confirmed"]


def test_case_b_faulted_task_can_later_accept_correlated_stopped_evidence():
    runtime, session, backend, _ = running_runtime()
    backend.emit(2, ExecutionFeedbackStatus.FAULTED, 0.2)
    runtime.step()
    assert backend.stop_calls == 1

    stopped = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2)
    backend.emit(
        3,
        ExecutionFeedbackStatus.STOPPING,
        0.2,
        stopped,
        stop_command_id=backend.stop_command_id,
    )
    terminal = backend.emit(
        4,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        stopped,
        stop_command_id=backend.stop_command_id,
    )
    backend.feedback.append(terminal)
    runtime.observe_world(observation(world(0, q=stopped.q), sequence=1))
    runtime.step()

    snapshot = runtime.snapshot()
    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.state is ExecutionState.FAILED
    assert snapshot["stop_lifecycle"]["execution_outcome"] == "FAULTED"
    assert snapshot["stop_lifecycle"]["stopped_confirmed"]
    assert snapshot["stop_lifecycle"]["recovery_required"]
    assert runtime.metrics.duplicate_feedback_ignored_count == 1


@pytest.mark.parametrize(
    "terminal_status",
    [ExecutionFeedbackStatus.FAILED, ExecutionFeedbackStatus.DEVIATED],
)
def test_case_c_failed_or_deviated_while_stopping_keeps_identity_and_stop_supervision(
    terminal_status,
):
    runtime, session, backend, _ = running_runtime()
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    assert backend.stop_calls == 1

    backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPING,
        0.2,
        stop_command_id=backend.stop_command_id,
    )
    backend.emit(3, terminal_status, 0.2)
    runtime.step()

    snapshot = runtime.snapshot()
    assert session.execution.state is ExecutionState.STOPPING
    assert snapshot["active_execution_id"] == backend.execution_id
    assert snapshot["active_plan_id"] == backend.plan.plan_id
    assert snapshot["stop_lifecycle"]["execution_outcome"] == terminal_status.value
    assert snapshot["stop_lifecycle"]["command_state"] == "ACCEPTED"
    assert not snapshot["stop_lifecycle"]["stopped_confirmed"]
    assert snapshot["stop_unconfirmed"]
    assert backend.start_calls == 1


@pytest.mark.parametrize(
    ("backend", "command_state"),
    [
        (ScriptedExecutionBackend(stop_status=ExecutionCommandStatus.REJECTED), "REJECTED"),
        (ScriptedExecutionBackend(stop_error=RuntimeError("stop channel down")), "ERROR"),
    ],
)
def test_stop_rejection_or_exception_remains_unconfirmed_and_never_repeats(
    backend,
    command_state,
):
    runtime, session, backend, _ = running_runtime(backend=backend)
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    runtime.step()

    snapshot = runtime.snapshot()
    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.state is ExecutionState.STOPPING
    assert backend.stop_calls == 1
    assert snapshot["active_execution_id"] == backend.execution_id
    assert snapshot["stop_unconfirmed"]
    assert snapshot["stop_lifecycle"]["command_state"] == command_state
    assert not [event for event in runtime.events if event.kind == "execution_stop_command"]


def test_stop_timeout_accepts_late_correlated_confirmation_then_explicit_resume():
    policy = RuntimeWatchdogPolicy(
        scene_freshness_timeout_seconds=None,
        execution_feedback_timeout_seconds=None,
        stop_ack_timeout_seconds=1.0,
        stopped_observation_timeout_seconds=None,
        planning_timeout_seconds=None,
    )
    runtime, session, backend, clock = running_runtime(watchdog_policy=policy)
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    clock.advance(1.0)
    runtime.step()

    timed_out = runtime.snapshot()
    assert timed_out["active_execution_id"] == backend.execution_id
    assert timed_out["stop_unconfirmed"]
    assert timed_out["stop_lifecycle"]["failure_reason"] == ReplanReason.STOP_ACK_TIMEOUT.value

    stopped = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2)
    backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        stopped,
        stop_command_id=backend.stop_command_id,
    )
    runtime.step()

    confirmed = runtime.snapshot()
    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.state is ExecutionState.FAILED
    assert confirmed["stop_lifecycle"]["stopped_confirmed"]
    assert confirmed["stop_lifecycle"]["world_reconciled"]
    assert confirmed["stop_lifecycle"]["resume_allowed"]

    assert runtime.resume_after_recovery() is RuntimeState.IDLE
    assert confirmed["stop_lifecycle"]["trigger_reason"] == ReplanReason.SCENE_REVISION_CHANGED.value
    next_request = PlanningRequest(
        "after-explicit-recovery",
        session.current_world,
        (PlanningCandidate("a", "top"),),
        motion_boundary=session.current_motion_boundary,
    )
    assert runtime.submit_initial(next_request).accepted
    runtime.step()
    runtime.step()
    assert backend.start_calls == 2


def test_wrong_stop_identity_is_rejected_but_current_confirmation_can_arrive_late():
    runtime, session, backend, _ = running_runtime()
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    stopped = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2)
    backend.emit(2, ExecutionFeedbackStatus.STOPPED, 0.2, stopped, stop_command_id="old-stop")
    runtime.step()

    assert runtime.metrics.invalid_feedback_count == 1
    assert session.execution.state is ExecutionState.STOPPING
    assert runtime.snapshot()["active_execution_id"] == backend.execution_id

    backend.emit(
        3,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        stopped,
        stop_command_id=backend.stop_command_id,
    )
    runtime.step()
    snapshot = runtime.snapshot()
    assert snapshot["stop_lifecycle"]["stopped_confirmed"]
    assert snapshot["stop_lifecycle"]["recovery_required"]


def test_duplicate_stop_confirmation_is_idempotent_but_conflict_fails_closed():
    runtime, _, backend, _ = running_runtime()
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    stopped = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2)
    terminal = backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        stopped,
        stop_command_id=backend.stop_command_id,
    )
    backend.feedback.append(terminal)
    runtime.step()
    assert runtime.metrics.duplicate_feedback_ignored_count == 1

    backend.feedback.append(
        ExecutionFeedback(
            3,
            terminal.execution_id,
            terminal.plan_id,
            ExecutionFeedbackStatus.STOPPED,
            0.2,
            MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
            feedback_stream_id=terminal.feedback_stream_id,
            producer_epoch=terminal.producer_epoch,
            stop_command_id=terminal.stop_command_id,
        )
    )
    runtime.step()
    assert runtime.metrics.invalid_feedback_count == 1
    assert runtime.snapshot()["stop_lifecycle"]["stopped_boundary"]["q"] == stopped.q


def test_unconfirmed_stop_rejects_new_work_and_observation_cannot_clear_fault():
    backend = ScriptedExecutionBackend(stop_status=ExecutionCommandStatus.REJECTED)
    runtime, session, backend, _ = running_runtime(backend=backend)
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    starts_before = backend.start_calls

    runtime.observe_world(observation(world(2, ("a", "newer"))))
    runtime.submit_initial(request("must-not-run", world(2, ("a", "newer"))))
    runtime.step()

    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.state is ExecutionState.STOPPING
    assert backend.start_calls == starts_before
    assert runtime.snapshot()["stop_unconfirmed"]


def test_second_stop_attempt_has_independent_identity_watchdog_and_stale_feedback_isolated():
    policy = RuntimeWatchdogPolicy(
        scene_freshness_timeout_seconds=None,
        execution_feedback_timeout_seconds=None,
        stop_ack_timeout_seconds=1.0,
        stopped_observation_timeout_seconds=None,
        planning_timeout_seconds=None,
    )
    runtime, session, backend, clock = running_runtime(watchdog_policy=policy)
    runtime.observe_world(observation(world(1, ("a", "first"))))
    runtime.step()
    first_id = runtime.snapshot()["stop_lifecycle"]["attempt_id"]
    first_terminal = backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2),
        stop_command_id=backend.stop_command_id,
    )
    runtime.step()
    runtime.step()
    assert backend.start_calls == 2

    backend.feedback.append(first_terminal)
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0)
    backend.emit(1, ExecutionFeedbackStatus.RUNNING, 0.1)
    runtime.step()
    assert session.execution.state is ExecutionState.RUNNING
    assert runtime.metrics.duplicate_feedback_ignored_count == 1

    runtime.observe_world(observation(world(2, ("a", "second"))))
    runtime.step()
    second = runtime.snapshot()["stop_lifecycle"]
    assert second["attempt_id"] != first_id
    assert backend.stop_calls == 2
    clock.advance(1.0)
    runtime.step()
    assert runtime.snapshot()["stop_lifecycle"]["failure_reason"] == ReplanReason.STOP_ACK_TIMEOUT.value
    assert runtime.metrics.stop_ack_timeout_count == 1
