from __future__ import annotations

from collections import deque
from dataclasses import replace

import pytest

from unloading_sim.online_execution import (
    DeterministicSimExecutionBackend,
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
    CooperativePlanningExecutor,
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
    ThreadedPlanningExecutor,
)
from unloading_sim.online_runtime import (
    ContinuousPlanningRuntime,
    RuntimeState,
    SuccessorRequestFactory,
)


WAIT_SECONDS = 3.0


def world(
    sequence: int = 0,
    cartons=("a", "b"),
    q=(0.0, 0.0),
    *,
    source: str = "synthetic_observation",
) -> PlanningWorldSnapshot:
    scene = {"cartons": list(cartons)}
    return PlanningWorldSnapshot(
        SceneRevision.from_scene(scene, sequence, source=source),
        scene,
        RobotStateRevision(sequence, q, {"mode": "AUTO"}),
        {"id": "tool"},
        None,
        {"x": 0.0},
        {"extension": 0.0},
        {
            "id": "runtime-test",
            "robot_model_fingerprint": "robot-runtime-test",
            "world_model_fingerprint": "world-runtime-test",
        },
    )


def request(request_id: str, snapshot: PlanningWorldSnapshot) -> PlanningRequest:
    return PlanningRequest(
        request_id,
        snapshot,
        (PlanningCandidate(snapshot.scene_snapshot["cartons"][0], "top"),),
    )


class RecordingPlanner(PlannerBackend):
    def __init__(self):
        super().__init__()
        self.requests: list[PlanningRequest] = []

    def plan(self, request_, candidate, planning_path):
        del planning_path
        self.requests.append(request_)
        start = request_.start_state
        end = tuple(value + 0.1 for value in start)
        return PlanningResult.succeeded(candidate, (start, end))


class SyntheticSuccessorFactory(SuccessorRequestFactory):
    def __init__(self, *, raises: bool = False, always_none: bool = False):
        self.raises = raises
        self.always_none = always_none
        self.calls = 0
        self.predicted: PlanningWorldSnapshot | None = None

    def create_successor(self, active_plan, current_world_snapshot):
        self.calls += 1
        if self.raises:
            raise RuntimeError("controlled factory failure")
        cartons = tuple(current_world_snapshot.scene_snapshot["cartons"])
        if self.always_none or len(cartons) < 2:
            return None
        remaining = cartons[1:]
        self.predicted = world(
            current_world_snapshot.scene_revision.sequence + 1,
            remaining,
            active_plan.expected_end_boundary.q,
            source="prediction_fixture",
        )
        return PlanningRequest(
            f"{active_plan.request.request_id}-successor",
            self.predicted,
            (PlanningCandidate(remaining[0], "top"),),
            speculative=True,
            motion_boundary=active_plan.expected_end_boundary,
        )


def runtime_fixture(
    *,
    execution_backend=None,
    successor_factory=None,
    executor=None,
    owns_executor=False,
):
    planner = RecordingPlanner()
    session = ContinuousPlanningSession(
        planner,
        executor=executor,
        rolling_horizon=2,
    )
    backend = execution_backend or DeterministicSimExecutionBackend(execution_steps=2)
    runtime = ContinuousPlanningRuntime(
        session,
        backend,
        successor_factory,
        owns_executor=owns_executor,
    )
    return runtime, session, planner, backend


class InjectedExecutionBackend(ExecutionBackend):
    def __init__(self, *, poll_error: Exception | None = None):
        self._identity = ExecutionBackendIdentity("injected", "1", "1")
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
        self._health = ExecutionBackendHealth.READY
        self.feedback = deque()
        self.execution_id = "injected-execution"
        self.plan = None
        self.poll_error = poll_error

    @property
    def identity(self):
        return self._identity

    @property
    def capabilities(self):
        return self._capabilities

    @property
    def health(self):
        return self._health

    @property
    def state(self):
        if self._health is ExecutionBackendHealth.SHUTDOWN:
            return ExecutionBackendState.SHUTDOWN
        return ExecutionBackendState.IDLE if self.plan is None else ExecutionBackendState.RUNNING

    def start(self, plan_envelope):
        self.plan = plan_envelope
        return ExecutionCommandResult(
            ExecutionCommandStatus.ACCEPTED,
            "injected-start",
            self.execution_id,
            plan_envelope.plan_id,
        )

    def poll(self):
        if self.poll_error is not None:
            error, self.poll_error = self.poll_error, None
            raise error
        return self.feedback.popleft() if self.feedback else None

    def request_stop(self, plan_id, reason):
        del reason
        return ExecutionCommandResult(
            ExecutionCommandStatus.ACCEPTED,
            "injected-stop",
            self.execution_id,
            plan_id,
        )

    def current_boundary(self):
        return None if self.plan is None else self.plan.expected_start_boundary

    def shutdown(self):
        self._health = ExecutionBackendHealth.SHUTDOWN

    def emit(self, sequence, status, progress, boundary=None, *, execution_id=None, plan_id=None):
        assert self.plan is not None
        item = ExecutionFeedback(
            sequence,
            execution_id or self.execution_id,
            plan_id or self.plan.plan_id,
            status,
            progress,
            boundary or self.plan.expected_start_boundary,
            observed_at_monotonic_seconds=float(sequence),
        )
        self.feedback.append(item)
        return item


def test_runtime_fixed_step_order_and_normal_k_to_k_plus_one_reuse():
    factory = SyntheticSuccessorFactory()
    runtime, session, _, _ = runtime_fixture(successor_factory=factory)
    runtime.submit_initial(request("k", world()))

    assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.WAITING_FOR_SCENE
    assert factory.calls == 1
    assert len(session.speculative_plans) == 1
    assert runtime.metrics.speculative_request_created_count == 1
    assert runtime.snapshot()["step_order"] == ContinuousPlanningRuntime.STEP_ORDER

    predicted = factory.predicted
    observed = world(
        predicted.scene_revision.sequence,
        tuple(predicted.scene_snapshot["cartons"]),
        session.last_actual_execution_boundary.q,
    )
    runtime.observe_world(observed)
    runtime.step()

    assert any(event.kind == "speculative_reused" for event in session.events)
    assert runtime.metrics.execution_start_command_count == 2
    assert runtime.state is RuntimeState.EXECUTING
    runtime.shutdown()


def test_success_without_world_observation_waits_and_prediction_cannot_be_observed():
    factory = SyntheticSuccessorFactory()
    runtime, session, _, _ = runtime_fixture(successor_factory=factory)
    runtime.submit_initial(request("k", world()))
    runtime.run_until_stable(timeout_seconds=WAIT_SECONDS)

    assert session.state is SessionState.WAITING_FOR_SCENE
    assert runtime.metrics.execution_success_count == 1
    with pytest.raises(ValueError, match="predicted"):
        runtime.observe_world(factory.predicted)
    runtime.shutdown()


def test_mismatching_observed_scene_invalidates_speculation_and_replans():
    factory = SyntheticSuccessorFactory()
    runtime, session, planner, _ = runtime_fixture(successor_factory=factory)
    runtime.submit_initial(request("k", world()))
    runtime.run_until_stable(timeout_seconds=WAIT_SECONDS)
    actual_q = session.last_actual_execution_boundary.q

    runtime.observe_world(world(1, ("b", "unexpected"), actual_q))
    runtime.step()

    assert any(
        plan.invalidated_by is ReplanReason.SPECULATIVE_MISMATCH
        for plan in session.invalidated_plans
    )
    assert len(planner.requests) >= 3
    assert runtime.state is RuntimeState.EXECUTING
    runtime.shutdown()


def test_stopping_waits_for_stopped_feedback_then_replans_from_actual_boundary():
    runtime, session, planner, backend = runtime_fixture(
        execution_backend=DeterministicSimExecutionBackend(execution_steps=5)
    )
    runtime.submit_initial(request("stop-k", world()))
    runtime.step()
    runtime.step()
    observed_boundary = backend.current_boundary()
    runtime.observe_world(world(1, ("a", "intrusion"), observed_boundary.q))

    runtime.step()
    assert session.state is SessionState.STOPPING
    assert runtime.metrics.execution_stop_request_count == 1
    requests_before_stop = len(planner.requests)

    runtime.step()
    assert session.state is SessionState.STOPPING
    assert len(planner.requests) == requests_before_stop
    runtime.step()

    assert session.state in {SessionState.READY, SessionState.EXECUTING}
    assert runtime.metrics.execution_stop_request_count == 1
    replanned = planner.requests[-1]
    assert replanned.motion_boundary.mode is BoundaryMode.STOP_BOUNDARY
    assert replanned.motion_boundary.qd == (0.0, 0.0)
    assert replanned.motion_boundary.qdd == (0.0, 0.0)
    assert replanned.motion_boundary.matches(session.last_actual_execution_boundary)
    runtime.shutdown()


@pytest.mark.parametrize(
    "terminal_status",
    [ExecutionFeedbackStatus.FAILED, ExecutionFeedbackStatus.DEVIATED],
)
def test_execution_failure_or_deviation_cascades_to_speculative_successor(terminal_status):
    factory = SyntheticSuccessorFactory()
    runtime, session, _, _ = runtime_fixture(
        execution_backend=DeterministicSimExecutionBackend(
            execution_steps=2,
            terminal_status=terminal_status,
        ),
        successor_factory=factory,
    )
    runtime.submit_initial(request("failure-k", world()))

    assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.RECOVERY
    assert not session.speculative_plans
    assert session.invalidated_plans
    if terminal_status is ExecutionFeedbackStatus.FAILED:
        assert runtime.metrics.execution_failure_count == 1
    else:
        assert runtime.metrics.execution_deviation_count == 1
    runtime.shutdown()


def test_factory_none_is_normal_and_factory_exception_enters_recovery():
    none_factory = SyntheticSuccessorFactory(always_none=True)
    runtime, _, _, _ = runtime_fixture(successor_factory=none_factory)
    runtime.submit_initial(request("none", world()))
    assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.WAITING_FOR_SCENE
    assert none_factory.calls == 1
    assert runtime.metrics.speculative_request_skipped_count == 1
    runtime.shutdown()

    raising_factory = SyntheticSuccessorFactory(raises=True)
    runtime, _, _, _ = runtime_fixture(successor_factory=raising_factory)
    runtime.submit_initial(request("raises", world()))
    assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.RECOVERY
    assert raising_factory.calls == 1
    runtime.shutdown()


def test_start_rejection_is_recovery_not_fake_execution_or_deadlock():
    runtime, session, _, _ = runtime_fixture(
        execution_backend=DeterministicSimExecutionBackend(reject_start=True)
    )
    runtime.submit_initial(request("rejected", world()))

    assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.RECOVERY
    assert session.execution.active_plan is None
    assert runtime.metrics.execution_start_command_count == 1
    assert runtime.metrics.execution_start_rejected_count == 1
    runtime.shutdown()


def test_feedback_sequence_progress_and_identity_fail_closed():
    cases = ("sequence", "progress", "identity")
    for case in cases:
        backend = InjectedExecutionBackend()
        runtime, session, _, _ = runtime_fixture(execution_backend=backend)
        runtime.submit_initial(request(f"invalid-{case}", world()))
        runtime.step()
        plan = backend.plan
        backend.emit(1, ExecutionFeedbackStatus.ACCEPTED, 0.0)
        backend.emit(2, ExecutionFeedbackStatus.RUNNING, 0.5)
        runtime.step()
        if case == "sequence":
            backend.emit(2, ExecutionFeedbackStatus.RUNNING, 0.6)
        elif case == "progress":
            backend.emit(3, ExecutionFeedbackStatus.RUNNING, 0.4)
        else:
            backend.emit(
                3,
                ExecutionFeedbackStatus.RUNNING,
                0.6,
                plan_id="wrong-plan",
            )
        runtime.step()

        assert runtime.state is RuntimeState.RECOVERY
        assert session.execution.active_plan is None
        assert runtime.metrics.invalid_feedback_count == 1
        runtime.shutdown()


def test_identical_terminal_feedback_is_idempotent_but_conflict_fails_closed():
    backend = InjectedExecutionBackend()
    runtime, _, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("duplicate", world()))
    runtime.step()
    terminal = backend.emit(
        1,
        ExecutionFeedbackStatus.SUCCEEDED,
        1.0,
        backend.plan.expected_end_boundary,
    )
    backend.feedback.append(terminal)
    runtime.step()
    assert runtime.state is RuntimeState.WAITING_FOR_SCENE
    assert runtime.metrics.duplicate_feedback_ignored_count == 1
    runtime.shutdown()

    backend = InjectedExecutionBackend()
    runtime, _, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("conflict", world()))
    runtime.step()
    backend.emit(
        1,
        ExecutionFeedbackStatus.SUCCEEDED,
        1.0,
        backend.plan.expected_end_boundary,
    )
    backend.emit(
        2,
        ExecutionFeedbackStatus.FAILED,
        1.0,
        backend.plan.expected_end_boundary,
    )
    runtime.step()
    assert runtime.state is RuntimeState.RECOVERY
    assert runtime.metrics.invalid_feedback_count == 1
    runtime.shutdown()


def test_feedback_backend_exception_enters_recovery():
    backend = InjectedExecutionBackend(poll_error=RuntimeError("controlled poll failure"))
    runtime, _, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("poll-error", world()))
    runtime.step()
    runtime.step()
    assert runtime.state is RuntimeState.RECOVERY
    assert runtime.metrics.invalid_feedback_count == 1
    runtime.shutdown()


def test_continuous_feedback_without_velocity_capability_fails_closed_without_zero_fill():
    backend = InjectedExecutionBackend()
    backend._capabilities = replace(backend.capabilities, reports_velocity=False)
    runtime, _, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("missing-velocity", world()))
    runtime.step()
    continuous = MotionBoundaryState(
        backend.plan.expected_start_boundary.q,
        (0.1, 0.0),
        (0.0, 0.0),
        0.1,
        BoundaryMode.CONTINUOUS_BOUNDARY,
    )
    backend.emit(1, ExecutionFeedbackStatus.RUNNING, 0.1, continuous)

    runtime.step()

    assert runtime.state is RuntimeState.RECOVERY
    assert runtime.metrics.invalid_feedback_count == 1
    assert continuous.qd == (0.1, 0.0)
    runtime.shutdown()


def test_stop_rejection_enters_recovery_and_is_not_retried():
    class StopRejectingBackend(InjectedExecutionBackend):
        def request_stop(self, plan_id, reason):
            del reason
            return ExecutionCommandResult(
                ExecutionCommandStatus.UNSUPPORTED,
                "stop-unsupported",
                None,
                plan_id,
            )

    backend = StopRejectingBackend()
    runtime, session, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("stop-rejected", world()))
    runtime.step()
    runtime.observe_world(world(1, ("a", "intrusion"), (0.0, 0.0)))

    runtime.step()
    runtime.step()

    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.active_plan is None
    assert runtime.metrics.execution_stop_request_count == 1
    assert runtime.metrics.execution_stop_rejected_count == 1
    runtime.shutdown()


def test_runtime_shutdown_is_idempotent_and_executor_ownership_is_explicit():
    shared = CooperativePlanningExecutor()
    runtime, _, _, backend = runtime_fixture(executor=shared, owns_executor=False)
    runtime.shutdown()
    runtime.shutdown()
    assert not shared.closed
    assert backend.health is ExecutionBackendHealth.SHUTDOWN
    with pytest.raises(RuntimeError, match="closed"):
        runtime.submit_initial(request("late", world()))
    with pytest.raises(RuntimeError, match="closed"):
        runtime.observe_world(world())
    with pytest.raises(RuntimeError, match="closed"):
        runtime.step()
    shared.shutdown()

    owned = CooperativePlanningExecutor()
    runtime, _, _, _ = runtime_fixture(executor=owned, owns_executor=True)
    runtime.shutdown()
    assert owned.closed


def test_threaded_planning_and_deterministic_execution_compose_without_shared_state():
    executor = ThreadedPlanningExecutor()
    runtime, session, _, _ = runtime_fixture(executor=executor, owns_executor=True)
    runtime.submit_initial(request("threaded-runtime", world()))
    try:
        assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.WAITING_FOR_SCENE
        assert session.last_actual_execution_boundary is not None
        assert runtime.metrics.execution_success_count == 1
    finally:
        runtime.shutdown()
    assert executor.closed
    assert not executor.worker_alive
