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
    PlanStatus,
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
    ObservationAuthority,
    RuntimeIngressStatus,
    RuntimeState,
    SuccessorRequestFactory,
    WorldObservation,
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


def observation(
    snapshot: PlanningWorldSnapshot,
    *,
    authority: ObservationAuthority = ObservationAuthority.AUTHORITATIVE,
    producer_id: str = "synthetic-perception",
    stream_id: str = "world",
    producer_epoch: int = 0,
    sequence: int | None = None,
    source: str = "synthetic fixture",
) -> WorldObservation:
    return WorldObservation(
        authority=authority,
        producer_id=producer_id,
        stream_id=stream_id,
        producer_epoch=producer_epoch,
        sequence=snapshot.scene_revision.sequence if sequence is None else sequence,
        observed_at_monotonic_seconds=float(
            snapshot.scene_revision.sequence + producer_epoch + 1
        ),
        snapshot=snapshot,
        source=source,
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


class FailingPlanner(PlannerBackend):
    def plan(self, request_, candidate, planning_path):
        del request_, planning_path
        return PlanningResult.failed(PlanStatus.NO_IK, candidate)


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
    **runtime_options,
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
        **runtime_options,
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
        self.execution_id = "injected-execution-0"
        self.execution_index = 0
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
        self.execution_index += 1
        self.execution_id = f"injected-execution-{self.execution_index}"
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

    def emit(
        self,
        sequence,
        status,
        progress,
        boundary=None,
        *,
        execution_id=None,
        plan_id=None,
        feedback_stream_id="injected-feedback",
        producer_epoch=0,
    ):
        assert self.plan is not None
        item = ExecutionFeedback(
            sequence,
            execution_id or self.execution_id,
            plan_id or self.plan.plan_id,
            status,
            progress,
            boundary or self.plan.expected_start_boundary,
            observed_at_monotonic_seconds=float(sequence),
            feedback_stream_id=feedback_stream_id,
            producer_epoch=producer_epoch,
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
    runtime.observe_world(observation(observed))
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
    result = runtime.observe_world(
        observation(
            factory.predicted,
            authority=ObservationAuthority.PREDICTED,
            source="ordinary-camera-name",
        )
    )
    assert result.status is RuntimeIngressStatus.REJECTED
    runtime.shutdown()


def test_mismatching_observed_scene_invalidates_speculation_and_replans():
    factory = SyntheticSuccessorFactory()
    runtime, session, planner, _ = runtime_fixture(successor_factory=factory)
    runtime.submit_initial(request("k", world()))
    runtime.run_until_stable(timeout_seconds=WAIT_SECONDS)
    actual_q = session.last_actual_execution_boundary.q

    runtime.observe_world(observation(world(1, ("b", "unexpected"), actual_q)))
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
    runtime.observe_world(
        observation(world(1, ("a", "intrusion"), observed_boundary.q))
    )

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
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0)
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
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0)
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


def test_faulted_feedback_enters_explicit_recovery():
    backend = InjectedExecutionBackend()
    runtime, session, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("faulted", world()))
    runtime.step()
    backend.emit(1, ExecutionFeedbackStatus.FAULTED, 0.0)

    runtime.step()

    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.active_plan is None
    assert runtime.metrics.execution_failure_count == 1
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
    runtime.observe_world(
        observation(world(1, ("a", "intrusion"), (0.0, 0.0)))
    )

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
        runtime.observe_world(observation(world()))
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


def test_world_observation_authority_is_typed_and_source_is_diagnostic_only():
    runtime, _, _, _ = runtime_fixture()
    authoritative = observation(
        world(1, source="predictive-camera-authoritative"),
        source="contains-predict-but-is-authoritative",
    )
    assert runtime.observe_world(authoritative).status is RuntimeIngressStatus.ACCEPTED
    runtime.step()
    assert runtime.snapshot()["latest_world_fingerprint"] == authoritative.snapshot.fingerprint

    predicted = observation(
        world(2, source="plain-source"),
        authority=ObservationAuthority.PREDICTED,
        source="plain-source",
    )
    assert runtime.observe_world(predicted).status is RuntimeIngressStatus.REJECTED
    runtime.step()
    assert runtime.snapshot()["latest_world_fingerprint"] == authoritative.snapshot.fingerprint
    runtime.shutdown()


def test_world_observation_duplicate_stale_conflict_and_epoch_restart():
    runtime, _, _, _ = runtime_fixture()
    first = observation(world(4), sequence=10)
    assert runtime.observe_world(first).status is RuntimeIngressStatus.ACCEPTED
    assert runtime.observe_world(first).status is RuntimeIngressStatus.DUPLICATE
    assert (
        runtime.observe_world(observation(world(3), sequence=9)).status
        is RuntimeIngressStatus.STALE
    )
    conflict = observation(world(4, cartons=("different",)), sequence=10)
    assert runtime.observe_world(conflict).status is RuntimeIngressStatus.CONFLICT
    runtime.step()
    assert runtime.state is RuntimeState.RECOVERY
    assert runtime.metrics.ingress_duplicate_count == 1
    assert runtime.metrics.ingress_stale_count == 1
    assert runtime.metrics.ingress_conflict_count == 1
    runtime.shutdown()

    runtime, _, _, _ = runtime_fixture()
    assert runtime.observe_world(observation(world(1), producer_epoch=0, sequence=50)).accepted
    reset = runtime.observe_world(observation(world(2), producer_epoch=1, sequence=0))
    assert reset.status in {RuntimeIngressStatus.ACCEPTED, RuntimeIngressStatus.COALESCED}
    runtime.step()
    assert runtime.state is not RuntimeState.RECOVERY
    runtime.shutdown()


def test_feedback_sequence_is_scoped_per_execution_and_late_prior_feedback_is_stale():
    backend = InjectedExecutionBackend()
    factory = SyntheticSuccessorFactory()
    runtime, session, _, _ = runtime_fixture(
        execution_backend=backend,
        successor_factory=factory,
        terminal_feedback_history_capacity=1,
    )
    initial_world = world()
    runtime.submit_initial(request("execution-a", initial_world))
    runtime.step()
    plan_a = backend.plan
    execution_a = backend.execution_id
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0)
    backend.emit(1, ExecutionFeedbackStatus.RUNNING, 0.5)
    backend.emit(2, ExecutionFeedbackStatus.SUCCEEDED, 1.0, plan_a.expected_end_boundary)
    runtime.step()
    assert runtime.state is RuntimeState.WAITING_FOR_SCENE

    predicted = factory.predicted
    next_world = world(
        predicted.scene_revision.sequence,
        tuple(predicted.scene_snapshot["cartons"]),
        session.last_actual_execution_boundary.q,
    )
    runtime.observe_world(observation(next_world))
    runtime.step()
    plan_b = backend.plan
    execution_b = backend.execution_id
    assert execution_a != execution_b

    backend.emit(
        1,
        ExecutionFeedbackStatus.RUNNING,
        0.5,
        plan_a.expected_start_boundary,
        execution_id=execution_a,
        plan_id=plan_a.plan_id,
    )
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0, producer_epoch=1)
    backend.emit(1, ExecutionFeedbackStatus.RUNNING, 0.1, producer_epoch=1)
    runtime.step()

    assert runtime.state is RuntimeState.EXECUTING
    assert runtime.metrics.stale_execution_feedback_count == 1
    assert session.execution.active_plan.plan_id == plan_b.plan_id
    backend.emit(
        2,
        ExecutionFeedbackStatus.SUCCEEDED,
        1.0,
        plan_b.expected_end_boundary,
        producer_epoch=1,
    )
    runtime.step()
    assert runtime.snapshot()["terminal_feedback_history"] == 1
    runtime.shutdown()


@pytest.mark.parametrize(
    "statuses",
    [
        (ExecutionFeedbackStatus.ACCEPTED, ExecutionFeedbackStatus.RUNNING, ExecutionFeedbackStatus.ACCEPTED),
        (ExecutionFeedbackStatus.ACCEPTED, ExecutionFeedbackStatus.RUNNING, ExecutionFeedbackStatus.STOPPING, ExecutionFeedbackStatus.RUNNING),
    ],
)
def test_illegal_feedback_state_transitions_fail_closed(statuses):
    backend = InjectedExecutionBackend()
    runtime, _, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("transition", world()))
    runtime.step()
    for sequence, status in enumerate(statuses):
        backend.emit(sequence, status, min(sequence * 0.1, 0.9))
    runtime.step()
    assert runtime.state is RuntimeState.RECOVERY
    assert runtime.metrics.invalid_feedback_count == 1
    runtime.shutdown()


def test_command_acknowledgement_capability_controls_first_feedback():
    backend = InjectedExecutionBackend()
    runtime, _, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("ack-required", world()))
    runtime.step()
    backend.emit(0, ExecutionFeedbackStatus.RUNNING, 0.1)
    runtime.step()
    assert runtime.state is RuntimeState.RECOVERY
    runtime.shutdown()

    backend = InjectedExecutionBackend()
    backend._capabilities = replace(
        backend.capabilities,
        supports_command_acknowledgement=False,
    )
    runtime, _, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("ack-not-required", world()))
    runtime.step()
    backend.emit(0, ExecutionFeedbackStatus.RUNNING, 0.1)
    runtime.step()
    assert runtime.state is RuntimeState.EXECUTING
    runtime.shutdown()


def test_stopped_feedback_waits_for_later_matching_authoritative_observation():
    backend = DeterministicSimExecutionBackend(execution_steps=5)
    runtime, session, planner, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("stopped-first", world()))
    runtime.step()
    runtime.step()
    running_boundary = backend.current_boundary()
    runtime.observe_world(
        observation(world(1, ("a", "intrusion"), (0.5, 0.5)))
    )
    runtime.step()
    requests_before_stop = len(planner.requests)
    runtime.step()
    runtime.step()

    assert runtime.state is RuntimeState.WAITING_FOR_OBSERVATION
    assert session.state is SessionState.STOPPING
    assert len(planner.requests) == requests_before_stop
    assert runtime.metrics.stop_waiting_observation_steps >= 1
    assert runtime.metrics.execution_start_command_count == 1

    stopped = backend.current_boundary()
    assert stopped is not None and stopped.mode is BoundaryMode.STOP_BOUNDARY
    runtime.observe_world(
        observation(world(2, ("a", "intrusion"), stopped.q), sequence=2)
    )
    runtime.step()
    assert session.state in {SessionState.PLANNING, SessionState.READY, SessionState.EXECUTING}
    assert planner.requests[-1].motion_boundary.matches(session.last_actual_execution_boundary)
    runtime.shutdown()


def test_world_observation_before_stopped_feedback_reconciles_safely():
    backend = DeterministicSimExecutionBackend(execution_steps=5)
    runtime, session, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("world-first", world()))
    runtime.step()
    runtime.step()
    boundary = backend.current_boundary()
    runtime.observe_world(observation(world(1, ("a", "intrusion"), boundary.q)))
    runtime.step()
    runtime.step()
    assert session.state is SessionState.STOPPING
    runtime.step()
    assert runtime.state is not RuntimeState.WAITING_FOR_OBSERVATION
    assert session.last_actual_execution_boundary.mode is BoundaryMode.STOP_BOUNDARY
    runtime.shutdown()


def test_bounded_ingress_backpressure_coalescing_and_terminal_delivery():
    backend = InjectedExecutionBackend()
    runtime, session, _, _ = runtime_fixture(execution_backend=backend)
    runtime = ContinuousPlanningRuntime(
        session,
        backend,
        initial_request_capacity=1,
        world_observation_capacity=1,
        execution_feedback_capacity=1,
        max_execution_feedback_per_step=1,
    )
    assert runtime.submit_initial(request("capacity-a", world())).accepted
    assert (
        runtime.submit_initial(request("capacity-b", world())).status
        is RuntimeIngressStatus.BACKPRESSURED
    )
    runtime.step()
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0)
    backend.emit(1, ExecutionFeedbackStatus.RUNNING, 0.5)
    terminal = backend.emit(
        2,
        ExecutionFeedbackStatus.SUCCEEDED,
        1.0,
        backend.plan.expected_end_boundary,
    )
    runtime.step()
    assert backend.feedback
    assert terminal in backend.feedback
    runtime.step()
    runtime.step()
    assert runtime.state is RuntimeState.WAITING_FOR_SCENE
    assert runtime.metrics.execution_success_count == 1

    first = runtime.observe_world(observation(world(1, q=backend.plan.expected_end_boundary.q)))
    newer = runtime.observe_world(observation(world(2, q=backend.plan.expected_end_boundary.q)))
    assert first.status is RuntimeIngressStatus.ACCEPTED
    assert newer.status is RuntimeIngressStatus.COALESCED
    runtime.step()
    assert runtime.metrics.ingress_backpressured_count >= 1
    assert runtime.metrics.ingress_coalesced_count == 1
    runtime.shutdown()


def test_coalesced_observation_still_invalidates_executing_plan():
    backend = DeterministicSimExecutionBackend(execution_steps=5)
    runtime, session, _, _ = runtime_fixture(execution_backend=backend)
    runtime.submit_initial(request("coalesce-stop", world()))
    runtime.step()
    runtime.step()
    boundary = backend.current_boundary()
    assert runtime.observe_world(observation(world(1, ("a", "x"), boundary.q))).accepted
    result = runtime.observe_world(observation(world(2, ("a", "y"), boundary.q)))
    assert result.status is RuntimeIngressStatus.COALESCED
    runtime.step()
    assert session.state is SessionState.STOPPING
    assert runtime.metrics.execution_stop_request_count == 1
    runtime.shutdown()


def test_planner_exhaustion_and_duplicate_request_error_reach_terminal_states():
    session = ContinuousPlanningSession(FailingPlanner())
    runtime = ContinuousPlanningRuntime(
        session,
        DeterministicSimExecutionBackend(),
    )
    runtime.submit_initial(request("planner-blocked", world()))
    assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.BLOCKED
    runtime.shutdown()

    runtime, _, _, _ = runtime_fixture(initial_request_capacity=2)
    duplicate = request("duplicate-ingress", world())
    runtime.submit_initial(duplicate)
    runtime.submit_initial(duplicate)
    assert runtime.run_until_stable(timeout_seconds=WAIT_SECONDS) is RuntimeState.RECOVERY
    runtime.shutdown()
