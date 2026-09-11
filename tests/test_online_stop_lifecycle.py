from __future__ import annotations

import pytest

from unloading_sim.online_execution import (
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
from tests.online_contract_fixtures import (
    FakeClock,
    ScriptedExecutionBackend,
)


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


class ResumeFailingSession(ContinuousPlanningSession):
    def resume_after_recovery(self):
        raise RuntimeError("injected resume failure")


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
    with pytest.raises(RuntimeError, match="before stopped evidence"):
        runtime.resume_after_recovery()


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


def test_problem_a_uncorrelated_stopped_cannot_reconcile_or_start_again():
    runtime, session, backend, _ = running_runtime()
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2),
        stop_command_id=None,
    )

    for _ in range(3):
        runtime.step()

    snapshot = runtime.snapshot()
    assert backend.start_calls == 1
    assert session.execution.state is ExecutionState.STOPPING
    assert snapshot["active_execution_id"] == backend.execution_id
    assert not snapshot["stop_lifecycle"]["stopped_confirmed"]
    assert not snapshot["stop_lifecycle"]["world_reconciled"]
    assert snapshot["stop_lifecycle"]["recovery_required"]
    assert not snapshot["stop_lifecycle"]["resume_allowed"]
    assert any(event.kind == "invalid_execution_feedback" for event in runtime.events)


def test_problem_b_start_response_loss_is_contained_without_retry_or_fake_identity():
    backend = ScriptedExecutionBackend(
        start_error_after_accept=RuntimeError("start response lost"),
    )
    clock = FakeClock()
    session = ContinuousPlanningSession(Planner(), clock=clock)
    runtime = ContinuousPlanningRuntime(session, backend, clock=clock)
    runtime.submit_initial(request("ambiguous-start", world()))

    runtime.step()
    runtime.submit_initial(request("must-not-repeat", world()))
    for _ in range(3):
        runtime.step()

    snapshot = runtime.snapshot()
    assert backend.start_calls == 1
    assert backend.stop_calls == 1
    assert snapshot["active_plan_id"] == backend.plan.plan_id
    assert snapshot["active_execution_id"] == backend.execution_id
    assert snapshot["start_lifecycle"]["state"] == "UNKNOWN"
    assert snapshot["start_lifecycle"]["execution_id"] == backend.execution_id
    assert snapshot["start_lifecycle"]["identity_source"] == "STOP_COMMAND_RESULT"
    assert snapshot["start_lifecycle"]["external_confirmation_required"]
    assert snapshot["stop_unconfirmed"]
    assert runtime.state is RuntimeState.STOPPING
    assert any(event.kind == "execution_start_unknown" for event in runtime.events)


def test_problem_c_conflicting_current_stop_evidence_latches_recovery():
    runtime, session, backend, _ = running_runtime()
    runtime.observe_world(observation(world(1, ("a", "intrusion"), q=(9.0, 9.0))))
    runtime.step()
    first = backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2),
        stop_command_id=backend.stop_command_id,
    )
    runtime.step()
    assert runtime.state is RuntimeState.WAITING_FOR_OBSERVATION

    backend.emit(
        3,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
        stop_command_id=backend.stop_command_id,
    )
    runtime.step()
    runtime.observe_world(observation(world(2, ("a", "intrusion"), q=first.current_boundary.q)))
    for _ in range(3):
        runtime.step()

    snapshot = runtime.snapshot()
    assert backend.start_calls == 1
    assert session.state is SessionState.RECOVERY
    assert snapshot["stop_lifecycle"]["evidence_conflict"]
    assert snapshot["stop_lifecycle"]["recovery_required"]
    assert not snapshot["stop_lifecycle"]["world_reconciled"]
    assert snapshot["stop_lifecycle"]["failure_reason"] == "STOP_EVIDENCE_CONFLICT"
    assert snapshot["stop_lifecycle"]["accepted_evidence"]["q"] == (0.0, 0.0)
    assert snapshot["stop_lifecycle"]["conflicting_evidence"]["q"] == (0.01, 0.0)


def test_explicit_start_rejection_is_not_treated_as_an_ambiguous_start():
    backend = ScriptedExecutionBackend(
        start_result=lambda plan, _: ExecutionCommandResult(
            ExecutionCommandStatus.REJECTED,
            "start-rejected",
            None,
            plan.plan_id,
            "controller rejected command",
        )
    )
    session = ContinuousPlanningSession(Planner())
    runtime = ContinuousPlanningRuntime(session, backend)
    runtime.submit_initial(request("rejected-start", world()))

    for _ in range(3):
        runtime.step()

    snapshot = runtime.snapshot()
    assert backend.start_calls == 1
    assert backend.stop_calls == 0
    assert snapshot["active_plan_id"] is None
    assert snapshot["active_execution_id"] is None
    assert snapshot["start_lifecycle"]["state"] == "REJECTED"
    assert not snapshot["start_lifecycle"]["external_confirmation_required"]
    assert runtime.state is RuntimeState.RECOVERY


@pytest.mark.parametrize(
    "start_result",
    [
        object(),
        lambda plan, execution_id: ExecutionCommandResult(
            ExecutionCommandStatus.ACCEPTED,
            "bad-start-identity",
            execution_id,
            f"wrong-{plan.plan_id}",
        ),
    ],
    ids=["wrong-result-type", "wrong-accepted-identity"],
)
def test_invalid_start_response_is_ambiguous_and_never_retried(start_result):
    backend = ScriptedExecutionBackend(start_result=start_result)
    session = ContinuousPlanningSession(Planner())
    runtime = ContinuousPlanningRuntime(session, backend)
    runtime.submit_initial(request("invalid-start-result", world()))

    for _ in range(5):
        runtime.step()

    snapshot = runtime.snapshot()
    assert backend.start_calls == 1
    assert backend.stop_calls == 1
    assert snapshot["start_lifecycle"]["state"] == "UNKNOWN"
    assert snapshot["start_lifecycle"]["external_confirmation_required"]
    assert snapshot["stop_unconfirmed"]
    assert snapshot["active_plan_id"] is not None
    assert not snapshot["stop_lifecycle"]["world_reconciled"]


@pytest.mark.parametrize("stop_command_id", [None, "wrong-stop", "stop-from-old-attempt"])
def test_uncorrelated_stopped_evidence_never_authorizes_replan(stop_command_id):
    runtime, session, backend, _ = running_runtime()
    runtime.observe_world(observation(world(1, ("a", "intrusion"))))
    runtime.step()
    backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2),
        stop_command_id=stop_command_id,
    )

    for _ in range(4):
        runtime.step()

    snapshot = runtime.snapshot()
    assert backend.start_calls == 1
    assert backend.stop_calls == 1
    assert session.execution.state is ExecutionState.STOPPING
    assert not snapshot["stop_lifecycle"]["stopped_confirmed"]
    assert not snapshot["stop_lifecycle"]["world_reconciled"]
    assert not snapshot["stop_lifecycle"]["resume_allowed"]


def test_conflicting_stop_evidence_requires_new_attempt_and_explicit_resume():
    runtime, session, backend, _ = running_runtime()
    mismatching_world = world(1, ("a", "intrusion"), q=(9.0, 9.0))
    runtime.observe_world(observation(mismatching_world))
    runtime.step()
    command_one = backend.stop_command_id
    backend.emit(
        2,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2),
        stop_command_id=command_one,
    )
    runtime.step()
    backend.emit(
        3,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
        stop_command_id=command_one,
    )
    runtime.step()

    with pytest.raises(RuntimeError, match="conflicting"):
        runtime.resume_after_recovery()
    assert runtime.retry_stop_after_evidence_conflict() is RuntimeState.STOPPING
    runtime.step()
    assert backend.stop_calls == 2
    command_two = backend.stop_command_id
    assert command_two != command_one

    trusted = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.3)
    backend.emit(
        4,
        ExecutionFeedbackStatus.STOPPED,
        0.3,
        trusted,
        stop_command_id=command_two,
    )
    runtime.observe_world(observation(world(2, ("a", "safe"), q=trusted.q)))
    runtime.step()

    snapshot = runtime.snapshot()
    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.state is ExecutionState.FAILED
    assert snapshot["stop_lifecycle"]["stopped_confirmed"]
    assert snapshot["stop_lifecycle"]["world_reconciled"]
    assert snapshot["stop_lifecycle"]["prior_evidence_conflict"]
    assert snapshot["stop_lifecycle"]["prior_accepted_evidence"]["q"] == (0.0, 0.0)
    assert snapshot["stop_lifecycle"]["prior_conflicting_evidence"]["q"] == (0.01, 0.0)
    assert runtime.resume_after_recovery() in {RuntimeState.IDLE, RuntimeState.PLANNING}


def test_reconciled_stop_evidence_conflict_revokes_unconsumed_recovery_grant():
    runtime, session, backend, _ = running_runtime()
    backend.emit(2, ExecutionFeedbackStatus.FAULTED, 0.2)
    runtime.step()
    command_id = backend.stop_command_id
    stopped = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2)
    backend.emit(
        3,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        stopped,
        stop_command_id=command_id,
    )
    runtime.step()
    runtime.observe_world(observation(world(1, q=stopped.q)))
    runtime.step()

    before_conflict = runtime.snapshot()
    assert runtime.state is RuntimeState.RECOVERY
    assert session.execution.state is ExecutionState.FAILED
    assert before_conflict["stop_lifecycle"]["stopped_confirmed"]
    assert before_conflict["stop_lifecycle"]["world_reconciled"]
    assert before_conflict["stop_lifecycle"]["resume_allowed"]

    backend.emit(
        4,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
        stop_command_id=command_id,
    )
    for _ in range(3):
        runtime.step()

    conflicted = runtime.snapshot()
    assert backend.start_calls == 1
    assert conflicted["stop_lifecycle"]["evidence_conflict"]
    assert not conflicted["stop_lifecycle"]["resume_allowed"]
    assert not conflicted["stop_lifecycle"]["recovery_resume_authorized"]
    assert conflicted["stop_lifecycle"]["failure_reason"] == "STOP_EVIDENCE_CONFLICT"
    assert conflicted["stop_lifecycle"]["accepted_evidence"]["q"] == (0.0, 0.0)
    assert conflicted["stop_lifecycle"]["conflicting_evidence"]["q"] == (0.01, 0.0)
    with pytest.raises(RuntimeError, match="conflicting"):
        runtime.resume_after_recovery()


def _faulted_runtime(
    *,
    feedback_limit: int,
    feedback_capacity: int = 64,
    session_class=ContinuousPlanningSession,
):
    clock = FakeClock()
    backend = ScriptedExecutionBackend()
    session = session_class(Planner(), clock=clock)
    runtime = ContinuousPlanningRuntime(
        session,
        backend,
        clock=clock,
        execution_feedback_capacity=feedback_capacity,
        max_execution_feedback_per_step=feedback_limit,
    )
    runtime.submit_initial(request("matrix-k", world()))
    runtime.step()
    backend.emit(0, ExecutionFeedbackStatus.ACCEPTED, 0.0)
    backend.emit(1, ExecutionFeedbackStatus.RUNNING, 0.2)
    for _ in range(2):
        runtime.step()
    backend.emit(2, ExecutionFeedbackStatus.FAULTED, 0.2)
    for _ in range(3):
        runtime.step()
        if backend.stop_calls:
            break
    assert backend.stop_calls == 1
    return runtime, session, backend


def _prepare_evidence_phase(
    phase: str,
    delivery: str,
    feedback_limit: int,
    *,
    feedback_capacity: int = 64,
    session_class=ContinuousPlanningSession,
):
    runtime, session, backend = _faulted_runtime(
        feedback_limit=feedback_limit,
        feedback_capacity=feedback_capacity,
        session_class=session_class,
    )
    command_id = backend.stop_command_id
    stopped = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2)
    terminal = ExecutionFeedback(
        3,
        backend.execution_id,
        backend.plan.plan_id,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        stopped,
        feedback_stream_id="scripted-feedback",
        producer_epoch=7,
        stop_command_id=command_id,
    )
    observed = observation(world(1, q=stopped.q))
    if phase == "A":
        backend.feedback.append(terminal)
        runtime.step()
    elif delivery == "stopped-first":
        backend.feedback.append(terminal)
        runtime.step()
        runtime.observe_world(observed)
        runtime.step()
    elif delivery == "observation-first":
        runtime.observe_world(observed)
        runtime.step()
        backend.feedback.append(terminal)
        runtime.step()
    elif delivery == "same-step":
        backend.feedback.append(terminal)
        runtime.observe_world(observed)
        runtime.step()
    else:
        raise AssertionError(f"unsupported matrix delivery: {delivery}")
    if phase in {"C", "D"}:
        runtime.resume_after_recovery()
    if phase == "D":
        runtime.submit_initial(
            PlanningRequest(
                "matrix-next",
                session.current_world,
                (PlanningCandidate("a", "top"),),
                motion_boundary=session.current_motion_boundary,
            )
        )
        for _ in range(4):
            runtime.step()
            if backend.start_calls == 2:
                break
        assert backend.start_calls == 2
    return runtime, session, backend, terminal


@pytest.mark.parametrize(
    "phase,input_kind,delivery,feedback_limit,conflict_expected,start_count",
    [
        ("A", "identical", "stopped-first", 1, False, 1),
        ("A", "same-sequence-conflict", "stopped-first", 8, True, 1),
        ("A", "higher-sequence-conflict", "stopped-first", 1, True, 1),
        ("B", "identical", "stopped-first", 8, False, 1),
        ("B", "same-sequence-conflict", "observation-first", 1, True, 1),
        ("B", "higher-sequence-conflict", "same-step", 8, True, 1),
        ("C", "identical", "observation-first", 8, False, 1),
        ("C", "same-sequence-conflict", "stopped-first", 8, True, 1),
        ("C", "higher-sequence-conflict", "same-step", 1, True, 1),
        ("D", "identical", "same-step", 1, False, 2),
        ("D", "higher-sequence-conflict", "observation-first", 8, False, 2),
    ],
    ids=lambda value: str(value),
)
def test_recovery_stop_evidence_event_ordering_matrix(
    phase,
    input_kind,
    delivery,
    feedback_limit,
    conflict_expected,
    start_count,
):
    replay = (
        f"phase={phase},input={input_kind},delivery={delivery},"
        f"feedback_limit={feedback_limit}"
    )
    runtime, session, backend, terminal = _prepare_evidence_phase(
        phase,
        delivery,
        feedback_limit,
    )
    if input_kind == "identical":
        injected = terminal
    else:
        sequence = terminal.feedback_sequence
        if input_kind == "higher-sequence-conflict":
            sequence += 1
        injected = ExecutionFeedback(
            sequence,
            terminal.execution_id,
            terminal.plan_id,
            ExecutionFeedbackStatus.STOPPED,
            terminal.progress,
            MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
            feedback_stream_id=terminal.feedback_stream_id,
            producer_epoch=terminal.producer_epoch,
            stop_command_id=terminal.stop_command_id,
        )
    backend.feedback.append(injected)
    for _ in range(4):
        runtime.step()

    snapshot = runtime.snapshot()
    lifecycle = snapshot["stop_lifecycle"]
    assert backend.start_calls == start_count, replay
    assert lifecycle["evidence_conflict"] is conflict_expected, replay
    assert lifecycle["retired"] is (phase == "D"), replay
    if conflict_expected:
        assert runtime.state is RuntimeState.RECOVERY, replay
        assert session.state is SessionState.RECOVERY, replay
        assert not lifecycle["resume_allowed"], replay
        assert not lifecycle["recovery_grant_valid"], replay
        assert lifecycle["failure_reason"] == "STOP_EVIDENCE_CONFLICT", replay
        assert lifecycle["accepted_evidence"]["q"] == (0.0, 0.0), replay
        assert lifecycle["conflicting_evidence"]["q"] == (0.01, 0.0), replay
    elif phase == "A":
        assert lifecycle["evidence_phase"] == "AWAITING_WORLD", replay
        assert not lifecycle["resume_allowed"], replay
    elif phase == "B":
        assert lifecycle["evidence_phase"] == "AWAITING_RECOVERY_AUTHORIZATION", replay
        assert lifecycle["resume_allowed"], replay
    elif phase == "C":
        assert lifecycle["evidence_phase"] == "RECOVERY_AUTHORIZED", replay
        assert lifecycle["recovery_grant_valid"], replay
    else:
        assert lifecycle["evidence_phase"] == "RETIRED", replay
        assert not lifecycle["recovery_grant_valid"], replay


def test_batched_stop_confirmations_cannot_bypass_feedback_limit_or_conflict_gate():
    runtime, session, backend = _faulted_runtime(feedback_limit=8)
    command_id = backend.stop_command_id
    plan_id = backend.plan.plan_id
    first = ExecutionFeedback(
        3,
        backend.execution_id,
        plan_id,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.2),
        feedback_stream_id="scripted-feedback",
        producer_epoch=7,
        stop_command_id=command_id,
    )
    conflict = ExecutionFeedback(
        4,
        backend.execution_id,
        plan_id,
        ExecutionFeedbackStatus.STOPPED,
        0.2,
        MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
        feedback_stream_id="scripted-feedback",
        producer_epoch=7,
        stop_command_id=command_id,
    )
    backend.feedback.extend((first, conflict))
    runtime.observe_world(observation(world(1, q=first.current_boundary.q)))
    for _ in range(3):
        runtime.step()

    lifecycle = runtime.snapshot()["stop_lifecycle"]
    assert backend.start_calls == 1
    assert session.state is SessionState.RECOVERY
    assert lifecycle["evidence_conflict"]
    assert not lifecycle["resume_allowed"]
    assert not lifecycle["recovery_grant_valid"]


def test_reconciled_conflict_can_reestablish_evidence_then_retire_on_accepted_handoff():
    runtime, session, backend, terminal = _prepare_evidence_phase("B", "same-step", 8)
    backend.feedback.append(
        ExecutionFeedback(
            4,
            terminal.execution_id,
            terminal.plan_id,
            ExecutionFeedbackStatus.STOPPED,
            terminal.progress,
            MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
            feedback_stream_id=terminal.feedback_stream_id,
            producer_epoch=terminal.producer_epoch,
            stop_command_id=terminal.stop_command_id,
        )
    )
    runtime.step()
    assert runtime.retry_stop_after_evidence_conflict() is RuntimeState.STOPPING
    runtime.step()
    assert backend.stop_calls == 2

    replacement = MotionBoundaryState.stopped((0.0, 0.0), time_seconds=0.3)
    backend.emit(
        5,
        ExecutionFeedbackStatus.STOPPED,
        0.3,
        replacement,
        stop_command_id=backend.stop_command_id,
    )
    runtime.observe_world(observation(world(2, q=replacement.q)))
    runtime.step()
    reconciled = runtime.snapshot()["stop_lifecycle"]
    assert reconciled["resume_allowed"]
    assert reconciled["prior_evidence_conflict"]

    runtime.resume_after_recovery()
    authorized = runtime.snapshot()["stop_lifecycle"]
    assert authorized["recovery_grant_valid"]
    runtime.submit_initial(
        PlanningRequest(
            "after-reestablished-stop",
            session.current_world,
            (PlanningCandidate("a", "top"),),
            motion_boundary=session.current_motion_boundary,
        )
    )
    for _ in range(4):
        runtime.step()
        if backend.start_calls == 2:
            break

    retired = runtime.snapshot()["stop_lifecycle"]
    assert backend.start_calls == 2
    assert retired["retired"]
    assert retired["evidence_phase"] == "RETIRED"
    assert retired["retired_by_start_attempt_id"] is not None
    assert not retired["recovery_grant_valid"]


def test_failed_resume_does_not_leave_a_partial_recovery_grant():
    runtime, session, backend, _ = _prepare_evidence_phase(
        "B",
        "same-step",
        8,
        session_class=ResumeFailingSession,
    )
    before = runtime.snapshot()["stop_lifecycle"]
    assert before["resume_allowed"]
    assert not before["recovery_grant_valid"]

    with pytest.raises(RuntimeError, match="injected resume failure"):
        runtime.resume_after_recovery()

    after = runtime.snapshot()["stop_lifecycle"]
    assert runtime.state is RuntimeState.RECOVERY
    assert session.state is SessionState.RECOVERY
    assert backend.start_calls == 1
    assert after["resume_allowed"]
    assert not after["recovery_resume_authorized"]
    assert after["recovery_grant_attempt_id"] is None
    assert after["recovery_grant_evidence_version"] is None
    assert not after["recovery_grant_valid"]


def test_reconfirmation_rejection_remains_recovery_with_external_confirmation_required():
    runtime, session, backend, terminal = _prepare_evidence_phase("B", "same-step", 8)
    backend.feedback.append(
        ExecutionFeedback(
            4,
            terminal.execution_id,
            terminal.plan_id,
            ExecutionFeedbackStatus.STOPPED,
            terminal.progress,
            MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
            feedback_stream_id=terminal.feedback_stream_id,
            producer_epoch=terminal.producer_epoch,
            stop_command_id=terminal.stop_command_id,
        )
    )
    runtime.step()
    backend.stop_status = ExecutionCommandStatus.UNSUPPORTED
    runtime.retry_stop_after_evidence_conflict()
    for _ in range(3):
        runtime.step()

    lifecycle = runtime.snapshot()["stop_lifecycle"]
    assert backend.start_calls == 1
    assert backend.stop_calls == 2
    assert runtime.state is RuntimeState.RECOVERY
    assert session.state is SessionState.RECOVERY
    assert lifecycle["command_state"] == "REJECTED"
    assert not lifecycle["stopped_confirmed"]
    assert not lifecycle["world_reconciled"]
    assert not lifecycle["resume_allowed"]
    assert not lifecycle["recovery_grant_valid"]
    with pytest.raises(RuntimeError, match="before stopped evidence"):
        runtime.resume_after_recovery()


def test_received_stop_conflict_backlog_blocks_successor_before_evidence_retirement():
    runtime, session, backend, terminal = _prepare_evidence_phase("C", "same-step", 1)
    backend.feedback.append(terminal)
    backend.feedback.append(
        ExecutionFeedback(
            terminal.feedback_sequence + 1,
            terminal.execution_id,
            terminal.plan_id,
            ExecutionFeedbackStatus.STOPPED,
            terminal.progress,
            MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
            feedback_stream_id=terminal.feedback_stream_id,
            producer_epoch=terminal.producer_epoch,
            stop_command_id=terminal.stop_command_id,
        )
    )
    runtime.submit_initial(
        PlanningRequest(
            "backlogged-stop-conflict-successor",
            session.current_world,
            (PlanningCandidate("a", "top"),),
            motion_boundary=session.current_motion_boundary,
        )
    )

    runtime.step()
    gated = runtime.snapshot()
    assert backend.start_calls == 1
    assert gated["pending_execution_feedback"] == 1
    assert gated["session"]["state"] == "READY"
    assert gated["execution_start_feedback_gate"]["active"]
    assert not gated["stop_lifecycle"]["retired"]

    runtime.step()
    conflicted = runtime.snapshot()
    assert backend.start_calls == 1
    assert runtime.state is RuntimeState.RECOVERY
    assert session.state is SessionState.RECOVERY
    assert conflicted["stop_lifecycle"]["evidence_conflict"]
    assert not conflicted["stop_lifecycle"]["resume_allowed"]
    assert not conflicted["stop_lifecycle"]["recovery_grant_valid"]
    assert conflicted["stop_lifecycle"]["accepted_evidence"]["q"] == (0.0, 0.0)
    assert conflicted["stop_lifecycle"]["conflicting_evidence"]["q"] == (0.01, 0.0)


@pytest.mark.parametrize(
    "sequence_kind,feedback_limit",
    [
        ("same", 1),
        ("same", 8),
        ("higher", 1),
        ("higher", 8),
    ],
)
def test_received_stop_conflict_gate_is_independent_of_sequence_and_step_budget(
    sequence_kind,
    feedback_limit,
):
    replay = f"sequence={sequence_kind},feedback_limit={feedback_limit}"
    runtime, session, backend, terminal = _prepare_evidence_phase(
        "C",
        "same-step",
        feedback_limit,
    )
    backend.feedback.extend(
        (
            terminal,
            ExecutionFeedback(
                terminal.feedback_sequence + int(sequence_kind == "higher"),
                terminal.execution_id,
                terminal.plan_id,
                ExecutionFeedbackStatus.STOPPED,
                terminal.progress,
                MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
                feedback_stream_id=terminal.feedback_stream_id,
                producer_epoch=terminal.producer_epoch,
                stop_command_id=terminal.stop_command_id,
            ),
        )
    )
    runtime.submit_initial(
        PlanningRequest(
            f"successor-{sequence_kind}-{feedback_limit}",
            session.current_world,
            (PlanningCandidate("a", "top"),),
            motion_boundary=session.current_motion_boundary,
        )
    )
    for _ in range(4):
        runtime.step()

    lifecycle = runtime.snapshot()["stop_lifecycle"]
    assert backend.start_calls == 1, replay
    assert runtime.state is RuntimeState.RECOVERY, replay
    assert session.state is SessionState.RECOVERY, replay
    assert lifecycle["evidence_conflict"], replay
    assert not lifecycle["retired"], replay
    assert not lifecycle["recovery_grant_valid"], replay


def test_finite_duplicate_backlog_clears_before_single_handoff_and_late_conflict_is_history():
    runtime, session, backend, terminal = _prepare_evidence_phase("C", "same-step", 1)
    backend.feedback.extend((terminal, terminal, terminal))
    runtime.submit_initial(
        PlanningRequest(
            "finite-duplicate-successor",
            session.current_world,
            (PlanningCandidate("a", "top"),),
            motion_boundary=session.current_motion_boundary,
        )
    )

    runtime.step()
    assert backend.start_calls == 1
    assert runtime.snapshot()["execution_start_feedback_gate"]["active"]
    for _ in range(4):
        runtime.step()
        if backend.start_calls == 2:
            break

    handed_off = runtime.snapshot()
    assert backend.start_calls == 2
    assert handed_off["pending_execution_feedback"] == 0
    assert handed_off["stop_lifecycle"]["retired"]

    backend.feedback.append(
        ExecutionFeedback(
            terminal.feedback_sequence + 1,
            terminal.execution_id,
            terminal.plan_id,
            ExecutionFeedbackStatus.STOPPED,
            terminal.progress,
            MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
            feedback_stream_id=terminal.feedback_stream_id,
            producer_epoch=terminal.producer_epoch,
            stop_command_id=terminal.stop_command_id,
        )
    )
    runtime.step()

    historical = runtime.snapshot()
    assert backend.start_calls == 2
    assert session.execution.state is ExecutionState.RUNNING
    assert historical["stop_lifecycle"]["retired"]
    assert not historical["stop_lifecycle"]["evidence_conflict"]


def test_feedback_capacity_boundary_remains_bounded_and_cannot_hide_conflict():
    runtime, session, backend, terminal = _prepare_evidence_phase(
        "C",
        "same-step",
        1,
        feedback_capacity=3,
    )
    conflict = ExecutionFeedback(
        terminal.feedback_sequence + 1,
        terminal.execution_id,
        terminal.plan_id,
        ExecutionFeedbackStatus.STOPPED,
        terminal.progress,
        MotionBoundaryState.stopped((0.01, 0.0), time_seconds=0.2),
        feedback_stream_id=terminal.feedback_stream_id,
        producer_epoch=terminal.producer_epoch,
        stop_command_id=terminal.stop_command_id,
    )
    backend.feedback.extend((terminal, terminal, terminal, conflict))
    runtime.submit_initial(
        PlanningRequest(
            "capacity-boundary-successor",
            session.current_world,
            (PlanningCandidate("a", "top"),),
            motion_boundary=session.current_motion_boundary,
        )
    )

    runtime.step()
    first = runtime.snapshot()
    assert backend.start_calls == 1
    assert first["pending_execution_feedback"] == 2
    assert first["pending_execution_feedback"] <= first["execution_start_feedback_gate"]["capacity"]
    assert first["execution_start_feedback_gate"]["active"]
    for _ in range(5):
        runtime.step()

    final = runtime.snapshot()
    assert backend.start_calls == 1
    assert runtime.state is RuntimeState.RECOVERY
    assert session.state is SessionState.RECOVERY
    assert final["pending_execution_feedback"] <= 3
    assert final["stop_lifecycle"]["evidence_conflict"]
    assert not final["stop_lifecycle"]["recovery_grant_valid"]
