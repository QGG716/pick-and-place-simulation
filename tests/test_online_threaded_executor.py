from __future__ import annotations

from dataclasses import replace
from threading import Event, get_ident

import pytest

from unloading_sim.online_planning import (
    BackendCapabilities,
    ContinuousPlanningSession,
    CooperativePlanningExecutor,
    DeterministicAsyncPlanningExecutor,
    ExecutionState,
    PlanStatus,
    PlanningCancellationResult,
    PlanningCandidate,
    PlanningPath,
    PlanningRequest,
    PlanningResult,
    PlanningWorldSnapshot,
    PlannerBackend,
    RobotStateRevision,
    SceneRevision,
    SessionState,
    SynchronousPlanningExecutor,
    ThreadedPlanningExecutor,
)


WAIT_SECONDS = 3.0


def world(sequence: int = 0, cartons=("a", "b"), q=(0.0, 0.0)) -> PlanningWorldSnapshot:
    scene = {"cartons": list(cartons)}
    return PlanningWorldSnapshot(
        SceneRevision.from_scene(scene, sequence),
        scene,
        RobotStateRevision(sequence, q, {"mode": "AUTO"}),
        {"id": "tool"},
        None,
        {"x": 0.0},
        {"extension": 0.0},
        {
            "id": "threaded-test",
            "robot_model_fingerprint": "robot-threaded-test",
            "world_model_fingerprint": "world-threaded-test",
        },
    )


def request(
    request_id: str,
    snapshot: PlanningWorldSnapshot,
    *,
    speculative: bool = False,
) -> PlanningRequest:
    return PlanningRequest(
        request_id,
        snapshot,
        (PlanningCandidate(snapshot.scene_snapshot["cartons"][0], "top"),),
        speculative=speculative,
    )


def success_result(request_: PlanningRequest, candidate: PlanningCandidate) -> PlanningResult:
    start = request_.start_state
    return PlanningResult.succeeded(
        candidate,
        (start, tuple(value + 0.1 for value in start)),
        latency_seconds=123.0,
    )


class BlockingBackend(PlannerBackend):
    def __init__(
        self,
        *,
        blocked_request_id: str | None = None,
        supports_logical_cancel: bool = True,
        raise_fast: bool = False,
    ) -> None:
        super().__init__()
        self.capabilities = replace(
            self.capabilities,
            supports_logical_cancel=supports_logical_cancel,
        )
        self.blocked_request_id = blocked_request_id
        self.raise_fast = raise_fast
        self.started = Event()
        self.release = Event()
        self.calls: list[tuple[str, PlanningPath, int]] = []
        self.cancelled: list[str] = []

    def plan(self, request_, candidate, planning_path):
        self.calls.append((request_.request_id, planning_path, get_ident()))
        if self.raise_fast and planning_path is PlanningPath.FAST:
            raise RuntimeError("controlled backend failure")
        if request_.request_id == self.blocked_request_id:
            self.started.set()
            if not self.release.wait(WAIT_SECONDS):
                raise TimeoutError("test did not release blocked backend")
        return success_result(request_, candidate)

    def logical_cancel(self, request_id: str) -> None:
        self.cancelled.append(request_id)


def ready_executing_session(
    backend: BlockingBackend,
    executor: ThreadedPlanningExecutor,
) -> tuple[ContinuousPlanningSession, object]:
    session = ContinuousPlanningSession(backend, executor=executor, rolling_horizon=2)
    session.submit(request("k", world()))
    session.run_until_stable(timeout_seconds=WAIT_SECONDS)
    plan = session.start_execution()
    assert plan is not None
    return session, plan


def test_threaded_operation_runs_on_worker_and_completion_is_consumed_once():
    executor = ThreadedPlanningExecutor()
    caller_thread = get_ident()
    candidate = PlanningCandidate("a", "top")
    snapshot = world()
    planning_request = request("direct", snapshot)
    try:
        executor.submit("task", lambda: success_result(planning_request, candidate))
        assert executor.wait_for_completion(WAIT_SECONDS)
        completion = executor.pop_completed()
        assert completion is not None
        assert completion.result is not None
        assert executor.pop_completed() is None
        # The session integration test below observes the backend's worker id.
        backend = BlockingBackend()
        session = ContinuousPlanningSession(backend, executor=executor)
        session.submit(request("thread-id", snapshot))
        session.run_until_stable(timeout_seconds=WAIT_SECONDS)
        assert backend.calls[0][2] != caller_thread
    finally:
        executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)
    assert not executor.worker_alive


def test_k_plus_one_really_runs_in_background_without_mutating_executing_k():
    executor = ThreadedPlanningExecutor()
    backend = BlockingBackend(blocked_request_id="k+1")
    try:
        session, plan = ready_executing_session(backend, executor)
        predicted = world(1, ("b",), plan.expected_end_state)
        session.submit_speculative(request("k+1", predicted, speculative=True))

        assert backend.started.wait(WAIT_SECONDS)
        assert session.state is SessionState.EXECUTING
        assert session.execution.state is ExecutionState.RUNNING
        assert session.execution.active_plan.plan_id == plan.plan_id
        assert not session.speculative_plans

        backend.release.set()
        assert executor.wait_for_completion(WAIT_SECONDS)
        session.advance()

        assert session.execution.state is ExecutionState.RUNNING
        assert len(session.speculative_plans) == 1
    finally:
        backend.release.set()
        executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)


def test_scene_change_discards_late_old_generation_result_after_logical_cancel():
    executor = ThreadedPlanningExecutor()
    backend = BlockingBackend(blocked_request_id="k+1")
    try:
        session, plan = ready_executing_session(backend, executor)
        predicted = world(1, ("b",), plan.expected_end_state)
        session.submit_speculative(request("k+1", predicted, speculative=True))
        assert backend.started.wait(WAIT_SECONDS)

        observed = world(1, ("b", "intrusion"), plan.expected_end_state)
        session.update_scene(observed)
        assert session.state is SessionState.STOPPING
        assert backend.cancelled == ["k+1"]

        backend.release.set()
        assert executor.wait_for_completion(WAIT_SECONDS)
        session.advance()

        assert session.state is SessionState.STOPPING
        assert not session.ready_plans
        assert not session.speculative_plans
        assert any(event.kind == "stale_result_discarded" for event in session.events)
    finally:
        backend.release.set()
        executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)


def test_predecessor_failure_logically_cancels_and_discards_running_successor():
    executor = ThreadedPlanningExecutor()
    backend = BlockingBackend(blocked_request_id="k+1")
    try:
        session, plan = ready_executing_session(backend, executor)
        predicted = world(1, ("b",), plan.expected_end_state)
        session.submit_speculative(request("k+1", predicted, speculative=True))
        assert backend.started.wait(WAIT_SECONDS)

        session.complete_execution(success=False, stopped_q=plan.expected_end_state)
        assert session.state is SessionState.RECOVERY
        assert backend.cancelled == ["k+1"]

        backend.release.set()
        assert executor.wait_for_completion(WAIT_SECONDS)
        session.advance()

        assert session.state is SessionState.RECOVERY
        assert not session.ready_plans
        assert not session.speculative_plans
        assert any(event.kind == "stale_lineage_result_discarded" for event in session.events)
    finally:
        backend.release.set()
        executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)


def test_queued_task_can_be_cancelled_without_calling_operation():
    executor = ThreadedPlanningExecutor()
    first_started = Event()
    release_first = Event()
    second_called = Event()
    candidate = PlanningCandidate("a", "top")
    planning_request = request("queued", world())

    def first():
        first_started.set()
        if not release_first.wait(WAIT_SECONDS):
            raise TimeoutError("test did not release first task")
        return success_result(planning_request, candidate)

    def second():
        second_called.set()
        return success_result(planning_request, candidate)

    try:
        executor.submit("first", first)
        assert first_started.wait(WAIT_SECONDS)
        executor.submit("second", second)
        assert executor.cancel("second") is PlanningCancellationResult.CANCELLED_BEFORE_START
        cancelled = executor.pop_completed()
        assert cancelled is not None
        assert cancelled.task_id == "second"
        assert cancelled.cancellation_result is PlanningCancellationResult.CANCELLED_BEFORE_START
        assert not second_called.is_set()
    finally:
        release_first.set()
        executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)


def test_backend_exception_does_not_kill_worker_and_falls_back():
    executor = ThreadedPlanningExecutor()
    backend = BlockingBackend(raise_fast=True)
    try:
        session = ContinuousPlanningSession(backend, executor=executor)
        session.submit(request("fallback", world()))
        session.run_until_stable(timeout_seconds=WAIT_SECONDS)

        assert session.state is SessionState.READY
        assert [call[1] for call in backend.calls[:2]] == [PlanningPath.FAST, PlanningPath.WARM]
        assert any(event.kind == "backend_error" for event in session.events)
        assert executor.worker_alive
    finally:
        executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)


def test_shutdown_is_idempotent_wait_false_is_not_a_stop_claim_and_submit_is_rejected():
    executor = ThreadedPlanningExecutor()
    started = Event()
    release = Event()
    candidate = PlanningCandidate("a", "top")
    planning_request = request("shutdown", world())

    def blocked():
        started.set()
        if not release.wait(WAIT_SECONDS):
            raise TimeoutError("test did not release shutdown task")
        return success_result(planning_request, candidate)

    executor.submit("running", blocked)
    assert started.wait(WAIT_SECONDS)
    executor.shutdown(wait=False, cancel_pending=True, timeout_seconds=WAIT_SECONDS)
    assert executor.closed
    assert executor.worker_alive
    with pytest.raises(RuntimeError, match="closed"):
        executor.submit("after-close", blocked)
    release.set()
    executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)
    executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)
    assert not executor.worker_alive


def test_single_worker_and_duplicate_task_contracts_are_explicit():
    with pytest.raises(ValueError, match="max_workers=1"):
        ThreadedPlanningExecutor(max_workers=2)

    executor = CooperativePlanningExecutor()
    planning_request = request("duplicate", world())
    candidate = planning_request.candidates[0]
    executor.submit("same", lambda: success_result(planning_request, candidate))
    with pytest.raises(ValueError, match="duplicate"):
        executor.submit("same", lambda: success_result(planning_request, candidate))
    executor.shutdown(wait=True, cancel_pending=True)


def test_sync_cooperative_compatibility_alias_and_threaded_have_same_business_trace():
    traces = []
    executors = (
        SynchronousPlanningExecutor(),
        CooperativePlanningExecutor(),
        DeterministicAsyncPlanningExecutor(),
        ThreadedPlanningExecutor(),
    )
    try:
        for executor in executors:
            backend = BlockingBackend(raise_fast=True)
            session = ContinuousPlanningSession(backend, executor=executor)
            session.submit(request("equivalent", world()))
            session.run_until_stable(timeout_seconds=WAIT_SECONDS)
            traces.append([(event.kind, event.reason) for event in session.events])
        assert all(trace == traces[0] for trace in traces[1:])
    finally:
        for executor in executors:
            executor.shutdown(wait=True, cancel_pending=True, timeout_seconds=WAIT_SECONDS)
