"""Deterministic orchestration for planning and execution control planes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import Counter, deque
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from threading import get_ident
from time import monotonic
from types import MappingProxyType
from typing import Any, Mapping

from .online_execution import (
    ExecutionBackend,
    ExecutionCommandResult,
    ExecutionCommandStatus,
    ExecutionFeedback,
    ExecutionFeedbackStatus,
)
from .online_planning import (
    BoundaryMode,
    ContinuousPlanningSession,
    ExecutionState,
    PlanEnvelope,
    PlanningRequest,
    PlanningWorldSnapshot,
    ReplanReason,
    SessionState,
)


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return repr(value)


class RuntimeState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    READY = "READY"
    EXECUTING = "EXECUTING"
    STOPPING = "STOPPING"
    WAITING_FOR_SCENE = "WAITING_FOR_SCENE"
    RECOVERY = "RECOVERY"
    BLOCKED = "BLOCKED"
    SHUTDOWN = "SHUTDOWN"


@dataclass(frozen=True)
class RuntimeEvent:
    sequence: int
    kind: str
    reason: str
    plan_id: str | None = None
    execution_id: str | None = None
    details: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.sequence < 1:
            raise ValueError("runtime event sequence must be positive")
        if not self.kind or not self.reason:
            raise ValueError("runtime event kind and reason must be non-empty")
        object.__setattr__(self, "details", _freeze(self.details))


class SuccessorRequestFactory(ABC):
    """Creates planning requests only; target selection remains out of scope."""

    @abstractmethod
    def create_successor(
        self,
        active_plan: PlanEnvelope,
        current_world_snapshot: PlanningWorldSnapshot,
    ) -> PlanningRequest | None: ...


class RuntimeMetrics:
    def __init__(self) -> None:
        self.execution_start_command_count = 0
        self.execution_start_rejected_count = 0
        self.execution_feedback_count_by_status: Counter[str] = Counter()
        self.execution_stop_request_count = 0
        self.execution_stop_rejected_count = 0
        self.execution_success_count = 0
        self.execution_failure_count = 0
        self.execution_deviation_count = 0
        self.duplicate_feedback_ignored_count = 0
        self.invalid_feedback_count = 0
        self.speculative_request_created_count = 0
        self.speculative_request_skipped_count = 0

    def report(self) -> dict[str, Any]:
        return {
            "execution_start_command_count": self.execution_start_command_count,
            "execution_start_rejected_count": self.execution_start_rejected_count,
            "execution_feedback_count_by_status": dict(
                sorted(self.execution_feedback_count_by_status.items())
            ),
            "execution_stop_request_count": self.execution_stop_request_count,
            "execution_stop_rejected_count": self.execution_stop_rejected_count,
            "execution_success_count": self.execution_success_count,
            "execution_failure_count": self.execution_failure_count,
            "execution_deviation_count": self.execution_deviation_count,
            "duplicate_feedback_ignored_count": self.duplicate_feedback_ignored_count,
            "invalid_feedback_count": self.invalid_feedback_count,
            "speculative_request_created_count": self.speculative_request_created_count,
            "speculative_request_skipped_count": self.speculative_request_skipped_count,
            "production_throughput": None,
        }


class ContinuousPlanningRuntime:
    """Single-thread-owned deterministic planning/execution orchestrator."""

    STEP_ORDER = (
        "submit_initial",
        "execution_feedback",
        "world_observation",
        "planning_poll",
        "execution_start",
        "successor_request",
        "stop_request",
        "execution_advance",
        "state_sync",
    )

    def __init__(
        self,
        session: ContinuousPlanningSession,
        execution_backend: ExecutionBackend,
        successor_factory: SuccessorRequestFactory | None = None,
        *,
        owns_executor: bool = False,
        owns_execution_backend: bool = True,
    ) -> None:
        if not isinstance(session, ContinuousPlanningSession):
            raise TypeError("session must be a ContinuousPlanningSession")
        if not isinstance(execution_backend, ExecutionBackend):
            raise TypeError("execution_backend must implement ExecutionBackend")
        if successor_factory is not None and not isinstance(successor_factory, SuccessorRequestFactory):
            raise TypeError("successor_factory must implement SuccessorRequestFactory")
        self.session = session
        self.execution_backend = execution_backend
        self.successor_factory = successor_factory
        self.owns_executor = bool(owns_executor)
        self.owns_execution_backend = bool(owns_execution_backend)
        self.metrics = RuntimeMetrics()
        self.events: list[RuntimeEvent] = []
        self.state = RuntimeState.IDLE
        self._closed = False
        self._owner_thread_id: int | None = None
        self._event_sequence = 0
        self._pending_initial: deque[PlanningRequest] = deque()
        self._pending_worlds: deque[PlanningWorldSnapshot] = deque()
        self._latest_world: PlanningWorldSnapshot | None = None
        self._active_plan: PlanEnvelope | None = None
        self._active_execution_id: str | None = None
        self._last_feedback_sequence = -1
        self._last_progress_by_execution: dict[str, float] = {}
        self._terminal_feedback: dict[str, ExecutionFeedback] = {}
        self._successor_attempted_plan_ids: set[str] = set()
        self._stop_requested_plan_ids: set[str] = set()

    @property
    def closed(self) -> bool:
        return self._closed

    def _assert_open(self) -> None:
        if self._closed:
            raise RuntimeError("continuous planning runtime is closed")
        if self._owner_thread_id is None:
            self._owner_thread_id = get_ident()
        elif get_ident() != self._owner_thread_id:
            raise RuntimeError("runtime operations must stay on the step owner thread")

    def _event(
        self,
        kind: str,
        reason: str,
        *,
        plan_id: str | None = None,
        execution_id: str | None = None,
        **details: Any,
    ) -> None:
        self._event_sequence += 1
        self.events.append(
            RuntimeEvent(
                self._event_sequence,
                kind,
                reason,
                plan_id,
                execution_id,
                details,
            )
        )

    def submit_initial(self, request: PlanningRequest) -> None:
        self._assert_open()
        if not isinstance(request, PlanningRequest):
            raise TypeError("initial request must be a PlanningRequest")
        self._pending_initial.append(request)

    def observe_world(self, snapshot: PlanningWorldSnapshot) -> None:
        self._assert_open()
        if not isinstance(snapshot, PlanningWorldSnapshot):
            raise TypeError("world observation must be a PlanningWorldSnapshot")
        if "predict" in snapshot.scene_revision.source.lower():
            raise ValueError("predicted scenes cannot be submitted as world observations")
        self._pending_worlds.append(snapshot)

    def _fail_active(
        self,
        message: str,
        *,
        reason: ReplanReason = ReplanReason.EXECUTION_FAILED,
        feedback: ExecutionFeedback | None = None,
    ) -> None:
        if self.session.execution.active_plan is not None:
            self.session.fail_execution_control_plane(
                message,
                reason=reason,
                actual_boundary=None if feedback is None else feedback.current_boundary,
                execution_id=self._active_execution_id,
            )
        else:
            self.session.state = SessionState.RECOVERY
        self.state = RuntimeState.RECOVERY
        self._event(
            "runtime_recovery",
            reason.value,
            plan_id=None if self._active_plan is None else self._active_plan.plan_id,
            execution_id=self._active_execution_id,
            message=message,
        )

    def _invalid_feedback(self, message: str, feedback: ExecutionFeedback | None = None) -> None:
        self.metrics.invalid_feedback_count += 1
        self._event(
            "invalid_execution_feedback",
            ReplanReason.EXECUTION_DEVIATION.value,
            plan_id=None if feedback is None else feedback.plan_id,
            execution_id=None if feedback is None else feedback.execution_id,
            message=message,
        )
        self._fail_active(
            message,
            reason=ReplanReason.EXECUTION_DEVIATION,
            feedback=feedback,
        )

    @staticmethod
    def _identical_feedback(first: ExecutionFeedback, second: ExecutionFeedback) -> bool:
        return bool(first == second and first.metadata == second.metadata)

    def _consume_feedback(self, feedback: ExecutionFeedback) -> None:
        if not isinstance(feedback, ExecutionFeedback):
            self._invalid_feedback(
                f"expected ExecutionFeedback, got {type(feedback).__name__}"
            )
            return
        previous_terminal = self._terminal_feedback.get(feedback.execution_id)
        if previous_terminal is not None:
            if self._identical_feedback(previous_terminal, feedback):
                self.metrics.duplicate_feedback_ignored_count += 1
                self._event(
                    "duplicate_terminal_feedback_ignored",
                    feedback.status.value,
                    plan_id=feedback.plan_id,
                    execution_id=feedback.execution_id,
                )
            else:
                self._invalid_feedback("conflicting terminal feedback", feedback)
            return
        if feedback.feedback_sequence <= self._last_feedback_sequence:
            self._invalid_feedback("feedback sequence is not strictly monotonic", feedback)
            return
        if (
            self._active_plan is None
            or feedback.plan_id != self._active_plan.plan_id
            or feedback.execution_id != self._active_execution_id
        ):
            self._invalid_feedback("feedback plan_id or execution_id does not match", feedback)
            return
        dof = len(self._active_plan.expected_start_boundary.q)
        if len(feedback.current_boundary.q) != dof:
            self._invalid_feedback("feedback boundary DOF does not match active plan", feedback)
            return
        capabilities = self.execution_backend.capabilities
        if not capabilities.reports_position:
            self._invalid_feedback("execution backend does not report position", feedback)
            return
        if (
            feedback.current_boundary.mode is BoundaryMode.CONTINUOUS_BOUNDARY
            and (not capabilities.reports_velocity or not capabilities.reports_acceleration)
        ):
            self._invalid_feedback(
                "continuous feedback requires reported velocity and acceleration",
                feedback,
            )
            return
        previous_progress = self._last_progress_by_execution.get(feedback.execution_id, 0.0)
        if feedback.status is ExecutionFeedbackStatus.RUNNING and feedback.progress < previous_progress:
            self._invalid_feedback("RUNNING progress moved backwards", feedback)
            return
        self._last_feedback_sequence = feedback.feedback_sequence
        self._last_progress_by_execution[feedback.execution_id] = max(
            previous_progress,
            feedback.progress,
        )
        self.metrics.execution_feedback_count_by_status[feedback.status.value] += 1
        self._event(
            "execution_feedback",
            feedback.status.value,
            plan_id=feedback.plan_id,
            execution_id=feedback.execution_id,
            progress=feedback.progress,
        )
        if feedback.status is ExecutionFeedbackStatus.RUNNING:
            if self.session.execution.state is not ExecutionState.RUNNING:
                self._invalid_feedback("RUNNING feedback received outside execution", feedback)
                return
            self.session.execution.update_progress(feedback.progress)
            return
        if feedback.status is ExecutionFeedbackStatus.ACCEPTED:
            return
        if feedback.status is ExecutionFeedbackStatus.STOPPING:
            if self.session.execution.state is not ExecutionState.STOPPING:
                self._invalid_feedback("unsolicited STOPPING feedback", feedback)
            return
        if not feedback.status.terminal:
            return
        self._terminal_feedback[feedback.execution_id] = feedback
        if feedback.status is ExecutionFeedbackStatus.STOPPED:
            if self.session.execution.state is not ExecutionState.STOPPING:
                self._invalid_feedback("STOPPED feedback received outside STOPPING", feedback)
                return
            if self._latest_world is None:
                self._invalid_feedback("STOPPED feedback has no independent world observation", feedback)
                return
            try:
                self.session.acknowledge_stop(
                    self._latest_world,
                    feedback.current_boundary,
                    feedback.message,
                    execution_id=feedback.execution_id,
                )
            except Exception as exc:
                self._invalid_feedback(f"stop acknowledgement rejected: {exc}", feedback)
                return
            self._active_plan = None
            self._active_execution_id = None
            return
        if feedback.status is ExecutionFeedbackStatus.SUCCEEDED:
            try:
                completed = self.session.complete_execution(
                    success=True,
                    actual_end_boundary=feedback.current_boundary,
                    message=feedback.message,
                    execution_id=feedback.execution_id,
                )
            except Exception as exc:
                self._invalid_feedback(f"execution completion rejected: {exc}", feedback)
                return
            if completed.invalidated_by is None:
                self.metrics.execution_success_count += 1
            else:
                self.metrics.execution_deviation_count += 1
            self._active_plan = None
            self._active_execution_id = None
            return
        if feedback.status is ExecutionFeedbackStatus.FAILED:
            self.metrics.execution_failure_count += 1
            if self.session.execution.state is ExecutionState.RUNNING:
                self.session.complete_execution(
                    success=False,
                    actual_end_boundary=feedback.current_boundary,
                    message=feedback.message,
                    execution_id=feedback.execution_id,
                )
            else:
                self._fail_active(feedback.message or "execution failed", feedback=feedback)
            self._active_plan = None
            self._active_execution_id = None
            return
        if feedback.status is ExecutionFeedbackStatus.DEVIATED:
            self.metrics.execution_deviation_count += 1
            if self.session.execution.state is ExecutionState.RUNNING:
                self.session.notify_execution_deviation(
                    feedback.message or "execution deviated",
                    feedback.current_boundary,
                    execution_id=feedback.execution_id,
                )
            else:
                self._fail_active(
                    feedback.message or "execution deviated",
                    reason=ReplanReason.EXECUTION_DEVIATION,
                    feedback=feedback,
                )
            self._active_plan = None
            self._active_execution_id = None
            return
        self.metrics.execution_failure_count += 1
        self._fail_active(feedback.message or feedback.status.value, feedback=feedback)
        self._active_plan = None
        self._active_execution_id = None

    def _poll_execution_feedback(self) -> None:
        while True:
            try:
                feedback = self.execution_backend.poll()
            except Exception as exc:
                self._invalid_feedback(f"execution backend poll failed: {exc}")
                return
            if feedback is None:
                return
            self._consume_feedback(feedback)
            if self.state is RuntimeState.RECOVERY:
                return

    def _apply_world_observations(self) -> None:
        while self._pending_worlds:
            snapshot = self._pending_worlds.popleft()
            try:
                self.session.update_scene(snapshot)
            except Exception as exc:
                self._fail_active(f"world observation rejected: {exc}")
                return
            self._latest_world = snapshot
            self._event(
                "world_observed",
                "WORLD_OBSERVATION",
                details_fingerprint=snapshot.fingerprint,
            )

    def _start_ready_plan(self) -> None:
        if self.session.state is not SessionState.READY or self._active_plan is not None:
            return
        plan = self.session.start_execution(self.session.current_motion_boundary)
        if plan is None:
            return
        self.metrics.execution_start_command_count += 1
        try:
            result = self.execution_backend.start(plan)
        except Exception as exc:
            self.metrics.execution_start_rejected_count += 1
            self._active_plan = plan
            self._fail_active(f"execution start raised: {exc}")
            return
        if not isinstance(result, ExecutionCommandResult):
            self.metrics.execution_start_rejected_count += 1
            self._active_plan = plan
            self._fail_active(
                f"execution start returned {type(result).__name__}, expected ExecutionCommandResult"
            )
            return
        if (
            result.status is not ExecutionCommandStatus.ACCEPTED
            or result.plan_id != plan.plan_id
            or not result.execution_id
        ):
            self.metrics.execution_start_rejected_count += 1
            self._active_plan = plan
            self._fail_active(
                f"execution start rejected: {result.status.value}: {result.message}"
            )
            return
        self._active_plan = plan
        self._active_execution_id = result.execution_id
        self._event(
            "execution_start_command",
            result.status.value,
            plan_id=plan.plan_id,
            execution_id=result.execution_id,
            command_id=result.command_id,
        )

    def _create_successor(self) -> None:
        if (
            self.successor_factory is None
            or self._active_plan is None
            or self._active_execution_id is None
            or self.session.execution.state is not ExecutionState.RUNNING
            or self._active_plan.plan_id in self._successor_attempted_plan_ids
        ):
            return
        plan = self._active_plan
        self._successor_attempted_plan_ids.add(plan.plan_id)
        if self.session.current_world is None:
            self.metrics.speculative_request_skipped_count += 1
            return
        try:
            request = self.successor_factory.create_successor(plan, self.session.current_world)
        except Exception as exc:
            self._fail_active(
                f"successor request factory failed: {exc}",
                reason=ReplanReason.PLANNER_FAILURE,
            )
            return
        if request is None:
            self.metrics.speculative_request_skipped_count += 1
            self._event(
                "speculative_request_skipped",
                "NO_SUCCESSOR",
                plan_id=plan.plan_id,
                execution_id=self._active_execution_id,
            )
            return
        try:
            self.session.submit_speculative(request)
        except Exception as exc:
            self._fail_active(
                f"successor request rejected: {exc}",
                reason=ReplanReason.PLANNER_FAILURE,
            )
            return
        self.metrics.speculative_request_created_count += 1
        self._event(
            "speculative_request_created",
            ReplanReason.ROLLING_HORIZON.value,
            plan_id=plan.plan_id,
            execution_id=self._active_execution_id,
            request_id=request.request_id,
        )

    def _request_stop_once(self) -> None:
        if (
            self.session.state is not SessionState.STOPPING
            or self._active_plan is None
            or self._active_plan.plan_id in self._stop_requested_plan_ids
        ):
            return
        plan_id = self._active_plan.plan_id
        self._stop_requested_plan_ids.add(plan_id)
        self.metrics.execution_stop_request_count += 1
        try:
            result = self.execution_backend.request_stop(
                plan_id,
                ReplanReason.SCENE_REVISION_CHANGED.value,
            )
        except Exception as exc:
            self.metrics.execution_stop_rejected_count += 1
            self._fail_active(f"execution stop request raised: {exc}")
            return
        if not isinstance(result, ExecutionCommandResult) or result.status not in {
            ExecutionCommandStatus.ACCEPTED,
            ExecutionCommandStatus.ALREADY_STOPPING,
        }:
            self.metrics.execution_stop_rejected_count += 1
            status = type(result).__name__ if not isinstance(result, ExecutionCommandResult) else result.status.value
            self._fail_active(f"execution stop request rejected: {status}")
            return
        self._event(
            "execution_stop_command",
            result.status.value,
            plan_id=plan_id,
            execution_id=self._active_execution_id,
            command_id=result.command_id,
        )

    def _sync_state(self) -> RuntimeState:
        if self._closed:
            self.state = RuntimeState.SHUTDOWN
        else:
            self.state = RuntimeState(self.session.state.value)
        return self.state

    def step(self) -> RuntimeState:
        self._assert_open()
        while self._pending_initial:
            self.session.submit(self._pending_initial.popleft())
        self._poll_execution_feedback()
        if self.state is RuntimeState.RECOVERY:
            return self._sync_state()
        self._apply_world_observations()
        if self.state is RuntimeState.RECOVERY:
            return self._sync_state()
        self.session.advance(1)
        self._start_ready_plan()
        if self.session.state is not SessionState.RECOVERY:
            self._create_successor()
        if self.session.state is not SessionState.RECOVERY:
            self._request_stop_once()
        if self.session.state is not SessionState.RECOVERY:
            try:
                self.execution_backend.advance(1)
            except Exception as exc:
                self._fail_active(f"execution backend advance failed: {exc}")
        return self._sync_state()

    def run_until_stable(
        self,
        *,
        max_steps: int = 1000,
        timeout_seconds: float = 5.0,
    ) -> RuntimeState:
        if max_steps < 1:
            raise ValueError("max_steps must be positive")
        if not isfinite(timeout_seconds) or timeout_seconds < 0.0:
            raise ValueError("timeout_seconds must be finite and non-negative")
        deadline = monotonic() + timeout_seconds
        for _ in range(max_steps):
            state = self.step()
            if state in {
                RuntimeState.IDLE,
                RuntimeState.WAITING_FOR_SCENE,
                RuntimeState.RECOVERY,
                RuntimeState.BLOCKED,
            } and not self._pending_initial and not self._pending_worlds:
                return state
            executor = self.session.executor
            if executor.running_count or executor.pending_count:
                remaining = deadline - monotonic()
                if remaining <= 0.0 or not executor.wait_for_completion(remaining):
                    raise TimeoutError("runtime did not receive planning completion before timeout")
            if monotonic() > deadline:
                raise TimeoutError("runtime did not stabilize before timeout")
        raise RuntimeError("runtime did not stabilize within max_steps")

    def snapshot(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "closed": self._closed,
            "step_order": self.STEP_ORDER,
            "active_plan_id": None if self._active_plan is None else self._active_plan.plan_id,
            "active_execution_id": self._active_execution_id,
            "pending_initial_requests": len(self._pending_initial),
            "pending_world_observations": len(self._pending_worlds),
            "metrics": self.metrics.report(),
            "session": self.session.snapshot(),
        }

    def shutdown(self) -> None:
        if self._closed:
            return
        if self._owner_thread_id is None:
            self._owner_thread_id = get_ident()
        elif get_ident() != self._owner_thread_id:
            raise RuntimeError("runtime shutdown must run on the step owner thread")
        if self.owns_execution_backend:
            self.execution_backend.shutdown()
        if self.owns_executor:
            self.session.executor.shutdown(wait=True, cancel_pending=True)
        self._closed = True
        self.state = RuntimeState.SHUTDOWN
        self._event("runtime_shutdown", "SHUTDOWN")

    def close(self) -> None:
        self.shutdown()

    def __enter__(self) -> ContinuousPlanningRuntime:
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()


__all__ = [
    "ContinuousPlanningRuntime",
    "RuntimeEvent",
    "RuntimeMetrics",
    "RuntimeState",
    "SuccessorRequestFactory",
]
