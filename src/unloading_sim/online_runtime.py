"""Deterministic orchestration for planning and execution control planes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import Counter, OrderedDict, deque
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from threading import get_ident
from time import monotonic
from types import MappingProxyType
from typing import Any, Callable, Mapping

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
    WAITING_FOR_OBSERVATION = "WAITING_FOR_OBSERVATION"
    WAITING_FOR_SCENE = "WAITING_FOR_SCENE"
    RECOVERY = "RECOVERY"
    BLOCKED = "BLOCKED"
    SHUTDOWN = "SHUTDOWN"


class ObservationAuthority(str, Enum):
    """Safety meaning of a world observation, independent of its source name."""

    AUTHORITATIVE = "AUTHORITATIVE"
    PREDICTED = "PREDICTED"


class RuntimeIngressStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    DUPLICATE = "DUPLICATE"
    STALE = "STALE"
    COALESCED = "COALESCED"
    BACKPRESSURED = "BACKPRESSURED"
    REJECTED = "REJECTED"
    CONFLICT = "CONFLICT"


@dataclass(frozen=True)
class RuntimeIngressResult:
    status: RuntimeIngressStatus
    port: str
    message: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", RuntimeIngressStatus(self.status))
        if not self.port:
            raise ValueError("runtime ingress port must be non-empty")
        object.__setattr__(self, "metadata", _freeze(self.metadata))

    @property
    def accepted(self) -> bool:
        return self.status in {
            RuntimeIngressStatus.ACCEPTED,
            RuntimeIngressStatus.DUPLICATE,
            RuntimeIngressStatus.COALESCED,
        }


@dataclass(frozen=True)
class WorldObservation:
    """Provider-neutral immutable world observation envelope.

    ``source`` is diagnostic only. Safety semantics come exclusively from
    ``authority`` and the producer/stream/epoch/sequence identity.
    """

    authority: ObservationAuthority
    producer_id: str
    stream_id: str
    producer_epoch: int
    sequence: int
    observed_at_monotonic_seconds: float
    snapshot: PlanningWorldSnapshot
    source: str = "unspecified"
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        authority = ObservationAuthority(self.authority)
        observed = float(self.observed_at_monotonic_seconds)
        if not self.producer_id or not self.stream_id:
            raise ValueError("world observation producer_id and stream_id must be non-empty")
        if self.producer_epoch < 0 or self.sequence < 0:
            raise ValueError("world observation epoch and sequence must be non-negative")
        if not isfinite(observed) or observed < 0.0:
            raise ValueError("world observation time must be finite and non-negative")
        if not isinstance(self.snapshot, PlanningWorldSnapshot):
            raise TypeError("world observation requires a PlanningWorldSnapshot")
        if not self.source:
            raise ValueError("world observation source must be non-empty")
        object.__setattr__(self, "authority", authority)
        object.__setattr__(self, "observed_at_monotonic_seconds", observed)
        object.__setattr__(self, "metadata", _freeze(self.metadata))

    @property
    def stream_key(self) -> tuple[str, str]:
        return (self.producer_id, self.stream_id)

    @property
    def sequence_domain(self) -> tuple[str, str, int]:
        return (self.producer_id, self.stream_id, self.producer_epoch)


@dataclass
class _ObservationCursor:
    producer_epoch: int
    sequence: int
    snapshot_fingerprint: str


@dataclass
class _FeedbackDomainState:
    last_sequence: int = -1
    last_status: ExecutionFeedbackStatus | None = None
    last_progress: float = 0.0
    last_feedback: ExecutionFeedback | None = None
    terminal_feedback: ExecutionFeedback | None = None


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
        self.ingress_accepted_count = 0
        self.ingress_stale_count = 0
        self.ingress_duplicate_count = 0
        self.ingress_conflict_count = 0
        self.ingress_coalesced_count = 0
        self.ingress_backpressured_count = 0
        self.ingress_rejected_count = 0
        self.ingress_count_by_port_and_status: Counter[str] = Counter()
        self.stale_execution_feedback_count = 0
        self.stop_waiting_observation_steps = 0
        self.stop_waiting_observation_seconds = 0.0

    def record_ingress(self, port: str, status: RuntimeIngressStatus) -> None:
        name = status.value.lower()
        attribute = f"ingress_{name}_count"
        if hasattr(self, attribute):
            setattr(self, attribute, getattr(self, attribute) + 1)
        self.ingress_count_by_port_and_status[f"{port}:{status.value}"] += 1

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
            "ingress_accepted_count": self.ingress_accepted_count,
            "ingress_stale_count": self.ingress_stale_count,
            "ingress_duplicate_count": self.ingress_duplicate_count,
            "ingress_conflict_count": self.ingress_conflict_count,
            "ingress_coalesced_count": self.ingress_coalesced_count,
            "ingress_backpressured_count": self.ingress_backpressured_count,
            "ingress_rejected_count": self.ingress_rejected_count,
            "ingress_count_by_port_and_status": dict(
                sorted(self.ingress_count_by_port_and_status.items())
            ),
            "stale_execution_feedback_count": self.stale_execution_feedback_count,
            "stop_waiting_observation_steps": self.stop_waiting_observation_steps,
            "stop_waiting_observation_seconds": self.stop_waiting_observation_seconds,
            "production_throughput": None,
        }


class ContinuousPlanningRuntime:
    """Single-thread-owned deterministic planning/execution orchestrator."""

    STEP_ORDER = (
        "ingress_fault",
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
        initial_request_capacity: int = 16,
        world_observation_capacity: int = 16,
        execution_feedback_capacity: int = 64,
        observation_stream_capacity: int = 32,
        terminal_feedback_history_capacity: int = 64,
        max_initial_requests_per_step: int = 1,
        max_world_observations_per_step: int = 1,
        max_execution_feedback_per_step: int = 16,
        clock: Callable[[], float] = monotonic,
    ) -> None:
        if not isinstance(session, ContinuousPlanningSession):
            raise TypeError("session must be a ContinuousPlanningSession")
        if not isinstance(execution_backend, ExecutionBackend):
            raise TypeError("execution_backend must implement ExecutionBackend")
        if successor_factory is not None and not isinstance(successor_factory, SuccessorRequestFactory):
            raise TypeError("successor_factory must implement SuccessorRequestFactory")
        capacities = {
            "initial_request_capacity": initial_request_capacity,
            "world_observation_capacity": world_observation_capacity,
            "execution_feedback_capacity": execution_feedback_capacity,
            "observation_stream_capacity": observation_stream_capacity,
            "terminal_feedback_history_capacity": terminal_feedback_history_capacity,
            "max_initial_requests_per_step": max_initial_requests_per_step,
            "max_world_observations_per_step": max_world_observations_per_step,
            "max_execution_feedback_per_step": max_execution_feedback_per_step,
        }
        if any(not isinstance(value, int) or value < 1 for value in capacities.values()):
            raise ValueError("runtime ingress capacities and per-step limits must be positive integers")
        if not callable(clock):
            raise TypeError("runtime clock must be callable")
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
        self._initial_request_capacity = initial_request_capacity
        self._world_observation_capacity = world_observation_capacity
        self._execution_feedback_capacity = execution_feedback_capacity
        self._observation_stream_capacity = observation_stream_capacity
        self._terminal_feedback_history_capacity = terminal_feedback_history_capacity
        self._max_initial_requests_per_step = max_initial_requests_per_step
        self._max_world_observations_per_step = max_world_observations_per_step
        self._max_execution_feedback_per_step = max_execution_feedback_per_step
        self._clock = clock
        self._pending_initial: deque[PlanningRequest] = deque()
        self._pending_worlds: deque[WorldObservation] = deque()
        self._pending_feedback: deque[ExecutionFeedback] = deque()
        self._observation_cursors: OrderedDict[tuple[str, str], _ObservationCursor] = OrderedDict()
        self._latest_world: PlanningWorldSnapshot | None = None
        self._latest_world_observation: WorldObservation | None = None
        self._active_plan: PlanEnvelope | None = None
        self._active_execution_id: str | None = None
        self._active_feedback_domain: tuple[str, int, str] | None = None
        self._feedback_domains: OrderedDict[tuple[str, int, str], _FeedbackDomainState] = OrderedDict()
        self._terminal_feedback: OrderedDict[tuple[str, int, str], ExecutionFeedback] = OrderedDict()
        self._pending_stopped_feedback: ExecutionFeedback | None = None
        self._pending_stop_observation: WorldObservation | None = None
        self._stop_wait_started_at: float | None = None
        self._stop_wait_last_at: float | None = None
        self._pending_ingress_fault: str | None = None
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

    def _ingress_result(
        self,
        status: RuntimeIngressStatus,
        port: str,
        message: str,
        **metadata: Any,
    ) -> RuntimeIngressResult:
        self.metrics.record_ingress(port, status)
        return RuntimeIngressResult(status, port, message, metadata)

    def submit_initial(self, request: PlanningRequest) -> RuntimeIngressResult:
        self._assert_open()
        if not isinstance(request, PlanningRequest):
            raise TypeError("initial request must be a PlanningRequest")
        if len(self._pending_initial) >= self._initial_request_capacity:
            return self._ingress_result(
                RuntimeIngressStatus.BACKPRESSURED,
                "initial_request",
                "initial request ingress is full",
                request_id=request.request_id,
            )
        self._pending_initial.append(request)
        return self._ingress_result(
            RuntimeIngressStatus.ACCEPTED,
            "initial_request",
            "initial request accepted",
            request_id=request.request_id,
        )

    def observe_world(self, observation: WorldObservation) -> RuntimeIngressResult:
        self._assert_open()
        if not isinstance(observation, WorldObservation):
            raise TypeError("world observation must be a WorldObservation envelope")
        if observation.authority is not ObservationAuthority.AUTHORITATIVE:
            return self._ingress_result(
                RuntimeIngressStatus.REJECTED,
                "world_observation",
                "only AUTHORITATIVE observations can update runtime world state",
                producer_id=observation.producer_id,
                stream_id=observation.stream_id,
            )
        key = observation.stream_key
        cursor = self._observation_cursors.get(key)
        if cursor is None and len(self._observation_cursors) >= self._observation_stream_capacity:
            return self._ingress_result(
                RuntimeIngressStatus.BACKPRESSURED,
                "world_observation",
                "world observation stream capacity is full",
                producer_id=observation.producer_id,
                stream_id=observation.stream_id,
            )
        if cursor is not None:
            if observation.producer_epoch < cursor.producer_epoch:
                return self._ingress_result(
                    RuntimeIngressStatus.STALE,
                    "world_observation",
                    "observation producer epoch is stale",
                )
            if observation.producer_epoch == cursor.producer_epoch:
                if observation.sequence < cursor.sequence:
                    return self._ingress_result(
                        RuntimeIngressStatus.STALE,
                        "world_observation",
                        "observation sequence is stale",
                    )
                if observation.sequence == cursor.sequence:
                    if observation.snapshot.fingerprint == cursor.snapshot_fingerprint:
                        return self._ingress_result(
                            RuntimeIngressStatus.DUPLICATE,
                            "world_observation",
                            "identical observation already accepted",
                        )
                    self._pending_ingress_fault = (
                        "same world observation identity has conflicting content"
                    )
                    return self._ingress_result(
                        RuntimeIngressStatus.CONFLICT,
                        "world_observation",
                        self._pending_ingress_fault,
                    )

        pending_index = next(
            (
                index
                for index, pending in enumerate(self._pending_worlds)
                if pending.stream_key == key
            ),
            None,
        )
        if pending_index is None and len(self._pending_worlds) >= self._world_observation_capacity:
            return self._ingress_result(
                RuntimeIngressStatus.BACKPRESSURED,
                "world_observation",
                "world observation ingress is full",
            )
        status = RuntimeIngressStatus.ACCEPTED
        if pending_index is None:
            self._pending_worlds.append(observation)
        else:
            self._pending_worlds[pending_index] = observation
            status = RuntimeIngressStatus.COALESCED
        self._observation_cursors[key] = _ObservationCursor(
            observation.producer_epoch,
            observation.sequence,
            observation.snapshot.fingerprint,
        )
        self._observation_cursors.move_to_end(key)
        return self._ingress_result(
            status,
            "world_observation",
            "authoritative observation accepted",
            producer_id=observation.producer_id,
            stream_id=observation.stream_id,
            producer_epoch=observation.producer_epoch,
            sequence=observation.sequence,
            source=observation.source,
        )

    def _fail_active(
        self,
        message: str,
        *,
        reason: ReplanReason = ReplanReason.EXECUTION_FAILED,
        feedback: ExecutionFeedback | None = None,
    ) -> None:
        self.session.enter_recovery(
            message,
            reason=reason,
            plan_id=None if self._active_plan is None else self._active_plan.plan_id,
            actual_boundary=None if feedback is None else feedback.current_boundary,
            execution_id=self._active_execution_id,
        )
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

    @staticmethod
    def _feedback_domain(feedback: ExecutionFeedback) -> tuple[str, int, str]:
        return (
            feedback.feedback_stream_id,
            feedback.producer_epoch,
            feedback.execution_id,
        )

    def _ignore_feedback(
        self,
        status: RuntimeIngressStatus,
        feedback: ExecutionFeedback,
        message: str,
    ) -> None:
        self.metrics.record_ingress("execution_feedback", status)
        if status is RuntimeIngressStatus.DUPLICATE:
            self.metrics.duplicate_feedback_ignored_count += 1
        elif status is RuntimeIngressStatus.STALE:
            self.metrics.stale_execution_feedback_count += 1
        self._event(
            f"{status.value.lower()}_execution_feedback_ignored",
            status.value,
            plan_id=feedback.plan_id,
            execution_id=feedback.execution_id,
            message=message,
        )

    def _remember_terminal(
        self,
        domain: tuple[str, int, str],
        feedback: ExecutionFeedback,
    ) -> None:
        self._terminal_feedback[domain] = feedback
        self._terminal_feedback.move_to_end(domain)
        while len(self._terminal_feedback) > self._terminal_feedback_history_capacity:
            expired_domain, _ = self._terminal_feedback.popitem(last=False)
            state = self._feedback_domains.get(expired_domain)
            if state is not None and state.terminal_feedback is not None:
                del self._feedback_domains[expired_domain]

    def _transition_allowed(
        self,
        previous: ExecutionFeedbackStatus | None,
        current: ExecutionFeedbackStatus,
    ) -> bool:
        acknowledgement = self.execution_backend.capabilities.supports_command_acknowledgement
        if previous is None:
            if acknowledgement:
                return current in {
                    ExecutionFeedbackStatus.ACCEPTED,
                    ExecutionFeedbackStatus.REJECTED,
                    ExecutionFeedbackStatus.FAULTED,
                }
            return current is not ExecutionFeedbackStatus.ACCEPTED
        allowed = {
            ExecutionFeedbackStatus.ACCEPTED: {
                ExecutionFeedbackStatus.RUNNING,
                ExecutionFeedbackStatus.STOPPING,
                ExecutionFeedbackStatus.SUCCEEDED,
                ExecutionFeedbackStatus.FAILED,
                ExecutionFeedbackStatus.REJECTED,
                ExecutionFeedbackStatus.DEVIATED,
                ExecutionFeedbackStatus.FAULTED,
            },
            ExecutionFeedbackStatus.RUNNING: {
                ExecutionFeedbackStatus.RUNNING,
                ExecutionFeedbackStatus.STOPPING,
                ExecutionFeedbackStatus.STOPPED,
                ExecutionFeedbackStatus.SUCCEEDED,
                ExecutionFeedbackStatus.FAILED,
                ExecutionFeedbackStatus.DEVIATED,
                ExecutionFeedbackStatus.FAULTED,
            },
            ExecutionFeedbackStatus.STOPPING: {
                ExecutionFeedbackStatus.STOPPING,
                ExecutionFeedbackStatus.STOPPED,
                ExecutionFeedbackStatus.FAILED,
                ExecutionFeedbackStatus.DEVIATED,
                ExecutionFeedbackStatus.FAULTED,
            },
        }
        return current in allowed.get(previous, set())

    def _clear_active_execution(self) -> None:
        if self._active_plan is not None:
            plan_id = self._active_plan.plan_id
            self._successor_attempted_plan_ids.discard(plan_id)
            self._stop_requested_plan_ids.discard(plan_id)
        self._active_plan = None
        self._active_execution_id = None
        self._active_feedback_domain = None

    def _try_reconcile_pending_stop(self) -> None:
        feedback = self._pending_stopped_feedback
        observation = self._pending_stop_observation
        if feedback is None or observation is None:
            return
        stopped_world = observation.snapshot
        boundary = feedback.current_boundary
        if len(stopped_world.current_q) != len(boundary.q):
            self._invalid_feedback(
                "stopped world and boundary DOF are incompatible",
                feedback,
            )
            return
        if any(abs(actual - stopped) > 1e-6 for actual, stopped in zip(stopped_world.current_q, boundary.q)):
            return
        try:
            self.session.acknowledge_stop(
                stopped_world,
                boundary,
                feedback.message,
                execution_id=feedback.execution_id,
            )
        except Exception as exc:
            self._invalid_feedback(f"stop acknowledgement rejected: {exc}", feedback)
            return
        if self._stop_wait_last_at is not None:
            self.metrics.stop_waiting_observation_seconds += max(
                0.0,
                float(self._clock()) - self._stop_wait_last_at,
            )
        self._pending_stopped_feedback = None
        self._pending_stop_observation = None
        self._stop_wait_started_at = None
        self._stop_wait_last_at = None
        self._clear_active_execution()

    def _consume_feedback(self, feedback: ExecutionFeedback) -> None:
        if not isinstance(feedback, ExecutionFeedback):
            self._invalid_feedback(
                f"expected ExecutionFeedback, got {type(feedback).__name__}"
            )
            return
        domain = self._feedback_domain(feedback)
        previous_terminal = self._terminal_feedback.get(domain)
        if previous_terminal is not None:
            if self._identical_feedback(previous_terminal, feedback):
                self._ignore_feedback(
                    RuntimeIngressStatus.DUPLICATE,
                    feedback,
                    "identical terminal feedback already processed",
                )
            elif feedback.feedback_sequence < previous_terminal.feedback_sequence:
                self._ignore_feedback(
                    RuntimeIngressStatus.STALE,
                    feedback,
                    "late feedback predates the recorded terminal feedback",
                )
            else:
                self._invalid_feedback("conflicting terminal feedback", feedback)
            return
        if (
            self._active_plan is None
            or feedback.plan_id != self._active_plan.plan_id
            or feedback.execution_id != self._active_execution_id
        ):
            known = self._feedback_domains.get(domain)
            if known is not None and feedback.feedback_sequence <= known.last_sequence:
                self._ignore_feedback(
                    RuntimeIngressStatus.STALE,
                    feedback,
                    "late feedback belongs to a prior execution",
                )
                return
            self._invalid_feedback("feedback plan_id or execution_id does not match", feedback)
            return
        if self._active_feedback_domain is None:
            self._active_feedback_domain = domain
        elif domain != self._active_feedback_domain:
            self._invalid_feedback("feedback stream or epoch changed during execution", feedback)
            return
        state = self._feedback_domains.setdefault(domain, _FeedbackDomainState())
        if feedback.feedback_sequence == state.last_sequence:
            if state.last_feedback is not None and self._identical_feedback(state.last_feedback, feedback):
                self._ignore_feedback(
                    RuntimeIngressStatus.DUPLICATE,
                    feedback,
                    "identical feedback already processed",
                )
            else:
                self._invalid_feedback("same feedback identity has conflicting content", feedback)
            return
        if feedback.feedback_sequence < state.last_sequence:
            self._ignore_feedback(
                RuntimeIngressStatus.STALE,
                feedback,
                "feedback sequence is stale within its execution domain",
            )
            return
        if not self._transition_allowed(state.last_status, feedback.status):
            self._invalid_feedback(
                f"illegal feedback transition {state.last_status} -> {feedback.status.value}",
                feedback,
            )
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
        previous_progress = state.last_progress
        if feedback.status is ExecutionFeedbackStatus.RUNNING and feedback.progress < previous_progress:
            self._invalid_feedback("RUNNING progress moved backwards", feedback)
            return
        state.last_sequence = feedback.feedback_sequence
        state.last_status = feedback.status
        state.last_progress = max(previous_progress, feedback.progress)
        state.last_feedback = feedback
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
        state.terminal_feedback = feedback
        self._remember_terminal(domain, feedback)
        if feedback.status is ExecutionFeedbackStatus.STOPPED:
            if self.session.execution.state is not ExecutionState.STOPPING:
                self._invalid_feedback("STOPPED feedback received outside STOPPING", feedback)
                return
            self._pending_stopped_feedback = feedback
            if self._stop_wait_started_at is None:
                now = float(self._clock())
                self._stop_wait_started_at = now
                self._stop_wait_last_at = now
            self._try_reconcile_pending_stop()
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
            self._clear_active_execution()
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
            self._clear_active_execution()
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
            self._clear_active_execution()
            return
        self.metrics.execution_failure_count += 1
        self._fail_active(feedback.message or feedback.status.value, feedback=feedback)
        self._clear_active_execution()

    def _poll_execution_feedback(self) -> None:
        while len(self._pending_feedback) < self._execution_feedback_capacity:
            try:
                feedback = self.execution_backend.poll()
            except Exception as exc:
                self._invalid_feedback(f"execution backend poll failed: {exc}")
                return
            if feedback is None:
                break
            if not isinstance(feedback, ExecutionFeedback):
                self._invalid_feedback(
                    f"expected ExecutionFeedback, got {type(feedback).__name__}"
                )
                return
            self._pending_feedback.append(feedback)
            self.metrics.record_ingress(
                "execution_feedback",
                RuntimeIngressStatus.ACCEPTED,
            )
        if len(self._pending_feedback) >= self._execution_feedback_capacity:
            self.metrics.record_ingress(
                "execution_feedback",
                RuntimeIngressStatus.BACKPRESSURED,
            )
            self._event(
                "execution_feedback_backpressured",
                RuntimeIngressStatus.BACKPRESSURED.value,
                queued=len(self._pending_feedback),
            )
        processed = 0
        while self._pending_feedback and processed < self._max_execution_feedback_per_step:
            self._consume_feedback(self._pending_feedback.popleft())
            processed += 1
            if self.state is RuntimeState.RECOVERY:
                return

    def _apply_world_observations(self) -> None:
        processed = 0
        while self._pending_worlds and processed < self._max_world_observations_per_step:
            observation = self._pending_worlds.popleft()
            snapshot = observation.snapshot
            try:
                self.session.update_scene(snapshot)
            except Exception as exc:
                self._fail_active(f"world observation rejected: {exc}")
                return
            self._latest_world = snapshot
            self._latest_world_observation = observation
            if self.session.state is SessionState.STOPPING:
                self._pending_stop_observation = observation
                self._try_reconcile_pending_stop()
                if self.state is RuntimeState.RECOVERY:
                    return
            self._event(
                "world_observed",
                "WORLD_OBSERVATION",
                fingerprint=snapshot.fingerprint,
                authority=observation.authority.value,
                producer_id=observation.producer_id,
                stream_id=observation.stream_id,
                producer_epoch=observation.producer_epoch,
                sequence=observation.sequence,
                source=observation.source,
            )
            processed += 1

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
        self._active_feedback_domain = None
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
        elif (
            self._pending_stopped_feedback is not None
            and self.session.state is SessionState.STOPPING
        ):
            self.state = RuntimeState.WAITING_FOR_OBSERVATION
        else:
            self.state = RuntimeState(self.session.state.value)
        return self.state

    def _record_stop_waiting(self) -> None:
        if self._pending_stopped_feedback is None:
            return
        now = float(self._clock())
        if self._stop_wait_last_at is None:
            self._stop_wait_last_at = now
        self.metrics.stop_waiting_observation_seconds += max(
            0.0,
            now - self._stop_wait_last_at,
        )
        self._stop_wait_last_at = now
        self.metrics.stop_waiting_observation_steps += 1

    def step(self) -> RuntimeState:
        self._assert_open()
        if self._pending_ingress_fault is not None:
            message, self._pending_ingress_fault = self._pending_ingress_fault, None
            self._fail_active(message, reason=ReplanReason.PLAN_INVALIDATED)
            return self._sync_state()
        submitted = 0
        while self._pending_initial and submitted < self._max_initial_requests_per_step:
            request = self._pending_initial.popleft()
            try:
                self.session.submit(request)
            except Exception as exc:
                self._fail_active(
                    f"initial planning request rejected: {exc}",
                    reason=ReplanReason.PLANNER_FAILURE,
                )
                return self._sync_state()
            submitted += 1
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
        self._record_stop_waiting()
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
                RuntimeState.WAITING_FOR_OBSERVATION,
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
            "pending_execution_feedback": len(self._pending_feedback),
            "latest_world_fingerprint": (
                None if self._latest_world is None else self._latest_world.fingerprint
            ),
            "ingress_capacities": {
                "initial_requests": self._initial_request_capacity,
                "world_observations": self._world_observation_capacity,
                "execution_feedback": self._execution_feedback_capacity,
            },
            "terminal_feedback_history": len(self._terminal_feedback),
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
    "ObservationAuthority",
    "RuntimeEvent",
    "RuntimeIngressResult",
    "RuntimeIngressStatus",
    "RuntimeMetrics",
    "RuntimeState",
    "SuccessorRequestFactory",
    "WorldObservation",
]
