"""Deterministic orchestration for planning and execution control planes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import Counter, OrderedDict, deque
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from threading import Lock, get_ident
from time import monotonic
from types import MappingProxyType
from typing import Any, Callable, Mapping

from .online_execution import (
    ExecutionBackend,
    ExecutionBackendHealth,
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
from .online_journal import BoundedEventJournal, EventJournalRead


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


class StopCommandState(str, Enum):
    """Command-side state; it deliberately does not imply physical stop."""

    REQUIRED = "REQUIRED"
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    ERROR = "ERROR"


@dataclass
class _StopLifecycle:
    attempt_id: str
    execution_id: str
    plan_id: str
    trigger_reason: str
    trigger_message: str
    command_state: StopCommandState = StopCommandState.REQUIRED
    command_id: str | None = None
    command_message: str = ""
    failure_reason: str | None = None
    failure_message: str = ""
    execution_outcome: ExecutionFeedbackStatus | None = None
    execution_outcome_message: str = ""
    stopped_confirmed: bool = False
    stopped_boundary: Any = None
    confirmation_stream_id: str | None = None
    confirmation_producer_epoch: int | None = None
    world_reconciled: bool = False
    recovery_required: bool = False
    recovery_resume_authorized: bool = False


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
class _ReceivedIngress:
    message: PlanningRequest | WorldObservation
    received_at_monotonic_seconds: float


@dataclass(frozen=True)
class _MailboxDrain:
    requests: tuple[_ReceivedIngress, ...]
    observations: tuple[_ReceivedIngress, ...]
    conflict: str | None = None


@dataclass(frozen=True)
class _ReceivedExecutionFeedback:
    feedback: ExecutionFeedback
    received_at_monotonic_seconds: float


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


class WatchdogTerminalAction(str, Enum):
    WAITING_FOR_SCENE = "WAITING_FOR_SCENE"
    RECOVERY = "RECOVERY"
    BLOCKED = "BLOCKED"


@dataclass(frozen=True)
class RuntimeWatchdogPolicy:
    """Local monotonic-time safety policy; ``None`` disables a deadline."""

    scene_freshness_timeout_seconds: float | None = 30.0
    execution_feedback_timeout_seconds: float | None = 5.0
    stop_ack_timeout_seconds: float | None = 5.0
    stopped_observation_timeout_seconds: float | None = 10.0
    planning_timeout_seconds: float | None = 30.0
    scene_stale_action: WatchdogTerminalAction = WatchdogTerminalAction.WAITING_FOR_SCENE
    stopped_observation_timeout_action: WatchdogTerminalAction = WatchdogTerminalAction.RECOVERY

    def __post_init__(self) -> None:
        for name in (
            "scene_freshness_timeout_seconds",
            "execution_feedback_timeout_seconds",
            "stop_ack_timeout_seconds",
            "stopped_observation_timeout_seconds",
            "planning_timeout_seconds",
        ):
            value = getattr(self, name)
            if value is not None and (not isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite and non-negative, or None")
        object.__setattr__(self, "scene_stale_action", WatchdogTerminalAction(self.scene_stale_action))
        object.__setattr__(
            self,
            "stopped_observation_timeout_action",
            WatchdogTerminalAction(self.stopped_observation_timeout_action),
        )
        if self.stopped_observation_timeout_action is WatchdogTerminalAction.WAITING_FOR_SCENE:
            raise ValueError("stopped observation timeout action must be RECOVERY or BLOCKED")


class RuntimeMetrics:
    def __init__(self) -> None:
        self._ingress_lock = Lock()
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
        self.scene_stale_count = 0
        self.execution_feedback_timeout_count = 0
        self.stop_ack_timeout_count = 0
        self.stop_observation_timeout_count = 0
        self.planning_timeout_count = 0
        self.backend_unhealthy_count = 0
        self.event_history_gap_count = 0
        self.event_dropped_count = 0
        self.event_overflow_count = 0
        self.accepted_ingress_discarded_count = 0
        self.stop_unconfirmed_count = 0

    def record_ingress(self, port: str, status: RuntimeIngressStatus) -> None:
        with self._ingress_lock:
            name = status.value.lower()
            attribute = f"ingress_{name}_count"
            if hasattr(self, attribute):
                setattr(self, attribute, getattr(self, attribute) + 1)
            self.ingress_count_by_port_and_status[f"{port}:{status.value}"] += 1

    def report(self) -> dict[str, Any]:
        with self._ingress_lock:
            ingress = {
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
            }
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
            **ingress,
            "stale_execution_feedback_count": self.stale_execution_feedback_count,
            "stop_waiting_observation_steps": self.stop_waiting_observation_steps,
            "stop_waiting_observation_seconds": self.stop_waiting_observation_seconds,
            "scene_stale_count": self.scene_stale_count,
            "execution_feedback_timeout_count": self.execution_feedback_timeout_count,
            "stop_ack_timeout_count": self.stop_ack_timeout_count,
            "stop_observation_timeout_count": self.stop_observation_timeout_count,
            "planning_timeout_count": self.planning_timeout_count,
            "backend_unhealthy_count": self.backend_unhealthy_count,
            "event_history_gap_count": self.event_history_gap_count,
            "event_dropped_count": self.event_dropped_count,
            "event_overflow_count": self.event_overflow_count,
            "accepted_ingress_discarded_count": self.accepted_ingress_discarded_count,
            "stop_unconfirmed_count": self.stop_unconfirmed_count,
            "production_throughput": None,
        }


class RuntimeIngressMailbox:
    """Thread-safe MPSC ingress; only the runtime step owner drains it."""

    def __init__(
        self,
        *,
        initial_request_capacity: int,
        world_observation_capacity: int,
        observation_stream_capacity: int,
        clock: Callable[[], float],
        record_ingress: Callable[[str, RuntimeIngressStatus], None],
    ) -> None:
        self._lock = Lock()
        self._closed = False
        self._requests: deque[_ReceivedIngress] = deque()
        self._observations: deque[_ReceivedIngress] = deque()
        self._cursors: OrderedDict[tuple[str, str], _ObservationCursor] = OrderedDict()
        self._conflict: str | None = None
        self._request_capacity = initial_request_capacity
        self._observation_capacity = world_observation_capacity
        self._stream_capacity = observation_stream_capacity
        self._clock = clock
        self._record_ingress = record_ingress

    def _result(
        self,
        status: RuntimeIngressStatus,
        port: str,
        message: str,
        **metadata: Any,
    ) -> RuntimeIngressResult:
        self._record_ingress(port, status)
        return RuntimeIngressResult(status, port, message, metadata)

    @property
    def closed(self) -> bool:
        with self._lock:
            return self._closed

    def submit_initial(self, request: PlanningRequest) -> RuntimeIngressResult:
        if not isinstance(request, PlanningRequest):
            raise TypeError("initial request must be a PlanningRequest")
        with self._lock:
            if self._closed:
                return self._result(RuntimeIngressStatus.REJECTED, "initial_request", "mailbox is closed")
            if len(self._requests) >= self._request_capacity:
                return self._result(
                    RuntimeIngressStatus.BACKPRESSURED,
                    "initial_request",
                    "initial request ingress is full",
                    request_id=request.request_id,
                )
            self._requests.append(_ReceivedIngress(request, float(self._clock())))
            return self._result(
                RuntimeIngressStatus.ACCEPTED,
                "initial_request",
                "initial request accepted",
                request_id=request.request_id,
            )

    def observe_world(self, observation: WorldObservation) -> RuntimeIngressResult:
        if not isinstance(observation, WorldObservation):
            raise TypeError("world observation must be a WorldObservation envelope")
        with self._lock:
            if self._closed:
                return self._result(RuntimeIngressStatus.REJECTED, "world_observation", "mailbox is closed")
            if observation.authority is not ObservationAuthority.AUTHORITATIVE:
                return self._result(
                    RuntimeIngressStatus.REJECTED,
                    "world_observation",
                    "only AUTHORITATIVE observations can update runtime world state",
                )
            key = observation.stream_key
            cursor = self._cursors.get(key)
            if cursor is None and len(self._cursors) >= self._stream_capacity:
                return self._result(
                    RuntimeIngressStatus.BACKPRESSURED,
                    "world_observation",
                    "world observation stream capacity is full",
                )
            if cursor is not None:
                if observation.producer_epoch < cursor.producer_epoch:
                    return self._result(RuntimeIngressStatus.STALE, "world_observation", "observation producer epoch is stale")
                if observation.producer_epoch == cursor.producer_epoch:
                    if observation.sequence < cursor.sequence:
                        return self._result(RuntimeIngressStatus.STALE, "world_observation", "observation sequence is stale")
                    if observation.sequence == cursor.sequence:
                        if observation.snapshot.fingerprint == cursor.snapshot_fingerprint:
                            return self._result(RuntimeIngressStatus.DUPLICATE, "world_observation", "identical observation already accepted")
                        self._conflict = "same world observation identity has conflicting content"
                        return self._result(RuntimeIngressStatus.CONFLICT, "world_observation", self._conflict)
            pending_index = next(
                (
                    index
                    for index, pending in enumerate(self._observations)
                    if isinstance(pending.message, WorldObservation)
                    and pending.message.stream_key == key
                ),
                None,
            )
            if pending_index is None and len(self._observations) >= self._observation_capacity:
                return self._result(RuntimeIngressStatus.BACKPRESSURED, "world_observation", "world observation ingress is full")
            received = _ReceivedIngress(observation, float(self._clock()))
            status = RuntimeIngressStatus.ACCEPTED
            if pending_index is None:
                self._observations.append(received)
            else:
                self._observations[pending_index] = received
                status = RuntimeIngressStatus.COALESCED
            self._cursors[key] = _ObservationCursor(
                observation.producer_epoch,
                observation.sequence,
                observation.snapshot.fingerprint,
            )
            self._cursors.move_to_end(key)
            return self._result(
                status,
                "world_observation",
                "authoritative observation accepted",
                producer_id=observation.producer_id,
                stream_id=observation.stream_id,
                producer_epoch=observation.producer_epoch,
                sequence=observation.sequence,
                source=observation.source,
            )

    def drain(self, max_requests: int, max_observations: int) -> _MailboxDrain:
        with self._lock:
            requests = tuple(
                self._requests.popleft()
                for _ in range(min(max_requests, len(self._requests)))
            )
            observations = tuple(
                self._observations.popleft()
                for _ in range(min(max_observations, len(self._observations)))
            )
            conflict, self._conflict = self._conflict, None
            return _MailboxDrain(requests, observations, conflict)

    def pending_counts(self) -> tuple[int, int]:
        with self._lock:
            return len(self._requests), len(self._observations)

    def shutdown(self) -> tuple[int, int]:
        with self._lock:
            self._closed = True
            pending = (len(self._requests), len(self._observations))
            self._requests.clear()
            self._observations.clear()
            return pending


class ContinuousPlanningRuntime:
    """Single-thread-owned deterministic planning/execution orchestrator."""

    STEP_ORDER = (
        "mailbox_drain",
        "ingress_fault",
        "submit_initial",
        "execution_feedback",
        "world_observation",
        "watchdog",
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
        watchdog_policy: RuntimeWatchdogPolicy | None = None,
        event_journal_capacity: int = 2048,
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
            "event_journal_capacity": event_journal_capacity,
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
        self.events: BoundedEventJournal[RuntimeEvent] = BoundedEventJournal(
            event_journal_capacity
        )
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
        self.watchdog_policy = watchdog_policy or RuntimeWatchdogPolicy()
        if not isinstance(self.watchdog_policy, RuntimeWatchdogPolicy):
            raise TypeError("watchdog_policy must be a RuntimeWatchdogPolicy")
        self.ingress_mailbox = RuntimeIngressMailbox(
            initial_request_capacity=initial_request_capacity,
            world_observation_capacity=world_observation_capacity,
            observation_stream_capacity=observation_stream_capacity,
            clock=clock,
            record_ingress=self.metrics.record_ingress,
        )
        self._pending_initial: deque[_ReceivedIngress] = deque()
        self._pending_worlds: deque[_ReceivedIngress] = deque()
        self._pending_feedback: deque[_ReceivedExecutionFeedback] = deque()
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
        self._scene_received_at: float | None = None
        self._execution_started_at: float | None = None
        self._last_feedback_received_at: float | None = None
        self._stop_requested_at: float | None = None
        self._planning_task_id: str | None = None
        self._planning_started_at: float | None = None
        self._timed_out_planning_task_id: str | None = None
        self._watchdog_latches: set[str] = set()
        self._stop_unconfirmed = False
        self._stop_attempt_sequence = 0
        self._stop_lifecycle: _StopLifecycle | None = None
        self._active_stop_feedback_domain: tuple[str, int, str, str] | None = None
        self._stop_feedback_state = _FeedbackDomainState()
        self._stop_confirmation_history: OrderedDict[str, ExecutionFeedback] = OrderedDict()

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
        dropped_before = self.events.dropped_count
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
        if self.events.dropped_count != dropped_before:
            self.metrics.event_dropped_count += 1
            self.metrics.event_overflow_count += 1

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
        return self.ingress_mailbox.submit_initial(request)

    def observe_world(self, observation: WorldObservation) -> RuntimeIngressResult:
        return self.ingress_mailbox.observe_world(observation)

    @staticmethod
    def _requires_explicit_recovery(reason: ReplanReason) -> bool:
        return reason is not ReplanReason.SCENE_REVISION_CHANGED

    def _ensure_stop_lifecycle(
        self,
        reason: ReplanReason,
        message: str,
    ) -> _StopLifecycle:
        if self._active_plan is None or self._active_execution_id is None:
            raise RuntimeError("a stop lifecycle requires an active execution identity")
        lifecycle = self._stop_lifecycle
        if (
            lifecycle is not None
            and lifecycle.plan_id == self._active_plan.plan_id
            and lifecycle.execution_id == self._active_execution_id
            and not lifecycle.world_reconciled
        ):
            lifecycle.recovery_required = bool(
                lifecycle.recovery_required or self._requires_explicit_recovery(reason)
            )
            return lifecycle
        self._stop_attempt_sequence += 1
        lifecycle = _StopLifecycle(
            attempt_id=f"stop-attempt-{self._stop_attempt_sequence}",
            execution_id=self._active_execution_id,
            plan_id=self._active_plan.plan_id,
            trigger_reason=reason.value,
            trigger_message=message,
            recovery_required=self._requires_explicit_recovery(reason),
        )
        self._stop_lifecycle = lifecycle
        self._active_stop_feedback_domain = None
        self._stop_feedback_state = _FeedbackDomainState()
        self._pending_stopped_feedback = None
        self._stop_unconfirmed = True
        return lifecycle

    def _mark_stop_unconfirmed(
        self,
        message: str,
        *,
        reason: ReplanReason,
        command_state: StopCommandState | None = None,
    ) -> None:
        lifecycle = self._ensure_stop_lifecycle(reason, message)
        if command_state is not None:
            lifecycle.command_state = command_state
        lifecycle.failure_reason = reason.value
        lifecycle.failure_message = message
        lifecycle.recovery_required = True
        self.session.mark_stop_unconfirmed(reason, message)
        self._stop_unconfirmed = True
        self.metrics.stop_unconfirmed_count += 1
        self.state = RuntimeState.RECOVERY
        self._event(
            "runtime_stop_unconfirmed",
            reason.value,
            plan_id=lifecycle.plan_id,
            execution_id=lifecycle.execution_id,
            stop_attempt_id=lifecycle.attempt_id,
            command_id=lifecycle.command_id,
            message=message,
        )

    def _record_execution_outcome(self, feedback: ExecutionFeedback) -> None:
        lifecycle = self._stop_lifecycle
        if lifecycle is None:
            reason = (
                ReplanReason.EXECUTION_DEVIATION
                if feedback.status is ExecutionFeedbackStatus.DEVIATED
                else ReplanReason.EXECUTION_FAILED
            )
            if self.session.execution.state is ExecutionState.STOPPING:
                lifecycle = self._ensure_stop_lifecycle(reason, feedback.message or feedback.status.value)
        if lifecycle is not None:
            if lifecycle.execution_outcome is None:
                lifecycle.execution_outcome = feedback.status
                lifecycle.execution_outcome_message = feedback.message
            elif lifecycle.execution_outcome is not feedback.status:
                self._invalid_feedback("conflicting execution task terminal outcome", feedback)
                return
            lifecycle.recovery_required = True

    def _fail_active(
        self,
        message: str,
        *,
        reason: ReplanReason = ReplanReason.EXECUTION_FAILED,
        feedback: ExecutionFeedback | None = None,
    ) -> None:
        if (
            self._active_plan is not None
            and self.session.execution.state is ExecutionState.RUNNING
        ):
            self.session.begin_safety_stop(reason, message)
            self._ensure_stop_lifecycle(reason, message)
            self.state = RuntimeState.STOPPING
            self._event(
                "runtime_safety_stop_required",
                reason.value,
                plan_id=self._active_plan.plan_id,
                execution_id=self._active_execution_id,
                message=message,
            )
            return
        if self.session.execution.state is ExecutionState.STOPPING:
            lifecycle = self._ensure_stop_lifecycle(reason, message)
            lifecycle.recovery_required = bool(
                lifecycle.recovery_required or self._requires_explicit_recovery(reason)
            )
            self._event(
                "runtime_fault_during_stopping",
                reason.value,
                plan_id=lifecycle.plan_id,
                execution_id=lifecycle.execution_id,
                stop_attempt_id=lifecycle.attempt_id,
                message=message,
            )
            return
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
        self._execution_started_at = None
        self._last_feedback_received_at = None
        self._stop_requested_at = None
        self._stop_unconfirmed = bool(
            self._stop_lifecycle is not None
            and not self._stop_lifecycle.stopped_confirmed
        )
        self._watchdog_latches.discard("execution_feedback")
        self._watchdog_latches.discard("stop_ack")

    def _remember_stop_confirmation(self, feedback: ExecutionFeedback) -> None:
        assert feedback.stop_command_id is not None
        self._stop_confirmation_history[feedback.stop_command_id] = feedback
        self._stop_confirmation_history.move_to_end(feedback.stop_command_id)
        while len(self._stop_confirmation_history) > self._terminal_feedback_history_capacity:
            self._stop_confirmation_history.popitem(last=False)

    def _consume_stop_feedback(self, received: _ReceivedExecutionFeedback) -> None:
        feedback = received.feedback
        command_id = feedback.stop_command_id
        assert command_id is not None
        historical = self._stop_confirmation_history.get(command_id)
        if historical is not None:
            if self._identical_feedback(historical, feedback):
                self._ignore_feedback(
                    RuntimeIngressStatus.DUPLICATE,
                    feedback,
                    "identical stop confirmation already processed",
                )
            else:
                self.metrics.invalid_feedback_count += 1
                self._event(
                    "invalid_historical_stop_confirmation_ignored",
                    ReplanReason.EXECUTION_DEVIATION.value,
                    plan_id=feedback.plan_id,
                    execution_id=feedback.execution_id,
                    command_id=command_id,
                    message="conflicting confirmation belongs to a completed stop attempt",
                )
            return
        lifecycle = self._stop_lifecycle
        if (
            lifecycle is None
            or lifecycle.command_state is not StopCommandState.ACCEPTED
            or lifecycle.command_id != command_id
            or lifecycle.plan_id != feedback.plan_id
            or lifecycle.execution_id != feedback.execution_id
        ):
            self._invalid_feedback("stop feedback does not match the active stop request", feedback)
            return
        domain = (
            feedback.feedback_stream_id,
            feedback.producer_epoch,
            feedback.execution_id,
            command_id,
        )
        if self._active_feedback_domain is not None and (
            feedback.feedback_stream_id,
            feedback.producer_epoch,
            feedback.execution_id,
        ) != self._active_feedback_domain:
            self._invalid_feedback("stop feedback stream or epoch does not match the execution", feedback)
            return
        if self._active_stop_feedback_domain is None:
            self._active_stop_feedback_domain = domain
        elif domain != self._active_stop_feedback_domain:
            self._invalid_feedback("stop feedback identity changed during a stop attempt", feedback)
            return
        state = self._stop_feedback_state
        if feedback.feedback_sequence == state.last_sequence:
            if state.last_feedback is not None and self._identical_feedback(state.last_feedback, feedback):
                self._ignore_feedback(
                    RuntimeIngressStatus.DUPLICATE,
                    feedback,
                    "identical stop feedback already processed",
                )
            else:
                self._invalid_feedback("same stop feedback identity has conflicting content", feedback)
            return
        if feedback.feedback_sequence < state.last_sequence:
            self._ignore_feedback(
                RuntimeIngressStatus.STALE,
                feedback,
                "stop feedback sequence is stale",
            )
            return
        if state.last_status is None:
            allowed = {
                ExecutionFeedbackStatus.STOPPING,
                ExecutionFeedbackStatus.STOPPED,
            }
        else:
            allowed = (
                {ExecutionFeedbackStatus.STOPPING, ExecutionFeedbackStatus.STOPPED}
                if state.last_status is ExecutionFeedbackStatus.STOPPING
                else set()
            )
        if feedback.status not in allowed:
            self._invalid_feedback("illegal stop feedback transition", feedback)
            return
        if self._active_plan is not None and len(feedback.current_boundary.q) != len(
            self._active_plan.expected_start_boundary.q
        ):
            self._invalid_feedback("stop feedback boundary DOF does not match active plan", feedback)
            return
        state.last_sequence = feedback.feedback_sequence
        state.last_status = feedback.status
        state.last_progress = max(state.last_progress, feedback.progress)
        state.last_feedback = feedback
        self._last_feedback_received_at = received.received_at_monotonic_seconds
        self.metrics.execution_feedback_count_by_status[feedback.status.value] += 1
        self._event(
            "execution_stop_feedback",
            feedback.status.value,
            plan_id=feedback.plan_id,
            execution_id=feedback.execution_id,
            stop_attempt_id=lifecycle.attempt_id,
            command_id=command_id,
            feedback_stream_id=feedback.feedback_stream_id,
            producer_epoch=feedback.producer_epoch,
        )
        if feedback.status is ExecutionFeedbackStatus.STOPPING:
            return
        state.terminal_feedback = feedback
        lifecycle.stopped_confirmed = True
        lifecycle.stopped_boundary = feedback.current_boundary
        lifecycle.confirmation_stream_id = feedback.feedback_stream_id
        lifecycle.confirmation_producer_epoch = feedback.producer_epoch
        self._stop_unconfirmed = False
        self._pending_stopped_feedback = feedback
        self._remember_stop_confirmation(feedback)
        if self._stop_wait_started_at is None:
            now = float(self._clock())
            self._stop_wait_started_at = now
            self._stop_wait_last_at = now
        self._try_reconcile_pending_stop()

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
                resume_after_stop=not (
                    self._stop_lifecycle is not None
                    and self._stop_lifecycle.recovery_required
                ),
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
        if self._stop_lifecycle is not None:
            self._stop_lifecycle.world_reconciled = True
            self._stop_lifecycle.stopped_boundary = boundary
            self._stop_unconfirmed = False
        self._clear_active_execution()

    def _consume_feedback(self, received: _ReceivedExecutionFeedback) -> None:
        feedback = received.feedback
        if not isinstance(feedback, ExecutionFeedback):
            self._invalid_feedback(
                f"expected ExecutionFeedback, got {type(feedback).__name__}"
            )
            return
        if feedback.stop_command_id is not None:
            self._consume_stop_feedback(received)
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
        self._last_feedback_received_at = received.received_at_monotonic_seconds
        processed_at = float(self._clock())
        self.metrics.execution_feedback_count_by_status[feedback.status.value] += 1
        self._event(
            "execution_feedback",
            feedback.status.value,
            plan_id=feedback.plan_id,
            execution_id=feedback.execution_id,
            progress=feedback.progress,
            received_at_monotonic_seconds=received.received_at_monotonic_seconds,
            processed_at_monotonic_seconds=processed_at,
            runtime_queue_wait_seconds=max(
                0.0,
                processed_at - received.received_at_monotonic_seconds,
            ),
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
            self._fail_active(feedback.message or "execution failed", feedback=feedback)
            self._record_execution_outcome(feedback)
            return
        if feedback.status is ExecutionFeedbackStatus.DEVIATED:
            self.metrics.execution_deviation_count += 1
            self._fail_active(
                feedback.message or "execution deviated",
                reason=ReplanReason.EXECUTION_DEVIATION,
                feedback=feedback,
            )
            self._record_execution_outcome(feedback)
            return
        self.metrics.execution_failure_count += 1
        self._fail_active(feedback.message or feedback.status.value, feedback=feedback)
        self._record_execution_outcome(feedback)
        if self.session.execution.state is not ExecutionState.STOPPING:
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
            self._pending_feedback.append(
                _ReceivedExecutionFeedback(feedback, float(self._clock()))
            )
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
            invalid_before = self.metrics.invalid_feedback_count
            self._consume_feedback(self._pending_feedback.popleft())
            processed += 1
            if (
                self.state is RuntimeState.RECOVERY
                or self.metrics.invalid_feedback_count != invalid_before
            ):
                return

    def _apply_world_observations(self) -> None:
        processed = 0
        while self._pending_worlds and processed < self._max_world_observations_per_step:
            received = self._pending_worlds.popleft()
            observation = received.message
            assert isinstance(observation, WorldObservation)
            snapshot = observation.snapshot
            try:
                self.session.update_scene(snapshot)
            except Exception as exc:
                self._fail_active(f"world observation rejected: {exc}")
                return
            self._latest_world = snapshot
            self._latest_world_observation = observation
            self._scene_received_at = received.received_at_monotonic_seconds
            self._watchdog_latches.discard("scene_stale")
            if self.session.execution.state is ExecutionState.STOPPING:
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
        now = float(self._clock())
        if self._expired(
            now,
            self._scene_received_at,
            self.watchdog_policy.scene_freshness_timeout_seconds,
        ):
            if "scene_stale" not in self._watchdog_latches:
                self._watchdog_latches.add("scene_stale")
                self.metrics.scene_stale_count += 1
                message = "authoritative scene observation is stale"
                self._apply_terminal_action(
                    self.watchdog_policy.scene_stale_action,
                    ReplanReason.SCENE_STALE,
                    message,
                )
                self._event("watchdog_scene_stale", ReplanReason.SCENE_STALE.value)
            return
        if self.execution_backend.health is not ExecutionBackendHealth.READY:
            self.metrics.backend_unhealthy_count += 1
            self._fail_active(
                f"execution backend is not ready: {self.execution_backend.health.value}",
                reason=ReplanReason.BACKEND_UNHEALTHY,
            )
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
        if result.status is not ExecutionCommandStatus.ACCEPTED:
            self.metrics.execution_start_rejected_count += 1
            self._active_plan = plan
            message = f"execution start rejected: {result.status.value}: {result.message}"
            self.session.enter_recovery(
                message,
                reason=ReplanReason.EXECUTION_FAILED,
                plan_id=plan.plan_id,
            )
            self.state = RuntimeState.RECOVERY
            self._event(
                "runtime_recovery",
                ReplanReason.EXECUTION_FAILED.value,
                plan_id=plan.plan_id,
                message=message,
            )
            self._clear_active_execution()
            return
        if result.plan_id != plan.plan_id or not result.execution_id:
            self.metrics.execution_start_rejected_count += 1
            self._active_plan = plan
            self._active_execution_id = result.execution_id
            self._fail_active("accepted execution start returned an invalid identity")
            return
        self._active_plan = plan
        self._active_execution_id = result.execution_id
        self._active_feedback_domain = None
        self._execution_started_at = now
        self._last_feedback_received_at = now
        self._stop_requested_at = None
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
            self.session.execution.state is not ExecutionState.STOPPING
            or self._active_plan is None
            or self._active_plan.plan_id in self._stop_requested_plan_ids
        ):
            return
        plan_id = self._active_plan.plan_id
        reason_value = self.session.terminal_reason or ReplanReason.SCENE_REVISION_CHANGED.value
        try:
            reason = ReplanReason(reason_value)
        except ValueError:
            reason = ReplanReason.EXECUTION_FAILED
        lifecycle = self._ensure_stop_lifecycle(reason, self.session.snapshot()["terminal_message"])
        self._stop_requested_plan_ids.add(plan_id)
        self.metrics.execution_stop_request_count += 1
        try:
            result = self.execution_backend.request_stop(
                plan_id,
                lifecycle.trigger_reason,
            )
        except Exception as exc:
            self.metrics.execution_stop_rejected_count += 1
            self._mark_stop_unconfirmed(
                f"execution stop request raised: {exc}",
                reason=ReplanReason.EXECUTION_FAILED,
                command_state=StopCommandState.ERROR,
            )
            return
        if not isinstance(result, ExecutionCommandResult) or result.status not in {
            ExecutionCommandStatus.ACCEPTED,
            ExecutionCommandStatus.ALREADY_STOPPING,
        }:
            self.metrics.execution_stop_rejected_count += 1
            status = type(result).__name__ if not isinstance(result, ExecutionCommandResult) else result.status.value
            self._mark_stop_unconfirmed(
                f"execution stop request rejected: {status}",
                reason=ReplanReason.EXECUTION_FAILED,
                command_state=StopCommandState.REJECTED,
            )
            return
        if (
            result.plan_id != plan_id
            or (
                self._active_execution_id is not None
                and result.execution_id != self._active_execution_id
            )
        ):
            self.metrics.execution_stop_rejected_count += 1
            lifecycle.command_id = result.command_id
            self._mark_stop_unconfirmed(
                "execution stop acknowledgement identity mismatch",
                reason=ReplanReason.EXECUTION_DEVIATION,
                command_state=StopCommandState.REJECTED,
            )
            return
        lifecycle.command_state = StopCommandState.ACCEPTED
        lifecycle.command_id = result.command_id
        lifecycle.command_message = result.message
        self._stop_requested_at = float(self._clock())
        self._event(
            "execution_stop_command",
            result.status.value,
            plan_id=plan_id,
            execution_id=self._active_execution_id,
            command_id=result.command_id,
        )

    def _apply_terminal_action(
        self,
        action: WatchdogTerminalAction,
        reason: ReplanReason,
        message: str,
    ) -> None:
        if action is WatchdogTerminalAction.WAITING_FOR_SCENE:
            self.session.wait_for_scene(message, reason=reason)
        elif action is WatchdogTerminalAction.BLOCKED:
            self.session.enter_blocked(message, reason=reason)
        else:
            self.session.enter_recovery(message, reason=reason)

    @staticmethod
    def _expired(now: float, started: float | None, timeout: float | None) -> bool:
        return bool(timeout is not None and started is not None and now - started >= timeout)

    def _sync_planning_watch(self, now: float) -> None:
        task_id = self.session.active_planning_task_id
        if task_id is None:
            self._planning_task_id = None
            self._planning_started_at = None
        elif task_id != self._planning_task_id:
            self._planning_task_id = task_id
            self._planning_started_at = now

    def _run_watchdogs(self, now: float) -> None:
        policy = self.watchdog_policy
        self._sync_planning_watch(now)
        if (
            self.session.state is SessionState.READY
            and self._expired(now, self._scene_received_at, policy.scene_freshness_timeout_seconds)
            and "scene_stale" not in self._watchdog_latches
        ):
            self._watchdog_latches.add("scene_stale")
            self.metrics.scene_stale_count += 1
            message = "authoritative scene observation is stale"
            self._apply_terminal_action(policy.scene_stale_action, ReplanReason.SCENE_STALE, message)
            self._event("watchdog_scene_stale", ReplanReason.SCENE_STALE.value, message=message)
            return
        if (
            self.session.execution.state is ExecutionState.RUNNING
            and self._expired(
                now,
                (
                    self._last_feedback_received_at
                    if self._last_feedback_received_at is not None
                    else self._execution_started_at
                ),
                policy.execution_feedback_timeout_seconds,
            )
            and "execution_feedback" not in self._watchdog_latches
        ):
            self._watchdog_latches.add("execution_feedback")
            self.metrics.execution_feedback_timeout_count += 1
            message = "execution feedback silence exceeded watchdog deadline"
            self.session.begin_safety_stop(ReplanReason.EXECUTION_FEEDBACK_TIMEOUT, message)
            self._event(
                "watchdog_execution_feedback_timeout",
                ReplanReason.EXECUTION_FEEDBACK_TIMEOUT.value,
                plan_id=None if self._active_plan is None else self._active_plan.plan_id,
                execution_id=self._active_execution_id,
            )
        if (
            self.session.state is SessionState.STOPPING
            and self._pending_stopped_feedback is None
            and self._expired(now, self._stop_requested_at, policy.stop_ack_timeout_seconds)
            and "stop_ack" not in self._watchdog_latches
        ):
            self._watchdog_latches.add("stop_ack")
            self.metrics.stop_ack_timeout_count += 1
            message = "safe-stop acknowledgement deadline exceeded"
            self._mark_stop_unconfirmed(
                message,
                reason=ReplanReason.STOP_ACK_TIMEOUT,
            )
            self._event("watchdog_stop_ack_timeout", ReplanReason.STOP_ACK_TIMEOUT.value)
            return
        if (
            self._pending_stopped_feedback is not None
            and self._expired(
                now,
                self._stop_wait_started_at,
                policy.stopped_observation_timeout_seconds,
            )
            and "stop_observation" not in self._watchdog_latches
        ):
            self._watchdog_latches.add("stop_observation")
            self.metrics.stop_observation_timeout_count += 1
            message = "STOPPED feedback was not reconciled with a fresh world observation"
            lifecycle = self._stop_lifecycle
            if lifecycle is not None:
                lifecycle.failure_reason = ReplanReason.STOP_OBSERVATION_TIMEOUT.value
                lifecycle.failure_message = message
                lifecycle.recovery_required = True
            self.session.mark_stop_unconfirmed(
                ReplanReason.STOP_OBSERVATION_TIMEOUT,
                message,
                blocked=(
                    policy.stopped_observation_timeout_action
                    is WatchdogTerminalAction.BLOCKED
                ),
            )
            self._stop_unconfirmed = False
            self._event("watchdog_stop_observation_timeout", ReplanReason.STOP_OBSERVATION_TIMEOUT.value)
            return
        if (
            self.session.active_planning_task_id is not None
            and self._expired(now, self._planning_started_at, policy.planning_timeout_seconds)
            and "planning" not in self._watchdog_latches
        ):
            self._watchdog_latches.add("planning")
            self.metrics.planning_timeout_count += 1
            task_id = self.session.timeout_active_planning()
            self._timed_out_planning_task_id = task_id
            self._event(
                "watchdog_planning_timeout",
                ReplanReason.PLANNING_TIMEOUT.value,
                task_id=task_id,
                worker_still_exiting=bool(self.session.executor.running_count),
            )
            return
        if (
            self._active_plan is not None
            and self.execution_backend.health is not ExecutionBackendHealth.READY
            and "backend_health" not in self._watchdog_latches
        ):
            self._watchdog_latches.add("backend_health")
            self.metrics.backend_unhealthy_count += 1
            self._fail_active(
                f"execution backend became unhealthy: {self.execution_backend.health.value}",
                reason=ReplanReason.BACKEND_UNHEALTHY,
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

    def _finish_step(self) -> RuntimeState:
        if self.session.execution.state is ExecutionState.STOPPING:
            self._request_stop_once()
        self._record_stop_waiting()
        return self._sync_state()

    def step(self) -> RuntimeState:
        self._assert_open()
        request_staging_space = max(
            0,
            self._initial_request_capacity - len(self._pending_initial),
        )
        world_staging_space = max(
            0,
            self._world_observation_capacity - len(self._pending_worlds),
        )
        drained = self.ingress_mailbox.drain(
            min(self._max_initial_requests_per_step, request_staging_space),
            min(self._max_world_observations_per_step, world_staging_space),
        )
        self._pending_initial.extend(drained.requests)
        self._pending_worlds.extend(drained.observations)
        if drained.conflict is not None:
            self._pending_ingress_fault = drained.conflict
        if self._pending_ingress_fault is not None:
            message, self._pending_ingress_fault = self._pending_ingress_fault, None
            self._fail_active(message, reason=ReplanReason.PLAN_INVALIDATED)
            return self._finish_step()
        submitted = 0
        while self._pending_initial and submitted < self._max_initial_requests_per_step:
            received = self._pending_initial.popleft()
            request = received.message
            assert isinstance(request, PlanningRequest)
            if self.session.state is SessionState.RECOVERY:
                self.metrics.accepted_ingress_discarded_count += 1
                self._event(
                    "accepted_initial_request_discarded",
                    "RUNTIME_IN_RECOVERY",
                    request_id=request.request_id,
                )
                submitted += 1
                continue
            try:
                self.session.submit(request)
            except Exception as exc:
                self._fail_active(
                    f"initial planning request rejected: {exc}",
                    reason=ReplanReason.PLANNER_FAILURE,
                )
                return self._finish_step()
            self._scene_received_at = received.received_at_monotonic_seconds
            self._latest_world = request.world_snapshot
            self._watchdog_latches.discard("scene_stale")
            submitted += 1
        self._poll_execution_feedback()
        self._apply_world_observations()
        if self.state is RuntimeState.RECOVERY:
            if self._timed_out_planning_task_id is not None:
                self.session.advance(1)
                if self.session.active_planning_task_id is None:
                    self._timed_out_planning_task_id = None
            return self._finish_step()
        now = float(self._clock())
        self._run_watchdogs(now)
        if self.session.state in {SessionState.RECOVERY, SessionState.BLOCKED}:
            if self._timed_out_planning_task_id is not None:
                self.session.advance(1)
                if self.session.active_planning_task_id is None:
                    self._timed_out_planning_task_id = None
            return self._finish_step()
        self.session.advance(1)
        if (
            self._timed_out_planning_task_id is not None
            and self.session.active_planning_task_id is None
        ):
            self._timed_out_planning_task_id = None
        self._sync_planning_watch(float(self._clock()))
        if self.session.active_planning_task_id is None:
            self._watchdog_latches.discard("planning")
        self._start_ready_plan()
        if self.session.state is not SessionState.RECOVERY:
            self._create_successor()
        self._request_stop_once()
        if self.session.state is not SessionState.RECOVERY:
            try:
                self.execution_backend.advance(1)
            except Exception as exc:
                self._fail_active(f"execution backend advance failed: {exc}")
        return self._finish_step()

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
            mailbox_requests, mailbox_worlds = self.ingress_mailbox.pending_counts()
            if state in {
                RuntimeState.IDLE,
                RuntimeState.WAITING_FOR_SCENE,
                RuntimeState.WAITING_FOR_OBSERVATION,
                RuntimeState.RECOVERY,
                RuntimeState.BLOCKED,
            } and not self._pending_initial and not self._pending_worlds and not mailbox_requests and not mailbox_worlds:
                return state
            executor = self.session.executor
            if executor.running_count or executor.pending_count:
                remaining = deadline - monotonic()
                if remaining <= 0.0 or not executor.wait_for_completion(remaining):
                    raise TimeoutError("runtime did not receive planning completion before timeout")
            if monotonic() > deadline:
                raise TimeoutError("runtime did not stabilize before timeout")
        raise RuntimeError("runtime did not stabilize within max_steps")

    def resume_after_recovery(self) -> RuntimeState:
        """Authorize recovery only after any required stop is proven and reconciled."""

        self._assert_open()
        lifecycle = self._stop_lifecycle
        if lifecycle is not None and lifecycle.recovery_required:
            if not lifecycle.stopped_confirmed or not lifecycle.world_reconciled:
                raise RuntimeError("cannot resume before stopped evidence and world reconciliation")
            if self.execution_backend.health is not ExecutionBackendHealth.READY:
                raise RuntimeError("cannot resume while the execution backend is unhealthy")
            lifecycle.recovery_resume_authorized = True
        self.session.resume_after_recovery()
        self._event(
            "runtime_recovery_resumed",
            "EXPLICIT_RECOVERY",
            plan_id=None if lifecycle is None else lifecycle.plan_id,
            execution_id=None if lifecycle is None else lifecycle.execution_id,
            stop_attempt_id=None if lifecycle is None else lifecycle.attempt_id,
        )
        return self._sync_state()

    def _stop_lifecycle_snapshot(self) -> Mapping[str, Any] | None:
        lifecycle = self._stop_lifecycle
        if lifecycle is None:
            return None
        resume_allowed = bool(
            lifecycle.recovery_required
            and lifecycle.stopped_confirmed
            and lifecycle.world_reconciled
            and self.execution_backend.health is ExecutionBackendHealth.READY
        )
        boundary = lifecycle.stopped_boundary
        return MappingProxyType({
            "attempt_id": lifecycle.attempt_id,
            "execution_id": lifecycle.execution_id,
            "plan_id": lifecycle.plan_id,
            "trigger_reason": lifecycle.trigger_reason,
            "trigger_message": lifecycle.trigger_message,
            "command_state": lifecycle.command_state.value,
            "command_id": lifecycle.command_id,
            "command_message": lifecycle.command_message,
            "failure_reason": lifecycle.failure_reason,
            "failure_message": lifecycle.failure_message,
            "execution_outcome": (
                None if lifecycle.execution_outcome is None else lifecycle.execution_outcome.value
            ),
            "execution_outcome_message": lifecycle.execution_outcome_message,
            "stopped_confirmed": lifecycle.stopped_confirmed,
            "stopped_boundary": None if boundary is None else {
                "q": boundary.q,
                "qd": boundary.qd,
                "qdd": boundary.qdd,
                "time_seconds": boundary.time_seconds,
                "boundary_mode": boundary.boundary_mode.value,
            },
            "confirmation_stream_id": lifecycle.confirmation_stream_id,
            "confirmation_producer_epoch": lifecycle.confirmation_producer_epoch,
            "waiting_for_compatible_observation": bool(
                lifecycle.stopped_confirmed and not lifecycle.world_reconciled
            ),
            "world_reconciled": lifecycle.world_reconciled,
            "recovery_required": lifecycle.recovery_required,
            "resume_allowed": resume_allowed,
            "recovery_resume_authorized": lifecycle.recovery_resume_authorized,
        })

    def snapshot(self) -> dict[str, Any]:
        mailbox_requests, mailbox_worlds = self.ingress_mailbox.pending_counts()
        return {
            "state": self.state.value,
            "closed": self._closed,
            "step_order": self.STEP_ORDER,
            "active_plan_id": None if self._active_plan is None else self._active_plan.plan_id,
            "active_execution_id": self._active_execution_id,
            "pending_initial_requests": len(self._pending_initial) + mailbox_requests,
            "pending_world_observations": len(self._pending_worlds) + mailbox_worlds,
            "pending_execution_feedback": len(self._pending_feedback),
            "latest_world_fingerprint": (
                None if self._latest_world is None else self._latest_world.fingerprint
            ),
            "ingress_capacities": {
                "semantics": "bounded per layer; total is mailbox plus runtime staging",
                "initial_requests": self._initial_request_capacity,
                "world_observations": self._world_observation_capacity,
                "mailbox_initial_requests": self._initial_request_capacity,
                "staging_initial_requests": self._initial_request_capacity,
                "maximum_total_initial_requests": 2 * self._initial_request_capacity,
                "mailbox_world_observations": self._world_observation_capacity,
                "staging_world_observations": self._world_observation_capacity,
                "maximum_total_world_observations": 2 * self._world_observation_capacity,
                "execution_feedback": self._execution_feedback_capacity,
            },
            "ingress_pending_by_layer": {
                "mailbox_initial_requests": mailbox_requests,
                "staging_initial_requests": len(self._pending_initial),
                "mailbox_world_observations": mailbox_worlds,
                "staging_world_observations": len(self._pending_worlds),
            },
            "terminal_feedback_history": len(self._terminal_feedback),
            "stop_unconfirmed": bool(
                self._stop_lifecycle is not None
                and not self._stop_lifecycle.stopped_confirmed
            ),
            "stop_lifecycle": self._stop_lifecycle_snapshot(),
            "event_journal": self.events.summary(),
            "watchdog": {
                "scene_received_at": self._scene_received_at,
                "last_feedback_received_at": self._last_feedback_received_at,
                "stop_requested_at": self._stop_requested_at,
                "planning_started_at": self._planning_started_at,
                "timed_out_planning_task_id": self._timed_out_planning_task_id,
            },
            "metrics": self.metrics.report(),
            "session": self.session.snapshot(),
        }

    def events_since(
        self,
        sequence: int,
        limit: int | None = None,
    ) -> EventJournalRead[RuntimeEvent]:
        result = self.events.events_since(sequence, limit)
        if result.history_gap:
            self.metrics.event_history_gap_count += 1
        return result

    def shutdown(self) -> None:
        if self._closed:
            return
        if self._owner_thread_id is None:
            self._owner_thread_id = get_ident()
        elif get_ident() != self._owner_thread_id:
            raise RuntimeError("runtime shutdown must run on the step owner thread")
        mailbox_requests, mailbox_worlds = self.ingress_mailbox.shutdown()
        discarded_requests = mailbox_requests + len(self._pending_initial)
        discarded_worlds = mailbox_worlds + len(self._pending_worlds)
        if discarded_requests or discarded_worlds:
            discarded = discarded_requests + discarded_worlds
            self.metrics.accepted_ingress_discarded_count += discarded
            self._event(
                "accepted_ingress_discarded",
                "RUNTIME_SHUTDOWN",
                planning_requests=discarded_requests,
                world_observations=discarded_worlds,
            )
            self._pending_initial.clear()
            self._pending_worlds.clear()
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
    "RuntimeIngressMailbox",
    "RuntimeEvent",
    "RuntimeIngressResult",
    "RuntimeIngressStatus",
    "RuntimeMetrics",
    "RuntimeState",
    "RuntimeWatchdogPolicy",
    "StopCommandState",
    "SuccessorRequestFactory",
    "WatchdogTerminalAction",
    "WorldObservation",
]
