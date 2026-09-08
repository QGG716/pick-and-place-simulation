"""Provider-neutral execution-side contracts for the online control plane.

The deterministic backend in this module is a state-machine fixture.  It does
not model controller interpolation, robot dynamics, tracking error, safety PLC
behavior, or physical execution time.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from time import monotonic
from types import MappingProxyType
from typing import Any, Callable, Mapping

from .online_planning import (
    BoundaryMode,
    MotionBoundaryState,
    PlanArtifactKind,
    PlanEnvelope,
)


def _freeze_metadata(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze_metadata(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_metadata(item) for item in value)
    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(f"execution metadata contains unsupported {type(value).__name__}")


class ExecutionBackendHealth(str, Enum):
    READY = "READY"
    UNAVAILABLE = "UNAVAILABLE"
    SHUTDOWN = "SHUTDOWN"
    FAULTED = "FAULTED"


class ExecutionBackendState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    FAULTED = "FAULTED"
    SHUTDOWN = "SHUTDOWN"


class ExecutionFeedbackStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    REJECTED = "REJECTED"
    DEVIATED = "DEVIATED"
    FAULTED = "FAULTED"

    @property
    def terminal(self) -> bool:
        return self in {
            ExecutionFeedbackStatus.STOPPED,
            ExecutionFeedbackStatus.SUCCEEDED,
            ExecutionFeedbackStatus.FAILED,
            ExecutionFeedbackStatus.REJECTED,
            ExecutionFeedbackStatus.DEVIATED,
            ExecutionFeedbackStatus.FAULTED,
        }


class ExecutionCommandStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    ALREADY_RUNNING = "ALREADY_RUNNING"
    ALREADY_STOPPING = "ALREADY_STOPPING"
    NOT_RUNNING = "NOT_RUNNING"
    BACKEND_UNAVAILABLE = "BACKEND_UNAVAILABLE"
    UNSUPPORTED = "UNSUPPORTED"
    ERROR = "ERROR"


@dataclass(frozen=True)
class ExecutionBackendIdentity:
    backend_name: str
    backend_version: str
    adapter_version: str

    def __post_init__(self) -> None:
        for name in ("backend_name", "backend_version", "adapter_version"):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")


@dataclass(frozen=True)
class ExecutionBackendCapabilities:
    supported_artifact_kinds: frozenset[PlanArtifactKind]
    supported_boundary_modes: frozenset[BoundaryMode]
    supports_stop_request: bool
    supports_command_acknowledgement: bool
    reports_position: bool
    reports_velocity: bool
    reports_acceleration: bool
    reports_progress: bool
    supports_deterministic_step: bool
    supports_continuous_handoff: bool

    def __post_init__(self) -> None:
        artifacts = frozenset(PlanArtifactKind(value) for value in self.supported_artifact_kinds)
        boundaries = frozenset(BoundaryMode(value) for value in self.supported_boundary_modes)
        if not artifacts or not boundaries:
            raise ValueError("execution capabilities require artifact and boundary support")
        object.__setattr__(self, "supported_artifact_kinds", artifacts)
        object.__setattr__(self, "supported_boundary_modes", boundaries)

    def supports(self, plan: PlanEnvelope) -> bool:
        continuous = bool(
            plan.expected_start_boundary.boundary_mode is BoundaryMode.CONTINUOUS_BOUNDARY
            or plan.expected_end_boundary.boundary_mode is BoundaryMode.CONTINUOUS_BOUNDARY
        )
        return bool(
            plan.artifact_kind in self.supported_artifact_kinds
            and plan.expected_start_boundary.boundary_mode in self.supported_boundary_modes
            and plan.expected_end_boundary.boundary_mode in self.supported_boundary_modes
            and (not continuous or self.supports_continuous_handoff)
            and (not continuous or self.reports_position)
            and (not continuous or self.reports_velocity)
            and (not continuous or self.reports_acceleration)
        )


@dataclass(frozen=True)
class ExecutionCommandResult:
    status: ExecutionCommandStatus
    command_id: str
    execution_id: str | None
    plan_id: str | None
    message: str = ""
    native_code: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", ExecutionCommandStatus(self.status))
        if not self.command_id:
            raise ValueError("execution command_id must be non-empty")
        if self.status in {
            ExecutionCommandStatus.ACCEPTED,
            ExecutionCommandStatus.ALREADY_RUNNING,
            ExecutionCommandStatus.ALREADY_STOPPING,
        } and (not self.execution_id or not self.plan_id):
            raise ValueError("accepted/running execution command results require execution and plan ids")
        object.__setattr__(self, "metadata", _freeze_metadata(self.metadata))

    @property
    def accepted(self) -> bool:
        return self.status is ExecutionCommandStatus.ACCEPTED


@dataclass(frozen=True)
class ExecutionFeedback:
    feedback_sequence: int
    execution_id: str
    plan_id: str
    status: ExecutionFeedbackStatus
    progress: float
    current_boundary: MotionBoundaryState
    message: str = ""
    native_code: str | None = None
    observed_at_monotonic_seconds: float = field(default_factory=monotonic)
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)
    feedback_stream_id: str = "default"
    producer_epoch: int = 0

    def __post_init__(self) -> None:
        status = ExecutionFeedbackStatus(self.status)
        progress = float(self.progress)
        observed = float(self.observed_at_monotonic_seconds)
        if self.feedback_sequence < 0:
            raise ValueError("feedback sequence must be non-negative")
        if not self.execution_id or not self.plan_id:
            raise ValueError("execution feedback requires execution_id and plan_id")
        if not self.feedback_stream_id:
            raise ValueError("execution feedback stream id must be non-empty")
        if self.producer_epoch < 0:
            raise ValueError("execution feedback producer epoch must be non-negative")
        if not isfinite(progress) or not 0.0 <= progress <= 1.0:
            raise ValueError("execution progress must be finite and inside [0, 1]")
        if not isinstance(self.current_boundary, MotionBoundaryState):
            raise TypeError("execution feedback requires a complete MotionBoundaryState")
        if not isfinite(observed) or observed < 0.0:
            raise ValueError("observed monotonic time must be finite and non-negative")
        if status is ExecutionFeedbackStatus.STOPPED and (
            self.current_boundary.boundary_mode is not BoundaryMode.STOP_BOUNDARY
        ):
            raise ValueError("STOPPED feedback requires a STOP_BOUNDARY")
        if status is ExecutionFeedbackStatus.SUCCEEDED and progress != 1.0:
            raise ValueError("SUCCEEDED feedback requires progress 1.0")
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "progress", progress)
        object.__setattr__(self, "observed_at_monotonic_seconds", observed)
        object.__setattr__(self, "metadata", _freeze_metadata(self.metadata))


class ExecutionBackend(ABC):
    """Execution-side adapter contract; it never owns planning-session state."""

    @property
    @abstractmethod
    def identity(self) -> ExecutionBackendIdentity: ...

    @property
    @abstractmethod
    def capabilities(self) -> ExecutionBackendCapabilities: ...

    @property
    @abstractmethod
    def health(self) -> ExecutionBackendHealth: ...

    @property
    @abstractmethod
    def state(self) -> ExecutionBackendState: ...

    def initialize(self) -> None: ...

    @abstractmethod
    def start(self, plan_envelope: PlanEnvelope) -> ExecutionCommandResult: ...

    @abstractmethod
    def poll(self) -> ExecutionFeedback | None: ...

    @abstractmethod
    def request_stop(self, plan_id: str, reason: str) -> ExecutionCommandResult: ...

    @abstractmethod
    def current_boundary(self) -> MotionBoundaryState | None: ...

    def advance(self, max_steps: int = 1) -> int:
        del max_steps
        return 0

    @abstractmethod
    def shutdown(self) -> None: ...

    def close(self) -> None:
        self.shutdown()

    def __enter__(self) -> ExecutionBackend:
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()


@dataclass
class _SimExecution:
    execution_id: str
    plan: PlanEnvelope
    current_boundary: MotionBoundaryState
    running_emitted: bool = False
    completed_steps: int = 0
    stop_requested: bool = False
    stopping_emitted: bool = False


class DeterministicSimExecutionBackend(ExecutionBackend):
    """Deterministic execution state-machine fixture with no background thread."""

    def __init__(
        self,
        *,
        execution_steps: int = 2,
        terminal_status: ExecutionFeedbackStatus = ExecutionFeedbackStatus.SUCCEEDED,
        reject_start: bool = False,
        clock: Callable[[], float] = monotonic,
        feedback_stream_id: str = "deterministic-sim-feedback",
        producer_epoch: int = 0,
    ) -> None:
        if execution_steps < 1:
            raise ValueError("execution_steps must be positive")
        terminal = ExecutionFeedbackStatus(terminal_status)
        if terminal not in {
            ExecutionFeedbackStatus.SUCCEEDED,
            ExecutionFeedbackStatus.FAILED,
            ExecutionFeedbackStatus.DEVIATED,
        }:
            raise ValueError("sim terminal status must be SUCCEEDED, FAILED, or DEVIATED")
        if not feedback_stream_id:
            raise ValueError("feedback_stream_id must be non-empty")
        if producer_epoch < 0:
            raise ValueError("producer_epoch must be non-negative")
        self._identity = ExecutionBackendIdentity("deterministic-sim-execution", "1", "1")
        self._capabilities = ExecutionBackendCapabilities(
            frozenset({PlanArtifactKind.GEOMETRIC_PATH, PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY}),
            frozenset({BoundaryMode.STOP_BOUNDARY}),
            supports_stop_request=True,
            supports_command_acknowledgement=True,
            reports_position=True,
            reports_velocity=True,
            reports_acceleration=True,
            reports_progress=True,
            supports_deterministic_step=True,
            supports_continuous_handoff=False,
        )
        self.execution_steps = int(execution_steps)
        self.terminal_status = terminal
        self.reject_start = bool(reject_start)
        self._clock = clock
        self.feedback_stream_id = feedback_stream_id
        self.producer_epoch = int(producer_epoch)
        self._health = ExecutionBackendHealth.READY
        self._active: _SimExecution | None = None
        self._last_boundary: MotionBoundaryState | None = None
        self._feedback: deque[ExecutionFeedback] = deque()
        self._execution_sequence = 0
        self._command_sequence = 0
        self._feedback_sequence = -1
        self._started_plan_ids: set[str] = set()

    @property
    def identity(self) -> ExecutionBackendIdentity:
        return self._identity

    @property
    def capabilities(self) -> ExecutionBackendCapabilities:
        return self._capabilities

    @property
    def health(self) -> ExecutionBackendHealth:
        return self._health

    @property
    def state(self) -> ExecutionBackendState:
        if self._health is ExecutionBackendHealth.SHUTDOWN:
            return ExecutionBackendState.SHUTDOWN
        if self._health is ExecutionBackendHealth.FAULTED:
            return ExecutionBackendState.FAULTED
        if self._active is None:
            return ExecutionBackendState.IDLE
        if self._active.stop_requested:
            return ExecutionBackendState.STOPPING
        return ExecutionBackendState.RUNNING

    def _command(
        self,
        status: ExecutionCommandStatus,
        *,
        execution_id: str | None,
        plan_id: str | None,
        message: str,
        native_code: str | None = None,
    ) -> ExecutionCommandResult:
        self._command_sequence += 1
        return ExecutionCommandResult(
            status,
            f"sim-command-{self._command_sequence}",
            execution_id,
            plan_id,
            message,
            native_code,
        )

    def _emit(
        self,
        status: ExecutionFeedbackStatus,
        progress: float,
        boundary: MotionBoundaryState,
        message: str,
    ) -> None:
        active = self._active
        if active is None:
            raise RuntimeError("cannot emit execution feedback without an active execution")
        self._feedback_sequence += 1
        self._feedback.append(
            ExecutionFeedback(
                self._feedback_sequence,
                active.execution_id,
                active.plan.plan_id,
                status,
                progress,
                boundary,
                message,
                observed_at_monotonic_seconds=self._clock(),
                feedback_stream_id=self.feedback_stream_id,
                producer_epoch=self.producer_epoch,
            )
        )

    def start(self, plan_envelope: PlanEnvelope) -> ExecutionCommandResult:
        if self._health is not ExecutionBackendHealth.READY:
            return self._command(
                ExecutionCommandStatus.BACKEND_UNAVAILABLE,
                execution_id=None,
                plan_id=plan_envelope.plan_id,
                message="execution backend is not ready",
                native_code=self._health.value,
            )
        if self._active is not None:
            return self._command(
                ExecutionCommandStatus.ALREADY_RUNNING,
                execution_id=self._active.execution_id,
                plan_id=self._active.plan.plan_id,
                message="another execution is active",
            )
        if plan_envelope.plan_id in self._started_plan_ids or self.reject_start:
            return self._command(
                ExecutionCommandStatus.REJECTED,
                execution_id=None,
                plan_id=plan_envelope.plan_id,
                message="start rejected by deterministic fixture",
                native_code="SIM_START_REJECTED",
            )
        if not plan_envelope.executable:
            return self._command(
                ExecutionCommandStatus.REJECTED,
                execution_id=None,
                plan_id=plan_envelope.plan_id,
                message="plan envelope is not executable",
                native_code="PLAN_NOT_EXECUTABLE",
            )
        if not self.capabilities.supports(plan_envelope):
            return self._command(
                ExecutionCommandStatus.UNSUPPORTED,
                execution_id=None,
                plan_id=plan_envelope.plan_id,
                message="plan artifact or boundary is unsupported",
                native_code="UNSUPPORTED_PLAN_CONTRACT",
            )
        self._execution_sequence += 1
        execution_id = f"sim-execution-{self._execution_sequence}"
        self._feedback_sequence = -1
        self._started_plan_ids.add(plan_envelope.plan_id)
        self._active = _SimExecution(
            execution_id,
            plan_envelope,
            plan_envelope.expected_start_boundary,
        )
        self._last_boundary = plan_envelope.expected_start_boundary
        result = self._command(
            ExecutionCommandStatus.ACCEPTED,
            execution_id=execution_id,
            plan_id=plan_envelope.plan_id,
            message="start accepted",
        )
        self._emit(
            ExecutionFeedbackStatus.ACCEPTED,
            0.0,
            plan_envelope.expected_start_boundary,
            "start accepted",
        )
        return result

    def poll(self) -> ExecutionFeedback | None:
        if not self._feedback:
            return None
        return self._feedback.popleft()

    def _interpolate_boundary(self, fraction: float) -> MotionBoundaryState:
        active = self._active
        assert active is not None
        start = active.plan.expected_start_boundary
        end = active.plan.expected_end_boundary
        q = tuple(a + fraction * (b - a) for a, b in zip(start.q, end.q, strict=True))
        logical_time = start.time_seconds + fraction * (end.time_seconds - start.time_seconds)
        return MotionBoundaryState.stopped(
            q,
            time_seconds=logical_time,
            predecessor_plan_id=start.predecessor_plan_id,
        )

    def advance(self, max_steps: int = 1) -> int:
        if max_steps < 0:
            raise ValueError("max_steps must be non-negative")
        advanced = 0
        while self._active is not None and advanced < max_steps:
            active = self._active
            if active.stop_requested:
                if not active.stopping_emitted:
                    active.stopping_emitted = True
                    self._emit(
                        ExecutionFeedbackStatus.STOPPING,
                        active.completed_steps / self.execution_steps,
                        active.current_boundary,
                        "stop in progress",
                    )
                else:
                    stopped = MotionBoundaryState.stopped(
                        active.current_boundary.q,
                        time_seconds=active.current_boundary.time_seconds,
                        predecessor_plan_id=active.current_boundary.predecessor_plan_id,
                    )
                    self._emit(
                        ExecutionFeedbackStatus.STOPPED,
                        active.completed_steps / self.execution_steps,
                        stopped,
                        "stopped",
                    )
                    self._last_boundary = stopped
                    self._active = None
                advanced += 1
                continue
            if not active.running_emitted:
                active.running_emitted = True
                self._emit(
                    ExecutionFeedbackStatus.RUNNING,
                    0.0,
                    active.current_boundary,
                    "execution running",
                )
                advanced += 1
                continue
            active.completed_steps += 1
            fraction = active.completed_steps / self.execution_steps
            boundary = self._interpolate_boundary(fraction)
            active.current_boundary = boundary
            self._last_boundary = boundary
            if active.completed_steps < self.execution_steps:
                self._emit(ExecutionFeedbackStatus.RUNNING, fraction, boundary, "execution running")
            else:
                terminal_boundary = (
                    active.plan.expected_end_boundary
                    if self.terminal_status is ExecutionFeedbackStatus.SUCCEEDED
                    else boundary
                )
                self._emit(self.terminal_status, 1.0, terminal_boundary, self.terminal_status.value.lower())
                self._last_boundary = terminal_boundary
                self._active = None
            advanced += 1
        return advanced

    def request_stop(self, plan_id: str, reason: str) -> ExecutionCommandResult:
        del reason
        if self._health is not ExecutionBackendHealth.READY:
            return self._command(
                ExecutionCommandStatus.BACKEND_UNAVAILABLE,
                execution_id=None if self._active is None else self._active.execution_id,
                plan_id=plan_id,
                message="execution backend is not ready",
                native_code=self._health.value,
            )
        if not self.capabilities.supports_stop_request:
            return self._command(
                ExecutionCommandStatus.UNSUPPORTED,
                execution_id=None if self._active is None else self._active.execution_id,
                plan_id=plan_id,
                message="stop requests are unsupported",
            )
        if self._active is None or self._active.plan.plan_id != plan_id:
            return self._command(
                ExecutionCommandStatus.NOT_RUNNING,
                execution_id=None,
                plan_id=plan_id,
                message="plan is not running",
            )
        if self._active.stop_requested:
            return self._command(
                ExecutionCommandStatus.ALREADY_STOPPING,
                execution_id=self._active.execution_id,
                plan_id=plan_id,
                message="stop was already requested",
            )
        self._active.stop_requested = True
        return self._command(
            ExecutionCommandStatus.ACCEPTED,
            execution_id=self._active.execution_id,
            plan_id=plan_id,
            message="stop request accepted",
        )

    def current_boundary(self) -> MotionBoundaryState | None:
        return self._last_boundary

    def shutdown(self) -> None:
        self._health = ExecutionBackendHealth.SHUTDOWN


__all__ = [
    "DeterministicSimExecutionBackend",
    "ExecutionBackend",
    "ExecutionBackendCapabilities",
    "ExecutionBackendHealth",
    "ExecutionBackendIdentity",
    "ExecutionBackendState",
    "ExecutionCommandResult",
    "ExecutionCommandStatus",
    "ExecutionFeedback",
    "ExecutionFeedbackStatus",
]
