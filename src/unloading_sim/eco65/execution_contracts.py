"""Exact selected definitions from src/unloading_sim/online_execution.py @ f7f7e0934ad425ab62cd2df8201812df1ba8e505.
Unused orchestration deliberately omitted; local contract types only.
"""
from __future__ import annotations
from .planning_contracts import BoundaryMode, MotionBoundaryState, PlanArtifactKind, PlanEnvelope

from abc import ABC, abstractmethod

from collections import deque

from dataclasses import dataclass, field

from enum import Enum

from math import isfinite

from time import monotonic

from types import MappingProxyType

from typing import Any, Callable, Mapping

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
    stop_command_id: str | None = None

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
        if self.stop_command_id is not None:
            if not self.stop_command_id:
                raise ValueError("stop command id must be non-empty when provided")
            if status not in {
                ExecutionFeedbackStatus.STOPPING,
                ExecutionFeedbackStatus.STOPPED,
            }:
                raise ValueError("stop command id is only valid on STOPPING or STOPPED feedback")
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
