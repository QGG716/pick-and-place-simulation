"""Fail-closed control plane for online continuous planning.

This module deliberately contains no grasp, IK, depalletizing, conveyor, or
base-pose algorithms.  A :class:`PlannerBackend` owns those decisions and may
fail normally.  The control plane owns revision binding, FAST/WARM/COLD
escalation, rolling-horizon bookkeeping, speculative reuse, and execution
invalidation.

The asynchronous executor is intentionally cooperative: work is queued in a
stable order and is run only when ``advance`` is called.  It therefore exposes
the same transition trace as the synchronous executor without scheduler races,
while still allowing plan k+1 to be computed while plan k is marked running.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import Counter, deque
from dataclasses import dataclass, field, replace
from enum import Enum
import hashlib
import json
from math import ceil, isfinite
from time import perf_counter
from types import MappingProxyType
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np


class PlanStatus(str, Enum):
    SUCCESS = "SUCCESS"
    NO_IK = "NO_IK"
    GRASP_CONSTRAINT_FAILED = "GRASP_CONSTRAINT_FAILED"
    INITIAL_CLEARANCE_FAILED = "INITIAL_CLEARANCE_FAILED"
    COLLISION = "COLLISION"
    NOT_EVALUATED = "NOT_EVALUATED"
    TIMEOUT = "TIMEOUT"


class PlanningPath(str, Enum):
    FAST = "FAST"
    WARM = "WARM"
    COLD = "COLD"


class BoundaryMode(str, Enum):
    STOP_BOUNDARY = "STOP_BOUNDARY"
    CONTINUOUS_BOUNDARY = "CONTINUOUS_BOUNDARY"


class PlanArtifactKind(str, Enum):
    GEOMETRIC_PATH = "GEOMETRIC_PATH"
    TIME_PARAMETERIZED_TRAJECTORY = "TIME_PARAMETERIZED_TRAJECTORY"


# Compatibility alias for the v0.5.1 public name.
ArtifactKind = PlanArtifactKind


class BackendHealth(str, Enum):
    NEW = "NEW"
    INITIALIZING = "INITIALIZING"
    WARMING = "WARMING"
    READY = "READY"
    DEGRADED = "DEGRADED"
    UNAVAILABLE = "UNAVAILABLE"
    SHUTDOWN = "SHUTDOWN"


class OperationalOutcome(str, Enum):
    PATH_UNSUPPORTED = "PATH_UNSUPPORTED"
    BACKEND_UNAVAILABLE = "BACKEND_UNAVAILABLE"
    DEADLINE_EXCEEDED = "DEADLINE_EXCEEDED"
    CANCELLED = "CANCELLED"
    BACKEND_ERROR = "BACKEND_ERROR"
    CACHE_MISS = "CACHE_MISS"


class OutcomeCategory(str, Enum):
    DOMAIN = "DOMAIN"
    OPERATIONAL = "OPERATIONAL"


class FailureScope(str, Enum):
    CANDIDATE_LOCAL = "CANDIDATE_LOCAL"
    TARGET_LOCAL = "TARGET_LOCAL"
    SCENE_LOCAL = "SCENE_LOCAL"
    BACKEND_LOCAL = "BACKEND_LOCAL"


class ValidationStatus(str, Enum):
    VALID = "VALID"
    INVALID = "INVALID"
    ERROR = "ERROR"


class ReplanReason(str, Enum):
    INITIAL_REQUEST = "INITIAL_REQUEST"
    ROLLING_HORIZON = "ROLLING_HORIZON"
    SPECULATIVE_MISMATCH = "SPECULATIVE_MISMATCH"
    SCENE_REVISION_CHANGED = "SCENE_REVISION_CHANGED"
    PLAN_VALIDATION_FAILED = "PLAN_VALIDATION_FAILED"
    PLAN_INVALIDATED = "PLAN_INVALIDATED"
    EXECUTION_DEVIATION = "EXECUTION_DEVIATION"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    PLANNER_FAILURE = "PLANNER_FAILURE"
    HORIZON_EXHAUSTED = "HORIZON_EXHAUSTED"


class FailureAction(str, Enum):
    ESCALATE_PATH = "ESCALATE_PATH"
    TRY_NEXT_CANDIDATE = "TRY_NEXT_CANDIDATE"
    TRY_NEXT_TARGET = "TRY_NEXT_TARGET"
    WAIT_FOR_SCENE = "WAIT_FOR_SCENE"
    ENTER_RECOVERY = "ENTER_RECOVERY"
    BLOCK = "BLOCK"


class SessionState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    READY = "READY"
    EXECUTING = "EXECUTING"
    STOPPING = "STOPPING"
    WAITING_FOR_SCENE = "WAITING_FOR_SCENE"
    RECOVERY = "RECOVERY"
    BLOCKED = "BLOCKED"


class ExecutionState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


def _canonical(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _canonical(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _canonical(item) for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    if hasattr(value, "center") and hasattr(value, "half_extents") and hasattr(value, "rotation"):
        return {
            "name": str(getattr(value, "name", "")),
            "category": str(getattr(value, "category", "")),
            "center": _canonical(value.center),
            "half_extents": _canonical(value.half_extents),
            "rotation": _canonical(value.rotation),
        }
    if hasattr(value, "obstacles") and hasattr(value, "cartons"):
        return {
            "obstacles": sorted((_canonical(item) for item in value.obstacles), key=lambda item: item["name"]),
            "cartons": sorted((_canonical(item) for item in value.cartons), key=lambda item: item["name"]),
            "metadata": _canonical(getattr(value, "metadata", {})),
        }
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(f"cannot create a deterministic scene fingerprint for {type(value).__name__}")


def scene_fingerprint(scene: Any) -> str:
    payload = json.dumps(_canonical(scene), sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _freeze(value: Any) -> Any:
    """Recursively detach mutable caller-owned state."""

    if isinstance(value, np.ndarray):
        return tuple(_freeze(item) for item in value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return _freeze(_canonical(value))


@dataclass(frozen=True, order=True)
class SceneRevision:
    """Monotonic scene version plus a content identity.

    Sequence numbers order observations; fingerprints determine whether a
    speculative observation is geometrically the one that was predicted.
    """

    sequence: int
    fingerprint: str
    source: str = field(default="perception", compare=False)
    parent_fingerprint: str | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("scene revision sequence must be non-negative")
        if not self.fingerprint:
            raise ValueError("scene revision fingerprint must be non-empty")
        if not self.source:
            raise ValueError("scene revision source must be non-empty")

    @classmethod
    def from_scene(
        cls,
        scene: Any,
        sequence: int,
        *,
        source: str = "perception",
        parent: SceneRevision | None = None,
    ) -> SceneRevision:
        return cls(
            sequence=sequence,
            fingerprint=scene_fingerprint(scene),
            source=source,
            parent_fingerprint=None if parent is None else parent.fingerprint,
        )

    def same_scene(self, other: SceneRevision) -> bool:
        return self.fingerprint == other.fingerprint


@dataclass(frozen=True)
class RobotStateRevision:
    sequence: int
    current_q: tuple[float, ...]
    robot_state: Mapping[str, Any] = field(default_factory=dict)
    fingerprint: str = field(init=False)

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("robot state revision sequence must be non-negative")
        q = tuple(float(value) for value in self.current_q)
        if not q or not all(isfinite(value) for value in q):
            raise ValueError("robot current_q must be finite and non-empty")
        state = _freeze(self.robot_state)
        identity = scene_fingerprint({"current_q": q, "robot_state": state})
        object.__setattr__(self, "current_q", q)
        object.__setattr__(self, "robot_state", state)
        object.__setattr__(self, "fingerprint", identity)


@dataclass(frozen=True)
class PlanningWorldSnapshot:
    """Immutable complete input used for planning and execution validation."""

    scene_revision: SceneRevision
    scene_snapshot: Any
    robot_state_revision: RobotStateRevision
    tool_attachment: Any
    payload_attachment: Any
    base_state: Any
    conveyor_state: Any
    config_identity: Any
    fingerprint: str = field(init=False)

    def __post_init__(self) -> None:
        scene = _freeze(self.scene_snapshot)
        if scene_fingerprint(scene) != self.scene_revision.fingerprint:
            raise ValueError("actual scene snapshot does not match scene revision fingerprint")
        frozen = {
            "tool_attachment": _freeze(self.tool_attachment),
            "payload_attachment": _freeze(self.payload_attachment),
            "base_state": _freeze(self.base_state),
            "conveyor_state": _freeze(self.conveyor_state),
            "config_identity": _freeze(self.config_identity),
        }
        object.__setattr__(self, "scene_snapshot", scene)
        for name, value in frozen.items():
            object.__setattr__(self, name, value)
        identity = {
            "scene_revision": self.scene_revision.fingerprint,
            "scene_snapshot": scene,
            "robot_state_revision": {
                "sequence": self.robot_state_revision.sequence,
                "fingerprint": self.robot_state_revision.fingerprint,
            },
            **frozen,
        }
        object.__setattr__(self, "fingerprint", scene_fingerprint(identity))

    @property
    def current_q(self) -> tuple[float, ...]:
        return self.robot_state_revision.current_q

    @property
    def robot_model_fingerprint(self) -> str:
        if isinstance(self.config_identity, Mapping):
            value = self.config_identity.get("robot_model_fingerprint")
            if value:
                return str(value)
        return scene_fingerprint({"robot_state": self.robot_state_revision.robot_state, "config": self.config_identity})

    @property
    def world_model_fingerprint(self) -> str:
        if isinstance(self.config_identity, Mapping):
            value = self.config_identity.get("world_model_fingerprint")
            if value:
                return str(value)
        return scene_fingerprint(
            {
                "scene_schema": tuple(sorted(self.scene_snapshot.keys())) if isinstance(self.scene_snapshot, Mapping) else type(self.scene_snapshot).__name__,
                "base_state": self.base_state,
                "conveyor_state": self.conveyor_state,
                "config": self.config_identity,
            }
        )

    def planning_context_matches(self, other: PlanningWorldSnapshot) -> bool:
        return bool(
            self.scene_revision.same_scene(other.scene_revision)
            and self.scene_snapshot == other.scene_snapshot
            and self.robot_state_revision.robot_state == other.robot_state_revision.robot_state
            and self.tool_attachment == other.tool_attachment
            and self.payload_attachment == other.payload_attachment
            and self.base_state == other.base_state
            and self.conveyor_state == other.conveyor_state
            and self.config_identity == other.config_identity
        )


@dataclass(frozen=True)
class MotionBoundaryState:
    q: tuple[float, ...]
    qd: tuple[float, ...]
    qdd: tuple[float, ...]
    time_seconds: float
    boundary_mode: BoundaryMode
    predecessor_plan_id: str | None = None

    STOP_TOLERANCE: float = field(default=1e-9, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        q = tuple(float(value) for value in self.q)
        qd = tuple(float(value) for value in self.qd)
        qdd = tuple(float(value) for value in self.qdd)
        mode = BoundaryMode(self.boundary_mode)
        time_seconds = float(self.time_seconds)
        if not q or not qd or not qdd:
            raise ValueError("motion boundary q, qd, and qdd must be non-empty")
        if len(qd) != len(q) or len(qdd) != len(q):
            raise ValueError("motion boundary q, qd, and qdd must have the same DOF")
        if not all(isfinite(value) for values in (q, qd, qdd) for value in values):
            raise ValueError("motion boundary q, qd, and qdd must be finite")
        if not isfinite(time_seconds) or time_seconds < 0.0:
            raise ValueError("motion boundary time_seconds must be finite and non-negative")
        if mode is BoundaryMode.STOP_BOUNDARY and (
            any(abs(value) > self.STOP_TOLERANCE for value in qd)
            or any(abs(value) > self.STOP_TOLERANCE for value in qdd)
        ):
            raise ValueError("STOP_BOUNDARY requires zero qd and qdd within tolerance")
        object.__setattr__(self, "q", q)
        object.__setattr__(self, "qd", qd)
        object.__setattr__(self, "qdd", qdd)
        object.__setattr__(self, "time_seconds", time_seconds)
        object.__setattr__(self, "boundary_mode", mode)

    @classmethod
    def stopped(
        cls,
        q: Sequence[float],
        *,
        time_seconds: float = 0.0,
        predecessor_plan_id: str | None = None,
    ) -> MotionBoundaryState:
        position = tuple(float(value) for value in q)
        zeros = tuple(0.0 for _ in position)
        return cls(
            position,
            zeros,
            zeros,
            time_seconds,
            BoundaryMode.STOP_BOUNDARY,
            predecessor_plan_id,
        )

    def matches(
        self,
        other: MotionBoundaryState,
        *,
        q_atol: float = 1e-6,
        qd_atol: float = 1e-6,
        qdd_atol: float = 1e-6,
        time_atol: float = 1e-6,
        require_predecessor: bool = True,
    ) -> bool:
        if not isinstance(other, MotionBoundaryState) or self.boundary_mode is not other.boundary_mode:
            return False
        if require_predecessor and self.predecessor_plan_id != other.predecessor_plan_id:
            return False
        return bool(
            np.allclose(self.q, other.q, atol=q_atol, rtol=0.0)
            and np.allclose(self.qd, other.qd, atol=qd_atol, rtol=0.0)
            and np.allclose(self.qdd, other.qdd, atol=qdd_atol, rtol=0.0)
            and abs(self.time_seconds - other.time_seconds) <= time_atol
        )

    @property
    def mode(self) -> BoundaryMode:
        return self.boundary_mode

    @property
    def current_q(self) -> tuple[float, ...]:
        return self.q

    @property
    def current_velocity(self) -> tuple[float, ...]:
        return self.qd

    @property
    def current_acceleration(self) -> tuple[float, ...]:
        return self.qdd


@dataclass(frozen=True)
class BackendIdentity:
    backend_name: str
    backend_version: str
    adapter_version: str
    robot_model_fingerprint: str
    world_model_fingerprint: str

    def __post_init__(self) -> None:
        for name in (
            "backend_name",
            "backend_version",
            "adapter_version",
            "robot_model_fingerprint",
            "world_model_fingerprint",
        ):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")


@dataclass(frozen=True)
class BackendProvenance:
    identity: BackendIdentity
    deterministic_seed: int | None = None
    compute_device: str | None = None
    compute_category: str | None = None

    @property
    def backend_name(self) -> str:
        return self.identity.backend_name

    @property
    def backend_version(self) -> str:
        return self.identity.backend_version

    @property
    def adapter_version(self) -> str:
        return self.identity.adapter_version

    @property
    def robot_model_fingerprint(self) -> str:
        return self.identity.robot_model_fingerprint

    @property
    def world_model_fingerprint(self) -> str:
        return self.identity.world_model_fingerprint


@dataclass(frozen=True)
class BackendCapabilities:
    supported_planning_paths: frozenset[PlanningPath]
    supported_boundary_modes: frozenset[BoundaryMode]
    supported_artifact_kinds: frozenset[PlanArtifactKind]
    supports_warm_start: bool = False
    supports_deterministic_seed: bool = False
    supports_attached_object: bool = False
    supports_incremental_world_update: bool = False
    supports_logical_cancel: bool = False
    supports_hard_deadline: bool = False
    supports_revalidation: bool = False
    initialization_required: bool = False
    prewarm_required: bool = False

    def __post_init__(self) -> None:
        paths = frozenset(PlanningPath(value) for value in self.supported_planning_paths)
        boundaries = frozenset(BoundaryMode(value) for value in self.supported_boundary_modes)
        artifacts = frozenset(PlanArtifactKind(value) for value in self.supported_artifact_kinds)
        if not paths or not boundaries or not artifacts:
            raise ValueError("backend capability sets must be non-empty")
        object.__setattr__(self, "supported_planning_paths", paths)
        object.__setattr__(self, "supported_boundary_modes", boundaries)
        object.__setattr__(self, "supported_artifact_kinds", artifacts)

    def supports(self, path: PlanningPath, boundary: MotionBoundaryState) -> bool:
        if path not in self.supported_planning_paths or boundary.mode not in self.supported_boundary_modes:
            return False
        if boundary.mode is BoundaryMode.CONTINUOUS_BOUNDARY:
            return PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY in self.supported_artifact_kinds
        return True

    @property
    def output_artifact_kinds(self) -> frozenset[PlanArtifactKind]:
        return self.supported_artifact_kinds

    @property
    def warm_start_support(self) -> bool:
        return self.supports_warm_start

    @property
    def deterministic_seed_support(self) -> bool:
        return self.supports_deterministic_seed

    @property
    def attached_object_support(self) -> bool:
        return self.supports_attached_object

    @property
    def incremental_world_update_support(self) -> bool:
        return self.supports_incremental_world_update

    @property
    def logical_cancellation_support(self) -> bool:
        return self.supports_logical_cancel

    @property
    def hard_deadline_support(self) -> bool:
        return self.supports_hard_deadline

    @property
    def revalidation_support(self) -> bool:
        return self.supports_revalidation


@dataclass(frozen=True)
class FailureDetails:
    retryable: bool
    scope: FailureScope
    native_code: str = ""
    native_message: str = ""
    allow_path_fallback: bool = False
    allow_backend_fallback: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "scope", FailureScope(self.scope))
        object.__setattr__(self, "metadata", _freeze(self.metadata))


@dataclass(frozen=True)
class PlanningCandidate:
    target_id: str
    candidate_id: str
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        if not self.target_id or not self.candidate_id:
            raise ValueError("planning target and candidate ids must be non-empty")
        object.__setattr__(self, "metadata", _freeze(self.metadata))


@dataclass(frozen=True)
class PlanningRequest:
    request_id: str
    world_snapshot: PlanningWorldSnapshot
    candidates: tuple[PlanningCandidate, ...]
    seed: int = 0
    horizon_index: int = 0
    speculative: bool = False
    replan_reason: ReplanReason = ReplanReason.INITIAL_REQUEST
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)
    motion_boundary: MotionBoundaryState | None = None

    def __post_init__(self) -> None:
        if not self.request_id:
            raise ValueError("planning request id must be non-empty")
        if not isinstance(self.world_snapshot, PlanningWorldSnapshot):
            raise TypeError("planning request must bind a PlanningWorldSnapshot")
        candidates = tuple(self.candidates)
        if not candidates:
            raise ValueError("planning request must contain at least one candidate")
        if self.horizon_index < 0:
            raise ValueError("planning horizon index must be non-negative")
        boundary = self.motion_boundary
        if boundary is None:
            boundary = MotionBoundaryState.stopped(self.world_snapshot.current_q)
        if not isinstance(boundary, MotionBoundaryState):
            raise TypeError("planning request motion_boundary must be a MotionBoundaryState")
        if boundary.current_q != self.world_snapshot.current_q:
            raise ValueError("motion boundary current_q must match the bound world snapshot")
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "motion_boundary", boundary)
        object.__setattr__(self, "metadata", _freeze(self.metadata))

    @property
    def scene_revision(self) -> SceneRevision:
        return self.world_snapshot.scene_revision

    @property
    def start_state(self) -> tuple[float, ...]:
        assert self.motion_boundary is not None
        return self.motion_boundary.current_q


@dataclass(frozen=True)
class PlanningResult:
    status: PlanStatus
    trajectory: tuple[tuple[float, ...], ...] = ()
    target_id: str | None = None
    candidate_id: str | None = None
    message: str = ""
    latency_seconds: float | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)
    artifact_kind: PlanArtifactKind = PlanArtifactKind.GEOMETRIC_PATH
    operational_outcome: OperationalOutcome | None = None
    failure: FailureDetails | None = None
    expected_start_boundary: MotionBoundaryState | None = None
    expected_end_boundary: MotionBoundaryState | None = None

    def __post_init__(self) -> None:
        status = PlanStatus(self.status)
        artifact_kind = PlanArtifactKind(self.artifact_kind)
        operational = None if self.operational_outcome is None else OperationalOutcome(self.operational_outcome)
        trajectory = tuple(tuple(float(value) for value in point) for point in self.trajectory)
        if self.latency_seconds is not None and (
            not isfinite(self.latency_seconds) or self.latency_seconds < 0.0
        ):
            raise ValueError("planning latency must be finite and non-negative")
        if status is PlanStatus.SUCCESS:
            if operational is not None or self.failure is not None:
                raise ValueError("a successful result cannot carry failure outcome details")
            if not trajectory:
                raise ValueError("a successful planning result must contain a trajectory")
            dof = len(trajectory[0])
            if dof == 0 or any(len(point) != dof for point in trajectory):
                raise ValueError("trajectory points must have a consistent non-zero dimension")
            if not all(isfinite(value) for point in trajectory for value in point):
                raise ValueError("trajectory must contain only finite values")
            if not self.target_id or not self.candidate_id:
                raise ValueError("a successful result must identify its target and candidate")
            start_boundary = self.expected_start_boundary
            end_boundary = self.expected_end_boundary
            if start_boundary is None and end_boundary is None and artifact_kind is PlanArtifactKind.GEOMETRIC_PATH:
                start_boundary = MotionBoundaryState.stopped(trajectory[0])
                end_boundary = MotionBoundaryState.stopped(trajectory[-1])
            elif start_boundary is None or end_boundary is None:
                raise ValueError("successful trajectory artifacts require complete start and end boundaries")
            if len(start_boundary.q) != dof or len(end_boundary.q) != dof:
                raise ValueError("proposal boundaries must match trajectory DOF")
            if start_boundary.q != trajectory[0] or end_boundary.q != trajectory[-1]:
                raise ValueError("proposal boundary positions must match trajectory endpoints")
            if artifact_kind is PlanArtifactKind.GEOMETRIC_PATH and (
                start_boundary.boundary_mode is not BoundaryMode.STOP_BOUNDARY
                or end_boundary.boundary_mode is not BoundaryMode.STOP_BOUNDARY
            ):
                raise ValueError("GEOMETRIC_PATH can only declare STOP_BOUNDARY endpoints")
            if (
                start_boundary.boundary_mode is BoundaryMode.CONTINUOUS_BOUNDARY
                or end_boundary.boundary_mode is BoundaryMode.CONTINUOUS_BOUNDARY
            ) and artifact_kind is not PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY:
                raise ValueError("CONTINUOUS_BOUNDARY requires a time-parameterized trajectory")
            if end_boundary.time_seconds < start_boundary.time_seconds:
                raise ValueError("proposal end boundary time cannot precede start boundary time")
            object.__setattr__(self, "expected_start_boundary", start_boundary)
            object.__setattr__(self, "expected_end_boundary", end_boundary)
        elif trajectory:
            raise ValueError("a failed planning result cannot carry an executable trajectory")
        elif self.expected_start_boundary is not None or self.expected_end_boundary is not None:
            raise ValueError("a failed planning result cannot carry motion boundaries")
        failure = self.failure
        if status is not PlanStatus.SUCCESS and failure is None:
            scope = {
                PlanStatus.NO_IK: FailureScope.CANDIDATE_LOCAL,
                PlanStatus.GRASP_CONSTRAINT_FAILED: FailureScope.CANDIDATE_LOCAL,
                PlanStatus.INITIAL_CLEARANCE_FAILED: FailureScope.TARGET_LOCAL,
                PlanStatus.COLLISION: FailureScope.CANDIDATE_LOCAL,
                PlanStatus.NOT_EVALUATED: FailureScope.SCENE_LOCAL,
                PlanStatus.TIMEOUT: FailureScope.BACKEND_LOCAL,
            }[status]
            failure = FailureDetails(
                retryable=status in {PlanStatus.NOT_EVALUATED, PlanStatus.TIMEOUT},
                scope=scope,
                native_code=status.value,
                native_message=self.message,
                allow_path_fallback=True,
                allow_backend_fallback=True,
            )
        if operational is not None and failure is None:
            raise ValueError("an operational failure requires FailureDetails")
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "trajectory", trajectory)
        object.__setattr__(self, "artifact_kind", artifact_kind)
        object.__setattr__(self, "operational_outcome", operational)
        object.__setattr__(self, "failure", failure)
        object.__setattr__(self, "metadata", _freeze(self.metadata))

    @property
    def success(self) -> bool:
        return self.status is PlanStatus.SUCCESS

    @property
    def outcome_category(self) -> OutcomeCategory:
        return OutcomeCategory.OPERATIONAL if self.operational_outcome is not None else OutcomeCategory.DOMAIN

    @property
    def outcome_code(self) -> str:
        return self.operational_outcome.value if self.operational_outcome is not None else self.status.value

    @classmethod
    def succeeded(
        cls,
        candidate: PlanningCandidate,
        trajectory: Sequence[Sequence[float]],
        *,
        latency_seconds: float | None = None,
        message: str = "",
        metadata: Mapping[str, Any] | None = None,
        artifact_kind: PlanArtifactKind = PlanArtifactKind.GEOMETRIC_PATH,
        expected_start_boundary: MotionBoundaryState | None = None,
        expected_end_boundary: MotionBoundaryState | None = None,
    ) -> PlanningResult:
        return cls(
            PlanStatus.SUCCESS,
            tuple(tuple(point) for point in trajectory),
            candidate.target_id,
            candidate.candidate_id,
            message,
            latency_seconds,
            {} if metadata is None else metadata,
            artifact_kind,
            None,
            None,
            expected_start_boundary,
            expected_end_boundary,
        )

    @classmethod
    def failed(
        cls,
        status: PlanStatus,
        candidate: PlanningCandidate,
        *,
        latency_seconds: float | None = None,
        message: str = "",
        metadata: Mapping[str, Any] | None = None,
        failure: FailureDetails | None = None,
    ) -> PlanningResult:
        if PlanStatus(status) is PlanStatus.SUCCESS:
            raise ValueError("use PlanningResult.succeeded for success")
        return cls(
            PlanStatus(status),
            (),
            candidate.target_id,
            candidate.candidate_id,
            message,
            latency_seconds,
            {} if metadata is None else metadata,
            PlanArtifactKind.GEOMETRIC_PATH,
            None,
            failure,
        )

    @classmethod
    def operational_failure(
        cls,
        outcome: OperationalOutcome,
        candidate: PlanningCandidate,
        *,
        failure: FailureDetails,
        latency_seconds: float | None = None,
        message: str = "",
        metadata: Mapping[str, Any] | None = None,
    ) -> PlanningResult:
        return cls(
            PlanStatus.NOT_EVALUATED,
            (),
            candidate.target_id,
            candidate.candidate_id,
            message or failure.native_message,
            latency_seconds,
            {} if metadata is None else metadata,
            PlanArtifactKind.GEOMETRIC_PATH,
            OperationalOutcome(outcome),
            failure,
        )


@dataclass(frozen=True)
class PlanValidationResult:
    status: ValidationStatus
    world_snapshot: PlanningWorldSnapshot
    motion_boundary: MotionBoundaryState
    message: str
    validator_name: str
    validator_version: str
    reused: bool = False
    robot_model_fingerprint: str | None = None
    world_model_fingerprint: str | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", ValidationStatus(self.status))
        if not self.validator_name or not self.validator_version:
            raise ValueError("validator identity must be non-empty")
        object.__setattr__(
            self,
            "robot_model_fingerprint",
            self.robot_model_fingerprint or self.world_snapshot.robot_model_fingerprint,
        )
        object.__setattr__(
            self,
            "world_model_fingerprint",
            self.world_model_fingerprint or self.world_snapshot.world_model_fingerprint,
        )

    @property
    def valid(self) -> bool:
        return self.status is ValidationStatus.VALID

    @property
    def revision(self) -> SceneRevision:
        return self.world_snapshot.scene_revision


# Source-compatible public name retained for existing callers.
PlanValidation = PlanValidationResult


@dataclass(frozen=True)
class PlanEnvelope:
    plan_id: str
    request: PlanningRequest
    candidate: PlanningCandidate
    planning_path: PlanningPath
    result: PlanningResult
    planned_snapshot: PlanningWorldSnapshot
    validated_snapshot: PlanningWorldSnapshot | None
    expected_start_boundary: MotionBoundaryState
    expected_end_boundary: MotionBoundaryState
    predecessor_plan_id: str | None
    planning_generation: int
    proposed_by: BackendProvenance
    artifact_kind: ArtifactKind
    robot_model_fingerprint: str
    world_model_fingerprint: str
    validated_by: str | None = None
    validation_timestamp_seconds: float | None = None
    validation_generation: int | None = None
    speculative: bool = False
    invalidated_by: ReplanReason | None = None
    validation_message: str = "planned against exact revision"

    def __post_init__(self) -> None:
        if not self.plan_id:
            raise ValueError("plan id must be non-empty")
        if not self.result.success:
            raise ValueError("only successful results can be placed in a plan envelope")
        if (self.result.target_id, self.result.candidate_id) != (
            self.candidate.target_id,
            self.candidate.candidate_id,
        ):
            raise ValueError("plan result does not match its candidate")
        if not self.expected_start_boundary.matches(
            self.result.expected_start_boundary,
            require_predecessor=False,
        ):
            raise ValueError("expected plan start boundary does not match proposal")
        if not self.expected_end_boundary.matches(
            self.result.expected_end_boundary,
            require_predecessor=False,
        ):
            raise ValueError("expected plan end boundary does not match proposal")
        if self.planning_generation < 0:
            raise ValueError("planning generation must be non-negative")
        if self.artifact_kind is not self.result.artifact_kind:
            raise ValueError("envelope artifact kind does not match proposal")
        if self.robot_model_fingerprint != self.proposed_by.robot_model_fingerprint:
            raise ValueError("envelope robot model fingerprint does not match provenance")
        if self.world_model_fingerprint != self.proposed_by.world_model_fingerprint:
            raise ValueError("envelope world model fingerprint does not match provenance")
        validation_fields = (
            self.validated_snapshot,
            self.validated_by,
            self.validation_timestamp_seconds,
            self.validation_generation,
        )
        if any(value is not None for value in validation_fields) and not all(
            value is not None for value in validation_fields
        ):
            raise ValueError("validated envelope fields must be populated atomically")
        if self.validation_timestamp_seconds is not None and (
            not isfinite(self.validation_timestamp_seconds) or self.validation_timestamp_seconds < 0.0
        ):
            raise ValueError("validation timestamp must be finite and non-negative")

    @property
    def planned_revision(self) -> SceneRevision:
        return self.planned_snapshot.scene_revision

    @property
    def expected_start_state(self) -> tuple[float, ...]:
        return self.expected_start_boundary.q

    @property
    def expected_end_state(self) -> tuple[float, ...]:
        return self.expected_end_boundary.q

    @property
    def validated_revision(self) -> SceneRevision | None:
        return None if self.validated_snapshot is None else self.validated_snapshot.scene_revision

    @property
    def planned_scene_revision(self) -> SceneRevision:
        return self.planned_revision

    @property
    def validated_scene_revision(self) -> SceneRevision | None:
        return self.validated_revision

    @property
    def executable(self) -> bool:
        return (
            self.invalidated_by is None
            and self.result.success
            and not self.speculative
            and self.validated_snapshot is not None
            and self.validated_by is not None
            and self.validation_timestamp_seconds is not None
            and self.validation_generation is not None
        )

    def invalidate(self, reason: ReplanReason, message: str = "") -> PlanEnvelope:
        return replace(self, invalidated_by=reason, validation_message=message or reason.value)

    def revalidated(
        self,
        validation: PlanValidationResult,
        *,
        timestamp_seconds: float,
        generation: int,
    ) -> PlanEnvelope:
        if not validation.valid:
            raise ValueError("cannot mark a failed validation as valid")
        return replace(
            self,
            validated_snapshot=validation.world_snapshot,
            invalidated_by=None,
            validation_message=validation.message,
            validated_by=f"{validation.validator_name}@{validation.validator_version}",
            validation_timestamp_seconds=float(timestamp_seconds),
            validation_generation=int(generation),
        )


class PlanValidator(ABC):
    """Authoritative, provider-neutral gate between proposals and READY."""

    def __init__(self, name: str, version: str) -> None:
        if not name or not version:
            raise ValueError("validator name and version must be non-empty")
        self.name = name
        self.version = version

    @abstractmethod
    def validate(
        self,
        plan_envelope: PlanEnvelope,
        current_world_snapshot: PlanningWorldSnapshot,
        current_motion_boundary: MotionBoundaryState,
    ) -> PlanValidationResult: ...


class ProviderNeutralPlanValidator(PlanValidator):
    def __init__(self, *, start_tolerance_rad: float = 1e-6) -> None:
        super().__init__("provider-neutral-control-plane", "1")
        self.start_tolerance_rad = float(start_tolerance_rad)

    def validate(
        self,
        plan_envelope: PlanEnvelope,
        current_world_snapshot: PlanningWorldSnapshot,
        current_motion_boundary: MotionBoundaryState,
    ) -> PlanValidationResult:
        valid = True
        message = "provider-neutral contract valid"
        if plan_envelope.planning_generation < 0:
            valid, message = False, "invalid planning generation"
        elif not plan_envelope.planned_snapshot.planning_context_matches(current_world_snapshot):
            valid, message = False, "planning world snapshot or revision changed"
        elif not current_motion_boundary.matches(plan_envelope.request.motion_boundary):
            valid, message = False, "start MotionBoundaryState changed"
        elif current_motion_boundary.predecessor_plan_id != plan_envelope.predecessor_plan_id:
            valid, message = False, "predecessor contract changed"
        elif plan_envelope.robot_model_fingerprint != current_world_snapshot.robot_model_fingerprint:
            valid, message = False, "robot model fingerprint mismatch"
        elif plan_envelope.world_model_fingerprint != current_world_snapshot.world_model_fingerprint:
            valid, message = False, "world model fingerprint mismatch"
        elif (
            current_motion_boundary.mode is BoundaryMode.CONTINUOUS_BOUNDARY
            and plan_envelope.artifact_kind is not ArtifactKind.TIME_PARAMETERIZED_TRAJECTORY
        ):
            valid, message = False, "geometric path cannot satisfy a continuous boundary"
        elif not np.allclose(
            plan_envelope.expected_start_state,
            current_motion_boundary.current_q,
            atol=self.start_tolerance_rad,
            rtol=0.0,
        ):
            valid, message = False, "proposal start does not match motion boundary"
        return PlanValidationResult(
            ValidationStatus.VALID if valid else ValidationStatus.INVALID,
            current_world_snapshot,
            current_motion_boundary,
            message,
            self.name,
            self.version,
        )


class BackendPlanValidatorAdapter(PlanValidator):
    """Compatibility gate for v0.5.1 backends with a validation hook.

    Provider-neutral checks run first. The selected backend hook is then called
    through this separate validator interface and cannot bypass those checks.
    """

    def __init__(
        self,
        backends: Sequence[PlannerBackend],
        *,
        start_tolerance_rad: float = 1e-6,
    ) -> None:
        super().__init__("backend-validation-compatibility-adapter", "1")
        self._backends = {backend.identity.backend_name: backend for backend in backends}
        self._neutral = ProviderNeutralPlanValidator(start_tolerance_rad=start_tolerance_rad)

    def validate(
        self,
        plan_envelope: PlanEnvelope,
        current_world_snapshot: PlanningWorldSnapshot,
        current_motion_boundary: MotionBoundaryState,
    ) -> PlanValidationResult:
        neutral = self._neutral.validate(
            plan_envelope,
            current_world_snapshot,
            current_motion_boundary,
        )
        if not neutral.valid:
            return neutral
        backend = self._backends.get(plan_envelope.proposed_by.backend_name)
        if backend is None:
            return PlanValidationResult(
                ValidationStatus.INVALID,
                current_world_snapshot,
                current_motion_boundary,
                "proposal backend is not registered in this validator",
                self.name,
                self.version,
            )
        result = backend.validate_plan(plan_envelope, current_world_snapshot)
        if not isinstance(result, PlanValidationResult):
            raise TypeError("PlannerBackend.validate_plan must return PlanValidationResult")
        return result


class PlannerBackend(ABC):
    """Replaceable bottom-layer planner contract with deterministic seeding."""

    def __init__(
        self,
        seed: int = 0,
        *,
        identity: BackendIdentity | None = None,
        capabilities: BackendCapabilities | None = None,
        compute_device: str | None = None,
        compute_category: str | None = None,
    ) -> None:
        self.seed = int(seed)
        self.identity = identity or BackendIdentity("legacy-backend", "unspecified", "1", "*", "*")
        self.capabilities = capabilities or BackendCapabilities(
            frozenset(PlanningPath),
            frozenset({BoundaryMode.STOP_BOUNDARY}),
            frozenset({PlanArtifactKind.GEOMETRIC_PATH}),
            supports_warm_start=True,
            supports_deterministic_seed=True,
            supports_attached_object=True,
            supports_revalidation=True,
        )
        self.compute_device = compute_device
        self.compute_category = compute_category

    @property
    def health(self) -> BackendHealth:
        return BackendHealth.READY

    @property
    def provenance(self) -> BackendProvenance:
        return BackendProvenance(
            self.identity,
            self.seed if self.capabilities.supports_deterministic_seed else None,
            self.compute_device,
            self.compute_category,
        )

    def set_seed(self, seed: int) -> None:
        self.seed = int(seed)

    def initialize(self) -> None: ...

    def prewarm(self) -> None: ...

    def logical_cancel(self, request_id: str) -> None: ...

    def shutdown(self) -> None: ...

    @abstractmethod
    def plan(
        self,
        request: PlanningRequest,
        candidate: PlanningCandidate,
        planning_path: PlanningPath,
    ) -> PlanningResult:
        """Return SUCCESS or an explicit fail-closed status."""

    def validate_plan(self, envelope: PlanEnvelope, snapshot: PlanningWorldSnapshot) -> PlanValidationResult:
        """Legacy hook retained for adapters; it is never the authoritative gate."""
        boundary = envelope.request.motion_boundary
        assert boundary is not None
        valid = envelope.planned_snapshot.planning_context_matches(snapshot)
        return PlanValidationResult(
            ValidationStatus.VALID if valid else ValidationStatus.INVALID,
            snapshot,
            boundary,
            "planning world identity matches" if valid else "planning world identity changed",
            self.identity.backend_name,
            self.identity.adapter_version,
            reused=valid and snapshot != envelope.planned_snapshot,
        )


def normalize_plan_status(value: str | PlanStatus) -> PlanStatus:
    """Map legacy planner reasons into the control-plane status vocabulary."""

    if isinstance(value, PlanStatus):
        return value
    reason = str(value).upper()
    if reason in {"OK", "SUCCESS"}:
        return PlanStatus.SUCCESS
    if "NO_IK" in reason:
        return PlanStatus.NO_IK
    if "GRASP" in reason and ("FAIL" in reason or "CONSTRAINT" in reason):
        return PlanStatus.GRASP_CONSTRAINT_FAILED
    if "INITIAL_CLEARANCE" in reason or "INITIAL_ATTACHED_STATE" in reason:
        return PlanStatus.INITIAL_CLEARANCE_FAILED
    if "COLLISION" in reason:
        return PlanStatus.COLLISION
    if "TIMEOUT" in reason or "TIME_LIMIT" in reason or "TIME LIMIT" in reason:
        return PlanStatus.TIMEOUT
    return PlanStatus.NOT_EVALUATED


class FailurePolicy:
    """Deterministic retry policy; it never promotes a failure to success."""

    def decide(
        self,
        status: PlanStatus,
        *,
        has_next_path: bool,
        has_next_candidate: bool,
        has_next_target: bool,
    ) -> FailureAction:
        if has_next_path:
            return FailureAction.ESCALATE_PATH
        if has_next_candidate:
            return FailureAction.TRY_NEXT_CANDIDATE
        if has_next_target:
            return FailureAction.TRY_NEXT_TARGET
        if status is PlanStatus.NOT_EVALUATED:
            return FailureAction.WAIT_FOR_SCENE
        return FailureAction.BLOCK


@dataclass(frozen=True)
class PlanningLatencySample:
    request_id: str
    planning_path: PlanningPath
    status: PlanStatus
    seconds: float
    speculative: bool
    backend_name: str = "unspecified"
    outcome_category: OutcomeCategory = OutcomeCategory.DOMAIN
    outcome_code: str = "NOT_EVALUATED"
    queue_seconds: float = 0.0
    validation_seconds: float = 0.0
    end_to_end_seconds: float | None = None


class PlanningLatencyStatistics:
    """Planning and robot-idle metrics only; no production cycle claim."""

    def __init__(self, clock: Callable[[], float] = perf_counter) -> None:
        self._clock = clock
        self.samples: list[PlanningLatencySample] = []
        self.backend_init_samples: dict[str, list[float]] = {}
        self.backend_warmup_samples: dict[str, list[float]] = {}
        self.fallback_count_by_path: Counter[str] = Counter()
        self.outcome_count_by_backend_category: dict[str, Counter[str]] = {}
        self.unsupported_route_skip_count = 0
        self._idle_started: float | None = None
        self.robot_idle_waiting_seconds = 0.0

    def record(self, sample: PlanningLatencySample) -> None:
        if not isfinite(sample.seconds) or sample.seconds < 0.0:
            raise ValueError("planning latency must be finite and non-negative")
        self.samples.append(sample)
        key = f"{sample.outcome_category.value}:{sample.outcome_code}"
        self.outcome_count_by_backend_category.setdefault(sample.backend_name, Counter())[key] += 1

    def record_lifecycle(self, backend_name: str, phase: str, seconds: float) -> None:
        if not isfinite(seconds) or seconds < 0.0:
            raise ValueError("backend lifecycle latency must be finite and non-negative")
        target = self.backend_init_samples if phase == "initialize" else self.backend_warmup_samples
        target.setdefault(backend_name, []).append(float(seconds))

    def record_fallback(self, path: PlanningPath) -> None:
        self.fallback_count_by_path[path.value] += 1

    def record_unsupported_route(self) -> None:
        self.unsupported_route_skip_count += 1

    def begin_robot_idle(self) -> None:
        if self._idle_started is None:
            self._idle_started = self._clock()

    def end_robot_idle(self) -> None:
        if self._idle_started is not None:
            elapsed = self._clock() - self._idle_started
            if elapsed < 0.0:
                raise RuntimeError("statistics clock moved backwards")
            self.robot_idle_waiting_seconds += elapsed
            self._idle_started = None

    @staticmethod
    def _summary(values: Sequence[float]) -> dict[str, float | int | None]:
        if not values:
            return {"count": 0, "total_seconds": 0.0, "mean_seconds": None, "p95_seconds": None, "max_seconds": None}
        ordered = sorted(values)
        p95_index = max(0, ceil(0.95 * len(ordered)) - 1)
        return {
            "count": len(ordered),
            "total_seconds": float(sum(ordered)),
            "mean_seconds": float(sum(ordered) / len(ordered)),
            "p95_seconds": float(ordered[p95_index]),
            "max_seconds": float(ordered[-1]),
        }

    def report(self) -> dict[str, Any]:
        idle = self.robot_idle_waiting_seconds
        if self._idle_started is not None:
            idle += max(0.0, self._clock() - self._idle_started)
        by_path = {
            path.value: self._summary([sample.seconds for sample in self.samples if sample.planning_path is path])
            for path in PlanningPath
        }
        queue = [sample.queue_seconds for sample in self.samples]
        validation = [sample.validation_seconds for sample in self.samples if sample.status is PlanStatus.SUCCESS]
        end_to_end = [
            sample.end_to_end_seconds if sample.end_to_end_seconds is not None else sample.queue_seconds + sample.seconds + sample.validation_seconds
            for sample in self.samples
        ]
        return {
            "model": "online_control_plane_latency_v2",
            "planning_latency": self._summary([sample.seconds for sample in self.samples]),
            "planning_latency_by_path": by_path,
            "backend_init_latency": {
                name: self._summary(values) for name, values in sorted(self.backend_init_samples.items())
            },
            "backend_warmup_latency": {
                name: self._summary(values) for name, values in sorted(self.backend_warmup_samples.items())
            },
            "queue_latency": self._summary(queue),
            "planning_compute_latency": self._summary([sample.seconds for sample in self.samples]),
            "validation_latency": self._summary(validation),
            "end_to_end_planning_latency": self._summary(end_to_end),
            "planning_status_counts": dict(sorted(Counter(sample.status.value for sample in self.samples).items())),
            "fallback_count_by_path": {
                path.value: self.fallback_count_by_path[path.value] for path in PlanningPath
            },
            "outcome_count_by_backend_category": {
                name: dict(sorted(counts.items()))
                for name, counts in sorted(self.outcome_count_by_backend_category.items())
            },
            "unsupported_route_skip_count": self.unsupported_route_skip_count,
            "speculative_planning_count": sum(sample.speculative for sample in self.samples),
            "robot_idle_waiting_for_planner_seconds": float(idle),
            "production_throughput": None,
            "timing_scope": "planning latency and robot idle waiting for planner only",
        }


@dataclass
class _ExecutorTask:
    task_id: str
    operation: Callable[[], PlanningResult]
    submitted_at: float


@dataclass
class _ExecutorCompletion:
    task_id: str
    result: PlanningResult | None
    elapsed_seconds: float
    queue_seconds: float = 0.0
    error: Exception | None = None


class PlanningExecutor(ABC):
    @abstractmethod
    def submit(self, task_id: str, operation: Callable[[], PlanningResult]) -> None: ...

    @abstractmethod
    def advance(self, max_tasks: int = 1) -> int: ...

    @abstractmethod
    def pop_completed(self) -> _ExecutorCompletion | None: ...

    @property
    @abstractmethod
    def pending_count(self) -> int: ...


class DeterministicAsyncPlanningExecutor(PlanningExecutor):
    """FIFO cooperative executor with deterministic completion ordering."""

    def __init__(self, clock: Callable[[], float] = perf_counter) -> None:
        self._clock = clock
        self._pending: deque[_ExecutorTask] = deque()
        self._completed: deque[_ExecutorCompletion] = deque()
        self._ids: set[str] = set()

    def submit(self, task_id: str, operation: Callable[[], PlanningResult]) -> None:
        if task_id in self._ids:
            raise ValueError(f"duplicate planning task id: {task_id}")
        self._ids.add(task_id)
        self._pending.append(_ExecutorTask(task_id, operation, self._clock()))

    def advance(self, max_tasks: int = 1) -> int:
        if max_tasks < 0:
            raise ValueError("max_tasks must be non-negative")
        completed = 0
        while self._pending and completed < max_tasks:
            task = self._pending.popleft()
            started = self._clock()
            queue_seconds = started - task.submitted_at
            if queue_seconds < 0.0:
                raise RuntimeError("executor clock moved backwards")
            try:
                result = task.operation()
                item = _ExecutorCompletion(task.task_id, result, self._clock() - started, queue_seconds)
            except Exception as exc:  # Backend failure is a recovery event, not a deadlock.
                item = _ExecutorCompletion(task.task_id, None, self._clock() - started, queue_seconds, exc)
            self._completed.append(item)
            completed += 1
        return completed

    def pop_completed(self) -> _ExecutorCompletion | None:
        if not self._completed:
            return None
        item = self._completed.popleft()
        self._ids.remove(item.task_id)
        return item

    @property
    def pending_count(self) -> int:
        return len(self._pending) + len(self._completed)


class SynchronousPlanningExecutor(DeterministicAsyncPlanningExecutor):
    """Same FIFO semantics, but submission computes the task immediately."""

    def submit(self, task_id: str, operation: Callable[[], PlanningResult]) -> None:
        super().submit(task_id, operation)
        self.advance(1)


class ExecutionMonitor:
    """Tracks the sole executable plan and blocks unvalidated continuation."""

    def __init__(self) -> None:
        self.state = ExecutionState.IDLE
        self.active_plan: PlanEnvelope | None = None
        self.progress = 0.0
        self.message = ""

    def start(self, envelope: PlanEnvelope) -> None:
        if self.state in {ExecutionState.RUNNING, ExecutionState.STOPPING}:
            raise RuntimeError("an execution is already active")
        if not envelope.executable:
            raise ValueError("cannot execute an invalidated plan")
        self.active_plan = envelope
        self.progress = 0.0
        self.message = "execution started"
        self.state = ExecutionState.RUNNING

    def update_progress(self, progress: float) -> None:
        if self.state is not ExecutionState.RUNNING:
            raise RuntimeError("execution is not running")
        if not isfinite(progress) or progress < self.progress or progress > 1.0:
            raise ValueError("execution progress must be finite, monotonic, and inside [0, 1]")
        self.progress = float(progress)

    def begin_stopping(self, message: str) -> PlanEnvelope:
        if self.state is not ExecutionState.RUNNING or self.active_plan is None:
            raise RuntimeError("no running execution to stop")
        plan = self.active_plan.invalidate(ReplanReason.SCENE_REVISION_CHANGED, message)
        self.active_plan = plan
        self.state = ExecutionState.STOPPING
        self.message = message
        return plan

    def complete(self, success: bool, message: str = "") -> PlanEnvelope:
        if self.state is not ExecutionState.RUNNING or self.active_plan is None:
            raise RuntimeError("no running execution to complete")
        plan = self.active_plan
        self.progress = 1.0 if success else self.progress
        self.state = ExecutionState.COMPLETED if success else ExecutionState.FAILED
        self.message = message or ("execution completed" if success else "execution failed")
        self.active_plan = None
        return plan

    def fail_for_deviation(self, message: str) -> PlanEnvelope:
        if self.state is not ExecutionState.RUNNING or self.active_plan is None:
            raise RuntimeError("no running execution")
        plan = self.active_plan.invalidate(ReplanReason.EXECUTION_DEVIATION, message)
        self.active_plan = None
        self.state = ExecutionState.FAILED
        self.message = message
        return plan

    def acknowledge_stop(self, message: str = "stop acknowledged") -> PlanEnvelope:
        if self.state is not ExecutionState.STOPPING or self.active_plan is None:
            raise RuntimeError("no stopping execution to acknowledge")
        plan = self.active_plan
        self.active_plan = None
        self.state = ExecutionState.FAILED
        self.message = message
        return plan


@dataclass
class _PlanningRoute:
    planning_path: PlanningPath
    backend_index: int


@dataclass
class _RequestProgress:
    request: PlanningRequest
    generation: int
    predecessor_plan_id: str | None = None
    candidate_index: int = 0
    route_index: int = 0
    routes: tuple[_PlanningRoute, ...] = ()
    attempts: list[PlanningResult] = field(default_factory=list)


@dataclass(frozen=True)
class SessionEvent:
    sequence: int
    kind: str
    request_id: str | None
    reason: str
    details: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "details", _freeze(self.details))


class ContinuousPlanningSession:
    """Rolling-horizon state machine independent of geometric success rate."""

    def __init__(
        self,
        backend: PlannerBackend | Sequence[PlannerBackend],
        *,
        validator: PlanValidator | None = None,
        executor: PlanningExecutor | None = None,
        rolling_horizon: int = 2,
        planning_paths: Sequence[PlanningPath] = (
            PlanningPath.FAST,
            PlanningPath.WARM,
            PlanningPath.COLD,
        ),
        failure_policy: FailurePolicy | None = None,
        clock: Callable[[], float] = perf_counter,
        start_tolerance_rad: float = 1e-6,
    ) -> None:
        if rolling_horizon < 1:
            raise ValueError("rolling horizon must be at least one")
        paths = tuple(PlanningPath(path) for path in planning_paths)
        if not paths or len(set(paths)) != len(paths):
            raise ValueError("planning paths must be non-empty and unique")
        if not isfinite(start_tolerance_rad) or start_tolerance_rad < 0.0:
            raise ValueError("start tolerance must be finite and non-negative")
        if isinstance(backend, PlannerBackend):
            backends = (backend,)
        else:
            backends = tuple(backend)
        if not backends or not all(isinstance(item, PlannerBackend) for item in backends):
            raise TypeError("backend must be a PlannerBackend or a non-empty sequence of them")
        names = [item.identity.backend_name for item in backends]
        if len(names) != len(set(names)):
            raise ValueError("backend names must be unique within a session")
        self.backends = backends
        self.backend = backends[0]
        self.validator = validator or BackendPlanValidatorAdapter(
            backends,
            start_tolerance_rad=start_tolerance_rad,
        )
        if not isinstance(self.validator, PlanValidator):
            raise TypeError("validator must implement PlanValidator")
        self.executor = SynchronousPlanningExecutor(clock) if executor is None else executor
        self.rolling_horizon = int(rolling_horizon)
        self.planning_paths = paths
        self.failure_policy = FailurePolicy() if failure_policy is None else failure_policy
        self.statistics = PlanningLatencyStatistics(clock)
        self.execution = ExecutionMonitor()
        self.state = SessionState.IDLE
        self.current_world: PlanningWorldSnapshot | None = None
        self.ready_plans: deque[PlanEnvelope] = deque()
        self.speculative_plans: deque[PlanEnvelope] = deque()
        self.invalidated_plans: list[PlanEnvelope] = []
        self.events: list[SessionEvent] = []
        self._requests: deque[_RequestProgress] = deque()
        self._active_progress: _RequestProgress | None = None
        self._submitted_task: str | None = None
        self._event_sequence = 0
        self._task_sequence = 0
        self._replan_sequence = 0
        self._last_exhausted_request: PlanningRequest | None = None
        self._planning_generation = 0
        self._request_ids: set[str] = set()
        self._terminal_failures: list[tuple[PlanningRequest, bool]] = []
        self._exhausted_progress: list[_RequestProgress] = []
        self._waiting_for_scene = False
        self._stopping_request: PlanningRequest | None = None
        self._discard_without_replan_ids: set[str] = set()
        self._successful_completed_plan_ids: set[str] = set()
        self.last_successful_plan_id: str | None = None
        self._plan_children: dict[str, set[str]] = {}
        self._invalid_lineage_plan_ids: set[str] = set()
        self._start_tolerance_rad = float(start_tolerance_rad)

    @property
    def terminal_results(self) -> tuple[PlanningResult, ...]:
        return tuple(
            attempt
            for progress in getattr(self, "_exhausted_progress", ())
            for attempt in progress.attempts
        )

    @property
    def successful_completed_plan_ids(self) -> frozenset[str]:
        return frozenset(self._successful_completed_plan_ids)

    @property
    def plan_children(self) -> Mapping[str, frozenset[str]]:
        return MappingProxyType(
            {parent: frozenset(children) for parent, children in self._plan_children.items()}
        )

    @property
    def known_request_ids(self) -> frozenset[str]:
        return frozenset(self._request_ids)

    @property
    def occupancy(self) -> int:
        executing = int(
            self.execution.active_plan is not None
            and self.execution.state in {ExecutionState.RUNNING, ExecutionState.STOPPING}
        )
        return (
            executing
            + len(self.ready_plans)
            + len(self.speculative_plans)
            + len(self._requests)
            + int(self._active_progress is not None)
        )

    def _has_live_successor(self, predecessor_plan_id: str) -> bool:
        return bool(
            any(plan.predecessor_plan_id == predecessor_plan_id for plan in self.ready_plans)
            or any(plan.predecessor_plan_id == predecessor_plan_id for plan in self.speculative_plans)
            or any(progress.predecessor_plan_id == predecessor_plan_id for progress in self._requests)
            or (
                self._active_progress is not None
                and self._active_progress.predecessor_plan_id == predecessor_plan_id
            )
        )

    def initialize_backends(self) -> None:
        for backend in self.backends:
            name = backend.identity.backend_name
            if backend.capabilities.initialization_required:
                started = self.statistics._clock()
                try:
                    backend.initialize()
                except Exception as exc:
                    self._event("backend_initialize_error", OperationalOutcome.BACKEND_UNAVAILABLE.value, error=repr(exc), backend=name)
                    continue
                self.statistics.record_lifecycle(name, "initialize", self.statistics._clock() - started)
            if backend.capabilities.prewarm_required and backend.health is BackendHealth.READY:
                started = self.statistics._clock()
                try:
                    backend.prewarm()
                except Exception as exc:
                    self._event("backend_prewarm_error", OperationalOutcome.BACKEND_UNAVAILABLE.value, error=repr(exc), backend=name)
                    continue
                self.statistics.record_lifecycle(name, "prewarm", self.statistics._clock() - started)

    def shutdown_backends(self) -> None:
        for backend in self.backends:
            backend.shutdown()

    def _logical_cancel_active(self) -> None:
        progress = self._active_progress
        if progress is None or self._submitted_task is None or not progress.routes:
            return
        route = progress.routes[progress.route_index]
        backend = self.backends[route.backend_index]
        if not backend.capabilities.supports_logical_cancel:
            return
        try:
            backend.logical_cancel(progress.request.request_id)
            self._event(
                "planning_cancel_requested",
                OperationalOutcome.CANCELLED.value,
                progress.request.request_id,
                backend=backend.identity.backend_name,
            )
        except Exception as exc:
            self._event(
                "planning_cancel_error",
                OperationalOutcome.BACKEND_ERROR.value,
                progress.request.request_id,
                backend=backend.identity.backend_name,
                error=repr(exc),
            )

    def _register_plan_lineage(self, envelope: PlanEnvelope) -> None:
        predecessor = envelope.predecessor_plan_id
        if predecessor is None:
            return
        active = self.execution.active_plan
        predecessor_known = bool(
            predecessor in self._successful_completed_plan_ids
            or (active is not None and active.plan_id == predecessor)
        )
        if not predecessor_known or predecessor in self._invalid_lineage_plan_ids:
            raise ValueError("plan proposal has an unknown, orphan, or invalid predecessor")
        self._plan_children.setdefault(predecessor, set()).add(envelope.plan_id)

    def _cascade_lineage(
        self,
        predecessor_plan_id: str,
        reason: ReplanReason,
        message: str,
    ) -> None:
        affected = {predecessor_plan_id}
        pending = [predecessor_plan_id]
        while pending:
            parent = pending.pop()
            for child in self._plan_children.get(parent, ()):
                if child not in affected:
                    affected.add(child)
                    pending.append(child)
        self._invalid_lineage_plan_ids.update(affected)

        def invalidate_plans(plans: deque[PlanEnvelope]) -> deque[PlanEnvelope]:
            retained: deque[PlanEnvelope] = deque()
            while plans:
                plan = plans.popleft()
                if plan.plan_id in affected or plan.predecessor_plan_id in affected:
                    self.invalidated_plans.append(plan.invalidate(reason, message))
                    self._event(
                        "lineage_invalidated",
                        reason.value,
                        plan.request.request_id,
                        plan_id=plan.plan_id,
                        predecessor_plan_id=plan.predecessor_plan_id,
                    )
                else:
                    retained.append(plan)
            return retained

        self.ready_plans = invalidate_plans(self.ready_plans)
        self.speculative_plans = invalidate_plans(self.speculative_plans)

        retained_requests: deque[_RequestProgress] = deque()
        while self._requests:
            progress = self._requests.popleft()
            if progress.predecessor_plan_id in affected:
                self._event(
                    "lineage_invalidated",
                    reason.value,
                    progress.request.request_id,
                    predecessor_plan_id=progress.predecessor_plan_id,
                )
            else:
                retained_requests.append(progress)
        self._requests = retained_requests

        if (
            self._active_progress is not None
            and self._active_progress.predecessor_plan_id in affected
        ):
            request_id = self._active_progress.request.request_id
            self._discard_without_replan_ids.add(request_id)
            self._logical_cancel_active()
            self._event(
                "lineage_invalidated",
                reason.value,
                request_id,
                predecessor_plan_id=self._active_progress.predecessor_plan_id,
            )
            if self._submitted_task is None:
                self._active_progress = None

    def _routes_for(self, request: PlanningRequest) -> tuple[_PlanningRoute, ...]:
        routes: list[_PlanningRoute] = []
        boundary = request.motion_boundary
        assert boundary is not None
        for path in self.planning_paths:
            for index, backend in enumerate(self.backends):
                supported = backend.capabilities.supports(path, boundary)
                if request.world_snapshot.payload_attachment is not None and not backend.capabilities.supports_attached_object:
                    supported = False
                if supported:
                    routes.append(_PlanningRoute(path, index))
                else:
                    self.statistics.record_unsupported_route()
                    self._event(
                        "unsupported_route_skipped",
                        OperationalOutcome.PATH_UNSUPPORTED.value,
                        request.request_id,
                        backend=backend.identity.backend_name,
                        path=path.value,
                        boundary=boundary.mode.value,
                    )
        return tuple(routes)

    def _event(self, kind: str, reason: str, request_id: str | None = None, **details: Any) -> None:
        self._event_sequence += 1
        self.events.append(SessionEvent(self._event_sequence, kind, request_id, reason, details))

    @property
    def blocked(self) -> bool:
        return self.state is SessionState.BLOCKED

    @property
    def robot_idle_waiting_for_planner(self) -> bool:
        return self.state is SessionState.PLANNING and self.execution.state is not ExecutionState.RUNNING

    @property
    def current_revision(self) -> SceneRevision | None:
        return None if self.current_world is None else self.current_world.scene_revision

    @property
    def current_state(self) -> tuple[float, ...] | None:
        return None if self.current_world is None else self.current_world.current_q

    def submit(self, request: PlanningRequest) -> None:
        self._submit(request, predecessor_plan_id=None)

    def _submit(self, request: PlanningRequest, predecessor_plan_id: str | None) -> None:
        if not isinstance(request, PlanningRequest):
            raise TypeError("request must be a PlanningRequest")
        declared_predecessor = request.motion_boundary.predecessor_plan_id
        if predecessor_plan_id is None and request.speculative:
            raise ValueError("speculative requests require submit_speculative and a live predecessor")
        if predecessor_plan_id is None and declared_predecessor is not None:
            raise ValueError("root request cannot declare an unknown or orphan predecessor")
        if predecessor_plan_id is not None:
            active = self.execution.active_plan
            if (
                active is None
                or self.execution.state is not ExecutionState.RUNNING
                or active.plan_id != predecessor_plan_id
            ):
                raise ValueError("speculative predecessor must be the currently executing plan")
            if declared_predecessor != predecessor_plan_id:
                raise ValueError("request motion boundary has the wrong predecessor")
            if self._has_live_successor(predecessor_plan_id):
                raise RuntimeError("a predecessor may have at most one live successor")
        if request.request_id in self._request_ids:
            raise ValueError(f"duplicate request id: {request.request_id}")
        if self.occupancy >= self.rolling_horizon:
            raise RuntimeError("rolling horizon is full")
        if self.current_world is None:
            self.current_world = request.world_snapshot
        elif not request.speculative:
            self._check_world_monotonic(request.world_snapshot)
            if request.world_snapshot != self.current_world:
                raise ValueError("non-speculative request must bind the current planning world snapshot")
        elif request.scene_revision.sequence <= self.current_world.scene_revision.sequence:
            raise ValueError("speculative request must bind a future scene revision")
        self._request_ids.add(request.request_id)
        self._requests.append(
            _RequestProgress(
                request,
                self._planning_generation,
                predecessor_plan_id,
                routes=self._routes_for(request),
            )
        )
        self._terminal_failures.clear()
        self._waiting_for_scene = False
        self._event("request_submitted", request.replan_reason.value, request.request_id, speculative=request.speculative)
        if self.execution.state is not ExecutionState.RUNNING:
            self.state = SessionState.PLANNING
            self.statistics.begin_robot_idle()
        self._schedule_if_possible()

    def submit_speculative(self, request: PlanningRequest) -> None:
        if self.execution.state is not ExecutionState.RUNNING:
            raise RuntimeError("speculative next planning requires plan k to be executing")
        if not request.speculative:
            request = replace(request, speculative=True, replan_reason=ReplanReason.ROLLING_HORIZON)
        active = self.execution.active_plan
        assert active is not None
        boundary = request.motion_boundary
        assert boundary is not None
        if not active.expected_end_boundary.matches(
            boundary,
            q_atol=self._start_tolerance_rad,
            qd_atol=self._start_tolerance_rad,
            qdd_atol=self._start_tolerance_rad,
            time_atol=self._start_tolerance_rad,
            require_predecessor=False,
        ):
            raise ValueError("speculative request must match plan k predicted end boundary")
        request = replace(
            request,
            motion_boundary=replace(boundary, predecessor_plan_id=active.plan_id),
        )
        self._submit(request, predecessor_plan_id=active.plan_id)

    def _schedule_if_possible(self) -> None:
        if self.state in {SessionState.STOPPING, SessionState.RECOVERY} or self.execution.state is ExecutionState.STOPPING:
            return
        if self._submitted_task is not None:
            return
        if self._active_progress is None:
            if not self._requests:
                return
            self._active_progress = self._requests.popleft()
        progress = self._active_progress
        if not progress.routes:
            self._event(
                "request_exhausted",
                "BLOCKED",
                progress.request.request_id,
                attempts=0,
                operational_reason=OperationalOutcome.PATH_UNSUPPORTED.value,
            )
            self._terminal_failures.append((progress.request, False))
            self._exhausted_progress.append(progress)
            self._active_progress = None
            self._schedule_if_possible()
            return
        candidate = progress.request.candidates[progress.candidate_index]
        route = progress.routes[progress.route_index]
        path = route.planning_path
        backend = self.backends[route.backend_index]
        self._task_sequence += 1
        task_id = f"{progress.request.request_id}:{candidate.candidate_id}:{path.value}:{backend.identity.backend_name}:{self._task_sequence}"
        self._submitted_task = task_id
        request = progress.request

        def operation() -> PlanningResult:
            if backend.health not in {BackendHealth.READY, BackendHealth.DEGRADED}:
                return PlanningResult.operational_failure(
                    OperationalOutcome.BACKEND_UNAVAILABLE,
                    candidate,
                    failure=FailureDetails(
                        True,
                        FailureScope.BACKEND_LOCAL,
                        native_code=backend.health.value,
                        native_message="backend is not ready",
                        allow_path_fallback=True,
                        allow_backend_fallback=True,
                    ),
                )
            if backend.capabilities.supports_deterministic_seed:
                backend.set_seed(request.seed)
            return backend.plan(request, candidate, path)

        self.executor.submit(task_id, operation)
        self._event("planning_started", path.value, request.request_id, target=candidate.target_id, candidate=candidate.candidate_id, backend=backend.identity.backend_name)

    def advance(self, max_tasks: int = 1) -> SessionState:
        if max_tasks < 0:
            raise ValueError("max_tasks must be non-negative")
        remaining = max_tasks
        while remaining > 0:
            self._schedule_if_possible()
            if self._submitted_task is None:
                break
            self.executor.advance(1)
            completion = self.executor.pop_completed()
            if completion is None:
                break
            remaining -= 1
            self._consume(completion)
        self._refresh_state()
        return self.state

    def run_until_stable(self, max_tasks: int = 1000) -> SessionState:
        for _ in range(max_tasks):
            before = (self._submitted_task, len(self._requests), self.state, len(self.ready_plans), len(self.speculative_plans))
            self.advance(1)
            after = (self._submitted_task, len(self._requests), self.state, len(self.ready_plans), len(self.speculative_plans))
            if self._submitted_task is None and not self._requests and self._active_progress is None:
                return self.state
            if before == after and self.executor.pending_count == 0:
                return self.state
        raise RuntimeError("planning session did not stabilize within max_tasks")

    def _consume(self, completion: _ExecutorCompletion) -> None:
        if completion.task_id != self._submitted_task or self._active_progress is None:
            raise RuntimeError("executor returned an out-of-order completion")
        self._submitted_task = None
        progress = self._active_progress
        request = progress.request
        candidate = request.candidates[progress.candidate_index]
        route = progress.routes[progress.route_index]
        path = route.planning_path
        backend = self.backends[route.backend_index]
        if progress.generation != self._planning_generation:
            self._event(
                "stale_result_discarded",
                ReplanReason.SCENE_REVISION_CHANGED.value,
                request.request_id,
                result_generation=progress.generation,
                current_generation=self._planning_generation,
            )
            self._active_progress = None
            already_requeued = any(
                item.request.request_id.startswith(f"{request.request_id}:replan:")
                for item in self._requests
            )
            if (
                self.state is not SessionState.STOPPING
                and self.current_world is not None
                and not already_requeued
                and request.request_id not in self._discard_without_replan_ids
            ):
                self._queue_replan(request, self.current_world, ReplanReason.SCENE_REVISION_CHANGED)
            self._discard_without_replan_ids.discard(request.request_id)
            self._schedule_if_possible()
            return
        if progress.predecessor_plan_id in self._invalid_lineage_plan_ids:
            self._event(
                "stale_lineage_result_discarded",
                ReplanReason.PLAN_INVALIDATED.value,
                request.request_id,
                predecessor_plan_id=progress.predecessor_plan_id,
            )
            self._discard_without_replan_ids.discard(request.request_id)
            self._active_progress = None
            self._schedule_if_possible()
            return
        if completion.error is not None:
            result = PlanningResult.operational_failure(
                OperationalOutcome.BACKEND_ERROR,
                candidate,
                failure=FailureDetails(
                    True,
                    FailureScope.BACKEND_LOCAL,
                    native_code=type(completion.error).__name__,
                    native_message=str(completion.error),
                    allow_path_fallback=True,
                    allow_backend_fallback=True,
                    metadata={"exception_repr": repr(completion.error)},
                ),
            )
            self._event(
                "backend_error",
                OperationalOutcome.BACKEND_ERROR.value,
                request.request_id,
                error=repr(completion.error),
                backend=backend.identity.backend_name,
            )
        elif not isinstance(completion.result, PlanningResult):
            self._event(
                "backend_contract_error",
                ReplanReason.PLANNER_FAILURE.value,
                request.request_id,
                error=f"expected PlanningResult, got {type(completion.result).__name__}",
            )
            self._active_progress = None
            self.state = SessionState.RECOVERY
            self.statistics.end_robot_idle()
            return
        else:
            result = completion.result
        if result.status is PlanStatus.TIMEOUT and result.operational_outcome is None:
            result = replace(result, operational_outcome=OperationalOutcome.DEADLINE_EXCEEDED)
        if (result.target_id, result.candidate_id) != (candidate.target_id, candidate.candidate_id):
            self._event("backend_contract_error", ReplanReason.PLANNER_FAILURE.value, request.request_id)
            self._active_progress = None
            self.state = SessionState.RECOVERY
            self.statistics.end_robot_idle()
            return
        if result.success:
            expected_dof = len(request.world_snapshot.current_q)
            if any(len(point) != expected_dof for point in result.trajectory) or not np.allclose(
                result.trajectory[0],
                request.world_snapshot.current_q,
                atol=self._start_tolerance_rad,
                rtol=0.0,
            ):
                self._event(
                    "backend_contract_error",
                    ReplanReason.PLANNER_FAILURE.value,
                    request.request_id,
                    error="trajectory DOF/start does not match bound world snapshot",
                )
                self._active_progress = None
                self.state = SessionState.RECOVERY
                self.statistics.end_robot_idle()
                return
            if result.artifact_kind not in backend.capabilities.supported_artifact_kinds:
                self._event(
                    "backend_contract_error",
                    ReplanReason.PLANNER_FAILURE.value,
                    request.request_id,
                    error="proposal artifact kind was not declared by backend capabilities",
                )
                self._active_progress = None
                self.state = SessionState.RECOVERY
                self.statistics.end_robot_idle()
                return
        latency = completion.elapsed_seconds if result.latency_seconds is None else result.latency_seconds
        result = replace(result, latency_seconds=latency)
        progress.attempts.append(result)
        validation_seconds = 0.0
        if result.success:
            validation_started = self.statistics._clock()
            try:
                envelope = self._envelope(request, candidate, path, result, backend)
                envelope, validation = self._validate(
                    envelope,
                    request.world_snapshot,
                    request.motion_boundary,
                )
            except Exception as exc:
                if "envelope" in locals():
                    self.invalidated_plans.append(
                        envelope.invalidate(ReplanReason.PLAN_VALIDATION_FAILED, repr(exc))
                    )
                self._event(
                    "validation_error",
                    ReplanReason.PLAN_VALIDATION_FAILED.value,
                    request.request_id,
                    error=repr(exc),
                )
                self._active_progress = None
                self.state = SessionState.RECOVERY
                self.statistics.end_robot_idle()
                return
            validation_seconds = self.statistics._clock() - validation_started
            if not validation.valid:
                self.invalidated_plans.append(
                    envelope.invalidate(ReplanReason.PLAN_VALIDATION_FAILED, validation.message)
                )
                self._event(
                    "proposal_rejected",
                    ReplanReason.PLAN_VALIDATION_FAILED.value,
                    request.request_id,
                    message=validation.message,
                    backend=backend.identity.backend_name,
                )
            else:
                sample = PlanningLatencySample(
                    request.request_id,
                    path,
                    result.status,
                    latency,
                    request.speculative,
                    backend.identity.backend_name,
                    result.outcome_category,
                    result.outcome_code,
                    completion.queue_seconds,
                    validation_seconds,
                    completion.queue_seconds + latency + validation_seconds,
                )
                self.statistics.record(sample)
                self._event("planning_finished", result.outcome_code, request.request_id, path=path.value, latency_seconds=latency, backend=backend.identity.backend_name)
                (self.speculative_plans if request.speculative else self.ready_plans).append(envelope)
                self._active_progress = None
                self._event("plan_ready", path.value, request.request_id, plan_id=envelope.plan_id, speculative=request.speculative)
                self._schedule_if_possible()
                return

        sample = PlanningLatencySample(
            request.request_id,
            path,
            result.status,
            latency,
            request.speculative,
            backend.identity.backend_name,
            result.outcome_category,
            result.outcome_code,
            completion.queue_seconds,
            validation_seconds,
            completion.queue_seconds + latency + validation_seconds,
        )
        self.statistics.record(sample)
        self._event("planning_finished", result.outcome_code, request.request_id, path=path.value, latency_seconds=latency, backend=backend.identity.backend_name)

        next_route_index: int | None = None
        for index in range(progress.route_index + 1, len(progress.routes)):
            next_route = progress.routes[index]
            same_path = next_route.planning_path is path
            failure = result.failure
            if failure is not None:
                if same_path and not failure.allow_backend_fallback:
                    continue
                if not same_path and not failure.allow_path_fallback:
                    continue
            next_route_index = index
            break
        if next_route_index is not None:
            self.statistics.record_fallback(path)
            progress.route_index = next_route_index
            self._event(
                "failure_decision",
                FailureAction.ESCALATE_PATH.value,
                request.request_id,
                status=result.outcome_code,
            )
            self._schedule_if_possible()
            return

        next_candidate_index = progress.candidate_index + 1
        if next_candidate_index < len(request.candidates):
            next_candidate = request.candidates[next_candidate_index]
            action = (
                FailureAction.TRY_NEXT_CANDIDATE
                if next_candidate.target_id == candidate.target_id
                else FailureAction.TRY_NEXT_TARGET
            )
            progress.candidate_index = next_candidate_index
            progress.route_index = 0
            self._event("failure_decision", action.value, request.request_id, status=result.outcome_code)
            self._schedule_if_possible()
            return

        self._active_progress = None
        all_not_evaluated = bool(progress.attempts) and all(
            attempt.operational_outcome is None and attempt.status is PlanStatus.NOT_EVALUATED
            for attempt in progress.attempts
        )
        self._last_exhausted_request = request if all_not_evaluated else None
        self._terminal_failures.append((request, all_not_evaluated))
        self._exhausted_progress.append(progress)
        self._event(
            "request_exhausted",
            "WAITING_FOR_SCENE" if all_not_evaluated else "BLOCKED",
            request.request_id,
            attempts=len(progress.attempts),
        )
        if not all_not_evaluated:
            self._event("failure_decision", FailureAction.BLOCK.value, request.request_id, status=result.outcome_code)
        self._schedule_if_possible()

    def _envelope(
        self,
        request: PlanningRequest,
        candidate: PlanningCandidate,
        path: PlanningPath,
        result: PlanningResult,
        backend: PlannerBackend,
    ) -> PlanEnvelope:
        identity = {
            "request": request.request_id,
            "revision": request.scene_revision.fingerprint,
            "target": candidate.target_id,
            "candidate": candidate.candidate_id,
            "path": path.value,
            "artifact_kind": result.artifact_kind.value,
            "backend": backend.identity.backend_name,
            "trajectory": result.trajectory,
            "start_boundary": {
                "q": result.expected_start_boundary.q,
                "qd": result.expected_start_boundary.qd,
                "qdd": result.expected_start_boundary.qdd,
                "time_seconds": result.expected_start_boundary.time_seconds,
                "mode": result.expected_start_boundary.boundary_mode.value,
            },
            "end_boundary": {
                "q": result.expected_end_boundary.q,
                "qd": result.expected_end_boundary.qd,
                "qdd": result.expected_end_boundary.qdd,
                "time_seconds": result.expected_end_boundary.time_seconds,
                "mode": result.expected_end_boundary.boundary_mode.value,
            },
            "predecessor": self._active_progress.predecessor_plan_id if self._active_progress else None,
            "generation": self._active_progress.generation if self._active_progress else self._planning_generation,
        }
        plan_id = hashlib.sha256(
            json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()[:20]
        start_boundary = request.motion_boundary
        result_start = result.expected_start_boundary
        result_end = result.expected_end_boundary
        assert start_boundary is not None and result_start is not None and result_end is not None
        if not start_boundary.matches(result_start, require_predecessor=False):
            raise ValueError("proposal start boundary does not match request boundary")
        end_boundary = replace(result_end, predecessor_plan_id=plan_id)
        provenance = backend.provenance
        if provenance.robot_model_fingerprint == "*" or provenance.world_model_fingerprint == "*":
            provenance = BackendProvenance(
                BackendIdentity(
                    provenance.backend_name,
                    provenance.backend_version,
                    provenance.adapter_version,
                    request.world_snapshot.robot_model_fingerprint,
                    request.world_snapshot.world_model_fingerprint,
                ),
                provenance.deterministic_seed,
                provenance.compute_device,
                provenance.compute_category,
            )
        envelope = PlanEnvelope(
            plan_id=plan_id,
            request=request,
            candidate=candidate,
            planning_path=path,
            result=result,
            planned_snapshot=request.world_snapshot,
            validated_snapshot=None,
            expected_start_boundary=start_boundary,
            expected_end_boundary=end_boundary,
            predecessor_plan_id=self._active_progress.predecessor_plan_id if self._active_progress else None,
            planning_generation=self._active_progress.generation if self._active_progress else self._planning_generation,
            proposed_by=provenance,
            artifact_kind=result.artifact_kind,
            robot_model_fingerprint=provenance.robot_model_fingerprint,
            world_model_fingerprint=provenance.world_model_fingerprint,
            speculative=request.speculative,
        )
        self._register_plan_lineage(envelope)
        return envelope

    def _refresh_state(self) -> None:
        if self.state is SessionState.RECOVERY:
            return
        if self.execution.state is ExecutionState.STOPPING:
            self.state = SessionState.STOPPING
            self.statistics.end_robot_idle()
        elif self.execution.state is ExecutionState.RUNNING:
            self.state = SessionState.EXECUTING
            self.statistics.end_robot_idle()
        elif self.ready_plans:
            self.state = SessionState.READY
            self.statistics.end_robot_idle()
        elif self._submitted_task is not None or self._active_progress is not None or self._requests:
            self.state = SessionState.PLANNING
            self.statistics.begin_robot_idle()
        elif self._waiting_for_scene or (
            self._terminal_failures and all(item[1] for item in self._terminal_failures)
        ):
            self.state = SessionState.WAITING_FOR_SCENE
            self.statistics.end_robot_idle()
        elif self._terminal_failures:
            self.state = SessionState.BLOCKED
            self.statistics.end_robot_idle()
        else:
            self.state = SessionState.IDLE
            self.statistics.end_robot_idle()

    def _validate(
        self,
        envelope: PlanEnvelope,
        snapshot: PlanningWorldSnapshot,
        motion_boundary: MotionBoundaryState | None = None,
    ) -> tuple[PlanEnvelope, PlanValidationResult]:
        boundary = motion_boundary or replace(
            envelope.request.motion_boundary,
            q=snapshot.current_q,
            predecessor_plan_id=envelope.predecessor_plan_id,
        )
        trajectory = envelope.result.trajectory
        dof = len(snapshot.current_q)
        if not trajectory or any(len(point) != dof for point in trajectory):
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "trajectory DOF does not match actual robot state", self.validator.name, self.validator.version)
        elif not all(isfinite(value) for point in trajectory for value in point):
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "trajectory contains non-finite values", self.validator.name, self.validator.version)
        elif tuple(trajectory[0]) != tuple(envelope.expected_start_state) or tuple(trajectory[-1]) != tuple(envelope.expected_end_state):
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "trajectory endpoints do not match envelope contract", self.validator.name, self.validator.version)
        elif not np.allclose(
            trajectory[0], snapshot.current_q, atol=self._start_tolerance_rad, rtol=0.0
        ):
            error = float(np.max(np.abs(np.asarray(trajectory[0]) - np.asarray(snapshot.current_q))))
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, f"trajectory start differs from actual current_q by {error:.9f} rad", self.validator.name, self.validator.version)
        elif envelope.robot_model_fingerprint != snapshot.robot_model_fingerprint:
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "robot model fingerprint mismatch", self.validator.name, self.validator.version)
        elif envelope.world_model_fingerprint != snapshot.world_model_fingerprint:
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "world model fingerprint mismatch", self.validator.name, self.validator.version)
        elif not envelope.planned_snapshot.planning_context_matches(snapshot):
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "scene/config/tool/payload/base/conveyor context changed", self.validator.name, self.validator.version)
        elif boundary.predecessor_plan_id != envelope.predecessor_plan_id:
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "predecessor contract mismatch", self.validator.name, self.validator.version)
        elif boundary.mode is BoundaryMode.CONTINUOUS_BOUNDARY and envelope.artifact_kind is not ArtifactKind.TIME_PARAMETERIZED_TRAJECTORY:
            validation = PlanValidationResult(ValidationStatus.INVALID, snapshot, boundary, "geometric path cannot satisfy continuous boundary", self.validator.name, self.validator.version)
        else:
            try:
                validation = self.validator.validate(envelope, snapshot, boundary)
            except Exception as exc:
                raise RuntimeError(f"authoritative plan validation failed: {exc}") from exc
            if not isinstance(validation, PlanValidationResult):
                raise TypeError("PlanValidator.validate must return PlanValidationResult")
            if (
                validation.world_snapshot != snapshot
                or not validation.motion_boundary.matches(boundary)
                or validation.robot_model_fingerprint != snapshot.robot_model_fingerprint
                or validation.world_model_fingerprint != snapshot.world_model_fingerprint
            ):
                raise ValueError("validation result is bound to the wrong world or motion boundary")
        return (
            envelope.revalidated(
                validation,
                timestamp_seconds=self.statistics._clock(),
                generation=self._planning_generation,
            )
            if validation.valid
            else envelope,
            validation,
        )

    def start_execution(
        self,
        current_motion_boundary: MotionBoundaryState | None = None,
    ) -> PlanEnvelope | None:
        if self.execution.state in {ExecutionState.RUNNING, ExecutionState.STOPPING}:
            raise RuntimeError("an execution is already active")
        if self._waiting_for_scene or not self.ready_plans or self.current_world is None:
            self._refresh_state()
            return None
        envelope = self.ready_plans.popleft()
        predecessor = envelope.predecessor_plan_id
        if envelope.plan_id in self._invalid_lineage_plan_ids or (
            predecessor is not None
            and (
                predecessor not in self._successful_completed_plan_ids
                or predecessor != self.last_successful_plan_id
            )
        ):
            self.invalidated_plans.append(
                envelope.invalidate(
                    ReplanReason.PLAN_INVALIDATED,
                    "predecessor contract is unknown, invalid, or not the last successful plan",
                )
            )
            self._event(
                "lineage_invalidated",
                ReplanReason.PLAN_INVALIDATED.value,
                envelope.request.request_id,
                predecessor_plan_id=predecessor,
            )
            self.state = SessionState.RECOVERY
            self.statistics.end_robot_idle()
            return None
        if current_motion_boundary is None:
            requested_boundary = envelope.request.motion_boundary
            assert requested_boundary is not None
            if requested_boundary.mode is BoundaryMode.CONTINUOUS_BOUNDARY:
                self.invalidated_plans.append(
                    envelope.invalidate(
                        ReplanReason.PLAN_VALIDATION_FAILED,
                        "actual continuous MotionBoundaryState is required before execution",
                    )
                )
                self._event(
                    "plan_invalidated",
                    ReplanReason.PLAN_VALIDATION_FAILED.value,
                    envelope.request.request_id,
                )
                self.state = SessionState.RECOVERY
                self.statistics.end_robot_idle()
                return None
            current_motion_boundary = MotionBoundaryState.stopped(
                self.current_world.current_q,
                predecessor_plan_id=envelope.predecessor_plan_id,
            )
        elif current_motion_boundary.current_q != self.current_world.current_q:
            raise ValueError("actual motion boundary current_q must match the current world snapshot")
        try:
            envelope, validation = self._validate(
                envelope,
                self.current_world,
                current_motion_boundary,
            )
        except Exception as exc:
            self.invalidated_plans.append(
                envelope.invalidate(ReplanReason.PLAN_VALIDATION_FAILED, repr(exc))
            )
            self._event("validation_error", ReplanReason.PLANNER_FAILURE.value, envelope.request.request_id, error=repr(exc))
            self.state = SessionState.RECOVERY
            self.statistics.end_robot_idle()
            return None
        if not validation.valid:
            invalid = envelope.invalidate(ReplanReason.PLAN_VALIDATION_FAILED, validation.message)
            self.invalidated_plans.append(invalid)
            self._event("plan_invalidated", ReplanReason.PLAN_VALIDATION_FAILED.value, envelope.request.request_id)
            self._queue_replan(envelope.request, self.current_world, ReplanReason.PLAN_VALIDATION_FAILED)
            self._refresh_state()
            return None
        self.execution.start(envelope)
        self.statistics.end_robot_idle()
        self.state = SessionState.EXECUTING
        self._event("execution_started", "validated", envelope.request.request_id, plan_id=envelope.plan_id)
        return envelope

    def _check_world_monotonic(self, snapshot: PlanningWorldSnapshot) -> None:
        if not isinstance(snapshot, PlanningWorldSnapshot):
            raise TypeError("scene update must provide a PlanningWorldSnapshot")
        previous = self.current_world
        if previous is None:
            return
        old_scene, new_scene = previous.scene_revision, snapshot.scene_revision
        if new_scene.sequence < old_scene.sequence:
            raise ValueError("scene revision cannot move backwards")
        if new_scene.sequence == old_scene.sequence and not new_scene.same_scene(old_scene):
            raise ValueError("same sequence cannot identify different scene fingerprints")
        old_robot = previous.robot_state_revision
        new_robot = snapshot.robot_state_revision
        if new_robot.sequence < old_robot.sequence:
            raise ValueError("robot state revision cannot move backwards")
        if new_robot.sequence == old_robot.sequence and new_robot.fingerprint != old_robot.fingerprint:
            raise ValueError("same sequence cannot identify different robot state fingerprints")

    @staticmethod
    def _scene_revision_changed(
        previous: PlanningWorldSnapshot | None, snapshot: PlanningWorldSnapshot
    ) -> bool:
        return previous is None or previous.scene_revision != snapshot.scene_revision

    def _new_replan_progress(
        self,
        request: PlanningRequest,
        snapshot: PlanningWorldSnapshot,
        reason: ReplanReason,
        predecessor_plan_id: str | None = None,
    ) -> _RequestProgress:
        self._replan_sequence += 1
        request_id = f"{request.request_id}:replan:{self._replan_sequence}"
        while request_id in self._request_ids:
            self._replan_sequence += 1
            request_id = f"{request.request_id}:replan:{self._replan_sequence}"
        replanned = replace(
            request,
            request_id=request_id,
            world_snapshot=snapshot,
            motion_boundary=replace(
                request.motion_boundary,
                q=snapshot.current_q,
                predecessor_plan_id=predecessor_plan_id,
            ),
            speculative=False,
            replan_reason=reason,
        )
        self._request_ids.add(request_id)
        return _RequestProgress(
            replanned,
            self._planning_generation,
            predecessor_plan_id,
            routes=self._routes_for(replanned),
        )

    def update_scene(self, snapshot: PlanningWorldSnapshot) -> None:
        self._check_world_monotonic(snapshot)
        previous = self.current_world
        changed = self._scene_revision_changed(previous, snapshot)
        self.current_world = snapshot
        self._event(
            "scene_updated",
            ReplanReason.SCENE_REVISION_CHANGED.value if changed else "ROBOT_STATE_UPDATED",
            None,
            sequence=snapshot.scene_revision.sequence,
        )
        if not changed:
            self._refresh_state()
            return

        self._planning_generation += 1
        self._logical_cancel_active()
        self._terminal_failures.clear()
        was_waiting = self._waiting_for_scene
        self._waiting_for_scene = False

        if self.execution.state is ExecutionState.RUNNING and self.execution.active_plan is not None:
            active = self.execution.begin_stopping("scene revision changed during execution")
            self.invalidated_plans.append(active)
            self._cascade_lineage(
                active.plan_id,
                ReplanReason.SCENE_REVISION_CHANGED,
                "predecessor entered STOPPING after scene change",
            )
            self._stopping_request = active.request
            if self._active_progress is not None:
                self._discard_without_replan_ids.add(self._active_progress.request.request_id)
            while self._requests:
                stale = self._requests.popleft()
                self._event("pending_request_invalidated", ReplanReason.SCENE_REVISION_CHANGED.value, stale.request.request_id)
            while self.speculative_plans:
                self.invalidated_plans.append(
                    self.speculative_plans.popleft().invalidate(ReplanReason.SCENE_REVISION_CHANGED)
                )
            self.state = SessionState.STOPPING
            self.statistics.end_robot_idle()
            self._event("execution_stopping", ReplanReason.SCENE_REVISION_CHANGED.value, active.request.request_id)
            return

        if self.execution.state is ExecutionState.STOPPING:
            while self._requests:
                stale = self._requests.popleft()
                self._event("pending_request_invalidated", ReplanReason.SCENE_REVISION_CHANGED.value, stale.request.request_id)
            self.state = SessionState.STOPPING
            return

        pending = list(self._requests)
        self._requests.clear()
        for progress in pending:
            self._requests.append(
                self._new_replan_progress(
                    progress.request,
                    snapshot,
                    ReplanReason.SCENE_REVISION_CHANGED,
                    progress.predecessor_plan_id,
                )
            )

        while self.ready_plans:
            plan = self.ready_plans.popleft()
            self.invalidated_plans.append(
                plan.invalidate(ReplanReason.SCENE_REVISION_CHANGED, "scene revision changed")
            )
            self._requests.append(
                self._new_replan_progress(plan.request, snapshot, ReplanReason.SCENE_REVISION_CHANGED)
            )

        if was_waiting and self.speculative_plans:
            self._reconcile_speculative()
        elif self.speculative_plans:
            while self.speculative_plans:
                plan = self.speculative_plans.popleft()
                self.invalidated_plans.append(plan.invalidate(ReplanReason.SPECULATIVE_MISMATCH))
                self._requests.append(
                    self._new_replan_progress(plan.request, snapshot, ReplanReason.SPECULATIVE_MISMATCH)
                )

        if self._last_exhausted_request is not None and not self._requests and self._active_progress is None:
            exhausted = self._last_exhausted_request
            self._last_exhausted_request = None
            self._requests.append(
                self._new_replan_progress(exhausted, snapshot, ReplanReason.SCENE_REVISION_CHANGED)
            )
        self._schedule_if_possible()
        self._refresh_state()

    def complete_execution(
        self,
        *,
        success: bool,
        stopped_q: Sequence[float],
        world_snapshot: PlanningWorldSnapshot | None = None,
        message: str = "",
    ) -> PlanEnvelope:
        if self.execution.state is ExecutionState.STOPPING:
            raise RuntimeError("stopping execution requires acknowledge_stop")
        q = tuple(float(value) for value in stopped_q)
        if not q or not all(isfinite(value) for value in q):
            raise ValueError("actual stopped_q must be finite and non-empty")
        if self.current_world is None:
            raise RuntimeError("no current planning world")
        if len(q) != len(self.current_world.current_q):
            raise ValueError("actual stopped_q has the wrong DOF")
        previous_world = self.current_world
        if world_snapshot is not None:
            self._check_world_monotonic(world_snapshot)
            if not np.allclose(world_snapshot.current_q, q, atol=self._start_tolerance_rad, rtol=0.0):
                raise ValueError("world snapshot current_q must equal actual stopped_q")
        completed = self.execution.complete(success, message)
        self._event(
            "execution_completed" if success else "execution_failed",
            "SUCCESS" if success else ReplanReason.EXECUTION_FAILED.value,
            completed.request.request_id,
        )
        if not success:
            self._cascade_lineage(
                completed.plan_id,
                ReplanReason.EXECUTION_FAILED,
                "predecessor execution failed",
            )
            self.state = SessionState.RECOVERY
            return completed
        self._successful_completed_plan_ids.add(completed.plan_id)
        self.last_successful_plan_id = completed.plan_id
        if world_snapshot is None:
            robot = RobotStateRevision(
                previous_world.robot_state_revision.sequence + 1,
                q,
                previous_world.robot_state_revision.robot_state,
            )
            self.current_world = replace(previous_world, robot_state_revision=robot)
            self._waiting_for_scene = True
            while self.ready_plans:
                self.invalidated_plans.append(
                    self.ready_plans.popleft().invalidate(
                        ReplanReason.PLAN_INVALIDATED,
                        "execution completed without a new scene revision",
                    )
                )
            self.state = SessionState.WAITING_FOR_SCENE
            self.statistics.end_robot_idle()
            return completed

        self.current_world = world_snapshot
        changed = self._scene_revision_changed(previous_world, world_snapshot)
        if not changed:
            self._waiting_for_scene = True
            self.state = SessionState.WAITING_FOR_SCENE
            self.statistics.end_robot_idle()
            return completed
        self._planning_generation += 1
        self._logical_cancel_active()
        self._waiting_for_scene = False
        while self.ready_plans:
            stale = self.ready_plans.popleft()
            self.invalidated_plans.append(
                stale.invalidate(
                    ReplanReason.SCENE_REVISION_CHANGED,
                    "predecessor execution produced a new scene revision",
                )
            )
            self._requests.append(
                self._new_replan_progress(
                    stale.request,
                    world_snapshot,
                    ReplanReason.SCENE_REVISION_CHANGED,
                )
            )
        self._reconcile_speculative()
        self._schedule_if_possible()
        self._refresh_state()
        return completed

    def acknowledge_stop(
        self,
        stopped_world: PlanningWorldSnapshot,
        message: str = "scene-invalidated execution stopped",
    ) -> PlanEnvelope:
        """Accept the real stopped_q and new scene before starting any replan."""

        if self.execution.state is not ExecutionState.STOPPING:
            raise RuntimeError("session is not waiting for a stop acknowledgement")
        self._check_world_monotonic(stopped_world)
        if self.current_world is not None and not stopped_world.scene_revision.same_scene(
            self.current_world.scene_revision
        ):
            raise ValueError("stop acknowledgement must include the latest scene")
        plan = self.execution.acknowledge_stop(message)
        self.current_world = stopped_world
        self._waiting_for_scene = False
        self._stopping_request = None
        while self.speculative_plans:
            self.invalidated_plans.append(
                self.speculative_plans.popleft().invalidate(ReplanReason.EXECUTION_FAILED)
            )
        self._requests.append(
            self._new_replan_progress(plan.request, stopped_world, ReplanReason.SCENE_REVISION_CHANGED)
        )
        self.state = SessionState.PLANNING
        self.statistics.begin_robot_idle()
        self._event("stop_acknowledged", ReplanReason.PLAN_INVALIDATED.value, plan.request.request_id)
        self._schedule_if_possible()
        return plan

    def stop_invalidated_execution(
        self,
        stopped_world: PlanningWorldSnapshot,
        message: str = "scene-invalidated execution stopped",
    ) -> PlanEnvelope:
        return self.acknowledge_stop(stopped_world, message)

    def _reconcile_speculative(self) -> None:
        if not self.speculative_plans:
            return
        if self.current_world is None:
            self.state = SessionState.WAITING_FOR_SCENE
            return
        while self.speculative_plans:
            plan = self.speculative_plans.popleft()
            if (
                plan.predecessor_plan_id not in self._successful_completed_plan_ids
                or plan.predecessor_plan_id != self.last_successful_plan_id
            ):
                invalid = plan.invalidate(
                    ReplanReason.PLAN_INVALIDATED,
                    "successor predecessor is not the last successfully completed plan",
                )
                self.invalidated_plans.append(invalid)
                self._invalid_lineage_plan_ids.add(plan.plan_id)
                self.state = SessionState.RECOVERY
                self._event(
                    "lineage_invalidated",
                    ReplanReason.PLAN_INVALIDATED.value,
                    plan.request.request_id,
                    predecessor_plan_id=plan.predecessor_plan_id,
                )
                return
            try:
                validated, result = self._validate(plan, self.current_world)
            except Exception as exc:
                self.invalidated_plans.append(
                    plan.invalidate(ReplanReason.PLAN_VALIDATION_FAILED, repr(exc))
                )
                self.state = SessionState.RECOVERY
                self._event("validation_error", ReplanReason.PLANNER_FAILURE.value, plan.request.request_id, error=repr(exc))
                return
            if result.valid:
                self.ready_plans.append(replace(validated, speculative=False))
                self._event("speculative_reused", "scene_match", plan.request.request_id, plan_id=plan.plan_id)
            else:
                invalid = plan.invalidate(ReplanReason.SPECULATIVE_MISMATCH, result.message)
                self.invalidated_plans.append(invalid)
                self._event("plan_invalidated", ReplanReason.SPECULATIVE_MISMATCH.value, plan.request.request_id)
                self._queue_replan(plan.request, self.current_world, ReplanReason.SPECULATIVE_MISMATCH)

    def notify_execution_deviation(self, message: str) -> PlanEnvelope:
        plan = self.execution.fail_for_deviation(message)
        self.invalidated_plans.append(plan)
        self._cascade_lineage(
            plan.plan_id,
            ReplanReason.EXECUTION_DEVIATION,
            "predecessor execution deviated",
        )
        if self.current_world is not None:
            self._queue_replan(plan.request, self.current_world, ReplanReason.EXECUTION_DEVIATION)
        self.state = SessionState.RECOVERY
        self._event("execution_deviation", ReplanReason.EXECUTION_DEVIATION.value, plan.request.request_id)
        return plan

    def _queue_replan(
        self,
        request: PlanningRequest,
        snapshot: PlanningWorldSnapshot,
        reason: ReplanReason,
    ) -> None:
        declared_predecessor = request.motion_boundary.predecessor_plan_id
        predecessor = (
            declared_predecessor
            if declared_predecessor in self._successful_completed_plan_ids
            and declared_predecessor == self.last_successful_plan_id
            else None
        )
        progress = self._new_replan_progress(request, snapshot, reason, predecessor)
        self._requests.append(progress)
        self._event("replan_queued", reason.value, progress.request.request_id)
        self._schedule_if_possible()

    def resume_after_recovery(self) -> SessionState:
        if self.state is not SessionState.RECOVERY:
            raise RuntimeError("session is not in recovery")
        self.state = SessionState.PLANNING if (
            self._submitted_task is not None or self._active_progress is not None or self._requests
        ) else SessionState.IDLE
        if self.state is SessionState.PLANNING:
            self.statistics.begin_robot_idle()
            self._schedule_if_possible()
        return self.state

    def snapshot(self) -> dict[str, Any]:
        return {
            "state": self.state.value,
            "scene_revision": None if self.current_revision is None else {
                "sequence": self.current_revision.sequence,
                "fingerprint": self.current_revision.fingerprint,
            },
            "execution_state": self.execution.state.value,
            "ready_plan_ids": [plan.plan_id for plan in self.ready_plans],
            "speculative_plan_ids": [plan.plan_id for plan in self.speculative_plans],
            "invalidated_plan_ids": [plan.plan_id for plan in self.invalidated_plans],
            "pending_requests": len(self._requests) + int(self._active_progress is not None),
            "occupancy": self.occupancy,
            "known_request_ids": sorted(self._request_ids),
            "successful_completed_plan_ids": sorted(self._successful_completed_plan_ids),
            "last_successful_plan_id": self.last_successful_plan_id,
            "plan_children": {
                parent: sorted(children) for parent, children in sorted(self._plan_children.items())
            },
            "planning_generation": self._planning_generation,
            "blocked": self.blocked,
            "metrics": self.statistics.report(),
        }


__all__ = [
    "ArtifactKind",
    "BackendPlanValidatorAdapter",
    "BackendCapabilities",
    "BackendHealth",
    "BackendIdentity",
    "BackendProvenance",
    "BoundaryMode",
    "ContinuousPlanningSession",
    "DeterministicAsyncPlanningExecutor",
    "ExecutionMonitor",
    "ExecutionState",
    "FailureAction",
    "FailureDetails",
    "FailurePolicy",
    "FailureScope",
    "MotionBoundaryState",
    "OperationalOutcome",
    "OutcomeCategory",
    "PlanEnvelope",
    "PlanArtifactKind",
    "PlanValidationResult",
    "PlanValidator",
    "PlanningCandidate",
    "PlanningExecutor",
    "PlanningLatencySample",
    "PlanningLatencyStatistics",
    "PlanningPath",
    "PlanningRequest",
    "PlanningResult",
    "PlanningWorldSnapshot",
    "PlannerBackend",
    "PlanStatus",
    "PlanValidation",
    "ProviderNeutralPlanValidator",
    "ReplanReason",
    "RobotStateRevision",
    "SceneRevision",
    "SessionEvent",
    "SessionState",
    "SynchronousPlanningExecutor",
    "ValidationStatus",
    "normalize_plan_status",
    "scene_fingerprint",
]
