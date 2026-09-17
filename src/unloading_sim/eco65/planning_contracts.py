"""Exact selected definitions from src/unloading_sim/online_planning.py @ f7f7e0934ad425ab62cd2df8201812df1ba8e505.
Unused orchestration deliberately omitted; local contract types only.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from collections import Counter, OrderedDict, deque

from dataclasses import dataclass, field, replace

from enum import Enum

import hashlib

import json

from math import ceil, isfinite

from threading import Condition, Thread

from time import monotonic, perf_counter

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
    SCENE_STALE = "SCENE_STALE"
    PLANNING_TIMEOUT = "PLANNING_TIMEOUT"
    EXECUTION_FEEDBACK_TIMEOUT = "EXECUTION_FEEDBACK_TIMEOUT"
    STOP_ACK_TIMEOUT = "STOP_ACK_TIMEOUT"
    STOP_OBSERVATION_TIMEOUT = "STOP_OBSERVATION_TIMEOUT"
    STOP_EVIDENCE_CONFLICT = "STOP_EVIDENCE_CONFLICT"
    BACKEND_UNHEALTHY = "BACKEND_UNHEALTHY"

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
