"""Dependency-free, fail-closed contracts shared by perception and planning.

The planning names and constructor order mirror ``feat/v0.5-online-continuous``
at the SHA recorded in ``integration/manifest.json``.  ROS messages are wire
mappings of these types; they are deliberately not imported here.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
from enum import Enum
import hashlib
import json
from math import isfinite, sqrt
from numbers import Integral, Real
from types import MappingProxyType
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable


SCHEMA_VERSION = "1.1.0"


class Validity(str, Enum):
    UNKNOWN = "UNKNOWN"
    VALID = "VALID"
    INVALID = "INVALID"
    NOT_EVALUATED = "NOT_EVALUATED"


class EvidenceKind(str, Enum):
    OBSERVED = "OBSERVED"
    MODEL_ESTIMATED = "MODEL_ESTIMATED"
    CONSTRAINT_COMPLETED = "CONSTRAINT_COMPLETED"
    SYNTHETIC = "SYNTHETIC"


class ObservationStatus(str, Enum):
    COMPLETE = "COMPLETE"
    PARTIAL = "PARTIAL"
    BACKEND_UNAVAILABLE = "BACKEND_UNAVAILABLE"
    FAILED = "FAILED"
    STALE = "STALE"


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
    BACKEND_UNHEALTHY = "BACKEND_UNHEALTHY"


class ExecutionEventKind(str, Enum):
    ACCEPTED = "ACCEPTED"
    STARTED = "STARTED"
    FEEDBACK = "FEEDBACK"
    CANCEL_ACCEPTED = "CANCEL_ACCEPTED"
    CANCELED = "CANCELED"
    STOPPING = "STOPPING"
    STOP_CONFIRMED = "STOP_CONFIRMED"
    SUCCEEDED = "SUCCEEDED"
    REJECTED = "REJECTED"
    FAILED = "FAILED"


def _finite_tuple(values: Sequence[float], length: int | None = None, name: str = "values") -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if length is not None and len(result) != length:
        raise ValueError(f"{name} must contain {length} values")
    if not result or not all(isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def deep_freeze(value: Any) -> Any:
    if isinstance(value, Enum):
        return value
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): deep_freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(deep_freeze(item) for item in value)
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        number = float(value)
        if not isfinite(number):
            raise ValueError("contract values must be finite")
        return number
    to_list = getattr(value, "tolist", None)
    if callable(to_list):
        return deep_freeze(to_list())
    raise TypeError(f"unsupported mutable contract value: {type(value).__name__}")


def _canonical(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _canonical(item) for key, item in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    if hasattr(value, "__dataclass_fields__"):
        return {
            name: _canonical(getattr(value, name))
            for name, definition in value.__dataclass_fields__.items()
            if definition.init
        }
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        number = float(value)
        if not isfinite(number):
            raise ValueError("fingerprint input must be finite")
        return number
    to_list = getattr(value, "tolist", None)
    if callable(to_list):
        return _canonical(to_list())
    raise TypeError(f"cannot fingerprint {type(value).__name__}")


def canonical_fingerprint(value: Any) -> str:
    payload = json.dumps(_canonical(value), ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


scene_fingerprint = canonical_fingerprint


@dataclass(frozen=True)
class ResourceReference:
    uri: str
    sha256: str | None = None
    media_type: str | None = None

    def __post_init__(self) -> None:
        if not self.uri:
            raise ValueError("resource uri must be non-empty")
        if self.sha256 is not None and (len(self.sha256) != 64 or any(c not in "0123456789abcdef" for c in self.sha256.lower())):
            raise ValueError("resource sha256 must contain 64 hexadecimal characters")


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    normalized: bool
    source: str
    calibration_version: str | None = None

    def __post_init__(self) -> None:
        values = (float(self.fx), float(self.fy), float(self.cx), float(self.cy))
        if not all(isfinite(value) for value in values) or values[0] <= 0.0 or values[1] <= 0.0:
            raise ValueError("camera intrinsics must be finite with positive focal lengths")
        if not self.source:
            raise ValueError("intrinsics source must be non-empty")
        for name, value in zip(("fx", "fy", "cx", "cy"), values):
            object.__setattr__(self, name, value)

    def pixels(self, width: int, height: int) -> CameraIntrinsics:
        if not self.normalized:
            return self
        return CameraIntrinsics(
            self.fx * width, self.fy * height, self.cx * width, self.cy * height,
            False, self.source, self.calibration_version,
        )


@dataclass(frozen=True)
class ImageMapping:
    source_width: int
    source_height: int
    crop_xywh: tuple[int, int, int, int] | None = None
    scale_xy: tuple[float, float] = (1.0, 1.0)

    def __post_init__(self) -> None:
        if self.source_width <= 0 or self.source_height <= 0:
            raise ValueError("source image dimensions must be positive")
        scale = _finite_tuple(self.scale_xy, 2, "image scale")
        if any(value <= 0.0 for value in scale):
            raise ValueError("image scale must be positive")
        if self.crop_xywh is not None:
            crop = tuple(int(value) for value in self.crop_xywh)
            if len(crop) != 4 or any(value < 0 for value in crop[:2]) or any(value <= 0 for value in crop[2:]):
                raise ValueError("crop must be non-negative xy and positive width/height")
            object.__setattr__(self, "crop_xywh", crop)
        object.__setattr__(self, "scale_xy", scale)


@dataclass(frozen=True)
class SensorFrame:
    source: str
    stream: str
    epoch: str
    sequence: int
    capture_time: float
    receive_time: float
    clock_domain: str
    frame_id: str
    width: int
    height: int
    encoding: str
    rgb: ResourceReference
    depth: ResourceReference | None = None
    point_cloud: ResourceReference | None = None
    depth_unit: str | None = None
    depth_frame_id: str | None = None
    depth_validity: Validity = Validity.UNKNOWN
    intrinsics: CameraIntrinsics | None = None
    image_mapping: ImageMapping | None = None

    def __post_init__(self) -> None:
        for name in ("source", "stream", "epoch", "clock_domain", "frame_id", "encoding"):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")
        if self.sequence < 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("frame sequence and dimensions are invalid")
        capture, receive = float(self.capture_time), float(self.receive_time)
        if not isfinite(capture) or not isfinite(receive) or receive < capture:
            raise ValueError("frame times must be finite and receive_time cannot precede capture_time")
        if self.depth is not None and (not self.depth_unit or not self.depth_frame_id):
            raise ValueError("depth requires explicit unit and coordinate frame")
        if self.depth is None and self.depth_validity is Validity.VALID:
            raise ValueError("missing depth cannot be marked valid")
        object.__setattr__(self, "capture_time", capture)
        object.__setattr__(self, "receive_time", receive)
        object.__setattr__(self, "depth_validity", Validity(self.depth_validity))


@dataclass(frozen=True)
class Pose3D:
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    frame_id: str
    axis_convention: str
    evidence: EvidenceKind
    covariance: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        position = _finite_tuple(self.position_m, 3, "position")
        quaternion = _finite_tuple(self.orientation_xyzw, 4, "quaternion")
        norm = sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1e-6:
            raise ValueError("quaternion must be normalized xyzw")
        if not self.frame_id or not self.axis_convention:
            raise ValueError("pose frame and axis convention must be explicit")
        covariance = None if self.covariance is None else _finite_tuple(self.covariance, 36, "pose covariance")
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "orientation_xyzw", quaternion)
        object.__setattr__(self, "evidence", EvidenceKind(self.evidence))
        object.__setattr__(self, "covariance", covariance)


@dataclass(frozen=True)
class FaceEvidence:
    face_id: str
    evidence: EvidenceKind
    source_label: str
    corners_2d: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        if not self.face_id or not self.source_label:
            raise ValueError("face evidence identity must be non-empty")
        corners = tuple(_finite_tuple(point, 2, "2D corner") for point in self.corners_2d)
        object.__setattr__(self, "evidence", EvidenceKind(self.evidence))
        object.__setattr__(self, "corners_2d", corners)


@dataclass(frozen=True)
class CargoObservation:
    source_instance_id: str
    object_id: str | None
    track_id: str | None
    category: str
    bbox_xyxy: tuple[float, float, float, float]
    mask_reference: ResourceReference | None
    detection_score: float | None
    contour_score: float | None
    reprojection_score: float | None
    geometry_error_m: float | None
    pose: Pose3D | None
    full_dimensions_m: tuple[float, float, float] | None
    corners_3d_m: tuple[tuple[float, float, float], ...] | None
    axes_3d_rows: tuple[tuple[float, float, float], ...] | None
    pose_evidence: EvidenceKind | None
    size_evidence: EvidenceKind | None
    depth_evidence: EvidenceKind | None
    scale_evidence: EvidenceKind | None
    metric_scale_validity: Validity
    geometry_validity: Validity
    candidate_eligible: bool
    eligibility_reasons: tuple[str, ...]
    occluded: bool = False
    association_status: str = "UNASSOCIATED"
    face_evidence: tuple[FaceEvidence, ...] = ()
    raw_result: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        if not self.source_instance_id or not self.category:
            raise ValueError("cargo source identity and category must be non-empty")
        bbox = _finite_tuple(self.bbox_xyxy, 4, "bbox")
        if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
            raise ValueError("bbox must have positive area")
        for name in ("detection_score", "contour_score", "reprojection_score"):
            value = getattr(self, name)
            if value is not None and (not isfinite(float(value)) or not 0.0 <= float(value) <= 1.0):
                raise ValueError(f"{name} must be in [0, 1] when present")
        if self.geometry_error_m is not None and (not isfinite(float(self.geometry_error_m)) or self.geometry_error_m < 0.0):
            raise ValueError("geometry error must be finite and non-negative")
        dimensions = None if self.full_dimensions_m is None else _finite_tuple(self.full_dimensions_m, 3, "full dimensions")
        if dimensions is not None and any(value <= 0.0 for value in dimensions):
            raise ValueError("full dimensions must be positive")
        corners = None if self.corners_3d_m is None else tuple(_finite_tuple(point, 3, "3D corner") for point in self.corners_3d_m)
        axes = None if self.axes_3d_rows is None else tuple(_finite_tuple(axis, 3, "3D axis") for axis in self.axes_3d_rows)
        if axes is not None and len(axes) != 3:
            raise ValueError("3D axes must contain three row vectors")
        metric = Validity(self.metric_scale_validity)
        geometry = Validity(self.geometry_validity)
        if self.candidate_eligible and (metric is not Validity.VALID or geometry is not Validity.VALID or self.pose is None or dimensions is None):
            raise ValueError("candidate eligibility requires valid metric pose and full dimensions")
        if self.candidate_eligible and self.eligibility_reasons:
            raise ValueError("eligible cargo cannot carry rejection reasons")
        if not self.candidate_eligible and not self.eligibility_reasons:
            raise ValueError("ineligible cargo must explain why")
        object.__setattr__(self, "bbox_xyxy", bbox)
        object.__setattr__(self, "full_dimensions_m", dimensions)
        object.__setattr__(self, "corners_3d_m", corners)
        object.__setattr__(self, "axes_3d_rows", axes)
        object.__setattr__(self, "metric_scale_validity", metric)
        object.__setattr__(self, "geometry_validity", geometry)
        object.__setattr__(self, "eligibility_reasons", tuple(str(item) for item in self.eligibility_reasons))
        object.__setattr__(self, "face_evidence", tuple(self.face_evidence))
        object.__setattr__(self, "raw_result", deep_freeze(self.raw_result))


@dataclass(frozen=True)
class UnknownRegion:
    region_id: str
    frame_id: str
    reason: str
    bbox_xyxy: tuple[float, float, float, float] | None = None

    def __post_init__(self) -> None:
        if not self.region_id or not self.frame_id or not self.reason:
            raise ValueError("unknown region identity, frame, and reason are required")
        if self.bbox_xyxy is not None:
            object.__setattr__(self, "bbox_xyxy", _finite_tuple(self.bbox_xyxy, 4, "unknown bbox"))


@dataclass(frozen=True)
class PerceptionObservation:
    schema_version: str
    observation_id: str
    source_epoch: str
    source_sequence: int
    capture_time: float
    processed_time: float
    clock_domain: str
    provider: str
    upstream_commit: str
    model_identity: str
    config_identity: str
    status: ObservationStatus
    failure_code: str | None
    failure_message: str | None
    cargo: tuple[CargoObservation, ...]
    unknown_regions: tuple[UnknownRegion, ...]
    coverage: Mapping[str, Any]
    synthetic: bool = False

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported perception schema {self.schema_version!r}")
        for name in ("observation_id", "source_epoch", "clock_domain", "provider", "upstream_commit", "model_identity", "config_identity"):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")
        capture, processed = float(self.capture_time), float(self.processed_time)
        if self.source_sequence < 0 or not isfinite(capture) or not isfinite(processed) or processed < capture:
            raise ValueError("observation sequence/times are invalid")
        status = ObservationStatus(self.status)
        if status in (ObservationStatus.FAILED, ObservationStatus.BACKEND_UNAVAILABLE) and not self.failure_code:
            raise ValueError("failed observation requires a failure code")
        object.__setattr__(self, "capture_time", capture)
        object.__setattr__(self, "processed_time", processed)
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "cargo", tuple(self.cargo))
        object.__setattr__(self, "unknown_regions", tuple(self.unknown_regions))
        object.__setattr__(self, "coverage", deep_freeze(self.coverage))


@dataclass(frozen=True)
class PerceptionSceneUpdate:
    observation: PerceptionObservation
    accepted_obstacles: tuple[CargoObservation, ...]
    candidate_objects: tuple[CargoObservation, ...]
    unknown_regions: tuple[UnknownRegion, ...]
    geometry_fingerprint: str
    planning_admissible: bool
    blocking_reasons: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "accepted_obstacles", tuple(self.accepted_obstacles))
        object.__setattr__(self, "candidate_objects", tuple(self.candidate_objects))
        object.__setattr__(self, "unknown_regions", tuple(self.unknown_regions))
        object.__setattr__(self, "blocking_reasons", tuple(self.blocking_reasons))
        expected = canonical_fingerprint({
            "obstacles": self.accepted_obstacles,
            "unknown_regions": self.unknown_regions,
        })
        if expected != self.geometry_fingerprint:
            raise ValueError("scene update geometry fingerprint mismatch")
        if self.planning_admissible and self.blocking_reasons:
            raise ValueError("admissible scene update cannot have blocking reasons")


@dataclass(frozen=True, order=True)
class SceneRevision:
    sequence: int
    fingerprint: str
    source: str = field(default="perception", compare=False)
    parent_fingerprint: str | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        if self.sequence < 0 or not self.fingerprint or not self.source:
            raise ValueError("invalid scene revision")

    @classmethod
    def from_scene(cls, scene: Any, sequence: int, *, source: str = "perception", parent: SceneRevision | None = None) -> SceneRevision:
        return cls(sequence, scene_fingerprint(scene), source, None if parent is None else parent.fingerprint)

    def same_scene(self, other: SceneRevision) -> bool:
        return self.fingerprint == other.fingerprint


@dataclass(frozen=True)
class RobotStateRevision:
    sequence: int
    current_q: tuple[float, ...]
    robot_state: Mapping[str, Any] = field(default_factory=dict)
    sample_time: float | None = field(default=None, compare=False)
    clock_domain: str | None = field(default=None, compare=False)
    source: str | None = field(default=None, compare=False)
    fingerprint: str = field(init=False)

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("robot state revision sequence must be non-negative")
        q = _finite_tuple(self.current_q, name="robot current_q")
        state = deep_freeze(self.robot_state)
        if self.sample_time is not None and not isfinite(float(self.sample_time)):
            raise ValueError("robot state sample time must be finite")
        if (self.sample_time is None) != (self.clock_domain is None):
            raise ValueError("robot state sample time and clock domain must be supplied together")
        object.__setattr__(self, "current_q", q)
        object.__setattr__(self, "robot_state", state)
        object.__setattr__(self, "fingerprint", scene_fingerprint({"current_q": q, "robot_state": state}))


@dataclass(frozen=True)
class PlanningWorldSnapshot:
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
        # ``None`` is the explicit, known "no payload attached" state used by
        # the online consumer.  All other mechanism/config inputs must carry
        # source-owned state; the assembler never invents home/zero defaults.
        if any(value is None for value in (self.tool_attachment, self.base_state, self.conveyor_state, self.config_identity)):
            raise ValueError("complete mechanism, tool, and config state is required")
        scene = deep_freeze(self.scene_snapshot)
        geometry_fingerprint = (
            scene.get("geometry_fingerprint") if isinstance(scene, Mapping) else None
        )
        expected_scene_fingerprint = (
            str(geometry_fingerprint) if geometry_fingerprint else scene_fingerprint(scene)
        )
        if expected_scene_fingerprint != self.scene_revision.fingerprint:
            raise ValueError("actual scene geometry does not match scene revision fingerprint")
        frozen = {name: deep_freeze(getattr(self, name)) for name in ("tool_attachment", "payload_attachment", "base_state", "conveyor_state", "config_identity")}
        object.__setattr__(self, "scene_snapshot", scene)
        for name, value in frozen.items():
            object.__setattr__(self, name, value)
        object.__setattr__(self, "fingerprint", scene_fingerprint({
            "scene_revision": self.scene_revision.fingerprint,
            "scene_snapshot": scene,
            "robot_state_revision": {"sequence": self.robot_state_revision.sequence, "fingerprint": self.robot_state_revision.fingerprint},
            **frozen,
        }))

    @property
    def current_q(self) -> tuple[float, ...]:
        return self.robot_state_revision.current_q

    @property
    def robot_model_fingerprint(self) -> str:
        if isinstance(self.config_identity, Mapping) and self.config_identity.get("robot_model_fingerprint"):
            return str(self.config_identity["robot_model_fingerprint"])
        return scene_fingerprint({"robot_state": self.robot_state_revision.robot_state, "config": self.config_identity})

    @property
    def world_model_fingerprint(self) -> str:
        if isinstance(self.config_identity, Mapping) and self.config_identity.get("world_model_fingerprint"):
            return str(self.config_identity["world_model_fingerprint"])
        return scene_fingerprint({"scene": self.scene_snapshot, "base": self.base_state, "conveyor": self.conveyor_state, "config": self.config_identity})

    @property
    def mechanism_fingerprint(self) -> str:
        """Identity of mechanism/attachment state, separate from scene geometry."""
        return scene_fingerprint({
            "tool": self.tool_attachment,
            "payload": self.payload_attachment,
            "base": self.base_state,
            "conveyor": self.conveyor_state,
            "config": self.config_identity,
        })

    def planning_context_matches(self, other: PlanningWorldSnapshot) -> bool:
        return bool(
            isinstance(other, PlanningWorldSnapshot)
            and self.scene_revision.same_scene(other.scene_revision)
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

    def __post_init__(self) -> None:
        q = _finite_tuple(self.q, name="boundary q")
        qd = _finite_tuple(self.qd, len(q), "boundary qd")
        qdd = _finite_tuple(self.qdd, len(q), "boundary qdd")
        time = float(self.time_seconds)
        mode = BoundaryMode(self.boundary_mode)
        if not isfinite(time) or time < 0.0:
            raise ValueError("boundary time must be finite and non-negative")
        if mode is BoundaryMode.STOP_BOUNDARY and any(abs(value) > 1e-9 for value in qd + qdd):
            raise ValueError("STOP_BOUNDARY requires zero velocity and acceleration")
        object.__setattr__(self, "q", q)
        object.__setattr__(self, "qd", qd)
        object.__setattr__(self, "qdd", qdd)
        object.__setattr__(self, "time_seconds", time)
        object.__setattr__(self, "boundary_mode", mode)

    @classmethod
    def stopped(cls, q: Sequence[float], *, time_seconds: float = 0.0, predecessor_plan_id: str | None = None) -> MotionBoundaryState:
        position = tuple(float(value) for value in q)
        zeros = tuple(0.0 for _ in position)
        return cls(position, zeros, zeros, time_seconds, BoundaryMode.STOP_BOUNDARY, predecessor_plan_id)

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

    def matches(
        self,
        other: MotionBoundaryState,
        *,
        tolerance: float | None = None,
        q_atol: float = 1e-6,
        qd_atol: float = 1e-6,
        qdd_atol: float = 1e-6,
        time_atol: float = 1e-6,
        require_predecessor: bool = True,
    ) -> bool:
        """Compare boundaries using the online-planning branch's public API.

        ``tolerance`` is retained as a backwards-compatible shorthand for callers
        that used the original shared-contract method.
        """
        if tolerance is not None:
            q_atol = qd_atol = qdd_atol = time_atol = float(tolerance)
        if not isinstance(other, MotionBoundaryState) or self.mode is not other.mode:
            return False
        if require_predecessor and self.predecessor_plan_id != other.predecessor_plan_id:
            return False
        if any(
            len(left) != len(right)
            for left, right in ((self.q, other.q), (self.qd, other.qd), (self.qdd, other.qdd))
        ):
            return False
        return (
            abs(self.time_seconds - other.time_seconds) <= time_atol
            and all(abs(a - b) <= q_atol for a, b in zip(self.q, other.q))
            and all(abs(a - b) <= qd_atol for a, b in zip(self.qd, other.qd))
            and all(abs(a - b) <= qdd_atol for a, b in zip(self.qdd, other.qdd))
        )


@dataclass(frozen=True)
class BackendIdentity:
    backend_name: str
    backend_version: str
    adapter_version: str
    robot_model_fingerprint: str
    world_model_fingerprint: str

    def __post_init__(self) -> None:
        if any(not getattr(self, name) for name in self.__dataclass_fields__):
            raise ValueError("backend identity fields must be non-empty")


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
        return boundary.mode is not BoundaryMode.CONTINUOUS_BOUNDARY or PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY in self.supported_artifact_kinds

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
class PlanningCandidate:
    target_id: str
    candidate_id: str
    metadata: Mapping[str, Any] = field(default_factory=dict, compare=False)

    def __post_init__(self) -> None:
        if not self.target_id or not self.candidate_id:
            raise ValueError("planning target and candidate ids must be non-empty")
        object.__setattr__(self, "metadata", deep_freeze(self.metadata))


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
            raise ValueError("planning request identity is required")
        if not isinstance(self.world_snapshot, PlanningWorldSnapshot):
            raise TypeError("planning request must bind a PlanningWorldSnapshot")
        candidates = tuple(self.candidates)
        if not candidates:
            raise ValueError("planning request candidates are required")
        if self.horizon_index < 0:
            raise ValueError("planning horizon index must be non-negative")
        boundary = self.motion_boundary
        if boundary is None:
            boundary = MotionBoundaryState.stopped(self.world_snapshot.current_q)
        if not isinstance(boundary, MotionBoundaryState):
            raise TypeError("planning request motion_boundary must be a MotionBoundaryState")
        if boundary.current_q != self.world_snapshot.current_q:
            raise ValueError("motion boundary must match bound world snapshot")
        object.__setattr__(self, "candidates", candidates)
        object.__setattr__(self, "motion_boundary", boundary)
        object.__setattr__(self, "replan_reason", ReplanReason(self.replan_reason))
        object.__setattr__(self, "metadata", deep_freeze(self.metadata))

    @property
    def scene_revision(self) -> SceneRevision:
        return self.world_snapshot.scene_revision

    @property
    def start_state(self) -> tuple[float, ...]:
        return self.motion_boundary.current_q  # type: ignore[union-attr]


@dataclass(frozen=True)
class TimedJointPoint:
    positions: tuple[float, ...]
    time_from_start: float
    velocities: tuple[float, ...] | None = None
    accelerations: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        positions = _finite_tuple(self.positions, name="joint positions")
        time = float(self.time_from_start)
        if not isfinite(time) or time < 0.0:
            raise ValueError("trajectory time must be finite and non-negative")
        velocities = None if self.velocities is None else _finite_tuple(self.velocities, len(positions), "joint velocities")
        accelerations = None if self.accelerations is None else _finite_tuple(self.accelerations, len(positions), "joint accelerations")
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "time_from_start", time)
        object.__setattr__(self, "velocities", velocities)
        object.__setattr__(self, "accelerations", accelerations)


@dataclass(frozen=True)
class TimedJointTrajectory:
    joint_names: tuple[str, ...]
    points: tuple[TimedJointPoint, ...]
    artifact_kind: PlanArtifactKind
    source: str
    robot_model_fingerprint: str
    config_identity: str

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.joint_names)
        points = tuple(self.points)
        if not names or len(set(names)) != len(names) or not points:
            raise ValueError("trajectory requires unique joint names and points")
        if PlanArtifactKind(self.artifact_kind) is not PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY:
            raise ValueError("timed trajectory artifact kind must be TIME_PARAMETERIZED_TRAJECTORY")
        if any(len(point.positions) != len(names) for point in points):
            raise ValueError("trajectory point DOF must match joint names")
        times = tuple(point.time_from_start for point in points)
        if any(later <= earlier for earlier, later in zip(times, times[1:])):
            raise ValueError("trajectory times must be strictly increasing")
        if not self.source or not self.robot_model_fingerprint or not self.config_identity:
            raise ValueError("trajectory provenance must be complete")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "points", points)
        object.__setattr__(self, "artifact_kind", PlanArtifactKind(self.artifact_kind))


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
        object.__setattr__(self, "metadata", deep_freeze(self.metadata))


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
    timed_trajectory: TimedJointTrajectory | None = None

    def __post_init__(self) -> None:
        status = PlanStatus(self.status)
        artifact = PlanArtifactKind(self.artifact_kind)
        trajectory = tuple(_finite_tuple(point, name="trajectory point") for point in self.trajectory)
        if self.latency_seconds is not None and (not isfinite(self.latency_seconds) or self.latency_seconds < 0.0):
            raise ValueError("planning latency must be finite and non-negative")
        if status is PlanStatus.SUCCESS:
            if not trajectory or not self.target_id or not self.candidate_id:
                raise ValueError("successful result requires trajectory and candidate identity")
            dof = len(trajectory[0])
            if any(len(point) != dof for point in trajectory):
                raise ValueError("trajectory DOF must be consistent")
            start = self.expected_start_boundary
            end = self.expected_end_boundary
            if start is None and end is None and artifact is PlanArtifactKind.GEOMETRIC_PATH:
                start, end = MotionBoundaryState.stopped(trajectory[0]), MotionBoundaryState.stopped(trajectory[-1])
            if start is None or end is None:
                raise ValueError("successful trajectory requires both motion boundaries")
            if start.q != trajectory[0] or end.q != trajectory[-1]:
                raise ValueError("trajectory boundaries must match endpoints")
            if artifact is PlanArtifactKind.GEOMETRIC_PATH and (
                start.boundary_mode is not BoundaryMode.STOP_BOUNDARY
                or end.boundary_mode is not BoundaryMode.STOP_BOUNDARY
            ):
                raise ValueError("GEOMETRIC_PATH can only declare STOP_BOUNDARY endpoints")
            if (
                start.boundary_mode is BoundaryMode.CONTINUOUS_BOUNDARY
                or end.boundary_mode is BoundaryMode.CONTINUOUS_BOUNDARY
            ) and artifact is not PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY:
                raise ValueError("CONTINUOUS_BOUNDARY requires a time-parameterized trajectory")
            if end.time_seconds < start.time_seconds:
                raise ValueError("proposal end boundary time cannot precede start boundary time")
            if self.timed_trajectory is not None:
                timed_positions = tuple(point.positions for point in self.timed_trajectory.points)
                if timed_positions != trajectory or artifact is not PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY:
                    raise ValueError("geometric and timed trajectory representations must agree")
            object.__setattr__(self, "expected_start_boundary", start)
            object.__setattr__(self, "expected_end_boundary", end)
        elif trajectory or self.timed_trajectory is not None:
            raise ValueError("failed result cannot carry a trajectory")
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
                status in (PlanStatus.NOT_EVALUATED, PlanStatus.TIMEOUT),
                scope,
                status.value,
                self.message,
                allow_path_fallback=True,
                allow_backend_fallback=True,
            )
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "trajectory", trajectory)
        object.__setattr__(self, "artifact_kind", artifact)
        object.__setattr__(self, "failure", failure)
        object.__setattr__(self, "metadata", deep_freeze(self.metadata))

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
    def succeeded(cls, candidate: PlanningCandidate, trajectory: Sequence[Sequence[float]], *, latency_seconds: float | None = None, message: str = "", metadata: Mapping[str, Any] | None = None, artifact_kind: PlanArtifactKind = PlanArtifactKind.GEOMETRIC_PATH, expected_start_boundary: MotionBoundaryState | None = None, expected_end_boundary: MotionBoundaryState | None = None, timed_trajectory: TimedJointTrajectory | None = None) -> PlanningResult:
        return cls(PlanStatus.SUCCESS, tuple(tuple(point) for point in trajectory), candidate.target_id, candidate.candidate_id, message, latency_seconds, {} if metadata is None else metadata, artifact_kind, None, None, expected_start_boundary, expected_end_boundary, timed_trajectory)

    @classmethod
    def failed(cls, status: PlanStatus, candidate: PlanningCandidate, *, latency_seconds: float | None = None, message: str = "", metadata: Mapping[str, Any] | None = None, failure: FailureDetails | None = None) -> PlanningResult:
        if PlanStatus(status) is PlanStatus.SUCCESS:
            raise ValueError("use PlanningResult.succeeded for success")
        return cls(PlanStatus(status), (), candidate.target_id, candidate.candidate_id, message, latency_seconds, {} if metadata is None else metadata, PlanArtifactKind.GEOMETRIC_PATH, None, failure)

    @classmethod
    def operational_failure(cls, outcome: OperationalOutcome, candidate: PlanningCandidate, *, failure: FailureDetails, latency_seconds: float | None = None, message: str = "", metadata: Mapping[str, Any] | None = None) -> PlanningResult:
        return cls(PlanStatus.NOT_EVALUATED, (), candidate.target_id, candidate.candidate_id, message or failure.native_message, latency_seconds, {} if metadata is None else metadata, PlanArtifactKind.GEOMETRIC_PATH, OperationalOutcome(outcome), failure)


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
        object.__setattr__(self, "robot_model_fingerprint", self.robot_model_fingerprint or self.world_snapshot.robot_model_fingerprint)
        object.__setattr__(self, "world_model_fingerprint", self.world_model_fingerprint or self.world_snapshot.world_model_fingerprint)

    @property
    def valid(self) -> bool:
        return ValidationStatus(self.status) is ValidationStatus.VALID

    @property
    def revision(self) -> SceneRevision:
        return self.world_snapshot.scene_revision


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

    @property
    def executable(self) -> bool:
        return self.result.success and not self.speculative and self.invalidated_by is None and self.validated_snapshot is not None and self.validated_by is not None and self.validation_timestamp_seconds is not None and self.validation_generation is not None

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

    planned_scene_revision = planned_revision
    validated_scene_revision = validated_revision

    def invalidate(self, reason: ReplanReason, message: str = "") -> PlanEnvelope:
        return replace(self, invalidated_by=reason, validation_message=message or reason.value)

    def revalidated(self, validation: PlanValidationResult, *, timestamp_seconds: float, generation: int) -> PlanEnvelope:
        if not validation.valid:
            raise ValueError("cannot mark a failed validation as valid")
        return replace(self, validated_snapshot=validation.world_snapshot, invalidated_by=None, validation_message=validation.message, validated_by=f"{validation.validator_name}@{validation.validator_version}", validation_timestamp_seconds=float(timestamp_seconds), validation_generation=int(generation))


@runtime_checkable
class CandidateProvider(Protocol):
    def candidates(self, snapshot: PlanningWorldSnapshot) -> tuple[PlanningCandidate, ...]: ...


class PlannerBackend(ABC):
    """Provider base copied from the online branch, without a concrete planner."""

    def __init__(self, seed: int = 0, *, identity: BackendIdentity | None = None, capabilities: BackendCapabilities | None = None, compute_device: str | None = None, compute_category: str | None = None) -> None:
        self.seed = int(seed)
        self.identity = identity or BackendIdentity("legacy-backend", "unspecified", "1", "*", "*")
        self.capabilities = capabilities or BackendCapabilities(frozenset(PlanningPath), frozenset({BoundaryMode.STOP_BOUNDARY}), frozenset({PlanArtifactKind.GEOMETRIC_PATH}), supports_warm_start=True, supports_deterministic_seed=True, supports_attached_object=True, supports_revalidation=True)
        self.compute_device = compute_device
        self.compute_category = compute_category

    @property
    def health(self) -> BackendHealth:
        return BackendHealth.READY

    @property
    def provenance(self) -> BackendProvenance:
        return BackendProvenance(self.identity, self.seed if self.capabilities.supports_deterministic_seed else None, self.compute_device, self.compute_category)

    def set_seed(self, seed: int) -> None:
        self.seed = int(seed)

    def initialize(self) -> None: ...
    def prewarm(self) -> None: ...
    def logical_cancel(self, request_id: str) -> None: ...
    def shutdown(self) -> None: ...

    @abstractmethod
    def plan(self, request: PlanningRequest, candidate: PlanningCandidate, planning_path: PlanningPath) -> PlanningResult: ...

    def validate_plan(self, envelope: PlanEnvelope, snapshot: PlanningWorldSnapshot) -> PlanValidationResult:
        boundary = envelope.request.motion_boundary
        assert boundary is not None
        valid = envelope.planned_snapshot.planning_context_matches(snapshot)
        return PlanValidationResult(ValidationStatus.VALID if valid else ValidationStatus.INVALID, snapshot, boundary, "planning world identity matches" if valid else "planning world identity changed", self.identity.backend_name, self.identity.adapter_version, reused=valid and snapshot != envelope.planned_snapshot)


class PlanValidator(ABC):
    def __init__(self, name: str, version: str) -> None:
        if not name or not version:
            raise ValueError("validator name and version must be non-empty")
        self.name, self.version = name, version

    @abstractmethod
    def validate(self, plan_envelope: PlanEnvelope, current_world_snapshot: PlanningWorldSnapshot, current_motion_boundary: MotionBoundaryState) -> PlanValidationResult: ...


@dataclass(frozen=True)
class ExecutionCommand:
    command_id: str
    plan_id: str
    request_id: str
    session_id: str
    epoch: str
    planning_generation: int
    predecessor_plan_id: str | None
    world_fingerprint: str
    robot_model_fingerprint: str
    config_identity: str
    validation_reference: str
    validation_generation: int
    trajectory: TimedJointTrajectory

    def __post_init__(self) -> None:
        for name in ("command_id", "plan_id", "request_id", "session_id", "epoch", "world_fingerprint", "robot_model_fingerprint", "config_identity", "validation_reference"):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")
        if self.planning_generation < 0 or self.validation_generation < 0:
            raise ValueError("command generations must be non-negative")
        if self.trajectory.robot_model_fingerprint != self.robot_model_fingerprint:
            raise ValueError("trajectory robot identity does not match command")

    @property
    def trajectory_fingerprint(self) -> str:
        return canonical_fingerprint(self.trajectory)


@dataclass(frozen=True)
class ExecutionGrant:
    """Authoritative, independently registered permission for one command."""

    grant_id: str
    command_id: str
    plan_id: str
    request_id: str
    session_id: str
    epoch: str
    planning_generation: int
    predecessor_plan_id: str | None
    world_fingerprint: str
    robot_model_fingerprint: str
    config_identity: str
    validation_reference: str
    validation_generation: int
    trajectory_fingerprint: str
    expires_at: float
    clock_domain: str
    mock_only: bool = False

    def __post_init__(self) -> None:
        for name in (
            "grant_id", "command_id", "plan_id", "request_id", "session_id",
            "epoch", "world_fingerprint", "robot_model_fingerprint",
            "config_identity", "validation_reference", "trajectory_fingerprint",
            "clock_domain",
        ):
            if not getattr(self, name):
                raise ValueError(f"{name} must be non-empty")
        if self.planning_generation < 0 or self.validation_generation < 0:
            raise ValueError("grant generations must be non-negative")
        if not isfinite(float(self.expires_at)):
            raise ValueError("grant expiry must be finite")

    def matches(self, command: ExecutionCommand) -> bool:
        fields = (
            "command_id", "plan_id", "request_id", "session_id", "epoch",
            "planning_generation", "predecessor_plan_id", "world_fingerprint",
            "robot_model_fingerprint", "config_identity", "validation_reference",
            "validation_generation",
        )
        return all(getattr(self, name) == getattr(command, name) for name in fields) and self.trajectory_fingerprint == command.trajectory_fingerprint


@dataclass(frozen=True)
class ControllerStopFact:
    """Controller feedback fact; it is not itself permission to replan."""

    controller_id: str
    controller_epoch: str
    goal_id: str
    sequence: int
    cancel_accepted_time: float
    sample_time: float
    clock_domain: str
    joint_names: tuple[str, ...]
    actual_positions: tuple[float, ...]
    actual_velocities: tuple[float, ...]
    evidence_reference: str

    def __post_init__(self) -> None:
        if not all((self.controller_id, self.controller_epoch, self.goal_id, self.clock_domain, self.evidence_reference)):
            raise ValueError("controller stop identity, clock, and evidence are required")
        if self.sequence < 0 or not isfinite(float(self.cancel_accepted_time)) or not isfinite(float(self.sample_time)) or self.sample_time < self.cancel_accepted_time:
            raise ValueError("controller stop sequence/time is invalid")
        names = tuple(str(name) for name in self.joint_names)
        if not names or len(names) != len(set(names)):
            raise ValueError("controller stop joint names must be non-empty and unique")
        positions = _finite_tuple(self.actual_positions, len(names), "stop positions")
        velocities = _finite_tuple(self.actual_velocities, len(names), "stop velocities")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "actual_positions", positions)
        object.__setattr__(self, "actual_velocities", velocities)


@dataclass(frozen=True)
class ExecutionEvent:
    command_id: str
    plan_id: str
    epoch: str
    planning_generation: int
    kind: ExecutionEventKind
    event_time: float
    clock_domain: str
    actual_positions: tuple[float, ...] | None = None
    actual_velocities: tuple[float, ...] | None = None
    message: str = ""

    def __post_init__(self) -> None:
        if not self.command_id or not self.plan_id or not self.epoch or not self.clock_domain:
            raise ValueError("execution event identity and clock are required")
        if self.planning_generation < 0 or not isfinite(float(self.event_time)):
            raise ValueError("execution event generation/time is invalid")
        positions = None if self.actual_positions is None else _finite_tuple(self.actual_positions, name="actual positions")
        velocities = None if self.actual_velocities is None else _finite_tuple(self.actual_velocities, len(positions) if positions else None, "actual velocities")
        object.__setattr__(self, "kind", ExecutionEventKind(self.kind))
        object.__setattr__(self, "actual_positions", positions)
        object.__setattr__(self, "actual_velocities", velocities)


@dataclass(frozen=True)
class StopAcknowledgement:
    command_id: str
    plan_id: str
    epoch: str
    planning_generation: int
    stopped_time: float
    clock_domain: str
    actual_positions: tuple[float, ...]
    actual_velocities: tuple[float, ...]
    criterion: str
    evidence_reference: str
    controller_id: str = "unspecified"
    controller_epoch: str = "unspecified"
    goal_id: str = "unspecified"
    stop_sequence: int = 0

    def __post_init__(self) -> None:
        positions = _finite_tuple(self.actual_positions, name="stop positions")
        velocities = _finite_tuple(self.actual_velocities, len(positions), "stop velocities")
        if any(abs(value) > 1e-3 for value in velocities):
            raise ValueError("stop acknowledgement requires measured near-zero velocity")
        if not self.criterion or not self.evidence_reference:
            raise ValueError("stop confirmation criterion and evidence are required")
        if not self.controller_id or not self.controller_epoch or not self.goal_id or self.stop_sequence < 0:
            raise ValueError("stop confirmation controller/goal identity is invalid")
        object.__setattr__(self, "actual_positions", positions)
        object.__setattr__(self, "actual_velocities", velocities)


__all__ = [name for name in globals() if not name.startswith("_")]
