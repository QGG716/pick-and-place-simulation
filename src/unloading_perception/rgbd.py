"""Registered RGB-D capture and source-neutral metric geometry contracts.

This module is intentionally independent of ROS, Isaac Sim and neural model
runtimes.  It validates synchronization/calibration before any 6D geometry is
allowed to run, and it keeps registered metric depth distinct from monocular
pseudo-depth provenance.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import json
from math import isfinite
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable

import numpy as np

from unloading_contracts import CameraIntrinsics, EvidenceKind, Pose3D, Validity

from .geometry import pose_from_axes_rows, transform_pose, validate_transform_parent_child


class IlluminationState(str, Enum):
    LIGHT_OFF = "LIGHT_OFF"
    LIGHT_ON_NOMINAL = "LIGHT_ON_NOMINAL"
    LIGHT_LOW = "LIGHT_LOW"
    LIGHT_HIGH = "LIGHT_HIGH"


class MetricPointMapSource(str, Enum):
    ISAAC_IDEAL_REGISTERED_DEPTH = "ISAAC_IDEAL_REGISTERED_DEPTH"
    REGISTERED_DEPTH_SENSOR = "REGISTERED_DEPTH_SENSOR"
    MOGE_MONOCULAR_ESTIMATE = "MOGE_MONOCULAR_ESTIMATE"


@dataclass(frozen=True)
class CaptureRequest:
    request_id: str
    module_id: str
    requested_time: float
    clock_domain: str
    illumination_state: IlluminationState
    trigger_group: str = "module_0_main"

    def __post_init__(self) -> None:
        if not self.request_id or not self.module_id or not self.clock_domain or not self.trigger_group:
            raise ValueError("capture request identities are required")
        if not isfinite(float(self.requested_time)):
            raise ValueError("requested capture time must be finite")
        object.__setattr__(self, "illumination_state", IlluminationState(self.illumination_state))


@dataclass(frozen=True)
class CaptureMetadata:
    capture_id: str
    sensor_epoch: str
    frame_sequence: int
    requested_time: float
    capture_start: float
    capture_center_time: float
    capture_end: float
    clock_domain: str
    rgb_frame_id: str
    depth_frame_id: str
    calibration_identity: str
    illumination_state: IlluminationState
    j1_state_identity: str
    q1_at_capture_rad: float
    T_W_C_at_capture: tuple[tuple[float, ...], ...]
    sync_status: str = "SYNCHRONIZED"

    def __post_init__(self) -> None:
        for name in (
            "capture_id", "sensor_epoch", "clock_domain", "rgb_frame_id", "depth_frame_id",
            "calibration_identity", "j1_state_identity", "sync_status",
        ):
            if not getattr(self, name):
                raise ValueError(f"{name} is required")
        times = tuple(float(getattr(self, name)) for name in (
            "requested_time", "capture_start", "capture_center_time", "capture_end"
        ))
        if self.frame_sequence < 0 or not all(isfinite(value) for value in times):
            raise ValueError("capture sequence and times must be valid")
        if not times[1] <= times[2] <= times[3]:
            raise ValueError("capture start, center and end times must be ordered")
        if not isfinite(float(self.q1_at_capture_rad)):
            raise ValueError("captured J1 value must be finite")
        transform = validate_transform_parent_child(self.T_W_C_at_capture)
        object.__setattr__(self, "T_W_C_at_capture", transform)
        object.__setattr__(self, "illumination_state", IlluminationState(self.illumination_state))
        for name, value in zip(("requested_time", "capture_start", "capture_center_time", "capture_end"), times):
            object.__setattr__(self, name, value)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": "rgbd_capture_metadata_v1",
            "capture_id": self.capture_id,
            "sensor_epoch": self.sensor_epoch,
            "frame_sequence": self.frame_sequence,
            "requested_time": self.requested_time,
            "capture_start": self.capture_start,
            "capture_center_time": self.capture_center_time,
            "capture_end": self.capture_end,
            "clock_domain": self.clock_domain,
            "rgb_frame_id": self.rgb_frame_id,
            "depth_frame_id": self.depth_frame_id,
            "calibration_identity": self.calibration_identity,
            "illumination_state": self.illumination_state.value,
            "j1_state_identity": self.j1_state_identity,
            "q1_at_capture_rad": self.q1_at_capture_rad,
            "T_W_C_at_capture": [list(row) for row in self.T_W_C_at_capture],
            "sync_status": self.sync_status,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "CaptureMetadata":
        if value.get("schema_version") != "rgbd_capture_metadata_v1":
            raise ValueError("unsupported RGB-D capture metadata schema")
        return cls(
            str(value["capture_id"]), str(value["sensor_epoch"]), int(value["frame_sequence"]),
            float(value["requested_time"]), float(value["capture_start"]),
            float(value["capture_center_time"]), float(value["capture_end"]),
            str(value["clock_domain"]), str(value["rgb_frame_id"]), str(value["depth_frame_id"]),
            str(value["calibration_identity"]), IlluminationState(value["illumination_state"]),
            str(value["j1_state_identity"]), float(value["q1_at_capture_rad"]),
            tuple(tuple(float(item) for item in row) for row in value["T_W_C_at_capture"]),
            str(value.get("sync_status", "SYNCHRONIZED")),
        )


class CaptureManager:
    """Deterministic capture identity allocator for one or more trigger groups."""

    def __init__(self, sensor_epoch: str, *, first_sequence: int = 0, multi_module_stagger_s: float = 0.0) -> None:
        if not sensor_epoch or first_sequence < 0 or multi_module_stagger_s < 0.0:
            raise ValueError("capture manager configuration is invalid")
        self.sensor_epoch = sensor_epoch
        self._next_sequence = int(first_sequence)
        self.multi_module_stagger_s = float(multi_module_stagger_s)

    def bind(
        self,
        request: CaptureRequest,
        *,
        capture_start: float,
        capture_center_time: float,
        capture_end: float,
        rgb_frame_id: str,
        depth_frame_id: str,
        calibration_identity: str,
        j1_state_identity: str,
        q1_at_capture_rad: float,
        T_W_C_at_capture: Sequence[Sequence[float]],
    ) -> CaptureMetadata:
        sequence = self._next_sequence
        self._next_sequence += 1
        return CaptureMetadata(
            capture_id=f"{self.sensor_epoch}:{request.trigger_group}:{sequence}",
            sensor_epoch=self.sensor_epoch,
            frame_sequence=sequence,
            requested_time=request.requested_time,
            capture_start=capture_start,
            capture_center_time=capture_center_time,
            capture_end=capture_end,
            clock_domain=request.clock_domain,
            rgb_frame_id=rgb_frame_id,
            depth_frame_id=depth_frame_id,
            calibration_identity=calibration_identity,
            illumination_state=request.illumination_state,
            j1_state_identity=j1_state_identity,
            q1_at_capture_rad=q1_at_capture_rad,
            T_W_C_at_capture=tuple(tuple(float(value) for value in row) for row in T_W_C_at_capture),
        )


@dataclass(frozen=True)
class RegisteredRgbdFrame:
    metadata: CaptureMetadata
    rgb: np.ndarray = field(compare=False, repr=False)
    depth_optical_z_m: np.ndarray = field(compare=False, repr=False)
    valid_depth_mask: np.ndarray = field(compare=False, repr=False)
    K: tuple[float, ...]
    T_rgb_depth: tuple[tuple[float, ...], ...]
    registration_mode: str

    def __post_init__(self) -> None:
        rgb = np.asarray(self.rgb)
        depth = np.asarray(self.depth_optical_z_m, dtype=np.float32)
        valid = np.asarray(self.valid_depth_mask, dtype=bool)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2] or valid.shape != depth.shape:
            raise ValueError("registered RGB, depth and validity shapes differ")
        if len(self.K) != 9 or not all(isfinite(float(value)) for value in self.K):
            raise ValueError("registered RGB-D requires a finite 3x3 K")
        if self.K[0] <= 0.0 or self.K[4] <= 0.0 or self.K[8] != 1.0:
            raise ValueError("registered RGB-D K is invalid")
        if np.any(valid & (~np.isfinite(depth) | (depth <= 0.0))):
            raise ValueError("valid depth pixels must contain positive finite optical Z metres")
        if self.registration_mode not in {"SIMULATION_IDEAL_REGISTERED_DEPTH", "CALIBRATED_REGISTERED_DEPTH"}:
            raise ValueError("RGB-D registration mode is not accepted")
        transform = validate_transform_parent_child(self.T_rgb_depth)
        rgb = np.ascontiguousarray(rgb)
        depth = np.ascontiguousarray(depth)
        valid = np.ascontiguousarray(valid)
        rgb.setflags(write=False)
        depth.setflags(write=False)
        valid.setflags(write=False)
        object.__setattr__(self, "rgb", rgb)
        object.__setattr__(self, "depth_optical_z_m", depth)
        object.__setattr__(self, "valid_depth_mask", valid)
        object.__setattr__(self, "K", tuple(float(value) for value in self.K))
        object.__setattr__(self, "T_rgb_depth", transform)


def register_rgbd(
    *,
    metadata: CaptureMetadata,
    rgb: np.ndarray,
    depth_optical_z_m: np.ndarray,
    rgb_K: Sequence[float],
    depth_K: Sequence[float],
    T_rgb_depth: Sequence[Sequence[float]],
    rgb_calibration_identity: str,
    depth_calibration_identity: str,
    registration_mode: str,
) -> RegisteredRgbdFrame:
    """Fail closed unless RGB and depth are one calibrated capture bundle."""

    if metadata.sync_status != "SYNCHRONIZED":
        raise ValueError("RGB-D capture is not synchronized")
    if not rgb_calibration_identity or rgb_calibration_identity != depth_calibration_identity:
        raise ValueError("RGB and depth calibration identities differ")
    if metadata.calibration_identity != rgb_calibration_identity:
        raise ValueError("capture calibration identity does not match RGB-D data")
    rgb_array = np.asarray(rgb)
    depth_array = np.asarray(depth_optical_z_m)
    if rgb_array.shape[:2] != depth_array.shape:
        raise ValueError("RGB and registered depth resolutions differ")
    rgb_values, depth_values = tuple(float(value) for value in rgb_K), tuple(float(value) for value in depth_K)
    if len(rgb_values) != 9 or len(depth_values) != 9 or not np.allclose(rgb_values, depth_values, atol=1e-9, rtol=0.0):
        raise ValueError("registered depth intrinsics do not match RGB K")
    transform = validate_transform_parent_child(T_rgb_depth)
    if registration_mode == "SIMULATION_IDEAL_REGISTERED_DEPTH" and not np.allclose(transform, np.eye(4), atol=1e-12, rtol=0.0):
        raise ValueError("ideal registered depth must be coaxial with RGB")
    valid = np.isfinite(depth_array) & (depth_array > 0.0)
    if not np.any(valid):
        raise ValueError("registered depth contains no valid optical-Z samples")
    return RegisteredRgbdFrame(metadata, rgb_array, depth_array, valid, rgb_values, transform, registration_mode)


@dataclass(frozen=True)
class PointCloudFilterConfig:
    boundary_erosion_px: int = 1
    depth_percentile_low: float = 1.0
    depth_percentile_high: float = 99.0
    discontinuity_mad_scale: float = 6.0
    discontinuity_floor_m: float = 0.02
    select_largest_component: bool = True
    minimum_points: int = 50

    def __post_init__(self) -> None:
        if self.boundary_erosion_px < 0 or self.minimum_points <= 0:
            raise ValueError("point filtering counts are invalid")
        if not 0.0 <= self.depth_percentile_low < self.depth_percentile_high <= 100.0:
            raise ValueError("depth percentiles are invalid")
        if self.discontinuity_mad_scale <= 0.0 or self.discontinuity_floor_m <= 0.0:
            raise ValueError("depth discontinuity thresholds must be positive")


@dataclass(frozen=True)
class MetricPointMap:
    schema_version: str
    source: MetricPointMapSource
    points_camera_xyz_m: np.ndarray = field(compare=False, repr=False)
    depth_optical_z_m: np.ndarray = field(compare=False, repr=False)
    valid_mask: np.ndarray = field(compare=False, repr=False)
    intrinsics_K: tuple[float, ...]
    camera_frame: str
    source_size: tuple[int, int]
    capture_id: str
    capture_time: float
    calibration_identity: str
    depth_identity: str
    filter_evidence: Mapping[str, Any]

    def __post_init__(self) -> None:
        if self.schema_version != "metric_point_map_v1":
            raise ValueError("unsupported metric point-map schema")
        source = MetricPointMapSource(self.source)
        points = np.asarray(self.points_camera_xyz_m, dtype=np.float32)
        depth = np.asarray(self.depth_optical_z_m, dtype=np.float32)
        valid = np.asarray(self.valid_mask, dtype=bool)
        if points.ndim != 3 or points.shape[2] != 3 or depth.shape != points.shape[:2] or valid.shape != depth.shape:
            raise ValueError("metric point-map arrays have inconsistent shapes")
        if self.source_size != (points.shape[1], points.shape[0]):
            raise ValueError("metric point-map source size does not match arrays")
        if np.any(valid & (~np.isfinite(points).all(axis=2) | ~np.isfinite(depth) | (depth <= 0.0))):
            raise ValueError("valid metric point-map samples are invalid")
        if len(self.intrinsics_K) != 9 or not self.camera_frame or not self.capture_id or not self.calibration_identity or not self.depth_identity:
            raise ValueError("metric point-map identity and intrinsics are required")
        if not isfinite(float(self.capture_time)):
            raise ValueError("metric point-map capture time must be finite")
        if source is MetricPointMapSource.MOGE_MONOCULAR_ESTIMATE and self.filter_evidence.get("metric_scale_validity") == "VALID":
            raise ValueError("monocular MoGe point maps cannot claim verified metric scale")
        points = np.ascontiguousarray(points)
        depth = np.ascontiguousarray(depth)
        valid = np.ascontiguousarray(valid)
        points.setflags(write=False)
        depth.setflags(write=False)
        valid.setflags(write=False)
        object.__setattr__(self, "source", source)
        object.__setattr__(self, "points_camera_xyz_m", points)
        object.__setattr__(self, "depth_optical_z_m", depth)
        object.__setattr__(self, "valid_mask", valid)
        object.__setattr__(self, "intrinsics_K", tuple(float(value) for value in self.intrinsics_K))
        object.__setattr__(self, "filter_evidence", dict(self.filter_evidence))

    def write_npz(self, path: str | Path) -> None:
        """Write both the formal K and normalized upstream compatibility K."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        width, height = self.source_size
        K = np.asarray(self.intrinsics_K, dtype=np.float64).reshape(3, 3)
        normalized = K.copy()
        normalized[0] /= width
        normalized[1] /= height
        metadata = {
            "schema_version": self.schema_version,
            "source": self.source.value,
            "camera_frame": self.camera_frame,
            "source_size": self.source_size,
            "capture_id": self.capture_id,
            "capture_time": self.capture_time,
            "calibration_identity": self.calibration_identity,
            "depth_identity": self.depth_identity,
            "filter_evidence": dict(self.filter_evidence),
        }
        np.savez_compressed(
            destination,
            points=self.points_camera_xyz_m,
            valid_mask=self.valid_mask,
            intrinsics=normalized,
            K=K,
            depth_optical_z_m=self.depth_optical_z_m,
            metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
        )


def _erode_mask(mask: np.ndarray, iterations: int) -> np.ndarray:
    result = np.asarray(mask, dtype=bool).copy()
    for _ in range(iterations):
        padded = np.pad(result, 1, mode="constant", constant_values=False)
        result = (
            padded[1:-1, 1:-1]
            & padded[:-2, 1:-1]
            & padded[2:, 1:-1]
            & padded[1:-1, :-2]
            & padded[1:-1, 2:]
        )
    return result


def _largest_component(mask: np.ndarray) -> np.ndarray:
    try:
        import cv2  # type: ignore

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=4)
        if count <= 1:
            return mask
        label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        return labels == label
    except ImportError:
        pass
    result = np.zeros_like(mask, dtype=bool)
    seen = np.zeros_like(mask, dtype=bool)
    best: list[tuple[int, int]] = []
    height, width = mask.shape
    for start_y, start_x in zip(*np.where(mask & ~seen)):
        queue = deque([(int(start_y), int(start_x))])
        seen[start_y, start_x] = True
        component: list[tuple[int, int]] = []
        while queue:
            y, x = queue.popleft()
            component.append((y, x))
            for next_y, next_x in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= next_y < height and 0 <= next_x < width and mask[next_y, next_x] and not seen[next_y, next_x]:
                    seen[next_y, next_x] = True
                    queue.append((next_y, next_x))
        if len(component) > len(best):
            best = component
    if best:
        ys, xs = zip(*best)
        result[np.asarray(ys), np.asarray(xs)] = True
    return result


def masked_metric_pointmap(
    frame: RegisteredRgbdFrame,
    instance_mask: np.ndarray,
    *,
    depth_identity: str,
    source: MetricPointMapSource = MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH,
    config: PointCloudFilterConfig = PointCloudFilterConfig(),
) -> MetricPointMap:
    """Lift a registered instance mask into optical-camera metric XYZ."""

    mask = np.asarray(instance_mask, dtype=bool)
    if mask.shape != frame.depth_optical_z_m.shape:
        raise ValueError("instance mask and registered depth resolution differ")
    eroded = _erode_mask(mask, config.boundary_erosion_px)
    selected = eroded & frame.valid_depth_mask
    if not np.any(selected):
        raise ValueError("instance mask contains no valid registered depth")
    depths = frame.depth_optical_z_m[selected].astype(np.float64)
    low, high = np.percentile(depths, (config.depth_percentile_low, config.depth_percentile_high))
    median = float(np.median(depths))
    mad = float(np.median(np.abs(depths - median)))
    discontinuity = max(config.discontinuity_floor_m, config.discontinuity_mad_scale * 1.4826 * mad)
    depth = frame.depth_optical_z_m
    selected &= (depth >= low) & (depth <= high) & (np.abs(depth - median) <= discontinuity)
    if config.select_largest_component:
        selected = _largest_component(selected)
    count = int(selected.sum())
    if count < config.minimum_points:
        raise ValueError(f"registered metric instance has too few filtered points: {count}")
    rows, columns = np.indices(depth.shape, dtype=np.float32)
    fx, fy, cx, cy = frame.K[0], frame.K[4], frame.K[2], frame.K[5]
    points = np.full((*depth.shape, 3), np.nan, dtype=np.float32)
    z = depth[selected]
    points[selected, 0] = (columns[selected] - cx) * z / fx
    points[selected, 1] = (rows[selected] - cy) * z / fy
    points[selected, 2] = z
    evidence = {
        "boundary_erosion_px": config.boundary_erosion_px,
        "depth_percentiles": [config.depth_percentile_low, config.depth_percentile_high],
        "depth_discontinuity_m": discontinuity,
        "connected_depth_component_selected": config.select_largest_component,
        "input_mask_pixels": int(mask.sum()),
        "filtered_point_count": count,
        "metric_scale_validity": "VALID" if source is not MetricPointMapSource.MOGE_MONOCULAR_ESTIMATE else "UNKNOWN",
        "depth_semantics": "optical_z_m",
    }
    return MetricPointMap(
        "metric_point_map_v1", source, points, depth, selected, frame.K,
        frame.metadata.rgb_frame_id, (depth.shape[1], depth.shape[0]), frame.metadata.capture_id,
        frame.metadata.capture_center_time, frame.metadata.calibration_identity, depth_identity, evidence,
    )


@dataclass(frozen=True)
class DetectionProposal:
    source_instance_id: str
    category: str
    bbox_xyxy: tuple[float, float, float, float]
    detection_score: float
    proposal_source: str


@dataclass(frozen=True)
class SegmentationResult:
    source_instance_id: str
    mask: np.ndarray = field(compare=False, repr=False)
    segmentation_score: float
    boundary_quality: float


@dataclass(frozen=True)
class CuboidHypothesis:
    source_instance_id: str
    hypothesis_rank: int
    presence_score: float
    pose_camera: Pose3D
    full_dimensions_xyz_m: tuple[float, float, float]
    visible_faces: tuple[Mapping[str, Any], ...]
    observed_surface_count: int
    point_support_ratio: float
    plane_residual_m: float
    completion_mode: str
    uncertainty: Mapping[str, Any]
    evidence: Mapping[str, Any]
    candidate_eligible: bool
    eligibility_reasons: tuple[str, ...]
    pose_world: Pose3D | None = None

    def __post_init__(self) -> None:
        if not self.source_instance_id or self.hypothesis_rank < 0:
            raise ValueError("cuboid hypothesis identity and rank are invalid")
        if not 0.0 <= float(self.presence_score) <= 1.0 or not 0.0 <= float(self.point_support_ratio) <= 1.0:
            raise ValueError("cuboid confidence and support must be in [0, 1]")
        dimensions = tuple(float(value) for value in self.full_dimensions_xyz_m)
        if len(dimensions) != 3 or any(not isfinite(value) or value <= 0.0 for value in dimensions):
            raise ValueError("cuboid dimensions must be positive finite XYZ")
        if self.plane_residual_m < 0.0 or not isfinite(float(self.plane_residual_m)):
            raise ValueError("cuboid plane residual must be finite and non-negative")
        if self.candidate_eligible and self.eligibility_reasons:
            raise ValueError("eligible cuboid hypothesis cannot have rejection reasons")
        if not self.candidate_eligible and not self.eligibility_reasons:
            raise ValueError("ineligible cuboid hypothesis must explain why")


@runtime_checkable
class ObjectDetector(Protocol):
    def detect(self, frame: RegisteredRgbdFrame) -> Sequence[DetectionProposal]: ...


@runtime_checkable
class InstanceSegmenter(Protocol):
    def segment(self, frame: RegisteredRgbdFrame, proposals: Sequence[DetectionProposal]) -> Sequence[SegmentationResult]: ...


@runtime_checkable
class CuboidPoseEstimator(Protocol):
    def estimate(
        self, segmentation: SegmentationResult, pointmap: MetricPointMap, category: str
    ) -> Sequence[CuboidHypothesis]: ...


def hypotheses_from_geometry_record(
    record: Mapping[str, Any],
    *,
    source_instance_id: str,
    pointmap_source: MetricPointMapSource,
    presence_score: float,
) -> tuple[CuboidHypothesis, ...]:
    """Adapt the pinned plane/cuboid recovery output without MoGe coupling."""

    if not record.get("accepted"):
        return ()
    source = MetricPointMapSource(pointmap_source)
    surfaces = int(record.get("depth_supported_face_count", record.get("visible_plane_count", 0)))
    axes = tuple(tuple(float(value) for value in axis) for axis in record["orthogonal_axes_3d"])
    corner_key = "corners_3d"
    plane_pair = record.get("orthogonal_plane_pair_diagnostics", {})
    if (
        source is not MetricPointMapSource.MOGE_MONOCULAR_ESTIMATE
        and surfaces >= 3
        and plane_pair.get("reliable") is True
        and len(record.get("unanchored_corners_3d", ())) == 8
    ):
        # The inherited monocular pipeline may replace a complete three-plane
        # cuboid with its 2D front-face anchoring heuristic.  With registered
        # metric depth the three observed planes are the stronger 3D datum;
        # retaining their shared cuboid centre avoids converting visible-face
        # thickness into a fictitious object depth.
        corner_key = "unanchored_corners_3d"
    corners = tuple(tuple(float(value) for value in point) for point in record[corner_key])
    dimensions = tuple(float(value) for value in record["shape_dimensions"])
    axis_order = (0, 1, 2)
    if source is not MetricPointMapSource.MOGE_MONOCULAR_ESTIMATE:
        # A cuboid has no intrinsic axis labels.  RANSAC plane discovery order
        # changes with small render differences, so bind local XYZ to
        # descending side length and restore a right-handed basis.  This uses
        # predicted geometry only; no ground-truth pose or size is consulted.
        axis_order = tuple(sorted(range(3), key=lambda index: (-dimensions[index], index)))
        axes = tuple(axes[index] for index in axis_order)
        dimensions = tuple(dimensions[index] for index in axis_order)
        if float(np.linalg.det(np.asarray(axes, dtype=float))) < 0.0:
            axes = (axes[0], axes[1], tuple(-value for value in axes[2]))
    center = tuple(sum(point[index] for point in corners) / len(corners) for index in range(3))
    pose = pose_from_axes_rows(center, axes, "module_0_main_rgb_optical", EvidenceKind.MODEL_ESTIMATED)
    support = min(1.0, max(0.0, float(record.get("plane_inlier_ratio", 0.0))))
    residual = max(0.0, float(record.get("plane_residual_mean", 0.0)))
    completion = str(record.get("completion_mode", "UNKNOWN"))
    ambiguous = surfaces < 2 or "single_visible_plane" in completion
    verified_metric = source is not MetricPointMapSource.MOGE_MONOCULAR_ESTIMATE
    base_reasons = []
    if ambiguous:
        base_reasons.append("GEOMETRIC_POSE_AMBIGUOUS")
    if not verified_metric:
        base_reasons.append("MONOCULAR_SCALE_UNVERIFIED")
    if not record.get("uncertainty"):
        base_reasons.append("GEOMETRY_UNCERTAINTY_INCOMPLETE")
    evidence = {
        "pointmap_source": source.value,
        "algorithm": "PINNED_PLANE_ORTHOGONAL_CUBOID_RECOVERY",
        "upstream_method": record.get("method"),
        "visible_face_evidence": record.get("camera_facing_faces", ()),
        "pose_corner_source": corner_key,
        "axis_order_from_predicted_dimensions": axis_order,
        "metric_scale_validity": Validity.VALID.value if verified_metric else Validity.UNKNOWN.value,
    }
    hypotheses = [CuboidHypothesis(
        source_instance_id, 0, float(presence_score), pose, dimensions,
        tuple(record.get("camera_facing_faces", ())), surfaces, support, residual, completion,
        dict(record.get("uncertainty") or {"status": "NOT_FULLY_QUANTIFIED"}), evidence,
        not base_reasons, tuple(base_reasons),
    )]
    if ambiguous:
        alternate_axes = (axes[0], axes[2], tuple(-value for value in axes[1]))
        alternate_dimensions = (dimensions[0], dimensions[2], dimensions[1])
        alternate_pose = pose_from_axes_rows(center, alternate_axes, pose.frame_id, EvidenceKind.CONSTRAINT_COMPLETED)
        hypotheses.append(CuboidHypothesis(
            source_instance_id, 1, max(0.0, float(presence_score) * 0.95), alternate_pose,
            alternate_dimensions, tuple(record.get("camera_facing_faces", ())), surfaces, support,
            residual, completion, {"status": "DISCRETE_AXIS_SWAP_HYPOTHESIS"},
            {**evidence, "ambiguity": "UNOBSERVED_AXIS_SWAP"}, False,
            tuple(dict.fromkeys((*base_reasons, "ALTERNATE_HYPOTHESIS_REQUIRES_DISAMBIGUATION"))),
        ))
    return tuple(hypotheses)


def transform_hypothesis_to_world(hypothesis: CuboidHypothesis, metadata: CaptureMetadata) -> CuboidHypothesis:
    """Use capture-time T_W_C; inference-time robot state is not an input."""

    world = transform_pose(metadata.T_W_C_at_capture, hypothesis.pose_camera, "world")
    return CuboidHypothesis(**{**hypothesis.__dict__, "pose_world": world})
