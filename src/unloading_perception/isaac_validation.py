"""Versioned, backend-neutral contracts for Isaac perception validation.

Isaac Sim, ROS and CUDA are deliberately absent from this module.  The
heavyweight capture process writes plain artifacts; this module validates
their identity, creates explicit simulation ground truth observations and
builds the handoff consumed by a future feasibility checkout.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from math import isfinite, sqrt
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from unloading_contracts import (
    SCHEMA_VERSION,
    CargoObservation,
    EvidenceKind,
    ObservationStatus,
    PerceptionObservation,
    PlanningWorldSnapshot,
    Pose3D,
    ResourceReference,
    UnknownRegion,
    Validity,
    canonical_fingerprint,
    to_wire,
)


ISAAC_SCENE_MANIFEST_SCHEMA = "isaac_scene_manifest_v1"
ISAAC_CAPTURE_BINDING_SCHEMA = "isaac_capture_binding_v1"
ISAAC_FEASIBILITY_HANDOFF_SCHEMA = "isaac_feasibility_handoff_v1"
WORLD_FRAME = "world"
AXIS_CONVENTION = "+X into trailer, +Y left, +Z up"
QUATERNION_CONVENTION = "xyzw"
LENGTH_UNIT = "metre"
ANGLE_UNIT = "radian"
MASS_UNIT = "kilogram"


def canonical_digest(value: Any) -> str:
    """Return the content identity used by scene and capture artifacts."""

    return canonical_fingerprint(value)


def feasibility_digest(value: Any) -> str:
    """Match the frozen feasibility layout/exporter's canonical JSON hash."""

    return hashlib.sha256(
        json.dumps(value, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_text_asset_identity(path: str | Path, expected_sha256: str) -> dict[str, str]:
    """Verify a text asset while making Git's newline checkout policy explicit.

    The frozen feasibility export was created from a Windows checkout and its
    byte hash therefore includes CRLF newlines.  Linux servers check the same
    Git blob out with LF newlines.  Only this lossless newline transformation
    is accepted; every other byte difference still fails closed.
    """

    asset_path = Path(path)
    expected = _require_sha256(expected_sha256, "expected asset hash")
    raw = asset_path.read_bytes()
    actual = hashlib.sha256(raw).hexdigest()
    if actual == expected:
        return {"contract_sha256": expected, "checkout_sha256": actual, "normalization": "EXACT_BYTES"}
    lf = raw.replace(b"\r\n", b"\n")
    crlf = lf.replace(b"\n", b"\r\n")
    if hashlib.sha256(crlf).hexdigest() == expected:
        return {
            "contract_sha256": expected,
            "checkout_sha256": actual,
            "normalization": "GIT_TEXT_LF_CHECKOUT_OF_CRLF_CONTRACT",
        }
    raise ValueError("text asset differs from the frozen contract beyond newline normalization")


def _require_sha256(value: Any, name: str) -> str:
    result = str(value).lower()
    if len(result) != 64 or any(character not in "0123456789abcdef" for character in result):
        raise ValueError(f"{name} must be a lowercase SHA-256")
    return result


def _finite_vector(value: Any, length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in value)
    if len(result) != length or not all(isfinite(item) for item in result):
        raise ValueError(f"{name} must contain {length} finite values")
    return result


def _transform(value: Any, name: str) -> tuple[tuple[float, ...], ...]:
    rows = tuple(_finite_vector(row, 4, name) for row in value)
    if len(rows) != 4 or any(abs(rows[3][index] - expected) > 1e-9 for index, expected in enumerate((0, 0, 0, 1))):
        raise ValueError(f"{name} must be a homogeneous 4x4 transform")
    rotation = tuple(row[:3] for row in rows[:3])
    for first in range(3):
        for second in range(3):
            dot = sum(rotation[row][first] * rotation[row][second] for row in range(3))
            if abs(dot - (1.0 if first == second else 0.0)) > 1e-6:
                raise ValueError(f"{name} rotation must be orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-6:
        raise ValueError(f"{name} rotation must be right handed")
    return rows


def _rigid_inverse(value: Sequence[Sequence[float]]) -> list[list[float]]:
    transform = _transform(value, "transform")
    rotation = tuple(row[:3] for row in transform[:3])
    translation = tuple(row[3] for row in transform[:3])
    inverse_rotation = tuple(tuple(rotation[column][row] for column in range(3)) for row in range(3))
    inverse_translation = tuple(-sum(inverse_rotation[row][column] * translation[column] for column in range(3)) for row in range(3))
    return [list(inverse_rotation[row]) + [inverse_translation[row]] for row in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _matmul(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[sum(float(left[row][k]) * float(right[k][column]) for k in range(4)) for column in range(4)] for row in range(4)]


def _rotation_quaternion_xyzw(rotation: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    matrix = tuple(tuple(float(value) for value in row) for row in rotation)
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        result = ((matrix[2][1] - matrix[1][2]) / scale, (matrix[0][2] - matrix[2][0]) / scale, (matrix[1][0] - matrix[0][1]) / scale, 0.25 * scale)
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        result = (0.25 * scale, (matrix[0][1] + matrix[1][0]) / scale, (matrix[0][2] + matrix[2][0]) / scale, (matrix[2][1] - matrix[1][2]) / scale)
    elif matrix[1][1] > matrix[2][2]:
        scale = sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        result = ((matrix[0][1] + matrix[1][0]) / scale, 0.25 * scale, (matrix[1][2] + matrix[2][1]) / scale, (matrix[0][2] - matrix[2][0]) / scale)
    else:
        scale = sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        result = ((matrix[0][2] + matrix[2][0]) / scale, (matrix[1][2] + matrix[2][1]) / scale, 0.25 * scale, (matrix[1][0] - matrix[0][1]) / scale)
    norm = sqrt(sum(value * value for value in result))
    return tuple(value / norm for value in result)


def load_validation_config(path: str | Path) -> dict[str, Any]:
    """Load and strictly validate the lightweight Isaac validation config."""

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != "isaac_perception_validation_v1":
        raise ValueError("unsupported Isaac perception validation config")
    if data.get("modes") != ["ISAAC_GROUND_TRUTH", "ISAAC_SENSOR_WITH_ORACLE_PROPOSALS"]:
        raise ValueError("validation config must keep Mode A and Mode B1 distinct")
    if data.get("raw_image_automatic") is not False:
        raise ValueError("RAW_IMAGE_AUTOMATIC must remain false for this validation round")
    cameras = data.get("cameras")
    if not isinstance(cameras, list) or not cameras:
        raise ValueError("at least one validation camera is required")
    for camera in cameras:
        if set(camera) != {
            "camera_id", "frame_id", "resolution", "K", "distortion_model", "distortion",
            "T_W_C", "near_clip_m", "far_clip_m", "publish_rate_hz", "modalities",
            "focal_length_mm", "horizontal_aperture_mm", "look_at_world_m",
        }:
            raise ValueError("validation camera contains unknown or missing fields")
        width, height = (int(value) for value in camera["resolution"])
        if width <= 0 or height <= 0:
            raise ValueError("camera resolution must be positive")
        intrinsics = _finite_vector(camera["K"], 9, "camera K")
        if intrinsics[0] <= 0.0 or intrinsics[4] <= 0.0 or intrinsics[8] != 1.0:
            raise ValueError("camera K is invalid")
        _transform(camera["T_W_C"], "T_W_C")
        near, far = float(camera["near_clip_m"]), float(camera["far_clip_m"])
        if not (0.0 < near < far) or float(camera["publish_rate_hz"]) <= 0.0:
            raise ValueError("camera clipping and publish rate are invalid")
        required_modalities = {"rgb", "metric_depth", "camera_info", "pointcloud", "gt_annotations"}
        if not required_modalities.issubset(set(camera["modalities"])):
            raise ValueError("camera must enable all acceptance modalities")
    return data


@dataclass(frozen=True)
class IsaacSceneManifest:
    run_id: str
    source: Mapping[str, Any]
    layout: Mapping[str, Any]
    robot: Mapping[str, Any]
    mechanisms: Mapping[str, Any]
    cameras: tuple[Mapping[str, Any], ...]
    objects: tuple[Mapping[str, Any], ...]
    timing: Mapping[str, Any]
    provenance: Mapping[str, Any]
    dynamic_scene_fingerprint: str
    world_fingerprint: str
    manifest_fingerprint: str
    schema_version: str = ISAAC_SCENE_MANIFEST_SCHEMA

    def __post_init__(self) -> None:
        if self.schema_version != ISAAC_SCENE_MANIFEST_SCHEMA or not self.run_id:
            raise ValueError("invalid Isaac scene manifest envelope")
        for name in ("perception_commit", "feasibility_reference_commit", "isaac_version"):
            if not self.source.get(name):
                raise ValueError(f"manifest source.{name} is required")
        _require_sha256(self.layout.get("layout_fingerprint"), "layout fingerprint")
        _require_sha256(self.layout.get("asset_manifest_fingerprint"), "asset manifest fingerprint")
        _transform(self.layout.get("T_W_A"), "T_W_A")
        if self.layout.get("world_frame") != WORLD_FRAME or self.layout.get("axis_convention") != AXIS_CONVENTION:
            raise ValueError("manifest world convention mismatch")
        if self.layout.get("units") != {"length": LENGTH_UNIT, "angle": ANGLE_UNIT, "mass": MASS_UNIT}:
            raise ValueError("manifest must use SI units")
        object_ids = [str(item.get("simulation_object_id", "")) for item in self.objects]
        if any(not value for value in object_ids) or len(object_ids) != len(set(object_ids)):
            raise ValueError("simulation object IDs must be non-empty and unique")
        if any(tuple(item.get("full_dimensions_m", ())) != _finite_vector(item.get("full_dimensions_m", ()), 3, "full dimensions") or any(float(v) <= 0.0 for v in item["full_dimensions_m"]) for item in self.objects):
            raise ValueError("simulation objects require positive full dimensions")
        for camera in self.cameras:
            _transform(camera.get("T_W_C"), "T_W_C")
        expected_dynamic = canonical_digest({
            "objects": self.objects,
            "mechanisms": self.mechanisms,
            "camera_calibration": tuple({key: camera[key] for key in ("camera_id", "frame_id", "resolution", "K", "distortion_model", "distortion", "T_W_C", "near_clip_m", "far_clip_m")} for camera in self.cameras),
        })
        if self.dynamic_scene_fingerprint != expected_dynamic:
            raise ValueError("dynamic scene fingerprint mismatch")
        expected_world = canonical_digest({
            "layout_fingerprint": self.layout["layout_fingerprint"],
            "dynamic_scene_fingerprint": self.dynamic_scene_fingerprint,
            "robot": self.robot,
        })
        if self.world_fingerprint != expected_world:
            raise ValueError("world fingerprint mismatch")
        payload = self.to_dict()
        recorded = payload.pop("manifest_fingerprint")
        if recorded != canonical_digest(payload):
            raise ValueError("manifest fingerprint mismatch")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "source": dict(self.source),
            "layout": dict(self.layout),
            "robot": dict(self.robot),
            "mechanisms": dict(self.mechanisms),
            "cameras": [dict(item) for item in self.cameras],
            "objects": [dict(item) for item in self.objects],
            "timing": dict(self.timing),
            "provenance": dict(self.provenance),
            "dynamic_scene_fingerprint": self.dynamic_scene_fingerprint,
            "world_fingerprint": self.world_fingerprint,
            "manifest_fingerprint": self.manifest_fingerprint,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "IsaacSceneManifest":
        return cls(
            run_id=str(value["run_id"]), source=dict(value["source"]), layout=dict(value["layout"]),
            robot=dict(value["robot"]), mechanisms=dict(value["mechanisms"]),
            cameras=tuple(dict(item) for item in value["cameras"]),
            objects=tuple(dict(item) for item in value["objects"]), timing=dict(value["timing"]),
            provenance=dict(value["provenance"]), dynamic_scene_fingerprint=str(value["dynamic_scene_fingerprint"]),
            world_fingerprint=str(value["world_fingerprint"]), manifest_fingerprint=str(value["manifest_fingerprint"]),
            schema_version=str(value["schema_version"]),
        )


def build_scene_manifest(
    snapshot: Mapping[str, Any],
    layout_contract: Mapping[str, Any],
    validation_config: Mapping[str, Any],
    *,
    run_id: str,
    perception_commit: str,
    feasibility_reference_commit: str,
    isaac_version: str,
    simulation_epoch: str,
    simulation_frame: int = 0,
    simulation_time: float = 0.0,
    object_overrides: Mapping[str, Mapping[str, Any]] | None = None,
) -> IsaacSceneManifest:
    """Build one manifest from the feasibility-exported frozen snapshot."""

    if layout_contract.get("layout_fingerprint") != snapshot.get("layout_fingerprint"):
        raise ValueError("snapshot and Isaac layout contract use different layouts")
    contract_payload = dict(layout_contract)
    recorded_contract_fingerprint = contract_payload.pop("contract_fingerprint", None)
    if recorded_contract_fingerprint != feasibility_digest(contract_payload):
        raise ValueError("Isaac layout contract fingerprint mismatch")
    if snapshot.get("scene_fingerprint") != layout_contract.get("scene_fingerprint"):
        raise ValueError("snapshot and Isaac layout contract scene identity mismatch")
    snapshot_payload = dict(snapshot)
    recorded_scene_fingerprint = snapshot_payload.pop("scene_fingerprint", None)
    if recorded_scene_fingerprint != feasibility_digest(snapshot_payload):
        raise ValueError("scene snapshot fingerprint mismatch")
    if not run_id or not simulation_epoch or simulation_frame < 0 or not isfinite(float(simulation_time)):
        raise ValueError("run and simulation timing identities are invalid")
    overrides = object_overrides or {}
    objects = []
    for source in snapshot["cartons"]:
        object_id = str(source["name"])
        override = dict(overrides.get(object_id, {}))
        unknown = set(override) - {"T_W_object", "occluded", "visible", "state"}
        if unknown:
            raise ValueError(f"unknown object override fields for {object_id}: {sorted(unknown)}")
        pose = override.get("T_W_object", source["pose_world"])
        _transform(pose, f"T_W_object[{object_id}]")
        objects.append({
            "simulation_object_id": object_id,
            "semantic_id": f"carton:{object_id}",
            "category": "box",
            "T_W_object": pose,
            "full_dimensions_m": [2.0 * float(value) for value in source["half_extents_m"]],
            "visible": bool(override.get("visible", True)),
            "occluded": bool(override.get("occluded", False)),
            "state": str(override.get("state", "INITIAL")),
        })
    asset_records = [snapshot["robot"]["urdf"], snapshot["robot"]["model_config"], *snapshot["tool"].get("assets", {}).values()]
    asset_manifest_fingerprint = canonical_digest(sorted((item["repository_path"], item["sha256"]) for item in asset_records))
    t_w_a = snapshot["assembly"]["pose_world"]
    t_a_robot = _matmul(_rigid_inverse(t_w_a), snapshot["robot"]["world_from_mount"])
    mechanisms = {
        "components": [{
            "component_id": item["name"],
            "category": item["category"],
            "T_A_component": _matmul(_rigid_inverse(t_w_a), item["pose_world"]),
            "state_identity": canonical_digest({"name": item["name"], "pose": item["pose_world"]}),
        } for item in snapshot["assembly"]["fixed_components"]],
        "tool_state": {"identity": canonical_digest(snapshot["tool"]), "attached": True},
        "payload_state": {"identity": canonical_digest(snapshot.get("attachments", [])), "object_id": None},
        "base_state": {"identity": canonical_digest({"T_W_A": t_w_a}), "T_W_A": t_w_a},
        "conveyor_state": {"identity": canonical_digest(snapshot["receiver"]), "running": False},
    }
    cameras = tuple(dict(camera) for camera in validation_config["cameras"])
    dynamic = canonical_digest({
        "objects": tuple(objects),
        "mechanisms": mechanisms,
        "camera_calibration": tuple({key: camera[key] for key in ("camera_id", "frame_id", "resolution", "K", "distortion_model", "distortion", "T_W_C", "near_clip_m", "far_clip_m")} for camera in cameras),
    })
    robot = {
        "robot_model_identity": snapshot["robot"]["model"],
        "robot_asset_hash": snapshot["robot"]["urdf"]["sha256"],
        "robot_asset_path": snapshot["robot"]["urdf"]["repository_path"],
        "T_A_robot": t_a_robot,
        "T_W_robot": snapshot["robot"]["world_from_mount"],
        "joint_names": snapshot["robot"]["joint_names"],
        "q_rad": snapshot["robot"]["q_rad"],
    }
    world_fingerprint = canonical_digest({
        "layout_fingerprint": snapshot["layout_fingerprint"],
        "dynamic_scene_fingerprint": dynamic,
        "robot": robot,
    })
    payload = {
        "schema_version": ISAAC_SCENE_MANIFEST_SCHEMA,
        "run_id": run_id,
        "source": {
            "perception_commit": perception_commit,
            "feasibility_reference_commit": feasibility_reference_commit,
            "isaac_version": isaac_version,
            "layout_contract_fingerprint": recorded_contract_fingerprint,
            "source_scene_fingerprint": recorded_scene_fingerprint,
        },
        "layout": {
            "layout_id": snapshot["layout_id"],
            "layout_schema_version": snapshot["schema"],
            "layout_fingerprint": snapshot["layout_fingerprint"],
            "asset_manifest_fingerprint": asset_manifest_fingerprint,
            "world_frame": WORLD_FRAME,
            "axis_convention": AXIS_CONVENTION,
            "units": {"length": LENGTH_UNIT, "angle": ANGLE_UNIT, "mass": MASS_UNIT},
            "T_W_A": t_w_a,
        },
        "robot": robot,
        "mechanisms": mechanisms,
        "cameras": list(cameras),
        "objects": objects,
        "timing": {
            "simulation_epoch": simulation_epoch,
            "simulation_frame": int(simulation_frame),
            "simulation_time": float(simulation_time),
        },
        "provenance": {
            "exact_isaac_state": "ISAAC_GROUND_TRUTH",
            "rendered_sensor": "ISAAC_SENSOR",
            "algorithm_estimate": "MODEL_ESTIMATED",
            "oracle_proposal": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
            "raw_image_automatic": False,
        },
        "dynamic_scene_fingerprint": dynamic,
        "world_fingerprint": world_fingerprint,
    }
    payload["manifest_fingerprint"] = canonical_digest(payload)
    return IsaacSceneManifest.from_dict(payload)


@dataclass(frozen=True)
class IsaacCaptureBinding:
    simulation_epoch: str
    frame_sequence: int
    simulation_time: float
    rgb_sha256: str
    camera_calibration_identity: str
    gt_snapshot_sha256: str
    schema_version: str = ISAAC_CAPTURE_BINDING_SCHEMA

    def __post_init__(self) -> None:
        if self.schema_version != ISAAC_CAPTURE_BINDING_SCHEMA or not self.simulation_epoch or self.frame_sequence < 0:
            raise ValueError("invalid Isaac capture binding")
        if not isfinite(float(self.simulation_time)) or self.simulation_time < 0.0:
            raise ValueError("simulation time must be finite and non-negative")
        _require_sha256(self.rgb_sha256, "RGB hash")
        _require_sha256(self.camera_calibration_identity, "camera calibration identity")
        _require_sha256(self.gt_snapshot_sha256, "GT snapshot hash")

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "IsaacCaptureBinding":
        return cls(
            str(value["simulation_epoch"]), int(value["frame_sequence"]), float(value["simulation_time"]),
            str(value["rgb_sha256"]), str(value["camera_calibration_identity"]), str(value["gt_snapshot_sha256"]),
            str(value.get("schema_version", ISAAC_CAPTURE_BINDING_SCHEMA)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "simulation_epoch": self.simulation_epoch,
            "frame_sequence": self.frame_sequence,
            "simulation_time": self.simulation_time,
            "rgb_sha256": self.rgb_sha256,
            "camera_calibration_identity": self.camera_calibration_identity,
            "gt_snapshot_sha256": self.gt_snapshot_sha256,
        }


class HistoricalResultGate:
    """Bind slow vision results to capture history without mutating live state."""

    def __init__(self) -> None:
        self._captures: dict[tuple[str, int], IsaacCaptureBinding] = {}
        self._live_key: tuple[str, int] | None = None

    def register_capture(self, binding: IsaacCaptureBinding, *, live: bool = True) -> None:
        key = (binding.simulation_epoch, binding.frame_sequence)
        previous = self._captures.get(key)
        if previous is not None and previous != binding:
            raise ValueError("capture identity collision")
        self._captures[key] = binding
        if live:
            if self._live_key is not None and self._live_key[0] == key[0] and key[1] <= self._live_key[1]:
                raise ValueError("live capture sequence must increase")
            self._live_key = key

    def classify_result(self, binding: IsaacCaptureBinding) -> dict[str, Any]:
        key = (binding.simulation_epoch, binding.frame_sequence)
        if self._captures.get(key) != binding:
            raise ValueError("vision result is not bound to a registered historical capture")
        live = key == self._live_key
        return {
            "evaluation_eligible": True,
            "live_world_eligible": live,
            "reason": "CURRENT_CAPTURE" if live else "HISTORICAL_RESULT_MUST_NOT_REPLACE_LIVE_WORLD",
            "capture": binding.to_dict(),
        }


class SimulationClockGuard:
    """Validate Isaac clock progress, pause and explicit epoch reset."""

    def __init__(self) -> None:
        self.epoch: str | None = None
        self.frame = -1
        self.simulation_time = 0.0

    def observe(
        self,
        epoch: str,
        frame: int,
        simulation_time: float,
        *,
        paused: bool = False,
        restart: bool = False,
    ) -> None:
        epoch, frame, value = str(epoch), int(frame), float(simulation_time)
        if not epoch or frame < 0 or not isfinite(value) or value < 0.0:
            raise ValueError("simulation clock sample is invalid")
        if self.epoch is None:
            if restart and frame != 0:
                raise ValueError("initial restart sample must use frame zero")
        elif epoch == self.epoch:
            if restart:
                raise ValueError("restart must create a new simulation epoch")
            if frame <= self.frame:
                raise ValueError("simulation frame must increase")
            if paused:
                if value != self.simulation_time:
                    raise ValueError("paused simulation time must not advance")
            elif value <= self.simulation_time:
                raise ValueError("running simulation time must increase")
        else:
            if not restart or frame != 0:
                raise ValueError("new simulation epoch requires explicit frame-zero restart")
        self.epoch, self.frame, self.simulation_time = epoch, frame, value


def ground_truth_observation(
    manifest: IsaacSceneManifest,
    annotations: Sequence[Mapping[str, Any]],
    *,
    processed_time: float | None = None,
) -> PerceptionObservation:
    """Convert exact Isaac object state to explicitly synthetic domain data."""

    timing = manifest.timing
    capture_time = float(timing["simulation_time"])
    processed = capture_time if processed_time is None else float(processed_time)
    if processed < capture_time:
        raise ValueError("processed_time cannot precede capture_time")
    objects = {str(item["simulation_object_id"]): item for item in manifest.objects}
    cargo = []
    unknown = []
    for annotation in annotations:
        object_id = str(annotation.get("simulation_object_id", ""))
        if object_id not in objects:
            raise ValueError(f"GT annotation references unknown object: {object_id}")
        source = objects[object_id]
        pose = _transform(source["T_W_object"], f"T_W_object[{object_id}]")
        bbox = _finite_vector(annotation.get("bbox_xyxy", ()), 4, "GT bbox")
        occluded = bool(annotation.get("occluded", source.get("occluded", False)))
        visible = bool(annotation.get("visible", source.get("visible", True)))
        projected_bbox = bbox
        if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
            if visible:
                raise ValueError("visible GT bbox must have positive area")
            width, height = (float(value) for value in manifest.cameras[0]["resolution"])
            bbox = (0.0, 0.0, width, height)
        eligible = visible and not occluded
        reasons = () if eligible else (("SIMULATION_OBJECT_OCCLUDED",) if occluded else ("SIMULATION_OBJECT_NOT_VISIBLE",))
        if not eligible:
            unknown.append(UnknownRegion(f"gt-{object_id}", manifest.cameras[0]["frame_id"], reasons[0], bbox))
        rotation = tuple(row[:3] for row in pose[:3])
        mask_value = annotation.get("mask_reference")
        mask_reference = None if mask_value is None else ResourceReference(
            str(mask_value["uri"]), str(mask_value["sha256"]), str(mask_value["media_type"])
        )
        cargo.append(CargoObservation(
            source_instance_id=object_id,
            object_id=object_id,
            track_id=object_id,
            category=str(source["category"]),
            bbox_xyxy=bbox,
            mask_reference=mask_reference,
            detection_score=1.0,
            contour_score=None,
            reprojection_score=1.0,
            geometry_error_m=0.0,
            pose=Pose3D(tuple(row[3] for row in pose[:3]), _rotation_quaternion_xyzw(rotation), WORLD_FRAME, AXIS_CONVENTION, EvidenceKind.SYNTHETIC),
            full_dimensions_m=tuple(float(value) for value in source["full_dimensions_m"]),
            corners_3d_m=None,
            axes_3d_rows=tuple(tuple(rotation[row][column] for row in range(3)) for column in range(3)),
            pose_evidence=EvidenceKind.SYNTHETIC,
            size_evidence=EvidenceKind.SYNTHETIC,
            depth_evidence=EvidenceKind.SYNTHETIC,
            scale_evidence=EvidenceKind.SYNTHETIC,
            metric_scale_validity=Validity.VALID,
            geometry_validity=Validity.VALID,
            candidate_eligible=eligible,
            eligibility_reasons=reasons,
            occluded=occluded,
            association_status="SIMULATION_IDENTITY",
            raw_result={
                "isaac_prim_path": annotation.get("prim_path"),
                "semantic_identity": source["semantic_id"],
                "simulation_frame": timing["simulation_frame"],
                "projected_bbox_xyxy": projected_bbox,
                "bbox_fallback": bbox != projected_bbox,
                "oracle_source": "ISAAC_GROUND_TRUTH",
                "simulation_oracle_state": True,
            },
        ))
    return PerceptionObservation(
        schema_version=SCHEMA_VERSION,
        observation_id=f"isaac-gt-{timing['simulation_epoch']}-{timing['simulation_frame']}",
        source_epoch=str(timing["simulation_epoch"]),
        source_sequence=int(timing["simulation_frame"]),
        capture_time=capture_time,
        processed_time=processed,
        clock_domain="ros_sim_time",
        provider="isaac-sim-ground-truth",
        upstream_commit=str(manifest.source["perception_commit"]),
        model_identity="isaac-exact-simulation-state",
        config_identity=str(manifest.manifest_fingerprint),
        status=ObservationStatus.COMPLETE if not unknown else ObservationStatus.PARTIAL,
        failure_code=None,
        failure_message=None,
        cargo=tuple(cargo),
        unknown_regions=tuple(unknown),
        coverage={
            "frame_id": WORLD_FRAME,
            "absence_means_free_space": not unknown,
            "provenance": "ISAAC_GROUND_TRUTH",
            "manifest_fingerprint": manifest.manifest_fingerprint,
        },
        synthetic=True,
    )


def build_feasibility_handoff(
    snapshot: PlanningWorldSnapshot,
    manifest: IsaacSceneManifest,
    *,
    evidence_mode: str,
) -> dict[str, Any]:
    if evidence_mode not in {"ISAAC_GT", "VISION_ESTIMATE"}:
        raise ValueError("handoff evidence mode must be ISAAC_GT or VISION_ESTIMATE")
    scene = snapshot.scene_snapshot
    unknown = list(scene.get("unknown_regions", ())) if isinstance(scene, Mapping) else []
    objects = list(scene.get("obstacles", ())) if isinstance(scene, Mapping) else []
    payload = to_wire({
        "schema_version": ISAAC_FEASIBILITY_HANDOFF_SCHEMA,
        "planning_world_snapshot": to_wire(snapshot),
        "isaac_scene_manifest": {
            "run_id": manifest.run_id,
            "manifest_fingerprint": manifest.manifest_fingerprint,
        },
        "layout_id": manifest.layout["layout_id"],
        "layout_fingerprint": manifest.layout["layout_fingerprint"],
        "dynamic_scene_fingerprint": manifest.dynamic_scene_fingerprint,
        "world_fingerprint": snapshot.fingerprint,
        "isaac_world_fingerprint": manifest.world_fingerprint,
        "robot_state_identity": snapshot.robot_state_revision.fingerprint,
        "tool_state": snapshot.tool_attachment,
        "payload_state": snapshot.payload_attachment,
        "base_state": snapshot.base_state,
        "conveyor_state": snapshot.conveyor_state,
        "simulation": dict(manifest.timing),
        "provenance": dict(manifest.provenance),
        "evidence_mode": evidence_mode,
        "world_frame": WORLD_FRAME,
        "axis_convention": AXIS_CONVENTION,
        "units": {"length": LENGTH_UNIT, "angle": ANGLE_UNIT, "mass": MASS_UNIT},
        "quaternion_convention": QUATERNION_CONVENTION,
        "unknown_regions": unknown,
        "objects": objects,
        "asset_manifest_fingerprint": manifest.layout["asset_manifest_fingerprint"],
    })
    payload["handoff_fingerprint"] = canonical_digest(payload)
    return payload


def check_feasibility_handoff(
    handoff: Mapping[str, Any],
    manifest: IsaacSceneManifest,
    layout_contract: Mapping[str, Any],
    *,
    tolerance: float = 1e-9,
) -> dict[str, Any]:
    """Perform a structural cross-branch check without invoking a planner."""

    if tolerance < 0.0 or not isfinite(float(tolerance)):
        raise ValueError("compatibility tolerance must be finite and non-negative")
    differences: list[dict[str, Any]] = []

    def compare(field: str, actual: Any, expected: Any) -> None:
        if actual != expected:
            differences.append({"field": field, "actual": actual, "expected": expected})

    compare("schema_version", handoff.get("schema_version"), ISAAC_FEASIBILITY_HANDOFF_SCHEMA)
    compare("layout_id", handoff.get("layout_id"), layout_contract.get("layout_id"))
    compare("layout_fingerprint", handoff.get("layout_fingerprint"), layout_contract.get("layout_fingerprint"))
    compare("world_frame", handoff.get("world_frame"), WORLD_FRAME)
    compare("axis_convention", handoff.get("axis_convention"), AXIS_CONVENTION)
    compare("units", handoff.get("units"), {"length": LENGTH_UNIT, "angle": ANGLE_UNIT, "mass": MASS_UNIT})
    compare("quaternion_convention", handoff.get("quaternion_convention"), QUATERNION_CONVENTION)
    compare("asset_manifest_fingerprint", handoff.get("asset_manifest_fingerprint"), manifest.layout["asset_manifest_fingerprint"])
    expected_ids = sorted(str(item["simulation_object_id"]) for item in manifest.objects)
    handoff_objects = handoff.get("objects", ())
    actual_ids = sorted(str(item.get("object_id")) for item in handoff_objects if item.get("object_id") is not None)
    compare("object_ids", actual_ids, expected_ids)
    manifest_by_id = {str(item["simulation_object_id"]): item for item in manifest.objects}
    for item in handoff_objects:
        object_id = str(item.get("object_id"))
        if object_id not in manifest_by_id:
            continue
        actual_dimensions = tuple(float(value) for value in item.get("full_dimensions_m") or ())
        expected_dimensions = tuple(float(value) for value in manifest_by_id[object_id]["full_dimensions_m"])
        if len(actual_dimensions) != 3 or max((abs(a - b) for a, b in zip(actual_dimensions, expected_dimensions)), default=float("inf")) > tolerance:
            differences.append({"field": f"objects.{object_id}.full_dimensions_m", "actual": actual_dimensions, "expected": expected_dimensions})
        if item.get("candidate_eligible") and item.get("eligibility_reasons"):
            differences.append({"field": f"objects.{object_id}.candidate_eligible", "actual": True, "expected": "no eligibility reasons"})
    payload = dict(handoff)
    recorded = payload.pop("handoff_fingerprint", None)
    if recorded != canonical_digest(payload):
        differences.append({"field": "handoff_fingerprint", "actual": recorded, "expected": canonical_digest(payload)})
    return {
        "schema_version": "isaac_feasibility_compatibility_report_v1",
        "status": "PASS" if not differences else "FAIL",
        "feasibility_reference_commit": manifest.source["feasibility_reference_commit"],
        "layout_fingerprint": manifest.layout["layout_fingerprint"],
        "tolerance": float(tolerance),
        "differences": differences,
    }


def write_json(path: str | Path, value: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
