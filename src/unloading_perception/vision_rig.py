"""Deterministic kinematics for the J1-coupled perception mast.

The module deliberately contains no Isaac Sim or ROS dependency.  All poses
are ``T_parent_child`` transforms in SI units.  Camera coordinates use the
usual optical convention: +X right, +Y down and +Z forward.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import atan2, cos, isfinite, pi, sin, sqrt, tan
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .geometry import validate_transform_parent_child


def _finite(value: Any, name: str) -> float:
    number = float(value)
    if not isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _positive(value: Any, name: str) -> float:
    number = _finite(value, name)
    if number <= 0.0:
        raise ValueError(f"{name} must be positive")
    return number


def _matmul(
    left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(float(left[row][k]) * float(right[k][column]) for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _translation(x: float, y: float, z: float) -> tuple[tuple[float, ...], ...]:
    return (
        (1.0, 0.0, 0.0, float(x)),
        (0.0, 1.0, 0.0, float(y)),
        (0.0, 0.0, 1.0, float(z)),
        (0.0, 0.0, 0.0, 1.0),
    )


def _yaw_transform(yaw_rad: float) -> tuple[tuple[float, ...], ...]:
    c, s = cos(float(yaw_rad)), sin(float(yaw_rad))
    return (
        (c, -s, 0.0, 0.0),
        (s, c, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _optical_pose(position: Sequence[float], heading_rad: float) -> tuple[tuple[float, ...], ...]:
    """Return T_W_C for a level optical camera at the requested world yaw."""

    c, s = cos(heading_rad), sin(heading_rad)
    x, y, z = (s, -c, 0.0), (0.0, 0.0, -1.0), (c, s, 0.0)
    return (
        (x[0], y[0], z[0], float(position[0])),
        (x[1], y[1], z[1], float(position[1])),
        (x[2], y[2], z[2], float(position[2])),
        (0.0, 0.0, 0.0, 1.0),
    )


def wrap_to_pi(angle_rad: float) -> float:
    return atan2(sin(float(angle_rad)), cos(float(angle_rad)))


@dataclass(frozen=True)
class ExactFovCamera:
    width_px: int
    height_px: int
    hfov_rad: float
    vfov_rad: float
    frame_rate_hz: float
    intrinsics_mode: str

    def __post_init__(self) -> None:
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("camera resolution must be positive")
        horizontal = _positive(self.hfov_rad, "horizontal FOV")
        vertical = _positive(self.vfov_rad, "vertical FOV")
        if horizontal >= pi or vertical >= pi:
            raise ValueError("camera FOV must be less than pi")
        _positive(self.frame_rate_hz, "camera frame rate")
        if self.intrinsics_mode not in {"EXACT_USER_SPEC", "EXACT_USER_SPEC_RESAMPLED"}:
            raise ValueError("unsupported exact-FOV intrinsics mode")

    @property
    def fx_px(self) -> float:
        return self.width_px / (2.0 * tan(self.hfov_rad / 2.0))

    @property
    def fy_px(self) -> float:
        return self.height_px / (2.0 * tan(self.vfov_rad / 2.0))

    @property
    def K(self) -> tuple[float, ...]:
        return (
            self.fx_px,
            0.0,
            (self.width_px - 1.0) / 2.0,
            0.0,
            self.fy_px,
            (self.height_px - 1.0) / 2.0,
            0.0,
            0.0,
            1.0,
        )

    @property
    def square_pixel_compatible(self) -> bool:
        return abs(self.fx_px - self.fy_px) <= 1e-9


@dataclass(frozen=True)
class VisionRigSpec:
    rig_id: str
    parent_frame: str
    j1_joint_origin_z_from_base_m: float
    mast_offset_j1_xy_m: tuple[float, float]
    flange_height_j1_m: float
    flange_height_status: str
    mast_height_m: float
    module_height_m: float
    mast_j1_yaw_offset_rad: float
    nominal_q1_rad: float
    nominal_camera_heading_world_rad: float
    independent_mast_yaw: bool
    geometry_qualification: str
    rgb: ExactFovCamera
    depth: ExactFovCamera
    depth_semantics: str
    calibration_identity: str
    light_offsets_module_m: tuple[tuple[float, float, float], tuple[float, float, float]]

    def __post_init__(self) -> None:
        if not self.rig_id or self.parent_frame != "J1_link":
            raise ValueError("vision rig requires a non-empty id and J1_link parent")
        values = (
            self.j1_joint_origin_z_from_base_m,
            *self.mast_offset_j1_xy_m,
            self.flange_height_j1_m,
            self.mast_height_m,
            self.module_height_m,
            self.mast_j1_yaw_offset_rad,
            self.nominal_q1_rad,
            self.nominal_camera_heading_world_rad,
        )
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("vision rig dimensions and angles must be finite")
        if self.mast_height_m <= 0.0 or not 0.0 < self.module_height_m <= self.mast_height_m:
            raise ValueError("module height must lie on the positive mast span")
        radius = sqrt(sum(float(value) ** 2 for value in self.mast_offset_j1_xy_m))
        if abs(radius - 0.4) > 1e-9:
            raise ValueError("J1-coupled mast radius must be exactly 0.4 m")
        if abs(abs(wrap_to_pi(self.mast_j1_yaw_offset_rad)) - pi / 2.0) > 1e-9:
            raise ValueError("mast and J1 heading must differ by exactly 90 degrees")
        if self.independent_mast_yaw:
            raise ValueError("independent mast yaw is not implemented")
        c, s = cos(self.nominal_q1_rad), sin(self.nominal_q1_rad)
        nominal_offset = (
            c * self.mast_offset_j1_xy_m[0] - s * self.mast_offset_j1_xy_m[1],
            s * self.mast_offset_j1_xy_m[0] + c * self.mast_offset_j1_xy_m[1],
        )
        if abs(nominal_offset[0]) > 1e-9 or abs(nominal_offset[1] - 0.4) > 1e-9:
            raise ValueError("nominal mast must be exactly 0.4 m on world +Y left")
        derived_heading = wrap_to_pi(self.nominal_q1_rad + self.mast_j1_yaw_offset_rad)
        if abs(wrap_to_pi(derived_heading - self.nominal_camera_heading_world_rad)) > 1e-9:
            raise ValueError("nominal camera heading is inconsistent with J1 phase")
        if self.depth_semantics != "optical_z_m":
            raise ValueError("registered depth must use optical_z_m")
        if not self.calibration_identity:
            raise ValueError("RGB-D calibration identity is required")
        if len(self.light_offsets_module_m) != 2:
            raise ValueError("the main module requires exactly two fill lights")
        for offset in self.light_offsets_module_m:
            if len(offset) != 3 or not all(isfinite(float(value)) for value in offset):
                raise ValueError("fill-light offsets must be finite XYZ triples")
        left, right = self.light_offsets_module_m
        if abs(left[0] - right[0]) > 1e-9 or abs(left[1] + right[1]) > 1e-9 or abs(left[2] - right[2]) > 1e-9:
            raise ValueError("fill lights must be left-right symmetric in the module frame")

    @property
    def mast_radius_m(self) -> float:
        return sqrt(sum(float(value) ** 2 for value in self.mast_offset_j1_xy_m))


@dataclass(frozen=True)
class VisionRigPose:
    q1_rad: float
    j1_heading_world_rad: float
    camera_heading_world_rad: float
    T_W_J1: tuple[tuple[float, ...], ...]
    T_W_vision_flange: tuple[tuple[float, ...], ...]
    T_W_mast_top: tuple[tuple[float, ...], ...]
    T_W_module: tuple[tuple[float, ...], ...]
    T_W_camera_optical: tuple[tuple[float, ...], ...]

    @property
    def j1_center_world_m(self) -> tuple[float, float, float]:
        return tuple(row[3] for row in self.T_W_J1[:3])

    @property
    def mast_center_world_m(self) -> tuple[float, float, float]:
        return tuple(row[3] for row in self.T_W_vision_flange[:3])

    @property
    def camera_center_world_m(self) -> tuple[float, float, float]:
        return tuple(row[3] for row in self.T_W_camera_optical[:3])


def evaluate_vision_rig_pose(
    T_W_robot_base: Sequence[Sequence[float]], q1_rad: float, spec: VisionRigSpec
) -> VisionRigPose:
    """Evaluate the full dynamic transform chain at one captured J1 value."""

    base = validate_transform_parent_child(T_W_robot_base)
    q1 = _finite(q1_rad, "q1")
    T_W_J1 = _matmul(
        _matmul(base, _translation(0.0, 0.0, spec.j1_joint_origin_z_from_base_m)),
        _yaw_transform(q1),
    )
    T_J1_flange = _translation(
        spec.mast_offset_j1_xy_m[0], spec.mast_offset_j1_xy_m[1], spec.flange_height_j1_m
    )
    T_W_flange = _matmul(T_W_J1, T_J1_flange)
    T_W_top = _matmul(T_W_flange, _translation(0.0, 0.0, spec.mast_height_m))
    T_W_module = _matmul(T_W_flange, _translation(0.0, 0.0, spec.module_height_m))
    camera_heading = wrap_to_pi(q1 + spec.mast_j1_yaw_offset_rad)
    T_W_camera = _optical_pose(tuple(row[3] for row in T_W_module[:3]), camera_heading)
    for transform in (T_W_J1, T_W_flange, T_W_top, T_W_module, T_W_camera):
        validate_transform_parent_child(transform)
    return VisionRigPose(q1, wrap_to_pi(q1), camera_heading, T_W_J1, T_W_flange, T_W_top, T_W_module, T_W_camera)


def audit_j1_sweep(
    T_W_robot_base: Sequence[Sequence[float]], q1_values_rad: Sequence[float], spec: VisionRigSpec
) -> dict[str, Any]:
    samples = []
    for value in q1_values_rad:
        pose = evaluate_vision_rig_pose(T_W_robot_base, value, spec)
        offset = tuple(pose.mast_center_world_m[index] - pose.j1_center_world_m[index] for index in range(3))
        radius = sqrt(offset[0] ** 2 + offset[1] ** 2)
        phase = wrap_to_pi(pose.camera_heading_world_rad - pose.j1_heading_world_rad)
        camera_rotation = tuple(row[:3] for row in pose.T_W_camera_optical[:3])
        optical_down = tuple(camera_rotation[row][1] for row in range(3))
        samples.append({
            "q1_rad": pose.q1_rad,
            "j1_center_world_m": pose.j1_center_world_m,
            "mast_center_world_m": pose.mast_center_world_m,
            "camera_center_world_m": pose.camera_center_world_m,
            "mast_offset_world_m": offset,
            "mast_radius_m": radius,
            "camera_heading_world_rad": pose.camera_heading_world_rad,
            "j1_heading_world_rad": pose.j1_heading_world_rad,
            "camera_j1_yaw_offset_rad": phase,
            "camera_roll_pitch_level": abs(optical_down[2] + 1.0) <= 1e-9,
            "radius_pass": abs(radius - spec.mast_radius_m) <= 1e-9,
            "yaw_phase_pass": abs(wrap_to_pi(phase - spec.mast_j1_yaw_offset_rad)) <= 1e-9,
        })
    return {
        "schema_version": "j1_mast_camera_kinematics_v2",
        "rig_id": spec.rig_id,
        "parent_frame": spec.parent_frame,
        "flange_height_status": spec.flange_height_status,
        "samples": samples,
        "status": "PASS" if all(
            item["radius_pass"] and item["yaw_phase_pass"] and item["camera_roll_pitch_level"]
            for item in samples
        ) else "FAIL",
    }


def load_vision_rig_spec(path: str | Path) -> VisionRigSpec:
    source = Path(path)
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping) or data.get("schema_version") != "j1_perception_sensing_pose_v2":
        raise ValueError("unsupported J1 perception sensing-pose schema")
    camera = data["module_0_main"]["rgb_camera"]
    depth = data["module_0_main"]["depth_camera"]

    def camera_spec(value: Mapping[str, Any]) -> ExactFovCamera:
        return ExactFovCamera(
            int(value["resolution_px"][0]), int(value["resolution_px"][1]),
            float(value["hfov_rad"]), float(value["vfov_rad"]), float(value["frame_rate_hz"]),
            str(value["intrinsics_mode"]),
        )

    lights = data["module_0_main"]["fill_lights"]
    return VisionRigSpec(
        rig_id=str(data["rig_id"]),
        parent_frame=str(data["kinematics"]["parent_frame"]),
        j1_joint_origin_z_from_base_m=float(data["kinematics"]["j1_joint_origin_z_from_base_m"]),
        mast_offset_j1_xy_m=tuple(float(value) for value in data["kinematics"]["mast_offset_j1_xy_m"]),
        flange_height_j1_m=float(data["kinematics"]["vision_flange_height_j1_m"]),
        flange_height_status=str(data["kinematics"]["vision_flange_height_status"]),
        mast_height_m=float(data["mast"]["height_from_flange_m"]),
        module_height_m=float(data["module_0_main"]["height_from_flange_m"]),
        mast_j1_yaw_offset_rad=float(data["kinematics"]["mast_j1_yaw_offset_rad"]),
        nominal_q1_rad=float(data["nominal_sensing_pose"]["q1_rad"]),
        nominal_camera_heading_world_rad=float(data["nominal_sensing_pose"]["camera_heading_world_rad"]),
        independent_mast_yaw=bool(data["mast_yaw_actuation"]["supported"]),
        geometry_qualification=str(data["geometry_qualification"]),
        rgb=camera_spec(camera),
        depth=camera_spec(depth),
        depth_semantics=str(depth["depth_semantics"]),
        calibration_identity=str(data["module_0_main"]["calibration_identity"]),
        light_offsets_module_m=tuple(tuple(float(number) for number in item["relative_xyz_m"]) for item in lights),
    )


def write_kinematics_audit(
    path: str | Path, T_W_robot_base: Sequence[Sequence[float]], spec: VisionRigSpec
) -> None:
    q1_values = tuple(value * pi / 180.0 for value in (-90.0, -45.0, 0.0, 45.0, 90.0))
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(audit_j1_sweep(T_W_robot_base, q1_values, spec), indent=2, sort_keys=True),
        encoding="utf-8",
    )
