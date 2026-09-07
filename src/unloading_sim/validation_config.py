"""Strict, source-traced acceptance configuration, independent of simulator UI."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .geometry import make_transform, rotation_matrix_from_rpy
from .identity import load_tool_config
from .robot import URDFRobot
from .timing import JointMotionLimits

DEFAULT = Path(__file__).resolve().parents[2] / "configs/validation/m710id70_v3.yaml"


def _read(path: Path, stack: tuple[Path, ...] = ()) -> tuple[dict, dict]:
    path = path.resolve()
    if path in stack:
        raise ValueError(f"configuration inheritance cycle: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("configuration must be a mapping")
    result, sources = ({}, {}) if "extends" not in raw else _read(path.parent / raw["extends"], (*stack, path))

    def merge(dst: dict, src: dict, prefix: str = "") -> None:
        for key, value in src.items():
            if key == "extends":
                continue
            dotted = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                if not isinstance(dst.get(key), dict):
                    dst[key] = {}
                merge(dst[key], value, dotted)
            else:
                dst[key] = value
                sources[dotted] = str(path)
    merge(result, raw)
    return result, sources


@dataclass
class ValidationConfig:
    data: dict[str, Any]
    sources: dict[str, str]
    model: dict
    tool: Any
    model_path: Path
    urdf_path: Path

    def world_from_chassis(self) -> np.ndarray:
        cfg = self.data["chassis"]
        return make_transform(rotation_matrix_from_rpy(*cfg["world_rpy_rad"]), cfg["world_xyz_m"])

    def robot(self, height: float | None = None) -> URDFRobot:
        d = self.data
        lift = d["lift"]["height_m"] if height is None else height
        install = make_transform(rotation_matrix_from_rpy(*d["robot"]["installation_rpy_rad"]), d["robot"]["installation_xyz_m"])
        base = self.world_from_chassis() @ make_transform(translation=[0, 0, lift]) @ install
        # Geometry dimensions in the planner frame: short, long, normal.
        outer = np.asarray(self.tool.data["geometry"]["outer_size_m"], float)[[1, 0, 2]]
        robot = URDFRobot.fanuc_m710id_70(urdf_path=self.urdf_path, tool_length=self.tool.tcp_translation_xyz_m[0],
                                        tool_collision_size=outer, tool_collision_center_offset=outer[2] / 2)
        robot.base_transform = base
        mechanical_tcp = make_transform(np.asarray(self.tool.tcp_rotation_matrix), self.tool.tcp_translation_xyz_m)
        task_axes = make_transform(rotation_matrix_from_rpy(0, np.pi / 2, 0))
        frames = robot.named_link_frames(np.zeros(6))
        flange_from_tip = np.linalg.inv(frames["flange"]) @ frames["tool0"]
        robot.tip_from_tcp = np.linalg.inv(flange_from_tip) @ mechanical_tcp @ task_axes
        return robot

    def motion_limits(self) -> JointMotionLimits:
        e = self.data["execution"]
        scales = e["profiles"][e["profile"]]
        return JointMotionLimits(
            np.asarray(self.model["joint_velocity_limits_rad_s"]) * scales["velocity_scale"],
            np.asarray(self.model["joint_acceleration_limits_rad_s2"]["values"]) * scales["acceleration_scale"],
            np.asarray(self.model["joint_jerk_limits_rad_s3"]["values"]) * scales["jerk_scale"],
            minimum_segment_seconds=e["minimum_segment_seconds"], source=str(self.model_path))

    def evidence(self) -> dict:
        robot = self.robot()
        return {"effective_config": self.data, "parameter_sources": self.sources,
                "robot_limits": self.model, "robot_limits_source": str(self.model_path),
                "tool": self.tool.to_mapping(), "urdf_path": str(self.urdf_path),
                "world_from_chassis": self.world_from_chassis().tolist(),
                "world_from_robot_mount": robot.base_transform.tolist(),
                "tool0_from_task_tcp": robot.tip_from_tcp.tolist(),
                "collision_tool_size_task_xyz_m": robot.tool_collision_size.tolist(),
                "motion_limits": {"velocity": self.motion_limits().effective_velocity.tolist(),
                                  "acceleration": self.motion_limits().effective_acceleration.tolist(),
                                  "jerk": self.motion_limits().effective_jerk.tolist()}}


def load_validation_config(path: str | Path = DEFAULT) -> ValidationConfig:
    data, sources = _read(Path(path))
    template, allowed = _read(DEFAULT)
    if data.get("schema") != "m710_validation_v3":
        raise ValueError("unsupported validation configuration schema")
    unknown = set(sources) - set(allowed)
    missing = set(allowed) - set(sources)
    if unknown or missing:
        raise ValueError(f"unknown parameters: {sorted(unknown)}; missing parameters: {sorted(missing)}")
    def resource(key: str, value: str) -> Path:
        return (Path(sources[key]).parent / value).resolve()
    model_path = resource("robot.model_config", data["robot"]["model_config"])
    model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    tool = load_tool_config(resource("tool.config", data["tool"]["config"]))
    if tool.mass_properties_reference_frame != "flange":
        raise ValueError("tool mass properties must be expressed in flange coordinates")
    result = ValidationConfig(data, sources, model, tool, model_path,
                              resource("robot.urdf_path", data["robot"]["urdf_path"]))
    result.motion_limits()  # Validate the selected execution profile now.
    robot = result.robot()
    limits = np.asarray([model["joint_position_limits_rad"][name] for name in robot.active_joint_names])
    if not np.allclose(robot.joint_limits, limits, atol=1e-9, rtol=0):
        raise ValueError("URDF and robot model joint limits disagree")
    return result
