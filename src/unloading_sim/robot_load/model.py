"""Legacy first-layer FANUC wrist load checks.

All vectors use metres and the flange frame.  A box uses local axes
``[depth, width, height]``.  ``grasp_point_xyz_m`` and
``estimated_com_xyz_m`` share that box frame, so their difference can be
rotated to the flange frame without guessing an absolute box origin.

``flange_axis_engineering_estimate_v1`` is retained only for reproducibility
and is ``MODEL_SUPERSEDED``.  It assumes flange +X/+Y/+Z are J4/J5/J6,
respectively.  It forms a full inertia tensor at the flange, but then uses its
X/Y/Z diagonal entries as joint inertias.  Its gravity term is the unsigned
worst-direction scalar ``m*g*r_perpendicular`` rather than the projection of
``r cross F_g`` on a URDF/FK joint axis.  Finally, it places the box from the
TCP label plus a grasp offset.  These assumptions are intentionally documented
here instead of changing v1 numbers; new conclusions must use
``spatial_inertia_joint_axis_v2`` from :mod:`unloading_sim.robot_load.spatial`.

The v1 result strings historically treated the published FANUC moment and
inertia numbers as hard limits.  They are retained only to reproduce old
artifacts and must not be interpreted as a current vendor qualification.  The
v2 evidence model reclassifies those comparisons as engineering references
and keeps the FANUC reference/diagram fail-closed ``NOT_EVALUATED``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from unloading_sim.identity import load_tool_config, normalize_robot_model_id


GRAVITY_M_S2 = 9.80665
AXIS_NAMES = ("J4", "J5", "J6")


class Qualification(str, Enum):
    PASS = "PASS"
    PASS_DERATED = "PASS_DERATED"
    FAIL_PAYLOAD = "FAIL_PAYLOAD"
    FAIL_COM = "FAIL_COM"
    FAIL_WRIST_MOMENT = "FAIL_WRIST_MOMENT"
    FAIL_WRIST_INERTIA = "FAIL_WRIST_INERTIA"
    NOT_EVALUATED = "NOT_EVALUATED"


class ToolMassPropertiesSource(str, Enum):
    """Traceability level for tool mass, CoM, and inertia inputs."""

    CAD = "CAD"
    MEASURED = "MEASURED"
    ENGINEERING_MODEL = "ENGINEERING_MODEL"


def _vector3(value: Sequence[float], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite SI values")
    return result


def _inertia3(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 tensor in kg*m^2")
    if not np.allclose(result, result.T, atol=1e-10):
        raise ValueError(f"{name} must be symmetric")
    if np.min(np.linalg.eigvalsh(result)) < -1e-10:
        raise ValueError(f"{name} must be positive semidefinite")
    return result


def _positive(value: float, name: str, *, allow_zero: bool = False) -> float:
    result = float(value)
    valid = np.isfinite(result) and (result >= 0.0 if allow_zero else result > 0.0)
    if not valid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


@dataclass(frozen=True)
class RobotLoadLimits:
    model: str
    rated_payload_kg: float
    reach_m: float
    joint_speed_rad_s: tuple[float, float, float, float, float, float]
    allowable_moment_nm: tuple[float, float, float]
    allowable_inertia_kg_m2: tuple[float, float, float]
    normal_utilization_limit: float
    com_limit_status: str
    com_limit_reason: str
    source: Mapping[str, Any]
    urdf_path: Path | None = None
    base_link: str = "base_link"
    flange_link: str = "flange"
    tip_link: str = "tool0"
    active_joint_names: tuple[str, ...] = ("J1", "J2", "J3", "J4", "J5", "J6")
    qualification_joint_pose_q: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        _positive(self.rated_payload_kg, "rated_payload_kg")
        _positive(self.reach_m, "reach_m")
        if len(self.joint_speed_rad_s) != 6:
            raise ValueError("joint_speed_rad_s must contain J1..J6")
        if any(not np.isfinite(v) or v <= 0.0 for v in self.joint_speed_rad_s):
            raise ValueError("joint speeds must be finite and positive")
        if any(not np.isfinite(v) or v <= 0.0 for v in self.allowable_moment_nm):
            raise ValueError("J4..J6 moment limits must be finite and positive")
        if any(not np.isfinite(v) or v <= 0.0 for v in self.allowable_inertia_kg_m2):
            raise ValueError("J4..J6 inertia limits must be finite and positive")
        if not 0.0 < self.normal_utilization_limit <= 1.0:
            raise ValueError("normal_utilization_limit must be in (0, 1]")
        if self.qualification_joint_pose_q and len(self.qualification_joint_pose_q) != len(self.active_joint_names):
            raise ValueError("qualification_joint_pose_q must match active_joint_names")


@dataclass(frozen=True)
class ToolLoad:
    mass_kg: float
    com_xyz_m: np.ndarray
    inertia_at_com_kg_m2: np.ndarray
    tcp_xyz_m: np.ndarray
    source: Mapping[str, Any]
    mass_properties_source: ToolMassPropertiesSource = ToolMassPropertiesSource.ENGINEERING_MODEL
    mass_properties_reference_frame: str = "flange"

    def __post_init__(self) -> None:
        object.__setattr__(self, "mass_kg", _positive(self.mass_kg, "tool.mass_kg"))
        object.__setattr__(self, "com_xyz_m", _vector3(self.com_xyz_m, "tool.com_xyz_m"))
        object.__setattr__(self, "inertia_at_com_kg_m2", _inertia3(self.inertia_at_com_kg_m2, "tool.inertia_at_com_kg_m2"))
        object.__setattr__(self, "tcp_xyz_m", _vector3(self.tcp_xyz_m, "tool.tcp_xyz_m"))
        object.__setattr__(
            self,
            "mass_properties_source",
            ToolMassPropertiesSource(self.mass_properties_source),
        )
        if not str(self.mass_properties_reference_frame).strip():
            raise ValueError("tool.mass_properties_reference_frame must be non-empty")

    @property
    def inertia_evidence_status(self) -> str:
        if self.mass_properties_source is ToolMassPropertiesSource.ENGINEERING_MODEL:
            return "TOOL_INERTIA_ENGINEERING_ASSUMPTION"
        return f"TOOL_INERTIA_{self.mass_properties_source.value}"

    @property
    def inertia_tensor_com_kg_m2(self) -> np.ndarray:
        """Explicit v2 name for the physical tensor about ``C_tool``.

        The legacy field spelling remains available so v1 artifacts can be
        reproduced without rewriting their inputs.
        """
        return self.inertia_at_com_kg_m2


@dataclass(frozen=True)
class BoxLoad:
    size_xyz_m: np.ndarray
    mass_kg: float
    grasp_face: str
    grasp_point_xyz_m: np.ndarray
    estimated_com_xyz_m: np.ndarray

    def __post_init__(self) -> None:
        size = _vector3(self.size_xyz_m, "box.size_xyz_m")
        if np.any(size <= 0.0):
            raise ValueError("box dimensions must be positive")
        if self.grasp_face not in {"front", "side", "front_center", "front_offset", "side_center"}:
            raise ValueError(f"unsupported grasp face {self.grasp_face!r}")
        object.__setattr__(self, "size_xyz_m", size)
        object.__setattr__(self, "mass_kg", _positive(self.mass_kg, "box.mass_kg"))
        object.__setattr__(self, "grasp_point_xyz_m", _vector3(self.grasp_point_xyz_m, "box.grasp_point_xyz_m"))
        object.__setattr__(self, "estimated_com_xyz_m", _vector3(self.estimated_com_xyz_m, "box.estimated_com_xyz_m"))


@dataclass(frozen=True)
class MotionLoad:
    linear_acceleration_m_s2: float
    angular_acceleration_rad_s2: float
    safety_factor: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "linear_acceleration_m_s2", _positive(self.linear_acceleration_m_s2, "motion.linear_acceleration_m_s2", allow_zero=True))
        object.__setattr__(self, "angular_acceleration_rad_s2", _positive(self.angular_acceleration_rad_s2, "motion.angular_acceleration_rad_s2", allow_zero=True))
        if not np.isfinite(self.safety_factor) or self.safety_factor < 1.0:
            raise ValueError("motion.safety_factor must be finite and >= 1")


@dataclass(frozen=True)
class LoadCase:
    robot: RobotLoadLimits
    tool: ToolLoad
    box: BoxLoad
    motion: MotionLoad


@dataclass(frozen=True)
class LoadQualificationResult:
    qualification: str
    reasons: tuple[str, ...]
    criteria: Mapping[str, str]
    intermediate: Mapping[str, Any]
    sources: Mapping[str, Any]

    def to_dict(self) -> dict[str, Any]:
        def convert(value: Any) -> Any:
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, (np.floating, np.integer)):
                return value.item()
            if isinstance(value, Mapping):
                return {str(k): convert(v) for k, v in value.items()}
            if isinstance(value, (tuple, list)):
                return [convert(v) for v in value]
            return value

        return convert(asdict(self))


def cuboid_inertia_at_com(mass_kg: float, size_xyz_m: Sequence[float]) -> np.ndarray:
    """Return a solid cuboid inertia tensor about its centre."""
    mass = _positive(mass_kg, "mass_kg")
    x, y, z = _vector3(size_xyz_m, "size_xyz_m")
    if min(x, y, z) <= 0.0:
        raise ValueError("cuboid dimensions must be positive")
    return np.diag(
        [mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0, mass * (x * x + y * y) / 12.0]
    )


def parallel_axis(inertia_at_com: np.ndarray, mass_kg: float, offset_xyz_m: Sequence[float]) -> np.ndarray:
    """Translate an inertia tensor using the parallel-axis theorem."""
    inertia = _inertia3(inertia_at_com, "inertia_at_com")
    offset = _vector3(offset_xyz_m, "offset_xyz_m")
    return inertia + float(mass_kg) * ((offset @ offset) * np.eye(3) - np.outer(offset, offset))


def _box_rotation(grasp_face: str) -> np.ndarray:
    if grasp_face.startswith("front") or grasp_face == "front":
        return np.eye(3)
    # Box +Y (side inward normal) -> flange +X; preserve a right-handed frame.
    return np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def qualify_load(case: LoadCase) -> LoadQualificationResult:
    """Evaluate payload, CoM, worst-orientation wrist moment and inertia.

    Moment demand is an auditable conservative engineering estimate on flange
    +X/+Y/+Z proxy axes, compared in J4/J5/J6 order:
    ``m * (g + a_linear) * r_perp + I_axis * alpha``.  It is not a
    controller- or pose-specific inverse-dynamics certification.
    """
    robot, tool, box, motion = case.robot, case.tool, case.box, case.motion
    rotation = _box_rotation(box.grasp_face)
    box_com = tool.tcp_xyz_m + rotation @ (box.estimated_com_xyz_m - box.grasp_point_xyz_m)
    box_inertia_com = rotation @ cuboid_inertia_at_com(box.mass_kg, box.size_xyz_m) @ rotation.T
    total_mass = tool.mass_kg + box.mass_kg
    combined_com = (tool.mass_kg * tool.com_xyz_m + box.mass_kg * box_com) / total_mass
    tool_inertia_flange = parallel_axis(tool.inertia_at_com_kg_m2, tool.mass_kg, tool.com_xyz_m)
    box_inertia_flange = parallel_axis(box_inertia_com, box.mass_kg, box_com)
    combined_inertia = tool_inertia_flange + box_inertia_flange
    effective_mass = total_mass * motion.safety_factor
    payload_util = effective_mass / robot.rated_payload_kg

    moments: list[float] = []
    static_moments: list[float] = []
    dynamic_linear_moments: list[float] = []
    dynamic_angular_moments: list[float] = []
    inertias: list[float] = []
    for index in range(3):
        radius_perpendicular = float(np.linalg.norm(np.delete(combined_com, index)))
        inertia_axis = float(combined_inertia[index, index])
        static = total_mass * GRAVITY_M_S2 * radius_perpendicular
        linear = total_mass * motion.linear_acceleration_m_s2 * radius_perpendicular
        angular = inertia_axis * motion.angular_acceleration_rad_s2
        static_moments.append(static)
        dynamic_linear_moments.append(linear)
        dynamic_angular_moments.append(angular)
        moments.append((static + linear + angular) * motion.safety_factor)
        inertias.append(inertia_axis * motion.safety_factor)

    moment_util = np.asarray(moments) / np.asarray(robot.allowable_moment_nm)
    inertia_util = np.asarray(inertias) / np.asarray(robot.allowable_inertia_kg_m2)
    criteria = {
        "payload": "PASS" if payload_util <= 1.0 + 1e-12 else "FAIL",
        "com": robot.com_limit_status,
        "wrist_moment": "PASS" if np.max(moment_util) <= 1.0 + 1e-12 else "FAIL",
        "wrist_inertia": "PASS" if np.max(inertia_util) <= 1.0 + 1e-12 else "FAIL",
    }
    reasons: list[str] = []
    if criteria["payload"] == "FAIL":
        qualification = Qualification.FAIL_PAYLOAD.value
        reasons.append(f"safety-factored external mass {effective_mass:.3f} kg exceeds {robot.rated_payload_kg:.3f} kg")
    elif criteria["wrist_moment"] == "FAIL":
        qualification = Qualification.FAIL_WRIST_MOMENT.value
        axis = AXIS_NAMES[int(np.argmax(moment_util))]
        reasons.append(f"{axis} estimated moment utilization is {float(np.max(moment_util)):.3f}")
    elif criteria["wrist_inertia"] == "FAIL":
        qualification = Qualification.FAIL_WRIST_INERTIA.value
        axis = AXIS_NAMES[int(np.argmax(inertia_util))]
        reasons.append(f"{axis} inertia utilization is {float(np.max(inertia_util)):.3f}")
    elif criteria["com"] != "PASS":
        qualification = Qualification.NOT_EVALUATED.value
        reasons.append(robot.com_limit_reason)
    else:
        maximum = max(payload_util, float(np.max(moment_util)), float(np.max(inertia_util)))
        if maximum > robot.normal_utilization_limit:
            qualification = Qualification.PASS_DERATED.value
            reasons.append(f"maximum known-limit utilization {maximum:.3f} exceeds normal threshold {robot.normal_utilization_limit:.3f}")
        else:
            qualification = Qualification.PASS.value
            reasons.append("all configured manufacturer criteria pass within the normal utilization threshold")

    intermediate = {
        "method": "flange_axis_engineering_estimate_v1",
        "model_status": "MODEL_SUPERSEDED",
        "axis_proxy_J4_J5_J6": ["flange_+X", "flange_+Y", "flange_+Z"],
        "total_external_mass_kg": total_mass,
        "safety_factored_external_mass_kg": effective_mass,
        "payload_utilization": payload_util,
        "tool_com_flange_xyz_m": tool.com_xyz_m,
        "box_com_flange_xyz_m": box_com,
        "combined_com_flange_xyz_m": combined_com,
        "box_inertia_at_com_kg_m2": box_inertia_com,
        "tool_inertia_at_flange_kg_m2": tool_inertia_flange,
        "box_inertia_at_flange_kg_m2": box_inertia_flange,
        "combined_inertia_at_flange_kg_m2": combined_inertia,
        "static_gravity_moment_nm_J4_J5_J6": static_moments,
        "dynamic_linear_moment_nm_J4_J5_J6": dynamic_linear_moments,
        "dynamic_angular_moment_nm_J4_J5_J6": dynamic_angular_moments,
        "safety_factored_moment_nm_J4_J5_J6": moments,
        "safety_factored_inertia_kg_m2_J4_J5_J6": inertias,
        "moment_utilization_J4_J5_J6": moment_util,
        "inertia_utilization_J4_J5_J6": inertia_util,
        "maximum_known_utilization": max(payload_util, float(np.max(moment_util)), float(np.max(inertia_util))),
        "safety_factor": motion.safety_factor,
    }
    return LoadQualificationResult(
        qualification=qualification,
        reasons=tuple(reasons),
        criteria=criteria,
        intermediate=intermediate,
        sources={"robot": dict(robot.source), "tool": dict(tool.source)},
    )


def _read_yaml(path: str | Path) -> tuple[dict[str, Any], Path]:
    resolved = Path(path).resolve()
    data = yaml.safe_load(resolved.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{resolved} must contain a YAML mapping")
    return data, resolved


def load_robot_limits(path: str | Path) -> RobotLoadLimits:
    data, resolved = _read_yaml(path)
    wrist = data["wrist_limits"]
    moment = wrist["allowable_moment_nm"]
    inertia = wrist["allowable_inertia_kg_m2"]
    com = data["payload_com_limits"]
    kinematics = data.get("kinematics", {})
    urdf_value = kinematics.get("urdf_path")
    urdf_path = None
    if urdf_value is not None:
        urdf_path = Path(str(urdf_value))
        if not urdf_path.is_absolute():
            urdf_path = (resolved.parent / urdf_path).resolve()
    return RobotLoadLimits(
        model=normalize_robot_model_id(data["model"]),
        rated_payload_kg=float(data["rated_payload_kg"]),
        reach_m=float(data["reach_m"]),
        joint_speed_rad_s=tuple(float(v) for v in data["joint_speed_rad_s"]),  # type: ignore[arg-type]
        allowable_moment_nm=tuple(float(moment[name]) for name in AXIS_NAMES),  # type: ignore[arg-type]
        allowable_inertia_kg_m2=tuple(float(inertia[name]) for name in AXIS_NAMES),  # type: ignore[arg-type]
        normal_utilization_limit=float(data["normal_utilization_limit"]),
        com_limit_status=str(com["status"]),
        com_limit_reason=str(com["reason"]),
        source=dict(data["source"]),
        urdf_path=urdf_path,
        base_link=str(kinematics.get("base_link", "base_link")),
        flange_link=str(kinematics.get("flange_link", "flange")),
        tip_link=str(kinematics.get("tip_link", "tool0")),
        active_joint_names=tuple(
            str(v)
            for v in kinematics.get("active_joint_names", ("J1", "J2", "J3", "J4", "J5", "J6"))
        ),
        qualification_joint_pose_q=tuple(float(v) for v in kinematics.get("qualification_joint_pose_q", ())),
    )


def load_tool(path: str | Path) -> ToolLoad:
    config = load_tool_config(path)
    return ToolLoad(
        mass_kg=config.mass_kg,
        com_xyz_m=np.asarray(config.com_xyz_m, dtype=float),
        inertia_at_com_kg_m2=np.asarray(config.inertia_tensor_com_kg_m2, dtype=float),
        tcp_xyz_m=np.asarray(config.tcp_translation_xyz_m, dtype=float),
        source={
            **dict(config.source),
            "tool_name": config.name,
            "resolved_config_path": str(config.config_path),
            "config_sha256": config.config_hash,
            "tcp_transform": config.tcp_transform,
        },
        mass_properties_source=ToolMassPropertiesSource(config.mass_properties_source),
        mass_properties_reference_frame=config.mass_properties_reference_frame,
    )


def load_case_from_mapping(data: Mapping[str, Any], robot: RobotLoadLimits, tool: ToolLoad) -> LoadCase:
    box = data["box"]
    motion = data["motion"]
    return LoadCase(
        robot=robot,
        tool=tool,
        box=BoxLoad(
            size_xyz_m=np.asarray(box["size_xyz_m"], dtype=float),
            mass_kg=float(box["mass_kg"]),
            grasp_face=str(box["grasp_face"]),
            grasp_point_xyz_m=np.asarray(box["grasp_point_xyz_m"], dtype=float),
            estimated_com_xyz_m=np.asarray(box["estimated_com_xyz_m"], dtype=float),
        ),
        motion=MotionLoad(
            linear_acceleration_m_s2=float(motion["linear_acceleration_m_s2"]),
            angular_acceleration_rad_s2=float(motion["angular_acceleration_rad_s2"]),
            safety_factor=float(motion["safety_factor"]),
        ),
    )


def load_qualification_case(path: str | Path) -> LoadCase:
    """Load one complete, SI-explicit qualification case from YAML."""
    data, resolved = _read_yaml(path)
    if data.get("schema_version") != "robot_load_case_v1":
        raise ValueError("unsupported robot load case schema")
    robot_data = data["robot"]
    robot_config = Path(str(robot_data["limits_config"]))
    if not robot_config.is_absolute():
        robot_config = (resolved.parent / robot_config).resolve()
    robot = load_robot_limits(robot_config)
    if normalize_robot_model_id(robot_data["model"]) != robot.model:
        raise ValueError("load-case robot model does not match the limits config")
    tool_data = data["tool"]
    tool_config = Path(str(tool_data["config"]))
    if not tool_config.is_absolute():
        tool_config = (resolved.parent / tool_config).resolve()
    tool = load_tool(tool_config)
    return load_case_from_mapping(data, robot, tool)
