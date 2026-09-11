"""Strict CPU-side dynamics contract for the FANUC M-710iD/70 workcell.

The module deliberately contains no simulator-backend imports. It selects the
official public model by default and retains explicit historical proxy-estimate
validation. Both loaders bind layout/model/tool sources and expose immutable
SI-unit values to replay adapters, independently of machine qualification.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from types import MappingProxyType
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .identity import load_tool_config, sha256_file


SCHEMA_VERSION = "m710id70_engineering_dynamics_v1"
ENGINEERING_STATUS = "ENGINEERING_ESTIMATE_NOT_MACHINE_QUALIFIED"
FIXED_TOOL_MASS_POLICY = "FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE"
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "configs"
    / "simulation"
    / "m710id70_official_dynamics_v2.yaml"
)
ROBOT_LINK_NAMES = ("base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link")
ACTIVE_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
CONTACT_MATERIAL_NAMES = ("carton", "painted_steel", "conveyor_belt", "robot_coating")
DAMPED_BODY_GROUPS = ("robot_links", "tool", "cartons")
CONVEYOR_SURFACE_NAMES = ("conveyor_transverse", "conveyor_longitudinal")
EXPECTED_CONVEYOR_DIRECTIONS = {
    "conveyor_transverse": (0.0, -1.0, 0.0),
    "conveyor_longitudinal": (-1.0, 0.0, 0.0),
}


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader which refuses silently overwritten mapping keys."""


def _construct_unique_mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False) -> dict:
    loader.flatten_mapping(node)
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f"duplicate YAML key {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _read_yaml(path: Path, label: str) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"{label} is not a readable file: {path}")
    try:
        data = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read {label}: {path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"{label} must contain a YAML mapping")
    return data


def _mapping(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _strict_keys(value: Mapping[str, Any], expected: set[str], name: str) -> None:
    actual = set(value)
    missing = sorted(expected - actual)
    unexpected = sorted(actual - expected)
    if missing or unexpected:
        details: list[str] = []
        if missing:
            details.append(f"missing {missing}")
        if unexpected:
            details.append(f"unexpected {unexpected}")
        raise ValueError(f"{name} has invalid fields: {', '.join(details)}")


def _text(value: object, name: str) -> str:
    result = str(value).strip() if value is not None else ""
    if not result:
        raise ValueError(f"{name} must be non-empty")
    return result


def _status(value: object, name: str) -> str:
    result = _text(value, name)
    if result != ENGINEERING_STATUS:
        raise ValueError(f"{name} must be {ENGINEERING_STATUS}")
    return result


def _boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite SI value")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite SI value") from exc
    if not np.isfinite(result):
        raise ValueError(f"{name} must be a finite SI value")
    return result


def _positive(value: object, name: str) -> float:
    result = _finite(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative(value: object, name: str) -> float:
    result = _finite(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _positive_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or int(value) <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def _vector(value: object, length: int, name: str) -> tuple[float, ...]:
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain {length} finite SI values") from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite SI values")
    return tuple(float(item) for item in result)


def _inertia(value: object, name: str) -> tuple[tuple[float, float, float], ...]:
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite 3x3 inertia tensor") from exc
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite 3x3 inertia tensor")
    if not np.allclose(matrix, matrix.T, atol=1e-12, rtol=0.0):
        raise ValueError(f"{name} must be symmetric")
    principal = np.linalg.eigvalsh(matrix)
    if float(principal[0]) <= 0.0:
        raise ValueError(f"{name} must be positive definite")
    tolerance = 1e-10 * max(1.0, float(principal[-1]))
    if float(principal[-1]) > float(principal[0] + principal[1]) + tolerance:
        raise ValueError(f"{name} principal moments violate the rigid-body triangle inequality")
    return tuple(tuple(float(item) for item in row) for row in matrix)  # type: ignore[return-value]


def _resolve(base: Path, value: object, name: str) -> Path:
    text = _text(value, name)
    path = Path(text)
    resolved = path.resolve() if path.is_absolute() else (base / path).resolve()
    if not resolved.is_file():
        raise ValueError(f"{name} is not a readable file: {resolved}")
    return resolved


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list | tuple):
        return tuple(_freeze(item) for item in value)
    return value


def _thaw(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw(item) for item in value]
    return value


def _canonical_hash(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _rpy_rotation(rpy: tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _xml_vector(value: str | None, name: str) -> tuple[float, float, float]:
    raw = "0 0 0" if value is None else value
    return _vector(raw.split(), 3, name)  # type: ignore[return-value]


def _urdf_proxy_properties(
    urdf_path: Path, mechanical_weight_kg: float
) -> dict[str, tuple[float, tuple[float, float, float], np.ndarray]]:
    """Derive the documented solid-proxy mass distribution from the URDF."""
    root = ET.parse(urdf_path).getroot()
    primitives: dict[str, tuple[float, tuple[float, float, float], np.ndarray, np.ndarray]] = {}
    for name in ROBOT_LINK_NAMES:
        link = root.find(f"./link[@name='{name}']")
        if link is None:
            raise ValueError(f"robot URDF is missing link {name}")
        collisions = link.findall("collision")
        if len(collisions) != 1:
            raise ValueError(f"robot link {name} must have one traceable primitive collision envelope")
        collision = collisions[0]
        origin = collision.find("origin")
        com = _xml_vector(None if origin is None else origin.attrib.get("xyz"), f"URDF {name} collision xyz")
        rpy = _xml_vector(None if origin is None else origin.attrib.get("rpy"), f"URDF {name} collision rpy")
        rotation = _rpy_rotation(rpy)
        geometry = collision.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise ValueError(f"robot link {name} collision must contain one primitive geometry")
        shape = geometry[0]
        if shape.tag == "box":
            size = _xml_vector(shape.attrib.get("size"), f"URDF {name} box size")
            if any(value <= 0.0 for value in size):
                raise ValueError(f"URDF {name} box size must be positive")
            x, y, z = size
            volume = x * y * z
            inertia_per_kg = np.diag([(y * y + z * z) / 12.0, (x * x + z * z) / 12.0, (x * x + y * y) / 12.0])
        elif shape.tag == "cylinder":
            radius = _positive(shape.attrib.get("radius"), f"URDF {name} cylinder radius")
            length = _positive(shape.attrib.get("length"), f"URDF {name} cylinder length")
            volume = float(np.pi * radius * radius * length)
            transverse = (3.0 * radius * radius + length * length) / 12.0
            inertia_per_kg = np.diag([transverse, transverse, 0.5 * radius * radius])
        elif shape.tag == "sphere":
            radius = _positive(shape.attrib.get("radius"), f"URDF {name} sphere radius")
            volume = float(4.0 * np.pi * radius**3 / 3.0)
            inertia_per_kg = np.eye(3) * (0.4 * radius * radius)
        else:
            raise ValueError(f"robot link {name} collision is not a supported traceable primitive")
        primitives[name] = (volume, com, rotation, inertia_per_kg)
    total_volume = sum(item[0] for item in primitives.values())
    if not np.isfinite(total_volume) or total_volume <= 0.0:
        raise ValueError("robot URDF collision proxy volume must be finite and positive")
    result: dict[str, tuple[float, tuple[float, float, float], np.ndarray]] = {}
    for name, (volume, com, rotation, inertia_per_kg) in primitives.items():
        mass = mechanical_weight_kg * volume / total_volume
        inertia = mass * (rotation @ inertia_per_kg @ rotation.T)
        result[name] = (mass, com, inertia)
    return result


@dataclass(frozen=True)
class RigidBodyInertial:
    name: str
    mass_kg: float
    com_xyz_m: tuple[float, float, float]
    inertia_tensor_com_kg_m2: tuple[tuple[float, float, float], ...]
    estimate_method: str


@dataclass(frozen=True)
class JointDrive:
    name: str
    effort_limit_nm: float
    velocity_limit_rad_s: float
    stiffness_nm_rad: float
    damping_nm_s_rad: float


@dataclass(frozen=True)
class ToolDynamics:
    name: str
    mass_kg: float
    com_xyz_m: tuple[float, float, float]
    inertia_tensor_com_kg_m2: tuple[tuple[float, float, float], ...]
    attach_to_link: str
    body_mode: str
    source_status: str
    mass_properties_source: str
    machine_qualified: bool
    config_path: Path
    config_hash: str


@dataclass(frozen=True)
class CartonDynamics:
    count: int
    mass_kg_each: float
    size_xyz_m: tuple[float, float, float]
    com_xyz_m: tuple[float, float, float]
    inertia_tensor_com_kg_m2: tuple[tuple[float, float, float], ...]

    @property
    def inventory_mass_kg(self) -> float:
        return self.count * self.mass_kg_each


@dataclass(frozen=True)
class SimulationSettings:
    gravity_world_m_s2: tuple[float, float, float]
    physics_time_step_s: float
    solver_position_iterations: int
    solver_velocity_iterations: int
    execution_backend: str = "physx_cpu"
    device: str = "cpu"
    broadphase_type: str = "MBP"
    gpu_dynamics_enabled: bool = False
    fabric_enabled: bool = True
    ccd_enabled: bool = True
    contact_offset_m: float = 0.010
    rest_offset_m: float = 0.0


@dataclass(frozen=True)
class PhysicalEnvironment:
    floor_z_m: float
    right_wall_y_m: float
    left_wall_y_m: float
    floor_material: str
    side_wall_material: str
    side_wall_extent_status: str


@dataclass(frozen=True)
class ContactMaterial:
    name: str
    static_friction: float
    dynamic_friction: float
    restitution: float


@dataclass(frozen=True)
class RigidBodyDamping:
    body_group: str
    linear_damping_s_inv: float
    angular_damping_s_inv: float


@dataclass(frozen=True)
class SettlingCriteria:
    maximum_settle_time_s: float
    required_stable_duration_s: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_position_drift_m: float
    max_penetration_m: float


@dataclass(frozen=True)
class ConveyorSurfaceMotion:
    component_name: str
    enabled: bool
    direction_world: tuple[float, float, float]
    speed_m_s: float
    material: str


@dataclass(frozen=True)
class VacuumAttachment:
    product_model: str
    cup_model: str
    physical_cup_count: int
    cup_rows: int
    cup_columns: int
    cup_pitch_m: tuple[float, float]
    cup_radius_m: float
    zone_count: int
    cups_per_zone: tuple[int, ...]
    cup_compression_m: float
    pull_off_force_per_cup_n: float
    shear_force_per_cup_n: float
    hardware_maximum_pull_off_force_n: float
    hardware_maximum_shear_force_n: float
    force_input_source: str
    constraint_type: str
    max_attachment_gap_m: float
    max_normal_misalignment_rad: float
    linear_stiffness_n_m: float
    linear_damping_n_s_m: float
    angular_stiffness_nm_rad: float
    angular_damping_nm_s_rad: float
    break_force_n: float
    break_force_policy: str
    break_torque_nm: float


@dataclass(frozen=True)
class M710EngineeringDynamicsConfig:
    config_path: Path
    qualification_status: str
    machine_qualified: bool
    mechanical_weight_kg: float
    manufacturer_evidence: Mapping[str, Any]
    robot_links: Mapping[str, RigidBodyInertial]
    joint_drives: Mapping[str, JointDrive]
    tool: ToolDynamics
    cartons: CartonDynamics
    simulation: SimulationSettings
    environment: PhysicalEnvironment
    contacts: Mapping[str, ContactMaterial]
    damping: Mapping[str, RigidBodyDamping]
    settling: SettlingCriteria
    conveyors: Mapping[str, ConveyorSurfaceMotion]
    exclusive_surface_drive_at_transfer: bool
    vacuum_attachment: VacuumAttachment
    source_file_hashes: Mapping[str, str]
    fingerprint: str
    data: Mapping[str, Any]

    @property
    def robot_mass_kg(self) -> float:
        return sum(link.mass_kg for link in self.robot_links.values())

    @property
    def robot_with_fixed_tool_mass_kg(self) -> float:
        """Rigid-body mass after the tool contribution is folded into J6 once."""
        return self.robot_mass_kg + self.tool.mass_kg

    @property
    def total_configured_mass_kg(self) -> float:
        """Mass inventory, without implying that the tool is a separate body."""
        return self.robot_with_fixed_tool_mass_kg + self.cartons.inventory_mass_kg

    @property
    def cache_identity(self) -> Mapping[str, Any]:
        return _freeze(
            {
                "schema_version": str(self.data["schema_version"]),
                "fingerprint": self.fingerprint,
                "source_file_hashes": dict(self.source_file_hashes),
                "components": _thaw(self.data["cache_identity_components"]),
            }
        )

    def to_mapping(self) -> dict[str, Any]:
        """Return a mutable copy of the validated source document."""
        return _thaw(self.data)


def _load_sources(data: Mapping[str, Any], config_path: Path) -> tuple[dict[str, Path], dict[str, dict[str, Any]]]:
    sources = _mapping(data["sources"], "sources")
    expected = {"robot_model_config", "robot_urdf", "workcell_layout", "tool_config", "gripper_asset_manifest"}
    _strict_keys(sources, expected, "sources")
    paths = {name: _resolve(config_path.parent, sources[name], f"sources.{name}") for name in sorted(expected)}
    documents = {
        "robot_model_config": _read_yaml(paths["robot_model_config"], "robot model config"),
        "workcell_layout": _read_yaml(paths["workcell_layout"], "workcell layout"),
        "gripper_asset_manifest": _read_yaml(paths["gripper_asset_manifest"], "gripper asset manifest"),
    }
    return paths, documents


def _validate_source_coherence(paths: Mapping[str, Path], documents: Mapping[str, Mapping[str, Any]]) -> None:
    robot = documents["robot_model_config"]
    if robot.get("model") != "fanuc_m710id_70":
        raise ValueError("robot model config must describe fanuc_m710id_70")
    kinematics = _mapping(robot.get("kinematics"), "robot model config.kinematics")
    if tuple(kinematics.get("active_joint_names", ())) != ACTIVE_JOINT_NAMES:
        raise ValueError("robot model config active_joint_names must be J1 through J6")
    referenced_urdf = _resolve(paths["robot_model_config"].parent, kinematics.get("urdf_path"), "robot kinematics.urdf_path")
    if referenced_urdf != paths["robot_urdf"]:
        raise ValueError("sources.robot_urdf does not match robot model config")

    try:
        root = ET.parse(paths["robot_urdf"]).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ValueError("sources.robot_urdf is not valid XML") from exc
    if root.tag != "robot" or root.attrib.get("name") != "fanuc_m710id_70":
        raise ValueError("sources.robot_urdf must be the fanuc_m710id_70 model")
    urdf_links = {node.attrib.get("name") for node in root.findall("link")}
    if not set(ROBOT_LINK_NAMES).issubset(urdf_links):
        raise ValueError("sources.robot_urdf is missing a configured inertial link")
    urdf_joints = {node.attrib.get("name") for node in root.findall("joint")}
    if not set(ACTIVE_JOINT_NAMES).issubset(urdf_joints):
        raise ValueError("sources.robot_urdf is missing an active joint")

    layout = documents["workcell_layout"]
    if layout.get("schema") != "m710id70_unloading_layout_v1" or layout.get("layout_id") != "m710id70_unloading_layout_v1":
        raise ValueError("workcell layout must be m710id70_unloading_layout_v1")
    layout_robot = _mapping(layout.get("robot"), "workcell layout.robot")
    referenced_model = _resolve(paths["workcell_layout"].parent, layout_robot.get("model_config"), "layout.robot.model_config")
    referenced_layout_urdf = _resolve(paths["workcell_layout"].parent, layout_robot.get("urdf_path"), "layout.robot.urdf_path")
    if referenced_model != paths["robot_model_config"] or referenced_layout_urdf != paths["robot_urdf"]:
        raise ValueError("dynamics robot sources do not match the confirmed workcell layout")
    layout_tool = _mapping(layout.get("tool"), "workcell layout.tool")
    referenced_tool = _resolve(paths["workcell_layout"].parent, layout_tool.get("load_config"), "layout.tool.load_config")
    if referenced_tool != paths["tool_config"]:
        raise ValueError("sources.tool_config does not match the confirmed workcell layout")

    gripper = documents["gripper_asset_manifest"]
    if gripper.get("asset_id") != "shanghai_wantai_three_zone_supplied_step_v1":
        raise ValueError("sources.gripper_asset_manifest must identify the Shanghai Wantai supplied STEP")
    contact_geometry = _mapping(gripper.get("contact_geometry"), "gripper asset manifest.contact_geometry")
    if contact_geometry.get("cup_model") != "FG42" or contact_geometry.get("cup_count") != 72:
        raise ValueError("gripper asset manifest must identify all 72 FG42 cups")
    if _finite(contact_geometry.get("configured_cup_compression_m"), "gripper configured cup compression") != 0.015:
        raise ValueError("gripper asset manifest cup compression must remain 15 mm")
    zones = _mapping(gripper.get("zones"), "gripper asset manifest.zones")
    if zones.get("count") != 3 or tuple(zones.get("cups_per_zone", ())) != (24, 24, 24):
        raise ValueError("gripper asset manifest must retain three 24-cup zones")


def _load_links_and_drives(
    data: Mapping[str, Any],
    robot_document: Mapping[str, Any],
    official_mechanical_weight_kg: float,
    urdf_path: Path,
) -> tuple[Mapping[str, RigidBodyInertial], Mapping[str, JointDrive]]:
    robot = _mapping(data["robot"], "robot")
    _strict_keys(
        robot,
        {
            "inertia_reference",
            "drive_mode",
            "source_status",
            "mechanical_weight_kg",
            "mass_estimate_basis",
            "inertia_estimate_basis",
            "links",
            "joint_drives",
        },
        "robot",
    )
    if robot["inertia_reference"] != "center_of_mass_expressed_in_link_frame":
        raise ValueError("robot.inertia_reference must identify COM tensors in each link frame")
    if robot["drive_mode"] != "force_limited_position_pd":
        raise ValueError("robot.drive_mode must be force_limited_position_pd")
    _status(robot["source_status"], "robot.source_status")
    mechanical_weight = _positive(robot["mechanical_weight_kg"], "robot.mechanical_weight_kg")
    if mechanical_weight != official_mechanical_weight_kg:
        raise ValueError("robot.mechanical_weight_kg does not match manufacturer evidence")
    if robot["mass_estimate_basis"] != "proxy_volume_distribution_scaled_to_official_mechanical_weight_580kg":
        raise ValueError("robot.mass_estimate_basis must document the proxy-volume 580 kg distribution")
    _text(robot["inertia_estimate_basis"], "robot.inertia_estimate_basis")

    links_data = _mapping(robot["links"], "robot.links")
    if set(links_data) != set(ROBOT_LINK_NAMES) or len(links_data) != len(ROBOT_LINK_NAMES):
        raise ValueError(f"robot.links must contain exactly these names: {ROBOT_LINK_NAMES}")
    links: dict[str, RigidBodyInertial] = {}
    proxy_properties = _urdf_proxy_properties(urdf_path, mechanical_weight)
    for name in ROBOT_LINK_NAMES:
        item = _mapping(links_data[name], f"robot.links.{name}")
        _strict_keys(item, {"mass_kg", "com_xyz_m", "inertia_tensor_com_kg_m2", "estimate_method"}, f"robot.links.{name}")
        link = RigidBodyInertial(
            name=name,
            mass_kg=_positive(item["mass_kg"], f"robot.links.{name}.mass_kg"),
            com_xyz_m=_vector(item["com_xyz_m"], 3, f"robot.links.{name}.com_xyz_m"),  # type: ignore[arg-type]
            inertia_tensor_com_kg_m2=_inertia(
                item["inertia_tensor_com_kg_m2"], f"robot.links.{name}.inertia_tensor_com_kg_m2"
            ),
            estimate_method=_text(item["estimate_method"], f"robot.links.{name}.estimate_method"),
        )
        expected_mass, expected_com, expected_inertia = proxy_properties[name]
        if not np.isclose(link.mass_kg, expected_mass, rtol=0.0, atol=1e-10):
            raise ValueError(f"robot.links.{name}.mass_kg does not match the declared proxy-volume distribution")
        if not np.allclose(link.com_xyz_m, expected_com, rtol=0.0, atol=1e-12):
            raise ValueError(f"robot.links.{name}.com_xyz_m does not match the URDF proxy centroid")
        if not np.allclose(link.inertia_tensor_com_kg_m2, expected_inertia, rtol=0.0, atol=1e-12):
            raise ValueError(f"robot.links.{name}.inertia_tensor_com_kg_m2 does not match the declared uniform proxy")
        links[name] = link
    if not np.isclose(sum(link.mass_kg for link in links.values()), mechanical_weight, rtol=0.0, atol=1e-9):
        raise ValueError("robot link masses must sum to the official 580 kg mechanical weight")

    drive_data = _mapping(robot["joint_drives"], "robot.joint_drives")
    if set(drive_data) != set(ACTIVE_JOINT_NAMES) or len(drive_data) != len(ACTIVE_JOINT_NAMES):
        raise ValueError(f"robot.joint_drives must contain exactly these names: {ACTIVE_JOINT_NAMES}")
    model_velocities = _vector(robot_document.get("joint_velocity_limits_rad_s"), 6, "robot model joint_velocity_limits_rad_s")
    drives: dict[str, JointDrive] = {}
    for index, name in enumerate(ACTIVE_JOINT_NAMES):
        item = _mapping(drive_data[name], f"robot.joint_drives.{name}")
        _strict_keys(
            item,
            {"effort_limit_nm", "velocity_limit_rad_s", "stiffness_nm_rad", "damping_nm_s_rad"},
            f"robot.joint_drives.{name}",
        )
        drive = JointDrive(
            name=name,
            effort_limit_nm=_positive(item["effort_limit_nm"], f"robot.joint_drives.{name}.effort_limit_nm"),
            velocity_limit_rad_s=_positive(item["velocity_limit_rad_s"], f"robot.joint_drives.{name}.velocity_limit_rad_s"),
            stiffness_nm_rad=_positive(item["stiffness_nm_rad"], f"robot.joint_drives.{name}.stiffness_nm_rad"),
            damping_nm_s_rad=_positive(item["damping_nm_s_rad"], f"robot.joint_drives.{name}.damping_nm_s_rad"),
        )
        if not np.isclose(drive.velocity_limit_rad_s, model_velocities[index], rtol=0.0, atol=1e-12):
            raise ValueError(f"robot.joint_drives.{name}.velocity_limit_rad_s does not match the robot model")
        drives[name] = drive
    return MappingProxyType(links), MappingProxyType(drives)


def _load_tool(data: Mapping[str, Any], tool_path: Path) -> ToolDynamics:
    item = _mapping(data["tool"], "tool")
    _strict_keys(item, {"attach_to_link", "body_mode", "mass_accounting", "expected_mass_kg", "source_status"}, "tool")
    if item["attach_to_link"] != "J6_link":
        raise ValueError("tool.attach_to_link must identify J6_link for fixed-tool mass combination")
    if item["body_mode"] != FIXED_TOOL_MASS_POLICY:
        raise ValueError(f"tool.body_mode must be {FIXED_TOOL_MASS_POLICY}")
    if item["mass_accounting"] != FIXED_TOOL_MASS_POLICY:
        raise ValueError(f"tool.mass_accounting must be {FIXED_TOOL_MASS_POLICY}")
    _status(item["source_status"], "tool.source_status")
    expected_mass = _positive(item["expected_mass_kg"], "tool.expected_mass_kg")
    if expected_mass != 20.0:
        raise ValueError("tool.expected_mass_kg must preserve the configured 20 kg tool")
    source = load_tool_config(tool_path)
    if source.mass_kg != expected_mass:
        raise ValueError("tool mass does not match sources.tool_config")
    if source.mass_properties_source != "ENGINEERING_MODEL":
        raise ValueError("tool mass properties must remain explicitly identified as an engineering model")
    inertia = _inertia(source.inertia_tensor_com_kg_m2, "tool source inertia_tensor_com_kg_m2")
    return ToolDynamics(
        name=source.name,
        mass_kg=source.mass_kg,
        com_xyz_m=source.com_xyz_m,
        inertia_tensor_com_kg_m2=inertia,
        attach_to_link="J6_link",
        body_mode=FIXED_TOOL_MASS_POLICY,
        source_status=ENGINEERING_STATUS,
        mass_properties_source=source.mass_properties_source,
        machine_qualified=False,
        config_path=source.config_path,
        config_hash=source.config_hash,
    )


def _load_cartons(data: Mapping[str, Any], layout: Mapping[str, Any]) -> CartonDynamics:
    item = _mapping(data["cartons"], "cartons")
    _strict_keys(
        item,
        {
            "count",
            "mass_kg_each",
            "size_xyz_m",
            "com_xyz_m",
            "inertia_reference",
            "inertia_tensor_com_kg_m2",
            "inertia_estimate_basis",
            "mass_accounting",
            "source_status",
        },
        "cartons",
    )
    count = _positive_int(item["count"], "cartons.count")
    mass = _positive(item["mass_kg_each"], "cartons.mass_kg_each")
    if count != 40 or mass != 42.5:
        raise ValueError("cartons must preserve all 40 bodies at 42.5 kg each")
    size = _vector(item["size_xyz_m"], 3, "cartons.size_xyz_m")
    if any(value <= 0.0 for value in size):
        raise ValueError("cartons.size_xyz_m must be positive")
    com = _vector(item["com_xyz_m"], 3, "cartons.com_xyz_m")
    if item["inertia_reference"] != "center_of_mass_expressed_in_carton_body_frame":
        raise ValueError("cartons.inertia_reference must identify the carton COM body frame")
    inertia = _inertia(item["inertia_tensor_com_kg_m2"], "cartons.inertia_tensor_com_kg_m2")
    x, y, z = size
    expected = np.diag(
        [mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0, mass * (x * x + y * y) / 12.0]
    )
    if not np.allclose(np.asarray(inertia), expected, rtol=1e-12, atol=1e-12):
        raise ValueError("cartons inertia does not match the declared uniform-cuboid estimate")
    if item["inertia_estimate_basis"] != "uniform_cuboid_at_configured_mass_and_dimensions":
        raise ValueError("cartons.inertia_estimate_basis must remain explicit")
    if item["mass_accounting"] != "exactly_once_per_carton":
        raise ValueError("cartons.mass_accounting must be exactly_once_per_carton")
    _status(item["source_status"], "cartons.source_status")
    stack = _mapping(layout.get("carton_stack"), "workcell layout.carton_stack")
    layout_count = int(stack["depth_rows"]) * int(stack["width_columns"]) * int(stack["height_layers"])
    layout_size = _vector(stack["carton_size_xyz_m"], 3, "workcell layout.carton_stack.carton_size_xyz_m")
    if count != layout_count or not np.allclose(size, layout_size, rtol=0.0, atol=1e-12):
        raise ValueError("carton dynamics do not match the confirmed workcell layout")
    return CartonDynamics(count, mass, size, com, inertia)  # type: ignore[arg-type]


def _load_simulation(data: Mapping[str, Any]) -> SimulationSettings:
    item = _mapping(data["simulation"], "simulation")
    _strict_keys(
        item,
        {"gravity_world_m_s2", "physics_time_step_s", "solver_position_iterations", "solver_velocity_iterations", "source_status"},
        "simulation",
    )
    gravity = _vector(item["gravity_world_m_s2"], 3, "simulation.gravity_world_m_s2")
    if not np.allclose(gravity[:2], (0.0, 0.0), atol=1e-12, rtol=0.0) or gravity[2] >= 0.0:
        raise ValueError("simulation.gravity_world_m_s2 must point down in the +Z-up world")
    _status(item["source_status"], "simulation.source_status")
    return SimulationSettings(
        gravity_world_m_s2=gravity,  # type: ignore[arg-type]
        physics_time_step_s=_positive(item["physics_time_step_s"], "simulation.physics_time_step_s"),
        solver_position_iterations=_positive_int(item["solver_position_iterations"], "simulation.solver_position_iterations"),
        solver_velocity_iterations=_positive_int(item["solver_velocity_iterations"], "simulation.solver_velocity_iterations"),
    )


def _load_environment(data: Mapping[str, Any], layout: Mapping[str, Any], contacts: set[str]) -> PhysicalEnvironment:
    item = _mapping(data["environment"], "environment")
    _strict_keys(item, {"floor", "side_walls"}, "environment")
    floor = _mapping(item["floor"], "environment.floor")
    _strict_keys(floor, {"collision_enabled", "kinematic", "z_m", "collision_model", "material"}, "environment.floor")
    walls = _mapping(item["side_walls"], "environment.side_walls")
    _strict_keys(
        walls,
        {"collision_enabled", "kinematic", "right_y_m", "left_y_m", "collision_model", "extent_status", "material"},
        "environment.side_walls",
    )
    for prefix, section in (("environment.floor", floor), ("environment.side_walls", walls)):
        if not _boolean(section["collision_enabled"], f"{prefix}.collision_enabled"):
            raise ValueError(f"{prefix}.collision_enabled must remain true")
        if not _boolean(section["kinematic"], f"{prefix}.kinematic"):
            raise ValueError(f"{prefix}.kinematic must remain true")
        if section["material"] not in contacts:
            raise ValueError(f"{prefix}.material names an unknown contact material")
    if floor["collision_model"] != "half_space_z_greater_than_or_equal_to_floor":
        raise ValueError("environment.floor.collision_model must retain physical support semantics")
    if walls["collision_model"] != "known_y_interval_constraint":
        raise ValueError("environment.side_walls.collision_model must retain the known lateral constraints")
    extent = _text(walls["extent_status"], "environment.side_walls.extent_status")
    if extent != "TRAILER_LENGTH_AND_HEIGHT_UNDEFINED_NO_EXTENT_CLAIM":
        raise ValueError("environment.side_walls.extent_status must preserve unknown trailer extents")
    floor_z = _finite(floor["z_m"], "environment.floor.z_m")
    right_y = _finite(walls["right_y_m"], "environment.side_walls.right_y_m")
    left_y = _finite(walls["left_y_m"], "environment.side_walls.left_y_m")
    if right_y >= left_y:
        raise ValueError("environment side walls must define an ordered non-empty interval")
    world = _mapping(layout.get("world"), "workcell layout.world")
    trailer = _mapping(layout.get("trailer"), "workcell layout.trailer")
    if not np.isclose(floor_z, _finite(world["floor_z_m"], "layout.world.floor_z_m"), atol=1e-12, rtol=0.0):
        raise ValueError("physical floor does not match the confirmed layout")
    if not np.allclose(
        (right_y, left_y),
        (_finite(trailer["right_wall_y_m"], "layout.trailer.right_wall_y_m"), _finite(trailer["left_wall_y_m"], "layout.trailer.left_wall_y_m")),
        atol=1e-12,
        rtol=0.0,
    ):
        raise ValueError("physical side walls do not match the confirmed layout")
    if trailer.get("length_m") is not None or trailer.get("height_m") is not None:
        raise ValueError("this dynamics schema must not invent trailer length or height")
    return PhysicalEnvironment(floor_z, right_y, left_y, str(floor["material"]), str(walls["material"]), extent)


def _load_contacts(data: Mapping[str, Any]) -> Mapping[str, ContactMaterial]:
    item = _mapping(data["contacts"], "contacts")
    _strict_keys(item, {"friction_combine_mode", "restitution_combine_mode", "source_status", "materials"}, "contacts")
    if item["friction_combine_mode"] != "min" or item["restitution_combine_mode"] != "min":
        raise ValueError("contacts combine modes must be deterministic min")
    _status(item["source_status"], "contacts.source_status")
    materials_data = _mapping(item["materials"], "contacts.materials")
    if set(materials_data) != set(CONTACT_MATERIAL_NAMES) or len(materials_data) != len(CONTACT_MATERIAL_NAMES):
        raise ValueError(f"contacts.materials must contain exactly these names: {CONTACT_MATERIAL_NAMES}")
    materials: dict[str, ContactMaterial] = {}
    for name in CONTACT_MATERIAL_NAMES:
        material = _mapping(materials_data[name], f"contacts.materials.{name}")
        _strict_keys(material, {"static_friction", "dynamic_friction", "restitution"}, f"contacts.materials.{name}")
        static = _nonnegative(material["static_friction"], f"contacts.materials.{name}.static_friction")
        dynamic = _nonnegative(material["dynamic_friction"], f"contacts.materials.{name}.dynamic_friction")
        restitution = _nonnegative(material["restitution"], f"contacts.materials.{name}.restitution")
        if dynamic > static:
            raise ValueError(f"contacts.materials.{name}.dynamic_friction cannot exceed static_friction")
        if restitution > 1.0:
            raise ValueError(f"contacts.materials.{name}.restitution must be in [0, 1]")
        materials[name] = ContactMaterial(name, static, dynamic, restitution)
    return MappingProxyType(materials)


def _load_damping(data: Mapping[str, Any]) -> Mapping[str, RigidBodyDamping]:
    item = _mapping(data["damping"], "damping")
    _strict_keys(item, {"source_status", "rigid_bodies"}, "damping")
    _status(item["source_status"], "damping.source_status")
    groups_data = _mapping(item["rigid_bodies"], "damping.rigid_bodies")
    if set(groups_data) != set(DAMPED_BODY_GROUPS) or len(groups_data) != len(DAMPED_BODY_GROUPS):
        raise ValueError(f"damping.rigid_bodies must contain exactly these names: {DAMPED_BODY_GROUPS}")
    groups: dict[str, RigidBodyDamping] = {}
    for name in DAMPED_BODY_GROUPS:
        group = _mapping(groups_data[name], f"damping.rigid_bodies.{name}")
        _strict_keys(group, {"linear_damping_s_inv", "angular_damping_s_inv"}, f"damping.rigid_bodies.{name}")
        groups[name] = RigidBodyDamping(
            name,
            _positive(group["linear_damping_s_inv"], f"damping.rigid_bodies.{name}.linear_damping_s_inv"),
            _positive(group["angular_damping_s_inv"], f"damping.rigid_bodies.{name}.angular_damping_s_inv"),
        )
    if groups["tool"] != RigidBodyDamping(
        "tool",
        groups["robot_links"].linear_damping_s_inv,
        groups["robot_links"].angular_damping_s_inv,
    ):
        raise ValueError(
            "damping.rigid_bodies.tool must match robot_links when the tool is combined into J6"
        )
    return MappingProxyType(groups)


def _load_settling(data: Mapping[str, Any]) -> SettlingCriteria:
    item = _mapping(data["settling"], "settling")
    _strict_keys(
        item,
        {
            "maximum_settle_time_s",
            "required_stable_duration_s",
            "max_linear_speed_m_s",
            "max_angular_speed_rad_s",
            "max_position_drift_m",
            "max_penetration_m",
            "source_status",
        },
        "settling",
    )
    _status(item["source_status"], "settling.source_status")
    result = SettlingCriteria(
        _positive(item["maximum_settle_time_s"], "settling.maximum_settle_time_s"),
        _positive(item["required_stable_duration_s"], "settling.required_stable_duration_s"),
        _positive(item["max_linear_speed_m_s"], "settling.max_linear_speed_m_s"),
        _positive(item["max_angular_speed_rad_s"], "settling.max_angular_speed_rad_s"),
        _positive(item["max_position_drift_m"], "settling.max_position_drift_m"),
        _positive(item["max_penetration_m"], "settling.max_penetration_m"),
    )
    if result.required_stable_duration_s > result.maximum_settle_time_s:
        raise ValueError("settling.required_stable_duration_s cannot exceed maximum_settle_time_s")
    return result


def _load_conveyors(data: Mapping[str, Any], material_names: set[str]) -> tuple[Mapping[str, ConveyorSurfaceMotion], bool]:
    item = _mapping(data["conveyors"], "conveyors")
    _strict_keys(item, {"source_status", "exclusive_surface_drive_at_transfer", "surfaces"}, "conveyors")
    _status(item["source_status"], "conveyors.source_status")
    exclusive = _boolean(item["exclusive_surface_drive_at_transfer"], "conveyors.exclusive_surface_drive_at_transfer")
    if not exclusive:
        raise ValueError("conveyors.exclusive_surface_drive_at_transfer must prevent competing belt drives")
    surfaces_data = _mapping(item["surfaces"], "conveyors.surfaces")
    if set(surfaces_data) != set(CONVEYOR_SURFACE_NAMES) or len(surfaces_data) != len(CONVEYOR_SURFACE_NAMES):
        raise ValueError(f"conveyors.surfaces must contain exactly these names: {CONVEYOR_SURFACE_NAMES}")
    surfaces: dict[str, ConveyorSurfaceMotion] = {}
    for name in CONVEYOR_SURFACE_NAMES:
        surface = _mapping(surfaces_data[name], f"conveyors.surfaces.{name}")
        _strict_keys(surface, {"enabled", "direction_world", "speed_m_s", "material"}, f"conveyors.surfaces.{name}")
        enabled = _boolean(surface["enabled"], f"conveyors.surfaces.{name}.enabled")
        if not enabled:
            raise ValueError(f"conveyors.surfaces.{name}.enabled must remain true")
        direction = _vector(surface["direction_world"], 3, f"conveyors.surfaces.{name}.direction_world")
        if not np.allclose(direction, EXPECTED_CONVEYOR_DIRECTIONS[name], atol=1e-12, rtol=0.0):
            raise ValueError(f"conveyors.surfaces.{name}.direction_world has the wrong unloading direction")
        if not np.isclose(np.linalg.norm(direction), 1.0, atol=1e-12, rtol=0.0):
            raise ValueError(f"conveyors.surfaces.{name}.direction_world must be a unit vector")
        material = _text(surface["material"], f"conveyors.surfaces.{name}.material")
        if material not in material_names:
            raise ValueError(f"conveyors.surfaces.{name}.material names an unknown contact material")
        surfaces[name] = ConveyorSurfaceMotion(
            name,
            enabled,
            direction,  # type: ignore[arg-type]
            _positive(surface["speed_m_s"], f"conveyors.surfaces.{name}.speed_m_s"),
            material,
        )
    return MappingProxyType(surfaces), exclusive


def _load_vacuum(
    data: Mapping[str, Any], carton_mass_kg: float, gravity_m_s2: float, gripper_manifest: Mapping[str, Any]
) -> VacuumAttachment:
    item = _mapping(data["vacuum_attachment"], "vacuum_attachment")
    _strict_keys(
        item,
        {
            "product_model",
            "cup_model",
            "physical_cup_count",
            "cup_rows",
            "cup_columns",
            "cup_pitch_m",
            "cup_radius_m",
            "zone_count",
            "cups_per_zone",
            "cup_compression_m",
            "pull_off_force_per_cup_n",
            "shear_force_per_cup_n",
            "hardware_maximum_pull_off_force_n",
            "hardware_maximum_shear_force_n",
            "force_input_source",
            "constraint_type",
            "require_validated_contact_endpoint",
            "require_suction_coverage_validation",
            "require_surface_normal_validation",
            "teleport_payload_on_attach",
            "unbreakable",
            "max_attachment_gap_m",
            "max_normal_misalignment_rad",
            "linear_stiffness_n_m",
            "linear_damping_n_s_m",
            "angular_stiffness_nm_rad",
            "angular_damping_nm_s_rad",
            "break_force_n",
            "break_force_policy",
            "break_torque_nm",
            "source_status",
        },
        "vacuum_attachment",
    )
    if item["constraint_type"] != "finite_breakable_6dof":
        raise ValueError("vacuum_attachment.constraint_type must be finite_breakable_6dof")
    product_model = _text(item["product_model"], "vacuum_attachment.product_model")
    cup_model = _text(item["cup_model"], "vacuum_attachment.cup_model")
    cup_count = _positive_int(item["physical_cup_count"], "vacuum_attachment.physical_cup_count")
    cup_rows = _positive_int(item["cup_rows"], "vacuum_attachment.cup_rows")
    cup_columns = _positive_int(item["cup_columns"], "vacuum_attachment.cup_columns")
    cup_pitch = _vector(item["cup_pitch_m"], 2, "vacuum_attachment.cup_pitch_m")
    cup_radius = _positive(item["cup_radius_m"], "vacuum_attachment.cup_radius_m")
    zone_count = _positive_int(item["zone_count"], "vacuum_attachment.zone_count")
    cups_per_zone_raw = item["cups_per_zone"]
    if not isinstance(cups_per_zone_raw, Sequence) or isinstance(cups_per_zone_raw, (str, bytes)):
        raise ValueError("vacuum_attachment.cups_per_zone must be a sequence")
    cups_per_zone = tuple(
        _positive_int(value, f"vacuum_attachment.cups_per_zone[{index}]")
        for index, value in enumerate(cups_per_zone_raw)
    )
    if product_model != "上海皖泰真空吸盘三分区" or cup_model != "FG42":
        raise ValueError("vacuum attachment must retain the Shanghai Wantai FG42 product identity")
    if (
        cup_count != 72
        or cup_rows != 6
        or cup_columns != 12
        or cup_rows * cup_columns != cup_count
        or not np.allclose(cup_pitch, (0.048, 0.048), atol=1e-12, rtol=0.0)
        or cup_radius != 0.0215
        or zone_count != 3
        or cups_per_zone != (24, 24, 24)
        or sum(cups_per_zone) != cup_count
    ):
        raise ValueError("vacuum attachment must retain all 72 cups in three 24-cup zones")
    compression = _positive(item["cup_compression_m"], "vacuum_attachment.cup_compression_m")
    pull_off_per_cup = _positive(item["pull_off_force_per_cup_n"], "vacuum_attachment.pull_off_force_per_cup_n")
    shear_per_cup = _positive(item["shear_force_per_cup_n"], "vacuum_attachment.shear_force_per_cup_n")
    hardware_pull_off = _positive(
        item["hardware_maximum_pull_off_force_n"], "vacuum_attachment.hardware_maximum_pull_off_force_n"
    )
    hardware_shear = _positive(
        item["hardware_maximum_shear_force_n"], "vacuum_attachment.hardware_maximum_shear_force_n"
    )
    if compression != 0.015 or pull_off_per_cup != 59.0 or shear_per_cup != 43.0:
        raise ValueError("vacuum attachment must preserve the configured 15 mm, 59 N, and 43 N project inputs")
    if hardware_pull_off != cup_count * pull_off_per_cup or hardware_shear != cup_count * shear_per_cup:
        raise ValueError("vacuum attachment hardware force totals must equal all 72 per-cup inputs")
    force_source = _text(item["force_input_source"], "vacuum_attachment.force_input_source")
    if force_source != "PROJECT_INPUT_NOT_MEASURED_OR_VENDOR_QUALIFIED":
        raise ValueError("vacuum force inputs must remain explicitly unmeasured and unqualified")
    manifest_contact = _mapping(gripper_manifest.get("contact_geometry"), "gripper manifest.contact_geometry")
    manifest_zones = _mapping(gripper_manifest.get("zones"), "gripper manifest.zones")
    if (
        manifest_contact.get("cup_model") != cup_model
        or manifest_contact.get("cup_count") != cup_count
        or manifest_contact.get("rows") != cup_rows
        or manifest_contact.get("columns") != cup_columns
        or not np.allclose(
            _vector(
                [
                    _finite(manifest_contact.get("pitch_length_mm"), "manifest pitch length") * 0.001,
                    _finite(manifest_contact.get("pitch_width_mm"), "manifest pitch width") * 0.001,
                ],
                2,
                "manifest cup pitch",
            ),
            cup_pitch,
            atol=1e-12,
            rtol=0.0,
        )
        or not np.isclose(
            _finite(manifest_contact.get("cup_radius_mm"), "manifest cup radius") * 0.001,
            cup_radius,
            atol=1e-12,
            rtol=0.0,
        )
        or _finite(manifest_contact.get("configured_cup_compression_m"), "manifest cup compression") != compression
        or manifest_zones.get("count") != zone_count
        or tuple(manifest_zones.get("cups_per_zone", ())) != cups_per_zone
    ):
        raise ValueError("vacuum attachment geometry does not match the gripper asset manifest")
    for field in (
        "require_validated_contact_endpoint",
        "require_suction_coverage_validation",
        "require_surface_normal_validation",
    ):
        if not _boolean(item[field], f"vacuum_attachment.{field}"):
            raise ValueError(f"vacuum_attachment.{field} must remain true")
    if _boolean(item["teleport_payload_on_attach"], "vacuum_attachment.teleport_payload_on_attach"):
        raise ValueError("vacuum attachment cannot teleport the payload")
    if _boolean(item["unbreakable"], "vacuum_attachment.unbreakable"):
        raise ValueError("vacuum attachment cannot be unbreakable")
    _status(item["source_status"], "vacuum_attachment.source_status")
    result = VacuumAttachment(
        product_model=product_model,
        cup_model=cup_model,
        physical_cup_count=cup_count,
        cup_rows=cup_rows,
        cup_columns=cup_columns,
        cup_pitch_m=cup_pitch,  # type: ignore[arg-type]
        cup_radius_m=cup_radius,
        zone_count=zone_count,
        cups_per_zone=cups_per_zone,
        cup_compression_m=compression,
        pull_off_force_per_cup_n=pull_off_per_cup,
        shear_force_per_cup_n=shear_per_cup,
        hardware_maximum_pull_off_force_n=hardware_pull_off,
        hardware_maximum_shear_force_n=hardware_shear,
        force_input_source=force_source,
        constraint_type="finite_breakable_6dof",
        max_attachment_gap_m=_positive(item["max_attachment_gap_m"], "vacuum_attachment.max_attachment_gap_m"),
        max_normal_misalignment_rad=_positive(item["max_normal_misalignment_rad"], "vacuum_attachment.max_normal_misalignment_rad"),
        linear_stiffness_n_m=_positive(item["linear_stiffness_n_m"], "vacuum_attachment.linear_stiffness_n_m"),
        linear_damping_n_s_m=_positive(item["linear_damping_n_s_m"], "vacuum_attachment.linear_damping_n_s_m"),
        angular_stiffness_nm_rad=_positive(item["angular_stiffness_nm_rad"], "vacuum_attachment.angular_stiffness_nm_rad"),
        angular_damping_nm_s_rad=_positive(item["angular_damping_nm_s_rad"], "vacuum_attachment.angular_damping_nm_s_rad"),
        break_force_n=_positive(item["break_force_n"], "vacuum_attachment.break_force_n"),
        break_force_policy=_text(item["break_force_policy"], "vacuum_attachment.break_force_policy"),
        break_torque_nm=_positive(item["break_torque_nm"], "vacuum_attachment.break_torque_nm"),
    )
    if result.max_normal_misalignment_rad >= np.pi / 2.0:
        raise ValueError("vacuum_attachment.max_normal_misalignment_rad must be less than pi/2")
    if result.break_force_n <= carton_mass_kg * abs(gravity_m_s2):
        raise ValueError("vacuum_attachment.break_force_n cannot support the configured carton in gravity")
    if result.break_force_n > result.hardware_maximum_pull_off_force_n:
        raise ValueError("vacuum_attachment.break_force_n cannot exceed the 72-cup pull-off total")
    if result.break_force_policy != "reduce_to_actual_sealed_cup_count_and_load_direction":
        raise ValueError("vacuum_attachment.break_force_policy must scale by sealed cups and load direction")
    if result.break_torque_nm != 180.0:
        raise ValueError("vacuum_attachment.break_torque_nm must preserve the finite 180 Nm engineering input")
    return result


def _validate_metadata(data: Mapping[str, Any]) -> tuple[str, float, Mapping[str, Any]]:
    if data["schema_version"] != SCHEMA_VERSION:
        raise ValueError(f"schema_version must be {SCHEMA_VERSION}")
    if data["model_id"] != "fanuc_m710id_70":
        raise ValueError("model_id must be fanuc_m710id_70")
    qualification = _mapping(data["qualification"], "qualification")
    _strict_keys(qualification, {"status", "machine_qualified", "permitted_use", "prohibited_use"}, "qualification")
    status = _status(qualification["status"], "qualification.status")
    if _boolean(qualification["machine_qualified"], "qualification.machine_qualified"):
        raise ValueError("qualification.machine_qualified must remain false")
    _text(qualification["permitted_use"], "qualification.permitted_use")
    _text(qualification["prohibited_use"], "qualification.prohibited_use")

    evidence = _mapping(data["manufacturer_evidence"], "manufacturer_evidence")
    _strict_keys(
        evidence,
        {
            "manufacturer",
            "product_url",
            "data_sheet_url",
            "verified_field",
            "mechanical_weight_kg",
            "scope",
            "link_distribution_status",
        },
        "manufacturer_evidence",
    )
    if evidence["manufacturer"] != "FANUC America":
        raise ValueError("manufacturer_evidence.manufacturer must be FANUC America")
    if evidence["product_url"] != "https://www.fanucamerica.com/products/robot/m-710id-70":
        raise ValueError("manufacturer_evidence.product_url must identify the official M-710iD/70 page")
    if evidence["data_sheet_url"] != "https://www.fanucamerica.com/docs/default-source/robotics-files/m-710id-70-data-sheet.pdf":
        raise ValueError("manufacturer_evidence.data_sheet_url must identify the official data sheet")
    if evidence["verified_field"] != "mechanical_weight_kg":
        raise ValueError("manufacturer_evidence.verified_field must be mechanical_weight_kg")
    mechanical_weight = _positive(evidence["mechanical_weight_kg"], "manufacturer_evidence.mechanical_weight_kg")
    if mechanical_weight != 580.0:
        raise ValueError("manufacturer_evidence.mechanical_weight_kg must preserve FANUC's 580 kg value")
    if evidence["scope"] != "robot_mechanical_unit_without_controller":
        raise ValueError("manufacturer_evidence.scope must preserve the data-sheet scope")
    if evidence["link_distribution_status"] != "NOT_PROVIDED_BY_MANUFACTURER":
        raise ValueError("manufacturer_evidence must not claim manufacturer link mass distribution")

    cache = _mapping(data["cache_identity_components"], "cache_identity_components")
    _strict_keys(cache, {"included", "external_replay_contract_components"}, "cache_identity_components")
    included = tuple(cache["included"])
    expected_included = (
        "dynamics_config",
        "source_files",
        "robot_link_inertials",
        "joint_drives",
        "contact_model",
        "simulation_integration",
        "settling",
        "conveyor_surface_motion",
        "vacuum_attachment",
    )
    if included != expected_included or len(set(included)) != len(included):
        raise ValueError("cache_identity_components.included is incomplete or reordered")
    external = tuple(cache["external_replay_contract_components"])
    expected_external = ("cad_visual_meshes", "cad_collision_meshes", "cad_link_mapping", "cad_converter_settings")
    if external != expected_external or len(set(external)) != len(external):
        raise ValueError("cache_identity_components.external_replay_contract_components is incomplete or reordered")

    accounting = _mapping(data["mass_accounting"], "mass_accounting")
    _strict_keys(accounting, {"robot_link_bodies", "tool_body", "carton_bodies"}, "mass_accounting")
    expected_accounting = {
        "robot_link_bodies": "exactly_once_each",
        "tool_body": FIXED_TOOL_MASS_POLICY,
        "carton_bodies": "exactly_40_independent_bodies_once_each",
    }
    if dict(accounting) != expected_accounting:
        raise ValueError(
            "mass_accounting must count the robot links, fixed tool contribution, "
            "and all 40 cartons exactly once"
        )
    return status, mechanical_weight, _freeze(evidence)


def load_m710id70_engineering_dynamics(path: str | Path | None = None) -> M710EngineeringDynamicsConfig:
    """Load and fail-closed validate the independent M-710 dynamics input.

    ``None`` loads the selected official repository default. Explicit v1
    documents retain the strict historical proxy-estimate validation. Relative
    source paths are always resolved against the chosen document, making copied or
    archived configs deterministic and prevents dependence on the process CWD.
    """
    config_path = DEFAULT_CONFIG_PATH if path is None else Path(path).resolve()
    data = _read_yaml(config_path, "M-710 engineering dynamics config")
    if data.get("schema_version") == "m710id70_official_dynamics_v2":
        from .m710_official_dynamics import load_m710id70_official_dynamics

        return load_m710id70_official_dynamics(config_path)
    _strict_keys(
        data,
        {
            "schema_version",
            "model_id",
            "qualification",
            "sources",
            "manufacturer_evidence",
            "cache_identity_components",
            "mass_accounting",
            "robot",
            "tool",
            "cartons",
            "simulation",
            "environment",
            "contacts",
            "damping",
            "settling",
            "conveyors",
            "vacuum_attachment",
        },
        "root",
    )
    qualification_status, mechanical_weight, manufacturer_evidence = _validate_metadata(data)
    paths, documents = _load_sources(data, config_path)
    _validate_source_coherence(paths, documents)
    robot_links, joint_drives = _load_links_and_drives(
        data, documents["robot_model_config"], mechanical_weight, paths["robot_urdf"]
    )
    tool = _load_tool(data, paths["tool_config"])
    cartons = _load_cartons(data, documents["workcell_layout"])
    simulation = _load_simulation(data)
    contacts = _load_contacts(data)
    environment = _load_environment(data, documents["workcell_layout"], set(contacts))
    damping = _load_damping(data)
    settling = _load_settling(data)
    conveyors, exclusive = _load_conveyors(data, set(contacts))
    vacuum = _load_vacuum(
        data,
        cartons.mass_kg_each,
        simulation.gravity_world_m_s2[2],
        documents["gripper_asset_manifest"],
    )

    source_hashes = {
        "dynamics_config": sha256_file(config_path),
        **{name: sha256_file(source_path) for name, source_path in sorted(paths.items())},
    }
    fingerprint = _canonical_hash(
        {
            "schema_version": SCHEMA_VERSION,
            "configuration": data,
            "source_file_hashes": source_hashes,
        }
    )
    return M710EngineeringDynamicsConfig(
        config_path=config_path,
        qualification_status=qualification_status,
        machine_qualified=False,
        mechanical_weight_kg=mechanical_weight,
        manufacturer_evidence=manufacturer_evidence,
        robot_links=robot_links,
        joint_drives=joint_drives,
        tool=tool,
        cartons=cartons,
        simulation=simulation,
        environment=environment,
        contacts=contacts,
        damping=damping,
        settling=settling,
        conveyors=conveyors,
        exclusive_surface_drive_at_transfer=exclusive,
        vacuum_attachment=vacuum,
        source_file_hashes=MappingProxyType(source_hashes),
        fingerprint=fingerprint,
        data=_freeze(data),
    )


def load_m710id70_dynamics(path: str | Path | None = None) -> M710EngineeringDynamicsConfig:
    """Load either the retained legacy estimate or the selected official model."""

    config_path = DEFAULT_CONFIG_PATH if path is None else Path(path).resolve()
    document = _read_yaml(config_path, "M-710 dynamics config")
    if document.get("schema_version") == SCHEMA_VERSION:
        return load_m710id70_engineering_dynamics(config_path)
    if document.get("schema_version") == "m710id70_official_dynamics_v2":
        from .m710_official_dynamics import load_m710id70_official_dynamics

        return load_m710id70_official_dynamics(config_path)
    raise ValueError(f"unsupported M-710 dynamics schema: {document.get('schema_version')!r}")


__all__ = [
    "ACTIVE_JOINT_NAMES",
    "CONVEYOR_SURFACE_NAMES",
    "DEFAULT_CONFIG_PATH",
    "ENGINEERING_STATUS",
    "FIXED_TOOL_MASS_POLICY",
    "ROBOT_LINK_NAMES",
    "CartonDynamics",
    "ContactMaterial",
    "ConveyorSurfaceMotion",
    "JointDrive",
    "M710EngineeringDynamicsConfig",
    "PhysicalEnvironment",
    "RigidBodyDamping",
    "RigidBodyInertial",
    "SettlingCriteria",
    "SimulationSettings",
    "ToolDynamics",
    "VacuumAttachment",
    "load_m710id70_engineering_dynamics",
    "load_m710id70_dynamics",
]
