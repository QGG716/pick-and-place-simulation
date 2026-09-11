"""Strict CPU-only loader for the pinned official M-710iD/70 dynamics model."""

from __future__ import annotations

from collections.abc import Mapping
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
from .m710_dynamics import (
    ACTIVE_JOINT_NAMES,
    FIXED_TOOL_MASS_POLICY,
    ROBOT_LINK_NAMES,
    CartonDynamics,
    ContactMaterial,
    ConveyorSurfaceMotion,
    JointDrive,
    M710EngineeringDynamicsConfig,
    PhysicalEnvironment,
    RigidBodyDamping,
    RigidBodyInertial,
    SettlingCriteria,
    SimulationSettings,
    ToolDynamics,
    _validate_source_coherence,
)


SCHEMA_VERSION = "m710id70_official_dynamics_v2"
OFFICIAL_STATUS = "OFFICIAL_PUBLIC_MODEL_ENGINEERING_SIMULATION"
OFFICIAL_COMMIT = "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2"
EXPECTED_EFFORT = (8000.0, 10000.0, 5000.0, 2000.0, 1000.0, 900.0)
EXPECTED_VELOCITY = tuple(np.radians([180.0, 180.0, 180.0, 260.0, 260.0, 370.0]))


@dataclass(frozen=True)
class IdealIndependentCupAttachment:
    suction_mode: str
    holding_capacity_assumption: str
    physical_cup_count: int
    cup_rows: int
    cup_columns: int
    cup_pitch_m: tuple[float, float]
    cup_radius_m: float
    cup_compression_m: float
    require_nonempty_geometric_contact: bool
    require_validated_contact_endpoint: bool
    require_actual_fk_contact_recheck: bool
    require_target_identity_match: bool
    teleport_payload_on_attach: bool
    constraint_type: str
    enforce_vacuum_force_capacity: bool
    enforce_vacuum_break_force: bool
    enforce_vacuum_break_torque: bool
    max_attachment_gap_m: float
    max_contact_penetration_m: float
    max_normal_misalignment_rad: float


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _finite(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not np.isfinite(result) or (positive and result <= 0.0):
        raise ValueError(f"{name} must be {'positive and ' if positive else ''}finite")
    return result


def _vector(value: Any, length: int, name: str) -> tuple[float, ...]:
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite values")
    return tuple(float(item) for item in result)


def _tensor(value: Any, name: str) -> tuple[tuple[float, float, float], ...]:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)) or not np.allclose(result, result.T, atol=1e-12):
        raise ValueError(f"{name} must be a finite symmetric 3x3 tensor")
    eigenvalues = np.linalg.eigvalsh(result)
    if eigenvalues[0] <= 0.0 or eigenvalues[-1] > eigenvalues[0] + eigenvalues[1] + 1e-10:
        raise ValueError(f"{name} is not a physical rigid-body inertia tensor")
    return tuple(tuple(float(item) for item in row) for row in result)  # type: ignore[return-value]


def _resolve(base: Path, value: Any, name: str) -> Path:
    declared = Path(str(value))
    result = declared.resolve() if declared.is_absolute() else (base / declared).resolve()
    if not result.is_file():
        raise FileNotFoundError(f"{name} is missing: {result}")
    return result


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, list | tuple):
        return tuple(_freeze(item) for item in value)
    return value


def _canonical_hash(value: Mapping[str, Any]) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _urdf_inertial(link: ET.Element, name: str) -> tuple[float, tuple[float, ...], tuple[tuple[float, ...], ...]]:
    inertial = link.find("inertial")
    if inertial is None or inertial.find("mass") is None or inertial.find("inertia") is None:
        raise ValueError(f"official URDF link {name} has no complete inertial")
    origin = inertial.find("origin")
    rpy = (0.0, 0.0, 0.0) if origin is None else _vector(origin.get("rpy", "0 0 0").split(), 3, f"{name} inertial rpy")
    if not np.allclose(rpy, 0.0, atol=1e-15):
        raise ValueError(f"official URDF link {name} uses an unsupported rotated inertial frame")
    xyz = (0.0, 0.0, 0.0) if origin is None else _vector(origin.get("xyz", "0 0 0").split(), 3, f"{name} inertial xyz")
    mass = _finite(inertial.find("mass").get("value"), f"{name} mass", positive=True)
    node = inertial.find("inertia")
    tensor = (
        (float(node.get("ixx")), float(node.get("ixy")), float(node.get("ixz"))),
        (float(node.get("ixy")), float(node.get("iyy")), float(node.get("iyz"))),
        (float(node.get("ixz")), float(node.get("iyz")), float(node.get("izz"))),
    )
    return mass, xyz, _tensor(tensor, f"{name} inertia")


def _validate_urdf(
    urdf_path: Path,
    configured_links: Mapping[str, RigidBodyInertial],
    configured_drives: Mapping[str, JointDrive],
) -> None:
    root = ET.parse(urdf_path).getroot()
    if root.tag != "robot" or root.get("name") != "fanuc_m710id_70":
        raise ValueError("expanded official URDF has the wrong robot identity")
    links = {str(node.get("name")): node for node in root.findall("link")}
    for name in ROBOT_LINK_NAMES:
        if name not in links:
            raise ValueError(f"expanded official URDF is missing {name}")
        mass, com, inertia = _urdf_inertial(links[name], name)
        configured = configured_links[name]
        if not np.isclose(mass, configured.mass_kg, atol=1e-12, rtol=0.0):
            raise ValueError(f"configured {name} mass differs from official URDF")
        if not np.allclose(com, configured.com_xyz_m, atol=1e-12, rtol=0.0):
            raise ValueError(f"configured {name} COM differs from official URDF")
        if not np.allclose(inertia, configured.inertia_tensor_com_kg_m2, atol=1e-12, rtol=0.0):
            raise ValueError(f"configured {name} inertia differs from official URDF")
        collision = links[name].find("collision/geometry/mesh")
        visual = links[name].find("visual/geometry/mesh")
        if collision is None or visual is None:
            raise ValueError(f"official {name} must retain visual and collision meshes")
    joints = {str(node.get("name")): node for node in root.findall("joint")}
    for index, name in enumerate(ACTIVE_JOINT_NAMES):
        if name not in joints or joints[name].get("type") != "revolute":
            raise ValueError(f"official URDF is missing independent revolute joint {name}")
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"official URDF joint {name} has no finite limit")
        configured = configured_drives[name]
        if not np.isclose(float(limit.get("effort")), configured.effort_limit_nm, atol=1e-12, rtol=0.0):
            raise ValueError(f"configured {name} effort differs from official URDF")
        if not np.isclose(float(limit.get("velocity")), configured.velocity_limit_rad_s, atol=1e-12, rtol=0.0):
            raise ValueError(f"configured {name} velocity differs from official URDF")
        if not np.isclose(configured.effort_limit_nm, EXPECTED_EFFORT[index], atol=1e-12):
            raise ValueError(f"unexpected official effort for {name}")
        if not np.isclose(configured.velocity_limit_rad_s, EXPECTED_VELOCITY[index], atol=1e-12):
            raise ValueError(f"unexpected official velocity for {name}")
    required_fixed = {"flange", "fanuc_flange", "tool0", "wbase"}
    if not required_fixed.issubset(links):
        raise ValueError(f"expanded official URDF is missing fixed frames {sorted(required_fixed - set(links))}")
    if "mimic" in {child.tag for joint in joints.values() for child in joint}:
        raise ValueError("official six-axis chain must not introduce a mimic joint")


def load_m710id70_official_dynamics(path: str | Path) -> M710EngineeringDynamicsConfig:
    config_path = Path(path).resolve()
    data = _mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")), "official dynamics")
    if data.get("schema_version") != SCHEMA_VERSION or data.get("model_id") != "fanuc_m710id_70":
        raise ValueError("unsupported official M-710 dynamics schema/model")
    qualification = _mapping(data.get("qualification"), "qualification")
    if (
        qualification.get("status") != OFFICIAL_STATUS
        or qualification.get("simulation_input_accepted") is not True
        or qualification.get("machine_qualified") is not False
    ):
        raise ValueError("official-model simulation acceptance and machine qualification must remain separate")
    source = _mapping(data.get("official_model"), "official_model")
    if source.get("commit") != OFFICIAL_COMMIT or source.get("license") != "Apache-2.0":
        raise ValueError("official source commit/license drift")

    sources = _mapping(data.get("sources"), "sources")
    source_paths = {name: _resolve(config_path.parent, value, f"sources.{name}") for name, value in sources.items()}
    model = _mapping(yaml.safe_load(source_paths["robot_model_config"].read_text(encoding="utf-8")), "robot model")
    if model.get("model_revision") != "fanuc_description_v2p3p0_fb40c980_official_model_v1":
        raise ValueError("robot model config is not bound to the fixed official source")
    _validate_source_coherence(
        source_paths,
        {
            "robot_model_config": model,
            "workcell_layout": _mapping(
                yaml.safe_load(source_paths["workcell_layout"].read_text(encoding="utf-8")),
                "workcell layout",
            ),
            "gripper_asset_manifest": _mapping(
                yaml.safe_load(source_paths["gripper_asset_manifest"].read_text(encoding="utf-8")),
                "gripper asset manifest",
            ),
        },
    )

    robot = _mapping(data.get("robot"), "robot")
    link_data = _mapping(robot.get("links"), "robot.links")
    if tuple(link_data) != ROBOT_LINK_NAMES:
        raise ValueError("official robot links must be ordered base_link through J6_link")
    links: dict[str, RigidBodyInertial] = {}
    for name in ROBOT_LINK_NAMES:
        item = _mapping(link_data[name], f"robot.links.{name}")
        links[name] = RigidBodyInertial(
            name,
            _finite(item.get("mass_kg"), f"{name}.mass_kg", positive=True),
            _vector(item.get("com_xyz_m"), 3, f"{name}.com_xyz_m"),  # type: ignore[arg-type]
            _tensor(item.get("inertia_tensor_com_kg_m2"), f"{name}.inertia"),
            "FANUC_OFFICIAL_DESCRIPTION_FIXED_COMMIT",
        )
    robot_mass = sum(item.mass_kg for item in links.values())
    if not np.isclose(robot_mass, 580.347, atol=1e-12, rtol=0.0):
        raise ValueError(f"official link masses must sum to 580.347 kg, got {robot_mass}")

    drive_data = _mapping(robot.get("joint_drives"), "robot.joint_drives")
    if tuple(drive_data) != ACTIVE_JOINT_NAMES:
        raise ValueError("official joint drives must be ordered J1 through J6")
    drives: dict[str, JointDrive] = {}
    for name in ACTIVE_JOINT_NAMES:
        item = _mapping(drive_data[name], f"robot.joint_drives.{name}")
        drives[name] = JointDrive(
            name,
            _finite(item.get("effort_limit_nm"), f"{name}.effort", positive=True),
            _finite(item.get("velocity_limit_rad_s"), f"{name}.velocity", positive=True),
            _finite(item.get("stiffness_nm_rad"), f"{name}.stiffness", positive=True),
            _finite(item.get("damping_nm_s_rad"), f"{name}.damping", positive=True),
        )
    _validate_urdf(source_paths["robot_urdf"], links, drives)

    tool_source = load_tool_config(source_paths["tool_config"])
    tool = ToolDynamics(
        tool_source.name,
        tool_source.mass_kg,
        tool_source.com_xyz_m,
        _tensor(tool_source.inertia_tensor_com_kg_m2, "tool inertia"),
        "J6_link",
        FIXED_TOOL_MASS_POLICY,
        "PROJECT_ENGINEERING_MASS_MODEL",
        tool_source.mass_properties_source,
        False,
        tool_source.config_path,
        tool_source.config_hash,
    )
    cartons_data = _mapping(data.get("cartons"), "cartons")
    cartons = CartonDynamics(
        int(cartons_data["count"]),
        _finite(cartons_data["mass_kg_each"], "carton mass", positive=True),
        _vector(cartons_data["size_xyz_m"], 3, "carton size"),  # type: ignore[arg-type]
        _vector(cartons_data["com_xyz_m"], 3, "carton COM"),  # type: ignore[arg-type]
        _tensor(cartons_data["inertia_tensor_com_kg_m2"], "carton inertia"),
    )
    if cartons.count != 40 or cartons.mass_kg_each != 42.5:
        raise ValueError("official execution must retain all 40 cartons at 42.5 kg")

    simulation_data = _mapping(data.get("simulation"), "simulation")
    backend = str(simulation_data.get("execution_backend", "physx_cpu"))
    device = str(simulation_data.get("device", "cpu"))
    broadphase = str(simulation_data.get("broadphase_type", "MBP"))
    gpu_dynamics = simulation_data.get("gpu_dynamics_enabled", False)
    fabric = simulation_data.get("fabric_enabled", True)
    ccd = simulation_data.get("ccd_enabled", True)
    contact_offset_m = _finite(
        simulation_data.get("contact_offset_m"), "simulation contact offset", positive=True
    )
    rest_offset_m = _finite(
        simulation_data.get("rest_offset_m"), "simulation rest offset"
    )
    if not all(isinstance(value, bool) for value in (gpu_dynamics, fabric, ccd)):
        raise ValueError("simulation GPU dynamics, Fabric, and CCD flags must be boolean")
    valid_backend = (
        (backend, device, broadphase, gpu_dynamics, fabric, ccd)
        == ("physx_cpu", "cpu", "MBP", False, True, True)
    )
    if not valid_backend:
        raise ValueError("simulation execution backend fields are inconsistent")
    if contact_offset_m != 0.010 or rest_offset_m != 0.0:
        raise ValueError(
            "official execution requires the explicit 10 mm predictive contact offset "
            "and zero physical rest offset"
        )
    simulation = SimulationSettings(
        _vector(simulation_data["gravity_world_m_s2"], 3, "gravity"),  # type: ignore[arg-type]
        _finite(simulation_data["physics_time_step_s"], "physics dt", positive=True),
        int(simulation_data["solver_position_iterations"]),
        int(simulation_data["solver_velocity_iterations"]),
        backend,
        device,
        broadphase,
        gpu_dynamics,
        fabric,
        ccd,
        contact_offset_m,
        rest_offset_m,
    )
    environment_data = _mapping(data.get("environment"), "environment")
    environment = PhysicalEnvironment(
        float(environment_data["floor_z_m"]),
        float(environment_data["right_wall_y_m"]),
        float(environment_data["left_wall_y_m"]),
        str(environment_data["floor_material"]),
        str(environment_data["side_wall_material"]),
        str(environment_data["side_wall_extent_status"]),
    )
    contacts = MappingProxyType(
        {
            name: ContactMaterial(name, float(item["static_friction"]), float(item["dynamic_friction"]), float(item["restitution"]))
            for name, item in _mapping(data.get("contacts"), "contacts").items()
        }
    )
    damping = MappingProxyType(
        {
            name: RigidBodyDamping(name, float(item["linear_damping_s_inv"]), float(item["angular_damping_s_inv"]))
            for name, item in _mapping(data.get("damping"), "damping").items()
        }
    )
    settling_data = _mapping(data.get("settling"), "settling")
    settling = SettlingCriteria(*(float(settling_data[name]) for name in (
        "maximum_settle_time_s", "required_stable_duration_s", "max_linear_speed_m_s",
        "max_angular_speed_rad_s", "max_position_drift_m", "max_penetration_m",
    )))
    conveyor_data = _mapping(data.get("conveyors"), "conveyors")
    conveyors = MappingProxyType(
        {
            name: ConveyorSurfaceMotion(
                name, bool(item["enabled"]), _vector(item["direction_world"], 3, f"{name} direction"),
                float(item["speed_m_s"]), str(item["material"]),
            )
            for name, item in _mapping(conveyor_data.get("surfaces"), "conveyor surfaces").items()
        }
    )
    vacuum_data = _mapping(data.get("vacuum_attachment"), "vacuum_attachment")
    vacuum = IdealIndependentCupAttachment(
        suction_mode=str(vacuum_data["suction_mode"]),
        holding_capacity_assumption=str(vacuum_data["holding_capacity_assumption"]),
        physical_cup_count=int(vacuum_data["physical_cup_count"]),
        cup_rows=int(vacuum_data["cup_rows"]),
        cup_columns=int(vacuum_data["cup_columns"]),
        cup_pitch_m=_vector(vacuum_data["cup_pitch_m"], 2, "cup pitch"),  # type: ignore[arg-type]
        cup_radius_m=float(vacuum_data["cup_radius_m"]),
        cup_compression_m=float(vacuum_data["cup_compression_m"]),
        require_nonempty_geometric_contact=bool(vacuum_data["require_nonempty_geometric_contact"]),
        require_validated_contact_endpoint=bool(vacuum_data["require_validated_contact_endpoint"]),
        require_actual_fk_contact_recheck=bool(vacuum_data["require_actual_fk_contact_recheck"]),
        require_target_identity_match=bool(vacuum_data["require_target_identity_match"]),
        teleport_payload_on_attach=bool(vacuum_data["teleport_payload_on_attach"]),
        constraint_type=str(vacuum_data["constraint_type"]),
        enforce_vacuum_force_capacity=bool(vacuum_data["enforce_vacuum_force_capacity"]),
        enforce_vacuum_break_force=bool(vacuum_data["enforce_vacuum_break_force"]),
        enforce_vacuum_break_torque=bool(vacuum_data["enforce_vacuum_break_torque"]),
        max_attachment_gap_m=float(vacuum_data["max_attachment_gap_m"]),
        max_contact_penetration_m=float(vacuum_data["max_contact_penetration_m"]),
        max_normal_misalignment_rad=float(vacuum_data["max_normal_misalignment_rad"]),
    )
    if (
        vacuum.suction_mode != "ideal_independent_cups"
        or vacuum.physical_cup_count != 72
        or not vacuum.require_nonempty_geometric_contact
        or not vacuum.require_validated_contact_endpoint
        or not vacuum.require_actual_fk_contact_recheck
        or not vacuum.require_target_identity_match
        or vacuum.teleport_payload_on_attach
        or vacuum.enforce_vacuum_force_capacity
        or vacuum.enforce_vacuum_break_force
        or vacuum.enforce_vacuum_break_torque
    ):
        raise ValueError("ideal independent-cup attachment policy was weakened or force-gated")

    source_hashes = {"dynamics_config": sha256_file(config_path), **{name: sha256_file(item) for name, item in sorted(source_paths.items())}}
    fingerprint = _canonical_hash({"schema_version": SCHEMA_VERSION, "configuration": data, "source_file_hashes": source_hashes})
    frozen_data = dict(data)
    frozen_data["cache_identity_components"] = {
        "included": ["dynamics_config", "official_urdf", "official_mesh_manifest", "robot_link_inertials", "joint_drives", "ideal_independent_cups", "scene_physics"],
        "external_replay_contract_components": ["official_visual_meshes", "official_collision_meshes", "rigid_tool_compound", "isaac_importer_settings"],
    }
    return M710EngineeringDynamicsConfig(
        config_path=config_path,
        qualification_status=OFFICIAL_STATUS,
        machine_qualified=False,
        mechanical_weight_kg=580.347,
        manufacturer_evidence=_freeze(dict(source)),
        robot_links=MappingProxyType(links),
        joint_drives=MappingProxyType(drives),
        tool=tool,
        cartons=cartons,
        simulation=simulation,
        environment=environment,
        contacts=contacts,
        damping=damping,
        settling=settling,
        conveyors=conveyors,
        exclusive_surface_drive_at_transfer=bool(conveyor_data["exclusive_surface_drive_at_transfer"]),
        vacuum_attachment=vacuum,  # type: ignore[arg-type]
        source_file_hashes=MappingProxyType(source_hashes),
        fingerprint=fingerprint,
        data=_freeze(frozen_data),
    )


__all__ = [
    "IdealIndependentCupAttachment",
    "OFFICIAL_COMMIT",
    "OFFICIAL_STATUS",
    "SCHEMA_VERSION",
    "load_m710id70_official_dynamics",
]
