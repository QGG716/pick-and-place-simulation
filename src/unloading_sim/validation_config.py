"""Strict, source-traced acceptance configuration, independent of simulator UI."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from importlib import metadata
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml

from .geometry import make_transform, rotation_matrix_from_rpy
from .identity import load_tool_config
from .robot import URDFRobot
from .timing import JointMotionLimits

DEFAULT = Path(__file__).resolve().parents[2] / "configs/validation/m710id70_v3.yaml"
MODEL_ASSET_MANIFEST_SCHEMA = "m710_model_assets_v1"
CACHE_IDENTITY_SCHEMA = "m710_validation_cache_identity_v2"
COLLISION_BACKEND_ID = "cpu_urdf_primitive_obb_capsule_v1"


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


def _canonical_digest(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _asset_entry(role: str, path: Path, declared_path: str) -> dict[str, str]:
    resolved = path.resolve()
    try:
        content = resolved.read_bytes()
    except OSError as exc:
        raise ValueError(f"required model asset {role} is unreadable: {resolved}: {exc}") from exc
    if not resolved.is_file():
        raise ValueError(f"required model asset {role} is not a file: {resolved}")
    return {"role": role, "declared_path": str(declared_path).replace("\\", "/"),
            "resolved_path": str(resolved), "sha256": hashlib.sha256(content).hexdigest()}


def _yaml_asset_entry(role: str, path: Path, declared_path: str) -> dict[str, str]:
    entry = _asset_entry(role, path, declared_path)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"required model asset {role} is not readable YAML: {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"required model asset {role} must contain a YAML mapping: {path}")
    entry["semantic_sha256"] = _canonical_digest(document)
    return entry


def _runtime_versions() -> dict[str, Any]:
    packages = {}
    for name in ("numpy", "PyYAML"):
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            packages[name] = "NOT_INSTALLED"
    return {"python": sys.version.split()[0], "python_implementation": sys.implementation.name,
            "packages": packages, "collision_backend": COLLISION_BACKEND_ID}


def _cache_runtime_identity(runtime: dict[str, Any]) -> dict[str, Any]:
    """Only numerical runtime versions participate in cache validity.

    YAML parser changes are reflected in ``effective_config`` and therefore do
    not independently invalidate a task.  Python major/minor and NumPy may
    change deterministic numerical results, so they are validity dimensions.
    """
    major, minor, *_ = runtime["python"].split(".")
    return {"python_implementation": runtime["python_implementation"],
            "python_major_minor": f"{major}.{minor}",
            "numpy": runtime["packages"]["numpy"],
            "collision_backend": runtime["collision_backend"]}


def _field(data: dict[str, Any], dotted: str) -> Any:
    value: Any = data
    for key in dotted.split("."):
        if not isinstance(value, dict) or key not in value:
            raise ValueError(f"{dotted}: field is required")
        value = value[key]
    return value


def _number(data: dict[str, Any], dotted: str, *, minimum: float | None = None,
            strict_minimum: bool = False, maximum: float | None = None) -> float:
    value = _field(data, dotted)
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ValueError(f"{dotted}: must be a finite number, not {type(value).__name__}")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{dotted}: must be finite")
    if minimum is not None and (result <= minimum if strict_minimum else result < minimum):
        relation = ">" if strict_minimum else ">="
        raise ValueError(f"{dotted}: must be {relation} {minimum}")
    if maximum is not None and result > maximum:
        raise ValueError(f"{dotted}: must be <= {maximum}")
    return result


def _integer(data: dict[str, Any], dotted: str, *, minimum: int = 0) -> int:
    value = _field(data, dotted)
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{dotted}: must be an integer, not {type(value).__name__}")
    if int(value) < minimum:
        raise ValueError(f"{dotted}: must be >= {minimum}")
    return int(value)


def _vector(data: dict[str, Any], dotted: str, size: int, *, positive: bool = False,
            nonnegative: bool = False, nonempty: bool = False) -> np.ndarray:
    value = _field(data, dotted)
    if isinstance(value, (str, bytes, dict)):
        raise ValueError(f"{dotted}: must be a sequence of {size if size else 'finite'} numbers")
    if any(isinstance(item, bool) for item in value):
        raise ValueError(f"{dotted}: boolean values are not numeric configuration values")
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{dotted}: must contain finite numbers") from exc
    if result.ndim != 1 or (size and result.shape != (size,)) or (nonempty and not len(result)):
        expected = str(size) if size else "one or more"
        raise ValueError(f"{dotted}: must contain {expected} values")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{dotted}: must contain only finite values")
    if positive and np.any(result <= 0):
        raise ValueError(f"{dotted}: values must be strictly positive")
    if nonnegative and np.any(result < 0):
        raise ValueError(f"{dotted}: values must be nonnegative")
    return result


def _validate_transform(matrix: np.ndarray, dotted: str) -> None:
    value = np.asarray(matrix, dtype=float)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError(f"{dotted}: must be a finite 4x4 rigid transform")
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
        raise ValueError(f"{dotted}: invalid homogeneous bottom row")
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0) \
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-9, rtol=0):
        raise ValueError(f"{dotted}: rotation must be proper orthonormal SO(3)")


def _validate_data(data: dict[str, Any]) -> None:
    for name in ("robot.installation_xyz_m", "robot.installation_rpy_rad",
                 "chassis.world_xyz_m", "chassis.world_rpy_rad", "scene.box_com_fraction"):
        _vector(data, name, 3)
    _vector(data, "robot.home_joints", 6)
    for name in ("chassis.size_xyz_m", "lift.column_size_xy_m", "tool.cup_pitch_m"):
        _vector(data, name, 3 if name == "chassis.size_xyz_m" else 2, positive=True)
    _vector(data, "lift.scan_heights_m", 0, nonnegative=True, nonempty=True)
    box_sizes = _field(data, "scene.box_sizes_m")
    if not isinstance(box_sizes, dict) or not box_sizes:
        raise ValueError("scene.box_sizes_m: must be a non-empty mapping")
    for name, values in box_sizes.items():
        if isinstance(values, (str, bytes, dict)):
            raise ValueError(f"scene.box_sizes_m.{name}: must contain three finite positive dimensions")
        if any(isinstance(item, bool) for item in values):
            raise ValueError(f"scene.box_sizes_m.{name}: boolean dimensions are invalid")
        array = np.asarray(values, dtype=float)
        if array.shape != (3,) or not np.all(np.isfinite(array)) or np.any(array <= 0):
            raise ValueError(f"scene.box_sizes_m.{name}: must contain three finite positive dimensions")
    for name in ("scene.trailer_x_limits_m", "scene.random_front_offset_limits_m",
                 "scene.random_gap_limits_m"):
        limits = _vector(data, name, 2)
        if limits[0] > limits[1] or (name == "scene.trailer_x_limits_m" and limits[0] == limits[1]):
            raise ValueError(f"{name}: lower bound must be {'<' if name == 'scene.trailer_x_limits_m' else '<='} upper bound")
    if np.any(_vector(data, "scene.random_gap_limits_m", 2) < 0):
        raise ValueError("scene.random_gap_limits_m: gaps must be nonnegative")
    for name in ("scene.front_x_m", "scene.random_start_y_m", "scene.random_stop_y_m",
                 "scene.random_max_edge_y_m"):
        _number(data, name)
    if _field(data, "scene.random_start_y_m") >= _field(data, "scene.random_stop_y_m"):
        raise ValueError("scene random Y bounds: random_start_y_m must be < random_stop_y_m")
    if _field(data, "scene.random_max_edge_y_m") <= 0:
        raise ValueError("scene.random_max_edge_y_m: must be strictly positive")
    if np.any(np.abs(_vector(data, "scene.box_com_fraction", 3)) > .5):
        raise ValueError("scene.box_com_fraction: components must be within [-0.5, 0.5]")

    positive = (
        "scene.trailer_width_m", "scene.trailer_height_m", "scene.wall_thickness_m",
        "scene.box_mass_kg", "scene.grid_step_m", "scene.row_pitch_m",
        "lift.step_m", "lift.speed_m_s", "tool.cup_radius_m",
        "planning.contact_tolerance_m", "planning.support_tolerance_m",
        "planning.extraction_scan_step_m", "planning.maximum_extraction_m",
        "planning.ik_position_tolerance_m", "planning.ik_orientation_tolerance_rad",
        "planning.ik_damping", "planning.ik_max_step_rad", "planning.ik_orientation_weight",
        "planning.cartesian_max_branch_step_rad", "planning.cartesian_orientation_tolerance_rad",
        "planning.cartesian_orientation_step_rad", "planning.maximum_jacobian_condition",
        "planning.edge_resolution_rad", "planning.cartesian_step_m", "planning.rrt_step_rad",
        "conveyor.cross_leg_size_xy_m", "conveyor.longitudinal_leg_width_m",
        "conveyor.deck_thickness_m", "conveyor.movement_step_m", "conveyor.extension_speed_m_s",
        "conveyor.lift_speed_m_s", "conveyor.belt_speed_m_s", "execution.minimum_segment_seconds",
        "execution.sample_period_s")
    for name in positive:
        if name.endswith("size_xy_m"):
            _vector(data, name, 2, positive=True)
        else:
            _number(data, name, minimum=0, strict_minimum=True)
    for name in ("lift.height_m", "planning.pregrasp_standoff_m",
                 "planning.extraction_free_clearance_m", "planning.collision_margin_m",
                 "planning.initial_proximity_monotonic_tolerance_m",
                 "planning.support_edge_clearance_m", "planning.support_relation_tolerance_m",
                 "planning.joint_margin_rad", "planning.radial_guard_tolerance_m",
                 "tool.suction_edge_margin_m", "scene.neighbor_gap_m",
                 "conveyor.maximum_extension_m", "conveyor.minimum_stack_surface_clearance_m",
                 "conveyor.minimum_receiving_surface_z_m", "conveyor.maximum_receiving_surface_z_m",
                 "conveyor.upper_stack_bottom_clearance_m", "conveyor.fixed_extension_m",
                 "conveyor.fixed_z_m", "execution.vacuum_establish_s", "execution.release_s"):
        _number(data, name, minimum=0)
    for name in ("planning.grasp_face_offset_candidates_m", "planning.grasp_tilt_candidates_rad",
                 "planning.escape_rotation_candidates_rad", "planning.roll_candidates_deg"):
        _vector(data, name, 0, nonempty=True)
    for name in ("scene.random_type_a_probability", "scene.random_skip_probability",
                 "planning.rrt_goal_bias", "planning.support_relation_minimum_overlap_ratio"):
        _number(data, name, minimum=0, maximum=1)
    for name, minimum in (("tool.cup_rows", 1), ("tool.cup_columns", 1),
                          ("tool.minimum_sealed_cups", 1), ("scene.regular_rows", 1),
                          ("scene.regular_columns", 1), ("planning.seed", 0),
                          ("planning.continuous_seed", 0),
                          ("planning.ik_iterations", 1), ("planning.ik_restarts", 0),
                          ("planning.grasp_downstream_candidate_limit_per_strategy", 1),
                          ("planning.escape_path_attempt_limit", 0), ("planning.rrt_iterations", 1)):
        _integer(data, name, minimum=minimum)
    random_seeds = _field(data, "scene.random_seeds")
    if isinstance(random_seeds, (str, bytes, dict)) or not random_seeds:
        raise ValueError("scene.random_seeds: must be a non-empty integer sequence")
    for index, seed in enumerate(random_seeds):
        if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)) or seed < 0:
            raise ValueError(f"scene.random_seeds[{index}]: must be a nonnegative integer")
    if _field(data, "tool.minimum_sealed_cups") > _field(data, "tool.cup_rows") * _field(data, "tool.cup_columns"):
        raise ValueError("tool.minimum_sealed_cups: cannot exceed cup_rows * cup_columns")
    if _field(data, "conveyor.minimum_receiving_surface_z_m") > _field(data, "conveyor.maximum_receiving_surface_z_m"):
        raise ValueError("conveyor receiving Z bounds: minimum must be <= maximum")
    _number(data, "conveyor.longitudinal_leg_nominal_length_m", minimum=0, strict_minimum=True)
    if not 0 <= _field(data, "conveyor.fixed_extension_m") <= _field(data, "conveyor.maximum_extension_m"):
        raise ValueError("conveyor.fixed_extension_m: must be within [0, maximum_extension_m]")
    if not (_field(data, "conveyor.minimum_receiving_surface_z_m") <= _field(data, "conveyor.fixed_z_m")
            <= _field(data, "conveyor.maximum_receiving_surface_z_m")):
        raise ValueError("conveyor.fixed_z_m: must be within receiving surface Z bounds")
    if _field(data, "planning.maximum_extraction_m") < _field(data, "planning.extraction_scan_step_m"):
        raise ValueError("planning.maximum_extraction_m: must be >= extraction_scan_step_m")
    profile = _field(data, "execution.profile")
    profiles = _field(data, "execution.profiles")
    if not isinstance(profile, str) or profile not in profiles:
        raise ValueError("execution.profile: must name an execution.profiles entry")
    for name, scales in profiles.items():
        if not isinstance(scales, dict):
            raise ValueError(f"execution.profiles.{name}: must be a mapping")
        for field in ("velocity_scale", "acceleration_scale", "jerk_scale"):
            value = scales.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f"execution.profiles.{name}.{field}: must be finite in (0, 1]")


def _validate_model(model: dict[str, Any], robot) -> None:
    if not isinstance(model, dict):
        raise ValueError("robot.model_config: must contain a YAML mapping")
    dof = robot.dof
    for name in ("joint_velocity_limits_rad_s",):
        try:
            values = np.asarray(model.get(name), dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"robot.model_config.{name}: must contain numeric values") from exc
        if values.shape != (dof,) or not np.all(np.isfinite(values)) or np.any(values <= 0):
            raise ValueError(f"robot.model_config.{name}: must contain {dof} finite positive values")
    for name in ("joint_acceleration_limits_rad_s2", "joint_jerk_limits_rad_s3"):
        entry = model.get(name)
        try:
            values = np.asarray(entry.get("values") if isinstance(entry, dict) else None, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"robot.model_config.{name}.values: must contain numeric values") from exc
        if values.shape != (dof,) or not np.all(np.isfinite(values)) or np.any(values <= 0):
            raise ValueError(f"robot.model_config.{name}.values: must contain {dof} finite positive values")
    for name in ("reach_m", "rated_payload_kg"):
        value = model.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"robot.model_config.{name}: must be finite and positive")
    configured = model.get("joint_position_limits_rad")
    if not isinstance(configured, dict):
        raise ValueError("robot.model_config.joint_position_limits_rad: must be a mapping")
    for name in robot.active_joint_names:
        try:
            limits = np.asarray(configured[name], dtype=float)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"robot.model_config.joint_position_limits_rad.{name}: invalid limits") from exc
        if limits.shape != (2,) or not np.all(np.isfinite(limits)) or limits[0] >= limits[1]:
            raise ValueError(f"robot.model_config.joint_position_limits_rad.{name}: lower must be finite and < upper")


def _build_asset_manifest(*, model_path: Path, model_declared: str, urdf_path: Path,
                          urdf_declared: str, tool, payload_path: Path,
                          payload_declared: str, robot, model: dict[str, Any]) -> dict[str, Any]:
    entries = sorted((
        _yaml_asset_entry("robot_model_config", model_path, model_declared),
        _asset_entry("robot_urdf", urdf_path, urdf_declared),
        _yaml_asset_entry("tool_config", tool.config_path, str(tool.config_path.name)),
        _yaml_asset_entry("payload_evidence", payload_path, payload_declared),
    ), key=lambda item: item["role"])
    semantic = {
        "schema_version": MODEL_ASSET_MANIFEST_SCHEMA,
        "assets": {entry["role"]: entry.get("semantic_sha256",entry["sha256"]) for entry in entries},
        "collision_backend": COLLISION_BACKEND_ID,
        "collision_geometry": "URDF collision primitives plus deterministic link capsules and tool OBB",
        "self_collision_policy": {
            "adjacent_chain_pairs_implicitly_excluded": True,
            "extra_link_index_pairs": [list(pair) for pair in sorted(robot.self_collision_exclusions)],
        },
        "tool_collision": {
            "size_task_xyz_m": robot.tool_collision_size.tolist(),
            "center_offset_m": robot.tool_collision_center_offset,
            "local_boxes": robot.tool_collision_local_boxes.tolist(),
        },
    }
    return {**semantic, "entries": entries, "semantic_fingerprint_sha256": _canonical_digest(semantic),
            "declared_but_not_loaded": {
                "srdf_path": model.get("kinematics",{}).get("srdf_path"),
                "collision_meshes": list(model.get("collision_meshes",[])),
                "reason": "the active CPU validation backend consumes URDF primitives, capsules and tool OBB only",
            }}


@dataclass
class ValidationConfig:
    data: dict[str, Any]
    sources: dict[str, str]
    model: dict
    tool: Any
    model_path: Path
    urdf_path: Path
    config_path: Path
    asset_manifest: dict[str, Any]
    runtime_versions: dict[str, Any]

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
                "model_asset_manifest": self.asset_manifest,
                "runtime_versions": self.runtime_versions,
                "cache_validity_runtime": _cache_runtime_identity(self.runtime_versions),
                "motion_limits": {"velocity": self.motion_limits().effective_velocity.tolist(),
                                  "acceleration": self.motion_limits().effective_acceleration.tolist(),
                                  "jerk": self.motion_limits().effective_jerk.tolist()}}

    def cache_identity(self) -> dict[str, Any]:
        """Return path-independent semantic inputs that affect task validity."""
        robot = self.robot()
        return {"schema_version": CACHE_IDENTITY_SCHEMA,
                "effective_config": self.data,
                "model_asset_fingerprint_sha256": self.asset_manifest["semantic_fingerprint_sha256"],
                "cache_validity_runtime": _cache_runtime_identity(self.runtime_versions),
                "world_from_chassis": self.world_from_chassis().tolist(),
                "world_from_robot_mount": robot.base_transform.tolist(),
                "tool0_from_task_tcp": robot.tip_from_tcp.tolist(),
                "motion_limits": {"velocity": self.motion_limits().effective_velocity.tolist(),
                                  "acceleration": self.motion_limits().effective_acceleration.tolist(),
                                  "jerk": self.motion_limits().effective_jerk.tolist()}}


def load_validation_config(path: str | Path = DEFAULT) -> ValidationConfig:
    config_path = Path(path).resolve()
    data, sources = _read(config_path)
    template, allowed = _read(DEFAULT)
    if data.get("schema") != "m710_validation_v3":
        raise ValueError("unsupported validation configuration schema")
    unknown = set(sources) - set(allowed)
    missing = set(allowed) - set(sources)
    if unknown or missing:
        raise ValueError(f"unknown parameters: {sorted(unknown)}; missing parameters: {sorted(missing)}")
    _validate_data(data)
    def resource(key: str, value: str) -> Path:
        return (Path(sources[key]).parent / value).resolve()
    model_path = resource("robot.model_config", data["robot"]["model_config"])
    _asset_entry("robot_model_config", model_path, data["robot"]["model_config"])
    model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    if not isinstance(model, dict):
        raise ValueError(f"robot.model_config: {model_path} must contain a YAML mapping")
    payload_declared = model["payload_com_evidence"]
    if not isinstance(payload_declared, str) or not payload_declared.strip():
        raise ValueError("robot.model_config.payload_com_evidence: must be a non-empty path")
    evidence_path = DEFAULT.parents[2] / payload_declared
    _asset_entry("payload_evidence", evidence_path, payload_declared)
    evidence = yaml.safe_load(evidence_path.read_text(encoding="utf-8"))
    if not isinstance(evidence, dict):
        raise ValueError("robot payload evidence: must contain a YAML mapping")
    model["resolved_payload_evidence"] = evidence
    model["resolved_payload_evidence_path"] = str(evidence_path)
    configured_tool_path = resource("tool.config", data["tool"]["config"])
    _asset_entry("tool_config", configured_tool_path, data["tool"]["config"])
    tool = load_tool_config(configured_tool_path)
    if tool.mass_properties_reference_frame != "flange":
        raise ValueError("tool mass properties must be expressed in flange coordinates")
    urdf_path = resource("robot.urdf_path", data["robot"]["urdf_path"])
    _asset_entry("robot_urdf", urdf_path, data["robot"]["urdf_path"])
    runtime = _runtime_versions()
    placeholder = ValidationConfig(data, sources, model, tool, model_path, urdf_path,
                                   config_path, {}, runtime)
    robot = placeholder.robot()
    _validate_model(model, robot)
    _validate_transform(placeholder.world_from_chassis(), "chassis world transform")
    _validate_transform(robot.base_transform, "robot mount transform")
    _validate_transform(robot.tip_from_tcp, "tool TCP transform")
    manifest = _build_asset_manifest(model_path=model_path, model_declared=data["robot"]["model_config"],
        urdf_path=urdf_path, urdf_declared=data["robot"]["urdf_path"], tool=tool,
        payload_path=evidence_path, payload_declared=payload_declared, robot=robot, model=model)
    result = ValidationConfig(data, sources, model, tool, model_path, urdf_path,
                              config_path, manifest, runtime)
    result.motion_limits()  # Validate the selected execution profile now.
    limits = np.asarray([model["joint_position_limits_rad"][name] for name in robot.active_joint_names])
    if not np.allclose(robot.joint_limits, limits, atol=1e-9, rtol=0):
        raise ValueError("URDF and robot model joint limits disagree")
    return result
