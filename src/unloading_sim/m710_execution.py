"""Content-addressed preflight for the layout-bound M-710 dynamic replay.

This module remains CPU-only.  It joins the frozen workcell snapshot, supplied
CAD manifests, the strict single-carton motion audit and the independent
engineering dynamics model.  The result is deliberately useful even when one
of those inputs is not execution-qualified: blockers are serialized and the
Isaac adapter must refuse to launch instead of silently substituting proxies.
"""

from __future__ import annotations

from dataclasses import dataclass
import copy
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from .asset_audit import (
    audit_m710id70_official_model,
    load_and_audit_asset_manifest,
)
from .independent_cups import (
    IDEAL_INDEPENDENT_CUPS_MODE,
    M710_CUP_ZONE_COUNT,
)
from .layout_single_carton import (
    EXPECTED_TOP_CARTONS,
    RESULT_SCHEMA as MOTION_RESULT_SCHEMA,
    FrozenLayoutMotionInput,
    build_verified_motion_input,
    load_layout_motion_policy,
    motion_implementation_identity,
    run_layout_single_carton_audit,
)
from .layout_trajectory import LayoutTrajectoryConnector
from .m710_dynamics import (
    ROBOT_LINK_NAMES,
    M710EngineeringDynamicsConfig,
    load_m710id70_dynamics,
)
from .m710_replay_contract import (
    M710ReplayContractError,
    build_replay_input_binding,
    validate_trajectory_segment,
    verify_m710_preflight_contract,
)
from .workcell_layout import audit_initial_state, canonical_digest, sha256_file


EXECUTION_CONFIG_SCHEMA = "m710id70_dynamic_execution_v1"
PREFLIGHT_SCHEMA = "m710id70_dynamic_execution_preflight_v1"
INITIALIZATION_DIAGNOSTIC_SCHEMA = "m710id70_initialization_diagnostic_preflight_v1"
TOOL_RIGID_COLLISION_COVERAGE_REASON = "TOOL_RIGID_COLLISION_COVERAGE_NOT_PROVEN"
ROBOT_TOOL_MOUNT_CONTACT_SCOPE_REASON = "ROBOT_TOOL_MOUNT_CONTACT_SCOPE_NOT_QUALIFIED"
TOOL_RIGID_COLLISION_COVERAGE_PROVEN_STATUS = "CONSERVATIVE_RIGID_SOLID_COVERAGE_PROVEN"
# The current exact validator ignores J6_link against all 58 tool boxes.  This
# stays false until that rule is replaced by a dimensioned mounting-interface
# contact check; initialization preflight must expose the blocker even though
# it cannot create a valid scene snapshot.
ROBOT_TOOL_MOUNT_CONTACT_SCOPE_QUALIFIED = False
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "configs"
    / "simulation"
    / "m710id70_dynamic_execution_v1.yaml"
)
EXECUTION_IMPLEMENTATION_FILES = (
    "src/unloading_sim/asset_audit.py",
    "src/unloading_sim/independent_cups.py",
    "src/unloading_sim/isaac_bridge.py",
    "src/unloading_sim/m710_dynamics.py",
    "src/unloading_sim/m710_execution.py",
    "src/unloading_sim/m710_official_dynamics.py",
    "src/unloading_sim/m710_replay_physics.py",
    "src/unloading_sim/m710_replay_contract.py",
    "src/unloading_sim/qualification.py",
    "scripts/export_isaac_fanuc_replay.py",
    "scripts/isaacsim_fanuc_replay.py",
)
SIMULATION_BLOCKING_ASSET_ISSUES = frozenset(
    {
        "J3_FOLLOWS_J2_CONTROLLER_TO_KINEMATIC_MAPPING",
        "TOOL0_CLOCKING_180_DEG_ABOUT_COMMON_TOOL_Z",
        "PER_LINK_VISUAL_MESHES_MISSING",
        "PER_LINK_COLLISION_MESHES_MISSING",
        "FLANGE_CLOCKING_PROVISIONAL",
        "DYNAMIC_COLLISION_REPRESENTATION_NOT_QUALIFIED",
        TOOL_RIGID_COLLISION_COVERAGE_REASON,
        ROBOT_TOOL_MOUNT_CONTACT_SCOPE_REASON,
    }
)
MACHINE_QUALIFICATION_ASSET_WARNINGS = frozenset(
    {
        "LINK_MASS_CENTER_OF_MASS_AND_INERTIA_NOT_SUPPLIED",
        "CERTIFIED_JOINT_DRIVE_TORQUE_LIMITS_NOT_SUPPLIED",
        "SUPPLIED_ASSEMBLY_MATERIAL_AND_COMPONENT_MASSES_NOT_AVAILABLE",
        "ZONE_ASSIGNMENT_NOT_VENDOR_CONFIRMED",
        "VACUUM_FORCE_SHEAR_AND_PEEL_ENVELOPE_NOT_MEASURED",
    }
)


def _execution_implementation_identity(project_root: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for relative in EXECUTION_IMPLEMENTATION_FILES:
        source = project_root / relative
        if not source.is_file():
            raise FileNotFoundError(f"execution implementation source is missing: {relative}")
        result[relative] = sha256_file(source)
    return result


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _keys(value: Mapping[str, Any], expected: set[str], name: str) -> None:
    missing = expected - set(value)
    extra = set(value) - expected
    if missing or extra:
        raise ValueError(f"{name} keys mismatch: missing={sorted(missing)} extra={sorted(extra)}")


def _resolve(config_path: Path, value: Any, name: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty path")
    declared = Path(value)
    result = declared.resolve() if declared.is_absolute() else (config_path.parent / declared).resolve()
    if not result.is_file():
        raise FileNotFoundError(f"{name} is missing: {result}")
    return result


def _positive(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite and positive")
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _nonnegative(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite and non-negative")
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return result


def _positive_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or int(value) <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


@dataclass(frozen=True)
class M710ExecutionConfig:
    config_path: Path
    data: Mapping[str, Any]
    layout_validation_path: Path
    motion_policy_path: Path
    dynamics_path: Path
    robot_manifest_path: Path
    tool_manifest_path: Path
    fingerprint: str

    @property
    def project_root(self) -> Path:
        return self.config_path.parents[2]


def load_m710_execution_config(path: str | Path | None = None) -> M710ExecutionConfig:
    config_path = DEFAULT_CONFIG_PATH if path is None else Path(path).resolve()
    data = _mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")), "execution config")
    _keys(
        data,
        {
            "schema",
            "layout_validation_config",
            "motion_policy_config",
            "dynamics_config",
            "robot_cad_manifest",
            "tool_cad_manifest",
            "physics_boundary_patches",
            "replay",
            "qualification",
        },
        "execution config",
    )
    if data["schema"] != EXECUTION_CONFIG_SCHEMA:
        raise ValueError("unsupported M-710 execution config schema")
    boundary = _mapping(data["physics_boundary_patches"], "physics_boundary_patches")
    _keys(
        boundary,
        {"extent_basis", "robot_reach_m", "task_tcp_extension_m", "thickness_m"},
        "physics_boundary_patches",
    )
    if boundary["extent_basis"] != "robot_reachable_envelope_not_trailer_dimensions":
        raise ValueError("boundary patches must not claim an unknown trailer length or height")
    if not np.isclose(_positive(boundary["robot_reach_m"], "robot_reach_m"), 2.104):
        raise ValueError("boundary reach must retain the M-710iD/70 2.104 m input")
    if not np.isclose(_positive(boundary["task_tcp_extension_m"], "task_tcp_extension_m"), 0.250):
        raise ValueError("boundary tool extension must retain the 0.250 m task TCP")
    _positive(boundary["thickness_m"], "boundary thickness")

    replay = _mapping(data["replay"], "replay")
    _keys(
        replay,
        {
            "controller_period_s",
            "motion_limit_scale",
            "pre_grasp_controller_settle_s",
            "vacuum_establish_s",
            "release_hold_s",
            "post_release_settle_s",
            "normal_time_scale",
            "output_width_px",
            "output_height_px",
            "output_fps",
            "camera_mode",
            "command_mode",
        },
        "replay",
    )
    for name in (
        "controller_period_s",
        "motion_limit_scale",
        "vacuum_establish_s",
        "normal_time_scale",
    ):
        _positive(replay[name], f"replay.{name}")
    for name in ("pre_grasp_controller_settle_s", "release_hold_s", "post_release_settle_s"):
        _nonnegative(replay[name], f"replay.{name}")
    for name in ("output_width_px", "output_height_px", "output_fps"):
        _positive_integer(replay[name], f"replay.{name}")
    if float(replay["motion_limit_scale"]) > 1.0 or float(replay["normal_time_scale"]) != 1.0:
        raise ValueError("replay must use a finite scale in (0,1] and normal-time video")
    if replay["command_mode"] != "initialize_once_then_force_limited_position_drive_targets":
        raise ValueError("replay command mode may not reset positions during motion")
    if replay["camera_mode"] != "fixed_overview_with_contact_and_place_keyframes":
        raise ValueError("replay camera mode must preserve the required overview and keyframes")

    qualification = _mapping(data["qualification"], "qualification")
    required_true = {
        "require_complete_collision_checked_motion_plan",
        "require_robot_per_link_cad_visuals",
        "require_robot_per_link_cad_collisions",
        "require_tool_dynamic_collision_qualification",
        "require_resolved_j3_coupling",
        "require_resolved_tool0_clocking",
        "require_real_base_mounting_extent",
    }
    required_false = {
        "allow_proxy_collision_for_execution",
        "allow_carton_pose_teleport_after_initialization",
        "allow_joint_pose_reset_after_initialization",
        "allow_infinite_drive_or_attachment_limits",
    }
    _keys(qualification, required_true | required_false, "qualification")
    if any(qualification[name] is not True for name in required_true) or any(
        qualification[name] is not False for name in required_false
    ):
        raise ValueError("execution qualification gates may not be weakened")

    paths = {
        "layout": _resolve(config_path, data["layout_validation_config"], "layout_validation_config"),
        "motion": _resolve(config_path, data["motion_policy_config"], "motion_policy_config"),
        "dynamics": _resolve(config_path, data["dynamics_config"], "dynamics_config"),
        "robot": _resolve(config_path, data["robot_cad_manifest"], "robot_cad_manifest"),
        "tool": _resolve(config_path, data["tool_cad_manifest"], "tool_cad_manifest"),
    }
    motion = load_layout_motion_policy(paths["motion"])
    if motion.layout_validation.config_path != paths["layout"]:
        raise ValueError("motion policy and execution config must reference the same layout validation config")
    fingerprint = canonical_digest(
        {
            "config": data,
            "source_sha256": {name: sha256_file(item) for name, item in sorted(paths.items())},
        }
    )
    return M710ExecutionConfig(
        config_path,
        copy.deepcopy(dict(data)),
        paths["layout"],
        paths["motion"],
        paths["dynamics"],
        paths["robot"],
        paths["tool"],
        fingerprint,
    )


def _primitive(record: Mapping[str, Any], *, dynamic: bool, mass: float | None = None,
               inertia: tuple[tuple[float, ...], ...] | None = None, **extra: Any) -> dict[str, Any]:
    pose = np.asarray(record["pose_world"], dtype=float)
    half = np.asarray(record["half_extents_m"], dtype=float)
    if pose.shape != (4, 4) or half.shape != (3,) or np.any(half <= 0.0):
        raise ValueError(f"invalid frozen primitive {record.get('name')}")
    result = {
        "name": str(record["name"]),
        "category": str(record["category"]),
        "center_m": pose[:3, 3].tolist(),
        "size_m": (2.0 * half).tolist(),
        "rotation_matrix": pose[:3, :3].tolist(),
        "dynamic": bool(dynamic),
        **extra,
    }
    if dynamic:
        if mass is None or inertia is None:
            raise ValueError("dynamic primitive requires explicit mass and inertia")
        result["mass_kg"] = float(mass)
        result["inertia_at_com_kg_m2"] = [list(row) for row in inertia]
    return result


def _boundary_primitives(
    scene: FrozenLayoutMotionInput,
    dynamics: M710EngineeringDynamicsConfig,
    execution: M710ExecutionConfig,
) -> list[dict[str, Any]]:
    boundary = execution.data["physics_boundary_patches"]
    radius = float(boundary["robot_reach_m"]) + float(boundary["task_tcp_extension_m"])
    thickness = float(boundary["thickness_m"])
    mount = np.asarray(scene.snapshot["robot"]["world_from_mount"], dtype=float)[:3, 3]
    # Cover every possible TCP point and all confirmed fixed/carton geometry.
    boxes = [*scene.fixed_components, *scene.cartons]
    x_min = min(float(mount[0] - radius), *(float(np.min(box.corners()[:, 0])) for box in boxes))
    x_max = max(float(mount[0] + radius), *(float(np.max(box.corners()[:, 0])) for box in boxes))
    z_max = max(float(mount[2] + radius), *(float(np.max(box.corners()[:, 2])) for box in boxes))
    floor_z = dynamics.environment.floor_z_m
    right = dynamics.environment.right_wall_y_m
    left = dynamics.environment.left_wall_y_m
    width = left - right
    extent_note = "FINITE_SOLVER_PATCH_COVERS_ROBOT_REACHABLE_ENVELOPE_NOT_A_TRAILER_DIMENSION"
    identity = np.eye(3).tolist()
    return [
        {
            "name": "physical_floor_reachable_patch",
            "category": "floor",
            "center_m": [0.5 * (x_min + x_max), 0.5 * (left + right), floor_z - 0.5 * thickness],
            "size_m": [x_max - x_min, width, thickness],
            "rotation_matrix": identity,
            "dynamic": False,
            "material": dynamics.environment.floor_material,
            "boundary": {"axis": "z", "value_m": floor_z, "inside": "+", "extent_status": extent_note},
        },
        {
            "name": "physical_right_sidewall_reachable_patch",
            "category": "side_wall",
            "center_m": [0.5 * (x_min + x_max), right - 0.5 * thickness, 0.5 * (floor_z + z_max)],
            "size_m": [x_max - x_min, thickness, z_max - floor_z],
            "rotation_matrix": identity,
            "dynamic": False,
            "material": dynamics.environment.side_wall_material,
            "boundary": {"axis": "y", "value_m": right, "inside": "+", "extent_status": extent_note},
        },
        {
            "name": "physical_left_sidewall_reachable_patch",
            "category": "side_wall",
            "center_m": [0.5 * (x_min + x_max), left + 0.5 * thickness, 0.5 * (floor_z + z_max)],
            "size_m": [x_max - x_min, thickness, z_max - floor_z],
            "rotation_matrix": identity,
            "dynamic": False,
            "material": dynamics.environment.side_wall_material,
            "boundary": {"axis": "y", "value_m": left, "inside": "-", "extent_status": extent_note},
        },
    ]


def _scene_primitives(
    scene: FrozenLayoutMotionInput,
    dynamics: M710EngineeringDynamicsConfig,
    execution: M710ExecutionConfig,
) -> list[dict[str, Any]]:
    snapshot_fixed = scene.snapshot["assembly"]["fixed_components"]
    fixed = []
    for item in snapshot_fixed:
        name = str(item["name"])
        material = "conveyor_belt" if name.startswith("conveyor_") else "painted_steel"
        fixed.append(_primitive(item, dynamic=False, material=material))
    cartons = [
        _primitive(
            item,
            dynamic=True,
            mass=dynamics.cartons.mass_kg_each,
            inertia=dynamics.cartons.inertia_tensor_com_kg_m2,
            material="carton",
        )
        for item in scene.snapshot["cartons"]
    ]
    primitives = [*fixed, *cartons, *_boundary_primitives(scene, dynamics, execution)]
    names = [item["name"] for item in primitives]
    if len(names) != len(set(names)) or len(cartons) != 40:
        raise ValueError("dynamic scene must contain unique primitives and exactly 40 cartons")
    return primitives


def _bridge_configuration(
    scene: FrozenLayoutMotionInput,
    dynamics: M710EngineeringDynamicsConfig,
    execution: M710ExecutionConfig,
) -> dict[str, Any]:
    model_source = (dynamics.config_path.parent / str(dynamics.data["sources"]["robot_model_config"])).resolve()
    model = _mapping(yaml.safe_load(model_source.read_text(encoding="utf-8")), "robot model")
    acceleration = model["joint_acceleration_limits_rad_s2"]["values"]
    jerk = model["joint_jerk_limits_rad_s3"]["values"]
    ordered_drives = [dynamics.joint_drives[f"J{index}"] for index in range(1, 7)]
    vacuum = dynamics.data["vacuum_attachment"]
    vacuum_model = dynamics.vacuum_attachment
    suction = scene.policy.data["suction"]
    suction_mode = str(
        getattr(vacuum_model, "suction_mode", vacuum.get("suction_mode", "per_sealed_cup"))
    )
    ideal_independent_cups = suction_mode == IDEAL_INDEPENDENT_CUPS_MODE
    tool_asset_document = _mapping(
        yaml.safe_load(execution.tool_manifest_path.read_text(encoding="utf-8")),
        "tool asset manifest",
    )
    tool_contact_geometry = _mapping(
        tool_asset_document.get("contact_geometry"), "tool asset contact_geometry"
    )
    vacuum_product_model = str(
        tool_asset_document.get("product_description", "Shanghai Wantai three-zone vacuum gripper")
    )
    vacuum_cup_model = str(tool_contact_geometry.get("cup_model", "FG42"))
    cup_rows = int(getattr(vacuum_model, "cup_rows", suction["cup_rows"]))
    cup_columns = int(getattr(vacuum_model, "cup_columns", suction["cup_columns"]))
    cup_pitch_m = list(getattr(vacuum_model, "cup_pitch_m", suction["cup_pitch_m"]))
    cup_radius_m = float(getattr(vacuum_model, "cup_radius_m", suction["cup_radius_m"]))
    if (
        suction_mode != str(suction.get("mode", suction_mode))
        or cup_rows != int(suction["cup_rows"])
        or cup_columns != int(suction["cup_columns"])
        or not np.allclose(cup_pitch_m, suction["cup_pitch_m"], atol=1e-12, rtol=0.0)
        or not np.isclose(cup_radius_m, suction["cup_radius_m"], atol=1e-12, rtol=0.0)
    ):
        raise ValueError("motion policy and dynamics independent-cup definitions disagree")
    tool_geometry = copy.deepcopy(dict(scene.snapshot["tool"]["geometry"]))
    materials = {
        name: {
            "static_friction": value.static_friction,
            "dynamic_friction": value.dynamic_friction,
            "restitution": value.restitution,
        }
        for name, value in dynamics.contacts.items()
    }
    damping = {
        name: {
            "linear_damping_s_inv": value.linear_damping_s_inv,
            "angular_damping_s_inv": value.angular_damping_s_inv,
        }
        for name, value in dynamics.damping.items()
    }
    conveyor_directions = {
        name: list(value.direction_world)
        for name, value in dynamics.conveyors.items()
        if value.enabled
    }
    speeds = {value.speed_m_s for value in dynamics.conveyors.values() if value.enabled}
    if len(speeds) != 1:
        raise ValueError("the current replay adapter requires one common conveyor speed")
    robot_link_dynamics = [
        {
            "link": name,
            "mass_kg": dynamics.robot_links[name].mass_kg,
            "com_xyz_m": list(dynamics.robot_links[name].com_xyz_m),
            "inertia_at_com_kg_m2": [list(row) for row in dynamics.robot_links[name].inertia_tensor_com_kg_m2],
            "source_status": dynamics.qualification_status,
        }
        for name in ROBOT_LINK_NAMES
    ]
    replay = execution.data["replay"]
    frame_contract = scene.policy.tool_frames.evidence()
    task_tcp_from_flange_m = float(
        scene.policy.tool_frames.flange_from_virtual_task_tcp[0, 3]
    )
    uncompressed_face_from_flange_m = float(
        scene.policy.tool_frames.flange_from_uncompressed_cup_plane[0, 3]
    )
    compressed_contact_plane_from_flange_m = float(
        scene.policy.tool_frames.flange_from_nominal_compressed_contact[0, 3]
    )
    if not np.isclose(
        uncompressed_face_from_flange_m - compressed_contact_plane_from_flange_m,
        dynamics.vacuum_attachment.cup_compression_m,
        atol=1e-12,
        rtol=0.0,
    ):
        raise ValueError("planner frame contract and dynamics cup compression disagree")
    return {
        "execution": {
            "limits_source": model_source.relative_to(execution.project_root).as_posix(),
            "joint_velocity_limits_rad_s": [item.velocity_limit_rad_s for item in ordered_drives],
            "joint_acceleration_limits_rad_s2": list(acceleration),
            "joint_jerk_limits_rad_s3": list(jerk),
            "joint_effort_limits_nm": [item.effort_limit_nm for item in ordered_drives],
            "joint_drive_stiffness_nm_rad": [item.stiffness_nm_rad for item in ordered_drives],
            "joint_drive_damping_nm_s_rad": [item.damping_nm_s_rad for item in ordered_drives],
            "motion_limit_scale": float(replay["motion_limit_scale"]),
            "minimum_segment_seconds": 0.002,
            "pre_grasp_controller_settle_seconds": float(replay["pre_grasp_controller_settle_s"]),
            "vacuum_establish_seconds": float(replay["vacuum_establish_s"]),
            "release_seconds": float(replay["release_hold_s"]),
            "post_release_settle_seconds": float(replay["post_release_settle_s"]),
            "isaac_replay_time_scale": float(replay["normal_time_scale"]),
        },
        "planning": {"trajectory_waypoint_period_seconds": float(replay["controller_period_s"])},
        "tool": {
            "mass_kg": dynamics.tool.mass_kg,
            "com_xyz_m": list(dynamics.tool.com_xyz_m),
            "inertia_tensor_com_kg_m2": [list(row) for row in dynamics.tool.inertia_tensor_com_kg_m2],
            "fixed_mass_accounting_policy": dynamics.tool.body_mode,
            "attach_to_link": dynamics.tool.attach_to_link,
            "independent_tool_rigid_body_required": False,
            "T_mass_accounting_link_tool": [
                [1.0, 0.0, 0.0, 0.175],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            "planner_tool_length_m": task_tcp_from_flange_m,
            "frame_contract": frame_contract,
            "geometry": tool_geometry,
        },
        "simulation_validation": {
            "vacuum_product_model": vacuum_product_model,
            "vacuum_cup_model": vacuum_cup_model,
            "vacuum_suction_mode": suction_mode,
            "vacuum_cup_count": vacuum_model.physical_cup_count,
            "vacuum_cup_rows": cup_rows,
            "vacuum_cup_columns": cup_columns,
            "vacuum_cup_pitch_m": cup_pitch_m,
            "vacuum_cup_radius_m": cup_radius_m,
            "vacuum_zone_count": (
                M710_CUP_ZONE_COUNT
                if ideal_independent_cups
                else int(vacuum["zone_count"])
            ),
            "vacuum_pull_off_force_per_cup_n": (
                None
                if ideal_independent_cups
                else float(vacuum["pull_off_force_per_cup_n"])
            ),
            "vacuum_shear_force_per_cup_n": (
                None
                if ideal_independent_cups
                else float(vacuum["shear_force_per_cup_n"])
            ),
            "vacuum_holding_torque_nm": (
                None if ideal_independent_cups else vacuum_model.break_torque_nm
            ),
            "vacuum_solver_total_break_force_n": (
                None if ideal_independent_cups else vacuum_model.break_force_n
            ),
            "vacuum_cup_compression_m": float(vacuum["cup_compression_m"]),
            "vacuum_max_grip_distance_m": vacuum_model.max_attachment_gap_m,
            "vacuum_max_normal_misalignment_rad": (
                vacuum_model.max_normal_misalignment_rad
            ),
            "vacuum_maximum_contact_penetration_m": float(
                getattr(
                    vacuum_model,
                    "max_contact_penetration_m",
                    scene.policy.data["state_validity"]["contact_tolerance_m"],
                )
            ),
            "vacuum_surface_gripper_capture_tolerance_m": (
                vacuum_model.max_attachment_gap_m
            ),
            "vacuum_footprint_size_m": [0.288, 0.576],
            "vacuum_effective_seal_area_m2": None,
            "vacuum_limits_calibrated": False,
            "vacuum_attachment_model": (
                IDEAL_INDEPENDENT_CUPS_MODE if ideal_independent_cups else "per_sealed_cup"
            ),
            "vacuum_solver_attachment_model": (
                "single_same_body_fixed_constraint_after_actual_contact"
                if ideal_independent_cups
                else "equivalent_zone_row_band_centers"
            ),
            "vacuum_solver_position_iterations": dynamics.simulation.solver_position_iterations,
            "vacuum_solver_velocity_iterations": dynamics.simulation.solver_velocity_iterations,
            "vacuum_task_tcp_from_flange_m": task_tcp_from_flange_m,
            "vacuum_physical_uncompressed_face_from_flange_m": (
                uncompressed_face_from_flange_m
            ),
            "vacuum_compressed_contact_plane_from_flange_m": (
                compressed_contact_plane_from_flange_m
            ),
            "vacuum_frame_contract": frame_contract,
            "robot_link_dynamics": robot_link_dynamics,
            "physics": {
                "parameter_status": dynamics.qualification_status,
                "gravity_world_m_s2": list(dynamics.simulation.gravity_world_m_s2),
                "physics_time_step_s": dynamics.simulation.physics_time_step_s,
                "materials": materials,
                "material_by_category": {
                    "carton": "carton",
                    "conveyor": "conveyor_belt",
                    "chassis": "painted_steel",
                    "floor": "painted_steel",
                    "side_wall": "painted_steel",
                },
                "damping": damping,
                "settling": {
                    "maximum_settle_time_s": dynamics.settling.maximum_settle_time_s,
                    "required_stable_duration_s": dynamics.settling.required_stable_duration_s,
                    "max_linear_speed_m_s": dynamics.settling.max_linear_speed_m_s,
                    "max_angular_speed_rad_s": dynamics.settling.max_angular_speed_rad_s,
                    "max_position_drift_m": dynamics.settling.max_position_drift_m,
                    "max_penetration_m": dynamics.settling.max_penetration_m,
                },
            },
            "rendering": {
                "required_output": {
                    "width_px": int(replay["output_width_px"]),
                    "height_px": int(replay["output_height_px"]),
                    "fps": int(replay["output_fps"]),
                    "camera_mode": str(replay["camera_mode"]),
                }
            },
            "conveyor": {
                "enabled": True,
                "speed_m_s": next(iter(speeds)),
                "surface_directions_world": conveyor_directions,
                "start_policy": "after_release_retreat",
                "exclusive_surface_drive_at_transfer": dynamics.exclusive_surface_drive_at_transfer,
            },
        },
    }


def _audited_execution_assets(
    execution: M710ExecutionConfig,
    scene: FrozenLayoutMotionInput,
    dynamics: M710EngineeringDynamicsConfig,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Audit and inline the exact official robot and supplied tool inputs.

    The official robot report deliberately has a narrower schema than the
    historical source/conversion audit.  This adapter retains the legacy keys
    consumed by the replay contract while also serializing every official
    integrity/semantic gate verbatim.  Machine qualification is intentionally
    absent from this asset-scoped qualification.
    """

    official_document = _mapping(
        yaml.safe_load(execution.robot_manifest_path.read_text(encoding="utf-8")),
        "official robot provenance manifest",
    )
    official_report = audit_m710id70_official_model(
        execution.project_root,
        execution.robot_manifest_path,
    )
    tool_report = load_and_audit_asset_manifest(
        execution.tool_manifest_path,
        repository_root=execution.project_root,
    )
    tool_document = _mapping(
        yaml.safe_load(execution.tool_manifest_path.read_text(encoding="utf-8")),
        "tool asset manifest",
    )
    source_records = tool_document.get("source_files")
    if not isinstance(source_records, list):
        raise ValueError("tool asset manifest source_files must be a list")
    analysis_records = [
        item
        for item in source_records
        if isinstance(item, Mapping)
        and item.get("role") == "derived_mass_and_contact_analysis"
    ]
    if len(analysis_records) != 1:
        raise ValueError("tool asset manifest must bind one mass/contact analysis")
    analysis_record = analysis_records[0]
    analysis_path = (
        execution.project_root / str(analysis_record["path"])
    ).resolve()
    analysis = _mapping(
        json.loads(analysis_path.read_text(encoding="utf-8")),
        "tool mass/contact analysis",
    )
    rigid_bounds = np.asarray(
        analysis.get("rigid_collision_bounding_boxes_step_mm"), dtype=float
    )
    tool_frames = _mapping(tool_document.get("frames"), "tool asset frames")
    frame_contract = scene.policy.tool_frames.evidence()
    vacuum_model = dynamics.vacuum_attachment
    geometry_qualification = {
        "schema": "m710id70_tool_execution_geometry_qualification_v1",
        "source_manifest_sha256": tool_report.manifest_sha256,
        "mass_contact_analysis_path": str(analysis_record["path"]),
        "mass_contact_analysis_sha256": sha256_file(analysis_path),
        "mass_contact_analysis_hash_matches_manifest": (
            sha256_file(analysis_path) == analysis_record.get("sha256")
        ),
        "rigid_solid_compound_count": int(rigid_bounds.shape[0]) if rigid_bounds.ndim == 2 else 0,
        "snapshot_rigid_collision_obb_count": len(
            scene.snapshot["tool"]["rigid_collision_obbs"]
        ),
        "snapshot_collision_representation": scene.snapshot["tool"][
            "execution_collision_representation"
        ],
        "step_frame_matches_analysis": (
            tool_frames.get("flange_origin_step_mm")
            == analysis.get("flange_origin_step_mm")
            and tool_frames.get("rotation_step_from_tool")
            == analysis.get("rotation_step_from_tool")
        ),
        "tool_frame_contract": copy.deepcopy(frame_contract),
        "suction_mode": str(getattr(vacuum_model, "suction_mode", "")),
        "physical_cup_count": int(vacuum_model.physical_cup_count),
        "cup_rows": int(getattr(vacuum_model, "cup_rows", 0)),
        "cup_columns": int(getattr(vacuum_model, "cup_columns", 0)),
    }
    structural_integration_checks_passed = bool(
        tool_report.source_integrity
        and geometry_qualification["mass_contact_analysis_hash_matches_manifest"]
        and rigid_bounds.shape == (58, 6)
        and np.all(np.isfinite(rigid_bounds))
        and int(analysis.get("solid_occurrence_count", -1)) == 202
        and len(analysis.get("deformable_cup_solid_indices", [])) == 144
        and geometry_qualification["snapshot_rigid_collision_obb_count"] == 58
        and geometry_qualification["snapshot_collision_representation"]
        == "58_CAD_DERIVED_RIGID_SOLID_COMPOUND_OBBS_PLUS_SEPARATE_FLEXIBLE_CUP_CONTACTS"
        and geometry_qualification["step_frame_matches_analysis"]
        and frame_contract.get("execution_qualified") is True
        and frame_contract.get("tool0_clocking_status")
        == "OFFICIAL_FLANGE_TO_PROJECT_TOOL0_ADAPTER_RESOLVED"
        and geometry_qualification["suction_mode"] == IDEAL_INDEPENDENT_CUPS_MODE
        and geometry_qualification["physical_cup_count"] == 72
        and geometry_qualification["cup_rows"] == 6
        and geometry_qualification["cup_columns"] == 12
    )
    # These checks bind the representation to the supplied source and selected
    # frame, but they do not prove that the heuristic 58/144 solid
    # classification is physically correct or that every serialized box is an
    # outward enclosure of its source rigid solid.  Until a per-solid semantic
    # and containment certificate is archived, this representation may reject
    # states conservatively but may not accept an execution path.
    geometry_qualification.update(
        {
            "structural_integration_checks_passed": structural_integration_checks_passed,
            "rigid_solid_semantic_classification_verified": False,
            "outward_containment_certificate_present": False,
            "no_false_negative_rigid_solid_coverage_proven": False,
            "robot_tool_mount_contact_scope_verified": False,
            "broad_j6_tool_collision_exception_present": True,
            "planning_collision_acceptance_qualified": False,
            "dynamic_collision_qualified": False,
            "execution_qualified": False,
        }
    )
    robot = {
        **official_report.to_mapping(),
        "schema_version": str(official_document["schema_version"]),
        "conversion_status": "PINNED_OFFICIAL_STATIC_URDF_AND_PER_LINK_MESHES_AUDITED",
        "verified_file_count": official_report.verified_source_file_count,
        "missing_converted_outputs": [],
        "unresolved": [],
    }
    source_tool = tool_report.to_mapping()
    source_unresolved = list(source_tool["unresolved"])
    resolved_integration_issues = {
        "FLANGE_CLOCKING_PROVISIONAL": (
            "selected official flange-to-project-tool0 engineering adapter; source metrology status unchanged"
        ),
    }
    integration_blocking_issues = {
        TOOL_RIGID_COLLISION_COVERAGE_REASON: (
            "the 58/144 solid classification and outward OBB containment are not certified"
        ),
        ROBOT_TOOL_MOUNT_CONTACT_SCOPE_REASON: (
            "current exact validator ignores J6_link against every tool rigid box; "
            "a dimensioned mounting-interface-only contact rule is not available"
        )
    }
    tool = {
        **source_tool,
        "source_execution_qualified": source_tool["execution_qualified"],
        "source_unresolved": source_unresolved,
        "resolved_by_execution_integration": resolved_integration_issues,
        "integration_blocking_issues": integration_blocking_issues,
        "unresolved": sorted(
            [issue for issue in source_unresolved if issue not in resolved_integration_issues]
            + list(integration_blocking_issues)
        ),
        "execution_geometry_qualification": geometry_qualification,
        "execution_qualified": False,
    }
    assets = {
        "status": "OFFICIAL_ROBOT_PASS_TOOL_SOURCE_PASS_COLLISION_ACCEPTANCE_BLOCKED",
        "execution_qualified": bool(
            official_report.execution_qualified and tool["execution_qualified"]
        ),
        "robot": robot,
        "tool": tool,
    }
    return assets, copy.deepcopy(dict(official_document)), official_report.to_mapping()


def _motion_trajectory_segment(
    motion: Mapping[str, Any],
) -> tuple[dict[str, Any] | None, str | None]:
    """Extract one evidence-bound complete segment or return a fail-closed blocker."""
    if motion.get("complete_trajectory_status") != "PASS":
        return None, None
    segment = motion.get("selected_trajectory_segment")
    if not isinstance(segment, Mapping):
        return None, "VERIFIED_COMPLETE_TRAJECTORY_SEGMENT_NOT_AVAILABLE"
    try:
        return validate_trajectory_segment(segment), None
    except M710ReplayContractError:
        return None, "VERIFIED_COMPLETE_TRAJECTORY_SEGMENT_INVALID"


def _initialization_blocked_preflight(execution, policy, dynamics, motion, backend_status):
    """Audit model inputs without inventing a verified scene or replay bundle."""
    if backend_status not in {"NOT_RUN", "NOT_RUN_PER_USER_REQUEST"}:
        raise ValueError("invalid initial-state diagnostic cannot claim physical execution")
    initial = audit_initial_state(policy.layout_validation)
    if initial["status"] != "FAIL" or initial != motion.get("initial_state_audit"):
        raise ValueError("blocked motion initial-state audit does not match current layout")
    if (
        motion.get("scene_fingerprint") is not None
        or motion.get("selected_trajectory_segment") is not None
        or motion.get("complete_trajectory_status") != "FAIL_CLOSED"
        or sorted(motion.get("task_population", {}).get("carton_ids", [])) != list(EXPECTED_TOP_CARTONS)
        or motion.get("statistics", {}).get("task_count") != 5
        or motion.get("statistics", {}).get("ik_calls") != 0
        or motion.get("statistics", {}).get("path_connection_attempts") != 0
    ):
        raise ValueError("invalid initial-state motion must contain no snapshot, trajectory, or search")
    robot = audit_m710id70_official_model(
        execution.project_root, execution.robot_manifest_path
    ).to_mapping()
    tool = load_and_audit_asset_manifest(
        execution.tool_manifest_path, repository_root=execution.project_root
    ).to_mapping()
    blockers = ["INITIAL_STATE_INVALID"]
    tool_geometry_status = str(
        policy.layout_validation.layout.data["tool"]["geometry_status"]
    )
    if tool_geometry_status != TOOL_RIGID_COLLISION_COVERAGE_PROVEN_STATUS:
        blockers.append(TOOL_RIGID_COLLISION_COVERAGE_REASON)
    if not ROBOT_TOOL_MOUNT_CONTACT_SCOPE_QUALIFIED:
        blockers.append(ROBOT_TOOL_MOUNT_CONTACT_SCOPE_REASON)
    if robot.get("execution_qualified") is not True:
        blockers.append("OFFICIAL_MODEL_ASSET_AUDIT_FAILED")
    if tool.get("source_integrity") is not True:
        blockers.append("TOOL_SOURCE_INTEGRITY_FAILED")
    blockers = sorted(set(blockers))
    model_ready = robot.get("execution_qualified") is True and tool.get("source_integrity") is True
    warnings = sorted(set(tool.get("unresolved", [])) & MACHINE_QUALIFICATION_ASSET_WARNINGS)
    identity = {
        "execution_config_fingerprint": execution.fingerprint,
        "layout_fingerprint": policy.layout_validation.layout.layout_fingerprint,
        "scene_fingerprint": None,
        "motion_evidence_fingerprint": motion["evidence_fingerprint"],
        "execution_implementation_source_sha256": _execution_implementation_identity(execution.project_root),
        "dynamics_fingerprint": dynamics.fingerprint,
        "robot_manifest_sha256": robot["manifest_sha256"],
        "tool_manifest_sha256": tool["manifest_sha256"],
    }
    result = {
        "schema": INITIALIZATION_DIAGNOSTIC_SCHEMA,
        "status": "BLOCKED",
        "simulation_execution_ready": False,
        "simulation_execution_qualified": False,
        "execution_qualified": False,
        "simulation_readiness_blockers": blockers,
        "blockers": blockers,
        "machine_qualified": dynamics.machine_qualified,
        "machine_qualification_warnings": warnings,
        "backend_execution_status": backend_status,
        "isaac_validation_performed": False,
        "input_identity": identity,
        "asset_audit": {"robot": robot, "tool": tool},
        "model_initialization": {
            "model_load_ready": model_ready,
            "scope": "official_robot_and_supplied_tool_inputs_only_not_validated_scene_or_execution",
            "official_robot_inertials_and_finite_drive_limits_validated": True,
            "tool_execution_integration": "NOT_RUN_NO_VALID_SCENE_SNAPSHOT",
            "deferred_tool_collision_qualification": {
                "layout_geometry_status": tool_geometry_status,
                "required_coverage_status": TOOL_RIGID_COLLISION_COVERAGE_PROVEN_STATUS,
                "robot_tool_mount_contact_scope_qualified": (
                    ROBOT_TOOL_MOUNT_CONTACT_SCOPE_QUALIFIED
                ),
            },
        },
        "dynamics": {
            "qualification_status": dynamics.qualification_status,
            "machine_qualified": dynamics.machine_qualified,
            "robot_mass_kg": dynamics.robot_mass_kg,
            "tool_mass_kg": dynamics.tool.mass_kg,
            "carton_count": dynamics.cartons.count,
            "carton_mass_kg_each": dynamics.cartons.mass_kg_each,
            "total_configured_mass_kg": dynamics.total_configured_mass_kg,
            "fingerprint": dynamics.fingerprint,
        },
        "motion": copy.deepcopy(dict(motion)),
        "initial_state_audit": initial,
        "scene": {
            "layout_id": policy.layout_validation.layout.data["layout_id"],
            "layout_fingerprint": policy.layout_validation.layout.layout_fingerprint,
            "scene_fingerprint": None,
            "snapshot_status": "NOT_CREATED_INVALID_INITIAL_STATE",
            "carton_count": 40,
            "carton_ids": [box.name for box in policy.layout_validation.layout.cartons()],
            "configured_dynamic_carton_count": dynamics.cartons.count,
            "dynamic_carton_count": 0,
            "primitives": [],
        },
        "replay_adapter_inputs": None,
        "claims": {
            "engineering_dynamics_input_validated": True,
            "complete_single_carton_motion": False,
            "physical_simulation_execution": backend_status,
            "video": "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION",
        },
    }
    result["preflight_fingerprint"] = canonical_digest(result)
    return result


def build_m710_execution_preflight(
    config: str | Path | M710ExecutionConfig | None = None,
    *,
    motion_result: Mapping[str, Any] | None = None,
    backend_execution_status: str = "NOT_RUN",
    trajectory_connector: LayoutTrajectoryConnector | None = None,
) -> dict[str, Any]:
    execution = config if isinstance(config, M710ExecutionConfig) else load_m710_execution_config(config)
    policy = load_layout_motion_policy(execution.motion_policy_path)
    dynamics = load_m710id70_dynamics(execution.dynamics_path)
    motion = (
        dict(motion_result)
        if motion_result is not None
        else run_layout_single_carton_audit(
            policy,
            project_root=execution.project_root,
            trajectory_connector=trajectory_connector,
        )
    )
    if motion.get("schema") != MOTION_RESULT_SCHEMA:
        raise ValueError("motion result has an unsupported schema")
    motion_identity = copy.deepcopy(motion)
    recorded_motion_fingerprint = motion_identity.pop("evidence_fingerprint", None)
    if (
        not isinstance(recorded_motion_fingerprint, str)
        or len(recorded_motion_fingerprint) != 64
        or canonical_digest(motion_identity) != recorded_motion_fingerprint
    ):
        raise ValueError("motion result evidence fingerprint mismatch")
    if motion.get("policy_fingerprint") != policy.policy_fingerprint:
        raise ValueError("motion result belongs to a different motion policy")
    if motion.get("implementation_identity") != motion_implementation_identity(execution.project_root):
        raise ValueError("motion result belongs to a different implementation or runtime")
    if motion.get("layout_fingerprint") != policy.layout_validation.layout.layout_fingerprint:
        raise ValueError("motion result belongs to a different layout")
    if (
        motion.get("run_status") == "BLOCKED"
        and motion.get("complete_trajectory_failure_reason") == "INITIAL_STATE_INVALID"
    ):
        return _initialization_blocked_preflight(execution, policy, dynamics, motion, backend_execution_status)
    scene = build_verified_motion_input(policy, execution.project_root)
    assets, official_manifest, official_model_audit = _audited_execution_assets(execution, scene, dynamics)
    if motion.get("scene_fingerprint") != scene.snapshot["scene_fingerprint"]:
        raise ValueError("motion result belongs to a different frozen scene")
    if motion.get("statistics", {}).get("task_count") != 5:
        raise ValueError("motion result must preserve the five-carton task population")
    if motion.get("task_population", {}).get("carton_ids") != list(scene.removable_cartons):
        raise ValueError("motion result task population does not match the frozen support graph")
    if backend_execution_status not in {"NOT_RUN", "NOT_RUN_PER_USER_REQUEST", "PASS", "FAIL"}:
        raise ValueError("unsupported backend execution status")

    primitives = _scene_primitives(scene, dynamics, execution)
    asset_issues = set(assets["robot"]["unresolved"]) | set(assets["tool"]["unresolved"])
    known_asset_issues = SIMULATION_BLOCKING_ASSET_ISSUES | MACHINE_QUALIFICATION_ASSET_WARNINGS
    # New, unclassified asset uncertainty must fail closed until explicitly
    # assigned to simulation readiness or machine qualification.
    blockers = list((asset_issues & SIMULATION_BLOCKING_ASSET_ISSUES) | (asset_issues - known_asset_issues))
    machine_warnings = sorted(asset_issues & MACHINE_QUALIFICATION_ASSET_WARNINGS)
    trajectory_candidate, trajectory_blocker = _motion_trajectory_segment(motion)
    if motion.get("complete_trajectory_status") != "PASS":
        blockers.append(str(motion.get("complete_trajectory_failure_reason", "NO_COMPLETE_MOTION_PLAN")))
        if int(motion.get("statistics", {}).get("ik_valid_solutions", 0)) == 0:
            blockers.append("NO_STRICT_GRASP_IK")
    elif trajectory_blocker is not None:
        blockers.append(trajectory_blocker)
    if scene.snapshot["robot"]["mounting_reference"]["status"] != "EXECUTION_QUALIFIED_CAD_COLLISION":
        blockers.append("REAL_BASE_MOUNTING_EXTENT_NOT_AVAILABLE")
    simulation_execution_ready = not blockers
    simulation_execution_qualified = bool(simulation_execution_ready)
    # Compatibility field: keep it scoped to simulation execution.  Machine or
    # controller certification is recorded independently and must not veto use
    # of the pinned official model as an engineering simulation input.
    execution_qualified = simulation_execution_qualified
    status = (
        "BLOCKED"
        if not simulation_execution_ready
        else "READY"
        if backend_execution_status in {"NOT_RUN", "NOT_RUN_PER_USER_REQUEST"}
        else f"EXECUTION_{backend_execution_status}"
    )
    identity = {
        "execution_config_fingerprint": execution.fingerprint,
        "layout_fingerprint": scene.snapshot["layout_fingerprint"],
        "scene_fingerprint": scene.snapshot["scene_fingerprint"],
        "motion_evidence_fingerprint": motion["evidence_fingerprint"],
        "execution_implementation_source_sha256": _execution_implementation_identity(
            execution.project_root
        ),
        "dynamics_fingerprint": dynamics.fingerprint,
        "robot_manifest_sha256": assets["robot"]["manifest_sha256"],
        "tool_manifest_sha256": assets["tool"]["manifest_sha256"],
        "robot_missing_mesh_outputs": assets["robot"]["missing_converted_outputs"],
    }
    robot_model_config_path = (
        execution.project_root
        / str(scene.snapshot["robot"]["model_config"]["repository_path"])
    ).resolve()
    robot_model_document = _mapping(
        yaml.safe_load(robot_model_config_path.read_text(encoding="utf-8")),
        "robot model configuration",
    )
    robot_kinematics = _mapping(robot_model_document.get("kinematics"), "robot kinematics")
    srdf_declared = Path(str(robot_kinematics["srdf_path"]))
    srdf_path = (
        srdf_declared.resolve()
        if srdf_declared.is_absolute()
        else (robot_model_config_path.parent / srdf_declared).resolve()
    )
    if not srdf_path.is_file():
        raise FileNotFoundError(f"official-model SRDF is missing: {srdf_path}")
    try:
        srdf_repository_path = srdf_path.relative_to(execution.project_root).as_posix()
    except ValueError as exc:
        raise ValueError("official-model SRDF must remain inside the repository") from exc
    identity["robot_srdf_sha256"] = sha256_file(srdf_path)
    execution_asset_fingerprint = canonical_digest(identity)
    bridge_cfg = _bridge_configuration(scene, dynamics, execution)
    normalized_blockers = sorted(set(blockers))
    trajectory_segment = trajectory_candidate if simulation_execution_ready else None
    trajectory_segment_status = "VERIFIED" if trajectory_segment is not None else "NOT_AVAILABLE"
    plan_common = {
        "robot": {
            "model": "fanuc_m710id_70",
            "urdf_path": official_manifest["integration"]["expanded_urdf"]["path"],
            "srdf_path": srdf_repository_path,
            "srdf_sha256": identity["robot_srdf_sha256"],
            "base_position": np.asarray(scene.snapshot["robot"]["world_from_mount"])[
                :3, 3
            ].tolist(),
            "base_rpy": [0.0, 0.0, 0.0],
            "isaac_grasp_body_path_suffix": (
                "Geometry/base_link/J1_link/J2_link/J3_link/J4_link/J5_link/J6_link"
            ),
            "flange_offset_from_grasp_body_m": [0.175, 0.0, 0.0],
            "official_model_manifest": copy.deepcopy(official_manifest),
        },
        "official_model_manifest": copy.deepcopy(official_manifest),
        "official_model_manifest_sha256": assets["robot"]["manifest_sha256"],
        "official_model_audit": copy.deepcopy(official_model_audit),
        "official_model_required": True,
        "scene_primitives": primitives,
        "layout_fingerprint": scene.snapshot["layout_fingerprint"],
        "scene_fingerprint": scene.snapshot["scene_fingerprint"],
        "motion_evidence_fingerprint": motion["evidence_fingerprint"],
        "execution_asset_fingerprint_sha256": execution_asset_fingerprint,
        "simulation_execution_ready": simulation_execution_ready,
        "simulation_execution_qualified": simulation_execution_qualified,
        "execution_qualified": execution_qualified,
        "execution_blockers": normalized_blockers,
        "machine_qualified": dynamics.machine_qualified,
        "machine_qualification_warnings": machine_warnings,
        "collision_geometry": "real_cad_per_link_visual_and_compound_collision_required",
    }
    input_binding = build_replay_input_binding(
        plan_common=plan_common,
        configuration=bridge_cfg,
        scene_primitives=primitives,
        trajectory_segment=trajectory_segment,
        trajectory_segment_status=trajectory_segment_status,
        input_identity=identity,
        execution_asset_fingerprint_sha256=execution_asset_fingerprint,
    )
    result: dict[str, Any] = {
        "schema": PREFLIGHT_SCHEMA,
        "status": status,
        "simulation_execution_ready": simulation_execution_ready,
        "simulation_execution_qualified": simulation_execution_qualified,
        "execution_qualified": execution_qualified,
        "simulation_readiness_blockers": normalized_blockers,
        "blockers": normalized_blockers,  # compatibility alias consumed by the replay adapter
        "machine_qualified": dynamics.machine_qualified,
        "machine_qualification_warnings": machine_warnings,
        "backend_execution_status": backend_execution_status,
        "isaac_validation_performed": backend_execution_status in {"PASS", "FAIL"},
        "input_identity": identity,
        "execution_asset_fingerprint_sha256": execution_asset_fingerprint,
        "official_model_manifest": copy.deepcopy(official_manifest),
        "official_model_manifest_sha256": assets["robot"]["manifest_sha256"],
        "official_model_audit": copy.deepcopy(official_model_audit),
        "asset_audit": assets,
        "dynamics": {
            "qualification_status": dynamics.qualification_status,
            "machine_qualified": dynamics.machine_qualified,
            "robot_mass_kg": dynamics.robot_mass_kg,
            "tool_mass_kg": dynamics.tool.mass_kg,
            "carton_count": dynamics.cartons.count,
            "carton_mass_kg_each": dynamics.cartons.mass_kg_each,
            "total_configured_mass_kg": dynamics.total_configured_mass_kg,
            "fingerprint": dynamics.fingerprint,
        },
        "motion": {
            "run_status": motion["run_status"],
            "complete_trajectory_status": motion["complete_trajectory_status"],
            "complete_trajectory_failure_reason": motion["complete_trajectory_failure_reason"],
            "statistics": copy.deepcopy(motion["statistics"]),
            "task_population": copy.deepcopy(motion["task_population"]["carton_ids"]),
            "evidence_fingerprint": motion["evidence_fingerprint"],
        },
        "scene": {
            "layout_id": scene.snapshot["layout_id"],
            "layout_fingerprint": scene.snapshot["layout_fingerprint"],
            "scene_fingerprint": scene.snapshot["scene_fingerprint"],
            "trailer_length_m": None,
            "trailer_height_m": None,
            "trailer_extent_claim": "NOT_DEFINED",
            "carton_count": 40,
            "dynamic_carton_count": sum(
                item["category"] == "carton" and item["dynamic"] for item in primitives
            ),
            "primitives": primitives,
            "receiver": "conveyor_transverse",
        },
        "replay_adapter_inputs": {
            "plan_common": plan_common,
            "configuration": bridge_cfg,
            "trajectory_segment": trajectory_segment,
            "trajectory_segment_status": trajectory_segment_status,
            "input_binding": input_binding,
        },
        "claims": {
            "real_robot_cad_source_integrity": assets["robot"]["source_integrity"],
            "real_robot_cad_per_link_conversion_complete": not bool(
                assets["robot"]["missing_converted_outputs"]
            ),
            "real_tool_source_integrity": assets["tool"]["source_integrity"],
            # Compatibility boolean retained for downstream readers.  It must
            # remain false until both conservative rigid-solid coverage and
            # the backend dynamic checks have their own evidence.
            "real_tool_dynamic_collision_qualified": False,
            "tool_planning_collision_representation_status": (
                "BLOCKED_NO_FALSE_NEGATIVE_RIGID_SOLID_COVERAGE_CERTIFICATE"
            ),
            "tool_collision_qualification_scope": (
                "SOURCE_INTEGRITY_AND_STRUCTURAL_INTEGRATION_ONLY"
            ),
            "engineering_dynamics_input_validated": True,
            "manufacturer_link_inertials_or_drive_limits_confirmed": True,
            "complete_single_carton_motion": motion["complete_trajectory_status"] == "PASS",
            "physical_simulation_execution": backend_execution_status,
            "video": "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION",
        },
    }
    result["preflight_fingerprint"] = canonical_digest(result)
    return result


def write_m710_execution_preflight(result: Mapping[str, Any], path: str | Path) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return destination


def verify_m710_execution_preflight(result: Mapping[str, Any]) -> dict[str, Any]:
    """Verify content plus internal readiness and replay-input consistency."""
    if result.get("schema") == INITIALIZATION_DIAGNOSTIC_SCHEMA:
        payload = copy.deepcopy(dict(result))
        recorded = payload.pop("preflight_fingerprint", None)
        if not isinstance(recorded, str) or canonical_digest(payload) != recorded:
            raise M710ReplayContractError("preflight fingerprint mismatch")
        if (
            result.get("status") != "BLOCKED"
            or any(result.get(key) is not False for key in (
                "simulation_execution_ready", "simulation_execution_qualified",
                "execution_qualified", "isaac_validation_performed",
            ))
            or result.get("replay_adapter_inputs") is not None
            or result.get("backend_execution_status") not in {"NOT_RUN", "NOT_RUN_PER_USER_REQUEST"}
            or result.get("blockers") != result.get("simulation_readiness_blockers")
            or "INITIAL_STATE_INVALID" not in result.get("blockers", [])
            or result.get("initial_state_audit", {}).get("status") != "FAIL"
            or not result.get("initial_state_audit", {}).get("failures")
            or result.get("scene", {}).get("scene_fingerprint") is not None
            or result.get("scene", {}).get("primitives") != []
        ):
            raise M710ReplayContractError("initialization diagnostic must remain fail-closed without replay inputs")
        motion = dict(result.get("motion", {}))
        motion_digest = motion.pop("evidence_fingerprint", None)
        if canonical_digest(motion) != motion_digest or motion_digest != result["input_identity"]["motion_evidence_fingerprint"]:
            raise M710ReplayContractError("initialization diagnostic motion identity mismatch")
        if motion.get("initial_state_audit") != result["initial_state_audit"]:
            raise M710ReplayContractError("initialization diagnostic initial failure mismatch")
        if (
            motion.get("run_status") != "BLOCKED"
            or motion.get("complete_trajectory_failure_reason") != "INITIAL_STATE_INVALID"
            or motion.get("complete_trajectory_status") != "FAIL_CLOSED"
            or motion.get("scene_fingerprint") is not None
            or motion.get("selected_trajectory_segment") is not None
            or result["input_identity"].get("scene_fingerprint") is not None
            or result["dynamics"]["fingerprint"] != result["input_identity"]["dynamics_fingerprint"]
        ):
            raise M710ReplayContractError("initialization diagnostic motion or dynamics contract mismatch")
        if any(result["asset_audit"][name]["manifest_sha256"] != result["input_identity"][f"{name}_manifest_sha256"] for name in ("robot", "tool")):
            raise M710ReplayContractError("initialization diagnostic asset identity mismatch")
        expected_model_ready = (
            result["asset_audit"]["robot"].get("execution_qualified") is True
            and result["asset_audit"]["tool"].get("source_integrity") is True
        )
        if result["model_initialization"].get("model_load_ready") is not expected_model_ready:
            raise M710ReplayContractError("initialization diagnostic model readiness mismatch")
        return {"status": "PASS", "preflight_fingerprint": recorded, "execution_ready": False}
    return verify_m710_preflight_contract(result)


__all__ = [
    "DEFAULT_CONFIG_PATH",
    "EXECUTION_CONFIG_SCHEMA",
    "PREFLIGHT_SCHEMA",
    "INITIALIZATION_DIAGNOSTIC_SCHEMA",
    "M710ExecutionConfig",
    "build_m710_execution_preflight",
    "load_m710_execution_config",
    "verify_m710_execution_preflight",
    "write_m710_execution_preflight",
]
