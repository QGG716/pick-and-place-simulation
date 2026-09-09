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

from .asset_audit import audit_m710id70_asset_set
from .layout_single_carton import (
    RESULT_SCHEMA as MOTION_RESULT_SCHEMA,
    FrozenLayoutMotionInput,
    build_verified_motion_input,
    load_layout_motion_policy,
    motion_implementation_identity,
    run_layout_single_carton_audit,
)
from .m710_dynamics import (
    ROBOT_LINK_NAMES,
    M710EngineeringDynamicsConfig,
    load_m710id70_engineering_dynamics,
)
from .m710_replay_contract import (
    M710ReplayContractError,
    build_replay_input_binding,
    validate_trajectory_segment,
    verify_m710_preflight_contract,
)
from .workcell_layout import canonical_digest, sha256_file


EXECUTION_CONFIG_SCHEMA = "m710id70_dynamic_execution_v1"
PREFLIGHT_SCHEMA = "m710id70_dynamic_execution_preflight_v1"
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "configs"
    / "simulation"
    / "m710id70_dynamic_execution_v1.yaml"
)
EXECUTION_IMPLEMENTATION_FILES = (
    "src/unloading_sim/asset_audit.py",
    "src/unloading_sim/isaac_bridge.py",
    "src/unloading_sim/m710_dynamics.py",
    "src/unloading_sim/m710_execution.py",
    "src/unloading_sim/m710_replay_physics.py",
    "src/unloading_sim/m710_replay_contract.py",
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
    suction = scene.policy.data["suction"]
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
            "planner_tool_length_m": task_tcp_from_flange_m,
            "frame_contract": frame_contract,
            "geometry": tool_geometry,
        },
        "simulation_validation": {
            "vacuum_product_model": str(vacuum["product_model"]),
            "vacuum_cup_model": str(vacuum["cup_model"]),
            "vacuum_cup_count": dynamics.vacuum_attachment.physical_cup_count,
            "vacuum_cup_rows": int(suction["cup_rows"]),
            "vacuum_cup_columns": int(suction["cup_columns"]),
            "vacuum_cup_pitch_m": list(suction["cup_pitch_m"]),
            "vacuum_cup_radius_m": float(suction["cup_radius_m"]),
            "vacuum_zone_count": int(vacuum["zone_count"]),
            "vacuum_pull_off_force_per_cup_n": float(vacuum["pull_off_force_per_cup_n"]),
            "vacuum_shear_force_per_cup_n": float(vacuum["shear_force_per_cup_n"]),
            "vacuum_holding_torque_nm": dynamics.vacuum_attachment.break_torque_nm,
            "vacuum_solver_total_break_force_n": dynamics.vacuum_attachment.break_force_n,
            "vacuum_cup_compression_m": float(vacuum["cup_compression_m"]),
            "vacuum_max_grip_distance_m": dynamics.vacuum_attachment.max_attachment_gap_m,
            "vacuum_max_normal_misalignment_rad": (
                dynamics.vacuum_attachment.max_normal_misalignment_rad
            ),
            "vacuum_maximum_contact_penetration_m": float(
                scene.policy.data["state_validity"]["contact_tolerance_m"]
            ),
            "vacuum_surface_gripper_capture_tolerance_m": (
                dynamics.vacuum_attachment.max_attachment_gap_m
            ),
            "vacuum_footprint_size_m": [0.288, 0.576],
            "vacuum_effective_seal_area_m2": None,
            "vacuum_limits_calibrated": False,
            "vacuum_attachment_model": "per_sealed_cup",
            "vacuum_solver_attachment_model": "equivalent_zone_row_band_centers",
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


def build_m710_execution_preflight(
    config: str | Path | M710ExecutionConfig | None = None,
    *,
    motion_result: Mapping[str, Any] | None = None,
    backend_execution_status: str = "NOT_RUN",
) -> dict[str, Any]:
    execution = config if isinstance(config, M710ExecutionConfig) else load_m710_execution_config(config)
    policy = load_layout_motion_policy(execution.motion_policy_path)
    scene = build_verified_motion_input(policy, execution.project_root)
    dynamics = load_m710id70_engineering_dynamics(execution.dynamics_path)
    assets = audit_m710id70_asset_set(execution.project_root)
    motion = dict(motion_result) if motion_result is not None else run_layout_single_carton_audit(
        policy, project_root=execution.project_root
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
    if motion.get("scene_fingerprint") != scene.snapshot["scene_fingerprint"]:
        raise ValueError("motion result belongs to a different frozen scene")
    if motion.get("policy_fingerprint") != policy.policy_fingerprint:
        raise ValueError("motion result belongs to a different motion policy")
    if motion.get("implementation_identity") != motion_implementation_identity(execution.project_root):
        raise ValueError("motion result belongs to a different implementation or runtime")
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
    overall_execution_qualified = bool(
        simulation_execution_ready and dynamics.machine_qualified
    )
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
    execution_asset_fingerprint = canonical_digest(identity)
    bridge_cfg = _bridge_configuration(scene, dynamics, execution)
    normalized_blockers = sorted(set(blockers))
    trajectory_segment = trajectory_candidate if simulation_execution_ready else None
    trajectory_segment_status = "VERIFIED" if trajectory_segment is not None else "NOT_AVAILABLE"
    plan_common = {
        "robot": {
            "model": "fanuc_m710id_70",
            "urdf_path": scene.snapshot["robot"]["urdf"]["repository_path"],
            "base_position": np.asarray(scene.snapshot["robot"]["world_from_mount"])[
                :3, 3
            ].tolist(),
            "base_rpy": [0.0, 0.0, 0.0],
            "isaac_grasp_body_path_suffix": (
                "Geometry/base_link/J1_link/J2_link/J3_link/J4_link/J5_link/J6_link"
            ),
            "flange_offset_from_grasp_body_m": [0.175, 0.0, 0.0],
        },
        "scene_primitives": primitives,
        "layout_fingerprint": scene.snapshot["layout_fingerprint"],
        "scene_fingerprint": scene.snapshot["scene_fingerprint"],
        "motion_evidence_fingerprint": motion["evidence_fingerprint"],
        "execution_asset_fingerprint_sha256": execution_asset_fingerprint,
        "simulation_execution_ready": simulation_execution_ready,
        "execution_qualified": overall_execution_qualified,
        "execution_blockers": normalized_blockers,
        "machine_qualified": False,
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
        "execution_qualified": overall_execution_qualified,
        "simulation_readiness_blockers": normalized_blockers,
        "blockers": normalized_blockers,  # compatibility alias consumed by the replay adapter
        "machine_qualified": False,
        "machine_qualification_warnings": machine_warnings,
        "backend_execution_status": backend_execution_status,
        "isaac_validation_performed": backend_execution_status in {"PASS", "FAIL"},
        "input_identity": identity,
        "execution_asset_fingerprint_sha256": execution_asset_fingerprint,
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
            "real_tool_dynamic_collision_qualified": assets["tool"]["execution_qualified"],
            "engineering_dynamics_input_validated": True,
            "manufacturer_link_inertials_or_drive_limits_confirmed": False,
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
    return verify_m710_preflight_contract(result)


__all__ = [
    "DEFAULT_CONFIG_PATH",
    "EXECUTION_CONFIG_SCHEMA",
    "PREFLIGHT_SCHEMA",
    "M710ExecutionConfig",
    "build_m710_execution_preflight",
    "load_m710_execution_config",
    "verify_m710_execution_preflight",
    "write_m710_execution_preflight",
]
