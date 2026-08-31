"""Backend-neutral command bundles for Isaac Sim trajectory validation.

This module deliberately imports no Isaac Sim, USD, CUDA, or ROS packages.
It converts a collision-checked geometric plan into a deterministic controller
command schedule that a heavyweight simulator adapter can consume.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from .geometry import OBB, rotation_matrix_from_rpy
from .scene import build_trailer_walls
from .timing import (
    motion_limits_from_config,
    scale_timed_trajectory_window,
    time_parameterize_joint_path,
)


FANUC_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")


def _canonical_digest(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _serialize_obb(obb: OBB, *, dynamic: bool = False, mass_kg: float | None = None) -> dict[str, Any]:
    primitive = {
        "name": obb.name,
        "category": obb.category,
        "center_m": obb.center.tolist(),
        "size_m": (2.0 * obb.half_extents).tolist(),
        "rotation_matrix": obb.rotation.tolist(),
        "dynamic": bool(dynamic),
    }
    if mass_kg is not None:
        primitive["mass_kg"] = float(mass_kg)
    return primitive


def _build_scene_primitives(
    plan: dict[str, Any], cfg: dict[str, Any], segment_index: int
) -> list[dict[str, Any]]:
    """Serialize the selected docked scene without importing a physics backend."""
    scene_cfg = cfg.get("scene")
    if not isinstance(scene_cfg, dict):
        return []

    primitives: list[dict[str, Any]] = []
    trailer = scene_cfg.get("trailer", {})
    required_trailer_keys = {"length", "width", "height"}
    if required_trailer_keys.issubset(trailer):
        walls = build_trailer_walls(
            length=float(trailer["length"]),
            width=float(trailer["width"]),
            height=float(trailer["height"]),
            wall_thickness=float(trailer.get("wall_thickness", 0.05)),
            floor_thickness=float(trailer.get("floor_thickness", 0.08)),
        )
        primitives.extend(_serialize_obb(wall) for wall in walls)

    segment = plan["segments"][segment_index]
    dock_value = segment.get("amr_dock_position")
    dock = np.asarray(dock_value, dtype=float) if dock_value is not None else None
    amr_cfg = cfg.get("amr", {})
    mounted_centers = dict(amr_cfg.get("mounted_surface_centers", {}))
    conveyor_name = str(amr_cfg.get("conveyor_name", "conveyor_deck"))
    if "conveyor_mount_center" in amr_cfg:
        mounted_centers.setdefault(conveyor_name, amr_cfg["conveyor_mount_center"])

    for item in scene_cfg.get("static_obstacles", []):
        center = np.asarray(item["center"], dtype=float)
        if dock is not None and item["name"] in mounted_centers:
            center = dock + np.asarray(mounted_centers[item["name"]], dtype=float)
        obb = OBB(
            center=center,
            half_extents=0.5 * np.asarray(item["size"], dtype=float),
            rotation=rotation_matrix_from_rpy(*item.get("rpy", [0.0, 0.0, 0.0])),
            name=str(item["name"]),
            category="static",
        )
        primitives.append(_serialize_obb(obb))

    validation_cfg = cfg.get("simulation_validation", {})
    conveyor_cfg = validation_cfg.get("conveyor", {})
    if bool(conveyor_cfg.get("enabled", False)):
        if dock is None:
            raise ValueError("an enabled AMR conveyor requires a segment dock position")
        outfeed_name = str(conveyor_cfg.get("outfeed_name", "conveyor_outfeed"))
        outfeed_offset = np.asarray(
            conveyor_cfg.get("outfeed_mount_center", []), dtype=float
        )
        outfeed_size = np.asarray(conveyor_cfg.get("outfeed_size_m", []), dtype=float)
        if outfeed_offset.shape != (3,) or outfeed_size.shape != (3,):
            raise ValueError("conveyor outfeed center and size must contain three values")
        if not np.all(np.isfinite(outfeed_offset)) or not np.all(np.isfinite(outfeed_size)):
            raise ValueError("conveyor outfeed geometry must be finite")
        if np.any(outfeed_size <= 0.0):
            raise ValueError("conveyor outfeed size must be positive")
        primitives.append(
            _serialize_obb(
                OBB(
                    center=dock + outfeed_offset,
                    half_extents=0.5 * outfeed_size,
                    rotation=np.eye(3),
                    name=outfeed_name,
                    category="static",
                )
            )
        )

    if dock is not None and "footprint_size" in amr_cfg:
        size = np.asarray(amr_cfg["footprint_size"], dtype=float)
        offset = np.asarray(amr_cfg.get("platform_center_offset", [0.0, 0.0, size[2] / 2.0]))
        primitives.append(
            _serialize_obb(
                OBB(
                    center=dock + offset,
                    half_extents=0.5 * size,
                    rotation=np.eye(3),
                    name="amr_base",
                    category="amr",
                )
            )
        )

    removed_names = {
        str(item.get("target")) for item in plan["segments"][:segment_index] if item.get("target")
    }
    target_name = str(segment.get("target", ""))
    carton_mass = float(cfg.get("simulation_validation", {}).get("carton_mass_kg", 7.0))
    if not np.isfinite(carton_mass) or carton_mass <= 0.0:
        raise ValueError("simulation_validation.carton_mass_kg must be finite and positive")
    for item in scene_cfg.get("cartons", []):
        name = str(item["name"])
        if name in removed_names:
            continue
        carton = OBB(
            center=np.asarray(item["center"], dtype=float),
            half_extents=0.5 * np.asarray(item["size"], dtype=float),
            rotation=rotation_matrix_from_rpy(*item.get("rpy", [0.0, 0.0, 0.0])),
            name=name,
            category="carton",
        )
        # Only the picked carton needs to be dynamic in a single-segment
        # qualification run. Its neighbours remain exact collision geometry.
        primitives.append(
            _serialize_obb(carton, dynamic=name == target_name, mass_kg=carton_mass)
        )
    return primitives


@dataclass(frozen=True)
class IsaacReplayBundle:
    timestamps_seconds: np.ndarray
    positions_rad: np.ndarray
    metadata: dict[str, Any]

    def __post_init__(self) -> None:
        timestamps = np.asarray(self.timestamps_seconds, dtype=float)
        positions = np.asarray(self.positions_rad, dtype=float)
        if timestamps.ndim != 1 or positions.ndim != 2 or len(timestamps) != len(positions):
            raise ValueError("Isaac replay commands require (N,) timestamps and (N, dof) positions")
        if len(timestamps) < 2 or timestamps[0] != 0.0 or np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("Isaac replay timestamps must start at zero and increase strictly")
        if not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(positions)):
            raise ValueError("Isaac replay commands must be finite")
        object.__setattr__(self, "timestamps_seconds", timestamps.copy())
        object.__setattr__(self, "positions_rad", positions.copy())

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": "isaacsim_fanuc_replay_v1",
            "timestamps_seconds": self.timestamps_seconds.tolist(),
            "positions_rad": self.positions_rad.tolist(),
            "metadata": self.metadata,
        }


def _sample_trajectory(
    timestamps: np.ndarray,
    positions: np.ndarray,
    period_seconds: float,
) -> tuple[np.ndarray, np.ndarray]:
    if not np.isfinite(period_seconds) or period_seconds <= 0.0:
        raise ValueError("controller period must be finite and positive")
    duration = float(timestamps[-1])
    command_times = np.arange(0.0, duration, period_seconds, dtype=float)
    if command_times.size == 0 or duration - command_times[-1] > 1e-12:
        command_times = np.append(command_times, duration)
    else:
        command_times[-1] = duration
    commands = np.column_stack(
        [np.interp(command_times, timestamps, positions[:, joint]) for joint in range(positions.shape[1])]
    )
    return command_times, commands


def _insert_command_hold(
    timestamps: np.ndarray,
    positions: np.ndarray,
    event_time_seconds: float,
    hold_seconds: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Insert an exact, stationary controller hold at a trajectory event."""
    event_time = float(event_time_seconds)
    hold = float(hold_seconds)
    if not np.isfinite(event_time) or event_time < 0.0 or event_time > timestamps[-1]:
        raise ValueError("controller hold event time is outside the command schedule")
    if not np.isfinite(hold) or hold < 0.0:
        raise ValueError("controller hold duration must be finite and non-negative")
    if hold == 0.0:
        return timestamps, positions

    event_position = np.asarray(
        [np.interp(event_time, timestamps, positions[:, joint]) for joint in range(positions.shape[1])]
    )
    tolerance = 1e-12
    before = timestamps < event_time - tolerance
    after = timestamps > event_time + tolerance
    held_times = np.concatenate(
        (timestamps[before], [event_time, event_time + hold], timestamps[after] + hold)
    )
    held_positions = np.vstack(
        (positions[before], event_position, event_position, positions[after])
    )
    return held_times, held_positions


def build_fanuc_isaac_replay_bundle(
    plan: dict[str, Any],
    cfg: dict[str, Any],
    *,
    segment_index: int = 0,
    controller_period_seconds: float | None = None,
) -> IsaacReplayBundle:
    """Build a limit-audited FANUC command stream from one planned segment."""
    robot = plan.get("robot")
    if not isinstance(robot, dict) or robot.get("model") != "fanuc_m20id35":
        raise ValueError("Isaac replay currently accepts FANUC M-20iD/35 plans only")
    segments = plan.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("plan contains no trajectory segments")
    if segment_index < 0 or segment_index >= len(segments):
        raise IndexError("segment index is outside the plan")

    segment = segments[segment_index]
    path = np.asarray(segment.get("path", []), dtype=float)
    if path.ndim != 2 or path.shape[1] != len(FANUC_JOINT_NAMES) or len(path) < 2:
        raise ValueError("FANUC segment path must have shape (N, 6) with at least two waypoints")
    limits = motion_limits_from_config(cfg, path.shape[1])
    execution = cfg.get("execution", {})
    loaded_motion_time_scale = float(execution.get("loaded_motion_time_scale", 1.0))
    release_index = int(segment.get("release_index", len(path) - 1))
    if not 0 <= release_index < len(path):
        raise ValueError("release_index is outside the source path")
    post_release_motion_limit_scale = float(
        execution.get("post_release_motion_limit_scale", limits.limit_scale)
    )
    if (
        not np.isfinite(post_release_motion_limit_scale)
        or not 0.0 < post_release_motion_limit_scale <= 1.0
    ):
        raise ValueError("post_release_motion_limit_scale must be in (0, 1]")

    # The release dwell is a commanded stop, so it is a genuine trajectory
    # phase boundary. Time the loaded prefix conservatively and the empty-tool
    # escape independently; otherwise the loaded motion scale needlessly keeps
    # the gripper beside a carton that is already moving on a live conveyor.
    prefix = time_parameterize_joint_path(path[: release_index + 1], limits)
    prefix = scale_timed_trajectory_window(
        prefix,
        int(segment.get("grasp_index", 0)),
        release_index,
        loaded_motion_time_scale,
    )
    prefix_audit = prefix.audit(limits)
    empty_limits = replace(
        limits,
        limit_scale=post_release_motion_limit_scale,
        source=f"{limits.source}; empty-tool post-release phase",
    )
    empty = None
    empty_audit = None
    if release_index < len(path) - 1:
        empty = time_parameterize_joint_path(path[release_index:], empty_limits)
        empty_audit = empty.audit(empty_limits)
        source_motion_times = np.concatenate(
            (
                prefix.time_from_start,
                prefix.duration_seconds + empty.time_from_start[1:],
            )
        )
    else:
        source_motion_times = prefix.time_from_start
    audit = {
        **prefix_audit,
        "duration_seconds": float(source_motion_times[-1]),
        "within_limits": bool(
            prefix_audit["within_limits"]
            and (empty_audit is None or empty_audit["within_limits"])
        ),
        "phase_model": "stopped_release_then_empty_tool",
        "pre_release": prefix_audit,
        "post_release": empty_audit,
    }
    if not audit["within_limits"]:
        raise RuntimeError("trajectory cannot be exported because its timing audit failed")
    replay_time_scale = float(cfg.get("execution", {}).get("isaac_replay_time_scale", 1.0))
    if not np.isfinite(replay_time_scale) or replay_time_scale < 1.0:
        raise ValueError("execution.isaac_replay_time_scale must be finite and at least one")
    replay_motion_times = source_motion_times * replay_time_scale

    if controller_period_seconds is None:
        controller_period_seconds = float(
            cfg.get("planning", {}).get("trajectory_waypoint_period_seconds", 0.02)
        )
    command_times, commands = _sample_trajectory(
        replay_motion_times, path, float(controller_period_seconds)
    )

    def event_time(index_key: str) -> float | None:
        value = segment.get(index_key)
        if value is None:
            return None
        index = int(value)
        if index < 0 or index >= len(replay_motion_times):
            raise ValueError(f"{index_key} is outside the source path")
        return float(replay_motion_times[index])

    pre_grasp_settle_seconds = float(
        execution.get("pre_grasp_controller_settle_seconds", 0.0)
    )
    vacuum_establish_seconds = float(execution.get("vacuum_establish_seconds", 0.0))
    release_seconds = float(execution.get("release_seconds", 0.0))
    if not np.isfinite(pre_grasp_settle_seconds) or pre_grasp_settle_seconds < 0.0:
        raise ValueError(
            "pre_grasp_controller_settle_seconds must be finite and non-negative"
        )
    if not np.isfinite(vacuum_establish_seconds) or vacuum_establish_seconds < 0.0:
        raise ValueError("vacuum_establish_seconds must be finite and non-negative")
    if not np.isfinite(release_seconds) or release_seconds < 0.0:
        raise ValueError("release_seconds must be finite and non-negative")

    source_grasp_time = event_time("grasp_index")
    source_release_time = event_time("release_index")
    grasp_arrival_time = source_grasp_time
    grasp_time = source_grasp_time
    if source_grasp_time is not None:
        command_times, commands = _insert_command_hold(
            command_times, commands, source_grasp_time, pre_grasp_settle_seconds
        )
        grasp_time += pre_grasp_settle_seconds
        command_times, commands = _insert_command_hold(
            command_times, commands, grasp_time, vacuum_establish_seconds
        )
    release_arrival_time = source_release_time
    if release_arrival_time is not None and source_grasp_time is not None and source_release_time > source_grasp_time:
        release_arrival_time += pre_grasp_settle_seconds + vacuum_establish_seconds
    release_time = release_arrival_time
    if release_arrival_time is not None:
        command_times, commands = _insert_command_hold(
            command_times, commands, release_arrival_time, release_seconds
        )
        release_time += release_seconds
    effort_limits = np.asarray(execution.get("joint_effort_limits_nm", []), dtype=float)
    if effort_limits.shape not in {(0,), (len(FANUC_JOINT_NAMES),)}:
        raise ValueError("joint_effort_limits_nm must contain one value per FANUC joint")
    if effort_limits.size and (not np.all(np.isfinite(effort_limits)) or np.any(effort_limits <= 0.0)):
        raise ValueError("joint effort limits must be finite and positive")

    validation_cfg = cfg.get("simulation_validation", {})
    footprint_size = np.asarray(
        validation_cfg.get("vacuum_footprint_size_m", [0.30, 0.40]), dtype=float
    )
    if footprint_size.shape != (2,) or not np.all(np.isfinite(footprint_size)) or np.any(footprint_size <= 0.0):
        raise ValueError("vacuum footprint must contain two finite positive dimensions")
    effective_seal_area = validation_cfg.get("vacuum_effective_seal_area_m2")
    if effective_seal_area is not None:
        effective_seal_area = float(effective_seal_area)
        geometric_area = float(np.prod(footprint_size))
        if not np.isfinite(effective_seal_area) or not 0.0 < effective_seal_area <= geometric_area:
            raise ValueError("effective vacuum seal area must be positive and no larger than the footprint")
    def optional_positive(name: str):
        value = validation_cfg.get(name)
        if value is None:
            return None
        result = float(value)
        if not np.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be null or finite and positive")
        return result

    product_model = str(validation_cfg.get("vacuum_product_model", "")).strip()
    cup_model = str(validation_cfg.get("vacuum_cup_model", "")).strip()
    physical_cup_count = int(validation_cfg.get("vacuum_cup_count", 0))
    if not product_model or not cup_model or physical_cup_count <= 0:
        raise ValueError("vacuum product model, cup model, and physical cup count are required")
    holding_torque = optional_positive("vacuum_holding_torque_nm")
    pull_off_force_per_cup = optional_positive("vacuum_pull_off_force_per_cup_n")
    shear_force_per_cup = optional_positive("vacuum_shear_force_per_cup_n")
    if pull_off_force_per_cup is None or shear_force_per_cup is None:
        raise ValueError("per-cup axial pull-off and shear forces are required")
    cup_rows = int(validation_cfg.get("vacuum_cup_rows", 0))
    cup_columns = int(validation_cfg.get("vacuum_cup_columns", 0))
    cup_pitch = np.asarray(validation_cfg.get("vacuum_cup_pitch_m", []), dtype=float)
    cup_radius = float(validation_cfg.get("vacuum_cup_radius_m", 0.0))
    zone_count = int(validation_cfg.get("vacuum_zone_count", 1))
    if (
        cup_rows <= 0
        or cup_columns <= 0
        or cup_rows * cup_columns != physical_cup_count
        or cup_pitch.shape != (2,)
        or not np.all(np.isfinite(cup_pitch))
        or np.any(cup_pitch <= 0.0)
        or not np.isfinite(cup_radius)
        or cup_radius <= 0.0
        or zone_count <= 0
        or cup_columns % zone_count != 0
    ):
        raise ValueError("vacuum cup grid, radius, count, and zones are inconsistent")
    width_offsets = (np.arange(cup_rows) - 0.5 * (cup_rows - 1)) * cup_pitch[0]
    length_offsets = (np.arange(cup_columns) - 0.5 * (cup_columns - 1)) * cup_pitch[1]
    cup_centers_tool_yz = np.asarray(
        [(length, -width) for length in length_offsets for width in width_offsets],
        dtype=float,
    )
    attachment_model = str(validation_cfg.get("vacuum_attachment_model", "all_cups"))
    active_cup_indices = tuple(int(index) for index in segment.get("sealed_cup_indices", []))
    if attachment_model == "per_sealed_cup" and not active_cup_indices:
        raise ValueError("per-sealed-cup attachment requires sealed cup evidence in the plan")
    if not active_cup_indices:
        active_cup_indices = tuple(range(physical_cup_count))
    if len(set(active_cup_indices)) != len(active_cup_indices) or any(
        index < 0 or index >= physical_cup_count for index in active_cup_indices
    ):
        raise ValueError("sealed cup indices must be unique and inside the physical cup grid")
    active_cup_count = len(active_cup_indices)
    solver_attachment_model = str(
        validation_cfg.get("vacuum_solver_attachment_model", "per_sealed_cup")
    )
    solver_position_iterations = int(
        validation_cfg.get("vacuum_solver_position_iterations", 32)
    )
    solver_velocity_iterations = int(
        validation_cfg.get("vacuum_solver_velocity_iterations", 8)
    )
    if solver_position_iterations <= 0 or solver_velocity_iterations <= 0:
        raise ValueError("vacuum solver iteration counts must be positive")
    if solver_attachment_model not in {
        "per_sealed_cup",
        "equivalent_center_of_pressure",
        "equivalent_zone_row_band_centers",
    }:
        raise ValueError("vacuum solver attachment model is unsupported")
    active_cup_centers = cup_centers_tool_yz[np.asarray(active_cup_indices, dtype=int)]
    if solver_attachment_model == "equivalent_center_of_pressure":
        solver_attachment_offsets = np.mean(active_cup_centers, axis=0, keepdims=True)
    elif solver_attachment_model == "equivalent_zone_row_band_centers":
        # Six non-collinear solver points retain the known 3-zone x 2-row-band
        # spatial support without creating 72 nearly coincident rigid joints.
        # This is a numerical attachment representation; physical force and
        # moment qualification continues to use the full sealed-cup evidence.
        columns_per_zone = cup_columns // zone_count
        row_split = cup_rows // 2
        grouped_centers: list[np.ndarray] = []
        for zone_index in range(zone_count):
            for row_band in range(2):
                group_indices = [
                    index
                    for index in active_cup_indices
                    if (index // cup_rows) // columns_per_zone == zone_index
                    and (0 if (index % cup_rows) < row_split else 1) == row_band
                ]
                if group_indices:
                    grouped_centers.append(
                        np.mean(cup_centers_tool_yz[np.asarray(group_indices, dtype=int)], axis=0)
                    )
        if len(grouped_centers) < 3:
            raise ValueError("zone-row-band solver model requires at least three populated groups")
        solver_attachment_offsets = np.asarray(grouped_centers, dtype=float)
    else:
        solver_attachment_offsets = active_cup_centers
    holding_force = pull_off_force_per_cup * active_cup_count
    shear_force = shear_force_per_cup * active_cup_count
    hardware_maximum_holding_force = pull_off_force_per_cup * physical_cup_count
    hardware_maximum_shear_force = shear_force_per_cup * physical_cup_count
    catalogue_theoretical_force = optional_positive(
        "vacuum_catalog_theoretical_total_force_n_at_minus_60_kpa"
    )
    max_grip_distance = float(validation_cfg.get("vacuum_max_grip_distance_m", 0.03))
    cup_compression = float(validation_cfg.get("vacuum_cup_compression_m", 0.0))
    capture_tolerance = float(
        validation_cfg.get("vacuum_surface_gripper_capture_tolerance_m", 0.0)
    )
    if any(
        not np.isfinite(value) or value <= 0.0
        for value in (holding_force, max_grip_distance, cup_compression)
    ) or not np.isfinite(capture_tolerance) or capture_tolerance < 0.0:
        raise ValueError("vacuum total force and grip distance must be finite and positive")
    if not np.isclose(max_grip_distance, cup_compression + capture_tolerance):
        raise ValueError("surface gripper distance must equal cup compression plus solver tolerance")
    gripper_mass = float(validation_cfg.get("vacuum_gripper_mass_kg", 0.0))
    gripper_com = np.asarray(
        validation_cfg.get("vacuum_center_of_mass_from_flange_m", []), dtype=float
    )
    gripper_inertia = np.asarray(
        validation_cfg.get("vacuum_inertia_at_com_kg_m2", []), dtype=float
    )
    flange_origin_step = np.asarray(
        validation_cfg.get("vacuum_flange_origin_step_mm", []), dtype=float
    )
    step_from_tool_rotation = np.asarray(
        validation_cfg.get("vacuum_step_from_tool_rotation_matrix", []), dtype=float
    )
    if (
        not np.isfinite(gripper_mass)
        or gripper_mass <= 0.0
        or gripper_com.shape != (3,)
        or not np.all(np.isfinite(gripper_com))
        or gripper_inertia.shape != (3, 3)
        or not np.all(np.isfinite(gripper_inertia))
        or flange_origin_step.shape != (3,)
        or not np.all(np.isfinite(flange_origin_step))
        or step_from_tool_rotation.shape != (3, 3)
        or not np.all(np.isfinite(step_from_tool_rotation))
        or not np.allclose(step_from_tool_rotation.T @ step_from_tool_rotation, np.eye(3))
        or not np.isclose(np.linalg.det(step_from_tool_rotation), 1.0)
    ):
        raise ValueError("gripper mass properties and STEP-to-tool transform are required")

    base_position = np.asarray(robot.get("base_position", [0.0, 0.0, 0.0]), dtype=float)
    dock_value = segment.get("amr_dock_position")
    robot_mount = cfg.get("amr", {}).get("robot_mount_position")
    if dock_value is not None and robot_mount is not None:
        base_position = np.asarray(dock_value, dtype=float) + np.asarray(robot_mount, dtype=float)

    metadata = {
        "robot_model": "fanuc_m20id35",
        "source_plan_sha256": _canonical_digest(plan),
        "merged_configuration_sha256": _canonical_digest(cfg),
        "joint_names": list(FANUC_JOINT_NAMES),
        "urdf_path": str(robot["urdf_path"]),
        "base_position_m": base_position.tolist(),
        "base_rpy_rad": list(robot.get("base_rpy", [0.0, 0.0, 0.0])),
        "tool_length_m": float(robot.get("tool_length", 0.0)),
        "segment_index": int(segment_index),
        "pick_index": int(segment.get("pick_index", segment_index)),
        "target": str(segment.get("target", "unknown")),
        "offline_planning_time_seconds": float(plan.get("planning_time_seconds", 0.0)),
        "source_waypoint_count": int(len(path)),
        "controller_period_seconds": float(controller_period_seconds),
        "command_count": int(len(command_times)),
        "duration_seconds": float(command_times[-1]),
        "motion_duration_seconds": float(replay_motion_times[-1]),
        "source_motion_duration_seconds": float(source_motion_times[-1]),
        "isaac_replay_time_scale": replay_time_scale,
        "timing_audit": {**audit, "isaac_replay_time_scale": replay_time_scale},
        "limits_source": limits.source,
        "joint_velocity_limits_rad_s": limits.velocity.tolist(),
        "joint_effort_limits_nm": effort_limits.tolist(),
        "grasp_arrival_time_seconds": grasp_arrival_time,
        "pre_grasp_controller_settle_seconds": pre_grasp_settle_seconds,
        "grasp_time_seconds": grasp_time,
        "vacuum_establish_seconds": vacuum_establish_seconds,
        "release_arrival_time_seconds": release_arrival_time,
        "release_time_seconds": release_time,
        "release_hold_seconds": release_seconds,
        "loaded_motion_time_scale": loaded_motion_time_scale,
        "post_release_motion_limit_scale": post_release_motion_limit_scale,
        "release_retreat_time_seconds": event_time("release_retreat_index"),
        "place_center_m": list(segment.get("place_center", [])),
        "release_center_m": list(segment.get("release_center", [])),
        "place_surface": segment.get("place_surface"),
        "free_fall_height_m": float(segment.get("free_fall_height_m", 0.0)),
        "collision_geometry": "urdf_collision_mesh",
        "scene_primitives": _build_scene_primitives(plan, cfg, segment_index),
        "camera": dict(cfg.get("simulation_validation", {}).get("camera", {})),
        "rendering": dict(cfg.get("simulation_validation", {}).get("rendering", {})),
        "conveyor": dict(cfg.get("simulation_validation", {}).get("conveyor", {})),
        "gripper": {
            "product_model": product_model,
            "cup_model": cup_model,
            "physical_cup_count": physical_cup_count,
            "active_sealed_cup_count": active_cup_count,
            "active_sealed_cup_indices": list(active_cup_indices),
            "sealed_cups_per_zone": list(segment.get("sealed_cups_per_zone", [])),
            "cup_rows": cup_rows,
            "cup_columns": cup_columns,
            "cup_pitch_m": cup_pitch.tolist(),
            "cup_radius_m": cup_radius,
            "cup_centers_tool_yz_m": cup_centers_tool_yz.tolist(),
            "holding_force_n": holding_force,
            "holding_force_total_n": holding_force,
            "hardware_maximum_holding_force_total_n": hardware_maximum_holding_force,
            "holding_force_derivation": "pull_off_force_per_cup_n_times_active_sealed_cup_count",
            "holding_torque_nm": holding_torque,
            "pull_off_force_per_cup_n": pull_off_force_per_cup,
            "shear_force_per_cup_n": shear_force_per_cup,
            "catalogue_theoretical_total_force_n_at_minus_60_kpa": (
                catalogue_theoretical_force
            ),
            "footprint_size_m": footprint_size.tolist(),
            "effective_seal_area_m2": effective_seal_area,
            "limits_calibrated": bool(
                cfg.get("simulation_validation", {}).get("vacuum_limits_calibrated", False)
            ),
            "shear_force_n": shear_force,
            "shear_force_total_n": shear_force,
            "hardware_maximum_shear_force_total_n": hardware_maximum_shear_force,
            "max_grip_distance_m": max_grip_distance,
            "physical_cup_compression_m": cup_compression,
            "surface_gripper_capture_tolerance_m": capture_tolerance,
            "attachment_model": attachment_model,
            "attachment_point_count": active_cup_count,
            "solver_attachment_model": solver_attachment_model,
            "solver_attachment_offsets_tool_yz_m": solver_attachment_offsets.tolist(),
            "simulation_attachment_point_count": int(len(solver_attachment_offsets)),
            "solver_position_iterations": solver_position_iterations,
            "solver_velocity_iterations": solver_velocity_iterations,
            "gripper_mass_kg": gripper_mass,
            "center_of_mass_from_flange_m": gripper_com.tolist(),
            "inertia_at_com_kg_m2": gripper_inertia.tolist(),
            "flange_origin_step_mm": flange_origin_step.tolist(),
            "step_from_tool_rotation_matrix": step_from_tool_rotation.tolist(),
            "outer_size_m": list(validation_cfg.get("vacuum_outer_size_m", [])),
            "step_path": validation_cfg.get("vacuum_step_path"),
            "visual_mesh_path": validation_cfg.get("vacuum_visual_mesh_path"),
            "collision_mesh_path": validation_cfg.get("vacuum_collision_mesh_path"),
            "mass_properties_path": validation_cfg.get("vacuum_mass_properties_path"),
            "flange_transform_confidence": validation_cfg.get(
                "vacuum_flange_transform_confidence"
            ),
            "zone_count": zone_count,
            "zone_assignment_confirmed": False,
            "model_source": "step_geometry_with_per_cup_forces_and_provisional_zone_mapping",
        },
    }
    return IsaacReplayBundle(command_times, commands, metadata)
