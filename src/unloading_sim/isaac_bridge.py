"""Backend-neutral command bundles for Isaac Sim trajectory validation.

This module deliberately imports no Isaac Sim, USD, CUDA, or ROS packages.
It converts a collision-checked geometric plan into a deterministic controller
command schedule that a heavyweight simulator adapter can consume.
"""

from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from .geometry import OBB, rotation_matrix_from_rpy
from .identity import normalize_robot_model_id
from .m710_replay_contract import (
    add_bundle_payload_sha256,
    build_m710_replay_contract,
)
from .scene import build_trailer_walls
from .timing import (
    motion_limits_from_config,
    scale_timed_trajectory_window,
    time_parameterize_joint_path,
)


FANUC_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
SUPPORTED_FANUC_REPLAY_MODELS = frozenset({"fanuc_m20id_35", "fanuc_m710id_70"})


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


def _declared_scene_primitives(plan: dict[str, Any]) -> list[dict[str, Any]] | None:
    """Validate a frozen, layout-bound scene supplied by a planning adapter.

    Legacy M-20 plans still use ``_build_scene_primitives`` below.  New workcell
    adapters must serialize the already verified frozen snapshot and place it
    in the plan; rebuilding that scene from mutable legacy configuration would
    silently change carton identities and known/unknown trailer boundaries.
    """
    declared = plan.get("scene_primitives")
    if declared is None:
        return None
    if not isinstance(declared, list) or not declared:
        raise ValueError("plan.scene_primitives must be a non-empty list")
    result: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, source in enumerate(declared):
        if not isinstance(source, dict):
            raise ValueError(f"plan.scene_primitives[{index}] must be a mapping")
        name = str(source.get("name", "")).strip()
        category = str(source.get("category", "")).strip()
        if not name or not category or name in names:
            raise ValueError("layout-bound scene primitive names/categories must be nonempty and unique")
        center = np.asarray(source.get("center_m", []), dtype=float)
        size = np.asarray(source.get("size_m", []), dtype=float)
        rotation = np.asarray(source.get("rotation_matrix", []), dtype=float)
        if (
            center.shape != (3,)
            or size.shape != (3,)
            or rotation.shape != (3, 3)
            or not np.all(np.isfinite(center))
            or not np.all(np.isfinite(size))
            or not np.all(np.isfinite(rotation))
            or np.any(size <= 0.0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0.0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-9, rtol=0.0)
        ):
            raise ValueError(f"invalid layout-bound scene primitive geometry: {name}")
        dynamic = bool(source.get("dynamic", False))
        item = {
            "name": name,
            "category": category,
            "center_m": center.tolist(),
            "size_m": size.tolist(),
            "rotation_matrix": rotation.tolist(),
            "dynamic": dynamic,
        }
        if dynamic:
            mass = float(source.get("mass_kg", float("nan")))
            inertia = np.asarray(source.get("inertia_at_com_kg_m2", []), dtype=float)
            if (
                not np.isfinite(mass)
                or mass <= 0.0
                or inertia.shape != (3, 3)
                or not np.all(np.isfinite(inertia))
                or not np.allclose(inertia, inertia.T, atol=1e-10, rtol=0.0)
                or np.min(np.linalg.eigvalsh(inertia)) <= 0.0
            ):
                raise ValueError(f"dynamic primitive requires positive mass and inertia: {name}")
            item["mass_kg"] = mass
            item["inertia_at_com_kg_m2"] = inertia.tolist()
        material = source.get("material")
        if material is not None:
            if not isinstance(material, str) or not material.strip():
                raise ValueError(f"invalid physics material name for scene primitive: {name}")
            item["material"] = material
        boundary = source.get("boundary")
        if boundary is not None:
            if not isinstance(boundary, dict) or set(boundary) != {
                "axis",
                "value_m",
                "inside",
                "extent_status",
            }:
                raise ValueError(f"invalid boundary contract for scene primitive: {name}")
            axis = boundary["axis"]
            inside = boundary["inside"]
            value = float(boundary["value_m"])
            if (
                axis not in {"x", "y", "z"}
                or inside not in {"+", "-"}
                or not np.isfinite(value)
                or not isinstance(boundary["extent_status"], str)
                or not boundary["extent_status"].strip()
            ):
                raise ValueError(f"invalid boundary contract for scene primitive: {name}")
            item["boundary"] = {
                "axis": axis,
                "value_m": value,
                "inside": inside,
                "extent_status": boundary["extent_status"],
            }
        result.append(item)
        names.add(name)
    return result


def _validated_link_dynamics(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise ValueError("robot_link_dynamics must be a list")
    result: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, source in enumerate(value):
        if not isinstance(source, dict):
            raise ValueError(f"robot_link_dynamics[{index}] must be a mapping")
        name = str(source.get("link", "")).strip()
        mass = float(source.get("mass_kg", float("nan")))
        center = np.asarray(source.get("com_xyz_m", []), dtype=float)
        inertia = np.asarray(source.get("inertia_at_com_kg_m2", []), dtype=float)
        if not name or name in names:
            raise ValueError("robot dynamic link names must be nonempty and unique")
        eigenvalues = np.linalg.eigvalsh(inertia) if inertia.shape == (3, 3) else np.asarray([])
        if (
            not np.isfinite(mass)
            or mass <= 0.0
            or center.shape != (3,)
            or not np.all(np.isfinite(center))
            or inertia.shape != (3, 3)
            or not np.all(np.isfinite(inertia))
            or not np.allclose(inertia, inertia.T, atol=1e-10, rtol=0.0)
            or eigenvalues.shape != (3,)
            or np.any(eigenvalues <= 0.0)
            or 2.0 * np.max(eigenvalues) > np.sum(eigenvalues) + 1e-10
        ):
            raise ValueError(f"invalid physical mass properties for robot link {name!r}")
        result.append(
            {
                "link": name,
                "mass_kg": mass,
                "com_xyz_m": center.tolist(),
                "inertia_at_com_kg_m2": inertia.tolist(),
                "source_status": str(source.get("source_status", "UNSPECIFIED")),
            }
        )
        names.add(name)
    return result


def _combine_fixed_mass_properties(
    first_mass_kg: float,
    first_com_m: np.ndarray,
    first_inertia_at_com_kg_m2: np.ndarray,
    second_mass_kg: float,
    second_com_m: np.ndarray,
    second_inertia_at_com_kg_m2: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Combine two fixed bodies expressed in one common body frame."""

    first_mass = float(first_mass_kg)
    second_mass = float(second_mass_kg)
    first_com = np.asarray(first_com_m, dtype=float)
    second_com = np.asarray(second_com_m, dtype=float)
    first_inertia = np.asarray(first_inertia_at_com_kg_m2, dtype=float)
    second_inertia = np.asarray(second_inertia_at_com_kg_m2, dtype=float)
    total_mass = first_mass + second_mass
    if (
        not np.isfinite(first_mass)
        or not np.isfinite(second_mass)
        or first_mass <= 0.0
        or second_mass <= 0.0
        or first_com.shape != (3,)
        or second_com.shape != (3,)
        or first_inertia.shape != (3, 3)
        or second_inertia.shape != (3, 3)
        or not np.all(np.isfinite(first_com))
        or not np.all(np.isfinite(second_com))
        or not np.all(np.isfinite(first_inertia))
        or not np.all(np.isfinite(second_inertia))
    ):
        raise ValueError("fixed-body mass properties must be finite and positive")
    combined_com = (first_mass * first_com + second_mass * second_com) / total_mass

    def shifted(inertia: np.ndarray, mass: float, source_com: np.ndarray) -> np.ndarray:
        offset = source_com - combined_com
        return inertia + mass * (
            float(offset @ offset) * np.eye(3) - np.outer(offset, offset)
        )

    combined_inertia = shifted(first_inertia, first_mass, first_com) + shifted(
        second_inertia, second_mass, second_com
    )
    return total_mass, combined_com, combined_inertia


def _validated_physics_contract(
    value: Any,
    *,
    solver_position_iterations: int,
    solver_velocity_iterations: int,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("M-710 physics contract must be a mapping")
    required = {
        "parameter_status",
        "gravity_world_m_s2",
        "physics_time_step_s",
        "materials",
        "material_by_category",
        "damping",
        "settling",
    }
    if set(value) != required:
        raise ValueError("M-710 physics contract keys are incomplete")
    gravity = np.asarray(value["gravity_world_m_s2"], dtype=float)
    time_step = float(value["physics_time_step_s"])
    if (
        not isinstance(value["parameter_status"], str)
        or not value["parameter_status"].strip()
        or gravity.shape != (3,)
        or not np.all(np.isfinite(gravity))
        or np.linalg.norm(gravity) <= 0.0
        or not np.isfinite(time_step)
        or time_step <= 0.0
    ):
        raise ValueError("M-710 gravity and physics time step must be explicit and finite")

    materials = value["materials"]
    if not isinstance(materials, dict) or not materials:
        raise ValueError("M-710 physics materials must be a non-empty mapping")
    audited_materials: dict[str, dict[str, float]] = {}
    for name, raw in materials.items():
        if not isinstance(name, str) or not name or not isinstance(raw, dict) or set(raw) != {
            "static_friction",
            "dynamic_friction",
            "restitution",
        }:
            raise ValueError("invalid M-710 physics material contract")
        static = float(raw["static_friction"])
        dynamic = float(raw["dynamic_friction"])
        restitution = float(raw["restitution"])
        if (
            not all(np.isfinite(item) for item in (static, dynamic, restitution))
            or static < 0.0
            or dynamic < 0.0
            or dynamic > static
            or not 0.0 <= restitution <= 1.0
        ):
            raise ValueError(f"invalid M-710 physics material coefficients: {name}")
        audited_materials[name] = {
            "static_friction": static,
            "dynamic_friction": dynamic,
            "restitution": restitution,
        }

    material_by_category = value["material_by_category"]
    if not isinstance(material_by_category, dict) or any(
        not isinstance(category, str)
        or not category
        or material not in audited_materials
        for category, material in material_by_category.items()
    ):
        raise ValueError("M-710 scene material mapping references an unknown material")

    damping = value["damping"]
    if not isinstance(damping, dict) or not damping:
        raise ValueError("M-710 rigid-body damping must be a non-empty mapping")
    audited_damping: dict[str, dict[str, float]] = {}
    for name, raw in damping.items():
        if not isinstance(name, str) or not name or not isinstance(raw, dict) or set(raw) != {
            "linear_damping_s_inv",
            "angular_damping_s_inv",
        }:
            raise ValueError("invalid M-710 rigid-body damping contract")
        linear = float(raw["linear_damping_s_inv"])
        angular = float(raw["angular_damping_s_inv"])
        if not all(np.isfinite(item) and item >= 0.0 for item in (linear, angular)):
            raise ValueError(f"invalid M-710 rigid-body damping coefficients: {name}")
        audited_damping[name] = {
            "linear_damping_s_inv": linear,
            "angular_damping_s_inv": angular,
        }
    if set(audited_damping) != {"robot_links", "tool", "cartons"}:
        raise ValueError("M-710 damping must cover robot_links, tool, and cartons")
    if audited_damping["tool"] != audited_damping["robot_links"]:
        raise ValueError(
            "fixed M-710 tool/J6 mass combination requires identical tool and robot-link damping"
        )

    settling = value["settling"]
    settling_keys = {
        "maximum_settle_time_s",
        "required_stable_duration_s",
        "max_linear_speed_m_s",
        "max_angular_speed_rad_s",
        "max_position_drift_m",
        "max_penetration_m",
    }
    if not isinstance(settling, dict) or set(settling) != settling_keys:
        raise ValueError("M-710 settling contract is incomplete")
    audited_settling = {name: float(settling[name]) for name in settling_keys}
    if any(not np.isfinite(item) or item <= 0.0 for item in audited_settling.values()):
        raise ValueError("M-710 settling thresholds must be finite and positive")
    if audited_settling["required_stable_duration_s"] > audited_settling["maximum_settle_time_s"]:
        raise ValueError("M-710 stable duration exceeds the maximum settle time")
    if solver_position_iterations <= 0 or solver_velocity_iterations <= 0:
        raise ValueError("M-710 solver iteration counts must be positive")
    return {
        "parameter_status": value["parameter_status"],
        "gravity_world_m_s2": gravity.tolist(),
        "physics_time_step_s": time_step,
        "solver_position_iterations": int(solver_position_iterations),
        "solver_velocity_iterations": int(solver_velocity_iterations),
        "friction_combine_mode": "min",
        "restitution_combine_mode": "min",
        "materials": audited_materials,
        "material_by_category": dict(material_by_category),
        "damping": audited_damping,
        "settling": audited_settling,
    }


def _validated_rendering_contract(value: Any, *, physics_time_step_s: float) -> dict[str, Any]:
    """Validate the content-addressed recording contract used by M-710 replay."""

    if not isinstance(value, dict) or set(value) != {"required_output"}:
        raise ValueError("M-710 rendering contract must contain required_output")
    required = value["required_output"]
    expected_keys = {"width_px", "height_px", "fps", "camera_mode"}
    if not isinstance(required, dict) or set(required) != expected_keys:
        raise ValueError("M-710 required output contract is incomplete")
    width = required["width_px"]
    height = required["height_px"]
    fps = required["fps"]
    camera_mode = required["camera_mode"]
    if (
        isinstance(width, bool)
        or isinstance(height, bool)
        or isinstance(fps, bool)
        or not isinstance(width, int)
        or not isinstance(height, int)
        or not isinstance(fps, int)
        or width <= 0
        or height <= 0
        or fps <= 0
        or not isinstance(camera_mode, str)
        or not camera_mode.strip()
    ):
        raise ValueError("M-710 required output values must be positive integers and a camera mode")
    physics_hz = 1.0 / float(physics_time_step_s)
    render_stride = physics_hz / fps
    if not np.isclose(render_stride, round(render_stride), atol=1e-9, rtol=0.0):
        raise ValueError("M-710 output FPS must divide the physics rate into an integer render stride")
    return {
        "required_output": {
            "width_px": width,
            "height_px": height,
            "fps": fps,
            "camera_mode": camera_mode,
            "render_every_physics_steps": int(round(render_stride)),
        }
    }


def _build_scene_primitives(
    plan: dict[str, Any], cfg: dict[str, Any], segment_index: int
) -> list[dict[str, Any]]:
    """Serialize the selected docked scene without importing a physics backend."""
    declared = _declared_scene_primitives(plan)
    if declared is not None:
        return declared
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
        return add_bundle_payload_sha256({
            "format": "isaacsim_fanuc_replay_v1",
            "timestamps_seconds": self.timestamps_seconds.tolist(),
            "positions_rad": self.positions_rad.tolist(),
            "metadata": copy.deepcopy(self.metadata),
        })


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
    preflight: dict[str, Any] | None = None,
) -> IsaacReplayBundle:
    """Build a limit-audited FANUC command stream from one planned segment."""
    robot = plan.get("robot")
    if not isinstance(robot, dict):
        raise ValueError("Isaac replay requires a supported six-axis FANUC plan")
    try:
        robot_model_id = normalize_robot_model_id(robot.get("model"))
    except ValueError as exc:
        raise ValueError("Isaac replay requires a supported six-axis FANUC plan") from exc
    if robot_model_id not in SUPPORTED_FANUC_REPLAY_MODELS:
        raise ValueError("Isaac replay requires a supported six-axis FANUC plan")
    segments = plan.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("plan contains no trajectory segments")
    if segment_index < 0 or segment_index >= len(segments):
        raise IndexError("segment index is outside the plan")

    segment = segments[segment_index]
    m710_replay_contract = None
    if robot_model_id == "fanuc_m710id_70":
        if preflight is None:
            raise ValueError("M-710 replay requires a full verified execution preflight")
        if segment_index != 0 or len(segments) != 1:
            raise ValueError("M-710 replay preflight binds exactly one selected trajectory segment")
        m710_replay_contract = build_m710_replay_contract(
            preflight,
            plan,
            cfg,
            segment,
        )
    path = np.asarray(segment.get("path", []), dtype=float)
    if path.ndim != 2 or path.shape[1] != len(FANUC_JOINT_NAMES) or len(path) < 2:
        raise ValueError("FANUC segment path must have shape (N, 6) with at least two waypoints")
    limits = motion_limits_from_config(cfg, path.shape[1])
    execution = cfg.get("execution", {})
    loaded_motion_time_scale = float(execution.get("loaded_motion_time_scale", 1.0))
    release_index = int(segment.get("release_index", len(path) - 1))
    if not 0 <= release_index < len(path):
        raise ValueError("release_index is outside the source path")
    event_indices: dict[str, int] = {}
    for index_key in ("grasp_index", "release_index", "release_retreat_index"):
        raw_index = segment.get(index_key)
        if raw_index is None:
            continue
        index = int(raw_index)
        if index < 0 or index >= len(path):
            raise ValueError(f"{index_key} is outside the source path")
        event_indices[index_key] = index
    if (
        "grasp_index" in event_indices
        and "release_index" in event_indices
        and event_indices["grasp_index"] > event_indices["release_index"]
    ):
        raise ValueError("grasp_index must not follow release_index")
    if (
        "release_index" in event_indices
        and "release_retreat_index" in event_indices
        and event_indices["release_index"] > event_indices["release_retreat_index"]
    ):
        raise ValueError("release_index must not follow release_retreat_index")
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
    if (
        release_arrival_time is not None
        and source_grasp_time is not None
        and source_release_time >= source_grasp_time
    ):
        release_arrival_time += pre_grasp_settle_seconds + vacuum_establish_seconds
    release_time = release_arrival_time
    if release_arrival_time is not None:
        command_times, commands = _insert_command_hold(
            command_times, commands, release_arrival_time, release_seconds
        )
        release_time += release_seconds
    source_release_retreat_time = event_time("release_retreat_index")
    release_retreat_time = source_release_retreat_time
    if release_retreat_time is not None:
        if source_grasp_time is not None and source_release_retreat_time >= source_grasp_time:
            release_retreat_time += pre_grasp_settle_seconds + vacuum_establish_seconds
        if source_release_time is not None and source_release_retreat_time >= source_release_time:
            release_retreat_time += release_seconds
    effort_limits = np.asarray(execution.get("joint_effort_limits_nm", []), dtype=float)
    if effort_limits.shape not in {(0,), (len(FANUC_JOINT_NAMES),)}:
        raise ValueError("joint_effort_limits_nm must contain one value per FANUC joint")
    if effort_limits.size and (not np.all(np.isfinite(effort_limits)) or np.any(effort_limits <= 0.0)):
        raise ValueError("joint effort limits must be finite and positive")
    stiffness = np.asarray(execution.get("joint_drive_stiffness_nm_rad", []), dtype=float)
    damping = np.asarray(execution.get("joint_drive_damping_nm_s_rad", []), dtype=float)
    if robot_model_id == "fanuc_m20id_35" and not stiffness.size and not damping.size:
        # Preserve the legacy replay controller as explicit metadata.  New
        # M-710 contracts are never allowed to inherit these M-20 values.
        stiffness = np.asarray([80000.0, 80000.0, 60000.0, 12000.0, 8000.0, 5000.0])
        damping = np.asarray([5000.0, 5000.0, 4000.0, 800.0, 500.0, 300.0])
    raw_simulation_execution_ready = plan.get(
        "simulation_execution_ready", robot_model_id == "fanuc_m20id_35"
    )
    raw_execution_blockers = plan.get("execution_blockers", [])
    if not isinstance(raw_simulation_execution_ready, bool):
        raise ValueError("simulation_execution_ready must be a boolean")
    if not isinstance(raw_execution_blockers, list) or any(
        not isinstance(item, str) or not item.strip() for item in raw_execution_blockers
    ):
        raise ValueError("execution_blockers must be a list of non-empty strings")
    simulation_execution_ready = raw_simulation_execution_ready
    execution_blockers = list(raw_execution_blockers)
    machine_qualified = plan.get(
        "machine_qualified", robot_model_id == "fanuc_m20id_35"
    )
    machine_qualification_warnings = plan.get("machine_qualification_warnings", [])
    if not isinstance(machine_qualified, bool):
        raise ValueError("machine_qualified must be a boolean")
    if not isinstance(machine_qualification_warnings, list) or any(
        not isinstance(item, str) or not item.strip()
        for item in machine_qualification_warnings
    ):
        raise ValueError("machine_qualification_warnings must be a list of non-empty strings")
    execution_qualified = plan.get(
        "execution_qualified", simulation_execution_ready and machine_qualified
    )
    if not isinstance(execution_qualified, bool):
        raise ValueError("execution_qualified must be a boolean")
    if execution_qualified != (simulation_execution_ready and machine_qualified):
        raise ValueError(
            "execution_qualified must equal simulation_execution_ready AND machine_qualified"
        )
    if robot_model_id == "fanuc_m710id_70":
        if effort_limits.shape != (len(FANUC_JOINT_NAMES),):
            raise ValueError("M-710 engineering replay requires six explicit finite effort limits")
        if stiffness.shape != (len(FANUC_JOINT_NAMES),) or damping.shape != (len(FANUC_JOINT_NAMES),):
            raise ValueError("M-710 engineering replay requires six explicit drive gains")
        if (
            not np.all(np.isfinite(stiffness))
            or not np.all(np.isfinite(damping))
            or np.any(stiffness <= 0.0)
            or np.any(damping <= 0.0)
        ):
            raise ValueError("M-710 drive stiffness and damping must be finite and positive")
        execution_identity = str(plan.get("execution_asset_fingerprint_sha256", "")).strip()
        if len(execution_identity) != 64 or any(
            character not in "0123456789abcdef" for character in execution_identity
        ):
            raise ValueError("M-710 plan requires a complete execution asset fingerprint")
        if simulation_execution_ready == bool(execution_blockers):
            raise ValueError(
                "M-710 simulation readiness and blocker list are inconsistent"
            )
        scene_primitives = _declared_scene_primitives(plan)
        if scene_primitives is None:
            raise ValueError("M-710 replay requires frozen layout-bound scene primitives")
        cartons = [item for item in scene_primitives if item["category"] == "carton"]
        if len(cartons) != 40 or not all(item["dynamic"] for item in cartons):
            raise ValueError("M-710 replay requires all 40 cartons as dynamic rigid bodies")
        if sum(item["name"] == str(segment.get("target", "")) for item in cartons) != 1:
            raise ValueError("M-710 replay target must identify exactly one dynamic carton")

    validation_cfg = cfg.get("simulation_validation", {})
    robot_link_dynamics = _validated_link_dynamics(
        validation_cfg.get("robot_link_dynamics", [])
    )
    if robot_model_id == "fanuc_m710id_70":
        expected_links = {"base_link", *(f"J{index}_link" for index in range(1, 7))}
        actual_links = {item["link"] for item in robot_link_dynamics}
        if actual_links != expected_links:
            raise ValueError(
                "M-710 engineering replay requires mass properties for base_link and J1_link..J6_link"
            )
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
    if robot_model_id == "fanuc_m710id_70" and holding_torque is None:
        raise ValueError("M-710 engineering replay requires a finite vacuum holding torque")
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
        solver_attachment_cup_counts = [active_cup_count]
    elif solver_attachment_model == "equivalent_zone_row_band_centers":
        # Six non-collinear solver points retain the known 3-zone x 2-row-band
        # spatial support without creating 72 nearly coincident rigid joints.
        # This is a numerical attachment representation; physical force and
        # moment qualification continues to use the full sealed-cup evidence.
        columns_per_zone = cup_columns // zone_count
        row_split = cup_rows // 2
        grouped_centers: list[np.ndarray] = []
        solver_attachment_cup_counts = []
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
                    solver_attachment_cup_counts.append(len(group_indices))
        if len(grouped_centers) < 3:
            raise ValueError("zone-row-band solver model requires at least three populated groups")
        solver_attachment_offsets = np.asarray(grouped_centers, dtype=float)
    else:
        solver_attachment_offsets = active_cup_centers
        solver_attachment_cup_counts = [1] * active_cup_count
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
    if robot_model_id == "fanuc_m20id_35" and not np.isclose(
        max_grip_distance, cup_compression + capture_tolerance
    ):
        raise ValueError("legacy surface gripper distance must equal cup compression plus solver tolerance")
    max_normal_misalignment = float(
        validation_cfg.get("vacuum_max_normal_misalignment_rad", np.deg2rad(5.0))
    )
    maximum_contact_penetration = float(
        validation_cfg.get("vacuum_maximum_contact_penetration_m", 1e-5)
    )
    if (
        not np.isfinite(max_normal_misalignment)
        or not 0.0 <= max_normal_misalignment < 0.5 * np.pi
        or not np.isfinite(maximum_contact_penetration)
        or maximum_contact_penetration < 0.0
    ):
        raise ValueError("vacuum contact angle and penetration tolerances are invalid")
    tool_cfg = cfg.get("tool", {})
    geometry_cfg = tool_cfg.get("geometry", {}) if isinstance(tool_cfg, dict) else {}
    declared_tool_mass_policy = str(tool_cfg.get("fixed_mass_accounting_policy", ""))
    declared_tool_attachment_link = str(tool_cfg.get("attach_to_link", ""))
    independent_tool_rigid_body_required = tool_cfg.get(
        "independent_tool_rigid_body_required"
    )
    tool_frame_contract = tool_cfg.get("frame_contract")
    validation_frame_contract = validation_cfg.get("vacuum_frame_contract")
    if robot_model_id == "fanuc_m710id_70":
        if (
            declared_tool_mass_policy
            != "FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE"
            or declared_tool_attachment_link != "J6_link"
            or independent_tool_rigid_body_required is not False
        ):
            raise ValueError(
                "M-710 tool topology must declare one fixed mass contribution combined into J6"
            )
        if (
            not isinstance(tool_frame_contract, dict)
            or tool_frame_contract != validation_frame_contract
        ):
            raise ValueError("M-710 tool frame contract must be explicit and consistent")
    gripper_mass = float(tool_cfg.get("mass_kg", validation_cfg.get("vacuum_gripper_mass_kg", 0.0)))
    gripper_com = np.asarray(
        tool_cfg.get("com_xyz_m", validation_cfg.get("vacuum_center_of_mass_from_flange_m", [])), dtype=float
    )
    gripper_inertia = np.asarray(
        tool_cfg.get("inertia_tensor_com_kg_m2", validation_cfg.get("vacuum_inertia_at_com_kg_m2", [])), dtype=float
    )
    gripper_inertia_eigenvalues = (
        np.linalg.eigvalsh(gripper_inertia)
        if gripper_inertia.shape == (3, 3)
        else np.asarray([])
    )
    flange_origin_step = np.asarray(
        geometry_cfg.get("flange_origin_step_mm", validation_cfg.get("vacuum_flange_origin_step_mm", [])), dtype=float
    )
    step_from_tool_rotation = np.asarray(
        geometry_cfg.get("step_from_tool_rotation_matrix", validation_cfg.get("vacuum_step_from_tool_rotation_matrix", [])), dtype=float
    )
    if (
        not np.isfinite(gripper_mass)
        or gripper_mass <= 0.0
        or gripper_com.shape != (3,)
        or not np.all(np.isfinite(gripper_com))
        or gripper_inertia.shape != (3, 3)
        or not np.all(np.isfinite(gripper_inertia))
        or not np.allclose(gripper_inertia, gripper_inertia.T, atol=1e-10, rtol=0.0)
        or gripper_inertia_eigenvalues.shape != (3,)
        or np.any(gripper_inertia_eigenvalues <= 0.0)
        or 2.0 * np.max(gripper_inertia_eigenvalues)
        > np.sum(gripper_inertia_eigenvalues) + 1e-10
        or flange_origin_step.shape != (3,)
        or not np.all(np.isfinite(flange_origin_step))
        or step_from_tool_rotation.shape != (3, 3)
        or not np.all(np.isfinite(step_from_tool_rotation))
        or not np.allclose(step_from_tool_rotation.T @ step_from_tool_rotation, np.eye(3))
        or not np.isclose(np.linalg.det(step_from_tool_rotation), 1.0)
    ):
        raise ValueError("gripper mass properties and STEP-to-tool transform are required")
    task_tcp_from_flange = float(
        validation_cfg.get(
            "vacuum_task_tcp_from_flange_m",
            tool_cfg.get(
                "planner_tool_length_m",
                robot.get("tool_length", 0.20 if robot_model_id == "fanuc_m20id_35" else 0.0),
            ),
        )
    )
    physical_face_from_flange = float(
        validation_cfg.get(
            "vacuum_physical_uncompressed_face_from_flange_m",
            task_tcp_from_flange,
        )
    )
    compressed_contact_from_flange = float(
        validation_cfg.get(
            "vacuum_compressed_contact_plane_from_flange_m",
            physical_face_from_flange - cup_compression,
        )
    )
    if (
        not np.isfinite(task_tcp_from_flange)
        or not np.isfinite(physical_face_from_flange)
        or not np.isfinite(compressed_contact_from_flange)
        or min(task_tcp_from_flange, physical_face_from_flange, compressed_contact_from_flange) <= 0.0
    ):
        raise ValueError("task TCP and physical suction contact planes must be finite and positive")
    if robot_model_id == "fanuc_m710id_70" and np.isclose(
        task_tcp_from_flange, physical_face_from_flange, atol=1e-12, rtol=0.0
    ):
        raise ValueError("M-710 replay must distinguish the virtual task TCP from the physical cup face")
    if robot_model_id == "fanuc_m710id_70" and not (
        task_tcp_from_flange > physical_face_from_flange > compressed_contact_from_flange
    ):
        raise ValueError(
            "M-710 task TCP, uncompressed cup face, and compressed contact plane are out of order"
        )
    if robot_model_id == "fanuc_m710id_70" and not np.isclose(
        compressed_contact_from_flange,
        physical_face_from_flange - cup_compression,
        atol=1e-12,
        rtol=0.0,
    ):
        raise ValueError("M-710 compressed contact plane must equal cup face minus compression")

    grasp_body_path_suffix = str(
        robot.get(
            "isaac_grasp_body_path_suffix",
            "Geometry/base_link/J1_link/J2_link/J3_link/J4_link/J5_link/J6_link",
        )
    ).strip("/")
    flange_offset_from_grasp_body = np.asarray(
        robot.get("flange_offset_from_grasp_body_m", [0.09, 0.0, 0.0]),
        dtype=float,
    )
    if (
        not grasp_body_path_suffix
        or flange_offset_from_grasp_body.shape != (3,)
        or not np.all(np.isfinite(flange_offset_from_grasp_body))
    ):
        raise ValueError("Isaac grasp body and finite flange offset are required")

    mass_accounting: dict[str, Any] | None = None
    if robot_model_id == "fanuc_m710id_70":
        grasp_body_link = grasp_body_path_suffix.rsplit("/", 1)[-1]
        if grasp_body_link != "J6_link":
            raise ValueError("M-710 fixed-tool mass accounting requires J6_link as the grasp body")
        original_robot_mass = float(sum(item["mass_kg"] for item in robot_link_dynamics))
        grasp_record = next(item for item in robot_link_dynamics if item["link"] == grasp_body_link)
        tool_com_in_grasp_body = flange_offset_from_grasp_body + gripper_com
        combined_mass, combined_com, combined_inertia = _combine_fixed_mass_properties(
            grasp_record["mass_kg"],
            np.asarray(grasp_record["com_xyz_m"], dtype=float),
            np.asarray(grasp_record["inertia_at_com_kg_m2"], dtype=float),
            gripper_mass,
            tool_com_in_grasp_body,
            gripper_inertia,
        )
        applied_link_dynamics: list[dict[str, Any]] = []
        for item in robot_link_dynamics:
            applied = dict(item)
            if item["link"] == grasp_body_link:
                applied.update(
                    mass_kg=combined_mass,
                    com_xyz_m=combined_com.tolist(),
                    inertia_at_com_kg_m2=combined_inertia.tolist(),
                    source_status=(
                        f"{item['source_status']}; FIXED_TOOL_MASS_COMBINED_EXACTLY_ONCE"
                    ),
                )
            applied_link_dynamics.append(applied)
        robot_link_dynamics = applied_link_dynamics
        applied_mass = float(sum(item["mass_kg"] for item in robot_link_dynamics))
        if not np.isclose(applied_mass, original_robot_mass + gripper_mass, atol=1e-10, rtol=0.0):
            raise ValueError("M-710 robot/tool mass accounting is inconsistent")
        mass_accounting = {
            "policy": "FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE",
            "declared_policy": declared_tool_mass_policy,
            "applied_policy": "FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE",
            "grasp_body_link": grasp_body_link,
            "declared_attachment_link": declared_tool_attachment_link,
            "independent_tool_rigid_body_created": False,
            "source_robot_mass_kg": original_robot_mass,
            "tool_mass_kg": gripper_mass,
            "applied_articulation_mass_kg": applied_mass,
            "tool_com_in_grasp_body_m": tool_com_in_grasp_body.tolist(),
        }
    physics_contract = dict(validation_cfg.get("physics", {}))
    rendering_contract = dict(validation_cfg.get("rendering", {}))
    required_post_release_settle_seconds = execution.get("post_release_settle_seconds")
    if robot_model_id == "fanuc_m710id_70":
        physics_contract = _validated_physics_contract(
            physics_contract,
            solver_position_iterations=solver_position_iterations,
            solver_velocity_iterations=solver_velocity_iterations,
        )
        rendering_contract = _validated_rendering_contract(
            rendering_contract,
            physics_time_step_s=physics_contract["physics_time_step_s"],
        )
        if required_post_release_settle_seconds is None:
            raise ValueError("M-710 replay requires a post-release settling duration")
        required_post_release_settle_seconds = float(required_post_release_settle_seconds)
        if (
            not np.isfinite(required_post_release_settle_seconds)
            or required_post_release_settle_seconds < 0.0
        ):
            raise ValueError("M-710 post-release settling duration must be finite and non-negative")

    base_position = np.asarray(robot.get("base_position", [0.0, 0.0, 0.0]), dtype=float)
    dock_value = segment.get("amr_dock_position")
    robot_mount = cfg.get("amr", {}).get("robot_mount_position")
    if dock_value is not None and robot_mount is not None:
        base_position = np.asarray(dock_value, dtype=float) + np.asarray(robot_mount, dtype=float)

    conveyor_contract = dict(cfg.get("simulation_validation", {}).get("conveyor", {}))
    if (
        robot_model_id == "fanuc_m710id_70"
        and conveyor_contract.get("enabled") is True
        and conveyor_contract.get("start_policy") == "after_release_retreat"
        and source_release_retreat_time is None
    ):
        raise ValueError("M-710 after_release_retreat conveyor requires release_retreat_index")

    metadata = {
        "robot_model": robot_model_id,
        "source_plan_sha256": _canonical_digest(plan),
        "merged_configuration_sha256": _canonical_digest(cfg),
        "joint_names": list(FANUC_JOINT_NAMES),
        "urdf_path": str(robot["urdf_path"]),
        "base_position_m": base_position.tolist(),
        "base_rpy_rad": list(robot.get("base_rpy", [0.0, 0.0, 0.0])),
        "tool_length_m": task_tcp_from_flange,
        "isaac_grasp_body_path_suffix": grasp_body_path_suffix,
        "flange_offset_from_grasp_body_m": flange_offset_from_grasp_body.tolist(),
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
        "joint_drive_stiffness_nm_rad": stiffness.tolist(),
        "joint_drive_damping_nm_s_rad": damping.tolist(),
        "execution_asset_fingerprint_sha256": plan.get("execution_asset_fingerprint_sha256"),
        "execution_qualified": execution_qualified,
        "simulation_execution_ready": simulation_execution_ready,
        "execution_blockers": execution_blockers,
        "machine_qualified": machine_qualified,
        "machine_qualification_warnings": list(machine_qualification_warnings),
        "robot_link_dynamics": robot_link_dynamics,
        "robot_tool_mass_accounting": mass_accounting,
        "physics": physics_contract,
        "required_post_release_settle_seconds": required_post_release_settle_seconds,
        "grasp_arrival_time_seconds": grasp_arrival_time,
        "pre_grasp_controller_settle_seconds": pre_grasp_settle_seconds,
        "grasp_time_seconds": grasp_time,
        "vacuum_establish_seconds": vacuum_establish_seconds,
        "release_arrival_time_seconds": release_arrival_time,
        "release_time_seconds": release_time,
        "release_hold_seconds": release_seconds,
        "loaded_motion_time_scale": loaded_motion_time_scale,
        "post_release_motion_limit_scale": post_release_motion_limit_scale,
        "release_retreat_time_seconds": release_retreat_time,
        "place_center_m": list(segment.get("place_center", [])),
        "release_center_m": list(segment.get("release_center", [])),
        "place_surface": segment.get("place_surface"),
        "free_fall_height_m": float(segment.get("free_fall_height_m", 0.0)),
        "collision_geometry": str(plan.get("collision_geometry", "urdf_collision_mesh")),
        "scene_primitives": _build_scene_primitives(plan, cfg, segment_index),
        "camera": dict(cfg.get("simulation_validation", {}).get("camera", {})),
        "rendering": rendering_contract,
        "conveyor": conveyor_contract,
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
            "max_normal_misalignment_rad": max_normal_misalignment,
            "maximum_contact_penetration_m": maximum_contact_penetration,
            "physical_cup_compression_m": cup_compression,
            "task_tcp_from_flange_m": task_tcp_from_flange,
            "physical_uncompressed_face_from_flange_m": physical_face_from_flange,
            "compressed_contact_plane_from_flange_m": compressed_contact_from_flange,
            "frame_contract": copy.deepcopy(tool_frame_contract),
            "virtual_tcp_beyond_uncompressed_face_m": task_tcp_from_flange - physical_face_from_flange,
            "surface_gripper_capture_tolerance_m": capture_tolerance,
            "attachment_model": attachment_model,
            "attachment_point_count": active_cup_count,
            "solver_attachment_model": solver_attachment_model,
            "solver_attachment_offsets_tool_yz_m": solver_attachment_offsets.tolist(),
            "solver_attachment_cup_counts": solver_attachment_cup_counts,
            "simulation_attachment_point_count": int(len(solver_attachment_offsets)),
            "solver_position_iterations": solver_position_iterations,
            "solver_velocity_iterations": solver_velocity_iterations,
            "gripper_mass_kg": gripper_mass,
            "center_of_mass_from_flange_m": gripper_com.tolist(),
            "inertia_at_com_kg_m2": gripper_inertia.tolist(),
            "flange_origin_step_mm": flange_origin_step.tolist(),
            "step_from_tool_rotation_matrix": step_from_tool_rotation.tolist(),
            "outer_size_m": list(geometry_cfg.get("outer_size_m", validation_cfg.get("vacuum_outer_size_m", []))),
            "step_path": geometry_cfg.get("step_path", validation_cfg.get("vacuum_step_path")),
            "visual_mesh_path": geometry_cfg.get("visual_mesh_path", validation_cfg.get("vacuum_visual_mesh_path")),
            "collision_mesh_path": geometry_cfg.get("collision_mesh_path", validation_cfg.get("vacuum_collision_mesh_path")),
            "mass_properties_path": geometry_cfg.get("mass_properties_path", validation_cfg.get("vacuum_mass_properties_path")),
            "flange_transform_confidence": geometry_cfg.get(
                "flange_transform_confidence", validation_cfg.get("vacuum_flange_transform_confidence")
            ),
            "zone_count": zone_count,
            "zone_assignment_confirmed": False,
            "model_source": "step_geometry_with_per_cup_forces_and_provisional_zone_mapping",
        },
    }
    if robot_model_id == "fanuc_m710id_70":
        metadata["m710_execution_preflight"] = copy.deepcopy(preflight)
        metadata["m710_replay_contract"] = m710_replay_contract
    return IsaacReplayBundle(command_times, commands, metadata)
