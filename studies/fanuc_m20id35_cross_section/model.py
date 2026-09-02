"""Deterministic model utilities for the independent cross-section study."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import itertools
import json
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from unloading_sim.robot_load import load_robot_limits


STATE_A = 0
STATE_B = 1
STATE_C = 2
STATE_LABELS = {STATE_A: "A_unreachable", STATE_B: "B_derated", STATE_C: "C_normal"}


def inclusive_grid(lower: float, upper: float, step: float) -> np.ndarray:
    """Return an endpoint-inclusive decimal grid without cumulative drift."""
    if not np.isfinite([lower, upper, step]).all() or step <= 0.0 or upper < lower:
        raise ValueError("invalid inclusive grid bounds")
    count = int(round((upper - lower) / step))
    values = lower + step * np.arange(count + 1, dtype=float)
    if not np.isclose(values[-1], upper, atol=1e-10):
        raise ValueError("range is not an integer multiple of the grid step")
    values[-1] = upper
    return values


def stable_seed(root_seed: int, *parts: object) -> int:
    payload = "|".join([str(root_seed), *(str(part) for part in parts)]).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "little")


def load_config(path: str | Path) -> tuple[dict, Path]:
    path = Path(path).resolve()
    cfg = json.loads(path.read_text(encoding="utf-8"))
    if cfg.get("schema_version") != "fanuc_cross_section_study_v1":
        raise ValueError("unsupported cross-section study schema")
    wrist_config = cfg.get("wrist_limits", {}).get("limits_config")
    if wrist_config:
        limits = load_robot_limits(resolve_project_path(path, wrist_config))
        cfg["wrist_limits"] = {
            "limits_config": wrist_config,
            "source": limits.source["title"],
            "source_url": limits.source["url"],
            "maximum_external_mass_kg": limits.rated_payload_kg,
            "joint_moment_limits_nm": dict(zip(("J4", "J5", "J6"), limits.allowable_moment_nm)),
            "joint_inertia_limits_kg_m2": dict(zip(("J4", "J5", "J6"), limits.allowable_inertia_kg_m2)),
            "normal_utilization_limit": limits.normal_utilization_limit,
            "payload_com_limit_status": limits.com_limit_status,
        }
    return cfg, path


def resolve_project_path(config_path: Path, value: str) -> Path:
    """Resolve repository-relative paths without depending on the process CWD."""
    for parent in [config_path.parent, *config_path.parents]:
        candidate = (parent / value).resolve()
        if candidate.exists() or (parent / "pyproject.toml").exists():
            return candidate
    return Path(value).resolve()


def rotation_about_local_z(rotation: np.ndarray, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    roll = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.asarray(rotation, dtype=float) @ roll


def target_rotations(face: str, lateral_x: float) -> list[np.ndarray]:
    """Return four roll variants while keeping the suction normal exact."""
    if face == "front":
        base = np.column_stack(([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]))
    elif face == "side":
        side = 1.0 if lateral_x >= 0.0 else -1.0
        # Use the side facing the trailer centreline. Tool +Z enters the carton.
        base = np.column_stack(([-side, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, side, 0.0]))
    else:
        raise ValueError(f"unsupported face {face!r}")
    if np.linalg.det(base) < 0.0:
        base[:, 0] *= -1.0
    return [rotation_about_local_z(base, angle) for angle in (0.0, np.pi / 2, np.pi, -np.pi / 2)]


def target_pose(
    cfg: dict,
    lateral_x: float,
    z: float,
    size: Sequence[float] | None,
    face: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return contact position, box centre and outward face normal in world axes."""
    front_x = float(cfg["trailer"]["target_front_face_world_x_m"])
    if size is None:
        box_center = np.array([front_x, lateral_x, z], dtype=float)
        return box_center.copy(), box_center, np.array([-1.0, 0.0, 0.0])
    width, _height, depth = np.asarray(size, dtype=float)
    box_center = np.array([front_x + 0.5 * depth, lateral_x, z], dtype=float)
    if face == "front":
        return np.array([front_x, lateral_x, z]), box_center, np.array([-1.0, 0.0, 0.0])
    side = 1.0 if lateral_x >= 0.0 else -1.0
    contact = box_center.copy()
    contact[1] -= side * 0.5 * width
    return contact, box_center, np.array([0.0, -side, 0.0])


def carton_inside_trailer(cfg: dict, lateral_x: float, z: float, size: Sequence[float]) -> bool:
    width, height, _depth = np.asarray(size, dtype=float)
    half_width = 0.5 * float(cfg["trailer"]["inside_width_m"])
    ceiling = float(cfg["trailer"]["inside_height_m"])
    eps = 1e-10
    return bool(
        lateral_x - 0.5 * width >= -half_width - eps
        and lateral_x + 0.5 * width <= half_width + eps
        and z - 0.5 * height >= -eps
        and z + 0.5 * height <= ceiling + eps
    )


def neighboring_carton_centers(
    cfg: dict, box_center: np.ndarray, size: Sequence[float]
) -> list[np.ndarray]:
    width, height, _depth = np.asarray(size, dtype=float)
    gap = float(cfg["cartons"]["neighbor_gap_m"])
    candidates = []
    # The target is the currently removable top carton in its support column:
    # retain both side neighbours, its support below and the next depth row,
    # but do not fabricate a carton resting on top of the removal candidate.
    width_depth_height_offsets = (
        (0.0, -(width + gap), 0.0),
        (0.0, width + gap, 0.0),
        (0.0, 0.0, -(height + gap)),
        (float(size[2]) + gap, 0.0, 0.0),
    )
    for delta_x, delta_y, delta_z in width_depth_height_offsets:
        center = np.asarray(box_center, dtype=float) + [delta_x, delta_y, delta_z]
        if carton_inside_trailer(cfg, float(center[1]), float(center[2]), size):
            candidates.append(center)
    return candidates


def load_gripper_model(cfg: dict, config_path: Path) -> tuple[list[np.ndarray], dict]:
    path = resolve_project_path(config_path, cfg["gripper"]["mass_properties"])
    data = json.loads(path.read_text(encoding="utf-8"))
    rigid_bounds = data.get("rigid_collision_bounding_boxes_step_mm", [])
    if len(rigid_bounds) < 3:
        raise ValueError("gripper audit has no per-solid rigid collision model")
    flange_step = np.asarray(data["flange_origin_step_mm"], dtype=float)
    step_from_tool = np.asarray(data["rotation_step_from_tool"], dtype=float)
    tool_length = float(cfg["gripper"]["requested_axial_length_m"])
    boxes: list[np.ndarray] = []
    for bounds in rigid_bounds:
        lower, upper = np.asarray(bounds[:3]), np.asarray(bounds[3:])
        corners = np.asarray(list(itertools.product(*zip(lower, upper))), dtype=float)
        cad_tool = (corners - flange_step) @ step_from_tool * 1e-3
        # CAD tool [normal,length,width] -> planner [width,length,normal].
        planner = cad_tool[:, [2, 1, 0]]
        planner[:, 2] -= tool_length
        box_min, box_max = np.min(planner, axis=0), np.max(planner, axis=0)
        boxes.append(np.concatenate((0.5 * (box_min + box_max), box_max - box_min)))
    return boxes, data


def transform_matrix(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation, dtype=float)
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return points @ transform[:3, :3].T + transform[:3, 3]


def box_inertia(mass: float, dimensions: Sequence[float]) -> np.ndarray:
    x, y, z = np.asarray(dimensions, dtype=float)
    return np.diag(
        [
            mass * (y * y + z * z) / 12.0,
            mass * (x * x + z * z) / 12.0,
            mass * (x * x + y * y) / 12.0,
        ]
    )


@dataclass(frozen=True)
class WristLoadResult:
    passed: bool
    derated: bool
    maximum_utilization: float
    moments_nm: tuple[float, float, float]
    inertias_kg_m2: tuple[float, float, float]
    reason: str


def wrist_load_check(
    cfg: dict,
    robot,
    q: np.ndarray,
    working_pose: np.ndarray,
    mass_properties: dict,
    carton_mass: float,
    carton_size: Sequence[float],
    face: str,
) -> WristLoadResult:
    """Check mass, gravity moment and inertia about the actual J4/J5/J6 axes."""
    limits = cfg["wrist_limits"]
    gripper_mass = float(cfg["gripper"]["mass_kg"])
    total_mass = gripper_mass + float(carton_mass)
    if total_mass > float(limits["maximum_external_mass_kg"]) + 1e-9:
        return WristLoadResult(False, False, total_mass / limits["maximum_external_mass_kg"], (0, 0, 0), (0, 0, 0), "payload_mass")

    local_working_pose = robot.fk(q)
    base_translation = working_pose[:3, 3] - local_working_pose[:3, 3]
    flange = robot.named_link_frames(q)["flange"].copy()
    flange[:3, 3] += base_translation
    rotation = working_pose[:3, :3]
    gripper_com_cad = np.asarray(mass_properties["center_of_mass_from_flange_tool_m"], dtype=float)
    gripper_com_local = gripper_com_cad[[2, 1, 0]]
    gripper_com_world = flange[:3, 3] + rotation @ gripper_com_local
    gripper_inertia_cad = np.asarray(mass_properties["inertia_at_com_tool_kg_m2"], dtype=float)
    permutation = np.eye(3)[[2, 1, 0]]
    gripper_inertia_local = permutation @ gripper_inertia_cad @ permutation.T

    width, height, depth = np.asarray(carton_size, dtype=float)
    if face == "front":
        carton_dimensions_tool = np.array([width, height, depth])
        normal_depth = depth
    else:
        carton_dimensions_tool = np.array([depth, height, width])
        normal_depth = width
    carton_com_world = working_pose[:3, 3] + rotation[:, 2] * (0.5 * normal_depth)
    carton_inertia_local = box_inertia(float(carton_mass), carton_dimensions_tool)
    gripper_inertia_world = rotation @ gripper_inertia_local @ rotation.T
    carton_inertia_world = rotation @ carton_inertia_local @ rotation.T
    _frames, origins, axes = robot._frames_and_axes(q)  # audited URDF axis locations
    force_gripper = np.array([0.0, 0.0, -9.80665 * gripper_mass])
    force_carton = np.array([0.0, 0.0, -9.80665 * float(carton_mass)])
    moment_limits = limits["joint_moment_limits_nm"]
    inertia_limits = limits["joint_inertia_limits_kg_m2"]
    moments: list[float] = []
    inertias: list[float] = []
    utilizations: list[float] = [total_mass / float(limits["maximum_external_mass_kg"])]
    for index, name in zip((3, 4, 5), ("J4", "J5", "J6")):
        origin, axis = origins[index] + base_translation, axes[index]
        torque = np.cross(gripper_com_world - origin, force_gripper) + np.cross(
            carton_com_world - origin, force_carton
        )
        moment = abs(float(axis @ torque))
        inertia = 0.0
        for body_mass, body_com, body_inertia in (
            (gripper_mass, gripper_com_world, gripper_inertia_world),
            (float(carton_mass), carton_com_world, carton_inertia_world),
        ):
            offset = body_com - origin
            perpendicular_sq = float(offset @ offset - (axis @ offset) ** 2)
            inertia += float(axis @ body_inertia @ axis) + body_mass * max(0.0, perpendicular_sq)
        moments.append(moment)
        inertias.append(inertia)
        utilizations.extend([moment / float(moment_limits[name]), inertia / float(inertia_limits[name])])
    maximum = max(utilizations)
    if maximum > 1.0 + 1e-9:
        reason = "wrist_moment" if max(m / moment_limits[n] for m, n in zip(moments, ("J4", "J5", "J6"))) >= max(i / inertia_limits[n] for i, n in zip(inertias, ("J4", "J5", "J6"))) else "wrist_inertia"
        return WristLoadResult(False, False, maximum, tuple(moments), tuple(inertias), reason)
    derated = maximum > float(limits["normal_utilization_limit"])
    return WristLoadResult(True, derated, maximum, tuple(moments), tuple(inertias), "derated_load" if derated else "ok")


def aggregate_cell_states(states: Sequence[int]) -> int:
    """C means every required scenario is normal; B means partial/derated coverage."""
    values = list(states)
    if not values or all(value == STATE_A for value in values):
        return STATE_A
    if all(value == STATE_C for value in values):
        return STATE_C
    return STATE_B
