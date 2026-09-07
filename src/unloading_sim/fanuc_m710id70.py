"""Traceable first-layer studies for the FANUC M-710iD/70.

The module deliberately separates what the supplied FANUC evidence proves
from engineering screens.  It provides kinematics, swept primitive collision,
carried-carton geometry, time parameterisation and wrist external-load
calculations.  It does not invent missing link inertials or drive torques.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

from .geometry import OBB, make_transform
from .depalletizing import (
    FACE_OUTWARD_NORMALS,
    analyze_box_neighborhood,
    minimum_clearance_extraction_distance,
)
from .identity import ToolConfig, load_tool_config
from .ik import IKResult, solve_ik_multistart
from .robot import URDFRobot
from .robot_load import (
    PoseSpecificLoadCase,
    box_spatial_load,
    load_robot_limits,
    load_tool,
    qualify_load_v2,
)


MODEL_ID = "fanuc_m710id_70"
ACTIVE_JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
OFFICIAL_REACH_M = 2.104
GRAVITY_M_S2 = 9.80665
HOME_JOINTS_RAD = np.array([-0.17301878, -1.13655578, -0.74874837, 0.63151726, -2.12393931, -2.83276516])


@dataclass(frozen=True)
class TaskPointResult:
    geometric_reachable: bool
    ik_reachable: bool
    collision_free_reachable: bool
    extraction_reachable: bool
    task_reachable: bool
    failure_reason: str
    grasp_mode: str
    roll_degrees: int
    q_pregrasp: np.ndarray | None
    q_contact: np.ndarray | None
    path: tuple[np.ndarray, ...]
    minimum_joint_margin_rad: float | None
    maximum_jacobian_condition: float | None
    minimum_clearance_extraction_distance_m: float | None = None
    tcp_path_length_m: float | None = None
    joint_space_path_length_rad: float | None = None

    def to_mapping(self) -> dict:
        result = asdict(self)
        for key in ("q_pregrasp", "q_contact"):
            value = result[key]
            result[key] = None if value is None else np.asarray(value).tolist()
        result["path"] = [np.asarray(value).tolist() for value in self.path]
        return result


def build_robot(
    base_x_m: float,
    *,
    base_y_m: float = 0.0,
    mounting_surface_z_m: float = 0.60,
    tool_config: str | Path = "configs/tools/unloading_gripper_20kg.yaml",
    urdf_path: str | Path = "assets/robots/fanuc_m710id_70/m710id_70.urdf",
) -> tuple[URDFRobot, ToolConfig]:
    tool = load_tool_config(tool_config)
    geometry = tool.data.get("geometry", {})
    outer = geometry.get("outer_size_m", [0.576, 0.288, tool.tcp_translation_xyz_m[0]])
    robot = URDFRobot.fanuc_m710id_70(
        urdf_path=urdf_path,
        base_position=[float(base_x_m), float(base_y_m), float(mounting_surface_z_m)],
        tool_length=float(tool.tcp_translation_xyz_m[0]),
        tool_collision_size=outer,
        tool_collision_center_offset=0.5 * float(outer[2]),
    )
    return robot, tool


def official_reach_guard(robot: URDFRobot, q: np.ndarray, tolerance_m: float = 0.002) -> bool:
    """Apply the published radial work-envelope boundary to the CAD-axis chain."""
    flange = robot.named_link_frames(np.asarray(q, dtype=float))["flange"][:3, 3]
    base = robot.base_transform[:3, 3]
    radial = float(np.linalg.norm((flange - base)[:2]))
    return radial <= OFFICIAL_REACH_M + tolerance_m


def validate_model(robot: URDFRobot) -> dict:
    zero = np.zeros(6)
    flange = robot.named_link_frames(zero)["flange"]
    axes = robot.joint_axis_frames(zero)
    expected_origins = {
        "J1": [0.0, 0.0, 0.0],
        "J2": [0.150, 0.0, 0.565],
        "J3": [0.150, 0.0, 1.460],
        "J4": [0.150, 0.0, 1.630],
        "J5": [1.195, 0.0, 1.630],
        "J6": [1.195, 0.0, 1.630],
    }
    expected_axes = {
        "J1": [0.0, 0.0, 1.0],
        "J2": [0.0, 1.0, 0.0],
        "J3": [0.0, -1.0, 0.0],
        "J4": [-1.0, 0.0, 0.0],
        "J5": [0.0, -1.0, 0.0],
        "J6": [-1.0, 0.0, 0.0],
    }
    base = robot.base_transform[:3, 3]
    origin_errors = {
        name: float(np.linalg.norm(origin - base - np.asarray(expected_origins[name])))
        for name, (origin, _axis) in axes.items()
    }
    axis_errors = {
        name: float(np.linalg.norm(axis - np.asarray(expected_axes[name])))
        for name, (_origin, axis) in axes.items()
    }
    jacobian = robot.geometric_jacobian(np.array([0.2, -0.4, 0.7, 0.3, -0.5, 0.2]))
    finite_difference = np.zeros_like(jacobian[:3])
    q = np.array([0.2, -0.4, 0.7, 0.3, -0.5, 0.2])
    eps = 1e-7
    for index in range(6):
        displaced = q.copy()
        displaced[index] += eps
        finite_difference[:, index] = (robot.fk(displaced)[:3, 3] - robot.fk(q)[:3, 3]) / eps
    jacobian_error = float(np.max(np.abs(finite_difference - jacobian[:3])))
    raw_chain_length = robot.max_reach
    checks = {
        "zero_flange_pose": bool(np.allclose(flange[:3, 3] - base, [1.370, 0.0, 1.630], atol=2e-6)),
        "joint_axis_origins": max(origin_errors.values()) <= 2e-6,
        "joint_axis_directions": max(axis_errors.values()) <= 2e-9,
        "joint_limits_finite_ordered": bool(np.all(np.isfinite(robot.joint_limits)) and np.all(robot.joint_limits[:, 0] < robot.joint_limits[:, 1])),
        "jacobian_translation_finite_difference": jacobian_error <= 2e-6,
        "zero_not_self_colliding": not robot.collision_result(zero, []).in_collision,
        "published_reach_guard_configured": True,
        "link_inertials_available": False,
        "joint_drive_torque_limits_available": False,
    }
    return {
        "model_id": MODEL_ID,
        "status": "PASS_KINEMATICS_BLOCKED_INVERSE_DYNAMICS" if all(value for key, value in checks.items() if key not in {"link_inertials_available", "joint_drive_torque_limits_available"}) else "FAIL",
        "checks": checks,
        "zero_pose_flange_xyz_from_mount_m": (flange[:3, 3] - base).tolist(),
        "joint_axis_origin_error_m": origin_errors,
        "joint_axis_direction_error": axis_errors,
        "jacobian_max_translation_error": jacobian_error,
        "raw_serial_chain_path_length_m": raw_chain_length,
        "official_radial_reach_guard_m": OFFICIAL_REACH_M,
        "raw_chain_note": "ROBCAD axis geometry can form poses outside the published operating envelope; all accepted task states apply the 2.104 m official radial guard.",
        "inverse_dynamics_status": "BLOCKED_BY_MISSING_INERTIAL_DATA",
    }


def trailer_obstacles(
    width_m: float = 2.30,
    height_m: float = 2.70,
    x_limits_m: Sequence[float] = (-1.50, 2.20),
    thickness_m: float = 0.05,
) -> list[OBB]:
    x0, x1 = (float(value) for value in x_limits_m)
    xc, xs = 0.5 * (x0 + x1), x1 - x0
    eye = np.eye(3)
    return [
        OBB(np.array([xc, -0.5 * width_m - 0.5 * thickness_m, 0.5 * height_m]), np.array([0.5 * xs, 0.5 * thickness_m, 0.5 * height_m + thickness_m]), eye, "right_wall"),
        OBB(np.array([xc, 0.5 * width_m + 0.5 * thickness_m, 0.5 * height_m]), np.array([0.5 * xs, 0.5 * thickness_m, 0.5 * height_m + thickness_m]), eye, "left_wall"),
        OBB(np.array([xc, 0.0, -0.5 * thickness_m]), np.array([0.5 * xs, 0.5 * width_m, 0.5 * thickness_m]), eye, "floor"),
        OBB(np.array([xc, 0.0, height_m + 0.5 * thickness_m]), np.array([0.5 * xs, 0.5 * width_m, 0.5 * thickness_m]), eye, "roof"),
    ]


def carton_inside_cross_section(y_m: float, z_m: float, size_xyz_m: Sequence[float], width_m: float = 2.30, height_m: float = 2.70) -> bool:
    _depth, width, height = np.asarray(size_xyz_m, dtype=float)
    return bool(
        y_m - 0.5 * width >= -0.5 * width_m - 1e-10
        and y_m + 0.5 * width <= 0.5 * width_m + 1e-10
        and z_m - 0.5 * height >= -1e-10
        and z_m + 0.5 * height <= height_m + 1e-10
    )


def _roll_about_local_z(rotation: np.ndarray, degrees: int) -> np.ndarray:
    angle = np.radians(float(degrees))
    c, s = np.cos(angle), np.sin(angle)
    return rotation @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def target_pose(front_face_x_m: float, y_m: float, z_m: float, size_xyz_m: Sequence[float], grasp_mode: str, roll_degrees: int = 0) -> tuple[np.ndarray, np.ndarray]:
    depth, _width, height = np.asarray(size_xyz_m, dtype=float)
    center = np.array([front_face_x_m + 0.5 * depth, y_m, z_m], dtype=float)
    if grasp_mode == "front":
        rotation = np.column_stack(([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]))
        contact = np.array([front_face_x_m, y_m, z_m], dtype=float)
    elif grasp_mode == "top":
        rotation = np.column_stack(([1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]))
        contact = center + [0.0, 0.0, 0.5 * height]
    elif grasp_mode == "left":
        rotation = np.column_stack(([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]))
        contact = center + [0.0, 0.5 * _width, 0.0]
    elif grasp_mode == "right":
        rotation = np.column_stack(([1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]))
        contact = center + [0.0, -0.5 * _width, 0.0]
    else:
        raise ValueError(f"unsupported grasp mode {grasp_mode!r}")
    return make_transform(_roll_about_local_z(rotation, roll_degrees), contact), center


def default_neighbor_obstacles(front_face_x_m: float, y_m: float, z_m: float, size_xyz_m: Sequence[float], gap_m: float = 0.01) -> list[OBB]:
    depth, width, height = np.asarray(size_xyz_m, dtype=float)
    center = np.array([front_face_x_m + 0.5 * depth, y_m, z_m])
    offsets = ([0.0, -(width + gap_m), 0.0], [0.0, width + gap_m, 0.0], [0.0, 0.0, -(height + gap_m)], [depth + gap_m, 0.0, 0.0])
    result = []
    for index, offset in enumerate(offsets):
        neighbor = center + offset
        if carton_inside_cross_section(float(neighbor[1]), float(neighbor[2]), size_xyz_m):
            result.append(OBB(neighbor, 0.5 * np.asarray(size_xyz_m), np.eye(3), f"neighbor_{index}", "carton"))
    return result


def _carried_box(pose_tcp: np.ndarray, size_xyz_m: Sequence[float], grasp_mode: str, name: str = "carried_box") -> OBB:
    depth, width, height = np.asarray(size_xyz_m, dtype=float)
    if grasp_mode == "front":
        local_center = np.array([0.0, 0.0, 0.5 * depth])
        local_size = np.array([width, height, depth])
    elif grasp_mode == "top":
        local_center = np.array([0.0, 0.0, 0.5 * height])
        local_size = np.array([depth, width, height])
    elif grasp_mode in {"left", "right"}:
        local_center = np.array([0.0, 0.0, 0.5 * width])
        local_size = np.array([depth, height, width])
    else:
        raise ValueError(f"unsupported grasp mode {grasp_mode!r}")
    return OBB(pose_tcp[:3, :3] @ local_center + pose_tcp[:3, 3], 0.5 * local_size, pose_tcp[:3, :3], name, "payload")


def _payload_collision_free(robot: URDFRobot, q: np.ndarray, box: OBB, obstacles: Sequence[OBB], margin_m: float) -> bool:
    if any(box.intersects_obb(obstacle, margin=margin_m) for obstacle in obstacles):
        return False
    capsules = robot.link_capsules(q)
    # The carried carton intentionally touches the tool/wrist. Check it against
    # base through J3; later links are inside the attachment exclusion zone.
    return not any(capsule.collides_obb(box, margin=margin_m) for capsule in capsules[:4])


def carried_payload_state_collision_free(
    robot: URDFRobot,
    q: np.ndarray,
    size_xyz_m: Sequence[float],
    grasp_mode: str,
    obstacles: Sequence[OBB],
    margin_m: float = 0.001,
) -> bool:
    """Check the complete rigid carried-carton sweep state at one joint pose."""
    pose = robot.fk(np.asarray(q, dtype=float))
    return _payload_collision_free(
        robot,
        np.asarray(q, dtype=float),
        _carried_box(pose, size_xyz_m, grasp_mode),
        obstacles,
        margin_m,
    )


def _joint_margin(robot: URDFRobot, q: np.ndarray) -> float:
    return float(np.min(np.minimum(q - robot.joint_limits[:, 0], robot.joint_limits[:, 1] - q)))


def evaluate_task_point(
    robot: URDFRobot,
    y_m: float,
    z_m: float,
    size_xyz_m: Sequence[float],
    *,
    grasp_mode: str,
    roll_degrees: int = 0,
    front_face_x_m: float = 1.0,
    seed_q: np.ndarray | None = None,
    rng_seed: int = 71070,
    obstacles: Sequence[OBB] | None = None,
    neighbor_obstacles: Sequence[OBB] | None = None,
    standoff_m: float = 0.15,
    extraction_scan_step_m: float = 0.01,
    extraction_free_clearance_m: float = 0.02,
    maximum_extraction_m: float = 1.50,
    margin_m: float = 0.001,
    max_iterations: int = 90,
    random_restarts: int = 0,
) -> TaskPointResult:
    empty = (False, False, False, False, False)
    if not carton_inside_cross_section(y_m, z_m, size_xyz_m):
        return TaskPointResult(*empty, "BOX_OUTSIDE_TRAILER", grasp_mode, roll_degrees, None, None, (), None, None)
    contact, _center = target_pose(front_face_x_m, y_m, z_m, size_xyz_m, grasp_mode, roll_degrees)
    normal = contact[:3, 2]
    pregrasp = contact.copy()
    pregrasp[:3, 3] -= normal * standoff_m
    flange_target = contact[:3, 3] - normal * robot.tool_length
    base = robot.base_transform[:3, 3]
    geometric = float(np.linalg.norm((flange_target - base)[:2])) <= OFFICIAL_REACH_M + 0.002
    if not geometric:
        return TaskPointResult(False, False, False, False, False, "OUTSIDE_OFFICIAL_RADIAL_REACH", grasp_mode, roll_degrees, None, None, (), None, None)

    home = HOME_JOINTS_RAD.copy() if seed_q is None else np.asarray(seed_q, dtype=float)
    rng = np.random.default_rng(int(rng_seed))
    scene_obstacles = list(trailer_obstacles() if obstacles is None else obstacles)
    neighbors = list(default_neighbor_obstacles(front_face_x_m, y_m, z_m, size_xyz_m) if neighbor_obstacles is None else neighbor_obstacles)
    all_obstacles = scene_obstacles + neighbors

    def solve(pose: np.ndarray, seeds: Sequence[np.ndarray], with_collision: bool) -> IKResult:
        return solve_ik_multistart(
            robot,
            pose,
            seeds=seeds,
            obstacles=all_obstacles if with_collision else (),
            random_restarts=random_restarts,
            rng=rng,
            max_iterations=max_iterations,
            damping=0.045,
            max_step=0.20,
            position_tolerance=0.012,
            orientation_tolerance=0.09,
            orientation_weight=0.55,
            collision_margin=margin_m,
            extra_state_valid=lambda q: official_reach_guard(robot, q),
        )

    # Try the collision-constrained solution first. A success proves both IK
    # and collision layers and avoids solving the same pose twice. Only a
    # constrained failure triggers an unconstrained diagnostic solve.
    pre = solve(pregrasp, [home], True)
    contact_result = solve(contact, [pre.q, home], True) if pre.success else pre
    if not pre.success or not contact_result.success:
        geometric_pre = solve(pregrasp, [home], False)
        geometric_contact = solve(contact, [geometric_pre.q, home], False) if geometric_pre.success else geometric_pre
        if not geometric_pre.success or not geometric_contact.success:
            return TaskPointResult(True, False, False, False, False, "NO_IK", grasp_mode, roll_degrees, geometric_pre.q if geometric_pre.success else None, geometric_contact.q if geometric_contact.success else None, (), None, None)
        return TaskPointResult(True, True, False, False, False, "ROBOT_COLLISION", grasp_mode, roll_degrees, pre.q if pre.success else None, contact_result.q if contact_result.success else None, (), None, None)

    path = [pre.q, contact_result.q]
    conditions: list[float] = []
    margins: list[float] = []
    for q in path:
        conditions.append(float(np.linalg.cond(robot.geometric_jacobian(q))))
        margins.append(_joint_margin(robot, q))
    if max(conditions) > 1.0e4 or min(margins) < np.radians(1.0):
        reason = "JOINT_LIMIT" if min(margins) < np.radians(1.0) else "SINGULARITY"
        return TaskPointResult(True, True, True, False, False, reason, grasp_mode, roll_degrees, pre.q, contact_result.q, tuple(path), min(margins), max(conditions))

    target = OBB(
        np.array([front_face_x_m + 0.5 * float(size_xyz_m[0]), y_m, z_m]),
        0.5 * np.asarray(size_xyz_m, dtype=float),
        np.eye(3),
        "target_box",
        "carton",
    )
    topology = analyze_box_neighborhood(target, neighbors)
    by_name = {box.name: box for box in neighbors}
    constraining_names = {
        topology.left_neighbor,
        topology.right_neighbor,
        topology.top_neighbor,
    }
    constraining_neighbors = [by_name[name] for name in constraining_names if name is not None]
    extraction_m = minimum_clearance_extraction_distance(
        target,
        FACE_OUTWARD_NORMALS[grasp_mode],
        constraining_neighbors,
        free_space_clearance_m=extraction_free_clearance_m,
        scan_step_m=extraction_scan_step_m,
        maximum_distance_m=maximum_extraction_m,
    )
    if extraction_m is None:
        return TaskPointResult(True, True, True, False, False, "EXTRACTION_FAILED", grasp_mode, roll_degrees, pre.q, contact_result.q, tuple(path), min(margins), max(conditions))

    endpoint_pose = contact.copy()
    endpoint_pose[:3, 3] += FACE_OUTWARD_NORMALS[grasp_mode] * extraction_m
    endpoint = solve(endpoint_pose, [contact_result.q, pre.q, home], True)
    if not endpoint.success:
        return TaskPointResult(True, True, True, False, False, "EXTRACTION_FAILED", grasp_mode, roll_degrees, pre.q, contact_result.q, tuple(path), min(margins), max(conditions), extraction_m)
    # Solve deterministic Cartesian samples along the required face normal.
    # Joint interpolation can bow the carried carton into a neighbour even
    # when both endpoint TCP poses lie on the requested straight extraction.
    for fraction in np.linspace(0.2, 1.0, 5):
        sample_pose = contact.copy()
        sample_pose[:3, 3] += FACE_OUTWARD_NORMALS[grasp_mode] * extraction_m * float(fraction)
        sample = endpoint if fraction >= 1.0 - 1e-12 else solve(sample_pose, [path[-1], endpoint.q, contact_result.q, home], True)
        if not sample.success:
            return TaskPointResult(True, True, True, False, False, "EXTRACTION_FAILED", grasp_mode, roll_degrees, pre.q, contact_result.q, tuple(path), min(margins), max(conditions), extraction_m)
        q = sample.q
        if not official_reach_guard(robot, q) or not robot.is_collision_free(q, all_obstacles, margin=margin_m):
            return TaskPointResult(True, True, True, False, False, "ROBOT_COLLISION", grasp_mode, roll_degrees, pre.q, contact_result.q, tuple(path), min(margins), max(conditions), extraction_m)
        payload = _carried_box(robot.fk(q), size_xyz_m, grasp_mode)
        if not _payload_collision_free(robot, q, payload, all_obstacles, margin_m):
            return TaskPointResult(True, True, True, False, False, "NEIGHBOR_BOX_COLLISION", grasp_mode, roll_degrees, pre.q, contact_result.q, tuple(path), min(margins), max(conditions), extraction_m)
        path.append(q)
        conditions.append(float(np.linalg.cond(robot.geometric_jacobian(q))))
        margins.append(_joint_margin(robot, q))
    task = bool(max(conditions) <= 1.0e4 and min(margins) >= np.radians(1.0))
    tcp_points = [robot.fk(q)[:3, 3] for q in path]
    tcp_length = float(sum(np.linalg.norm(b - a) for a, b in zip(tcp_points[:-1], tcp_points[1:])))
    joint_length = float(sum(np.linalg.norm(b - a) for a, b in zip(path[:-1], path[1:])))
    reason = "OK" if task else ("JOINT_LIMIT" if min(margins) < np.radians(1.0) else "SINGULARITY")
    return TaskPointResult(True, True, True, True, task, reason, grasp_mode, roll_degrees, pre.q, contact_result.q, tuple(path), min(margins), max(conditions), extraction_m, tcp_length, joint_length)


def combined_payload_properties(tool: ToolConfig, box_mass_kg: float, box_size_xyz_m: Sequence[float], com_offset_fraction_xyz: Sequence[float] = (0.0, 0.0, 0.0)) -> dict:
    depth, width, height = np.asarray(box_size_xyz_m, dtype=float)
    fractions = np.asarray(com_offset_fraction_xyz, dtype=float)
    if fractions.shape != (3,) or np.any(np.abs(fractions) > 0.10 + 1e-12):
        raise ValueError("box CoM offset fractions must be xyz values within +/-10%")
    box_com = np.array([tool.tcp_translation_xyz_m[0] + 0.5 * depth, 0.0, 0.0]) + fractions * np.array([depth, width, height])
    tool_com = np.asarray(tool.com_xyz_m)
    total = float(tool.mass_kg + box_mass_kg)
    combined = (tool.mass_kg * tool_com + box_mass_kg * box_com) / total
    # FANUC diagram Z is faceplate axial distance; X/Y are radial.
    vendor_axial = float(combined[0])
    vendor_radial = float(np.linalg.norm(combined[1:]))
    # Select the next-higher published payload curve. This is conservative
    # between the five discrete FANUC curves and exact for the 70 kg boundary.
    published = (
        (30.0, 0.520, 0.825),
        (40.0, 0.396, 0.615),
        (50.0, 0.317, 0.457),
        (60.0, 0.264, 0.352),
        (70.0, 0.225, 0.277),
    )
    selected = next((row for row in published if total <= row[0] + 1e-12), None)
    curve_mass, radial_limit, axial_limit = selected if selected is not None else (70.0, 0.225, 0.277)
    diagram_pass = bool(total <= 70.0 + 1e-12 and vendor_axial <= axial_limit + 1e-12 and vendor_radial <= radial_limit + 1e-12)
    return {
        "tool_mass_kg": tool.mass_kg,
        "box_mass_kg": float(box_mass_kg),
        "total_external_mass_kg": total,
        "payload_utilization": total / 70.0,
        "box_com_flange_xyz_m": box_com.tolist(),
        "combined_com_flange_xyz_m": combined.tolist(),
        "vendor_diagram_coordinates_m": {"radial_xy": vendor_radial, "axial_z": vendor_axial},
        "selected_curve_limits_m": {"radial_xy": radial_limit, "axial_z": axial_limit},
        "conservative_70kg_limits_m": {"radial_xy": 0.225, "axial_z": 0.277},
        "vendor_com_diagram_status": "PASS" if diagram_pass else "FAIL",
        "vendor_curve_selection": f"{curve_mass:g}kg_next_higher_published_curve_for_{total:g}kg",
        "box_com_offset_fraction_xyz": fractions.tolist(),
    }


def wrist_load_at_pose(
    robot: URDFRobot,
    q: np.ndarray,
    tool_config_path: str | Path,
    box_mass_kg: float = 42.5,
    box_size_xyz_m: Sequence[float] = (0.6, 0.4, 0.3),
    grasp_mode: str = "front",
    qd_rad_s: np.ndarray | None = None,
    qdd_rad_s2: np.ndarray | None = None,
) -> dict:
    """Screen payload-only wrist load, including trajectory acceleration.

    This is independent of whole-robot inverse dynamics. It uses rigid tool
    and carton properties with a finite-difference spatial-acceleration upper
    bound; missing robot-link inertials remain irrelevant to this screen.
    """
    limits = load_robot_limits("configs/robots/fanuc_m710id_70.yaml")
    tool = load_tool(tool_config_path)
    depth, width, height = np.asarray(box_size_xyz_m, dtype=float)
    load_size = (depth, width, height) if grasp_mode == "front" else (height, width, depth)
    q = np.asarray(q, dtype=float)
    result = qualify_load_v2(PoseSpecificLoadCase(limits, robot, tool, box_spatial_load(load_size, box_mass_kg, "front_center"), q))
    data = result.to_dict()
    intermediate = data["intermediate"]
    moments = [abs(float(intermediate[f"gravity_moment_{name.lower()}"])) for name in ("J4", "J5", "J6")]
    inertias = [float(intermediate[f"inertia_{name.lower()}"]) for name in ("J4", "J5", "J6")]
    qd = np.zeros(robot.dof) if qd_rad_s is None else np.asarray(qd_rad_s, dtype=float)
    qdd = np.zeros(robot.dof) if qdd_rad_s2 is None else np.asarray(qdd_rad_s2, dtype=float)
    if qd.shape != (robot.dof,) or qdd.shape != (robot.dof,):
        raise ValueError("qd_rad_s and qdd_rad_s2 must match the robot DOF")
    dt = 1e-4
    plus = robot.clamp(q + qd * dt + 0.5 * qdd * dt * dt)
    minus = robot.clamp(q - qd * dt + 0.5 * qdd * dt * dt)
    tcp = robot.fk(q)[:3, 3]
    linear_acceleration = (robot.fk(plus)[:3, 3] - 2.0 * tcp + robot.fk(minus)[:3, 3]) / (dt * dt)
    jacobian = robot.geometric_jacobian(q)
    jacobian_plus = robot.geometric_jacobian(plus)
    jacobian_minus = robot.geometric_jacobian(minus)
    omega = jacobian[3:] @ qd
    alpha = jacobian[3:] @ qdd + ((jacobian_plus[3:] - jacobian_minus[3:]) / (2.0 * dt)) @ qd
    combined_com = np.asarray(intermediate["combined_com_xyz"], dtype=float)
    flange = robot.named_link_frames(q)["flange"][:3, 3]
    radius_flange = float(np.linalg.norm(combined_com - flange))
    com_acceleration_bound = float(np.linalg.norm(linear_acceleration) + np.linalg.norm(alpha) * radius_flange + np.linalg.norm(omega) ** 2 * radius_flange)
    total_mass = float(intermediate["total_external_mass_kg"])
    dynamic_moments: list[float] = []
    joint_frames = robot.joint_axis_frames(q)
    for index, name in enumerate(("J4", "J5", "J6")):
        origin, axis = joint_frames[name]
        offset = combined_com - origin
        perpendicular = float(np.sqrt(max(0.0, offset @ offset - (axis @ offset) ** 2)))
        rotational_bound = inertias[index] * (float(np.linalg.norm(alpha)) + float(np.linalg.norm(omega)) ** 2)
        dynamic_moments.append(moments[index] + total_mass * com_acceleration_bound * perpendicular + rotational_bound)
    return {
        "q_rad": q.tolist(),
        "gravity_moment_nm_J4_J5_J6": moments,
        "dynamic_total_moment_nm_J4_J5_J6": dynamic_moments,
        "inertia_kg_m2_J4_J5_J6": inertias,
        "moment_utilization_J4_J5_J6": (np.asarray(moments) / [300.0, 300.0, 150.0]).tolist(),
        "dynamic_moment_utilization_J4_J5_J6": (np.asarray(dynamic_moments) / [300.0, 300.0, 150.0]).tolist(),
        "inertia_utilization_J4_J5_J6": (np.asarray(inertias) / [30.0, 30.0, 15.0]).tolist(),
        "tcp_linear_acceleration_m_s2": linear_acceleration.tolist(),
        "tcp_angular_velocity_rad_s": omega.tolist(),
        "tcp_angular_acceleration_rad_s2": alpha.tolist(),
        "method": "payload_only_finite_difference_spatial_acceleration_engineering_upper_bound",
        "qualification": data["qualification"],
        "load_model": data,
    }
