"""Analytic suction-grasp generation and pick-motion planning."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Callable, Sequence

import numpy as np

from .geometry import OBB, make_tool_rotation, make_transform, normalize, rotation_vector_from_matrix
from .ik import IKResult, solve_ik_multistart
from .planner import PlanResult, RRTConnectPlanner
from .robot import RobotKinematics6
from .scene import TrailerScene


@dataclass
class SuctionGraspCandidate:
    carton_name: str
    contact_point: np.ndarray
    outward_normal: np.ndarray
    face_mode: str
    pregrasp_pose: np.ndarray
    grasp_pose: np.ndarray
    score: float


@dataclass
class PickPlan:
    success: bool
    candidate: SuctionGraspCandidate | None
    pregrasp_ik: IKResult | None
    grasp_ik: IKResult | None
    place_ik: IKResult | None
    transit_plan: PlanResult | None
    place_plan: PlanResult | None
    approach_path: list[np.ndarray]
    place_path: list[np.ndarray]
    full_path: list[np.ndarray]
    base_path: list[np.ndarray]
    grasp_index: int
    release_index: int
    release_retreat_index: int
    message: str
    place_surface_name: str | None = None


def generate_suction_candidates(
    carton: OBB,
    robot_reference_point: np.ndarray,
    standoff: float = 0.18,
    cup_clearance: float = 0.015,
    grid_fraction: float = 0.42,
    face_modes: Sequence[str] = ("front", "side", "top"),
) -> list[SuctionGraspCandidate]:
    """Generate suction candidates on front, side, and top carton faces."""
    toward_robot = normalize(np.asarray(robot_reference_point, dtype=float) - carton.center)
    allowed = set(face_modes)
    face_options: list[tuple[float, int, float, np.ndarray, str]] = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            normal = carton.rotation[:, axis] * sign
            if axis == 0 and normal @ toward_robot > 0.0:
                mode = "front"
            elif axis == 1:
                mode = "side"
            elif axis == 2 and normal[2] > 0.0:
                mode = "top"
            else:
                continue
            if mode in allowed:
                face_options.append((float(normal @ toward_robot), axis, sign, normal, mode))

    candidates: list[SuctionGraspCandidate] = []
    offsets = (-grid_fraction, 0.0, grid_fraction)
    mode_bonus = {"front": 0.16, "side": 0.08, "top": 0.04}
    for face_alignment, axis, sign, normal, mode in face_options:
        uv_axes = [idx for idx in range(3) if idx != axis]
        u_axis, v_axis = uv_axes
        for fu in offsets:
            for fv in offsets:
                local = np.zeros(3)
                local[axis] = sign * carton.half_extents[axis]
                local[u_axis] = fu * carton.half_extents[u_axis]
                local[v_axis] = fv * carton.half_extents[v_axis]
                contact = carton.to_world(local)
                tool_z = -normal
                rotation = make_tool_rotation(tool_z)
                pregrasp = make_transform(rotation, contact + normal * standoff)
                grasp = make_transform(rotation, contact + normal * cup_clearance)
                centrality = 1.0 - 0.55 * (abs(fu) + abs(fv))
                height_fraction = (contact[2] - carton.center[2]) / max(float(carton.half_extents[2]), 1e-6)
                exposed_edge_bonus = 0.42 * max(0.0, height_fraction) if mode in {"front", "side"} else 0.0
                score = 0.66 * centrality + 0.24 * max(0.0, face_alignment) + mode_bonus[mode] + exposed_edge_bonus
                candidates.append(
                    SuctionGraspCandidate(
                        carton_name=carton.name,
                        contact_point=contact,
                        outward_normal=normal,
                        face_mode=mode,
                        pregrasp_pose=pregrasp,
                        grasp_pose=grasp,
                        score=score,
                    )
                )
    return sorted(candidates, key=lambda c: c.score, reverse=True)


def select_fast_suction_candidates(
    candidates: Sequence[SuctionGraspCandidate],
    face_modes: Sequence[str] = ("front", "side", "top"),
    candidates_per_face: int = 3,
) -> list[SuctionGraspCandidate]:
    """Keep the best few candidates for each requested face in face order.

    A global score cutoff can discard every top or side candidate when the
    front face is preferred.  Selecting per face keeps the fast planning mode
    representative of all available suction directions.
    """
    if candidates_per_face <= 0:
        raise ValueError("candidates_per_face must be positive")
    selected: list[SuctionGraspCandidate] = []
    for face_mode in face_modes:
        face_candidates = [candidate for candidate in candidates if candidate.face_mode == face_mode]
        selected.extend(face_candidates[:candidates_per_face])
    return selected


def _expand_candidate_wrist_rolls(
    candidates: Sequence[SuctionGraspCandidate],
    roll_angles_degrees: Sequence[float],
) -> list[SuctionGraspCandidate]:
    """Exploit circular-cup roll freedom without changing the contact normal.

    Rolling around tool Z leaves the suction face square to the selected box
    face.  It gives IK several nearby J4/J6 branches while keeping the grasp
    point and carton pose identical, which is much cheaper than asking a
    six-dimensional RRT to discover that redundancy later.
    """
    expanded: list[SuctionGraspCandidate] = []
    angles = list(roll_angles_degrees) or [0.0]
    for candidate in candidates:
        for angle_degrees in angles:
            angle = np.radians(float(angle_degrees))
            c, s = np.cos(angle), np.sin(angle)
            local_roll = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            pregrasp = candidate.pregrasp_pose.copy()
            grasp = candidate.grasp_pose.copy()
            pregrasp[:3, :3] = pregrasp[:3, :3] @ local_roll
            grasp[:3, :3] = grasp[:3, :3] @ local_roll
            expanded.append(
                SuctionGraspCandidate(
                    carton_name=candidate.carton_name,
                    contact_point=candidate.contact_point.copy(),
                    outward_normal=candidate.outward_normal.copy(),
                    face_mode=candidate.face_mode,
                    pregrasp_pose=pregrasp,
                    grasp_pose=grasp,
                    score=candidate.score,
                )
            )
    return expanded


def _edge_collision_free(
    robot: RobotKinematics6,
    q0: np.ndarray,
    q1: np.ndarray,
    obstacles: Sequence[OBB],
    ignored: set[str] | None = None,
    resolution: float = 0.025,
    extra_state_valid: Callable[[np.ndarray], bool] | None = None,
) -> tuple[bool, list[np.ndarray]]:
    n = max(1, int(np.ceil(np.max(np.abs(q1 - q0)) / resolution)))
    path: list[np.ndarray] = []
    for i in range(n + 1):
        q = q0 + (i / n) * (q1 - q0)
        path.append(q)
        if not robot.is_collision_free(q, obstacles, ignored_obstacle_names=ignored):
            return False, path
        if extra_state_valid is not None and not extra_state_valid(q):
            return False, path
    return True, path


def _nearest_equivalent_joint_vector(
    robot: RobotKinematics6,
    q: np.ndarray,
    reference: np.ndarray,
) -> np.ndarray:
    """Choose the closest 2*pi-equivalent revolute representation.

    Industrial wrists commonly expose more than one revolution of J4/J6.
    IK may therefore return the same physical tool pose on a distant numeric
    wrap.  Interpolating to that value creates a visually pointless full wrist
    turn.  Keep every joint within its declared limits while selecting the
    equivalent value nearest the preceding phase.
    """
    result = np.asarray(q, dtype=float).copy()
    reference = np.asarray(reference, dtype=float)
    two_pi = 2.0 * np.pi
    for index, value in enumerate(result):
        lower, upper = robot.joint_limits[index]
        equivalents = value + two_pi * np.arange(-3, 4, dtype=float)
        feasible = equivalents[(equivalents >= lower) & (equivalents <= upper)]
        if feasible.size:
            result[index] = feasible[np.argmin(np.abs(feasible - reference[index]))]
    return result


def _plan_wrist_first_carry(
    start_q: np.ndarray,
    goal_q: np.ndarray,
    state_valid: Callable[[np.ndarray], bool],
    resolution: float = 0.025,
) -> PlanResult:
    """Try a carried motion that presets the wrist before rotating J1.

    The first segment changes J4--J6, the second rotates J1 with the arm
    posture fixed, and the final segment settles into the placement posture.
    Every interpolated state remains subject to the caller's carried-box and
    posture constraints.
    """
    start = np.asarray(start_q, dtype=float)
    goal = np.asarray(goal_q, dtype=float)
    wrist_ready = start.copy()
    wrist_ready[3:] = goal[3:]
    base_turned = wrist_ready.copy()
    base_turned[0] = goal[0]
    waypoints = [start, wrist_ready, base_turned, goal]
    path: list[np.ndarray] = [start.copy()]
    for source, target in zip(waypoints[:-1], waypoints[1:]):
        steps = max(1, int(np.ceil(np.max(np.abs(target - source)) / resolution)))
        for index in range(1, steps + 1):
            q = source + (index / steps) * (target - source)
            if not state_valid(q):
                return PlanResult(False, path, 0, "wrist-first carry edge is blocked")
            path.append(q)
    return PlanResult(True, path, 0, "wrist-first carry")


def _carried_box_at(
    robot: RobotKinematics6,
    carton: OBB,
    carton_from_tool: np.ndarray,
    q: np.ndarray,
) -> OBB:
    carried_pose = robot.fk(q) @ carton_from_tool
    return OBB(
        center=carried_pose[:3, 3],
        half_extents=carton.half_extents,
        rotation=carried_pose[:3, :3],
        name=f"carried_{carton.name}",
        category=carton.category,
    )


def _carton_overlaps_trailer_interior(carton: OBB, scene: TrailerScene) -> bool:
    interior = scene.metadata.get("trailer_interior")
    if interior is None:
        return False
    corners = carton.corners()
    inside = (
        (corners[:, 0] >= 0.0)
        & (corners[:, 0] <= float(interior["length"]))
        & (np.abs(corners[:, 1]) <= float(interior["width"]) / 2.0)
        & (corners[:, 2] >= 0.0)
        & (corners[:, 2] <= float(interior["height"]))
    )
    return bool(np.any(inside))


def _in_trailer_orientation_valid(
    carton: OBB,
    carried_box: OBB,
    scene: TrailerScene,
    max_tilt_degrees: float | None,
) -> bool:
    if max_tilt_degrees is None or not _carton_overlaps_trailer_interior(carried_box, scene):
        return True
    up_alignment = float(np.clip(carried_box.rotation[:, 2] @ carton.rotation[:, 2], -1.0, 1.0))
    return bool(np.degrees(np.arccos(up_alignment)) <= max_tilt_degrees)


def _carried_box_collision_free(
    robot: RobotKinematics6,
    carton: OBB,
    grasp_q: np.ndarray,
    carried_path: Sequence[np.ndarray],
    obstacles: Sequence[OBB],
    allowed_obstacle_names: set[str] | None = None,
    box_margin: float = 0.01,
    robot_margin: float = 0.003,
    extra_state_valid: Callable[[np.ndarray], bool] | None = None,
    carton_from_tool: np.ndarray | None = None,
    max_carton_tilt_degrees: float | None = None,
    carton_contact_tolerance: float = 0.001,
    orientation_valid: Callable[[OBB], bool] | None = None,
) -> bool:
    allowed = allowed_obstacle_names or set()
    if carton_from_tool is None:
        grasp_tool = robot.fk(grasp_q)
        carton_from_tool = np.linalg.inv(grasp_tool) @ carton.world_from_local
    for q in carried_path:
        if extra_state_valid is not None and not extra_state_valid(q):
            return False
        carried_box = _carried_box_at(robot, carton, carton_from_tool, q)
        if orientation_valid is not None and not orientation_valid(carried_box):
            return False
        if orientation_valid is None and max_carton_tilt_degrees is not None:
            up_alignment = float(np.clip(carried_box.rotation[:, 2] @ carton.rotation[:, 2], -1.0, 1.0))
            if np.degrees(np.arccos(up_alignment)) > max_carton_tilt_degrees:
                return False
        for obstacle in obstacles:
            if obstacle.name in allowed:
                continue
            obstacle_margin = -carton_contact_tolerance if obstacle.category == "carton" else box_margin
            if carried_box.intersects_obb(obstacle, margin=obstacle_margin):
                return False
        for capsule in robot.link_capsules(q):
            if capsule.name == "tool":
                continue
            if capsule.collides_obb(carried_box, margin=robot_margin):
                return False
    return True


def _carried_box_state_valid(
    robot: RobotKinematics6,
    carton: OBB,
    grasp_q: np.ndarray,
    q: np.ndarray,
    obstacles: Sequence[OBB],
    box_margin: float = 0.01,
    robot_margin: float = 0.003,
    extra_state_valid: Callable[[np.ndarray], bool] | None = None,
    carton_from_tool: np.ndarray | None = None,
    max_carton_tilt_degrees: float | None = None,
    carton_contact_tolerance: float = 0.001,
    orientation_valid: Callable[[OBB], bool] | None = None,
) -> bool:
    if extra_state_valid is not None and not extra_state_valid(q):
        return False
    if carton_from_tool is None:
        grasp_tool = robot.fk(grasp_q)
        carton_from_tool = np.linalg.inv(grasp_tool) @ carton.world_from_local
    carried_box = _carried_box_at(robot, carton, carton_from_tool, q)
    if orientation_valid is not None and not orientation_valid(carried_box):
        return False
    if orientation_valid is None and max_carton_tilt_degrees is not None:
        up_alignment = float(np.clip(carried_box.rotation[:, 2] @ carton.rotation[:, 2], -1.0, 1.0))
        if np.degrees(np.arccos(up_alignment)) > max_carton_tilt_degrees:
            return False
    for obstacle in obstacles:
        obstacle_margin = -carton_contact_tolerance if obstacle.category == "carton" else box_margin
        if carried_box.intersects_obb(obstacle, margin=obstacle_margin):
            return False
    for capsule in robot.link_capsules(q):
        if capsule.name == "tool":
            continue
        if capsule.collides_obb(carried_box, margin=robot_margin):
            return False
    return True


def _mobile_base_translation_safe(
    robot: RobotKinematics6,
    carton: OBB,
    grasp_q: np.ndarray,
    carried_q: np.ndarray,
    source_base: np.ndarray,
    target_base: np.ndarray,
    obstacles: Sequence[OBB],
    carton_from_tool: np.ndarray,
    extra_state_valid: Callable[[np.ndarray], bool] | None = None,
    resolution: float = 0.05,
) -> list[np.ndarray] | None:
    """Check the full robot and held carton while the mobile base translates."""
    distance = float(np.linalg.norm(target_base[:3, 3] - source_base[:3, 3]))
    steps = max(1, int(np.ceil(distance / resolution)))
    base_path: list[np.ndarray] = []
    for index in range(steps + 1):
        fraction = index / steps
        robot.base_transform = source_base.copy()
        robot.base_transform[:3, 3] = (1.0 - fraction) * source_base[:3, 3] + fraction * target_base[:3, 3]
        base_path.append(robot.base_transform[:3, 3].copy())
        if not robot.is_collision_free(carried_q, obstacles):
            robot.base_transform = source_base
            return None
        if not _carried_box_state_valid(
            robot,
            carton,
            grasp_q,
            carried_q,
            obstacles,
            box_margin=0.01,
            extra_state_valid=extra_state_valid,
            carton_from_tool=carton_from_tool,
        ):
            robot.base_transform = source_base
            return None
    robot.base_transform = target_base.copy()
    return base_path


def _rotation_from_vector(rotation_vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-10:
        return np.eye(3)
    axis = rotation_vector / angle
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _pose_transfer_cost(
    source_pose: np.ndarray,
    target_pose: np.ndarray,
    orientation_weight: float = 0.35,
) -> float:
    position_cost = float(np.linalg.norm(target_pose[:3, 3] - source_pose[:3, 3]))
    orientation_cost = float(
        np.linalg.norm(rotation_vector_from_matrix(target_pose[:3, :3] @ source_pose[:3, :3].T))
    )
    return position_cost + orientation_weight * orientation_cost


def _conveyor_preplace_poses(
    release_pose: np.ndarray,
    clearance: float,
    directions: Sequence[str] = ("front", "top", "left", "right"),
    conveyor_rotation: np.ndarray | None = None,
) -> list[tuple[str, np.ndarray]]:
    """Return approach poses around a stable conveyor release pose.

    The carton/tool orientation is already fixed by the release pose.  Only
    the translation changes. Once a direction is selected, the final segment
    is a direct Cartesian insertion into release without further wrist motion.
    Direction labels are expressed in the conveyor frame.
    """
    offsets = {
        "front": np.array([1.0, 0.0, 0.0]),
        "top": np.array([0.0, 0.0, 1.0]),
        "left": np.array([0.0, 1.0, 0.0]),
        "right": np.array([0.0, -1.0, 0.0]),
    }
    rotation = np.eye(3) if conveyor_rotation is None else np.asarray(conveyor_rotation, dtype=float)
    poses: list[tuple[str, np.ndarray]] = []
    for direction in directions:
        if direction not in offsets:
            raise ValueError(f"unknown conveyor approach direction: {direction!r}")
        preplace = np.asarray(release_pose, dtype=float).copy()
        preplace[:3, 3] += float(clearance) * (rotation @ offsets[direction])
        poses.append((direction, preplace))
    return poses


def _plan_cartesian_carry(
    robot: RobotKinematics6,
    carton: OBB,
    grasp_q: np.ndarray,
    start_q: np.ndarray,
    target_tool_pose: np.ndarray,
    robot_obstacles: Sequence[OBB],
    carried_obstacles: Sequence[OBB],
    carton_from_tool: np.ndarray,
    orientation_valid: Callable[[OBB], bool] | None,
    rng: np.random.Generator,
    extra_state_valid: Callable[[np.ndarray], bool] | None = None,
    position_step: float = 0.10,
    orientation_step: float = 0.16,
    joint_resolution: float = 0.035,
    ik_iterations: int = 180,
    position_tolerance: float = 0.025,
    orientation_tolerance: float = 0.18,
) -> PlanResult:
    start_pose = robot.fk(start_q)
    position_distance = float(np.linalg.norm(target_tool_pose[:3, 3] - start_pose[:3, 3]))
    relative_rotation = target_tool_pose[:3, :3] @ start_pose[:3, :3].T
    rotation_vector = rotation_vector_from_matrix(relative_rotation)
    steps = max(
        1,
        int(np.ceil(position_distance / position_step)),
        int(np.ceil(np.linalg.norm(rotation_vector) / orientation_step)),
    )
    path = [np.asarray(start_q, dtype=float).copy()]

    def carried_valid(q: np.ndarray) -> bool:
        return (extra_state_valid is None or extra_state_valid(q)) and _carried_box_state_valid(
            robot,
            carton,
            grasp_q,
            q,
            carried_obstacles,
            box_margin=0.01,
            carton_from_tool=carton_from_tool,
            orientation_valid=orientation_valid,
        )

    for index in range(1, steps + 1):
        fraction = index / steps
        target = np.eye(4)
        target[:3, :3] = _rotation_from_vector(rotation_vector * fraction) @ start_pose[:3, :3]
        target[:3, 3] = (1.0 - fraction) * start_pose[:3, 3] + fraction * target_tool_pose[:3, 3]
        result = solve_ik_multistart(
            robot,
            target,
            seeds=[path[-1]],
            obstacles=robot_obstacles,
            random_restarts=2,
            rng=rng,
            max_iterations=ik_iterations,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            extra_state_valid=carried_valid,
        )
        if not result.success:
            return PlanResult(False, [], index, f"Cartesian carry IK failed at waypoint {index}/{steps}")
        edge_ok, edge = _edge_collision_free(
            robot,
            path[-1],
            result.q,
            robot_obstacles,
            resolution=joint_resolution,
            extra_state_valid=carried_valid,
        )
        if not edge_ok:
            return PlanResult(False, [], index, f"Cartesian carry edge blocked at waypoint {index}/{steps}")
        path.extend(edge[1:])
    return PlanResult(True, path, steps, "Cartesian carry")


def _plan_via_joint_hints(
    planner: RRTConnectPlanner,
    start_q: np.ndarray,
    goal_q: np.ndarray,
    hints: Sequence[np.ndarray],
    time_limit_seconds: float | None = None,
) -> PlanResult:
    deadline = None if time_limit_seconds is None else perf_counter() + max(0.0, float(time_limit_seconds))
    waypoints = [np.asarray(start_q, dtype=float), *[np.asarray(hint, dtype=float) for hint in hints], np.asarray(goal_q, dtype=float)]
    path = [waypoints[0].copy()]
    iterations = 0
    for segment_index, (source, target) in enumerate(zip(waypoints[:-1], waypoints[1:]), start=1):
        remaining = None if deadline is None else max(0.0, deadline - perf_counter())
        segment = planner.plan(source, target, time_limit_seconds=remaining)
        iterations += segment.iterations
        if not segment.success:
            return PlanResult(False, path, iterations, f"transit segment {segment_index} failed: {segment.message}")
        path.extend(segment.path[1:])
    return PlanResult(True, path, iterations, "connected via joint hints" if hints else "connected")


def plan_pick(
    robot: RobotKinematics6,
    scene: TrailerScene,
    start_q: np.ndarray,
    target_carton_name: str,
    planner_options: dict | None = None,
    rng: np.random.Generator | None = None,
) -> PickPlan:
    rng = rng or np.random.default_rng(7)
    planner_options = planner_options or {}
    verbose = bool(planner_options.get("verbose", False))

    def log(message: str) -> None:
        if verbose:
            print(f"[plan_pick] {message}", flush=True)

    from .perception import conveyor_place_pose_candidates, detect_carton_obbs, select_detection, surface_place_pose_candidates

    detections = detect_carton_obbs(scene)
    log(f"detected {len(detections)} carton OBBs")
    carton = select_detection(detections, target_carton_name).obb
    pick_base = robot.base_transform.copy()
    if planner_options.get("mobile_base_place_position") is not None:
        raise ValueError("AMR motion while carrying a carton is not allowed")
    mobile_place_position = None
    place_base = pick_base.copy()
    if mobile_place_position is not None:
        place_base[:3, 3] = np.asarray(mobile_place_position, dtype=float)
    robot_ref = robot.base_transform[:3, 3]
    face_modes = planner_options.get("grasp_face_modes", ["front", "side", "top"])
    enable_place = bool(planner_options.get("enable_place", False))
    if "place_target" in planner_options:
        place_target_names = [str(planner_options["place_target"])]
    else:
        place_target_names = [
            str(name)
            for name in planner_options.get(
                "place_obstacles",
                [planner_options.get("place_obstacle", "conveyor_deck")],
            )
        ]
    place_surfaces = {
        obstacle.name: obstacle
        for obstacle in scene.obstacles
        if obstacle.name in place_target_names
    }
    place_enabled = enable_place and bool(place_surfaces)
    place_target_name: str | None = None
    place_surface: OBB | None = None

    def placement_poses(grasp_tool_pose: np.ndarray) -> list[tuple[str, OBB, np.ndarray]]:
        edge_clearance = float(planner_options.get("conveyor_edge_clearance", 0.01))
        entries: list[tuple[str, OBB, np.ndarray]] = []
        for surface_name in place_target_names:
            surface = place_surfaces.get(surface_name)
            if surface is None:
                continue
            requires_release_zone = bool(
                planner_options.get("place_requires_release_zone", "place_target" not in planner_options)
            )
            if requires_release_zone:
                poses = conveyor_place_pose_candidates(
                    surface,
                    carton,
                    grasp_tool_pose,
                    robot.base_transform[:3, 3],
                    edge_clearance=edge_clearance,
                    allow_all_carton_faces=bool(planner_options.get("conveyor_allow_all_carton_faces", False)),
                    require_front_release_zone=bool(planner_options.get("conveyor_require_front_release_zone", True)),
                    release_height=float(planner_options.get("conveyor_release_height", 0.0)),
                )
            else:
                poses = surface_place_pose_candidates(
                    surface,
                    carton,
                    grasp_tool_pose,
                    edge_clearance=edge_clearance,
                )
            entries.extend((surface_name, surface, pose) for pose in poses)
        return entries
    candidates = generate_suction_candidates(
        carton,
        robot_ref,
        face_modes=face_modes,
    )
    candidates = _expand_candidate_wrist_rolls(
        candidates,
        planner_options.get("wrist_roll_angles_deg", [0.0]),
    )
    if place_enabled:
        candidates = [
            candidate
            for candidate in candidates
            if placement_poses(candidate.grasp_pose)
        ]
    candidates_per_face = planner_options.get("grasp_candidates_per_face", 3)
    candidates = select_fast_suction_candidates(candidates, face_modes, int(candidates_per_face))
    log(f"generated {len(candidates)} suction candidates for {target_carton_name}")
    all_obstacles = scene.all_obstacles
    max_link3_elevation = planner_options.get("max_link3_elevation_deg")

    def posture_valid(q: np.ndarray) -> bool:
        return max_link3_elevation is None or robot.link_elevation_degrees(q, 3) <= float(max_link3_elevation)

    carried_box_clearance = float(planner_options.get("carried_box_clearance", 0.01))
    max_in_trailer_tilt = planner_options.get("max_in_trailer_carton_tilt_deg")
    carton_contact_tolerance = float(planner_options.get("carton_contact_tolerance_m", 0.001))
    use_cartesian_carry = bool(planner_options.get("use_cartesian_carry", False))
    # Online unloading now plans directly to release. Keep the legacy option
    # from reintroducing a pre-place insertion or its matching empty retreat.
    use_cartesian_final_approach = False
    use_wrist_first_carry = bool(planner_options.get("use_wrist_first_carry", False))
    conveyor_preplace_clearance = float(planner_options.get("conveyor_preplace_clearance", 0.30))

    failure_messages: list[str] = []
    best_pick_plan: PickPlan | None = None
    best_pick_cost = float("inf")
    plan_t0 = perf_counter()
    candidate_time_budget = float(planner_options.get("target_planning_time_limit_seconds", 0.0))
    for candidate_index, candidate in enumerate(candidates):
        robot.base_transform = pick_base.copy()
        if candidate_time_budget > 0.0 and perf_counter() - plan_t0 >= candidate_time_budget:
            failure_messages.append(f"target planning time budget ({candidate_time_budget:.1f}s) exceeded")
            log(f"target={target_carton_name}: time budget exceeded")
            break
        candidate_t0 = perf_counter()
        log(
            f"candidate {candidate_index}/{len(candidates) - 1}: "
            f"face={candidate.face_mode}, score={candidate.score:.3f}, "
            f"contact={np.round(candidate.contact_point, 3).tolist()}"
        )
        seeds = [start_q]
        pregrasp_cache = planner_options.get("pregrasp_seed_cache")
        if pregrasp_cache is not None:
            cached_seed = pregrasp_cache.nearest_seed(
                candidate.pregrasp_pose[:3, 3],
                robot.base_transform[:3, 3],
            )
            if cached_seed is not None:
                # Preserve trajectory continuity: the current arm state is
                # the primary seed.  The heatmap seed is a reachability
                # fallback, not permission to jump to a distant IK branch.
                seeds.append(cached_seed)
        if candidate_index > 0:
            seeds.append(rng.uniform(robot.joint_limits[:, 0] * 0.55, robot.joint_limits[:, 1] * 0.55))

        # This pose is a collision-free RRT connector only. It is not exposed
        # as a pre-grasp phase: the emitted trajectory passes through it
        # continuously into contact.
        approach_guide = solve_ik_multistart(
            robot,
            candidate.pregrasp_pose,
            seeds=seeds,
            obstacles=all_obstacles,
            random_restarts=planner_options.get("ik_random_restarts", 12),
            rng=rng,
            max_iterations=planner_options.get("ik_max_iterations", 250),
            position_tolerance=planner_options.get("ik_position_tolerance", 0.012),
            orientation_tolerance=planner_options.get("ik_orientation_tolerance", 0.10),
            extra_state_valid=posture_valid,
        )
        if not approach_guide.success:
            failure_messages.append(f"candidate {candidate_index}: contact approach guide IK failed")
            continue

        log(f"candidate {candidate_index}: solving grasp IK")
        grasp = solve_ik_multistart(
            robot,
            candidate.grasp_pose,
            seeds=[approach_guide.q, start_q],
            obstacles=all_obstacles,
            ignored_obstacle_names={carton.name},
            random_restarts=4,
            rng=rng,
            max_iterations=planner_options.get("ik_max_iterations", 250),
            position_tolerance=planner_options.get("ik_position_tolerance", 0.012),
            orientation_tolerance=planner_options.get("ik_orientation_tolerance", 0.10),
            extra_state_valid=posture_valid,
        )
        if not grasp.success:
            failure_messages.append(f"candidate {candidate_index}: grasp IK failed")
            log(f"candidate {candidate_index}: grasp IK failed after {perf_counter() - candidate_t0:.1f}s")
            continue

        place_poses = (
            placement_poses(robot.fk(grasp.q))
            if place_enabled
            else []
        )
        place_hint_joints = [np.asarray(q, dtype=float) for q in planner_options.get("place_hint_joints", [])]
        if place_poses and place_hint_joints:
            hint_poses = [robot.fk(q) for q in place_hint_joints]

            def place_hint_cost(entry: tuple[str, OBB, np.ndarray]) -> float:
                pose = entry[2]
                return min(
                    float(np.linalg.norm(pose[:3, 3] - hint_pose[:3, 3]))
                    + 0.25 * float(np.linalg.norm(rotation_vector_from_matrix(pose[:3, :3] @ hint_pose[:3, :3].T)))
                    for hint_pose in hint_poses
                )

            place_poses.sort(key=place_hint_cost)
        if place_enabled and not place_poses:
            failure_messages.append(f"candidate {candidate_index}: carton does not fit conveyor")
            log(f"candidate {candidate_index}: carton does not fit conveyor")
            continue

        carton_from_tool = np.linalg.inv(robot.fk(grasp.q)) @ carton.world_from_local
        # Carry-exit must avoid both belts. The selected release surface is
        # ignored only after a concrete placement candidate has been chosen.
        carried_obstacles = scene.obstacles_without({carton.name})
        place_robot_obstacles = scene.obstacles_without({carton.name})

        def carried_orientation_valid(carried_box: OBB) -> bool:
            clearance_zone_max_x = planner_options.get("clearance_zone_max_x")
            if clearance_zone_max_x is not None and carried_box.center[0] <= float(clearance_zone_max_x):
                # Once the carton center has left the stack envelope, exact
                # OBB checks against cartons, robot links, floor, roof and
                # trailer walls remain active; only the conservative upright
                # rule is released so the wrist can reorient in free space.
                return True
            return _in_trailer_orientation_valid(carton, carried_box, scene, max_in_trailer_tilt)

        def carried_goal_valid(q: np.ndarray) -> bool:
            return posture_valid(q) and _carried_box_state_valid(
                robot,
                carton,
                grasp.q,
                q,
                carried_obstacles,
                box_margin=carried_box_clearance,
                carton_from_tool=carton_from_tool,
                carton_contact_tolerance=carton_contact_tolerance,
                orientation_valid=carried_orientation_valid,
            )

        approach_ok, contact_approach = _edge_collision_free(
            robot,
            approach_guide.q,
            grasp.q,
            all_obstacles,
            ignored={carton.name},
            resolution=planner_options.get("approach_resolution", 0.02),
            extra_state_valid=posture_valid,
        )
        if not approach_ok:
            failure_messages.append(f"candidate {candidate_index}: contact approach is blocked")
            continue

        log(f"candidate {candidate_index}: planning transit")
        state_valid = lambda q: posture_valid(q) and robot.is_collision_free(q, all_obstacles)
        planner = RRTConnectPlanner(
            robot.joint_limits[:, 0],
            robot.joint_limits[:, 1],
            state_valid,
            step_size=planner_options.get("rrt_step_size", 0.22),
            edge_resolution=planner_options.get("rrt_edge_resolution", 0.065),
            max_iterations=planner_options.get("rrt_max_iterations", 4500),
            goal_bias=planner_options.get("rrt_goal_bias", 0.15),
            rng=rng,
        )
        transit_hints = [np.asarray(q, dtype=float) for q in planner_options.get("transit_hint_joints", [])]
        remaining_budget = None
        if candidate_time_budget > 0.0:
            remaining_budget = max(0.0, candidate_time_budget - (perf_counter() - plan_t0))
        transit_limit = planner_options.get("transit_time_limit_seconds")
        if transit_limit is not None:
            remaining_budget = float(transit_limit) if remaining_budget is None else min(remaining_budget, float(transit_limit))
        transit = _plan_via_joint_hints(
            planner,
            start_q,
            approach_guide.q,
            transit_hints,
            time_limit_seconds=remaining_budget,
        )
        if not transit.success:
            failure_messages.append(f"candidate {candidate_index}: {transit.message}")
            log(f"candidate {candidate_index}: {transit.message} after {perf_counter() - candidate_t0:.1f}s")
            continue
        smooth = planner.shortcut(transit.path, attempts=planner_options.get("shortcut_attempts", 180))
        dense_transit = planner.densify(smooth, resolution=planner_options.get("output_resolution", 0.035))

        place = None
        place_transit = None
        selected_place_pose = None
        carry_exit = None
        dense_place: list[np.ndarray] = []
        retreat_path: list[np.ndarray] = []
        dense_exit: list[np.ndarray] = []
        if place_enabled:
            standoff_by_face = planner_options.get("carry_exit_standoff_by_face", {})
            carry_exit_standoff = float(
                standoff_by_face.get(
                    candidate.face_mode,
                    planner_options.get("carry_exit_standoff", 0.68),
                )
            )
            carry_exit_pose = make_transform(
                candidate.pregrasp_pose[:3, :3],
                candidate.contact_point + candidate.outward_normal * carry_exit_standoff,
            )
            carry_exit = solve_ik_multistart(
                robot,
                carry_exit_pose,
                seeds=[approach_guide.q, grasp.q, start_q],
                obstacles=scene.obstacles_without({carton.name}),
                random_restarts=planner_options.get("place_ik_random_restarts", 10),
                rng=rng,
                max_iterations=planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250)),
                position_tolerance=planner_options.get("place_position_tolerance", 0.025),
                orientation_tolerance=planner_options.get("place_orientation_tolerance", 0.22),
                extra_state_valid=carried_goal_valid,
            )
            if not carry_exit.success:
                failure_messages.append(f"candidate {candidate_index}: carry-exit IK failed")
                continue
            carry_exit.q = _nearest_equivalent_joint_vector(robot, carry_exit.q, approach_guide.q)
            if mobile_place_position is None:
                carry_exit_tool_pose = robot.fk(carry_exit.q)

                def transfer_pose_cost(pose: np.ndarray) -> float:
                    return _pose_transfer_cost(carry_exit_tool_pose, pose)

                place_poses.sort(key=lambda entry: transfer_pose_cost(entry[2]))
                surface_counts = {
                    name: sum(entry[0] == name for entry in place_poses)
                    for name in place_target_names
                }
                log(f"candidate {candidate_index}: evaluating supported conveyor poses {surface_counts}")
                feasible_places = []
                max_place_candidates = int(planner_options.get("max_place_pose_candidates", len(place_poses)))
                max_feasible_places = int(planner_options.get("max_feasible_place_iks", max_place_candidates))
                # Preserve the global transfer-cost order while reserving an
                # equal share of the bounded IK budget for each conveyor.
                # Otherwise the first surface can consume every online probe
                # even when the cross conveyor gives the shorter final path.
                surface_buckets = {
                    name: [entry for entry in place_poses if entry[0] == name]
                    for name in place_target_names
                }
                balanced_place_poses: list[tuple[str, OBB, np.ndarray]] = []
                bucket_index = 0
                while len(balanced_place_poses) < max_place_candidates:
                    added = False
                    for name in place_target_names:
                        bucket = surface_buckets.get(name, [])
                        if bucket_index < len(bucket):
                            balanced_place_poses.append(bucket[bucket_index])
                            added = True
                            if len(balanced_place_poses) >= max_place_candidates:
                                break
                    if not added:
                        break
                    bucket_index += 1
                place_ik_deadline = perf_counter() + float(
                    planner_options.get("place_ik_budget_seconds", 0.0)
                )
                for surface_name, surface, pose in balanced_place_poses:
                    if (
                        float(planner_options.get("place_ik_budget_seconds", 0.0)) > 0.0
                        and perf_counter() >= place_ik_deadline
                    ):
                        break
                    surface_carried_obstacles = scene.obstacles_without({carton.name, surface_name})

                    def placement_goal_valid(q: np.ndarray) -> bool:
                        return posture_valid(q) and _carried_box_state_valid(
                            robot,
                            carton,
                            grasp.q,
                            q,
                            surface_carried_obstacles,
                            box_margin=carried_box_clearance,
                            carton_from_tool=carton_from_tool,
                            carton_contact_tolerance=carton_contact_tolerance,
                            orientation_valid=carried_orientation_valid,
                        )

                    result = solve_ik_multistart(
                        robot,
                        pose,
                        seeds=[*place_hint_joints, carry_exit.q, grasp.q],
                        obstacles=scene.obstacles_without({carton.name}),
                        random_restarts=planner_options.get("place_ik_random_restarts", 10),
                        rng=rng,
                        max_iterations=planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250)),
                        position_tolerance=planner_options.get("place_position_tolerance", 0.025),
                        orientation_tolerance=planner_options.get("place_orientation_tolerance", 0.22),
                        extra_state_valid=placement_goal_valid,
                    )
                    if result.success:
                        result.q = _nearest_equivalent_joint_vector(robot, result.q, carry_exit.q)
                        feasible_places.append((surface_name, surface, pose, result))
                        if len(feasible_places) >= max_feasible_places:
                            break
                if not feasible_places:
                    failure_messages.append(f"candidate {candidate_index}: place IK failed")
                    log(f"candidate {candidate_index}: place IK failed after {perf_counter() - candidate_t0:.1f}s")
                    continue
                if use_cartesian_final_approach:
                    approach_directions = tuple(planner_options.get("conveyor_approach_directions", ("front", "top", "left", "right")))
                    preplace_results = []
                    for surface_name, surface, release_pose, release_ik in feasible_places:
                        for approach_direction, candidate_preplace_pose in _conveyor_preplace_poses(
                            release_pose,
                            conveyor_preplace_clearance,
                            approach_directions,
                            surface.rotation,
                        ):
                            preplace_ik = solve_ik_multistart(
                                robot,
                                candidate_preplace_pose,
                                seeds=[release_ik.q, carry_exit.q, grasp.q, start_q],
                                obstacles=scene.obstacles_without({carton.name}),
                                random_restarts=planner_options.get("place_ik_random_restarts", 10),
                                rng=rng,
                                max_iterations=planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250)),
                                position_tolerance=planner_options.get("place_position_tolerance", 0.025),
                                orientation_tolerance=planner_options.get("place_orientation_tolerance", 0.22),
                                extra_state_valid=placement_goal_valid,
                            )
                            if preplace_ik.success:
                                preplace_results.append((surface_name, surface, release_pose, release_ik, approach_direction, candidate_preplace_pose, preplace_ik))
                    if not preplace_results:
                        failure_messages.append(f"candidate {candidate_index}: conveyor preplace IK failed")
                        log(f"candidate {candidate_index}: conveyor preplace IK failed after {perf_counter() - candidate_t0:.1f}s")
                        continue
                    place_target_name, place_surface, selected_place_pose, place, approach_direction, preplace_pose, preplace = min(
                        preplace_results,
                        key=lambda item: float(np.linalg.norm(item[3].q - carry_exit.q) + np.linalg.norm(item[6].q - item[3].q)),
                    )
                    log(f"candidate {candidate_index}: selected conveyor approach={approach_direction}")
                else:
                    joint_weight = float(planner_options.get("place_joint_motion_weight", 0.10))
                    surface_penalties = planner_options.get("place_surface_penalties", {})
                    place_target_name, place_surface, selected_place_pose, place = min(
                        feasible_places,
                        key=lambda item: (
                            transfer_pose_cost(item[2])
                            + joint_weight * float(np.linalg.norm(item[3].q - carry_exit.q))
                            + float(surface_penalties.get(item[0], 0.0))
                        ),
                    )
                carried_obstacles = scene.obstacles_without({carton.name, place_target_name})
                log(f"candidate {candidate_index}: selected placement surface={place_target_name}")

        mobile_base_path: list[np.ndarray] = []
        if place_enabled:
            log(f"candidate {candidate_index}: planning place RRT")
            place_attempt_limit = planner_options.get("place_attempt_time_limit_seconds")
            place_valid = lambda q: robot.is_collision_free(q, place_robot_obstacles) and _carried_box_state_valid(
                robot,
                carton,
                grasp.q,
                q,
                carried_obstacles,
                box_margin=carried_box_clearance,
                extra_state_valid=posture_valid,
                carton_contact_tolerance=carton_contact_tolerance,
                orientation_valid=carried_orientation_valid,
            )
            place_planner = RRTConnectPlanner(
                robot.joint_limits[:, 0],
                robot.joint_limits[:, 1],
                place_valid,
                step_size=planner_options.get("rrt_step_size", 0.22),
                edge_resolution=planner_options.get("rrt_edge_resolution", 0.065),
                max_iterations=planner_options.get("rrt_max_iterations", 4500),
                goal_bias=planner_options.get("rrt_goal_bias", 0.15),
                rng=rng,
            )
            retreat = _plan_cartesian_carry(
                robot,
                carton,
                grasp.q,
                grasp.q,
                candidate.pregrasp_pose,
                place_robot_obstacles,
                carried_obstacles,
                carton_from_tool,
                carried_orientation_valid,
                rng,
                extra_state_valid=posture_valid,
                position_step=float(planner_options.get("destack_position_step", 0.04)),
                orientation_step=float(planner_options.get("cartesian_carry_orientation_step", 0.16)),
                joint_resolution=float(planner_options.get("output_resolution", 0.035)),
                ik_iterations=int(planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250))),
                position_tolerance=float(planner_options.get("place_position_tolerance", 0.025)),
                orientation_tolerance=float(planner_options.get("place_orientation_tolerance", 0.22)),
            )
            if not retreat.success:
                direct_retreat_ok, direct_retreat_path = _edge_collision_free(
                    robot,
                    grasp.q,
                    approach_guide.q,
                    place_robot_obstacles,
                    resolution=float(planner_options.get("output_resolution", 0.035)),
                    extra_state_valid=place_valid,
                )
                if direct_retreat_ok:
                    retreat = PlanResult(
                        True,
                        direct_retreat_path,
                        0,
                        "direct collision-checked destack fallback",
                    )
                else:
                    failure_messages.append(f"candidate {candidate_index}: {retreat.message}")
                    log(f"candidate {candidate_index}: {retreat.message} after {perf_counter() - candidate_t0:.1f}s")
                    continue
            retreat_path = retreat.path
            withdrawal_q = retreat_path[-1]
            if bool(planner_options.get("use_cartesian_destack_to_clearance", False)):
                # Stretch-style phase 1: keep the front-grasp orientation and
                # withdraw in a straight line until the carton reaches the
                # clearance zone.  Wrist reorientation is forbidden during
                # this phase, eliminating RRT wrist excursions near the stack.
                exit_transit = _plan_cartesian_carry(
                    robot,
                    carton,
                    grasp.q,
                    withdrawal_q,
                    robot.fk(carry_exit.q),
                    place_robot_obstacles,
                    carried_obstacles,
                    carton_from_tool,
                    carried_orientation_valid,
                    rng,
                    extra_state_valid=posture_valid,
                    position_step=float(planner_options.get("destack_position_step", 0.04)),
                    orientation_step=float(planner_options.get("cartesian_carry_orientation_step", 0.16)),
                    joint_resolution=float(planner_options.get("output_resolution", 0.035)),
                    ik_iterations=int(planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250))),
                    position_tolerance=float(planner_options.get("place_position_tolerance", 0.025)),
                    orientation_tolerance=float(planner_options.get("place_orientation_tolerance", 0.22)),
                )
            elif use_wrist_first_carry:
                exit_transit = _plan_wrist_first_carry(
                    withdrawal_q, carry_exit.q, place_valid,
                    resolution=float(planner_options.get("approach_resolution", 0.02)),
                )
                if not exit_transit.success:
                    exit_transit = place_planner.plan(
                        withdrawal_q, carry_exit.q, time_limit_seconds=place_attempt_limit
                    )
            else:
                exit_transit = place_planner.plan(
                    withdrawal_q, carry_exit.q, time_limit_seconds=place_attempt_limit
                )
            if (
                not exit_transit.success
                and bool(planner_options.get("use_cartesian_destack_to_clearance", False))
            ):
                # Numerical IK can miss an intermediate Cartesian sample even
                # when both endpoints share the same tool orientation.  A
                # direct, densely collision-checked joint connector is the
                # bounded fallback; unlike RRT it cannot add wrist wandering.
                direct_exit_ok, direct_exit_path = _edge_collision_free(
                    robot,
                    withdrawal_q,
                    carry_exit.q,
                    place_robot_obstacles,
                    resolution=float(planner_options.get("output_resolution", 0.035)),
                    extra_state_valid=place_valid,
                )
                if direct_exit_ok:
                    exit_transit = PlanResult(True, direct_exit_path, 0, "direct destack-to-clearance phase")
                else:
                    # Inner cartons sometimes need a small shoulder/elbow arc
                    # to clear their neighbours. Keep the same fixed tool
                    # orientation at both ends and use RRT only for that
                    # geometrically necessary connector.
                    exit_transit = place_planner.plan(
                        withdrawal_q, carry_exit.q, time_limit_seconds=place_attempt_limit
                    )
            if not exit_transit.success:
                failure_messages.append(f"candidate {candidate_index}: {exit_transit.message}")
                log(f"candidate {candidate_index}: {exit_transit.message} after {perf_counter() - candidate_t0:.1f}s")
                continue
            preserve_exit_phase = use_wrist_first_carry or bool(
                planner_options.get("use_cartesian_destack_to_clearance", False)
            )
            dense_exit = place_planner.densify(
                exit_transit.path if preserve_exit_phase else place_planner.shortcut(
                    exit_transit.path,
                    attempts=planner_options.get("shortcut_attempts", 180),
                ),
                resolution=planner_options.get("output_resolution", 0.035),
            )
            if mobile_place_position is not None:
                carton_from_tool = np.linalg.inv(robot.fk(grasp.q)) @ carton.world_from_local
                mobile_base_path = _mobile_base_translation_safe(
                    robot,
                    carton,
                    grasp.q,
                    carry_exit.q,
                    pick_base,
                    place_base,
                    place_robot_obstacles,
                    carton_from_tool,
                    extra_state_valid=posture_valid,
                    resolution=float(planner_options.get("mobile_base_resolution", 0.05)),
                )
                if mobile_base_path is None:
                    failure_messages.append(f"candidate {candidate_index}: mobile base translation blocked")
                    log(f"candidate {candidate_index}: mobile base translation blocked after {perf_counter() - candidate_t0:.1f}s")
                    continue
                log(f"candidate {candidate_index}: solving mobile-base conveyor place IK")
                place = solve_ik_multistart(
                    robot,
                    place_poses[0],
                    seeds=[carry_exit.q, grasp.q, approach_guide.q],
                    obstacles=place_robot_obstacles,
                    random_restarts=planner_options.get("place_ik_random_restarts", 10),
                    rng=rng,
                    max_iterations=planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250)),
                    position_tolerance=planner_options.get("place_position_tolerance", 0.025),
                    orientation_tolerance=planner_options.get("place_orientation_tolerance", 0.22),
                    extra_state_valid=posture_valid,
                )
                if not place.success:
                    robot.base_transform = pick_base.copy()
                    failure_messages.append(f"candidate {candidate_index}: mobile-base place IK failed")
                    log(f"candidate {candidate_index}: mobile-base place IK failed after {perf_counter() - candidate_t0:.1f}s")
                    continue
                place_planner = RRTConnectPlanner(
                    robot.joint_limits[:, 0],
                    robot.joint_limits[:, 1],
                    place_valid,
                    step_size=planner_options.get("rrt_step_size", 0.22),
                    edge_resolution=planner_options.get("rrt_edge_resolution", 0.065),
                    max_iterations=planner_options.get("rrt_max_iterations", 4500),
                    goal_bias=planner_options.get("rrt_goal_bias", 0.15),
                    rng=rng,
                )
            if use_cartesian_final_approach and mobile_place_position is None:
                assert selected_place_pose is not None
                assert preplace_pose is not None and preplace is not None
                reconfigure_transit = place_planner.plan(
                    carry_exit.q, preplace.q, time_limit_seconds=place_attempt_limit
                )
                if not reconfigure_transit.success:
                    failure_messages.append(f"candidate {candidate_index}: {reconfigure_transit.message}")
                    log(f"candidate {candidate_index}: {reconfigure_transit.message} after {perf_counter() - candidate_t0:.1f}s")
                    continue
                dense_reconfigure = place_planner.densify(
                    place_planner.shortcut(reconfigure_transit.path, attempts=planner_options.get("shortcut_attempts", 180)),
                    resolution=planner_options.get("output_resolution", 0.035),
                )
                place_transit = _plan_cartesian_carry(
                    robot,
                    carton,
                    grasp.q,
                    dense_reconfigure[-1],
                    selected_place_pose,
                    place_robot_obstacles,
                    carried_obstacles,
                    carton_from_tool,
                    carried_orientation_valid,
                    rng,
                    extra_state_valid=posture_valid,
                    position_step=float(planner_options.get("cartesian_carry_position_step", 0.10)),
                    orientation_step=float(planner_options.get("cartesian_carry_orientation_step", 0.16)),
                    joint_resolution=float(planner_options.get("output_resolution", 0.035)),
                )
            elif use_wrist_first_carry:
                place_transit = _plan_wrist_first_carry(
                    carry_exit.q,
                    place.q,
                    place_valid,
                    resolution=float(planner_options.get("approach_resolution", 0.02)),
                )
                if not place_transit.success:
                    place_transit = place_planner.plan(
                        carry_exit.q, place.q, time_limit_seconds=place_attempt_limit
                    )
            else:
                direct_ok = False
                direct_path: list[np.ndarray] = []
                if bool(planner_options.get("prefer_direct_clearance_to_place", False)):
                    direct_ok, direct_path = _edge_collision_free(
                        robot,
                        carry_exit.q,
                        place.q,
                        place_robot_obstacles,
                        resolution=float(planner_options.get("output_resolution", 0.035)),
                        extra_state_valid=place_valid,
                    )
                place_transit = (
                    PlanResult(True, direct_path, 0, "direct clearance-to-place phase")
                    if direct_ok
                    else place_planner.plan(
                        carry_exit.q, place.q, time_limit_seconds=place_attempt_limit
                    )
                )
            if not place_transit.success:
                failure_messages.append(f"candidate {candidate_index}: {place_transit.message}")
                log(f"candidate {candidate_index}: {place_transit.message} after {perf_counter() - candidate_t0:.1f}s")
                continue
            if use_cartesian_final_approach and mobile_place_position is None:
                dense_place = retreat_path + dense_exit[1:] + dense_reconfigure[1:] + place_transit.path[1:]
            else:
                carry_path = place_transit.path if use_wrist_first_carry else place_planner.shortcut(
                    place_transit.path,
                    attempts=planner_options.get("shortcut_attempts", 180),
                )
                dense_place = place_planner.densify(
                    carry_path,
                    resolution=planner_options.get("output_resolution", 0.035),
                )
            if not (use_cartesian_final_approach and mobile_place_position is None):
                dense_place = retreat_path + dense_exit[1:] + dense_place[1:]
            post_translation_path = dense_place
            if mobile_place_position is not None:
                post_translation_path = dense_place[len(retreat_path) + len(dense_exit) - 1 :]
            if not _carried_box_collision_free(
                robot,
                carton,
                grasp.q,
                post_translation_path,
                carried_obstacles,
                allowed_obstacle_names={carton.name},
                box_margin=carried_box_clearance,
                extra_state_valid=posture_valid,
                carton_from_tool=carton_from_tool if mobile_place_position is not None else None,
                carton_contact_tolerance=carton_contact_tolerance,
                orientation_valid=carried_orientation_valid,
            ):
                failure_messages.append(f"candidate {candidate_index}: carried carton path collides")
                log(f"candidate {candidate_index}: carried carton path collides after {perf_counter() - candidate_t0:.1f}s")
                continue
        approach_path = contact_approach
        full = dense_transit + approach_path[1:] + dense_place[1:]
        grasp_index = len(dense_transit) + len(approach_path) - 2
        release_index = len(full) - 1
        release_retreat_index = len(full) - 1
        return_path: list[np.ndarray] = []
        if place_enabled and bool(planner_options.get("return_home_after_place", True)):
            empty_return_planner = RRTConnectPlanner(
                robot.joint_limits[:, 0],
                robot.joint_limits[:, 1],
                lambda q: robot.is_collision_free(q, scene.obstacles_without({carton.name})),
                step_size=planner_options.get("rrt_step_size", 0.22),
                edge_resolution=planner_options.get("rrt_edge_resolution", 0.065),
                max_iterations=planner_options.get("rrt_max_iterations", 4500),
                goal_bias=planner_options.get("rrt_goal_bias", 0.15),
                rng=rng,
            )
            return_plan = empty_return_planner.plan(full[-1], start_q)
            if not return_plan.success:
                failure_messages.append(f"candidate {candidate_index}: return-home RRT failed")
                log(f"candidate {candidate_index}: return-home RRT failed after {perf_counter() - candidate_t0:.1f}s")
                continue
            return_path = empty_return_planner.densify(
                empty_return_planner.shortcut(return_plan.path, attempts=planner_options.get("shortcut_attempts", 180)),
                resolution=planner_options.get("output_resolution", 0.035),
            )
            full.extend(return_path[1:])
        base_path = [pick_base[:3, 3].copy() for _ in dense_transit + approach_path[1:] + retreat_path[1:] + dense_exit[1:]]
        if mobile_base_path:
            full = dense_transit + approach_path[1:] + retreat_path[1:] + dense_exit[1:] + [carry_exit.q] * (len(mobile_base_path) - 1) + dense_place[len(retreat_path) + len(dense_exit) - 1 :]
            base_path += mobile_base_path[1:] + [place_base[:3, 3].copy() for _ in dense_place[len(retreat_path) + len(dense_exit) - 1 :]]
        else:
            base_path = [pick_base[:3, 3].copy() for _ in full]
        transit.path = smooth
        log(f"candidate {candidate_index}: success after {perf_counter() - candidate_t0:.1f}s")
        pick_plan = PickPlan(
            True,
            candidate,
            None,
            grasp,
            place,
            transit,
            place_transit,
            approach_path,
            dense_place,
            full,
            base_path,
            grasp_index,
            release_index,
            release_retreat_index,
            f"planned with candidate {candidate_index}",
            place_target_name if place_enabled else None,
        )
        if not bool(planner_options.get("optimize_across_grasp_candidates", False)):
            return pick_plan
        path_cost = float(
            np.linalg.norm(np.diff(np.asarray(full, dtype=float), axis=0), axis=1).sum()
        )
        surface_penalties = planner_options.get("place_surface_penalties", {})
        path_cost += float(surface_penalties.get(place_target_name, 0.0))
        if path_cost < best_pick_cost:
            best_pick_plan = pick_plan
            best_pick_cost = path_cost
        if surface_penalties and float(surface_penalties.get(place_target_name, 0.0)) <= min(
            float(value) for value in surface_penalties.values()
        ):
            # The first collision-free plan using a least-loaded conveyor is
            # sufficient for online load balancing; do not enumerate the
            # remaining grasp/wrist alternatives unnecessarily.
            return pick_plan

    if best_pick_plan is not None:
        return best_pick_plan

    return PickPlan(
        False,
        None,
        None,
        None,
        None,
        None,
        None,
        [],
        [],
        [],
        [],
        -1,
        -1,
        -1,
        "; ".join(failure_messages[-8:]) or "no candidates",
    )
