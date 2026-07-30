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
    place_target_name = str(planner_options.get("place_target", planner_options.get("place_obstacle", "conveyor_deck")))
    place_surface = next(
        (obstacle for obstacle in scene.obstacles if obstacle.name == place_target_name),
        None,
    )
    place_enabled = enable_place and place_surface is not None
    placement_requires_release_zone = bool(
        planner_options.get("place_requires_release_zone", place_target_name == planner_options.get("place_obstacle", "conveyor_deck"))
    )

    def placement_poses(grasp_tool_pose: np.ndarray) -> list[np.ndarray]:
        assert place_surface is not None
        edge_clearance = float(planner_options.get("conveyor_edge_clearance", 0.01))
        if placement_requires_release_zone:
            return conveyor_place_pose_candidates(
                place_surface,
                carton,
                grasp_tool_pose,
                robot.base_transform[:3, 3],
                edge_clearance=edge_clearance,
                allow_all_carton_faces=bool(planner_options.get("conveyor_allow_all_carton_faces", False)),
                require_front_release_zone=bool(planner_options.get("conveyor_require_front_release_zone", True)),
            )
        return surface_place_pose_candidates(
            place_surface,
            carton,
            grasp_tool_pose,
            edge_clearance=edge_clearance,
        )
    candidates = generate_suction_candidates(
        carton,
        robot_ref,
        face_modes=face_modes,
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
    use_cartesian_final_approach = bool(planner_options.get("use_cartesian_final_approach", False))
    use_wrist_first_carry = bool(planner_options.get("use_wrist_first_carry", False))
    conveyor_preplace_clearance = float(planner_options.get("conveyor_preplace_clearance", 0.30))

    failure_messages: list[str] = []
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
        if candidate_index > 0:
            seeds.append(rng.uniform(robot.joint_limits[:, 0] * 0.55, robot.joint_limits[:, 1] * 0.55))

        log(f"candidate {candidate_index}: solving pregrasp IK")
        pre = solve_ik_multistart(
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
        if not pre.success:
            failure_messages.append(f"candidate {candidate_index}: pregrasp IK failed")
            log(f"candidate {candidate_index}: pregrasp IK failed after {perf_counter() - candidate_t0:.1f}s")
            continue

        log(f"candidate {candidate_index}: solving grasp IK")
        grasp = solve_ik_multistart(
            robot,
            candidate.grasp_pose,
            seeds=[pre.q, start_q],
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
        if place_enabled and not place_poses:
            failure_messages.append(f"candidate {candidate_index}: carton does not fit conveyor")
            log(f"candidate {candidate_index}: carton does not fit conveyor")
            continue

        carton_from_tool = np.linalg.inv(robot.fk(grasp.q)) @ carton.world_from_local
        carried_obstacles = scene.obstacles_without(
            {carton.name, place_target_name}
        )
        place_robot_obstacles = scene.obstacles_without({carton.name})

        def carried_orientation_valid(carried_box: OBB) -> bool:
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

        log(f"candidate {candidate_index}: checking straight approach")
        approach_ok, approach_path = _edge_collision_free(
            robot,
            pre.q,
            grasp.q,
            all_obstacles,
            ignored={carton.name},
            resolution=planner_options.get("approach_resolution", 0.02),
            extra_state_valid=posture_valid,
        )
        if not approach_ok:
            failure_messages.append(f"candidate {candidate_index}: approach edge blocked")
            log(f"candidate {candidate_index}: approach blocked after {perf_counter() - candidate_t0:.1f}s")
            continue

        place = None
        place_transit = None
        selected_place_pose = None
        preplace_pose = None
        preplace = None
        carry_exit = None
        dense_place: list[np.ndarray] = []
        retreat_path: list[np.ndarray] = []
        dense_exit: list[np.ndarray] = []
        if place_enabled:
            carry_exit_pose = make_transform(
                candidate.pregrasp_pose[:3, :3],
                candidate.contact_point + candidate.outward_normal * planner_options.get("carry_exit_standoff", 0.68),
            )
            log(f"candidate {candidate_index}: solving carry-exit IK")
            carry_exit = solve_ik_multistart(
                robot,
                carry_exit_pose,
                seeds=[pre.q, grasp.q, start_q],
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
                log(f"candidate {candidate_index}: carry-exit IK failed after {perf_counter() - candidate_t0:.1f}s")
                continue

            if mobile_place_position is None:
                log(f"candidate {candidate_index}: evaluating {len(place_poses)} supported conveyor poses")
                place_results = [
                    (
                        pose,
                        solve_ik_multistart(
                            robot,
                            pose,
                            seeds=[carry_exit.q, grasp.q, pre.q, start_q],
                            obstacles=scene.obstacles_without({carton.name}),
                            random_restarts=planner_options.get("place_ik_random_restarts", 10),
                            rng=rng,
                            max_iterations=planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250)),
                            position_tolerance=planner_options.get("place_position_tolerance", 0.025),
                            orientation_tolerance=planner_options.get("place_orientation_tolerance", 0.22),
                            extra_state_valid=carried_goal_valid,
                        ),
                    )
                    for pose in place_poses
                ]
                feasible_places = [(pose, result) for pose, result in place_results if result.success]
                if not feasible_places:
                    failure_messages.append(f"candidate {candidate_index}: place IK failed")
                    log(f"candidate {candidate_index}: place IK failed after {perf_counter() - candidate_t0:.1f}s")
                    continue
                if use_cartesian_final_approach:
                    approach_directions = tuple(planner_options.get("conveyor_approach_directions", ("front", "top", "left", "right")))
                    preplace_results = []
                    for release_pose, release_ik in feasible_places:
                        for approach_direction, candidate_preplace_pose in _conveyor_preplace_poses(
                            release_pose,
                            conveyor_preplace_clearance,
                            approach_directions,
                            place_surface.rotation,
                        ):
                            preplace_ik = solve_ik_multistart(
                                robot,
                                candidate_preplace_pose,
                                seeds=[release_ik.q, carry_exit.q, grasp.q, pre.q, start_q],
                                obstacles=scene.obstacles_without({carton.name}),
                                random_restarts=planner_options.get("place_ik_random_restarts", 10),
                                rng=rng,
                                max_iterations=planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250)),
                                position_tolerance=planner_options.get("place_position_tolerance", 0.025),
                                orientation_tolerance=planner_options.get("place_orientation_tolerance", 0.22),
                                extra_state_valid=carried_goal_valid,
                            )
                            if preplace_ik.success:
                                preplace_results.append((release_pose, release_ik, approach_direction, candidate_preplace_pose, preplace_ik))
                    if not preplace_results:
                        failure_messages.append(f"candidate {candidate_index}: conveyor preplace IK failed")
                        log(f"candidate {candidate_index}: conveyor preplace IK failed after {perf_counter() - candidate_t0:.1f}s")
                        continue
                    selected_place_pose, place, approach_direction, preplace_pose, preplace = min(
                        preplace_results,
                        key=lambda item: float(np.linalg.norm(item[1].q - carry_exit.q) + np.linalg.norm(item[4].q - item[1].q)),
                    )
                    log(f"candidate {candidate_index}: selected conveyor approach={approach_direction}")
                else:
                    selected_place_pose, place = min(
                        feasible_places,
                        key=lambda item: float(np.linalg.norm(item[1].q - carry_exit.q)),
                    )

        log(f"candidate {candidate_index}: planning transit RRT")
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
        transit = planner.plan(start_q, pre.q)
        if not transit.success:
            failure_messages.append(f"candidate {candidate_index}: transit RRT failed")
            log(f"candidate {candidate_index}: transit RRT failed after {perf_counter() - candidate_t0:.1f}s")
            continue
        smooth = planner.shortcut(transit.path, attempts=planner_options.get("shortcut_attempts", 180))
        dense_transit = planner.densify(smooth, resolution=planner_options.get("output_resolution", 0.035))
        mobile_base_path: list[np.ndarray] = []
        if place_enabled:
            log(f"candidate {candidate_index}: planning place RRT")
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
            retreat_ok, retreat_path = _edge_collision_free(
                robot,
                grasp.q,
                pre.q,
                place_robot_obstacles,
                ignored={carton.name},
                resolution=planner_options.get("approach_resolution", 0.02),
                extra_state_valid=posture_valid,
            )
            if not retreat_ok or not _carried_box_collision_free(
                robot,
                carton,
                grasp.q,
                retreat_path,
                carried_obstacles,
                box_margin=carried_box_clearance,
                extra_state_valid=posture_valid,
                carton_contact_tolerance=carton_contact_tolerance,
                orientation_valid=carried_orientation_valid,
            ):
                failure_messages.append(f"candidate {candidate_index}: retreat with carton blocked")
                log(f"candidate {candidate_index}: retreat with carton blocked after {perf_counter() - candidate_t0:.1f}s")
                continue
            if use_cartesian_carry:
                exit_transit = _plan_cartesian_carry(
                    robot,
                    carton,
                    grasp.q,
                    pre.q,
                    carry_exit_pose,
                    place_robot_obstacles,
                    carried_obstacles,
                    carton_from_tool,
                    carried_orientation_valid,
                    rng,
                    position_step=float(planner_options.get("cartesian_carry_position_step", 0.10)),
                    orientation_step=float(planner_options.get("cartesian_carry_orientation_step", 0.16)),
                    joint_resolution=float(planner_options.get("output_resolution", 0.035)),
                )
            elif use_wrist_first_carry:
                exit_transit = _plan_wrist_first_carry(
                    pre.q,
                    carry_exit.q,
                    place_valid,
                    resolution=float(planner_options.get("approach_resolution", 0.02)),
                )
                if not exit_transit.success:
                    exit_transit = place_planner.plan(pre.q, carry_exit.q)
            else:
                exit_transit = place_planner.plan(pre.q, carry_exit.q)
            if not exit_transit.success:
                failure_messages.append(f"candidate {candidate_index}: {exit_transit.message}")
                log(f"candidate {candidate_index}: {exit_transit.message} after {perf_counter() - candidate_t0:.1f}s")
                continue
            if use_cartesian_carry:
                dense_exit = exit_transit.path
            else:
                dense_exit = place_planner.densify(
                    place_planner.shortcut(exit_transit.path, attempts=planner_options.get("shortcut_attempts", 180)),
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
                    seeds=[carry_exit.q, grasp.q, pre.q],
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
                reconfigure_transit = place_planner.plan(carry_exit.q, preplace.q)
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
                    place_transit = place_planner.plan(carry_exit.q, place.q)
            else:
                place_transit = place_planner.plan(carry_exit.q, place.q)
            if not place_transit.success:
                failure_messages.append(f"candidate {candidate_index}: {place_transit.message}")
                log(f"candidate {candidate_index}: {place_transit.message} after {perf_counter() - candidate_t0:.1f}s")
                continue
            if use_cartesian_final_approach and mobile_place_position is None:
                dense_place = retreat_path + dense_exit[1:] + dense_reconfigure[1:] + place_transit.path[1:]
            else:
                dense_place = place_planner.densify(
                    place_planner.shortcut(place_transit.path, attempts=planner_options.get("shortcut_attempts", 180)),
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
        full = dense_transit + approach_path[1:] + dense_place[1:]
        grasp_index = len(dense_transit) + len(approach_path) - 2
        release_index = len(full) - 1
        release_retreat_path: list[np.ndarray] = []
        if place_enabled and use_cartesian_final_approach and mobile_place_position is None:
            # Retrace the direct preplace-to-release segment after releasing.
            # The carton is no longer attached, so this is an empty-tool exit.
            release_retreat_path = list(reversed(place_transit.path))
            full.extend(release_retreat_path[1:])
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
        return PickPlan(
            True,
            candidate,
            pre,
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
        )

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
