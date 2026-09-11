"""Analytic suction-grasp generation and pick-motion planning."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Callable, Sequence

import numpy as np

from .geometry import OBB, make_tool_rotation, make_transform, normalize, rotation_vector_from_matrix
from .ik import IKResult, solve_ik_multistart
from .planner import PlanResult, RRTConnectPlanner
from .robot import RobotBackend
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
    sealed_cup_indices: tuple[int, ...] = ()
    sealed_cups_per_zone: tuple[int, ...] = ()


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


@dataclass(frozen=True)
class DestackClearance:
    """Geometry-derived constraint for withdrawing a carton from its neighbours."""

    requires_straight_withdrawal: bool
    withdrawal_distance_m: float
    enclosed_local_axes: tuple[int, ...] = ()
    blocking_carton_names: tuple[str, ...] = ()


def generate_suction_candidates(
    carton: OBB,
    robot_reference_point: np.ndarray,
    standoff: float = 0.18,
    cup_clearance: float = 0.015,
    grid_fraction: float = 0.42,
    contact_grid_fractions: Sequence[float] | None = None,
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
    offsets = (
        tuple(float(value) for value in contact_grid_fractions)
        if contact_grid_fractions is not None
        else (-grid_fraction, 0.0, grid_fraction)
    )
    if not offsets or any(not np.isfinite(value) or abs(value) >= 1.0 for value in offsets):
        raise ValueError("contact grid fractions must be finite values inside (-1, 1)")
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
    """Generate tool-roll alternatives without changing contact or normal.

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
                    sealed_cup_indices=candidate.sealed_cup_indices,
                    sealed_cups_per_zone=candidate.sealed_cups_per_zone,
                )
            )
    return expanded


def suction_footprint_fits_face(
    candidate: SuctionGraspCandidate,
    carton: OBB,
    footprint_size_m: Sequence[float],
    *,
    edge_margin_m: float = 0.0,
) -> bool:
    """Return whether a rectangular suction seal lies completely on one face.

    The two footprint dimensions follow the tool X/Y axes. Projection keeps
    the test correct for arbitrary wrist roll and rotated cartons.
    """
    size = np.asarray(footprint_size_m, dtype=float)
    if size.shape != (2,) or not np.all(np.isfinite(size)) or np.any(size <= 0.0):
        raise ValueError("suction footprint must contain two finite positive dimensions")
    if not np.isfinite(edge_margin_m) or edge_margin_m < 0.0:
        raise ValueError("suction edge margin must be finite and non-negative")

    normal_local = carton.rotation.T @ np.asarray(candidate.outward_normal, dtype=float)
    face_axis = int(np.argmax(np.abs(normal_local)))
    if abs(float(normal_local[face_axis])) < 1.0 - 1e-6:
        return False
    plane_axes = [axis for axis in range(3) if axis != face_axis]
    contact_local = carton.to_local(candidate.contact_point)
    tool_x = candidate.grasp_pose[:3, 0]
    tool_y = candidate.grasp_pose[:3, 1]
    half_x, half_y = 0.5 * size
    for axis in plane_axes:
        face_direction = carton.rotation[:, axis]
        projected_half_extent = (
            half_x * abs(float(face_direction @ tool_x))
            + half_y * abs(float(face_direction @ tool_y))
        )
        available = float(carton.half_extents[axis]) - float(edge_margin_m)
        if available < 0.0 or abs(float(contact_local[axis])) + projected_half_extent > available + 1e-9:
            return False
    return True


def suction_cup_seal_indices(
    candidate: SuctionGraspCandidate,
    carton: OBB,
    cup_centers_m: Sequence[Sequence[float]],
    cup_radius_m: float,
    *,
    edge_margin_m: float = 0.0,
) -> tuple[int, ...]:
    """Return cups whose complete circular sealing lip lies on the selected face."""
    centers = np.asarray(cup_centers_m, dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 2 or not np.all(np.isfinite(centers)):
        raise ValueError("cup centers must have finite shape (N, 2)")
    if not np.isfinite(cup_radius_m) or cup_radius_m <= 0.0:
        raise ValueError("cup radius must be finite and positive")
    if not np.isfinite(edge_margin_m) or edge_margin_m < 0.0:
        raise ValueError("suction edge margin must be finite and non-negative")

    normal_local = carton.rotation.T @ np.asarray(candidate.outward_normal, dtype=float)
    face_axis = int(np.argmax(np.abs(normal_local)))
    if abs(float(normal_local[face_axis])) < 1.0 - 1e-6:
        return ()
    plane_axes = [axis for axis in range(3) if axis != face_axis]
    tool_u = candidate.grasp_pose[:3, 0]
    tool_v = candidate.grasp_pose[:3, 1]
    sealed: list[int] = []
    for index, (offset_u, offset_v) in enumerate(centers):
        cup_world = candidate.contact_point + tool_u * offset_u + tool_v * offset_v
        cup_local = carton.to_local(cup_world)
        if all(
            abs(float(cup_local[axis])) + cup_radius_m + edge_margin_m
            <= float(carton.half_extents[axis]) + 1e-9
            for axis in plane_axes
        ):
            sealed.append(index)
    return tuple(sealed)


def filter_suction_candidates_by_seal(
    candidates: Sequence[SuctionGraspCandidate],
    carton: OBB,
    planner_options: dict,
) -> list[SuctionGraspCandidate]:
    """Apply either discrete-cup seal coverage or the legacy rectangle test."""
    layout = planner_options.get("suction_cup_layout")
    edge_margin = float(planner_options.get("suction_edge_margin_m", 0.0))
    if layout is None:
        footprint = planner_options.get("suction_footprint_size_m")
        if footprint is None:
            return list(candidates)
        return [
            candidate
            for candidate in candidates
            if suction_footprint_fits_face(
                candidate, carton, footprint, edge_margin_m=edge_margin
            )
        ]

    rows = int(layout.get("rows", 0))
    columns = int(layout.get("columns", 0))
    pitch = np.asarray(layout.get("pitch_m", []), dtype=float)
    cup_radius = float(layout.get("cup_radius_m", 0.0))
    zone_count = int(layout.get("zone_count", 1))
    minimum_sealed = int(layout.get("minimum_sealed_cups", rows * columns))
    if rows <= 0 or columns <= 0 or pitch.shape != (2,) or np.any(pitch <= 0.0):
        raise ValueError("suction cup layout rows, columns, and pitch must be positive")
    if zone_count <= 0 or columns % zone_count != 0:
        raise ValueError("suction cup columns must divide evenly across zones")
    width_offsets = (np.arange(rows) - 0.5 * (rows - 1)) * pitch[0]
    length_offsets = (np.arange(columns) - 0.5 * (columns - 1)) * pitch[1]
    centers = np.asarray(
        [(width, length) for length in length_offsets for width in width_offsets],
        dtype=float,
    )
    columns_per_zone = columns // zone_count
    accepted: list[SuctionGraspCandidate] = []
    for candidate in candidates:
        sealed = suction_cup_seal_indices(
            candidate,
            carton,
            centers,
            cup_radius,
            edge_margin_m=edge_margin,
        )
        zone_counts = [0] * zone_count
        for cup_index in sealed:
            column = cup_index // rows
            zone_counts[min(column // columns_per_zone, zone_count - 1)] += 1
        candidate.sealed_cup_indices = sealed
        candidate.sealed_cups_per_zone = tuple(zone_counts)
        if len(sealed) >= minimum_sealed:
            accepted.append(candidate)
    return accepted


def filter_suction_candidates_by_tool_clearance(
    candidates: Sequence[SuctionGraspCandidate],
    obstacles: Sequence[OBB],
    tool_size_m: Sequence[float] | None,
    *,
    target_name: str,
    margin_m: float = 0.0,
) -> list[SuctionGraspCandidate]:
    """Reject contact poses where the rigid suction-head envelope overlaps a neighbour."""
    if tool_size_m is None:
        return list(candidates)
    size = np.asarray(tool_size_m, dtype=float)
    if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0.0):
        raise ValueError("tool collision size must contain three finite positive dimensions")
    if not np.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("tool collision margin must be finite and non-negative")
    accepted: list[SuctionGraspCandidate] = []
    local_center = np.asarray([0.0, 0.0, -0.5 * size[2]], dtype=float)
    for candidate in candidates:
        pose = candidate.grasp_pose
        envelope = OBB(
            center=pose[:3, :3] @ local_center + pose[:3, 3],
            half_extents=0.5 * size,
            rotation=pose[:3, :3],
            name="tool_envelope",
            category="robot",
        )
        if any(
            obstacle.name != target_name
            and envelope.intersects_obb(obstacle, margin=margin_m)
            for obstacle in obstacles
        ):
            continue
        accepted.append(candidate)
    return accepted


def _edge_collision_free(
    robot: RobotBackend,
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


def _path_within_task_space_tube(
    robot: RobotBackend,
    path: Sequence[np.ndarray],
    *,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> bool:
    """Check that a joint interpolation stays near its Cartesian chord."""
    if len(path) < 2:
        return True
    poses = [robot.fk(np.asarray(q, dtype=float)) for q in path]
    start = poses[0]
    end = poses[-1]
    chord = end[:3, 3] - start[:3, 3]
    chord_norm_sq = float(chord @ chord)
    for pose in poses:
        offset = pose[:3, 3] - start[:3, 3]
        fraction = 0.0 if chord_norm_sq < 1e-12 else float(np.clip((offset @ chord) / chord_norm_sq, 0.0, 1.0))
        expected = start[:3, 3] + fraction * chord
        if float(np.linalg.norm(pose[:3, 3] - expected)) > position_tolerance_m:
            return False
        rotation_error = float(
            np.linalg.norm(rotation_vector_from_matrix(pose[:3, :3] @ start[:3, :3].T))
        )
        if rotation_error > orientation_tolerance_rad:
            return False
    return True


def _nearest_equivalent_joint_vector(
    robot: RobotBackend,
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


def _continuous_equivalent_joint_path(
    robot: RobotBackend,
    path: Sequence[np.ndarray],
) -> list[np.ndarray]:
    """Unwrap a revolute path without leaving the robot's declared limits."""
    if not path:
        return []
    continuous = [np.asarray(path[0], dtype=float).copy()]
    for q in path[1:]:
        continuous.append(_nearest_equivalent_joint_vector(robot, q, continuous[-1]))
    return continuous


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
    robot: RobotBackend,
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


def _carried_orientation_policy_valid(
    reference_carton: OBB,
    carried_box: OBB,
    scene: TrailerScene,
    planner_options: dict,
) -> bool:
    """Apply the configured post-destack wrist-reorientation policy.

    Straight/adaptive destack segments are certified separately by their
    task-space tube.  Once that segment is complete, layouts that explicitly
    allow free-space reorientation may use cleared volume inside the trailer;
    the exact robot, carton, trailer, and tool collision checks remain active.
    Keeping this policy shared prevents offline cache validation from applying
    a stricter, stale orientation rule than the planner used.
    """
    if bool(planner_options.get("allow_free_space_reorientation", False)):
        return True
    clearance_zone_max_x = planner_options.get("clearance_zone_max_x")
    if (
        clearance_zone_max_x is not None
        and carried_box.center[0] <= float(clearance_zone_max_x)
    ):
        return True
    return _in_trailer_orientation_valid(
        reference_carton,
        carried_box,
        scene,
        planner_options.get("max_in_trailer_carton_tilt_deg"),
    )


def _carried_task_envelope_valid(
    reference_carton: OBB,
    carried_box: OBB,
    *,
    maximum_rotation_degrees: float | None,
    maximum_lift_m: float | None,
) -> bool:
    """Enforce task-space carry style without prescribing spatial waypoints."""
    if maximum_rotation_degrees is not None:
        rotation_delta = carried_box.rotation @ reference_carton.rotation.T
        rotation_degrees = float(
            np.degrees(np.linalg.norm(rotation_vector_from_matrix(rotation_delta)))
        )
        if rotation_degrees > float(maximum_rotation_degrees):
            return False
    if maximum_lift_m is not None and (
        carried_box.center[2] > reference_carton.center[2] + float(maximum_lift_m)
    ):
        return False
    return True


def _obb_projection_interval(box: OBB, axis: np.ndarray) -> tuple[float, float]:
    direction = normalize(np.asarray(axis, dtype=float))
    radius = float(np.abs(box.rotation.T @ direction) @ box.half_extents)
    center = float(direction @ box.center)
    return center - radius, center + radius


def _adaptive_destack_clearance(
    carton: OBB,
    obstacles: Sequence[OBB],
    outward_normal: np.ndarray,
    *,
    clearance_m: float,
) -> DestackClearance:
    """Require a straight withdrawal when local geometry encloses both sides.

    The test is local and scale-aware: a copy of the target carton is shifted
    by the requested collision clearance in each direction of both face-plane
    axes.  Cartons and trailer structure can jointly block both signs; an
    outer stack column beside a wall is therefore not misclassified as open.
    The required normal withdrawal is computed from carton projection
    intervals where possible, rather than a model-specific fixed standoff.
    """
    if not np.isfinite(clearance_m) or clearance_m <= 0.0:
        raise ValueError("destack clearance must be finite and positive")
    normal = normalize(np.asarray(outward_normal, dtype=float))
    normal_local = carton.rotation.T @ normal
    face_axis = int(np.argmax(np.abs(normal_local)))
    tangent_axes = [axis for axis in range(3) if axis != face_axis]
    neighbours = [
        obstacle
        for obstacle in obstacles
        if obstacle.name != carton.name
        and obstacle.category in {"carton", "trailer"}
        and obstacle.name not in {"trailer_floor", "trailer_roof"}
    ]
    vertical = np.array([0.0, 0.0, 1.0])
    target_bottom = _obb_projection_interval(carton, vertical)[0]
    support_blockers: list[OBB] = []
    for neighbour in neighbours:
        if neighbour.category != "carton" or neighbour.center[2] >= carton.center[2]:
            continue
        neighbour_top = _obb_projection_interval(neighbour, vertical)[1]
        if abs(target_bottom - neighbour_top) > clearance_m + 1e-9:
            continue
        horizontal_overlap = True
        for horizontal in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])):
            target_interval = _obb_projection_interval(carton, horizontal)
            neighbour_interval = _obb_projection_interval(neighbour, horizontal)
            if min(target_interval[1], neighbour_interval[1]) - max(
                target_interval[0], neighbour_interval[0]
            ) <= 1e-9:
                horizontal_overlap = False
                break
        if horizontal_overlap:
            support_blockers.append(neighbour)
    enclosed_axes: list[int] = []
    blockers: dict[str, OBB] = {}
    for tangent_axis in tangent_axes:
        tangent = carton.rotation[:, tangent_axis]
        # A centimetre-scale nudge only detects touching neighbours and can
        # misclassify a narrow wall gap as an escape route.  Probe one full
        # carton span: lateral escape is useful only if the carried carton can
        # clear its opposite-side neighbour, not merely move a few millimetres.
        probe_distance = 2.0 * float(carton.half_extents[tangent_axis]) + float(
            clearance_m
        )
        directional_blockers: list[list[OBB]] = []
        for sign in (-1.0, 1.0):
            shifted = OBB(
                center=carton.center + sign * probe_distance * tangent,
                half_extents=carton.half_extents,
                rotation=carton.rotation,
                name=f"{carton.name}_destack_probe",
                category="carton",
            )
            directional_blockers.append(
                [
                    neighbour
                    for neighbour in neighbours
                    if sign * float((neighbour.center - carton.center) @ tangent) > 1e-9
                    # Exclude mere edge/face contact on another axis; only
                    # positive-volume overlap after the lateral probe counts.
                    and shifted.intersects_obb(neighbour, margin=-1e-9)
                ]
            )
        if directional_blockers[0] and directional_blockers[1]:
            enclosed_axes.append(tangent_axis)
            for neighbour in directional_blockers[0] + directional_blockers[1]:
                blockers[neighbour.name] = neighbour

    for neighbour in support_blockers:
        blockers[neighbour.name] = neighbour

    if not enclosed_axes and not support_blockers:
        return DestackClearance(False, 0.0)

    # A side wall establishes that lateral escape is unavailable, but its
    # trailer-length projection must not force withdrawal all the way outside
    # the door when clearing the adjacent carton is sufficient to turn inward.
    # If no carton participates, fall back to all structural blockers.
    withdrawal_blockers = [
        obstacle for obstacle in blockers.values() if obstacle.category == "carton"
    ] or list(blockers.values())
    target_min, _ = _obb_projection_interval(carton, normal)
    required = max(
        _obb_projection_interval(neighbour, normal)[1] - target_min + clearance_m
        for neighbour in withdrawal_blockers
    )
    return DestackClearance(
        True,
        max(0.0, float(required)),
        tuple(enclosed_axes),
        tuple(sorted(blockers)),
    )


def _reorientation_clearance_standoff(
    carton: OBB,
    outward_normal: np.ndarray,
    base_standoff_m: float,
    *,
    clearance_m: float,
) -> float:
    """Return a geometry-derived standoff at which the carton may rotate freely.

    ``base_standoff_m`` clears the carton in its current orientation. The
    additional distance replaces the current face-normal radius with the
    carton's orientation-independent bounding-sphere radius. It therefore
    varies with carton dimensions and grasp face instead of prescribing a
    model-specific withdrawal waypoint.
    """
    if not np.isfinite(base_standoff_m) or base_standoff_m < 0.0:
        raise ValueError("base standoff must be finite and non-negative")
    if not np.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("reorientation clearance must be finite and non-negative")
    normal = normalize(np.asarray(outward_normal, dtype=float))
    face_normal_radius = float(np.abs(carton.rotation.T @ normal) @ carton.half_extents)
    rotation_invariant_radius = float(np.linalg.norm(carton.half_extents))
    return float(
        base_standoff_m
        + max(0.0, rotation_invariant_radius - face_normal_radius)
        + clearance_m
    )


def _reorientation_wall_clearance_offsets(
    carried_box: OBB,
    obstacles: Sequence[OBB],
    outward_normal: np.ndarray,
    *,
    clearance_m: float,
) -> list[np.ndarray]:
    """Return shortest-first shifts from wall clearance to free-space centre."""
    if not np.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("wall clearance must be finite and non-negative")
    normal = normalize(np.asarray(outward_normal, dtype=float))
    face_axis = int(np.argmax(np.abs(carried_box.rotation.T @ normal)))
    radius = float(np.linalg.norm(carried_box.half_extents))
    nearest_offset = np.zeros(3, dtype=float)
    centered_offset = np.zeros(3, dtype=float)
    for axis in range(3):
        if axis == face_axis:
            continue
        tangent = carried_box.rotation[:, axis]
        # Side-wall correction is horizontal. Vertical movement remains under
        # the independent lift/roof/floor task constraints.
        if abs(float(tangent @ np.array([0.0, 0.0, 1.0]))) > 0.5:
            continue
        center_projection = float(carried_box.center @ tangent)
        lower = -np.inf
        upper = np.inf
        for obstacle in obstacles:
            if obstacle.category != "trailer" or obstacle.name in {
                "trailer_floor",
                "trailer_roof",
            }:
                continue
            thin_axis = int(np.argmin(obstacle.half_extents))
            slab_normal = obstacle.rotation[:, thin_axis]
            if abs(float(slab_normal @ tangent)) < 0.9:
                continue
            obstacle_min, obstacle_max = _obb_projection_interval(obstacle, tangent)
            obstacle_center = float(obstacle.center @ tangent)
            if obstacle_center >= center_projection:
                upper = min(upper, obstacle_min - radius - clearance_m)
            else:
                lower = max(lower, obstacle_max + radius + clearance_m)
        if lower <= upper:
            nearest = float(np.clip(center_projection, lower, upper))
            centered = (
                0.5 * (lower + upper)
                if np.isfinite(lower) and np.isfinite(upper)
                else nearest
            )
            nearest_offset += (nearest - center_projection) * tangent
            centered_offset += (centered - center_projection) * tangent
    offsets: list[np.ndarray] = []
    for fraction in (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0):
        candidate = nearest_offset + fraction * (centered_offset - nearest_offset)
        if not offsets or not np.allclose(candidate, offsets[-1], atol=1e-9, rtol=0.0):
            offsets.append(candidate)
    return offsets


def _reorientation_wall_clearance_offset(
    carried_box: OBB,
    obstacles: Sequence[OBB],
    outward_normal: np.ndarray,
    *,
    clearance_m: float,
) -> np.ndarray:
    """Compute the smallest in-plane shift that gives rotation room at side walls."""
    return _reorientation_wall_clearance_offsets(
        carried_box,
        obstacles,
        outward_normal,
        clearance_m=clearance_m,
    )[0]


def _carried_box_collision_free(
    robot: RobotBackend,
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
            obstacle_margin = _carried_obstacle_margin(obstacle, box_margin, carton_contact_tolerance)
            if carried_box.intersects_obb(obstacle, margin=obstacle_margin):
                return False
        for capsule in robot.link_capsules(q):
            if capsule.name == "tool":
                continue
            if capsule.collides_obb(carried_box, margin=robot_margin):
                return False
    return True


def _carried_box_state_valid(
    robot: RobotBackend,
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
        obstacle_margin = _carried_obstacle_margin(obstacle, box_margin, carton_contact_tolerance)
        if carried_box.intersects_obb(obstacle, margin=obstacle_margin):
            return False
    for capsule in robot.link_capsules(q):
        if capsule.name == "tool":
            continue
        if capsule.collides_obb(carried_box, margin=robot_margin):
            return False
    return True


def _carried_obstacle_margin(obstacle: OBB, box_margin: float, contact_tolerance: float) -> float:
    """Permit existing support contact without permitting penetration.

    A floor carton starts exactly on ``trailer_floor``. Expanding the carried
    carton by the normal clearance margin made that valid initial support
    contact look like a collision, so the first upward destack sample could
    never be solved. Cartons and the floor use a small negative tolerance;
    walls, roof, robot and other obstacles retain the requested clearance.
    """
    if obstacle.category == "carton" or obstacle.name == "trailer_floor":
        return -float(contact_tolerance)
    if obstacle.name.startswith("conveyor_"):
        # A commanded release remains above the belt by the controlled drop
        # height. Keep every belt in the carried-path collision set so an
        # attached carton cannot skim it, but do not double-inflate the carton
        # and deck by the much larger trailer-wall clearance.
        return min(float(box_margin), float(contact_tolerance))
    return float(box_margin)


def _mobile_base_translation_safe(
    robot: RobotBackend,
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


def _joint_motion_time_cost(
    source: np.ndarray,
    target: np.ndarray,
    velocity_limits_rad_s: Sequence[float] | None,
) -> float:
    delta = np.abs(np.asarray(target, dtype=float) - np.asarray(source, dtype=float))
    if velocity_limits_rad_s is None:
        return float(np.linalg.norm(delta))
    limits = np.asarray(velocity_limits_rad_s, dtype=float)
    if limits.shape != delta.shape or np.any(~np.isfinite(limits)) or np.any(limits <= 0.0):
        raise ValueError("joint velocity limits must match the joint vector and be positive")
    # Parallel joint motion is controller-time limited by the slowest axis.
    # FANUC wrist axes are faster, so useful post-destack wrist reorientation
    # is not incorrectly penalised like equal shoulder motion.
    return float(np.max(delta / limits))


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
    robot: RobotBackend,
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
    carton_contact_tolerance: float = 0.001,
    box_margin: float = 0.01,
    position_step: float = 0.10,
    orientation_step: float = 0.16,
    joint_resolution: float = 0.035,
    ik_iterations: int = 180,
    position_tolerance: float = 0.025,
    orientation_tolerance: float = 0.18,
    ik_random_restarts: int = 2,
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
            box_margin=box_margin,
            carton_from_tool=carton_from_tool,
            carton_contact_tolerance=carton_contact_tolerance,
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
            random_restarts=ik_random_restarts,
            rng=rng,
            max_iterations=ik_iterations,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            extra_state_valid=carried_valid,
        )
        if not result.success:
            closest_q = np.asarray(result.q, dtype=float)
            robot_valid = robot.is_collision_free(closest_q, robot_obstacles)
            posture_valid = extra_state_valid is None or extra_state_valid(closest_q)
            closest_box = _carried_box_at(robot, carton, carton_from_tool, closest_q)
            orientation_ok = orientation_valid is None or orientation_valid(closest_box)
            colliding_obstacles = [
                obstacle.name
                for obstacle in carried_obstacles
                if closest_box.intersects_obb(
                    obstacle,
                    margin=_carried_obstacle_margin(
                        obstacle,
                        box_margin,
                        carton_contact_tolerance,
                    ),
                )
            ]
            colliding_links = [
                capsule.name
                for capsule in robot.link_capsules(closest_q)
                if capsule.name != "tool"
                and capsule.collides_obb(closest_box, margin=0.003)
            ]
            return PlanResult(
                False,
                [],
                index,
                f"Cartesian carry IK failed at waypoint {index}/{steps}: "
                f"{result.message}; position_error={result.position_error:.6f}m; "
                f"orientation_error={result.orientation_error:.6f}rad; "
                f"robot_collision_free={robot_valid}; posture_valid={posture_valid}; "
                f"orientation_valid={orientation_ok}; "
                f"carried_obstacle_collisions={colliding_obstacles}; "
                f"robot_carton_collisions={colliding_links}",
            )
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


def _plan_cartesian_tool_translation(
    robot: RobotBackend,
    start_q: np.ndarray,
    target_position: np.ndarray,
    robot_obstacles: Sequence[OBB],
    rng: np.random.Generator,
    *,
    extra_state_valid: Callable[[np.ndarray], bool] | None = None,
    position_step: float = 0.03,
    joint_resolution: float = 0.035,
) -> PlanResult:
    """Translate an empty tool in a straight line with fixed orientation."""
    start_pose = robot.fk(start_q)
    target_position = np.asarray(target_position, dtype=float)
    if target_position.shape != (3,) or not np.all(np.isfinite(target_position)):
        raise ValueError("Cartesian tool target position must contain three finite values")
    if not np.isfinite(position_step) or position_step <= 0.0:
        raise ValueError("Cartesian tool position step must be finite and positive")
    distance = float(np.linalg.norm(target_position - start_pose[:3, 3]))
    steps = max(1, int(np.ceil(distance / position_step)))
    path = [np.asarray(start_q, dtype=float).copy()]
    for index in range(1, steps + 1):
        fraction = index / steps
        target = start_pose.copy()
        target[:3, 3] = (
            (1.0 - fraction) * start_pose[:3, 3] + fraction * target_position
        )
        result = solve_ik_multistart(
            robot,
            target,
            seeds=[path[-1]],
            obstacles=robot_obstacles,
            random_restarts=2,
            rng=rng,
            max_iterations=320,
            position_tolerance=0.008,
            orientation_tolerance=0.04,
            extra_state_valid=extra_state_valid,
        )
        if not result.success:
            return PlanResult(
                False,
                [],
                index,
                f"Cartesian empty-tool IK failed at waypoint {index}/{steps}",
            )
        edge_ok, edge = _edge_collision_free(
            robot,
            path[-1],
            result.q,
            robot_obstacles,
            resolution=joint_resolution,
            extra_state_valid=extra_state_valid,
        )
        if not edge_ok:
            return PlanResult(
                False,
                [],
                index,
                f"Cartesian empty-tool edge blocked at waypoint {index}/{steps}",
            )
        path.extend(edge[1:])
    return PlanResult(True, path, steps, "Cartesian empty-tool translation")


def post_release_escape_target(
    robot: RobotBackend,
    release_q: np.ndarray,
    place_surface: str | None,
    planner_options: dict,
    *,
    normal_disengage_distance_m: float,
    vertical_lift_distance_m: float,
    tool_collision_obbs_provider: Callable[[np.ndarray], Sequence[OBB]] | None = None,
) -> tuple[np.ndarray, dict[str, object]]:
    """Derive a belt-aware empty-tool escape target after payload release.

    A continuously moving conveyor starts translating the carton immediately.
    Moving only along the suction normal can therefore leave the wide wrist and
    gripper envelope in the carton's swept path. When a surface direction is
    available, combine normal disengagement, vertical lift, and motion opposite
    the belt direction in one Cartesian translation. The lateral distance is
    derived from the actual rigid-tool collision envelope projected onto that
    direction, plus the configured collision clearance.
    """

    release_q = np.asarray(release_q, dtype=float)
    normal_distance = float(normal_disengage_distance_m)
    lift_distance = float(vertical_lift_distance_m)
    if not np.isfinite(normal_distance) or normal_distance < 0.0:
        raise ValueError("post-release normal disengage distance must be finite and non-negative")
    if not np.isfinite(lift_distance) or lift_distance < 0.0:
        raise ValueError("post-release vertical lift distance must be finite and non-negative")

    release_pose = robot.fk(release_q)
    target = (
        release_pose[:3, 3]
        - normal_distance * release_pose[:3, 2]
        + np.array([0.0, 0.0, lift_distance], dtype=float)
    )
    metadata: dict[str, object] = {
        "model": "normal_disengage_then_vertical_lift_v1",
        "normal_disengage_distance_m": normal_distance,
        "vertical_lift_distance_m": lift_distance,
        "conveyor_escape_enabled": False,
        "conveyor_escape_distance_m": 0.0,
    }

    direction_map = planner_options.get("post_release_surface_directions_world", {})
    if place_surface is None or not isinstance(direction_map, dict):
        return target, metadata
    configured_direction = direction_map.get(str(place_surface))
    if configured_direction is None:
        return target, metadata
    belt_direction = np.asarray(configured_direction, dtype=float)
    if belt_direction.shape != (3,) or not np.all(np.isfinite(belt_direction)):
        raise ValueError("post-release conveyor direction must contain three finite values")
    direction_norm = float(np.linalg.norm(belt_direction))
    if direction_norm <= 1e-12:
        raise ValueError("post-release conveyor direction must be non-zero")
    belt_direction /= direction_norm
    escape_direction = -belt_direction

    provider = tool_collision_obbs_provider or getattr(
        robot, "tool_collision_obbs", lambda _q: []
    )
    tool_obbs = list(provider(release_q))
    if not tool_obbs:
        legacy_obb = getattr(robot, "tool_collision_obb", lambda _q: None)(release_q)
        if legacy_obb is not None:
            tool_obbs = [legacy_obb]
    tool_origin = release_pose[:3, 3]
    projected_tool_extent = max(
        (
            float(np.max(np.abs((tool_obb.corners() - tool_origin) @ escape_direction)))
            for tool_obb in tool_obbs
        ),
        default=0.0,
    )
    clearance = float(
        planner_options.get(
            "post_release_conveyor_escape_clearance_m",
            planner_options.get("carried_box_clearance", 0.0),
        )
    )
    if not np.isfinite(clearance) or clearance < 0.0:
        raise ValueError("post-release conveyor escape clearance must be finite and non-negative")
    minimum_distance = float(
        planner_options.get("post_release_conveyor_escape_minimum_m", 0.0)
    )
    if not np.isfinite(minimum_distance) or minimum_distance < 0.0:
        raise ValueError("post-release conveyor escape minimum must be finite and non-negative")
    escape_distance = max(minimum_distance, projected_tool_extent + clearance)
    if escape_distance <= 0.0:
        return target, metadata

    target += escape_distance * escape_direction
    metadata.update(
        {
            "model": "normal_vertical_and_opposed_conveyor_diagonal_v2",
            "conveyor_escape_enabled": True,
            "conveyor_surface": str(place_surface),
            "conveyor_direction_world": belt_direction.tolist(),
            "conveyor_escape_direction_world": escape_direction.tolist(),
            "projected_tool_extent_m": projected_tool_extent,
            "conveyor_escape_clearance_m": clearance,
            "conveyor_escape_distance_m": escape_distance,
        }
    )
    return target, metadata


def adaptive_post_release_normal_disengage_distance(
    robot: RobotBackend,
    release_q: np.ndarray,
    planner_options: dict,
    configured_minimum_m: float,
) -> tuple[float, dict[str, float]]:
    """Cover the terminal-link radius and tracking allowance after release."""
    configured_minimum = float(configured_minimum_m)
    if not np.isfinite(configured_minimum) or configured_minimum < 0.0:
        raise ValueError("configured normal disengagement must be finite and non-negative")
    capsules = list(getattr(robot, "link_capsules", lambda _q: [])(release_q))
    terminal_radius = float(capsules[-1].radius) if capsules else 0.0
    collision_clearance = float(planner_options.get("carried_box_clearance", 0.0))
    tracking_allowance = float(
        planner_options.get("post_release_normal_tracking_allowance_m", 0.0)
    )
    if any(
        not np.isfinite(value) or value < 0.0
        for value in (terminal_radius, collision_clearance, tracking_allowance)
    ):
        raise ValueError("post-release normal-clearance inputs must be finite and non-negative")
    envelope_distance = terminal_radius + collision_clearance + tracking_allowance
    distance = max(configured_minimum, envelope_distance)
    return distance, {
        "configured_minimum_m": configured_minimum,
        "terminal_link_radius_m": terminal_radius,
        "collision_clearance_m": collision_clearance,
        "tracking_allowance_m": tracking_allowance,
        "derived_normal_disengage_distance_m": distance,
    }


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
    robot: RobotBackend,
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

    from .perception import controlled_release_height, conveyor_place_pose_candidates, detect_carton_obbs, select_detection, surface_place_pose_candidates

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
        carton_from_candidate_tool = (
            np.linalg.inv(grasp_tool_pose) @ carton.world_from_local
        )
        exclusion_names = {
            str(name)
            for name in planner_options.get(
                "conveyor_release_exclusion_surfaces", []
            )
        }
        exclusion_surfaces = [
            obstacle
            for obstacle in scene.obstacles
            if obstacle.name in exclusion_names
        ]

        def stays_in_selected_infeed(pose: np.ndarray) -> bool:
            if not exclusion_surfaces:
                return True
            desired_carton = pose @ carton_from_candidate_tool
            candidate_obb = OBB(
                center=desired_carton[:3, 3],
                half_extents=carton.half_extents,
                rotation=desired_carton[:3, :3],
                name=carton.name,
                category=carton.category,
            )
            carton_xy = candidate_obb.corners()[:, :2]
            carton_min = np.min(carton_xy, axis=0)
            carton_max = np.max(carton_xy, axis=0)
            for excluded in exclusion_surfaces:
                excluded_xy = excluded.corners()[:, :2]
                overlap = np.minimum(carton_max, np.max(excluded_xy, axis=0)) - np.maximum(
                    carton_min, np.min(excluded_xy, axis=0)
                )
                if np.all(overlap > 1e-9):
                    return False
            return True

        for surface_name in place_target_names:
            surface = place_surfaces.get(surface_name)
            if surface is None:
                continue
            requires_release_zone = bool(
                planner_options.get("place_requires_release_zone", "place_target" not in planner_options)
            )
            if requires_release_zone:
                surface_directions = planner_options.get(
                    "post_release_surface_directions_world", {}
                )
                surface_transport_direction = (
                    surface_directions.get(surface_name)
                    if isinstance(surface_directions, dict)
                    else None
                )
                poses = conveyor_place_pose_candidates(
                    surface,
                    carton,
                    grasp_tool_pose,
                    robot.base_transform[:3, 3],
                    edge_clearance=edge_clearance,
                    allow_all_carton_faces=bool(planner_options.get("conveyor_allow_all_carton_faces", False)),
                    require_front_release_zone=bool(planner_options.get("conveyor_require_front_release_zone", True)),
                    release_height=controlled_release_height(planner_options),
                    minimum_tool_outward_support_alignment=planner_options.get(
                        "continuous_conveyor_min_tool_outward_support_alignment"
                    ),
                    conveyor_transport_direction_world=surface_transport_direction,
                    minimum_conveyor_separation_alignment=planner_options.get(
                        "continuous_conveyor_min_release_separation_alignment"
                    ),
                    prefer_upright_support=bool(
                        planner_options.get("conveyor_prefer_upright_release", False)
                    ),
                    upright_yaw_step_degrees=planner_options.get(
                        "conveyor_upright_yaw_step_degrees"
                    ),
                )
                if bool(
                    planner_options.get(
                        "continuous_conveyor_adaptive_release_separation", False
                    )
                ):
                    direction_map = planner_options.get(
                        "post_release_surface_directions_world", {}
                    )
                    transition_names = planner_options.get(
                        "conveyor_release_transition_surfaces", [surface_name]
                    )
                    transition_directions = []
                    if isinstance(direction_map, dict):
                        for transition_name in transition_names:
                            value = direction_map.get(str(transition_name))
                            if value is None:
                                continue
                            direction = normalize(np.asarray(value, dtype=float))
                            if not any(
                                np.allclose(direction, existing)
                                for existing in transition_directions
                            ):
                                transition_directions.append(direction)
                    tool_size = np.asarray(
                        getattr(robot, "tool_collision_size", []), dtype=float
                    )
                    required_clearance = float(
                        planner_options.get("carried_box_clearance", 0.0)
                    ) + float(
                        planner_options.get(
                            "post_release_normal_tracking_allowance_m", 0.0
                        )
                    )
                    if tool_size.shape != (3,) or np.any(tool_size <= 0.0):
                        raise ValueError(
                            "adaptive conveyor separation requires tool collision size"
                        )

                    def dynamically_separates(pose: np.ndarray) -> bool:
                        for direction in transition_directions:
                            projected_extent = float(
                                np.abs(pose[:3, :3].T @ direction)
                                @ (0.5 * tool_size)
                            )
                            required_alignment = min(
                                1.0,
                                required_clearance / max(projected_extent, 1e-9),
                            )
                            if float(pose[:3, 2] @ direction) < required_alignment:
                                return False
                        return True

                    poses = [pose for pose in poses if dynamically_separates(pose)]
                poses = [pose for pose in poses if stays_in_selected_infeed(pose)]
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
        cup_clearance=float(planner_options.get("vacuum_grasp_clearance_m", 0.015)),
        contact_grid_fractions=planner_options.get("suction_contact_grid_fractions"),
        face_modes=face_modes,
    )
    candidates = _expand_candidate_wrist_rolls(
        candidates,
        planner_options.get("wrist_roll_angles_deg", [0.0]),
    )
    candidates = filter_suction_candidates_by_seal(candidates, carton, planner_options)
    candidates = filter_suction_candidates_by_tool_clearance(
        candidates,
        scene.all_obstacles,
        getattr(robot, "tool_collision_size", None),
        target_name=carton.name,
        margin_m=float(planner_options.get("tool_collision_margin_m", 0.001)),
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
    use_cartesian_final_approach = bool(
        planner_options.get("use_cartesian_final_approach", False)
    )
    use_wrist_first_carry = bool(planner_options.get("use_wrist_first_carry", False))
    configured_preplace_clearances = planner_options.get("conveyor_preplace_clearances")
    if configured_preplace_clearances is None:
        configured_preplace_clearances = [
            planner_options.get("conveyor_preplace_clearance", 0.30)
        ]
    conveyor_preplace_clearances = tuple(
        float(value) for value in configured_preplace_clearances
    )
    if not conveyor_preplace_clearances or any(
        not np.isfinite(value) or value <= 0.0
        for value in conveyor_preplace_clearances
    ):
        raise ValueError("conveyor preplace clearances must be finite and positive")

    failure_messages: list[str] = []
    best_pick_plan: PickPlan | None = None
    best_pick_cost = float("inf")
    plan_t0 = perf_counter()
    candidate_time_budget = float(planner_options.get("target_planning_time_limit_seconds", 0.0))
    for candidate_index, candidate in enumerate(candidates):
        candidate_cartesian_final_approach = use_cartesian_final_approach
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
        destack_clearance = _adaptive_destack_clearance(
            carton,
            scene.all_obstacles,
            candidate.outward_normal,
            clearance_m=max(
                carried_box_clearance,
                float(planner_options.get("carton_contact_tolerance_m", 0.001)),
            ),
        )
        destack_mode = str(planner_options.get("destack_mode", "configured")).lower()
        if destack_mode not in {"auto", "always", "never", "configured"}:
            raise ValueError("destack_mode must be auto, always, never, or configured")
        straight_destack_required = (
            destack_clearance.requires_straight_withdrawal
            if destack_mode == "auto"
            else destack_mode == "always"
            if destack_mode in {"always", "never"}
            else bool(planner_options.get("use_cartesian_destack_to_clearance", False))
        )
        if straight_destack_required:
            log(
                f"candidate {candidate_index}: adaptive straight destack "
                f"distance={destack_clearance.withdrawal_distance_m:.3f}m; "
                f"blockers={list(destack_clearance.blocking_carton_names)}"
            )
        else:
            log(f"candidate {candidate_index}: local face-plane clearance permits free destack")
        # Carry-exit must avoid both belts. The selected release surface is
        # ignored only after a concrete placement candidate has been chosen.
        carried_obstacles = scene.obstacles_without({carton.name})
        place_robot_obstacles = scene.obstacles_without({carton.name})

        def carried_orientation_valid(carried_box: OBB) -> bool:
            return _carried_orientation_policy_valid(
                carton,
                carried_box,
                scene,
                planner_options,
            )

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
        validated_middle_carry = None
        dense_place: list[np.ndarray] = []
        retreat_path: list[np.ndarray] = []
        dense_exit: list[np.ndarray] = []
        if place_enabled:
            pregrasp_standoff = max(
                0.0,
                float(
                    candidate.outward_normal
                    @ (candidate.pregrasp_pose[:3, 3] - candidate.contact_point)
                ),
            )
            if straight_destack_required:
                carry_exit_standoff = max(
                    pregrasp_standoff,
                    destack_clearance.withdrawal_distance_m,
                )
            else:
                # An open neighbourhood needs no prescribed straight segment;
                # the pregrasp clearance is merely the collision-free RRT start.
                carry_exit_standoff = pregrasp_standoff
            if candidate.face_mode == "top":
                vertical = np.array([0.0, 0.0, 1.0])
                carton_bottom = _obb_projection_interval(carton, vertical)[0]
                conveyor_lifts = []
                for obstacle in scene.obstacles:
                    if not obstacle.name.startswith("conveyor_"):
                        continue
                    support_top = _obb_projection_interval(obstacle, vertical)[1]
                    support_margin = _carried_obstacle_margin(
                        obstacle,
                        carried_box_clearance,
                        carton_contact_tolerance,
                    )
                    # OBB.intersects_obb expands both bodies by its margin.
                    conveyor_lifts.append(
                        support_top - carton_bottom + 2.0 * support_margin
                    )
                if conveyor_lifts:
                    carry_exit_standoff = max(
                        carry_exit_standoff,
                        max(conveyor_lifts),
                    )
            top_doorward_offset = np.zeros(3, dtype=float)
            if candidate.face_mode == "top":
                # A top-grasped carton can begin face-to-face with the rear
                # row. Lift while moving slightly toward the trailer door so
                # small IK residuals separate those faces instead of pressing
                # them together. The shift follows the live clearance and
                # task-tube tolerances rather than a carton-index waypoint.
                top_doorward_offset[0] = -(
                    carried_box_clearance
                    + float(planner_options.get("destack_position_tolerance_m", 0.006))
                )
            carry_exit_pose = make_transform(
                candidate.pregrasp_pose[:3, :3],
                candidate.contact_point
                + candidate.outward_normal * carry_exit_standoff
                + top_doorward_offset,
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

                def conveyor_separation_cost(
                    surface_name: str, pose: np.ndarray
                ) -> float:
                    weight = float(
                        planner_options.get(
                            "conveyor_release_separation_cost_weight_m", 0.0
                        )
                    )
                    if weight <= 0.0:
                        return 0.0
                    directions = planner_options.get(
                        "post_release_surface_directions_world", {}
                    )
                    transition_names = planner_options.get(
                        "conveyor_release_transition_surfaces", [surface_name]
                    )
                    direction_values = (
                        [directions.get(str(name)) for name in transition_names]
                        if isinstance(directions, dict)
                        else []
                    )
                    direction_values = [
                        value for value in direction_values if value is not None
                    ]
                    if not direction_values:
                        return 0.0
                    # Positive alignment means the running belt carries the
                    # released carton away from the suction face instead of
                    # sweeping it across the gripper. Use the least favourable
                    # leg of an L conveyor so the transfer cannot reverse that
                    # separation immediately after release.
                    alignment = min(
                        float(
                            np.clip(
                                pose[:3, 2]
                                @ normalize(np.asarray(value, dtype=float)),
                                -1.0,
                                1.0,
                            )
                        )
                        for value in direction_values
                    )
                    return weight * (1.0 - alignment)

                def release_pose_cost(surface_name: str, pose: np.ndarray) -> float:
                    return transfer_pose_cost(pose) + conveyor_separation_cost(
                        surface_name, pose
                    )

                prefer_upright_release = bool(
                    planner_options.get("conveyor_prefer_upright_release", False)
                )

                def place_pose_cost(entry: tuple[str, OBB, np.ndarray]) -> tuple[float, float]:
                    if not prefer_upright_release:
                        return (0.0, release_pose_cost(entry[0], entry[2]))
                    desired_carton = entry[2] @ carton_from_tool
                    support_normal = entry[1].rotation[:, 2]
                    upright_alignment = float(desired_carton[:3, 2] @ support_normal)
                    # Lexicographic preference keeps upright yaw rotations
                    # ahead of side/top support while the latter remain in the
                    # candidate list as deterministic reachability fallbacks.
                    return (-upright_alignment, release_pose_cost(entry[0], entry[2]))

                place_poses.sort(key=place_pose_cost)
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
                    # All L-belt sections remain collision-active. The release
                    # pose is above the support plane, so excluding the chosen
                    # deck would only hide a premature carton/belt contact.
                    surface_carried_obstacles = scene.obstacles_without({carton.name})

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
                # An endpoint-only cost can select a nearby release IK whose
                # connecting edge sweeps the carton through the stack, then
                # waste the full RRT budget even though another feasible
                # release IK has a direct collision-free connection.  Probe
                # the bounded feasible set and make direct connectivity the
                # first ranking key; distance remains the optimizer within
                # the same connectivity class.
                direct_place_connectivity: dict[tuple[str, tuple[float, ...]], bool] = {}
                for surface_name, _surface, _pose, result in feasible_places:
                    candidate_obstacles = scene.obstacles_without(
                        {carton.name, surface_name}
                    )

                    def candidate_direct_valid(q: np.ndarray) -> bool:
                        return posture_valid(q) and robot.is_collision_free(
                            q, place_robot_obstacles
                        ) and _carried_box_state_valid(
                            robot,
                            carton,
                            grasp.q,
                            q,
                            candidate_obstacles,
                            box_margin=carried_box_clearance,
                            carton_from_tool=carton_from_tool,
                            carton_contact_tolerance=carton_contact_tolerance,
                            orientation_valid=carried_orientation_valid,
                        )

                    direct_ok, _ = _edge_collision_free(
                        robot,
                        carry_exit.q,
                        result.q,
                        place_robot_obstacles,
                        resolution=float(planner_options.get("output_resolution", 0.035)),
                        extra_state_valid=candidate_direct_valid,
                    )
                    direct_place_connectivity[
                        (surface_name, tuple(np.asarray(result.q, dtype=float)))
                    ] = direct_ok
                log(
                    f"candidate {candidate_index}: direct conveyor connections="
                    f"{sum(direct_place_connectivity.values())}/{len(feasible_places)}"
                )

                def direct_connectivity_rank(item) -> int:
                    return 0 if direct_place_connectivity.get(
                        (item[0], tuple(np.asarray(item[3].q, dtype=float))), False
                    ) else 1

                if candidate_cartesian_final_approach:
                    approach_directions = tuple(planner_options.get("conveyor_approach_directions", ("front", "top", "left", "right")))
                    preplace_results = []
                    for surface_name, surface, release_pose, release_ik in feasible_places:
                        candidate_carried_obstacles = scene.obstacles_without(
                            {carton.name, surface_name}
                        )

                        def candidate_placement_valid(q: np.ndarray) -> bool:
                            return posture_valid(q) and _carried_box_state_valid(
                                robot,
                                carton,
                                grasp.q,
                                q,
                                candidate_carried_obstacles,
                                box_margin=carried_box_clearance,
                                carton_from_tool=carton_from_tool,
                                carton_contact_tolerance=carton_contact_tolerance,
                                orientation_valid=carried_orientation_valid,
                            )

                        preplace_pose_options = [
                            (
                                f"{direction}@{clearance:.3f}m",
                                pose,
                            )
                            for clearance in conveyor_preplace_clearances
                            for direction, pose in _conveyor_preplace_poses(
                                release_pose,
                                clearance,
                                approach_directions,
                                surface.rotation,
                            )
                        ]
                        for approach_direction, candidate_preplace_pose in preplace_pose_options:
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
                                extra_state_valid=candidate_placement_valid,
                            )
                            if preplace_ik.success:
                                final_approach = _plan_cartesian_carry(
                                    robot,
                                    carton,
                                    grasp.q,
                                    preplace_ik.q,
                                    release_pose,
                                    place_robot_obstacles,
                                    candidate_carried_obstacles,
                                    carton_from_tool,
                                    carried_orientation_valid,
                                    rng,
                                    extra_state_valid=posture_valid,
                                    carton_contact_tolerance=carton_contact_tolerance,
                                    box_margin=carried_box_clearance,
                                    position_step=float(
                                        planner_options.get("cartesian_carry_position_step", 0.10)
                                    ),
                                    orientation_step=float(
                                        planner_options.get("cartesian_carry_orientation_step", 0.16)
                                    ),
                                    joint_resolution=float(
                                        planner_options.get("output_resolution", 0.035)
                                    ),
                                )
                                if final_approach.success:
                                    middle_carry = _plan_cartesian_carry(
                                        robot,
                                        carton,
                                        grasp.q,
                                        carry_exit.q,
                                        candidate_preplace_pose,
                                        place_robot_obstacles,
                                        candidate_carried_obstacles,
                                        carton_from_tool,
                                        carried_orientation_valid,
                                        rng,
                                        extra_state_valid=posture_valid,
                                        carton_contact_tolerance=carton_contact_tolerance,
                                        box_margin=carried_box_clearance,
                                        position_step=float(
                                            planner_options.get(
                                                "cartesian_carry_position_step", 0.10
                                            )
                                        ),
                                        orientation_step=float(
                                            planner_options.get(
                                                "cartesian_carry_orientation_step", 0.16
                                            )
                                        ),
                                        joint_resolution=float(
                                            planner_options.get("output_resolution", 0.035)
                                        ),
                                        ik_iterations=int(
                                            planner_options.get(
                                                "place_ik_max_iterations",
                                                planner_options.get("ik_max_iterations", 250),
                                            )
                                        ),
                                        position_tolerance=float(
                                            planner_options.get(
                                                "place_position_tolerance", 0.025
                                            )
                                        ),
                                        orientation_tolerance=float(
                                            planner_options.get(
                                                "place_orientation_tolerance", 0.22
                                            )
                                        ),
                                    )
                                    preplace_results.append(
                                        (
                                            surface_name,
                                            surface,
                                            release_pose,
                                            release_ik,
                                            approach_direction,
                                            candidate_preplace_pose,
                                            preplace_ik,
                                            final_approach,
                                            middle_carry,
                                        )
                                    )
                    if not preplace_results:
                        if bool(
                            planner_options.get("cartesian_final_approach_required", False)
                        ):
                            failure_messages.append(
                                f"candidate {candidate_index}: conveyor preplace IK failed"
                            )
                            log(
                                f"candidate {candidate_index}: conveyor preplace IK failed "
                                f"after {perf_counter() - candidate_t0:.1f}s"
                            )
                            continue
                        candidate_cartesian_final_approach = False
                        joint_weight = float(
                            planner_options.get("place_joint_motion_weight", 0.10)
                        )
                        surface_penalties = planner_options.get("place_surface_penalties", {})
                        place_target_name, place_surface, selected_place_pose, place = min(
                            feasible_places,
                            key=lambda item: (
                                direct_connectivity_rank(item),
                                release_pose_cost(item[0], item[2])
                                + joint_weight
                                * _joint_motion_time_cost(
                                    carry_exit.q,
                                    item[3].q,
                                    planner_options.get("joint_velocity_limits_rad_s"),
                                )
                                + float(surface_penalties.get(item[0], 0.0))
                            ),
                        )
                        log(
                            f"candidate {candidate_index}: no straight final approach; "
                            "using collision-checked free-space placement"
                        )
                    else:
                        joint_weight = float(
                            planner_options.get("place_joint_motion_weight", 0.10)
                        )
                        surface_penalties = planner_options.get("place_surface_penalties", {})
                        place_target_name, place_surface, selected_place_pose, place, approach_direction, preplace_pose, preplace, validated_final_approach, validated_middle_carry = min(
                            preplace_results,
                            key=lambda item: (
                                0 if item[8].success else 1,
                                direct_connectivity_rank(item),
                                # Rank the actual clearance-to-preplace task
                                # displacement, not just endpoint joint time.
                                # The old joint-only key could choose a nearby
                                # IK branch whose Cartesian middle connector
                                # crossed the stack, forcing a long RRT detour.
                                transfer_pose_cost(item[5])
                                + conveyor_separation_cost(item[0], item[2])
                                + joint_weight
                                * (
                                    _joint_motion_time_cost(
                                        carry_exit.q,
                                        item[6].q,
                                        planner_options.get("joint_velocity_limits_rad_s"),
                                    )
                                    + _joint_motion_time_cost(
                                        item[6].q,
                                        item[3].q,
                                        planner_options.get("joint_velocity_limits_rad_s"),
                                    )
                                )
                                + float(surface_penalties.get(item[0], 0.0))
                            ),
                        )
                        log(
                            f"candidate {candidate_index}: selected conveyor "
                            f"approach={approach_direction}"
                        )
                else:
                    joint_weight = float(planner_options.get("place_joint_motion_weight", 0.10))
                    surface_penalties = planner_options.get("place_surface_penalties", {})
                    place_target_name, place_surface, selected_place_pose, place = min(
                        feasible_places,
                        key=lambda item: (
                            direct_connectivity_rank(item),
                            release_pose_cost(item[0], item[2])
                            + joint_weight * float(np.linalg.norm(item[3].q - carry_exit.q))
                            + float(surface_penalties.get(item[0], 0.0))
                        ),
                    )
                carried_obstacles = scene.obstacles_without({carton.name})
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
            log(
                f"candidate {candidate_index}: carried endpoints "
                f"grasp_valid={place_valid(grasp.q)} "
                f"clearance_valid={place_valid(carry_exit.q)}"
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
            support_clearance_retreat = candidate.face_mode == "top"
            direct_full_destack = False
            if straight_destack_required or support_clearance_retreat:
                direct_destack_ok, direct_destack_path = _edge_collision_free(
                    robot,
                    grasp.q,
                    carry_exit.q,
                    place_robot_obstacles,
                    resolution=float(planner_options.get("output_resolution", 0.035)),
                    extra_state_valid=place_valid,
                )
                direct_full_destack = direct_destack_ok and _path_within_task_space_tube(
                    robot,
                    direct_destack_path,
                    position_tolerance_m=float(
                        planner_options.get("destack_position_tolerance_m", 0.006)
                    ),
                    orientation_tolerance_rad=float(
                        planner_options.get("destack_orientation_tolerance_rad", 0.035)
                    ),
                )
                if direct_full_destack:
                    retreat = PlanResult(
                        True,
                        direct_destack_path,
                        0,
                        "joint interpolation certified inside Cartesian destack tube",
                    )
                    log(f"candidate {candidate_index}: direct joint destack certified in task tube")
                else:
                    retreat_target_pose = candidate.pregrasp_pose.copy()
                    retreat_target_pose[:3, 3] += top_doorward_offset
                    retreat = _plan_cartesian_carry(
                        robot,
                        carton,
                        grasp.q,
                        grasp.q,
                        retreat_target_pose,
                        place_robot_obstacles,
                        carried_obstacles,
                        carton_from_tool,
                        carried_orientation_valid,
                        rng,
                        extra_state_valid=posture_valid,
                        carton_contact_tolerance=carton_contact_tolerance,
                        box_margin=carried_box_clearance,
                        position_step=float(planner_options.get("destack_position_step", 0.04)),
                        orientation_step=float(planner_options.get("cartesian_carry_orientation_step", 0.16)),
                        joint_resolution=float(planner_options.get("output_resolution", 0.035)),
                        ik_iterations=int(planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250))),
                        position_tolerance=float(
                            planner_options.get("destack_position_tolerance_m", 0.006)
                        ),
                        orientation_tolerance=float(
                            planner_options.get("destack_orientation_tolerance_rad", 0.035)
                        ),
                        ik_random_restarts=int(
                            planner_options.get("destack_ik_random_restarts", 2)
                        ),
                    )
            else:
                direct_retreat_ok, direct_retreat_path = _edge_collision_free(
                    robot,
                    grasp.q,
                    carry_exit.q,
                    place_robot_obstacles,
                    resolution=float(planner_options.get("output_resolution", 0.035)),
                    extra_state_valid=place_valid,
                )
                retreat = (
                    PlanResult(
                        True,
                        direct_retreat_path,
                        0,
                        "direct collision-checked open-neighbourhood retreat",
                    )
                    if direct_retreat_ok
                    else place_planner.plan(
                        grasp.q,
                        carry_exit.q,
                        time_limit_seconds=place_attempt_limit,
                    )
                )
            if not retreat.success and (straight_destack_required or support_clearance_retreat):
                direct_retreat_ok, direct_retreat_path = _edge_collision_free(
                    robot,
                    grasp.q,
                    approach_guide.q,
                    place_robot_obstacles,
                    resolution=float(planner_options.get("output_resolution", 0.035)),
                    extra_state_valid=place_valid,
                )
                direct_retreat_in_task_tube = direct_retreat_ok and _path_within_task_space_tube(
                    robot,
                    direct_retreat_path,
                    position_tolerance_m=float(
                        planner_options.get("destack_position_tolerance_m", 0.006)
                    ),
                    orientation_tolerance_rad=float(
                        planner_options.get("destack_orientation_tolerance_rad", 0.035)
                    ),
                )
                if direct_retreat_in_task_tube:
                    retreat = PlanResult(
                        True,
                        direct_retreat_path,
                        0,
                        "direct collision-checked destack fallback certified in task tube",
                    )
                else:
                    failure_messages.append(f"candidate {candidate_index}: {retreat.message}")
                    log(f"candidate {candidate_index}: {retreat.message} after {perf_counter() - candidate_t0:.1f}s")
                    continue
            if not retreat.success:
                failure_messages.append(f"candidate {candidate_index}: {retreat.message}")
                log(
                    f"candidate {candidate_index}: {retreat.message} "
                    f"after {perf_counter() - candidate_t0:.1f}s"
                )
                continue
            retreat_path = retreat.path
            withdrawal_q = retreat_path[-1]
            if direct_full_destack:
                exit_transit = PlanResult(
                    True, [withdrawal_q.copy()], 0, "destack already reached clearance pose"
                )
            elif straight_destack_required:
                # Neighbours block both lateral escape directions. Preserve
                # tool orientation while withdrawing only as far as the OBB
                # projections require; after this segment the wrist is free.
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
                    carton_contact_tolerance=carton_contact_tolerance,
                    box_margin=carried_box_clearance,
                    position_step=float(planner_options.get("destack_position_step", 0.04)),
                    orientation_step=float(planner_options.get("cartesian_carry_orientation_step", 0.16)),
                    joint_resolution=float(planner_options.get("output_resolution", 0.035)),
                    ik_iterations=int(planner_options.get("place_ik_max_iterations", planner_options.get("ik_max_iterations", 250))),
                    position_tolerance=float(
                        planner_options.get("destack_position_tolerance_m", 0.006)
                    ),
                    orientation_tolerance=float(
                        planner_options.get("destack_orientation_tolerance_rad", 0.035)
                    ),
                    ik_random_restarts=int(
                        planner_options.get("destack_ik_random_restarts", 2)
                    ),
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
                direct_exit_ok, direct_exit_path = _edge_collision_free(
                    robot,
                    withdrawal_q,
                    carry_exit.q,
                    place_robot_obstacles,
                    resolution=float(planner_options.get("output_resolution", 0.035)),
                    extra_state_valid=place_valid,
                )
                exit_transit = (
                    PlanResult(True, direct_exit_path, 0, "direct free-space destack phase")
                    if direct_exit_ok
                    else place_planner.plan(
                        withdrawal_q,
                        carry_exit.q,
                        time_limit_seconds=place_attempt_limit,
                    )
                )
            if (
                not exit_transit.success
                and straight_destack_required
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
            preserve_exit_phase = use_wrist_first_carry or straight_destack_required
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
            if candidate_cartesian_final_approach and mobile_place_position is None:
                assert selected_place_pose is not None
                assert preplace_pose is not None and preplace is not None
                if use_cartesian_carry:
                    assert validated_middle_carry is not None
                    reconfigure_transit = validated_middle_carry
                else:
                    direct_reconfigure_ok, direct_reconfigure_path = _edge_collision_free(
                        robot,
                        carry_exit.q,
                        preplace.q,
                        place_robot_obstacles,
                        resolution=float(
                            planner_options.get("output_resolution", 0.035)
                        ),
                        extra_state_valid=place_valid,
                    )
                    reconfigure_transit = (
                        PlanResult(
                            True,
                            direct_reconfigure_path,
                            0,
                            "direct free-space conveyor reconfiguration",
                        )
                        if direct_reconfigure_ok
                        else place_planner.plan(
                            carry_exit.q,
                            preplace.q,
                            time_limit_seconds=place_attempt_limit,
                        )
                    )
                if use_cartesian_carry and not reconfigure_transit.success:
                    # The straight task-space chord may cross the remaining
                    # stack even though both the geometry-derived clearance
                    # pose and conveyor pre-place pose are valid. First extend
                    # the fixed-orientation withdrawal just far enough for the
                    # carton's bounding sphere to clear the former stack
                    # plane. This creates reorientation room from carton
                    # geometry rather than a fixed channel waypoint.
                    log(
                        f"candidate {candidate_index}: Cartesian middle connector blocked; "
                        "trying geometry-derived reorientation clearance"
                    )
                    reorientation_standoff = _reorientation_clearance_standoff(
                        carton,
                        candidate.outward_normal,
                        carry_exit_standoff,
                        clearance_m=carried_box_clearance,
                    )
                    reorientation_pose = make_transform(
                        candidate.pregrasp_pose[:3, :3],
                        candidate.contact_point
                        + candidate.outward_normal * reorientation_standoff,
                    )
                    clearance_extension = _plan_cartesian_carry(
                        robot,
                        carton,
                        grasp.q,
                        carry_exit.q,
                        reorientation_pose,
                        place_robot_obstacles,
                        carried_obstacles,
                        carton_from_tool,
                        carried_orientation_valid,
                        rng,
                        extra_state_valid=posture_valid,
                        carton_contact_tolerance=carton_contact_tolerance,
                        box_margin=carried_box_clearance,
                        position_step=float(
                            planner_options.get("cartesian_carry_position_step", 0.10)
                        ),
                        orientation_step=float(
                            planner_options.get("cartesian_carry_orientation_step", 0.16)
                        ),
                        joint_resolution=float(
                            planner_options.get("output_resolution", 0.035)
                        ),
                        ik_iterations=int(
                            planner_options.get(
                                "place_ik_max_iterations",
                                planner_options.get("ik_max_iterations", 250),
                            )
                        ),
                        position_tolerance=float(
                            planner_options.get("place_position_tolerance", 0.025)
                        ),
                        orientation_tolerance=float(
                            planner_options.get("place_orientation_tolerance", 0.22)
                        ),
                    )
                    if clearance_extension.success:
                        extension_q = clearance_extension.path[-1]
                        clearance_to_place = _plan_cartesian_carry(
                            robot,
                            carton,
                            grasp.q,
                            extension_q,
                            preplace_pose,
                            place_robot_obstacles,
                            carried_obstacles,
                            carton_from_tool,
                            carried_orientation_valid,
                            rng,
                            extra_state_valid=posture_valid,
                            carton_contact_tolerance=carton_contact_tolerance,
                            box_margin=carried_box_clearance,
                            position_step=float(
                                planner_options.get("cartesian_carry_position_step", 0.10)
                            ),
                            orientation_step=float(
                                planner_options.get("cartesian_carry_orientation_step", 0.16)
                            ),
                            joint_resolution=float(
                                planner_options.get("output_resolution", 0.035)
                            ),
                            ik_iterations=int(
                                planner_options.get(
                                    "place_ik_max_iterations",
                                    planner_options.get("ik_max_iterations", 250),
                                )
                            ),
                            position_tolerance=float(
                                planner_options.get("place_position_tolerance", 0.025)
                            ),
                            orientation_tolerance=float(
                                planner_options.get("place_orientation_tolerance", 0.22)
                            ),
                        )
                        if not clearance_to_place.success:
                            log(
                                f"candidate {candidate_index}: direct clearance-to-place "
                                f"failure: {clearance_to_place.message}"
                            )
                            wall_offsets = _reorientation_wall_clearance_offsets(
                                _carried_box_at(
                                    robot,
                                    carton,
                                    carton_from_tool,
                                    extension_q,
                                ),
                                carried_obstacles,
                                candidate.outward_normal,
                                clearance_m=carried_box_clearance,
                            )
                            wall_offset = wall_offsets[0]
                            if float(np.linalg.norm(wall_offset)) > 1e-6:
                                log(
                                    f"candidate {candidate_index}: applying geometry-derived "
                                    f"side-wall clearance offset={np.round(wall_offset, 3).tolist()}"
                                )
                                wall_clearance_pose = robot.fk(extension_q)
                                wall_clearance_pose[:3, 3] += wall_offset
                                wall_clearance = _plan_cartesian_carry(
                                    robot,
                                    carton,
                                    grasp.q,
                                    extension_q,
                                    wall_clearance_pose,
                                    place_robot_obstacles,
                                    carried_obstacles,
                                    carton_from_tool,
                                    carried_orientation_valid,
                                    rng,
                                    extra_state_valid=posture_valid,
                                    carton_contact_tolerance=carton_contact_tolerance,
                                    box_margin=carried_box_clearance,
                                    position_step=float(
                                        planner_options.get("cartesian_carry_position_step", 0.10)
                                    ),
                                    orientation_step=float(
                                        planner_options.get("cartesian_carry_orientation_step", 0.16)
                                    ),
                                    joint_resolution=float(
                                        planner_options.get("output_resolution", 0.035)
                                    ),
                                    ik_iterations=int(
                                        planner_options.get(
                                            "place_ik_max_iterations",
                                            planner_options.get("ik_max_iterations", 250),
                                        ),
                                    ),
                                    position_tolerance=float(
                                        planner_options.get("place_position_tolerance", 0.025)
                                    ),
                                    orientation_tolerance=float(
                                        planner_options.get("place_orientation_tolerance", 0.22)
                                    ),
                                )
                                if wall_clearance.success:
                                    wall_q = wall_clearance.path[-1]
                                    wall_to_place = _plan_cartesian_carry(
                                        robot,
                                        carton,
                                        grasp.q,
                                        wall_q,
                                        preplace_pose,
                                        place_robot_obstacles,
                                        carried_obstacles,
                                        carton_from_tool,
                                        carried_orientation_valid,
                                        rng,
                                        extra_state_valid=posture_valid,
                                        carton_contact_tolerance=carton_contact_tolerance,
                                        box_margin=carried_box_clearance,
                                        position_step=float(
                                            planner_options.get("cartesian_carry_position_step", 0.10)
                                        ),
                                        orientation_step=float(
                                            planner_options.get("cartesian_carry_orientation_step", 0.16)
                                        ),
                                        joint_resolution=float(
                                            planner_options.get("output_resolution", 0.035)
                                        ),
                                        ik_iterations=int(
                                            planner_options.get(
                                                "place_ik_max_iterations",
                                                planner_options.get("ik_max_iterations", 250),
                                            )
                                        ),
                                        position_tolerance=float(
                                            planner_options.get("place_position_tolerance", 0.025)
                                        ),
                                        orientation_tolerance=float(
                                            planner_options.get("place_orientation_tolerance", 0.22)
                                        ),
                                    )
                                    if wall_to_place.success:
                                        clearance_to_place = PlanResult(
                                            True,
                                            wall_clearance.path + wall_to_place.path[1:],
                                            wall_clearance.iterations + wall_to_place.iterations,
                                            "geometry-derived side-wall clearance connector",
                                        )
                                    else:
                                        log(
                                            f"candidate {candidate_index}: minimum-wall connector "
                                            f"failure: {wall_to_place.message}"
                                        )
                                        rrt_start_q = wall_q
                                        rrt_prefix = wall_clearance.path
                                        for offset_index, candidate_offset in enumerate(
                                            wall_offsets[1:], start=1
                                        ):
                                            log(
                                                f"candidate {candidate_index}: trying side-wall "
                                                f"clearance candidate {offset_index + 1}/"
                                                f"{len(wall_offsets)} offset="
                                                f"{np.round(candidate_offset, 3).tolist()}"
                                            )
                                            candidate_pose = robot.fk(extension_q)
                                            candidate_pose[:3, 3] += candidate_offset
                                            candidate_clearance = _plan_cartesian_carry(
                                                robot,
                                                carton,
                                                grasp.q,
                                                extension_q,
                                                candidate_pose,
                                                place_robot_obstacles,
                                                carried_obstacles,
                                                carton_from_tool,
                                                carried_orientation_valid,
                                                rng,
                                                extra_state_valid=posture_valid,
                                                carton_contact_tolerance=carton_contact_tolerance,
                                                box_margin=carried_box_clearance,
                                                position_step=float(
                                                    planner_options.get(
                                                        "cartesian_carry_position_step", 0.10
                                                    )
                                                ),
                                                orientation_step=float(
                                                    planner_options.get(
                                                        "cartesian_carry_orientation_step", 0.16
                                                    )
                                                ),
                                                joint_resolution=float(
                                                    planner_options.get("output_resolution", 0.035)
                                                ),
                                                ik_iterations=int(
                                                    planner_options.get(
                                                        "place_ik_max_iterations",
                                                        planner_options.get("ik_max_iterations", 250),
                                                    )
                                                ),
                                                position_tolerance=float(
                                                    planner_options.get(
                                                        "place_position_tolerance", 0.025
                                                    )
                                                ),
                                                orientation_tolerance=float(
                                                    planner_options.get(
                                                        "place_orientation_tolerance", 0.22
                                                    )
                                                ),
                                            )
                                            if not candidate_clearance.success:
                                                log(
                                                    f"candidate {candidate_index}: wall-offset "
                                                    f"candidate {offset_index + 1} shift failure: "
                                                    f"{candidate_clearance.message}"
                                                )
                                                continue
                                            rrt_start_q = candidate_clearance.path[-1]
                                            rrt_prefix = candidate_clearance.path
                                            candidate_to_place = _plan_cartesian_carry(
                                                robot,
                                                carton,
                                                grasp.q,
                                                rrt_start_q,
                                                preplace_pose,
                                                place_robot_obstacles,
                                                carried_obstacles,
                                                carton_from_tool,
                                                carried_orientation_valid,
                                                rng,
                                                extra_state_valid=posture_valid,
                                                carton_contact_tolerance=carton_contact_tolerance,
                                                box_margin=carried_box_clearance,
                                                position_step=float(
                                                    planner_options.get(
                                                        "cartesian_carry_position_step", 0.10
                                                    )
                                                ),
                                                orientation_step=float(
                                                    planner_options.get(
                                                        "cartesian_carry_orientation_step", 0.16
                                                    )
                                                ),
                                                joint_resolution=float(
                                                    planner_options.get(
                                                        "output_resolution", 0.035
                                                    )
                                                ),
                                                ik_iterations=int(
                                                    planner_options.get(
                                                        "place_ik_max_iterations",
                                                        planner_options.get(
                                                            "ik_max_iterations", 250
                                                        ),
                                                    )
                                                ),
                                                position_tolerance=float(
                                                    planner_options.get(
                                                        "place_position_tolerance", 0.025
                                                    )
                                                ),
                                                orientation_tolerance=float(
                                                    planner_options.get(
                                                        "place_orientation_tolerance", 0.22
                                                    )
                                                ),
                                            )
                                            if candidate_to_place.success:
                                                clearance_to_place = PlanResult(
                                                    True,
                                                    candidate_clearance.path
                                                    + candidate_to_place.path[1:],
                                                    candidate_clearance.iterations
                                                    + candidate_to_place.iterations,
                                                    "shortest feasible side-wall clearance connector",
                                                )
                                                break
                                            log(
                                                f"candidate {candidate_index}: wall-offset "
                                                f"candidate {offset_index + 1} connector failure: "
                                                f"{candidate_to_place.message}"
                                            )
                                        if not clearance_to_place.success:
                                            log(
                                                f"candidate {candidate_index}: wall-clearance Cartesian "
                                                "connector blocked; trying local carried-box RRT"
                                            )
                                            wall_to_place = place_planner.plan(
                                                rrt_start_q,
                                                preplace.q,
                                                time_limit_seconds=place_attempt_limit,
                                            )
                                            if wall_to_place.success:
                                                dense_wall_to_place = place_planner.densify(
                                                    wall_to_place.path,
                                                    resolution=float(
                                                        planner_options.get(
                                                            "output_resolution", 0.035
                                                        )
                                                    ),
                                                )
                                                clearance_to_place = PlanResult(
                                                    True,
                                                    rrt_prefix + dense_wall_to_place[1:],
                                                    wall_clearance.iterations
                                                    + wall_to_place.iterations,
                                                    "wall-cleared local carried-box RRT repair",
                                                )
                            if not clearance_to_place.success:
                                log(
                                    f"candidate {candidate_index}: clearance-to-place Cartesian "
                                    "connector blocked; trying local carried-box RRT repair"
                                )
                                clearance_to_place = place_planner.plan(
                                    extension_q,
                                    preplace.q,
                                    time_limit_seconds=place_attempt_limit,
                                )
                                if clearance_to_place.success:
                                    clearance_to_place = PlanResult(
                                        True,
                                        place_planner.densify(
                                            clearance_to_place.path,
                                            resolution=float(
                                                planner_options.get("output_resolution", 0.035)
                                            ),
                                        ),
                                        clearance_to_place.iterations,
                                        "geometry-clearance local carried-box RRT repair",
                                    )
                        if clearance_to_place.success:
                            reconfigure_transit = PlanResult(
                                True,
                                clearance_extension.path + clearance_to_place.path[1:],
                                clearance_extension.iterations + clearance_to_place.iterations,
                                "geometry-derived reorientation clearance connector",
                            )
                    if not reconfigure_transit.success:
                        # If the fixed-orientation extension itself is not
                        # reachable, retain the bounded legacy repair as a
                        # final fallback for genuinely open neighbourhoods.
                        log(
                            f"candidate {candidate_index}: geometry-clearance connector failed; "
                            "trying local carried-box RRT repair"
                        )
                        reconfigure_transit = place_planner.plan(
                            carry_exit.q,
                            preplace.q,
                            time_limit_seconds=place_attempt_limit,
                        )
                        if reconfigure_transit.success:
                            reconfigure_transit = PlanResult(
                                True,
                                place_planner.densify(
                                    reconfigure_transit.path,
                                    resolution=float(
                                        planner_options.get("output_resolution", 0.035)
                                    ),
                                ),
                                reconfigure_transit.iterations,
                                "local carried-box RRT repair",
                            )
                if not reconfigure_transit.success:
                    failure_messages.append(f"candidate {candidate_index}: {reconfigure_transit.message}")
                    log(f"candidate {candidate_index}: {reconfigure_transit.message} after {perf_counter() - candidate_t0:.1f}s")
                    continue
                # A Cartesian carry already contains collision-checked, task-space
                # interpolated edges.  Joint-space shortcutting here can replace the
                # intended straight withdrawal/downward sweep with a wrist-heavy arc.
                dense_reconfigure = (
                    reconfigure_transit.path
                    if use_cartesian_carry
                    else place_planner.densify(
                        place_planner.shortcut(
                            reconfigure_transit.path,
                            attempts=planner_options.get("shortcut_attempts", 180),
                        ),
                        resolution=planner_options.get("output_resolution", 0.035),
                    )
                )
                # Re-solve the final Cartesian approach from the *actual*
                # reconfiguration endpoint. The earlier validated approach
                # proves geometric feasibility, but its independently solved
                # preplace IK can lie on the opposite spherical-wrist branch
                # and would otherwise introduce an instantaneous J4/J5/J6
                # flip at the phase boundary.
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
                    carton_contact_tolerance=carton_contact_tolerance,
                    box_margin=carried_box_clearance,
                    position_step=float(
                        planner_options.get("cartesian_carry_position_step", 0.10)
                    ),
                    orientation_step=float(
                        planner_options.get("cartesian_carry_orientation_step", 0.16)
                    ),
                    joint_resolution=float(
                        planner_options.get("output_resolution", 0.035)
                    ),
                    ik_iterations=int(
                        planner_options.get(
                            "place_ik_max_iterations",
                            planner_options.get("ik_max_iterations", 250),
                        )
                    ),
                    position_tolerance=float(
                        planner_options.get("place_position_tolerance", 0.025)
                    ),
                    orientation_tolerance=float(
                        planner_options.get("place_orientation_tolerance", 0.22)
                    ),
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
            if candidate_cartesian_final_approach and mobile_place_position is None:
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
            if not (candidate_cartesian_final_approach and mobile_place_position is None):
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
        release_retreat_index = release_index
        if place_enabled and not mobile_base_path:
            retreat_distance = float(
                planner_options.get("post_release_retreat_distance_m", 0.0)
            )
            if retreat_distance > 0.0:
                disengage_distance = float(
                    planner_options.get("post_release_normal_disengage_distance_m", 0.0)
                )
                release_retreat_path = [np.asarray(full[release_index], dtype=float).copy()]
                disengage_distance, _ = adaptive_post_release_normal_disengage_distance(
                    robot,
                    release_retreat_path[-1],
                    planner_options,
                    disengage_distance,
                )
                if disengage_distance > 0.0:
                    release_pose = robot.fk(release_retreat_path[-1])
                    normal_target = (
                        release_pose[:3, 3]
                        - disengage_distance * release_pose[:3, 2]
                    )
                    normal_escape = _plan_cartesian_tool_translation(
                        robot,
                        release_retreat_path[-1],
                        normal_target,
                        place_robot_obstacles,
                        rng,
                        extra_state_valid=posture_valid,
                        position_step=float(
                            planner_options.get(
                                "post_release_normal_position_step_m", 0.01
                            )
                        ),
                        joint_resolution=float(
                            planner_options.get("output_resolution", 0.035)
                        ),
                    )
                    if not normal_escape.success:
                        failure_messages.append(
                            f"candidate {candidate_index}: {normal_escape.message}"
                        )
                        continue
                    release_retreat_path.extend(normal_escape.path[1:])
                escape_target, _ = post_release_escape_target(
                    robot,
                    release_retreat_path[-1],
                    place_target_name,
                    planner_options,
                    normal_disengage_distance_m=0.0,
                    vertical_lift_distance_m=retreat_distance,
                )
                escape = _plan_cartesian_tool_translation(
                    robot,
                    release_retreat_path[-1],
                    escape_target,
                    place_robot_obstacles,
                    rng,
                    extra_state_valid=posture_valid,
                    position_step=float(
                        planner_options.get("post_release_lift_position_step_m", 0.03)
                    ),
                    joint_resolution=float(
                        planner_options.get("output_resolution", 0.035)
                    ),
                )
                if not escape.success:
                    failure_messages.append(f"candidate {candidate_index}: {escape.message}")
                    continue
                release_retreat_path.extend(escape.path[1:])
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
        # IK calls at adjacent phase boundaries can encode the same J4/J6
        # posture on opposite sides of a 2*pi wrap. Preserve the physical path
        # while preventing time parameterization and replay from interpreting
        # that representation change as a pointless full wrist revolution.
        full = _continuous_equivalent_joint_path(robot, full)
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
        path_array = np.asarray(full, dtype=float)
        path_cost = sum(
            _joint_motion_time_cost(
                source,
                target,
                planner_options.get("joint_velocity_limits_rad_s"),
            )
            for source, target in zip(path_array[:-1], path_array[1:])
        )
        if release_index > grasp_index:
            carried_centers = np.asarray(
                [
                    (robot.fk(q) @ carton_from_tool)[:3, 3]
                    for q in path_array[grasp_index : release_index + 1]
                ],
                dtype=float,
            )
            task_path_length = float(
                np.sum(np.linalg.norm(np.diff(carried_centers, axis=0), axis=1))
            )
            path_cost += float(
                planner_options.get("carried_task_path_length_weight", 0.0)
            ) * task_path_length
        surface_penalties = planner_options.get("place_surface_penalties", {})
        path_cost += float(surface_penalties.get(place_target_name, 0.0))
        if path_cost < best_pick_cost:
            best_pick_plan = pick_plan
            best_pick_cost = path_cost
        if len(surface_penalties) > 1 and float(surface_penalties.get(place_target_name, 0.0)) <= min(
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
