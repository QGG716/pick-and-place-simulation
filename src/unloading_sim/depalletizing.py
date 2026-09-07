"""Deterministic local-stack and independent conveyor geometry.

This module contains no robot backend.  It turns carton OBB topology into
grasp/extraction candidates and models the two commanded degrees of freedom of
an independent L conveyor.  Robot IK and path planning consume these results;
they do not define them.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import inf
from typing import Iterable, Sequence

import numpy as np

from .geometry import OBB


FACE_OUTWARD_NORMALS: dict[str, np.ndarray] = {
    "front": np.array([-1.0, 0.0, 0.0]),
    "left": np.array([0.0, 1.0, 0.0]),
    "right": np.array([0.0, -1.0, 0.0]),
    "top": np.array([0.0, 0.0, 1.0]),
}


def _world_interval(box: OBB, axis: int) -> tuple[float, float]:
    values = box.corners()[:, axis]
    return float(np.min(values)), float(np.max(values))


def _interval_overlap(a: tuple[float, float], b: tuple[float, float]) -> float:
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


def _interval_gap(a: tuple[float, float], b: tuple[float, float]) -> float:
    return max(0.0, max(a[0], b[0]) - min(a[1], b[1]))


def _overlap_fraction(target: OBB, other: OBB, axes: Iterable[int]) -> float:
    fractions = []
    for axis in axes:
        target_interval = _world_interval(target, axis)
        other_interval = _world_interval(other, axis)
        denominator = min(target_interval[1] - target_interval[0], other_interval[1] - other_interval[0])
        fractions.append(_interval_overlap(target_interval, other_interval) / max(denominator, 1e-12))
    return min(fractions, default=0.0)


@dataclass(frozen=True)
class BoxNeighborhoodState:
    """Local carton topology derived from projected overlap and face gaps."""

    target_name: str
    left_neighbor: str | None
    right_neighbor: str | None
    top_neighbor: str | None
    bottom_support: str | None
    front_clearance: float
    exposed_left_face: bool
    exposed_right_face: bool
    exposed_front_face: bool
    exposed_top_face: bool

    @property
    def constrained_both_sides(self) -> bool:
        return self.left_neighbor is not None and self.right_neighbor is not None

    @property
    def exposed_face_count(self) -> int:
        return sum(
            (
                self.exposed_left_face,
                self.exposed_right_face,
                self.exposed_front_face,
                self.exposed_top_face,
            )
        )


def analyze_box_neighborhood(
    target: OBB,
    boxes: Sequence[OBB],
    *,
    adjacency_gap_m: float = 0.03,
    minimum_projection_overlap: float = 0.20,
) -> BoxNeighborhoodState:
    """Classify face neighbours using projected OBB extents and face gaps.

    The method is exact for the axis-aligned cartons used by the acceptance
    scenes and conservative for yawed OBBs because their world projections are
    used.  Centre distance alone is never used as an adjacency test.
    """

    intervals = [_world_interval(target, axis) for axis in range(3)]
    relations: dict[str, tuple[float, str] | None] = {
        "left": None,
        "right": None,
        "top": None,
        "bottom": None,
    }
    front_clearance = inf
    for other in boxes:
        if other.name == target.name:
            continue
        other_intervals = [_world_interval(other, axis) for axis in range(3)]

        overlap_xz = _overlap_fraction(target, other, (0, 2))
        if overlap_xz >= minimum_projection_overlap:
            if other_intervals[1][0] >= intervals[1][1] - 1e-9:
                gap = other_intervals[1][0] - intervals[1][1]
                if gap <= adjacency_gap_m and (relations["left"] is None or gap < relations["left"][0]):
                    relations["left"] = (gap, other.name)
            if other_intervals[1][1] <= intervals[1][0] + 1e-9:
                gap = intervals[1][0] - other_intervals[1][1]
                if gap <= adjacency_gap_m and (relations["right"] is None or gap < relations["right"][0]):
                    relations["right"] = (gap, other.name)

        overlap_xy = _overlap_fraction(target, other, (0, 1))
        if overlap_xy >= minimum_projection_overlap:
            if other_intervals[2][0] >= intervals[2][1] - 1e-9:
                gap = other_intervals[2][0] - intervals[2][1]
                if gap <= adjacency_gap_m and (relations["top"] is None or gap < relations["top"][0]):
                    relations["top"] = (gap, other.name)
            if other_intervals[2][1] <= intervals[2][0] + 1e-9:
                gap = intervals[2][0] - other_intervals[2][1]
                if gap <= adjacency_gap_m and (relations["bottom"] is None or gap < relations["bottom"][0]):
                    relations["bottom"] = (gap, other.name)

        if _overlap_fraction(target, other, (1, 2)) >= minimum_projection_overlap:
            # The accessible trailer face is the target's -X face.
            if other_intervals[0][1] <= intervals[0][0] + 1e-9:
                front_clearance = min(front_clearance, intervals[0][0] - other_intervals[0][1])

    name = lambda key: None if relations[key] is None else relations[key][1]
    left, right, top, bottom = name("left"), name("right"), name("top"), name("bottom")
    return BoxNeighborhoodState(
        target_name=target.name,
        left_neighbor=left,
        right_neighbor=right,
        top_neighbor=top,
        bottom_support=bottom,
        front_clearance=front_clearance,
        exposed_left_face=left is None,
        exposed_right_face=right is None,
        exposed_front_face=front_clearance > adjacency_gap_m,
        exposed_top_face=top is None,
    )


@dataclass(frozen=True)
class ExtractionCandidate:
    grasp_face: str
    outward_direction_world: np.ndarray
    requires_local_linear_extraction: bool
    strategy: str

    def __post_init__(self) -> None:
        direction = np.asarray(self.outward_direction_world, dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            raise ValueError("extraction direction must be non-zero")
        object.__setattr__(self, "outward_direction_world", direction / norm)


def generate_extraction_candidates(state: BoxNeighborhoodState) -> tuple[ExtractionCandidate, ...]:
    """Generate topology-aware candidates without assigning top priority."""

    def candidate(face: str, linear: bool, strategy: str) -> ExtractionCandidate:
        return ExtractionCandidate(face, FACE_OUTWARD_NORMALS[face], linear, strategy)

    result: list[ExtractionCandidate] = []
    if state.constrained_both_sides:
        if state.exposed_front_face:
            result.append(candidate("front", True, "front_minimum_clearance"))
        if state.exposed_top_face:
            result.append(candidate("top", True, "top_minimum_clearance"))
    elif state.left_neighbor is not None and state.exposed_right_face:
        result.append(candidate("right", False, "open_side_direct"))
        if state.exposed_front_face:
            result.append(candidate("front", True, "front_fallback"))
        if state.exposed_top_face:
            result.append(candidate("top", True, "top_candidate"))
    elif state.right_neighbor is not None and state.exposed_left_face:
        result.append(candidate("left", False, "open_side_direct"))
        if state.exposed_front_face:
            result.append(candidate("front", True, "front_fallback"))
        if state.exposed_top_face:
            result.append(candidate("top", True, "top_candidate"))
    else:
        for face, exposed in (
            ("front", state.exposed_front_face),
            ("left", state.exposed_left_face),
            ("right", state.exposed_right_face),
            ("top", state.exposed_top_face),
        ):
            if exposed:
                result.append(candidate(face, face in {"front", "top"}, "multi_face_candidate"))
    return tuple(result)


def _translated(box: OBB, direction: np.ndarray, distance_m: float) -> OBB:
    return OBB(
        box.center + np.asarray(direction, dtype=float) * float(distance_m),
        box.half_extents,
        box.rotation,
        box.name,
        box.category,
    )


def box_is_free_from_stack(
    moved_target: OBB,
    constraining_neighbors: Sequence[OBB],
    *,
    free_space_clearance_m: float = 0.02,
) -> bool:
    """True once every local constraint has a real surface-space escape axis."""

    for other in constraining_neighbors:
        gaps = [
            _interval_gap(_world_interval(moved_target, axis), _world_interval(other, axis))
            for axis in range(3)
        ]
        if max(gaps) + 1e-12 < free_space_clearance_m:
            return False
    return True


def minimum_clearance_extraction_distance(
    target: OBB,
    direction_world: Sequence[float],
    constraining_neighbors: Sequence[OBB],
    *,
    free_space_clearance_m: float = 0.02,
    scan_step_m: float = 0.005,
    maximum_distance_m: float = 1.50,
) -> float | None:
    """Incrementally find and refine the first locally-free box translation."""

    direction = np.asarray(direction_world, dtype=float)
    direction /= np.linalg.norm(direction)
    if box_is_free_from_stack(target, constraining_neighbors, free_space_clearance_m=free_space_clearance_m):
        return 0.0
    previous = 0.0
    distance = scan_step_m
    while distance <= maximum_distance_m + 1e-12:
        moved = _translated(target, direction, distance)
        if box_is_free_from_stack(moved, constraining_neighbors, free_space_clearance_m=free_space_clearance_m):
            low, high = previous, distance
            for _ in range(24):
                mid = 0.5 * (low + high)
                if box_is_free_from_stack(
                    _translated(target, direction, mid),
                    constraining_neighbors,
                    free_space_clearance_m=free_space_clearance_m,
                ):
                    high = mid
                else:
                    low = mid
            return high
        previous = distance
        distance += scan_step_m
    return None


@dataclass(frozen=True)
class CandidateMetrics:
    grasp_feasible: bool
    ik_margin: float
    joint_limit_margin: float
    singularity_margin: float
    robot_collision_clearance_m: float
    tool_collision_clearance_m: float
    target_box_collision_clearance_m: float
    neighbor_box_clearance_m: float
    extraction_distance_m: float
    loaded_path_length_m: float
    tcp_path_length_m: float
    joint_space_path_length_rad: float
    conveyor_handoff_distance_m: float
    wrist_load_inertia_margin: float


@dataclass(frozen=True)
class CandidateCostWeights:
    loaded_path_length: float = 4.0
    extraction_distance: float = 2.0
    joint_motion: float = 1.0
    collision_risk: float = 8.0
    singularity_penalty: float = 5.0
    joint_limit_penalty: float = 5.0
    conveyor_handoff_distance: float = 2.0
    wrist_margin_penalty: float = 4.0


def score_candidate(metrics: CandidateMetrics, weights: CandidateCostWeights = CandidateCostWeights()) -> float:
    """Score a complete candidate; infeasible candidates sort last."""

    if not metrics.grasp_feasible:
        return inf
    clearances = (
        metrics.robot_collision_clearance_m,
        metrics.tool_collision_clearance_m,
        metrics.target_box_collision_clearance_m,
        metrics.neighbor_box_clearance_m,
    )
    collision_risk = sum(1.0 / max(value, 0.001) for value in clearances)
    singularity_penalty = 1.0 / max(metrics.singularity_margin, 1e-6)
    joint_limit_penalty = 1.0 / max(metrics.joint_limit_margin, 1e-6)
    wrist_penalty = 1.0 / max(metrics.wrist_load_inertia_margin, 1e-6)
    return float(
        weights.loaded_path_length * metrics.loaded_path_length_m
        + weights.extraction_distance * metrics.extraction_distance_m
        + weights.joint_motion * metrics.joint_space_path_length_rad
        + weights.collision_risk * collision_risk
        + weights.singularity_penalty * singularity_penalty
        + weights.joint_limit_penalty * joint_limit_penalty
        + weights.conveyor_handoff_distance * metrics.conveyor_handoff_distance_m
        + weights.wrist_margin_penalty * wrist_penalty
    )


@dataclass(frozen=True)
class LConveyorGeometry:
    """L conveyor dimensions in the robot-base-aligned chassis frame.

    +X is into the trailer, +Y is chassis-left and +Z is up.  The cross leg's
    rear edge touches the chassis front (+X).  The longitudinal leg's right
    edge touches the chassis right (-Y) and extends toward +Y, making the conveyor independent but
    topologically adjacent to the mobile base.
    """

    chassis_size_xyz_m: tuple[float, float, float] = (1.80, 2.00, 0.30)
    chassis_center_from_robot_base_xyz_m: tuple[float, float, float] = (-1.00, -0.075, -0.45)
    cross_leg_size_xy_m: tuple[float, float] = (0.45, 2.00)
    longitudinal_leg_nominal_length_m: float = 0.65
    longitudinal_leg_width_m: float = 0.70
    deck_thickness_m: float = 0.12
    maximum_extension_m: float = 1.50
    minimum_stack_surface_clearance_m: float = 0.02
    minimum_receiving_surface_z_m: float = 0.20
    upper_stack_bottom_clearance_m: float = 0.015

    def __post_init__(self) -> None:
        values = np.asarray(
            [
                *self.chassis_size_xyz_m,
                *self.cross_leg_size_xy_m,
                self.longitudinal_leg_nominal_length_m,
                self.longitudinal_leg_width_m,
                self.deck_thickness_m,
                self.maximum_extension_m,
            ],
            dtype=float,
        )
        if np.any(values <= 0.0):
            raise ValueError("chassis and conveyor dimensions must be positive")

    def base_from_chassis(self) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, 3] = -np.asarray(self.chassis_center_from_robot_base_xyz_m, dtype=float)
        return transform

    def chassis_from_base(self) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, 3] = np.asarray(self.chassis_center_from_robot_base_xyz_m, dtype=float)
        return transform

    def obstacles(
        self,
        robot_base_world_xyz_m: Sequence[float],
        conveyor_extension_m: float,
        conveyor_z_m: float,
    ) -> tuple[OBB, OBB]:
        extension = float(conveyor_extension_m)
        if extension < -1e-12 or extension > self.maximum_extension_m + 1e-12:
            raise ValueError("conveyor_extension is outside configured travel")
        base = np.asarray(robot_base_world_xyz_m, dtype=float)
        chassis_center = base + np.asarray(self.chassis_center_from_robot_base_xyz_m, dtype=float)
        chassis_half = 0.5 * np.asarray(self.chassis_size_xyz_m, dtype=float)
        cross_depth, cross_width = self.cross_leg_size_xy_m
        chassis_front_x = chassis_center[0] + chassis_half[0]
        chassis_right_y = chassis_center[1] - chassis_half[1]
        cross_center = np.array(
            [chassis_front_x + 0.5 * cross_depth, chassis_center[1], conveyor_z_m - 0.5 * self.deck_thickness_m]
        )
        length = self.longitudinal_leg_nominal_length_m + extension
        longitudinal_center = np.array(
            [
                chassis_front_x + cross_depth + 0.5 * length,
                chassis_right_y + 0.5 * self.longitudinal_leg_width_m,
                conveyor_z_m - 0.5 * self.deck_thickness_m,
            ]
        )
        return (
            OBB(cross_center, 0.5 * np.array([cross_depth, cross_width, self.deck_thickness_m]), np.eye(3), "conveyor_cross_leg", "conveyor"),
            OBB(longitudinal_center, 0.5 * np.array([length, self.longitudinal_leg_width_m, self.deck_thickness_m]), np.eye(3), "conveyor_longitudinal_leg", "conveyor"),
        )

    def receiving_front_x_m(self, robot_base_world_xyz_m: Sequence[float], conveyor_extension_m: float) -> float:
        base = np.asarray(robot_base_world_xyz_m, dtype=float)
        chassis_center = base + np.asarray(self.chassis_center_from_robot_base_xyz_m, dtype=float)
        return float(
            chassis_center[0]
            + 0.5 * self.chassis_size_xyz_m[0]
            + self.cross_leg_size_xy_m[0]
            + self.longitudinal_leg_nominal_length_m
            + conveyor_extension_m
        )


@dataclass(frozen=True)
class ConveyorPose:
    conveyor_extension_m: float
    conveyor_z_m: float
    receiving_front_x_m: float
    target_surface_clearance_m: float
    obstacles: tuple[OBB, OBB]


@dataclass
class HandoffProtocol:
    """Fail-closed vacuum release gate for a conveyor handoff."""

    box_supported: bool = False
    handoff_pose_collision_free: bool = False
    vacuum_released: bool = False

    def confirm_supported_handoff(self, *, box_supported: bool, collision_free: bool) -> None:
        self.box_supported = bool(box_supported)
        self.handoff_pose_collision_free = bool(collision_free)

    def release_vacuum(self) -> None:
        if not self.box_supported or not self.handoff_pose_collision_free:
            raise RuntimeError("vacuum release requires a supported collision-free conveyor handoff")
        self.vacuum_released = True


def dynamic_conveyor_z_limits(
    remaining_boxes: Sequence[OBB],
    geometry: LConveyorGeometry,
    *,
    trailer_floor_z_m: float = 0.0,
) -> tuple[float, float]:
    minimum = trailer_floor_z_m + geometry.minimum_receiving_surface_z_m
    if not remaining_boxes:
        return minimum, minimum
    highest = max(remaining_boxes, key=lambda box: _world_interval(box, 2)[1])
    highest_bottom = _world_interval(highest, 2)[0]
    maximum = highest_bottom - geometry.upper_stack_bottom_clearance_m
    return minimum, maximum


def plan_conveyor_preposition(
    geometry: LConveyorGeometry,
    robot_base_world_xyz_m: Sequence[float],
    target: OBB,
    remaining_boxes: Sequence[OBB],
    collision_obstacles: Sequence[OBB],
    *,
    robot_capsules: Sequence[object] = (),
    collision_margin_m: float = 0.0,
    extension_step_m: float = 0.01,
    z_step_m: float = 0.01,
) -> ConveyorPose | None:
    """Jointly choose extension and height, rejecting every colliding pose."""

    target_front = _world_interval(target, 0)[0]
    nominal_front = geometry.receiving_front_x_m(robot_base_world_xyz_m, 0.0)
    desired_extension = np.clip(
        target_front - geometry.minimum_stack_surface_clearance_m - nominal_front,
        0.0,
        geometry.maximum_extension_m,
    )
    minimum_z, maximum_z = dynamic_conveyor_z_limits(remaining_boxes, geometry)
    if maximum_z < minimum_z - 1e-12:
        return None
    target_bottom = _world_interval(target, 2)[0]
    desired_z = float(np.clip(target_bottom, minimum_z, maximum_z))

    extension_values = np.arange(desired_extension, -0.5 * extension_step_m, -extension_step_m)
    z_offsets = [0.0]
    span_steps = int(np.ceil((maximum_z - minimum_z) / z_step_m))
    for index in range(1, span_steps + 1):
        z_offsets.extend((-index * z_step_m, index * z_step_m))
    for extension in extension_values:
        for offset in z_offsets:
            z = desired_z + offset
            if z < minimum_z - 1e-12 or z > maximum_z + 1e-12:
                continue
            conveyor = geometry.obstacles(robot_base_world_xyz_m, float(max(extension, 0.0)), float(z))
            if any(
                deck.intersects_obb(obstacle, margin=collision_margin_m)
                for deck in conveyor
                for obstacle in collision_obstacles
            ):
                continue
            if any(
                getattr(capsule, "collides_obb")(deck, margin=collision_margin_m)
                for capsule in robot_capsules
                for deck in conveyor
            ):
                continue
            front = geometry.receiving_front_x_m(robot_base_world_xyz_m, float(max(extension, 0.0)))
            return ConveyorPose(
                conveyor_extension_m=float(max(extension, 0.0)),
                conveyor_z_m=float(z),
                receiving_front_x_m=front,
                target_surface_clearance_m=float(target_front - front),
                obstacles=conveyor,
            )
    return None
