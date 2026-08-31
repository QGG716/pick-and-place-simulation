"""Deterministic first-layer carton detection primitives.

This module intentionally models the output of a detector rather than an RGB-D
network.  It exposes detected OBBs in the same world frame used by planning so
the geometric pipeline can be exercised end to end.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations, product
from typing import Sequence

import numpy as np

from .geometry import OBB, make_transform
from .scene import TrailerScene


def controlled_release_height(planner_options: dict) -> float:
    """Return a drop height clamped by configured velocity and energy limits.

    The limits are explicit engineering inputs, not inferred carton damage
    ratings.  When no policy is supplied the legacy fixed release height is
    preserved for backward compatibility.
    """
    policy = planner_options.get("conveyor_release_policy")
    legacy_height = float(planner_options.get("conveyor_release_height", 0.0))
    if policy is None:
        if not np.isfinite(legacy_height) or legacy_height < 0.0:
            raise ValueError("conveyor release height must be finite and non-negative")
        return legacy_height
    desired = float(policy.get("desired_height_m", legacy_height))
    maximum = float(policy.get("maximum_height_m", desired))
    gravity = float(policy.get("gravity_m_s2", 9.81))
    mass = float(planner_options.get("carton_mass_kg", policy.get("payload_mass_kg", 0.0)))
    if any(not np.isfinite(value) or value < 0.0 for value in (desired, maximum, mass)):
        raise ValueError("release height policy values must be finite and non-negative")
    if not np.isfinite(gravity) or gravity <= 0.0:
        raise ValueError("release gravity must be finite and positive")
    limits = [desired, maximum]
    if policy.get("maximum_impact_velocity_m_s") is not None:
        velocity = float(policy["maximum_impact_velocity_m_s"])
        if not np.isfinite(velocity) or velocity < 0.0:
            raise ValueError("maximum impact velocity must be finite and non-negative")
        limits.append(velocity * velocity / (2.0 * gravity))
    if policy.get("maximum_drop_energy_j") is not None:
        energy = float(policy["maximum_drop_energy_j"])
        if not np.isfinite(energy) or energy < 0.0 or mass <= 0.0:
            raise ValueError("drop energy requires positive mass and non-negative energy")
        limits.append(energy / (mass * gravity))
    return max(0.0, float(min(limits)))


@dataclass(frozen=True)
class OBBDetection:
    name: str
    obb: OBB
    score: float
    label: str = "carton"


def detect_carton_obbs(scene: TrailerScene, min_score: float = 0.5) -> list[OBBDetection]:
    detections: list[OBBDetection] = []
    for carton in scene.cartons:
        detections.append(OBBDetection(carton.name, carton, 0.99, "carton"))
    return [detection for detection in detections if detection.score >= min_score]


def select_detection(detections: Sequence[OBBDetection], name: str) -> OBBDetection:
    for detection in detections:
        if detection.name == name:
            return detection
    raise KeyError(f"No detected carton OBB named {name!r}")


def fixed_conveyor_place_pose(
    conveyor: OBB,
    carton: OBB,
    x_margin: float = 0.10,
    z_clearance: float = 0.02,
) -> np.ndarray:
    from .geometry import make_tool_rotation, make_transform

    local = np.array([conveyor.half_extents[0] - x_margin, 0.0, conveyor.half_extents[2] + z_clearance])
    contact = conveyor.to_world(local)
    contact[2] += carton.half_extents[2]
    rotation = make_tool_rotation(np.array([0.0, 0.0, -1.0]))
    return make_transform(rotation, contact)


def surface_place_pose_candidates(
    surface: OBB,
    carton: OBB,
    grasp_tool_pose: np.ndarray,
    robot_base_position: np.ndarray | None = None,
    edge_clearance: float = 0.01,
    release_x_range: tuple[float, float] | None = None,
    allow_all_carton_faces: bool = False,
    release_height: float = 0.0,
    minimum_tool_outward_support_alignment: float | None = None,
    conveyor_transport_direction_world: np.ndarray | None = None,
    minimum_conveyor_separation_alignment: float | None = None,
    prefer_upright_support: bool = False,
    upright_yaw_step_degrees: float | None = None,
) -> list[np.ndarray]:
    """Return stable tool poses above a fully supporting surface.

    ``release_height`` leaves the carton above its settled pose so a replay or
    downstream dynamics backend can model the short free-fall after release.
    Orientations are returned round-robin across the stable carton rotations;
    a bounded planner therefore explores different support faces and wrist
    directions instead of exhausting one rotation first.
    """
    if minimum_tool_outward_support_alignment is not None:
        minimum_tool_outward_support_alignment = float(
            minimum_tool_outward_support_alignment
        )
        if not np.isfinite(minimum_tool_outward_support_alignment) or not (
            -1.0 <= minimum_tool_outward_support_alignment <= 1.0
        ):
            raise ValueError("minimum tool/support alignment must be within [-1, 1]")
    transport_direction = None
    if minimum_conveyor_separation_alignment is not None:
        minimum_conveyor_separation_alignment = float(
            minimum_conveyor_separation_alignment
        )
        if not np.isfinite(minimum_conveyor_separation_alignment) or not (
            -1.0 <= minimum_conveyor_separation_alignment <= 1.0
        ):
            raise ValueError("minimum conveyor separation alignment must be within [-1, 1]")
        transport_direction = np.asarray(conveyor_transport_direction_world, dtype=float)
        if transport_direction.shape != (3,) or not np.all(np.isfinite(transport_direction)):
            raise ValueError("conveyor transport direction must contain three finite values")
        transport_norm = float(np.linalg.norm(transport_direction))
        if transport_norm <= 1e-12:
            raise ValueError("conveyor transport direction must be non-zero")
        transport_direction /= transport_norm
    carton_from_tool = np.linalg.inv(grasp_tool_pose) @ carton.world_from_local
    tool_from_carton = np.linalg.inv(carton_from_tool)
    if release_x_range is None and robot_base_position is not None:
        robot_front_x = float(surface.to_local(np.asarray(robot_base_position, dtype=float))[0])
        release_x_range = (robot_front_x, float(surface.half_extents[0]))
    candidate_groups: list[list[tuple[float, np.ndarray]]] = []
    if allow_all_carton_faces:
        # All proper signed-permutation rotations. Each carton face can then
        # become the downward support face while preserving a right-handed tool pose.
        relative_rotations = []
        for permutation in permutations(range(3)):
            for signs in product((-1.0, 1.0), repeat=3):
                rotation = np.zeros((3, 3))
                rotation[np.arange(3), permutation] = signs
                if np.linalg.det(rotation) > 0.0:
                    relative_rotations.append(rotation)
        if upright_yaw_step_degrees is not None:
            yaw_step = float(upright_yaw_step_degrees)
            if not np.isfinite(yaw_step) or not 0.0 < yaw_step <= 90.0:
                raise ValueError("upright yaw step must be in (0, 90] degrees")
            for yaw_degrees in np.arange(0.0, 360.0, yaw_step):
                yaw = np.deg2rad(yaw_degrees)
                c, s = np.cos(yaw), np.sin(yaw)
                relative_rotations.append(
                    np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
                )
        if prefer_upright_support:
            # Preserve every stable support face as a fallback, but consider
            # yaw-only orientations first. They avoid an unnecessary 90-degree
            # payload roll and its associated wrist/attachment moment.
            relative_rotations.sort(
                key=lambda rotation: -float(rotation[:, 2] @ np.array([0.0, 0.0, 1.0]))
            )
    else:
        relative_rotations = []
        for yaw in (0.0, np.pi / 2.0, np.pi, -np.pi / 2.0):
            c, s = np.cos(yaw), np.sin(yaw)
            relative_rotations.append(np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]))

    seen: set[tuple[float, ...]] = set()
    for relative_rotation in relative_rotations:
        key = tuple(relative_rotation.ravel())
        if key in seen:
            continue
        seen.add(key)
        carton_rotation = surface.rotation @ relative_rotation
        projected_half_extents = np.abs(relative_rotation) @ carton.half_extents
        available = surface.half_extents[:2] - projected_half_extents[:2] - edge_clearance
        if np.any(available < 0.0):
            continue

        tool_offset_local = surface.rotation.T @ (carton_rotation @ tool_from_carton[:3, 3])
        supported_x_min, supported_x_max = -float(available[0]), float(available[0])
        if release_x_range is None:
            release_x_min, release_x_max = supported_x_min, supported_x_max
        else:
            release_x_min = release_x_range[0] - float(tool_offset_local[0])
            release_x_max = release_x_range[1] - float(tool_offset_local[0])
        center_x_min = max(supported_x_min, release_x_min)
        center_x_max = min(supported_x_max, release_x_max)
        if center_x_min > center_x_max:
            continue
        x_offsets = [center_x_max, 0.5 * (center_x_min + center_x_max), center_x_min]
        y_offsets = [0.0]
        if available[1] > 0.04:
            y_offsets.extend([float(available[1]), float(-available[1])])
        rotation_candidates: list[tuple[float, np.ndarray]] = []
        for x_offset in x_offsets:
            for y_offset in y_offsets:
                local_center = np.array(
                    [
                        x_offset,
                        y_offset,
                        surface.half_extents[2] + projected_half_extents[2] + edge_clearance + float(release_height),
                    ]
                )
                desired_carton = make_transform(carton_rotation, surface.to_world(local_center))
                tool_pose = desired_carton @ tool_from_carton
                if minimum_tool_outward_support_alignment is not None:
                    tool_outward = -tool_pose[:3, 2]
                    support_normal = surface.rotation[:, 2]
                    if (
                        float(tool_outward @ support_normal)
                        < minimum_tool_outward_support_alignment
                    ):
                        continue
                if minimum_conveyor_separation_alignment is not None:
                    tool_outward = -tool_pose[:3, 2]
                    if (
                        float((-tool_outward) @ transport_direction)
                        < minimum_conveyor_separation_alignment
                    ):
                        continue
                release_x = float(surface.to_local(tool_pose[:3, 3])[0])
                if release_x_range is not None and not (release_x_range[0] - 1e-9 <= release_x <= release_x_range[1] + 1e-9):
                    continue
                score = (release_x - release_x_range[0] if release_x_range is not None else abs(x_offset)) + 0.2 * abs(y_offset)
                rotation_candidates.append((score, tool_pose))
        if rotation_candidates:
            candidate_groups.append(sorted(rotation_candidates, key=lambda item: item[0]))

    candidates: list[np.ndarray] = []
    for offset_index in range(max((len(group) for group in candidate_groups), default=0)):
        for group in candidate_groups:
            if offset_index < len(group):
                candidates.append(group[offset_index][1])
    return candidates


def conveyor_place_pose_candidates(
    conveyor: OBB,
    carton: OBB,
    grasp_tool_pose: np.ndarray,
    robot_base_position: np.ndarray,
    edge_clearance: float = 0.01,
    allow_all_carton_faces: bool = False,
    require_front_release_zone: bool = True,
    release_height: float = 0.0,
    minimum_tool_outward_support_alignment: float | None = None,
    conveyor_transport_direction_world: np.ndarray | None = None,
    minimum_conveyor_separation_alignment: float | None = None,
    prefer_upright_support: bool = False,
    upright_yaw_step_degrees: float | None = None,
) -> list[np.ndarray]:
    release_x_range = None
    if require_front_release_zone:
        robot_front_x = float(conveyor.to_local(np.asarray(robot_base_position, dtype=float))[0])
        release_x_range = (robot_front_x, float(conveyor.half_extents[0]))
    return surface_place_pose_candidates(
        conveyor,
        carton,
        grasp_tool_pose,
        edge_clearance=edge_clearance,
        release_x_range=release_x_range,
        allow_all_carton_faces=allow_all_carton_faces,
        release_height=release_height,
        minimum_tool_outward_support_alignment=minimum_tool_outward_support_alignment,
        conveyor_transport_direction_world=conveyor_transport_direction_world,
        minimum_conveyor_separation_alignment=minimum_conveyor_separation_alignment,
        prefer_upright_support=prefer_upright_support,
        upright_yaw_step_degrees=upright_yaw_step_degrees,
    )
