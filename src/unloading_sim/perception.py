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
) -> list[np.ndarray]:
    """Return stable tool poses with a fully supported attached carton."""
    carton_from_tool = np.linalg.inv(grasp_tool_pose) @ carton.world_from_local
    tool_from_carton = np.linalg.inv(carton_from_tool)
    if release_x_range is None and robot_base_position is not None:
        robot_front_x = float(surface.to_local(np.asarray(robot_base_position, dtype=float))[0])
        release_x_range = (robot_front_x, float(surface.half_extents[0]))
    candidates: list[tuple[float, np.ndarray]] = []
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
        for x_offset in x_offsets:
            for y_offset in y_offsets:
                local_center = np.array(
                    [
                        x_offset,
                        y_offset,
                        surface.half_extents[2] + carton.half_extents[2] + edge_clearance,
                    ]
                )
                desired_carton = make_transform(carton_rotation, surface.to_world(local_center))
                tool_pose = desired_carton @ tool_from_carton
                release_x = float(surface.to_local(tool_pose[:3, 3])[0])
                if release_x_range is not None and not (release_x_range[0] - 1e-9 <= release_x <= release_x_range[1] + 1e-9):
                    continue
                score = (release_x - release_x_range[0] if release_x_range is not None else abs(x_offset)) + 0.2 * abs(y_offset)
                candidates.append((score, tool_pose))
    return [pose for _, pose in sorted(candidates, key=lambda item: item[0])]


def conveyor_place_pose_candidates(
    conveyor: OBB,
    carton: OBB,
    grasp_tool_pose: np.ndarray,
    robot_base_position: np.ndarray,
    edge_clearance: float = 0.01,
    allow_all_carton_faces: bool = False,
    require_front_release_zone: bool = True,
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
    )