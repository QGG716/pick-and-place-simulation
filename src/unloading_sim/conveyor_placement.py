"""Bounded placement sampling and complete support footprints on conveyor unions.

Only NumPy and the core OBB primitive are required.  Convex polygon subtraction
checks the entire footprint, including an unsupported concave union notch that
corner-only checks would miss.  A placement candidate still needs actual loaded
IK, connection, support, release and withdrawal validation by the caller.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import zip_longest
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from .geometry import OBB, rotation_matrix_from_rpy


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _area(polygon: np.ndarray) -> float:
    if len(polygon) < 3:
        return 0.0
    return abs(float(np.dot(polygon[:, 0], np.roll(polygon[:, 1], -1))
                     - np.dot(polygon[:, 1], np.roll(polygon[:, 0], -1)))) / 2


def _halfplane(polygon: np.ndarray, start: np.ndarray, end: np.ndarray, inside: bool) -> np.ndarray:
    """Clip a convex polygon against a directed edge's left or right halfplane."""
    if len(polygon) < 3:
        return np.empty((0, 2))
    edge = end - start
    sign = 1.0 if inside else -1.0
    output = []
    prev = polygon[-1]
    prev_distance = sign * _cross(edge, prev - start)
    for point in polygon:
        distance = sign * _cross(edge, point - start)
        if (distance >= 0) != (prev_distance >= 0):
            fraction = prev_distance / (prev_distance - distance)
            output.append(prev + fraction * (point - prev))
        if distance >= 0:
            output.append(point)
        prev, prev_distance = point, distance
    return np.asarray(output, dtype=float).reshape(-1, 2)


def _intersection(subject: np.ndarray, clip: np.ndarray) -> np.ndarray:
    result = subject
    for start, end in zip(clip, np.roll(clip, -1, axis=0)):
        result = _halfplane(result, start, end, True)
        if len(result) < 3:
            break
    return result


def _subtract(subject: np.ndarray, clip: np.ndarray) -> list[np.ndarray]:
    """Partition subject minus convex clip into nonoverlapping convex pieces."""
    inside = subject
    outside = []
    for start, end in zip(clip, np.roll(clip, -1, axis=0)):
        piece = _halfplane(inside, start, end, False)
        if _area(piece) > 1e-14:
            outside.append(piece)
        inside = _halfplane(inside, start, end, True)
        if _area(inside) <= 1e-14:
            break
    return outside


def _face_corners(box: OBB, z_sign: float, inset: float = 0.0) -> np.ndarray:
    hx, hy, hz = box.half_extents
    if inset >= min(hx, hy):
        return np.empty((0, 3))
    local = np.array([[-hx + inset, -hy + inset, z_sign * hz],
                      [hx - inset, -hy + inset, z_sign * hz],
                      [hx - inset, hy - inset, z_sign * hz],
                      [-hx + inset, hy - inset, z_sign * hz]])
    return local @ box.rotation.T + box.center


@dataclass(frozen=True)
class ConveyorSupport:
    body: OBB
    outlet_direction_world: tuple[float, float, float] | None = None
    properties: Mapping[str, Any] = field(default_factory=dict)

    @property
    def name(self) -> str:
        return self.body.name


def conveyor_support_surfaces(components: Iterable[OBB | ConveyorSupport | Mapping[str, Any]]
                              ) -> tuple[ConveyorSupport, ...]:
    """Select declared conveyor/support roles, never component names or indices."""
    result = []
    for component in components:
        if isinstance(component, ConveyorSupport):
            result.append(component)
            continue
        if isinstance(component, OBB):
            if component.category in {"conveyor", "conveyor_support", "receiver"}:
                result.append(ConveyorSupport(component))
            continue
        role = component.get("role", component.get("category"))
        if role not in {"conveyor", "conveyor_support", "receiver"}:
            continue
        pose = np.asarray(component.get("pose_world", np.eye(4)), dtype=float)
        center = component.get("center_m", pose[:3, 3])
        half = component.get("half_extents_m")
        if half is None:
            raise ValueError("declared conveyor support needs half_extents_m")
        box = OBB(center, half, pose[:3, :3], str(component["name"]), "conveyor")
        direction = component.get("outlet_direction_world", component.get("transport_direction_world"))
        result.append(ConveyorSupport(box, None if direction is None else tuple(direction), dict(component)))
    if len({item.name for item in result}) != len(result):
        raise ValueError("conveyor support identities must be unique")
    return tuple(result)


def _supports(items: Iterable[OBB | ConveyorSupport]) -> tuple[ConveyorSupport, ...]:
    # Explicit supports may be generic OBBs; role filtering belongs at discovery.
    result = tuple(item if isinstance(item, ConveyorSupport) else ConveyorSupport(item) for item in items)
    if len({item.name for item in result}) != len(result):
        raise ValueError("support identities must be unique")
    return result


def support_union_audit(payload: OBB, supports: Sequence[OBB | ConveyorSupport], *,
                        contact_tolerance_m: float = 0.002,
                        edge_tolerance_m: float = 1e-6,
                        engineering_edge_margin_m: float = 0.0) -> dict[str, Any]:
    """Check all actual bottom points are on a fully covering coplanar union.

    ``edge_tolerance_m`` is numerical only.  Rigid support penetration deeper
    than ``contact_tolerance_m`` fails even if XY coverage is complete.
    Tilted actual cartons pass only if every bottom corner is within the stated
    contact band; the projection of the full bottom quad is still audited.
    """
    for value in (contact_tolerance_m, edge_tolerance_m, engineering_edge_margin_m):
        if not np.isfinite(value) or value < 0:
            raise ValueError("support tolerances must be finite and nonnegative")
    items = _supports(supports)
    bottom = _face_corners(payload, -1.0)
    footprint = bottom[:, :2]
    bottom_z = [float(bottom[:, 2].min()), float(bottom[:, 2].max())]
    footprint_area = _area(footprint)
    audit: dict[str, Any] = {
        "schema": "complete_bottom_support_union_v1",
        "actual_box_pose": payload.world_from_local.tolist(),
        "payload_half_extents_m": payload.half_extents.tolist(),
        "support_obbs": [{"name": item.name, "category": item.body.category,
                          "pose_world": item.body.world_from_local.tolist(),
                          "half_extents_m": item.body.half_extents.tolist()} for item in items],
        "supported": False, "reason": "NO_COPLANAR_SUPPORT", "receiver_names": [],
        "footprint_area_m2": footprint_area, "unsupported_area_m2": footprint_area,
        "bottom_z_range_m": bottom_z, "support_z_m": None, "plane_error_m": None,
        "tolerance_m": contact_tolerance_m, "edge_tolerance_m": edge_tolerance_m,
        "engineering_edge_margin_m": engineering_edge_margin_m,
        "coverage_method": "complete_convex_footprint_minus_support_polygon_union",
    }
    if footprint_area <= 1e-14 or payload.rotation[2, 2] <= 0:
        audit["reason"] = "NO_HORIZONTAL_BOTTOM_FOOTPRINT"
        return audit
    eligible: list[tuple[ConveyorSupport, np.ndarray, float]] = []
    for item in items:
        if not np.allclose(item.body.rotation[:, 2], [0, 0, 1], atol=1e-9, rtol=0):
            continue
        corners = _face_corners(item.body, 1.0, engineering_edge_margin_m - edge_tolerance_m)
        if not len(corners):
            continue
        surface_z = float(corners[0, 2])
        if (bottom_z[0] < surface_z - contact_tolerance_m - 1e-12
            and _area(_intersection(footprint, corners[:, :2])) > 1e-12):
            audit.update(reason="SUPPORT_PENETRATION", plane_error_m=surface_z - bottom_z[0])
            return audit
        if max(abs(bottom_z[0] - surface_z), abs(bottom_z[1] - surface_z)) <= contact_tolerance_m + 1e-12:
            eligible.append((item, corners[:, :2], surface_z))
    if not eligible:
        # Distinguish actual excessive penetration from a carton still hovering.
        for item in items:
            top = _face_corners(item.body, 1)
            if _area(_intersection(footprint, top[:, :2])) > 1e-14:
                z = float(top[:, 2].max())
                if bottom_z[0] < z - contact_tolerance_m:
                    audit["reason"] = "SUPPORT_PENETRATION"
                    audit["plane_error_m"] = z - bottom_z[0]
                    return audit
        return audit
    # Never combine two different height planes into an invented flat support.
    groups: list[list[tuple[ConveyorSupport, np.ndarray, float]]] = []
    for entry in sorted(eligible, key=lambda item: (item[2], item[0].name)):
        if not groups or abs(entry[2] - groups[-1][0][2]) > 1e-8:
            groups.append([])
        groups[-1].append(entry)
    best = None
    for group in groups:
        remaining = [footprint]
        involved = []
        for item, polygon, _ in group:
            if _area(_intersection(footprint, polygon)) > 1e-14:
                involved.append(item.name)
            remaining = [piece for region in remaining for piece in _subtract(region, polygon)]
        uncovered = sum(_area(region) for region in remaining)
        if best is None or uncovered < best[0]:
            best = (uncovered, involved, group[0][2])
    assert best is not None
    uncovered, names, surface_z = best
    audit.update(unsupported_area_m2=float(uncovered), receiver_names=names, support_z_m=surface_z,
                 plane_error_m=max(abs(bottom_z[0] - surface_z), abs(bottom_z[1] - surface_z)))
    # Linear numerical tolerance has already expanded the support polygons;
    # only machine-roundoff area remains permissible here.
    audit["supported"] = uncovered <= max(1e-12, footprint_area * 1e-10)
    audit["reason"] = "SUPPORTED" if audit["supported"] else "UNSUPPORTED_FOOTPRINT"
    return audit


@dataclass(frozen=True)
class PlacementPolicy:
    yaw_offsets_rad: tuple[float, ...] = (0.0, np.pi / 2, -np.pi / 2)
    coarse_samples_per_axis: int = 3
    fine_samples_per_axis: int = 7
    maximum_candidates: int = 24
    edge_tolerance_m: float = 1e-6
    engineering_edge_margin_m: float = 0.0
    occupancy_clearance_m: float = 0.02
    contact_tolerance_m: float = 0.002

    def __post_init__(self) -> None:
        if not self.yaw_offsets_rad or not np.all(np.isfinite(self.yaw_offsets_rad)):
            raise ValueError("placement needs finite yaw offsets")
        if self.coarse_samples_per_axis < 2 or self.fine_samples_per_axis < self.coarse_samples_per_axis:
            raise ValueError("placement sampling needs a coarse grid and a finer grid")
        if self.maximum_candidates < 1:
            raise ValueError("placement candidate budget must be positive")
        for name in ("edge_tolerance_m", "engineering_edge_margin_m", "occupancy_clearance_m", "contact_tolerance_m"):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class PlacementCandidate:
    payload: OBB
    receiver_names: tuple[str, ...]
    support: Mapping[str, Any]
    reasons: tuple[str, ...]
    yaw_rad: float
    score: float
    outlet_directions: Mapping[str, tuple[float, float, float] | None]

    @property
    def target_obb(self) -> OBB:
        return self.payload

    def as_dict(self) -> dict[str, Any]:
        return {"target_name": self.payload.name, "target_world": self.payload.world_from_local.tolist(),
                "half_extents_m": self.payload.half_extents.tolist(),
                "receiver_names": list(self.receiver_names), "support": dict(self.support),
                "reasons": list(self.reasons), "yaw_rad": self.yaw_rad, "score": self.score,
                "outlet_directions": dict(self.outlet_directions)}


def _placement_stream(target: OBB, family: tuple[ConveyorSupport, ...], all_supports: tuple[ConveyorSupport, ...],
                      occupied: tuple[OBB, ...], preferred: np.ndarray, policy: PlacementPolicy
                      ) -> Iterable[PlacementCandidate]:
    points = np.concatenate([_face_corners(item.body, 1) for item in family])
    lower, upper = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    z = float(points[0, 2])
    original_yaw = float(np.arctan2(target.rotation[1, 0], target.rotation[0, 0]))
    scale = max(float(np.linalg.norm(upper - lower)), 1e-6)
    seen = set()
    for count in (policy.coarse_samples_per_axis, policy.fine_samples_per_axis):
        # Visit every allowed yaw in each coarse/fine round; later families are
        # interleaved by the public generator, so neither conveyor is starved.
        yaw_grids = []
        for offset in policy.yaw_offsets_rad:
            yaw = original_yaw + offset
            rotation = rotation_matrix_from_rpy(0, 0, yaw)
            relative = _face_corners(OBB(np.zeros(3), target.half_extents, rotation), -1)[:, :2]
            low_center = lower - relative.min(axis=0)
            high_center = upper - relative.max(axis=0)
            if np.any(low_center > high_center + policy.edge_tolerance_m):
                continue
            grid = [np.clip(preferred[:2], low_center, high_center), 0.5 * (low_center + high_center)]
            grid.extend(np.array([x, y]) for x in np.linspace(low_center[0], high_center[0], count)
                        for y in np.linspace(low_center[1], high_center[1], count))
            grid.sort(key=lambda xy: float(np.linalg.norm(xy - preferred[:2])))
            yaw_grids.append([(xy, yaw, rotation, offset) for xy in grid])
        for batch in zip_longest(*yaw_grids):
            for entry in batch:
                if entry is None:
                    continue
                xy, yaw, rotation, offset = entry
                key = (*np.round(xy, 9), round(float(np.cos(yaw)), 9), round(float(np.sin(yaw)), 9))
                if key in seen:
                    continue
                seen.add(key)
                payload = OBB([*xy, z + target.half_extents[2]], target.half_extents.copy(), rotation,
                              target.name, target.category)
                audit = support_union_audit(payload, family, contact_tolerance_m=policy.contact_tolerance_m,
                                            edge_tolerance_m=policy.edge_tolerance_m,
                                            engineering_edge_margin_m=policy.engineering_edge_margin_m)
                if not audit["supported"]:
                    continue
                if any(payload.signed_distance_obb(other) < policy.occupancy_clearance_m - 1e-10
                       for other in occupied if other.name != target.name):
                    continue
                names = tuple(audit["receiver_names"])
                distance = float(np.linalg.norm(xy - preferred[:2]))
                yield PlacementCandidate(
                    payload, names, audit,
                    ("COMPLETE_BOTTOM_SUPPORT", "CURRENT_OCCUPANCY_CLEAR",
                     "BOTH_CONVEYORS_SHARE_SAMPLING_BUDGET", "PRESERVE_YAW" if abs(offset) < 1e-10 else "BOUNDED_YAW_ALTERNATIVE"),
                    yaw, distance / scale + 0.1 * abs(offset) / np.pi,
                    {item.name: item.outlet_direction_world for item in all_supports if item.name in names})


def generate_conveyor_placements(target: OBB, supports: Sequence[OBB | ConveyorSupport], *,
                                 occupied: Iterable[OBB] = (), preferred_point_world: Sequence[float] | None = None,
                                 policy: PlacementPolicy | None = None) -> tuple[PlacementCandidate, ...]:
    """Return a bounded fair sequence, including noncentral and joint-support poses.

    Every live single-belt family gets a turn before the union gets another;
    callers can continue to the next candidate after failed IK or connection.
    No receiver position or orientation is forced by a component name.
    """
    policy = policy or PlacementPolicy()
    items = _supports(supports)
    if not items:
        return ()
    if policy.maximum_candidates < len(items):
        raise ValueError("placement budget must reserve at least one candidate for each support")
    preferred = np.asarray(target.center if preferred_point_world is None else preferred_point_world, dtype=float)
    if preferred.shape != (3,) or not np.all(np.isfinite(preferred)):
        raise ValueError("preferred placement point must be a finite world xyz")
    horizontal = tuple(item for item in items if np.allclose(item.body.rotation[:, 2], [0, 0, 1], atol=1e-9, rtol=0))
    families = [(item,) for item in horizontal]
    groups: list[list[ConveyorSupport]] = []
    for item in sorted(horizontal, key=lambda entry: float(_face_corners(entry.body, 1)[0, 2])):
        z = _face_corners(item.body, 1)[0, 2]
        if not groups or abs(z - _face_corners(groups[-1][0].body, 1)[0, 2]) > 1e-8:
            groups.append([])
        groups[-1].append(item)
    families.extend(tuple(group) for group in groups if len(group) > 1)
    occupancy = tuple(occupied)
    streams = [iter(_placement_stream(target, family, horizontal, occupancy, preferred, policy)) for family in families]
    result = []
    seen = set()
    while streams and len(result) < policy.maximum_candidates:
        live = []
        for stream in streams:
            for candidate in stream:
                key = tuple(np.round(candidate.payload.world_from_local.flatten(), 8))
                if key not in seen:
                    seen.add(key)
                    result.append(candidate)
                    live.append(stream)
                    break
            if len(result) >= policy.maximum_candidates:
                break
        streams = live
    return tuple(result)
