"""Pure geometry policies consumed by the M-710 Isaac replay adapter.

The functions in this module deliberately have no Isaac/Kit dependencies.  The
CPU test suite therefore exercises the exact attachment and conveyor-selection
logic that the heavyweight runtime consumes, even when Isaac is unavailable.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, cos
from typing import Any, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class AttachmentContactAudit:
    """Result of checking every active cup ray against the target carton."""

    accepted: bool
    reason: str | None
    signed_gaps_m: tuple[float | None, ...]
    normal_alignments: tuple[float | None, ...]
    hit_count: int
    within_gap_count: int
    normal_aligned_count: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "reason": self.reason,
            "signed_gaps_m": list(self.signed_gaps_m),
            "normal_alignments": list(self.normal_alignments),
            "hit_count": self.hit_count,
            "within_gap_count": self.within_gap_count,
            "normal_aligned_count": self.normal_aligned_count,
        }


def _rotation(value: Sequence[Sequence[float]], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if (
        result.shape != (3, 3)
        or not np.all(np.isfinite(result))
        or not np.allclose(result.T @ result, np.eye(3), atol=1e-9, rtol=0.0)
        or not np.isclose(np.linalg.det(result), 1.0, atol=1e-9, rtol=0.0)
    ):
        raise ValueError(f"{name} must be a finite proper rotation matrix")
    return result


def audit_surface_attachment_contact(
    *,
    grasp_body_position_m: Sequence[float],
    grasp_body_rotation: Sequence[Sequence[float]],
    contact_plane_from_grasp_body_m: float,
    active_cup_offsets_yz_m: Sequence[Sequence[float]],
    target_center_m: Sequence[float],
    target_rotation: Sequence[Sequence[float]],
    target_size_m: Sequence[float],
    max_attachment_gap_m: float,
    max_normal_misalignment_rad: float,
    maximum_penetration_m: float = 1e-5,
) -> AttachmentContactAudit:
    """Audit physical cup-plane contact without moving either rigid body.

    Cup rays start on the *nominal compressed* physical contact plane and point
    along grasp-body +X.  A negative signed gap means that the nominal contact
    point has already crossed the carton face; only the small explicit
    numerical penetration tolerance is accepted.
    """

    body_position = np.asarray(grasp_body_position_m, dtype=float)
    carton_center = np.asarray(target_center_m, dtype=float)
    carton_size = np.asarray(target_size_m, dtype=float)
    cup_offsets = np.asarray(active_cup_offsets_yz_m, dtype=float)
    body_rotation = _rotation(grasp_body_rotation, "grasp_body_rotation")
    carton_rotation = _rotation(target_rotation, "target_rotation")
    scalars = (
        float(contact_plane_from_grasp_body_m),
        float(max_attachment_gap_m),
        float(max_normal_misalignment_rad),
        float(maximum_penetration_m),
    )
    if body_position.shape != (3,) or carton_center.shape != (3,):
        raise ValueError("body and target centers must contain three values")
    if carton_size.shape != (3,) or np.any(carton_size <= 0.0):
        raise ValueError("target_size_m must contain three positive values")
    if cup_offsets.ndim != 2 or cup_offsets.shape[1] != 2 or len(cup_offsets) == 0:
        raise ValueError("active_cup_offsets_yz_m must have shape (N, 2)")
    if not all(np.all(np.isfinite(value)) for value in (body_position, carton_center, carton_size, cup_offsets)):
        raise ValueError("attachment geometry must be finite")
    if not all(np.isfinite(value) for value in scalars):
        raise ValueError("attachment tolerances and contact-plane offset must be finite")
    if max_attachment_gap_m < 0.0 or maximum_penetration_m < 0.0:
        raise ValueError("attachment gap and penetration tolerance must be non-negative")
    if not 0.0 <= max_normal_misalignment_rad < 0.5 * np.pi:
        raise ValueError("normal misalignment must be in [0, pi/2)")

    half_extents = 0.5 * carton_size
    ray_world = body_rotation[:, 0]
    ray_local = carton_rotation.T @ ray_world
    minimum_alignment = cos(max_normal_misalignment_rad)
    signed_gaps: list[float | None] = []
    alignments: list[float | None] = []

    for offset_y, offset_z in cup_offsets:
        point_body = np.asarray(
            [contact_plane_from_grasp_body_m, float(offset_y), float(offset_z)],
            dtype=float,
        )
        point_world = body_position + body_rotation @ point_body
        point_local = carton_rotation.T @ (point_world - carton_center)
        ray_min = -float("inf")
        ray_max = float("inf")
        entry_axis: int | None = None
        entry_outward_sign = 0.0
        intersects = True
        for axis in range(3):
            direction = float(ray_local[axis])
            if abs(direction) <= 1e-12:
                if abs(float(point_local[axis])) > half_extents[axis] + 1e-12:
                    intersects = False
                    break
                continue
            lower = (-half_extents[axis] - point_local[axis]) / direction
            upper = (half_extents[axis] - point_local[axis]) / direction
            if lower <= upper:
                near, far, outward_sign = lower, upper, -1.0
            else:
                near, far, outward_sign = upper, lower, 1.0
            if near > ray_min:
                ray_min = near
                entry_axis = axis
                entry_outward_sign = outward_sign
            ray_max = min(ray_max, far)
            if ray_min > ray_max + 1e-12:
                intersects = False
                break
        if not intersects or entry_axis is None or ray_max < max(ray_min, 0.0) - 1e-12:
            signed_gaps.append(None)
            alignments.append(None)
            continue
        outward_local = np.zeros(3, dtype=float)
        outward_local[entry_axis] = entry_outward_sign
        outward_world = carton_rotation @ outward_local
        alignment = float(np.clip((-ray_world) @ outward_world, -1.0, 1.0))
        signed_gaps.append(float(ray_min))
        alignments.append(alignment)

    hit_count = sum(value is not None for value in signed_gaps)
    comparison_epsilon_m = 1e-12
    within_gap_count = sum(
        value is not None
        and -maximum_penetration_m - comparison_epsilon_m
        <= value
        <= max_attachment_gap_m + comparison_epsilon_m
        for value in signed_gaps
    )
    normal_aligned_count = sum(
        value is not None and value >= minimum_alignment
        for value in alignments
    )
    cup_count = len(cup_offsets)
    if hit_count != cup_count:
        reason = "ACTIVE_CUP_RAY_MISSES_TARGET"
    elif within_gap_count != cup_count:
        if any(
            value is not None
            and value < -maximum_penetration_m - comparison_epsilon_m
            for value in signed_gaps
        ):
            reason = "PHYSICAL_CONTACT_PLANE_PENETRATES_TARGET"
        else:
            reason = "PHYSICAL_CONTACT_GAP_EXCEEDS_LIMIT"
    elif normal_aligned_count != cup_count:
        reason = "PHYSICAL_CONTACT_NORMAL_MISALIGNED"
    else:
        reason = None
    return AttachmentContactAudit(
        accepted=reason is None,
        reason=reason,
        signed_gaps_m=tuple(signed_gaps),
        normal_alignments=tuple(alignments),
        hit_count=hit_count,
        within_gap_count=within_gap_count,
        normal_aligned_count=normal_aligned_count,
    )


def _horizontal_footprint_contains(
    payload_center_m: np.ndarray,
    primitive: Mapping[str, Any],
    tolerance_m: float,
) -> bool:
    center = np.asarray(primitive.get("center_m", []), dtype=float)
    size = np.asarray(primitive.get("size_m", []), dtype=float)
    rotation = _rotation(primitive.get("rotation_matrix", []), "conveyor rotation")
    if center.shape != (3,) or size.shape != (3,) or np.any(size <= 0.0):
        raise ValueError("conveyor primitives require finite positive OBB geometry")
    local = rotation.T @ (payload_center_m - center)
    # Only the OBB's horizontal footprint decides transfer ownership.  Vertical
    # support/contact remains PhysX's responsibility and is audited separately.
    world_axes = rotation[:, :2]
    if np.any(np.abs(world_axes[2, :]) > 1e-9):
        raise ValueError("conveyor support surfaces must be horizontal")
    return bool(
        abs(float(local[0])) <= 0.5 * float(size[0]) + tolerance_m
        and abs(float(local[1])) <= 0.5 * float(size[1]) + tolerance_m
    )


def select_active_conveyor_surfaces(
    *,
    payload_center_m: Sequence[float] | None,
    conveyor_primitives: Mapping[str, Mapping[str, Any]],
    started: bool,
    exclusive: bool,
    current_surface: str | None = None,
    preferred_initial_surface: str | None = None,
    footprint_tolerance_m: float = 1e-6,
) -> tuple[str, ...]:
    """Select driven belt surfaces from the payload center deterministically.

    In exclusive mode an overlap retains the already active surface.  Without
    that deterministic ownership, all drives are disabled.  This prevents two
    orthogonal surface velocities from pulling the same carton at a transfer.
    """

    if not started:
        return ()
    if footprint_tolerance_m < 0.0 or not np.isfinite(footprint_tolerance_m):
        raise ValueError("footprint_tolerance_m must be finite and non-negative")
    names = tuple(sorted(str(name) for name in conveyor_primitives))
    if not exclusive:
        return names
    if payload_center_m is None:
        return ()
    center = np.asarray(payload_center_m, dtype=float)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("payload_center_m must contain three finite values")
    matching = tuple(
        name
        for name in names
        if _horizontal_footprint_contains(center, conveyor_primitives[name], footprint_tolerance_m)
    )
    if len(matching) == 1:
        return matching
    if len(matching) > 1:
        if current_surface in matching:
            return (str(current_surface),)
        if preferred_initial_surface in matching:
            return (str(preferred_initial_surface),)
    return ()


__all__ = [
    "AttachmentContactAudit",
    "audit_surface_attachment_contact",
    "select_active_conveyor_surfaces",
]
