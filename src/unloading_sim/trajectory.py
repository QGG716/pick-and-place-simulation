"""Collision-validated joint-space corner blending.

The geometric planner intentionally returns piecewise-linear paths.  Industrial
controllers do not stop at every dense collision-check sample, so this module
rounds path corners with C2 quintic Bezier blends and validates every changed
edge before it is accepted.  It remains deterministic and backend-neutral.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np


@dataclass(frozen=True)
class BlendResult:
    path: list[np.ndarray]
    original_waypoints: int
    blended_waypoints: int
    accepted_corners: int
    rejected_corners: int

    def audit(self) -> dict[str, int | str]:
        return {
            "model": "collision_validated_c2_quintic_corner_blend_v1",
            "original_waypoints": self.original_waypoints,
            "blended_waypoints": self.blended_waypoints,
            "accepted_corners": self.accepted_corners,
            "rejected_corners": self.rejected_corners,
        }


def simplify_collinear_joint_path(
    path: Sequence[Sequence[float] | np.ndarray], tolerance: float = 1e-9
) -> list[np.ndarray]:
    """Remove dense samples that lie on the same directed joint-space edge."""
    points = [np.asarray(point, dtype=float).copy() for point in path]
    if len(points) <= 2:
        return points
    simplified = [points[0]]
    for index in range(1, len(points) - 1):
        incoming = points[index] - simplified[-1]
        outgoing = points[index + 1] - points[index]
        incoming_norm = float(np.linalg.norm(incoming))
        outgoing_norm = float(np.linalg.norm(outgoing))
        if incoming_norm <= tolerance or outgoing_norm <= tolerance:
            continue
        same_direction = float(incoming @ outgoing) > 0.0
        residual = outgoing - incoming * float(outgoing @ incoming) / (incoming_norm**2)
        if same_direction and float(np.linalg.norm(residual)) <= tolerance:
            continue
        simplified.append(points[index])
    simplified.append(points[-1])
    return simplified


def _edge_valid(
    a: np.ndarray,
    b: np.ndarray,
    state_valid: Callable[[np.ndarray], bool],
    resolution: float,
) -> bool:
    count = max(1, int(np.ceil(np.max(np.abs(b - a)) / resolution)))
    return all(state_valid(a + (index / count) * (b - a)) for index in range(1, count + 1))


def blend_joint_path(
    path: Sequence[Sequence[float] | np.ndarray],
    state_valid: Callable[[np.ndarray], bool],
    *,
    corner_fraction: float = 0.25,
    samples_per_corner: int = 7,
    edge_resolution: float = 0.02,
) -> BlendResult:
    """Round collision-safe corners without silently changing invalid input.

    ``corner_fraction`` selects entry and exit points on adjacent line
    segments.  Each candidate is a quintic Bezier with position, tangent, and
    zero-curvature continuity at both line junctions.  A corner is retained
    unchanged when any curve sample or connecting edge is invalid.
    """
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or len(points) == 0 or points.shape[1] == 0:
        raise ValueError("path must have shape (N, dof) with at least one waypoint")
    if not np.all(np.isfinite(points)):
        raise ValueError("path contains non-finite values")
    if not 0.0 < corner_fraction < 0.5:
        raise ValueError("corner_fraction must be in (0, 0.5)")
    if samples_per_corner < 3:
        raise ValueError("samples_per_corner must be at least 3")
    if not np.isfinite(edge_resolution) or edge_resolution <= 0.0:
        raise ValueError("edge_resolution must be finite and positive")
    if not state_valid(points[0]):
        raise ValueError("input path starts at an invalid state")
    for index, (a, b) in enumerate(zip(points[:-1], points[1:])):
        if not _edge_valid(a, b, state_valid, edge_resolution):
            raise ValueError(f"input path edge {index}->{index + 1} is invalid")
    if len(points) < 3:
        copied = [point.copy() for point in points]
        return BlendResult(copied, len(points), len(copied), 0, 0)

    output = [points[0].copy()]
    accepted = 0
    rejected = 0
    for index in range(1, len(points) - 1):
        previous, corner, following = points[index - 1 : index + 2]
        entry = corner_fraction * previous + (1.0 - corner_fraction) * corner
        exit_point = (1.0 - corner_fraction) * corner + corner_fraction * following
        incoming = corner - entry
        outgoing = exit_point - corner
        control = np.asarray(
            [
                entry,
                entry + 0.4 * incoming,
                entry + 0.8 * incoming,
                exit_point - 0.8 * outgoing,
                exit_point - 0.4 * outgoing,
                exit_point,
            ]
        )
        parameters = np.linspace(0.0, 1.0, samples_per_corner)
        curve = []
        for t in parameters:
            u = 1.0 - t
            weights = np.array([u**5, 5*u**4*t, 10*u**3*t**2, 10*u**2*t**3, 5*u*t**4, t**5])
            curve.append(weights @ control)
        candidate = [entry, *curve[1:]]
        valid = _edge_valid(output[-1], entry, state_valid, edge_resolution)
        valid = valid and all(
            _edge_valid(a, b, state_valid, edge_resolution) for a, b in zip(candidate[:-1], candidate[1:])
        )
        if valid:
            if not np.allclose(output[-1], entry):
                output.append(entry.copy())
            output.extend(point.copy() for point in curve[1:])
            accepted += 1
        else:
            if not np.allclose(output[-1], corner):
                output.append(corner.copy())
            rejected += 1
    if not _edge_valid(output[-1], points[-1], state_valid, edge_resolution):
        raise RuntimeError("blending invalidated the final connecting edge")
    if not np.allclose(output[-1], points[-1]):
        output.append(points[-1].copy())
    return BlendResult(output, len(points), len(output), accepted, rejected)
