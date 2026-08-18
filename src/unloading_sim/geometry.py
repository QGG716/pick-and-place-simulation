"""Lightweight geometric primitives used by the layer-1 simulator.

The first layer intentionally avoids a full rigid-body engine.  Trailer walls and
cartons are represented by oriented bounding boxes (OBBs); robot links are
represented by capsules.  This gives fast, deterministic collision queries that
are suitable for reachability sweeps and motion-planning regression tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

_EPS = 1e-12


def normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < _EPS:
        raise ValueError("Cannot normalize a near-zero vector")
    return v / n


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def make_transform(rotation: np.ndarray | None = None, translation: Iterable[float] = (0, 0, 0)) -> np.ndarray:
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = np.asarray(rotation, dtype=float)
    t[:3, 3] = np.asarray(translation, dtype=float)
    return t


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


@dataclass(frozen=True)
class OBB:
    """Oriented bounding box.

    Parameters are expressed in world coordinates.  ``rotation`` maps local box
    coordinates to world coordinates.
    """

    center: np.ndarray
    half_extents: np.ndarray
    rotation: np.ndarray
    name: str = "obb"
    category: str = "obstacle"

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", np.asarray(self.center, dtype=float))
        object.__setattr__(self, "half_extents", np.asarray(self.half_extents, dtype=float))
        object.__setattr__(self, "rotation", np.asarray(self.rotation, dtype=float))
        if self.center.shape != (3,):
            raise ValueError("center must be shape (3,)")
        if self.half_extents.shape != (3,) or np.any(self.half_extents <= 0):
            raise ValueError("half_extents must be positive shape (3,)")
        if self.rotation.shape != (3, 3):
            raise ValueError("rotation must be shape (3,3)")

    @property
    def world_from_local(self) -> np.ndarray:
        return make_transform(self.rotation, self.center)

    @property
    def local_from_world(self) -> np.ndarray:
        t = np.eye(4)
        t[:3, :3] = self.rotation.T
        t[:3, 3] = -self.rotation.T @ self.center
        return t

    def to_local(self, point: np.ndarray) -> np.ndarray:
        return self.rotation.T @ (np.asarray(point, dtype=float) - self.center)

    def to_world(self, point: np.ndarray) -> np.ndarray:
        return self.rotation @ np.asarray(point, dtype=float) + self.center

    def corners(self) -> np.ndarray:
        signs = np.array(
            [[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]
        )
        local = signs * self.half_extents
        return (self.rotation @ local.T).T + self.center

    def point_distance_squared(self, point_world: np.ndarray) -> float:
        p = self.to_local(point_world)
        excess = np.maximum(np.abs(p) - self.half_extents, 0.0)
        return float(excess @ excess)

    def contains(self, point_world: np.ndarray, margin: float = 0.0) -> bool:
        p = np.abs(self.to_local(point_world))
        return bool(np.all(p <= self.half_extents + margin))

    def segment_intersects(self, p0_world: np.ndarray, p1_world: np.ndarray, inflation: float = 0.0) -> bool:
        """Slab test in the box frame, optionally with isotropic inflation."""
        p0 = self.to_local(p0_world)
        p1 = self.to_local(p1_world)
        d = p1 - p0
        lo = -self.half_extents - inflation
        hi = self.half_extents + inflation
        t_min, t_max = 0.0, 1.0
        for i in range(3):
            if abs(d[i]) < _EPS:
                if p0[i] < lo[i] or p0[i] > hi[i]:
                    return False
                continue
            inv = 1.0 / d[i]
            t1 = (lo[i] - p0[i]) * inv
            t2 = (hi[i] - p0[i]) * inv
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
            if t_min > t_max:
                return False
        return True

    def segment_distance_squared(self, p0_world: np.ndarray, p1_world: np.ndarray, iterations: int = 24) -> float:
        """Return the exact squared distance from a segment to this OBB.

        In the box frame, point-to-box squared distance along the segment is a
        convex piecewise quadratic.  Its active coordinates can only change
        where the segment crosses one of the six box planes, so evaluating the
        quadratic minimum on each resulting interval is exact and avoids an
        iterative optimizer in the collision hot path. ``iterations`` remains
        accepted for backward compatibility and is intentionally unused.
        """
        del iterations
        p0 = self.to_local(p0_world)
        p1 = self.to_local(p1_world)
        d = p1 - p0
        half = self.half_extents

        breakpoints = [0.0, 1.0]
        for axis in range(3):
            if abs(d[axis]) < _EPS:
                continue
            for plane in (-half[axis], half[axis]):
                t = float((plane - p0[axis]) / d[axis])
                if 0.0 < t < 1.0:
                    breakpoints.append(t)
        breakpoints = sorted(set(breakpoints))

        def local_distance_squared(t: float) -> float:
            excess = np.maximum(np.abs(p0 + t * d) - half, 0.0)
            return float(excess @ excess)

        best = min(local_distance_squared(t) for t in breakpoints)
        for lo, hi in zip(breakpoints[:-1], breakpoints[1:]):
            mid = 0.5 * (lo + hi)
            point = p0 + mid * d
            active = np.abs(point) > half
            if not np.any(active):
                return 0.0
            bounds = np.where(point < -half, -half, half)
            active_d = d[active]
            offset = p0[active] - bounds[active]
            quadratic = float(active_d @ active_d)
            if quadratic <= _EPS:
                continue
            stationary = -float(active_d @ offset) / quadratic
            t = float(np.clip(stationary, lo, hi))
            best = min(best, local_distance_squared(t))
        return best

    def transformed(self, transform: np.ndarray, name: str | None = None, category: str | None = None) -> "OBB":
        transform = np.asarray(transform, dtype=float)
        return OBB(
            center=transform[:3, :3] @ self.center + transform[:3, 3],
            half_extents=self.half_extents,
            rotation=transform[:3, :3] @ self.rotation,
            name=self.name if name is None else name,
            category=self.category if category is None else category,
        )

    def intersects_obb(self, other: "OBB", margin: float = 0.0) -> bool:
        a_axes = self.rotation
        b_axes = other.rotation
        r = a_axes.T @ b_axes
        t = a_axes.T @ (other.center - self.center)
        abs_r = np.abs(r) + 1e-9
        a = np.maximum(self.half_extents + margin, 0.0)
        b = np.maximum(other.half_extents + margin, 0.0)

        for i in range(3):
            if abs(t[i]) > a[i] + float(abs_r[i, :] @ b):
                return False
        for j in range(3):
            if abs(float(t @ r[:, j])) > float(a @ abs_r[:, j]) + b[j]:
                return False
        for i in range(3):
            for j in range(3):
                ra = a[(i + 1) % 3] * abs_r[(i + 2) % 3, j] + a[(i + 2) % 3] * abs_r[(i + 1) % 3, j]
                rb = b[(j + 1) % 3] * abs_r[i, (j + 2) % 3] + b[(j + 2) % 3] * abs_r[i, (j + 1) % 3]
                lhs = abs(t[(i + 2) % 3] * r[(i + 1) % 3, j] - t[(i + 1) % 3] * r[(i + 2) % 3, j])
                if lhs > ra + rb:
                    return False
        return True


@dataclass(frozen=True)
class Capsule:
    p0: np.ndarray
    p1: np.ndarray
    radius: float
    name: str = "capsule"

    def __post_init__(self) -> None:
        object.__setattr__(self, "p0", np.asarray(self.p0, dtype=float))
        object.__setattr__(self, "p1", np.asarray(self.p1, dtype=float))
        if self.p0.shape != (3,) or self.p1.shape != (3,):
            raise ValueError("capsule endpoints must be shape (3,)")
        if self.radius <= 0:
            raise ValueError("capsule radius must be positive")

    def collides_obb(self, box: OBB, margin: float = 0.0) -> bool:
        r = self.radius + margin
        midpoint = 0.5 * (self.p0 + self.p1)
        half_segment_length = 0.5 * float(np.linalg.norm(self.p1 - self.p0))
        box_bounding_radius = float(np.linalg.norm(box.half_extents))
        if float(np.linalg.norm(midpoint - box.center)) > half_segment_length + box_bounding_radius + r:
            return False
        if box.segment_intersects(self.p0, self.p1, inflation=r):
            return True
        return box.segment_distance_squared(self.p0, self.p1) <= r * r


def segment_segment_distance_squared(
    p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray
) -> float:
    """Squared distance between two finite line segments.

    Implementation follows the standard closest-points derivation and handles
    degenerate segments.
    """
    p1 = np.asarray(p1, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(d1 @ d1)
    e = float(d2 @ d2)
    f = float(d2 @ r)

    if a <= _EPS and e <= _EPS:
        return float(r @ r)
    if a <= _EPS:
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0)
    else:
        c = float(d1 @ r)
        if e <= _EPS:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(d1 @ d2)
            denom = a * e - b * b
            if denom != 0.0:
                s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)
    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    delta = c1 - c2
    return float(delta @ delta)


def capsules_collide(a: Capsule, b: Capsule, margin: float = 0.0) -> bool:
    threshold = a.radius + b.radius + margin
    return segment_segment_distance_squared(a.p0, a.p1, b.p0, b.p1) <= threshold * threshold


def rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """SO(3) logarithm mapped to a 3-vector."""
    r = np.asarray(rotation, dtype=float)
    cos_theta = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    theta = float(np.arccos(cos_theta))
    if theta < 1e-8:
        return 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if abs(np.pi - theta) < 1e-5:
        # Robust axis extraction around pi.
        diag = np.maximum((np.diag(r) + 1.0) / 2.0, 0.0)
        axis = np.sqrt(diag)
        axis[0] = np.copysign(axis[0], r[2, 1] - r[1, 2])
        axis[1] = np.copysign(axis[1], r[0, 2] - r[2, 0])
        axis[2] = np.copysign(axis[2], r[1, 0] - r[0, 1])
        return normalize(axis) * theta
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    axis /= 2.0 * np.sin(theta)
    return axis * theta


def make_tool_rotation(tool_z_world: np.ndarray, up_hint_world: np.ndarray = np.array([0.0, 0.0, 1.0])) -> np.ndarray:
    """Construct a right-handed tool frame with the requested tool-Z axis."""
    z = normalize(tool_z_world)
    up = normalize(up_hint_world)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
    x = normalize(x)
    y = normalize(np.cross(z, x))
    return np.column_stack((x, y, z))
