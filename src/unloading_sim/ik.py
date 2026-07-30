"""Damped least-squares inverse kinematics with collision-aware restarts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from .geometry import rotation_vector_from_matrix
from .robot import RobotKinematics6
from .geometry import OBB


@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    iterations: int
    position_error: float
    orientation_error: float
    message: str = ""


def pose_error(current: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float, float]:
    p_error = target[:3, 3] - current[:3, 3]
    r_error = target[:3, :3] @ current[:3, :3].T
    w_error = rotation_vector_from_matrix(r_error)
    return np.concatenate([p_error, w_error]), float(np.linalg.norm(p_error)), float(np.linalg.norm(w_error))


def solve_ik(
    robot: RobotKinematics6,
    target: np.ndarray,
    seed: np.ndarray,
    obstacles: Sequence[OBB] | None = None,
    ignored_obstacle_names: set[str] | None = None,
    max_iterations: int = 250,
    damping: float = 0.05,
    max_step: float = 0.18,
    position_tolerance: float = 0.008,
    orientation_tolerance: float = 0.06,
    position_weight: float = 1.0,
    orientation_weight: float = 0.45,
    collision_check_stride: int = 8,
    extra_state_valid: Callable[[np.ndarray], bool] | None = None,
) -> IKResult:
    q = robot.clamp(np.asarray(seed, dtype=float).copy())
    obstacles = list(obstacles or [])
    last_pos = float("inf")
    last_ori = float("inf")

    weights = np.diag([position_weight] * 3 + [orientation_weight] * 3)
    for iteration in range(1, max_iterations + 1):
        current = robot.fk(q)
        err, pos_err, ori_err = pose_error(current, target)
        last_pos, last_ori = pos_err, ori_err
        if pos_err <= position_tolerance and ori_err <= orientation_tolerance:
            collision_free = not obstacles or robot.is_collision_free(q, obstacles, ignored_obstacle_names=ignored_obstacle_names)
            if collision_free and (extra_state_valid is None or extra_state_valid(q)):
                return IKResult(True, q, iteration, pos_err, ori_err, "converged")

        j = robot.geometric_jacobian(q)
        jw = weights @ j
        ew = weights @ err
        # DLS: dq = J^T (J J^T + lambda^2 I)^-1 e
        lhs = jw @ jw.T + (damping * damping) * np.eye(6)
        try:
            dq = jw.T @ np.linalg.solve(lhs, ew)
        except np.linalg.LinAlgError:
            dq = jw.T @ np.linalg.pinv(lhs) @ ew

        norm = float(np.linalg.norm(dq))
        if norm > max_step:
            dq *= max_step / norm

        # Weak joint-centering term to avoid drifting toward large wraps.
        center = np.mean(robot.joint_limits, axis=1)
        span = np.maximum(robot.joint_limits[:, 1] - robot.joint_limits[:, 0], 1e-6)
        dq += 0.015 * (center - q) / span
        candidate = robot.clamp(q + dq)

        # During IK, reject gross collision excursions periodically.  This is
        # not a full constrained optimizer, but greatly improves restart quality.
        if iteration % collision_check_stride == 0:
            collision_free = not obstacles or robot.is_collision_free(candidate, obstacles, ignored_obstacle_names=ignored_obstacle_names)
            posture_valid = extra_state_valid is None or extra_state_valid(candidate)
            if not collision_free or not posture_valid:
                candidate = robot.clamp(q + 0.25 * dq)
        q = candidate

    return IKResult(False, q, max_iterations, last_pos, last_ori, "maximum iterations reached")


def solve_ik_multistart(
    robot: RobotKinematics6,
    target: np.ndarray,
    seeds: Sequence[np.ndarray],
    obstacles: Sequence[OBB] | None = None,
    ignored_obstacle_names: set[str] | None = None,
    random_restarts: int = 10,
    rng: np.random.Generator | None = None,
    **kwargs,
) -> IKResult:
    rng = rng or np.random.default_rng(0)
    candidates = [np.asarray(s, dtype=float) for s in seeds]
    for _ in range(random_restarts):
        candidates.append(rng.uniform(robot.joint_limits[:, 0], robot.joint_limits[:, 1]))

    best: IKResult | None = None
    for seed in candidates:
        result = solve_ik(
            robot,
            target,
            seed,
            obstacles=obstacles,
            ignored_obstacle_names=ignored_obstacle_names,
            **kwargs,
        )
        if result.success:
            return result
        cost = result.position_error + 0.25 * result.orientation_error
        if best is None or cost < best.position_error + 0.25 * best.orientation_error:
            best = result
    assert best is not None
    best.message = "all IK restarts failed; returning closest result"
    return best
