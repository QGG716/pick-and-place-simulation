"""Damped least-squares inverse kinematics with collision-aware restarts."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Sequence

import numpy as np

from .geometry import rotation_vector_from_matrix
from .robot import RobotBackend
from .geometry import OBB


@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    iterations: int
    position_error: float
    orientation_error: float
    message: str = ""
    search_evidence: dict = field(default_factory=dict)


def pose_error(current: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float, float]:
    p_error = target[:3, 3] - current[:3, 3]
    r_error = target[:3, :3] @ current[:3, :3].T
    w_error = rotation_vector_from_matrix(r_error)
    return np.concatenate([p_error, w_error]), float(np.linalg.norm(p_error)), float(np.linalg.norm(w_error))


def solve_ik(
    robot: RobotBackend,
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
    collision_margin: float = 0.015,
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
            collision_free = not obstacles or robot.is_collision_free(
                q,
                obstacles,
                margin=collision_margin,
                ignored_obstacle_names=ignored_obstacle_names,
            )
            if collision_free and (extra_state_valid is None or extra_state_valid(q)):
                return IKResult(True, q, iteration, pos_err, ori_err, "converged")
            # This optimizer has no obstacle gradient. Once the primary
            # residual is zero, repeating identical updates cannot leave a
            # colliding branch; let multistart try its next seeded branch.
            return IKResult(False, q, iteration, pos_err, ori_err, "converged pose violates collision or task constraint")

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

        # Center only in the true numerical null space. A damped projector
        # leaks into the primary task and leaves a nonzero FK residual on a
        # nonredundant six-axis robot.
        center = np.mean(robot.joint_limits, axis=1)
        span = np.maximum(robot.joint_limits[:, 1] - robot.joint_limits[:, 0], 1e-6)
        if len(q) > np.linalg.matrix_rank(jw):
            null = np.eye(len(q)) - np.linalg.pinv(jw, rcond=1e-10) @ jw
            dq += null @ (0.015 * (center - q) / span)
        norm = float(np.linalg.norm(dq))
        if norm > max_step:
            dq *= max_step / norm
        candidate = robot.clamp(q + dq)

        # During IK, reject gross collision excursions periodically.  This is
        # not a full constrained optimizer, but greatly improves restart quality.
        if iteration % collision_check_stride == 0:
            collision_free = not obstacles or robot.is_collision_free(
                candidate,
                obstacles,
                margin=collision_margin,
                ignored_obstacle_names=ignored_obstacle_names,
            )
            posture_valid = extra_state_valid is None or extra_state_valid(candidate)
            if not collision_free or not posture_valid:
                candidate = robot.clamp(q + 0.25 * dq)
        q = candidate

    # The loop checks convergence before applying each update. A solution that
    # enters tolerance on the final permitted update must still receive the
    # same collision and task-state validation; otherwise it is incorrectly
    # reported as a failure and its stored errors refer to the previous q.
    _, last_pos, last_ori = pose_error(robot.fk(q), target)
    if last_pos <= position_tolerance and last_ori <= orientation_tolerance:
        collision_free = not obstacles or robot.is_collision_free(
            q,
            obstacles,
            margin=collision_margin,
            ignored_obstacle_names=ignored_obstacle_names,
        )
        if collision_free and (extra_state_valid is None or extra_state_valid(q)):
            return IKResult(
                True,
                q,
                max_iterations,
                last_pos,
                last_ori,
                "converged on final update",
            )
    return IKResult(False, q, max_iterations, last_pos, last_ori, "maximum iterations reached")


def solve_ik_multistart(
    robot: RobotBackend,
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
    attempted = 0
    iterations_consumed = 0
    converged_pose_results = 0
    max_iterations = int(kwargs.get("max_iterations", 250))
    position_tolerance = float(kwargs.get("position_tolerance", 0.008))
    orientation_tolerance = float(kwargs.get("orientation_tolerance", 0.06))
    for seed_index, seed in enumerate(candidates):
        result = solve_ik(
            robot,
            target,
            seed,
            obstacles=obstacles,
            ignored_obstacle_names=ignored_obstacle_names,
            **kwargs,
        )
        attempted += 1
        iterations_consumed += result.iterations
        converged_pose_results += int(
            result.position_error <= position_tolerance
            and result.orientation_error <= orientation_tolerance
        )
        if result.success:
            result.search_evidence = {
                "policy": "legacy_first_valid_result",
                "seed_pool_available": len(candidates),
                "explicit_seed_count": len(candidates) - random_restarts,
                "random_restart_count": random_restarts,
                "seeds_attempted": attempted,
                "last_seed_index": seed_index,
                "iterations_per_seed_available": max_iterations,
                "iteration_capacity_available": len(candidates) * max_iterations,
                "iteration_capacity_for_attempted_seeds": attempted * max_iterations,
                "iterations_consumed": iterations_consumed,
                "converged_pose_results": converged_pose_results,
                "valid_solutions": 1,
                "deduplicated_candidates": 1,
                "duplicate_candidates": 0,
                "termination": "FIRST_VALID_SOLUTION",
            }
            return result
        cost = result.position_error + 0.25 * result.orientation_error
        if best is None or cost < best.position_error + 0.25 * best.orientation_error:
            best = result
    assert best is not None
    best.message = "all IK restarts failed; returning closest result"
    best.search_evidence = {
        "policy": "legacy_first_valid_result",
        "seed_pool_available": len(candidates),
        "explicit_seed_count": len(candidates) - random_restarts,
        "random_restart_count": random_restarts,
        "seeds_attempted": attempted,
        "last_seed_index": attempted - 1,
        "iterations_per_seed_available": max_iterations,
        "iteration_capacity_available": len(candidates) * max_iterations,
        "iteration_capacity_for_attempted_seeds": attempted * max_iterations,
        "iterations_consumed": iterations_consumed,
        "converged_pose_results": converged_pose_results,
        "valid_solutions": 0,
        "deduplicated_candidates": 0,
        "duplicate_candidates": 0,
        "termination": "SEED_STREAM_EXHAUSTED",
    }
    return best
