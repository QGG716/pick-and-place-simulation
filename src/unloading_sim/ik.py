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


class IKCandidateStream:
    """Lazily continue one deterministic seed stream and yield distinct solutions."""

    def __init__(
        self,
        robot: RobotBackend,
        target: np.ndarray,
        seeds: Sequence[np.ndarray],
        *,
        obstacles: Sequence[OBB] | None = None,
        ignored_obstacle_names: set[str] | None = None,
        random_restarts: int = 10,
        rng: np.random.Generator | None = None,
        candidate_limit: int | None = None,
        dedup_tolerance_rad: float = 1e-3,
        dedup_tolerance_m: float = 1e-4,
        **kwargs,
    ) -> None:
        self.robot = robot
        self.target = target
        self.obstacles = obstacles
        self.ignored_obstacle_names = ignored_obstacle_names
        self.kwargs = kwargs
        self.explicit_seed_count = len(seeds)
        self.random_restart_count = int(random_restarts)
        generator = rng or np.random.default_rng(0)
        self.seeds = [np.asarray(seed, dtype=float) for seed in seeds]
        self.seeds.extend(
            generator.uniform(robot.joint_limits[:, 0], robot.joint_limits[:, 1])
            for _ in range(self.random_restart_count)
        )
        self.candidate_limit = candidate_limit
        self.dedup_tolerance_rad = float(dedup_tolerance_rad)
        self.dedup_tolerance_m = float(dedup_tolerance_m)
        self.seed_index = 0
        self.iterations_consumed = 0
        self.converged_pose_results = 0
        self.valid_solutions = 0
        self.duplicate_candidates = 0
        self.solutions: list[np.ndarray] = []
        self.best_failure: IKResult | None = None
        self.termination = "NOT_STARTED"

    def __iter__(self):
        return self

    def __next__(self) -> IKResult:
        if self.candidate_limit is not None and len(self.solutions) >= self.candidate_limit:
            self.termination = "IK_CANDIDATE_LIMIT_REACHED"
            raise StopIteration
        position_tolerance = float(self.kwargs.get("position_tolerance", 0.008))
        orientation_tolerance = float(self.kwargs.get("orientation_tolerance", 0.06))
        while self.seed_index < len(self.seeds):
            seed = self.seeds[self.seed_index]
            self.seed_index += 1
            result = solve_ik(
                self.robot,
                self.target,
                seed,
                obstacles=self.obstacles,
                ignored_obstacle_names=self.ignored_obstacle_names,
                **self.kwargs,
            )
            self.iterations_consumed += result.iterations
            converged = (
                result.position_error <= position_tolerance
                and result.orientation_error <= orientation_tolerance
            )
            self.converged_pose_results += int(converged)
            if not result.success:
                cost = result.position_error + 0.25 * result.orientation_error
                if self.best_failure is None or cost < (
                    self.best_failure.position_error + 0.25 * self.best_failure.orientation_error
                ):
                    self.best_failure = result
                continue
            self.valid_solutions += 1
            if any(joint_solutions_equivalent(
                self.robot, result.q, prior,
                revolute_tolerance_rad=self.dedup_tolerance_rad,
                prismatic_tolerance_m=self.dedup_tolerance_m,
            ) for prior in self.solutions):
                self.duplicate_candidates += 1
                continue
            self.solutions.append(result.q.copy())
            self.termination = "VALID_SOLUTION_YIELDED"
            result.search_evidence = self.evidence()
            result.search_evidence["candidate_id"] = (
                f"seed_{self.seed_index - 1:03d}_unique_{len(self.solutions) - 1:02d}"
            )
            return result
        self.termination = "SEED_STREAM_EXHAUSTED"
        raise StopIteration

    def evidence(self) -> dict:
        max_iterations = int(self.kwargs.get("max_iterations", 250))
        return {
            "policy": "lazy_distinct_valid_solutions",
            "seed_pool_available": len(self.seeds),
            "explicit_seed_count": self.explicit_seed_count,
            "random_restart_count": self.random_restart_count,
            "seeds_attempted": self.seed_index,
            "last_seed_index": self.seed_index - 1,
            "iterations_per_seed_available": max_iterations,
            "iteration_capacity_available": len(self.seeds) * max_iterations,
            "iteration_capacity_for_attempted_seeds": self.seed_index * max_iterations,
            "iterations_consumed": self.iterations_consumed,
            "converged_pose_results": self.converged_pose_results,
            "valid_solutions": self.valid_solutions,
            "deduplicated_candidates": len(self.solutions),
            "duplicate_candidates": self.duplicate_candidates,
            "candidate_limit": self.candidate_limit,
            "dedup_tolerance_rad": self.dedup_tolerance_rad,
            "dedup_tolerance_m": self.dedup_tolerance_m,
            "seed_stream_exhausted": self.seed_index >= len(self.seeds),
            "termination": self.termination,
        }


def iter_ik_solutions(
    robot: RobotBackend,
    target: np.ndarray,
    seeds: Sequence[np.ndarray],
    **kwargs,
) -> IKCandidateStream:
    return IKCandidateStream(robot, target, seeds, **kwargs)


def joint_solutions_equivalent(
    robot: RobotBackend,
    first: np.ndarray,
    second: np.ndarray,
    *,
    revolute_tolerance_rad: float,
    prismatic_tolerance_m: float,
) -> bool:
    """Compare joints without folding bounded revolute or mixing SI units."""
    a = np.asarray(first, dtype=float)
    b = np.asarray(second, dtype=float)
    if a.shape != b.shape:
        return False
    joints = getattr(robot, "active_joints", None)
    joint_types = [joint.joint_type for joint in joints] if joints is not None else \
                  ["revolute"] * len(a)
    if len(joint_types) != len(a):
        raise ValueError("robot joint type metadata does not match candidate dimension")
    for delta, joint_type in zip(np.abs(a - b), joint_types):
        if joint_type == "continuous":
            delta = abs((float(delta) + np.pi) % (2 * np.pi) - np.pi)
        tolerance = prismatic_tolerance_m if joint_type == "prismatic" else revolute_tolerance_rad
        if delta > tolerance:
            return False
    return True


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
