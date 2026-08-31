"""Deterministic continuous base-pose optimization for trailer coverage."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np


@dataclass(frozen=True)
class BasePose:
    x_m: float
    y_m: float
    yaw_rad: float

    def as_array(self) -> np.ndarray:
        return np.array([self.x_m, self.y_m, self.yaw_rad], dtype=float)


@dataclass(frozen=True)
class BaseOptimizationResult:
    pose: BasePose
    score: float
    evaluations: int
    seed: int
    initial_samples: int
    refinement_iterations: int

    def audit(self) -> dict:
        return {
            "method": "seeded_continuous_sample_and_pattern_search_v1",
            "pose": {
                "x_m": self.pose.x_m,
                "y_m": self.pose.y_m,
                "yaw_rad": self.pose.yaw_rad,
            },
            "score": self.score,
            "evaluations": self.evaluations,
            "seed": self.seed,
            "initial_samples": self.initial_samples,
            "refinement_iterations": self.refinement_iterations,
        }


def _validate_bounds(bounds: Sequence[Sequence[float]]) -> np.ndarray:
    limits = np.asarray(bounds, dtype=float)
    if limits.shape != (3, 2) or not np.all(np.isfinite(limits)):
        raise ValueError("bounds must be finite shape (3, 2) for x, y, yaw")
    if np.any(limits[:, 1] <= limits[:, 0]):
        raise ValueError("every base-pose upper bound must exceed its lower bound")
    return limits


def optimize_base_pose_continuous(
    objective: Callable[[BasePose], float],
    bounds: Sequence[Sequence[float]],
    *,
    seed: int,
    initial_samples: int = 128,
    refinement_iterations: int = 20,
    initial_pose: BasePose | None = None,
) -> BaseOptimizationResult:
    """Maximize a finite scalar objective over continuous x/y/yaw bounds.

    Seeded uniform exploration is followed by deterministic coordinate pattern
    search.  The objective can run batch IK and exact collision checks; keeping
    it as a callback makes this optimizer independent of a heavyweight backend.
    """
    limits = _validate_bounds(bounds)
    if initial_samples < 1 or refinement_iterations < 0:
        raise ValueError("initial_samples must be positive and refinement_iterations non-negative")
    rng = np.random.default_rng(seed)
    samples = rng.uniform(limits[:, 0], limits[:, 1], size=(initial_samples, 3))
    midpoint = np.mean(limits, axis=1)
    samples = np.vstack((midpoint, samples))
    if initial_pose is not None:
        samples = np.vstack((initial_pose.as_array(), samples))
    samples = np.clip(samples, limits[:, 0], limits[:, 1])

    evaluations = 0

    def evaluate(values: np.ndarray) -> float:
        nonlocal evaluations
        score = float(objective(BasePose(*values)))
        evaluations += 1
        if not np.isfinite(score):
            raise ValueError("base-pose objective returned a non-finite score")
        return score

    scores = np.array([evaluate(sample) for sample in samples])
    best_index = int(np.argmax(scores))
    best = samples[best_index].copy()
    best_score = float(scores[best_index])
    step = 0.2 * (limits[:, 1] - limits[:, 0])
    for _ in range(refinement_iterations):
        improved = False
        for axis in range(3):
            for direction in (-1.0, 1.0):
                candidate = best.copy()
                candidate[axis] = np.clip(
                    candidate[axis] + direction * step[axis], limits[axis, 0], limits[axis, 1]
                )
                score = evaluate(candidate)
                if score > best_score:
                    best, best_score, improved = candidate, score, True
        step *= 0.72 if improved else 0.5
    return BaseOptimizationResult(
        BasePose(*best), best_score, evaluations, int(seed), initial_samples, refinement_iterations
    )


def geometric_trailer_coverage_objective(
    target_points_world_m: Sequence[Sequence[float]],
    *,
    minimum_reach_m: float,
    maximum_reach_m: float,
    minimum_height_m: float,
    maximum_height_m: float,
    preferred_reach_m: float | None = None,
) -> Callable[[BasePose], float]:
    """Create a fast pre-IK coverage score for continuous base optimization."""
    targets = np.asarray(target_points_world_m, dtype=float)
    if targets.ndim != 2 or targets.shape[1] != 3 or len(targets) == 0:
        raise ValueError("target points must have shape (N, 3)")
    if not 0.0 <= minimum_reach_m < maximum_reach_m:
        raise ValueError("reach interval must be positive and ordered")
    if minimum_height_m >= maximum_height_m:
        raise ValueError("height interval must be ordered")
    preferred = 0.5 * (minimum_reach_m + maximum_reach_m) if preferred_reach_m is None else float(preferred_reach_m)

    def objective(pose: BasePose) -> float:
        delta = targets[:, :2] - np.array([pose.x_m, pose.y_m])
        cosine, sine = np.cos(pose.yaw_rad), np.sin(pose.yaw_rad)
        base_delta = delta @ np.array([[cosine, -sine], [sine, cosine]])
        radial = np.linalg.norm(base_delta, axis=1)
        reachable = (
            (radial >= minimum_reach_m)
            & (radial <= maximum_reach_m)
            & (targets[:, 2] >= minimum_height_m)
            & (targets[:, 2] <= maximum_height_m)
            & (base_delta[:, 0] >= 0.0)
        )
        coverage = float(np.mean(reachable))
        if not np.any(reachable):
            return coverage
        reach_quality = 1.0 - np.mean(
            np.minimum(1.0, np.abs(radial[reachable] - preferred) / maximum_reach_m)
        )
        return coverage + 0.01 * float(reach_quality)

    return objective
