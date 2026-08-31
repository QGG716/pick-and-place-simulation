"""Seeded Monte Carlo robustness scenarios for simulated unloading."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np


@dataclass(frozen=True)
class RobustnessNoise:
    perception_position_std_m: np.ndarray
    carton_size_std_m: np.ndarray
    base_position_std_m: np.ndarray
    base_yaw_std_rad: float
    suction_leak_probability: float

    def __post_init__(self) -> None:
        perception = np.asarray(self.perception_position_std_m, dtype=float)
        carton_size = np.asarray(self.carton_size_std_m, dtype=float)
        base_position = np.asarray(self.base_position_std_m, dtype=float)
        if perception.shape != (3,) or carton_size.shape != (3,) or base_position.shape != (2,):
            raise ValueError("noise standard deviations must have shapes (3,), (3,), and (2,)")
        if any(np.any(values < 0.0) or not np.all(np.isfinite(values)) for values in (perception, carton_size, base_position)):
            raise ValueError("noise standard deviations must be finite and non-negative")
        if not np.isfinite(self.base_yaw_std_rad) or self.base_yaw_std_rad < 0.0:
            raise ValueError("base_yaw_std_rad must be finite and non-negative")
        if not 0.0 <= self.suction_leak_probability <= 1.0:
            raise ValueError("suction_leak_probability must be in [0, 1]")
        object.__setattr__(self, "perception_position_std_m", perception.copy())
        object.__setattr__(self, "carton_size_std_m", carton_size.copy())
        object.__setattr__(self, "base_position_std_m", base_position.copy())


@dataclass(frozen=True)
class RobustnessTrial:
    index: int
    perception_offset_m: np.ndarray
    carton_size_offset_m: np.ndarray
    base_position_offset_m: np.ndarray
    base_yaw_offset_rad: float
    suction_leak: bool


@dataclass(frozen=True)
class TrialOutcome:
    success: bool
    reason: str = "success"
    minimum_clearance_m: float | None = None


def _wilson_interval(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    proportion = successes / trials
    denominator = 1.0 + z * z / trials
    center = (proportion + z * z / (2.0 * trials)) / denominator
    radius = z * np.sqrt(proportion * (1.0 - proportion) / trials + z * z / (4.0 * trials * trials)) / denominator
    return max(0.0, float(center - radius)), min(1.0, float(center + radius))


def run_monte_carlo_robustness(
    evaluator: Callable[[RobustnessTrial], TrialOutcome | bool],
    noise: RobustnessNoise,
    *,
    trials: int,
    seed: int,
) -> dict:
    """Run reproducible perturbation trials through a simulator callback."""
    if trials <= 0:
        raise ValueError("trials must be positive")
    rng = np.random.default_rng(seed)
    outcomes: list[TrialOutcome] = []
    for index in range(trials):
        trial = RobustnessTrial(
            index=index,
            perception_offset_m=rng.normal(0.0, noise.perception_position_std_m),
            carton_size_offset_m=rng.normal(0.0, noise.carton_size_std_m),
            base_position_offset_m=rng.normal(0.0, noise.base_position_std_m),
            base_yaw_offset_rad=float(rng.normal(0.0, noise.base_yaw_std_rad)),
            suction_leak=bool(rng.random() < noise.suction_leak_probability),
        )
        raw = evaluator(trial)
        outcome = raw if isinstance(raw, TrialOutcome) else TrialOutcome(bool(raw), "success" if raw else "unspecified")
        outcomes.append(outcome)
    successes = sum(outcome.success for outcome in outcomes)
    low, high = _wilson_interval(successes, trials)
    clearances = [
        float(outcome.minimum_clearance_m)
        for outcome in outcomes
        if outcome.minimum_clearance_m is not None
    ]
    failures = Counter(outcome.reason for outcome in outcomes if not outcome.success)
    return {
        "model": "seeded_simulation_monte_carlo_v1",
        "seed": int(seed),
        "trials": int(trials),
        "successes": int(successes),
        "success_probability": successes / trials,
        "success_probability_95pct_wilson": [low, high],
        "failure_reasons": dict(sorted(failures.items())),
        "minimum_clearance_m": min(clearances) if clearances else None,
        "noise": {
            "perception_position_std_m": noise.perception_position_std_m.tolist(),
            "carton_size_std_m": noise.carton_size_std_m.tolist(),
            "base_position_std_m": noise.base_position_std_m.tolist(),
            "base_yaw_std_rad": noise.base_yaw_std_rad,
            "suction_leak_probability": noise.suction_leak_probability,
        },
    }
