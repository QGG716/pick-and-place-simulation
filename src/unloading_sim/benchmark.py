"""Separate planning-compute latency from simulated or measured cycle time."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def _statistics(values: list[float]) -> dict:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        return {"count": 0, "total_seconds": 0.0, "mean_seconds": None, "p95_seconds": None, "max_seconds": None}
    if np.any(array < 0.0) or not np.all(np.isfinite(array)):
        raise ValueError("benchmark durations must be finite and non-negative")
    return {
        "count": int(array.size),
        "total_seconds": float(np.sum(array)),
        "mean_seconds": float(np.mean(array)),
        "p95_seconds": float(np.percentile(array, 95.0)),
        "max_seconds": float(np.max(array)),
    }


@dataclass
class BenchmarkRecorder:
    execution_source: str = "simulation"
    planning_seconds: list[float] = field(default_factory=list)
    cycle_seconds: list[float] = field(default_factory=list)

    def record_planning(self, elapsed_seconds: float) -> None:
        self.planning_seconds.append(float(elapsed_seconds))

    def record_cycle(self, elapsed_seconds: float) -> None:
        self.cycle_seconds.append(float(elapsed_seconds))

    def report(self, *, planning_deadline_seconds: float | None = None) -> dict:
        planning = _statistics(self.planning_seconds)
        cycle = _statistics(self.cycle_seconds)
        result = {
            "model": "separated_planning_and_cycle_benchmark_v1",
            "execution_source": self.execution_source,
            "planning_compute": planning,
            "execution_cycle": cycle,
            "theoretical_cases_per_hour": (
                None if cycle["mean_seconds"] in (None, 0.0) else 3600.0 / cycle["mean_seconds"]
            ),
        }
        if planning_deadline_seconds is not None:
            if planning_deadline_seconds <= 0.0:
                raise ValueError("planning deadline must be positive")
            result["planning_deadline_seconds"] = float(planning_deadline_seconds)
            result["planning_within_deadline"] = bool(
                self.planning_seconds and max(self.planning_seconds) <= planning_deadline_seconds
            )
        return result
