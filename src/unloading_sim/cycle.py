"""Deterministic Monte Carlo cycle-time model for trailer unloading."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


@dataclass(frozen=True)
class CycleEvent:
    name: str
    probability: float
    duration_seconds: float | tuple[float, float, float]
    conditional_on: str = "ALWAYS"

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("event name cannot be empty")
        if not np.isfinite(self.probability) or not 0.0 <= self.probability <= 1.0:
            raise ValueError(f"event {self.name} probability must be in [0, 1]")
        values = (self.duration_seconds,) if isinstance(self.duration_seconds, float) else self.duration_seconds
        if not all(np.isfinite(value) for value in values):
            raise ValueError(f"event {self.name} durations must be finite")
        if len(values) == 3 and not values[0] <= values[1] <= values[2]:
            raise ValueError(f"event {self.name} triangular duration must be low <= mode <= high")


@dataclass(frozen=True)
class CycleModel:
    seed: int
    sample_count: int
    shift_hours: float
    target_boxes_per_hour: float
    normal_cycle_seconds: float
    normal_cycle_candidates_seconds: tuple[float, ...]
    base_events: tuple[CycleEvent, ...]
    loss_events: tuple[CycleEvent, ...]
    source: Mapping[str, Any]

    def __post_init__(self) -> None:
        if self.sample_count < 1:
            raise ValueError("sample_count must be positive")
        if self.shift_hours <= 0.0 or self.target_boxes_per_hour <= 0.0:
            raise ValueError("shift and target throughput must be positive")
        if self.normal_cycle_seconds <= 0.0 or any(v <= 0.0 for v in self.normal_cycle_candidates_seconds):
            raise ValueError("normal cycle values must be positive")
        probability_sum = sum(event.probability for event in self.base_events)
        if not np.isclose(probability_sum, 1.0, atol=1e-12):
            raise ValueError("base event probabilities must sum to one")
        known = {event.name for event in self.base_events}
        for event in self.loss_events:
            if event.conditional_on != "ALWAYS" and event.conditional_on not in known:
                raise ValueError(f"event {event.name} depends on unknown or later event {event.conditional_on}")
            known.add(event.name)


@dataclass(frozen=True)
class CycleRandomDraws:
    base_indices: np.ndarray
    loss_occurrences: Mapping[str, np.ndarray]
    loss_durations_seconds: Mapping[str, np.ndarray]


def _event(mapping: Mapping[str, Any], *, base: bool) -> CycleEvent:
    raw_duration = mapping["duration_seconds"]
    if isinstance(raw_duration, Sequence) and not isinstance(raw_duration, (str, bytes)):
        duration: float | tuple[float, float, float] = tuple(float(value) for value in raw_duration)  # type: ignore[assignment]
        if len(duration) != 3:
            raise ValueError("duration sequence must contain low, mode and high")
    else:
        duration = float(raw_duration)
    return CycleEvent(
        name=str(mapping["name"]),
        probability=float(mapping["probability"]),
        duration_seconds=duration,
        conditional_on="ALWAYS" if base else str(mapping.get("conditional_on", "ALWAYS")),
    )


def load_cycle_model(path: str | Path) -> tuple[CycleModel, dict[str, Any], Path]:
    resolved = Path(path).resolve()
    data = yaml.safe_load(resolved.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != "unloading_cycle_monte_carlo_v1":
        raise ValueError("unsupported cycle model schema")
    model = CycleModel(
        seed=int(data["seed"]),
        sample_count=int(data["sample_count"]),
        shift_hours=float(data["shift_hours"]),
        target_boxes_per_hour=float(data["target_boxes_per_hour"]),
        normal_cycle_seconds=float(data["normal_cycle_seconds"]),
        normal_cycle_candidates_seconds=tuple(float(v) for v in data["normal_cycle_candidates_seconds"]),
        base_events=tuple(_event(item, base=True) for item in data["base_events"]),
        loss_events=tuple(_event(item, base=False) for item in data["loss_events"]),
        source=dict(data.get("source", {})),
    )
    return model, data, resolved


def draw_cycle_randomness(model: CycleModel) -> CycleRandomDraws:
    """Draw once so cycle candidates use common random numbers."""
    rng = np.random.default_rng(model.seed)
    probabilities = np.asarray([event.probability for event in model.base_events])
    base_indices = rng.choice(len(model.base_events), size=model.sample_count, p=probabilities)
    occurred: dict[str, np.ndarray] = {
        event.name: base_indices == index for index, event in enumerate(model.base_events)
    }
    durations: dict[str, np.ndarray] = {}
    loss_occurrences: dict[str, np.ndarray] = {}
    for event in model.loss_events:
        eligible = np.ones(model.sample_count, dtype=bool) if event.conditional_on == "ALWAYS" else occurred[event.conditional_on]
        selected = eligible & (rng.random(model.sample_count) < event.probability)
        loss_occurrences[event.name] = selected
        occurred[event.name] = selected
        if isinstance(event.duration_seconds, tuple):
            low, mode, high = event.duration_seconds
            values = rng.triangular(low, mode, high, size=model.sample_count)
        else:
            values = np.full(model.sample_count, event.duration_seconds, dtype=float)
        durations[event.name] = values
    return CycleRandomDraws(base_indices, loss_occurrences, durations)


def simulate_cycles(model: CycleModel, normal_cycle_seconds: float, draws: CycleRandomDraws | None = None) -> tuple[np.ndarray, dict[str, Any]]:
    if normal_cycle_seconds <= 0.0 or not np.isfinite(normal_cycle_seconds):
        raise ValueError("normal_cycle_seconds must be finite and positive")
    draws = draw_cycle_randomness(model) if draws is None else draws
    cycles = np.full(model.sample_count, float(normal_cycle_seconds), dtype=float)
    breakdown: dict[str, dict[str, float | int]] = {}
    for index, event in enumerate(model.base_events):
        selected = draws.base_indices == index
        count = int(np.count_nonzero(selected))
        offset = float(event.duration_seconds)
        cycles[selected] += offset
        breakdown[event.name] = {
            "count": count,
            "probability_observed": count / model.sample_count,
            "total_seconds": count * offset,
            "mean_seconds_per_box": count * offset / model.sample_count,
        }
    for event in model.loss_events:
        selected = draws.loss_occurrences[event.name]
        values = draws.loss_durations_seconds[event.name]
        cycles[selected] += values[selected]
        total = float(np.sum(values[selected]))
        count = int(np.count_nonzero(selected))
        breakdown[event.name] = {
            "count": count,
            "probability_observed": count / model.sample_count,
            "total_seconds": total,
            "mean_seconds_per_box": total / model.sample_count,
        }
    if np.any(cycles <= 0.0):
        raise ValueError("cycle configuration produced a non-positive cycle")
    mean = float(np.mean(cycles))
    summary = {
        "normal_cycle_seconds": float(normal_cycle_seconds),
        "sample_count": model.sample_count,
        "seed": model.seed,
        "mean_cycle_time_seconds": mean,
        "p50_seconds": float(np.percentile(cycles, 50)),
        "p90_seconds": float(np.percentile(cycles, 90)),
        "p95_seconds": float(np.percentile(cycles, 95)),
        "p99_seconds": float(np.percentile(cycles, 99)),
        "boxes_per_hour": 3600.0 / mean,
        "boxes_per_shift": model.shift_hours * 3600.0 / mean,
        "target_boxes_per_hour": model.target_boxes_per_hour,
        "target_met": 3600.0 / mean >= model.target_boxes_per_hour,
        "event_loss_breakdown": breakdown,
        "source": dict(model.source),
    }
    return cycles, summary


def required_normal_cycle_seconds(model: CycleModel, draws: CycleRandomDraws | None = None) -> float:
    """Return the normal-cycle budget that yields the target mean throughput."""
    draws = draw_cycle_randomness(model) if draws is None else draws
    cycles, _ = simulate_cycles(model, 1.0, draws)
    average_offset = float(np.mean(cycles) - 1.0)
    return 3600.0 / model.target_boxes_per_hour - average_offset
