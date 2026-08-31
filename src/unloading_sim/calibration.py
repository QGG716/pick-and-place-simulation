"""Measured controller-log calibration for execution-model assumptions."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ControllerLog:
    timestamps: np.ndarray
    positions: np.ndarray
    source: str

    def __post_init__(self) -> None:
        timestamps = np.asarray(self.timestamps, dtype=float)
        positions = np.asarray(self.positions, dtype=float)
        if timestamps.ndim != 1 or positions.ndim != 2 or len(timestamps) != len(positions):
            raise ValueError("timestamps and positions must have shapes (N,) and (N, dof)")
        if len(timestamps) < 4:
            raise ValueError("controller log must contain at least four samples")
        if not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(positions)):
            raise ValueError("controller log contains non-finite values")
        if np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("controller timestamps must be strictly increasing")
        object.__setattr__(self, "timestamps", timestamps.copy())
        object.__setattr__(self, "positions", positions.copy())


def load_controller_csv(
    path: str | Path,
    *,
    time_column: str = "time_from_start_s",
    joint_columns: Sequence[str] | None = None,
) -> ControllerLog:
    path = Path(path)
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if time_column not in fields:
            raise ValueError(f"controller log is missing time column {time_column!r}")
        if joint_columns is None:
            joint_columns = sorted(
                (name for name in fields if name.startswith("q") and name[1:].isdigit()),
                key=lambda name: int(name[1:]),
            )
        if not joint_columns or any(name not in fields for name in joint_columns):
            raise ValueError("controller log must contain explicit joint columns")
        rows = list(reader)
    timestamps = np.asarray([float(row[time_column]) for row in rows], dtype=float)
    positions = np.asarray([[float(row[name]) for name in joint_columns] for row in rows], dtype=float)
    return ControllerLog(timestamps, positions, str(path))


def _gradient(values: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
    return np.gradient(values, timestamps, axis=0, edge_order=1)


def calibrate_motion_limits(
    log: ControllerLog,
    *,
    percentile: float = 99.5,
    headroom: float = 1.15,
    minimum_limit: float = 1e-6,
) -> dict:
    """Estimate auditable limits from measured joint-position logs.

    Percentiles reduce sensitivity to encoder spikes; ``headroom`` keeps the
    result above ordinary observed motion.  These values are proposed limits,
    not manufacturer safety ratings, and retain their provenance in output.
    """
    if not 50.0 <= percentile <= 100.0:
        raise ValueError("percentile must be in [50, 100]")
    if not np.isfinite(headroom) or headroom < 1.0:
        raise ValueError("headroom must be finite and at least 1")
    velocity = _gradient(log.positions, log.timestamps)
    acceleration = _gradient(velocity, log.timestamps)
    jerk = _gradient(acceleration, log.timestamps)

    def estimate(values: np.ndarray) -> np.ndarray:
        observed = np.percentile(np.abs(values), percentile, axis=0)
        return np.maximum(observed * headroom, minimum_limit)

    return {
        "model": "measured_joint_log_percentile_v1",
        "source": log.source,
        "samples": int(len(log.timestamps)),
        "duration_seconds": float(log.timestamps[-1] - log.timestamps[0]),
        "percentile": float(percentile),
        "headroom": float(headroom),
        "joint_velocity_limits_rad_s": estimate(velocity).tolist(),
        "joint_acceleration_limits_rad_s2": estimate(acceleration).tolist(),
        "joint_jerk_limits_rad_s3": estimate(jerk).tolist(),
        "observed_peak_velocity_rad_s": np.max(np.abs(velocity), axis=0).tolist(),
        "observed_peak_acceleration_rad_s2": np.max(np.abs(acceleration), axis=0).tolist(),
        "observed_peak_jerk_rad_s3": np.max(np.abs(jerk), axis=0).tolist(),
    }


def calibrate_process_delays_from_csv(
    path: str | Path,
    *,
    percentile: float = 95.0,
    cycle_column: str = "cycle_id",
    event_column: str = "event",
    time_column: str = "timestamp_s",
) -> dict:
    """Estimate perception, vacuum, and release delays from event logs."""
    if not 50.0 <= percentile <= 100.0:
        raise ValueError("percentile must be in [50, 100]")
    path = Path(path)
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {cycle_column, event_column, time_column}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"process event log is missing columns: {sorted(required)}")
        cycles: dict[str, dict[str, float]] = {}
        for row in reader:
            cycle = str(row[cycle_column])
            event = str(row[event_column])
            timestamp = float(row[time_column])
            if not np.isfinite(timestamp):
                raise ValueError("process event log contains a non-finite timestamp")
            if event in cycles.setdefault(cycle, {}):
                raise ValueError(f"duplicate event {event!r} in cycle {cycle!r}")
            cycles[cycle][event] = timestamp
    pairs = {
        "perception_update_seconds": ("perception_start", "perception_ready"),
        "vacuum_establish_seconds": ("vacuum_on", "vacuum_sealed"),
        "release_seconds": ("release_command", "release_confirmed"),
    }
    result = {
        "model": "measured_process_event_percentile_v1",
        "source": str(path),
        "percentile": float(percentile),
        "cycles": len(cycles),
        "sample_counts": {},
    }
    for output_name, (start_event, end_event) in pairs.items():
        durations = []
        for events in cycles.values():
            if start_event in events and end_event in events:
                duration = events[end_event] - events[start_event]
                if duration < 0.0:
                    raise ValueError(f"event pair {start_event}->{end_event} has negative duration")
                durations.append(duration)
        if not durations:
            raise ValueError(f"no complete event pairs for {start_event}->{end_event}")
        result[output_name] = float(np.percentile(durations, percentile))
        result["sample_counts"][output_name] = len(durations)
    return result


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Calibrate execution assumptions from a controller joint log")
    parser.add_argument("--log", required=True)
    parser.add_argument("--event-log")
    parser.add_argument("--time-column", default="time_from_start_s")
    parser.add_argument("--percentile", type=float, default=99.5)
    parser.add_argument("--headroom", type=float, default=1.15)
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    result = calibrate_motion_limits(
        load_controller_csv(args.log, time_column=args.time_column),
        percentile=args.percentile,
        headroom=args.headroom,
    )
    if args.event_log:
        result["process_delays"] = calibrate_process_delays_from_csv(args.event_log)
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
