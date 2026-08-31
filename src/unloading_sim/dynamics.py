"""Backend-neutral contracts for live dynamics and camera validation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class TimedJointCommands:
    timestamps: np.ndarray
    positions: np.ndarray

    def __post_init__(self) -> None:
        timestamps = np.asarray(self.timestamps, dtype=float)
        positions = np.asarray(self.positions, dtype=float)
        if timestamps.ndim != 1 or positions.ndim != 2 or len(timestamps) != len(positions):
            raise ValueError("timed commands require (N,) timestamps and (N, dof) positions")
        if len(timestamps) == 0 or not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(positions)):
            raise ValueError("timed commands must be non-empty and finite")
        if timestamps[0] != 0.0 or np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("timestamps must start at zero and increase strictly")
        object.__setattr__(self, "timestamps", timestamps.copy())
        object.__setattr__(self, "positions", positions.copy())

    @property
    def duration_seconds(self) -> float:
        return float(self.timestamps[-1])

    def sample(self, timestamp: float) -> np.ndarray:
        if not np.isfinite(timestamp):
            raise ValueError("sample timestamp must be finite")
        clamped = float(np.clip(timestamp, 0.0, self.duration_seconds))
        return np.asarray(
            [np.interp(clamped, self.timestamps, self.positions[:, joint]) for joint in range(self.positions.shape[1])],
            dtype=float,
        )


@dataclass(frozen=True)
class CameraSpec:
    name: str
    width: int
    height: int
    horizontal_fov_rad: float
    near_m: float = 0.03
    far_m: float = 10.0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera image size must be positive")
        if not 0.0 < self.horizontal_fov_rad < np.pi:
            raise ValueError("camera horizontal FOV must be in (0, pi)")
        if not 0.0 < self.near_m < self.far_m:
            raise ValueError("camera clipping planes are invalid")

    @property
    def intrinsic_matrix(self) -> np.ndarray:
        fx = 0.5 * self.width / np.tan(0.5 * self.horizontal_fov_rad)
        fy = fx
        return np.array([[fx, 0.0, self.width / 2.0], [0.0, fy, self.height / 2.0], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class CameraFrame:
    timestamp_seconds: float
    rgba: np.ndarray
    depth_m: np.ndarray
    segmentation: np.ndarray
    intrinsic_matrix: np.ndarray


class DynamicsValidationBackend(Protocol):
    name: str

    def run(self, commands: TimedJointCommands, realtime: bool = True) -> dict: ...


def tracking_error_audit(commanded: np.ndarray, measured: np.ndarray, timestamps: np.ndarray) -> dict:
    commanded = np.asarray(commanded, dtype=float)
    measured = np.asarray(measured, dtype=float)
    timestamps = np.asarray(timestamps, dtype=float)
    if commanded.shape != measured.shape or commanded.ndim != 2 or len(commanded) != len(timestamps):
        raise ValueError("tracking samples have incompatible shapes")
    errors = measured - commanded
    absolute = np.abs(errors)
    return {
        "samples": int(len(timestamps)),
        "duration_seconds": 0.0 if len(timestamps) == 0 else float(timestamps[-1] - timestamps[0]),
        "rms_joint_error_rad": np.sqrt(np.mean(errors**2, axis=0)).tolist() if len(errors) else [],
        "peak_joint_error_rad": np.max(absolute, axis=0).tolist() if len(errors) else [],
        "peak_error_rad": float(np.max(absolute, initial=0.0)),
    }
