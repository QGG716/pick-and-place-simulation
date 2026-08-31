"""Deterministic joint-trajectory timing and discrete motion-limit audits.

The geometric planner returns collision-checked joint waypoints without time.
This module assigns monotonically increasing timestamps and expands segment
durations until finite-difference velocity, acceleration, and jerk estimates
respect explicit configured limits.  It deliberately has no controller or
heavyweight dynamics dependency, so it remains useful as a conservative
first-layer execution model and as an input contract for later backends.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Sequence

import numpy as np


@dataclass(frozen=True)
class JointMotionLimits:
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    limit_scale: float = 1.0
    minimum_segment_seconds: float = 0.002
    source: str = "unspecified"

    def __post_init__(self) -> None:
        velocity = np.asarray(self.velocity, dtype=float)
        acceleration = np.asarray(self.acceleration, dtype=float)
        jerk = np.asarray(self.jerk, dtype=float)
        if velocity.ndim != 1 or velocity.size == 0:
            raise ValueError("velocity limits must be a non-empty one-dimensional array")
        if acceleration.shape != velocity.shape or jerk.shape != velocity.shape:
            raise ValueError("velocity, acceleration, and jerk limits must have matching shapes")
        if not all(np.all(np.isfinite(values)) and np.all(values > 0.0) for values in (velocity, acceleration, jerk)):
            raise ValueError("all motion limits must be finite and strictly positive")
        if not np.isfinite(self.limit_scale) or not 0.0 < self.limit_scale <= 1.0:
            raise ValueError("limit_scale must be in (0, 1]")
        if not np.isfinite(self.minimum_segment_seconds) or self.minimum_segment_seconds <= 0.0:
            raise ValueError("minimum_segment_seconds must be finite and positive")
        object.__setattr__(self, "velocity", velocity.copy())
        object.__setattr__(self, "acceleration", acceleration.copy())
        object.__setattr__(self, "jerk", jerk.copy())

    @property
    def dof(self) -> int:
        return int(self.velocity.size)

    @property
    def effective_velocity(self) -> np.ndarray:
        return self.velocity * self.limit_scale

    @property
    def effective_acceleration(self) -> np.ndarray:
        return self.acceleration * self.limit_scale

    @property
    def effective_jerk(self) -> np.ndarray:
        return self.jerk * self.limit_scale


@dataclass(frozen=True)
class JointDriverLimits:
    continuous_torque_nm: np.ndarray
    peak_mechanical_power_w: np.ndarray
    maximum_payload_kg: float
    source: str = "unspecified"

    def __post_init__(self) -> None:
        torque = np.asarray(self.continuous_torque_nm, dtype=float)
        power = np.asarray(self.peak_mechanical_power_w, dtype=float)
        if torque.ndim != 1 or torque.size == 0 or power.shape != torque.shape:
            raise ValueError("driver torque and power limits must be matching non-empty arrays")
        if np.any(torque <= 0.0) or np.any(power <= 0.0) or not np.all(np.isfinite(torque)) or not np.all(np.isfinite(power)):
            raise ValueError("driver torque and power limits must be finite and positive")
        if not np.isfinite(self.maximum_payload_kg) or self.maximum_payload_kg < 0.0:
            raise ValueError("maximum_payload_kg must be finite and non-negative")
        object.__setattr__(self, "continuous_torque_nm", torque.copy())
        object.__setattr__(self, "peak_mechanical_power_w", power.copy())

    @property
    def dof(self) -> int:
        return int(self.continuous_torque_nm.size)


@dataclass(frozen=True)
class TimedTrajectory:
    positions: np.ndarray
    time_from_start: np.ndarray
    segment_velocity: np.ndarray
    waypoint_acceleration: np.ndarray
    segment_jerk: np.ndarray
    peak_velocity: np.ndarray
    peak_acceleration: np.ndarray
    peak_jerk: np.ndarray
    iterations: int

    @property
    def duration_seconds(self) -> float:
        return float(self.time_from_start[-1]) if self.time_from_start.size else 0.0

    def audit(self, limits: JointMotionLimits) -> dict[str, Any]:
        velocity_ratio = self.peak_velocity / limits.effective_velocity
        acceleration_ratio = self.peak_acceleration / limits.effective_acceleration
        jerk_ratio = self.peak_jerk / limits.effective_jerk
        return {
            "duration_seconds": self.duration_seconds,
            "waypoints": int(len(self.positions)),
            "iterations": int(self.iterations),
            "peak_velocity_rad_s": self.peak_velocity.tolist(),
            "peak_acceleration_rad_s2": self.peak_acceleration.tolist(),
            "peak_jerk_rad_s3": self.peak_jerk.tolist(),
            "max_velocity_ratio": float(np.max(velocity_ratio, initial=0.0)),
            "max_acceleration_ratio": float(np.max(acceleration_ratio, initial=0.0)),
            "max_jerk_ratio": float(np.max(jerk_ratio, initial=0.0)),
            "within_limits": bool(
                np.all(velocity_ratio <= 1.0 + 1e-8)
                and np.all(acceleration_ratio <= 1.0 + 1e-8)
                and np.all(jerk_ratio <= 1.0 + 1e-8)
            ),
        }


def motion_limits_from_config(cfg: dict[str, Any], dof: int) -> JointMotionLimits:
    execution = cfg.get("execution")
    if not isinstance(execution, dict):
        raise ValueError("config must define an execution section with explicit joint motion limits")
    required = (
        "joint_velocity_limits_rad_s",
        "joint_acceleration_limits_rad_s2",
        "joint_jerk_limits_rad_s3",
    )
    missing = [key for key in required if key not in execution]
    if missing:
        raise ValueError(f"execution motion limits are missing: {missing}")
    limits = JointMotionLimits(
        velocity=np.asarray(execution[required[0]], dtype=float),
        acceleration=np.asarray(execution[required[1]], dtype=float),
        jerk=np.asarray(execution[required[2]], dtype=float),
        limit_scale=float(execution.get("motion_limit_scale", 1.0)),
        minimum_segment_seconds=float(execution.get("minimum_segment_seconds", 0.002)),
        source=str(execution.get("limits_source", "unspecified")),
    )
    if limits.dof != int(dof):
        raise ValueError(f"execution limits contain {limits.dof} joints, expected {dof}")
    return limits


def _discrete_derivatives(positions: np.ndarray, durations: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dof = positions.shape[1]
    if len(positions) == 1:
        return np.zeros((0, dof)), np.zeros((1, dof)), np.zeros((0, dof))
    velocities = np.diff(positions, axis=0) / durations[:, None]
    accelerations = np.zeros_like(positions)
    accelerations[0] = 2.0 * velocities[0] / durations[0]
    accelerations[-1] = -2.0 * velocities[-1] / durations[-1]
    if len(positions) > 2:
        transition_seconds = durations[:-1] + durations[1:]
        accelerations[1:-1] = 2.0 * (velocities[1:] - velocities[:-1]) / transition_seconds[:, None]
    jerks = np.diff(accelerations, axis=0) / durations[:, None]
    return velocities, accelerations, jerks


def time_parameterize_joint_path(
    path: Sequence[Sequence[float] | np.ndarray],
    limits: JointMotionLimits,
    max_iterations: int = 200,
    tolerance: float = 1e-8,
) -> TimedTrajectory:
    """Assign timestamps and iteratively enforce discrete joint motion limits.

    The returned positions are unchanged, so the geometric collision proof is
    preserved.  Acceleration is estimated at path knots from adjacent segment
    velocities, including zero start/end velocity; jerk is the corresponding
    acceleration change across each segment.  This is a deterministic command
    schedule audit, not a replacement for controller-specific interpolation.
    """
    positions = np.asarray(path, dtype=float)
    if positions.size == 0:
        raise ValueError("path must contain at least one waypoint")
    if positions.ndim != 2 or positions.shape[1] != limits.dof:
        raise ValueError(f"path must have shape (N, {limits.dof})")
    if not np.all(np.isfinite(positions)):
        raise ValueError("path contains non-finite joint values")
    if len(positions) == 1:
        zeros = np.zeros(limits.dof)
        return TimedTrajectory(
            positions.copy(), np.array([0.0]), np.zeros((0, limits.dof)), np.zeros((1, limits.dof)),
            np.zeros((0, limits.dof)), zeros.copy(), zeros.copy(), zeros.copy(), 0
        )

    displacement = np.abs(np.diff(positions, axis=0))
    durations = np.maximum(
        np.max(displacement / limits.effective_velocity[None, :], axis=1),
        limits.minimum_segment_seconds,
    )

    iterations = 0
    for iterations in range(1, max_iterations + 1):
        velocities, accelerations, jerks = _discrete_derivatives(positions, durations)
        segment_scale = np.ones_like(durations)

        velocity_ratio = np.max(np.abs(velocities) / limits.effective_velocity[None, :], axis=1)
        segment_scale = np.maximum(segment_scale, velocity_ratio)

        acceleration_ratio = np.max(
            np.abs(accelerations) / limits.effective_acceleration[None, :], axis=1
        )
        for waypoint_index, ratio in enumerate(acceleration_ratio):
            if ratio <= 1.0 + tolerance:
                continue
            scale = np.sqrt(ratio) * 1.000001
            if waypoint_index > 0:
                segment_scale[waypoint_index - 1] = max(segment_scale[waypoint_index - 1], scale)
            if waypoint_index < len(durations):
                segment_scale[waypoint_index] = max(segment_scale[waypoint_index], scale)

        jerk_ratio = np.max(np.abs(jerks) / limits.effective_jerk[None, :], axis=1)
        for segment_index, ratio in enumerate(jerk_ratio):
            if ratio <= 1.0 + tolerance:
                continue
            scale = np.cbrt(ratio) * 1.000001
            first = max(0, segment_index - 1)
            last = min(len(durations), segment_index + 2)
            segment_scale[first:last] = np.maximum(segment_scale[first:last], scale)

        if np.max(segment_scale) <= 1.0 + tolerance:
            break
        durations *= segment_scale
    else:
        raise RuntimeError(f"trajectory timing did not converge after {max_iterations} iterations")

    velocities, accelerations, jerks = _discrete_derivatives(positions, durations)
    peak_velocity = np.max(np.abs(velocities), axis=0, initial=0.0)
    peak_acceleration = np.max(np.abs(accelerations), axis=0, initial=0.0)
    peak_jerk = np.max(np.abs(jerks), axis=0, initial=0.0)
    result = TimedTrajectory(
        positions.copy(),
        np.concatenate(([0.0], np.cumsum(durations))),
        velocities,
        accelerations,
        jerks,
        peak_velocity,
        peak_acceleration,
        peak_jerk,
        iterations,
    )
    if not result.audit(limits)["within_limits"]:
        raise RuntimeError("trajectory timing converged without satisfying motion limits")
    return result


def _waypoint_velocities(segment_velocity: np.ndarray, waypoint_count: int, dof: int) -> np.ndarray:
    velocities = np.zeros((waypoint_count, dof), dtype=float)
    if waypoint_count > 2:
        velocities[1:-1] = 0.5 * (segment_velocity[:-1] + segment_velocity[1:])
    return velocities


def scale_timed_trajectory_window(
    trajectory: TimedTrajectory,
    start_waypoint: int,
    end_waypoint: int,
    scale: float,
) -> TimedTrajectory:
    """Stretch a waypoint interval without changing its geometric path."""
    if not np.isfinite(scale) or scale < 1.0:
        raise ValueError("trajectory window scale must be finite and at least one")
    waypoint_count, dof = trajectory.positions.shape
    if not 0 <= start_waypoint <= end_waypoint < waypoint_count:
        raise ValueError("trajectory window waypoint indices are invalid")
    if waypoint_count <= 1 or start_waypoint == end_waypoint or scale == 1.0:
        return trajectory
    durations = np.diff(trajectory.time_from_start)
    durations[start_waypoint:end_waypoint] *= scale
    velocities, accelerations, jerks = _discrete_derivatives(
        trajectory.positions, durations
    )
    return TimedTrajectory(
        trajectory.positions.copy(),
        np.concatenate(([0.0], np.cumsum(durations))),
        velocities,
        accelerations,
        jerks,
        np.max(np.abs(velocities), axis=0, initial=0.0),
        np.max(np.abs(accelerations), axis=0, initial=0.0),
        np.max(np.abs(jerks), axis=0, initial=0.0),
        trajectory.iterations,
    )


def audit_driver_limits(
    trajectory: TimedTrajectory,
    limits: JointDriverLimits,
    inverse_dynamics: Callable[[np.ndarray, np.ndarray, np.ndarray, float], np.ndarray],
    *,
    payload_kg: float,
) -> dict[str, Any]:
    if limits.dof != trajectory.positions.shape[1]:
        raise ValueError("driver-limit DOF does not match trajectory")
    if not np.isfinite(payload_kg) or payload_kg < 0.0:
        raise ValueError("payload_kg must be finite and non-negative")
    if payload_kg > limits.maximum_payload_kg:
        raise ValueError(
            f"payload {payload_kg:.3f} kg exceeds driver certificate {limits.maximum_payload_kg:.3f} kg"
        )
    velocities = _waypoint_velocities(
        trajectory.segment_velocity, len(trajectory.positions), trajectory.positions.shape[1]
    )
    efforts = np.asarray(
        [
            inverse_dynamics(q, velocity, acceleration, float(payload_kg))
            for q, velocity, acceleration in zip(
                trajectory.positions, velocities, trajectory.waypoint_acceleration
            )
        ],
        dtype=float,
    )
    if efforts.shape != trajectory.positions.shape or not np.all(np.isfinite(efforts)):
        raise ValueError("inverse_dynamics must return one finite effort vector per waypoint")
    power = np.abs(efforts * velocities)
    peak_effort = np.max(np.abs(efforts), axis=0)
    peak_power = np.max(power, axis=0)
    torque_ratio = peak_effort / limits.continuous_torque_nm
    power_ratio = peak_power / limits.peak_mechanical_power_w
    return {
        "limits_source": limits.source,
        "payload_kg": float(payload_kg),
        "maximum_payload_kg": float(limits.maximum_payload_kg),
        "peak_joint_torque_nm": peak_effort.tolist(),
        "peak_joint_mechanical_power_w": peak_power.tolist(),
        "max_torque_ratio": float(np.max(torque_ratio, initial=0.0)),
        "max_power_ratio": float(np.max(power_ratio, initial=0.0)),
        "within_limits": bool(np.all(torque_ratio <= 1.0 + 1e-8) and np.all(power_ratio <= 1.0 + 1e-8)),
    }


def time_parameterize_joint_path_with_dynamics(
    path: Sequence[Sequence[float] | np.ndarray],
    motion_limits: JointMotionLimits,
    driver_limits: JointDriverLimits,
    inverse_dynamics: Callable[[np.ndarray, np.ndarray, np.ndarray, float], np.ndarray],
    *,
    payload_kg: float,
    max_iterations: int = 20,
) -> tuple[TimedTrajectory, dict[str, Any]]:
    """Conservatively slow a path until motion and simulated driver limits pass.

    The inverse-dynamics callback is backend-specific. It permits Pinocchio or
    a simulator to supply load-aware torque estimates without adding a heavy
    dependency to the core package. Global scaling is deterministic and safe,
    but is not claimed to be time-optimal.
    """
    trajectory = time_parameterize_joint_path(path, motion_limits)
    audit = audit_driver_limits(
        trajectory, driver_limits, inverse_dynamics, payload_kg=payload_kg
    )
    durations = np.diff(trajectory.time_from_start)
    for _ in range(max_iterations):
        if audit["within_limits"]:
            return trajectory, audit
        scale = max(
            1.01,
            float(np.sqrt(audit["max_torque_ratio"])),
            float(audit["max_power_ratio"]),
        )
        durations *= scale * 1.000001
        velocities, accelerations, jerks = _discrete_derivatives(trajectory.positions, durations)
        trajectory = TimedTrajectory(
            trajectory.positions.copy(),
            np.concatenate(([0.0], np.cumsum(durations))),
            velocities,
            accelerations,
            jerks,
            np.max(np.abs(velocities), axis=0, initial=0.0),
            np.max(np.abs(accelerations), axis=0, initial=0.0),
            np.max(np.abs(jerks), axis=0, initial=0.0),
            trajectory.iterations,
        )
        audit = audit_driver_limits(
            trajectory, driver_limits, inverse_dynamics, payload_kg=payload_kg
        )
    raise RuntimeError("driver-aware timing cannot satisfy torque/power limits; static load may be infeasible")


def time_parameterize_segments(segments: list[dict[str, Any]], cfg: dict[str, Any]) -> dict[str, Any]:
    """Attach timing to pick segments and return aggregate execution metrics."""
    if not segments:
        return {
            "segment_count": 0,
            "total_motion_seconds": 0.0,
            "total_cycle_seconds": 0.0,
            "theoretical_cases_per_hour": 0.0,
        }
    first_path = np.asarray(segments[0].get("path", []), dtype=float)
    if first_path.ndim != 2:
        raise ValueError("segment paths must be two-dimensional joint arrays")
    limits = motion_limits_from_config(cfg, first_path.shape[1])
    execution = cfg["execution"]
    perception_seconds = float(execution.get("perception_update_seconds", 0.0))
    vacuum_seconds = float(execution.get("vacuum_establish_seconds", 0.0))
    release_seconds = float(execution.get("release_seconds", 0.0))
    loaded_motion_time_scale = float(execution.get("loaded_motion_time_scale", 1.0))
    for name, value in (
        ("perception_update_seconds", perception_seconds),
        ("vacuum_establish_seconds", vacuum_seconds),
        ("release_seconds", release_seconds),
    ):
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
    if not np.isfinite(loaded_motion_time_scale) or loaded_motion_time_scale < 1.0:
        raise ValueError("loaded_motion_time_scale must be finite and at least one")

    total_motion = 0.0
    total_cycle = 0.0
    max_velocity_ratio = 0.0
    max_acceleration_ratio = 0.0
    max_jerk_ratio = 0.0
    for segment in segments:
        timed = time_parameterize_joint_path(segment.get("path", []), limits)
        grasp_index = int(segment.get("grasp_index", 0))
        release_index = int(segment.get("release_index", len(timed.positions) - 1))
        timed = scale_timed_trajectory_window(
            timed,
            grasp_index,
            release_index,
            loaded_motion_time_scale,
        )
        audit = timed.audit(limits)
        free_fall_seconds = np.sqrt(2.0 * max(0.0, float(segment.get("free_fall_height_m", 0.0))) / 9.81)
        process_seconds = perception_seconds + vacuum_seconds + release_seconds + free_fall_seconds
        cycle_seconds = timed.duration_seconds + process_seconds
        segment["time_from_start_seconds"] = timed.time_from_start.tolist()
        segment["timing"] = {
            **audit,
            "loaded_motion_time_scale": loaded_motion_time_scale,
            "process_overhead_seconds": process_seconds,
            "estimated_cycle_seconds": cycle_seconds,
        }
        total_motion += timed.duration_seconds
        total_cycle += cycle_seconds
        max_velocity_ratio = max(max_velocity_ratio, audit["max_velocity_ratio"])
        max_acceleration_ratio = max(max_acceleration_ratio, audit["max_acceleration_ratio"])
        max_jerk_ratio = max(max_jerk_ratio, audit["max_jerk_ratio"])

    return {
        "model": "discrete_waypoint_limit_audit_v1",
        "limits_source": limits.source,
        "motion_limit_scale": limits.limit_scale,
        "loaded_motion_time_scale": loaded_motion_time_scale,
        "joint_velocity_limits_rad_s": limits.velocity.tolist(),
        "joint_acceleration_limits_rad_s2": limits.acceleration.tolist(),
        "joint_jerk_limits_rad_s3": limits.jerk.tolist(),
        "segment_count": len(segments),
        "total_motion_seconds": total_motion,
        "total_cycle_seconds": total_cycle,
        "mean_cycle_seconds": total_cycle / len(segments),
        "theoretical_cases_per_hour": 3600.0 * len(segments) / total_cycle if total_cycle > 0.0 else 0.0,
        "max_velocity_ratio": max_velocity_ratio,
        "max_acceleration_ratio": max_acceleration_ratio,
        "max_jerk_ratio": max_jerk_ratio,
        "within_limits": bool(all(segment["timing"]["within_limits"] for segment in segments)),
    }
