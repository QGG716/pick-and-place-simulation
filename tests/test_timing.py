import numpy as np
import pytest

from unloading_sim.timing import (
    JointDriverLimits,
    JointMotionLimits,
    audit_driver_limits,
    motion_limits_from_config,
    scale_timed_trajectory_window,
    time_parameterize_joint_path,
    time_parameterize_joint_path_with_dynamics,
    time_parameterize_segments,
)


def _limits() -> JointMotionLimits:
    return JointMotionLimits(
        velocity=np.array([1.0, 2.0]),
        acceleration=np.array([2.0, 3.0]),
        jerk=np.array([8.0, 10.0]),
        limit_scale=0.8,
        minimum_segment_seconds=0.001,
        source="test",
    )


def test_time_parameterization_is_deterministic_and_respects_limits():
    path = [np.array([0.0, 0.0]), np.array([0.4, 0.2]), np.array([0.8, -0.1])]

    first = time_parameterize_joint_path(path, _limits())
    second = time_parameterize_joint_path(path, _limits())

    assert np.allclose(first.positions, path)
    assert np.all(np.diff(first.time_from_start) > 0.0)
    assert np.allclose(first.time_from_start, second.time_from_start)
    assert first.audit(_limits())["within_limits"]


def test_direction_change_requires_more_than_velocity_only_timing():
    limits = _limits()
    path = np.array([[0.0, 0.0], [0.5, 0.0], [0.0, 0.0]])
    velocity_only = 2.0 * 0.5 / limits.effective_velocity[0]

    timed = time_parameterize_joint_path(path, limits)

    assert timed.duration_seconds > velocity_only
    assert timed.audit(limits)["max_acceleration_ratio"] <= 1.0 + 1e-8
    assert timed.audit(limits)["max_jerk_ratio"] <= 1.0 + 1e-8


def test_loaded_window_scaling_preserves_path_and_stretches_only_that_window():
    timed = time_parameterize_joint_path(
        [[0.0, 0.0], [0.2, 0.1], [0.4, 0.0], [0.6, -0.1]], _limits()
    )
    original_durations = np.diff(timed.time_from_start)

    scaled = scale_timed_trajectory_window(timed, 1, 3, 1.25)

    assert np.array_equal(scaled.positions, timed.positions)
    assert np.diff(scaled.time_from_start)[0] == pytest.approx(original_durations[0])
    assert np.allclose(
        np.diff(scaled.time_from_start)[1:3], original_durations[1:3] * 1.25
    )


def test_motion_limits_must_be_explicit_and_match_dof():
    with pytest.raises(ValueError, match="execution section"):
        motion_limits_from_config({}, 2)
    with pytest.raises(ValueError, match="expected 2"):
        motion_limits_from_config(
            {
                "execution": {
                    "joint_velocity_limits_rad_s": [1.0],
                    "joint_acceleration_limits_rad_s2": [1.0],
                    "joint_jerk_limits_rad_s3": [1.0],
                }
            },
            2,
        )


def test_invalid_or_empty_paths_are_rejected():
    with pytest.raises(ValueError, match="at least one"):
        time_parameterize_joint_path([], _limits())
    with pytest.raises(ValueError, match="shape"):
        time_parameterize_joint_path([[0.0, 0.0, 0.0]], _limits())


def test_segment_timing_reports_process_overhead_and_throughput():
    cfg = {
        "execution": {
            "joint_velocity_limits_rad_s": [1.0, 2.0],
            "joint_acceleration_limits_rad_s2": [2.0, 3.0],
            "joint_jerk_limits_rad_s3": [8.0, 10.0],
            "motion_limit_scale": 0.8,
            "limits_source": "test limits",
            "perception_update_seconds": 0.2,
            "vacuum_establish_seconds": 0.3,
            "release_seconds": 0.1,
        }
    }
    segments = [
        {
            "path": [[0.0, 0.0], [0.4, 0.2]],
            "free_fall_height_m": 0.0,
        }
    ]

    summary = time_parameterize_segments(segments, cfg)

    assert summary["within_limits"]
    assert summary["limits_source"] == "test limits"
    assert summary["total_cycle_seconds"] == pytest.approx(summary["total_motion_seconds"] + 0.6)
    assert summary["theoretical_cases_per_hour"] == pytest.approx(3600.0 / summary["total_cycle_seconds"])
    assert len(segments[0]["time_from_start_seconds"]) == len(segments[0]["path"])


def test_driver_aware_timing_slows_acceleration_dependent_torque():
    path = np.array([[0.0, 0.0], [0.8, 0.3], [0.0, -0.2]])
    drivers = JointDriverLimits(
        continuous_torque_nm=np.array([0.25, 0.25]),
        peak_mechanical_power_w=np.array([10.0, 10.0]),
        maximum_payload_kg=35.0,
        source="simulated test limits",
    )

    def simulated_inverse_dynamics(q, velocity, acceleration, payload_kg):
        del q, velocity
        return 0.05 + (0.15 + 0.002 * payload_kg) * acceleration

    geometric_timing = time_parameterize_joint_path(path, _limits())
    initial = audit_driver_limits(
        geometric_timing, drivers, simulated_inverse_dynamics, payload_kg=20.0
    )
    timed, audit = time_parameterize_joint_path_with_dynamics(
        path, _limits(), drivers, simulated_inverse_dynamics, payload_kg=20.0
    )

    assert not initial["within_limits"]
    assert audit["within_limits"]
    assert timed.duration_seconds > geometric_timing.duration_seconds
    assert audit["limits_source"] == "simulated test limits"


def test_driver_audit_rejects_payload_over_certificate():
    trajectory = time_parameterize_joint_path([[0.0, 0.0], [0.1, 0.1]], _limits())
    drivers = JointDriverLimits(np.ones(2), np.ones(2), maximum_payload_kg=5.0)
    with pytest.raises(ValueError, match="exceeds"):
        audit_driver_limits(
            trajectory,
            drivers,
            lambda q, velocity, acceleration, payload: np.zeros(2),
            payload_kg=5.1,
        )
