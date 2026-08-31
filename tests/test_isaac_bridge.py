from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle


def _config() -> dict:
    return {
        "execution": {
            "limits_source": "test",
            "joint_velocity_limits_rad_s": [2.0] * 6,
            "joint_acceleration_limits_rad_s2": [4.0] * 6,
            "joint_jerk_limits_rad_s3": [20.0] * 6,
            "joint_effort_limits_nm": [100.0] * 6,
            "motion_limit_scale": 0.5,
        },
        "planning": {"trajectory_waypoint_period_seconds": 0.02},
        "simulation_validation": {
            "vacuum_product_model": "上海皖泰真空吸盘三分区",
            "vacuum_cup_model": "FG42",
            "vacuum_cup_count": 72,
            "vacuum_cup_rows": 6,
            "vacuum_cup_columns": 12,
            "vacuum_cup_pitch_m": [0.048, 0.048],
            "vacuum_cup_radius_m": 0.0215,
            "vacuum_zone_count": 3,
            "vacuum_pull_off_force_per_cup_n": 59.0,
            "vacuum_shear_force_per_cup_n": 43.0,
            "vacuum_holding_torque_nm": None,
            "vacuum_cup_compression_m": 0.015,
            "vacuum_max_grip_distance_m": 0.020,
            "vacuum_surface_gripper_capture_tolerance_m": 0.005,
            "vacuum_footprint_size_m": [0.288, 0.576],
            "vacuum_effective_seal_area_m2": None,
            "vacuum_limits_calibrated": False,
            "vacuum_attachment_model": "per_sealed_cup",
            "vacuum_solver_attachment_model": "equivalent_center_of_pressure",
            "vacuum_gripper_mass_kg": 10.0,
            "vacuum_center_of_mass_from_flange_m": [0.1498, 0.0008, 0.0026],
            "vacuum_inertia_at_com_kg_m2": np.diag([0.315, 0.268, 0.096]).tolist(),
            "vacuum_flange_origin_step_mm": [-48.5617, -162.6592, 0.0],
            "vacuum_step_from_tool_rotation_matrix": [
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
        },
    }


def _plan() -> dict:
    return {
        "robot": {
            "model": "fanuc_m20id35",
            "urdf_path": "robot.urdf",
            "base_position": [1.0, 2.0, 0.3],
            "base_rpy": [0.0, 0.0, 0.1],
        },
        "segments": [
            {
                "pick_index": 4,
                "target": "box_a",
                "path": [np.zeros(6).tolist(), (np.ones(6) * 0.2).tolist(), np.zeros(6).tolist()],
                "grasp_index": 1,
                "release_index": 2,
                "sealed_cup_indices": list(range(60)),
                "sealed_cup_count": 60,
                "sealed_cups_per_zone": [18, 24, 18],
            }
        ],
    }


def test_build_fanuc_replay_is_deterministic_and_limit_audited():
    first = build_fanuc_isaac_replay_bundle(_plan(), _config())
    second = build_fanuc_isaac_replay_bundle(_plan(), _config())

    assert np.array_equal(first.timestamps_seconds, second.timestamps_seconds)
    assert np.array_equal(first.positions_rad, second.positions_rad)
    assert first.positions_rad.shape[1] == 6
    assert first.timestamps_seconds[0] == 0.0
    assert np.all(np.diff(first.timestamps_seconds) > 0.0)
    assert first.metadata["timing_audit"]["within_limits"]
    assert first.metadata["grasp_time_seconds"] > 0.0
    assert first.metadata["release_time_seconds"] == first.metadata["duration_seconds"]


def test_replay_inserts_stationary_vacuum_and_release_holds():
    cfg = _config()
    cfg["execution"]["vacuum_establish_seconds"] = 0.35
    cfg["execution"]["release_seconds"] = 0.15

    bundle = build_fanuc_isaac_replay_bundle(_plan(), cfg)

    assert bundle.metadata["duration_seconds"] == pytest.approx(
        bundle.metadata["motion_duration_seconds"] + 0.5
    )
    assert bundle.metadata["release_time_seconds"] == bundle.metadata["duration_seconds"]
    grasp_time = bundle.metadata["grasp_time_seconds"]
    grasp_indices = np.flatnonzero(np.isclose(bundle.timestamps_seconds, grasp_time))
    assert len(grasp_indices) == 1
    grasp_index = int(grasp_indices[0])
    establish_end = grasp_time + 0.35
    establish_index = int(np.flatnonzero(np.isclose(bundle.timestamps_seconds, establish_end))[0])
    assert np.array_equal(bundle.positions_rad[grasp_index], bundle.positions_rad[establish_index])
    assert np.array_equal(bundle.positions_rad[-2], bundle.positions_rad[-1])


def test_replay_settles_at_contact_before_closing_vacuum():
    cfg = _config()
    cfg["execution"]["pre_grasp_controller_settle_seconds"] = 0.25
    cfg["execution"]["vacuum_establish_seconds"] = 0.35
    cfg["execution"]["release_seconds"] = 0.15

    bundle = build_fanuc_isaac_replay_bundle(_plan(), cfg)

    arrival_time = bundle.metadata["grasp_arrival_time_seconds"]
    grasp_time = bundle.metadata["grasp_time_seconds"]
    assert grasp_time == pytest.approx(arrival_time + 0.25)
    assert bundle.metadata["duration_seconds"] == pytest.approx(
        bundle.metadata["motion_duration_seconds"] + 0.75
    )
    arrival_index = int(
        np.flatnonzero(np.isclose(bundle.timestamps_seconds, arrival_time))[0]
    )
    grasp_index = int(np.flatnonzero(np.isclose(bundle.timestamps_seconds, grasp_time))[0])
    establish_index = int(
        np.flatnonzero(np.isclose(bundle.timestamps_seconds, grasp_time + 0.35))[0]
    )
    assert np.array_equal(
        bundle.positions_rad[arrival_index], bundle.positions_rad[grasp_index]
    )
    assert np.array_equal(
        bundle.positions_rad[grasp_index], bundle.positions_rad[establish_index]
    )


def test_replay_time_scale_slows_physics_without_changing_joint_path():
    baseline = build_fanuc_isaac_replay_bundle(_plan(), _config())
    cfg = _config()
    cfg["execution"]["isaac_replay_time_scale"] = 1.6

    slowed = build_fanuc_isaac_replay_bundle(_plan(), cfg)

    assert slowed.metadata["isaac_replay_time_scale"] == 1.6
    assert slowed.metadata["motion_duration_seconds"] == pytest.approx(
        baseline.metadata["motion_duration_seconds"] * 1.6
    )
    assert slowed.metadata["source_motion_duration_seconds"] == pytest.approx(
        baseline.metadata["motion_duration_seconds"]
    )
    assert slowed.metadata["grasp_time_seconds"] == pytest.approx(
        baseline.metadata["grasp_time_seconds"] * 1.6
    )
    assert np.array_equal(slowed.positions_rad[0], baseline.positions_rad[0])
    assert np.array_equal(slowed.positions_rad[-1], baseline.positions_rad[-1])


def test_replay_time_scale_rejects_speedup_or_non_finite_value():
    for value in (0.9, float("nan")):
        cfg = _config()
        cfg["execution"]["isaac_replay_time_scale"] = value
        with pytest.raises(ValueError, match="isaac_replay_time_scale"):
            build_fanuc_isaac_replay_bundle(_plan(), cfg)


def test_replay_retimes_empty_tool_phase_after_stopped_release():
    baseline = build_fanuc_isaac_replay_bundle(_plan(), _config())
    cfg = _config()
    cfg["execution"]["release_seconds"] = 0.15
    cfg["execution"]["post_release_motion_limit_scale"] = 1.0
    plan = _plan()
    plan["segments"][0]["path"].append((np.ones(6) * -0.2).tolist())

    faster = build_fanuc_isaac_replay_bundle(plan, cfg)

    assert faster.metadata["post_release_motion_limit_scale"] == 1.0
    assert faster.metadata["timing_audit"]["phase_model"] == (
        "stopped_release_then_empty_tool"
    )
    assert faster.metadata["timing_audit"]["pre_release"]["within_limits"]
    assert faster.metadata["timing_audit"]["post_release"]["within_limits"]
    assert faster.metadata["motion_duration_seconds"] < (
        baseline.metadata["motion_duration_seconds"]
        + baseline.metadata["source_motion_duration_seconds"]
    )


def test_replay_derives_whole_head_forces_without_claiming_calibration():
    gripper = build_fanuc_isaac_replay_bundle(_plan(), _config()).metadata["gripper"]

    assert gripper["holding_force_n"] == 59.0 * 60
    assert gripper["holding_force_total_n"] == 59.0 * 60
    assert gripper["hardware_maximum_holding_force_total_n"] == 59.0 * 72
    assert gripper["product_model"] == "上海皖泰真空吸盘三分区"
    assert gripper["cup_model"] == "FG42"
    assert gripper["physical_cup_count"] == 72
    assert gripper["active_sealed_cup_count"] == 60
    assert gripper["pull_off_force_per_cup_n"] == 59.0
    assert gripper["shear_force_per_cup_n"] == 43.0
    assert gripper["shear_force_total_n"] == 43.0 * 60
    assert gripper["hardware_maximum_shear_force_total_n"] == 43.0 * 72
    assert gripper["solver_attachment_model"] == "equivalent_center_of_pressure"
    assert gripper["simulation_attachment_point_count"] == 1
    expected_center = np.mean(np.asarray(gripper["cup_centers_tool_yz_m"])[0:60], axis=0)
    assert gripper["solver_attachment_offsets_tool_yz_m"][0] == pytest.approx(expected_center)
    assert gripper["flange_origin_step_mm"] == [-48.5617, -162.6592, 0.0]
    assert gripper["holding_torque_nm"] is None
    assert gripper["footprint_size_m"] == [0.288, 0.576]
    assert gripper["effective_seal_area_m2"] is None
    assert gripper["limits_calibrated"] is False
    assert gripper["model_source"] == (
        "step_geometry_with_per_cup_forces_and_provisional_zone_mapping"
    )


def test_replay_derives_six_non_collinear_zone_row_band_solver_points():
    cfg = _config()
    cfg["simulation_validation"]["vacuum_solver_attachment_model"] = (
        "equivalent_zone_row_band_centers"
    )

    gripper = build_fanuc_isaac_replay_bundle(_plan(), cfg).metadata["gripper"]
    offsets = np.asarray(gripper["solver_attachment_offsets_tool_yz_m"], dtype=float)

    assert gripper["simulation_attachment_point_count"] == 6
    assert offsets.shape == (6, 2)
    assert len(np.unique(np.round(offsets[:, 0], 9))) == 3
    assert len(np.unique(np.round(offsets[:, 1], 9))) == 2


def test_build_fanuc_replay_rejects_wrong_robot_and_effort_dimension():
    plan = _plan()
    plan["robot"]["model"] = "other"
    with pytest.raises(ValueError, match="FANUC"):
        build_fanuc_isaac_replay_bundle(plan, _config())

    plan = _plan()
    cfg = _config()
    cfg["execution"]["joint_effort_limits_nm"] = [1.0, 2.0]
    with pytest.raises(ValueError, match="effort"):
        build_fanuc_isaac_replay_bundle(plan, cfg)


def test_build_fanuc_replay_serializes_docked_scene_and_dynamic_target():
    plan = _plan()
    plan["segments"][0]["amr_dock_position"] = [0.8, -0.3, 0.0]
    cfg = _config()
    cfg["scene"] = {
        "trailer": {"length": 4.0, "width": 2.35, "height": 3.0},
        "static_obstacles": [
            {"name": "belt", "center": [0.0, 0.0, 0.1], "size": [1.0, 0.5, 0.2]}
        ],
        "cartons": [
            {"name": "box_a", "center": [1.2, 0.0, 0.25], "size": [0.4, 0.5, 0.5]},
            {"name": "box_b", "center": [1.6, 0.0, 0.25], "size": [0.4, 0.5, 0.5]},
        ],
    }
    cfg["amr"] = {
        "footprint_size": [1.8, 1.5, 0.3],
        "platform_center_offset": [-1.0, 0.0, 0.15],
        "robot_mount_position": [-0.85, 0.3, 0.3],
        "mounted_surface_centers": {"belt": [-0.5, 0.2, 0.4]},
    }

    metadata = build_fanuc_isaac_replay_bundle(plan, cfg).metadata
    primitives = metadata["scene_primitives"]
    by_name = {item["name"]: item for item in primitives}

    assert metadata["base_position_m"] == pytest.approx([-0.05, 0.0, 0.3])
    assert len([item for item in primitives if item["category"] == "trailer"]) == 5
    assert by_name["belt"]["center_m"] == pytest.approx([0.3, -0.1, 0.4])
    assert by_name["amr_base"]["center_m"] == pytest.approx([-0.2, -0.3, 0.15])
    assert by_name["box_a"]["dynamic"] is True
    assert by_name["box_b"]["dynamic"] is False


def test_build_fanuc_replay_serializes_dynamic_conveyor_and_outfeed():
    plan = _plan()
    plan["segments"][0]["amr_dock_position"] = [0.8, -0.3, 0.0]
    cfg = _config()
    cfg["scene"] = {
        "trailer": {"length": 4.0, "width": 2.35, "height": 3.0},
        "static_obstacles": [
            {"name": "conveyor_deck", "center": [0.0, 0.0, 0.1], "size": [1.2, 0.75, 0.12]},
            {"name": "conveyor_cross_deck", "center": [0.0, 0.0, 0.1], "size": [0.45, 0.75, 0.12]},
        ],
        "cartons": [
            {"name": "box_a", "center": [1.2, 0.0, 0.25], "size": [0.4, 0.5, 0.5]},
        ],
    }
    cfg["amr"] = {
        "footprint_size": [1.8, 1.5, 0.3],
        "platform_center_offset": [-1.0, 0.0, 0.15],
        "mounted_surface_centers": {
            "conveyor_deck": [-0.7, -0.45, 0.37],
            "conveyor_cross_deck": [-0.325, 0.30, 0.37],
        },
    }
    cfg["simulation_validation"]["conveyor"] = {
        "enabled": True,
        "speed_m_s": 0.5,
        "surface_directions_world": {
            "conveyor_cross_deck": [0.0, -1.0, 0.0],
            "conveyor_deck": [-1.0, 0.0, 0.0],
            "conveyor_outfeed": [-1.0, 0.0, 0.0],
        },
        "outfeed_name": "conveyor_outfeed",
        "outfeed_mount_center": [-2.8, -0.45, 0.37],
        "outfeed_size_m": [3.0, 0.75, 0.12],
    }

    metadata = build_fanuc_isaac_replay_bundle(plan, cfg).metadata
    by_name = {item["name"]: item for item in metadata["scene_primitives"]}

    assert metadata["conveyor"]["speed_m_s"] == pytest.approx(0.5)
    assert by_name["conveyor_outfeed"]["center_m"] == pytest.approx([-2.0, -0.75, 0.37])
    assert by_name["conveyor_outfeed"]["size_m"] == pytest.approx([3.0, 0.75, 0.12])
