from __future__ import annotations

import ast
import copy
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
from unloading_sim.m710_replay_contract import (
    M710ReplayContractError,
    build_replay_input_binding,
    canonical_sha256,
    verify_bundle_payload_sha256,
    verify_m710_replay_bundle,
)


ROOT = Path(__file__).resolve().parents[1]


def _m710_frame_contract() -> dict:
    rotation = [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]

    def transform(distance: float) -> list[list[float]]:
        return [
            [*rotation[0], distance],
            [*rotation[1], 0.0],
            [*rotation[2], 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    return {
        "schema": "m710id70_planner_tool_frames_v1",
        "transform_convention": "T_parent_child_maps_child_coordinates_into_parent",
        "active_physical_contact_frame": "nominal_compressed_contact",
        "T_flange_virtual_task_tcp": transform(0.2500),
        "T_flange_uncompressed_cup_plane": transform(0.2275),
        "T_flange_nominal_compressed_contact": transform(0.2175),
        "virtual_to_physical_contact_offset_m": 0.0325,
        "virtual_task_tcp_role": "strict_ik_fk_residual_only_never_attachment",
        "attachment_frame_role": "actual_physical_contact_only",
        "tool0_clocking_status": "PROVISIONAL_180_DEGREE_DISCREPANCY_UNRESOLVED",
        "execution_qualified": False,
    }


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


def _m710_config() -> dict:
    cfg = _config()
    frame_contract = _m710_frame_contract()
    cfg["tool"] = {
        "mass_kg": 20.0,
        "com_xyz_m": [0.1498, 0.0008, 0.0026],
        "inertia_tensor_com_kg_m2": np.diag([0.630, 0.536, 0.192]).tolist(),
        "planner_tool_length_m": 0.250,
        "fixed_mass_accounting_policy": (
            "FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE"
        ),
        "attach_to_link": "J6_link",
        "independent_tool_rigid_body_required": False,
        "frame_contract": copy.deepcopy(frame_contract),
        "geometry": {},
    }
    cfg["execution"].update(
        {
            "joint_effort_limits_nm": [8000.0, 9000.0, 5000.0, 1000.0, 800.0, 450.0],
            "joint_drive_stiffness_nm_rad": [80000.0, 80000.0, 60000.0, 12000.0, 8000.0, 5000.0],
            "joint_drive_damping_nm_s_rad": [5000.0, 5000.0, 4000.0, 800.0, 500.0, 300.0],
            "joint_gravity_feedforward_enabled": True,
            "joint_velocity_feedforward_enabled": True,
            "attached_payload_gravity_feedforward_enabled": True,
            "post_release_settle_seconds": 1.0,
        }
    )
    cfg["simulation_validation"].update(
        {
            "vacuum_cup_compression_m": 0.010,
            "vacuum_task_tcp_from_flange_m": 0.250,
            "vacuum_physical_uncompressed_face_from_flange_m": 0.2275,
            "vacuum_compressed_contact_plane_from_flange_m": 0.2175,
            "vacuum_frame_contract": copy.deepcopy(frame_contract),
            "vacuum_max_grip_distance_m": 0.002,
            "vacuum_surface_gripper_capture_tolerance_m": 0.002,
            "vacuum_maximum_contact_penetration_m": 0.0002,
            "vacuum_holding_torque_nm": 250.0,
            "vacuum_gripper_mass_kg": 20.0,
            "vacuum_inertia_at_com_kg_m2": np.diag([0.630, 0.536, 0.192]).tolist(),
            "robot_link_dynamics": [
                {
                    "link": name,
                    "mass_kg": 580.0 / 7.0,
                    "com_xyz_m": [0.0, 0.0, 0.1],
                    "inertia_at_com_kg_m2": np.diag([10.0, 11.0, 12.0]).tolist(),
                    "source_status": "ENGINEERING_ESTIMATE_NOT_MACHINE_QUALIFIED",
                }
                for name in ["base_link", *(f"J{index}_link" for index in range(1, 7))]
            ],
            "physics": {
                "parameter_status": "ENGINEERING_ESTIMATE_NOT_MACHINE_QUALIFIED",
                "gravity_world_m_s2": [0.0, 0.0, -9.81],
                "physics_time_step_s": 1.0 / 240.0,
                "contact_offset_m": 0.010,
                "rest_offset_m": 0.0,
                "execution_backend": {
                    "mode": "physx_cpu",
                    "device": "cpu",
                    "broadphase_type": "MBP",
                    "gpu_dynamics_enabled": False,
                    "fabric_enabled": True,
                    "ccd_enabled": True,
                },
                "materials": {
                    "carton": {
                        "static_friction": 0.55,
                        "dynamic_friction": 0.45,
                        "restitution": 0.02,
                    },
                    "painted_steel": {
                        "static_friction": 0.65,
                        "dynamic_friction": 0.50,
                        "restitution": 0.02,
                    },
                    "conveyor_belt": {
                        "static_friction": 0.80,
                        "dynamic_friction": 0.70,
                        "restitution": 0.02,
                    },
                    "robot_coating": {
                        "static_friction": 0.40,
                        "dynamic_friction": 0.30,
                        "restitution": 0.02,
                    },
                },
                "material_by_category": {
                    "carton": "carton",
                    "conveyor": "conveyor_belt",
                    "chassis": "painted_steel",
                    "floor": "painted_steel",
                    "side_wall": "painted_steel",
                },
                "damping": {
                    "robot_links": {
                        "linear_damping_s_inv": 0.05,
                        "angular_damping_s_inv": 0.10,
                    },
                    "tool": {
                        "linear_damping_s_inv": 0.05,
                        "angular_damping_s_inv": 0.10,
                    },
                    "cartons": {
                        "linear_damping_s_inv": 0.15,
                        "angular_damping_s_inv": 0.20,
                    },
                },
                "settling": {
                    "maximum_settle_time_s": 10.0,
                    "required_stable_duration_s": 0.5,
                    "max_linear_speed_m_s": 0.01,
                    "max_angular_speed_rad_s": 0.03,
                    "max_position_drift_m": 0.002,
                    "max_penetration_m": 0.001,
                },
            },
            "rendering": {
                "required_output": {
                    "width_px": 1920,
                    "height_px": 1080,
                    "fps": 30,
                    "camera_mode": "fixed_overview_with_contact_and_place_keyframes",
                },
                "material_palette": {
                    "chassis_rgb": [0.08, 0.09, 0.11],
                    "conveyor_rgb": [0.035, 0.04, 0.045],
                    "conveyor_frame_rgb": [0.32, 0.36, 0.40],
                    "conveyor_motion_marker_rgb": [0.62, 0.67, 0.70],
                    "conveyor_roller_rgb": [0.18, 0.20, 0.22],
                    "trailer_rgb": [0.56, 0.60, 0.64],
                },
                "conveyor_visual_motion": {
                    "model": "industrial_belt_surface_and_roller_phase_v2",
                    "markers_have_collision": False,
                    "markers_follow_active_physx_surface_velocity": True,
                    "rollers_follow_active_physx_surface_velocity": True,
                    "independent_phase_accumulators": True,
                    "stopped_surface_phase_is_frozen": True,
                },
                "pbr_materials": {
                    "conveyor_belt": {"rgb": [0.035, 0.04, 0.045], "roughness": 0.78, "metallic": 0.02},
                    "conveyor_frame": {"rgb": [0.32, 0.36, 0.40], "roughness": 0.28, "metallic": 0.82},
                    "conveyor_roller": {"rgb": [0.18, 0.20, 0.22], "roughness": 0.24, "metallic": 0.86},
                    "conveyor_seam": {"rgb": [0.62, 0.67, 0.70], "roughness": 0.62, "metallic": 0.18},
                    "trailer": {"rgb": [0.56, 0.60, 0.64], "roughness": 0.46, "metallic": 0.52},
                },
            },
        }
    )
    return cfg


def _m710_plan() -> dict:
    plan = _plan()
    plan["robot"] = {
        "model": "fanuc_m710id_70",
        "urdf_path": "assets/robots/fanuc_m710id_70/m710id_70.urdf",
        "base_position": [-1.41, 0.35, 0.6],
        "base_rpy": [0.0, 0.0, 0.0],
        "flange_offset_from_grasp_body_m": [0.175, 0.0, 0.0],
    }
    carton_inertia = np.diag([0.8854166667, 1.59375, 1.8416666667]).tolist()
    plan["scene_primitives"] = [
        {
            "name": f"carton_l{index // 5:02d}_c{index % 5:02d}",
            "category": "carton",
            "center_m": [0.3, -0.84 + 0.42 * (index % 5), 0.15 + 0.3 * (index // 5)],
            "size_m": [0.6, 0.4, 0.3],
            "rotation_matrix": np.eye(3).tolist(),
            "dynamic": True,
            "mass_kg": 42.5,
            "inertia_at_com_kg_m2": carton_inertia,
        }
        for index in range(40)
    ]
    plan["execution_asset_fingerprint_sha256"] = "a" * 64
    plan["execution_qualified"] = False
    plan["execution_blockers"] = ["M710_PER_LINK_CAD_MESH_CONVERSION_REQUIRED"]
    plan["collision_geometry"] = "cad_per_link_mesh_required"
    plan["segments"][0]["target"] = "carton_l07_c02"
    plan["segments"][0]["path"].append((np.ones(6) * -0.1).tolist())
    plan["segments"][0]["release_retreat_index"] = 3
    return plan


def _ready_m710_inputs(
    *,
    cfg: dict | None = None,
    segment: dict | None = None,
) -> tuple[dict, dict, dict]:
    """Build internally consistent, content-bound synthetic READY inputs."""

    configuration = copy.deepcopy(_m710_config() if cfg is None else cfg)
    plan = _m710_plan()
    if segment is not None:
        plan["segments"] = [copy.deepcopy(segment)]
    selected = copy.deepcopy(plan["segments"][0])
    identity = {
        "execution_config_fingerprint": "1" * 64,
        "layout_fingerprint": "2" * 64,
        "scene_fingerprint": "3" * 64,
        "motion_evidence_fingerprint": "4" * 64,
        "execution_implementation_source_sha256": {
            "src/unloading_sim/isaac_bridge.py": "5" * 64,
        },
        "dynamics_fingerprint": "6" * 64,
        "robot_manifest_sha256": "7" * 64,
        "tool_manifest_sha256": "8" * 64,
        "robot_missing_mesh_outputs": [],
    }
    execution_fingerprint = canonical_sha256(identity)
    plan.update(
        execution_asset_fingerprint_sha256=execution_fingerprint,
        simulation_execution_ready=True,
        execution_qualified=False,
        execution_blockers=[],
        machine_qualified=False,
        machine_qualification_warnings=[
            "ENGINEERING_DYNAMICS_NOT_MACHINE_QUALIFIED"
        ],
        collision_geometry="real_cad_per_link_visual_and_compound_collision_required",
    )
    plan_common = copy.deepcopy(plan)
    plan_common.pop("segments")
    primitives = copy.deepcopy(plan_common["scene_primitives"])
    binding = build_replay_input_binding(
        plan_common=plan_common,
        configuration=configuration,
        scene_primitives=primitives,
        trajectory_segment=selected,
        trajectory_segment_status="VERIFIED",
        input_identity=identity,
        execution_asset_fingerprint_sha256=execution_fingerprint,
    )
    preflight = {
        "schema": "m710id70_dynamic_execution_preflight_v1",
        "status": "READY",
        "simulation_execution_ready": True,
        "execution_qualified": False,
        "simulation_readiness_blockers": [],
        "blockers": [],
        "machine_qualified": False,
        "machine_qualification_warnings": [
            "ENGINEERING_DYNAMICS_NOT_MACHINE_QUALIFIED"
        ],
        "input_identity": identity,
        "execution_asset_fingerprint_sha256": execution_fingerprint,
        "asset_audit": {
            "robot": {
                "manifest_path": "assets/robots/fanuc_m710id_70/cad_asset_manifest.yaml",
                "manifest_sha256": identity["robot_manifest_sha256"],
                "source_integrity": True,
                "execution_qualified": True,
            },
            "tool": {
                "manifest_path": "assets/grippers/shanghai_wantai_three_zone/asset_manifest.yaml",
                "manifest_sha256": identity["tool_manifest_sha256"],
                "source_integrity": True,
                "execution_qualified": True,
            },
        },
        "motion": {"complete_trajectory_status": "PASS"},
        "scene": {"primitives": primitives},
        "replay_adapter_inputs": {
            "plan_common": plan_common,
            "configuration": copy.deepcopy(configuration),
            "trajectory_segment": selected,
            "trajectory_segment_status": "VERIFIED",
            "input_binding": binding,
        },
    }
    preflight["preflight_fingerprint"] = canonical_sha256(preflight)
    return plan, configuration, preflight


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
    assert gripper["solver_attachment_cup_counts"] == [12, 12, 12, 12, 6, 6]


def test_m710_bundle_binds_explicit_dynamics_full_stack_and_physical_cup_face():
    plan, cfg, preflight = _ready_m710_inputs()
    bundle = build_fanuc_isaac_replay_bundle(plan, cfg, preflight=preflight)
    metadata = bundle.metadata
    cartons = [item for item in metadata["scene_primitives"] if item["category"] == "carton"]

    assert metadata["robot_model"] == "fanuc_m710id_70"
    assert len(cartons) == 40 and all(item["dynamic"] for item in cartons)
    assert {item["mass_kg"] for item in cartons} == {42.5}
    assert len(metadata["robot_link_dynamics"]) == 7
    assert sum(item["mass_kg"] for item in metadata["robot_link_dynamics"]) == pytest.approx(
        600.0
    )
    mass_accounting = metadata["robot_tool_mass_accounting"]
    assert mass_accounting["policy"] == "FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE"
    assert mass_accounting["declared_policy"] == mass_accounting["policy"]
    assert mass_accounting["declared_policy"] == cfg["tool"]["fixed_mass_accounting_policy"]
    assert mass_accounting["grasp_body_link"] == "J6_link"
    assert mass_accounting["independent_tool_rigid_body_created"] is False
    assert mass_accounting["source_robot_mass_kg"] == pytest.approx(580.0)
    assert mass_accounting["tool_mass_kg"] == pytest.approx(20.0)
    assert mass_accounting["applied_articulation_mass_kg"] == pytest.approx(600.0)
    assert mass_accounting["tool_com_in_grasp_body_m"] == pytest.approx(
        [0.3248, 0.0008, 0.0026]
    )
    assert metadata["joint_effort_limits_nm"] == cfg["execution"]["joint_effort_limits_nm"]
    assert metadata["gripper"]["task_tcp_from_flange_m"] == pytest.approx(0.250)
    assert metadata["gripper"]["physical_uncompressed_face_from_flange_m"] == pytest.approx(0.2275)
    assert metadata["gripper"]["compressed_contact_plane_from_flange_m"] == pytest.approx(0.2175)
    assert metadata["gripper"]["virtual_tcp_beyond_uncompressed_face_m"] == pytest.approx(0.0225)
    assert metadata["gripper"]["max_grip_distance_m"] == pytest.approx(0.002)
    assert metadata["gripper"]["maximum_contact_penetration_m"] == pytest.approx(0.0002)
    assert metadata["physics"]["physics_time_step_s"] == pytest.approx(1.0 / 240.0)
    assert metadata["physics"]["contact_offset_m"] == pytest.approx(0.010)
    assert metadata["physics"]["rest_offset_m"] == pytest.approx(0.0)
    assert metadata["joint_velocity_feedforward_enabled"] is True
    assert metadata["attached_payload_gravity_feedforward_enabled"] is True
    assert metadata["actual_state_gates"]["maximum_free_transit_wait_s"] == pytest.approx(1.0)
    assert metadata["physics"]["friction_combine_mode"] == "min"
    assert metadata["physics"]["solver_position_iterations"] == 32
    assert metadata["rendering"]["required_output"] == {
        "width_px": 1920,
        "height_px": 1080,
        "fps": 30,
        "camera_mode": "fixed_overview_with_contact_and_place_keyframes",
        "render_every_physics_steps": 8,
    }
    assert metadata["rendering"]["material_palette"]["chassis_rgb"] == [
        0.08, 0.09, 0.11
    ]
    assert metadata["rendering"]["material_palette"]["conveyor_rgb"] == [
        0.035, 0.04, 0.045
    ]
    assert metadata["rendering"]["conveyor_visual_motion"][
        "markers_have_collision"
    ] is False
    assert metadata["required_post_release_settle_seconds"] == pytest.approx(1.0)
    assert metadata["simulation_execution_ready"] is True
    assert metadata["execution_qualified"] is False
    assert metadata["execution_blockers"] == []
    assert metadata["m710_execution_preflight"] == preflight
    assert metadata["m710_replay_contract"]["source_preflight_fingerprint"] == (
        preflight["preflight_fingerprint"]
    )


def test_m710_simulation_readiness_is_distinct_from_machine_qualification():
    plan, cfg, preflight = _ready_m710_inputs()
    metadata = build_fanuc_isaac_replay_bundle(
        plan, cfg, preflight=preflight
    ).metadata

    assert metadata["simulation_execution_ready"] is True
    assert metadata["execution_qualified"] is False
    assert metadata["machine_qualified"] is False
    assert metadata["execution_blockers"] == []
    assert metadata["machine_qualification_warnings"] == [
        "ENGINEERING_DYNAMICS_NOT_MACHINE_QUALIFIED"
    ]


def test_m710_random_hash_and_self_reported_ready_cannot_replace_full_preflight():
    plan = _m710_plan()
    plan.update(
        execution_asset_fingerprint_sha256="a" * 64,
        simulation_execution_ready=True,
        execution_qualified=False,
        execution_blockers=[],
        machine_qualified=False,
        machine_qualification_warnings=["NOT_MACHINE_QUALIFIED"],
    )
    with pytest.raises(ValueError, match="full verified execution preflight"):
        build_fanuc_isaac_replay_bundle(plan, _m710_config())


def test_m710_bundle_payload_hash_round_trip_and_tamper_detection():
    plan, cfg, preflight = _ready_m710_inputs()
    payload = build_fanuc_isaac_replay_bundle(
        plan, cfg, preflight=preflight
    ).to_dict()

    assert verify_bundle_payload_sha256(payload) == payload["bundle_payload_sha256"]
    assert verify_m710_replay_bundle(payload)["status"] == "PASS"

    changed = copy.deepcopy(payload)
    changed["positions_rad"][-1][0] += 0.01
    with pytest.raises(M710ReplayContractError, match="payload fingerprint"):
        verify_m710_replay_bundle(changed)


def test_m710_release_retreat_time_includes_every_inserted_hold():
    zero_cfg = _m710_config()
    zero_cfg["execution"].update(
        pre_grasp_controller_settle_seconds=0.0,
        vacuum_establish_seconds=0.0,
        release_seconds=0.0,
    )
    zero_plan, zero_cfg, zero_preflight = _ready_m710_inputs(cfg=zero_cfg)
    zero = build_fanuc_isaac_replay_bundle(
        zero_plan, zero_cfg, preflight=zero_preflight
    )

    held_cfg = _m710_config()
    held_cfg["execution"].update(
        pre_grasp_controller_settle_seconds=0.20,
        vacuum_establish_seconds=0.30,
        release_seconds=0.10,
    )
    held_plan, held_cfg, held_preflight = _ready_m710_inputs(cfg=held_cfg)
    held = build_fanuc_isaac_replay_bundle(
        held_plan, held_cfg, preflight=held_preflight
    )

    assert zero.metadata["grasp_time_seconds"] == pytest.approx(
        zero.metadata["grasp_arrival_time_seconds"]
    )
    assert zero.metadata["release_time_seconds"] == pytest.approx(
        zero.metadata["release_arrival_time_seconds"]
    )
    assert zero.metadata["release_retreat_time_seconds"] == pytest.approx(
        zero.metadata["motion_duration_seconds"]
    )
    assert held.metadata["grasp_time_seconds"] == pytest.approx(
        zero.metadata["grasp_time_seconds"] + 0.20
    )
    assert held.metadata["release_arrival_time_seconds"] == pytest.approx(
        zero.metadata["release_arrival_time_seconds"] + 0.20 + 0.30
    )
    assert held.metadata["release_time_seconds"] == pytest.approx(
        zero.metadata["release_time_seconds"] + 0.20 + 0.30 + 0.10
    )
    assert held.metadata["release_retreat_time_seconds"] == pytest.approx(
        zero.metadata["release_retreat_time_seconds"] + 0.20 + 0.30 + 0.10
    )
    assert (
        held.metadata["grasp_arrival_time_seconds"]
        <= held.metadata["grasp_time_seconds"]
        < held.metadata["release_arrival_time_seconds"]
        <= held.metadata["release_time_seconds"]
        < held.metadata["release_retreat_time_seconds"]
        <= held.metadata["duration_seconds"]
    )


@pytest.mark.parametrize(
    ("mutate", "match"),
    [
        (lambda segment: segment.pop("release_retreat_index"), "release_retreat_index"),
        (lambda segment: segment.update(release_retreat_index=1), "event order"),
    ],
)
def test_m710_missing_or_out_of_order_release_retreat_fails_closed(mutate, match):
    segment = copy.deepcopy(_m710_plan()["segments"][0])
    mutate(segment)
    with pytest.raises(M710ReplayContractError, match=match):
        _ready_m710_inputs(segment=segment)


def test_replay_adapter_source_keeps_m710_fail_closed_and_avoids_duplicate_ground():
    source = (ROOT / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8")

    ast.parse(source)
    assert "surface_solver_force_limit" not in source
    assert "surface_solver_shear_force_limit" not in source
    assert 'metadata.get("simulation_execution_ready") is not True' in source
    assert "synthetic_ground_created = not explicit_floor_declared" in source
    assert '"surface_gripper_torque_limit_applied": False' in source
    assert "physics_materials" in source
    assert "initial-state settling requires all snapshot dynamic cartons" in source
    assert "peak_penetration <= max_penetration" in source
    assert '"peak_position_drift_from_configured_m"' in source
    assert source.index("contract_module.verify_m710_replay_bundle(") < source.index(
        "from isaacsim import SimulationApp"
    )
    selector_call = source.index("desired_surfaces = select_active_conveyor_surfaces(")
    selector_consumed = source.index(
        "_apply_conveyor_surface_selection(desired_surfaces", selector_call
    )
    assert selector_call < selector_consumed
    physical_audit = source.index(
        "physical_contact_audit = audit_surface_attachment_contact("
    )
    audit_rejection = source.index("if not physical_contact_audit.accepted:", physical_audit)
    close_command = source.index("surface_gripper_interface.close_gripper", audit_rejection)
    assert physical_audit < audit_rejection < close_command


@pytest.mark.parametrize("mutation", [
    lambda plan, cfg: plan["scene_primitives"].pop(),
    lambda plan, cfg: plan.pop("execution_asset_fingerprint_sha256"),
    lambda plan, cfg: cfg["execution"].pop("joint_drive_stiffness_nm_rad"),
    lambda plan, cfg: cfg["simulation_validation"].pop("robot_link_dynamics"),
    lambda plan, cfg: plan["segments"][0]["path"][1].__setitem__(0, 0.123),
    lambda plan, cfg: plan["segments"][0].update(target="carton_l07_c01"),
])
def test_m710_bundle_fails_closed_on_input_that_differs_from_preflight(mutation):
    plan, cfg, preflight = _ready_m710_inputs()
    mutation(plan, cfg)
    with pytest.raises(M710ReplayContractError):
        build_fanuc_isaac_replay_bundle(plan, cfg, preflight=preflight)


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
