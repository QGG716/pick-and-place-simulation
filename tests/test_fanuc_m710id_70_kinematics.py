from pathlib import Path

import numpy as np

from unloading_sim.demo import build_robot as build_runtime_robot
from unloading_sim.fanuc_m710id70 import (
    HOME_JOINTS_RAD,
    build_robot,
    evaluate_task_point,
    official_reach_guard,
    trailer_obstacles,
    validate_model,
)
from unloading_sim.scene import load_scene_config


ROOT = Path(__file__).resolve().parents[1]


def test_zero_pose_axes_flange_and_jacobian_match_supplied_robocad():
    robot, _tool = build_robot(-0.70)
    result = validate_model(robot)
    assert result["status"] == "PASS_KINEMATICS_BLOCKED_INVERSE_DYNAMICS"
    assert result["checks"]["zero_flange_pose"]
    assert result["checks"]["joint_axis_origins"]
    assert result["checks"]["joint_axis_directions"]
    assert result["checks"]["jacobian_translation_finite_difference"]
    assert np.allclose(result["zero_pose_flange_xyz_from_mount_m"], [1.370, 0.0, 1.630])


def test_configured_home_is_valid_and_not_silently_accepted_if_colliding():
    robot, _tool = build_robot(-0.70)
    assert robot.within_limits(HOME_JOINTS_RAD)
    assert official_reach_guard(robot, HOME_JOINTS_RAD)
    assert not robot.collision_result(HOME_JOINTS_RAD, trailer_obstacles(), margin=0.001).in_collision


def test_known_reachable_and_unreachable_task_grasps_are_regression_fixtures():
    robot, _tool = build_robot(-0.70)
    reachable = evaluate_task_point(
        robot, 0.0, 0.90, [0.6, 0.4, 0.3], grasp_mode="top",
        seed_q=HOME_JOINTS_RAD, max_iterations=100,
    )
    assert reachable.task_reachable, reachable.failure_reason
    outside = evaluate_task_point(
        robot, 1.15, 2.70, [0.6, 0.4, 0.3], grasp_mode="front",
        seed_q=HOME_JOINTS_RAD, max_iterations=20,
    )
    assert not outside.task_reachable
    assert outside.failure_reason == "BOX_OUTSIDE_TRAILER"


def test_runtime_config_selects_new_model_without_overwriting_m20():
    _scene, config = load_scene_config(ROOT / "config/fanuc_m710id_70.yaml")
    robot = build_runtime_robot(config)
    assert robot.name == "fanuc_m710id_70"
    assert robot.dof == 6
    assert (ROOT / "assets/robots/fanuc_m20id35/m20_35_18d.urdf").exists()
