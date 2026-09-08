import numpy as np

from unloading_sim.ik import solve_ik_multistart
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell, evaluate_task
from unloading_sim.validation_scenes import grid_tasks


class OneAxisRobot:
    joint_limits = np.array([[-2.0, 2.0]])

    def clamp(self, q):
        return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

    def fk(self, q):
        pose = np.eye(4)
        pose[0, 3] = q[0]
        return pose

    def geometric_jacobian(self, _q):
        jacobian = np.zeros((6, 1))
        jacobian[0, 0] = 1.0
        return jacobian

    def is_collision_free(self, _q, _obstacles, **_kwargs):
        return True


def test_legacy_multistart_reports_available_and_consumed_seed_budget():
    target = np.eye(4)
    result = solve_ik_multistart(
        OneAxisRobot(), target, [np.zeros(1)], random_restarts=2,
        rng=np.random.default_rng(4), max_iterations=5,
        position_tolerance=1e-6, orientation_tolerance=1e-6,
    )

    assert result.success
    assert result.search_evidence == {
        "policy": "legacy_first_valid_result",
        "seed_pool_available": 3,
        "explicit_seed_count": 1,
        "random_restart_count": 2,
        "seeds_attempted": 1,
        "last_seed_index": 0,
        "iterations_per_seed_available": 5,
        "iteration_capacity_available": 15,
        "iteration_capacity_for_attempted_seeds": 5,
        "iterations_consumed": 1,
        "converged_pose_results": 1,
        "valid_solutions": 1,
        "deduplicated_candidates": 1,
        "duplicate_candidates": 0,
        "termination": "FIRST_VALID_SOLUTION",
    }


def test_rrt_reports_real_state_edge_and_extension_work():
    planner = RRTConnectPlanner(
        np.array([-1.0]), np.array([1.0]), lambda _q: True,
        max_iterations=9, edge_resolution=0.1, rng=np.random.default_rng(2),
    )

    result = planner.plan(np.array([0.0]), np.array([0.25]))

    assert result.success and result.message == "direct edge"
    assert result.search_evidence["planning_iteration_budget"] == 9
    assert result.search_evidence["planning_iterations_consumed"] == 0
    assert result.search_evidence["state_validations"] == 5
    assert result.search_evidence["edge_validation_calls"] == 1
    assert result.search_evidence["edge_state_samples"] == 3
    assert result.search_evidence["extension_attempts"] == 0


def test_task_search_statistics_count_all_observed_ik_work():
    cfg = load_validation_config()
    cell = Cell(cfg)
    _, target, neighbors, valid = list(grid_tasks(cfg.data["scene"]))[20]
    conveyor = cfg.data["conveyor"]
    result = evaluate_task(
        cell, target, [target, *neighbors], np.asarray(cfg.data["robot"]["home_joints"]),
        (conveyor["fixed_extension_m"], conveyor["fixed_z_m"]),
        seed=cfg.data["planning"]["seed"] + 20, mode="fixed", grasp_only=True,
    )

    totals = result["search_statistics"]["totals"]
    assert valid
    assert totals["ik_calls"] == len(result["search_statistics"]["ik_events"])
    assert totals["ik_seeds_attempted"] >= totals["ik_calls"]
    assert totals["ik_iteration_capacity_available"] >= totals["ik_iterations_consumed"]
    assert totals["escape_robot_validations_all_attempts"] == 0
    assert result["search_statistics"]["budget_scope"]["escape"].endswith("not task-global")
