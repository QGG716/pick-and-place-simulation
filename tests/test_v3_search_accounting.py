import json
from pathlib import Path
from types import MethodType, SimpleNamespace

import numpy as np

from unloading_sim.geometry import OBB
from unloading_sim.ik import (IKResult, iter_ik_solutions, joint_solutions_equivalent,
                              solve_ik_multistart)
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell, evaluate_task
from unloading_sim.validation_physics import RigidAttachment
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


CONTACT_Q = np.array([-0.300479359612743, 1.2937834890279378, 0.4895834197945346,
                      3.666909416623987e-06, -0.7665782546700639, -4.411918905504988])


def _grid_022_cell():
    cfg = load_validation_config()
    cell = Cell(cfg)
    _, target, neighbors, valid = list(grid_tasks(cfg.data["scene"]))[22]
    conveyor = cfg.data["conveyor"]
    belt = (conveyor["fixed_extension_m"], conveyor["fixed_z_m"])
    decks = cell.decks(belt)
    obstacles = [*cell.fixtures(), *neighbors, *decks]
    assert valid
    return cell, target, obstacles, decks[1]


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


def test_pregrasp_endpoint_keeps_target_as_an_unattached_obstacle():
    cell, target, obstacles, _ = _grid_022_cell()

    contact_semantics = cell.state_failure(
        CONTACT_Q, [*obstacles, target], target_contact=target,
    )
    pregrasp_failure = cell.validate_pregrasp_endpoint(CONTACT_Q, obstacles, target)

    assert contact_semantics is None
    assert pregrasp_failure["reason"] == "PREGRASP_ENDPOINT_INVALID"
    assert pregrasp_failure["stage_failure"]["reason"] == "TOOL_COLLISION"
    assert target.name in pregrasp_failure["stage_failure"]["pair"]


def _archived_handoff_fixture():
    cell, target, obstacles, deck = _grid_022_cell()
    evidence_path = Path("docs/validation/evidence/m710id70_v3_hardening_baseline/") / \
                    "representative_success_grid_022.json"
    selected = json.loads(evidence_path.read_text(encoding="utf-8"))["task"]["selected"]
    q = np.asarray(selected["paths"]["carry"][-1])
    attachment = RigidAttachment(np.asarray(selected["tcp_from_box"]), target.half_extents,
                                 target.name)
    return cell, q, attachment, obstacles, deck


def test_handoff_candidate_filter_and_path_endpoint_share_support_semantics():
    cell, q, attachment, obstacles, deck = _archived_handoff_fixture()
    valid = lambda candidate: cell.validate_handoff_endpoint(
        candidate, obstacles, attachment, deck,
    )[1] is None

    result = cell.solve(cell.robot.fk(q), [q], 9022, valid, stage="handoff_test")
    support, endpoint_failure = cell.validate_handoff_endpoint(
        result.q, obstacles, attachment, deck,
    )

    assert result.success
    assert cell.path_failure([result.q], obstacles, attachment, [deck.name]) is None
    assert endpoint_failure is None
    assert support["supported"]


def test_handoff_support_contact_rejects_penetration_beyond_existing_tolerance():
    cell, q, attachment, obstacles, deck = _archived_handoff_fixture()
    shifted = OBB(
        deck.center + np.array([0.0, 0.0, 2 * cell.p["support_tolerance_m"]]),
        deck.half_extents, deck.rotation, deck.name, deck.category,
    )
    shifted_obstacles = [shifted if obstacle.name == deck.name else obstacle
                         for obstacle in obstacles]

    support, failure = cell.validate_handoff_endpoint(
        q, shifted_obstacles, attachment, shifted,
    )

    assert not support["supported"]
    assert failure["reason"] == "HANDOFF_ENDPOINT_COLLISION"
    assert failure["stage_failure"]["reason"] == "PAYLOAD_COLLISION"


def test_lazy_ik_stream_continues_after_converged_invalid_solution(monkeypatch):
    def fake_solve(_robot, _target, seed, **_kwargs):
        valid = bool(seed[0] > 0.5)
        return IKResult(valid, np.asarray(seed), 2, 0.0, 0.0,
                        "converged" if valid else "converged pose violates collision")

    monkeypatch.setattr("unloading_sim.ik.solve_ik", fake_solve)
    stream = iter_ik_solutions(
        OneAxisRobot(), np.eye(4), [np.array([0.0]), np.array([1.0])],
        random_restarts=0, candidate_limit=2,
    )

    result = next(stream)

    assert np.array_equal(result.q, [1.0])
    assert stream.evidence()["seeds_attempted"] == 2
    assert stream.evidence()["converged_pose_results"] == 2
    assert stream.evidence()["valid_solutions"] == 1


def _connection_cell():
    cell = object.__new__(Cell)
    cell.robot = OneAxisRobot()
    cell.p = {
        "stage_ik_search_mode": "multi_solution",
        "stage_ik_candidate_limit": 3,
        "stage_connection_attempt_limit": 3,
        "stage_connection_iteration_budget": 8,
        "ik_candidate_dedup_tolerance_rad": 1e-3,
        "ik_candidate_dedup_tolerance_m": 1e-4,
        "ik_restarts": 0,
        "ik_iterations": 5,
        "ik_damping": 0.01,
        "ik_max_step_rad": 1.0,
        "ik_position_tolerance_m": 1e-6,
        "ik_orientation_tolerance_rad": 1e-6,
        "ik_orientation_weight": 1.0,
        "rrt_iterations": 8,
    }
    cell.search_events = {"ik": [], "connection": []}
    return cell


def _successful_fake_ik(_robot, _target, seed, **_kwargs):
    return IKResult(True, np.asarray(seed), 1, 0.0, 0.0, "converged")


def test_second_valid_candidate_gets_real_connection_attempt_from_same_start(monkeypatch):
    monkeypatch.setattr("unloading_sim.ik.solve_ik", _successful_fake_ik)
    cell = _connection_cell()
    starts = []
    attachment = object()
    obstacles = [object()]

    def fake_transit(self, start, goal, seen_obstacles, seed, seen_attachment=None,
                     support_names=(), target_contact=None, stage="transit", max_iterations=None):
        starts.append(np.asarray(start).copy())
        assert seen_obstacles is obstacles
        assert seen_attachment is attachment
        success = bool(goal[0] > 0.5)
        consumed = 1 if success else max_iterations
        self.search_events["connection"].append({
            "stage": stage, "planning_iterations_consumed": consumed,
            "planning_iteration_budget": max_iterations, "termination": "CONNECTED" if success else "MAXIMUM_ITERATIONS_REACHED",
        })
        return ([np.asarray(start), np.asarray(goal)] if success else []), \
               (None if success else {"reason": "PATH_SEARCH_EXHAUSTED"})

    cell.transit = MethodType(fake_transit, cell)
    selected, path, failure, evidence = cell.connect_pose_candidates(
        np.eye(4), [np.array([0.0]), np.array([1.0])], 10, np.array([-1.0]),
        obstacles, 20, "approach", lambda _q: True, attachment=attachment,
    )

    assert failure is None and np.array_equal(selected.q, [1.0])
    assert np.array_equal(path[-1], selected.q)
    assert len(starts) == 2 and all(np.array_equal(start, [-1.0]) for start in starts)
    assert evidence["connection_attempts"][0]["failure"]["reason"] == "PATH_SEARCH_EXHAUSTED"
    assert evidence["connection_attempts"][1]["connection_success"]
    assert evidence["shared_rrt_iterations_consumed"] <= 8


def test_connector_rejects_actual_endpoint_that_differs_from_selected_candidate(monkeypatch):
    monkeypatch.setattr("unloading_sim.ik.solve_ik", _successful_fake_ik)
    cell = _connection_cell()

    def mismatched_transit(self, start, goal, _obstacles, _seed, *_args,
                           stage="transit", max_iterations=None, **_kwargs):
        self.search_events["connection"].append({
            "stage": stage, "planning_iterations_consumed": 0,
            "planning_iteration_budget": max_iterations, "termination": "DIRECT_EDGE",
        })
        return [np.asarray(start), np.asarray(goal) + 0.1], None

    cell.transit = MethodType(mismatched_transit, cell)
    selected, _, failure, evidence = cell.connect_pose_candidates(
        np.eye(4), [np.array([0.0])], 30, np.array([-1.0]), [], 40,
        "approach", lambda _q: True,
    )

    assert selected is None
    assert failure["reason"] == "CONNECTION_ENDPOINT_MISMATCH"
    assert not evidence["connection_attempts"][0]["connection_success"]


def test_joint_candidate_dedup_respects_bounded_continuous_and_prismatic_units():
    bounded = SimpleNamespace(active_joints=[SimpleNamespace(joint_type="revolute")])
    continuous = SimpleNamespace(active_joints=[SimpleNamespace(joint_type="continuous")])
    prismatic = SimpleNamespace(active_joints=[SimpleNamespace(joint_type="prismatic")])

    assert not joint_solutions_equivalent(
        bounded, np.array([0.0]), np.array([2 * np.pi]),
        revolute_tolerance_rad=1e-3, prismatic_tolerance_m=1e-4,
    )
    assert joint_solutions_equivalent(
        continuous, np.array([0.0]), np.array([2 * np.pi]),
        revolute_tolerance_rad=1e-3, prismatic_tolerance_m=1e-4,
    )
    assert not joint_solutions_equivalent(
        prismatic, np.array([0.0]), np.array([2e-4]),
        revolute_tolerance_rad=1e-3, prismatic_tolerance_m=1e-4,
    )


def test_near_duplicate_does_not_consume_lazy_candidate_limit(monkeypatch):
    monkeypatch.setattr("unloading_sim.ik.solve_ik", _successful_fake_ik)
    robot = SimpleNamespace(
        active_joints=[SimpleNamespace(joint_type="revolute")],
        joint_limits=np.array([[-2.0, 2.0]]),
    )
    stream = iter_ik_solutions(
        robot, np.eye(4), [np.array([0.0]), np.array([5e-4]), np.array([1.0])],
        random_restarts=0, candidate_limit=2, dedup_tolerance_rad=1e-3,
        dedup_tolerance_m=1e-4,
    )

    solutions = list(stream)

    assert [solution.q.tolist() for solution in solutions] == [[0.0], [1.0]]
    assert stream.evidence()["duplicate_candidates"] == 1
    assert stream.evidence()["deduplicated_candidates"] == 2


def test_multi_candidate_connections_share_budget_and_are_reproducible(monkeypatch):
    monkeypatch.setattr("unloading_sim.ik.solve_ik", _successful_fake_ik)

    def run_once():
        cell = _connection_cell()

        def failed_transit(self, _start, _goal, _obstacles, _seed, *_args,
                           stage="transit", max_iterations=None, **_kwargs):
            self.search_events["connection"].append({
                "stage": stage, "planning_iterations_consumed": max_iterations,
                "planning_iteration_budget": max_iterations, "termination": "MAXIMUM_ITERATIONS_REACHED",
            })
            return [], {"reason": "PATH_SEARCH_EXHAUSTED"}

        cell.transit = MethodType(failed_transit, cell)
        return cell.connect_pose_candidates(
            np.eye(4), [np.array([0.0]), np.array([0.5]), np.array([1.0])],
            50, np.array([-1.0]), [], 60, "carry", lambda _q: True,
        )[3]

    first = run_once()
    second = run_once()

    allocations = [item["rrt_iteration_allocation"] for item in first["connection_attempts"]]
    assert allocations == [3, 3, 2]
    assert first == second
    assert first["shared_rrt_iterations_consumed"] == 8
    assert first["termination"] == "STAGE_CONNECTION_ITERATION_BUDGET_EXHAUSTED"
