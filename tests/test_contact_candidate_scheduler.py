from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import json

import numpy as np
import pytest

from unloading_sim.contact_scheduler import ContactCandidateScheduler
from unloading_sim.geometry import OBB
from unloading_sim import layout_single_carton as production
from unloading_sim.layout_trajectory import LayoutTrajectoryBudget


def poses(lengths=(4,) * 12):
    result = []
    for family, count in enumerate(lengths):
        for variant in range(count):
            pose = np.eye(4)
            pose[0, 3] = .024 * variant
            result.append((("front", "left", "top")[family // 4],
                ((family % 4) * 90, pose, {"variant": variant,
                    "face_offset_local_xy_m": [.024 * variant, 0.],
                    "orientation_offset_local_xy_rad": [0., 0.]})))
    return result


def scheduler(pool=None, *, limit=48, complete_limit=48):
    return ContactCandidateScheduler(poses() if pool is None else pool,
        target=OBB([0, 0, 1], [.3, .2, .15], np.eye(3), "target", "carton"),
        context={"scene": "same", "policy": "same"}, request_seed=71070,
        batch_size=12, attempt_limit=limit, complete_connection_limit=complete_limit)


def finish(s, record, reason="NO_GEOMETRIC_CUP_CONTACT", *, connected=False, success=False):
    s.finish(record, {"failure_stage": "coverage" if reason == "NO_GEOMETRIC_CUP_CONTACT" else "pregrasp",
        "failure_reason": reason, "complete_trajectory": success,
        "search_status": "FINITE_SEARCH_NOT_INFEASIBILITY_PROOF"},
        complete_connection_attempted=connected, elapsed_s=.001)


@pytest.mark.parametrize("limit,unique,remaining,termination", [
    (48, 48, 0, "CANDIDATE_POOL_EXHAUSTED"),
    (36, 36, 12, "CANDIDATE_ATTEMPT_LIMIT_REACHED")])
def test_production_scheduler_advances_every_family(limit, unique, remaining, termination):
    s = scheduler(limit=limit)
    while (item := s.next_attempt()) is not None:
        finish(s, item[1])
    summary = s.summary()
    assert summary["generated_candidate_count"] == 48
    assert summary["unique_candidates_evaluated"] == summary["total_attempt_count"] == unique
    assert summary["retry_count"] == 0
    assert summary["remaining_unsearched_count"] == remaining
    assert summary["termination"] == termination
    for begin in range(0, unique, 12):
        batch = s.records[begin:begin+12]
        assert len({r["family_id"] for r in batch}) == 12
        assert {r["variant"]["variant"] for r in batch} == {begin // 12}
    assert sum(f["remaining_unsearched_count"] for f in summary["per_family"].values()) == remaining


@pytest.mark.parametrize("lengths", [(), (0,), (1, 0, 3, 2), (1, 5)])
def test_empty_and_unequal_families_exhaust_without_blocking(lengths):
    s = scheduler(poses(lengths))
    while (item := s.next_attempt()) is not None:
        finish(s, item[1])
    assert s.summary()["unique_candidates_evaluated"] == sum(lengths)
    assert s.summary()["remaining_unsearched_count"] == 0
    assert s.termination == "CANDIDATE_POOL_EXHAUSTED"


def test_identity_deduplicates_exact_context_but_keeps_tiny_pose_changes():
    pool = poses((1,))
    shifted = deepcopy(pool[0])
    shifted[1][1][0, 3] += 1e-12
    s = scheduler([pool[0], deepcopy(pool[0]), shifted])
    assert s.summary()["generated_candidate_count"] == 3
    assert s.summary()["unique_generated_candidate_count"] == 2
    assert s.summary()["duplicate_generated_candidate_count"] == 1


def test_unvisited_first_and_bounded_retries_have_reproducible_distinct_seeds():
    def trace():
        s = scheduler(poses((2, 1)), limit=20)
        while (item := s.next_attempt()) is not None:
            finish(s, item[1], "DISCONNECTED", connected=True)
        assert s.summary()["unique_candidates_evaluated"] == 3
        assert s.summary()["total_attempt_count"] == 9
        assert s.summary()["retry_count"] == 6
        assert [r["retry_index"] for r in s.records[:3]] == [0, 0, 0]
        for candidate_id in s.candidates:
            retries = [r for r in s.records if r["candidate_id"] == candidate_id]
            assert len({r["attempt_id"] for r in retries}) == 3
            assert len({r["ik_seed"] for r in retries}) == 3
            assert len({r["path_seed"] for r in retries}) == 3
            assert all(r["retry_reason"] for r in retries[1:])
        return s.records
    assert trace() == trace()


def test_deadline_connection_limit_success_and_numerical_timeout_are_distinct():
    s = scheduler()
    finish(s, s.next_attempt()[1], "GRASP_IK_DEADLINE")
    assert len(s.retries) == 1
    assert s.next_attempt(deadline_reached=True) is None
    assert s.summary()["termination"] == "PLANNING_WALL_CLOCK_DEADLINE"
    assert s.summary()["total_attempt_count"] == 1
    assert s.summary()["remaining_unsearched_count"] == 47
    assert not s.summary()["unsearched_candidates_are_infeasible"]
    s = scheduler(complete_limit=2)
    for _ in range(2):
        finish(s, s.next_attempt()[1], "DISCONNECTED", connected=True)
    assert s.next_attempt() is None
    assert s.termination == "COMPLETE_CONNECTION_ATTEMPT_LIMIT_REACHED"
    s = scheduler()
    finish(s, s.next_attempt()[1], None, connected=True, success=True)
    assert s.next_attempt() is None and s.termination == "COMPLETE_TRAJECTORY_FOUND"
    assert s.summary()["remaining_unsearched_count"] == 47


def test_existing_budget_compatibility_keeps_default_total_36():
    budget = LayoutTrajectoryBudget()
    assert budget.task_pose_batch_size == 12
    assert budget.task_complete_connection_attempt_limit == 36
    assert replace(budget, task_pose_connection_attempts=2).task_complete_connection_attempt_limit == 6


@pytest.fixture(scope="module")
def actual_scene():
    pytest.importorskip("pinocchio")
    pytest.importorskip("coal")
    policy = production.load_layout_motion_policy("configs/validation/m710id70_layout_v1_single_carton.yaml")
    scene = production.build_verified_motion_input(policy)
    connector = production._build_automatic_trajectory_connector(scene, policy.layout_validation.layout.robot()).connector
    assert connector is not None
    return scene, connector


def test_generation_preserves_all_configured_variants_and_fingerprint(actual_scene):
    scene, _ = actual_scene
    target = next(b for b in scene.cartons if b.name == scene.removable_cartons[0])
    faces = production._exposed_faces(scene, target.name)
    pool = list(production._scheduled_contact_poses(scene, target, faces))
    # This fixture's sparse cross has nominal + 4 offsets + 4 tilts per roll.
    assert len(pool) == len(faces) * 4 * 9
    assert len(pool) > scene.policy.data["search_strategy"]["grasp_poses_per_task"]
    assert {item[1][2]["variant"].split("_")[0] for item in pool} == {"nominal", "offset", "tilt"}
    identity = production.motion_implementation_identity(Path(__file__).resolve().parents[1])
    assert "src/unloading_sim/contact_scheduler.py" in identity["source_sha256"]


@pytest.mark.parametrize("mode", ["tail_success", "retry", "deadline", "attempt_limit"])
def test_real_audit_entry_dispatches_pose_and_actual_path_seed(actual_scene, monkeypatch, tmp_path, mode):
    scene, c = actual_scene
    evaluated, connected = [], []
    rng_states = []
    if mode == "retry":
        from unloading_sim import layout_trajectory
        original_planner = layout_trajectory.RRTConnectPlanner
        class RecordingPlanner(original_planner):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                rng_states.append(deepcopy(self.rng.bit_generator.state))
        monkeypatch.setattr(layout_trajectory, "RRTConnectPlanner", RecordingPlanner)
    first_target = []
    def pool(_scene, target, faces):
        if not first_target:
            first_target.append(target.name)
        return poses((1,)) if mode == "retry" and target.name == first_target[0] else (
            poses() if target.name == first_target[0] else [])
    def evaluate(scene, target, face, roll, physical, virtual, variant, seed, *args, **kwargs):
        evaluated.append({"target": target.name, "variant": dict(variant), "seed": seed,
                          "physical": physical.tolist(), "virtual": virtual.tolist()})
        valid = mode != "tail_success" or variant["variant"] == 1
        if mode == "deadline":
            c._deadline_monotonic = 0.
        return dict(face=face, failure_stage="trajectory_search" if valid else "coverage",
            failure_reason="TRAJECTORY_SEARCH_PENDING" if valid else "NO_GEOMETRIC_CUP_CONTACT",
            search_status="TEST_INJECTED_EVALUATION", strict_grasp_candidates=[{
                "q_rad": scene.policy.layout_validation.initial_q.tolist(), "candidate_id": "fixture_ik"}] if valid else [],
            ik_stream=None, path_connection_attempts=0, complete_trajectory=False,
            planning_timing_seconds={"coverage_and_candidate_qualification": 0.,
                                     "grasp_ik_inclusive_of_endpoint_validation": 0.})
    def branch(**kw):
        connected.append({"target": kw["target"].name, "pose": kw["requested_virtual_contact"].tolist(), "seed": kw["seed"]})
        if mode == "retry":
            # A zero-distance connection isolates RNG delivery into the real
            # RRT constructor without claiming a pick/place success.
            _, failure, _ = c._transit(kw["home_q"], kw["home_q"], kw["all_obstacles"],
                                      seed=kw["seed"], iteration_budget=1, stage="pregrasp")
            assert failure is None
        if mode == "tail_success":
            from test_layout_trajectory import _segment
            segment = _segment()
            segment["test_wiring_only_not_physical_success"] = True
            segment["target"] = kw["target"].name
            segment["contact"]["cup_selection"]["target_id"] = kw["target"].name
            failure = c._finalize_task(segment,
                [box for box in kw["all_obstacles"] if box.name != kw["target"].name] + [kw["target"]],
                kw["target"])
            return segment if failure is None else None, failure, {}
        return None, {"reason": "DISCONNECTED", "stage": "pregrasp"}, {}
    monkeypatch.setattr(production, "_scheduled_contact_poses", pool)
    monkeypatch.setattr(production, "_audit_pose", evaluate)
    monkeypatch.setattr(c, "_plan_branch", branch)  # plan() and seed forwarding remain production.
    result = production.run_layout_single_carton_audit(scene.policy, motion_input=scene, trajectory_connector=c)
    task = result["tasks"][0]
    summary = task["candidate_schedule"]
    assert len(evaluated) == summary["total_attempt_count"]
    assert len(connected) == summary["actual_complete_connection_attempt_count"]
    called = [a for a in task["attempts"] if a["scheduler"]["actual_complete_connection_attempted"]]
    assert [a["scheduler"]["path_seed"] for a in called] == [call["seed"] for call in connected]
    for attempt, observation in zip(task["attempts"], evaluated):
        assert attempt["scheduler"]["ik_seed"] == observation["seed"]
        assert attempt["scheduler"]["variant"] == observation["variant"]
        assert attempt["candidate_id"] == attempt["scheduler"]["candidate_id"]
    if mode == "tail_success":
        assert len(evaluated) == 13 and len(connected) == 1
        assert connected[0]["pose"] == evaluated[-1]["virtual"]
        assert evaluated[-1]["variant"]["variant"] == 1
        assert summary["termination"] == "COMPLETE_TRAJECTORY_FOUND"
        assert summary["remaining_unsearched_count"] == 35
    elif mode == "retry":
        assert len(connected) == 3 and len({x["seed"] for x in connected}) == 3
        assert rng_states == [np.random.default_rng(x["seed"]).bit_generator.state for x in connected]
        assert summary["retry_count"] == 2
        assert task["strict_grasp_candidate_count"] == 1
        assert task["strict_grasp_candidate_record_count"] == 3
    elif mode == "deadline":
        assert len(evaluated) == 1 and not connected
        assert summary["termination"] == "PLANNING_WALL_CLOCK_DEADLINE"
    else:
        assert len(evaluated) == len(connected) == 36
        assert summary["unique_candidates_evaluated"] == 36
        assert summary["retry_count"] == 0 and summary["remaining_unsearched_count"] == 12
        assert summary["termination"] == "CANDIDATE_ATTEMPT_LIMIT_REACHED"
    (tmp_path / "wiring.json").write_text(json.dumps({"mode": mode,
        "physical_robot_planning_success_claimed": False,
        "evaluations": evaluated, "actual_plan_branch_calls": connected,
        "actual_rrt_rng_states": rng_states, "summary": summary}, indent=2), encoding="utf-8")
