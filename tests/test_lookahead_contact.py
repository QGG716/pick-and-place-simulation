"""Focused official-model regressions; no Isaac world or historical task sweep.

The literal FK/start fixture comes from the retained first-carton contact in
20260915/runtime-repo/outputs/new_world_first_final. Only random IK restarts
are omitted for this known nearby branch; the 4 s / 12 s lookahead caps remain.
IK, cup geometry, exact endpoint collision and approach edges are production.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.layout_single_carton import (
    load_layout_motion_policy, build_verified_motion_input,
    _build_automatic_trajectory_connector,
)


CONTACT_Q = np.array([-.35901958261242695, -.21005953163247132,
    -.1488362496014934, 1.7325125925312939, .36397880205562794, -4.885213472402706])
START_Q = np.array([-.37183334454120454, -.245641902048464,
    -.17740205572363327, 1.7438672860074798, .37775574769199, -4.898300480640961])


@pytest.fixture(scope="module")
def official_scene():
    pytest.importorskip("pinocchio")
    pytest.importorskip("coal")
    policy = load_layout_motion_policy("configs/validation/m710id70_layout_v1_single_carton.yaml")
    scene = build_verified_motion_input(policy)
    build = _build_automatic_trajectory_connector(scene, policy.layout_validation.layout.robot())
    assert build.connector is not None, build.evidence
    return scene, build.connector


@pytest.fixture
def case(official_scene):
    scene, c = official_scene
    c.ik["random_restarts"] = 0  # One known local seed, not a search benchmark.
    c.start_planning_request()
    c.stack_carton_names = {b.name for b in scene.cartons}
    v = c.robot_state_validator
    c.collision_policy = replace(c.collision_policy, compliant_cup_neighbor_contact_mode="check")
    v.collision_policy = c.collision_policy
    v.commanded_cup_mask = [True, False] * 36  # Deliberately unrelated current-task mask.
    v.stack_carton_names = {"current_task_neighbor"}
    v.contact_target_name = "current_task"
    v.actual_contact_mask = [False] * 72  # Sentinel: planning must never write runtime contact.
    target = next(b for b in scene.cartons if b.name == "carton_l07_c02")
    candidate = dict(target=target, requested_virtual_contact=c.robot.fk(CONTACT_Q),
        face="front", suction=scene.policy.data["suction"], row_id="fixture_highest_row")
    c.next_contact_provider = lambda _: [candidate]
    placed = OBB([-5, 0, 1], [.3, .2, .15], np.eye(3), "previous_carton", "carton")
    return SimpleNamespace(c=c, v=v, scene=scene, target=target, candidate=candidate, placed=placed)


def context(case):
    c, v = case.c, case.v
    return deepcopy((v.commanded_cup_mask, v.stack_carton_names, v.contact_target_name,
                     c.stack_carton_names, v.actual_contact_mask, c._deadline_monotonic))


def run(case):
    return case.c._next_contact_cost(START_Q, case.placed, case.scene.all_obstacles,
                                    [case.placed], seed=71070)


def assert_restored_cache_is_equivalent_to_fresh(case):
    # Complete semantic keys permit incremental retention across contact scopes.
    # The restored permissions must give exactly the uncached verdict.
    c=case.c
    cached=c._state_failure(START_Q,case.scene.all_obstacles,stage='pregrasp')
    c._state_cache.clear()
    fresh=c._state_failure(START_Q,case.scene.all_obstacles,stage='pregrasp')
    assert cached==fresh


@pytest.mark.parametrize("neighbor_mode", ["check", "ignore"])
def test_production_entry_connects_with_candidate_masks_and_restores_context(case, neighbor_mode):
    c, v = case.c, case.v
    c.collision_policy = replace(c.collision_policy, compliant_cup_neighbor_contact_mode=neighbor_mode)
    v.collision_policy = c.collision_policy
    saved = context(case)
    assert c._state_failure(START_Q, case.scene.all_obstacles, stage="pregrasp") is None
    result = run(case)
    assert result["status"] == "CHECKED_NEXT_CONTACT_CONNECTION", result
    assert 0 < result["joint_path_length_rad"] < .1
    assert np.isfinite(result["joint_path_length_rad"])
    assert result["requires_actual_state_replan"]
    checks = result["endpoint_checks"]
    assert checks and checks[0]["endpoint_stage"] == "contact_endpoint"
    assert checks[0]["requested_stage"] == "next_contact"
    assert checks[0]["mask_source"] == "candidate_fk_target_face_suction"
    assert checks[0]["failure"] is None
    assert sum(checks[0]["commanded_active_mask"]) == 40
    assert checks[0]["commanded_active_mask"] != saved[0]
    assert context(case) == saved
    assert_restored_cache_is_equivalent_to_fresh(case)
    print("LOOKAHEAD", neighbor_mode, result["status"], result["joint_path_length_rad"])


def test_two_candidates_recompute_distinct_masks_before_exact_ik_check(case, monkeypatch):
    c = case.c
    shifted = deepcopy(case.candidate)
    shifted["requested_virtual_contact"][:3, 3] += (
        c.physical_from_virtual(shifted["requested_virtual_contact"])[:3, 0] * .048)
    c.next_contact_provider = lambda _: [case.candidate, shifted]
    saved = context(case)
    # Failure injection only for isolation; the positive test above runs the
    # complete real approach. Both IK streams, selections and exact checks here
    # remain production and must have accepted their candidate before this call.
    monkeypatch.setattr(c, "_approach", lambda *a, **kw:
        ([], [], {"reason": "INJECTED_PATH_FAILURE"}, {}))
    result = run(case)
    assert result["joint_path_length_rad"] is None and result["cost_status"] == "UNKNOWN"
    checks = [a["endpoint_checks"][0] for a in result["attempts"]]
    assert len(checks) == 2
    assert [sum(x["commanded_active_mask"]) for x in checks] == [40, 32]
    for check in checks:
        assert check["failure"] is None
        with c._contact_context():
            selected = c._contact_selection(np.array(check["q_rad"]), case.target,
                                           "front", case.candidate["suction"])
        assert check["commanded_active_mask"] == selected["commanded_active_mask"]
        assert check["geometrically_eligible_mask"] == selected["geometrically_eligible_mask"]
    assert context(case) == saved


@pytest.mark.parametrize("exit_mode", ["coverage", "path", "timeout", "exception"])
def test_failure_timeout_and_exception_restore_mutable_context(case, monkeypatch, exit_mode):
    c, v = case.c, case.v
    saved = context(case)
    select = c._contact_selection
    if exit_mode == "coverage":
        case.candidate["face"] = "top"  # Real full-ring/normal mismatch at front pose.
    else:
        def selection(*args, **kwargs):
            # Exercise in-place mutation before replacement, not just assignment.
            if isinstance(v.commanded_cup_mask, list):
                v.commanded_cup_mask[0] = not v.commanded_cup_mask[0]
            v.stack_carton_names.add("temporary")
            c.stack_carton_names.add("temporary")
            result = select(*args, **kwargs)
            if exit_mode == "exception":
                raise RuntimeError("injected endpoint exception")
            if exit_mode == "timeout":
                c._deadline_monotonic = 0.0
            return result
        monkeypatch.setattr(c, "_contact_selection", selection)
    if exit_mode == "path":
        monkeypatch.setattr(c, "_approach", lambda *a, **kw:
            ([], [], {"reason": "INJECTED_PATH_FAILURE"}, {}))
    if exit_mode == "exception":
        with pytest.raises(RuntimeError, match="injected endpoint exception"):
            run(case)
    else:
        result = run(case)
        assert result["joint_path_length_rad"] is None
        assert result["cost_status"] == "UNKNOWN"
        if exit_mode == "coverage":
            assert result["attempts"][0]["endpoint_checks"][0]["failure"]["reason"] == "NEXT_CONTACT_COVERAGE_FAILED"
        if exit_mode == "timeout":
            assert result["attempts"][0]["failure"]["reason"] == "LOOKAHEAD_DEADLINE"
    assert context(case) == saved
    assert_restored_cache_is_equivalent_to_fresh(case)


def test_exhausted_budget_returns_none_without_touching_context(case):
    case.c._lookahead_remaining_s = 0
    saved = context(case)
    result = run(case)
    assert result["status"] == "NOT_EVALUATED" and result["joint_path_length_rad"] is None
    assert context(case) == saved


def test_old_mask_stage_and_neighbor_cache_results_do_not_leak(case):
    c, v = case.c, case.v
    c._contact_selection(CONTACT_Q, case.target, "front", case.candidate["suction"])
    state = lambda stage: c._state_failure(CONTACT_Q, case.scene.all_obstacles,
                                          target_contact=case.target, stage=stage)
    good = tuple(v.commanded_cup_mask)
    assert state("contact_endpoint") is None  # Original approved inactive neighbor rule.
    bad = list(good)
    bad[0] = True  # This physical cup is over carton_l07_c03, not this target.
    v.commanded_cup_mask = np.array(bad, dtype=bool)
    assert state("contact_endpoint")["reason"] == "RIGID_TOOL_COLLISION"
    v.commanded_cup_mask = good
    assert state("contact_endpoint") is None
    for stage in ("pregrasp", "pregrasp_ik_endpoint", "next_contact_ik_endpoint",
                  "next_contact_unrecognized", "other_ik_endpoint"):
        assert state(stage)["reason"] == "RIGID_TOOL_COLLISION"
    v.stack_carton_names = set()
    assert state("contact_endpoint")["reason"] == "RIGID_TOOL_COLLISION"


@pytest.mark.parametrize("category", ["carton", "environment"])
def test_real_ik_endpoint_rejects_rigid_tool_collision(case, category):
    c = case.c
    tool = c.robot_state_validator.tool_transform_robot.tool_collision_obbs(CONTACT_Q)[0]
    blocker = OBB(tool.center, tool.half_extents, tool.rotation, "rigid_blocker", category)
    checks = []
    stream = c._ik_stream(c.robot.fk(CONTACT_Q), [CONTACT_Q], [*case.scene.all_obstacles, blocker],
        seed=71070, attachment=None, support_names=(), target_contact=case.target,
        stage="next_contact", contact_candidate=case.candidate, endpoint_checks=checks)
    assert next(stream, None) is None
    assert checks and checks[0]["commanded_active_mask"] is not None
    assert checks[0]["failure"]["reason"] in {"RIGID_TOOL_COLLISION", "ROBOT_MESH_COLLISION"}


def test_current_neighbor_ignore_policy_retains_target_and_environment_checks(case):
    c, v = case.c, case.v
    c.collision_policy = replace(c.collision_policy, compliant_cup_neighbor_contact_mode="ignore")
    v.collision_policy = c.collision_policy
    c._contact_selection(CONTACT_Q, case.target, "front", case.candidate["suction"])
    mask = list(v.commanded_cup_mask)
    mask[0] = True
    v.commanded_cup_mask = mask
    assert c._state_failure(CONTACT_Q, case.scene.all_obstacles,
                            target_contact=case.target, stage="contact_endpoint") is None
    # Pregrasp supplies no target-contact permission. The bound identity must
    # still keep this target out of the neighbor exemption.
    failure = c._state_failure(CONTACT_Q, case.scene.all_obstacles, stage="pregrasp")
    assert failure["reason"] == "RIGID_TOOL_COLLISION"
    assert failure["pair"][1] == case.target.name
    # Moving the real target into its cups also fails real seal geometry.
    shifted = OBB(case.target.center - np.array([.006, 0, 0]), case.target.half_extents,
                  case.target.rotation, case.target.name, case.target.category)
    with pytest.raises(ValueError, match="non-empty geometrically eligible"):
        c._contact_selection(CONTACT_Q, shifted, "front", case.candidate["suction"])
    checks = []
    world = [shifted if b.name == shifted.name else b for b in case.scene.all_obstacles]
    stream = c._ik_stream(c.robot.fk(CONTACT_Q), [CONTACT_Q], world,
        seed=71070, attachment=None, support_names=(), target_contact=shifted,
        stage="next_contact", contact_candidate=case.candidate, endpoint_checks=checks)
    assert next(stream, None) is None
    assert checks[0]["failure"]["reason"] == "NEXT_CONTACT_COVERAGE_FAILED"
    assert c._state_failure(CONTACT_Q, world, target_contact=shifted,
                            stage="contact_endpoint")["reason"] == "RIGID_TOOL_COLLISION"


def test_target_identity_is_part_of_cached_neighbor_permissions(case):
    c, v = case.c, case.v
    c.collision_policy = replace(c.collision_policy, compliant_cup_neighbor_contact_mode="ignore")
    v.collision_policy = c.collision_policy
    c._contact_selection(CONTACT_Q, case.target, "front", case.candidate["suction"])
    # Same q, mask, obstacles, stage: only the bound target identity changes.
    # No target_contact argument is supplied on the free pregrasp path.
    assert c._state_failure(CONTACT_Q, [case.target], stage="pregrasp") is not None
    v.contact_target_name = "another_current_target"
    assert c._state_failure(CONTACT_Q, [case.target], stage="pregrasp") is None
    v.contact_target_name = case.target.name
    assert c._state_failure(CONTACT_Q, [case.target], stage="pregrasp") is not None


def test_unknown_ik_stage_never_inherits_target_permission(case):
    c, v = case.c, case.v
    c.collision_policy = replace(c.collision_policy, compliant_cup_neighbor_contact_mode="ignore")
    v.collision_policy = c.collision_policy
    c._contact_selection(CONTACT_Q, case.target, "front", case.candidate["suction"])
    for stage in ("pregrasp", "next_contact_unknown", "unknown"):
        checks = []
        stream = c._ik_stream(c.robot.fk(CONTACT_Q), [CONTACT_Q], case.scene.all_obstacles,
            seed=71070, attachment=None, support_names=(), target_contact=case.target,
            stage=stage, endpoint_checks=checks)
        assert next(stream, None) is None
        assert checks[0]["failure"]["reason"] == "RIGID_TOOL_COLLISION"
        assert checks[0]["failure"]["pair"][1] == case.target.name
