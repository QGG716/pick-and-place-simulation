"""Bounded source/entry regressions; real geometry where success is asserted."""
from copy import deepcopy
from dataclasses import replace
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import unloading_sim.history_candidates as history
import unloading_sim.history_adaptation as adaptation
import unloading_sim.layout_single_carton as audit
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState
from unloading_sim.workcell_layout import canonical_digest
from test_feasible_result_budget import clock


def write_motion(path, document):
    document = deepcopy(document)
    document.pop("evidence_fingerprint", None)
    document["evidence_fingerprint"] = canonical_digest(document)
    path.write_text(json.dumps(document))
    return document


@pytest.fixture
def source_fixture():
    root = Path(os.environ.get("M710_HISTORY_FIXTURE_ROOT", "inputs"))
    old_path = root / "history/original_motion.json"
    state_path = root / "actual_remaining_state.json"
    if not old_path.is_file() or not state_path.is_file():
        pytest.skip("requires the documented archived input copies on the GPU server")
    return json.loads(old_path.read_text()), json.loads(state_path.read_text())


@pytest.fixture
def actual_case(source_fixture):
    old, state = source_fixture
    scene = audit.build_verified_motion_input(audit.load_layout_motion_policy(
        "configs/validation/m710id70_layout_v1_single_carton.yaml"))
    rows = RowUnloadingState()
    rows.rank(scene.cartons, support_graph=scene.support_graph)
    scene = apply_actual_motion_state(scene, state, row_state=rows)
    build = audit._build_automatic_trajectory_connector(scene, scene.policy.layout_validation.layout.robot())
    c = build.connector
    assert c is not None
    c.start_planning_request()
    c.stack_carton_names = set(scene.remaining_stack_names)
    target = next(b for b in scene.cartons if b.name == old["selected_trajectory_segment"]["target"])
    return SimpleNamespace(old=old, state=state, scene=scene, c=c, backend=build.evidence, target=target, rows=rows)


@pytest.mark.parametrize("content", ["{", "[]", '123', '{"x":NaN}', '{"x":1e999}', '{"x":1,"x":2}'])
def test_corrupt_history_is_recorded_and_not_executed(tmp_path, content):
    (tmp_path / "candidate.json").write_text(content)
    source = history.HistorySource(history.HistoryPolicy(source=str(tmp_path)))
    assert not source.documents
    assert source.records[0]["status"] == "REJECTED"


def test_source_missing_empty_size_count_and_nonrecursive_bounds(tmp_path):
    assert history.HistorySource(history.HistoryPolicy()).records[0]["status"] == "NOT_CONFIGURED"
    missing = history.HistorySource(history.HistoryPolicy(source=str(tmp_path / "missing")))
    assert not missing.documents and missing.records[0]["status"] == "REJECTED"
    assert history.HistorySource(history.HistoryPolicy(source=str(tmp_path))).records[0]["status"] == "EMPTY_SOURCE"
    nested = tmp_path / "nested"
    nested.mkdir()
    (nested / "hidden.json").write_text("{}")
    for i in range(4):
        (tmp_path / f"{i}.json").write_text("x" * 100)
    source = history.HistorySource(history.HistoryPolicy(source=str(tmp_path), maximum_files=2, maximum_file_bytes=20))
    rejected = [r for r in source.records if r["status"] == "REJECTED"]
    assert len(rejected) == 2 and all("size limit" in r["reason"] for r in rejected)
    assert {"status": "INPUT_FILE_LIMIT", "omitted_files": 2} in source.records


@pytest.mark.parametrize("change", ["model", "tool", "layout", "dimensions", "joint_order", "stage", "target"])
def test_incompatible_hints_fail_before_adaptation(actual_case, change):
    case = actual_case
    old = deepcopy(case.old)
    if change in {"model", "tool"}:
        key = "urdf_sha256" if change == "model" else "rigid_tool_compound_q0_sha256"
        old["trajectory_backend"]["source_identity"][key] = "wrong"
    elif change == "layout":
        old["layout_fingerprint"] = "wrong"
    elif change == "dimensions":
        old["selected_trajectory_segment"]["place"]["selection"]["half_extents_m"][0] += .001
    elif change == "joint_order":
        old["history_compatibility"] = {"joint_names": list(reversed(case.scene.snapshot["robot"]["joint_names"]))}
    elif change == "stage":
        old["selected_trajectory_segment"]["stage_ranges"]["extraction"][0] += 1
    else:
        old["selected_trajectory_segment"]["target"] = "unrelated"
    with pytest.raises(ValueError):
        history.compatible_hint(old, case.scene, case.c, case.target, case.backend, history.HistoryPolicy())


def test_old_policy_and_implementation_are_provenance_not_inherited_permission(actual_case):
    case = actual_case
    old = deepcopy(case.old)
    old["policy_fingerprint"] = "different"
    old["scene_fingerprint"] = "different"
    old["implementation_identity"] = {"obsolete": True}
    hint = history.compatible_hint(old, case.scene, case.c, case.target, case.backend, history.HistoryPolicy())
    nominal, offset = next(history.contact_variants(hint, case.target))
    np.testing.assert_allclose(nominal, case.target.world_from_local @ np.linalg.inv(
        hint["old_target_pose"]) @ old["selected_trajectory_segment"]["contact"]["actual_physical_contact_pose_world"])
    assert offset["normal_outward_m"] == 0
    moved = replace(case.target, center=case.target.center + [.05, 0, 0])
    with pytest.raises(ValueError, match="local adaptation"):
        history.compatible_hint(old, case.scene, case.c, moved, case.backend, history.HistoryPolicy())


@pytest.mark.parametrize("mode", ["empty", "corrupt", "model", "target", "adaptation_failure"])
def test_default_audit_source_falls_back_in_same_target_order(actual_case, monkeypatch, tmp_path, mode):
    case = actual_case
    old = deepcopy(case.old)
    if mode == "corrupt":
        (tmp_path / "old.json").write_text("[")
    elif mode != "empty":
        if mode == "model":
            old["trajectory_backend"]["source_identity"]["urdf_sha256"] = "wrong"
        if mode == "target":
            old["selected_trajectory_segment"]["target"] = "unrelated"
        write_motion(tmp_path / "old.json", old)
    data = deepcopy(case.scene.policy.data)
    data["search_strategy"]["history"] = {"source": str(tmp_path), "maximum_variants": 1}
    policy = replace(case.scene.policy, data=data)
    scene = replace(case.scene, policy=policy)
    calls, history_calls = [], []
    original_pool = audit._scheduled_contact_poses
    monkeypatch.setattr(audit, "_scheduled_contact_poses", lambda scene, target, faces:
                        list(original_pool(scene, target, faces))[:1])
    def evaluate(scene, target, face, *args, **kwargs):
        calls.append(target.name)
        return dict(face=face, strict_grasp_candidates=[], ik_stream=None, path_connection_attempts=0,
            failure_stage="coverage", failure_reason="NO_GEOMETRIC_CUP_CONTACT", complete_trajectory=False)
    monkeypatch.setattr(audit, "_audit_pose", evaluate)
    def fail(c, **kwargs):
        history_calls.append((kwargs["target"].name, c._request_deadline_monotonic, c._deadline_monotonic))
        return None, {"reason": "INJECTED_ADAPTATION_FAILURE", "stage": "pregrasp"}, {}
    monkeypatch.setattr(adaptation, "adapt_branch", fail)
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnectorBuildResult
    monkeypatch.setattr(audit, "_build_automatic_trajectory_connector", lambda *args:
        LayoutTrajectoryConnectorBuildResult(case.c, "AVAILABLE", None, case.backend))
    result = audit.run_layout_single_carton_audit(policy, motion_input=scene,
        row_state=case.rows)
    assert calls and calls[0] == case.target.name
    assert result["complete_trajectory_status"] == "FAIL_CLOSED"
    if mode == "adaptation_failure":
        assert history_calls[0][0] == calls[0]
        assert history_calls[0][2] < history_calls[0][1]
        assert result["tasks"][0]["attempts"][0]["failure_reason"] == "INJECTED_ADAPTATION_FAILURE"
        assert result["tasks"][0]["history_attempt_count"] == 1
    else:
        assert not history_calls


def test_contact_variants_follow_current_frame_and_do_not_force_offset():
    from unloading_sim.geometry import OBB, rotation_matrix_from_rotation_vector
    target = OBB([.1, .2, .3], [.3, .2, .15], rotation_matrix_from_rotation_vector([.2, -.4, .3]), "opaque", "carton")
    hint = {"policy": history.HistoryPolicy(), "local_contact": np.eye(4)}
    values = list(history.contact_variants(hint, target))
    assert len(values) == 4
    for (pose, offset), distance in zip(values, [0., .0005, .001, .0015]):
        np.testing.assert_allclose(pose[:3, 3], target.center - target.rotation[:, 2] * distance)
        assert offset["normal_outward_m"] == distance
    with pytest.raises(ValueError):
        history.HistoryPolicy(maximum_normal_offset_m=.0021)


def test_precise_contact_correction_keeps_real_ik_coverage_and_collision(monkeypatch):
    from test_lookahead_contact import CONTACT_Q
    scene = audit.build_verified_motion_input(audit.load_layout_motion_policy(
        "configs/validation/m710id70_layout_v1_single_carton.yaml"))
    c = audit._build_automatic_trajectory_connector(scene, scene.policy.layout_validation.layout.robot()).connector
    c.start_planning_request()
    c.stack_carton_names = {b.name for b in scene.cartons}
    target = next(b for b in scene.cartons if b.name == "carton_l07_c02")
    physical = c.physical_from_virtual(c.robot.fk(CONTACT_Q))
    hint = {"policy": history.HistoryPolicy(), "local_contact": np.linalg.inv(target.world_from_local) @ physical}
    original = c._state_failure
    calls = []
    def injected(q, *args, **kwargs):
        failure = original(q, *args, **kwargs)
        calls.append(failure)
        delta = c.physical_from_virtual(c.robot.fk(q))[:3, 3] - physical[:3, 3]
        # Fault boundary only; exact official geometry is still checked first.
        if failure is None and float(delta @ -physical[:3, 2]) < .00025:
            return {"reason": "INJECTED_RIGID_TOOL_COLLISION", "stage": "contact_endpoint"}
        return failure
    monkeypatch.setattr(c, "_state_failure", injected)
    records = []
    with c._contact_context():
        for pose, offset in history.contact_variants(hint, target):
            solved = adaptation.solve_local(c, c.virtual_from_physical(pose), CONTACT_Q)
            assert solved.success
            selection = c._contact_selection(solved.q, target, "front", scene.policy.data["suction"])
            failure = c._state_failure(solved.q, scene.all_obstacles, target_contact=target, stage="contact_endpoint")
            records.append((offset, selection, failure))
            if failure is None:
                break
    assert records[0][2]["reason"] == "INJECTED_RIGID_TOOL_COLLISION"
    assert records[-1][2] is None and records[-1][0]["normal_outward_m"] == .0005
    assert all(item is None for item in calls)  # Actual official collision checker ran.
    assert sum(records[-1][1]["commanded_active_mask"]) > 0


def test_registration_never_claims_physical_completion(tmp_path):
    result = {"complete_trajectory_status": "PASS", "selected_trajectory_segment": {"target": "opaque"}}
    result["evidence_fingerprint"] = canonical_digest(result)
    preflight = {"simulation_execution_ready": True,
                 "input_identity": {"motion_evidence_fingerprint": result["evidence_fingerprint"]}}
    registered = history.register_planned_motion(tmp_path, result, preflight)
    assert not registered["physical_execution_completed"]
    assert not registered["completed_carton_ids_modified"]
    assert json.loads(Path(registered["path"]).read_text()) == result
    assert history.register_planned_motion(tmp_path, result, preflight) == registered
    with pytest.raises(ValueError):
        history.register_planned_motion(tmp_path, result, {"simulation_execution_ready": False})
    with pytest.raises(ValueError, match="bound motion"):
        history.register_planned_motion(tmp_path, {**result, "extra": "changed"}, preflight)


@pytest.mark.parametrize("mode", ["changed_mask", "blocked_join", "changed_obstacle", "partial_approach"])
def test_adaptation_rechecks_current_context_before_completion(actual_case, monkeypatch, mode):
    from unloading_sim.geometry import OBB
    case = actual_case
    old = deepcopy(case.old)
    # An obsolete mask cannot dictate this request's independent cup commands.
    selection = old["selected_trajectory_segment"]["contact"]["cup_selection"]
    historical_mask = [i == selection["commanded_active_mask"].index(True) for i in range(72)]
    for name in ("commanded_active", "actual_contact"):
        selection[name + "_mask"] = historical_mask.copy()
        selection[name + "_count"] = 1
        selection[name + "_indices"] = [historical_mask.index(True)]
        selection[name + "_ids"] = [selection["mask_bit_order_cup_ids"][historical_mask.index(True)]]
    hint = history.compatible_hint(old, case.scene, case.c, case.target, case.backend, history.HistoryPolicy())
    hint["attempt_provenance"] = {"old_commanded_mask": historical_mask}
    physical, _ = list(history.contact_variants(hint, case.target))[1]
    obstacles = list(case.scene.all_obstacles)
    if mode == "changed_obstacle":
        # New non-target rigid geometry at the contact position, not an exempt stack carton.
        obstacles.append(OBB(physical[:3, 3], [.15, .15, .15], np.eye(3), "new_fixture", "wall"))
    seen = []
    real_path = case.c._path_failure
    def path_check(points, *args, **kwargs):
        failure = real_path(points, *args, **kwargs)
        seen.append(kwargs["stage"])
        if failure is None and mode == "blocked_join" and kwargs["stage"] == "pregrasp":
            return {"reason": "INJECTED_JOIN_BLOCKED", "stage": "pregrasp"}
        if failure is None and mode == "partial_approach" and kwargs["stage"] == "extraction":
            return {"reason": "INJECTED_AFTER_APPROACH_STOP", "stage": "extraction"}
        return failure
    monkeypatch.setattr(case.c, "_path_failure", path_check)
    captured = []
    def stop_before_suffix(**kwargs):
        captured.append(kwargs)
        return None, {"reason": "TEST_STOPS_BEFORE_LOADED_SUFFIX", "stage": "transit"}, kwargs["trace"]
    monkeypatch.setattr(case.c, "_finish_place_branch", stop_before_suffix)
    with case.c._contact_context():
        segment, failure, trace = adaptation.adapt_branch(case.c, hint=hint,
            target=case.target, face=hint["segment"]["face"],
            requested_virtual_contact=case.c.virtual_from_physical(physical),
            grasp_q=np.asarray(hint["segment"]["path"][hint["segment"]["grasp_index"]]),
            home_q=np.asarray(case.scene.policy.layout_validation.initial_q),
            all_obstacles=obstacles, receiver=case.scene.receiver,
            support_names=tuple(case.scene.support_graph.supported_by[case.target.name]),
            suction=case.scene.policy.data["suction"], seed=19)
    assert segment is None and failure and not case.c._completed_tasks
    if mode == "changed_mask":
        assert captured and sum(trace["history"]["new_commanded_mask"]) > 0
        assert trace["history"]["old_commanded_mask"] != trace["history"]["new_commanded_mask"]
        np.testing.assert_allclose(captured[0]["attachment"].box_at(captured[0]["contact_q"]).world_from_local,
                                   case.target.world_from_local, atol=1e-10, rtol=0)
        assert "pregrasp" in seen and "contact" in seen and "extraction" in seen
    elif mode == "blocked_join":
        assert failure["reason"] == "INJECTED_JOIN_BLOCKED" and not captured
    elif mode == "partial_approach":
        assert failure["reason"] == "INJECTED_AFTER_APPROACH_STOP" and "contact" in seen and not captured
    else:
        assert "COLLISION" in failure["reason"] and not captured


def test_current_cup_coverage_rejects_contact_off_target(actual_case):
    case = actual_case
    old = case.old["selected_trajectory_segment"]
    q = np.asarray(old["path"][old["grasp_index"]])
    physical = case.c.physical_from_virtual(case.c.robot.fk(q))
    # Move the carton tangentially outside the entire cup array; no IK or rule relaxation.
    moved = replace(case.target, center=case.target.center + physical[:3, 0] * 2.)
    with case.c._contact_context(), pytest.raises(ValueError):
        case.c._contact_selection(q, moved, old["face"], case.scene.policy.data["suction"])


def test_history_timeout_preserves_parent_and_contact_context(actual_case, clock, monkeypatch, tmp_path):
    case = actual_case
    write_motion(tmp_path / "old.json", case.old)
    monkeypatch.setattr(history, "perf_counter", clock)
    source = history.HistorySource(history.HistoryPolicy(source=str(tmp_path)))
    case.c.start_planning_request(clock())
    parent = case.c._deadline_monotonic
    hard = case.c._request_deadline_monotonic
    validator = case.c.robot_state_validator
    validator.commanded_cup_mask = [True, False]*36
    validator.contact_target_name = "current-task"
    validator.stack_carton_names = {"current-neighbor"}
    saved = deepcopy((validator.commanded_cup_mask, validator.contact_target_name, validator.stack_carton_names))
    calls = []
    def timeout(c, **kwargs):
        calls.append((c._deadline_monotonic, c._candidate_final_deadline, c._branch_final_deadline))
        validator.commanded_cup_mask = [False]*72
        clock.now = c._deadline_monotonic
        return None, {"reason": "INJECTED_FINAL_RECHECK_TIMEOUT", "stage": "final_validation"}, {}
    monkeypatch.setattr(adaptation, "adapt_branch", timeout)
    attempts, selected = history.evaluate_history(source, case.scene, case.c, case.target,
        case.backend, deadline=110., attempt_limit=4)
    assert selected is None and len(attempts) == len(calls) == 1
    assert calls == [(110., 110., 110.)]
    assert case.c._deadline_monotonic == parent and case.c._request_deadline_monotonic == hard
    assert clock() < parent and not case.c._completed_tasks
    assert saved == (validator.commanded_cup_mask, validator.contact_target_name, validator.stack_carton_names)


def test_runner_consumes_once_configured_source_without_manual_motion(actual_case, monkeypatch, tmp_path):
    from tools import run_m710_contact_unloading as runner
    original_policy = audit.load_layout_motion_policy("configs/validation/m710id70_layout_v1_single_carton.yaml")
    data = deepcopy(original_policy.data)
    source = tmp_path / "history"
    source.mkdir()
    data["search_strategy"]["history"]["source"] = str(source)
    configured = replace(original_policy, data=data)
    monkeypatch.setattr(runner, "load_layout_motion_policy", lambda _: configured)
    calls = []
    def plan(policy, **kwargs):
        calls.append((policy, kwargs))
        return {"complete_trajectory_status": "FAIL_CLOSED", "statistics": {},
                "planning_performance": {"planning_total_wall_seconds": .01}}
    monkeypatch.setattr(runner, "run_layout_single_carton_audit", plan)
    actual_path = tmp_path / "actual.json"
    actual_path.write_text(json.dumps(actual_case.state))
    assert runner.main(["--actual-state", str(actual_path), "--output", str(tmp_path / "out")]) == 2
    assert len(calls) == 1
    assert calls[0][0].data["search_strategy"]["history"]["source"] == str(source)
    assert calls[0][1]["motion_input"].snapshot["actual_state_context"]["world_session_id"] == actual_case.state["world_session_id"]
