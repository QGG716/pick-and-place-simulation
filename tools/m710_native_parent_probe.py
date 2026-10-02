"""Finite current-input parent/ownership probes; no Isaac or historical input.

Every motion below is submitted to the real worker and the ordinary independent
authority. Synthetic target placement is a separately declared development scene.
"""
from copy import deepcopy
from dataclasses import replace
import os

import numpy as np

from unloading_sim.geometry import OBB
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.moveit2_backend import MoveItUnavailable, ResidentMoveItClient, model_request
from unloading_sim.moveit2_native_cold import audit_native_motion_coverage
from unloading_sim.stage_motion_policy import MotionPurpose
from unloading_sim.support import SupportRelationGraph
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.workcell_layout import canonical_digest


def _session(c, label):
    return dict(label=label, task_id=c.native_task_id, native_stages=deepcopy(c.native_evidence),
        native_ik=deepcopy(c.native_ik_evidence), authority_paths=deepcopy(c.authority_path_evidence),
        cold_audit=deepcopy(c.native_cold_evidence()))


def _receipt(q):
    return deepcopy(getattr(q, "native_receipt", None))


def _require(add, name, passed, **details):
    add(name, passed, **details)
    if not passed:
        raise RuntimeError("NATIVE_PARENT_PROBE_FAILED:" + name)


def _rpc(c, request):
    _, timeout = c._native_limits()
    return c.native.request(request, timeout=timeout, cancelled=c._native_cancelled)


def _inspect(c, request):
    return _rpc(c, {**deepcopy(request), "op": "inspect", "inspect_pairs": []})


def run_parent_chain_probe(c, scene, q, policy, add, report):
    """Use selected receipts through normal pose connection and native LIN."""
    report["probe_suite"] = "explicit_parent_and_ownership"
    report["task_sessions"] = []
    root = c.native_root_state(q, scene.all_obstacles, stage="pregrasp")
    selected = path = connection = first = None
    trials = []
    # These finite goals are derived from this invocation's configured home.
    # A missing natural numeric difference is a failed coverage claim, never
    # permission to perturb an emitted sample or import a prior successful q.
    for index, delta in enumerate((.01, .015, -.01)):
        proposed = np.asarray(q).copy()
        proposed[0] += delta
        pose = c.robot.fk(proposed)
        selected, path, failure, connection = c._connect_pose(pose, [root], root,
            scene.all_obstacles, stage="pregrasp", purpose=MotionPurpose.FREE_APPROACH,
            ik_seed=71200 + index, connection_seed=71210 + index, allow_rrt=False)
        returned = connection.get("returned_endpoint", {})
        trials.append(dict(delta_j1_rad=delta, requested_pose=pose.tolist(), failure=failure,
            connection=deepcopy(connection)))
        if failure is None and returned.get("differs_from_ik_goal") is True:
            first = deepcopy(c.native_verified[-1])
            break
    valid = (first is not None and np.array_equal(selected, path[-1])
        and first.get("native_solver_calls", {}).get("PTP") == 1
        and first.get("authoritative_status") == "PASS" and _receipt(selected) is not None)
    _require(add, "ordinary_connect_pose_natural_native_endpoint", valid, trials=trials,
        selected_q=None if selected is None else selected.tolist(), selected_receipt=None if selected is None else _receipt(selected),
        scope="real current native IK/PTP; no fabricated endpoint perturbation")
    first_path = list(path)
    selected_a = selected.copy()
    selected_b, second_path, failure, second = c._connect_pose(pose, [root], root,
        scene.all_obstacles, stage="pregrasp", purpose=MotionPurpose.FREE_APPROACH,
        ik_seed=71200 + index, connection_seed=71210 + index, allow_rrt=False)
    second_record = deepcopy(c.native_verified[-1]) if failure is None else {}
    _require(add, "distinct_native_parents_with_same_numeric_endpoint",
        failure is None and first["stage_id"] != second_record.get("stage_id")
        and np.array_equal(selected_a, selected_b), failure=failure, connection=second,
        selected_a_receipt=_receipt(selected_a), selected_b_receipt=_receipt(selected_b),
        stage_a=first["stage_id"], stage_b=second_record.get("stage_id"),
        scope="two fresh worker solutions in one Task; identical q cannot select their parent")
    destination = c.robot.fk(selected_a)
    destination[2, 3] += .001
    suffix, failure, linear = c._cartesian(selected_a, destination, scene.all_obstacles,
        purpose=MotionPurpose.FREE_APPROACH, seed=71230, stage="pregrasp")
    last = deepcopy(c.native_verified[-1]) if failure is None else {}
    submitted = last.get("submitted_request", {})
    full = first_path + suffix[1:] if suffix else first_path
    coverage = audit_native_motion_coverage(full, [first, last], task_id=c.native_task_id) if last else {}
    _require(add, "selected_earlier_parent_native_lin_backtrack",
        failure is None and submitted.get("parent_stage_id") == first["stage_id"]
        and last.get("parent_stage_id") == first["stage_id"]
        and last.get("parent_stage_id") != second_record.get("stage_id")
        and submitted.get("q_start") == first["points"][-1]["q"]
        and last.get("native_solver_calls", {}).get("LIN") == 1
        and last.get("task_backtracks", 0) > 0 and coverage.get("passed") is True
        and last.get("authoritative_status") == "PASS",
        failure=failure, connection=linear, source_coverage=coverage,
        expected_parent=first["stage_id"], most_recent_same_q_parent=second_record.get("stage_id"),
        parent_state_receipt=submitted.get("parent_state_receipt"),
        scope="ordinary selected endpoint; real native append restores explicitly selected earlier prefix")
    report["task_sessions"].append(_session(c, "ordinary_free_parent_chain"))

    _ownership_probe(c, scene, q, policy, add, report, first["submitted_request"])


def run_ownership_only_probe(c, scene, q, policy, add, report):
    """Recheck ownership independently without reading earlier probe output."""
    report["probe_suite"] = "sparse_synthetic_ownership_only"
    report["task_sessions"] = []
    goal = np.asarray(q).copy()
    goal[0] += .01
    free_request = c._build_native_request(q, goal, scene.all_obstacles,
        seed=71210, stage="pregrasp")
    free_request.update(parent_stage_id="", stage_id=c.native_task_id + ":free-root",
        pipeline_id="pilz_industrial_motion_planner", planner_id="PTP",
        purpose=MotionPurpose.FREE_APPROACH.value)
    report["identity_counterexample_input"] = dict(
        source="current configured home plus 0.01 rad J1; geometric request only",
        prior_probe_inputs_read=0, source_carton_count=len(scene.cartons),
        generated_native_solution_required=True)
    _ownership_probe(c, scene, q, policy, add, report, free_request)


def _ownership_probe(c, scene, q, policy, add, report, free_request):
    target = next(box for box in scene.cartons if box.name == "carton_l07_c02")
    physical = c.physical_from_virtual(c.robot.fk(q))
    rotation = physical[:3, [2, 0, 1]]
    center = physical[:3, 3] + physical[:3, 2] * target.half_extents[0]
    synthetic = OBB(center, target.half_extents, rotation, target.name, target.category)
    # Ownership is a protocol diagnostic, not a 40-carton feasibility claim.
    # Preserve static colliders and policy; explicitly declare the sparse scene.
    cartons = (synthetic,)
    excluded_cartons = sorted(item.name for item in scene.cartons if item.name != target.name)
    snapshot = deepcopy(scene.snapshot)
    record = next(item for item in snapshot["cartons"] if item["name"] == target.name)
    record.update(pose_world=synthetic.world_from_local.tolist(), center_m=synthetic.center.tolist(),
        half_extents_m=synthetic.half_extents.tolist())
    snapshot["cartons"] = [record]
    snapshot["development_scope"] = dict(kind="SPARSE_SINGLE_CARTON_OWNERSHIP_PROTOCOL",
        source_scene_fingerprint=scene.snapshot["scene_fingerprint"],
        source_carton_count=len(scene.cartons), carton_count=1,
        excluded_carton_ids=excluded_cartons, remaining_stack_names=[target.name],
        formal_layout_acceptance=False, physical_execution=False)
    snapshot["initial_state_audit"] = dict(status="DEVELOPMENT_ONLY_NOT_FORMAL_INITIAL_STATE",
        source="this invocation's configured-home FK synthetic target")
    snapshot.pop("scene_fingerprint", None)
    snapshot["scene_fingerprint"] = canonical_digest(snapshot)
    support_graph = SupportRelationGraph.build(cartons)
    declared = replace(scene, cartons=cartons, snapshot=snapshot,
        support_graph=support_graph, removable_cartons=tuple(support_graph.removable_cartons()),
        remaining_stack_names=(target.name,),
        snapshot_verification=dict(status="DEVELOPMENT_ONLY", scene_fingerprint=snapshot["scene_fingerprint"]),
        snapshot_consistency=dict(status="DEVELOPMENT_ONLY_NOT_FORMAL_LAYOUT_ACCEPTANCE"))
    report["ownership_scene"] = dict(scope="SPARSE_SYNTHETIC_CURRENT_GEOMETRY_DEVELOPMENT_ONLY",
        native_model_identity=deepcopy(c.native_identity),
        native_model_identity_scope="unchanged official model/tool/policy initialized from original development scene",
        source_scene_fingerprint=scene.snapshot["scene_fingerprint"],
        scene_fingerprint=snapshot["scene_fingerprint"], changed_object=target.name,
        target_pose_world=synthetic.world_from_local.tolist(), target_size_m=(2 * target.half_extents).tolist(),
        source_carton_count=len(scene.cartons), carton_count=len(cartons),
        excluded_carton_ids=excluded_cartons, remaining_stack_names=[target.name],
        declared_carton_mass_kg=42.5, mass_dynamics_evaluated=False,
        fixed_components_unchanged=True, collision_policy_unchanged=True,
        formal_40_carton_acceptance=False, configured_q_unchanged=True,
        physical_observation=False, input_to_formal_request=False)
    c.start_planning_request()
    c.native_scene = declared
    c.stack_carton_names = {target.name}
    c.robot_state_validator.stack_carton_names = frozenset(c.stack_carton_names)
    selection = c._contact_selection(q, synthetic, "front", policy.data["suction"])
    root = c.native_root_state(q, declared.all_obstacles, stage="contact", target_contact=synthetic)
    destination = c.robot.fk(root)
    destination[:3, 3] += physical[:3, 2] * .00005
    contact, failure, evidence = c._cartesian(root, destination, declared.all_obstacles,
        purpose=MotionPurpose.CONTACT_PROCESS, seed=71240, stage="contact", target_contact=synthetic)
    _require(add, "ownership_current_synthetic_contact", failure is None, failure=failure,
        evidence=evidence, selected_cups=sum(selection["commanded_active_mask"]))
    contact_record = deepcopy(c.native_verified[-1])
    before = _inspect(c, contact_record["submitted_request"])
    actual_contact = contact[-1]
    attachment = PhysicalContactAttachment(c.robot,
        RigidAttachment.capture(c.physical_from_virtual(c.robot.fk(actual_contact)), synthetic),
        c.flange_from_virtual_task_tcp, c.flange_from_physical_contact)
    obstacles = tuple(item for item in declared.all_obstacles if item.name != target.name)
    destination = c.robot.fk(actual_contact)
    destination[:3, 3] -= physical[:3, 2] * .001
    attached, failure, evidence = c._cartesian(actual_contact, destination, obstacles,
        purpose=MotionPurpose.EXTRACTION_PROCESS, seed=71241, stage="extraction", attachment=attachment)
    _require(add, "ownership_native_world_to_attached", failure is None, failure=failure, evidence=evidence)
    attached_record = deepcopy(c.native_verified[-1])
    held = _inspect(c, attached_record["submitted_request"])
    # This is a semantic place endpoint in a synthetic protocol test, with no
    # receiving-support or physical-reception claim and no manufactured motion.
    placed, failure, place_evidence = c._cartesian(attached[-1], c.robot.fk(attached[-1]), obstacles,
        purpose=MotionPurpose.PLACEMENT_PROCESS, seed=71242, stage="place", attachment=attachment)
    _require(add, "ownership_zero_motion_place_semantics", failure is None and len(placed) == 1,
        failure=failure, evidence=place_evidence, physical_reception_claim=False)
    released = attachment.box_at(placed[-1])
    world = tuple(released if item.name == target.name else item for item in declared.all_obstacles)
    c._native_contact_context["target"] = released
    destination = c.robot.fk(placed[-1])
    withdrawal, failure, evidence = c._cartesian(placed[-1], destination, world,
        purpose=MotionPurpose.DEPARTURE_PROCESS, seed=71243, stage="withdrawal", target_contact=released)
    _require(add, "ownership_zero_motion_release_selected", failure is None and len(withdrawal) == 1,
        failure=failure, evidence=evidence, selected_state_receipt=_receipt(withdrawal[-1]) if withdrawal else None)
    terminal = c._native_terminal_state_request(withdrawal[-1])
    records = c._native_selected_records(withdrawal[-1])
    task_request = dict(op="task_audit", identity=c.native_identity, task_id=c.native_task_id,
        stage_ids=[row["stage_id"] for row in records], target_id=target.name,
        terminal_state_request=terminal)
    task = _rpc(c, task_request)
    transition = task.get("terminal_transition") or {}
    _require(add, "selected_zero_terminal_native_task_audit",
        task.get("status") == "SUCCESS" and task.get("complete_task") is True
        and task.get("attach_transitions") == 1 and task.get("release_transitions") == 1
        and transition.get("event") == "RELEASE" and transition.get("zero_motion") is True
        and type(transition.get("native_solver_calls")) is int and transition["native_solver_calls"] == 0,
        request=task_request, result=task,
        scope="synthetic native scene transition audit; no physical completion or receiving-support claim")
    after = _inspect(c, terminal)
    attach_pose_error = float(np.max(np.abs(np.asarray(before["target_world_pose"]) - np.asarray(held["target_world_pose"]))))
    release_pose_error = float(np.max(np.abs(np.asarray(after["target_world_pose"]) - released.world_from_local)))
    same_shape = all(request["process_policy"]["target"]["size"] == (2 * target.half_extents).tolist()
                     for request in (contact_record["submitted_request"], attached_record["submitted_request"], terminal))
    _require(add, "native_ownership_readback_and_independent_checks",
        before.get("target_in_world") is True and before.get("target_attached") is False
        and held.get("target_in_world") is False and held.get("target_attached") is True
        and after.get("target_in_world") is True and after.get("target_attached") is False
        and before.get("target_id") == held.get("target_id") == after.get("target_id") == target.name
        and [before.get("world_count"), held.get("world_count"), after.get("world_count")]
            == [len(declared.all_obstacles), len(declared.all_obstacles) - 1, len(declared.all_obstacles)]
        and attach_pose_error <= 1e-7 and release_pose_error <= 1e-7
        and same_shape and all(row.get("authoritative_status") == "PASS" for row in (contact_record, attached_record))
        and attached_record.get("parent_stage_id") == contact_record["stage_id"]
        and terminal.get("parent_stage_id") == attached_record["stage_id"],
        readbacks=dict(world=before, attached=held, released_world=after),
        attachment_pose_continuity_max_abs=attach_pose_error,
        release_pose_continuity_max_abs=release_pose_error,
        scope="real native Task scene ownership transitions; independent full-edge authority; no physical attachment")
    report["task_sessions"].append(_session(c, "synthetic_native_ownership_chain"))
    # These are protocol identity counterexamples, not formal cycle trials.
    # Production ERROR handling intentionally closes the resident stream.
    altered = deepcopy(task_request)
    original_q = altered["terminal_state_request"]["q_start"]
    original_q[0] = float(np.nextafter(original_q[0], np.inf))
    _rejected(c.native, altered, "NATIVE_TASK_TERMINAL_STATE_CHANGED", add,
        "changed_zero_terminal_state_rejected", c.native_request_timeout)
    init, _, _ = model_request(c, scene, asset_root=os.environ.get("M710_MOVEIT_ASSET_ROOT"), seed=71070)
    # A separate fresh worker generates one short root from this invocation's
    # geometric request. No trajectory or native subsolution crosses processes.
    client = ResidentMoveItClient(os.environ.get("M710_MOVEIT_COMMAND"))
    try:
        client.request(init)
        current_root = deepcopy(free_request)
        current_root.pop("parent_state_receipt", None)
        current_root["task_id"] += ":target-identity-probe"
        current_root["stage_id"] = current_root["task_id"] + ":1"
        current_root["parent_stage_id"] = ""
        fresh = client.request(current_root, timeout=c.native_request_timeout)
        _require(add, "target_identity_fresh_native_root", fresh.get("status") == "SUCCESS"
            and fresh.get("native_solver_calls", {}).get("PTP") == 1,
            request=current_root, response=fresh,
            scope="fresh worker root for identity rejection only; no accepted path imported")
        altered = dict(op="task_audit", identity=c.native_identity, task_id=current_root["task_id"],
            target_id=target.name + "_wrong", stage_ids=[current_root["stage_id"]])
        _rejected(client, altered, "NATIVE_TASK_TARGET_ID_CHANGED", add,
            "target_identity_change", c.native_request_timeout)
    finally:
        client.close()
    missing = deepcopy(free_request)
    missing.pop("parent_stage_id", None)
    missing["stage_id"] += ":missing-parent-probe"
    attached_root = deepcopy(attached_record["submitted_request"])
    attached_root["parent_stage_id"] = ""
    attached_root["stage_id"] += ":attached-root-probe"
    for name, request, expected in (
        ("explicit_parent_required", missing, "NATIVE_TASK_EXPLICIT_PARENT_REQUIRED"),
        ("attached_root_forbidden", attached_root, "NATIVE_COLD_INITIAL_ATTACHMENT_FORBIDDEN")):
        client = ResidentMoveItClient(os.environ.get("M710_MOVEIT_COMMAND"))
        try:
            startup = client.request(init)
            _require(add, name + "_worker_initialized", startup.get("status") == "READY",
                scope="isolated negative-case worker; no native motion requested by init")
            _rejected(client, request, expected, add, name, c.native_request_timeout)
        finally:
            client.close()


def _rejected(client, request, expected, add, name, timeout):
    try:
        response = client.request(request, timeout=timeout)
    except MoveItUnavailable as exc:
        _require(add, name, str(exc) == expected and client.process.poll() is not None,
            request=request, observed_rejection=str(exc), worker_exit_code=client.process.poll(),
            expected_guard=expected, solver_calls_in_error_response=None,
            scope="real compiled guard rejection; production transport poisons stream; no solver counter fabricated")
    else:
        _require(add, name, False, request=request, response=response, expected_guard=expected)
