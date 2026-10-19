"""Planning attachment and one normal extraction after the single-contact prefix."""
from copy import deepcopy
from time import perf_counter
import numpy as np

from .single_contact import SingleContactConnector, check_contact_samples
from .layout_trajectory import PhysicalContactAttachment
from .validation_physics import RigidAttachment
from .depalletizing import minimum_clearance_extraction_distance
from .pair_clearance import obb_surface_distance
from .moveit2_backend import MoveItUnavailable
from .stage_motion_policy import MotionPurpose

MARKER = "APPROACH_CONTACT_EXTRACTION_PLANNED_NOT_EXECUTABLE"


def extraction_geometry(connector, target, attachment, start, obstacles, support_names):
    """One bounded full-box calculation; no IK, route alternatives or planning."""
    payload = attachment.box_at(start)
    names = {b.name for b in obstacles}
    if target.target_id in names or not set(support_names) <= names:
        raise MoveItUnavailable("EXTRACTION_WORLD_OR_SUPPORT_IDENTITY_INVALID")
    required = max(connector.collision_policy.pair_clearance("external", connector.collision_margin_m)
        + connector.contact_tolerance_m, connector.collision_policy.free_space_clearance_m)
    required += connector.budget.extraction_runtime_clearance_reserve_m
    # Same existing boundary guard as the complete connector, not a relaxed threshold.
    guard = (2 * float(connector.ik["position_tolerance_m"])
        + 2 * float(np.linalg.norm(payload.half_extents)) * float(connector.ik["orientation_tolerance_rad"])
        + connector.contact_tolerance_m)
    distance = minimum_clearance_extraction_distance(payload, target.outward_normal, obstacles,
        free_space_clearance_m=required + guard,
        scan_step_m=connector.budget.extraction_scan_step_m,
        maximum_distance_m=connector.budget.maximum_extraction_m)
    evidence = dict(direction_world=target.outward_normal.tolist(), distance_m=distance,
        required_terminal_clearance_m=required, fk_boundary_guard_m=guard,
        maximum_distance_m=connector.budget.maximum_extraction_m,
        scan_step_m=connector.budget.extraction_scan_step_m,
        constraint_ids=[b.name for b in obstacles], support_names=list(support_names),
        method="minimum_clearance_extraction_distance_full_box_all_remaining_world")
    if distance is None or not np.isfinite(distance) or distance <= 0:
        return None, evidence, dict(reason="NO_POSITIVE_GEOMETRIC_EXTRACTION_WITHIN_BOUND",
            stage="extraction", scope="FACE_ATTEMPT")
    physical = connector.physical_contact_pose(start)
    goal = physical.copy()
    goal[:3, 3] += target.outward_normal * distance
    evidence.update(start_physical_pose=physical.tolist(), goal_physical_pose=goal.tolist())
    return goal, evidence, None


def extraction_sample_check(connector, path, attachment, target, geometry, tracker, obstacles, supports, *, terminal=True):
    """Existing returned samples only; preserve branch-local ordered payload history."""
    from types import SimpleNamespace
    actual_start = np.asarray(geometry["start_physical_pose"])
    goal = np.asarray(geometry["goal_physical_pose"])
    line = SimpleNamespace(approach_pose=actual_start, contact_pose=goal,
        outward_normal=-np.asarray(geometry["direction_world"]), standoff_m=geometry["distance_m"])
    fk = check_contact_samples(path, connector.physical_contact_pose, line,
        position_tolerance=float(connector.ik["position_tolerance_m"]),
        orientation_tolerance=float(connector.ik["orientation_tolerance_rad"]))
    poses = []
    failure = fk["failure"]
    for index, q in enumerate(path):
        connector.check_context(target, getattr(connector, "active_obstacles", connector.native_scene.all_obstacles))
        box = attachment.box_at(q)
        poses.append(box.world_from_local.tolist())
        if not np.isfinite(box.world_from_local).all():
            raise MoveItUnavailable("EXTRACTION_PAYLOAD_POSE_UNKNOWN")
        rejected = tracker.state_failure(box, obstacles, supports)
        if rejected and failure is None:
            failure = dict(reason="EXTRACTION_RETURNED_SAMPLE_PAYLOAD_REJECTED",
                sample=index, detail=rejected, scope="FACE_ATTEMPT")
    end = attachment.box_at(path[-1])
    distances = [(b.name, obb_surface_distance(end, b)) for b in obstacles]
    if any(not np.isfinite(d) for _, d in distances):
        raise MoveItUnavailable("EXTRACTION_END_DISTANCE_UNKNOWN")
    short = [(name, d) for name, d in distances
        if d + 1e-9 < geometry["required_terminal_clearance_m"]]
    if terminal and (not tracker.fully_released or short):
        failure = failure or dict(reason="ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED",
            scope="FACE_ATTEMPT", short_pairs=short, fully_released=tracker.fully_released)
    return dict(passed=failure is None, failure=failure, fk=fk, payload_poses_world=poses,
        checked_samples=len(path), additional_samples=0,
        ordered_history_scope="RETURNED_SAMPLES_ONLY_NOT_DENSE_OR_CONTINUOUS_PROOF",
        terminal_tracker=tracker.evidence(), short_pairs=short,
        minimum_payload_world_gap_m=min((d for _, d in distances), default=None),
        final_separation_required=terminal)


class SingleExtractionConnector(SingleContactConnector):
    def plan_extraction(self, start, target, attachment, payload_world, supports, tracker, seed, row):
        before = perf_counter()
        goal, geometry, failure = extraction_geometry(self, target, attachment, start, payload_world, supports)
        row["times"]["extraction_target_s"] = perf_counter() - before
        if failure:
            return [], geometry, failure
        before = perf_counter()
        path, failure, _ = self._native_plan(start, None, payload_world, seed=seed,
            stage="extraction", purpose=MotionPurpose.EXTRACTION_PROCESS,
            goal_pose=self.virtual_from_physical(goal), attachment=attachment,
            support_names=supports, initial_proximity=tracker,
            allow_ompl=False, max_ompl_attempts=0, try_ptp=False, try_prior=False)
        row["times"]["extraction_s"] = perf_counter() - before
        return path, geometry, failure

    def check_attached_end(self, end, target, payload_world, attachment, seed, *, stage):
        if stage not in {"extraction","transit"}:
            raise MoveItUnavailable("ATTACHED_ENDPOINT_STAGE_INVALID")
        prefix = stage.upper()+"_END"
        request = self._build_native_request(end, end, payload_world, seed=seed,
            stage=stage, attachment=attachment, support_names=(), initial_proximity=None)
        receipt, _, transition = self._native_parent_for_request(end, request)
        if transition is not None:
            raise MoveItUnavailable(prefix+"_OWNERSHIP_CHANGED")
        request.update(op="check_"+stage+"_end", parent_stage_id=receipt.stage_id,
            parent_state_receipt=receipt.evidence(), ownership_transition=None)
        if stage == "transit":
            request["purpose"] = MotionPurpose.FREE_LOADED_TRANSFER.value
        _, timeout = self._native_limits()
        checked = self.native.request(request, timeout=timeout, cancelled=self._native_cancelled)
        if checked["status"] != "SUCCESS":
            return checked
        if (checked.get("task_id") != self.native_task_id
                or checked.get("parent_stage_id") != receipt.stage_id
                or checked.get("target_id") != target.target_id
                or checked.get("target_attached") is not True or checked.get("target_in_world") is not False
                or checked.get("free_space_rules_restored") is not True or checked.get("checked_states") != 1):
            raise MoveItUnavailable(prefix+"_EVIDENCE_IDENTITY_CHANGED")
        native_pose = np.asarray(checked.get("target_world_pose"), dtype=float)
        if (native_pose.shape != (4,4) or not np.isfinite(native_pose).all()
                or not np.allclose(native_pose, attachment.box_at(end).world_from_local, atol=1e-7, rtol=0)):
            raise MoveItUnavailable(prefix+"_PAYLOAD_POSE_CHANGED")
        return checked

    def complete_extraction_plan(self, plan, row, end, target, obstacles, attachment, seed):
        """Called inside the active attachment/mask context; default still stops here."""
        return plan, row

    def complete_contact_plan(self, plan, row, contact_end, target, obstacles, suction, seed):
        # This hook is reached only after the unchanged two-segment checks pass.
        row["contact_complete"] = True
        row["complete"] = False
        row["contact_end_q_rad"] = row.pop("actual_end_q_rad")
        for key in ("attachment_context_s", "extraction_target_s", "extraction_s",
                    "extraction_sample_check_s", "terminal_separation_s"):
            row["times"][key] = None
        began = perf_counter()
        with self._contact_context():
            return self._attached_suffix(plan, row, contact_end, target, obstacles, seed, began)

    def _reject_suffix(self, row, failure):
        reason = failure.get("reason", "UNKNOWN_EXTRACTION_FAILURE")
        if any(word in reason for word in ("CANCEL", "DEADLINE", "CONTEXT", "IDENTITY", "UNKNOWN", "NATIVE_TASK_")):
            raise MoveItUnavailable("EXTRACTION_REQUEST_STOP:" + reason)
        allowed = {"NO_POSITIVE_GEOMETRIC_EXTRACTION_WITHIN_BOUND", "NATIVE_PLANNING_FAILED",
            "NATIVE_PLANNING_TIMEOUT", "SEARCH_EXHAUSTED", "MOVEIT2_SEARCH_EXHAUSTED",
            "INVALID_GOAL_CLEARANCE", "INVALID_GOAL_PROCESS", "CONTACT_NORMAL_LINE_OR_ORIENTATION",
            "CONTACT_ENDPOINT_FK", "EXTRACTION_RETURNED_SAMPLE_PAYLOAD_REJECTED",
            "ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED", "EXTRACTION_END_CLEARANCE_REJECTED",
            "EXTRACTION_END_PROCESS_REJECTED", "PAYLOAD_INITIAL_PENETRATION"}
        if reason not in allowed:
            raise MoveItUnavailable("EXTRACTION_REQUEST_FAILURE:" + reason)
        row["failure"] = {**failure, "scope": "FACE_ATTEMPT", "stage": "extraction"}
        row["times"]["total_s"] = perf_counter() - self._single_face_started
        return None, row

    def _attached_suffix(self, plan, row, start, target, obstacles, seed, began):
        self.check_context(target, obstacles)
        terminal = row.get("terminal_contact", {})
        expected = row.get("expected_contact", {})
        actual = terminal.get("actual_contact_mask")
        if (not row.get("contact_fk_check", {}).get("passed")
                or not isinstance(actual, list) or len(actual) != 72 or not any(actual)
                or not all(isinstance(bit, bool) for bit in actual)
                or len(terminal.get("geometrically_eligible_mask", [])) != 72
                or any(bit and terminal["geometrically_eligible_mask"][i] is not True for i,bit in enumerate(actual))
                or actual != terminal.get("commanded_active_mask")
                or actual != expected.get("commanded_active_mask")
                or terminal.get("measured") is not False or terminal.get("attached") is not False):
            raise MoveItUnavailable("ATTACH_REQUIRES_CURRENT_TERMINAL_GEOMETRY")
        if (not np.array_equal(start, plan["contact"][-1])
                or start.native_receipt != plan["final_state_receipt"]
                or self._native_state_receipt(start).stage_id != plan["stages"][-1]["stage_id"]):
            raise MoveItUnavailable("ATTACH_CONTACT_PARENT_IDENTITY_CHANGED")
        # Captured at the actual generated q, never the nominal target or another IK.
        physical = self.physical_contact_pose(start)
        rigid = RigidAttachment.capture(physical, target.box)
        attachment = PhysicalContactAttachment(self.robot, rigid,
            self.flange_from_virtual_task_tcp, self.flange_from_physical_contact)
        error = float(np.max(np.abs(attachment.box_at(start).world_from_local - target.box.world_from_local)))
        if error > 1e-10:
            raise MoveItUnavailable("ATTACH_WORLD_POSE_CHANGED")
        self._native_contact_context["selection"] = {
            key: deepcopy(terminal[key]) for key in
            ("actual_contact_mask", "commanded_active_mask", "geometrically_eligible_mask")}
        payload_world = [b for b in obstacles if b.name != target.target_id]
        supports = sorted(self.native_scene.support_graph.supported_by[target.target_id])
        tracker, failure = self._initial_proximity(target.box, payload_world, supports)
        row["attachment"] = dict(status="PREPARED_PLANNING_CONTEXT", target_id=target.target_id,
            contact_q_rad=np.asarray(start).tolist(), parent_receipt=start.native_receipt,
            actual_physical_contact_pose=physical.tolist(), tcp_from_box=rigid.tcp_from_box.tolist(),
            flange_from_box=(np.linalg.inv(self.robot.named_link_frames(start)["flange"])
                @ target.box.world_from_local).tolist(),
            before_box_pose_world=target.box.world_from_local.tolist(),
            after_box_pose_world=attachment.box_at(start).world_from_local.tolist(),
            pose_continuity_max_abs=error, selection=deepcopy(self._native_contact_context["selection"]),
            physical_execution="NOT_RUN", measured=False)
        row["times"]["attachment_context_s"] = perf_counter() - began
        if failure:
            return self._reject_suffix(row, failure)
        return self._plan_extraction_suffix(plan, row, start, target, obstacles,
            attachment, payload_world, supports, tracker, seed)

    def _plan_extraction_suffix(self, plan, row, start, target, obstacles,
                                attachment, payload_world, supports, tracker, seed):
        """Default one-target suffix; regional search overrides only this hook."""
        original_tracker = tracker.clone()
        path, geometry, failure = self.plan_extraction(start, target, attachment, payload_world,
            supports, tracker, seed, row)
        row["extraction_geometry"] = geometry
        if failure:
            return self._reject_suffix(row, failure)
        return self._complete_extraction_candidate(plan, row, start, path, geometry,
            target, obstacles, attachment, payload_world, supports, original_tracker, seed)

    def _complete_extraction_candidate(self, plan, row, start, path, geometry,
            target, obstacles, attachment, payload_world, supports, tracker, seed, *, guard=None, checked_prefix=None):
        """Evaluate and assemble this exact candidate once; no acceptance cache.

        Regional callers provide a fresh branch plan/row and initial tracker.
        The default suffix retains its existing checks and completion hook.
        """
        if not np.array_equal(path[0], start):
            raise MoveItUnavailable("EXTRACTION_PARENT_STATE_CHANGED")
        records = self._native_selected_records(path[-1])
        extra = 1 if checked_prefix is None else 2
        if (len(records) != len(plan["stages"])+extra
                or [r["stage_id"] for r in records[:-extra]] != [r["stage_id"] for r in plan["stages"]]
                or records[-extra]["parent_stage_id"] != plan["stages"][-1]["stage_id"]):
            raise MoveItUnavailable("EXTRACTION_PARENT_CHAIN_INVALID")
        if checked_prefix is not None:
            self._validate_checked_turn(checked_prefix, start, path, records, attachment, tracker)
        row["attachment"]["status"] = "NATIVE_OWNERSHIP_TRANSITION_ACCEPTED"
        before = perf_counter()
        final_path = path if checked_prefix is None else checked_prefix["second_path"]
        samples = extraction_sample_check(self, final_path, attachment, target, geometry,
            tracker, payload_world, supports)
        row["extraction_samples"] = samples
        row["times"]["extraction_sample_check_s"] = perf_counter() - before
        if guard is not None:
            guard()
            if (type(samples.get("passed")) is not bool
                    or samples["passed"] != (samples.get("failure") is None)):
                raise MoveItUnavailable("EXTRACTION_SAMPLE_RESULT_UNKNOWN")
        if not samples["passed"]:
            return self._reject_suffix(row, samples["failure"])
        self.check_context(target, obstacles)
        before = perf_counter()
        checked = self.check_attached_end(path[-1], target, payload_world, attachment, seed, stage="extraction")
        row["terminal_separation"] = deepcopy(checked)
        row["times"]["terminal_separation_s"] = perf_counter() - before
        if guard is not None:
            guard()
        if checked["status"] != "SUCCESS":
            return self._reject_suffix(row, dict(reason=checked["status"], detail=checked.get("failure")))
        event = dict(event_type="ATTACH", scope="PLANNED_SCENE_TRANSITION_ONLY",
            target_id=target.target_id, q_rad=np.asarray(start).tolist(),
            parent_stage_id=plan["stages"][-1]["stage_id"], next_stage_id=records[-extra]["stage_id"],
            physical_execution="NOT_RUN", measured=False)
        plan.update(status=MARKER, extraction=[np.asarray(q).tolist() for q in path],
            end_q_rad=np.asarray(path[-1]).tolist(), final_state_receipt=path[-1].native_receipt,
            extraction_geometry=geometry, attachment=row["attachment"], events=[*plan["events"],event],
            stages=[{k:deepcopy(r[k]) for k in ("stage","stage_id","parent_stage_id",
                "generation_source","pipeline_id","planner_id","points","time_parameterization",
                "native_solver_calls","authoritative_status","native_output_status") if k in r} for r in records])
        row.update(complete=True, failure=None, actual_end_q_rad=np.asarray(path[-1]).tolist())
        if checked_prefix is not None:
            plan["extraction_segments"] = [deepcopy(checked_prefix["evidence"]), dict(
                substage="RIGHT_INNER_X", stage_id=records[-1]["stage_id"],
                parent_stage_id=records[-1]["parent_stage_id"], geometry=deepcopy(geometry),
                sample_check=deepcopy(samples), points=deepcopy(records[-1]["points"]))]
        row["times"]["total_s"] = perf_counter() - self._single_face_started
        return self.complete_extraction_plan(plan, row, path[-1], target, obstacles, attachment, seed)
