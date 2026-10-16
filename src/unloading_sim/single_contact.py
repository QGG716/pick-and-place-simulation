"""No-prior, one-IK / one-OMPL / one-LIN approach-contact experiment."""
from collections import Counter
from copy import deepcopy
from time import perf_counter
import numpy as np

from .contact_target import ContactTarget
from .independent_cups import (m710_cup_array_from_mapping, evaluate_independent_cup_geometry,
                               select_ideal_independent_cups, audit_actual_independent_cup_contacts)
from .layout_single_carton import virtual_tcp_from_physical_contact
from .geometry import rotation_vector_from_matrix
from .moveit2_backend import MoveItUnavailable, box_message, digest
from .planning_only import PlanningOnlyConnector
from .stage_motion_policy import MotionPurpose

MARKER = "APPROACH_CONTACT_PLANNED_NOT_EXECUTABLE"


class SingleContactClient:
    """Fail-closed protocol fence, including startup. No old planner entry can hide."""
    def __init__(self, client, *, extraction_enabled=False, preplace_enabled=False):
        self.client, self.records = client, []
        self.forbidden_attempts = 0
        self.preplace_enabled = preplace_enabled
        self.extraction_enabled = extraction_enabled or preplace_enabled

    def request(self, request, **kwargs):
        op = request.get("op")
        allowed = op in {"init", "fk", "inspect", "check_start", "ik", "plan"}
        allowed |= self.extraction_enabled and op == "check_extraction_end"
        allowed |= self.preplace_enabled and op == "check_transit_end"
        allowed &= request.get("planning_mode") == "cold_from_scratch"
        if op == "plan":
            extraction = self.extraction_enabled and request.get("stage") == "extraction"
            allowed &= (extraction and request.get("purpose") == "EXTRACTION_PROCESS"
                and request.get("pipeline_id") == "pilz_industrial_motion_planner"
                and request.get("planner_id") == "LIN") or (
                self.preplace_enabled and request.get("stage") == "transit"
                and request.get("purpose") == "FREE_LOADED_TRANSFER"
                and request.get("pipeline_id") == "ompl"
                and request.get("planner_id") == "RRTConnectkConfigDefault") or (
                request.get("attachment") is None and
                (request.get("stage"), request.get("pipeline_id"), request.get("planner_id")) in {
                ("pregrasp", "ompl", "RRTConnectkConfigDefault"),
                ("contact", "pilz_industrial_motion_planner", "LIN")})
        if not allowed or any(k in request for k in ("prior_context", "database", "history", "path", "ik_seeds")):
            self.forbidden_attempts += 1
            raise MoveItUnavailable("SINGLE_CONTACT_FORBIDDEN_REQUEST:"+str(op))
        if op in {"plan","check_start","ik","check_extraction_end","check_transit_end"}:
            policy=request.get("process_policy",{})
            target=policy.get("target_id")
            world_ids=[b["id"] for b in request.get("world",[])]
            attached=request.get("attachment")
            extraction=self.extraction_enabled and request.get("stage")=="extraction" and op in {"plan","check_extraction_end"}
            transit=self.preplace_enabled and request.get("stage")=="transit" and op in {"plan","ik","check_transit_end"}
            if extraction or transit:
                actual=policy.get("actual_contact_mask",[])
                valid=(attached is not None and attached.get("id")==target and target not in world_ids
                    and policy.get("attached") is True and len(actual)==72 and any(actual)
                    and all(isinstance(x,bool) for x in actual)
                    and actual==policy.get("commanded_active_mask"))
                if transit:
                    valid &= (request.get("purpose")=="FREE_LOADED_TRANSFER"
                        and policy.get("initial_proximity") is None and policy.get("support_names")==[]
                        and bool(request.get("parent_stage_id"))
                        and request.get("ownership_transition") is None)
                if op in {"check_extraction_end","check_transit_end"}:
                    valid &= policy.get("initial_proximity") is None and policy.get("support_names")==[]
                    valid &= bool(request.get("parent_stage_id")) and request.get("ownership_transition") is None
            else:
                valid=attached is None and policy.get("attached") is False and target in world_ids and op not in {"check_extraction_end","check_transit_end"}
            if not valid or len(world_ids)!=len(set(world_ids)):
                raise MoveItUnavailable("SINGLE_CONTACT_TARGET_OWNERSHIP_INVALID")
        began = perf_counter()
        entry = {k: deepcopy(request.get(k)) for k in (
            "op","task_id","stage_id","parent_stage_id","stage","candidate_id","seed",
            "pipeline_id","planner_id","q_start","q_goal","goal_pose","scene_fingerprint",
            "allowed_planning_time_s","ik_timeout_s")}
        if op in {"plan","check_start","ik","check_extraction_end","check_transit_end"}:
            entry.update(attachment=request.get("attachment"),
                target_id=request["process_policy"]["target_id"],
                target_in_world=request["process_policy"]["target_id"] in {b["id"] for b in request["world"]},
                ownership_transition=request.get("ownership_transition"),
                pair_profile_enabled=request.get("pair_profile_enabled"),
                native_check_reuse=request.get("native_check_reuse"),
                process_context={k:deepcopy(policy.get(k)) for k in
                    ("support_names","initial_proximity","actual_contact_mask","commanded_active_mask")})
        try:
            response = self.client.request(request, **kwargs)
        except Exception as exc:
            entry.update(status="TRANSPORT_OR_CANCELLATION_ERROR", detail=str(exc))
            raise
        else:
            entry["response"] = {k: deepcopy(v) for k,v in response.items()
                if k not in {"points","identity","capabilities","frames"}}
            if isinstance(entry["response"].get("clearance"), dict):
                entry["response"]["clearance"].pop("policy", None)
            status = response.get("status","MISSING_STATUS")
            entry["status"] = status
            if (status in {"ERROR","CANCELLED"} or status.startswith(("INVALID_START","NATIVE_TASK_"))
                    or any(word in status for word in ("CONTEXT","IDENTITY","MISMATCH"))):
                raise MoveItUnavailable("REQUEST_STOP:"+status+":"+str(response.get("detail","")))
            return response
        finally:
            entry["round_trip_s"] = perf_counter()-began
            self.records.append(entry)

    def close(self):
        self.client.close()


def check_contact_samples(path, physical_fk, target, *, position_tolerance, orientation_tolerance):
    """Only returned samples, no extra collision or dense interpolation."""
    origin = target.approach_pose[:3,3]
    direction = -target.outward_normal
    previous = 0.
    maximum_line = maximum_angle = 0.
    failure = None
    for index,q in enumerate(path):
        pose = physical_fk(q)
        progress = float((pose[:3,3]-origin)@direction)
        closest = origin + np.clip(progress,0.,target.standoff_m)*direction
        line_error = float(np.linalg.norm(pose[:3,3]-closest))
        angle = float(np.linalg.norm(rotation_vector_from_matrix(
            pose[:3,:3]@target.contact_pose[:3,:3].T)))
        maximum_line=max(maximum_line,line_error); maximum_angle=max(maximum_angle,angle)
        if (line_error>position_tolerance or angle>orientation_tolerance
                or progress < previous-position_tolerance):
            failure = failure or dict(reason="CONTACT_NORMAL_LINE_OR_ORIENTATION",sample=index,
                line_error_m=line_error,orientation_error_rad=angle,progress_m=progress)
        previous = max(previous, progress)
    start_error = float(np.linalg.norm(physical_fk(path[0])[:3,3]-origin))
    end_error = float(np.linalg.norm(physical_fk(path[-1])[:3,3]-target.contact_pose[:3,3]))
    if start_error>position_tolerance or end_error>position_tolerance:
        failure = failure or dict(reason="CONTACT_ENDPOINT_FK",start_error_m=start_error,end_error_m=end_error)
    return dict(passed=failure is None,failure=failure,samples=len(path),additional_samples=0,
        maximum_line_error_m=maximum_line,maximum_orientation_error_rad=maximum_angle,
        start_error_m=start_error,end_error_m=end_error,position_tolerance_m=position_tolerance,
        orientation_tolerance_rad=orientation_tolerance)


class SingleContactConnector(PlanningOnlyConnector):
    """Consumes one target and a supplied scene; never chooses a face internally."""
    def _forbidden_old_entry(self, *args, **kwargs):
        raise MoveItUnavailable("SINGLE_CONTACT_OLD_SCHEDULER_FORBIDDEN")
    _approach = _connect_pose = native_ik_stream = _ik_stream = _forbidden_old_entry
    load_static_prior = plan = _forbidden_old_entry

    def _build_native_request(self, *args, **kwargs):
        request=super()._build_native_request(*args,**kwargs)
        request.update(pair_profile_enabled=False,native_check_reuse=True)
        return request

    def physical_contact_pose(self, q):
        return self.physical_from_virtual(self.robot.fk(q))

    def check_context(self, target, obstacles):
        target.validate()
        if getattr(self, "cancel_requested", False):
            raise MoveItUnavailable("REQUEST_CANCELLED")
        if self._native_cancelled():
            raise MoveItUnavailable("REQUEST_DEADLINE")
        if (target.scene_revision != self.native_scene.snapshot["scene_fingerprint"]
                or digest([box_message(b) for b in obstacles]) != self._single_world_signature):
            raise MoveItUnavailable("SCENE_CONTEXT_CHANGED")
        matches = [b for b in obstacles if b.name == target.target_id]
        if len(matches)!=1 or box_message(matches[0])!=box_message(target.box):
            raise MoveItUnavailable("TARGET_SCENE_GEOMETRY_CHANGED")

    def expected_contact(self, target, suction):
        array=m710_cup_array_from_mapping(suction)
        geometry=evaluate_independent_cup_geometry(target.contact_pose,target.box,target.legacy_face,array,
            max_attachment_gap_m=.002,maximum_penetration_m=self.contact_tolerance_m,
            max_normal_misalignment_rad=np.deg2rad(5.),
            suction_edge_margin_m=float(suction.get("suction_edge_margin_m",0.)))
        selection=select_ideal_independent_cups(geometry,pose_source="REQUESTED_CONTACT_EXPECTATION")
        # The worker receives command/eligible masks, never a preclaimed actual seal.
        context=dict(geometrically_eligible_mask=list(selection.geometrically_eligible_mask),
            commanded_active_mask=list(selection.commanded_active_mask),actual_contact_mask=[False]*72)
        self._native_contact_context=dict(target=target.box,face=target.legacy_face,
            suction=deepcopy(suction),selection=context)
        self.robot_state_validator.contact_target_name=target.target_id
        self.robot_state_validator.commanded_cup_mask=tuple(context["commanded_active_mask"])
        return selection, array

    def ik_once(self, target, start, obstacles, seed):
        pose=virtual_tcp_from_physical_contact(target.approach_pose,
            self.flange_from_virtual_task_tcp,self.flange_from_physical_contact)
        return self.ik_pose_once(pose,start,obstacles,seed,stage="pregrasp")

    def ik_pose_once(self, pose, start, obstacles, seed, *, stage, attachment=None,
                     purpose=None, residual_reason="APPROACH_IK_FK_RESIDUAL"):
        request=self._build_native_request(start,None,obstacles,seed=seed,stage=stage,
            goal_pose=pose,attachment=attachment)
        if attachment is not None:
            receipt,_,transition=self._native_parent_for_request(start,request)
            if transition is not None:
                raise MoveItUnavailable("LOADED_IK_OWNERSHIP_CHANGED")
            request.update(parent_stage_id=receipt.stage_id,parent_state_receipt=receipt.evidence(),
                ownership_transition=None,purpose=getattr(purpose,"value",purpose))
        stage_s,timeout=self._native_limits()
        request.update(op="ik",position_tolerance_m=float(self.ik["position_tolerance_m"]),
            orientation_tolerance_rad=float(self.ik["orientation_tolerance_rad"]),
            allowed_planning_time_s=stage_s,ik_timeout_s=min(stage_s,self.native_ik_seconds),
            collision_check_in_native_ik=False)
        raw=self.native.request(request,timeout=timeout,cancelled=self._native_cancelled)
        self.native_ik_evidence.append(deepcopy(raw))
        if raw["status"]!="SUCCESS":
            return None,dict(reason=raw["status"],scope="FACE_ATTEMPT",native=raw)
        q=np.asarray(raw.get("q"),dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all() or not self.robot.within_limits(q):
            raise MoveItUnavailable("INVALID_NATIVE_IK_RETURN")
        actual=self.robot.fk(q)
        pe=float(np.linalg.norm(actual[:3,3]-pose[:3,3]))
        re=float(np.linalg.norm(rotation_vector_from_matrix(actual[:3,:3]@pose[:3,:3].T)))
        if pe>self.ik["position_tolerance_m"] or re>self.ik["orientation_tolerance_rad"]:
            return None,dict(reason=residual_reason,position_m=pe,orientation_rad=re,scope="FACE_ATTEMPT")
        return q,None

    def complete_contact_plan(self, plan, row, contact_end, target, obstacles, suction, seed):
        """Default endpoint remains contact; extensions consume the actual state receipt."""
        return plan, row

    def plan_target(self, target: ContactTarget, start, obstacles, suction, seed):
        began=perf_counter()
        row=dict(target=target.evidence(),complete=False,times=dict(
            target_geometry_s=None,ik_s=None,pregrasp_s=None,contact_s=None,terminal_check_s=None),
            failure=None)
        self._single_active_row = row
        self._single_face_started = began
        self.check_context(target,obstacles)
        geometry_started=perf_counter()
        try:
            expected,array=self.expected_contact(target,suction)
        except ValueError as exc:
            row["failure"]=dict(reason="EXPECTED_CONTACT_NOT_APPLICABLE",detail=str(exc),scope="FACE_ATTEMPT")
            row["times"]["target_geometry_s"]=perf_counter()-geometry_started
            row["times"]["total_s"]=perf_counter()-began
            return None,row
        row["times"]["target_geometry_s"]=perf_counter()-geometry_started
        row["expected_contact"]=dict(geometrically_eligible_mask=list(expected.geometrically_eligible_mask),
            commanded_active_mask=list(expected.commanded_active_mask),
            actual_contact_mask=None,source="EXPECTED_GEOMETRY_NOT_MEASUREMENT")
        self._candidate_identity="single-contact:"+target.geometric_face_id
        initial=self.native_root_state(start,obstacles,target_contact=target.box)
        check=self._build_native_request(initial,initial,obstacles,seed=seed,stage="pregrasp")
        check.update(op="check_start")
        _,timeout=self._native_limits()
        self.native.request(check,timeout=timeout,cancelled=self._native_cancelled)
        self.check_context(target,obstacles)
        before=perf_counter()
        goal,failure=self.ik_once(target,initial,obstacles,seed)
        row["times"]["ik_s"]=perf_counter()-before
        if failure:
            row["failure"]=failure
        else:
            self.check_context(target,obstacles)
            before=perf_counter()
            prefix,failure,ev=self._native_plan(initial,goal,obstacles,seed=seed,stage="pregrasp",
                purpose=MotionPurpose.FREE_APPROACH,allow_ompl=True,max_ompl_attempts=1,
                try_ptp=False,try_prior=False)
            row["times"]["pregrasp_s"]=perf_counter()-before
            row["failure"]=failure
            if failure is None:
                self.check_context(target,obstacles)
                if not np.array_equal(prefix[0],initial):
                    raise MoveItUnavailable("PREGRASP_EXACT_START_CHANGED")
                # Output samples and receipt remain unchanged, including roundoff.
                contact_goal=virtual_tcp_from_physical_contact(target.contact_pose,
                    self.flange_from_virtual_task_tcp,self.flange_from_physical_contact)
                before=perf_counter()
                contact,failure,ev=self._native_plan(prefix[-1],None,obstacles,seed=seed,stage="contact",
                    purpose=MotionPurpose.CONTACT_PROCESS,goal_pose=contact_goal,target_contact=target.box,
                    allow_ompl=False,max_ompl_attempts=0,try_ptp=False,try_prior=False)
                row["times"]["contact_s"]=perf_counter()-before
                row["failure"]=failure
                if failure is None:
                    self.check_context(target,obstacles)
                    if not np.array_equal(contact[0],prefix[-1]):
                        raise MoveItUnavailable("CONTACT_EXACT_PARENT_STATE_CHANGED")
                    before=perf_counter()
                    audit=check_contact_samples(contact,self.physical_contact_pose,target,
                        position_tolerance=float(self.ik["position_tolerance_m"]),
                        orientation_tolerance=float(self.ik["orientation_tolerance_rad"]))
                    row["contact_fk_check"]=audit
                    row["failure"]=audit["failure"]
                    if audit["passed"]:
                        geometry=evaluate_independent_cup_geometry(self.physical_contact_pose(contact[-1]),
                            target.box,target.legacy_face,array,max_attachment_gap_m=.002,
                            maximum_penetration_m=self.contact_tolerance_m,
                            max_normal_misalignment_rad=np.deg2rad(5.),
                            suction_edge_margin_m=float(suction.get("suction_edge_margin_m",0.)))
                        terminal=audit_actual_independent_cup_contacts(geometry,expected.commanded_active_ids,
                            pose_source="ACTUAL_RETURNED_Q_FK_PLANNING_GEOMETRY")
                        row["terminal_contact"]=dict(actual_contact_mask=list(terminal.actual_contact_mask),
                            commanded_active_mask=list(terminal.commanded_active_mask),
                            geometrically_eligible_mask=list(terminal.geometrically_eligible_mask),
                            measured=False,attached=False)
                        if terminal.actual_contact_mask!=terminal.commanded_active_mask or not any(terminal.actual_contact_mask):
                            row["failure"]=dict(reason="TERMINAL_CONTACT_GEOMETRY_NOT_SATISFIED",scope="FACE_ATTEMPT")
                    row["times"]["terminal_check_s"]=perf_counter()-before
                    if row["failure"] is None:
                        records=self._native_selected_records(contact[-1])
                        if len(records)!=2 or records[1]["parent_stage_id"]!=records[0]["stage_id"]:
                            raise MoveItUnavailable("TWO_STAGE_PARENT_CHAIN_INVALID")
                        row.update(complete=True,actual_end_q_rad=np.asarray(contact[-1]).tolist())
                        row["times"]["total_s"]=perf_counter()-began
                        plan = dict(status=MARKER,execution_ready=False,qualification_status="NOT_EVALUATED",
                            continuous_swept_proof=False,physical_execution="NOT_RUN",target=target.evidence(),
                            task_id=self.native_task_id,start_q_rad=np.asarray(initial).tolist(),
                            end_q_rad=np.asarray(contact[-1]).tolist(),
                            pregrasp=[np.asarray(q).tolist() for q in prefix],
                            contact=[np.asarray(q).tolist() for q in contact],
                            stages=[{k:deepcopy(r[k]) for k in ("stage","stage_id","parent_stage_id",
                                "generation_source","pipeline_id","planner_id","points","time_parameterization",
                                "native_solver_calls","authoritative_status","native_output_status") if k in r} for r in records],
                            final_state_receipt=contact[-1].native_receipt,events=[])
                        return self.complete_contact_plan(plan,row,contact[-1],target,obstacles,suction,seed)
        row["times"]["total_s"]=perf_counter()-began
        # Only normal bounded solver/geometry outcomes allow another face.
        reason=(row["failure"] or {}).get("reason","UNKNOWN_FAILURE")
        allowed={"NATIVE_IK_FAILED","APPROACH_IK_FK_RESIDUAL","INVALID_GOAL_CLEARANCE",
            "INVALID_GOAL_PROCESS","MOVEIT2_SEARCH_EXHAUSTED","NATIVE_PLANNING_FAILED",
            "NATIVE_PLANNING_TIMEOUT","SEARCH_EXHAUSTED","CONTACT_NORMAL_LINE_OR_ORIENTATION",
            "CONTACT_ENDPOINT_FK","TERMINAL_CONTACT_GEOMETRY_NOT_SATISFIED"}
        if reason not in allowed:
            raise MoveItUnavailable("REQUEST_FAILURE:"+reason)
        row["failure"]["scope"]="FACE_ATTEMPT"
        return None,row


def plan_faces(connector, targets, start, velocity, source, obstacles, suction, seed):
    """Upstream coordinator: first completed face wins; a single shared deadline."""
    q=np.asarray(start,dtype=float);v=np.asarray(velocity,dtype=float)
    if (not source or q.shape!=(6,) or v.shape!=(6,) or not np.isfinite(q).all()
            or not np.isfinite(v).all() or np.any(v!=0.) or not connector.robot.within_limits(q)):
        raise MoveItUnavailable("INVALID_OR_NONSTATIONARY_START_INPUT")
    connector._single_world_signature=digest([box_message(b) for b in obstacles])
    targets=list(targets)
    roles=[t.face_role for t in targets]
    ranks={"FRONT":0,"TOP":1,"SIDE":2}
    if (len(targets)>4 or roles!=sorted(roles,key=ranks.__getitem__)
            or roles.count("FRONT")>1 or roles.count("TOP")>1 or roles.count("SIDE")>2
            or len({t.geometric_face_id for t in targets})!=len(targets)
            or len({(t.target_id,t.scene_revision) for t in targets})>1):
        raise ValueError("INVALID_FIXED_FACE_QUEUE")
    rows=[]
    for index,target in enumerate(targets):
        connector._single_active_row=None
        if index>=4:
            raise ValueError("TOO_MANY_PHYSICAL_FACES")
        try:
            plan,row=connector.plan_target(target,q,obstacles,suction,seed)
        except Exception as exc:
            row=connector._single_active_row or dict(target=target.evidence(),complete=False)
            row["failure"]=dict(reason=type(exc).__name__,detail=str(exc),scope="REQUEST")
            if "times" in row:
                row["times"]["total_s"]=perf_counter()-connector._single_face_started
            rows.append(row)
            return None,rows
        rows.append(row)
        if plan is not None:
            return plan,rows
    return None,rows
