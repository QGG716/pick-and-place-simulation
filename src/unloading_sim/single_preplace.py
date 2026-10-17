"""One loaded IK and one RRTConnect to a fixed preplace, still attached."""
from copy import deepcopy
from time import perf_counter
import numpy as np

from .single_extraction import SingleExtractionConnector
from .preplace_target import fixed_preplace_target
from .stage_motion_policy import MotionPurpose
from .moveit2_backend import MoveItUnavailable
from .geometry import rotation_vector_from_matrix

MARKER="APPROACH_CONTACT_EXTRACTION_PREPLACE_PLANNED_NOT_EXECUTABLE"


class SinglePreplaceConnector(SingleExtractionConnector):
    def _reject_transit(self,row,failure):
        self.check_context(self._preplace_contact,self.native_scene.all_obstacles)
        reason=failure.get("reason","UNKNOWN_TRANSIT_FAILURE")
        if reason not in {"FIXED_PREPLACE_TARGET_NOT_APPLICABLE","NATIVE_IK_FAILED","PREPLACE_IK_FK_RESIDUAL",
            "NATIVE_PLANNING_FAILED","NATIVE_PLANNING_TIMEOUT","SEARCH_EXHAUSTED","MOVEIT2_SEARCH_EXHAUSTED",
            "INVALID_GOAL_CLEARANCE","INVALID_GOAL_PROCESS","PREPLACE_ENDPOINT_FK",
            "TRANSIT_END_CLEARANCE_REJECTED","TRANSIT_END_PROCESS_REJECTED"}:
            raise MoveItUnavailable("TRANSIT_REQUEST_FAILURE:"+reason)
        row["failure"]={**failure,"stage":"transit","scope":"FACE_ATTEMPT",
            "interpretation":"BOUNDED_ATTEMPT_NOT_COMPLETED_NOT_GLOBAL_UNREACHABILITY"}
        row["times"]["total_s"]=perf_counter()-self._single_face_started
        return None,row

    def complete_extraction_plan(self,plan,row,start,target,obstacles,attachment,seed):
        # This hook runs before SingleExtractionConnector exits its _contact_context.
        row.update(complete=False,extraction_complete=True)
        row["extraction_end_q_rad"]=row.pop("actual_end_q_rad")
        self._preplace_contact=target
        for key in ("preplace_target_s","transit_ik_s","transit_s","preplace_endpoint_s"):
            row["times"][key]=None
        self.check_context(target,obstacles)
        if (not row["extraction_samples"]["passed"] or row["terminal_separation"]["status"]!="SUCCESS"
            or not np.array_equal(start,plan["extraction"][-1])
            or start.native_receipt!=plan["final_state_receipt"]
            or self._native_state_receipt(start).stage_id!=plan["stages"][-1]["stage_id"]
            or attachment.rigid.name!=target.target_id
            or not np.array_equal(attachment.rigid.half_extents,target.box.half_extents)
            or not np.array_equal(attachment.rigid.tcp_from_box,np.asarray(row["attachment"]["tcp_from_box"]))
            or self._native_contact_context["selection"]!=row["attachment"]["selection"]):
            raise MoveItUnavailable("TRANSIT_EXTRACTION_CONTEXT_CHANGED")
        world=[b for b in obstacles if b.name!=target.target_id]
        before=perf_counter()
        goal,failure=fixed_preplace_target(self,target,attachment,start,world)
        row["times"]["preplace_target_s"]=perf_counter()-before
        row["preplace_target"]=None if goal is None else goal.evidence()
        if failure:return self._reject_transit(row,failure)
        self.check_context(target,obstacles)
        before=perf_counter()
        q,failure=self.ik_pose_once(goal.task_tcp_pose,start,world,seed,stage="transit",
            attachment=attachment,purpose=MotionPurpose.FREE_LOADED_TRANSFER,
            residual_reason="PREPLACE_IK_FK_RESIDUAL",ik_mode="bounded_preplace",solver_budget_s=.10)
        row["times"]["transit_ik_s"]=perf_counter()-before
        if failure:return self._reject_transit(row,failure)
        row["preplace_ik_q_rad"]=q.tolist()
        self.check_context(target,obstacles)
        before=perf_counter()
        path,failure,_=self._native_plan(start,q,world,seed=seed,stage="transit",
            attachment=attachment,support_names=(),initial_proximity=None,
            purpose=MotionPurpose.FREE_LOADED_TRANSFER,allow_ompl=True,max_ompl_attempts=1,
            try_ptp=False,try_prior=False)
        row["times"]["transit_s"]=perf_counter()-before
        if failure:return self._reject_transit(row,failure)
        if not np.array_equal(path[0],start):
            raise MoveItUnavailable("TRANSIT_PARENT_STATE_CHANGED")
        self.check_context(target,obstacles)
        before=perf_counter()
        end=path[-1]
        records=self._native_selected_records(end)
        if (len(records)!=4 or [r["stage"] for r in records]!=["pregrasp","contact","extraction","transit"]
            or records[-1]["parent_stage_id"]!=plan["stages"][-1]["stage_id"]
            or not np.array_equal(records[-1]["points"][-1]["q"],end)):
            raise MoveItUnavailable("TRANSIT_PARENT_CHAIN_INVALID")
        velocity=np.asarray(records[-1]["points"][-1]["v"],dtype=float)
        if velocity.shape!=(6,) or not np.isfinite(velocity).all() or np.max(np.abs(velocity))>1e-8:
            raise MoveItUnavailable("TRANSIT_NONZERO_BOUNDARY_VELOCITY")
        actual_physical=self.physical_contact_pose(end);actual_tcp=self.robot.fk(end)
        actual_box=attachment.box_at(end)
        errors={}
        for name,actual,expected in (("physical_contact",actual_physical,goal.physical_contact_pose),
            ("task_tcp",actual_tcp,goal.task_tcp_pose),("box",actual_box.world_from_local,goal.preplace_box.world_from_local)):
            errors[name]=dict(position_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
                rotation_rad=float(np.linalg.norm(rotation_vector_from_matrix(actual[:3,:3]@expected[:3,:3].T))))
        row["preplace_endpoint"]=dict(errors=errors,actual_q_rad=np.asarray(end).tolist(),
            actual_physical_contact_pose=actual_physical.tolist(),actual_tcp_pose=actual_tcp.tolist(),
            actual_box_pose=actual_box.world_from_local.tolist(),end_velocity_rad_s=velocity.tolist(),
            target_attached=True,received=False,physical_execution="NOT_RUN")
        if any(not np.isfinite([e["position_m"],e["rotation_rad"]]).all()
            or e["position_m"]>self.ik["position_tolerance_m"] or e["rotation_rad"]>self.ik["orientation_tolerance_rad"]
            for e in errors.values()):
            row["times"]["preplace_endpoint_s"]=perf_counter()-before
            return self._reject_transit(row,dict(reason="PREPLACE_ENDPOINT_FK",errors=errors))
        checked=self.check_attached_end(end,target,world,attachment,seed,stage="transit")
        row["preplace_endpoint"]["native_check"]=deepcopy(checked)
        row["times"]["preplace_endpoint_s"]=perf_counter()-before
        if checked["status"]!="SUCCESS":
            return self._reject_transit(row,dict(reason=checked["status"],detail=checked.get("failure")))
        if len(plan["events"])!=1 or plan["events"][0]["event_type"]!="ATTACH":
            raise MoveItUnavailable("TRANSIT_ATTACHMENT_EVENT_CHANGED")
        plan.update(status=MARKER,transit=[np.asarray(q).tolist() for q in path],
            preplace_target=goal.evidence(),preplace_endpoint=deepcopy(row["preplace_endpoint"]),
            end_q_rad=np.asarray(end).tolist(),final_state_receipt=end.native_receipt,
            stages=[{k:deepcopy(r[k]) for k in ("stage","stage_id","parent_stage_id","generation_source",
                "pipeline_id","planner_id","points","time_parameterization","native_solver_calls",
                "authoritative_status","native_output_status") if k in r} for r in records])
        row.update(complete=True,failure=None,actual_end_q_rad=np.asarray(end).tolist())
        row["times"]["total_s"]=perf_counter()-self._single_face_started
        return plan,row
