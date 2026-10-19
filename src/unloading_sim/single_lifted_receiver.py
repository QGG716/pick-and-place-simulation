"""Front contact -> checked external belt lift -> normal LIN into a region.

The frozen task snapshot remains immutable. Each generated external-axis stage
owns a new world and a new receipt; no old scene acceptance is transferred.
"""
from collections import deque
from copy import deepcopy
from time import perf_counter
import numpy as np

from .single_extraction import SingleExtractionConnector
from .lifted_reception import StartHeight, receiving_region, translated, bounds, DEFERRED
from .moveit2_backend import MoveItUnavailable, box_message, digest
from .stage_motion_policy import MotionPurpose

MARKER = "LIFTED_RECEIVER_RELEASE_READY_PLANNED_NOT_EXECUTABLE"


def receiver_binding(target, obstacles, designed_contacts=()):
    matches = [b for b in obstacles if b.name == "conveyor_transverse"]
    if len(matches) != 1:
        raise ValueError("RECEIVER_IDENTITY_INVALID")
    low = matches[0]
    h = StartHeight.capture(target)
    high = translated(low, [0, 0, h.hmax-low.center[2]-low.half_extents[2]])
    if not np.allclose(low.rotation[:, 2], [0, 0, 1], atol=1e-12, rtol=0):
        raise ValueError("RECEIVER_NOT_HORIZONTAL")
    if high.center[2] <= low.center[2]:
        raise ValueError("POSITIVE_HMAX_LIFT_REQUIRED")
    contacts=[list(pair) for pair in designed_contacts if low.name in pair]
    return dict(target_id=target.name, low=box_message(low), high=box_message(high),designed_contacts=contacts), high


def region_samples(region, belt, target, tolerance):
    """Explore the full raw interval, including conservatively clipped portions.

    Nearest inward boundary first, then far boundary and interval bisection.
    Resolution is the existing FK tolerance, not a solver-attempt quota. A
    sampled rejection never asserts continuous regional unreachability.
    """
    if not np.allclose(belt.rotation, np.eye(3), atol=1e-12, rtol=0):
        raise ValueError("FRONT_REGION_REQUIRES_WORLD_ALIGNED_BELT")
    a,b,c,d = region["projection_center_limits_before_occupancy"]
    y = target.center[1]-belt.center[1]
    if not c <= y <= d or a > b:
        return
    guard = 2*tolerance
    a,b = a+belt.center[0]+guard, b+belt.center[0]-guard
    if a > b:
        return
    yield b
    if b-a <= tolerance:
        return
    yield a
    queue=deque([(a,b)])
    while queue:
        left,right=queue.popleft()
        if right-left <= 2*tolerance:
            continue
        mid=(left+right)/2
        yield mid
        queue.extend(((mid,right),(left,mid)))


def release_ready_geometry(box, belt, *, tolerance, expected_bottom, expected_rotation):
    local=(box.corners()-belt.center) @ belt.rotation
    inside=bool(np.all(np.abs(local[:,:2]) <= belt.half_extents[:2]-.01+tolerance))
    bottom=float(bounds(box)[0][2]);gap=bottom-belt.center[2]-belt.half_extents[2]
    orientation=float(np.max(np.abs(box.rotation-expected_rotation)))
    valid=bool(inside and abs(gap-.05)<=tolerance and abs(bottom-expected_bottom)<=tolerance and orientation<=2e-4)
    return dict(passed=valid,full_bottom_projection_inside=inside,actual_gap_m=float(gap),
        expected_gap_m=.05,actual_box=box_message(box),orientation_max_abs_error=orientation,
        receiver_id=belt.name,receiver_surface_z_m=float(belt.center[2]+belt.half_extents[2]),
        physical_execution="NOT_RUN",qualification_status="NOT_EVALUATED")


class SingleLiftedReceiverConnector(SingleExtractionConnector):
    def validate_template(self, target):
        if target.face_role != "FRONT" or not np.allclose(target.outward_normal,[-1,0,0],atol=1e-12,rtol=0):
            raise MoveItUnavailable("FRONT_HMAX_TEMPLATE_SCOPE_ONLY")

    def plan_target(self, target, start, obstacles, suction, seed):
        self.validate_template(target)
        self.receiver_binding,self.high_belt=receiver_binding(target.box,obstacles,
            self.native_scene.snapshot["assembly"]["designed_contacts"])
        if self.receiver_binding != self.native.lifted_receiver_binding:
            raise MoveItUnavailable("RECEIVER_BINDING_IDENTITY_CHANGED")
        self.initial_receiver_checks=[]
        self._lift_recovery_state={}
        began=perf_counter();stage_s,_=self._native_limits()
        self._lift_approach_deadline=began+stage_s
        history=[]
        while True:
            self.active_obstacles=list(obstacles)
            self._single_world_signature=digest([box_message(b) for b in obstacles])
            self.receiver_raised=False;self.receiver_events=[];self.receiver_sync_attempts=[]
            plan,row=super().plan_target(target,start,obstacles,suction,seed)
            row["previous_prefix_attempts"]=deepcopy(history)
            row["times"]["total_s"]=perf_counter()-began
            if plan is not None:
                return plan,row
            reason=(row.get("failure") or {}).get("reason")
            retry=reason in {"NO_CHECKED_RECEIVER_SYNC_AT_GENERATED_ENDPOINTS","NATIVE_PLANNING_FAILED",
                "CONTACT_NORMAL_LINE_OR_ORIENTATION","CONTACT_ENDPOINT_FK","TERMINAL_CONTACT_GEOMETRY_NOT_SATISFIED"}
            if reason=="NO_CHECKED_RECEIVER_SYNC_AT_GENERATED_ENDPOINTS" and any(
                    (s["response"].get("failure") or {}).get("reason")=="RECEIVER_WORLD_CLEARANCE"
                    for s in self.receiver_sync_attempts):
                retry=False  # The unchanged fixed world cannot improve by changing robot IK.
            if not retry or self._native_cancelled() or perf_counter()>=self._lift_approach_deadline:
                return None,row
            # Same physical target/root/scene/Task; another numerical approach
            # configuration, never a hardcoded synchronization pose or old q.
            saved=deepcopy(row);saved.pop("previous_prefix_attempts",None);history.append(saved)

    def connect_fixed_target(self,pose,start,obstacles,seed,**kwargs):
        from .single_target_recovery import connect_fixed_target
        previous=getattr(self,"_single_connection_deadline",None)
        self._single_connection_deadline=min(previous,self._lift_approach_deadline) if previous else self._lift_approach_deadline
        try:
            return connect_fixed_target(self,pose,start,obstacles,seed,
                recovery_state=self._lift_recovery_state,**kwargs)
        finally:
            self._single_connection_deadline=previous

    def expected_contact(self, target, suction):
        value=super().expected_contact(target,suction)
        if self.initial_receiver_checks:
            self._single_active_row["initial_receiver_checks"]=self.initial_receiver_checks
            return value
        # Compare the same configured q without creating a Task or a motion.
        q=np.asarray(self.native_scene.snapshot["robot"]["q_rad"])
        for label,world in (("LOW",self.active_obstacles),("HMAX",self.high_world())):
            req=self._build_native_request(q,q,world,seed=0,stage="pregrasp")
            req["op"]="check_receiver_state"
            before=perf_counter();_,timeout=self._native_limits()
            result=self.native.request(req,timeout=timeout,cancelled=self._native_cancelled)
            self.initial_receiver_checks.append(dict(height=label,q_rad=q.tolist(),
                scene_fingerprint=req["scene_fingerprint"],response=deepcopy(result),round_trip_s=perf_counter()-before))
            if label=="LOW" and result["status"]!="SUCCESS":
                raise MoveItUnavailable("INVALID_START_LOW_RECEIVER")
        self._single_active_row["initial_receiver_checks"]=self.initial_receiver_checks
        return value

    def high_world(self):
        return [self.high_belt if b.name==self.high_belt.name else b for b in self.active_obstacles]

    def lift_at(self, start, target, obstacles, seed, row, *, stage):
        self.check_context(target,obstacles)
        req=self._build_native_request(start,start,obstacles,seed=seed,stage=stage)
        receipt,_,transition=self._native_parent_for_request(start,req)
        if transition is not None:
            raise MoveItUnavailable("RECEIVER_LIFT_OWNERSHIP_CHANGED")
        stage_s,timeout=self._native_limits()
        world=self.high_world();serialized=[box_message(b) for b in world]
        req.update(parent_stage_id=receipt.stage_id,parent_state_receipt=receipt.evidence(),
            ownership_transition=None,stage_id=self.native_task_id+":"+str(next(self._native_stage_ids)),
            pipeline_id="external_axis",planner_id="LIFT_Z",receiver_end_world=serialized,
            receiver_end_fingerprint=digest(serialized),allowed_planning_time_s=stage_s,
            purpose=MotionPurpose.FREE_APPROACH.value if stage=="pregrasp" else MotionPurpose.CONTACT_PROCESS.value)
        before=perf_counter()
        raw=self.native.request(req,timeout=timeout,cancelled=self._native_cancelled)
        entry=dict(sync_stage=stage,parent_stage_id=receipt.stage_id,q_rad=np.asarray(start).tolist(),
            status=raw["status"],response=deepcopy(raw),round_trip_s=perf_counter()-before)
        self.receiver_sync_attempts.append(entry)
        row["receiver_sync_attempts"]=self.receiver_sync_attempts
        if raw["status"]!="SUCCESS":
            if raw["status"] not in {"RECEIVER_SWEEP_REJECTED","RECEIVER_SWEEP_TIMEOUT"}:
                raise MoveItUnavailable("RECEIVER_LIFT_REQUEST_STOP:"+raw["status"])
            return start,obstacles,False
        if (self._native_cancelled() or raw.get("task_id")!=self.native_task_id
                or raw.get("stage_id")!=req["stage_id"] or raw.get("parent_stage_id")!=receipt.stage_id
                or raw.get("generation_source")!="NATIVE_EXTERNAL_AXIS"
                or not np.array_equal(raw.get("terminal_q"),start)):
            raise MoveItUnavailable("RECEIVER_RETURN_IDENTITY_OR_DEADLINE")
        # Original snapshot and receipt stay unchanged. The new generated record
        # explicitly binds the changed other_world, preserving q and ownership.
        record=dict(raw,stage=stage,submitted_request=deepcopy(req),
            authoritative_status="SKIPPED_PLANNING_ONLY")
        self.native_generated.append(record)
        end_request=dict(req,world=serialized,scene_fingerprint=digest(serialized))
        state=self._native_issue_state(start,req["stage_id"],self._native_state_context(end_request))
        self.active_obstacles=world
        self._single_world_signature=digest(serialized)
        self.receiver_raised=True
        event=dict(raw["external_axis"],event_type="EXTERNAL_AXIS_MOTION",stage_id=req["stage_id"],
            parent_stage_id=receipt.stage_id,task_id=self.native_task_id,sync_stage=stage,
            selected_state_receipt=state.native_receipt)
        self.receiver_events.append(event)
        return state,world,True

    def prepare_contact_start(self, start, target, obstacles, seed, row):
        end,world,_=self.lift_at(start,target,obstacles,seed,row,stage="pregrasp")
        return end,world

    def complete_contact_plan(self, plan, row, contact_end, target, obstacles, suction, seed):
        if not self.receiver_raised:
            contact_end,obstacles,success=self.lift_at(contact_end,target,obstacles,seed,row,stage="contact")
            if not success:
                row.update(complete=False,failure=dict(reason="NO_CHECKED_RECEIVER_SYNC_AT_GENERATED_ENDPOINTS",
                    scope="FRONT_TEMPLATE_ATTEMPT_NOT_GLOBAL_UNREACHABILITY"))
                return None,row
            records=self._native_selected_records(contact_end)
            plan["stages"]=[{k:deepcopy(r[k]) for k in ("stage","stage_id","parent_stage_id",
                "generation_source","points","pipeline_id","planner_id") if k in r} for r in records]
            plan["final_state_receipt"]=contact_end.native_receipt
        plan["events"]=deepcopy(self.receiver_events)
        plan["receiver_world"]=dict(world=[box_message(b) for b in self.active_obstacles],
            scene_fingerprint=digest([box_message(b) for b in self.active_obstacles]))
        return super().complete_contact_plan(plan,row,contact_end,target,obstacles,suction,seed)

    def _region_context_binding(self, start, target, obstacles, attachment, payload_world, supports, tracker, seed):
        self.check_context(target, obstacles)
        if self._native_cancelled():
            raise MoveItUnavailable("REQUEST_CANCELLED_OR_DEADLINE")
        request=self._build_native_request(start,start,payload_world,seed=seed,stage="extraction",
            attachment=attachment,support_names=supports,initial_proximity=tracker)
        parent,_,transition=self._native_parent_for_request(start,request)
        if transition!="ATTACH":
            raise MoveItUnavailable("RECEIVER_REGION_CONTACT_OWNERSHIP_CHANGED")
        return digest(dict(request=request,parent=parent.evidence(),receiver=box_message(self.high_belt),
            acceptance_policy=dict(position_tolerance_m=self.ik["position_tolerance_m"],
                orientation_tolerance_rad=self.ik["orientation_tolerance_rad"],
                contact_tolerance_m=self.contact_tolerance_m,
                extraction_reserve_m=self.budget.extraction_runtime_clearance_reserve_m)))

    def _region_path_binding(self, start, path, prefix, native, checked_prefix=None):
        values=np.asarray(path,dtype=float)
        if values.ndim!=2 or values.shape[1]!=6 or len(values)<2 or not np.isfinite(values).all():
            raise MoveItUnavailable("RECEIVER_REGION_INVALID_PATH_VALUES")
        if not np.array_equal(path[0],start):
            raise MoveItUnavailable("EXTRACTION_PARENT_STATE_CHANGED")
        records=self._native_selected_records(path[-1])
        extra=1 if checked_prefix is None else 2
        if (len(records)!=len(prefix["stages"])+extra
                or [r["stage_id"] for r in records[:-extra]]!=[r["stage_id"] for r in prefix["stages"]]
                or native.get("task_id")!=self.native_task_id
                or records[-1]["stage_id"]!=native.get("stage_id")
                or records[-extra]["parent_stage_id"]!=self._native_state_receipt(start).stage_id
                or native.get("parent_stage_id")!=records[-1]["parent_stage_id"]
                or not np.array_equal([p["q"] for r in records[-extra:] for p in r["points"]],
                    values)):
            raise MoveItUnavailable("RECEIVER_REGION_PATH_IDENTITY_CHANGED")
        return digest(dict(path=values.tolist(),terminal_receipt=path[-1].native_receipt))

    def region_candidates(self, region, payload, payload_world, attachment, start, row):
        for x in region_samples(region,self.high_belt,payload,float(self.ik["position_tolerance_m"])):
            yield np.array([x,payload.center[1],payload.center[2]])

    def region_motion(self, start, center, target, attachment, payload_world, supports,
                      tracker, seed, required, guard, entry):
        payload=attachment.box_at(start);physical=self.physical_contact_pose(start)
        distance=float(payload.center[0]-center[0])
        goal=physical.copy();goal[0,3]-=distance
        geometry=dict(direction_world=[-1.,0.,0.],distance_m=distance,
            start_physical_pose=physical.tolist(),goal_physical_pose=goal.tolist(),
            required_terminal_clearance_m=required,support_names=supports,
            constraint_ids=[b.name for b in payload_world],
            method="CURRENT_FULL_BOTTOM_REGION_NORMAL_LIN",maximum_distance_m=None,
            old_maximum_extraction_m=self.budget.maximum_extraction_m,
            old_bound_scope="OLD_EXPERIMENT_SEARCH_BOUND_NOT_MECHANICAL_LIMIT")
        path,failure,evidence=self._native_plan(start,None,payload_world,seed=seed,
            stage="extraction",purpose=MotionPurpose.EXTRACTION_PROCESS,
            goal_pose=self.virtual_from_physical(goal),attachment=attachment,
            support_names=supports,initial_proximity=tracker.clone(),
            allow_ompl=False,max_ompl_attempts=0,try_ptp=False,try_prior=False)
        return path,failure,evidence,geometry,None,tracker.clone()

    def _plan_extraction_suffix(self, plan, row, start, target, obstacles,
                                attachment, payload_world, supports, tracker, seed):
        """A native-generated LIN remains a candidate until the entire suffix passes.

        Each branch is evaluated directly, with fresh row/plan and tracker clones.
        The accepted branch is returned as-is: no validation cache or second audit.
        """
        began=perf_counter();stage_s,_=self._native_limits();deadline=began+stage_s
        payload=attachment.box_at(start)
        region=receiving_region(payload,self.high_belt,payload_world)
        row["receiving_region"]=region
        row["region_attempts"]=[]
        self._region_turn_cache={}
        required=max(self.collision_policy.pair_clearance("external",self.collision_margin_m)+self.contact_tolerance_m,
            self.collision_policy.free_space_clearance_m)+self.budget.extraction_runtime_clearance_reserve_m
        original_deadline=getattr(self,"_single_connection_deadline",None)
        self._single_connection_deadline=min(deadline,original_deadline) if original_deadline else deadline
        row["times"]["extraction_target_s"]=perf_counter()-began
        physical=self.physical_contact_pose(start)
        validation_times=("extraction_sample_check_s","terminal_separation_s","release_ready_check_s")
        for key in validation_times:
            row["times"][key]=None
        row["times"]["extraction_s"]=row["times"]["extraction_target_s"]
        binding=None
        def guard():
            if self._region_context_binding(start,target,obstacles,attachment,payload_world,supports,tracker,seed)!=binding:
                raise MoveItUnavailable("RECEIVER_REGION_CONTEXT_CHANGED")
            if perf_counter()>=self._single_connection_deadline:
                raise MoveItUnavailable("RECEIVER_REGION_STAGE_DEADLINE")
        stop="DISCRETE_EXPLORATION_EXHAUSTED"
        try:
            binding=self._region_context_binding(start,target,obstacles,attachment,payload_world,supports,tracker,seed)
            for center in self.region_candidates(region,payload,payload_world,attachment,start,row):
                guard()
                distance=float(payload.center[0]-center[0])
                if distance<=0:
                    continue
                candidate_started=perf_counter()
                entry=dict(center_world_m=center.tolist(),
                    distance_m=distance,planning_s=None,failure=None,selected=False,
                    native_result=dict(status="NOT_OBSERVED"),sample_check=dict(status="NOT_REACHED"),
                    terminal_separation=dict(status="NOT_REACHED"),release_ready_check=dict(status="NOT_REACHED"),
                    acceptance_status="PENDING",context_sha256=binding)
                row["region_attempts"].append(entry)
                branch=dict(row,times={**row["times"],**dict.fromkeys(validation_times)},
                    attachment=deepcopy(row["attachment"]),complete=False,failure=None)
                try:
                    before=perf_counter()
                    try:
                        path,failure,evidence,geometry,checked_prefix,branch_tracker=self.region_motion(
                            start,center,target,attachment,payload_world,supports,tracker,seed,required,guard,entry)
                    finally:
                        entry["planning_s"]=perf_counter()-before
                        row["times"]["extraction_s"]+=entry["planning_s"]
                    attempts=evidence.get("attempts",[])
                    if attempts:
                        entry["native_result"]={k:deepcopy(attempts[-1].get(k)) for k in
                            ("status","task_id","stage_id","parent_stage_id","request_id","solver_message","moveit_error_code")}
                        entry.update(stage_id=attempts[-1].get("stage_id"),parent_stage_id=attempts[-1].get("parent_stage_id"))
                    guard()
                    if failure is not None:
                        if entry.get('first_leg_postcheck_failure') is not None:
                            if failure.get('reason') not in {'CONTACT_NORMAL_LINE_OR_ORIENTATION','CONTACT_ENDPOINT_FK',
                                    'EXTRACTION_RETURNED_SAMPLE_PAYLOAD_REJECTED'}:
                                raise MoveItUnavailable('RECEIVER_REGION_UNKNOWN_FIRST_LEG_CHECK')
                            entry.update(failure={**deepcopy(failure),'scope':'CANDIDATE_PATH'},acceptance_status='POSTCHECK_REJECTED')
                            continue
                        if failure.get("reason") not in {"NATIVE_PLANNING_FAILED","NATIVE_PLANNING_TIMEOUT",
                                "MOVEIT2_SEARCH_EXHAUSTED","SEARCH_EXHAUSTED","INVALID_GOAL_CLEARANCE","INVALID_GOAL_PROCESS"}:
                            raise MoveItUnavailable("RECEIVER_REGION_REQUEST_STOP:"+str(failure.get("reason")))
                        entry.update(failure={**deepcopy(failure),"scope":"CANDIDATE_METHOD"},
                            acceptance_status="NATIVE_METHOD_NOT_CONNECTED")
                        continue
                    if entry["native_result"]["status"]!="SUCCESS":
                        raise MoveItUnavailable("RECEIVER_REGION_NATIVE_RESULT_UNKNOWN")
                    entry["path_sha256"]=self._region_path_binding(start,path,plan,entry["native_result"],checked_prefix)
                    entry["acceptance_status"]="NATIVE_GENERATED_CANDIDATE"
                    branch["extraction_geometry"]=geometry
                    accepted,branch=self._complete_extraction_candidate(deepcopy(plan),branch,start,path,geometry,
                        target,obstacles,attachment,payload_world,supports,branch_tracker,seed,guard=guard,
                        checked_prefix=checked_prefix)
                    guard()
                    if entry["path_sha256"]!=self._region_path_binding(start,path,plan,entry["native_result"],checked_prefix):
                        raise MoveItUnavailable("RECEIVER_REGION_VALIDATED_PATH_CHANGED")
                    if accepted is None:
                        failure=branch.get("failure") or {}
                        allowed={"CONTACT_NORMAL_LINE_OR_ORIENTATION","CONTACT_ENDPOINT_FK",
                            "EXTRACTION_RETURNED_SAMPLE_PAYLOAD_REJECTED","ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED",
                            "EXTRACTION_END_CLEARANCE_REJECTED","EXTRACTION_END_PROCESS_REJECTED",
                            "ACTUAL_RELEASE_REGION_NOT_REACHED"}
                        if failure.get("reason") not in allowed or (
                                failure.get("reason") in {"EXTRACTION_END_CLEARANCE_REJECTED","EXTRACTION_END_PROCESS_REJECTED"}
                                and not failure.get("detail")):
                            raise MoveItUnavailable("RECEIVER_REGION_UNKNOWN_POSTCHECK:"+str(failure))
                        entry.update(failure={**deepcopy(failure),"scope":"CANDIDATE_PATH_OR_GOAL"},
                            acceptance_status="POSTCHECK_REJECTED")
                        continue
                    entry.update(selected=True,acceptance_status="FINAL_SELECTED")
                    for key in ("extraction_geometry","extraction_samples","terminal_separation","release_ready_check",
                                "actual_end_q_rad","attachment"):
                        row[key]=branch[key]
                    row.update(complete=True,failure=None)
                    return accepted,row
                except Exception as exc:
                    entry.update(selected=False,acceptance_status="REQUEST_STOPPED",
                        failure=dict(reason=str(exc),scope="REQUEST"))
                    if str(exc) in {"RECEIVER_REGION_STAGE_DEADLINE","SINGLE_TARGET_STAGE_DEADLINE"} and not self._native_cancelled():
                        entry["acceptance_status"]="SEARCH_INCOMPLETE"
                        entry["failure"]["scope"]="REGION_BUDGET"
                    raise
                finally:
                    for key in validation_times:
                        value=branch["times"].get(key)
                        entry[key]=value
                        if value is not None:
                            row["times"][key]=(row["times"].get(key) or 0.)+value
                    for src,dst in (("extraction_samples","sample_check"),("terminal_separation","terminal_separation"),
                                    ("release_ready_check","release_ready_check")):
                        if src in branch:
                            data=branch[src]
                            entry[dst]={k:deepcopy(data[k]) for k in data if k in
                                {"status","passed","failure","checked_samples","short_pairs","minimum_payload_world_gap_m",
                                 "actual_gap_m","full_bottom_projection_inside","orientation_max_abs_error"}}
                    if entry.get('segments') and entry['segments'][-1]['substage']=='RIGHT_INNER_X':
                        entry['segments'][-1].update(sample_check=deepcopy(entry['sample_check']),
                            terminal_separation=deepcopy(entry['terminal_separation']),
                            release_ready_check=deepcopy(entry['release_ready_check']))
                    entry["candidate_total_s"]=perf_counter()-candidate_started
        except MoveItUnavailable as exc:
            if str(exc) not in {"RECEIVER_REGION_STAGE_DEADLINE","SINGLE_TARGET_STAGE_DEADLINE"} or self._native_cancelled():
                row["complete"]=False
                raise
            stop="STAGE_BUDGET_EXHAUSTED"
        finally:
            row["times"]["region_search_s"]=perf_counter()-began
            row["times"]["total_s"]=perf_counter()-self._single_face_started
            self._single_connection_deadline=original_deadline
            self._region_turn_cache={}
        return self._reject_suffix(row,dict(reason="RECEIVER_REGION_SEARCH_INCOMPLETE",scope="FRONT_TEMPLATE_ONLY",
            termination=stop,samples_attempted=len(row["region_attempts"]),global_unreachable=False))

    def _reject_suffix(self,row,failure):
        if failure.get("reason")=="RECEIVER_REGION_SEARCH_INCOMPLETE":
            row.update(complete=False,failure=failure)
            row["times"]["total_s"]=perf_counter()-self._single_face_started
            return None,row
        return super()._reject_suffix(row,failure)

    def complete_extraction_plan(self, plan, row, end, target, obstacles, attachment, seed):
        before=perf_counter()
        check=release_ready_geometry(attachment.box_at(end),self.high_belt,
            tolerance=float(self.ik["position_tolerance_m"]),expected_bottom=StartHeight.capture(target.box).z_b,
            expected_rotation=target.box.rotation)
        row["release_ready_check"]=check
        row["times"]["release_ready_check_s"]=perf_counter()-before
        row["times"]["total_s"]=perf_counter()-self._single_face_started
        if not check["passed"]:
            row.update(complete=False,failure=dict(reason="ACTUAL_RELEASE_REGION_NOT_REACHED",scope="FRONT_TEMPLATE_ONLY"))
            return None,row
        plan.update(status=MARKER,release_ready=check,target_still_attached=True,
            receiving_region=row["receiving_region"],region_attempts=row["region_attempts"],deferred=dict(DEFERRED))
        return plan,row
