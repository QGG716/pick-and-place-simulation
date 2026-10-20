"""Shared-budget local IK/method recovery. A new native request is a RESTART."""
from time import perf_counter
import numpy as np
from .moveit2_backend import MoveItUnavailable, digest
from .approach_recovery_policy import equivalent_bounded_configurations
from .approach_quality import path_metrics


def connect_fixed_target(c,pose,start,obstacles,seed,*,stage,purpose,attachment=None,residual_reason,recovery_state=None):
    stage_s,_=c._native_limits()
    previous=getattr(c,"_single_connection_deadline",None)
    c._single_connection_deadline=min(perf_counter()+stage_s,previous) if previous is not None else perf_counter()+stage_s
    original_q=np.asarray(start).copy();original_pose=np.asarray(pose).copy()
    def current_context():
        request=c._build_native_request(start,None,obstacles,seed=seed,stage=stage,attachment=attachment,goal_pose=pose)
        receipt,_,transition=c._native_parent_for_request(start,request)
        return digest(dict(request=request,parent=receipt.evidence(),transition=transition))
    context=current_context();state={} if recovery_state is None else recovery_state
    if state and state.get("context_sha256")!=context:
        c._single_connection_deadline=previous
        raise MoveItUnavailable("FIXED_TARGET_RECOVERY_CONTEXT_CHANGED")
    state.setdefault("context_sha256",context)
    invalid=state.setdefault("invalid_local_ik",[])
    pool=state.setdefault("valid_local_ik",[])
    tolerance=c.ik["candidate_dedup_tolerance_rad"]
    costs=dict(ik_s=0.,motion_s=0.);last_failure=None;attempt=0
    if not hasattr(c,"connection_attempts"):c.connection_attempts=[]
    invocation=len(c.connection_attempts)
    direct=(getattr(c,"empty_approach_direct_first",False) and stage=="pregrasp" and
        getattr(purpose,"value",purpose)=="FREE_APPROACH" and attachment is None)
    def same(a,b):return np.max(np.abs(np.asarray(a)-np.asarray(b)))<=tolerance
    def guard():
        if c._native_cancelled():raise MoveItUnavailable("REQUEST_CANCELLED_OR_DEADLINE")
        if current_context()!=context or not np.array_equal(start,original_q) or not np.array_equal(pose,original_pose):
            raise MoveItUnavailable("FIXED_TARGET_CONTEXT_CHANGED")
    def brief(f):
        if f is None:return None
        return dict(reason=f.get("reason"),native_request_id=f.get("native",{}).get("request_id"),
            requests=[{k:a.get(k) for k in ("request_id","stage_id","status","solver_message","moveit_error_code")}
                      for a in f.get("attempts",[])])
    def add(q):
        variants=equivalent_bounded_configurations(c.robot,q) if hasattr(c,"robot") else [q]
        for v in variants:
            if not any(same(v,p["q"]) for p in pool) and not any(same(v,x) for x in invalid):
                pool.append(dict(q=np.asarray(v).copy(),motion_s=0.,returned_prefixes=0,
                    joint_cost=float(np.abs(v-original_q).sum()),endpoint_invalid=False))
    def quality(path,evidence,elapsed):
        if not hasattr(c,"robot"):return None
        successful=[a for a in evidence.get("attempts",[]) if a.get("status")=="SUCCESS" and a.get("points")]
        if not successful:return None
        record=successful[-1]
        metrics=path_metrics(c.robot,record["points"],getattr(c,"flange_from_physical_contact",None),elapsed)
        metrics.update(planner_id=record["planner_id"],stage_id=record["stage_id"])
        return metrics
    try:
        while perf_counter()<c._single_connection_deadline:
            guard();attempt+=1
            row=dict(connection_id=invocation,attempt=attempt,stage=stage,context_sha256=context,
                method="DIRECT_PREFERRED_WITH_RRT_RESTART" if direct else "BUDGET_RRT_RESTART",
                tree_retained=False,recovery_mode="shared_budget",invalid_endpoint_count=len(invalid),
                retained_legal_configuration_count=sum(not p["endpoint_invalid"] for p in pool),
                remaining_stage_s=c._single_connection_deadline-perf_counter())
            c.connection_attempts.append(row)
            # Enumeration excludes known q only to obtain a DIFFERENT numerical
            # solution. Known legal configurations remain in the planning pool.
            excluded=[*invalid,*[p["q"] for p in pool if not p["endpoint_invalid"]]]
            before=perf_counter()
            q,failure=c.ik_pose_once(pose,start,obstacles,seed,stage=stage,attachment=attachment,
                purpose=purpose,residual_reason=residual_reason,ik_mode="bounded_target",
                solver_budget_s=.10,excluded_ik=excluded)
            row["ik_s"]=perf_counter()-before;costs["ik_s"]+=row["ik_s"];guard()
            if failure:
                last_failure=failure;row.update(status="IK_NOT_COMPLETED",failure=brief(failure))
                if failure.get("reason") not in {"NATIVE_IK_FAILED",residual_reason}:
                    raise MoveItUnavailable("FIXED_TARGET_IK_REQUEST_FAILURE:"+str(failure.get("reason")))
                raw=failure.get("native",{});unaccepted=raw.get("solver_candidate_q",raw.get("q"))
                if unaccepted is not None:
                    u=np.asarray(unaccepted,float)
                    if u.shape!=(6,) or not np.isfinite(u).all():raise MoveItUnavailable("INVALID_NATIVE_IK_RETURN")
                    if not any(same(u,x) for x in invalid):invalid.append(u.copy())
                if not pool and not raw.get("bounded_search",False) and unaccepted is None:
                    return [],None,dict(reason="SINGLE_TARGET_NO_NUMERICAL_PROGRESS",last_failure=failure),costs
            else:
                if any(same(q,x) for x in invalid):
                    return [],None,dict(reason="SINGLE_TARGET_NO_NUMERICAL_PROGRESS"),costs
                add(q)
            if direct and pool and not state.get("initial_comparison_done"):
                # A short numerical comparison at this SAME business pose. Do
                # not let the first wrist branch become the only candidate just
                # because its first path succeeds. No pose/roll/door scan.
                state["initial_comparison_done"]=True
                comparison_end=min(c._single_connection_deadline,
                    perf_counter()+min(.3,max(0.,c._single_connection_deadline-perf_counter())*.01))
                while perf_counter()<comparison_end:
                    before=perf_counter()
                    if before>=comparison_end:break
                    alternate,failed=c.ik_pose_once(pose,start,obstacles,seed,stage=stage,
                        attachment=None,purpose=purpose,residual_reason=residual_reason,
                        ik_mode="bounded_target",solver_budget_s=min(.10,comparison_end-before),
                        excluded_ik=[*invalid,*[p["q"] for p in pool]])
                    spent=perf_counter()-before;costs["ik_s"]+=spent;row["ik_s"]+=spent;guard()
                    row.setdefault("initial_ik_comparison",[]).append(dict(wall_s=spent,
                        status="INCOMPLETE" if failed else "QUALIFIED",failure=brief(failed)))
                    if failed:
                        if failed.get("reason") not in {"NATIVE_IK_FAILED",residual_reason}:
                            raise MoveItUnavailable("FIXED_TARGET_IK_REQUEST_FAILURE:"+str(failed.get("reason")))
                        break  # optional comparison made no new progress
                    count=len(pool);add(alternate)
                    if len(pool)==count:break
            eligible=[p for p in pool if not p["endpoint_invalid"]]
            if not eligible:continue
            # Fair accumulated solve time before light joint-travel ranking.
            # Returned prefixes are deferred after suffix failure, not erased.
            candidate=min(eligible,key=lambda p:(p["returned_prefixes"],round(p["motion_s"],1),p["joint_cost"]))
            q=candidate["q"]
            row.update(q_goal=q.tolist(),endpoint_joint_cost_rad=candidate["joint_cost"],
                valid_configuration_retained=True)
            before=perf_counter()
            path,failure,evidence=c._native_plan(start,q,obstacles,seed=seed,stage=stage,attachment=attachment,
                support_names=(),initial_proximity=None,purpose=purpose,allow_ompl=True,
                recovery_mode="shared_budget",search_deadline=min(c._single_connection_deadline,before+6.),
                try_ptp=direct,try_lin_pose=pose if direct else None,try_prior=False)
            elapsed=perf_counter()-before;row["motion_s"]=elapsed;costs["motion_s"]+=elapsed
            candidate["motion_s"]+=elapsed
            row.update(status="SUCCESS" if failure is None else "CONNECTION_NOT_COMPLETED",failure=brief(failure))
            guard()
            if failure is None:
                incumbent=(path,np.asarray(path[-1]).copy(),quality(path,evidence,elapsed))
                row["raw_candidate_quality"]=incumbent[2]
                # Improve only an actual detour, before any suffix is bound.
                # A legal incumbent survives numerical timeout or quality loss.
                m=incumbent[2]
                best_endpoint=min(p["joint_cost"] for p in eligible)
                detour=m and (m["weighted_joint_length_rad"]>1.4*max(min(m["endpoint_joint_distance_rad"],best_endpoint),1e-6)
                    or (m.get("physical_contact_length_over_chord") or 0)>1.6)
                remaining=c._single_connection_deadline-perf_counter()
                if direct and detour and remaining>2.:
                    limit=min(c._single_connection_deadline,perf_counter()+min(4.,remaining*.1))
                    # LIN may end at a different wrist branch than the chosen
                    # joint goal. Include that original qualified goal: a failed
                    # PTP does not mean a cheaper RRT route to it is impossible.
                    alternatives=sorted((p for p in eligible
                        if p["joint_cost"]<incumbent[2]["weighted_joint_length_rad"]-1e-8),
                        key=lambda p:p["joint_cost"])
                    for alternate in alternatives:
                        if perf_counter()>=limit:break
                        then=perf_counter()
                        try:
                            better,failed,ev=c._native_plan(start,alternate["q"],obstacles,seed=seed,stage=stage,
                                attachment=None,support_names=(),initial_proximity=None,purpose=purpose,
                                allow_ompl=True,recovery_mode="shared_budget",search_deadline=limit,
                                try_ptp=False,try_lin_pose=None,try_prior=False)
                        except MoveItUnavailable as exc:
                            if str(exc)=="SINGLE_TARGET_STAGE_DEADLINE" and not c._native_cancelled():
                                row["quality_stop"]="STAGE_DEADLINE_KEEP_INCUMBENT"
                                costs["motion_s"]+=perf_counter()-then
                                break
                            raise
                        spent=perf_counter()-then;costs["motion_s"]+=spent;alternate["motion_s"]+=spent
                        row.setdefault("quality_attempts",[]).append(dict(q=alternate["q"].tolist(),failure=brief(failed),wall_s=spent))
                        guard()
                        if failed:
                            if failed["reason"] not in {"DIRECT_CONNECTION_REJECTED","NATIVE_PLANNING_FAILED",
                                    "NATIVE_PLANNING_TIMEOUT","INVALID_GOAL_CLEARANCE","INVALID_GOAL_PROCESS",
                                    "MOVEIT2_SEARCH_EXHAUSTED","NATIVE_NO_SEARCH_PROGRESS"}:
                                raise MoveItUnavailable("QUALITY_REQUEST_FAILURE:"+failed["reason"])
                            continue
                        value=quality(better,ev,spent)
                        if value and value["weighted_joint_length_rad"]<incumbent[2]["weighted_joint_length_rad"]-1e-8:
                            incumbent=(better,np.asarray(better[-1]).copy(),value)
                            row["quality_stop"]="LOWER_JOINT_TRAVEL_NATIVE_PATH"
                            break
                row["accepted_quality"]=incumbent[2]
                row["incumbent_retained"]=incumbent[0] is path
                add(incumbent[1])
                selected=next((p for p in pool if same(p["q"],incumbent[1])),candidate)
                selected["returned_prefixes"]+=1
                return incumbent[0],incumbent[1],None,costs
            last_failure=failure
            if failure.get("reason") in {"INVALID_GOAL_CLEARANCE","INVALID_GOAL_PROCESS","INVALID_GOAL_AUTHORITY"}:
                invalid.append(q.copy());candidate["endpoint_invalid"]=True
            elif failure.get("reason")=="NATIVE_NO_SEARCH_PROGRESS":
                return [],None,dict(reason="SINGLE_TARGET_NO_NUMERICAL_PROGRESS",last_failure=failure),costs
            elif failure.get("reason") not in {"MOVEIT2_SEARCH_EXHAUSTED","NATIVE_PLANNING_FAILED","NATIVE_PLANNING_TIMEOUT","SEARCH_EXHAUSTED"}:
                raise MoveItUnavailable("FIXED_TARGET_CONNECTION_REQUEST_FAILURE:"+str(failure.get("reason")))
        return [],None,dict(reason="SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED",last_failure=last_failure,
            interpretation="INCOMPLETE_NOT_GLOBAL_UNREACHABILITY",attempts=attempt,
            retained_legal_configurations=sum(not p["endpoint_invalid"] for p in pool)),costs
    except MoveItUnavailable as exc:
        if str(exc) in {"SINGLE_TARGET_STAGE_DEADLINE","REQUEST_CANCELLED_OR_DEADLINE"} and not c._native_cancelled() and perf_counter()>=c._single_connection_deadline:
            return [],None,dict(reason="SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED",last_failure=last_failure,
                interpretation="INCOMPLETE_NOT_GLOBAL_UNREACHABILITY",attempts=attempt),costs
        raise
    finally:
        c._single_connection_deadline=previous
