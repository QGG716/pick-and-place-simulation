"""Derive compact c00 evidence from saved requests; never call a solver."""
import argparse
from collections import Counter, defaultdict
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
from unloading_sim.layout_single_carton import (
    load_layout_motion_policy, build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.upper_arm_preference import upper_arm

p=argparse.ArgumentParser()
p.add_argument("run",type=Path);p.add_argument("--source-root",type=Path,required=True)
p.add_argument("--output",type=Path,required=True)
a=p.parse_args()
read=lambda path:json.loads(path.read_text())
protocol=read(a.run/"protocol.json");result=read(a.run/"seed-71070/result.json")
layout=load_workcell_layout(a.source_root/protocol["active_layout_file"])
policy=load_layout_motion_policy(a.source_root/"configs/validation/m710id70_proof_of_concept.yaml")
policy=replace(policy,layout_validation=replace(policy.layout_validation,layout=layout))
scene=build_verified_motion_input(policy,a.source_root)
robot=_build_automatic_trajectory_connector(scene,layout.robot()).connector.robot
assert type(robot).__name__=="PinocchioHppFclBackend"
rows=[];total_calls=Counter();total_rtt=defaultdict(float);ik_cpu=0.
for row in result["attempts"]:
    if not row.get("folder"):continue
    folder=a.run/"seed-71070"/row["folder"]
    calls=read(folder/"requests.json");detail=read(folder/"attempt.json")
    connections=read(folder/"connection-attempts.json")
    trace=read(folder/"search-trace.json")
    trace_by={t["request"]["stage_id"]:t for t in trace["attempts"]}
    counts=Counter();rtt=defaultdict(float);iks=[];motions=[];witnesses=[];goal_passed=0
    for call in calls:
        key="/".join(str(call.get(k) or "-") for k in ("op","stage","planner_id","status"))
        counts[key]+=1;total_calls[key]+=1
        rtt[call["op"]]+=call["round_trip_s"];total_rtt[call["op"]]+=call["round_trip_s"]
        response=call.get("response",{})
        if call["op"]=="ik":
            ik_cpu+=response.get("actual_ik_s",0.)
            q=response.get("q")
            iks.append(dict(request_id=response.get("request_id"),status=call["status"],q=q,
                arm=None if q is None else upper_arm(robot,np.asarray(q)),wall_s=call["round_trip_s"],
                solver_s=response.get("actual_ik_s"),numerical_seed=response.get("numerical_ik_seed"),
                numerical_seed_index=response.get("numerical_seed_index"),
                start_q=response.get("real_start_q_unchanged"),
                position_error_m=response.get("position_error_m"),
                orientation_error_rad=response.get("orientation_error_rad")))
        if call["op"]=="plan":
            t=trace_by[call["stage_id"]]
            points=t.get("final_timed_path") or []
            endpoint=response.get("clearance",{}).get("counts",{}).get("endpoint",{})
            qualified_goal=bool(call.get("q_goal") is not None and call["pipeline_id"]!="external_axis"
                and endpoint.get("queries",0)>=2 and endpoint.get("rejected",1)==0)
            goal_passed+=qualified_goal and call["stage"]=="pregrasp"
            motions.append(dict(stage=call["stage"],stage_id=call["stage_id"],
                parent_stage_id=call["parent_stage_id"],planner_id=call["planner_id"],
                status=call["status"],wall_s=call["round_trip_s"],
                q_start=call["q_start"],q_goal=call.get("q_goal"),
                goal_arm=upper_arm(robot,np.asarray(call["q_goal"])) if call.get("q_goal") is not None else None,
                endpoint_goal_check_passed=qualified_goal,endpoint_check_counts=endpoint,
                final_q=points[-1]["q"] if points else None,
                native_duration_s=points[-1]["t"] if points else None,
                native_end_arm=upper_arm(robot,np.asarray(points[-1]["q"])) if points else None,
                actual_check_evidence=response.get("request_check_evidence"),
                solver_message=response.get("solver_message")))
            for w in response.get("clearance",{}).get("rejected_examples",[]):
                witnesses.append(dict(stage_id=call["stage_id"],stage=call["stage"],
                    request_id=response.get("request_id"),witness=w))
    expected=detail.get("expected_contact",{});actual=detail.get("terminal_contact")
    mask=lambda value,key:sum(value[key]) if value and value.get(key) is not None else None
    failure=row.get("failure")
    rows.append(dict(folder=row["folder"],complete=row["complete"],delta_m=row["option"]["world_y_offset_m"],
        temporary_height_m=row["option"]["H_temporary_m"],receive_height_m=row["option"]["H_receive_m"],
        synchronization=detail.get("receive_sync_location","NOT_REACHED"),
        wall_s=row["wall_s"],failure=failure.get("reason") if isinstance(failure,dict) else failure,
        counts=dict(counts),round_trip_s_by_op=dict(rtt),
        numerical_ik_success=sum(i["status"]=="SUCCESS" for i in iks),
        explicit_pregrasp_goal_check_passed_requests=goal_passed,
        successful_approach_requests=sum(m["stage"]=="pregrasp" and m["planner_id"]!="COUPLED_LIFT"
            and m["status"]=="SUCCESS" for m in motions),
        successful_contact_requests=sum(m["stage"]=="contact" and m["planner_id"]=="LIN"
            and m["status"]=="SUCCESS" for m in motions),
        predicted_eligible_count=mask(expected,"geometrically_eligible_mask"),
        commanded_count=mask(expected,"commanded_active_mask"),
        actual_contact_geometric_count=mask(actual,"actual_contact_mask"),
        expected_masks=expected,actual_contact=actual,
        ik_results=iks,motion_requests=motions,collision_witnesses=witnesses,
        selected_configurations=[{k:c[k] for k in ("connection_id","attempt","q_goal","upper_arm_preference",
            "status","ik_s","motion_s","failure") if k in c} for c in connections]))
c=result["cycles"][0];attachment=c["attachment"]
selected=next(row for row in rows if row["complete"])
contact_q=np.asarray(attachment["contact_q_rad"])
flange_from_box=np.asarray(attachment["flange_from_box"])
extraction=next(s for s in c["stages"] if s["stage"]=="extraction" and s["generation_source"]=="NATIVE_GENERATED")
box_poses=[robot.named_link_frames(np.asarray(p["q"]))["flange"]@flange_from_box for p in extraction["points"]]
y_error=max(abs(p[1,3]+.84) for p in box_poses)
assert y_error<1e-5
assert attachment["pose_continuity_max_abs"]<1e-8
assert c["third_sync_disabled"] and c["actual_extraction_height_m"]==1.75
assert all(len(attachment["selection"][key])==72 for key in (
    "geometrically_eligible_mask","commanded_active_mask","actual_contact_mask"))
assert selected["actual_contact_geometric_count"]==selected["commanded_count"]==18
metrics=dict(source_commit=protocol["source_commit"],backend=type(robot).__name__,
    fk_analysis_source="SAVED_CURRENT_RUN_Q_THROUGH_FORMAL_PRODUCTION_BACKEND_NO_SOLVER_CALLS",
    status=result["status"],completed_count=result["completed_count"],target_count=1,
    historical_completed_count=3,initial_cartons=37,terminal_cartons=36,
    planning_wall_s=result["planning_wall_s"],attempt_wall_s_total=sum(r["wall_s"] for r in rows),
    cost_accounting="Attempt wall and request/IK costs are nested, not additive to planning wall.",
    round_trip_s_by_op=dict(total_rtt),native_ik_solver_s=ik_cpu,request_counts=dict(total_calls),
    task_budget_s=300.,shared_prefix_budget_s=90.,
    dispatched_candidate_visits=len(rows),dispatched_distinct_deltas=sorted({r["delta_m"] for r in rows}),
    dispatched_distinct_heights=sorted({r["temporary_height_m"] for r in rows}),
    unvisited_defined_candidate_visits=64*185-len(rows),
    unsearched_note="Stopped at first complete candidate. Domain/preview counts are not planning attempts.",
    selected=dict(folder=selected["folder"],delta_m=selected["delta_m"],height_m=c["height_m"],
        synchronization=c["receive_sync_location"],contact_q_rad=contact_q.tolist(),
        contact_arm=upper_arm(robot,contact_q),actual_geometric_cups=18,
        holding_capacity_assumption=c["eccentric_contact_validation"]["suction"]["holding_capacity_assumption"],
        attachment_pose_continuity_max_abs=attachment["pose_continuity_max_abs"],
        box_y_max_error_m=y_error,release_box_pose_world=c["nominal_release"]["samples"][0]["box"]["pose"],
        nominal_drop_m=c["nominal_release"]["drop_m"],nominal_drop_s=c["nominal_release"]["samples"][-1]["t"],
        stage_duration_s=[dict(stage=s["stage"],planner=s["planner_id"],duration_s=s["points"][-1]["t"])
            for s in c["stages"] if s["generation_source"]=="NATIVE_GENERATED"]),
    attempts=rows,qualification_status="NOT_EVALUATED",continuous_swept_proof=False,
    physical_execution="NOT_RUN",execution_ready=False)
a.output.write_text(json.dumps(metrics,indent=2))
print(json.dumps({k:metrics[k] for k in ("completed_count","planning_wall_s","attempt_wall_s_total",
    "dispatched_candidate_visits","dispatched_distinct_deltas","dispatched_distinct_heights")}))
for row in rows:
    print(json.dumps({k:row[k] for k in ("folder","delta_m","temporary_height_m","wall_s",
        "numerical_ik_success","explicit_pregrasp_goal_check_passed_requests","successful_approach_requests",
        "successful_contact_requests","predicted_eligible_count","actual_contact_geometric_count")}))
