import collections,hashlib,json
from pathlib import Path
base=Path("/tmp/junction-hmin-20261020")
run=base/"formal-envfixed"
r=json.loads((run/"seed-71070/result.json").read_text())
p=json.loads((run/"protocol.json").read_text())
rows=[];calls=[]
for a in r["attempts"]:
    if a.get("folder"):
        folder=run/"seed-71070"/a["folder"]
        requests=json.loads((folder/"requests.json").read_text())
        calls.extend(requests)
        rows.append(dict(folder=a["folder"],option=a["option"],complete=a["complete"],failure=a.get("failure"),
            wall_s=a["wall_s"],request_wall_s=sum(x.get("round_trip_s",0) for x in requests),
            statuses=dict(collections.Counter(x.get("status","UNKNOWN") for x in requests))))
failed=json.loads((base/"formal/seed-71070/result.json").read_text())
cycles=[]
for c in r["cycles"]:
    cycles.append({k:c[k] for k in ("target_id","option","height_m","receive_sync_location",
        "actual_extraction_height_m","receive_policy_check","first_touch_tool_geometry","separation")})
    cycles[-1]["native_stages"]=[{k:s[k] for k in ("stage","stage_id","parent_stage_id","generation_source",
        "pipeline_id","planner_id","external_axis") if k in s} for s in c["stages"]]
    cycles[-1]["robot_trajectory_s"]=sum(s["points"][-1]["t"] for s in c["stages"] if s.get("generation_source") not in ("NATIVE_EXTERNAL_AXIS","SEMANTIC_EVENT"))
    cycles[-1]["drop_m"]=c["nominal_release"]["drop_m"]
    cycles[-1]["nominal_drop_s"]=c["nominal_release"]["samples"][-1]["t"]
    cycles[-1]["events"]=[e["event_type"] for e in c["events"]]
    cycles[-1]["commanded_cups"]=sum(c["attachment"]["selection"]["commanded_active_mask"])
summary=dict(source_commit=p["source_commit"],worker_sha256=p["worker_sha256"],
    status=r["status"],completed=r["completed_count"],targets=r["target_count"],
    initial_count=len(r["initial"]["world_cartons"]),final_count=len(r["terminal"]["world_cartons"]),
    historical_completed=r["historical_completed_count"],lower_preserved=r["lower_layer_preserved"],
    planning_wall_s=r["planning_wall_s"],all_native_request_wall_s=sum(x.get("round_trip_s",0) for x in calls),
    failed_initialization_wall_s=failed["planning_wall_s"],failed_initialization_attempts=failed["attempts"],
    attempts=rows,cycles=cycles,
    asset_root="/work-seam-approach-formal-v2-20261020",
    runtime_wrapper_sha256=hashlib.sha256((base/"worker-fixed.sh").read_bytes()).hexdigest(),
    qualification_status="NOT_EVALUATED",continuous_swept_proof=False,physical_execution="NOT_RUN",execution_ready=False)
(base/"metrics.json").write_text(json.dumps(summary,indent=2))
print(json.dumps({k:summary[k] for k in ("status","completed","initial_count","final_count","planning_wall_s","failed_initialization_wall_s")}))
for c in cycles:print(json.dumps({k:c[k] for k in ("option","receive_sync_location","actual_extraction_height_m","robot_trajectory_s","nominal_drop_s","drop_m","commanded_cups","first_touch_tool_geometry")}))
