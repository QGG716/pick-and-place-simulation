import collections,json
from pathlib import Path
base=Path("/tmp/junction-hmin-20261020")
run=base/"width060";p=json.loads((run/"protocol.json").read_text())
r=json.loads((run/"seed-71070/result.json").read_text())
previous=json.loads((base/"formal-envfixed/seed-71070/result.json").read_text())
assert r["initial"]==previous["initial"]
rows=[];total=0.;math_ik=0;rejects=0
for a in r["attempts"]:
    if not a.get("folder"):continue
    folder=run/"seed-71070"/a["folder"]
    calls=json.loads((folder/"requests.json").read_text())
    trace=json.loads((folder/"search-trace.json").read_text())
    assert trace["model"]["identity"]["model_tool_fingerprint"]=="b910af57c0a7e9441a33c21c292a7407fa93574cab7bf67ca5589e41f16c836a"
    elapsed=sum(c.get("round_trip_s",0) for c in calls);total+=elapsed
    ik=sum(c.get("op")=="ik" and c.get("status")=="SUCCESS" for c in calls);math_ik+=ik
    rejected=sum(c.get("status")=="INVALID_GOAL_CLEARANCE" for c in calls);rejects+=rejected
    lifts=[c["response"]["external_axis"] for c in calls if c.get("response",{}).get("generation_source")=="NATIVE_EXTERNAL_AXIS" and c.get("status")=="SUCCESS"]
    pairs=collections.Counter(tuple(e["pair"]) for c in calls for e in c.get("response",{}).get("clearance",{}).get("rejected_examples",[]))
    rows.append(dict(folder=a["folder"],option=a["option"],complete=a["complete"],failure=a.get("failure"),
        wall_s=a["wall_s"],native_wall_s=elapsed,numerical_ik_successes=ik,invalid_goal_requests=rejected,
        native_coupled_samples=sum(len(l["checks"]) for l in lifts),
        rejected_pairs=[dict(pair=list(k),recorded_examples=v) for k,v in pairs.items()]))
summary=dict(source_commit=p["source_commit"],worker_sha256=p["worker_sha256"],authorized_geometry_change=p["authorized_geometry_change"],
    status=r["status"],completed=r["completed_count"],target_count=r["target_count"],
    initial_cartons=len(r["initial"]["world_cartons"]),terminal_cartons=len(r["terminal"]["world_cartons"]),
    lower_layer_preserved=r["lower_layer_preserved"],same_predecessor_as_070=True,
    planning_wall_s=r["planning_wall_s"],all_native_request_wall_s=total,numerical_ik_successes=math_ik,
    invalid_goal_requests=rejects,unsearched_height_sync_options=sum(a.get("unsearched_option_count",0) for a in r["attempts"]),
    attempts=rows,cycles=r["cycles"],physical_execution="NOT_RUN",execution_ready=False,
    qualification_status="NOT_EVALUATED",continuous_swept_proof=False)
(base/"width-summary.json").write_text(json.dumps(summary,indent=2))
print(json.dumps({k:summary[k] for k in ("status","completed","planning_wall_s","all_native_request_wall_s","numerical_ik_successes","invalid_goal_requests","unsearched_height_sync_options")}))
