"""Read the four archived diagnostics only; never plan or check geometry."""
from collections import Counter,defaultdict
from pathlib import Path
import csv,json,sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"tools"))
from summarize_m710_top_row_retest import load,summarize,unique_records,request_cost
base=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent
results={};request_rows=[]
for name in ("c04-A","c04-B","c02-A","c02-B"):
    folder=base/name;t=load(folder/"timing.json");box=t["boxes"][0]
    s=summarize(folder)
    motion,dups=unique_records(box["calls"]["stages"],("request_id",))
    if (folder/"requests.jsonl").exists():
        raw=[json.loads(l) for l in (folder/"requests.jsonl").read_text().splitlines()]
    else:
        import gzip
        with gzip.open(folder/"requests.jsonl.gz","rt") as f:raw=[json.loads(l) for l in f]
    raw,raw_dups=unique_records(raw,("request_id",));by_id={x["request_id"]:x for x in raw}
    methods=defaultdict(lambda:dict(calls=0,failed=0,round_trip_s=0.,failed_round_trip_s=0.))
    approach=[];before_graph=[];first_graph=None
    for record in motion:
        method=record.get("planner_id") or "ENDPOINT_CHECK_BEFORE_STAGE"
        cost=request_cost(record)
        item=dict(run=name,request_id=record["request_id"],stage=record["stage"],
            scope=record.get("connection_scope"),method=method,status=record["status"],
            rtt_s=record["request_round_trip_s"],**cost)
        usage=record.get("prior_usage") or {}
        item.update(edge_checks=usage.get("edge_checks"),state_checks=usage.get("state_checks"),
            graph_search_performed=usage.get("graph_search_performed"),
            direct_rejection_reused=usage.get("direct_rejection_reused"),
            solver_message=record.get("solver_message"),moveit_error_code=record.get("moveit_error_code"))
        ws=(record.get("clearance") or {}).get("workspace") or {}
        for field in ("aabb_tests","aabb_skips","narrow_calls","fixed_qualification_queries",
                      "fixed_aabb_qualifications","fixed_narrow_qualifications","fixed_aabb_reuses","fixed_narrow_reuses"):
            item[field]=sum(p[field] for p in ws.get("phases",{}).values()) if ws else None
        request_rows.append(item)
        key=record["stage"]+"/"+method
        m=methods[key];m["calls"]+=1;m["round_trip_s"]+=item["rtt_s"]
        if record["status"]!="SUCCESS":
            m["failed"]+=1;m["failed_round_trip_s"]+=item["rtt_s"]
        if record["stage"]=="pregrasp":
            approach.append(item)
            if method=="SPARSE_PRM" and first_graph is None:first_graph=record["request_id"]
            if first_graph is None and method=="CHECKED_JOINT_DIRECT":
                before_graph.append(by_id[record["request_id"]]["q_goal"])
    native_binary=t["environment"]["worker_sha256"]
    assert all(s["checks"][key] for key in ("boundary_and_target_contexts_consistent",
        "exact_cross_box_continuity","pair_profile_disabled_in_all_available_responses",
        "pair_times_null","process_preparsed_and_profile_disabled"))
    results[name]=dict(run_id=t["run_id"],source_commit=t["source_commit"],worker_sha256=native_binary,
        model_identity=t["model_identity"],prior=t["static_prior"],
        input_sha256=t["diagnostic_input_sha256"],input=t["diagnostic_input"],
        complete=box["complete"],plan_s=box["plan_s"],online_s=s["attempted_online_s"],
        initialization_s=t["initialization_s"],scene_update_s=box["scene_update_s"],
        checks=s["checks"],native_calls=box["calls"]["counts"],
        measured_clearance=s["measured_clearance"],stages=s["stages"],methods=dict(methods),
        first_pregrasp_graph_request=first_graph,
        direct_round_endpoints_before_graph=len(before_graph),
        distinct_direct_q_before_graph=len({tuple(q) for q in before_graph}),
        approach_sequence=approach,boundary_partition_s=s["boxes"][0]["boundary_partition_s"],
        failed_grasp_candidates=len(box["failed_candidates"]),
        rejected_motion_requests=sum(r["status"]!="SUCCESS" for r in motion),
        prior_timeouts=sum(r["status"]=="PRIOR_QUERY_TIMEOUT" for r in motion),
        duplicates_removed=dups+raw_dups,missing_clearance=s["missing_clearance_request_ids"])
for target in ("c04","c02"):
    a,b=results[target+"-A"],results[target+"-B"]
    assert a["worker_sha256"]==b["worker_sha256"] and a["model_identity"]==b["model_identity"]
    assert a["input_sha256"]==b["input_sha256"]
summary=dict(status="PLANNING_ONLY_NOT_EXECUTABLE",qualification_status="NOT_EVALUATED",
    continuous_swept_proof=False,run_order=["c04-A","c04-B","c02-B","c02-A"],
    formal_source_commit="f34443996148062afccbbe7d1f02154b39698b09",
    final_code_commit="3ec35410d98a821b0f722e24c124e3764ee882a7",
    final_code_task_ab_status="NOT_EVALUATED_AFTER_INVALID_GOAL_SCOPE_FIX",
    caveat="Four run limit used. Invalid goal candidate-scope regression fixed after A/B; no formal rerun.",
    results=results)
(base/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
with (base/"requests.csv").open("w",newline="",encoding="utf-8") as f:
    writer=csv.DictWriter(f,fieldnames=list(request_rows[0]));writer.writeheader();writer.writerows(request_rows)
fields=("run","complete","plan_s","online_s","initialization_s","scene_update_s",
        "direct_round_endpoints_before_graph","prior_timeouts","rejected_motion_requests")
with (base/"tasks.csv").open("w",newline="",encoding="utf-8") as f:
    writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
    for name,r in results.items():writer.writerow(dict(run=name,**{k:r[k] for k in fields[1:]}))
for name,r in results.items():
    print(name,json.dumps({k:r[k] for k in ("complete","plan_s","online_s","initialization_s",
        "scene_update_s","native_calls","direct_round_endpoints_before_graph","distinct_direct_q_before_graph",
        "prior_timeouts","measured_clearance","stages","methods")}))
