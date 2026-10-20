"""Measured OFF/ON outcomes, including failed candidates and their cost."""
import argparse
from collections import Counter
import json
from pathlib import Path
from export_m710_approach_comparison import read


def summarize(off, on, presentation, output):
    protocols=[read(p/"protocol.json") for p in (off,on)]
    for key in ("source_commit","worker_sha256","configured_initial_q","original_cartons",
                "seeds","task_budget_s","stage_budget_s","connection_slice_s",
                "native_rrt_request_slice_s","approach_recovery_mode","ompl_attempt_quota"):
        assert protocols[0][key] == protocols[1][key], key
    assert not protocols[0]["direct_first"] and protocols[1]["direct_first"]
    rows=[]
    for run, mode in ((off,"OFF"),(on,"ON")):
        for seed in protocols[0]["seeds"]:
            folder=run/f"seed-{seed}"
            result=read(folder/"result.json")
            metrics=read(presentation/f"{mode.lower()}-{seed}.json")["attempts"]
            selected={a["stage_id"]: a for a in metrics if a["selected"]}
            calls=[]
            for attempt in result["attempts"]:
                if not attempt.get("folder"):
                    continue
                calls.extend(read(folder/attempt["folder"]/"requests.json"))
            transitions=[];previous="INITIAL"
            for cycle in result["cycles"]:
                stage=next(s for s in cycle["stages"] if s["stage"]=="pregrasp" and
                           s["generation_source"] not in ("NATIVE_EXTERNAL_AXIS","SEMANTIC_EVENT"))
                metric=selected[stage["stage_id"]]
                transitions.append(dict(previous=previous,target=cycle["target_id"],
                    option=cycle["option"],planner_id=stage["planner_id"],quality=metric["quality"]))
                previous=cycle["target_id"]
            plan_calls=[c for c in calls if c.get("op")=="plan" and c.get("stage")=="pregrasp"
                        and c.get("pipeline_id") in ("ompl","pilz_industrial_motion_planner")]
            native=sum(s["points"][-1]["t"] for c in result["cycles"] for s in c["stages"]
                       if s["generation_source"] not in ("NATIVE_EXTERNAL_AXIS","SEMANTIC_EVENT"))
            ext=[s for c in result["cycles"] for s in c["stages"] if s["generation_source"]=="NATIVE_EXTERNAL_AXIS"]
            rows.append(dict(mode=mode,seed=seed,status=result["status"],
                completed=result["completed_count"],target_count=result["target_count"],
                lower_layer_preserved=result["lower_layer_preserved"],
                remaining_cartons=len(result["terminal"]["world_cartons"]),
                planning_wall_s=result["planning_wall_s"],
                native_request_wall_s=sum(c["round_trip_s"] for c in calls),
                pregrasp_request_wall_s=sum(c["round_trip_s"] for c in calls if c.get("stage")=="pregrasp"),
                accepted_robot_motion_s=native,
                accepted_pregrasp_motion_s=sum(t["quality"]["motion_duration_s"] for t in transitions),
                accepted_pregrasp_joint_rad=sum(t["quality"]["weighted_joint_length_rad"] for t in transitions),
                accepted_pregrasp_contact_m=sum(t["quality"]["physical_contact_length_m"] for t in transitions),
                mechanism_demo_s=sum(.6 if s["external_axis"]["from_height_m"]==s["external_axis"]["to_height_m"] else 4. for s in ext),
                hardware_cycle_s=None,method_requests=dict(Counter(c["planner_id"] for c in plan_calls)),
                method_failures=dict(Counter(c["planner_id"] for c in plan_calls if c["status"]!="SUCCESS")),
                failed_process_attempts=[{k:a[k] for k in (
                    "folder","target_id","option","failure","classification","wall_s",
                    "unsearched_option_count") if k in a}
                    for a in result["attempts"] if not a.get("complete")],
                transitions=transitions))
    report=dict(source_commit=protocols[0]["source_commit"],worker_sha256=protocols[0]["worker_sha256"],
        shared_budget={k:protocols[0][k] for k in ("task_budget_s","stage_budget_s","ompl_attempt_quota")},
        scope="Two fixed seeds, independent continuous 40-carton worlds. ON also supplies final regression; no extra runs.",
        no_statistical_success_rate_claim=True,runs=rows,qualification_status="NOT_EVALUATED",
        continuous_swept_proof=False,physical_execution="NOT_RUN",execution_ready=False)
    output.write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps([{k:v for k,v in r.items() if k not in ("transitions","failed_process_attempts")} for r in rows]))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("off",type=Path);p.add_argument("on",type=Path)
    p.add_argument("--presentation",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();summarize(a.off,a.on,a.presentation,a.output)
