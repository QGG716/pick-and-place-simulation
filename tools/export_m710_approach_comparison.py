"""Display recorded approaches in their own world; never plan or alter paths."""
import argparse
from copy import deepcopy
import gzip
import json
from pathlib import Path
import sys
import numpy as np
from export_m710_search_replay import export_model
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from unloading_sim.approach_quality import path_metrics
from unloading_sim.workcell_layout import load_workcell_layout


def read(path):
    return json.loads(path.read_text() if path.exists() else gzip.decompress(
        path.with_suffix(path.suffix+".gz").read_bytes()))


def export(source, output, seed, old):
    layout = load_workcell_layout(ROOT/"configs/workcells"/(
        "m710id70_unloading_layout_v1_rear_fixed_20261019.yaml" if old else
        "m710id70_unloading_layout_v1.yaml"))
    robot = layout.robot()
    attempts, logs, summary = [], [], []
    if old:
        cases = []
        for p in sorted(source.glob(f"{seed}-*.json")):
            r = read(p)
            cases.extend([
                (p.stem+" OLD_ACCEPTED", r["original_request"],
                 r["original_planning_record"]["response"], r["old_stage"]["points"],
                 r["original_planning_record"]["round_trip_s"], True, r["model"], None),
                (p.stem+" PTP_DIAGNOSTIC", r["diagnostic_request"],
                 r["diagnostic_reply"], r["diagnostic_reply"].get("points", []),
                 r["diagnostic_wall_s"], False, r["model"], None)])
        source_commit = "OLD_ARCHIVED_GEOMETRY_DIAGNOSTIC_ONLY"
        complete = False
    else:
        protocol = read(source/"protocol.json")
        result = read(source/f"seed-{seed}"/"result.json")
        source_commit = protocol["source_commit"]
        complete = result["completed_count"] == result["target_count"]
        final_ids = {s["stage_id"] for c in result["cycles"] for s in c["stages"]}
        cases = []
        for a in result["attempts"]:
            if not a.get("folder"):
                continue
            folder = source/f"seed-{seed}"/a["folder"]
            if not (folder/"search-trace.json").exists() and not (folder/"search-trace.json.gz").exists():
                continue
            trace = read(folder/"search-trace.json")
            calls = {r["stage_id"]: r for r in read(folder/"requests.json") if r.get("stage_id")}
            for t in trace["attempts"]:
                req = t["request"]
                if (req["stage"] != "pregrasp" or req.get("op") != "plan" or
                        req.get("pipeline_id") not in ("ompl","pilz_industrial_motion_planner")):
                    continue
                call = calls[req["stage_id"]]
                selected = req["stage_id"] in final_ids
                cases.append((a["folder"]+" "+req["process_policy"]["target_id"], req,
                    call["response"], t["final_timed_path"], call["round_trip_s"],
                    selected, trace["model"], t.get("trace")))
    if not cases:
        raise ValueError("NO_RECORDED_APPROACHES")
    model = export_model(cases[0][6])
    for label, req, response, points, elapsed, selected, raw_model, trace in cases:
        # Cartons are removed between committed cycles, so scene identity changes.
        # The complete robot/tool description and every other model identity stay.
        assert raw_model["parameters"]["robot_description"] == cases[0][6]["parameters"]["robot_description"]
        assert {k: v for k, v in raw_model["identity"].items() if k != "scene_fingerprint"} == {
            k: v for k, v in cases[0][6]["identity"].items() if k != "scene_fingerprint"}
        points = points or []
        quality = path_metrics(robot, points, np.asarray(model["flange_from_contact"]), elapsed) if points else None
        status = response["status"]
        display = label+" · "+("已接受" if selected else
            "诊断候选" if old and status == "SUCCESS" else
            "原生候选，未最终接受" if status == "SUCCESS" else "失败")
        # Keep the authoritative trajectories and all diagnostic witnesses. Large
        # PlannerData trees stay in the lossless raw archive, not this HTML.
        compact_response = {k: response[k] for k in (
            "request_id","task_id","stage_id","parent_stage_id","status","reason","failure",
            "solver_message","moveit_error_code","pipeline_id","planner_id","native_solver_calls")
            if k in response}
        if response.get("clearance"):
            compact_response["clearance"]={k:response["clearance"][k] for k in
                ("counts","rejected_examples","search_gap_witness") if k in response["clearance"]}
        display_request={k:req[k] for k in (
            "op","identity","task_id","stage_id","parent_stage_id","stage","candidate_id",
            "scene_fingerprint","world","q_start","q_goal","goal_pose","attachment","pipeline_id",
            "planner_id","purpose","process_substage") if k in req}
        display_request["process_policy"]={"target_id":req["process_policy"]["target_id"]}
        entry = dict(request=display_request, response_identity={k: response.get(k) for k in
            ("request_id", "task_id", "stage_id", "parent_stage_id", "status", "solver_message")},
            final_timed_path=points, trace=None, external_axis=None, display_label=display,
            path_quality=quality, failure_witness=compact_response if status != "SUCCESS" else None,
            history_status="RECORDED_IN_RAW_ARCHIVE_NOT_EMBEDDED" if trace else
                "SEARCH_HISTORY_NOT_RECORDED_IN_DIAGNOSTIC_INPUT")
        attempts.append(entry)
        logs.append(dict(op="plan", response=compact_response, round_trip_s=elapsed))
        summary.append(dict(label=label, selected=selected, status=status,
            planner_id=req["planner_id"], stage_id=req["stage_id"],
            q_start=req["q_start"], q_goal=req.get("q_goal"),
            scene_fingerprint=req["scene_fingerprint"], quality=quality,
            failure=compact_response if status != "SUCCESS" else None))
    data = dict(label=f"{'旧几何同输入诊断' if old else source.name} · seed {seed}",
        raw=dict(run_id=f"{source.name}-{seed}", source_commit=source_commit, attempts=attempts),
        result=dict(complete=complete, diagnostic_only=old, target_id="顶层箱间接近"),
        model=model, request_log=logs,
        qualification_status="NOT_EVALUATED", continuous_swept_proof=False,
        physical_execution="NOT_RUN", execution_ready=False)
    data["display_projection"]="Actual worlds, q, paths and failure witnesses. Full requests/permissions/trees remain in raw archives."
    vendor = ROOT/"tools/search_replay/vendor"
    html = (ROOT/"tools/search_replay/viewer.html").read_text(encoding="utf-8")
    html = html.replace("/*__VENDOR__*/", (vendor/"three.min.js").read_text()+"\n"+
        (vendor/"OrbitControls.js").read_text()).replace("/*__DATA__*/",
        json.dumps(data, separators=(",", ":"), allow_nan=False).replace("</", "<\\/"))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    output.with_suffix(".json").write_text(json.dumps(dict(source_commit=source_commit,
        seed=seed, diagnostic_only=old, attempts=summary), indent=2), encoding="utf-8")
    print(json.dumps(dict(output=str(output), attempts=len(attempts))))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", type=Path)
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--old-diagnostics", action="store_true")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    export(a.source, a.output, a.seed, a.old_diagnostics)
