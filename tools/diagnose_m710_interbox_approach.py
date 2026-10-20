"""Finite old-input PTP diagnostics, never a formal-batch solution source."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from unloading_sim.approach_quality import path_metrics
from unloading_sim.moveit2_backend import ResidentMoveItClient
from unloading_sim.workcell_layout import load_workcell_layout


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    robot = load_workcell_layout(args.layout).robot()
    rows = []
    for seed in (71070, 71071):
        base = args.run/f"seed-{seed}"
        previous = json.loads((base/"attempt-001/plan.json").read_text())
        for number in (2, 3):
            folder = base/f"attempt-{number:03d}"
            plan = json.loads((folder/"plan.json").read_text())
            trace = json.loads((folder/"search-trace.json").read_text())
            stage = next(s for s in plan["stages"] if s["stage"] == "pregrasp")
            original = next(a["request"] for a in trace["attempts"]
                if a["request"]["stage_id"] == stage["stage_id"])
            assert np.allclose(previous["end_q_rad"], original["q_start"], atol=1e-12, rtol=0)
            assert np.allclose(stage["points"][-1]["q"], original["q_goal"], atol=1e-8, rtol=0)
            requests = json.loads((folder/"requests.json").read_text())
            recorded = next(r for r in requests if r["stage_id"] == stage["stage_id"])
            model = deepcopy(trace["model"])
            request = deepcopy(original)
            label = f"{seed}-{previous['target_id']}-to-{plan['target_id']}"
            task = "diagnostic-"+label
            request.update(op="diagnostic_connection", task_id=task, stage_id=task+":ptp",
                parent_stage_id="", parent_state_receipt=None, pipeline_id="pilz_industrial_motion_planner",
                planner_id="PTP", allowed_planning_time_s=5.)
            contact = np.asarray(original["process_policy"]["flange_from_physical_contact"])
            client = ResidentMoveItClient(args.worker_command, log_path=str(args.output/(label+".log")))
            try:
                ready = client.request(model)
                assert ready["status"] == "READY", ready
                began = perf_counter()
                reply = client.request(request)
                elapsed = perf_counter()-began
            finally:
                client.close()
            raw = dict(scope="OLD_GEOMETRY_SAME_INPUT_DIAGNOSTIC_ONLY_NO_BATCH_CREDIT",
                source_plan=str(folder/"plan.json"), model=model, original_request=original,
                batch_root=plan["batch_root"], previous_final_state_receipt=previous["final_state_receipt"],
                old_stage=stage, original_planning_record=recorded, diagnostic_request=request,
                diagnostic_reply=reply, diagnostic_wall_s=elapsed)
            (args.output/(label+".json")).write_text(json.dumps(raw, separators=(",", ":"), allow_nan=False))
            row = dict(label=label, seed=seed, height_m=plan["batch_root"]["height_m"],
                old=path_metrics(robot, stage["points"], contact, recorded["round_trip_s"]),
                direct_status=reply.get("status"), direct_reason=reply.get("reason"),
                direct_wall_s=elapsed, direct_response_fields=list(reply))
            points = reply.get("points")
            if reply.get("status") == "SUCCESS" and points:
                row["direct"] = path_metrics(robot, points, contact, elapsed)
            rows.append(row)
            print(json.dumps(row), flush=True)
            previous = plan
    report = dict(rows=rows, qualification_status="NOT_EVALUATED", continuous_swept_proof=False,
        physical_execution="NOT_RUN", execution_ready=False)
    (args.output/"summary.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("--layout", type=Path, default=ROOT/"configs/workcells/m710id70_unloading_layout_v1.yaml")
    p.add_argument("--worker-command", required=True)
    p.add_argument("--output", type=Path, required=True)
    main(p.parse_args())
