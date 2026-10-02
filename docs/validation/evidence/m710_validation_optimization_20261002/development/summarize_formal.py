"""Read one downloaded formal directory; never run planning, curation or Isaac.

Usage: python .tmp/summarize_20261002.py FORMAL_DIR [--output NEW_SUMMARY.json]
Only explicit files under FORMAL_DIR are read. Missing stages remain null and
NOT_REACHED; partial/in-progress evidence is never upgraded to a final success.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def measured_sum(records, field):
    values = [row[field] for row in records if number(row.get(field))]
    return {"seconds": sum(values) if values else None,
            "records_with_measurement": len(values), "records_total": len(records)}


def stage_result(present, status=None, **fields):
    return {"status": (status or "RECORDED") if present else "NOT_REACHED", **fields}


def summarize(formal):
    inputs = {}

    def read(relative, *, lines=False, required=False):
        path = formal / relative
        if not path.is_file():
            if required:
                raise FileNotFoundError(path)
            inputs[relative] = {"present": False}
            return None
        raw = path.read_bytes()
        inputs[relative] = {"present": True, "sha256": hashlib.sha256(raw).hexdigest(),
                            "bytes": len(raw)}
        text = raw.decode("utf-8-sig")
        return ([json.loads(line) for line in text.splitlines() if line.strip()]
                if lines else json.loads(text))

    run = read("run.json", required=True)
    motion_doc = read("plan/motion.json")
    motion = motion_doc or {}
    delivery_doc = read("plan/delivery.json")
    delivery = delivery_doc or {}
    preflight_doc = read("plan/preflight.json")
    physics_doc = read("physics/result.json")
    physics_status = read("physics/run_status.json")
    requests = read("native-requests.jsonl", lines=True)
    failure = motion.get("native_failure_evidence")
    if isinstance(failure, dict):
        source = "motion.native_failure_evidence"
        stages, ik, audit = (failure.get("native_stages"), failure.get("native_ik"),
                             failure.get("audit") or {})
    else:
        source = "motion.native_backend_evidence" if "native_backend_evidence" in motion else None
        stages, ik, audit = (motion.get("native_backend_evidence"), motion.get("native_ik_evidence"),
                             motion.get("native_cold_audit") or {})
    stages_known, ik_known = isinstance(stages, list), isinstance(ik, list)
    stages, ik = stages or [], ik or []
    warnings = []
    totals = dict.fromkeys(("PTP", "LIN", "OMPL"), 0)
    per_stage = defaultdict(lambda: dict.fromkeys(("PTP", "LIN", "OMPL", "native_ik"), 0))
    counter_records = 0
    raw_stages = []
    for record in stages:
        counts = record.get("native_solver_calls")
        stage = record.get("stage", "unknown")
        if isinstance(counts, dict):
            counter_records += 1
            for solver, count in counts.items():
                if type(count) is not int or count < 0:
                    raise ValueError("Invalid raw native_solver_calls: " + repr(counts))
                totals[solver] = totals.get(solver, 0) + count
                per_stage[stage][solver] = per_stage[stage].get(solver, 0) + count
        else:
            warnings.append("Missing native_solver_calls for stage " + str(record.get("stage_id")))
        raw_stages.append({key: record.get(key) for key in (
            "request_id", "task_id", "stage_id", "parent_stage_id", "stage", "solver",
            "status", "native_output_status", "authoritative_status", "native_solver_calls",
            "mtc_plan_s", "native_output_check_s", "authoritative_s", "failure", "path_sha256")}
            | {"native_points": len(record["points"]) if isinstance(record.get("points"), list) else None})
    ik_calls = 0
    ik_counter_records = 0
    for record in ik:
        count = record.get("native_ik_calls")
        if type(count) is not int or count < 0:
            warnings.append("Missing/invalid native_ik_calls for " + str(record.get("request_id")))
            continue
        ik_counter_records += 1
        ik_calls += count
        per_stage[record.get("stage", "unknown")]["native_ik"] += count
    if audit.get("per_stage_calls") is not None and dict(per_stage) != audit["per_stage_calls"]:
        warnings.append("Raw-record per-stage counts differ from recorded cold audit; inspect both.")

    selected = motion.get("selected_trajectory_segment") or {}
    backend = selected.get("native_backend") or {}
    coverage = backend.get("source_coverage")
    task_audit = backend.get("mtc_task_audit")
    performance = motion.get("planning_performance") or {}
    final_seconds = performance.get("final_recheck_seconds")
    # The ordinary result defaults final_recheck_seconds to zero even if no
    # complete candidate was checked. Do not label that default a reached gate.
    final_reached = bool(selected or backend or (number(final_seconds) and final_seconds > 0))
    coverage_summary = None if coverage is None else {key: coverage.get(key) for key in (
        "passed", "task_id", "nonzero_edge_count", "covered_nonzero_edge_count", "uncovered_edges")}
    physical = physics_doc or {}
    export_reached = (delivery.get("export_seconds") is not None
                      or delivery.get("bundle_readback") is not None or delivery.get("bundle_path") is not None)
    request_counts = None if requests is None else dict(Counter(
        (row.get("stage") or "unknown") + ":" + (row.get("planner_id") or row.get("op") or "unknown")
        for row in requests))

    return {
        "schema": "m710_native_cold_offline_summary_v1", "scope": "read-only current formal output evidence",
        "formal_directory": str(formal), "status": run.get("status"),
        "identity": {key: run.get(key) for key in ("run_id", "world_session_id", "source_commit",
            "worker_sha256", "target", "carton_count", "random_seed")},
        "counts": {key: run.get(key) for key in ("world_launches", "planning_requests", "execution_requests")},
        "failure": {"supervisor": run.get("reason"), "planning_exception": motion.get("reason"),
            "planning_failure": motion.get("failure"),
            "complete_trajectory_failure_reason": motion.get("complete_trajectory_failure_reason")},
        "history": {key: audit.get(key, motion.get(key)) for key in (
            "history_enabled", "history_inputs_read", "legacy_motion_generator_calls")},
        "native_evidence_source": source,
        "actual_native_calls": {
            "returned_record_totals": totals if stages_known else None,
            "native_ik": ik_calls if ik_known else None,
            "per_stage": dict(per_stage) if stages_known or ik_known else None,
            "stage_records": len(stages) if stages_known else None,
            "stage_records_with_counters": counter_records if stages_known else None,
            "ik_records": len(ik) if ik_known else None,
            "ik_records_with_counters": ik_counter_records if ik_known else None,
            "note": "Only returned native_solver_calls/native_ik_calls are summed; requests are not solver calls."
        },
        "request_log": {"submitted_records": None if requests is None else len(requests),
            "by_stage_and_requested_planner": request_counts,
            "note": "Submission evidence only; an unanswered/rejected request cannot establish invocation."},
        "raw_stage_summary": raw_stages,
        "recorded_cold_audit_counts": audit.get("per_stage_calls"),
        "gates": {
            "complete_native_geometry": stage_result(motion_doc is not None,
                motion.get("complete_trajectory_status"), planning_success=motion.get("planning_success")),
            "complete_path_source_coverage": stage_result(coverage is not None,
                "PASS" if coverage and coverage.get("passed") is True else "FAIL", result=coverage_summary),
            "native_task_audit": stage_result(task_audit is not None,
                (task_audit or {}).get("status"), result=task_audit),
            "complete_path_final_check": stage_result(final_reached,
                result=None if not final_reached else {"reported_complete_status": motion.get("complete_trajectory_status")}),
            "execution_preflight": stage_result(preflight_doc is not None,
                (preflight_doc or {}).get("status"),
                simulation_execution_ready=(preflight_doc or {}).get("simulation_execution_ready")),
            "bundle_export_and_readback": stage_result(export_reached,
                (delivery.get("bundle_readback") or {}).get("status"),
                bundle_path=delivery.get("bundle_path"), readback=delivery.get("bundle_readback")),
            "isaac_result": stage_result(physics_doc is not None, physical.get("status"),
                workflow_cycle_completed=physical.get("workflow_cycle_completed"),
                physical_cycle_completed=physical.get("physical_cycle_completed"),
                qualification_passed=physical.get("qualification_passed"),
                runtime_stop_reason=physical.get("runtime_stop_reason"),
                execution_counts=physical.get("execution_counts"),
                assumed_reception_state=physical.get("assumed_reception_state"),
                actual_reception_succeeded=physical.get("actual_reception_succeeded")),
        },
        "physics_process_status": physics_status,
        "timings_seconds": {
            "supervisor_phase_wall": run.get("timings"), "supervisor_wall": run.get("wall_seconds"),
            "model_warmup": audit.get("model_warmup_s"),
            "native_ik_compute_sum": measured_sum(ik, "ik_s"),
            "native_ik_inclusive_sum": measured_sum(ik, "elapsed_s"),
            "native_mtc_plan_sum": measured_sum(stages, "mtc_plan_s"),
            "native_output_check_sum": measured_sum(stages, "native_output_check_s"),
            "independent_stage_check_sum": measured_sum(stages, "authoritative_s"),
            "complete_path_final_check": final_seconds if final_reached else None,
            "preflight": delivery.get("preflight_seconds"), "export": delivery.get("export_seconds"),
            "bundle_readback": delivery.get("bundle_readback_seconds"),
            "isaac_motion_physical": physical.get("replayed_simulation_seconds"),
            "isaac_replay_wall": physical.get("replay_wall_seconds"),
            "ordinary_planning_performance": performance or None,
            "note": "Nested/inclusive intervals must not be added; null means not reached or unmeasured, never zero cost."
        },
        "inputs": inputs, "warnings": warnings,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("formal_directory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    value = summarize(args.formal_directory.resolve())
    text = json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.output:
        # Never overwrite retained evidence or an earlier summary.
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(text)
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
