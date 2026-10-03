#!/usr/bin/env python3
"""Read experiment evidence only; never imports planners or starts a worker."""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

MARKER = "PLANNING_ONLY_NOT_EXECUTABLE"
def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()

def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)

def read_json(path, warnings):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        warnings.append(dict(file=str(path), error=str(exc), action="SKIPPED_UNREADABLE_OR_IN_PROGRESS"))
        return None

def write_json(path, data):
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+"\n")
    temporary.replace(path)

def source_name(row):
    return row.get("generation_source") or ":".join(str(row.get(k) or "unspecified") for k in ("pipeline_id", "planner_id"))

def timeout(status):
    return "TIMEOUT" in str(status).upper() or "TIMED_OUT" in str(status).upper()

def summarize_stage_rows(stages, ik_rows, request_rows=()):
    by_id = {r.get("stage_id"): r for r in request_rows if r.get("stage_id")}
    groups = {}
    def get(stage):
        return groups.setdefault(stage or "UNSPECIFIED", dict(motion_requests=0,
            motion_statuses=Counter(), generation_sources=Counter(), native_calls=Counter(),
            native_solver_s=0., prior_query_s=0., motion_request_round_trip_s=0.,
            motion_round_trip_observed_requests=0, prior_queries=0, prior_successes=0,
            prior_hits=0, direct_short_connections=0, prior_misses=0, prior_timeouts=0,
            prior_edge_check_limits=0, edge_checks=0, rejected_edges=0, timed_out_edges=0,
            state_checks=0, local_repairs=0, ik_rows=0, ik_statuses=Counter(),
            ik_solver_s=0., ik_including_ipc_s=0., planning_phases=Counter()))
    for original in stages:
        # Request log fills timing absent from older compact stage records.
        # Never sum connection_elapsed_s: it can include preceding attempts.
        row = {**by_id.get(original.get("stage_id"), {}), **original}
        g = get(row.get("stage"))
        g["motion_requests"] += 1
        status = str(row.get("status"))
        g["motion_statuses"][status] += 1
        g["generation_sources"][source_name(row)] += 1
        g["planning_phases"][str(row.get("planning_phase", "UNRECORDED"))] += 1
        g["native_calls"].update(row.get("native_solver_calls") or {})
        value = row.get("native_solver_s")
        if not number(value):
            value = row.get("mtc_plan_s")
        if number(value):
            g["native_solver_s"] += value
        rt = row.get("request_round_trip_s")
        if not number(rt):
            rt = row.get("elapsed_s")
        if number(rt):
            g["motion_request_round_trip_s"] += rt
            g["motion_round_trip_observed_requests"] += 1
        usage = row.get("prior_usage") or {}
        prior = row.get("pipeline_id") == "static_prior" or bool(usage)
        if prior:
            g["prior_queries"] += 1
            success = status == "SUCCESS"
            g["prior_successes"] += success
            g["prior_misses"] += not success
            g["prior_hits"] += bool(success and usage.get("prior_hit"))
            g["direct_short_connections"] += bool(success and not usage.get("prior_hit")
                and usage.get("connector_kind") == "CHECKED_JOINT_INTERPOLATION")
            g["prior_timeouts"] += timeout(status)
            g["prior_edge_check_limits"] += status == "PRIOR_EDGE_CHECK_LIMIT"
            if number(row.get("prior_query_s")):
                g["prior_query_s"] += row["prior_query_s"]
            for key in ("edge_checks", "rejected_edges", "timed_out_edges", "state_checks", "local_repairs"):
                if number(usage.get(key)):
                    g[key] += usage[key]
    for row in ik_rows:
        g = get(row.get("stage"))
        g["ik_rows"] += 1
        g["ik_statuses"][str(row.get("status"))] += 1
        for output, keys in (("ik_solver_s", ("native_solver_s", "ik_s")),
                             ("ik_including_ipc_s", ("request_round_trip_s", "elapsed_s"))):
            value = next((row[k] for k in keys if number(row.get(k))), None)
            if value is not None:
                g[output] += value
    return groups

def box_geometry_binding(timing):
    identity = timing.get("model_identity") or {}
    diagnostic = timing.get("diagnostic_input") or {}
    removed = set(diagnostic.get("removed_targets") or [])
    remaining = [x for x in timing.get("initial_cartons", []) if x.get("id") not in removed]
    # The mode-sensitive model_tool_fingerprint is retained separately.
    # This hash binds the recorded common geometry/validator/TCP and task input.
    return dict(scope="RECORDED_GEOMETRY_POLICY_TCP_AND_DIAGNOSTIC_INPUT_NOT_BINARY_IDENTITY",
        diagnostic_input=diagnostic, remaining_cartons=remaining,
        initial_scene_fingerprint=timing.get("initial_scene_fingerprint"),
        validator_identity=identity.get("validator_identity"),
        policy_fingerprint=identity.get("policy_fingerprint"),
        task_tcp_fingerprint=identity.get("task_tcp_fingerprint"),
        flange_from_task_tcp=identity.get("flange_from_task_tcp"),
        tool_mass_kg=timing.get("tool_mass_kg"), payload_mass_kg=timing.get("payload_mass_kg"))

def requests(directory, warnings):
    path = directory/"requests.jsonl"
    if not path.exists():
        return []
    result = []
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            result.append(json.loads(line))
        except ValueError:
            warnings.append(dict(file=str(path), line=line_no, action="SKIPPED_PARTIAL_JSONL_LINE"))
    return result

def summarize_box(directory, timing, box, request_rows, formal=False):
    calls = box.get("calls") or {}
    rows = [r for r in request_rows if r.get("task_id") == box.get("task_id")]
    stage_groups = summarize_stage_rows(calls.get("stages", []), calls.get("ik", []), rows)
    ordering = box.get("candidate_ordering_s")
    plan = box.get("plan_s")
    comparable = plan if formal else (
        plan-(ordering if number(ordering) else 0.) if number(plan) else None)
    row = dict(group="FORMAL" if formal else "B", dataset=directory.name,
        case_id=box.get("target"), target=box.get("target"), order=box.get("order"),
        complete=box.get("complete", False), status="SUCCESS" if box.get("complete") else "INCOMPLETE",
        plan_s=plan, reported_plan_s=plan, candidate_ordering_s=ordering,
        candidate_ordering_recorded=number(ordering), comparable_s=comparable,
        time_basis="FULL_PLAN_INCLUDING_ORDERING" if formal else
            "PLAN_BOX_EXCLUDING_NEW_ORDERING_BASELINE_ORDERING_WAS_IN_SCENE_UPDATE",
        cumulative_plan_s=box.get("cumulative_plan_s"), scene_update_s=box.get("scene_update_s"),
        scene_cartons=box.get("scene_cartons"), counts=calls.get("counts") or {},
        stages=stage_groups, prior_query_s=calls.get("prior_query_s"),
        prior_hits=calls.get("prior_hits"), prior_misses=calls.get("prior_misses"),
        direct_short_connections=calls.get("direct_short_connections"),
        prior_nodes_reused=calls.get("prior_nodes_reused"), prior_edges_reused=calls.get("prior_edges_reused"),
        semantic_events=calls.get("semantic_events"), local_repairs=calls.get("local_repairs"),
        slow_ompl_calls=calls.get("slow_ompl_calls"),
        native_solver_s=calls.get("native_solver_s"), ik_solver_s=calls.get("ik_solver_s"),
        ik_including_ipc_s=calls.get("ik_including_ipc_s"),
        motion_request_round_trip_s=calls.get("motion_request_round_trip_s"),
        source_commit=timing.get("source_commit"), run_id=timing.get("run_id"),
        worker_sha256=(timing.get("environment") or {}).get("worker_sha256"),
        diagnostic_input_sha256=timing.get("diagnostic_input_sha256"),
        comparison_geometry_scope_sha256=digest(box_geometry_binding(timing)),
        comparison_geometry_scope=box_geometry_binding(timing)["scope"],
        model_tool_fingerprint=(timing.get("model_identity") or {}).get("model_tool_fingerprint"),
        source_file=str(directory.name+"/timing.json"),
        last_failure=box.get("last_failure"), initialization_s=timing.get("initialization_s"),
        static_prior=timing.get("static_prior"), budgets=timing.get("budgets"),
        fast_phase_budget_s=timing.get("fast_phase_budget_s"),
        completed_within_fast_phase=box.get("completed_within_fast_phase"),
        fast_phase_plan_s=box.get("fast_phase_plan_s"),
        slow_completion_plan_s=box.get("slow_completion_plan_s"))
    for key in ("prior_queries", "prior_timeouts", "prior_edge_check_limits",
                "edge_checks", "rejected_edges", "timed_out_edges", "state_checks"):
        row[key] = sum(s[key] for s in stage_groups.values())
    aggregate_counts = Counter()
    for stage in stage_groups.values():
        aggregate_counts.update(stage["native_calls"])
    row["native_count_consistency"] = {key: aggregate_counts.get(key, 0) == row["counts"].get(key, 0)
        for key in ("PTP", "LIN", "OMPL")}
    row["ik_record_count_matches_native_count"] = sum(s["ik_rows"] for s in stage_groups.values()) == row["counts"].get("IK", 0)
    row["motion_statuses"] = dict(sum((Counter(s["motion_statuses"]) for s in stage_groups.values()), Counter()))
    row["generation_sources"] = dict(sum((Counter(s["generation_sources"]) for s in stage_groups.values()), Counter()))
    row["failed_motion_requests"] = sum(n for status, n in row["motion_statuses"].items() if status != "SUCCESS")
    row["timeout_motion_requests"] = sum(n for status, n in row["motion_statuses"].items() if timeout(status))
    return row

def collect_a(root, warnings):
    result = []
    for file in sorted(root.glob("fixed*/comparison.json")):
        data = read_json(file, warnings)
        if not data:
            continue
        for source in data.get("rows", []):
            usage = source.get("prior_usage") or {}
            row = {k: source.get(k) for k in ("case_id", "target", "stage", "planning_mode", "status",
                "complete", "held_out", "perturbation", "connection_elapsed_s", "request_round_trip_s",
                "native_solver_s", "initialization_s", "static_prior", "prior_usage", "failure",
                "processing_branch", "geometry_input_sha256", "source_commit", "worker_sha256",
                "seed", "request_budget_s", "ipc_timeout_s", "protocol_failure")}
            row.update(group="A", dataset=file.parent.name, comparable_s=source.get("connection_elapsed_s"),
                time_basis="FIXED_START_GOAL_CONNECTION_WALL", counts=source.get("native_solver_calls") or {},
                source_file=str(file.relative_to(root)), prior_hits=int(bool(usage.get("prior_hit"))),
                prior_queries=int(source.get("planning_mode") == "static_prior_fast"),
                prior_misses=int(source.get("planning_mode") == "static_prior_fast" and source.get("status") != "SUCCESS"),
                prior_timeouts=int(source.get("planning_mode") == "static_prior_fast" and timeout(source.get("status"))),
                prior_edge_check_limits=int(source.get("status") == "PRIOR_EDGE_CHECK_LIMIT"))
            for key in ("edge_checks", "rejected_edges", "timed_out_edges", "state_checks", "local_repairs"):
                row[key] = usage.get(key)
            response_path = file.parent/str(source.get("case_id"))/str(source.get("planning_mode"))/"response.json"
            response = read_json(response_path, warnings) or {}
            row["prior_query_s"] = response.get("prior_query_s")
            row["generation_source"] = response.get("generation_source")
            row["returned_waypoint_count"] = response.get("returned_waypoint_count")
            row["stages"] = {source.get("stage", "UNSPECIFIED"): dict(
                request_round_trip_s=source.get("request_round_trip_s"),
                native_solver_s=source.get("native_solver_s"),
                prior_query_s=response.get("prior_query_s"),
                generation_source=response.get("generation_source"),
                counts=source.get("native_solver_calls") or {})}
            result.append(row)
    return result

def comparisons(rows):
    result = []
    for row in rows:
        if row["group"] == "A" and row.get("planning_mode") != "cold_from_scratch":
            base = next((r for r in rows if r["group"] == "A" and r.get("planning_mode") == "cold_from_scratch"
                and r["case_id"] == row["case_id"]), None)
            fields = ("geometry_input_sha256",)
        elif row["group"] == "B" and not row["dataset"].startswith("baseline-"):
            base = next((r for r in rows if r["group"] == "B" and r["dataset"].startswith("baseline-")
                and r["case_id"] == row["case_id"]), None)
            fields = ("diagnostic_input_sha256", "comparison_geometry_scope_sha256")
        else:
            continue
        if base is None:
            continue
        matches = {key: bool(base.get(key) and base.get(key) == row.get(key)) for key in fields}
        valid = all(matches.values()) and base.get("complete") is True and row.get("complete") is True
        valid = valid and number(base.get("comparable_s")) and number(row.get("comparable_s")) and row["comparable_s"] > 0
        ratio = base["comparable_s"]/row["comparable_s"] if valid else None
        result.append(dict(group=row["group"], case_id=row["case_id"], baseline_dataset=base["dataset"],
            variant_dataset=row["dataset"], baseline_status=base["status"], variant_status=row["status"],
            baseline_s=base.get("comparable_s"), variant_s=row.get("comparable_s"),
            time_basis=row["time_basis"], input_hash_matches=matches,
            ratio_baseline_over_variant=ratio,
            reduction_percent=(1.-row["comparable_s"]/base["comparable_s"])*100. if valid and base["comparable_s"]>0 else None,
            outcome=("IMPROVED" if ratio > 1 else "REGRESSED_OR_EQUAL") if valid else "NO_SUCCESS_SPEED_RATIO",
            held_out=row.get("held_out")))
    return result

def collect_builds(root, warnings):
    result = []
    for path in sorted(root.glob("static-prior*.build.json")):
        data = read_json(path, warnings)
        if not data:
            continue
        record = {key: data.get(key) for key in ("result", "generation_s", "initialization_s",
            "database_write_s", "load", "source_commit", "worker_sha256", "seed", "build_limits", "failure")}
        record["source_file"] = str(path.relative_to(root))
        record["modes"] = {}
        for name, m in (data.get("modes") or {}).items():
            raw_file = m.get("raw_response_file")
            raw = read_json(root/raw_file, warnings) if raw_file else None
            native = m.get("native") or raw or {}
            record["modes"][name] = dict(nodes=m.get("nodes", len(raw["nodes"]) if raw and "nodes" in raw else None),
                edges=m.get("edges", len(raw["edges"]) if raw and "edges" in raw else None),
                native_status=native.get("status"),
                generation_s=m.get("generation_s"), build_s=native.get("build_s"),
                accepted_portals=len(native["accepted_portals"]) if "accepted_portals" in native else None,
                state_checks=native.get("state_checks"), portal_attempts=native.get("portal_attempts"),
                context_equal_after_serialization=m.get("context_equal_after_serialization"))
        portals = data.get("layout_portals")
        if portals:
            record["layout_portals"] = {k: portals.get(k) for k in ("pose_count", "candidate_count",
                "native_ik_calls", "native_batch_calls", "wall_s")}
        result.append(record)
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    root, warnings = args.root.resolve(), []
    rows = collect_a(root, warnings)
    for directory in sorted(root.iterdir()):
        if directory.is_dir() and directory.name.startswith(("baseline-", "optimized-", "tuned-")):
            timing = read_json(directory/"timing.json", warnings)
            if timing:
                rr = requests(directory, warnings)
                rows.extend(summarize_box(directory, timing, box, rr) for box in timing.get("boxes", []))
    summary = dict(schema="m710_static_prior_comparison_summary_v1", status=MARKER,
        qualification_status="NOT_EVALUATED", generated_at_utc=datetime.now(timezone.utc).isoformat(),
        counts_and_timings_are_components_not_added_to_plan_total=True,
        preserves_failed_and_regressed_runs=True,
        timing_notes=[
            "A uses independently measured connection_elapsed_s at fixed start and goal.",
            "B subtracts recorded candidate_ordering_s only from the new plan_s; the old baseline measured ordering in scene_update_s.",
            "Missing baseline ordering is unrecorded, not asserted to cost zero.",
            "Formal T_plan_5 uses complete source plan_s including ordering; never subtract ordering.",
            "Native solver, IK, prior query and IPC times overlap the planning wall clock and are not additive.",
            "Source connection_elapsed_s inside per-request logs may be cumulative; stage aggregation uses request round trip only.",
            "Stage times aggregate recorded request/IK fields; they are not complete stage wall clocks and exclude unrecorded Python work.",
            "No ratio is produced for incomplete/failed or input-hash-mismatched pairs."],
        rows=rows, pair_comparisons=comparisons(rows), offline_builds=collect_builds(root, warnings),
        pending_expected=[name for name in ("tuned-c03", "tuned-c04", "formal-once") if not (root/name/"timing.json").exists()],
        warnings=warnings)
    write_json(root/"comparison-summary.json", summary)
    fields = ["group", "dataset", "case_id", "status", "complete", "held_out",
        "reported_plan_s", "candidate_ordering_s", "comparable_s", "time_basis",
        "PTP", "LIN", "OMPL", "IK", "prior_queries", "prior_hits", "prior_misses",
        "prior_timeouts", "prior_edge_check_limits", "prior_query_s", "edge_checks",
        "rejected_edges", "timed_out_edges", "state_checks", "native_solver_s",
        "ik_including_ipc_s", "failed_motion_requests", "timeout_motion_requests",
        "geometry_input_sha256", "diagnostic_input_sha256", "comparison_geometry_scope_sha256",
        "source_commit", "worker_sha256", "source_file", "baseline_s",
        "ratio_baseline_over_variant", "reduction_percent", "pair_input_hashes_match", "pair_outcome"]
    with (root/"comparison-summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            flat = {k: row.get(k) for k in fields}
            for key in ("PTP", "LIN", "OMPL", "IK"):
                flat[key] = row.get("counts", {}).get(key, 0)
            variant = ((row["group"] == "A" and row.get("planning_mode") != "cold_from_scratch")
                or (row["group"] == "B" and not row["dataset"].startswith("baseline-")))
            pair = next((x for x in summary["pair_comparisons"] if x["group"] == row["group"]
                and x["case_id"] == row["case_id"] and x["variant_dataset"] == row["dataset"]), None) if variant else None
            if pair:
                flat.update(baseline_s=pair["baseline_s"], ratio_baseline_over_variant=pair["ratio_baseline_over_variant"],
                    reduction_percent=pair["reduction_percent"], pair_input_hashes_match=all(pair["input_hash_matches"].values()),
                    pair_outcome=pair["outcome"])
            writer.writerow(flat)
    formal_dir = root/"formal-once"
    timing = read_json(formal_dir/"timing.json", warnings)
    formal = dict(schema="m710_formal_perbox_summary_v1", status=MARKER,
        qualification_status="NOT_EVALUATED", result="NOT_RUN_OR_PENDING", boxes=[],
        T_plan_5_s=None, T_online_5_s=None, completed_count=0,
        note="Formal planning time includes ordering and every failed attempt; no local comparison clock adjustment.")
    if timing:
        for key in ("run_id", "source_commit", "result", "completed_count", "completed_targets",
            "missing_targets", "T_plan_5_s", "T_online_5_s", "cumulative_attempted_plan_s",
            "initialization_s", "scene_update_s", "first_box_scene_preparation_s",
            "inter_box_scene_update_s", "batch_wall_s", "result_file_write_s",
            "batch_bookkeeping_s", "target_1s_achieved", "fast_phase_budget_s",
            "fast_phase_completed_count", "slow_completion_plan_s", "budgets",
            "static_prior", "seed", "fresh_five_box_batch"):
            formal[key] = timing.get(key)
        rr = requests(formal_dir, warnings)
        formal["boxes"] = [summarize_box(formal_dir, timing, box, rr, formal=True) for box in timing.get("boxes", [])]
        formal["sum_recorded_box_plan_s"] = sum(box.get("plan_s", 0.) for box in timing.get("boxes", []))
        formal["source_T_plan_matches_box_sum"] = (abs(formal["T_plan_5_s"]-formal["sum_recorded_box_plan_s"]) < 1e-8
            if number(formal.get("T_plan_5_s")) else None)
        formal["worker_sha256"] = (timing.get("environment") or {}).get("worker_sha256")
    write_json(root/"perbox-summary.json", formal)
    print(json.dumps(dict(rows=len(rows), pairs=len(summary["pair_comparisons"]),
        offline_builds=len(summary["offline_builds"]), formal_completed=formal.get("completed_count"),
        outputs=["comparison-summary.json", "comparison-summary.csv", "perbox-summary.json"],
        warnings=len(warnings))))

if __name__ == "__main__":
    main()
