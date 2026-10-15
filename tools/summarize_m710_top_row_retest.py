"""Summarize existing planning-only responses; never solve or recheck geometry."""
import argparse
from collections import Counter, defaultdict
import csv
import gzip
import json
import math
from pathlib import Path


def unique_records(rows, keys):
    """Dedup exact repeated records only; conflicting snapshots require review."""
    seen = {}
    for row in rows:
        key = tuple(row[k] for k in keys)
        if key in seen and seen[key] != row:
            raise ValueError("conflicting records for " + repr(key))
        seen[key] = row
    return list(seen.values()), len(rows)-len(seen)


def load(path):
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    with gzip.open(str(path)+".gz", "rt", encoding="utf-8") as stream:
        return json.load(stream)


def summarize(folder):
    t, w = load(folder/"timing.json"), load(folder/"write_timing.json")
    archive = load(folder/"trajectories.json")
    per, requests, checks = [], [], {}
    missing_clearance, counts, stages = [], Counter(), defaultdict(Counter)
    seen_motion, seen_ik = set(), set()
    profile_flags, process_flags, null_flags = [], [], []
    duplicate_count = 0
    previous = t["initial_q_rad"]
    initial = {b["id"]: b for b in t["initial_cartons"]}
    removed = []
    for box in t["boxes"]:
        motion, duplicates = unique_records(box["calls"]["stages"], ("request_id",))
        ik, ik_duplicates = unique_records(box["calls"]["ik"], ("request_id", "batch_seed_index"))
        duplicate_count += duplicates + ik_duplicates
        assert not seen_motion.intersection(r["request_id"] for r in motion)
        seen_motion.update(r["request_id"] for r in motion)
        assert not seen_ik.intersection((r["request_id"], r["batch_seed_index"]) for r in ik)
        seen_ik.update((r["request_id"], r["batch_seed_index"]) for r in ik)
        batches = defaultdict(list)
        for r in ik:
            batches[r["request_id"]].append(r)
        # Backend charges the actual batch round trip to seed 0 only.
        for rows in batches.values():
            assert len(rows) == rows[0]["batch_consume_count"]
            assert sorted(r["batch_seed_index"] for r in rows) == list(range(len(rows)))
            assert all(r["request_round_trip_s"] == 0 for r in rows if r["batch_seed_index"] != 0)
        row = {k: box.get(k) for k in ("order", "target", "complete", "plan_s",
            "cumulative_plan_s", "scene_update_s", "start_q_rad", "end_q_rad", "task_id")}
        row.update(counts={k: sum(r.get("native_solver_calls", {}).get(k, 0) for r in motion)
                           for k in ("PTP", "LIN", "OMPL")})
        row["counts"]["IK"] = sum(r["native_ik_calls"] for r in ik)
        row.update(ik_requests=len(batches),
            motion_requests=len(motion),
            ik_failed_seeds=sum(r["status"] != "SUCCESS" for r in ik),
            ik_round_trip_s=sum(r["request_round_trip_s"] for r in ik),
            ik_solver_s=sum(r["ik_s"] for r in ik),
            motion_round_trip_s=sum(r["request_round_trip_s"] for r in motion),
            prior_query_s=sum(r.get("prior_query_s", 0) for r in motion),
            native_solver_s=sum(r.get("mtc_plan_s", 0) for r in motion),
            prior_hits=sum(r.get("generation_source") == "STATIC_PRIOR_REUSE" and r["status"] == "SUCCESS" for r in motion),
            direct_connections=sum(r.get("generation_source") == "NATIVE_DIRECT_CONNECTION" and r["status"] == "SUCCESS" for r in motion),
            prior_queries=sum(r.get("pipeline_id") == "static_prior" for r in motion),
            prior_failed_queries=sum(r.get("pipeline_id") == "static_prior" and r["status"] != "SUCCESS" for r in motion),
            prior_timeouts=sum(r["status"] == "PRIOR_QUERY_TIMEOUT" for r in motion),
            failed_candidates=len(box["failed_candidates"]),
            failures=[{k:r.get(k) for k in ("request_id", "stage", "status", "failure", "prior_usage")} for r in motion if r["status"] != "SUCCESS"])
        row["unattributed_remaining_s"] = row["plan_s"]-row["ik_round_trip_s"]-row["motion_round_trip_s"]
        row["cross_box_start_exact"] = box["start_q_rad"] == previous
        if box["complete"]:
            previous = box["end_q_rad"]
        for r in motion:
            clear = r.get("clearance")
            req = {k:r.get(k) for k in ("request_id", "stage_id", "parent_stage_id", "stage", "status",
                "request_round_trip_s", "prior_query_s", "mtc_plan_s", "pipeline_id")}
            req["target"] = box["target"]
            req["clearance_s"] = sum(p["seconds"] for p in clear["counts"].values()) if clear else None
            req["workspace_prepare_s"] = clear["workspace"]["prepare_s"] if clear else None
            req["clearance_queries"] = sum(p["queries"] for p in clear["counts"].values()) if clear else None
            requests.append(req)
            stages[r["stage"]]["requests"] += 1
            stages[r["stage"]]["request_round_trip_s"] += r["request_round_trip_s"]
            stages[r["stage"]]["prior_query_s"] += r.get("prior_query_s", 0)
            if not clear:
                missing_clearance.append(r["request_id"])
                continue
            # Every request owns a fresh Clearance; phases are disjoint counters.
            counts["measured_clearance_requests"] += 1
            counts["clearance_s"] += req["clearance_s"]
            counts["workspace_prepare_s"] += req["workspace_prepare_s"]
            counts["clearance_queries"] += req["clearance_queries"]
            stages[r["stage"]]["measured_clearance_s"] += req["clearance_s"]
            ws = clear["workspace"]
            profile_flags.append(ws["pair_profile_enabled"] is False and ws["pair_profile_status"] == "NOT_MEASURED")
            profile_flags.append(clear["native_check_reuse"] is True and clear["fixed_narrow_qualification"] is True and clear["mode"] == "optimized")
            pp = r["process_policy"]
            process_flags.append(pp["native_check_reuse"] is True and pp["profile_enabled"] is False and pp["seal_input_compilations"] == 1)
            counts["seal_input_compilations"] += pp["seal_input_compilations"]
            for p in ws["phases"].values():
                null_flags.append(all(p[k] is None for k in ("broad_phase_s", "narrow_s", "self_narrow_s", "world_narrow_s")))
                for k in ("aabb_tests", "aabb_skips", "narrow_calls", "fixed_qualification_queries",
                          "fixed_aabb_qualifications", "fixed_narrow_qualifications", "fixed_aabb_reuses", "fixed_narrow_reuses"):
                    counts[k] += p[k]
        per.append(row)
    assert len(archive["trajectories"]) == t["completed_count"]
    assert len({tr["task_id"] for tr in archive["trajectories"]}) == t["completed_count"]
    # Boundary/identity checks only. No interior sampling or source coverage scan.
    for box, tr in zip(t["boxes"], archive["trajectories"]):
        records = {r["stage_id"]:r for r in tr["native_stages"]}
        for e in tr["semantic_events"]:
            ev = e["event"]
            records[ev["stage_id"]] = dict(e, stage_id=ev["stage_id"], parent_stage_id=ev["parent_stage_id"], stage="place")
        cursor = tr["final_state_receipt"]["stage_id"]
        chain = []
        while cursor:
            assert cursor in records and cursor not in [r["stage_id"] for r in chain]
            chain.append(records[cursor]); cursor = records[cursor]["parent_stage_id"]
        chain.reverse()
        assert len(chain) == len(records)
        assert [r["stage"] for r in chain] == ["pregrasp", "contact", "extraction", "transit", "transit", "place", "withdrawal"]
        q = box["start_q_rad"]
        assert q == tr["path"][0]
        calls = {r["stage_id"]:r for r in box["calls"]["stages"]}
        for r in chain:
            assert r["stage_id"].startswith(tr["task_id"]+":")
            call = calls[r["stage_id"]]
            assert call["status"] == "SUCCESS" and call["process_policy"]["target_id"] == box["target"]
            assert call["parent_stage_id"] == r["parent_stage_id"]
            if r.get("points"):
                assert r["points"][0]["q"] == q
                q = r["points"][-1]["q"]
            else:
                assert r["terminal_q"] == q and r["event"]["target_id"] == box["target"]
        assert q == box["end_q_rad"] == tr["path"][-1]
        ctx = tr["final_state_receipt"]["context"]
        assert ctx["attached"] is False and ctx["stage"] == "withdrawal"
        assert ctx["target_id"] == box["target"] == ctx["target"]["id"]
        assert all(math.isclose(a, b, abs_tol=1e-10, rel_tol=0)
                   for ra, rb in zip(ctx["target"]["pose"], tr["place"]["actual_box_pose_world"])
                   for a, b in zip(ra, rb))
        assert box["scene_cartons"] == 40-len(removed)
        remaining = {b["id"]:b for b in ctx["other_world"] if b["category"] == "carton"}
        expected = {k:v for k,v in initial.items() if k not in removed+[box["target"]]}
        assert remaining == expected
        assert [e["event"] for e in tr["events"]] == ["ATTACH", "RELEASE", "RELEASE_RETREAT_COMPLETE"]
        assert all(r["process_policy"]["attached"] == (r["stage"] in ("extraction", "transit", "place"))
                   for r in box["calls"]["stages"] if r["status"] == "SUCCESS")
        removed.append(box["target"])
    checks.update(boundary_and_target_contexts_consistent=True,
        exact_cross_box_continuity=all(r["cross_box_start_exact"] for r in per),
        pair_profile_disabled_in_all_available_responses=bool(profile_flags) and all(profile_flags),
        pair_times_null=bool(null_flags) and all(null_flags),
        process_preparsed_and_profile_disabled=bool(process_flags) and all(process_flags),
        planning_only_isolation=t["execution_requests"] == t["isaac_launches"] == 0
            and not t["executable_bundle_generated"] and archive["execution_ready"] is False,
        no_history_or_legacy_calls=all(not b["calls"]["history_and_legacy"]["history_enabled"]
            and all(b["calls"]["history_and_legacy"][k] == 0 for k in ("history_inputs_read", "legacy_motion_generator_calls", "forbidden_entry_attempts")) for b in t["boxes"]))
    totals = {k:sum(r[k] for r in per) for k in ("ik_round_trip_s", "ik_solver_s", "motion_round_trip_s",
        "prior_query_s", "native_solver_s", "unattributed_remaining_s", "prior_queries", "prior_hits",
        "direct_connections", "prior_failed_queries", "prior_timeouts", "ik_requests", "motion_requests", "ik_failed_seeds")}
    totals["counts"] = {k:sum(r["counts"][k] for r in per) for k in ("IK", "PTP", "LIN", "OMPL")}
    return dict(status=t["status"], qualification_status="NOT_EVALUATED", continuous_swept_proof=False,
        run_id=t["run_id"], source_commit=t["source_commit"], completed_count=t["completed_count"],
        T_plan_5_s=t["T_plan_5_s"], T_online_5_s=w["T_online_5_s"],
        initialization_s=t["initialization_s"], inter_box_scene_update_s=t["inter_box_scene_update_s"],
        scene_update_s=t["scene_update_s"], result_file_write_s=w["result_file_write_s"],
        batch_wall_s=t["batch_wall_s"], batch_bookkeeping_s=w["batch_bookkeeping_s"],
        boxes=per, totals=totals, measured_clearance=dict(counts), stages=dict(stages),
        requests=requests, checks=checks, exact_duplicates_removed=duplicate_count,
        missing_clearance_request_ids=missing_clearance, full_clearance_total_s=None if missing_clearance else counts["clearance_s"],
        limitations=["Missing failure Clearance responses are unmeasured, not zero.",
            "Clearance and solver/prior timings are nested inside motion round trips; never add them.",
            "Workspace prepare excludes other unmeasured context preparation.",
            "No pair timings: cannot isolate traversal/AABB/FCL costs.",
            "Only endpoint/metadata review, not dense verification or execution qualification."])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("folder", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = summarize(args.folder)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/"summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    with (args.output/"boxes.csv").open("w", newline="", encoding="utf-8") as f:
        fields = list(result["boxes"][0])
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        writer.writerows({k:json.dumps(v, ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in row.items()} for row in result["boxes"])
    print(json.dumps({k:result[k] for k in ("completed_count", "T_plan_5_s", "T_online_5_s", "totals", "measured_clearance", "stages", "checks", "missing_clearance_request_ids")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
