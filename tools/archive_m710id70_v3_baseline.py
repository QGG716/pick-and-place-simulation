"""Create a lightweight, reviewable archive from two V3 fixed-grid runs."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any


COMPARISON_FIELDS = (
    "GRASP_REACHABLE",
    "EXTRACTION_FEASIBLE",
    "GEOMETRICALLY_REACHABLE",
    "failure_stage",
    "failure_reason",
    "face",
    "roll_deg",
)
STAGES = ("approach", "contact", "handoff", "carry", "place", "withdrawal")


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_digest(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def compare_rows(before: list[dict[str, str]], after: list[dict[str, str]]) -> list[dict[str, Any]]:
    before_by_id = {row["task_id"]: row for row in before}
    after_by_id = {row["task_id"]: row for row in after}
    if set(before_by_id) != set(after_by_id):
        missing_after = sorted(set(before_by_id) - set(after_by_id))
        missing_before = sorted(set(after_by_id) - set(before_by_id))
        raise ValueError(f"task sets differ: missing_after={missing_after}, missing_before={missing_before}")
    rows = []
    for task_id in sorted(before_by_id):
        old, new = before_by_id[task_id], after_by_id[task_id]
        row: dict[str, Any] = {"task_id": task_id}
        for field in COMPARISON_FIELDS:
            row[f"before_{field}"] = old.get(field, "")
            row[f"after_{field}"] = new.get(field, "")
        old_success = old["GEOMETRICALLY_REACHABLE"] == "True"
        new_success = new["GEOMETRICALLY_REACHABLE"] == "True"
        row["success_transition"] = (
            "NEW_SUCCESS" if new_success and not old_success else
            "LOST_SUCCESS" if old_success and not new_success else
            "COMMON_SUCCESS" if old_success else "COMMON_FAILURE"
        )
        row["review_fields_equal"] = all(old.get(field, "") == new.get(field, "")
                                                 for field in COMPARISON_FIELDS)
        rows.append(row)
    return rows


def stage_statistics(task_dir: Path) -> dict[str, Any]:
    task_failure_ids: dict[str, list[str]] = defaultdict(list)
    candidate_failure_counts = Counter()
    candidate_failure_task_ids: dict[str, set[str]] = defaultdict(set)
    grasp_not_extracted = []
    extracted_not_complete = []
    escape_validations = 0
    observed_grasp_ik_calls = 0
    observed_grasp_ik_diagnostics = 0

    task_paths = sorted(task_dir.glob("grid_*_fixed.json"))
    for path in task_paths:
        envelope = read_json(path)
        result = envelope["result"]
        task_id = path.stem.removesuffix("_fixed")
        if result["grasp_reachable"] and not result["extraction_feasible"]:
            grasp_not_extracted.append(task_id)
        if result["extraction_feasible"] and not result["geometric_feasible"]:
            extracted_not_complete.append(task_id)
        if not result["geometric_feasible"]:
            task_failure_ids[result["failure_stage"]].append(task_id)
        for attempt in result["attempts"]:
            if "ik" in attempt:
                observed_grasp_ik_calls += 1
                observed_grasp_ik_diagnostics += int("unconstrained_diagnostic" in attempt["ik"])
            for sub in attempt.get("conveyor_attempts", []):
                escape_validations += len(sub.get("escape_attempts", []))
                stage = sub.get("stage")
                if stage in STAGES and sub.get("reason") != "OK":
                    candidate_failure_counts[stage] += 1
                    candidate_failure_task_ids[stage].add(task_id)

    return {
        "schema_version": "m710_v3_search_statistics_v1_observed_legacy",
        "task_count": len(task_paths),
        "task_level": {
            "grasp_valid_but_not_extracted_count": len(grasp_not_extracted),
            "grasp_valid_but_not_extracted_task_ids": grasp_not_extracted,
            "extracted_but_not_complete_count": len(extracted_not_complete),
            "extracted_but_not_complete_task_ids": extracted_not_complete,
            "final_failure_stage_counts": {key: len(value) for key, value in sorted(task_failure_ids.items())},
            "final_failure_stage_task_ids": {key: value for key, value in sorted(task_failure_ids.items())},
        },
        "candidate_level": {
            "failure_counts_by_stage": dict(candidate_failure_counts),
            "failure_task_ids_by_stage": {
                key: sorted(value) for key, value in sorted(candidate_failure_task_ids.items())
            },
            "escape_robot_validations_all_attempts": escape_validations,
            "observed_grasp_ik_calls": observed_grasp_ik_calls,
            "observed_grasp_unconstrained_diagnostic_calls": observed_grasp_ik_diagnostics,
        },
        "legacy_evidence_limitations": [
            "pregrasp and handoff IK calls were not recorded by the reviewed result schema",
            "per-seed IK iterations and RRT state/edge validation counts were not recorded",
            "selected escape_robot_validations in compact CSV is not a task total",
        ],
    }


def representative_task(task_path: Path) -> dict[str, Any]:
    envelope = read_json(task_path)
    result = envelope["result"]
    selected = dict(result["selected"])
    trajectory = dict(selected["trajectory"])
    trajectory.pop("samples", None)
    selected["trajectory"] = trajectory
    return {
        "cache_schema_version": envelope.get("cache_schema_version"),
        "input_digest": envelope["input_digest"],
        "inputs": envelope["inputs"],
        "validation_evidence": result.get("validation_evidence"),
        "task": {
            "box": result["box"],
            "seed": result["seed"],
            "mode": result["mode"],
            "initial_box_pose": result["initial_box_pose"],
            "initial_q": result["initial_q"],
            "failure_stage": result["failure_stage"],
            "failure_reason": result["failure_reason"],
            "selected": selected,
        },
        "revalidation": {
            "command": ".venv\\Scripts\\python.exe -m pytest -q --basetemp .tmp\\pytest-slow-evidence tests\\test_v3_motion_contract.py -m slow",
            "test": "test_original_grid_022_top_escape_is_a_reproducible_full_geometry_witness",
        },
    }


def archive(baseline_dir: Path, after_dir: Path, output_dir: Path) -> None:
    before_csv = baseline_dir / "fixed_task_reachability.csv"
    after_csv = after_dir / "fixed_task_reachability.csv"
    before = read_csv(before_csv)
    after = read_csv(after_csv)
    comparison = compare_rows(before, after)
    if len(comparison) != 104:
        raise ValueError(f"expected 104 fixed-grid tasks, found {len(comparison)}")

    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(before_csv, output_dir / "before_fixed_task_reachability.csv")
    shutil.copyfile(after_csv, output_dir / "after_fixed_task_reachability.csv")
    shutil.copyfile(after_dir / "effective_config.json", output_dir / "effective_config.json")
    shutil.copyfile(after_dir / "source_manifest.json", output_dir / "source_manifest.json")
    write_csv(output_dir / "task_comparison.csv", comparison)

    transitions = Counter(row["success_transition"] for row in comparison)
    successes = [row["task_id"] for row in comparison
                 if row["after_GEOMETRICALLY_REACHABLE"] == "True"]
    write_json(output_dir / "comparison_summary.json", {
        "denominator": len(comparison),
        "success_transition_counts": dict(transitions),
        "success_task_ids": successes,
        "review_fields": list(COMPARISON_FIELDS),
        "all_review_fields_equal": all(row["review_fields_equal"] for row in comparison),
        "normalized_review_digest_sha256": canonical_digest([
            {field: row[field] if field == "task_id" else row[f"after_{field}"]
             for field in ("task_id", *COMPARISON_FIELDS)}
            for row in comparison
        ]),
    })
    write_json(output_dir / "legacy_search_statistics.json", stage_statistics(after_dir / "tasks"))
    witness_id = successes[0] if successes else None
    if witness_id is not None:
        write_json(
            output_dir / f"representative_success_{witness_id}.json",
            representative_task(after_dir / "tasks" / f"{witness_id}_fixed.json"),
        )

    example = read_json(after_dir / "tasks" / "grid_020_fixed.json")
    result_evidence = example["result"]["validation_evidence"]
    manifest = {
        "schema_version": "m710_v3_lightweight_baseline_archive_v1",
        "baseline_source": str(baseline_dir),
        "after_source": str(after_dir),
        "task_set": "fixed conveyor original 104 valid grid tasks",
        "commands": {
            "baseline": ".venv\\Scripts\\python.exe tools\\run_m710id70_v3.py --phase grid-fixed --workers 12 --output-dir .tmp\\review_c6fa455_baseline",
            "after": ".venv\\Scripts\\python.exe tools\\run_m710id70_v3.py --phase grid-fixed --workers 12 --output-dir .tmp\\review_hardened_post",
            "archive": ".venv\\Scripts\\python.exe tools\\archive_m710id70_v3_baseline.py --baseline-dir .tmp\\review_c6fa455_baseline --after-dir .tmp\\review_hardened_post --output-dir docs\\validation\\evidence\\m710id70_v3_hardening_baseline",
        },
        "calculation_identity": result_evidence,
        "asset_manifest": read_json(after_dir / "effective_config.json")["model_asset_manifest"],
        "runtime_versions": read_json(after_dir / "effective_config.json")["runtime_versions"],
        "seed": read_json(after_dir / "effective_config.json")["effective_config"]["planning"]["seed"],
        "files": {},
    }
    for path in sorted(output_dir.iterdir()):
        if path.is_file() and path.name != "archive_manifest.json":
            manifest["files"][path.name] = {"sha256": sha256_file(path), "bytes": path.stat().st_size}
    write_json(output_dir / "archive_manifest.json", manifest)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-dir", type=Path, required=True)
    parser.add_argument("--after-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    archive(args.baseline_dir.resolve(), args.after_dir.resolve(), args.output_dir.resolve())


if __name__ == "__main__":
    main()
