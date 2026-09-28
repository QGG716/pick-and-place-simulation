"""Offline analysis of preserved telemetry; never rewrites the historical run."""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from unloading_sim.joint_effort import JointEffortMonitor, projected_source


def analyze(history: Path) -> dict:
    trial = history / "trial/isaac_after_import_fix"
    csv_path = trial / "joint_tracking.csv.gz"
    summary_path = history / "physical_trial_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))

    # Read the actual replay bundle, not duplicated nominal limits.
    bundle_path = history / "trial/execution/replay_bundle.json"
    bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
    metadata = bundle["metadata"]
    names = list(metadata["joint_names"])
    limits = metadata["joint_effort_limits_nm"]
    monitor = JointEffortMonitor(names, limits, f"{bundle_path}:metadata.joint_effort_limits_nm", 1/240,
        projected_source(isaac_version="6.0.1.0", tensor_version="110.1.13", backend="CPU_PhysX"))
    prefixes = ["drive_input_effort", "projected_constraint_reaction", "model_inverse_dynamics",
                "external_joint_load_residual", "gravity_compensation"]
    stats = {p: {n: dict(finite_samples=0, missing_samples=0, peak_abs_nm=None,
                        first_valid_time_s=None, last_valid_time_s=None) for n in names} for p in prefixes}
    raw_sha = hashlib.sha256()
    with gzip.open(csv_path, "rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            raw_sha.update(block)
    first_time = last_time = None
    maximum_grid_error = 0.
    with gzip.open(csv_path, "rt") as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames
        for step, row in enumerate(reader, 1):
            t = float(row["time_s"])
            if first_time is None:
                first_time = t
            last_time = t
            maximum_grid_error = max(maximum_grid_error, abs(t-step/240))
            for prefix in prefixes:
                for name in names:
                    value = float(row[f"{prefix}_{name}_nm"])
                    stat = stats[prefix][name]
                    if math.isfinite(value):
                        stat["finite_samples"] += 1
                        stat["peak_abs_nm"] = max(abs(value), stat["peak_abs_nm"] or 0.)
                        if stat["first_valid_time_s"] is None:
                            stat["first_valid_time_s"] = t
                        stat["last_valid_time_s"] = t
                    else:
                        stat["missing_samples"] += 1
            monitor.observe(joint_names=names, physics_step=step, simulation_time=t,
                observation_phase="post_physics_step",
                raw_values=[float(row[f"projected_constraint_reaction_{n}_nm"]) for n in names],
                applicability={"zero_explicit_effort": all(float(row[f"drive_input_effort_{n}_nm"]) == 0 for n in names)})
    sources = [csv_path, summary_path, bundle_path, trial / "runtime_feedback.json"]
    return dict(schema="saved_joint_effort_analysis_v1", analysis_mode="OFFLINE_REANALYSIS_NOT_HISTORICAL_RUNTIME_MONITOR",
        sources=[dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sources],
        decompressed_csv_sha256=raw_sha.hexdigest(), csv_header=header,
        joint_names=names, physics_steps=monitor.samples, first_time_s=first_time, last_time_s=last_time,
        maximum_grid_error_s=maximum_grid_error,
        physics_step_origin="inferred from ordered CSV rows and verified 240Hz time grid; not originally a CSV column",
        observation_phase_basis="archived adapter reads after world.step; simulation_time=(step+1)*physics_dt",
        channels=stats, effort_evidence=monitor.summary(summary["physics_steps"]),
        projected_diagnostic_peak_ratios=[stats["projected_constraint_reaction"][n]["peak_abs_nm"]/limit for n,limit in zip(names,limits)],
        projected_diagnostic_is_drive_qualification=False,
        inverse_dynamics_missing_reason="first post-step sample has no previous velocity; subsequent samples retained",
        external_residual_missing_reason="deliberately unavailable; no calibrated subtraction identifying external load",
        historical_conclusions={key: summary[key] for key in ["workflow_cycle_completed", "physical_cycle_completed",
            "qualification_passed", "new_target_execution_counts", "runtime_stop_reason"]},
        qualification_measured_failures=[], qualification_not_evaluated=["joint_efforts_within_limit"],
        missing_evidence=["per-step separated implicit drive output or validated multi-DOF decomposition",
                          "historical joint friction/limit contribution readback and applicability",
                          "historical incoming 6D wrench and external attachment reaction decomposition"],
        no_measured_overload_conclusion=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--history", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("refusing to overwrite evidence")
    report = analyze(args.history)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
