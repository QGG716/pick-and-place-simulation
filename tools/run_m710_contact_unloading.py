"""Server CPU entry for bounded planning and a replay-ready first action."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.layout_single_carton import (  # noqa: E402
    build_verified_motion_input, load_layout_motion_policy,
    run_layout_single_carton_audit, write_layout_single_carton_audit,
)
from unloading_sim.serial_unloading import apply_actual_motion_state  # noqa: E402
from unloading_sim.unloading_sequence import RowSequencePolicy, RowUnloadingState  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/validation/m710id70_layout_v1_single_carton.yaml")
    parser.add_argument("--output", default="outputs/m710_contact_unloading_round01")
    parser.add_argument("--actual-state", type=Path,
                        help="actual_remaining_state.json from the still-running physical world")
    parser.add_argument("--execution-config", type=Path,
                        help="optional execution policy paired with the motion --config")
    parser.add_argument("--execution-bundle", type=Path, metavar="JSON",
                        help="export a ready motion through scripts/export_isaac_fanuc_replay.py")
    args = parser.parse_args(argv)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    scene = None
    planning_policy = load_layout_motion_policy(args.config)
    strategy = planning_policy.data.get("search_strategy", {})
    row_state = RowUnloadingState(RowSequencePolicy(
        row_height_fraction=float(strategy.get("row_height_fraction", 0.05))))
    if args.actual_state is not None:
        actual_state = json.loads(args.actual_state.read_text(encoding="utf-8"))
        if not isinstance(actual_state, dict):
            raise ValueError("--actual-state must contain one actual-state JSON object")
        initial_scene = build_verified_motion_input(planning_policy)
        # Establish the full initial row widths before applying explicit
        # completed IDs; a new CLI process retains the original row centres.
        row_state.rank(initial_scene.cartons, support_graph=initial_scene.support_graph)
        scene = apply_actual_motion_state(initial_scene, actual_state, row_state=row_state)
        planning_policy = scene.policy
        (output / "actual_scene_snapshot.json").write_text(
            json.dumps(scene.snapshot, indent=2), encoding="utf-8")
    with (output / "planning_progress.jsonl").open("w", encoding="utf-8") as stream:
        def progress(item):
            record = {"elapsed_s": time.monotonic() - start, **dict(item)}
            stream.write(json.dumps(record, default=str) + "\n")
            stream.flush()
            summary = {key: record[key] for key in ("elapsed_s", "event", "stage", "success", "iterations", "target", "face", "roll", "pose", "valid_grasps", "trajectory_success") if key in record}
            failure = record.get("failure")
            summary["failure"] = failure.get("reason") if isinstance(failure, dict) else failure
            print(json.dumps(summary), flush=True)
        result = run_layout_single_carton_audit(
            planning_policy, progress_callback=progress, motion_input=scene, row_state=row_state)
    write_layout_single_carton_audit(result, output / "motion.json")
    print(json.dumps({"status": result["complete_trajectory_status"], "elapsed_s": time.monotonic() - start,
                      "statistics": result["statistics"]}), flush=True)
    if result["complete_trajectory_status"] == "PASS":
        from unloading_sim.m710_execution import build_m710_execution_preflight, write_m710_execution_preflight
        preflight = build_m710_execution_preflight(
            args.execution_config, motion_result=result, motion_input=scene)
        write_m710_execution_preflight(preflight, output / "preflight.json")
        print(json.dumps({"preflight": preflight["status"], "blockers": preflight["blockers"]}), flush=True)
        if not preflight.get("simulation_execution_ready", False):
            return 3
        if args.execution_bundle is not None:
            # Reuse the existing verified exporter and its exact bound inputs;
            # do not construct a second representation of the replay contract.
            child_env = os.environ.copy()
            python_path = child_env.get("PYTHONPATH", "")
            source_root = str(ROOT / "src")
            # Relative entries are interpreted by the child at cwd=ROOT.
            # Retain every caller entry and all unrelated environment values.
            existing_roots = {os.path.normcase(str((ROOT / entry).resolve()))
                              for entry in python_path.split(os.pathsep)}
            if os.path.normcase(source_root) not in existing_roots:
                child_env["PYTHONPATH"] = source_root + (os.pathsep + python_path if python_path else "")
            subprocess.run([
                sys.executable, str(ROOT / "scripts/export_isaac_fanuc_replay.py"),
                "--preflight", str((output / "preflight.json").resolve()),
                "--output", str(args.execution_bundle.resolve()),
            ], check=True, cwd=ROOT, env=child_env)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
