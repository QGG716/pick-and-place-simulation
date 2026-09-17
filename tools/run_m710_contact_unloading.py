"""Server CPU entry for bounded planning and a replay-ready first action."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import copy
import signal
import shutil
from dataclasses import replace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.layout_single_carton import (  # noqa: E402
    build_verified_motion_input, load_layout_motion_policy,
    run_layout_single_carton_audit, write_layout_single_carton_audit,
)
from unloading_sim.planning_profile import DEFAULT_MOTION, DEFAULT_EXECUTION, POC, profile_evidence
from unloading_sim.serial_unloading import apply_actual_motion_state  # noqa: E402
from unloading_sim.unloading_sequence import RowSequencePolicy, RowUnloadingState  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_MOTION)
    parser.add_argument("--output", default="outputs/m710_contact_unloading_round01")
    parser.add_argument("--approach-mode", choices=("auto", "direct", "adaptive_pregrasp"))
    parser.add_argument("--planning-wall-time-s", type=float)
    parser.add_argument("--history-source", type=Path, help="controlled nonrecursive directory of JSON motion hints")
    parser.add_argument("--reuse-motion", type=Path, help="compatibility alias for one historical hint; same planner and budget")
    parser.add_argument("--actual-state", type=Path,
                        help="actual_remaining_state.json; archived snapshots support offline planning only")
    parser.add_argument("--execution-config", type=Path,
                        help="optional execution policy paired with the motion --config")
    parser.add_argument("--execution-bundle", type=Path, metavar="JSON",
                        help="export a ready motion through scripts/export_isaac_fanuc_replay.py")
    args = parser.parse_args(argv)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if (output / "motion.json").exists() or (output / "first_feasible").exists():
        raise FileExistsError("use a new output directory; existing planning evidence is immutable")
    start = time.monotonic()
    def cancelled(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")
    signal.signal(signal.SIGTERM, cancelled)
    scene = None
    planning_policy = load_layout_motion_policy(args.config)
    if args.execution_config is None and profile_evidence(planning_policy.data)["name"] == POC:
        args.execution_config = ROOT / DEFAULT_EXECUTION
    if args.history_source is not None and args.reuse_motion is not None:
        raise ValueError("choose one history source")
    if (args.approach_mode is not None or args.planning_wall_time_s is not None
            or args.history_source is not None or args.reuse_motion is not None):
        data = copy.deepcopy(planning_policy.data)
        if args.history_source is not None or args.reuse_motion is not None:
            data["search_strategy"]["history"] = {**data["search_strategy"].get("history", {}),
                "source": str((args.history_source or args.reuse_motion).resolve())}
        if args.approach_mode is not None:
            data["search_strategy"]["approach_mode"] = args.approach_mode
        if args.planning_wall_time_s is not None:
            if not 0 <= args.planning_wall_time_s < float("inf"):
                raise ValueError("planning-wall-time-s must be finite and positive")
            data["search_strategy"]["planning_wall_time_s"] = args.planning_wall_time_s
        planning_policy = replace(planning_policy, data=data)
        scene = build_verified_motion_input(planning_policy)
    strategy = planning_policy.data.get("search_strategy", {})
    row_state = RowUnloadingState(RowSequencePolicy(
        row_height_fraction=float(strategy.get("row_height_fraction", 0.05))))
    if args.actual_state is not None:
        actual_state_bytes = args.actual_state.read_bytes()
        actual_state = json.loads(actual_state_bytes)
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
        try:
            result = run_layout_single_carton_audit(
                planning_policy, progress_callback=progress, motion_input=scene, row_state=row_state)
        except BaseException as exc:
            progress({"event": "CANCELLED" if isinstance(exc, KeyboardInterrupt) else "ERROR",
                      "reason": str(exc), "exception_type": type(exc).__name__,
                      "simulation_profile": profile_evidence(planning_policy.data),
                      "best_complete_result": None, "resume_supported": False,
                      "progress_evidence": "planning_progress.jsonl"})
            raise
    serialize_started = time.monotonic()
    write_layout_single_carton_audit(result, output / "motion.json")
    delivery = {"simulation_profile": profile_evidence(planning_policy.data), "planning_seconds": result["planning_performance"]["planning_total_wall_seconds"],
        "serialization_seconds": time.monotonic()-serialize_started,
        "preflight_seconds": None, "export_seconds": None, "isaac_executed": False,
        "actual_state_sha256": None, "archived_or_live_state_not_modified": True}
    if args.actual_state:
        import hashlib
        delivery["actual_state_sha256"] = hashlib.sha256(actual_state_bytes).hexdigest()
        if args.actual_state.read_bytes() != actual_state_bytes:
            delivery["status"] = "ACTUAL_STATE_CHANGED_DURING_PLANNING_NO_EXPORT"
            (output / "delivery.json").write_text(json.dumps(delivery, indent=2), encoding="utf-8")
            raise RuntimeError("actual state changed during planning; no bundle exported")
    print(json.dumps({"status": result["complete_trajectory_status"], "elapsed_s": time.monotonic() - start,
                      "statistics": result["statistics"]}), flush=True)
    if result["complete_trajectory_status"] == "PASS":
        from unloading_sim.m710_execution import build_m710_execution_preflight, write_m710_execution_preflight
        preflight_started = time.monotonic()
        preflight = build_m710_execution_preflight(
            args.execution_config, motion_result=result, motion_input=scene)
        write_m710_execution_preflight(preflight, output / "preflight.json")
        delivery["preflight_seconds"] = time.monotonic()-preflight_started
        delivery["simulation_execution_ready"] = preflight.get("simulation_execution_ready", False)
        (output / "delivery.json").write_text(json.dumps(delivery, indent=2), encoding="utf-8")
        print(json.dumps({"preflight": preflight["status"], "blockers": preflight["blockers"]}), flush=True)
        if not preflight.get("simulation_execution_ready", False):
            return 3
        if args.execution_bundle is None:
            args.execution_bundle = output / "replay_bundle.json"
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
            export_started = time.monotonic()
            subprocess.run([
                sys.executable, str(ROOT / "scripts/export_isaac_fanuc_replay.py"),
                "--preflight", str((output / "preflight.json").resolve()),
                "--output", str(args.execution_bundle.resolve()),
            ], check=True, cwd=ROOT, env=child_env)
            delivery["export_seconds"] = time.monotonic()-export_started
            from unloading_sim.m710_replay_contract import verify_m710_replay_bundle
            readback_started = time.monotonic()
            delivery["bundle_readback"] = verify_m710_replay_bundle(
                json.loads(args.execution_bundle.read_text(encoding="utf-8")), project_root=ROOT)
            delivery["bundle_readback_seconds"] = time.monotonic()-readback_started
            delivery["bundle_path"] = str(args.execution_bundle.resolve())
            baseline = output / "first_feasible"
            baseline.mkdir(exist_ok=False)
            for source, name in ((output / "motion.json", "motion.json"),
                                 (output / "preflight.json", "preflight.json"),
                                 (args.execution_bundle, "replay_bundle.json")):
                shutil.copyfile(source, baseline / name)
            delivery["first_complete_feasible_result_seconds"] = result["planning_performance"]["planning_total_wall_seconds"]
            delivery["first_exportable_result_seconds"] = time.monotonic() - start
            delivery["subsequent_optimization_seconds"] = 0.
            delivery["immutable_baseline_directory"] = str(baseline.resolve())
            register = planning_policy.data["search_strategy"].get("history", {}).get("register_directory")
            if register:
                from unloading_sim.history_candidates import register_planned_motion
                register_path = Path(register)
                if not register_path.is_absolute():
                    register_path = planning_policy.project_root / register_path
                delivery["history_registration"] = register_planned_motion(register_path, result, preflight)
            (output / "delivery.json").write_text(json.dumps(delivery, indent=2), encoding="utf-8")
        return 0
    (output / "delivery.json").write_text(json.dumps(delivery, indent=2), encoding="utf-8")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
