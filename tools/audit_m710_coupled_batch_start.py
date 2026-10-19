"""Check the full coupled initial world before a continuous top-layer batch.

An invalid fixed mechanism is a batch-start failure, independent of robot IK.
No old single-box planner, prior trajectory, release or drain is invoked here.
This entry produces diagnostic evidence; it is not a replacement batch planner.
"""
import argparse
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from unloading_sim.coupled_lift_checks import check_coupled_state, check_coupled_interval
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.top_layer_lifted import FrozenTopLayer, rank_remaining
from unloading_sim.moveit2_backend import ResidentMoveItClient, box_message, digest
from unloading_sim.single_contact import SingleContactClient, SingleContactConnector
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.workcell_layout import _obb_record


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def native_initial_probe(original, built, state, args, seed, out):
    """Official robot/tool versus exactly the same complete coupled world."""
    snapshot = deepcopy(original.snapshot)
    fixed = tuple(b for b in state.boxes if b.category != "carton")
    snapshot["assembly"]["fixed_components"] = [_obb_record(b) for b in fixed]
    snapshot["coupled_mechanism"] = state.mechanism
    snapshot["batch_start_scope"] = "CANDIDATE_GEOMETRY_NOT_ACCEPTED_MOTION"
    snapshot.pop("scene_fingerprint", None)
    snapshot["scene_fingerprint"] = digest(snapshot)
    scene = replace(original, fixed_components=fixed, snapshot=snapshot,
                    receiver=next(b for b in fixed if b.name == "conveyor_transverse"))
    targets = FrozenTopLayer.capture(scene.cartons)
    q = np.asarray(snapshot["robot"]["q_rad"])
    ranked = rank_remaining(targets, scene.cartons, [], q, built.robot, scene.receiver)
    target = next(b for b in scene.cartons if b.name == ranked[0]["target_id"])
    os.environ["M710_MOVEIT_SEED"] = str(seed)
    client = SingleContactClient(ResidentMoveItClient(args.worker_command, log_path=str(out/"worker.log")),
                                 search_trace_enabled=True)
    try:
        c = SingleContactConnector.from_existing(built, scene, native_client=client,
            planning_mode="cold_from_scratch", native_budget=dict(task_wall_time_s=300.,
                stage_wall_time_s=90., ipc_timeout_s=100.))
        c.stack_carton_names = {b.name for b in scene.cartons}
        c.robot_state_validator.stack_carton_names = set(c.stack_carton_names)
        faces, screening = simulation_contact_targets(scene, target, c.robot.base_transform[:3, 3],
                                                       c.budget.pregrasp_standoff_m)
        face = next(t for t in faces if t.face_role == "FRONT")
        c.expected_contact(face, scene.policy.data["suction"])
        req = c._build_native_request(q, q, scene.all_obstacles, seed=seed, stage="pregrasp")
        req["op"] = "check_start"
        save(out/"native-request.json", req)
        response = client.request(req, timeout=100.)
        save(out/"native-response.json", response)
        save(out/"model-init.json", client.trace_inputs["model"])
        canonical = lambda world: sorted(world, key=lambda b: b["id"])
        if canonical(req["world"]) != canonical([box_message(b) for b in state.boxes]):
            raise ValueError("NATIVE_AND_COUPLED_WORLD_DIVERGED")
        return dict(status=response["status"], scope="OFFICIAL_ROBOT_AND_TOOL_AT_UNCHANGED_INITIAL_Q",
                    native_world_sha256=digest(canonical(req["world"])), object_count=len(req["world"]),
                    q_rad=q.tolist(), screening=screening,
                    prior_calls=0, motion_planner_calls=0, target_attached=False,
                    world_world_collision_checked_by="coupled_lift_checks.py SEPARATE_GATE")
    finally:
        save(out/"native-requests.json", client.records)
        client.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--source-commit", required=True)
    p.add_argument("--source-manifest", type=Path,
                   help="SHA-bound source export for an isolated server without full Git objects")
    p.add_argument("--worker-command", required=True)
    p.add_argument("--worker-binary", type=Path, required=True)
    p.add_argument("--seeds", type=int, nargs="+", default=[71070, 71071])
    p.add_argument("--height-step-m", type=float, default=.01)
    args = p.parse_args()
    if args.source_manifest:
        manifest = json.loads(args.source_manifest.read_text(encoding="utf-8"))
        actual = manifest["source_commit"]
        expected_files = set(manifest["sha256"])
        actual_files = {str(path.relative_to(ROOT)).replace("\\", "/")
                        for directory in ("src", "tools", "configs", "ros2")
                        for path in (ROOT/directory).rglob("*")
                        if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc"}
        if expected_files != actual_files:
            raise ValueError("SOURCE_MANIFEST_FILE_SET_MISMATCH")
        for relative, expected in manifest["sha256"].items():
            path = (ROOT/relative).resolve()
            if not path.is_relative_to(ROOT) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError("SOURCE_FILE_MISMATCH:"+relative)
    else:
        actual = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        dirty = subprocess.check_output(["git", "diff", "HEAD", "--", "src", "tools", "configs", "ros2"], cwd=ROOT)
        if dirty:
            raise ValueError("SOURCE_HAS_UNCOMMITTED_IMPLEMENTATION_CHANGES")
    if actual != args.source_commit:
        raise ValueError("SOURCE_COMMIT_MISMATCH")
    if len(set(args.seeds)) != len(args.seeds) or not args.seeds:
        raise ValueError("DISTINCT_PREFIXED_SEEDS_REQUIRED")
    args.output.mkdir(parents=True, exist_ok=False)
    began = perf_counter()
    policy = load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
    original = build_verified_motion_input(policy, ROOT)
    layout = policy.layout_validation.layout
    frozen = FrozenTopLayer.capture(original.cartons)
    belt = next(b for b in original.fixed_components if b.name == "conveyor_transverse")
    low = float(belt.center[2]+belt.half_extents[2])
    state = coupled_conveyor_state(layout, low, original.cartons)
    built = _build_automatic_trajectory_connector(original, layout.robot())
    if built.connector is None:
        raise ValueError(built.failure_reason)
    initialization_s = perf_counter()-began
    q = np.asarray(original.snapshot["robot"]["q_rad"])
    ranking = rank_remaining(frozen, original.cartons, [], q, built.connector.robot, belt)
    protocol = dict(schema="m710_coupled_top_layer_start_protocol_v1", source_commit=actual,
                    seeds=args.seeds, seed_selection="FIXED_BEFORE_NATIVE_PROBES",
                    initial_state_source="CONFIGURED_OFFLINE_Q_NOT_PHYSICAL_FEEDBACK",
                    q_rad=q.tolist(), velocity_rad_s=[0.]*6, height_m=low,
                    frozen_top_layer=frozen.evidence(), proposed_process_queue=ranking,
                    original_scene_fingerprint=original.snapshot["scene_fingerprint"],
                    coupled_scene_fingerprint=state.scene_fingerprint,
                    ideal_receiver_drain="REQUESTED_FOR_BATCH_BUT_NOT_REACHED",
                    geometry=state.evidence(), command=[sys.executable, *sys.argv])
    save(args.output/"protocol.json", protocol)
    started = perf_counter()
    initial = check_coupled_state(layout, low, original.cartons,
                                  designed_contacts=original.snapshot["assembly"]["designed_contacts"])
    save(args.output/"initial-world-check.json", initial)
    world_check_s = perf_counter()-started
    rows = []
    for seed in args.seeds:
        out = args.output/f"seed-{seed}"
        out.mkdir()
        started = perf_counter()
        try:
            native = native_initial_probe(original, built.connector, state, args, seed, out)
        except Exception as exc:
            native = dict(status="PROBE_ERROR", error=f"{type(exc).__name__}: {exc}")
        row = dict(seed=seed, status="BATCH_START_REJECTED" if not initial["passed"] else
                   "INITIAL_WORLD_ONLY_NOT_A_BATCH_PLAN", completed_count=0,
                   initial_target_ids=list(frozen.target_ids), remaining_target_ids=list(frozen.target_ids),
                   unload_order=[], reception_count=0, ideal_drain_count=0,
                   root_q_rad=q.tolist(), terminal_q_rad=q.tolist(), root_height_m=low, terminal_height_m=low,
                   carton_count_before=len(original.cartons), carton_count_after=len(original.cartons),
                   accepted_native_motion_edges=0, native_motion_planning="NOT_STARTED",
                   blocking_stage="INITIAL_COUPLED_WORLD" if not initial["passed"] else "AUDIT_SCOPE_ONLY",
                   native_initial_probe=native, probe_wall_s=perf_counter()-started,
                   continuous_batch_completed=False, events=[], physical_execution="NOT_RUN",
                   qualification_status="NOT_EVALUATED", continuous_swept_proof=False, execution_ready=False)
        rows.append(row)
        save(out/"result.json", row)
    # Diagnostic synchronized parameter sweep, NOT an attempted movement after
    # an invalid start. Both directions use the actual current height supplied.
    heights = frozen.evidence()["heights"]
    high = max(h["Hmax_world_m"] for h in heights.values())
    lower = min(h["Hmin_world_m"] for h in heights.values())
    started = perf_counter()
    interval = check_coupled_interval(layout, low, high, original.cartons,
        height_step_m=args.height_step_m, designed_contacts=original.snapshot["assembly"]["designed_contacts"])
    save(args.output/"diagnostic-height-interval.json", interval)
    height_check_s = perf_counter()-started
    summary = dict(schema="m710_coupled_top_layer_start_audit_v1", source_commit=actual,
                   status="BLOCKED_BY_INITIAL_COUPLED_WORLD" if not initial["passed"] else "INITIAL_AUDIT_ONLY",
                   completed_count=0, target_count=len(frozen.target_ids), runs=rows,
                   initial_failures=initial["failures"], initial_allowed_pairs=initial["allowed_assembly_pairs"],
                   full_batches_run=0, motion_planner_calls=0,
                   real_success_rate="NOT_ESTIMATED", physical_execution="NOT_RUN",
                   qualification_status="NOT_EVALUATED", continuous_swept_proof=False, execution_ready=False,
                   reception_count=0, ideal_drain_count=0, ideal_receiver_drain_used=False,
                   requested_cycle_status="NOT_REACHED", nominal_reception="NOT_RUN",
                   full_batch_replay="UNAVAILABLE_NO_ACCEPTED_NATIVE_MOTION",
                   world_sha256=initial["world_sha256"],
                   height_snapshot=dict(low_m=low, Hmin_m=lower, Hmax_m=high),
                   timings=dict(initialization_s=initialization_s, initial_world_check_s=world_check_s,
                       native_probe_total_s=sum(r["probe_wall_s"] for r in rows),
                       diagnostic_height_check_s=height_check_s, native_trajectory_s=0.,
                       hardware_lift_duration_s=None, physical_reception_duration_s=None),
                   worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
                   environment=dict(python=platform.python_version(), affinity=sorted(os.sched_getaffinity(0))),
                   no_changes=dict(cartons=True, dimensions=True, initial_q=True, fixed_assembly=True,
                       collision_margins=True, legacy_entries=True))
    save(args.output/"summary.json", summary)
    print(json.dumps(dict(status=summary["status"], completed=0, targets=len(frozen.target_ids),
                         initial_failure_count=len(initial["failures"]), runs=len(rows))), flush=True)
    return 2 if not initial["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
