"""One fresh, sequential top-row planning-only batch. No execution/export imports."""
from __future__ import annotations

import argparse
import csv
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
from time import perf_counter
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.contact_scheduler import ContactCandidateScheduler
from unloading_sim.conveyor_placement import PlacementPolicy
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector,
    _exposed_faces, _scheduled_contact_poses, virtual_tcp_from_physical_contact)
from unloading_sim.moveit2_backend import box_message, digest
from unloading_sim.planning_only import PlanningOnlyConnector, MARKER, SKIPPED, SKIPPED_CHECKS
from unloading_sim.support import SupportRelationGraph
from unloading_sim.unloading_sequence import (RowUnloadingState, RowSequencePolicy,
    actual_tcp_approach_costs)
from unloading_sim.validation_motion import grasp_seed_configurations


def next_scene(scene, completed, q, robot):
    """Called only after a full generated withdrawal; ideal removal is offline."""
    cartons = tuple(b for b in scene.cartons if b.name not in completed)
    snapshot = deepcopy(scene.snapshot)
    snapshot["cartons"] = [b for b in snapshot["cartons"] if b["name"] not in completed]
    snapshot["robot"].update(q_rad=np.asarray(q).tolist(), tcp_pose_world=robot.fk(q).tolist(),
        flange_pose_world=robot.named_link_frames(q)["flange"].tolist())
    snapshot["tool"]["task_tcp_pose_world"] = robot.fk(q).tolist()
    # These display/audit snapshots are not current physical observations.
    snapshot["robot"].pop("link_collision_obbs", None)
    snapshot["tool"].pop("rigid_collision_obbs", None)
    snapshot["tool"].pop("collision_obb", None)
    snapshot["initial_state_audit"] = {"status": SKIPPED}
    snapshot["planning_only"] = {"status": MARKER, "ideal_removed": list(completed),
        "source": "CONFIGURED_OFFLINE_GEOMETRY_NOT_PHYSICAL_MEASUREMENT"}
    snapshot.pop("scene_fingerprint", None)
    snapshot["scene_fingerprint"] = digest(snapshot)
    population = scene.policy.data["task_population"]
    graph = SupportRelationGraph.build(cartons,
        contact_tolerance_m=float(population["support_contact_tolerance_m"]),
        minimum_overlap_ratio=float(population["support_minimum_overlap_ratio"]))
    return replace(scene, snapshot=snapshot, cartons=cartons, support_graph=graph,
        remaining_stack_names=tuple(b.name for b in cartons),
        snapshot_verification={"status": SKIPPED}, snapshot_consistency={"status": SKIPPED})


def rank(scene, sequence, connector, q):
    costs = actual_tcp_approach_costs(scene.cartons, connector.robot.fk(q),
        {b.name: _exposed_faces(scene, b.name) for b in scene.cartons},
        pregrasp_standoff_m=connector.budget.pregrasp_standoff_m,
        virtual_contact_offset_m=scene.policy.tool_frames.virtual_to_physical_contact_offset_m)
    return sequence.rank(scene.cartons, support_graph=scene.support_graph,
        candidate_costs={name: item["normalized_cost"] for name, item in costs["candidates"].items()},
        scene_context={"q_rad": np.asarray(q).tolist(), "receiver_occupancy": []})


def plan_box(c, scene, target, q, seed):
    """Candidate generation through in-memory complete result, no file I/O."""
    poses = tuple(_scheduled_contact_poses(scene, target, _exposed_faces(scene, target.name)))
    strategy = scene.policy.data["search_strategy"]
    cap = strategy.get("grasp_poses_per_task") or len(poses) * 3
    scheduler = ContactCandidateScheduler(poses, target=target,
        context={"scene_fingerprint": scene.snapshot["scene_fingerprint"],
                 "policy_fingerprint": scene.policy.policy_fingerprint},
        request_seed=seed, batch_size=c.budget.task_pose_batch_size,
        attempt_limit=cap, complete_connection_limit=cap, fair_retries=True)
    suction = scene.policy.data["suction"]
    last_failure = {"reason": "NO_NATIVE_IK", "stage": "contact_endpoint"}
    candidate_attempts = []
    while not c._native_cancelled():
        item = scheduler.next_attempt(deadline_reached=c._native_cancelled())
        if item is None:
            break
        candidate, schedule = item
        began = perf_counter()
        requested = virtual_tcp_from_physical_contact(candidate["pose"],
            c.flange_from_virtual_task_tcp, c.flange_from_physical_contact)
        seeds = grasp_seed_configurations(c.robot, [np.asarray(q)])
        stream = c.native_ik_stream(requested, seeds, scene.all_obstacles,
            seed=schedule["ik_seed"], target_contact=target, stage="contact_endpoint",
            contact_candidate={"face": candidate["face"], "suction": suction})
        connected = False
        for solution in stream:
            connected = True
            outcome = c.plan(target=target, face=candidate["face"],
                requested_virtual_contact=requested,
                grasp_candidates=[{"q_rad": solution.q.tolist(),
                    "candidate_id": solution.search_evidence["candidate_id"]}],
                home_q=q, all_obstacles=scene.all_obstacles, receiver=scene.receiver,
                support_names=sorted(scene.support_graph.supported_by[target.name]),
                suction=suction, seed=schedule["path_seed"])
            if outcome.success:
                scheduler.finish(schedule, {"complete_trajectory": True},
                    complete_connection_attempted=True, elapsed_s=perf_counter()-began)
                return outcome.segment, dict(candidates=scheduler.summary(),
                    failed_candidates=candidate_attempts, last_failure=None)
            last_failure = outcome.failure
            if c._native_cancelled():
                break
        scheduler.finish(schedule, dict(complete_trajectory=False,
            failure_stage=last_failure.get("stage") if connected else "grasp_ik",
            failure_reason=last_failure.get("reason") if connected else "NO_IK"),
            complete_connection_attempted=connected, elapsed_s=perf_counter()-began)
        candidate_attempts.append(dict(candidate_id=schedule["candidate_id"],
            face=candidate["face"], roll=candidate["roll"], ik=stream.evidence(),
            failure=compact_failure(last_failure)))
    return None, dict(candidates=scheduler.summary(), failed_candidates=candidate_attempts,
        last_failure=compact_failure(last_failure), budget_exhausted=c._native_cancelled())


def compact_failure(value):
    if not isinstance(value, dict):
        return value
    return {k: compact_failure(v) for k, v in value.items()
            if k not in {"attempts", "trace", "points", "submitted_request", "extraction_attempts"}}


def call_summary(c):
    stages = []
    for record in c.native_evidence:
        keep = {k: record[k] for k in ("stage", "status", "stage_id", "parent_stage_id",
            "pipeline_id", "planner_id", "native_solver_calls", "mtc_plan_s", "native_output_check_s",
            "solver_returned_success", "solver_message", "moveit_error_code", "returned_waypoint_count",
            "processing_branch", "goal_translation_delta_m", "goal_rotation_delta_rad", "candidate_id",
            "native_output_status", "authoritative_status", "failure", "clearance", "process_policy") if k in record}
        # Each reply repeats the full tool/ACM policy (up to 146 KB). Its
        # identity is already retained at batch level; save counters/witnesses
        # here, not hundreds of copies of this static model metadata.
        if "clearance" in keep:
            keep["clearance"] = {k: v for k, v in keep["clearance"].items() if k != "policy"}
        stages.append(keep)
    counts = {name: sum(int(r.get("native_solver_calls", {}).get(name, 0))
                       for r in c.native_evidence) for name in ("PTP", "LIN", "OMPL")}
    counts["IK"] = sum(int(r.get("native_ik_calls", 0)) for r in c.native_ik_evidence)
    return dict(counts=counts, stages=stages, ik=[{k: r[k] for k in (
        "stage", "status", "ik_s", "elapsed_s", "authority_failure") if k in r} for r in c.native_ik_evidence],
        native_solver_s=sum(r.get("mtc_plan_s", 0) for r in c.native_evidence),
        ik_solver_s=sum(r.get("ik_s", 0) for r in c.native_ik_evidence),
        ik_including_ipc_s=sum(r.get("elapsed_s", 0) for r in c.native_ik_evidence),
        skipped_calls=dict(c.skipped_calls), independent_path_checks_executed=len(c.authority_path_evidence),
        validated_stage_records=len(c.native_verified), history_and_legacy=dict(c._cold_counters))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--worker-command", required=True)
    parser.add_argument("--worker-binary", required=True, type=Path)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--seed", type=int, default=71070)
    parser.add_argument("--stage-budget-s", type=float, default=300.)
    parser.add_argument("--box-budget-s", type=float, default=1800.)
    parser.add_argument("--batch-budget-s", type=float, default=7200.)
    parser.add_argument("--ipc-timeout-s", type=float, default=360.)
    parser.add_argument("--prepare-only", action="store_true", help="Development preload only; no IK or motion solve")
    parser.add_argument("--diagnostic-input", type=Path,
        help="Fixed-input single-box diagnostic only; JSON with target, start_q_rad, removed_targets, seed_offset")
    args = parser.parse_args()
    diagnostic = json.loads(args.diagnostic_input.read_text()) if args.diagnostic_input else None
    if diagnostic is not None:
        q_input = np.asarray(diagnostic["start_q_rad"], dtype=float)
        if q_input.shape != (6,) or not np.isfinite(q_input).all():
            parser.error("invalid diagnostic starting state")
    budgets = {k: getattr(args, k) for k in ("stage_budget_s", "box_budget_s", "batch_budget_s", "ipc_timeout_s")}
    if any(not np.isfinite(v) or v <= 0 for v in budgets.values()) or args.ipc_timeout_s <= args.stage_budget_s:
        parser.error("positive finite budgets and IPC > stage required")
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ["M710_MOVEIT_SEED"] = str(args.seed)
    # Per-request full payload logging is unnecessary for this experiment.
    os.environ.pop("M710_MOVEIT_REQUEST_LOG", None)
    os.environ["M710_MOVEIT_LOG"] = str(args.output / "worker.log")
    os.environ["M710_MOVEIT_DIAGNOSTICS"] = str(args.output / "requests.jsonl")
    initialized = perf_counter()
    policy = load_layout_motion_policy(ROOT / "configs/validation/m710id70_proof_of_concept.yaml")
    scene = build_verified_motion_input(policy, ROOT)
    built = _build_automatic_trajectory_connector(scene, policy.layout_validation.layout.robot())
    if built.connector is None:
        raise RuntimeError(built.failure_reason)
    c = PlanningOnlyConnector.from_existing(built.connector, scene, args.worker_command,
        native_budget=dict(task_wall_time_s=args.box_budget_s, stage_wall_time_s=args.stage_budget_s,
                           ipc_timeout_s=args.ipc_timeout_s))
    try:
        c.budget = replace(c.budget, planning_wall_time_s=args.box_budget_s,
            candidate_wall_time_s=None, stage_wall_time_s=args.stage_budget_s,
            postprocess_wall_time_s=args.stage_budget_s)
        strategy = policy.data["search_strategy"]
        c.placement_policy = replace(PlacementPolicy(),
            maximum_candidates=int(strategy.get("placement_candidates", 12)),
            coarse_samples_per_axis=int(strategy.get("coarse_place_samples_per_axis", 3)),
            fine_samples_per_axis=int(strategy.get("fine_place_samples_per_axis", 7)),
            contact_tolerance_m=c.contact_tolerance_m,
            edge_tolerance_m=float(strategy.get("conveyor_footprint_boundary_tolerance_m", 1e-6)),
            occupancy_clearance_m=c.collision_policy.pair_clearance("external", c.collision_margin_m),
            process_family_by_support=dict(strategy.get("surface_process_families", {})),
            allowed_families_by_process={k: tuple(v) for k,v in strategy.get("allowed_placement_families", {}).items()},
            overlap_process_priority=tuple(strategy.get("overlap_process_priority", ["longitudinal", "transverse"])))
        q = np.asarray(policy.layout_validation.initial_q).copy()
        sequence = RowUnloadingState(config=RowSequencePolicy(
            row_height_fraction=float(strategy.get("row_height_fraction", .05)),
            actual_cost_weight=float(strategy.get("actual_cost_weight", .15))))
        selected_row = rank(scene, sequence, c, q)
        top_ids = set(selected_row.row_remaining_names)
        expected = {f"carton_l07_c{i:02d}" for i in range(5)}
        if len(scene.cartons) != 40 or top_ids != expected:
            raise ValueError("CONFIGURED_GEOMETRIC_TOP_ROW_NOT_EXPECTED_FIVE")
        initial_removed = diagnostic["removed_targets"] if diagnostic else []
        if diagnostic:
            if (diagnostic["target"] not in top_ids or set(initial_removed) - top_ids
                    or diagnostic["target"] in initial_removed):
                raise ValueError("INVALID_DIAGNOSTIC_TARGET_CONTEXT")
            q = q_input.copy()
        first_target = selected_row.candidates[0].carton
        c.stack_carton_names = set(b.name for b in scene.cartons)
        c.robot_state_validator.contact_target_name = first_target.name
        preload = c._build_native_request(q, q, scene.all_obstacles, seed=args.seed,
                                         stage="pregrasp", target_contact=first_target)
        preload.update(op="inspect", inspect_pairs=[])
        preload_result = c.native.request(preload)
        summary = dict(schema="m710_top_row_planning_only_batch_v1", status=MARKER,
            run_id="planning-only-" + uuid.uuid4().hex, source_commit=args.source_commit,
            command=[sys.executable, *sys.argv], seed=args.seed, budgets=budgets,
            environment=dict(python=platform.python_version(), platform=platform.platform(),
                native=c.native_startup, worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest()),
            initial_state_source="离线规划基准，初始机器人静止，场景采用配置几何，非本次物理实测状态",
            initial_q_rad=q.tolist(), initial_scene_fingerprint=scene.snapshot["scene_fingerprint"],
            initial_cartons=[box_message(b) for b in scene.cartons],
            top_row=selected_row.as_dict(), tool_mass_kg=scene.snapshot["tool"]["mass_kg"],
            payload_mass_kg=42.5, model_identity=c.native_identity,
            skipped_checks={k: SKIPPED for k in SKIPPED_CHECKS},
            execution_requests=0, isaac_launches=0, executable_bundle_generated=False,
            execution_qualification="NOT_EVALUATED", initialization_s=perf_counter()-initialized,
            preload=preload_result, development_prepare_only=args.prepare_only, boxes=[])
        trajectories, completed = [], []
        batch_start = perf_counter()
        if not args.prepare_only:
            for index in ([int(diagnostic["seed_offset"])] if diagnostic else range(5)):
                scene_started = perf_counter()
                current = next_scene(scene, initial_removed + completed, q, c.robot)
                selection = rank(current, sequence, c, q)
                target = selection.candidates[0].carton
                if diagnostic and target.name != diagnostic["target"]:
                    raise ValueError("DIAGNOSTIC_TARGET_ORDER_CHANGED")
                if target.name not in top_ids - set(completed):
                    raise ValueError("TOP_ROW_SELECTION_CHANGED")
                c.native_scene = current
                c.stack_carton_names = {b.name for b in current.cartons}
                c.robot_state_validator.stack_carton_names = set(c.stack_carton_names)
                c.robot_state_validator.contact_target_name = target.name
                c._native_contact_context = None
                request = c._build_native_request(q, q, current.all_obstacles, seed=args.seed,
                                                  stage="pregrasp", target_contact=target)
                request.update(op="inspect", inspect_pairs=[])
                prepared = c.native.request(request)
                scene_s = perf_counter()-scene_started
                box_started = perf_counter()
                c.start_planning_request(box_started)
                c.native_task_id = "planning-only-" + uuid.uuid4().hex
                c._native_task_started = box_started
                c._request_deadline_monotonic = min(box_started+args.box_budget_s, batch_start+args.batch_budget_s)
                c._deadline_monotonic = c._request_deadline_monotonic
                c._final_export_reserve_s = 0.
                c.skipped_calls.clear()
                trajectory = None
                try:
                    trajectory, detail = plan_box(c, current, target, q, args.seed+index)
                except Exception as exc:
                    detail = dict(last_failure={"reason": type(exc).__name__, "detail": str(exc)})
                elapsed = perf_counter()-box_started
                row = dict(order=index+1, target=target.name, complete=trajectory is not None,
                    plan_s=elapsed, cumulative_plan_s=sum(b["plan_s"] for b in summary["boxes"])+elapsed,
                    scene_update_s=scene_s, start_q_rad=q.tolist(), task_id=c.native_task_id,
                    scene_cartons=len(current.cartons), scene_prepared=prepared,
                    row_selection=selection.as_dict(), calls=call_summary(c), **detail)
                if trajectory is not None:
                    if not np.array_equal(q, trajectory["path"][0]):
                        raise ValueError("CROSS_BOX_START_CHANGED")
                    q = np.asarray(trajectory["path"][-1]).copy()
                    row["end_q_rad"] = q.tolist()
                    row["ideal_remove_after_withdrawal"] = target.name
                    trajectories.append(trajectory)
                    completed.append(target.name)
                summary["boxes"].append(row)
                print(json.dumps(dict(target=target.name, complete=row["complete"], plan_s=elapsed,
                    cumulative_plan_s=row["cumulative_plan_s"], progress=f"{len(completed)}/{1 if diagnostic else 5}")), flush=True)
                if trajectory is None:
                    break
        if diagnostic:
            summary.update(diagnostic_input=diagnostic, diagnostic_input_sha256=hashlib.sha256(
                args.diagnostic_input.read_bytes()).hexdigest(), fresh_five_box_batch=False)
        summary.update(batch_wall_s=perf_counter()-batch_start, completed_count=len(completed),
            completed_targets=completed, missing_targets=sorted(top_ids-set(completed)),
            cumulative_attempted_plan_s=sum(b["plan_s"] for b in summary["boxes"]),
            T_plan_diagnostic_s=sum(b["plan_s"] for b in summary["boxes"]) if diagnostic else None,
            T_plan_5_s=sum(b["plan_s"] for b in summary["boxes"]) if len(completed)==5 else None,
            scene_update_s=sum(b["scene_update_s"] for b in summary["boxes"]),
            first_box_scene_preparation_s=summary["boxes"][0]["scene_update_s"] if summary["boxes"] else None,
            inter_box_scene_update_s=sum(b["scene_update_s"] for b in summary["boxes"][1:]),
            result="DIAGNOSTIC_COMPLETE" if diagnostic and len(completed)==1 else "COMPLETE_5_OF_5" if len(completed)==5 else "DEVELOPMENT_PRELOAD_ONLY" if args.prepare_only else "INCOMPLETE")
        (args.output / "timing.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        (args.output / "trajectories.json").write_text(json.dumps(dict(status=MARKER,
            execution_ready=False, run_id=summary["run_id"], trajectories=trajectories), allow_nan=False), encoding="utf-8")
        with (args.output / "timing.csv").open("w", newline="", encoding="utf-8") as stream:
            fields = ["order", "target", "complete", "plan_s", "cumulative_plan_s", "scene_update_s"]
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
            writer.writeheader(); writer.writerows(summary["boxes"])
        print(json.dumps({k: summary[k] for k in ("result", "completed_count", "T_plan_5_s",
            "cumulative_attempted_plan_s", "initialization_s", "scene_update_s", "batch_wall_s")}), flush=True)
    finally:
        c.native.close()


if __name__ == "__main__":
    main()
