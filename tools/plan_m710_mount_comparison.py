"""Four paired mounting layouts, five fixed historical starting worlds.

Each cell is a new independent planning world. Historical roots are initial
conditions only: no old candidate q, trajectory, attachment or native receipt.
The existing complete-cycle planner and its shared deadlines remain unchanged.
"""
import argparse
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

from plan_m710_top_layer_lifted import (
    ROOT, save, source_identity, run_seed, load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector,
    FrozenTopLayer, BatchState, box_message)
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.coupled_batch import batch_scene, message_box
from unloading_sim.workcell_layout import load_workcell_layout, audit_layout_constraints

LAYOUTS = {
    "baseline": "m710id70_unloading_layout_v1_transverse_scissor_026.yaml",
    "y_minus_100": "m710id70_unloading_layout_v1_mount_y_minus_100.yaml",
    "x_minus_100": "m710id70_unloading_layout_v1_mount_x_minus_100.yaml",
    "xy_minus_100": "m710id70_unloading_layout_v1_mount_xy_minus_100.yaml",
}
TARGETS = ("carton_l06_c02", "carton_l06_c03", "carton_l06_c00",
           "carton_l07_c00", "carton_l07_c04")
FLAGS = dict(qualification_status="NOT_EVALUATED", continuous_swept_proof=False,
             physical_execution="NOT_RUN", execution_ready=False)


def load_cases(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if tuple(c["target_id"] for c in data["cases"]) != TARGETS:
        raise ValueError("MOUNT_COMPARISON_TARGET_SCOPE_CHANGED")
    for case in data["cases"]:
        root = case["root"]
        if (root["predecessor_final_state_receipt"] is not None
                or root["velocity_rad_s"] != [0.]*6
                or case["right_front_eccentric"] != (case["target_id"] == "carton_l07_c00")):
            raise ValueError("MOUNT_COMPARISON_INPUT_PERMISSIONS_OR_REST_CHANGED")
        if set(root) != {"q_rad", "velocity_rad_s", "height_m", "world_cartons",
                         "completed", "predecessor_final_state_receipt"}:
            raise ValueError("MOUNT_COMPARISON_ROOT_CONTAINS_EXTRA_HINTS")
    return data


def make_context(layout_name):
    policy = load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
    layout = load_workcell_layout(ROOT/"configs/workcells"/LAYOUTS[layout_name])
    audit = audit_layout_constraints(layout)
    if audit["overall_status"] != "PASS":
        raise ValueError("MOUNT_LAYOUT_AUDIT_REJECTED:"+layout_name)
    policy = replace(policy, layout_validation=replace(policy.layout_validation, layout=layout))
    original = build_verified_motion_input(policy, ROOT)
    built = _build_automatic_trajectory_connector(original, layout.robot())
    if built.connector is None:
        raise ValueError(built.failure_reason)
    return original, built.connector, audit


def make_state(case, original):
    root = case["root"]
    frozen_type = FrozenTopLayer if case["right_front_eccentric"] else FrozenCartonPopulation
    frozen = frozen_type.capture(original.cartons)
    originals = {b.name: box_message(b) for b in original.cartons}
    active = {b["id"]: b for b in root["world_cartons"]}
    if (set(active) != set(originals)-set(root["completed"])
            or any(originals[n] != b for n, b in active.items())
            or case["target_id"] not in active):
        raise ValueError("MOUNT_COMPARISON_INITIAL_CARTONS_CHANGED")
    state = BatchState(frozen, tuple(message_box(b) for b in root["world_cartons"]),
                       np.asarray(root["q_rad"], float), root["height_m"],
                       ledger=[dict(target_id=n, source="HISTORICAL_INITIAL_CONDITION_ONLY")
                               for n in root["completed"]])
    if state.root() != root:
        raise ValueError("MOUNT_COMPARISON_ROOT_REBUILD_CHANGED")
    return state


def check_inputs(cases):
    """Small production-backend root check; not IK or a planning success."""
    checks = []
    for variant in LAYOUTS:
        original, built, audit = make_context(variant)
        for case in cases:
            state = make_state(case, original)
            scene = batch_scene(original, state, built.robot, original.policy.layout_validation.layout)
            if scene.support_graph.supports[case["target_id"]]:
                raise ValueError("MOUNT_COMPARISON_TARGET_NOT_REMOVABLE")
            failure = built.validate_unloaded_state(state.q, scene.all_obstacles, stage="unloaded")
            checks.append(dict(variant=variant, target_id=case["target_id"],
                backend=type(built.robot).__name__, q_rad=state.q.tolist(),
                base_transform=built.robot.base_transform.tolist(),
                layout_fingerprint=original.policy.layout_validation.layout.layout_fingerprint,
                scene_fingerprint=scene.snapshot["scene_fingerprint"],
                layout_audit=audit, failure=failure, passed=failure is None,
                native_complete_cycle_checked=False))
    return checks


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inputs", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--worker-command")
    p.add_argument("--worker-binary", type=Path)
    p.add_argument("--source-commit", required=True)
    p.add_argument("--source-manifest", type=Path)
    p.add_argument("--development", action="store_true")
    p.add_argument("--check-only", action="store_true")
    args = p.parse_args()
    identity = source_identity(args)
    data = load_cases(args.inputs)
    args.output.mkdir(parents=True, exist_ok=False)
    started = perf_counter()
    checks = check_inputs(data["cases"])
    save(args.output/"initial-state-checks.json", dict(**identity, checks=checks,
         wall_s=perf_counter()-started, all_passed=all(c["passed"] for c in checks)))
    if not all(c["passed"] for c in checks):
        raise ValueError("PAIRED_INITIAL_STATE_INVALID_NO_FORMAL_SEARCH_STARTED")
    if args.check_only:
        print(json.dumps(dict(initial_states_checked=len(checks), passed=True)), flush=True)
        return
    if args.worker_binary is None or not args.worker_command:
        raise ValueError("NATIVE_WORKER_REQUIRED")
    worker_sha = hashlib.sha256(args.worker_binary.read_bytes()).hexdigest()
    args.task_budget_s, args.stage_budget_s, args.ipc_timeout_s = 300., 90., 100.
    args.direct_first, args.upper_arm_preference = True, True
    args.receive_height_range = (.60, 2.10)
    save(args.output/"protocol.json", dict(**identity, **FLAGS,
        schema="m710_mount_comparison_v1", layouts=LAYOUTS, targets=TARGETS,
        input_sha256=hashlib.sha256(args.inputs.read_bytes()).hexdigest(), inputs=data,
        worker_sha256=worker_sha, seed=71070, task_budget_s=300., shared_prefix_budget_s=90.,
        independent_worlds=True, paired_initial_q=True, historical_solution_hints=False,
        complete_cycle_required=True, native_planner_unchanged=True,
        command=[sys.executable, *sys.argv]))
    outcomes = []

    def checkpoint(status):
        summary = dict(**identity, **FLAGS, status=status, outcomes=outcomes,
                       planned_trial_count=len(LAYOUTS)*len(TARGETS),
                       experiment_wall_s=perf_counter()-started)
        save(args.output/"summary.json", summary)

    # Target-major order keeps each paired comparison close in server time.
    for case in data["cases"]:
        for variant in LAYOUTS:
            original, built, audit = make_context(variant)
            state = make_state(case, original)
            args.right_front_eccentric = case["right_front_eccentric"]
            args.junction_only = args.right_front_eccentric
            run = args.output/variant/case["target_id"]
            run.mkdir(parents=True)
            folder = run/"seed-71070"
            folder.mkdir()
            save(run/"protocol.json", dict(**identity, **FLAGS, seeds=[71070],
                scope="PAIRED_MOUNT_INDEPENDENT_SINGLE_CARTON", variant=variant,
                frozen=state.frozen.evidence(), scoped_target_ids=[case["target_id"]],
                configured_initial_q=state.q.tolist(), initial_root=state.root(),
                original_cartons=[box_message(b) for b in original.cartons],
                active_layout_file="configs/workcells/"+LAYOUTS[variant],
                worker_sha256=worker_sha, right_front_eccentric=args.right_front_eccentric,
                third_sync_disabled=True, task_budget_s=300., stage_budget_s=90.,
                candidate_visit_prefix_slice_s=10. if args.right_front_eccentric else 20.,
                upper_arm_preference="BOUNDED_SOFT_FK_WITH_GENERAL_SEEDS",
                temporary_height_minimum_m=.60, historical_completed_count=len(state.ledger)))
            print(json.dumps(dict(event="MOUNT_TRIAL_START", variant=variant,
                                  target=case["target_id"])), flush=True)
            checkpoint("RUNNING")
            try:
                result = run_seed(args, original, built, state.frozen, 71070, folder,
                                  initial_state=state, target_ids=[case["target_id"]])
            except Exception:
                import traceback
                (run/"integration-exception.txt").write_text(traceback.format_exc(), encoding="utf-8")
                outcomes.append(dict(variant=variant, target_id=case["target_id"],
                                     status="INTEGRATION_ERROR_RETAINED"))
                checkpoint("INTEGRATION_ERROR_RETAINED")
                raise
            outcomes.append(dict(variant=variant, target_id=case["target_id"],
                status=result["status"], completed_count=result["completed_count"],
                planning_wall_s=result["planning_wall_s"],
                attempts=result["attempts"], initial=result["initial"], terminal=result["terminal"]))
            checkpoint("RUNNING")
            print(json.dumps(dict(event="MOUNT_TRIAL_RECORDED", variant=variant,
                target=case["target_id"], completed=result["completed_count"],
                wall_s=result["planning_wall_s"])), flush=True)
    checkpoint("COMPARISON_COMPLETE")


if __name__ == "__main__":
    main()
