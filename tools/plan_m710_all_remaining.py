"""Continue all remaining cartons from the accepted c00 planning checkpoint.

One unchanged 300 s task / 90 s shared prefix per eligible carton, seed 71070.
Failed tasks retain their carton. This is planning with nominal reception and
ideal drain, not a physical unloading experiment or a statistical repeat trial.
"""
import argparse
from copy import deepcopy
from dataclasses import replace
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tarfile
from time import perf_counter

from plan_m710_junction_single import predecessor
from plan_m710_top_layer_lifted import (
    ROOT, save, source_identity, run_seed, load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector,
    FrozenTopLayer, box_message)
from unloading_sim.carton_population import FrozenCartonPopulation, rank_population, blocked_population
from unloading_sim.coupled_batch import batch_scene
from unloading_sim.workcell_layout import load_workcell_layout

C00_RESULT_SHA = "12d826ee718c2cc639d08fc5702662040407615fab5caf6cd632d4832a94d2cb"
C00_SOURCE = "13ce5f88bb65e26f29fed2022a5d24c1ab9a6911"
WORKER_SHA = "5aaff6025df88c7c931e25e5bc2d2d7ff11e22a102c19c8dc2c3b5d6a92b0f29"
FLAGS = dict(qualification_status="NOT_EVALUATED", continuous_swept_proof=False,
             physical_execution="NOT_RUN", execution_ready=False)


def restore(args):
    policy = load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
    original = build_verified_motion_input(policy, ROOT)
    frozen = FrozenTopLayer.capture(original.cartons)
    built = _build_automatic_trajectory_connector(original, policy.layout_validation.layout.robot())
    if built.connector is None:
        raise ValueError(built.failure_reason)
    state, evidence = predecessor(args.archive, original, built.connector, frozen)
    layout = load_workcell_layout(
        ROOT/"configs/workcells/m710id70_unloading_layout_v1_transverse_scissor_026.yaml")
    policy = replace(policy, layout_validation=replace(policy.layout_validation, layout=layout))
    variant = build_verified_motion_input(policy, ROOT)
    if [box_message(b) for b in variant.cartons] != [box_message(b) for b in original.cartons]:
        raise ValueError("CONTINUATION_CARTON_GEOMETRY_CHANGED")
    built = _build_automatic_trajectory_connector(variant, layout.robot())
    if built.connector is None:
        raise ValueError(built.failure_reason)
    raw = args.c00_result.read_bytes()
    if args.c00_result.suffix == ".gz":
        raw = gzip.decompress(raw)
    if hashlib.sha256(raw).hexdigest() != C00_RESULT_SHA:
        raise ValueError("C00_PREDECESSOR_RESULT_HASH_CHANGED")
    result = json.loads(raw)
    if (result["status"] != "COMPLETE_PLANNING_ONLY" or result["completed_count"] != 1
            or len(result["cycles"]) != 1 or result["initial"] != state.root()):
        raise ValueError("C00_PREDECESSOR_NOT_COMPLETE_OR_ROOT_CHANGED")
    state.commit(result["cycles"][0])
    if state.root() != result["terminal"] or len(state.cartons) != 36:
        raise ValueError("C00_TERMINAL_RESTORE_CHANGED")
    evidence["c00"] = dict(result_sha256=C00_RESULT_SHA, source_commit=C00_SOURCE,
                          committed_root=deepcopy(state.root()),
                          use="LEGAL_INITIAL_STATE_ONLY_NO_SOLVER_HINTS_OR_OLD_NATIVE_TREE")
    # Previous motion receipts are historical evidence, not the new worker root.
    state.cycles = []
    state.frozen = FrozenCartonPopulation.capture(variant.cartons)
    args.expected_model_identity = deepcopy(evidence["archived_model_identity"])
    args.expected_model_identity.update(policy_fingerprint=policy.policy_fingerprint,
                                       validator_identity=built.connector.validator_identity)
    args.receive_height_range = (
        float(layout.data["assembly"]["world_xyz_m"][2]+layout.data["conveyors"]["surface_z_a_m"]),
        float(layout.data["conveyors"]["coupling"]["maximum_surface_height_world_m"]))
    evidence["new_native_root"] = state.root()
    return variant, built.connector, state, evidence


def archive_task(folder, output):
    """Verify every archived byte before removing only this newly made task dir."""
    folder, output = folder.resolve(), output.resolve()
    if folder.parent != output or not folder.name.startswith("task-"):
        raise ValueError("ARCHIVE_TASK_OUTSIDE_RUN")
    files = {str(p.relative_to(folder)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in folder.rglob("*") if p.is_file()}
    archive = output/(folder.name+".tar.gz")
    if archive.exists():
        raise ValueError("TASK_ARCHIVE_ALREADY_EXISTS")
    with tarfile.open(archive, "w:gz", compresslevel=3) as tar:
        tar.add(folder, arcname=folder.name)
    with tarfile.open(archive) as tar:
        archived = {str(Path(m.name).relative_to(folder.name)):
                    hashlib.sha256(tar.extractfile(m).read()).hexdigest()
                    for m in tar.getmembers() if m.isfile()}
    if archived != files:
        raise ValueError("TASK_ARCHIVE_VERIFICATION_FAILED")
    receipt = dict(archive=archive.name, sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                   file_sha256=files, verified_before_removing_uncompressed_copy=True)
    save(output/(folder.name+"-archive.json"), receipt)
    shutil.rmtree(folder)
    return receipt


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--c00-result", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--worker-command", required=True)
    p.add_argument("--worker-binary", type=Path, required=True)
    p.add_argument("--source-commit", required=True)
    p.add_argument("--source-manifest", type=Path)
    p.add_argument("--development", action="store_true")
    args = p.parse_args()
    args.task_budget_s, args.stage_budget_s, args.ipc_timeout_s = 300., 90., 100.
    args.direct_first, args.junction_only = True, False
    args.right_front_eccentric, args.upper_arm_preference = False, True
    identity = source_identity(args)
    if hashlib.sha256(args.worker_binary.read_bytes()).hexdigest() != WORKER_SHA:
        raise ValueError("NATIVE_WORKER_BINARY_CHANGED")
    args.output.mkdir(parents=True, exist_ok=False)
    original, built, state, evidence = restore(args)
    initial = state.root()
    scoped_ids = [b.name for b in state.cartons]
    history = deepcopy(state.ledger)
    save(args.output/"predecessor.json", evidence)
    save(args.output/"protocol.json", dict(
        **identity, **FLAGS, scope="ALL_REMAINING_FROM_ACCEPTED_C00", seeds=[71070],
        frozen=state.frozen.evidence(), scoped_target_ids=scoped_ids,
        original_cartons=[box_message(b) for b in original.cartons],
        initial_root=initial, historical_completed_count=len(history),
        active_layout_file=str(original.policy.layout_validation.layout.config_path.relative_to(ROOT)),
        worker_sha256=WORKER_SHA, task_budget_s=300., stage_budget_s=90.,
        receive_height_range_m=args.receive_height_range,
        candidate_visit_prefix_slice_s=20., direct_first=True,
        upper_arm_preference="BOUNDED_SOFT_FK_WITH_GENERAL_SEEDS",
        right_front_eccentric=False, new_eccentric_permissions=False,
        attempted_target_restarts=False, search_operation="NEW_NATIVE_TASK_AT_COMMITTED_ROOT",
        tree_retained=False, nominal_drop_assumed=True, ideal_receiver_drain_assumed=True,
        command=[sys.executable, *sys.argv]))
    outcomes, attempted = [], set()
    started = perf_counter()

    def checkpoint(status):
        completed = len(state.ledger)-len(history)
        data = dict(**identity, **FLAGS, schema="m710_all_remaining_v1", status=status, seed=71070,
                    initial=initial, terminal=state.root(), target_count=len(scoped_ids),
                    completed_count=completed, success_rate=completed/len(scoped_ids),
                    historical_completed_count=len(history), historical_ledger=history,
                    ledger=state.ledger, outcomes=outcomes,
                    blocked=blocked_population(state.cartons, attempted),
                    attempted_target_ids=sorted(attempted),
                    remaining_target_ids=[b.name for b in state.cartons],
                    planning_wall_s=perf_counter()-started,
                    native_task_wall_s=sum(o.get("planning_wall_s", 0.) for o in outcomes),
                    release_count=completed, nominal_first_contact_count=completed,
                    separation_count=completed, ideal_drain_count=completed)
        save(args.output/"summary.json", data)
        return data

    while state.cartons:
        scene = batch_scene(original, state, built.robot, original.policy.layout_validation.layout)
        ranked = rank_population(state.frozen, state.cartons, [e["target_id"] for e in state.ledger],
                                 attempted, state.q, built.robot, scene.receiver,
                                 longitudinal=next(b for b in scene.fixed_components
                                                   if b.name == "conveyor_longitudinal"))
        if not ranked:
            break
        selected = ranked[0]
        name = selected["target_id"]
        attempted.add(name)
        folder = args.output/f"task-{len(outcomes)+1:03d}"
        folder.mkdir()
        save(folder/"dispatch.json", dict(selection=selected, root=state.root(),
                                         task_budget_s=300., prefix_budget_s=90.))
        checkpoint("RUNNING")
        try:
            result = run_seed(args, original, built, state.frozen, 71070, folder,
                              initial_state=state, target_ids=[name])
            # run_seed commits on a copy after evidence checkpoint; reproduce
            # exactly that accepted state, never remove a failed candidate.
            for plan in result["cycles"]:
                state.commit(plan)
            if state.root() != result["terminal"]:
                raise ValueError("PER_CARTON_TERMINAL_CHANGED")
            state.cycles = state.cycles[-1:]
        except Exception:
            import traceback
            (folder/"integration-exception.txt").write_text(traceback.format_exc())
            outcomes.append(dict(target_id=name, status="INTEGRATION_ERROR_RETAINED", folder=folder.name))
            checkpoint("INTEGRATION_ERROR_RETAINED")
            raise
        outcome = dict(target_id=name, status=result["status"],
                       completed_count=result["completed_count"],
                       planning_wall_s=result["planning_wall_s"],
                       dispatched_candidate_count=sum("folder" in a for a in result["attempts"]),
                       attempts=result["attempts"], root=result["initial"], terminal=result["terminal"])
        # Preserve raw success/failure/requests before reclaiming disk.
        outcome["evidence"] = archive_task(folder, args.output)
        outcomes.append(outcome)
        final = checkpoint("RUNNING")
        print(json.dumps(dict(event="CARTON_TASK_RECORDED", target=name,
                              completed=final["completed_count"], remaining=len(state.cartons),
                              attempted=len(attempted))), flush=True)
    final = checkpoint("COMPLETE_PLANNING_ONLY" if not state.cartons else "INCOMPLETE_RETAINED")
    if len(final["outcomes"])+len(final["blocked"]) != len(scoped_ids):
        raise ValueError("POPULATION_ACCOUNTING_GAP")
    print(json.dumps(dict(status=final["status"], completed=final["completed_count"],
                          targets=final["target_count"], success_rate=final["success_rate"],
                          blocked=len(final["blocked"]))), flush=True)


if __name__ == "__main__":
    main()
