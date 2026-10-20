"""One c00 task from the committed c01 predecessor, not a new top-layer batch."""
import argparse
from copy import deepcopy
from dataclasses import replace
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import tarfile
import numpy as np
from plan_m710_top_layer_lifted import (
    ROOT,save,source_identity,run_seed,load_layout_motion_policy,build_verified_motion_input,
    _build_automatic_trajectory_connector,FrozenTopLayer,BatchState,box_message)
from unloading_sim.workcell_layout import load_workcell_layout

BASELINE="6416c192d1bae5a93e82418b4c8ae3d6c3e6280a"
ARCHIVED_SOURCE="feba0c5491ffe4958e61e30ee3baa3d4ee954e5e"
ARCHIVE_SHA="341fab7e63ac711ce4f0933a42db817a2587fed9121bad4e1ff138981ce6df01"


def predecessor(archive,original,built,frozen):
    """Read ONLY protocol and accepted attempts 001..003, never old c00 paths."""
    archive=Path(archive)
    archive_hash=hashlib.sha256()
    with archive.open("rb") as stream:
        for block in iter(lambda:stream.read(1024*1024),b""):
            archive_hash.update(block)
    if archive_hash.hexdigest()!=ARCHIVE_SHA:
        raise ValueError("PREDECESSOR_ARCHIVE_HASH_CHANGED")
    accessed=[]
    with tarfile.open(archive) as tar:
        def read(name):
            name="final-v2-on/"+name
            try:member=tar.getmember(name)
            except KeyError:member=tar.getmember(name+".gz")
            raw=tar.extractfile(member).read();decoded=gzip.decompress(raw) if member.name.endswith(".gz") else raw
            accessed.append(dict(member=member.name,sha256=hashlib.sha256(raw).hexdigest()))
            return json.loads(decoded)
        protocol=read("protocol.json")
        if (protocol["source_commit"]!=ARCHIVED_SOURCE or protocol["seeds"]!=[71070,71071] or
                not protocol["direct_first"] or protocol["original_cartons"]!=[box_message(b) for b in original.cartons] or
                protocol["configured_initial_q"]!=original.snapshot["robot"]["q_rad"]):
            raise ValueError("PREDECESSOR_MODEL_OR_INITIAL_SCENE_CHANGED")
        state=BatchState(frozen,tuple(original.cartons),np.asarray(protocol["configured_initial_q"]),
            float(original.receiver.center[2]+original.receiver.half_extents[2]))
        expected=["carton_l07_c02","carton_l07_c03","carton_l07_c01"]
        for i,name in enumerate(expected,1):
            prefix=f"seed-71070/attempt-{i:03d}/"
            summary=read(prefix+"summary.json");plan=read(prefix+"plan.json")
            if not summary["complete"] or plan["target_id"]!=name or summary["root"]!=state.root():
                raise ValueError("PREDECESSOR_COMMIT_SEQUENCE_CHANGED")
            last=plan["stages"][-1]
            if max(abs(v) for v in last["points"][-1]["v"])>1e-8:
                raise ValueError("PREDECESSOR_NOT_AT_REST")
            state.commit(plan)
        trace=read("seed-71070/attempt-003/search-trace.json")
        identity=trace["model"]["identity"]
    archived_root=state.root()
    if (len(state.cartons)!=37 or not {"carton_l07_c00","carton_l07_c04"}<=
            {b.name for b in state.cartons} or not built.robot.within_limits(state.q)):
        raise ValueError("PREDECESSOR_REMAINING_STATE_INVALID")
    evidence=dict(archive_sha256=ARCHIVE_SHA,baseline_commit=BASELINE,
        archived_planning_source=ARCHIVED_SOURCE,archived_seed=71070,
        selected_predecessor="committed c01; accepted attempts 001,002,003",
        accessed_members=accessed,archived_model_identity=identity,
        archived_root=deepcopy(archived_root),historical_ledger=deepcopy(state.ledger),
        occupancy=[],occupancy_source="THREE_RECORDED_IDEAL_RECEIVER_DRAIN_EVENTS",
        historical_receipt_use="PROVENANCE_ONLY_NOT_A_CURRENT_WORKER_RECEIPT",
        old_c00_solution_read=False)
    # Keep the physical q/world and ledger, but issue a fresh native root.
    state.cycles=[]
    evidence["new_native_root"]=state.root()
    return state,evidence


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--archive",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--worker-command",required=True);p.add_argument("--worker-binary",type=Path,required=True)
    p.add_argument("--source-commit",required=True);p.add_argument("--source-manifest",type=Path)
    p.add_argument("--development",action="store_true")
    geometry=p.add_mutually_exclusive_group()
    geometry.add_argument("--longitudinal-060",action="store_true",
        help="Explicit user-authorized width experiment; preserve the archived q/cartons and Ymin")
    geometry.add_argument("--transverse-scissor-030",action="store_true",
        help="300 mm transverse support spacing with the previous 600 mm longitudinal belt")
    args=p.parse_args()
    args.seed=71070;args.task_budget_s=300.;args.stage_budget_s=90.;args.ipc_timeout_s=100.
    args.direct_first=True;args.junction_only=True
    identity=source_identity(args)
    args.output.mkdir(parents=True,exist_ok=False)
    policy=load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
    original=build_verified_motion_input(policy,ROOT);frozen=FrozenTopLayer.capture(original.cartons)
    built=_build_automatic_trajectory_connector(original,policy.layout_validation.layout.robot())
    if built.connector is None:raise ValueError(built.failure_reason)
    state,origin=predecessor(args.archive,original,built.connector,frozen)
    args.expected_model_identity=deepcopy(origin["archived_model_identity"])
    if args.longitudinal_060 or args.transverse_scissor_030:
        variant_name=("m710id70_unloading_layout_v1_transverse_scissor_030.yaml"
                      if args.transverse_scissor_030 else "m710id70_unloading_layout_v1_longitudinal_060.yaml")
        layout=load_workcell_layout(ROOT/"configs/workcells"/variant_name)
        policy=replace(policy,layout_validation=replace(policy.layout_validation,layout=layout))
        variant=build_verified_motion_input(policy,ROOT)
        if [box_message(b) for b in variant.cartons]!=[box_message(b) for b in original.cartons]:
            raise ValueError("WIDTH_EXPERIMENT_CHANGED_CARTONS")
        new_built=_build_automatic_trajectory_connector(variant,layout.robot())
        if new_built.connector is None:raise ValueError(new_built.failure_reason)
        origin["authorized_geometry_change"]=dict(
            source="USER_FOLLOWUP_LONGITUDINAL_WIDTH",from_width_m=.7,to_width_m=.6,
            alignment="YMIN_FIXED",from_layout_fingerprint=original.policy.layout_validation.layout.layout_fingerprint,
            to_layout_fingerprint=layout.layout_fingerprint,config=str(layout.config_path),
            previous_policy_fingerprint=original.policy.policy_fingerprint,new_policy_fingerprint=policy.policy_fingerprint,
            cartons_unchanged=True,archived_q_unchanged=True,old_geometry_results_reused=False)
        if args.transverse_scissor_030:
            origin["authorized_geometry_change"].update(
                source="USER_FOLLOWUP_TRANSVERSE_SCISSOR_WIDTH",
                previous_experiment_layout="m710id70_unloading_layout_v1_longitudinal_060.yaml",
                transverse_support_spacing_from_m=.46,transverse_support_spacing_to_m=.30,
                transverse_support_outer_width_from_m=.56,transverse_support_outer_width_to_m=.40,
                support_centering="SYMMETRIC_X_ABOUT_UNCHANGED_TRANSVERSE_DECK",
                arm_lengths_cross_sections_and_deck_unchanged=True)
        args.expected_model_identity["policy_fingerprint"]=policy.policy_fingerprint
        args.expected_model_identity["validator_identity"]=new_built.connector.validator_identity
        original,built=variant,new_built
    save(args.output/"predecessor.json",origin)
    save(args.output/"protocol.json",dict(**identity,seeds=[71070],scope="ONE_NEW_JUNCTION_C00",
        frozen=frozen.evidence(),scoped_target_ids=["carton_l07_c00"],historical_completed_count=3,
        original_cartons=[box_message(b) for b in original.cartons],configured_initial_q=state.q.tolist(),
        active_layout_file=str(original.policy.layout_validation.layout.config_path.relative_to(ROOT)),
        authorized_geometry_change=origin.get("authorized_geometry_change"),
        initial_root=state.root(),worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
        task_budget_s=300.,stage_budget_s=90.,height_visit_prefix_slice_s=20.,
        height_resolution_m=.01,height_coarse_priority_spacing_m=.05,
        temporary_height_minimum_m=float(original.receiver.center[2]+original.receiver.half_extents[2]),
        temporary_height_domain_source="EXISTING_CONCEPT_COUPLED_GEOMETRY_MINIMUM_TO_FROZEN_HMIN",
        hardware_lift_travel="UNKNOWN",hardware_lift_duration_s=None,
        ompl_attempt_quota=None,pair_profile_enabled=False,direct_first=True,
        search_operation="RESTART",tree_retained=False,third_sync_disabled=True,
        command=[sys.executable,*sys.argv]))
    out=args.output/"seed-71070";out.mkdir()
    result=run_seed(args,original,built.connector,frozen,71070,out,
                    initial_state=state,target_ids=["carton_l07_c00"])
    save(args.output/"summary.json",dict(**identity,runs=[result]))
    print(json.dumps(dict(completed=result["completed_count"],targets=1,status=result["status"])),flush=True)


if __name__=="__main__":
    main()
