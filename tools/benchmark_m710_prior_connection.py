"""Serial fixed-connection diagnostic; never a formal unloading Task or answer cache."""
from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
from time import perf_counter
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.moveit2_backend import digest, native_generation_evidence, validate_native_result
from unloading_sim.planning_only import PlanningOnlyConnector, MARKER
from unloading_sim.static_prior import placement_policy_fingerprint, request_context
from plan_m710_top_row_only import next_scene


def fixed_inputs(archive):
    """Read endpoint witnesses and attachment state only, never archived motion paths."""
    timing = json.loads((archive / "timing.json").read_text())
    archived_contacts = json.loads((archive / "trajectories.json").read_text())
    requests = [json.loads(line) for line in (archive / "requests.jsonl").read_text().splitlines() if line]
    contacts = {row["target"]: dict(contact=deepcopy(row["contact"]), face=row["face"])
                for row in archived_contacts["trajectories"]}
    cases = []
    for target, stage in (("carton_l07_c03", "pregrasp"), ("carton_l07_c03", "transit"),
                          ("carton_l07_c04", "transit")):
        box = next(row for row in timing["boxes"] if row["target"] == target)
        matches = [row for row in requests if row.get("task_id") == box["task_id"]
            and row.get("stage") == stage and row.get("pipeline_id") == "ompl"
            and row.get("status") == "SUCCESS" and row.get("q_goal") is not None]
        if len(matches) != 1:
            raise ValueError(f"EXPECTED_ONE_ARCHIVED_FIXED_CONNECTION:{target}:{stage}:{len(matches)}")
        witness = matches[0]
        q_start, q_goal = (np.asarray(witness[name],dtype=float) for name in ("q_start", "q_goal"))
        if any(q.shape != (6,) or not np.isfinite(q).all() for q in (q_start, q_goal)):
            raise ValueError("INVALID_ARCHIVED_ENDPOINT")
        removed = [row["target"] for row in timing["boxes"] if row["order"] < box["order"]]
        cases.append(dict(case_id=target+"-"+stage,target=target,stage=stage,
            q_start=q_start.tolist(),q_goal=q_goal.tolist(),seed=int(witness["seed"]),
            removed_targets=removed,contact=contacts[target]["contact"],face=contacts[target]["face"],
            historical_request_id=witness.get("request_id"),historical_stage_id=witness["stage_id"],
            held_out=False,perturbation=None))
    for original in list(cases):
        if original["stage"] == "transit":
            case=deepcopy(original)
            case["case_id"] += "-heldout-j1-plus-0p005"
            case["q_start"][0] += .005
            case.update(held_out=True,perturbation=dict(joint_index=0,delta_rad=.005,
                source="NOT_USED_IN_DATABASE_CONSTRUCTION"))
            cases.append(case)
    return cases,timing


def configure_input(c, scene, case):
    current = next_scene(scene, case["removed_targets"], case["q_start"], c.robot)
    target = next(box for box in current.cartons if box.name == case["target"])
    c.native_scene=current
    c.stack_carton_names={box.name for box in current.cartons}
    c.robot_state_validator.stack_carton_names=set(c.stack_carton_names)
    c.robot_state_validator.contact_target_name=target.name
    c._native_contact_context=dict(target=target,face=case["face"],
        suction=current.policy.data["suction"],selection=deepcopy(case["contact"]["cup_selection"]))
    attachment=None
    if case["stage"] == "transit":
        contact_q=np.asarray(case["contact"]["actual_q_rad"],dtype=float)
        # This historical state defines the fixed attached-body transform. It
        # is not imported as a new motion sample or a solved query answer.
        physical_contact=c.physical_from_virtual(c.robot.fk(contact_q))
        attachment=PhysicalContactAttachment(c.robot,RigidAttachment.capture(physical_contact,target),
            c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
    return current,target,attachment


def run_case(args, built, scene, case, mode):
    run_dir=args.output/case["case_id"]/mode
    run_dir.mkdir(parents=True,exist_ok=False)
    os.environ["M710_MOVEIT_LOG"]=str(run_dir/"worker.log")
    started=perf_counter()
    c=PlanningOnlyConnector.from_existing(built.connector,scene,args.worker_command,
        planning_mode=mode,native_budget=dict(stage_wall_time_s=args.request_budget_s,
            task_wall_time_s=args.request_budget_s,ipc_timeout_s=args.ipc_timeout_s))
    try:
        c.static_prior_placement_policy_sha256=placement_policy_fingerprint(scene.policy)
        if mode == "static_prior_fast":
            c.load_static_prior(args.static_prior)
        initialization_s=perf_counter()-started
        prepare_started=perf_counter()
        current,target,attachment=configure_input(c,scene,case)
        input_prepare_s=perf_counter()-prepare_started
        began=perf_counter()
        req=c._build_native_request(case["q_start"],case["q_goal"],current.all_obstacles,
            seed=case["seed"],stage=case["stage"],attachment=attachment,
            target_contact=target if attachment is None else None)
        task_id="diagnostic-"+uuid.uuid4().hex
        req.update(op="diagnostic_connection",diagnostic_only=True,task_id=task_id,
            stage_id=task_id+":connection",parent_stage_id="",require_native_motion=True,
            allowed_planning_time_s=args.request_budget_s,
            pipeline_id="static_prior" if mode=="static_prior_fast" else "ompl",
            planner_id="SPARSE_PRM" if mode=="static_prior_fast" else "RRTConnectkConfigDefault")
        if mode == "static_prior_fast":
            req["prior_context"]=request_context(req,
                placement_policy_sha256=c.static_prior_placement_policy_sha256,
                joint_limits_rad=c.robot.joint_limits.tolist())
        requested_at=perf_counter()
        raw=c.native.request(req,timeout=args.ipc_timeout_s)
        returned_at=perf_counter()
        round_trip_s=returned_at-requested_at
        protocol_failure=None
        if raw.get("status") == "SUCCESS":
            try:
                if (raw.get("diagnostic_only") is not True or raw.get("mtc_generation") is not True
                        or any(raw.get(key)!=req[key] for key in ("task_id","stage_id","parent_stage_id"))
                        or raw.get("planning_only_status") != MARKER
                        or not native_generation_evidence(raw,req,planning_mode=mode)):
                    raise ValueError("DIAGNOSTIC_NATIVE_SOURCE_MISMATCH")
                validate_native_result(raw,case["q_start"],case["q_goal"],c.native_joint_names)
            except (ValueError,KeyError,TypeError) as exc:
                protocol_failure=str(exc)
        row=dict(case_id=case["case_id"],target=case["target"],stage=case["stage"],planning_mode=mode,
            status=raw.get("status"),complete=raw.get("status")=="SUCCESS" and protocol_failure is None,
            protocol_failure=protocol_failure,held_out=case["held_out"],perturbation=case["perturbation"],
            connection_elapsed_s=perf_counter()-began,request_round_trip_s=round_trip_s,
            native_solver_s=raw.get("native_solver_s",raw.get("mtc_plan_s")),input_prepare_s=input_prepare_s,
            initialization_s=initialization_s,static_prior=getattr(c,"static_prior_metadata",None),
            native_solver_calls=raw.get("native_solver_calls",{}),prior_usage=raw.get("prior_usage"),
            failure=raw.get("failure"),processing_branch=raw.get("processing_branch"),
            geometry_input_sha256=digest({key:req.get(key) for key in (
                "q_start","q_goal","world","attachment","process_policy","clearance_policy","identity")}),
            source_commit=args.source_commit,worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
            seed=case["seed"],request_budget_s=args.request_budget_s,ipc_timeout_s=args.ipc_timeout_s,
            experimental_status=MARKER,qualification_status="NOT_EVALUATED",diagnostic_only=True,
            historical_input_scope=["request start/goal","remaining carton identities","contact actual q and cup masks"],
            historical_motion_paths_used_as_answer=False,formal_task_chain=False)
        # Write after the complete reply and protocol checks; file I/O is separate.
        (run_dir/"request.json").write_text(json.dumps(req,indent=2,allow_nan=False))
        (run_dir/"response.json").write_text(json.dumps(raw,allow_nan=False))
        (run_dir/"result.json").write_text(json.dumps(row,indent=2,allow_nan=False))
        print(json.dumps(row,allow_nan=False),flush=True)
        return row
    finally:
        c.native.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--static-prior",type=Path,required=True)
    parser.add_argument("--worker-command",required=True)
    parser.add_argument("--worker-binary",type=Path,required=True)
    parser.add_argument("--source-commit",required=True)
    parser.add_argument("--request-budget-s",type=float,default=60.)
    parser.add_argument("--ipc-timeout-s",type=float,default=90.)
    parser.add_argument("--seed",type=int,default=71070)
    parser.add_argument("--case",action="append",dest="cases",help="Repeat to select declared case IDs")
    parser.add_argument("--mode",action="append",choices=("cold_from_scratch","static_prior_fast"),
        dest="modes",help="Default is a serial cold/prior pair for each case")
    args=parser.parse_args()
    if not 0 < args.request_budget_s < args.ipc_timeout_s:
        parser.error("positive request budget smaller than IPC watchdog required")
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ["M710_MOVEIT_SEED"]=str(args.seed)
    os.environ.pop("M710_MOVEIT_REQUEST_LOG",None)
    os.environ.pop("M710_MOVEIT_DIAGNOSTICS",None)
    cases,archive_timing=fixed_inputs(args.archive)
    if args.cases:
        unknown=set(args.cases)-{case["case_id"] for case in cases}
        if unknown: parser.error("unknown cases: "+str(sorted(unknown)))
        cases=[case for case in cases if case["case_id"] in args.cases]
    policy=load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
    scene=build_verified_motion_input(policy,ROOT)
    built=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
    if built.connector is None: raise RuntimeError(built.failure_reason)
    manifest=dict(status=MARKER,qualification_status="NOT_EVALUATED",diagnostic_only=True,
        command=[sys.executable,*sys.argv],case_ids=[case["case_id"] for case in cases],
        historical_run_id=archive_timing["run_id"],historical_source_commit=archive_timing["source_commit"],
        archive_files={name:hashlib.sha256((args.archive/name).read_bytes()).hexdigest()
            for name in ("timing.json","requests.jsonl","trajectories.json")},
        result_path_cache_used=False,source_commit=args.source_commit,
        thread_environment={key:os.environ.get(key) for key in ("OPENBLAS_NUM_THREADS","OMP_NUM_THREADS")})
    (args.output/"manifest.json").write_text(json.dumps(manifest,indent=2))
    rows=[]
    for case in cases:
        for mode in args.modes or ["cold_from_scratch","static_prior_fast"]:
            row=run_case(args,built,scene,case,mode)
            rows.append(row)
            (args.output/"comparison.json").write_text(json.dumps(dict(**manifest,rows=rows),indent=2,allow_nan=False))
    with (args.output/"comparison.csv").open("w",newline="") as stream:
        fields=["case_id","planning_mode","complete","status","connection_elapsed_s","request_round_trip_s",
            "native_solver_s","held_out","geometry_input_sha256","seed"]
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction="ignore")
        writer.writeheader();writer.writerows(rows)


if __name__ == "__main__":
    main()
