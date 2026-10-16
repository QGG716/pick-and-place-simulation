"""Single fixed-face approach/contact experiment. Never exports or executes."""
from pathlib import Path
import argparse
from dataclasses import replace
from collections import Counter
import hashlib
import json
import os
import platform
import sys
from time import perf_counter
import uuid
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.single_contact import SingleContactConnector, SingleContactClient, plan_faces, MARKER
from unloading_sim.moveit2_backend import ResidentMoveItClient, box_message
# Reuse scene reconstruction only; never call old plan_box/rank/main.
from plan_m710_top_row_only import next_scene


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target",required=True)
    stages=parser.add_mutually_exclusive_group()
    stages.add_argument("--through-extraction",action="store_true",
        help="extend contact with planned attachment and one controlled extraction LIN")
    stages.add_argument("--through-preplace",action="store_true",
        help="extend extraction with one fixed preplace IK and loaded RRTConnect")
    parser.add_argument("--initial-state",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--worker-command",required=True)
    parser.add_argument("--worker-binary",type=Path,required=True)
    parser.add_argument("--source-commit",required=True)
    parser.add_argument("--seed",type=int,default=71070)
    parser.add_argument("--task-budget-s",type=float,default=300.)
    parser.add_argument("--stage-budget-s",type=float,default=90.)
    parser.add_argument("--ipc-timeout-s",type=float,default=100.)
    args=parser.parse_args()
    if (not all(np.isfinite(x) and x>0 for x in (args.task_budget_s,args.stage_budget_s,args.ipc_timeout_s))
            or args.ipc_timeout_s<=args.stage_budget_s):
        parser.error("finite positive budgets and IPC > stage required")
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ["M710_MOVEIT_SEED"]=str(args.seed)
    os.environ["M710_MOVEIT_LOG"]=str(args.output/"worker.log")
    os.environ.pop("M710_MOVEIT_REQUEST_LOG",None)
    os.environ.pop("M710_MOVEIT_DIAGNOSTICS",None)
    initialized=perf_counter()
    policy=load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
    original=build_verified_motion_input(policy,ROOT)
    built=_build_automatic_trajectory_connector(original,policy.layout_validation.layout.robot())
    if built.connector is None: raise RuntimeError(built.failure_reason)
    q=np.asarray(policy.layout_validation.initial_q,dtype=float)
    velocity=np.zeros(6)
    removed=[]
    source="CONFIGURED_OFFLINE_STATIC_STATE"
    initial_input=None
    if args.initial_state:
        initial_input=json.loads(args.initial_state.read_text())
        if initial_input["target"]!=args.target:raise ValueError("INITIAL_TARGET_MISMATCH")
        q=np.asarray(initial_input["start_q_rad"],dtype=float)
        velocity=np.asarray(initial_input.get("velocity_rad_s",[0.]*6),dtype=float)
        removed=initial_input["removed_targets"]
        source="ARCHIVED_PLANNED_WITHDRAWAL_REST_STATE_NOT_PHYSICAL_MEASUREMENT"
        names={b.name for b in original.cartons}
        if (set(removed)-names or len(set(removed))!=len(removed) or args.target in removed):
            raise ValueError("INVALID_REMOVED_CARTONS")
    if (q.shape!=(6,) or velocity.shape!=(6,) or not np.isfinite(q).all()
            or not np.isfinite(velocity).all() or np.any(velocity!=0.)):
        raise ValueError("INVALID_OR_NONSTATIONARY_INITIAL_STATE")
    prepared=perf_counter()
    scene=next_scene(original,removed,q,built.connector.robot)
    target=next((b for b in scene.cartons if b.name==args.target),None)
    if target is None:raise ValueError("UNKNOWN_TARGET")
    scene_prepare_s=perf_counter()-prepared
    client=SingleContactClient(ResidentMoveItClient(args.worker_command,log_path=str(args.output/"worker.log")),
        extraction_enabled=args.through_extraction,preplace_enabled=args.through_preplace)
    connector_type=SingleContactConnector
    marker=MARKER
    if args.through_extraction:
        from unloading_sim.single_extraction import SingleExtractionConnector, MARKER as EXTRACTION_MARKER
        connector_type=SingleExtractionConnector
        marker=EXTRACTION_MARKER
    if args.through_preplace:
        from unloading_sim.single_preplace import SinglePreplaceConnector, MARKER as PREPLACE_MARKER
        connector_type=SinglePreplaceConnector
        marker=PREPLACE_MARKER
    c=None
    try:
        c=connector_type.from_existing(built.connector,scene,native_client=client,
            planning_mode="cold_from_scratch",native_budget=dict(task_wall_time_s=args.task_budget_s,
                stage_wall_time_s=args.stage_budget_s,ipc_timeout_s=args.ipc_timeout_s))
        c.budget=replace(c.budget,planning_wall_time_s=args.task_budget_s,candidate_wall_time_s=None,
            stage_wall_time_s=args.stage_budget_s,postprocess_wall_time_s=args.stage_budget_s)
        c.stack_carton_names={b.name for b in scene.cartons}
        c.robot_state_validator.stack_carton_names=set(c.stack_carton_names)
        initialization_s=perf_counter()-initialized
        began=perf_counter()
        c.start_planning_request(began)
        c._native_task_started=began
        c._request_deadline_monotonic=began+args.task_budget_s
        c._deadline_monotonic=c._request_deadline_monotonic
        c._final_export_reserve_s=0.
        c.boundary_costs={}
        generation=perf_counter()
        targets,screening=simulation_contact_targets(scene,target,c.robot.base_transform[:3,3],
            c.budget.pregrasp_standoff_m)
        target_generation_s=perf_counter()-generation
        plan,attempts=plan_faces(c,targets,q,velocity,source,scene.all_obstacles,policy.data["suction"],args.seed)
        total_s=perf_counter()-began
        counts=Counter()
        for record in client.records:
            for key,value in record.get("response",{}).get("native_solver_calls",{}).items():counts[key]+=value
            counts["IK"]+=record.get("response",{}).get("native_ik_calls",0)
        costs=[]
        for record in client.records:
            if record.get("op") not in ("plan","check_start","check_extraction_end","check_transit_end"):continue
            response=record.get("response",{})
            clearance=response.get("clearance",{})
            if clearance:
                ws=clearance.get("workspace",{})
                assert ws.get("pair_profile_enabled") is False
            costs.append(dict(request_id=response.get("request_id"),op=record["op"],status=record["status"],
                clearance=clearance,request_check_evidence=response.get("request_check_evidence")))
        result=dict(schema="m710_single_approach_contact_v1",status=marker,through_extraction=args.through_extraction or args.through_preplace,through_preplace=args.through_preplace,
            qualification_status="NOT_EVALUATED",continuous_swept_proof=False,
            physical_execution="NOT_RUN",execution_ready=False,run_id="single-contact-"+uuid.uuid4().hex,
            source_commit=args.source_commit,command=[sys.executable,*sys.argv],seed=args.seed,
            target_id=args.target,complete=plan is not None,
            selected_face=None if plan is None else plan["target"]["geometric_face_id"],
            start_q_rad=q.tolist(),start_velocity_rad_s=velocity.tolist(),start_source=source,
            initial_input=initial_input,initial_input_sha256=None if not args.initial_state else
                hashlib.sha256(args.initial_state.read_bytes()).hexdigest(),
            initial_cartons=[box_message(b) for b in scene.cartons],removed_cartons=removed,
            scene_revision=scene.snapshot["scene_fingerprint"],model_identity=c.native_identity,
            worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
            environment=dict(python=platform.python_version(),affinity=sorted(os.sched_getaffinity(0)),
                OMP_NUM_THREADS=os.environ.get("OMP_NUM_THREADS"),OPENBLAS_NUM_THREADS=os.environ.get("OPENBLAS_NUM_THREADS")),
            budgets=dict(task_s=args.task_budget_s,stage_s=args.stage_budget_s,ipc_s=args.ipc_timeout_s,
                ik_timeout_field_s=c.native_ik_seconds,ik_timeout_field_is_solver_wall_deadline=False,ik_method="one finite-iteration KDL getPositionIK; shared IPC deadline",
                standoff_m=c.budget.pregrasp_standoff_m),
            initialization_s=initialization_s,scene_prepare_s=scene_prepare_s,
            target_generation_s=target_generation_s,total_planning_s=total_s,
            timing_semantics="initialization includes scene preparation; total planning includes target generation and all face attempts; request RTT/solver/checks nested",
            screening=screening,attempts=attempts,plan=plan,
            generated_stage_evidence=[{k:r[k] for k in ("stage","stage_id","parent_stage_id",
                "generation_source","pipeline_id","planner_id","points","time_parameterization",
                "native_solver_calls","authoritative_status","native_output_status") if k in r}
                for r in c.native_generated],
            native_calls={k:counts[k] for k in ("IK","OMPL","LIN","PTP")},
            prior_build_load_query_calls=sum("prior" in str(r.get("op")) or r.get("pipeline_id")=="static_prior" for r in client.records),
            forbidden_requests=client.forbidden_attempts,history_and_legacy=c._cold_counters,
            boundary_costs=c.boundary_costs,checks=costs,events=[] if plan is None else plan["events"])
        (args.output/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
        with (args.output/"requests.jsonl").open("w") as f:
            for record in client.records:f.write(json.dumps(record,allow_nan=False)+"\n")
        print(json.dumps({k:result[k] for k in ("run_id","complete","selected_face","total_planning_s",
            "native_calls","prior_build_load_query_calls")}),flush=True)
    finally:
        # Retain actual request evidence even if initialization/output handling fails.
        with (args.output/"requests.jsonl").open("w") as f:
            for record in client.records:f.write(json.dumps(record,allow_nan=False)+"\n")
        client.close()


if __name__=="__main__":
    main()
