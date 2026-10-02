"""Current-input native worker development probes; never a physical-cycle result.

No history fixture, saved trajectory, contact q, or previous mask is read.  The
configured home is a diagnostic input, not an observation of an Isaac world.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import replace
import json
import os
from pathlib import Path
import sys
from time import perf_counter
import traceback

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))

from unloading_sim.geometry import OBB
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input,_build_automatic_trajectory_connector)
from unloading_sim.moveit2_backend import (MoveItLayoutConnector,MoveItUnavailable,
    ResidentMoveItClient,digest)
from unloading_sim.moveit2_native_cold import audit_native_motion_coverage
from unloading_sim.stage_motion_policy import MotionPurpose


def free_context(connector,target,suction):
    # A free-space diagnostic has no contact command. These masks are new empty
    # commands, never an imported successful grasp. No attachment is requested.
    connector._native_contact_context=dict(target=target,face="front",suction=suction,
        selection={key:[False]*72 for key in (
            "geometrically_eligible_mask","commanded_active_mask","actual_contact_mask")})
    connector.robot_state_validator.contact_target_name=target.name


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--stage-seconds",type=float,default=30.)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument("--parent-chain-only",action="store_true",
        help="This round's finite real-worker explicit-parent, ownership and identity probes only")
    mode.add_argument("--ownership-only",action="store_true",
        help="Fresh sparse single-carton ownership and identity probes; no earlier probe inputs")
    parser.add_argument("--ompl-counterexample",action="store_true",
        help="Bounded synthetic obstacle counterexample generated from this probe's new PTP output")
    args=parser.parse_args()
    if (args.parent_chain_only or args.ownership_only) and args.ompl_counterexample:
        parser.error("targeted parent/ownership modes exclude the broad OMPL counterexample matrix")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.exists():
        parser.error("output already exists; retain previous development evidence")
    began=perf_counter();c=None
    report=dict(schema="m710_native_cold_development_probe_v1",status="RUNNING",checks=[],
        scope="configured-home development probes; no measured initial state or complete-cycle acceptance",
        history_enabled=False,history_inputs_read=0,isaac="NOT_RUN",physical_success_count=0)
    def save():
        report["wall_seconds"]=perf_counter()-began
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    def add(name,passed,**details):
        report["checks"].append(dict(name=name,passed=bool(passed),**details));save()
        print(name,"PASS" if passed else "FAIL",flush=True)
    try:
        policy=load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
        scene=build_verified_motion_input(policy,ROOT)
        built=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
        if built.connector is None:
            raise RuntimeError(str(built.failure_reason))
        core=built.connector
        core.budget=replace(core.budget,planning_wall_time_s=None,candidate_wall_time_s=None,
            stage_wall_time_s=None,stage_connection_attempts=1)
        c=MoveItLayoutConnector.from_existing(core,scene,native_cold=True,
            native_budget=dict(task_wall_time_s=900.,stage_wall_time_s=args.stage_seconds,
                               ipc_timeout_s=max(120.,args.stage_seconds+30.)))
        c.start_planning_request()
        c.stack_carton_names={box.name for box in scene.cartons}
        c.robot_state_validator.stack_carton_names=frozenset(c.stack_carton_names)
        target=next(box for box in scene.cartons if box.name=="carton_l07_c02")
        free_context(c,target,policy.data["suction"])
        q=np.asarray(policy.layout_validation.initial_q).copy()
        report.update(startup=c.native_startup,scene_fingerprint=scene.snapshot["scene_fingerprint"],
            target=target.name,carton_count=len(scene.cartons),configured_home=q.tolist(),
            fk_checks=c.native_fk,policy_fingerprint=policy.policy_fingerprint,
            input_manifest=["current official model and tool geometry","current frozen 40-carton scene",
                "current static process policy","configured home (development only)","seed 71070"],
            native_budget=c.native_cold_evidence())
        save()
        home_failure=c.validate_unloaded_state(q,scene.all_obstacles)
        add("independent_configured_home",home_failure is None,failure=home_failure)
        if home_failure is not None:
            raise RuntimeError("CONFIGURED_HOME_REJECTED")
        if args.parent_chain_only or args.ownership_only:
            from m710_native_parent_probe import run_parent_chain_probe,run_ownership_only_probe
            run_probe=run_ownership_only_probe if args.ownership_only else run_parent_chain_probe
            run_probe(c,scene,q,policy,add,report)
            report["status"]="PASS" if all(item["passed"] for item in report["checks"]) else "FAIL"
            return 0 if report["status"]=="PASS" else 1
        q=c.native_root_state(q,scene.all_obstacles,stage="pregrasp")
        stream=c.native_ik_stream(c.robot.fk(q),[q],scene.all_obstacles,seed=71070,stage="pregrasp")
        solved=next(stream,None)
        add("native_ik_current_home",solved is not None,evidence=stream.evidence(),
            native=c.native_ik_evidence[-1] if c.native_ik_evidence else None)
        goal=q.copy();goal[0]+=.01
        path,failure,evidence=c._transit(q,goal,scene.all_obstacles,purpose=MotionPurpose.FREE_APPROACH,
            seed=71071,iteration_budget=1,stage="pregrasp",allow_rrt=False)
        add("native_ptp_small_motion",failure is None,failure=failure,evidence=evidence,
            path=[point.tolist() for point in path])
        first=list(path)
        if path and failure is None:
            destination=c.robot.fk(path[-1]);destination[2,3]+=.001
            suffix,failure,evidence=c._cartesian(path[-1],destination,scene.all_obstacles,
                purpose=MotionPurpose.FREE_APPROACH,seed=71072,stage="pregrasp")
            full=path+suffix[1:] if suffix else path
            coverage=audit_native_motion_coverage(full,c.native_verified,task_id=c.native_task_id)
            add("native_same_task_lin_append",failure is None and coverage["passed"],failure=failure,
                evidence=evidence,source_coverage=coverage)
            # A new sibling from the original start exercises MTC suffix removal
            # and native cache reuse. It does not rerun a historical subsolution.
            sibling=q.copy();sibling[0]-=.01
            branch,failed,branch_evidence=c._transit(q,sibling,scene.all_obstacles,
                purpose=MotionPurpose.FREE_APPROACH,seed=71073,iteration_budget=1,
                stage="pregrasp",allow_rrt=False)
            native_attempts=branch_evidence.get("attempts",[])
            backtracks=max((item.get("task_backtracks",0) for item in native_attempts),default=0)
            add("native_same_task_backtrack",failed is None and backtracks>0,
                failure=failed,task_backtracks=backtracks,evidence=branch_evidence)
        # A real OMPL request in the unchanged diagnostic world proves native
        # connectivity even when PTP solved the normal directed short motion.
        ompl,failed,ompl_evidence=c._native_plan(q,goal,scene.all_obstacles,seed=71074,
            stage="pregrasp",purpose=MotionPurpose.FREE_APPROACH,try_ptp=False,max_ompl_attempts=1)
        invoked=any(item.get("native_solver_calls",{}).get("OMPL",0)>0 for item in ompl_evidence.get("attempts",[]))
        add("native_ompl_independent_connectivity",invoked,failure=failed,evidence=ompl_evidence,
            solution_found=bool(ompl) and failed is None,counts_toward_single_carton=False,
            scope="direct native OMPL connectivity; normal-case fallback not forced")
        if args.ompl_counterexample:
            # Bounded obstacle search uses only current geometry and a newly
            # computed PTP path. Neither candidate states nor paths are fixtures.
            counterexample=None;trials=[]
            for joint,delta in ((4,.15),(4,-.15),(0,.15),(0,-.15)):
                endpoint=q.copy();endpoint[joint]+=delta
                fresh,failed,ev=c._native_plan(q,endpoint,scene.all_obstacles,seed=71080+len(trials),
                    stage="pregrasp",purpose=MotionPurpose.FREE_APPROACH,allow_ompl=False)
                trials.append(dict(joint=joint,delta=delta,failure=failed))
                if failed is not None or len(fresh)<3:
                    continue
                midpoint=np.asarray(fresh[len(fresh)//2])
                for box in c.tool_collision_obbs_provider(midpoint):
                    for corner in box.corners():
                        blocker=OBB(corner,np.full(3,.001),np.eye(3),"probe_obstacle","obstacle")
                        changed=[*scene.all_obstacles,blocker]
                        if c._state_failure(q,changed,stage="pregrasp") is not None or c._state_failure(endpoint,changed,stage="pregrasp") is not None:
                            continue
                        if c._state_failure(midpoint,changed,stage="pregrasp") is None:
                            continue
                        # Different world => explicitly separate development Task.
                        c.native_task_id="diagnostic-obstacle-"+digest([q.tolist(),endpoint.tolist(),corner.tolist()])[:24]
                        c.native_verified=[]
                        _,failed,ev=c._transit(q,endpoint,changed,purpose=MotionPurpose.FREE_APPROACH,
                            seed=71090,iteration_budget=1,stage="pregrasp",allow_rrt=True)
                        counterexample=dict(obstacle=dict(center=corner.tolist(),half_extents=[.001]*3),
                            endpoints=[q.tolist(),endpoint.tolist()],failure=failed,evidence=ev)
                        break
                    if counterexample is not None:break
                if counterexample is not None:break
            wired=counterexample is not None and counterexample["evidence"].get("native_ompl_calls",0)>0
            add("native_ompl_ptp_rejection_fallback_counterexample",wired,result=counterexample,
                bounded_candidate_search=trials,counts_toward_single_carton=False)
        # Contact rules are tested on a separately declared synthetic target at
        # the actual model's home TCP. Frozen target geometry is retained; pose
        # is deliberately diagnostic and is never passed to formal planning.
        physical=c.physical_from_virtual(c.robot.fk(q))
        rotation=physical[:3,[2,0,1]]
        center=physical[:3,3]+physical[:3,2]*target.half_extents[0]
        synthetic=OBB(center,target.half_extents,rotation,target.name,target.category)
        selected=c._contact_selection(q,synthetic,"front",policy.data["suction"])
        contact_results=[]
        for name,offset in (("seal",0.),("penetration",-.012)):
            box=OBB(center+physical[:3,2]*offset,target.half_extents,rotation,target.name,target.category)
            world=[box if item.name==target.name else item for item in scene.all_obstacles]
            request=c._build_native_request(q,None,world,seed=71100,stage="contact",target_contact=box,
                goal_pose=c.robot.fk(q))
            request.update(op="validate",probe_path=[q.tolist(),q.tolist()])
            result=c.native.request(request,timeout=c.native_request_timeout)
            contact_results.append(dict(case=name,offset_m=offset,result=result))
        valid=contact_results[0]["result"].get("process_valid") is True
        rejected=contact_results[1]["result"].get("process_valid") is False
        add("native_contact_seal_and_compression_counterexample",valid and rejected,
            scope="synthetic current-home target pose; process diagnostics only",results=contact_results,
            selected_count=sum(selected["commanded_active_mask"]),counts_toward_single_carton=False)
        # Cancellation poisons this real worker stream before any response can
        # be reused. It is the final operation on this resident worker.
        try:
            c.native.request(dict(op="fk",identity=c.native_identity,q_start=q.tolist(),links=["flange"]),cancelled=lambda:True)
        except MoveItUnavailable as exc:
            add("native_transport_cancel_poison",str(exc)=="CANCELLED" and c.native.process.poll() is not None,detail=str(exc))
        else:
            add("native_transport_cancel_poison",False,detail="cancelled request returned")
        expired=ResidentMoveItClient(os.environ.get("M710_MOVEIT_COMMAND"))
        try:
            expired.request(dict(op="fk",identity=c.native_identity,q_start=q.tolist(),links=["flange"]),timeout=0.)
        except MoveItUnavailable as exc:
            add("native_transport_expired_poison","TIMEOUT" in str(exc) and expired.process.poll() is not None,detail=str(exc),
                scope="expired IPC rejected before observing uninitialized worker response")
        finally:
            expired.close()
        report["status"]="PASS" if all(item["passed"] for item in report["checks"]) else "FAIL"
    except Exception as exc:
        report.update(status="ERROR",error=str(exc),traceback=traceback.format_exc())
        print(report["traceback"],flush=True)
    finally:
        if c is not None:
            report.update(native_stage_attempts=c.native_evidence,native_ik_attempts=c.native_ik_evidence,
                native_cold_audit=c.native_cold_evidence(),authority_paths=c.authority_path_evidence)
            c.native.close()
        save()
    return 0 if report["status"]=="PASS" else 1


if __name__=="__main__":
    raise SystemExit(main())
