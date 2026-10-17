"""Four fixed-input IK API calls only; no motion operations, no results fed into formal tasks."""
import sys,json,os,hashlib
from pathlib import Path
from copy import deepcopy
import numpy as np
sys.path[:0]=["/work-single-contact-20261016/src","/work-single-contact-20261016/tools"]
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.single_contact import SingleContactClient
from unloading_sim.single_preplace import SinglePreplaceConnector
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.moveit2_backend import ResidentMoveItClient
from plan_m710_top_row_only import next_scene
root=Path("/work-single-contact-20261016")
o=Path("/tmp/preplace-ik-20261017")
old=Path("/tmp/single-preplace-20261016")
os.environ["M710_MOVEIT_SEED"]="71070"
policy=load_layout_motion_policy(root/"configs/validation/m710id70_proof_of_concept.yaml")
original=build_verified_motion_input(policy,root)
built=_build_automatic_trajectory_connector(original,policy.layout_validation.layout.robot())
rows=[]
for case in ("c02","c04"):
    # Read only the archived request's numeric input and actual capture, never output q.
    archive=json.loads((old/case/"result.json").read_text())
    row=archive["attempts"][0]
    rec=next(json.loads(line) for line in (old/case/"requests.jsonl").read_text().splitlines()
             if json.loads(line).get("op")=="ik" and json.loads(line).get("stage")=="transit")
    q=np.asarray(rec["q_start"])
    assert q.tolist()==row["extraction_end_q_rad"]
    scene=next_scene(original,archive["removed_cartons"],np.asarray(archive["start_q_rad"]),built.connector.robot)
    for mode in ("single_attempt","bounded_preplace"):
        label=case+"-"+mode
        client=SingleContactClient(ResidentMoveItClient(str(o/"ik-probe-worker.sh"),
            log_path=str(o/(label+".log"))),preplace_enabled=True)
        try:
            c=SinglePreplaceConnector.from_existing(built.connector,scene,native_client=client,
                planning_mode="cold_from_scratch",native_budget=dict(task_wall_time_s=300.,stage_wall_time_s=90.,ipc_timeout_s=100.))
            assert c.native_identity==archive["model_identity"]
            target=next(b for b in scene.cartons if b.name==archive["target_id"])
            t=simulation_contact_targets(scene,target,c.robot.base_transform[:3,3],c.budget.pregrasp_standoff_m)[0][0]
            assert t.face_role=="FRONT"
            c.expected_contact(t,policy.data["suction"])
            c._native_contact_context["selection"]=deepcopy(row["attachment"]["selection"])
            rigid=RigidAttachment(np.asarray(row["attachment"]["tcp_from_box"]),target.half_extents,target.name)
            a=PhysicalContactAttachment(c.robot,rigid,c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
            req=c._build_native_request(q,None,[b for b in scene.all_obstacles if b.name!=target.name],
                seed=71070,stage="transit",attachment=a,goal_pose=np.asarray(rec["goal_pose"]))
            # Exact archived numerical payload transform, no reconstruction roundoff.
            assert np.max(np.abs(np.asarray(req["attachment"]["pose"])-np.asarray(rec["attachment"]["pose"])))<1e-12
            req["attachment"]=deepcopy(rec["attachment"])
            req.update(op="ik",ik_mode=mode,allowed_planning_time_s=90.,ik_timeout_s=.25,
                position_tolerance_m=c.ik["position_tolerance_m"],
                orientation_tolerance_rad=c.ik["orientation_tolerance_rad"],
                collision_check_in_native_ik=False,purpose="FREE_LOADED_TRANSFER",
                parent_stage_id="fixed-input-extraction-not-a-live-receipt",ownership_transition=None)
            if mode=="bounded_preplace":req["solver_budget_s"]=.10
            assert req["q_start"]==rec["q_start"] and req["goal_pose"]==rec["goal_pose"]
            evidence=dict(case=case,mode=mode,input=req,base_transform=c.robot.base_transform.tolist(),
                joint_limits=c.robot.joint_limits.tolist() if hasattr(c.robot.joint_limits,"tolist") else c.robot.joint_limits)
            (o/(label+"-input.json")).write_text(json.dumps(evidence,indent=2)+"\n")
            raw=client.request(req,timeout=100.)
            evidence.update(response=raw,requests=client.records)
            (o/(label+"-result.json")).write_text(json.dumps(evidence,indent=2)+"\n")
            rows.append(dict(case=case,mode=mode,response=raw,round_trip_s=client.records[-1]["round_trip_s"]))
            print(label,raw["status"],raw["actual_ik_s"],raw["error_code"],flush=True)
        finally:client.close()
(o/"ik-comparison.json").write_text(json.dumps(dict(scope="FIXED_INPUT_IK_ONLY_NOT_FORMAL",
    internal_rng="KDL private RobotState RNG has no exposed plugin seed setter; OMPL seed 71070 is not its seed",
    rows=rows),indent=2)+"\n")
