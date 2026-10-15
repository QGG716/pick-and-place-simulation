"""Development single-edge probe: fresh configured target, no archived endpoints."""
import sys,json,os,uuid
from pathlib import Path
import numpy as np
sys.path[:0]=["/work-approach-tier-20261015/tools","/work-approach-tier-20261015/src"]
import plan_m710_top_row_only as e
from unloading_sim.static_prior import placement_policy_fingerprint,request_context
from unloading_sim.moveit2_backend import native_generation_evidence,validate_native_result
root=Path("/work-approach-tier-20261015")
policy=e.load_layout_motion_policy(root/"configs/validation/m710id70_proof_of_concept.yaml")
scene=e.build_verified_motion_input(policy,root)
built=e._build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
c=e.PlanningOnlyConnector.from_existing(built.connector,scene,
 "/tmp/approach-tier-20261015/worker.sh",planning_mode="static_prior_fast",
 native_budget=dict(task_wall_time_s=300.,stage_wall_time_s=90.,ipc_timeout_s=100.))
try:
 c.static_prior_placement_policy_sha256=placement_policy_fingerprint(policy)
 c.load_static_prior(Path("/tmp/static-prior-20261003/static-prior-v3.json"))
 q=np.asarray(policy.layout_validation.initial_q).copy()
 target=e.rank(scene,e.RowUnloadingState(config=e.RowSequencePolicy()),c,q).candidates[0].carton
 c.robot_state_validator.contact_target_name=target.name
 state=c.native_root_state(q,scene.all_obstacles,target_contact=target)
 goal=q.copy();goal[0]+=.002
 req=c._build_native_request(state,goal,scene.all_obstacles,seed=71070,stage="pregrasp",target_contact=target)
 req.update(purpose="FREE_APPROACH",pipeline_id="static_prior",planner_id="CHECKED_JOINT_DIRECT",
   connection_scope="DIRECT_ONLY",parent_stage_id="",parent_state_receipt=state.native_receipt,
   stage_id=c.native_task_id+":probe",allowed_planning_time_s=6.)
 from unloading_sim.independent_cups import m710_cup_array_from_mapping
 req["process_policy"].update(cups=[cup.to_dict() for cup in m710_cup_array_from_mapping(policy.data["suction"]).cups],
   commanded_active_mask=[False]*72,geometrically_eligible_mask=[False]*72,actual_contact_mask=[False]*72,
   target_face="front",suction_edge_margin_m=float(policy.data["suction"].get("suction_edge_margin_m",0.)))
 req["prior_context"]=request_context(req,placement_policy_sha256=c.static_prior_placement_policy_sha256,
   joint_limits_rad=c.robot.joint_limits.tolist())
 raw=c.native.request(req,timeout=100)
 assert raw["status"]=="SUCCESS",raw
 assert native_generation_evidence(raw,req,planning_mode=c.planning_mode)
 path=validate_native_result(raw,q,goal,c.native_joint_names)
 assert raw["mtc_generation"] and raw["mtc_task_solution_count"]==1
 assert raw["prior_usage"]["graph_search_performed"] is False
 assert raw["prior_usage"]["edge_checks"]==1 and raw["native_solver_calls"]=={}
 assert raw["clearance"]["workspace"]["pair_profile_enabled"] is False
 output=dict(status="PLANNING_ONLY_NOT_EXECUTABLE",qualification_status="NOT_EVALUATED",
   continuous_swept_proof=False,scope="DEVELOPMENT_SINGLE_EDGE_NOT_C04_TASK",
   start=q.tolist(),goal=goal.tolist(),target=target.name,
   actual_last=path[-1].tolist(),response={k:v for k,v in raw.items() if k not in {"points","pipeline_calls"}})
 if isinstance(output["response"].get("clearance"),dict):output["response"]["clearance"].pop("policy",None)
 Path("/tmp/approach-tier-20261015/probe-direct.json").write_text(json.dumps(output,indent=2))
 print(json.dumps(dict(status=raw["status"],source=raw["generation_source"],points=len(path),
   query_s=raw["prior_query_s"],task_id=raw["task_id"],stage_id=raw["stage_id"],
   edge_checks=raw["prior_usage"]["edge_checks"],graph_search=False,native_solver_calls=raw["native_solver_calls"])))
finally:c.native.close()
