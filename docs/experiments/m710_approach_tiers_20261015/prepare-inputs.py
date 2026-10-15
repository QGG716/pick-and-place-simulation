import sys,json,hashlib,shutil
from pathlib import Path
import numpy as np
sys.path[:0]=["/work-approach-tier-20261015/tools","/work-approach-tier-20261015/src"]
import plan_m710_top_row_only as e
root=Path("/work-approach-tier-20261015")
out=Path("/tmp/approach-tier-20261015")
policy=e.load_layout_motion_policy(root/"configs/validation/m710id70_proof_of_concept.yaml")
scene=e.build_verified_motion_input(policy,root)
built=e._build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
robot=built.connector.robot
q=np.asarray(policy.layout_validation.initial_q).copy()
target=e.rank(scene,e.RowUnloadingState(config=e.RowSequencePolicy()),built.connector,q).candidates[0].carton
assert target.name=="carton_l07_c02"
(out/"c02-input.json").write_text(json.dumps(dict(target=target.name,start_q_rad=q.tolist(),
 removed_targets=[],seed_offset=0,planning_phase="FAST",source="CURRENT_CONFIG_INITIAL_STATE"),indent=2))
shutil.copyfile("/tmp/c04-cost-20261015/diagnostic-input.json",out/"c04-input.json")
records=[json.loads(x) for x in Path("/tmp/c04-cost-20261015/diagnostic-once/requests.jsonl").read_text().splitlines()]
pre=[r for r in records if r["stage"]=="pregrasp" and r["pipeline_id"]=="static_prior"]
assert len(pre)==2
a,b=[np.asarray(r["q_goal"]) for r in pre]
pa,pb=robot.fk(a),robot.fk(b)
result=dict(scope="HISTORICAL_INPUT_SEMANTICS_ONLY_NOT_A_PLANNING_INPUT",
 same_start=pre[0]["q_start"]==pre[1]["q_start"],
 same_task_parent=(pre[0]["task_id"],pre[0]["parent_stage_id"])==(pre[1]["task_id"],pre[1]["parent_stage_id"]),
 goal_delta_q_rad=(b-a).tolist(),goal_fk_position_delta_m=float(np.linalg.norm(pa[:3,3]-pb[:3,3])),
 goal_fk_orientation_delta_rad=float(np.arccos(np.clip((np.trace(pa[:3,:3].T@pb[:3,:3])-1)/2,-1,1))),
 connection_seed_delta=pre[1]["seed"]-pre[0]["seed"],
 conclusion="same _connect_pose fixed pose, alternate wrist IK configurations; same target/process context")
(out/"candidate-semantics.json").write_text(json.dumps(result,indent=2))
print(json.dumps(result))
