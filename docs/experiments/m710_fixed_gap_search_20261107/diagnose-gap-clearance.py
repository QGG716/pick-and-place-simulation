import json,sys,time
from pathlib import Path
import numpy as np
sys.path[:0]=['/work-gap-20261107-v1/src','/work-gap-20261107-v1/tools']
from plan_m710_mount_comparison import make_context
from unloading_sim.ik import solve_ik_multistart
b=Path('/gap-search-20261107');d=json.loads((b/'run-v1/checkpoint.json').read_text());r=d['layouts'][d['best']]
o,built,_=make_context('gap',layout_path=Path('/work-gap-20261107-v1/configs/workcells')/r['layout_path'],initial_q=r['initial']['q_rad'])
a=json.loads((b/'endpoint-kinematic-diagnostic.json').read_text())[0];results=[]
for label,x,y in [('CLEAR_X_ONLY',-.575,0.),('RELEASE_RIGHT_LANE',-.7777,-.84),('RELEASE_LEFT_LANE',-.7777,.84)]:
 goal=np.asarray(a['goal_pose']);goal[0,3]=x;goal[1,3]=y;began=time.monotonic()
 v=solve_ik_multistart(built.robot,goal,[np.asarray(a['contact_q_rad'])],random_restarts=48,rng=np.random.default_rng(71153),obstacles=[],max_iterations=400,position_tolerance=1e-4,orientation_tolerance=1e-4)
 result=dict(label=label,goal=goal.tolist(),success=bool(v.success),position_error_m=v.position_error,orientation_error_rad=v.orientation_error,search=v.search_evidence,wall_s=time.monotonic()-began,scope='KINEMATICS_ONLY_NO_COLLISION_NO_MOTION_NO_LEDGER_CHANGE',global_unreachable=False,fed_to_native=False);results.append(result);print(json.dumps(result),flush=True)
(b/'clearance-kinematic-diagnostic.json').write_text(json.dumps(results,indent=2))
