import json,tarfile,sys,time
from pathlib import Path
import numpy as np
sys.path[:0]=['/work-gap-20261107-v1/src','/work-gap-20261107-v1/tools']
from plan_m710_mount_comparison import make_context
from unloading_sim.ik import solve_ik_multistart

run=Path('/gap-search-20261107/run-v1')
db=json.loads((run/'checkpoint.json').read_text())
record=db['layouts'][db['best']]
original,built,audit=make_context('gap',layout_path=Path('/work-gap-20261107-v1/configs/workcells')/record['layout_path'],initial_q=record['initial']['q_rad'])
results=[]
for name in ('carton_l07_c02','carton_l07_c03'):
    trial=next(r for r in reversed(db['trials']) if r['layout']==db['best'] and r['target_id']==name and r.get('contact_complete') and not r['complete'])
    p=run/trial['evidence']['archive']
    with tarfile.open(p) as t:
        calls=json.load(t.extractfile(p.name.removesuffix('.tar.gz')+'/requests.json'))
    call=next(c for c in calls if c.get('stage')=='extraction' and c.get('goal_pose'))
    q=np.asarray(call['q_start']);goal=np.asarray(call['goal_pose'])
    started=time.monotonic()
    result=solve_ik_multistart(built.robot,goal,[q],random_restarts=48,rng=np.random.default_rng(71151),
        obstacles=[],max_iterations=400,position_tolerance=1e-4,orientation_tolerance=1e-4)
    row=dict(target=name,trial=trial['trial_number'],gap_m=record['gap_m'],layout=trial['layout'],goal_pose=goal.tolist(),contact_q_rad=q.tolist(),
        success=bool(result.success),q_rad=result.q.tolist(),position_error_m=result.position_error,
        orientation_error_rad=result.orientation_error,search=result.search_evidence,wall_s=time.monotonic()-started,
        scope='CPU_KINEMATICS_ONLY_NO_COLLISION_NO_MOTION_NO_LEDGER_CHANGE',
        fed_to_native_planner=False,global_unreachable=False)
    results.append(row);print(json.dumps(row),flush=True)
Path('/gap-search-20261107/endpoint-kinematic-diagnostic.json').write_text(json.dumps(results,indent=2))
