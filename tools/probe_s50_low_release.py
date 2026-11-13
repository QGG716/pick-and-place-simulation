"""Diagnose low receiving poses without changing the fixed gap or joint limits."""
from pathlib import Path
import sys,json,copy,argparse
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,ik_candidates,target_contact,save
from unloading_sim.s50_sim_collision import SimCollision
from unloading_sim.s50_kinematic_motion import posture

p=argparse.ArgumentParser();p.add_argument('--gap',type=float,default=.1);p.add_argument('--yaw',type=float,default=0.);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
settings=yaml.safe_load((ROOT/'configs/robots/huayan_s50_2000_kinematic_sim.yaml').read_text());settings['layout_gap_m']=a.gap
c=make_context(ROOT,a.output,settings);checker=SimCollision(c);target,T=target_contact(c);rows=[]
seeds=[np.deg2rad(x) for x in [[10,-50,100,-230,-80,180],[170,-50,-100,-60,80,180],[60,-100,100,-90,-30,180],[240,-100,100,-90,70,0]]]
relative=np.linalg.inv(T)@target.world_from_local
from unloading_sim.geometry import rotation_matrix_from_rpy
for y in [-.5,.5,-.8,.8,0.]:
    for z in ([1.70] if a.yaw else [1.70,1.75,1.85]):
        box=target.world_from_local.copy();box[:3,3]=[-a.gap-.35,y,z];box[:3,:3]=rotation_matrix_from_rpy(0,0,np.deg2rad(a.yaw));pose=box@np.linalg.inv(relative)
        found=ik_candidates(c,pose,seeds=seeds,draws=24,stage=f'release_{y}_{z}',limit=10)
        result=[]
        for q in found:
            good=checker.check(q,1.5,box,True,'release_probe');result.append(dict(q_deg=np.rad2deg(q).tolist(),passed=good,posture=posture(c.robot,q),collision=copy.deepcopy(checker.last)))
        rows.append(dict(y=y,z=z,solutions=result));save(a.output/f'low-release-yaw-{a.yaw:g}.json',dict(gap_m=a.gap,yaw_deg=a.yaw,rows=rows,ik_attempts=c.attempts))
        print(y,z,'PASS',sum(x['passed'] for x in result),'/',len(result),'hits',[x['collision']['pair'] for x in result if x['collision']],flush=True)
