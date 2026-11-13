"""Prospective endpoint probes; removed upper layers are never completion claims."""
from pathlib import Path
import sys,argparse,json,copy
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,ik_candidates,target_contact,save
from unloading_sim.s50_sim_collision import SimCollision
from unloading_sim.s50_kinematic_motion import posture


def main():
    p=argparse.ArgumentParser();p.add_argument('--gap',type=float,default=.1);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();settings=yaml.safe_load((ROOT/'configs/robots/huayan_s50_2000_kinematic_sim.yaml').read_text())
    settings['layout_gap_m']=a.gap;c=make_context(ROOT,a.output,settings);checker=SimCollision(c)
    result=dict(scope='PROSPECTIVE_ENDPOINTS_NOT_EXECUTED_TASKS',gap_m=a.gap,base=c.world[:3,3].tolist(),limits_deg=settings['joint_ranges_deg'],probes=[])
    seeds=[np.deg2rad(x) for x in [[160,-80,80,-90,70,180],[160,-30,-60,-90,70,180],[-20,-100,80,90,-70,180],[-20,-100,-80,90,-70,0],[60,-100,100,-90,-30,180]]]
    for name in ['carton_l07_c02','carton_l07_c00','carton_l07_c04','carton_l00_c02','carton_l00_c00','carton_l00_c04']:
        settings['target']=name;c.settings['target']=name;target,T=target_contact(c)
        c.active_cartons=tuple(b for b in c.layout.cartons() if b.center[2]<=target.center[2]+1e-8)
        for face in ['front','top']:
            pose=T.copy()
            if face=='top':pose[:3,:3]=np.diag([1.,-1.,-1.]);pose[:3,3]=target.center+[0,0,target.half_extents[2]]
            height=float(np.clip(target.center[2]-target.half_extents[2]-.1,.55,1.5))
            failures=[]
            def valid(q):
                good=checker.check(q,height,target.world_from_local,contact=True,phase='probe_'+face)
                if not good:failures.append(copy.deepcopy(checker.last))
                return good
            found=ik_candidates(c,pose,seeds=seeds,draws=48,stage=name+'_'+face,limit=6,valid=valid)
            row=dict(target=name,face=face,height_m=height,q_deg=[np.rad2deg(q).tolist() for q in found],postures=[posture(c.robot,q) for q in found],rejections=failures)
            result['probes'].append(row);save(a.output/'endpoint-probes.json',result)
            print(name,face,'solutions',len(found),json.dumps(row['q_deg']),flush=True)
    c.settings['target']='carton_l07_c02';target,T=target_contact(c);c.active_cartons=tuple(c.layout.cartons())
    for face in ['front','top']:
        pose=T.copy();box=target.world_from_local.copy();box[:3,3]=[-a.gap-.32,-.5,1.75]
        if face=='front':pose[:3,3]=box[:3,3]-[target.half_extents[0],0,0]
        else:pose[:3,:3]=np.diag([1.,-1.,-1.]);pose[:3,3]=box[:3,3]+[0,0,target.half_extents[2]]
        found=ik_candidates(c,pose,seeds=seeds,draws=64,stage='low_release_'+face,limit=8,
            valid=lambda q:checker.check(q,1.5,box,contact=True,phase='low_release_'+face))
        row=dict(target='LOW_RELEASE',face=face,q_deg=[np.rad2deg(q).tolist() for q in found],postures=[posture(c.robot,q) for q in found])
        result['probes'].append(row);save(a.output/'endpoint-probes.json',result)
        print('low release',face,len(found),json.dumps(row['q_deg']),flush=True)
    result['ik_attempts']=c.attempts;save(a.output/'endpoint-probes.json',result)

if __name__=='__main__':main()
