"""Audit every shoulder/elbow branch for the recorded fixed-orientation endpoints.

Position-only inverse geometry is derived from the same official native URDF;
every resulting arm branch is checked by the production Pinocchio FK. This is
not a second FK, a motion planner, a relaxed-limit acceptance, or a swept proof.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from time import perf_counter
import xml.etree.ElementTree as ET


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--core-root',type=Path,required=True)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    sys.path[:0]=[str(args.core_root/'tools'),str(args.core_root/'src')]
    import numpy as np
    from plan_m710_mount_comparison import make_context,FLAGS
    from plan_m710_top_layer_lifted import save
    began=perf_counter()
    _,c,_=make_context('xy_minus_100');robot=c.robot
    assert type(robot).__name__=='PinocchioHppFclBackend'
    limits=robot.joint_limits;zero=robot.named_link_frames(np.zeros(6))
    base_inverse=np.linalg.inv(robot.base_transform)
    origin={n:(base_inverse@zero[n])[:3,3] for n in ('J1_link','J2_link','J3_link','J4_link','J5_link','J6_link','flange')}
    shoulder_offset=origin['J2_link'][0]-origin['J1_link'][0]
    upper=origin['J3_link'][2]-origin['J2_link'][2]
    elbow_offset=origin['J4_link'][2]-origin['J3_link'][2]
    forearm=origin['J5_link'][0]-origin['J4_link'][0]
    flange_from_wrist=np.linalg.inv(zero['flange'])@zero['J5_link']
    assert np.allclose([shoulder_offset,upper,elbow_offset,forearm],[.15,.895,.17,1.045],atol=1e-12,rtol=0)
    assert np.allclose(flange_from_wrist[:3,3],[-.175,0,0],atol=1e-12,rtol=0)
    assert np.allclose(origin['J5_link'],origin['J6_link'],atol=1e-12,rtol=0)
    lower=math.hypot(forearm,elbow_offset);offset_angle=math.atan2(forearm,elbow_offset)
    def aliases(angle,j):
        # Full coverage for this model's literal finite bounds (all within +/- 400 deg).
        return [angle+2*math.pi*k for k in range(-3,4) if limits[j,0]-1e-12<=angle+2*math.pi*k<=limits[j,1]+1e-12]
    def audit(req):
        goal=np.asarray(req['goal_pose'])
        flange=goal@np.linalg.inv(np.asarray(req['flange_from_task_tcp']))
        wrist_world=(flange@flange_from_wrist)[:3,3]
        wrist=(base_inverse@np.r_[wrist_world,1.])[:3]
        x,y,z=wrist-origin['J1_link'];radius=math.hypot(x,y)
        branches=[]
        for shoulder_sign in (1,-1):
            q1=math.atan2(y,x)+(0 if shoulder_sign==1 else math.pi)
            X=shoulder_sign*radius-shoulder_offset;Z=z
            cosine=(X*X+Z*Z-upper*upper-lower*lower)/(2*upper*lower)
            if abs(cosine)>1+1e-12:
                branches.append(dict(shoulder_sign=shoulder_sign,outside_unbounded_two_link_reach=True,cosine=cosine));continue
            for elbow_sign in (1,-1):
                bend=elbow_sign*math.acos(max(-1.,min(1.,cosine)))
                q3=offset_angle-bend
                q2=math.atan2(X,Z)-math.atan2(lower*math.sin(bend),upper+lower*math.cos(bend))
                raw=[q1,q2,q3];legal=[aliases(q,j) for j,q in enumerate(raw)]
                representative=[xs[0] if xs else min((q+2*math.pi*k for k in range(-3,4)),key=lambda v:max(limits[j,0]-v,v-limits[j,1],0))
                                for j,(q,xs) in enumerate(zip(raw,legal))]
                q=np.r_[representative,[0.,0.,0.]]
                actual=robot.named_link_frames(q)['J5_link'][:3,3]
                error=float(np.linalg.norm(actual-wrist_world))
                assert error<1e-9, (representative,error)
                branches.append(dict(shoulder_sign=shoulder_sign,elbow_sign=elbow_sign,q123_rad=representative,
                    q123_deg=np.rad2deg(representative).tolist(),legal_angle_aliases_rad=legal,
                    arm_within_limits=all(legal),violating_joints=[f'J{j+1}' for j,xs in enumerate(legal) if not xs],
                    production_wrist_fk_error_m=error))
        return dict(stage_id=req['stage_id'],goal_pose=req['goal_pose'],wrist_center_world_m=wrist_world.tolist(),
                    branches=branches,legal_arm_branch_count=sum(bool(b.get('arm_within_limits')) for b in branches))
    args.output.mkdir(parents=True,exist_ok=False)
    cases=[]
    for carton in ('carton_l05_c03','carton_l04_c02'):
        path=args.run/carton/'attempt-001/search-trace.json';trace=json.loads(path.read_text())
        urdf=ET.fromstring(trace['model']['parameters']['robot_description'])
        for name,axis in [('J1',[0,0,1]),('J2',[0,1,0]),('J3',[0,-1,0]),('J4',[-1,0,0])]:
            element=urdf.find(f"joint[@name='{name}']")
            assert np.allclose(list(map(float,element.find('axis').get('xyz').split())),axis,atol=1e-12,rtol=0)
        requests=[a['request'] for a in trace['attempts'] if a['request']['stage']=='extraction']
        rows=[audit(req) for req in requests]
        assert rows
        # Positive control: the actually accepted contact must have a legal arm
        # branch, and enumeration must recover its real J1/J2/J3 modulo 2*pi.
        actual_contact=np.asarray(requests[0]['q_start'])
        control=audit(dict(requests[0],goal_pose=robot.fk(actual_contact).tolist(),
                           stage_id='accepted_contact_positive_control'))
        matches=[b for b in control['branches'] if b.get('arm_within_limits') and
            np.linalg.norm(np.arctan2(np.sin(np.asarray(b['q123_rad'])-actual_contact[:3]),
                                     np.cos(np.asarray(b['q123_rad'])-actual_contact[:3])))<1e-8]
        assert matches, control
        assert robot.within_limits(actual_contact)
        wrist_at_contact=robot.named_link_frames(actual_contact)['J5_link'][:3,3]
        assert np.linalg.norm(wrist_at_contact-np.asarray(control['wrist_center_world_m']))<1e-9
        control['accepted_contact_q_rad']=actual_contact.tolist()
        control['actual_arm_branch_recovered']=True
        first=rows[0]
        case=dict(target_id=carton,source_trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            endpoints_checked=len(rows),endpoints_with_legal_arm_branch=sum(r['legal_arm_branch_count']>0 for r in rows),
            official_limits_deg=np.rad2deg(limits).tolist(),geometry=dict(shoulder_offset_m=shoulder_offset,
            upper_arm_m=upper,elbow_offset_m=elbow_offset,forearm_m=forearm,flange_from_wrist=flange_from_wrist.tolist()),
            first_endpoint=first,all_endpoints=rows,accepted_contact_positive_control=control,
            max_production_wrist_fk_error_m=max(b['production_wrist_fk_error_m'] for r in rows
                for b in r['branches'] if 'production_wrist_fk_error_m' in b),**FLAGS)
        save(args.output/(carton+'.json'),case)
        cases.append({k:case[k] for k in ('target_id','source_trace_sha256','endpoints_checked','endpoints_with_legal_arm_branch','official_limits_deg','geometry','first_endpoint','accepted_contact_positive_control','max_production_wrist_fk_error_m')})
        print(json.dumps(cases[-1]),flush=True)
    save(args.output/'summary.json',dict(cases=cases,wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        elapsed_s=perf_counter()-began,
        scope='ALL_RECORDED_ENDPOINT_ARM_BRANCHES_ONLY_NOT_ALL_GRASPS_OR_PROCESS_OPTIONS',**FLAGS))


if __name__=='__main__':main()
