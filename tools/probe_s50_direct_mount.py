"""Actual same-centre-carton contact/withdrawal checks for a direct mount.

This does not qualify an invalid home. Explicit roll values are separate scene
conventions, not hidden per-face business candidate scans or executed motions.
"""
from pathlib import Path
import sys,yaml,argparse,copy,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,ik_candidates,simulation_coordinates,save
from unloading_sim.s50_face_kinematic import FaceRun


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--roll',type=float,default=0.);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();settings=yaml.safe_load((ROOT/'configs/robots/huayan_s50_2000_kinematic_sim.yaml').read_text(encoding='utf8'))
    settings['task_front_tool_roll_deg']=a.roll;settings['layout_gap_m']=.1
    c=make_context(ROOT,a.output,settings);run=FaceRun(c);run.target=run.active['carton_l07_c02'];run.box=run.target.world_from_local.copy();run.height=1.5
    target=np.eye(4);target[:3,:3]=run.front;target[:3,3]=run.target.center-[.3,0,0]
    seeds=[simulation_coordinates(settings,np.deg2rad(q)) for q in [[10,-40,100,-240,-80,180],[160,-90,-80,-90,90,180],[160,-120,60,-190,90,200],[160,-80,-60,50,-90,20]]]
    found=ik_candidates(c,target,seeds=seeds,draws=128,stage='carton_l07_c02_front_contact',limit=12,valid=lambda q:run.valid(q,'grasp_candidate'))
    results=[]
    for q in found:
        relative=np.linalg.inv(c.robot.fk(q))@run.box
        try:
            extraction=run.extraction(q,relative);results.append(dict(status='STRAIGHT_EXTRACTION_PASSED',q_rad=q.tolist(),path_rad=[x.tolist() for x in extraction['path']],preference=run.withdrawal_score(extraction['path'])))
        except RuntimeError as e:results.append(dict(status='FAILED',q_rad=q.tolist(),reason=str(e),detail=copy.deepcopy(run.last_pointwise_failure)))
    save(a.output/'contact-withdrawal.json',dict(scope='EXPLICIT_MOUNT_ORIENTATION_DIAGNOSTIC_NOT_HOME_OR_TASK_ACCEPTANCE',settings=settings,
        target=run.target.name,grasp_pose=target.tolist(),contact_solutions=len(found),extractions=results,ik_attempts=c.attempts,failures=run.failures))
    print('roll',a.roll,'contact solutions',len(found),'withdrawal passes',sum(r['status']=='STRAIGHT_EXTRACTION_PASSED' for r in results),flush=True)


if __name__=='__main__':main()
