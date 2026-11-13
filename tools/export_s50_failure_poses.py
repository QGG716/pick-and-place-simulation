"""Prepare separately labelled, unexecuted collision poses for a task replay."""
from pathlib import Path
import sys,argparse,json,hashlib,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,save


def export(cycle_file,recovery_file,output):
    source=cycle_file.read_bytes();cycle=json.loads(source);recovery=json.loads(recovery_file.read_text(encoding='utf8'))
    if recovery['environment_fingerprint']!=cycle['environment_fingerprint']:raise ValueError('FAILURE_POSE_SCENE_CHANGED')
    if recovery['completed_carton_ids']!=cycle['completed_carton_ids']:raise ValueError('FAILURE_POSE_WORLD_STATE_CHANGED')
    c=make_context(ROOT,ROOT/'outputs/s50_failure_pose_model',cycle['settings'])
    target=next(b for b in c.layout.cartons() if b.name==recovery['target']);poses=[]
    for branch in recovery['branches']:
        detail=branch.get('detail') or {};collision=detail.get('collision')
        if not collision:continue
        q=np.asarray(detail['q_rad']);contact=c.robot.fk(q)
        if 'pregrasp' in detail['phase']:body=target.world_from_local
        else:body=contact@np.linalg.inv(c.robot.fk(branch['q_rad']))@target.world_from_local
        poses.append(dict(target=target.name,q_rad=q.tolist(),world_from_carton=body.tolist(),world_from_physical_contact=contact.tolist(),
            reason=detail['reason'],phase=detail['phase'],pair=collision['pair'],contact_points=collision['contact_points'],
            point_index=detail['point_index'],point_count=detail['point_count'],position_error_m=detail['position_error_m'],
            scope='REJECTED_IK_CANDIDATE_NOT_EXECUTED_NOT_COMPLETED',source_grasp_q_rad=branch['q_rad']))
    result=dict(source_report_sha256=hashlib.sha256(source).hexdigest(),source_recovery_sha256=hashlib.sha256(recovery_file.read_bytes()).hexdigest(),
        completed_cartons=cycle['completed_cartons'],remaining_cartons=cycle['remaining_cartons'],poses=poses)
    save(output,result);print('exported rejected poses',len(poses))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('cycle',type=Path);p.add_argument('recovery',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();export(a.cycle,a.recovery,a.output)
