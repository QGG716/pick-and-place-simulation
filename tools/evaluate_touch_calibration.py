"""Read held-out observations and independent truth only AFTER extrinsics solve."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from unloading_sim.touch_calibration import fk,points,camera_from_board,read_observation_image


def evaluate(observations,result,truth,output):
    import cv2
    started=time.perf_counter()
    obs=json.loads((observations/'observations.json').read_text());r=json.loads(result.read_text());truth=json.loads(truth.read_text())
    output.mkdir(parents=True,exist_ok=False);X=np.array(r['T_parent_child']);true=np.array(truth['T_L_C']);th=obs['thresholds'];records=[]
    truth_error=dict(translation_xyz_m=(X[:3,3]-true[:3,3]).tolist(),translation_m=float(np.linalg.norm(X[:3,3]-true[:3,3])),
        rotation_deg=float(np.degrees(Rotation.from_matrix(true[:3,:3].T@X[:3,:3]).magnitude())))
    for photo in obs['photos']:
        if photo['split']!='validation':continue
        touches=[t for t in obs['touches'] if t['board_placement_id']==photo['board_placement_id'] and t['split']=='validation']
        if len(touches)<3:raise ValueError('INDEPENDENT_THREE_TOUCH_POINTS_REQUIRED')
        image=read_observation_image(observations,photo,obs['camera']);C,diagnostic=camera_from_board(image,obs['board'],np.array(obs['camera']['K']),th,output/(photo['sample_id']+'_detected.png'))
        D=fk(obs['kinematics'],photo['q_rad'])['link_1']@X@C
        for t in touches:
            p=next(p['p_P_m'] for p in obs['board']['touch_points'] if p['point_id']==t['point_id'])
            measured=points(fk(obs['kinematics'],t['q_rad'])['link_6'],obs['probe']['p_F_tip_m']);predicted=points(D,p);delta=predicted-measured
            records.append(dict(photo=photo['sample_id'],point=t['point_id'],board_placement_id=t['board_placement_id'],
                predicted_D_m=predicted.tolist(),FK_touch_D_m=measured.tolist(),error_xyz_m=delta.tolist(),error_3d_m=float(np.linalg.norm(delta)),reprojection=diagnostic))
    if not records:raise ValueError('NO_HELD_OUT_DATA')
    consistency=[]
    for placement,pid in sorted({(r['board_placement_id'],r['point']) for r in records}):
        p=np.array([r['predicted_D_m'] for r in records if r['point']==pid and r['board_placement_id']==placement]);d=np.linalg.norm(p[:,None]-p[None,:],axis=2)
        consistency.append(dict(board_placement_id=placement,point_id=pid,max_pairwise_m=float(d.max())))
    max_error=max(r['error_3d_m'] for r in records)
    passed=truth_error['translation_m']<=th['truth_translation_m'] and truth_error['rotation_deg']<=th['truth_rotation_deg'] and max_error<=th['validation_point_m'] and max(x['max_pairwise_m'] for x in consistency)<=th['validation_point_m']
    report=dict(status='PASS' if passed else 'FAIL',extrinsic_truth_error=truth_error,held_out_points=records,
        held_out_max_3d_m=max_error,multi_J1_consistency=consistency,thresholds=th,known_perturbation=truth['perturbation'],
        conclusion='Rendered RGB and synthetic known-tip geometric touches only; no probe self-calibration, physical touch, continuous paths or hardware qualification')
    report['seconds']=time.perf_counter()-started
    (output/'validation.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    validated=dict(r,status='SPECIFIED_SIMULATION_VALIDATION_PASSED' if passed else 'SIMULATION_VALIDATION_FAILED',
        validation_status=dict(touch_registration='PASS',image_reprojection='PASS',independent_held_out=report['status'],
            extrinsic_truth=report['status'],probe_TCP_self_calibration='NOT_RUN',physical_contact='NOT_EVALUATED',hardware='NOT_RUN'),
        validation_summary=dict(extrinsic_truth_error=truth_error,held_out_max_3d_m=max_error,multi_J1_consistency=consistency),
        source_result_sha256=hashlib.sha256(result.read_bytes()).hexdigest())
    (output/'validated_extrinsics.json').write_text(json.dumps(validated,indent=2,allow_nan=False));print(report['status'],truth_error,'held_out_max_m',max_error)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['observations','result','truth','output']:p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();evaluate(a.observations,a.result,a.truth,a.output)
