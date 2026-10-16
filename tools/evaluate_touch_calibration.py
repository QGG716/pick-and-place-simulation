"""Bound-result acceptance; truth is read only by this independent evaluator."""
import argparse,json,time,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from unloading_sim.touch_calibration import (fk,points,rigid,camera_from_board,read_observation_image,
    validate_observations,validate_result,coverage,training_evidence,TRAINING_CHECKS,observation_binding)


def evaluate(observations,result,truth,output):
    observations,result,truth,output=map(Path,(observations,result,truth,output))
    output.mkdir(parents=True,exist_ok=False);started=time.perf_counter()
    checks={name:dict(status='NOT_EVALUATED') for name in ('input_contract','training_coverage',*TRAINING_CHECKS,
        'result_reproduction','held_out_coverage','held_out_point_error','held_out_multi_J1_consistency','simulation_truth_error')}
    report=dict(status='FAIL',checks=checks,motion_geometry_qualification='NOT_EVALUATED_BY_CALIBRATION_ESTIMATOR',
        conclusion='Rendered RGB and synthetic known-tip touches only; no physical contact, paths or hardware qualification')
    r=None;active_check='input_contract'
    try:
        obs=validate_observations(json.loads((observations/'observations.json').read_text(encoding='utf8')))
        r=json.loads(result.read_text(encoding='utf8'));validate_result(r,obs)
        X=rigid(r['T_parent_child']);th=obs['thresholds'];checks['input_contract']=dict(status='PASS')
        report.update(observation_binding=observation_binding(obs),thresholds=th,source_result_sha256=hashlib.sha256(result.read_bytes()).hexdigest())
        active_check='training_coverage';checks['training_coverage']=coverage(obs,'train')
        active_check='training_image_reprojection'
        evidence=training_evidence(obs,observations,output)
        report['recomputed_training_evidence']=evidence;checks.update(evidence['training_checks'])
        reproduction=float(np.max(np.abs(X-np.array(evidence['T_parent_child']))))
        checks['result_reproduction']=dict(status='PASS' if reproduction<=1e-9 else 'FAIL',maximum_matrix_difference=reproduction,threshold=1e-9)
        active_check='held_out_coverage';checks['held_out_coverage']=coverage(obs,'validation')
        active_check='held_out_point_error'
        local={p['point_id']:p['p_P_m'] for p in obs['board']['touch_points']};records=[]
        for photo in obs['photos']:
            if photo['split']!='validation':continue
            touches=[t for t in obs['touches'] if t['board_placement_id']==photo['board_placement_id']]
            image=read_observation_image(observations,photo,obs['camera'])
            C,diagnostic=camera_from_board(image,obs['board'],np.array(obs['camera']['K']),th,output/(photo['sample_id']+'_detected.png'))
            D=fk(obs['kinematics'],photo['q_rad'])['link_1']@X@C
            for t in touches:
                measured=points(fk(obs['kinematics'],t['q_rad'])['link_6'],obs['probe']['p_F_tip_m'])
                predicted=points(D,local[t['point_id']]);delta=predicted-measured
                records.append(dict(photo=photo['sample_id'],point=t['point_id'],board_placement_id=t['board_placement_id'],
                    predicted_D_m=predicted.tolist(),FK_touch_D_m=measured.tolist(),error_xyz_m=delta.tolist(),
                    error_3d_m=float(np.linalg.norm(delta)),reprojection=diagnostic))
        consistency=[]
        for placement,pid in sorted({(row['board_placement_id'],row['point']) for row in records}):
            p=np.array([row['predicted_D_m'] for row in records if row['point']==pid and row['board_placement_id']==placement])
            # coverage() already proved three DISTINCT J1 views of each placement.
            d=np.linalg.norm(p[:,None]-p[None,:],axis=2)
            consistency.append(dict(board_placement_id=placement,point_id=pid,max_pairwise_m=float(d.max())))
        max_error=max(row['error_3d_m'] for row in records);spread=max(row['max_pairwise_m'] for row in consistency)
        checks['held_out_point_error']=dict(status='PASS' if max_error<=th['validation_point_m'] else 'FAIL',maximum_m=max_error,threshold_m=th['validation_point_m'])
        checks['held_out_multi_J1_consistency']=dict(status='PASS' if spread<=th['validation_point_m'] else 'FAIL',maximum_m=spread,threshold_m=th['validation_point_m'])
        report.update(held_out_points=records,held_out_max_3d_m=max_error,multi_J1_consistency=consistency)
        active_check='simulation_truth_error'
        truth_data=json.loads(truth.read_text(encoding='utf8'));true=rigid(truth_data['T_L_C'])
        truth_error=dict(translation_xyz_m=(X[:3,3]-true[:3,3]).tolist(),translation_m=float(np.linalg.norm(X[:3,3]-true[:3,3])),
            rotation_deg=float(np.degrees(Rotation.from_matrix(true[:3,:3].T@X[:3,:3]).magnitude())))
        checks['simulation_truth_error']=dict(status='PASS' if truth_error['translation_m']<=th['truth_translation_m'] and truth_error['rotation_deg']<=th['truth_rotation_deg'] else 'FAIL',
            **truth_error,threshold_m=th['truth_translation_m'],threshold_deg=th['truth_rotation_deg'])
        report.update(extrinsic_truth_error=truth_error,known_perturbation=truth_data['perturbation'],truth_file_sha256=hashlib.sha256(truth.read_bytes()).hexdigest())
        report['status']='PASS' if all(c['status']=='PASS' for c in checks.values()) else 'FAIL'
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        checks[active_check]=dict(status='FAIL',error=report['error'])
        report['failure_kind']='INPUT_COVERAGE_OR_RECOMPUTATION_FAILED'
    report['seconds']=time.perf_counter()-started
    (output/'validation.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf8')
    if r is not None:
        validated=dict(r,status='SPECIFIED_SIMULATION_VALIDATION_PASSED' if report['status']=='PASS' else 'SIMULATION_VALIDATION_FAILED',
            validation_status={key:value['status'] for key,value in checks.items()},validation_summary=report,
            motion_geometry_qualification='NOT_EVALUATED_BY_CALIBRATION_ESTIMATOR')
        try:serialized=json.dumps(validated,indent=2,allow_nan=False)
        except (ValueError,TypeError):serialized=None
        if serialized:(output/'validated_extrinsics.json').write_text(serialized,encoding='utf8')
    print(report['status'],report.get('error',report.get('extrinsic_truth_error')))
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['observations','result','truth','output']:p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args(argv)
    try:return 0 if evaluate(a.observations,a.result,a.truth,a.output)['status']=='PASS' else 1
    except Exception as error:
        print('FAIL',type(error).__name__,str(error));return 2


if __name__=='__main__':raise SystemExit(main())
