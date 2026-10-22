"""Bind validated known-probe TCP calibration, then detect the current RGB-D boxes."""
import argparse
from pathlib import Path
from unloading_sim.eco65.model import load_json,save_json
from unloading_sim.eco65.native_io import file_hash
from unloading_sim.eco65.perception import signed,verify_record,require_tcp_calibration,detect
from unloading_sim.touch_calibration import validate_observations,validate_result,observation_binding,fingerprint


def run(a):
    obs=validate_observations(load_json(a.tcp_observations/'observations.json'))
    solver=validate_result(load_json(a.solver),obs);validation=load_json(a.validation)
    vision=verify_record(load_json(a.observations));out=a.output;out.mkdir(parents=True,exist_ok=False)
    if file_hash(a.tcp_observations/'observations.json')!=vision['calibration_observations_sha256']:
        raise ValueError('TCP_AND_RGBD_CAPTURE_MISMATCH')
    detections={r['sample_id']:r for r in solver['detections']}
    held={r['photo']:r['reprojection'] for r in validation['held_out_points']}
    views=[dict(sample_id=r['sample_id'],split=r['split'],rms_px=(detections[r['sample_id']] if r['split']=='train' else held[r['sample_id']])['reprojection_rms_px']) for r in obs['photos']]
    cal=signed(dict(schema='eco65_tcp_referenced_camera_calibration_v1',status='PASS',
        calibration_method='TCP_FK_REFERENCED_CAMERA_EXTRINSICS',
        source='SYNTHETIC_RENDERED_RGBD',snapshot_fingerprint=vision['snapshot_fingerprint'],
        installation_fingerprint=vision['installation_fingerprint'],observation_fingerprint=vision['fingerprint'],
        tcp_definition_fingerprint=fingerprint(obs['probe']),tcp_observation_binding=observation_binding(obs),
        tcp_observations=obs,tcp_solver_result=solver,tcp_heldout_validation=validation,
        T_link1_optical=solver['T_parent_child'],K=obs['camera']['K'],distortion=obs['camera']['distortion'],
        resolution=obs['camera']['resolution'],views=views,prior_calibration_reused=False,
        nominal_camera_transform_used_by_estimator=False,
        intrinsics_source='KNOWN_SIMULATION_PROJECTION_WITH_RENDERED_RAY_VALIDATION',
        physical_execution='NOT_RUN',execution_ready=False))
    require_tcp_calibration(cal);save_json(out/'calibration.json',cal)
    d=detect(a.observations,out/'calibration.json',out/'vision',selected_column_rank=a.target_column_rank);print('TCP_CALIBRATION_AND_VISION',d['status'])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target-column-rank',type=int,choices=(0,1,2),default=1)
    for key in ('observations','tcp-observations','solver','validation','output'):p.add_argument('--'+key,type=Path,required=True)
    run(p.parse_args())
