"""Image/FK/estimated-extrinsics target formation; no scene or truth input."""
import numpy as np
from unloading_sim.touch_calibration import (validate_observations,validate_result,observation_binding,
    camera_from_board,fk,points,rigid)


def nominal_camera_point(scene, T_D_link1, p_camera):
    """Explicit changed-installation simulation input, never calibrated PnP evidence."""
    from unloading_sim.eco65.camera_installation import camera_world_transform
    T=camera_world_transform(scene,T_D_link1)
    return dict(p_D_target_nominal=points(T,p_camera).tolist(),
        T_D_camera_optical=T.tolist(),calibration=scene['camera_extrinsics'],
        source='NOMINAL_CHANGED_INSTALLATION_NOT_REAL_VISION_CLOSED_LOOP')


def predict(image,q_capture,obs,solver,validated,point_id,overlay=None):
    validate_observations(obs);validate_result(solver,obs)
    if (validated['status']!='SPECIFIED_SIMULATION_VALIDATION_PASSED' or
        validated['observation_binding']!=observation_binding(obs) or
        any(v!='PASS' for v in validated['validation_status'].values()) or
        not np.array_equal(validated['T_parent_child'],solver['T_parent_child'])):
        raise ValueError('FROZEN_VALIDATED_EXTRINSICS_REQUIRED')
    C,detection=camera_from_board(image,obs['board'],np.asarray(obs['camera']['K']),obs['thresholds'],overlay)
    X=rigid(validated['T_parent_child']);L=fk(obs['kinematics'],q_capture)['link_1'];P=L@X@C
    p=next(p['p_P_m'] for p in obs['board']['touch_points'] if p['point_id']==point_id)
    return dict(T_D_P_est=P.tolist(),p_D_target_est=points(P,p).tolist(),normal_into_board_D=P[:3,2].tolist(),
        T_C_P=C.tolist(),q_capture_rad=np.asarray(q_capture).tolist(),T_D_link1_capture=L.tolist(),point_id=point_id,
        detection=detection,target_frame='D',target_update_after_capture=False,
        source='Fresh RGB PnP + frozen estimated X + FK(q_capture); no USD/true extrinsics/old touch solution input')
