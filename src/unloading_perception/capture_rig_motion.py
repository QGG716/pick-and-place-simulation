"""Explicit virtual rig translation, never optical-only motion or robot kinematics."""
from copy import deepcopy
import numpy as np


def translated_virtual_rig(payload, shift_y_m):
    if not np.isfinite(shift_y_m):raise ValueError('INVALID_RIG_TRANSLATION')
    result=deepcopy(payload)
    def move(matrix):
        t=np.asarray(matrix,float).copy()
        if t.shape!=(4,4) or not np.isfinite(t).all():raise ValueError('INVALID_RIG_TRANSFORM')
        t[1,3]+=shift_y_m
        return t.tolist()
    rig=result['mechanisms']['vision_rig']
    for key in ('T_W_vision_flange','T_W_mast_top','T_W_module_0_main'):
        if key in rig:rig[key]=move(rig[key])
    for module in rig['modules']:
        for key in ('T_W_module','T_W_camera_optical'):
            module[key]=move(module[key])
    # T_W_J1 remains the real commanded robot pose; this is an explicit virtual
    # acquisition experiment, not a kinematically attached / executable rig pose.
    rig['pose_mode']='VIRTUAL_RIG_TRANSLATION_DETACHED_FROM_J1'
    rig['nominal_parent_frame']=rig['parent_frame']
    rig['parent_frame']='world_virtual_capture_rig'
    for camera in result['cameras']:
        camera['T_W_C']=move(camera['T_W_C'])
        camera['position_world_m']=[r[3] for r in camera['T_W_C'][:3]]
    return result
