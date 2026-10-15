"""Fixed J1 accessory transforms; no IK, target tracking or timeline updates."""
import numpy as np
from unloading_sim.eco65.model import check_transform


def horizontal(v):
    v=np.asarray(v,float).copy();v[2]=0
    if not np.isfinite(v).all() or np.linalg.norm(v)<1e-8:
        raise ValueError('DEGENERATE_HORIZONTAL_HEADING')
    return v/np.linalg.norm(v)


def evaluate(link1,config):
    if config['attachment']!='link_1':raise ValueError('MAST_PARENT_MUST_BE_LINK1')
    link1=check_transform(link1)
    local=check_transform(config['T_link1_mast']);camera=check_transform(config['T_mast_camera'])
    mast=link1@local;optical=mast@camera
    arm=horizontal(link1[:3,:3]@np.array(config['arm_forward_link1']))
    cam=horizontal(optical[:3,2]) # Camera local +Z is the explicitly defined optical axis.
    angle=float(np.degrees(np.arctan2(np.cross(arm,cam)[2],arm@cam)))
    target=config['relative_heading_deg']
    if abs(target)!=90 or abs(angle-target)>1e-8:raise ValueError('CAMERA_AXIS_NOT_FIXED_90_DEGREES')
    return dict(T_D_link1=link1.tolist(),T_D_mast=mast.tolist(),T_D_camera=optical.tolist(),
        arm_heading_D=arm.tolist(),camera_heading_D=cam.tolist(),signed_angle_deg=angle,
        absolute_angle_deg=abs(angle),error_deg=abs(angle-target),
        status='CAMERA_AXIS_90DEG_CONSTRAINT_APPLIED',optical_calibration='NOT_EVALUATED')
