"""Explicit mechanical pitch and optical-axis checks shared by old/new scenes."""
import numpy as np
from scipy.spatial.transform import Rotation


def pivot_transform(pivot, right_axis, downward_deg):
    axis=np.asarray(right_axis,float);axis/=np.linalg.norm(axis)
    R=Rotation.from_rotvec(-np.radians(downward_deg)*axis).as_matrix()
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(pivot)-R@pivot
    return T


def mount_check(T_W_C,T_W_link1,camera,tolerance=1e-8):
    C=np.asarray(T_W_C);L=np.asarray(T_W_link1)
    expected=float(camera.get('mount',{}).get('downward_pitch_deg',0.))
    expected_heading=float(camera.get('mount',{}).get('heading_from_link1_minus_X_deg',-90.))
    f=C[:3,2];right=C[:3,0];arm=-L[:3,0]
    yaw=np.degrees(np.arctan2(arm[0]*f[1]-arm[1]*f[0],arm[:2]@f[:2]))
    pitch=np.degrees(np.arcsin(np.clip(-f[2],-1,1)))
    errors=dict(optical_z_error=abs(float(f[2]+np.sin(np.radians(expected)))),
                right_axis_vertical_error=abs(float(right[2])),
                heading_error_deg=abs(float(np.degrees(np.arctan2(
                    np.sin(np.radians(yaw-expected_heading)),np.cos(np.radians(yaw-expected_heading)))))))
    return dict(accepted=max(errors.values())<tolerance,downward_pitch_deg=float(pitch),
                expected_downward_pitch_deg=expected,signed_heading_from_link1_minus_X_deg=float(yaw),
                expected_heading_from_link1_minus_X_deg=expected_heading,
                image_down_dot_world_down=float(-C[2,1]),errors=errors)
