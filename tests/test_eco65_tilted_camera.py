"""Mechanical pitch and target-directed J1 regressions, without private CAD."""
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from eco65_camera_mount import pivot_transform,mount_check
from eco65_j1_visualization import target_endpoints


class Robot:
    def named_link_frames(self,q):
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',q[0]).as_matrix()
        return {'link_1':T}


def test_mechanical_pivot_moves_pupil_and_preserves_internal_transform():
    C=np.eye(4);C[:3,:3]=[[1,0,0],[0,0,1],[0,-1,0]];C[:3,3]=[.14,.02,.43]
    cad=np.eye(4);cad[:3,3]=[.11,-.004,.41]
    A=pivot_transform(np.array([.113,-.004,.46]),[1,0,0],30)
    assert np.linalg.norm((A@C)[:3,3]-C[:3,3])>.01
    np.testing.assert_allclose(np.linalg.inv(A@cad)@(A@C),np.linalg.inv(cad)@C,atol=1e-14)
    for q in ([0]*6,[.3,0,0,0,0,0],[-.3,0,0,0,0,0],[0,.2,-.4,0,0,0]):
        L=Robot().named_link_frames(q)['link_1'];readback=mount_check(L@A@C,L,{'mount':{'downward_pitch_deg':30}})
        assert readback['accepted']
        assert abs(readback['signed_heading_from_link1_minus_X_deg']+90)<1e-12
    assert not mount_check(A@C,np.eye(4),{})['accepted']
    assert not mount_check(C,np.eye(4),{'mount':{'downward_pitch_deg':30}})['accepted']
    assert mount_check(C,np.eye(4),{})['accepted']


def test_receiver_target_can_require_negative_non_right_angle():
    X=np.eye(4);X[:3,:3]=[[0,0,1],[1,0,0],[0,1,0]];X[:3,3]=[0,.15,1]
    stack,receiver,report=target_endpoints(Robot(),np.zeros(6),X,[1,0,0],[.2,-1,0],[-3.1,3.1])
    assert receiver<stack and abs(receiver-stack+np.pi/2)>.1
    q=np.zeros(6);q[0]=receiver;C=Robot().named_link_frames(q)['link_1']@X
    d=np.array([.2,-1])-C[:2,3];f=C[:2,2]
    assert abs(f[0]*d[1]-f[1]*d[0])<1e-12 and f@d>0
    plus=next(x for x in report['plus_minus_90_diagnostics'] if x['offset_deg']==90)
    assert abs(plus['heading_error_deg'])>150


def test_wrist_winding_stays_inside_limits_and_avoids_full_turn():
    from eco65_touch_native import nearest_equivalent
    q=np.array([-1.22,-.8,1.8,-2.07,2.52,-.95])
    current=np.array([1.6,-.3,-1.34,.075,1.57,4.17])
    limits=np.array([[-3.1,3.1]]*5+[[-6.28,6.28]])
    result=nearest_equivalent(q,current,limits)
    np.testing.assert_allclose(result[:5],q[:5])
    assert 5.33<result[5]<5.34 and abs(result[5]-current[5])<1.17
    np.testing.assert_allclose(np.cos(result),np.cos(q),atol=1e-14)
    np.testing.assert_allclose(np.sin(result),np.sin(q),atol=1e-14)
