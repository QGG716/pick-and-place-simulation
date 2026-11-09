from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.front_arm_posture import front_arm_posture


def robot(upper, forearm, forward=1.):
    def frame(xyz):
        f=np.eye(4);f[:3,3]=xyz;return f
    u=np.deg2rad(upper);v=np.deg2rad(forearm)
    shoulder=np.array([0.,0.,1.565]);elbow=shoulder+.895*np.array([np.cos(u),0,np.sin(u)])
    root=elbow+np.array([0.,0.,.17]);wrist=root+1.045*np.array([forward*np.cos(v),0,np.sin(v)])
    return SimpleNamespace(named_link_frames=lambda q:dict(J2_link=frame(shoulder),J3_link=frame(elbow),J4_link=frame(root),J5_link=frame(wrist)))


def test_vertical_upper_horizontal_forearm_accepts_despite_large_q2():
    r=front_arm_posture(robot(90,0),[0,2.,0,0,0,0],initial=True)
    assert r['passed'] and r['upper_arm']['elevation_deg']==pytest.approx(90)
    assert r['forearm_elevation_deg']==pytest.approx(0)
    assert r['forearm_length_m']==pytest.approx(1.045)


def test_previous_folded_configuration_is_rejected():
    r=front_arm_posture(robot(-24.681,80.439),np.zeros(6))
    assert not r['passed']


def test_initial_stricter_than_contact_and_forearm_must_point_forward():
    assert front_arm_posture(robot(65.868,2.093),np.zeros(6))['passed']
    assert not front_arm_posture(robot(65.868,2.093),np.zeros(6),initial=True)['passed']
    assert not front_arm_posture(robot(90,0,-1.),np.zeros(6))['passed']


@pytest.mark.parametrize('upper,forearm',[(44.9,0),(90,30.1),(90,-30.1)])
def test_posture_bounds(upper,forearm):
    assert not front_arm_posture(robot(upper,forearm),np.zeros(6))['passed']
