from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.initial_front_working import front_working_initial_evidence


def robot(upper, forearm):
    def frames(q):
        result={name:np.eye(4) for name in ('J1_link','J2_link','J3_link','J4_link','J5_link')}
        result['J1_link'][0,0]=np.cos(q[0])
        result['J3_link'][:3,3]=upper
        result['J5_link'][:3,3]=forearm
        return result
    return SimpleNamespace(named_link_frames=frames)


def test_near_zero_j1_does_not_accept_backward_upper_arm():
    q=np.zeros(6);q[0]=np.deg2rad(-12.853)
    evidence=front_working_initial_evidence(robot([-.59,.13,.80],[.97,-.22,-.10]),q)
    assert not evidence['passed']


@pytest.mark.parametrize('j1',[-35.,-8.,0.,22.,65.])
def test_forward_initial_allows_variable_j1_and_sloping_links(j1):
    q=np.zeros(6);q[0]=np.deg2rad(j1)
    evidence=front_working_initial_evidence(robot([.3,0,.8],[.6,0,-.7]),q)
    assert evidence['passed']
    assert not evidence['j1_fixed'] and not evidence['affects_grasp_endpoints']


def test_j1_is_soft_preference_after_directional_acceptance():
    r=robot([.3,0,.8],[.6,0,-.7]);a=np.zeros(6);b=a.copy();b[0]=.4
    assert front_working_initial_evidence(r,a)['selection_cost'] < front_working_initial_evidence(r,b)['selection_cost']


def test_backwards_base_or_forearm_rejected():
    q=np.zeros(6);q[0]=np.pi
    assert not front_working_initial_evidence(robot([.3,0,.8],[.6,0,-.7]),q)['passed']
    assert not front_working_initial_evidence(robot([.3,0,.8],[-.6,0,-.7]),np.zeros(6))['passed']
