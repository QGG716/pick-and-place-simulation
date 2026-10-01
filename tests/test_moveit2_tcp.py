"""Directed task TCP geometry checks, independent of ROS and collision mocks."""
import numpy as np
import pytest
from unloading_sim.moveit2_tcp import audit_linear_tcp
from unloading_sim.geometry import rotation_matrix_from_rotation_vector as exp
from unloading_sim.moveit2_backend import linear_capability


def pose(x=0., angle=0.):
    value=np.eye(4);value[0,3]=x;value[:3,:3]=exp(np.array([0.,0.,angle]));return value


def audit(path,fk,a,b):
    return audit_linear_tcp(path,fk,a,b,position_tolerance=1e-4,
        orientation_tolerance=2e-4,edge_resolution_rad=.04)


@pytest.mark.parametrize('distance,angle',[(.1,0.),(0.,.4),(.1,.4)])
def test_shared_shortest_progress_and_pure_rotation(distance,angle):
    fk=lambda q:pose(distance*q[0],angle*q[0])
    result=audit([[0.],[1.]],fk,fk([0.]),fk([1.]))
    assert result['passed'] and result['samples']>3


def test_flange_line_correct_endpoints_wrong_offset_tcp_arc_is_rejected():
    offset=np.eye(4);offset[0,3]=.25
    first=pose(0.,0.);last=pose(.1,.5)
    flange_a=first@np.linalg.inv(offset);flange_b=last@np.linalg.inv(offset)
    def wrong(q):
        t=q[0];flange=pose(0.,.5*t)
        flange[:3,3]=(1-t)*flange_a[:3,3]+t*flange_b[:3,3]
        return flange@offset
    result=audit([[0.],[1.]],wrong,first,last)
    assert not result['passed'] and result['maximum_line_error_m']>1e-3
    assert max(x['position_m'] for x in result['endpoint_errors'])<1e-12


def test_dense_edges_detect_violation_missed_by_knots_and_midpoint():
    def fk(q):
        p=pose(.1*q[0]);p[1,3]=.002*np.sin(2*np.pi*q[0]);return p
    assert all(abs(fk([t])[1,3])<1e-12 for t in (0.,.5,1.))
    assert not audit([[0.],[1.]],fk,fk([0.]),fk([1.]))['passed']


def test_orientation_progress_must_match_position_and_be_monotonic():
    fk=lambda q:pose(.1*q[0],.4*q[0]**2)
    assert not audit([[0.],[1.]],fk,fk([0.]),fk([1.]))['passed']
    fk=lambda q:pose(.1*q[0],.4*q[0])
    assert not audit([[0.],[.6],[.4],[1.]],fk,fk([0.]),fk([1.]))['passed']


def test_context_mismatch_and_invalid_transform_are_capability_failures():
    a=pose();b=pose(.1,.3);offset=pose(.25,.6)
    assert linear_capability(a,b,offset,stage='transit',location='before_checks',bound_tcp=offset)['supported']
    result=linear_capability(a,b,offset,stage='transit',location='before_checks',bound_tcp=a)
    assert result['reason']=='UNSUPPORTED_TASK_TCP_CONTEXT_MISMATCH'
    invalid=offset.copy();invalid[3,3]=2
    assert not linear_capability(a,b,invalid,stage='transit',location='before_checks')['supported']
