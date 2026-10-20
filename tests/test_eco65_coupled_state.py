"""Directed finite-scale checks for the new continuous cycle, no physical claims."""
import numpy as np
import pytest
from unloading_sim.eco65.coupled_state import (
    check_business_direction, receiving_geometry, frozen_receive_heights,
    sign_state, check_state, J1_REVERSE_LIMIT)


def test_many_small_reverse_edges_cannot_accumulate_across_motion_segments():
    q=np.zeros((20,6))
    q[:,0]=np.arange(20)*.7e-7
    with pytest.raises(ValueError,match='TOTAL_REVERSE'):
        check_business_direction(q,'OUTBOUND')
    used=check_business_direction(q[:10],'OUTBOUND')
    with pytest.raises(ValueError,match='TOTAL_REVERSE'):
        check_business_direction(q[9:],'OUTBOUND',used)


def test_next_contact_is_return_interval_not_contact_stage_outbound():
    q=np.zeros((3,6))
    q[:,0]=[-1.,-.8,-.5]
    assert check_business_direction(q,'RETURN')==0.
    with pytest.raises(ValueError):
        check_business_direction(q,'OUTBOUND')
    with pytest.raises(ValueError,match='REQUIRED'):
        check_business_direction(q,None)


def test_zero_direction_only_for_initial_approach_and_small_total_numerics():
    q=np.zeros((2,6))
    q[-1,0]=J1_REVERSE_LIMIT*.5
    assert check_business_direction(q,'OUTBOUND')==pytest.approx(J1_REVERSE_LIMIT*.5)
    q[-1,0]=2.
    assert check_business_direction(q,'FIRST_APPROACH')==0.


def test_receive_candidates_are_frozen_from_initial_150x100x100_carton():
    box=dict(pose=[.205,0,.53],size_m=[.15,.1,.1])
    rows=frozen_receive_heights(box,.18,.46)
    assert [r['height_m'] for r in rows]==pytest.approx([.43,.33])
    assert rows[0]['frozen_initial_bottom_m']==pytest.approx(.48)
    assert frozen_receive_heights(box,.18,.40)[0]['height_m']==pytest.approx(.33)


def test_actual_release_drop_changes_if_extraction_raised_box():
    T=np.eye(4);T[:3,3]=[-.08,0,.53]
    bounds=[[-.18,-.2,.42],[.02,.24,.43]]
    assert receiving_geometry(T,[.15,.1,.1],bounds,desired_drop=.05)['passed']
    T[2,3]+=.01
    r=receiving_geometry(T,[.15,.1,.1],bounds,desired_drop=.05)
    assert not r['passed'] and r['actual_gap_m']==pytest.approx(.06)


def test_entire_box_projection_not_just_center_must_be_in_receiver():
    T=np.eye(4);T[:3,3]=[-.03,0,.53]
    r=receiving_geometry(T,[.15,.1,.1],[[-.18,-.2,.42],[.02,.24,.43]],desired_drop=.05)
    assert not r['full_projection_inside']

def test_upright_yaw_release_uses_rotated_full_projection_and_actual_gap():
    T=np.eye(4);T[:3,3]=[-.04,0.,.53]
    T[:3,:3]=[[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]]
    bounds=[[-.18,-.3,.42],[.02,.1443197215708027,.43]]
    assert receiving_geometry(T,[.15,.1,.1],bounds,desired_drop=.05)['passed']
    T[1,3]=.08
    assert not receiving_geometry(T,[.15,.1,.1],bounds,desired_drop=.05)['full_projection_inside']
    T[:3,:3]=[[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]]
    assert not receiving_geometry(T,[.15,.1,.1],bounds)['level']


def test_world_attached_completed_ownership_is_disjoint():
    state=dict(q=[0.]*6,business_interval='RETURN',world=[dict(id='first'),dict(id='second')],
               attachment=None,completed_ids=[],remaining_ids=['first','second'])
    check_state(sign_state(state),['first','second'])
    state['attachment']=dict(id='second')
    with pytest.raises(ValueError,match='OWNERSHIP'):
        check_state(sign_state(state),['first','second'])
    state['world']=[dict(id='first')]
    check_state(sign_state(state),['first','second'])
    state['completed_ids']=['second']
    with pytest.raises(ValueError,match='OWNERSHIP'):
        check_state(sign_state(state),['first','second'])


@pytest.mark.parametrize('clock_after_request,connection_only',[(81.,True),(101.,False)])
def test_request_construction_crossing_connection_deadline_keeps_global_scope(
        monkeypatch,clock_after_request,connection_only):
    from unloading_sim.eco65 import coupled_task
    from unloading_sim.eco65.native_task import NativeTask
    from unloading_sim.moveit2_backend import MoveItUnavailable
    task=coupled_task.CoupledTask.__new__(coupled_task.CoupledTask)
    task.deadline=100.;task.connection_deadline=80.;task.rpc_seconds=100.
    task.business_interval='FIRST_APPROACH';task.reverse_used={'FIRST_APPROACH':0.}
    times=iter([0.,clock_after_request,clock_after_request])
    monkeypatch.setattr(coupled_task,'perf_counter',lambda:next(times))
    def exhausted(self,request):
        raise MoveItUnavailable('NATIVE_TASK_BUDGET_EXHAUSTED')
    monkeypatch.setattr(NativeTask,'call',exhausted)
    failure=ValueError if connection_only else MoveItUnavailable
    with pytest.raises(failure,match='BUDGET_EXHAUSTED'):
        task.call({'op':'fk'})
    assert task.deadline==100.
