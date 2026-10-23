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


def test_first_approach_is_clockwise_with_small_total_numerics():
    q=np.zeros((2,6))
    q[-1,0]=J1_REVERSE_LIMIT*.5
    assert check_business_direction(q,'OUTBOUND')==pytest.approx(J1_REVERSE_LIMIT*.5)
    q[-1,0]=2.
    with pytest.raises(ValueError,match='TOTAL_REVERSE'):
        check_business_direction(q,'FIRST_APPROACH')
    q[-1,0]=-2.
    assert check_business_direction(q,'FIRST_APPROACH')==0.


def test_capture_first_approach_policy_does_not_relax_loaded_or_return_direction():
    from unloading_sim.eco65.coupled_state import J1_CAPTURE_POLICY
    q=np.zeros((2,6));q[:,0]=[-1.57,-.4]
    assert check_business_direction(q,'FIRST_APPROACH',policy=J1_CAPTURE_POLICY)==0.
    assert check_business_direction(q,'RETURN',policy=J1_CAPTURE_POLICY)==0.
    with pytest.raises(ValueError,match='TOTAL_REVERSE'):
        check_business_direction(q,'OUTBOUND',policy=J1_CAPTURE_POLICY)


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


def test_new_initial_candidates_use_separate_native_tasks_without_resetting_budget():
    from types import SimpleNamespace
    from unloading_sim.eco65.coupled_task import CoupledTask
    starts=[np.array([1.,0,0,0,0,0]),np.array([2.,0,0,0,0,0])]
    attempted=[]
    task=SimpleNamespace(progress_path=None,task_id='run-root',max_grasps=1,
                         deadline=100.,cycle_failures=[],initial_mode='new_simulation',
                         completed_ids=[],first_target_id='target')
    task.geometry_candidates=lambda:dict(candidates=[dict(candidate_id='g',tcp_pose_D=np.eye(4).tolist())])
    task.select_grasp=lambda g:None
    task.select_home=lambda pose:(np.zeros(6),{})
    task.checkpoint=lambda:None
    task.restore=lambda saved:None
    task.initial_candidates=lambda g,h:iter((q,dict(mode='new_simulation')) for q in starts)
    task.state=lambda:dict(q=task.current_q.tolist())
    task.approach=lambda g:attempted.append((task.task_id,task.current_q.copy(),task.deadline))
    task.attach=lambda:None
    def extract():
        if len(attempted)==1:raise ValueError('NATIVE_EXTRACTION_FAILED')
    task.extract_and_release=extract
    task.save_checkpoint=lambda label:None
    task.return_next=lambda:dict(complete=True)
    assert CoupledTask.run(task)['complete']
    assert len({row[0] for row in attempted})==2
    assert all(row[2]==100. for row in attempted)
    assert all(np.array_equal(row[1],q) for row,q in zip(attempted,starts))


def test_diagonal_and_yaw_recovery_are_absent_from_new_process():
    from unloading_sim.eco65.coupled_task import CoupledTask
    assert not hasattr(CoupledTask,'clear_stack_pose')
    assert not hasattr(CoupledTask,'release_candidates')


def test_return_search_reaches_other_carton_and_retains_matching_tool_family(monkeypatch):
    from types import SimpleNamespace
    from unloading_sim.eco65 import coupled_task
    monkeypatch.setattr(coupled_task,'perf_counter',lambda:0.)
    task=coupled_task.CoupledTask.__new__(coupled_task.CoupledTask)
    boxes=[dict(id='left',pose=[.2,.11,.53]),dict(id='right',pose=[.2,-.11,.53])]
    task.scene=dict(boxes=boxes);task.completed_ids=[];task.current_q=np.zeros(6)
    task.robot=SimpleNamespace(fk=lambda q:np.eye(4));task.height_m=.33
    task.spec=dict(low_height_m=.18);task.max_grasps=8;task.deadline=123.
    task.carton_poses={b['id']:np.eye(4) for b in boxes};task.cycle_failures=[];task.snapshot={}
    task.checkpoint=lambda:dict(face='top',active_ids=['known_cup'])
    task.restore=lambda cp:None
    # The compatible family lies beyond the old first-eight truncation.
    front=[dict(candidate_id='front'+str(i),target_face='front',active_ids=['other'],
                process_option={'option_id':'high'},tcp_pose_D=np.eye(4).tolist()) for i in range(10)]
    compatible=dict(candidate_id='compatible',target_face='top',active_ids=['known_cup'],
                    process_option={'option_id':'low'},tcp_pose_D=np.eye(4).tolist())
    task.geometry_candidates=lambda:dict(candidates=front+[compatible])
    attempts=[]
    task.select_next=lambda target,g:attempts.append((target['id'],g['candidate_id'],task.deadline))
    task.lift=lambda h:None
    def approach(g):
        if attempts[-1][0]!='left' or g['candidate_id']!='compatible':raise ValueError('bounded rejection')
    task.approach=approach;task.attach=lambda **kw:None
    task.result=lambda **kw:dict(complete=True,fingerprint='old')
    monkeypatch.setattr(coupled_task,'validate_cycle',lambda *args:dict(status='PASS'))
    assert task.return_next()['complete']
    assert {row[0] for row in attempts[:6]}=={'left','right'}
    assert attempts[-1][1]=='compatible'
    assert all(row[2]==123. for row in attempts)
