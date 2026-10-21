"""Directed batch completion and lower-layer geometry checks; no mock success claims."""
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.eco65.coupled_state import frozen_receive_heights
from unloading_sim.eco65.coupled_task import CoupledTask
from unloading_sim.eco65.batch_task import BatchTask


def test_bottom_layer_uses_hardware_height_and_requires_real_payload_raise():
    box=dict(pose=[.205,0.,.2307527833],size_m=[.15,.1,.1])
    assert frozen_receive_heights(box,.18,.46)==[]
    rows=frozen_receive_heights(box,.18,.46,hardware_fallback=True)
    assert rows[0]['height_m']==.18
    assert rows[0]['frozen_initial_bottom_m']==pytest.approx(.1807527833)
    assert rows[0]['required_box_bottom_at_release_m']==pytest.approx(.23)
    assert box['pose'][2]==.2307527833


def test_last_carton_extraction_has_no_remaining_stack_minimum_error():
    actual=np.eye(4);actual[:3,3]=[.205,0.,.23]
    box=dict(id='last',pose=[.205,0.,.23],size_m=[.15,.1,.1])
    task=SimpleNamespace(robot=SimpleNamespace(named_link_frames=lambda q:{'link_6':actual}),
        current_q=np.zeros(6),attachment=np.eye(4),target=box,scene=dict(boxes=[box]),
        completed_ids=[],identity=dict(flange_from_task_tcp=np.eye(4)))
    result=CoupledTask.clear_stack_pose(task)
    assert result[0,3]==pytest.approx(.03486121811340027)
    assert result[2,3]==pytest.approx(.236)


def test_new_carton_deadline_is_capped_by_absolute_batch_deadline(monkeypatch):
    from unloading_sim.eco65 import batch_task
    task=SimpleNamespace(batch_deadline=1000.,carton_seconds=900.,completed_ids=['first'],
        started=0.,carton_budgets=[],reception_deadline=500.)
    monkeypatch.setattr(batch_task,'perf_counter',lambda:500.)
    BatchTask.begin_carton_budget(task)
    assert task.deadline==1000.
    assert task.reception_deadline is None
    assert task.carton_budgets[0]['carton_number']==2


def test_later_failure_does_not_restart_committed_first_carton(monkeypatch):
    restored=[]
    task=SimpleNamespace(completed_ids=['first'],original_boxes={'first':{},'second':{}},begin_carton_budget=lambda:None)
    task.checkpoint=lambda:'committed-drained-state'
    task.restore=restored.append
    def fail(*a,**kw):raise ValueError('NATIVE_LOADED_CYCLE_EXHAUSTED')
    monkeypatch.setattr(CoupledTask,'return_next',fail)
    # Use the real subclass instance so super() dispatch retains its contract.
    actual=BatchTask.__new__(BatchTask);actual.__dict__.update(task.__dict__)
    with pytest.raises(RuntimeError,match='BATCH_CONTINUATION_FAILED'):
        actual.return_next()
    assert actual.completed_ids==['first']
    assert restored==['committed-drained-state']


def test_pregrasp_success_near_cutoff_keeps_actual_q_for_contact(monkeypatch):
    from contextlib import contextmanager
    from unloading_sim.eco65 import batch_task
    clock=[0.]
    monkeypatch.setattr(batch_task,'perf_counter',lambda:clock[0])
    task=BatchTask.__new__(BatchTask)
    task.deadline=100.;task.connection_deadline=None;task.current_q=np.zeros(6)
    task.target_pose=np.eye(4);task.face='top';task.target={'id':'next'};task.cycle_failures=[]
    task.select_grasp=lambda g:None
    @contextmanager
    def connection():
        task.connection_deadline=80.
        try:yield
        finally:task.connection_deadline=None
    task.connection=connection;task.ik=lambda *a:iter([np.zeros(6)])
    task.direction_goal_valid=lambda *a:True;task.goal_cost=lambda *a:0
    task.checkpoint=lambda:None;task.seal=lambda:{'status':'PASS'}
    motions=[]
    def move(stage,**kw):
        motions.append((stage,task.current_q.copy()))
        if stage=='pregrasp':
            clock[0]=79.;task.current_q=np.ones(6)
        else:
            assert task.connection_deadline is None and task.deadline==100.
            clock[0]=82.;task.current_q=np.ones(6)*2
    task.move=move
    task.approach(dict(candidate_id='current-native',tcp_pose_D=np.eye(4).tolist()))
    assert motions[1][0]=='contact' and np.array_equal(motions[1][1],np.ones(6))
    assert task.deadline==100. and task.connection_deadline is None


def test_top_release_keeps_existing_orientation_first(monkeypatch):
    task=BatchTask.__new__(BatchTask);task.face='top';seen=[]
    def candidates(self,height,yaws):
        seen.extend(yaws);return iter(())
    monkeypatch.setattr(CoupledTask,'release_candidates',candidates)
    list(task.release_candidates(.33,(-np.pi/2,np.pi/2,np.pi,0.)))
    assert seen==[0.,-np.pi/2,np.pi/2,np.pi]
