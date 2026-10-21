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
    task=SimpleNamespace(completed_ids=['first'],original_boxes={'first':{},'second':{}},begin_carton_budget=lambda:None)
    def fail(*a,**kw):raise ValueError('NATIVE_LOADED_CYCLE_EXHAUSTED')
    monkeypatch.setattr(CoupledTask,'return_next',fail)
    # Use the real subclass instance so super() dispatch retains its contract.
    actual=BatchTask.__new__(BatchTask);actual.__dict__.update(task.__dict__)
    with pytest.raises(RuntimeError,match='BATCH_CONTINUATION_FAILED'):
        actual.return_next()
    assert actual.completed_ids==['first']
