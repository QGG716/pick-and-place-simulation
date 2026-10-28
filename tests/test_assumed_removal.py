from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.assumed_removal import TARGET,assume_l05_removed,validate_assumption
from unloading_sim.carton_population import FrozenCartonPopulation,rank_population
from unloading_sim.coupled_batch import BatchState
from unloading_sim.current_population_recipe import LAYOUT
from unloading_sim.moveit2_backend import box_message
from unloading_sim.support import SupportRelationGraph
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state


def fixture():
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells'/LAYOUT)
    original=layout.cartons();frozen=FrozenCartonPopulation.capture(original)
    ids={'carton_l00_c00','carton_l00_c01','carton_l00_c02','carton_l00_c03','carton_l00_c04',
         'carton_l01_c02','carton_l01_c03','carton_l01_c04','carton_l02_c02','carton_l02_c03',
         'carton_l02_c04','carton_l03_c02','carton_l03_c03','carton_l04_c02','carton_l04_c03',TARGET}
    state=BatchState(frozen,tuple(b for b in original if b.name in ids),np.arange(6,dtype=float),.25,
        ledger=[dict(target_id=b.name,source='IDEAL_RECEIVER_DRAIN') for b in original if b.name not in ids])
    return layout,original,state


def test_assumption_preserves_motion_and_success_ledger_and_opens_support():
    layout,original,state=fixture();before=deepcopy(state.root());ledger=deepcopy(state.ledger)
    assert TARGET in SupportRelationGraph.build(state.cartons).supports['carton_l04_c03']
    event=assume_l05_removed(state)
    validate_assumption(event,{b.name:box_message(b) for b in original},state.root(),ledger)
    assert len(state.cartons)==15 and len(state.ledger)==24 and state.ledger==ledger and not state.cycles
    assert state.root()['q_rad']==before['q_rad'] and state.height==.25
    assert TARGET not in state.root()['completed'] and state.root()['assumed_removed']==[TARGET]
    assert not SupportRelationGraph.build(state.cartons).supports['carton_l04_c03']
    world={b.name:b for b in coupled_conveyor_state(layout,.25,state.cartons).boxes}
    robot=SimpleNamespace(within_limits=lambda q:True,fk=lambda q:np.eye(4))
    ranked=rank_population(state.frozen,state.cartons,state.accounted_ids(),set(),state.q,robot,
        world['conveyor_transverse'],longitudinal=world['conveyor_longitudinal'])
    assert 'carton_l04_c03' in {r['target_id'] for r in ranked} and TARGET not in {r['target_id'] for r in ranked}
    for change in ('q','completion','wrong_target'):
        bad=deepcopy(event)
        if change=='q':bad['after']['q_rad'][0]+=.1
        if change=='completion':bad['counted_as_planning_success']=True
        if change=='wrong_target':bad['target_id']='carton_l04_c03'
        with pytest.raises(AssertionError):validate_assumption(bad,{b.name:box_message(b) for b in original},state.root(),ledger)
    with pytest.raises(ValueError):assume_l05_removed(state)


def test_assumed_carton_cannot_commit_a_fabricated_cycle():
    _,_,state=fixture();assume_l05_removed(state)
    plan=dict(target_id=TARGET,batch_root=state.root(),events=[dict(event_type=n,target_id=TARGET)
        for n in ('RELEASE','NOMINAL_FIRST_CONTACT','SAFE_TOOL_SEPARATION','IDEAL_RECEIVER_DRAIN')])
    with pytest.raises(ValueError,match='PREMATURE_OR_DUPLICATE_DRAIN'):state.commit(plan)
    assert len(state.ledger)==24


def test_ordinary_roots_retain_their_original_schema():
    _,_,state=fixture()
    assert 'assumed_removed' not in state.root()
    assert state.accounted_ids()==state.root()['completed']
