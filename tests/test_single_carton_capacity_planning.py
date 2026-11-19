"""Capacity gates on the real search/verification paths, with fake IO."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest
from test_single_carton_planner import case
from test_single_carton_holding_capacity import fixture
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.single_carton.planner import SinglePlanner,SegmentRejected


@pytest.mark.parametrize('native',[False,True])
def test_insufficient_capacity_is_searched_failure_before_native_screen(case,monkeypatch,native):
    p,obs,box,folder,calls,_=case
    candidate=contact_candidates(box,p.source['tool'],p.site['topology'])[0]
    candidate['active_ids']=candidate['active_ids'][:1]
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates',lambda *a:[candidate])
    monkeypatch.setattr(p,'native_planning',lambda:native)
    monkeypatch.setattr(p,'native_screen',lambda *a:pytest.fail('insufficient holding reached native screening'))
    failures=[]
    assert p.search_box(obs.q_start,[],box,folder,failures=failures) is None
    assert not calls and failures[0]['reason']=='HOLDING_CAPACITY_INSUFFICIENT'
    summary=p.budget_summary([box],failures,None,None)
    assert summary['targets'][0]['searched']==1 and summary['targets'][0]['unsearched']==0
    assert not p.authority._plans


def test_actual_fk_postcheck_rejects_candidate_without_permit(case):
    p,_,_,folder,_,_=case;box,candidate,tool,config=fixture(depth=.217)
    candidate['tcp_pose'][2][3]+=.06;base=np.asarray(candidate['tcp_pose'])
    def tcp(q):
        T=base.copy();T[1,3]+=.1*q[0]**2;return T
    p.coordinate_model=SimpleNamespace(tcp=tcp);p.source['tool']=tool;p.site=config
    planned=dict(capacity_box=box,capacity_candidate=candidate,actual=base,
        extraction=dict(prepared=dict(points=[dict(t=t,q=[t,0,0,0,0,0]) for t in np.linspace(0,1,201)])),
        returning=dict(prepared=dict(points=[dict(t=t,q=[1+t,0,0,0,0,0]) for t in np.linspace(0,1,201)])))
    with pytest.raises(SegmentRejected,match='HOLDING_CAPACITY_INSUFFICIENT') as error:
        SinglePlanner.holding_postcheck(p,planned,folder)
    with pytest.raises(SegmentRejected):p.replan_after(planned,error.value)
    assert not p.authority._plans
