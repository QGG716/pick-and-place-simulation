"""November 19 start guards: fake IO and clocks only."""
import numpy as np
import pytest
from test_controlled_execution import Harness
from test_single_carton_cycle import make
from test_single_carton_review_fixes import segment, succeeded
from test_single_carton_pre_execute import manager, DISABLED
from unloading_sim.workbench.single_carton.config import start_joint_check, plan_validity
from unloading_sim.workbench.single_carton.runtime import TaskManager
from unloading_sim.workbench.controlled.trajectory import digest


def policy(enabled=True):
    c=dict(start_joint_tolerance_deg=.02,start_joint_tolerance_basis='Stationary encoder jitter measured <= 0.006 degree')
    if not enabled:c.update(start_joint_check='DISABLED_BY_USER',start_joint_check_basis='synthetic explicit instruction')
    return start_joint_check(c)


@pytest.mark.parametrize('enabled,error,passes',[(True,.015,True),(True,.03,False),(False,.4,True),(False,.6,False),(False,90,False)])
def test_cycle_and_supervisor_use_identical_start_guards(enabled,error,passes):
    p=policy(enabled);h=Harness();prepared=segment(0.);prepared['start_joint_check']=p
    c,t,r,v,records,plan=make();plan['start_joint_check']=p;c.ready('test',plan)
    r.q=[np.deg2rad(error)]+[0.]*5
    if passes:
        c.start();assert h.engine.submit(prepared,h.sample(r.q))
    else:
        with pytest.raises(ValueError,match='START_CHANGED_REPLAN'):c.start()
        with pytest.raises(ValueError,match='START_CHANGED_REPLAN_REQUIRED'):h.engine.submit(prepared,h.sample(r.q))
        assert not r.calls and h.engine.index==0 and not v.commands


def test_continuation_keeps_arrival_tolerance():
    h=Harness();succeeded(h);held=h.engine.hold_command[0]
    prepared=segment(held);prepared['start_joint_check']=dict(policy(),hard_cap_deg=.01)
    assert h.engine.submit(prepared,h.sample([held+np.deg2rad(.3)]*6))


def test_expiry_after_final_evidence_barrier_revokes_all_permits(tmp_path,monkeypatch):
    from unloading_sim.workbench.single_carton import runtime
    m,events,started,saved=manager(tmp_path,DISABLED);now=[100.]
    m.observation.frame.stamp.received_monotonic_ns=0
    monkeypatch.setattr(runtime.time,'monotonic',lambda:now[0])
    calls=[]
    def flush(**kwargs):
        calls.append(1)
        if len(calls)==2:now[0]=121.
    m.journal.flush=flush
    with pytest.raises(ValueError,match='PLAN_EXPIRED_REPLAN'):TaskManager.execute(m)
    assert not started and not m.planner.authority._plans and len(calls)==2


def test_start_policy_is_digest_bound():
    p=dict(start_joint_check=policy());before=digest(p)
    p['start_joint_check']['tolerance_deg']=.01
    assert digest(p)!=before


@pytest.mark.parametrize('values',[dict(start_joint_tolerance_deg=.03,start_joint_tolerance_basis='x'),dict(start_hard_cap_deg=.6),dict(start_joint_tolerance_deg=.02)])
def test_field_start_limits_cannot_be_relaxed_or_unexplained(values):
    with pytest.raises(ValueError):start_joint_check(values)


def test_plan_validity_only_tightens():
    assert plan_validity({})['seconds']==120
    with pytest.raises(ValueError):plan_validity(dict(plan_validity_s=121,plan_validity_basis='x'))
