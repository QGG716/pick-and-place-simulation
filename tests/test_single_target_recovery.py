"""Control-flow regressions; real robot runs are recorded separately."""
from types import SimpleNamespace as NS
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.single_target_recovery import connect_fixed_target
from unloading_sim.moveit2_backend import MoveItUnavailable


def fixture(monkeypatch):
    import unloading_sim.single_target_recovery as module
    clock=[0.];monkeypatch.setattr(module,'perf_counter',lambda:clock[0])
    context=dict(parent='root',world='world',attachment='attached',target='fixed')
    c=NS(_native_limits=lambda:(1.,2.),_native_cancelled=lambda:False,
        _build_native_request=lambda *a,**k:deepcopy(context),
        _native_parent_for_request=lambda *a:(NS(evidence=lambda:dict(parent=context['parent'])),{},None),
        ik={'candidate_dedup_tolerance_rad':.001},connection_attempts=[])
    iks=[];motions=[]
    def ik(pose,start,obs,seed,**kw):
        clock[0]+=.01;iks.append(dict(deepcopy(kw),pose=pose.copy(),start=start.copy()))
        return np.full(6,len(iks)/100),None
    def plan(start,q,obs,**kw):
        clock[0]+=.02;motions.append(dict(kw,start=start.copy(),q=q.copy()))
        return [start,q],None,{}
    c.ik_pose_once=ik;c._native_plan=plan
    return c,clock,context,iks,motions

def run(c):
    return connect_fixed_target(c,np.eye(4),np.zeros(6),[],71070,stage='transit',purpose='FREE_LOADED_TRANSFER',attachment=object(),residual_reason='FK')

def test_early_rejections_keep_other_configurations_without_small_attempt_cap(monkeypatch):
    c,clock,context,iks,motions=fixture(monkeypatch);original=c._native_plan
    def plan(*a,**kw):
        path,f,e=original(*a,**kw)
        return ([],dict(reason='INVALID_GOAL_CLEARANCE'),{}) if len(motions)<8 else (path,f,e)
    c._native_plan=plan
    path,q,f,cost=run(c)
    assert path and f is None and len(motions)==len(iks)==8
    assert [len(x['excluded_ik']) for x in iks]==list(range(8))  # immutable snapshots below
    assert all(np.array_equal(x['start'],np.zeros(6)) and np.array_equal(x['pose'],np.eye(4)) for x in iks)
    assert all(x['tree_retained'] is False for x in c.connection_attempts)
    assert cost['motion_s']==pytest.approx(.16) and cost['ik_s']==pytest.approx(.08)

def test_bounded_ik_incomplete_can_recover(monkeypatch):
    c,clock,_,iks,motions=fixture(monkeypatch);original=c.ik_pose_once
    def ik(*a,**kw):
        q,f=original(*a,**kw)
        if len(iks)==1:return None,dict(reason='NATIVE_IK_FAILED',native=dict(bounded_search=True))
        return q,f
    c.ik_pose_once=ik
    path,q,f,cost=run(c);assert path and len(iks)==2 and len(motions)==1
    assert c.connection_attempts[0]['status']=='IK_NOT_COMPLETED'

def test_stage_budget_not_reset_and_incomplete_not_unreachable(monkeypatch):
    c,clock,_,iks,motions=fixture(monkeypatch)
    def ik(*a,**kw):clock[0]+=.4;return None,dict(reason='NATIVE_IK_FAILED',native=dict(bounded_search=True))
    c.ik_pose_once=ik
    path,q,f,cost=run(c)
    assert not path and f['reason']=='SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED' and f['attempts']==3
    assert [r['remaining_stage_s'] for r in c.connection_attempts]==pytest.approx([1.,.6,.2])
    assert c._single_connection_deadline is None

@pytest.mark.parametrize('mutation',['cancel','world','parent','attachment','target'])
def test_context_or_cancel_after_ik_stops_before_motion(monkeypatch,mutation):
    c,clock,context,iks,motions=fixture(monkeypatch);original=c.ik_pose_once
    def ik(*a,**kw):
        out=original(*a,**kw)
        if mutation=='cancel':c._native_cancelled=lambda:True
        else:context[mutation]='changed'
        return out
    c.ik_pose_once=ik
    with pytest.raises(MoveItUnavailable):run(c)
    assert not motions

def test_same_excluded_q_is_not_replanned_forever(monkeypatch):
    c,clock,_,iks,motions=fixture(monkeypatch)
    def ik(*a,**kw):clock[0]+=.01;return np.ones(6),None
    c.ik_pose_once=ik
    def plan(*a,**kw):motions.append(1);return [],dict(reason='INVALID_GOAL_CLEARANCE'),{}
    c._native_plan=plan
    path,q,f,cost=run(c);assert f['reason']=='SINGLE_TARGET_NO_NUMERICAL_PROGRESS' and len(motions)==1

@pytest.mark.parametrize('reason',['INVALID_START_CLEARANCE','CANCELLED','NATIVE_TASK_CONTEXT_MISMATCH'])
def test_request_failures_are_not_candidate_retries(monkeypatch,reason):
    c,_,_,iks,motions=fixture(monkeypatch)
    c._native_plan=lambda *a,**k:([],dict(reason=reason),{})
    with pytest.raises(MoveItUnavailable):run(c)
    assert len(iks)==1

def test_budget_success_stops_without_optional_work(monkeypatch):
    c,_,_,iks,motions=fixture(monkeypatch);path,q,f,cost=run(c)
    assert path and f is None and len(iks)==len(motions)==1

def test_exclusions_do_not_cross_connection_calls(monkeypatch):
    c,_,_,iks,motions=fixture(monkeypatch);run(c);run(c)
    assert all(len(i['excluded_ik'])==0 for i in iks)


def test_explicit_same_template_recovery_retains_attempts_but_rejects_parent_change(monkeypatch):
    c,_,context,iks,motions=fixture(monkeypatch)
    state={}
    def call():
        return connect_fixed_target(c,np.eye(4),np.zeros(6),[],71070,stage='pregrasp',
            purpose='FREE_APPROACH',residual_reason='FK',recovery_state=state)
    call();call()
    assert [len(i['excluded_ik']) for i in iks]==[0,1]
    context['parent']='different-parent'
    with pytest.raises(MoveItUnavailable,match='RECOVERY_CONTEXT_CHANGED'):call()
    assert len(motions)==2
