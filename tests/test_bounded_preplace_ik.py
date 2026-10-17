"""Bounded preplace scope and one-request semantics; no planning evidence from mocks."""
from copy import deepcopy
from types import SimpleNamespace as NS
import numpy as np
import pytest
from unloading_sim.single_contact import SingleContactClient, SingleContactConnector
from unloading_sim.moveit2_backend import MoveItUnavailable
from test_single_preplace import transit_request, integration_fixture

def fixture():
    c=SingleContactConnector.__new__(SingleContactConnector)
    sent=[];q=np.arange(6,dtype=float)/10
    c._native_limits=lambda:(90.,100.)
    c._native_cancelled=lambda:False
    c.native_ik_seconds=.25;c.native_ik_evidence=[]
    c.ik=dict(position_tolerance_m=.0001,orientation_tolerance_rad=.0002)
    c.robot=NS(within_limits=lambda q:np.all(np.abs(q)<3),fk=lambda q:np.eye(4))
    c._native_parent_for_request=lambda *a:(NS(stage_id="extraction",evidence=lambda:{"stage_id":"extraction"}),{},None)
    c._build_native_request=lambda start,*a,**k:dict(transit_request("ik"),q_start=start.tolist(),planning_only=True)
    def native(r,**kw):
        sent.append(deepcopy(r))
        return dict(status="SUCCESS",q=q.tolist(),ik_mode=r.get("ik_mode","single_attempt"))
    c.native=NS(request=native)
    return c,q,sent

def invoke(c,q,**kw):
    return c.ik_pose_once(np.eye(4),q,[],71070,stage="transit",attachment=object(),
        purpose="FREE_LOADED_TRANSFER",ik_mode="bounded_preplace",solver_budget_s=.1,**kw)

def test_single_attempt_interface_unchanged():
    c,q,sent=fixture()
    out,f=c.ik_pose_once(np.eye(4),q,[],71070,stage="pregrasp")
    assert f is None and len(sent)==1 and "ik_mode" not in sent[0] and sent[0]["ik_timeout_s"]==.25

@pytest.mark.parametrize("remaining",[90.,.03])
def test_one_call_budget_first_seed_and_parent_unchanged(remaining):
    c,q,sent=fixture();c._native_limits=lambda:(remaining,100.)
    old=q.copy();out,f=invoke(c,q)
    assert f is None and len(sent)==1
    r=sent[0];assert r["solver_budget_s"]==min(.1,remaining) and r["parent_stage_id"]=="extraction"
    np.testing.assert_array_equal(q,old);assert r["q_start"]==old.tolist()
    assert r["collision_check_in_native_ik"] is False

@pytest.mark.parametrize("when",["before","after"])
def test_cancel_or_shared_deadline_never_accepts(when):
    c,q,sent=fixture()
    c._native_cancelled=lambda: when=="before" or bool(sent)
    with pytest.raises(MoveItUnavailable,match="CANCELLED_OR_DEADLINE"):invoke(c,q)
    assert len(sent)==(when=="after")

@pytest.mark.parametrize("bad",["dimension","nan","limits","position","rotation","failure"])
def test_strict_return_rejection(bad):
    c,q,sent=fixture()
    raw=dict(status="SUCCESS",q=q.tolist())
    if bad=="dimension":raw["q"]=[0.]
    if bad=="nan":raw["q"][0]=float("nan")
    if bad=="limits":raw["q"][0]=4.
    if bad=="failure":raw["status"]="NATIVE_IK_FAILED"
    if bad in ("position","rotation"):
        pose=np.eye(4)
        if bad=="position":pose[0,3]=.01
        else:pose[:2,:2]=[[0,-1],[1,0]]
        c.robot.fk=lambda q:pose
    c.native.request=lambda *a,**k:raw
    if bad in ("dimension","nan","limits"):
        with pytest.raises(MoveItUnavailable,match="INVALID_NATIVE_IK_RETURN"):invoke(c,q)
    else:
        out,f=invoke(c,q);assert out is None and f

@pytest.mark.parametrize("bad",["old_entry","extraction_entry","stage","purpose","budget","unknown","target","mask","parent","support"])
def test_protocol_scope_rejects_before_dispatch(bad):
    r=transit_request("ik");r.update(planning_only=True,ik_mode="bounded_preplace",solver_budget_s=.1)
    if bad=="stage":r["stage"]="pregrasp"
    if bad=="purpose":r["purpose"]="EXTRACTION_PROCESS"
    if bad=="budget":r["solver_budget_s"]=0.
    if bad=="unknown":r["ik_mode"]="random_batch"
    if bad=="target":r["attachment"]["id"]="wrong"
    if bad=="mask":r["process_policy"]["actual_contact_mask"]=[False]*72
    if bad=="parent":r["parent_stage_id"]=""
    if bad=="support":r["process_policy"]["support_names"]=["support"]
    c=SingleContactClient(NS(request=lambda *a,**k:pytest.fail("dispatch")),
        preplace_enabled=bad not in {"old_entry","extraction_entry"},extraction_enabled=bad=="extraction_entry")
    with pytest.raises(MoveItUnavailable):c.request(r)

def test_only_suffix_enables_bounded_ik_and_one_transit(monkeypatch):
    c,t,q,w,p,r,calls,checks,transit,ik,captured=integration_fixture(monkeypatch)
    out,row=c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert out and len(ik)==len(transit)==1
    assert ik[0]["ik_mode"]=="bounded_preplace" and ik[0]["solver_budget_s"]==.1

def test_ik_failure_does_not_dispatch_transit(monkeypatch):
    c,t,q,w,p,r,calls,checks,transit,ik,captured=integration_fixture(monkeypatch)
    c.ik_pose_once=lambda *a,**k:(None,dict(reason="NATIVE_IK_FAILED"))
    out,row=c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert out is None and not row["complete"] and not transit

def test_unobservable_native_attempts_are_null():
    from pathlib import Path
    src=Path("ros2/m710_moveit_backend/src/worker.cpp").read_text()
    assert '{"internal_attempts",bounded?J(nullptr):J(1)}' in src
    assert '{"random_restarts",bounded?J(nullptr):J(0)}' in src
    assert '"NOT_OBSERVABLE"' in src


def test_cancel_during_request_construction_does_not_solve():
    c,q,sent=fixture();original=c._build_native_request
    def build(*a,**k):
        r=original(*a,**k);c._native_cancelled=lambda:True;return r
    c._build_native_request=build
    with pytest.raises(MoveItUnavailable,match="CANCELLED_OR_DEADLINE"):invoke(c,q)
    assert not sent
