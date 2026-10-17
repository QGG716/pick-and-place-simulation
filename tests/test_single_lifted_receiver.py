"""Scoped Hmax integration contracts; synthetic responses are not native evidence."""
from copy import deepcopy
from itertools import islice, count
from types import SimpleNamespace
import numpy as np
import pytest

from test_lifted_reception import layout, box
from test_single_extraction import suffix_fixture
from unloading_sim.lifted_reception import receiving_region, translated, DEFERRED, process_scheme
from unloading_sim.single_lifted_receiver import (receiver_binding, region_samples,
    release_ready_geometry, SingleLiftedReceiverConnector)
from unloading_sim.single_contact import SingleContactClient, SingleContactConnector
from unloading_sim.moveit2_backend import MoveItUnavailable, box_message, digest


def test_binding_preserves_default_and_only_moves_unique_belt(layout):
    original=layout.fixed_components()+layout.cartons()
    frozen=[box_message(b) for b in original]
    target=next(b for b in original if b.name=="carton_l07_c02")
    binding,high=receiver_binding(target,original)
    assert [box_message(b) for b in original]==frozen
    assert high.center[2]+high.half_extents[2]==pytest.approx(2.05)
    assert binding["low"]["pose"][2][3]==pytest.approx(.54)
    world=[high if b.name==high.name else b for b in original]
    assert sum(a!=b for a,b in zip(frozen,[box_message(b) for b in world]))==1
    assert digest(frozen)!=digest([box_message(b) for b in world])
    with pytest.raises(ValueError,match="IDENTITY"):
        receiver_binding(target,[*original,high])


def test_region_is_not_midpoint_or_old_point_eight_limit(layout):
    target=next(b for b in layout.cartons() if b.name=="carton_l07_c02")
    _,belt=receiver_binding(target,layout.fixed_components()+layout.cartons())
    region=receiving_region(target,belt)
    samples=list(islice(region_samples(region,belt,target,.0001),9))
    assert samples[:3]==pytest.approx([-.5102,-.5898,-.55])
    assert len(set(samples))==9 and all(target.center[0]-x>.8 for x in samples)
    assert region["robot_reachability"]=="NOT_EVALUATED"
    occupant=box((-.55,0,2.25),(.8,2,.15),"occupant")
    blocked=receiving_region(target,belt,[occupant])
    assert not blocked["exists"] and blocked["geometric_fit"]
    assert blocked["region_status"]=="CONSERVATIVE_OCCUPANCY_NEEDS_CHECK"
    assert list(islice(region_samples(blocked,belt,target,.0001),3))==samples[:3]


def test_first_lin_failure_does_not_reject_region_or_mutate_start(layout):
    c,target,start,obstacles,plan,row,*_=suffix_fixture()
    c.__class__=SingleLiftedReceiverConnector
    c.high_belt=box((-.55,0,target.box.center[2]-.15-.05-.06),(.34,.8,.06),"conveyor_transverse")
    c._single_connection_deadline=None
    calls=[];frozen=start.copy()
    attachment=SimpleNamespace(box_at=lambda q:target.box)
    tracker=SimpleNamespace(clone=lambda:object())
    def native(q,goal,world,**kw):
        assert np.array_equal(q,frozen) and kw["attachment"] is attachment
        assert not kw["allow_ompl"] and not kw["try_ptp"] and not kw["try_prior"]
        assert kw["stage"]=="extraction" and kw["purpose"].value=="EXTRACTION_PROCESS"
        calls.append(kw)
        if len(calls)==1:return [],dict(reason="NATIVE_PLANNING_FAILED"),{}
        return [q,q.copy()],None,{}
    c._native_plan=native
    path,geometry,failure=c.plan_extraction(start,target,attachment,[],[],tracker,71070,row)
    assert failure is None and len(calls)==2 and len(row["region_attempts"])==2
    assert not row["region_attempts"][0]["selected"] and row["region_attempts"][1]["selected"]
    assert calls[0]["goal_pose"][0,3]!=calls[1]["goal_pose"][0,3]
    assert np.array_equal(calls[0]["goal_pose"][:3,:3],c.physical_contact_pose(start)[:3,:3])
    assert geometry["maximum_distance_m"] is None and c.budget.maximum_extraction_m==.8
    assert c._single_connection_deadline is None


def test_actual_end_controls_release_ready_and_does_not_release(layout):
    target=next(b for b in layout.cartons() if b.name=="carton_l07_c02")
    _,belt=receiver_binding(target,layout.fixed_components()+layout.cartons())
    end=translated(target,[-.85,0,0])
    opts=dict(tolerance=.0001,expected_bottom=2.1,expected_rotation=target.rotation)
    ok=release_ready_geometry(end,belt,**opts)
    assert ok["passed"] and ok["actual_gap_m"]==pytest.approx(.05)
    assert ok["physical_execution"]=="NOT_RUN"
    assert not release_ready_geometry(translated(end,[.1,0,0]),belt,**opts)["passed"]
    assert not release_ready_geometry(translated(end,[0,0,-.05]),belt,**opts)["passed"]
    assert all(v not in {"PASS","SUCCESS"} for v in DEFERRED.values())


def test_lift_scope_not_enabled_in_old_client():
    c=SingleContactClient(SimpleNamespace(request=lambda *a,**k:pytest.fail("dispatched")))
    with pytest.raises(MoveItUnavailable,match="FORBIDDEN"):
        c.request(dict(op="plan",planning_mode="cold_from_scratch",stage="pregrasp",
            pipeline_id="external_axis",planner_id="LIFT_Z",attachment=None))
    with pytest.raises(MoveItUnavailable,match="FORBIDDEN"):
        c.request(dict(op="check_receiver_state",planning_mode="cold_from_scratch"))
    start=np.arange(6.)
    plain=SingleContactConnector.__new__(SingleContactConnector)
    obstacles=[]
    assert plain.prepare_contact_start(start,None,obstacles,1,{})==(start,obstacles)


def test_lift_commits_new_receipt_only_after_native_success():
    c=SingleLiftedReceiverConnector.__new__(SingleLiftedReceiverConnector)
    q=np.nextafter(np.arange(6.),np.inf)
    low=box((-.55,.35,.54),(.34,.8,.06),"conveyor_transverse")
    high=translated(low,[0,0,1.45]);target=box((.3,0,2.25))
    c.active_obstacles=[low,target];c.high_belt=high
    c.receiver_sync_attempts=[];c.receiver_events=[];c.receiver_raised=False
    c._native_stage_ids=count(2);c.native_task_id="this-task";c.native_generated=[]
    c.check_context=lambda *a:None;c._native_limits=lambda:(90,100);c._native_cancelled=lambda:False
    receipt=SimpleNamespace(stage_id="this-task:1",evidence=lambda:dict(stage_id="this-task:1"))
    c._native_parent_for_request=lambda *a:(receipt,{},None)
    c._build_native_request=lambda *a,**kw:dict(world=[box_message(low),box_message(target)],stage="pregrasp")
    c._native_state_context=lambda req:dict(world=deepcopy(req["world"]))
    issued=[]
    class State(np.ndarray):
        native_receipt={"stage_id":"this-task:3"}
    c._native_issue_state=lambda value,stage,ctx:(issued.append((value.copy(),stage,ctx)) or value.view(State))
    outcomes=iter(["RECEIVER_SWEEP_REJECTED","SUCCESS"])
    def request(req,**kw):
        return dict(status=next(outcomes),task_id="this-task",stage_id=req["stage_id"],
            parent_stage_id=receipt.stage_id,generation_source="NATIVE_EXTERNAL_AXIS",terminal_q=q.tolist(),
            external_axis=dict(robot_q_constant=True,physical_execution="NOT_RUN"))
    c.native=SimpleNamespace(request=request)
    original=deepcopy(c.active_obstacles);row={}
    out,world,success=c.lift_at(q,target,original,1,row,stage="pregrasp")
    assert not success and not issued and not c.receiver_raised
    out,world,success=c.lift_at(q,target,original,1,row,stage="contact")
    assert success and len(issued)==1 and np.array_equal(issued[0][0],q)
    assert c.receiver_raised and issued[0][1]=="this-task:3"
    assert issued[0][2]["world"][1]==box_message(target)
    assert c.receiver_events[0]["event_type"]=="EXTERNAL_AXIS_MOTION"
    assert c.receiver_events[0]["robot_q_constant"]

