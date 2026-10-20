"""Geometry regressions and explicitly synthetic failures, not physical evidence."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.lifted_reception import translated,StartHeight,bounds
from unloading_sim.geometry import OBB
from unloading_sim.top_layer_lifted import FrozenTopLayer,process_options
from unloading_sim.junction_reception import (
    reception_classification,require_receive_height,validate_junction_release,
    temporary_heights,synchronization_options,reject_late_sync)
from unloading_sim.coupled_batch import CoupledCycleConnector,CoupledBatchClient,nominal_drop,message_box
from unloading_sim.moveit2_backend import MoveItUnavailable
from time import perf_counter


@pytest.fixture(scope="module")
def layout():
    return load_workcell_layout(Path(__file__).resolve().parents[1]/"configs/workcells/m710id70_unloading_layout_v1.yaml")


def world(layout,height):
    state=coupled_conveyor_state(layout,height,layout.cartons())
    return list(state.boxes),{b.name:b for b in state.boxes}


@pytest.mark.parametrize("name,junction",[
    ("carton_l07_c00",True),("carton_l07_c01",True),("carton_l07_c02",False),("carton_l07_c04",False)])
def test_full_projection_inlet_not_box_name_or_center(layout,name,junction):
    _,by=world(layout,1.75)
    target=by[name];received=translated(target,[-.85,0,0])
    classification=reception_classification(received,by["conveyor_transverse"],by["conveyor_longitudinal"])
    assert classification["junction"] is junction
    assert classification["outfeed_direction_world"]==([-1.,0.,0.] if junction else [0.,-1.,0.])
    renamed=OBB(received.center,received.half_extents,received.rotation,"arbitrary-name","carton")
    assert reception_classification(renamed,by["conveyor_transverse"],by["conveyor_longitudinal"])==classification
    # c01's footprint crosses the inlet boundary; a centre-only test is not the rule.
    if name.endswith("c01"):
        crossing=translated(received,[0,.1,0])
        assert crossing.center[1]>-.4
        assert reception_classification(crossing,by["conveyor_transverse"],by["conveyor_longitudinal"])["junction"]


def test_process_hmin_is_mandatory_only_for_junction_front(layout):
    cartons=layout.cartons();frozen=FrozenTopLayer.capture(cartons);_,by=world(layout,2.05)
    for name,h in [("carton_l07_c00",1.75),("carton_l07_c02",2.05)]:
        opts=process_options(by[name],by["conveyor_transverse"],frozen,cartons,longitudinal=by["conveyor_longitudinal"])
        assert opts[0]["height_m"]==pytest.approx(h)
        if name.endswith("c00"):
            assert not any(o["template"]=="FRONT_DIRECT_HMAX" for o in opts)
            assert opts[0]["segment_directions_world"]==[[-1.,0.,0.]]


def test_actual_world_and_actual_release_region_override_option_labels(layout):
    low,by=world(layout,1.75);original=by["carton_l07_c00"];received=translated(original,[-.85,0,0])
    check=validate_junction_release(received,original,low,[1.,0.,0.],1.75)
    assert check["passed"] and check["junction"]
    high,_=world(layout,2.05)
    with pytest.raises(ValueError,match="BEFORE_REQUIRED"):require_receive_height(high,1.75)
    with pytest.raises(ValueError,match="REQUIRES_HMIN"):
        validate_junction_release(received,original,high,[1.,0.,0.],2.05)
    with pytest.raises(ValueError,match="Y_OR_ORIENTATION"):
        validate_junction_release(translated(received,[0,.4,0]),original,low,[1.,0.,0.],1.75)


def test_three_heights_resolution_not_fixed_height_quota(layout):
    target=next(b for b in layout.cartons() if b.name=="carton_l07_c00")
    capture=StartHeight.capture(target)
    assert capture.z_b==pytest.approx(2.10) and capture.h==pytest.approx(.30)
    assert capture.hmin==pytest.approx(1.75) and capture.hmax==pytest.approx(2.05)
    samples=list(temporary_heights(capture.hmin,.6))
    assert samples[:2]==pytest.approx([1.75,1.70]) and samples[0]!=.6
    assert len(samples)>100 and len(set(samples))==len(samples)
    assert max(np.diff(sorted(samples)))<=.01+1e-9
    option=dict(height_m=1.75,reception=dict(junction=True))
    routes=list(synchronization_options(option,2.05,.6))
    assert routes[0]["synchronization"]=="ALREADY_AT_RECEIVE"
    assert routes[0]["H_temporary_m"]==routes[0]["H_receive_m"]==1.75
    assert routes[0]["H_current_m"]==2.05
    assert [x["synchronization"] for x in routes[1:3]]==["PREGRASP","CONTACT"]
    assert all(.6<=r["H_temporary_m"]<=1.75 for r in routes)
    with pytest.raises(ValueError):list(temporary_heights(.5,.6))


@pytest.mark.parametrize("option",[
    {"late_receiver_sync":True},
    {"synchronization":"CHECKED_LOW_APPROACH_EXTRACTION_THEN_ATTACHED_LIFT"}])
def test_old_late_sync_cannot_dispatch_or_commit(option):
    with pytest.raises(ValueError,match="DISABLED"):reject_late_sync(option)
    c=CoupledCycleConnector.__new__(CoupledCycleConnector);c.option=option
    with pytest.raises(ValueError,match="DISABLED"):c._build_native_request(None,None,[])


def test_native_client_refuses_attached_extraction_lift():
    client=CoupledBatchClient(SimpleNamespace(request=lambda *a,**k:pytest.fail("must not dispatch")),binding={})
    with pytest.raises(MoveItUnavailable,match="FORBIDDEN"):
        client.request(dict(op="plan",planning_mode="cold_from_scratch",stage="extraction",
            pipeline_id="external_axis",planner_id="COUPLED_LIFT",attachment={"id":"target"},
            parent_stage_id="native-parent",process_policy={"initial_proximity":None,"support_names":[]}))


def test_hmin_direct_does_not_force_another_lift(layout):
    obstacles,_=world(layout,1.75);q=np.arange(6.)
    c=CoupledCycleConnector.__new__(CoupledCycleConnector)
    c.option=dict(height_m=1.75,synchronization="ALREADY_AT_RECEIVE")
    c.lift_at=lambda *a,**k:pytest.fail("unnecessary height motion")
    row={};end,actual=c.prepare_contact_start(q,None,obstacles,71070,row)
    assert end is q and actual is obstacles and c.receiver_raised


@pytest.mark.parametrize("where",["PREGRASP","CONTACT"])
def test_allowed_sync_locations_use_actual_state_and_keep_target(layout,where):
    obstacles,by=world(layout,1.70);high,_=world(layout,1.75);q=np.arange(6.)
    c=CoupledCycleConnector.__new__(CoupledCycleConnector)
    c.option=dict(height_m=1.75,synchronization=where);c.receiver_raised=False
    seen=[]
    def lift(start,target,world,seed,row,**kw):
        assert start is q and world is obstacles and any(b is by["carton_l07_c00"] for b in world)
        seen.append(kw["stage"]);return start,high,True
    c.lift_at=lift
    end,actual=c.prepare_contact_start(q,None,obstacles,71070,{})
    assert end is q
    assert seen==(["pregrasp"] if where=="PREGRASP" else [])
    if where=="CONTACT":assert actual is obstacles and not c.receiver_raised


def test_failed_pregrasp_sync_does_not_claim_contact_at_hmin(layout):
    obstacles,_=world(layout,1.70);c=CoupledCycleConnector.__new__(CoupledCycleConnector)
    c.option=dict(height_m=1.75,synchronization="PREGRASP");c.receiver_raised=False
    c.lift_at=lambda q,t,w,s,r,**kw:(q,w,False)  # Synthetic intermediate-sweep failure.
    row={}
    c.prepare_contact_start(np.zeros(6),None,obstacles,71070,row)
    assert row["failure"]["reason"]=="NO_CHECKED_RECEIVER_SYNC_AT_GENERATED_ENDPOINTS"
    assert not c.receiver_raised and require_receive_height(obstacles,1.70)==pytest.approx(1.70)


def test_hmin_drop_is_box_only_full_35cm_and_freezes_initial_height(layout):
    _,by=world(layout,1.75);original=by["carton_l07_c00"];box=translated(original,[-.85,0,0])
    capture=StartHeight.capture(original);drop=nominal_drop(box,by["conveyor_transverse"])
    assert drop["drop_m"]==pytest.approx(.35)
    assert drop["samples"][-1]["t"]==pytest.approx(np.sqrt(.7/9.81))
    for s in drop["samples"]:
        moved=message_box(s["box"])
        np.testing.assert_array_equal(moved.rotation,original.rotation)
        assert moved.center[:2]==pytest.approx(box.center[:2])
    assert bounds(message_box(drop["samples"][-1]["box"]))[1][2]==pytest.approx(2.05)
    assert capture.hmin==pytest.approx(1.75)


def test_height_changes_share_prefix_and_suffix_deadlines(monkeypatch):
    from unloading_sim.single_right_receiver import SingleRightReceiverConnector
    monkeypatch.setattr(SingleRightReceiverConnector,"_native_limits",lambda self:(90.,100.))
    shared={"prefix":perf_counter()+2.}
    first=CoupledCycleConnector.__new__(CoupledCycleConnector)
    second=CoupledCycleConnector.__new__(CoupledCycleConnector)
    for c in (first,second):
        c.height_stage_deadlines=shared;c.height_prefix_active=True
        c.height_visit_deadline=perf_counter()+20.;c.budget=SimpleNamespace(stage_wall_time_s=90.)
        assert 0<c._native_limits()[0]<=2.
    shared["prefix"]=perf_counter()-1
    with pytest.raises(MoveItUnavailable,match="STAGE_DEADLINE"):second._native_limits()
    first.height_prefix_active=False;first._native_limits()
    suffix=shared["suffix"]
    second.height_prefix_active=False;second._native_limits()
    assert shared["suffix"]==suffix


def test_junction_commit_rejects_missing_full_acceptance_before_state_change(layout):
    from unloading_sim.coupled_batch import BatchState
    state=BatchState(FrozenTopLayer.capture(layout.cartons()),tuple(layout.cartons()),np.zeros(6),1.75)
    root=deepcopy(state.root())
    with pytest.raises(ValueError,match="COMPLETE_ACCEPTANCE"):
        state.commit(dict(option=dict(reception=dict(junction=True)),height_m=1.75))
    assert state.root()==root
