"""Focused synthetic protocol/geometry regressions; real cases are archived separately."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import subprocess
import sys
import numpy as np
import pytest
from test_single_contact import targets
from test_single_extraction import suffix_fixture, extraction_request
from unloading_sim.geometry import OBB
from unloading_sim.conveyor_placement import PlacementPolicy, placement_working_normal
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.single_contact import SingleContactClient, plan_faces
from unloading_sim.single_preplace import SinglePreplaceConnector
from unloading_sim.preplace_target import fixed_preplace_target
from unloading_sim.moveit2_backend import MoveItUnavailable
from unloading_sim.collision_policy import SimulationCollisionPolicy

def geometry_fixture(face=0):
    t=targets()[0][face]
    a=NS(rigid=RigidAttachment.capture(t.contact_pose,t.box))
    receiver=OBB([-1.8,-1.5,.7],[.65,.35,.05],np.eye(3),"belt","conveyor")
    c=NS(native_scene=NS(policy=NS(data=dict(receiver=dict(name="belt",fixed=True),
        search_strategy=dict(surface_process_families={"belt":"longitudinal"},
            allowed_placement_families={"longitudinal":["TOP_DOWN","RIGHT_WALL_FACING"],
                "transverse":["TOP_DOWN","TRANSVERSE_SIDE"]},
            overlap_process_priority=["longitudinal","transverse"]))),occupied=[]),
        placement_policy=PlacementPolicy(),surface_directions_world={"belt":(-1,0,0)},
        budget=NS(preplace_standoff_m=.1,receiver_edge_reserve_m=.01,
            receiver_runtime_clearance_reserve_m=.003),
        contact_tolerance_m=.0002,collision_margin_m=.005,
        collision_policy=SimulationCollisionPolicy.from_mapping(dict(
            schema="m710_poc_pair_collision_policy_v4",required_pair_clearance_m=.005,
            boundary_mode="finite_frozen_scene_walls",free_space_clearance_m=.0052)),
        physical_contact_pose=lambda q:t.contact_pose,
        virtual_from_physical=lambda x:x.copy(),physical_from_virtual=lambda x:x.copy(),
        native_compliant=set(),stack_carton_names=set(),
        robot_state_validator=NS(_compliant_boxes=lambda q:[],nominal_cup_compression_m=.005))
    c.tool_collision_obbs_provider=lambda q:[OBB(t.contact_pose[:3,3]-.03*t.contact_pose[:3,2],
        [.01]*3,t.contact_pose[:3,:3],"tool","tool")]
    return c,t,a,np.zeros(6),[receiver]

@pytest.mark.parametrize("face,half",[(0,.3),(1,.15)])
def test_rotated_box_bottom_and_actual_attachment_roundtrip(face,half):
    c,t,a,q,w=geometry_fixture(face)
    # Retain an actual capture differing by machine precision.
    pose=t.contact_pose.copy();pose[0,3]=np.nextafter(pose[0,3],np.inf)
    a.rigid=RigidAttachment.capture(pose,t.box)
    before=a.rigid.tcp_from_box.copy()
    g,f=fixed_preplace_target(c,t,a,q,w)
    assert f is None
    np.testing.assert_array_equal(g.tcp_from_box,before)
    np.testing.assert_allclose(g.physical_contact_pose@g.tcp_from_box,g.preplace_box.world_from_local,atol=1e-14)
    assert g.geometry["rotated_vertical_half_extent_m"]==pytest.approx(half)
    assert g.landing_box.center[2]==pytest.approx(.75+half)
    assert g.preplace_box.center[2]-g.landing_box.center[2]==pytest.approx(.1)
    assert g.geometry["support"]["supported"] and g.geometry["edge_reserve"]["supported"]
    assert not g.evidence()["received"]
    assert not np.array_equal(g.task_tcp_pose[:3,3],g.preplace_box.center)
    np.testing.assert_array_equal(g.physical_contact_pose[:3,2],[0,0,-1])
    np.testing.assert_array_equal(placement_working_normal("RIGHT_WALL_FACING"),[0,-1,0])

@pytest.mark.parametrize("bad",["occupied","footprint","edge","direction","tool","standoff","normal"])
def test_fixed_target_rejects_without_search_or_clearance_relaxation(bad):
    c,t,a,q,w=geometry_fixture()
    if bad=="occupied":w.append(OBB([-1.8,-1.5,1.05],[.1]*3,np.eye(3),"occupied","carton"))
    elif bad=="footprint":w[0]=OBB(w[0].center,[.1,.1,.05],np.eye(3),"belt","conveyor")
    elif bad=="edge":c.budget.receiver_edge_reserve_m=.6
    elif bad=="direction":c.surface_directions_world["belt"]=(0,0,-1)
    elif bad=="tool":
        c.tool_collision_obbs_provider=lambda q:[OBB(t.contact_pose[:3,3],[2]*3,np.eye(3),"tool","tool")]
    elif bad=="standoff":c.budget.preplace_standoff_m=.001
    else:
        c.native_scene.policy.data["search_strategy"]["allowed_placement_families"]["longitudinal"]=["TRANSVERSE_SIDE"]
    g,f=fixed_preplace_target(c,t,a,q,w)
    assert g is None and f["reason"]=="FIXED_PREPLACE_TARGET_NOT_APPLICABLE"

def transit_request(op="plan"):
    r=extraction_request(op)
    r.update(stage="transit",purpose="FREE_LOADED_TRANSFER",
        pipeline_id="ompl",planner_id="RRTConnectkConfigDefault",ownership_transition=None,
        parent_stage_id="extraction")
    r["process_policy"].update(support_names=[],initial_proximity=None)
    return r

@pytest.mark.parametrize("op",["ik","plan","check_transit_end"])
def test_transit_protocol_only_attached_free_context(op):
    calls=[]
    client=SingleContactClient(NS(request=lambda r,**k:calls.append(r) or dict(status="SUCCESS")),
        preplace_enabled=True)
    r=transit_request(op)
    assert client.request(r)["status"]=="SUCCESS"
    assert len(calls)==1 and not client.records[-1]["target_in_world"]

@pytest.mark.parametrize("bad",["default","extraction_only","duplicate","target","mask","support",
    "proximity","purpose","parent","ownership","ptp","lin","prior","pregrasp"])
def test_transit_fences(bad):
    client=SingleContactClient(NS(request=lambda *a,**k:pytest.fail("dispatch forbidden")),
        preplace_enabled=bad not in ("default","extraction_only"),extraction_enabled=bad=="extraction_only")
    r=transit_request()
    if bad=="duplicate":r["world"].append(dict(id="box"))
    elif bad=="target":r["attachment"]["id"]="wrong"
    elif bad=="mask":r["process_policy"]["actual_contact_mask"]=[False]*72
    elif bad=="support":r["process_policy"]["support_names"]=["support"]
    elif bad=="proximity":r["process_policy"]["initial_proximity"]={}
    elif bad=="purpose":r["purpose"]="EXTRACTION_PROCESS"
    elif bad=="parent":r["parent_stage_id"]=""
    elif bad=="ownership":r["ownership_transition"]="ATTACH"
    elif bad=="ptp":r.update(pipeline_id="pilz_industrial_motion_planner",planner_id="PTP")
    elif bad=="lin":r.update(pipeline_id="pilz_industrial_motion_planner",planner_id="LIN")
    elif bad=="prior":r["op"]="prior_query"
    elif bad=="pregrasp":r["stage"]="pregrasp"
    with pytest.raises(MoveItUnavailable):client.request(r)

def integration_fixture(monkeypatch):
    import unloading_sim.single_preplace as module
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    c.__class__=SinglePreplaceConnector
    original_motion=c._native_plan
    original_records=c._native_selected_records
    original_endpoint=c.native.request
    c._native_state_receipt=lambda state:NS(stage_id=state.native_receipt["stage_id"])
    c._native_parent_for_request=lambda state,req:(NS(stage_id=state.native_receipt["stage_id"],
        evidence=lambda:state.native_receipt),{},None)
    transit=[];ik=[];captured={}
    class End(np.ndarray):
        @property
        def native_receipt(self):return dict(task_id="task",stage_id="transit")
    def goal(c2,t2,a,start,w):
        assert c2 is c and start.native_receipt["stage_id"]=="extraction"
        assert c._native_contact_context["selection"]["actual_contact_mask"]==[True]*72
        captured.update(attachment=a,start=start)
        p=c.physical_contact_pose(start).copy();p[1,3]-=1.
        b=p@a.rigid.tcp_from_box
        result=NS(task_tcp_pose=p,physical_contact_pose=p,
            preplace_box=OBB(b[:3,3],a.rigid.half_extents,b[:3,:3],"box","payload"),
            evidence=lambda:dict(target_id="box",received=False))
        return result,None
    monkeypatch.setattr(module,"fixed_preplace_target",goal)
    def inverse(p,start,w,seed,**kw):
        ik.append(kw)
        assert start is captured["start"] and kw["attachment"] is captured["attachment"]
        assert kw["purpose"].value=="FREE_LOADED_TRANSFER"
        return np.r_[p[:3,3],[0.,0.,0.]],None
    c.ik_pose_once=inverse
    def motion(start,goal,w,**kw):
        if kw["stage"]=="extraction":return original_motion(start,goal,w,**kw)
        transit.append(kw)
        assert start is captured["start"] and kw["attachment"] is captured["attachment"]
        assert kw["support_names"]==() and kw["initial_proximity"] is None
        assert kw["try_ptp"] is kw["try_prior"] is False and kw["max_ompl_attempts"]==1
        assert kw["purpose"].value=="FREE_LOADED_TRANSFER" and kw["allow_ompl"]
        assert "box" not in {b.name for b in w}
        end=goal.copy().view(End);end[0]=np.nextafter(end[0],np.inf)
        captured["end"]=end
        return [start,end],None,{}
    c._native_plan=motion
    def records(state):
        rows=original_records(state)
        if state.native_receipt["stage_id"]=="transit":
            rows.append(dict(stage="transit",stage_id="transit",parent_stage_id="extraction",
                generation_source="OMPL",points=[dict(q=state.tolist(),v=[0.]*6)]))
        return rows
    c._native_selected_records=records
    def endpoint(req,**kw):
        if req["stage"]=="extraction":return original_endpoint(req,**kw)
        checks.append(req)
        assert req["op"]=="check_transit_end" and req["parent_stage_id"]=="transit"
        return dict(status="SUCCESS",task_id="task",parent_stage_id="transit",target_id="box",
            target_attached=True,target_in_world=False,free_space_rules_restored=True,checked_states=1,
            target_world_pose=captured["attachment"].box_at(np.asarray(req["q_start"])).world_from_local.tolist())
    c.native.request=endpoint
    return c,t,q,world,plan,row,calls,checks,transit,ik,captured

def test_context_handoff_actual_end_and_unique_methods(monkeypatch):
    c,t,q,w,p,r,calls,checks,transit,ik,captured=integration_fixture(monkeypatch)
    out,r=c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert out and r["complete"] and len(calls)==len(transit)==len(ik)==1 and len(checks)==2
    assert out["transit"][0]==out["extraction"][-1]
    assert out["transit"][-1]!=r["preplace_ik_q_rad"]
    assert out["final_state_receipt"]==dict(task_id="task",stage_id="transit")
    assert out["stages"][-1]["parent_stage_id"]=="extraction"
    assert len(out["events"])==1 and out["events"][0]["event_type"]=="ATTACH"
    assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72
    assert out["preplace_endpoint"]["native_check"]["target_attached"]

@pytest.mark.parametrize("bad",["mid_payload","mid_tool","endpoint","velocity","parent","identity",
    "cancel","deadline","scene"])
def test_no_success_prefix_and_request_failures_stop(monkeypatch,bad):
    c,t,q,w,p,r,calls,checks,transit,ik,captured=integration_fixture(monkeypatch)
    original=c._native_plan
    def changed(*args,**kw):
        path,f,e=original(*args,**kw)
        if kw["stage"]=="transit":
            if bad.startswith("mid_"):return [],dict(reason="NATIVE_PLANNING_FAILED",pair=[bad,"wall"]),{}
            if bad=="endpoint":path[-1][1]+=.1
            if bad in ("cancel","deadline","scene"):
                c.check_context=lambda *a:(_ for _ in ()).throw(MoveItUnavailable(bad.upper()))
        return path,f,e
    c._native_plan=changed
    records=c._native_selected_records
    def modified(state):
        rows=records(state)
        if state.native_receipt["stage_id"]=="transit":
            if bad=="velocity":rows[-1]["points"][-1]["v"][0]=.1
            if bad=="parent":rows[-1]["parent_stage_id"]="other"
        return rows
    c._native_selected_records=modified
    if bad=="identity":
        endpoint=c.native.request
        c.native.request=lambda req,**kw:dict(endpoint(req,**kw),target_id="other") if req["stage"]=="transit" else endpoint(req,**kw)
    if bad in ("velocity","parent","identity","cancel","deadline","scene"):
        with pytest.raises(MoveItUnavailable):c.complete_contact_plan(p,r,q,t,w,{},71070)
    else:
        out,r=c.complete_contact_plan(p,r,q,t,w,{},71070)
        assert out is None and r["failure"]["scope"]=="FACE_ATTEMPT"
    assert not r["complete"]
    assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72

def test_stage_flags_are_mutually_exclusive():
    r=subprocess.run([sys.executable,"tools/plan_m710_approach_contact.py","--target","x","--output","unused",
        "--worker-command","unused","--worker-binary","unused","--source-commit","unused",
        "--through-extraction","--through-preplace"],capture_output=True,text=True)
    assert r.returncode==2 and "not allowed with argument" in r.stderr



def test_failed_transit_next_face_keeps_root_and_restored_masks(monkeypatch):
    c,t,q,w,p,r,calls,checks,transit,ik,captured=integration_fixture(monkeypatch)
    original=c._native_plan
    def fail_transit(*args,**kw):
        if kw["stage"]=="transit":return [],dict(reason="NATIVE_PLANNING_FAILED"),{}
        return original(*args,**kw)
    c._native_plan=fail_transit
    roots=[]
    def attempt(target,start,world,suction,seed):
        roots.append(start.copy())
        assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72
        if len(roots)==1:return c.complete_contact_plan(deepcopy(p),deepcopy(r),q,t,w,{},seed)
        return dict(complete=True),dict(complete=True)
    c.plan_target=attempt
    out,rows=plan_faces(c,targets()[0],np.zeros(6),np.zeros(6),"configured",w,{},71070)
    assert out and len(rows)==2 and not rows[0]["complete"]
    assert all(np.array_equal(root,np.zeros(6)) for root in roots)

@pytest.mark.parametrize("mutation",["receipt","selection","capture","half"])
def test_handoff_rejects_stale_loaded_context_before_ik(monkeypatch,mutation):
    c,t,q,w,p,r,calls,checks,transit,ik,captured=integration_fixture(monkeypatch)
    true_hook=c.complete_extraction_plan
    def changed(plan,row,start,target,world,attachment,seed):
        if mutation=="receipt":plan["final_state_receipt"]={}
        elif mutation=="selection":c._native_contact_context["selection"]["actual_contact_mask"]=[False]*72
        elif mutation=="capture":row["attachment"]["tcp_from_box"][0][3]+=.01
        else:attachment.rigid.half_extents[0]+=.01
        return true_hook(plan,row,start,target,world,attachment,seed)
    c.complete_extraction_plan=changed
    with pytest.raises(MoveItUnavailable,match="TRANSIT_EXTRACTION_CONTEXT_CHANGED"):
        c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert not ik and not transit and not r["complete"]
