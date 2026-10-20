"""Focused no-prior target and dispatch tests; synthetic replies are not run evidence."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.support import SupportRelationGraph, SupportEdge
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.single_contact import (SingleContactConnector, SingleContactClient, plan_faces,
    check_contact_samples)
from unloading_sim.independent_cups import m710_cup_array_from_mapping
from unloading_sim.moveit2_backend import MoveItUnavailable, digest, box_message
from unloading_sim.layout_single_carton import virtual_tcp_from_physical_contact, physical_contact_from_virtual_tcp


def scene(rotation=None, neighbours=(),edges=()):
    b=OBB([1.,0.,1.], [.3,.3,.15], np.eye(3) if rotation is None else rotation,"box","carton")
    boxes=[b,*neighbours]
    s=SimpleNamespace(support_graph=SupportRelationGraph(boxes,edges),
        snapshot={"scene_fingerprint":"scene"},policy=SimpleNamespace(data={
            "task_population":{"face_modes":["front","top","side"]}}))
    return s,b


def targets(rotation=None,base=(0,0,0),neighbours=(),edges=()):
    s,b=scene(rotation,neighbours,edges)
    return simulation_contact_targets(s,b,base,.1)


@pytest.mark.parametrize("yaw",[0,np.pi/2,np.pi,-np.pi/2])
def test_physical_normal_and_rotated_face_mapping(yaw):
    items,screen=targets(rotation_matrix_from_rpy(0,0,yaw))
    assert [r["face_role"] for r in screen]==["FRONT","TOP","SIDE","SIDE"]
    for t in items:
        t.validate()
        np.testing.assert_allclose(t.contact_pose[:3,2],-t.outward_normal,atol=1e-12)
        np.testing.assert_allclose(t.approach_pose[:3,3]-t.contact_pose[:3,3],.1*t.outward_normal,atol=1e-12)
        np.testing.assert_array_equal(t.approach_pose[:3,:3],t.contact_pose[:3,:3])
        assert np.linalg.det(t.contact_pose[:3,:3])==pytest.approx(1.)
    front=screen[0]
    expected={0:"x-",np.pi/2:"y+",np.pi:"x+",-np.pi/2:"y-"}[yaw]
    assert front["geometric_face_id"]==expected
    if yaw==np.pi:
        assert front["reason"]=="UNSUPPORTED_GEOMETRIC_FACE_MAPPING"
        assert all(t.face_role!="FRONT" for t in items)
    else:
        assert items[0].face_role=="FRONT"
        np.testing.assert_allclose(items[0].outward_normal,[-1,0,0],atol=1e-12)
        if yaw:assert items[0].legacy_face in ("left","right")  # Never rename it front.


def test_center_short_x_long_y_and_frame_conversion():
    t=targets()[0][0]
    np.testing.assert_array_equal(t.face_center,[.7,0,1.])
    np.testing.assert_array_equal(t.in_plane_x,[0,0,1])
    from unloading_sim.independent_cups import build_m710_independent_cup_array
    array=build_m710_independent_cup_array()
    centers=np.array([c.center_contact_frame_m for c in array.cups])
    assert np.ptp(centers[:,0])==pytest.approx(.240)
    assert np.ptp(centers[:,1])==pytest.approx(.528)
    fv=np.eye(4);fv[:3,3]=[.25,0,0];fc=np.eye(4);fc[:3,3]=[.224,0,0]
    fv[:3,:3]=fc[:3,:3]=np.array([[0,0,1],[0,1,0],[-1,0,0]])
    virtual=virtual_tcp_from_physical_contact(t.contact_pose,fv,fc)
    assert not np.array_equal(virtual,t.contact_pose)
    np.testing.assert_allclose(physical_contact_from_virtual_tcp(virtual,fv,fc),t.contact_pose,atol=1e-12)


def test_exposure_and_support_not_assumed():
    neighbor=OBB([.3,0,1],[.1,.3,.15],np.eye(3),"front_blocker","carton")
    result,screen=targets(neighbours=[neighbor])
    assert screen[0]["reason"]=="FACE_OCCLUDED"
    assert result[0].face_role=="TOP"
    result,screen=targets(neighbours=[neighbor],edges=[SupportEdge("box","front_blocker",0,1)])
    assert not result and all(r["reason"]=="TARGET_SUPPORTS_REMAINING_CARTONS" for r in screen)


@pytest.mark.parametrize("change",["normal","distance","pose","identity","boundary"])
def test_corrupt_target_fails_closed(change):
    t=targets()[0][0]
    changes={"normal":{"outward_normal":np.array([1.,0,0])},
        "distance":{"standoff_m":float("nan")},"pose":{"contact_pose":np.zeros((4,4))},
        "identity":{"geometric_face_id":"y+"},"boundary":{"face_boundary":np.zeros((4,3))}}
    with pytest.raises(ValueError):replace(t,**changes[change]).validate()


def test_coordinator_first_complete_face_and_finite_order():
    ts=targets()[0]
    for winner in range(len(ts)):
        calls=[]
        def one(target,*args):
            calls.append(target.face_role)
            return ({"complete":True} if len(calls)==winner+1 else None),{"complete":len(calls)==winner+1}
        c=SimpleNamespace(robot=SimpleNamespace(within_limits=lambda q:True),plan_target=one)
        plan,rows=plan_faces(c,ts,np.zeros(6),np.zeros(6),"CONFIGURED",[],{},7)
        assert plan and calls==[t.face_role for t in ts[:winner+1]]
    with pytest.raises(ValueError,match="INVALID_FIXED_FACE_QUEUE"):
        plan_faces(c,list(reversed(ts)),np.zeros(6),np.zeros(6),"CONFIGURED",[],{},7)


@pytest.mark.parametrize("reason",["CANCELLED","SCENE_CONTEXT_CHANGED","INVALID_START_CLEARANCE","REQUEST_DEADLINE"])
def test_request_failure_stops_faces(reason):
    calls=[]
    def one(target,*args):
        calls.append(target)
        raise MoveItUnavailable(reason)
    c=SimpleNamespace(robot=SimpleNamespace(within_limits=lambda q:True),plan_target=one)
    plan,rows=plan_faces(c,targets()[0],np.zeros(6),np.zeros(6),"CONFIGURED",[],{},7)
    assert plan is None and len(calls)==1 and rows[0]["failure"]["scope"]=="REQUEST"


def test_nonzero_input_velocity_is_not_erased():
    c=SimpleNamespace(robot=SimpleNamespace(within_limits=lambda q:True))
    with pytest.raises(MoveItUnavailable,match="NONSTATIONARY"):
        plan_faces(c,targets()[0],np.zeros(6),np.full(6,1e-8),"MEASURED",[],{},7)


@pytest.mark.parametrize("bad",["middle","orientation","reverse","terminal"])
def test_only_returned_lin_samples_are_checked(bad):
    t=targets()[0][0]
    poses=[]
    for progress in (0,.05,.1):
        p=t.approach_pose.copy();p[:3,3]-=progress*t.outward_normal;poses.append(p)
    if bad=="middle":poses[1][1,3]+=.001
    if bad=="orientation":poses[1][:3,:3]=rotation_matrix_from_rpy(.01,0,0)@poses[1][:3,:3]
    if bad=="reverse":poses=[poses[0],poses[2],poses[1],poses[2]]
    if bad=="terminal":poses[-1][0,3]+=.001
    seen=[]
    def fk(i):seen.append(i);return poses[i]
    out=check_contact_samples(list(range(len(poses))),fk,t,position_tolerance=.0001,orientation_tolerance=.0002)
    assert not out["passed"] and out["additional_samples"]==0
    assert set(seen)==set(range(len(poses)))


def test_good_normal_samples_and_terminal_masks_are_separate():
    t=targets()[0][0]
    poses=[]
    for progress in (0,.05,.1):
        p=t.approach_pose.copy();p[:3,3]-=progress*t.outward_normal;poses.append(p)
    out=check_contact_samples([0,1,2],lambda i:poses[i],t,position_tolerance=.0001,orientation_tolerance=.0002)
    assert out["passed"] and out["samples"]==3


@pytest.mark.parametrize("op,planner",[
    ("load_static_prior",None),("build_static_prior",None),("prior_query",None),
    ("ik_batch",None),("plan","PTP"),("plan","CHECKED_JOINT_DIRECT")])
def test_protocol_fence_blocks_hidden_work(op,planner):
    calls=[]
    c=SingleContactClient(SimpleNamespace(request=lambda r,**kw:calls.append(r)))
    req=dict(op=op,planning_mode="cold_from_scratch",stage="pregrasp",
        pipeline_id="pilz_industrial_motion_planner",planner_id=planner)
    with pytest.raises(MoveItUnavailable,match="FORBIDDEN"):c.request(req)
    assert not calls and c.forbidden_attempts==1


def test_actual_native_dispatch_is_one_rrt_no_prior_no_ptp(monkeypatch):
    from test_approach_connection_tiers import connector,replies
    from unloading_sim.stage_motion_policy import MotionPurpose
    c=connector();c.planning_mode="cold_from_scratch";c.static_prior_loaded=False
    sent=replies(c,monkeypatch,lambda r:"SUCCESS")
    p,f,ev=c._native_plan(np.zeros(6),np.full(6,.1),[],stage="pregrasp",seed=7,
        purpose=MotionPurpose.FREE_APPROACH,allow_ompl=True,max_ompl_attempts=1,
        try_ptp=False,try_prior=False)
    assert f is None and len(sent)==1 and sent[0]["planner_id"]=="RRTConnectkConfigDefault"


def test_old_candidate_entries_are_explicitly_forbidden():
    c=SingleContactConnector.__new__(SingleContactConnector)
    for name in ("_approach","_connect_pose","_ik_stream","native_ik_stream","plan","load_static_prior"):
        with pytest.raises(MoveItUnavailable,match="OLD_SCHEDULER_FORBIDDEN"):getattr(c,name)()


@pytest.mark.parametrize("failure_stage",["ik","pregrasp","contact",None])
def test_single_target_dispatch_counts_and_real_terminal_start(failure_stage):
    """Exercise actual plan_target, with synthetic stage responses only."""
    target=targets()[0][0]
    c=SingleContactConnector.__new__(SingleContactConnector)
    calls=[]
    c._native_limits=lambda:(90.,100.)
    c._native_cancelled=lambda:False
    c.native=SimpleNamespace(request=lambda req,**kw:dict(status="SUCCESS"))
    c.check_context=lambda *a:None
    c._build_native_request=lambda *a,**k:dict(planning_mode="cold_from_scratch")
    c.native_root_state=lambda q,*a,**k:q
    c.robot=SimpleNamespace(fk=lambda q:np.eye(4))
    c.flange_from_virtual_task_tcp=c.flange_from_physical_contact=np.eye(4)
    c.ik=dict(position_tolerance_m=.0001,orientation_tolerance_rad=.0002)
    c.expected_contact=lambda *a:(SimpleNamespace(geometrically_eligible_mask=[True]*72,
        commanded_active_mask=[True]*72,commanded_active_ids=[]),None)
    def ik(*a):
        calls.append("ik")
        return (None,dict(reason="NATIVE_IK_FAILED")) if failure_stage=="ik" else (np.ones(6),None)
    c.ik_once=ik
    actual=np.ones(6);actual[0]=np.nextafter(1.,2.)
    def native(start,goal,obs,**kw):
        calls.append(kw["stage"])
        assert kw["try_ptp"] is False and kw["try_prior"] is False
        if kw["stage"]=="pregrasp":
            assert kw["allow_ompl"] is True and kw["recovery_mode"]=="shared_budget"
            assert "max_ompl_attempts" not in kw
        else:
            assert np.array_equal(start,actual) and not np.array_equal(start,np.ones(6))
            assert kw["allow_ompl"] is False and kw["max_ompl_attempts"]==0
        if failure_stage==kw["stage"]:return [],dict(reason="NATIVE_PLANNING_FAILED"),{}
        if kw["stage"]=="contact":
            # Explicit invalid terminal geometry must not return the pregrasp prefix.
            return [actual,np.ones(6)*2],None,{}
        return [start,actual],None,{}
    c._native_plan=native
    c.physical_contact_pose=lambda q:np.eye(4)  # fails returned-point normal contract
    plan,row=c.plan_target(target,np.zeros(6),[],{},7)
    assert plan is None and not row["complete"]
    assert calls==["ik"]+(["pregrasp"] if failure_stage!="ik" else [])+(
        ["contact"] if failure_stage not in ("ik","pregrasp") else [])
    if failure_stage is None:assert row["failure"]["reason"] in (
        "CONTACT_NORMAL_LINE_OR_ORIENTATION","CONTACT_ENDPOINT_FK")


@pytest.mark.parametrize("bad_terminal",[False,True])
def test_complete_two_segments_and_actual_terminal_geometry(monkeypatch,bad_terminal):
    target=targets()[0][0]
    c=SingleContactConnector.__new__(SingleContactConnector)
    c.robot_state_validator=SimpleNamespace()
    c.contact_tolerance_m=.0002
    c._native_limits=lambda:(90.,100.)
    c._native_cancelled=lambda:False
    c.native=SimpleNamespace(request=lambda req,**kw:dict(status="SUCCESS"))
    c.check_context=lambda *a:None
    c._build_native_request=lambda *a,**k:dict(planning_mode="cold_from_scratch")
    c.native_root_state=lambda q,*a,**k:q
    c.flange_from_virtual_task_tcp=c.flange_from_physical_contact=np.eye(4)
    c.ik=dict(position_tolerance_m=.0001,orientation_tolerance_rad=.0002)
    c.native_task_id="this-task"
    def fk(q):
        p=target.contact_pose.copy();p[:3,3]=np.asarray(q)[:3];return p
    c.robot=SimpleNamespace(fk=fk)
    approach=np.r_[target.approach_pose[:3,3],[0.]*3]
    contact=np.r_[target.contact_pose[:3,3],[0.]*3]
    actual=approach.copy();actual[0]=np.nextafter(actual[0],np.inf)
    class State(np.ndarray):
        @property
        def native_receipt(self):return dict(task_id="this-task",stage_id="contact")
    c.ik_once=lambda *a:(approach,None)
    calls=[]
    def plan(start,goal,obs,**kw):
        calls.append(kw)
        if kw["stage"]=="pregrasp":return [start,actual],None,{}
        np.testing.assert_array_equal(start,actual)
        return [actual,(actual+contact)/2,contact.view(State)],None,{}
    c._native_plan=plan
    c._native_selected_records=lambda end:[
        dict(stage="pregrasp",stage_id="approach",parent_stage_id="",generation_source="NATIVE_GENERATED"),
        dict(stage="contact",stage_id="contact",parent_stage_id="approach",generation_source="NATIVE_GENERATED")]
    if bad_terminal:
        from unloading_sim import single_contact
        audit=single_contact.audit_actual_independent_cup_contacts
        def bad(*a,**kw):
            original=audit(*a,**kw)
            return replace(original,actual_contact_mask=(False,)*72)
        monkeypatch.setattr(single_contact,"audit_actual_independent_cup_contacts",bad)
    plan,row=c.plan_target(target,approach.copy(),[],{},7)
    assert row["expected_contact"]["actual_contact_mask"] is None
    assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72
    assert row["contact_fk_check"]["passed"]
    assert len(calls)==2
    if bad_terminal:
        assert plan is None and row["failure"]["reason"]=="TERMINAL_CONTACT_GEOMETRY_NOT_SATISFIED"
    else:
        assert plan["status"]=="APPROACH_CONTACT_PLANNED_NOT_EXECUTABLE"
        assert row["complete"] and plan["events"]==[] and not plan["execution_ready"]
        assert plan["final_state_receipt"]["task_id"]=="this-task"
        assert all(row["terminal_contact"]["actual_contact_mask"])
        assert row["terminal_contact"]["attached"] is False
        assert plan["pregrasp"][-1]==plan["contact"][0]


def test_single_ik_uses_current_seed_and_fixed_approach():
    target=targets()[0][0];c=SingleContactConnector.__new__(SingleContactConnector)
    c.flange_from_virtual_task_tcp=c.flange_from_physical_contact=np.eye(4)
    c.ik=dict(position_tolerance_m=.0001,orientation_tolerance_rad=.0002)
    c.native_ik_seconds=.25;c.native_ik_evidence=[]
    c._native_limits=lambda:(90.,100.);c._native_cancelled=lambda:False
    c._build_native_request=lambda q,goal,obs,**kw:dict(q_start=q.tolist(),goal_pose=kw["goal_pose"].tolist())
    c.robot=SimpleNamespace(within_limits=lambda q:True,fk=lambda q:target.approach_pose)
    requests=[]
    c.native=SimpleNamespace(request=lambda r,**kw:(requests.append(r) or dict(status="SUCCESS",q=[.1]*6)))
    q,f=c.ik_once(target,np.arange(6)/10,[],7)
    assert f is None and len(requests)==1 and requests[0]["op"]=="ik"
    assert requests[0]["q_start"]==list(np.arange(6)/10) and "ik_seeds" not in requests[0]


def test_scene_revision_and_actual_geometry_change_stop():
    t=targets()[0][0];c=SingleContactConnector.__new__(SingleContactConnector)
    c._native_cancelled=lambda:False
    c.native_scene=SimpleNamespace(snapshot={"scene_fingerprint":"scene"})
    c._single_world_signature=digest([box_message(t.box)])
    c.check_context(t,[t.box])
    c.native_scene.snapshot["scene_fingerprint"]="next"
    with pytest.raises(MoveItUnavailable,match="SCENE_CONTEXT_CHANGED"):c.check_context(t,[t.box])
    c.native_scene.snapshot["scene_fingerprint"]="scene"
    moved=OBB(t.box.center+[0,.01,0],t.box.half_extents,t.box.rotation,t.box.name,t.box.category)
    with pytest.raises(MoveItUnavailable,match="SCENE_CONTEXT_CHANGED"):c.check_context(t,[moved])
