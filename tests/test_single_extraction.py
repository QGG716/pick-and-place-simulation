"""Focused extraction integration tests. Synthetic replies are not real-run evidence."""
from copy import deepcopy
from time import perf_counter
from types import SimpleNamespace
import numpy as np
import pytest

from test_single_contact import targets
from unloading_sim.single_contact import SingleContactClient, SingleContactConnector, plan_faces
from unloading_sim.single_extraction import SingleExtractionConnector, extraction_geometry, extraction_sample_check
from unloading_sim.geometry import OBB
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.moveit2_backend import MoveItUnavailable, box_message


def suffix_fixture():
    target = targets()[0][0]
    c = SingleExtractionConnector.__new__(SingleExtractionConnector)
    c._single_face_started = perf_counter()
    c.native_task_id = "task"
    c.check_context = lambda *a: None
    c.robot_state_validator = SimpleNamespace(commanded_cup_mask=(True,)*72)
    c.stack_carton_names = {"box","support"}
    c._native_contact_context = dict(target=target.box, selection=dict(actual_contact_mask=[False]*72))
    c.collision_policy = SimulationCollisionPolicy.from_mapping(dict(
        schema="m710_poc_pair_collision_policy_v4", required_pair_clearance_m=.005,
        boundary_mode="finite_frozen_scene_walls", free_space_clearance_m=.0052,
        stack_contact_mode="planner_relaxed_physics_checked"))
    c.collision_margin_m = .005
    c.contact_tolerance_m = .0002
    c.budget = SimpleNamespace(extraction_runtime_clearance_reserve_m=.003,
        extraction_scan_step_m=.005, maximum_extraction_m=.8)
    c.ik = dict(position_tolerance_m=.0001, orientation_tolerance_rad=.0002)
    c.flange_from_virtual_task_tcp = c.flange_from_physical_contact = np.eye(4)
    c.virtual_from_physical = lambda pose:pose.copy()
    c._native_limits = lambda:(90.,100.)
    c._native_cancelled = lambda:False
    class State(np.ndarray):
        @property
        def native_receipt(self):
            return dict(task_id="task",stage_id="extraction" if self[0]<.2 else "contact")
    q = np.r_[target.contact_pose[:3,3],np.zeros(3)]
    q[0] = np.nextafter(q[0],np.inf)
    q = q.view(State)
    def fk(q):
        t = target.contact_pose.copy();t[:3,3]=q[:3];return t
    c.physical_contact_pose = fk
    c.robot = SimpleNamespace(fk=fk, named_link_frames=lambda q:{"flange":fk(q)}, within_limits=lambda q:True)
    support=OBB([1,0,.7],[.3,.3,.15],np.eye(3),"support","carton")
    obstacles=[target.box,support]
    c.native_scene=SimpleNamespace(all_obstacles=obstacles,
        support_graph=SimpleNamespace(supported_by={"box":{"support"}}))
    c._native_state_receipt=lambda state:SimpleNamespace(stage_id="contact")
    plan=dict(contact=[q.tolist()],pregrasp=[q.tolist()],final_state_receipt=q.native_receipt,
        stages=[dict(stage_id="pregrasp"),dict(stage_id="contact")],events=[])
    terminal=dict(actual_contact_mask=[True]*72,commanded_active_mask=[True]*72,
        geometrically_eligible_mask=[True]*72,measured=False,attached=False)
    row=dict(complete=True,times={},actual_end_q_rad=q.tolist(),terminal_contact=terminal,
        expected_contact=dict(commanded_active_mask=[True]*72,actual_contact_mask=None),
        contact_fk_check=dict(passed=True))
    calls=[]
    def motion(start, goal, world, **kw):
        calls.append(kw)
        assert start is q
        assert kw["stage"]=="extraction" and kw["purpose"].value=="EXTRACTION_PROCESS"
        assert kw["allow_ompl"] is False and kw["max_ompl_attempts"]==0
        assert kw["try_ptp"] is kw["try_prior"] is False
        assert kw["support_names"]==["support"]
        assert "box" not in {b.name for b in world}
        assert c._native_contact_context["selection"]["actual_contact_mask"]==[True]*72
        dest=np.r_[kw["goal_pose"][:3,3],np.zeros(3)].view(State)
        # Simulate only the existing adapter's endpoint update, not an ordered audit.
        kw["initial_proximity"]._update_released(kw["attachment"].box_at(dest))
        return [start, (start+dest)/2, dest],None,{}
    c._native_plan=motion
    def build(start, goal, world, **kw):
        assert kw["support_names"]==() and kw["initial_proximity"] is None
        return dict(op="plan",q_start=start.tolist(),stage=kw["stage"])
    c._build_native_request=build
    c._native_parent_for_request=lambda state,req:(SimpleNamespace(stage_id="extraction",
        evidence=lambda:dict(stage_id="extraction")),{},None)
    c._native_selected_records=lambda state:[
        dict(stage="pregrasp",stage_id="pregrasp",parent_stage_id=""),
        dict(stage="contact",stage_id="contact",parent_stage_id="pregrasp"),
        dict(stage="extraction",stage_id="extraction",parent_stage_id="contact",
             submitted_request=dict(ownership_transition='ATTACH'))]
    checks=[]
    def endpoint(req,**kw):
        checks.append(req)
        pose=target.box.world_from_local.copy();pose[:3,3]+=np.asarray(req["q_start"])[:3]-q[:3]
        return dict(status="SUCCESS",task_id="task",parent_stage_id="extraction",target_id="box",
            target_attached=True,target_in_world=False,checked_states=1,free_space_rules_restored=True,
            target_world_pose=pose.tolist())
    c.native=SimpleNamespace(request=endpoint)
    return c,target,q,obstacles,plan,row,calls,checks


def test_actual_contact_attachment_unique_world_and_one_lin():
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    # Exact contact q differs by machine precision from the requested face center.
    assert q[0]!=t.contact_pose[0,3]
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result is not None and row["complete"] and len(calls)==len(checks)==1
    a=row["attachment"]
    assert a["contact_q_rad"]==q.tolist() and a["parent_receipt"]==q.native_receipt
    np.testing.assert_allclose(a["before_box_pose_world"],a["after_box_pose_world"],atol=1e-14,rtol=0)
    assert a["pose_continuity_max_abs"]<1e-14
    assert row["extraction_geometry"]["distance_m"]>.6  # full box clears support, not 0.1 m standoff
    assert row["extraction_geometry"]["constraint_ids"]==["support"]
    assert row["extraction_samples"]["passed"] and row["extraction_samples"]["additional_samples"]==0
    assert result["contact"][-1]==result["extraction"][0]
    assert result["events"][0]["scope"]=="PLANNED_SCENE_TRANSITION_ONLY"
    assert result["stages"][-1]["parent_stage_id"]=="contact"
    assert checks[0]["op"]=="check_extraction_end"
    assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72


@pytest.mark.parametrize("bad",["mask","command","eligible","fk","receipt","q"])
def test_attachment_rejects_unproven_or_changed_contact(bad):
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    if bad=="mask":row["terminal_contact"]["actual_contact_mask"]=[False]*72
    elif bad=="eligible":row["terminal_contact"]["geometrically_eligible_mask"]=[False]*72
    elif bad=="command":row["expected_contact"]["commanded_active_mask"]=[False]*72
    elif bad=="fk":row["contact_fk_check"]["passed"]=False
    elif bad=="receipt":plan["final_state_receipt"]={}
    else:plan["contact"][-1][0]+=.01
    with pytest.raises(MoveItUnavailable,match="ATTACH_"):
        c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert not calls and not checks and not row["complete"]


@pytest.mark.parametrize("bad",["length","lin","end","middle","orientation"])
def test_incomplete_suffix_never_returns_prefix(bad):
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    if bad=="length":c.budget.maximum_extraction_m=.1
    original=c._native_plan
    def changed(*args,**kw):
        if bad=="lin":
            calls.append(kw)
            return [],dict(reason="NATIVE_PLANNING_FAILED"),{}
        path,f,e=original(*args,**kw)
        if bad=="end":path[-1][0]=q[0]-.1
        if bad=="middle":path[1][0]=q[0]+.05
        if bad=="orientation":
            original_fk=c.physical_contact_pose
            def fk(q2):
                result=original_fk(q2)
                if np.array_equal(q2,path[1]):result[:3,:3]=np.eye(3)
                return result
            c.physical_contact_pose=fk
        return path,f,e
    c._native_plan=changed
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result is None and not row["complete"] and row["failure"]["scope"]=="FACE_ATTEMPT"
    assert len(calls)==(0 if bad=="length" else 1) and not checks
    assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72


def test_free_endpoint_rejection_not_extraction_complete():
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    c.native.request=lambda *a,**k:dict(status="EXTRACTION_END_CLEARANCE_REJECTED",
        failure=dict(pair=["robot","wall"]))
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result is None and row["failure"]["detail"]["pair"]==["robot","wall"]


def test_same_root_after_abandoned_attachment_and_tracker():
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    alltargets=targets()[0]
    roots=[];initial_release=[]
    def attempt(target,start,obstacles,suction,seed):
        roots.append(start.copy())
        assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72
        if len(roots)==1:
            freshplan,freshrow=deepcopy(plan),deepcopy(row)
            initial_release.append(c._initial_proximity(t.box,world[1:],["support"])[0].fully_released)
            c.native.request=lambda *a,**k:dict(status="EXTRACTION_END_CLEARANCE_REJECTED")
            return c.complete_contact_plan(freshplan,freshrow,q,t,world,{},seed)
        initial_release.append(c._initial_proximity(t.box,world[1:],["support"])[0].fully_released)
        return {"complete":True},dict(complete=True)
    c.plan_target=attempt
    result,rows=plan_faces(c,alltargets,np.zeros(6),np.zeros(6),"configured",world,{},71070)
    assert result and len(rows)==2 and initial_release==[False,False]
    assert all(np.array_equal(root,np.zeros(6)) for root in roots)


def extraction_request(op="plan"):
    return dict(op=op,planning_mode="cold_from_scratch",stage="extraction",
        pipeline_id="pilz_industrial_motion_planner",planner_id="LIN",purpose="EXTRACTION_PROCESS",
        attachment=dict(id="box"),world=[dict(id="support")],parent_stage_id="contact",
        ownership_transition="ATTACH",process_policy=dict(target_id="box",attached=True,
            actual_contact_mask=[True]*72,commanded_active_mask=[True]*72,
            initial_proximity=dict(),support_names=["support"]))


@pytest.mark.parametrize("bad",["default","duplicate","wrong_target","mask","ompl","ptp","contact_attached","prior"])
def test_protocol_extension_remains_stage_scoped(bad):
    client=SingleContactClient(SimpleNamespace(request=lambda *a,**k:pytest.fail("must not dispatch")),
        extraction_enabled=bad!="default")
    r=extraction_request()
    if bad=="duplicate":r["world"].append(dict(id="box"))
    elif bad=="wrong_target":r["attachment"]["id"]="other"
    elif bad=="mask":r["process_policy"]["actual_contact_mask"]=[False]*72
    elif bad=="ompl":r.update(pipeline_id="ompl",planner_id="RRTConnectkConfigDefault")
    elif bad=="ptp":r["planner_id"]="PTP"
    elif bad=="contact_attached":r["stage"]="contact"
    elif bad=="prior":r["op"]="prior_query"
    with pytest.raises(MoveItUnavailable):client.request(r)


def test_endpoint_has_no_support_or_relaxed_stack_permission():
    client=SingleContactClient(SimpleNamespace(request=lambda r,**k:dict(status="SUCCESS")),
        extraction_enabled=True)
    r=extraction_request("check_extraction_end")
    with pytest.raises(MoveItUnavailable):client.request(r)
    r["process_policy"].update(support_names=[],initial_proximity=None)
    r["ownership_transition"]=None
    assert client.request(r)["status"]=="SUCCESS"
    assert client.records[-1]["target_in_world"] is False
    assert client.records[-1]["response"].get("native_solver_calls",{})=={}


def test_sampled_history_reentry_stays_rejected():
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    attachment=PhysicalContactAttachment(c.robot,RigidAttachment.capture(c.physical_contact_pose(q),t.box),
        np.eye(4),np.eye(4))
    tracker,_=c._initial_proximity(t.box,world[1:],["support"])
    goal,geometry,f=extraction_geometry(c,t,attachment,q,world[1:],["support"])
    end=q.copy();end[:3]=goal[:3,3]
    result=extraction_sample_check(c,[q,end,q],attachment,t,geometry,tracker,world[1:],["support"])
    assert not result["passed"] and not tracker.state_failure(attachment.box_at(end),world[1:],[])
    rejected=tracker.state_failure(attachment.box_at(q),world[1:],[])
    assert rejected["contact_state"]=="FREE_SPACE_RULES_RESTORED"


def test_endpoint_success_without_bound_state_evidence_is_rejected():
    c,t,q,world,plan,row,calls,checks=suffix_fixture()
    c.native.request=lambda *a,**k:dict(status="SUCCESS")
    with pytest.raises(MoveItUnavailable,match="EXTRACTION_END_EVIDENCE_IDENTITY_CHANGED"):
        c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert not row["complete"]
    assert c._native_contact_context["selection"]["actual_contact_mask"]==[False]*72
