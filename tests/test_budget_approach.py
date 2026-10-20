from types import SimpleNamespace as NS
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.approach_recovery_policy import native_schedule,equivalent_bounded_configurations
from unloading_sim.moveit2_backend import MoveItLayoutConnector,MoveItUnavailable,JOINT_NAMES
from unloading_sim.single_contact import SingleContactClient
from test_single_target_recovery import fixture,run


def test_schedule_has_time_limit_no_attempt_limit():
    clock=[0.]
    methods=[]
    for method in native_schedule([],budget_driven=True,legacy_attempts=1,deadline=1.,
                                  now=lambda:clock[0],allow_ompl=True,constrained=False):
        methods.append(method);clock[0]+=.03
    assert len(methods)>30
    assert len(list(native_schedule([],budget_driven=False,legacy_attempts=2,deadline=None,
        now=lambda:0.,allow_ompl=True,constrained=False)))==2


def backend(monkeypatch,success_at=8,direct=False):
    import unloading_sim.moveit2_backend as module
    clock=[0.];monkeypatch.setattr(module,"perf_counter",lambda:clock[0])
    c=MoveItLayoutConnector.__new__(MoveItLayoutConnector);calls=[]
    c._native_limits=lambda:(1.,2.);c.budget=NS(stage_connection_attempts=1)
    c._state_failure=lambda *a,**kw:None;c._path_failure=lambda *a,**kw:None
    c.collision_policy=NS(poc_pair_clearance=True)
    c._context_identity=lambda *a,**kw:"unchanged";c._deadline_reached=lambda:False
    c.native_evidence=[];c.native_verified=[];c.native_joint_names=JOINT_NAMES
    c._build_native_request=lambda *a,**kw:dict(allowed_planning_time_s=1.,q_start=[0.]*6,q_goal=[.1]*6,
        stage="pregrasp",parent_stage_id="same-parent")
    def request(r,**kw):
        calls.append(deepcopy(r));clock[0]+=.02
        if len(calls)<success_at:return dict(status="NATIVE_PLANNING_FAILED",native_solver_calls=1)
        return dict(status="SUCCESS",joint_names=JOINT_NAMES,
            points=[dict(q=[0.]*6,v=[0.]*6,a=[0.]*6,t=0.),dict(q=[.1]*6,v=[0.]*6,a=[0.]*6,t=1.)])
    c.native=NS(request=request)
    return c,calls,clock


def test_full_adapter_shared_budget_does_not_inherit_default_one(monkeypatch):
    c,calls,_=backend(monkeypatch)
    path,f,ev=c._native_plan(np.zeros(6),np.full(6,.1),[],seed=1,stage="pregrasp",
        purpose="FREE_APPROACH",recovery_mode="shared_budget",try_ptp=False,try_prior=False)
    assert path and f is None and len(calls)==8
    assert all(r["search_operation"]=="RESTART" and r["parent_stage_id"]=="same-parent" for r in calls)
    with pytest.raises(MoveItUnavailable,match="QUOTA"):
        c._native_plan(np.zeros(6),np.full(6,.1),[],seed=1,stage="pregrasp",
            recovery_mode="shared_budget",max_ompl_attempts=1,try_prior=False)


def test_direct_failure_leaves_rrt_and_same_root(monkeypatch):
    c,calls,_=backend(monkeypatch,success_at=3)
    c.robot=NS(fk=lambda q:np.eye(4));c.capability_check=lambda *a,**k:None
    path,f,ev=c._native_plan(np.zeros(6),np.full(6,.1),[],seed=1,stage="pregrasp",
        purpose="FREE_APPROACH",recovery_mode="shared_budget",try_ptp=True,try_prior=False,try_lin_pose=np.eye(4))
    assert path and f is None
    assert [r["planner_id"] for r in calls]==["PTP","LIN","RRTConnectkConfigDefault"]
    assert all(r["q_start"]==[0.]*6 and r["parent_stage_id"]=="same-parent" for r in calls)
    assert sum(r["allowed_planning_time_s"] for r in calls[:2])<=.2


def test_path_timeout_retains_legal_q_for_later_attempt(monkeypatch):
    c,clock,_,iks,motions=fixture(monkeypatch)
    q=np.full(6,.1);calls=[]
    def ik(*a,**kw):
        clock[0]+=.01
        return (q.copy(),None) if not calls else (None,dict(reason="NATIVE_IK_FAILED",native=dict(bounded_search=True)))
    def plan(*a,**kw):
        clock[0]+=.03;calls.append(a[1].copy())
        assert kw["recovery_mode"]=="shared_budget" and "max_ompl_attempts" not in kw
        if len(calls)<9:return [],dict(reason="MOVEIT2_SEARCH_EXHAUSTED"),{}
        return [a[0],a[1]],None,{}
    c.ik_pose_once=ik;c._native_plan=plan
    path,_,f,_=run(c)
    assert path and f is None and len(calls)==9
    assert all(np.array_equal(x,q) for x in calls)


@pytest.mark.parametrize("method",["PTP","LIN"])
def test_direct_client_scope_and_parent_preserved(method):
    calls=[]
    c=SingleContactClient(NS(request=lambda r,**k:(calls.append(r) or dict(status="SUCCESS"))))
    request=dict(op="plan",planning_mode="cold_from_scratch",planning_only=True,
        stage="pregrasp",purpose="FREE_APPROACH",attachment=None,pipeline_id="pilz_industrial_motion_planner",
        planner_id=method,empty_approach_direct=True,recovery_mode="shared_budget",parent_stage_id="checked-parent",
        start_velocity=[0.]*6,
        world=[dict(id="target")],process_policy=dict(target_id="target",attached=False))
    c.request(request);assert calls[0]["parent_stage_id"]=="checked-parent"
    for change in [dict(stage="transit"),dict(purpose="EXTRACTION_PROCESS"),
                   dict(attachment=dict(id="target")),dict(empty_approach_direct=False),
                   dict(planning_only=False),dict(start_velocity=[.1]*6)]:
        with pytest.raises(MoveItUnavailable):c.request(dict(request,**change))


def test_equivalent_angles_require_all_link_fk_and_bounds():
    robot=NS(joint_limits=np.array([[-7.,7.]]*6),within_limits=lambda q:np.all(np.abs(q)<=7.))
    def fk(q):
        t=np.eye(4);t[:3,:3]=[[np.cos(q[0]),-np.sin(q[0]),0],[np.sin(q[0]),np.cos(q[0]),0],[0,0,1]]
        other=np.eye(4);other[:3,3]=q[1:4]  # prevents merging different intermediate links
        return dict(tcp=t,intermediate=other)
    robot.named_link_frames=fk
    original=np.array([6.,1.,1.,1.,0.,0.])
    variants=equivalent_bounded_configurations(robot,original)
    assert any(np.array_equal(v,original) for v in variants)
    assert any(abs(v[0]-(6-2*np.pi))<1e-12 for v in variants)
    assert all(np.array_equal(v[1:4],original[1:4]) for v in variants)


def test_quality_failure_keeps_native_incumbent(monkeypatch):
    import unloading_sim.single_target_recovery as module
    c,clock,_,_,_=fixture(monkeypatch)
    c._native_limits=lambda:(20.,30.)
    c.empty_approach_direct_first=True;c.robot=NS()
    monkeypatch.setattr(module,"equivalent_bounded_configurations",lambda robot,q:[q,q+1])
    monkeypatch.setattr(module,"path_metrics",lambda *a,**kw:dict(weighted_joint_length_rad=10.,
        endpoint_joint_distance_rad=1.,physical_contact_length_over_chord=2.))
    calls=[];incumbent=[np.zeros(6),np.full(6,.01)]
    def plan(*a,**kw):
        calls.append(kw);clock[0]+=.1
        if len(calls)==1:
            return incumbent,None,dict(attempts=[dict(status="SUCCESS",points=[1],
                planner_id="RRTConnectkConfigDefault",stage_id="original-native-stage")])
        return [],dict(reason="DIRECT_CONNECTION_REJECTED"),{}
    c._native_plan=plan
    path,_,failure,_=module.connect_fixed_target(c,np.eye(4),np.zeros(6),[],1,
        stage="pregrasp",purpose="FREE_APPROACH",residual_reason="FK")
    assert path is incumbent and failure is None and len(calls)>1
    assert all(x["allow_ompl"] and not x["try_ptp"] for x in calls[1:])
    assert c.connection_attempts[0]["incumbent_retained"]
    assert c.connection_attempts[0]["accepted_quality"]["stage_id"]=="original-native-stage"


def test_short_same_pose_ik_comparison_precedes_motion_without_exhausting_stage(monkeypatch):
    import unloading_sim.single_target_recovery as module
    c,clock,_,iks,motions=fixture(monkeypatch)
    c._native_limits=lambda:(90.,100.);c.empty_approach_direct_first=True
    def ik(*a,**kw):
        clock[0]+=.11;iks.append(kw)
        if len(iks)==1:return np.ones(6),None
        if len(iks)==2:return np.full(6,.1),None
        return None,dict(reason="NATIVE_IK_FAILED",native=dict(bounded_search=True))
    c.ik_pose_once=ik
    path,q,f,cost=module.connect_fixed_target(c,np.eye(4),np.zeros(6),[],1,
        stage="pregrasp",purpose="FREE_APPROACH",residual_reason="FK")
    assert f is None and len(motions)==1 and len(iks)==3
    assert np.array_equal(q,np.full(6,.1))
    assert np.array_equal(motions[0]["start"],np.zeros(6))
    assert cost["ik_s"]==pytest.approx(.33) and clock[0]<1.


def test_lin_different_expensive_branch_compares_native_rrt_to_original_goal(monkeypatch):
    import unloading_sim.single_target_recovery as module
    c,clock,_,_,_=fixture(monkeypatch)
    c._native_limits=lambda:(90.,100.);c.empty_approach_direct_first=True;c.robot=NS()
    monkeypatch.setattr(module,"equivalent_bounded_configurations",lambda robot,q:[q])
    ik_count=[0]
    def ik(*a,**kw):
        ik_count[0]+=1;clock[0]+=.1
        if ik_count[0]==1:return np.full(6,.1),None
        return None,dict(reason="NATIVE_IK_FAILED",native=dict(bounded_search=True))
    c.ik_pose_once=ik;calls=[]
    paths=[[np.zeros(6),np.ones(6)],[np.zeros(6),np.full(6,.1)]]
    def plan(start,q,*a,**kw):
        calls.append((q.copy(),kw));clock[0]+=.2
        index=len(calls)-1
        return paths[index],None,dict(attempts=[dict(status="SUCCESS",points=[dict(q=paths[index][-1])],
            planner_id="LIN" if index==0 else "RRTConnectkConfigDefault",stage_id=f"native-{index}")])
    c._native_plan=plan
    monkeypatch.setattr(module,"path_metrics",lambda robot,p,*a:
        dict(weighted_joint_length_rad=float(sum(p[0]["q"])),
             endpoint_joint_distance_rad=float(sum(p[0]["q"])),physical_contact_length_over_chord=1.))
    path,q,f,cost=module.connect_fixed_target(c,np.eye(4),np.zeros(6),[],1,
        stage="pregrasp",purpose="FREE_APPROACH",residual_reason="FK")
    assert f is None and path is paths[1] and len(calls)==2
    assert np.array_equal(calls[0][0],calls[1][0])
    assert calls[1][1]["allow_ompl"] and not calls[1][1]["try_ptp"] and calls[1][1]["try_lin_pose"] is None
    assert c.connection_attempts[0]["accepted_quality"]["stage_id"]=="native-1"


def test_failed_checkpoint_never_mutates_completed_state(monkeypatch,tmp_path):
    import importlib.util
    from pathlib import Path
    spec=importlib.util.spec_from_file_location("batch_checkpoint_test",
        Path(__file__).resolve().parents[1]/"tools/plan_m710_top_layer_lifted.py")
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    class State:
        def __init__(self):self.ledger=[]
        def commit(self,plan):self.ledger.append(plan["target"])
        def root(self):return dict(completed=self.ledger)
    state=State();folder=tmp_path/"attempt";folder.mkdir()
    (folder/"attempt.json").write_text("{}")
    client=NS(records=[],trace_inputs={},traces=[])
    actual_save=module.save
    def fail(path,data):
        if path.name=="requests.json":raise OSError("injected disk full")
        actual_save(path,data)
    monkeypatch.setattr(module,"save",fail)
    with pytest.raises(OSError):
        module.checkpoint_candidate(state,dict(target="box"),client,folder,tmp_path,{},[])
    assert not state.ledger and not (tmp_path/"checkpoint.json").exists()
    monkeypatch.setattr(module,"save",actual_save)
    committed=module.checkpoint_candidate(state,dict(target="box"),client,folder,tmp_path,{},[])
    assert not state.ledger and committed.ledger==["box"]
    assert (tmp_path/"checkpoint.json").exists()
