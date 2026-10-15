"""Approach scheduling regressions; synthetic protocol replies are not planning evidence."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
from unloading_sim.moveit2_backend import MoveItLayoutConnector
from unloading_sim.stage_motion_policy import MotionPurpose

def connector():
    c=MoveItLayoutConnector.__new__(MoveItLayoutConnector)
    c.planning_only=True;c.planning_mode="static_prior_fast";c.static_prior_loaded=True
    c.static_prior_placement_policy_sha256="policy";c.require_native_motion=False
    c.collision_policy=SimpleNamespace(poc_pair_clearance=True)
    c.budget=SimpleNamespace(stage_connection_attempts=2)
    c.robot=SimpleNamespace(joint_limits=np.array([[-3.,3.]]*6))
    c._state_failure=lambda *a,**k:None
    c._context_identity=lambda *a,**k:"scene"
    c._deadline_reached=lambda:False
    c._planning_phase=lambda:"SLOW_COMPLETION";c._fast_phase_overrun=lambda phase:0.
    c._build_native_request=lambda start,goal,obs,**kw:dict(q_start=list(start),q_goal=list(goal),
        allowed_planning_time_s=90., stage=kw["stage"],seed=kw["seed"],world=list(obs),
        task_id="task",parent_stage_id="",attachment=None)
    c.native_evidence=[];c.native_verified=[]
    c._accept_generated_stage=lambda path,attempt,attempts,*a,**k:(path,None,dict(attempts=attempts))
    return c

def test_direct_round_does_not_submit_full_graph(monkeypatch):
    c=connector();sent=[]
    monkeypatch.setattr("unloading_sim.static_prior.request_context",lambda *a,**k:{})
    c.native=SimpleNamespace(request=lambda req,**kw:(sent.append(deepcopy(req)) or
        dict(status="NATIVE_PLANNING_FAILED",pipeline_id=req["pipeline_id"],moveit_error_code=-2)))
    c._transit(np.zeros(6),np.ones(6)*.1,[],purpose=MotionPurpose.FREE_APPROACH,
        seed=1,iteration_budget=2,stage="pregrasp",allow_rrt=False)
    assert sent[0].get("connection_scope")=="DIRECT_ONLY"
    assert [r["planner_id"] for r in sent]==["CHECKED_JOINT_DIRECT","PTP"]

import pytest
from unloading_sim.moveit2_backend import JOINT_NAMES, native_selected_method, MoveItUnavailable

def replies(c, monkeypatch, decide):
    sent=[]
    monkeypatch.setattr("unloading_sim.static_prior.request_context",lambda *a,**k:{})
    c.native_joint_names=JOINT_NAMES
    def call(req,**kw):
        sent.append(deepcopy(req))
        status=decide(req)
        result=dict(status=status,pipeline_id=req["pipeline_id"],planner_id=req["planner_id"],
            stage_id="s"+str(len(sent)),request_id=str(len(sent)),
            native_solver_calls={"OMPL":1} if req["pipeline_id"]=="ompl" else
                {"PTP":1} if req["planner_id"]=="PTP" else {})
        if status=="NATIVE_PLANNING_FAILED":result["moveit_error_code"]=-2
        if status=="SUCCESS":
            result.update(joint_names=JOINT_NAMES,points=[
                dict(q=req["q_start"],t=0.,v=[0.]*6,a=[0.]*6),
                dict(q=req["q_goal"],t=1.,v=[0.]*6,a=[0.]*6)],
                generation_source="STATIC_PRIOR_REUSE" if req["planner_id"]=="SPARSE_PRM" else
                  "NATIVE_DIRECT_CONNECTION" if req["planner_id"]=="CHECKED_JOINT_DIRECT" else "NATIVE_GENERATED")
        return result
    c.native=SimpleNamespace(request=call)
    c._approach_direct_attempts={}
    return sent

def rejection(req):
    return "DIRECT_CONNECTION_REJECTED" if req["planner_id"]=="CHECKED_JOINT_DIRECT" else "NATIVE_PLANNING_FAILED"

def transit(c, *, scope="DIRECT_ONLY", goal=None, obstacles=(), **kwargs):
    return c._transit(np.zeros(6),np.ones(6)*.1 if goal is None else goal,list(obstacles),
        purpose=MotionPurpose.FREE_APPROACH,seed=1,iteration_budget=2,stage="pregrasp",
        allow_rrt=scope=="GLOBAL_ALLOWED",connection_scope=scope,**kwargs)

@pytest.mark.parametrize("winner",["CHECKED_JOINT_DIRECT","PTP"])
def test_both_direct_methods_are_retained(monkeypatch,winner):
    c=connector();sent=replies(c,monkeypatch,lambda r:"SUCCESS" if r["planner_id"]==winner else rejection(r))
    path,failure,ev=transit(c)
    assert failure is None and len(path)==2
    assert [r["planner_id"] for r in sent]==(["CHECKED_JOINT_DIRECT"] if winner=="CHECKED_JOINT_DIRECT" else ["CHECKED_JOINT_DIRECT","PTP"])
    assert ev["selected_method"]==("JOINT_DIRECT" if winner=="CHECKED_JOINT_DIRECT" else "PILZ_PTP")

def test_deferred_reuses_exact_direct_rejections_and_reports_prior_source(monkeypatch):
    c=connector();sent=replies(c,monkeypatch,lambda r:"SUCCESS" if r["planner_id"]=="SPARSE_PRM" else rejection(r))
    assert transit(c)[1]["reason"]=="DIRECT_CONNECTION_REJECTED"
    path,failure,ev=transit(c,scope="GLOBAL_ALLOWED")
    assert failure is None and ev["selected_method"]=="STATIC_PRIOR_REUSE"
    assert [r["planner_id"] for r in sent]==["CHECKED_JOINT_DIRECT","PTP","SPARSE_PRM"]
    assert sent[-1]["direct_rejection_stage_id"]=="s1"
    assert sent[-1]["allowed_planning_time_s"]==6.

@pytest.mark.parametrize("change",["world","goal","task","parent","target","policy","model","lease"])
def test_local_attempts_not_reused_after_context_change(monkeypatch,change):
    c=connector();sent=replies(c,monkeypatch,rejection)
    transit(c)
    if change=="lease":c._approach_direct_attempts={}
    elif change not in {"world","goal"}:
        build=c._build_native_request
        c._build_native_request=lambda *a,**kw:{**build(*a,**kw),change:"changed"}
    transit(c,obstacles=["new"] if change=="world" else (),
        goal=np.ones(6)*.2 if change=="goal" else None)
    assert [r["planner_id"] for r in sent]==["CHECKED_JOINT_DIRECT","PTP"]*2

@pytest.mark.parametrize("status",["DIRECT_CONNECTION_TIMEOUT","CANCELLED","ERROR","PRIOR_CONTEXT_MISMATCH"])
def test_incomplete_or_identity_error_is_not_cached_as_geometry(monkeypatch,status):
    c=connector();sent=replies(c,monkeypatch,lambda r:status)
    _,failure,_=transit(c)
    assert failure["reason"]==status and not c._approach_direct_attempts
    from unloading_sim.stage_motion_policy import interrupts_generation
    assert interrupts_generation(failure)==("TIMEOUT" not in status)
    assert len(sent)==1

def test_global_timeout_still_allows_original_ompl_fallback(monkeypatch):
    c=connector()
    def decide(r):
        if r["planner_id"]=="SPARSE_PRM":return "PRIOR_QUERY_TIMEOUT"
        if r["pipeline_id"]=="ompl":return "SUCCESS"
        return rejection(r)
    sent=replies(c,monkeypatch,decide)
    transit(c);path,failure,ev=transit(c,scope="GLOBAL_ALLOWED")
    assert failure is None and ev["native_ompl_calls"]==1 and ev["selected_method"]=="RRT_CONNECT"
    assert [r["planner_id"] for r in sent]==["CHECKED_JOINT_DIRECT","PTP","SPARSE_PRM","RRTConnectkConfigDefault"]
    assert ev["direct_attempts"][0]["status"]=="PRIOR_QUERY_TIMEOUT"

@pytest.mark.parametrize("case",["cold","transit","legacy"])
def test_outside_scope_preserves_original_schedule(monkeypatch,case):
    c=connector()
    if case=="cold":c.planning_mode="cold_from_scratch"
    if case=="legacy":c.approach_direct_first=False
    sent=replies(c,monkeypatch,lambda r:"SUCCESS")
    _,failure,_=c._transit(np.zeros(6),np.ones(6)*.1,[],
        purpose=MotionPurpose.FREE_APPROACH,seed=1,iteration_budget=2,
        stage="transit" if case=="transit" else "pregrasp",allow_rrt=False)
    assert failure is None
    assert sent[0]["planner_id"]==("PTP" if case=="cold" else "SPARSE_PRM")
    assert "connection_scope" not in sent[0]

def test_loaded_and_process_scope_cannot_be_enabled():
    c=connector()
    for kwargs in (dict(stage="transit",purpose=MotionPurpose.FREE_LOADED_TRANSFER,attachment=object()),
                   dict(stage="contact",purpose=MotionPurpose.CONTACT_PROCESS)):
        assert not c._approach_layering_enabled(**kwargs)
    with pytest.raises(MoveItUnavailable,match="SCOPE_INVALID"):
        c._native_plan(np.zeros(6),None,[],seed=1,stage="contact",
            purpose=MotionPurpose.CONTACT_PROCESS,goal_pose=np.eye(4),connection_scope="DIRECT_ONLY")

@pytest.mark.parametrize("second_direct", [True,False])
def test_actual_lazy_scheduler_all_endpoints_before_global(monkeypatch,second_direct):
    from test_stage_motion_policy import connector as core_connector, START, GOAL
    core=core_connector();native=connector()
    goals=[GOAL.copy(),GOAL.copy()];goals[1][3]=.2
    def decide(r):
        if r["planner_id"]=="SPARSE_PRM":return "SUCCESS"
        if second_direct and r["q_goal"][3]==.2 and r["planner_id"]=="CHECKED_JOINT_DIRECT":return "SUCCESS"
        return rejection(r)
    sent=replies(native,monkeypatch,decide)
    core._transit=native._transit;core._approach_layering_enabled=native._approach_layering_enabled
    class Stream:
        index=0
        def __next__(self):
            if self.index==len(goals):raise StopIteration
            q=goals[self.index];self.index+=1
            return SimpleNamespace(q=q,position_error=0.,orientation_error=0.,search_evidence={})
        def evidence(self):return dict(seeds_attempted=self.index)
    monkeypatch.setattr(core,"_ik_stream",lambda *a,**kw:Stream())
    _,path,failure,trace=core._connect_pose(core.robot.fk(GOAL),[START],START,[],
        purpose=MotionPurpose.FREE_APPROACH,stage="pregrasp",ik_seed=1,connection_seed=2)
    assert failure is None
    methods=[r["planner_id"] for r in sent]
    assert methods==(["CHECKED_JOINT_DIRECT","PTP","CHECKED_JOINT_DIRECT"] if second_direct else
        ["CHECKED_JOINT_DIRECT","PTP","CHECKED_JOINT_DIRECT","PTP","SPARSE_PRM"])
    assert trace["selected_method"]==("JOINT_DIRECT" if second_direct else "STATIC_PRIOR_REUSE")


def test_identical_direct_attempt_not_reexecuted_inside_lease(monkeypatch):
    c=connector();sent=replies(c,monkeypatch,rejection)
    transit(c);_,failure,ev=transit(c)
    assert len(sent)==2 and failure["reason"]=="DIRECT_CONNECTION_REJECTED"
    assert set(ev["reused_direct_attempts"])=={"PTP","CHECKED_JOINT_DIRECT"}

def test_attempt_lease_is_restored_even_on_exception(monkeypatch):
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
    c=connector();original={"outer":{}};c._approach_direct_attempts=original
    def inner(*args,**kw):
        assert c._approach_direct_attempts=={} and c._approach_direct_attempts is not original
        raise ValueError("stop")
    monkeypatch.setattr(LayoutTrajectoryConnector,"_connect_pose",inner)
    with pytest.raises(ValueError,match="stop"):c._connect_pose()
    assert c._approach_direct_attempts is original
