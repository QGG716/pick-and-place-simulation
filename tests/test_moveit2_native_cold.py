"""Pure contract regressions. Fabricated records are never native run evidence."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.moveit2_backend import MoveItLayoutConnector, MoveItUnavailable, NativeStageState, digest
from unloading_sim.moveit2_native_cold import audit_native_motion_coverage, verify_native_cold_segment
from unloading_sim.stage_motion_policy import GenerationMethod, MotionPurpose


def record(path, stage_id="stage-1", request_id="request-1"):
    request=dict(task_id="task-now",stage_id=stage_id,q_start=path[0],q_goal=path[-1])
    return dict(stage_id=stage_id, request_id=request_id, task_id="task-now", solver="Pilz/LIN",
        parent_stage_id="",native_solver_calls=dict(LIN=1),submitted_request=request,
        request_fingerprint=digest(request),input_state_sha256=digest(path[0]),
        constraints_sha256=digest({k:v for k,v in request.items() if k!="q_start"}),
        path_sha256=digest(path), authoritative_status="PASS", mtc_generation=True,
        points=[dict(q=q, t=float(i)) for i,q in enumerate(path)])


def segment():
    path=[[0.]*6,[.1]*6,[.2]*6]
    return dict(path=path,native_cold=True,require_native_motion=True,native_backend=dict(
        native_cold=True,require_native_motion=True,task_id="task-now",stages=[record(path)],
        cold_audit=dict(history_enabled=False,history_inputs_read=0,legacy_motion_generator_calls=0),
        mtc_task_audit=dict(status="SUCCESS",task_id="task-now",generated_during_task=True,
            complete_task=True,stage_ids=["stage-1"],stage_sources=[dict(stage_id="stage-1",
                parent_stage_id="",request_id="request-1",native_solver_calls=dict(LIN=1))])))


def test_all_nonzero_edges_require_actual_native_points():
    value=segment()
    assert verify_native_cold_segment(value) is None
    path=value["path"]
    partial=record(path[:2])
    checked=audit_native_motion_coverage(path,[partial],task_id="task-now")
    assert checked["uncovered_edges"]==[1]
    assert checked["covered_nonzero_edge_count"]==1
    assert checked["coverage_fraction"]==.5
    value["native_backend"]["stages"]=[partial]
    assert verify_native_cold_segment(value)["reason"]=="NATIVE_COLD_MOTION_SOURCE_GAP"


@pytest.mark.parametrize("mutate,reason",[
    (lambda s:s["native_backend"].pop("stages"),"MOTION_SOURCE_GAP"),
    (lambda s:s.pop("native_backend"),"BACKEND_CONTRACT_MISSING"),
    (lambda s:s.pop("native_cold"),"REQUEST_MARKER_MISSING"),
    (lambda s:s["native_backend"]["cold_audit"].update(history_inputs_read=1),"FORBIDDEN_SOURCE"),
    (lambda s:s["native_backend"]["cold_audit"].update(legacy_motion_generator_calls=False),"FORBIDDEN_SOURCE"),
    (lambda s:s["native_backend"]["stages"][0].update(task_id="previous-run"),"MOTION_SOURCE_GAP"),
    (lambda s:s["native_backend"]["stages"][0].update(mtc_generation=False),"MOTION_SOURCE_GAP"),
    (lambda s:s["native_backend"]["mtc_task_audit"].update(generated_during_task=False),"COMPLETE_MTC_TASK_NOT_PROVEN"),
])
def test_source_or_task_claim_cannot_replace_run_objects(mutate,reason):
    value=segment();mutate(value)
    assert reason in verify_native_cold_segment(value)["reason"]


def test_no_unplanned_stitch_edge_and_no_changed_native_range():
    path=[[0.]*6,[.1]*6,[.2]*6,[.3]*6]
    checked=audit_native_motion_coverage(path,[record(path[:2]),record(path[2:],"stage-2","request-2")],task_id="task-now")
    assert checked["uncovered_edges"]==[1]
    source=record(path);source["path_range"]=[1,4]
    assert audit_native_motion_coverage(path,[source])["rejected_records"][0]["reason"]=="NATIVE_STAGE_RANGE_MISMATCH"


def test_zero_edges_are_separate_and_not_synthetic_native_calls():
    checked=audit_native_motion_coverage([[0.]*6,[0.]*6],[],task_id="task-now")
    assert checked["passed"] and checked["nonzero_edge_count"]==0
    assert checked["zero_length_edge_indices"]==[0] and checked["records"]==[]


def connector():
    c=MoveItLayoutConnector.__new__(MoveItLayoutConnector)
    c.native_cold=c.require_native_motion=True
    c._cold_counters=dict(history_enabled=False,history_inputs_read=0,legacy_motion_generator_calls=0,forbidden_entry_attempts=0)
    c.budget=SimpleNamespace(stage_connection_attempts=3)
    return c


def receipt_connector():
    """Small independent identity model; never native execution evidence."""
    c=connector();c.native_task_id="task-now";c.native_identity={"model":"test-model"}
    c.native_scene=SimpleNamespace(snapshot={"robot":{"q_rad":[0.]*6}},all_obstacles=[])
    c._native_receipts={};c._native_zero_state_requests={}
    c._native_root_receipt=None;c._native_final_selection=None
    c.native_verified=[];c.native_semantic_events=[]
    target=dict(id="target",size=[.4,.3,.2],category="carton",pose=np.eye(4).tolist())
    def build(start,goal,obstacles,*,stage,attachment=None,**kwargs):
        owned_target=deepcopy(target)
        if attachment is not None: owned_target["category"]="payload"
        return dict(task_id=c.native_task_id,identity=c.native_identity,stage=stage,
            q_start=np.asarray(start).tolist(),q_goal=np.asarray(goal).tolist(),
            world=deepcopy(obstacles),attachment=None if attachment is None else {"id":"target"},
            process_policy=dict(target_id="target",target=owned_target))
    c._build_native_request=build
    c._path_failure=lambda *a,**k:None
    c.robot=SimpleNamespace(fk=lambda q:np.eye(4))
    return c


def accepted_state(c,start,end,stage_id,*,stage="pregrasp",attachment=None):
    """Emulate only post-authority receipt issuance with numeric test doubles."""
    request=c._build_native_request(start,end,[],stage=stage,attachment=attachment)
    parent,context,_=c._native_parent_for_request(start,request)
    path=[np.asarray(start).tolist(),np.asarray(end).tolist()]
    source=record(path,stage_id,request_id="request-"+stage_id)
    request.update(stage_id=stage_id,parent_stage_id=parent.stage_id)
    source.update(parent_stage_id=parent.stage_id,submitted_request=request,
        request_fingerprint=digest(request),constraints_sha256=digest({k:v for k,v in request.items() if k!="q_start"}))
    c.native_verified.append(source)
    return c._native_issue_state(end,stage_id,context)


@pytest.mark.parametrize("name,args",[
    ("_rrt_transit",()),("_cartesian_process",()),
    ("history_linear_suffix_requests",(None,None,None,None)),
    ("history_capability_check",(None,None,None,None)),
])
def test_legacy_generation_entries_fail_before_work(name,args):
    c=connector()
    with pytest.raises(MoveItUnavailable,match="NATIVE_COLD_FORBIDDEN_ENTRY"):
        getattr(c,name)(*args)
    assert c._cold_counters["legacy_motion_generator_calls"]==0
    assert c._cold_counters["forbidden_entry_attempts"]==1


def test_transit_defers_ompl_and_does_not_consume_candidates_when_disallowed():
    c=connector();calls=[]
    c._native_plan=lambda *a,**k:(calls.append(k) or [],dict(reason="DIRECT_CONNECTION_REJECTED"),{})
    def sentinel():
        raise AssertionError("deferred candidate generator must not be entered")
    _,failure,evidence=c._transit(np.zeros(6),np.ones(6),[],purpose=MotionPurpose.FREE_APPROACH,
        seed=1,iteration_budget=9,stage="pregrasp",allow_rrt=False,
        candidates=[(GenerationMethod.HISTORY_HINT,sentinel)])
    assert failure["reason"]=="DIRECT_CONNECTION_REJECTED"
    assert len(calls)==1 and calls[0]["allow_ompl"] is False
    assert evidence["native_ompl_allowed"] is False


def test_transit_history_is_rejected_before_callback_and_before_ompl():
    c=connector();calls=[]
    c._native_plan=lambda *a,**k:(calls.append(k) or [],dict(reason="DIRECT_CONNECTION_REJECTED"),{})
    def sentinel():
        raise AssertionError("history path must never be read")
    with pytest.raises(MoveItUnavailable,match="transit_history_candidate"):
        c._transit(np.zeros(6),np.ones(6),[],purpose=MotionPurpose.FREE_APPROACH,
            seed=1,iteration_budget=9,stage="pregrasp",candidates=[(GenerationMethod.HISTORY_HINT,sentinel)])
    assert len(calls)==1


def test_native_ptp_finite_candidates_then_ompl_schedule_and_budget():
    c=connector();order=[]
    def native(*a,**k):
        order.append("PTP" if k.get("try_ptp",True) else "OMPL")
        return [],dict(reason="DIRECT_CONNECTION_REJECTED"),dict(attempts=[{"status":"FAILED"}])
    def local():
        order.append("local")
        return [],dict(reason="NO_NATIVE_LIN_CANDIDATE"),{}
    c._native_plan=native
    _,_,evidence=c._transit(np.zeros(6),np.ones(6),[],purpose=MotionPurpose.FREE_APPROACH,
        seed=1,iteration_budget=1,stage="pregrasp",candidates=[(GenerationMethod.LOCAL_CARTESIAN_CANDIDATE,local)])
    assert order==["PTP","local","OMPL"]
    assert evidence["planning_iterations_consumed"]==1
    assert evidence["rrt_called"] is False


def test_transit_rejects_process_intent_and_payload_mismatch():
    c=connector()
    for purpose in (MotionPurpose.CONTACT_PROCESS,MotionPurpose.FREE_LOADED_TRANSFER):
        with pytest.raises(ValueError):
            c._transit(np.zeros(6),np.ones(6),[],purpose=purpose,seed=1,iteration_budget=3,stage="transit")


def test_strict_process_cannot_fall_back_when_worker_lacks_semantics():
    c=connector()
    c.native_startup=dict(capabilities=dict(native_task_session=True))
    c.collision_policy=SimpleNamespace(poc_pair_clearance=True)
    _,failure,_=c._native_plan(np.zeros(6),None,[],seed=1,stage="contact",goal_pose=np.eye(4))
    assert failure["reason"]=="NATIVE_PROCESS_SEMANTICS_UNAVAILABLE"


def test_native_ik_stream_is_lazy_and_preserves_outer_statistics(monkeypatch):
    import unloading_sim.layout_trajectory as core
    c=connector();calls=[]
    def forbidden(*a,**k):
        raise AssertionError("legacy IK generator called")
    monkeypatch.setattr(core,"iter_ik_solutions",forbidden)
    c.native_startup=dict(capabilities=dict(native_ik=True))
    c.native_task_id="task-now";c.native_ik_evidence=[];c.native_ik_seconds=.25
    c.budget=SimpleNamespace(stage_ik_candidates=2)
    c.ik=dict(random_restarts=0,position_tolerance_m=1e-6,orientation_tolerance_rad=1e-6,
        candidate_dedup_tolerance_rad=.001)
    def fk(q):
        pose=np.eye(4);pose[0,3]=q[0];return pose
    c.robot=SimpleNamespace(joint_limits=np.array([[-3.,3.]]*6),within_limits=lambda _:True,fk=fk)
    c._native_cancelled=lambda:False;c._native_limits=lambda:(1.,2.)
    c._build_native_request=lambda q,goal,obstacles,**kw:dict(q_start=q.tolist(),goal_pose=kw["goal_pose"].tolist())
    c._state_failure=lambda *a,**kw:None
    def request(payload,**kwargs):
        calls.append(payload)
        return dict(status="SUCCESS",q=[.1,0.,0.,0.,0.,0.],native_ik_calls=1,
            request_id="native-1",solver="MoveIt_KDL_getPositionIK")
    c.native=SimpleNamespace(request=request)
    goal=fk([.1]*6)
    stream=c.native_ik_stream(goal,[np.zeros(6)],[],seed=7,stage="pregrasp")
    assert calls==[]
    result=next(stream)
    assert result.success and result.search_evidence["candidate_id"].startswith("native-ik-")
    assert calls[0]["op"]=="ik"
    evidence=stream.evidence()
    for key in ("seed_pool_available","seeds_attempted","converged_pose_results","valid_solutions","deduplicated_candidates"):
        assert evidence[key]==1
    assert evidence["iterations_consumed"]==0 and evidence["iterations_observable"] is False
    assert c.native_ik_evidence[0]["task_id"]=="task-now"


def rest_snapshot():
    import hashlib
    from pathlib import Path
    from unloading_sim import m710_replay_physics
    gate=m710_replay_physics.RestStartGate()
    parameters={key:getattr(gate,key) for key in ("maximum_wait_s","position_tolerance_rad",
        "velocity_tolerance_rad_s","position_derived_speed_tolerance_rad_s","required_stable_s")}
    samples=[];q=[0.]*6;qd=[.012]*6
    for time in (0.,.05,.10,.15,.20):
        result=gate.evaluate(time,q,qd,q)
        samples.append(dict(time_s=time,q_rad=q.copy(),qd_rad_s=qd.copy(),reference_q_rad=q.copy(),result=result))
        if gate.passed:break
    assert gate.passed
    evidence=dict(schema="m710_native_rest_start_evidence_v1",gate="RestStartGate",passed=True,
        world_session_id="world-new",parameters=parameters,samples=samples,
        implementation_sha256=hashlib.sha256(Path(m710_replay_physics.__file__).read_bytes()).hexdigest())
    return dict(robot=dict(q_rad=q,qd_rad_s=qd,native_rest_start_evidence=evidence),
        actual_state_context=dict(world_session_id="world-new"))


def test_measured_rest_gate_retains_nonzero_raw_velocity_with_explicit_conversion():
    from unloading_sim.moveit2_backend import native_rest_start_contract
    snapshot=rest_snapshot()
    result=native_rest_start_contract(snapshot)
    assert result["measured_qd_rad_s"]==[.012]*6
    assert result["solver_start_velocity_rad_s"]==[0.]*6
    assert result["measured_rest_gate_used"] is True
    assert snapshot["robot"]["qd_rad_s"]==[.012]*6


@pytest.mark.parametrize("mutation",[
    lambda s:s["robot"]["native_rest_start_evidence"].update(samples=s["robot"]["native_rest_start_evidence"]["samples"][-1:]),
    lambda s:s["robot"]["native_rest_start_evidence"]["parameters"].update(velocity_tolerance_rad_s=.04),
    lambda s:s["robot"]["native_rest_start_evidence"].update(implementation_sha256="old"),
    lambda s:s["actual_state_context"].update(world_session_id="other-world"),
    lambda s:s["robot"]["q_rad"].__setitem__(0,.001),
    lambda s:s["robot"].pop("native_rest_start_evidence"),
])
def test_rest_conversion_requires_current_exact_multiframe_gate(mutation):
    from unloading_sim.moveit2_backend import native_rest_start_contract
    snapshot=rest_snapshot();mutation(snapshot)
    with pytest.raises(MoveItUnavailable):
        native_rest_start_contract(snapshot)


def test_native_nominal_compression_matches_authority_and_preserves_rigid_geometry():
    from unloading_sim.geometry import OBB
    from unloading_sim.moveit2_backend import native_tool_collision_boxes
    raw=OBB(np.array([0.,0.,.04]),np.array([.02,.02,.02]),np.eye(3),"bellows_18","tool")
    compressed=OBB(np.array([0.,0.,.035]),np.array([.02,.02,.015]),np.eye(3),"bellows_18","tool")
    rigid=OBB(np.array([.2,.1,.03]),np.array([.005,.004,.003]),np.eye(3),"rigid_insert_18","tool")
    target=OBB(np.array([0.,0.,.15]),np.array([.1,.1,.1]),np.eye(3),"target","carton")
    c=SimpleNamespace(tool_collision_obbs_provider=lambda q:[rigid,raw],
        robot_state_validator=SimpleNamespace(_compliant_boxes=lambda q:[compressed],nominal_cup_compression_m=.01))
    tools,audit=native_tool_collision_boxes(c,np.zeros(6))
    assert tools[0] is rigid
    np.testing.assert_array_equal(tools[0].center,[.2,.1,.03])
    np.testing.assert_array_equal(tools[0].half_extents,[.005,.004,.003])
    np.testing.assert_array_equal(tools[1].center,[0.,0.,.035])
    np.testing.assert_array_equal(tools[1].half_extents,[.02,.02,.015])
    assert raw.signed_distance_obb(target)==pytest.approx(-.01,abs=1e-12)
    assert tools[1].signed_distance_obb(target)==pytest.approx(0.,abs=1e-12)
    assert audit["compliant_replacements"]==["bellows_18"]
    assert audit["rigid_links_unchanged"]==["rigid_insert_18"]


@pytest.mark.parametrize("stage,loaded",[("pregrasp",False),("transit",True)])
def test_connected_pose_carries_exact_native_endpoint_into_next_stage(stage,loaded):
    """Exercise the real core selection code with a numeric solver test double.

    This is a regression for the retained failed run, not a native rerun or a
    reuse of its path. These six-joint numbers are an independent minimal case.
    """
    from contextlib import nullcontext
    from unloading_sim.ik import IKResult
    c=receipt_connector()
    initial=c.native_root_state(np.zeros(6),[])
    ik_goal=np.array([.1,.1,.2,.1,.1,1.25])
    emitted=ik_goal.copy()
    emitted[2]=np.nextafter(emitted[2],np.inf)
    emitted[5]+=8.881784197001252e-16
    assert emitted[2]-ik_goal[2]==2.7755575615628914e-17
    assert emitted[5]-ik_goal[5]==8.881784197001252e-16
    endpoint=accepted_state(c,initial,emitted,"stage-1",stage=stage)
    native_path=[initial.copy(),endpoint]
    before=deepcopy(c.native_verified)
    c._statistics={name:0 for name in ("trajectory_ik_wall_seconds","ik_calls","ik_seeds_attempted",
        "ik_iterations_consumed","cartesian_samples","state_validations","state_cache_hits")}
    c.budget=SimpleNamespace(stage_connection_attempts=1,stage_connection_iterations=10,proof_of_concept=True)
    validator=SimpleNamespace(check_states=lambda states,budget:[SimpleNamespace(valid=True)]*len(states),
        context=SimpleNamespace(context_id="current-scene",guarantee="unit-test-context"),
        context_current=lambda:"current-scene")
    c._motion_validator=lambda *a,**k:validator
    c._validation_request=lambda:None
    c._deadline_monotonic=None;c._deadline_reached=lambda:False
    c._budget_scope=lambda *a,**k:nullcontext()
    c._optional_deadline=lambda:None;c._optional_quality=lambda *a,**k:None
    class Stream:
        def __init__(self):self.results=iter([IKResult(True,ik_goal,1,0.,0.,search_evidence={"candidate_id":"current-native-ik"})])
        def __iter__(self):return self
        def __next__(self):return next(self.results)
        def evidence(self):return dict(seeds_attempted=1,iterations_consumed=0)
    c._ik_stream=lambda *a,**k:Stream()
    def solved_transit(start,goal,*args,**kwargs):
        np.testing.assert_array_equal(start,initial)
        np.testing.assert_array_equal(goal,ik_goal)
        return native_path,None,dict(selected_method=GenerationMethod.JOINT_DIRECT.value,
                                    planning_iterations_consumed=0)
    c._transit=solved_transit
    selected,path,failure,evidence=c._connect_pose(np.eye(4),[initial],initial,[],stage=stage,
        purpose=MotionPurpose.FREE_LOADED_TRANSFER if loaded else MotionPurpose.FREE_APPROACH,
        attachment=SimpleNamespace() if loaded else None,ik_seed=1,connection_seed=2)
    assert failure is None
    np.testing.assert_array_equal(selected,emitted)
    np.testing.assert_array_equal(path[-1],emitted)
    assert c._native_parent_stage_id(selected)=="stage-1"
    with pytest.raises(MoveItUnavailable,match="NATIVE_PARENT_IDENTITY_MISSING"):
        c._native_parent_stage_id(ik_goal)
    assert selected.native_receipt==endpoint.native_receipt
    assert evidence["returned_endpoint"]["source"]=="NATIVE_PATH_FINAL_SAMPLE"
    assert evidence["returned_endpoint"]["differs_from_ik_goal"] is True
    coverage=audit_native_motion_coverage(path,c.native_verified,task_id=c.native_task_id)
    assert coverage["passed"] and coverage["nonzero_edge_count"]==1
    assert c.native_verified==before  # Native points and receipt remain untouched.
    selected[0]+=1.
    np.testing.assert_array_equal(path[-1],emitted)  # The next-stage state owns a copy.
    with pytest.raises(MoveItUnavailable,match="NATIVE_PARENT_ENDPOINT_CHANGED"):
        c._native_parent_stage_id(selected)


def test_equal_q_distinct_selected_stage_and_backtracking_are_explicit():
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    a=accepted_state(c,root,np.full(6,.1),"A")
    b=accepted_state(c,root,np.full(6,.1),"B")
    np.testing.assert_array_equal(a,b)
    assert a.native_receipt["receipt_id"]!=b.native_receipt["receipt_id"]
    # The saved A branch wins even though B is the latest and exactly equal.
    selected=a.copy()
    request=c._build_native_request(selected,np.full(6,.2),[],stage="contact")
    assert c._native_parent_for_request(selected,request)[0].stage_id=="A"
    child=accepted_state(c,selected,np.full(6,.2),"C",stage="contact")
    assert [r["stage_id"] for r in c._native_selected_records(child)]==["A","C"]
    assert [r["stage_id"] for r in c._native_selected_records(b)]==["B"]
    assert c._native_parent_stage_id(root)==""
    assert c._native_parent_stage_id(a)=="A"
    # Same entire path from B must never enter the selected A→C source audit.
    sources=c._native_selected_records(child)
    coverage=audit_native_motion_coverage([root,a,child],sources,task_id=c.native_task_id)
    assert coverage["passed"] and [r["stage_id"] for r in coverage["records"]]==["A","C"]
    full=[root.copy()];stages={}
    c._append_stage(full,stages,"contact",[root,a,child])
    assert c._native_final_selection[0].native_receipt==child.native_receipt
    assert c._native_final_selection[1]==digest([q.tolist() for q in full])


def test_zero_motion_preserves_parent_and_registers_attach_and_release():
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    contact=accepted_state(c,root,np.full(6,.1),"contact",stage="contact")
    # Real inherited zero support-release branch, followed by zero place and
    # withdrawal. The native stage identity never resets to the task root.
    c.collision_policy=SimpleNamespace(allows_stack_planning_contact=lambda stage:True)
    attachment=SimpleNamespace()
    attached,failure,_=c._support_release(contact,attachment,[],(),None,seed=1)
    assert failure is None
    assert c._native_parent_stage_id(attached[-1])=="contact"
    assert attached[-1].native_receipt["context"]["attached"] is True
    placed=c._native_zero_state(attached[-1],[],stage="place",attachment=attachment)
    withdrawn=c._native_zero_state(placed,[],stage="withdrawal")
    assert c._native_parent_stage_id(withdrawn)=="contact"
    assert withdrawn.native_receipt["context"]["attached"] is False
    transitions=[e["ownership_transition"] for e in c.native_semantic_events
        if e["event"]=="ZERO_MOTION_STATE_CONTINUATION"]
    assert transitions==["ATTACH",None,"RELEASE"]
    assert len(c.native_verified)==1
    terminal=c._native_terminal_state_request(withdrawn)
    assert terminal["task_id"]==c.native_task_id and terminal["parent_stage_id"]=="contact"
    assert terminal["q_start"]==withdrawn.tolist() and terminal["attachment"] is None
    assert not {"q_goal","goal_pose","pipeline_id","planner_id","path","stages"}&terminal.keys()
    terminal["q_start"][0]=99.
    assert c._native_terminal_state_request(withdrawn)["q_start"]==withdrawn.tolist()
    assert c._native_terminal_state_request(contact) is None
    with pytest.raises(MoveItUnavailable,match="TERMINAL_PROCESS_UNFINISHED"):
        c._native_terminal_state_request(placed)
    # The saved contact identity can be selected again after release backtrack.
    attached_again=c._native_zero_state(contact,[],stage="support-release",attachment=attachment)
    assert attached_again.native_receipt["context"]["attached"] is True


@pytest.mark.parametrize("key,value",[("parent_stage_id","unselected"),("task_id","other-task"),
                                     ("q_start",[.10000000000000002]*6)])
def test_terminal_zero_state_cannot_change_selected_parent_or_endpoint(key,value):
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    contact=accepted_state(c,root,np.full(6,.1),"contact",stage="contact")
    attached=c._native_zero_state(contact,[],stage="support-release",attachment=object())
    placed=c._native_zero_state(attached,[],stage="place",attachment=object())
    withdrawn=c._native_zero_state(placed,[],stage="withdrawal")
    c._native_zero_state_requests[withdrawn._native_receipt.receipt_id][key]=value
    with pytest.raises(MoveItUnavailable,match="TERMINAL_STATE_IDENTITY_CHANGED"):
        c._native_terminal_state_request(withdrawn)


def test_checked_departure_wait_registers_release_at_the_selected_native_state(monkeypatch):
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    contact=accepted_state(c,root,np.full(6,.1),"contact",stage="contact")
    attached=c._native_zero_state(contact,[],stage="support-release",attachment=object())
    placed=c._native_zero_state(attached,[],stage="place",attachment=object())
    monkeypatch.setattr(LayoutTrajectoryConnector,"_departure",
        lambda self,start,*args,**kwargs:([start.copy()],None,{"model":"checked-wait-test-double"}))
    target=deepcopy(placed.native_receipt["context"]["target"])
    path,failure,_=c._departure(placed,target,[],np.array([0.,-1.,0.]),seed=1)
    assert failure is None and len(path)==1 and len(c.native_verified)==1
    assert c._native_parent_stage_id(path[-1])=="contact"
    terminal=c._native_terminal_state_request(path[-1])
    assert terminal["stage"]=="withdrawal" and terminal["attachment"] is None


@pytest.mark.parametrize("entry",["transit","cartesian"])
def test_zero_motion_entry_keeps_explicit_stage_identity(entry):
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    state=accepted_state(c,root,np.full(6,.1),"selected")
    if entry=="transit":
        path,failure,_=c._transit(state,state.copy(),[],purpose=MotionPurpose.FREE_APPROACH,
            seed=1,iteration_budget=1,stage="pregrasp")
    else:
        path,failure,_=c._cartesian(state,np.eye(4),[],purpose=MotionPurpose.FREE_APPROACH,
            seed=1,stage="pregrasp")
    assert failure is None and len(path)==1
    assert c._native_parent_stage_id(path[-1])=="selected"
    assert len(c.native_verified)==1


@pytest.mark.parametrize("mutate,reason",[
    (lambda c,q:np.asarray(q),"IDENTITY_MISSING"),
    (lambda c,q:q+.001,"ENDPOINT_CHANGED"),
    (lambda c,q:q[:3],"ENDPOINT_CHANGED"),
    (lambda c,q:NativeStageState(q,replace(q._native_receipt,task_id="different-task")),"TASK_MISMATCH"),
    (lambda c,q:NativeStageState(q,replace(q._native_receipt,stage_id="different-stage")),"IDENTITY_UNISSUED"),
])
def test_missing_modified_or_forged_parent_identity_fails_closed(mutate,reason):
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    state=accepted_state(c,root,np.full(6,.1),"A")
    with pytest.raises(MoveItUnavailable,match=reason):
        c._native_parent_stage_id(mutate(c,state))


def test_parent_identity_rejects_stale_model_scene_source_and_new_task():
    for mutation,reason in (
        (lambda c:c.native_identity.update(model="changed"),"MODEL_CONTEXT_CHANGED"),
        (lambda c:c.native_scene.snapshot.update(changed=True),"FROZEN_SCENE_CHANGED"),
        (lambda c:c.native_verified[0].update(authoritative_status="REJECTED"),"SOURCE_MISMATCH"),
        (lambda c:setattr(c,"native_task_id","new-task"),"TASK_MISMATCH")):
        c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
        state=accepted_state(c,root,np.full(6,.1),"A")
        mutation(c)
        with pytest.raises(MoveItUnavailable,match=reason):
            c._native_parent_stage_id(state)


def test_parent_context_rejects_wrong_target_world_pose_and_ownership():
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    state=accepted_state(c,root,np.full(6,.1),"A")
    request=c._build_native_request(state,state,[],stage="pregrasp")
    variants=[]
    changed=deepcopy(request);changed["process_policy"]["target"]["pose"][0][3]=.01
    variants.append((changed,"TARGET_POSE_CHANGED"))
    changed=deepcopy(request);changed["world"].append(dict(id="unknown-object"))
    variants.append((changed,"CONTEXT_CHANGED:other_world"))
    changed=deepcopy(request);changed["attachment"]={"id":"target"}
    variants.append((changed,"OWNERSHIP_TRANSITION_INVALID"))
    changed=deepcopy(request);changed["process_policy"]["target_id"]="other"
    changed["process_policy"]["target"]["id"]="other"
    variants.append((changed,"CONTEXT_CHANGED:target_id"))
    for changed,reason in variants:
        with pytest.raises(MoveItUnavailable,match=reason):
            c._native_parent_for_request(state,changed)


def test_root_registration_is_frozen_and_cannot_reset_a_selected_branch():
    c=receipt_connector();root=c.native_root_state(np.zeros(6),[])
    accepted_state(c,root,np.full(6,.1),"A")
    assert c.native_root_state(np.zeros(6),[]).native_receipt==root.native_receipt
    for q,kwargs,reason in ((np.full(6,.1),{},"NOT_FROZEN_INITIAL"),
            (np.zeros(6),{"attachment":object()},"ROOT_ATTACHED")):
        with pytest.raises(MoveItUnavailable,match=reason):
            c.native_root_state(q,[],**kwargs)


@pytest.mark.parametrize("exceptional",[False,True])
def test_speculative_contact_context_restores_native_and_core_nested_masks(exceptional):
    c=receipt_connector()
    c._native_contact_context=dict(face="front",selection=dict(commanded_active_mask=[True,False]))
    c.robot_state_validator=SimpleNamespace(commanded_cup_mask=(True,False),
        contact_target_name="selected-target",stack_carton_names={"selected-target","neighbor"})
    c.stack_carton_names={"selected-target","neighbor"}
    baseline=deepcopy(c._native_contact_context)
    def speculative():
        with c._contact_context():
            c._native_contact_context["face"]="side"
            c._native_contact_context["selection"]["commanded_active_mask"][0]=False
            c.robot_state_validator.commanded_cup_mask=(False,False)
            c.robot_state_validator.contact_target_name="candidate-target"
            c.stack_carton_names.add("candidate-target")
            with c._contact_context():
                c._native_contact_context["face"]="top"
                c._native_contact_context["selection"]["commanded_active_mask"][1]=True
                c.robot_state_validator.commanded_cup_mask=(False,True)
            assert c._native_contact_context==dict(face="side",selection=dict(commanded_active_mask=[False,False]))
            assert c.robot_state_validator.commanded_cup_mask==(False,False)
            if exceptional:
                raise RuntimeError("candidate rejected")
    if exceptional:
        with pytest.raises(RuntimeError,match="candidate rejected"):
            speculative()
    else:
        speculative()
    assert c._native_contact_context==baseline
    assert c.robot_state_validator.commanded_cup_mask==(True,False)
    assert c.robot_state_validator.contact_target_name=="selected-target"
    assert c.stack_carton_names=={"selected-target","neighbor"}
