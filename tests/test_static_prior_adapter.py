"""Protocol-only regressions; synthetic replies are never planning evidence."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.moveit2_backend import (JOINT_NAMES, MoveItLayoutConnector,
    MoveItUnavailable, NativeIKCandidateStream, native_generation_evidence,
    validate_native_result)
from unloading_sim.planning_only import MARKER, SKIPPED
from test_planning_only import experiment


def semantic_result():
    binding = dict(task_id="task", stage_id="place", parent_stage_id="transit")
    return dict(status="SUCCESS", joint_names=JOINT_NAMES.copy(), **binding,
        generation_source="SEMANTIC_EVENT", native_solver_calls={},
        returned_waypoint_count=0, points=[], terminal_q=[0.]*6,
        semantic_event=dict(event_type="PLACE_TARGET_REACHED", **binding,
            target_id="target", goal_constraints_satisfied=True, start_unchanged=True))


def request():
    return dict(stage="place", planning_only=True, pipeline_id="pilz_industrial_motion_planner",
        process_policy=dict(target_id="target"))


def test_semantic_event_is_opt_in_and_has_no_fabricated_points():
    raw = semantic_result()
    with pytest.raises(ValueError, match="SEMANTIC_EVENT"):
        validate_native_result(raw, np.zeros(6), None, JOINT_NAMES)
    state = validate_native_result(raw, np.zeros(6), None, JOINT_NAMES,
        allow_semantic_place=True)
    assert len(state) == 1 and np.array_equal(state[0], np.zeros(6))
    assert raw["points"] == [] and raw["native_solver_calls"] == {}
    assert native_generation_evidence(raw, request(), planning_mode="static_prior_fast")
    assert not native_generation_evidence(raw, request())


@pytest.mark.parametrize("kind", ["task", "parent", "stage", "goal", "changed", "nan",
    "points", "calls", "source", "target", "missing"])
def test_semantic_event_rejects_changed_state_and_binding(kind):
    raw = semantic_result()
    if kind in {"task", "parent", "stage"}:
        raw["semantic_event"][{"task":"task_id", "parent":"parent_stage_id", "stage":"stage_id"}[kind]] = "other"
    elif kind == "goal": raw["semantic_event"]["goal_constraints_satisfied"] = False
    elif kind == "changed": raw["terminal_q"][0] = 1e-12
    elif kind == "nan": raw["terminal_q"][0] = float("nan")
    elif kind == "points": raw["points"] = [{"q":[0.]*6}]
    elif kind == "calls": raw["native_solver_calls"] = {"LIN":1}
    elif kind == "source": raw["generation_source"] = "NATIVE_GENERATED"
    elif kind == "target": raw["semantic_event"]["target_id"] = ""
    elif kind == "missing": raw.pop("semantic_event")
    with pytest.raises(ValueError):
        validate_native_result(raw, np.zeros(6), None, JOINT_NAMES, allow_semantic_place=True)


def test_semantic_target_and_joint_goal_are_checked():
    raw = semantic_result()
    raw["semantic_event"]["target_id"] = "another-target"
    assert not native_generation_evidence(raw, request(), planning_mode="static_prior_fast")
    with pytest.raises(ValueError, match="GOAL_CHANGED"):
        validate_native_result(raw, np.zeros(6), np.ones(6)*.01, JOINT_NAMES, allow_semantic_place=True)


def test_static_prior_source_requires_current_geometry_and_correct_scope():
    raw = dict(generation_source="STATIC_PRIOR_REUSE", native_solver_calls={},
        prior_usage=dict(prior_id="static-id", context_checked=True, prior_hit=True,
            prior_nodes_reused=2, prior_edges_reused=1, connector_edges=2,
            current_geometry_checked=True, connector_kind="CHECKED_JOINT_INTERPOLATION", route_nodes=[2, 4]))
    submitted = dict(planning_only=True, stage="transit", pipeline_id="static_prior")
    assert native_generation_evidence(raw, submitted, planning_mode="static_prior_fast")
    assert not native_generation_evidence(raw, submitted)
    for change in (dict(current_geometry_checked=False), dict(context_checked=False),
                   dict(connector_kind="UNPLANNED"), dict(prior_id=""), dict(route_nodes=None)):
        bad=deepcopy(raw);bad["prior_usage"].update(change)
        assert not native_generation_evidence(bad, submitted, planning_mode="static_prior_fast")
    for change in (dict(planning_only=False), dict(stage="contact"), dict(goal_pose=np.eye(4).tolist())):
        assert not native_generation_evidence(raw, {**submitted, **change}, planning_mode="static_prior_fast")
    assert not native_generation_evidence(dict(native_solver_calls={"LIN":True}), {})


def test_explicit_fast_mode_cannot_enter_normal_connector():
    with pytest.raises(ValueError, match="PLANNING_ONLY"):
        MoveItLayoutConnector.from_existing(SimpleNamespace(), None, planning_mode="static_prior_fast")
    with pytest.raises(ValueError, match="UNKNOWN"):
        MoveItLayoutConnector.from_existing(SimpleNamespace(), None, planning_mode="typo")


def test_semantic_receipt_reads_terminal_state_and_preserves_stage_identity():
    c=experiment();c.planning_mode="static_prior_fast"
    root=c.native_root_state(np.zeros(6), [])
    req=c._build_native_request(root, None, [], stage="place")
    context=c._native_state_context(req)
    raw=semantic_result();raw.update(task_id=c.native_task_id, stage_id="place-event",
        parent_stage_id=root.native_receipt["stage_id"], authoritative_status=SKIPPED,
        planning_only_status=MARKER)
    c.native_generated.append(raw)
    end=c._native_issue_state(np.zeros(6), "place-event", context)
    assert c._native_state_receipt(end).stage_id == "place-event"
    c.native_generated[0]["terminal_q"][0]=.1
    with pytest.raises(MoveItUnavailable, match="SOURCE_MISMATCH"):
        c._native_state_receipt(end)


def ik_stream(reply, mode="static_prior_fast"):
    c=MoveItLayoutConnector.__new__(MoveItLayoutConnector)
    c.planning_only=True;c.planning_mode=mode
    c.ik=dict(random_restarts=0,position_tolerance_m=.001,orientation_tolerance_rad=.001)
    c.robot=SimpleNamespace(joint_limits=np.array([[-3.,3.]]*6))
    c.native_ik_seconds=.25;c._native_limits=lambda:(1.,2.)
    c._native_cancelled=lambda:False
    c._build_native_request=lambda start, goal, obs, **kwargs: dict(
        q_start=np.asarray(start).tolist(),goal_pose=np.eye(4).tolist(),seed=kwargs["seed"])
    sent=[]
    c.native=SimpleNamespace(request=lambda req, **kwargs:(sent.append(deepcopy(req)) or deepcopy(reply)))
    stream=NativeIKCandidateStream(c,np.eye(4),[np.ones(6)*i*.1 for i in range(5)],[],seed=17,stage="transit")
    return stream,sent


def test_batch_ik_consumes_only_reported_seeds_and_counts_ipc_once():
    reply=dict(status="SUCCESS",request_id="batch",consume_count=2,native_ik_calls=2,ik_s=.03,
        results=[dict(status="FAILED",native_ik_calls=1,seed_index=0),
                 dict(status="SUCCESS",native_ik_calls=1,seed_index=1,q=[.1]*6)])
    stream,sent=ik_stream(reply)
    first=stream._next_native_result();second=stream._next_native_result()
    assert sent[0]["op"] == "ik_batch" and len(sent[0]["ik_seeds"]) == 4
    assert sent[0]["stop_on_first_success"] is True
    assert stream.seed_index == 2 and len(sent) == 1
    assert first[0] == 0 and second[0] == 1 and second[4] == 0.
    assert first[5] == .03 and second[5] == 0.
    stream._next_native_result()
    assert len(sent[1]["ik_seeds"]) == 3 and sent[1]["seed"] == 19


@pytest.mark.parametrize("change", [dict(consume_count=0),dict(consume_count=5),
    dict(native_ik_calls=2),dict(results=[]),dict(status="FAILED")])
def test_batch_ik_rejects_inconsistent_native_evidence(change):
    reply=dict(status="SUCCESS",consume_count=1,native_ik_calls=1,
        results=[dict(status="SUCCESS",native_ik_calls=1,seed_index=0)])
    stream,_=ik_stream({**reply,**change})
    with pytest.raises(MoveItUnavailable, match="BATCH_EVIDENCE"):
        stream._next_native_result()


def test_cold_ik_stays_one_seed_per_request():
    stream,sent=ik_stream(dict(status="FAILED",native_ik_calls=1),mode="cold_from_scratch")
    stream._next_native_result()
    assert sent[0]["op"] == "ik" and "ik_seeds" not in sent[0] and stream.seed_index == 1


def test_fast_phase_uses_shared_deadline_and_keeps_ipc_watchdog(monkeypatch):
    c=MoveItLayoutConnector.__new__(MoveItLayoutConnector)
    c.planning_mode="static_prior_fast";c._native_task_started=100.
    c.native_stage_seconds=15.;c.native_request_timeout=60.;c.native_task_seconds=300.
    c._fast_phase_deadline_monotonic=101.;c._slow_completion_allowed=True
    monkeypatch.setattr("unloading_sim.moveit2_backend.perf_counter",lambda:100.8)
    stage,ipc=c._native_limits()
    assert stage == pytest.approx(.2) and ipc == 60. and c._planning_phase() == "FAST"
    monkeypatch.setattr("unloading_sim.moveit2_backend.perf_counter",lambda:102.)
    stage,ipc=c._native_limits()
    assert stage == 15. and ipc == 60. and c._planning_phase() == "SLOW_COMPLETION"
    assert c._fast_phase_overrun("FAST") == 1.
    assert c._native_task_started == 100.


def test_partial_batch_timeout_keeps_completed_calls_and_next_seed():
    reply=dict(status="NATIVE_IK_BATCH_TIMEOUT",consume_count=1,native_ik_calls=1,ik_s=.25,
        results=[dict(status="FAILED",native_ik_calls=1,seed_index=0,ik_s=.25)])
    stream,sent=ik_stream(reply)
    row=stream._next_native_result()
    assert row[3]["batch_status"] == "NATIVE_IK_BATCH_TIMEOUT"
    assert stream.seed_index == 1 and row[5] == .25


def test_empty_batch_timeout_preserves_real_status_without_fake_ik():
    stream,_=ik_stream(dict(status="NATIVE_IK_BATCH_TIMEOUT",
        consume_count=0,native_ik_calls=0,ik_s=0.,results=[]))
    stream.connector.native_ik_evidence=[]
    with pytest.raises(StopIteration):
        stream._next_native_result()
    assert stream.termination == "NATIVE_IK_BATCH_TIMEOUT"
    assert stream.native_calls == 0 and stream.seed_index == 0
    assert stream.connector.native_ik_evidence[0]["native_ik_calls"] == 0


def test_cancelled_batch_retains_completed_work():
    stream,_=ik_stream(dict(status="CANCELLED",consume_count=1,native_ik_calls=1,ik_s=.25,
        results=[dict(status="FAILED",seed_index=0,native_ik_calls=1)]))
    stream.connector.native_ik_evidence=[]
    with pytest.raises(StopIteration):
        stream._next_native_result()
    assert stream.termination == "CANCELLED" and stream.native_calls == 1
    assert stream.connector.native_ik_evidence[0]["ik_s"] == .25


def test_portal_node_reuse_does_not_require_a_stored_graph_edge():
    raw=dict(generation_source="STATIC_PRIOR_REUSE",native_solver_calls={},
        prior_usage=dict(prior_id="prior",context_checked=True,current_geometry_checked=True,
            connector_kind="CHECKED_JOINT_INTERPOLATION",route_nodes=[10,2,11],
            prior_hit=True,prior_nodes_reused=1,prior_edges_reused=0,connector_edges=2))
    req=dict(planning_only=True,stage="transit",pipeline_id="static_prior")
    assert native_generation_evidence(raw,req,planning_mode="static_prior_fast")
    for change in (dict(prior_hit=False),dict(prior_nodes_reused=0),
                   dict(prior_nodes_reused=True),dict(prior_edges_reused=-1)):
        bad=deepcopy(raw);bad["prior_usage"].update(change)
        assert not native_generation_evidence(bad,req,planning_mode="static_prior_fast")


def direct_result():
    return dict(generation_source="NATIVE_DIRECT_CONNECTION",native_solver_calls={},
        prior_usage=dict(prior_id="prior",context_checked=True,current_geometry_checked=True,
            connector_kind="CHECKED_JOINT_INTERPOLATION",route_nodes=[10,11],
            prior_hit=False,prior_nodes_reused=0,prior_edges_reused=0,connector_edges=1))


def test_direct_online_connection_is_distinct_from_prior_reuse_and_solver_calls():
    raw=direct_result();req=dict(planning_only=True,stage="transit",pipeline_id="static_prior")
    assert native_generation_evidence(raw,req,planning_mode="static_prior_fast")
    assert not native_generation_evidence(raw,req)
    assert not native_generation_evidence(raw,{**req,"planning_only":False},planning_mode="static_prior_fast")
    assert not native_generation_evidence({**raw,"native_solver_calls":{"OMPL":1}},req,planning_mode="static_prior_fast")
    assert not native_generation_evidence({**raw,"generation_source":"STATIC_PRIOR_REUSE"},req,planning_mode="static_prior_fast")


@pytest.mark.parametrize("change", [
    dict(prior_hit=True),dict(prior_nodes_reused=1),dict(prior_nodes_reused=False),
    dict(prior_edges_reused=1),dict(prior_edges_reused=False),
    dict(connector_edges=0),dict(connector_edges=2),dict(connector_edges=True),
    dict(context_checked=False),dict(current_geometry_checked=False),
    dict(connector_kind="UNPLANNED")])
def test_direct_online_connection_requires_zero_reuse_and_one_checked_connector(change):
    raw=direct_result();raw["prior_usage"].update(change)
    req=dict(planning_only=True,stage="transit",pipeline_id="static_prior")
    assert not native_generation_evidence(raw,req,planning_mode="static_prior_fast")


@pytest.mark.parametrize("mode", ["static_prior_fast", "cold_from_scratch"])
@pytest.mark.parametrize("shared_remaining_s", [15., 2.5])
def test_static_prior_query_ceiling_preserves_shared_budget_and_cold_schedule(
        monkeypatch, mode, shared_remaining_s):
    """Synthetic failed replies exercise real request scheduling, not planning."""
    from itertools import count
    c=MoveItLayoutConnector.__new__(MoveItLayoutConnector)
    c.require_native_motion=True;c.planning_only=True;c.planning_mode=mode
    c.static_prior_loaded=True;c.static_prior_placement_policy_sha256="synthetic-policy"
    c.native_startup={"capabilities":{"native_task_session":True}}
    c.native_task_id="synthetic-budget-test";c._native_stage_ids=count()
    c._native_task_started=100.;c.native_task_seconds=300.
    c.native_stage_seconds=30.;c.native_request_timeout=90.
    c._deadline_monotonic=100.+shared_remaining_s
    c.robot=SimpleNamespace(joint_limits=np.array([[-3.,3.]]*6))
    c.budget=SimpleNamespace(stage_connection_attempts=1)
    c.collision_policy=SimpleNamespace(poc_pair_clearance=True)
    c.native_evidence=[]
    c._deadline_reached=lambda:False;c._native_cancelled=lambda:False
    c._context_identity=lambda *args, **kwargs:"synthetic-context"
    c._state_failure=lambda *args, **kwargs:None
    c._native_parent_for_request=lambda *args:(
        SimpleNamespace(stage_id="synthetic-root", evidence=lambda:{}),None,None)
    c._build_native_request=lambda start, goal, obstacles, **kwargs:dict(
        q_start=start.tolist(),q_goal=goal.tolist(),planning_only=True,
        planning_mode=mode,stage=kwargs["stage"],allowed_planning_time_s=30.)
    monkeypatch.setattr("unloading_sim.moveit2_backend.perf_counter",lambda:100.)
    monkeypatch.setattr("unloading_sim.static_prior.request_context",lambda *args, **kwargs:{})
    monkeypatch.delenv("M710_MOVEIT_REQUEST_LOG",raising=False)
    monkeypatch.delenv("M710_MOVEIT_DIAGNOSTICS",raising=False)
    sent=[]
    def synthetic_transport(submitted, **kwargs):
        sent.append((deepcopy(submitted),kwargs["timeout"]))
        return {"status":"NATIVE_PLANNING_FAILED"}
    c.native=SimpleNamespace(request=synthetic_transport)
    path,failure,evidence=c._native_plan(np.zeros(6),np.full(6,.1),[],seed=17,stage="transit")
    pipelines=["pilz_industrial_motion_planner","ompl"]
    allowed=[shared_remaining_s,shared_remaining_s]
    if mode == "static_prior_fast":
        pipelines.insert(0,"static_prior");allowed.insert(0,min(shared_remaining_s,6.))
    assert not path and failure["reason"] == "MOVEIT2_SEARCH_EXHAUSTED"
    assert [req["pipeline_id"] for req,_ in sent] == pipelines
    assert [req["allowed_planning_time_s"] for req,_ in sent] == allowed
    assert [a["requested_planning_time_s"] for a in evidence["attempts"]] == allowed
    assert [timeout for _,timeout in sent] == [shared_remaining_s]*len(sent)
