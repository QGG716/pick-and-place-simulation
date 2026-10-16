from pathlib import Path
import json, math, hashlib
HERE=Path(__file__).resolve().parent
def load_result(case):
    import gzip
    path=HERE/case/"result.json"
    return json.loads(path.read_text() if path.exists() else gzip.decompress(path.with_suffix(".json.gz").read_bytes()))
def load_requests(case):
    import gzip
    path=HERE/case/"requests.jsonl"
    data=path.read_text() if path.exists() else gzip.decompress(path.with_suffix(".jsonl.gz").read_bytes()).decode()
    return [json.loads(line) for line in data.splitlines()]
summary=dict(schema="m710_single_contact_result_summary_v1",
    source_commit="91e6d2caf1a2decb548992a0c38520d08a932379",
    status="APPROACH_CONTACT_PLANNED_NOT_EXECUTABLE",
    qualification_status="NOT_EVALUATED",continuous_swept_proof=False,physical_execution="NOT_RUN",
    timing_semantics="total includes generation/all face attempts; initialization separate; scene_prepare within initialization; IK/pregrasp/contact include request build/round-trip/return processing; native solve/checks nested, not additive",
    tests=dict(python_passed=156,new_cases_passed=38,native_passed=["m710_native_semantic_event_test","m710_request_evidence_test","m710_clearance_test"]),
    cases=[])
for case in ["c02","c04"]:
    result=load_result(case);requests=load_requests(case);plan=result["plan"];attempt=result["attempts"][0]
    assert result["source_commit"]==summary["source_commit"]
    assert result["complete"] and plan["events"]==[] and result["events"]==[]
    assert len(result["attempts"])==1 and result["selected_face"]=="x-"
    assert len(result["initial_cartons"])==({"c02":40,"c04":36}[case])
    assert result["prior_build_load_query_calls"]==0 and result["forbidden_requests"]==0
    assert result["native_calls"]==dict(IK=1,OMPL=1,LIN=1,PTP=0)
    assert all(result["history_and_legacy"][k]==0 for k in ["history_inputs_read","legacy_motion_generator_calls","forbidden_entry_attempts"])
    assert [r["op"] for r in requests]==["init","fk","fk","fk","check_start","ik","plan","plan"]
    ids=[r["response"]["request_id"] for r in requests];assert len(ids)==len(set(ids))
    stages=plan["stages"];pre,contact=stages
    assert pre["parent_stage_id"]=="" and contact["parent_stage_id"]==pre["stage_id"]
    assert pre["stage_id"].startswith(plan["task_id"]+":") and contact["stage_id"].startswith(plan["task_id"]+":")
    assert plan["start_q_rad"]==plan["pregrasp"][0]
    assert plan["pregrasp"][-1]==plan["contact"][0]
    assert plan["end_q_rad"]==plan["contact"][-1]
    assert requests[-1]["q_start"]==plan["pregrasp"][-1] and requests[-1]["parent_stage_id"]==pre["stage_id"]
    for stage in stages:
        assert stage["authoritative_status"]=="SKIPPED_PLANNING_ONLY"
        assert stage["native_output_status"]=="SKIPPED_PLANNING_ONLY"
        assert stage["generation_source"]=="NATIVE_GENERATED"
        assert all(math.isfinite(x) for point in stage["points"] for key in ["q","v","a"] for x in point[key])
        assert all(x==0 for point in [stage["points"][0],stage["points"][-1]] for x in point["v"])
    assert attempt["contact_fk_check"]["passed"] and attempt["contact_fk_check"]["additional_samples"]==0
    terminal=attempt["terminal_contact"];expected=attempt["expected_contact"]
    assert expected["actual_contact_mask"] is None
    assert terminal["actual_contact_mask"]==expected["commanded_active_mask"]==terminal["commanded_active_mask"]
    assert terminal["attached"] is False and terminal["measured"] is False
    costrows=[]
    for request in requests:
        resp=request["response"]
        if "clearance" not in resp:continue
        evidence=resp["request_check_evidence"];cl=resp["clearance"];ws=cl["workspace"]
        assert evidence["scope"]=="CURRENT_REQUEST_ONLY"
        assert cl["native_check_reuse"] and cl["fixed_narrow_qualification"] and cl["mode"]=="optimized"
        assert ws["pair_profile_enabled"] is False and ws["pair_profile_status"]=="NOT_MEASURED"
        assert resp["process_policy"]["profile_enabled"] is False
        assert request["target_in_world"] and request["attachment"] is None and resp["process_policy"]["attached"] is False
        for phase in ws["phases"].values():
            assert all(phase[key] is None for key in ["broad_phase_s","narrow_s","self_narrow_s","world_narrow_s"])
        counts={key:sum(phase[key] for phase in ws["phases"].values()) for key in
            ["aabb_tests","aabb_skips","narrow_calls","fixed_qualification_queries","fixed_aabb_qualifications","fixed_aabb_reuses","fixed_narrow_qualifications","fixed_narrow_reuses"]}
        costrows.append(dict(request_id=resp["request_id"],op=request["op"],stage=request["stage"],status=request["status"],
            request_round_trip_s=request["round_trip_s"],mtc_plan_s=resp.get("mtc_plan_s"),
            clearance_check_s=evidence["clearance_check_s"],clearance_states=evidence["clearance_queries"],
            workspace_prepare_s=ws["prepare_s"],process_state_checks=resp["process_policy"]["state_checks"],
            seal_input_compilations=resp["process_policy"]["seal_input_compilations"],**counts))
    ik=next(x for x in requests if x["op"]=="ik")
    assert ik["q_start"]==plan["start_q_rad"] and ik["response"]["random_restarts"]==0
    summary["cases"].append(dict(case=case,run_id=result["run_id"],complete=True,attempted_faces=["x-"],selected_face="x-",
        initialization_s=result["initialization_s"],scene_prepare_s=result["scene_prepare_s"],
        target_generation_s=result["target_generation_s"],**attempt["times"],total_planning_s=result["total_planning_s"],
        ik_request_round_trip_s=ik["round_trip_s"],ik_solver_s=ik["response"]["ik_s"],
        native_calls=result["native_calls"],prior_calls=0,initial_carton_count=len(result["initial_cartons"]),
        removed_cartons=result["removed_cartons"],target=plan["target"],
        start_q_rad=plan["start_q_rad"],pregrasp_end_q_rad=plan["pregrasp"][-1],contact_end_q_rad=plan["end_q_rad"],
        waypoint_counts=dict(pregrasp=len(plan["pregrasp"]),contact=len(plan["contact"])),
        parent_and_exact_state_continuity=True,stationary_stage_boundaries=True,
        terminal_planned_geometric_cup_count=sum(terminal["actual_contact_mask"]),
        contact_fk_check=attempt["contact_fk_check"],costs=costrows))
(HERE/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
for r in summary["cases"]:
    print(r["case"],r["total_planning_s"],r["waypoint_counts"],"costs",r["costs"])
