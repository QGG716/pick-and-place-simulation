"""Summarize existing outputs only: no model, IK, planning or geometry checks."""
from pathlib import Path
import json,gzip,hashlib,shutil,csv
from collections import Counter
import numpy as np
root=Path("/work-single-contact-20261016")
run=Path("/tmp/preplace-ik-20261017")
out=root/"docs/experiments/m710_bounded_preplace_ik_20261017"
out.mkdir(parents=True,exist_ok=False)
freeze=json.loads((run/"freeze.json").read_text())
for name,sha in freeze["source_files"].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
assert hashlib.sha256((run/"build/m710_moveit_worker").read_bytes()).hexdigest()==freeze["worker_sha256"]
manifest={}
def save(src,dst,compress=False):
    data=src.read_bytes()
    dest=out/(dst+".gz" if compress else dst);dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_bytes(gzip.compress(data,mtime=0) if compress else data)
    manifest[str(dest.relative_to(out))]=dict(raw_sha256=hashlib.sha256(data).hexdigest(),
        raw_bytes=len(data),stored_sha256=hashlib.sha256(dest.read_bytes()).hexdigest())
for name in ["freeze.json","asset-identities.json","c04-input.json","run-cases.sh","worker.sh","prepare.sh","host-cpu.txt","ik-probe.py","ik-probe-worker.sh","run-ik-probe.sh","target-comparison.json","ik-comparison.json"]:
    save(run/name,name)
for name in ["environment.txt","build.log","ik-probe.log","build-retry.log","m710_bounded_ik_test.log",
    "run.log","python-tests.log","m710_native_semantic_event_test.log","m710_request_evidence_test.log"]:
    save(run/name,name,True)
for case in ["c02","c04"]:
    for mode in ["single_attempt","bounded_preplace"]:
        for suffix in ["-input.json","-result.json",".log"]:
            name=case+"-"+mode+suffix
            save(run/name,"ik-probe/"+name,True)
summaries=[];csvrows=[]
keys=["aabb_tests","aabb_skips","narrow_calls","fixed_qualification_queries",
    "fixed_aabb_qualifications","fixed_narrow_qualifications","fixed_aabb_reuses","fixed_narrow_reuses"]
for case in ["c02","c04"]:
    r=json.loads((run/case/"result.json").read_text())
    requests=[json.loads(l) for l in (run/case/"requests.jsonl").read_text().splitlines()]
    assert r["source_commit"]==freeze["source_commit"] and r["worker_sha256"]==freeze["worker_sha256"]
    assert r["qualification_status"]=="NOT_EVALUATED" and not r["continuous_swept_proof"]
    assert r["physical_execution"]=="NOT_RUN" and not r["execution_ready"]
    assert not r["forbidden_requests"] and r["prior_build_load_query_calls"]==0
    assert all(r["history_and_legacy"][k]==0 for k in ["history_inputs_read","legacy_motion_generator_calls","forbidden_entry_attempts"])
    calls=Counter();totals=Counter();request_summary=[];seen=set()
    motion=[d for d in requests if d["op"]=="plan"]
    for d in requests:
        resp=d["response"];rid=resp.get("request_id")
        assert rid not in seen and rid is not None
        seen.add(rid)
        calls.update(resp.get("native_solver_calls",{}));calls["IK"]+=resp.get("native_ik_calls",0)
        checks=resp.get("clearance",{});count=Counter()
        if checks:
            assert checks["mode"]=="optimized" and checks["native_check_reuse"] and checks["fixed_narrow_qualification"]
            ws=checks["workspace"]
            assert ws["pair_profile_enabled"] is False and ws["pair_profile_status"]=="NOT_MEASURED"
            for p in ws["phases"].values():
                assert all(p[k] is None for k in ["broad_phase_s","narrow_s","self_narrow_s","world_narrow_s"])
                count.update({k:p[k] for k in keys})
            count["states"]=sum(p["queries"] for p in checks["counts"].values())
            count["check_s"]=sum(p["seconds"] for p in checks["counts"].values())
            totals.update(count)
        if d["op"]=="ik":
            assert resp["solver_api_calls"]==resp["native_ik_calls"]==resp["ik_request_count"]==1
            assert resp["ik_collision_checked"] is False
            if d["stage"]=="transit":
                assert resp["ik_mode"]=="bounded_preplace" and resp["solver_api"]=="searchPositionIK"
                assert resp["internal_attempts"] is None and resp["random_restarts"] is None
                assert 0<resp["effective_solver_budget_s"]<=resp["requested_solver_budget_s"]<=.1
                parent=next(x for x in motion if x.get("response",{}).get("stage_id")==d["parent_stage_id"])
                assert parent["stage"]=="extraction"
                generated=next(x for x in r["generated_stage_evidence"] if x["stage_id"]==d["parent_stage_id"])
                assert generated["points"][-1]["q"]==d["q_start"]
            else:assert resp["solver_api"]=="getPositionIK" and resp["ik_mode"]=="single_attempt"
        proc=resp.get("process_policy",{})
        if proc:assert proc["profile_enabled"] is False
        if d["op"] in ["ik","plan","check_transit_end"] and d.get("stage")=="transit":
            assert d["attachment"]["id"]==r["target_id"] and not d["target_in_world"]
            assert d["ownership_transition"] is None and d["parent_stage_id"]
            ctx=d["process_context"]
            assert ctx["support_names"]==[] and ctx["initial_proximity"] is None
            assert any(ctx["actual_contact_mask"]) and ctx["actual_contact_mask"]==ctx["commanded_active_mask"]
            if proc:assert not proc["relaxed_stack"] and not proc["support_names"] and proc["attached"]
        if d["op"] in ["ik","plan","check_start"] and d.get("stage")=="pregrasp":
            assert not d["attachment"] and d["target_in_world"]
            assert d["q_start"]==r["start_q_rad"]
        request_summary.append(dict(request_id=rid,op=d["op"],stage=d.get("stage"),candidate_id=d.get("candidate_id"),
            task_id=d.get("task_id"),stage_id=resp.get("stage_id"),parent_stage_id=d.get("parent_stage_id"),
            status=d["status"],round_trip_s=d["round_trip_s"],native_solver_calls=resp.get("native_solver_calls"),
            native_ik_calls=resp.get("native_ik_calls"),error_code=resp.get("error_code"),
            solver_message=resp.get("solver_message"),
            ik={k:resp.get(k) for k in ["ik_mode","solver_api","requested_solver_budget_s","effective_solver_budget_s",
                "actual_ik_s","error_code","bounded_search","solver_api_calls","ik_request_count","internal_attempts",
                "internal_attempts_observability","ik_collision_checked","random_restarts","seed_scope"]} if d["op"]=="ik" else None,clearance=dict(count) if checks else None,
            process={k:proc[k] for k in ["attached","relaxed_stack","support_names","state_checks",
                "seal_input_compilations","seal_geometry_evaluations","compression_sat_evaluations",
                "fixed_seal_verdict_reuses","profile_enabled"] if k in proc},
            rejection_examples=checks.get("rejected_examples",[])))
    assert {k:calls[k] for k in r["native_calls"]}==r["native_calls"]
    stages=r["generated_stage_evidence"]
    byid={s["stage_id"]:s for s in stages}
    assert len(byid)==len(stages)
    boundaries=[]
    for s in stages:
        points=s["points"]
        parent=byid.get(s["parent_stage_id"])
        if parent:assert points[0]["q"]==parent["points"][-1]["q"]
        else:assert points[0]["q"]==r["start_q_rad"] and not s["parent_stage_id"]
        boundaries.append(dict(stage=s["stage"],stage_id=s["stage_id"],parent_stage_id=s["parent_stage_id"],
            generation_source=s["generation_source"],pipeline_id=s["pipeline_id"],planner_id=s["planner_id"],
            start_q_rad=points[0]["q"],end_q_rad=points[-1]["q"],returned_points=len(points),
            end_velocity_rad_s=points[-1]["v"],native_output_status=s["native_output_status"],
            authoritative_status=s["authoritative_status"]))
    attempts=[]
    for a in r["attempts"]:
        p=a.get("preplace_target")
        attempts.append(dict(face=a["target"]["face_role"],face_id=a["target"]["geometric_face_id"],
            complete=a["complete"],failure=a["failure"],times=a["times"],
            extraction_complete=a.get("extraction_complete",False),
            attachment={k:v for k,v in a.get("attachment",{}).items() if k!="parent_receipt"},preplace_target=p,preplace_endpoint=a.get("preplace_endpoint"),
            extraction_end_q_rad=a.get("extraction_end_q_rad"),actual_end_q_rad=a.get("actual_end_q_rad")))
        csvrows.append(dict(case=case,face=a["target"]["face_role"],face_id=a["target"]["geometric_face_id"],
            complete=a["complete"],failure="" if not a["failure"] else a["failure"]["reason"],**a["times"]))
    if r["complete"]:
        p=r["plan"];assert len(p["stages"])==4 and len(p["events"])==1 and p["events"][0]["event_type"]=="ATTACH"
        assert p["transit"][0]==p["extraction"][-1]
        assert p["final_state_receipt"]["context"]["attached"]
        assert p["final_state_receipt"]["stage_id"]==p["stages"][-1]["stage_id"]
        assert p["preplace_endpoint"]["native_check"]["status"]=="SUCCESS"
        assert all(v==0 for v in p["stages"][-1]["points"][-1]["v"])
        assert np.allclose(np.asarray(p["preplace_target"]["physical_contact_pose"])@np.asarray(p["attachment"]["tcp_from_box"]),
            p["preplace_target"]["preplace_box_pose"],atol=1e-10,rtol=0)
    item={k:r[k] for k in ["target_id","run_id","complete","selected_face","source_commit","worker_sha256",
        "initialization_s","scene_prepare_s","target_generation_s","total_planning_s","native_calls",
        "prior_build_load_query_calls","history_and_legacy","start_q_rad","removed_cartons","scene_revision",
        "initial_input_sha256","budgets","environment"]}
    item.update(initial_carton_count=len(r["initial_cartons"]),attempts=attempts,requests=request_summary,
        stage_boundaries=boundaries,clearance_totals=dict(totals),
        lightweight_record_checks=dict(source_worker_identity=True,parent_and_q_continuity=True,
            free_transit_context=True,actual_masks=True,pair_profile_disabled=True,
            prior_and_legacy_zero=True,abandoned_faces_start_from_root=True,
            no_additional_geometry_or_solves=True))
    summaries.append(item)
    for f in ["result.json","requests.jsonl","worker.log"]:save(run/case/f,case+"/"+f,True)
    save(run/(case+"-stdout.txt"),case+"/stdout.txt")
summary=dict(schema="m710_bounded_preplace_ik_summary_v1",status="PLANNING_ONLY_NOT_EXECUTABLE",
    qualification_status="NOT_EVALUATED",continuous_swept_proof=False,physical_execution="NOT_RUN",
    complete_cases=sum(x["complete"] for x in summaries),case_count=2,source_commit=freeze["source_commit"],
    frozen=freeze,cases=summaries,ik_comparison=json.loads((run/"ik-comparison.json").read_text()),
    target_comparison=json.loads((run/"target-comparison.json").read_text()),
    timing_semantics="total includes all face candidates/failed attempts; initialization includes scene_prepare; RTT/native/checks nested, never added to stage time; no five-box result")
(out/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+"\n")
fields=list(dict.fromkeys(k for row in csvrows for k in row))
with (out/"timing.csv").open("w",newline="") as f:
    writer=csv.DictWriter(f,fieldnames=fields,lineterminator="\n");writer.writeheader();writer.writerows(csvrows)
(out/"raw-manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
save(run/"summarize.py","summarize.py")
for s in summaries:
    print(json.dumps(dict(case=s["target_id"],total=s["total_planning_s"],clearance=s["clearance_totals"],
        stage_counts=[(x["stage"],x["returned_points"]) for x in s["stage_boundaries"]])))
print("ARCHIVE_COMPLETE")
