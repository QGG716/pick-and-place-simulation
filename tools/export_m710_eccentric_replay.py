"""Extend the existing native replay with scoped contact and structural FK evidence."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from export_m710_search_replay import export_model
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.upper_arm_preference import upper_arm
from unloading_sim.independent_cups import build_m710_independent_cup_array
from unloading_sim.coupled_batch import message_box
from unloading_sim.top_layer_lifted import FrozenTopLayer
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.right_front_eccentric import eccentric_target,offset_intervals,candidate_deltas
from unloading_sim.support import SupportRelationGraph
from types import SimpleNamespace


def export(run, output):
    read=lambda p:json.loads(p.read_text(encoding="utf-8"))
    protocol=read(run/"protocol.json");result=read(run/"seed-71070/result.json")
    robot=load_workcell_layout(ROOT/protocol["active_layout_file"]).robot()
    attempts=[];logs=[];model=None;targets={}
    frozen=FrozenTopLayer.capture([message_box(b) for b in protocol["original_cartons"]])
    for row in result["attempts"]:
        if not row.get("folder"):continue
        folder=run/"seed-71070"/row["folder"]
        trace=read(folder/"search-trace.json");calls=read(folder/"requests.json")
        detail=read(folder/"attempt.json") if (folder/"attempt.json").exists() else {}
        if model is None:model=export_model(trace["model"])
        # An exception can precede attempt.json. Reconstruct only target geometry
        # from the saved native process contract, then verify its candidate ID.
        if not detail.get("target") and trace["attempts"]:
            req=trace["attempts"][0]["request"];process=req["process_policy"]
            target_box=message_box(process["target"])
            scene=SimpleNamespace(support_graph=SupportRelationGraph.build(
                [message_box(b) for b in row["root"]["world_cartons"]]),
                snapshot={"scene_fingerprint":trace["model"]["identity"]["scene_fingerprint"]},
                policy=SimpleNamespace(data={"task_population":{"face_modes":["front"]}}))
            # Existing entry uses this standoff; verify against recorded native
            # virtual pose and identity instead of treating it as run evidence.
            centered=simulation_contact_targets(scene,target_box,robot.base_transform[:3,3],.1)[0][0]
            suction={"suction_edge_margin_m":process["suction_edge_margin_m"]}
            target=eccentric_target(centered,frozen,row["option"]["world_y_offset_m"],suction)
            assert req["candidate_id"]=="single-contact:"+target.candidate_id
            detail=dict(target=target.evidence(),expected_contact={k:process[k] for k in
                ("geometrically_eligible_mask","commanded_active_mask")})
            detail["geometry_reconstructed_from_bound_native_request"]=True
            pending=[]
            domain=offset_intervals(centered,frozen,suction)
            for delta in candidate_deltas(domain):
                if delta == row["option"]["world_y_offset_m"]:continue
                preview=eccentric_target(centered,frozen,delta,suction)
                pending.append(dict(target=preview.evidence(),status="STATIC_GEOMETRY_ONLY_NOT_DISPATCHED",
                    eligible_count=sum(preview.predicted_eligible_mask)))
            detail["pending_target_geometry"]=pending
        task_id=next((c["task_id"] for c in calls if c.get("task_id")),None)
        targets[task_id]=dict(folder=row["folder"],option=row["option"],target=detail.get("target"),
            expected=detail.get("expected_contact"),actual=detail.get("terminal_contact"),
            contact_q=detail.get("contact_end_q_rad",detail.get("actual_end_q_rad")),
            receive_sync=detail.get("receive_sync_location"),current_run_complete=row["complete"],
            pending_target_geometry=detail.get("pending_target_geometry",[]))
        by={c.get("stage_id"):c for c in calls if c.get("stage_id")}
        for entry in trace["attempts"]:
            req=entry["request"]
            if req.get("op")!="plan" or req.get("pipeline_id")=="external_axis":continue
            call=by[req["stage_id"]];response=call["response"]
            # Bind all annotations to THIS native task. No old q is imported.
            targets[req["task_id"]].update(dict(folder=row["folder"],option=row["option"],
                target=detail.get("target"),expected=detail.get("expected_contact"),
                actual=detail.get("terminal_contact"),contact_q=detail.get("contact_end_q_rad",
                    detail.get("actual_end_q_rad")),receive_sync=detail.get("receive_sync_location"),
                current_run_complete=row["complete"]))
            display={k:req[k] for k in ("op","identity","task_id","stage_id","parent_stage_id","stage",
                "candidate_id","scene_fingerprint","world","q_start","q_goal","goal_pose","attachment",
                "pipeline_id","planner_id","purpose") if k in req}
            display["process_policy"]={"target_id":req["process_policy"]["target_id"]}
            compact={k:response[k] for k in ("request_id","task_id","stage_id","parent_stage_id",
                "status","reason","failure","solver_message","moveit_error_code") if k in response}
            q=req.get("q_goal")
            attempts.append(dict(request=display,response_identity=compact,
                display_label=f'{row["folder"]} delta={row["option"].get("world_y_offset_m",0):.5f} m · {response["status"]}',
                final_timed_path=entry.get("final_timed_path") or [],trace=None,external_axis=None,
                history_status="RECORDED_IN_RAW_ARCHIVE_NOT_EMBEDDED",
                fk_upper_arm=None if q is None else upper_arm(robot,q),
                failure_witness=compact if response["status"]!="SUCCESS" else None))
            logs.append(dict(op="plan",response=compact,round_trip_s=call["round_trip_s"]))
        logs.extend({k:c[k] for k in ("op","status","stage","candidate_id","q_start","goal_pose",
            "scene_fingerprint","attachment","response") if k in c} for c in calls if c.get("op")=="ik")
        # Retain an actual IK result even if integration failed before a motion
        # request. This is explicitly NOT a path or a collision-qualified goal.
        for c in calls:
            if c.get("op")!="ik" or not c.get("response",{}).get("q"):continue
            q=c["response"]["q"]
            parent=next(e["request"] for e in trace["attempts"] if e["request"]["stage_id"]==c["parent_stage_id"])
            assert parent["receiver_end_fingerprint"]==c["scene_fingerprint"]
            req=dict(op="ik",task_id=c["task_id"],stage_id=None,parent_stage_id=c["parent_stage_id"],
                stage=c["stage"],candidate_id=c["candidate_id"],scene_fingerprint=c["scene_fingerprint"],
                world=parent["receiver_end_world"],q_start=c["q_start"],q_goal=q,goal_pose=c["goal_pose"],
                attachment=None,pipeline_id=None,planner_id="IK_ONLY",process_policy={"target_id":c["target_id"]})
            response=dict(c["response"],task_id=c["task_id"],stage_id=None,parent_stage_id=c["parent_stage_id"])
            attempts.append(dict(request=req,response_identity=response,
                display_label=row["folder"]+" · 原生 IK 数值解 / 碰撞与接近路径尚未检查",
                final_timed_path=[],trace=None,external_axis=None,fk_upper_arm=upper_arm(robot,q),
                history_status="NO_MOTION_REQUEST_AFTER_INTEGRATION_FAILURE"))
            logs.append(dict(op="ik_result_inspection",response=response))
    if not attempts:raise ValueError("NO_CURRENT_RUN_NATIVE_REQUESTS")
    data=dict(label="c00 · 右端正面 +Y / 上臂 FK",raw=dict(run_id="eccentric-71070",
        source_commit=protocol["source_commit"],attempts=attempts),
        result=dict(complete=result["completed_count"]==1,target_id="carton_l07_c00"),
        model=model,request_log=logs,eccentric=dict(targets=targets,
            cups=build_m710_independent_cup_array().to_dict(),planning_wall_s=result["planning_wall_s"],
            all_attempts=result["attempts"],completed_count=result["completed_count"]),
        qualification_status="NOT_EVALUATED",physical_execution="NOT_RUN",
        execution_ready=False,continuous_swept_proof=False)
    vendor=ROOT/"tools/search_replay/vendor"
    html=(ROOT/"tools/search_replay/viewer.html").read_text(encoding="utf-8")
    html=html.replace("/*__VENDOR__*/",(vendor/"three.min.js").read_text()+"\n"+(vendor/"OrbitControls.js").read_text())
    html=html.replace("/*__DATA__*/",json.dumps(data,separators=(",",":"),allow_nan=False).replace("</","<\\/"))
    # Use the existing actual-q FK/rendering and native request controls.
    extension=(ROOT/"tools/search_replay/eccentric_overlay.js").read_text(encoding="utf-8")
    html=html.replace("</script></html>",extension+"\n</script></html>")
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(html,encoding="utf-8")
    print(json.dumps(dict(output=str(output),native_requests=len(attempts),target_tasks=len(targets))))


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("run",type=Path);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();export(a.run,a.output)
