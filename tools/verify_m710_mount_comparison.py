"""Summarize actual paired trials and verify complete native lineage."""
import argparse
from collections import Counter
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from plan_m710_mount_comparison import LAYOUTS, TARGETS, FLAGS, save
from verify_m710_top_layer_batch import verify


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("run",type=Path)
    a=p.parse_args()
    protocol=json.loads((a.run/"protocol.json").read_text())
    summary=json.loads((a.run/"summary.json").read_text())
    assert summary["status"]=="COMPARISON_COMPLETE"
    assert len(summary["outcomes"])==len(LAYOUTS)*len(TARGETS)==20
    rows=[];collision_policy=None
    expected_mounts={"baseline":[-1.325,.35,.6],"y_minus_100":[-1.325,.25,.6],
                     "x_minus_100":[-1.425,.35,.6],"xy_minus_100":[-1.425,.25,.6]}
    for target in TARGETS:
        paired_root=None
        for variant in LAYOUTS:
            run=a.run/variant/target
            checked=verify(run)
            r=json.loads((run/"seed-71070/result.json").read_text())
            first=next(x for x in r["attempts"] if "folder" in x)
            model=json.loads((run/"seed-71070"/first["folder"]/"search-trace.json").read_text())["model"]
            urdf=ET.fromstring(model["parameters"]["robot_description"])
            mount=next(j for j in urdf.findall("joint") if j.find("parent").get("link")=="world" and j.find("child").get("link")=="base_link")
            xyz=list(map(float,mount.find("origin").get("xyz").split()))
            assert max(abs(x-y) for x,y in zip(xyz,expected_mounts[variant]))<1e-12
            assert list(map(float,mount.find("origin").get("rpy").split()))==[0.]*3
            if collision_policy is None:collision_policy=model["collision_policy"]
            assert model["collision_policy"]==collision_policy
            if paired_root is None:paired_root=r["initial"]
            assert r["initial"]==paired_root
            counts=Counter();attempts=[];pairs=Counter();first_witness={}
            for attempt in r["attempts"]:
                if "folder" not in attempt:
                    attempts.append(attempt);continue
                folder=run/"seed-71070"/attempt["folder"]
                calls=json.loads((folder/"requests.json").read_text())
                connections=json.loads((folder/"connection-attempts.json").read_text())
                detail=json.loads((folder/"attempt.json").read_text()) if (folder/"attempt.json").exists() else {}
                local=Counter()
                for call in calls:
                    counts["native_requests"]+=1
                    success=call["status"]=="SUCCESS"
                    if call["op"]=="ik" and success:local["numerical_ik_success"]+=1
                    if call["op"]!="plan":continue
                    response=call.get("response",{})
                    endpoint=response.get("clearance",{}).get("counts",{}).get("endpoint",{})
                    if (call.get("q_goal") is not None and call.get("pipeline_id")!="external_axis"
                            and endpoint.get("queries",0)>=2 and endpoint.get("rejected",1)==0):
                        local["target_check_passed"]+=1
                    if success:
                        local["checked_lift" if call["planner_id"]=="COUPLED_LIFT" else call["stage"]]+=1
                    for witness in response.get("clearance",{}).get("rejected_examples",[]):
                        key=json.dumps([call["stage"],witness["pair"],witness.get("phase")])
                        pairs[key]+=1
                        first_witness.setdefault(key,dict(stage=call["stage"],stage_id=call["stage_id"],witness=witness))
                counts.update(local)
                masks=lambda source,key:sum(source[key]) if source and key in source else None
                attempts.append(dict(folder=attempt["folder"],complete=attempt["complete"],
                    wall_s=attempt["wall_s"],failure=attempt.get("failure"),option=attempt["option"],
                    counts=dict(local),synchronization=detail.get("receive_sync_location","NOT_REACHED"),
                    predicted_eligible_cups=masks(detail.get("expected_contact"),"geometrically_eligible_mask"),
                    commanded_cups=masks(detail.get("expected_contact"),"commanded_active_mask"),
                    actual_contact_cups=masks(detail.get("terminal_contact"),"actual_contact_mask"),
                    selected_configurations=[{k:c[k] for k in ("stage","attempt","q_goal","status","failure","upper_arm_preference") if k in c} for c in connections]))
            cycle=r["cycles"][0] if r["cycles"] else None
            rows.append(dict(variant=variant,target_id=target,complete=bool(cycle),
                native_model_mount_world_m=xyz,
                status=r["status"],wall_s=r["planning_wall_s"],counts=dict(counts),attempts=attempts,
                accepted=None if cycle is None else dict(
                    delta_m=cycle["target"].get("world_y_offset_m",0.),
                    receive_height_m=cycle["height_m"],option=cycle["option"],
                    receive_sync_location=cycle.get("receive_sync_location"),
                    temporary_height_m=cycle["option"].get("pre_approach_height_m",r["initial"]["height_m"]),
                    cups=sum(cycle["attachment"]["selection"]["actual_contact_mask"]),
                    actual_extraction_height_m=cycle["actual_extraction_height_m"]),
                collision_witness_groups=[dict(key=json.loads(k),recorded_example_count=n,example=first_witness[k]) for k,n in pairs.most_common()],
                verification=checked))
    report=dict(**FLAGS,source_commit=protocol["source_commit"],worker_sha256=protocol["worker_sha256"],
        rows=rows,by_layout={v:dict(complete=sum(r["complete"] for r in rows if r["variant"]==v),
            targets=len(TARGETS),wall_s=sum(r["wall_s"] for r in rows if r["variant"]==v)) for v in LAYOUTS},
        interpretation="PAIRED_FIVE_TARGET_SAMPLE_NOT_WHOLE_STACK_SUCCESS_RATE")
    save(a.run/"verification.json",report)
    print(json.dumps(report["by_layout"]))


if __name__=="__main__":main()
