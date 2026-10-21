"""Verify saved population evidence and optionally prepare the existing replay.

No IK/planning requests are made. Rejected candidates never change the replay's
committed root; archived native stages retain their original IDs and parents.
"""
import argparse
from collections import Counter
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, separators=(",", ":"))+"\n", encoding="utf-8")


def verify(run, replay=None):
    protocol = json.loads((run/"protocol.json").read_text())
    summary = json.loads((run/"summary.json").read_text())
    xy_top = protocol["scope"] == "XY_TOP_NEW_WORLD_40"
    fixture = xy_top and protocol.get("integration_fixture", False)
    assert summary["status"] in (("INTEGRATION_FIXTURE_FINISHED",) if fixture else
                                  ("COMPLETE_PLANNING_ONLY", "INCOMPLETE_RETAINED"))
    assert xy_top or protocol["scope"] == "ALL_REMAINING_FROM_ACCEPTED_C00"
    original = {b["id"]: b for b in protocol["original_cartons"]}
    initial, root = summary["initial"], summary["initial"]
    targets = set(protocol["scoped_target_ids"])
    assert targets == {b["id"] for b in initial["world_cartons"]}
    assert len(targets) == summary["target_count"] == (20 if fixture else 40 if xy_top else 36)
    assert len(initial["completed"]) == summary["historical_completed_count"] == (20 if fixture else 0 if xy_top else 4)
    assert len(original) == 40 and len(protocol["frozen"]["target_ids"]) == 40
    outcomes, attempts, cycles, replay_attempts = [], [], [], []
    counts = Counter()
    native_collision_policy = None
    if replay:
        replay.mkdir(parents=True, exist_ok=False)
        (replay/"seed-71070").mkdir()
        save(replay/"protocol.json", protocol)
    for outcome in summary["outcomes"]:
        name = outcome["target_id"]
        assert name in targets and outcome["root"] == root
        archive = run/outcome["evidence"]["archive"]
        prefix = archive.name.removesuffix(".tar.gz")
        with ExitStack() as stack:
            if archive.exists():
                assert hashlib.sha256(archive.read_bytes()).hexdigest() == outcome["evidence"]["sha256"]
                tar = stack.enter_context(tarfile.open(archive))
                read_bytes = lambda file: tar.extractfile(prefix+"/"+file).read()
            else:
                # Losslessly expanded offline bundle; check original per-file
                # digests below, without requiring the storage gzip container.
                expanded = (run/prefix).resolve()
                assert expanded.parent == run.resolve() and expanded.is_dir()
                def read_bytes(file):
                    path = (expanded/file).resolve()
                    assert path.is_relative_to(expanded)
                    return path.read_bytes()
            def raw(file):
                value = read_bytes(file)
                assert hashlib.sha256(value).hexdigest() == outcome["evidence"]["file_sha256"][file]
                return value

            def read(file):
                return json.loads(raw(file))

            result = read("result.json")
            assert result["initial"] == root
            assert result["completed_count"] in (0, 1)
            assert len(result["cycles"]) == result["completed_count"]
            task_counts = Counter()
            task_attempts = []
            for row in result["attempts"]:
                if "folder" not in row:
                    task_attempts.append(row)
                    continue
                folder = row["folder"]
                calls = read(folder+"/requests.json")
                connections = read(folder+"/connection-attempts.json")
                detail = read(folder+"/attempt.json") if folder+"/attempt.json" in outcome["evidence"]["file_sha256"] else {}
                trace = read(folder+"/search-trace.json")
                if xy_top:
                    model = trace['model']
                    urdf = ET.fromstring(model['parameters']['robot_description'])
                    mount = next(j for j in urdf.findall('joint') if
                        j.find('parent').get('link') == 'world' and j.find('child').get('link') == 'base_link')
                    xyz = list(map(float,mount.find('origin').get('xyz').split()))
                    assert max(abs(a-b) for a,b in zip(xyz,[-1.425,.25,.6])) < 1e-12
                    assert list(map(float,mount.find('origin').get('rpy').split())) == [0.,0.,0.]
                    if native_collision_policy is None:native_collision_policy = model['collision_policy']
                    assert model['collision_policy'] == native_collision_policy
                successful = [c for c in calls if c["status"] == "SUCCESS"]
                local = Counter(numerical_ik_success=sum(c["op"] == "ik" for c in successful))
                witnesses = []
                for call in calls:
                    counts["native_requests"] += 1
                    response = call.get("response", {})
                    if call["op"] != "plan":
                        continue
                    endpoint = response.get("clearance", {}).get("counts", {}).get("endpoint", {})
                    if (call.get("q_goal") is not None and call.get("pipeline_id") != "external_axis"
                            and endpoint.get("queries", 0) >= 2 and endpoint.get("rejected", 1) == 0):
                        local["target_check_passed"] += 1
                    if call["status"] == "SUCCESS":
                        key = "checked_lift" if call["planner_id"] == "COUPLED_LIFT" else call["stage"]
                        local[key] += 1
                    for witness in response.get("clearance", {}).get("rejected_examples", []):
                        witnesses.append(dict(stage=call["stage"], stage_id=call["stage_id"], witness=witness))
                task_counts.update(local)
                masks = lambda source, key: (sum(source[key]) if source and key in source else None)
                option = row["option"]
                task_attempts.append(dict(
                    folder=folder, complete=row["complete"], wall_s=row["wall_s"],
                    failure=row.get("failure"), template=option["template"],
                    delta_m=option.get("world_y_offset_m", 0.),
                    receive_height_m=option["height_m"],
                    temporary_height_m=option.get("pre_approach_height_m", root["height_m"]),
                    synchronization=detail.get("receive_sync_location", "NOT_REACHED"),
                    counts=dict(local),
                    predicted_eligible_cups=masks(detail.get("expected_contact"), "geometrically_eligible_mask"),
                    commanded_cups=masks(detail.get("expected_contact"), "commanded_active_mask"),
                    actual_contact_cups=masks(detail.get("terminal_contact"), "actual_contact_mask"),
                    selected_configurations=[{k: c[k] for k in (
                        "stage", "attempt", "q_goal", "status", "failure", "upper_arm_preference") if k in c}
                        for c in connections],
                    collision_witnesses=witnesses))
                if not row["complete"]:
                    continue
                assert result["completed_count"] == 1
                cycle = result["cycles"][0]
                assert cycle["batch_root"] == root and cycle["target_id"] == name
                eccentric = xy_top and name in protocol['eccentric_authorized_ids']
                if not eccentric:
                    assert cycle["target"].get("world_y_offset_m", 0.) == 0
                    assert cycle["target"].get("eccentric_authorization") is None
                low_top = xy_top and protocol['frozen']['heights'][name]['Hmin_world_m'] < protocol['low_top_minimum_m']-1e-9
                if low_top:
                    assert cycle['option']['template'] == 'TOP_FIXED_MINIMUM_RECEIVER'
                    assert cycle['target']['face_role'] == 'TOP'
                    assert cycle['stationary_low_receiver']['height_m'] == .6
                    assert cycle['stationary_low_receiver']['throughout_contact_lift_transfer_release']
                requests = {t["request"]["stage_id"]: t["request"] for t in trace["attempts"]}
                responses = {c["stage_id"]: c["response"] for c in successful if c["op"] == "plan"}
                q, height, parent = root["q_rad"], root["height_m"], ""
                extraction_started = False
                extraction_count = 0
                for stage in cycle["stages"]:
                    req, resp = requests[stage["stage_id"]], responses[stage["stage_id"]]
                    assert req["q_start"] == q and req["parent_stage_id"] == parent
                    assert not req["pair_profile_enabled"] and req["planning_mode"] == "cold_from_scratch"
                    assert req["start_velocity"] == [0.]*6 and resp["generated_during_task"] and resp["mtc_generation"]
                    if stage["generation_source"] == "NATIVE_EXTERNAL_AXIS":
                        assert not extraction_started and stage["stage"] in ("pregrasp", "contact")
                        axis = resp["external_axis"]
                        assert axis["from_height_m"] == height and resp["terminal_q"] == q
                        assert all(c["passed"] for c in axis["checks"])
                        assert axis["maximum_height_step_m"] <= .010000001 and not axis["continuous_swept_proof"]
                        height = axis["to_height_m"]
                        if low_top:assert height == .6 and stage['stage'] == 'pregrasp'
                        counts["checked_lift_samples"] += len(axis["checks"])
                    elif stage["generation_source"] == "SEMANTIC_EVENT":
                        drop = resp["nominal_release"]
                        assert drop["same_pose_detachment"] and drop["target_in_world_once"] and drop["first_contact"]
                        assert all(c["passed"] for c in drop["checks"]) and resp["terminal_q"] == q
                        counts["nominal_drop_samples"] += len(drop["checks"])
                    else:
                        if stage["stage"] == "extraction":
                            extraction_count += 1
                            extraction_started = True
                            assert abs(height-cycle["actual_extraction_height_m"]) < 1e-8
                            if low_top:
                                assert height == .6
                                if extraction_count == 1:
                                    assert req['ownership_transition'] == 'ATTACH'
                                else:
                                    assert req.get('ownership_transition') is None
                                    policy = req['process_policy']
                                    assert not policy.get('initial_proximity') and not policy.get('support_names')
                        assert stage["points"][0]["q"] == q and stage["points"][-1]["t"] > 0
                        assert max(abs(v) for v in stage["points"][-1]["v"]) <= 1e-8
                        q = stage["points"][-1]["q"]
                    parent = stage["stage_id"]
                    counts["accepted_native_stages"] += 1
                selection = cycle["attachment"]["selection"]
                assert all(len(selection[k]) == 72 for k in (
                    "actual_contact_mask", "commanded_active_mask", "geometrically_eligible_mask"))
                assert selection["actual_contact_mask"] == selection["commanded_active_mask"]
                assert any(selection["actual_contact_mask"])
                assert cycle["attachment"]["pose_continuity_max_abs"] < 1e-8
                assert cycle["third_sync_disabled"] and cycle["nominal_release"]["passed"]
                assert cycle["separation"]["passed"] and cycle["separation"]["response"]["status"] == "SUCCESS"
                events = [e["event_type"] for e in cycle["events"]]
                if low_top:
                    assert extraction_count == 2 and events.count('ATTACH') == 1
                    lift = cycle['stationary_low_receiver']['top_lift']
                    assert lift['sample_check']['passed']
                    assert lift['terminal_check']['status'] == 'SUCCESS'
                    assert lift['terminal_check']['free_space_rules_restored']
                    assert lift['geometry']['direction_world'] == [0.,0.,1.]
                    assert cycle['extraction_geometry']['direction_world'] == [-1.,0.,0.]
                    assert height == .6
                required = ["RELEASE", "NOMINAL_FIRST_CONTACT", "SAFE_TOOL_SEPARATION", "IDEAL_RECEIVER_DRAIN"]
                assert all(events.count(e) == 1 for e in required)
                assert [events.index(e) for e in required] == sorted(events.index(e) for e in required)
                assert q == cycle["end_q_rad"] and height == cycle["height_m"]
                root = dict(q_rad=q, velocity_rad_s=[0.]*6, height_m=height,
                            completed=[*root["completed"], name],
                            world_cartons=[b for b in root["world_cartons"] if b["id"] != name],
                            predecessor_final_state_receipt=cycle["final_state_receipt"])
                if replay:
                    destination = replay/"seed-71070"/(prefix+"-"+folder)
                    destination.mkdir()
                    for f in ("search-trace.json", "requests.json"):
                        (destination/f).write_bytes(raw(folder+"/"+f))
                    replay_attempts.append(dict(row, folder=destination.name))
                    cycles.append(cycle)
            assert root == result["terminal"] == outcome["terminal"]
            assert {b["id"]: b for b in root["world_cartons"]} == {
                n: b for n, b in original.items() if n not in root["completed"]}
            counts.update(task_counts)
            outcomes.append(dict(target_id=name, complete=bool(result["completed_count"]),
                                 planning_wall_s=result["planning_wall_s"],
                                 counts=dict(task_counts), attempts=task_attempts))
    assert root == summary["terminal"]
    attempted = {o["target_id"] for o in outcomes}
    blocked = {b["target_id"] for b in summary["blocked"]}
    assert len(attempted) == len(outcomes) and not attempted & blocked
    if not fixture:assert attempted | blocked == targets
    # A box left under a failed carton must remain an actual support, not a
    # made-up failure count or a box silently removed to expose its successors.
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
    from unloading_sim.coupled_batch import message_box
    from unloading_sim.support import SupportRelationGraph
    support = SupportRelationGraph.build(tuple(message_box(b) for b in root["world_cartons"]))
    for row in summary["blocked"]:
        assert row["supported_cartons"] == sorted(support.supports[row["target_id"]])
        assert row["supported_cartons"] and not row["native_search_performed"]
    completed = sum(o["complete"] for o in outcomes)
    assert completed == summary["completed_count"]
    assert len(root["world_cartons"]) == len(targets)-completed
    assert all(summary[k] == completed for k in (
        "release_count", "nominal_first_contact_count", "separation_count", "ideal_drain_count"))
    assert summary["physical_execution"] == "NOT_RUN" and not summary["execution_ready"]
    assert summary["qualification_status"] == "NOT_EVALUATED" and not summary["continuous_swept_proof"]
    if replay:
        save(replay/"seed-71070/result.json", dict(summary, cycles=cycles, attempts=replay_attempts,
                                                 lower_layer_preserved=True))
    verification = dict(status="PASS_SAVED_NATIVE_LINEAGE_POPULATION_AND_FAILURE_RETENTION",
                        source_commit=protocol["source_commit"], completed_count=completed,
                        target_count=len(targets), attempted_count=len(outcomes), blocked_count=len(blocked),
                        planning_wall_s=summary["planning_wall_s"], native_task_wall_s=summary.get("native_task_wall_s",sum(o['planning_wall_s'] for o in outcomes)),
                        integration_fixture=fixture,
                        counts=dict(counts), outcomes=outcomes, blocked=summary["blocked"],
                        physical_execution="NOT_RUN", execution_ready=False,
                        qualification_status="NOT_EVALUATED", continuous_swept_proof=False)
    save(run/"verification.json", verification)
    return verification


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("--prepare-replay", type=Path)
    args = p.parse_args()
    r = verify(args.run, args.prepare_replay)
    print(json.dumps({k: r[k] for k in ("status", "completed_count", "target_count",
                                       "attempted_count", "blocked_count", "counts")}))
