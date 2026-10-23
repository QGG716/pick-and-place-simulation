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


def expected_population_mount(protocol):
    """Bind saved native geometry to one explicit approved population layout."""
    mounts = {'xy_minus_100': [-1.425, .25, .6],
              'x_minus_300_y_minus_100': [-1.625, .25, .6],
              'x_minus_400_y_minus_100': [-1.725, .25, .6],
              'x_minus_500_y_minus_100': [-1.825, .25, .6]}
    variant = protocol.get('mounting_variant', 'xy_minus_100')
    if variant not in mounts:
        raise ValueError('UNKNOWN_POPULATION_MOUNT')
    expected = 'configs/workcells/m710id70_unloading_layout_v1_mount_'+variant+'.yaml'
    if protocol.get('scope') in ('XY_TOP_MIN250_CONTINUATION','XY_TOP_SELECTED_LAYER_CONTINUATION'):
        if variant!='x_minus_500_y_minus_100':raise ValueError('MIN250_MOUNT_CHANGED')
        expected=expected.removesuffix('.yaml')+'_min250.yaml'
    if protocol.get('scope') in ('XY_TOP_LOWPROFILE250_CONTINUATION','XY_TOP_LOWPROFILE250_BOTTOM_TWO_TOP','XY_TOP_LOWPROFILE250_L01_TOOL_CHASSIS','XY_TOP_L01_SELECTED_SIDE_TOP_HMAX','XY_TOP_L01_C01_FINITE_SIDE_CHANNEL_HMAX'):
        if variant!='x_minus_500_y_minus_100':raise ValueError('LOWPROFILE_MOUNT_CHANGED')
        expected=expected.removesuffix('.yaml')+'_min250_lowprofile.yaml'
    extension=protocol.get('tool_connector_extension_m',0.)
    if extension:
        if extension!=.1 or protocol['scope']!='XY_TOP_L01_C01_FINITE_SIDE_CHANNEL_HMAX':
            raise ValueError('UNAPPROVED_POPULATION_TOOL_EXTENSION')
        expected=expected.removesuffix('.yaml')+'_tool_extension100.yaml'
    if protocol['active_layout_file'] != expected:
        raise ValueError('POPULATION_LAYOUT_VARIANT_MISMATCH')
    return mounts[variant]


def verify(run, replay=None, *, retained_prefix=False):
    protocol = json.loads((run/"protocol.json").read_text())
    summary = json.loads((run/"summary.json").read_text())
    all_rates = protocol['scope']=='XY_TOP_L05_ALL_JOINT_RATES_RESTART'
    diagnostic = all_rates or protocol["scope"] == "XY_TOP_L05_J4_VELOCITY_RESTART"
    continuation=protocol['scope']=='XY_TOP_MIN250_CONTINUATION'
    bottom_two_top=protocol['scope']=='XY_TOP_LOWPROFILE250_BOTTOM_TWO_TOP'
    finite_c01=protocol['scope']=='XY_TOP_L01_C01_FINITE_SIDE_CHANNEL_HMAX'
    selected_routes=finite_c01 or protocol['scope']=='XY_TOP_L01_SELECTED_SIDE_TOP_HMAX'
    l01_tool=selected_routes or protocol['scope']=='XY_TOP_LOWPROFILE250_L01_TOOL_CHASSIS'
    layer_top=bottom_two_top or protocol['scope']=='XY_TOP_SELECTED_LAYER_CONTINUATION'
    lowprofile=l01_tool or bottom_two_top or protocol['scope']=='XY_TOP_LOWPROFILE250_CONTINUATION'
    repair=continuation and protocol.get('integration_repair_restart',False)
    xy_top = lowprofile or layer_top or continuation or diagnostic or protocol["scope"] == "XY_TOP_NEW_WORLD_40"
    expected_mount = expected_population_mount(protocol) if xy_top else None
    fixture = xy_top and protocol.get("integration_fixture", False)
    stopped=retained_prefix and continuation and summary['status']=='INTEGRATION_ERROR_RETAINED'
    assert stopped or summary["status"] in (("INTEGRATION_FIXTURE_FINISHED",) if fixture else
                                  ("COMPLETE_PLANNING_ONLY", "INCOMPLETE_RETAINED"))
    assert xy_top or protocol["scope"] == "ALL_REMAINING_FROM_ACCEPTED_C00"
    original = {b["id"]: b for b in protocol["original_cartons"]}
    initial, root = summary["initial"], summary["initial"]
    targets = set(protocol["scoped_target_ids"])
    initial_ids={b['id'] for b in initial['world_cartons']}
    assert targets <= initial_ids
    assert initial_ids-targets == (set(protocol['out_of_scope_target_ids']) if l01_tool else set())
    assert len(targets) == summary["target_count"] == (1 if finite_c01 else 3 if selected_routes else 5 if l01_tool else 10 if bottom_two_top else 12 if layer_top or lowprofile else 17 if repair else 20 if continuation else 29 if diagnostic else 20 if fixture else 40 if xy_top else 36)
    assert len(initial["completed"]) == summary["historical_completed_count"] == (32 if selected_routes else 30 if bottom_two_top or l01_tool else 28 if layer_top or lowprofile else 23 if repair else 20 if continuation else 11 if diagnostic else 20 if fixture else 0 if xy_top else 4)
    if lowprofile:
        from unloading_sim.collision_policy import SimulationCollisionPolicy
        predecessor=json.loads((run/'predecessor.json').read_text())
        assert initial==protocol['initial_root']==predecessor['new_native_root']
        assert initial==dict(predecessor['archived_root'],predecessor_final_state_receipt=None)
        assert protocol['low_definition']=='FROZEN_BOX_TOP_STRICTLY_BELOW_MINIMUM'
        assert protocol['low_top_minimum_m']==.25 and protocol['all_joint_rate_limits_retained']
        assert protocol['scissor_profile']=='low_profile_250_v1'
        if not bottom_two_top:assert 'layer_top_permission' not in protocol
        assert summary['outcomes'][0]['target_id']==('carton_l01_c01' if selected_routes else 'carton_l01_c02' if bottom_two_top or l01_tool else 'carton_l02_c01')
        policy=SimulationCollisionPolicy.from_mapping(protocol['collision_policy'])
        assert policy.robot_mount_contact_exemption
        assert protocol['robot_mount_allowed_pairs']==[list(p) for p in sorted(policy.robot_mount_pairs())]
        if l01_tool:
            from l01_tool_chassis import frozen_l01_scope
            from unloading_sim.coupled_batch import message_box
            assert protocol['tool_chassis_permission']==frozen_l01_scope(tuple(message_box(b) for b in protocol['original_cartons']))
            assert set(policy.tool_chassis_contact_target_ids)==set(protocol['tool_chassis_permission']['allowed_target_ids'])
            assert targets <= set(policy.tool_chassis_contact_target_ids)
            assert len(initial_ids)==(8 if selected_routes else 10) and len(initial_ids-targets)==(7 if finite_c01 else 5)
        if selected_routes:
            from unloading_sim.l01_selected_routes import selected_routes_permission
            from unloading_sim.carton_population import FrozenCartonPopulation
            routes_permission=selected_routes_permission(FrozenCartonPopulation.capture(tuple(message_box(b) for b in protocol['original_cartons'])),finite_c01_channel=finite_c01)
            assert routes_permission==protocol['selected_routes_permission']
            assert set(routes_permission['routes'])==targets
            assert [o['target_id'] for o in summary['outcomes']]==routes_permission['dispatch_order']
            assert all(a['option']['template']==routes_permission['routes'][o['target_id']]['template']
                for o in summary['outcomes'] for a in o['attempts'] if 'option' in a)
    if layer_top:
        predecessor=json.loads((run/'predecessor.json').read_text())
        assert initial==protocol['initial_root']==predecessor['new_native_root']
        assert initial==dict(predecessor['archived_root'],predecessor_final_state_receipt=None)
        assert protocol['all_joint_rate_limits_retained'] and protocol['low_top_minimum_m']==.25
        assert summary['outcomes'][0]['target_id']==('carton_l01_c02' if bottom_two_top else 'carton_l02_c01')
        permission=protocol['layer_top_permission']
        anchor=protocol['frozen']['heights'][permission['anchor_id']]
        assert permission['anchor_box_identity']==anchor['start_box_identity']
        assert permission['ceiling_world_m']==anchor['z_b_m']+anchor['h_m']
        assert permission['allowed_target_ids']==sorted(n for n,h in protocol['frozen']['heights'].items()
            if h['z_b_m']+h['h_m']<=permission['ceiling_world_m']+1e-9)
        if bottom_two_top:
            assert protocol['layer_top_only'] and initial['height_m']==.25
            assert abs(permission['ceiling_world_m']-.6)<1e-9
            assert set(permission['allowed_target_ids'])==targets
            assert all(a['option']['template']=='TOP_FIXED_CURRENT_RECEIVER'
                       for o in summary['outcomes'] for a in o['attempts'] if 'option' in a)
    if continuation:
        predecessor=json.loads((run/'predecessor.json').read_text())
        retained=json.loads((run/'restart-predecessor.json').read_text()) if repair else predecessor
        assert initial==retained['new_native_root']
        assert initial==protocol['initial_root']
        assert initial==dict(retained['archived_root'],predecessor_final_state_receipt=None)
        assert len(predecessor['outcomes'])==25 and sum(o['completed_count'] for o in predecessor['outcomes'])==20
        assert protocol['predecessor_summary_sha256']==predecessor['summary_sha256']
        assert protocol['low_definition']=='FROZEN_BOX_TOP_STRICTLY_BELOW_MINIMUM'
        assert protocol['low_top_minimum_m']==.25 and protocol['all_joint_rate_limits_retained']
        assert summary['outcomes'][0]['target_id']=='carton_l03_c02'
    if diagnostic:
        policy=protocol['planning_diagnostic']
        assert policy==summary['planning_diagnostic']
        assert policy['target_id']=='carton_l05_c03'
        if all_rates:
            assert policy['joint']=='ALL_SIX' and policy['ignored_check']=='PILZ_ALL_JOINT_VELOCITY_ACCELERATION_DECELERATION'
            assert not policy['acceleration_deceleration_retained'] and not policy['other_joints_retained']
            assert policy['ik_and_position_limits_retained']
        else:
            assert policy['joint']=='J4' and policy['ignored_check']=='PILZ_JOINT_VELOCITY_ONLY'
            assert policy['acceleration_deceleration_retained'] and policy['other_joints_retained']
        assert policy['position_collision_and_process_checks_retained'] and policy['physical_execution']=='NOT_RUN'
        predecessor=json.loads((run/'predecessor.json').read_text())
        assert initial==predecessor['new_native_root']
        assert initial==dict(predecessor['archived_root'],predecessor_final_state_receipt=None)
        assert len(predecessor['prefix'])==11
        assert initial['completed']==[p['target_id'] for p in predecessor['prefix']]
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
        if stopped and outcome.get('status')=='INTEGRATION_ERROR_RETAINED':
            assert outcome is summary['outcomes'][-1] and 'evidence' not in outcome
            folder=(run/outcome['folder']).resolve()
            assert folder.is_relative_to(run.resolve())
            partial=json.loads((folder/'result.json').read_text())
            assert partial['initial']==root==partial['terminal']
            assert partial['completed_count']==0 and not partial['cycles']
            assert 'RECEIVE_MECHANISM_WORLD_CHANGED' in (folder/'integration-exception.txt').read_text()
            hashes={str(p.relative_to(folder)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in folder.rglob('*') if p.is_file()}
            outcomes.append(dict(target_id=name,complete=False,status='INTEGRATION_ERROR_RETAINED',
                counts={},attempts=partial['attempts'],planning_wall_s=partial['planning_wall_s'],
                uncommitted_failure_files_sha256=hashes))
            continue
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
            if diagnostic:
                enabled=name=='carton_l05_c03'
                assert outcome.get('motion_limit_diagnostic_enabled',outcome.get('j4_velocity_diagnostic_enabled'))==enabled
                dispatch=read('dispatch.json')
                assert dispatch.get('motion_limit_diagnostic_enabled',dispatch.get('j4_velocity_diagnostic_enabled'))==enabled
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
                if lowprofile:
                    for item in trace['attempts']:
                        req=item['request']
                        if 'clearance_policy' not in req:continue
                        assert req['clearance_policy']['source_policy']==protocol['collision_policy']
                        assert set(map(tuple,protocol['robot_mount_allowed_pairs'])) <= set(map(tuple,req['allowed_pairs']))
                        if l01_tool:
                            cp=req['clearance_policy']
                            assert req['planning_only'] and req['planning_mode']=='cold_from_scratch'
                            assert cp['target_id']==name
                            expected=policy.tool_chassis_pairs(name,cp['tool_links'])|policy.robot_mount_pairs()
                            assert {tuple(p) for p in req['allowed_pairs'] if 'chassis' in p}==expected
                if diagnostic:
                    marker='M710_DIAGNOSTIC_JOINT_RATE_IGNORED' if all_rates else 'M710_DIAGNOSTIC_J4_VELOCITY_IGNORED'
                    markers=raw(folder+'/worker.log').decode(errors='replace').count(marker)
                    if name!='carton_l05_c03':assert markers==0
                    counts['diagnostic_joint_rate_bypasses' if all_rates else 'diagnostic_j4_velocity_bypasses']+=markers
                if xy_top:
                    model = trace['model']
                    urdf = ET.fromstring(model['parameters']['robot_description'])
                    mount = next(j for j in urdf.findall('joint') if
                        j.find('parent').get('link') == 'world' and j.find('child').get('link') == 'base_link')
                    xyz = list(map(float,mount.find('origin').get('xyz').split()))
                    assert max(abs(a-b) for a,b in zip(xyz,expected_mount)) < 1e-12
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
                height_record=protocol['frozen']['heights'][name]
                low_top = xy_top and (height_record['z_b_m']+height_record['h_m'] < protocol['low_top_minimum_m']
                    if continuation or lowprofile else height_record['Hmin_world_m'] < protocol['low_top_minimum_m']-1e-9)
                top_height=protocol['low_top_minimum_m']
                if layer_top:
                    low_top=cycle['option']['template']=='TOP_FIXED_CURRENT_RECEIVER'
                    top_height=root['height_m']
                    if low_top:
                        assert name in permission['allowed_target_ids']
                        assert cycle['option']['layer_top_permission']==permission
                        assert cycle['option']['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]]
                if selected_routes:
                    route=routes_permission['routes'][name]
                    assert cycle['option']['selected_routes_permission']==routes_permission
                    assert cycle['option']['template']==route['template']
                    assert cycle['option']['segment_directions_world']==route['segment_directions_world']
                    assert cycle['height_m']==root['height_m']==route['height_m']
                    assert cycle['target']['geometric_face_id']==route['geometric_face_id']
                    low_top=route['template']=='SELECTED_TOP_HMAX';top_height=route['height_m']
                    if finite_c01:
                        assert cycle['receive_policy_check']['nonjunction_side_route']['passed']
                        assert cycle['side_turn_policy_check']['passed']
                if low_top:
                    assert cycle['option']['template'] == ('SELECTED_TOP_HMAX' if selected_routes else 'TOP_FIXED_CURRENT_RECEIVER' if layer_top else 'TOP_FIXED_MINIMUM_RECEIVER')
                    assert cycle['target']['face_role'] == 'TOP'
                    assert cycle['stationary_low_receiver']['height_m'] == top_height
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
                        if low_top:assert height == top_height and stage['stage'] == 'pregrasp'
                        if (layer_top and low_top) or selected_routes:assert axis['from_height_m']==axis['to_height_m']
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
                                assert height == top_height
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
                    assert height == top_height
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
    if not fixture and not stopped:assert attempted | blocked == targets
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
    assert len(root["world_cartons"]) == len(initial_ids)-completed
    assert all(summary[k] == completed for k in (
        "release_count", "nominal_first_contact_count", "separation_count", "ideal_drain_count"))
    assert summary["physical_execution"] == "NOT_RUN" and not summary["execution_ready"]
    assert summary["qualification_status"] == "NOT_EVALUATED" and not summary["continuous_swept_proof"]
    if replay:
        save(replay/"seed-71070/result.json", dict(summary, cycles=cycles, attempts=replay_attempts,
                                                 lower_layer_preserved=True))
    verification = dict(status=("PASS_SAVED_NATIVE_PREFIX_INTEGRATION_ERROR_RETAINED" if stopped else
                        "PASS_SAVED_NATIVE_LINEAGE_POPULATION_AND_FAILURE_RETENTION"),
                        source_commit=protocol["source_commit"], completed_count=completed,
                        target_count=len(targets), attempted_count=len(outcomes), blocked_count=len(blocked),
                        planning_wall_s=summary["planning_wall_s"], native_task_wall_s=summary.get("native_task_wall_s",sum(o['planning_wall_s'] for o in outcomes)),
                        integration_fixture=fixture,
                        counts=dict(counts), outcomes=outcomes, blocked=summary["blocked"],
                        physical_execution="NOT_RUN", execution_ready=False,
                        qualification_status="NOT_EVALUATED", continuous_swept_proof=False)
    if diagnostic:verification.update(planning_diagnostic=protocol['planning_diagnostic'],historical_completed_count=11)
    if continuation:verification.update(historical_completed_count=summary['historical_completed_count'],low_definition=protocol['low_definition'],
        requested_minimum_surface_m=.25,predecessor_summary_sha256=protocol['predecessor_summary_sha256'])
    if stopped:verification.update(unsearched_after_error=sorted(targets-attempted-blocked),
        completed_prefix_only=True,terminal=summary['terminal'],
        summary_sha256=hashlib.sha256((run/'summary.json').read_bytes()).hexdigest())
    save(run/("prefix-verification.json" if stopped else "verification.json"), verification)
    return verification


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("--prepare-replay", type=Path)
    p.add_argument("--retained-prefix",action='store_true')
    args = p.parse_args()
    r = verify(args.run, args.prepare_replay,retained_prefix=args.retained_prefix)
    print(json.dumps({k: r[k] for k in ("status", "completed_count", "target_count",
                                       "attempted_count", "blocked_count", "counts")}))
