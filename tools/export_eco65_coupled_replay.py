"""Export a self-contained ECO65 coupled-scene preview or current-run cycle replay."""
from __future__ import annotations

import argparse
from copy import deepcopy
import gzip
import hashlib
import json
import re
from pathlib import Path
import time
import zipfile

import numpy as np

from export_eco65_search_replay import export_model
from export_m710_search_replay import validate_attempt
from unloading_sim.eco65.model import ROOT, load_json, robot_for
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.coupled_scene import coupled_conveyor_state, receiving_height_candidates
from unloading_sim.eco65.collision_policy import cup_neighbor_pairs


SHARED_KEYS = {'source_policy','collision_policy','robot_mast_exemption',
               'process_geometry','task_contract','identity','camera_mount',
               'contours','fixed_attachment_interfaces'}


def reuse_runtime_values(value, cache):
    """Share identical decoded records before retaining an entire large run."""
    if isinstance(value,dict):
        for key,child in list(value.items()):
            if key in SHARED_KEYS | {'world'} and isinstance(child,(dict,list)):
                encoded=json.dumps(child,sort_keys=True,separators=(',',':'),allow_nan=False)
                ref=hashlib.sha256(encoded.encode()).hexdigest()
                if ref in cache:
                    value[key]=cache[ref]
                else:
                    cache[ref]=child
                    reuse_runtime_values(child,cache)
            else:reuse_runtime_values(child,cache)
    elif isinstance(value,list):
        for child in value:reuse_runtime_values(child,cache)


def policy_display(request=None, result=None, *, configured_policy=None):
    """Explain the captured request scope, never infer observed carton contact."""
    source = (configured_policy if request is None else
              request.get('clearance_policy', {}).get('source_policy',
                  request.get('process_policy', {}).get('collision_policy')))
    source = source or {}
    descriptor = source.get('extraction_carton_contact')
    display = dict(policy_schema=source.get('schema'), policy_mode=source.get('mode'),
                   extraction_carton_contact=deepcopy(descriptor),
                   source='FROZEN_MODEL_CONFIGURATION' if request is None else 'ACTUAL_NATIVE_REQUEST_POLICY',
                   request_scope_applies=None, permitted_pairs=[], native_evidence=None,
                   contact_observation='NO_PHYSICAL_CONTACT_OBSERVATION',
                   physical_execution='NOT_RUN', execution_ready=False)
    cup_rule = source.get('cup_neighbor_contact')
    display.update(cup_neighbor_contact=deepcopy(cup_rule),cup_neighbor_pairs=[],
                   cup_neighbor_native_evidence=None)
    if request is not None and cup_rule is not None:
        original = set(cup_rule['original_carton_ids'])
        display['cup_neighbor_pairs'] = cup_neighbor_pairs(source,
            continuous_task_mode=request.get('continuous_task_mode'),
            target_id=request.get('process_policy',{}).get('target_id'),
            world_carton_ids=[b['id'] for b in request.get('world',[]) if b['id'] in original],
            completed_ids=request.get('completed_ids',[]))
        for key in ('process_policy','process'):
            evidence = ((result or {}).get(key) or {}).get('cup_neighbor_contact')
            if isinstance(evidence,dict) and 'pairs' in evidence:
                display['cup_neighbor_native_evidence']=deepcopy(evidence)
                break
    if request is None or descriptor is None:
        return display
    process = request.get('process_policy', {})
    attached = request.get('attachment')
    target = process.get('target_id')
    applies = (descriptor.get('schema') == 'eco65_attached_target_stack_exemption_v1'
               and descriptor.get('scope') == 'ATTACHED_TARGET_VS_REMAINING_ORIGINAL_CARTONS_ONLY'
               and descriptor.get('allow_intersection') is True
               and request.get('continuous_task_mode') == descriptor.get('continuous_task_mode')
               and request.get('stage') == descriptor.get('stage')
               and attached is not None and attached.get('id') == target)
    remaining = set(process.get('stack_carton_ids', []))-set(request.get('completed_ids', []))
    world = {o['id'] for o in request.get('world', [])}
    neighbors = sorted(set(descriptor.get('original_carton_ids', [])) & remaining & world-{target})
    display.update(stage=request.get('stage'), operation=request.get('op'),
                   ik_collision_checked=(result or {}).get('ik_collision_checked'),
                   request_scope_applies=applies,
                   permitted_pairs=[[target, neighbor] for neighbor in neighbors] if applies else [])
    for key in ('process_policy', 'process'):
        evidence = ((result or {}).get(key) or {}).get('extraction_carton_contact')
        if isinstance(evidence, dict) and 'active' in evidence and 'pairs' in evidence:
            display['native_evidence'] = deepcopy(evidence)
            break
    return display


def read_trace(result, folder):
    reference = result.get('search_trace_ref')
    if not reference:
        return None
    path = Path(reference['path'])
    if not path.is_absolute():
        path = ROOT / path
    if not path.exists():
        path = folder / 'search_traces' / path.name
    if hashlib.sha256(path.read_bytes()).hexdigest() != reference['sha256']:
        raise ValueError('TRACE_ARCHIVE_CHANGED')
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return json.load(stream)


def transition_failures(request, result):
    """Use the checked after/sample world for a failed stopped-axis transition."""
    transition = request.get('transition', {})
    checks = []
    if result.get('after_check') and transition.get('after_request'):
        checks.append(('AFTER_STATE', transition['after_request'], result['after_check']))
    samples = transition.get('sample_requests', [])
    for row in result.get('sample_checks', []):
        checks.append(('SAMPLE_'+str(row['index']), samples[row['index']], row['check']))
    failures = []
    for label, req, check in checks:
        witness = check.get('failure')
        if check.get('status') == 'SUCCESS' or not isinstance(witness, dict):
            continue
        process = req.get('process_policy', {})
        state = dict(q=req['q_start'],world=req['world'],attachment=req.get('attachment'),
            active_ids=process.get('active_ids', []),target_id=process.get('target_id'),
            target_face=process.get('target_face'),height_m=req.get('mechanism_state',{}).get('height_m'),
            business_interval=req.get('business_interval'),completed_ids=req.get('completed_ids',[]))
        failures.append(dict(q=req['q_start'],witness=witness,state=state,
            source='SAME_NATIVE_TRANSITION_'+label))
    return failures


def attempts_from_run(folder, records):
    """Use actual requests, responses and same-call search traces, including failures."""
    adopted = {r['record_id'] for r in records}
    path = folder / 'native_calls.jsonl.gz'
    if not path.exists():
        return []
    attempts = []
    shared = {}
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            reuse_runtime_values(row,shared)
            req = row['request']
            if req['op'] == 'init':
                continue
            result = row.get('result', {})
            if 'q_start' not in req or ('world' not in req and req['op'] not in ('ik', 'ik_batch')):
                continue
            trace = read_trace(result, folder)
            if trace:reuse_runtime_values(trace,shared)
            if trace:
                captured = trace['input']
                for key in ('identity', 'task_id', 'stage_id', 'parent_stage_id',
                            'q_start', 'q_goal', 'world', 'attachment'):
                    if key in req and captured.get(key) != req[key]:
                        raise ValueError('TRACE_REQUEST_CONTEXT_MISMATCH:'+key)
                validate_attempt(dict(trace=trace, response_identity=result))
            ik_results = (result.get('results', []) if req['op'] == 'ik_batch'
                          else [result] if req['op'] == 'ik' else [])
            candidates = [dict(q=r['q'], status=r.get('status'),
                              seed_index=r.get('seed_index'),
                              ik_collision_checked=r.get('ik_collision_checked'),
                              position_error_m=r.get('position_error_m'),
                              orientation_error_rad=r.get('orientation_error_rad'))
                          for r in ik_results if 'q' in r]
            witnesses = transition_failures(req, result)
            for witness in [result.get('failure'), (result.get('basic_path_check') or {}).get('failure'),
                            *(result.get('clearance') or {}).get('rejected_examples', [])]:
                if isinstance(witness, dict) and witness.get('q_rad'):
                    witnesses.append(dict(q=witness['q_rad'], witness=witness))
            process_failure = (result.get('process') or {}).get('last_failure')
            if req['op'] == 'check_state_offline' and isinstance(process_failure, dict):
                witnesses.append(dict(q=req['q_start'], witness=process_failure,
                                      source='SAME_SINGLE_STATE_REQUEST'))
            if isinstance(result.get('failure'), dict) and not result['failure'].get('q_rad'):
                endpoint = ('q_start' if str(result.get('status')).startswith('INVALID_START_')
                            else 'q_goal' if str(result.get('status')).startswith('INVALID_GOAL_') else None)
                if endpoint and endpoint in req:
                    witnesses.append(dict(q=req[endpoint], witness=result['failure'],
                                          source='SAME_NATIVE_ENDPOINT_CHECK'))
            attempts.append(dict(
                request=req, response=result, trace=trace,
                adopted=req.get('stage_id') in adopted,
                round_trip_s=row.get('wall_s', row.get('round_trip_s')),
                error=row.get('error'), native_path=result.get('points', []),
                ik_candidates=candidates, failure_samples=witnesses,
                display_policy=policy_display(req, result),
                world_context_status='CAPTURED' if 'world' in req else 'NOT_CAPTURED',
                trace_status=trace.get('status', 'CAPTURED') if trace else
                    ('NOT_CAPTURED' if req.get('pipeline_id') == 'ompl'
                     else 'NOT_APPLICABLE_NO_RRT_TREE')))
    return attempts


def share_worlds(data):
    """Keep request worlds exact while storing repeated immutable arrays once."""
    worlds = {}
    shared = {}
    # Immutable request policies contain thousands of exact mast pairs.
    # Intern them losslessly instead of emitting megabytes per IK request.
    definition_ids={}
    visited=set()
    def intern(value):
        if isinstance(value,(dict,list)):
            if id(value) in visited:return
            visited.add(id(value))
        if isinstance(value,dict):
            for key,child in list(value.items()):
                if key in SHARED_KEYS and isinstance(child,(dict,list)):
                    ref=definition_ids.get(id(child))
                    if ref is None:
                        encoded=json.dumps(child,sort_keys=True,separators=(',',':'),allow_nan=False)
                        ref=hashlib.sha256(encoded.encode()).hexdigest()
                        definition_ids[id(child)]=ref
                    if ref not in shared:
                        shared[ref]=child
                        intern(child)
                    value[key]={'shared_definition_ref':ref}
                else:intern(child)
        elif isinstance(value,list):
            for child in value:intern(child)
    def walk(value):
        if isinstance(value, dict):
            if 'world' in value:
                world = value.pop('world')
                key = hashlib.sha256(json.dumps(
                    world, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
                worlds.setdefault(key, world)
                value['world_ref'] = key
            for child in value.values():
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)
    walk(data)
    data['shared_worlds'] = worlds
    intern(data)
    data['shared_definitions']=shared
    return data


def mechanism_state(snapshot, height):
    return coupled_conveyor_state(snapshot['scene']['coupled_conveyor_spec'], float(height))


def prepare_data(snapshot, plan=None, run_folder=None, preview_q=None, *, check_cycle=True):
    """No display-side mechanism solver or ownership reconstruction."""
    model = export_model(snapshot)
    robot = robot_for(snapshot['tool'], snapshot['scene'])
    scene = snapshot['scene']
    spec = scene['coupled_conveyor_spec']
    initial_height = scene['coupled_state']['height_m']
    first = scene['box']
    # Working candidates are frozen from the actual initial carton; bounds below
    # are only used to retain candidates, not to infer mechanical stroke.
    working = [r for r in receiving_height_candidates(spec,first) if r['in_hardware_range']]
    heights = [('LOW_INITIAL', initial_height)] + [(r['name'], r['height_m']) for r in working]
    previews = []
    # Full static world/cartons come from the authoritative initialized model.
    from unloading_sim.eco65.native_scene import world_objects
    base_world = world_objects(scene)
    mechanisms = {}
    base_dynamic = {o['id'] for o in scene['coupled_state']['world_objects']}
    fixed = [deepcopy(o) for o in base_world if o['id'] not in base_dynamic]
    cartons = [dict(id=b['id'], size=b['size_m'], category='carton',
                    pose=np.array([[1., 0., 0., b['pose'][0]],
                                   [0., 1., 0., b['pose'][1]],
                                   [0., 0., 1., b['pose'][2]],
                                   [0., 0., 0., 1.]]).tolist()) for b in scene['boxes']]
    q0 = (plan['initial_state']['q'] if plan else preview_q)
    if q0 is None:
        q0 = scene['source_equipment_config']['task']['initial_seed_q_rad']
    def mechanism(height):
        key = format(float(height), '.12f')
        if key not in mechanisms:
            mechanisms[key] = mechanism_state(snapshot, height)
        return key
    for label, height in heights:
        key = mechanism(height)
        previews.append(dict(label=label, height_m=height, mechanism_ref=key,
            state=dict(q=q0, height_m=height, world=fixed+mechanisms[key]['world_objects']+cartons,
                attachment=None, active_ids=[], target_id=scene['target_box_id'],
                completed_ids=[], business_interval='PREVIEW_NOT_A_TASK',
                mechanism=mechanisms[key]['mechanism'])))
    segments = []
    duration = 0.
    totals = dict(robot_motion_s=0., mechanism_display_s=0., nominal_drop_s=0.,
                  event_display_s=0., planning_wall_s=plan.get('planning_wall_s') if plan else None)
    validation = None
    records = plan['records'] if plan else []
    if plan:
        from unloading_sim.eco65.coupled_state import validate_cycle
        validation = (validate_cycle(plan, snapshot, require_complete=bool(plan['complete']))
                      if check_cycle else dict(status='NOT_RUN_USER_REQUEST',
                                               complete=bool(plan['complete'])))
        for original in records:
            row = deepcopy(original)
            row['display_policy'] = policy_display(row['native_request'], row['native_result'])
            kind = row['kind']
            if kind == 'robot_motion':
                elapsed = float(row['times'][-1])
                totals['robot_motion_s'] += elapsed
                diff=np.diff(np.asarray(row['q'])[:,0])
                direction=row['native_request'].get('j1_direction')
                reverse=float(np.maximum(-direction*diff,0.).sum()) if direction else 0.
                row['j1_travel']=dict(
                    q1_start=row['q'][0][0],q1_end=row['q'][-1][0],
                    clockwise_rad=float(np.maximum(-diff,0.).sum()),
                    counterclockwise_rad=float(np.maximum(diff,0.).sum()),
                    total_rad=float(np.abs(diff).sum()),direction=direction,
                    interval=row['state_before']['business_interval'],
                    cumulative_reverse_rad=reverse+row['native_request'].get('j1_reverse_used_rad',0.),
                    status='PASS' if reverse+row['native_request'].get('j1_reverse_used_rad',0.)<=1e-6 else 'FAIL')
            elif kind == 'external_axis':
                elapsed = float(row.get('duration_s', 0.))
                if elapsed <= 0:
                    raise ValueError('EXTERNAL_AXIS_DISPLAY_DURATION_REQUIRED')
                totals['mechanism_display_s'] += elapsed
                # Evaluate every displayed frame with the *same Python function*.
                # Nearest-frame selection preserves rigid body poses and length.
                h0, h1 = row['state_before']['height_m'], row['state_after']['height_m']
                count = max(2, int(np.ceil(elapsed*30))+1)
                row['display_samples'] = [dict(t=float(t), height_m=float(h),
                    mechanism_ref=mechanism(h)) for t, h in
                    zip(np.linspace(0., elapsed, count), np.linspace(h0, h1, count))]
            else:
                elapsed = float(row.get('duration_s', 0.))
                if row.get('event_type') == 'NOMINAL_RECEIVE':
                    totals['nominal_drop_s'] += elapsed
                elif elapsed == 0:
                    # This is an event hold for inspection, never robot motion.
                    elapsed = .35
                    row['display_hold_assumption'] = 'EVENT_INSPECTION_ONLY'
                    totals['event_display_s'] += elapsed
                else:
                    totals['event_display_s'] += elapsed
            for endpoint in ('state_before', 'state_after'):
                row[endpoint]['mechanism_ref'] = mechanism(row[endpoint]['height_m'])
                row[endpoint].pop('mechanism', None)
            if row.get('event_type') == 'ATTACH':
                geometry = snapshot['native_model']['init']['process_geometry']
                transform = (robot.named_link_frames(row['state_after']['q'])['link_6']
                             @ np.asarray(geometry['flange_from_contact_frame']))
                row['contact_contours_D'] = [dict(
                    id=contour['id'], points=(np.asarray(contour['points_m'])
                        @ transform[:3, :3].T+transform[:3, 3]).tolist(),
                    observation='PREDICTED_GEOMETRY_NOT_PHYSICAL_CONTACT')
                    for contour in geometry['contours']
                    if contour['id'] in row['state_after']['active_ids']]
            if kind == 'robot_motion' and row['stage'] == 'extraction':
                row['actual_extraction_tcp_D'] = robot.fk(row['q'][-1]).tolist()
            row.update(start_s=duration, duration_s=elapsed)
            duration += elapsed
            # Search requests/results live in the archive/attempt list once.
            row.pop('native_request', None)
            row.pop('native_result', None)
            # Original checked samples remain in cycle.json/the native archive.
            # Display uses authoritative per-frame mechanism samples or the
            # explicitly recorded nominal-drop samples, never these duplicates.
            row.pop('samples', None)
            segments.append(row)
    all_attempts = attempts_from_run(run_folder, records) if run_folder else []
    # The first request freezes the run's root task. Alternative initial q's
    # use distinct native trial IDs; accepted trajectory records remain one
    # continuous trial and must never be stitched across those alternatives.
    root_task = next((a['request'].get('task_id') for a in all_attempts
                      if a['request'].get('task_id')),None)
    def same_run(task_id):
        return (task_id==root_task or isinstance(task_id,str) and root_task is not None
                and re.fullmatch(re.escape(root_task)+r'-initial-[0-9]+-[0-9]+',task_id) is not None)
    if root_task and plan and not same_run(plan['task_id']):
        raise ValueError('REPLAY_PLAN_RUN_MISMATCH')
    if plan and any(r['native_request']['task_id']!=plan['task_id'] for r in records):
        raise ValueError('REPLAY_CANNOT_JOIN_NATIVE_TRIALS')
    run_report = None
    if run_folder and (run_folder/'run_report.json').exists():
        run_report=load_json(run_folder/'run_report.json')
        if run_report.get('planning_wall_s') is not None:
            totals['planning_wall_s']=run_report['planning_wall_s']
    unbound_ik = [a for a in all_attempts if a['world_context_status'] == 'NOT_CAPTURED']
    attempts = [a for a in all_attempts if a['world_context_status'] == 'CAPTURED']
    for attempt in attempts:
        request = attempt['request']
        if request.get('identity') != snapshot['native_model']['init']['identity']:
            raise ValueError('REPLAY_REQUEST_MODEL_MISMATCH')
        if root_task and not same_run(request.get('task_id')):
            raise ValueError('REPLAY_REQUEST_RUN_MISMATCH')
        # Bind mechanism to this request's own world; never current replay time.
        height = request.get('mechanism_state', {}).get(
            'height_m', request.get('height_m', request.get('external_height_m')))
        if height is None:
            for record in records:
                if record['record_id'] == request.get('stage_id'):
                    height = record['state_before']['height_m']
                    break
        if height is not None:
            attempt['mechanism_ref'] = mechanism(height)
        for failure in attempt['failure_samples']:
            state = failure.get('state')
            if state is not None and state['height_m'] is not None:
                state['mechanism_ref'] = mechanism(state['height_m'])
    fixtures = []
    fixture_q = [q0] + [s['state_after']['q'] for s in segments[:4]]
    for q in fixture_q:
        fixtures.append(dict(q=q, frames={k:np.asarray(v).tolist()
            for k, v in robot.named_link_frames(q).items()}))
    return share_worlds(dict(schema='eco65_coupled_replay_v1', model=model,
        snapshot_fingerprint=snapshot['fingerprint'],
        collision_policy=snapshot['native_model']['init']['collision_policy'],
        preview_display_policy=policy_display(
            configured_policy=snapshot['native_model']['init']['collision_policy']),
        previews=previews, mechanisms=mechanisms, segments=segments, attempts=attempts,
        unbound_ik_attempts=unbound_ik,
        run_report=run_report,
        native_trials=dict(root_task_id=root_task,
            displayed_task_id=plan.get('task_id') if plan else None,
            trial_ids=sorted({a['request']['task_id'] for a in all_attempts if a['request'].get('task_id')}),
            displayed_chain_scope='COMPLETE_PLAN' if plan and plan['complete'] else 'SINGLE_TRIAL_SUCCESSFUL_PREFIX_ONLY'),
        plan_summary=None if plan is None else {k:plan.get(k) for k in (
            'schema','complete','status','first_target_id','next_target_id','planning_wall_s',
            'physical_execution','execution_ready','j1_direction_policy','failures')},
        initial_state=plan['initial_state'] if plan else previews[0]['state'],
        terminal_state=plan['terminal_state'] if plan else None,
        validation=validation, duration_s=duration, times=totals, fk_fixtures=fixtures,
        physical_execution='NOT_RUN', execution_ready=False, planning_only=True,
        status='KINEMATIC_REPLAY', coordinate_frame='D: +X 入垛 / +Y 左 / +Z 上; m, rad, s',
        source_contract='SAME_PYTHON_COUPLED_SCENE_AND_CURRENT_RUN_NATIVE_RECORDS'))


def export(snapshot_path, output, *, plan_path=None, run_folder=None, preview_q=None,
           check_cycle=True):
    start = time.perf_counter()
    output = Path(output)
    output.resolve().relative_to(ROOT / 'outputs')
    if output.exists():
        raise FileExistsError('Use a fresh output path')
    snapshot = verify_snapshot(load_json(snapshot_path))
    plan = load_json(plan_path) if plan_path else None
    data = prepare_data(snapshot, plan, Path(run_folder) if run_folder else None, preview_q,
                        check_cycle=check_cycle)
    vendor = ROOT / 'tools/search_replay/vendor'
    template = (ROOT / 'tools/search_replay/eco65_coupled_viewer.html').read_text(encoding='utf-8')
    payload = json.dumps(data, separators=(',', ':'), allow_nan=False)
    html = template.replace('/*__VENDOR__*/', (vendor/'three.min.js').read_text()+'\n'+
                            (vendor/'OrbitControls.js').read_text())
    html = html.replace('/*__DATA__*/', payload.replace('</', '<\\/'))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding='utf-8')
    data_path = output.with_suffix('.data.json.gz')
    with gzip.open(data_path, 'wt', encoding='utf-8') as stream:
        stream.write(payload)
    report = dict(html=output.name, sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        bytes=output.stat().st_size, export_wall_s=time.perf_counter()-start,
        complete=bool(plan and plan['complete']), segments=len(data['segments']),
        requests=len(data['attempts']), previews=len(data['previews']),
        snapshot_fingerprint=snapshot['fingerprint'], times=data['times'],
        validation=data['validation'], physical_execution='NOT_RUN', execution_ready=False)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    # The bundle contains the actual evidence, not only paths to ignored files.
    with zipfile.ZipFile(output.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in (output, data_path, output.with_suffix('.json')):
            archive.write(path, path.name)
        archive.writestr('records/snapshot.json', json.dumps(snapshot))
        archive.write(vendor/'LICENSE.three', 'LICENSE.three')
        if plan_path:
            archive.write(plan_path, 'records/cycle.json')
        if run_folder:
            run_folder = Path(run_folder)
            for path in sorted(run_folder.rglob('*')):
                if path.is_file() and path.suffix in ('.json', '.gz', '.jsonl'):
                    archive.write(path, 'records/native/'+path.relative_to(run_folder).as_posix())
    print(json.dumps(report, ensure_ascii=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--plan', type=Path)
    parser.add_argument('--run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preview-q', type=float, nargs=6)
    parser.add_argument('--skip-cycle-validation', action='store_true',
                        help='Export existing records for inspection without rerunning cycle validation.')
    args = parser.parse_args()
    export(args.snapshot, args.output, plan_path=args.plan, run_folder=args.run,
           preview_q=args.preview_q, check_cycle=not args.skip_cycle_validation)


if __name__ == '__main__':
    main()
