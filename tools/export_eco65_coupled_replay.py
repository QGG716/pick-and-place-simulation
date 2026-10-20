"""Export a self-contained ECO65 coupled-scene preview or current-run cycle replay."""
from __future__ import annotations

import argparse
from copy import deepcopy
import gzip
import hashlib
import json
from pathlib import Path
import time
import zipfile

import numpy as np

from export_eco65_search_replay import export_model
from unloading_sim.eco65.model import ROOT, load_json, robot_for
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.coupled_scene import coupled_conveyor_state, receiving_height_candidates


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


def attempts_from_run(folder, records):
    """Use actual requests, responses and same-call search traces, including failures."""
    adopted = {r['record_id'] for r in records}
    path = folder / 'native_calls.jsonl.gz'
    if not path.exists():
        return []
    attempts = []
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            req = row['request']
            if req['op'] == 'init':
                continue
            result = row.get('result', {})
            if 'world' not in req or 'q_start' not in req:
                continue
            trace = read_trace(result, folder)
            attempts.append(dict(
                request=req, response=result, trace=trace,
                adopted=req.get('stage_id') in adopted,
                round_trip_s=row.get('wall_s', row.get('round_trip_s')),
                error=row.get('error'), native_path=result.get('points', []),
                trace_status=trace.get('status', 'CAPTURED') if trace else
                    ('NOT_CAPTURED' if req.get('pipeline_id') == 'ompl'
                     else 'NOT_APPLICABLE_NO_RRT_TREE')))
    return attempts


def mechanism_state(snapshot, height):
    return coupled_conveyor_state(snapshot['scene']['coupled_conveyor_spec'], float(height))


def prepare_data(snapshot, plan=None, run_folder=None, preview_q=None):
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
        validation = validate_cycle(plan, snapshot, require_complete=bool(plan['complete']))
        for original in records:
            row = deepcopy(original)
            kind = row['kind']
            if kind == 'robot_motion':
                elapsed = float(row['times'][-1])
                totals['robot_motion_s'] += elapsed
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
            row.update(start_s=duration, duration_s=elapsed)
            duration += elapsed
            # Search requests/results live in the archive/attempt list once.
            row.pop('native_request', None)
            row.pop('native_result', None)
            segments.append(row)
    attempts = attempts_from_run(run_folder, records) if run_folder else []
    for attempt in attempts:
        request = attempt['request']
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
    fixtures = []
    fixture_q = [q0] + [s['state_after']['q'] for s in segments[:4]]
    for q in fixture_q:
        fixtures.append(dict(q=q, frames={k:np.asarray(v).tolist()
            for k, v in robot.named_link_frames(q).items()}))
    return dict(schema='eco65_coupled_replay_v1', model=model,
        snapshot_fingerprint=snapshot['fingerprint'], collision_policy=scene.get('collision_policy'),
        previews=previews, mechanisms=mechanisms, segments=segments, attempts=attempts,
        plan_summary=None if plan is None else {k:plan.get(k) for k in (
            'schema','complete','status','first_target_id','next_target_id','planning_wall_s',
            'physical_execution','execution_ready','j1_direction_policy','failures')},
        initial_state=plan['initial_state'] if plan else previews[0]['state'],
        terminal_state=plan['terminal_state'] if plan else None,
        validation=validation, duration_s=duration, times=totals, fk_fixtures=fixtures,
        physical_execution='NOT_RUN', execution_ready=False, planning_only=True,
        status='KINEMATIC_REPLAY', coordinate_frame='D: +X 入垛 / +Y 左 / +Z 上; m, rad, s',
        source_contract='SAME_PYTHON_COUPLED_SCENE_AND_CURRENT_RUN_NATIVE_RECORDS')


def export(snapshot_path, output, *, plan_path=None, run_folder=None, preview_q=None):
    start = time.perf_counter()
    output = Path(output)
    output.resolve().relative_to(ROOT / 'outputs')
    if output.exists():
        raise FileExistsError('Use a fresh output path')
    snapshot = verify_snapshot(load_json(snapshot_path))
    plan = load_json(plan_path) if plan_path else None
    data = prepare_data(snapshot, plan, Path(run_folder) if run_folder else None, preview_q)
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
    args = parser.parse_args()
    export(args.snapshot, args.output, plan_path=args.plan, run_folder=args.run,
           preview_q=args.preview_q)


if __name__ == '__main__':
    main()
