"""Python prepare() vs worker sc_prepare (native_prepare.h) on recorded native plans (no hardware).

Every native planning result with points found in the record folders
(``*-native.json`` whose result has ``points``) is prepared by both
implementations at the configured time scale times every SEND_SLOWDOWN_STEPS
factor, with the planner's limits, period and reach bound (plus the payload
reach of a loaded request). Constructed cases add limit violations (time axis
compressed), nonzero endpoint velocity, a broken time axis and an invalid
reach bound. Acceptance: identical outcome (same error string, or both
prepared) and every output value within 1e-12.

Usage: python tools/check_native_prepare.py --records DIR [DIR ...] --output OUT.json
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
TOLERANCE = 1e-12


def collect(folders):
    seen = set(); out = []
    for folder in folders:
        for path in sorted(Path(folder).rglob('*-native.json')):
            try:
                record = json.loads(path.read_text())
            except ValueError:
                continue
            result = record.get('result') if isinstance(record, dict) else None
            if not isinstance(result, dict) or not result.get('points'):
                continue
            key = json.dumps(result['points'], sort_keys=True)
            if key in seen:
                continue
            seen.add(key)
            out.append(dict(source=str(path), points=result['points'], request=record.get('request') or {}))
    return out


def python_prepare(points, limits, scale, period, reach):
    from unloading_sim.workbench.controlled.trajectory import prepare
    started = time.perf_counter()
    try:
        p = prepare(points, limits, scale, period, reach)
        return dict(status='PREPARED', t=[x['t'] for x in p['points']], q=[x['q'] for x in p['points']],
                    v=[x['v'] for x in p['points']], a=[x['a'] for x in p['points']], collision_path=p['collision_path'],
                    analytic_speed_bounds_rad_s=p['collision_sampling']['analytic_speed_bounds_rad_s'],
                    subdivisions_per_send=p['collision_sampling']['subdivisions_per_send'],
                    send_duration_s=p['send_duration_s'], scaled_duration_s=p['scaled_duration_s'],
                    source_duration_s=p['source_duration_s'], terminal_hold_s=p['terminal_hold_s']), time.perf_counter()-started
    except ValueError as e:
        return dict(status='PREPARE_REJECTED', error=str(e)), time.perf_counter()-started


def compare(py, nat):
    if py['status'] != nat['status']:
        return False, dict(status=(py['status'], nat['status']), errors=(py.get('error'), nat.get('error')))
    if py['status'] != 'PREPARED':
        return py['error'] == nat['error'], dict(errors=(py['error'], nat['error']))
    p = nat['prepared']; diffs = {}
    for key in ('t', 'q', 'v', 'a', 'collision_path', 'analytic_speed_bounds_rad_s'):
        a = np.asarray(py[key], float); b = np.asarray(p[key], float)
        diffs[key] = float('inf') if a.shape != b.shape else (float(np.max(abs(a-b))) if a.size else 0.)
    for key in ('send_duration_s', 'scaled_duration_s', 'source_duration_s', 'terminal_hold_s'):
        diffs[key] = abs(py[key]-p[key])
    same_counts = py['subdivisions_per_send'] == p['subdivisions_per_send'] and len(py['t']) == len(p['t']) and \
        len(py['collision_path']) == len(p['collision_path'])
    return same_counts and max(diffs.values()) <= TOLERANCE, dict(max_abs=diffs, subdivisions=(py['subdivisions_per_send'], p['subdivisions_per_send']),
                                                                   points=(len(py['t']), len(p['t'])),
                                                                   collision_states=(len(py['collision_path']), len(p['collision_path'])))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import check_planning_deadline as tool
    from unloading_sim.moveit2_backend import ResidentMoveItClient
    from unloading_sim.workbench.single_carton.planner import SEND_SLOWDOWN_STEPS
    from unloading_sim.workbench.single_carton.geometry import corners
    from dataclasses import asdict
    planner, _ = tool.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    sources = collect(args.records)
    if not sources:
        raise SystemExit('NO_RECORDED_NATIVE_PLANS')
    client = ResidentMoveItClient(['env', 'ROS_LOG_DIR=/tmp/native-prepare', str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                  log_path='/tmp/native-prepare.log')
    ready = client.request(planner.init, timeout=180)
    if not ready.get('capabilities', {}).get('native_prepare'):
        raise SystemExit('NATIVE_WORKER_UPDATE_REQUIRED_FOR_NATIVE_PREPARE')
    limits = planner.limits; period = planner.config['period_s']; scale = planner.site['time_scale']
    cases = []
    for s in sources:
        attachment = s['request'].get('attachment')
        payload = 0. if not attachment else float(np.linalg.norm(corners(attachment['pose'], attachment['size']), axis=1).max())
        reach = planner.reach_bound+payload
        for factor in SEND_SLOWDOWN_STEPS:
            cases.append(dict(source=s['source'], kind=f'recorded x{factor:g}', points=s['points'], scale=scale*factor, reach=reach))
        # Constructed: compressed time axis (limit violations), endpoint velocity, time axis, reach bound.
        for squeeze in (.5, .2):
            pts = copy.deepcopy(s['points'])
            for p in pts:
                p['t'] *= squeeze; p['v'] = [x/squeeze for x in p['v']]; p['a'] = [x/squeeze**2 for x in p['a']]
            cases.append(dict(source=s['source'], kind=f'compressed x{squeeze:g}', points=pts, scale=scale, reach=reach))
    first = sources[0]['points']
    bad_v = copy.deepcopy(first); bad_v[-1]['v'] = [1e-3]+[0.]*5
    bad_t = copy.deepcopy(first); bad_t[1]['t'] = bad_t[0]['t']
    cases += [dict(source='constructed', kind='nonzero_endpoint_velocity', points=bad_v, scale=scale, reach=planner.reach_bound),
              dict(source='constructed', kind='time_axis', points=bad_t, scale=scale, reach=planner.reach_bound),
              dict(source='constructed', kind='reach_bound', points=first, scale=scale, reach=0.),
              dict(source='constructed', kind='timing', points=first, scale=.5, reach=planner.reach_bound)]
    rows = []
    for case in cases:
        py, py_s = python_prepare(case['points'], limits, case['scale'], period, case['reach'])
        started = time.perf_counter()
        nat = client.request(dict(op='sc_prepare', identity=planner.init['identity'], points=case['points'], limits=asdict(limits),
                                  time_scale=case['scale'], period_s=period, reach_bound_m=case['reach']), timeout=600)
        round_trip = time.perf_counter()-started
        ok, detail = compare(py, nat)
        rows.append(dict(source=case['source'], kind=case['kind'], identical=ok, outcome=py['status'], error=py.get('error'),
                         python_s=py_s, native_s=nat.get('prepare_s'), native_round_trip_s=round_trip, **detail))
    client.close()
    prepared = [r for r in rows if r['outcome'] == 'PREPARED']
    worst = {k: max((r['max_abs'][k] for r in prepared), default=0.) for k in
             ('t', 'q', 'v', 'a', 'collision_path', 'analytic_speed_bounds_rad_s', 'send_duration_s')}
    report = dict(schema='eco65_native_prepare_equivalence_v1', scope='SYNTHETIC_SCENE_RECORDS_NOT_FIELD_DATA',
                  inputs=[str(p) for p in args.records], recorded_native_plans=len(sources), cases=len(rows),
                  identical=sum(r['identical'] for r in rows), prepared=len(prepared),
                  rejected=len(rows)-len(prepared), rejection_errors=sorted({r['error'] for r in rows if r['error']}),
                  worst_max_abs=worst, tolerance=TOLERANCE,
                  python_prepare_s=dict(total=sum(r['python_s'] for r in rows), median=float(np.median([r['python_s'] for r in rows]))),
                  native_prepare_s=dict(total=sum(r['native_s'] or 0 for r in rows), median=float(np.median([r['native_s'] or 0 for r in rows]))),
                  acceptance=dict(all_identical=all(r['identical'] for r in rows)),
                  mismatches=[r for r in rows if not r['identical']][:20], rows=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps({k: report[k] for k in ('recorded_native_plans', 'cases', 'identical', 'prepared', 'rejected', 'rejection_errors',
                                             'worst_max_abs', 'python_prepare_s', 'native_prepare_s', 'acceptance')}, indent=1))


if __name__ == '__main__':
    main()
