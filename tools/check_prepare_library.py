"""Python prepare() vs libm710_prepare.so (native_prepare.h) on recorded native plans (no hardware).

User decision C (2026-11-23): the Python planning flow may call the worker's
C++ prepare through ctypes (prepare_implementation=NATIVE_LIBRARY). This
records how far that switch moves the prepared trajectories on saved records:
every native result with points (``*-native.json``) is prepared by both
implementations with both probe grids at the configured time scale times every
SEND_SLOWDOWN_STEPS factor, with the planner's limits, period and reach bound
(plus the payload reach of a loaded request). Reported per case: identical
decision (both prepared, or the same error string), bitwise identity, and
the largest absolute difference of every output.

Usage: python tools/check_prepare_library.py --records DIR [DIR ...] --output OUT.json
       (M710_MOVEIT_INSTALL_PREFIX selects the library, as for the worker)
"""
import argparse
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'src'))
KEYS = ('t', 'q', 'v', 'a')


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


def run(function, *args, **kwargs):
    started = time.perf_counter()
    try:
        return function(*args, **kwargs), None, time.perf_counter()-started
    except ValueError as error:
        return None, str(error), time.perf_counter()-started


def difference(a, b):
    out = {}
    for k in KEYS:
        x = np.asarray([p[k] for p in a['points']], float); y = np.asarray([p[k] for p in b['points']], float)
        out[k] = float('inf') if x.shape != y.shape else float(np.max(abs(x-y)))
    x = np.asarray(a['collision_path']); y = np.asarray(b['collision_path'])
    out['collision_path'] = float('inf') if x.shape != y.shape else float(np.max(abs(x-y)))
    x = np.asarray(a['collision_sampling']['analytic_speed_bounds_rad_s']); y = np.asarray(b['collision_sampling']['analytic_speed_bounds_rad_s'])
    out['analytic_speed_bounds_rad_s'] = float(np.max(abs(x-y)))
    for k in ('send_duration_s', 'scaled_duration_s', 'source_duration_s', 'terminal_hold_s'):
        out[k] = abs(a[k]-b[k])
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--limit', type=int, default=0, help='at most this many recorded plans (0: all)')
    args = parser.parse_args()
    import check_planning_deadline as tool
    from unloading_sim.workbench.controlled.trajectory import prepare
    from unloading_sim.workbench.single_carton import native_prepare
    from unloading_sim.workbench.single_carton.planner import SEND_SLOWDOWN_STEPS, merge_coincident_knots
    from unloading_sim.workbench.single_carton.geometry import corners
    planner, _ = tool.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    _, identity = native_prepare.load()
    sources = collect(args.records)
    if args.limit:
        sources = sources[:args.limit]
    if not sources:
        raise SystemExit('NO_RECORDED_NATIVE_PLANS')
    limits = planner.limits; period = planner.config['period_s']; scale = planner.site['time_scale']
    rows = []
    for s in sources:
        attachment = s['request'].get('attachment')
        payload = 0. if not attachment else float(np.linalg.norm(corners(attachment['pose'], attachment['size']), axis=1).max())
        reach = planner.reach_bound+payload
        points, _ = merge_coincident_knots(s['points'])
        for grid in ('GEOMETRIC', 'SENT_GRID'):
            for factor in SEND_SLOWDOWN_STEPS:
                a, ea, ta = run(prepare, points, limits, scale*factor, period, reach, probe_grid=grid)
                b, eb, tb = run(native_prepare.prepare, points, limits, scale*factor, period, reach, probe_grid=grid)
                row = dict(source=s['source'], grid=grid, time_scale=scale*factor, reach_bound_m=reach, python_s=ta, library_s=tb,
                           python_error=ea, library_error=eb, same_decision=(ea == eb) and ((a is None) == (b is None)))
                if a is not None and b is not None:
                    d = difference(a, b); row['max_abs'] = d
                    row['bitwise_points_and_probe_path'] = all(d[k] == 0. for k in KEYS+('collision_path',))
                    row['same_sampling_except_speed_bounds'] = all(a['collision_sampling'][k] == b['collision_sampling'][k]
                                                                  for k in a['collision_sampling'] if k != 'analytic_speed_bounds_rad_s')
                rows.append(row)
    numeric = [r for r in rows if 'max_abs' in r]
    worst = {k: max((r['max_abs'][k] for r in numeric), default=None) for k in KEYS+('collision_path', 'analytic_speed_bounds_rad_s', 'send_duration_s')}
    report = dict(schema='eco65_prepare_library_equivalence_v1', date=time.strftime('%Y-%m-%d'),
                  scope='RECORDED_NATIVE_PLANS_OFFLINE_NOT_PHYSICAL_ACCEPTANCE', library=identity,
                  inputs=[str(p) for p in args.records], recorded_native_plans=len(sources), cases=len(rows),
                  same_decision=sum(r['same_decision'] for r in rows), prepared=len(numeric),
                  bitwise_points_and_probe_path=sum(bool(r.get('bitwise_points_and_probe_path')) for r in numeric),
                  above_1e_12=sum(any(v > 1e-12 for v in r['max_abs'].values()) for r in numeric),
                  same_sampling_except_speed_bounds=sum(bool(r.get('same_sampling_except_speed_bounds')) for r in numeric),
                  worst_max_abs=worst, errors=sorted({r['python_error'] for r in rows if r['python_error']}),
                  python_prepare_s=dict(total=sum(r['python_s'] for r in rows), median=float(np.median([r['python_s'] for r in rows]))),
                  library_prepare_s=dict(total=sum(r['library_s'] for r in rows), median=float(np.median([r['library_s'] for r in rows]))),
                  environment=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__),
                  decision_mismatches=[r for r in rows if not r['same_decision']][:20], rows=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps({k: report[k] for k in ('recorded_native_plans', 'cases', 'same_decision', 'prepared', 'bitwise_points_and_probe_path',
                                             'above_1e_12', 'same_sampling_except_speed_bounds', 'worst_max_abs', 'python_prepare_s',
                                             'library_prepare_s')}, indent=1))
    return 0 if report['same_decision'] == report['cases'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
