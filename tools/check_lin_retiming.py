"""User decision D (2026-11-23): PILZ_PROFILE vs PATH_TOPP on recorded Pilz LIN results (offline, no hardware).

For every successful recorded Pilz LIN result (``*-native.json`` whose request
is a LIN plan), with the planner's execution limits, time scale, period and
reach bound (plus the payload reach of a loaded request):

* PILZ_PROFILE: the recorded profile through the send-slowdown search
  (send duration, factor) - when it fails, planning re-plans the line at a
  lower Pilz scale (not reproduced here);
* PATH_TOPP: the waypoints retimed along the line (lin_retiming.retime) through
  the same search;
* the largest TCP distance from the straight start-end line over every probe
  state of each prepared path (both), and the retiming time.

Usage: python tools/check_lin_retiming.py --records DIR [DIR ...] --output OUT.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools')); sys.path.insert(0, str(ROOT/'src'))


def lin_results(folders):
    seen = set(); out = []
    for folder in folders:
        for path in sorted(Path(folder).rglob('*-native.json')):
            try:
                record = json.loads(path.read_text())
            except ValueError:
                continue
            request = record.get('request') or {}; result = record.get('result') or {}
            if request.get('planner_id') != 'LIN' or result.get('status') != 'SUCCESS' or not result.get('points'):
                continue
            key = json.dumps(result['points'], sort_keys=True)
            if key in seen:
                continue
            seen.add(key); out.append(dict(source=str(path), stage=request.get('stage'), points=result['points'], request=request))
    return out


def deviation(model, path):
    p = np.array([np.asarray(model.tcp(q), float)[:3, 3] for q in path])
    axis = p[-1]-p[0]; length = np.linalg.norm(axis)
    if length < 1e-12:
        return float(np.max(np.linalg.norm(p-p[0], axis=1)))
    axis = axis/length; r = p-p[0]
    return float(np.max(np.linalg.norm(r-np.outer(r@axis, axis), axis=1)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fraction', type=float, default=.95)
    args = parser.parse_args()
    import check_planning_deadline as tool
    from unloading_sim.workbench.controlled.trajectory import slowdown_search
    from unloading_sim.workbench.single_carton.planner import SEND_SLOWDOWN_STEPS, merge_coincident_knots
    from unloading_sim.workbench.single_carton.geometry import corners
    from unloading_sim.workbench.single_carton.lin_retiming import RetimingUnavailable
    planner, model = tool.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    planner.site['lin_time_parametrization'] = dict(mode='PATH_TOPP', joint_limit_fraction=args.fraction, basis='OFFLINE_EVALUATION')
    limits = planner.limits; period = planner.config['period_s']; scale = planner.site['time_scale']
    rows = []
    for s in lin_results(args.records):
        attachment = s['request'].get('attachment')
        payload = 0. if not attachment else float(np.linalg.norm(corners(attachment['pose'], attachment['size']), axis=1).max())
        reach = planner.reach_bound+payload
        points, _ = merge_coincident_knots(s['points'])
        row = dict(source=s['source'], stage=s['stage'], pilz_velocity_scale=s['request'].get('velocity_scale'), waypoints=len(points))
        try:
            prepared, factor, _ = slowdown_search(points, limits, scale, SEND_SLOWDOWN_STEPS, period, reach, 'GEOMETRIC')
            row.update(pilz_send_s=prepared['send_duration_s'], pilz_factor=factor, pilz_deviation_m=deviation(model, prepared['collision_path']))
        except ValueError as error:
            row.update(pilz_error=str(error))
        started = time.perf_counter()
        try:
            knots, info = planner.retime_line(points)
            row['retime_s'] = time.perf_counter()-started; row['retime'] = info
            prepared, factor, _ = slowdown_search(knots, limits, scale, SEND_SLOWDOWN_STEPS, period, reach, 'GEOMETRIC')
            row.update(topp_send_s=prepared['send_duration_s'], topp_factor=factor, topp_deviation_m=deviation(model, prepared['collision_path']),
                       topp_probe_states=len(prepared['collision_path']))
        except (ValueError, RetimingUnavailable) as error:
            row.update(topp_error=str(error), retime_s=time.perf_counter()-started)
        rows.append(row); print(json.dumps({k: v for k, v in row.items() if k not in ('retime', 'source')}), flush=True)
    both = [r for r in rows if 'pilz_send_s' in r and 'topp_send_s' in r]
    report = dict(schema='eco65_lin_retiming_evaluation_v1', date=time.strftime('%Y-%m-%d'), scope='RECORDED_PILZ_LIN_RESULTS_OFFLINE_NOT_EXECUTABLE',
                  inputs=[str(p) for p in args.records], joint_limit_fraction=args.fraction, lin_results=len(rows),
                  both_prepared=len(both), pilz_failed=sum('pilz_error' in r for r in rows), topp_failed=sum('topp_error' in r for r in rows),
                  send_s_total=dict(pilz=sum(r['pilz_send_s'] for r in both), topp=sum(r['topp_send_s'] for r in both)),
                  ratio_median=float(np.median([r['topp_send_s']/r['pilz_send_s'] for r in both])) if both else None,
                  by_stage={st: dict(n=len(x), pilz_median_s=float(np.median([r['pilz_send_s'] for r in x])),
                                     topp_median_s=float(np.median([r['topp_send_s'] for r in x])))
                            for st in sorted({r['stage'] for r in both}) for x in [[r for r in both if r['stage'] == st]]},
                  max_line_deviation_m=dict(pilz=max((r['pilz_deviation_m'] for r in rows if 'pilz_deviation_m' in r), default=None),
                                            topp=max((r['topp_deviation_m'] for r in rows if 'topp_deviation_m' in r), default=None)),
                  retime_s_median=float(np.median([r['retime_s'] for r in rows])) if rows else None, rows=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}, indent=1))


if __name__ == '__main__':
    raise SystemExit(main())
