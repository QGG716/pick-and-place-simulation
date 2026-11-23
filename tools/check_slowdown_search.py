"""analytic_send_slowdown: sequential factor loop vs slowdown_search on recorded native plans (no hardware).

For every native result with points (``*-native.json``), at the configured
time scale, both probe grids, with the planner's limits, period and reach
bound (plus the payload reach of a loaded request): the factor found, the
prepared result (canonical JSON) and the final error must be identical.
Reports how many prepare() calls were skipped and both wall times.

Usage: python tools/check_slowdown_search.py --records DIR [DIR ...] --output OUT.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools')); sys.path.insert(0, str(ROOT/'src'))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import check_planning_deadline as tool
    from check_prepare_library import collect
    from unloading_sim.workbench.controlled.trajectory import prepare, slowdown_search
    from unloading_sim.workbench.single_carton.planner import SEND_SLOWDOWN_STEPS, merge_coincident_knots
    from unloading_sim.workbench.single_carton.geometry import corners
    planner, _ = tool.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    limits = planner.limits; period = planner.config['period_s']; scale = planner.site['time_scale']
    rows = []
    for s in collect(args.records):
        attachment = s['request'].get('attachment')
        payload = 0. if not attachment else float(np.linalg.norm(corners(attachment['pose'], attachment['size']), axis=1).max())
        reach = planner.reach_bound+payload
        points, _ = merge_coincident_knots(s['points'])
        for grid in ('GEOMETRIC', 'SENT_GRID'):
            started = time.perf_counter(); seq = seq_error = None; calls = 0
            for factor in SEND_SLOWDOWN_STEPS:
                calls += 1
                try:
                    seq = (prepare(points, limits, scale*factor, period, reach, probe_grid=grid), factor); break
                except ValueError as error:
                    if not str(error).startswith(('INTERPOLATION_LIMIT', 'DISCRETE_COMMAND_LIMIT')) or factor == SEND_SLOWDOWN_STEPS[-1]:
                        seq_error = str(error); break
            seq_s = time.perf_counter()-started; started = time.perf_counter(); found = found_error = None; attempts = []
            try:
                prepared, factor, attempts = slowdown_search(points, limits, scale, SEND_SLOWDOWN_STEPS, period, reach, grid)
                found = (prepared, factor)
            except ValueError as error:
                found_error = str(error)
            search_s = time.perf_counter()-started
            same = (seq_error == found_error) and ((seq is None and found is None) or
                   (seq is not None and found is not None and seq[1] == found[1] and json.dumps(seq[0]) == json.dumps(found[0])))
            rows.append(dict(source=s['source'], grid=grid, identical=same, factor=None if seq is None else seq[1], error=seq_error,
                             sequential_calls=calls, search_calls=sum(not a['result'].startswith('SKIPPED') for a in attempts)+(found is not None),
                             skipped=sum(a['result'].startswith('SKIPPED') for a in attempts), sequential_s=seq_s, search_s=search_s))
    report = dict(schema='eco65_slowdown_search_equivalence_v1', date=time.strftime('%Y-%m-%d'),
                  scope='RECORDED_NATIVE_PLANS_OFFLINE_NOT_PHYSICAL_ACCEPTANCE', inputs=[str(p) for p in args.records],
                  cases=len(rows), identical=sum(r['identical'] for r in rows),
                  factors={str(k): sum(r['factor'] == k for r in rows) for k in SEND_SLOWDOWN_STEPS}, errors=sum(r['error'] is not None for r in rows),
                  sequential_prepare_calls=sum(r['sequential_calls'] for r in rows), search_prepare_calls=sum(r['search_calls'] for r in rows),
                  skipped_calls=sum(r['skipped'] for r in rows),
                  sequential_s=sum(r['sequential_s'] for r in rows), search_s=sum(r['search_s'] for r in rows),
                  mismatches=[r for r in rows if not r['identical']][:20], rows=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps({k: report[k] for k in ('cases', 'identical', 'factors', 'errors', 'sequential_prepare_calls', 'search_prepare_calls',
                                             'skipped_calls', 'sequential_s', 'search_s')}, indent=1))
    return 0 if report['identical'] == report['cases'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
