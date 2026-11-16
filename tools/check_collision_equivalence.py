"""EXACT vs TWO_LEVEL collision decisions on recorded checked states (no hardware).

Reads the native records of plan folders (``*-native.json`` written by
plan_cycle / tools/check_step4_mechanisms.py), takes every validate request:
endpoint checks give their q_start, sent-path checks give every probe-path
state. Each record is re-sent once as ``sc_collision_equivalence`` with its own
scene, attachment and policy, adding perturbed copies of its states and
uniform random states. The worker checks each state with EXACT and TWO_LEVEL
and reports every disagreement. A TWO_LEVEL pass where EXACT fails is a
counterexample; there must be none.

Usage: python tools/check_collision_equivalence.py --records DIR [DIR ...] --output OUT
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))


def stage_of(path, request):
    name = path.name
    for stage in ('loaded-start', 'approach', 'contact', 'detach', 'return'):
        if name.startswith(stage):
            return 'return' if stage == 'loaded-start' else stage
    return request.get('stage', 'other')


def collect(folders):
    jobs = []
    for folder in folders:
        for path in sorted(Path(folder).rglob('*-native.json')):
            try:
                record = json.loads(path.read_text())
            except ValueError:
                continue
            request = record.get('request') or {}
            if request.get('op') != 'validate' or 'world' not in request:
                continue
            states = request['probe_path'] if 'probe_path' in request else [request['q_start']]
            kind = 'sent_path' if 'probe_path' in request else 'endpoint'
            base = {k: v for k, v in request.items() if k not in ('probe_path', 'request_id', 'collision_check_mode')}
            jobs.append(dict(source=str(path), stage=stage_of(path, request), kind=kind, request=base, states=states))
    return jobs


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--random-per-record', type=int, default=200)
    parser.add_argument('--perturbed-per-state', type=int, default=1)
    parser.add_argument('--perturb-rad', type=float, default=.02)
    parser.add_argument('--workers', type=int, default=8)
    args = parser.parse_args()
    import check_planning_deadline as tool
    from unloading_sim.moveit2_backend import ResidentMoveItClient
    planner, _model = tool.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    jobs = collect(args.records)
    if not jobs:
        raise SystemExit('NO_RECORDED_VALIDATE_REQUESTS')
    clients = []
    for k in range(min(args.workers, len(jobs))):
        client = ResidentMoveItClient(['env', 'ROS_LOG_DIR=/tmp/equivalence', str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                      log_path=f'/tmp/equivalence-{k}.log')
        ready = client.request(planner.init, timeout=180)
        if not ready.get('capabilities', {}).get('two_level_collision'):
            raise SystemExit('NATIVE_WORKER_UPDATE_REQUIRED_FOR_TWO_LEVEL_COLLISION')
        clients.append(client)

    def run(index):
        job = jobs[index]; client = clients[index % len(clients)]
        request = dict(job['request'], op='sc_collision_equivalence', probe_states=job['states'],
                       random_states=args.random_per_record, random_seed=6518+index,
                       perturbed_per_state=args.perturbed_per_state, perturb_rad=args.perturb_rad,
                       phases=['output', 'search'] if job['kind'] == 'sent_path' else ['diagnostic', 'search'],
                       request_budget_s=3000., return_times=True)
        started = time.monotonic()
        reply = client.request(request, timeout=3100)
        return dict({k: v for k, v in job.items() if k not in ('request', 'states')}, given_states=len(job['states']),
                    elapsed_s=time.monotonic()-started, reply=reply)
    by_client = {}
    for i in range(len(jobs)):
        by_client.setdefault(i % len(clients), []).append(i)
    results = [None]*len(jobs)

    def lane(k):
        for i in by_client[k]:
            results[i] = run(i)
    with ThreadPoolExecutor(max_workers=len(clients)) as executor:
        list(executor.map(lane, list(by_client)))
    for client in clients:
        client.close()
    totals = dict(records=len(results), checks=0, counterexamples=0, extra_rejects=0, failure_differences=0,
                  states_by_origin=dict(given=0, perturbed=0, random=0), valid=0, invalid=0)
    raw = {}; fallback = {}; mismatches = []
    for r in results:
        reply = r['reply']
        totals['checks'] += reply['checks']; totals['counterexamples'] += reply['counterexamples']
        totals['extra_rejects'] += reply['extra_rejects']; totals['failure_differences'] += reply['failure_differences']
        mismatches += [dict(m, source=r['source']) for m in reply['mismatches']]
        for phase, data in reply['phases'].items():
            totals['valid'] += data['valid']; totals['invalid'] += data['invalid']
            for origin, timing in data['timing'].items():
                totals['states_by_origin'][origin] = totals['states_by_origin'].get(origin, 0)+timing['exact']['count']
                key = (r['stage'], r['kind'], phase, origin)
                bucket = raw.setdefault(key, dict(exact=[], two_level=[]))
                bucket['exact'] += timing.get('exact_raw_s', []); bucket['two_level'] += timing.get('two_level_raw_s', [])
        eligible = reply['two_level_workspace'].get('eligible_pairs', 0)
        for phase, s in reply['two_level_workspace']['phases'].items():
            f = fallback.setdefault((r['stage'], phase), dict(states=0, pair_checks=0, exact_fcl=0, states_with_exact=0,
                                                               proxy_cleared=0, aabb_after_proxy=0))
            f['states'] += s['queries']; f['pair_checks'] += s['queries']*eligible; f['exact_fcl'] += s['narrow_calls']
            f['states_with_exact'] += s['states_with_exact']
            f['proxy_cleared'] += s['proxy_constant_cleared']+s['proxy_group_cleared']+s['proxy_sphere_cleared']+s['proxy_unvisited_cleared']
            f['aabb_after_proxy'] += s['exact_candidates']-s['narrow_calls']

    def quant(values):
        if not values:
            return None
        v = sorted(values)
        return dict(count=len(v), median_ms=1e3*v[len(v)//2], p90_ms=1e3*v[min(len(v)-1, int(.9*len(v)))],
                    max_ms=1e3*v[-1], mean_ms=1e3*statistics.fmean(v))
    timing = [dict(stage=k[0], kind=k[1], phase=k[2], origin=k[3], exact=quant(v['exact']), two_level=quant(v['two_level']))
              for k, v in sorted(raw.items())]
    per_stage = {}
    for k, v in raw.items():
        if k[1] == 'sent_path' and k[2] == 'output' and k[3] == 'given':
            s = per_stage.setdefault(k[0], dict(exact=[], two_level=[]))
            s['exact'] += v['exact']; s['two_level'] += v['two_level']
    stage_summary = {stage: dict(exact=quant(v['exact']), two_level=quant(v['two_level'])) for stage, v in per_stage.items()}
    fallback_rows = [dict(stage=k[0], phase=k[1], **v,
                          exact_fallback_ratio_of_pairs=v['exact_fcl']/v['pair_checks'] if v['pair_checks'] else None,
                          proxy_cleared_ratio_of_pairs=v['proxy_cleared']/v['pair_checks'] if v['pair_checks'] else None,
                          states_needing_exact_ratio=v['states_with_exact']/v['states'] if v['states'] else None)
                     for k, v in sorted(fallback.items())]
    report = dict(schema='eco65_collision_equivalence_v1', scope='SYNTHETIC_SCENE_RECORDS_NOT_FIELD_DATA',
                  inputs=[str(p) for p in args.records], totals=totals, mismatches=mismatches[:50],
                  sent_path_output_single_state_by_stage=stage_summary, timing=timing, fallback=fallback_rows,
                  records=[{k: v for k, v in r.items() if k != 'reply'} | dict(
                      checks=r['reply']['checks'], counterexamples=r['reply']['counterexamples'],
                      extra_rejects=r['reply']['extra_rejects'], build_s=r['reply']['build_s']) for r in results])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps(dict(totals=totals, per_stage={k: dict(exact_median_ms=v['exact']['median_ms'], two_level_median_ms=v['two_level']['median_ms'])
                                                    for k, v in stage_summary.items()}), indent=1))


if __name__ == '__main__':
    main()
