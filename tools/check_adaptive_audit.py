"""SAMPLED vs ADAPTIVE sent-path audit on recorded validate requests (no hardware).

Every recorded sent-path validate request (``*-sent*-native.json``) of an
eligible segment (no contact permission, no support) is sent with
``sent_path_audit`` SAMPLED and ADAPTIVE on the same worker, in variants:

  recorded   the recorded request unchanged;
  cluttered  the same request plus seeded synthetic boxes placed near link
             origins of random path states (some graze or penetrate);
  bounded    a generous and a tight site workspace box.

Per request: validity, endpoint failure, first path failure (reason, pair,
distance, edge, fraction, q) and the number of native samples walked must be
identical; checked states and check time are compared. A TWO_LEVEL run of the
same requests may be added with --collision-check-mode TWO_LEVEL.

``--graze N`` adds N constructed cases: a 30 mm cube on the tool axis at the
middle state of a short sub-path, binary-searched to where the SAMPLED audit
starts to pass, then 0.5 mm closer (grazing: below the pair limit, not
touching) and 12 mm closer (penetrating). Both methods must reject.

Usage: python tools/check_adaptive_audit.py --records DIR [DIR ...] --output OUT.json
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
from pathlib import Path
import random
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
DROP = ('request_id', 'absolute_deadline_monotonic_s', 'deadline_clock_reference_monotonic_s', 'sent_path_audit',
        'scene_handle', 'collision_check_mode')


def eligible(request):
    spec = request.get('single_carton_contact') or {}
    return request.get('op') == 'validate' and 'probe_path' in request and 'world' in request and \
        not spec.get('allow_contact') and not spec.get('supports')


def collect(folders):
    jobs = []
    for folder in folders:
        for path in sorted(Path(folder).rglob('*-sent*-native.json')):
            try:
                request = json.loads(path.read_text()).get('request') or {}
            except ValueError:
                continue
            if eligible(request):
                jobs.append(dict(source=str(path), stage=request['stage'],
                                 request={k: v for k, v in request.items() if k not in DROP}))
    return jobs


def with_world(request, extra=(), bounds=None):
    from unloading_sim.workbench.controlled.trajectory import digest
    out = copy.deepcopy(request)
    out['world'] = out['world']+list(extra)
    out['scene_fingerprint'] = digest(out['world'])
    if bounds is not None:
        out['site_workspace_bounds_m'] = bounds
    return out


def cube(name, center, size, rotation=None):
    pose = np.eye(4)
    if rotation is not None:
        pose[:3, :3] = rotation
    pose[:3, 3] = center
    return dict(id=name, size=[size]*3, pose=pose.tolist(), category='obstacle')


def link_points(model, q):
    frames = model.frames(q)
    return [(model.world_from_base@frames[name])[:3, 3] for name in ('link_2', 'link_3', 'link_4', 'link_5', 'link_6') if name in frames] + \
        [model.tcp(q)[:3, 3]]


def outcome(reply):
    clearance = reply.get('clearance') or {}
    output = (clearance.get('counts') or {}).get('output') or {}
    audit = clearance.get('sent_path_audit')
    return dict(native_valid=reply.get('native_valid'), legacy_intersection_valid=reply.get('legacy_intersection_valid'),
                failure=reply.get('failure'), path_failure=reply.get('path_failure'),
                checked=output.get('queries', 0), check_s=output.get('seconds', 0.)+(audit['constraint_s']+audit['proof_s'] if audit else 0.),
                sampled=audit['sampled_states'] if audit else output.get('queries', 0),
                skipped=audit['skipped_states'] if audit else 0,
                minimum_distance_lower_bound_m=audit['minimum_distance_lower_bound_m'] if audit else None,
                reused=clearance.get('exact_repeated_path_states_reused', 0))


def same(a, b):
    keys = ('native_valid', 'legacy_intersection_valid', 'failure', 'path_failure', 'sampled')
    return all(a[k] == b[k] for k in keys), [k for k in keys if a[k] != b[k]]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--clutter-variants', type=int, default=3)
    parser.add_argument('--graze', type=int, default=4)
    parser.add_argument('--collision-check-mode', choices=('EXACT', 'TWO_LEVEL'), default='EXACT')
    parser.add_argument('--seed', type=int, default=6518)
    args = parser.parse_args()
    import check_planning_deadline as tool
    from unloading_sim.moveit2_backend import ResidentMoveItClient
    planner, model = tool.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    base_jobs = collect(args.records)
    if not base_jobs:
        raise SystemExit('NO_ELIGIBLE_RECORDED_SENT_PATH_REQUESTS')
    rng = random.Random(args.seed)
    jobs = []
    for job in base_jobs:
        request = job['request']
        if args.collision_check_mode == 'TWO_LEVEL':
            request = dict(request, collision_check_mode='TWO_LEVEL')
        jobs.append(dict(job, variant='recorded', request=request))
        path = request['probe_path']
        for v in range(args.clutter_variants):
            extra = []
            for k in range(rng.randint(2, 5)):
                q = path[rng.randrange(len(path))]
                anchor = rng.choice(link_points(model, q))
                direction = np.array([rng.gauss(0, 1) for _ in range(3)]); direction /= np.linalg.norm(direction)
                size = rng.uniform(.04, .1)
                extra.append(cube(f"clutter-{v}-{k}", anchor+direction*(size/2+rng.uniform(.06, .3)), size))
            jobs.append(dict(job, variant=f'cluttered-{v}', request=with_world(request, extra)))
        tcp_z = [model.tcp(q)[2, 3] for q in path[::max(1, len(path)//50)]]
        jobs.append(dict(job, variant='bounded-generous', request=with_world(request, bounds=[[-6., -6., -6.], [6., 6., 6.]])))
        jobs.append(dict(job, variant='bounded-tight', request=with_world(request, bounds=[[-6., -6., min(tcp_z)+.03], [6., 6., 6.]])))
    clients = []
    for k in range(min(args.workers, len(jobs))):
        client = ResidentMoveItClient(['env', 'ROS_LOG_DIR=/tmp/adaptive-audit', str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                      log_path=f'/tmp/adaptive-audit-{k}.log')
        ready = client.request(planner.init, timeout=180)
        if not ready.get('capabilities', {}).get('adaptive_sent_path_audit'):
            raise SystemExit('NATIVE_WORKER_UPDATE_REQUIRED_FOR_ADAPTIVE_SENT_PATH_AUDIT')
        clients.append(client)

    def send(client, request, method):
        payload = dict(request, op='validate', request_budget_s=900.)
        if method == 'ADAPTIVE':
            payload['sent_path_audit'] = 'ADAPTIVE'
        started = time.monotonic()
        reply = client.request(payload, timeout=1000)
        return reply, time.monotonic()-started

    def compare(client, request):
        send(client, dict(request, probe_path=request['probe_path'][:2]), 'SAMPLED')  # scene import, not timed
        a, ta = send(client, request, 'SAMPLED')
        b, tb = send(client, request, 'ADAPTIVE')
        oa, ob = outcome(a), outcome(b)
        identical, differences = same(oa, ob)
        return dict(identical=identical, differences=differences, sampled=oa, adaptive=ob, sampled_wall_s=ta, adaptive_wall_s=tb)

    results = [None]*len(jobs)

    def lane(k):
        for i in range(k, len(jobs), len(clients)):
            job = jobs[i]
            results[i] = dict({key: job[key] for key in ('source', 'stage', 'variant')}, states=len(job['request']['probe_path']),
                              **compare(clients[k], job['request']))
    with ThreadPoolExecutor(max_workers=len(clients)) as executor:
        list(executor.map(lane, range(len(clients))))

    # Constructed grazing / penetrating obstacles on the tool axis.
    graze = []
    candidates = [j for j in base_jobs if len(j['request']['probe_path']) > 60][:args.graze]

    def graze_case(k):
        job = candidates[k]; client = clients[k % len(clients)]
        path = job['request']['probe_path']; middle = len(path)//2
        sub = dict(job['request'], probe_path=path[max(0, middle-30):middle+30])
        if args.collision_check_mode == 'TWO_LEVEL':
            sub['collision_check_mode'] = 'TWO_LEVEL'
        tcp = model.tcp(path[middle]); axis = tcp[:3, 2]

        def at(d):
            return with_world(sub, [cube('graze', tcp[:3, 3]+axis*(d+.015), .03, tcp[:3, :3])])

        def valid(d):
            reply, _ = send(client, at(d), 'SAMPLED')
            return bool(reply.get('native_valid') and reply.get('path_failure') is None)
        lo, hi = -.3, .3
        if valid(lo) or not valid(hi):
            return dict(source=job['source'], status='NO_THRESHOLD_ON_AXIS')
        for _ in range(16):
            mid = (lo+hi)/2
            if valid(mid):
                hi = mid
            else:
                lo = mid
        row = dict(source=job['source'], stage=job['stage'], threshold_offset_m=hi, cases=[])
        for name, d in (('grazing', hi-.0005), ('penetrating', hi-.012), ('clear', hi+.002)):
            result = compare(client, at(d))
            row['cases'].append(dict(case=name, offset_m=d, **result,
                                     surface_distance_m=(result['sampled']['path_failure'] or {}).get('surface_distance_m'),
                                     reason=(result['sampled']['path_failure'] or {}).get('reason')))
        return row
    with ThreadPoolExecutor(max_workers=len(clients)) as executor:
        graze = list(executor.map(graze_case, range(len(candidates))))
    for client in clients:
        client.close()

    def total(rows):
        return dict(requests=len(rows), identical=sum(r['identical'] for r in rows),
                    valid=sum(bool(r['sampled']['native_valid'] and r['sampled']['path_failure'] is None) for r in rows),
                    sampled_checked=sum(r['sampled']['checked'] for r in rows), adaptive_checked=sum(r['adaptive']['checked'] for r in rows),
                    adaptive_skipped=sum(r['adaptive']['skipped'] for r in rows),
                    sampled_check_s=sum(r['sampled']['check_s'] for r in rows), adaptive_check_s=sum(r['adaptive']['check_s'] for r in rows),
                    sampled_wall_s=sum(r['sampled_wall_s'] for r in rows), adaptive_wall_s=sum(r['adaptive_wall_s'] for r in rows))
    by_variant = {}
    for r in results:
        by_variant.setdefault(r['variant'].split('-')[0], []).append(r)
    passing = [r for r in results if r['adaptive']['native_valid'] and r['adaptive']['path_failure'] is None]
    graze_cases = [c for g in graze for c in g.get('cases', [])]
    report = dict(
        schema='eco65_adaptive_audit_equivalence_v1', scope='RECORDED_SENT_PATHS_NOT_PHYSICAL_ACCEPTANCE',
        collision_check_mode=args.collision_check_mode, inputs=[str(p) for p in args.records], seed=args.seed,
        totals=total(results), by_variant={k: total(v) for k, v in by_variant.items()},
        by_stage={s: total([r for r in results if r['stage'] == s and r['variant'] == 'recorded'])
                  for s in sorted({r['stage'] for r in results})},
        acceptance=dict(
            all_identical=all(r['identical'] for r in results),
            adaptive_pass_implies_sampled_pass=all(r['sampled']['native_valid'] and r['sampled']['path_failure'] is None for r in passing),
            adaptive_passes=len(passing),
            constructed_grazing_and_penetrating_rejected_by_both=all(
                not c['sampled']['native_valid'] or c['sampled']['path_failure'] is not None for c in graze_cases if c['case'] != 'clear') and all(
                not c['adaptive']['native_valid'] or c['adaptive']['path_failure'] is not None for c in graze_cases if c['case'] != 'clear'),
            constructed_cases_identical=all(c['identical'] for c in graze_cases),
            constructed_cases=len(graze_cases)),
        mismatches=[r for r in results if not r['identical']][:20], graze=graze,
        requests=[{k: (v if k not in ('sampled', 'adaptive') else {x: v[x] for x in ('native_valid', 'checked', 'skipped', 'sampled', 'check_s', 'minimum_distance_lower_bound_m')})
                   for k, v in r.items()} for r in results])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps(dict(totals=report['totals'], by_variant=report['by_variant'], acceptance=report['acceptance']), indent=1))


if __name__ == '__main__':
    main()
