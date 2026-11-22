"""Step-0 latency profile of one offline six-segment replay (ECO65_TIMING=1 output).

Reads ``timing-replay.json``, ``replay-report.json`` and the selected
candidate's record folder of a ``tools/replay_single_carton_record.py`` output
and reports, without re-running anything:

* the phase split of ``plan_cycle.search_box`` (candidate generation,
  screening, lane planning wait, final audits, holding post-check);
* every native request of the selected candidate, matched exactly through
  the ``transport_and_worker_s`` value the planner writes into both the record
  file and the timing event, split into fixed costs (Python dump + IPC,
  per-request scene diff / world import, clearance workspace construction,
  legacy start collision) and variable costs (Pilz / OMPL solve, in-planner
  output check, waypoint conversion, path audit);
* the critical path of the selected candidate's chain(s): per segment the
  first request start, the last prepare end and how much of it was native
  fixed cost, native variable cost and Python ``prepare``;
* every segment's final audit: probe states, exactly checked states, skipped
  states, worker check seconds and the per-exact-state cost;
* aggregate fixed / variable costs over all requests of the replay.

Measurement only; it never changes a record.
"""
import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

FIXED_SEGMENTS = ('clearance.construct_and_workspace', 'scene.diff_acm_attachment', 'world.import_new_scene',
                  'world.compare_same_fingerprint', 'plan.start_legacy_collision', 'validate.legacy_collision',
                  'validate.reply_evidence')
SOLVE_SEGMENTS = ('plan.solve_pilz', 'plan.solve_ompl', 'plan.solution_predicate_check', 'plan.solution_path_check',
                  'plan.solution_to_points', 'plan.start_goal_checks', 'validate.check_path', 'validate.endpoint_state', 'ik')
CHAIN_STAGES = ('approach', 'contact', 'detach', 'transfer', 'withdraw', 'return_empty')


def seg_seconds(native, name):
    s = (native.get('segments') or {}).get(name)
    if isinstance(s, dict):
        return float(s.get('seconds', s.get('total_s', 0.)) or 0.)
    return 0.


def split_request(event):
    """wall = python dump + transport/IPC + worker fixed + worker variable (+ unattributed worker time)."""
    native = event.get('native') or {}
    wall = float(event['duration_s'])
    handled = float(native.get('handled_s') or 0.)
    fixed = {k: seg_seconds(native, k) for k in FIXED_SEGMENTS}
    variable = {k: seg_seconds(native, k) for k in SOLVE_SEGMENTS}
    out = dict(wall_s=wall, handled_s=handled, python_dump_s=float(event.get('request_dumps_s') or 0.),
               transport_and_ipc_s=max(0., wall-handled-float(event.get('request_dumps_s') or 0.)),
               worker_fixed_s=sum(fixed.values()), worker_variable_s=sum(variable.values()),
               workspace_s=fixed['clearance.construct_and_workspace'],
               scene_setup_s=fixed['scene.diff_acm_attachment']+fixed['world.import_new_scene']+fixed['world.compare_same_fingerprint'],
               legacy_collision_s=fixed['plan.start_legacy_collision']+fixed['validate.legacy_collision'],
               request_bytes=event.get('request_bytes'))
    out['worker_other_s'] = max(0., handled-out['worker_fixed_s']-out['worker_variable_s'])
    out['fixed_total_s'] = out['python_dump_s']+out['transport_and_ipc_s']+out['worker_fixed_s']+out['worker_other_s']
    out['segments'] = {k: v for k, v in {**fixed, **variable}.items() if v}
    return out


def stage_of(record):
    for s in CHAIN_STAGES:
        if record == s or record.startswith(s+'-'):
            return s
    if record.startswith('detach'):
        return 'detach'
    return record.split('-')[0]


def summarize(values):
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), total=round(sum(values), 4), median=round(statistics.median(values), 4),
                max=round(values[-1], 4))


def profile(folder):
    folder = Path(folder)
    timing = json.loads((folder/'timing-replay.json').read_text())
    report = json.loads((folder/'replay-report.json').read_text())
    events = [e for e in timing['events'] if e.get('start_monotonic_s') is not None]
    first = lambda name: next((e for e in events if e['name'] == name), None)
    search = first('plan_cycle.search_box')
    t0 = search['start_monotonic_s']
    rel = lambda e: e['start_monotonic_s']-t0
    end = lambda e: e['start_monotonic_s']+e['duration_s']-t0
    named = lambda name: [e for e in events if e['name'] == name]
    plan = report.get('complete_plan') or {}
    selected = plan.get('candidate_index', plan.get('index'))
    if selected is None:
        for seg in (plan.get('segments') or []):
            selected = seg.get('candidate_index', selected)
    box_ids = [b['id'] for b in report.get('boxes', [])]
    # Phases of the target search.
    contact = first('search.contact_candidates')
    lanes = first('search.parallel_candidates')
    waits = named('main.wait_planned_candidate')+named('main.wait_replanned_candidate')
    verifies = named('main.verify_candidate')
    concurrent = named('sent_check.concurrent')
    phases = dict(planning_s=round(search['duration_s'], 4))
    if contact:
        phases['payload_rest_check_s'] = round(sum(e['duration_s'] for e in named('search.payload_rest_check')), 4)
        phases['contact_candidates_s'] = round(contact['duration_s'], 4)
        if lanes:
            phases['screening_s'] = round(rel(lanes)-end(contact), 4)
    if lanes:
        phases['lanes_start_s'] = round(rel(lanes), 4)
    phases['lanes_wait_s'] = round(sum(e['duration_s'] for e in waits), 4)
    phases['verify_s'] = round(sum(e['duration_s'] for e in verifies), 4)
    phases['concurrent_audit_s'] = round(sum(e['duration_s'] for e in concurrent), 4)
    phases['verify_other_s'] = round(phases['verify_s']-phases['concurrent_audit_s'], 4)
    for name, key in (('replay.planning_entry_to_search_end', 'replay_entry_to_search_end_s'),
                      ('prepare_plan.cycle_ready', 'prepare_plan_cycle_ready_s'),
                      ('replay.worker_prewarm', 'worker_prewarm_s'), ('observe.planner_build', 'planner_build_s')):
        e = first(name)
        if e:
            phases[key] = round(e['duration_s'], 4)
    phases['register_digest_s'] = round(sum(e['duration_s'] for e in named('authority.register_digest')), 4)
    # Every native request of the replay.
    requests = named('native.request')
    by_key = defaultdict(list)
    for e in requests:
        by_key[(e.get('record'), e.get('transport_and_worker_s'))].append(e)
    kinds = defaultdict(list)
    for e in requests:
        op, record, thread = e.get('op'), e.get('record') or '', e['thread']
        if op == 'plan':
            kind = 'plan_'+('success' if e.get('status') == 'SUCCESS' else 'failed')
        elif op == 'validate' and '-sent' in record:
            kind = 'audit'
        elif op == 'validate':
            kind = 'endpoint_validate'
        else:
            kind = op or 'other'
        kinds[kind].append(split_request(e))
    aggregate = {}
    for kind, rows in sorted(kinds.items()):
        aggregate[kind] = dict(count=len(rows),
                               **{k: summarize([r[k] for r in rows]) for k in
                                  ('wall_s', 'fixed_total_s', 'workspace_s', 'scene_setup_s', 'legacy_collision_s',
                                   'transport_and_ipc_s', 'python_dump_s', 'worker_variable_s')})
    # Selected candidate: exact request matching through transport_and_worker_s.
    chain = None
    if selected is not None and box_ids:
        sub = folder/f'{box_ids[0]}-{selected}'
        matched = []
        if sub.is_dir():
            for f in sorted(sub.glob('*-native.json')):
                d = json.loads(f.read_text())
                res = d.get('result') or {}
                key = (f.name[:-len('-native.json')], res.get('transport_and_worker_s'))
                hits = by_key.get(key) or []
                # segment-native.json duplicates the last attempt; the attempt file has the same key.
                if len(hits) == 1:
                    matched.append((f.name, hits[0], res))
        seen = set()
        rows = []
        for name, e, res in sorted(matched, key=lambda x: x[1]['start_monotonic_s']):
            if id(e) in seen:
                continue
            seen.add(id(e))
            row = dict(record=e.get('record'), op=e.get('op'), status=e.get('status'), start_s=round(rel(e), 4),
                       end_s=round(end(e), 4), worker=str(res.get('request_id', '')).rsplit('-', 1)[0],
                       thread=e['thread'], **{k: (round(v, 4) if isinstance(v, float) else v)
                                              for k, v in split_request(e).items() if k != 'segments'},
                       segments={k: round(v, 4) for k, v in split_request(e)['segments'].items()})
            rows.append(row)
        # Python prepare spans of the selected chain: same thread name, after the
        # segment's successful native request, same record prefix, before the next
        # native request of that thread (tags make this exact when present).
        prepares = named('plan.prepare_send_grid')
        tagged = any('candidate' in p for p in prepares)
        segments = {}
        for st in CHAIN_STAGES:
            reqs = [r for r in rows if r['op'] == 'plan' and stage_of(r['record']) == st]
            if not reqs:
                continue
            success = [r for r in reqs if r['status'] == 'SUCCESS']
            last = success[-1] if success else reqs[-1]
            if tagged:
                mine = [p for p in prepares if p.get('candidate') == selected and p.get('stage') == st]
            else:
                nxt = [r['start_s'] for r in rows if r['thread'] == last['thread'] and r['start_s'] > last['end_s']]
                horizon = min(nxt) if nxt else 1e9
                mine = [p for p in prepares if p['thread'] == last['thread'] and p.get('stage') == st
                        and last['end_s']-1e-6 <= rel(p) <= horizon]
            prep = sum(p['duration_s'] for p in mine)
            seg_end = max([end(p) for p in mine] or [last['end_s']])
            segments[st] = dict(requests=len(reqs), failed_requests=sum(r['status'] != 'SUCCESS' for r in reqs),
                                first_request_start_s=reqs[0]['start_s'], native_last_end_s=last['end_s'],
                                prepare_calls=len(mine), prepare_s=round(prep, 4),
                                prepare_factors=[p.get('factor') for p in mine], segment_end_s=round(seg_end, 4),
                                native_wall_s=round(sum(r['wall_s'] for r in reqs), 4),
                                native_fixed_s=round(sum(r['fixed_total_s'] for r in reqs), 4),
                                native_workspace_s=round(sum(r['workspace_s'] for r in reqs), 4),
                                native_variable_s=round(sum(r['worker_variable_s'] for r in reqs), 4),
                                prepare_attribution='TAGGED' if tagged else 'THREAD_AND_TIME_WINDOW')
        audits = {}
        for st in CHAIN_STAGES:
            f = sub/f'{st}-sent-check.json'
            if not f.is_file():
                continue
            d = json.loads(f.read_text())
            a = d.get('sent_path_audit') or {}
            check = []
            for c in d.get('chunks') or []:
                r = c.get('result') or {}
                hit = [x for x in rows if x['op'] == 'validate' and x['record'] and x['record'].startswith(st+'-sent')
                       and abs((x.get('handled_s') or 0)-0) >= 0 and r.get('request_wall_s') is not None]
                check.append(r.get('request_wall_s'))
            audit_rows = [x for x in rows if x['record'] and x['record'].startswith(st+'-sent')]
            check_path = sum(x['segments'].get('validate.check_path', 0.) for x in audit_rows)
            checked = a.get('checked_states') or 0
            audits[st] = dict(method=a.get('audit_method'), probe_states=a.get('probe_states'), checked_states=checked,
                              sampled_states=a.get('sampled_states'), skipped_states=a.get('skipped_states'),
                              chunks=len(d.get('chunks') or []), wall_s=round(d.get('wall_s', 0.), 4),
                              worker_check_path_s=round(check_path, 4),
                              worker_fixed_s=round(sum(x['fixed_total_s'] for x in audit_rows), 4),
                              seconds_per_exact_state=round(check_path/checked, 6) if checked else None)
        chain = dict(candidate=selected, requests=rows, segments=segments, audits=audits,
                     requests_matched=len(rows))
    return dict(schema='eco65_planning_latency_profile_v1', folder=str(folder), task_id=report.get('task_id'),
                status=plan.get('status'), phases=phases, request_aggregate=aggregate, selected=chain,
                failures=sorted({str(f.get('reason', '')).split(':')[0] for f in report.get('failures', [])}),
                failure_count=len(report.get('failures', [])))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folders', nargs='+')
    ap.add_argument('--json')
    a = ap.parse_args()
    out = [profile(f) for f in a.folders]
    for p in out:
        print('==', p['task_id'], p['status'], json.dumps(p['phases']))
        for kind, agg in p['request_aggregate'].items():
            print(f"   {kind:18} n={agg['count']:4} wall={agg['wall_s']['total']:7.2f} fixed={agg['fixed_total_s']['total']:7.2f}"
                  f" ws={agg['workspace_s']['total']:6.2f} (med {agg['workspace_s']['median']:.3f}) var={agg['worker_variable_s']['total']:7.2f}")
        s = p['selected']
        if s:
            for st, seg in s['segments'].items():
                print(f"   {st:13} req={seg['requests']} failed={seg['failed_requests']} start={seg['first_request_start_s']:6.2f}"
                      f" native_end={seg['native_last_end_s']:6.2f} end={seg['segment_end_s']:6.2f} fixed={seg['native_fixed_s']:.2f}"
                      f" ws={seg['native_workspace_s']:.2f} var={seg['native_variable_s']:.2f} prep={seg['prepare_s']:.2f}x{seg['prepare_calls']}")
            for st, au in s['audits'].items():
                print(f"   audit {st:13} {au['method']} probe={au['probe_states']} exact={au['checked_states']} skipped={au['skipped_states']}"
                      f" chunks={au['chunks']} wall={au['wall_s']:.2f} check={au['worker_check_path_s']:.2f} per_exact={au['seconds_per_exact_state']}")
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
