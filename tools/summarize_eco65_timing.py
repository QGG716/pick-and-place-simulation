"""Summarize ECO65 timing files into the realtime baseline breakdown.

Reads ``timing-*.json`` files written with ``ECO65_TIMING=1`` (workbench task
folders, offline replay folders, the fake-IO scheduling run) and prints one
JSON document with planning, vision and scheduling breakdowns. Development
measurement only; it never reads devices or changes records.

Usage: python tools/summarize_eco65_timing.py FILE_OR_DIR [...] [--output OUT]
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path


def load(paths):
    files = []
    for p in map(Path, paths):
        files += sorted(p.rglob('timing-*.json')) if p.is_dir() else [p]
    return [(f, json.loads(f.read_text())) for f in files]


def total(events, name, **match):
    xs = [e for e in events if e['name'] == name and all(e.get(k) == v for k, v in match.items())]
    return dict(count=len(xs), seconds=round(sum(e['duration_s'] for e in xs), 6))


def by_name(events, prefix=''):
    out = defaultdict(lambda: [0, 0.])
    for e in events:
        if e['name'].startswith(prefix):
            out[e['name']][0] += 1; out[e['name']][1] += e['duration_s']
    return {k: dict(count=n, seconds=round(s, 6)) for k, (n, s) in sorted(out.items(), key=lambda x: -x[1][1])}


def stage_of(event):
    record = str(event.get('record') or '')
    for stage in ('approach', 'contact', 'detach', 'return', 'loaded-start'):
        if record.startswith(stage):
            return stage
    return event.get('stage') or 'other'


def native_breakdown(events):
    """Sum native worker segments, per-op round trips and per-stage exact-state histograms."""
    requests = [e for e in events if e['name'] == 'native.request']
    ops = defaultdict(lambda: dict(count=0, round_trip_s=0., worker_handled_s=0., request_bytes=0,
                                   request_dumps_s=0., request_parse_s=0., reply_dump_s=0.))
    segments = defaultdict(lambda: [0, 0.]); workspace = defaultdict(lambda: defaultdict(float))
    states = defaultdict(lambda: dict(count=0, sum_s=0., max_s=0., bins=defaultdict(int)))
    for e in requests:
        op = e.get('op') or 'unknown'; o = ops[op]; nt = e.get('native') or {}
        o['count'] += 1; o['round_trip_s'] += e['duration_s']; o['request_bytes'] += e.get('request_bytes') or 0
        o['request_dumps_s'] += e.get('request_dumps_s') or 0.; o['worker_handled_s'] += nt.get('handled_s') or 0.
        o['request_parse_s'] += nt.get('request_parse_s') or 0.; o['reply_dump_s'] += nt.get('previous_reply_dump_s') or 0.
        for name, s in (nt.get('segments') or {}).items():
            segments[name][0] += s['count']; segments[name][1] += s['seconds']
        for phase, w in (nt.get('workspace_by_phase') or {}).items():
            for k, v in w.items(): workspace[phase][k] += v
        stage = stage_of(e)
        for key, h in (nt.get('exact_state_check_s') or {}).items():
            phase = key.split('/')[0]
            label = f'{stage}:{phase}' if phase in ('output', 'diagnostic', 'search') else f'{stage}:{phase}'
            s = states[label]; s['count'] += h['count']; s['sum_s'] += h['sum_s']; s['max_s'] = max(s['max_s'], h['max_s'])
            for upper, n in h.get('bins_upper_s_count', []): s['bins'][upper] += n
    def quant(s, q):
        target = q*s['count']; seen = 0
        for upper in sorted(s['bins']):
            seen += s['bins'][upper]
            if seen >= target: return upper
        return s['max_s']
    exact = {k: dict(count=v['count'], sum_s=round(v['sum_s'], 4), mean_s=v['sum_s']/v['count'] if v['count'] else None,
                     p50_upper_s=quant(v, .5), p90_upper_s=quant(v, .9), p99_upper_s=quant(v, .99), max_s=v['max_s'])
             for k, v in sorted(states.items())}
    for o in ops.values():
        o['transport_overhead_s'] = round(o['round_trip_s']-o['worker_handled_s'], 6)
    return dict(requests_by_op=dict(ops),
                worker_segments={k: dict(count=n, seconds=round(s, 6)) for k, (n, s) in sorted(segments.items(), key=lambda x: -x[1][1])},
                workspace_by_phase={k: dict(v) for k, v in workspace.items()},
                exact_state_check_by_stage_and_phase=exact)


def aggregates(events, name):
    """Fixed-size aggregate rows of one name, keyed by their key."""
    return {str(e['key']): {k: e[k] for k in ('count', 'sum', 'mean', 'max', 'p50_upper', 'p90_upper', 'p99_upper', 'bytes') if k in e}
            for e in events if e.get('aggregate') and e['name'] == name}


def gc_pauses(events):
    rows = aggregates(events, 'python.gc_pause')
    return dict(by_generation=rows, note='Python collection pauses in this process while measuring')


def rehash(events):
    files = aggregates(events, 'binding.rehash_file')
    whole = aggregates(events, 'binding.rehash_all')
    detector = {k: v for k, v in files.items() if '/models/huggingface/' in k or k.startswith('models/huggingface/')}
    top = sorted(files.items(), key=lambda kv: -kv[1]['sum'])[:15]
    return dict(per_binding=whole, files=len(files), detector_model_files=detector,
                top_files=[dict(file=k, bytes=v.get('bytes'), mean_s=v['mean'], count=v['count']) for k, v in top])


def planning(events):
    entry = [e for e in events if e['name'] in ('prepare_plan.entry_to_plan_ready', 'replay.planning_entry_to_search_end')]
    writes = [e for e in events if e['name'] in ('disk.write', 'json.dumps')]
    return dict(entry_to_ready=[dict(name=e['name'], seconds=e['duration_s']) for e in entry],
                top_level=by_name(events, 'plan_cycle.'), search=by_name(events, 'search.'),
                lanes=by_name(events, 'lane.'), main_thread=by_name(events, 'main.'),
                sent_check=total(events, 'sent_check'), prepare_send_grid=total(events, 'plan.prepare_send_grid'),
                context_digest_world=total(events, 'context.digest_world'),
                authority_register_digest=total(events, 'authority.register_digest'),
                cycle_ready_digest=total(events, 'prepare_plan.cycle_ready'),
                pool=by_name(events, 'native.pool_'), main_worker_start=total(events, 'native.main_worker_process_start'),
                binding_rehash=rehash(events),
                disk=dict(json_dumps=by_name([e for e in writes if e['name'] == 'json.dumps']),
                          writes=total(events, 'disk.write'),
                          by_file=_by_file(writes)),
                native=native_breakdown(events), gc=gc_pauses(events))


def _by_file(events):
    out = defaultdict(lambda: dict(count=0, seconds=0., bytes=0))
    for e in events:
        key = f"{e['name']}:{str(e.get('file')).split(':')[0]}"
        out[key]['count'] += 1; out[key]['seconds'] += e['duration_s']; out[key]['bytes'] += e.get('bytes') or 0
    return dict(sorted(out.items(), key=lambda x: -x[1]['seconds']))


def vision(events):
    return dict(entry_to_target_ready=total(events, 'observe.entry_to_target_ready'),
                observe=by_name(events, 'observe.'), freeze=by_name(events, 'freeze.'),
                vision=by_name(events, 'vision.'), remote=by_name(events, 'remote.'),
                remote_round_trips=[{k: e.get(k) for k in ('op', 'duration_s', 'server_inference_s', 'transfer_and_protocol_s',
                                                          'sent_image_bytes', 'received_bytes')}
                                    for e in events if e['name'] == 'remote.round_trip'],
                reconstruct=total(events, 'reconstruct.box'), scene=by_name(events, 'scene.'),
                disk=_by_file([e for e in events if e['name'] in ('disk.write', 'json.dumps')]), gc=gc_pauses(events))


def scheduling(doc):
    events = doc['events']; entered = doc.get('entered_monotonic_s')
    first = sorted(x['monotonic_s'] for x in doc.get('fake_io_first_sends', []))
    submits = []
    for c in doc.get('child_submits', []):
        steps = {e['name'].replace('child.submit.', ''): e['duration_s'] for e in c['events']}
        submits.append(dict(task_id=c['task_id'], pipe_transfer_s=c.get('pipe_transfer_s'), steps=steps,
                            first_send_due_after_accept_s=None))
    return dict(execute_entry_to_first_send_s=(first[0]-entered) if first and entered else None,
                first_sends_monotonic_s=first,
                execute=by_name(events, 'execute.'), service_submit=by_name(events, 'service.submit.'),
                ipc=dict(pipe_send_by_op=aggregates(events, 'ipc.pipe_send'),
                         wait_and_receive_reply_by_op=aggregates(events, 'ipc.wait_and_receive_reply')),
                authority=by_name(events, 'authority.'), binding_rehash=rehash(events),
                child_submits=submits, cycle_submit=by_name(events, 'cycle.submit'),
                ticks_by_state=aggregates(events, 'cycle.tick'), rpc_per_tick_by_state=aggregates(events, 'cycle.rpc_per_tick'),
                rpc_by_call=aggregates(events, 'cycle.rpc'), gc=gc_pauses(events), handoffs=doc.get('handoffs'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = dict(files=[])
    for path, doc in load(args.paths):
        phase = doc.get('phase'); events = doc.get('events', [])
        item = dict(file=str(path), phase=phase, task_id=doc.get('task_id'), events=len(events),
                    measurement_overhead_s=doc.get('measurement_overhead_s'))
        if phase in ('prepare_plan', 'offline_replay', 'native_component_check'): item['planning'] = planning(events)
        if phase in ('observe', 'offline_replay'): item['vision'] = vision(events)
        if phase == 'scheduling_fake_io': item['scheduling'] = scheduling(doc)
        report['files'].append(item)
    text = json.dumps(report, indent=1, default=str)
    if args.output: args.output.write_text(text)
    print(text)


if __name__ == '__main__':
    main()
