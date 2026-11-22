"""Summary and equivalence report of repeated offline six-segment replays (2-second planning work).

Reads ``<root>/<tag>/<frame>-r<k>/`` folders written by
``tools/replay_single_carton_record.py`` with ``ECO65_TIMING=1`` (and,
optionally, ``--plan-cycle-entry``) and reports per tag and frame:

* ``planning_s`` (acquire_workers .. search_box end) and the phase split from
  ``tools/profile_planning_latency.profile`` (median, max, every value);
* the part after the search: ``measure_plan_tail`` spans (binding, package
  digests, complete-plan serialization and write, Cycle.ready digest) and the
  evidence flush, or with ``--plan-cycle-entry`` the production
  entry -> PLAN_READY interval and its own tail spans;
* the decision of every run (candidate, stage planners, velocity scales, send
  points, probe states, audits, failure categories) and, against a reference
  tag, whether it is identical and the largest sent-point difference.

Measurement only; it never changes a record.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import statistics
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from profile_planning_latency import profile  # noqa: E402

TAIL = ('plan_cycle.binding', 'authority.register_digest', 'prepare_plan.cycle_ready', 'plan_cycle.evidence_flush',
        'plan_cycle.register', 'plan_cycle.close_workers', 'native.resident_begin_task',
        'native.resident_wait_ready', 'search.payload_rest_check')


def stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return dict(n=len(values), median=round(statistics.median(values), 3), max=round(max(values), 3),
                min=round(min(values), 3), values=[round(v, 3) for v in values])


def failure_category(reason):
    parts = str(reason).split(':')
    return ':'.join(parts[:2]) if len(parts) > 1 else parts[0]


def run_summary(folder):
    folder = Path(folder)
    report = json.loads((folder/'replay-report.json').read_text())
    timing = json.loads((folder/'timing-replay.json').read_text()) if (folder/'timing-replay.json').is_file() else dict(events=[])
    events = timing['events']
    named = lambda name: [e for e in events if e['name'] == name]
    total = lambda name: sum(e['duration_s'] for e in named(name))
    out = dict(folder=str(folder), planning_s=report.get('planning_s'), measurement_overhead_s=timing.get('measurement_overhead_s'))
    entry = report.get('plan_cycle_entry')
    search = named('plan_cycle.search_box')
    if entry:
        out.update(entry_to_plan_ready_s=entry.get('entry_to_plan_ready_s'), entry_to_return_s=entry.get('entry_to_return_s'),
                   outcome=entry.get('outcome'))
        if search:
            out['planning_s'] = search[0]['duration_s']
            begin = named('replay.plan_cycle_entry')
            if begin:
                out['entry_to_search_start_s'] = search[0]['start_monotonic_s']-begin[0]['start_monotonic_s']
                if entry.get('entry_to_plan_ready_s') is not None:
                    out['search_end_to_plan_ready_s'] = (begin[0]['start_monotonic_s']+entry['entry_to_plan_ready_s']
                                                         - search[0]['start_monotonic_s']-search[0]['duration_s'])
    out['tail_spans'] = {name: round(total(name), 4) for name in TAIL if named(name)}
    dumps = [e for e in named('json.dumps') if e.get('file') == 'complete-plan.json']
    writes = [e for e in named('disk.write') if e.get('file') == 'complete-plan.json']
    out['tail_spans']['complete_plan_json_dumps'] = round(sum(e['duration_s'] for e in dumps), 4)
    out['tail_spans']['complete_plan_disk_write'] = round(sum(e['duration_s'] for e in writes), 4)
    out['tail_spans']['complete_plan_bytes'] = sum(e.get('bytes') or 0 for e in dumps)
    if not entry:
        out['tail_measured_s'] = round(sum(v for k, v in out['tail_spans'].items() if k in (
            'plan_cycle.binding', 'authority.register_digest', 'prepare_plan.cycle_ready', 'plan_cycle.evidence_flush',
            'complete_plan_json_dumps', 'complete_plan_disk_write')), 4)
    try:
        prof = profile(folder)
        out['phases'] = prof['phases']
        out['requests'] = {k: dict(count=v['count'], wall_total=v['wall_s']['total'] if v['wall_s'] else None,
                                   fixed_total=v['fixed_total_s']['total'] if v['fixed_total_s'] else None)
                           for k, v in prof['request_aggregate'].items()}
        if prof.get('selected'):
            out['audits'] = prof['selected']['audits']
    except Exception as error:
        out['profile_error'] = str(error)[:200]
    plan = report.get('complete_plan') or {}
    segments = plan.get('segments') or (entry or {}).get('segments') or []
    out['decision'] = dict(
        candidate=plan.get('candidate_index', (entry or {}).get('candidate_index')),
        detach_profile=plan.get('detach_profile'),
        segments=[dict(stage=s['stage'], planner=(s.get('planner') or {}).get('planner_id'),
                       pipeline=(s.get('planner') or {}).get('pipeline_id'),
                       velocity_scale=(s.get('planner') or {}).get('velocity_scale'),
                       send_time_scale=(s.get('planner') or {}).get('send_time_scale'),
                       points=s.get('points'), probe_states=s.get('probe_states'),
                       audit=(s.get('sent_path_audit') or {}).get('audit_method'),
                       audit_checked=(s.get('sent_path_audit') or {}).get('checked_states'))
                  for s in segments],
        failures=dict(Counter(failure_category(f.get('reason')) for f in report.get('failures', []))))
    pysample = folder/'pysample-replay.json'
    if pysample.is_file():
        p = json.loads(pysample.read_text())
        out['python_threads'] = dict(wall_s=p['wall_s'], python_thread_cpu_s=p['python_thread_cpu_s'],
                                     groups={g: dict(cpu_s=v['cpu_s'], fraction=v['fraction']) for g, v in p['groups'].items()})
    return out


def sent_points(folder):
    folder = Path(folder)
    f = folder/'measurement-segments-NOT-EXECUTABLE.json'
    if not f.is_file():
        f = folder/'complete-plan-NOT-EXECUTABLE.json'
        if not f.is_file():
            return None
        plan = json.loads(f.read_text())
        return {s['stage']: np.asarray([p['q'] for p in s['package']['prepared']['points']]) for s in plan['segments']}
    data = json.loads(f.read_text())
    return {s['stage']: np.asarray([p['q'] for p in s['prepared']['points']]) for s in data['segments']}


def compare(reference, other):
    """Decision equality and sent-point difference between two runs of one frame."""
    a, b = reference['decision'], other['decision']
    same = dict(candidate=a['candidate'] == b['candidate'], detach_profile=a['detach_profile'] == b['detach_profile'],
                planners=[(s['stage'], s['pipeline'], s['planner'], s['velocity_scale']) for s in a['segments']] ==
                         [(s['stage'], s['pipeline'], s['planner'], s['velocity_scale']) for s in b['segments']],
                send_time_scales=[s['send_time_scale'] for s in a['segments']] == [s['send_time_scale'] for s in b['segments']],
                points=[s['points'] for s in a['segments']] == [s['points'] for s in b['segments']],
                failures=a['failures'] == b['failures'])
    pa, pb = sent_points(reference['folder']), sent_points(other['folder'])
    if pa is not None and pb is not None:
        diff = {}
        for stage in pa:
            if stage in pb and pa[stage].shape == pb[stage].shape:
                diff[stage] = float(np.max(np.abs(pa[stage]-pb[stage])))
            else:
                diff[stage] = None
        same['sent_point_max_abs_diff_rad'] = diff
    return same


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('root', type=Path)
    ap.add_argument('--tags', nargs='+', required=True)
    ap.add_argument('--reference', help='reference tag for the equivalence comparison (default: first tag)')
    ap.add_argument('--json', type=Path)
    a = ap.parse_args()
    reference = a.reference or a.tags[0]
    runs = defaultdict(dict)
    for tag in a.tags:
        for folder in sorted((a.root/tag).glob('*-r*')):
            if folder.is_dir() and (folder/'replay-report.json').is_file():
                frame = folder.name.rsplit('-r', 1)[0]
                runs[tag].setdefault(frame, []).append(run_summary(folder))
    out = dict(schema='eco65_planning_2s_summary_v1', root=str(a.root), reference=reference, tags={})
    for tag in a.tags:
        frames = {}
        for frame, rows in sorted(runs[tag].items()):
            phases = defaultdict(list)
            for r in rows:
                for k, v in (r.get('phases') or {}).items():
                    if isinstance(v, (int, float)):
                        phases[k].append(v)
            tails = defaultdict(list)
            for r in rows:
                for k, v in r['tail_spans'].items():
                    tails[k].append(v)
            ref_rows = runs[reference].get(frame) or []
            frames[frame] = dict(
                runs=len(rows), planning_s=stats([r['planning_s'] for r in rows]),
                entry_to_plan_ready_s=stats([r.get('entry_to_plan_ready_s') for r in rows]),
                search_end_to_plan_ready_s=stats([r.get('search_end_to_plan_ready_s') for r in rows]),
                entry_to_search_start_s=stats([r.get('entry_to_search_start_s') for r in rows]),
                tail_measured_s=stats([r.get('tail_measured_s') for r in rows]),
                phases={k: stats(v) for k, v in phases.items()}, tail_spans={k: stats(v) for k, v in tails.items()},
                decisions=[r['decision'] for r in rows],
                decisions_identical_within_tag=all(r['decision'] == rows[0]['decision'] for r in rows),
                equivalence_vs_reference=[compare(ref_rows[0], r) for r in rows] if ref_rows else None,
                python_threads=[r.get('python_threads') for r in rows if r.get('python_threads')][:1],
                audits=[r.get('audits') for r in rows][:1])
        out['tags'][tag] = frames
    text = json.dumps(out, indent=1, ensure_ascii=False, default=str)
    if a.json:
        a.json.write_text(text)
    for tag, frames in out['tags'].items():
        for frame, f in frames.items():
            ps = f['planning_s'] or {}
            ep = f['entry_to_plan_ready_s'] or {}
            eq = f['equivalence_vs_reference'] or []
            same = all(all(v for k, v in e.items() if k != 'sent_point_max_abs_diff_rad') for e in eq) if eq else None
            print(f"{tag:14} {frame:9} n={f['runs']} planning med={ps.get('median')} max={ps.get('max')}"
                  + (f" entry->PLAN_READY med={ep.get('median')} max={ep.get('max')}" if ep else '')
                  + f" same_as_{reference}={same}")


if __name__ == '__main__':
    main()
