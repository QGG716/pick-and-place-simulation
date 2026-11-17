"""Latency bench for the ECO65 real-site chain: planning, vision and scheduling.

Replays every available saved task record (outputs/single_carton/<task_id>/
with observation.json, observation.npz and segmentation.npz) through
tools/replay_single_carton_record.py with ECO65_TIMING=1, and/or reads timing
files already written with ECO65_TIMING=1 (timing-observe.json,
timing-prepare_plan.json, timing-execute.json from workbench task folders,
timing-scheduling.json from tools/measure_scheduling_fake_io.py). Reports
p50 / p95 / max, success rate, a per-stage breakdown and the stages over the
budget table (configs/workbench/planning_budget.json; reporting only).

No hardware, device, SDK or valve is touched. No record is ever modified; the
replay tool hashes each record before and after. With no input it reports
NO_SAMPLES instead of inventing numbers.

Usage:
  python tools/bench_planning_latency.py --discover [--planning-deadline-s 2.0]
  python tools/bench_planning_latency.py --records DIR [DIR ...] --timing FILE_OR_DIR [...]
"""
import argparse
from collections import defaultdict
from datetime import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
RECORD_FILES = ('observation.json', 'observation.npz', 'segmentation.npz')


def discover(root):
    root = Path(root)
    if not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if p.is_dir() and all((p/n).is_file() for n in RECORD_FILES))


def quantile(values, q):
    """Nearest-rank quantile (no interpolation between samples)."""
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(q*len(ordered))-1)]


def stats(values):
    return dict(count=len(values), p50_s=quantile(values, .5), p95_s=quantile(values, .95),
                max_s=max(values) if values else None)


def match(event, spec):
    name, _, label = spec.partition(':')
    if event.get('name') != name:
        return False
    if not label:
        return True
    value = event.get('record') if event.get('record') is not None else event.get('file')
    return value is not None and str(value).split(':')[0] == label


def span_sum(events, specs):
    intervals=sorted((e['start_monotonic_s'],e['start_monotonic_s']+e['duration_s'])
                     for e in events if not e.get('aggregate') and e.get('start_monotonic_s') is not None and any(match(e,s) for s in specs))
    total=0.;end=-math.inf
    for lo,hi in intervals:
        total+=max(0.,hi-max(lo,end));end=max(end,hi)
    return total


def stage_values(events, section):
    out = {}
    for stage in section['stages']:
        value = span_sum(events, stage['spans'])
        if stage.get('minus_spans_inside') and any(e.get('name') == 'search.sequential_plan_candidate' for e in events):
            value -= span_sum(events, stage['minus_spans_inside'])
        out[stage['name']] = max(0., value)
    return out


def load_timing(paths):
    files = []
    for p in map(Path, paths):
        files += sorted(p.rglob('timing-*.json')) if p.is_dir() else [p]
    return [(f, json.loads(f.read_text())) for f in files]


def first(events, name):
    return next((e for e in events if e.get('name') == name), None)


def samples_from_timing(doc, path):
    """(kind, sample) pairs from one timing file."""
    events = doc.get('events', []); phase = doc.get('phase')
    if phase == 'prepare_plan':
        ready = first(events, 'prepare_plan.entry_to_plan_ready')
        whole = first(events, 'prepare_plan.entry_to_return')
        duration = ready['duration_s'] if ready else (whole['duration_s'] if whole else None)
        return [('planning', dict(source=str(path), task_id=doc.get('task_id'), seconds=duration,
                                  success=doc.get('final_phase') == 'PLAN_READY', events=events,
                                  outcome=doc.get('outcome',doc.get('final_phase')),scope='PREPARE_PLAN_ENTRY_TO_PLAN_READY' if ready else 'PREPARE_PLAN_ENTRY_TO_FAILED_RETURN'))]
    if phase == 'observe':
        ready = first(events, 'observe.entry_to_target_ready')
        return [('vision', dict(source=str(path), task_id=doc.get('task_id'),
                                seconds=ready['duration_s'] if ready else None,
                                success=doc.get('final_phase') == 'TARGET_READY', events=events))]
    if phase == 'offline_replay':
        return []  # replay files are read through their replay report (see replay_samples)
    if phase == 'scheduling_fake_io':
        sends = sorted(x['monotonic_s'] for x in doc.get('fake_io_first_sends', []))
        entered = doc.get('entered_monotonic_s')
        return [('scheduling', dict(source=str(path), task_id=doc.get('task_id'),
                                    **scheduling_values((sends[0]-entered) if sends and entered is not None else None,doc.get('handoffs',{}).get('boundaries',[])),
                                    success=doc.get('result') == 'COMPLETE', events=events, fake_io=True))]
    if phase == 'execute':
        return [('scheduling', dict(source=str(path), task_id=doc.get('task_id'),
                                    **scheduling_values(field_first_send(path, events),field_handoffs(path)), success=doc.get('final_state') == 'COMPLETE',
                                    events=events, fake_io=False))]
    return []


def scheduling_values(first_send,boundaries):
    complete=first_send is not None and len(boundaries)==4 and all(b.get('excess_s') is not None for b in boundaries)
    return dict(seconds=first_send+sum(b['excess_s'] for b in boundaries) if complete else None,
                first_send_s=first_send,handoffs=boundaries,
                partial_seconds=first_send if not complete else None,
                scope='FULL_EXECUTE_AND_FOUR_HANDOFFS' if complete else 'PARTIAL_MISSING_HANDOFF_TIMESTAMPS')


def field_first_send(path, events):
    """Field task folder: first 'send' of its first motion task minus the execute entry span start."""
    entry = first(events, 'execute.entry')
    motions = sorted(Path(path).parent.glob('motion/*/events.jsonl'))
    sends = []
    for m in motions:
        for line in m.read_text().splitlines():
            row = json.loads(line)
            if row.get('kind') == 'send':
                sends.append(row['sent_s']); break
    return (min(sends)-entry['start_monotonic_s']) if entry and sends else None


def field_handoffs(path):
    from unloading_sim.workbench.single_carton.scheduling_metrics import read_handoffs
    return read_handoffs(Path(path).parent/'events.jsonl',Path(path).parent/'motion')


def replay(record, out, deadline):
    """Run the replay tool with timing on; returns (report, timing) or raises."""
    command = [sys.executable, str(ROOT/'tools/replay_single_carton_record.py'), str(record), '--output', str(out)]
    if deadline is not None:
        command += ['--planning-deadline-s', str(deadline)]
    env = dict(os.environ, ECO65_TIMING='1')
    log = out.parent/(out.name+'.log')
    with open(log, 'w') as stream:
        code = subprocess.run(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT).returncode
    report = json.loads((out/'replay-report.json').read_text()) if (out/'replay-report.json').is_file() else None
    timing = json.loads((out/'timing-replay.json').read_text()) if (out/'timing-replay.json').is_file() else None
    return code, report, timing, log


def replay_samples(record, code, report, timing, log):
    events = (timing or {}).get('events', [])
    entry = first(events, 'replay.planning_entry_to_search_end')
    tail = span_sum(events, ['plan_cycle.binding', 'authority.register_digest', 'json.dumps:complete-plan.json',
                             'disk.write:complete-plan.json', 'prepare_plan.cycle_ready'])
    found = bool(report and report.get('complete_plan'))
    budget = (report or {}).get('planning_budget') or {}
    planning = dict(source=str(record), task_id=Path(record).name, replay_exit=code, log=str(log),
                    seconds=entry['duration_s'] if entry else None, success=found, events=events,
                    budget_status=budget.get('status'), source_record_unchanged=(report or {}).get('source_record_unchanged'),
                    scope='PARTIAL_REPLAY_NOT_PREPARE_PLAN',
                    definition='replay worker start through search, measured tail and blocking cleanup; excludes prepare_plan/Cycle.ready')
    vision = dict(source=str(record), task_id=Path(record).name, seconds=None, success=bool(report and report.get('boxes')),
                  events=events, partial_seconds=span_sum(events, ['observe.reconstruct', 'observe.planner_build',
                                                                    'observe.self_filter_construct', 'observe.build_scene']),
                  definition='REPLAY_PARTIAL: reconstruct, scene model and build_scene only; no freeze or live inference')
    return [('planning', planning), ('vision', vision)]


def summarize(kind, samples, section, factor):
    measured = [s for s in samples if s.get('seconds') is not None]
    values = [s['seconds'] for s in measured]
    out = dict(metric=section.get('metric'), samples=len(samples), timed_samples=len(values),
               success_rate=(sum(bool(s['success']) for s in samples)/len(samples)) if samples else None,
               **stats(values), status='MEASURED' if values else 'NO_SAMPLES')
    stages = defaultdict(list)
    for s in samples:
        for name, value in stage_values(s['events'], section).items():
            stages[name].append(value)
    rows, over = [], []
    for stage in section['stages']:
        row = dict(name=stage['name'], target_s=stage.get('target_s'), **stats(stages.get(stage['name'], [])))
        rows.append(row)
        if row['target_s'] is not None and row['p95_s'] is not None and row['p95_s'] > row['target_s']:
            over.append(dict(name=stage['name'], p95_s=row['p95_s'], target_s=row['target_s'],
                             ratio=row['p95_s']/row['target_s'] if row['target_s'] else None,
                             above_flag_factor=row['p95_s'] > factor*row['target_s']))
    partial=any('PARTIAL' in s.get('scope','') for s in measured)
    if partial:out['status']='PARTIAL_SCOPE'
    out['outcomes']={name:sum((s.get('outcome') or s.get('budget_status') or ('SUCCESS' if s['success'] else 'FAILED'))==name for s in samples)
                     for name in sorted({s.get('outcome') or s.get('budget_status') or ('SUCCESS' if s['success'] else 'FAILED') for s in samples})}
    out.update(stages=rows, over_budget=sorted(over, key=lambda x: -(x['ratio'] or math.inf)))
    if section.get('total_s') is not None and out['p95_s'] is not None:
        out['target_total_s'] = section['total_s']; out['meets_total_p95'] = None if partial else out['p95_s'] <= section['total_s']
    out['per_sample'] = [{k: v for k, v in s.items() if k != 'events'} for s in samples]
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', nargs='*', default=[], type=Path, help='saved task record folders to replay')
    parser.add_argument('--discover', action='store_true', help='replay every record under outputs/single_carton')
    parser.add_argument('--timing', nargs='*', default=[], help='timing-*.json files or folders containing them')
    parser.add_argument('--budget', type=Path, default=ROOT/'configs/workbench/planning_budget.json')
    parser.add_argument('--planning-deadline-s', type=float, help='passed to every replay')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    budget = json.loads(args.budget.read_text())
    if budget.get('schema') != 'eco65_latency_budget_v1':
        raise SystemExit('LATENCY_BUDGET_SCHEMA_REQUIRED')
    allocated = sum(s['target_s'] for s in budget['planning']['stages'])
    if abs(allocated-budget['planning']['total_s']) > 1e-9:
        raise SystemExit(f'PLANNING_BUDGET_ALLOCATION_{allocated}_NOT_EQUAL_TOTAL')
    records = list(args.records)+(discover(ROOT/'outputs/single_carton') if args.discover else [])
    out = (args.output or ROOT/'outputs/single_carton_development'/f'bench-{datetime.now():%Y%m%dT%H%M%S}').resolve()
    out.mkdir(parents=True, exist_ok=False)
    samples = defaultdict(list); replays = []
    for record in records:
        code, report, timing, log = replay(record, out/f'replay-{Path(record).name[:8]}', args.planning_deadline_s)
        replays.append(dict(record=str(record), exit=code, log=str(log), report=bool(report), timing=bool(timing)))
        for kind, sample in replay_samples(record, code, report, timing, log):
            samples[kind].append(sample)
    for path, doc in load_timing(args.timing):
        for kind, sample in samples_from_timing(doc, path):
            samples[kind].append(sample)
    factor = budget.get('flag_factor', 1.2)
    result = dict(schema='eco65_latency_bench_v1', generated=datetime.now().isoformat(timespec='seconds'),
                  hardware_execution='NOT_RUN', budget_file=str(args.budget), budget_status=budget['status'],
                  planning_deadline_s=args.planning_deadline_s, records_requested=[str(r) for r in records],
                  replays=replays, timing_inputs=[str(p) for p, _ in load_timing(args.timing)],
                  planning=summarize('planning', samples['planning'], budget['planning'], factor),
                  vision=summarize('vision', samples['vision'], budget['vision'], factor),
                  scheduling=summarize('scheduling', samples['scheduling'], budget['scheduling'], factor))
    if not records and not args.timing:
        result['note'] = 'NO_INPUT: no saved record and no timing file; nothing measured, nothing inferred'
    (out/'bench.json').write_text(json.dumps(result, indent=1, ensure_ascii=False, default=str))
    print(json.dumps({k: (dict((x, v[x]) for x in ('status', 'samples', 'p50_s', 'p95_s', 'max_s', 'success_rate'))
                          if isinstance(v, dict) and 'status' in v else v)
                      for k, v in result.items() if k in ('planning', 'vision', 'scheduling', 'note', 'replays')},
                     indent=1, ensure_ascii=False, default=str))
    print('bench', out/'bench.json')


if __name__ == '__main__':
    main()
