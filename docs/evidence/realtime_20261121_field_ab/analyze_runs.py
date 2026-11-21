"""Summarize matched offline field replays; never treats them as PLAN_READY samples."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import statistics


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def summarize(folder):
    report = read(folder/'replay-report.json')
    events = read(folder/'timing-replay.json')['events']
    spans = defaultdict(list)
    for e in events:
        if not e.get('aggregate') and 'duration_s' in e:
            spans[e['name']].append(e)
    seconds = lambda name: sum(e['duration_s'] for e in spans[name])
    plan = report.get('complete_plan')
    allocations = []
    allocation_groups = defaultdict(list)
    for p in sorted(folder.rglob('*-native.json')):
        native = read(p)
        clearance = (native.get('result') or {}).get('clearance') or {}
        value = (clearance.get('profile') or {}).get('workspace_allocation_s')
        if value is None:
            continue
        allocations.append(value)
        group = 'sent_audit' if '-sent-' in p.name else ('endpoint' if 'endpoint' in p.name or 'loaded-start' in p.name else 'other')
        allocation_groups[group].append(value)
    allocation = dict(measured_constructions=len(allocations),
        sum_across_workers_s=sum(allocations), median_s=statistics.median(allocations) if allocations else None,
        interpretation='SUM_ACROSS_REQUESTS_AND_WORKERS_NOT_ELAPSED_WALL_TIME',
        by_group={k: dict(count=len(v), sum_s=sum(v), median_s=statistics.median(v)) for k,v in allocation_groups.items()})
    return dict(folder=str(folder), source_record=report['record'], source_hashes=report['source_hashes'],
        source_unchanged=report['source_record_unchanged'], mechanisms=report['planning_mechanisms'],
        search_and_audit_s=report.get('planning_s'),
        replay_entry_to_search_end_s=seconds('replay.planning_entry_to_search_end'),
        prewarm_s=report.get('worker_prewarm_s', 0.),
        process_teardown_s=seconds('replay.process_teardown'),
        endpoint_screening_s=seconds('search.screen_candidate'),
        wait_candidate_s=seconds('main.wait_planned_candidate')+seconds('main.wait_replanned_candidate'),
        sent_audits=[dict(stage=e.get('stage'), seconds=e['duration_s']) for e in spans['sent_check']],
        workspace_construction=allocation,
        complete=plan is not None, candidate_index=plan['candidate_index'] if plan else None,
        segments=plan['segments'] if plan else [],
        failure_counts=dict(Counter(f['reason'].split(':')[0] for f in report['failures'])),
        resident_task_start=report.get('resident_task_start'),
        permits_issued=report['permits_issued'], physical_result=report['physical_result'],
        report_sha256=sha(folder/'replay-report.json'), timing_sha256=sha(folder/'timing-replay.json'))


def compare(a, b):
    left, right = Path(a['folder']), Path(b['folder'])
    result = dict(record=left.name.rsplit('-',1)[0], source_hashes_identical=a['source_hashes']==b['source_hashes'],
        candidate_priority_identical=read(left/'candidate-priority.json')==read(right/'candidate-priority.json'),
        search_speedup=a['search_and_audit_s']/b['search_and_audit_s'],
        entry_speedup=a['replay_entry_to_search_end_s']/b['replay_entry_to_search_end_s'])
    if not a['complete'] or not b['complete']:
        return dict(result, accepted_plan_comparison='UNAVAILABLE')
    x=read(left/'measurement-segments-NOT-EXECUTABLE.json')
    y=read(right/'measurement-segments-NOT-EXECUTABLE.json')
    result['accepted_decisions']={k:x[k]==y[k] for k in ('q_start','candidate','target','attachment','release')}
    result['stages'] = [dict(stage=s['stage'], same_stage=s['stage']==t['stage'],
        prepared_exactly_equal=s['prepared']==t['prepared'],
        prepared_sha256_baseline=digest(s['prepared']), prepared_sha256_accelerated=digest(t['prepared']),
        baseline_audit=s['evidence']['sent_path_collision'], accelerated_audit=t['evidence']['sent_path_collision'],
        baseline_method=s['evidence']['collision_result']['sent_path_audit'],
        accelerated_method=t['evidence']['collision_result']['sent_path_audit'])
        for s,t in zip(x['segments'], y['segments'])]
    result['accepted_plan_identical']=(len(x['segments'])==len(y['segments'])==4
        and all(result['accepted_decisions'].values()) and all(s['same_stage'] and s['prepared_exactly_equal'] for s in result['stages']))
    result['scope']='SELECTED_PLAN_AND_CANDIDATE_ORDER_ON_SAVED_INPUT_NOT_GENERAL_EQUIVALENCE'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    runs={p.name:summarize(p) for p in sorted(args.runs.iterdir())
        if p.is_dir() and p.name.endswith(('-baseline','-accelerated')) and (p/'timing-replay.json').is_file()}
    pairs=[compare(a,runs[k.removesuffix('-baseline')+'-accelerated']) for k,a in runs.items()
        if k.endswith('-baseline') and k.removesuffix('-baseline')+'-accelerated' in runs]
    result=dict(schema='eco65_saved_field_planning_ab_v1',date='2026-11-21',
        scope='OFFLINE_REPLAY_NOT_PREPARE_PLAN_NOT_EXECUTABLE',manifest=read(args.runs/'manifest.json'),
        runs=runs,comparisons=pairs,
        official_successful_plan_ready_samples_added=0,field_p95_s=None,physical_result='NOT_RUN',
        limitations=['One trial per saved input and configuration; no p95 claim.',
          'Frozen geometry excludes live capture, inference and observation-to-planning delay.',
          'Warm worker cost and process teardown are separate; existing workbench was left running.',
          'PYTHON orchestration only; NATIVE branch fallback and GPU prepare equivalence are not accepted.',
          'This task did not modify field settings or thresholds; concurrent field changes are recorded in the manifest. Benchmarks used unchanged separate copies.'])
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(dict(runs={k:dict(seconds=v['search_and_audit_s'],candidate=v['candidate_index'],complete=v['complete']) for k,v in runs.items()},
        comparisons=[{k:v for k,v in p.items() if k!='stages'} for p in pairs]),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
