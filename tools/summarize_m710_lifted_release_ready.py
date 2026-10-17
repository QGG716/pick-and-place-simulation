"""Read-only aggregation of this scoped native experiment; never rechecks geometry."""
from pathlib import Path
import argparse, csv, hashlib, json
from collections import Counter


def compact_evidence(value):
    """Keep measured aggregates; full repeated static policies stay in raw records."""
    if isinstance(value, list):
        return [compact_evidence(item) for item in value]
    if isinstance(value, dict):
        return {key: compact_evidence(item) for key, item in value.items() if key != 'policy'}
    return value


def summarize(directory):
    result=json.loads((directory/'result.json').read_text())
    requests=[json.loads(line) for line in (directory/'requests.jsonl').read_text().splitlines()]
    unique={}
    for r in requests:
        key=r.get('response',{}).get('request_id')
        if key is None:continue
        if key in unique and unique[key]!=r:raise ValueError('CONFLICTING_REQUEST_ID')
        unique[key]=r
    request_cost=Counter();calls=Counter();external=[]
    for r in unique.values():
        key='external_axis' if r.get('pipeline_id')=='external_axis' else r['op']+':'+str(r.get('stage'))
        request_cost[key]+=r['round_trip_s'];calls[key]+=1
        if r.get('pipeline_id')=='external_axis':external.append(r['response'].get('external_axis'))
        clearance=r.get('response',{}).get('clearance')
        if clearance and clearance.get('workspace'):
            assert clearance['workspace']['pair_profile_enabled'] is False
            for phase in clearance['workspace']['phases'].values():
                assert phase['broad_phase_s'] is None and phase['narrow_s'] is None
    rows=[]
    for row in result['attempts']:
        rows.extend(row.get('previous_prefix_attempts',[]));rows.append(row)
    times=Counter()
    for row in rows:
        for key,value in row['times'].items():
            if key!='total_s' and value is not None:times[key]+=value
    plan=result.get('plan');continuity=None
    if plan:
        previous=None;previous_id='';continuity=True
        for stage in plan['stages']:
            points=stage['points'];first=points[0]['q'];last=points[-1]['q']
            assert stage['parent_stage_id']==previous_id
            if previous is not None:assert first==previous
            previous=last;previous_id=stage['stage_id']
        assert previous==plan['end_q_rad']
        assert plan['target_still_attached'] and not any(e.get('event_type')=='RELEASE' for e in plan['events'])
    return dict(target=result['target_id'],run_id=result['run_id'],source_commit=result['source_commit'],
        worker_sha256=result['worker_sha256'],model_identity=result['model_identity'],budgets=result['budgets'],
        environment=result['environment'],command=result['command'],initial_carton_count=len(result['initial_cartons']),
        start_source=result['start_source'],start_q_rad=result['start_q_rad'],complete=result['complete'],
        initialization_s=result['initialization_s'],scene_prepare_s=result['scene_prepare_s'],
        planning_s=result['total_planning_s'],row_times_s=dict(times),request_round_trip_s=dict(request_cost),
        request_counts=dict(calls),native_calls=result['native_calls'],prior_calls=result['prior_build_load_query_calls'],
        pair_profile_enabled=False,exact_selected_q_and_parent_continuity=continuity,
        initial_receiver_checks=compact_evidence(rows[0].get('initial_receiver_checks',[])),
        external_axis_attempts=compact_evidence(external),
        receiver_sync_attempts=[{key: item[key] for key in
            ('sync_stage','parent_stage_id','q_rad','status','round_trip_s')}
            for row in rows for item in row.get('receiver_sync_attempts',[])],
        prefix_attempt_count=len(rows),region_attempts=[a for row in rows for a in row.get('region_attempts',[])],
        failure=result['attempts'][-1].get('failure'),release_ready=None if not plan else plan['release_ready'],
        status=result['status'],qualification_status='NOT_EVALUATED',continuous_swept_proof=False,
        physical_execution='NOT_RUN',
        timing_semantics='Planning inclusive of all attempts; row/request/solver/check times are nested. Do not sum columns. External axis has no execution time law.')


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rows=[summarize(run) for run in args.runs]
    (args.output/'summary.json').write_text(json.dumps(dict(cases=rows),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    keys=['target','complete','planning_s','initialization_s','scene_prepare_s','prefix_attempt_count']
    with (args.output/'timing.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader()
        writer.writerows({key:row[key] for key in keys} for row in rows)
    print(json.dumps([{key:row[key] for key in keys} for row in rows]))

if __name__=='__main__':main()
