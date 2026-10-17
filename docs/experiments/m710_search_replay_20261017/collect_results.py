import json,csv,hashlib,gzip,shutil
from pathlib import Path
d=Path('/tmp/search-trace-20261017');w=Path('/work-single-contact-20261016')
out=w/'docs/experiments/m710_search_replay_20261017';out.mkdir(parents=True,exist_ok=True)
names=['baseline-c04-v2','baseline-c04-normal','baseline-c02-normal','revised-c04-trace','revised-c04-normal','revised-c02-normal']
rows=[]
for name in names:
    p=d/name;r=json.loads((p/'result.json').read_text());requests=[json.loads(l) for l in (p/'requests.jsonl').read_text().splitlines()]
    seen=set();costs={};checks={};scope=[]
    for req in requests:
        raw=req.get('response',{});identity=(r['run_id'],raw.get('request_id'))
        if raw.get('request_id'):
            assert identity not in seen,identity;seen.add(identity)
        if req['op']!='plan':continue
        stage=req['stage'];costs[stage]=costs.get(stage,0)+req['round_trip_s']
        clearance=raw.get('clearance',{});ws=clearance.get('workspace',{})
        scope.append(dict(request_id=raw.get('request_id'),stage=stage,status=req['status'],round_trip_s=req['round_trip_s'],
            solver_message=raw.get('solver_message'),evidence_state=raw.get('request_check_evidence'),
            pair_profile_enabled=ws.get('pair_profile_enabled'),reuse=clearance.get('native_check_reuse')))
        for phase,counts in ws.get('phases',{}).items():
            assert all(counts.get(k) is None for k in ('broad_phase_s','narrow_s','self_narrow_s','world_narrow_s'))
            for k in ('queries','aabb_tests','aabb_skips','narrow_calls','fixed_qualification_queries','fixed_aabb_qualifications','fixed_narrow_qualifications','fixed_aabb_reuses','fixed_narrow_reuses'):
                checks[k]=checks.get(k,0)+counts.get(k,0)
        if ws:assert ws['pair_profile_enabled'] is False
    row={k:r.get(k) for k in ('run_id','source_commit','worker_sha256','complete','selected_face','target_id','initialization_s','scene_prepare_s','total_planning_s','native_calls','prior_build_load_query_calls','forbidden_requests','budgets','environment','model_identity','initial_input_sha256','command')}
    row.update(name=name,trace_enabled=('trace' in name or name=='baseline-c04-v2'),motion_request_s=costs,clearance_counts=checks,
        faces=[dict(face=x['target']['face_role'],complete=x['complete'],times=x['times'],failure=x['failure']) for x in r['attempts']],
        connection_attempts=r.get('connection_attempts',[]),request_summary=scope)
    rows.append(row)
    dest=out/name;dest.mkdir(exist_ok=True)
    for filename in ('result.json','requests.jsonl','search-trace.json','worker.log'):
        if (p/filename).exists():
            with (p/filename).open('rb') as a,gzip.GzipFile(filename='',mode='wb',fileobj=(dest/(filename+'.gz')).open('wb'),mtime=0) as b:shutil.copyfileobj(a,b)
summary=dict(status='PLANNING_REPLAY_NOT_PHYSICAL_EXECUTION',qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',
    success_rate_non_regression='NOT_PROVEN_SMALL_SAMPLE',runs=rows,
    timing_semantics='total planning includes failed attempts and returned-path checks; request times are nested and are not added to task totals; trace mode is not a normal-mode performance score')
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
with (out/'timing.csv').open('w',newline='') as f:
    writer=csv.writer(f);writer.writerow(['name','complete','face','total_planning_s','pregrasp_requests_s','transit_requests_s','initialization_s','scene_prepare_s','IK','OMPL','LIN','PTP'])
    for r in rows:writer.writerow([r['name'],r['complete'],r['selected_face'],r['total_planning_s'],r['motion_request_s'].get('pregrasp'),r['motion_request_s'].get('transit'),r['initialization_s'],r['scene_prepare_s']]+[r['native_calls'].get(k,0) for k in ('IK','OMPL','LIN','PTP')])
print((out/'timing.csv').read_text())
