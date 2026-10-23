from pathlib import Path
import json,tarfile,collections,hashlib
p=Path('artifacts/m710_min250_20261023')
rows=[]
for version in ('v1','v2','sync'):
 archive=p/(f'min250-{version}-formal-raw.tar.xz' if version!='sync' else 'min250-sync-regression.tar.xz')
 counts=collections.Counter();ops=collections.Counter();candidates=[];tasks=[]
 with tarfile.open(archive) as t:
  manifests=json.load(t.extractfile('file-sha256.json')) if version!='sync' else None
  for m in t.getmembers():
   if not m.isfile():continue
   if manifests and m.name in manifests:
    raw=t.extractfile(m).read();assert hashlib.sha256(raw).hexdigest()==manifests[m.name]
   elif m.name.endswith('requests.json') or m.name.endswith('/result.json'):raw=t.extractfile(m).read()
   else:continue
   if m.name.endswith('/result.json'):
    r=json.loads(raw);tasks.append(dict(file=m.name,wall_s=r['planning_wall_s'],complete=r['completed_count']))
   if not m.name.endswith('requests.json'):continue
   calls=json.loads(raw);counts['native_requests']+=len(calls)
   candidates.append(dict(file=m.name,requests=len(calls),target_search_dispatched=any(c.get('op') in ('ik','plan') for c in calls)))
   for c in calls:
    ops[str(c.get('op'))+':'+str(c.get('stage'))+':'+str(c.get('status'))]+=1
    response=c.get('response') or {}
    if c.get('op')=='ik' and c.get('status')=='SUCCESS':counts['numerical_ik_success']+=1
    if c.get('op')=='plan':
     endpoint=response.get('clearance',{}).get('counts',{}).get('endpoint',{})
     if c.get('q_goal') is not None and c.get('pipeline_id')!='external_axis' and endpoint.get('queries',0)>=2 and endpoint.get('rejected',1)==0:counts['target_check_passed']+=1
     if c.get('status')=='SUCCESS':counts['checked_lift' if c.get('planner_id')=='COUPLED_LIFT' else str(c.get('stage'))]+=1
 rows.append(dict(run=version,counts=dict(counts),operations=dict(ops),initialized_candidates=sum(c['requests']>0 for c in candidates),dispatched_candidates=sum(c['target_search_dispatched'] for c in candidates),candidates=candidates,task_wall_s=sum(t['wall_s'] for t in tasks),tasks=tasks,population_credit=0 if version=='sync' else (3 if version=='v1' else 5)))
print(json.dumps([dict(run=r['run'],counts=r['counts'],candidates=r['dispatched_candidates'],task_wall_s=r['task_wall_s']) for r in rows]))
(p/'all-attempt-costs.json').write_text(json.dumps(dict(costs_are_nested_not_additive=True,runs=rows),indent=2)+'\n')
for version in ('v1','v2'):
 a=json.loads((p/f'download-{version}/analysis-{version}/all-targets.json').read_text())
 for r in a['rows']:
  if r['complete']: print(r['target_id'],r['accepted']['synchronization'],r['accepted']['temporary_height_m'],r['accepted']['actual_contact_upper_arm'])
