import hashlib,json
from pathlib import Path
D=Path('/root/autodl-tmp/m710-clearance-perf-20260928/evidence/final2')
def read(n):return json.loads((D/(n+'.json')).read_text())
a,b=read('fixed-reference'),read('fixed-optimized')
assert a['states']==b['states'] and a['input_sha256']==b['input_sha256']
x,y=dict(a['request']),dict(b['request']);x.pop('clearance_mode');y.pop('clearance_mode');assert x==y
max_difference=0
for i,(r,o,p) in enumerate(zip(a['native']['states'],b['native']['states'],a['authority'])):
 assert r['valid']==o['valid']==p['valid'],i
 if not r['valid']:
  assert set(r['failure'].get('pair',[]))==set(o['failure'].get('pair',[])),i
  assert r['failure']['required_pair_clearance_m']==o['failure']['required_pair_clearance_m']
  if r['failure'].get('surface_distance_m') is not None and r['failure']['surface_distance_m']>0:
   max_difference=max(max_difference,abs(r['failure']['surface_distance_m']-o['failure']['surface_distance_m']))
for mode in ('reference','optimized'):
 assert read('historical-'+mode)['status']=='PASS'
 assert read('synthetic-'+mode)['status']=='PASS'
 assert read('fixed-'+mode)['status']=='PASS'
r,o=read('loaded-reference'),read('loaded-optimized')
assert o['results'][0]['failure'] is None
assert o['results'][0]['authoritative_status']=='PASS'
rr=[json.loads(x) for x in (D/'loaded-reference-requests.jsonl').read_text().splitlines()]
ro=[json.loads(x) for x in (D/'loaded-optimized-requests.jsonl').read_text().splitlines()]
x,y=dict(rr[0]),dict(ro[0]);x.pop('clearance_mode');y.pop('clearance_mode');assert x==y
accepted=read('first-accepted-loaded');assert accepted['path']==o['results'][0]['path']
assert read('timing-contract')['status']=='PASS'
cap=read('capability');assert cap['native_calls']==[] and cap['heavy_state_or_edge_checks']==0
assert all(x['failure'] is None for x in read('small')['results'])
summary=dict(schema='m710_clearance_performance_summary_v1',status='PASS',same_requests_except_mode=True,
 fixed_count=len(a['states']),unique_states=a['unique_states'],maximum_positive_failure_distance_difference_native_m=max_difference,
 loaded_reference_failure=r['results'][0]['failure']['reason'],loaded_optimized_status='PASS',
 diagnostic_only='NOT_RUN: optimized passed within the original fixed budget',
 first_final_path_sha256=hashlib.sha256((D/'first-accepted-loaded.json').read_bytes()).hexdigest(),
 fixed={},timing_contract=read('timing-contract'),capability={'end_to_end_s':cap['end_to_end_s'],'candidate_s':cap['results'][0]['end_to_end_s'],'native_calls':0,'heavy_checks':0})
for mode,p in [('reference',a),('optimized',b)]:
 c=p['native']['clearance'];q=c['counts']['fixed_set'];summary['fixed'][mode]=dict(queries=q['queries'],valid=q['valid'],rejected=q['rejected'],seconds=q['seconds'],mean_ms=q['seconds']*1000/q['queries'],states_per_s=q['queries']/q['seconds'],profile=c['profile'],workspace=c['workspace'],legacy_intersection_s=p['native']['legacy_intersection_s'],evidence_materialization_s=c['evidence_materialization_s'],bookkeeping_s=q['bookkeeping_s'])
(D/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k not in ('timing_contract','fixed')},indent=2))
