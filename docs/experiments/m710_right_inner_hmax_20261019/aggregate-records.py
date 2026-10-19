from pathlib import Path
import json,collections,csv,shutil,hashlib
import argparse
p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
root=args.raw;out=args.output
s=json.loads((out/'summary.json').read_text(encoding='utf-8'));names=[r[0]for r in json.loads((root/'protocol.json').read_text(encoding='utf-8'))['cases']]
agg=[]
for name,case in zip(names,s['cases']):
 x=json.loads((root/name/'result.json').read_text(encoding='utf-8'));rows=[json.loads(l)for l in (root/name/'requests.jsonl').read_text(encoding='utf-8').splitlines()]
 ids=[r['response']['request_id']for r in rows];assert len(ids)==len(set(ids))
 count=collections.Counter();clear=collections.Counter();pairs=collections.Counter();phase_times=collections.Counter();solver=collections.Counter();sources=collections.Counter();ik_s=0;ik_api=0;pair_measured=0
 for r in rows:
  v=r['response'];count[(r['op'],r.get('stage'),v['status'])]+=1
  ik_s+=v.get('actual_ik_s',0);ik_api+=v.get('solver_api_calls',0)
  solver.update(v.get('native_solver_calls',{}))
  if v.get('failure',{}):pairs[(v['status'],v['failure'].get('reason'),tuple(v['failure'].get('pair',[])))]+=1
  c=v.get('clearance')or{}
  if c.get('workspace'):
   pair_measured+=1;w=c['workspace'];assert not w['pair_profile_enabled'];assert c['native_check_reuse'] and c['fixed_narrow_qualification']
   clear['workspace_prepare_s']+=w['prepare_s'];clear['workspace_allocation_s']+=c.get('profile',{}).get('workspace_allocation_s',0)
   for phase,z in c.get('counts',{}).items():
    clear['state_checks']+=z['queries'];clear['check_s']+=z['seconds'];phase_times[phase]+=z['seconds']
   for z in w['phases'].values():
    assert z['broad_phase_s'] is None and z['narrow_s'] is None
    for key in ['aabb_tests','aabb_skips','narrow_calls','fixed_qualification_queries','fixed_aabb_qualifications','fixed_aabb_reuses','fixed_narrow_qualifications','fixed_narrow_reuses']:clear[key]+=z[key]
  proc=v.get('process_policy')or{}
  for key in ['seal_input_compilations','seal_geometry_evaluations','compression_sat_evaluations','side_distance_queries']:sources[key]+=proc.get(key,0)
 attempts=x['attempts'];times=attempts[-1]['times']if attempts else{}
 stage_names=['target_geometry_s','ik_s','pregrasp_s','contact_s','terminal_check_s','attachment_context_s','extraction_target_s','extraction_s','extraction_sample_check_s','terminal_separation_s','release_ready_check_s','region_search_s']
 entry=dict(case=name,request_identity_rule='Unique response.request_id within this fresh worker run; reject duplicates; CURRENT_REQUEST_ONLY aggregates, not parent stages',
  request_count=len(rows),request_status_counts=[dict(op=k[0],stage=k[1],status=k[2],count=n)for k,n in count.items()],
  failure_witness_counts=[dict(status=k[0],reason=k[1],pair=list(k[2]),count=n)for k,n in pairs.items()],
  ik_requests=sum(r['op']=='ik'for r in rows),ik_solver_api_calls=ik_api,ik_actual_s=ik_s,ik_internal_attempts=None,
  ik_internal_attempts_observability='NOT_OBSERVABLE',seed_scope='OMPL_ONLY_KDL_INTERNAL_RNG_NOT_CONTROLLED',
  actual_motion_solver_calls=dict(solver),clearance_aggregates=dict(clear),clearance_phase_s=dict(phase_times),process_aggregates=dict(sources),
  profile_evidence_request_count=pair_measured,pair_profile_enabled=False if pair_measured else None,pair_profile_status='NOT_MEASURED'if pair_measured else'CHECKER_NOT_CREATED',
  measured_stage_s={k:times.get(k)for k in stage_names},boundary_costs=x['boundary_costs'],
  not_reached=[]if x['complete']else['contact','receiver_sync','attachment','region_generation','RIGHT_INNER_Y','RIGHT_INNER_X','terminal_separation','release_ready_check'],
  returned_native_stage_count=len(x.get('generated_stage_evidence',[])),region_attempt_count=sum(len(a.get('region_attempts',[]))for a in attempts),
  forbidden_requests=x['forbidden_requests'],prior_calls=x['prior_build_load_query_calls'],history_and_legacy=x['history_and_legacy'])
 case['case']=name;case['measured_stage_s']=entry['measured_stage_s'];case['post_contact_native_validation']='REACHED_FRONT_ONLY'if x['complete']else'NOT_REACHED'
 agg.append(entry)
(out/'summary.json').write_text(json.dumps(s,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
(out/'request-costs.json').write_text(json.dumps(dict(scope='READ_ONLY_EXISTING_RECORDS_NO_RECHECK',cases=agg,timing='Parent and child intervals overlap; do not add solver, RTT, row and total times.'),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
keys=['case','target','seed','receiver_template','complete','planning_s','initialization_s','scene_prepare_s','prefix_attempt_count','planned_attach_events']
with (out/'timing.csv').open('w',encoding='utf-8',newline='')as f:
 w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:r[k]for k in keys}for r in s['cases'])
for name in ['protocol.json','run.sh','environment.txt','batch.log','tests-final.log','native-tests-final.log','build-final.log']:shutil.copy2(root/name,out/name)
# Raw archive copying is separate from record aggregation.
for a in agg:print(a['case'],'clearance',a['clearance_aggregates'],'process',a['process_aggregates'])
