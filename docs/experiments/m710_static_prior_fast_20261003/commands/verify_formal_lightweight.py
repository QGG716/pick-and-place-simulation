"""Read archived metadata/endpoints only; no planning or geometry validation."""
import hashlib,json,math,re,subprocess
from pathlib import Path
from collections import Counter
A=Path(__file__).resolve().parent
P=A/'formal-once'
ROOT=Path('/root/autodl-tmp/m710-moveit2-20260922/rootfs/work-static-prior-20261003')
t=json.loads((P/'timing.json').read_text()); archive=json.loads((P/'trajectories.json').read_text())
w=json.loads((P/'write_timing.json').read_text());tr=archive['trajectories']
diag=[json.loads(line) for line in (P/'requests.jsonl').read_text().splitlines()]
byid={r['stage_id']:r for r in diag if r.get('stage_id')}
checks=[];per=[];marker='PLANNING_ONLY_NOT_EXECUTABLE';skip='SKIPPED_PLANNING_ONLY'
def check(name,ok,**details):
 checks.append(dict(name=name,result='CONSISTENT' if ok else 'INCONSISTENT',**details))
def close(a,b):return math.isclose(a,b,rel_tol=0.,abs_tol=1e-10)
def blob(path):
 return subprocess.check_output(['git','show',t['source_commit']+':'+path],cwd=ROOT,text=True)
config_path='configs/validation/m710id70_layout_poc_pair_clearance.yaml'
config=blob(config_path)
q=json.loads(re.search(r'^  q_rad: (\[.*\])$',config,re.M).group(1))
check('initial_q_from_frozen_configuration',q==t['initial_q_rad']==t['boxes'][0]['start_q_rad']==tr[0]['path'][0],configuration=config_path,source_commit=t['source_commit'],initial_q_rad=q)
check('complete_five_distinct_top_row_targets',len(tr)==len(t['boxes'])==t['completed_count']==5 and set(t['completed_targets'])=={f'carton_l07_c{i:02d}' for i in range(5)} and not t['missing_targets'],actual_order=t['completed_targets'])
check('initial_scene_40_cartons',len(t['initial_cartons'])==40)
check('unique_task_per_carton',len({s['task_id'] for s in tr})==5)
check('four_exact_cross_box_connections',all(tr[i-1]['path'][-1]==tr[i]['path'][0]==t['boxes'][i]['start_q_rad'] for i in range(1,5)),numeric_comparison='Exact JSON numeric equality; no tolerance or wrapping')
check('planning_only_markers',t['status']==archive['status']==w['status']==marker and t['qualification_status']==archive['qualification_status']==w['qualification_status']=='NOT_EVALUATED' and archive['execution_ready'] is False and all(v==skip for v in t['skipped_checks'].values()) and t['execution_requests']==t['isaac_launches']==0 and t['executable_bundle_generated'] is False)
initial={b['id']:b for b in t['initial_cartons']};completed=[];selected_sources=Counter()
for box,s in zip(t['boxes'],tr):
 target=s['target'];task=s['task_id'];calls={v['stage_id']:v for v in box['calls']['stages']}
 motions=s['native_stages'];events=s['semantic_events'];receipt=s['final_state_receipt'];ctx=receipt['context']
 records={m['stage_id']:{'kind':'motion',**m} for m in motions}
 for e in events:
  ev=e['event'];records[ev['stage_id']]={'kind':'event','stage':'place','stage_id':ev['stage_id'],'parent_stage_id':ev['parent_stage_id'],**e}
 chain=[];current=receipt['stage_id'];seen=set();valid=True
 while current:
  if current not in records or current in seen:valid=False;break
  seen.add(current);record=records[current];chain.append(record);current=record['parent_stage_id']
 chain.reverse();valid=valid and len(seen)==len(records)
 prev=box['start_q_rad'];boundary_index=0;endpoint_ok=True;identity_ok=box['task_id']==task==receipt['task_id']
 source_ok=True;motion_ranges=[]
 for r in chain:
  call=calls.get(r['stage_id'],{});d=byid.get(r['stage_id'],{})
  identity_ok &= (r['stage_id'].startswith(task+':') and d.get('task_id')==task and d.get('parent_stage_id')==r['parent_stage_id'] and call.get('process_policy',{}).get('target_id')==target and call.get('status')=='SUCCESS')
  source=r['generation_source'];selected_sources[source]+=1
  if r['kind']=='motion':
   pts=r['points'];endpoint_ok &= len(pts)>=2 and pts[0]['q']==prev
   first=boundary_index;boundary_index+=len(pts)-1;prev=pts[-1]['q']
   endpoint_ok &= boundary_index<len(s['path']) and s['path'][first]==pts[0]['q'] and s['path'][boundary_index]==prev
   motion_ranges.append(dict(stage=r['stage'],stage_id=r['stage_id'],parent_stage_id=r['parent_stage_id'],path_range=[first,boundary_index],generation_source=source))
   source_ok &= r.get('planning_only_status')==marker and r.get('native_output_status')==skip and r.get('authoritative_status')==skip
   if source=='NATIVE_GENERATED':source_ok &= sum(r['native_solver_calls'].values())>0
   elif source in {'STATIC_PRIOR_REUSE','NATIVE_DIRECT_CONNECTION'}:
    u=r['prior_usage'];source_ok &= r['native_solver_calls']=={} and u['context_checked'] is True and u['current_geometry_checked'] is True and u['static_clearance_inherited'] is False and u['connector_kind']=='CHECKED_JOINT_INTERPOLATION'
    if source=='STATIC_PRIOR_REUSE':source_ok &= u['prior_hit'] is True and u['prior_nodes_reused']>0
    else:source_ok &= u['prior_hit'] is False and u['prior_nodes_reused']==u['prior_edges_reused']==0 and u['connector_edges']==1
   else:source_ok=False
  else:
   ev=r['event'];endpoint_ok &= r['terminal_q']==prev
   source_ok &= source=='SEMANTIC_EVENT' and r['points']==[] and r['native_solver_calls']=={} and ev['event_type']=='PLACE_TARGET_REACHED' and ev['goal_constraints_satisfied'] is True and ev['start_unchanged'] is True
   identity_ok &= ev['task_id']==task and ev['target_id']==target
 endpoint_ok &= prev==s['path'][-1]==box['end_q_rad']==receipt['q_rad'] and boundary_index==len(s['path'])-1
 labels=[r['stage'] for r in chain]
 check(target+'.complete_selected_stage_chain',valid and endpoint_ok and labels==['pregrasp','contact','extraction','transit','transit','place','withdrawal'],stage_labels=labels,selected_motion_stages=len(motions),semantic_place_events=len(events),motion_ranges=motion_ranges)
 check(target+'.task_target_and_generation_binding',identity_ok and source_ok)
 ranges=s['stage_ranges'];evmap={e['event']:e['index'] for e in s['events']}
 check(target+'.ordered_attach_release_retreat',evmap.get('ATTACH')==ranges['contact'][1]==ranges['extraction'][0] and ranges['extraction'][1]==ranges['transit'][0] and ranges['transit'][1]==ranges['place'][0]==ranges['place'][1]==evmap.get('RELEASE')==ranges['withdrawal'][0] and evmap.get('RELEASE_RETREAT_COMPLETE')==ranges['withdrawal'][1]==len(s['path'])-1 and ranges['withdrawal'][1]>ranges['withdrawal'][0] and box['ideal_remove_after_withdrawal']==target)
 transit_calls=[calls[r['stage_id']]['process_policy'] for r in chain if r['stage']=='transit']
 check(target+'.normal_transit_permissions',all(p['attached'] is True and p['relaxed_stack'] is False and p['support_names']==[] for p in transit_calls))
 withdrawal=calls[receipt['stage_id']]['process_policy']
 release_context=(ctx['stage']=='withdrawal' and ctx['attached'] is False and ctx['target_id']==ctx['target']['id']==target and ctx['target']['pose']==s['place']['actual_box_pose_world'] and withdrawal['attached'] is False and withdrawal['target_id']==target and withdrawal['relaxed_stack'] is False and withdrawal['state_checks']>0)
 check(target+'.placed_target_retained_in_withdrawal_context',release_context,evidence_scope='Recorded accepted withdrawal receipt and native process summary; frozen source passes [*obstacles, placed] to withdrawal. Full request world inventory was intentionally not archived.')
 other={b['id']:b for b in ctx['other_world'] if b['id'] in initial}
 remaining=set(initial)-set(completed)-{target}
 check(target+'.unremoved_neighbor_geometry_unchanged',set(other)==remaining and all(other[k]['pose']==initial[k]['pose'] and other[k]['size']==initial[k]['size'] for k in remaining) and box['scene_cartons']==40-len(completed),remaining_neighbor_count=len(other))
 check(target+'.markers_and_no_history',s['status']==marker and s['qualification_status']=='NOT_EVALUATED' and s['execution_ready'] is False and all(v==skip for v in s['skipped_checks'].values()) and box['calls']['history_and_legacy']=={'history_enabled':False,'history_inputs_read':0,'legacy_motion_generator_calls':0,'forbidden_entry_attempts':0})
 completed.append(target);per.append(dict(target=target,task_id=task,plan_s=box['plan_s'],complete=box['complete'],native_counts=box['calls']['counts'],selected_stage_count=len(chain),semantic_place_events=len(events)))
plan=sum(b['plan_s'] for b in t['boxes']);scene=sum(b['scene_update_s'] for b in t['boxes']);slow=sum(b['slow_completion_plan_s'] for b in t['boxes']);fast=sum(b['fast_phase_plan_s'] for b in t['boxes'])
check('planning_time_accounting',close(plan,t['T_plan_5_s']) and close(plan,t['cumulative_attempted_plan_s']) and all(close(sum(v['plan_s'] for v in t['boxes'][:i+1]),b['cumulative_plan_s']) for i,b in enumerate(t['boxes'])),T_plan_5_s=plan,rule='Sum each box plan wall time once; native/IK/IPC/prior subtotals are not added.')
check('delivery_time_accounting',close(t['batch_wall_s'],plan+scene+t['batch_bookkeeping_s']) and close(w['T_online_5_s'],t['batch_wall_s']+w['result_file_write_s']) and close(scene,t['first_box_scene_preparation_s']+t['inter_box_scene_update_s']),T_online_5_s=w['T_online_5_s'],batch_wall_s=t['batch_wall_s'],scene_update_s=scene,batch_bookkeeping_s=t['batch_bookkeeping_s'],result_file_write_s=w['result_file_write_s'],excluded=w['excluded'],initialization_s_excluded=t['initialization_s'])
check('fast_slow_partition',close(plan,fast+slow) and close(slow,t['slow_completion_plan_s']) and t['fast_phase_completed_count']==0 and t['target_1s_achieved'] is False,fast_planning_s=fast,slow_completion_planning_s=slow,shared_fast_phase_budget_s=t['fast_phase_budget_s'])
check('no_ompl_calls',sum(b['calls']['counts']['OMPL'] for b in t['boxes'])==0)
check('no_diagnostic_or_historical_answers',not any(arg in t['command'] for arg in ['--diagnostic-input','--prepare-only']) and t['development_prepare_only'] is False and 'diagnostic_input' not in t)
worker=A/'worker-frozen';db=A/'static-prior-v3.json'
hash_file=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
check('frozen_worker_and_database_hashes',hash_file(worker)==t['environment']['worker_sha256'] and hash_file(db)==t['static_prior']['sha256'],worker_sha256=hash_file(worker),database_sha256=hash_file(db))
result=dict(schema='m710_planning_only_lightweight_verification_v1',run_id=t['run_id'],status=marker,qualification_status='NOT_EVALUATED',audit_status='CONSISTENT_WITH_AVAILABLE_EVIDENCE' if all(c['result']=='CONSISTENT' for c in checks) else 'INCONSISTENCY_FOUND',source_commit=t['source_commit'],scope='Read archived summaries, selected stage endpoints, parent IDs, event indices and accepted scene contexts. No planning, per-sample path audit, collision checking or execution qualification.',checks=checks,boxes=per,selected_generation_sources=dict(selected_sources),diagnostic_request_count=len(diag),input_sha256={n:hash_file(P/n) for n in ['timing.json','trajectories.json','requests.jsonl','write_timing.json']},limitations=['Full submitted request payloads/world inventories were intentionally not saved. Withdrawal target presence is supported by accepted target context, native process evidence and frozen code, not an independent native scene snapshot.','Conditional zero-length support-release is folded between contact and extraction; there is no separate native motion stage for it. No missing motion is invented.','Only stage boundaries and metadata were inspected; interior geometry, dense path/source coverage, TCP contracts, dynamics, execution and real unloading cycle time remain NOT_EVALUATED.','The immutable initial configuration itself was derived before this batch. This review confirms use of that configuration, not a new physical measurement.'])
(A/'verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps({k:result[k] for k in ['audit_status','run_id','source_commit','selected_generation_sources','diagnostic_request_count']}))
print('checks',len(checks),'inconsistencies',[c['name'] for c in checks if c['result']!='CONSISTENT'])
print('boxes',json.dumps(per))
