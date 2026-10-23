from pathlib import Path
from collections import Counter
import hashlib,io,json,sys,tarfile
import numpy as np
work=Path('/work-c01-ext100-v2-2f6e1ea');base=Path('/c01-ext100-v2-20261023');run=base/'formal-v1';out=base/'analysis-v1'
sys.path[:0]=[str(work/'src'),str(work/'tools')]
from plan_m710_mount_comparison import make_context
from unloading_sim.upper_arm_preference import upper_arm
read=lambda p:json.loads(p.read_text())
save=lambda p,d:p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
out.mkdir(exist_ok=False)
summary=read(run/'summary.json');protocol=read(run/'protocol.json');verification=read(run/'verification.json')
context,built,audit=make_context(protocol['mounting_variant'],layout_path=work/protocol['active_layout_file'],robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
assert type(built.robot).__name__=='PinocchioHppFclBackend'
rows=[];total=Counter();ops=Counter();candidates=[]
for o in summary['outcomes']:
 prefix=o['evidence']['archive'].removesuffix('.tar.gz')
 with tarfile.open(run/o['evidence']['archive']) as archive:
  def get(n):
   b=archive.extractfile(prefix+'/'+n).read()
   assert hashlib.sha256(b).hexdigest()==o['evidence']['file_sha256'][n]
   return json.loads(b)
  result=get('result.json');row=dict(target_id=o['target_id'],complete=bool(result['completed_count']),wall_s=result['planning_wall_s'],attempts=[])
  for a in result['attempts']:
   option=a.get('option',{});item=dict(template=option.get('template'),height_m=option.get('height_m'),
    temporary_height_m=option.get('pre_approach_height_m'),failure=a.get('failure'),
    geometry_evidence=a.get('geometry_evidence'),screening=a.get('screening'),wall_s=a['wall_s'],
    unsearched_option_count=a.get('unsearched_option_count',0),counts={},collision_witnesses=[])
   if a.get('folder'):
    folder=a['folder'];calls=get(folder+'/requests.json');count=Counter(native_requests=len(calls));witnesses=[]
    searched=any(c['op'] in ('ik','plan') for c in calls)
    candidates.append(dict(target=o['target_id'],folder=prefix+'/'+folder,target_search_dispatched=searched,requests=len(calls)))
    for call in calls:
     ops[f"{call.get('op')}:{call.get('stage')}:{call.get('status')}"]+=1
     response=call.get('response') or {}
     if call['op']=='ik':count['ik_requests']+=1
     if call['op']=='ik' and call.get('status')=='SUCCESS':count['numerical_ik_success']+=1
     if call['op']=='plan':
      endpoint=response.get('clearance',{}).get('counts',{}).get('endpoint',{})
      if call.get('q_goal') is not None and call.get('pipeline_id')!='external_axis' and endpoint.get('queries',0)>=2 and endpoint.get('rejected',1)==0:count['explicit_target_check_passed']+=1
      if call.get('status')=='SUCCESS':count['checked_lift' if call.get('planner_id')=='COUPLED_LIFT' else call['stage']]+=1
     for w in response.get('clearance',{}).get('rejected_examples',[]):
      witnesses.append(dict(request_id=response.get('request_id'),stage=call.get('stage'),**w))
    item.update(folder=prefix+'/'+folder,counts=dict(count),collision_witness_count=len(witnesses),
      collision_witnesses=witnesses[:10],witness_display_truncated=len(witnesses)>10,
      target_search_dispatched=searched,collision_pairs=dict(Counter(' | '.join(w['pair']) for w in witnesses)))
    total.update(count)
    if folder+'/attempt.json' in o['evidence']['file_sha256']:
     detail=get(folder+'/attempt.json');item['detail_failure']=detail.get('failure')
     item['contact_complete']=detail.get('contact_complete',False)
     item['top_lift']=detail.get('top_lift')
     item['receive_sync_location']=detail.get('receive_sync_location','NOT_REACHED')
     item['cup_geometry']={section:{k:(sum(value[k]) if value[k] is not None else None) for k in ('geometrically_eligible_mask','commanded_active_mask','actual_contact_mask') if k in value} for section in ('expected_contact','terminal_contact') if (value:=detail.get(section))}
     if detail.get('contact_complete'):
      q=np.array(detail['contact_end_q_rad'])
      item['contact_configuration']=dict(q_rad=q.tolist(),q2_rad=float(q[1]),upper_arm=upper_arm(built.robot,q))
   row['attempts'].append(item)
  if result['cycles']:
   c=result['cycles'][0];q=np.array(c['attachment']['contact_q_rad'])
   row['accepted']=dict(face=c['target']['face_role'],option=c['option'],delta_m=c['target'].get('world_y_offset_m',0.),
    height_m=c['height_m'],receive_sync_location=c['receive_sync_location'],q2_rad=float(q[1]),
    actual_contact_q_rad=q.tolist(),upper_arm=upper_arm(built.robot,q),
    cups={k:sum(v) for k,v in c['attachment']['selection'].items() if k.endswith('_mask')},
    top_lift_geometry=c.get('stationary_low_receiver',{}).get('top_lift',{}).get('geometry'),
    extraction_geometry=c['extraction_geometry'],nominal_drop_m=c['nominal_release']['drop_m'],
    stages=[dict(stage=s['stage'],source=s['generation_source']) for s in c['stages']])
  rows.append(row)
for b in summary['blocked']:rows.append(dict(b,complete=False))
report=dict(source_commit=summary['source_commit'],backend=type(built.robot).__name__,new_completed=summary['completed_count'],
 new_targets=summary['target_count'],historical_completed=summary['historical_completed_count'],cumulative_completed=summary['historical_completed_count']+summary['completed_count'],
 remaining=len(summary['remaining_target_ids']),rows=rows,counts=dict(total),operations=dict(ops),
 task_wall_s=summary['native_task_wall_s'],loop_wall_s=summary['planning_wall_s'],costs_are_nested_not_additive=True,
 candidates=candidates,actual_target_search_candidates=sum(c['target_search_dispatched'] for c in candidates),
 worker_initialized_candidates=len(candidates),physical_execution='NOT_RUN',qualification_status='NOT_EVALUATED',
 continuous_swept_proof=False,execution_ready=False)
save(out/'all-targets.json',report)
manifest=read(work/'source-manifest.json')
for n,sha in manifest['sha256'].items():assert hashlib.sha256((work/n).read_bytes()).hexdigest()==sha,n
save(out/'source-check.json',dict(status='PASS',source_commit=summary['source_commit'],source_files=len(manifest['sha256']),
 worker_sha256=hashlib.sha256(Path('/l01-tool-v2-build/m710_moveit_worker').read_bytes()).hexdigest(),
 trace_sha256=hashlib.sha256(Path('/l01-tool-v2-install/lib/libm710_search_trace.so').read_bytes()).hexdigest()))
print(json.dumps(dict(completed=report['new_completed'],targets=report['new_targets'],cumulative=report['cumulative_completed'],counts=total)))


# Preserve every source record, including all rejected numerical requests.
files={};rawout=base/'formal-v1-raw.tar.xz'
with tarfile.open(rawout,'w:xz',preset=3) as dest:
 def add(name,b):
  if name in files:raise ValueError('DUPLICATE_EVIDENCE_FILE')
  files[name]=hashlib.sha256(b).hexdigest();m=tarfile.TarInfo(name);m.size=len(b);m.mode=0o644;dest.addfile(m,io.BytesIO(b))
 for f in sorted(run.iterdir()):
  if f.is_file() and f.name.endswith('.tar.gz'):
   with tarfile.open(f) as source:
    for m in source:
     if m.isfile():add('formal-v1/'+m.name,source.extractfile(m).read())
  elif f.is_file():add('formal-v1/'+f.name,f.read_bytes())
 for name in ('formal-v1.log','formal-v1.exit','focused-tests.log','production-check.log','formal-v1.sh','native-build-reuse.json'):
  add(name,(base/name).read_bytes())
 add('production-check/initial-state-check.json',(base/'production-check/initial-state-check.json').read_bytes())
 raw=json.dumps(files,indent=2).encode();m=tarfile.TarInfo('file-sha256.json');m.size=len(raw);dest.addfile(m,io.BytesIO(raw))
save(base/'raw-receipt.json',dict(file=rawout.name,sha256=hashlib.sha256(rawout.read_bytes()).hexdigest(),bytes=rawout.stat().st_size,files=len(files)))
