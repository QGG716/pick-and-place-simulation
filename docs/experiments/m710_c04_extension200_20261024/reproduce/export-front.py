"""Read-only c04 front-contact and rejected LIN states, using existing viewer."""
from pathlib import Path
import copy,hashlib,json,sys,tarfile
import numpy as np
work=Path('/work-c04-extension200-20b3ca7')
base=Path('/c04-extension200-v2-20261024/front');out=base/'analysis-v1'
sys.path[:0]=[str(work/'src'),str(work/'tools')]
from plan_m710_mount_comparison import make_context
from export_m710_search_replay import export_model
with tarfile.open(base/'formal-v1-raw.tar.xz') as t:
    hashes=json.load(t.extractfile('file-sha256.json'))
    blobs={n:t.extractfile(n).read() for n in hashes}
for n,sha in hashes.items():assert hashlib.sha256(blobs[n]).hexdigest()==sha
read=lambda n:json.loads(blobs['formal-v1/'+n])
protocol=read('protocol.json');summary=read('summary.json');result=read('task-001/result.json')
assert protocol['scope']=='XY_TOP_L01_C04_FACE_EXTENSION200_HMAX' and protocol['c04_face_permission']['requested_face']=='front'
assert summary['completed_count']==0 and summary['historical_completed_count']==34
context,built,_=make_context(protocol['mounting_variant'],layout_path=work/protocol['active_layout_file'],
    robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
assert type(built.robot).__name__=='PinocchioHppFclBackend'
prefix='task-001/attempt-001/';calls=read(prefix+'requests.json');trace=read(prefix+'search-trace.json');detail=read(prefix+'attempt.json')
assert detail['contact_complete'] and sum(detail['terminal_contact']['actual_contact_mask'])==48
model=export_model(trace['model']);requests={a['request']['stage_id']:a['request'] for a in trace['attempts']}
extraction=[c for c in calls if c['op']=='plan' and c['stage']=='extraction']
assert len(extraction)==150 and all(c['status']=='NATIVE_PLANNING_FAILED' for c in extraction)
displays=[]
def snapshot(req,q,label,witness=None,response=None):
    i=len(displays)
    displays.append(dict(request=copy.deepcopy(req),response_identity=response or dict(status='SAVED_INSPECTION_STATE'),
        final_timed_path=[],trace=None,external_axis=None))
    return dict(display_index=i,q_rad=q,state_label=label,witness=witness,
        production_flange=built.robot.named_link_frames(np.asarray(q))['flange'].T.reshape(-1).tolist())
first=requests[extraction[0]['stage_id']];target='carton_l01_c04'
assert first['q_start']==detail['contact_end_q_rad'] and first['attachment']['id']==target
assert target not in {b['id'] for b in first['world']}
contact=[a for a in trace['attempts'] if a['request']['stage']=='contact' and a['final_timed_path']]
assert contact[-1]['final_timed_path'][-1]['q']==first['q_start'] and contact[-1]['response_identity']['status']=='SUCCESS'
case=dict(target_id=target,kind='clearance',target_box=next(b for b in summary['initial']['world_cartons'] if b['id']==target),
    snapshots=[snapshot(first,first['q_start'],'已通过contact末态 / 抽离起点')],blockers=[],task='task-001',
    options=[a['option'] for a in result['attempts'] if 'option' in a],failure=detail['failure'],
    actual_contact_cups=48,commanded_cups=48,
    region_samples=[dict(center_world_m=r['center_world_m'],distance_m=r.get('distance_m')) for r in detail['region_attempts']],pair_counts={})
unique={}
for c in extraction:
    for w in c['response']['clearance']['rejected_examples']:
        assert w['pair']==['J2_link','trailer_left_wall'] and w['required_pair_clearance_m']==.005
        pair=' / '.join(w['pair']);case['pair_counts'][pair]=case['pair_counts'].get(pair,0)+1
        unique.setdefault((pair,tuple(w['q_rad'])),(c,w))
for c,w in sorted(unique.values(),key=lambda item:item[1]['surface_distance_m'],reverse=True):
    case['snapshots'].append(snapshot(requests[c['stage_id']],w['q_rad'],'被拒绝的LIN搜索构型 / 非执行位置',w,
        {k:v for k,v in c['response'].items() if k in ('request_id','stage_id','status','reason','solver_message')}))
data=dict(label='c04正吸抽离失败',model=model,inspection_cases=[case],
    raw=dict(source_commit=summary['source_commit'],attempts=displays),result=dict(complete=False,target_id=target),request_log=[],
    qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
template=(work/'tools/search_replay/viewer.html').read_text(encoding='utf-8')
vendor=work/'tools/search_replay/vendor'
template=template.replace('/*__VENDOR__*/','\n'.join((vendor/n).read_text(encoding='utf-8') for n in ('three.min.js','OrbitControls.js')))
template=template.replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
overlay=(work/'tools/search_replay/l01_failures.js').read_text(encoding='utf-8')
overlay=overlay.replace('l01正吸/侧吸 · 五箱失败原因','c04正吸 · 连接件总加长200 mm')
overlay=overlay.replace('l01 五箱失败 · 正吸 / 侧吸原始记录','c04正吸 · 抽离J2 / 左壁见证')
overlay=overlay.replace('250 mm薄型剪叉 · 历史正吸/侧吸轮次<br>不是随后顶吸IK失败那一轮','连接件总加长200 mm · Hmax 250 mm<br>本抓法0/1 · 历史34箱，仍剩6箱')
overlay=overlay.replace('历史正吸/侧吸轮次的保留记录；','本次200 mm正吸的保留记录；')
overlay=overlay.replace('requested>=0?requested:2','requested>=0?requested:0')
template=template.replace('</html>','<script>'+overlay+'</script></html>')
(out/'front-clearance.html').write_text(template,encoding='utf-8')
checks=dict(status='PASS',source_commit=summary['source_commit'],backend=type(built.robot).__name__,
    verified_raw_files=len(hashes),snapshots=len(displays),witness_occurrences=sum(case['pair_counts'].values()),
    unique_witnesses=len(unique),solver_calls_added=0,generated_robot_motion_points=0,
    exporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),overlay_sha256=hashlib.sha256(overlay.encode()).hexdigest())
(out/'front-clearance-checks.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks))
