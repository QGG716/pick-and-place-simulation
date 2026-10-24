"""Read saved extraction witnesses; no planning or new distance query."""
from pathlib import Path
import copy,hashlib,json,sys,tarfile
import numpy as np
work=Path('/work-current-c04-c3b815d');base=Path('/current-c04-comparison-v2-20261024');run=base/'front'
sys.path[:0]=[str(work/'src'),str(work/'tools')]
from export_m710_search_replay import export_model
from plan_m710_mount_comparison import make_context
read=lambda p:json.loads(p.read_text())
summary=read(run/'summary.json');protocol=read(run/'protocol.json');outcome=summary['outcomes'][0]
assert not outcome['completed_count']
receipt=outcome['evidence'];archive=run/receipt['archive'];prefix=archive.name.removesuffix('.tar.gz')
assert hashlib.sha256(archive.read_bytes()).hexdigest()==receipt['sha256']
with tarfile.open(archive) as t:
 blobs={n:t.extractfile(prefix+'/'+n).read() for n in receipt['file_sha256']}
for n,sha in receipt['file_sha256'].items():assert hashlib.sha256(blobs[n]).hexdigest()==sha
get=lambda n:json.loads(blobs['attempt-001/'+n])
calls=get('requests.json');trace=get('search-trace.json');detail=get('attempt.json')
extraction=[c for c in calls if c['op']=='plan' and c['stage']=='extraction']
assert len(extraction)==156 and all(c['status']=='NATIVE_PLANNING_FAILED' for c in extraction)
assert detail['contact_complete'] and sum(detail['terminal_contact']['actual_contact_mask'])==48
requests={a['request']['stage_id']:a['request'] for a in trace['attempts']}
call=extraction[0];request=requests[call['stage_id']]
assert request['q_start']==detail['contact_end_q_rad'] and request['attachment']['id']=='carton_l01_c04'
witnesses=call['response']['clearance']['rejected_examples']
selected=[next(w for w in witnesses if w['reason']==reason) for reason in ('CLEARANCE_INSUFFICIENT','INTERSECTION_OR_CONTACT')]
_,built,_=make_context(protocol['mounting_variant'],layout_path=work/protocol['active_layout_file'],robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)
assert type(built.robot).__name__=='PinocchioHppFclBackend'
cases=[];attempts=[]
for index,witness in enumerate(selected):
 assert witness['pair']==['J2_link','trailer_left_wall'] and witness['phase']=='search'
 q=witness['q_rad'];response={k:call['response'][k] for k in ('request_id','status','reason','solver_message') if k in call['response']}
 attempts.append(dict(request=copy.deepcopy(request),response_identity=response,final_timed_path=[],trace=None,external_axis=None))
 cases.append(dict(target_id='carton_l01_c04',kind='collision',short_label=['J2 净空首次不足','J2 与左壁相交/接触'][index],
  title='正吸已接触，−X 抽离被 J2／左壁拦截',
  detail='接近及 contact 已通过，48 个命令杯实际几何有效。156 次原生 LIN 抽离请求均失败，共保留 1248 条 J2／左壁搜索见证。首个见证净空 4.660 mm，低于 5 mm；后续构型出现相交/接触。图中是被拒绝的搜索构型，不是实际机器人运动到达的位置。',
  state_label=['被拒绝的 LIN 搜索构型 · 净空 4.660 mm','被拒绝的 LIN 搜索构型 · 相交/接触'][index],
  display_q=q,target_box=next(b for b in summary['initial']['world_cartons'] if b['id']=='carton_l01_c04'),
  target=detail['target'],collision=witness,counts=dict(numerical_ik_success=4,pregrasp=1,contact=1),
  task_wall_s=summary['native_task_wall_s'],failure=detail['failure'],height_m=.25,delta_m=0.,
  actual_contact_cups=48,commanded_cups=48,region_samples=[],native_response=response,
  production_flange_column_major=built.robot.named_link_frames(np.asarray(q))['flange'].T.reshape(-1).tolist()))
data=dict(label='本轮 c04 正吸抽离失败',model=export_model(trace['model']),inspection_cases=cases,
 inspection_title='l01_c04 正吸 · 抽离 LIN 失败',inspection_subtitle='同一 37 箱完成末态的独立对比 · 固定 Hmax 250 mm',
 inspection_summary='顶吸和侧吸已经完整通过。正吸对比未提交箱体移除，也不改变主连续流程 3/3、累计 39 箱的末态。-1 是相交/接触标记，不是穿透深度。',
 raw=dict(source_commit=summary['source_commit'],attempts=attempts),result=dict(complete=False,target_id='carton_l01_c04'),request_log=[],
 qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
template=(work/'tools/search_replay/viewer.html').read_text();vendor=work/'tools/search_replay/vendor'
template=template.replace('/*__VENDOR__*/','\n'.join((vendor/n).read_text() for n in ('three.min.js','OrbitControls.js')))
template=template.replace('/*__DATA__*/',json.dumps(data,ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
overlay=(work/'tools/search_replay/five_failures.js').read_text()
overlay=overlay.replace("${c.target_id.replace('carton_','')} · ${c.kind==='collision'?'工具碰撞':c.kind==='region'?'抽离未完成':'IK 未求解'}",'${c.short_label}')
overlay=overlay.replace("'tool_rigid_0 · 刚性工具'","'J2_link · 上臂'").replace("' · 碰撞箱'","' · 左壁'")
overlay=overlay.replace("i===(c.kind==='collision'?1:failAt)",'i===3')
overlay=overlay.replace("${c.kind==='region'?'规划请求中已附着 · 48 杯有效 · 尚未抽离':'未附着 · contact 未到达'}",'规划请求中已附着 · 48 杯有效 · 抽离未通过')
overlay=overlay.replace('显示被拒绝的静态目标','显示被拒绝的 LIN 搜索构型')
overlay=overlay.replace("camera.position.set(-1.7,p.y-2.8,2.65);controls.target.set(.15,p.y,1.4);",
    "camera.position.set(-3.4,-3.1,2.7);controls.target.set(-.8,.65,1.2);")
template=template.replace('</html>','<script>\n'+overlay+'\n</script></html>')
output=base/'analysis-front/failure.html';output.write_text(template,encoding='utf-8')
checks=dict(status='PASS_SAVED_REQUEST_WITNESSES_ONLY',source_commit=summary['source_commit'],
 verified_files=len(blobs),archive_sha256=receipt['sha256'],extraction_requests=156,search_witnesses=1248,displayed_witnesses=2,
 native_accepted_extraction_paths=0,solver_calls_added=0,generated_robot_motion_points=0,physical_execution='NOT_RUN')
(base/'analysis-front/failure-checks.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps(checks))
