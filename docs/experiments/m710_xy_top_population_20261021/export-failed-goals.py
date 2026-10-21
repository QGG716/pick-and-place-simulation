"""Two actual rejected top-suction goals in the existing native 3-D viewer."""
from pathlib import Path
import json,sys,tarfile
import numpy as np
work=Path('/work-xy-top-f37448f');sys.path[:0]=[str(work/'tools'),str(work/'src')]
from export_m710_search_replay import export_model
from plan_m710_mount_comparison import make_context
run=Path('/xy-top-formal-v1')
protocol=json.loads((run/'protocol.json').read_text())
summary=json.loads((run/'summary.json').read_text())
_,connector,_=make_context('xy_minus_100')
attempts=[];logs=[];cases=[];model=None
for outcome in summary['outcomes']:
    if outcome['target_id'] not in ('carton_l03_c01','carton_l03_c04'):continue
    prefix=outcome['evidence']['archive'].removesuffix('.tar.gz')
    with tarfile.open(run/outcome['evidence']['archive']) as archive:
        read=lambda name:json.load(archive.extractfile(prefix+'/'+name))
        calls=read('attempt-001/requests.json');trace=read('attempt-001/search-trace.json')
    requests={a['request']['stage_id']:a['request'] for a in trace['attempts']}
    call=next(c for c in calls if c.get('response',{}).get('clearance',{}).get('rejected_examples'))
    req=requests[call['stage_id']];response=call['response']
    witness=response['clearance']['rejected_examples'][0]
    assert witness['phase']=='endpoint' and np.allclose(witness['q_rad'],req['q_goal'],atol=1e-9,rtol=0)
    assert req['attachment'] is None and response['status']!='SUCCESS'
    if model is None:model=export_model(trace['model'])
    display={k:req[k] for k in ('op','identity','task_id','stage_id','parent_stage_id','stage','candidate_id',
        'scene_fingerprint','world','q_start','q_goal','goal_pose','attachment','pipeline_id','planner_id','purpose') if k in req}
    display['process_policy']={'target_id':outcome['target_id']}
    compact={k:response[k] for k in ('request_id','task_id','stage_id','parent_stage_id','status','reason','failure',
        'solver_message','moveit_error_code') if k in response}
    projected=dict(compact,failure=witness,annotation_source='CURRENT_RESPONSE_CLEARANCE_REJECTED_EXAMPLES_0')
    attempts.append(dict(request=display,response_identity=compact,
        display_label=outcome['target_id']+' · 本次碰撞目标（无合法接近轨迹）',
        final_timed_path=[],trace=None,external_axis=None,current_cycle_selected=False,
        history_status='RECORDED_IN_RAW_ARCHIVE_NOT_EMBEDDED',failure_witness=projected))
    logs.append(dict(op='plan',response=projected,round_trip_s=call['round_trip_s']))
    frames=connector.robot.named_link_frames(np.asarray(req['q_goal']))
    cases.append(dict(target_id=outcome['target_id'],witness=witness,q_rad=req['q_goal'],
        flange_column_major=frames['flange'].T.reshape(-1).tolist(),
        source_archive=outcome['evidence']['archive'],source_archive_sha256=outcome['evidence']['sha256'],
        stage_id=req['stage_id'],native_response=compact))
assert len(cases)==2
data=dict(label='本轮低层顶吸碰撞 · 静态原生失败目标',
    raw=dict(run_id='xy-top-formal-v1-current-failed-goals',source_commit=protocol['source_commit'],attempts=attempts),
    result=dict(complete=False,diagnostic_only=False,target_id='l03_c01 / l03_c04'),
    model=model,request_log=logs,inspection_cases=cases,
    display_projection='Exact native q/world; highlighted pair is selected from this response. No motion synthesized.',
    qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
vendor=work/'tools/search_replay/vendor'
page=(work/'tools/search_replay/viewer.html').read_text()
page=page.replace('/*__VENDOR__*/',(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text())
page=page.replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),ensure_ascii=False).replace('</','<\\/'))
page=page.replace('</html>',"""<script>
const inspectGoal=document.getElementById('goalInspect'),originalInspect=inspectGoal.onclick;
inspectGoal.onclick=()=>{originalInspect();const b=active.request.world.find(b=>b.id===active.request.process_policy.target_id);
camera.position.set(-2.4,b.pose[1][3]-2.5,2.8);controls.target.set(.2,b.pose[1][3],1.25);controls.update();};
inspectGoal.click();</script></html>""")
Path('/xy-top-failed-goals.html').write_text(page,encoding='utf-8')
Path('/xy-top-failed-goals-checks.json').write_text(json.dumps(dict(status='PASS_CURRENT_REJECTED_Q_AND_WORLD',
    source_commit=protocol['source_commit'],cases=cases),indent=2)+'\n')
print(json.dumps(dict(cases=len(cases),physical_execution='NOT_RUN')))
