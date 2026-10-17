import json,copy,uuid
from pathlib import Path
from unloading_sim.moveit2_backend import ResidentMoveItClient
d=Path('/tmp/search-trace-20261017');t=json.loads((d/'baseline-c04-v2/search-trace.json').read_text())
records=[json.loads(s) for s in (d/'baseline-c04-v2/requests.jsonl').read_text().splitlines()]
ik=next(r for r in records if r['op']=='ik' and r['stage']=='pregrasp')
req=copy.deepcopy(next(a['request'] for a in t['attempts'] if a['request']['stage']=='pregrasp'))
req.pop('q_goal',None);req.pop('stage_id',None)
req.update(op='ik',task_id='interface-probe-'+uuid.uuid4().hex,parent_stage_id='',ownership_transition=None,
    goal_pose=ik['goal_pose'],ik_mode='bounded_target',solver_budget_s=.1,allowed_planning_time_s=10.,
    purpose='FREE_APPROACH',position_tolerance_m=.0001,orientation_tolerance_rad=.0002,
    excluded_ik=[],ik_dedup_tolerance_rad=.001)
init=copy.deepcopy(t['model']);init.update(search_trace_enabled=False,single_target_recovery_enabled=True)
c=ResidentMoveItClient(str(d/'worker.sh'),log_path=str(d/'recovery-ik-probe-worker.log'))
try:
    init_result=c.request(init);assert init_result['status']=='READY',init_result
    a=c.request(req);rows=[dict(request=copy.deepcopy(req),response=a)]
    if a.get('q'):
        req['excluded_ik']=[a['q']];b=c.request(req);rows.append(dict(request=copy.deepcopy(req),response=b))
    result=dict(scope='NATIVE_INTERFACE_PROBE_NO_MOTION_NOT_FORMAL_INPUT',records=rows,
        internal_attempts='NOT_OBSERVABLE',seed_source='ARCHIVED_ACTUAL_START_NOT_SUCCESS_Q',
        physical_execution='NOT_RUN',qualification_status='NOT_EVALUATED')
    (d/'recovery-ik-probe.json').write_text(json.dumps(result,indent=2))
    print(json.dumps([{k:r['response'].get(k) for k in ('status','solver_api','actual_ik_s','excluded_local_ik_count','q')} for r in rows]))
finally:c.close()
