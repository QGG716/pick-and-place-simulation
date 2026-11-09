"""Compact display of real failed native requests; raw evidence stays on server."""
import argparse,hashlib,json,tarfile
from collections import Counter
from pathlib import Path
import numpy as np
from export_m710_search_replay import export_model
from unloading_sim.workcell_layout import load_workcell_layout

ROOT=Path(__file__).resolve().parents[1]


def export(run,trial_number,output):
    db=json.loads((run/'checkpoint.json').read_text());trial=db['trials'][trial_number]
    assert not trial['complete'] and trial['trial_number']==trial_number
    record=db['layouts'][trial['layout']];archive=run/trial['evidence']['archive']
    assert hashlib.sha256(archive.read_bytes()).hexdigest()==trial['evidence']['sha256']
    files={}
    with tarfile.open(archive,mode='r|gz') as tar:
        for member in tar:
            name=member.name.split('/')[-1]
            if name not in ('search-trace.json','requests.json','dispatch.json'):continue
            raw=tar.extractfile(member).read()
            assert hashlib.sha256(raw).hexdigest()==trial['evidence']['file_sha256'][name]
            files[name]=json.loads(raw)
    trace=files['search-trace.json'];calls=files['requests.json'];dispatch=files['dispatch.json']
    responses={c['stage_id']:c for c in calls if c.get('stage_id') and c.get('op')=='plan'}
    attempts=[];witnesses=[];pairs=Counter();logs=[];statuses=Counter();lift_failures=[]
    for item in trace['attempts']:
        req=item['request']
        if req.get('op')!='plan' or req.get('pipeline_id')=='semantic_event':continue
        call=responses[req['stage_id']];response=call['response'];status=response['status']
        for key in ('task_id','stage_id','parent_stage_id'):
            assert call[key]==req[key]
            if key in response:assert req[key]==response[key]
        compact={k:response[k] for k in ('request_id','task_id','stage_id','parent_stage_id','status','reason','solver_message','moveit_error_code','failure') if k in response}
        display={k:req[k] for k in ('op','identity','task_id','stage_id','parent_stage_id','stage','candidate_id','scene_fingerprint',
            'world','q_start','q_goal','goal_pose','attachment','pipeline_id','planner_id','purpose') if k in req}
        display['process_policy']={'target_id':req['process_policy']['target_id']}
        index=len(attempts);seen=set()
        if req.get('pipeline_id')=='external_axis':
            axis=response.get('external_axis',{})
            binding=trace['model']['lifted_receiver_binding']
            possibilities=[binding,*([binding['pre_approach_transition']] if 'pre_approach_transition' in binding else [])]
            match=[b for b in possibilities if b['samples'][0]['world']==req['world'] and b['samples'][-1]['world']==req['receiver_end_world']]
            assert match and req['attachment'] is None
            for checked in axis.get('checks',[]):
                if checked['passed']:continue
                failure=checked['failure'];height=checked['height_m']
                sample=next(s for s in match[0]['samples'] if s['height_m']==height)
                assert axis['robot_q']==req['q_start'] and failure==response['failure']
                lift_failures.append(dict(request_id=response['request_id'],height_m=height,pair=failure.get('pair'),
                    from_height_m=axis['from_height_m'],to_height_m=axis['to_height_m']))
                pairs[tuple(failure.get('pair',[]))]+=1
                witnesses.append(dict(attempt_index=index,request_id=response['request_id'],stage_id=req['stage_id'],
                    q_rad=req['q_start'],phase='receiver_lift',height_m=height,world_at_witness=sample['world'],
                    witness_source='ARCHIVED_EXTERNAL_AXIS_FAILED_CHECK_AND_EXACT_BOUND_WORLD',**failure))
        for w in response.get('clearance',{}).get('rejected_examples',[]):
            pairs[tuple(w.get('pair',[]))]+=1
            key=(tuple(w.get('pair',[])),w.get('reason'),w.get('phase'))
            if key in seen or not isinstance(w.get('q_rad'),list):continue
            seen.add(key)
            assert len(w['q_rad'])==6 and np.isfinite(w['q_rad']).all()
            witnesses.append(dict(attempt_index=index,request_id=response['request_id'],stage_id=req['stage_id'],**w))
        points=item.get('final_timed_path') or []
        if points:assert np.allclose(points[0]['q'],req['q_start'],atol=1e-12,rtol=0)
        attempts.append(dict(request=display,response_identity=compact,final_timed_path=points,trace=None,
            history_status='RECORDED_IN_RAW_ARCHIVE_NOT_EMBEDDED',external_axis=None,
            display_label='失败任务中的原生请求：'+status,current_cycle_selected=False,
            failure_witness=dict(native=compact) if status!='SUCCESS' else None))
        logs.append(dict(op='plan',stage=req['stage'],status=status,response=compact,round_trip_s=call.get('round_trip_s')))
        statuses[(req['stage'],status,response.get('solver_message'))]+=1
    assert attempts  # IK-only failures need a start/goal view, not invented collision witnesses.
    layout=load_workcell_layout(ROOT/'configs/workcells'/record['layout_path']);robot=layout.robot()
    for w in witnesses:
        w['expected_flange_flat']=robot.named_link_frames(np.asarray(w['q_rad']))['flange'].T.reshape(-1).tolist()
    endpoints=[]
    from unloading_sim.front_arm_posture import front_arm_posture
    for index,a in enumerate(attempts):
        a['start_expected_flange_flat']=robot.named_link_frames(np.asarray(a['request']['q_start']))['flange'].T.reshape(-1).tolist()
        if not a['final_timed_path']:continue
        q=np.asarray(a['final_timed_path'][-1]['q'])
        endpoints.append(dict(attempt_index=index,stage=a['request']['stage'],q_rad=q.tolist(),
            expected_flange_flat=robot.named_link_frames(q)['flange'].T.reshape(-1).tolist(),
            arm_posture=front_arm_posture(robot,q)))
    initial_q=record['initial']['q_rad']
    metadata=dict(run=run.name,trial_number=trial_number,gap_m=record['gap_m'],target_id=trial['target_id'],
        completed_count=len(dispatch['initial']['completed']),carton_count=len(dispatch['initial']['world_cartons']),wall_s=trial['wall_s'],category=trial['category'],
        contact_complete=trial.get('contact_complete',False),initial_q_rad=initial_q,
        initial_expected_flange_flat=robot.named_link_frames(np.asarray(initial_q))['flange'].T.reshape(-1).tolist(),
        initial_orientation=record.get('initial_orientation_evidence'),target=dispatch['target'],
        initial_arm_posture=record.get('initial_arm_posture_evidence'),
        base_world_m=record['candidate']['base_xyz_world_m'],
        failure={k:v for k,v in (trial.get('failure') or {}).items() if isinstance(v,(str,int,float,bool))},
        archive_server_path=str(archive),archive_sha256=trial['evidence']['sha256'],source_identity=db['identity'],
        witness_pair_counts=[dict(pair=list(k),count=v) for k,v in pairs.most_common()],
        native_requests=[dict(stage=k[0],status=k[1],solver=k[2],count=v) for k,v in statuses.items()],
        receiver_lift_failures=lift_failures,
        witnesses=witnesses,native_endpoints=endpoints,physical_execution='NOT_RUN',global_unreachable=False)
    data=dict(label=f"g={record['gap_m']*1000:.0f} mm · 失败诊断",raw=dict(run_id=run.name,source_commit=db['source_commit'],attempts=attempts),
        result=dict(complete=False,target_id=trial['target_id']),model=export_model(trace['model']),request_log=logs,
        failure_review=metadata,qualification_status='NOT_EVALUATED',physical_execution='NOT_RUN')
    vendor=ROOT/'tools/search_replay/vendor';html=(ROOT/'tools/search_replay/viewer.html').read_text(encoding='utf-8')
    html=html.replace('/*__VENDOR__*/',(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text())
    html=html.replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    html=html.replace('</script></html>',(ROOT/'tools/search_replay/gap_failure_overlay.js').read_text(encoding='utf-8')+'\n</script></html>')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(html,encoding='utf-8')
    summary={k:v for k,v in metadata.items() if k!='witnesses'}
    summary.update(witness_count=len(witnesses),native_request_count=len(attempts),html_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),html_bytes=output.stat().st_size)
    output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ('run','trial_number','target_id','contact_complete','witness_count','native_request_count','html_bytes')}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--trial',type=int,default=0);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();export(a.run,a.trial,a.output)
