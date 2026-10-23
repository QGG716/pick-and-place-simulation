"""Export accepted continuous native cycles, never stitch independent trials."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from export_m710_search_replay import export_model
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.moveit2_backend import box_message
from unloading_sim.approach_quality import path_metrics


def mechanism_key(height):
    # Distinct native IEEE-754 endpoints can round to the same decimal string.
    # Cache exact geometry by exact height; keep the strict world equality gate.
    return float(height).hex()


def export_incomplete(run,seed,output,protocol,result,attempt_name):
    """Show one isolated checked prefix. Never animate a failed goal as a path."""
    rows=[a for a in result['attempts'] if a.get('folder')]
    attempt=next(a for a in rows if a['folder']==attempt_name) if attempt_name else rows[0]
    folder=run/f'seed-{seed}'/attempt['folder']
    trace=json.loads((folder/'search-trace.json').read_text())
    calls=json.loads((folder/'requests.json').read_text())
    model=export_model(trace['model'])
    requests={a['request']['stage_id']:a['request'] for a in trace['attempts']}
    layout=load_workcell_layout(ROOT/protocol.get('active_layout_file','configs/workcells/m710id70_unloading_layout_v1.yaml'))
    root=attempt['root'];q=root['q_rad'];height=root['height_m'];worlds={};segments=[];time=0.
    def mechanism(h):
        key=mechanism_key(h)
        worlds[key]=[box_message(b) for b in coupled_conveyor_state(layout,h,()).boxes]
        return key
    base=dict(target=attempt['target_id'],option=attempt['option'],cartons=root['world_cartons'],
        commanded_count=0,completed=0)
    for call in calls:
        response=call.get('response',{})
        if call.get('op')!='plan' or response.get('generation_source')!='NATIVE_EXTERNAL_AXIS' or call['status']!='SUCCESS':continue
        request=requests[call['stage_id']];axis=response['external_axis']
        assert request['q_start']==q and response['terminal_q']==q
        assert axis['from_height_m']==height and all(c['passed'] for c in axis['checks'])
        height=axis['to_height_m']
        frames=[dict(height=float(h),world=mechanism(float(h))) for h in np.linspace(
            axis['from_height_m'],height,max(1,int(np.ceil(abs(height-axis['from_height_m'])/.01)))+1)]
        cartons={b['id'] for b in protocol['original_cartons']}
        assert sorted((b for b in request['receiver_end_world'] if b['id'] not in cartons),key=lambda b:b['id'])==sorted(worlds[mechanism(height)],key=lambda b:b['id'])
        segments.append(dict(base,kind='lift',phase='候选初始降带 · 原生检查通过',q=q,heights=frames,
            source='ISOLATED_CHECKED_PREFIX_NOT_COMMITTED',start=time,duration=4.))
        time+=4.
    assert segments, 'NO_CHECKED_PREFIX_TO_ANIMATE'
    segments.append(dict(base,kind='final',phase='接近未完成 · 候选未接受',q=q,world=mechanism(height),
        source='NO_CONTACT_EXTRACTION_RELEASE_OR_DRAIN',start=time,duration=2.))
    time+=2.
    data=dict(model=model,segments=segments,worlds=worlds,seed=seed,source_commit=protocol['source_commit'],
        result={k:result[k] for k in ('completed_count','target_count','lower_layer_preserved','planning_wall_s','remaining_target_ids')},
        lower_count=len(protocol['original_cartons'])-len(protocol['frozen']['target_ids']),
        frozen_height=protocol['frozen']['heights'][attempt['target_id']],
        initial_carton_count=len(root['world_cartons']),historical_completed_count=result['historical_completed_count'],
        single_junction=True,incomplete_run=True,candidate_folder=attempt['folder'],
        geometry_variant=protocol.get('authorized_geometry_change'),
        failure_detail=attempt.get('failure'),unsearched_options=sum(a.get('unsearched_option_count',0) for a in result['attempts']),
        junction_evidence=dict(option=attempt['option'],receive_sync_location='NOT_REACHED',actual_extraction_height_m=None),
        duration=time,times=dict(native_trajectory_s=0.,lift_demo_s=time-2.,nominal_drop_s=0.,
            nominal_drop_distance_m=None,event_display_s=2.,hardware_lift_s=None),
        physical_execution='NOT_RUN',execution_ready=False,qualification_status='NOT_EVALUATED',continuous_swept_proof=False)
    data['fk_witnesses']=[layout.robot().named_link_frames(np.asarray(q))['flange'].T.reshape(-1).tolist() for s in segments]
    if protocol.get('right_front_eccentric'):
        from unloading_sim.right_front_eccentric import candidate_deltas
        from unloading_sim.junction_reception import temporary_heights
        domain=attempt['eccentric_domain_intervals_m']
        delta_count=sum(1 for _ in candidate_deltas(domain))
        height_count=sum(1 for _ in temporary_heights(attempt['option']['height_m'],
            protocol['temporary_height_minimum_m']))
        data['unsearched_options']=delta_count*height_count-len(rows)
        data['unsearched_label']='未访问候选（δ/高度；两同步路线共享接近）'
        data['integration_failure']=result['status']=='REQUEST_STOPPED'
    write_viewer(data,output)
    print(json.dumps(dict(output=str(output),candidate=attempt['folder'],completed=0,duration=time)))


def write_viewer(data,output):
    vendor=ROOT/'tools/search_replay/vendor'
    html=(ROOT/'tools/lifted_reception/batch_viewer.html').read_text(encoding='utf-8')
    html=html.replace('/*__VENDOR__*/',(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text())
    html=html.replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(html,encoding='utf-8')


def export(run,seed,output,attempt=None):
    protocol=json.loads((run/'protocol.json').read_text());folder=run/f'seed-{seed}'
    result=json.loads((folder/'result.json').read_text());model=None;requests={};calls={}
    if not result['cycles']:
        return export_incomplete(run,seed,output,protocol,result,attempt)
    for attempt in result['attempts']:
        if not attempt.get('complete'):continue
        trace=json.loads((folder/attempt['folder']/'search-trace.json').read_text())
        if model is None:model=export_model(trace['model'])
        for t in trace['attempts']:
            req=t['request'];requests[req['stage_id']]=req
        for call in json.loads((folder/attempt['folder']/'requests.json').read_text()):
            if call.get('stage_id'):calls[call['stage_id']]=call
    if model is None:raise ValueError('NO_ACCEPTED_NATIVE_CYCLE_TO_REPLAY')
    layout=load_workcell_layout(ROOT/protocol.get('active_layout_file','configs/workcells/m710id70_unloading_layout_v1.yaml'))
    robot=layout.robot()
    root=result['initial'];q=root['q_rad'];height=root['height_m'];segments=[];time=0.;completed=0
    native_time=0.;mechanism_time=0.;drop_time=0.;event_time=0.
    worlds={}
    def mechanism(h):
        key=mechanism_key(h)
        if key not in worlds:worlds[key]=[box_message(b) for b in coupled_conveyor_state(layout,h,()).boxes]
        return key
    def add(segment,duration):
        nonlocal time
        segment.update(start=time,duration=duration,completed=completed);segments.append(segment);time+=duration
    for cycle in result['cycles']:
        assert cycle['batch_root']['q_rad']==q and cycle['batch_root']['height_m']==height
        cartons=cycle['batch_root']['world_cartons'];base=dict(target=cycle['target_id'],option=cycle['option'],cartons=cartons,
            commanded_count=sum(cycle['attachment']['selection']['commanded_active_mask']))
        first=True;extraction_leg=0
        for stage in cycle['stages']:
            req=requests[stage['stage_id']]
            assert req['q_start']==q
            carton_ids={b['id'] for b in protocol['original_cartons']}
            # Released target retains its identity but changes category.
            actual_fixed=sorted((b for b in req['world'] if b['id'] not in carton_ids),key=lambda b:b['id'])
            expected_fixed=sorted(worlds[mechanism(height)],key=lambda b:b['id'])
            assert actual_fixed==expected_fixed, 'NATIVE_REPLAY_WORLD_GEOMETRY_MISMATCH'
            if first:assert not req.get('parent_stage_id');first=False
            if stage['generation_source']=='NATIVE_EXTERNAL_AXIS':
                h0=height;h1=stage['external_axis']['to_height_m'];duration=0.6 if h0==h1 else 4.
                heights=np.linspace(h0,h1,max(1,int(np.ceil(abs(h1-h0)/.01)))+1) if h0!=h1 else [h0]
                add(dict(base,kind='lift',phase='保持高度' if h0==h1 else '联动升降',q=q,
                    attachment=req.get('attachment'),
                    heights=[dict(height=float(h),world=mechanism(float(h))) for h in heights],
                    source='SAME_SOURCE_COUPLED_GEOMETRY; DEMO_DURATION_ONLY'),duration)
                height=h1;mechanism_time+=duration
            elif stage['generation_source']=='SEMANTIC_EVENT':
                assert stage['planner_id']=='NOMINAL_DROP_RELEASE' and stage['terminal_q']==q
                drop=cycle['nominal_release'];duration=drop['samples'][-1]['t'];drop_time+=duration
                add(dict(base,kind='drop',phase='释放 / 名义下落',q=q,world=mechanism(height),
                    drop={k:v for k,v in drop.items() if k!='native'},source='IDEAL_GRAVITY_NOT_PHYSICS'),duration)
                add(dict(base,kind='landed',phase='名义首次触带',q=q,world=mechanism(height),landed=cycle['landed_box'],
                    source='NOMINAL_FIRST_CONTACT'),.6);event_time+=.6
            else:
                points=stage['points'];assert points[0]['q']==q
                duration=points[-1]['t'];assert duration>0
                phase=stage['stage']
                if phase=='extraction' and cycle['option']['template'] in ('TOP_FIXED_MINIMUM_RECEIVER','TOP_FIXED_CURRENT_RECEIVER','SELECTED_TOP_HMAX'):
                    extraction_leg+=1
                    phase='top_lift' if extraction_leg==1 else 'top_transfer'
                add(dict(base,kind='motion',phase=phase,points=points,world=mechanism(height),
                    attachment=req.get('attachment'),landed=cycle['landed_box'] if stage['stage']=='withdrawal' else None,
                    source='CURRENT_RUN_NATIVE_JOINT_TRAJECTORY',stage_id=stage['stage_id'],
                    planner_id=stage['planner_id'],q_start=points[0]['q'],q_goal=points[-1]['q'],
                    quality=path_metrics(robot,points,np.asarray(model['flange_from_contact']),
                        calls[stage['stage_id']]['round_trip_s']),
                    tcp_path=[robot.fk(np.asarray(p['q']))[:3,3].tolist() for p in points]
                        if stage['stage']=='pregrasp' else None),duration)
                q=points[-1]['q'];native_time+=duration
        assert q==cycle['end_q_rad']
        add(dict(base,kind='landed',phase='安全分离 / IDEAL_RECEIVER_DRAIN',q=q,world=mechanism(height),
            landed=cycle['landed_box'],source='CHECKED_SEPARATION_THEN_ASSUMED_DRAIN'),.8);event_time+=.8
        completed+=1
    add(dict(kind='final',phase='整批结束',q=q,world=mechanism(height),cartons=result['terminal']['world_cartons'],
        target=None,option={},source='COMMITTED_BATCH_STATE'),1.);event_time+=1.
    data=dict(model=model,segments=segments,worlds=worlds,seed=seed,source_commit=protocol['source_commit'],
        result={k:result[k] for k in ('completed_count','target_count','lower_layer_preserved','planning_wall_s','remaining_target_ids')},
        lower_count=len(protocol['original_cartons'])-len(protocol['frozen']['target_ids']),
        initial_carton_count=len(root['world_cartons']),historical_completed_count=result.get('historical_completed_count',0),
        single_junction=protocol.get('scope')=='ONE_NEW_JUNCTION_C00',
        all_remaining=protocol.get('scope') in ('ALL_REMAINING_FROM_ACCEPTED_C00','XY_TOP_NEW_WORLD_40','XY_TOP_MIN250_CONTINUATION','XY_TOP_SELECTED_LAYER_CONTINUATION','XY_TOP_LOWPROFILE250_CONTINUATION','XY_TOP_LOWPROFILE250_BOTTOM_TWO_TOP','XY_TOP_LOWPROFILE250_L01_TOOL_CHASSIS','XY_TOP_L01_SELECTED_SIDE_TOP_HMAX','XY_TOP_L01_C01_FINITE_SIDE_CHANNEL_HMAX','XY_TOP_L01_C01_FRONT_FINITE_CHANNEL_HMAX',
            'XY_TOP_L05_J4_VELOCITY_RESTART','XY_TOP_L05_ALL_JOINT_RATES_RESTART'),
        planning_diagnostic=protocol.get('planning_diagnostic'),
        mount_comparison=protocol.get('scope')=='PAIRED_MOUNT_INDEPENDENT_SINGLE_CARTON',
        mounting_variant=protocol.get('variant'),
        geometry_variant=protocol.get('authorized_geometry_change'),
        junction_evidence=({k:result['cycles'][0].get(k) for k in ('option','receive_sync_location',
            'actual_extraction_height_m','receive_policy_check','first_touch_tool_geometry')} if protocol.get('scope')=='ONE_NEW_JUNCTION_C00' else None),
        duration=time,times=dict(native_trajectory_s=native_time,lift_demo_s=mechanism_time,nominal_drop_s=drop_time,
            nominal_drop_distance_m=result['cycles'][0]['nominal_release']['drop_m'] if protocol.get('scope')=='ONE_NEW_JUNCTION_C00' else None,
            event_display_s=event_time,hardware_lift_s=None),physical_execution='NOT_RUN',execution_ready=False,
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False)
    data['fk_witnesses']=[robot.named_link_frames(np.asarray(s['points'][0]['q'] if s['kind']=='motion' else s['q']))['flange'].T.reshape(-1).tolist() for s in segments]
    write_viewer(data,output)
    (output.parent/(output.stem+'-checks.json')).write_text(json.dumps(dict(status='PASS_NATIVE_Q_AND_BATCH_ROOT_CONTINUITY',
        seed=seed,cycles=completed,segments=len(segments),**data['times']),indent=2))
    print(json.dumps(dict(output=str(output),completed=completed,duration=time)),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--seed',type=int,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--attempt',help='One isolated failed candidate; never join discarded states')
    a=p.parse_args();export(a.run,a.seed,a.output,a.attempt)
