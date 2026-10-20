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


def export(run,seed,output):
    protocol=json.loads((run/'protocol.json').read_text());folder=run/f'seed-{seed}'
    result=json.loads((folder/'result.json').read_text());model=None;requests={};calls={}
    for attempt in result['attempts']:
        if not attempt.get('complete'):continue
        trace=json.loads((folder/attempt['folder']/'search-trace.json').read_text())
        if model is None:model=export_model(trace['model'])
        for t in trace['attempts']:
            req=t['request'];requests[req['stage_id']]=req
        for call in json.loads((folder/attempt['folder']/'requests.json').read_text()):
            if call.get('stage_id'):calls[call['stage_id']]=call
    if model is None:raise ValueError('NO_ACCEPTED_NATIVE_CYCLE_TO_REPLAY')
    layout=load_workcell_layout(ROOT/'configs/workcells/m710id70_unloading_layout_v1.yaml')
    robot=layout.robot()
    root=result['initial'];q=root['q_rad'];height=root['height_m'];segments=[];time=0.;completed=0
    native_time=0.;mechanism_time=0.;drop_time=0.;event_time=0.
    worlds={}
    def mechanism(h):
        key=format(h,'.12f')
        if key not in worlds:worlds[key]=[box_message(b) for b in coupled_conveyor_state(layout,h,()).boxes]
        return key
    def add(segment,duration):
        nonlocal time
        segment.update(start=time,duration=duration,completed=completed);segments.append(segment);time+=duration
    for cycle in result['cycles']:
        assert cycle['batch_root']['q_rad']==q and cycle['batch_root']['height_m']==height
        cartons=cycle['batch_root']['world_cartons'];base=dict(target=cycle['target_id'],option=cycle['option'],cartons=cartons,
            commanded_count=sum(cycle['attachment']['selection']['commanded_active_mask']))
        first=True
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
                add(dict(base,kind='motion',phase=stage['stage'],points=points,world=mechanism(height),
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
        junction_evidence=({k:result['cycles'][0].get(k) for k in ('option','receive_sync_location',
            'actual_extraction_height_m','receive_policy_check','first_touch_tool_geometry')} if protocol.get('scope')=='ONE_NEW_JUNCTION_C00' else None),
        duration=time,times=dict(native_trajectory_s=native_time,lift_demo_s=mechanism_time,nominal_drop_s=drop_time,
            nominal_drop_distance_m=result['cycles'][0]['nominal_release']['drop_m'] if protocol.get('scope')=='ONE_NEW_JUNCTION_C00' else None,
            event_display_s=event_time,hardware_lift_s=None),physical_execution='NOT_RUN',execution_ready=False,
        qualification_status='NOT_EVALUATED',continuous_swept_proof=False)
    data['fk_witnesses']=[robot.named_link_frames(np.asarray(s['points'][0]['q'] if s['kind']=='motion' else s['q']))['flange'].T.reshape(-1).tolist() for s in segments]
    vendor=ROOT/'tools/search_replay/vendor'
    html=(ROOT/'tools/lifted_reception/batch_viewer.html').read_text(encoding='utf-8')
    html=html.replace('/*__VENDOR__*/',(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text())
    html=html.replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),allow_nan=False).replace('</','<\\/'))
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(html,encoding='utf-8')
    (output.parent/(output.stem+'-checks.json')).write_text(json.dumps(dict(status='PASS_NATIVE_Q_AND_BATCH_ROOT_CONTINUITY',
        seed=seed,cycles=completed,segments=len(segments),**data['times']),indent=2))
    print(json.dumps(dict(output=str(output),completed=completed,duration=time)),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--seed',type=int,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();export(a.run,a.seed,a.output)
