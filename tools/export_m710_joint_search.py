"""Export only one verified accepted branch, including actual wait/outfeed timing."""
import argparse,json,tarfile
from pathlib import Path
import numpy as np
from verify_m710_joint_search import verify_search
from export_m710_search_replay import export_model
from export_m710_top_layer_replay import write_viewer,mechanism_key
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.moveit2_backend import box_message

ROOT=Path(__file__).resolve().parents[1]

def export(run,output,key=None):
    db=json.loads((run/'checkpoint.json').read_text());key=key or db['best'];record=db['layouts'][key]
    proof=verify_search(run,key);node=record['nodes'][record['best_node']];chain=[]
    while node['incoming'] is not None:
        chain.append(db['trials'][node['incoming']]);node=record['nodes'][node['parent']]
    chain.reverse();layout=load_workcell_layout(ROOT/'configs/workcells'/('joint-search-'+key+'.yaml'))
    robot=layout.robot();model=None;segments=[];worlds={};elapsed=0.;completed=0;native=0.;lifts=0.;drops=0.;outfeed=0.
    root=record['initial'];q=root['q_rad'];height=root['height_m'];cartons=root['world_cartons']
    def mechanism(h):
        k=mechanism_key(h)
        if k not in worlds:worlds[k]=[box_message(b) for b in coupled_conveyor_state(layout,h,()).boxes]
        return k
    def add(data,duration):
        nonlocal elapsed
        segments.append(dict(data,start=elapsed,duration=duration,completed=completed));elapsed+=duration
    def read_trial(trial):
        archive=run/trial['evidence']['archive']
        with tarfile.open(archive) as tar:
            prefix=archive.name.removesuffix('.tar.gz')+'/'
            names=('search-trace.json','plan.json') if trial.get('complete') else ('search-trace.json',)
            return {name:json.load(tar.extractfile(prefix+name)) for name in names}
    for trial in chain:
        files=read_trial(trial);trace=files['search-trace.json'];cycle=files['plan.json']
        if model is None:model=export_model(trace['model'])
        requests={t['request']['stage_id']:t['request'] for t in trace['attempts']}
        assert cycle['batch_root']['q_rad']==q and cycle['batch_root']['height_m']==height
        cartons=cycle['batch_root']['world_cartons'];base=dict(target=cycle['target_id'],option=cycle['option'],cartons=cartons,
            commanded_count=sum(cycle['attachment']['selection']['commanded_active_mask']))
        for stage in cycle['stages']:
            request=requests[stage['stage_id']];assert request['q_start']==q
            if stage['generation_source']=='NATIVE_EXTERNAL_AXIS':
                new=stage['external_axis']['to_height_m'];duration=max(.1,abs(height-new)/.15)
                heights=np.linspace(height,new,max(1,int(np.ceil(abs(new-height)/.01)))+1)
                add(dict(base,kind='lift',phase='联动升降（展示速度）',q=q,attachment=request.get('attachment'),
                    heights=[dict(height=float(h),world=mechanism(float(h))) for h in heights],source='SAME_SOURCE_CHECKED_GEOMETRY'),duration)
                height=new;lifts+=duration
            elif stage['generation_source']=='SEMANTIC_EVENT':
                assert stage['terminal_q']==q
                if stage['planner_id']=='NOMINAL_DROP_RELEASE':
                    drop=cycle['nominal_release'];duration=drop['samples'][-1]['t'];drops+=duration
                    add(dict(base,kind='drop',phase='名义下落',q=q,world=mechanism(height),drop={k:v for k,v in drop.items() if k!='native'},source='NOMINAL_GRAVITY_NOT_PHYSICS'),duration)
                else:
                    assert stage['planner_id']=='WAIT_IDEAL_OUTFEED'
                    path=cycle['wait_outfeed']['path'];duration=path['samples'][-1]['t'];outfeed+=duration
                    add(dict(base,kind='outfeed',phase='机械臂等待 / 理想输送',q=q,world=mechanism(height),outfeed=path,source='CHECKED_FULL_ROUTE_ROBOT_WAITS'),duration)
            else:
                points=stage['points'];assert points[0]['q']==q;duration=points[-1]['t'];native+=duration
                add(dict(base,kind='motion',phase=stage['stage'],points=points,world=mechanism(height),attachment=request.get('attachment'),
                    landed=next(b for b in request['world'] if b['id']==cycle['target_id']) if stage['stage'] in ('withdrawal','residence') else None,
                    source='CURRENT_RUN_NATIVE_JOINT_TRAJECTORY',stage_id=stage['stage_id']),duration)
                q=points[-1]['q']
        completed+=1;cartons=[b for b in cartons if b['id']!=cycle['target_id']]
    if model is None:
        for trial in db['trials']:
            if trial['layout']!=key or 'evidence' not in trial:continue
            try:trace=read_trial(trial)['search-trace.json']
            except (KeyError,OSError):continue
            if trace['model']:model=export_model(trace['model']);break
        if model is None:raise ValueError('NO_NATIVE_MODEL_AVAILABLE_FOR_REPLAY')
    add(dict(kind='final',phase='已接受分支末态' if completed else '初态；暂无接受周期',q=q,world=mechanism(height),cartons=cartons,target=None,
        option={},source='VERIFIED_SINGLE_BRANCH_STATE'),1.)
    data=dict(model=model,segments=segments,worlds=worlds,seed=db['options']['seed'],source_commit=db['source_commit'],
        joint_search=True,all_remaining=True,initial_carton_count=40,historical_completed_count=0,lower_count=0,
        result=dict(completed_count=completed,target_count=40,planning_wall_s=db['elapsed_search_s'],remaining_target_ids=[b['id'] for b in cartons]),
        duration=elapsed,times=dict(native_trajectory_s=native,lift_demo_s=lifts,nominal_drop_s=drops,ideal_outfeed_s=outfeed,event_display_s=1.),
        physical_execution='NOT_RUN',continuous_swept_proof=False,verification=proof)
    data['fk_witnesses']=[robot.named_link_frames(np.asarray(s['points'][0]['q'] if s['kind']=='motion' else s['q']))['flange'].T.reshape(-1).tolist() for s in segments]
    write_viewer(data,output);output.with_suffix('.checks.json').write_text(json.dumps(proof,indent=2))
    return dict(output=str(output),completed=completed,segments=len(segments))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--layout');a=p.parse_args()
    print(json.dumps(export(a.run,a.output,a.layout)))
