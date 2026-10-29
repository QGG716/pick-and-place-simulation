"""Append verified XY relocation and native cycles to the exact accepted prefix."""
import argparse,json,tarfile,hashlib
from pathlib import Path
import numpy as np
from plan_m710_joint_search import ROOT
from verify_m710_xy_continuation import verify,candidate
from export_m710_top_layer_replay import write_viewer,mechanism_key
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.moveit2_backend import box_message

def export(run,output):
    run=Path(run);output=Path(output);db=json.loads((run/'checkpoint.json').read_text());proof=verify(run)
    prefix=Path(db['prefix']['path']);html=(prefix/'replay.html').read_text(encoding='utf-8')
    marker='\nconst D=';assert html.count(marker)==1
    start=html.index(marker)+len(marker);data,_=json.JSONDecoder().raw_decode(html[start:])
    assert data['result']['completed_count']==31 and data['segments'][-1]['kind']=='final'
    final=data['segments'].pop();elapsed=final['start'];q=final['q'];height=final.get('height_m',db['accepted_nodes'][0]['root']['height_m'])
    initial_base=np.array(db['accepted_nodes'][0]['layout']['base_xyz_world_m']);worlds=data['worlds'];segments=data['segments']
    data['fk_witnesses']=data['fk_witnesses'][:-1];completed=31;layout=None
    def mechanism(h):
        key=layout.layout_fingerprint+':'+mechanism_key(h)
        if key not in worlds:worlds[key]=[box_message(b) for b in coupled_conveyor_state(layout,h,()).boxes]
        return key
    def add(segment,duration):
        nonlocal elapsed
        assert duration>0
        segments.append(dict(segment,start=elapsed,duration=duration,completed=completed));elapsed+=duration
        sq=segment['points'][0]['q'] if segment['kind']=='motion' else segment['q']
        data['fk_witnesses'].append(layout.robot().named_link_frames(np.asarray(sq))['flange'].T.reshape(-1).tolist())
    for previous,node in zip(db['accepted_nodes'],db['accepted_nodes'][1:]):
        trial=db['trials'][node['incoming']];source=candidate(previous['layout']);destination=candidate(node['layout'])
        assert previous['root']['q_rad']==q and previous['root']['height_m']==height
        cartons=previous['root']['world_cartons'];delta=np.array(previous['layout']['base_xyz_world_m'])-initial_base
        layout=load_workcell_layout(ROOT/'configs/workcells'/('joint-search-'+source.key+'.yaml'))
        if trial['move'] is not None:
            row=json.loads((run/db['relocations'][trial['move']]['folder']/'xy-motion.json').read_text());path=np.array(row['path_xy_a_m'])
            distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))];assert distances[-1]>0
            samples=[dict(t=float(d/.1),delta_m=(delta+np.r_[xy-path[0],0.]).tolist()) for xy,d in zip(path,distances)]
            duration=samples[-1]['t']
            add(dict(kind='base_xy',phase='基座平移（运动学假设）',q=q,world=mechanism(height),cartons=cartons,target=None,option={},
                base_delta_m=delta.tolist(),base_path=samples,commanded_count=0,source='NATIVE_DISCRETE_COLLISION_CHECKED_EXTERNAL_XY',
                display_speed_m_s=.1,display_speed_is_not_hardware_rating=True),duration)
            data['times']['base_xy_demo_s']=data['times'].get('base_xy_demo_s',0.)+duration
        layout=load_workcell_layout(ROOT/'configs/workcells'/('joint-search-'+destination.key+'.yaml'))
        delta=np.array(node['layout']['base_xyz_world_m'])-initial_base
        archive=run/trial['evidence']['archive']
        with tarfile.open(archive) as tar:
            stem=archive.name.removesuffix('.tar.gz')+'/'
            cycle=json.load(tar.extractfile(stem+'plan.json'));trace=json.load(tar.extractfile(stem+'search-trace.json'))
        requests={t['request']['stage_id']:t['request'] for t in trace['attempts']}
        base=dict(target=cycle['target_id'],option=cycle['option'],cartons=cartons,base_delta_m=delta.tolist(),
            commanded_count=sum(cycle['attachment']['selection']['commanded_active_mask']))
        for stage in cycle['stages']:
            request=requests[stage['stage_id']];assert request['q_start']==q
            if stage['generation_source']=='NATIVE_EXTERNAL_AXIS':
                new=stage['external_axis']['to_height_m'];duration=max(.1,abs(height-new)/.15)
                heights=np.linspace(height,new,max(1,int(np.ceil(abs(new-height)/.01)))+1)
                add(dict(base,kind='lift',phase='联动升降（展示速度）',q=q,attachment=request.get('attachment'),
                    heights=[dict(height=float(h),world=mechanism(float(h))) for h in heights],source='SAME_SOURCE_CHECKED_GEOMETRY'),duration)
                height=new;data['times']['lift_demo_s']+=duration
            elif stage['generation_source']=='SEMANTIC_EVENT':
                assert stage['terminal_q']==q
                if stage['planner_id']=='NOMINAL_DROP_RELEASE':
                    drop=cycle['nominal_release'];duration=drop['samples'][-1]['t'];data['times']['nominal_drop_s']+=duration
                    add(dict(base,kind='drop',phase='名义下落',q=q,world=mechanism(height),drop={k:v for k,v in drop.items() if k!='native'},source='NOMINAL_GRAVITY_NOT_PHYSICS'),duration)
                else:
                    assert stage['planner_id']=='WAIT_IDEAL_OUTFEED'
                    path=cycle['wait_outfeed']['path'];duration=path['samples'][-1]['t'];data['times']['ideal_outfeed_s']+=duration
                    add(dict(base,kind='outfeed',phase='机械臂等待 / 理想输送',q=q,world=mechanism(height),outfeed=path,source='CHECKED_FULL_ROUTE_ROBOT_WAITS'),duration)
            else:
                points=stage['points'];assert points[0]['q']==q;duration=points[-1]['t'];data['times']['native_trajectory_s']+=duration
                add(dict(base,kind='motion',phase=stage['stage'],points=points,world=mechanism(height),attachment=request.get('attachment'),
                    landed=next(b for b in request['world'] if b['id']==cycle['target_id']) if stage['stage'] in ('withdrawal','residence') else None,
                    source='CURRENT_RUN_NATIVE_JOINT_TRAJECTORY',stage_id=stage['stage_id']),duration)
                q=points[-1]['q']
        completed+=1;assert node['root']['q_rad']==q and node['root']['height_m']==height
    final_node=db['accepted_nodes'][-1];end=candidate(final_node['layout'])
    layout=load_workcell_layout(ROOT/'configs/workcells'/('joint-search-'+end.key+'.yaml'));cartons=final_node['root']['world_cartons']
    add(dict(kind='final',phase='已接受分支末态',q=q,world=mechanism(height),cartons=cartons,target=None,option={},
        base_delta_m=(np.array(end.evidence()['base_xyz_world_m'])-initial_base).tolist(),source='VERIFIED_MOVING_BASE_BRANCH'),1.)
    data.update(xy_base_continuation=True,initial_base_world_m=initial_base.tolist(),duration=elapsed,verification=proof,
        source_commit=db['source_baseline_commit'],base_actuation='KINEMATIC_ASSUMPTION',physical_execution='NOT_RUN',continuous_swept_proof=False)
    data['result']=dict(completed_count=completed,target_count=40,remaining_target_ids=[b['id'] for b in cartons],
        planning_wall_s=db['prefix']['prior_search_cost_s']+db['previous_session_elapsed_s']+db['current_session_elapsed_s'])
    write_viewer(data,output)
    html=output.read_text(encoding='utf-8');needle='const scene=new T.Scene();'
    text="if(D.xy_base_continuation){document.title='M710 | 允许基座移动的卸箱规划';document.querySelector('h1').textContent='FANUC M-710 | 允许基座移动 · '+D.result.completed_count+'/40';document.querySelector('header small').textContent='31 箱已接受前缀 + 检查后的 XY 平移与后续抓放 · 名义接收 / 理想出料 · 非物理执行';document.querySelector('.muted').textContent='机器人与台座沿 XY 平移；底盘、输送带和剩余箱子固定。平移时关节保持实际末态，按 ≤5 mm 采样检查。平移速度仅用于展示；没有验证移动机构动力学。抓放使用本轮原生轨迹、名义接收及理想出料。physical_execution=NOT_RUN；continuous_swept_proof=false。';}\n"
    text+="document.querySelector('aside h3').textContent='规划完成';\n"
    assert html.count(needle)==1;html=html.replace(needle,text+needle);output.write_text(html,encoding='utf-8')
    proof['replay_sha256']=hashlib.sha256(output.read_bytes()).hexdigest();output.with_suffix('.checks.json').write_text(json.dumps(proof,indent=2))
    return dict(completed=completed,segments=len(segments),moves=len(proof['moves']),output=str(output))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(json.dumps(export(a.run,a.output)))
