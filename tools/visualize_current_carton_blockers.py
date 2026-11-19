"""Read-only, source-bound current-carton blockers: HTML and scientific figure.

Reads frozen RGB-D, saved native static replies and the existing hardware CAD.
Does not infer, plan, connect devices, issue permits or construct execution IO.
"""
import argparse
import base64
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import numpy as np
from PIL import Image
from visualize_field_planning_failures import plt,full_frames
from export_eco65_search_replay import export_model
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.geometry import corners,transform
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.site_scene import _project
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation,CoordinateModel
from unloading_sim.workbench.detection import metric_points
from unloading_sim.workbench.ui import ROOT,URDF,URDF_SHA,load_mapping
from types import SimpleNamespace


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def dump(path,value):Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def image_url(rgb):
    stream=io.BytesIO();Image.fromarray(rgb).save(stream,format='PNG')
    return 'data:image/png;base64,'+base64.b64encode(stream.getvalue()).decode()


def normal_gap_witness(vertices,box,face,native_gap_m):
    """Return a real vertex/box-face witness only if it matches native distance.

    No nearest points are invented from a scalar distance. Other cases retain
    only the saved native scalar and the highlighted collision pair.
    """
    n=np.asarray(face['normal'],float);origin=np.asarray(face['center_m'],float)
    R=np.asarray(box['pose'],float)[:3,:3]
    if abs(n@R[:,2]-1)>1e-7:return None
    outer=float(np.max((np.asarray(box['corners_m'])-origin)@n))
    values=(vertices-origin)@n;index=int(np.argmin(values));point=vertices[index]
    gap=float(values[index]-outer)
    if gap<0 or abs(gap-native_gap_m)>1e-6:return None
    hit=point-n*gap;local=transform(np.linalg.inv(box['pose']),hit[None])[0]
    if np.any(abs(local[:2])>np.asarray(box['size_m'])[:2]/2+1e-8):return None
    return dict(source='EXACT_SOURCE_MESH_VERTEX_TO_BOX_FACE_MATCHES_NATIVE_WITHIN_1_UM',
        rigid_point_m=point.tolist(),envelope_point_m=hit.tolist(),
        observed_plane_point_m=(point-n*values[index]).tolist(),
        native_gap_m=native_gap_m,independent_gap_m=gap,
        envelope_front_from_observed_plane_m=outer,rigid_from_observed_plane_m=float(values[index]))


def solid_vertices(model,frames,ident):
    points=[]
    for item in model['links']:
        if item['id']!=ident or item['layer']!='collision':continue
        if item['kind']=='mesh':
            vertices=np.frombuffer(base64.b64decode(model['meshes'][item['asset']]['positions_b64']),dtype='<f4').reshape(-1,3)
        elif item['kind']=='box':vertices=corners(np.eye(4),item['size'])
        else:continue
        points.append(transform(frames[item['link']]@np.asarray(item['origin']),vertices))
    if not points:raise ValueError('NO_BOUND_COLLISION_VERTICES:'+ident)
    return np.concatenate(points)


def build(record,diagnostic,config_path):
    source={}
    def read(path):
        source[str(path.relative_to(ROOT))]=sha(path);return json.loads(path.read_text())
    surface=read(diagnostic/'surface-report.json');static=read(diagnostic/'static-report.json')
    if surface['source_task']!=record.name or static['hardware_execution']!='NOT_RUN' or static['permits_issued']!=0:
        raise ValueError('OFFLINE_DIAGNOSTIC_SOURCE_MISMATCH')
    for name,expected in surface['source_hashes'].items():
        if sha(record/name)!=expected:raise ValueError('FROZEN_SOURCE_CHANGED:'+name)
        source[str((record/name).relative_to(ROOT))]=expected
    obs=read(record/'observation.json');geo=read(record/'geometry.json');scene=read(record/'site-scene.json')
    if surface['observation_id']!=obs['observation_id'] or scene['observation_id']!=obs['observation_id']:
        raise ValueError('FROZEN_OBSERVATION_MISMATCH')
    if static['q_start']!=obs['q_start'] or static['q_capture']!=obs['q_capture']:raise ValueError('JOINT_SOURCE_MISMATCH')
    box=next(b for b in geo['boxes'] if b['id']==surface['target_id'])
    source[str(config_path.relative_to(ROOT))]=sha(config_path)
    config=load(config_path);installation=Installation.load(config['execution']['installation'])
    coordinates=CoordinateModel(URDF,URDF_SHA,installation.world_from_base,installation.tool)
    planner=SinglePlanner(config,load_mapping(config['execution']['mapping']),Authority('eco65_single_carton_segment_v1'),coordinates)
    planner.build()  # file/model construction only: no worker or device
    display=deepcopy(planner.source);display['native_model']['init']=deepcopy(planner.init)
    display['native_model']['init']['process_geometry']['expected_contour_ids']=config['topology']['connected_cup_ids']
    display['scene']['mesh_obstacles']=[];model=export_model(display)
    if model['static_visual']:raise ValueError('SIMULATED_ENVIRONMENT_IN_VIEWER')
    counts=dict(collision_solids=sum(x['layer']=='collision' for x in model['links']),cups=len(model['cup_ids']))
    if counts!=dict(collision_solids=715,cups=12):raise ValueError('HARDWARE_MODEL_COUNT_CHANGED:'+str(counts))
    candidates=[];fixtures=[dict(q=obs['q_start'],frames={k:v.tolist() for k,v in coordinates.frames(obs['q_start']).items()})]
    for row in static['rows']:
        data=deepcopy(row);data['surface']=surface['candidates'][row['index']]
        data['nominal_estimate']=next(x for x in static['nominal_holding_comparison']['values'] if x['candidate']==row['index'])
        for stage in data['stages']:
            for index,state in enumerate(stage['checked']):
                path=diagnostic/f"candidate-{row['index']}"/f"{stage['stage']}-branch-{index}-native.json"
                native=read(path);req=native['request'];saved=native['result']
                if req['identity']!=planner.init['identity'] or req['q_start']!=state['q']:
                    raise ValueError('NATIVE_IDENTITY_OR_Q_CHANGED')
                if saved['native_valid']!=state['native']['native_valid'] or saved.get('failure')!=state['native']['failure']:
                    raise ValueError('STATIC_VERDICT_CHANGED')
                if req['single_carton_contact']['target_id']!=box['id'] or req.get('attachment') is not None:
                    raise ValueError('STATIC_TARGET_OR_ATTACHMENT_CHANGED')
                expected=scene['world']+[dict(id=b['id'],pose=b['pose'],size=b['size_m'],category='carton') for b in geo['boxes']]
                if req['world']!=expected:raise ValueError('NATIVE_WORLD_DIFFERS_FROM_FROZEN_SCENE')
                frames=full_frames(model,state['q']);actual=coordinates.frames(state['q'])
                for name,T in actual.items():
                    if name in frames and not np.allclose(frames[name],T,atol=1e-9,rtol=0):raise ValueError('DISPLAY_FK_MISMATCH')
                state['source']=str(path.relative_to(ROOT));state['gap_witness']=None
                failure=state['native']['failure']
                if failure and box['id'] in failure['pair']:
                    ident=next(i for i in failure['pair'] if i!=box['id'])
                    vertices=solid_vertices(model,frames,ident)
                    state['gap_witness']=normal_gap_witness(vertices,box,box['faces'][0],failure['surface_distance_m'])
                    state['focus_m']=vertices.mean(0).tolist()
                fixtures.append(dict(q=state['q'],frames={k:v.tolist() for k,v in actual.items()}))
        candidates.append(data)
    with np.load(record/'observation.npz',allow_pickle=False) as a:rgb=a['rgb'];depth=a['depth']
    with np.load(record/'segmentation.npz',allow_pickle=False) as a:masks=[a[f'mask_{i}'].astype(bool) for i in range(len(a.files))]
    frame=SimpleNamespace(rgb=rgb,depth=depth,observation_id=obs['observation_id'],intrinsics=obs['intrinsics'],depth_scale_m=obs['depth_scale_m'])
    points=metric_points(frame);world=transform(obs['T_world_camera'],points.reshape(-1,3)).reshape(points.shape)
    valid=np.isfinite(world).all(-1)&(points[...,2]>.1)&(points[...,2]<3)
    lo,hi=np.asarray(config['site']['workspace_bounds_m']);valid&=np.all((world>=lo)&(world<=hi),axis=-1)
    upper=masks[box['detection_index']];other=np.logical_or.reduce([m for i,m in enumerate(masks) if i!=box['detection_index']])
    clouds=[]
    for name,selected,color,limit in [('upper',valid&upper&~other,'#29dbc1',6500),('other',valid&other,'#83a7ce',6500)]:
        p=world[selected];stride=max(1,int(np.ceil(len(p)/limit)));clouds.append(dict(name=name,color=color,points=p[::stride].tolist(),source_points=len(p),stride=stride))
    masked=rgb.copy();masked[other]=(masked[other]*.75+np.array([85,145,210])*.25).astype(np.uint8)
    masked[upper]=(masked[upper]*.72+np.array([35,225,185])*.28).astype(np.uint8)
    rest=deepcopy(static['payload_rest_check']);primary=rest['intersections'][0]
    boundary=next(o for o in scene['world'] if o['id']==primary['id'])
    data=dict(schema='eco65_current_carton_blockers_v1',task_id=record.name,observation_id=obs['observation_id'],
        hardware_execution='NOT_RUN',execution_interface=False,full_path_search='NOT_RUN',
        model=model,model_counts=counts,world=scene['world'],box=box,q_start=obs['q_start'],q_capture=obs['q_capture'],
        camera=obs['T_world_camera'],intrinsics=obs['intrinsics'],clouds=clouds,
        images=dict(rgb=image_url(rgb),segmentation=image_url(masked)),candidates=candidates,fk_fixtures=fixtures,
        tool_rings_tcp={s['id']:transform(np.linalg.inv(planner.source['tool']['T_flange_tcp'])@
            np.asarray(planner.source['tool']['T_flange_tool_cad']),s['ring_cad_m']).tolist()
            for s in planner.source['tool']['seals']},
        summary=dict(contact_candidates=surface['contact_candidate_count'],seal_pass=surface['geometric_seal_pass'],
            max_seals=surface['maximum_expected_sealed_cups'],holding_pass=surface['holding_estimate_pass'],
            measured_size_m=surface['measured_size_m'],envelope_size_m=box['size_m'],workspace_bounds_m=config['site']['workspace_bounds_m'],
            box_min_m=np.asarray(box['corners_m']).min(0).tolist(),box_max_m=np.asarray(box['corners_m']).max(0).tolist(),
            primary=primary,boundary=boundary,rest_intersections=len(rest['intersections']),
            missing_boundaries=[x for x in box['uncertainty']['intervals'] if x['method']=='MISSING_BOUNDARY_MAXIMUM_EXTENT'],
            tool_robot_clearance_m=config['tool_robot_clearance']['clearance_m']),sources=source)
    for name,expected in source.items():
        if sha(ROOT/name)!=expected:raise ValueError('SOURCE_CHANGED_DURING_EXPORT:'+name)
    planner.close();return data,rgb


def figure(data,rgb,output):
    b=data['box'];m=b['measured_box'];s=data['summary'];c=data['candidates'][0];face=b['faces'][0]
    fig=plt.figure(figsize=(16,10),dpi=150,facecolor='#f3f6fa');gs=fig.add_gridspec(2,2,height_ratios=[1.18,1],hspace=.30,wspace=.20)
    fig.suptitle('当前卡在模型包络与接触检查；完整路径尚未开始搜索',fontsize=20,y=.965)
    fig.text(.06,.918,f"冻结任务 {data['task_id'][:8]}  ·  同帧 RGB-D  ·  32 个正面密封候选  ·  无实际动作",fontsize=12,color='#485b70')
    ax=fig.add_subplot(gs[0,0]);ax.imshow(rgb)
    W=np.linalg.inv(data['camera']);project=lambda p:_project(transform(W,p),data['intrinsics'])
    center=np.asarray(face['center_m']);u=np.asarray(face['u']);v=np.asarray(face['v']);half=np.asarray(face['size_m'])/2
    rectangle=np.array([center+u*x*half[0]+v*y*half[1] for x,y in [(-1,-1),(1,-1),(1,1),(-1,1),(-1,-1)]])
    uv=project(rectangle);ax.plot(*uv.T,color='#15d2ac',lw=2.4);ax.scatter(*project(center[None])[0],c='#15d2ac',s=40)
    ax.set(xlim=(0,rgb.shape[1]),ylim=(rgb.shape[0],0),title='① 实际冻结图像：上箱正面已找到');ax.axis('off')
    ax=fig.add_subplot(gs[0,1]);envelope=np.asarray(b['corners_m']);fitted=np.asarray(m['corners_m'])
    from scipy.spatial import ConvexHull
    for pts,color,label in [(envelope,'#e69a2e','保守碰撞包络'),(fitted,'#398ddd','可见面/侧面拟合箱体')]:
        xy=pts[:,:2];h=ConvexHull(xy);p=xy[np.r_[h.vertices,h.vertices[0]]];ax.fill(p[:,0],p[:,1],color=color,alpha=.16);ax.plot(p[:,0],p[:,1],color=color,lw=2,label=label)
    bound=s['workspace_bounds_m'][0][0];ax.axvline(bound,color='#df5264',lw=2,ls='--',label=f'配置工作范围 X = {bound:.3f} m')
    ax.axvspan(envelope[:,0].min()-.03,bound,color='#df5264',alpha=.08)
    ax.annotate('',xy=(envelope[:,0].min(),-.45),xytext=(bound,-.45),arrowprops=dict(arrowstyle='<->',color='#c84455',lw=1.5))
    ax.text((bound+envelope[:,0].min())/2,-.477,f"包络越界 {1000*(bound-envelope[:,0].min()):.1f} mm",ha='center',fontsize=11,color='#b6394c')
    ax.set(xlabel='base_link X / m',ylabel='base_link Y / m',ylim=(-.50,.083),title='② 首要阻塞：缺失边界使包络扩大（俯视）');ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(loc='upper left',fontsize=9)
    ax=fig.add_subplot(gs[1,0]);state=next(x for x in c['stages'] if x['stage']=='contact')['checked'][0];w=state['gap_witness']
    if w:
        outer=w['envelope_front_from_observed_plane_m']*1000;rigid=w['rigid_from_observed_plane_m']*1000
        ax.axvspan(-.6,outer,color='#eba337',alpha=.18);ax.axvline(0,color='#17ae98',lw=2,label='实际观测正面')
        ax.axvline(outer,color='#d68c1f',lw=2,label='碰撞包络前界');ax.axvline(rigid,color='#d84458',lw=3,label='刚性杯芯最近顶点')
        ax.annotate('',xy=(outer,.61),xytext=(rigid,.61),arrowprops=dict(arrowstyle='<->',color='#cf3d51',lw=1.5))
        ax.text((outer+rigid)/2,.68,f"模型间隙 {w['native_gap_m']*1000:.3f} mm",ha='center',fontsize=12,color='#a42f40')
        ax.text(.02,.17,'要求 2.4 mm；不是吸具 / 机器人之间的 5 mm 参数。\n这张图展示模型拒绝原因，不证明实物已经碰撞。',transform=ax.transAxes,fontsize=11)
        ax.set(xlim=(-.6,rigid+2.4),ylim=(0,1),yticks=[],xlabel='沿正面法向的距离 / mm（实测平面为 0）')
        ax.legend(loc='upper left',fontsize=9)
    else:
        ax.text(.05,.55,'原生间隙 0.669 mm < 要求 2.4 mm\n最近点未单独恢复；查看三维冲突对。',transform=ax.transAxes);ax.axis('off')
    ax.set_title('③ 独立接触诊断：刚性吸具间隙未通过')
    ax=fig.add_subplot(gs[1,1]);values=c['nominal_estimate'];bars=[values['full_envelope'],values['unverified_nominal_only']]
    ax.barh([1,0],bars,color=['#d85b70','#7a9fbb'],height=.5);ax.axvline(1,color='#315879',ls='--')
    for y,value in zip([1,0],bars):ax.text(value+.035,y,f'{value:.3f}',va='center',fontsize=12)
    ax.set(yticks=[1,0],yticklabels=['完整包络重心假设','名义重心对照（未认证）'],xlim=(0,2.5),xlabel='最小持力估算余量（≥1 才通过）',title='④ 估算也受几何影响；不等于实际吸力测量')
    ax.text(.02,-.27,'接近终态：检查的 9 个样本通过。接触终态：检查的 9 个样本未通过。\n这是三个候选、各三支逆解的静态诊断，不是完整路径或全体逆解结论。',transform=ax.transAxes,fontsize=10,color='#485b70')
    fig.subplots_adjust(top=.88,bottom=.13,left=.065,right=.97)
    fig.savefig(output,facecolor=fig.get_facecolor());plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--record',type=Path,required=True)
    parser.add_argument('--diagnostic',type=Path,required=True);parser.add_argument('--config',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    data,rgb=build(a.record.resolve(),a.diagnostic.resolve(),a.config.resolve());dump(out/'data.json',data)
    template=(ROOT/'tools/search_replay/current_carton_blockers.html').read_text();vendor=ROOT/'tools/search_replay/vendor'
    js=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    (out/'index.html').write_text(template.replace('/*__VENDOR__*/',js).replace('/*__DATA__*/',json.dumps(data,separators=(',',':'),ensure_ascii=False,allow_nan=False).replace('</','<\\/')))
    figure(data,rgb,out/'overview.png')
    report=dict(scope=data['schema'],task_id=data['task_id'],hardware_execution='NOT_RUN',execution_interface=False,
        source_records_unchanged=True,model_counts=data['model_counts'],source_hashes=data['sources'],summary=data['summary'],
        gap_witnesses=sum(bool(c['gap_witness']) for row in data['candidates'] for s in row['stages'] for c in s['checked']),
        output=str(out.relative_to(ROOT)),html_sha256=sha(out/'index.html'),figure_sha256=sha(out/'overview.png'))
    dump(out/'report.json',report);print(json.dumps({k:v for k,v in report.items() if k not in ['source_hashes','summary']},ensure_ascii=False))


if __name__=='__main__':main()
