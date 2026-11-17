"""Export saved failed field plans as PNG and a self-contained offline 3D viewer.

Reads archived requests/replies only. Never connects devices or reruns planning.
Native output points are displayed as diagnostic samples, never audited paths.
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
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from export_eco65_search_replay import export_model
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.scene import load_snapshot
from unloading_sim.workbench.single_carton.geometry import corners, transform
from unloading_sim.workbench.single_carton.site_scene import _project
from unloading_sim.workbench.ui import ROOT, URDF, URDF_SHA, load_mapping
from unloading_sim.robot import _axis_angle_transform

_font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if _font.exists():font_manager.fontManager.addfont(str(_font))
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=str(_font)).get_name() if _font.exists() else 'sans-serif','axes.unicode_minus':False})
COLORS={'ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED':'#e65b66','INTERPOLATION_LIMIT':'#e0a332','NATIVE_SOLVER_BUDGET_EXHAUSTED':'#8f7bd5'}


def kind(reason):
    return 'NATIVE_SOLVER_BUDGET_EXHAUSTED' if 'NATIVE_SOLVER_BUDGET_EXHAUSTED' in reason else reason.split(':')[0]


def full_frames(model,q):
    frames={'world':np.eye(4)};pending=list(model['joints'])
    while pending:
        next_pending=[]
        for joint in pending:
            if joint['parent'] not in frames:next_pending.append(joint);continue
            pose=frames[joint['parent']]@np.asarray(joint['origin'])
            if joint['name'] in model['joint_names']:
                pose=pose@_axis_angle_transform(np.asarray(joint['axis']),q[model['joint_names'].index(joint['name'])],joint['type'])
            frames[joint['child']]=pose
        if len(next_pending)==len(pending):raise ValueError('UNRESOLVED_NATIVE_MODEL_TREE')
        pending=next_pending
    return frames


def read_case(record,coordinates):
    files=[record/n for n in ('observation.json','observation.npz','segmentation.npz','geometry.json','site-scene.json','planning-failures.json','timing-prepare_plan.json')]
    sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    obs=json.loads(files[0].read_text());arr=np.load(files[1]);masks=np.load(files[2]);geo=json.loads(files[3].read_text());scene=json.loads(files[4].read_text())
    failures=sorted(json.loads(files[5].read_text()),key=lambda x:x['candidate']);timing=json.loads(files[6].read_text())
    rgb=arr['rgb'].copy();mask=masks['mask_0'].astype(bool);rgb[mask]=(rgb[mask]*.6+np.array([30,220,160])*.4).astype(np.uint8)
    image=io.BytesIO();Image.fromarray(rgb).save(image,format='PNG')
    candidates=[]
    for failure in failures:
        folder=record/(failure['target']+'-'+str(failure['candidate']));states=[];paths=[]
        precheck=folder/'endpoint-precheck.json'
        if precheck.exists():
            sources[str(precheck.relative_to(ROOT))]=hashlib.sha256(precheck.read_bytes()).hexdigest()
            for check in json.loads(precheck.read_text()):
                if check.get('q') is not None:
                    native=folder/(check['stage']+'-endpoint-native.json');attachment=None
                    if native.exists():
                        sources[str(native.relative_to(ROOT))]=hashlib.sha256(native.read_bytes()).hexdigest()
                        attachment=json.loads(native.read_text()).get('request',{}).get('attachment')
                    states.append(dict(q=check['q'],stage=check['stage'],failure=check.get('failure'),label=check['stage']+' 终态',attachment=attachment))
        for f in sorted(folder.glob('*native.json')):
            d=json.loads(f.read_text());reply=d.get('result',{});request=d.get('request',{})
            if not isinstance(reply,dict):continue
            points=reply.get('points',[])
            if points:
                sources[str(f.relative_to(ROOT))]=hashlib.sha256(f.read_bytes()).hexdigest()
                step=max(1,len(points)//150);shown=points[::step]
                if shown[-1]!=points[-1]:shown.append(points[-1])
                paths.append(dict(name=f.name,stage=request.get('stage','UNKNOWN'),status=reply.get('status'),
                    q=[p['q'] for p in shown],tcp=[coordinates.tcp(p['q'])[:3,3].tolist() for p in shown],
                    samples=len(points),displayed_samples=len(shown),
                    max_velocity_rad_s=max(abs(v) for p in points for v in p.get('v',[0])),
                    max_acceleration_rad_s2=max(abs(v) for p in points for v in p.get('a',[0])),
                    source=str(f.relative_to(ROOT)),attachment=request.get('attachment'),audited=False))
        candidates.append(dict(index=failure['candidate'],reason=failure['reason'],kind=kind(failure['reason']),states=states,paths=paths))
    return dict(task_id=record.name,observation_id=obs['observation_id'],q_start=obs['q_start'],q_capture=obs['q_capture'],
        stream=obs['stream'],world=scene['world'],box=geo['boxes'][0],candidates=candidates,
        duration_s=next(e['duration_s'] for e in timing['events'] if e['name']=='prepare_plan.entry_to_return'),
        image='data:image/png;base64,'+base64.b64encode(image.getvalue()).decode(),sources=sources,
        camera=obs['T_world_camera'],intrinsics=obs['intrinsics'],
        fk_fixture={k:v.tolist() for k,v in coordinates.frames(obs['q_start']).items()})


def overview(case,model,coordinates,output,number):
    fig=plt.figure(figsize=(16,9),dpi=140,facecolor='#f5f7fa')
    gs=fig.add_gridspec(2,2,height_ratios=[3.3,1.5],hspace=.28,wspace=.15)
    fig.subplots_adjust(top=.81)
    ax=fig.add_subplot(gs[0,0]);im=Image.open(io.BytesIO(base64.b64decode(case['image'].split(',')[1])));ax.imshow(im)
    camera=np.linalg.inv(case['camera']);uv=lambda p:_project(transform(camera,np.asarray(p)),case['intrinsics'])
    box=case['box'];points=np.array(box['corners_m']);edges=[(a,b) for a in range(8) for b in range(a+1,8) if bin(a^b).count('1')==1]
    for a,b in edges:ax.plot(*uv(points[[a,b]]).T,color='#ffc800',ls='--',lw=1.6)
    blocked=next(c for c in case['candidates'] if c['kind']=='ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED')
    state=next(s for s in blocked['states'] if s.get('failure'));failure=state['failure'];pair=failure['pair']
    obj=next((w for w in case['world'] if w['id'] in pair),None)
    if obj:
        p=corners(obj['pose'],obj['size'])
        for a,b in edges:ax.plot(*uv(p[[a,b]]).T,color='#ff4455',lw=1.3)
    ax.set(xlim=(0,640),ylim=(480,0));ax.axis('off');ax.set_title('原始冻结图像 + 同帧掩码\n绿：实际分割区域　黄虚线：完整箱体包络　红：被报告的障碍',fontsize=11)
    ax=fig.add_subplot(gs[0,1],projection='3d')
    bounds=[]
    for w in case['world']+[dict(id=box['id'],pose=box['pose'],size=box['size_m'])]:
        if w.get('category')=='workspace_boundary':continue  # local overview; retained in HTML
        p=corners(w['pose'],w['size']);bounds.extend(p);hit=w['id'] in pair;target=w['id']==box['id']
        faces=[[p[i] for i in face] for face in ((0,1,3,2),(4,5,7,6),(0,1,5,4),(2,3,7,6),(0,2,6,4),(1,3,7,5))]
        ax.add_collection3d(Poly3DCollection(faces,facecolors='#e65b66' if hit else '#e8b45b' if target else '#8c9dab',edgecolors='#e65b66' if hit else '#718499',alpha=.65 if hit else .22 if target else .08,linewidths=.4))
    frames=full_frames(model,state['q']);chain=np.array([frames['base_link'][:3,3]]+[frames[f'link_{i}'][:3,3] for i in range(1,7)]+[coordinates.tcp(state['q'])[:3,3]])
    ax.plot(*chain.T,'o-',color='#334e68',lw=4,ms=5,label='拒绝终态：连杆中心线');bounds.extend(chain)
    # Draw the reported tool solid's actual source mesh, without distance claims.
    for item in model['links']:
        if item['id'] not in pair or item['layer']!='collision' or item['kind']!='mesh':continue
        mesh=model['meshes'][item['asset']];v=np.frombuffer(base64.b64decode(mesh['positions_b64']),dtype='<f4').reshape(-1,3);idx=np.frombuffer(base64.b64decode(mesh['indices_b64']),dtype='<u4').reshape(-1,3)
        v=transform(frames[item['link']]@np.asarray(item['origin']),v);ax.add_collection3d(Poly3DCollection(v[idx],facecolors='#ef3340',alpha=.9,linewidths=0))
    tcp=coordinates.tcp(state['q'])[:3,3];ax.scatter(*tcp,s=60,c='#ef3340');p=np.array(bounds);lo=p.min(0);hi=p.max(0);center=(lo+hi)/2;radius=max(hi-lo)/2*1.06
    ax.set(xlim=(center[0]-radius,center[0]+radius),ylim=(center[1]-radius,center[1]+radius),zlim=(center[2]-radius,center[2]+radius),xlabel='base X / m',ylabel='base Y / m',zlabel='base Z / m');ax.set_box_aspect((1,1,1));ax.view_init(25,-125)
    gap=failure.get('surface_distance_m',-1);detail='相交/接触（负值是状态码，不是穿入深度）' if gap<0 else f'间隙 {gap*1000:.2f} mm < 要求 {failure["required_pair_clearance_m"]*1000:.2f} mm'
    ax.set_title(f'候选 {blocked["index"]} · 保存的拒绝终态（局部，边界见三维页面）\n{pair[0]} ↔ {pair[1][:28]}\n{detail}',fontsize=10)
    ax=fig.add_subplot(gs[1,:]);ax.set_xlim(-.7,31.7);ax.set_ylim(0,1);ax.axis('off')
    for c in case['candidates']:
        i=c['index'];ax.bar(i,.24,bottom=.64,width=.85,color=COLORS.get(c['kind'],'gray'));ax.text(i,.53,str(i),ha='center',fontsize=8)
    counts={k:sum(c['kind']==k for c in case['candidates']) for k in COLORS}
    ax.text(0,.32,f'全部32个候选：红＝终态阻挡 {counts["ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED"]}　黄＝插值限值 {counts["INTERPOLATION_LIMIT"]}　紫＝搜索预算耗尽 {counts["NATIVE_SOLVER_BUDGET_EXHAUSTED"]}',transform=ax.transAxes,fontsize=12)
    ax.text(0,.08,'搜索预算耗尽不等于已证明无解。候选局部轨迹可在配套三维页面查看；两次都没有完整已审计路径，也没有实际运动。',transform=ax.transAxes,fontsize=11,color='#4b6073')
    fig.suptitle(f'现场箱位 {number} · 规划失败诊断　{case["duration_s"]:.1f} s\n任务 {case["task_id"]}',fontsize=17,y=.98)
    fig.savefig(output,bbox_inches='tight',facecolor=fig.get_facecolor());plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('records',nargs='+',type=Path);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    config=load(ROOT/'configs/workbench/single_carton.example.json');installation=Installation.load(config['execution']['installation']);coordinates=CoordinateModel(URDF,URDF_SHA,installation.world_from_base,installation.tool)
    planner=SinglePlanner(config,load_mapping(config['execution']['mapping']),Authority('eco65_single_carton_segment_v1'),coordinates);planner.build()
    snapshot=deepcopy(load_snapshot(config['execution']['snapshot'],geometry_only=True));snapshot['native_model']['init']=planner.init;snapshot['scene']['mesh_obstacles']=[]
    snapshot['native_model']['init']['process_geometry']['expected_contour_ids']=planner.init['process_geometry']['connected_cup_ids'];model=export_model(snapshot)
    cases=[read_case(p.resolve(),coordinates) for p in args.records]
    for case in cases:
        for requestfile in args.records[cases.index(case)].glob('*/*endpoint-native.json'):
            request=json.loads(requestfile.read_text()).get('request',{})
            if 'identity' in request:assert request['identity']==planner.init['identity'],'MODEL_CHANGED';break
    for i,case in enumerate(cases,1):overview(case,model,coordinates,args.output/f'case-{i}.png',i)
    data=dict(model=model,cases=cases,physical_execution='NOT_RUN',executable=False)
    template=(ROOT/'tools/search_replay/field_failures.html').read_text();vendor=ROOT/'tools/search_replay/vendor';js=(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text()
    (args.output/'index.html').write_text(template.replace('/*__VENDOR__*/',js).replace('/*__DATA__*/',json.dumps(data,separators=(',',':')).replace('</','<\\/')))
    report=dict(cases=[dict(task_id=c['task_id'],candidates=len(c['candidates']),native_output_paths=sum(len(x['paths']) for x in c['candidates']),sources=c['sources']) for c in cases],collision_shapes=sum(x['layer']=='collision' for x in model['links']),cups=len(model['cup_ids']),hardware_execution='NOT_RUN')
    for case in cases:
        for name,digest in case['sources'].items():
            assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,'SAVED_EVIDENCE_CHANGED'
    report['saved_evidence_unchanged']=True
    (args.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()
