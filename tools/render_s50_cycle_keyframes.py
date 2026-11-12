"""Render actual recorded CAD poses as figures (not browser screenshots)."""
from pathlib import Path
import sys,json,argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,tool_virtual
from unloading_sim.s50_cad_model import read_obj
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.geometry import OBB

FACES=np.array([[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]])


def render(report,output,phases=None):
    data=json.loads(Path(report).read_text(encoding='utf8'));output=Path(output);output.mkdir(parents=True,exist_ok=True)
    c=make_context(ROOT,ROOT/'outputs/s50_kinematic_keyframe_model')
    if data['source_model_sha256']!=c.definition['definition_sha256']:raise ValueError('KEYFRAME_MODEL_CHANGED')
    meshes={}
    for link in c.definition['links']:
        v,t=read_obj(ROOT/link['visual_mesh']['path']);meshes[link['name']]=(v,t)
    plt.rcParams['font.sans-serif']=['Microsoft YaHei','DejaVu Sans'];plt.rcParams['axes.unicode_minus']=False
    colors=['#7f95a5','#4b8195','#a9beca','#d6dee2','#65c4c8','#76a7b5','#c9e7e6']
    names={'home':'独立求解初态','withdraw':'正吸后抽离','transfer':'转运至横带 / 收拢','exit':'释放后退出'}
    closest_path=Path(report).parent/'closest-belt-pose.json'
    closest=json.loads(closest_path.read_text(encoding='utf8')) if closest_path.is_file() else None
    if closest:names['closest_belt']='横带最近记录构型 / 出料避让'
    prior=output/'keyframes.json'
    records=([r for r in json.loads(prior.read_text(encoding='utf8')) if r['phase'] not in phases]
        if phases and prior.is_file() else [])
    for phase in names:
        if phases and phase not in phases:continue
        stage=({'last_sample':closest['sample_index']} if phase=='closest_belt'
            else next((s for s in data['stages'] if s['phase']==phase),None))
        if stage is None:continue
        index=stage['last_sample'];sample=data['samples'][index];q=np.array(sample['q_rad'])
        fig=plt.figure(figsize=(12.8,9.6),dpi=130,facecolor='#f6f9fb')
        ax=fig.add_axes([0,.10,1,.83],projection='3d');ax.set_facecolor('#f6f9fb')
        def poly(vertices,faces,color,alpha=1,shade=False):
            verts=vertices[np.asarray(faces)]
            collection=Poly3DCollection(verts,facecolors=color,linewidths=0,alpha=alpha,shade=shade)
            ax.add_collection3d(collection)
        frames=c.robot.named_link_frames(q)
        for (name,(v,t)),color in zip(meshes.items(),colors):
            T=frames[name];poly(v@T[:3,:3].T+T[:3,3],t,color,shade=True)
        for b in coupled_conveyor_state(c.layout,sample['belt_height_m'],tuple(c.layout.cartons())).boxes:
            if b.name==data['target']:
                T=np.array(sample['world_from_carton']);b=OBB(T[:3,3],b.half_extents,T[:3,:3],b.name,b.category)
            if b.name.startswith('trailer_'):continue # Camera diagram omits walls only; runtime collision retained.
            color='#efbc59' if b.name==data['target'] else '#bd9c73' if b.category=='carton' else '#3d919e' if 'conveyor' in b.name else '#6c8391'
            solid=getattr(b,'solid',None)
            if solid:
                v=np.array(solid['vertices_m'])@b.rotation.T+b.center
                tri=[[f[0],f[i],f[i+1]] for f in solid['faces'] for i in range(1,len(f)-1)]
                poly(v,tri,color,shade=True)
            else:poly(b.corners(),FACES,color,shade=True)
        tv=tool_virtual(c,q)
        for key,color in [('rigid_local_boxes_virtual_tcp_m','#397a9a'),('compliant_local_boxes_virtual_tcp_m','#65cbb2')]:
            for r in c.intrinsic[key]:
                r=np.array(r);b=OBB(tv[:3,:3]@r[:3]+tv[:3,3],r[3:]/2,tv[:3,:3],'tool','tool');poly(b.corners(),FACES,color,shade=True)
        ax.set(xlim=(-2.5,.65),ylim=(-1.25,1.15),zlim=(.2,2.8),xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]')
        ax.set_box_aspect((3.15,2.4,2.6));ax.view_init(elev=19,azim=-64)
        ax.tick_params(labelsize=9);ax.grid(False)
        fig.text(.045,.955,'S50-2000  |  '+names[phase],fontsize=20,weight='bold',color='#17354a')
        fig.text(.045,.919,f"{data['target']} · t={sample['time_s']:.2f} s · 横带 {sample['belt_height_m']*1000:.0f} mm · {'吸附' if sample['attached'] else '未吸附'}",fontsize=12,color='#38576b')
        metric=closest['compactness'] if phase=='closest_belt' else next(m for m in data['compactness'] if m['phase']==phase)
        values=[]
        for key,label in [('active_links','活动连杆'),('elbow','肘部'),('wrist','腕部')]:
            g=metric['groups'][key];xy=g['envelope']['horizontal_span_xy_m']
            values.append(f"{label}：距横带 {1000*g['minimum_unsigned_separation_m']:.1f} mm；X×Y 占用 {1000*xy[0]:.0f}×{1000*xy[1]:.0f} mm")
        fig.text(.045,.073,'\n'.join(values),fontsize=10,color='#25475d')
        fig.text(.045,.015,'实际关节序列的 CAD 网格渲染；非浏览器截图。临时运动学约定，非动力学验证。',fontsize=10,color='#637887')
        path=output/(phase+'.png');fig.savefig(path,facecolor=fig.get_facecolor());plt.close(fig)
        records.append(dict(phase=phase,sample_index=index,q_rad=q.tolist(),file=path.name,render_scope='ACTUAL_CAD_RECORDED_POSE_NOT_BROWSER_SCREENSHOT'))
        print(path,flush=True)
    (output/'keyframes.json').write_text(json.dumps(records,indent=2)+'\n',encoding='utf8')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--phases',nargs='+',help='Optional subset of home, withdraw, transfer, exit, closest_belt')
    a=p.parse_args();render(a.report,a.output,a.phases)
