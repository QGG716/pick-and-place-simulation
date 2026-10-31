"""Dimensioned orthographic drawing from the same native scene solids."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from check_eco65_coupled_geometry import vertices
from unloading_sim.eco65.model import load_json,robot_for,ROOT
from unloading_sim.eco65.native_scene import world_objects
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.eco65.transverse_process import corners


def draw(snapshot,home,output):
    import trimesh
    s=verify_snapshot(load_json(snapshot));scene=s['scene'];cfg=scene['prototype']['configuration']
    q=load_json(home)['q_rad'];frames=robot_for(s['tool'],scene).named_link_frames(q)
    solids=[(r['id'],vertices(r),r.get('rgba',[.5,.6,.65,1])[:3],.18 if r.get('role')=='prototype_enclosure' else .35)
            for r in world_objects(scene)]
    solids.extend((b['id'],corners(b),[.75,.52,.27],.6) for b in scene['boxes'])
    from export_eco65_search_replay import export_model
    model=export_model(s)
    from scipy.spatial.transform import Rotation
    frames={'world':np.eye(4)};pending=list(model['joints'])
    while pending:
        next_rows=[]
        for joint in pending:
            if joint['parent'] not in frames:next_rows.append(joint);continue
            T=frames[joint['parent']]@np.asarray(joint['origin'])
            if joint['name'] in model['joint_names']:
                value=q[model['joint_names'].index(joint['name'])];motion=np.eye(4)
                if joint['type']=='prismatic':motion[:3,3]=np.asarray(joint['axis'])*value
                else:motion[:3,:3]=Rotation.from_rotvec(np.asarray(joint['axis'])*value).as_matrix()
                T=T@motion
            frames[joint['child']]=T
        if len(next_rows)==len(pending):raise ValueError('NATIVE_FRAME_TREE_UNRESOLVED')
        pending=next_rows
    # Native collision meshes include actual robot, tool and moving mast CAD.
    import base64
    for row in model['links']:
        if row['layer']!='collision':continue
        if row.get('asset'):
            v=np.frombuffer(base64.b64decode(model['meshes'][row['asset']]['positions_b64']),dtype=np.float32).reshape(-1,3)
        elif row.get('size'):v=trimesh.creation.box(row['size']).vertices
        elif row.get('kind')=='cylinder':v=trimesh.creation.cylinder(radius=row['radius'],height=row['length']).vertices
        else:continue
        # Model link names may include fixed tool/mast joints; use its full tree.
        parent=row['link'];local=np.asarray(row['origin'])
        T=frames[parent]@local
        solids.append((row['id'],v@T[:3,:3].T+T[:3,3],[.3,.35,.45],.45))
    fig,axes=plt.subplots(1,2,figsize=(15,7),layout='constrained')
    for ax,indices,title in [(axes[0],(0,1),'TOP: +X into enclosure, +Y left'),(axes[1],(0,2),'SIDE: common ground Z = 0')]:
        for name,v,color,alpha in solids:
            points=np.unique(v[:,indices],axis=0)*1000
            if len(points)<3:continue
            hull=ConvexHull(points)
            ax.add_patch(Polygon(points[hull.vertices],facecolor=color,edgecolor=color,alpha=alpha,lw=.6))
        base=np.asarray(scene['base_position_m'])*1000;ax.plot(base[indices[0]],base[indices[1]],'r+',ms=12,mew=2)
        ax.annotate('Robot base',base[list(indices)],xytext=(-30,25),textcoords='offset points',color='darkred')
        ax.axhline(0,color='black',lw=.6);ax.axvline(0,color='black',lw=.6)
        ax.set_title(title);ax.set_xlabel('X / mm');ax.set_ylabel(('Y' if indices[1]==1 else 'Z')+' / mm')
        ax.set_aspect('equal');ax.set_xlim(-1400,850);ax.set_ylim((-400,400) if indices[1]==1 else (-40,cfg['trailer']['inner_height_m']*1000+40));ax.grid(alpha=.15)
    c=cfg['cartons'];t=cfg['trailer'];r=cfg['receiver']
    stack=np.vstack([corners(b) for b in scene['boxes']])*1000;lo,hi=stack.min(0),stack.max(0)
    bounds=np.asarray(r['transverse_bounds_xy_m'])*1000;belt= bounds[1]-bounds[0]
    text=(f"Cartons: {' x '.join(f'{v*1000:.0f}' for v in c['size_m'])} mm; {c['columns']} columns x {c['layers']} layers = {len(scene['boxes'])}\n"
        f"Stack: X {lo[0]:.0f}..{hi[0]:.0f}; Y {lo[1]:.1f}..{hi[1]:.1f}; Z {lo[2]:.0f}..{hi[2]:.0f} mm\n"
        f"Enclosure clear L/W/H: {(t['closed_end_x_m']-t['entrance_x_m'])*1000:.0f} / {t['inner_width_m']*1000:.0f} / {t['inner_height_m']*1000:.0f} mm; -X entrance open\n"
        f"Base XYZ: {', '.join(f'{v*1000:.1f}' for v in scene['base_position_m'])} mm; fixed installation candidate\n"
        f"Transverse: {belt[1]:.0f} x {belt[0]:.0f} mm, surface travel {r['low_height_m']*1000:.0f}..{r['high_height_m']*1000:.0f} mm; longitudinal: {r['longitudinal_length_m']*1000:.0f} x {r['longitudinal_width_m']*1000:.0f} mm, far end {r['longitudinal_lower_height_m']*1000:.0f} mm\n"
        "Layout reference scale 1:4; real ECO65/tool/camera unscaled. Static dimension drawing, not a task success.")
    fig.suptitle(text,fontsize=11,ha='left',x=.07)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=140);fig.savefig(output.with_suffix('.svg'));plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','home','output'):p.add_argument('--'+name,required=True)
    a=p.parse_args();draw(a.snapshot,a.home,a.output)
