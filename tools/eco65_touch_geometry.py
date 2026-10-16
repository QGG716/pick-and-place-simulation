"""Static MuJoCo distance adapter covering the added calibration accessories.

No mj_step, dynamics, motion generation, or blanket board/probe contact waiver.
Convex envelopes are conservative; rejected overlaps are not CAD penetration proof.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import ROOT,transform,robot_for,load_json,mounting_pairs


def accessory_meshes(s,c):
    """Return named local meshes shared by the display and distance adapter."""
    out=[];mast=s['scene']['mast'];T=np.array(mast['T_link1_mast'])
    def add(name,frame,m,T=np.eye(4),category='accessory'):
        m=m.copy();m.apply_transform(T);out.append(dict(name=name,frame=frame,mesh=m,category=category))
    for p in mast['parts']:
        if p['id'] not in ['mast_part_001','mast_part_010']:continue
        add(p['id'],'link_1',trimesh.load(ROOT/p['collision'],process=False),T)
    # Exact primitive dimensions read from original mast_part_019 planar faces.
    add('mast_pole','link_1',trimesh.creation.cylinder(radius=.0125,height=.4,sections=32),T@transform([.1045,-.00405,.208]))
    for name,size,center in [('pocket_floor',[.087,.049,.002],[.113,-.00405,.409]),
        ('pocket_left',[.002,.049,.04],[.0705,-.00405,.43]),('pocket_right',[.002,.049,.04],[.1555,-.00405,.43])]:
        add(name,'link_1',trimesh.creation.box(size),T@transform(center))
    add('mounting_seat','link_1',trimesh.load(ROOT/mast['mount_interface']['seat']['mesh'],process=False))
    # A single conservative outer camera hull for external distance queries;
    # original per-part meshes remain the actual visual source.
    camera=trimesh.util.concatenate([trimesh.load(ROOT/p['visual'],process=False) for p in c['camera']['parts']])
    add('camera_body','link_1',camera,T@np.array(c['camera']['T_mast_CAD']))
    p=c['probe'];tip=np.array(p['p_F_tip_m']);start=np.array(p['rod_start_F_m']);cone=p['cone_length_m']
    mid=(tip+start)/2;mid[2]-=cone/2
    add('probe_rod','link_6',trimesh.creation.cylinder(radius=p['radius_m'],height=tip[2]-start[2]-cone,sections=24),transform(mid),'probe')
    add('probe_tip','link_6',trimesh.creation.cone(radius=p['radius_m'],height=cone,sections=24),transform(tip-[0,0,cone]),'probe')
    add('probe_bracket','link_6',trimesh.creation.box(p['bracket_size_m']),np.array(p['bracket_T_F']),'probe')
    b=c['board'];size=b['outer_size_m'];pattern=b['pattern_size_m'];th=b['thickness_m']
    add('board_body','board',trimesh.creation.box([*size,th]),transform([pattern[0]/2,pattern[1]/2,th/2]),'fixture')
    for p in b['touch_points']:
        xyz=np.array(p['p_P_m']);xyz[2]/=2
        add('stud_'+p['point_id'],'board',trimesh.creation.cylinder(radius=b['stud_radius_m'],height=b['stud_projection_m'],sections=24),transform(xyz),'fixture')
    return out


class StaticGeometry:
    def __init__(self,s,c,out):
        import mujoco as mj
        from pxr import Usd,UsdGeom
        self.mj=mj;self.robot=robot_for(s['tool'],s['scene']);self.entries=[]
        xml=ET.Element('mujoco');ET.SubElement(xml,'compiler',angle='radian')
        assets=ET.SubElement(xml,'asset');world=ET.SubElement(xml,'worldbody')
        def add(name,vertices,frame,T,kind,owner):
            name='g'+str(len(self.entries))
            ET.SubElement(assets,'mesh',name=name,vertex=' '.join(map(str,np.asarray(vertices).reshape(-1))))
            body=ET.SubElement(world,'body',name=name,mocap='true')
            ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,contype='0',conaffinity='0')
            self.entries.append(dict(name=name,frame=frame,T=np.array(T),kind=kind,owner=owner))
        manifest=load_json(ROOT/'outputs/eco65_native/j1_mast_20261016_connected_run01/build/usd_A/workstation_manifest.json')
        old=self.robot.named_link_frames(manifest['home_q_rad']);cad=np.array(s['tool']['T_flange_tool_cad'])
        for row in manifest['collision_records']:
            path=row['path']
            if '/Mast/' in path or '/MountingSeat/' in path:continue
            if '/Robot/' in path:
                owner=path.split('/')[3];add(path,row['points'],owner,np.linalg.inv(old[owner])@row['T_D'],'robot',owner)
            else:
                owner=path.rsplit('/',1)[1].removeprefix('collision_')
                add(path,row['points'],'link_6',np.linalg.inv(old['link_6'])@row['T_D'],'tool',owner)
        for obj in s['scene']['obstacles']+[dict(id=b['id'],size_m=b['size_m'],position_m=b['pose']) for b in s['scene']['boxes']]:
            add(obj['id'],trimesh.creation.box(obj['size_m']).vertices,'D',transform(obj['position_m']),'environment',obj['id'])
        self.conveyor_count=0
        for item in s['scene']['conveyors']['items']:
            stage=Usd.Stage.Open(str(ROOT/item['usd_path']))
            for row in item['meshes']:
                if not row['collision']:continue
                m=UsdGeom.Mesh(stage.GetPrimAtPath(row['prim']));v=np.array(m.GetPointsAttr().Get());idx=np.array(m.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
                for n in m.GetFaceVertexCountsAttr().Get():
                    ids=idx[offset:offset+n];offset+=n;faces.extend([[ids[0],ids[k],ids[k+1]] for k in range(1,n-1)])
                for part in trimesh.Trimesh(v,np.array(faces),process=True).split(only_watertight=False):
                    if len(part.vertices)<4 or np.linalg.matrix_rank(part.vertices-part.vertices.mean(0))<3:continue
                    add('conveyor',part.convex_hull.vertices,'D',np.eye(4),'environment',item['id']);self.conveyor_count+=1
        self.extra=accessory_meshes(s,c);self.config=c;folder=out/'geometry';folder.mkdir()
        for row in self.extra:
            row['mesh'].export(folder/(row['name']+'.obj'))
            add(row['name'],row['mesh'].vertices,row['frame'],np.eye(4),row['category'],row['name'])
        self.model=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));self.data=mj.MjData(self.model)
        for e in self.entries:e['gid']=self.model.geom(e['name']).id;e['bid']=self.model.body(e['name']).mocapid[0]
        adjacent={frozenset([j.parent,j.child]) for j in self.robot.joints};mount=mounting_pairs(s['tool']);self.pairs=[]
        for i,a in enumerate(self.entries):
            for b in self.entries[i+1:]:
                if a['kind']==b['kind'] and a['kind'] in ['environment','tool','accessory','probe','fixture']:continue
                if a['kind']==b['kind']=='robot' and frozenset([a['owner'],b['owner']]) in adjacent:continue
                owners={a['owner'],b['owner']}
                if owners in [{'mast_part_001','link_1'},{'mounting_seat','link_1'}]:continue
                names=frozenset([('robot_'+e['owner']) if e['kind']=='robot' else e['owner'] for e in [a,b]])
                allowance=names in mount or names==frozenset(['robot_base_link','robot_mount_plate']) or owners=={'probe_bracket','part_000'}
                self.pairs.append((a,b,-.0004 if allowance else .0001))
        self.a=np.array([a['gid'] for a,b,t in self.pairs]);self.b=np.array([b['gid'] for a,b,t in self.pairs])
        self.radii=self.model.geom_rbound[self.a]+self.model.geom_rbound[self.b]

    def check(self,q,P,point_id):
        if not self.robot.within_limits(q):return dict(accepted=False,failures=['joint_limits'])
        frames=self.robot.named_link_frames(q);frames.update(D=np.eye(4),board=P)
        for e in self.entries:
            T=frames[e['frame']]@e['T'];bid=e['bid']
            self.data.mocap_pos[bid]=T[:3,3];self.data.mocap_quat[bid]=Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]
        self.mj.mj_forward(self.model,self.data)
        lower=np.linalg.norm(self.data.geom_xpos[self.a]-self.data.geom_xpos[self.b],axis=1)-self.radii
        failures=[];minimum=1.;checked=0;tip_contact=None
        for index in np.flatnonzero(lower<=.003):
            a,b,threshold=self.pairs[index]
            if {a['owner'],b['owner']}=={'probe_tip','stud_'+str(point_id)}:threshold=-.0001
            d=float(self.mj.mj_geomDistance(self.model,self.data,a['gid'],b['gid'],.003,None));checked+=1;minimum=min(minimum,d)
            if {a['owner'],b['owner']}=={'probe_tip','stud_'+str(point_id)}:
                # A pointed convex mesh at exactly coincident support planes
                # can yield an unstable EPA signed distance. Verify the actual
                # cone vertices against the finite target disk, not a waiver.
                cone=next(r['mesh'] for r in self.extra if r['name']=='probe_tip')
                T=np.linalg.inv(P)@frames['link_6'];v=np.array(cone.vertices)@T[:3,:3].T+T[:3,3]
                target=np.array(next(p['p_P_m'] for p in self.config['board']['touch_points'] if p['point_id']==point_id))
                tip=T[:3,:3]@self.config['probe']['p_F_tip_m']+T[:3,3]
                plane_gap=float(target[2]-v[:,2].max());tip_error=float(np.linalg.norm(tip-target))
                tip_contact=dict(raw_mujoco_distance_m=d,support_plane_gap_m=plane_gap,tip_error_m=tip_error,
                    method='Finite target centre and separating plane of every original cone vertex; designated tip only')
                if plane_gap>=-self.config['thresholds']['tip_target_m'] and tip_error<self.config['thresholds']['tip_target_m']:continue
            if d<threshold-1e-9:failures.append(dict(pair=[a['owner'],b['owner']],distance_m=d,threshold_m=threshold))
        return dict(accepted=not failures,failures=failures[:12],failure_count=len(failures),exact_convex_queries=checked,
            minimum_queried_distance_m=minimum,tip_contact=tip_contact,conveyor_components=self.conveyor_count,geometry_count=len(self.entries),
            scope='All original collision objects plus camera, mast, seat, probe and board; conservative convex distances, no dynamics/swept paths')
