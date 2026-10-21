"""Static MuJoCo distance adapter covering the added calibration accessories.

No mj_step, dynamics, motion generation, or blanket board/probe contact waiver.
Convex envelopes are conservative; rejected overlaps are not CAD penetration proof.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import ROOT,transform,robot_for,load_json,save_json,mounting_pairs
from unloading_sim.eco65.native_model import POLICY
from eco65_touch_sources import verify_scene_sources


def pair_rule(a,b,mount,table_support=False):
    names=frozenset([('robot_'+e['owner']) if e['kind']=='robot' else e['owner'] for e in [a,b]])
    if names in mount:return dict(name='asset_bound_flange_mount',threshold_m=-POLICY['contact_tolerance_m'],source='tool.mounting_contacts')
    if names==frozenset(['robot_base_link','robot_mount_plate']):
        return dict(name='base_installation',threshold_m=-POLICY['contact_tolerance_m'],source='snapshot robot base/installation plate')
    if names==frozenset(['probe_bracket','part_000']):
        return dict(name='probe_bracket_rigid_plate',threshold_m=-POLICY['contact_tolerance_m'],source='configuration.probe.plate_source and bracket_T_F')
    if table_support and names==frozenset(['support_base','trailer_floor']):
        return dict(name='named_fixture_base_table_contact',threshold_m=-POLICY['contact_tolerance_m'],source='configuration.board.support.connections')
    self_pair=a['kind']==b['kind']=='robot'
    return dict(name='robot_self_clearance' if self_pair else 'general_clearance',
        threshold_m=POLICY['self_collision_clearance_m' if self_pair else 'required_pair_clearance_m'],source='eco65.native_model.POLICY')


def distance_evidence(query,threshold,distmax,analytic_contact=None):
    """Fail closed on unknown distance; designated analytical contact is mandatory."""
    try:
        value=float(query())
        if not np.isfinite(value):return dict(accepted=False,error='NONFINITE_DISTANCE',raw_value=repr(value),distance_m=None)
    except Exception as error:
        return dict(accepted=False,error='DISTANCE_BACKEND_EXCEPTION',detail=type(error).__name__+': '+str(error),distance_m=None)
    result=dict(distance_m=value,distmax_m=distmax,possibly_truncated=bool(value>=distmax-1e-9))
    if analytic_contact is not None:
        return dict(result,accepted=bool(analytic_contact['accepted']),analytic_contact=analytic_contact,
            decision='DESIGNATED_FINITE_DATUM_ANALYTIC_CHECK',error=None if analytic_contact['accepted'] else 'DESIGNATED_TOUCH_CONDITION_FAILED')
    return dict(result,accepted=bool(value+1e-9>=threshold),decision='NATIVE_CLEARANCE_POLICY',
        error=None if value+1e-9>=threshold else 'CLEARANCE_BELOW_POLICY')


def index_ranges(indices):
    ranges=[]
    for i in map(int,indices):
        if ranges and i==ranges[-1][1]:ranges[-1][1]=i+1
        else:ranges.append([i,i+1])
    return ranges


def accessory_meshes(s,c):
    """Return named local meshes shared by the display and distance adapter."""
    out=[];mast=s['scene']['mast'];T=np.array(mast['T_link1_mast'])
    def add(name,frame,m,T=np.eye(4),category='accessory'):
        m=m.copy();m.apply_transform(T);out.append(dict(name=name,frame=frame,mesh=m,category=category))
    for p in mast['parts']:
        if p['id'] not in ['mast_part_001','mast_part_010']:continue
        add(p['id'],'link_1',trimesh.load(ROOT/p['collision'],process=False),T@np.asarray(p.get('T_mast',np.eye(4))))
    pole=next(p for p in mast['parts'] if p['id']=='mast_part_019')
    head=T@np.asarray(pole.get('T_mast',np.eye(4)))
    # Exact primitive dimensions read from original mast_part_019 planar faces.
    add('mast_pole','link_1',trimesh.creation.cylinder(radius=.0125,height=.4,sections=32),head@transform([.1045,-.00405,.208]))
    for name,size,center in [('pocket_floor',[.087,.049,.002],[.113,-.00405,.409]),
        ('pocket_left',[.002,.049,.04],[.0705,-.00405,.43]),('pocket_right',[.002,.049,.04],[.1555,-.00405,.43])]:
        add(name,'link_1',trimesh.creation.box(size),head@transform(center))
    add('mounting_seat','link_1',trimesh.load(ROOT/mast['mount_interface']['seat']['mesh'],process=False))
    # A single conservative outer camera hull for external distance queries;
    # original per-part meshes remain the actual visual source.
    camera=trimesh.util.concatenate([trimesh.load(ROOT/p['visual'],process=False) for p in c['camera']['parts']])
    add('camera_body','link_1',camera,T@np.array(c['camera']['T_mast_CAD']))
    for p in c['camera'].get('pitch_adapter',[]):
        add(p['name'],'link_1',trimesh.creation.box(p['size_m']),T@p['T_mast'],'camera_adapter')
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
    for p in b.get('support',{}).get('parts',[]):
        add(p['name'],'board',trimesh.creation.box(p['size_m']),np.array(p['T_P']),'fixture')
    return out


class StaticGeometry:
    def __init__(self,s,c,out):
        import mujoco as mj
        from pxr import Usd,UsdGeom
        from build_eco65_full_face_layout import verify_static
        current = c.get('current_native_layout',False)
        if current:
            from unloading_sim.eco65.native_io import verify_snapshot
            verify_snapshot(s)
        else: verify_static(s)
        self.mj=mj;self.robot=robot_for(s['tool'],s['scene']);self.entries=[]
        self.sources=verify_scene_sources(c)
        if s['fingerprint']!=self.sources['files']['snapshot']['fingerprint']:raise ValueError('GEOMETRY_SNAPSHOT_MISMATCH')
        xml=ET.Element('mujoco');ET.SubElement(xml,'compiler',angle='radian')
        assets=ET.SubElement(xml,'asset');world=ET.SubElement(xml,'worldbody')
        def add(name,vertices,frame,T,kind,owner):
            label=name
            name='g'+str(len(self.entries))
            ET.SubElement(assets,'mesh',name=name,vertex=' '.join(map(str,np.asarray(vertices).reshape(-1))))
            body=ET.SubElement(world,'body',name=name,mocap='true')
            ET.SubElement(body,'geom',name=name,type='mesh',mesh=name,contype='0',conaffinity='0')
            self.entries.append(dict(name=name,source_label=label,frame=frame,T=np.array(T),kind=kind,owner=owner,vertices=np.asarray(vertices)))
        manifest=load_json(ROOT/self.sources['files']['geometry_manifest']['path'])
        old=self.robot.named_link_frames(manifest['home_q_rad']);cad=np.array(s['tool']['T_flange_tool_cad'])
        for row in manifest['collision_records']:
            path=row['path']
            if current and '/NativeWorldCollision/' in path:
                add(path,row['points'],'D',row['T_D'],'environment',path.rsplit('/',1)[1]);continue
            if current and '/NativeCollision/' in path:
                parent=path.split('/')[3];owner=path.rsplit('/',1)[1]
                add(path,row['points'],parent,np.linalg.inv(old[parent])@row['T_D'],'accessory',owner);continue
            if '/Mast/' in path or '/MountingSeat/' in path:continue
            if '/Robot/' in path:
                owner=path.split('/')[3];add(path,row['points'],owner,np.linalg.inv(old[owner])@row['T_D'],'robot',owner)
            else:
                owner=path.rsplit('/',1)[1].removeprefix('collision_').removesuffix('_collision')
                add(path,row['points'],'link_6',np.linalg.inv(old['link_6'])@row['T_D'],'tool',owner)
        for obj in s['scene']['obstacles']+[dict(id=b['id'],size_m=b['size_m'],position_m=b['pose']) for b in s['scene']['boxes']]:
            add(obj['id'],trimesh.creation.box(obj['size_m']).vertices,'D',transform(obj['position_m']),'environment',obj['id'])
        self.conveyor_count=0
        for item in ([] if current else s['scene']['conveyors']['items']):
            stage=Usd.Stage.Open(str(ROOT/item['usd_path']))
            for row in item['meshes']:
                if not row['collision']:continue
                m=UsdGeom.Mesh(stage.GetPrimAtPath(row['prim']));v=np.array(m.GetPointsAttr().Get());idx=np.array(m.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
                for n in m.GetFaceVertexCountsAttr().Get():
                    ids=idx[offset:offset+n];offset+=n;faces.extend([[ids[0],ids[k],ids[k+1]] for k in range(1,n-1)])
                for part in trimesh.Trimesh(v,np.array(faces),process=True).split(only_watertight=False):
                    if len(part.vertices)<4 or np.linalg.matrix_rank(part.vertices-part.vertices.mean(0))<3:continue
                    add(item['id']+':'+row['prim']+':component_'+str(self.conveyor_count),part.convex_hull.vertices,'D',np.eye(4),'environment',item['id']);self.conveyor_count+=1
        self.extra=accessory_meshes(s,c)
        if current:self.extra=[r for r in self.extra if r['category'] in ('probe','fixture')]
        self.config=c;folder=out/'geometry';folder.mkdir()
        for row in self.extra:
            row['mesh'].export(folder/(row['name']+'.obj'))
            add(row['name'],row['mesh'].vertices,row['frame'],np.eye(4),'accessory' if row['category']=='camera_adapter' else row['category'],row['name'])
        self.model=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));self.data=mj.MjData(self.model)
        for e in self.entries:e['gid']=self.model.geom(e['name']).id;e['bid']=self.model.body(e['name']).mocapid[0]
        adjacent={frozenset([j.parent,j.child]) for j in self.robot.joints};mount=mounting_pairs(s['tool']);self.pairs=[]
        native_disabled=set()
        if current:
            srdf=ET.fromstring(s['native_model']['init']['parameters']['robot_description_semantic'])
            native_disabled={frozenset([r.get('link1'),r.get('link2')]) for r in srdf.findall('disable_collisions')}
        skipped=dict(same_rigid_assembly=0,static_environment_internal=0,URDF_adjacent_links=0,named_fixed_interfaces=[])
        for i,a in enumerate(self.entries):
            for b in self.entries[i+1:]:
                if a['kind']==b['kind']=='environment':skipped['static_environment_internal']+=1;continue
                if current and (a['owner']==b['owner'] or frozenset([a['owner'],b['owner']]) in native_disabled):
                    skipped['same_rigid_assembly']+=1;continue
                if a['kind']==b['kind'] and a['frame']==b['frame'] and a['kind'] in ['tool','accessory','probe','fixture']:
                    skipped['same_rigid_assembly']+=1;continue
                if a['kind']==b['kind']=='robot' and frozenset([a['owner'],b['owner']]) in adjacent:skipped['URDF_adjacent_links']+=1;continue
                owners={a['owner'],b['owner']}
                if owners in [{'mast_part_001','link_1'},{'mounting_seat','link_1'}]:
                    interface=s['scene']['mast']['mount_interface']
                    if not interface.get('source') or not interface.get('seat'):raise ValueError('NAMED_MAST_INTERFACE_SOURCE_REQUIRED')
                    skipped['named_fixed_interfaces'].append(dict(pair=sorted(owners),source=interface,
                        scope='Existing fixed interface; conformal original-triangle contact not requalified by convex envelopes'))
                    continue
                self.pairs.append((a,b,pair_rule(a,b,mount,bool(c['board'].get('support')))))
        self.a=np.array([a['gid'] for a,b,t in self.pairs]);self.b=np.array([b['gid'] for a,b,t in self.pairs])
        self.radii=self.model.geom_rbound[self.a]+self.model.geom_rbound[self.b]
        self.query_horizon=max(POLICY['required_pair_clearance_m'],POLICY['self_collision_clearance_m'],
            POLICY['contact_tolerance_m'],c['thresholds']['tip_target_m'])+POLICY['mesh_error_reserve_m']
        self.backend=dict(mujoco_version=mj.__version__,query='mj_geomDistance on compiled convex mesh pairs after mj_forward; no mj_step',
            enableflags=int(self.model.opt.enableflags),disableflags=int(self.model.opt.disableflags),
            ccd_iterations=int(self.model.opt.ccd_iterations),ccd_tolerance=float(self.model.opt.ccd_tolerance),distmax_m=self.query_horizon)
        save_json(out/'pair_catalog.json',dict(policy=POLICY,backend=self.backend,scene_sources=self.sources,
            derived_scene_identity=c['derived_scene_identity'],skipped=skipped,
            geometries=[{k:e[k] for k in ['name','source_label','frame','kind','owner']} for e in self.entries],
            pairs=[dict(index=i,geometries=[a['name'],b['name']],owners=[a['owner'],b['owner']],rule=rule)
                   for i,(a,b,rule) in enumerate(self.pairs)]))

    def check(self,q,P,point_id,*,contact_neighborhood=False):
        if not self.robot.within_limits(q):return dict(accepted=False,failures=['joint_limits'])
        frames=self.robot.named_link_frames(q);frames.update(D=np.eye(4),board=P)
        for e in self.entries:
            T=frames[e['frame']]@e['T'];bid=e['bid']
            self.data.mocap_pos[bid]=T[:3,3];self.data.mocap_quat[bid]=Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]
        try:self.mj.mj_forward(self.model,self.data)
        except Exception as error:return dict(accepted=False,failures=[dict(error='FORWARD_BACKEND_EXCEPTION',detail=str(error))],backend=self.backend)
        if not np.isfinite(self.data.geom_xpos).all():return dict(accepted=False,failures=[dict(error='NONFINITE_GEOMETRY_POSITIONS')],backend=self.backend)
        lower=np.linalg.norm(self.data.geom_xpos[self.a]-self.data.geom_xpos[self.b],axis=1)-self.radii
        if not np.isfinite(lower).all():return dict(accepted=False,failures=[dict(error='NONFINITE_BROADPHASE_BOUND')],backend=self.backend)
        failures=[];queries=[];tip_contact=None
        designated=np.array([{a['owner'],b['owner']}=={'probe_tip','stud_'+str(point_id)} for a,b,_ in self.pairs])
        selected=(lower<=self.query_horizon)|designated
        for index in np.flatnonzero(selected):
            a,b,rule=self.pairs[index];threshold=rule['threshold_m'];analytic=None
            if designated[index]:
                threshold=-self.config['thresholds']['tip_target_m']
                # A pointed convex mesh at exactly coincident support planes
                # can yield an unstable EPA signed distance. Verify the actual
                # cone vertices against the finite target disk, not a waiver.
                cone=next(r['mesh'] for r in self.extra if r['name']=='probe_tip')
                T=np.linalg.inv(P)@frames['link_6'];v=np.array(cone.vertices)@T[:3,:3].T+T[:3,3]
                target=np.array(next(p['p_P_m'] for p in self.config['board']['touch_points'] if p['point_id']==point_id))
                tip=T[:3,:3]@self.config['probe']['p_F_tip_m']+T[:3,3]
                plane_gap=float(target[2]-v[:,2].max());tip_error=float(np.linalg.norm(tip-target))
                radial_error=float(np.linalg.norm(tip[:2]-target[:2]));radius=self.config['board']['stud_radius_m']
                arrival=tip_error<self.config['thresholds']['tip_target_m']
                analytic=dict(accepted=bool(np.isfinite([plane_gap,tip_error,radial_error]).all() and plane_gap>=-self.config['thresholds']['tip_target_m'] and (arrival or contact_neighborhood) and radial_error<=radius),
                    support_plane_gap_m=plane_gap,tip_error_m=tip_error,finite_datum_radial_error_m=radial_error,finite_datum_radius_m=radius,
                    arrival_required=not contact_neighborhood,method='Finite target centre and separating plane of every original cone vertex; designated tip only')
            evidence=distance_evidence(lambda:self.mj.mj_geomDistance(self.model,self.data,a['gid'],b['gid'],self.query_horizon,None),threshold,self.query_horizon,analytic)
            row=dict(pair_index=int(index),pair=[a['owner'],b['owner']],geometries=[a['name'],b['name']],
                rule='designated_tip_datum' if designated[index] else rule['name'],threshold_m=threshold,
                legacy_threshold_m=(-self.config['thresholds']['tip_target_m'] if designated[index] else (-.0004 if threshold<0 else .0001)),
                broadphase_lower_bound_m=float(lower[index]),**evidence)
            queries.append(row)
            if designated[index]:tip_contact=row
            if not row['accepted']:failures.append(row)
        if point_id is not None and tip_contact is None:failures.append(dict(error='DESIGNATED_TOUCH_PAIR_NOT_FOUND',point_id=point_id))
        raw=[r['distance_m'] for r in queries if r.get('distance_m') is not None and not r.get('possibly_truncated')]
        excluded=np.flatnonzero(~selected)
        return dict(accepted=not failures,failures=failures,failure_count=len(failures),distance_queries=queries,
            broadphase=dict(excluded_pair_index_ranges=index_ranges(excluded),range_convention='start inclusive, end exclusive; indices address pair_catalog.json',
                excluded_count=len(excluded),tested_count=len(queries),rule='sphere-centre distance minus compiled radii > policy-derived distmax',
                minimum_excluded_lower_bound_m=float(lower[excluded].min()) if len(excluded) else None),
            minimum_raw_untruncated_query_m=min(raw) if raw else None,minimum_scope='Queried subset only, includes raw EPA contact output; NOT an exact scene minimum',
            tip_contact=tip_contact,conveyor_components=self.conveyor_count,geometry_count=len(self.entries),backend=self.backend,policy=POLICY,
            scope='Discrete static convex distances with named fixed interfaces; no dynamics or swept paths')
