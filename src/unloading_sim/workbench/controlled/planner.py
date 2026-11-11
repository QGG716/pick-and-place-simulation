"""Fresh native MoveIt planning against source-bound complete geometry.

Historical snapshot supplies geometry only. No historical plan or acceptance
report can mint an execution permit. This module has no hardware imports.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import uuid
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ..coordinates import Installation
from .trajectory import Limits, digest, prepare,expected_controller_payload

ROOT=Path(__file__).resolve().parents[4]
POLICY=dict(schema='eco65_pair_collision_policy_v1',collision_margin_m=.002,
            mesh_error_reserve_m=.0002,required_pair_clearance_m=.0024,
            self_collision_clearance_m=.0024,contact_tolerance_m=.0004)
# Installed Humble Pilz trajectory_generator_ptp.h: MIN_MOVEMENT = 0.001 rad.
# Below this, PTP can return a single stationary point at t=0.1, not a timed
# motion. Sensor variation after "fill current" must not become a motion task.
MIN_GOAL_CHANGE_RAD=.001

def validate_live_goal(sample,goal,now=None):
    now=time.monotonic() if now is None else now
    if (sample is None or sample.get('source')!='LIVE'
            or not isinstance(sample.get('received_s'),(int,float))
            or not 0<=now-sample['received_s']<=.1):
        raise ValueError('FRESH_LIVE_START_REQUIRED')
    q=np.asarray(sample.get('q',[]),float);goal=np.asarray(goal,float)
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('INVALID_LIVE_START')
    if goal.shape!=(6,) or not np.isfinite(goal).all() or np.max(abs(goal-q))>np.deg2rad(10):
        raise ValueError('NEARBY_GOAL_REQUIRED_MAX_10_DEGREES')
    delta=float(np.max(abs(goal-q)))
    if delta<=MIN_GOAL_CHANGE_RAD:
        raise ValueError(f'目标与当前实测姿态基本相同（最大关节差 {np.rad2deg(delta):.4f}°），无需运动。'
                         '请先修改目标，再规划；至少一个关节差需大于 0.0573°（当前规划器最小运动量）。')
    return q.tolist(),goal

def file_hash(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def load_config(path):
    path=Path(path).resolve();c=json.loads(path.read_text())
    if c.get('schema')!='eco65_controlled_empty_config_v1':raise ValueError('CONTROLLED_CONFIG_REQUIRED')
    if c.get('period_s')!=.005:raise ValueError('REVIEWED_CANFD_PERIOD_MUST_BE_5_MS')
    if c.get('tool_mass_kg')!=1.5 or c.get('robot_rated_payload_kg')!=5:raise ValueError('CONFIRMED_MASS_CHANGED')
    controller_mass=expected_controller_payload(c)
    if controller_mass!=c['tool_mass_kg'] and not (controller_mass==3. and c.get('controller_payload_basis')=='USER_REQUESTED_EMPTY_TEST_3_KG'):
        raise ValueError('CONTROLLER_TEST_PAYLOAD_REQUIRES_EXPLICIT_FIELD_SETTING')
    if c.get('field_confirmation')!='USER_CONFIRMED_DEVICES_COORDINATES_CALIBRATION':raise ValueError('FIELD_CONTEXT_MISSING')
    for k in ('snapshot','installation','mapping','record_root'):
        c[k]=str((ROOT/c[k]).resolve())
    c['config_path']=str(path)
    return c

class EmptyPlanner:
    def __init__(self,config,mapping,authority):
        self.config,self.mapping,self.authority=config,mapping,authority
        self.files={};self.client=None;self.ready=False
    def track(self,path,expected=None):
        path=Path(path).resolve();value=file_hash(path)
        if expected is not None and value!=expected:raise ValueError('ASSET_HASH_MISMATCH:'+str(path))
        self.files[str(path)]=value;return path
    def binding(self):
        current={p:file_hash(p) for p in self.files}
        if current!=self.files:raise ValueError('INPUT_CHANGED_REBUILD_PLAN')
        return dict(files=current,model=self.model_hash,scene=self.scene_hash,
                    implementation_commit=self.commit,
                    joint_mapping=asdict(self.mapping),tool_mass_properties=self.mass,
                    controller_payload_kg=expected_controller_payload(self.config),
                    controller_payload_basis=self.config.get('controller_payload_basis','MATCH_PHYSICAL_TOOL_MASS'),
                    field_confirmation=self.config['field_confirmation'],
                    camera_mount='link_1',optical_extrinsics_use='NOT_USED_BY_JOINT_GOAL',
                    world_from_base=np.eye(4).tolist(),source_snapshot_geometry_only=True)
    def build(self):
        if self.ready:return
        self.commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        self.track(self.config['config_path']);self.track(self.config['mapping'])
        installation=Installation.load(self.track(self.config['installation']))
        if installation.camera_parent_frame!='link_1' or not np.allclose(installation.world_from_base,np.eye(4)):
            raise ValueError('CONFIRMED_INSTALLATION_CHANGED')
        s=json.loads(self.track(self.config['snapshot']).read_text());self.source=s
        content=dict(s);expected=content.pop('fingerprint')
        if digest(content)!=expected:raise ValueError('SOURCE_GEOMETRY_SNAPSHOT_CHANGED')
        if installation.tool is None or not np.allclose(installation.tool.matrix,s['tool']['T_flange_tcp'],atol=1e-10,rtol=0):
            raise ValueError('CONFIRMED_TOOL_AND_COLLISION_MODEL_DISAGREE')
        native=s['native_model'];old=native['init'];u=ET.fromstring(old['parameters']['robot_description'])
        hashes=dict(s['asset_sha256'])
        for row in native['convex_bindings']:
            if row.get('convex_sha256'):hashes[row['convex_path']]=row['convex_sha256']
        def local_mesh(name):
            relative=None
            for prefix in ('assets','outputs'):
                if '/'+prefix+'/' in name:relative=prefix+'/'+name.split('/'+prefix+'/',1)[1];break
                if name.startswith(prefix+'/'):relative=name;break
            if relative is None or relative not in hashes:raise ValueError('UNBOUND_MESH:'+name)
            return self.track(ROOT/relative,hashes[relative])
        for mesh in u.findall('.//mesh'):mesh.set('filename',local_mesh(mesh.get('filename')).as_uri())
        # Keep all robot inertials, tool/camera collision links and all cup IDs.
        for j in u.findall('joint'):
            if j.find('child').get('link')=='base_link':
                j.find('origin').set('xyz','0 0 0');j.find('origin').set('rpy','0 0 0')
        self.reach_bound=self.model_reach_bound(u)
        self.mass=self.tool_mass_properties(s['tool'],local_mesh)
        link=ET.SubElement(u,'link',name='controlled_tool_mass')
        inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'origin',xyz=' '.join(map(str,self.mass['com_flange_m'])),rpy='0 0 0')
        ET.SubElement(inertial,'mass',value=str(self.mass['mass_kg']));I=np.asarray(self.mass['inertia_com_flange_kg_m2'])
        ET.SubElement(inertial,'inertia',**{k:str(I[a,b]) for k,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
        joint=ET.SubElement(u,'joint',name='controlled_tool_mass_fixed',type='fixed')
        ET.SubElement(joint,'parent',link='link_6');ET.SubElement(joint,'child',link='controlled_tool_mass');ET.SubElement(joint,'origin',xyz='0 0 0',rpy='0 0 0')
        sr=ET.fromstring(old['parameters']['robot_description_semantic'])
        self.removed=0
        for elem in list(sr.findall('disable_collisions')):
            if elem.get('reason') not in {'Adjacent_official_chain','Owned_tool_assembly','Owned_J1_permanent_assembly'}:
                sr.remove(elem);self.removed+=1
        # Only physical fixed mating interfaces. Geometry stays present against
        # every other link and every environmental object.
        mounts=[('link_1',x['attachment_id']) for x in s['scene']['fixed_attachment_interfaces']]
        for row in s['tool']['mounting_contacts']:
            candidates=[x for x in old['tool_links'] if x.endswith(row['tool_collision'])]
            if len(candidates)!=1:raise ValueError('MOUNT_LINK_ID_AMBIGUOUS')
            mounts.append((row['robot_link'],candidates[0]))
        for a,b in mounts:ET.SubElement(sr,'disable_collisions',link1=a,link2=b,reason='Source_bound_fixed_mating_interface')
        params=deepcopy(old['parameters']);params['robot_description']=ET.tostring(u,encoding='unicode');params['robot_description_semantic']=ET.tostring(sr,encoding='unicode')
        official=native['official_limits'];names=old['joint_names']
        # Conservative initial commissioning envelope; below official velocity
        # maxima. Acceleration is a configured engineering limit, not a rating.
        self.limits=Limits(tuple(float(official[n]['lower']) for n in names),tuple(float(official[n]['upper']) for n in names),(.10,)*6,(.20,)*6)
        for n in names:
            for quantity,value in [('velocity',.10),('acceleration',.20)]:
                params[f'robot_description_planning.joint_limits.{n}.max_{quantity}']=value
        from unloading_sim.eco65.native_scene import world_objects
        from unloading_sim.geometry import rotation_matrix_from_rpy
        scene=s['scene'];D_base=np.eye(4);D_base[:3,:3]=rotation_matrix_from_rpy(*scene['base_rpy_rad']);D_base[:3,3]=scene['base_position_m']
        base_D=np.linalg.inv(D_base);world=world_objects(scene)
        for b in scene['boxes']:
            if b['size_m']!=[.15,.1,.1]:raise ValueError('CARTON_DIMENSIONS_CHANGED')
            pose=np.eye(4);pose[:3,3]=b['pose'];world.append(dict(id=b['id'],size=b['size_m'],pose=pose.tolist(),category='carton'))
        for obj in world:obj['pose']=(base_D@np.asarray(obj['pose'])).tolist()
        self.world=world;self.scene_hash=digest(world);self.model_hash=digest(dict(parameters=params,policy=POLICY,mass=self.mass))
        oldid=old['identity'];identity={k:oldid[k] for k in ('robot_id','robot_variant','planning_group','base_link','flange_link','task_tcp_link','flange_from_task_tcp','task_tcp_fingerprint')}
        identity.update(schema='eco65_controlled_empty_model_v1',model_tool_fingerprint=self.model_hash,scene_fingerprint=self.scene_hash,policy_fingerprint=digest(POLICY),policy_scope='CONTROLLED_EMPTY_NO_CONTACT')
        self.init=dict(op='init',parameters=params,identity=identity,joint_names=names,seed=6518,collision_policy=POLICY,tool_links=old['tool_links'],compliant_tool_links=[],process_geometry=None,planning_only=False,planning_mode='cold_from_scratch',expected_collision_shapes=old['expected_collision_shapes'])
        self.clearance=dict(schema='eco65_native_empty_clearance_v1',source_policy=POLICY,stage='residence',tool_links=old['tool_links'],compliant_tool_links=[],conveyor_ids=[x['id'] for x in world if 'conveyor' in x['id']],payload_id='',target_id='',receiver_reserve_m=0.,numerical_gap_tolerance_m=1e-9,edge_resolution_rad=.0005)
        self.track(ROOT/'install/moveit/lib/m710_moveit_backend/m710_moveit_worker')
        for name in ('planner.py','trajectory.py','engine.py','transport.py','service.py','ui.py'):self.track(Path(__file__).parent/name)
        self.ready=True
    @staticmethod
    def model_reach_bound(urdf):
        """Triangle inequality over every ancestor translation and collision vertex.

        Rotation cannot increase these norms. This bounds the distance of every
        moving point from the root for all joint angles; no home-pose assumption.
        """
        import trimesh
        from urllib.parse import unquote,urlparse
        parents={j.find('child').get('link'):(j.find('parent').get('link'),
                 np.linalg.norm(np.fromstring(j.find('origin').get('xyz','0 0 0'),sep=' ')) if j.find('origin') is not None else 0.) for j in urdf.findall('joint')}
        cache={}
        def offset(name):
            if name not in cache:cache[name]=offset(parents[name][0])+parents[name][1] if name in parents else 0.
            return cache[name]
        reach=0.
        for link in urdf.findall('link'):
            for c in link.findall('collision'):
                origin=c.find('origin');translation=0. if origin is None else np.linalg.norm(np.fromstring(origin.get('xyz','0 0 0'),sep=' '))
                g=c.find('geometry');m=g.find('mesh')
                if m is not None:
                    mesh=trimesh.load(unquote(urlparse(m.get('filename')).path),force='mesh',process=False)
                    vertices=np.asarray(mesh.vertices)*np.fromstring(m.get('scale','1 1 1'),sep=' ')
                    radius=float(np.linalg.norm(vertices,axis=1).max())
                elif g.find('box') is not None:radius=np.linalg.norm(np.fromstring(g.find('box').get('size'),sep=' '))/2
                elif g.find('cylinder') is not None:
                    v=g.find('cylinder');radius=np.hypot(float(v.get('radius')),float(v.get('length'))/2)
                elif g.find('sphere') is not None:radius=float(g.find('sphere').get('radius'))
                else:raise ValueError('UNSUPPORTED_COLLISION_GEOMETRY')
                reach=max(reach,offset(link.get('name'))+translation+radius)
        if not 0<reach<=10:raise ValueError('MODEL_REACH_UNBOUNDED')
        return float(reach)
    def tool_mass_properties(self,tool,local_mesh):
        # CAD solid volumes and centroids are exact STEP-derived quantities.
        # Internal materials are unknown: distribute the confirmed total mass
        # by volume; use each solid's bounding cuboid for finite conservative
        # inertia estimates. This estimate is not a measured inertia tensor.
        rows=tool['visual_parts'];volumes=np.array([r['geometric_volume_m3'] for r in rows])
        centers=np.array([r['geometric_volume_centroid_m'] for r in rows])
        for row in rows:local_mesh(row['visual'])
        if np.any(volumes<=0):raise ValueError('INVALID_SOLID_VOLUME')
        mass=self.config['tool_mass_kg'];weights=mass*volumes/volumes.sum()
        com=np.average(centers,axis=0,weights=weights);inertia=np.zeros((3,3))
        for row,weight,center in zip(rows,weights,centers):
            size=np.diff(np.array(row['bounds_m']),axis=0)[0]
            d=center-com
            inertia+=weight*(np.diag([(size[1]**2+size[2]**2)/12,(size[0]**2+size[2]**2)/12,(size[0]**2+size[1]**2)/12])+np.dot(d,d)*np.eye(3)-np.outer(d,d))
        T=np.asarray(tool['T_flange_tool_cad']);com=T[:3,:3]@com+T[:3,3];inertia=T[:3,:3]@inertia@T[:3,:3].T
        if np.min(np.linalg.eigvalsh(inertia))<=0:raise ValueError('INVALID_TOOL_INERTIA')
        return dict(mass_kg=mass,com_flange_m=com.tolist(),inertia_com_flange_kg_m2=inertia.tolist(),mass_source='USER_CONFIRMED_1_5_KG',distribution_source='CAD_VOLUME_WEIGHTED_CENTROIDS_AND_BOUNDING_CUBOID_INERTIA_ESTIMATE_NOT_MEASURED',robot_rated_payload_kg=5.)
    def request(self,q,**kw):
        return dict(identity=self.init['identity'],q_start=list(q),start_velocity=[0.]*6,path_constraints=[],scene_fingerprint=self.scene_hash,world=self.world,allowed_pairs=[['base_link','robot_mount_plate']],attachment=None,clearance_policy=self.clearance,stage='residence',planning_only=False,seed=6518,**kw)
    def plan(self,sample,goal,time_scale=10.,progress=None):
        progress=progress or (lambda message:None)
        q,goal=validate_live_goal(sample,goal)
        progress('核对模型、工具、场景及版本')
        self.build();binding=self.binding();self.authority.invalidate()
        from unloading_sim.moveit2_backend import ResidentMoveItClient
        directory=Path(self.config['record_root'])/('plan-'+uuid.uuid4().hex);directory.mkdir(parents=True,exist_ok=True)
        if self.client:self.client.close()
        self.client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(directory),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=directory/'moveit.log')
        try:
            progress('启动原生规划器并导入完整场景')
            self.client.request(self.init)
            progress('生成 MoveIt / Pilz 关节路径')
            result=self.client.request(self.request(q,op='plan',q_goal=goal.tolist(),pipeline_id='pilz_industrial_motion_planner',planner_id='PTP',velocity_scale=1.,acceleration_scale=1.,allowed_planning_time_s=20.),timeout=120)
            (directory/'native-plan.json').write_text(json.dumps(result,indent=2))
            if result['status']!='SUCCESS' or result.get('native_output_status')!='PASS':raise ValueError('NATIVE_PLAN_REJECTED:'+str(result.get('failure',result['status'])))
            if len(result.get('points',[]))<2:
                raise ValueError('原生规划器未生成可运动的时间轨迹（少于两个时间点）。请检查目标后重新规划；当前轨迹不可执行。')
            prepared=prepare(result['points'],self.limits,time_scale=time_scale,period=self.config['period_s'],reach_bound_m=self.reach_bound)
            progress(f"检查最终发送路径碰撞：{len(prepared['points'])} 个发送点，{len(prepared['collision_path'])} 个路径点；计划运动 {prepared['send_duration_s']:.3f} s")
            checked=self.client.request(self.request(q,op='validate',probe_path=prepared['collision_path']),timeout=600)
            (directory/'sent-path-check.json').write_text(json.dumps(checked,indent=2))
            if not checked.get('native_valid') or not checked.get('legacy_intersection_valid') or checked.get('path_failure') is not None:raise ValueError('SENT_PATH_COLLISION_REJECTED:'+str(checked.get('path_failure',checked.get('failure'))))
            evidence=dict(native_plan='PASS',sent_path_collision='PASS',native_result=result,collision_result=checked,start_sample=sample,removed_simulation_exemptions=self.removed,planning_record=str(directory),hardware_execution='NOT_RUN')
            package=self.authority.register(prepared,binding,evidence)
            (directory/'preview-NOT-IMPORTABLE.json').write_text(json.dumps(package))
            return package
        finally:self.client.close();self.client=None
    def close(self):
        if self.client:self.client.close();self.client=None
