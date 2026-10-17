"""Bound native probe adapter for one look/touch experiment, no Python planner."""
import copy,itertools,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from unloading_sim.eco65.model import ROOT,load_json,save_json,transform,robot_for
from unloading_sim.eco65.native_model import POLICY
from unloading_sim.eco65.native_io import file_hash
from unloading_sim.moveit2_backend import ResidentMoveItClient,digest,_rpy
from eco65_touch_geometry import StaticGeometry
from eco65_touch_sources import verify_scene_sources


def build(c,out,point_id='p2',native_asset_root='/work-eco65-20261015'):
    verify_scene_sources(c);out=Path(out).resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    s=load_json(ROOT/c['snapshot']);g=StaticGeometry(s,c,out);robot=g.robot
    P=np.array(next(p['T_D_P'] for p in c['placements'] if p['split']=='validation'))
    old=ET.fromstring(s['native_model']['init']['parameters']['robot_description']);urdf=ET.Element('robot',name='eco65_known_tip_test')
    # Preserve the official six-joint chain and the original suction TCP.
    for el in old:
        if (el.tag=='link' and el.get('name') in ['world','base_link',*[f'link_{i}' for i in range(1,7)],'eco65_task_tcp'] or
            el.tag=='joint' and (el.get('name') in robot.active_joint_names or el.find('child').get('link') in ['base_link','eco65_task_tcp'])):
            new=copy.deepcopy(el)
            for col in list(new.findall('collision')):new.remove(col)
            urdf.append(new)
    def fixed(name,parent,T):
        link=ET.SubElement(urdf,'link',name=name);j=ET.SubElement(urdf,'joint',name='touch_mount_'+name,type='fixed')
        ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=name)
        ET.SubElement(j,'origin',xyz=' '.join(map(str,T[:3,3])),rpy=' '.join(map(str,_rpy(T[:3,:3]))));return link
    fixed('calibration_board_frame','world',P)
    Ftip=transform(c['probe']['p_F_tip_m']);fixed('calibration_probe_tcp','link_6',Ftip)
    srdf=ET.Element('robot',name=urdf.get('name'));group=ET.SubElement(srdf,'group',name='eco65_arm')
    ET.SubElement(group,'chain',base_link='base_link',tip_link='calibration_probe_tcp')
    world=[];names={};bindings=[];moving=[];meshdir=out/'native_meshes';meshdir.mkdir()
    for i,e in enumerate(g.entries):
        hull=trimesh.convex.convex_hull(e['vertices']);name=e['owner']
        if e['kind']=='environment' and e['source_label']!=e['owner']:name+='__component_'+str(i)
        names[e['name']]=name
        if e['frame'] in ['D','board']:
            T=(P if e['frame']=='board' else np.eye(4))@e['T']
            world.append(dict(id=name,pose=T.tolist(),category='fixed_fixture' if e['frame']=='board' else 'environment',
                convex_mesh=dict(vertices=hull.vertices.tolist(),triangles=hull.faces.tolist())))
        else:
            moving.append(e)
            path=meshdir/(name+'.obj');hull.export(path)
            link=urdf.find(f"link[@name='{name}']") if e['kind']=='robot' else fixed(name,e['frame'],e['T'])
            col=ET.SubElement(link,'collision')
            if e['kind']=='robot':ET.SubElement(col,'origin',xyz=' '.join(map(str,e['T'][:3,3])),rpy=' '.join(map(str,_rpy(e['T'][:3,:3]))))
            rel=path.relative_to(ROOT).as_posix()
            ET.SubElement(ET.SubElement(col,'geometry'),'mesh',filename='file://'+native_asset_root+'/'+rel)
            bindings.append(dict(path=rel,sha256=file_hash(path),owner=name))
    pairs={frozenset([a['name'],b['name']]) for a,b,_ in g.pairs}
    for a,b in itertools.combinations(moving,2):
        if frozenset([a['name'],b['name']]) not in pairs:
            ET.SubElement(srdf,'disable_collisions',link1=names[a['name']],link2=names[b['name']],reason='Bound_static_geometry_rigid_or_named_installation_interface')
    mounts=[dict(pair=[names[a['name']],names[b['name']]],threshold_m=rule['threshold_m'],source=rule['source'])
            for a,b,rule in g.pairs if rule['threshold_m']<0]
    datum=next(p for p in c['board']['touch_points'] if p['point_id']==point_id)
    process=dict(geometry_type='known_tip_finite_datum_v1',p_F_tip_m=c['probe']['p_F_tip_m'],p_P_datum_m=datum['p_P_m'],
        board_frame_link='calibration_board_frame',target_id='stud_'+point_id,datum_radius_m=c['board']['stud_radius_m'],
        maximum_tip_penetration_m=.0001,installation_contacts=mounts,derived_scene_identity=c['derived_scene_identity'])
    init=copy.deepcopy(s['native_model']['init']);params=init['parameters'];params['robot_description']=ET.tostring(urdf,encoding='unicode')
    params['robot_description_semantic']=ET.tostring(srdf,encoding='unicode')
    init.update(process_geometry=process,tool_links=sorted(names[e['name']] for e in moving if e['kind']!='robot'),compliant_tool_links=['probe_tip'],
                expected_collision_shapes={l.get('name'):len(l.findall('collision')) for l in urdf.findall('link') if l.findall('collision')})
    identity=init['identity'];identity.update(model_tool_fingerprint=digest(params),scene_fingerprint=digest(world),task_tcp_link='calibration_probe_tcp',
        flange_from_task_tcp=Ftip.tolist(),task_tcp_fingerprint=digest(Ftip.tolist()),contact_geometry_fingerprint=digest(process),
        original_suction_tcp=c['probe']['original_T_flange_tcp'],derived_scene_identity=c['derived_scene_identity'])
    config=dict(init=init,world=world,P=P.tolist(),point_id=point_id,bindings=bindings,
        source_configuration_sha256=digest(c),scope='ONE_KNOWN_TIP_SIMULATION_TEST',
        legacy_compliant_tool_links_field='Names the scoped probe contact body only; no flexible/compliant response assumption',
        static_fixed_pair_policy='Named support/base/table contact checked separately; ordinary fixed environment internal pairs do not move')
    save_json(out/'native_context.json',config)
    (out/'probe_robot.urdf').write_text(params['robot_description'],encoding='utf8')
    return config,g


def nearest_equivalent(q,seed,limits):
    """Revolute winding only, never cross official limits or change FK."""
    result=np.asarray(q,dtype=float).copy()
    for i,(lo,hi) in enumerate(limits):
        options=[result[i]+2*np.pi*k for k in range(-2,3) if lo<=result[i]+2*np.pi*k<=hi]
        if not options:raise ValueError('IK_OUTSIDE_OFFICIAL_LIMITS')
        result[i]=min(options,key=lambda v:abs(v-seed[i]))
    return result


class NativeTouch:
    def __init__(self,context,c,geometry,out,worker):
        self.ctx=context;self.c=c;self.g=geometry;self.out=Path(out);self.records=[];self.counter=0
        self.client=ResidentMoveItClient(worker,log_path=self.out/'worker.log');self.identity=context['init']['identity']
        if self.call(context['init']).get('status')!='READY':raise ValueError('TOUCH_NATIVE_INIT_FAILED')
    def call(self,request):
        start=time.perf_counter();row=dict(request=copy.deepcopy(request));self.records.append(row)
        try:row['result']=self.client.request(request,timeout=180 if request.get('op')=='plan' else 40);return row['result']
        except Exception as error:row['error']=str(error);raise
        finally:
            row['seconds']=time.perf_counter()-start
            with (self.out/'native_calls.jsonl').open('a',encoding='utf8') as f:
                import json
                f.write(json.dumps(row,allow_nan=False)+'\n')
    def request(self,q,stage):
        init=self.ctx['init'];self.counter+=1
        return dict(op='check_state_offline',identity=self.identity,q_start=np.asarray(q).tolist(),world=self.ctx['world'],scene_fingerprint=digest(self.ctx['world']),
            attachment=None,stage=stage,seed=6518+self.counter,start_velocity=[0.]*6,path_constraints={},allowed_pairs=[],
            process_policy=dict(init['process_geometry'],schema='eco65_known_tip_touch_process_v1',stage=stage,initial_proximity=None,collision_policy=POLICY),
            clearance_policy=dict(schema='eco65_native_process_clearance_v1',source_policy=POLICY,stage=stage,tool_links=init['tool_links'],
                compliant_tool_links=init['compliant_tool_links'],target_id=init['process_geometry']['target_id'],payload_id='',conveyor_ids=[],
                receiver_reserve_m=0.,numerical_gap_tolerance_m=1e-9,edge_resolution_rad=.002),
            clearance_mode='optimized',cancelled=False,flange_from_task_tcp=self.identity['flange_from_task_tcp'],planning_only=True,
            planning_mode='cold_from_scratch',history_enabled=False,velocity_scale=.15,acceleration_scale=.15,record_contact_distances=True)
    def check(self,q,stage='pregrasp'):
        result=self.call(self.request(q,stage));guard=stage in ['contact','withdrawal']
        independent=self.g.check(q,np.asarray(self.ctx['P']),self.ctx['point_id'] if guard else None,contact_neighborhood=guard)
        return dict(accepted=result.get('status')=='SUCCESS' and independent['accepted'],native=result,independent=independent)
    def ik(self,pose,seed,stage,count=16):
        rng=np.random.default_rng(651700+self.counter);found=[]
        # Keep a small fixed budget, but first try local/J1-facing seeds.
        # KDL's equivalent +/-2pi answers must not cause a full wrist sweep.
        seed=np.asarray(seed);limits=self.g.robot.joint_limits
        seeds=[seed.copy()]
        for yaw in [-np.pi/2,np.pi/2,np.pi]:
            value=seed.copy();value[0]+=yaw
            value=nearest_equivalent(value,seed,limits);seeds.append(value)
        for previous in list(seeds):
            value=previous.copy();value[3]+=np.pi;value[4]*=-1;value[5]+=np.pi
            seeds.append(nearest_equivalent(value,seed,limits))
        seeds+=list(rng.uniform(limits[:,0],limits[:,1],(max(0,count-len(seeds)),6)))
        for q in seeds[:count]:
            r=self.call(dict(op='ik',identity=self.identity,q_start=q.tolist(),goal_pose=np.asarray(pose).tolist(),seed=6518,
                planning_only=True,planning_mode='cold_from_scratch',position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4))
            if r.get('status')!='SUCCESS':continue
            if r.get('native_ik_calls')!=1:raise ValueError('NATIVE_IK_PROVENANCE_REQUIRED')
            answer=nearest_equivalent(np.asarray(r['q']),seed,limits)
            if any(np.max(abs(answer-v))<1e-5 for v in found):continue
            if self.check(answer,stage)['accepted']:found.append(answer)
        return sorted(found,key=lambda x:np.linalg.norm(x-seed))
    def motion(self,q,stage,*,goal=None,pose=None):
        solvers=[('pilz_industrial_motion_planner','LIN' if pose is not None else 'PTP')]
        if pose is None:solvers.append(('ompl','RRTConnectkConfigDefault'))
        for pipeline,planner in solvers:
            req=self.request(q,stage);req.update(op='plan',task_id='touch_actual_'+str(self.counter),parent_stage_id='',stage_id='stage_'+str(self.counter),
                pipeline_id=pipeline,planner_id=planner,require_native_motion=True,allowed_planning_time_s=8.,record_contact_distances=False)
            req['goal_pose' if pose is not None else 'q_goal']=np.asarray(pose if pose is not None else goal).tolist()
            r=self.call(req)
            self.call(dict(op='retire_touch_task',identity=self.identity,task_id=req['task_id']))
            if r.get('status')!='SUCCESS':continue
            if not(r.get('mtc_generation') and r.get('generated_during_task') and r.get('native_solver_calls')):raise ValueError('NATIVE_GENERATION_REQUIRED')
            path=np.array([p['q'] for p in r['points']]);times=np.array([p['t'] for p in r['points']])
            guard=stage in ['contact','withdrawal'];checks=[]
            for a,b in zip(path[:-1],path[1:]):
                count=max(1,int(np.ceil(np.max(abs(b-a))/.005)))
                for u in np.linspace(0,1,count+1):
                    value=a+u*(b-a);check=self.g.check(value,np.asarray(self.ctx['P']),self.ctx['point_id'] if guard else None,contact_neighborhood=guard)
                    checks.append(dict(q_rad=value.tolist(),accepted=check['accepted'],failures=check['failures']))
                    if not check['accepted']:
                        save_json(self.out/('path_rejected_'+str(self.counter)+'.json'),checks);break
                if not checks[-1]['accepted']:break
            if not checks or not all(x['accepted'] for x in checks):continue
            if pose is not None:
                A=self.g.robot.named_link_frames(q)['link_6']@np.array(self.identity['flange_from_task_tcp']);v=np.array(pose)[:3,3]-A[:3,3]
                for point in path:
                    T=self.g.robot.named_link_frames(point)['link_6']@np.array(self.identity['flange_from_task_tcp'])
                    fraction=0 if v@v<1e-16 else np.clip((T[:3,3]-A[:3,3])@v/(v@v),0,1)
                    if np.linalg.norm(T[:3,3]-(A[:3,3]+fraction*v))>.0001:raise ValueError('PROBE_TIP_LIN_NOT_STRAIGHT')
            output=dict(q=path.tolist(),times=times.tolist(),stage=stage,native_request=req,native_result=r,independent_samples=checks)
            save_json(self.out/('motion_'+str(self.counter)+'.json'),output);return output
        raise ValueError('BOUNDED_NATIVE_TOUCH_MOTION_FAILED:'+stage)
    def close(self):self.client.close()
