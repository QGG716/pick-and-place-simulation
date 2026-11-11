"""Full physical single-carton native MTC/Pilz/OMPL plan before any actuation."""
from copy import deepcopy
from collections import Counter
from pathlib import Path
import json
import time
import numpy as np
from ..controlled.planner import EmptyPlanner,ROOT
from ..controlled.trajectory import digest,prepare
from ..coordinates import rigid
from .geometry import corners,contact_candidates,candidate_contact_check,transform


class SinglePlanner(EmptyPlanner):
    def __init__(self,config,mapping,authority,coordinate_model):
        self.site=config;self.coordinate_model=coordinate_model;self.cycle_binding=None
        super().__init__(config['execution'],mapping,authority)
    def build(self):
        if self.ready:return
        super().build()
        self.ready=False
        self.track(self.site['config_path']);self.track(self.site['camera_extrinsics'])
        for directory in ('dino_directory','sam_directory'):
            for path in Path(self.site['detector'][directory]).iterdir():
                if path.is_file():self.track(path)
        for path in Path(__file__).parent.glob('*.py'):self.track(path)
        g=self.source['native_model']['init']['process_geometry'];top=self.site['topology']
        self.contact_geometry={k:deepcopy(g[k]) for k in ('contours','flange_from_contact_frame','terminal_band_by_contour')}
        self.contact_geometry.update(normal_cad=self.source['tool']['seals'][0]['normal_cad'],geometry_type='physical_common_valve_cad_seals_v1',connected_cup_ids=top['connected_cup_ids'],topology=top)
        self.init['process_geometry']=self.contact_geometry
        self.model_hash=digest(dict(parameters=self.init['parameters'],policy=self.init['collision_policy'],
                                    tool_mass=self.mass,physical_contact_geometry=self.contact_geometry))
        self.init['identity'].update(schema='eco65_physical_single_carton_model_v1',policy_scope='PHYSICAL_SINGLE_CARTON')
        self.init['identity']['model_tool_fingerprint']=self.model_hash
        self.init['compliant_tool_links']=[] # dedicated, source-bound contact gate below
        self.clearance['schema']='eco65_native_single_carton_clearance_v1'
        self.ready=True
    def binding(self):
        b=super().binding();b.update(physical_cycle=self.cycle_binding,camera_extrinsics=self.files.get(self.site['camera_extrinsics']),
            optical_extrinsics_use='WORLD_BASE_BASE_LINK1_Q_CAPTURE_LINK1_CAMERA',carton_mass_kg=self.site['carton_mass_kg'])
        return b
    def receiver(self):
        from unloading_sim.geometry import rotation_matrix_from_rpy
        scene=self.source['scene'];s=scene['native_task']['receiving']
        B=np.eye(4);B[:3,:3]=rotation_matrix_from_rpy(*scene['base_rpy_rad']);B[:3,3]=scene['base_position_m']
        lo,hi=np.asarray(s['bounds_D_m']);vertices=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
        vertices=transform(np.linalg.inv(B),vertices)
        return dict(ids=s['support_ids'],lower=vertices.min(0).tolist(),upper=vertices.max(0).tolist(),margin=s['margin_m'],source='BOUND_CURRENT_SCENE_RECEIVING_REGION')
    def release_check(self,q_start,flange_box,box,world):
        flange=self.coordinate_model.frames(q_start)['link_6']
        pose=self.coordinate_model.world_from_base@flange@flange_box
        vertices=corners(pose,box['size_m']);region=self.receiver();lo=np.asarray(region['lower']);hi=np.asarray(region['upper']);margin=region['margin']
        if self.site.get('release_policy')=='USER_DEFERRED_RECEIVER_QUALIFICATION':
            return dict(status='RECEIVER_QUALIFICATION_DEFERRED_BY_USER',pose=pose.tolist(),corners_m=vertices.tolist(),
                        region=dict(ids=[]),q_return=list(q_start),support_confirmed=False,
                        basis='SAME_LOCKED_Q_START_COLLISION_CHECKED_RECEIVING_NOT_QUALIFIED')
        if np.any(vertices[:,:2].min(0)<lo[:2]+margin) or np.any(vertices[:,:2].max(0)>hi[:2]-margin):
            raise ValueError('Q_START_LOADED_BOX_OUTSIDE_CONFIRMED_RECEIVING_REGION')
        drop=float(vertices[:,2].min()-hi[2])
        if not -.0004<=drop<=.005:raise ValueError('Q_START_RELEASE_NOT_SUPPORTED_MAX_5_MM_GAP')
        return dict(status='PASS',pose=pose.tolist(),corners_m=vertices.tolist(),region=region,drop_m=drop,
                    q_return=list(q_start),basis='SAME_LOCKED_Q_START_NO_SUBSTITUTE_PLACEMENT')
    @staticmethod
    def object_top(obj):
        T=np.asarray(obj['pose'])
        if 'convex_mesh' in obj:return float(transform(T,obj['convex_mesh']['vertices'])[:,2].max())
        return float(T[2,3]+abs(T[2,:3])@(np.asarray(obj['size'])/2))
    def context(self,q,world,box,candidate,stage,attachment=None,supports=()):
        loaded=attachment is not None;contact=stage=='contact'
        contact_ids=candidate['target_lip_contact_ids']
        lips=[self.contact_geometry['terminal_band_by_contour'][i] for i in contact_ids]
        pairs=[['base_link','robot_mount_plate']]
        if contact or loaded:pairs += [[box['id'],lip] for lip in lips]
        pairs += [[box['id'],s['id']] for s in supports]
        attached=None if not loaded else dict(id=box['id'],size=box['size_m'],pose=attachment.tolist(),touch_links=lips)
        policy=dict(self.clearance,stage=stage,target_id=box['id'],payload_id=box['id'] if loaded else '')
        spec=dict(geometry=self.contact_geometry,target_id=box['id'],size_m=box['size_m'],target_pose=box['pose'],
                  face=candidate['face'],active_ids=candidate['active_ids'],target_lip_contact_ids=contact_ids,
                  allow_contact=contact,supports=list(supports))
        return dict(identity=self.init['identity'],q_start=list(q),start_velocity=[0.]*6,path_constraints=[],
            scene_fingerprint=digest(world),world=world,allowed_pairs=pairs,attachment=attached,clearance_policy=policy,
            single_carton_contact=spec,stage=stage,planning_only=False,seed=6518)
    def native_call(self,request,folder,name,timeout,cancelled=None):
        # Persist before dispatch: a timed-out worker cannot return its request
        # or profiling data. Keep every Pilz/OMPL attempt instead of overwriting.
        request=dict(request,request_budget_s=timeout-10.)
        path=folder/(name+'-native.json');started=time.monotonic()
        record=dict(request=request,status='PENDING',ipc_timeout_s=timeout)
        path.write_text(json.dumps(record))
        try:
            result=self.client.request(request,timeout=timeout,cancelled=cancelled)
            record.update(status='REPLIED',result=result,elapsed_s=time.monotonic()-started)
            return result
        except Exception as error:
            record.update(status='REQUEST_FAILED',error=str(error),elapsed_s=time.monotonic()-started)
            raise RuntimeError(f'{name}:{error}；详情：{path.name}') from error
        finally:path.write_text(json.dumps(record))
    def checked_segment(self,context,goal,stage,scale,folder,progress,linear=False,cancelled=None):
        progress('原生规划 '+stage)
        req=dict(context,op='plan',pipeline_id='pilz_industrial_motion_planner',planner_id='LIN' if linear else 'PTP',
                 velocity_scale=1.,acceleration_scale=1.,allowed_planning_time_s=25.)
        if np.asarray(goal).shape==(4,4):req.update(goal_pose=np.asarray(goal).tolist(),flange_from_task_tcp=self.init['identity']['flange_from_task_tcp'])
        else:req['q_goal']=list(goal)
        result=self.native_call(req,folder,stage+'-pilz',120,cancelled)
        if result.get('status')!='SUCCESS' and not linear:
            # A new native search, with the identical loaded scene, never reverse playback.
            req.update(pipeline_id='ompl',planner_id='RRTConnectkConfigDefault')
            progress(f'{stage} 直连未通过（{result.get("status")}），正在搜索避让路径（25 秒预算）')
            result=self.native_call(req,folder,stage+'-ompl',120,cancelled)
        (folder/(stage+'-native.json')).write_text(json.dumps(dict(request=req,result=result)))
        if result.get('status')!='SUCCESS' or result.get('native_output_status')!='PASS':
            raise ValueError(stage+':NATIVE_REJECTED:'+str(result.get('failure',result.get('status'))))
        payload_reach=0.
        if context.get('attachment'):
            a=context['attachment']
            payload_reach=float(np.linalg.norm(corners(a['pose'],a['size']),axis=1).max())
        prepared=prepare(result['points'],self.limits,scale,self.config['period_s'],self.reach_bound+payload_reach)
        progress(f"检查 {stage} 最终发送轨迹：{len(prepared['points'])} 点")
        checked=self.native_call(dict(context,op='validate',probe_path=prepared['collision_path']),folder,stage+'-sent',600,cancelled)
        (folder/(stage+'-sent-check.json')).write_text(json.dumps(checked))
        if not checked.get('native_valid') or not checked.get('legacy_intersection_valid') or checked.get('path_failure') is not None:
            raise ValueError(stage+':SENT_PATH_REJECTED:'+str(checked.get('path_failure',checked.get('failure'))))
        return dict(stage=stage,prepared=prepared,evidence=dict(native_plan='PASS',sent_path_collision='PASS',
                    native_result=result,collision_result=checked,context=context,hardware_execution='NOT_RUN'))
    def plan_cycle(self,observation,boxes,folder,progress=lambda _:None,selected=None,cancelled=lambda:False):
        from ..contracts import Source,HOST_SESSION
        if (observation.frame.stamp.source!=Source.LIVE or observation.frame.stamp.host_session!=HOST_SESSION or
                observation.robot_sample.get('source')!='LIVE'):
            raise ValueError('PHYSICAL_PLANNER_REQUIRES_CURRENT_SESSION_LIVE_CAPTURE')
        self.build();self.authority.invalidate();self.cycle_binding=None
        world=[deepcopy(x) for x in self.world if x.get('category')!='carton']
        world += [dict(id=b['id'],size=b['size_m'],pose=b['pose'],category='carton') for b in boxes]
        q=np.array(observation.q_start);failures=[]
        from unloading_sim.moveit2_backend import ResidentMoveItClient
        self.client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(folder),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=folder/'moveit.log')
        try:
            ready=self.client.request(self.init)
            if not ready.get('capabilities',{}).get('bounded_single_carton_planning'):
                raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_BOUNDED_SINGLE_CARTON_PLANNING')
            for box in boxes:
                if selected is not None and box['detection_index']!=selected:continue
                try: candidates=contact_candidates(box,self.source['tool'],self.site['topology'])
                except ValueError as e:failures.append(dict(target=box['id'],reason=str(e)));continue
                for i,candidate in enumerate(candidates):
                    if cancelled():raise ValueError('CANCELLED')
                    sub=folder/(box['id']+'-'+str(i));sub.mkdir()
                    progress(f'候选 {i+1}/{len(candidates)}：检查带载返回起点')
                    try:
                        tcp=rigid(candidate['tcp_pose']);flange=tcp@np.linalg.inv(self.coordinate_model.tool.matrix)
                        attachment=np.linalg.inv(flange)@rigid(box['pose'])
                        release=self.release_check(q,attachment,box,world)
                        # Full loaded endpoint is checked before any approach planning.
                        release_supports=[dict(id=o['id'],top_world_m=self.object_top(o)) for o in world if o['id'] in release['region']['ids']]
                        start_loaded=self.context(q,world,box,candidate,'return',attachment,release_supports)
                        endcheck=self.native_call(dict(start_loaded,op='validate'),sub,'loaded-start',120,cancelled)
                        (sub/'loaded-start-check.json').write_text(json.dumps(dict(request=start_loaded,result=endcheck)))
                        if not endcheck.get('native_valid') or not endcheck.get('legacy_intersection_valid'):raise ValueError('LOADED_Q_START_COLLISION:'+str(endcheck.get('failure')))
                        approach=self.checked_segment(self.context(q,world,box,candidate,'approach'),candidate['precontact_pose'],'approach',self.site['time_scale'],sub,progress,cancelled=cancelled)
                        qpre=approach['prepared']['points'][-1]['q']
                        contact=self.checked_segment(self.context(qpre,world,box,candidate,'contact'),tcp,'contact',self.site['time_scale'],sub,progress,True,cancelled)
                        qc=contact['prepared']['points'][-1]['q'];actual=self.coordinate_model.tcp(qc)
                        seal=candidate_contact_check(actual,candidate)
                        if not seal['valid']:raise ValueError('ACTUAL_NATIVE_CONTACT_FK_SEAL_REJECTED')
                        actual_flange=actual@np.linalg.inv(self.coordinate_model.tool.matrix)
                        attachment=np.linalg.inv(actual_flange)@rigid(box['pose'])
                        release=self.release_check(q,attachment,box,world)
                        bottom=min(np.asarray(box['corners_m'])[:,2]);original=[]
                        for o in world:
                            if o['id']==box['id']:continue
                            top=self.object_top(o)
                            if abs(top-bottom)<=.0004:original.append(dict(id=o['id'],top_world_m=top))
                        # Outward separation plus upward support clearance, sized from the
                        # actual observed box. All samples keep full attached box geometry.
                        outward=np.asarray(candidate['face']['normal']);detach=actual.copy()
                        detach[:3,3]+=outward*(max(box['size_m'])+.01)
                        detach[2,3]+=max(0.,.01-(detach[2,3]-actual[2,3]))
                        extraction=self.checked_segment(self.context(qc,world,box,candidate,'detach',attachment,original),detach,'detach',self.site['time_scale'],sub,progress,True,cancelled)
                        qe=extraction['prepared']['points'][-1]['q']
                        returning=self.checked_segment(self.context(qe,world,box,candidate,'return',attachment,release_supports),q,'return',self.site['time_scale'],sub,progress,cancelled=cancelled)
                        if np.max(abs(np.asarray(returning['prepared']['points'][-1]['q'])-q))>1e-7:raise ValueError('RETURN_NOT_LOCKED_Q_START')
                        self.cycle_binding=dict(task_id=folder.name,observation=observation.metadata(),target=box,world=digest(world),candidate=candidate,
                            attachment=attachment.tolist(),release=release,expected_attachment='PLANNED_NOT_PHYSICALLY_CONFIRMED')
                        size=np.array(box['size_m']);mass=self.site['carton_mass_kg']
                        self.cycle_binding['payload_mass_properties']=dict(mass_kg=mass,com_box_m=[0.,0.,0.],
                            inertia_box_kg_m2=(mass/12*np.diag([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])).tolist(),
                            basis='USER_CONFIRMED_2_KG_UNIFORM_CUBOID_INERTIA_ESTIMATE')
                        binding=self.binding();segments=[approach,contact,extraction,returning]
                        for segment in segments:segment['package']=self.authority.register(segment.pop('prepared'),binding,segment.pop('evidence'))
                        plan=dict(binding=binding,segments=segments,target=box,candidate=candidate,release=release,
                            q_start=q.tolist(),T_world_tcp_start=observation.world_tcp_start,full_path_validated=True,
                            duration_s=sum(s['package']['prepared']['send_duration_s'] for s in segments),hardware_execution='NOT_RUN')
                        (folder/'complete-plan.json').write_text(json.dumps(plan));return plan
                    except ValueError as e:failures.append(dict(target=box['id'],candidate=i,reason=str(e)))
            # Keep every concrete failure on disk; one task-status label must
            # not grow into 32 copies of the full native collision JSON.
            (folder/'planning-failures.json').write_text(json.dumps(failures,ensure_ascii=False,indent=2))
            counts=Counter(f['reason'].split(':',1)[0] for f in failures)
            summary='；'.join(f'{reason} × {count}' for reason,count in counts.items())
            first=failures[0]['reason'][:600] if failures else '没有符合所选编号的目标'
            raise ValueError(f'NO_COMPLETE_FEASIBLE_SINGLE_CARTON_PLAN: {summary}；首项：{first}；详情：planning-failures.json')
        finally:self.client.close();self.client=None
