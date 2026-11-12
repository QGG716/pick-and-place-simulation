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
from .config import release_policy_evidence

# Successively slower Pilz LIN profiles after a full-scale failure.
LIN_RETRY_SCALES=(.5,.25,.1)
# Minimal send slowdowns when interpolation would exceed a joint limit.
SEND_SLOWDOWN_STEPS=(1.,1.1,1.25,1.5,2.)
# Detach profiles after straight outward separation: (name, outward m, lift m).
DETACH_ALTERNATIVES=(('DIAGONAL_45',.04,.04),('LIFT_FIRST',.01,.05))

class SinglePlanner(EmptyPlanner):
    def __init__(self,config,mapping,authority,coordinate_model):
        self.site=config;self.coordinate_model=coordinate_model;self.cycle_binding=None
        super().__init__(config['execution'],mapping,authority)
    def build_world(self,s):
        # Source snapshot supplies robot/tool/camera solids only. Its trailer,
        # base placement, conveyors, floor, lift and cartons never enter real IO.
        return []
    def build(self):
        if self.ready:return
        for path,expected in self.site.get('loaded_file_sha256',{}).items():self.track(path,expected)
        super().build()
        self.ready=False
        self.track(self.site['config_path']);self.track(self.site['camera_extrinsics'])
        if self.site.get('site_scene'):self.track(self.site['site_scene'])
        if self.site.get('device_config') and Path(self.site['device_config']).is_file():self.track(self.site['device_config'])
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
        if self.site.get('site'):
            s=self.site['site'].get('receiver')
            if not s:raise ValueError('SITE_RECEIVER_NOT_CONFIGURED: 请填写 q_start 承接面实测范围及支撑物 ID')
            return dict(ids=s['support_ids'],lower=s['bounds_m'][0],upper=s['bounds_m'][1],
                        margin=s.get('margin_m',.003),source=s['measurement_basis'])
        from unloading_sim.geometry import rotation_matrix_from_rpy
        scene=self.source['scene'];s=scene['native_task']['receiving']
        B=np.eye(4);B[:3,:3]=rotation_matrix_from_rpy(*scene['base_rpy_rad']);B[:3,3]=scene['base_position_m']
        lo,hi=np.asarray(s['bounds_D_m']);vertices=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
        vertices=transform(np.linalg.inv(B),vertices)
        return dict(ids=s['support_ids'],lower=vertices.min(0).tolist(),upper=vertices.max(0).tolist(),margin=s['margin_m'],source='BOUND_CURRENT_SCENE_RECEIVING_REGION')
    def release_check(self,q_start,flange_box,box,world):
        qualification=release_policy_evidence(self.site)
        flange=self.coordinate_model.frames(q_start)['link_6']
        pose=self.coordinate_model.world_from_base@flange@flange_box
        vertices=corners(pose,box['size_m'])
        if qualification['qualification_deferred']:
            # This defers support qualification only. Empty support IDs grant
            # no receiver contact exemption; native loaded endpoint and all
            # four sent-path checks still use the complete unchanged world.
            return dict(status='RECEIVER_QUALIFICATION_DEFERRED_BY_USER',pose=pose.tolist(),corners_m=vertices.tolist(),
                        region=dict(ids=[]),q_return=list(q_start),support_confirmed=False,
                        qualification=qualification,
                        basis='SAME_LOCKED_Q_START_RECEIVING_NOT_QUALIFIED_COLLISION_CHECKS_REQUIRED')
        region=self.receiver();lo=np.asarray(region['lower']);hi=np.asarray(region['upper']);margin=region['margin']
        support_contains_region=False
        for ident in region['ids']:
            support=next((o for o in world if o['id']==ident),None)
            if support is None:raise ValueError('SITE_RECEIVER_SUPPORT_NOT_IN_COLLISION_SCENE')
            if abs(self.object_top(support)-hi[2])>.0004:raise ValueError('SITE_RECEIVER_HEIGHT_DISAGREES_WITH_SUPPORT')
            if 'size' in support:
                T=rigid(support['pose'])
                if abs(T[2,2]-1)>1e-6:raise ValueError('SITE_RECEIVER_REQUIRES_HORIZONTAL_TOP')
                points=[[x,y,hi[2]] for x in (lo[0],hi[0]) for y in (lo[1],hi[1])]
                local=transform(np.linalg.inv(T),points)
                support_contains_region |= bool(np.all(abs(local[:,:2])<=np.asarray(support['size'])[:2]/2+1e-9))
        if self.site.get('site') and not support_contains_region:raise ValueError('SITE_RECEIVER_REGION_NOT_COVERED_BY_MEASURED_SUPPORT')
        if np.any(vertices[:,:2].min(0)<lo[:2]+margin) or np.any(vertices[:,:2].max(0)>hi[:2]-margin):
            raise ValueError('Q_START_LOADED_BOX_OUTSIDE_CONFIRMED_RECEIVING_REGION')
        drop=float(vertices[:,2].min()-hi[2])
        if not -.0004<=drop<=.005:raise ValueError('Q_START_RELEASE_NOT_SUPPORTED_MAX_5_MM_GAP')
        if box.get('uncertainty'):
            from .partial_geometry import possible_bottom_interval
            lower_bottom,upper_bottom=possible_bottom_interval(box,pose)
            if lower_bottom<hi[2]-.0004 or upper_bottom>hi[2]+.005:
                raise ValueError('GEOMETRY_UNCERTAINTY_AFFECTS_RECEIVER: 部分箱体假设的释放高度不能全部落入承接面 5 mm 范围')
        return dict(status='PASS',pose=pose.tolist(),corners_m=vertices.tolist(),region=region,drop_m=drop,
                    q_return=list(q_start),support_confirmed=False,basis='MEASURED_SITE_GEOMETRY_SAME_LOCKED_Q_START_NOT_CONTACT_SENSOR_CERTIFICATION')
    @staticmethod
    def object_top(obj):
        T=np.asarray(obj['pose'])
        if 'convex_mesh' in obj:return float(transform(T,obj['convex_mesh']['vertices'])[:,2].max())
        return float(T[2,3]+abs(T[2,:3])@(np.asarray(obj['size'])/2))
    @staticmethod
    def loaded_start_failure(result,world):
        failure=result.get('failure');pair=failure.get('pair',[]) if isinstance(failure,dict) else []
        unknown=next((o for o in world if o.get('category')=='unknown' and o['id'] in pair),None)
        if unknown is not None:
            vertices=corners(unknown['pose'],unknown['size'])
            bounds=np.array([vertices.min(0),vertices.max(0)]).round(4).tolist()
            return ('LOADED_Q_START_UNKNOWN_SPACE: 带载起点与未观测区域间隙不足，尚不能确认通道空旷；'+
                    ' ↔ '.join(pair)+f'；该区域 base_link 范围（m）{bounds}；'+
                    '请补充对应盲区的实际观测或现场测量，不能直接清空未知障碍；原生诊断：'+str(failure))
        return 'LOADED_Q_START_COLLISION:'+str(failure)
    def detach_supports(self,world,box):
        bottom=min(np.asarray(box['corners_m'])[:,2]);supports=[]
        for o in world:
            if o['id']==box['id']:continue
            top=self.object_top(o)
            if abs(top-bottom)<=.0004:supports.append(dict(id=o['id'],top_world_m=top))
        return supports
    def ik_seeds(self,q,solutions):
        """Deterministic native IK seeds: q_start, then this task's own solutions.

        Candidates on one face differ by small offsets and quarter turns about
        the suction normal, so recent solutions and their J6 quarter-turn
        variants converge where the distant capture pose often does not. Every
        seed is a current-run native result; nothing historical is reused.
        """
        lower,upper,_,_=self.limits.arrays();seeds=[list(q)]
        for s in solutions[-2:][::-1]:
            for turn in (0.,np.pi/2,-np.pi/2,np.pi):
                seed=np.asarray(s,float).copy();seed[5]+=turn
                if np.all(seed>=lower) and np.all(seed<=upper) and not any(np.allclose(seed,x) for x in seeds):
                    seeds.append(seed.tolist())
        return seeds
    def endpoint_ik(self,pose,seeds,folder,name,cancelled=None):
        attempts=[]
        for k,seed in enumerate(seeds):
            ik=self.native_call(dict(op='ik',identity=self.init['identity'],q_start=list(seed),goal_pose=np.asarray(pose).tolist(),
                seed=6518,planning_only=False,position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4),
                folder,name+('' if k==0 else f'-{k}'),120,cancelled)
            attempts.append(ik.get('status'))
            if ik.get('status')=='SUCCESS':return list(ik['q']),attempts
        return None,attempts
    def endpoint_precheck(self,q,world,box,candidate,attachment,folder,cancelled=None,solutions=None):
        """Classify a candidate's task endpoints before any path search.

        Tool solids are rigid on the flange and the attached box is rigid on the
        tool, so their world collisions at a task pose do not depend on the IK
        branch: a (tool or payload) vs world failure rejects the candidate.
        IK failure or arm-link failures are not conclusive; the candidate keeps
        a lower search priority. Contact and detach share one TCP pose, so the
        detach check reuses the contact configuration with the payload attached.
        Returns CLEAR with the checked approach configuration, or NOT_CONCLUSIVE.
        """
        tool=set(self.clearance.get('tool_links',[]));ids={o['id'] for o in world}
        solutions=[] if solutions is None else solutions
        checks=[('approach',candidate['precontact_pose'],None,()),('contact',candidate['tcp_pose'],None,()),
                ('detach',None,attachment,self.detach_supports(world,box))]
        record=[];found={}
        try:
            for stage,pose,attached,supports in checks:
                if pose is None:qe=found['contact']
                else:
                    seeds=self.ik_seeds(q,solutions) if stage=='approach' else [found['approach']]
                    qe,attempts=self.endpoint_ik(pose,seeds,folder,stage+'-endpoint-ik',cancelled)
                    if qe is None:
                        record.append(dict(stage=stage,ik=attempts,conclusion='IK_NOT_FOUND_NOT_CONCLUSIVE'))
                        return dict(conclusion='NOT_CONCLUSIVE',reason=stage+'_IK_NOT_FOUND',record=record)
                result=self.native_call(dict(self.context(qe,world,box,candidate,stage,attached,supports),op='validate'),folder,stage+'-endpoint',120,cancelled)
                failure=result.get('failure') if isinstance(result.get('failure'),dict) else {}
                pair=list(failure.get('pair') or [])
                movable=tool|({box['id']} if attached is not None else set())
                valid=bool(result.get('native_valid') and result.get('legacy_intersection_valid'))
                blocked=(not valid and len(pair)==2 and
                         any(a in movable and b in ids-movable for a,b in (pair,pair[::-1])))
                record.append(dict(stage=stage,q=qe,native_valid=result.get('native_valid'),failure=failure or None,
                                   conclusion='TOOL_OR_PAYLOAD_POSE_BLOCKED' if blocked else 'PASS' if valid else 'NOT_CONCLUSIVE'))
                if blocked:
                    distance=failure.get('surface_distance_m')
                    raise ValueError(f'ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED: {stage} 终态 {pair[0]} ↔ {pair[1]}'+
                        (f' 间隙 {distance*1000:.2f} mm' if isinstance(distance,(int,float)) and distance>=0 else ' 相交')+
                        '；与逆解分支无关，跳过本候选的路径搜索')
                if not valid:return dict(conclusion='NOT_CONCLUSIVE',reason=stage+'_ARM_CONFIGURATION_REJECTED',record=record)
                found[stage]=qe
            solutions.append(found['approach'])
            return dict(conclusion='CLEAR',approach_q=found['approach'],record=record)
        finally:(folder/'endpoint-precheck.json').write_text(json.dumps(record,ensure_ascii=False))
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
        if box.get('uncertainty'):spec['geometry_uncertainty']=box['uncertainty']
        context=dict(identity=self.init['identity'],q_start=list(q),start_velocity=[0.]*6,path_constraints=[],
            scene_fingerprint=digest(world),world=world,allowed_pairs=pairs,attachment=attached,clearance_policy=policy,
            single_carton_contact=spec,stage=stage,planning_only=False,seed=6518)
        if self.site.get('site'):context['site_workspace_bounds_m']=self.site['site']['workspace_bounds_m']
        return context
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
    def checked_segment(self,context,goal,stage,scale,folder,progress,linear=False,cancelled=None,record=None):
        record=record or stage;progress('原生规划 '+record)
        req=dict(context,op='plan',pipeline_id='pilz_industrial_motion_planner',planner_id='LIN' if linear else 'PTP',
                 velocity_scale=1.,acceleration_scale=1.,allowed_planning_time_s=25.)
        if np.asarray(goal).shape==(4,4):req.update(goal_pose=np.asarray(goal).tolist(),flange_from_task_tcp=self.init['identity']['flange_from_task_tcp'])
        else:req['q_goal']=list(goal)
        result=self.native_call(req,folder,record+'-pilz',120,cancelled)
        if result.get('status')!='SUCCESS' and linear:
            # Pilz LIN rejects a Cartesian profile whose joint acceleration
            # exceeds the limits at full scaling. A slower profile is a new
            # native plan on the same scene and line; the sent trajectory is
            # retimed by time_scale afterwards and fully rechecked either way.
            for factor in LIN_RETRY_SCALES:
                req.update(velocity_scale=factor,acceleration_scale=factor)
                progress(f'{stage} 直线规划未通过（{result.get("status")}），以 {factor:g} 倍速度/加速度重新原生规划')
                result=self.native_call(req,folder,f'{record}-pilz-s{factor:g}',120,cancelled)
                if result.get('status')=='SUCCESS':break
        if result.get('status')!='SUCCESS' and not linear:
            # A new native search, with the identical loaded scene, never reverse playback.
            req.update(pipeline_id='ompl',planner_id='RRTConnectkConfigDefault')
            progress(f'{stage} 直连未通过（{result.get("status")}），正在搜索避让路径（25 秒预算）')
            result=self.native_call(req,folder,record+'-ompl',120,cancelled)
        (folder/(record+'-native.json')).write_text(json.dumps(dict(request=req,result=result)))
        if result.get('status')!='SUCCESS' or result.get('native_output_status')!='PASS':
            raise ValueError(stage+':NATIVE_REJECTED:'+str(result.get('failure',result.get('status'))))
        payload_reach=0.
        if context.get('attachment'):
            a=context['attachment']
            payload_reach=float(np.linalg.norm(corners(a['pose'],a['size']),axis=1).max())
        # Send at the configured scale when the interpolated trajectory stays
        # within every joint limit; a native profile planned exactly at its
        # limits can overshoot between knots, so slow down only as needed.
        for factor in SEND_SLOWDOWN_STEPS:
            try:prepared=prepare(result['points'],self.limits,scale*factor,self.config['period_s'],self.reach_bound+payload_reach);break
            except ValueError as error:
                if not str(error).startswith(('INTERPOLATION_LIMIT','DISCRETE_COMMAND_LIMIT')) or factor==SEND_SLOWDOWN_STEPS[-1]:raise
                progress(f'{record} 插值后超出关节限值（{error}），发送时间放大到 ×{scale*SEND_SLOWDOWN_STEPS[SEND_SLOWDOWN_STEPS.index(factor)+1]:g}')
        progress(f"检查 {stage} 最终发送轨迹：{len(prepared['points'])} 点")
        checked=self.native_call(dict(context,op='validate',probe_path=prepared['collision_path']),folder,record+'-sent',600,cancelled)
        (folder/(record+'-sent-check.json')).write_text(json.dumps(checked))
        if not checked.get('native_valid') or not checked.get('legacy_intersection_valid') or checked.get('path_failure') is not None:
            raise ValueError(stage+':SENT_PATH_REJECTED:'+str(checked.get('path_failure',checked.get('failure'))))
        planner=dict(pipeline_id=req['pipeline_id'],planner_id=req['planner_id'],velocity_scale=req['velocity_scale'],
                     acceleration_scale=req['acceleration_scale'],goal='JOINT' if 'q_goal' in req else 'POSE',
                     configured_time_scale=scale,send_time_scale=prepared['time_scale'])
        return dict(stage=stage,prepared=prepared,evidence=dict(native_plan='PASS',sent_path_collision='PASS',planner=planner,
                    native_result=result,collision_result=checked,context=context,hardware_execution='NOT_RUN'))
    @staticmethod
    def detach_goals(actual,box,candidate):
        """Deterministic detach TCP goals, tried in order.

        OUTWARD separates along the face normal by the box's largest extent with
        at least 10 mm of lift. When an observed obstacle stands just in front
        of the box bottom (e.g. a tray lip), a straight outward line cannot
        rise in time, so a 45-degree and a lift-first profile follow. The loaded
        return then plans freely from the detach end to q_start.
        """
        outward=np.asarray(candidate['face']['normal'],float);up=np.array([0.,0.,1.])
        far=rigid(actual).copy();far[:3,3]+=outward*(max(box['size_m'])+.01)
        far[2,3]+=max(0.,.01-(far[2,3]-actual[2,3]))
        goals=[('OUTWARD',far)]
        for name,out,lift in DETACH_ALTERNATIVES:
            goal=rigid(actual).copy();goal[:3,3]+=outward*out+up*lift
            if min(np.linalg.norm(goal[:3,3]-g[:3,3]) for _,g in goals)>.005:goals.append((name,goal))
        return goals
    def screen_candidate(self,q,world,box,candidate,sub,cancelled=None,solutions=None):
        """Loaded q_start endpoint (rejecting) and task endpoints (ranking)."""
        tcp=rigid(candidate['tcp_pose']);flange=tcp@np.linalg.inv(self.coordinate_model.tool.matrix)
        attachment=np.linalg.inv(flange)@rigid(box['pose'])
        release=self.release_check(q,attachment,box,world)
        # Full loaded endpoint is checked before any approach planning.
        release_supports=[dict(id=o['id'],top_world_m=self.object_top(o)) for o in world if o['id'] in release['region']['ids']]
        start_loaded=self.context(q,world,box,candidate,'return',attachment,release_supports)
        endcheck=self.native_call(dict(start_loaded,op='validate'),sub,'loaded-start',120,cancelled)
        (sub/'loaded-start-check.json').write_text(json.dumps(dict(request=start_loaded,result=endcheck)))
        if not endcheck.get('native_valid') or not endcheck.get('legacy_intersection_valid'):
            raise ValueError(self.loaded_start_failure(endcheck,world))
        screen=self.endpoint_precheck(q,world,box,candidate,attachment,sub,cancelled,solutions)
        return dict(release_supports=release_supports,screen=screen)
    def plan_candidate(self,q,world,box,candidate,sub,screened,progress=lambda _:None,cancelled=None):
        """Four checked native segments for one candidate; no permit issued here."""
        scale=self.site['time_scale'];tcp=rigid(candidate['tcp_pose']);release_supports=screened['release_supports']
        # A screened approach configuration is a native IK result already
        # validated in this scene; the planner re-validates it as its goal.
        goal=screened['screen'].get('approach_q') or candidate['precontact_pose']
        approach=self.checked_segment(self.context(q,world,box,candidate,'approach'),goal,'approach',scale,sub,progress,cancelled=cancelled)
        qpre=approach['prepared']['points'][-1]['q']
        contact=self.checked_segment(self.context(qpre,world,box,candidate,'contact'),tcp,'contact',scale,sub,progress,True,cancelled)
        qc=contact['prepared']['points'][-1]['q'];actual=self.coordinate_model.tcp(qc)
        seal=candidate_contact_check(actual,candidate)
        if not seal['valid']:raise ValueError('ACTUAL_NATIVE_CONTACT_FK_SEAL_REJECTED')
        actual_flange=actual@np.linalg.inv(self.coordinate_model.tool.matrix)
        attachment=np.linalg.inv(actual_flange)@rigid(box['pose'])
        release=self.release_check(q,attachment,box,world)
        original=self.detach_supports(world,box)
        extraction=None;rejected=[]
        for name,goal in self.detach_goals(actual,box,candidate):
            # Each profile is a new native LIN in the same scene with the full
            # attached box; the first that passes every check is used.
            try:extraction=self.checked_segment(self.context(qc,world,box,candidate,'detach',attachment,original),goal,'detach',scale,sub,progress,True,cancelled,
                                                'detach' if name=='OUTWARD' else 'detach-'+name.lower())
            except ValueError as e:rejected.append(dict(profile=name,reason=str(e)[:400]));continue
            extraction['evidence'].update(detach_profile=name,rejected_detach_profiles=rejected);break
        if extraction is None:raise ValueError('detach:ALL_PROFILES_REJECTED:'+json.dumps(rejected,ensure_ascii=False))
        qe=extraction['prepared']['points'][-1]['q']
        returning=self.checked_segment(self.context(qe,world,box,candidate,'return',attachment,release_supports),q,'return',scale,sub,progress,cancelled=cancelled)
        if np.max(abs(np.asarray(returning['prepared']['points'][-1]['q'])-q))>1e-7:raise ValueError('RETURN_NOT_LOCKED_Q_START')
        return dict(segments=[approach,contact,extraction,returning],attachment=attachment,release=release,seal=seal)
    def search_box(self,q,world,box,folder,progress=lambda _:None,cancelled=lambda:False,failures=None,candidate_filter=None):
        """Two-phase search for one target; returns the first complete plan.

        Phase 1 screens every candidate cheaply, so a blocked tool pose never
        costs a full path-search budget. Phase 2 plans all-endpoints-clear
        candidates first, preserving the seal-count order inside each group.
        Every failure is appended to ``failures``. No permit is issued here.
        """
        failures=[] if failures is None else failures
        try:candidates=contact_candidates(box,self.source['tool'],self.site['topology'])
        except ValueError as e:failures.append(dict(target=box['id'],reason=str(e)));return None
        screened=[];solutions=[]
        for i,candidate in enumerate(candidates):
            if candidate_filter is not None and i not in candidate_filter:continue
            if cancelled():raise ValueError('CANCELLED')
            sub=folder/(box['id']+'-'+str(i));sub.mkdir()
            progress(f'预检候选 {i+1}/{len(candidates)}：带载返回起点及接近 / 接触 / 脱离终态')
            try:screened.append((i,candidate,sub,self.screen_candidate(q,world,box,candidate,sub,cancelled,solutions)))
            except ValueError as e:failures.append(dict(target=box['id'],candidate=i,reason=str(e)))
        order=sorted(screened,key=lambda s:s[3]['screen']['conclusion']!='CLEAR')
        clear=sum(s[3]['screen']['conclusion']=='CLEAR' for s in order)
        progress(f'预检完成：{clear} 个候选终态全部通过，{len(order)-clear} 个逆解未定论，'
                 f'{len(candidates)-len(order)} 个已排除；按此顺序完整规划')
        for rank,(i,candidate,sub,screened_candidate) in enumerate(order):
            if cancelled():raise ValueError('CANCELLED')
            say=lambda text,i=i,rank=rank:progress(f'规划候选 {i+1}（{rank+1}/{len(order)}）：{text}')
            try:result=self.plan_candidate(q,world,box,candidate,sub,screened_candidate,say,cancelled)
            except ValueError as e:failures.append(dict(target=box['id'],candidate=i,reason=str(e)));continue
            return dict(result,index=i,candidate=candidate,screen=screened_candidate['screen'])
        return None
    def plan_cycle(self,observation,boxes,folder,progress=lambda _:None,selected=None,cancelled=lambda:False,scene=None):
        from ..contracts import Source,HOST_SESSION
        if (observation.frame.stamp.source!=Source.LIVE or observation.frame.stamp.host_session!=HOST_SESSION or
                observation.robot_sample.get('source')!='LIVE'):
            raise ValueError('PHYSICAL_PLANNER_REQUIRES_CURRENT_SESSION_LIVE_CAPTURE')
        self.authority.invalidate();self.cycle_binding=None
        qualification=release_policy_evidence(self.site)
        self.build()
        # A global configuration failure is not 32 independent infeasible
        # grasps. Retain the explicit measured policy without silently deferring.
        if not qualification['qualification_deferred']:self.receiver()
        if self.site.get('site') and scene is None:raise ValueError('FROZEN_REAL_SITE_SCENE_REQUIRED')
        world=deepcopy(scene['world']) if scene else [deepcopy(x) for x in self.world if x.get('category')!='carton']
        if scene and scene['observation_id']!=observation.frame.observation_id:raise ValueError('REAL_SCENE_CAPTURE_MISMATCH')
        world += [dict(id=b['id'],size=b['size_m'],pose=b['pose'],category='carton') for b in boxes]
        q=np.array(observation.q_start);failures=[]
        from unloading_sim.moveit2_backend import ResidentMoveItClient
        self.client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(folder),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=folder/'moveit.log')
        try:
            ready=self.client.request(self.init)
            if not ready.get('capabilities',{}).get('bounded_single_carton_planning'):
                raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_BOUNDED_SINGLE_CARTON_PLANNING')
            if any(b.get('uncertainty') for b in boxes) and not ready.get('capabilities',{}).get('partial_observed_contact_envelope'):
                raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_PARTIAL_OBSERVED_CONTACT')
            for box in boxes:
                if selected is not None and box['detection_index']!=selected:continue
                found=self.search_box(q,world,box,folder,progress,cancelled,failures)
                if found is None:continue
                i,candidate,screen,result=found['index'],found['candidate'],found['screen'],found
                attachment,release=result['attachment'],result['release']
                self.cycle_binding=dict(task_id=folder.name,observation=observation.metadata(),target=box,world=digest(world),candidate=candidate,
                    attachment=attachment.tolist(),release=release,expected_attachment='PLANNED_NOT_PHYSICALLY_CONFIRMED')
                self.cycle_binding['site_scene']=scene
                self.cycle_binding['release_world']=world
                size=np.array(box['size_m']);mass=self.site['carton_mass_kg']
                self.cycle_binding['payload_mass_properties']=dict(mass_kg=mass,com_box_m=[0.,0.,0.],
                    inertia_box_kg_m2=(mass/12*np.diag([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])).tolist(),
                    basis='USER_CONFIRMED_2_KG_UNIFORM_CUBOID_INERTIA_ESTIMATE')
                binding=self.binding();segments=result['segments']
                for segment in segments:segment['package']=self.authority.register(segment.pop('prepared'),binding,segment.pop('evidence'))
                plan=dict(binding=binding,segments=segments,target=box,candidate=candidate,release=release,
                    q_start=q.tolist(),T_world_tcp_start=observation.world_tcp_start,full_path_validated=True,
                    candidate_index=i,endpoint_screen=screen['conclusion'],
                    duration_s=sum(s['package']['prepared']['send_duration_s'] for s in segments),hardware_execution='NOT_RUN')
                (folder/'complete-plan.json').write_text(json.dumps(plan));return plan
            # Keep every concrete failure on disk; one task-status label must
            # not grow into 32 copies of the full native collision JSON.
            (folder/'planning-failures.json').write_text(json.dumps(failures,ensure_ascii=False,indent=2))
            counts=Counter(f['reason'].split(':',1)[0] for f in failures)
            summary='；'.join(f'{reason} × {count}' for reason,count in counts.items())
            first=failures[0]['reason'][:600] if failures else '没有符合所选编号的目标'
            raise ValueError(f'NO_COMPLETE_FEASIBLE_SINGLE_CARTON_PLAN: {summary}；首项：{first}；详情：planning-failures.json')
        finally:self.client.close();self.client=None
