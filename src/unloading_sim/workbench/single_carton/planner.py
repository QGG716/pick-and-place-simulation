"""Full physical single-carton native MTC/Pilz/OMPL plan before any actuation."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from collections import Counter
from pathlib import Path
import json
import threading
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
# Deterministic extra IK seeds sampled inside the joint limits, and how many
# distinct IK branches (closest to q_start first) get a native endpoint check.
IK_RANDOM_SEEDS=64
IK_BRANCHES_CHECKED=3
# A payload/world pair is target-level (candidate independent) only when it
# fails by more than the IK pose tolerance can move the attached box.
TARGET_BLOCK_MARGIN_M=1e-4
# Definite payload/world intersection depth for the pre-native geometric test.
PAYLOAD_REST_DEPTH_M=.001
# Smallest probe-path chunk handed to one native checker process.
CHECK_CHUNK_MIN_STATES=500
# Worker status for a plan whose own waypoint check the caller deferred to
# its exact sent-path check (physical single-carton scope only).
DEFERRED_OUTPUT='DEFERRED_TO_CALLER_SENT_PATH_CHECK'


def failure_category(reason):
    """Summary key that keeps the stage and distinguishes how it failed.

    'approach:NATIVE_REJECTED:NATIVE_SOLVER_BUDGET_EXHAUSTED' and
    'approach:SENT_PATH_REJECTED:{...}' must not be counted as one cause:
    the first is a bounded search running out of time, the second a planned
    path whose exact sent samples failed the collision check.
    """
    parts=str(reason).split(':')
    if len(parts)>=3 and parts[1]=='NATIVE_REJECTED':
        status=parts[2].strip()
        return f'{parts[0]}:{status if status.isupper() and " " not in status else "NATIVE_REJECTED"}'
    if len(parts)>=2 and parts[1] in ('SENT_PATH_REJECTED','ALL_PROFILES_REJECTED'):return f'{parts[0]}:{parts[1]}'
    return parts[0]


class TargetBlocked(ValueError):
    """Every candidate of this target fails identically; stop its search."""


class SegmentRejected(ValueError):
    def __init__(self,stage,message,result=None):
        super().__init__(message);self.stage=stage;self.native_result=result


def obb_overlap_depth(Ta,half_a,Tb,half_b):
    """Separating-axis test for two oriented boxes.

    Returns the smallest overlap along all 15 candidate axes: positive is a
    certain penetration of at least that depth, zero or negative is touching
    or separated.
    """
    Ta=np.asarray(Ta,float);Tb=np.asarray(Tb,float);Ra=Ta[:3,:3];Rb=Tb[:3,:3]
    half_a=np.asarray(half_a,float);half_b=np.asarray(half_b,float);t=Tb[:3,3]-Ta[:3,3]
    axes=[Ra[:,i] for i in range(3)]+[Rb[:,i] for i in range(3)]+[np.cross(Ra[:,i],Rb[:,j]) for i in range(3) for j in range(3)]
    depth=np.inf
    for axis in axes:
        length=np.linalg.norm(axis)
        if length<1e-9:continue
        axis=axis/length
        depth=min(depth,np.abs(Ra.T@axis)@half_a+np.abs(Rb.T@axis)@half_b-abs(axis@t))
    return float(depth)

class SinglePlanner(EmptyPlanner):
    def __init__(self,config,mapping,authority,coordinate_model):
        self.site=config;self.coordinate_model=coordinate_model;self.cycle_binding=None
        self.pool=[];self.pool_status=[];self.pool_threads=[];self.pool_lock=threading.Lock();self.pool_closed=False
        self.defer_output_check=False
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
    def spread_seeds(self):
        """Fixed pseudo-random seeds inside the joint limits (same every run).

        A single seed-directed KDL solve only converges near its seed; these
        cover the other IK branches. Each costs about one millisecond.
        """
        lower,upper,_,_=self.limits.arrays()
        return (lower+np.random.default_rng(6518).random((IK_RANDOM_SEEDS,6))*(upper-lower)).tolist()
    def endpoint_ik(self,pose,seeds,folder,name,cancelled=None,q_reference=None):
        """Distinct native IK solutions of one pose, closest to q_reference first.

        One record per pose: the request and all seeds are persisted before
        dispatch, every reply after. IK never checks collision; callers do.
        """
        template=dict(op='ik',identity=self.init['identity'],goal_pose=np.asarray(pose).tolist(),seed=6518,planning_only=False,
                      position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4,request_budget_s=110.)
        path=folder/(name+'-native.json');started=time.monotonic()
        record=dict(request=template,seeds=[list(map(float,s)) for s in seeds],status='PENDING',ipc_timeout_s=120)
        path.write_text(json.dumps(record));replies=[];solutions=[]
        try:
            for seed in seeds:
                if cancelled is not None and cancelled():raise ValueError('CANCELLED')
                reply=self.client.request(dict(template,q_start=list(map(float,seed))),timeout=120,cancelled=cancelled)
                replies.append(dict(status=reply.get('status'),q=reply.get('q'),position_error_m=reply.get('position_error_m'),
                                    orientation_error_rad=reply.get('orientation_error_rad')))
                if reply.get('status')=='SUCCESS':
                    q=list(map(float,reply['q']))
                    if not any(np.max(abs(np.asarray(q)-x))<1e-3 for x in solutions):solutions.append(q)
            reference=np.asarray(seeds[0] if q_reference is None else q_reference,float)
            solutions.sort(key=lambda x:float(np.linalg.norm(np.asarray(x)-reference)))
            record.update(status='REPLIED',replies=replies,distinct_solutions=solutions,elapsed_s=time.monotonic()-started)
            return solutions,[r['status'] for r in replies]
        except Exception as error:
            record.update(status='REQUEST_FAILED',error=str(error),replies=replies,elapsed_s=time.monotonic()-started);raise
        finally:path.write_text(json.dumps(record))
    def payload_world_blocked(self,failure,box,world):
        """A payload/world failure bigger than the IK tolerance, at the box's rest pose."""
        pair=list((failure or {}).get('pair') or []);others={o['id'] for o in world}-{box['id']}
        if len(pair)!=2 or box['id'] not in pair or not set(pair)-{box['id']}<=others:return False
        distance=failure.get('surface_distance_m');required=failure.get('required_pair_clearance_m',0.)
        return isinstance(distance,(int,float)) and (distance<0 or required-distance>TARGET_BLOCK_MARGIN_M)
    def target_blocked_message(self,failure,box,source):
        pair=failure['pair'];other=pair[1] if pair[0]==box['id'] else pair[0];distance=failure.get('surface_distance_m')
        gap=f'间隙 {distance*1000:.2f} mm（要求 {failure.get("required_pair_clearance_m",0)*1000:.1f} mm）' if distance>=0 else '相交'
        return (f'TARGET_PAYLOAD_BLOCKED_AT_REST: 箱体在原位时碰撞包络与 {other} {gap}；带箱脱离的起点与吸取候选无关，'
                f'本目标所有候选都无法脱离，停止搜索（{source}）')
    def payload_rest_check(self,world,box):
        """Candidate-independent payload geometry at the box's rest pose.

        At contact the attached box is exactly the observed envelope, whatever
        the grasp; any definite intersection with a non-support world object
        (or leaving the site workspace) rejects every candidate. Only certain
        overlaps deeper than PAYLOAD_REST_DEPTH_M are reported here; anything
        closer is left to the native clearance checks.
        """
        supports={s['id'] for s in self.detach_supports(world,box)};hits=[]
        for o in world:
            if o['id']==box['id'] or o['id'] in supports or 'size' not in o:continue
            depth=obb_overlap_depth(box['pose'],np.asarray(box['size_m'])/2,o['pose'],np.asarray(o['size'])/2)
            if depth>PAYLOAD_REST_DEPTH_M:hits.append(dict(id=o['id'],category=o.get('category'),overlap_depth_m=depth))
        bounds=(self.site.get('site') or {}).get('workspace_bounds_m')
        outside=False
        if bounds:
            lo,hi=np.asarray(bounds,float);c=np.asarray(box['corners_m'],float)
            outside=bool(np.any(c<lo) or np.any(c>hi))
        return dict(intersections=sorted(hits,key=lambda h:-h['overlap_depth_m']),outside_workspace=outside,supports=sorted(supports))
    def endpoint_precheck(self,q,world,box,candidate,attachment,folder,cancelled=None,solutions=None):
        """Classify a candidate's task endpoints before any path search.

        Tool solids are rigid on the flange and the attached box is rigid on the
        tool, so their world collisions at a task pose do not depend on the IK
        branch: a (tool or payload) vs world failure rejects the candidate, and
        a payload vs world failure at detach rejects the whole target. IK is
        searched from many deterministic seeds; up to IK_BRANCHES_CHECKED
        distinct approach branches (closest to q_start first) are checked.
        Contact and detach share one TCP pose, so detach reuses the contact
        configuration with the payload attached. Returns CLEAR with checked
        approach/contact configurations, or NOT_CONCLUSIVE with the reason.
        """
        tool=set(self.clearance.get('tool_links',[]));ids={o['id'] for o in world}
        solutions=[] if solutions is None else solutions;supports=self.detach_supports(world,box)
        record=[];notes=[]
        def check(stage,qe,attached,stage_supports):
            result=self.native_call(dict(self.context(qe,world,box,candidate,stage,attached,stage_supports),op='validate'),
                                    folder,stage+'-endpoint'+('' if not notes else f'-b{len(notes)}'),120,cancelled)
            failure=result.get('failure') if isinstance(result.get('failure'),dict) else {}
            pair=list(failure.get('pair') or [])
            movable=tool|({box['id']} if attached is not None else set())
            valid=bool(result.get('native_valid') and result.get('legacy_intersection_valid'))
            blocked=(not valid and len(pair)==2 and any(a in movable and b in ids-movable for a,b in (pair,pair[::-1])))
            record.append(dict(stage=stage,q=qe,native_valid=result.get('native_valid'),failure=failure or None,
                               conclusion='TOOL_OR_PAYLOAD_POSE_BLOCKED' if blocked else 'PASS' if valid else 'NOT_CONCLUSIVE'))
            if blocked and attached is not None and self.payload_world_blocked(failure,box,world):
                raise TargetBlocked(self.target_blocked_message(failure,box,'终态预检'))
            if blocked:
                distance=failure.get('surface_distance_m')
                raise ValueError(f'ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED: {stage} 终态 {pair[0]} ↔ {pair[1]}'+
                    (f' 间隙 {distance*1000:.2f} mm' if isinstance(distance,(int,float)) and distance>=0 else ' 相交')+
                    '；与逆解分支无关，跳过本候选的路径搜索')
            return valid
        try:
            branches,attempts=self.endpoint_ik(candidate['precontact_pose'],self.ik_seeds(q,solutions)+self.spread_seeds(),
                                               folder,'approach-endpoint-ik',cancelled,q)
            if not branches:
                record.append(dict(stage='approach',ik=dict(seeds=len(attempts),solutions=0),conclusion='IK_NOT_FOUND_NOT_CONCLUSIVE'))
                return dict(conclusion='NOT_CONCLUSIVE',reason='approach_IK_NOT_FOUND',record=record)
            for qa in branches[:IK_BRANCHES_CHECKED]:
                if not check('approach',qa,None,()):notes.append('approach_ARM_CONFIGURATION_REJECTED');continue
                contacts,_=self.endpoint_ik(candidate['tcp_pose'],[qa],folder,'contact-endpoint-ik'+('' if not notes else f'-b{len(notes)}'),cancelled,qa)
                if not contacts:
                    record.append(dict(stage='contact',ik='NATIVE_IK_FAILED',conclusion='IK_NOT_FOUND_NOT_CONCLUSIVE'));notes.append('contact_IK_NOT_FOUND');continue
                qc=contacts[0]
                if not check('contact',qc,None,()):notes.append('contact_ARM_CONFIGURATION_REJECTED');continue
                if not check('detach',qc,attachment,supports):notes.append('detach_ARM_CONFIGURATION_REJECTED');continue
                solutions.append(qa)
                return dict(conclusion='CLEAR',approach_q=qa,contact_q=qc,ik_branches=len(branches),record=record)
            return dict(conclusion='NOT_CONCLUSIVE',reason=notes[-1] if notes else 'NO_CHECKED_BRANCH',ik_branches=len(branches),record=record)
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
    def native_call(self,request,folder,name,timeout,cancelled=None,client=None):
        # Persist before dispatch: a timed-out worker cannot return its request
        # or profiling data. Keep every Pilz/OMPL attempt instead of overwriting.
        request=dict(request,request_budget_s=timeout-10.)
        path=folder/(name+'-native.json');started=time.monotonic()
        record=dict(request=request,status='PENDING',ipc_timeout_s=timeout)
        path.write_text(json.dumps(record))
        try:
            result=(client or self.client).request(request,timeout=timeout,cancelled=cancelled)
            record.update(status='REPLIED',result=result,elapsed_s=time.monotonic()-started)
            return result
        except Exception as error:
            record.update(status='REQUEST_FAILED',error=str(error),elapsed_s=time.monotonic()-started)
            raise RuntimeError(f'{name}:{error}；详情：{path.name}') from error
        finally:path.write_text(json.dumps(record))
    # -- resident checker pool (final sent-path checks and concurrent plans) ----
    def worker_ready(self,ready,boxes=()):
        """Required capabilities of the main native worker; binds output-check deferral."""
        capabilities=ready.get('capabilities',{})
        if not capabilities.get('bounded_single_carton_planning'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_BOUNDED_SINGLE_CARTON_PLANNING')
        if any(b.get('uncertainty') for b in boxes) and not capabilities.get('partial_observed_contact_envelope'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_PARTIAL_OBSERVED_CONTACT')
        self.defer_output_check=bool(capabilities.get('single_carton_output_check_deferral'))
        return ready
    def start_check_pool(self,folder):
        """Extra resident native workers, initialised in the background.

        Same worker binary, model, identity and policy as the main worker; they
        only split final sent-path validation (every state still checked) and
        run independent segment plans concurrently. Fewer ready workers only
        means less parallelism, never a weaker check.
        """
        from unloading_sim.moveit2_backend import ResidentMoveItClient
        self.pool=[];self.pool_status=[];self.pool_threads=[]
        count=int(self.site.get('native_check_workers',0))
        def start(k):
            status=dict(index=k,state='STARTING');self.pool_status.append(status);client=None
            try:
                client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(folder),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                            log_path=folder/f'moveit-check-{k}.log')
                ready=client.request(self.init,timeout=180)
                if not ready.get('capabilities',{}).get('bounded_single_carton_planning'):raise RuntimeError('NATIVE_WORKER_CAPABILITY_MISSING')
                with self.pool_lock:
                    if self.pool_closed:client.close();status['state']='CLOSED';return
                    self.pool.append(client)
                status['state']='READY'
            except Exception as error:
                status.update(state='FAILED',error=str(error))
                if client is not None:client.close()
        self.pool_closed=False
        for k in range(count):
            thread=threading.Thread(target=start,args=(k,),daemon=True,name=f'native-check-{k}');thread.start();self.pool_threads.append(thread)
    def wait_check_pool(self,timeout=240.):
        """Wait until every pool worker is READY or FAILED, so lane count is fixed."""
        deadline=time.monotonic()+timeout
        for thread in self.pool_threads:thread.join(max(0.,deadline-time.monotonic()))
    def close_check_pool(self):
        with self.pool_lock:
            self.pool_closed=True;pool,self.pool=list(self.pool),[]
        for client in pool:
            try:client.close()
            except Exception:pass
    def checkers(self):
        """Live native workers: the main one first, then ready pool workers.

        A worker whose process ended (a cancelled or failed request closes
        it) is never handed another request.
        """
        alive=lambda c:getattr(getattr(c,'process',None),'poll',lambda:None)() is None
        with self.pool_lock:return [c for c in [self.client]+list(self.pool) if alive(c)]
    # -- segments -----------------------------------------------------------------
    def plan_segment(self,context,goal,stage,scale,folder,progress,linear=False,cancelled=None,record=None,client=None):
        """Native plan and send-grid preparation, without the final dense check."""
        record=record or stage;progress('原生规划 '+record)
        req=dict(context,op='plan',pipeline_id='pilz_industrial_motion_planner',planner_id='LIN' if linear else 'PTP',
                 velocity_scale=1.,acceleration_scale=1.,allowed_planning_time_s=25.)
        # The worker's own dense check of its linear-joint waypoint path is
        # deferred only when it supports that, because sent_check() runs the
        # identical output-phase check on the exact sent path (all waypoints
        # included) before any segment can be registered.
        if self.defer_output_check:req['defer_output_check_to_sent_path']=True
        if np.asarray(goal).shape==(4,4):req.update(goal_pose=np.asarray(goal).tolist(),flange_from_task_tcp=self.init['identity']['flange_from_task_tcp'])
        else:req['q_goal']=list(goal)
        result=self.native_call(req,folder,record+'-pilz',120,cancelled,client)
        if result.get('status')!='SUCCESS' and linear and not str(result.get('status','')).startswith('INVALID_START'):
            # Pilz LIN rejects a Cartesian profile whose joint acceleration
            # exceeds the limits at full scaling. A slower profile is a new
            # native plan on the same scene and line; the sent trajectory is
            # retimed by time_scale afterwards and fully rechecked either way.
            # An invalid start state does not depend on the speed profile.
            for factor in LIN_RETRY_SCALES:
                req.update(velocity_scale=factor,acceleration_scale=factor)
                progress(f'{stage} 直线规划未通过（{result.get("status")}），以 {factor:g} 倍速度/加速度重新原生规划')
                result=self.native_call(req,folder,f'{record}-pilz-s{factor:g}',120,cancelled,client)
                if result.get('status')=='SUCCESS':break
        if result.get('status')!='SUCCESS' and not linear and not str(result.get('status','')).startswith('INVALID_START'):
            # A new native search, with the identical loaded scene, never reverse playback.
            req.update(pipeline_id='ompl',planner_id='RRTConnectkConfigDefault')
            progress(f'{stage} 直连未通过（{result.get("status")}），正在搜索避让路径（25 秒预算）')
            result=self.native_call(req,folder,record+'-ompl',120,cancelled,client)
        (folder/(record+'-native.json')).write_text(json.dumps(dict(request=req,result=result)))
        accepted={'PASS'}|({DEFERRED_OUTPUT} if req.get('defer_output_check_to_sent_path') else set())
        if result.get('status')!='SUCCESS' or result.get('native_output_status') not in accepted:
            raise SegmentRejected(stage,stage+':NATIVE_REJECTED:'+str(result.get('failure',result.get('status'))),result)
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
        planner=dict(pipeline_id=req['pipeline_id'],planner_id=req['planner_id'],velocity_scale=req['velocity_scale'],
                     acceleration_scale=req['acceleration_scale'],goal='JOINT' if 'q_goal' in req else 'POSE',
                     configured_time_scale=scale,send_time_scale=prepared['time_scale'],
                     native_output_check='DEFERRED_TO_SENT_PATH_CHECK' if result['native_output_status']==DEFERRED_OUTPUT else 'PASS')
        return dict(stage=stage,record=record,prepared=prepared,context=context,native_result=result,planner=planner)
    def sent_check(self,segment,folder,progress,cancelled=None):
        """Final dense validation of the exact sent path, split across checkers.

        Chunks are contiguous and share their boundary state, so every edge of
        the probe path, and every sample the native checker inserts on it, is
        checked by exactly one worker with the identical scene and policy.
        """
        stage,record,context=segment['stage'],segment['record'],segment['context']
        path=segment['prepared']['collision_path'];workers=self.checkers()
        count=max(1,min(len(workers),(len(path)-1)//CHECK_CHUNK_MIN_STATES))
        edges=np.linspace(0,len(path)-1,count+1).round().astype(int)
        progress(f"检查 {stage} 最终发送轨迹：{len(segment['prepared']['points'])} 点，{len(path)} 个检查状态，{count} 个进程并行")
        def run(k):
            a,b=int(edges[k]),int(edges[k+1]);name=record+'-sent'+('' if count==1 else f'-p{k}')
            return a,b,self.native_call(dict(context,op='validate',probe_path=path[a:b+1]),folder,name,600,cancelled,workers[k])
        started=time.monotonic()
        with ThreadPoolExecutor(max_workers=count) as executor:parts=list(executor.map(run,range(count)))
        valid=all(r.get('native_valid') and r.get('legacy_intersection_valid') and r.get('path_failure') is None for _,_,r in parts)
        path_failure=failure=None
        for a,b,r in parts:
            if r.get('path_failure') is not None and path_failure is None:
                path_failure=dict(r['path_failure'],chunk_start_state=a,edge_in_path=a+int(r['path_failure'].get('edge',0)))
            if r.get('failure') is not None and failure is None:failure=r['failure']
        checked=dict(native_valid=all(r.get('native_valid') for _,_,r in parts),
                     legacy_intersection_valid=all(r.get('legacy_intersection_valid') for _,_,r in parts),
                     path_failure=path_failure,failure=failure,probe_states=len(path),parallel_checkers=count,
                     wall_s=time.monotonic()-started,chunks=[dict(states=[a,b],result=r) for a,b,r in parts])
        (folder/(record+'-sent-check.json')).write_text(json.dumps(checked))
        if not valid:raise SegmentRejected(stage,stage+':SENT_PATH_REJECTED:'+str(path_failure or failure),checked)
        segment['evidence']=dict(native_plan='PASS',sent_path_collision='PASS',planner=segment['planner'],
                                 native_result=segment['native_result'],collision_result=checked,context=context,hardware_execution='NOT_RUN')
        return segment
    def checked_segment(self,context,goal,stage,scale,folder,progress,linear=False,cancelled=None,record=None):
        segment=self.plan_segment(context,goal,stage,scale,folder,progress,linear,cancelled,record)
        self.sent_check(segment,folder,progress,cancelled)
        return dict(stage=stage,prepared=segment['prepared'],evidence=segment['evidence'])
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
    def plan_candidate_segments(self,q,world,box,candidate,sub,screened,progress=lambda _:None,cancelled=None,
                                clients=None,reuse=None):
        """Plan all four native segments for one candidate; no dense sent check.

        With a screened approach configuration the contact/detach chain is
        planned from it first; approach and loaded return then run on two
        workers when two are given. Detach profiles are tried in order;
        ``reuse`` carries an earlier attempt's approach/contact and the profiles
        already rejected, so only a new detach + return are planned.
        """
        clients=list(clients or self.checkers())[:2];primary=clients[0];helper=clients[1] if len(clients)>1 else None
        plan=lambda *a,client=primary,**k:self.plan_segment(*a,client=client,**k)
        scale=self.site['time_scale'];tcp=rigid(candidate['tcp_pose']);release_supports=screened['release_supports']
        if reuse is None:
            screen=screened['screen'];known=screen.get('approach_q');approach=None
            if known is None:
                approach=plan(self.context(q,world,box,candidate,'approach'),candidate['precontact_pose'],'approach',scale,sub,progress,cancelled=cancelled)
                qpre=approach['prepared']['points'][-1]['q']
            else:qpre=list(known)
            contact=plan(self.context(qpre,world,box,candidate,'contact'),tcp,'contact',scale,sub,progress,True,cancelled)
            qc=contact['prepared']['points'][-1]['q'];actual=self.coordinate_model.tcp(qc)
            seal=candidate_contact_check(actual,candidate)
            if not seal['valid']:raise ValueError('ACTUAL_NATIVE_CONTACT_FK_SEAL_REJECTED')
            actual_flange=actual@np.linalg.inv(self.coordinate_model.tool.matrix)
            attachment=np.linalg.inv(actual_flange)@rigid(box['pose'])
            reuse=dict(approach=approach,known=known,qpre=qpre,contact=contact,qc=qc,actual=actual,seal=seal,attachment=attachment,
                       release=self.release_check(q,attachment,box,world),original=self.detach_supports(world,box),rejected=[])
        r=dict(reuse);r['rejected']=list(r['rejected']);excluded={x['profile'] for x in r['rejected']}
        for name,goal in self.detach_goals(r['actual'],box,candidate):
            if name in excluded:continue
            # Each profile is a new native LIN in the same scene with the full
            # attached box; the first whose whole chain passes is used.
            record='detach' if name=='OUTWARD' else 'detach-'+name.lower()
            try:extraction=plan(self.context(r['qc'],world,box,candidate,'detach',r['attachment'],r['original']),goal,'detach',scale,sub,progress,True,cancelled,record)
            except SegmentRejected as e:
                failure=(e.native_result or {}).get('failure')
                if str((e.native_result or {}).get('status','')).startswith('INVALID_START') and self.payload_world_blocked(failure,box,world):
                    raise TargetBlocked(self.target_blocked_message(failure,box,'带箱脱离起点')) from e
                r['rejected'].append(dict(profile=name,reason=str(e)[:400]));continue
            qe=extraction['prepared']['points'][-1]['q']
            return_context=self.context(qe,world,box,candidate,'return',r['attachment'],release_supports)
            try:
                if r['approach'] is None and helper is None:
                    # One worker: fixed order keeps its OMPL request history reproducible.
                    r['approach']=plan(self.context(q,world,box,candidate,'approach'),r['known'],'approach',scale,sub,progress,False,cancelled)
                    if np.max(abs(np.asarray(r['approach']['prepared']['points'][-1]['q'])-r['qpre']))>1e-7:
                        raise ValueError('APPROACH_END_NOT_SCREENED_CONFIGURATION')
                    returning=plan(return_context,q,'return',scale,sub,progress,cancelled=cancelled)
                elif r['approach'] is None:
                    with ThreadPoolExecutor(max_workers=2) as executor:
                        first=executor.submit(plan,self.context(q,world,box,candidate,'approach'),r['known'],'approach',scale,sub,progress,False,cancelled)
                        second=executor.submit(plan,return_context,q,'return',scale,sub,progress,False,cancelled,None,client=helper)
                        approach_error=first.exception()
                        if approach_error is not None:raise approach_error
                        r['approach']=first.result()
                        if np.max(abs(np.asarray(r['approach']['prepared']['points'][-1]['q'])-r['qpre']))>1e-7:
                            raise ValueError('APPROACH_END_NOT_SCREENED_CONFIGURATION')
                        returning=second.result()
                else:returning=plan(return_context,q,'return',scale,sub,progress,cancelled=cancelled)
            except SegmentRejected as e:
                if e.stage=='approach':raise
                r['rejected'].append(dict(profile=name,reason=str(e)[:400]));continue
            if np.max(abs(np.asarray(returning['prepared']['points'][-1]['q'])-q))>1e-7:raise ValueError('RETURN_NOT_LOCKED_Q_START')
            return dict(r,extraction=extraction,returning=returning,profile=name)
        raise ValueError('detach:ALL_PROFILES_REJECTED:'+json.dumps(r['rejected'],ensure_ascii=False))
    def verify_candidate(self,planned,sub,progress=lambda _:None,cancelled=None):
        """Final dense sent-path checks of a planned candidate, cheapest first.

        A segment checked in an earlier detach-profile attempt (approach and
        contact) keeps its evidence. Raises SegmentRejected on any failure.
        """
        segments=[planned[k] for k in ('approach','contact','extraction','returning')]
        for segment in sorted(segments,key=lambda s:len(s['prepared']['collision_path'])):
            if segment.get('evidence') is None:self.sent_check(segment,sub,progress,cancelled)
        planned['extraction']['evidence'].update(detach_profile=planned['profile'],rejected_detach_profiles=planned['rejected'])
        return dict(segments=[dict(stage=s['stage'],prepared=s['prepared'],evidence=s['evidence']) for s in segments],
                    attachment=planned['attachment'],release=planned['release'],seal=planned['seal'])
    def replan_after(self,planned,error):
        """reuse for the next detach profile after a detach/return sent-path rejection."""
        if error.stage in ('approach','contact'):raise error
        return dict(planned,rejected=planned['rejected']+[dict(profile=planned['profile'],reason=str(error)[:400])])
    def plan_candidate(self,q,world,box,candidate,sub,screened,progress=lambda _:None,cancelled=None,clients=None):
        """Four native segments for one candidate, planned then densely checked; no permit."""
        planned=self.plan_candidate_segments(q,world,box,candidate,sub,screened,progress,cancelled,clients)
        while True:
            try:return self.verify_candidate(planned,sub,progress,cancelled)
            except SegmentRejected as e:
                planned=self.plan_candidate_segments(q,world,box,candidate,sub,screened,progress,cancelled,clients,self.replan_after(planned,e))
    def parallel_candidates(self,q,world,box,order,progress,cancelled,failures):
        """Plan candidates concurrently, verify them strictly in priority order.

        Candidate ranks are assigned to lanes round-robin (rank % lanes); each
        lane plans its own candidates in rank order on its own native worker,
        so every worker's request history, and therefore every OMPL result,
        is reproducible. Lanes plan only. The main thread checks candidates in
        rank order with all workers and accepts the first that passes, exactly
        like the sequential search; a detach/return rejection sends the
        candidate back to its own lane for the next detach profile. New lane
        work pauses while a check runs. Any exit stops and joins every lane.
        """
        lanes=self.checkers()[:max(1,int(self.site.get('parallel_candidate_lanes',1)))]
        count=len(lanes);stop=threading.Event();idle=threading.Event();idle.set()
        lane_cancelled=lambda:stop.is_set() or bool(cancelled())
        import queue
        results={rank:queue.Queue() for rank in range(len(order))};commands={rank:queue.Queue() for rank in range(len(order))}
        progress(f'{len(order)} 个候选分配到 {count} 个原生进程同时规划；按优先级逐个检查发送路径')
        def lane(index):
            client=lanes[index]
            for rank in range(index,len(order),count):
                while not idle.wait(.05):
                    if lane_cancelled():break
                if lane_cancelled():results[rank].put(('STOPPED',None));continue
                i,candidate,sub,screened=order[rank]
                say=lambda text,i=i,rank=rank:progress(f'规划候选 {i+1}（{rank+1}/{len(order)}，进程 {index}）：{text}')
                reuse=None
                while True:
                    try:
                        planned=self.plan_candidate_segments(q,world,box,candidate,sub,screened,say,lane_cancelled,[client],reuse)
                        results[rank].put(('PLANNED',planned))
                    except TargetBlocked as e:results[rank].put(('TARGET_BLOCKED',e));break
                    except ValueError as e:results[rank].put(('FAILED',e));break
                    except Exception as e:
                        # This lane's worker is closed; later ranks report it too.
                        results[rank].put(('ERROR',e))
                        for later in range(rank+count,len(order),count):results[later].put(('ERROR',e))
                        return
                    command=commands[rank].get()
                    if command[0]!='REPLAN':break
                    reuse=command[1]
        threads=[threading.Thread(target=lane,args=(k,),daemon=True,name=f'candidate-lane-{k}') for k in range(count)]
        for thread in threads:thread.start()
        def receive(rank):
            while True:
                if cancelled():raise ValueError('CANCELLED')
                try:return results[rank].get(timeout=.1)
                except queue.Empty:continue
        try:
            for rank,(i,candidate,sub,screened) in enumerate(order):
                kind,value=receive(rank)
                while kind=='PLANNED':
                    idle.clear()
                    try:
                        result=self.verify_candidate(value,sub,lambda text,i=i,rank=rank:progress(f'检查候选 {i+1}（{rank+1}/{len(order)}）：{text}'),cancelled)
                    except SegmentRejected as e:
                        try:reuse=self.replan_after(value,e)
                        except SegmentRejected as fatal:kind,value='FAILED',fatal;commands[rank].put(('DONE',));break
                        commands[rank].put(('REPLAN',reuse));idle.set();kind,value=receive(rank);continue
                    finally:idle.set()
                    commands[rank].put(('DONE',))
                    return dict(result,index=i,candidate=candidate,screen=screened['screen'],lanes=count)
                if kind=='TARGET_BLOCKED':raise value
                if kind=='ERROR':raise value
                if kind=='FAILED':failures.append(dict(target=box['id'],candidate=i,reason=str(value)))
            return None
        finally:
            stop.set();idle.set()
            for rank in commands:commands[rank].put(('STOP',))
            for thread in threads:thread.join()
    def search_box(self,q,world,box,folder,progress=lambda _:None,cancelled=lambda:False,failures=None,candidate_filter=None):
        """Two-phase search for one target; returns the first complete plan.

        A candidate-independent payload check at the box's rest pose runs
        first. Phase 1 screens every candidate cheaply, so a blocked tool pose
        never costs a full path-search budget. Phase 2 plans all-endpoints-clear
        candidates first, preserving the seal-count order inside each group.
        A target-level payload failure stops this target's search at once.
        Every failure is appended to ``failures``. No permit is issued here.
        """
        failures=[] if failures is None else failures
        rest=self.payload_rest_check(world,box);(folder/(box['id']+'-payload-rest-check.json')).write_text(json.dumps(rest))
        if rest['intersections'] or rest['outside_workspace']:
            hit=rest['intersections'][0] if rest['intersections'] else None
            reason=('TARGET_PAYLOAD_INTERSECTS_AT_REST: 箱体在原位时碰撞包络已与 '+
                    (f"{hit['id']} 相交 {hit['overlap_depth_m']*1000:.1f} mm" if hit else '现场工作范围边界相交')+
                    f"（共 {len(rest['intersections'])} 处）；带箱脱离的起点与吸取候选无关，本目标所有候选都无法脱离，未进行路径规划")
            progress(reason);failures.append(dict(target=box['id'],reason=reason,evidence=rest));return None
        try:candidates=contact_candidates(box,self.source['tool'],self.site['topology'])
        except ValueError as e:failures.append(dict(target=box['id'],reason=str(e)));return None
        screened=[];solutions=[]
        try:
            for i,candidate in enumerate(candidates):
                if candidate_filter is not None and i not in candidate_filter:continue
                if cancelled():raise ValueError('CANCELLED')
                sub=folder/(box['id']+'-'+str(i));sub.mkdir()
                progress(f'预检候选 {i+1}/{len(candidates)}：带载返回起点及接近 / 接触 / 脱离终态')
                try:screened.append((i,candidate,sub,self.screen_candidate(q,world,box,candidate,sub,cancelled,solutions)))
                except TargetBlocked:raise
                except ValueError as e:failures.append(dict(target=box['id'],candidate=i,reason=str(e)))
            order=sorted(screened,key=lambda s:s[3]['screen']['conclusion']!='CLEAR')
            clear=sum(s[3]['screen']['conclusion']=='CLEAR' for s in order)
            progress(f'预检完成：{clear} 个候选终态全部通过，{len(order)-clear} 个逆解未定论，'
                     f'{len(candidates)-len(order)} 个已排除；按此顺序完整规划')
            if int(self.site.get('parallel_candidate_lanes',1))>1 and len(order)>1:
                self.wait_check_pool()
                if len(self.checkers())>1:return self.parallel_candidates(q,world,box,order,progress,cancelled,failures)
            for rank,(i,candidate,sub,screened_candidate) in enumerate(order):
                if cancelled():raise ValueError('CANCELLED')
                say=lambda text,i=i,rank=rank:progress(f'规划候选 {i+1}（{rank+1}/{len(order)}）：{text}')
                try:result=self.plan_candidate(q,world,box,candidate,sub,screened_candidate,say,cancelled)
                except TargetBlocked:raise
                except ValueError as e:failures.append(dict(target=box['id'],candidate=i,reason=str(e)));continue
                return dict(result,index=i,candidate=candidate,screen=screened_candidate['screen'],lanes=1)
        except TargetBlocked as e:
            progress(str(e));failures.append(dict(target=box['id'],reason=str(e)))
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
            self.worker_ready(self.client.request(self.init),boxes)
            self.start_check_pool(folder)
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
                # A segment is registered only with its exact sent path checked;
                # a deferred worker waypoint check is never accepted on its own.
                if any(s['evidence'].get('sent_path_collision')!='PASS' for s in segments):raise ValueError('SENT_PATH_CHECK_MISSING')
                native_checks=sorted({s['evidence']['planner'].get('native_output_check','PASS') for s in segments})
                for segment in segments:segment['package']=self.authority.register(segment.pop('prepared'),binding,segment.pop('evidence'))
                plan=dict(binding=binding,segments=segments,target=box,candidate=candidate,release=release,
                    q_start=q.tolist(),T_world_tcp_start=observation.world_tcp_start,full_path_validated=True,
                    candidate_index=i,endpoint_screen=screen['conclusion'],candidate_lanes=found.get('lanes',1),
                    native_output_check=native_checks,
                    duration_s=sum(s['package']['prepared']['send_duration_s'] for s in segments),hardware_execution='NOT_RUN')
                (folder/'complete-plan.json').write_text(json.dumps(plan));return plan
            # Keep every concrete failure on disk; one task-status label must
            # not grow into 32 copies of the full native collision JSON.
            (folder/'planning-failures.json').write_text(json.dumps(failures,ensure_ascii=False,indent=2))
            counts=Counter(failure_category(f['reason']) for f in failures)
            summary='；'.join(f'{reason} × {count}' for reason,count in counts.items())
            first=failures[0]['reason'][:600] if failures else '没有符合所选编号的目标'
            raise ValueError(f'NO_COMPLETE_FEASIBLE_SINGLE_CARTON_PLAN: {summary}；首项：{first}；详情：planning-failures.json')
        finally:
            self.close_check_pool()
            (folder/'native-check-pool.json').write_text(json.dumps(self.pool_status,ensure_ascii=False))
            self.client.close();self.client=None
