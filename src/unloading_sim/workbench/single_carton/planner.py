"""Full physical single-carton native MTC/Pilz/OMPL plan before any actuation."""
from concurrent.futures import ThreadPoolExecutor,TimeoutError as FutureTimeout
from copy import deepcopy
from collections import Counter
from pathlib import Path
import json
import threading
import time
import numpy as np
import os
from ..controlled.planner import EmptyPlanner,ROOT,file_hash
from ..controlled.trajectory import digest,prepare
from ..coordinates import rigid
from .geometry import corners,contact_candidates,candidate_contact_check,transform,obb_overlap_depth
from .config import release_policy_evidence,start_joint_check,execution_lock,tool_robot_clearance,wrist_branch_limits,wrist_branch_violation
from .. import timing
from . import posture
from . import cycle_structure

# Successively slower Pilz LIN profiles after a full-scale failure.
LIN_RETRY_SCALES=(.5,.25,.1,.05)
# Minimal send slowdowns when interpolation would exceed a joint limit.
SEND_SLOWDOWN_STEPS=(1.,1.1,1.25,1.5,2.)
# Detach profiles after straight outward separation: (name, outward m, lift m).
DETACH_ALTERNATIVES=(('DIAGONAL_45',.04,.04),('LIFT_FIRST',.01,.05))
# Deterministic extra IK seeds sampled inside the joint limits, and how many
# distinct IK branches (closest to q_start first) get a native endpoint check.
IK_RANDOM_SEEDS=64
# User decision 2026-11-21: up to four distinct approach IK branches (was
# three); endpoint-clear branches beyond the first are kept as alternatives
# and tried in the same order when the approach path of an earlier one fails
# (field 7fc259a5: the only J5-same-sign branch was the fourth).
IK_BRANCHES_CHECKED=4
# Pilz LIN ends with a duplicate sample: the same joint position 0.01-0.1 ms
# before the final point but with a nonzero velocity (field cda4e336). A
# quintic through both reads as tens of rad/s^2 that no uniform slowdown can
# remove. Position-identical knots closer than this are merged (the later,
# final state is kept); the path geometry is unchanged and the prepared sent
# path is still densely checked.
COINCIDENT_KNOT_DT_S=1e-3
COINCIDENT_KNOT_DQ_RAD=1e-9


def merge_coincident_knots(points):
    """Drop a knot whose position equals the next one within COINCIDENT_KNOT_DT_S."""
    out=[];merged=[]
    for k,point in enumerate(points):
        if k+1<len(points):
            nxt=points[k+1]
            if (float(nxt['t'])-float(point['t'])<COINCIDENT_KNOT_DT_S and
                    max(abs(a-b) for a,b in zip(point['q'],nxt['q']))<=COINCIDENT_KNOT_DQ_RAD):
                merged.append(dict(index=k,t=float(point['t']),next_t=float(nxt['t'])));continue
        out.append(point)
    return out,merged
# A payload/world pair is target-level (candidate independent) only when it
# fails by more than the IK pose tolerance can move the attached box.
TARGET_BLOCK_MARGIN_M=1e-4
# Definite payload/world intersection depth for the pre-native geometric test.
PAYLOAD_REST_DEPTH_M=.001
# Smallest probe-path chunk handed to one native checker process.
CHECK_CHUNK_MIN_STATES=500
# Face fields the native single-carton policy reads (eco65_single_carton.h).
FACE_REQUEST_FIELDS=('center_m','normal','u','v','size_m')
# concurrent_audits scheduling estimate (seconds per probe state on one
# worker, and per request), measured on the 2026-11-22 field replays.
AUDIT_UNIT_S=dict(SAMPLED=4.5e-3,ADAPTIVE=2.5e-4,ADAPTIVE_CONTACT=6e-4)
AUDIT_REQUEST_S=.3
# Worker status for a plan whose own waypoint check the caller deferred to
# its exact sent-path check (physical single-carton scope only).
DEFERRED_OUTPUT='DEFERRED_TO_CALLER_SENT_PATH_CHECK'
# Optional unified planning deadline (config planning_deadline_s). Native work
# ends this long before the absolute deadline so its reply and the Python-side
# classification are complete when the deadline arrives.
DEADLINE_REPLY_RESERVE_S=.1
# A reply no longer awaited after the deadline is still collected for this
# long in the background, so the worker can be closed cleanly.
DEADLINE_IPC_GRACE_S=10.
DEADLINE_EXPIRED='NATIVE_ABSOLUTE_DEADLINE_EXPIRED'
# binding_hash_cache: a file changed this recently is never served from cache.
BINDING_RACY_WINDOW_NS=100_000_000


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


class PlanningBudgetExhausted(Exception):
    """The unified planning deadline passed before the search concluded.

    Deliberately not a ValueError: no candidate or target handler may record
    it as an infeasibility. ``evidence`` carries the per-target searched and
    unsearched candidate counts once ``plan_cycle`` has classified it.
    """
    def __init__(self,where,detail=''):
        super().__init__(f'PLANNING_BUDGET_EXHAUSTED:{where}'+(f':{detail}' if detail else ''))
        self.where=where;self.detail=detail;self.evidence=None


class SinglePlanner(EmptyPlanner):
    def __init__(self,config,mapping,authority,coordinate_model):
        self.task_lock=threading.RLock();self.task_generation=0;self.last_task=None
        self.record_commit_lock=threading.Lock()
        self.closed=False;self.prewarm_errors=[]
        self.site=config;self.coordinate_model=coordinate_model;self.cycle_binding=None
        self.pool=[];self.pool_status=[];self.pool_threads=[];self.pool_lock=threading.Lock();self.pool_closed=False
        self.defer_output_check=False
        # Unified planning deadline (absolute time.monotonic seconds) of the
        # current plan_cycle; None keeps every request and timeout unchanged.
        self.deadline=None;self.deadline_reference=None
        # Keyed by id(client): fakes and real clients alike, no hashing needed.
        self.ipc_executors={};self.abandoned=set();self.search_counts={};self.search_thread=None
        # Step 4 mechanisms, each off unless enabled in the config (契约 A).
        self.resident=None;self.task_start=None;self.evidence=None;self.build_lock=threading.RLock()
        self.hash_cache={};self.binding_evidence=None
        self.scene_world=None;self.scene_fingerprint=None;self.scene_handles={};self.scene_lock=threading.Lock()
        self.wrist_limits=None;self.wrist_reference=None;self.posture_policy=None
        # cycle_structure START_RELEASE_SEPARATE (user decision 2026-11-22):
        # the loaded segment ends at q_release, then withdraw + empty return.
        self.cycle_policy=cycle_structure.settings(config);self.q_release=None
        super().__init__(config['execution'],mapping,authority)
        # planning_module NATIVE: packages are digested over the canonical binary form (AGENTS 3g).
        if self.native_planning() and hasattr(authority,'use_canonical_binary_digest'):authority.use_canonical_binary_digest()
    def release_pose(self,q):
        """Where the carton is released: q_start, or q_release in START_RELEASE_SEPARATE."""
        if self.cycle_policy is None:return np.asarray(q,float)
        if self.q_release is None:raise ValueError('RELEASE_POSE_NOT_DERIVED_FOR_TASK')
        return np.asarray(self.q_release,float)
    def binary_transport(self):
        """Protocol-2 binary frames: planning_module NATIVE, or binary_transport in the PYTHON flow (2026-11-22)."""
        return self.native_planning() or bool(self.site.get('binary_transport'))
    def native_planning(self):
        """planning_module NATIVE (realtime step 7): composite native requests replayed by the unchanged Python flow."""
        return self.site.get('planning_module','PYTHON')=='NATIVE'
    def enabled(self,name):
        """A step-4 mechanism switch of the single-carton config (default off)."""
        return self.site.get(name) is True
    # -- step 4: binding hash cache (AGENTS 2026-11-16 3f) ----------------------------
    @staticmethod
    def file_identity(path):
        try:s=os.stat(path)
        except OSError:return None
        return (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
    @staticmethod
    def cacheable(before,after):
        """Reusable only when unchanged during hashing and last changed well before it.

        File timestamps have coarse granularity: a second change within the
        same tick as a change just before hashing could keep size, mtime and
        ctime. Such recent ("racy") files are rehashed every time instead.
        """
        return (before is not None and before==after and
                time.time_ns()-max(after[3],after[4])>BINDING_RACY_WINDOW_NS)
    def track(self,path,expected=None):
        if not self.enabled('binding_hash_cache'):return super().track(path,expected)
        path=Path(path).resolve();before=self.file_identity(path);value=file_hash(path);after=self.file_identity(path)
        if expected is not None and value!=expected:raise ValueError('ASSET_HASH_MISMATCH:'+str(path))
        self.files[str(path)]=value
        # Only a file unchanged during its own hashing gets a reusable entry.
        if self.cacheable(before,after):self.hash_cache[str(path)]=(after,value)
        else:self.hash_cache.pop(str(path),None)
        return path
    def current_file_hashes(self):
        if not self.enabled('binding_hash_cache'):return super().current_file_hashes()
        current={};rows=[];started=time.monotonic()
        for p in self.files:
            key=self.file_identity(p);cached=self.hash_cache.get(p)
            if key is not None and cached is not None and cached[0]==key:
                current[p]=cached[1];status='REUSED'
            else:
                # Any difference of device, inode, size, mtime_ns or ctime_ns: full rehash.
                current[p]=file_hash(p);after=self.file_identity(p);status='REHASHED'
                if self.cacheable(key,after):self.hash_cache[p]=(after,current[p])
                else:self.hash_cache.pop(p,None)
            rows.append(dict(file=p,status=status,bytes=None if key is None else key[2]))
        self.binding_evidence=dict(policy='REUSE_WHILE_DEV_INO_SIZE_MTIME_NS_CTIME_NS_UNCHANGED',
            reused=sum(r['status']=='REUSED' for r in rows),rehashed=sum(r['status']=='REHASHED' for r in rows),
            seconds=time.monotonic()-started,files=rows)
        return current
    # -- unified planning deadline -------------------------------------------------
    def native_left(self):
        """Seconds of native work left before the reply reserve; None without a deadline."""
        return None if self.deadline is None else self.deadline-DEADLINE_REPLY_RESERVE_S-time.monotonic()
    def deadline_fields(self,request,budget):
        """The worker's request budget and absolute deadline, both derived from the unified deadline."""
        left=self.native_left()
        if left<=0:raise PlanningBudgetExhausted('BEFORE_DISPATCH',str(request.get('op')))
        return dict(request,request_budget_s=min(budget,left),
                    absolute_deadline_monotonic_s=self.deadline-DEADLINE_REPLY_RESERVE_S,
                    deadline_clock_reference_monotonic_s=self.deadline_reference)
    def bounded_request(self,client,payload,timeout,cancelled,name):
        """``client.request`` that returns or is classified by the unified deadline.

        The request runs on a per-client IPC thread. At the deadline the caller
        gets PlanningBudgetExhausted at once; the client is never used again
        and its late reply is collected in the background.
        """
        deadline=self.deadline;left=deadline-time.monotonic()
        if left<=0:raise PlanningBudgetExhausted('BEFORE_DISPATCH',name)
        with self.pool_lock:
            entry=self.ipc_executors.get(id(client))
            if entry is None:
                entry=self.ipc_executors[id(client)]=(client,ThreadPoolExecutor(max_workers=1,thread_name_prefix='native-ipc'))
        stop=cancelled
        if hasattr(client,'retire'):
            # Resident worker: cancel this request by id at the deadline; the
            # worker replies and stays usable for later tasks.
            stop=lambda:(cancelled is not None and cancelled()) or time.monotonic()>=deadline
        future=entry[1].submit(timing.native_request,client,payload,timeout=min(timeout,left+DEADLINE_IPC_GRACE_S),
                               cancelled=stop,record=name)
        try:return future.result(timeout=max(0.,deadline-time.monotonic()))
        except FutureTimeout:
            with self.pool_lock:self.abandoned.add(id(client))
            if cancelled is not None and cancelled():raise RuntimeError('CANCELLED')
            raise PlanningBudgetExhausted('NO_REPLY_BY_DEADLINE',name) from None
    def bounded_search(self,q,world,box,folder,progress,cancelled,failures,candidate_filter=None):
        """``search_box`` on a helper thread; the caller is released at the deadline.

        Pure-Python steps between native calls (candidate generation, send-grid
        preparation, record writes) cannot be preempted. At the deadline the
        caller classifies the expiry while the helper finishes its current
        step; its next native call is refused before dispatch, it never
        registers a permit, and the next plan_cycle waits for it to end.
        """
        outcome={}
        def run():
            try:outcome['found']=self.search_box(q,world,box,folder,progress,cancelled,failures,candidate_filter)
            except BaseException as error:outcome['error']=error
        thread=self.search_thread=threading.Thread(target=run,daemon=True,name='target-search')
        thread.start();thread.join(max(0.,self.deadline-time.monotonic()))
        if thread.is_alive():raise PlanningBudgetExhausted('SEARCH_STEP_RUNNING_AT_DEADLINE',box['id'])
        if 'error' in outcome:raise outcome['error']
        return outcome['found']
    def deadline_outcome(self,result,solver_capped):
        """Whether a native reply ended because the unified deadline expired."""
        if not isinstance(result,dict):return False
        text=json.dumps({k:result.get(k) for k in ('status','failure','path_failure','detail')},default=str)
        if DEADLINE_EXPIRED in text:return True
        # A solver budget is global only when the deadline, not the per-call
        # configured per-call cap, set it; per-call exhaustion stays distinct.
        return result.get('status')=='NATIVE_SOLVER_BUDGET_EXHAUSTED' and (
            solver_capped or time.monotonic()>=self.deadline-DEADLINE_REPLY_RESERVE_S-1e-3)
    def build_world(self,s):
        # Source snapshot supplies robot/tool/camera solids only. Its trailer,
        # base placement, conveyors, floor, lift and cartons never enter real IO.
        return []
    def build(self):
        # A background prewarm (resident_workers) and observe may both build.
        with self.build_lock:return self._build()
    def _build(self):
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
        trc=tool_robot_clearance(self.site,self.clearance.get('tool_links',[]))
        if trc:self.clearance['tool_robot_clearance']=trc
        else:self.clearance.pop('tool_robot_clearance',None)
        self.wrist_limits=wrist_branch_limits(self.site)
        self.posture_policy=posture.bind(self.site,self.coordinate_model)
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
        if box.get('uncertainty',{}).get('bound_method')=='RGBD_VISIBILITY_INTERVALS':return []
        if (box.get('uncertainty',{}).get('kind')=='MEASURED_BOUNDARY_INTERVALS'
                and not box.get('quality',{}).get('observed_support_plane')
                and not box.get('quality',{}).get('confirmed_stack_support_clip')):return []
        bottom=min(np.asarray(box['corners_m'])[:,2]);supports=[]
        for o in world:
            if o['id']==box['id']:continue
            if o.get('category')=='confirmed_stack_support':
                if o.get('supports_target_id')==box['id']:
                    plane=o['support_plane'];n=np.asarray(plane['normal_world']);d=plane['offset_m']
                    if abs(float(np.min(np.asarray(box['corners_m'])@n))-d)>.0004:
                        raise ValueError('STACK_SUPPORT_NOT_BOUND_TO_TARGET_BOTTOM')
                    supports.append(dict(id=o['id'],plane=plane))
                continue
            if o.get('category') in ('unresolved_carton_volume','neighbor_shadow'):continue
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
    def endpoint_ik(self,pose,seeds,folder,name,cancelled=None,q_reference=None,client=None):
        """Distinct native IK solutions; optional soft posture, then continuity.

        One record per pose: the request and all seeds are persisted before
        dispatch, every reply after. IK never checks collision; callers do.
        """
        template=dict(op='ik',identity=self.init['identity'],goal_pose=np.asarray(pose).tolist(),seed=6518,planning_only=False,
                      position_tolerance_m=2e-5,orientation_tolerance_rad=2e-4,request_budget_s=110.)
        if self.deadline is not None:template=self.deadline_fields(template,110.)
        path=folder/(name+'-native.json');started=time.monotonic()
        record=dict(request=template,seeds=[list(map(float,s)) for s in seeds],status='PENDING',ipc_timeout_s=120)
        timing.write_json(path,record,'ik-record:pending');replies=[];solutions=[]
        try:
            if self.site.get('native_ik_batch'):
                # One sc_ik_batch request: the worker runs the identical "ik"
                # request for every seed and returns each reply in seed order.
                if cancelled is not None and cancelled():raise ValueError('CANCELLED')
                batch=dict(template,op='sc_ik_batch',ik_seeds=[list(map(float,s)) for s in seeds])
                if self.deadline is None:
                    reply=timing.native_request(client or self.client,batch,timeout=120,cancelled=cancelled,record=name)
                else:reply=self.bounded_request(client or self.client,batch,120,cancelled,name)
                if reply.get('status')=='NATIVE_ABSOLUTE_DEADLINE_EXPIRED':raise PlanningBudgetExhausted('NATIVE_DEADLINE',name)
                if reply.get('status')!='SUCCESS' or len(reply.get('results') or [])!=len(seeds):raise RuntimeError('SC_IK_BATCH_FAILED:'+str(reply)[:200])
                answers=reply['results']
            else:answers=None
            for k,seed in enumerate(seeds):
                if answers is not None:reply=answers[k]
                else:
                    if cancelled is not None and cancelled():raise ValueError('CANCELLED')
                    request=dict(template,q_start=list(map(float,seed)))
                    if self.deadline is None:
                        reply=timing.native_request(client or self.client,request,timeout=120,cancelled=cancelled,record=name)
                    else:reply=self.bounded_request(client or self.client,request,120,cancelled,name)
                replies.append(dict(status=reply.get('status'),q=reply.get('q'),position_error_m=reply.get('position_error_m'),
                                    orientation_error_rad=reply.get('orientation_error_rad')))
                if reply.get('status')=='SUCCESS':
                    q=list(map(float,reply['q']))
                    if not any(np.max(abs(np.asarray(q)-x))<1e-3 for x in solutions):solutions.append(q)
            reference=np.asarray(seeds[0] if q_reference is None else q_reference,float)
            solutions.sort(key=lambda x:posture.key(self.posture_policy,x,reference))
            if self.posture_policy:
                record['posture_ranking']=[dict(q=x,**posture.metrics(self.posture_policy,x,reference)) for x in solutions]
            # Apply the explicitly configured wrist policy relative to frozen
            # q_start. The user may disable J5 flip/band without removing travel limits.
            if self.wrist_limits and self.wrist_reference is not None:
                dropped=[x for x in solutions if wrist_branch_violation(self.wrist_limits,self.wrist_reference,[x])]
                solutions=[x for x in solutions if x not in dropped];record['wrist_branch_rejected']=dropped
            record.update(status='REPLIED',replies=replies,distinct_solutions=solutions,elapsed_s=time.monotonic()-started)
            return solutions,[r['status'] for r in replies]
        except PlanningBudgetExhausted as error:
            record.update(status='PLANNING_BUDGET_EXHAUSTED',error=str(error),replies=replies,
                          unsent_seeds=len(seeds)-len(replies),elapsed_s=time.monotonic()-started);raise
        except Exception as error:
            record.update(status='REQUEST_FAILED',error=str(error),replies=replies,last_cancellation=getattr(client or self.client,'last_cancellation',None),elapsed_s=time.monotonic()-started);raise
        finally:self.write_evidence(path,record,'ik-record:final')
    def payload_world_blocked(self,failure,box,world):
        """A payload/world failure bigger than the IK tolerance, at the box's rest pose."""
        pair=list((failure or {}).get('pair') or []);others={o['id'] for o in world}-{box['id']}
        if len(pair)!=2 or box['id'] not in pair or not set(pair)-{box['id']}<=others:return False
        distance=failure.get('surface_distance_m');required=failure.get('required_pair_clearance_m',0.)
        return isinstance(distance,(int,float)) and (distance<0 or required-distance>TARGET_BLOCK_MARGIN_M)
    def target_blocked_message(self,failure,box,source):
        pair=failure['pair'];other=pair[1] if pair[0]==box['id'] else pair[0];distance=failure.get('surface_distance_m')
        gap=f'间隙 {distance*1000:.2f} mm（要求 {failure.get("required_pair_clearance_m",0)*1000:.1f} mm）' if distance>=0 else '相交'
        if box.get('uncertainty') or other.startswith(('rgbd_carton_possible_','rgbd_neighbor_shadow_')):
            return (f'GEOMETRY_UNCERTAINTY_BLOCKED_AT_REST: 保守不确定性包络与 {other} {gap}；'
                    f'当前观测尚不能证明带载路径安全，不代表实物相交或全部箱体假设均无解（{source}）')
        return (f'TARGET_PAYLOAD_BLOCKED_AT_REST: 箱体在原位时碰撞包络与 {other} {gap}；带箱脱离的起点与吸取候选无关，'
                f'本目标所有候选都无法脱离，停止搜索（{source}）')
    def payload_rest_check(self,world,box):
        """Candidate-independent payload geometry at the box's rest pose.

        At contact the attached collision object is this complete envelope,
        whatever the grasp. An intersection blocks planning with that envelope,
        but does not prove every underlying geometry hypothesis infeasible.
        Only model overlaps deeper than PAYLOAD_REST_DEPTH_M are reported here;
        anything closer is left to the native clearance checks.
        """
        supports={s['id'] for s in self.detach_supports(world,box)};hits=[]
        for o in world:
            if o['id']==box['id'] or o['id'] in supports or 'size' not in o:continue
            depth=obb_overlap_depth(box['pose'],np.asarray(box['size_m'])/2,o['pose'],np.asarray(o['size'])/2)
            if depth>PAYLOAD_REST_DEPTH_M:hits.append(dict(id=o['id'],category=o.get('category'),overlap_depth_m=depth,
                geometry_uncertain=bool(o.get('geometry_uncertain') or o.get('category') in {'unresolved_carton_volume','neighbor_shadow'})))
        bounds=(self.site.get('site') or {}).get('workspace_bounds_m')
        outside=False
        if bounds:
            lo,hi=np.asarray(bounds,float);c=np.asarray(box['corners_m'],float)
            outside=bool(np.any(c<lo) or np.any(c>hi))
        result=dict(intersections=sorted(hits,key=lambda h:-h['overlap_depth_m']),outside_workspace=outside,supports=sorted(supports))
        if box.get('uncertainty') or any(h['geometry_uncertain'] for h in hits):
            result.update(geometry_basis='CONSERVATIVE_UNCERTAINTY_ENVELOPE',
                          measured_volume_overlap_proven=False,all_hypotheses_infeasible_proven=False)
        return result
    def endpoint_precheck(self,q,world,box,candidate,attachment,folder,cancelled=None,solutions=None,
                          branches_override=None,label='',client=None):
        """Classify a candidate's task endpoints before any path search.

        Tool solids are rigid on the flange and the attached box is rigid on the
        tool, so their world collisions at a task pose do not depend on the IK
        branch: a (tool or payload) vs world failure rejects the candidate, and
        a payload vs world failure at detach rejects the whole target. IK is
        searched from many deterministic seeds; up to IK_BRANCHES_CHECKED
        distinct approach branches (optional soft posture, then continuity)
        are checked.
        Contact and detach share one TCP pose, so detach reuses the contact
        configuration with the payload attached. Returns CLEAR with checked
        approach/contact configurations, or NOT_CONCLUSIVE with the reason.
        """
        tool=set(self.clearance.get('tool_links',[]));ids={o['id'] for o in world}
        solutions=[] if solutions is None else solutions;supports=self.detach_supports(world,box)
        record=[];notes=[]
        def check(stage,qe,attached,stage_supports):
            result=self.native_call(dict(self.context(qe,world,box,candidate,stage,attached,stage_supports),op='validate'),
                                    folder,stage+'-endpoint'+label+('' if not notes else f'-b{len(notes)}'),120,cancelled,client)
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
            if branches_override is None:
                branches,attempts=self.endpoint_ik(candidate['precontact_pose'],self.ik_seeds(q,solutions)+self.spread_seeds(),
                                                   folder,'approach-endpoint-ik',cancelled,q,client)
            else:branches,attempts=[list(b) for b in branches_override],[]
            if not branches:
                record.append(dict(stage='approach',ik=dict(seeds=len(attempts),solutions=0),conclusion='IK_NOT_FOUND_NOT_CONCLUSIVE'))
                return dict(conclusion='NOT_CONCLUSIVE',reason='approach_IK_NOT_FOUND',record=record)
            checked=branches[:IK_BRANCHES_CHECKED]
            for rank,qa in enumerate(checked):
                if not check('approach',qa,None,()):notes.append('approach_ARM_CONFIGURATION_REJECTED');continue
                contacts,_=self.endpoint_ik(candidate['tcp_pose'],[qa],folder,'contact-endpoint-ik'+label+('' if not notes else f'-b{len(notes)}'),cancelled,qa,client)
                if not contacts:
                    record.append(dict(stage='contact',ik='NATIVE_IK_FAILED',conclusion='IK_NOT_FOUND_NOT_CONCLUSIVE'));notes.append('contact_IK_NOT_FOUND');continue
                qc=contacts[0]
                if not check('contact',qc,None,()):notes.append('contact_ARM_CONFIGURATION_REJECTED');continue
                if not check('detach',qc,attachment,supports):notes.append('detach_ARM_CONFIGURATION_REJECTED');continue
                if branches_override is None:solutions.append(qa)
                # Unchecked later branches (same order) stay available for a
                # fallback when this branch's approach path fails (2026-11-21).
                return dict(conclusion='CLEAR',approach_q=qa,contact_q=qc,ik_branches=len(branches),
                            pending_branches=[list(b) for b in checked[rank+1:]],record=record)
            return dict(conclusion='NOT_CONCLUSIVE',reason=notes[-1] if notes else 'NO_CHECKED_BRANCH',ik_branches=len(branches),record=record)
        finally:self.write_evidence(folder/f'endpoint-precheck{label}.json',record,'endpoint-precheck.json',ensure_ascii=False)
    def context(self,q,world,box,candidate,stage,attachment=None,supports=()):
        # The withdraw LIN off a released carton (START_RELEASE_SEPARATE) is the
        # contact stage reversed and keeps exactly its permissions and guards.
        loaded=attachment is not None;contact=stage in ('contact','withdraw')
        contact_ids=candidate['target_lip_contact_ids']
        lips=[self.contact_geometry['terminal_band_by_contour'][i] for i in contact_ids]
        pairs=[['base_link','robot_mount_plate']]
        if contact or loaded:pairs += [[box['id'],lip] for lip in lips]
        pairs += [[box['id'],s['id']] for s in supports]
        attached=None if not loaded else dict(id=box['id'],size=box['size_m'],pose=attachment.tolist(),touch_links=lips)
        policy=dict(self.clearance,stage=stage,target_id=box['id'],payload_id=box['id'] if loaded else '')
        # compact_requests (2026-11-22): the native policy reads only these face
        # and measured-box fields; the observed support points and the rest
        # stay in the plan's target/candidate records, not in every request.
        compact=bool(self.site.get('compact_requests'))
        face=candidate['face'] if not compact else {k:candidate['face'][k] for k in FACE_REQUEST_FIELDS}
        spec=dict(geometry=self.contact_geometry,target_id=box['id'],size_m=box['size_m'],target_pose=box['pose'],
                  face=face,active_ids=candidate['active_ids'],target_lip_contact_ids=contact_ids,
                  allow_contact=contact,supports=list(supports))
        if box.get('uncertainty'):spec['geometry_uncertainty']=box['uncertainty']
        measured=box.get('quality') or {}
        if not box.get('observed_side_geometry') and measured.get('observed_side_envelope_source')=='ALL_ACCEPTED_SIDE_PLANE_SAMPLES_NO_SIZE_PRIOR':
            spec['observed_side_envelope']=dict(source=measured['observed_side_envelope_source'],
                front_dimensions_m=measured['front_face_and_depth_dimensions_m'],
                expansion_m=measured['observed_side_envelope_expansion_m'])
        if box.get('measured_box'):
            spec['measured_box']=box['measured_box'] if not compact else {k:box['measured_box'][k] for k in ('pose','size_m')}
        if self.enabled('scene_handles') and world is self.scene_world:
            # The task's world was hashed once when its scene was registered.
            fingerprint=self.scene_fingerprint
        else:
            with timing.span('context.digest_world',stage=stage,objects=len(world)):fingerprint=digest(world)
        context=dict(identity=self.init['identity'],q_start=list(q),start_velocity=[0.]*6,path_constraints=[],
            scene_fingerprint=fingerprint,world=world,allowed_pairs=pairs,attachment=attached,clearance_policy=policy,
            single_carton_contact=spec,stage=stage,planning_only=False,seed=6518)
        if self.site.get('site'):context['site_workspace_bounds_m']=self.site['site']['workspace_bounds_m']
        if self.site.get('collision_check_mode')=='TWO_LEVEL':context['collision_check_mode']='TWO_LEVEL'
        return context
    # -- step 4: evidence writer and scene handles --------------------------------------
    def write_evidence(self,path,value,name,**options):
        """A finished record: through the async writer when enabled, else written now."""
        if self.evidence is None:return timing.write_json(path,value,name,**options)
        self.evidence.write_json(path,value,name,**options)
    def register_scene(self,world,folder):
        """Hash the task's world once and record it; workers get it uploaded on first use."""
        with timing.span('context.digest_world',stage='register_scene',objects=len(world)):fingerprint=digest(world)
        with self.scene_lock:self.scene_world,self.scene_fingerprint,self.scene_handles=world,fingerprint,{}
        timing.write_json(folder/'native-scene.json',dict(scene_fingerprint=fingerprint,world=world,
            scope='ONE_UPLOAD_PER_WORKER_PER_TASK_REQUESTS_CARRY_THE_HANDLE'),'native-scene.json')
        return fingerprint
    def scene_handle(self,client,folder,cancelled=None):
        """This worker's handle for the task scene, uploading it once (thread safe per worker)."""
        client=getattr(client,'target',client)  # a replay client (planning_module NATIVE) uses its worker's handle
        key=id(client)
        with self.scene_lock:
            entry=self.scene_handles.get(key)
            if entry is None:entry=self.scene_handles[key]=dict(lock=threading.Lock(),handle=None)
        with entry['lock']:
            if entry['handle'] is None:
                upload=dict(op='sc_scene_upload',identity=self.init['identity'],world=self.scene_world,
                            scene_fingerprint=self.scene_fingerprint)
                started=time.monotonic()
                kwargs=dict(timeout=120,cancelled=cancelled)
                if self.deadline is not None:
                    left=self.native_left()
                    if left<=0:raise PlanningBudgetExhausted('BEFORE_DISPATCH','sc_scene_upload')
                    upload.update(request_budget_s=left);kwargs['timeout']=left+DEADLINE_IPC_GRACE_S
                reply=timing.native_request(client,upload,record='scene-upload',**kwargs)
                if reply.get('status')!='SCENE_READY' or reply.get('scene_fingerprint')!=self.scene_fingerprint:
                    raise RuntimeError('SCENE_UPLOAD_FAILED:'+str(reply)[:300])
                entry['handle']=reply['scene_handle']
                self.write_evidence(folder/f'scene-upload-{getattr(client,"name",key)}.json',dict(
                    scene_fingerprint=self.scene_fingerprint,scene_handle=reply['scene_handle'],objects=reply.get('objects'),
                    upload_s=time.monotonic()-started),'scene-upload.json')
            return entry['handle']
    def start_scene_uploads(self,folder,cancelled=None):
        """early_scene_upload: upload the task scene to every live worker now, in parallel.

        The same one-upload-per-worker handles that the first request of each
        worker would otherwise create lazily; requests wait for their worker's
        upload through the per-worker lock in scene_handle. Errors surface at
        that worker's first request (scene_handle uploads again or raises).
        """
        if self.scene_world is None:return
        def one(client):
            try:
                with timing.span('scene.early_upload',worker=getattr(client,'name',None)):self.scene_handle(client,folder,cancelled)
            except Exception:
                pass
        for client in self.checkers():
            threading.Thread(target=one,args=(client,),daemon=True,name='scene-upload').start()
    def with_scene_handle(self,request,client,folder,cancelled=None):
        """The request with this task's world replaced by the worker's scene handle."""
        if (self.scene_world is None or request.get('world') is not self.scene_world or
                request.get('scene_fingerprint')!=self.scene_fingerprint):return request
        handle=self.scene_handle(client,folder,cancelled)
        return {**{k:v for k,v in request.items() if k!='world'},'scene_handle':handle}
    def native_call(self,request,folder,name,timeout,cancelled=None,client=None,record_view=None):
        if self.enabled('scene_handles'):request=self.with_scene_handle(request,client or self.client,folder,cancelled)
        if self.deadline is not None:return self.deadline_native_call(request,folder,name,timeout,cancelled,client,record_view)
        # Persist before dispatch: a timed-out worker cannot return its request
        # or profiling data. Keep every Pilz/OMPL attempt instead of overwriting.
        request=dict(request,request_budget_s=timeout-10.)
        path=folder/(name+'-native.json');started=time.monotonic()
        record=dict(request=request if record_view is None else record_view(request),status='PENDING',ipc_timeout_s=timeout)
        timing.write_json(path,record,'native-record:pending')
        try:
            result=timing.native_request(client or self.client,request,timeout=timeout,cancelled=cancelled,record=name)
            record.update(status='REPLIED',result=result,elapsed_s=time.monotonic()-started)
            return result
        except Exception as error:
            record.update(status='REQUEST_FAILED',error=str(error),last_cancellation=getattr(client or self.client,'last_cancellation',None),elapsed_s=time.monotonic()-started)
            raise RuntimeError(f'{name}:{error}；详情：{path.name}') from error
        finally:self.write_evidence(path,record,'native-record:final')
    def deadline_native_call(self,request,folder,name,timeout,cancelled=None,client=None,record_view=None):
        """``native_call`` under the unified deadline.

        Request budget, solver time and IPC wait all derive from the one
        absolute deadline, which the worker receives as well. The request is
        persisted before dispatch as before. An expiry raises
        PlanningBudgetExhausted, never a rejection of this candidate.
        """
        client=client or self.client
        capped=False
        if 'allowed_planning_time_s' in request:
            left=self.native_left()
            capped=left<request['allowed_planning_time_s']
            if capped:request=dict(request,allowed_planning_time_s=max(left,1e-3))
        request=self.deadline_fields(request,timeout-10.)
        path=folder/(name+'-native.json');started=time.monotonic()
        record=dict(request=request if record_view is None else record_view(request),status='PENDING',ipc_timeout_s=min(timeout,self.deadline-started+DEADLINE_IPC_GRACE_S),
                    planning_deadline=dict(absolute_monotonic_s=self.deadline,reply_reserve_s=DEADLINE_REPLY_RESERVE_S,
                                           solver_budget_from_deadline=capped))
        timing.write_json(path,record,'native-record:pending')
        try:
            result=self.bounded_request(client,request,timeout,cancelled,name)
            record.update(status='REPLIED',result=result,elapsed_s=time.monotonic()-started)
            if self.deadline_outcome(result,capped):
                record['status']='PLANNING_BUDGET_EXHAUSTED'
                raise PlanningBudgetExhausted('NATIVE_DEADLINE',name)
            return result
        except PlanningBudgetExhausted as error:
            record.update(status='PLANNING_BUDGET_EXHAUSTED',error=str(error),last_cancellation=getattr(client,'last_cancellation',None),elapsed_s=time.monotonic()-started);raise
        except Exception as error:
            record.update(status='REQUEST_FAILED',error=str(error),last_cancellation=getattr(client or self.client,'last_cancellation',None),elapsed_s=time.monotonic()-started)
            if DEADLINE_EXPIRED in str(error):raise PlanningBudgetExhausted('NATIVE_DEADLINE',name) from error
            raise RuntimeError(f'{name}:{error}；详情：{path.name}') from error
        finally:self.write_evidence(path,record,'native-record:final')
    # -- resident checker pool (final sent-path checks and concurrent plans) ----
    def worker_ready(self,ready,boxes=()):
        """Required capabilities of the main native worker; binds output-check deferral."""
        capabilities=ready.get('capabilities',{})
        if not capabilities.get('bounded_single_carton_planning'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_BOUNDED_SINGLE_CARTON_PLANNING')
        if any(b.get('uncertainty') for b in boxes) and not capabilities.get('partial_observed_contact_envelope'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_PARTIAL_OBSERVED_CONTACT')
        if any(not b.get('observed_side_geometry') and (b.get('quality') or {}).get('observed_side_envelope_source') for b in boxes) and not capabilities.get('observed_side_envelope_v1'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_OBSERVED_SIDE_ENVELOPE')
        from .stack_support import policy as stack_policy
        if stack_policy(getattr(self,'site',{}).get('geometry')) and not capabilities.get('inclined_carton_support_v1'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_INCLINED_CARTON_SUPPORT')
        self.defer_output_check=bool(capabilities.get('single_carton_output_check_deferral'))
        if 'tool_robot_clearance' in self.clearance and not capabilities.get('single_carton_tool_robot_clearance'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_TOOL_ROBOT_CLEARANCE')
        if getattr(self,'posture_policy',None) and self.native_planning() and not capabilities.get('single_carton_posture_preference'):
            raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_POSTURE_PREFERENCE')
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
        if self.deadline is not None and count:
            # A worker whose start would end after the deadline (estimated by
            # this plan's measured main-worker init) adds no parallelism, only
            # process starts competing with the deadline classification.
            estimate=getattr(self,'worker_init_s',None);left=self.deadline-time.monotonic()
            if estimate is not None and left<estimate:
                self.pool_status=[dict(index=k,state='NOT_STARTED',reason='DEADLINE_BEFORE_MEASURED_WORKER_INIT',
                                       remaining_s=left,measured_init_s=estimate) for k in range(count)]
                self.pool_closed=False;return
        def start(k):
            status=dict(index=k,state='STARTING');self.pool_status.append(status);client=None
            try:
                with timing.span('native.pool_worker_process_start',index=k):
                    client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(folder),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],
                                                log_path=folder/f'moveit-check-{k}.log',binary=self.binary_transport())
                # Under a deadline the IPC wait also derives from it; this
                # background start never blocks the planning result.
                init_timeout=180 if deadline is None else max(1e-3,min(180.,deadline-time.monotonic()+DEADLINE_IPC_GRACE_S))
                ready=timing.native_request(client,self.init,timeout=init_timeout,record=f'pool-{k}-init')
                if not ready.get('capabilities',{}).get('bounded_single_carton_planning'):raise RuntimeError('NATIVE_WORKER_CAPABILITY_MISSING')
                if deadline is None:
                    with self.pool_lock:
                        if self.pool_closed:client.close();status['state']='CLOSED';return
                        self.pool.append(client)
                else:
                    # Under a deadline: a worker whose plan already ended (or a
                    # later plan began) never joins a pool, and is closed
                    # outside the lock so the 5 s close never blocks a plan.
                    with self.pool_lock:
                        stale=self.pool_closed or generation!=self.pool_generation
                        if not stale:self.pool.append(client)
                    if stale:status['state']='CLOSED';client.close();return
                status['state']='READY'
            except Exception as error:
                status.update(state='FAILED',error=str(error))
                if client is not None:client.close()
        self.pool_closed=False;deadline=self.deadline
        with self.pool_lock:self.pool_generation=getattr(self,'pool_generation',0)+1;generation=self.pool_generation
        for k in range(count):
            thread=threading.Thread(target=start,args=(k,),daemon=True,name=f'native-check-{k}');thread.start();self.pool_threads.append(thread)
    def wait_check_pool(self,timeout=240.):
        """Wait until every pool worker is READY or FAILED, so lane count is fixed."""
        deadline=time.monotonic()+timeout
        if self.deadline is not None:deadline=min(deadline,self.deadline)
        with timing.span('native.pool_wait_ready',workers=len(self.pool_threads)):
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
        # A worker whose reply was abandoned at the deadline is never reused.
        abandoned=getattr(self,'abandoned',())
        with self.pool_lock:return [c for c in [self.client]+list(self.pool)
                                    if alive(c) and id(c) not in abandoned and getattr(c,'retired',None) is None]
    # -- segments -----------------------------------------------------------------
    def plan_request(self,context,goal,linear):
        """The first native plan request of a segment (Pilz PTP or LIN)."""
        req=dict(context,op='plan',pipeline_id='pilz_industrial_motion_planner',planner_id='LIN' if linear else 'PTP',
                 velocity_scale=1.,acceleration_scale=1.,allowed_planning_time_s=self.site.get('native_solver_budget_s',25.))
        # The worker's own dense check of its linear-joint waypoint path is
        # deferred only when it supports that, because sent_check() runs the
        # identical output-phase check on the exact sent path (all waypoints
        # included) before any segment can be registered.
        if self.defer_output_check:req['defer_output_check_to_sent_path']=True
        # Optional (2026-11-22): the worker checks the planner's returned
        # waypoints with this request's clearance predicate only; MoveIt's mesh
        # intersection test there is implied by positive clearance (worker.cpp).
        if self.site.get('solution_path_check','PIPELINE')=='CLEARANCE_PREDICATE':req['solution_path_check']='CLEARANCE_PREDICATE'
        if np.asarray(goal).shape==(4,4):req.update(goal_pose=np.asarray(goal).tolist(),flange_from_task_tcp=self.init['identity']['flange_from_task_tcp'])
        else:req['q_goal']=list(goal)
        return req
    def plan_segment(self,context,goal,stage,scale,folder,progress,linear=False,cancelled=None,record=None,client=None):
        """Native plan and send-grid preparation, without the final dense check."""
        record=record or stage;progress('原生规划 '+record)
        req=self.plan_request(context,goal,linear)
        # planning_module NATIVE: the replay client serves the worker's own prepare results.
        prepare_segment=getattr(client,'prepare_segment',None) or prepare
        if prepare_segment is prepare and self.site.get('sent_path_probe','SENT_GRID')=='GEOMETRIC':
            # Optional geometric probe grid (PYTHON flow): same 1 mm pair bound, per knot interval.
            prepare_segment=lambda *a:prepare(*a,probe_grid='GEOMETRIC')
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
            # Optional OMPL-only budget (user decision 2026-11-21, 5 s in the field);
            # absent keeps the Pilz request's budget, i.e. native_solver_budget_s.
            req.update(pipeline_id='ompl',planner_id='RRTConnectkConfigDefault',
                       allowed_planning_time_s=float(self.site.get('native_ompl_budget_s',req['allowed_planning_time_s'])))
            progress(f'{stage} 直连未通过（{result.get("status")}），正在搜索避让路径（{req["allowed_planning_time_s"]:g} 秒预算）')
            result=self.native_call(req,folder,record+'-ompl',120,cancelled,client)
        self.write_evidence(folder/(record+'-native.json'),dict(request=req,result=result),'segment-native.json')
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
        def prepare_sent(result):
            points,merged=merge_coincident_knots(result['points'])
            if merged:result['coincident_knots_merged']=merged
            for factor in SEND_SLOWDOWN_STEPS:
                try:
                    with timing.span('plan.prepare_send_grid',stage=stage,record=record,factor=factor,source_points=len(points),candidate=getattr(folder,'name',None)):
                        return prepare_segment(points,self.limits,scale*factor,self.config['period_s'],self.reach_bound+payload_reach)
                except ValueError as error:
                    if not str(error).startswith(('INTERPOLATION_LIMIT','DISCRETE_COMMAND_LIMIT')) or factor==SEND_SLOWDOWN_STEPS[-1]:raise
                    progress(f'{record} 插值后超出关节限值（{error}），发送时间放大到 ×{scale*SEND_SLOWDOWN_STEPS[SEND_SLOWDOWN_STEPS.index(factor)+1]:g}')
        try:prepared=prepare_sent(result)
        except ValueError as error:
            # A Cartesian LIN profile that still overshoots between knots after
            # uniform slowdown is planned again natively at a lower velocity /
            # acceleration scale (same scene, same line, same limits); its sent
            # trajectory is rechecked in full like every other segment.
            if not linear or not str(error).startswith(('INTERPOLATION_LIMIT','DISCRETE_COMMAND_LIMIT')):raise
            prepared=None
            for factor in [f for f in LIN_RETRY_SCALES if f<req['velocity_scale']]:
                req.update(velocity_scale=factor,acceleration_scale=factor)
                progress(f'{stage} 插值后仍超出关节限值（{error}），以 {factor:g} 倍速度/加速度重新原生规划')
                retry=self.native_call(req,folder,f'{record}-pilz-limit-s{factor:g}',120,cancelled,client)
                if retry.get('status')!='SUCCESS' or retry.get('native_output_status') not in accepted:continue
                try:prepared=prepare_sent(retry)
                except ValueError as again:
                    if not str(again).startswith(('INTERPOLATION_LIMIT','DISCRETE_COMMAND_LIMIT')):raise
                    error=again;continue
                result=retry;self.write_evidence(folder/(record+'-native.json'),dict(request=req,result=result),'segment-native.json')
                break
            if prepared is None:raise error
        if self.wrist_limits and self.wrist_reference is not None:
            violation=wrist_branch_violation(self.wrist_limits,self.wrist_reference,prepared['collision_path'])
            if violation:raise SegmentRejected(stage,stage+':WRIST_BRANCH_REJECTED:'+json.dumps(violation),dict(status='WRIST_BRANCH_REJECTED',failure=violation))
        planner=dict(pipeline_id=req['pipeline_id'],planner_id=req['planner_id'],velocity_scale=req['velocity_scale'],
                     acceleration_scale=req['acceleration_scale'],goal='JOINT' if 'q_goal' in req else 'POSE',
                     configured_time_scale=scale,send_time_scale=prepared['time_scale'],
                     native_output_check='DEFERRED_TO_SENT_PATH_CHECK' if result['native_output_status']==DEFERRED_OUTPUT else 'PASS')
        segment=dict(stage=stage,record=record,prepared=prepared,context=context,native_result=result,planner=planner)
        # pipelined_audits: the scheduler queues this segment's unchanged audit now.
        hook=getattr(self,'segment_ready_hook',None)
        if hook is not None:hook(folder,segment)
        return segment
    def sent_check_chunks(self,segment,folder,count):
        """The sent-path audit of one segment as ``count`` contiguous chunk requests.

        Chunks share their boundary state, so every edge of the probe path, and
        every sample the native checker inserts on it, is checked by exactly one
        request with the identical scene and policy. With compact_requests the
        probe path file is written here, before any chunk is dispatched.
        Returns (audit method, [(start state, end state, record name, request, native_call options)]).
        """
        record,context=segment['record'],segment['context']
        path=segment['prepared']['collision_path']
        edges=np.linspace(0,len(path)-1,count+1).round().astype(int)
        audit=self.audit_method(context)
        view=None
        if self.site.get('compact_requests'):
            # The exact probe path is persisted once per segment, before any
            # chunk is dispatched (float64 bit patterns, with its SHA-256);
            # each chunk record names its slice of that file.
            import hashlib,io
            buffer=io.BytesIO();np.save(buffer,np.asarray(path,dtype='<f8'));data=buffer.getvalue()
            probe_file=record+'-probe-path.npy';probe_sha=hashlib.sha256(data).hexdigest()
            with timing.span('sent_check.probe_path_file',record=record,bytes=len(data)):(folder/probe_file).write_bytes(data)
            def view(request):
                out={k:v for k,v in request.items() if k!='probe_path'}
                out['probe_path_ref']=dict(file=probe_file,sha256=probe_sha,start_state=request['_chunk'][0],end_state=request['_chunk'][1],
                                           format='NUMPY_FLOAT64_LITTLE_ENDIAN_STATES_X_JOINTS')
                out.pop('_chunk',None);return out
        chunks=[]
        for k in range(count):
            a,b=int(edges[k]),int(edges[k+1]);name=record+'-sent'+('' if count==1 else f'-p{k}')
            check=dict(context,op='validate',probe_path=path[a:b+1])
            if self.enabled('path_endpoint_reuse'):check['path_endpoint_reuse']=True
            if audit in ('ADAPTIVE','ADAPTIVE_CONTACT'):check['sent_path_audit']=audit
            chunk_view=None if view is None else (lambda request,a=a,b=b:view(dict(request,_chunk=(a,b))))
            chunks.append((a,b,name,check,{} if chunk_view is None else dict(record_view=chunk_view)))
        return audit,chunks
    def sent_check_finish(self,segment,folder,audit,parts,started):
        """Combine the chunk replies of one segment's audit; sets its evidence or raises SegmentRejected."""
        stage,record,context=segment['stage'],segment['record'],segment['context']
        path=segment['prepared']['collision_path']
        valid=all(r.get('native_valid') and r.get('legacy_intersection_valid') and r.get('path_failure') is None for _,_,r in parts)
        path_failure=failure=None
        for a,b,r in parts:
            if r.get('path_failure') is not None and path_failure is None:
                path_failure=dict(r['path_failure'],chunk_start_state=a,edge_in_path=a+int(r['path_failure'].get('edge',0)))
            if r.get('failure') is not None and failure is None:failure=r['failure']
        checked=dict(native_valid=all(r.get('native_valid') for _,_,r in parts),
                     legacy_intersection_valid=all(r.get('legacy_intersection_valid') for _,_,r in parts),
                     path_failure=path_failure,failure=failure,probe_states=len(path),parallel_checkers=len(parts),
                     wall_s=time.monotonic()-started,chunks=[dict(states=[a,b],result=r) for a,b,r in parts])
        checked['sent_path_audit']=self.audit_summary(audit,[r for _,_,r in parts],segment['prepared'],valid)
        self.write_evidence(folder/(record+'-sent-check.json'),checked,'sent-check.json')
        if not valid:raise SegmentRejected(stage,stage+':SENT_PATH_REJECTED:'+str(path_failure or failure),checked)
        segment['evidence']=dict(native_plan='PASS',sent_path_collision='PASS',planner=segment['planner'],
                                 native_result=segment['native_result'],collision_result=checked,context=context,hardware_execution='NOT_RUN')
        return segment
    def sent_check(self,segment,folder,progress,cancelled=None,workers=None):
        """Final dense validation of the exact sent path, split across checkers (see sent_check_chunks)."""
        stage,record=segment['stage'],segment['record']
        path=segment['prepared']['collision_path'];workers=self.checkers() if workers is None else workers
        count=max(1,min(len(workers),(len(path)-1)//CHECK_CHUNK_MIN_STATES))
        audit,chunks=self.sent_check_chunks(segment,folder,count)
        progress(f"检查 {stage} 最终发送轨迹：{len(segment['prepared']['points'])} 点，{len(path)} 个检查状态，{count} 个进程并行"
                 +('（自适应审计）' if audit=='ADAPTIVE' else '（自适应审计，接触/支撑规则逐点精确）' if audit=='ADAPTIVE_CONTACT' else ''))
        def run(k):
            a,b,name,check,extra=chunks[k]
            return a,b,self.native_call(check,folder,name,600,cancelled,workers[k],**extra)
        started=time.monotonic()
        with timing.span('sent_check',stage=stage,record=record,states=len(path),checkers=count):
            with ThreadPoolExecutor(max_workers=count) as executor:parts=list(executor.map(run,range(count)))
        return self.sent_check_finish(segment,folder,audit,parts,started)
    def concurrent_sent_checks(self,pending,sub,progress,cancelled,pool):
        """concurrent_audits: every pending segment's unchanged sent-path check at once.

        Every segment starts with one worker; each further worker goes to the
        segment with the largest estimated wall time (per-request cost plus
        probe states times a per-state cost for its audit method). Each segment's
        chunks still cover its exact probe path once, with the identical scene
        and policy. On a rejection the other checks are cancelled and the first
        rejected segment in the sequential (cheapest-first) order is raised.
        """
        methods=[self.audit_method(s['context']) for s in pending]
        states=[len(s['prepared']['collision_path']) for s in pending]
        # Scheduling estimate only (never affects a result): seconds per probe
        # state on one worker, from the 2026-11-22 field replays, plus a fixed
        # per-request cost; workers go one at a time to the slowest segment.
        unit=[AUDIT_UNIT_S.get(m,AUDIT_UNIT_S['SAMPLED']) for m in methods]
        most=[max(1,(n-1)//CHECK_CHUNK_MIN_STATES) for n in states]
        shares={k:1 for k in range(len(pending))}
        cost=lambda k,n:AUDIT_REQUEST_S+states[k]*unit[k]/n
        for _ in range(len(pool)-len(pending)):
            open_=[k for k in shares if shares[k]<most[k]]
            if not open_:break
            k=max(open_,key=lambda k:cost(k,shares[k]));shares[k]+=1
        subsets={};start=0
        for k in range(len(pending)):subsets[k]=pool[start:start+shares[k]];start+=shares[k]
        stop=threading.Event();local=lambda:stop.is_set() or bool(cancelled and cancelled())
        outcome={}
        def run(k):
            try:self.sent_check(pending[k],sub,progress,local,subsets[k]);outcome[k]=None
            except SegmentRejected as e:outcome[k]=e;stop.set()
            except Exception as e:
                outcome[k]=e
                if not stop.is_set():stop.set()
        with timing.span('sent_check.concurrent',segments=len(pending),workers=len(pool)):
            threads=[threading.Thread(target=run,args=(k,),daemon=True,name=f'audit-{pending[k]["stage"]}') for k in range(len(pending))]
            for thread in threads:thread.start()
            for thread in threads:thread.join()
        for k in range(len(pending)):
            if isinstance(outcome.get(k),SegmentRejected):raise outcome[k]
        for k in range(len(pending)):
            if outcome.get(k) is not None:raise outcome[k]
    def audit_method(self,context):
        """ADAPTIVE only when configured and the segment has no contact permission and no support.

        Contact (seal/lip conditions) and loaded segments with supports (payload
        against the support plane) keep the SAMPLED audit; the worker refuses
        ADAPTIVE for them as well.
        """
        mode=self.site.get('sent_path_audit')
        if mode not in ('ADAPTIVE','ADAPTIVE_CONTACT'):return 'SAMPLED'
        spec=context['single_carton_contact']
        if not (spec['allow_contact'] or spec['supports']):return 'ADAPTIVE'
        # ADAPTIVE_CONTACT (2026-11-22): contact and support segments skip only
        # the pair distances proven from the last exact sample; every
        # single-carton contact/support rule is evaluated at every sample.
        return 'ADAPTIVE_CONTACT' if mode=='ADAPTIVE_CONTACT' else 'SAMPLED'
    @staticmethod
    def audit_summary(method,results,prepared,valid):
        """Method, checked/skipped samples and minimum distance lower bound of one segment's audit."""
        sampling=prepared.get('collision_sampling') or {}
        counts=[((r.get('clearance') or {}).get('counts') or {}).get('output') or {} for r in results]
        out=dict(audit_method=method,probe_states=len(prepared['collision_path']),
                 checked_states=sum(c.get('queries',0) for c in counts),
                 maximum_pair_displacement_between_probe_states_m=sampling.get('maximum_pair_displacement_between_checks_m'))
        if method in ('ADAPTIVE','ADAPTIVE_CONTACT'):
            audits=[(r.get('clearance') or {}).get('sent_path_audit') for r in results]
            if any(a is None for a in audits):raise ValueError('NATIVE_WORKER_DID_NOT_REPORT_ADAPTIVE_AUDIT')
            lows=[a['anchor_dynamic_pair_distance_lower_bound_m'] for a in audits if a['anchor_dynamic_pair_distance_lower_bound_m'] is not None]
            work=[a['anchor_workspace_margin_m'] for a in audits if a['anchor_workspace_margin_m'] is not None]
            out.update(sampled_states=sum(a['sampled_states'] for a in audits),skipped_states=sum(a['skipped_states'] for a in audits),
                       anchor_dynamic_pair_distance_lower_bound_m=min(lows) if lows else None,
                       anchor_workspace_margin_m=min(work) if work else None,margin_cap_m=audits[0]['margin_cap_m'],
                       proof_epsilon_m=audits[0]['proof_epsilon_m'],skip_rule=audits[0]['skip_rule'],
                       distance_evidence_scope='EXACT_ANCHORS_DYNAMIC_CONSTRAINTS_ONLY_EXCLUDES_SKIPPED_STATES_AND_FIXED_PAIRS',
                       whole_path_minimum_distance_status='NOT_COMPUTED')
        else:
            out.update(sampled_states=out['checked_states'],skipped_states=0,anchor_dynamic_pair_distance_lower_bound_m=None)
        # Every native sample of the sent path is checked exactly (>= its pair
        # limit, 2.4 mm) or, under ADAPTIVE, proven to keep its pair limit;
        # consecutive probe states move any body pair <= 1 mm (prepare), so
        # every state of the sent quintic keeps >= 1.4 mm, given prepare's
        # reach bound and analytic joint-speed bounds.
        out['continuous_swept_proof']='BOUNDED_DISPLACEMENT_1_4_MM' if valid else None
        return out
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
        rise in time, so a 45-degree and a lift-first profile follow. Both
        still move outward. A final purely vertical withdrawal can clear a
        nearby front edge before the separately planned loaded return turns
        outward. Every profile keeps the full payload and all obstacle checks.
        """
        outward=np.asarray(candidate['face']['normal'],float);up=np.array([0.,0.,1.])
        far=rigid(actual).copy();far[:3,3]+=outward*(max(box['size_m'])+.01)
        far[2,3]+=max(0.,.01-(far[2,3]-actual[2,3]))
        goals=[('OUTWARD',far)]
        for name,out,lift in DETACH_ALTERNATIVES:
            goal=rigid(actual).copy();goal[:3,3]+=outward*out+up*lift
            if min(np.linalg.norm(goal[:3,3]-g[:3,3]) for _,g in goals)>.005:goals.append((name,goal))
        # First reuse the existing lift-first height without its outward
        # component; a larger fallback clears one measured payload height.
        # A detached endpoint alone never suffices: the loaded return must
        # still be newly planned and all four sent paths audited.
        # Follow a near-vertical carton edge when present. A world-Z move of
        # a slightly tilted box otherwise drifts into its lateral neighbours.
        R=rigid(box['pose'])[:3,:3];axis=int(np.argmax(abs(R[2])))
        edge=R[:,axis]*np.sign(R[2,axis]);lift_axis=edge if edge[2]>=np.cos(np.deg2rad(10)) else up
        height=float(np.ptp(np.asarray(box['corners_m'])[:,2]))
        for name,lift in (('VERTICAL_CLEARANCE',DETACH_ALTERNATIVES[-1][2]),('VERTICAL_FULL_HEIGHT',height+.01)):
            vertical=rigid(actual).copy();vertical[:3,3]+=lift_axis*lift
            if min(np.linalg.norm(vertical[:3,3]-g[:3,3]) for _,g in goals)>.005:goals.append((name,vertical))
        return goals
    def screen_candidate(self,q,world,box,candidate,sub,cancelled=None,solutions=None,client=None):
        """Loaded q_start endpoint (rejecting) and task endpoints (ranking)."""
        tcp=rigid(candidate['tcp_pose']);flange=tcp@np.linalg.inv(self.coordinate_model.tool.matrix)
        attachment=np.linalg.inv(flange)@rigid(box['pose'])
        q_rel=self.release_pose(q)
        release=self.release_check(q_rel,attachment,box,world)
        # Full loaded endpoint (the release pose) is checked before any approach planning.
        release_supports=[dict(id=o['id'],top_world_m=self.object_top(o)) for o in world if o['id'] in release['region']['ids']]
        loaded_stage,record='return','loaded-start'
        if self.cycle_policy is not None:loaded_stage,record='transfer','loaded-release'
        start_loaded=self.context(q_rel,world,box,candidate,loaded_stage,attachment,release_supports)
        endcheck=self.native_call(dict(start_loaded,op='validate'),sub,record,120,cancelled,client)
        self.write_evidence(sub/(record+'-check.json'),dict(request=start_loaded,result=endcheck),'loaded-start-check.json')
        if not endcheck.get('native_valid') or not endcheck.get('legacy_intersection_valid'):
            raise ValueError(self.loaded_start_failure(endcheck,world))
        screen=self.endpoint_precheck(q,world,box,candidate,attachment,sub,cancelled,solutions,client=client)
        return dict(release_supports=release_supports,screen=screen)
    def plan_candidate_segments(self,q,world,box,candidate,sub,screened,progress=lambda _:None,cancelled=None,
                                clients=None,reuse=None):
        """As _plan_candidate_segments; a failed approach path moves on to the
        next endpoint-clear IK branch of the same candidate (rank order)."""
        screen=screened.get('screen') or {}
        pending=list(screen.get('pending_branches') or [])
        if reuse is not None or not pending:
            return self._plan_candidate_segments(q,world,box,candidate,sub,screened,progress,cancelled,clients,reuse)
        tried=[];attempt=screened;k=0
        while True:
            try:
                planned=self._plan_candidate_segments(q,world,box,candidate,sub,attempt,progress,cancelled,clients,None)
                planned['ik_branch']=dict(fallback_index=k,approach_q=list(attempt['screen']['approach_q']),earlier_failures=tried)
                return planned
            except SegmentRejected as e:
                if e.stage!='approach':raise
                tried.append(dict(approach_q=list(attempt['screen']['approach_q']),reason=str(e)[:200]))
                # Endpoint-check the next unchecked branch (same checks as the
                # screen), then plan with it; none left re-raises.
                tcp=rigid(candidate['tcp_pose']);flange=tcp@np.linalg.inv(self.coordinate_model.tool.matrix)
                attachment=np.linalg.inv(flange)@rigid(box['pose'])
                found=None
                while pending and found is None:
                    k+=1;qa=pending.pop(0)
                    progress(f'approach 未通过，检查下一个逆解分支（还剩 {len(pending)} 个）')
                    alt=self.endpoint_precheck(q,world,box,candidate,attachment,sub,cancelled,None,[qa],f'-alt{k}')
                    if alt['conclusion']=='CLEAR':found=alt
                if found is None:raise
                attempt=dict(screened,screen=dict(screen,approach_q=found['approach_q'],contact_q=found['contact_q']))
    def _plan_candidate_segments(self,q,world,box,candidate,sub,screened,progress=lambda _:None,cancelled=None,
                                 clients=None,reuse=None):
        """Plan all four native segments for one candidate; no dense sent check.

        With a screened approach configuration the contact/detach chain is
        planned from it first; approach and loaded return then run on two
        workers when two are given. Detach profiles are tried in order;
        ``reuse`` carries an earlier attempt's approach/contact and the profiles
        already rejected, so only a new detach + return are planned.
        """
        clients=list(clients or self.checkers())
        # START_RELEASE_SEPARATE with lane_clients >= 2: independent chains of
        # one candidate run concurrently on its own workers (fixed assignment):
        # clients[0] contact -> detach -> transfer, clients[1] withdraw ->
        # return_empty (after contact), clients[2] (if any) approach from the
        # start. Every segment is still planned and later audited unchanged.
        concurrent=self.cycle_policy is not None and len(clients)>=2 and reuse is None
        if not concurrent:clients=clients[:2]
        primary=clients[0];helper=clients[1] if len(clients)>1 and not concurrent else None
        plan=lambda *a,client=primary,**k:self.plan_segment(*a,client=client,**k)
        on=lambda client:(lambda *a,**k:self.plan_segment(*a,client=client,**k))
        scale=self.site['time_scale'];tcp=rigid(candidate['tcp_pose']);release_supports=screened['release_supports']
        local_stop=threading.Event();outer_cancelled=cancelled
        if concurrent:cancelled=lambda:local_stop.is_set() or bool(outer_cancelled and outer_cancelled())
        pool=ThreadPoolExecutor(max_workers=2,thread_name_prefix='candidate-chain') if concurrent else None
        approach_future=empty_future=None
        try:
            return self._plan_candidate_chain(q,world,box,candidate,sub,screened,progress,cancelled,clients,reuse,
                                              concurrent,primary,helper,plan,on,scale,tcp,release_supports,pool)
        finally:
            local_stop.set()
            if pool is not None:pool.shutdown(wait=True)
    def _plan_candidate_chain(self,q,world,box,candidate,sub,screened,progress,cancelled,clients,reuse,
                              concurrent,primary,helper,plan,on,scale,tcp,release_supports,pool):
        approach_future=empty_future=None
        if reuse is None:
            screen=screened['screen'];known=screen.get('approach_q');approach=None
            if concurrent and known is not None and len(clients)>2:
                approach_future=pool.submit(on(clients[2]),self.context(q,world,box,candidate,'approach'),list(known),'approach',scale,sub,progress,False,cancelled)
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
                       release=self.release_check(self.release_pose(q),attachment,box,world),original=self.detach_supports(world,box),rejected=[],
                       capacity_box=box,capacity_candidate=candidate)
            if concurrent:
                empty_future=pool.submit(self.plan_empty_return,q,world,box,candidate,dict(reuse),sub,progress,cancelled,on(clients[1]),scale)
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
            # Original cycle: loaded return to q_start. START_RELEASE_SEPARATE:
            # loaded transfer to q_release in the configured J1 direction.
            loaded_stage='return' if self.cycle_policy is None else 'transfer'
            loaded_goal=list(q) if self.cycle_policy is None else list(self.q_release)
            if self.cycle_policy is not None:
                wrong=cycle_structure.goal_violation(self.cycle_policy,'transfer_j1',qe,loaded_goal)
                if wrong:
                    r['rejected'].append(dict(profile=name,reason='transfer:'+json.dumps(wrong)));continue
            return_context=self.context(qe,world,box,candidate,loaded_stage,r['attachment'],release_supports)
            try:
                if concurrent:
                    returning=plan(return_context,loaded_goal,loaded_stage,scale,sub,progress,cancelled=cancelled)
                    if r['approach'] is None:
                        if approach_future is not None:r['approach']=approach_future.result()
                        else:r['approach']=plan(self.context(q,world,box,candidate,'approach'),r['known'],'approach',scale,sub,progress,False,cancelled)
                        if np.max(abs(np.asarray(r['approach']['prepared']['points'][-1]['q'])-r['qpre']))>1e-7:
                            raise ValueError('APPROACH_END_NOT_SCREENED_CONFIGURATION')
                elif r['approach'] is None and helper is None:
                    # One worker: fixed order keeps its OMPL request history reproducible.
                    r['approach']=plan(self.context(q,world,box,candidate,'approach'),r['known'],'approach',scale,sub,progress,False,cancelled)
                    if np.max(abs(np.asarray(r['approach']['prepared']['points'][-1]['q'])-r['qpre']))>1e-7:
                        raise ValueError('APPROACH_END_NOT_SCREENED_CONFIGURATION')
                    returning=plan(return_context,loaded_goal,loaded_stage,scale,sub,progress,cancelled=cancelled)
                elif r['approach'] is None:
                    with ThreadPoolExecutor(max_workers=2) as executor:
                        first=executor.submit(plan,self.context(q,world,box,candidate,'approach'),r['known'],'approach',scale,sub,progress,False,cancelled)
                        second=executor.submit(plan,return_context,loaded_goal,loaded_stage,scale,sub,progress,False,cancelled,None,client=helper)
                        approach_error=first.exception()
                        if approach_error is not None:raise approach_error
                        r['approach']=first.result()
                        if np.max(abs(np.asarray(r['approach']['prepared']['points'][-1]['q'])-r['qpre']))>1e-7:
                            raise ValueError('APPROACH_END_NOT_SCREENED_CONFIGURATION')
                        returning=second.result()
                else:returning=plan(return_context,loaded_goal,loaded_stage,scale,sub,progress,cancelled=cancelled)
                if self.cycle_policy is not None:self.check_j1_direction(returning,'transfer_j1')
            except SegmentRejected as e:
                if e.stage=='approach':raise
                r['rejected'].append(dict(profile=name,reason=str(e)[:400]));continue
            if np.max(abs(np.asarray(returning['prepared']['points'][-1]['q'])-np.asarray(loaded_goal)))>1e-7:
                raise ValueError('RETURN_NOT_LOCKED_Q_START' if self.cycle_policy is None else 'TRANSFER_NOT_AT_Q_RELEASE')
            if self.cycle_policy is not None and r.get('empty') is None:
                # Withdraw + empty return depend on the actual contact (the
                # released carton's pose) and q_release only, not on the detach
                # profile: planned once per candidate; a failure ends the candidate.
                r['empty']=empty_future.result() if empty_future is not None else self.plan_empty_return(q,world,box,candidate,r,sub,progress,cancelled,plan,scale)
            out=dict(r,extraction=extraction,returning=returning,profile=name)
            if self.cycle_policy is not None:out.update(withdrawal=r['empty']['withdrawal'],empty_return=r['empty']['empty_return'])
            return out
        raise ValueError('detach:ALL_PROFILES_REJECTED:'+json.dumps(r['rejected'],ensure_ascii=False))
    def check_j1_direction(self,segment,key):
        """Raw J1 direction of every state of the exact sent path (START_RELEASE_SEPARATE)."""
        states=segment['prepared']['collision_path'];sent=[p['q'] for p in segment['prepared']['points']]
        summary=cycle_structure.j1_summary(states);segment['j1_direction']=dict(summary,rule=self.cycle_policy[key],key=key,
            checked='PROBE_PATH_AND_SENT_POINTS')
        # Both the probe path and the 5 ms sent points (with a GEOMETRIC probe
        # grid the sent samples are not probe states).
        wrong=cycle_structure.path_violation(self.cycle_policy,key,states) or cycle_structure.path_violation(self.cycle_policy,key,sent)
        if wrong:raise SegmentRejected(segment['stage'],segment['stage']+':J1_DIRECTION_REJECTED:'+json.dumps(wrong),
                                       dict(status='J1_DIRECTION_REJECTED',failure=wrong))
        return summary
    def plan_empty_return(self,q,world,box,candidate,r,sub,progress,cancelled,plan,scale):
        """Withdraw (empty LIN off the released carton) and return to q_start (START_RELEASE_SEPARATE).

        The released carton's full envelope stays a collision object at its
        release pose (``cycle_structure.released_task``). The withdraw LIN keeps
        the native contact-stage rules on it (permitted lips only, 0.4 mm
        penetration and normal guards; rigid inserts and every other body keep
        their pair clearances). The withdraw distance is this candidate's own approach standoff, so it is
        the approach->contact LIN reversed in the carton frame. The empty
        return has no exemption and must move J1 in the configured direction.
        """
        q_rel=np.asarray(self.q_release,float)
        rbox,rcandidate,release_world,released=cycle_structure.released_task(box,candidate,world,r['release']['pose'])
        standoff=float(np.linalg.norm(np.asarray(candidate['precontact_pose'])[:3,3]-np.asarray(candidate['tcp_pose'])[:3,3]))
        goal=cycle_structure.withdraw_goal(self.coordinate_model.tcp(q_rel),r['actual'],candidate['face']['normal'],standoff)
        withdrawal=plan(self.context(q_rel,release_world,rbox,rcandidate,'withdraw'),goal,'withdraw',scale,sub,progress,True,cancelled)
        withdrawal['j1_direction']=cycle_structure.j1_summary(withdrawal['prepared']['collision_path'])
        qw=withdrawal['prepared']['points'][-1]['q']
        wrong=cycle_structure.goal_violation(self.cycle_policy,'return_j1',qw,q)
        if wrong:raise SegmentRejected('return_empty','return_empty:J1_DIRECTION_REJECTED:'+json.dumps(wrong),dict(status='J1_DIRECTION_REJECTED',failure=wrong))
        empty_return=plan(self.context(qw,release_world,rbox,rcandidate,'return_empty'),list(q),'return_empty',scale,sub,progress,False,cancelled)
        self.check_j1_direction(empty_return,'return_j1')
        if np.max(abs(np.asarray(empty_return['prepared']['points'][-1]['q'])-np.asarray(q)))>1e-7:raise ValueError('RETURN_EMPTY_NOT_AT_Q_START')
        return dict(withdrawal=withdrawal,empty_return=empty_return,released_carton=released,withdraw_distance_m=standoff)
    def segment_keys(self):
        keys=['approach','contact','extraction','returning']
        return keys+(['withdrawal','empty_return'] if self.cycle_policy is not None else [])
    def holding_postcheck(self,planned,sub):
        from .holding_capacity import evaluate,sent_poses
        poses,times=sent_poses(planned,self.coordinate_model)
        args=(planned['capacity_box'],planned['capacity_candidate'],self.source['tool'],self.site)
        static=evaluate(*args,contact_tcp=planned['actual'])
        dynamic=evaluate(*args,poses=poses,times=times,contact_tcp=planned['actual'])
        result=dict(static_actual_contact=static,loaded_sent_states=dynamic,
            valid=static['valid'] and dynamic['valid'],hardware_validation='NOT_RUN')
        self.write_evidence(sub/'holding-capacity-final.json',result,'holding-capacity-final.json')
        if not result['valid']:
            raise SegmentRejected('holding_capacity','HOLDING_CAPACITY_INSUFFICIENT:'+json.dumps(result))
        return result
    def verify_candidate(self,planned,sub,progress=lambda _:None,cancelled=None,workers=None):
        """Final dense sent-path checks of a planned candidate, cheapest first.

        A segment checked in an earlier detach-profile attempt (approach and
        contact) keeps its evidence. Raises SegmentRejected on any failure.
        """
        segments=[planned[k] for k in self.segment_keys()]
        pending=[s for s in sorted(segments,key=lambda s:len(s['prepared']['collision_path'])) if s.get('evidence') is None]
        pool=workers if workers is not None else self.checkers()
        if self.site.get('concurrent_audits') and not self.native_planning() and 1<len(pending)<=len(pool):
            self.concurrent_sent_checks(pending,sub,progress,cancelled,pool)
        else:
            for segment in pending:self.sent_check(segment,sub,progress,cancelled,workers)
        capacity=self.holding_postcheck(planned,sub)
        for segment in segments:segment['evidence']['holding_capacity']=capacity
        planned['extraction']['evidence'].update(detach_profile=planned['profile'],rejected_detach_profiles=planned['rejected'])
        for segment in segments:
            if segment.get('j1_direction') is not None:segment['evidence']['j1_direction']=segment['j1_direction']
        if self.cycle_policy is not None:
            planned['empty_return']['evidence'].update(released_carton=planned['empty']['released_carton'],
                withdraw_distance_m=planned['empty']['withdraw_distance_m'])
        return dict(segments=[dict(stage=s['stage'],prepared=s['prepared'],evidence=s['evidence']) for s in segments],
                    attachment=planned['attachment'],release=planned['release'],seal=planned['seal'])
    def replan_after(self,planned,error):
        """reuse for the next detach profile after a detach/return sent-path rejection."""
        if error.stage in ('approach','contact','holding_capacity','withdraw','return_empty'):raise error
        return dict(planned,rejected=planned['rejected']+[dict(profile=planned['profile'],reason=str(error)[:400])])
    def plan_candidate(self,q,world,box,candidate,sub,screened,progress=lambda _:None,cancelled=None,clients=None,workers=None):
        """Four native segments for one candidate, planned then densely checked; no permit."""
        planned=self.plan_candidate_segments(q,world,box,candidate,sub,screened,progress,cancelled,clients)
        while True:
            try:return self.verify_candidate(planned,sub,progress,cancelled,workers)
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
        workers=self.checkers();group=int(self.site.get('lane_clients',1)) if self.cycle_policy is not None else 1
        count=max(1,min(int(self.site.get('parallel_candidate_lanes',1)),len(workers)//group))
        lanes=[workers[k*group:(k+1)*group] for k in range(count)] if group>1 else workers[:count]
        stop=threading.Event();idle=threading.Event();idle.set()
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
                        with timing.span('lane.plan_candidate',lane=index,rank=rank,candidate=i,replan=reuse is not None):
                            planned=self.plan_candidate_segments(q,world,box,candidate,sub,screened,say,lane_cancelled,
                                                                 list(client) if isinstance(client,list) else [client],reuse)
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
                wait=.1
                if self.deadline is not None:
                    left=self.deadline-time.monotonic()
                    if left<=0:raise PlanningBudgetExhausted('WAITING_FOR_CANDIDATE_LANE',f'rank {rank}')
                    wait=min(wait,left)
                try:return results[rank].get(timeout=wait)
                except queue.Empty:continue
        try:
            for rank,(i,candidate,sub,screened) in enumerate(order):
                with timing.span('main.wait_planned_candidate',rank=rank,candidate=i):kind,value=receive(rank)
                while kind=='PLANNED':
                    idle.clear()
                    try:
                        with timing.span('main.verify_candidate',rank=rank,candidate=i):result=self.verify_candidate(value,sub,lambda text,i=i,rank=rank:progress(f'检查候选 {i+1}（{rank+1}/{len(order)}）：{text}'),cancelled)
                    except SegmentRejected as e:
                        try:reuse=self.replan_after(value,e)
                        except SegmentRejected as fatal:kind,value='FAILED',fatal;commands[rank].put(('DONE',));break
                        commands[rank].put(('REPLAN',reuse));idle.set()
                        with timing.span('main.wait_replanned_candidate',rank=rank,candidate=i):kind,value=receive(rank)
                        continue
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
    def parallel_screening_enabled(self):
        return bool(self.site.get('parallel_screening')) and not self.native_planning()
    def parallel_screen(self,q,world,box,candidates,candidate_filter,folder,progress,cancelled,failures):
        """Screen every candidate concurrently over the resident workers (parallel_screening).

        Candidate i runs on worker i % n with IK seeds of its own only (q_start,
        the fixed spread seeds and its own solutions), so each screen depends
        on nothing but its candidate and the outcome is independent of thread
        timing. The checks themselves are unchanged. Results and failures are
        kept in candidate order; a target-level block (TargetBlocked) of the
        lowest blocked candidate is raised after every screen has stopped.
        """
        clients=self.checkers();rows=[i for i in range(len(candidates)) if candidate_filter is None or i in candidate_filter]
        stop=threading.Event();lane_cancelled=lambda:stop.is_set() or bool(cancelled())
        progress(f'并行预检 {len(rows)} 个候选（{len(clients)} 个原生进程）：带载释放位姿及接近 / 接触 / 脱离终态')
        def one(i):
            sub=folder/(box['id']+'-'+str(i));sub.mkdir()
            try:
                with timing.span('search.screen_candidate',candidate=i,parallel=True):
                    return ('OK',sub,self.screen_candidate(q,world,box,candidates[i],sub,lane_cancelled,[],clients[i%len(clients)]))
            except TargetBlocked as e:stop.set();return ('BLOCKED',sub,e)
            except ValueError as e:return ('FAILED',sub,e)
        # One thread per worker, each serving its own candidates in rank order.
        results={}
        def lane(k):
            for i in rows[k::len(clients)]:
                if lane_cancelled():results[i]=('STOPPED',None,None);continue
                results[i]=one(i)
        threads=[threading.Thread(target=lane,args=(k,),daemon=True,name=f'screen-lane-{k}') for k in range(min(len(clients),len(rows)))]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        screened=[]
        for i in rows:
            kind,sub,value=results[i]
            if kind=='BLOCKED':raise value
        if cancelled():raise ValueError('CANCELLED')
        for i in rows:
            kind,sub,value=results[i]
            if kind=='OK':screened.append((i,candidates[i],sub,value))
            elif kind=='FAILED':failures.append(dict(target=box['id'],candidate=i,reason=str(value)))
        return screened
    def native_screen(self,q,world,box,candidates,candidate_filter,folder,cancelled=None):
        """planning_module NATIVE: the sc_endpoint_screen composite; returns its replay client."""
        from .native_flow import NativeFlow
        considered=[i for i in range(len(candidates)) if candidate_filter is None or i in candidate_filter]
        return NativeFlow(self).screen(q,world,box,candidates,considered,folder,cancelled)
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
        self.wrist_reference=list(q)
        if self.cycle_policy is not None:
            if self.native_planning():raise ValueError('CYCLE_STRUCTURE_REQUIRES_PLANNING_MODULE_PYTHON')
            lower,upper,_,_=self.limits.arrays()
            self.q_release=cycle_structure.release_q(self.cycle_policy,q,lower,upper).tolist()
        with timing.span('search.payload_rest_check'):rest=self.payload_rest_check(world,box)
        self.write_evidence(folder/(box['id']+'-payload-rest-check.json'),rest,'payload-rest-check.json')
        if rest['intersections'] or rest['outside_workspace']:
            hit=rest['intersections'][0] if rest['intersections'] else None
            seam=seam_band_contacts(world,box,rest)
            if seam:rest['coplanar_seam_contact']=seam
            if seam and seam['all_within_seam_band'] and not rest['outside_workspace']:
                # Not merged into the generic overlap: the user decides whether
                # a payload-only seam contact rule may be added (none exists).
                reason=('COPLANAR_SEAM_CONTACT_NOT_PERMITTED: 接缝策略外界与共面邻物重叠，全部位于接缝带内：'+
                        '；'.join(f"{c['id']} {c['overlap_depth_m']*1000:.3f} mm" for c in seam['contacts'])+
                        '；本轮未增加接缝接触规则，未进行候选/路径搜索')
            elif rest.get('geometry_basis')=='CONSERVATIVE_UNCERTAINTY_ENVELOPE':
                reason=('GEOMETRY_UNCERTAINTY_OVERLAP_AT_REST: 保守不确定性包络与 '+
                        (f"{hit['id']} 的模型包络重叠 {hit['overlap_depth_m']*1000:.3f} mm" if hit else '现场工作范围边界重叠')+
                        '；当前观测尚不能证明带载路径安全，不代表实物穿入或全部几何假设均不可行；未进行候选/路径搜索')
            else:reason=('TARGET_PAYLOAD_INTERSECTS_AT_REST: 箱体在原位时碰撞包络已与 '+
                    (f"{hit['id']} 相交 {hit['overlap_depth_m']*1000:.1f} mm" if hit else '现场工作范围边界相交')+
                    f"（共 {len(rest['intersections'])} 处）；带箱脱离的起点与吸取候选无关，本目标所有候选都无法脱离，未进行路径规划")
            progress(reason);failures.append(dict(target=box['id'],reason=reason,evidence=rest));return None
        if self.site.get('early_scene_upload'):self.start_scene_uploads(folder,cancelled)
        # Bookkeeping for the searched / unsearched summary only.
        self.search_counts[box['id']]=dict(candidates=None,phase='CANDIDATE_GENERATION',screened=0,considered=None)
        try:
            with timing.span('search.contact_candidates'):
                candidates=contact_candidates(box,self.source['tool'],self.site['topology'],self.site.get('contact_search'))
        except ValueError as e:failures.append(dict(target=box['id'],reason=str(e)));return None
        considered=[i for i in range(len(candidates)) if candidate_filter is None or i in candidate_filter]
        viable=[]
        from .holding_capacity import evaluate
        for i in considered:
            try:capacity=evaluate(box,candidates[i],self.source['tool'],self.site)
            except ValueError as e:capacity=dict(valid=False,status='HOLDING_CAPACITY_INSUFFICIENT',invalid_evidence=str(e))
            candidates[i]['holding_capacity']=capacity
            if capacity['valid']:viable.append(i)
            else:failures.append(dict(target=box['id'],candidate=i,reason='HOLDING_CAPACITY_INSUFFICIENT',evidence=capacity))
        self.write_evidence(folder/(box['id']+'-candidates.json'),[
            dict(index=i,active_ids=c['active_ids'],commanded_cup_ids=c['commanded_cup_ids'],
                 expected_passively_closed_ids=c['expected_passively_closed_ids'],tcp_pose=c['tcp_pose'],
                 precontact_pose=c['precontact_pose'],seal_check=c['seal_check'],
                 contact_search=c.get('contact_search'),rings_tcp=c['rings_tcp'],holding_capacity=c.get('holding_capacity'))
            for i,c in enumerate(candidates)],'contact-candidates.json')
        # Bookkeeping for the searched / unsearched summary only.
        self.search_counts[box['id']]=dict(candidates=len(candidates),phase='SCREENING',screened=0,
            considered=considered,holding_capacity_rejected=len(considered)-len(viable))
        candidate_filter=viable
        if not viable:
            self.search_counts[box['id']]['phase']='DONE'
            return None
        screened=[];solutions=[]
        try:
            # planning_module NATIVE: one composite screens every candidate; the
            # unchanged loop below then runs against its recorded results.
            replay=self.native_screen(q,world,box,candidates,candidate_filter,folder,cancelled) if self.native_planning() else None
            main_client=self.client
            if replay is not None:self.client=replay
            try:
                if self.parallel_screening_enabled():
                    if int(self.site.get('native_check_workers',0))>0:self.wait_check_pool()
                    screened=self.parallel_screen(q,world,box,candidates,candidate_filter,folder,progress,cancelled,failures)
                else:
                  for i,candidate in enumerate(candidates):
                    if candidate_filter is not None and i not in candidate_filter:continue
                    if cancelled():raise ValueError('CANCELLED')
                    sub=folder/(box['id']+'-'+str(i));sub.mkdir()
                    progress(f'预检候选 {i+1}/{len(candidates)}：带载返回起点及接近 / 接触 / 脱离终态')
                    try:
                        with timing.span('search.screen_candidate',candidate=i):
                            screened.append((i,candidate,sub,self.screen_candidate(q,world,box,candidate,sub,cancelled,solutions)))
                    except TargetBlocked:raise
                    except ValueError as e:failures.append(dict(target=box['id'],candidate=i,reason=str(e)))
            finally:self.client=main_client
            if replay is not None and replay.unused():
                from .native_flow import NativeFlowDiverged
                raise NativeFlowDiverged(f'NATIVE_FLOW_DIVERGED: {replay.unused()} recorded screening results not requested')
            def candidate_priority(row):
                from .holding_capacity import priority
                screen=row[3]['screen'];clear=screen['conclusion']=='CLEAR'
                preferred=posture.key(self.posture_policy,screen['approach_q'],q) if self.posture_policy and clear else ()
                return priority(clear,row[1]['holding_capacity']['minimum_margin'],preferred)
            order=sorted(screened,key=candidate_priority)
            self.write_evidence(folder/'candidate-priority.json',[
                dict(candidate=i,screen=s['screen']['conclusion'],
                     holding_capacity_tier=c['holding_capacity']['tier'],holding_capacity_margin=c['holding_capacity']['minimum_margin'],
                     posture=posture.metrics(self.posture_policy,s['screen']['approach_q'],q)
                     if self.posture_policy and s['screen']['conclusion']=='CLEAR' else None)
                for i,c,sub,s in order],'candidate-priority.json')
            self.search_counts[box['id']].update(phase='PLANNING',screened=len(order))
            clear=sum(s[3]['screen']['conclusion']=='CLEAR' for s in order)
            progress(f'预检完成：{clear} 个候选终态全部通过，{len(order)-clear} 个逆解未定论，'
                     f'{len(candidates)-len(order)} 个已排除；按此顺序完整规划')
            if self.native_planning() and order:
                if int(self.site.get('native_check_workers',0))>0:self.wait_check_pool()
                if self.deadline is not None and time.monotonic()>=self.deadline:
                    raise PlanningBudgetExhausted('WAITING_FOR_CHECK_POOL')
                from .native_flow import NativeFlow
                with timing.span('search.native_candidates',candidates=len(order)):
                    return NativeFlow(self).plan_ranks(q,world,box,order,progress,cancelled,failures)
            if self.site.get('pipelined_audits') and order:
                self.wait_check_pool()
                if self.deadline is not None and time.monotonic()>=self.deadline:
                    raise PlanningBudgetExhausted('WAITING_FOR_CHECK_POOL')
                from .pipeline import pipelined_candidates
                with timing.span('search.pipelined_candidates',candidates=len(order)):
                    return pipelined_candidates(self,q,world,box,order,progress,cancelled,failures)
            if int(self.site.get('parallel_candidate_lanes',1))>1 and len(order)>1:
                self.wait_check_pool()
                if self.deadline is not None and time.monotonic()>=self.deadline:
                    raise PlanningBudgetExhausted('WAITING_FOR_CHECK_POOL')
                if len(self.checkers())>1:
                    with timing.span('search.parallel_candidates',candidates=len(order)):
                        return self.parallel_candidates(q,world,box,order,progress,cancelled,failures)
            for rank,(i,candidate,sub,screened_candidate) in enumerate(order):
                if cancelled():raise ValueError('CANCELLED')
                say=lambda text,i=i,rank=rank:progress(f'规划候选 {i+1}（{rank+1}/{len(order)}）：{text}')
                try:
                    with timing.span('search.sequential_plan_candidate',rank=rank,candidate=i):
                        result=self.plan_candidate(q,world,box,candidate,sub,screened_candidate,say,cancelled)
                except TargetBlocked:raise
                except ValueError as e:failures.append(dict(target=box['id'],candidate=i,reason=str(e)));continue
                return dict(result,index=i,candidate=candidate,screen=screened_candidate['screen'],lanes=1)
        except TargetBlocked as e:
            progress(str(e));failures.append(dict(target=box['id'],reason=str(e)))
        return None
    def budget_summary(self,boxes,failures,selected,error):
        from .budget import snapshot
        return snapshot(self,boxes,failures,selected,error)
    def background_close(self,folder):
        """Under a deadline: record the pool, then close every worker off this thread.

        A worker ignores SIGTERM, so each ResidentMoveItClient.close waits 5 s
        before SIGKILL; sequential closes must not delay the classified result.
        Closed workers never receive another request.
        """
        with self.pool_lock:
            self.pool_closed=True;pool,self.pool=list(self.pool),[]
            executors,self.ipc_executors=dict(self.ipc_executors),{}
        status=[dict(s) for s in self.pool_status]
        threading.Thread(target=timing.write_json,args=(folder/'native-check-pool.json',status,'native-check-pool.json'),
                         kwargs=dict(ensure_ascii=False),daemon=True,name='native-pool-record').start()
        clients=[c for c in [self.client]+pool if c is not None];self.client=None
        self.cleanup_errors=[]
        def close(client):
            try:
                entry=executors.get(id(client))
                if entry is not None:entry[1].shutdown(wait=True)
                client.close()
            except Exception as error:
                self.cleanup_errors.append(dict(worker=getattr(client,'name','worker'),error=str(error)))
        self.cleanup_threads=[]
        for client in clients:
            thread=threading.Thread(target=close,args=(client,),daemon=True,name='native-close')
            self.cleanup_threads.append(thread);thread.start()
    def plan_cycle(self,observation,boxes,folder,progress=lambda _:None,selected=None,cancelled=lambda:False,scene=None,deadline=None,finalize=None):
        from .lifecycle import run_task
        return run_task(self,observation,boxes,folder,progress,selected,cancelled,scene,deadline,finalize)
    def _plan_cycle(self,observation,boxes,folder,progress=lambda _:None,selected=None,cancelled=lambda:False,scene=None,deadline=None):
        """Full search and permit registration; ``deadline`` is the optional absolute
        time.monotonic() planning deadline (None keeps the original behaviour)."""
        self.task_cancelled=cancelled
        from ..contracts import Source,HOST_SESSION
        if (observation.frame.stamp.source!=Source.LIVE or observation.frame.stamp.host_session!=HOST_SESSION or
                observation.robot_sample.get('source')!='LIVE'):
            raise ValueError('PHYSICAL_PLANNER_REQUIRES_CURRENT_SESSION_LIVE_CAPTURE')
        self.deadline=deadline;self.abandoned=set();self.search_counts={};self.search_thread=None
        self.authority.invalidate();self.cycle_binding=None
        from .size_policy import unmodeled_cartons
        if scene and unmodeled_cartons(scene):
            raise ValueError('UNBOUNDED_INSTANCE_GEOMETRY: 未知箱体缺少保守现场障碍表示；未开始路径搜索')
        qualification=release_policy_evidence(self.site)
        with timing.span('plan_cycle.build'):self.build()
        # A global configuration failure is not 32 independent infeasible
        # grasps. Retain the explicit measured policy without silently deferring.
        if not qualification['qualification_deferred']:self.receiver()
        if self.site.get('site') and scene is None:raise ValueError('FROZEN_REAL_SITE_SCENE_REQUIRED')
        world=deepcopy(scene['world']) if scene else [deepcopy(x) for x in self.world if x.get('category')!='carton']
        if scene and scene['observation_id']!=observation.frame.observation_id:raise ValueError('REAL_SCENE_CAPTURE_MISMATCH')
        if self.site.get('site_layout') or any(o.get('category')=='site_layout_cad' for o in world):
            from .site_layout import load_layout
            layout=load_layout(self.site,ROOT);registration=(scene or {}).get('layout_registration') or {}
            if (layout is None or not registration.get('valid') or registration.get('identity')!=layout['identity']
                    or [o for o in world if o.get('category')=='site_layout_cad']!=layout['world']):
                raise ValueError('LAYOUT_REGISTRATION_MISMATCH')
            for path in layout['inputs']:self.track(path)
        world += [dict(id=b['id'],size=b['size_m'],pose=b['pose'],category='carton') for b in boxes]
        q=np.array(observation.q_start);failures=[];self.task_failures=failures
        # Unified deadline for every native request of this plan (None: unchanged).
        self.deadline=deadline;self.deadline_reference=None if deadline is None else time.monotonic()
        self.start_task_services(world,folder)
        self.start_main_worker(folder)
        try:
            self.acquire_workers(folder,boxes)
            from .site_layout import check_layout_start
            check_layout_start(self,q,world,boxes,folder,cancelled)
            for box in boxes:
                if selected is not None and box['detection_index']!=selected:continue
                with timing.span('plan_cycle.search_box',target=box['id']):
                    if self.deadline is None:found=self.search_box(q,world,box,folder,progress,cancelled,failures)
                    else:found=self.bounded_search(q,world,box,folder,progress,cancelled,failures)
                if box['id'] in self.search_counts:
                    self.search_counts[box['id']].update(phase='DONE' if found is None else 'SEARCH_SUCCEEDED',
                                                       accepted=None if found is None else found['index'])
                if found is None:continue
                i,candidate,screen,result=found['index'],found['candidate'],found['screen'],found
                attachment,release=result['attachment'],result['release']
                self.cycle_binding=dict(task_id=folder.name,observation=observation.metadata(),target=box,world=digest(world),candidate=candidate,
                    attachment=attachment.tolist(),release=release,expected_attachment='PLANNED_NOT_PHYSICALLY_CONFIRMED')
                self.cycle_binding['site_scene']=scene
                self.cycle_binding['release_world']=world
                from .measured_envelope import payload_mass_properties
                self.cycle_binding['payload_mass_properties']=payload_mass_properties(box,self.site['carton_mass_kg'])
                with timing.span('plan_cycle.binding'):binding=self.binding()
                segments=result['segments']
                if [s['stage'] for s in segments]!=list(cycle_structure.stages(self.cycle_policy)):raise ValueError('INVALID_STAGE_CHAIN')
                # A segment is registered only with its exact sent path checked;
                # a deferred worker waypoint check is never accepted on its own.
                if any(s['evidence'].get('sent_path_collision')!='PASS' for s in segments):raise ValueError('SENT_PATH_CHECK_MISSING')
                native_checks=sorted({s['evidence']['planner'].get('native_output_check','PASS') for s in segments})
                # A user-disabled start check is bound into the first segment's
                # package (and therefore its digest), never applied implicitly.
                start_policy=start_joint_check(self.site)
                segments[0]['prepared']['start_joint_check']=start_policy
                # Final audit of every sent state against the wrist limits.
                for segment in segments:
                    violation=wrist_branch_violation(self.wrist_limits,q,segment['prepared']['collision_path'])
                    if violation:raise ValueError('WRIST_BRANCH_REJECTED_AT_AUDIT:'+segment['stage']+':'+json.dumps(violation))
                # A configured execution lock travels inside every package, so
                # the motion engine refuses it even if another path submits it.
                lock=execution_lock(self.site)
                if lock:
                    for segment in segments:segment['prepared']['execution_lock']=lock
                for segment in segments:
                    with timing.span('plan_cycle.register',stage=segment['stage']):
                        segment['package']=self.authority.register(segment.pop('prepared'),binding,segment.pop('evidence'))
                plan=dict(binding=binding,segments=segments,target=box,candidate=candidate,release=release,start_joint_check=start_policy,
                    execution_lock=lock,wrist_branch_limits=self.wrist_limits,posture_preference=self.posture_policy,
                    contact_search=self.site.get('contact_search'),tool_robot_clearance=self.clearance.get('tool_robot_clearance'),
                    q_start=q.tolist(),T_world_tcp_start=observation.world_tcp_start,full_path_validated=True,
                    candidate_index=i,endpoint_screen=screen['conclusion'],candidate_lanes=found.get('lanes',1),
                    native_output_check=native_checks,cycle_structure=self.cycle_policy,
                    q_release=None if self.cycle_policy is None else list(self.q_release),
                    duration_s=sum(s['package']['prepared']['send_duration_s'] for s in segments),hardware_execution='NOT_RUN')
                if self.binding_evidence is not None:plan['binding_hash_evidence']=self.binding_evidence
                if self.task_start is not None:plan['resident_task_start']=self.task_start
                if self.evidence is None:
                    timing.write_json(getattr(self,'complete_plan_path',folder/'complete-plan.json'),plan,'complete-plan.json');return plan
                # PLAN_READY needs the plan evidence on disk: everything queued
                # for this task, then the plan itself, before returning.
                self.evidence.write_json(getattr(self,'complete_plan_path',folder/'complete-plan.json'),plan,'complete-plan.json')
                with timing.span('plan_cycle.evidence_flush'):
                    self.evidence.flush(timeout=120 if deadline is None else max(0.,deadline-time.monotonic()),cancelled=cancelled)
                return plan
            # Keep every concrete failure on disk; one task-status label must
            # not grow into 32 copies of the full native collision JSON.
            searched=None
            if self.deadline is None:
                timing.write_json(folder/'planning-failures.json',failures,'planning-failures.json',ensure_ascii=False,indent=2)
            else:
                searched=self.budget_summary(boxes,failures,selected,None)
                timing.write_json(folder/'planning-failures.json',dict(failures=failures,planning_budget=searched),
                                  'planning-failures.json',ensure_ascii=False,indent=2)
            counts=Counter(failure_category(f['reason']) for f in failures)
            summary='；'.join(f'{reason} × {count}' for reason,count in counts.items())
            first=failures[0]['reason'][:600] if failures else '没有符合所选编号的目标'
            if searched is not None:
                summary+='；'+'；'.join(f"{t['target']}: 已搜索 {t['searched']}，未搜索 {t['unsearched']}" for t in searched['targets'])
            raise ValueError(f'NO_COMPLETE_FEASIBLE_SINGLE_CARTON_PLAN: {summary}；首项：{first}；详情：planning-failures.json')
        except PlanningBudgetExhausted as error:
            # Not an infeasibility verdict and not an IPC failure: record what
            # was and was not searched, then report the expiry as such. A
            # released search thread may still append to ``failures``, so the
            # summary is taken from a snapshot of what was concluded by now.
            concluded=list(failures)
            error.evidence=self.budget_summary(boxes,concluded,selected,error)
            # The classification is complete here; its record is written off
            # this thread, so lanes still writing their own records cannot
            # delay the return (AGENTS 2026-11-16 3e: evidence may be async).
            record=dict(failures=concluded,planning_budget=error.evidence)
            error.record_thread=threading.Thread(target=timing.write_json,args=(folder/'planning-failures.json',record,'planning-failures.json'),
                kwargs=dict(ensure_ascii=False,indent=2),daemon=True,name='planning-budget-record')
            error.record_thread.start()
            progress('规划预算耗尽：'+'；'.join(f"{t['target']} 已搜索 {t['searched']}、未搜索 {t['unsearched']}（未搜索的不算不可行）"
                                          for t in error.evidence['targets']))
            raise
        finally:
            with timing.span('plan_cycle.close_workers'):self.release_workers(folder)
            # A released search thread keeps its (past) deadline until it ends.
            if self.search_thread is None or not self.search_thread.is_alive():self.deadline=None;self.deadline_reference=None
    # -- step 4: worker lifecycle per task -------------------------------------------------
    def start_task_services(self,world,folder):
        """Per-task evidence writer and scene registration (each only when enabled)."""
        if self.enabled('async_evidence'):
            from .evidence import EvidenceWriter
            if self.evidence is not None:self.evidence.close()
            self.evidence=EvidenceWriter('planning-evidence')
        if self.enabled('scene_handles'):self.register_scene(world,folder)
        else:self.scene_world=self.scene_fingerprint=None
    def resident_pool(self):
        """The resident main worker and checker pool, started on first use."""
        with self.build_lock:
            with self.task_lock:
                if self.closed:raise RuntimeError('PLANNER_CLOSED')
                if self.resident is not None:return self.resident
            if self.resident is None:
                from .resident import WorkerPool
                logs=Path(self.site['record_root'])/'native-workers';logs.mkdir(parents=True,exist_ok=True)
                command=lambda name:['env','ROS_LOG_DIR='+str(logs),str(ROOT/'ros2/m710_moveit_backend/worker.sh')]
                self.build()
                pool=WorkerPool(command,self.init,int(self.site.get('native_check_workers',0)),logs,binary=self.binary_transport())
                with self.task_lock:
                    if self.closed:
                        pool.close();raise RuntimeError('PLANNER_CLOSED')
                    self.resident=pool;pool.start()
            return self.resident
    def prewarm(self):
        """Build on isolated state; a late prewarm cannot overwrite an active task."""
        if not self.enabled('resident_workers'):return None
        import copy
        with self.task_lock:
            if self.closed:return None
            generation=self.task_generation;warm=copy.copy(self)
            warm.files=dict(self.files);warm.hash_cache=dict(self.hash_cache)
            warm.build_lock=threading.RLock();warm.resident=None
        def run():
            pool=None
            try:
                pool=warm.resident_pool()
                with self.task_lock:
                    if not self.closed and self.task_generation==generation and self.resident is None:
                        # A task may reuse it; acquire_workers bounds readiness.
                        self.resident=pool
                        return
                pool.close()
                if hasattr(pool,'wait_closed') and not pool.wait_closed(240.):
                    raise RuntimeError('PREWARM_CLEANUP_INCOMPLETE:'+str(pool.cleanup_errors))
            except Exception as error:
                with self.task_lock:self.prewarm_errors.append(str(error))
        thread=threading.Thread(target=run,daemon=True,name='resident-prewarm');thread.start()
        return thread
    def start_main_worker(self,folder):
        """Default path: start this task's main worker process (outside the task's try block, as before)."""
        self.task_start=None
        if self.enabled('resident_workers'):return
        from unloading_sim.moveit2_backend import ResidentMoveItClient
        with timing.span('native.main_worker_process_start'):
            self.client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(folder),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=folder/'moveit.log',binary=self.binary_transport())
    def acquire_workers(self,folder,boxes):
        """Main worker and checker pool for one task (resident or started for this task)."""
        if self.enabled('resident_workers'):
            pool=self.resident_pool()
            wait=240. if self.deadline is None else max(0.,self.deadline-time.monotonic())
            with timing.span('native.resident_wait_ready'):ready=pool.wait_ready(wait)
            with timing.span('native.resident_begin_task'):self.task_start=pool.begin_task(deadline=self.deadline,cancelled=getattr(self,'task_cancelled',lambda:False))
            if self.deadline is not None and time.monotonic()>=self.deadline:raise PlanningBudgetExhausted('RESIDENT_TASK_START')
            self.client=pool.main;self.pool=pool.checkers();self.pool_threads=[];self.pool_status=pool.snapshot()
            self.worker_ready(ready,boxes);self.require_mechanism_capabilities(ready)
            return
        if self.deadline is None:ready=timing.native_request(self.client,self.init,record='main-init')
        else:
            initialising=time.monotonic()
            ready=self.bounded_request(self.client,self.init,120.,None,'main-init')
            self.worker_init_s=time.monotonic()-initialising
        self.worker_ready(ready,boxes);self.require_mechanism_capabilities(ready)
        self.start_check_pool(folder)
    def require_mechanism_capabilities(self,ready):
        """An enabled mechanism needs the worker to declare it; never silently ignored."""
        capabilities=ready.get('capabilities',{})
        needed=[n for n in ('scene_handles','path_endpoint_reuse') if self.enabled(n)]
        if self.site.get('collision_check_mode')=='TWO_LEVEL':needed.append('two_level_collision')
        if self.site.get('sent_path_audit') in ('ADAPTIVE','ADAPTIVE_CONTACT'):needed.append('adaptive_sent_path_audit')
        if self.site.get('sent_path_audit')=='ADAPTIVE_CONTACT':needed.append('adaptive_contact_sent_path_audit')
        if self.site.get('solution_path_check','PIPELINE')=='CLEARANCE_PREDICATE':needed.append('clearance_predicate_solution_check')
        if self.site.get('native_ik_batch'):needed.append('sc_ik_batch')
        if self.native_planning():needed+=['composite_requests','native_prepare','attempt_journal']
        for name in needed:
            if not capabilities.get(name):raise ValueError('NATIVE_WORKER_UPDATE_REQUIRED_FOR_'+name.upper())
    def release_workers(self,folder):
        if self.enabled('resident_workers') and self.resident is not None:
            # Workers stay; every one is reset in the background for the next task.
            status=dict(resident=True,pool=self.resident.snapshot(),task_start=self.task_start)
            self.write_evidence(folder/'native-check-pool.json',status,'native-check-pool.json',ensure_ascii=False)
            self.client=None;self.pool=[];self.resident.end_task()
        elif self.deadline is None:
            self.close_check_pool()
            timing.write_json(folder/'native-check-pool.json',self.pool_status,'native-check-pool.json',ensure_ascii=False)
            if self.client is not None:self.client.close()
            self.client=None
        else:self.background_close(folder)
        self.scene_world=None
    def close(self):
        with self.task_lock:
            self.closed=True;self.task_generation+=1
            resident,self.resident=self.resident,None
            evidence,self.evidence=self.evidence,None
            client,self.client=self.client,None
        # No slow close holds the lifecycle lock. Late prewarm/task results
        # see the closed state/generation and cannot publish resources.
        if resident is not None:resident.close()
        if client is not None:client.close()
        self.close_check_pool()
        if evidence is not None:evidence.close()


def seam_band_contacts(world,box,rest):
    """Classify at-rest overlaps against the coplanar seam band (report only).

    The band of a seam edge lies between the seam position (RGB line / mask
    boundary) and its outer bound (+uncertainty). An overlap lies entirely in
    the band when the envelope with every seam edge retracted to its seam
    position no longer overlaps that object beyond PAYLOAD_REST_DEPTH_M.
    Returns None when the envelope has no seam edge. No permission is granted.
    """
    u=box.get('uncertainty') or {}
    seams=[i for i in u.get('intervals',[]) if i.get('method')=='COPLANAR_SEAM_MASK_RGB_EDGE_POLICY']
    if not seams or not rest['intersections']:return None
    lo=np.asarray(u['lower_m'],float).copy();hi=np.asarray(u['upper_m'],float).copy()
    for i in seams:
        retracted=i['outer_m']-i['side']*i['uncertainty_m']
        if i['side']<0:lo[i['axis']]=retracted
        else:hi[i['axis']]=retracted
    T=np.asarray((box.get('measured_box') or box)['pose'],float);pose=T.copy();pose[:3,3]+=T[:3,:3]@((lo+hi)/2)
    objects={o['id']:o for o in world};contacts=[]
    for hit in rest['intersections']:
        o=objects.get(hit['id'])
        inner=obb_overlap_depth(pose,(hi-lo)/2,o['pose'],np.asarray(o['size'])/2) if o and 'size' in o else np.inf
        contacts.append(dict(id=hit['id'],category=hit.get('category'),overlap_depth_m=hit['overlap_depth_m'],
            overlap_without_seam_band_m=float(inner),within_seam_band=bool(inner<=PAYLOAD_REST_DEPTH_M)))
    return dict(contacts=contacts,all_within_seam_band=all(c['within_seam_band'] for c in contacts),
        seam_edges=[dict(axis=i['axis'],side=i['side'],outer_m=i['outer_m'],uncertainty_m=i['uncertainty_m']) for i in seams],
        rule='ENVELOPE_WITH_SEAM_EDGES_RETRACTED_TO_SEAM_POSITION',permission_granted=False)
