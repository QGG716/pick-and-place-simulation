"""Cold native single-carton coordinator. Every motion is generated in MTC.

This module has no Python IK/RRT import or hardware execution provider.
Independent MuJoCo review remains available for strict selected plans.
"""
from __future__ import annotations
from copy import deepcopy
from time import perf_counter
import uuid
import json
import gzip
import hashlib
from pathlib import Path
import numpy as np

from .model import robot_for, transform, check_transform, ROOT
from .native_model import POLICY
from .native_scene import verify_scene, world_objects, validate_fixed_interfaces
from .native_contact import check_contours, validate_task_contract, top_grasp_candidates, face_outward, selected_ids, INDEPENDENT
from .native_cups import grasp_candidates
from .home_candidate import verify_candidate
from .mount_alignment import check_alignment
from ..moveit2_backend import ResidentMoveItClient, MoveItUnavailable, digest, validate_native_result, native_generation_evidence
from ..timing import JointMotionLimits, time_parameterize_joint_path, sample_quintic_knots
from ..moveit2_tcp import audit_linear_tcp
from .evaluation import evaluation_profile as checked_profile, validate_output_evaluation, STRICT, CHAIN, INTERPOLATION
from .transfer_policy import J1_POLICY, J1_EPSILON, j1_direction, j1_valid, stage_chain_valid, endpoint_fk, request_budgets, clear_stack_extractions, transfer_waypoints

STATUS = 'PLANNING_ONLY_NOT_EXECUTABLE'
PHASES = ['pregrasp', 'contact', 'extraction', 'transit', 'place', 'withdrawal', 'residence']
INDEPENDENT_PHASE = dict(pregrasp='approach', contact='contact', extraction='loaded_lift',
    transit='loaded_transfer', place='place_contact', withdrawal='retreat', residence='retreat')


def verify_ompl_output_interpolation(request,result):
    if 'ompl_interpolate_output' in request and (
            request['ompl_interpolate_output'] is not False or request.get('pipeline_id')!='ompl' or
            request.get('stage') not in ('pregrasp','transit','residence') or
            result.get('ompl_output_interpolation') is not False):
        raise ValueError('NATIVE_OMPL_OUTPUT_INTERPOLATION_MISMATCH')


def prioritize_grasp_face(geometry, preferred_face):
    """Change search order only; retain every candidate and its exact binding."""
    if preferred_face is None:return geometry
    if preferred_face not in ('top','front'):raise ValueError('INVALID_PREFERRED_GRASP_FACE')
    candidates=geometry['candidates']
    order=sorted(range(len(candidates)),key=lambda i:candidates[i]['target_face']!=preferred_face)
    return dict(geometry,candidates=[candidates[i] for i in order],
                search_order=dict(preferred_face=preferred_face,original_geometry_rank_indices=order,
                    semantics='STABLE_FACE_PRIORITY_NO_CANDIDATES_REMOVED_NO_PATH_IMPORTED'))


def extraction_goal(scene, target_pose, size, tcp, face='top', outward_m=None):
    """Retain the 10 mm lift clearance above crossed support surfaces."""
    half=np.asarray(size)/2; start=np.asarray(target_pose)[:3,3]
    task = scene.get('native_task')
    receive=(np.asarray(task['receiving']['box_pose_D'])[:2, 3] if task else
             np.asarray(scene['source_equipment_config']['conveyors']['longitudinal']['receive_center_xy_m']))
    lo=np.minimum(start[:2],receive)-half[:2]; hi=np.maximum(start[:2],receive)+half[:2]
    bottom=float(start[2]-np.abs(np.asarray(target_pose)[2,:3])@half)
    crossed=[]
    for row in world_objects(scene):
        if row['id'] not in scene['support_surface_ids']: continue
        pose = np.asarray(row['pose'])
        if 'convex_mesh' in row:
            vertices = np.asarray(row['convex_mesh']['vertices'])@pose[:3, :3].T+pose[:3, 3]
            lower, upper = vertices.min(axis=0), vertices.max(axis=0)
        else:
            half = np.abs(pose[:3, :3])@np.asarray(row['size'])/2
            lower, upper = pose[:3, 3]-half, pose[:3, 3]+half
        if np.all(upper[:2]>lo) and np.all(lower[:2]<hi):
            crossed.append(dict(id=row['id'],top_m=float(upper[2])))
    highest=max([bottom,*[r['top_m'] for r in crossed]])
    clearance = task['minimum_lift_m'] if task else .01
    lift=highest+clearance-bottom; goal=np.array(tcp,copy=True);goal[2,3]+=lift
    if face == 'front':
        outward_m=size[0]+clearance if outward_m is None else float(outward_m)
        if not np.isfinite(outward_m) or outward_m<clearance:raise ValueError('INVALID_FRONT_EXTRACTION_DISTANCE')
        goal[:3,3] += face_outward(target_pose, face)*outward_m
    return goal,dict(initial_bottom_m=bottom,crossed_supports=crossed,highest_support_m=highest,
        retained_lift_clearance_m=clearance,lift_m=lift,goal_bottom_m=bottom+lift,
        outward_m=outward_m if face=='front' else 0.,
        fully_before_original_front=bool(face=='front' and outward_m>=size[0]+clearance-1e-12),
        transit_requirement='FULL_PAYLOAD_NEIGHBOR_CLEARANCE_NO_SUPPORT_OR_NEIGHBOR_EXEMPTION')


def extraction_candidates(scene,target_pose,size,tcp,face='top'):
    clearance=scene.get('native_task',{}).get('minimum_lift_m',.01)
    distances=[None] if face=='top' else [size[0]+clearance,size[0]/2+clearance,size[0]/4+clearance,clearance]
    original=[extraction_goal(scene,target_pose,size,tcp,face,d) for d in distances]
    if face=='top' and scene.get('native_task',{}).get('cup_selection_mode')!=INDEPENDENT:
        return original
    # A short normal withdrawal can be reachable yet leave too little room
    # to turn between adjacent cartons. Also try clearing the neighboring
    # stack height, and a halfway lift, before abandoning this cup selection.
    bottom=float(np.asarray(target_pose)[2,3]-np.abs(np.asarray(target_pose)[2,:3])@ (np.asarray(size)/2))
    tops=[]
    for box in scene['boxes']:
        if box['id']==scene['target_box_id']:continue
        T=transform(box['pose']);tops.append(float(T[2,3]+np.abs(T[2,:3])@(np.asarray(box['size_m'])/2)))
    base_lift=original[0][1]['lift_m'];above=max(base_lift,max(tops,default=bottom)+clearance-bottom)
    extra=[]
    # The actual 80 mm lift is reachable at reduced LIN speed where clearing
    # the full 100 mm neighboring carton is not. It still requires an ordinary
    # payload/world-checked transit; partial height is never an exemption.
    partial=max(base_lift,.8*(max(tops,default=bottom)-bottom))
    for lift in dict.fromkeys([above,partial,(above+base_lift)/2]):
        if lift<=base_lift+1e-9:continue
        # Top suction also needs room to turn the attached carton. Actual PTP
        # and OMPL attempts from the 10 mm lift were rejected on neighbors.
        # Keep top extraction vertical and check the same finite height set
        # with native LIN before an ordinary, fully checked free transit.
        distances=(None,) if face=='top' else (size[0]/2+clearance,size[0]+clearance,size[0]/4+clearance)
        for distance in distances:
            goal,evidence=extraction_goal(scene,target_pose,size,tcp,face,distance)
            goal[2,3]+=lift-evidence['lift_m']
            evidence.update(lift_m=lift,goal_bottom_m=bottom+lift,neighbor_top_m=max(tops,default=bottom),
                lift_basis='BOUNDED_NEIGHBOR_HEIGHT_CANDIDATE_NOT_PATH_ACCEPTANCE')
            extra.append((goal,evidence))
    return extra+original


class NativeTask:
    def __init__(self, snapshot, command, *, task_seconds=900., stage_seconds=60., attempt=0, cancelled=None, home_candidate=None,
                 progress_path=None, rpc_seconds=180., preferred_grasp_face=None, evaluation_profile=STRICT, search_trace=False, trace_stages=('transit',), trace_requests=(), preferred_candidate_id=None, continuous_task_mode=None):
        self.evaluation_profile = checked_profile(evaluation_profile)
        from .native_io import verify_snapshot
        verify_snapshot(snapshot)
        self.snapshot = snapshot; self.scene = verify_scene(snapshot['scene'])
        self.model = snapshot['native_model']; self.init = deepcopy(self.model['init']); self.identity = self.init['identity']
        self.init['evaluation_profile'] = self.evaluation_profile
        if continuous_task_mode is not None:
            if continuous_task_mode != 'eco65_coupled_cycle_v1' or not self.scene.get('coupled_conveyor_spec'):
                raise ValueError('CONTINUOUS_TASK_SCOPE_INVALID')
            self.init['continuous_task_mode'] = continuous_task_mode
        self.directed_j1=self.evaluation_profile==CHAIN
        self.init['directed_j1_enabled']=self.directed_j1
        self.init['search_trace_enabled']=bool(search_trace)
        self.trace_stages=tuple(trace_stages);self.trace_requests=tuple(trace_requests)
        self.preferred_candidate_id=preferred_candidate_id
        self.extraction_branch_id=None
        self.final_home=None
        if self.identity['scene_fingerprint'] != self.scene['scene_fingerprint']: raise ValueError('MODEL_SCENE_MISMATCH')
        self.task_contract = validate_task_contract(self.scene)
        if preferred_grasp_face is not None and (not self.task_contract or
                self.task_contract.get('cup_selection_mode')!=INDEPENDENT or
                preferred_grasp_face not in self.task_contract['allowed_faces']):
            raise ValueError('PREFERRED_GRASP_FACE_NOT_IN_TASK_CONTRACT')
        self.preferred_grasp_face=preferred_grasp_face
        if self.init['process_geometry'].get('task_contract') != self.task_contract:
            raise ValueError('MODEL_TASK_CONTRACT_MISMATCH')
        if self.init['process_geometry'].get('fixed_attachment_interfaces', []) != validate_fixed_interfaces(self.scene):
            raise ValueError('MODEL_FIXED_ATTACHMENT_INTERFACES_MISMATCH')
        if not np.isfinite([task_seconds, stage_seconds,rpc_seconds]).all() or min(task_seconds, stage_seconds,rpc_seconds) <= 0:
            raise ValueError('INVALID_NATIVE_BUDGET')
        self.robot = robot_for(snapshot['tool'], self.scene)
        self.home_candidate = None if home_candidate is None else deepcopy(verify_candidate(home_candidate, snapshot['tool'], self.scene))
        self.home_checks = []; self.native_calls = []
        self.progress_path = Path(progress_path).resolve() if progress_path else None
        if self.progress_path: self.progress_path.parent.mkdir(parents=True, exist_ok=True)
        self.call_archive = self.progress_path.with_name('native_calls.jsonl.gz') if self.progress_path else None
        self.extraction_evidence = None
        self.extraction_attempts = []
        self.approach_attempts = []
        self.grasp_geometry = None; self.grasp_failures = []; self.grasp_home_rejections = []
        self.grasp_search = dict(status='NOT_STARTED', candidate_count=0,
                                 attempted_indices=[], unsearched_indices=[])
        self.cancelled = cancelled or (lambda: False)
        self.stage_seconds = stage_seconds; self.rpc_seconds=rpc_seconds; self.deadline = perf_counter()+task_seconds
        self.task_id = 'eco65-'+uuid.uuid4().hex
        self.counter = 10000*attempt; self.parent = ''; self.attachment = None
        self.target = deepcopy(self.scene['box']); self.target_pose = transform(self.target['pose'])
        self.face = 'top'; self.active_ids = list(self.init['process_geometry']['expected_contour_ids'])
        self.candidate_id = 'INITIAL_STATE_DIAGNOSTIC'
        self.records = []; self.ik_records = []; self.stages = []; self.events = []
        started = perf_counter(); self.native = ResidentMoveItClient(command)
        try:
            self.ready = self.call(self.init)
            if self.ready.get('status') != 'READY' or self.ready.get('identity') != self.identity:
                raise MoveItUnavailable('NATIVE_MODEL_IDENTITY_REJECTED')
            self.initialization_s = perf_counter()-started
        except Exception as error:
            error.native_calls = self.native_calls
            self.native.close(); raise

    def close(self): self.native.close()

    def call(self, request):
        remaining = self.deadline-perf_counter()
        if self.cancelled(): raise MoveItUnavailable('CANCELLED')
        if remaining <= 0: raise MoveItUnavailable('NATIVE_TASK_BUDGET_EXHAUSTED')
        request = deepcopy(request)
        profile = getattr(self, 'evaluation_profile', STRICT)
        if checked_profile(request.get('evaluation_profile', profile)) != profile:
            raise ValueError('EVALUATION_PROFILE_MISMATCH')
        request['evaluation_profile'] = profile
        if request['op'] in ('plan','diagnostic_connection','eco65_transition') and profile == CHAIN:
            request.update(request_budgets(remaining,self.stage_seconds,self.rpc_seconds))
        elif request['op'] != 'init': request['allowed_planning_time_s'] = min(self.stage_seconds, remaining)
        if request['op']=='plan' and self.init.get('search_trace_enabled'):
            request['search_trace']=(request['stage'] in self.trace_stages and
                (not self.trace_requests or request['stage_id'] in self.trace_requests))
        request['extraction_branch_id']=getattr(self,'extraction_branch_id',None)
        started = perf_counter()
        record = dict(request=request)
        summary = (dict(request={k:request[k] for k in ('op','stage','stage_id','evaluation_profile') if k in request},
                        archive_index=len(self.native_calls), request_sha256=digest(request))
                   if self.call_archive else record)
        self.native_calls.append(summary)
        def progress(event, **extra):
            if self.progress_path:
                with self.progress_path.open('a', encoding='utf-8') as stream:
                    stream.write(json.dumps(dict(event=event, call_index=len(self.native_calls)-1,
                        evaluation_profile=profile,
                        op=request['op'], stage=request.get('stage'), stage_id=request.get('stage_id'),
                        **extra), allow_nan=False)+'\n')
        progress('STARTED')
        try:
            # Native solve time excludes scene import and the profile's output
            # path check. The RPC watchdog must cover those operations.
            result = self.native.request(request, timeout=request.get('request_budget_s',min(remaining,self.rpc_seconds))+10., cancelled=self.cancelled)
            if request['op'] in ('plan','diagnostic_connection'):
                self._last_motion_request=dict(request,request_id=result.get('request_id'))
            if 'search_trace' in result:
                result['search_trace'].update(resident_seed=self.ready['seed'],request_seed=request.get('seed'),
                    rng_semantics='OMPL_RNG_SEEDED_ONCE_AT_RESIDENT_INIT_REQUEST_SEED_IS_A_REQUEST_LABEL',
                    resident_seed_source='SAME_RESIDENT_READY_RESPONSE')
            if 'search_trace' in result and self.progress_path:
                archive_started=perf_counter()
                trace=result.pop('search_trace');directory=self.progress_path.parent/'search_traces';directory.mkdir(exist_ok=True)
                path=directory/(request['stage_id']+'.json.gz')
                with gzip.open(path,'wt',encoding='utf-8') as stream:json.dump(trace,stream,separators=(',',':'),allow_nan=False)
                result['search_trace_ref']=dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    status=trace['status'],request_id=result.get('request_id'),archive_wall_s=perf_counter()-archive_started)
            record['result'] = result
            if checked_profile(result.get('evaluation_profile', STRICT)) != profile:
                raise ValueError('NATIVE_RESPONSE_EVALUATION_PROFILE_MISMATCH')
            progress('COMPLETED', status=result.get('status'), wall_s=perf_counter()-started,
                process_failure=(result.get('process') or {}).get('last_failure'),
                clearance_failures=(result.get('clearance') or {}).get('rejected_examples'))
            return result
        except Exception as error:
            record['error'] = str(error)
            progress('FAILED', error=str(error), wall_s=perf_counter()-started)
            raise
        finally:
            record['wall_s'] = perf_counter()-started
            if self.call_archive:
                # Preserve every complete request/response, without duplicating
                # the multi-megabyte immutable world in every failure/report.
                with gzip.open(self.call_archive,'at',encoding='utf-8',compresslevel=1) as stream:
                    stream.write(json.dumps(record,separators=(',',':'),allow_nan=False)+'\n')
                summary.update(wall_s=record['wall_s'],archive=self.call_archive.relative_to(ROOT).as_posix())
                if 'result' in record:summary.update(result_sha256=digest(record['result']),result={'status':record['result'].get('status')})
                if 'error' in record:summary['error']=record['error']

    def call_archive_info(self):
        if not self.call_archive or not self.call_archive.exists():return None
        return dict(schema='LOSSLESS_NATIVE_CALLS_GZIP_JSONL_V1',path=self.call_archive.relative_to(ROOT).as_posix(),
                    sha256=hashlib.sha256(self.call_archive.read_bytes()).hexdigest(),records=len(self.native_calls))

    def request(self, q, stage, *, op='check_state_offline', endpoint=False):
        world = world_objects(self.scene)
        for b in self.scene['boxes']:
            if b['id'] == self.target['id'] and self.attachment is not None: continue
            pose = self.target_pose if b['id'] == self.target['id'] else transform(b['pose'])
            world.append(dict(id=b['id'], size=b['size_m'], pose=pose.tolist(), category='carton'))
        target_pose = (self.robot.named_link_frames(q)['link_6'] @ self.attachment
                       if self.attachment is not None else self.target_pose)
        target = dict(id=self.target['id'], size=self.target['size_m'], pose=target_pose.tolist())
        attached = None if self.attachment is None else {**target, 'pose': self.attachment.tolist(),
                    'touch_links': self.init['compliant_tool_links']}
        task = self.scene.get('native_task')
        support_source = task['original_support_names'] if task else self.target['supported_by']
        support_receiver = task['receiving']['support_ids'] if task else ['conveyor_longitudinal_belt']
        supports = support_source if stage == 'extraction' else support_receiver if stage == 'place' else []
        process = dict(self.init['process_geometry'], schema='eco65_native_process_v1', stage=stage,
            target_id=self.target['id'], target=target, target_face=self.face, support_names=supports,
            stack_carton_ids=[b['id'] for b in self.scene['boxes']], initial_proximity=None,
            max_attachment_gap_m=.0004, maximum_penetration_m=.0004, suction_edge_margin_m=.003,
            max_normal_misalignment_rad=self.snapshot['tool']['contact']['normal_tolerance_rad'], collision_policy=self.init['collision_policy'])
        if self.init['process_geometry'].get('cup_selection_mode') == INDEPENDENT:
            process.update(active_ids=self.active_ids, selection_binding=self.selection_binding())
        clearance = dict(schema='eco65_native_process_clearance_v1', source_policy=self.init['collision_policy'], stage=stage,
            tool_links=self.init['tool_links'], compliant_tool_links=self.init['compliant_tool_links'],
            target_id=self.target['id'], payload_id='' if attached is None else self.target['id'],
            conveyor_ids=sorted({o['id'] for o in self.scene['obstacles'] if o.get('owner', '').startswith('conveyor')}
                               | set(support_receiver)),
            receiver_reserve_m=0., numerical_gap_tolerance_m=1e-9, edge_resolution_rad=.002)
        return dict(op=op, identity=self.identity, q_start=np.asarray(q).tolist(), world=world,
            evaluation_profile=getattr(self, 'evaluation_profile', STRICT),
            scene_fingerprint=digest(world), attachment=attached, stage=stage, task_id=self.task_id,
            parent_stage_id=self.parent, seed=6518+self.counter, start_velocity=[0.]*6, path_constraints={},
            allowed_pairs=[['base_link', 'robot_mount_plate']], process_policy=process, clearance_policy=clearance,
            clearance_mode='optimized', endpoint=endpoint, cancelled=False,
            j1_direction=self.direction_for_stage(stage),
            extraction_branch_id=getattr(self,'extraction_branch_id',None),
            flange_from_task_tcp=self.identity['flange_from_task_tcp'], planning_only=True,
            planning_mode='cold_from_scratch', history_enabled=False, velocity_scale=.5, acceleration_scale=.5)

    def selection_binding(self):
        return dict(active_ids=list(self.active_ids), target_id=self.target['id'], target_face=self.face,
                    candidate_id=self.candidate_id, task_id=self.task_id, identity=self.identity)

    def direction_for_stage(self,stage):
        return j1_direction(stage) if getattr(self,'directed_j1',False) else 0

    def direction_goal_valid(self,start,end,stage):
        return j1_valid([start,end],self.direction_for_stage(stage))

    def select_grasp(self, grasp):
        self.face = grasp.get('target_face', 'top')
        self.active_ids = selected_ids(self.init['process_geometry'], grasp.get('active_ids'))
        self.candidate_id = grasp.get('candidate_id', 'LEGACY_ALL_CUPS')

    def check_fk(self, home):
        reports = []
        for q in [np.zeros(6), home, [.2, -.3, .4, .5, -.6, .7], [-.4, .2, -.35, -.6, .4, -.8]]:
            frames = self.robot.named_link_frames(q)
            reply = self.call(dict(op='fk', identity=self.identity, q_start=np.asarray(q).tolist(),
                links=list(frames)+[self.identity['task_tcp_link']]))
            expected = {**frames, self.identity['task_tcp_link']: self.robot.fk(q)}
            error = max(float(np.max(np.abs(np.asarray(reply['frames'][name])-value))) for name, value in expected.items())
            if error > 1e-8: raise ValueError('ECO65_NATIVE_FK_MISMATCH')
            reports.append(dict(q_rad=np.asarray(q).tolist(), max_matrix_error=error))
        return reports

    def ik(self, pose, seed, stage, count=32):
        rng = np.random.default_rng(6518+self.counter)
        unique = []
        for q in [np.asarray(seed), *rng.uniform(self.robot.joint_limits[:, 0], self.robot.joint_limits[:, 1], (count-1, 6))]:
            self.counter += 1
            req = dict(op='ik', identity=self.identity, q_start=q.tolist(), goal_pose=check_transform(pose).tolist(),
                evaluation_profile=getattr(self, 'evaluation_profile', STRICT),
                seed=6518+self.counter, planning_only=True, planning_mode='cold_from_scratch',
                position_tolerance_m=3e-5, orientation_tolerance_rad=3e-4)
            result = self.call(req); self.ik_records.append(dict(stage=stage, request=req, result=result))
            if result.get('native_ik_calls') != 1: raise ValueError('NATIVE_IK_EVIDENCE_MISSING')
            if result['status'] == 'SUCCESS':
                value = np.asarray(result['q'], float)
                if value.shape != (6,) or not np.isfinite(value).all(): continue
                if np.any(value < self.robot.joint_limits[:, 0]) or np.any(value > self.robot.joint_limits[:, 1]): continue
                # Deduplicate before the expensive immutable-world state check,
                # not only after collecting all candidate goals for PTP.
                if any(np.max(np.abs(value-other)) <= 1e-5 for other in unique): continue
                unique.append(value.copy())
                checked = self.call(self.request(value, stage))
                if checked['status'] == 'SUCCESS': yield value

    def motion(self, q, stage, *, pose=None, goal=None, ik_seed=None, return_compatible=False):
        constrained = stage in {'contact', 'extraction', 'place', 'withdrawal'}
        solvers = [('pilz_industrial_motion_planner', 'LIN' if constrained else 'PTP', .5)]
        # Pilz can reject a valid Cartesian branch on a joint velocity or
        # deceleration limit. One slower native retry retains all geometric,
        # rate, provenance and profile-specific output checks.
        if constrained: solvers.append(('pilz_industrial_motion_planner', 'LIN', .25))
        else: solvers.append(('ompl', 'RRTConnectkConfigDefault', .5))
        if constrained: solvers.append(('pilz_industrial_motion_planner','LIN',.125))
        goals = [None] if constrained else ([np.asarray(goal)] if goal is not None else list(self.ik(pose, q if ik_seed is None else ik_seed, stage)))
        if not constrained:
            if stage=='transit' and pose is not None and self.evaluation_profile==CHAIN:
                # Finite elbow/wrist seeds cover both shoulder branches, including
                # the negative-J1 preplace branch needed for a monotone HOME return.
                # Only native IK and the unchanged state checker produce goals.
                home_seed=getattr(self,'final_home',None)
                for elbow in (-1.,1.):
                    for wrist in (-1.,1.):
                        seed=np.array([q[0] if home_seed is None else home_seed[0],1.5,elbow*1.5,
                                       -elbow*1.5,wrist*np.pi/2,q[5]])
                        seed=np.clip(seed,self.robot.joint_limits[:,0],self.robot.joint_limits[:,1])
                        goals.extend(self.ik(pose,seed,stage,count=1))
                # Ask native IK for a closer finite wrist branch. These are seeds,
                # never edited solutions or +/-2pi post-processing of a path.
                for value in list(goals)[:8]:
                    seed=value.copy();seed[3]=q[3];seed[5]=q[5]
                    goals.extend(self.ik(pose,seed,stage,count=1))
            goals.sort(key=lambda candidate: float(np.abs(candidate-q).sum()))
            unique=[]
            for candidate in goals:
                # IK numerical replicas are one branch, not repeated expensive
                # OMPL attempts. This does not change endpoint acceptance.
                if all(np.max(np.abs(candidate-other))>1e-4 for other in unique): unique.append(candidate)
            goals=unique
            if getattr(self,'directed_j1',False):
                goals=[candidate for candidate in goals if self.direction_goal_valid(q,candidate,stage)]
                # Preplace must allow the *subsequent* nondecreasing return to fixed HOME.
                if return_compatible and getattr(self,'final_home',None) is not None:
                    goals=[candidate for candidate in goals if candidate[0]<=self.final_home[0]+J1_EPSILON]
        # A first valid IK branch can require a long turn through a wall. Give
        # the existing bounded native IK set its PTP attempts before spending
        # the OMPL budget on any branch; never substitute Python motion search.
        for pipeline, planner, scale in solvers:
            if scale==.125 and (not self.records or (self.records[-1]['result'].get('failure') or {}).get('reason')!='OUTPUT_ENDPOINT_TCP_GOAL_MISMATCH'):
                continue
            for joint_goal in goals:
                self.counter += 1; req = self.request(q, stage, op='plan')
                req.update(stage_id=f'{stage}-{self.counter}', pipeline_id=pipeline, planner_id=planner,
                    require_native_motion=True,velocity_scale=scale,acceleration_scale=scale)
                if pipeline=='ompl' and stage=='transit':
                    req['clearance_policy']['search_payload_reserve_m']=.001
                if pipeline=='ompl' and self.init['process_geometry'].get('cup_selection_mode')==INDEPENDENT:
                    # Keep the native simplified path, without expanding it
                    # into thousands of extra timing knots. Search precision,
                    # profile-specific FCL output checking remains.
                    req['ompl_interpolate_output']=False
                if constrained: req['goal_pose'] = check_transform(pose).tolist()
                else: req['q_goal'] = joint_goal.tolist()
                started = perf_counter(); raw = self.call(req)
                submitted=getattr(self,'_last_motion_request',{})
                if submitted.get('stage_id')==req['stage_id']:req=submitted
                record=dict(request=req,result=raw,wall_s=perf_counter()-started)
                if self.evaluation_profile==CHAIN and getattr(self,'call_archive',None):
                    # Full immutable requests/results live once in the lossless
                    # archive; candidate backtracking carries small references.
                    record.update(request={k:req[k] for k in ('stage','stage_id','parent_stage_id','q_start','q_goal','goal_pose','pipeline_id','planner_id') if k in req},
                        result={k:raw[k] for k in ('status','solver_message','native_output_status','failure','basic_path_check','search_trace_ref','native_solver_s') if k in raw},
                        archive_index=len(self.native_calls)-1,archive=self.call_archive.relative_to(ROOT).as_posix())
                self.records.append(record)
                if raw['status'] != 'SUCCESS': continue
                verify_ompl_output_interpolation(req,raw)
                if (not native_generation_evidence(raw, req) or raw.get('mtc_generation') is not True
                        or raw.get('generated_during_task') is not True or raw.get('task_id') != self.task_id
                        or raw.get('parent_stage_id') != self.parent or raw.get('stage_id') != req['stage_id']):
                    raise ValueError('NATIVE_MTC_PROVENANCE_MISSING')
                path = validate_native_result(raw, q, joint_goal, self.init['joint_names'])
                validate_output_evaluation(self.evaluation_profile, req, raw)
                if constrained and self.evaluation_profile == STRICT:
                    audit = audit_linear_tcp(path, self.robot.fk, self.robot.fk(q), pose,
                        position_tolerance=.0001, orientation_tolerance=.001, edge_resolution_rad=.002)
                    if not audit['passed']: continue
                limits = self.model['simulation_limits']
                velocity = [min(float(self.model['official_limits'][name]['velocity']), limits['joint_velocity_rad_s']) for name in self.init['joint_names']]
                timed = time_parameterize_joint_path(path, JointMotionLimits(np.array(velocity),
                    np.full(6, limits['joint_acceleration_rad_s2']), np.full(6, limits['joint_jerk_rad_s3'])))
                self.stages.append(dict(stage=stage, stage_id=req['stage_id'], parent_stage_id=self.parent,
                    evaluation_profile=self.evaluation_profile,
                    python_linear_audit='NOT_RUN_PROFILE' if self.evaluation_profile == CHAIN else ('PASS' if constrained else 'NOT_APPLICABLE'),
                    q=timed.positions.tolist(), times=timed.time_from_start.tolist(), native_result=raw,
                    native_request=req, attachment=None if self.attachment is None else self.attachment.tolist(),
                    target_pose_start=self.target_pose.tolist()))
                self.parent = req['stage_id']
                return np.asarray(path[-1])
        raise ValueError('NATIVE_STAGE_CANDIDATES_EXHAUSTED:'+stage)

    def select_home(self, pre):
        q = np.asarray(self.home_candidate['q_rad'] if self.home_candidate else
                       self.scene['source_equipment_config']['task']['initial_seed_q_rad'], float)
        row = dict(source='EXPLICIT_CANDIDATE' if self.home_candidate else 'CONFIGURED_SEED', q_rad=q.tolist(),
                   scene_fingerprint=self.scene['scene_fingerprint'], candidate=self.home_candidate)
        self.home_checks.append(row)
        e = self.snapshot['tool']['mounting_evidence']
        row['mount_alignment'] = check_alignment(e['official_bolt_centers_xy_m'], e['bolt_centers_cad_xz_m'],
            e['mount_point_cad_m'][1], self.snapshot['tool']['T_flange_tool_cad'])
        if row['mount_alignment']['status'] != 'PASS': raise ValueError('HOME_MOUNT_INTERFACE_INVALID')
        in_limits = (q.shape == (6,) and np.isfinite(q).all() and
            np.all(q >= self.robot.joint_limits[:, 0]) and np.all(q <= self.robot.joint_limits[:, 1]))
        row['joint_limits'] = 'PASS' if in_limits else 'FAIL'
        if in_limits:
            row['native_state'] = self.call(self.request(q, 'pregrasp'))
            failure = (row['native_state'].get('process') or {}).get('last_failure') or {}
            fixed = {r['attachment_id'] for r in self.scene.get('fixed_attachment_interfaces', [])}
            if 'link_1' in failure.get('pair', []) and fixed.intersection(failure.get('pair', [])):
                row['status'] = 'REJECTED'
                raise ValueError('FIXED_J1_MOUNT_REPRESENTATION_REJECTED:'+str(failure))
        if not in_limits or row['native_state']['status'] != 'SUCCESS':
            row['status'] = 'REJECTED'
            q = next(self.ik(pre, q if in_limits else np.mean(self.robot.joint_limits, axis=1), 'pregrasp', 64), None)
            if q is None: raise ValueError('NATIVE_HOME_CANDIDATES_EXHAUSTED')
            row = dict(source='BOUNDED_NATIVE_IK_SEARCH', q_rad=q.tolist(), max_candidates=64,
                       scene_fingerprint=self.scene['scene_fingerprint'])
            self.home_checks.append(row)
        try: row['fk_crosscheck'] = self.check_fk(q)
        except Exception as error:
            row.update(status='REJECTED', reason=str(error))
            raise  # Inconsistent FK is a model/worker failure, not an IK search miss.
        row['status'] = 'ACCEPTED_SIMULATION_INITIAL_STATE'
        return q, row['fk_crosscheck']

    def run_grasp(self, grasp, home):
        """Generate one complete branch from the frozen initial state."""
        contract = self.scene.get('native_task')
        top = check_transform(grasp['tcp_pose_D'])
        self.select_grasp(grasp)
        # A free pregrasp can be reachable while inactive cups collide with
        # lower/neighbor cartons at contact. Check the actual contact IK and
        # complete world before spending a dense audit on the free approach.
        contact_seed=None
        for value in self.ik(top,home,'contact'):
            probe=check_contours(self.init['process_geometry'],self.robot.named_link_frames(value)['link_6'],
                self.target_pose,self.target['size_m'],gap=.0004,penetration=.0004,
                angle=self.snapshot['tool']['contact']['normal_tolerance_rad'],face=self.face,active_ids=self.active_ids)
            if probe['status']=='PASS':contact_seed=value;break
        if contact_seed is None:raise ValueError('NATIVE_STAGE_CANDIDATES_EXHAUSTED:contact_pose')
        distance=contract['preapproach_m'] if contract else .03
        offsets=[distance]
        if contract and contract.get('cup_selection_mode')==INDEPENDENT:
            offsets.extend(d for d in [.025,.010,.005] if d<distance)
        # Real contact IK can pass while the 30 mm normal-offset pose fails.
        # Every shorter approach still requires a new complete HOME connection
        # and native LIN contact. This never imports a diagnostic short path.
        approach_parent=self.parent;approach_stages=list(self.stages)
        for offset in offsets:
            self.parent=approach_parent;self.stages=list(approach_stages)
            pre=top.copy();pre[:3,3]+=face_outward(self.target_pose,self.face)*offset
            # Keep a distinct approach if the configured HOME already equals it.
            if np.linalg.norm(self.robot.fk(home)[:3,3]-pre[:3,3])<1e-4:
                pre[:3,3]+=face_outward(self.target_pose,self.face)*.005
            attempt=dict(normal_offset_m=offset,goal_tcp_D=pre.tolist(),
                         native_record_start=len(self.records),status='SEARCHING')
            self.approach_attempts.append(attempt)
            try:
                q=self.motion(home.copy(),'pregrasp',pose=pre,ik_seed=contact_seed)
                q=self.motion(q,'contact',pose=top)
                seal=check_contours(self.init['process_geometry'],self.robot.named_link_frames(q)['link_6'],
                    self.target_pose,self.target['size_m'],gap=.0004,penetration=.0004,
                    angle=self.snapshot['tool']['contact']['normal_tolerance_rad'],face=self.face,active_ids=self.active_ids)
                if seal['status']!='PASS':raise ValueError('CONTACT_ENDPOINT_CONTOURS_REJECTED')
                attempt.update(status='HOME_APPROACH_AND_CONTACT_PASS',native_record_end=len(self.records));break
            except ValueError as error:
                if str(error) not in ('NATIVE_STAGE_CANDIDATES_EXHAUSTED:pregrasp',
                        'NATIVE_STAGE_CANDIDATES_EXHAUSTED:contact','CONTACT_ENDPOINT_CONTOURS_REJECTED'):raise
                attempt.update(status='REJECTED',reason=str(error),native_record_end=len(self.records))
        else:raise ValueError('NATIVE_STAGE_CANDIDATES_EXHAUSTED:approach_contact')
        self.attachment = np.linalg.inv(self.robot.named_link_frames(q)['link_6']) @ self.target_pose
        self.events.append(dict(event='ATTACH_PREDICTED', after_stage=self.parent,
            target_id=self.target['id'], flange_from_target=self.attachment.tolist(), q=q.tolist(),
            selection_binding=self.selection_binding(), actual_sealed_ids=seal['actual_sealed_ids']))
        if contract:
            place_box = check_transform(contract['receiving']['box_pose_D'])
        else:
            cfg = self.scene['source_equipment_config']['conveyors']
            center = [*cfg['longitudinal']['receive_center_xy_m'], cfg['surface_z_m']+self.target['size_m'][2]/2]
            place_box = transform(center)
        place_tcp = place_box @ np.linalg.inv(self.attachment) @ np.asarray(self.identity['flange_from_task_tcp'])
        preplace = place_tcp.copy(); preplace[2, 3] += contract['preplace_m'] if contract else .03
        contact_q=q.copy();contact_parent=self.parent;contact_stages=list(self.stages)
        self.final_home=np.asarray(home)
        last_error='NATIVE_STAGE_CANDIDATES_EXHAUSTED:extraction'
        lifts=extraction_candidates(self.scene,self.target_pose,self.target['size_m'],self.robot.fk(q),self.face)
        if self.evaluation_profile==CHAIN:
            front_lifts=clear_stack_extractions(self.scene,self.target_pose,self.target['size_m'],self.robot.fk(q),self.face)
            front_lifts.sort(key=lambda item:-item[1].get('turn_envelope_extra_m',0.))
            lifts=front_lifts+lifts
        for extraction_index,(lift,evidence) in enumerate(lifts):
            self.extraction_branch_id=f'{self.candidate_id}:extraction-{extraction_index}'
            self.parent=contact_parent;self.stages=list(contact_stages)
            self.extraction_evidence=None
            attempt=dict(geometry=evidence,goal_tcp_D=lift.tolist(),native_record_start=len(self.records),status='SEARCHING')
            self.extraction_attempts.append(attempt)
            try:
                value=self.motion(contact_q,'extraction',pose=lift)
                self.extraction_evidence=evidence
                q=self.transfer(value,preplace)
                attempt.update(status='EXTRACTION_AND_TRANSIT_PASS',native_record_end=len(self.records));break
            except ValueError as error:
                if str(error) not in ('NATIVE_STAGE_CANDIDATES_EXHAUSTED:extraction','NATIVE_STAGE_CANDIDATES_EXHAUSTED:transit'):raise
                attempt.update(status='REJECTED',reason=str(error),native_record_end=len(self.records))
                last_error=str(error)
        else:raise ValueError(last_error)
        q = self.motion(q, 'place', pose=place_tcp)
        self.target_pose = self.robot.named_link_frames(q)['link_6'] @ self.attachment
        if not supported(self.scene, self.target_pose): raise ValueError('SUPPORTED_RELEASE_REJECTED')
        self.events.append(dict(event='SUPPORTED_RELEASE_PREDICTED', after_stage=self.parent,
            target_id=self.target['id'], pose=self.target_pose.tolist(), q=q.tolist(), selection_binding=self.selection_binding()))
        self.attachment = None
        retreat = self.robot.fk(q).copy()
        retreat[:3, 3] += face_outward(self.target_pose,self.face)*(contract['withdrawal_m'] if contract else .03)
        q = self.motion(q, 'withdrawal', pose=retreat)
        q = self.motion(q, 'residence', goal=home)
        return seal

    def transfer(self,q,preplace,*,skip_direct=False):
        """Each route edge is a separate native MTC stage, with its real parent."""
        prefix=list(self.stages);parent=self.parent
        routes=[]
        if self.evaluation_profile==CHAIN:
            if self.final_home is not None:
                # Current legal HOME supplies a reachable pose proposal, not a
                # historical motion. Loaded IK, all edges and each native parent
                # are regenerated with the current attachment/world.
                routes.append(dict(tcp_pose=self.robot.fk(self.final_home),seed=self.final_home,
                    basis='CURRENT_HOME_POSE_WITH_CURRENT_PAYLOAD_RECHECKED'))
            if not skip_direct:routes.append(None)
            routes+=transfer_waypoints(self.scene,self.target['size_m'],self.attachment,np.asarray(self.identity['flange_from_task_tcp']))
        elif not skip_direct:routes.append(None)
        for route in routes:
            self.stages=list(prefix);self.parent=parent
            try:
                value=q
                if route is not None:value=self.motion(value,'transit',pose=route['tcp_pose'],ik_seed=route.get('seed'))
                return self.motion(value,'transit',pose=preplace,return_compatible=True)
            except ValueError as error:
                if str(error)!='NATIVE_STAGE_CANDIDATES_EXHAUSTED:transit':raise
        raise ValueError('NATIVE_STAGE_CANDIDATES_EXHAUSTED:transit')

    def run(self):
        if self.task_contract and self.task_contract.get('coverage_context'):
            raise ValueError('CONDITIONAL_COVERAGE_IS_NOT_A_COMPLETE_TASK')
        started = perf_counter()
        contract = self.scene.get('native_task')
        initial_target_pose = self.target_pose.copy()
        normal_tolerance = self.snapshot['tool']['contact']['normal_tolerance_rad']
        if contract and contract.get('cup_selection_mode') == INDEPENDENT:
            self.grasp_geometry = grasp_candidates(self.init['process_geometry'],initial_target_pose,
                self.target['size_m'],self.identity['flange_from_task_tcp'],self.scene['base_position_m'],
                target_id=self.target['id'],faces=contract['allowed_faces'],angle=normal_tolerance,
                neighbor_boxes=self.scene['boxes'])
        elif contract:
            self.grasp_geometry = top_grasp_candidates(self.init['process_geometry'],
                initial_target_pose, self.target['size_m'], self.identity['flange_from_task_tcp'],
                self.scene['base_position_m'], margin=.003, angle=normal_tolerance)
        else:
            # Preserve the existing four-carton branch's centred top grasp.
            top = initial_target_pose@transform([0, 0, self.target['size_m'][2]/2], np.diag([1., -1., -1.]))
            seal = check_contours(self.init['process_geometry'],
                top@np.linalg.inv(np.asarray(self.identity['flange_from_task_tcp'])),
                initial_target_pose, self.target['size_m'], gap=.0004, penetration=.0004,
                angle=normal_tolerance)
            self.grasp_geometry = dict(candidates=[] if seal['status'] != 'PASS' else [dict(
                kind='legacy_face_center', roll_rad=0., offset_tcp_xy_m=[0.,0.], tcp_pose_D=top.tolist(),
                nominal_contact_geometry=seal, qualification='GEOMETRY_ONLY_NOT_IK_OR_PATH_VALIDATED')],
                rejected=[] if seal['status'] == 'PASS' else [seal], checked_contour_count=12,
                edge_margin_m=.003, method='LEGACY_CENTRED_TOP_GRASP')
        self.grasp_geometry=prioritize_grasp_face(self.grasp_geometry,getattr(self,'preferred_grasp_face',None))
        candidates = self.grasp_geometry['candidates']
        if getattr(self,'preferred_candidate_id',None):
            if not any(g.get('candidate_id')==self.preferred_candidate_id for g in candidates):raise ValueError('PREFERRED_CANDIDATE_NOT_IN_CURRENT_GEOMETRY')
            candidates.sort(key=lambda g:g.get('candidate_id')!=self.preferred_candidate_id)
        if getattr(self,'progress_path',None):
            self.progress_path.with_name('grasp_candidates.json').write_text(
                json.dumps(self.grasp_geometry,separators=(',',':'),allow_nan=False),encoding='utf-8')
        self.grasp_search = dict(status='HOME_SEARCH', candidate_count=len(candidates),
                                 preferred_face=getattr(self,'preferred_grasp_face',None),
                                 attempted_indices=[], unsearched_indices=list(range(len(candidates))))
        if not candidates: raise ValueError('NO_NONEMPTY_FULL_SEAL_CANDIDATE')
        home = None
        for index, grasp in enumerate(candidates):
            self.select_grasp(grasp)
            pre = check_transform(grasp['tcp_pose_D']).copy()
            pre[:3, 3] += face_outward(initial_target_pose,self.face)*(contract['preapproach_m'] if contract else .03)
            try:
                home, fk = self.select_home(pre)
                home = home.copy()
                break
            except ValueError as error:
                if not contract or str(error) != 'NATIVE_HOME_CANDIDATES_EXHAUSTED': raise
                self.grasp_home_rejections.append(dict(candidate_index=index, reason=str(error)))
        if home is None:
            self.grasp_search['status'] = 'HOME_EXHAUSTED'
            raise ValueError('NATIVE_HOME_CANDIDATES_EXHAUSTED')
        selected = None
        for index, grasp in enumerate(candidates):
            # This is offline MTC branch backtracking, before any replay. The
            # worker verifies that a root branch has the identical initial q,
            # complete world and no attachment; stage IDs remain unique.
            self.parent = ''; self.attachment = None; self.target_pose = initial_target_pose.copy()
            self.stages = []; self.events = []; self.extraction_evidence = None;self.extraction_attempts=[];self.approach_attempts=[]
            self.grasp_search.update(status='SEARCHING', active_index=index)
            self.grasp_search['attempted_indices'].append(index)
            self.grasp_search['unsearched_indices'] = list(range(index+1, len(candidates)))
            if getattr(self,'progress_path',None):
                with self.progress_path.open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(dict(event='GRASP_BRANCH_STARTED',candidate_index=index,
                        candidate_count=len(candidates),candidate_id=grasp.get('candidate_id'),
                        target_face=grasp.get('target_face','top'),active_ids=grasp.get('active_ids'),
                        ranking_cost=grasp.get('ranking_cost')))+'\n')
            branch_started = perf_counter()
            record_start, ik_start, call_start = len(self.records), len(self.ik_records), len(self.native_calls)
            try:
                actual_seal = self.run_grasp(grasp, home)
                selected = dict(grasp, candidate_index=index)
                self.grasp_search.update(status='SELECTED', selected_index=index)
                break
            except Exception as error:
                recoverable = (bool(contract) and isinstance(error, ValueError) and
                    (str(error).startswith('NATIVE_STAGE_CANDIDATES_EXHAUSTED:') or str(error) in {
                        'CONTACT_ENDPOINT_CONTOURS_REJECTED', 'SUPPORTED_RELEASE_REJECTED'}))
                self.grasp_failures.append(dict(candidate_index=index, candidate=grasp, reason=str(error),
                    recoverable_geometry_failure=recoverable, planning_wall_s=perf_counter()-branch_started,
                    native_record_range=[record_start,len(self.records)], native_ik_record_range=[ik_start,len(self.ik_records)],
                    native_call_range=[call_start,len(self.native_calls)],
                    partial_stages=[{k:s[k] for k in ('stage','stage_id','parent_stage_id','q','times')} for s in self.stages] if getattr(self,'call_archive',None) else list(self.stages),
                    partial_events=deepcopy(self.events), extraction_geometry=deepcopy(self.extraction_evidence),
                    extraction_attempts=deepcopy(self.extraction_attempts),approach_attempts=deepcopy(self.approach_attempts)))
                if not recoverable:
                    self.grasp_search.update(status='INTERRUPTED', reason=str(error))
                    # A timeout closes the resident worker. Do not retry a
                    # branch on that process or reinterpret missing evidence.
                    raise
        if selected is None:
            self.grasp_search['status'] = 'EXHAUSTED'
            raise ValueError('NATIVE_GRASP_CANDIDATES_EXHAUSTED')
        result = dict(schema='eco65_native_plan_v1', status=STATUS, task_id=self.task_id,
            evaluation_profile=self.evaluation_profile, replay_interpolation=INTERPOLATION,
            j1_direction_policy=J1_POLICY if self.directed_j1 else None,
            independent_review='NOT_RUN_PROFILE' if self.evaluation_profile == CHAIN else 'NOT_EVALUATED',
            identity=self.identity, snapshot_fingerprint=self.snapshot['fingerprint'], home_q=home.tolist(),
            stages=self.stages, events=self.events, fk_crosscheck=fk,
            native_records=self.records, native_ik_records=self.ik_records,
            native_calls=self.native_calls, initial_state_checks=self.home_checks,
            native_calls_archive=self.call_archive_info(),
            target_face=self.face, active_ids=self.active_ids, selection_binding=self.selection_binding(),
            independent_control_assumed=bool(contract and contract.get('cup_selection_mode') == INDEPENDENT),
            nominal_contact_geometry=selected['nominal_contact_geometry'],
            actual_contact_geometry=actual_seal, selected_grasp=selected, grasp_geometry=self.grasp_geometry,
            grasp_search=self.grasp_search, grasp_failures=self.grasp_failures,
            grasp_home_rejections=self.grasp_home_rejections,
            extraction_geometry=self.extraction_evidence,
            extraction_attempts=self.extraction_attempts,
            approach_attempts=self.approach_attempts,
            initial_state_semantics='SIMULATION_INITIAL_STATE_NOT_A_VERIFIED_HOMING_MOTION',
            native_initialization_s=self.initialization_s,
            planning_wall_s=perf_counter()-started, simulated_duration_s=sum(s['times'][-1] for s in self.stages),
            dynamics='NOT_EVALUATED', hardware_enabled=False, independent_geometry='NOT_EVALUATED')
        result['fingerprint'] = digest(result)
        validate_plan_contract(result, self.snapshot)
        return result


def supported(scene, pose):
    b = scene['box']; half = np.array(b['size_m'])/2
    if scene.get('native_task'):
        lo, hi = np.asarray(scene['native_task']['receiving']['bounds_D_m'])
    else:
        belt = next(o for o in scene['obstacles'] if o['id'] == 'conveyor_longitudinal_belt')
        center = np.array(belt['position_m']); bh = np.array(belt['size_m'])/2
        lo, hi = center-bh, center+bh
    corners = np.array([[x, y, -half[2]] for x in (-half[0], half[0]) for y in (-half[1], half[1])])
    corners = corners@pose[:3, :3].T+pose[:3, 3]
    return bool(np.max(np.abs(corners[:, 2]-hi[2])) <= .0004
                and np.all(corners[:, :2] >= lo[:2]-1e-8)
                and np.all(corners[:, :2] <= hi[:2]+1e-8))


def validate_plan_contract(plan, snapshot):
    profile = checked_profile(plan.get('evaluation_profile', STRICT))
    if snapshot['scene'].get('native_task',{}).get('coverage_context'):
        raise ValueError('CONDITIONAL_COVERAGE_IS_NOT_A_COMPLETE_TASK')
    expected = dict(plan); fingerprint = expected.pop('fingerprint')
    if digest(expected) != fingerprint or plan['status'] != STATUS or plan['hardware_enabled']:
        raise ValueError('PLAN_IDENTITY_OR_MODE_MISMATCH')
    if plan['identity'] != snapshot['native_model']['init']['identity'] or plan['snapshot_fingerprint'] != snapshot['fingerprint']:
        raise ValueError('PLAN_MODEL_TOOL_SCENE_MISMATCH')
    if not stage_chain_valid(plan['stages']): raise ValueError('PLAN_STAGE_CHAIN_INCOMPLETE')
    if [e['event'] for e in plan['events']] != ['ATTACH_PREDICTED', 'SUPPORTED_RELEASE_PREDICTED']:
        raise ValueError('PLAN_EVENT_CHAIN_INCOMPLETE')
    previous = np.asarray(plan['home_q']); parent = ''; seen = set()
    model = snapshot['native_model']; joints = model['init']['joint_names']; limits = model['simulation_limits']
    contract = validate_task_contract(snapshot['scene'])
    independent = bool(contract and contract.get('cup_selection_mode') == INDEPENDENT)
    if independent:
        ids = selected_ids(model['init']['process_geometry'],plan['active_ids'])
        binding = dict(active_ids=ids,target_id=contract['target_id'],target_face=plan['target_face'],
                       candidate_id=plan['selected_grasp']['candidate_id'],task_id=plan['task_id'],identity=plan['identity'])
        expected_candidate=digest(dict(target_id=contract['target_id'],target_face=plan['target_face'],
            active_ids=ids,tcp_pose_D=plan['selected_grasp']['tcp_pose_D']))
        if (plan['selection_binding'] != binding or plan['selected_grasp']['active_ids'] != ids or
                plan['selected_grasp']['target_face']!=plan['target_face'] or binding['candidate_id']!=expected_candidate):
            raise ValueError('PLAN_CUP_SELECTION_BINDING_CHANGED')
        for event in plan['events']:
            if event.get('selection_binding') != binding: raise ValueError('PLAN_EVENT_CUP_SELECTION_CHANGED')
    if model['init']['process_geometry'].get('task_contract') != contract:
        raise ValueError('PLAN_TASK_CONTRACT_CHANGED')
    if model['init']['process_geometry'].get('fixed_attachment_interfaces', []) != validate_fixed_interfaces(snapshot['scene']):
        raise ValueError('PLAN_FIXED_ATTACHMENT_INTERFACES_CHANGED')
    bounds = np.array([[float(model['official_limits'][j][k]) for k in ('lower', 'upper')] for j in joints])
    velocity = np.array([min(float(model['official_limits'][j]['velocity']), limits['joint_velocity_rad_s']) for j in joints])
    attach, release = plan['events']; target = snapshot['scene']['target_box_id']
    for event, phase in [(attach, 'contact'), (release, 'place')]:
        s = next(s for s in plan['stages'] if s['stage'] == phase)
        if event['target_id'] != target or event['after_stage'] != s['stage_id'] or not np.array_equal(event['q'], s['q'][-1]):
            raise ValueError('PLAN_EVENT_IDENTITY_MISMATCH')
    check_transform(attach['flange_from_target']); check_transform(release['pose'])
    if profile == CHAIN:
        if plan.get('replay_interpolation') != INTERPOLATION or plan.get('independent_review') != 'NOT_RUN_PROFILE':
            raise ValueError('CHAIN_PLAN_EVALUATION_CHANGED')
        robot = robot_for(snapshot['tool'], snapshot['scene'])
        for event, pose in [(attach, transform(snapshot['scene']['box']['pose'])), (release, np.asarray(release['pose']))]:
            if not np.allclose(robot.named_link_frames(event['q'])['link_6']@np.asarray(attach['flange_from_target']), pose, atol=1e-8, rtol=0):
                raise ValueError('PLAN_EVENT_TARGET_TRANSFORM_MISMATCH')
    for s in plan['stages']:
        if checked_profile(s.get('evaluation_profile', STRICT)) != profile:
            raise ValueError('EVALUATION_PROFILE_MISMATCH')
        if profile == CHAIN and s.get('python_linear_audit') != 'NOT_RUN_PROFILE':
            raise ValueError('CHAIN_PYTHON_AUDIT_UNEXPECTED')
        q, t = np.asarray(s['q']), np.asarray(s['times'])
        if (q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or len(t) != len(q) or
                not np.isfinite(q).all() or not np.isfinite(t).all() or t[0] != 0 or np.any(np.diff(t) <= 0)):
            raise ValueError('EMPTY_OR_INVALID_TRAJECTORY')
        if not np.any(np.abs(np.diff(q, axis=0)) > 1e-12): raise ValueError('STATIONARY_MOTION_NOT_AN_EVENT')
        if np.max(np.abs(previous-q[0])) > 1e-9 or s['parent_stage_id'] != parent:
            raise ValueError('PLAN_STAGE_DISCONTINUITY')
        if s['stage_id'] in seen: raise ValueError('PLAN_STAGE_ID_REUSED')
        seen.add(s['stage_id'])
        if np.any(q < bounds[:, 0]-1e-9) or np.any(q > bounds[:, 1]+1e-9): raise ValueError('PLAN_JOINT_LIMIT')
        d, dt = np.abs(np.diff(q, axis=0)), np.diff(t)[:, None]
        if (np.any(1.875*d/dt > velocity+1e-8) or
                np.any(10/np.sqrt(3)*d/dt**2 > limits['joint_acceleration_rad_s2']+1e-8) or
                np.any(60*d/dt**3 > limits['joint_jerk_rad_s3']+1e-8)):
            raise ValueError('PLAN_SIMULATION_LIMIT')
        raw, req = s['native_result'], s['native_request']
        validate_output_evaluation(profile, req, raw)
        if profile == CHAIN and (req.get('flange_from_task_tcp') != plan['identity']['flange_from_task_tcp'] or
                raw.get('task_tcp_identity') != plan['identity']['task_tcp_fingerprint'] or
                ('goal_pose' in req and raw.get('interpolated_link') != plan['identity']['task_tcp_link'])):
            raise ValueError('PLAN_TASK_TCP_IDENTITY_MISMATCH')
        if profile==CHAIN and 'goal_pose' in req:
            fk=endpoint_fk(robot,q[-1],req['goal_pose'])
            if fk['position_error_m']>1e-6 or fk['orientation_error_rad']>1e-6:raise ValueError('PLAN_ACTUAL_ENDPOINT_FK_FAILED')
        if plan.get('j1_direction_policy') is not None:
            direction=j1_direction(s['stage'])
            if plan['j1_direction_policy']!=J1_POLICY or req.get('j1_direction')!=direction or not j1_valid(q,direction):
                raise ValueError('PLAN_J1_DIRECTION_FAILED')
            basic=raw.get('basic_path_check',{})
            if basic.get('j1_direction')!=direction or basic.get('j1_edge_checks')!=len(q)-1:raise ValueError('PLAN_J1_EVIDENCE_MISSING')
        verify_ompl_output_interpolation(req,raw)
        if independent and (req['process_policy'].get('selection_binding') != binding or
                req['process_policy'].get('active_ids') != ids or req['process_policy'].get('target_face') != plan['target_face']):
            raise ValueError('PLAN_CUP_SELECTION_CHANGED_WITHIN_BRANCH')
        if s['stage'] in {'contact', 'extraction', 'place', 'withdrawal'} and (
                req.get('pipeline_id') != 'pilz_industrial_motion_planner' or req.get('planner_id') != 'LIN'
                or 'goal_pose' not in req): raise ValueError('PLAN_PROCESS_REQUIRES_LIN')
        if req['clearance_policy']['source_policy'] != model['init']['collision_policy'] or any(
                req['process_policy'].get(k) != v for k, v in model['init']['process_geometry'].items()):
            raise ValueError('PLAN_POLICY_OR_CONTACT_GEOMETRY_CHANGED')
        if (req['identity'] != plan['identity'] or req['task_id'] != plan['task_id'] or
                raw.get('task_id') != plan['task_id'] or req['stage_id'] != s['stage_id'] or req['stage'] != s['stage'] or
                req['planning_only'] is not True or req['planning_mode'] != 'cold_from_scratch' or
                raw.get('planning_only_status') != STATUS):
            raise ValueError('PLAN_NATIVE_IDENTITY_MISMATCH')
        loaded = s['stage'] in {'extraction', 'transit', 'place'}
        expected_attachment = attach['flange_from_target'] if loaded else None
        if s['attachment'] != expected_attachment or (req['attachment'] is not None) != loaded:
            raise ValueError('PLAN_ATTACHMENT_LIFECYCLE_INVALID')
        expected_world = {o['id']: o for o in world_objects(snapshot['scene'])}
        expected_world.update({b['id']: dict(id=b['id'], size=b['size_m'], pose=transform(b['pose']).tolist(), category='carton')
                               for b in snapshot['scene']['boxes'] if b['id'] != target})
        expected_target = release['pose'] if s['stage'] in {'withdrawal', 'residence'} else transform(snapshot['scene']['box']['pose']).tolist()
        if not loaded: expected_world[target] = dict(id=target, size=snapshot['scene']['box']['size_m'], pose=expected_target, category='carton')
        actual_world = {b['id']: b for b in req['world']}
        if len(actual_world) != len(req['world']) or actual_world != expected_world: raise ValueError('PLAN_WORLD_CHANGED')
        if req.get('scene_fingerprint') != digest(req['world']): raise ValueError('PLAN_WORLD_FINGERPRINT_CHANGED')
        if loaded and (req['attachment']['id'] != target or req['attachment']['pose'] != expected_attachment or
                       req['attachment']['touch_links'] != model['init']['compliant_tool_links']):
            raise ValueError('PLAN_ATTACHMENT_IDENTITY_INVALID')
        path = validate_native_result(raw, q[0], req.get('q_goal'), snapshot['native_model']['init']['joint_names'])
        if not np.array_equal(path, q) or not native_generation_evidence(raw, req) or raw.get('generation_source') != 'NATIVE_GENERATED':
            raise ValueError('PLAN_NATIVE_PATH_CHANGED')
        if (raw.get('status') != 'SUCCESS' or raw.get('generated_during_task') is not True or
                raw.get('mtc_generation') is not True or raw.get('stage_id') != s['stage_id'] or raw.get('parent_stage_id') != parent):
            raise ValueError('PLAN_MTC_CHAIN_MISMATCH')
        previous = q[-1]; parent = s['stage_id']
    if not np.allclose(previous, plan['home_q'], atol=1e-9, rtol=0): raise ValueError('PLAN_FINAL_HOME_CHANGED')
    if abs(sum(s['times'][-1] for s in plan['stages'])-plan['simulated_duration_s']) > 1e-8:
        raise ValueError('PLAN_DURATION_MISMATCH')
    return True


def review_plan(plan, snapshot, *, progress_path=None):
    """One independent world and exactly the same monotonic joint-edge replay."""
    from .task_world import UnloadingTaskWorld
    from .native_io import verify_snapshot
    verify_snapshot(snapshot)
    validate_plan_contract(plan, snapshot); started = perf_counter()
    if snapshot['scene']['schema'] == 'eco65_native_12carton_scene_v1':
        from .native_review_world import NativeSceneWorld
        world = NativeSceneWorld(snapshot['tool'], snapshot['scene'])
    else:
        world = UnloadingTaskWorld(snapshot['tool'], snapshot['scene'])
    world.select_target(snapshot['scene']['target_box_id'], face=plan.get('target_face','top'))
    world.receiving_region = (snapshot['scene']['native_task']['receiving']['region_id']
                              if snapshot['scene'].get('native_task') else 'conveyor_longitudinal')
    checks = 0
    # Recompute contact against the extracted outer CAD loops, not the source
    # script's fitted circles. This also governs the source contact permission.
    world.tool = deepcopy(snapshot['tool'])
    world.tool['native_contact_geometry'] = snapshot['native_model']['init']['process_geometry']
    world.tool['active_seal_ids'] = selected_ids(world.tool['native_contact_geometry'],plan.get('active_ids'))
    def progress(event, **details):
        if progress_path:
            with Path(progress_path).open('a',encoding='utf-8') as stream:
                stream.write(json.dumps(dict(event=event,checked_states=checks,
                    wall_s=perf_counter()-started,**details),allow_nan=False)+'\n')
    try:
        for stage in plan['stages']:
            progress('STAGE_STARTED',stage=stage['stage'])
            phase = INDEPENDENT_PHASE[stage['stage']]
            q, ts = np.asarray(stage['q']), np.asarray(stage['times'])
            if stage['stage'] in {'contact', 'extraction', 'place', 'withdrawal'}:
                audit = audit_linear_tcp(q, world.robot.fk, world.robot.fk(q[0]), stage['native_request']['goal_pose'],
                    position_tolerance=.0001, orientation_tolerance=.001, edge_resolution_rad=.0008)
                if not audit['passed']: raise ValueError('INDEPENDENT_TCP_LIN_REJECTED:'+str(audit))
            for i, (a, b) in enumerate(zip(q[:-1], q[1:])):
                n = max(1, int(np.ceil((ts[i+1]-ts[i])/.02)), int(np.ceil(1.875*np.abs(b-a).sum()/.0008)))
                values, _ = sample_quintic_knots(ts[i:i+2], q[i:i+2], np.linspace(ts[i], ts[i+1], n+1))
                for value in values:
                    checks += 1
                    if not world.valid(value, phase):
                        progress('FAILED',stage=stage['stage'],edge=i,failure=world.last_failure)
                        return dict(status='FAIL', stage=stage['stage'], edge=i, q=value.tolist(),
                                    failure=world.last_failure, checked_states=checks, wall_s=perf_counter()-started,
                                    distance_query=getattr(world,'distance_query_settings',None),
                                    plan_fingerprint=plan['fingerprint'], snapshot_fingerprint=snapshot['fingerprint'])
                    if checks%1000==0:progress('PROGRESS',stage=stage['stage'],edge=i)
            if stage['stage'] == 'contact':
                relative = world.attach(q[-1])
                expected = np.asarray(snapshot['tool']['T_flange_tcp'])@relative
                if not np.allclose(expected, plan['events'][0]['flange_from_target'], atol=1e-8, rtol=0):
                    raise ValueError('INDEPENDENT_ATTACHMENT_TRANSFORM_MISMATCH')
            if stage['stage'] == 'place':
                pose = world.release(q[-1])
                if not np.allclose(pose, plan['events'][1]['pose'], atol=1e-8, rtol=0):
                    raise ValueError('INDEPENDENT_RELEASE_TRANSFORM_MISMATCH')
        return dict(status='PASS', checked_states=checks, wall_s=perf_counter()-started,
                    distance_query=getattr(world,'distance_query_settings',None),
                    actual_box_box_distance_queries=getattr(world,'box_box_distance_queries',None),
                    fixed_mount_actual_mujoco_queries=getattr(world,'fixed_distance_queries',None),
                    fixed_mount_query_scope='RIGID_INTERFACE_INVARIANT_WITHIN_SAME_REVIEW_WORLD',
                    plan_fingerprint=plan['fingerprint'], snapshot_fingerprint=snapshot['fingerprint'],
                    target_id=world.box_id, release_pose=world.released_pose.tolist(),
                    other_cartons_preserved=True, continuous_sweep_proof=False, dynamics='NOT_EVALUATED')
    finally: world.close()
