"""One cold ECO65 cycle followed by the next carton's actual contact.

Every robot edge is produced by the resident MTC worker. The external-axis and
ownership transitions use native, no-trajectory receipts in the same task.
"""
from contextlib import contextmanager
from copy import deepcopy
from time import perf_counter
import numpy as np

from ..moveit2_backend import MoveItUnavailable, digest
from .model import transform, check_transform, save_json
from .native_task import NativeTask
from .native_contact import check_contours, face_outward, EDGE_ALIGNMENT
from .native_cups import grasp_candidates
from .evaluation import CHAIN, INTERPOLATION
from . import posture_preference
from .coupled_state import (MODE, J1_POLICY, J1_FREE_POLICY, business_direction, INTERVALS, sign_state,
    check_business_direction, receiving_geometry, validate_cycle)
from .transverse_process import POLICY, process_options, release_poses, corners, pose_of


class CoupledTask(NativeTask):
    def __init__(self, snapshot, command, *, connection_seconds=100., ik_seeds=8,
                 max_grasps=24, initial_mode='new_simulation', first_approach_direction='unrestricted', **kwargs):
        if first_approach_direction!='unrestricted':raise ValueError('NEW_PROCESS_REQUIRES_UNRESTRICTED_J1')
        if snapshot['scene'].get('transverse_process_policy')!=POLICY:
            raise ValueError('REBUILD_SNAPSHOT_WITH_TRANSVERSE_PROCESS_V2')
        self.j1_policy=J1_FREE_POLICY
        self.process_option=None
        self.sync_location='BEFORE_APPROACH'
        self.process_search=[]
        if initial_mode not in ('new_simulation','inherited_state'):
            raise ValueError('INITIAL_STATE_MODE_INVALID')
        if snapshot['native_model']['init']['process_geometry'].get('edge_alignment')!=EDGE_ALIGNMENT:
            raise ValueError('COUPLED_EDGE_ALIGNMENT_RULE_REQUIRED')
        if initial_mode=='inherited_state' and kwargs.get('home_candidate') is None:
            raise ValueError('INHERITED_INITIAL_Q_REQUIRED')
        perception=snapshot['scene'].get('perception_binding')
        if perception:
            from .perception import require_tcp_calibration
            require_tcp_calibration(perception['calibration'])
        if perception and (initial_mode!='inherited_state' or
                kwargs['home_candidate']['q_rad']!=perception['initial_q_rad']):
            raise ValueError('PERCEPTION_PLAN_MUST_CONTINUE_ACTUAL_CAPTURE_Q')
        self.initial_mode=initial_mode
        self.allow_initial_reselection=initial_mode=='new_simulation'
        self.initial_selection=None
        self.business_interval = 'FIRST_APPROACH'
        self.reverse_used = dict.fromkeys(INTERVALS, 0.)
        self.connection_seconds = connection_seconds
        self.connection_deadline = None
        self.reception_deadline = None
        self.phase_budgets = {}
        self.ik_seeds = ik_seeds
        self.max_grasps = max_grasps
        self.cycle_records = []
        self.completed_ids = []
        self.receiver_history = []
        self.cycle_failures = []
        self.current_q = None
        self.accepted_state = None
        self.spec = snapshot['scene']['coupled_conveyor_spec']
        self.desired_drop = float(self.spec['config']['desired_drop_m'])
        self.height_m = float(snapshot['scene']['coupled_state']['height_m'])
        self.carton_poses = {b['id']: transform(b['pose']) for b in snapshot['scene']['boxes']}
        self.started = perf_counter()
        kwargs.update(evaluation_profile=CHAIN, continuous_task_mode=MODE,
                      preferred_grasp_face='front')
        super().__init__(snapshot, command, **kwargs)
        self.original_boxes = {b['id']: deepcopy(b) for b in self.scene['boxes']}
        self.first_target_id = self.target['id']
        self.next_target_id = None

    def direction_for_stage(self, stage):
        return business_direction(self.business_interval,getattr(self,'j1_policy',J1_POLICY))

    def goal_cost(self,start,goal):
        return posture_preference.goal_cost(self.robot,start,goal,self.posture_stack_center())

    def posture_stack_center(self):
        # Use the frozen stack, never the moving attached carton's current pose.
        return np.mean([b['pose'][:3] for b in self.scene['boxes']],axis=0)

    def ik_seed_candidates(self,seed,count):
        return posture_preference.ik_seeds(self.robot,seed,count,6518+self.counter,self.posture_stack_center())

    def transit_seed_candidates(self,q):
        return posture_preference.transit_seeds(self.robot,q,self.posture_stack_center())

    def initial_candidates(self,grasp,home):
        """New-world starting poses; never an unshown initialization trajectory."""
        if self.initial_mode=='inherited_state':
            binding=self.scene.get('perception_binding')
            yield home,dict(mode=self.initial_mode,source='ACTUAL_RGBD_CAPTURE_Q' if binding else 'EXACT_SUPPLIED_PREVIOUS_TERMINAL_Q',
                camera_observation='FRESH_CALIBRATED_SIMULATED_RGBD' if binding else 'NOT_PERFORMED_FROZEN_SCENE_INPUT',
                perception_binding_fingerprint=binding['fingerprint'] if binding else None)
            return
        contact=check_transform(grasp['tcp_pose_D'])
        # A finite stand-off one carton height away from the face and toward
        # +Y gives the approach room. Native IK and the ordinary
        # state check decide whether each seed actually works.
        distance=float(self.target['size_m'][2])
        standby=contact.copy()
        standby[:3,3]+=face_outward(self.target_pose,self.face)*distance
        standby[1,3]+=distance
        with self.connection():
            values=list(self.ik(standby,home,'pregrasp'))
        values.sort(key=lambda q:self.goal_cost(home,q))
        for q in values[:3]:
            yield q,dict(mode=self.initial_mode,source='CURRENT_RUN_NATIVE_STANDOFF_IK',
                standby_tcp_D=standby.tolist(),stand_off_m=distance,
                camera_observation='NOT_PERFORMED_FROZEN_SCENE_INPUT',
                hidden_initialization_motion=False)
        yield home,dict(mode=self.initial_mode,source='PRIOR_HOME_CHECKED_CANDIDATE',
                       camera_observation='NOT_PERFORMED_FROZEN_SCENE_INPUT')

    def direction_goal_valid(self,start,end,stage):
        try:
            check_business_direction([start,end],self.business_interval,
                                     self.reverse_used[self.business_interval],self.j1_policy)
            return True
        except ValueError:
            return False

    @contextmanager
    def connection(self):
        prior = self.connection_deadline
        proposed = min(self.deadline, perf_counter()+self.connection_seconds)
        self.connection_deadline = min(prior, proposed) if prior else proposed
        try:
            yield
        finally:
            self.connection_deadline = prior

    def call(self, request):
        if (self.connection_deadline is not None and perf_counter()>=self.connection_deadline
                and perf_counter()<self.deadline):
            raise ValueError('NATIVE_CONNECTION_BUDGET_EXHAUSTED')
        request = deepcopy(request)
        if request['op']=='ik':
            # Retain the real solve-time world for diagnosis. The KDL solve
            # itself still reports ik_collision_checked=false; its returned
            # candidate receives the ordinary separate native state check.
            context=self.request(request['q_start'],getattr(self,'_ik_stage','pregrasp'),op='ik')
            request={**context,**request}
        if request['op'] != 'init':
            request.update(continuous_task_mode=MODE, business_interval=self.business_interval,
                           j1_direction=self.direction_for_stage(request.get('stage')),
                           j1_direction_policy=getattr(self,'j1_policy',J1_POLICY),
                           j1_reverse_used_rad=self.reverse_used[self.business_interval],
                           j1_reverse_limit_rad=0.)
        deadline = self.deadline
        if self.connection_deadline:
            self.deadline = min(deadline, self.connection_deadline)
        try:
            if (request['op'] not in ('init', 'plan', 'diagnostic_connection', 'eco65_transition')
                    and 'request_budget_s' not in request):
                # State checks share the task/connection cutoff too. The base
                # call keeps its additional 10 s solely for IPC reply handling.
                request['request_budget_s'] = min(self.rpc_seconds, self.deadline-perf_counter())
            try:
                return super().call(request)
            except MoveItUnavailable as error:
                # Building/serializing a request can cross the connection
                # deadline after the entry check. Do not mistake that bounded
                # candidate exhaustion for the whole task's absolute deadline.
                recoverable = (str(error)=='NATIVE_TASK_BUDGET_EXHAUSTED' or
                    (request['op']=='check_state_offline' and
                     str(error)=='NATIVE_ABSOLUTE_DEADLINE_EXPIRED'))
                if (recoverable
                        and self.connection_deadline is not None
                        and perf_counter()<deadline):
                    raise ValueError('NATIVE_CONNECTION_BUDGET_EXHAUSTED') from error
                raise
        finally:
            self.deadline = deadline

    def request_budgets(self, remaining):
        # Keep one absolute request/connection deadline. A finished solver's
        # unused time is available to mandatory output checking; a fractional
        # sub-budget must not discard a path while most of its budget remains.
        budget = min(remaining, self.rpc_seconds)
        if not np.isfinite([budget, self.stage_seconds]).all() or budget <= 10.:
            raise ValueError('NATIVE_REQUEST_BUDGET_EXHAUSTED')
        return dict(request_budget_s=budget,
                    allowed_planning_time_s=min(self.stage_seconds, budget-10.),
                    output_check_timeout_s=budget-5., return_reserve_s=5.)

    def mechanism(self, height=None):
        from .coupled_scene import coupled_conveyor_state
        return coupled_conveyor_state(self.spec, self.height_m if height is None else height)

    def mechanism_world(self, height=None):
        from .coupled_scene import native_coupled_world
        return native_coupled_world(self.scene, self.height_m if height is None else height)

    def receiver(self, height=None):
        mechanism = self.mechanism(height)
        return mechanism['receiving']

    def runtime_contract(self, height=None):
        contract = deepcopy(self.task_contract)
        contract.update(target_id=self.target['id'], target_face=self.face,
                        stack_carton_ids=[b['id'] for b in self.scene['boxes']
                                          if b['id'] not in self.completed_ids],
                        original_support_names=self.target['supported_by'],
                        receiving=self.receiver(height),
                        continuous_task_mode=MODE, completed_ids=list(self.completed_ids))
        return contract

    def request(self, q, stage, *, op='check_state_offline', endpoint=False):
        req = super().request(q, stage, op=op, endpoint=endpoint)
        world = self.mechanism_world()
        for b in self.scene['boxes']:
            if b['id'] in self.completed_ids or (self.attachment is not None and b['id'] == self.target['id']):
                continue
            world.append(dict(id=b['id'],size=b['size_m'],pose=self.carton_poses[b['id']].tolist(),category='carton'))
        contract = self.runtime_contract()
        process = req['process_policy']
        process.update(task_contract=contract,stack_carton_ids=contract['stack_carton_ids'],
                       support_names=contract['original_support_names'] if stage=='extraction'
                       else contract['receiving']['support_ids'] if stage=='place' else [])
        req.update(world=world,scene_fingerprint=digest(world),continuous_task_mode=MODE,
                   ik_posture_preference=deepcopy(posture_preference.POLICY),
                   business_interval=self.business_interval,j1_direction_policy=self.j1_policy,
                   j1_direction=self.direction_for_stage(stage),
                   j1_reverse_used_rad=self.reverse_used[self.business_interval],
                   j1_reverse_limit_rad=0.,
                   transverse_process_policy=POLICY,process_option=deepcopy(self.process_option),
                   mechanism_state=dict(height_m=self.height_m,
                                        spec_fingerprint=self.spec['fingerprint']),
                   completed_ids=list(self.completed_ids))
        req['clearance_policy']['conveyor_ids'] = [x['id'] for x in self.mechanism()['world_objects']]
        return req

    def state(self, q=None):
        q = self.current_q if q is None else np.asarray(q)
        req = self.request(q, 'transit' if self.attachment is not None else 'pregrasp')
        return sign_state(dict(schema=MODE,q=q.tolist(),height_m=self.height_m,
            world=req['world'],attachment=req['attachment'],target_id=self.target['id'],
            target_face=self.face,active_ids=list(self.active_ids),candidate_id=self.candidate_id,
            completed_ids=list(self.completed_ids),remaining_ids=[b['id'] for b in self.scene['boxes']
                                                                 if b['id'] not in self.completed_ids],
            receiver_history=deepcopy(self.receiver_history),mechanism=self.mechanism(),
            parent_stage_id=self.parent,business_interval=self.business_interval,
            scene_fingerprint=self.scene['scene_fingerprint'],
            collision_policy=self.init['collision_policy'],process_option=deepcopy(self.process_option),
            j1_direction_policy=self.j1_policy))

    def ik(self, pose, seed, stage, count=32):
        previous=getattr(self,'_ik_stage','pregrasp')
        self._ik_stage=stage
        try:
            yield from super().ik(pose, seed, stage, min(count, self.ik_seeds))
        finally:
            self._ik_stage=previous

    def move(self, stage, *, pose=None, goal=None, seed=None):
        before = self.state()
        with self.connection():
            value = super().motion(self.current_q.copy(),stage,pose=pose,goal=goal,ik_seed=seed)
        record = self.stages[-1]
        self.reverse_used[self.business_interval] = check_business_direction(
            record['q'],self.business_interval,self.reverse_used[self.business_interval],self.j1_policy)
        self.current_q = value.copy()
        after = self.state()
        self.cycle_records.append(dict(kind='robot_motion',record_id=record['stage_id'],
            stage_id=record['stage_id'],stage=stage,parent_stage_id=record['parent_stage_id'],
            state_before=before,state_after=after,q=record['q'],times=record['times'],
            points=[dict(q=q,t=t) for q,t in zip(record['q'],record['times'])],
            native_request=record['native_request'],native_result=record['native_result']))
        self.accepted_state = after
        return value

    def checkpoint(self):
        return dict(parent=self.parent,q=self.current_q.copy(),height_m=self.height_m,
            target=deepcopy(self.target),target_pose=self.target_pose.copy(),
            attachment=None if self.attachment is None else self.attachment.copy(),
            carton_poses=deepcopy(self.carton_poses),completed_ids=list(self.completed_ids),
            receiver_history=deepcopy(self.receiver_history),business_interval=self.business_interval,
            reverse_used=dict(self.reverse_used),face=self.face,active_ids=list(self.active_ids),
            candidate_id=self.candidate_id,records=list(self.cycle_records),stages=list(self.stages),
            accepted_state=self.accepted_state,next_target_id=self.next_target_id,
            process_option=deepcopy(self.process_option),sync_location=self.sync_location)

    def restore(self, cp):
        for key in ('parent','height_m','target','target_pose','attachment','carton_poses','completed_ids',
                    'receiver_history','business_interval','reverse_used','face','active_ids','candidate_id',
                    'accepted_state','next_target_id','process_option','sync_location'):
            setattr(self,key,deepcopy(cp[key]))
        # Accepted native stage rows are immutable. Backtracking changes the
        # history list, never the original receipts/geometry in its prefix.
        self.stages=list(cp['stages'])
        self.current_q=cp['q'].copy()
        self.cycle_records=list(cp['records'])

    def attached_check_stage(self):
        # Once a real free-transfer segment is accepted, a later stopped lift
        # keeps free-transfer collision rules. It must not regain the earlier
        # extraction-only carton contact permission.
        return ('transit' if self.stages and self.stages[-1]['stage'] in ('transit','place')
                else 'extraction')

    def transition(self, event_type, change, *, kind='event', samples=None, duration_s=0.):
        """Commit a state change only after the worker accepts its native receipt."""
        checkpoint = self.checkpoint()
        before = self.state()
        stage = 'contact' if event_type=='ATTACH' else 'place' if event_type=='RELEASE' else (
            self.attached_check_stage() if self.attachment is not None else 'withdrawal')
        req = self.request(self.current_q,stage,op='eco65_transition',endpoint=event_type=='ATTACH')
        self.counter+=1
        stage_id=f'{event_type.lower()}-{self.counter}'
        req.update(stage_id=stage_id,pipeline_id='external_axis' if kind=='external_axis' else 'semantic_event',
                   planner_id='STOPPED_COUPLED_AXIS' if kind=='external_axis' else 'NONE')
        try:
            change()
            after_req=self.request(self.current_q,self.attached_check_stage() if self.attachment is not None else 'withdrawal')
            after=self.state()
        finally:
            self.restore(checkpoint)
        req['transition']=dict(kind=kind,event_type=event_type,after_request=after_req,
                               sample_requests=[] if samples is None else [s['request'] for s in samples])
        raw=self.call(req)
        if raw.get('status')!='SUCCESS':
            raise ValueError('NATIVE_TRANSITION_REJECTED:'+event_type+':'+str(raw.get('failure',raw.get('status'))))
        change()
        self.parent=stage_id
        after=self.state()
        record=dict(kind=kind,record_id=stage_id,stage_id=stage_id,stage=event_type,
            event_type=event_type,parent_stage_id=checkpoint['parent'],state_before=before,
            state_after=after,native_request=req,native_result=raw,duration_s=duration_s)
        if samples is not None:
            record['samples']=[dict(height_m=s['height_m'],state=s['state']) for s in samples]
        self.cycle_records.append(record)
        self.accepted_state=after
        return record

    def lift(self, height):
        if self.attachment is not None:
            raise ValueError('RECEIVER_SYNC_AFTER_ATTACHMENT_DISABLED')
        # The native parent binds the complete generated world. Only exact
        # equality can omit its external-axis transition (reference sync fix).
        if height==self.height_m:
            return
        checkpoint=self.checkpoint()
        samples=[]
        count=max(1,int(np.ceil(abs(height-self.height_m)/.01)))
        try:
            for h in np.linspace(self.height_m,height,count+1):
                self.require_connection_budget()
                self.height_m=float(h)
                samples.append(dict(height_m=float(h),state=self.state(),
                    request=self.request(self.current_q,'pregrasp')))
        finally:
            self.restore(checkpoint)
        self.transition('COUPLED_LIFT',lambda:setattr(self,'height_m',float(height)),
            kind='external_axis',samples=samples,duration_s=abs(height-self.height_m)/.06)

    def seal(self):
        return check_contours(self.init['process_geometry'],
            self.robot.named_link_frames(self.current_q)['link_6'],self.target_pose,self.target['size_m'],
            gap=.0004,penetration=.0004,angle=self.snapshot['tool']['contact']['normal_tolerance_rad'],
            face=self.face,active_ids=self.active_ids)

    def attach(self, *, next_target=False):
        self.require_receive_height()
        seal=self.seal()
        if seal['status']!='PASS':
            raise ValueError('ACTUAL_CONTACT_SEAL_FAILED')
        local=np.linalg.inv(self.robot.named_link_frames(self.current_q)['link_6'])@self.target_pose
        def change():
            self.attachment=local.copy()
            if not next_target:
                self.business_interval='OUTBOUND'
                self.reverse_used['OUTBOUND']=0.
        record=self.transition('ATTACH',change)
        record['actual_seal']=seal

    def geometry_candidates(self):
        remaining=[b for b in self.scene['boxes'] if b['id'] not in self.completed_ids]
        options=process_options(self.original_boxes[self.target['id']],list(self.original_boxes.values()),
                                remaining,self.spec,self.receiver())
        candidates=[];rejected=list(options['rejected']);pools=[];face_cache={}
        for option in options['options']:
            if option['face'] not in face_cache:
                face_cache[option['face']]=grasp_candidates(self.init['process_geometry'],self.target_pose,self.target['size_m'],
                    self.identity['flange_from_task_tcp'],self.scene['base_position_m'],
                    target_id=self.target['id'],faces=[option['face']],
                    angle=self.snapshot['tool']['contact']['normal_tolerance_rad'],neighbor_boxes=remaining)
            geometry=face_cache[option['face']]
            geometry['candidates'].sort(key=lambda g:(-len(g['active_ids']),g['ranking_cost'],g['candidate_id']))
            first=[];rest=[];seen=set()
            for g in geometry['candidates']:
                key=(tuple(g['active_ids']),round(g.get('roll_rad',0.),6))
                g=dict(g,process_option=deepcopy(option))
                (rest if key in seen else first).append(g);seen.add(key)
            # Do not let family diversification demote a larger sealed group.
            ordered=first+rest
            ordered.sort(key=lambda g:-len(g['active_ids']))
            pools.append(ordered);rejected.extend(geometry['rejected'])
        # Template order is decided before cups; round-robin leaves permitted
        # low/side alternatives in the bounded prefix of numeric branches.
        for rank in range(max(map(len,pools),default=0)):
            candidates.extend(pool[rank] for pool in pools if rank<len(pool))
        return dict(candidates=candidates,rejected=rejected,process_options=options,
                    checked_contour_count=12,policy=POLICY)

    def select_grasp(self,grasp):
        super().select_grasp(grasp)
        self.process_option=deepcopy(grasp['process_option'])

    def require_receive_height(self):
        if not self.process_option or abs(self.height_m-self.process_option['height_m'])>1e-8:
            raise ValueError('EXTRACTION_BEFORE_REQUIRED_RECEIVE_HEIGHT')

    def approach_offsets(self):
        offsets=[(.025,0.),(.01,0.),(.005,0.)]
        # With explicitly relaxed rigid-tool/carton contacts, the remaining
        # terminal lips still require a face-aligned approach. Give the native
        # solver finite stand-offs outside the box before approaching its face.
        # Both lateral sides are retained; native raw-J1/path checks decide.
        if (self.face=='front' and getattr(self,'scene',{}).get('rigid_tool_carton_contact_mode')):
            depth,width=map(float,self.target['size_m'][:2])
            offsets=[(depth,-width),(depth,width),(depth,0.),*offsets]
        return offsets

    def approach(self, grasp):
        self.select_grasp(grasp)
        if self.sync_location=='BEFORE_APPROACH':self.lift(self.process_option['height_m'])
        contact=check_transform(grasp['tcp_pose_D'])
        prior=self.connection_deadline
        with self.connection():
            approach_deadline=self.connection_deadline
            seeds=[q for q in self.ik(contact,self.current_q,'contact')
                   if self.direction_goal_valid(self.current_q,q,'contact')]
            seeds.sort(key=lambda q:self.goal_cost(self.current_q,q))
            if not seeds:raise ValueError('NATIVE_CONTACT_IK_EXHAUSTED')
            for seed,distance,lateral in [(q,d,y) for q in seeds[:3] for d,y in self.approach_offsets()]:
                if perf_counter()>=approach_deadline:
                    raise ValueError('NATIVE_APPROACH_SEARCH_BUDGET_EXHAUSTED')
                cp=self.checkpoint()
                pre=contact.copy();pre[:3,3]+=face_outward(self.target_pose,self.face)*distance
                pre[:3,3]+=self.target_pose[:3,1]*lateral
                try:
                    self.move('pregrasp',pose=pre,seed=seed)
                    self.finish_contact(contact,prior)
                    return
                except ValueError as error:
                    self.cycle_failures.append(dict(stage='approach',target_id=self.target['id'],
                        candidate_id=self.candidate_id,offset_m=distance,lateral_m=lateral,
                        pregrasp_tcp_D=pre.tolist(),contact_ik_q=seed.tolist(),error=str(error)))
                    self.restore(cp)
                    # The straight camera-capture posture can fold a rigid
                    # tool into link_3 when all joints turn together. Try one
                    # genuine wrist-first connection per contact branch,
                    # inside this same absolute approach deadline. This is a
                    # native checked joint waypoint, never an edited IK goal
                    # or a replacement simulation initial state.
                    if (distance==.025 and self.business_interval=='FIRST_APPROACH'
                            and self.attachment is None):
                        wrist=self.current_q.copy();wrist[3:]=seed[3:]
                        if np.max(np.abs(wrist-self.current_q))<1e-5:continue
                        try:
                            checked=self.call(self.request(wrist,'pregrasp'))
                            if checked['status']!='SUCCESS':
                                raise ValueError('WRIST_PREPARATION_ENDPOINT_REJECTED')
                            self.move('pregrasp',goal=wrist)
                            self.cycle_records[-1]['motion_purpose']='WRIST_PREPARATION_BEFORE_APPROACH'
                            self.move('pregrasp',pose=pre,seed=seed)
                            self.finish_contact(contact,prior)
                            return
                        except ValueError as recovery_error:
                            self.cycle_failures.append(dict(stage='wrist_preparation',
                                target_id=self.target['id'],candidate_id=self.candidate_id,
                                waypoint_q=wrist.tolist(),error=str(recovery_error)))
                            self.restore(cp)
            raise ValueError('NATIVE_APPROACH_CONTACT_EXHAUSTED')

    def finish_contact(self,contact,parent_deadline):
        """Accepted pregrasp starts a bounded contact phase under the task clock.

        Failed contact returns to the original approach cutoff; switching an
        offset never renews that search budget or the task's absolute deadline.
        """
        approach_deadline=self.connection_deadline
        self.connection_deadline=parent_deadline
        try:
            with self.connection():
                key=f"contact-{self.counter}-{self.target['id']}"
                self.phase_budgets[key]=dict(start_elapsed_s=perf_counter()-self.started,
                    deadline_elapsed_s=self.connection_deadline-self.started)
                if self.sync_location=='PREGRASP':self.lift(self.process_option['height_m'])
                self.move('contact',pose=contact)
                if self.sync_location=='CONTACT':self.lift(self.process_option['height_m'])
                if self.seal()['status']!='PASS':raise ValueError('ACTUAL_CONTACT_SEAL_FAILED')
        finally:
            self.connection_deadline=approach_deadline

    def require_connection_budget(self):
        limit=min(self.deadline,self.connection_deadline or self.deadline)
        if perf_counter()>=limit:raise ValueError('NATIVE_CONNECTION_BUDGET_EXHAUSTED')

    def loaded_lin(self,goal,direction,phase):
        current=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
        delta=goal[:3,3]-current[:3,3];axis=np.asarray(direction,float)
        if (np.max(np.abs(goal[:3,:3]-current[:3,:3]))>1e-6 or
                np.linalg.norm(delta-axis*np.dot(delta,axis))>1e-6 or np.dot(delta,axis)<-1e-8):
            raise ValueError('TEMPLATE_CARTESIAN_DIRECTION_OR_ORIENTATION_CHANGED')
        if np.linalg.norm(delta)<1e-7:return  # No fabricated zero-motion edge.
        tcp=goal@np.linalg.inv(self.attachment)@np.asarray(self.identity['flange_from_task_tcp'])
        self.move(phase,pose=tcp)
        self.cycle_records[-1]['process_leg']=dict(direction_world=axis.tolist(),
            template=self.process_option['template'],carton_start_pose=current.tolist(),
            carton_goal_pose=goal.tolist(),orientation_held=True)

    def extract_and_release(self):
        self.require_receive_height()
        attached=self.checkpoint()
        original=self.original_boxes[self.target['id']]
        remaining=[b for b in self.scene['boxes'] if b['id'] not in self.completed_ids]
        goals=release_poses(original,self.process_option,remaining)
        search=dict(target_id=self.target['id'],option_id=self.process_option['option_id'],
            candidate_id=self.candidate_id,resolution_m=.002,attempted_region_points=0,
            status='SEARCHING',unsearched_region='REMAINING_INTERVAL_SUBDIVISIONS')
        self.process_search.append(search)
        for box_goal in goals:
            self.restore(attached);self.require_connection_budget()
            search['attempted_region_points']+=1
            try:
                with self.connection():
                    current=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
                    first=self.process_option['segment_directions_world'][0]
                    if first==[0.,0.,1.]:
                        lifted=current.copy();lifted[2,3]=box_goal[2,3]
                        self.loaded_lin(lifted,first,'extraction')
                    elif first[1]!=0:
                        turn=current.copy();turn[1,3]=box_goal[1,3]
                        self.loaded_lin(turn,first,'extraction')
                    # Split at actual stack clearance so the neighbor-contact
                    # assumption ends before free transfer. Both legs are LIN.
                    current=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
                    others=[b for b in remaining if b['id']!=self.target['id']]
                    half_x=float((np.abs(current[:3,:3])@np.asarray(self.target['size_m'])/2)[0])
                    front=min(corners(b)[:,0].min() for b in others or [original])
                    clear_x=front-half_x-.0034
                    if first!=[0.,0.,1.] and current[0,3]>clear_x+1e-7:
                        clear=current.copy();clear[0,3]=max(box_goal[0,3],clear_x)
                        self.loaded_lin(clear,[-1.,0.,0.],'extraction')
                    # 'place' keeps a native LIN and restores ordinary payload
                    # neighbor checks; receiver-only support rules are retained.
                    self.loaded_lin(box_goal,[-1.,0.,0.],'place')
                    self.save_checkpoint('FIRST_CARTON_EXTRACTED')
                    actual=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
                    option=self.process_option
                    region=receiving_geometry(actual,self.target['size_m'],self.receiver()['bounds_D_m'],
                        desired_drop=option['expected_drop_m'],margin=self.receiver()['margin_m'])
                    if (not region['passed'] or
                            np.max(np.abs(actual[:3,:3]-pose_of(original)[:3,:3]))>1e-6 or
                            abs(region['actual_box_bottom_m']-option['release_bottom_world_m'])>1e-6):
                        raise ValueError('ACTUAL_TEMPLATE_RELEASE_NOT_REACHED:'+str(region))
                    self.release_and_drain(actual,region)
                    self.adopted_process=dict(target_id=self.target['id'],face=self.face,
                        candidate_id=self.candidate_id,active_ids=list(self.active_ids),
                        template=option['template'],frozen=option['frozen'],receiver_height_m=self.height_m,
                        coordination_order=self.sync_location,release_pose=actual.tolist(),
                        actual_drop_m=region['actual_gap_m'],full_suffix_accepted=True)
                    search['status']='COMPLETE_SUFFIX_ACCEPTED';return
            except ValueError as error:
                self.cycle_failures.append(dict(stage='template_suffix',target_id=self.target['id'],
                    candidate_id=self.candidate_id,process_option=deepcopy(self.process_option),
                    box_goal=box_goal.tolist(),error=str(error)))
        search.update(status='FINITE_REGION_EXHAUSTED',unsearched_region=None)
        self.restore(attached)
        raise ValueError('NATIVE_TEMPLATE_SUFFIX_EXHAUSTED')

    def release_and_drain(self, actual, region):
        target=self.target['id']
        def release():
            self.target_pose=actual.copy();self.carton_poses[target]=actual.copy()
            self.attachment=None;self.business_interval='RETURN'
            self.reverse_used['RETURN']=0.
        self.transition('RELEASE',release)
        receive=actual.copy();receive[2,3]-=region['actual_gap_m']
        cp=self.checkpoint();samples=[]
        count=max(1,int(np.ceil(region['actual_gap_m']/.01)))
        for f in np.linspace(0.,1.,count+1):
            pose=actual.copy();pose[2,3]-=f*region['actual_gap_m']
            self.target_pose=pose;self.carton_poses[target]=pose
            samples.append(dict(height_m=self.height_m,state=self.state(),
                                request=self.request(self.current_q,'withdrawal')))
        self.restore(cp)
        def receive_box():
            self.target_pose=receive.copy();self.carton_poses[target]=receive.copy()
            self.receiver_history.append(dict(target_id=target,release_pose=actual.tolist(),
                receiving_pose=receive.tolist(),actual_drop_m=region['actual_gap_m'],
                assumption='PRESCRIBED_ZERO_INITIAL_SPEED_GRAVITY_TIMING_NO_DYNAMICS'))
        record=self.transition('NOMINAL_RECEIVE',receive_box,samples=samples,
                               duration_s=float(np.sqrt(2*region['actual_gap_m']/9.81)))
        record['nominal_drop']=dict(actual_drop_m=region['actual_gap_m'],
            samples=[dict(t=float(np.sqrt(f))*record['duration_s'],pose=s['state']['world'][
                next(i for i,x in enumerate(s['state']['world']) if x['id']==target)]['pose'])
                for f,s in zip(np.linspace(0.,1.,len(samples)),samples)])
        # The box remains in world while the real tool withdraws.
        received=self.checkpoint()
        def drain():
            self.completed_ids.append(target)
            self.receiver_history[-1]['drain']='IDEAL_RECEIVER_DRAIN'
        # Project every actual tool collision solid onto the selected face
        # normal. The native drain receipt independently requires 10 mm gap.
        import trimesh
        from .model import ROOT
        direction=face_outward(self.target_pose,self.face)
        flange=self.robot.named_link_frames(self.current_q)['link_6']
        cad=flange@np.asarray(self.snapshot['tool']['T_flange_tool_cad'])
        points=[]
        for row in self.snapshot['tool']['collisions']:
            mesh=trimesh.load(ROOT/row['mesh'],force='mesh',process=False)
            points.append(np.asarray(mesh.vertices)@cad[:3,:3].T+cad[:3,3])
        if not points:raise ValueError('TOOL_RETREAT_SOLID_GEOMETRY_MISSING')
        tool_min=min(float((v@direction).min()) for v in points)
        box_max=float((corners(self.target,receive)@direction).max())
        distance=max(.002,.010+box_max-tool_min+.002)
        offsets=[direction*distance,direction*(distance+.01)]
        for offset in offsets:
            self.restore(received)
            retreat=self.robot.fk(self.current_q).copy()
            retreat[:3,3]+=offset
            try:
                self.move('withdrawal',pose=retreat)
                self.transition('IDEAL_RECEIVER_DRAIN',drain)
                break
            except ValueError as error:
                self.cycle_failures.append(dict(stage='withdrawal',target_id=target,
                    offset_m=offset.tolist(),error=str(error)))
        else:
            self.restore(received)
            raise ValueError('NATIVE_WITHDRAWAL_CANDIDATES_EXHAUSTED')

    def select_next(self, target, grasp):
        def change():
            self.target=deepcopy(target);self.target_pose=self.carton_poses[target['id']].copy()
            self.select_grasp(grasp)
        self.transition('NEXT_TARGET',change)
        self.next_target_id=target['id']

    def run(self):
        geometry=self.geometry_candidates()
        if self.progress_path:
            save_json(self.progress_path.with_name('first_grasp_candidates.json'),geometry)
        if not geometry['candidates']:
            self.cycle_failures.extend(geometry['rejected'])
            raise ValueError('NO_LEGAL_TRANSVERSE_PROCESS_TEMPLATE_OR_CUP_GROUP')
        self.select_grasp(geometry['candidates'][0])
        home,_=self.select_home(np.asarray(geometry['candidates'][0]['tcp_pose_D']))
        self.current_q=home.copy()
        initial=self.checkpoint()
        task_root=self.task_id
        trials=[(g,location) for g in geometry['candidates'][:self.max_grasps]
                for location in ('BEFORE_APPROACH','PREGRASP','CONTACT')]
        self.grasp_search=dict(candidate_count=len(geometry['candidates']),
            bounded_grasp_count=min(self.max_grasps,len(geometry['candidates'])),
            attempted_indices=[],unsearched_indices=list(range(len(trials))),
            excluded_by_resource_bound=max(0,len(geometry['candidates'])-self.max_grasps))
        for index,(grasp,sync) in enumerate(trials):
            if perf_counter()>=self.deadline:
                raise ValueError('FIRST_CARTON_ABSOLUTE_BUDGET_EXHAUSTED')
            self.grasp_search['attempted_indices'].append(index)
            self.grasp_search['unsearched_indices'].remove(index)
            self.restore(initial)
            self.select_grasp(grasp)
            try: starts=list(self.initial_candidates(grasp,home))
            except ValueError as error:
                self.cycle_failures.append(dict(stage='initial_selection',error=str(error)))
                continue
            for start_index,(q,selection) in enumerate(starts):
                self.restore(initial)
                try:
                    # Inherited capture q/world are identical across grasp
                    # branches. Reuse that native root (the worker compares
                    # both exactly), rather than consuming a session per
                    # grasp and hitting its eight-session capacity. Actual
                    # alternative initial states still require separate roots.
                    self.task_id=(task_root if self.initial_mode=='inherited_state'
                                  else f'{task_root}-initial-{index}-{start_index}')
                    self.select_grasp(grasp)
                    self.sync_location=sync
                    self.current_q=q.copy()
                    self.initial_selection=dict(selection,q_rad=q.tolist(),native_task_id=self.task_id)
                    self.initial_state=self.state()
                    self.approach(grasp)
                    self.attach()
                    self.save_checkpoint('FIRST_CARTON_ATTACHED')
                    self.extract_and_release()
                    self.save_checkpoint('FIRST_CARTON_COMPLETE')
                    return self.return_next()
                except ValueError as error:
                    if self.first_target_id in self.completed_ids:raise
                    self.cycle_failures.append(dict(stage='first_cycle',candidate_index=index,
                        initial_selection=selection,candidate_id=grasp['candidate_id'],error=str(error)))
        raise ValueError('FIRST_CARTON_OR_CONTINUATION_BOUNDED_CANDIDATES_EXHAUSTED')

    def return_next(self, *, complete_carton=False):
        """Try finite next faces/heights, then let the caller revisit grasp branches."""
        drained=self.checkpoint()
        boxes=[b for b in self.scene['boxes'] if b['id'] not in self.completed_ids
               and not any(x['id'] not in self.completed_ids and
                   abs(x['pose'][0]-b['pose'][0])<1e-8 and abs(x['pose'][1]-b['pose'][1])<1e-8
                   and x['pose'][2]>b['pose'][2]+1e-8 for x in self.scene['boxes'])]
        boxes.sort(key=lambda b:(-b['pose'][2],abs(b['pose'][1]),b['pose'][1]))
        pools=[]
        current_tcp=self.robot.fk(self.current_q)
        for target in boxes:
            self.restore(drained)
            self.target=deepcopy(target);self.target_pose=self.carton_poses[target['id']].copy()
            candidates=self.geometry_candidates()['candidates']
            # Keep a front attempt, then prefer the actual outgoing tool
            # orientation/cup family. The old first-eight slice could omit that
            # family and spend the entire return budget on one carton.
            front=next((g for g in candidates if g['target_face']=='front'),None)
            candidates.sort(key=lambda g:(
                -len(g['active_ids']),
                g['target_face']!=drained['face'],
                float(np.linalg.norm(np.asarray(g['tcp_pose_D'])[:3,:3]-current_tcp[:3,:3])),
                len(set(g['active_ids'])^set(drained['active_ids'])),
                float(np.linalg.norm(np.asarray(g['tcp_pose_D'])[:3,3]-current_tcp[:3,3]))))
            if front is not None:
                # High and low process options may share the same contact pose.
                # Keep both; a failed suffix is not evidence against the other.
                key=lambda g:(g['candidate_id'],g['process_option']['option_id'])
                candidates=[front]+[g for g in candidates if key(g)!=key(front)]
            pools.append((target,candidates[:min(self.max_grasps,8)]))
            self.restore(drained)
        locations=('BEFORE_APPROACH','PREGRASP','CONTACT')
        # Round-robin targets before lowering or revisiting a grasp family;
        # every candidate still uses this run's same absolute task deadline.
        for rank in range(max((len(g) for _,g in pools),default=0)):
            for sync,target,grasp in [(s,t,g[rank]) for t,g in pools if rank<len(g) for s in locations]:
                if perf_counter()>=self.deadline:
                    self.restore(drained)
                    raise ValueError('NEXT_CARTON_ABSOLUTE_BUDGET_EXHAUSTED')
                self.restore(drained)
                try:
                    self.select_next(target,grasp)
                    self.sync_location=sync
                    self.approach(grasp)
                    self.attach(next_target=not complete_carton)
                    if complete_carton:
                        self.extract_and_release()
                        return
                    result=self.result(complete=True)
                    result['validation']=validate_cycle(result,self.snapshot)
                    result.pop('fingerprint');result['fingerprint']=digest(result)
                    return result
                except ValueError as error:
                    self.cycle_failures.append(dict(stage='next_contact',target_id=target['id'],
                        candidate_id=grasp['candidate_id'],error=str(error)))
        self.restore(drained)
        raise ValueError('NEXT_CARTON_CONTACT_NOT_REACHED')

    def result(self, *, complete=False):
        state=self.state() if self.current_q is not None else None
        result=dict(schema=MODE,status='PLANNING_ONLY_NOT_EXECUTABLE',planning_only=True,
            complete=complete,physical_execution='NOT_RUN',execution_ready=False,
            identity=self.identity,snapshot_fingerprint=self.snapshot['fingerprint'],task_id=self.task_id,
            evaluation_profile=CHAIN,replay_interpolation=INTERPOLATION,j1_direction_policy=self.j1_policy,
            first_target_id=self.first_target_id,next_target_id=self.next_target_id,
            initial_state=getattr(self,'initial_state',state),terminal_state=state,records=self.cycle_records,
            planning_wall_s=perf_counter()-self.started,
            robot_motion_s=sum(r['times'][-1] for r in self.cycle_records if r['kind']=='robot_motion'),
            mechanism_display_s=sum(r.get('duration_s',0.) for r in self.cycle_records if r['kind']=='external_axis'),
            event_display_s=sum(r.get('duration_s',0.) for r in self.cycle_records if r['kind']=='event'),
            failures=self.cycle_failures,native_calls_archive=self.call_archive_info(),
            j1_reverse_used_rad=self.reverse_used)
        result['transverse_process_policy']=deepcopy(POLICY)
        result['process_search']=deepcopy(self.process_search)
        result['grasp_search']=deepcopy(self.grasp_search)
        result['adopted_process']=getattr(self,'adopted_process',None)
        result['phase_budgets']=deepcopy(self.phase_budgets)
        result['initial_selection']=deepcopy(self.initial_selection)
        result['initial_state_checks']=self.home_checks
        result['perception_binding_fingerprint']=self.scene.get('perception_binding',{}).get('fingerprint')
        result['ik_posture_preference']=deepcopy(posture_preference.POLICY)
        result['fingerprint']=digest(result)
        return result

    def save_checkpoint(self,label):
        if self.progress_path:
            result=self.result()
            save_json(self.progress_path.with_name(label.lower()+'.json'),result)
