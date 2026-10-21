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
from .coupled_state import (MODE, J1_POLICY, J1_REVERSE_LIMIT, INTERVALS, sign_state,
    check_business_direction, receiving_geometry, frozen_receive_heights, validate_cycle)


class CoupledTask(NativeTask):
    def __init__(self, snapshot, command, *, connection_seconds=100., ik_seeds=8,
                 max_grasps=24, initial_mode='new_simulation', **kwargs):
        if initial_mode not in ('new_simulation','inherited_state'):
            raise ValueError('INITIAL_STATE_MODE_INVALID')
        if snapshot['native_model']['init']['process_geometry'].get('edge_alignment')!=EDGE_ALIGNMENT:
            raise ValueError('COUPLED_EDGE_ALIGNMENT_RULE_REQUIRED')
        if initial_mode=='inherited_state' and kwargs.get('home_candidate') is None:
            raise ValueError('INHERITED_INITIAL_Q_REQUIRED')
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
        self.height_m = float(self.spec['low_height_m'])
        self.carton_poses = {b['id']: transform(b['pose']) for b in snapshot['scene']['boxes']}
        self.started = perf_counter()
        kwargs.update(evaluation_profile=CHAIN, continuous_task_mode=MODE,
                      preferred_grasp_face='front')
        super().__init__(snapshot, command, **kwargs)
        self.original_boxes = {b['id']: deepcopy(b) for b in self.scene['boxes']}
        self.first_target_id = self.target['id']
        self.next_target_id = None

    def direction_for_stage(self, stage):
        return INTERVALS[self.business_interval]

    def goal_cost(self,start,goal):
        return (abs(float(goal[0]-start[0])),float(np.abs(goal[1:]-start[1:]).sum()))

    def initial_candidates(self,grasp,home):
        """New-world starting poses; never an unshown initialization trajectory."""
        if self.initial_mode=='inherited_state':
            yield home,dict(mode=self.initial_mode,source='EXACT_SUPPLIED_PREVIOUS_TERMINAL_Q')
            return
        contact=check_transform(grasp['tcp_pose_D'])
        # A finite stand-off one carton height away from the face and toward
        # +Y gives the clockwise approach room. Native IK and the ordinary
        # state check decide whether each seed actually works.
        distance=float(self.target['size_m'][2])
        standby=contact.copy()
        standby[:3,3]+=face_outward(self.target_pose,self.face)*distance
        standby[1,3]+=distance
        with self.connection():
            values=list(self.ik(standby,home,'pregrasp'))
        values.sort(key=lambda q:(abs(q[0]-home[0]),float(np.abs(q[1:]-home[1:]).sum())))
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
                                     self.reverse_used[self.business_interval])
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
                           j1_direction=INTERVALS[self.business_interval],
                           j1_direction_policy=J1_POLICY,
                           j1_reverse_used_rad=self.reverse_used[self.business_interval],
                           j1_reverse_limit_rad=J1_REVERSE_LIMIT)
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
        from .coupled_scene import with_coupled_state
        from .native_scene import world_objects
        current = with_coupled_state(self.scene, self.height_m if height is None else height)
        return world_objects(current)

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
                   business_interval=self.business_interval,j1_direction_policy=J1_POLICY,
                   j1_direction=INTERVALS[self.business_interval],
                   j1_reverse_used_rad=self.reverse_used[self.business_interval],
                   j1_reverse_limit_rad=J1_REVERSE_LIMIT,
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
            collision_policy=self.init['collision_policy']))

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
            record['q'],self.business_interval,self.reverse_used[self.business_interval])
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
            accepted_state=self.accepted_state,next_target_id=self.next_target_id)

    def restore(self, cp):
        for key in ('parent','height_m','target','target_pose','attachment','carton_poses','completed_ids',
                    'receiver_history','business_interval','reverse_used','face','active_ids','candidate_id',
                    'stages','accepted_state','next_target_id'):
            setattr(self,key,deepcopy(cp[key]))
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
        if abs(height-self.height_m)<1e-10:
            return
        checkpoint=self.checkpoint()
        samples=[]
        count=max(1,int(np.ceil(abs(height-self.height_m)/.01)))
        for h in np.linspace(self.height_m,height,count+1):
            self.height_m=float(h)
            samples.append(dict(height_m=float(h),state=self.state(),
                request=self.request(self.current_q,self.attached_check_stage() if self.attachment is not None else 'pregrasp')))
        self.restore(checkpoint)
        self.transition('COUPLED_LIFT',lambda:setattr(self,'height_m',float(height)),
            kind='external_axis',samples=samples,duration_s=abs(height-self.height_m)/.06)

    def seal(self):
        return check_contours(self.init['process_geometry'],
            self.robot.named_link_frames(self.current_q)['link_6'],self.target_pose,self.target['size_m'],
            gap=.0004,penetration=.0004,angle=self.snapshot['tool']['contact']['normal_tolerance_rad'],
            face=self.face,active_ids=self.active_ids)

    def attach(self, *, next_target=False):
        seal=self.seal()
        if seal['status']!='PASS':
            raise ValueError('ACTUAL_CONTACT_SEAL_FAILED')
        local=np.linalg.inv(self.robot.named_link_frames(self.current_q)['link_6'])@self.target_pose
        def change():
            self.attachment=local.copy()
            if not next_target:self.business_interval='OUTBOUND'
        record=self.transition('ATTACH',change)
        record['actual_seal']=seal

    def geometry_candidates(self):
        geometry=grasp_candidates(self.init['process_geometry'],self.target_pose,self.target['size_m'],
            self.identity['flange_from_task_tcp'],self.scene['base_position_m'],
            target_id=self.target['id'],faces=['front','top'],
            angle=self.snapshot['tool']['contact']['normal_tolerance_rad'],
            neighbor_boxes=[b for b in self.scene['boxes'] if b['id'] not in self.completed_ids])
        candidates=geometry['candidates']
        candidates.sort(key=lambda x:x['target_face']!='front')
        # First sample each distinct cup/roll family, then retain the rest. No
        # candidate is declared invalid merely because a connection timed out.
        first=[];rest=[];seen=set()
        for g in candidates:
            key=(g['target_face'],tuple(g['active_ids']),round(g.get('roll_rad',0.),6))
            (rest if key in seen else first).append(g)
            seen.add(key)
        candidates=first+rest
        front=[g for g in candidates if g['target_face']=='front']
        top=[g for g in candidates if g['target_face']=='top']
        candidates=front[:4]+top[:4]+front[4:]+top[4:]
        return dict(geometry,candidates=candidates)

    def approach(self, grasp):
        self.select_grasp(grasp)
        contact=check_transform(grasp['tcp_pose_D'])
        with self.connection():
            seeds=[q for q in self.ik(contact,self.current_q,'contact')
                   if self.direction_goal_valid(self.current_q,q,'contact')]
            seeds.sort(key=lambda q:self.goal_cost(self.current_q,q))
            if not seeds:raise ValueError('NATIVE_CONTACT_IK_EXHAUSTED')
            for seed,distance in [(q,d) for q in seeds[:3] for d in (.025,.01,.005)]:
                cp=self.checkpoint()
                pre=contact.copy();pre[:3,3]+=face_outward(self.target_pose,self.face)*distance
                try:
                    self.move('pregrasp',pose=pre,seed=seed)
                    self.move('contact',pose=contact)
                    if self.seal()['status']!='PASS':raise ValueError('ACTUAL_CONTACT_SEAL_FAILED')
                    return
                except ValueError as error:
                    self.cycle_failures.append(dict(stage='approach',target_id=self.target['id'],
                        candidate_id=self.candidate_id,offset_m=distance,contact_ik_q=seed.tolist(),error=str(error)))
                    self.restore(cp)
            raise ValueError('NATIVE_APPROACH_CONTACT_EXHAUSTED')

    def release_candidates(self, height, yaws=(0.,)):
        receiver=self.receiver(height)
        lo,hi=np.asarray(receiver['bounds_D_m'])
        current=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
        half=np.asarray(self.target['size_m'])/2
        for yaw in yaws:
            c,s=np.cos(yaw),np.sin(yaw)
            rotation=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
            extent=np.abs(rotation)@half
            margin=receiver['margin_m']
            left,right=lo[:2]+extent[:2]+margin,hi[:2]-extent[:2]-margin
            if np.any(left>right):continue
            nearest=np.clip(current[:2,3],left,right)
            for xy in (nearest,(left+right)/2):
                pose=np.eye(4);pose[:3,:3]=rotation
                pose[:3,3]=[*xy,height+self.desired_drop+half[2]]
                yield pose

    def clear_stack_pose(self, lateral_m=0., above=False, recovery=None):
        """One geometry-derived withdrawal with room for an upright yaw turn."""
        box=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
        remaining=[b for b in self.scene['boxes'] if b['id']!=self.target['id']
                   and b['id'] not in self.completed_ids]
        front=min(b['pose'][0]-b['size_m'][0]/2 for b in remaining)
        radius=float(np.linalg.norm(np.asarray(self.target['size_m'])[:2]/2))
        top=max(b['pose'][2]+b['size_m'][2]/2 for b in remaining)
        clear_z=max(box[2,3],top+self.target['size_m'][2]/2+.005)
        if recovery=='diagonal_above':
            box[0,3]-=self.target['size_m'][0]/2
            box[1,3]-=self.target['size_m'][1]/2
            box[2,3]=clear_z
        elif recovery=='clear_half_rise':
            box[0,3]=min(box[0,3],front-radius-.005)
            box[1,3]-=self.target['size_m'][1]
            box[2,3]=(box[2,3]+clear_z)/2
        elif recovery is not None:
            raise ValueError('UNKNOWN_EXTRACTION_GEOMETRY')
        elif above:
            # Clear every remaining carton vertically before an upright yaw
            # turn. The endpoint is frozen stack geometry, not a guessed lift.
            box[2,3]=clear_z
        else:
            box[0,3]=min(box[0,3],front-radius-.005)
            # A real diagonal LIN lifts off the original support before its
            # horizontal overlap ends. The release goal still uses the
            # actual receiver height and drop.
            box[2,3]+=.006
            box[1,3]+=lateral_m
        return box@np.linalg.inv(self.attachment)@np.asarray(self.identity['flange_from_task_tcp'])

    def finish_release(self,height,order,route):
        # Reception is a distinct stopped-axis / release / retreat phase.
        # Its one absolute deadline is shared by every reception candidate;
        # revisiting a candidate never resets it. The global deadline remains.
        previous=self.connection_deadline
        if self.reception_deadline is None:
            start=perf_counter()
            # One shared absolute phase/task cutoff. Single connections and
            # native calls retain their finite budgets; later grasp branches
            # cannot restart the task clock or a fresh reception allowance.
            self.reception_deadline=self.deadline
            self.phase_budgets['reception_release_withdrawal']=dict(
                start_elapsed_s=start-self.started,
                deadline_elapsed_s=self.reception_deadline-self.started,
                maximum_s=self.reception_deadline-start)
        self.connection_deadline=self.reception_deadline
        try:
            if order=='EXTRACT_TRANSFER_THEN_RAISE':
                self.lift(height['height_m'])
            actual=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
            region=receiving_geometry(actual,self.target['size_m'],
                self.receiver()['bounds_D_m'],desired_drop=self.desired_drop,
                margin=self.receiver()['margin_m'])
            if not region['passed']:
                raise ValueError('ACTUAL_RELEASE_REGION_NOT_REACHED:'+str(region))
            self.release_and_drain(actual,region)
            self.adopted_process=dict(target_id=self.target['id'],face=self.face,
                candidate_id=self.candidate_id,active_ids=list(self.active_ids),
                receiver_height_m=height['height_m'],coordination_order=order,
                loaded_route=route,release_pose=actual.tolist(),actual_drop_m=region['actual_gap_m'])
        finally:
            self.connection_deadline=previous

    def extract_and_release(self):
        with self.connection():
            return self._extract_and_release()

    def _extract_and_release(self):
        attached=self.checkpoint()
        heights=frozen_receive_heights(self.original_boxes[self.target['id']],
            self.spec['low_height_m'],self.spec['high_height_m'],self.desired_drop)
        # Start with the least external-axis travel from the actual accepted
        # height. Both frozen candidates remain available within this same
        # connection deadline; a high belt must not consume the whole budget
        # before the reachable lower receiver is even attempted.
        heights.sort(key=lambda h: abs(h['height_m']-self.height_m))
        # Keep the receiver low during arm transit when the high longitudinal
        # belt obstructs link_2. The final raise is a checked fixed-q operation,
        # with the same attachment, before release.
        orders=('EXTRACT_TRANSFER_THEN_RAISE','RAISE_THEN_EXTRACT','EXTRACT_THEN_RAISE')
        for height,order in [(h,o) for h in heights for o in orders]:
            self.restore(attached)
            try:
                # This order is a real candidate, not a mandatory low/reset cycle.
                if order=='RAISE_THEN_EXTRACT':
                    self.lift(height['height_m'])
                lifted=self.checkpoint()
                for box_goal in self.release_candidates(height['height_m']):
                    self.restore(lifted)
                    try:
                        tcp=box_goal@np.linalg.inv(self.attachment)@np.asarray(self.identity['flange_from_task_tcp'])
                        # Leave the stack horizontally before lowering a box.
                        # A stopped low-belt withdrawal also provides a bounded
                        # alternative when lifting at contact collides with the tool.
                        current_box=self.robot.named_link_frames(self.current_q)['link_6']@self.attachment
                        intermediate=box_goal.copy()
                        intermediate[2,3]=max(current_box[2,3],box_goal[2,3])
                        extract_tcp=intermediate@np.linalg.inv(self.attachment)@np.asarray(self.identity['flange_from_task_tcp'])
                        if self.face=='front':
                            # Fixed-orientation native LIN out of the original support.
                            self.move('extraction',pose=extract_tcp)
                        else:
                            lift=self.robot.fk(self.current_q).copy()
                            lift[0,3]-=.15+.02;lift[2,3]+=.015
                            self.move('extraction',pose=lift)
                            self.move('transit',pose=extract_tcp)
                        if order=='EXTRACT_THEN_RAISE':
                            self.lift(height['height_m'])
                        if np.max(np.abs(self.robot.fk(self.current_q)-tcp))>1e-6:
                            self.move('place',pose=tcp)
                        self.finish_release(height,order,'DIRECT_WITHDRAWAL_TO_RECEIVER')
                        return
                    except ValueError as error:
                        self.cycle_failures.append(dict(stage='loaded_transfer',target_id=self.target['id'],
                            candidate_id=self.candidate_id,receiving=height,coordination_order=order,error=str(error)))
                # The front-grasp TCP can be unreachable directly above the
                # near belt because the flange lies behind the carton. First
                # leave the stack, then use actual native loaded transfer with
                # a finite set of upright carton yaw orientations.
                # A straight front withdrawal can cross a wrist/shoulder
                # singularity. Prefer diagonal clearance derived from this
                # box and the remaining stack, then retain the finite original
                # alternatives. Every candidate still needs a real native LIN.
                routes=((0.,False,'diagonal_above'),(0.,False,'clear_half_rise'),
                        (0.,True,None),(-float(self.target['size_m'][1]),False,None),(0.,False,None))
                for lateral,above,recovery in routes:
                    self.restore(lifted)
                    try:
                        self.move('extraction',pose=self.clear_stack_pose(lateral,above,recovery))
                    except ValueError as error:
                        self.cycle_failures.append(dict(stage='lateral_extraction',
                            candidate_id=self.candidate_id,lateral_m=lateral,above_stack=above,
                            geometry_recovery=recovery,error=str(error)))
                        continue
                    if order=='EXTRACT_THEN_RAISE':
                        self.lift(height['height_m'])
                    self.save_checkpoint('FIRST_CARTON_EXTRACTED')
                    clear=self.checkpoint()
                    for box_goal in self.release_candidates(height['height_m'],
                                                             (-np.pi/2,np.pi/2,np.pi,0.)):
                        self.restore(clear)
                        try:
                            tcp=box_goal@np.linalg.inv(self.attachment)@np.asarray(self.identity['flange_from_task_tcp'])
                            self.move('transit',pose=tcp)
                            self.finish_release(height,order,'CLEAR_STACK_THEN_NATIVE_LOADED_TRANSFER')
                            self.adopted_process['extraction_lateral_m']=lateral
                            self.adopted_process['extraction_above_stack']=above
                            self.adopted_process['extraction_geometry_recovery']=recovery
                            return
                        except ValueError as error:
                            self.cycle_failures.append(dict(stage='loaded_transfer_after_clear',
                                candidate_id=self.candidate_id,receiving=height,coordination_order=order,
                                box_goal=box_goal.tolist(),error=str(error)))
                raise ValueError('LOADED_RELEASE_CANDIDATES_EXHAUSTED')
            except ValueError as error:
                self.cycle_failures.append(dict(stage='lift_or_transfer',target_id=self.target['id'],
                    candidate_id=self.candidate_id,receiving=height,coordination_order=order,error=str(error)))
        self.restore(attached)
        raise ValueError('NATIVE_LOADED_CYCLE_EXHAUSTED')

    def release_and_drain(self, actual, region):
        target=self.target['id']
        def release():
            self.target_pose=actual.copy();self.carton_poses[target]=actual.copy()
            self.attachment=None;self.business_interval='RETURN'
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
        offsets=[face_outward(self.target_pose,self.face)*.04,
                 np.array([0.,0.,.06]),np.array([0.,.05,.04])]
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
        self.select_grasp(geometry['candidates'][0])
        home,_=self.select_home(np.asarray(geometry['candidates'][0]['tcp_pose_D']))
        self.current_q=home.copy()
        initial=self.checkpoint()
        task_root=self.task_id
        for index,grasp in enumerate(geometry['candidates'][:self.max_grasps]):
            self.restore(initial)
            self.select_grasp(grasp)
            try: starts=list(self.initial_candidates(grasp,home))
            except ValueError as error:
                self.cycle_failures.append(dict(stage='initial_selection',error=str(error)))
                continue
            for start_index,(q,selection) in enumerate(starts):
                self.restore(initial)
                try:
                    # A new simulation initial-state candidate is a separate
                    # native trial. Never change the root q of an existing
                    # MTC task, or concatenate motions across these trials.
                    self.task_id=f'{task_root}-initial-{index}-{start_index}'
                    self.select_grasp(grasp)
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
                    self.cycle_failures.append(dict(stage='first_cycle',candidate_index=index,
                        initial_selection=selection,candidate_id=grasp['candidate_id'],error=str(error)))
        raise ValueError('FIRST_CARTON_OR_CONTINUATION_BOUNDED_CANDIDATES_EXHAUSTED')

    def return_next(self):
        """Try finite next faces/heights, then let the caller revisit grasp branches."""
        drained=self.checkpoint()
        boxes=[b for b in self.scene['boxes'] if b['id'] not in self.completed_ids
               and not any(x['id'] not in self.completed_ids and
                   abs(x['pose'][0]-b['pose'][0])<1e-8 and abs(x['pose'][1]-b['pose'][1])<1e-8
                   and x['pose'][2]>b['pose'][2]+1e-8 for x in self.scene['boxes'])]
        boxes.sort(key=lambda b:(-b['pose'][2],abs(b['pose'][1]),b['pose'][1]))
        for target in boxes:
            self.restore(drained)
            self.target=deepcopy(target);self.target_pose=self.carton_poses[target['id']].copy()
            candidates=self.geometry_candidates()['candidates']
            self.restore(drained)
            for grasp,height in [(g,h) for g in candidates[:min(self.max_grasps,8)]
                                  for h in dict.fromkeys((self.height_m,self.spec['low_height_m']))]:
                self.restore(drained)
                try:
                    self.select_next(target,grasp)
                    self.lift(height)
                    self.approach(grasp)
                    self.attach(next_target=True)
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
            evaluation_profile=CHAIN,replay_interpolation=INTERPOLATION,j1_direction_policy=J1_POLICY,
            first_target_id=self.first_target_id,next_target_id=self.next_target_id,
            initial_state=getattr(self,'initial_state',state),terminal_state=state,records=self.cycle_records,
            planning_wall_s=perf_counter()-self.started,
            robot_motion_s=sum(r['times'][-1] for r in self.cycle_records if r['kind']=='robot_motion'),
            mechanism_display_s=sum(r.get('duration_s',0.) for r in self.cycle_records if r['kind']=='external_axis'),
            event_display_s=sum(r.get('duration_s',0.) for r in self.cycle_records if r['kind']=='event'),
            failures=self.cycle_failures,native_calls_archive=self.call_archive_info(),
            j1_reverse_used_rad=self.reverse_used)
        result['adopted_process']=getattr(self,'adopted_process',None)
        result['phase_budgets']=deepcopy(self.phase_budgets)
        result['initial_selection']=deepcopy(self.initial_selection)
        result['initial_state_checks']=self.home_checks
        result['fingerprint']=digest(result)
        return result

    def save_checkpoint(self,label):
        if self.progress_path:
            result=self.result()
            save_json(self.progress_path.with_name(label.lower()+'.json'),result)


def check_direction_endpoint(start,end,direction):
    return direction*(end[0]-start[0]) >= -J1_REVERSE_LIMIT
