"""Explicit planning-only coupled cycle and committed batch state.

Nominal gravity and receiver drain are assumptions, never physical observations.
All robot edges come from the current native Task. Failed candidates cannot commit.
"""
from copy import deepcopy
from dataclasses import dataclass, field, replace
import math
import numpy as np
from .conveyor_lift import coupled_conveyor_state
from .coupled_lift_checks import assembly_mates, check_coupled_state
from .geometry import OBB
from .lifted_reception import bounds, translated, StartHeight
from .moveit2_backend import box_message, digest, MoveItUnavailable, native_tool_collision_boxes
from .single_contact import SingleContactClient, SingleContactConnector
from .single_extraction import SingleExtractionConnector
from .single_lifted_receiver import SingleLiftedReceiverConnector
from .single_right_receiver import SingleRightReceiverConnector
from .stage_motion_policy import MotionPurpose
from .support import SupportRelationGraph
from .workcell_layout import _obb_record


def message_box(b):
    p=np.asarray(b['pose'],float)
    return OBB(p[:3,3],np.asarray(b['size'])/2,p[:3,:3],b['id'],b['category'])


def coupled_binding(layout, cartons, start_height, end_height, target_id, contacts, *, step=.01):
    if not np.isfinite([start_height,end_height,step]).all() or step<=0 or step>.01:
        raise ValueError('COUPLED_SAMPLE_RESOLUTION')
    n=max(1,math.ceil(abs(end_height-start_height)/step)) if start_height!=end_height else 0
    if n>=4096:raise ValueError('COUPLED_SAMPLE_RESOURCE_LIMIT')
    states=[coupled_conveyor_state(layout,float(h),cartons) for h in np.linspace(start_height,end_height,n+1)]
    # Match FrozenLayoutMotionInput's ordering, including every mechanism body.
    world=lambda s:[box_message(b) for b in s.boxes if b.category!='carton']+[box_message(b) for b in s.boxes if b.category=='carton']
    return dict(schema='m710_coupled_receiver_v2',target_id=target_id,
        samples=[dict(height_m=s.mechanism['height_m'],world=world(s),scene_fingerprint=s.scene_fingerprint) for s in states],
        moving_ids=states[0].mechanism['moving_object_ids'],assembly_pairs=[list(p) for p in assembly_mates(states[0])],
        designed_contacts=[list(p) for p in contacts],maximum_height_step_m=abs(end_height-start_height)/max(n,1),
        continuous_swept_proof=False,geometry_revision=states[0].mechanism['geometry_revision'])


def nominal_drop(box,belt,*,step=.002):
    if not 0<step<=.002:raise ValueError('NOMINAL_DROP_RESOLUTION')
    gap=float(bounds(box)[0][2]-bounds(belt)[1][2])
    local=(box.corners()-belt.center)@belt.rotation
    if gap<=0 or not np.all(np.abs(local[:,:2])<=belt.half_extents[:2]-.01+1e-9):
        raise ValueError('NOMINAL_DROP_REGION_INVALID')
    n=math.ceil(gap/step)
    if n>=4096:raise ValueError('NOMINAL_DROP_RESOURCE_LIMIT')
    # Uniform displacement gives <=2mm spatial steps; time is analytic gravity.
    samples=[dict(t=math.sqrt(2*dz/9.81),box=box_message(translated(box,[0,0,-dz]))) for dz in np.linspace(0,gap,n+1)]
    return dict(gravity_m_s2=9.81,initial_linear_velocity_m_s=[0.]*3,initial_angular_velocity_rad_s=[0.]*3,
        orientation='CONSTANT',samples=samples,drop_m=gap,maximum_displacement_step_m=gap/n,
        first_contact_model='NOMINAL_GRAVITY_NO_IMPACT_OR_STABILITY',physical_execution='NOT_RUN')


@dataclass
class BatchState:
    frozen: object
    cartons: tuple
    q: np.ndarray
    height: float
    ledger: list=field(default_factory=list)
    cycles: list=field(default_factory=list)

    def root(self):
        return dict(q_rad=self.q.tolist(),velocity_rad_s=[0.]*6,height_m=self.height,
            world_cartons=[box_message(b) for b in self.cartons],completed=[e['target_id'] for e in self.ledger],
            predecessor_final_state_receipt=self.cycles[-1]['final_state_receipt'] if self.cycles else None)

    def commit(self,plan):
        if plan['batch_root']!=self.root():raise ValueError('BATCH_COMMIT_ROOT_CHANGED')
        target=plan['target_id'];remaining=self.frozen.remaining_ids([e['target_id'] for e in self.ledger])
        events=plan['events'];types=[e['event_type'] for e in events]
        required=['RELEASE','NOMINAL_FIRST_CONTACT','SAFE_TOOL_SEPARATION','IDEAL_RECEIVER_DRAIN']
        if target not in remaining or any(types.count(t)!=1 for t in required):raise ValueError('PREMATURE_OR_DUPLICATE_DRAIN')
        if [types.index(t) for t in required]!=sorted(types.index(t) for t in required):raise ValueError('DRAIN_EVENT_ORDER')
        if any(e.get('target_id')!=target for e in events if e['event_type'] in required):raise ValueError('DRAIN_TARGET_CHANGED')
        if not plan['separation']['passed'] or not plan['nominal_release']['passed']:raise ValueError('DRAIN_WITHOUT_CHECKS')
        self.q=np.asarray(plan['end_q_rad']);self.height=float(plan['height_m'])
        self.cartons=tuple(b for b in self.cartons if b.name!=target)
        self.ledger.append(dict(target_id=target,last_pose=plan['landed_box'],source='IDEAL_RECEIVER_DRAIN',
            source_stage_id=plan['release_stage_id'],physical_outfeed=False))
        self.cycles.append(plan)


def batch_scene(original,state,robot,layout):
    mechanism=coupled_conveyor_state(layout,state.height,state.cartons)
    fixed=tuple(b for b in mechanism.boxes if b.category!='carton')
    snapshot=deepcopy(original.snapshot)
    snapshot['cartons']=[_obb_record(b) for b in state.cartons]
    snapshot['assembly']['fixed_components']=[_obb_record(b) for b in fixed]
    snapshot['robot'].update(q_rad=state.q.tolist(),tcp_pose_world=robot.fk(state.q).tolist(),
        flange_pose_world=robot.named_link_frames(state.q)['flange'].tolist())
    snapshot['robot'].pop('link_collision_obbs',None)
    snapshot['tool'].pop('rigid_collision_obbs',None);snapshot['tool'].pop('collision_obb',None)
    snapshot['tool']['task_tcp_pose_world']=robot.fk(state.q).tolist()
    snapshot['coupled_mechanism']=mechanism.mechanism;snapshot['batch_committed_root']=state.root()
    snapshot.pop('scene_fingerprint',None);snapshot['scene_fingerprint']=digest(snapshot)
    graph=SupportRelationGraph.build(state.cartons)
    return replace(original,snapshot=snapshot,cartons=state.cartons,fixed_components=fixed,support_graph=graph,
        remaining_stack_names=tuple(b.name for b in state.cartons),receiver=next(b for b in fixed if b.name=='conveyor_transverse'))


class CoupledBatchClient(SingleContactClient):
    def __init__(self,client,*,binding):
        self.coupled_enabled=True
        super().__init__(client,lifted_receiver_binding=binding,right_inner_enabled=True,search_trace_enabled=True)


class CoupledCycleConnector(SingleRightReceiverConnector):
    lift_planner_id='COUPLED_LIFT'

    def validate_template(self,target):
        target.validate()
        if target.geometric_face_id!=self.option['face']['geometric_face_id']:
            raise MoveItUnavailable('COUPLED_BUSINESS_FACE_CHANGED')

    def prepare_receiver_binding(self,target,obstacles):
        b=self.native.lifted_receiver_binding
        root=b.get('pre_approach_transition',b)
        if [box_message(o) for o in obstacles]!=root['samples'][0]['world']:raise MoveItUnavailable('COUPLED_ROOT_WORLD_MISMATCH')
        return b,next(message_box(x) for x in b['samples'][-1]['world'] if x['id']=='conveyor_transverse')

    def high_world(self):
        b=self.receiver_binding['pre_approach_transition'] if getattr(self,'_pre_approach_lift',False) else self.receiver_binding
        return [message_box(x) for x in b['samples'][-1]['world']]

    def prepare_approach_start(self,start,target,obstacles,seed,row):
        if 'pre_approach_transition' not in self.receiver_binding:return start,obstacles
        self._pre_approach_lift=True
        try:
            state,world,success=self.lift_at(start,target,obstacles,seed,row,stage='pregrasp')
        finally:self._pre_approach_lift=False
        self.receiver_raised=False
        if not success:raise MoveItUnavailable('PRE_APPROACH_COUPLED_SWEEP_REJECTED')
        # A newly generated external-axis parent is a new connection context.
        # Keep prior prefix failures in the enclosing history, never transplant
        # exclusions or pretend this new request continues the old search tree.
        self._lift_recovery_state={}
        return state,world

    def expected_contact(self,target,suction):return SingleContactConnector.expected_contact(self,target,suction)

    def prepare_contact_start(self,start,target,obstacles,seed,row):
        if self.option.get('late_receiver_sync'):return start,obstacles
        return super().prepare_contact_start(start,target,obstacles,seed,row)

    def complete_contact_plan(self,plan,row,end,target,obstacles,suction,seed):
        if not self.option.get('late_receiver_sync'):return super().complete_contact_plan(plan,row,end,target,obstacles,suction,seed)
        plan['events']=deepcopy(self.receiver_events)
        return SingleExtractionConnector.complete_contact_plan(self,plan,row,end,target,obstacles,suction,seed)

    def lift_attached_at(self,end,target,obstacles,attachment,seed):
        world=[b for b in obstacles if b.name!=target.target_id]
        final_world=[b for b in self.high_world() if b.name!=target.target_id]
        req=self._build_native_request(end,end,world,seed=seed,stage='extraction',attachment=attachment)
        receipt,_,transition=self._native_parent_for_request(end,req)
        if transition is not None:raise MoveItUnavailable('LATE_SYNC_OWNERSHIP_CHANGED')
        stage_s,timeout=self._native_limits();serialized=[box_message(b) for b in final_world]
        req.update(parent_stage_id=receipt.stage_id,parent_state_receipt=receipt.evidence(),ownership_transition=None,
            stage_id=self.native_task_id+':'+str(next(self._native_stage_ids)),pipeline_id='external_axis',planner_id='COUPLED_LIFT',
            receiver_end_world=serialized,receiver_end_fingerprint=digest(serialized),allowed_planning_time_s=stage_s,
            purpose=MotionPurpose.EXTRACTION_PROCESS.value)
        raw=self.native.request(req,timeout=timeout,cancelled=self._native_cancelled)
        if raw['status']!='SUCCESS':return None,None,raw
        if raw.get('generation_source')!='NATIVE_EXTERNAL_AXIS' or not np.array_equal(raw.get('terminal_q'),end):
            raise MoveItUnavailable('LATE_SYNC_RETURN_IDENTITY')
        self.native_generated.append(dict(raw,stage='extraction',submitted_request=deepcopy(req),authoritative_status='SKIPPED_PLANNING_ONLY'))
        state=self._native_issue_state(end,req['stage_id'],self._native_state_context(dict(req,world=serialized),endpoint=end,attachment=attachment))
        return state,final_world,raw

    @property
    def planar(self):return len(self.option['segment_directions_world'])==2

    @property
    def lateral_sign(self):return self.option['segment_directions_world'][0][1]

    def _initial_proximity(self,*args):
        return (SingleRightReceiverConnector if self.planar else SingleExtractionConnector)._initial_proximity(self,*args)

    def _build_native_request(self,*args,**kwargs):
        req=SingleContactConnector._build_native_request(self,*args,**kwargs)
        if getattr(self,'_pre_approach_lift',False):req['coupled_leg']='PRE_APPROACH'
        if getattr(self,'option',None) and self.planar and kwargs.get('stage')=='extraction' and kwargs.get('initial_proximity') is not None:
            req.update(receiver_template='COUPLED_PLANAR',process_substage=getattr(self,'_side_substage','RIGHT_INNER_Y'))
        return req

    def region_candidates(self,*args):
        return (SingleRightReceiverConnector if self.planar else SingleLiftedReceiverConnector).region_candidates(self,*args)

    def region_motion(self,*args):
        return (SingleRightReceiverConnector if self.planar else SingleLiftedReceiverConnector).region_motion(self,*args)

    def complete_extraction_plan(self,plan,row,end,target,obstacles,attachment,seed):
        if self.planar and self._native_selected_records(end)[-1].get('process_policy',{}).get('terminal_proximity')!=row['extraction_samples']['terminal_tracker']:
            raise MoveItUnavailable('COUPLED_FINAL_NATIVE_HISTORY_MISMATCH')
        box=attachment.box_at(end);height=StartHeight.capture(target.box)
        gap=float(bounds(box)[0][2]-bounds(self.high_belt)[1][2])
        expected=height.z_b-self.option['height_m']
        check=dict(passed=abs(gap-expected)<=self.ik['position_tolerance_m'],actual_gap_m=gap,expected_gap_m=expected)
        row['release_ready_check']=check
        def reject(reason,detail):
            row.update(complete=False,failure=dict(reason='ACTUAL_RELEASE_REGION_NOT_REACHED',
                scope='COMPLETE_CYCLE_CANDIDATE',blocking_stage=reason,detail=detail))
            return None,row
        if self.option.get('late_receiver_sync'):
            lifted,lift_world,checked=self.lift_attached_at(end,target,obstacles,attachment,seed)
            if lifted is None:return reject('LATE_RECEIVER_SYNC',checked)
            end=lifted;obstacles=[*lift_world,target.box]
            plan['events'].append(dict(event_type='EXTERNAL_AXIS_MOTION',stage_id=checked['stage_id'],
                attached=True,**checked['external_axis']))
        if not check['passed']:return reject('RELEASE_HEIGHT',check)
        try:drop=nominal_drop(box,self.high_belt)
        except ValueError as exc:return reject('NOMINAL_DROP_GEOMETRY',str(exc))
        world=[b for b in obstacles if b.name!=target.target_id]
        req=self._build_native_request(end,end,world,seed=seed,stage='extraction',attachment=attachment)
        receipt,_,transition=self._native_parent_for_request(end,req)
        if transition is not None:raise MoveItUnavailable('RELEASE_PARENT_OWNERSHIP_CHANGED')
        stage_s,timeout=self._native_limits()
        req.update(pipeline_id='semantic_event',planner_id='NOMINAL_DROP_RELEASE',nominal_release=drop,
            parent_stage_id=receipt.stage_id,parent_state_receipt=receipt.evidence(),ownership_transition=None,
            stage_id=self.native_task_id+':'+str(next(self._native_stage_ids)),allowed_planning_time_s=stage_s,
            purpose=MotionPurpose.EXTRACTION_PROCESS.value)
        raw=self.native.request(req,timeout=timeout,cancelled=self._native_cancelled)
        if raw['status']!='SUCCESS':return reject('NOMINAL_DROP_NATIVE',raw)
        if raw.get('generation_source')!='SEMANTIC_EVENT' or not np.array_equal(raw.get('terminal_q'),end):
            raise MoveItUnavailable('NOMINAL_RELEASE_SOURCE_MISMATCH')
        self.native_generated.append(dict(raw,stage='extraction',submitted_request=deepcopy(req),authoritative_status='SKIPPED_PLANNING_ONLY'))
        landed=message_box(drop['samples'][-1]['box']);released_world=[*world,landed]
        terminal=dict(req,attachment=None,world=[box_message(b) for b in released_world],stage='withdrawal',
            process_policy=dict(req['process_policy'],target=box_message(landed),attached=False,stage='withdrawal'))
        state=self._native_issue_state(end,req['stage_id'],self._native_state_context(terminal))
        def separation(q):
            request=self._build_native_request(q,q,released_world,seed=seed,stage='pregrasp',target_contact=landed)
            parent,_,transition=self._native_parent_for_request(q,request)
            if transition is not None:raise MoveItUnavailable('SEPARATION_OWNERSHIP_CHANGED')
            request.update(op='check_start',coupled_separation_probe=True,parent_stage_id=parent.stage_id,parent_state_receipt=parent.evidence(),ownership_transition=None)
            result=self.native.request(request,timeout=timeout,cancelled=self._native_cancelled)
            return dict(passed=result['status']=='SUCCESS',response=result)
        # check_start rejects invalid states at protocol scope; only use a native
        # checked departure, then require the restored ordinary clearance gate.
        tools,_=native_tool_collision_boxes(self,state)
        direction=np.asarray(target.outward_normal)
        normal_gap=min(float(np.min(t.corners()@direction)-np.max(landed.corners()@direction)) for t in tools)
        distance=max(.005-normal_gap,0.)+2*float(self.ik['position_tolerance_m'])
        goal=self.physical_contact_pose(state).copy();goal[:3,3]+=direction*distance
        path,failure,evidence=self._native_plan(state,None,released_world,seed=seed,stage='withdrawal',target_contact=landed,
            goal_pose=self.virtual_from_physical(goal),purpose=MotionPurpose.DEPARTURE_PROCESS,
            allow_ompl=False,max_ompl_attempts=0,try_ptp=False,try_prior=False)
        if failure:return reject('TOOL_DEPARTURE',dict(failure=failure,evidence=evidence))
        final=path[-1];separated=separation(final)
        if not separated['passed']:return reject('TOOL_SEPARATION',separated)
        events=[*plan['events'],dict(event_type='RELEASE',target_id=target.target_id,stage_id=req['stage_id'],
            q_rad=np.asarray(end).tolist(),box=box_message(box),source='NATIVE_SAME_POSE_OWNERSHIP_EVENT'),
            dict(event_type='NOMINAL_FIRST_CONTACT',target_id=target.target_id,box=box_message(landed),model=drop['first_contact_model']),
            dict(event_type='SAFE_TOOL_SEPARATION',target_id=target.target_id,q_rad=np.asarray(final).tolist()),
            dict(event_type='IDEAL_RECEIVER_DRAIN',target_id=target.target_id,last_box=box_message(landed),physical_outfeed=False)]
        records=self._native_selected_records(final)
        plan.update(status='COUPLED_COMPLETE_CYCLE_PLANNED_NOT_EXECUTABLE',target_id=target.target_id,
            end_q_rad=np.asarray(final).tolist(),height_m=self.option['height_m'],option=deepcopy(self.option),
            release_stage_id=req['stage_id'],landed_box=box_message(landed),nominal_release=dict(drop,passed=True,native=raw),
            separation=separated,target_still_attached=False,events=events,withdrawal=[np.asarray(q).tolist() for q in path],
            stages=[{k:deepcopy(r[k]) for k in ('stage','stage_id','parent_stage_id','generation_source','points','terminal_q','pipeline_id','planner_id','external_axis') if k in r} for r in records],
            final_state_receipt=final.native_receipt,batch_root=deepcopy(self.batch_root))
        row.update(actual_end_q_rad=np.asarray(final).tolist(),complete=True,failure=None)
        return plan,row
