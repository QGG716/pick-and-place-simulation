"""Explicit right-inner +Y / -X suffix, sharing the Hmax candidate evaluator."""
from collections import deque
from copy import deepcopy
from time import perf_counter
import numpy as np
from .single_lifted_receiver import SingleLiftedReceiverConnector
from .single_extraction import extraction_sample_check
from .side_transfer import SideTransferTracker
from .lifted_reception import layer_role, directional_contact, side_turn_regions
from .moveit2_backend import MoveItUnavailable, digest, native_tool_collision_boxes
from .stage_motion_policy import MotionPurpose


def right_inner_targets(targets, target, originals, remaining):
    provenance=layer_role(target, originals, remaining)
    if provenance['role']!='RIGHT_END':
        raise ValueError('RIGHT_INNER_REQUIRES_ORIGINAL_RIGHT_END')
    face=directional_contact(target,[0,1,0])
    if face is None:raise ValueError('RIGHT_INNER_FACE_NOT_WORLD_ALIGNED')
    selected=[t for t in targets if t.face_role=='SIDE' and t.geometric_face_id==face['geometric_face_id']]
    return selected,dict(template='RIGHT_INNER_HMAX',role=provenance,face=face,
        scope='EXPLICIT_TEMPLATE_ONLY_NOT_GLOBAL_FACE_ORDER',
        availability='AVAILABLE' if selected else 'INNER_FACE_BLOCKED_OR_DISABLED',newly_unloaded_count=0)


def interval_samples(a,b,tolerance,reverse=False):
    a+=2*tolerance;b-=2*tolerance
    if a>b:return
    yield b if reverse else a
    if b-a<=tolerance:return
    yield a if reverse else b
    queue=deque([(a,b)])
    while queue:
        a,b=queue.popleft()
        if b-a<=2*tolerance:continue
        m=(a+b)/2;yield m;queue.extend(((a,m),(m,b)))


def rectangle_samples(rect,tolerance):
    """Fair diagonal exploration; resolution, not a small attempt quota."""
    a,b,c,d=rect;xs=iter(interval_samples(a,b,tolerance,True));ys=iter(interval_samples(c,d,tolerance))
    xx=[];yy=[];ex=ey=False;k=0
    while True:
        if not ex:
            try:xx.append(next(xs))
            except StopIteration:ex=True
        if not ey:
            try:yy.append(next(ys))
            except StopIteration:ey=True
        if not xx or not yy:return
        for j in range(max(0,k-len(xx)+1),min(k,len(yy)-1)+1):
            i=k-j
            if i<len(xx):yield xx[i],yy[j]
        if ex and ey and k>=len(xx)+len(yy)-2:return
        k+=1


def retain_exact_native_start(path, start):
    """Native returned samples have values; lineage belongs to the real input.

    Never infer a new root/receipt from FK or tolerate a changed first sample.
    """
    if not path or not np.array_equal(path[0],start):
        raise MoveItUnavailable('RIGHT_TURN_NATIVE_START_CHANGED')
    if hasattr(path[0],'native_receipt') and path[0].native_receipt!=start.native_receipt:
        raise MoveItUnavailable('RIGHT_TURN_NATIVE_START_RECEIPT_CHANGED')
    path[0]=start
    return path


class SingleRightReceiverConnector(SingleLiftedReceiverConnector):
    def complete_extraction_plan(self,plan,row,end,target,obstacles,attachment,seed):
        record=self._native_selected_records(end)[-1]
        native=record.get('process_policy',{}).get('terminal_proximity')
        if native!=row['extraction_samples']['terminal_tracker']:
            raise MoveItUnavailable('RIGHT_FINAL_NATIVE_HISTORY_MISMATCH')
        return super().complete_extraction_plan(plan,row,end,target,obstacles,attachment,seed)

    def validate_template(self,target):
        target.validate()
        if (target.face_role!='SIDE' or not np.allclose(target.outward_normal,[0,1,0],atol=1e-12,rtol=0)
                or getattr(self,'template_provenance',{}).get('role',{}).get('role')!='RIGHT_END'):
            raise MoveItUnavailable('RIGHT_INNER_TEMPLATE_SCOPE')

    def _initial_proximity(self,target,obstacles,support_names):
        if self.collision_policy.stack_contact_mode!='planner_relaxed_physics_checked':
            raise MoveItUnavailable('RIGHT_INNER_REQUIRES_DECLARED_STACK_POLICY')
        tracker=SideTransferTracker(target,[b for b in obstacles if b.name in self.stack_carton_names],
            self.collision_policy,self.collision_margin_m,self.contact_tolerance_m)
        return tracker,tracker.state_failure(target,obstacles,support_names)

    def _build_native_request(self,*args,**kwargs):
        req=super()._build_native_request(*args,**kwargs)
        if kwargs.get('stage')=='extraction' and kwargs.get('initial_proximity') is not None:
            req['receiver_template']='RIGHT_INNER_HMAX'
            req['process_substage']=getattr(self,'_side_substage','RIGHT_INNER_Y')
        return req

    def region_candidates(self,region,payload,payload_world,attachment,start,row):
        if not np.allclose(self.high_belt.rotation,np.eye(3),atol=1e-12,rtol=0):
            raise MoveItUnavailable('RIGHT_REGION_REQUIRES_WORLD_ALIGNED_BELT')
        a,b,c,d=region['projection_center_limits_before_occupancy'];x,y=self.high_belt.center[:2]
        sign=getattr(self,'lateral_sign',1.)
        raw=[a+x,min(b+x,payload.center[0]),max(c+y,payload.center[1]) if sign>0 else c+y,
             d+y if sign>0 else min(d+y,payload.center[1])]
        tools,_=native_tool_collision_boxes(self,start)
        compliant={box.name for box in self.robot_state_validator._compliant_boxes(start)}
        tracker=SideTransferTracker(payload,[o for o in payload_world if o.name in self.stack_carton_names],
            self.collision_policy,self.collision_margin_m,self.contact_tolerance_m)
        screened,blockers=side_turn_regions([raw],payload,tools,payload_world,sign,
            support_ids=tracker.planar,compliant_ids=compliant,stack_ids=self.stack_carton_names,
            ignore_flexible_neighbors=self.collision_policy.compliant_cup_neighbor_contact_mode=='ignore',
            clearance_m=self.collision_policy.free_space_clearance_m)
        row['side_region']=dict(raw_world_rectangle=raw,screened_world_rectangles=screened,
            conservative_blockers=blockers,scope='SCREEN_ONLY_RAW_REGION_RETAINED',
            original_role=deepcopy(self.template_provenance),robot_reachability='NOT_EVALUATED',
            planar_support_ids=tracker.planar)
        # Round-robin includes raw portions even when the conservative screen is empty.
        streams=deque(iter(rectangle_samples(r,float(self.ik['position_tolerance_m']))) for r in [*screened,raw])
        seen=set()
        while streams:
            stream=streams.popleft()
            try:xx,yy=next(stream)
            except StopIteration:continue
            streams.append(stream)
            if (xx,yy) in seen:continue
            seen.add((xx,yy));yield np.array([xx,yy,payload.center[2]])

    def _validate_checked_turn(self,prefix,start,path,records,attachment,tracker):
        p=prefix['path'];second=prefix['second_path']
        if (not prefix['evidence']['sample_check']['passed']
                or not np.array_equal(p[0],start) or not np.array_equal(second[0],p[-1])
                or second[0].native_receipt!=p[-1].native_receipt
                or records[-2]['stage_id']!=p[-1].native_receipt['stage_id']
                or records[-1]['parent_stage_id']!=records[-2]['stage_id']
                or digest([np.asarray(q).tolist() for q in p])!=prefix['path_sha256']
                or tracker.evidence()!=prefix['tracker'].evidence()
                or digest(attachment.rigid.tcp_from_box.tolist())!=prefix['attachment_sha256']):
            raise MoveItUnavailable('RIGHT_TURN_CONTEXT_OR_HISTORY_CHANGED')

    def region_motion(self,start,center,target,attachment,payload_world,supports,tracker,seed,required,guard,entry):
        root=self.physical_contact_pose(start);box=attachment.box_at(start)
        entry['turn_center_world_m']=[float(box.center[0]),float(center[1]),float(box.center[2])]
        turn=root.copy();turn[1,3]+=float(center[1]-box.center[1])
        key=digest(dict(y=float(center[1]),parent=start.native_receipt,attachment=attachment.rigid.tcp_from_box.tolist(),
            context=entry['context_sha256'],tracker=tracker.evidence()))
        def generate(q,pose,history,substage):
            self._side_substage=substage
            try:
                result=self._native_plan(q,None,payload_world,seed=seed,stage='extraction',
                    purpose=MotionPurpose.EXTRACTION_PROCESS,goal_pose=self.virtual_from_physical(pose),
                    attachment=attachment,support_names=supports,initial_proximity=history.clone(),
                    allow_ompl=False,max_ompl_attempts=0,try_ptp=False,try_prior=False)
                attempts=result[2].get('attempts',[])
                if attempts and attempts[-1].get('status') in {'NATIVE_PROCESSING_FAILED','ERROR','CANCELLED'}:
                    guard()
                    raise MoveItUnavailable('RIGHT_INNER_NATIVE_REQUEST_ERROR:'+str(attempts[-1].get('failure')))
                if result[1] is None:
                    retain_exact_native_start(result[0],q)
                return result
            finally:self._side_substage='RIGHT_INNER_Y'
        def geometry(q,goal,direction):
            origin=self.physical_contact_pose(q)
            return dict(direction_world=direction,distance_m=float(np.linalg.norm(goal[:3,3]-origin[:3,3])),
                start_physical_pose=origin.tolist(),goal_physical_pose=goal.tolist(),
                required_terminal_clearance_m=required,method='RIGHT_INNER_TWO_LEG_LIN',support_names=supports)
        def brief(evidence):
            attempts=evidence.get('attempts',[])
            if not attempts:return dict(status='NOT_OBSERVED')
            last=attempts[-1]
            return {**{k:deepcopy(last.get(k)) for k in ('status','task_id','stage_id','parent_stage_id','request_id',
                'solver_message','moveit_error_code','native_solver_calls','failure')},
                'process_policy':{'terminal_proximity':deepcopy(last.get('process_policy',{}).get('terminal_proximity'))}}
        entry['segments']=[]
        cached=self._region_turn_cache.get(key)
        if cached is None:
            began=perf_counter();first,failure,evidence=generate(start,turn,tracker,'RIGHT_INNER_Y');guard()
            leg=dict(substage='RIGHT_INNER_Y',native_result=brief(evidence),
                planning_s=perf_counter()-began,reused=False,sample_check={'status':'NOT_REACHED'})
            entry['segments'].append(leg)
            if failure:return first,failure,evidence,{},None,tracker.clone()
            self._region_path_binding(start,first,{'stages':self._native_selected_records(start)},leg['native_result'])
            first_geometry=geometry(start,turn,[0.,getattr(self,'lateral_sign',1.),0.]);history=tracker.clone();began=perf_counter()
            samples=extraction_sample_check(self,first,attachment,target,first_geometry,history,payload_world,supports,terminal=False)
            leg.update(sample_check=samples,sample_check_s=perf_counter()-began);guard()
            if type(samples.get('passed')) is not bool or samples['passed']!=(samples.get('failure') is None):
                raise MoveItUnavailable('RIGHT_TURN_SAMPLE_RESULT_UNKNOWN')
            if not samples['passed']:
                entry['first_leg_postcheck_failure']=deepcopy(samples['failure'])
                return [],samples['failure'],evidence,first_geometry,None,history
            record=self._native_selected_records(first[-1])[-1]
            native_history=leg['native_result'].get('process_policy',{}).get('terminal_proximity')
            if native_history!=history.evidence():raise MoveItUnavailable('RIGHT_TURN_NATIVE_HISTORY_MISMATCH')
            cached=dict(path=first,tracker=history,first_attempt=deepcopy(leg),path_sha256=digest([np.asarray(q).tolist() for q in first]),
                attachment_sha256=digest(attachment.rigid.tcp_from_box.tolist()),evidence=dict(substage='RIGHT_INNER_Y',
                    stage_id=record['stage_id'],parent_stage_id=record['parent_stage_id'],geometry=first_geometry,
                    sample_check=samples,points=deepcopy(record['points'])))
            self._region_turn_cache[key]=cached
        else:
            entry['segments'].append(dict(substage='RIGHT_INNER_Y',stage_id=cached['evidence']['stage_id'],
                parent_stage_id=cached['evidence']['parent_stage_id'],reused=True,planning_s=0.,
                sample_check=deepcopy(cached['evidence']['sample_check'])))
        guard();first=cached['path'];q=first[-1];goal=self.physical_contact_pose(q).copy()
        goal[0,3]+=float(center[0]-attachment.box_at(q).center[0])
        before=perf_counter();second,failure,evidence=generate(q,goal,cached['tracker'],'RIGHT_INNER_X');guard()
        entry['segments'].append(dict(substage='RIGHT_INNER_X',native_result=brief(evidence),
            planning_s=perf_counter()-before,reused=False,sample_check={'status':'NOT_REACHED'}))
        geom=geometry(q,goal,[-1.,0.,0.])
        if failure:return [],failure,evidence,geom,None,cached['tracker'].clone()
        prefix={**cached,'second_path':second}
        return [*first,*second],None,evidence,geom,prefix,cached['tracker'].clone()
