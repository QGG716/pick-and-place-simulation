"""Sequential, fixed-layout S50 face unloading with checked low-drop reception.

No carton is removed until its actual planned attachment, release and outfeed
have completed. Candidate failures are retained; all movement is solved here.
"""
import math
import json,hashlib
from copy import deepcopy
from time import perf_counter
import numpy as np
from .geometry import OBB,rotation_matrix_from_rpy,rotation_vector_from_matrix,rotation_matrix_from_rotation_vector
from .ik import solve_ik
from .planner import RRTConnectPlanner
from .s50_kinematic_sim import ik_candidates,save,simulation_coordinates
from .s50_kinematic_motion import posture,posture_cost,receiving_height,continuous_samples
from .s50_sim_collision import SimCollision
from .independent_cups import build_m710_independent_cup_array,evaluate_independent_cup_geometry
from .support import SupportRelationGraph
from .conveyor_lift import coupled_conveyor_state
from .joint_search_outfeed import outfeed_path,PARK_FIRST
from .coupled_batch import message_box
from .workcell_layout import canonical_digest


def pose_line(a,b,step=.025):
    rv=rotation_vector_from_matrix(b[:3,:3]@a[:3,:3].T)
    n=max(1,math.ceil(np.linalg.norm(b[:3,3]-a[:3,3])/step),math.ceil(np.linalg.norm(rv)/.06))
    rows=[]
    for u in np.linspace(0,1,n+1):
        T=np.eye(4);T[:3,:3]=rotation_matrix_from_rotation_vector(u*rv)@a[:3,:3]
        T[:3,3]=(1-u)*a[:3,3]+u*b[:3,3];rows.append(T)
    return rows


class FaceRun:
    def __init__(self,c):
        self.c=c;self.robot=c.robot;self.checker=SimCollision(c);self.height=.6;self.q=None
        self.active={b.name:b for b in c.layout.cartons()};c.active_cartons=tuple(self.active.values())
        self.completed=[];self.target=None;self.box=None;self.attached=False;self.relative=None;self.cups=None
        self.samples=[];self.stages=[];self.failures=[];self.tasks=[];self.compactness=[];self.started=perf_counter()
        self.home=None;self.front=np.asarray(c.frozen['initial_physical_contact_target'])[:3,:3]
        self.previous_wall_s=0.;self.previous_state_checks=0;self.previous_pair_queries=0
        self.front=rotation_matrix_from_rpy(np.deg2rad(c.settings.get('task_front_tool_roll_deg',0.)),0,0)@self.front
        self.cup_array=build_m710_independent_cup_array()
        self.report=dict(schema='s50_full_kinematic_cycle_v1',task_scope='FULL_FRONT_FACE_FIXED_STATION',status='RUNNING',
            scope=c.settings['scope'],settings=deepcopy(c.settings),source_model_sha256=c.definition['definition_sha256'],
            source_step_path=c.definition['source_step_path'],source_step_sha256=c.definition['source_step_sha256'],
            environment_fingerprint=c.frozen['environment_fingerprint'],world_from_model_base=c.world.tolist(),
            flange_from_physical_contact=c.frames['nominal_compressed_contact'].tolist(),
            flange_from_virtual_task_tcp=c.frames['virtual_task_tcp'].tolist(),initial_carton_ids=c.frozen['initial_carton_ids'],
            initial_cartons=len(self.active),completed_cartons=0,remaining_cartons=len(self.active),
            controller_mapping_verified=False,dynamics_verified=False,native_planning_required=False,
            tool_mass_kg=c.intrinsic['tool_mass_kg'],carton_mass_kg=c.intrinsic['carton_mass_kg'],core_pin_fk_max_error=self.checker.fk_crosscheck,
            simulation_scene_fingerprint=canonical_digest(dict(environment=c.frozen['environment_fingerprint'],
                model=c.definition['definition_sha256'],conventions=c.settings)),
            samples=self.samples,stages=self.stages,failures=self.failures,tasks=self.tasks,compactness=self.compactness,
            layout_gap_m=c.settings['layout_gap_m'],maximum_release_drop_m=c.settings['face_run']['maximum_release_drop_m'],
            layout_motion_during_unloading=False,continuous_collision_certified=False)
        self.report['tool_mount']=dict(profile=c.intrinsic.get('mount_profile','INHERITED_WANTAI_CONNECTOR'),
            removed_body_ids=c.intrinsic.get('removed_body_ids',[]),connector_extension_m=c.intrinsic['connector_extension_m'],
            added_mount_body_ids=c.intrinsic.get('added_mount_body_ids',[]),standoff_m=c.intrinsic.get('mount_standoff_m'),
            flange_spacer_m=c.intrinsic.get('flange_spacer_m',0.),
            flange_to_contact_m=float(np.linalg.norm(c.frames['nominal_compressed_contact'][:3,3])))

    @classmethod
    def resume_empty_boundary(cls,c,path):
        """Continue recorded world state; numerical searches restart explicitly."""
        raw=path.read_bytes();data=json.loads(raw);last=data['samples'][-1]
        if data['status'] not in ['KINEMATIC_FACE_ATTEMPT_FAILED','CANCELLED_AT_CHECKED_EMPTY_BOUNDARY']:
            raise ValueError('RESUME_REQUIRES_FINISHED_FAILED_OR_CANCELLED_ATTEMPT')
        if last['attached'] or last['phase'] not in ['task_start','home','carton_complete','belt_raise','belt_lower']:
            raise ValueError('RESUME_REQUIRES_CHECKED_EMPTY_TASK_BOUNDARY')
        r=cls(c)
        if data['environment_fingerprint']!=c.frozen['environment_fingerprint'] or data['source_model_sha256']!=c.definition['definition_sha256']:
            raise ValueError('RESUME_SCENE_OR_MODEL_CHANGED')
        r.report=data;r.samples=data['samples'];r.stages=data['stages'];r.failures=data['failures'];r.tasks=data['tasks'];r.compactness=data['compactness']
        r.completed=list(data['completed_carton_ids']);r.active={name:box for name,box in r.active.items() if name not in r.completed}
        if set(r.active)!=set(data['remaining_carton_ids']):raise ValueError('RESUME_CARTON_IDENTITIES_CHANGED')
        c.active_cartons=tuple(r.active.values());c.outfed_carton_ids=set(r.completed);c.attempts=data['ik_attempts']
        c.rng.bit_generator.state=data['numerical_rng_state']
        r.q=np.asarray(last['q_rad']);r.height=last['belt_height_m'];r.home=np.asarray(data['independent_home_q_rad'])
        r.target=next(b for b in c.layout.cartons() if b.name==last['target']);r.box=np.asarray(last['world_from_carton'])
        c.settings['target']=r.target.name
        r.previous_wall_s=data['elapsed_wall_s'];r.previous_state_checks=data['collision_state_checks'];r.previous_pair_queries=data['collision_narrow_pair_queries']
        r.report.setdefault('continuations',[]).append(dict(source_report=str(path),source_sha256=hashlib.sha256(raw).hexdigest(),
            previous_status=data['status'],previous_failure=data.pop('failure',None),completed_cartons=len(r.completed),
            numerical_search='RESTART_NO_RETAINED_RRT_TREE',same_recorded_world=True))
        r.report['status']='RUNNING'
        if not r.valid(r.q,'resume_empty_boundary'):raise ValueError('RESUME_RECORDED_STATE_COLLIDES')
        return r

    def valid(self,q,phase,*,relative=None,box=None,height=None,contact=True):
        body=self.robot.fk(q)@relative if relative is not None else (self.box if box is None else box)
        ok=self.checker.check(q,self.height if height is None else height,body,contact,phase)
        if not ok:self.failures.append(dict(record_kind='REJECTED_CANDIDATE_STATE',target=self.target.name if self.target else None,**deepcopy(self.checker.last)))
        return ok

    def event(self,phase,dt=0.,q=None,velocity=None,acceleration=None):
        if q is not None:self.q=np.asarray(q).copy()
        if self.attached:self.box=self.robot.fk(self.q)@self.relative
        row=dict(time_s=(self.samples[-1]['time_s'] if self.samples else 0.)+float(dt),phase=phase,target=self.target.name,
            task_index=len(self.completed),completed_carton_ids=list(self.completed),q_rad=self.q.tolist(),
            belt_height_m=float(self.height),attached=self.attached,world_from_carton=self.box.tolist(),
            world_from_physical_contact=self.robot.fk(self.q).tolist(),posture=posture(self.robot,self.q),
            physical_contact_from_attached_box=None if self.relative is None else self.relative.tolist(),
            commanded_cup_count=self.cups['commanded_count'] if self.attached and self.cups else 0)
        if velocity is not None:row.update(q_velocity_rad_s=np.asarray(velocity).tolist(),q_acceleration_rad_s2=np.asarray(acceleration).tolist())
        self.samples.append(row)

    def mark(self,phase,start,method,**extra):
        self.stages.append(dict(target=self.target.name,task_index=len(self.completed),phase=phase,first_sample=start,
            last_sample=len(self.samples)-1,status='PASS',method=method,**extra))
        if phase in ['release','home']:
            value=self.checker.compactness(self.q,self.height,phase);value['target']=self.target.name
            self.compactness.append(value)
        save(self.c.output/'progress.json',dict(status='RUNNING',completed=self.completed,target=self.target.name,
            phase=phase,samples=len(self.samples),checks=self.checker.check_count,failures=len(self.failures)))
        print(self.target.name,phase,'PASS','samples',len(self.samples),'completed',len(self.completed),flush=True)

    def shaped(self,path,phase,relative=None):
        timing=self.c.settings['timing'];rows,law=continuous_samples(path,timing['joint_velocity_rad_s'],timing['joint_acceleration_rad_s2'])
        for row in rows:
            if not self.valid(row['q'],phase,relative=relative):raise RuntimeError('SMOOTH_PATH_COLLISION:'+phase)
        return rows,law

    def emit(self,rows,law,phase,labels=None):
        begin=len(self.samples);previous=0.;current=phase
        for row in rows[1:]:
            label=phase if labels is None else labels[min(row['knot_index']+1,len(labels)-1)]
            if label!=current and len(self.samples)>begin:
                self.mark(current,begin,'CONTINUOUS_SOLVED_IK');begin=len(self.samples)
            current=label;self.event(label,row['time_s']-previous,row['q'],row['velocity'],row['acceleration']);previous=row['time_s']
        self.mark(current,begin,'CONTINUOUS_SOLVED_IK' if labels else 'CHECKED_JOINT_PATH',timing=law)

    def pointwise(self,poses,start,phase,relative=None):
        path=[np.asarray(start).copy()]
        self.last_pointwise_failure=None
        for point_index,T in enumerate(poses[1:],1):
            seed=path[-1]
            for damping,iterations in [(.008,180),(.002,300),(.0005,300)]:
                r=solve_ik(self.robot,T,seed,max_iterations=iterations,damping=damping,max_step=.15,
                    position_tolerance=.00004,orientation_tolerance=.0001)
                self.c.attempts.append(dict(target=self.target.name,stage=phase,ik_success=r.success,iterations=r.iterations,
                    point_index=point_index,point_count=len(poses),desired_physical_contact=T.tolist(),
                    damping=damping,position_error_m=r.position_error,orientation_error_rad=r.orientation_error,q_rad=r.q.tolist()))
                if r.success:break
                seed=r.q
            reason=None
            if not r.success:reason='POINTWISE_IK_FAILED'
            elif np.max(abs(r.q-path[-1]))>.5:reason='POINTWISE_JOINT_JUMP'
            elif not self.valid(r.q,phase,relative=relative):reason='POINTWISE_COLLISION'
            if reason:
                self.last_pointwise_failure=dict(reason=reason,phase=phase,point_index=point_index,
                    point_count=len(poses),desired_physical_contact=T.tolist(),last_valid_q_rad=path[-1].tolist(),
                    last_valid_physical_contact=self.robot.fk(path[-1]).tolist(),q_rad=r.q.tolist(),
                    ik_success=r.success,position_error_m=r.position_error,orientation_error_rad=r.orientation_error,
                    max_joint_step_rad=float(np.max(abs(r.q-path[-1]))),
                    collision=deepcopy(self.checker.last) if reason=='POINTWISE_COLLISION' else None)
                raise RuntimeError(reason+':'+phase)
            path.append(r.q)
        return path

    def joint_path(self,start,goal,phase):
        planner=RRTConnectPlanner(*self.robot.joint_limits.T,is_state_valid=lambda q:self.valid(q,phase),
            rng=np.random.default_rng(self.c.settings['seed']+len(self.completed)*17+len(self.stages)),
            max_iterations=2400,edge_resolution=.035,step_size=.20)
        plan=planner.plan(start,goal)
        if not plan.success:raise RuntimeError('FINITE_JOINT_SEARCH_FAILED:'+phase+':'+plan.message)
        path=planner.shortcut(plan.path,attempts=30)
        try:return self.shaped(path,phase)
        except RuntimeError:
            # Keep the already checked linear segments when smoothing crosses an obstacle.
            rows=[];offset=0.;laws=[]
            for a,b in zip(path,path[1:]):
                segment,law=self.shaped([a,b],phase);laws.append(law)
                for row in segment if not rows else segment[1:]:rows.append(dict(row,time_s=row['time_s']+offset))
                offset+=law['duration_s']
            return rows,dict(method='CHECKED_RRT_EDGES',duration_s=offset,segments=laws)

    def change_height(self,desired):
        if abs(desired-self.height)<1e-10:return
        begin=len(self.samples);phase='belt_raise' if desired>self.height else 'belt_lower'
        for h in np.linspace(self.height,desired,max(2,math.ceil(abs(desired-self.height)/.025)+1))[1:]:
            if not self.valid(self.q,phase,height=h):raise RuntimeError('BELT_MOTION_COLLISION')
            dt=abs(h-self.height)/self.c.settings['timing']['belt_velocity_m_s'];self.height=float(h);self.event(phase,dt)
        self.mark(phase,begin,'BOX_BOTTOM_MINUS_100MM_CLAMPED_TO_STROKE')

    def score(self,q,reference=None):
        preferences=self.c.settings['posture_preferences']
        weight=preferences.get('j2_more_negative_weight',.2)
        return posture_cost(self.robot,q,preferences,reference)+weight*((np.rad2deg(q[1])+190)/200)**2

    def withdrawal_score(self,path):
        q=np.rad2deg(path);preferences=self.c.settings['posture_preferences']
        span=float(np.ptp(q[:,0]));scale=preferences.get('j1_withdrawal_preferred_span_deg',15.)
        value=preferences.get('j1_withdrawal_weight',0.)*(span/scale)**2
        reposition=0. if self.q is None else abs(float(q[0,0]-np.rad2deg(self.q[0])))
        reposition_weight=preferences.get('j1_empty_reposition_weight',1.)
        value+=reposition_weight*(reposition/scale)**2
        value+=preferences.get('j2_more_negative_weight',.2)*float(np.mean(((q[:,1]+190)/200)**2))
        value+=.1*float(np.mean([posture_cost(self.robot,row,preferences) for row in path]))
        forward=np.array([posture(self.robot,row)['upper_arm_forward_tilt_deg'] for row in path])
        free=preferences.get('upper_arm_forward_free_deg',30.)
        value+=.9*preferences.get('upper_arm_large_forward_weight',0.)*float(np.mean((np.maximum(0,forward-free)/30.)**2))
        return dict(score=value,j1_range_deg=[float(q[:,0].min()),float(q[:,0].max())],j1_span_deg=span,
            j1_empty_reposition_deg=reposition,j1_empty_reposition_weight=reposition_weight,
            j2_range_deg=[float(q[:,1].min()),float(q[:,1].max())],j2_start_deg=float(q[0,1]),j2_end_deg=float(q[-1,1]),
            upper_arm_forward_tilt_range_deg=[float(forward.min()),float(forward.max())],upper_arm_forward_free_deg=free,
            preference_semantics='SOFT_BRANCH_ORDERING_NO_LIMIT_OR_COLLISION_EXCEPTION')

    def select_home(self):
        T=np.eye(4);T[:3,:3]=self.front;T[:3,3]=[-.15,0,2.25]
        seeds=[simulation_coordinates(self.c.settings,np.deg2rad(q)) for q in [[10,-30,100,-250,-80,180],[160,-40,-80,-60,80,180],[10,-30,100,-70,80,0]]]
        found=ik_candidates(self.c,T,seeds=seeds,draws=80,stage='independent_home',limit=6,
            valid=lambda q:all(self.valid(q,'home_height',height=h,contact=False) for h in np.linspace(.55,1.5,12)))
        if not found:raise RuntimeError('NO_CHECKED_INDEPENDENT_HOME')
        preferences=self.c.settings['posture_preferences']
        def home_cost(q):
            j1=(np.rad2deg(q[0])+180)%360-180
            return self.score(q,seeds[0])+preferences.get('j1_withdrawal_weight',0.)*(j1/preferences.get('j1_withdrawal_preferred_span_deg',15.))**2
        self.report['independent_home_candidates']=[dict(q_rad=q.tolist(),score=home_cost(q)) for q in found]
        self.q=min(found,key=home_cost).copy();self.home=self.q.copy()
        self.report['independent_home_q_rad']=self.home.tolist()
        self.event('home');self.mark('home',0,'INDEPENDENT_IK_WITH_FULL_BELT_STROKE_CHECK')

    def extraction(self,qgrasp,relative,extra_lift_m=0.,lift_after_withdraw_m=None):
        start=self.robot.fk(qgrasp);body0=start@relative
        initial_extra=extra_lift_m if lift_after_withdraw_m is None else 0.
        liftbody=body0.copy();liftbody[2,3]+=max(.02+initial_extra,self.height+.1-(body0[2,3]-self.target.half_extents[2]))
        lifted=liftbody@np.linalg.inv(relative)
        poses=pose_line(start,lifted);labels=['separate']*len(poses)
        if lift_after_withdraw_m is not None:
            midbody=liftbody.copy();midbody[0,3]-=lift_after_withdraw_m
            middle=midbody@np.linalg.inv(relative);segment=pose_line(lifted,middle);poses+=segment[1:];labels+=['withdraw']*(len(segment)-1)
            midbody[2,3]+=extra_lift_m;raised=midbody@np.linalg.inv(relative)
            segment=pose_line(middle,raised);poses+=segment[1:];labels+=['separate']*(len(segment)-1)
            liftbody=midbody;lifted=raised
        # Upright straight extraction needs the actual X extent plus clearance.
        # Subsequent yaw/transit still undergoes the full payload collision check.
        clearbody=liftbody.copy();clearbody[0,3]=-(self.target.half_extents[0]+.015)
        cleared=clearbody@np.linalg.inv(relative);segment=pose_line(lifted,cleared);poses+=segment[1:];labels+=['withdraw']*(len(segment)-1)
        path=self.pointwise(poses,qgrasp,'extract',relative)
        return dict(body0=body0,clearbody=clearbody,path=path,labels=labels,lateral_shift_m=0.,separation_lift_m=float(liftbody[2,3]-body0[2,3]),lift_after_withdraw_m=lift_after_withdraw_m)

    def loaded_candidate(self,extraction,relative,face,release_y,yaw,route):
        body0=extraction['body0'];clearbody=extraction['clearbody'];path=extraction['path'];labels=list(extraction['labels'])
        goalbody=body0.copy();goalbody[:3,:3]=rotation_matrix_from_rpy(0,0,yaw)
        goalbody[:3,3]=[-self.c.settings['layout_gap_m']-.35,release_y,self.height+self.target.half_extents[2]+self.c.settings['face_run']['release_drop_m']]
        if route=='line':bodies=pose_line(clearbody,goalbody)
        elif route=='level_then_lower':
            # Keep the payload above the belt while the shoulder turns away
            # from it, then lower at the receiving region. One intermediate
            # pose, checked pointwise and shaped without a required stop.
            overhead=goalbody.copy();overhead[2,3]=max(clearbody[2,3],goalbody[2,3])
            bodies=pose_line(clearbody,overhead)+pose_line(overhead,goalbody)[1:]
        else:
            bodies=pose_line(clearbody,goalbody);center=self.c.world[:2,3]
            a=clearbody[:2,3]-center;b=goalbody[:2,3]-center;t0=np.arctan2(a[1],a[0]);delta=(np.arctan2(b[1],b[0])-t0+np.pi)%(2*np.pi)-np.pi
            for u,T in zip(np.linspace(0,1,len(bodies)),bodies):
                radius=(1-u)*np.linalg.norm(a)+u*np.linalg.norm(b);T[:2,3]=center+radius*np.array([np.cos(t0+u*delta),np.sin(t0+u*delta)])
        transfer=self.pointwise([b@np.linalg.inv(relative) for b in bodies],path[-1],'transfer',relative)
        path=path+transfer[1:];labels+=['transfer']*(len(transfer)-1);final=self.robot.fk(path[-1])@relative
        ext=np.abs(final[:3,:3])@self.target.half_extents;g=self.c.settings['layout_gap_m']
        if not(-g-.695<=final[0,3]-ext[0] and final[0,3]+ext[0]<=-g-.005 and abs(final[1,3])+ext[1]<=1.095):
            raise RuntimeError('RECEIVER_FOOTPRINT_OUTSIDE_BELT')
        drop=float(final[2,3]-ext[2]-self.height)
        if not 0<=drop<=self.c.settings['face_run']['maximum_release_drop_m']+1e-6:raise RuntimeError('RELEASE_DROP_TOO_HIGH')
        for z in np.linspace(final[2,3],self.height+ext[2],8):
            box=final.copy();box[2,3]=z
            if not self.valid(path[-1],'drop_corridor',box=box):raise RuntimeError('DROP_CORRIDOR_COLLISION')
        rows,law=self.shaped(path,'loaded_flow',relative)
        return dict(rows=rows,law=law,labels=labels,relative=relative,release_body=final,release_drop_m=drop,release_y_m=release_y,
            release_yaw_deg=float(np.rad2deg(yaw)),route=route,ik_knots=len(path),face=face,
            extraction_lateral_shift_m=extraction['lateral_shift_m'])

    def plan_departure(self,loaded):
        original=self.box.copy()
        try:
            q=loaded['rows'][-1]['q'];self.box=loaded['release_body'].copy()
            ext=np.abs(self.box[:3,:3])@self.target.half_extents;self.box[2,3]=self.height+ext[2]
            T=self.robot.fk(q);end=T.copy();end[:3,3]-=.12*end[:3,2]
            path=self.pointwise(pose_line(T,end),q,'exit');rows,law=self.shaped(path,'exit')
            park,park_law=self.joint_path(path[-1],self.home,'park')
            loaded.update(exit_rows=rows,exit_law=law,park_rows=park,park_law=park_law)
        finally:self.box=original

    def grasp_pose(self,face):
        T=np.eye(4);T[:3,3]=self.target.center
        if face=='front':
            T[:3,:3]=self.front;T[0,3]-=self.target.half_extents[0]
        elif face=='top':
            T[:3,:3]=np.diag([1.,-1.,-1.]);T[2,3]+=self.target.half_extents[2]
        elif face in ['left','right']:
            side=1 if face=='left' else -1
            # One explicit side business pose: same upright gripper, facing
            # inward, 150 mm behind the front carton edge. Full rings and all
            # neighbouring cartons are still checked on the actual geometry.
            T[:3,:3]=rotation_matrix_from_rpy(0,0,-side*np.pi/2)@self.front
            T[0,3]-=self.target.half_extents[0]-self.c.settings['face_run'].get('side_contact_from_front_m',.15)
            T[1,3]+=side*self.target.half_extents[1]
        else:raise ValueError('UNKNOWN_GRASP_FACE:'+face)
        return T

    def plan_carton(self):
        faces=['front','top'] if self.target.center[2]>.8 else ['top','front']
        if self.c.settings['face_run'].get('allow_side_suction',False):
            faces+=['left','right'] if self.target.center[1]<=0 else ['right','left']
        seeds=[self.q]+[simulation_coordinates(self.c.settings,np.deg2rad(q)) for q in
            [[10,-40,100,-240,-80,180],[160,-90,-80,-90,90,180],[160,-120,60,-190,90,200],[160,-80,-60,50,-90,20]]]
        for face in faces:
            T=self.grasp_pose(face)
            found=ik_candidates(self.c,T,seeds=seeds,draws=64,stage=self.target.name+'_'+face,limit=8,
                valid=lambda q:self.valid(q,'grasp_candidate'))
            print(self.target.name,face,'grasp solutions',len(found),flush=True)
            prepared=[]
            for qi,qgrasp in enumerate(sorted(found,key=lambda q:self.score(q,self.q))):
                try:
                    actual=self.robot.fk(qgrasp);pre=actual.copy();pre[:3,3]-=.12*pre[:3,2]
                    retreat=self.pointwise(pose_line(actual,pre),qgrasp,'pregrasp_reverse');approach=list(reversed(retreat))
                    relative=np.linalg.inv(actual)@self.target.world_from_local
                    geometry=evaluate_independent_cup_geometry(actual,self.target,face,self.cup_array)
                    if not any(geometry.geometrically_eligible_mask):raise RuntimeError('NO_FULL_TARGET_SEAL_RING')
                    approach_rows,approach_law=self.shaped(approach,'approach')
                    # Fixed target and straight withdrawal: neither a neighbouring
                    # carton nor an emptied adjacent bay may substitute for failure.
                    try:extraction=self.extraction(qgrasp,relative)
                    except RuntimeError as e:
                        self.failures.append(dict(kind='EXTRACTION_CANDIDATE_FAILED',target=self.target.name,face=face,branch=qi,
                            lateral_shift_m=0.,reason=str(e),detail=deepcopy(self.last_pointwise_failure)))
                        print(self.target.name,'branch',qi,str(e),flush=True);continue
                    preference=self.withdrawal_score(extraction['path'])
                    prepared.append(dict(branch=qi,extraction=extraction,relative=relative,geometry=geometry,
                        approach_rows=approach_rows,approach_law=approach_law,pregrasp=approach[0],preference=preference))
                    self.report.setdefault('withdrawal_candidates',[]).append(dict(target=self.target.name,face=face,branch=qi,**preference))
                    print(self.target.name,'branch',qi,'withdraw PASS',preference,flush=True)
                except RuntimeError as e:self.failures.append(dict(kind='GRASP_BRANCH_FAILED',target=self.target.name,face=face,branch=qi,reason=str(e)))
            # Rank actual same-target, straight-withdrawal solutions by the
            # user's J1 excursion and negative-J2 preferences before transfer.
            for candidate in sorted(prepared,key=lambda p:p['preference']['score']):
                qi=candidate['branch'];ys=list(dict.fromkeys([float(self.target.center[1]),0.,-.1,.1,-.2,.2,-.3,.3,-.5,.5,-.8,.8]))
                # This empty edge is identical for every receiver candidate of
                # this grasp branch. Check it once before the loaded suffix.
                try:travel,law=self.joint_path(self.q,candidate['pregrasp'],'pregrasp')
                except RuntimeError as e:
                    self.failures.append(dict(kind='PREGRASP_PATH_FAILED',target=self.target.name,face=face,branch=qi,reason=str(e)))
                    self.checkpoint();print(self.target.name,'branch',qi,str(e),flush=True);continue
                for yaw in [0.,np.pi/2,-np.pi/2]:
                    for y in ys:
                        if yaw and abs(y)>.79:continue
                        for route in self.c.settings['face_run'].get('transfer_routes',['line','arc']):
                            try:
                                loaded=self.loaded_candidate(candidate['extraction'],candidate['relative'],face,y,yaw,route)
                                self.report['current_checked_loaded_candidate']=dict(branch=qi,release_y_m=y,yaw_deg=float(np.rad2deg(yaw)),route=route,
                                    release_q_rad=loaded['rows'][-1]['q'].tolist(),ik_knots=loaded['ik_knots'],release_drop_m=loaded['release_drop_m'])
                                self.checkpoint()
                                print(self.target.name,'loaded path PASS, checking empty return',flush=True)
                                self.plan_departure(loaded)
                            except RuntimeError as e:
                                self.failures.append(dict(kind='LOADED_CANDIDATE_FAILED',target=self.target.name,face=face,branch=qi,
                                    release_y_m=y,yaw_deg=float(np.rad2deg(yaw)),route=route,reason=str(e)))
                                self.checkpoint()
                                print(self.target.name,'branch',qi,'release',y,'yaw',yaw,'route',route,str(e),flush=True);continue
                            loaded.update(travel=travel,travel_law=law,approach=candidate['approach_rows'],
                                approach_law=candidate['approach_law'],geometry=candidate['geometry'],
                                withdrawal_preference=candidate['preference'],selected_grasp_branch=qi)
                            return loaded
        raise RuntimeError('FINITE_CARTON_SEARCH_EXHAUSTED:'+self.target.name)

    def execute_carton(self,plan,rule):
        first=len(self.samples)
        self.emit(plan['travel'],plan['travel_law'],'pregrasp');self.emit(plan['approach'],plan['approach_law'],'approach')
        mask=list(plan['geometry'].geometrically_eligible_mask)
        self.cups=dict(mode='ideal_independent_cups',eligible=mask,commanded=list(mask),actual_geometric_contact=list(mask),
            commanded_count=sum(mask),cup_ids=[cup.cup_id for cup in plan['geometry'].cups],actual_contact_semantics='GEOMETRIC_NOT_PHYSICAL_SENSOR')
        self.relative=plan['relative'];self.attached=True;self.event('grasp',.3);self.mark('grasp',len(self.samples)-1,'FULL_RING_THEN_FIXED_RELATIVE_TRANSFORM')
        self.report['cups']=self.cups
        self.report['physical_contact_from_attached_box']=self.relative.tolist()
        self.emit(plan['rows'],plan['law'],'separate',plan['labels']);self.attached=False
        self.event('release',.2);self.mark('release',len(self.samples)-1,'CHECKED_LOW_DROP_RELEASE',drop_m=plan['release_drop_m'])
        begin=len(self.samples);start_z=self.box[2,3];ext=np.abs(self.box[:3,:3])@self.target.half_extents
        for u in np.linspace(0,1,8)[1:]:
            self.box[2,3]=(1-u)*start_z+u*(self.height+ext[2])
            if not self.valid(self.q,'drop'):raise RuntimeError('ACTUAL_DROP_COLLISION')
            self.event('drop',.06)
        self.mark('drop',begin,'KINEMATIC_ASSUMED_RECEPTION')
        self.emit(plan['exit_rows'],plan['exit_law'],'exit');self.emit(plan['park_rows'],plan['park_law'],'park')
        landed=OBB(self.box[:3,3],self.target.half_extents,self.box[:3,:3],self.target.name,'carton')
        world=coupled_conveyor_state(self.c.layout,self.height,()).boxes
        out=outfeed_path(landed,world,speed=self.c.settings['timing']['outfeed_velocity_m_s'],tool_timing=PARK_FIRST,finite_support=True)
        begin=len(self.samples);previous=0.;indices=list(range(4,len(out['samples']),4))
        if not indices or indices[-1]!=len(out['samples'])-1:indices.append(len(out['samples'])-1)
        for i in indices:
            step=out['samples'][i];self.box=message_box(step['box']).world_from_local
            if not self.valid(self.q,'outfeed',contact=False):raise RuntimeError('OUTFEED_COLLISION')
            self.event('outfeed',step['t']-previous);previous=step['t']
        self.mark('outfeed',begin,'ORIGINAL_ROUTING_KINEMATIC_SAMPLES_MAX_20MM')
        envelope=np.abs(self.box[:3,:3])@self.target.half_extents
        if self.box[0,3]+envelope[0]>-3.2+1e-9:raise RuntimeError('OUTFEED_ENVELOPE_NOT_CLEARED')
        # The POC handoff retires this exact body only after its entire envelope
        # has crossed the existing X=-3.2 m outfeed boundary.
        self.c.outfed_carton_ids=set(getattr(self.c,'outfed_carton_ids',()))|{self.target.name}
        self.completed.append(self.target.name);del self.active[self.target.name];self.c.active_cartons=tuple(self.active.values())
        self.event('carton_complete',.1)
        self.tasks.append(dict(target=self.target.name,status='COMPLETED',first_sample=first,last_sample=len(self.samples)-1,
            face=plan['face'],receiver_height_rule=rule,release_drop_m=plan['release_drop_m'],release_y_m=plan['release_y_m'],
            release_yaw_deg=plan['release_yaw_deg'],loaded_route=plan['route'],loaded_ik_knots=plan['ik_knots'],
            extraction_lateral_shift_m=plan['extraction_lateral_shift_m'],
            withdrawal_preference=plan.get('withdrawal_preference'),selected_grasp_branch=plan.get('selected_grasp_branch'),
            physical_contact_from_attached_box=self.relative.tolist(),cups=self.cups))
        self.checkpoint()

    def checkpoint(self):
        self.report.update(completed_cartons=len(self.completed),remaining_cartons=len(self.active),completed_carton_ids=self.completed,
            remaining_carton_ids=list(self.active),ik_attempts=self.c.attempts,collision_state_checks=self.previous_state_checks+self.checker.check_count,
            collision_narrow_pair_queries=self.previous_pair_queries+self.checker.pair_queries,elapsed_wall_s=self.previous_wall_s+perf_counter()-self.started,
            numerical_rng_state=deepcopy(self.c.rng.bit_generator.state),
            simulated_duration_s=self.samples[-1]['time_s'] if self.samples else 0.)
        save(self.c.output/'cycle.json',self.report)

    def run(self,maximum_cartons=None):
        try:
            graph=SupportRelationGraph.build(tuple(self.c.layout.cartons()))
            self.report.setdefault('preferred_order',graph.removal_order(('front','top')));self.report['target']=self.report['preferred_order'][0]
            self.report['target_order_policy']='FIXED_CENTRE_FIRST_STOP_ON_TARGET_FAILURE_NO_NEIGHBOUR_RECOVERY'
            while self.active and (maximum_cartons is None or len(self.completed)<maximum_cartons):
                name=next(n for n in self.report['preferred_order'] if n in self.active)
                if name not in graph.removable_cartons(self.active,('front','top')):
                    raise RuntimeError('FIXED_TARGET_STILL_SUPPORTED:'+name)
                self.target=self.active[name];self.box=self.target.world_from_local.copy();self.relative=None;self.cups=None
                self.c.settings['target']=name
                if self.q is None:self.select_home()
                else:self.event('task_start',.001)
                self.report['current_target']=name
                self.checkpoint()
                rule=receiving_height(self.box,self.target.half_extents,**self.c.settings['receiving_height'])
                self.change_height(rule['surface_height_m'])
                plan=self.plan_carton()
                self.execute_carton(plan,rule)
            if not self.active:
                self.change_height(.6);self.event('return',.1)
                self.mark('return',len(self.samples)-1,'HOME_ALREADY_REACHED_BY_CHECKED_EMPTY_RETURN_PATH')
                self.event('complete',.1);self.report['status']='COMPLETED_KINEMATIC_FULL_FACE'
            elif maximum_cartons==1 and len(self.completed)==1:
                self.change_height(.6);self.event('return',.1)
                self.mark('return',len(self.samples)-1,'HOME_ALREADY_REACHED_BY_CHECKED_EMPTY_RETURN_PATH')
                self.event('complete',.1);self.report['status']='COMPLETED_KINEMATIC_SINGLE_CARTON'
                self.report['task_scope']='REQUESTED_SINGLE_CARTON_IN_COMPLETE_40_CARTON_INITIAL_SCENE'
            else:self.report['status']='COMPLETED_REQUESTED_PREFIX_NOT_FULL_FACE'
        except Exception as e:
            self.report.update(status='KINEMATIC_FACE_ATTEMPT_FAILED',failure=dict(type=type(e).__name__,reason=str(e),
                target=None if self.target is None else self.target.name,last_collision=self.checker.last))
            print('FACE FAILED',str(e),flush=True)
        self.checkpoint();return self.report
