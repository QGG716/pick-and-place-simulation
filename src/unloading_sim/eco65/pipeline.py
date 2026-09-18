"""Known-pose -> real IK/RRT -> authoritative geometry validation -> simulated execution."""
from __future__ import annotations
import json,time,hashlib,subprocess,copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from ..ik import solve_ik
from ..planner import RRTConnectPlanner
from ..timing import JointMotionLimits,time_parameterize_joint_path,sample_quintic_knots
from .model import *
from . import planning_contracts as P
from . import execution_contracts as E

def independent_fk(robot,q):
    # Separate SciPy quaternion/axis-angle implementation, direct XML chain; no robot.fk call.
    T=robot.base_transform.copy()
    root=ET.parse(URDF).getroot()
    for node,angle in zip(root.findall("joint"),q):
        origin=node.find("origin");xyz=np.fromstring(origin.attrib["xyz"],sep=" ");rpy=np.fromstring(origin.attrib["rpy"],sep=" ")
        axis=np.fromstring(node.find("axis").attrib["xyz"],sep=" ")
        A=np.eye(4);A[:3,:3]=Rotation.from_euler("xyz",rpy).as_matrix();A[:3,3]=xyz
        B=np.eye(4);B[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix();T=T@A@B
    return T@robot.tip_from_tcp

def numeric_checks(world):
    r=world.robot; rng=np.random.default_rng(12);errors=[]
    for q in [np.zeros(6),np.array([.2,.6,-.8,.4,-.7,.2]),rng.uniform(r.joint_limits[:,0],r.joint_limits[:,1])]:
        actual=r.fk(q);ind=independent_fk(r,q);err=float(np.max(np.abs(actual-ind)))
        jac=r.geometric_jacobian(q);fd=np.zeros_like(jac);eps=1e-6
        for j in range(6):
            dq=np.eye(6)[j]*eps;a=independent_fk(r,q+dq);b=independent_fk(r,q-dq)
            fd[:3,j]=(a[:3,3]-b[:3,3])/(2*eps)
            fd[3:,j]=Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).as_rotvec()/(2*eps)
        jerr=float(np.max(np.abs(jac-fd)));assert err<1e-10 and jerr<2e-7
        errors.append(dict(q=q.tolist(),fk_max_error=err,jacobian_fd_max_error=jerr))
    assert not r.within_limits(r.joint_limits[:,1]+.01)
    for k in ("T_flange_tcp","T_flange_tool_cad","T_tool_cad_tcp"):check_transform(world.tool[k])
    # Geometric render/collision mesh bodies must consume the exact same official link frames.
    world.set_state(np.zeros(6),"initial")
    for name,T in r.named_link_frames(np.zeros(6)).items():
        bid=world.model.body(name).id
        assert np.allclose(world.data.xpos[bid],T[:3,3],atol=1e-9)
        assert np.allclose(world.data.xmat[bid].reshape(3,3),T[:3,:3],atol=1e-9)
    return dict(status="PASS",independent_method="SciPy XML FK + central differences",states=errors,zero_pose="numerical test only, not HOME")

class IKSearchExhausted(RuntimeError):
    def __init__(self,phase,found,collision):
        self.found=found;self.collision=collision
        super().__init__("IK_COLLISION_CANDIDATES_EXHAUSTED " + phase if found else "IK_NOT_FOUND_WITHIN_BUDGET " + phase)

def find_ik(world,target,phase,seeds,rng,deadline,cancel=lambda:False):
    found=0;collision=None
    for index,seed in enumerate(list(seeds)+[rng.uniform(world.robot.joint_limits[:,0],world.robot.joint_limits[:,1]) for _ in range(world.scene["ik_seeds"])]):
        if cancel():raise InterruptedError("CANCELLED")
        if time.perf_counter()>deadline:raise TimeoutError("BUDGET_EXHAUSTED")
        r=solve_ik(world.robot,target,seed,max_iterations=350,position_tolerance=.00002,orientation_tolerance=.0002,
                   deadline_monotonic=deadline)
        if r.success:found+=1
        if r.success and world.valid(r.q,phase):
            return r.q,dict(seed_index=index,iterations=r.iterations,position_error_m=r.position_error,orientation_error_rad=r.orientation_error)
        if r.success:collision=copy.deepcopy(world.last_failure)
        if index and index%20==0:print("IK",phase,"seeds",index,"last",world.last_failure,flush=True)
    raise IKSearchExhausted(phase,found,collision)

def known_world(world,q):
    scene=world.scene
    pose=Pose3D(tuple(scene["box"]["pose"]),(0.,0.,0.,1.),"world",scene["axis_convention"],EvidenceKind.SYNTHETIC)
    fixture=dict(provider="known_pose_fixture",pose_m=list(pose.position_m),orientation_xyzw=list(pose.orientation_xyzw),
                 evidence=pose.evidence.value,object_id=scene["box"]["id"],detection_score=None,calibration=None)
    snapshot=copy.deepcopy(scene);snapshot["known_observation"]=fixture
    model_id=hashlib.sha256(URDF.read_bytes()).hexdigest();tool_id=canonical_fingerprint(world.tool)
    config=dict(robot_model_fingerprint=model_id,world_model_fingerprint=canonical_fingerprint(snapshot),tool_fingerprint=tool_id)
    revision=P.SceneRevision(1,P.scene_fingerprint(snapshot),"known_pose_fixture")
    return P.PlanningWorldSnapshot(revision,snapshot,P.RobotStateRevision(1,tuple(q),{"joint_names":world.robot.active_joint_names}),
        {"tool_fingerprint":tool_id},None,{"fixed":True,"position_m":scene["base_position_m"]},{"installed":False},config)

class ECO65PlannerBackend(P.PlannerBackend):
    def __init__(self,world):
        self.world=world;self.cancelled=False
        super().__init__(seed=world.scene["seed"],identity=P.BackendIdentity("eco65-ik-rrt","1","1",hashlib.sha256(URDF.read_bytes()).hexdigest(),canonical_fingerprint(world.scene)),
            capabilities=P.BackendCapabilities(frozenset({P.PlanningPath.COLD}),frozenset({P.BoundaryMode.STOP_BOUNDARY}),
                frozenset({P.PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY}),supports_deterministic_seed=True,supports_attached_object=True,supports_revalidation=True,supports_logical_cancel=True))
    def logical_cancel(self,request_id):self.cancelled=True
    def plan(self,request,candidate,planning_path):
        start=time.perf_counter();w=self.world;s=w.scene;rng=np.random.default_rng(request.seed)
        deadline=start+s["planning_budget_s"];q=np.array(request.start_state);segments=[];box=s["box"];top=box["pose"][2]+box["size_m"][2]/2
        R=np.diag([1.,-1.,-1.]);clearance=box["size_m"][2]+2*s["margin_m"] # support/payload dimension-derived lift
        pick=transform([*box["pose"][:2],top],R)
        approach=pick.copy();approach[2,3]+=.035
        lift=pick.copy();lift[2,3]+=clearance
        limits=JointMotionLimits(np.full(6,s["joint_velocity_rad_s"]),np.full(6,s["joint_acceleration_rad_s2"]),np.full(6,s["joint_jerk_rad_s3"]),source="desktop simulation finite design limits; below official velocities")
        def add(phase,target):
            nonlocal q
            goal,ik=find_ik(w,target,phase,[q],rng,deadline,lambda:self.cancelled)
            def valid(state):
                if self.cancelled:raise InterruptedError("CANCELLED")
                if time.perf_counter()>deadline:raise TimeoutError("BUDGET_EXHAUSTED")
                return w.valid(state,phase)
            planner=RRTConnectPlanner(w.robot.joint_limits[:,0],w.robot.joint_limits[:,1],valid,edge_resolution=.01,
                max_iterations=s["rrt_iterations"],rng=rng)
            path=planner.plan(q,goal,time_limit_seconds=max(0.,deadline-time.perf_counter()))
            if not path.success:raise RuntimeError(phase+": "+path.message+" "+str(w.last_failure))
            speed=limits if phase not in ("contact","place_contact") else JointMotionLimits(np.full(6,.10),np.full(6,.3),np.full(6,1.),source="slow contact design limits")
            timed=time_parameterize_joint_path(path.path,speed)
            seg=dict(phase=phase,q=timed.positions.tolist(),t=timed.time_from_start.tolist(),duration_s=timed.duration_seconds,
                target_tcp=target.tolist(),ik=ik,search=path.search_evidence,timing=timed.audit(speed))
            segments.append(seg);q=goal
            print("PLANNED",phase,"waypoints",len(path.path),"duration",round(timed.duration_seconds,2),flush=True)
        try:
            add("approach",approach);add("contact",pick)
            relative=w.attach(q)
            add("loaded_lift",lift)
            destination=transform(box["receiver_pose"])
            place=destination@np.linalg.inv(relative);preplace=place.copy();preplace[2,3]+=clearance
            add("loaded_transfer",preplace);add("place_contact",place)
            released=w.release(q)
            retreat=place.copy();retreat[2,3]+=max(.035,.5*box["size_m"][2])
            add("retreat",retreat)
        except TimeoutError:
            return P.PlanningResult.failed(status=P.PlanStatus.TIMEOUT,candidate=candidate,message="BUDGET_EXHAUSTED",metadata={"segments_completed":segments})
        except InterruptedError:
            return P.PlanningResult.operational_failure(outcome=P.OperationalOutcome.CANCELLED,candidate=candidate,failure=P.FailureDetails(True,P.FailureScope.BACKEND_LOCAL,native_code="CANCELLED"))
        except IKSearchExhausted as exc:
            return P.PlanningResult.failed(status=P.PlanStatus.COLLISION if exc.found else P.PlanStatus.NO_IK,candidate=candidate,message=str(exc),metadata={"collision":exc.collision,"bounded_search":True})
        except RuntimeError as exc:
            return P.PlanningResult.failed(status=P.PlanStatus.NOT_EVALUATED,candidate=candidate,message="PATH_SEARCH_EXHAUSTED: "+str(exc),metadata={"segments_completed":segments,"last_collision":w.last_failure})
        payload=dict(segments=segments,attachment_tcp_to_box=relative.tolist(),released_box_pose=released.tolist(),
            box_id=box["id"],source_world=request.world_snapshot.fingerprint,tool_fingerprint=canonical_fingerprint(w.tool),
            geometry_mode="GEOMETRIC_REPLAY_ONLY",dynamics="NOT_EVALUATED",lift_distance_m=clearance)
        from .task_checks import runtime_fingerprint
        payload["runtime_fingerprint"]=runtime_fingerprint(w)
        payload["trajectory_fingerprint"]=canonical_fingerprint(payload)
        self.artifact=payload
        trajectory=[request.start_state]+[tuple(row) for seg in segments for row in seg["q"][1:]]
        return P.PlanningResult.succeeded(candidate,trajectory,latency_seconds=time.perf_counter()-start,
            artifact_kind=P.PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY,metadata=payload,
            expected_start_boundary=P.MotionBoundaryState.stopped(request.start_state),expected_end_boundary=P.MotionBoundaryState.stopped(tuple(q)))

def artifact_dict(result):
    # Local online _freeze makes mappings read-only; canonicalize for JSON/replay.
    return json.loads(json.dumps(result.metadata,default=lambda x:dict(x)))

class ECO65PlanValidator(P.PlanValidator):
    def __init__(self,world,output=None):
        super().__init__("eco65-complete-geometry","2");self.world=world;self.report={};self.output=output
    def validate(self,plan_envelope,current_world_snapshot,current_motion_boundary):
        from .task_checks import audit_task,runtime_fingerprint,current_kinematics_match_source,measure_motion_envelope
        w=self.world;plan=plan_envelope;errors=[];count=0;pose_errors=[];events=[]
        artifact=artifact_dict(plan.result)
        try:
            actual_snapshot=known_world(w,w.current_q)
            if not plan.planned_snapshot.planning_context_matches(current_world_snapshot) or not plan.planned_snapshot.planning_context_matches(actual_snapshot):errors.append('actual_world_identity')
            if not np.allclose(w.current_q,current_motion_boundary.q,atol=1e-9,rtol=0):errors.append('actual_start_state')
            if not current_motion_boundary.matches(plan.expected_start_boundary):errors.append('start_boundary')
            if plan.expected_start_boundary.boundary_mode!=P.BoundaryMode.STOP_BOUNDARY:errors.append('start_not_stopped')
            if plan.expected_end_boundary.boundary_mode!=P.BoundaryMode.STOP_BOUNDARY:errors.append('end_not_stopped')
            if artifact.get('tool_fingerprint')!=canonical_fingerprint(w.tool):errors.append('tool_identity')
            if artifact.get('runtime_fingerprint')!=runtime_fingerprint(w) or not current_kinematics_match_source(w):errors.append('actual_model_identity')
            if artifact.get('source_world')!=plan.planned_snapshot.fingerprint:errors.append('source_world')
            if plan.result.target_id!=w.box_id or plan.candidate.target_id!=w.box_id:errors.append('target_identity')
            velocity=[float(j.find('limit').attrib['velocity']) for j in ET.parse(URDF).getroot().findall('joint') if j.find('limit') is not None]
            audit=audit_task(artifact,plan.result.trajectory,plan.expected_start_state,plan.expected_end_state,w.box_id,w.robot.joint_limits,w.scene,velocity)
            errors.extend(audit['errors'])
            if getattr(w,'attached',False) or w.released_pose is not None:errors.append('world_not_initial')
            if errors:raise ValueError('preflight_failed')
            if not w.valid(np.asarray(plan.expected_start_state),'initial'):raise ValueError('initial_clearance '+str(w.last_failure))
            attached=False;released=False
            for seg in artifact['segments']:
                phase=seg['phase'];knots=np.array(seg['q']);times=np.array(seg['t']);queries=[]
                if phase.startswith('loaded') or phase in ('front_separation','place_contact'):
                    if not attached:raise ValueError('loaded_without_attach')
                if phase=='retreat' and not released:raise ValueError('retreat_without_supported_release')
                for a,b,ta,tb in zip(knots[:-1],knots[1:],times[:-1],times[1:]):
                    n=max(2,int(np.ceil(np.sum(np.abs(b-a))/.0008)),int(np.ceil((tb-ta)/.02)))
                    queries.extend(np.linspace(ta,tb,n+1))
                qs,_=sample_quintic_knots(times,knots,np.array(queries))
                for row in qs:
                    if not w.valid(row,phase):raise ValueError('collision '+str(w.last_failure))
                    measure_motion_envelope(w);count+=1
                    if count%1000==0:print('VALIDATION_PROGRESS',phase,'samples',count,flush=True)
                target=np.array(seg['target_tcp']);actual=w.robot.fk(knots[-1])
                pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));oe=float(Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).magnitude())
                pose_errors.append(dict(phase=phase,position_error_m=pe,orientation_error_rad=oe))
                if pe>.00005 or oe>.0003:raise ValueError('FK_residual')
                if phase=='contact':
                    relative=w.attach(knots[-1]);attached=True;events.append('ATTACHED')
                    if not np.allclose(relative,artifact['attachment_tcp_to_box'],atol=1e-8,rtol=0):raise ValueError('attachment_mismatch')
                if phase=='place_contact':
                    pose=w.release(knots[-1]);attached=False;released=True;events.append('SUPPORTED_RELEASE')
                    if not np.allclose(pose,artifact['released_box_pose'],atol=1e-8,rtol=0):raise ValueError('release_mismatch')
                if phase=='retreat':events.append('RETREAT_COMPLETE')
                print('VALIDATED',phase,'samples',count,flush=True)
            if events!=['ATTACHED','SUPPORTED_RELEASE','RETREAT_COMPLETE']:errors.append('incomplete_task')
        except (KeyError,ValueError,TypeError,IndexError) as exc:
            if str(exc)!='preflight_failed':errors.append(str(exc))
        self.report=dict(status='VALID' if not errors else 'INVALID',samples=count,errors=errors,pose_errors=pose_errors,events=events,
            trajectory_fingerprint=artifact.get('trajectory_fingerprint'),world_fingerprint=current_world_snapshot.fingerprint,
            timing_audit=locals().get('audit'),motion_envelope=getattr(w,'motion_envelope',None),joint_l1_sampling_rad=.0008,time_sampling_s=.02,continuous_collision_certification=False,
            collision_margin_m=w.scene['margin_m'],mesh_error_reserve_m=w.scene['mesh_error_reserve_m'],scope='GEOMETRIC_REPLAY_ONLY')
        if self.output:save_json(self.output,self.report)
        return P.PlanValidationResult(P.ValidationStatus.VALID if not errors else P.ValidationStatus.INVALID,current_world_snapshot,current_motion_boundary,
            'complete task '+self.report['status'],self.name,self.version)

class GeometricExecutionBackend(E.ExecutionBackend):
    """Stoppable, stepped geometric executor; no dynamics or hardware claims."""
    def __init__(self,world,enable_hardware=False):
        require_simulation({'enable_hardware':enable_hardware});self.w=world;self._state=E.ExecutionBackendState.IDLE
        self._boundary=None;self.feedback=[];self.events=[];self.frames=[];self.execution_id=None;self.sequence=0
    @property
    def identity(self):return E.ExecutionBackendIdentity('eco65-geometric-simulation','2','2')
    @property
    def capabilities(self):return E.ExecutionBackendCapabilities(frozenset({P.PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY}),frozenset({P.BoundaryMode.STOP_BOUNDARY}),True,True,True,True,True,True,True,False)
    @property
    def health(self):
        return {E.ExecutionBackendState.FAULTED:E.ExecutionBackendHealth.FAULTED,E.ExecutionBackendState.SHUTDOWN:E.ExecutionBackendHealth.SHUTDOWN}.get(self._state,E.ExecutionBackendHealth.READY)
    @property
    def state(self):return self._state
    def current_boundary(self):return self._boundary
    def _command(self,status,plan_id,message):
        import uuid
        return E.ExecutionCommandResult(status,uuid.uuid4().hex,self.execution_id,plan_id,message)
    def start(self,plan_envelope):
        import uuid
        from .task_checks import audit_task,runtime_fingerprint,current_kinematics_match_source,measure_motion_envelope
        require_simulation(self.w.scene)
        if self._state in (E.ExecutionBackendState.RUNNING,E.ExecutionBackendState.STOPPING):return self._command(E.ExecutionCommandStatus.ALREADY_RUNNING,self.plan.plan_id,'Execution already active')
        if self._state!=E.ExecutionBackendState.IDLE:return self._command(E.ExecutionCommandStatus.BACKEND_UNAVAILABLE,plan_envelope.plan_id,'Backend not idle')
        if not plan_envelope.executable:raise ValueError('Unvalidated envelope')
        if not self.capabilities.supports(plan_envelope):raise ValueError('Unsupported motion boundary or artifact')
        w=self.w;artifact=artifact_dict(plan_envelope.result)
        if not np.allclose(w.current_q,plan_envelope.expected_start_state,atol=1e-9,rtol=0):raise ValueError('Actual start mismatch')
        if not plan_envelope.planned_snapshot.planning_context_matches(known_world(w,w.current_q)):raise ValueError('Actual world mismatch')
        if artifact.get('runtime_fingerprint')!=runtime_fingerprint(w) or not current_kinematics_match_source(w):raise ValueError('Actual model mismatch')
        velocity=[float(j.find('limit').attrib['velocity']) for j in ET.parse(URDF).getroot().findall('joint') if j.find('limit') is not None]
        audit=audit_task(artifact,plan_envelope.result.trajectory,plan_envelope.expected_start_state,plan_envelope.expected_end_state,w.box_id,w.robot.joint_limits,w.scene,velocity)
        if not audit['valid']:raise ValueError(str(audit['errors']))
        if getattr(w,'attached',False) or w.released_pose is not None:raise ValueError('Execution world must be initial')
        if not w.valid(w.current_q,'initial'):raise ValueError('Initial geometry rejected '+str(w.last_failure))
        self.plan=plan_envelope;self.artifact=artifact;self.execution_id=uuid.uuid4().hex;w.execution_id=self.execution_id
        self.elapsed=0.;self.local_time=0.;self.segment_index=0;self.events=[];self.frames=[];self.feedback=[];self.sequence=0;self.next_frame=.2
        self.duration=sum(s['duration_s'] for s in artifact['segments']);self._state=E.ExecutionBackendState.RUNNING
        self._boundary=P.MotionBoundaryState.stopped(w.current_q);self._record('initial')
        return self._command(E.ExecutionCommandStatus.ACCEPTED,self.plan.plan_id,'GEOMETRIC_REPLAY_ONLY')
    def _record(self,phase):
        self.frames.append(dict(t=self.elapsed,q=list(self._boundary.q),qd=list(self._boundary.qd),phase=phase,box_id=self.w.box_id,
            box_pose=self.w.box_pose(self._boundary.q,phase).tolist(),box_state=getattr(self.w,'box_states',{}).get(self.w.box_id,'GEOMETRIC')))
    def _feedback(self,status,message=''):
        self.sequence+=1
        value=E.ExecutionFeedback(self.sequence,self.execution_id,self.plan.plan_id,status,min(1.,self.elapsed/self.duration),self._boundary,message,
            metadata=dict(simulation_time_s=self.elapsed,box_id=self.w.box_id,box_pose=self.w.box_pose(self._boundary.q,self.w.phase if hasattr(self.w,'phase') else 'retreat').tolist()))
        self.feedback.append(value)
        # Feedback is a latest-state stream; keep terminal events and bounded recent states.
        if len(self.feedback)>32:self.feedback.pop(0)
    def request_stop(self,plan_id,reason):
        if self._state!=E.ExecutionBackendState.RUNNING or plan_id!=self.plan.plan_id:return self._command(E.ExecutionCommandStatus.NOT_RUNNING,plan_id,'No matching active plan')
        self._state=E.ExecutionBackendState.STOPPING
        return self._command(E.ExecutionCommandStatus.ACCEPTED,plan_id,reason)
    def poll(self):return self.feedback.pop(0) if self.feedback else None
    def step(self,dt=.02):
        if not np.isfinite(dt) or dt<=0 or dt>.02+1e-12:raise ValueError('Geometric control step must be in (0,0.02] s')
        if self._state==E.ExecutionBackendState.STOPPING:
            self._boundary=P.MotionBoundaryState.stopped(self._boundary.q,time_seconds=self.elapsed)
            self._state=E.ExecutionBackendState.IDLE;self._feedback(E.ExecutionFeedbackStatus.STOPPED,'Geometric simulation stop completed; no controller claim');return False
        if self._state!=E.ExecutionBackendState.RUNNING:return False
        try:
            seg=self.artifact['segments'][self.segment_index];phase=seg['phase']
            if self.next_frame>self.elapsed+1e-10:dt=min(dt,self.next_frame-self.elapsed)
            new=min(seg['duration_s'],self.local_time+dt)
            q,v=sample_quintic_knots(seg['t'],seg['q'],np.array([new]));q=q[0];v=v[0]
            ts=np.array(seg['t']);knots=np.array(seg['q']);i=min(max(0,np.searchsorted(ts,new,side='right')-1),len(ts)-2)
            h=ts[i+1]-ts[i];u=(new-ts[i])/h;acc=(60*u-180*u*u+120*u**3)/h**2*(knots[i+1]-knots[i])
            if not self.w.valid(q,phase):raise ValueError('Execution geometry: '+str(self.w.last_failure))
            self.elapsed+=new-self.local_time;self.local_time=new
            stopped=bool(np.max(np.abs(v))<1e-9 and np.max(np.abs(acc))<1e-9)
            self._boundary=P.MotionBoundaryState(tuple(q),tuple(v),tuple(acc),self.elapsed,P.BoundaryMode.STOP_BOUNDARY if stopped else P.BoundaryMode.CONTINUOUS_BOUNDARY)
            if self.elapsed+1e-9>=self.next_frame:
                self._record(phase);self.next_frame+=.2
            if new>=seg['duration_s']-1e-10:
                if phase=='contact':
                    relative=self.w.attach(q)
                    if not np.allclose(relative,self.artifact['attachment_tcp_to_box'],atol=1e-8,rtol=0):raise ValueError('Actual attachment mismatch')
                    self.events.append(dict(event='ATTACHED',box_id=self.w.box_id,time_s=self.elapsed,q=q.tolist(),relative_transform=relative.tolist()))
                if phase=='place_contact':
                    pose=self.w.release(q);self.events.append(dict(event='SUPPORTED_RELEASE',box_id=self.w.box_id,time_s=self.elapsed,q=q.tolist(),pose=pose.tolist(),support=getattr(self.w,'release_support',None)))
                if phase=='retreat':self.events.append(dict(event='RETREAT_COMPLETE',box_id=self.w.box_id,time_s=self.elapsed,q=q.tolist()))
                print('EXECUTED',phase,'actual_time_s',round(self.elapsed,3),flush=True)
                self.segment_index+=1;self.local_time=0.
                if self.segment_index==len(self.artifact['segments']):
                    if [e['event'] for e in self.events]!=['ATTACHED','SUPPORTED_RELEASE','RETREAT_COMPLETE']:raise ValueError('Incomplete executed task')
                    self.elapsed=self.duration;self._state=E.ExecutionBackendState.IDLE;self._record(phase);self._feedback(E.ExecutionFeedbackStatus.SUCCEEDED,'GEOMETRIC_PICK_PLACE_COMPLETE');return False
            self._feedback(E.ExecutionFeedbackStatus.RUNNING)
            return True
        except Exception as exc:
            self._state=E.ExecutionBackendState.FAULTED
            try:
                if hasattr(self.w,'set_state'):self.w.set_state(self._boundary.q,getattr(self.w,'phase','initial'))
            except Exception as restore_error:
                self.restore_error=str(restore_error)
            self._feedback(E.ExecutionFeedbackStatus.FAULTED,str(exc));raise
    def advance(self,max_steps=1):
        done=0
        for _ in range(max_steps):
            if not self.step():break
            done+=1
        return done
    def run(self):
        while self._state in (E.ExecutionBackendState.RUNNING,E.ExecutionBackendState.STOPPING):self.step()
        complete=[e['event'] for e in self.events]==['ATTACHED','SUPPORTED_RELEASE','RETREAT_COMPLETE'] and self._state==E.ExecutionBackendState.IDLE
        return dict(status='GEOMETRIC_PICK_PLACE_COMPLETE' if complete else 'INCOMPLETE',geometry_mode='GEOMETRIC_REPLAY_ONLY',execution_id=self.execution_id,
            geometric_tasks_completed=int(complete),dynamics='NOT_EVALUATED',hardware_connected=False,trajectory_fingerprint=self.artifact['trajectory_fingerprint'],
            duration_s=self.elapsed,events=self.events,frames=self.frames,supports_continuous_handoff=False)
    def shutdown(self):self._state=E.ExecutionBackendState.SHUTDOWN


def plan_and_execute():
    cad=asset_check();tool=load_json(LOCAL/"tool.json");scene=load_json(LOCAL/"scene.json") if (LOCAL/"scene.json").exists() else design_scene(tool)
    require_simulation(scene);w=World(tool,scene)
    save_json(OUTPUT/"reports/numerics.json",numeric_checks(w))
    rng=np.random.default_rng(scene["seed"]);home_target=transform([.06,-.14,.98],np.diag([1.,-1.,-1.]))
    home,evidence=find_ik(w,home_target,"initial",[],rng,time.perf_counter()+scene["planning_budget_s"])
    save_json(OUTPUT/"reports/home.json",dict(q=home.tolist(),tcp=home_target.tolist(),evidence=evidence))
    snapshot=known_world(w,home);candidate=P.PlanningCandidate(scene["box"]["id"],"top_all_seals")
    request=P.PlanningRequest("desktop_1",snapshot,(candidate,),seed=scene["seed"])
    planner=ECO65PlannerBackend(w);result=planner.plan(request,candidate,P.PlanningPath.COLD)
    if not result.success:raise RuntimeError(result.message)
    envelope=P.PlanEnvelope("eco65_single_box",request,candidate,P.PlanningPath.COLD,result,snapshot,None,
        result.expected_start_boundary,result.expected_end_boundary,None,0,planner.provenance,result.artifact_kind,
        planner.provenance.robot_model_fingerprint,planner.provenance.world_model_fingerprint)
    save_json(OUTPUT/"scene_snapshot.json",dict(scene=scene,tool=tool,world_fingerprint=snapshot.fingerprint,home_q=home.tolist()))
    save_json(OUTPUT/"trajectory/plan.json",planner.artifact)
    validation_world=World(copy.deepcopy(tool),copy.deepcopy(scene));validation_world.set_state(home,"initial")
    validator=ECO65PlanValidator(validation_world,OUTPUT/"reports/validation.json");validation=validator.validate(envelope,snapshot,request.motion_boundary)
    validation_world.close()
    if not validation.valid:raise RuntimeError(validation.message)
    envelope=envelope.revalidated(validation,timestamp_seconds=time.monotonic(),generation=0)
    w.close();w=World(copy.deepcopy(tool),copy.deepcopy(scene));w.set_state(home,"initial")
    execution=GeometricExecutionBackend(w);execution.start(envelope);replay=execution.run()
    save_json(OUTPUT/"replay/states.json",replay)
    print("GEOMETRIC EXECUTION COMPLETE",replay["duration_s"],"seconds",len(replay["frames"]),"video states",flush=True)
    return w,home,planner.artifact,replay,envelope
