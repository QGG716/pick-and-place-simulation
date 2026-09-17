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

def find_ik(world,target,phase,seeds,rng,deadline,cancel=lambda:False):
    for index,seed in enumerate(list(seeds)+[rng.uniform(world.robot.joint_limits[:,0],world.robot.joint_limits[:,1]) for _ in range(world.scene["ik_seeds"])]):
        if cancel():raise InterruptedError("CANCELLED")
        if time.perf_counter()>deadline:raise TimeoutError("BUDGET_EXHAUSTED")
        r=solve_ik(world.robot,target,seed,max_iterations=350,position_tolerance=.00002,orientation_tolerance=.0002,
                   deadline_monotonic=deadline)
        if r.success and world.valid(r.q,phase):
            return r.q,dict(seed_index=index,iterations=r.iterations,position_error_m=r.position_error,orientation_error_rad=r.orientation_error)
        if index and index%20==0:print("IK",phase,"seeds",index,"last",world.last_failure,flush=True)
    raise RuntimeError("CANDIDATES_EXHAUSTED "+phase+" "+str(world.last_failure))

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
            return P.PlanningResult.failed(candidate,P.PlanStatus.TIMEOUT,message="BUDGET_EXHAUSTED",metadata={"segments_completed":segments})
        except InterruptedError:
            return P.PlanningResult.failed(candidate,P.PlanStatus.NOT_EVALUATED,message="CANCELLED")
        except RuntimeError as exc:
            save_json(OUTPUT/"reports/planning_failure.json",dict(message=str(exc),last_collision=w.last_failure,segments=segments))
            return P.PlanningResult.failed(candidate,P.PlanStatus.COLLISION,message=str(exc))
        payload=dict(segments=segments,attachment_tcp_to_box=relative.tolist(),released_box_pose=released.tolist(),
            box_id=box["id"],source_world=request.world_snapshot.fingerprint,tool_fingerprint=canonical_fingerprint(w.tool),
            geometry_mode="GEOMETRIC_REPLAY_ONLY",dynamics="NOT_EVALUATED",lift_distance_m=clearance)
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
    def __init__(self,world):super().__init__("eco65-complete-geometry","1");self.world=world;self.report={}
    def validate(self,plan_envelope,current_world_snapshot,current_motion_boundary):
        w=self.world;plan=plan_envelope;errors=[];artifact=artifact_dict(plan.result)
        if not plan.planned_snapshot.planning_context_matches(current_world_snapshot):errors.append("world_identity")
        if not np.allclose(current_motion_boundary.q,plan.expected_start_state,atol=1e-9):errors.append("start_state")
        if artifact["tool_fingerprint"]!=canonical_fingerprint(w.tool):errors.append("tool_identity")
        claimed=artifact.pop("trajectory_fingerprint")
        if claimed!=canonical_fingerprint(artifact):errors.append("artifact_hash")
        artifact["trajectory_fingerprint"]=claimed
        if errors:return P.PlanValidationResult(P.ValidationStatus.INVALID,current_world_snapshot,current_motion_boundary,",".join(errors),self.name,self.version)
        w.payload_relative=None;w.released_pose=None;count=0;pose_errors=[]
        previous=np.array(plan.expected_start_state)
        for seg in artifact["segments"]:
            phase=seg["phase"];knots=np.array(seg["q"]);times=np.array(seg["t"])
            if not np.allclose(previous,knots[0],atol=1e-9):errors.append("segment_discontinuity")
            if not np.isfinite(times).all() or np.any(np.diff(times)<=0):errors.append("timestamps")
            queries=[]
            # Same monotone quintic interpolation as playback, refined in joint-space L1 and time.
            # The nominal 0.0008-rad L1 subdivision is uniform in time; quintic peak spacing can be 1.875x larger.
            # This is dense sampling, not a certified continuous swept-volume bound.
            for a,b,ta,tb in zip(knots[:-1],knots[1:],times[:-1],times[1:]):
                n=max(2,int(np.ceil(np.sum(np.abs(b-a))/.0008)),int(np.ceil((tb-ta)/.02)))
                queries.extend(np.linspace(ta,tb,n+1))
            qs,vel=sample_quintic_knots(times,knots,np.array(queries))
            for row in qs:
                if not w.valid(row,phase):errors.append(w.last_failure);break
                count+=1
            target=np.array(seg["target_tcp"]);actual=w.robot.fk(knots[-1])
            pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]));oe=float(Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).magnitude())
            pose_errors.append(dict(phase=phase,position_error_m=pe,orientation_error_rad=oe))
            if pe>.00005 or oe>.0003:errors.append("FK_residual")
            if phase=="contact":
                try:w.attach(knots[-1])
                except ValueError as exc:errors.append(str(exc))
            if phase=="place_contact":
                try:w.release(knots[-1])
                except ValueError as exc:errors.append(str(exc))
            previous=knots[-1]
            print("VALIDATED",phase,"samples",count,"errors",len(errors),flush=True)
            if errors:break
        self.report=dict(status="VALID" if not errors else "INVALID",samples=count,errors=errors,pose_errors=pose_errors,
            trajectory_fingerprint=claimed,world_fingerprint=current_world_snapshot.fingerprint,
            checks="robot self/environment/tool, all rigid tool protrusions, 12 terminal seals, same payload/support, full final quintic interpolation",
            joint_l1_sampling_rad=.0008,time_sampling_s=.02,collision_margin_m=w.scene["margin_m"],
            mesh_error_reserve_m=w.scene["mesh_error_reserve_m"],continuous_collision_certification=False,
            scope="Dense geometric validation, no dynamics/tracking/hardware claim",
            support="GEOMETRIC_SUPPORT_COMPUTED" if w.released_pose is not None else "NOT_RELEASED")
        save_json(OUTPUT/"reports/validation.json",self.report)
        return P.PlanValidationResult(P.ValidationStatus.VALID if not errors else P.ValidationStatus.INVALID,current_world_snapshot,current_motion_boundary,
            "complete geometry "+self.report["status"],self.name,self.version)

class GeometricExecutionBackend(E.ExecutionBackend):
    """Consumes the validated interpolation and preserves one payload identity; never a hardware adapter."""
    def __init__(self,world,enable_hardware=False):
        require_simulation({"enable_hardware":enable_hardware});self.w=world;self._state=E.ExecutionBackendState.IDLE;self._boundary=None;self.feedback=[];self.events=[]
    @property
    def identity(self):return E.ExecutionBackendIdentity("eco65-geometric-simulation","1","1")
    @property
    def capabilities(self):return E.ExecutionBackendCapabilities(frozenset({P.PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY}),frozenset({P.BoundaryMode.STOP_BOUNDARY}),True,True,True,True,True,True,False,False)
    @property
    def health(self):return E.ExecutionBackendHealth.READY
    @property
    def state(self):return self._state
    def current_boundary(self):return self._boundary
    def start(self,plan_envelope):
        require_simulation(self.w.scene)
        if not plan_envelope.executable:raise ValueError("Unvalidated envelope")
        self.plan=plan_envelope;self.artifact=artifact_dict(plan_envelope.result);self._state=E.ExecutionBackendState.RUNNING
        self.w.payload_relative=None;self.w.released_pose=None;self._boundary=plan_envelope.expected_start_boundary
        return E.ExecutionCommandResult(E.ExecutionCommandStatus.ACCEPTED,"start_1","geometric_1",plan_envelope.plan_id,"simulation only")
    def request_stop(self,plan_id,reason):
        self._state=E.ExecutionBackendState.IDLE
        return E.ExecutionCommandResult(E.ExecutionCommandStatus.ACCEPTED,"stop_1","geometric_1",plan_id,reason)
    def poll(self):return self.feedback.pop(0) if self.feedback else None
    def run(self):
        if self._state!=E.ExecutionBackendState.RUNNING:raise RuntimeError("not started")
        tglobal=0.;frames=[]
        for seg in self.artifact["segments"]:
            duration=seg["t"][-1];times=np.unique(np.r_[np.arange(0,duration,.02),duration])
            qs,vs=sample_quintic_knots(seg["t"],seg["q"],times)
            next_video=0.
            for t,q,v in zip(times,qs,vs):
                if self._state!=E.ExecutionBackendState.RUNNING:raise InterruptedError("stopped")
                if not self.w.valid(q,seg["phase"]):raise ValueError("Execution geometry rejected "+str(self.w.last_failure))
                if t+1e-9>=next_video or t==duration:
                    frames.append(dict(t=tglobal+float(t),q=q.tolist(),phase=seg["phase"],box_pose=self.w.box_pose(q,seg["phase"]).tolist()))
                    next_video+=.2
            if seg["phase"]=="contact":
                actual=self.w.attach(q);self.events.append(dict(event="ATTACHED",box_id=self.w.box_id,time_s=tglobal+duration,relative_transform=actual.tolist(),seal_check=self.w.seal_fit(q,transform(self.w.scene["box"]["pose"]))))
            if seg["phase"]=="place_contact":
                released=self.w.release(q);self.events.append(dict(event="SUPPORTED_RELEASE",box_id=self.w.box_id,time_s=tglobal+duration,pose=released.tolist(),evidence="GEOMETRIC_SUPPORT_COMPUTED"))
            tglobal+=duration
        self._boundary=self.plan.expected_end_boundary;self._state=E.ExecutionBackendState.IDLE
        self.feedback.append(E.ExecutionFeedback(1,"geometric_1",self.plan.plan_id,E.ExecutionFeedbackStatus.SUCCEEDED,1.,self._boundary,"GEOMETRIC_REPLAY_ONLY"))
        self.events.append(dict(event="RETREAT_COMPLETE",time_s=tglobal,box_id=self.w.box_id))
        return dict(status="GEOMETRIC_REPLAY_ONLY",geometric_tasks_completed=1,dynamics="NOT_EVALUATED",hardware_connected=False,
            trajectory_fingerprint=self.artifact["trajectory_fingerprint"],
            duration_s=tglobal,events=self.events,frames=frames,supports_continuous_handoff=False)
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
    validator=ECO65PlanValidator(w);validation=validator.validate(envelope,snapshot,request.motion_boundary)
    if not validation.valid:raise RuntimeError(validation.message)
    envelope=envelope.revalidated(validation,timestamp_seconds=time.monotonic(),generation=0)
    execution=GeometricExecutionBackend(w);execution.start(envelope);replay=execution.run()
    save_json(OUTPUT/"replay/states.json",replay)
    print("GEOMETRIC EXECUTION COMPLETE",replay["duration_s"],"seconds",len(replay["frames"]),"video states",flush=True)
    return w,home,planner.artifact,replay,envelope
