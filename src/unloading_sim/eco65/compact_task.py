"""Bounded one-carton planning and explicit ideal outfeed, using the frozen layout."""
from __future__ import annotations
import copy,time,hashlib,json
import numpy as np
from .model import *
from .task_world import UnloadingTaskWorld
from .unloading_layout import face_tcp,FACE_ROTATIONS,support_at
from .pipeline import known_world,ECO65PlanValidator,GeometricExecutionBackend
from . import planning_contracts as P
from .task_checks import motion_limits,runtime_fingerprint,EVENTS,outfeed_preconditions
from ..ik import solve_ik
from ..planner import RRTConnectPlanner
from ..timing import time_parameterize_joint_path

class CandidateFailure(RuntimeError):
    def __init__(self,code,phase,evidence):
        super().__init__(code+' '+phase);self.code=code;self.phase=phase;self.evidence=evidence

def candidates(scene):
    top=sorted((b for b in scene['boxes'] if not b['supports']),key=lambda b:-b['pose'][1])
    c=scene['source_config']['conveyors'];rows=[]
    for face,route in [('top','A'),('top','B'),('front','B')]:
        region='transverse' if route=='A' else 'longitudinal'
        for b in top:
            rows.append(dict(box_id=b['id'],face=face,route=route,region='conveyor_'+region,receive_xy=c[region]['receive_center_xy_m']))
    return rows

def prepare_world(snapshot,selection,home):
    w=UnloadingTaskWorld(snapshot['tool'],snapshot['scene']);w.select_target(selection['box_id'],face=selection['face'])
    w.receiving_region=selection['region'];w.set_state(home,'initial');return w

def plan_candidate(w,home,selection,seed,deadline,cancel=lambda:False):
    cfg=w.scene['source_config']['task'];rng=np.random.default_rng(seed);segments=[];attempts=[];q=np.array(home);phase='initial';last_progress=time.perf_counter()
    def check():
        nonlocal last_progress
        if time.perf_counter()-last_progress>=30:
            print('SEARCH_PROGRESS',selection['box_id'],selection['route'],phase,'geometry_checks',w.calls,'remaining_s',round(max(0,deadline-time.perf_counter())),flush=True);last_progress=time.perf_counter()
        if cancel():raise CandidateFailure('CANCELLED',phase,attempts)
        if time.perf_counter()>deadline:raise CandidateFailure('BUDGET_EXHAUSTED',phase,attempts)
    def valid(state):check();return w.valid(state,phase)
    def ik(target):
        found=0;collisions=[];best=None
        seeds=[q]+[rng.uniform(w.robot.joint_limits[:,0],w.robot.joint_limits[:,1]) for _ in range(cfg['ik_seeds'])]
        for i,initial in enumerate(seeds):
            check();r=solve_ik(w.robot,target,initial,max_iterations=350,position_tolerance=.00002,orientation_tolerance=.0002,deadline_monotonic=deadline)
            if best is None or r.position_error+r.orientation_error<best['score']:best=dict(score=float(r.position_error+r.orientation_error),position_error_m=float(r.position_error),orientation_error_rad=float(r.orientation_error))
            if r.success:
                found+=1
                if valid(r.q):
                    report=dict(phase=phase,seed_index=i,ik_solutions=found,prior_collisions=collisions,best=best);attempts.append(report);return np.array(r.q),report
                collisions.append(dict(q=np.asarray(r.q).tolist(),collision=copy.deepcopy(w.last_failure),target_tcp=target.tolist()))
        evidence=dict(phase=phase,attempts=len(seeds),ik_solutions=found,collisions=collisions,best=best);attempts.append(evidence)
        raise CandidateFailure('IK_COLLISION_CANDIDATES_EXHAUSTED' if found else 'IK_NOT_FOUND_WITHIN_BUDGET',phase,attempts)
    velocity=[float(j.find('limit').attrib['velocity']) for j in ET.parse(URDF).getroot().findall('joint') if j.find('limit') is not None]
    def guided_path(target):
        """Finite Cartesian guide; every IK state and joint edge is checked with the same world."""
        from scipy.spatial.transform import Rotation
        path=[q.copy()];states=0;prefix=0
        if phase=='retreat':
            for previous in reversed(segments[-2:]):
                for goal in reversed(np.asarray(previous['q'])):
                    last=path[-1];n=max(1,int(np.ceil(np.max(np.abs(goal-last))/.003)))
                    for fraction in np.linspace(0,1,n+1):
                        check();states+=1
                        state=last+(goal-last)*fraction
                        if not valid(state):return None,dict(status='RELEASED_SCENE_BACKTRACK_COLLISION',q=state.tolist(),collision=copy.deepcopy(w.last_failure),states=states)
                    if np.linalg.norm(goal-last)>1e-10:path.append(goal);prefix+=1
        start=w.robot.fk(path[-1]);waypoints=[]
        if phase=='loaded_transfer' and selection['route']=='B':
            mid=target.copy();mid[0,3]=w.scene['source_config']['conveyors']['transverse']['bounds_xy_m'][1]-w.boxes[w.box_id]['size_m'][0]/4
            waypoints.append(mid)
        waypoints.append(target);current=start
        for end in waypoints:
            delta=Rotation.from_matrix(end[:3,:3]@current[:3,:3].T).as_rotvec();steps=max(1,int(np.ceil(np.linalg.norm(end[:3,3]-current[:3,3])/.008)),int(np.ceil(np.linalg.norm(delta)/.02)))
            first=current.copy()
            for i in range(1,steps+1):
                check();u=i/steps;pose=transform(first[:3,3]+u*(end[:3,3]-first[:3,3]),Rotation.from_rotvec(u*delta).as_matrix()@first[:3,:3])
                result=solve_ik(w.robot,pose,path[-1],max_iterations=220,position_tolerance=.00002,orientation_tolerance=.0002,deadline_monotonic=deadline)
                if not result.success:return None,dict(status='GUIDE_IK_FAILED',target_tcp=pose.tolist(),states=states)
                last=path[-1];goal=np.asarray(result.q);n=max(1,int(np.ceil(np.max(np.abs(goal-last))/.003)))
                for fraction in np.linspace(0,1,n+1):
                    states+=1
                    if not valid(last+(goal-last)*fraction):return None,dict(status='GUIDE_COLLISION',target_tcp=pose.tolist(),q=(last+(goal-last)*fraction).tolist(),collision=copy.deepcopy(w.last_failure),states=states)
                path.append(goal);current=pose
        return path,dict(status='GUIDE_VALID',states=states,waypoints=len(path),backtracked_knots=prefix,method='bounded Cartesian guide and released-scene backtrack; full joint-edge checks')
    def add(name,target):
        nonlocal q,phase
        phase=name;check();guided=None;guide_report=None
        if phase in ('loaded_transfer','retreat') and selection.get('guide',False):guided,guide_report=guided_path(target)
        if guided is not None:
            path=guided;goal=path[-1];record=dict(method='cartesian_guided');search=guide_report
        else:
            goal,record=ik(target)
            if guide_report:record['guide_failure']=guide_report
            planner=RRTConnectPlanner(w.robot.joint_limits[:,0],w.robot.joint_limits[:,1],valid,edge_resolution=.006,max_iterations=cfg['rrt_iterations'],rng=rng)
            result=planner.plan(q,goal,time_limit_seconds=max(0.,deadline-time.perf_counter()))
            if not result.success:raise CandidateFailure('BUDGET_EXHAUSTED' if 'time limit' in result.message else 'PATH_SEARCH_EXHAUSTED',phase,dict(message=result.message,last_collision=w.last_failure,ik=attempts,guide=guide_report,search=result.search_evidence))
            path=result.path;search=result.search_evidence
        limits=motion_limits(w.scene,phase,velocity);timed=time_parameterize_joint_path(path,limits)
        segments.append(dict(phase=phase,q=timed.positions.tolist(),t=timed.time_from_start.tolist(),duration_s=timed.duration_seconds,target_tcp=target.tolist(),ik=record,search=search,timing=timed.audit(limits)))
        q=goal;w.set_state(q,phase);print('PLANNED',selection['box_id'],selection['face'],selection['route'],phase,len(path),round(timed.duration_seconds,2),flush=True)
    box=w.boxes[w.box_id];pick=face_tcp(transform(box['pose']),box['size_m'],selection['face']);normal=-pick[:3,2]
    approach=pick.copy();approach[:3,3]+=.035*normal
    try:
        add('approach',approach);add('contact',pick);relative=w.attach(q)
        # Entire bottom clears the 180 mm receiving belt plus 30 mm design transit clearance.
        lift=pick.copy();rise=max(.025,w.scene['source_config']['conveyors']['surface_z_m']+.03-(box['pose'][2]-box['size_m'][2]/2));lift[2,3]+=rise
        add('loaded_lift',lift)
        if selection['face']=='front':
            separate=lift.copy();separate[0,3]-=box['size_m'][0]+2*w.scene['margin_m'];add('front_separation',separate)
        destination=transform([*selection['receive_xy'],w.scene['source_config']['conveyors']['surface_z_m']+box['size_m'][2]/2])
        place=destination@np.linalg.inv(relative);preplace=place.copy();preplace[2,3]+=.03
        add('loaded_transfer',preplace);add('place_contact',place);released=w.release(q)
        add('retreat',w.robot.fk(home))
    except ValueError as exc:
        raise CandidateFailure('GEOMETRIC_EVENT_REJECTED',phase,dict(error=str(exc),segments=segments,last_collision=copy.deepcopy(w.last_failure))) from exc
    except CandidateFailure as exc:
        exc.evidence=dict(details=exc.evidence,segments=segments,last_collision=copy.deepcopy(w.last_failure));raise
    artifact=dict(task_schema=2,segments=segments,attachment_tcp_to_box=relative.tolist(),released_box_pose=released.tolist(),box_id=w.box_id,selection=selection,
        required_events=[dict(after_phase=p,event=e,box_id=w.box_id) for p,e in EVENTS],runtime_fingerprint=runtime_fingerprint(w),tool_fingerprint=canonical_fingerprint(w.tool),
        source_world=known_world_from_initial(w,home).fingerprint,geometry_mode='GEOMETRIC_REPLAY_ONLY',dynamics='NOT_EVALUATED',seed=seed)
    artifact['trajectory_fingerprint']=canonical_fingerprint(artifact);return artifact

def known_world_from_initial(w,home):
    current=w.scene
    try:w.scene=w.initial_scene;return known_world(w,home)
    finally:w.scene=current

def envelope_for(w,home,artifact,run_id):
    snap=known_world(w,home);candidate=P.PlanningCandidate(w.box_id,artifact['selection']['face']+'_'+artifact['selection']['route'])
    req=P.PlanningRequest(run_id,snap,(candidate,),seed=artifact['seed'])
    trajectory=[tuple(home)]+[tuple(q) for seg in artifact['segments'] for q in seg['q'][1:]]
    result=P.PlanningResult.succeeded(candidate,trajectory,artifact_kind=P.PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY,metadata=artifact,
        expected_start_boundary=P.MotionBoundaryState.stopped(home),expected_end_boundary=P.MotionBoundaryState.stopped(trajectory[-1]))
    identity=P.BackendIdentity('eco65-compact-ik-rrt','2','2',snap.robot_model_fingerprint,snap.world_model_fingerprint)
    provenance=P.BackendProvenance(identity,artifact['seed'])
    return P.PlanEnvelope(run_id,req,candidate,P.PlanningPath.COLD,result,snap,None,result.expected_start_boundary,result.expected_end_boundary,None,0,provenance,result.artifact_kind,snap.robot_model_fingerprint,snap.world_model_fingerprint)

def ideal_outfeed(w,replay,selection):
    events=replay['events'];result=dict(status='NOT_STARTED',outfed_assumed=0,mode='IDEAL_OUTFEED',dynamics='NOT_EVALUATED',frames=[],events=[])
    if not outfeed_preconditions(w,replay):return result
    result['execution_id']=replay['execution_id'];result['box_id']=w.box_id
    q=np.array(replay['frames'][-1]['q']);pose=w.released_pose.copy();start=pose.copy();cfg=w.scene['source_config']['conveyors'];size=w.boxes[w.box_id]['size_m'];z=cfg['surface_z_m'];speed=w.scene['source_config']['task']['outfeed_speed_m_s']
    if any(ids and ids!=[w.box_id] for ids in w.scene['occupancy'].values()):result['status']='OCCUPANCY_CONFLICT';return result
    route=[pose[:2,3].tolist()]
    if selection['route']=='A':route.append([pose[0,3],sum(cfg['longitudinal']['bounds_xy_m'][2:])/2])
    route.append(cfg['outlet']['final_center_xy_m']);t=replay['duration_s'];next_video=t
    # Preflight whole downstream corridor at <=1 mm before starting; robot stays at actual retreat q.
    samples=[]
    for a,b in zip(route[:-1],route[1:]):
        a,b=np.array(a),np.array(b);dist=float(np.linalg.norm(b-a));n=max(1,int(np.ceil(dist/.001)))
        for i in range(1,n+1):samples.append((a+(b-a)*i/n,dist/n/speed))
    probe=UnloadingTaskWorld(w.tool,w.initial_scene);probe.select_target(w.box_id,face=w.active_face);probe.receiving_region=w.receiving_region;probe.released=True
    failure=None
    for xy,dt in samples:
        pose[:2,3]=xy;probe.released_pose=pose.copy()
        # Outfeed may span the coplanar transverse/longitudinal union, preserving full bottom support.
        half=np.array(size)/2;extent=np.abs(pose[:2,:3])@half
        support=support_at(w.scene,xy,[* (2*extent),size[2]],z)
        if support['coverage']<1-1e-8 or not probe.valid(q,'ideal_outfeed'):
            failure=dict(xy_m=xy.tolist(),support=support,collision=copy.deepcopy(probe.last_failure));break
    probe.close()
    if failure:result.update(status='OUTFEED_PREFLIGHT_FAILED',failure=failure);return result
    pose=start.copy()
    for xy,dt in samples:
        pose[:2,3]=xy;w.released_pose=pose.copy()
        if not w.valid(q,'ideal_outfeed'):
            result.update(status='OUTFEED_EXECUTION_FAILED',failure=copy.deepcopy(w.last_failure));return result
        t+=dt
        for ids in w.scene['occupancy'].values():
            if w.box_id in ids:ids.remove(w.box_id)
        owners={o['owner'] for o in w.scene['obstacles'] if o['id'] in support_at(w.scene,xy,size,z)['surface_ids']}
        for owner in owners:w.scene['occupancy'][owner]=[w.box_id]
        if t>=next_video:
            result['frames'].append(dict(t=t,q=q.tolist(),qd=[0.]*6,phase='ideal_outfeed',box_id=w.box_id,box_pose=pose.tolist(),box_state='IDEAL_OUTFEED'));next_video+=.2
    extent=float(np.abs(pose[0,:3])@(np.array(size)/2));outside=pose[0,3]+extent<w.scene['opening_x_m']
    if outside:
        w.box_states[w.box_id]='OUTFED_ASSUMED';result.update(status='OUTFED_ASSUMED',outfed_assumed=1,final_box_pose=pose.tolist(),entire_box_outside=True,duration_s=t-replay['duration_s'])
        result['events'].append(dict(event='OUTFED_ASSUMED',box_id=w.box_id,time_s=t))
        result['frames'].append(dict(t=t,q=q.tolist(),qd=[0.]*6,phase='ideal_outfeed',box_id=w.box_id,box_pose=pose.tolist(),box_state='OUTFED_ASSUMED'))
    else:result['status']='OUTSIDE_CHECK_FAILED'
    return result
