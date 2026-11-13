"""Continuous single-carton kinematic extraction with geometric pose preferences."""
import math
from time import perf_counter
import numpy as np
from .geometry import OBB
from .ik import solve_ik
from .planner import RRTConnectPlanner
from .s50_kinematic_sim import ik_candidates,target_contact,save,simulation_coordinates
from .s50_kinematic_motion import posture,posture_cost,receiving_height,continuous_samples
from .workcell_layout import canonical_digest


def run(c):
    from .s50_sim_collision import SimCollision
    from .independent_cups import build_m710_independent_cup_array,evaluate_independent_cup_geometry
    from .conveyor_lift import coupled_conveyor_state
    from .joint_search_outfeed import outfeed_path,PARK_FIRST
    from .coupled_batch import message_box
    started=perf_counter();robot=c.robot;checker=SimCollision(c);target,contact=target_contact(c)
    settings=c.settings;preferences=settings['posture_preferences'];timing=settings['timing']
    q=None;height=.6;attached=False;relative=None;boxT=target.world_from_local.copy()
    samples=[];stages=[];failures=[];compactness=[]
    report=dict(schema='s50_full_kinematic_cycle_v1',status='RUNNING',scope=settings['scope'],settings=settings,
        target=target.name,source_model_sha256=c.definition['definition_sha256'],source_step_path=c.definition['source_step_path'],
        source_step_sha256=c.definition['source_step_sha256'],environment_fingerprint=c.frozen['environment_fingerprint'],
        world_from_model_base=c.world.tolist(),flange_from_physical_contact=c.frames['nominal_compressed_contact'].tolist(),
        flange_from_virtual_task_tcp=c.frames['virtual_task_tcp'].tolist(),initial_carton_ids=c.frozen['initial_carton_ids'],
        initial_cartons=40,completed_cartons=0,controller_mapping_verified=False,dynamics_verified=False,native_planning_required=False,
        tool_mass_kg=c.intrinsic['tool_mass_kg'],carton_mass_kg=c.intrinsic['carton_mass_kg'],core_pin_fk_max_error=checker.fk_crosscheck,
        ik_backend='EXISTING_URDF_DLS_PREVIOUS_POINT_SEED',path_backend='DIRECT_CARTESIAN_FLOW_AND_EXISTING_RRT_CONNECT',
        collision_backend='PINOCCHIO_COAL_CAD_AND_ORIGINAL_TOOL_COMPOUND_SAMPLED',
        simulation_scene_fingerprint=canonical_digest(dict(environment=c.frozen['environment_fingerprint'],
            model=c.definition['definition_sha256'],conventions=settings,world_from_model_base=c.world.tolist())),
        samples=samples,stages=stages,failures=failures,compactness=compactness)

    def valid(joints,phase,h=None,body=None,carrying=None,allow_contact=None):
        hp=attached if carrying is None else carrying
        payload=robot.fk(joints)@relative if hp else (boxT if body is None else body)
        ok=checker.check(joints,height if h is None else h,payload=payload,
            contact=(attached or phase in ['approach','grasp','drop']) if allow_contact is None else allow_contact,phase=phase)
        if not ok:failures.append(dict(checker.last))
        return ok

    def event(phase,dt=0.,joints=None,velocity=None,acceleration=None):
        nonlocal q,boxT
        if joints is not None:q=np.asarray(joints).copy()
        if attached:boxT=robot.fk(q)@relative
        row=dict(time_s=(samples[-1]['time_s'] if samples else 0.)+float(dt),phase=phase,q_rad=q.tolist(),
            belt_height_m=float(height),attached=attached,world_from_carton=boxT.tolist(),
            world_from_physical_contact=robot.fk(q).tolist(),posture=posture(robot,q))
        if velocity is not None:row.update(q_velocity_rad_s=np.asarray(velocity).tolist(),q_acceleration_rad_s2=np.asarray(acceleration).tolist())
        samples.append(row)

    def mark(phase,method,begin,**extra):
        stages.append(dict(phase=phase,method=method,status='PASS',first_sample=begin,last_sample=len(samples)-1,**extra))
        compactness.append(checker.compactness(q,height,phase))
        save(c.output/'cycle-progress.json',dict(status='RUNNING',stages=stages,sample_count=len(samples),last_q=q.tolist()))
        print(phase,'PASS',len(samples),'checks',checker.check_count,flush=True)

    def score(joints,reference=None):return posture_cost(robot,joints,preferences,reference)

    def emit_continuous(path,phase,labels=None,stations=None):
        rows,law=continuous_samples(path,timing['joint_velocity_rad_s'],timing['joint_acceleration_rad_s2'],stations)
        # Validate the changed interpolation itself, not just its original knots.
        for row in rows:
            if not valid(row['q'],phase):raise RuntimeError('CONTINUOUS_PATH_COLLISION:'+phase)
        begin=len(samples);current=None;previous=0.
        for row in rows[1:]:
            label=phase if labels is None else labels[min(row['knot_index']+1,len(labels)-1)]
            if current is not None and label!=current:
                mark(current,'POINTWISE_IK_CONTINUOUS_FLOW',begin);begin=len(samples)
            current=label
            event(label,row['time_s']-previous,row['q'],row['velocity'],row['acceleration']);previous=row['time_s']
        mark(current,'POINTWISE_IK_CONTINUOUS_FLOW' if labels else 'CONTINUOUS_CHECKED_JOINT_PATH',begin,timing=law)
        return law

    def joint_move(goals,phase):
        for i,goal in enumerate(sorted(goals,key=lambda v:score(v,q)+.1*np.linalg.norm(v-q))):
            planner=RRTConnectPlanner(*robot.joint_limits.T,is_state_valid=lambda v:valid(v,phase),
                rng=np.random.default_rng(settings['seed']+len(stages)+i),max_iterations=1600,edge_resolution=.04,step_size=.20)
            plan=planner.plan(q,goal)
            if not plan.success:
                failures.append(dict(kind='FINITE_JOINT_SEARCH',phase=phase,message=plan.message));continue
            path=planner.shortcut(plan.path,attempts=24)
            try:emit_continuous(path,phase);return
            except RuntimeError:
                # Recover only an unloaded interpolation using the already checked
                # original edges; a loaded flow never falls back to a large RRT tour.
                if attached:raise
                for a,b in zip(path,path[1:]):emit_continuous([a,b],phase)
                return
        raise RuntimeError('JOINT_PATH_EXHAUSTED:'+phase)

    def pointwise(poses,start,phase):
        path=[np.asarray(start).copy()]
        for T in poses[1:]:
            r=solve_ik(robot,T,path[-1],max_iterations=160,damping=.008,max_step=.15,
                position_tolerance=.00003,orientation_tolerance=.0001)
            c.attempts.append(dict(stage=phase,ik_success=r.success,iterations=r.iterations,
                position_error_m=r.position_error,orientation_error_rad=r.orientation_error,q_rad=r.q.tolist()))
            if not r.success or np.max(abs(r.q-path[-1]))>.4:
                raise RuntimeError('POINTWISE_IK_FAILED:'+phase)
            if not valid(r.q,phase):raise RuntimeError('POINTWISE_COLLISION:'+phase)
            path.append(r.q)
        return path

    def line(a,b,step=.015):
        count=max(1,math.ceil(np.linalg.norm(b[:3,3]-a[:3,3])/step));out=[]
        for u in np.linspace(0,1,count+1):
            T=b.copy();T[:3,3]=(1-u)*a[:3,3]+u*b[:3,3];out.append(T)
        return out

    def cartesian_to(T,phase):
        poses=line(robot.fk(q),T);path=pointwise(poses,q,phase)
        return emit_continuous(path,phase)

    try:
        seed=simulation_coordinates(settings,np.deg2rad([150,-20,-100,-90,70,180]))
        homes=ik_candidates(c,np.asarray(c.frozen['initial_physical_contact_target']),seeds=[seed],draws=64,
            stage='home',limit=6,valid=lambda x:valid(x,'home'))
        if not homes:raise RuntimeError('HOME_IK_EXHAUSTED')
        q=min(homes,key=lambda x:score(x,seed));home=q.copy();event('home');mark('home','SOFT_POSTURE_RANKED_INDEPENDENT_IK',0)
        pre=contact.copy();pre[0,3]-=.15
        goals=ik_candidates(c,pre,seeds=[q,seed],draws=48,stage='pregrasp',limit=6,valid=lambda x:valid(x,'pregrasp'))
        if not goals:raise RuntimeError('PREGRASP_IK_EXHAUSTED')
        joint_move(goals,'pregrasp')
        # Raise while the empty tool is safely at pregrasp, so the attached
        # extraction can flow to release with no belt-wait or joint-path detour.
        reception=receiving_height(boxT,target.half_extents,**settings['receiving_height'])
        report['receiver_rule_at_pick']=reception;desired=reception['surface_height_m'];begin=len(samples)
        for h in np.linspace(height,desired,max(2,math.ceil(abs(desired-height)/.02)+1))[1:]:
            dt=abs(h-height)/timing['belt_velocity_m_s'];height=float(h)
            if not valid(q,'belt_raise'):raise RuntimeError('BELT_RAISE_COLLISION')
            event('belt_raise',dt)
        mark('belt_raise','BOX_BOTTOM_MINUS_100MM_CLAMPED_TO_FROZEN_STROKE',begin,rule=reception)
        cartesian_to(contact,'approach')
        geometry=evaluate_independent_cup_geometry(robot.fk(q),target,'front',build_m710_independent_cup_array())
        eligible=list(geometry.geometrically_eligible_mask)
        if not any(eligible):raise RuntimeError('NO_FULL_TARGET_SEAL_RING')
        report['cups']=dict(mode='ideal_independent_cups',cup_ids=[cup.cup_id for cup in geometry.cups],eligible=eligible,
            commanded=list(eligible),actual_geometric_contact=list(eligible),commanded_count=sum(eligible),force_verified=False,
            actual_contact_semantics='GEOMETRIC_FK_CONTACT_NOT_PHYSICAL_SENSOR',geometry=geometry.to_dict())
        relative=np.linalg.inv(robot.fk(q))@boxT;report['physical_contact_from_attached_box']=relative.tolist();attached=True
        event('grasp',.4);mark('grasp','GEOMETRIC_CONTACT_THEN_FIXED_RELATIVE_TRANSFORM',len(samples)-1)
        initial=robot.fk(q);lift=initial.copy();lift[2,3]+=.02
        # The carton is fully clear before changing its lateral direction.
        clear=lift.copy();clear[0,3]=-(2*target.half_extents[0]+.05)
        loaded_path=None
        for release_y in settings['loaded_flow']['release_y_candidates_m']:
            body=boxT.copy();body[:3,3]=[settings['loaded_flow']['release_box_center_x_m'],release_y,lift[2,3]]
            goal=body@np.linalg.inv(relative)
            poses=line(initial,lift);labels=['separate']*len(poses)
            segment=line(lift,clear);poses+=segment[1:];labels+=['withdraw']*(len(segment)-1);clear_index=len(poses)-1
            # A short monotone-radius arc avoids the near-axis wrist singularity.
            # It has fixed endpoints in the receiving belt, never a base scan.
            flange=robot.named_link_frames(q)['s50_flange']
            center=c.world[:2,3]+(initial[:2,3]-flange[:2,3]);a=clear[:2,3]-center;b=goal[:2,3]-center
            r0,r1=np.linalg.norm(a),np.linalg.norm(b);theta0=np.arctan2(a[1],a[0]);delta=(np.arctan2(b[1],b[0])-theta0+np.pi)%(2*np.pi)-np.pi
            count=max(20,math.ceil((abs(delta)*max(r0,r1)+abs(r1-r0))/.015))
            for u in np.linspace(0,1,count+1)[1:]:
                T=goal.copy();radius=(1-u)*r0+u*r1;theta=theta0+u*delta
                T[:2,3]=center+radius*np.array([np.cos(theta),np.sin(theta)]);poses.append(T);labels.append('transfer')
            try:
                path=pointwise(poses,q,'loaded_flow');end=path[-1];body=robot.fk(end)@relative
                ext=np.abs(body[:3,:3])@target.half_extents
                if not (-1.395<=body[0,3]-ext[0] and body[0,3]+ext[0]<=-.705 and abs(body[1,3])+ext[1]<=1.095):
                    raise RuntimeError('RELEASE_FOOTPRINT_OUTSIDE_RECEIVER')
                for z in np.linspace(body[2,3],height+ext[2],41):
                    falling=body.copy();falling[2,3]=z
                    if not valid(end,'drop_corridor',body=falling,carrying=False,allow_contact=True):raise RuntimeError('DROP_CORRIDOR_COLLISION')
                stations=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.array([p[:3,3] for p in poses]),axis=0),axis=1))]
                law=emit_continuous(path,'loaded_flow',labels,stations)
                loaded_path=path;park=path[clear_index];report['loaded_flow']=dict(method='POINTWISE_IK_SHORT_TASK_SPACE_ARC',
                    release_y_m=release_y,center_world_xy_m=center.tolist(),cartesian_path_length_m=float(stations[-1]),
                    solver_knots=len(path),timing=law,rrt_used=False,intermediate_dwell=False)
                report['selected_release_box_pose']=body.tolist();break
            except RuntimeError as exc:
                failures.append(dict(kind='LOADED_ROUTE_REJECTED',release_y_m=release_y,reason=str(exc)))
                print('loaded route rejected',release_y,str(exc),flush=True)
        if loaded_path is None:raise RuntimeError('DIRECT_LOADED_FLOW_EXHAUSTED')
        report['receiver_rule_at_release']=receiving_height(boxT,target.half_extents,**settings['receiving_height'])
        report['receiving_belt_height_m']=height
        attached=False;event('release',.2);mark('release','DETACH_IN_CHECKED_RECEIVER',len(samples)-1)
        begin=len(samples);from_z=boxT[2,3];ext=np.abs(boxT[:3,:3])@target.half_extents;to_z=height+ext[2]
        report['nominal_drop_m']=float(from_z-to_z)
        for u in np.linspace(0,1,41)[1:]:
            boxT[2,3]=(1-u)*from_z+u*to_z
            if not valid(q,'drop',allow_contact=True):raise RuntimeError('DROP_COLLISION')
            event('drop',.04)
        mark('drop','KINEMATIC_ASSUMED_RECEPTION',begin)
        departure=robot.fk(q);departure[:3,3]-=.12*departure[:3,2];cartesian_to(departure,'exit')
        try:cartesian_to(robot.fk(park),'park')
        except RuntimeError:joint_move([park],'park')
        begin=len(samples);world=coupled_conveyor_state(c.layout,height,()).boxes
        landed=OBB(boxT[:3,3],target.half_extents,boxT[:3,:3],target.name,'carton')
        outfeed=outfeed_path(landed,world,speed=timing['outfeed_velocity_m_s'],tool_timing=PARK_FIRST,finite_support=True)
        previous=0.
        for step in outfeed['samples'][1:]:
            boxT=message_box(step['box']).world_from_local
            if not valid(q,'outfeed'):raise RuntimeError('OUTFEED_COLLISION')
            event('outfeed',step['t']-previous);previous=step['t']
        report['outfeed_contract']={k:v for k,v in outfeed.items() if k!='samples'}
        mark('outfeed','ORIGINAL_MINUS_Y_MINUS_X_ROUTING',begin)
        begin=len(samples)
        for h in np.linspace(height,.6,46)[1:]:
            dt=abs(h-height)/timing['belt_velocity_m_s'];height=float(h)
            if not valid(q,'belt_lower'):raise RuntimeError('BELT_LOWER_COLLISION')
            event('belt_lower',dt)
        mark('belt_lower','FROZEN_COUPLED_MECHANISM',begin);joint_move([home],'return');event('complete',.5)
        report.update(status='COMPLETED_KINEMATIC_SINGLE_CARTON',completed_cartons=1,remaining_cartons=39)
    except Exception as exc:
        report.update(status='KINEMATIC_ATTEMPT_FAILED',failure=dict(type=type(exc).__name__,reason=str(exc),last_collision=checker.last))
    report.update(ik_attempts=c.attempts,collision_state_checks=checker.check_count,collision_narrow_pair_queries=checker.pair_queries,
        simulated_duration_s=samples[-1]['time_s'] if samples else 0.,elapsed_wall_s=perf_counter()-started,continuous_collision_certified=False)
    if samples:
        report['posture_summary']={}
        for scope,rows in [('whole_cycle',samples),('approach_to_release',[s for s in samples if s['phase'] in ['approach','grasp','separate','withdraw','transfer','release']])]:
            if rows:report['posture_summary'][scope]=dict(j2_range_deg=[min(s['posture']['j2_deg'] for s in rows),max(s['posture']['j2_deg'] for s in rows)],
                maximum_abs_forearm_elevation_deg=max(abs(s['posture']['forearm_elevation_deg']) for s in rows),
                maximum_wrist_down_error_deg=max(s['posture']['wrist_down_error_deg'] for s in rows),soft_preferences_only=True)
    index=len(list(c.output.glob('cycle-attempt-*.json')))+1
    save(c.output/f'cycle-attempt-{index:02d}.json',report);save(c.output/'cycle.json',report)
    return report
