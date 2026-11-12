"""Actually solve one cycle; replay samples carry robot, payload and belt state."""
import math
from pathlib import Path
from time import perf_counter
import numpy as np
from .geometry import OBB, make_transform, rotation_matrix_from_rpy
from .ik import solve_ik
from .planner import RRTConnectPlanner
from .conveyor_lift import coupled_conveyor_state
from .joint_search_outfeed import outfeed_path, PARK_FIRST
from .coupled_batch import message_box
from .s50_kinematic_sim import ik_candidates, target_contact, save, tool_virtual
from .s50_sim_collision import SimCollision
from .independent_cups import build_m710_independent_cup_array, evaluate_independent_cup_geometry
from .workcell_layout import canonical_digest


def run(c):
    start=perf_counter();checker=SimCollision(c);robot=c.robot
    target,contact=target_contact(c);height=.6;attached=False;relative=None;boxT=target.world_from_local.copy()
    samples=[];stages=[];failures=[];metrics=[];q=None
    velocity=c.settings['timing']['joint_velocity_rad_s'];accel=c.settings['timing']['joint_acceleration_rad_s2']
    report=dict(schema='s50_full_kinematic_cycle_v1',status='RUNNING',scope=c.settings['scope'],settings=c.settings,
        target=target.name,environment_fingerprint=c.frozen['environment_fingerprint'],initial_carton_ids=c.frozen['initial_carton_ids'],
        world_from_model_base=c.world.tolist(),flange_from_physical_contact=c.frames['nominal_compressed_contact'].tolist(),
        flange_from_virtual_task_tcp=c.frames['virtual_task_tcp'].tolist(),source_model_sha256=c.definition['definition_sha256'],
        initial_cartons=40,completed_cartons=0,controller_mapping_verified=False,dynamics_verified=False,
        native_planning_required=False,ik_backend='EXISTING_URDF_DLS',path_backend='EXISTING_RRT_CONNECT',
        collision_backend='PINOCCHIO_COAL_CAD_MESH_AND_ORIGINAL_TOOL_COMPOUND_SAMPLED',
        simulation_scene_fingerprint=canonical_digest(dict(environment=c.frozen['environment_fingerprint'],
            model=c.definition['definition_sha256'],conventions=c.settings,world_from_model_base=c.world.tolist())),
        tool_mass_kg=c.intrinsic['tool_mass_kg'],carton_mass_kg=c.intrinsic['carton_mass_kg'],
        core_pin_fk_max_error=checker.fk_crosscheck,stages=stages,samples=samples,failures=failures,compactness=metrics)
    def event(phase,dt=0.,qnew=None,box=None,attached_override=None):
        nonlocal q,boxT
        if qnew is not None:q=np.asarray(qnew).copy()
        if box is not None:boxT=np.asarray(box).copy()
        if attached:boxT=robot.fk(q)@relative
        samples.append(dict(time_s=(samples[-1]['time_s'] if samples else 0.)+float(dt),phase=phase,q_rad=q.tolist(),
            belt_height_m=float(height),attached=attached if attached_override is None else attached_override,
            world_from_carton=boxT.tolist(),world_from_physical_contact=robot.fk(q).tolist()))
    def valid(joints,phase,*,h=None,box=None,has_payload=None,allow_contact=None):
        hp=attached if has_payload is None else has_payload
        payload=robot.fk(joints)@relative if hp else (boxT if box is None else box)
        good=checker.check(joints,height if h is None else h,payload=payload,contact=(attached or phase in ['approach','grasp','release','drop']) if allow_contact is None else allow_contact,phase=phase)
        if not good and (not failures or failures[-1]!=checker.last):failures.append(dict(checker.last))
        return good
    def mark(phase,kind,begin,**data):
        stages.append(dict(phase=phase,method=kind,status='PASS',first_sample=begin,last_sample=len(samples)-1,**data))
        metrics.append(checker.compactness(q,height,phase))
        save(c.output/'cycle-progress.json',dict(status='RUNNING',stages=stages,sample_count=len(samples),last_q=q.tolist(),elapsed_s=perf_counter()-start))
        print(phase,'PASS',len(samples),'checks',checker.check_count,flush=True)
    def append_path(path,phase):
        # Stop-to-stop cubic per original solved edge: continuous conservative
        # velocity, zero speed at knots, bounded acceleration; no invented q.
        for a,b in zip(path,path[1:]):
            a=np.asarray(a);b=np.asarray(b);delta=float(np.max(np.abs(b-a)))
            duration=max(.08,1.5*delta/velocity,math.sqrt(6*delta/accel))
            n=max(1,math.ceil(delta/c.settings['timing']['joint_check_step_rad']),math.ceil(duration/.12))
            for u in np.linspace(0,1,n+1)[1:]:
                v=u*u*(3-2*u);candidate=a+(b-a)*v
                if not valid(candidate,phase):raise RuntimeError('PATH_SAMPLE_COLLISION:'+phase)
                event(phase,duration/n,qnew=candidate)
    def joint_move(goals,phase):
        begin=len(samples)
        for index,goal in enumerate(sorted(goals,key=lambda x:np.linalg.norm(x-q))):
            plan=RRTConnectPlanner(*robot.joint_limits.T,is_state_valid=lambda x:valid(x,phase),
                rng=np.random.default_rng(c.settings['seed']+len(stages)+index),max_iterations=1600,
                edge_resolution=.06,step_size=.20).plan(q,goal)
            if plan.success:
                append_path(plan.path,phase);mark(phase,'JOINT_RRT_CONNECT',begin,iterations=plan.iterations,path_nodes=len(plan.path));return
            failures.append(dict(stage=phase,kind='FINITE_PATH_SEARCH',message=plan.message,iterations=plan.iterations,goal_q=goal.tolist()))
        raise RuntimeError('FINITE_PATH_SEARCH_EXHAUSTED:'+phase)
    def cartesian(T,phase):
        begin=len(samples);initial=robot.fk(q);path=[q.copy()]
        n=max(1,math.ceil(np.linalg.norm(T[:3,3]-initial[:3,3])/c.settings['timing']['cartesian_step_m']))
        for u in np.linspace(0,1,n+1)[1:]:
            goal=T.copy();goal[:3,3]=initial[:3,3]+u*(T[:3,3]-initial[:3,3])
            r=solve_ik(robot,goal,path[-1],max_iterations=120,damping=.008,max_step=.15,position_tolerance=.00005,orientation_tolerance=.0002)
            c.attempts.append(dict(stage=phase,ik_success=r.success,iterations=r.iterations,position_error_m=r.position_error,orientation_error_rad=r.orientation_error,q_rad=r.q.tolist()))
            if not r.success or np.max(np.abs(r.q-path[-1]))>.4:raise RuntimeError('CARTESIAN_IK_FAILED:'+phase)
            if not valid(r.q,phase):raise RuntimeError('CARTESIAN_COLLISION:'+phase)
            path.append(r.q)
        append_path(path,phase);mark(phase,'POINTWISE_IK_PREVIOUS_SOLUTION_SEED',begin,ik_points=n)
    try:
        homes=ik_candidates(c,np.asarray(c.frozen['initial_physical_contact_target']),draws=96,stage='home',limit=4,valid=lambda x:valid(x,'home'))
        if not homes:raise RuntimeError('HOME_FINITE_IK_EXHAUSTED')
        homes.sort(key=lambda x:abs(x[0]));q=homes[0];home=q.copy();event('home');mark('home','INDEPENDENT_IK',0)
        pre=contact.copy();pre[0,3]-=.15
        goals=ik_candidates(c,pre,seeds=[q],draws=48,stage='pregrasp',limit=4,valid=lambda x:valid(x,'pregrasp'))
        if not goals:raise RuntimeError('PREGRASP_IK_EXHAUSTED')
        joint_move(goals,'pregrasp');cartesian(contact,'approach')
        # Eligible/commanded rings from the unchanged 72 cup geometry. Physical
        # target front is +X; contact X/Y are vertical/lateral on that face.
        cup_geometry=evaluate_independent_cup_geometry(robot.fk(q),target,'front',build_m710_independent_cup_array())
        eligible=list(cup_geometry.geometrically_eligible_mask)
        if not any(eligible):raise RuntimeError('NO_FULL_TARGET_CUP_RING')
        report['cups']=dict(mode='ideal_independent_cups',cup_ids=[cup.cup_id for cup in cup_geometry.cups],
            eligible=eligible,commanded=list(eligible),actual_geometric_contact=list(eligible),
            commanded_count=sum(eligible),force_verified=False,geometry=cup_geometry.to_dict(),
            actual_contact_semantics='GEOMETRIC_FK_CONTACT_NOT_PHYSICAL_SENSOR')
        relative=np.linalg.inv(robot.fk(q))@boxT;report['physical_contact_from_attached_box']=relative.tolist();attached=True
        event('grasp',.4);mark('grasp','GEOMETRIC_SUCTION_EVENT',len(samples)-1)
        lift=robot.fk(q).copy();lift[2,3]+=.02;cartesian(lift,'separate')
        withdraw=robot.fk(q).copy();withdraw[0,3]=-.7;cartesian(withdraw,'withdraw')
        park=q.copy() # Same-run solved posture, already clear of the complete lift stroke.
        # Keep the 400mm dimension across the 600mm longitudinal conveyor.
        # The released box's complete vertical corridor and the departing tool
        # must clear the arm, not just the attached endpoint.
        goals=[]
        receiving_height=1.3
        # At withdrawal the arm clears the complete lift stroke. Raising the
        # conveyor after folding can sweep through the shoulder housing even
        # when both endpoint heights are clear.
        begin=len(samples)
        for h in np.linspace(height,receiving_height,29)[1:]:
            dt=(h-height)/c.settings['timing']['belt_velocity_m_s'];height=float(h)
            if not valid(q,'belt_raise'):raise RuntimeError('BELT_RAISE_COLLISION')
            event('belt_raise',dt)
        mark('belt_raise','FROZEN_COUPLED_MECHANISM_AT_CLEAR_WITHDRAWAL',begin)
        def release_valid(joints):
            if not valid(joints,'transfer_goal',h=receiving_height):return False
            body=robot.fk(joints)@relative
            bottom=receiving_height+(np.abs(body[:3,:3])@target.half_extents)[2]
            for z in np.linspace(body[2,3],bottom,31):
                falling=body.copy();falling[2,3]=z
                if not valid(joints,'drop_corridor',h=receiving_height,box=falling,has_payload=False,allow_contact=True):return False
            start_pose=robot.fk(joints);goal_pose=start_pose.copy();goal_pose[:3,3]-=.12*goal_pose[:3,2];previous=joints
            for u in np.linspace(0,1,7)[1:]:
                pose=goal_pose.copy();pose[:3,3]=start_pose[:3,3]+u*(goal_pose[:3,3]-start_pose[:3,3])
                r=solve_ik(robot,pose,previous,max_iterations=120,damping=.008,max_step=.15,position_tolerance=.00005,orientation_tolerance=.0002)
                c.attempts.append(dict(stage='exit_corridor',ik_success=r.success,iterations=r.iterations,position_error_m=r.position_error,orientation_error_rad=r.orientation_error,q_rad=r.q.tolist()))
                if not r.success:return False
                if not valid(r.q,'exit_corridor',h=receiving_height,box=falling,has_payload=False,allow_contact=False):return False
                previous=r.q
            return True
        for release_z in [2.1,2.,1.9]:
            release_box=make_transform(np.eye(3),[-1.05,-.4,release_z])
            release=release_box@np.linalg.inv(relative)
            goals=ik_candidates(c,release,seeds=[q],draws=80,stage=f'transfer_goal_z{release_z:.3f}',limit=4,
                valid=release_valid)
            print('transfer goal',release_z,'valid',len(goals),flush=True)
            if goals:
                report['selected_release_box_pose']=release_box.tolist()
                report['receiving_belt_height_m']=receiving_height;break
        if not goals:raise RuntimeError('TRANSFER_GOAL_IK_OR_COLLISION_EXHAUSTED')
        joint_move(goals,'transfer')
        # Full box footprint must fit the unchanged transverse receiving top.
        ext=np.abs(boxT[:3,:3])@target.half_extents
        if not (-1.4+.005<=boxT[0,3]-ext[0] and boxT[0,3]+ext[0]<=-.7-.005 and abs(boxT[1,3])+ext[1]<=1.1-.005):
            raise RuntimeError('RELEASE_FOOTPRINT_OUTSIDE_TRANSVERSE_BELT')
        attached=False;event('release',.2);mark('release','KINEMATIC_DETACH_IN_RECEIVING_REGION',len(samples)-1)
        begin=len(samples);from_z=boxT[2,3];to_z=height+ext[2];report['nominal_drop_m']=float(from_z-to_z)
        for u in np.linspace(0,1,41)[1:]:
            boxT[2,3]=from_z+(to_z-from_z)*u
            if not valid(q,'drop',allow_contact=True):raise RuntimeError('NOMINAL_DROP_COLLISION')
            event('drop',.04)
        mark('drop','KINEMATIC_NOMINAL_RECEPTION_NOT_DYNAMICS',begin)
        exit_pose=robot.fk(q).copy();exit_pose[:3,3]-=.12*exit_pose[:3,2]
        cartesian(exit_pose,'exit')
        # Park in the same-run solved high withdrawal before allowing the carton to
        # pass the shoulder along the longitudinal belt. This is the existing
        # WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED process option.
        joint_move([park],'park')
        begin=len(samples)
        world=coupled_conveyor_state(c.layout,height,()).boxes
        landed=OBB(boxT[:3,3],target.half_extents,boxT[:3,:3],target.name,'carton')
        outfeed=outfeed_path(landed,world,speed=c.settings['timing']['outfeed_velocity_m_s'],tool_timing=PARK_FIRST,finite_support=True)
        previous=0.
        for step in outfeed['samples'][1:]:
            body=message_box(step['box']);boxT=body.world_from_local
            if not valid(q,'outfeed'):raise RuntimeError('OUTFEED_COLLISION')
            event('outfeed',step['t']-previous);previous=step['t']
        report['outfeed_contract']={k:v for k,v in outfeed.items() if k!='samples'}
        mark('outfeed','ORIGINAL_MINUS_Y_MINUS_X_KINEMATIC_ROUTING',begin)
        begin=len(samples)
        for h in np.linspace(height,.6,37)[1:]:
            dt=(height-h)/c.settings['timing']['belt_velocity_m_s'];height=float(h)
            if not valid(q,'belt_lower'):raise RuntimeError('BELT_LOWER_COLLISION')
            event('belt_lower',dt)
        mark('belt_lower','FROZEN_COUPLED_MECHANISM',begin)
        joint_move([home],'return')
        event('complete',.5)
        report.update(status='COMPLETED_KINEMATIC_SINGLE_CARTON',completed_cartons=1,remaining_cartons=39)
    except Exception as exc:
        report.update(status='KINEMATIC_ATTEMPT_FAILED',failure=dict(type=type(exc).__name__,reason=str(exc),last_collision=checker.last))
    report.update(ik_attempts=c.attempts,collision_state_checks=checker.check_count,collision_narrow_pair_queries=checker.pair_queries,
        elapsed_wall_s=perf_counter()-start,simulated_duration_s=samples[-1]['time_s'] if samples else 0.,continuous_collision_certified=False)
    index=len(list(c.output.glob('cycle-attempt-*.json')))+1
    save(c.output/f'cycle-attempt-{index:02d}.json',report);save(c.output/'cycle.json',report)
    return report
