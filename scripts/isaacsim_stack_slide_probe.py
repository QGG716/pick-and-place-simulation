"""One finite native-box slide, using the production stack adjudicator.

Mechanism fixture only: a fixed lower box and a finite-force prismatic drive
constrain the upper box like an attached payload. No rigid-body pose is set
after initialization. This is not a robot cycle or a completed carton.
"""
import argparse
import json
from pathlib import Path
import sys
import time

parser=argparse.ArgumentParser()
parser.add_argument('--project-root',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(args.project_root/'src'))
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':640,'height':360})
gate=None
writer=None
result={'scope':'NATIVE_BOX_CONTACT_TO_EDGE_SLIDE_MECHANISM_ONLY','robot_cartons_completed':0}
try:
    import numpy as np
    import yaml
    import cv2
    import omni.usd
    import omni.replicator.core as rep
    from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf,PhysicsSchemaTools
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import RigidPrim
    from isaacsim.core.simulation_manager import SimulationManager
    from omni.physx import get_physx_simulation_interface
    from unloading_sim.geometry import OBB
    from unloading_sim.serial_unloading import rotation_from_actual_quaternion
    from unloading_sim.collision_policy import SimulationCollisionPolicy
    from unloading_sim.m710_replay_physics import ActualStackContactMonitor
    from unloading_sim.stack_clearance import StackClearanceStep,copy_contact_points
    from unloading_sim.isaac_collision_policy import (verified_stack_cube_shapes,
        author_explicit_collision_offsets,read_effective_collision_offsets,
        verify_effective_collision_offsets,classify_poc_runtime_pair)
    dynamics=yaml.safe_load((args.project_root/'configs/simulation/m710id70_official_dynamics_v2.yaml').read_text())
    sim=dynamics['simulation'];dt=float(sim['physics_time_step_s'])
    policy=SimulationCollisionPolicy.from_mapping(yaml.safe_load((args.project_root/'configs/validation/m710id70_layout_poc_pair_clearance.yaml').read_text())['collision_policy'])
    stage=omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    material=UsdShade.Material.Define(stage,'/Probe/CartonMaterial')
    api=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    for key,create in [('static_friction',api.CreateStaticFrictionAttr),('dynamic_friction',api.CreateDynamicFrictionAttr),('restitution',api.CreateRestitutionAttr)]:
        create(float(dynamics['contacts']['carton'][key]))
    records=[];paths=[]
    for name,z,color in [('neighbor',.15,(.55,.35,.2)),('target',.45,(.1,.6,.8))]:
        path='/Probe/'+name;paths.append(path)
        cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(1.)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xf=UsdGeom.XformCommonAPI(cube.GetPrim());xf.SetTranslate(Gf.Vec3d(0,0,z));xf.SetScale(Gf.Vec3f(.6,.4,.3))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim());UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        mass=UsdPhysics.MassAPI.Apply(cube.GetPrim());mass.CreateMassAttr(42.5)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.diag(np.asarray(dynamics['cartons']['inertia_tensor_com_kg_m2'])).tolist()))
        body=PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim())
        body.CreateSolverPositionIterationCountAttr(int(sim['solver_position_iterations']))
        body.CreateSolverVelocityIterationCountAttr(int(sim['solver_velocity_iterations']))
        body.CreateLinearDampingAttr(float(dynamics['damping']['cartons']['linear_damping_s_inv']))
        body.CreateAngularDampingAttr(float(dynamics['damping']['cartons']['angular_damping_s_inv']))
        PhysxSchema.PhysxContactReportAPI.Apply(cube.GetPrim()).CreateThresholdAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
        records.append(dict(name=name,size_m=[.6,.4,.3],mass_kg=42.5))
    fixed=UsdPhysics.FixedJoint.Define(stage,'/Probe/LowerSupport')
    fixed.CreateBody1Rel().SetTargets([paths[0]]);fixed.CreateLocalPos0Attr(Gf.Vec3f(0,0,.15))
    slide=UsdPhysics.PrismaticJoint.Define(stage,'/Probe/SlideDrive')
    slide_enabled=slide.CreateJointEnabledAttr(False)
    slide.CreateBody1Rel().SetTargets([paths[1]]);slide.CreateLocalPos0Attr(Gf.Vec3f(0,0,.45))
    slide.CreateAxisAttr('X');slide.CreateLowerLimitAttr(-1.);slide.CreateUpperLimitAttr(.1)
    drive=UsdPhysics.DriveAPI.Apply(slide.GetPrim(),'linear')
    drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(20000.);drive.CreateDampingAttr(2000.)
    drive.CreateMaxForceAttr(1000.);position=drive.CreateTargetPositionAttr(0.);velocity=drive.CreateTargetVelocityAttr(0.)
    author_explicit_collision_offsets(stage,contact_offset_m=sim['contact_offset_m'],rest_offset_m=sim['rest_offset_m'])
    world=World(stage_units_in_meters=1.,physics_dt=0.,rendering_dt=dt*48)
    SimulationManager.set_physics_sim_device('cpu')
    context=world.get_physics_context();context.set_broadphase_type(sim['broadphase_type'])
    context.enable_gpu_dynamics(sim['gpu_dynamics_enabled']);context.enable_fabric(sim['fabric_enabled']);context.enable_ccd(sim['ccd_enabled'])
    scene=next(UsdPhysics.Scene(p) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    bodies=RigidPrim(paths);world.reset()
    offsets=read_effective_collision_offsets(bodies._physics_rigid_body_view)
    verify_effective_collision_offsets(offsets,contact_offset_m=sim['contact_offset_m'],rest_offset_m=sim['rest_offset_m'],expected_shape_count=2)
    shapes=verified_stack_cube_shapes(stage,records,paths,bodies._physics_rigid_body_view)
    world.set_simulation_dt(physics_dt=dt,rendering_dt=dt*48)
    def capture():
        p,q=bodies.get_world_poses();v,w=bodies.get_velocities()
        return [dict(name=r['name'],prim_path=path,center_m=p.numpy()[i].tolist(),quaternion_wxyz=q.numpy()[i].tolist(),
            linear_velocity_m_s=v.numpy()[i].tolist(),angular_velocity_rad_s=w.numpy()[i].tolist(),size_m=r['size_m'])
            for i,(r,path) in enumerate(zip(records,paths))]
    initial=capture();boxes={x['name']:OBB(x['center_m'],[.3,.2,.15],rotation_from_actual_quaternion(x['quaternion_wxyz']),x['name']) for x in initial}
    monitor=ActualStackContactMonitor(boxes['target'],[boxes['neighbor']],policy)
    gate=StackClearanceStep(shapes=shapes,target='target',neighbors=['neighbor'],policy=policy,
        world_id='stack-slide-'+str(time.time()),task_id='mechanism',maximum_wait_s=1.)
    def callback(headers,data):
        for h in headers:
            a,b,c,d=[str(PhysicsSchemaTools.intToSdfPath(getattr(h,k))) for k in ('actor0','actor1','collider0','collider1')]
            gate.collect(actor0=a,actor1=b,collider0=c,collider1=d,
                event=str(getattr(h.type,'name',h.type)),points=copy_contact_points(h,data))
    subscription=get_physx_simulation_interface().subscribe_contact_report_events(callback)
    camera=rep.create.camera(position=(-1.8,-1.7,1.3),look_at=(-.25,0,.3))
    rep.create.light(light_type='distant',intensity=2000.,rotation=(315,0,20))
    product=rep.create.render_product(camera,(640,360));annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach(product)
    writer=cv2.VideoWriter(str(args.output/'slide.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),5.,(640,360))
    assert writer.isOpened()
    rows=[];old_new_disagreements=[];support_points=0;support_response_points=0
    command_time=0.;returning=False;drive_enabled=False
    for step in range(4200):
        physical_time=step*dt
        if command_time >= .5 and not drive_enabled:
            measured=next(s for s in capture() if s['name']=='target')
            slide.CreateLocalPos0Attr(Gf.Vec3f(*measured['center_m']))
            q=measured['quaternion_wxyz'];slide.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
            slide_enabled.Set(True);drive_enabled=True
            result['drive_attachment_actual_state']=measured
        if command_time < .5:
            target_x=0.;target_v=0.
        elif not returning:
            target_x=-min(.612,(command_time-.5)*.05);target_v=-.05 if target_x>-.612 else 0.
        else:
            target_x=min(-.598,return_start_x+(command_time-return_start_time)*.02);target_v=.02
        if gate.hold:target_v=0.
        position.Set(target_x);velocity.Set(target_v)
        gate.begin_step(step=step,time_s=physical_time,trajectory_time_s=command_time,
            context=dict(stage='extraction',attached=True,actual_free_space=monitor.free_space_reached),states=capture())
        world.step(render=False,update_fabric=True)
        outcome=gate.finish_step(states=capture(),time_s=(step+1)*dt,monitor=monitor,commanded_motion=bool(target_v))
        row=gate.last
        for contact in row['contacts']:
            for p in contact['points']:
                sep=p.get('contact_point_separation_m')
                if sep is not None and sep<=.0002:support_points+=1
                if not drive_enabled and np.linalg.norm(p.get('impulse_ns',[0,0,0]))>0:support_response_points+=1
                old=classify_poc_runtime_pair(collider0=contact['collider0'],collider1=contact['collider1'],minimum_separation_m=sep,
                    policy=policy,robot_link_by_collider={},owned_tool_colliders=set(),stage='extraction')
                if sep is not None and 0<sep<.005 and row['pair_geometry']['neighbor']['surface_distance_m']>=.0052:
                    if len(old_new_disagreements)<6:old_new_disagreements.append(dict(step=step,old=old,new=row))
        if step%48==47:
            rows.append(row)
            rep.orchestrator.step(rt_subframes=1,pause_timeline=False,delta_time=0.,wait_for_render=True)
            rgba=np.asarray(annotator.get_data());writer.write(cv2.cvtColor(rgba[:,:,:3],cv2.COLOR_RGB2BGR))
        if not returning and monitor.free_space_reached and row['observation']['minimum_stack_clearance_m']>=.010:
            returning=True;return_start_time=command_time;return_start_x=target_x
        if outcome['stop_reason']:
            result['stop_reason']=outcome['stop_reason'];break
        if not outcome['hold']:command_time+=dt
    result.update(physics_seconds=(step+1)*dt,command_seconds=command_time,initial_states=initial,
        final_states=capture(),raw_support_contact_points=support_points,
        initial_support_response_points=support_response_points,entered_free_space=monitor.free_space_reached,
        returning_to_insufficient_clearance=returning,positive_feature_geometry_disagreements=old_new_disagreements,
        final_geometry=gate.last['pair_geometry'],counts=dict(gate.counts),physics_config=sim,
        effective_offsets=offsets,solver_iterations=[sim['solver_position_iterations'],sim['solver_velocity_iterations']],
        drive_scope='MECHANISM_ONLY_FINITE_FORCE_PRISMATIC_1000N_20000N_PER_M_2000NS_PER_M',
        body_pose_resets_during_motion=0,recording=dict(width=640,height=360,fps=5,physical_time_scale=1.))
    result['status']='PASS' if support_response_points and returning and result.get('stop_reason')=='ACTUAL_FREE_TRANSIT_STACK_CLEARANCE_LOST' else 'REVIEW_REQUIRED'
    print('STACK_SLIDE_RESULT='+json.dumps({k:v for k,v in result.items() if not isinstance(v,(dict,list))}),flush=True)
except BaseException as e:
    import traceback
    result.update(status='ERROR',exception=repr(e),traceback=traceback.format_exc())
    traceback.print_exc()
finally:
    if writer is not None:writer.release()
    if gate is not None:(args.output/'stack_clearance_steps.json').write_text(json.dumps(gate.evidence(),indent=2))
    if 'rows' in globals():(args.output/'sampled_steps.json').write_text(json.dumps(rows,indent=2))
    (args.output/'result.json').write_text(json.dumps(result,indent=2))
    app.close()
