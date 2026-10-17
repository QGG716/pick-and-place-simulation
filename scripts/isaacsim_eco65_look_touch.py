"""One continuous, finite-drive Isaac look/touch/retreat experiment."""
import argparse,json,os,time,traceback
from pathlib import Path


def run(a):
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':1280,'height':720})
    created=False;client=None;states_file=None;frames=0;result={};out=a.output.resolve();started=time.perf_counter()
    try:
        import numpy as np,cv2,subprocess
        from pxr import Usd,UsdGeom,Gf,PhysicsSchemaTools
        import omni.usd,omni.replicator.core as rep
        from omni.physx import get_physx_simulation_interface
        from isaacsim.core.api import World
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from unloading_sim.eco65.model import ROOT,load_json,save_json,transform
        from unloading_sim.eco65.native_io import file_hash
        from unloading_sim.moveit2_backend import digest
        from unloading_sim.touch_calibration import intrinsics,points
        from unloading_sim.stack_clearance import copy_contact_points
        from eco65_touch_geometry import StaticGeometry
        from eco65_touch_sources import verify_scene_sources
        from eco65_touch_native import NativeTouch
        from eco65_touch_physics import author
        from eco65_visual_touch_target import predict
        out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False);created=True
        for name in ['external_frames','camera_frames','geometry_check']:(out/name).mkdir()
        source_files=[Path(__file__).resolve(),ROOT/'tools/eco65_touch_native.py',ROOT/'tools/eco65_touch_physics.py',
            ROOT/'tools/eco65_touch_geometry.py',ROOT/'tools/eco65_visual_touch_target.py']
        save_json(out/'source_bindings.json',dict(files={p.relative_to(ROOT).as_posix():file_hash(p) for p in source_files},
            inputs={key:dict(path=str(getattr(a,key)),sha256=file_hash(getattr(a,key))) for key in
                ['configuration','native_context','scene','solver_result','validated']}))
        c=load_json(a.configuration);verify_scene_sources(c);s=load_json(ROOT/c['snapshot']);context=load_json(a.native_context)
        if file_hash(a.scene)!=a.scene_sha256:raise ValueError('PHYSICAL_SCENE_SOURCE_CHANGED')
        if context['source_configuration_sha256']!=digest(c):raise ValueError('NATIVE_CONFIGURATION_CHANGED')
        for b in context['bindings']:
            if file_hash(ROOT/b['path'])!=b['sha256']:raise ValueError('NATIVE_COLLIDER_CHANGED')
        obs=load_json(a.observations/'observations.json');solver=load_json(a.solver_result);validated=load_json(a.validated)
        if obs['identifiers']['camera']!=digest(c['camera']):raise ValueError('NEW_INSTALLATION_OBSERVATIONS_REQUIRED')
        g=StaticGeometry(s,c,out/'geometry_check');client=NativeTouch(context,c,g,out,a.worker_command)
        q0=np.array(next(p['q_rad'] for p in c['photos'] if p['sample_id']=='heldout_01_photo_1'))
        initial=client.check(q0);save_json(out/'initial_static_check.json',initial)
        if not initial['accepted']:raise ValueError('PHYSICAL_INITIAL_STATE_STATIC_REJECTED')
        thresholds=dict(end_to_end_m=.005,datum_radius_m=.004,valid_contact_surface_separation_m=.0001,
            force_stop_n=5.,max_guard_travel_m=.006,guard_speed_m_s=.002,guard_timeout_s=8.,
            approach_offset_m=.004,preapproach_m=.035,tracking_joint_rad=.04,settled_velocity_rad_s=.01,
            physics_dt_s=.001,control_hz=1000,video_fps=5,scope='Locked simulation settings; not manufacturer safety parameters')
        save_json(out/'locked_thresholds.json',thresholds)
        stage_path=out/'physics.usda';stage=Usd.Stage.CreateNew(str(stage_path));UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
        root=UsdGeom.Xform.Define(stage,'/World').GetPrim();stage.SetDefaultPrim(root);root.GetReferences().AddReference(str(a.scene.resolve()),'/World')
        physics=author(stage,a.scene.resolve(),s,c,context,g,q0,out);stage.GetRootLayer().Save()
        omni.usd.get_context().open_stage(str(stage_path))
        for _ in range(20):app.update()
        stage=omni.usd.get_context().get_stage();station='/World/Workstation';TWD=np.array(s['scene']['T_W_D']);TDW=np.linalg.inv(TWD)
        sensor=station+'/Robot/link_1/Mast/CameraMount/camera_optical/RenderCamera'
        nativecam=UsdGeom.Camera(stage.GetPrimAtPath(sensor));fov=np.degrees(2*np.arctan(np.array([nativecam.GetHorizontalApertureAttr().Get(),nativecam.GetVerticalApertureAttr().Get()])/(2*nativecam.GetFocalLengthAttr().Get())))
        if max(abs(fov-[64,64]))>1e-4:raise ValueError('SENSOR_PROJECTION_CHANGED')
        rp=rep.create.render_product(sensor,(1024,1024));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp)
        ext=rep.create.camera(position=(TWD@[-1.8,-1.9,1.3,1])[:3],look_at=(TWD@[-.50,.05,.40,1])[:3],focal_length=35)
        rp2=rep.create.render_product(ext,(1280,720));external=rep.AnnotatorRegistry.get_annotator('rgb');external.attach(rp2)
        K=np.array(obs['camera']['K']);Ks=intrinsics(1024,1024,64,64);yy,xx=np.indices((480,640),np.float32)
        mx=((xx-K[0,2])/K[0,0]*Ks[0,0]+Ks[0,2]).astype(np.float32);my=((yy-K[1,2])/K[1,1]*Ks[1,1]+Ks[1,2]).astype(np.float32)
        world=World(stage_units_in_meters=1.,physics_dt=0.,rendering_dt=1/5)
        SimulationManager.set_physics_sim_device('cpu');pc=world.get_physics_context();pc.enable_gpu_dynamics(False);pc.enable_fabric(False);pc.set_gravity(-9.81)
        robot=Articulation(physics['robot_root']);names=list(robot.dof_names);expected=g.robot.active_joint_names
        if names!=expected:raise ValueError('OFFICIAL_DOF_ORDER_MISMATCH:'+str(names))
        world.reset();robot.switch_dof_control_mode('position')
        robot.set_dof_max_efforts(np.array([physics['effort_limits_nm']],np.float32));robot.set_dof_max_velocities(np.array([physics['velocity_limits_rad_s']],np.float32))
        robot.set_dof_gains(np.array([physics['stiffness']],np.float32),np.array([physics['damping']],np.float32));robot.set_dof_position_targets(q0[None,:])
        def state():return np.array(robot.get_dof_positions().numpy()[0],float),np.array(robot.get_dof_velocities().numpy()[0],float)
        reset_q,_=state();save_json(out/'initial_physics_readback.json',dict(authored_q=q0.tolist(),actual_q=reset_q.tolist(),max_error=float(max(abs(reset_q-q0)))))
        if max(abs(reset_q-q0))>1e-5:raise ValueError('ARTICULATION_INITIALIZATION_MISMATCH')
        dt=thresholds['physics_dt_s'];world.set_simulation_dt(physics_dt=dt,rendering_dt=1/5)
        pending=[];events=[];step_index=0;phase='observation_settle';contact=None;peak_tracking=0.;peak_force=0.;current_command=q0.copy();previous_q=reset_q.copy()
        def callback(headers,data):
            for h in headers:
                pending.append(dict(colliders=[str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))],
                    actors=[str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1))],
                    event=str(getattr(h.type,'name',h.type)),points=copy_contact_points(h,data)))
        subscription=get_physx_simulation_interface().subscribe_contact_report_events(callback)
        states_file=(out/'actual_joint_feedback.jsonl').open('w',encoding='utf8')
        def render(save=True):
            nonlocal frames
            before,_=state();rep.orchestrator.step(delta_time=0.,rt_subframes=3,pause_timeline=False,wait_for_render=True)
            after,_=state()
            if max(abs(after-before))>1e-7:raise ValueError('RENDER_ADVANCED_UNCOMMANDED_PHYSICS')
            def array(ann):
                data=ann.get_data();return np.asarray(data['data'] if isinstance(data,dict) else data)[...,:3].copy()
            raw=cv2.remap(array(rgb),mx,my,cv2.INTER_LINEAR);view=array(external)
            if save:
                bgr=cv2.cvtColor(view,cv2.COLOR_RGB2BGR);cv2.rectangle(bgr,(0,650),(760,720),(20,20,20),-1)
                cv2.putText(bgr,f'PHYSICAL SIMULATION | {phase} | t={step_index*dt:.2f}s',(14,679),cv2.FONT_HERSHEY_SIMPLEX,.6,(240,240,240),1)
                cv2.putText(bgr,'Finite drives / actual feedback / no pose teleport',(14,706),cv2.FONT_HERSHEY_SIMPLEX,.55,(100,220,255),1)
                cv2.imwrite(str(out/'external_frames'/f'{frames:05d}.png'),bgr);cv2.imwrite(str(out/'camera_frames'/f'{frames:05d}.png'),cv2.cvtColor(raw,cv2.COLOR_RGB2BGR));frames+=1
            return cv2.cvtColor(raw,cv2.COLOR_RGB2BGR)
        render(False)
        def step(command,guard=False):
            nonlocal step_index,contact,peak_tracking,peak_force,current_command,previous_q
            current_command=np.array(command);robot.set_dof_position_targets(current_command[None,:]);pending.clear();world.step(render=False);step_index+=1
            result['physical_steps']=True
            q,v=state();tracking=float(max(abs(q-current_command)));peak_tracking=max(peak_tracking,tracking)
            row=dict(step=step_index,time_s=step_index*dt,phase=phase,q_rad=q.tolist(),qd_rad_s=v.tolist(),q_difference_velocity_rad_s=((q-previous_q)/dt).tolist(),command_q_rad=current_command.tolist(),tracking_error_rad=tracking,contacts=list(pending));previous_q=q.copy()
            states_file.write(json.dumps(row,allow_nan=False)+'\n')
            if not g.robot.within_limits(q) or not np.isfinite([q,v]).all():raise ValueError('ACTUAL_JOINT_LIMIT_OR_FEEDBACK')
            if max(abs(v))>.55+.02:raise ValueError('ACTUAL_JOINT_VELOCITY_LIMIT')
            if tracking>thresholds['tracking_joint_rad']:raise ValueError('ACTUAL_DRIVE_TRACKING_LIMIT')
            for event in pending:
                events.append(dict(step=step_index,phase=phase,**event));pair={physics['colliders'].get(p,{}).get('owner') for p in event['colliders']}
                for point in event['points']:
                    if point.get('invalid_contact_data'):raise ValueError('CONTACT_DATA_INVALID')
                    force=float(np.linalg.norm(point['impulse_ns'])/dt);peak_force=max(peak_force,force)
                    if point['contact_point_separation_m']>.0001 or force<1e-5:continue
                    if pair!={'probe_tip','stud_'+context['point_id']}:
                        result['non_target_contact']=True;raise ValueError('NON_TARGET_PHYSICAL_CONTACT:'+str(pair))
                    if not guard:raise ValueError('UNEXPECTED_EARLY_PROBE_CONTACT')
                    pD=points(TDW,point['position_m']);pP=points(np.linalg.inv(context['P']),pD);datum=np.array(next(p['p_P_m'] for p in c['board']['touch_points'] if p['point_id']==context['point_id']))
                    if np.linalg.norm(pP[:2]-datum[:2])>.004+.00002 or abs(pP[2]-datum[2])>.0001:raise ValueError('CONTACT_OUTSIDE_FINITE_FRONT_DATUM')
                    if contact is None:
                        actual_tip=points(g.robot.named_link_frames(q)['link_6'],c['probe']['p_F_tip_m'])
                        command_tip=points(g.robot.named_link_frames(current_command)['link_6'],c['probe']['p_F_tip_m'])
                        usd_frame=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(station+'/Robot/link_6'))).T
                        usd_tip=points(TDW@usd_frame,c['probe']['p_F_tip_m'])
                        contact=dict(step=step_index,time_s=step_index*dt,q_rad=q.tolist(),qd_rad_s=v.tolist(),command_q_rad=current_command.tolist(),
                            point_D_m=pD.tolist(),point_P_m=pP.tolist(),force_n=force,event=event,
                            actual_tip_D_m=actual_tip.tolist(),command_tip_D_m=command_tip.tolist(),USD_tip_D_m=usd_tip.tolist(),
                            tracking_tip_error_m=float(np.linalg.norm(actual_tip-command_tip)),USD_FK_tip_error_m=float(np.linalg.norm(usd_tip-actual_tip)))
                    if force>thresholds['force_stop_n']:result['force_stop_triggered']=True
            if step_index%round(1/(dt*5))==0:render()
            if step_index%round(.1/dt)==0:
                check=g.check(q,np.array(context['P']),context['point_id'] if guard else None,contact_neighborhood=guard)
                if not check['accepted']:save_json(out/'actual_geometry_rejection.json',dict(step=step_index,check=check));raise ValueError('ACTUAL_STATE_GEOMETRY_REJECTED')
            return q,v
        print('PHYSICS_STARTED',flush=True)
        for _ in range(round(1/dt)):q,v=step(q0)
        if max(abs(v))>thresholds['settled_velocity_rad_s']:raise ValueError('OBSERVATION_NOT_SETTLED')
        phase='fresh_image_capture';image=render(False);q_capture,v_capture=state();image_path=out/'fresh_test_rgb.png';cv2.imwrite(str(image_path),image)
        cache=UsdGeom.XformCache();Cactual=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(sensor.rsplit('/',1)[0]))).T
        Cexpected=TWD@g.robot.named_link_frames(q_capture)['link_1']@c['camera']['T_link1_optical']
        camera_error=float(np.max(abs(Cactual-Cexpected)));save_json(out/'physical_camera_chain.json',dict(T_W_C=Cactual.tolist(),FK_expected=Cexpected.tolist(),max_error=camera_error))
        if camera_error>.0002:raise ValueError('PHYSICAL_CAMERA_JOINT_CHAIN_MISMATCH')
        target=predict(image,q_capture,obs,solver,validated,context['point_id'],out/'fresh_test_detected.png')
        target.update(image_sha256=file_hash(image_path),capture_simulation_time_s=step_index*dt,capture_wall_time_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            validated_extrinsics_sha256=file_hash(a.validated),heldout_placement='heldout_01',q_capture_velocity_rad_s=v_capture.tolist())
        save_json(out/'frozen_visual_target.json',target);target_hash=file_hash(out/'frozen_visual_target.json')
        print('FRESH_VISUAL_TARGET_FROZEN',target['p_D_target_est'],flush=True)
        result.update(fresh_image=True,target_sha256=target_hash,physical_steps=True,contact=False,retreat=False)
        normal=np.array(target['normal_into_board_D']);p=np.array(target['p_D_target_est']);R=np.array(target['T_D_P_est'])[:3,:3]
        def follow(motion,guard=False,slow=1.,stop_on_contact=True):
            qs=np.array(motion['q']);times=np.array(motion['times'])*slow
            for k in range(int(np.ceil(times[-1]/dt))+1):
                t=min(k*dt,times[-1]);command=np.array([np.interp(t,times,qs[:,i]) for i in range(6)])
                q,v=step(command,guard)
                if guard and stop_on_contact and contact is not None:return q
            for _ in range(round((.05 if guard else .5)/dt)):
                q,v=step(qs[-1],guard)
                if guard and stop_on_contact and contact is not None:return q
            return q
        phase='native_transfer';q,_=state();motion=None;failures=[];candidates=[]
        # A half turn about the probe axis changes the wrist posture but not
        # the image-derived target or approach normal. No taught q is a seed.
        for roll in [np.eye(3),np.diag([-1.,-1.,1.])]:
            orientation=R@roll;goal=transform(p-normal*.035,orientation)
            candidates.extend((candidate,orientation) for candidate in client.ik(goal,q,'pregrasp',count=12))
        candidates.sort(key=lambda item:np.linalg.norm(item[0]-q));print('VALID_VISUAL_IK',len(candidates),flush=True)
        for candidate,orientation in candidates[:3]:
            try:motion=client.motion(q,'pregrasp',goal=candidate);R=orientation;break
            except ValueError as error:failures.append(str(error))
        if motion is None:save_json(out/'transfer_failure.json',dict(valid_ik_candidates=len(candidates),attempts=failures));raise ValueError('BOUNDED_VISUAL_TRANSFER_FAILED')
        q=follow(motion);phase='controlled_preapproach'
        motion=client.motion(q,'contact',pose=transform(p-normal*.004,R));q=follow(motion,True)
        guard_start=step_index*dt;guard_commands=[];phase='guarded_touch'
        if contact is None:
            for travel in np.arange(.0005,.00601,.0005):
                if step_index*dt-guard_start>thresholds['guard_timeout_s']:raise ValueError('GUARD_TIMEOUT')
                q,_=state();goal=transform(p-normal*(.004-travel),R);guard_commands.append(goal.tolist())
                motion=client.motion(q,'contact',pose=goal);q=follow(motion,True,slow=8.)
                if contact is not None:break
        save_json(out/'guard_commands.json',guard_commands)
        if contact is None:raise ValueError('BOUNDED_GUARD_NO_VALID_CONTACT')
        result['contact']=True;save_json(out/'first_contact.json',contact);phase='contact_stop'
        q_stop,_=state()
        for _ in range(round(.4/dt)):step(q_stop,True)
        phase='retreat_from_actual_stop';q_stop,_=state();F=g.robot.named_link_frames(q_stop)['link_6'];actual_tip=points(F,c['probe']['p_F_tip_m'])
        retreat=transform(actual_tip-normal*.035,F[:3,:3]);motion=client.motion(q_stop,'withdrawal',pose=retreat)
        q=follow(motion,True,stop_on_contact=False);result['retreat']=True
        if file_hash(out/'frozen_visual_target.json')!=target_hash:raise ValueError('VISUAL_TARGET_MUTATED')
        datum=np.array(next(p['p_P_m'] for p in c['board']['touch_points'] if p['point_id']==context['point_id']))
        true_target=points(context['P'],datum);contact_error=np.array(contact['point_D_m'])-true_target;tip_at_contact=points(g.robot.named_link_frames(contact['q_rad'])['link_6'],c['probe']['p_F_tip_m'])
        result.update(status='PASS' if np.linalg.norm(contact_error)<=.005 and not result.get('force_stop_triggered',False) else 'FAIL',visual_prediction_error_m=float(np.linalg.norm(p-true_target)),
            contact_centre_error_m=float(np.linalg.norm(contact_error)),contact_normal_error_m=float(contact_error@np.array(context['P'])[:3,2]),
            contact_tangential_error_m=float(np.linalg.norm((np.array(context['P'])[:3,:3].T@contact_error)[:2])),
            actual_tip_at_contact_D_m=tip_at_contact.tolist(),peak_joint_tracking_error_rad=peak_tracking,peak_reported_force_n=peak_force,
            non_target_contact=False,retreat_actual_q=q.tolist(),actual_tip_tracking_error_at_contact_m=contact['tracking_tip_error_m'])
    except Exception as error:
        result.update(status='FAIL',error=str(error),traceback=traceback.format_exc());traceback.print_exc()
    finally:
        if states_file:states_file.flush();states_file.close()
        if client:client.close()
        if created:
            result.update(scope='ONE_PHYSICAL_SIMULATION_TEST',hardware='NOT_CONNECTED',per_frame_teleport=False,video_fps=5,frames=frames)
            result.update(wall_seconds=time.perf_counter()-started,final_phase=locals().get('phase','initialization'),simulation_time_s=locals().get('step_index',0)*locals().get('dt',0),
                peak_joint_tracking_error_rad=locals().get('peak_tracking'),peak_reported_force_n=locals().get('peak_force'))
            if 'target' in locals():
                datum=next(p['p_P_m'] for p in c['board']['touch_points'] if p['point_id']==context['point_id'])
                truth=points(context['P'],datum)
                result['visual_prediction_error_m']=float(np.linalg.norm(np.asarray(target['p_D_target_est'])-truth))
            if 'contact' in locals() and contact is not None:
                save_json(out/'first_contact.json',contact)
                result['contact']=True
            if 'events' in locals():save_json(out/'contact_events.json',events)
            if frames:
                videos=[]
                for folder,name in [('external_frames','look_then_touch_external.mp4'),('camera_frames','look_then_touch_camera_rgb.mp4')]:
                    command=['ffmpeg','-hide_banner','-loglevel','error','-framerate','5','-i',str(out/folder/'%05d.png'),'-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(out/name)]
                    code=subprocess.run(command).returncode
                    videos.append(dict(file=name,exit_code=code,sha256=file_hash(out/name) if code==0 else None))
                result['videos']=videos
            save_json(out/'test_report.json',result)
        if result.get('status')!='PASS':
            import sys
            sys.stdout.flush();sys.stderr.flush();os._exit(1)
        app.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['configuration','native-context','scene','observations','solver-result','validated','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--worker-command',required=True);p.add_argument('--scene-sha256',required=True);run(p.parse_args())
