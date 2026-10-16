"""External static installation/tip views of a completed sensor-capture scene."""
import argparse,time
from pathlib import Path


def review(prepared,capture,out):
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':1920,'height':1080})
    try:
        import numpy as np,cv2
        from pxr import Usd,UsdGeom,Gf
        import omni.usd,omni.timeline,omni.replicator.core as rep
        from unloading_sim.eco65.model import ROOT,load_json,save_json,robot_for
        from unloading_sim.touch_calibration import points
        from unloading_sim.eco65.native_io import file_hash
        start=time.perf_counter();out=out.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
        c=load_json(prepared/'configuration.json');s=load_json(ROOT/c['snapshot']);robot=robot_for(s['tool'],s['scene']);TWD=np.array(s['scene']['T_W_D'])
        stage_path=capture/'calibration.usda';omni.usd.get_context().open_stage(str(stage_path.resolve()))
        timeline=omni.timeline.get_timeline_interface();timeline.stop();events=[]
        subscription=timeline.get_timeline_event_stream().create_subscription_to_pop(lambda e:events.append(int(e.type)))
        for _ in range(20):app.update()
        stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer());root='/World/Workstation';records=[]
        def pose(path,T):UsdGeom.Xformable(stage.GetPrimAtPath(path)).MakeMatrixXform().Set(Gf.Matrix4d(np.array(T).T.tolist()))
        def matrix(path):return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
        def set_state(q,P):
            f=robot.named_link_frames(q)
            for name,T in f.items():pose(root+'/Robot/'+name,T)
            for p in stage.GetPrimAtPath(root+'/Tool').GetChildren():pose(str(p.GetPath()),f['link_6']@s['tool']['T_flange_tool_cad'])
            pose(root+'/CalibrationBoard',P)
            for name,T in f.items():
                for child in ['Visual','Collision']:
                    assert np.max(abs(matrix(root+'/Robot/'+name+'/'+child)-TWD@T))<1e-9
            tipmesh=UsdGeom.Mesh(stage.GetPrimAtPath(root+'/Robot/link_6/CalibrationProbe/probe_tip'))
            v=np.array(tipmesh.GetPointsAttr().Get());world=points(matrix(str(tipmesh.GetPath())),v)
            tip=points(TWD@f['link_6'],c['probe']['p_F_tip_m']);error=float(np.linalg.norm(world-tip,axis=1).min())
            assert error<1e-8
            opt=matrix(root+'/Robot/link_1/Mast/CameraMount/camera_optical');L=TWD@f['link_1'];arm=-L[:3,0];axis=opt[:3,2]
            angle=float(np.degrees(np.arctan2(np.cross(arm,axis)[2],arm@axis)))
            records.append(dict(q_rad=q,T_W_C=opt.tolist(),T_L_C=(np.linalg.inv(L)@opt).tolist(),signed_heading_deg=angle,
                optical_horizontal_error=abs(float(axis[2])),image_down_dot_world_down=float(-opt[2,1]),probe_USD_tip_error_m=error,
                robot_visual_collision_transforms='READ_BACK_SINGLE_APPLICATION'))
            return opt,tip
        placements={p['board_placement_id']:p['T_D_P'] for p in c['placements']}
        for p in c['photos']:set_state(p['q_rad'],placements[p['board_placement_id']])
        for joint in range(1,6):
            q=np.array(c['photos'][0]['q_rad']);q[joint]+=.03;set_state(q.tolist(),placements['train_01'])
        assert all(abs(r['signed_heading_deg']+90)<1e-8 and r['optical_horizontal_error']<1e-9 and abs(r['image_down_dot_world_down']-1)<1e-9 for r in records)
        camera=UsdGeom.Camera.Define(stage,'/World/TouchReviewCamera');camera.CreateFocalLengthAttr(35.);camera.CreateHorizontalApertureAttr(36.);camera.CreateVerticalApertureAttr(20.25);camera.CreateClippingRangeAttr(Gf.Vec2f(.005,100.))
        rp=rep.create.render_product(str(camera.GetPath()),(1920,1080));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp)
        def render(name,eye,target,focal):
            camera.CreateFocalLengthAttr(focal);pose(str(camera.GetPath()),np.array(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()).T)
            for _ in range(20):rep.orchestrator.step(delta_time=0.,rt_subframes=4,pause_timeline=True)
            a=rgb.get_data();a=a['data'] if isinstance(a,dict) else a;path=out/(name+'.png');cv2.imwrite(str(path),cv2.cvtColor(np.asarray(a)[...,:3],cv2.COLOR_RGB2BGR))
            return dict(name=name,sha256=file_hash(path),eye_W=eye.tolist(),target_W=target.tolist(),focal=focal)
        p=c['photos'][1];C,tip=set_state(p['q_rad'],placements[p['board_placement_id']]);images=[]
        images.append(render('01_calibration_overview',TWD[:3,3]+[-.2,1.1,1.4],TWD[:3,3]+[-.45,.1,.4],28.))
        images.append(render('02_camera_front_installation',C[:3,3]+C[:3,2]*.22+C[:3,0]*.14-C[:3,1]*.10,C[:3,3],42.))
        t=next(t for t in c['touches'] if t['split']=='train' and t['point_id']=='p0');_,tip=set_state(t['q_rad'],placements[t['board_placement_id']])
        images.append(render('03_discrete_tip_contact',tip+[.36,.28,.18],tip,42.))
        assert not timeline.is_playing() and timeline.get_current_time()==0 and int(omni.timeline.TimelineEventType.PLAY) not in events
        save_json(out/'review.json',dict(images=images,readbacks=records,seconds=time.perf_counter()-start,touch_sample=t['sample_id'],
            status='SYNTHETIC_GEOMETRIC_TOUCH',physical_contact='NOT_EVALUATED',motion='NO_MOTION_EXECUTED',timeline_time_s=0.,play_events=0))
    except Exception as error:
        import traceback,json
        out.mkdir(parents=True,exist_ok=True);(out/'failure.json').write_text(json.dumps(dict(error=str(error),traceback=traceback.format_exc())));traceback.print_exc();raise
    finally:app.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['prepared','capture','output']:p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();review(a.prepared,a.capture,a.output)
