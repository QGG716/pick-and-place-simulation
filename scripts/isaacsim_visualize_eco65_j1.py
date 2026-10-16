"""Actual mast-camera video and layout readback, independent of calibration gates.

This entry only writes FK transforms into a fresh USD derivative and renders with
the physical timeline stopped. It does not qualify motion, clearances or hardware.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import time


def run(args):
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'width': 1920, 'height': 1080})
    out = args.output.resolve()
    created = False
    started = time.perf_counter()
    try:
        import cv2
        import numpy as np
        import trimesh
        import xml.etree.ElementTree as ET
        import importlib.metadata
        from pxr import Usd, UsdGeom, Gf
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from unloading_sim.eco65.model import ROOT, URDF, OFFICIAL, load_json, save_json, robot_for
        from unloading_sim.eco65.native_io import file_hash
        from unloading_sim.touch_calibration import intrinsics, OPTICAL_FROM_USD
        from eco65_touch_sources import verify_scene_sources
        from eco65_j1_visualization import face_angle, schedule

        out.relative_to(ROOT/'outputs')
        out.mkdir(parents=True, exist_ok=False)
        created = True
        for sub in ['camera_frames', 'external_frames', 'keyframes', 'annotated']:
            (out/sub).mkdir()
        c = load_json(args.configuration)
        sources = verify_scene_sources(c)
        # Bind the actual FK/mesh inputs, without requiring unrelated robot
        # variants, manufacturing drawings or unused source CAD on the worker.
        official_manifest=load_json(OFFICIAL/'SOURCE_MANIFEST.json')
        used_official=[URDF]+list((URDF.parents[1]/'meshes').glob('*.STL'))
        expected_hashes={e['local_path']:e['sha256'] for e in official_manifest['entries']}
        for path in used_official:
            if file_hash(path)!=expected_hashes[path.relative_to(OFFICIAL).as_posix()]:
                raise ValueError('OFFICIAL_FK_OR_MESH_CHANGED:'+str(path))
        if len(used_official)!=8:raise ValueError('OFFICIAL_ECO65_B_MESH_SET_INCOMPLETE')
        source = args.scene.resolve()
        if file_hash(source) != args.scene_sha256:
            raise ValueError('SELECTED_CAMERA_USD_CHANGED')
        s = load_json(ROOT/c['snapshot'])
        robot = robot_for(s['tool'], s['scene'])
        TWD = np.array(s['scene']['T_W_D']); TDW = np.linalg.inv(TWD)
        q_fixed = np.array(c['display_q'])
        station = '/World/Workstation'
        optical = station+'/Robot/link_1/Mast/CameraMount/camera_optical'
        sensor_path = optical+'/RenderCamera'
        stage_path = out/'visualization.usda'
        stage = Usd.Stage.CreateNew(str(stage_path))
        UsdGeom.SetStageMetersPerUnit(stage, 1.); UsdGeom.SetStageUpAxis(stage, 'Z')
        root = UsdGeom.Xform.Define(stage, '/World').GetPrim(); stage.SetDefaultPrim(root)
        root.GetReferences().AddReference(Path(os.path.relpath(source,out)).as_posix(), '/World')

        def matrix(path):
            p = stage.GetPrimAtPath(path)
            if not p:
                raise ValueError('REQUIRED_PRIM_MISSING:'+path)
            return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(p)).T

        def local(path):
            return np.array(UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetLocalTransformation()).T

        def pose(path, T):
            p = stage.GetPrimAtPath(path) or UsdGeom.Xform.Define(stage,path).GetPrim()
            if not p.GetTypeName():
                p = UsdGeom.Xform.Define(stage,path).GetPrim()
            UsdGeom.Xformable(p).MakeMatrixXform().Set(Gf.Matrix4d(np.asarray(T).T.tolist()))

        def bound(path, purposes=('default','render','proxy','guide')):
            p = stage.GetPrimAtPath(path)
            if not p:
                raise ValueError('BOUND_PRIM_MISSING:'+path)
            b = UsdGeom.BBoxCache(Usd.TimeCode.Default(),list(purposes),useExtentsHint=False).ComputeWorldBound(p).ComputeAlignedRange()
            return np.array([b.GetMin(),b.GetMax()])

        def in_D(b):
            vertices=np.array([[x,y,z,1] for x in b[:,0] for y in b[:,1] for z in b[:,2]])
            points=(vertices@TDW.T)[:,:3]
            return np.array([points.min(axis=0),points.max(axis=0)])

        used = {str(Path(layer.realPath).resolve()) for layer in stage.GetUsedLayers() if layer.realPath}
        if str((ROOT/sources['files']['composed_usd']['path']).resolve()) not in used:
            raise ValueError('SELECTED_CAMERA_USD_NOT_DERIVED_FROM_BOUND_SCENE')
        assert np.max(abs(matrix(station)-TWD)) < 1e-10
        X = np.linalg.inv(matrix(station+'/Robot/link_1')) @ matrix(optical)
        if np.max(abs(X-c['camera']['T_link1_optical'])) > 1e-9:
            raise ValueError('SELECTED_CAMERA_MOUNT_NOT_BOUND_NOMINAL_CAMERA')
        assert np.max(abs(local(sensor_path)-OPTICAL_FROM_USD)) < 1e-12
        for part in c['camera']['parts']:
            path = optical.rsplit('/',1)[0]+'/CameraBody/'+part['id']
            actual = np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(path)).GetPointsAttr().Get())
            expected = np.asarray(trimesh.load(ROOT/part['visual'],process=False).vertices)
            if actual.shape != expected.shape or np.max(abs(actual-expected)) > 1e-7:
                raise ValueError('SELECTED_CAMERA_GEOMETRY_MISMATCH:'+part['id'])
            expected_T = matrix(station+'/Robot/link_1/Mast')@c['camera']['T_mast_CAD']
            assert np.max(abs(matrix(path)-expected_T)) < 1e-9
        for pid in c['camera']['replaced_mast_parts']:
            assert not stage.GetPrimAtPath(station+'/Robot/link_1/Mast/visual_'+pid).IsActive()

        boxes=[]
        for b in s['scene']['boxes']:
            path=station+'/Cartons/'+b['id']; proxy=in_D(bound(path+'/Collision'))
            visual=in_D(bound(path,('default','render')))
            expected=np.array(b['pose'])+np.array([-.5,.5])[:,None]*b['size_m']
            boxes.append(dict(id=b['id'],center_D_m=(TDW@matrix(path))[:3,3].tolist(),
                              rotation_D=(TDW@matrix(path))[:3,:3].tolist(),size_m=(proxy[1]-proxy[0]).tolist(),
                              collision_bounds_D_m=proxy.tolist(),visual_bounds_D_m=visual.tolist(),
                              snapshot_bounds_D_m=expected.tolist(),max_snapshot_delta_m=float(np.max(abs(proxy-expected))),
                              visual_proxy_bound_delta_m=float(np.max(abs(visual-proxy)))))
        assert len(boxes)==12 and max(b['max_snapshot_delta_m'] for b in boxes)<1e-7
        stack_bounds=np.array([np.min([b['collision_bounds_D_m'][0] for b in boxes],axis=0),
                               np.max([b['collision_bounds_D_m'][1] for b in boxes],axis=0)])
        target=stack_bounds.mean(axis=0)
        joint=ET.parse(URDF).getroot().find("joint[@name='joint_1']/limit")
        limits=[float(joint.get(k)) for k in ('lower','upper')]
        face,away,solve=face_angle(robot,q_fixed,X,target,limits)
        velocity=float(joint.get('velocity'))
        if 'joint_velocity_rad_s' in s['scene']:
            velocity=min(velocity,float(s['scene']['joint_velocity_rad_s']))
        rows,timing=schedule(face,away,fps=args.fps,turn=args.turn_seconds,velocity=velocity)
        print('J1_ENDPOINTS_DEG',np.degrees([face,away]).tolist(),'FRAMES',len(rows),flush=True)
        moving=[station+'/Robot/link_'+str(i) for i in range(1,7)]+[station+'/Tool',station+'/Adapter',station+'/Frames']
        static={str(p.GetPath()):matrix(str(p.GetPath())) for p in stage.Traverse()
                if p.IsA(UsdGeom.Xformable) and not any(str(p.GetPath())==x or str(p.GetPath()).startswith(x+'/') for x in moving)}
        mount_locals={str(p.GetPath()):local(str(p.GetPath())) for prefix in [station+'/Robot/link_1/Mast',station+'/Robot/link_1/MountingSeat',station+'/Robot/link_6/CalibrationProbe']
                      for p in Usd.PrimRange(stage.GetPrimAtPath(prefix)) if p and p.IsA(UsdGeom.Xformable)}
        initial_board=matrix(station+'/CalibrationBoard')
        max_errors=dict(camera_link1=0.,FK=0.,static_transform=0.,mount_local=0.,horizontal=0.,upright=0.)

        def set_state(q):
            if not robot.within_limits(np.asarray(q)):
                raise ValueError('VISUALIZATION_JOINT_LIMIT')
            frames=robot.named_link_frames(q)
            for name,T in frames.items():
                pose(station+'/Robot/'+name,T)
            for p in stage.GetPrimAtPath(station+'/Tool').GetChildren():
                pose(str(p.GetPath()),frames['link_6']@s['tool']['T_flange_tool_cad'])
            if stage.GetPrimAtPath(station+'/Adapter'):
                for p in stage.GetPrimAtPath(station+'/Adapter').GetChildren():
                    pose(str(p.GetPath()),frames['link_6'])
            for name,T in dict(frames,flange=frames['link_6'],tcp=robot.fk(q),tool_cad=frames['link_6']@s['tool']['T_flange_tool_cad']).items():
                if stage.GetPrimAtPath(station+'/Frames/'+name):pose(station+'/Frames/'+name,T)
            C=matrix(optical);L=matrix(station+'/Robot/link_1')
            errors=dict(camera_link1=float(np.max(abs(np.linalg.inv(L)@C-X))),
                        FK=max(float(np.max(abs(matrix(station+'/Robot/'+name+'/'+child)-TWD@T))) for name,T in frames.items() for child in ['Visual','Collision']),
                        static_transform=max(float(np.max(abs(matrix(path)-T))) for path,T in static.items()),
                        mount_local=max(float(np.max(abs(local(path)-T))) for path,T in mount_locals.items()),
                        horizontal=abs(float(C[2,2])),upright=abs(float(C[2,1]+1)))
            for name,value in errors.items():
                max_errors[name]=max(max_errors[name],value)
            if max(errors.values())>1e-8:
                raise ValueError('KINEMATIC_READBACK_FAILED:'+str(errors))
            return C

        q=np.array(q_fixed);q[0]=face;C=set_state(q)
        conveyors=[]
        for item in s['scene']['conveyors']['items']:
            path=station+'/'+item['id'];bb=in_D(bound(path));belt=in_D(bound(path+'/Belt/SM_ConveyorBelt_A06_Belt_02'))
            conveyors.append(dict(id=item['id'],bounds_D_m=bb.tolist(),belt_bounds_D_m=belt.tolist(),
                                  belt_height_W_m=float(bound(path+'/Belt/SM_ConveyorBelt_A06_Belt_02')[1,2]),
                                  snapshot_delta_m=float(np.max(abs(bb-item['bounds_D_m']))),
                                  nominal_direction_D=item['direction_D'],velocity_m_s=0.,direction_source='snapshot configuration'))
        layout=dict(schema='eco65_j1_visual_layout_v1',status='KINEMATIC_VISUALIZATION',source_commit=args.commit,
                    configuration=dict(path=args.configuration.as_posix(),sha256=file_hash(args.configuration)),scene_sources=sources,
                    selected_camera_USD=dict(path=source.relative_to(ROOT).as_posix(),sha256=file_hash(source)),
                    rendered_USD=stage_path.relative_to(ROOT).as_posix(),snapshot_fingerprint=s['fingerprint'],
                    units=UsdGeom.GetStageMetersPerUnit(stage),up_axis=UsdGeom.GetStageUpAxis(stage),T_W_D_readback=matrix(station).tolist(),
                    table=dict(bounds_D_m=in_D(bound(station+'/trailer_floor')).tolist(),height_W_m=float(bound(station+'/trailer_floor')[1,2])),
                    base_T_D=(TDW@matrix(station+'/Robot/base_link')).tolist(),q_face_rad=q.tolist(),display_q_source='configuration.display_q; only J1 replaced by optical orbit alignment',
                    q_face=face,q_away=away,joint1_limits=limits,face_solver=solve,boxes=boxes,stack_bounds_D_m=stack_bounds.tolist(),
                    conveyors=conveyors,roof_present=bool(stage.GetPrimAtPath(station+'/trailer_roof')),
                    mast_parent=station+'/Robot/link_1',camera=dict(prim=sensor_path,T_W_C=C.tolist(),T_L_C=X.tolist(),
                        output_resolution=[640,480],fov_deg=[64,50],optical_assumption=c['camera']['optical_assumption']),
                    temporary_attachments=dict(board_T_W=initial_board.tolist(),board_bounds_W_m=bound(station+'/CalibrationBoard').tolist(),
                        probe_parent=station+'/Robot/link_6',probe_bounds_W_m=bound(station+'/Robot/link_6/CalibrationProbe').tolist(),
                        support='Existing fixed simulation fixture; preserved from selected camera USD'),
                    known_issue=dict(pair=['probe_bracket','part_001'],prior_clearance_m=.00099999918447,required_clearance_m=.0024,
                        status='UNRESOLVED; previous static evidence, not requalified here'),
                    measurement_sources='Bounds, poses and dimensions are final composed USD readback; snapshot deltas retained. Direction and timing are configuration. Optical pupil and board support are simulation assumptions.',
                    motion_safety='NOT_EVALUATED',dynamics='NOT_EVALUATED',timing=timing)
        save_json(out/'layout.json',layout)
        stage.GetRootLayer().Save()
        timeline=omni.timeline.get_timeline_interface();timeline.stop();events=[]
        subscription=timeline.get_timeline_event_stream().create_subscription_to_pop(lambda event:events.append(int(event.type)))
        omni.usd.get_context().open_stage(str(stage_path))
        for _ in range(25):app.update()
        stage=omni.usd.get_context().get_stage();timeline.stop()
        # Freeze the derivative at q_face. Per-frame changes are session opinions.
        stage.SetEditTarget(stage.GetSessionLayer())
        cam=UsdGeom.Camera(stage.GetPrimAtPath(sensor_path))
        apertures=[cam.GetHorizontalApertureAttr().Get(),cam.GetVerticalApertureAttr().Get()]
        native_fov=np.degrees(2*np.arctan(np.array(apertures)/(2*cam.GetFocalLengthAttr().Get())))
        assert np.max(abs(native_fov-[64,64]))<1e-4
        K=np.array(c['camera']['K']);Ks=intrinsics(1024,1024,*native_fov)
        assert np.max(abs(K-intrinsics(640,480,64,50)))<1e-9
        yy,xx=np.indices((480,640),np.float32)
        mapx=((xx-K[0,2])/K[0,0]*Ks[0,0]+Ks[0,2]).astype(np.float32)
        mapy=((yy-K[1,2])/K[1,1]*Ks[1,1]+Ks[1,2]).astype(np.float32)
        rp=rep.create.render_product(sensor_path,(1024,1024));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp)
        ext=UsdGeom.Camera.Define(stage,'/World/J1VisualizationExternalCamera')
        ext.CreateHorizontalApertureAttr(36.);ext.CreateVerticalApertureAttr(20.25);ext.CreateFocalLengthAttr(35.)
        ext.CreateClippingRangeAttr(Gf.Vec2f(.005,100.))
        external_rp=rep.create.render_product(str(ext.GetPath()),(1920,1080));external_rgb=rep.AnnotatorRegistry.get_annotator('rgb');external_rgb.attach(external_rp)

        def rgb_array(annotator):
            data=annotator.get_data();data=data['data'] if isinstance(data,dict) else data
            return np.asarray(data)[...,:3].copy()

        def render(steps):
            for _ in range(steps):rep.orchestrator.step(delta_time=0.,rt_subframes=4,pause_timeline=True)
            native=rgb_array(rgb)
            return native,cv2.remap(native,mapx,mapy,cv2.INTER_LINEAR),rgb_array(external_rgb)

        def write_rgb(path,a):
            if not cv2.imwrite(str(path),cv2.cvtColor(a,cv2.COLOR_RGB2BGR)):
                raise IOError('IMAGE_WRITE_FAILED:'+str(path))

        def view(eye_D,target_D,*,ortho=None,up=(0,0,1),focal=35.):
            eye=(TWD@[*eye_D,1])[:3];target_W=(TWD@[*target_D,1])[:3]
            T=np.array(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target_W),Gf.Vec3d(*up)).GetInverse()).T
            pose(str(ext.GetPath()),T)
            ext.CreateProjectionAttr('orthographic' if ortho else 'perspective');ext.CreateFocalLengthAttr(focal)
            # USD orthographic aperture is in tenths of a scene unit.
            ext.CreateHorizontalApertureAttr(ortho*10 if ortho else 36.)
            ext.CreateVerticalApertureAttr(ortho*10*1080/1920 if ortho else 20.25)
            return dict(T_W_view=T.tolist(),projection='orthographic' if ortho else 'perspective',horizontal_aperture=ext.GetHorizontalApertureAttr().Get(),
                        vertical_aperture=ext.GetVerticalApertureAttr().Get(),focal_length=focal,resolution=[1920,1080])

        # Only diagnostic external cameras use look-at. Sensor extrinsics untouched.
        camera_D=(TDW@C)[:3,3]
        views=[('layout_overview',[-2.1,-2.7,1.9],[-.22,-.02,.24],{}),
               ('layout_top',[-.3,0,3.],[-.3,0,0],dict(ortho=3.85,up=(1,0,0))),
               ('layout_front',[-3,0,.29],[.33,0,.29],dict(ortho=2.8)),
               ('layout_side',[-.3,-3,.13],[-.3,0,.13],dict(ortho=3.25)),
               ('camera_mount_closeup',(camera_D+np.array([.85,-1.1,.25])).tolist(),
                ((camera_D+np.array(s['scene']['base_position_m']))/2).tolist(),dict(focal=32.))]
        view_records=[]
        for name,eye,target_D,kwargs in views:
            record=view(eye,target_D,**kwargs);_,_,external=render(18)
            write_rgb(out/(name+'.png'),external)
            record.update(name=name,sha256=file_hash(out/(name+'.png')));view_records.append(record)
            print('LAYOUT_RENDERED',name,flush=True)
        save_json(out/'view_cameras.json',view_records)
        if args.layout_only:
            assert not timeline.is_playing() and timeline.get_current_time()==0.
            assert int(omni.timeline.TimelineEventType.PLAY) not in events
            save_json(out/'layout_check.json',dict(status='ACTUAL_ISAAC_LAYOUT_RENDERED',max_readback_errors=max_errors,
                      physical_timeline_time_s=0.,play_events=0,seconds=time.perf_counter()-started,
                      isaac_version=importlib.metadata.version('isaacsim'),motion_safety='NOT_EVALUATED',dynamics='NOT_EVALUATED'))
            print('LAYOUT_ONLY_COMPLETE',out,flush=True)
            return
        external_view=view([-1.55,-1.95,1.35],[-.25,.0,.4],focal=35.)
        render(18)
        keyframes={}
        for phase in ['face_1','away_1','face_2','away_2']:
            indices=[r['frame'] for r in rows if r['phase']==phase]
            keyframes[indices[len(indices)//2]]=phase
        readbacks=[];hashes=[]
        fieldnames=['frame','replay_time_s','phase']+['q'+str(i)+'_rad' for i in range(1,7)]+['T_W_C_'+str(i)+str(j) for i in range(4) for j in range(4)]
        with (out/'frames.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=fieldnames);writer.writeheader()
            for row in rows:
                q=np.array(q_fixed);q[0]=row['q1'];C=set_state(q)
                native,a,external=render(args.steps_per_frame)
                write_rgb(out/'camera_frames'/f"{row['frame']:05d}.png",a)
                write_rgb(out/'external_frames'/f"{row['frame']:05d}.png",cv2.resize(external,(1280,720),interpolation=cv2.INTER_AREA))
                h=file_hash(out/'camera_frames'/f"{row['frame']:05d}.png");hashes.append(h)
                record=dict(frame=row['frame'],replay_time_s=row['replay_time_s'],phase=row['phase'],q_rad=q.tolist(),T_W_C=C.tolist(),sha256=h)
                readbacks.append(record)
                writer.writerow(dict(frame=row['frame'],replay_time_s=row['replay_time_s'],phase=row['phase'],
                                     **{'q'+str(i+1)+'_rad':float(v) for i,v in enumerate(q)},
                                     **{'T_W_C_'+str(i)+str(j):float(C[i,j]) for i in range(4) for j in range(4)}))
                f.flush()
                if row['frame'] in keyframes:
                    name=keyframes[row['frame']];write_rgb(out/'keyframes'/(name+'.png'),a)
                    write_rgb(out/'keyframes'/(name+'_native.png'),native)
                if row['frame']%10==0:print('FRAME',row['frame'],'/',len(rows),'REPLAY',row['replay_time_s'],'J1_DEG',float(np.degrees(q[0])),flush=True)
        render_seconds=time.perf_counter()-started
        save_json(out/'frame_readbacks.json',readbacks)
        pairs=[]
        for a,b in [('face_1','face_2'),('away_1','away_2')]:
            i=cv2.imread(str(out/'keyframes'/(a+'.png')));j=cv2.imread(str(out/'keyframes'/(b+'.png')))
            pairs.append(dict(pair=[a,b],mean_absolute_RGB_difference=float(np.mean(abs(i.astype(float)-j))),
                              normalized_MAE=float(np.mean(abs(i.astype(float)-j))/255)))
        assert all(p['normalized_MAE']<.03 for p in pairs)
        assert len(set(hashes))>50, 'TURNING_IMAGES_NOT_UPDATING'
        for phase in ['turn_1','return','turn_2']:
            hs={hashes[r['frame']] for r in rows if r['phase']==phase}
            assert len(hs)>.8*sum(r['phase']==phase for r in rows), 'CACHED_TURN_FRAMES'
        assert not timeline.is_playing() and timeline.get_current_time()==0.
        assert int(omni.timeline.TimelineEventType.PLAY) not in events
        videos=[]
        for directory,name,resolution in [('camera_frames','camera_rgb_raw.mp4',[640,480]),('external_frames','j1_external_reference.mp4',[1280,720])]:
            command=['ffmpeg','-hide_banner','-loglevel','error','-framerate',str(args.fps),'-i',str(out/directory/'%05d.png'),
                     '-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(out/name)]
            subprocess.run(command,check=True)
            cap=cv2.VideoCapture(str(out/name));count=0
            fps=cap.get(cv2.CAP_PROP_FPS)
            while True:
                ok,frame=cap.read()
                if not ok:break
                assert [frame.shape[1],frame.shape[0]]==resolution
                count+=1
            cap.release()
            assert count==len(rows) and abs(fps-args.fps)<1e-6
            videos.append(dict(file=name,sha256=file_hash(out/name),decoded_frames=count,fps=fps,duration_s=count/fps,resolution=resolution,command=command))
        held={phase:readbacks[index] for index,phase in keyframes.items()}
        save_json(out/'check_report.json',dict(status='RENDERED_AND_DECODED',mode='KINEMATIC_VISUALIZATION',motion_safety='NOT_EVALUATED',dynamics='NOT_EVALUATED',
            isaac_version=importlib.metadata.version('isaacsim'),sensor_prim=sensor_path,native_resolution=[1024,1024],native_FOV_deg=native_fov.tolist(),output_FOV_deg=[64,50],
            steps_per_frame=args.steps_per_frame,physical_timeline_time_s=timeline.get_current_time(),play_events=0,timing=timing,
            max_readback_errors=max_errors,q2_q6_max_change=float(np.max(np.ptp(np.array([r['q_rad'][1:] for r in readbacks]),axis=0))),
            endpoint_delta_deg=float(np.degrees(away-face)),return_pose_max_error=float(np.max(abs(np.array(held['face_1']['T_W_C'])-held['face_2']['T_W_C']))),
            unique_camera_images=len(set(hashes)),repeat_image_comparison=pairs,videos=videos,external_camera=external_view,
            render_seconds=render_seconds,total_seconds=time.perf_counter()-started,
            limitations=['Known probe bracket/tool gap below policy; unchanged.','No collision or motion safety qualification.','Camera stays horizontal; stack coverage must be visually reviewed.','Temporary board and all occluders retained.']))
        print('J1_VISUALIZATION_COMPLETE',out,flush=True)
    except Exception as error:
        import traceback
        if created:
            (out/'failure.json').write_text(json.dumps(dict(error=str(error),traceback=traceback.format_exc()),indent=2),encoding='utf-8')
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['configuration','scene','output']:p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--scene-sha256',required=True);p.add_argument('--commit',required=True)
    p.add_argument('--fps',type=int,default=5);p.add_argument('--turn-seconds',type=float,default=6.)
    p.add_argument('--steps-per-frame',type=int,default=6)
    p.add_argument('--layout-only',action='store_true',help='Render/read back layout only; no replay or video')
    run(p.parse_args())
