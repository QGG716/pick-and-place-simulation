"""Stopped-timeline RGB/depth acquisition from the actual link_1 camera prim."""
import argparse,time,os
from pathlib import Path


def capture(prepared,output,perturb=False):
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':1024,'height':1024})
    try:
        import numpy as np,cv2,trimesh
        from scipy.spatial.transform import Rotation
        from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,Gf,Sdf,Vt
        import omni.usd,omni.timeline,omni.replicator.core as rep
        from unloading_sim.eco65.model import ROOT,load_json,save_json,robot_for,transform
        from unloading_sim.eco65.native_io import file_hash
        from unloading_sim.moveit2_backend import digest
        from unloading_sim.touch_calibration import intrinsics,OPTICAL_FROM_USD
        from eco65_touch_geometry import accessory_meshes
        from eco65_touch_sources import verify_scene_sources
        started=time.perf_counter();prepared=prepared.resolve();output=output.resolve();output.relative_to(ROOT/'outputs');output.mkdir(parents=True,exist_ok=False)
        c=load_json(prepared/'configuration.json');s=load_json(ROOT/c['snapshot']);robot=robot_for(s['tool'],s['scene'])
        sources=verify_scene_sources(c)
        if file_hash(ROOT/c['snapshot'])!=c['snapshot_sha256']:raise ValueError('SNAPSHOT_CHANGED')
        if len(c.get('touches',[]))<9:raise ValueError('PREPARED_TOUCH_SAMPLES_REQUIRED')
        if any(not t['geometry']['accepted'] for t in c['touches']) or any(not p['static_check']['accepted'] for p in c['photos']):
            raise ValueError('DISCRETE_STATIC_GEOMETRY_NOT_ACCEPTED')
        observations=output/'observations';observations.mkdir();(output/'truth').mkdir();(observations/'images').mkdir()
        source=ROOT/sources['files']['composed_usd']['path'];stage_path=output/'calibration.usda';stage=Usd.Stage.CreateNew(str(stage_path))
        UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
        root=UsdGeom.Xform.Define(stage,'/World').GetPrim();stage.SetDefaultPrim(root)
        root.GetReferences().AddReference(Path(os.path.relpath(source,output)).as_posix(),'/World')
        station='/World/Workstation';mast=station+'/Robot/link_1/Mast';mount=mast+'/CameraMount';optical=mount+'/camera_optical';camera_path=optical+'/RenderCamera'
        def pose(path,T):
            prim=stage.GetPrimAtPath(path) or UsdGeom.Xform.Define(stage,path).GetPrim()
            if not prim.GetTypeName():prim=UsdGeom.Xform.Define(stage,path).GetPrim()
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(np.asarray(T).T.tolist()));return prim
        def material(name,color,emission=False):
            mat=UsdShade.Material.Define(stage,'/World/CalibrationMaterials/'+name);shader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Shader');shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*([0,0,0] if emission else color)))
            if emission:shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.7);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');return mat
        def mesh(path,m,mat,T=np.eye(4),collision=False):
            obj=UsdGeom.Mesh.Define(stage,path);obj.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(m.vertices,np.float32)))
            obj.CreateFaceVertexCountsAttr([3]*len(m.faces));obj.CreateFaceVertexIndicesAttr(np.asarray(m.faces).ravel().tolist());obj.CreateSubdivisionSchemeAttr('none')
            pose(path,T);UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
            if collision:UsdPhysics.CollisionAPI.Apply(obj.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr('convexHull')
            return obj
        for pid in c['camera']['replaced_mast_parts']:
            for prefix in ['visual_','collision_']:stage.OverridePrim(mast+'/'+prefix+pid).SetActive(False)
        stage.OverridePrim(mast+'/CameraFrame').SetActive(False)
        M=transform([.113,-.00405,.410]);pose(mount,M)
        for p in c['camera']['parts']:
            if file_hash(ROOT/p['visual'])!=p['visual_sha256']:raise ValueError('CAMERA_MESH_CHANGED')
            mat=material(p['id'],p['rgb'] or [.3,.3,.3])
            mesh(mount+'/CameraBody/'+p['id'],trimesh.load(ROOT/p['visual'],process=False),mat,np.linalg.inv(M)@c['camera']['T_mast_CAD'])
            col=mesh(mount+'/CameraCollision/'+p['id'],trimesh.load(ROOT/p['collision'],process=False),mat,np.linalg.inv(M)@c['camera']['T_mast_CAD'],True)
            col.CreatePurposeAttr('guide')
        X=np.array(c['camera']['T_link1_optical']);delta=np.eye(4)
        if perturb:delta=transform([.008,-.005,.006],Rotation.from_euler('xyz',[2,-1,1.5],degrees=True).as_matrix())
        X=X@delta;local=np.linalg.inv(M)@np.linalg.inv(s['scene']['mast']['T_link1_mast'])@X;pose(optical,local)
        cam=UsdGeom.Camera.Define(stage,camera_path);pose(camera_path,OPTICAL_FROM_USD)
        cam.CreateFocalLengthAttr(1.);aperture=float(2*np.tan(np.radians(32)))
        cam.CreateHorizontalApertureAttr(aperture);cam.CreateVerticalApertureAttr(aperture);cam.CreateClippingRangeAttr(Gf.Vec2f(.001,100.))
        cam.GetPrim().SetCustomDataByKey('projection_units','dimensionless ratio; not manufacturer sensor/focal specification')
        materials={k:material(k,color) for k,color in [('probe',[.3,.4,.5]),('fixture',[.7,.7,.7]),('stud',[.9,.5,.1])]}
        extras=accessory_meshes(s,c)
        for row in extras:
            if row['category']=='probe':mesh(station+'/Robot/link_6/CalibrationProbe/'+row['name'],row['mesh'],materials['probe'],collision=True)
            if row['category']=='fixture':mesh(station+'/CalibrationBoard/'+row['name'],row['mesh'],materials['stud' if row['name'].startswith('stud') else 'fixture'],collision=True)
        # Exact planar printed pattern in P, with OpenCV runtime +X right/+Y down.
        w,h=c['board']['pattern_size_m'];pattern=UsdGeom.Mesh.Define(stage,station+'/CalibrationBoard/Pattern')
        pattern.CreatePointsAttr([(0,0,-.00002),(w,0,-.00002),(w,h,-.00002),(0,h,-.00002)])
        pattern.CreateFaceVertexCountsAttr([4]);pattern.CreateFaceVertexIndicesAttr([3,2,1,0]);pattern.CreateDoubleSidedAttr(True);pattern.CreateSubdivisionSchemeAttr('none')
        uv=UsdGeom.PrimvarsAPI(pattern).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex);uv.Set([(0,1),(1,1),(1,0),(0,0)])
        mat=material('charuco',[1,1,1],True);shader=UsdShade.Shader.Get(stage,str(mat.GetPath())+'/Shader')
        reader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/ST');reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
        texture=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Texture');texture.CreateIdAttr('UsdUVTexture')
        texture.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Path(os.path.relpath(prepared/'board_pattern.png',output)).as_posix());texture.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('raw')
        texture.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result')
        shader.GetInput('emissiveColor').ConnectToSource(texture.ConnectableAPI(),'rgb');UsdShade.MaterialBindingAPI.Apply(pattern.GetPrim()).Bind(mat)
        # Explicit simulator fixture: static rigid board; no physical settling.
        stage.GetPrimAtPath(station+'/CalibrationBoard').SetCustomDataByKey('support','Fixed simulation fixture, placement is not a hardware mounting claim')
        TWD=np.array(s['scene']['T_W_D']);tool=np.array(s['tool']['T_flange_tool_cad']);readbacks=[]
        def state(q,P):
            frames=robot.named_link_frames(q)
            for name,T in frames.items():pose(station+'/Robot/'+name,T)
            for prim in stage.GetPrimAtPath(station+'/Tool').GetChildren():pose(str(prim.GetPath()),frames['link_6']@tool)
            if stage.GetPrimAtPath(station+'/Adapter'):
                for prim in stage.GetPrimAtPath(station+'/Adapter').GetChildren():pose(str(prim.GetPath()),frames['link_6'])
            pose(station+'/CalibrationBoard',P)
            cache=UsdGeom.XformCache();C=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(optical))).T
            L=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(station+'/Robot/link_1'))).T
            render=np.array(cache.GetLocalToWorldTransform(cam.GetPrim())).T
            board_world=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(station+'/CalibrationBoard'))).T
            err=max(float(np.max(np.abs(C-TWD@frames['link_1']@X))),float(np.max(np.abs(render-C@OPTICAL_FROM_USD))),float(np.max(np.abs(board_world-TWD@P))))
            if err>1e-9:raise ValueError('COMPOSED_CAMERA_CHAIN_MISMATCH')
            if not perturb and (abs(C[2,2])>1e-9 or abs(C[2,1]+1)>1e-9):raise ValueError('CAMERA_NOT_HORIZONTAL_UPRIGHT')
            readbacks.append(dict(q_rad=np.asarray(q).tolist(),T_W_C=C.tolist(),T_L_C=(np.linalg.inv(L)@C).tolist(),max_error=err))
            return C
        placement={p['board_placement_id']:np.array(p['T_D_P']) for p in c['placements']}
        first=c['photos'][0];state(first['q_rad'],placement[first['board_placement_id']])
        # Test all J2-J6 independently on the FINAL composed prim chain.
        for joint in range(1,6):
            q=np.array(first['q_rad']);q[joint]+=.03;state(q,placement[first['board_placement_id']])
        stage.GetRootLayer().Save();timeline=omni.timeline.get_timeline_interface();timeline.stop();events=[]
        sub=timeline.get_timeline_event_stream().create_subscription_to_pop(lambda e:events.append(int(e.type)))
        omni.usd.get_context().open_stage(str(stage_path))
        for _ in range(30):app.update()
        stage=omni.usd.get_context().get_stage();cam=UsdGeom.Camera(stage.GetPrimAtPath(camera_path));timeline.stop()
        rp=rep.create.render_product(camera_path,(1024,1024));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp)
        depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth.attach(rp)
        segmentation=rep.AnnotatorRegistry.get_annotator('instance_id_segmentation_fast');segmentation.attach(rp)
        K=np.array(c['camera']['K']);Ks=intrinsics(1024,1024,64,64);width,height=c['camera']['resolution']
        yy,xx=np.indices((height,width),dtype=np.float32);mapx=((xx-K[0,2])/K[0,0]*Ks[0,0]+Ks[0,2]).astype(np.float32);mapy=((yy-K[1,2])/K[1,1]*Ks[1,1]+Ks[1,2]).astype(np.float32)
        def render():
            for _ in range(14):rep.orchestrator.step(delta_time=0.,rt_subframes=4,pause_timeline=True)
            a=rgb.get_data();a=a['data'] if isinstance(a,dict) else a;a=np.asarray(a)[...,:3]
            d=depth.get_data();d=d['data'] if isinstance(d,dict) else d;d=np.asarray(d)
            return a,cv2.remap(a,mapx,mapy,cv2.INTER_LINEAR),cv2.remap(d,mapx,mapy,cv2.INTER_NEAREST)
        # Known optical rays are separately rendered, then removed from photos.
        cx,cy=K[:2,2];colors=[[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1]];pixels=[[cx,cy],[25,cy],[width-26,cy],[cx,25],[cx,height-26]]
        for i,(color,pixel) in enumerate(zip(colors,pixels)):
            xyz=np.linalg.inv(K)@[*pixel,1.];xyz*=.06
            mesh(optical+'/ProjectionTargets/p'+str(i),trimesh.creation.icosphere(subdivisions=2,radius=.001),material('ray'+str(i),color,True),transform(xyz))
        state(first['q_rad'],placement[first['board_placement_id']]);native,adapted,d=render();cv2.imwrite(str(output/'projection_probe.png'),cv2.cvtColor(adapted,cv2.COLOR_RGB2BGR))
        measured=[];seg=segmentation.get_data();save_json(output/'projection_instance_labels.json',seg['info'])
        for i in range(len(pixels)):
            ids=[int(k) for k,v in seg['info']['idToLabels'].items() if str(v).endswith('/ProjectionTargets/p'+str(i))]
            if not ids:raise ValueError('PROJECTION_INSTANCE_NOT_LABELLED:'+str(i))
            mask=cv2.remap(np.isin(seg['data'],ids).astype(np.uint8),mapx,mapy,cv2.INTER_NEAREST)
            rows,cols=np.where(mask)
            if len(rows)<30:raise ValueError('PROJECTION_MARKER_NOT_VISIBLE')
            measured.append([float(cols.mean()),float(rows.mean())])
        measured=np.array(measured);expected=np.array(pixels);error=float(np.max(np.linalg.norm(measured-expected,axis=1)))
        # Infer fx/fy from actual image marker displacements and known optical rays.
        fx=(measured[2,0]-measured[1,0])/((expected[2,0]-expected[1,0])/K[0,0]);fy=(measured[4,1]-measured[3,1])/((expected[4,1]-expected[3,1])/K[1,1])
        fovs=np.degrees(2*np.arctan(np.array([width,height])/(2*np.array([fx,fy]))))
        projection=dict(expected_pixels=expected.tolist(),measured_pixels=measured.tolist(),max_error_px=error,target_fov_deg=[64,50],measured_fov_deg=fovs.tolist(),fov_error_deg=(fovs-[64,50]).tolist(),
            actual_apertures=[cam.GetHorizontalApertureAttr().Get(),cam.GetVerticalApertureAttr().Get()],native_resolution=[1024,1024],output_resolution=[width,height])
        save_json(output/'projection_check.json',projection)
        if error>c['thresholds']['projection_error_px'] or max(abs(fovs-[64,50]))>c['thresholds']['fov_error_deg']:raise ValueError('ACTUAL_RENDER_PROJECTION_FAILED')
        stage.GetPrimAtPath(optical+'/ProjectionTargets').SetActive(False)
        photos=[];frame=0
        for item in c['photos']:
            frame+=1;C=state(item['q_rad'],placement[item['board_placement_id']]);native,a,d=render();name=item['sample_id']
            image=observations/'images'/(name+'.png');cv2.imwrite(str(image),cv2.cvtColor(a,cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(output/(name+'_native.png')),cv2.cvtColor(native,cv2.COLOR_RGB2BGR))
            np.save(observations/'images'/(name+'_depth_z.npy'),d);cv2.imwrite(str(observations/'images'/(name+'_depth_valid.png')),(np.isfinite(d)&(d>0)).astype(np.uint8)*255)
            photos.append(dict(item,image=image.relative_to(observations).as_posix(),K=c['camera']['K'],camera_configuration_id=digest(c['camera']),
                render_frame_id=frame,simulation_time_s=float(timeline.get_current_time()),capture_wall_time_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                timestamp_semantics='Host readback time; not a real sensor exposure',sha256=file_hash(image)))
            print('SENSOR_CAPTURED',name,flush=True)
        if timeline.is_playing() or timeline.get_current_time()!=0 or int(omni.timeline.TimelineEventType.PLAY) in events:raise ValueError('MOTION_EXECUTED')
        ids=dict(camera=digest(c['camera']),probe=digest(c['probe']),board=digest(c['board']))
        save_json(observations/'observations.json',dict(schema='known_tip_touch_observations_v1',kinematics=c['kinematics'],
            camera={k:c['camera'][k] for k in ['resolution','K','distortion','pixel_convention']},board=c['board'],probe=c['probe'],
            photos=photos,touches=c['touches'],thresholds=c['thresholds'],source='SIMULATION',identifiers=ids))
        save_json(output/'truth/truth.json',dict(T_L_C=X.tolist(),perturbation=delta.tolist(),placements=c['placements'],snapshot_fingerprint=s['fingerprint']))
        save_json(output/'camera_chain_readback.json',dict(checks=readbacks,perturbed=perturb,source_USD=str(source),optical_to_USD=OPTICAL_FROM_USD.tolist()))
        stage.GetRootLayer().Save()
        # Separate external views are labelled diagnostics, never sensor inputs.
        review=UsdGeom.Camera.Define(stage,'/World/CalibrationReviewCamera');review.CreateFocalLengthAttr(42.);review.CreateHorizontalApertureAttr(36.);review.CreateVerticalApertureAttr(20.25)
        review.CreateClippingRangeAttr(Gf.Vec2f(.01,100.));rp2=rep.create.render_product(str(review.GetPath()),(1920,1080));rgb2=rep.AnnotatorRegistry.get_annotator('rgb');rgb2.attach(rp2)
        base=np.array(s['scene']['base_position_m'])+TWD[:3,3]
        for name,eye,target in [('overview',[-1.6,1.7,1.4],[-.15,.05,.45]),('camera_mount',[-.1,.5,.85],[.02,.14,.62])]:
            pose(str(review.GetPath()),np.array(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*(base+eye)),Gf.Vec3d(*(base+target)),Gf.Vec3d(0,0,1)).GetInverse()).T)
            for _ in range(20):rep.orchestrator.step(delta_time=0.,rt_subframes=4,pause_timeline=True)
            a=rgb2.get_data();a=a['data'] if isinstance(a,dict) else a;cv2.imwrite(str(output/(name+'.png')),cv2.cvtColor(np.asarray(a)[...,:3],cv2.COLOR_RGB2BGR))
        import importlib.metadata
        save_json(output/'capture_report.json',dict(isaac_version=importlib.metadata.version('isaacsim'),camera_prim=camera_path,photos=len(photos),
            timeline_time_s=timeline.get_current_time(),timeline_playing=timeline.is_playing(),play_events=events.count(int(omni.timeline.TimelineEventType.PLAY)),
            seconds=time.perf_counter()-started,status='RENDERED_SENSOR_IMAGES',dynamics='NOT_EVALUATED',motion='NO_MOTION_EXECUTED'))
    except Exception as error:
        import json,traceback
        output.mkdir(parents=True,exist_ok=True)
        (output/'capture_failure.json').write_text(json.dumps(dict(status='FAILED',error=str(error),traceback=traceback.format_exc()),indent=2))
        traceback.print_exc()
        raise
    finally:app.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepared',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--perturb',action='store_true');a=p.parse_args()
    capture(a.prepared,a.output,a.perturb)
