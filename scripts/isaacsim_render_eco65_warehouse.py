"""Compose official Warehouse and frozen private workstation, render without Play."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,required=True)
    p.add_argument('--workstation',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--compose-only',action='store_true',help='Check USD composition without Isaac or rendering')
    a=p.parse_args()
    a.output=a.output.resolve();a.assets=a.assets.resolve();a.workstation=a.workstation.resolve()
    a.output.mkdir(parents=True,exist_ok=False)
    app=None
    if not a.compose_only:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':1920,'height':1080})
    try:
        import numpy as np
        from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,UsdShade,Gf,Sdf
        from PIL import Image,ImageDraw,ImageFont
        sources=json.loads((a.assets/'asset_inspection.json').read_text())
        source_manifest=json.loads((a.assets/'official/manifest.json').read_text())
        for record in source_manifest['files']:
            if hashlib.sha256((a.assets/'official'/record['path']).read_bytes()).hexdigest()!=record['sha256']:
                raise ValueError('Official cached asset changed: '+record['path'])
        warehouse,carton=sources
        for item in sources:
            if item['up_axis']!='Z' or item['meters_per_unit']!=1.:
                raise ValueError('Unexpected official units/up axis; explicit unit conversion required')
        data=json.loads(a.workstation.with_name('workstation_manifest.json').read_text())
        if hashlib.sha256(a.workstation.read_bytes()).hexdigest()!=data['workstation_sha256']:
            raise ValueError('Workstation changed after export')
        stage_path=a.output/'eco65_desktop_warehouse.usda'
        stage=Usd.Stage.CreateNew(str(stage_path))
        UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
        root=UsdGeom.Xform.Define(stage,'/World').GetPrim();stage.SetDefaultPrim(root)
        env=UsdGeom.Xform.Define(stage,'/World/Warehouse')
        env.GetPrim().GetReferences().AddReference(Path(os.path.relpath(warehouse['path'],a.output)).as_posix())
        bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],useExtentsHint=False)
        # Select the broad, horizontal floor surface from actual official meshes.
        floor_candidates=[]
        for prim in Usd.PrimRange(env.GetPrim(), Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh) and 'floor' in str(prim.GetPath()).lower():
                b=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
                lo,hi=np.array(b.GetMin()),np.array(b.GetMax());size=hi-lo
                if size[0]>2 and size[1]>2 and size[2]<.5:
                    floor_candidates.append((float(size[0]*size[1]),prim,lo,hi))
        if not floor_candidates:raise RuntimeError('No verified Warehouse floor surface')
        _,floor,flo,fhi=max(floor_candidates,key=lambda r:r[0])
        floor_z=float(fhi[2])
        floor_tiles=[(lo,hi) for area,prim,lo,hi in floor_candidates if area>=max(r[0] for r in floor_candidates)*.9 and abs(hi[2]-floor_z)<.001]
        flo=np.min([lo for lo,hi in floor_tiles],axis=0);fhi=np.max([hi for lo,hi in floor_tiles],axis=0)
        env.AddTranslateOp(opSuffix="floorNormalization").Set(Gf.Vec3d(0,0,-floor_z))
        bbox.Clear()
        # All mesh and collider bounds participate, including pallets and shelves.
        obstacles=[];floor_paths=[];colliders=[]
        for prim in Usd.PrimRange(env.GetPrim(), Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.CollisionAPI):colliders.append(str(prim.GetPath()))
            if not (prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI)):continue
            b=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            if b.IsEmpty():continue
            lo,hi=np.array(b.GetMin()),np.array(b.GetMax())
            if hi[2]<=.0001:
                floor_paths.append(str(prim.GetPath()));continue
            obstacles.append((str(prim.GetPath()),lo,hi))
        if not colliders:raise RuntimeError('Official environment has no enabled collision representation')
        # Infer the interior from structural walls; the apron is not indoor space.
        enclosure=[]
        for axis in [0,1]:
            walls=[(lo,hi) for name,lo,hi in obstacles if ('SM_WallA' in name or 'SM_WallB' in name)
                   and hi[2]-lo[2]>2 and hi[axis]-lo[axis]<.5 and hi[1-axis]-lo[1-axis]>2]
            if not walls:raise RuntimeError('Cannot establish warehouse interior from walls')
            centers=[(lo[axis]+hi[axis])/2 for lo,hi in walls]
            low=max(hi[axis] for (lo,hi),v in zip(walls,centers) if v<min(centers)+.5)
            high=min(lo[axis] for (lo,hi),v in zip(walls,centers) if v>max(centers)-.5)
            enclosure.append([float(low),float(high)])
        def floor_covered(low,high):
            tiles=[(np.maximum(low[:2],lo[:2]),np.minimum(high[:2],hi[:2])) for lo,hi in floor_tiles]
            tiles=[(lo,hi) for lo,hi in tiles if np.all(hi>lo)]
            cuts=sorted({float(v) for lo,hi in tiles for v in [lo[0],hi[0]]}|{float(low[0]),float(high[0])})
            area=0.
            for left,right in zip(cuts,cuts[1:]):
                intervals=sorted((lo[1],hi[1]) for lo,hi in tiles if lo[0]<right and hi[0]>left)
                end=low[1];length=0.
                for bottom,top in intervals:
                    length+=max(0.,top-max(bottom,end));end=max(end,top)
                area+=(right-left)*length
            return abs(area-np.prod(high[:2]-low[:2]))<1e-5
        candidates=[]
        for x in np.arange(enclosure[0][0]+1.7,enclosure[0][1]-1.1,.25):
            for y in np.arange(enclosure[1][0]+1.4,enclosure[1][1]-1.4,.25):
                low=np.array([x-1.7,y-1.4,.001]);high=np.array([x+1.1,y+1.4,2.1])
                if not floor_covered(low,high):continue
                if not any(np.all(high>lo) and np.all(hi>low) for _,lo,hi in obstacles):
                    candidates.append((float((x-.3)**2+y*y),float(x),float(y)))
        if not candidates:raise RuntimeError('No conservative collision-free table placement')
        _,x,y=min(candidates);x=round(x,3);y=round(y,3);translation=[x,y,.8]
        station=UsdGeom.Xform.Define(stage,'/World/Workstation')
        station.GetPrim().GetReferences().AddReference(Path(os.path.relpath(a.workstation,a.output)).as_posix())
        station.AddTranslateOp(opSuffix='T_W_D').Set(Gf.Vec3d(*translation))
        glass=UsdShade.Shader.Get(stage,'/World/Workstation/Materials/glass/Shader')
        glass.CreateInput('useSpecularWorkflow',Sdf.ValueTypeNames.Int).Set(1)
        glass.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0))
        glass.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(0.)
        glass.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.)
        glass.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.08)
        # Complete carton references: retain every mesh, material and texture.
        source=Usd.Stage.Open(carton['path'])
        sb=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],useExtentsHint=False)
        r=sb.ComputeWorldBound(source.GetDefaultPrim()).ComputeAlignedRange()
        rawlo,rawhi=np.array(r.GetMin()),np.array(r.GetMax());rawsize=rawhi-rawlo
        if np.any(rawsize<=0):raise RuntimeError('Invalid carton asset bounds')
        scale=np.array([.22,.22,.16])/rawsize;center=(rawlo+rawhi)/2
        carton_records=[]
        for box in data['cartons']:
            path='/World/Workstation/Cartons/'+box['id']
            fit=UsdGeom.Xform.Define(stage,path+'/Fit');fit.AddScaleOp().Set(Gf.Vec3d(*scale))
            pivot=UsdGeom.Xform.Define(stage,path+'/Fit/Pivot');pivot.AddTranslateOp().Set(Gf.Vec3d(*(-center)))
            visual=stage.DefinePrim(path+'/Fit/Pivot/Official');visual.GetReferences().AddReference(Path(os.path.relpath(carton['path'],a.output)).as_posix())
            for prim in Usd.PrimRange(visual):
                if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            bbox.Clear();r=bbox.ComputeWorldBound(visual).ComputeAlignedRange()
            lo,hi=np.array(r.GetMin()),np.array(r.GetMax())
            expected=np.array(box['pose'])+translation
            if not np.allclose(hi-lo,[.22,.22,.16],atol=2e-6) or not np.allclose((hi+lo)/2,expected,atol=2e-6):
                raise RuntimeError('Carton dimension/center mismatch')
            carton_records.append(dict(id=box['id'],bounds_W_m=[lo.tolist(),hi.tolist()],center_W_m=expected.tolist(),
                mesh_count=sum(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(visual)),collision_proxy=path+'/Collision'))
        if stage.GetCompositionErrors():raise RuntimeError(str(stage.GetCompositionErrors()))
        bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'],useExtentsHint=False)
        def bounds(path):
            r=bbox.ComputeWorldBound(stage.GetPrimAtPath('/World/Workstation/'+path)).ComputeAlignedRange()
            return np.array(r.GetMin()),np.array(r.GetMax())
        heights={}
        for name,path,side,expected in [('tabletop','trailer_floor',1,.8),('robot_mount_top','robot_mount_plate',1,.98),
                ('transverse_belt','conveyor_transverse_belt',1,.98),('longitudinal_belt','conveyor_longitudinal_belt',1,.98),
                ('trailer_roof_inner','trailer_roof',0,1.45)]:
            heights[name]=float(bounds(path)[side][2]);assert abs(heights[name]-expected)<2e-6
        assert np.allclose(bounds('trailer_floor')[1][:2]-bounds('trailer_floor')[0][:2],[2,2])
        # Check each actual workstation mesh/collider, not just its design footprint.
        intersections=[];checked=0
        for prim in Usd.PrimRange(station.GetPrim()):
            if not (prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI)):continue
            r=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():continue
            lo,hi=np.array(r.GetMin()),np.array(r.GetMax());checked+=1
            if lo[2]<-2e-6:raise RuntimeError('Workstation penetrates warehouse floor')
            for name,elo,ehi in obstacles:
                overlap=np.minimum(hi,ehi)-np.maximum(lo,elo)
                if np.all(overlap>2e-6):intersections.append([str(prim.GetPath()),name,overlap.tolist()])
        if intersections:raise RuntimeError('Workstation/environment interference: '+str(intersections[:5]))
        # Keep the official lighting; add a soft photographic fill at the workbench.
        light=UsdLux.DomeLight.Define(stage,'/World/WorkbenchFill')
        light.CreateIntensityAttr(400.);light.CreateColorAttr(Gf.Vec3f(.91,.95,1.))
        key=UsdLux.RectLight.Define(stage,'/World/WorkbenchKey')
        key.CreateIntensityAttr(500.);key.CreateWidthAttr(3.);key.CreateHeightAttr(3.)
        key.CreateColorAttr(Gf.Vec3f(1.,.97,.92))
        UsdGeom.Xformable(key).AddTranslateOp().Set(Gf.Vec3d(x-.3,y-.3,3.5))
        stage.GetRootLayer().Save()
        print('COMPOSITION_VALIDATED',translation,heights,flush=True)
        if a.compose_only:
            (a.output/'composition_check.json').write_text(json.dumps(dict(translation_W_D_m=translation,height_checks_m=heights,cartons=carton_records,environment_colliders=len(colliders),intersections=intersections),indent=2))
            print('COMPOSITION_PASS',translation,heights,flush=True)
            return
        import omni.usd
        import omni.replicator.core as rep
        import omni.timeline
        omni.usd.get_context().open_stage(str(stage_path))
        for _ in range(30):app.update()
        print('STAGE_OPENED',flush=True)
        timeline=omni.timeline.get_timeline_interface();timeline.stop()
        # A single camera is retargeted; only rendering advances, delta_time=0.
        render_stage=omni.usd.get_context().get_stage()
        camera=UsdGeom.Camera.Define(render_stage,'/World/ReviewCamera')
        camera.CreateHorizontalApertureAttr(36.)
        camera.CreateVerticalApertureAttr(20.25)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01,200.))
        camera_transform=UsdGeom.Xformable(camera).AddTransformOp()
        rp=rep.create.render_product(str(camera.GetPath()),(1920,1080))
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp)
        # Camera coordinates are recorded exactly for reproducibility.
        views=[
            ('01_warehouse_overall',(x+4.5,y-2.0,2.6),(x-.3,y,1.3),28),
            ('02_workstation_oblique',(x-2.3,y-2.5,2.7),(x-.3,y,1.02),45),
            ('03_layout_top',(x-.3,y,4.2),(x-.3,y,.8),28),
            ('04_nvidia_cartons',(x-.75,y-1.25,1.65),(x+.11,y,.98),70),
            ('05_table_dimensions',(x-2.0,y-4.5,2.6),(x-.3,y,.6),28),
        ]
        results=[]
        for name,eye,target,focal in views:
            print('RENDERING',name,flush=True)
            up=Gf.Vec3d(0,1,0) if name=='03_layout_top' else Gf.Vec3d(0,0,1)
            camera_transform.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),up).GetInverse())
            camera.CreateFocalLengthAttr(float(focal))
            for _ in range(40):
                rep.orchestrator.step(delta_time=0.,rt_subframes=4,pause_timeline=True)
            assert not timeline.is_playing() and timeline.get_current_time()==0.
            arr=rgb.get_data();arr=arr['data'] if isinstance(arr,dict) else arr
            pixels=np.asarray(arr)[...,:3]
            if pixels.shape!=(1080,1920,3) or np.std(pixels)<5:raise RuntimeError('Invalid camera render')
            path=a.output/(name+'.png');Image.fromarray(pixels).save(path)
            if name=='05_table_dimensions':
                # Preserve the untouched render alongside its measured annotations.
                raw_path=a.output/'05_table_dimensions_raw.png'
                Image.fromarray(pixels).save(raw_path)
                prim=camera.GetPrim()
                gf_camera=UsdGeom.Camera(prim).GetCamera(Usd.TimeCode.Default())
                gf_camera.transform=UsdGeom.XformCache().GetLocalToWorldTransform(prim)
                f=gf_camera.frustum
                vp=f.ComputeViewMatrix()*f.ComputeProjectionMatrix()
                def project(point):
                    ndc=vp.Transform(Gf.Vec3d(*point))
                    return ((ndc[0]+1)*960,(1-ndc[1])*540)
                im=Image.fromarray(pixels);draw=ImageDraw.Draw(im)
                try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',28)
                except OSError:font=ImageFont.load_default()
                def dimension(start,end,label,offset=(0,0)):
                    u=np.array(project(start));v=np.array(project(end));draw.line([tuple(u),tuple(v)],fill='#ffd84b',width=4)
                    direction=(v-u)/np.linalg.norm(v-u);normal=np.array([-direction[1],direction[0]])
                    for at,sign in [(u,1),(v,-1)]:
                        arrow=[tuple(at),tuple(at+sign*direction*18+normal*7),tuple(at+sign*direction*18-normal*7)]
                        draw.polygon(arrow,fill='#ffd84b')
                    middle=(u+v)/2+np.array(offset);bb=draw.textbbox((0,0),label,font=font);width=bb[2]-bb[0]
                    pos=(middle[0]-width/2,middle[1]-18)
                    draw.rectangle([pos[0]-9,pos[1]-4,pos[0]+width+9,pos[1]+36],fill='#172432')
                    draw.text(pos,label,font=font,fill='#ffd84b')
                dimension((x-1.3,y-1.10,.81),(x+.7,y-1.10,.81),'2000 mm',(0,28))
                dimension((x-1.40,y-1,.81),(x-1.40,y+1,.81),'2000 mm',(-30,-8))
                dimension((x-1.48,y-1.10,0),(x-1.48,y-1.10,.8),'800 mm',(-85,0))
                draw.rectangle([32,28,965,92],fill='#172432')
                draw.text((48,43),'WAREHOUSE | Table 2.000 x 2.000 m | Top Z = 0.800 m',font=font,fill='white')
                im.save(path)
            results.append(dict(name=name,file=path.name,eye=eye,target=target,focal_length=focal,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print('RENDERED',path,flush=True)
        render_stage.GetRootLayer().Save()
        report=dict(status='STATIC_RENDER_COMPLETE',runtime=json.loads((a.assets/'runtime.json').read_text()),
            warehouse_source=warehouse['source'],carton_source=carton['source'],official_file_count=source_manifest['file_count'],
            floor_prim=str(floor.GetPath()),floor_original_top_z_m=floor_z,warehouse_translation_m=[0,0,-floor_z],
            T_W_D=[[1,0,0,x],[0,1,0,y],[0,0,1,.8],[0,0,0,1]],yaw_rad=0.,
            floor_world_z_m=0.,height_checks_m=heights,cartons=carton_records,
            carton_original_bounds_m=[rawlo.tolist(),rawhi.tolist()],carton_visual_scale=scale.tolist(),
            environment_meshes_and_colliders_checked=len(obstacles),environment_colliders=len(colliders),
            workstation_shapes_checked=checked,intersections=intersections,clearance_reserve_m=.4,
            interior_bounds_xy_m=enclosure,environment_removed_prims=[],environment_scale=1.,workstation_scale=1.,
            timeline_time_s=timeline.get_current_time(),physics='NOT_EVALUATED',planning='NOT_EXECUTED',hardware=False,
            renders=results,source_snapshot_fingerprint=data['source_snapshot_fingerprint'])
        (a.output/'render_report.json').write_text(json.dumps(report,indent=2))
        print('STATIC_RENDER_COMPLETE',flush=True)
    except Exception:
        import traceback
        error=traceback.format_exc()
        (a.output/'failure.txt').write_text(error)
        print(error,flush=True)
        raise
    finally:
        if app:app.close()


if __name__=='__main__':main()
