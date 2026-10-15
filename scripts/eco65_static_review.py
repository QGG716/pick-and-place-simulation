"""USD readback, display-only collision edges and stopped-timeline review views."""
import hashlib
import json
from pathlib import Path
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt
from PIL import Image, ImageDraw, ImageFont


def audit(stage,data):
    cache=UsdGeom.XformCache();bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'])
    T=np.array(data['T_W_D']);rows=[];issues=[]
    def matrix(path): return np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
    def compare(name,expected,actual):
        expected=np.asarray(expected);actual=np.asarray(actual);delta=float(np.max(np.abs(expected-actual)))
        rows.append(dict(item=name,snapshot=expected.tolist(),usd=actual.tolist(),max_abs_delta=delta,match=delta<2e-6))
        if delta>=2e-6:issues.append(name)
    def bounds(path):
        r=bbox.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array([r.GetMin(),r.GetMax()])
    compare('meters_per_unit',1.,UsdGeom.GetStageMetersPerUnit(stage))
    if UsdGeom.GetStageUpAxis(stage)!='Z':issues.append('UP_AXIS_NOT_Z')
    compare('T_W_D_applied_once',T,matrix('/World/Workstation'))
    compare('HOME_joint_values',data['home_q_rad'],stage.GetPrimAtPath('/World/Workstation').GetCustomDataByKey('home_q_rad'))
    for name,frame in data['frame_transforms_D'].items():
        compare('frame/'+name,T@np.array(frame),matrix('/World/Workstation/Frames/'+name))
        visual='/World/Workstation/Robot/'+name+'/Visual'
        if stage.GetPrimAtPath(visual):compare('robot_visual/'+name,T@np.array(frame),matrix(visual))
    for record in data['collision_records']:
        path=record['path'].replace('/Workstation','/World/Workstation',1)
        compare('collision_transform/'+path,T@np.array(record['T_D']),matrix(path))
        compare('collision_vertices/'+path,record['points'],UsdGeom.Mesh(stage.GetPrimAtPath(path)).GetPointsAttr().Get())
    for p in stage.GetPrimAtPath('/World/Workstation/Tool').GetChildren():
        if p.GetName().startswith('visual_'):
            compare('tool_visual/'+p.GetName(),T@np.array(data['frame_transforms_D']['tool_cad']),matrix(str(p.GetPath())))
    dims={}
    for item in data['obstacles']:
        path='/World/Workstation/'+item['id'];b=bounds(path);center=np.array(item['position_m'])+T[:3,3]
        compare('obstacle/'+item['id']+'/center',center,b.mean(axis=0))
        compare('obstacle/'+item['id']+'/size',item['size_m'],b[1]-b[0]);dims[item['id']]=b.tolist()
    for item in data['cartons']:
        path='/World/Workstation/Cartons/'+item['id'];b=bounds(path+'/Collision');center=np.array(item['pose'])+T[:3,3]
        compare('carton/'+item['id']+'/center',center,b.mean(axis=0));compare('carton/'+item['id']+'/size',item['size_m'],b[1]-b[0])
        compare('carton/'+item['id']+'/orientation',np.eye(3),matrix(path)[:3,:3])
        compare('carton/'+item['id']+'/appearance_proxy',b,bounds(path+'/Fit'));dims[item['id']]=b.tolist()
        proxies=[p for p in Usd.PrimRange(stage.GetPrimAtPath(path)) if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()!=False]
        if len(proxies)!=1:issues.append('CARTON_COLLIDER_COUNT:'+item['id'])
    environment=[]
    for item in data['environment']['objects']:
        path='/World/Warehouse'+item['prim'].removeprefix('/Root');actual=bounds(path)
        # Match the existing planner's conservative downward/sideways extension.
        for axis in range(3):
            if actual[1,axis]-actual[0,axis]<1e-6:actual[0,axis]=actual[1,axis]-1e-6
        delta=float(np.max(np.abs(np.array(item['bounds_W_m'])-actual)))
        environment.append(dict(id=item['id'],prim=path,max_abs_delta_m=delta))
        if delta>2e-6:issues.append('ENVIRONMENT_BOUND:'+item['id'])
    potential=[]
    for p in stage.Traverse():
        name=str(p.GetPath())
        if not name.startswith(('/World/Workstation/Robot/','/World/Workstation/Tool/')) or not p.IsA(UsdGeom.Mesh):continue
        if 'Collision' in name or 'collision_' in name:continue
        b=bounds(name)
        for target in ['trailer_roof','trailer_left','trailer_right']:
            other=np.array(dims[target]);overlap=np.minimum(b[1],other[1])-np.maximum(b[0],other[0])
            if np.all(overlap>0):potential.append(dict(pair=[name,target],overlap_m=overlap.tolist(),interpretation='AABB_POTENTIAL_ONLY_NOT_PROVEN_PENETRATION'))
    return dict(status='STATIC_LAYOUT_REVIEW',checks=rows,issues=issues,dimensions_bounds_W_m=dims,
        environment_bounds=environment,potential_interference=potential,
        home_source=data['home_source'],source_snapshot_fingerprint=data['source_snapshot_fingerprint'],
        units='metres',up_axis='Z',motion='NO_MOTION_EXECUTED',dynamics='NOT_EVALUATED')


def wall_edges(stage,data):
    """Visible edges of the unchanged transparent panels; no collision shapes."""
    cache=UsdGeom.XformCache()
    for row in data['obstacles']:
        if row['role'] not in ['wall','roof']:continue
        p=stage.GetPrimAtPath('/World/Workstation/'+row['id']);h=UsdGeom.Cube(p).GetSizeAttr().Get()/2
        v=np.array([[x,y,z] for x in [-h,h] for y in [-h,h] for z in [-h,h]])
        edges=[(a,b) for a in range(8) for b in range(a+1,8) if sum(v[a]!=v[b])==1]
        T=np.array(cache.GetLocalToWorldTransform(p)).T;v=v@T[:3,:3].T+T[:3,3]
        c=UsdGeom.BasisCurves.Define(stage,'/World/PanelEdges/'+row['id']);c.CreateTypeAttr('linear')
        c.CreateCurveVertexCountsAttr([2]*len(edges));c.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.array([v[i] for pair in edges for i in pair],dtype=np.float32)))
        c.CreateWidthsAttr([.0015]);c.SetWidthsInterpolation('constant');c.CreateDisplayColorAttr([Gf.Vec3f(.04,.12,.18)])


def overlay(stage):
    """Curves have no physics API and only display existing planning geometry."""
    root=UsdGeom.Xform.Define(stage,'/World/CollisionOverlay');cache=UsdGeom.XformCache();count=0
    for p in list(stage.Traverse()):
        path=str(p.GetPath())
        if not path.startswith('/World/Workstation/') or '/warehouse/' in path or not p.HasAPI(UsdPhysics.CollisionAPI):continue
        if '/Fit/' in path:continue
        if p.IsA(UsdGeom.Mesh):
            obj=UsdGeom.Mesh(p);vertices=np.array(obj.GetPointsAttr().Get());indices=list(obj.GetFaceVertexIndicesAttr().Get());counts=obj.GetFaceVertexCountsAttr().Get();edges=set();offset=0
            for n in counts:
                face=indices[offset:offset+n];offset+=n
                for a,b in zip(face,face[1:]+face[:1]):edges.add(tuple(sorted((a,b))))
        elif p.IsA(UsdGeom.Cube):
            h=UsdGeom.Cube(p).GetSizeAttr().Get()/2;vertices=np.array([[x,y,z] for x in [-h,h] for y in [-h,h] for z in [-h,h]])
            edges={(a,b) for a in range(8) for b in range(a+1,8) if sum(vertices[a]!=vertices[b])==1}
        else:continue
        T=np.array(cache.GetLocalToWorldTransform(p)).T;vertices=vertices@T[:3,:3].T+T[:3,3]
        color=(.05,.95,1.) if '/Robot/' in path or '/Tool/' in path else (1.,.5,.03) if '/Cartons/' in path else (.9,.1,.85)
        curves=UsdGeom.BasisCurves.Define(stage,f'/World/CollisionOverlay/shape_{count:03}');curves.CreateTypeAttr('linear')
        curves.CreateCurveVertexCountsAttr([2]*len(edges));curves.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.array([vertices[i] for edge in sorted(edges) for i in edge],dtype=np.float32)))
        curves.CreateWidthsAttr([.0008 if p.IsA(UsdGeom.Mesh) else .002]);curves.SetWidthsInterpolation('constant');curves.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        curves.GetPrim().SetCustomDataByKey('source_collision_prim',path);count+=1
    root.MakeInvisible();return root,count


def views(stage,data):
    T=np.array(data['T_W_D']);x,y,z=T[:3,3]
    cache=UsdGeom.XformCache();f=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Workstation/Frames/flange')))[3,:3]
    tcp=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Workstation/Frames/tcp')))[3,:3];target=(f+tcp)/2
    oblique=('02_workstation_oblique',(x-2.3,y-2.5,z+2.1),(x-.24,y,z+.30),50,None)
    return [
        ('01_warehouse_overall',(x-3.6,y-3.8,z+2.0),(x-.3,y,z-.05),32,None),
        oblique,
        ('03_layout_top',(x-.3,y,z+4),(x-.3,y,z),35,3.9),
        ('04_layout_side',(x-.3,y-4,z+.05),(x-.3,y,z+.05),35,3.6),
        ('05_layout_front',(x-4,y,z+.02),(x,y,z+.02),35,3.6),
        ('06_flange_tool_closeup',tuple(target+[-.45,-.50,.25]),tuple(target),55,None),
        ('07_collision_overlay',*oblique[1:])]


def annotate(pixels,camera,stage,data,review,name):
    im=Image.fromarray(pixels);draw=ImageDraw.Draw(im)
    try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
    except OSError:font=ImageFont.load_default()
    cam=camera.GetCamera(Usd.TimeCode.Default());cam.transform=UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim())
    vp=cam.frustum.ComputeViewMatrix()*cam.frustum.ComputeProjectionMatrix();W,H=im.size
    def project(point):
        p=vp.Transform(Gf.Vec3d(*point));return np.array([(p[0]+1)*W/2,(1-p[1])*H/2])
    def text(pos,label,color='white'):
        bb=draw.textbbox(pos,label,font=font);draw.rectangle([bb[0]-5,bb[1]-4,bb[2]+5,bb[3]+4],fill='#152333');draw.text(pos,label,fill=color,font=font)
    def arrow(a,b,label,color):
        a=project(a);b=project(b);d=b-a;n=np.linalg.norm(d)
        if n<1:return
        d=d/n;normal=np.array([-d[1],d[0]]);draw.line([tuple(a),tuple(b)],fill=color,width=4)
        draw.polygon([tuple(b),tuple(b-16*d+6*normal),tuple(b-16*d-6*normal)],fill=color)
        text(tuple(b+[8,-42 if name=='03_layout_top' else -12]),label,color)
    dims=review['dimensions_bounds_W_m'];T=np.array(data['T_W_D']);origin=T[:3,3];table=np.array(dims['trailer_floor'])
    text((28,25),'STATIC_LAYOUT_REVIEW | NO_MOTION_EXECUTED | Dynamics: NOT_EVALUATED')
    text((28,H-48),'USD readback | '+('MISMATCH: '+', '.join(review['issues'][:2]) if review['issues'] else 'geometry/pose checks match snapshot'))
    if name=='03_layout_top':
        o=table[0]+[.25,.25,.04];arrow(o,o+[.35,0,0],'+X (table D)','#ff6969');arrow(o,o+[0,.35,0],'+Y (table D)','#70ed88')
        for id,direction,label in [('conveyor_transverse_belt',[0,-.25,0],'transverse -Y'),('conveyor_longitudinal_belt',[-.40,0,0],'longitudinal -X')]:
            b=np.array(dims[id]);o=b.mean(axis=0);o[2]=b[1,2]+.005;arrow(o-np.array(direction)/2,o+np.array(direction)/2,label,'#ffd966')
        text((28,70),f'Table USD: {table[1,0]-table[0,0]:.3f} x {table[1,1]-table[0,1]:.3f} m')
    if name in ['04_layout_side','05_layout_front']:
        roof=np.array(dims['trailer_roof']);mount=np.array(dims['robot_mount_plate']);belt=np.array(dims['conveyor_longitudinal_belt'])
        text((28,70),f'World Z: tabletop {table[1,2]:.3f} | base/mount {mount[1,2]:.3f} | belt {belt[1,2]:.3f} m')
        text((28,110),f'Roof underside {roof[0,2]:.3f} m | carton centres '+', '.join(f'{np.mean(dims[b["id"]],axis=0)[2]:.3f}' for b in data['cartons'])+' m')
    if name=='06_flange_tool_closeup':
        cache=UsdGeom.XformCache()
        for key,length in [('flange',.045),('tcp',.035)]:
            f=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Workstation/Frames/'+key))).T
            for axis,label,color in [(0,'X','#ff6969'),(1,'Y','#70ed88'),(2,'Z','#65baff')]:arrow(f[:3,3],f[:3,3]+f[:3,axis]*length,key+' '+label,color)
    if name=='07_collision_overlay':text((28,70),'CYAN: robot/tool convex | ORANGE: cartons | MAGENTA: station obstacles')
    return im
