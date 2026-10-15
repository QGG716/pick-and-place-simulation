"""Read back the new 27-carton static layout; no motion acceptance."""
import hashlib
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt
from PIL import Image, ImageDraw, ImageFont
from eco65_static_review import audit as base_audit


def audit(stage,data):
    report=base_audit(stage,data);T=np.array(data['T_W_D']);origin=T[:3,3]
    cache=UsdGeom.XformCache();bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','guide','proxy'])
    dims=report['dimensions_bounds_W_m'];rows=report['checks'];issues=report['issues']
    def compare(name,expected,actual):
        e=np.asarray(expected);a=np.asarray(actual);delta=float(np.max(np.abs(e-a)))
        rows.append(dict(item=name,snapshot=e.tolist(),usd=a.tolist(),max_abs_delta=delta,match=delta<2e-6))
        if delta>=2e-6:issues.append(name)
    def bounds(p):
        r=bbox.ComputeWorldBound(p).ComputeAlignedRange();return np.array([r.GetMin(),r.GetMax()])
    conveyors=[]
    for item in data['conveyors']['items']:
        meshrows=[]
        for mesh in item['meshes']:
            path='/World/Workstation/'+item['id']+mesh['prim'].removeprefix('/Conveyor')
            p=stage.GetPrimAtPath(path);m=UsdGeom.Mesh(p);b=bounds(p)
            compare('official_mesh_bounds/'+path,np.array(mesh['bounds_D_m'])+origin,b)
            compare('official_mesh_transform/'+path,T,np.array(cache.GetLocalToWorldTransform(p)).T)
            if hashlib.sha256(np.asarray(m.GetPointsAttr().Get(),np.float32).tobytes()).hexdigest()!=mesh['points_sha256']:issues.append('CONVEYOR_VERTICES_CHANGED:'+path)
            enabled=p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()!=False
            if enabled!=mesh['collision']:issues.append('CONVEYOR_DUPLICATE_OR_MISSING_COLLISION:'+path)
            meshrows.append(dict(path=path,bounds_W_m=b.tolist(),collision_enabled=enabled))
        belt=next(r for r in meshrows if '/Belt/' in r['path']);outer=np.array([r['bounds_W_m'] for r in meshrows])
        conveyors.append(dict(id=item['id'],meshes=meshrows,belt_bounds_W_m=belt['bounds_W_m'],bounds_W_m=[outer[:,0].min(0).tolist(),outer[:,1].max(0).tolist()]))
    boxes=data['cartons'];allbounds=np.array([dims[b['id']] for b in boxes]);lo=allbounds[:,0].min(0);hi=allbounds[:,1].max(0)
    front=max(r['bounds_W_m'][1][0] for r in conveyors)
    gap=float(lo[0]-front);compare('actual_conveyor_carton_gap',data['layout_config']['carton_conveyor_gap_m'],gap)
    compare('carton_count',27,len(stage.GetPrimAtPath('/World/Workstation/Cartons').GetChildren()))
    pedestal=np.array(dims['cargo_pedestal']);roof=np.array(dims['trailer_roof'])
    compare('bottom_cartons_on_pedestal',pedestal[1,2],lo[2])
    for item in conveyors:compare(item['id']+'/belt_pedestal_level',pedestal[1,2],item['belt_bounds_W_m'][1][2])
    compare('boxwall_dimensions',[.250,.650,.900],hi-lo)
    compare('effective_cargo_height',.900,roof[0,2]-pedestal[1,2])
    compare('effective_inner_width',2.3/3,np.array(dims['trailer_left'])[0,1]-np.array(dims['trailer_right'])[1,1])
    supports=[]
    for b in boxes:
        actual=list(stage.GetPrimAtPath('/World/Workstation/Cartons/'+b['id']).GetCustomDataByKey('supported_by'))
        if actual!=b['supported_by']:issues.append('SUPPORT_ID:'+b['id'])
        bb=np.array(dims[b['id']]);sb=np.array(dims[actual[0]])
        compare('support_height/'+b['id'],sb[1,2],bb[0,2])
        covered=np.all(bb[0,:2]>=sb[0,:2]-2e-6) and np.all(bb[1,:2]<=sb[1,:2]+2e-6)
        if not covered:issues.append('SUPPORT_COVERAGE:'+b['id'])
        supports.append(dict(id=b['id'],supported_by=actual,footprint_covered=bool(covered)))
    # A surface vertex strictly inside an axis-aligned solid is stronger evidence
    # than intersecting AABBs. Absence of such a vertex is NOT a clearance proof.
    targets={b['id']:np.array(dims[b['id']]) for b in boxes}
    targets.update({k:np.array(v) for k,v in dims.items() if k.startswith('trailer_') or k=='cargo_pedestal'})
    penetration=[];potential=[]
    for p in stage.Traverse():
        name=str(p.GetPath())
        if not name.startswith(('/World/Workstation/Robot/','/World/Workstation/Tool/')) or not p.IsA(UsdGeom.Mesh) or 'collision' in name.lower():continue
        m=UsdGeom.Mesh(p);v=np.array(m.GetPointsAttr().Get());M=np.array(cache.GetLocalToWorldTransform(p)).T;v=v@M[:3,:3].T+M[:3,3]
        for key,b in targets.items():
            overlap=np.minimum(v.max(0),b[1])-np.maximum(v.min(0),b[0])
            if not np.all(overlap>2e-6):continue
            potential.append(dict(pair=[name,key],interpretation='AABB_POTENTIAL_ONLY'))
            inside=np.all((v>b[0]+1e-5)&(v<b[1]-1e-5),axis=1)
            if inside.any():penetration.append(dict(pair=[name,key],surface_vertices_strictly_inside_solid=int(inside.sum()),witness_W_m=v[inside][0].tolist(),status='GEOMETRIC_INTERSECTION_WITNESS_NOT_DYNAMIC_TEST'))
    # Equipment cuboids and conveyor exact vertices against retained solid panels,
    # pedestal and mount: record contacts separately from strict interior points.
    equipment=[]
    for item in conveyors:
        for mesh in item['meshes']:
            if not mesh['collision_enabled']:continue
            p=stage.GetPrimAtPath(mesh['path']);v=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get());M=np.array(cache.GetLocalToWorldTransform(p)).T;v=v@M[:3,:3].T+M[:3,3]
            for key in ['cargo_pedestal','trailer_rear','trailer_left','trailer_right','robot_mount_plate','trailer_floor']:
                b=np.array(dims[key]);n=int(np.all((v>b[0]+1e-5)&(v<b[1]-1e-5),axis=1).sum())
                if n:equipment.append(dict(pair=[mesh['path'],key],vertices_inside=n))
    report.update(status='STATIC_LAYOUT_REVIEW',motion_status='FULL_FACE_TARGET_NOT_MOTION_VALIDATED',
        full_face=dict(box_count=len(boxes),boxwall_bounds_W_m=[lo.tolist(),hi.tolist()],actual_gap_m=gap,
            conveyor_most_protruding_x_D_m=front-origin[0],box_front_x_D_m=lo[0]-origin[0],
            belt_fronts_x_D_m={i['id']:i['belt_bounds_W_m'][1][0]-origin[0] for i in conveyors},
            effective_height_m=roof[0,2]-pedestal[1,2],effective_width_m=np.array(dims['trailer_left'])[0,1]-np.array(dims['trailer_right'])[1,1],
            pedestal_top_D_m=pedestal[1,2]-origin[2],roof_underside_D_m=roof[0,2]-origin[2],
            carton_center_heights_D_m=sorted({round(np.mean(dims[b['id']],axis=0)[2]-origin[2],8) for b in boxes}),
            supports=supports,conveyors=conveyors,top_geometry=data['top_geometry']),
        actual_HOME_surface_intersection_witnesses=penetration,actual_HOME_AABB_potential=potential,
        equipment_surface_intersection_witnesses=equipment,clearance_proof='NOT_PERFORMED; vertex test cannot prove separation')
    return report


def views(stage,data):
    o=np.array(data['T_W_D'])[:3,3]
    def v(name,eye,target,f=45,w=None):return(name,tuple(o+eye),tuple(o+target),f,w)
    oblique=v('02_workstation_oblique',[-2.6,-3.2,2.3],[-.23,0,.48],50)
    return [v('01_warehouse_overall',[-4,-4,2.6],[-.3,0,.05],34),oblique,
        v('03_layout_top',[-.3,0,4],[-.3,0,0],35,3.9),
        v('04_layout_front',[-4,0,.26],[.25,0,.26],35,4.1),
        v('05_layout_side',[-.28,-4,.26],[-.28,0,.26],35,4.1),
        v('06_official_conveyor',[-1.8,-2.1,1.45],[-.28,-.20,.14],50),
        ('07_collision_overlay',*oblique[1:]),
        v('08_top_operating_space',[-1.5,-1.8,1.85],[.34,0,1.10],58)]


def diagnostic_visibility(stage,data,visible):
    path='/World/HypotheticalContactEnvelope';root=UsdGeom.Xform.Define(stage,path)
    if not root.GetPrim().GetChildren():
        origin=np.array(data['T_W_D'])[:3,3];b=max(data['cartons'],key=lambda b:(b['layer'],-abs(b['column']-1)))
        tcp=np.array(b['pose'])+origin;tcp[2]+=b['size_m'][2]/2
        bounds=np.array(data['top_geometry']['tool_bounds_TCP_m']);R=np.diag([1.,-1.,-1.])
        vertices=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])@R.T+tcp
        edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(vertices[i]!=vertices[j])==1]
        for suffix,offset,color in [('Contact',0.,(1.,.1,.05)),('Preapproach',data['top_geometry']['preapproach_m'],(1.,.8,.03))]:
            c=UsdGeom.BasisCurves.Define(stage,path+'/'+suffix);c.CreateTypeAttr('linear');c.CreateCurveVertexCountsAttr([2]*len(edges))
            v=vertices+[0,0,offset];c.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.array([v[i] for pair in edges for i in pair],np.float32)))
            c.CreateWidthsAttr([.003]);c.SetWidthsInterpolation('constant');c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        root.GetPrim().SetCustomDataByKey('status','HYPOTHETICAL_TOOL_ENVELOPE_NOT_ROBOT_POSE_NO_PHYSICS')
    root.MakeVisible() if visible else root.MakeInvisible()


def annotate(pixels,camera,stage,data,review,name):
    im=Image.fromarray(pixels);draw=ImageDraw.Draw(im);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24)
    def text(x,y,s,color='white'):
        b=draw.textbbox((x,y),s,font=font);draw.rectangle([b[0]-5,b[1]-4,b[2]+5,b[3]+4],fill='#152333');draw.text((x,y),s,font=font,fill=color)
    f=review['full_face'];g=f['top_geometry'];o=np.array(data['T_W_D'])[:3,3]
    cam=camera.GetCamera(Usd.TimeCode.Default());cam.transform=UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim());vp=cam.frustum.ComputeViewMatrix()*cam.frustum.ComputeProjectionMatrix()
    def project(v):
        p=vp.Transform(Gf.Vec3d(*v));return np.array([(p[0]+1)*960,(1-p[1])*540])
    def line(a,b,label):
        a=project(a);b=project(b);draw.line([tuple(a),tuple(b)],fill='#ffe16b',width=4)
        for p in [a,b]:draw.ellipse([*(p-4),*(p+4)],fill='#ffe16b')
        p=(a+b)/2;text(float(p[0]+8),float(p[1]-32),label,'#ffe16b')
    def arrow(a,b,label):
        a=project(a);b=project(b);d=(b-a)/np.linalg.norm(b-a);n=np.array([-d[1],d[0]])
        draw.line([tuple(a),tuple(b)],fill='#68e4bd',width=4)
        draw.polygon([tuple(b),tuple(b-15*d+6*n),tuple(b-15*d-6*n)],fill='#68e4bd')
        text(float(b[0]+8),float(b[1]-36),label,'#68e4bd')
    text(25,22,'STATIC_LAYOUT_REVIEW | 27-carton target | NO_MOTION_EXECUTED | Dynamics: NOT_EVALUATED')
    text(25,1030,'Top suction: BLOCKED | L-transfer: NOT_VALIDATED | inherited HOME: NOT_ACCEPTED for this new scene','#ffbc83')
    if name=='03_layout_top':
        text(25,70,f'USD gap {f["actual_gap_m"]*1000:.2f} mm | carton 250 x 210 mm | three columns, 10 mm column gaps')
        x=f['conveyor_most_protruding_x_D_m'];z=.19
        line(o+[x,.20,z],o+[f['box_front_x_D_m'],.20,z],'100 mm')
        for item in f['conveyors']:
            b=np.array(item['belt_bounds_W_m']);a=0 if item['id'].endswith('transverse') else 1;p=b.mean(0);p[2]=b[1,2]+.005
            lo=p.copy();hi=p.copy();lo[a]=b[0,a];hi[a]=b[1,a];line(lo,hi,f'{(hi[a]-lo[a])*1000:.1f} mm belt')
        text(25,115,'D axes: +X into trailer, +Y left | transverse -Y | longitudinal -X')
        arrow(o+[-.85,.30,.01],o+[-.55,.30,.01],'+X D')
        arrow(o+[-.85,.30,.01],o+[-.85,.55,.01],'+Y D')
        for item in f['conveyors']:
            b=np.array(item['belt_bounds_W_m']);p=b.mean(0);p[2]=b[1,2]+.01
            direction=np.array([0,-.20,0]) if item['id'].endswith('transverse') else np.array([-.20,0,0])
            arrow(p-direction/2,p+direction/2,'-Y' if item['id'].endswith('transverse') else '-X')
    elif name=='04_layout_front':
        text(25,70,f'USD: 3 x 9 = 27 | wall 650 x 900 mm | inner width {f["effective_width_m"]*1000:.2f} mm, height {f["effective_height_m"]*1000:.1f} mm')
        lo,hi=np.array(f['boxwall_bounds_W_m']);line([lo[0],lo[1],lo[2]],[lo[0],hi[1],lo[2]],'650 mm')
        line([lo[0],hi[1],lo[2]],[lo[0],hi[1],hi[2]],'900 mm')
    elif name=='05_layout_side':
        text(25,70,f'USD D-Z: pedestal/belt/bottom {f["pedestal_top_D_m"]:.3f} m | wall top/roof {f["roof_underside_D_m"]:.3f} m')
        text(25,110,'Table W-Z 0.800 m | pedestal is the effective cargo floor | roof is retained')
    elif name=='06_official_conveyor':
        text(25,70,'Official NVIDIA A06 (asset 5.0): uniform scale, lower supports cut at table; speed = 0')
        text(25,110,'Visible L-junction gap; no transfer mechanism or 90-degree transfer qualification','#ffbc83')
    elif name=='07_collision_overlay':text(25,70,'CYAN robot/tool | ORANGE carton proxies | MAGENTA equipment incl. exact A06 triangles')
    elif name=='08_top_operating_space':
        text(25,70,f'BLOCKED: actual headroom {g["available_headroom_m"]*1000:.1f} mm < required {g["required_headroom_m"]*1000:.2f} mm','#ffbc83')
        text(25,110,'RED = hypothetical contact tool envelope; YELLOW = +30 mm approach; neither is an achieved robot pose')
        text(25,150,f'Tool/flange height {g["contact_tool_height_m"]*1000:.2f} + approach 30 + clearance 2.40 mm; roof unchanged')
    else:text(25,70,'Full warehouse + new full-face layout; robot/tool at unchanged previous accepted HOME')
    if review['actual_HOME_surface_intersection_witnesses']:
        text(25,985,'GEOMETRIC ISSUE: inherited HOME intersects new carton wall; no new HOME was searched','#ff9191')
    return im
