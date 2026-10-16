"""Read back the actual nested USD attachment; draw diagnostic-only overlays."""
import numpy as np
from pxr import Usd,UsdGeom,Gf
from PIL import Image,ImageDraw,ImageFont
from eco65_full_face_review import audit as layout_audit


def audit(stage,data):
    report=layout_audit(stage,data);mast=data['mast'];cache=UsdGeom.XformCache()
    path=mast['usd_path'].replace('/Workstation','/World/Workstation',1)
    def matrix(p):return np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T
    link=matrix('/World/Workstation/Robot/link_1');M=matrix(path);C=matrix(path+'/CameraFrame')
    def compare(name,e,a):
        error=float(np.max(np.abs(np.array(e)-a)));report['checks'].append(dict(item=name,snapshot=np.asarray(e).tolist(),usd=np.asarray(a).tolist(),max_abs_delta=error,match=error<1e-8))
        if error>=1e-8:report['issues'].append(name)
    compare('J1_local_mast_transform',mast['T_link1_mast'],np.linalg.inv(link)@M)
    compare('mast_local_camera_transform',mast['T_mast_camera'],np.linalg.inv(M)@C)
    arm=link[:3,:3]@mast['arm_forward_link1'];cam=C[:3,2].copy();arm[2]=0;cam[2]=0
    arm/=np.linalg.norm(arm);cam/=np.linalg.norm(cam)
    signed=float(np.degrees(np.arctan2(np.cross(arm,cam)[2],arm@cam)))
    compare('USD_signed_heading_deg',mast['relative_heading_deg'],signed)
    seat=mast.get('mount_interface',{}).get('seat')
    if seat:
        import trimesh
        from pathlib import Path
        from pxr import UsdPhysics
        repo=Path(__file__).resolve().parents[1]
        original=trimesh.load(repo/seat['mesh'],process=False)
        for name in ['Visual','Collision']:
            seat_path='/World/Workstation/Robot/link_1/MountingSeat/'+name
            compare('seat_transform/'+name,link,matrix(seat_path))
            compare('seat_vertices/'+name,np.array(original.vertices),np.array(UsdGeom.Mesh(stage.GetPrimAtPath(seat_path)).GetPointsAttr().Get()))
        p=stage.GetPrimAtPath('/World/Workstation/Robot/link_1/MountingSeat/Collision')
        if UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()!='none':report['issues'].append('SEAT_COLLISION_NOT_EXACT_TRIANGLES')
    parent=str(stage.GetPrimAtPath(path).GetParent().GetPath())
    if parent!='/World/Workstation/Robot/link_1':report['issues'].append('WRONG_MAST_PARENT')
    report['j1_mast']=dict(case=data['home_source']['static_case'],parent=parent,
        T_W_link1=link.tolist(),T_W_mast=M.tolist(),T_W_camera=C.tolist(),
        arm_heading_W=arm.tolist(),camera_heading_W=cam.tolist(),signed_angle_deg=signed,
        source_camera_axis='CAD window normal +Y, camera frame +Z; fixed, not optically calibrated',
        mounting_status=mast['placement'],mount_interface=mast['mount_interface'],
        status='CAMERA_AXIS_90DEG_CONSTRAINT_APPLIED',joint_values=data['home_q_rad'])
    # Only AABB overlap is reported here; a convex envelope is not a CAD surface.
    potential=[];meshes={}
    for p in stage.Traverse():
        s=str(p.GetPath())
        if not p.IsA(UsdGeom.Mesh) or 'collision' in s.lower():continue
        if not s.startswith(('/World/Workstation/Robot/','/World/Workstation/Tool/')):continue
        v=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get());T=matrix(s);v=v@T[:3,:3].T+T[:3,3]
        meshes[s]=np.array([v.min(0),v.max(0)])
    for s,b in meshes.items():
        if '/Mast/' not in s:continue
        for other,bb in meshes.items():
            if '/Mast/' in other:continue
            overlap=np.minimum(b[1],bb[1])-np.maximum(b[0],bb[0])
            if np.all(overlap>1e-6):potential.append(dict(pair=[s,other],overlap_m=overlap.tolist(),status='AABB_POTENTIAL_ONLY'))
    report['j1_mast']['potential_interference']=potential
    return report


def views(stage,data):
    # Same world camera in A/B/C/D; camera never drives the physical accessory.
    o=np.array(data['T_W_D'])[:3,3];base=np.array(data['base_position_D_m'])+o
    def v(name,eye,target,f=42,w=None):return name,tuple(base+eye),tuple(base+target),f,w
    result=[v('01_j1_installation',[-1.05,1.15,.9],[-.04,.04,.40],40),
        v('02_heading_layout_top',[0,0,2.4],[0,0,.4],35,1.65),
        v('03_interface_closeup',[-.55,.65,.48],[-.045,.025,.205],65),
        v('04_workstation',[-3,-3,2],[-.05,-.05,.05],40)]
    return result if data['home_source']['static_case']=='A' else result[:2]


def annotate(pixels,camera,stage,data,review,name):
    im=Image.fromarray(pixels).copy();draw=ImageDraw.Draw(im)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',25)
    cam=camera.GetCamera(Usd.TimeCode.Default());cam.transform=UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim())
    vp=cam.frustum.ComputeViewMatrix()*cam.frustum.ComputeProjectionMatrix()
    def project(v):
        p=vp.Transform(Gf.Vec3d(*v));return np.array([(p[0]+1)*960,(1-p[1])*540])
    def text(x,y,s,color='white'):
        b=draw.textbbox((x,y),s,font=font);draw.rectangle([b[0]-5,b[1]-3,b[2]+5,b[3]+3],fill='#152333');draw.text((x,y),s,font=font,fill=color)
    def arrow(a,b,color):
        a,b=project(a),project(b);d=(b-a)/np.linalg.norm(b-a);n=np.array([-d[1],d[0]])
        draw.line([tuple(a),tuple(b)],fill=color,width=5);draw.polygon([tuple(b),tuple(b-18*d+7*n),tuple(b-18*d-7*n)],fill=color)
    r=review['j1_mast'];L=np.array(r['T_W_link1']);C=np.array(r['T_W_camera']);u=np.array(r['arm_heading_W']);v=np.array(r['camera_heading_W'])
    text(25,20,'CAMERA_AXIS_90DEG_CONSTRAINT_APPLIED | NO_MOTION_EXECUTED')
    text(25,62,f"Case {r['case']} | J1 {np.degrees(data['home_q_rad'][0]):.3f} deg | signed angle {r['signed_angle_deg']:+.6f} deg")
    text(25,1030,'STATIC INSTALLATION REVIEW | Dynamics: NOT_EVALUATED | mounting holes / optical calibration: NOT_VERIFIED','#ffbc83')
    if name in ['01_j1_installation','02_heading_layout_top']:
        p=L[:3,3].copy();p[2]=C[2,3]+.025
        arrow(C[:3,3],C[:3,3]+v*.22,'#60dfef')
        arrow(p,p+u*.27,'#ffbd53');arrow(p,p+v*.27,'#60dfef')
        square=[p+u*.07,p+(u+v)*.07,p+v*.07]
        draw.line([tuple(project(x)) for x in square],fill='white',width=4)
        label=project(p+(u+v)*.10)
        text(float(label[0]),float(label[1]),f"{r['signed_angle_deg']:+.0f} deg",'#60dfef')
        text(25,110,'ORANGE: arm heading = link_1 local -X','#ffbd53')
        text(25,151,'CYAN: camera optical axis / horizontal projection','#60dfef')
        text(25,192,f"Horizontal angle {r['signed_angle_deg']:+.0f} deg; arrows translated to common height")
        a,b=project(L[:3,3]),project(p);draw.line([tuple(a),tuple(b)],fill='#bbbbbb',width=2)
    elif name=='03_interface_closeup':
        text(25,110,'Silver CAD connector -> black mast -> original CAD camera')
        label='Rigid parent: link_1; mounting fit is a simulation assembly assumption'
        if data['mast'].get('mount_interface',{}).get('seat'):
            label='Original surfaces connected by a project-derived conformal mounting seat'
        text(25,151,label,'#ffbc83')
    elif name=='04_workstation':text(25,110,'Existing 12-carton / four-layer warehouse; all other layout objects unchanged')
    if r['potential_interference']:text(25,982,'DIAGNOSTIC: mast/robot AABB overlap recorded; not a proven surface penetration','#ffbc83')
    return im
