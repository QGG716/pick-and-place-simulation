"""Isaac-only articulation authoring for the bound one-point experiment."""
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import trimesh
from unloading_sim.eco65.model import URDF,ROOT,transform,save_json


def author(stage,source,s,c,context,g,q,out):
    from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,Sdf,Vt
    station='/World/Workstation';robot_path=station+'/Robot';TWD=np.array(s['scene']['T_W_D'])
    def pose(path,T):
        prim=stage.GetPrimAtPath(path) or UsdGeom.Xform.Define(stage,path).GetPrim()
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(np.asarray(T).T.tolist()));return prim
    def quat(R):return Gf.Quatf(Gf.Matrix3d(np.asarray(R).T.tolist()).ExtractRotation().GetQuat())
    frames=g.robot.named_link_frames(q)
    for name,T in frames.items():pose(robot_path+'/'+name,T)
    pose(station+'/CalibrationBoard',context['P'])
    # Tool visuals become actual link_6 children, not per-frame FK copies.
    tool_path=robot_path+'/link_6/RealTool';p=UsdGeom.Xform.Define(stage,tool_path).GetPrim()
    p.GetReferences().AddReference(str(source),station+'/Tool')
    pose(tool_path,np.eye(4))
    for child in p.GetChildren():pose(str(child.GetPath()),s['tool']['T_flange_tool_cad'])
    stage.OverridePrim(station+'/Tool').SetActive(False)
    for name in ['Frames','Adapter']:stage.OverridePrim(station+'/'+name).SetActive(False)
    # Exactly one collision representation. Historical visual/proxy schemas
    # remain in source files, but are inactive for this derived physics stage.
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdPhysics.Scene):prim.SetActive(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    paths={};mass_components={name:[] for name in frames}
    def collider(path,mesh,T):
        obj=UsdGeom.Mesh.Define(stage,path);obj.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(mesh.vertices,np.float32)))
        obj.CreateFaceVertexCountsAttr([3]*len(mesh.faces));obj.CreateFaceVertexIndicesAttr(mesh.faces.reshape(-1).tolist());obj.CreateSubdivisionSchemeAttr('none')
        obj.CreateVisibilityAttr('invisible');pose(path,T);UsdPhysics.CollisionAPI.Apply(obj.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr('convexHull')
        collision=PhysxSchema.PhysxCollisionAPI.Apply(obj.GetPrim());collision.CreateContactOffsetAttr(.00002);collision.CreateRestOffsetAttr(0.)
        return obj.GetPrim()
    for i,e in enumerate(g.entries):
        mesh=trimesh.convex.convex_hull(e['vertices']);T=e['T'];name=e['owner']
        if e['frame'] in ['D','board']:
            path='/World/TouchPhysics/Static/g'+str(i);T=TWD@(np.array(context['P']) if e['frame']=='board' else np.eye(4))@T
        else:
            path=robot_path+'/'+e['frame']+'/TouchCollisions/g'+str(i)
            if e['kind']!='robot':
                m=mesh.copy();m.apply_transform(T);mass_components[e['frame']].append((e['kind'],m))
        collider(path,mesh,T);paths[e['name']]=path
    # Filter only the exact original robot adjacency and installation pairs.
    # No joint-wide wrist/tool exclusion; joint collisions are enabled below.
    eligible={frozenset([a['name'],b['name']]) for a,b,_ in g.pairs}
    filters=[]
    for i,a in enumerate(g.entries):
        for b in g.entries[i+1:]:
            if a['frame'] in ['D','board'] and b['frame'] in ['D','board']:continue
            if frozenset([a['name'],b['name']]) not in eligible:
                UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths[a['name']])).CreateFilteredPairsRel().AddTarget(paths[b['name']]);filters.append([a['owner'],b['owner']])
    for a,b,rule in g.pairs:
        if rule['threshold_m']<0:
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths[a['name']])).CreateFilteredPairsRel().AddTarget(paths[b['name']]);filters.append([a['owner'],b['owner']])
    official=ET.parse(URDF).getroot();mass_records={}
    for name in frames:
        body=stage.GetPrimAtPath(robot_path+'/'+name);UsdPhysics.RigidBodyAPI.Apply(body)
        pa=PhysxSchema.PhysxRigidBodyAPI.Apply(body);pa.CreateSolverPositionIterationCountAttr(64);pa.CreateSolverVelocityIterationCountAttr(16)
        PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.)
        inertial=official.find(f"link[@name='{name}']/inertial");origin=inertial.find('origin')
        mass=float(inertial.find('mass').get('value'));centre=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        R=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix();v=inertial.find('inertia')
        I=np.array([[float(v.get('i'+a+b)) for b in 'xyz'] for a in 'xyz']) if all(v.get('i'+a+b) for a in 'xyz' for b in 'xyz') else np.array([[float(v.get('ixx')),float(v.get('ixy')),float(v.get('ixz'))],[float(v.get('ixy')),float(v.get('iyy')),float(v.get('iyz'))],[float(v.get('ixz')),float(v.get('iyz')),float(v.get('izz'))]])
        I=R@I@R.T;pieces=[(mass,centre,I,'official_URDF')]
        extras=mass_components[name]
        if extras:
            # Preserve the existing 1 kg tool simulation input. Mast/camera
            # have no measured inertia; explicitly freeze a 1 kg assumption.
            for category in sorted({k for k,m in extras}):
                meshes=[m for k,m in extras if k==category];volume=sum(m.volume for m in meshes)
                target=float(s['tool']['physics']['tool_mass_kg']) if category=='tool' else (volume*2700 if category=='probe' else None)
                if target is None:
                    all_accessory_volume=sum(m.volume for k,m in extras if k not in ['tool','probe'])
                    target=volume/all_accessory_volume
                for m in meshes:
                    m.density=target/volume;pieces.append((m.mass,m.center_mass,m.moment_inertia,'simulation_'+category))
        total=sum(p[0] for p in pieces);com=sum(p[0]*p[1] for p in pieces)/total;inertia=np.zeros((3,3))
        for weight,point,tensor,_ in pieces:
            d=point-com;inertia+=tensor+weight*(np.eye(3)*(d@d)-np.outer(d,d))
        moments,axes=np.linalg.eigh(inertia)
        if min(moments)<=0:raise ValueError('FINITE_POSITIVE_INERTIA_REQUIRED')
        if np.linalg.det(axes)<0:axes[:,0]*=-1
        api=UsdPhysics.MassAPI.Apply(body);api.CreateMassAttr(float(total));api.CreateCenterOfMassAttr(Gf.Vec3f(*com));api.CreateDiagonalInertiaAttr(Gf.Vec3f(*moments));api.CreatePrincipalAxesAttr(quat(axes))
        mass_records[name]=dict(mass_kg=total,com_m=com.tolist(),inertia_kg_m2=inertia.tolist(),parts=[dict(mass_kg=float(p[0]),source=p[3]) for p in pieces])
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(robot_path))
    PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(robot_path)).CreateEnabledSelfCollisionsAttr(True)
    fixed=UsdPhysics.FixedJoint.Define(stage,robot_path+'/PhysicsJoints/base_fixed');fixed.CreateBody1Rel().SetTargets([robot_path+'/base_link'])
    Wbase=TWD@frames['base_link'];fixed.CreateLocalPos0Attr(Gf.Vec3f(*Wbase[:3,3]));fixed.CreateLocalRot0Attr(quat(Wbase[:3,:3]))
    efforts=[];velocities=[];stiffness=[8000.,8000.,6000.,3000.,3000.,3000.];damping=[200.,200.,150.,80.,80.,80.]
    for index,j in enumerate(g.robot.joints):
        if j.name not in g.robot.active_joint_names:continue
        k=g.robot.active_joint_names.index(j.name);joint=UsdPhysics.RevoluteJoint.Define(stage,robot_path+'/PhysicsJoints/'+j.name)
        joint.CreateBody0Rel().SetTargets([robot_path+'/'+j.parent]);joint.CreateBody1Rel().SetTargets([robot_path+'/'+j.child])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*j.origin[:3,3]));joint.CreateLocalRot0Attr(quat(j.origin[:3,:3]));joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(Gf.Quatf(1))
        axis=np.asarray(j.axis);axis_index=int(np.argmax(abs(axis)))
        if not np.allclose(axis,np.eye(3)[axis_index]):raise ValueError('UNSUPPORTED_OFFICIAL_JOINT_AXIS')
        joint.CreateAxisAttr('XYZ'[axis_index]);joint.CreateLowerLimitAttr(float(np.degrees(g.robot.joint_limits[k,0])));joint.CreateUpperLimitAttr(float(np.degrees(g.robot.joint_limits[k,1])));joint.CreateCollisionEnabledAttr(True)
        limit=official.find(f"joint[@name='{j.name}']/limit");efforts.append(float(limit.get('effort')));velocities.append(min(.55,float(limit.get('velocity'))))
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateMaxForceAttr(efforts[-1]);drive.CreateStiffnessAttr(stiffness[k]);drive.CreateDampingAttr(damping[k]);drive.CreateTargetPositionAttr(float(np.degrees(q[k])));drive.CreateTargetVelocityAttr(0.)
        state=PhysxSchema.JointStateAPI.Apply(joint.GetPrim(),'angular');state.CreatePositionAttr(float(np.degrees(q[k])));state.CreateVelocityAttr(0.)
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateMaxJointVelocityAttr(float(np.degrees(velocities[-1])))
    report=dict(robot_root=robot_path,masses=mass_records,effort_limits_nm=efforts,velocity_limits_rad_s=velocities,stiffness=stiffness,damping=damping,
        mass_assumptions='Original tool 1 kg unchanged; mast+camera 1 kg total, new metal probe at 2700 kg/m^3. Explicit simulation assumptions, not measured hardware inertia.',
        collision_count=len(paths),colliders={paths[e['name']]:dict(owner=e['owner'],frame=e['frame'],kind=e['kind']) for e in g.entries},filters=filters,
        contact_offset_m=.00002,rest_offset_m=0.,initial_state_sets=1,per_frame_pose_writes=False,articulation_self_collision=True,
        adjacent_body_note='PhysX articulation adjacency is solver-controlled; exact native/MuJoCo pair checks additionally retain tool-versus-neighbor-link clearance')
    save_json(out/'physics_model.json',report);return report
