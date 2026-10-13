"""Export the same local geometry and finite assumptions as an OpenUSD physics stage.
No Isaac import, no physics success claim, no hardware provider or remote transfer.
"""
import json
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from .model import *

def export_isaac():
    from pxr import Usd,UsdGeom,UsdPhysics,Gf,Vt
    tool=load_json(LOCAL/"tool.json");scene=load_json(LOCAL/"scene.json");require_simulation(scene)
    robot=robot_for(tool,scene);home=np.array(load_json(OUTPUT/"reports/home.json")["q"])
    folder=OUTPUT/"isaac_export";folder.mkdir(parents=True,exist_ok=True)
    path=folder/"eco65_desktop.usda";stage=Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.SetTimeCodesPerSecond(250)
    world=UsdGeom.Xform.Define(stage,"/World");stage.SetDefaultPrim(world.GetPrim())
    phys=UsdPhysics.Scene.Define(stage,"/World/Physics");phys.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));phys.CreateGravityMagnitudeAttr(9.81)
    def pose(prim,T):
        xf=UsdGeom.Xformable(prim);xf.AddTranslateOp().Set(Gf.Vec3d(*T[:3,3]))
        q=Rotation.from_matrix(T[:3,:3]).as_quat();xf.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    def mass(prim,m,center,inertia):
        api=UsdPhysics.MassAPI.Apply(prim);api.CreateMassAttr(float(m));api.CreateCenterOfMassAttr(Gf.Vec3f(*center))
        eigen,axes=np.linalg.eigh(inertia)
        if np.linalg.det(axes)<0:axes[:,0]*=-1
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.maximum(eigen,1e-8)))
        q=Rotation.from_matrix(axes).as_quat();api.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
    def mesh(path,m,color,collision=False):
        obj=UsdGeom.Mesh.Define(stage,path);obj.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(m.vertices,dtype=np.float32)))
        obj.CreateFaceVertexCountsAttr([3]*len(m.faces));obj.CreateFaceVertexIndicesAttr(np.asarray(m.faces,dtype=np.int32).ravel().tolist())
        obj.CreateSubdivisionSchemeAttr("none");obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr("convexHull")
            obj.MakeInvisible()
        return obj.GetPrim()
    def rigid(path,T):
        prim=UsdGeom.Xform.Define(stage,path).GetPrim();pose(prim,T);UsdPhysics.RigidBodyAPI.Apply(prim);return prim
    def fixed(path,body0,body1,T0=np.eye(4),T1=np.eye(4)):
        j=UsdPhysics.FixedJoint.Define(stage,path)
        if body0:j.CreateBody0Rel().SetTargets([body0])
        j.CreateBody1Rel().SetTargets([body1])
        for index,T in enumerate([T0,T1]):
            getattr(j,f"CreateLocalPos{index}Attr")(Gf.Vec3f(*T[:3,3]))
            q=Rotation.from_matrix(T[:3,:3]).as_quat();getattr(j,f"CreateLocalRot{index}Attr")(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
        return j
    UsdGeom.Xform.Define(stage,"/World/Robot")
    frames=robot.named_link_frames(home);root=ET.parse(URDF).getroot()
    for link in root.findall("link"):
        name=link.attrib["name"];body=rigid("/World/Robot/"+name,frames[name]);geo=link.find("visual")
        file=URDF.parents[1]/geo.find("geometry/mesh").attrib["filename"].split("package://ECO65-B/")[1]
        m=trimesh.load(file,process=False);mesh(str(body.GetPath())+"/visual",m,[.8,.83,.86]);mesh(str(body.GetPath())+"/collision",m.convex_hull,[.2,.7,.8],True)
        inert=link.find("inertial");origin=inert.find("origin");center=np.fromstring(origin.attrib["xyz"],sep=" ")
        r=Rotation.from_euler("xyz",np.fromstring(origin.attrib.get("rpy","0 0 0"),sep=" ")).as_matrix()
        a=inert.find("inertia").attrib;I=np.array([[float(a["ixx"]),float(a["ixy"]),float(a["ixz"])],[float(a["ixy"]),float(a["iyy"]),float(a["iyz"])],[float(a["ixz"]),float(a["iyz"]),float(a["izz"])]])
        mass(body,float(inert.find("mass").attrib["value"]),center,r@I@r.T)
    base=stage.GetPrimAtPath("/World/Robot/base_link");UsdPhysics.ArticulationRootAPI.Apply(base)
    fixed("/World/Robot/base_anchor",None,str(base.GetPath()),frames["base_link"],np.eye(4))
    joint_records=[]
    for node,value in zip(root.findall("joint"),home):
        name=node.attrib["name"];parent=node.find("parent").attrib["link"];child=node.find("child").attrib["link"]
        origin=node.find("origin");T=transform(np.fromstring(origin.attrib["xyz"],sep=" "),Rotation.from_euler("xyz",np.fromstring(origin.attrib["rpy"],sep=" ")).as_matrix())
        axis=np.fromstring(node.find("axis").attrib["xyz"],sep=" ");align=Rotation.align_vectors([axis],[[1,0,0]])[0].as_matrix()
        T0=T@transform(rotation=align);T1=transform(rotation=align)
        j=UsdPhysics.RevoluteJoint.Define(stage,"/World/Robot/"+name);j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets(["/World/Robot/"+parent]);j.CreateBody1Rel().SetTargets(["/World/Robot/"+child])
        for k,F in enumerate([T0,T1]):
            getattr(j,f"CreateLocalPos{k}Attr")(Gf.Vec3f(*F[:3,3]));q=Rotation.from_matrix(F[:3,:3]).as_quat()
            getattr(j,f"CreateLocalRot{k}Attr")(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
        lim=node.find("limit").attrib;j.CreateLowerLimitAttr(float(np.degrees(float(lim["lower"]))));j.CreateUpperLimitAttr(float(np.degrees(float(lim["upper"]))))
        drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),"angular");drive.CreateTypeAttr("force");drive.CreateStiffnessAttr(200.)
        drive.CreateDampingAttr(20.);drive.CreateMaxForceAttr(float(lim["effort"]));drive.CreateTargetPositionAttr(float(np.degrees(value)))
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath("/World/Robot/"+parent)).CreateFilteredPairsRel().AddTarget("/World/Robot/"+child)
        joint_records.append(dict(name=name,axis=node.find("axis").attrib["xyz"],limits=lim,drive_target_deg=float(np.degrees(value))))
    flange=frames["link_6"]
    body=rigid("/World/Tool",flange@np.array(tool["T_flange_tool_cad"]))
    allbounds=np.array([p["bounds_m"] for p in tool["visual_parts"]]);lo=allbounds[:,0].min(axis=0);hi=allbounds[:,1].max(axis=0);dims=hi-lo
    mass(body,tool["physics"]["tool_mass_kg"],(hi+lo)/2,np.diag([(dims[1]**2+dims[2]**2)/12,(dims[0]**2+dims[2]**2)/12,(dims[0]**2+dims[1]**2)/12])*tool["physics"]["tool_mass_kg"])
    for p in tool["visual_parts"]:mesh("/World/Tool/visual_"+p["id"],trimesh.load(ROOT/p["visual"],process=False),p["rgb"] or [.6,.6,.6])
    for c in tool["collisions"]:mesh("/World/Tool/collision_"+c["id"],trimesh.load(ROOT/c["mesh"],process=False),[.1,.8,.8],True)
    fixed("/World/tool_mount","/World/Robot/link_6","/World/Tool",np.array(tool["T_flange_tool_cad"]))
    body=rigid("/World/Adapter",flange);total=[]
    for a in tool["adapter"]:
        m=trimesh.creation.box(a["size"]) if a["kind"]=="box" else trimesh.creation.cylinder(radius=a["radius"],height=2*a["half_length"],sections=48)
        m.apply_translation(a["position"]);total.append(m)
        mesh("/World/Adapter/visual_"+a["id"],m,[.85,.57,.15]);mesh("/World/Adapter/collision_"+a["id"],m,[.8,.5,.1],True)
    combined=trimesh.util.concatenate(total);mp=combined.mass_properties
    mass(body,tool["physics"]["adapter_mass_kg"],mp["center_mass"],mp["inertia"]*tool["physics"]["adapter_mass_kg"]/mp["mass"])
    fixed("/World/adapter_mount","/World/Robot/link_6","/World/Adapter")
    # Only the explicit attached interface pairs; never whole wrist/tool versus environment.
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath("/World/Robot/link_6/collision")).CreateFilteredPairsRel().AddTarget("/World/Adapter/collision_adapter_flange_disk")
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath("/World/Adapter")).CreateFilteredPairsRel().AddTarget("/World/Tool")
    for obj in scene["obstacles"]+[dict(id=scene["box"]["id"],position_m=scene["box"]["pose"],size_m=scene["box"]["size_m"],rgba=[.65,.43,.22,1])]:
        pathobj="/World/"+obj["id"];cube=UsdGeom.Cube.Define(stage,pathobj);cube.CreateSizeAttr(1.)
        pose(cube.GetPrim(),transform(obj["position_m"]));UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(*obj["size_m"]))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*obj["rgba"][:3])]);UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if obj["id"]==scene["box"]["id"]:
            UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim());UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(scene["box"]["mass_kg"])
    UsdPhysics.FilteredPairsAPI.Apply(base).CreateFilteredPairsRel().AddTarget("/World/bench")
    stage.GetRootLayer().Save()
    reopened=Usd.Stage.Open(str(path));assert reopened and reopened.GetPrimAtPath("/World/Tool")
    artifact=load_json(OUTPUT/"trajectory/plan.json")
    validation=load_json(OUTPUT/"reports/validation.json")
    assert validation["status"]=="VALID" and validation["trajectory_fingerprint"]==artifact["trajectory_fingerprint"]
    save_json(folder/"validated_trajectory.json",artifact)
    manifest=dict(status="EXPORTED_NOT_PHYSICS_VALIDATED",dynamics="NOT_EVALUATED",stage=path.name,
        trajectory="validated_trajectory.json",joint_order=robot.active_joint_names,joints=joint_records,
        interpolation="same C2 quintic; drive angular target units degrees, source q radians",simulation_timestep_s=.004,
        expected_actions=["drive validated approach/contact","confirm all seals against actual carton pose","create ideal_suction fixed joint at actual relative transform",
                          "drive loaded trajectory","confirm actual receiver support contact","remove fixed joint without pose/velocity reset","drive retreat"],
        limitations=["Isaac runtime unavailable locally; no contact, drive tracking or payload qualification evaluated.",
                     "USD export is a reproducible physical-scene starting point, not an executed suction controller.",
                     "Tool 1kg and adapter0.25kg/inertia are simulation assumptions; official robot inertials preserved.",
                     "No per-frame pose-setting dynamics claim, no hardware provider, no asset upload."])
    save_json(folder/"manifest.json",manifest)
    print("ISAAC USD EXPORT",path,flush=True)
    return manifest
