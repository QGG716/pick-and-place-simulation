"""ECO65-B model, private scene, and complete convex-mesh distance checks."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ..robot import URDFRobot
from .observation_contracts import Pose3D, EvidenceKind, canonical_fingerprint

ROOT=Path(__file__).resolve().parents[3]
OFFICIAL=ROOT/"assets/robots/realman_eco65/official"
URDF=OFFICIAL/"description/rm_models/ECO65/urdf/ECO65-B/urdf/ECO65-B.urdf"
LOCAL=ROOT/"configs/local/eco65_desktop"
OUTPUT=ROOT/"outputs/eco65_desktop_round1/round1"

def load_json(p):return json.loads(Path(p).read_text(encoding="utf-8"))
def save_json(p,value):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
def transform(position=(0,0,0),rotation=None):
    t=np.eye(4);t[:3,3]=position
    if rotation is not None:t[:3,:3]=rotation
    return t
def check_transform(t):
    t=np.asarray(t,float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-10) or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-8) or not np.isclose(np.linalg.det(t[:3,:3]),1):
        raise ValueError("Invalid rigid SE(3), no scale/reflection permitted")
    return t
def require_simulation(config):
    if config.get("enable_hardware",False) or config.get("execution_mode","simulation")!="simulation":
        raise PermissionError("HARDWARE_DISABLED: this implementation has no hardware execution provider")
def asset_check():
    m=load_json(OFFICIAL/"SOURCE_MANIFEST.json")
    for e in m["entries"]:
        if "rm_models/ECO65" in e["local_path"]:
            p=OFFICIAL/e["local_path"]
            if hashlib.sha256(p.read_bytes()).hexdigest()!=e["sha256"]:raise ValueError("Official asset hash mismatch: "+str(p))
    cad=load_json(ROOT/"assets/tools/desktop_suction/derived/manifests/cad.json")
    if hashlib.sha256((ROOT/cad["source"]).read_bytes()).hexdigest()!=cad["source_sha256"]:
        raise ValueError("Private STEP changed: regenerate derived assets")
    return cad

def robot_for(tool,scene):
    require_simulation(scene)
    robot=URDFRobot.from_urdf(URDF,active_joint_names=[f"joint_{i}" for i in range(1,7)],
        base_link="base_link",tip_link="link_6",base_position=scene["base_position_m"],tool_length=0.,
        link_radii=[.055,.055,.050,.045,.040,.035],name="official_realman_eco65_b")
    robot.tip_from_tcp=check_transform(tool["T_flange_tcp"])
    robot.self_collision_exclusions=set() # No inherited capsule/FANUC policy; World owns mesh checks.
    return robot

def design_scene(tool):
    # Dimensions follow the actual seal footprint plus 20mm edge reserve on each side.
    rings=np.concatenate([s["ring_cad_m"] for s in tool["seals"]])
    dims=np.ceil((np.ptp(rings[:,[0,2]],axis=0)+.04)/.01)*.01
    size=[float(dims[0]),float(dims[1]),.10]
    top=.68
    scene=dict(schema="eco65_desktop_v1",execution_mode="simulation",enable_hardware=False,
        axis_convention="+X into trailer,+Y left,+Z up",base_position_m=[-.22,0,.64],
        robot_variant="ECO65-B",hardware_variant_verified=True,controller_generation=None,
        wall_configuration=dict(left=True,right=False,rear=True,roof=False),
        box=dict(id="carton_001",size_m=size,mass_kg=.3,pose=[.23,.14,top+.05],receiver_pose=[.23,-.23,top+.05],
                 evidence="simulation design known-pose fixture; not a camera measurement"),
        obstacles=[
            dict(id="bench",size_m=[1.2,.90,.06],position_m=[.12,0,.61],rgba=[.27,.32,.36,1]),
            dict(id="source_support",size_m=[max(size[0]+.05,.28),max(size[1]+.05,.22),.04],position_m=[.23,.14,.66],rgba=[.42,.50,.56,1]),
            dict(id="receiver",size_m=[max(size[0]+.08,.30),max(size[1]+.08,.25),.04],position_m=[.23,-.23,.66],rgba=[.18,.55,.48,1]),
            dict(id="left_panel",size_m=[.40,.015,.32],position_m=[.37,.38,.80],rgba=[.55,.61,.69,.45]),
            dict(id="rear_panel",size_m=[.015,.47,.32],position_m=[.57,.145,.80],rgba=[.55,.61,.69,.45])],
        margin_m=.002,mesh_error_reserve_m=.0002,contact_tolerance_m=.0004,
        interpolation="C2_piecewise_quintic_rest_to_rest",seed=6501,
        planning_budget_s=1200,ik_seeds=80,rrt_iterations=8000,
        joint_velocity_rad_s=.55,joint_acceleration_rad_s2=1.0,joint_jerk_rad_s3=4.,
        notes=["All scene dimensions/masses are simulation design values.","Open roof and right side; front is -X.",
               "No ideal outfeed, throw, drop or hardware."])
    save_json(LOCAL/"scene.json",scene);return scene

class World:
    """Mocap bodies here are geometric transforms ONLY; no dynamics success is inferred."""
    def __init__(self,tool,scene):
        import mujoco
        self.mj=mujoco;require_simulation(scene)
        self.tool,self.scene=tool,scene;self.robot=robot_for(tool,scene)
        self.boxes={b["id"]:b for b in (scene.get("boxes") or [scene["box"]])}
        self.flange_cad=check_transform(tool["T_flange_tool_cad"])
        assert np.allclose(self.flange_cad@check_transform(tool["T_tool_cad_tcp"]),self.robot.tip_from_tcp)
        self.entries=[];self.body_names=[];self.adjacent=set()
        xml=ET.Element("mujoco",model="eco65_geometric_replay")
        ET.SubElement(xml,"compiler",angle="radian",autolimits="true")
        ET.SubElement(xml,"option",gravity="0 0 -9.81",timestep=".004")
        vis=ET.SubElement(xml,"visual");ET.SubElement(vis,"global",offwidth="1280",offheight="720")
        ET.SubElement(vis,"headlight",diffuse=".8 .8 .8",ambient=".4 .4 .4",specular=".1 .1 .1")
        assets=ET.SubElement(xml,"asset");world=ET.SubElement(xml,"worldbody")
        ET.SubElement(world,"light",pos="0 -1 2",dir="0 0 -1",diffuse=".8 .8 .8")
        def mesh_asset(name,file):
            ET.SubElement(assets,"mesh",name=name,file=str(Path(file).resolve()).replace("\\","/"))
        def body(name):
            self.body_names.append(name)
            return ET.SubElement(world,"body",name=name,mocap="true")
        def mesh_geom(b,name,file,rgba,kind,owner,local=np.eye(4),visible=True):
            mesh_asset(name,file)
            quat=Rotation.from_matrix(local[:3,:3]).as_quat()[[3,0,1,2]]
            ET.SubElement(b,"geom",name=name,type="mesh",mesh=name,pos=" ".join(map(str,local[:3,3])),quat=" ".join(map(str,quat)),
                rgba=" ".join(map(str,rgba)),group="0" if visible else "3",contype="0",conaffinity="0",mass=".01")
            self.entries.append(dict(name=name,kind=kind,owner=owner))
        ur=ET.parse(URDF).getroot()
        for link in ur.findall("link"):
            name=link.attrib["name"];geo=link.find("visual");org=geo.find("origin")
            xyz=[float(x) for x in org.attrib.get("xyz","0 0 0").split()];rpy=[float(x) for x in org.attrib.get("rpy","0 0 0").split()]
            T=transform(xyz,Rotation.from_euler("xyz",rpy).as_matrix())
            file=URDF.parents[1]/geo.find("geometry/mesh").attrib["filename"].split("package://ECO65-B/")[1]
            mesh_geom(body(name),"robot_"+name,file,[.80,.83,.86,1],"robot",name,T)
        for j in self.robot.joints:self.adjacent.add(frozenset([j.parent,j.child]))
        for p in tool["visual_parts"]:
            b=body("tool_"+p["id"])
            mesh_geom(b,"visual_"+p["id"],ROOT/p["visual"],(p["rgb"] or [.65,.65,.65])+[1],"visual",p["id"])
            for c in tool["collisions"]:
                if c["source_part"]==p["id"]:
                    mesh_geom(b,c["id"],ROOT/c["mesh"],[.1,.8,.9,.18],c["semantic"],p["id"],visible=False)
        for a in tool["adapter"]:
            b=body(a["id"]);attrs=dict(name=a["id"],type=a["kind"],pos=" ".join(map(str,a["position"])),rgba=".85 .57 .15 1",contype="0",conaffinity="0")
            attrs["size"]=" ".join(map(str,np.array(a["size"])/2)) if a["kind"]=="box" else f'{a["radius"]} {a["half_length"]}'
            ET.SubElement(b,"geom",**attrs);self.entries.append(dict(name=a["id"],kind="adapter",owner=a["id"]))
        for obj in scene["obstacles"]+[dict(id=b["id"],size_m=b["size_m"],position_m=b["pose"],rgba=b.get("rgba",[.65,.43,.22,1])) for b in self.boxes.values()]:
            b=body(obj["id"]);ET.SubElement(b,"geom",name=obj["id"],type="box",size=" ".join(map(str,np.array(obj["size_m"])/2)),rgba=" ".join(map(str,obj["rgba"])),contype="0",conaffinity="0")
            self.entries.append(dict(name=obj["id"],kind="payload" if obj["id"] in self.boxes else "environment",owner=obj["id"]))
        self.xml=ET.tostring(xml,encoding="unicode")
        self.model=mujoco.MjModel.from_xml_string(self.xml);self.data=mujoco.MjData(self.model)
        for e in self.entries:e["gid"]=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,e["name"])
        self.geoms={e["name"]:e for e in self.entries}
        self.box_id=scene["box"]["id"];self.calls=0;self.min_clearance=1.;self.last_failure=None
        self.payload_relative=None;self.released_pose=None
        self.pairs=[]
        es=[e for e in self.entries if e["kind"]!="visual"]
        for i,a in enumerate(es):
            for b in es[i+1:]:
                if a["kind"]==b["kind"]=="environment":continue
                ta=a["kind"] in ("adapter","conservative_rigid","terminal_contact_band")
                tb=b["kind"] in ("adapter","conservative_rigid","terminal_contact_band")
                if ta and tb:continue
                if a["kind"]==b["kind"]=="robot" and frozenset([a["owner"],b["owner"]]) in self.adjacent:continue
                self.pairs.append((a,b))
        self.pair_a=np.array([a["gid"] for a,b in self.pairs]);self.pair_b=np.array([b["gid"] for a,b in self.pairs])
        self.radii=self.model.geom_rbound[self.pair_a]+self.model.geom_rbound[self.pair_b]
        self.set_state(np.zeros(6),"initial")
    def pose_body(self,name,T):
        idx=self.model.body(name).mocapid[0]
        self.data.mocap_pos[idx]=T[:3,3]
        self.data.mocap_quat[idx]=Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]
    def box_pose(self,q,phase):
        if phase in ("loaded_lift","loaded_transfer","place_contact"):
            if self.payload_relative is None:raise ValueError("missing actual attachment transform")
            return self.robot.fk(q)@self.payload_relative
        return self.released_pose if phase=="retreat" and self.released_pose is not None else transform(self.scene["box"]["pose"])
    def set_state(self,q,phase):
        frames=self.robot.named_link_frames(q);flange=frames["link_6"];cad=flange@self.flange_cad
        for name,T in frames.items():self.pose_body(name,T)
        for p in self.tool["visual_parts"]:self.pose_body("tool_"+p["id"],cad)
        for a in self.tool["adapter"]:self.pose_body(a["id"],flange)
        for o in self.scene["obstacles"]:self.pose_body(o["id"],transform(o["position_m"]))
        box=self.box_pose(q,phase)
        for name,carton in self.boxes.items():self.pose_body(name,box if name==self.box_id else transform(carton["pose"]))
        self.mj.mj_forward(self.model,self.data);return cad,box
    def seal_fit(self,q,box):
        T=self.robot.named_link_frames(q)["link_6"]@self.flange_cad
        inv=np.linalg.inv(box);half=np.array(self.scene["box"]["size_m"])/2
        worst=0.
        normal=inv[:3,:3]@T[:3,:3]@np.array([0,-1,0])
        if np.dot(normal,[0,0,-1])<np.cos(self.tool["contact"]["normal_tolerance_rad"]):return False,"seal_normal"
        for ring in self.tool["seals"]:
            pts=np.array(ring["ring_cad_m"]);pts=(pts@T[:3,:3].T+T[:3,3]-box[:3,3])@box[:3,:3]
            if np.any(np.abs(pts[:,:2])>half[:2]-.003):return False,"seal_outside_target"
            error=np.max(np.abs(pts[:,2]-half[2]));worst=max(worst,float(error))
        return worst<=self.scene["contact_tolerance_m"],f"seal_plane_error={worst:.7f}"
    def support_fit(self,box,support):
        o=next(x for x in self.scene["obstacles"] if x["id"]==support);half=np.array(self.scene["box"]["size_m"])/2
        corners=np.array([[x,y,z] for x in [-half[0],half[0]] for y in [-half[1],half[1]] for z in [-half[2],half[2]]])@box[:3,:3].T+box[:3,3]
        ohalf=np.array(o["size_m"])/2;pos=np.array(o["position_m"]);z=pos[2]+ohalf[2]
        return (abs(corners[:,2].min()-z)<=self.scene["contact_tolerance_m"] and
                np.all(np.abs(corners[:,:2]-pos[:2])<=ohalf[:2]+1e-8) and np.allclose(box[:3,2],[0,0,1],atol=.003))
    def valid(self,q,phase,detail=False):
        self.calls+=1
        if not np.isfinite(q).all() or not self.robot.within_limits(q):
            self.last_failure=dict(reason="joint_limits",phase=phase);return False
        cad,box=self.set_state(q,phase)
        margin=self.scene["margin_m"]+2*self.scene["mesh_error_reserve_m"]
        centers=self.data.geom_xpos
        distances=np.linalg.norm(centers[self.pair_a]-centers[self.pair_b],axis=1)-self.radii
        for index in np.flatnonzero(distances<=margin):
                a,b=self.pairs[index]
                pair={a["name"],b["name"]}
                allowance=None
                if pair=={"robot_base_link","bench"}:allowance="fixed_base_mount"
                if pair=={"robot_link_6","adapter_flange_disk"}:allowance="named_flange_mount"
                if self.box_id in pair:
                    other=b if a["name"]==self.box_id else a
                    if other["kind"]=="terminal_contact_band" and phase in ("contact","loaded_lift","loaded_transfer","place_contact","retreat"):
                        # Entire cup is NOT exempt: only the CAD terminal skirt band, and only aligned/top contact.
                        T=self.robot.fk(q);boxlocal=np.linalg.inv(box)@T
                        normal=boxlocal[:3,2]
                        if normal[2]<-np.cos(.01) and boxlocal[2,3]>=self.scene["box"]["size_m"][2]/2-.0004:
                            allowance="terminal_lip_target_top"
                    if other["name"] in ("source_support","receiver"):
                        pos=np.array(next(o["position_m"] for o in self.scene["obstacles"] if o["id"]==other["name"]))
                        sz=np.array(next(o["size_m"] for o in self.scene["obstacles"] if o["id"]==other["name"]))
                        half=np.array(self.scene["box"]["size_m"])/2
                        corner_z=box[2,3]-np.abs(box[2,:3])@half
                        if corner_z>=pos[2]+sz[2]/2-self.scene["contact_tolerance_m"]:
                            allowance="payload_support_top"
                ga,gb=a["gid"],b["gid"]
                # Safe bounding-sphere rejection, exact convex distance for remaining pairs.
                r=self.model.geom_rbound[ga]+self.model.geom_rbound[gb]
                delta=np.linalg.norm(self.data.geom_xpos[ga]-self.data.geom_xpos[gb])
                if delta-r>margin:continue
                dist=self.mj.mj_geomDistance(self.model,self.data,ga,gb,margin,None)
                threshold=-self.scene["contact_tolerance_m"] if allowance else margin
                if dist<threshold-1e-7:
                    self.last_failure=dict(reason="collision",phase=phase,pair=[a["name"],b["name"]],distance_m=float(dist),threshold_m=threshold,allowance=allowance)
                    return False
                if not allowance:self.min_clearance=min(self.min_clearance,float(dist))
        return True
    def attach(self,q):
        box=transform(self.scene["box"]["pose"])
        good,why=self.seal_fit(q,box)
        if not good:raise ValueError("Attachment rejected: "+why)
        self.payload_relative=np.linalg.inv(self.robot.fk(q))@box
        return self.payload_relative.copy()
    def release(self,q):
        box=self.box_pose(q,"place_contact")
        if not self.support_fit(box,"receiver"):raise ValueError("Release requires geometric receiver support")
        self.released_pose=box.copy();return box.copy()
    def render(self,q,phase,width=1280,height=720,azimuth=130,elevation=-25,distance=1.45,lookat=(.15,0,.85),overlay=False):
        self.set_state(q,phase)
        if not hasattr(self,"renderer") or self.renderer.width!=width or self.renderer.height!=height:
            if hasattr(self,"renderer"):self.renderer.close()
            self.renderer=self.mj.Renderer(self.model,height=height,width=width)
        camera=self.mj.MjvCamera();camera.azimuth=azimuth;camera.elevation=elevation;camera.distance=distance;camera.lookat[:]=lookat
        opt=self.mj.MjvOption();opt.geomgroup[3]=1 if overlay else 0
        self.renderer.update_scene(self.data,camera=camera,scene_option=opt)
        return self.renderer.render().copy()
    def close(self):
        if hasattr(self,"renderer"):self.renderer.close()
