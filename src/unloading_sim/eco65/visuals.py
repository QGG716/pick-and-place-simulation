"""Render actual private geometry and the already validated quintic replay, locally."""
from pathlib import Path
import json,numpy as np
from PIL import Image,ImageDraw
from .model import *

def label(image,title,subtitle=""):
    im=Image.fromarray(image);d=ImageDraw.Draw(im);d.rectangle((0,0,im.width,54),fill=(20,29,38))
    d.text((16,10),title,fill="white");d.text((16,30),subtitle,fill=(193,207,219))
    return im
def current(w,lookat,distance=0.4,azimuth=120,elevation=-25,axes=None,overlay=False):
    camera=w.mj.MjvCamera();camera.lookat[:]=lookat;camera.distance=distance;camera.azimuth=azimuth;camera.elevation=elevation
    opt=w.mj.MjvOption();opt.geomgroup[3]=int(overlay)
    if not hasattr(w,"renderer"):w.renderer=w.mj.Renderer(w.model,height=720,width=1280)
    w.renderer.update_scene(w.data,camera=camera,scene_option=opt)
    if axes is not None:
        for i,color in enumerate([[1,0,0,1],[0,1,0,1],[.15,.45,1,1]]):
            idx=w.renderer.scene.ngeom;g=w.renderer.scene.geoms[idx]
            w.mj.mjv_initGeom(g,w.mj.mjtGeom.mjGEOM_ARROW,np.zeros(3),np.zeros(3),np.eye(3).flatten(),np.array(color))
            w.mj.mjv_connector(g,w.mj.mjtGeom.mjGEOM_ARROW,.0015,axes[:3,3],axes[:3,3]+.035*axes[:3,i])
            w.renderer.scene.ngeom+=1
    return w.renderer.render().copy()
def assembly_views(w,q):
    folder=OUTPUT/"assembly";folder.mkdir(parents=True,exist_ok=True)
    img=w.render(q,"initial")
    label(img,"ECO65-B + actual CAD suction tool","Gold: simulation-only adapter | geometric assembly, not measured installation").save(folder/"robot_tool.png")
    flange=w.robot.named_link_frames(q)["link_6"];tcp=w.robot.fk(q)
    label(current(w,flange[:3,3],.40,axes=flange),"Flange detail | 6xM4 / 49 mm official bolt circle","Gold adapter is an engineering assumption; tool holes do not directly match").save(folder/"flange_mount.png")
    label(current(w,tcp[:3,3],.40,azimuth=210,elevation=-5,axes=tcp),"TCP at CAD free-state seal plane","RGB = TCP XYZ; +Z approaches target | actual compression is unknown").save(folder/"tcp.png")
    for name in w.body_names:w.pose_body(name,transform([100,100,100]))
    display=transform(rotation=np.array([[1,0,0],[0,0,1],[0,-1,0]]))
    for part in w.tool["visual_parts"]:w.pose_body("tool_"+part["id"],display)
    w.mj.mj_forward(w.model,w.data)
    tcp_display=display@np.array(w.tool["T_tool_cad_tcp"])
    label(current(w,[0,0,.005],.40,azimuth=120,elevation=-30),"Actual STEP geometry | 36 solids / 12 suction components","180 x 105.8 x 132.8 mm CAD envelope | no mass/material inference").save(folder/"tool_cad.png")
    label(current(w,[0,0,.02],.37,azimuth=95,elevation=-65,axes=tcp_display),"Actual working face and common TCP","All 12 CAD seal rings must fit; independent pneumatic control is NOT assumed").save(folder/"working_face.png")
    label(current(w,[0,0,.01],.40,azimuth=120,elevation=-30,overlay=True),"Visual geometry + convex collision regions","Cyan: per-part hulls; only terminal skirts have pair/stage/direction contact rules").save(folder/"collision_overlay.png")
    w.set_state(q,"initial")
    # Named 180deg yaw candidate for the unresolved mounting orientation.
    alt=np.array(w.tool["T_flange_tool_cad"]);turn=transform(rotation=np.diag([-1,-1,1]));alt=turn@alt
    for part in w.tool["visual_parts"]:w.pose_body("tool_"+part["id"],flange@alt)
    w.mj.mj_forward(w.model,w.data)
    label(current(w,flange[:3,3],.40),"Alternative mounting yaw: 180 deg (comparison only)","Actual mounting is unverified; selected simulation uses yaw=0, no physical claim").save(folder/"mount_yaw180.png")
    w.set_state(q,"initial")
    return [str(p) for p in folder.glob("*.png")]
def replay_video(w,artifact,replay):
    import imageio.v2 as imageio
    from ..timing import sample_quintic_knots
    folder=OUTPUT/"replay";folder.mkdir(parents=True,exist_ok=True)
    total=sum(seg["duration_s"] for seg in artifact["segments"])
    ends=np.cumsum([s["duration_s"] for s in artifact["segments"]]);starts=np.r_[0,ends[:-1]]
    timestamps=np.arange(0,total+1e-8,.2)
    w.payload_relative=np.array(artifact["attachment_tcp_to_box"]);w.released_pose=np.array(artifact["released_box_pose"])
    writer=imageio.get_writer(str(folder/"single_box.mp4"),fps=5,codec="libx264",quality=8,macro_block_size=None)
    frames=[]
    selected_times=[0.,15.4,30.8,ends[1],ends[2],77.,ends[4],total-.2]
    selected_frames={min(len(timestamps)-1,max(0,int(round(t/.2)))) for t in selected_times}
    try:
        for index,t in enumerate(timestamps):
            k=min(int(np.searchsorted(ends,t,side="right")),len(ends)-1);s=artifact["segments"][k]
            q,_=sample_quintic_knots(s["t"],s["q"],np.array([t-starts[k]]));q=q[0]
            img=w.render(q,s["phase"],width=640,height=360,distance=2.1,lookat=(.12,0,1.04))
            im=label(img,f'GEOMETRIC_REPLAY_ONLY | {t:05.1f}s | {s["phase"]}',"ECO65-B / actual CAD / assumed adapter / dynamics NOT_EVALUATED")
            writer.append_data(np.asarray(im))
            if index in selected_frames:frames.append(im.copy())
            if index%50==0:print("VIDEO",index,"/",len(timestamps),flush=True)
        s=artifact["segments"][-1];q=np.array(s["q"][-1])
        final=label(w.render(q,"retreat",640,360,distance=2.1,lookat=(.12,0,1.04)),"GEOMETRIC_REPLAY_ONLY | RETREAT COMPLETE","Same carton supported on receiver; hardware never connected")
        writer.append_data(np.asarray(final));final.save(folder/"final.png")
    finally:writer.close()
    sheet=Image.new("RGB",(640*2,360*4),(20,29,38))
    for index,frame in enumerate(frames[:8]):sheet.paste(frame,((index%2)*640,(index//2)*360))
    sheet.save(folder/"contact_sheet.png")
    save_json(folder/"video_metadata.json",dict(fps=5,size=[640,360],playback_speed=1.,source_interpolator="sample_quintic_knots",
        source_fingerprint=artifact["trajectory_fingerprint"],frames=len(timestamps)+1,planned_duration_s=total,physics="NOT_EVALUATED"))
    return str(folder/"single_box.mp4")
