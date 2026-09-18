from pathlib import Path
import cv2,json,hashlib
p=Path('outputs/isaac_step31_once/replay.mp4');cap=cv2.VideoCapture(str(p));fps=cap.get(cv2.CAP_PROP_FPS);width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));declared=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));count=0
while True:
 ok,frame=cap.read()
 if not ok:break
 assert frame.shape[:2]==(360,640);count+=1
cap.release();r=dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),width=width,height=height,fps=fps,declared_frames=declared,decoded_frames=count,duration_s=count/fps,all_frames_decoded=count==declared,normal_time_profile_verified=width==640 and height==360 and fps==5.)
d=json.loads(Path('outputs/isaac_step31_once/result.json').read_text());r.update(replayed_simulation_seconds=d['replayed_simulation_seconds'],video_minus_physics_seconds=r['duration_s']-d['replayed_simulation_seconds'],metadata_physical_time_scale=d['replay_video_physical_time_scale']);assert abs(r['video_minus_physics_seconds'])<=.2 and r['metadata_physical_time_scale']==1
assert r['all_frames_decoded'] and r['normal_time_profile_verified'];Path('outputs/video_audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
