"""Endpoint-only equivalence on saved RGB-D, never plans or registers a permit."""
import json,time,sys,hashlib,subprocess
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner,TargetBlocked
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.site_scene import build_scene
from unloading_sim.workbench.single_carton.self_filter import RobotSelfFilter
from unloading_sim.workbench.single_carton.native_flow import NativeFlow
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.coordinates import Installation,CoordinateModel
from unloading_sim.workbench.ui import ROOT,URDF,URDF_SHA,load_mapping
from unloading_sim.moveit2_backend import ResidentMoveItClient
record=Path(sys.argv[1]).resolve();out=Path(sys.argv[2]).resolve();out.mkdir(parents=True,exist_ok=False)
names=('observation.json','observation.npz','segmentation.npz','geometry.json','site-scene.json')
def hashes():return {n:hashlib.sha256((record/n).read_bytes()).hexdigest() for n in names if (record/n).exists()}
before=hashes();obs=json.loads((record/'observation.json').read_text());data=np.load(record/'observation.npz');seg=np.load(record/'segmentation.npz')
old=json.loads((record/'geometry.json').read_text());masks=tuple(seg[f'mask_{i}'] for i in range(len(seg.files)))
frame=SimpleNamespace(observation_id=obs['observation_id'],rgb=data['rgb'],depth=data['depth'],intrinsics=obs['intrinsics'],depth_scale_m=obs['depth_scale_m'])
config=load(ROOT/'configs/workbench/single_carton.example.json')
installation=Installation.load(config['execution']['installation']);model=CoordinateModel(URDF,URDF_SHA,installation.world_from_base,installation.tool)
p=SinglePlanner(config,load_mapping(config['execution']['mapping']),Authority('eco65_single_carton_segment_v1'),model);p.build()
boxes,rejected=recover_boxes(frame,obs['T_world_camera'],old.get('detections') or [dict(label='saved-mask')]*len(masks),masks,config.get('geometry'))
geometry=config.get('geometry') or {};depth=float(np.linalg.norm(geometry['prior_size_m']))+geometry.get('dimension_tolerance_m',0) if geometry.get('prior_size_m') else None
scene=build_scene(SimpleNamespace(frame=frame,world_camera=obs['T_world_camera'],q_capture=obs['q_capture']),SimpleNamespace(observation_id=frame.observation_id,masks=masks),boxes,config['site'],RobotSelfFilter(p.init['parameters']['robot_description'],model),neighbor_depth_m=depth)
world=scene['world']+[dict(id=b['id'],size=b['size_m'],pose=b['pose'],category='carton') for b in boxes]
q=np.asarray(obs['q_start']);p.wrist_reference=q.tolist()
report=dict(scope='SAVED_REAL_RGBD_ENDPOINT_SCREENING_ONLY_NO_PLAN_NO_PERMIT',physical='NOT_RUN',sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),source=str(record),input_sha256=before,boxes=len(boxes),scene_objects=len(world),targets=[])
for box in boxes:
 candidates=contact_candidates(box,p.source['tool'],p.site['topology']);results={}
 for mode in ('PYTHON','NATIVE'):
  p.site['planning_module']=mode;folder=out/mode;folder.mkdir(exist_ok=True)
  client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(folder),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=folder/'worker.log',binary=mode=='NATIVE')
  try:
   p.client=client;p.worker_ready(client.request(p.init),boxes)
   replay=NativeFlow(p).screen(q,world,box,candidates,list(range(len(candidates))),folder,lambda:False) if mode=='NATIVE' else None
   if replay:p.client=replay
   rows=[];solutions=[]
   for i,candidate in enumerate(candidates):
    sub=folder/str(i);sub.mkdir(exist_ok=True)
    try:rows.append(dict(index=i,status='SCREENED',result=p.screen_candidate(q,world,box,candidate,sub,lambda:False,solutions)))
    except TargetBlocked as e:rows.append(dict(index=i,status='TARGET_BLOCKED',reason=str(e)));break
    except ValueError as e:rows.append(dict(index=i,status='REJECTED',reason=str(e)))
   results[mode]=rows
   if replay:
    report['replay']=dict(recorded=replay.recorded,served=replay.served,unused=replay.unused())
    assert replay.unused()==0
  finally:client.close();p.client=None
 from collections import Counter
 report['targets'].append(dict(id=box['id'],candidates=len(candidates),identical=results['PYTHON']==results['NATIVE'],
   outcomes={mode:dict(Counter(r['status'] if r['status']!='SCREENED' else r['result']['screen']['conclusion'] for r in rows)) for mode,rows in results.items()},results=results))
report.update(all_identical=all(t['identical'] for t in report['targets']),source_record_unchanged=before==hashes(),permits_issued=len(p.authority._plans))
(out/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False));assert report['source_record_unchanged'] and report['permits_issued']==0
print(json.dumps({k:v for k,v in report.items() if k not in ('targets','input_sha256')},indent=2))
