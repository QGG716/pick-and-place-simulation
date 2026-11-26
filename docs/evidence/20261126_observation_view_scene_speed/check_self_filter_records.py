"""Saved field records: build_scene with the previous and the new RobotSelfFilter.contains; identical JSON?"""
import json,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.perception import recover_boxes
from unloading_sim.workbench.single_carton.size_policy import size_policy,neighbor_depth_bound,NO_PRIOR_POLICIES
from unloading_sim.workbench.single_carton import site_scene
from unloading_sim.workbench.single_carton.self_filter import RobotSelfFilter
from unloading_sim.workbench.single_carton.geometry import transform
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping,URDF,URDF_SHA,ROOT
from unloading_sim.workbench.coordinates import Installation,CoordinateModel
def old_contains(self,points,q,group_size=1):
    points=np.asarray(points);inside=np.zeros(len(points)//group_size,bool)
    frames={name:self.model.world_from_base@T for name,T in self.model.frames(q).items()};frames['world']=self.model.world_from_base
    def frame(name):
        if name not in frames:
            parent,T=self.fixed[name];frames[name]=frame(parent)@T
        return frames[name]
    for name,T,equations,bounds in self.solids:
        world=frame(name)@T
        center=world[:3,:3]@bounds.mean(0)+world[:3,3];half=abs(world[:3,:3])@((bounds[1]-bounds[0])/2)
        candidates=np.flatnonzero(~np.repeat(inside,group_size) & np.all(abs(points-center)<=half,axis=1))
        if len(candidates):
            local=transform(np.linalg.inv(world),points[candidates]);same=np.zeros(len(points),bool)
            same[candidates]=np.all(local@equations[:,:3].T+equations[:,3]<=1e-9,axis=1)
            inside|=same.reshape(-1,group_size).all(1)
    return inside
cfg=load(Path(sys.argv[1]));limit=int(sys.argv[2]);out=Path(sys.argv[3])
inst=Installation.load(cfg['execution']['installation']);model=CoordinateModel(URDF,URDF_SHA,inst.world_from_base,inst.tool)
pl=SinglePlanner(cfg,load_mapping(cfg['execution']['mapping']),Authority('eco65_single_carton_segment_v1'),model);pl.build()
sf=RobotSelfFilter(pl.init['parameters']['robot_description'],model);g=cfg.get('geometry')
root=ROOT/'outputs/single_carton'
recs=[d for d in sorted(root.iterdir(),key=lambda p:p.stat().st_mtime,reverse=True) if (d/'segmentation.npz').is_file() and (d/'geometry.json').is_file()][:limit]
rows=[]
for d in recs:
    obs=json.loads((d/'observation.json').read_text())
    with np.load(d/'observation.npz') as z:rgb,depth=z['rgb'],z['depth']
    with np.load(d/'segmentation.npz') as z:masks=tuple(z[f'mask_{i}'].astype(bool) for i in range(len(z.files)))
    dets=json.loads((d/'geometry.json').read_text())['detections']
    frame=SimpleNamespace(observation_id=obs['observation_id'],rgb=rgb,depth=depth,intrinsics=obs['intrinsics'],depth_scale_m=obs['depth_scale_m'],stamp=SimpleNamespace(source='X'))
    try:boxes,rej=recover_boxes(frame,obs['T_world_camera'],dets,masks,g,cfg['site'].get('workspace_bounds_m'))
    except Exception as e:rows.append(dict(record=d.name,error=str(e)[:200]));continue
    def build():
        return site_scene.build_scene(SimpleNamespace(frame=frame,world_camera=obs['T_world_camera'],q_capture=obs['q_capture']),SimpleNamespace(observation_id=obs['observation_id'],masks=masks),boxes,cfg['site'],sf,neighbor_depth_m=neighbor_depth_bound(g),unresolved_cartons=rej if size_policy(g) in NO_PRIOR_POLICIES else (),geometry=g)
    res={}
    for label,fn in (('previous',old_contains),('new',RobotSelfFilter.contains)):
        RobotSelfFilter.contains=fn
        t=time.monotonic()
        try:s=json.dumps(build(),sort_keys=True,default=str);err=None
        except Exception as e:s=None;err=f'{type(e).__name__}: {e}'[:200]
        res[label]=(time.monotonic()-t,s,err)
    RobotSelfFilter.contains=res and RobotSelfFilter.contains
    row=dict(record=d.name,previous_s=round(res['previous'][0],3),new_s=round(res['new'][0],3),
             identical=res['previous'][1]==res['new'][1] and res['previous'][2]==res['new'][2],error=res['new'][2],
             robot_returns_removed=(json.loads(res['new'][1]).get('self_returns_removed') if res['new'][1] else None))
    rows.append(row);print(json.dumps(row),flush=True)
out.write_text(json.dumps(dict(config=str(sys.argv[1]),rows=rows),indent=1))
