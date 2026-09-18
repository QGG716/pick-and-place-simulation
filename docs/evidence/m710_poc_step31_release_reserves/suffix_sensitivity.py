"""Replay two aligned prior observed perturbations on the rebuilt receiving suffix."""
from pathlib import Path
import json,time,hashlib,numpy as np
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input
from unloading_sim.geometry import OBB
from unloading_sim.pair_clearance import obb_surface_distance
from unloading_sim.release_motion import ideal_reception_region,ReleasePolicy
root=Path.cwd(); source=Path('/root/autodl-tmp/m710-poc-step31-20260918/outputs/recovery_001/motion.json')
while not source.exists():
 if not Path('/proc/8569').exists():raise RuntimeError('CPU ended without motion')
 time.sleep(1)
m=json.loads(source.read_text());assert m['complete_trajectory_status']=='PASS'
s=m['selected_trajectory_segment'];path=np.array(s['path'])
policy=load_layout_motion_policy('configs/validation/m710id70_step31_recovery.yaml');scene=build_verified_motion_input(policy);robot=policy.layout_validation.layout.robot()
frames=policy.tool_frames.evidence();fv=np.array(frames['T_flange_virtual_task_tcp']);fp=np.array(frames['T_flange_nominal_compressed_contact'])
physical=lambda q:robot.fk(q)@np.linalg.inv(fv)@fp
A=np.array(s['contact']['physical_contact_from_box']);target=next(b for b in scene.cartons if b.name==s['target']);receivers=[b for b in scene.fixed_components if b.category=='conveyor']
oldpath=Path('/root/autodl-tmp/m710-poc-fifth-step3-v4-20260917/outputs/fifth_search_007_delivery/first_feasible/motion.json');old=json.loads(oldpath.read_text())['selected_trajectory_segment'];Aold=np.array(old['contact']['physical_contact_from_box'])
auditpath=Path('/root/autodl-tmp/m710-poc-step31-20260918/outputs/prior_tracking_point_audit.json');audit=json.loads(auditpath.read_text())['scopes']['receiver_approach_transit']['maxima']
scenarios={'nominal':(np.zeros(6),A,None)}
for label,key in [('observed_combined_max','command_to_actual_corner_displacement_m'),('observed_attachment_difference_max','measured_nominal_to_actual_corner_displacement_m')]:
 f=audit[key];effective=np.linalg.inv(physical(np.array(f['q_rad'])))@np.array(f['actual_box_pose']);deltaA=np.linalg.inv(Aold)@effective
 scenarios[label]=(np.array(f['q_rad'])-np.array(f['q_command_rad']),A@deltaA,dict(frame=f,box_local_attachment_delta=deltaA.tolist()))
def box(q,a):
 p=physical(q)@a;return OBB(p[:3,3],target.half_extents,p[:3,:3],target.name,target.category)
top=max(float(b.corners()[:,2].max()) for b in receivers)
start,last=s['stage_ranges']['transit'];place_last=s['stage_ranges']['place'][1]
# Include the rebuilt suffix and its immediately preceding receiver approach.
indices=[i for i in range(start,last+1) if box(path[i],A).corners()[:,2].min()-top<.15]
first=max(start,min(indices)-1)
result=dict(scope='TWO_ALIGNED_OBSERVED_PERTURBATIONS_NOT_GLOBAL_ROBUSTNESS_PROOF',motion_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_audit_sha256=hashlib.sha256(auditpath.read_bytes()).hexdigest(),path_indices=[first,place_last],stage_ranges=s['stage_ranges'],receiver_geometry=[dict(name=b.name,pose=b.world_from_local.tolist(),half_extents_m=b.half_extents.tolist()) for b in receivers],sampling='SAME_DENSE_EDGE_FORMULA_AS_PRODUCTION_PATH_FAILURE',scenarios={})
for label,(dq,attach,provenance) in scenarios.items():
 minima={b.name:dict(distance_m=float('inf')) for b in receivers};samples=0;max_corner=0.
 for edge in range(first,place_last):
  q0,q1=path[edge:edge+2];n=max(1,int(np.ceil(np.max(np.abs(q1-q0))/.04)),int(np.ceil(4*np.sum(np.abs(q1-q0))/.0025)))
  for fraction in np.linspace(0,1,2*n+1):
   q=q0+fraction*(q1-q0);payload=box(q+dq,attach);samples+=1
   max_corner=max(max_corner,float(np.linalg.norm(payload.corners()-box(q,A).corners(),axis=1).max()))
   for receiver in receivers:
    distance=obb_surface_distance(payload,receiver)
    if distance<minima[receiver.name]['distance_m']:minima[receiver.name]=dict(distance_m=distance,edge=edge,fraction=float(fraction),stage='transit' if edge<last else 'place',q_rad=(q+dq).tolist(),pose=payload.world_from_local.tolist())
 release=box(path[place_last]+dq,attach);supports=[b for b in receivers if b.name in s['place']['support_names']]
 region=ideal_reception_region(release,supports,policy=ReleasePolicy(**s['place']['release_prediction']['policy']))
 result['scenarios'][label]=dict(samples=samples,provenance=provenance,q_delta_rad=dq.tolist(),minimum_by_receiver=minima,maximum_corresponding_corner_displacement_m=max_corner,release_region=region,runtime_5mm_pass=all(x['distance_m']>=.005 for x in minima.values()))
 print(label,samples,{k:v['distance_m'] for k,v in minima.items()},region['accepted'],flush=True)
(root/'outputs/suffix_sensitivity.json').write_text(json.dumps(result,indent=2))
