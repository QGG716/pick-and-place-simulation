from pathlib import Path
import sys,json,math,hashlib
import numpy as np
work=Path('/work-population-x300-cb6eb01');folder=Path('/population-x300-20261021/l04-departure-diagnosis')
sys.path[:0]=[str(work/'tools'),str(work/'src')]
from plan_m710_mount_comparison import make_context,FLAGS
from unloading_sim.ik import pose_error
_,c,_=make_context('x_minus_300_y_minus_100',layout_path=work/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_300_y_minus_100.yaml');robot=c.robot
assert type(robot).__name__=='PinocchioHppFclBackend'
exec(compile((folder/'arm-branch-kernel.py').read_text(),str(folder/'arm-branch-kernel.py'),'exec'))
data=json.loads((folder/'input.json').read_text());rows=[]
for req in data['requests']:
 def shifted(d):
  target=np.array(req['goal_pose']);target[0,3]+=d
  return dict(req,goal_pose=target.tolist())
 def branch(d):
  return next(b for b in audit(shifted(d))['branches'] if b.get('shoulder_sign')==1 and b.get('elbow_sign')==1)
 lo,hi=0.,.01
 assert not branch(lo)['arm_within_limits'] and branch(hi)['arm_within_limits']
 for _ in range(45):
  mid=(lo+hi)/2
  if branch(mid)['arm_within_limits']:hi=mid
  else:lo=mid
 trials=[]
 for distance in (0.,.01,.02,.05):
  request=shifted(distance);goal=np.asarray(request['goal_pose']);fit=np.array(req['q_start'])
  for iteration in range(100):
   error=pose_error(robot.fk(fit),goal)[0]
   if np.linalg.norm(error)<1e-11:break
   jac=robot.geometric_jacobian(fit);step=np.linalg.solve(jac,error);fit+=step*min(1.,.1/max(float(np.linalg.norm(step)),1e-12))
  error=pose_error(robot.fk(fit),goal)[0];assert np.linalg.norm(error)<1e-9
  b=branch(distance);assert np.max(np.abs(fit[:3]-np.asarray(b['q123_rad'])))<1e-7
  physical_fk=robot.fk(fit).copy();physical_fk[0,3]-=distance
  assert np.max(np.abs(physical_fk-np.asarray(req['goal_pose'])))<1e-9
  trials.append(dict(additional_base_minus_x_m=distance,new_world_base_x_m=float(robot.base_transform[0,3]-distance),q_deg=np.rad2deg(fit).tolist(),J3_lower_margin_deg=float(np.rad2deg(fit[2]-limits[2,0])),within_all_six_joint_limits=robot.within_limits(fit),position_error_m=float(np.linalg.norm(error[:3])),orientation_error_rad=float(np.linalg.norm(error[3:])),collision_checked=False,full_cycle_planned=False))
 rows.append(dict(stage_id=req['stage_id'],minimum_extra_retreat_m=(lo+hi)/2,threshold_scope='SAME_SHOULDER_ELBOW_BRANCH_ZERO_J3_LIMIT_MARGIN_ONLY',trials=trials))
r=dict(source_commit=data['source_commit'],source_trace_sha256=data['source_trace_sha256'],script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),method='Rigid base translation equivalence: same production robot with +X relative target; convert FK back to unchanged world target',current_base_xyz_m=robot.base_transform[:3,3].tolist(),current_layout_unchanged=True,solver_requests_added=0,rows=rows,**FLAGS)
(folder/'base-retreat-estimate.json').write_text(json.dumps(r,indent=2)+'\n')
for row in rows:print(json.dumps(row))
