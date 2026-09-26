"""Re-evaluate saved real V2 B-spline knots on CUDA; audit fitted geometry."""
import argparse,json
from dataclasses import asdict
from pathlib import Path
from time import perf_counter
import numpy as np
import torch,trimesh,yaml
from curobo._src.state.state_joint import JointState
from curobo._src.types.device_cfg import DeviceCfg
from curobo._src.types.control_space import ControlSpace
from curobo._src.util.trajectory import get_batch_interpolated_trajectory,TrajInterpolationType
from curobo._src.geom.sphere_fit.metrics import compute_sphere_fit_metrics
from curobo.content import get_task_configs_path
from unloading_sim.geometry import rotation_matrix_from_rpy


def main():
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--baseline',required=True);a=p.parse_args()
 root=Path(a.root);old=json.loads(Path(a.baseline).read_text())['preparation'];new=json.loads((root/'direct/gpu_model.json').read_text())['preparation']
 out=dict(output=[],geometry={},scope='finite numerical mesh coverage, not full geometry certification')
 def save():(root/'output_geometry_audit.json').write_text(json.dumps(out,indent=2,allow_nan=False))
 cfg=yaml.safe_load((Path(get_task_configs_path())/'trajopt/transition_bspline_trajopt.yml').read_text())
 space=ControlSpace[cfg['transition_model_cfg']['control_space']];device=DeviceCfg()
 for result in json.loads((root/'direct/results.json').read_text()):
  assert result['authority_accepted'];candidate=result['attempts'][-1];raw=candidate['native_output'];t=candidate['trajectory'];d=result['request']
  q=np.array(t['q']);dt=raw['interpolation_dt_s'];n=len(q)
  def tensor(x):return torch.tensor(x,device='cuda',dtype=torch.float32)
  js=JointState.zeros([1,81,6],device);js.knot=tensor(raw['control_points']).reshape(1,-1,6);js.knot_dt=tensor(raw['knot_dt']).reshape(1);js.control_space=space
  start=JointState.zeros([1,6],device);start.position[:]=tensor([d['q_start']])
  goal=JointState.zeros([1,6],device);goal.position[:]=tensor([d['q_goal']])
  idx=torch.zeros(1,device='cuda',dtype=torch.int32);implicit=torch.ones(1,device='cuda',dtype=torch.uint8)
  def evaluate(step):return get_batch_interpolated_trajectory(js,tensor([step]),kind=TrajInterpolationType.BSPLINE_KNOTS_CUDA,out_traj_state=JointState.zeros([1,5000,6],device),current_state=start,goal_state=goal,start_idx=idx,goal_idx=idx,use_implicit_goal_state=implicit)
  evaluated,last=evaluate(dt);dense,dense_last=evaluate(dt/2)
  errors={key:float(np.max(np.abs(getattr(evaluated,attr).cpu().numpy().reshape(-1,6)[:n]-np.asarray(t[key])))) for key,attr in [('q','position'),('dq','velocity'),('ddq','acceleration')]}
  assert int(last[0])==n and max(errors.values())<1e-6,(last,n,errors)
  dense_q=dense.position.cpu().numpy().reshape(-1,6)
  mid_error=float(np.max(np.abs(dense_q[1:2*n-2:2]-(q[:-1]+q[1:])/2)))
  out['output'].append(dict(run=result['run_kind'],reevaluated_bspline_max_error=errors,control_space=space.name,
      delivered_interpolation='linear_joint_samples',midpoint_bspline_vs_delivered_linear_max_rad=mid_error,
      samples=n,padding=raw['padding_count'],time_start=t['time_s'][0],time_end=t['time_s'][-1],dt=dt,
      finite_dq_ddq=True,authority=result['authority_accepted'],derivative_semantics=t['derivative_semantics']))
  save()
 for link,quality in new['quality'].items():
  item=quality['source'];mesh=trimesh.load(item['path'],force='mesh');mesh.apply_scale(item['scale'])
  m=np.eye(4);m[:3,:3]=rotation_matrix_from_rpy(*item['rpy']);m[:3,3]=item['xyz'];mesh.apply_transform(m)
  entry={}
  for label,record in [('before_global_compensation',old),('after_local_compensation',new)]:
   spheres=record['robot_cfg']['kinematics']['collision_spheres'][link]
   centers=np.array([x['center'] for x in spheres]);radii=np.array([x['radius'] for x in spheres])
   np.random.seed(716);torch.manual_seed(716);start_time=perf_counter()
   metrics=compute_sphere_fit_metrics(mesh,centers,radii)
   entry[label]=dict(metrics=asdict(metrics),audit_s=perf_counter()-start_time)
  out['geometry'][link]=entry;save()
 print('OUTPUT_GEOMETRY_AUDIT_PASSED',flush=True)
if __name__=='__main__':main()
