"""Real CUDA/native endpoint and negative-case audit, after CPU case collection."""
import argparse,copy,importlib.util,json
from pathlib import Path
import numpy as np
import torch
from unloading_sim.curobo_v2_backend import CuroboWorker
from unloading_sim.stage_backend import fingerprint
from unloading_sim.curobo_collision import install_pair_costs,PairCollisionCost


def main():
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--baseline-source');p.add_argument('--baseline-root');a=p.parse_args()
 root=Path(a.root);cpu=json.loads((root/'cpu_pairs.json').read_text());out={'endpoints':{},'negatives':[]}
 def save(): (root/'native_audit.json').write_text(json.dumps(out,indent=2,allow_nan=False))
 if a.baseline_source:
  spec=importlib.util.spec_from_file_location('unloading_sim.baseline26',a.baseline_source);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
  out['baseline']={}
  for name in ('final_business','final_direct_gpu'):
   folder=Path(a.baseline_root)/name;b=json.loads((folder/'bundle.json').read_text());w=mod.CuroboWorker(b,folder/'sphere_cache')
   q=[b['request']['q_start'],b['request']['q_goal']]
   f=w.planner.graph_planner.check_samples_feasibility(torch.tensor(q,device='cuda',dtype=torch.float32))
   out['baseline'][name]=dict(native_feasible=f.cpu().tolist(),sphere_diagnostics=w.collision_diagnostics(q));save();del w
 for name in ('business','direct'):
  b=json.loads((root/name/'bundle.json').read_text());w=CuroboWorker(b,root/'direct/sphere_cache')
  for manager in w.planner.trajopt_solver.additional_metrics_rollouts['interpolated_rollout']._cost_manager_list:
   cost=manager.get_cost('scene_collision')
   assert cost is None or isinstance(cost,PairCollisionCost)
  out['endpoints'][name]=w.native_endpoints([b['request']['q_start'],b['request']['q_goal']]);save()
  if name!='business':continue
  revoked=copy.deepcopy(b);revoked['pair_permissions']['base_mount']=[]
  revoked['request']['collision_policy']['wrist_tool_exempt_links']=[]
  revoked['request']['collision_policy_fingerprint']=fingerprint(revoked['request']['collision_policy'])
  revoked['bundle_fingerprint']=fingerprint({k:v for k,v in revoked.items() if k!='bundle_fingerprint'})
  rebuilt=CuroboWorker(revoked,root/'direct/sphere_cache')
  check=rebuilt.native_endpoints([revoked['request']['q_start']])
  assert rebuilt.prepared['geometry_cache']['cache_hit'] and check['feasible']==[False]
  assert rebuilt.prepared['runtime_policy']['wrist_tool_exempt_links']==[]
  assert any(x['pair']==['base_link','chassis'] for x in check['pairs'][0])
  out['revoked_policy_with_same_geometry_cache']=dict(cache=rebuilt.prepared['geometry_cache'],native=check)
  save();del rebuilt
  for case in cpu['negatives']:
   modified=copy.deepcopy(b)
   if case['added_obstacle']:modified['obstacles'].append(case['added_obstacle'])
   w.pair_collision=install_pair_costs(w.planner,modified,w.prepared)
   result=w.native_endpoints([case['q']]);assert result['feasible']==[False],case['name']
   out['negatives'].append(dict(name=case['name'],native=result,authority=case['authority']));save()
  w.pair_collision=install_pair_costs(w.planner,b,w.prepared)
  q=torch.tensor([b['request']['q_start'],b['request']['q_goal']],device='cuda',dtype=torch.float32,requires_grad=True)
  state=w.planner.compute_kinematics(w.JointState.from_position(q,w.d['joint_names']))
  g=w.pair_collision;c,r=g.moving_poses(state);s=state.robot_spheres.reshape(len(c),-1,4)
  full=sum(torch.relu(th+.005-gap).flatten(1).sum(1) for gap,th in g.terms(s,c,r))
  broad=g.cost(state,.005).reshape(-1)
  assert torch.allclose(full,broad,atol=3e-6,rtol=1e-4),(full,broad)
  broad.sum().backward();assert torch.isfinite(q.grad).all()
  out['broadphase']=dict(all_pairs=full.detach().cpu().tolist(),broadphase=broad.detach().cpu().tolist(),finite_gradient=True);save()
 print('NATIVE_AUDIT_PASSED',flush=True)
if __name__=='__main__':main()
