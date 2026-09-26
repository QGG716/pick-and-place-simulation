"""Collect exact CPU pair evidence and labelled negative cases, without Isaac."""
import argparse,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from run_curobo_v2_transit import context,write
from unloading_sim.stage_export import box_record
from unloading_sim.geometry import OBB
from unloading_sim.pair_clearance import obb_pair_evidence


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
 _,scene,c,attachment,path,target=context('fixtures/curobo_v2/source_motion.json','fixtures/curobo_v2/actual_remaining_state.json','configs/validation/m710id70_handoff_continuation.yaml')
 world=[x for x in scene.all_obstacles if x.name!=target.name];by_name={x.name:x for x in world}
 base_inv=np.linalg.inv(c.robot.base_transform);v=c.robot_state_validator
 endpoints=[]
 states=[('business_start',path[0]),('business_goal',path[-1]),('direct_start',path[-1]),('direct_goal',path[-1]+np.array([0,0,.001,0,0,0]))]
 for name,q in states:
  payload=attachment.box_at(q);rigid=v.tool_transform_robot.tool_collision_obbs(q)
  pairs=[]
  for first,second in [(rigid[0],payload),(payload,by_name['carton_l06_c04']),(payload,by_name['conveyor_longitudinal'])]:
   e=obb_pair_evidence(first,second,c.collision_policy,stage='transit',proxy=first.name.startswith('tool_'))
   e['ownership']=['flange-owned rigid tool' if first.name.startswith('tool_') else 'held carton', 'held carton' if second.name==target.name else 'world obstacle']
   e['permission']=None;pairs.append(e)
  raw=v.mesh_robot.collision_result(q,[by_name['chassis']],check_self=False,
      ignored_geometry_obstacle_pairs={(x,'chassis') for x in v.mesh_robot.collision_link_names if x!='base_link'},pair_policy=c.collision_policy,stage='transit')
  pairs.append(dict(pair=['base_link','chassis'],ownership=['fixed robot base','world installation'],permission='existing exact fixed installation pair',required_pair_clearance_m=0,raw_geometry=raw.evidence,accepted=True))
  endpoints.append(dict(name=name,q=q.tolist(),authority=c._state_failure(q,world,attachment=attachment,stage='transit'),pairs=pairs))
 negatives=[];q=path[0];payload=attachment.box_at(q)
 def add(name,probe,q=q):
  env=world if probe is None else [*world,probe]
  failure=c._state_failure(q,env,attachment=attachment,stage='transit')
  negatives.append(dict(name=name,q=q.tolist(),added_obstacle=None if probe is None else box_record(probe,base_inv),authority=failure))
 tcp=attachment.physical_contact_pose(q)[:3,3]
 for corner in sorted(payload.corners(),key=lambda x:-np.linalg.norm(x-tcp)):
  probe=OBB(corner,[.004]*3,np.eye(3),'payload_corner_probe','diagnostic')
  if c._state_failure(q,[*world,probe],stage='transit') is None:
   add('payload_corner_robot_tool_clear',probe);break
 bounds=v.mesh_robot.collision_world_axis_extrema(q)
 for name in ('J5_link','J6_link'):
  add(name+'_environment',OBB((bounds[name]['lower_m']+bounds[name]['upper_m'])/2,[.05]*3,np.eye(3),name+'_probe','diagnostic'))
 rigid=v.tool_transform_robot.tool_collision_obbs(q)[0]
 add('rigid_tool_despite_cup_permissions',OBB(rigid.center,[.004]*3,np.eye(3),'rigid_probe','diagnostic'))
 edge=c._path_failure([path[0],path[-1]],world,attachment=attachment,stage='transit')
 middle=path[0]+edge['fraction']*(path[-1]-path[0]);add('legal_endpoints_colliding_middle',None,middle)
 assert all(x['authority'] is None for x in endpoints)
 assert all(x['authority'] is not None for x in negatives)
 write(a.output,dict(endpoints=endpoints,negatives=negatives,edge_failure=edge,nominal_cup_compression_m=v.nominal_cup_compression_m,scope='synthetic negatives preserve the business input; no physical trial'))
if __name__=='__main__':main()
