"""Synthetic collision counterexamples on the official business fixture.

These are diagnostic perturbations, never additional business requests or
physical trials. They keep the original robot, tool and attachment.
"""
import argparse,json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from run_curobo_v2_transit import context,write
from unloading_sim.geometry import OBB
from unloading_sim.stage_export import export_request,released_world


def main():
 p=argparse.ArgumentParser();p.add_argument('--motion',required=True);p.add_argument('--state');p.add_argument('--output',required=True)
 a=p.parse_args();motion,scene,c,attachment,path,target=context(a.motion,a.state,'configs/validation/m710id70_handoff_continuation.yaml')
 world=[x for x in scene.all_obstacles if x.name!=target.name];q=path[0]
 result={'kind':'synthetic_counterexamples_on_business_model','original_scene_fingerprint':scene.snapshot['scene_fingerprint'],
         'unmodified_start':c._state_failure(q,world,attachment=attachment,stage='transit')}
 # Pick the payload corner farthest from the TCP, so it cannot be mistaken for
 # a robot/tool collision; verify this premise with the exact state validator.
 payload=attachment.box_at(q);tcp=attachment.physical_contact_pose(q)[:3,3]
 corners=sorted(payload.corners(),key=lambda x:-np.linalg.norm(x-tcp))
 probes=[]
 for i,corner in enumerate(corners):
  probe=OBB(corner,[.004,.004,.004],np.eye(3),f'corner_probe_{i}','diagnostic_obstacle')
  robot_tool=c._state_failure(q,[*world,probe],stage='transit')
  held=c._state_failure(q,[*world,probe],attachment=attachment,stage='transit')
  probes.append({'corner':corner.tolist(),'robot_tool_failure':robot_tool,'held_failure':held})
  if robot_tool is None and held is not None and held['reason']=='PAYLOAD_COLLISION':break
 result['payload_corner']=probes
 bad=q.copy();bad[0]=c.robot.joint_limits[0,1]+.1
 result['joint_out_of_range']=c._state_failure(bad,world,attachment=attachment,stage='transit')
 # A wrist-owned tool exception never suppresses an unrelated world collider.
 from dataclasses import asdict
 mesh=c.robot_state_validator.mesh_robot
 bounds=mesh.collision_world_axis_extrema(q)
 result['wrist_environment']={}
 for link in ('J5_link','J6_link'):
  center=(bounds[link]['lower_m']+bounds[link]['upper_m'])/2
  probe=OBB(center,[.05,.05,.05],np.eye(3),link+'_environment_probe','diagnostic_obstacle')
  # Isolate the named pair for attribution only; the full state check below
  # retains every link, tool collider, world obstacle and approved exception.
  ignored={(name,probe.name) for name in mesh.collision_link_names if name!=link}
  pair=mesh.collision_result(q,[probe],check_self=False,ignored_geometry_obstacle_pairs=ignored,
                              pair_policy=c.collision_policy,stage='transit')
  result['wrist_environment'][link]=dict(probe_center=center.tolist(),isolated_pair=asdict(pair),
     full_authority=c._state_failure(q,[*world,probe],attachment=attachment,stage='transit'),
     isolation_is_diagnostic_only=True)
 result['direct_edge']=c._path_failure([path[0],path[-1]],world,attachment=attachment,stage='transit')
 req,bundle=export_request(scene,c,attachment,path[0],path[-1],request_id='identity-check')
 released=released_world(bundle,attachment.box_at(path[-1]).world_from_local)
 result['identity']=dict(before=bundle['world_count_before'],attached=bundle['world_count_attached'],
                        released=len(released['obstacles']),payload_id=target.name)
 write(a.output,result)
 print(json.dumps(result,indent=2))

if __name__=='__main__': main()
