import json,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path.cwd()))
from tools.probe_m710_free_approach import rebuild,write
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
p=pathlib.Path('../evidence');f=json.load(open(p/'allocated_connection.frozen.json'));case=json.load(open('/root/autodl-tmp/m710-free-approach-20260928/evidence/frozen/frozen_case.json'))
c,s,target=rebuild(case);a=PhysicalContactAttachment(c.robot,RigidAttachment(np.asarray(f['attachment']['tcp_from_box']),np.asarray(f['attachment']['half_extents']),target.name),c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
events=[json.loads(x) for x in (p/'allocated_connection.progress.jsonl').read_text().splitlines()]
e=next(x for x in events if x.get('event')=='stage_exit' and x.get('entry')=='_cartesian' and x.get('stage')=='transit' and x.get('failure'))
failure=e['failure'];q=np.asarray(failure['q_rad']);start=np.asarray(f['start_q'])
work=c._local_transit_work(start,np.asarray(f['preplace_virtual']))
outward=-a.physical_contact_pose(start)[:3,2]
write(p/'first_geometry_failure.json',dict(scope='FK_OF_RECORDED_REJECTED_PLANNING_SAMPLE_NOT_EXECUTED_STATE',failure=failure,cartesian_sample=e['statistics']['cartesian_samples'],virtual_pose=c.robot.fk(q),physical_contact_pose=a.physical_contact_pose(q),payload_pose=a.box_at(q).world_from_local,work_estimate=work,existing_outward_step_m=c.budget.local_transit_outward_step_m,existing_outward_direction=outward,geometry_or_permission_changes=False))
print(work)
