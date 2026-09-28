import json,sys,hashlib
from pathlib import Path
from time import perf_counter
import numpy as np
sys.path.insert(0,str(Path.cwd()))
from tools.probe_m710_free_approach import rebuild,write
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.stage_motion_policy import interrupts_generation,failure_status
from unloading_sim.layout_single_carton import motion_implementation_identity
base=Path('/root/autodl-tmp/m710-local-transit-budget-20260928/evidence')
f=json.load(open(base/'baseline.frozen.json'))
case=json.load(open('/root/autodl-tmp/m710-free-approach-20260928/evidence/frozen/frozen_case.json'))
c,s,target=rebuild(case)
c._contact_selection(np.asarray(f['contact_q']),target,case['face'],s.policy.data['suction'])
a=PhysicalContactAttachment(c.robot,RigidAttachment.capture(c.physical_from_virtual(c.robot.fk(np.asarray(f['contact_q']))),target),c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
np.testing.assert_array_equal(a.rigid.tcp_from_box,f['attachment']['tcp_from_box'])
world=[b for b in s.all_obstacles if b.name!=target.name]
prior=json.load(open('/root/autodl-tmp/m710-free-approach-20260928/evidence/single_carton_0bfd708_delivery/motion.json'))
t=prior['tasks'][0]['attempts'][0]['trajectory_search']['attempts'][0]['trace']['stages']['transit']
goal=np.asarray(t['attempts'][0]['q_rad']);v=c._motion_validator(world,attachment=a,stage='transit')
assert v.context.context_id==t['validation_context']
c._local_transit_remaining=240
before=c._statistics.copy();start=perf_counter()
if hasattr(c,'_bounded_local_transit'):
 path,failure,evidence=c._bounded_local_transit(np.asarray(f['start_q']),c.robot.fk(goal),world,a,seed=f['seed']+55)
else:
 pool=c._local_transit_remaining;allocation=80;c._local_transit_remaining=allocation
 try:path,failure,evidence=c._local_cartesian_transit(np.asarray(f['start_q']),c.robot.fk(goal),world,a,seed=f['seed']+55)
 finally:
  consumed=allocation-c._local_transit_remaining;c._local_transit_remaining=pool-consumed
 evidence.update(allocated=allocation,consumed=consumed,candidate_remaining=allocation-consumed,shared_remaining=c._local_transit_remaining,request_budget_remaining=c._validation_request().available())
write(base/(sys.argv[1]+'.json'),dict(source=motion_implementation_identity(Path.cwd()),context_id=v.context.context_id,input_sha256=hashlib.sha256((base/'baseline.frozen.json').read_bytes()).hexdigest(),failure=failure,evidence=evidence,status=failure_status(failure),interrupts_request=interrupts_generation(failure),elapsed_seconds=perf_counter()-start,statistics_delta={k:c._statistics[k]-before[k] for k in before},shared_after=c._local_transit_remaining))
