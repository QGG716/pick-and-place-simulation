import json,pathlib,sys,hashlib
import numpy as np
sys.path.insert(0,str(pathlib.Path.cwd()))
from tools.probe_m710_free_approach import rebuild,write
from unloading_sim.ik import pose_error
p=pathlib.Path('../evidence');d=json.load(open(p/'allocated_connection.json'));assert d['status']=='VALID' and d['evidence']['validation_completed']
c,s,target=rebuild(json.load(open('/root/autodl-tmp/m710-free-approach-20260928/evidence/frozen/frozen_case.json')))
q=np.asarray(d['path'][-1]);fk=c.robot.fk(q);_,position,angle=pose_error(fk,np.asarray(d['preplace_virtual']))
assert position<=c.ik['position_tolerance_m'] and angle<=c.ik['orientation_tolerance_rad']
m=json.load(open(p/'source_192f139.json'));unchanged=all(hashlib.sha256(pathlib.Path(n).read_bytes()).hexdigest()==v['runtime_sha256'] for n,v in m['files'].items());assert unchanged
write(p/'endpoint_compatibility.json',dict(status='PASS',position_error_m=position,orientation_error_rad=angle,position_tolerance_m=c.ik['position_tolerance_m'],orientation_tolerance_rad=c.ik['orientation_tolerance_rad'],actual_virtual_fk=fk,preplace_virtual=d['preplace_virtual'],full_connection_context=d['evidence']['validation_context'],guarantee=d['evidence']['guarantee'],validation_completed=True,rrt_called=d['evidence']['rrt_called'],runtime_files_unchanged=unchanged,scope='VALIDATED_CONNECTION_ENDPOINT_MATCHES_ORIGINAL_PROCESS_ENTRY_NOT_COMPLETE_PLACEMENT'))
print(position,angle)
