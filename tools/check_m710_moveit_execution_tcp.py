"""Reaudit rotating LIN geometry in the final C2 executor reference, before Isaac."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.m710_replay_contract import verify_m710_replay_bundle
from unloading_sim.moveit2_backend import digest
from unloading_sim.moveit2_tcp import audit_linear_tcp


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',required=True,type=Path);p.add_argument('--requests',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);args=p.parse_args();started=perf_counter()
    result=dict(status='FAIL',isaac='NOT_RUN')
    try:
        bundle=json.loads(args.bundle.read_text());result['entry_verification']=verify_m710_replay_bundle(bundle,project_root=ROOT)
        metadata=bundle['metadata'];reference=metadata['joint_reference']
        assert reference['interpolation']=='C2_piecewise_quintic_rest_to_rest'
        joints=np.asarray(reference['positions_rad'])
        policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
        scene=build_verified_motion_input(policy,ROOT);built=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
        assert built.connector is not None
        c=built.connector
        requests={}
        for line in args.requests.read_text().splitlines():
            request=json.loads(line);base={k:v for k,v in request.items() if k not in {'pipeline_id','planner_id'}}
            requests[digest(base)]=request
        audits=[]
        for record in metadata['native_backend']['stages']:
            if record['planner_id']!='LIN':continue
            request=requests[record['request_fingerprint']]
            assert request['identity']==metadata['native_backend']['startup']['identity']
            np.testing.assert_array_equal(request['flange_from_task_tcp'],c.flange_from_virtual_task_tcp)
            first=np.asarray(record['points'][0]['q']);last=np.asarray(record['points'][-1]['q'])
            starts=np.flatnonzero(np.all(joints==first,axis=1));ends=np.flatnonzero(np.all(joints==last,axis=1))
            assert len(starts) and len(ends),'LIN_BOUNDARY_NOT_RETAINED_IN_EXECUTOR_REFERENCE'
            a=int(starts[-1]);b=int(next(i for i in ends if i>a))
            audit=audit_linear_tcp(joints[a:b+1],c.robot.fk,c.robot.fk(request['q_start']),request['goal_pose'],
                position_tolerance=c.ik['position_tolerance_m'],orientation_tolerance=c.ik['orientation_tolerance_rad'],
                edge_resolution_rad=c.budget.edge_resolution_rad)
            audits.append(dict(stage=record['stage'],reference_range=[a,b],audit=audit,
                native_path_nodes=len(record['points']),executor_path_nodes=b-a+1,
                request_fingerprint=record['request_fingerprint']))
        assert audits and all(x['audit']['passed'] for x in audits),'EXECUTOR_TASK_TCP_REJECTED'
        result.update(status='PASS',audits=audits,reference_semantics=reference['interpolation'],
            actual_executor_edges_checked=True,continuous_sweep_proof=False)
    except Exception as e:result['reason']=str(e)
    result.update(wall_s=perf_counter()-started,bundle_sha256=hashlib.sha256(args.bundle.read_bytes()).hexdigest(),
        requests_sha256=hashlib.sha256(args.requests.read_bytes()).hexdigest())
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8');print(result['status'],result.get('reason'),result['wall_s'])
    return int(result['status']!='PASS')

if __name__=='__main__':raise SystemExit(main())
