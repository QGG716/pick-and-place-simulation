"""Re-run one frozen loaded request. Diagnostic scope never becomes a task plan."""
import argparse
from copy import deepcopy
import gzip
import hashlib
import json
from pathlib import Path
from time import perf_counter
import uuid
from unloading_sim.eco65.model import load_json, save_json
from unloading_sim.eco65.native_io import verify_snapshot
from unloading_sim.moveit2_backend import ResidentMoveItClient, digest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--source-request',type=Path,required=True)
    p.add_argument('--worker-command',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--solver-seconds',type=float,default=120.)
    p.add_argument('--rpc-seconds',type=float,default=300.)
    p.add_argument('--trace-event-limit',type=int,default=20000)
    p.add_argument('--trace-sample-limit',type=int,default=40000)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    snapshot=verify_snapshot(load_json(a.snapshot));source=load_json(a.source_request)
    req=deepcopy(source.get('request',source));init=deepcopy(snapshot['native_model']['init'])
    if req['identity']!=init['identity'] or req['stage']!='transit' or not req['attachment']:
        raise ValueError('LOADED_TRANSIT_IDENTITY_REQUIRED')
    run_id='diagnostic-'+uuid.uuid4().hex
    req.update(op='diagnostic_connection',task_id=run_id,stage_id='transit-fixed-01',parent_stage_id='',
               request_budget_s=a.rpc_seconds-10.,return_reserve_s=5.,
               allowed_planning_time_s=a.solver_seconds,output_check_timeout_s=max(1.,a.rpc_seconds-a.solver_seconds-15.),
               search_trace=True,trace_event_limit=a.trace_event_limit,trace_sample_limit=a.trace_sample_limit)
    req.pop('request_id',None)
    req['process_policy']['selection_binding']['task_id']=run_id
    init.update(evaluation_profile='chain_integration_v1',search_trace_enabled=True,
                directed_j1_enabled=req.get('j1_direction',0)!=0)
    save_json(a.output/'request.json',req)
    report=dict(run_id=run_id,status='STARTED',scope='DIAGNOSTIC_LOADED_START_NOT_A_COMPLETE_PLAN',
                source_request=str(a.source_request),source_request_sha256=hashlib.sha256(a.source_request.read_bytes()).hexdigest(),
                original_request_id=source.get('request',source).get('request_id'),snapshot_fingerprint=snapshot['fingerprint'],
                new_solve=True,source_tree=source.get('result',{}).get('search_trace_ref',{}).get('status','NOT_RECORDED'),dynamics='NOT_EVALUATED')
    start=perf_counter()
    client=ResidentMoveItClient(a.worker_command)
    try:
        try:
            ready=client.request(init,timeout=a.rpc_seconds)
            save_json(a.output/'initialization.json',ready)
            begin=perf_counter();response=client.request(req,timeout=a.rpc_seconds)
            report['request_wall_s']=perf_counter()-begin
            trace=response.pop('search_trace',None)
            if trace:
                trace.update(resident_seed=ready['seed'],request_seed=req.get('seed'),
                    rng_semantics='OMPL_RNG_SEEDED_ONCE_AT_RESIDENT_INIT_REQUEST_SEED_IS_A_REQUEST_LABEL',
                    resident_seed_source='SAME_RESIDENT_READY_RESPONSE')
                with gzip.open(a.output/'search_trace.json.gz','wt',encoding='utf-8') as f:json.dump(trace,f,separators=(',',':'))
                response['search_trace_ref']=dict(path='search_trace.json.gz',sha256=hashlib.sha256((a.output/'search_trace.json.gz').read_bytes()).hexdigest(),status=trace['status'])
            save_json(a.output/'response.json',response)
            report.update(status=response['status'],solver_message=response.get('solver_message'),
                          trace_status=trace['status'] if trace else 'NOT_STARTED_NO_TREE')
        except Exception as error:
            report.update(status='INTERRUPTED_NOT_CAPTURED',error=str(error),trace_status='INCOMPLETE')
        finally:
            report['total_wall_s']=perf_counter()-start;save_json(a.output/'run.json',report)
    finally: client.close()
    print(json.dumps(report))


if __name__=='__main__':main()
