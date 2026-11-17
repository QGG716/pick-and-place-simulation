"""Actual native worker cancellation protocol; synthetic model, no devices."""
import argparse,json,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_planning_deadline as setup
from unloading_sim.workbench.single_carton.resident import CancellableWorkerClient
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.ui import ROOT


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    planner,model=setup.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    box=setup.synthetic_box();candidate=contact_candidates(box,planner.source['tool'],planner.site['topology'])[0]
    world=[dict(id=box['id'],pose=box['pose'],size=box['size_m'],category='carton')]
    context=planner.context(setup.Q_START,world,box,candidate,'approach')
    report=dict(scope='ACTUAL_NATIVE_PROTOCOL_SYNTHETIC_NO_DEVICES',hardware_execution='NOT_RUN',cases=[])
    try:
        for fmt in ('spaces','compact','reordered_padded','binary'):
            client=CancellableWorkerClient(['env','ROS_LOG_DIR='+str(args.output),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=args.output/(fmt+'.log'),name=fmt,binary=fmt=='binary')
            try:
                ready=client.request(planner.init)
                send=client._send
                def serialize(payload):
                    if payload.get('op')!='cancel' or fmt=='binary':return send(payload)
                    if fmt=='compact':line=json.dumps(payload,separators=(',',':'))
                    elif fmt=='reordered_padded':line=json.dumps(dict(target_request_id=payload['target_request_id'],padding=' '*400,op='cancel'))
                    else:line=json.dumps(payload)
                    client.process.stdin.write(line+'\n');client.process.stdin.flush()
                client._send=serialize
                client._send(dict(op='cancel',target_request_id='wrong-id'))
                request=dict(context,op='validate',probe_path=[setup.Q_START.tolist() for _ in range(20000)])
                started=time.monotonic()
                try:
                    client.request(request,cancelled=lambda:True)
                    raise AssertionError('cancellation returned usable result')
                except Exception as error:
                    response=getattr(error,'response',{})
                    row=dict(format=fmt,error=str(error),native_detail=response.get('detail'),native_status=response.get('status'),elapsed_s=time.monotonic()-started,cancellation=client.last_cancellation,binary_active=client.binary_active)
                    assert str(error)=='CANCELLED',row
                    assert client.last_cancellation['worker_finished'] and client.alive(),row
                    assert 'NATIVE_ABSOLUTE_DEADLINE_EXPIRED' in json.dumps(response),row
                for _ in range(2):client._send(dict(op='cancel',target_request_id=client.last_cancellation['request_id']))
                following=client.reset(planner.init['identity'])
                row.update(next_request=following['status'],worker_reused=client.alive());report['cases'].append(row)
            finally:client.close()
        report['status']='PASS'
    finally:
        planner.close();(args.output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
