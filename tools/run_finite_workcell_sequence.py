"""Process a finite list serially and deliver to one persistent read-only ROS graph."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.finite_sequence import (atomic_json,read_json,manifest_groups,
    verify_capture,verify_result,verify_sam_files,input_identity)
from unloading_contracts import canonical_fingerprint


class RosDelivery:
    """A single launch; completion handshake is acknowledged by received world content."""
    def __init__(self, output, domain):
        self.output,self.domain = output,domain
        self.process = self.log = None

    def start(self):
        self.log = (self.output/'ros-launch.log').open('w',encoding='utf-8')
        self.process = subprocess.Popen(['ros2','launch',
            str(ROOT/'ros2_ws/src/unloading_bringup/launch/finite_algorithm_replay.launch.py'),
            'progress:='+str(self.output/'progress.json'), 'receipts:='+str(self.output/'receipts')],
            env={**os.environ,'ROS_DOMAIN_ID':str(self.domain)},stdout=self.log,stderr=subprocess.STDOUT,
            start_new_session=True)

    def await_receipt(self, request, timeout=35.):
        deadline = time.monotonic()+timeout
        path = self.output/'receipts'/(request['task_id']+'.json')
        while time.monotonic()<deadline:
            if self.process.poll() is not None:
                raise RuntimeError('ROS launch exited before delivery')
            if path.exists():
                receipt = read_json(path)
                if any(receipt.get(k)!=request[k] for k in ('batch_id','task_id','session','artifact')):
                    raise ValueError('ROS receipt belongs to a different delivery')
                if receipt['status']!='ROS_ACCEPTED':
                    raise RuntimeError('ROS delivery rejected: '+receipt.get('error','unknown'))
                return receipt
            time.sleep(.05)  # Bounded polling of an explicit acknowledgement, not guessed DDS ordering.
        raise TimeoutError('ROS acceptance timeout')

    def close(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try: self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGKILL)
                self.process.wait(timeout=3)
        if self.log is not None: self.log.close()


def run_sequence(groups,output,*,models,config,algorithm_python,vision,delivery,run_algorithm=subprocess.run):
    """Tests may substitute only external inference/transport; production reduction stays here."""
    state = {'schema_version':'finite_capture_progress_v1','batch_id':uuid.uuid4().hex,
        'sequence_kind':'INDEPENDENT_CAPTURE_GROUPS','status':'RUNNING','exit_code':1,
        'target_task':None,'displayed_task':None,'delivery':None,'errors':[],
        'groups':[dict(g,order=i,status='PENDING',stage='pending',algorithm_attempts=0,
                       sam_attempts=0,metric_attempts=0) for i,g in enumerate(groups)]}
    active = None
    capture_identities = set()
    def save(): atomic_json(output/'progress.json',state)
    try:
        save()
        (output/'receipts').mkdir()
        delivery.start()
        for row in state['groups']:
            active = row
            folder = output/row['task_id']
            state['target_task']=row['task_id']
            state['delivery']=None
            row.update(status='VALIDATING_INPUT',stage='capture_validation',started_monotonic=time.monotonic(),
                       output_directory=str(folder))
            save()
            try:
                folder.mkdir(exist_ok=False)
                inputs = verify_capture(row['capture'])
                identity=canonical_fingerprint({k:input_identity(v) for k,v in inputs.items()})
                if identity in capture_identities: raise ValueError('DUPLICATE_CAPTURE_GROUP')
                capture_identities.add(identity)
                row['input_provenance']=inputs
                save()
                summary_path = folder/'algorithm'/'summary.json'
                if row['action']=='REUSE_VERIFIED':
                    summary_path = Path(row['reuse_summary'])
                    row['stage']='reuse_validation'
                else:
                    row.update(status='ALGORITHM_RUNNING',stage='algorithm',algorithm_attempts=1)
                    save()
                    started = time.monotonic()
                    env = {k:v for k,v in os.environ.items() if k not in ('PYTHONPATH','PYTHONHOME')}
                    env['PYTHONPATH']=os.pathsep.join((str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src'),str(ROOT/'tools')))
                    with (folder/'algorithm.log').open('w',encoding='utf-8') as log:
                        result = run_algorithm([str(algorithm_python),str(ROOT/'tools/run_workcell_perception_once.py'),
                            '--capture',row['capture'],'--vision',str(vision),'--models',str(output/'models.json'),
                            '--output-directory',str(folder/'algorithm')],env=env,stdout=log,stderr=subprocess.STDOUT)
                    row['algorithm_seconds']=time.monotonic()-started
                    if summary_path.exists():
                        partial = read_json(summary_path)
                        row['modules']=partial.get('runs',[])
                        for key in ('sam_attempts','metric_attempts'):
                            row[key]=sum(m.get(key,0) for m in row['modules'])
                    if result.returncode!=0:
                        raise RuntimeError('single-capture CLI exited '+str(result.returncode))
                row['stage']='artifact_validation'
                summary,reference,counts = verify_result(summary_path,inputs,models,config)
                row.update(status='ARTIFACT_READY',artifact=reference,run_id=summary['run_id'],counts=counts,
                    result_origin='REUSED_VERIFIED' if row['action']=='REUSE_VERIFIED' else 'NEW_ALGORITHM_RUN',
                    artifact_completed_monotonic=time.monotonic(),summary_path=str(summary_path))
                atomic_json(folder/'result.json',row)
                save()
                row['stage']='ros_delivery'
                request = dict(batch_id=state['batch_id'],task_id=row['task_id'],order=row['order'],
                    session=uuid.uuid4().hex,artifact=reference,submitted_monotonic=time.monotonic())
                state['delivery']=request
                row['switch_submitted_monotonic']=request['submitted_monotonic']
                save()
                receipt = delivery.await_receipt(request)
                row.update(status='ROS_ACCEPTED',stage='completed',
                    ros={k:v for k,v in receipt.items() if k!='world_events'},
                    ros_receipt=str(output/'receipts'/(row['task_id']+'.json')))
                state['displayed_task']=row['task_id']
            except Exception as exc:
                row.update(status='FAILED',error_type=type(exc).__name__,error=str(exc),failure_stage=row['stage'])
                print(f"{row['task_id']}/{row['stage']}: {exc}",file=sys.stderr,flush=True)
            row['finished_monotonic']=time.monotonic()
            save()
            active = None
    except BaseException as exc:
        state['errors'].append({'type':type(exc).__name__,'error':str(exc)})
        if active is not None: active.update(status='FAILED',error=str(exc),failure_stage=active['stage'])
        if isinstance(exc,(KeyboardInterrupt,SystemExit)):
            state['exit_code']=130 if isinstance(exc,KeyboardInterrupt) else 1
        print(f'batch failure: {type(exc).__name__}: {exc}',file=sys.stderr,flush=True)
    finally:
        try: delivery.close()
        except Exception as exc:
            state['errors'].append({'type':type(exc).__name__,'error':'ROS cleanup: '+str(exc)})
            print(f'ROS cleanup failed: {exc}',file=sys.stderr,flush=True)
        state['status']='COMPLETED' if state['groups'] and all(r['status']=='ROS_ACCEPTED' for r in state['groups']) and not state['errors'] else 'PARTIAL_OR_FAILED'
        state['exit_code']=0 if state['status']=='COMPLETED' else state['exit_code'] or 1
        try: save()
        except Exception as exc:
            state['exit_code']=1
            print(f'progress write failed: {exc}',file=sys.stderr,flush=True)
    return state['exit_code']


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--models',type=Path,required=True)
    parser.add_argument('--vision',type=Path,required=True)
    parser.add_argument('--algorithm-python',type=Path,required=True)
    parser.add_argument('--domain-id',type=int,default=174)
    args=parser.parse_args(argv)
    try:
        groups=manifest_groups(args.manifest)
        output=args.output.resolve()
        if any(output.is_relative_to(Path(g['capture'])) for g in groups):
            raise ValueError('batch output must be outside original capture directories')
        output.mkdir(exist_ok=False)
        models=read_json(args.models)
        verify_sam_files(models)
        revision=subprocess.check_output(['git','-C',str(args.vision),'rev-parse','HEAD'],text=True,encoding='utf-8').strip()
        if revision!='1d208f2ed380a207e6e46b4a62d2ac640edfe477':
            raise ValueError('upstream geometry/SAM integration commit differs from pinned implementation')
        import yaml
        config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text(encoding='utf-8'))
        atomic_json(output/'models.json',models)
        atomic_json(output/'manifest.json',{'schema_version':'finite_capture_sequence_v1',
                    'sequence_kind':'INDEPENDENT_CAPTURE_GROUPS','groups':groups})
        return run_sequence(groups,output,models=models,config=config,algorithm_python=args.algorithm_python,
            vision=args.vision,delivery=RosDelivery(output,args.domain_id))
    except Exception as exc:
        print(f'finite sequence: {type(exc).__name__}: {exc}',file=sys.stderr,flush=True)
        return 1


if __name__=='__main__':
    try: raise SystemExit(main())
    except KeyboardInterrupt: raise SystemExit(130)
