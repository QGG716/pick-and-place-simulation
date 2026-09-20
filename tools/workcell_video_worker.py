"""One GPU process, one resident SAM, serial immutable RGB-D jobs from the demo."""
import argparse
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.finite_sequence import atomic_json,read_json
from run_workcell_perception_once import main as run_capture


class RuntimeCache:
    def __init__(self): self.runtime=None; self.identity=None; self.loads=0
    def __call__(self,args):
        identity=(str(args.upstream_root.resolve()),args.sam_model)
        if self.runtime is None:
            from vision_resident_worker import ResidentRuntime
            self.runtime=ResidentRuntime(args); self.identity=identity; self.loads+=1
        elif identity!=self.identity:
            raise ValueError('resident model identity changed')
        else:
            self.runtime.args=args
            self.runtime.output_root=args.output_root.resolve()
            self.runtime.output_root.mkdir(exist_ok=False)
        return self.runtime


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--models',type=Path,required=True)
    parser.add_argument('--vision',type=Path,required=True)
    args=parser.parse_args()
    cache=RuntimeCache(); seen=set()
    while not (args.output/'stop-worker').exists():
        path=args.output/'worker-request.json'
        if not path.exists(): time.sleep(.05); continue
        request=read_json(path)
        identity=request['task_id']
        if identity in seen: time.sleep(.05); continue
        seen.add(identity)
        if len(seen)>16: raise ValueError('finite demonstration exceeded 16 jobs')
        started=time.monotonic()
        result={'task_id':identity,'frame_sequence':request['frame_sequence'],'source_time':request['source_time'],
                'started_monotonic':started,'status':'RUNNING'}
        atomic_json(args.output/'worker-status.json',result)
        try:
            code=run_capture(['--capture',request['capture'],'--vision',str(args.vision),
                '--models',str(args.models),'--output-directory',request['algorithm_output']],runtime_factory=cache)
            summary=read_json(Path(request['algorithm_output'])/'summary.json')
            result.update(exit_code=code,status='COMPLETED' if code==0 else 'FAILED',summary=summary)
        except Exception as exc:
            result.update(exit_code=1,status='FAILED',error=f'{type(exc).__name__}: {exc}')
        result.update(finished_monotonic=time.monotonic(),wall_seconds=time.monotonic()-started,
                      resident_model_loads=cache.loads)
        atomic_json(args.output/'worker-status.json',result)
    return 0


if __name__=='__main__': raise SystemExit(main())
