"""Two fixed serial jobs through the existing entry; never latest-wins or retries."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.finite_sequence import atomic_json, verify_capture, verify_result, verify_sam_files


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--vision',type=Path,required=True)
    a=p.parse_args(argv)
    plan=json.loads(a.plan.read_text());groups=plan['groups']
    if not 1<=len(groups)<=2 or len({g['name'] for g in groups})!=len(groups):raise ValueError('INVALID_FIXED_GROUPS')
    for g in groups:
        if g['name'] not in ('frame32','frame602'):raise ValueError('UNDECLARED_INPUT_GROUP')
    # Validate all original buffers BEFORE any model initialization.
    inputs={g['name']:verify_capture(g['capture']) for g in groups}
    models=json.loads(a.models.read_text());verify_sam_files(models)
    import yaml
    from geometry_runtime_config import effective_config
    config=effective_config(yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text()),'subprocess',1)
    a.output.mkdir(exist_ok=False)
    atomic_json(a.output/'frozen_plan.json',dict(plan=plan,inputs=inputs,models=models,effective_config=config,
        selection='Both original fixed groups, all modules and instances, serial, no retries',
        visual_selection='First fused object, best candidate by existing rank even if rejected',
        policy='SIMULATION_CAPTURE / NEW_ALGORITHM_RUN / ORACLE_PROPOSAL / READ_ONLY',enable_hardware=False))
    from workcell_video_worker import RuntimeCache
    from run_workcell_perception_once import main as run_capture
    cache=RuntimeCache();rows=[]
    for g in groups:
        folder=a.output/g['name'];folder.mkdir()
        row=dict(name=g['name'],status='RUNNING',capture=g['capture']);rows.append(row)
        atomic_json(a.output/'progress.json',dict(groups=rows,status='RUNNING'))
        start=perf_counter()
        try:
            code=run_capture(['--capture',g['capture'],'--vision',str(a.vision),'--models',str(a.models),
                '--output-directory',str(folder/'algorithm'),'--geometry-backend','subprocess',
                '--geometry-blas-threads','1','--geometry-timeout','1200','--stage-trace'],runtime_factory=cache)
            if code:raise RuntimeError('ALGORITHM_FAILED: '+str(code))
            summary,ref,counts=verify_result(folder/'algorithm/summary.json',inputs[g['name']],models,config,
                                            expected_output=folder/'algorithm')
            row.update(status='COMPLETED',artifact=ref,counts=counts,run_id=summary['run_id'],
                       resident_model_loads=cache.loads,sam_object_id=id(cache.runtime.sam_model))
        except Exception as exc:
            row.update(status='FAILED',error=str(exc))
        row['algorithm_with_trace_wall_seconds']=perf_counter()-start
        atomic_json(a.output/'progress.json',dict(groups=rows,status='RUNNING'))
    ok=all(r['status']=='COMPLETED' for r in rows)
    atomic_json(a.output/'progress.json',dict(groups=rows,status='COMPLETED' if ok else 'FAILED'))
    return 0 if ok else 1


if __name__=='__main__':raise SystemExit(main())
