"""Audit the saved engineering path without invoking either search planner."""
from __future__ import annotations
import argparse
import gzip
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from run_tesseract_ompl_comparison import fixture_context, ROOT
from unloading_sim.tesseract_scene import export_scene
from unloading_sim.tesseract_ompl_backend import NativeWorker


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--worker', required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--native-only', action='store_true')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    fixtures = ROOT / 'tests/fixtures/tesseract_ompl'
    previous = ROOT / 'docs/validation/evidence/backend_tesseract_ompl_v0_1/comparison.json'
    case = next(c for c in json.loads(previous.read_text())['cases'] if c['name'] == 'empty_detour')
    legacy = next(r for r in case['results'] if r['backend'] == 'legacy')
    path = np.asarray(legacy['path'])
    assert legacy['status'] == 'VERIFIED' and len(path) == 29
    world, segment, c, attachment = fixture_context(fixtures/'historical_state.json', fixtures/'historical_segment.json')
    scene = export_scene(c, world.all_obstacles, stage='pregrasp')
    assert case['seed'] == 71070 and case['source_path_indices'] == [78, 208]
    assert np.array_equal(path[0], segment['path'][78]) and np.array_equal(path[-1], segment['path'][208])
    # Absolute model resource paths are retained on this same server.
    assert all(scene[k] == case[k] for k in ['fingerprint'] if k in case)
    for key in ['model_fingerprint', 'tool_fingerprint', 'policy_fingerprint']:
        assert scene[key] == case[key], key
    output = dict(schema='known_legacy_path_audit_v2', source_comparison_sha256=sha(previous),
        source_request={k:v for k,v in case.items() if k!='results'}, path=path.tolist(),
        original_native_benchmark_worker_sha256='8cb22acfb19898d8fbdd271cc1aa5e5270993b3ba5f405cd307ba120f1dd7c9e',
        worker_sha256=sha(args.worker), sources={str(p.relative_to(ROOT)):sha(p) for p in
            [ROOT/'native/tesseract_ompl/worker.cpp', Path(__file__),
             *sorted((ROOT/'src/unloading_sim').glob('*.py'))]},
        scene_fingerprint=scene['fingerprint'], constraints=scene['constraints'],
        authority_sampling='2*max(1,ceil(Linf/edge_resolution),ceil(4*L1/.0025)) intervals, np.linspace',
        native_sampling='max(1,ceil(L1/.0003125),2*ceil(Linf/edge_resolution)) intervals; actual q exported',
        search_started=False, isaac_run=False)
    write(args.output/'scene.json',scene)
    write(args.output/'audit.json',output)
    def progress(event): print(json.dumps(dict(event=event)),flush=True)
    if not args.native_only:
        progress('current_authority_path_begin')
        began=perf_counter()
        rejection=c._path_failure(path,world.all_obstacles,stage='pregrasp',diagnostic_origin='v2_known_path')
        output['authority_path']=dict(accepted=rejection is None, failure=rejection, wall_s=perf_counter()-began,
                                      statistics=dict(c._statistics))
        write(args.output/'audit.json',output)
        progress('current_authority_path_finish')
    worker=NativeWorker(args.worker)
    try:
        message=dict(scene=scene,q_start=path[0].tolist(),q_goal=path[-1].tolist(),seed=71070,
                     max_state_checks=100000,operation='audit',audit_path=path.tolist(),profile=True,trace_samples=True)
        progress('native_path_begin')
        raw=worker.call(message)
        samples=raw.pop('checked_samples',[])
        payload=json.dumps(samples,separators=(',',':'),allow_nan=False).encode()
        with gzip.open(args.output/'native-samples.json.gz','wb') as f: f.write(payload)
        output['native_samples']=dict(count=len(samples),uncompressed_sha256=hashlib.sha256(payload).hexdigest(),
                                      file='native-samples.json.gz')
        output['native_path']=raw
        write(args.output/'audit.json',output)
        progress('native_path_finish')
        if not args.native_only:
            began=perf_counter(); mismatch=None; checked=0
            for sample in samples:
                q=np.asarray(sample['q_rad'])
                failure=c._state_failure(q,world.all_obstacles,stage='pregrasp')
                native_accepted=not (raw['status']=='AUDIT_INVALID' and checked==len(samples)-1)
                checked+=1
                if (failure is None)!=native_accepted:
                    limits=np.asarray(c.robot.joint_limits)
                    mismatch=dict(classification='A_SAME_Q_MODEL_OR_RULE',sample=sample,
                        native_accepted=native_accepted,authority_accepted=failure is None,
                        native_failure=raw.get('failure'),authority_failure=failure,
                        authority_tcp=c.robot.fk(q).tolist(),
                        authority_condition=float(np.linalg.cond(c.robot.geometric_jacobian(q))),
                        authority_joint_margin=float(np.minimum(q-limits[:,0],limits[:,1]-q).min()))
                    probe=worker.call({**message,'states':[q.tolist()], 'trace_samples':False,'fk_probes':[q.tolist()]})
                    mismatch['native_probe']=probe
                    break
            output['same_actual_q_comparison']=dict(checked=checked,mismatch=mismatch,wall_s=perf_counter()-began,
                complete=raw.get('audit_complete',False) and checked==len(samples),
                native_grid_differs_from_authority_grid=True,
                budget_is_not_collision=True)
            write(args.output/'audit.json',output)
            progress('same_q_comparison_finish')
        # A fixed small batch measures timer overhead; this is not a planner sweep.
        batch=[path[i].tolist() for i in [0,7,14,21,28]]*20
        output['fixed_state_profile_pair']=[]
        for enabled in [False,True]:
            output['fixed_state_profile_pair'].append(worker.call({**message,'states':batch,
                'trace_samples':False,'profile':enabled}))
        write(args.output/'audit.json',output)
    finally:
        worker.log.flush()
        worker.log.seek(0)
        (args.output/'native.log').write_text(worker.log.read())
        worker.close()
    progress('audit_done')

if __name__=='__main__': main()
