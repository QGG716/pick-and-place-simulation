"""Fixed accepted states, matched warmup and profile toggle; no search."""
from __future__ import annotations
import argparse
import gzip
import json
from pathlib import Path
from run_tesseract_ompl_audit import sha,write
from unloading_sim.tesseract_ompl_backend import NativeWorker


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--worker',required=True)
    ap.add_argument('--audit',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    audit=json.loads((args.audit/'audit.json').read_text())
    assert audit['native_path']['status']=='AUDIT_VALID' and audit['worker_sha256']==sha(args.worker)
    scene=json.loads((args.audit/'scene.json').read_text())
    with gzip.open(args.audit/'native-samples.json.gz','rt') as f:
        samples=sorted(json.load(f),key=lambda s:(s['edge'],s['index']))
    # 100 fixed, distinct, actually checked states distributed along the known path.
    states=[samples[i*(len(samples)-1)//99]['q_rad'] for i in range(100)]
    assert len({tuple(q) for q in states})==100
    worker=NativeWorker(args.worker)
    request=dict(scene=scene,operation='audit',states=states,q_start=states[0],q_goal=states[-1],
                 seed=71070,max_state_checks=1000)
    output=dict(worker_sha256=sha(args.worker),source_sha256=sha(__file__),audit_sha256=sha(args.audit/'audit.json'),
                scene_fingerprint=scene['fingerprint'],states=states,search_started=False,runs=[],
                note='Two fixed off/on pairs after identical warmup; no sweep or search. Single pairs are noisy; no calibrated pure overhead claim.')
    try:
        output['warmup']=worker.call({**request,'profile':False})
        for profile in [False,True,False,True]:
            raw=worker.call({**request,'profile':profile})
            assert raw['status']=='AUDIT_VALID' and raw['counters']['actual_state_computations']==100
            output['runs'].append(raw)
        write(args.output,output)
    finally: worker.close()

if __name__=='__main__': main()
