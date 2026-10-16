"""Fixed frame32 + fresh notch through the resident production entry, stop at geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.finite_sequence import verify_capture,atomic_json,read_json
from unloading_perception.review_selection import validate_review_capture


def main():
    p=argparse.ArgumentParser();p.add_argument('--frame32',type=Path,required=True)
    p.add_argument('--notch',type=Path,required=True);p.add_argument('--visibility',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True);p.add_argument('--vision',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policy',type=Path,default=ROOT/'configs/perception/notch_geometry_review.json');a=p.parse_args()
    policy=read_json(a.policy);visibility=read_json(a.visibility)
    if not all(visibility[k] for k in ('has_single','has_double','has_triple')):
        raise ValueError('NOTCH_VISIBLE_FACE_REQUIREMENTS_NOT_CONFIRMED')
    if Path(visibility['capture']).resolve()!=a.notch.resolve():raise ValueError('VISIBILITY_CAPTURE_MISMATCH')
    groups=[]
    for spec,capture in zip(policy['samples'],(a.frame32,a.notch)):
        inputs=verify_capture(capture,scene=spec['scene'])
        for module,identity in inputs.items():
            validate_review_capture(identity,module,policy)
            for key in ('sensor_epoch','frame_sequence'):
                if key in spec and identity[key]!=spec[key]:raise ValueError('DECLARED_CAPTURE_IDENTITY_MISMATCH')
        groups.append(dict(**spec,capture=str(capture.resolve()),inputs=inputs))
    a.output.mkdir(parents=True,exist_ok=False)
    files=['tools/vision_resident_worker.py','tools/sam_image_session.py',
        'tools/grounding_dino_runtime.py','src/unloading_perception/proposals.py',
        'configs/perception/grounding_dino.json','configs/isaac/perception_validation.yaml']
    hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files}
    atomic_json(a.output/'frozen_plan.json',dict(groups=groups,policy=policy,model_manifest=read_json(a.models),
        frozen_files=hashes,visibility_sha256=hashlib.sha256(a.visibility.read_bytes()).hexdigest(),
        gt_role='legacy archive integrity and post-freeze evaluation only; never detector/plane/completion input'))
    from workcell_video_worker import RuntimeCache
    from run_workcell_perception_once import main as run
    cache=RuntimeCache();rows=[]
    for g in groups:
        code=run(['--capture',g['capture'],'--capture-scene',g['scene'],'--vision',str(a.vision),
            '--models',str(a.models),'--output-directory',str(a.output/g['name']),
            '--stop-after','geometry','--geometry-backend','subprocess','--geometry-blas-threads','1',
            '--geometry-timeout','1200','--no-legacy-cuboid-diagnostic','--stage-trace','--evaluate-gt'],runtime_factory=cache)
        rows.append(dict(name=g['name'],exit_code=code,resident_model_loads=cache.loads))
        atomic_json(a.output/'progress.json',dict(groups=rows,stop_after='geometry',
            later_stages={s:'NOT_RUN' for s in ('contacts','fusion','ros','planning','execution')}))
    return int(any(r['exit_code'] for r in rows))


if __name__=='__main__':raise SystemExit(main())
