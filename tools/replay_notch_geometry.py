"""Replay the production geometry dispatcher on frozen DINO/SAM outputs."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import uuid
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--vision',type=Path,required=True);a=p.parse_args()
    from unloading_perception.finite_sequence import read_json,atomic_json
    from unloading_perception.isaac_payload import load_capture_payload
    from unloading_perception.isaac_validation import IsaacSceneManifest
    from run_isaac_rgbd_geometry import _worker_artifacts
    from workcell_geometry_process import dispatch_geometry,effective_config
    import yaml
    config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text())
    config['vision'].update(legacy_cuboid_diagnostic=False,processing_divisor=1,sam_image_reuse=True)
    config=effective_config(config,'subprocess',1)
    a.output.mkdir(parents=True,exist_ok=False);rows=[]
    plan=read_json(a.baseline/'frozen_plan.json')
    atomic_json(a.output/'frozen_plan.json',plan)
    for group in plan['groups']:
        manifest=IsaacSceneManifest.from_dict(read_json(Path(group['capture'])/'manifest.json'))
        destination=a.output/group['name'];destination.mkdir()
        for camera in manifest.cameras:
            module=camera['module_id'];source=(a.baseline/group['name']/module).resolve()
            payload=load_capture_payload(source,manifest,expected_module_id=module,with_instance_masks=True)
            artifacts=_worker_artifacts(source,payload=payload)
            frozen={k:hashlib.sha256(Path(v['path']).read_bytes()).hexdigest() for k,v in artifacts.items()}
            row=dict(group=group['name'],module=module,frozen_sam_artifacts=frozen,detector_and_sam='REUSED_EXACT_BASELINE_OUTPUTS')
            try:
                result=dispatch_geometry(backend='subprocess',blas_threads=1,geometry_python=Path(sys.executable),
                    run_id=uuid.uuid4().hex,scene=group['scene'],module_dir=source,manifest=manifest,artifacts=artifacts,
                    config=config,vision_root=a.vision,upstream_python=Path(sys.executable),timeout=1200,
                    payload=payload,output_directory=destination/module,stage_trace=True)
                row.update(status='COMPLETED',timing=result['timing_seconds'])
            except Exception as exc:row.update(status='FAILED',error=str(exc))
            if frozen!={k:hashlib.sha256(Path(v['path']).read_bytes()).hexdigest() for k,v in artifacts.items()}:
                raise ValueError('FROZEN_SAM_ARTIFACT_CHANGED')
            rows.append(row);atomic_json(a.output/'progress.json',dict(groups=rows,stop_after='geometry',
                later_stages={s:'NOT_RUN' for s in ('contacts','fusion','ros','planning','execution')}))
    return int(any(r['status']!='COMPLETED' for r in rows))


if __name__=='__main__':raise SystemExit(main())
