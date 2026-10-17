"""Replay the production geometry dispatcher on frozen DINO/SAM outputs."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import uuid
from datetime import datetime, timezone
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]


def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--vision',type=Path,required=True)
    p.add_argument('--approximate-mode',choices=['off','outline_projection','raw_boundary']);a=p.parse_args()
    from unloading_perception.finite_sequence import read_json,atomic_json
    from unloading_perception.isaac_payload import load_capture_payload
    from unloading_perception.isaac_validation import IsaacSceneManifest
    from run_isaac_rgbd_geometry import _worker_artifacts
    from workcell_geometry_process import dispatch_geometry,effective_config
    import yaml
    config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text())
    config['vision'].update(legacy_cuboid_diagnostic=False,processing_divisor=1,sam_image_reuse=True)
    if a.approximate_mode:config['vision'].setdefault('face_quality',{})['approximate_cuboid_mode']=a.approximate_mode
    config=effective_config(config,'subprocess',1)
    a.output.mkdir(parents=True,exist_ok=False);rows=[]
    plan=read_json(a.baseline/'frozen_plan.json')
    atomic_json(a.output/'frozen_plan.json',plan)
    code_files=['tools/replay_notch_geometry.py','tools/workcell_geometry_process.py',
        'tools/run_isaac_rgbd_geometry.py','tools/metric_depth_runner.py',
        'src/unloading_perception/metric_faces.py','src/unloading_perception/physical_bounds.py',
        'src/unloading_perception/approximate_cuboid.py','configs/perception/approximate_cuboid_review.json',
        'src/unloading_perception/metric_support.py','src/unloading_perception/final_geometry.py',
        'configs/perception/metric_geometry_acceptance.json','configs/isaac/perception_validation.yaml']
    atomic_json(a.output/'geometry_run_identity.json',dict(started_utc=datetime.now(timezone.utc).isoformat(),
        source_directory=str(ROOT),frozen_model_run=str(a.baseline.resolve()),
        frozen_plan_sha256=hashlib.sha256((a.baseline/'frozen_plan.json').read_bytes()).hexdigest(),
        code_sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in code_files if (ROOT/f).exists()},
        config=config,solver_inputs='RGB-D, SAM masks, calibration, frozen algorithm config; no GT geometry',
        archive_gt_validation='legacy capture loader verifies archive; GT never enters fit_metric_faces'))
    for group in plan['groups']:
        manifest=IsaacSceneManifest.from_dict(read_json(Path(group['capture'])/'manifest.json'))
        destination=a.output/group['name'];destination.mkdir()
        for camera in manifest.cameras:
            module=camera['module_id'];source=(a.baseline/group['name']/module).resolve()
            payload=load_capture_payload(source,manifest,expected_module_id=module,with_instance_masks=True)
            from unloading_perception.review_selection import validate_review_capture
            validate_review_capture(read_json(source/'capture_metadata.json'),module,plan['policy'])
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
