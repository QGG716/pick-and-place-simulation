"""Exactly three fixed module runs: encoder reuse, then one working resolution.

No fusion claim for a single module. All records and failures are preserved.
"""
import argparse
import json
from pathlib import Path
import sys
from time import perf_counter

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('capture','output','vision','models'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    from unloading_perception.finite_sequence import verify_capture,verify_sam_files,atomic_json
    from unloading_perception.isaac_validation import IsaacSceneManifest
    from unloading_perception.isaac_payload import load_capture_payload,write_capture_snapshot
    from run_metric_small_matrix import oracle_proposals,infer
    from run_isaac_rgbd_geometry import _worker_artifacts
    from workcell_geometry_process import dispatch_geometry,effective_config
    from vision_resident_worker import ResidentRuntime,parser
    import yaml
    inputs=verify_capture(a.capture);models=json.loads(a.models.read_text());verify_sam_files(models)
    manifest=IsaacSceneManifest.from_dict(json.loads((a.capture/'manifest.json').read_text()))
    camera=next(c for c in manifest.cameras if c['module_id']=='module_0_upper')
    payload=load_capture_payload(a.capture/'FULL_STACK_NOMINAL/modules/module_0_upper',manifest,
                                 expected_module_id='module_0_upper',with_instance_masks=True)
    a.output.mkdir(exist_ok=False)
    arms=[('A-full-no-reuse',1,False),('B-full-reuse',1,True),('C-half-reuse',2,True)]
    atomic_json(a.output/'frozen_plan.json',dict(inputs=inputs,models=models,arms=arms,
        module='module_0_upper',frame=payload.metadata.frame_sequence,all_proposals=True,retries=0,
        encoder_reuse_gate='exact masks, IDs, scores and decision records (only paths/timing differ)',
        resolution_gates=dict(roster_exact=True,minimum_mask_iou=.97,maximum_boundary_p95_native_px=2.,
            maximum_main_normal_degrees=.5,maximum_main_offset_m=.003,minimum_main_support_ratio=.95,
            no_new_geometric_acceptance_failures=True),
        resolution_scope='SAM and initial extraction working grid; native-depth final fit and independent validation',
        final_two_groups='use full resolution if any resolution gate fails; no additional levels'))
    start=perf_counter()
    runtime=ResidentRuntime(parser().parse_args(['--upstream-root',str(a.vision),'--output-root',str(a.output/'sam-runs'),
        '--input-root',str(a.output),'--sam-model',models['sam']['snapshot_path']]))
    rows=[];cold=perf_counter()-start
    for name,divisor,reuse in arms:
        folder=a.output/name;folder.mkdir()
        config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text())
        config['vision'].update(processing_divisor=divisor,sam_image_reuse=reuse)
        config=effective_config(config,'subprocess',1)
        runtime.args.processing_divisor=divisor;runtime.args.sam_image_reuse=reuse
        snapshot=write_capture_snapshot(payload,folder/'module_0_upper');module=folder/'module_0_upper'
        atomic_json(folder/'effective_config.json',config)
        start=perf_counter();row=dict(arm=name,status='RUNNING',cold_model_seconds=cold if not rows else 0.)
        rows.append(row);atomic_json(a.output/'progress.json',rows)
        try:
            binding,_,_=oracle_proposals(module,camera,payload=snapshot)
            sam_start=perf_counter();response=infer(runtime,module,binding,camera,name,payload=snapshot)
            row['sam_seconds']=perf_counter()-sam_start
            response=json.loads((module/'mode_b1_worker_response.json').read_text())
            if response['status']!='COMPLETE':raise RuntimeError(str(response))
            artifacts=_worker_artifacts(module,payload=snapshot)
            result=dispatch_geometry(backend='subprocess',blas_threads=1,geometry_python=Path(sys.executable),
                run_id=name,scene='FULL_STACK_NOMINAL',module_dir=module,manifest=manifest,artifacts=artifacts,
                config=config,vision_root=a.vision,upstream_python=Path(sys.executable),timeout=1200,
                payload=snapshot,stage_trace=True)
            row.update(status='COMPLETED',geometry_timing=result['timing_seconds'])
        except Exception as exc:
            import traceback
            row.update(status='FAILED',error=str(exc));(folder/'failure.txt').write_text(traceback.format_exc())
        row['wall_seconds']=perf_counter()-start
        atomic_json(a.output/'progress.json',rows)
        if row['status']=='FAILED':return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
