"""One fixed, reused-SAM dual-module acceptance through the production dispatcher.

No SAM/model, simulator, ROS, profiler or thread search is started here.
"""
import argparse
from pathlib import Path
import sys
from time import perf_counter

from workcell_geometry_process import dispatch_geometry, effective_config, reference, read


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('plan', 'reference', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args(argv)
    from validate_metric_faces_ab import verify_plan, digest, ROOT
    from unloading_perception.isaac_payload import load_capture_payload
    from unloading_perception.isaac_validation import IsaacSceneManifest
    from unloading_perception.algorithm_artifact import publish_algorithm_run, load_algorithm_artifact
    from run_isaac_rgbd_geometry import _worker_artifacts
    from run_workcell_perception_once import _write_json
    from compare_metric_faces_ab import compare
    import numpy as np
    output = args.output.resolve()
    output.mkdir(exist_ok=False)
    started = perf_counter()
    summary = {'status': 'RUNNING', 'modules': [], 'plan_sha256': digest(args.plan),
               'planning_admissible': False, 'raw_image_automatic': False,
               'validation_scope': 'reused SAM artifacts; production geometry subprocess acceptance'}
    try:
        plan = read(args.plan)
        verify_plan(plan)
        if (plan['python'], plan['executable'], plan['numpy_version']) != (sys.version, sys.executable, np.__version__):
            raise ValueError('INTERPRETER_ENVIRONMENT_CHANGED')
        if digest(Path(plan['vision_root'])/'pipeline/geometry/recover_box_cuboids_3d.py') != plan['extractor_file_sha256']:
            raise ValueError('EXTRACTOR_FILE_CHANGED')
        if plan['config']['vision']['legacy_cuboid_diagnostic'] is not False:
            raise ValueError('legacy diagnostic must remain disabled')
        # Read and verify the historical artifact before starting any geometry.
        old = read(args.reference/'ab_summary.json')
        load_algorithm_artifact(old['artifact']['path'], old['artifact']['sha256'])
        production = read(args.reference/'function_timings.json')['production_sha256']
        summary['production_sha256'] = {name: digest(ROOT/name) for name in production}
        if not production or summary['production_sha256'] != production:
            raise ValueError('GEOMETRY_CODE_CHANGED')
        summary['code_sha256'] = {name: digest(ROOT/name) for name in old['code_sha256']}
        summary['entry_code_sha256'] = {name: digest(ROOT/'tools'/name) for name in
            ('workcell_geometry_process.py', 'run_workcell_perception_once.py', 'workcell_video_worker.py',
             'metric_thread_policy.py', 'validate_workcell_geometry_process.py')}
        config = effective_config(plan['config'], 'subprocess', 1)
        summary['effective_config'] = config
        _write_json(output/'effective_config.json', config)
        summary['initial_input_verification_seconds'] = perf_counter()-started
        manifest = IsaacSceneManifest.from_dict(read(plan['capture_manifest']))
        results = {}
        for spec in plan['modules']:
            module = spec['module_id']
            source = Path(spec['input_directory'])
            t = perf_counter()
            payload = load_capture_payload(source, manifest, with_instance_masks=True, expected_module_id=module)
            artifacts = _worker_artifacts(source, payload=payload)
            row = {'module_id': module, 'mask_ids': spec['mask_ids'], 'status': 'RUNNING',
                   'parent_input_verification_seconds': perf_counter()-t}
            summary['modules'].append(row)
            result = dispatch_geometry(backend='subprocess', blas_threads=1, geometry_python=Path(sys.executable),
                run_id=output.name, scene='FULL_STACK_NOMINAL', module_dir=source, manifest=manifest,
                artifacts=artifacts, config=config, vision_root=Path(plan['vision_root']),
                upstream_python=Path(sys.executable), timeout=1200, payload=payload, output_directory=output/module)
            results[module] = result
            records = read(output/module/'rgbd_cuboids.json')['instances']
            if [r['mask_id'] for r in records] != spec['mask_ids']: raise ValueError('INSTANCE_PLAN_MISMATCH')
            row.update(status='COMPLETED', timing_seconds=result['timing_seconds'],
                       child_pid=result['geometry_process']['pid'],
                       process_timing_seconds=result['geometry_process_timing_seconds'],
                       child_timing_seconds=result['geometry_process']['timing_seconds'],
                       instance_count=len(records), face_count=sum(len(r['camera_facing_faces']) for r in records))
            _write_json(output/'progress.json', summary)
        t = perf_counter()
        ref = publish_algorithm_run(output, results, expected_modules=[m['module_id'] for m in plan['modules']],
            run_id=output.name, models=plan['model_manifest'], config=config, write_json=_write_json)
        summary['fusion_and_artifact_publication_seconds'] = perf_counter()-t
        t = perf_counter()
        observation, _ = load_algorithm_artifact(ref['path'], ref['sha256'])
        summary['artifact_load_validation_seconds'] = perf_counter()-t
        verify_plan(plan)
        summary.update(status='COMPLETED', artifact=ref, fused_objects=len(observation.cargo),
                       unknown_regions=len(observation.unknown_regions))
    except Exception as exc:
        summary.update(status='FAILED', error_type=type(exc).__name__, error=str(exc))
    summary['group_wall_seconds'] = perf_counter()-started
    _write_json(output/'ab_summary.json', summary)
    if summary['status'] != 'COMPLETED': return 1
    try:
        comparison = compare(args.reference, output, args.plan, mode='geometry-subprocess')
    except Exception as exc:
        comparison = {'plan_complete': False, 'results_equal': False, 'error': str(exc)}
    _write_json(output/'comparison.json', comparison)
    return 0 if comparison['results_equal'] else 1


if __name__ == '__main__': raise SystemExit(main())
