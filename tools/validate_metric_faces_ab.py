"""Bounded, fixed-input legacy diagnostic A/B; reuse SAM, run real metric faces.

Prepare once before either run. Each run uses a new process/directory, and calls
the production module entry, fusion publisher and artifact loader unchanged.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'packages/unloading_contracts/src'), str(ROOT/'tools')]

import numpy as np
import yaml
from unloading_contracts import canonical_fingerprint, to_wire
from unloading_perception.algorithm_artifact import load_algorithm_artifact, publish_algorithm_run
from unloading_perception.isaac_payload import load_capture_payload
from run_isaac_rgbd_geometry import _worker_artifacts, _run_secondary_module, IsaacSceneManifest
from compare_depth_preprocessing import require_roster


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def verify_plan(plan):
    require_roster(plan['modules'], 'module_id', ['module_0_upper', 'module_1_lower'], 'fixed modules')
    for m, count in zip(plan['modules'], (16, 31)):
        require_roster([{'id': i} for i in m['mask_ids']], 'id', list(range(1, count+1)), m['module_id'])
        if m['mask_array_shape'] != [count, 1944, 2592]:
            raise ValueError(f'{m["module_id"]}: unexpected original resolution')
    for path, sha in plan['input_sha256'].items():
        if digest(path) != sha:
            raise ValueError(f'INPUT_HASH_MISMATCH: {path}')


def prepare(args):
    old = read(args.input_plan)
    verify_plan(old)
    manifest = IsaacSceneManifest.from_dict(read(old['capture_manifest']))
    for m in old['modules']:
        payload = load_capture_payload(m['input_directory'], manifest, with_instance_masks=True)
        artifacts = _worker_artifacts(Path(m['input_directory']), payload=payload)
        if artifacts['cargo_masks.npz']['path'] != m['masks_path']:
            raise ValueError('MASK_PATH_MISMATCH')
        with np.load(m['masks_path']) as data:
            require_roster([{'id': int(i)} for i in data['mask_ids']], 'id', m['mask_ids'], m['module_id'])
            if list(data['masks'].shape) != m['mask_array_shape']:
                raise ValueError('MASK_SHAPE_MISMATCH')
    from diagnose_metric_calibration import load_extractor
    load_extractor(args.vision)  # genuine pinned-version check; no legacy subprocess
    original_index = Path(old['modules'][0]['input_directory']).parent/'algorithm_artifact.json'
    _, index = load_algorithm_artifact(original_index, digest(original_index))
    config = yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text(encoding='utf-8'))
    if config['vision']['pointcloud_filter'] != old['config']:
        raise ValueError('PREPROCESSING_CONFIG_CHANGED')
    plan = {**old, 'baseline_sha': 'dbc70a843618a47a6424b7311a039012a4e64498',
        'source_plan_sha256': digest(args.input_plan), 'config': config,
        'model_manifest': index['model_manifest'], 'vision_root': str(args.vision.resolve()),
        'extractor_file_sha256': digest(args.vision/'pipeline/geometry/recover_box_cuboids_3d.py'),
        'runs': ['A: diagnostic enabled once, upper then lower', 'B: diagnostic disabled once, upper then lower'],
        'python': sys.version, 'executable': sys.executable, 'numpy_version': np.__version__}
    write(args.plan, plan)


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = {'status': 'RUNNING', 'modules': [], 'diagnostic_enabled': args.diagnostic == 'on',
               'plan_sha256': digest(args.plan), 'planning_admissible': False,
               'load_before': os.getloadavg(), 'python': sys.version, 'executable': sys.executable,
               'platform': platform.platform(), 'numpy_version': np.__version__}
    wall = perf_counter()
    try:
        plan = read(args.plan)
        verify_plan(plan)
        if (plan['python'], plan['executable'], plan['numpy_version']) != (sys.version, sys.executable, np.__version__):
            raise ValueError('INTERPRETER_ENVIRONMENT_CHANGED')
        vision = Path(plan['vision_root'])
        if digest(vision/'pipeline/geometry/recover_box_cuboids_3d.py') != plan['extractor_file_sha256']:
            raise ValueError('EXTRACTOR_FILE_CHANGED')
        config = deepcopy(plan['config'])
        config['vision']['legacy_cuboid_diagnostic'] = summary['diagnostic_enabled']
        summary['effective_config'] = config
        summary['code_sha256'] = {p: digest(ROOT/p) for p in ('tools/run_isaac_rgbd_geometry.py',
            'tools/metric_depth_runner.py', 'src/unloading_perception/rgbd.py')}
        write(output/'effective_config.json', config)
        manifest = IsaacSceneManifest.from_dict(read(plan['capture_manifest']))
        summary['initial_input_verification_seconds'] = perf_counter()-wall
        results = {}
        for m in plan['modules']:
            row = {'module_id': m['module_id'], 'mask_ids': m['mask_ids'], 'status': 'RUNNING'}
            summary['modules'].append(row)
            t = perf_counter()
            payload = load_capture_payload(m['input_directory'], manifest, with_instance_masks=True)
            artifacts = _worker_artifacts(Path(m['input_directory']), payload=payload)
            row['input_load_binding_worker_verification_seconds'] = perf_counter()-t
            result = _run_secondary_module(scene='FULL_STACK_NOMINAL', module_dir=Path(m['input_directory']),
                manifest=manifest, artifacts=artifacts, config=config, vision_root=vision,
                upstream_python=Path(sys.executable), timeout=1200, payload=payload,
                output_directory=output/m['module_id'])
            geometry = read(output/m['module_id']/'rgbd_cuboids.json')
            require_roster(geometry['instances'], 'mask_id', m['mask_ids'], m['module_id']+'/final geometry')
            if len(result['observation'].cargo) != len(m['mask_ids']):
                raise ValueError(f'{m["module_id"]}: missing observation instances')
            row.update(status='COMPLETED', timing_seconds=result['timing_seconds'],
                legacy_cuboid_diagnostic=result['legacy_cuboid_diagnostic'],
                instance_count=len(geometry['instances']), face_count=sum(len(r['camera_facing_faces']) for r in geometry['instances']),
                instances_without_faces=sum(not r['camera_facing_faces'] for r in geometry['instances']),
                accepted_complete_cuboids=sum(r['accepted'] for r in geometry['instances']))
            results[m['module_id']] = result
            print(json.dumps(row), flush=True)
        t = perf_counter()
        reference = publish_algorithm_run(output, results, expected_modules=[m['module_id'] for m in plan['modules']],
            run_id=output.name, models=plan['model_manifest'], config=config,
            write_json=lambda p, v: p.write_text(json.dumps(v, indent=2, sort_keys=True), encoding='utf-8'))
        summary['fusion_and_artifact_publication_seconds'] = perf_counter()-t
        t = perf_counter()
        observation, _ = load_algorithm_artifact(reference['path'], reference['sha256'])
        summary['artifact_load_validation_seconds'] = perf_counter()-t
        summary.update(artifact=reference, fused_objects=len(observation.cargo), unknown_regions=len(observation.unknown_regions),
                       status='COMPLETED', raw_image_automatic=False)
        t = perf_counter()
        verify_plan(plan)
        summary['final_input_hash_verification_seconds'] = perf_counter()-t
    except Exception as exc:
        summary.update(status='FAILED', error_type=type(exc).__name__, error=str(exc))
    finally:
        summary['wall_seconds_before_summary_write'] = perf_counter()-wall
        summary['load_after'] = os.getloadavg()
        write(output/'ab_summary.json', summary)
    return 0 if summary['status'] == 'COMPLETED' else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['prepare', 'run'])
    p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--input-plan', type=Path)
    p.add_argument('--vision', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--diagnostic', choices=['on', 'off'])
    a = p.parse_args()
    if a.mode == 'prepare':
        prepare(a)
        return 0
    return run(a)


if __name__ == '__main__':
    raise SystemExit(main())
