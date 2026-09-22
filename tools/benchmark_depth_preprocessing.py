"""One fixed, fully audited preprocessing run per isolated code directory.

No inference, fitting or simulator. Prepare freezes every box mask before either
run. Wrappers time production functions; all original NPZ/JSON audits are written.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from time import perf_counter


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, data):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def resources():
    return {'python': sys.version, 'executable': sys.executable, 'platform': platform.platform(),
            'load_average': os.getloadavg(), 'cpu_count': os.cpu_count(),
            'processes': subprocess.check_output(['ps', '-eo', 'comm,pcpu,pmem', '--sort=-pcpu'], text=True).splitlines()[:12]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--code-root', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--prepare', action='store_true')
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--algorithm-root', type=Path)
    parser.add_argument('--baseline-sha')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--label')
    args = parser.parse_args()
    code = args.code_root.resolve()
    sys.path[:0] = [str(code / p) for p in ('src', 'packages/unloading_contracts/src', 'tools')]
    import numpy as np
    import yaml
    import run_isaac_rgbd_geometry as entry
    from unloading_perception import rgbd
    from unloading_perception.algorithm_artifact import load_algorithm_artifact
    from unloading_perception.isaac_payload import load_capture_payload, write_capture_snapshot

    if args.prepare:
        manifest_path = args.capture / 'manifest.json'
        manifest = entry.IsaacSceneManifest.from_dict(json.loads(manifest_path.read_text()))
        artifact_path = args.algorithm_root / 'algorithm_artifact.json'
        load_algorithm_artifact(artifact_path, digest(artifact_path))
        config_path = code / 'configs/isaac/perception_validation.yaml'
        config = yaml.safe_load(config_path.read_text())['vision']['pointcloud_filter']
        plan = {'baseline_sha': args.baseline_sha, 'selection_rule': 'all box/cardboard_box masks from both modules; original archive order',
                'runs': ['baseline once, upper then lower', 'candidate once, upper then lower'],
                'capture_manifest': str(manifest_path), 'config': config, 'modules': [],
                'input_sha256': {str(manifest_path): digest(manifest_path), str(artifact_path): digest(artifact_path)},
                'config_file_sha256': digest(config_path), 'benchmark_sha256': digest(__file__)}
        for module in ('module_0_upper', 'module_1_lower'):
            directory = args.algorithm_root / module
            payload = load_capture_payload(directory, manifest, with_instance_masks=True, expected_module_id=module)
            artifacts = entry._worker_artifacts(directory, payload=payload)
            masks = Path(artifacts['cargo_masks.npz']['path'])
            with np.load(masks, allow_pickle=False) as archive:
                selected = [int(i) for i, label in zip(archive['mask_ids'], archive['labels'].astype(str))
                            if label in {'box', 'cardboard_box'}]
                shape = list(archive['masks'].shape)
            for name in (*payload.raw_files, 'mode_b1_worker_response.json', 'oracle_proposals.json'):
                path = directory / name
                plan['input_sha256'][str(path)] = digest(path)
            for reference in artifacts.values():
                if isinstance(reference, dict) and 'path' in reference and Path(reference['path']).is_file():
                    plan['input_sha256'][reference['path']] = digest(reference['path'])
            response = json.loads((directory / 'mode_b1_worker_response.json').read_text().strip().splitlines()[-1])
            metrics = response['metrics_reference']['path']
            plan['input_sha256'][metrics] = digest(metrics)
            plan['modules'].append({'module_id': module, 'input_directory': str(directory),
                'masks_path': str(masks), 'mask_ids': selected, 'mask_array_shape': shape,
                'capture_id': payload.metadata.capture_id, 'K': list(payload.camera['K'])})
        write(args.plan, plan)
        print(json.dumps(plan['modules']), flush=True)
        return

    plan = json.loads(args.plan.read_text())
    assert digest(__file__) == plan['benchmark_sha256']
    for path, expected in plan['input_sha256'].items():
        assert digest(path) == expected, path
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest = entry.IsaacSceneManifest.from_dict(json.loads(Path(plan['capture_manifest']).read_text()))
    report = {'label': args.label, 'plan_sha256': digest(args.plan), 'environment_before': resources(),
        'numpy_version': np.__version__, 'code_sha256': {p: digest(code / p) for p in
        ('src/unloading_perception/rgbd.py', 'tools/run_isaac_rgbd_geometry.py')}, 'modules': []}
    original_filter, original_components = rgbd.filter_registered_instance_depth, rgbd._depth_continuous_components
    map_name = 'masked_metric_pointmap_with_filter' if hasattr(entry, 'masked_metric_pointmap_with_filter') else 'masked_metric_pointmap'
    original_map = getattr(entry, map_name)
    original_indices, original_save = np.indices, np.savez_compressed
    stats = None
    audit_start = None
    component_finished = None

    def components(mask, depth, threshold):
        nonlocal component_finished
        started = perf_counter()
        value = original_components(mask, depth, threshold)
        seconds = perf_counter() - started
        stats['components'].append({'seconds': seconds, 'count': len(value),
            'pixel_memberships': sum(map(len, value)), 'input_pixels': int(np.count_nonzero(mask))})
        component_finished = perf_counter()
        return value

    def filtering(*a, **kw):
        nonlocal component_finished
        component_finished = None
        started = perf_counter()
        record = {'status': 'REJECTED'}
        try:
            value = original_filter(*a, **kw)
            record['status'] = 'PASS'
            return value
        finally:
            ended = perf_counter()
            record['seconds'] = ended - started
            record['post_components_including_audit_records_seconds'] = 0 if component_finished is None else ended - component_finished
            stats['filter_calls'].append(record)

    def mapping(*a, **kw):
        before = sum(v['seconds'] for v in stats['filter_calls'])
        started = perf_counter()
        try:
            return original_map(*a, **kw)
        finally:
            stats['pointmap_lifting_seconds'] += perf_counter() - started - (sum(v['seconds'] for v in stats['filter_calls']) - before)

    def indices(*a, **kw):
        nonlocal audit_start
        # Audit arrays begin here in BOTH versions; no tracing inside BFS/numpy.
        if sys._getframe(1).f_code.co_name == '_build_pointmap':
            audit_start = perf_counter()
        return original_indices(*a, **kw)

    def save(*a, **kw):
        nonlocal audit_start
        started = perf_counter()
        if audit_start is not None:
            stats['audit_array_generation_seconds'] += started - audit_start
            audit_start = None
        result = original_save(*a, **kw)
        stats['audit_npz_write_seconds'] += perf_counter() - started
        return result

    for item in plan['modules']:
        payload = load_capture_payload(item['input_directory'], manifest, with_instance_masks=True)
        payload = write_capture_snapshot(payload, output / item['module_id'])
        stats = {'module_id': item['module_id'], 'mask_ids': item['mask_ids'], 'components': [], 'filter_calls': [],
                 'pointmap_lifting_seconds': 0., 'audit_array_generation_seconds': 0., 'audit_npz_write_seconds': 0.}
        rgbd._depth_continuous_components, rgbd.filter_registered_instance_depth = components, filtering
        if hasattr(entry, 'filter_registered_instance_depth'):
            entry.filter_registered_instance_depth = filtering
        setattr(entry, map_name, mapping)
        np.indices, np.savez_compressed = indices, save
        started = perf_counter()
        pointmap, audits = entry._build_pointmap(payload.directory, manifest, Path(item['masks_path']), plan['config'], payload=payload)
        stats['build_pointmap_wall_seconds'] = perf_counter() - started
        np.indices, np.savez_compressed = original_indices, original_save
        started = perf_counter()
        pointmap.write_npz(payload.directory / 'registered_metric_pointmap.npz')
        stats['combined_pointmap_write_seconds'] = perf_counter() - started
        started = perf_counter()
        write(payload.directory / 'metric_pointmap_filter_audit.json', audits)
        stats['audit_json_write_seconds'] = perf_counter() - started
        assert [a['mask_id'] for a in audits] == item['mask_ids']
        stats['audit_records'] = len(audits)
        stats['component_audit_records'] = sum(len(a.get('components', [])) for a in audits)
        stats['audit_pixel_memberships'] = sum(c['point_count'] for a in audits for c in a.get('components', []))
        stats['statuses'] = [{'mask_id': a['mask_id'], 'status': a['status'], 'reason': a.get('reason')} for a in audits]
        generated = [payload.directory / 'registered_metric_pointmap.npz', payload.directory / 'metric_pointmap_filter_audit.json',
                     *sorted((payload.directory / 'pointcloud_filter').glob('*.npz'))]
        stats['output_files'] = {str(p.relative_to(output)): {'bytes': p.stat().st_size, 'sha256': digest(p)} for p in generated}
        report['modules'].append(stats)
        write(output / (item['module_id'] + '_measurements.json'), stats)
        print(json.dumps({'module': item['module_id'], 'build_seconds': stats['build_pointmap_wall_seconds'],
                          'filter_calls': len(stats['filter_calls']), 'components': stats['component_audit_records']}), flush=True)
    report['environment_after'] = resources()
    report['inputs_unchanged'] = all(digest(p) == h for p, h in plan['input_sha256'].items())
    assert report['inputs_unchanged']
    write(output / 'measurements.json', report)


if __name__ == '__main__':
    main()
