import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from tools.compare_depth_preprocessing import compare, main, sha


def write(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def fixture(tmp_path):
    plan = tmp_path/'plan.json'
    specs = [{'module_id': m, 'mask_ids': [1, 2]} for m in ('upper', 'lower')]
    write(plan, {'modules': specs})
    roots = [tmp_path/'before', tmp_path/'after']
    for root in roots:
        root.mkdir()
        write(root/'measurements.json', {'plan_sha256': sha(plan), 'modules': specs,
            'environment_before': {'python': 'same', 'executable': 'same', 'platform': 'same'}, 'numpy_version': 'same'})
        for spec in specs:
            folder = root/spec['module_id']; folder.mkdir()
            audits = []
            for i in spec['mask_ids']:
                path = folder/f'{i}.npz'
                np.savez(path, raw_valid_mask=np.ones((1, 1), bool), filtered_mask=np.ones((1, 1), bool),
                         rejected_reason=np.zeros((1, 1), np.uint8), raw_points_camera_xyz_m=np.ones((1, 1, 3)),
                         filtered_points_camera_xyz_m=np.ones((1, 1, 3)))
                audits.append({'mask_id': i, 'status': 'PASS', 'audit_npz': str(path), 'components': []})
            write(folder/'metric_pointmap_filter_audit.json', audits)
            np.savez(folder/'registered_metric_pointmap.npz', points=np.ones((1, 1, 3)), valid_mask=np.ones((1, 1), bool),
                     intrinsics=np.eye(3), K=np.eye(3), depth_optical_z_m=np.ones((1, 1)),
                     metadata_json=np.asarray(json.dumps({'filter_evidence': {'per_instance_filtering': audits}})))
    return plan, *roots


@pytest.mark.parametrize('failure', ['missing_module', 'both_empty', 'missing_instance', 'duplicate_id', 'missing_file', 'array_difference'])
def test_plan_failures_and_value_differences_have_nonzero_cli(tmp_path, failure):
    plan, before, after = fixture(tmp_path)
    if failure in ('missing_module', 'both_empty'):
        for root in (before, after) if failure == 'both_empty' else (after,):
            p = root/'measurements.json'; v = json.loads(p.read_text()); v['modules'] = [] if failure == 'both_empty' else v['modules'][:-1]; write(p, v)
    elif failure in ('missing_instance', 'duplicate_id'):
        p = after/'upper/metric_pointmap_filter_audit.json'; v = json.loads(p.read_text())
        if failure == 'missing_instance': v.pop()
        else: v[-1]['mask_id'] = 1
        write(p, v)
    elif failure == 'missing_file':
        (after/'upper/2.npz').unlink()
    else:
        p = after/'upper/2.npz'
        with np.load(p) as data: arrays = {k: data[k] for k in data.files}
        arrays['filtered_points_camera_xyz_m'][0, 0, 0] = 2
        np.savez(p, **arrays)
    out = tmp_path/'report.json'
    completed = subprocess.run([sys.executable, '-O', 'tools/compare_depth_preprocessing.py', '--before', str(before),
        '--after', str(after), '--plan', str(plan), '--output', str(out)], capture_output=True, text=True)
    assert completed.returncode == 1, completed.stderr
    result = json.loads(out.read_text())
    assert result['all_preserved_outputs_equal'] is False
    assert result.get('error') or result.get('differences')


def test_complete_comparison_counts_actual_passes_and_rejections(tmp_path):
    plan, before, after = fixture(tmp_path)
    result = compare(before, after, plan)
    assert result['plan_complete'] and result['all_preserved_outputs_equal']
    assert sum(m['after_pass_count'] for m in result['modules']) == 4
    for root in (before, after):
        for folder in (root/'upper', root/'lower'):
            p = folder/'metric_pointmap_filter_audit.json'; rows = json.loads(p.read_text())
            for r in rows: r.update(status='REJECTED', reason='too few points')
            write(p, rows)
    result = compare(before, after, plan)
    assert result['all_preserved_outputs_equal']
    assert sum(m['after_pass_count'] for m in result['modules']) == 0
    assert sum(m['after_rejected_count'] for m in result['modules']) == 4
