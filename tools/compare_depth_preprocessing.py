"""Compare both complete preprocessing runs, separating corrected audits from geometry."""
import argparse
import gc
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_roster(records, key, expected, context):
    ids = [r[key] for r in records]
    if not expected or len(set(expected)) != len(expected):
        raise ValueError(f'{context}: invalid/empty fixed plan')
    if len(set(ids)) != len(ids):
        raise ValueError(f'{context}: duplicate {key}: {ids}')
    if ids != list(expected):
        raise ValueError(f'{context}: expected {list(expected)}, received {ids}')


def compare(before, after, plan_path):
    plan = json.loads(Path(plan_path).read_text())
    expected_modules = [m['module_id'] for m in plan['modules']]
    require_roster(plan['modules'], 'module_id', expected_modules, 'plan modules')
    summaries = [json.loads((p / 'measurements.json').read_text()) for p in (before, after)]
    for label, summary in zip(('before', 'after'), summaries):
        if summary['plan_sha256'] != sha(Path(plan_path)):
            raise ValueError(f'{label}: fixed plan hash mismatch')
        require_roster(summary['modules'], 'module_id', expected_modules, f'{label} modules')
    for key in ('python', 'executable', 'platform'):
        if summaries[0]['environment_before'][key] != summaries[1]['environment_before'][key]:
            raise ValueError(f'environment mismatch: {key}')
    if summaries[0]['numpy_version'] != summaries[1]['numpy_version']:
        raise ValueError('numpy version mismatch')
    result = {'plan_sha256': summaries[0]['plan_sha256'], 'modules': [], 'plan_complete': True, 'differences': [],
              'measurement_sha256': {str(p): sha(p / 'measurements.json') for p in (before, after)}}
    for index, spec in enumerate(plan['modules']):
        b, a = (r['modules'][index] for r in summaries)
        name = b['module_id']
        for label, summary in (('before', b), ('after', a)):
            require_roster([{'mask_id': i} for i in summary['mask_ids']], 'mask_id', spec['mask_ids'], f'{label}/{name}/masks')
        audits = [json.loads((p / name / 'metric_pointmap_filter_audit.json').read_text()) for p in (before, after)]
        for label, rows in zip(('before', 'after'), audits):
            require_roster(rows, 'mask_id', spec['mask_ids'], f'{label}/{name}/audits')
            for row in rows:
                if row.get('status') not in ('PASS', 'REJECTED') or (row['status'] == 'REJECTED' and not row.get('reason')):
                    raise ValueError(f'{label}/{name}/{row["mask_id"]}: invalid status/rejection reason')
        rows = []
        for old, new in zip(*audits):
            clean = lambda v: {k: x for k, x in v.items() if k not in ('components', 'audit_npz')}
            row = {'mask_id': old['mask_id'], 'status_reason_and_noncomponent_evidence_equal': clean(old) == clean(new),
                   'components_before': len(old.get('components', [])), 'components_after': len(new.get('components', [])),
                   'memberships_before': sum(c['point_count'] for c in old.get('components', [])),
                   'memberships_after': sum(c['point_count'] for c in new.get('components', [])), 'arrays_equal': {}}
            if old['status'] == new['status'] == 'PASS':
                with np.load(old['audit_npz']) as x, np.load(new['audit_npz']) as y:
                    required = {'raw_valid_mask', 'filtered_mask', 'rejected_reason', 'raw_points_camera_xyz_m', 'filtered_points_camera_xyz_m'}
                    if set(x.files) != required or set(y.files) != required:
                        raise ValueError(f'{name}/{old["mask_id"]}: audit array keys mismatch')
                    for key in x.files:
                        left, right = x[key], y[key]
                        row['arrays_equal'][key] = bool(left.dtype == right.dtype and left.shape == right.shape
                                                       and np.array_equal(left, right, equal_nan=True))
                del left, right
            rows.append(row)
            if not row['status_reason_and_noncomponent_evidence_equal']:
                result['differences'].append(f'{name}/{old["mask_id"]}: status/reason/evidence mismatch')
            result['differences'].extend(f'{name}/{old["mask_id"]}/{key}: array mismatch' for key, same in row['arrays_equal'].items() if not same)
        pointmaps = {}
        with np.load(before / name / 'registered_metric_pointmap.npz') as x, np.load(after / name / 'registered_metric_pointmap.npz') as y:
            required = {'points', 'valid_mask', 'intrinsics', 'K', 'depth_optical_z_m', 'metadata_json'}
            if set(x.files) != required or set(y.files) != required:
                raise ValueError(f'{name}: pointmap array keys mismatch')
            for key in x.files:
                if key == 'metadata_json':
                    # Component records and output paths must change; capture/calibration/source must not.
                    left, right = json.loads(str(x[key])), json.loads(str(y[key]))
                    for metadata in (left, right):
                        require_roster(metadata['filter_evidence']['per_instance_filtering'], 'mask_id', spec['mask_ids'], f'{name}/pointmap metadata')
                        for audit in metadata['filter_evidence']['per_instance_filtering']:
                            audit.pop('components', None)
                            audit.pop('audit_npz', None)
                    pointmaps['metadata_except_components_and_output_paths_equal'] = left == right
                else:
                    left, right = x[key], y[key]
                    pointmaps[key] = bool(left.dtype == right.dtype and left.shape == right.shape
                                          and np.array_equal(left, right, equal_nan=True))
                del left, right
                gc.collect()
        result['differences'].extend(f'{name}/pointmap/{key}: mismatch' for key, same in pointmaps.items() if not same)
        result['modules'].append({'module_id': name, 'instances': rows, 'pointmap_equal': pointmaps,
            'before_pass_count': sum(r['status'] == 'PASS' for r in audits[0]),
            'after_pass_count': sum(r['status'] == 'PASS' for r in audits[1]),
            'before_rejected_count': sum(r['status'] == 'REJECTED' for r in audits[0]),
            'after_rejected_count': sum(r['status'] == 'REJECTED' for r in audits[1])})
        del audits
        gc.collect()
    result['all_preserved_outputs_equal'] = all(
        all(m['pointmap_equal'].values()) and all(r['status_reason_and_noncomponent_evidence_equal']
        and all(r['arrays_equal'].values()) for r in m['instances']) for m in result['modules'])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = compare(args.before, args.after, args.plan)
    except Exception as exc:
        result = {'plan_complete': False, 'all_preserved_outputs_equal': False,
                  'error_type': type(exc).__name__, 'error': str(exc)}
    with args.output.open('x', encoding='utf-8') as out:
        json.dump(result, out, indent=2, sort_keys=True)
        out.write('\n')
    print(json.dumps({'all_preserved_outputs_equal': result['all_preserved_outputs_equal']}))
    return 0 if result['all_preserved_outputs_equal'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
