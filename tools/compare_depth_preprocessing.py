"""Compare both complete preprocessing runs, separating corrected audits from geometry."""
import argparse
import gc
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare(before, after):
    summaries = [json.loads((p / 'measurements.json').read_text()) for p in (before, after)]
    assert summaries[0]['plan_sha256'] == summaries[1]['plan_sha256']
    for key in ('python', 'executable', 'platform'):
        assert summaries[0]['environment_before'][key] == summaries[1]['environment_before'][key]
    assert summaries[0]['numpy_version'] == summaries[1]['numpy_version']
    result = {'plan_sha256': summaries[0]['plan_sha256'], 'modules': [],
              'measurement_sha256': {str(p): sha(p / 'measurements.json') for p in (before, after)}}
    for b, a in zip(summaries[0]['modules'], summaries[1]['modules']):
        assert b['module_id'] == a['module_id'] and b['mask_ids'] == a['mask_ids']
        name = b['module_id']
        audits = [json.loads((p / name / 'metric_pointmap_filter_audit.json').read_text()) for p in (before, after)]
        rows = []
        for old, new in zip(*audits):
            assert old['mask_id'] == new['mask_id']
            clean = lambda v: {k: x for k, x in v.items() if k not in ('components', 'audit_npz')}
            row = {'mask_id': old['mask_id'], 'status_reason_and_noncomponent_evidence_equal': clean(old) == clean(new),
                   'components_before': len(old.get('components', [])), 'components_after': len(new.get('components', [])),
                   'memberships_before': sum(c['point_count'] for c in old.get('components', [])),
                   'memberships_after': sum(c['point_count'] for c in new.get('components', [])), 'arrays_equal': {}}
            if old['status'] == new['status'] == 'PASS':
                with np.load(old['audit_npz']) as x, np.load(new['audit_npz']) as y:
                    assert x.files == y.files
                    for key in x.files:
                        left, right = x[key], y[key]
                        row['arrays_equal'][key] = bool(left.dtype == right.dtype and left.shape == right.shape
                                                       and np.array_equal(left, right, equal_nan=True))
                del left, right
            rows.append(row)
        pointmaps = {}
        with np.load(before / name / 'registered_metric_pointmap.npz') as x, np.load(after / name / 'registered_metric_pointmap.npz') as y:
            assert x.files == y.files
            for key in x.files:
                if key == 'metadata_json':
                    # Component records and output paths must change; capture/calibration/source must not.
                    left, right = json.loads(str(x[key])), json.loads(str(y[key]))
                    for metadata in (left, right):
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
        result['modules'].append({'module_id': name, 'instances': rows, 'pointmap_equal': pointmaps})
        del audits
        gc.collect()
    result['all_preserved_outputs_equal'] = all(
        all(m['pointmap_equal'].values()) and all(r['status_reason_and_noncomponent_evidence_equal']
        and all(r['arrays_equal'].values()) for r in m['instances']) for m in result['modules'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.before, args.after)
    with args.output.open('x', encoding='utf-8') as out:
        json.dump(result, out, indent=2, sort_keys=True)
        out.write('\n')
    print(json.dumps({'all_preserved_outputs_equal': result['all_preserved_outputs_equal']}))


if __name__ == '__main__':
    main()
