"""Post-adaptation nominal-cuboid evaluation. Never used by the adapter.

Reads existing mask associations only after the derived file is frozen. These
annotations are NOMINAL_CUBOID_ONLY, not the rendered appearance mesh surface.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from unloading_perception.algorithm_artifact import file_reference, load_algorithm_artifact
from unloading_perception.geometry import validate_transform_parent_child
from unloading_perception.planning_geometry import envelope_to_obb
from unloading_sim.geometry import OBB


def evaluate(derived_path, output_path):
    derived_path, output_path = Path(derived_path), Path(output_path)
    derived_ref = file_reference(derived_path)
    derived = json.loads(derived_path.read_text(encoding='utf-8'))
    artifact = derived['source_artifact']
    original, _ = load_algorithm_artifact(artifact['path'], artifact['sha256'])
    if original.source_epoch != derived['source_epoch']:
        raise ValueError('derived source epoch mismatch')
    root = Path(artifact['path']).parent
    references, modules = {}, {}
    for module in original.coverage['received_modules']:
        files = {}
        for name in ('gt_annotations.json', 'capture_binding.json', 'mask_evaluation.json'):
            path = root / module / name
            references[f'{module}/{name}'] = file_reference(path)
            files[name] = json.loads(path.read_text(encoding='utf-8'))
        binding, gt = files['capture_binding.json'], files['gt_annotations.json']
        if references[f'{module}/gt_annotations.json']['sha256'] != binding['gt_snapshot_sha256']:
            raise ValueError('GT snapshot binding mismatch')
        if gt['simulation_epoch'] != original.source_epoch or gt['simulation_frame'] != original.source_sequence:
            raise ValueError('GT acquisition mismatch')
        modules[module] = (files['mask_evaluation.json'], {o['simulation_object_id']: o for o in gt['objects']})
    rows = []
    for item in derived['objects']:
        matches, records = [], []
        for module, source_id in item['source_members']:
            evaluation, truth = modules[module]
            sam_id = int(source_id.rsplit('/', 1)[1])
            prediction, = [p for p in evaluation['predictions'] if p['sam_id'] == sam_id]
            ids = prediction['significant_gt_objects']
            matches.append(ids)
            if len(ids) == 1:
                records.append(truth[ids[0]])
        row = {'source_instance_id': item['source_instance_id'], 'existing_mask_matches': matches,
               'actual_appearance_mesh_coverage': 'NOT_EVALUATED',
               'evaluation_reference': 'HISTORICAL_NOMINAL_CUBOID_ONLY'}
        if not records or any(len(m) != 1 for m in matches) or len({r['simulation_object_id'] for r in records}) != 1:
            row['nominal_coverage_status'] = 'UNRESOLVED_EVALUATION_ASSOCIATION'
            rows.append(row)
            continue
        truth = records[0]
        if any(r['T_W_object'] != truth['T_W_object'] or r['full_dimensions_m'] != truth['full_dimensions_m'] for r in records):
            raise ValueError('cross-module GT geometry mismatch')
        if any(not r.get('geometry_truth_status', '').startswith('NOMINAL_CUBOID_ONLY') for r in records):
            raise ValueError('this evaluator requires explicitly nominal-only annotations')
        row.update(evaluation_gt_object_id=truth['simulation_object_id'],
                   annotation_geometry_truth_status=truth['geometry_truth_status'])
        if item['planning_geometry'] is None:
            row['nominal_coverage_status'] = 'NO_DERIVED_ENVELOPE'
        else:
            t = np.asarray(validate_transform_parent_child(truth['T_W_object']))
            nominal = OBB(t[:3, 3], np.array(truth['full_dimensions_m']) / 2, t[:3, :3])
            envelope = envelope_to_obb(item['planning_geometry'])
            local = (nominal.corners() - envelope.center) @ envelope.rotation
            outside = np.max(np.abs(local) - envelope.half_extents, axis=0)
            row.update(nominal_coverage_status='COVERED' if np.max(outside) <= 1e-9 else 'NOT_COVERED',
                       max_outside_per_box_axis_m=outside.tolist(),
                       nominal_world_corners_m=nominal.corners().tolist())
        rows.append(row)
    if file_reference(derived_path) != derived_ref:
        raise ValueError('derived artifact changed during evaluation')
    result = {'schema_version': 'size_prior_nominal_evaluation_v1', 'derived_geometry': derived_ref,
              'evaluation_inputs': references, 'objects': rows,
              'policy': 'Existing unique mask matches; no best-candidate selection; no adapter calls or parameter updates',
              'actual_mesh_conservatism_proven': False, 'planning_admissible': False}
    with output_path.open('x', encoding='utf-8') as out:
        json.dump(result, out, indent=2, sort_keys=True, allow_nan=False)
        out.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--derived', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = evaluate(args.derived, args.output)
    print(json.dumps([(r['source_instance_id'], r['nominal_coverage_status']) for r in result['objects']]))


if __name__ == '__main__':
    main()
