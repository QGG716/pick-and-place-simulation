from copy import deepcopy
from dataclasses import replace
import json

import pytest

from unloading_contracts import canonical_fingerprint
from unloading_perception.algorithm_artifact import publish_algorithm_run
from test_fusion_face_reduction import batch_fixture, module_observations
from tools.compare_metric_faces_ab import compare, main
from tools.compare_depth_preprocessing import sha


def write(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


def fixture(tmp_path):
    batches = batch_fixture(False)
    observations = module_observations(batches)
    specs = [{'module_id': b.module_id, 'mask_ids': list(range(1, len(b.face_sets)+1))} for b in batches]
    plan = {'modules': specs, 'config': {'vision': {'legacy_cuboid_diagnostic': True}}, 'model_manifest': {'test': 'SYNTHETIC'}}
    plan_path = tmp_path/'plan.json'; write(plan_path, plan)
    roots = [tmp_path/'on', tmp_path/'off']
    for root, enabled in zip(roots, (True, False)):
        root.mkdir(); config = deepcopy(plan['config']); config['vision']['legacy_cuboid_diagnostic'] = enabled
        results, rows = {}, []
        for batch, original in zip(batches, observations):
            folder = root/batch.module_id; folder.mkdir()
            cargo = tuple(replace(c, raw_result={'instance_lineage': {'mask_id': i}}) for i, c in enumerate(original.cargo, 1))
            obs = replace(original, cargo=cargo, synthetic=False, config_identity=canonical_fingerprint(config),
                processed_time=original.capture_time+(2 if enabled else 1), coverage={**original.coverage,
                'raw_image_automatic': False, 'proposal_source': 'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL'})
            records = [{'mask_id': i, 'camera_facing_faces': [f.to_dict() for f in fs.faces], 'accepted': False}
                       for i, fs in enumerate(batch.face_sets, 1)]
            write(folder/'rgbd_cuboids.json', {'instances': records})
            write(folder/'legacy_cuboid_diagnostic.json', {'enabled': enabled, 'status': 'COMPLETED' if enabled else 'DISABLED'})
            if enabled:
                for name in ('rgbd_cuboids_baseline_raw.json', 'rgbd_cuboids_baseline.png'): (folder/name).write_bytes(b'SYNTHETIC')
            results[batch.module_id] = {'observation': obs, 'observed_face_sets': batch.face_sets}
            rows.append({'module_id': batch.module_id, 'mask_ids': list(range(1, len(cargo)+1)), 'timing_seconds': {'legacy_geometry': 2 if enabled else 1}})
        ref = publish_algorithm_run(root, results, expected_modules=[b.module_id for b in batches], run_id=root.name,
                                    models=plan['model_manifest'], config=config, write_json=write)
        write(root/'ab_summary.json', {'status': 'COMPLETED', 'plan_sha256': sha(plan_path), 'modules': rows,
              'artifact': ref, 'effective_config': config, 'code_sha256': {'fixture': 'same'}})
    return plan_path, *roots


def test_complete_exact_comparison_uses_real_artifact_validation(tmp_path):
    plan, a, b = fixture(tmp_path)
    result = compare(a, b, plan)
    assert result['plan_complete'] and result['results_equal']
    assert all(m['after']['accepted_complete_cuboids'] == 0 for m in result['modules'])


@pytest.mark.parametrize('failure', ['module', 'instance', 'duplicate', 'coordinate', 'missing_file', 'artifact_hash'])
def test_geometry_comparison_requires_complete_plan_and_exact_values(tmp_path, failure):
    plan, a, b = fixture(tmp_path)
    if failure == 'module':
        path=b/'ab_summary.json'; v=json.loads(path.read_text()); v['modules'].pop(); write(path,v)
    elif failure == 'missing_file':
        (b/'m0/rgbd_cuboids.json').unlink()
    elif failure == 'artifact_hash':
        (b/'fusion_result.json').write_text('{}')
    else:
        path=b/'m0/rgbd_cuboids.json'; v=json.loads(path.read_text())
        if failure == 'instance': v['instances'].pop()
        elif failure == 'duplicate': v['instances'][-1]['mask_id']=v['instances'][0]['mask_id']
        else: v['instances'][0]['camera_facing_faces'][0]['corners_3d_m'][0][0] += .00000001
        write(path,v)
    out=tmp_path/'comparison.json'
    assert main(['--before',str(a),'--after',str(b),'--plan',str(plan),'--output',str(out)]) == 1
    result=json.loads(out.read_text())
    assert not result['results_equal'] and (result.get('error') or result.get('differences'))
