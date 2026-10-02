from pathlib import Path
import argparse
import json

parser = argparse.ArgumentParser(description='Compare the two completed offline benchmark files; never plans or executes.')
parser.add_argument('directory', nargs='?', type=Path, default=Path(__file__).resolve().parent/'validation-opt')
p = parser.parse_args().directory
r, o = [json.loads((p / name).read_text()) for name in ('final-reference.json', 'final-optimized.json')]
assert r['input_files'] == o['input_files']
assert r['sampled_prefix_edges'] == o['sampled_prefix_edges'] == 0
assert not r['profile_enabled'] and not o['profile_enabled']
rows = []
for a, b in zip(r['stages'], o['stages']):
    assert a['stage'] == b['stage']
    assert a['result'] == b['result'] == 'ACCEPT'
    for key in ('stage_id', 'task_id', 'parent_stage_id', 'request_fingerprint', 'path_sha256', 'scene_fingerprint', 'tracker_initial', 'tracker_initial_pose'):
        assert a['identity'][key] == b['identity'][key], key
    assert a['tracker_final'] == b['tracker_final']
    for key in ('state_visits', 'unique_state_visits', 'state_samples', 'edge_calls', 'subdivisions', 'certified_intervals', 'pair_certificates'):
        assert a['profile']['motion'][key] == b['profile']['motion'][key], key
    row = {'stage': a['stage'], 'path_edges': a['path_edges'], 'reference_seconds': a['wall_seconds'],
           'optimized_seconds': b['wall_seconds'], 'speedup': a['wall_seconds']/b['wall_seconds'],
           'reduction_percent': 100*(1-b['wall_seconds']/a['wall_seconds'])}
    for mode, x in [('reference', a), ('optimized', b)]:
        row[mode] = {k: x[k] for k in ('model_preparation_seconds', 'cache_state', 'result', 'failure', 'profile')}
    rows.append(row)
for report in (r, o):
    assert report['counterexample']['observed']['classification'] == 'CLEARANCE_INSUFFICIENT'
assert r['counterexample']['observed']['pair'] == o['counterexample']['observed']['pair']
assert r['counterexample']['observed']['surface_distance_m'] == o['counterexample']['observed']['surface_distance_m']
out = {'scope': 'SERIAL_SAME_INPUT_COLD_VALIDATOR_COMPARISON_NOT_NATIVE_COLD_PLANNING',
       'reference_source': '0efe9cffd81be03c393b02107bb2b60d37b5f6a7',
       'optimized_source': '58eabd9b872dbb4ba6c1b7849d3298a8cbe971fc',
       'source_difference': 'Nonfinite AABB-bound guard and test only; helper is unused in reference mode.',
       'input_files': r['input_files'], 'stages': rows, 'same_grid_and_ordered_tracker': True,
       'counterexample': o['counterexample']['observed'], 'warm_result_cache_benchmark': 'NOT_MEASURED',
       'reference_total_seconds': sum(x['wall_seconds'] for x in r['stages']),
       'optimized_total_seconds': sum(x['wall_seconds'] for x in o['stages'])}
(p/'comparison.json').write_text(json.dumps(out, indent=2, allow_nan=False)+'\n')
print(json.dumps([{k:v for k,v in row.items() if k not in ('reference', 'optimized')} for row in rows], indent=2))
