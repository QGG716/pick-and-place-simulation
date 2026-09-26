"""Strict fixed-roster comparison of metric geometry, observations and fusion.

Whitelist: run-directory prefixes; observation processed_time/config_identity;
algorithm index run_id/diagnostic flag/config_identity and verified reference
hashes whose complete documents are compared separately. No numeric tolerance.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'packages/unloading_contracts/src'), str(ROOT/'tools')]
from unloading_contracts import canonical_fingerprint, loads, to_wire
from unloading_perception.algorithm_artifact import load_algorithm_artifact
from compare_depth_preprocessing import require_roster, sha


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def paths(value, root):
    if isinstance(value, dict): return {k: paths(v, root) for k, v in value.items()}
    if isinstance(value, list): return [paths(v, root) for v in value]
    if isinstance(value, str):
        for prefix in (str(root) + '/', str(root) + '\\', root.as_posix() + '/'):
            if value.startswith(prefix):
                return '<RUN>/' + value[len(prefix):]
    if isinstance(value, str) and value.startswith(root.as_uri() + '/'):
        return '<RUN_URI>/' + value[len(root.as_uri())+1:]
    return value


def index_identity(value, *, thread_policy=False):
    value = deepcopy(value)
    value['run_id'] = '<RUN_ID>'
    value['config_identity'] = '<VERIFIED_CONFIG_IDENTITY>'
    value['config']['vision']['legacy_cuboid_diagnostic'] = '<DIAGNOSTIC_FLAG>'
    if thread_policy:
        value['config']['metric_runtime_policy']['requested_blas_threads'] = '<VERIFIED_THREAD_POLICY>'
        value['config']['metric_runtime_policy']['mechanism'] = '<VERIFIED_THREAD_MECHANISM>'
    for ref in value['module_observations'].values(): ref['sha256'] = '<VERIFIED_AND_COMPARED_DOCUMENT>'
    value['fusion_result']['sha256'] = '<VERIFIED_AND_COMPARED_DOCUMENT>'
    if 'observation' in value: value['observation']['sha256'] = '<VERIFIED_AND_COMPARED_DOCUMENT>'
    return value


def observation_identity(value, *, thread_policy=False):
    value = deepcopy(value)
    value['processed_time'] = '<PROCESSING_TIME>'
    value['config_identity'] = '<VERIFIED_CONFIG_IDENTITY>'
    if 'algorithm_run' in value['coverage']:
        value['coverage']['algorithm_run'] = index_identity(value['coverage']['algorithm_run'], thread_policy=thread_policy)
    return value


def differences(a, b, path, output):
    if len(output) >= 40: return
    if type(a) != type(b):
        output.append(path + ': type differs'); return
    if isinstance(a, dict):
        if a.keys() != b.keys():
            output.append(path + ': keys differ'); return
        for k in a: differences(a[k], b[k], path+'/'+str(k), output)
    elif isinstance(a, list):
        if len(a) != len(b):
            output.append(path + f': lengths {len(a)} != {len(b)}'); return
        for i in range(len(a)): differences(a[i], b[i], path+'/'+str(i), output)
    elif a != b:
        output.append(path + f': {str(a)[:100]} != {str(b)[:100]}')


def compare(before, after, plan_path, *, mode='diagnostic-toggle', code_change=None, difference_collector=None):
    # Read-only diagnostic callers may count every difference. CLI semantics and
    # its exact-equality requirement remain unchanged.
    def collect(a, b, path, output):
        differences(a, b, path, output)  # strict verdict is never delegated
        if difference_collector is not None:
            difference_collector(a, b, path, [])
    if mode not in ('diagnostic-toggle', 'hotspot-off', 'blas-threads'):
        raise ValueError('unknown comparison mode')
    before, after = Path(before).resolve(), Path(after).resolve()
    diagnostic_flags = (True, False) if mode == 'diagnostic-toggle' else (False, False)
    plan = read(plan_path)
    thread_mode = mode == 'blas-threads'
    thread_reports = []
    if thread_mode and plan.get('thread_policies') != [None, 1]:
        raise ValueError('thread comparison requires frozen inherit/1 policies')
    modules = [r['module_id'] for r in plan['modules']]
    require_roster(plan['modules'], 'module_id', modules, 'fixed plan')
    result = {'plan_complete': True, 'results_equal': False, 'differences': [], 'modules': [],
              'comparison_policy': __doc__, 'mode': mode, 'plan_sha256': sha(Path(plan_path))}
    indexes, observations, summaries = [], [], []
    for ordinal, (root, enabled) in enumerate(zip((before, after), diagnostic_flags)):
        if thread_mode:
            from metric_thread_policy import policy, verify_blas, verify_non_targets
            runtime = policy((None, 1)[ordinal])
            report = read(root/'thread_policy_report.json')
            if report['status'] != 'COMPLETED' or report['policy'] != runtime:
                raise ValueError(f'{root.name}: THREAD_POLICY_NOT_VERIFIED')
            if report['policy_identity'] != canonical_fingerprint(runtime) or report['plan_sha256'] != result['plan_sha256']:
                raise ValueError(f'{root.name}: thread policy identity mismatch')
            for phase in ('before_geometry', 'after_geometry'):
                verify_blas(report[phase]['blas'], runtime['requested_blas_threads'])
                verify_non_targets(report['before_policy'], report[phase])
                if report[phase]['pid'] != report['pid']:
                    raise ValueError(f'{root.name}: thread evidence from another process')
            if ordinal == 0 and (report['applied'] or report['before_policy']['blas'] != report['after_geometry']['blas']):
                raise ValueError('inherited thread policy was changed')
            thread_reports.append(report)
        summary = read(root/'ab_summary.json')
        if summary['status'] != 'COMPLETED' or summary['plan_sha256'] != result['plan_sha256']:
            raise ValueError(f'{root.name}: run failed or plan identity differs')
        require_roster(summary['modules'], 'module_id', modules, root.name+'/modules')
        ref = summary['artifact']
        if Path(ref['path']).resolve() != root/'algorithm_artifact.json':
            raise ValueError(f'{root.name}: artifact belongs to another run')
        observation, index = load_algorithm_artifact(ref['path'], ref['sha256'])
        expected = deepcopy(plan['config']); expected['vision']['legacy_cuboid_diagnostic'] = enabled
        if thread_mode:
            expected['metric_runtime_policy'] = runtime
            if report['artifact'] != ref: raise ValueError('thread report artifact identity mismatch')
        if index['config'] != expected or index['config_identity'] != canonical_fingerprint(expected):
            raise ValueError(f'{root.name}: effective config mismatch')
        if index['run_id'] != root.name or observation.config_identity != canonical_fingerprint(expected):
            raise ValueError(f'{root.name}: observation/run config identity mismatch')
        if index['model_manifest'] != plan['model_manifest']:
            raise ValueError(f'{root.name}: model identity mismatch')
        indexes.append(index); observations.append(to_wire(observation)); summaries.append(summary)
    if summaries[0]['code_sha256'] != summaries[1]['code_sha256']:
        raise ValueError('A/B production code differs')
    if mode in ('hotspot-off', 'blas-threads'):
        timed = [read(r/'function_timings.json') for r in (before, after)]
        for root, measurement in zip((before, after), timed):
            if measurement['heavy_profiler_enabled'] is not False or measurement['status'] != 'COMPLETED':
                raise ValueError(f'{root.name}: performance run failed or heavy profiler enabled')
            actual = [(r['module_id'], r['mask_id']) for r in measurement['instances']]
            expected = [(m['module_id'], i) for m in plan['modules'] for i in m['mask_ids']]
            require_roster([{'id': i} for i in actual], 'id', expected, root.name+'/timings')
        a, b = (m['production_sha256'] for m in timed)
        if mode == 'hotspot-off':
            change = read(code_change)
            if a.keys() != b.keys() or [p for p in a if a[p] != b[p]] != [change['path']]:
                raise ValueError('expected exactly the declared production hotspot file change')
            if a[change['path']] != change['before_sha256'] or b[change['path']] != change['after_sha256']:
                raise ValueError('declared hotspot code identity mismatch')
        else:
            if not a or a != b or thread_reports[0]['entry_code_sha256'] != thread_reports[1]['entry_code_sha256']:
                raise ValueError('thread comparison requires identical production and entry code')
            for key in ('blas', 'opencv_threads', 'thread_environment', 'affinity', 'python', 'numpy', 'scipy', 'opencv'):
                if thread_reports[0]['before_policy'][key] != thread_reports[1]['before_policy'][key]:
                    raise ValueError('different inherited configuration: '+key)
        for key in ('python', 'executable', 'platform', 'numpy', 'scipy', 'opencv', 'affinity',
                    'thread_environment', 'threadpools', '/sys/fs/cgroup/cpu.max'):
            if thread_mode and key == 'threadpools': continue  # native pools verified above
            if timed[0]['environment_before'].get(key) != timed[1]['environment_before'].get(key):
                raise ValueError(f'performance environment mismatch: {key}')
    for spec in plan['modules']:
        name = spec['module_id']
        geometry, obs = [], []
        counts = []
        for root, enabled, summary in zip((before, after), diagnostic_flags, summaries):
            row = next(m for m in summary['modules'] if m['module_id'] == name)
            require_roster([{'id': i} for i in row['mask_ids']], 'id', spec['mask_ids'], root.name+'/'+name+'/summary')
            diag = read(root/name/'legacy_cuboid_diagnostic.json')
            if diag['enabled'] != enabled or diag['status'] != ('COMPLETED' if enabled else 'DISABLED'):
                raise ValueError(f'{root.name}/{name}: diagnostic status failure')
            for filename in ('rgbd_cuboids_baseline_raw.json', 'rgbd_cuboids_baseline.png'):
                if (root/name/filename).exists() != enabled:
                    raise ValueError(f'{root.name}/{name}: unexpected diagnostic file {filename}')
            final = read(root/name/'rgbd_cuboids.json')
            require_roster(final['instances'], 'mask_id', spec['mask_ids'], root.name+'/'+name+'/final')
            module_obs = loads((root/name/'mode_b_rgbd_observation.json').read_text(encoding='utf-8'))
            require_roster([{'mask_id': c.raw_result['instance_lineage']['mask_id']} for c in module_obs.cargo],
                'mask_id', spec['mask_ids'], root.name+'/'+name+'/observation')
            if module_obs.config_identity != canonical_fingerprint(summary['effective_config']):
                raise ValueError(f'{root.name}/{name}: module config identity mismatch')
            if module_obs.processed_time != module_obs.capture_time + row['timing_seconds']['legacy_geometry']:
                raise ValueError(f'{root.name}/{name}: processing interval mismatch')
            geometry.append(paths(final, root))
            obs.append(paths(observation_identity(to_wire(module_obs), thread_policy=thread_mode), root))
            counts.append({'instances': len(final['instances']), 'faces': sum(len(r['camera_facing_faces']) for r in final['instances']),
                'with_faces': sum(bool(r['camera_facing_faces']) for r in final['instances']),
                'without_faces': sum(not r['camera_facing_faces'] for r in final['instances']),
                'accepted_complete_cuboids': sum(bool(r['accepted']) for r in final['instances'])})
        collect(*geometry, name+'/final', result['differences'])
        collect(*obs, name+'/observation', result['differences'])
        result['modules'].append({'module_id': name, 'before': counts[0], 'after': counts[1],
            'instances': [{'mask_id': r['mask_id'], 'faces': len(r['camera_facing_faces']), 'accepted_complete_cuboid': r['accepted'],
                'final_record_exact_equal': r == s} for r, s in zip(geometry[0]['instances'], geometry[1]['instances'])]})
    for title, pair in (
        ('index', [paths(index_identity(i, thread_policy=thread_mode), r) for i, r in zip(indexes, (before, after))]),
        ('fusion', [paths(read(r/'fusion_result.json'), r) for r in (before, after)]),
        ('fused_observation', [paths(observation_identity(o, thread_policy=thread_mode), r) for o, r in zip(observations, (before, after))])):
        collect(*pair, title, result['differences'])
    result['results_equal'] = not result['differences']
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('before', 'after', 'plan', 'output'): parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--mode', choices=('diagnostic-toggle', 'hotspot-off', 'blas-threads'), default='diagnostic-toggle')
    parser.add_argument('--code-change', type=Path, help='required for hotspot-off; one explicit old/new source SHA')
    args = parser.parse_args(argv)
    try:
        result = compare(args.before, args.after, args.plan, mode=args.mode, code_change=args.code_change)
    except Exception as exc:
        result = {'plan_complete': False, 'results_equal': False, 'error_type': type(exc).__name__, 'error': str(exc)}
    with args.output.open('x', encoding='utf-8') as stream: json.dump(result, stream, indent=2, sort_keys=True)
    print(json.dumps(result), flush=True)
    return 0 if result['results_equal'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
