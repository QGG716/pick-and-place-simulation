"""Standalone fixed-input metric geometry with an explicit process-local BLAS budget.

No option means inherit. Only this entry controls threads; resident/model workers
are outside its scope. Delegates all geometry, timing and delivery to existing code.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter, process_time

from metric_thread_policy import positive_int, policy, snapshot, configure_blas, verify_blas, verify_non_targets


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--blas-threads', type=positive_int)
    args = parser.parse_args(argv)  # stdlib only: invalid values rejected before numeric imports
    if args.output.exists(): parser.error('output already exists; use a new directory')
    started, cpu = perf_counter(), process_time()
    runtime = policy(args.blas_threads)
    report = {'status': 'THREAD_POLICY_NOT_VERIFIED', 'policy': runtime, 'pid': __import__('os').getpid(),
              'plan_sha256': hashlib.sha256(args.plan.read_bytes()).hexdigest()}
    exit_code = 2
    try:
        report['before_policy'] = snapshot()
        report['applied'] = configure_blas(report['before_policy']['blas'], args.blas_threads)
        report['before_geometry'] = snapshot()
        verify_blas(report['before_geometry']['blas'], args.blas_threads)
        verify_non_targets(report['before_policy'], report['before_geometry'])
        import profile_metric_faces
        from unloading_contracts import canonical_fingerprint
        report['policy_identity'] = canonical_fingerprint(runtime)
        report['entry_code_sha256'] = {n: hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest()
            for n in ('run_metric_geometry_blas.py', 'metric_thread_policy.py', 'profile_metric_faces.py', 'validate_metric_faces_ab.py')}
        report['status'] = 'RUNNING'
        exit_code = profile_metric_faces.main(['--plan', str(args.plan), '--output', str(args.output)], runtime_policy=runtime)
        report['after_geometry'] = snapshot()
        verify_blas(report['after_geometry']['blas'], args.blas_threads)
        verify_non_targets(report['before_policy'], report['after_geometry'])
        if args.blas_threads is None and report['before_geometry']['blas'] != report['after_geometry']['blas']:
            raise ValueError('THREAD_POLICY_NOT_VERIFIED: inherited BLAS changed during geometry')
        summary = json.loads((args.output/'ab_summary.json').read_text())
        report.update(status='COMPLETED' if exit_code == 0 else 'GEOMETRY_FAILED',
            artifact=summary.get('artifact'), geometry_exit_code=exit_code,
            algorithm_process_cpu_seconds=report['after_geometry']['process_cpu_seconds']-report['before_geometry']['process_cpu_seconds'],
            algorithm_scope_wall_seconds=report['after_geometry']['monotonic_seconds']-report['before_geometry']['monotonic_seconds'])
    except Exception as exc:
        report.update(status='THREAD_POLICY_NOT_VERIFIED' if report['status'] != 'RUNNING' or 'THREAD_' in str(exc)
                      else 'RUN_FAILED', error_type=type(exc).__name__, error=str(exc))
        exit_code = 2
    finally:
        report.update(entry_wall_seconds=perf_counter()-started, entry_process_cpu_seconds=process_time()-cpu)
        args.output.mkdir(parents=True, exist_ok=True)
        with (args.output/'thread_policy_report.json').open('x', encoding='utf-8') as out:
            json.dump(report, out, indent=2, sort_keys=True, allow_nan=False)
        print(json.dumps({'thread_policy_status': report['status'], 'exit_code': exit_code}), flush=True)
    return exit_code


if __name__ == '__main__': raise SystemExit(main())
