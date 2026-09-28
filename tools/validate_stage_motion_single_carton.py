"""One observable normal plan/export/preflight/readback attempt from fixed source.

History supplies target identity and actual scene only, never a path answer.
No retries, budget resets or Isaac execution.
"""
import argparse
import hashlib
import json
import signal
from pathlib import Path
from time import perf_counter
from unloading_sim.layout_single_carton import motion_implementation_identity
from unloading_sim.planning_profile import DEFAULT_MOTION, DEFAULT_EXECUTION


class OfflineResourceLimit(BaseException):
    """External experiment stop, not a production deadline or infeasibility."""


def identity(root):
    return dict(implementation=motion_implementation_identity(root),
        entry_files={str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (root/'tools/validate_stage_motion_single_carton.py',
                      root/'tools/run_m710_contact_unloading.py')})


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, default=str), encoding='utf-8')
    temporary.replace(path)


def main(args):
    import sys
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    from tools.run_m710_contact_unloading import main as normal_entry
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError('use a fresh evidence path')
    delivery = output.parent/(output.stem+'_delivery')
    motion_path = args.history/'planning/motion.json'
    state_path = args.history/'inputs/actual_remaining_state.json'
    target = json.loads(motion_path.read_text(encoding='utf-8'))['selected_trajectory_segment']['target']
    inputs = (motion_path, state_path, root/DEFAULT_MOTION, root/DEFAULT_EXECUTION)
    fingerprints = lambda: {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    started = perf_counter()
    document = dict(scope='ONE_NORMAL_NEW_SINGLE_CARTON_PLANNING_ATTEMPT',
        status='RUNNING', source=identity(root), input_sha256=fingerprints(), target=target,
        source_snapshot=str(root), historical_path_supplied=False,
        historical_path_revalidation=False, archived_contact_seed_supplied=False,
        planning_entry='tools.run_m710_contact_unloading.main',
        progress_path=str(delivery/'planning_progress.jsonl'),
        offline_resource_limit_seconds=args.offline_resource_seconds,
        resource_limit_scope='EXTERNAL_EXPERIMENT_ONLY_NOT_PRODUCTION_BUDGET',
        checkpoint_interval_seconds=5, isaac_executed=False,
        complete_planning=False, independent_preflight_run=False, bundle_readback=False)
    atomic_json(output, document)
    def source_guard():
        if document['source'] != identity(root) or document['input_sha256'] != fingerprints():
            raise RuntimeError('SOURCE_OR_INPUT_CHANGED_NO_PUBLICATION')
    alarm_handler = None
    if args.offline_resource_seconds is not None:
        if args.offline_resource_seconds <= 0:
            raise ValueError('offline resource limit must be positive')
        if not hasattr(signal, 'setitimer'):
            raise RuntimeError('external experiment alarm requires POSIX')
        def stop_experiment(signum, frame):
            raise OfflineResourceLimit('offline experiment resource limit; feasibility unknown')
        alarm_handler = signal.signal(signal.SIGALRM, stop_experiment)
        signal.setitimer(signal.ITIMER_REAL, args.offline_resource_seconds)
    try:
        code = normal_entry(['--config', DEFAULT_MOTION, '--execution-config', DEFAULT_EXECUTION,
            '--actual-state', str(state_path.resolve()), '--target-id', target,
            '--output', str(delivery)], source_guard=source_guard)
        document.update(status='NORMAL_EXIT', entry_exit_code=code)
    except OfflineResourceLimit as exc:
        document.update(status='OFFLINE_RESOURCE_LIMIT', validation_status='INDETERMINATE',
                        reason=str(exc), planning_infeasible=False)
    except KeyboardInterrupt as exc:
        document.update(status='MANUAL_CANCELLED', validation_status='CANCELLED',
                        reason=str(exc), planning_infeasible=False)
    except Exception as exc:
        document.update(status='ERROR', reason=str(exc), exception_type=type(exc).__name__)
        raise
    finally:
        if alarm_handler is not None:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, alarm_handler)
        document.update(elapsed_seconds=perf_counter()-started,
            source_unchanged_at_exit=document['source'] == identity(root),
            inputs_unchanged_at_exit=document['input_sha256'] == fingerprints())
        motion_file = delivery/'motion.json'
        if motion_file.exists():
            motion = json.loads(motion_file.read_text(encoding='utf-8'))
            document.update(motion_path=str(motion_file),
                complete_planning=motion.get('complete_trajectory_status') == 'PASS',
                motion_failure=motion.get('complete_trajectory_failure_reason'))
        receipt = delivery/'delivery.json'
        if receipt.exists():
            document['delivery'] = json.loads(receipt.read_text(encoding='utf-8'))
            document['bundle_readback'] = document['delivery'].get('bundle_readback', False)
        document['independent_preflight_run'] = (delivery/'preflight.json').exists()
        progress = delivery/'planning_progress.jsonl'
        if progress.exists():
            records = [json.loads(line) for line in progress.read_text(encoding='utf-8').splitlines()]
            document['last_progress'] = records[-1] if records else None
            document['last_known_stage'] = next((r for r in reversed(records)
                if r.get('event') in ('stage_enter', 'work_checkpoint', 'connection_started')), None)
            document['last_completed_stage'] = next((r for r in reversed(records)
                if r.get('event') == 'stage_exit'), None)
            document['location_is_last_observed_only'] = True
        atomic_json(output, document)
    print(json.dumps({k: document[k] for k in ('status', 'elapsed_seconds', 'complete_planning',
        'independent_preflight_run', 'source_unchanged_at_exit')}, indent=2), flush=True)
    return 0 if document['complete_planning'] and document['bundle_readback'] else 2


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--offline-resource-seconds', type=float)
    raise SystemExit(main(parser.parse_args()))
