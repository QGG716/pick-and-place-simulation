"""Serial, file-bound geometry exec boundary; never controls the SAM parent's pools."""
import argparse
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'packages/unloading_contracts/src')]
from metric_thread_policy import positive_int, policy
from geometry_runtime_config import add_arguments, validate_arguments, forwarded_arguments, effective_config


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def reference(path):
    path = Path(path).resolve()
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def checked(ref, root=None):
    path = Path(ref['path']).resolve()
    if root is not None and not path.is_relative_to(Path(root).resolve()):
        raise ValueError('GEOMETRY_RESULT_OUTSIDE_TASK')
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != ref['sha256']:
        raise ValueError('GEOMETRY_HASH_MISMATCH: '+str(path))
    return raw


def capture_identity(payload):
    m = payload.metadata
    return json.loads(json.dumps({'module_id': payload.camera['module_id'], 'capture_id': m.capture_id,
            'source_epoch': m.sensor_epoch, 'source_sequence': m.frame_sequence,
            'capture_time': m.capture_center_time, 'calibration_identity': m.calibration_identity,
            'K': list(payload.camera['K']), 'T_W_C_at_capture': m.T_W_C_at_capture}))


def child_environment():
    env = os.environ.copy()
    env.pop('PYTHONHOME', None)
    env['PYTHONPATH'] = os.pathsep.join(str(p) for p in
        (ROOT/'tools', ROOT/'src', ROOT/'packages/unloading_contracts/src'))
    return env


def run_owned_child(command, task, timeout, stop_requested=None):
    """Exec, monitor and reap only this owned process (and its POSIX group)."""
    started = time.perf_counter()
    child = None
    old_handler = None
    main_thread = threading.current_thread() is threading.main_thread()
    def terminate_handler(signum, frame):
        raise SystemExit(128 + signum)
    try:
        if main_thread:
            old_handler = signal.signal(signal.SIGTERM, terminate_handler)
        with (task/'stdout.log').open('xb') as out, (task/'stderr.log').open('xb') as err:
            child = subprocess.Popen(command, stdout=out, stderr=err, env=child_environment(),
                                     start_new_session=os.name == 'posix')
            launched = time.perf_counter()
            while child.poll() is None:
                if stop_requested is not None and stop_requested():
                    raise InterruptedError('GEOMETRY_WORKER_STOP_REQUESTED')
                if time.perf_counter() - started >= timeout:
                    raise TimeoutError('GEOMETRY_PROCESS_TIMEOUT')
                time.sleep(.05)
            if child.returncode:
                raise RuntimeError(f'GEOMETRY_CHILD_EXIT: {child.returncode}; stderr={task/"stderr.log"}')
            return child.pid, {'launch': launched-started, 'child_wait': time.perf_counter()-launched}
    finally:
        if child is not None:
            if child.poll() is None:
                if os.name == 'posix':
                    try: os.killpg(child.pid, signal.SIGKILL)
                    except ProcessLookupError: pass
                else:
                    # Windows venv launchers may own an interpreter descendant.
                    # Terminate this PID tree only, never processes by name.
                    subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
                    if child.poll() is None: child.kill()
            child.wait()
        if main_thread and old_handler is not None:
            signal.signal(signal.SIGTERM, old_handler)


def verify_thread_report(report, request, pid):
    from unloading_contracts import canonical_fingerprint
    from metric_thread_policy import verify_blas, verify_non_targets
    runtime = request['config']['metric_runtime_policy']
    if (report['status'] != 'COMPLETED' or report['pid'] != pid or
            report['task_id'] != request['task_id'] or report['policy'] != runtime or
            report['policy_identity'] != canonical_fingerprint(runtime)):
        raise ValueError('GEOMETRY_THREAD_IDENTITY_MISMATCH')
    for phase in ('before_geometry', 'after_geometry'):
        if report[phase]['pid'] != pid: raise ValueError('GEOMETRY_THREAD_PID_MISMATCH')
        verify_blas(report[phase]['blas'], runtime['requested_blas_threads'])
        verify_non_targets(report['before_policy'], report[phase])
    if report['before_policy']['pid'] != pid: raise ValueError('GEOMETRY_THREAD_PID_MISMATCH')
    if runtime['requested_blas_threads'] is None and (
            report['applied'] or report['before_policy']['blas'] != report['after_geometry']['blas']):
        raise ValueError('GEOMETRY_INHERITED_POLICY_CHANGED')


def accept_result(request, request_ref, pid):
    from unloading_contracts import canonical_fingerprint, loads, PerceptionObservation, ObservationStatus
    from unloading_perception.observed_faces import ObservedFace, ObservedFaceSet
    root = Path(request['output_directory'])
    task = Path(request['task_directory'])
    index = read(task/'completed.json')
    for key in ('task_id', 'run_id', 'capture'):
        if index[key] != request[key]: raise ValueError('GEOMETRY_RESULT_IDENTITY_MISMATCH: '+key)
    if index['status'] != 'COMPLETED' or index['pid'] != pid or index['request'] != request_ref:
        raise ValueError('GEOMETRY_RESULT_NOT_COMPLETED_FOR_REQUEST')
    checked(request_ref, task)
    expected_files = {'thread_report': task/'thread_report.json', 'face_sets': task/'face_sets.json',
                      'observation': task/'observation.json', 'geometry': root/'rgbd_cuboids.json'}
    if set(index['files']) != set(expected_files) or any(
            Path(index['files'][k]['path']).resolve() != p.resolve() for k, p in expected_files.items()):
        raise ValueError('GEOMETRY_RESULT_FILE_SET_MISMATCH')
    documents = {key: json.loads(checked(ref)) for key, ref in index['files'].items()}
    verify_thread_report(documents['thread_report'], request, pid)
    observation = loads(json.dumps(documents['observation']))
    identity = request['capture']
    if (not isinstance(observation, PerceptionObservation) or observation.synthetic or
            observation.status not in (ObservationStatus.COMPLETE, ObservationStatus.PARTIAL) or
            observation.config_identity != canonical_fingerprint(request['config']) or
            any(getattr(observation, key) != identity[key] for key in
                ('source_epoch', 'source_sequence', 'capture_time')) or
            canonical_fingerprint(observation.coverage['module_binding']) != canonical_fingerprint({k: identity[k] for k in
                ('module_id', 'capture_id', 'calibration_identity', 'T_W_C_at_capture')})):
        raise ValueError('GEOMETRY_OBSERVATION_IDENTITY_MISMATCH')
    sets = tuple(ObservedFaceSet(**{**item,
        'faces': tuple(ObservedFace(**face) for face in item['faces']),
        'shared_edges': tuple(item['shared_edges'])}) for item in documents['face_sets'])
    if len({s.source_instance_id for s in sets}) != len(sets) or any(
            s.module_id != identity['module_id'] or s.capture_id != identity['capture_id'] or
            s.capture_time != identity['capture_time'] or s.frame_id != 'world' for s in sets):
        raise ValueError('GEOMETRY_FACE_SET_IDENTITY_MISMATCH')
    records = documents['geometry']['instances']
    if [r['mask_id'] for r in records] != request['mask_ids']:
        raise ValueError('GEOMETRY_INSTANCE_ROSTER_MISMATCH')
    if len(sets) != len(records) or len(observation.cargo) != len(records):
        raise ValueError('GEOMETRY_INCOMPLETE_DOMAIN_RESULT')
    if {s.source_instance_id for s in sets} != {c.source_instance_id for c in observation.cargo}:
        raise ValueError('GEOMETRY_FACE_CARGO_IDENTITY_MISMATCH')
    if sorted(c.raw_result['instance_lineage']['mask_id'] for c in observation.cargo) != sorted(request['mask_ids']):
        raise ValueError('GEOMETRY_CARGO_ROSTER_MISMATCH')
    from run_workcell_perception_once import _algorithm_counts
    result = {'observation': observation, 'observed_face_sets': sets, **index['result_metadata'],
              'module_directory': root, 'geometry_process': index}
    _algorithm_counts(result, documents['geometry'], len(records))
    return result


def dispatch_geometry(*, backend='inline', blas_threads=None, geometry_python=None,
                      run_id, stop_requested=None, **kwargs):
    """Shared by the single-capture entry, resident video worker and offline check."""
    if backend == 'inline':
        if blas_threads is not None: raise ValueError('inline cannot set BLAS threads')
        from run_isaac_rgbd_geometry import _run_secondary_module
        return _run_secondary_module(**kwargs)
    if backend != 'subprocess': raise ValueError('unknown geometry backend')
    from unloading_contracts import canonical_fingerprint
    from run_workcell_perception_once import _write_json
    from unloading_perception.isaac_payload import require_capture_payload
    started = time.perf_counter()
    source = Path(kwargs['module_dir']).resolve()
    folder = Path(kwargs.get('output_directory') or source).resolve()
    payload = require_capture_payload(source, kwargs['manifest'], payload=kwargs['payload'], with_instance_masks=True)
    replay = folder != source
    if ((replay and folder.exists()) or (not replay and payload.directory == payload.source_directory)
            or (folder/'rgbd_cuboids.json').exists()):
        raise ValueError('GEOMETRY_REQUIRES_NEW_RUN_SNAPSHOT')
    config = kwargs['config']
    if config != effective_config(config, backend, blas_threads):
        raise ValueError('GEOMETRY_CONFIG_POLICY_MISMATCH')
    task = folder.parent/('.geometry-task-'+folder.name) if replay else folder/'geometry-task'
    task.mkdir(exist_ok=False)
    import numpy as np
    with np.load(kwargs['artifacts']['cargo_masks.npz']['path'], allow_pickle=False) as archive:
        ids = [int(i) for i in archive['mask_ids']]
    if len(set(ids)) != len(ids): raise ValueError('duplicate mask IDs')
    response = read(source/'mode_b1_worker_response.json')
    names = [*payload.raw_files, 'oracle_proposals.json', 'mode_b1_worker_response.json']
    if not replay: names.append('input_provenance.json')
    inputs = {name: reference(source/name) for name in names}
    inputs['sam_metrics'] = response['metrics_reference']
    for name, ref in kwargs['artifacts'].items(): inputs['sam/'+name] = ref
    request = {'schema_version': 'workcell_geometry_v1', 'task_id': uuid.uuid4().hex, 'run_id': run_id,
        'capture': capture_identity(payload), 'manifest': kwargs['manifest'].to_dict(),
        'inputs': inputs, 'artifacts': kwargs['artifacts'], 'mask_ids': ids,
        'source_directory': str(payload.source_directory), 'input_directory': str(source),
        'output_directory': str(folder), 'task_directory': str(task),
        'scene': kwargs['scene'], 'vision_root': str(kwargs['vision_root']),
        'timeout': kwargs['timeout'], 'config': config, 'config_identity': canonical_fingerprint(config),
        'stage_trace': bool(kwargs.get('stage_trace', False))}
    path = task/'request.json'
    _write_json(path, request)
    request_ref = reference(path)
    # absolute() preserves a venv's Python symlink, unlike resolve().
    executable = str(Path(geometry_python or sys.executable).absolute())
    try:
        pid, timings = run_owned_child([executable, str(Path(__file__).resolve()),
            '--request', str(path), '--request-sha256', request_ref['sha256']], task, kwargs['timeout'], stop_requested)
        accepting = time.perf_counter()
        result = accept_result(request, request_ref, pid)
        timings.update(parent_accept=time.perf_counter()-accepting, dispatch_total=time.perf_counter()-started)
        result['geometry_process_timing_seconds'] = timings
        _write_json(task/'parent_acceptance.json', {'status': 'ACCEPTED', 'pid': pid,
            'request': request_ref, 'timing_seconds': timings})
        return result
    except BaseException as exc:
        _write_json(task/'parent_failure.json', {'status': 'TECHNICAL_FAILURE', 'request': request_ref,
            'error_type': type(exc).__name__, 'error': str(exc)})
        raise


def child_main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--request-sha256', required=True)
    args = parser.parse_args(argv)
    request_ref = {'path': str(args.request.resolve()), 'sha256': args.request_sha256}
    request = json.loads(checked(request_ref))
    folder = Path(request['output_directory']).resolve()
    task = Path(request['task_directory']).resolve()
    source = Path(request['input_directory']).resolve()
    replay = source != folder
    expected_task = folder.parent/('.geometry-task-'+folder.name) if replay else folder/'geometry-task'
    if task != expected_task or args.request.resolve() != task/'request.json':
        raise ValueError('GEOMETRY_REQUEST_OUTSIDE_TASK')
    from run_workcell_perception_once import _write_json
    stage = 'input_validation'
    started = time.perf_counter()
    try:
        from unloading_contracts import canonical_fingerprint, dumps
        from unloading_perception.isaac_validation import IsaacSceneManifest
        from unloading_perception.isaac_payload import load_capture_payload
        from run_isaac_rgbd_geometry import _worker_artifacts, _run_secondary_module
        from metric_thread_policy import snapshot, configure_blas
        if request['schema_version'] != 'workcell_geometry_v1' or not request['run_id'] or not request['task_id']:
            raise ValueError('GEOMETRY_REQUEST_INVALID')
        if request['config_identity'] != canonical_fingerprint(request['config']):
            raise ValueError('GEOMETRY_CONFIG_IDENTITY_MISMATCH')
        for ref in request['inputs'].values(): checked(ref)
        manifest = IsaacSceneManifest.from_dict(request['manifest'])
        payload = load_capture_payload(source, manifest, with_instance_masks=True,
                                       expected_module_id=request['capture']['module_id'])
        for name, raw in payload.raw_files.items():
            ref = request['inputs'][name]
            if Path(ref['path']).resolve() != source/name or hashlib.sha256(raw).hexdigest() != ref['sha256']:
                raise ValueError('GEOMETRY_CONSUMED_INPUT_MISMATCH: '+name)
        # Restore provenance of the parent's immutable snapshot after reloading
        # its actual bytes. Geometry writes only into this new run snapshot.
        payload = replace(payload, source_directory=Path(request['source_directory']))
        if json.loads(json.dumps(capture_identity(payload))) != request['capture']:
            raise ValueError('GEOMETRY_CAPTURE_IDENTITY_MISMATCH')
        if not replay and payload.input_provenance() != read(source/'input_provenance.json'):
            raise ValueError('GEOMETRY_SNAPSHOT_PROVENANCE_MISMATCH')
        artifacts = _worker_artifacts(source, payload=payload)
        if artifacts != request['artifacts']: raise ValueError('GEOMETRY_SAM_IDENTITY_MISMATCH')
        if ((replay and (folder.exists() or payload.directory != payload.source_directory)) or
                (not replay and payload.directory == payload.source_directory) or (folder/'rgbd_cuboids.json').exists()):
            raise ValueError('GEOMETRY_OUTPUT_NOT_NEW')
        verified = time.perf_counter()
        stage = 'thread_policy'
        runtime = request['config']['metric_runtime_policy']
        if request['config'] != effective_config(request['config'], 'subprocess', runtime['requested_blas_threads']):
            raise ValueError('GEOMETRY_CONFIG_POLICY_MISMATCH')
        report = {'status': 'COMPLETED', 'pid': os.getpid(), 'task_id': request['task_id'],
                  'policy': runtime, 'policy_identity': canonical_fingerprint(runtime), 'before_policy': snapshot()}
        report['applied'] = configure_blas(report['before_policy']['blas'], runtime['requested_blas_threads'])
        report['before_geometry'] = snapshot()
        report['after_geometry'] = report['before_geometry']
        verify_thread_report(report, request, os.getpid())
        stage = 'geometry'
        computing = time.perf_counter()
        result = _run_secondary_module(scene=request['scene'], module_dir=source, manifest=manifest,
            artifacts=artifacts, config=request['config'], vision_root=Path(request['vision_root']),
            upstream_python=Path(sys.executable), timeout=request['timeout'], payload=payload,
            output_directory=folder if replay else None,
            **({'stage_trace': True} if request.get('stage_trace', False) else {}))
        computed = time.perf_counter()
        stage = 'result_publication'
        report['after_geometry'] = snapshot()
        verify_thread_report(report, request, os.getpid())
        _write_json(task/'thread_report.json', report)
        _write_json(task/'face_sets.json', [s.to_dict() for s in result['observed_face_sets']])
        _write_json(task/'observation.json', json.loads(dumps(result['observation'])))
        files = {key: reference(path) for key, path in {
            'thread_report': task/'thread_report.json', 'face_sets': task/'face_sets.json',
            'observation': task/'observation.json', 'geometry': folder/'rgbd_cuboids.json'}.items()}
        index = {'status': 'COMPLETED', 'task_id': request['task_id'], 'run_id': request['run_id'],
            'capture': request['capture'], 'pid': os.getpid(), 'request': request_ref, 'files': files,
            'result_metadata': {k: result[k] for k in ('elapsed_seconds', 'timing_seconds', 'legacy_cuboid_diagnostic')},
            'timing_seconds': {'input_validation': verified-started, 'thread_initialization': computing-verified,
                               'geometry': computed-computing, 'child_before_index': time.perf_counter()-started}}
        _write_json(task/'completed.json', index)  # last, atomic; exit must also succeed
        return 0
    except BaseException as exc:
        _write_json(task/'child_failure.json', {'status': 'TECHNICAL_FAILURE',
            'stage': stage, 'error_type': type(exc).__name__, 'error': str(exc), 'pid': os.getpid()})
        raise


if __name__ == '__main__': raise SystemExit(child_main())
