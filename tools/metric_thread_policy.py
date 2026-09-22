"""Process-local OpenBLAS control and Linux resource evidence (stdlib only).

Only already loaded NumPy/SciPy wheel BLAS libraries are controlled. Unknown
backends/ownership fail verification; no environment or machine settings change.
"""
import ctypes
import os
from pathlib import Path
import platform
import sys
from time import process_time, perf_counter


THREAD_ENV = ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
              'GOTO_NUM_THREADS', 'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS')


def positive_int(value):
    import argparse
    try:
        result = int(value)
    except (ValueError, TypeError):
        raise argparse.ArgumentTypeError('BLAS threads must be a positive integer')
    if result < 1:
        raise argparse.ArgumentTypeError('BLAS threads must be a positive integer')
    return result


def policy(threads):
    return {'schema_version': 'metric_blas_policy_v1', 'requested_blas_threads': threads,
            'scope': 'standalone_metric_geometry_process', 'targets': ['numpy', 'scipy'],
            'mechanism': 'inherit' if threads is None else 'native_openblas_runtime_api',
            'non_target_thread_settings': 'unchanged'}


def read_optional(path):
    try:
        return {'value': Path(path).read_text().strip(), 'error': None}
    except OSError as exc:
        return {'value': None, 'error': str(exc)}


def cgroup_snapshot():
    membership = read_optional('/proc/self/cgroup')
    mounts = read_optional('/proc/self/mountinfo')
    result = {'membership': membership, 'levels': [],
              'scope': 'cgroup aggregate, may include other processes; unmounted ancestors unknown'}
    if membership['value'] is None or mounts['value'] is None:
        result['mapping_error'] = 'cgroup membership/mountinfo unavailable'; return result
    group = next((r[3:] for r in membership['value'].splitlines() if r.startswith('0::')), None)
    for row in mounts['value'].splitlines():
        left, _, right = row.partition(' - ')
        if not right.startswith('cgroup2 '): continue
        fields = left.split(); mount_root, mount = fields[3:5]
        result.update(current_path=group, mount_root=mount_root, mount_point=mount)
        if group is None or not (group == mount_root or group.startswith(mount_root.rstrip('/')+'/')):
            result['mapping_error'] = 'cgroup v2 path not mapped to accessible mount'; return result
        relative = group[len(mount_root):].lstrip('/')
        top = Path(mount).resolve(); current = (top/relative).resolve()
        if current != top and top not in current.parents:
            result['mapping_error'] = 'cgroup path outside mount'; return result
        while True:
            result['levels'].append({'path': str(current), **{name: read_optional(current/name)
                for name in ('cpu.max', 'cpu.stat', 'cpuset.cpus.effective')}})
            if current == top: break
            current = current.parent
        return result
    result['mapping_error'] = 'accessible cgroup v2 mount not found'
    return result


def discover_blas():
    maps = read_optional('/proc/self/maps')
    if maps['value'] is None: return []
    paths = sorted({line.split()[-1] for line in maps['value'].splitlines()
                    if 'openblas' in line.lower() and line.split()[-1].startswith('/')})
    rows = []
    for path in paths:
        owner = next((name for name in ('numpy', 'scipy') if '/'+name+'.libs/' in path), 'non_target')
        row = {'path': path, 'owner': owner, 'num_threads': None, 'getter': None, 'setter': None}
        try:
            lib = ctypes.CDLL(path)
            for getter in ('openblas_get_num_threads64_', 'openblas_get_num_threads', 'scipy_openblas_get_num_threads'):
                if not hasattr(lib, getter): continue
                function = getattr(lib, getter); function.restype = ctypes.c_int; function.argtypes = []
                setter = getter.replace('get_num_threads', 'set_num_threads')
                row.update(getter=getter, num_threads=function(), setter=setter if hasattr(lib, setter) else None)
                for token, restype in (('get_config', ctypes.c_char_p), ('get_parallel', ctypes.c_int)):
                    name = getter.replace('get_num_threads', token)
                    if hasattr(lib, name):
                        function = getattr(lib, name); function.restype = restype; function.argtypes = []
                        value = function(); row[token] = value.decode() if isinstance(value, bytes) else value
                break
        except (OSError, AttributeError) as exc:
            row['error'] = str(exc)
        rows.append(row)
    return rows


def verify_blas(rows, requested):
    targets = [r for r in rows if r['owner'] in ('numpy', 'scipy')]
    if {r['owner'] for r in targets} != {'numpy', 'scipy'}:
        raise ValueError('THREAD_POLICY_NOT_VERIFIED: NumPy/SciPy loaded BLAS ownership missing')
    for row in targets:
        value = row.get('num_threads')
        if type(value) is not int or value < 1 or (requested is not None and value != requested):
            raise ValueError('THREAD_POLICY_NOT_VERIFIED: '+row['path'])


def configure_blas(rows, requested):
    verify_blas(rows, None)
    targets = [r for r in rows if r['owner'] in ('numpy', 'scipy')]
    if requested is None: return []
    if type(requested) is not int or requested < 1: raise ValueError('invalid BLAS thread budget')
    for row in targets:
        if row['num_threads'] != requested and not row.get('setter'):
            raise ValueError('THREAD_POLICY_NOT_VERIFIED: supported setter missing: '+row['path'])
    applied = []
    for row in targets:
        if row['num_threads'] == requested: continue
        function = getattr(ctypes.CDLL(row['path']), row['setter'])
        function.restype = None; function.argtypes = [ctypes.c_int]
        function(requested)
        applied.append({'path': row['path'], 'setter': row['setter'], 'requested': requested})
    return applied


def snapshot():
    # Force both real BLAS libraries to load before querying/control, not after.
    import numpy as np
    import scipy.linalg
    import scipy
    import cv2
    status = read_optional('/proc/self/status')
    threads = next((int(r.split()[1]) for r in (status['value'] or '').splitlines()
                    if r.startswith('Threads:')), None)
    try:
        import resource
        usage = resource.getrusage(resource.RUSAGE_SELF)
        cpu = {'user_seconds': usage.ru_utime, 'system_seconds': usage.ru_stime, 'peak_rss_kib': usage.ru_maxrss}
    except ImportError:
        cpu = {'user_seconds': None, 'system_seconds': None, 'peak_rss_kib': None}
    return {'pid': os.getpid(), 'python': sys.version, 'executable': sys.executable, 'platform': platform.platform(),
        'numpy': np.__version__, 'scipy': scipy.__version__, 'opencv': cv2.__version__,
        'blas': discover_blas(), 'opencv_threads': cv2.getNumThreads(),
        'thread_environment': {k: os.environ.get(k) for k in THREAD_ENV},
        'visible_cpus': os.cpu_count(), 'affinity': sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else None,
        'created_threads': threads, 'simultaneously_executing_threads': None,
        'load_average': os.getloadavg() if hasattr(os, 'getloadavg') else None,
        'cgroup': cgroup_snapshot(), 'process_cpu_seconds': process_time(), 'monotonic_seconds': perf_counter(), **cpu}


def verify_non_targets(before, after):
    for field in ('opencv_threads', 'thread_environment', 'affinity'):
        if before[field] != after[field]: raise ValueError('NON_TARGET_THREAD_CONFIGURATION_CHANGED: '+field)
    pools = lambda s: [(r['path'], r['num_threads']) for r in s['blas'] if r['owner'] == 'non_target']
    if pools(before) != pools(after): raise ValueError('NON_TARGET_THREAD_CONFIGURATION_CHANGED: BLAS')
