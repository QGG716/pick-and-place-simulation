"""prepare() through the worker's C++ native_prepare.h (user decision C, 2026-11-23).

``prepare_implementation=NATIVE_LIBRARY`` (PYTHON planning flow) calls
``libm710_prepare.so`` from the configured native install prefix with ctypes.
It is the same header the worker uses, so the step-3 in-process flow and this
Python flow share one implementation. The C++ arithmetic follows scipy/numpy
step by step and agrees with controlled/trajectory.py prepare() to rounding,
not bitwise; tools/check_prepare_library.py records the differences on saved
plans. ctypes releases the interpreter lock during the call. Errors carry
the Python error strings; a missing or mismatched library fails closed.
"""
import ctypes
import hashlib
import os
import threading
from dataclasses import asdict
from pathlib import Path

import numpy as np

ABI_VERSION = 1
LOCK = threading.Lock()
LIBRARY = {}


def library_path():
    # The same prefix rule as ros2/m710_moveit_backend/worker.sh.
    prefix = os.environ.get('M710_MOVEIT_INSTALL_PREFIX') or str(Path(__file__).resolve().parents[4]/'install'/'moveit')
    return Path(prefix)/'lib'/'libm710_prepare.so'


def load():
    """The loaded library and its identity (path, SHA-256); raises if unavailable."""
    path = library_path()
    with LOCK:
        cached = LIBRARY.get(str(path))
        if cached is not None:
            return cached
        if not path.is_file():
            raise RuntimeError(f'NATIVE_PREPARE_LIBRARY_MISSING:{path}')
        lib = ctypes.CDLL(str(path))
        lib.m710_prepare_abi_version.restype = ctypes.c_int
        if lib.m710_prepare_abi_version() != ABI_VERSION:
            raise RuntimeError('NATIVE_PREPARE_LIBRARY_ABI_MISMATCH')
        d = ctypes.POINTER(ctypes.c_double)
        lib.m710_prepare_run.restype = ctypes.c_void_p
        lib.m710_prepare_run.argtypes = [d, d, d, d, ctypes.c_size_t, d, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int]
        lib.m710_prepare_error.restype = ctypes.c_char_p
        lib.m710_prepare_error.argtypes = [ctypes.c_void_p]
        lib.m710_prepare_size.restype = ctypes.c_size_t
        lib.m710_prepare_size.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.m710_prepare_copy.restype = None
        lib.m710_prepare_copy.argtypes = [ctypes.c_void_p, d, d, d, d, d, d, ctypes.POINTER(ctypes.c_long), d]
        lib.m710_prepare_free.restype = None
        lib.m710_prepare_free.argtypes = [ctypes.c_void_p]
        identity = dict(name='m710_prepare', abi_version=ABI_VERSION, path=str(path),
                        sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        LIBRARY[str(path)] = cached = (lib, identity)
        return cached


def _pointer(array):
    return array.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


def prepare(points, limits, time_scale=10., period=.005, reach_bound_m=10., probe_grid='SENT_GRID'):
    """controlled/trajectory.prepare() with the same arguments, result layout and error strings."""
    from ..controlled.trajectory import PROBE_GRIDS
    if probe_grid not in PROBE_GRIDS:
        raise ValueError('INVALID_PROBE_GRID')
    lib, identity = load()
    try:
        t = np.ascontiguousarray([p['t'] for p in points], dtype=np.float64)
        q = np.ascontiguousarray([p['q'] for p in points], dtype=np.float64)
        v = np.ascontiguousarray([p['v'] for p in points], dtype=np.float64)
        a = np.ascontiguousarray([p['a'] for p in points], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        raise ValueError('INVALID_TIMED_TRAJECTORY') from None
    if t.ndim != 1 or q.shape != (len(t), 6) or v.shape != q.shape or a.shape != q.shape:
        raise ValueError('INVALID_TIMED_TRAJECTORY')
    lower, upper, vmax, amax = limits.arrays()
    bounds = np.ascontiguousarray(np.r_[lower, upper, vmax, amax], dtype=np.float64)
    geometric = probe_grid == 'GEOMETRIC'
    handle = lib.m710_prepare_run(_pointer(t), _pointer(q), _pointer(v), _pointer(a), len(t), _pointer(bounds),
                                  float(time_scale), float(period), float(reach_bound_m), int(geometric))
    if not handle:
        raise RuntimeError('NATIVE_PREPARE_LIBRARY_CALL_FAILED')
    try:
        error = lib.m710_prepare_error(handle).decode()
        if error:
            raise ValueError(error)
        sizes = [lib.m710_prepare_size(handle, k) for k in range(4)]
        ts = np.empty(sizes[0]); qs = np.empty((sizes[0], 6)); vs = np.empty((sizes[0], 6)); acc = np.empty((sizes[0], 6))
        collision = np.empty((sizes[1], 6)); speed = np.empty(sizes[3]); per_interval = np.zeros(max(1, sizes[2]), dtype=ctypes.c_long)
        scalars = np.empty(5)
        lib.m710_prepare_copy(handle, _pointer(ts), _pointer(qs), _pointer(vs), _pointer(acc), _pointer(collision), _pointer(speed),
                              per_interval.ctypes.data_as(ctypes.POINTER(ctypes.c_long)), _pointer(scalars))
    finally:
        lib.m710_prepare_free(handle)
    speed_bounds = speed.tolist()
    if geometric:
        steps = per_interval[:sizes[2]].tolist()
        sampling = dict(method='GEOMETRIC_PER_KNOT_INTERVAL_ANALYTIC_SPEED_BOUND', reach_bound_m=reach_bound_m,
                        analytic_speed_bounds_rad_s=speed_bounds, maximum_pair_displacement_between_checks_m=.001,
                        knot_intervals=len(steps), probe_states=int(sizes[1]), max_subdivisions_per_interval=int(max(steps)),
                        rule='SUM_J_MAX_ABS_QDOT_ON_INTERVAL_TIMES_STEP_TIMES_4_REACH_LE_1_MM')
    else:
        sampling = dict(reach_bound_m=reach_bound_m, analytic_speed_bounds_rad_s=speed_bounds,
                        maximum_pair_displacement_between_checks_m=.001, subdivisions_per_send=int(scalars[4]))
    return dict(schema='eco65_timed_send_v1', period_s=period, time_scale=time_scale,
                source_points=points, source_duration_s=float(scalars[0]),
                scaled_duration_s=float(scalars[1]), send_duration_s=float(scalars[2]),
                points=[dict(t=float(tt), q=qq, v=vv, a=aa) for tt, qq, vv, aa in zip(ts.tolist(), qs.tolist(), vs.tolist(), acc.tolist())],
                collision_path=collision.tolist(), limits=asdict(limits), collision_sampling=sampling,
                interpolation='quintic_q_v_a_nonuniform_knots', terminal_hold_s=float(scalars[3]),
                prepare_implementation=dict(identity, scope='USER_DECISION_C_2026_11_23_SAME_CPP_PREPARE_AS_THE_WORKER'))
