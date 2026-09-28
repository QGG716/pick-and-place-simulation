"""Shared geometry parameters/configuration; standard library only, no runtime imports."""
import argparse
from pathlib import Path
import sys
from types import SimpleNamespace


def positive_int(value):
    try:
        result = int(value)
    except (ValueError, TypeError):
        raise argparse.ArgumentTypeError('BLAS threads/timeout must be a positive integer')
    if result < 1 or isinstance(value, bool):
        raise argparse.ArgumentTypeError('BLAS threads/timeout must be a positive integer')
    return result


def policy(threads):
    return {'schema_version': 'metric_blas_policy_v1', 'requested_blas_threads': threads,
            'scope': 'standalone_metric_geometry_process', 'targets': ['numpy', 'scipy'],
            'mechanism': 'inherit' if threads is None else 'native_openblas_runtime_api',
            'non_target_thread_settings': 'unchanged'}


def validate_settings(backend, threads, timeout=1200):
    if backend not in ('inline', 'subprocess'): raise ValueError('unknown geometry backend')
    if threads is not None and (type(threads) is not int or threads < 1):
        raise ValueError('BLAS threads must be a positive integer')
    if backend == 'inline' and threads is not None:
        raise ValueError('--geometry-blas-threads requires --geometry-backend subprocess')
    if type(timeout) is not int or timeout < 1: raise ValueError('geometry timeout must be a positive integer')


def add_arguments(parser, *, python_default=Path(sys.executable)):
    parser.add_argument('--geometry-backend', choices=('inline', 'subprocess'), default='inline')
    parser.add_argument('--geometry-blas-threads', type=positive_int)
    parser.add_argument('--geometry-python', type=Path, default=python_default)
    parser.add_argument('--geometry-timeout', type=positive_int, default=1200,
                        help='seconds for the entire geometry child, including input validation')


def validate_arguments(parser, args):
    try: validate_settings(args.geometry_backend, args.geometry_blas_threads, args.geometry_timeout)
    except ValueError as exc: parser.error(str(exc))


def forwarded_arguments(args):
    result = ['--geometry-backend', args.geometry_backend, '--geometry-python', str(args.geometry_python),
              '--geometry-timeout', str(args.geometry_timeout)]
    if args.geometry_blas_threads is not None:
        result += ['--geometry-blas-threads', str(args.geometry_blas_threads)]
    return result


def effective_config(config, backend, threads):
    validate_settings(backend, threads)
    if backend == 'inline': return config
    return {**config, 'metric_runtime_policy': policy(threads),
            'geometry_runtime': {'backend': 'subprocess', 'protocol': 'workcell_geometry_v1'}}


def ros_parameters(args):
    validate_settings(args.geometry_backend, args.geometry_blas_threads, args.geometry_timeout)
    return {'geometry_backend': args.geometry_backend,
            'geometry_blas_threads': args.geometry_blas_threads or 0,  # ROS-only sentinel: inherit
            'geometry_python': str(args.geometry_python.absolute()), 'geometry_timeout': args.geometry_timeout}


def from_ros_parameters(value, algorithm_python):
    threads = value('geometry_blas_threads')
    if type(threads) is not int or threads < 0:
        raise ValueError('ROS geometry_blas_threads must be 0 (inherit) or a positive integer')
    args = SimpleNamespace(geometry_backend=value('geometry_backend'),
        geometry_blas_threads=None if threads == 0 else threads,
        geometry_python=Path(value('geometry_python') or algorithm_python).absolute(),
        geometry_timeout=value('geometry_timeout'))
    validate_settings(args.geometry_backend, args.geometry_blas_threads, args.geometry_timeout)
    return args


def worker_command(project, algorithm_python, output, models, vision, geometry):
    validate_settings(geometry.geometry_backend, geometry.geometry_blas_threads, geometry.geometry_timeout)
    return [str(Path(algorithm_python).absolute()), str(Path(project)/'tools/workcell_video_worker.py'),
            '--output', str(output), '--models', str(models), '--vision', str(vision)] + forwarded_arguments(geometry)
