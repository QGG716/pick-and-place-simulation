"""Instrument the existing fixed-input runner, without changing algorithm records.

Run once with --profile to locate work; omit it on BOTH performance runs.
Light timers report inclusive time and time excluding other instrumented calls,
not Python self time. cProfile supplies actual function self/cumulative times.
"""
import argparse
from collections import defaultdict
import cProfile
from functools import wraps
import json
import os
from pathlib import Path
import platform
import pstats
import resource
import sys
from time import perf_counter
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'packages/unloading_contracts/src'), str(ROOT/'tools')]
import numpy as np
import metric_depth_runner as runner
import diagnose_metric_calibration as calibration
import unloading_perception.metric_faces as faces
import unloading_perception.metric_support as support
import unloading_perception.final_geometry as final
import run_isaac_rgbd_geometry as geometry
import validate_metric_faces_ab as delivery


def environment():
    import scipy
    import cv2
    result = dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
        numpy=np.__version__, scipy=scipy.__version__, opencv=cv2.__version__,
        visible_cpus=os.cpu_count(), affinity=sorted(os.sched_getaffinity(0)), load=os.getloadavg(),
        thread_environment={k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
            'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS')})
    for path in ('/sys/fs/cgroup/cpu.max', '/sys/fs/cgroup/cpu.stat', '/sys/fs/cgroup/memory.max',
                 '/sys/fs/cgroup/cpuset.cpus.effective'):
        if Path(path).is_file(): result[path] = Path(path).read_text().strip()
    try:
        from threadpoolctl import threadpool_info
        result['threadpools'] = threadpool_info()
    except ImportError:
        result['threadpools'] = 'threadpoolctl unavailable; no dependency installed'
    return result


class Timers:
    def __init__(self, plan):
        self.plan = plan
        self.module = None
        self.current = None
        self.phase = 'module'
        self.rows = {}
        self.stack = []
        self.totals = {}
        self.originals = []

    def call(self, name, function, *args, **kwargs):
        frame = [perf_counter(), 0.]
        self.stack.append(frame)
        try:
            return function(*args, **kwargs)
        finally:
            elapsed = perf_counter()-frame[0]
            self.stack.pop()
            if self.stack: self.stack[-1][1] += elapsed
            destinations = [self.totals]
            if self.current is not None: destinations.append(self.current['timings'])
            for destination in destinations:
                row = destination.setdefault(self.phase+'/'+name,
                    {'calls': 0, 'inclusive_seconds': 0., 'excluding_instrumented_children_seconds': 0.})
                row['calls'] += 1
                row['inclusive_seconds'] += elapsed
                row['excluding_instrumented_children_seconds'] += elapsed-frame[1]

    def replace(self, module, name, function):
        self.originals.append((module, name, getattr(module, name)))
        setattr(module, name, function)

    def install(self):
        modules = (faces, support, final, runner, geometry)
        targets = [(faces, n) for n in ('_orthogonal_fit', 'maximum_observation_rectangle', '_boundary_kinds',
            '_complete', 'validate_metric_record', '_binding', '_points', '_support_mask', '_support_runs')]
        targets += [(support, n) for n in ('observation_support', 'check_metric_plane', 'backproject_pixels')]
        targets += [(final, 'polygon_pixels')]
        for owner, name in targets:
            original = getattr(owner, name)
            @wraps(original)
            def timed(*args, _f=original, _n=name, **kwargs):
                return self.call(_n, _f, *args, **kwargs)
            # Patch existing import aliases as well; never change arguments/results.
            for module in modules:
                for alias, value in list(vars(module).items()):
                    if value is original: self.replace(module, alias, timed)
        original_extract = runner.extract_observation_labels
        def extract(depth, mask, K, extractor, config):
            spec = next(m for m in self.plan['modules'] if m['module_id'] == self.module)
            ordinal = sum(key[0] == self.module for key in self.rows)
            identity = spec['mask_ids'][ordinal]
            row = {'module_id': self.module, 'mask_id': identity, 'mask_pixels': int(np.count_nonzero(mask)),
                   'timings': {}, 'status': 'RUNNING'}
            self.rows[self.module, identity] = self.current = row
            self.phase = 'extraction'
            result = self.call('extract_observation_labels', original_extract, depth, mask, K, extractor, config)
            labels, seeds, audit = result
            row.update(initial_plane_count=len({s['initial_plane_index'] for s in seeds}),
                connected_label_count=len(seeds), extraction_retained_pixels=audit['retained_pixels'])
            return result
        self.replace(runner, 'extract_observation_labels', extract)
        original_fit = runner.fit_metric_faces
        def fit(*args, **kwargs):
            if kwargs['mask_id'] != self.current['mask_id']: raise ValueError('profiling instance order mismatch')
            self.phase = 'fit'
            result = self.call('fit_metric_faces', original_fit, *args, **kwargs)
            self.current.update(status='COMPLETED', final_face_count=len(result['camera_facing_faces']),
                accepted_complete_cuboid=result['accepted'], final_validation=result['final_face_validation'],
                complete_observability=result['complete_observability'])
            self.current = None; self.phase = 'module'
            return result
        self.replace(runner, 'fit_metric_faces', fit)
        original_validate = geometry.validate_final_record
        def validate(record, *args, **kwargs):
            self.current = self.rows[self.module, record['mask_id']]
            self.phase = 'independent_final_validation'
            try:
                return self.call('validate_final_record', original_validate, record, *args, **kwargs)
            finally:
                self.current = None; self.phase = 'module'
        self.replace(geometry, 'validate_final_record', validate)
        original_load = calibration.load_extractor
        def load(*args, **kwargs):
            extractor = original_load(*args, **kwargs)
            return lambda *a, **kw: self.call('pinned_upstream_fit_planes', extractor, *a, **kw)
        self.replace(calibration, 'load_extractor', load)
        original_module = delivery._run_secondary_module
        def module(**kwargs):
            self.module = kwargs['payload'].camera['module_id']
            return original_module(**kwargs)
        self.replace(delivery, '_run_secondary_module', module)

    def restore(self):
        for module, name, original in reversed(self.originals): setattr(module, name, original)


def profile_rows(profile):
    stats = pstats.Stats(profile)
    rows = []
    for (filename, line, name), (primitive, total, self_time, cumulative, callers) in stats.stats.items():
        rows.append(dict(file=filename, line=line, function=name, primitive_calls=primitive, total_calls=total,
            self_seconds=self_time, cumulative_seconds=cumulative))
    return sorted(rows, key=lambda r: -r['cumulative_seconds'])


def main(argv=None, *, runtime_policy=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', action='store_true')
    args = parser.parse_args(argv)
    plan = delivery.read(args.plan)
    timers = Timers(plan)
    before = environment()
    profiler = cProfile.Profile() if args.profile else None
    timers.install()
    try:
        if profiler: profiler.enable()
        status = delivery.run(SimpleNamespace(plan=args.plan, output=args.output, diagnostic='off', runtime_policy=runtime_policy))
    finally:
        if profiler: profiler.disable()
        timers.restore()
    if profiler:
        profiler.dump_stats(str(args.output/'functions.prof'))
        delivery.write(args.output/'function_profile.json', profile_rows(profiler))
    for spec in plan['modules']:
        actual = [r['mask_id'] for r in timers.rows.values() if r['module_id'] == spec['module_id']]
        if actual != spec['mask_ids']: status = 1
    payload = {'status': 'COMPLETED' if status == 0 else 'FAILED', 'heavy_profiler_enabled': args.profile,
        'environment_before': before, 'environment_after': environment(),
        'max_rss_kib_process_lifetime': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        'timing_semantics': __doc__, 'totals': timers.totals, 'instances': list(timers.rows.values()),
        'production_sha256': {str(p.relative_to(ROOT)): delivery.digest(p) for p in
            [ROOT/'src/unloading_perception'/n for n in ('metric_faces.py', 'metric_support.py', 'final_geometry.py')]
            + [ROOT/'tools'/n for n in ('metric_depth_runner.py', 'run_isaac_rgbd_geometry.py')]}}
    delivery.write(args.output/'function_timings.json', payload)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
