"""prepare_processes (2-second planning, 2026-11-23): the send-grid preparation in resident child processes.

``controlled.trajectory.prepare`` is a pure function of its arguments. In the
planning process it runs as hundreds of short numpy/scipy calls on the lane
threads, and every call that releases the GIL waits again behind a dozen
CPU-bound planning threads: a segment that takes 20-80 ms alone took
0.5-1.4 s in the six-segment replays. Here the identical function runs in a
child process on the identical arguments (same interpreter, numpy and scipy;
the result is unpickled bit for bit), so the planning process only pays one
unpickling. Exceptions (INTERPOLATION_LIMIT, ...) come back unchanged.
"""
import multiprocessing
import os
from concurrent.futures import ProcessPoolExecutor


def _warm():
    from ..controlled import trajectory  # noqa: F401  (imports numpy / scipy in the child)
    return os.getpid()


def _prepare(points, limits, time_scale, period, reach_bound_m, probe_grid):
    from ..controlled.trajectory import prepare
    return prepare(points, limits, time_scale, period, reach_bound_m, probe_grid=probe_grid)


def _search(points, limits, time_scale, factors, period, reach_bound_m, probe_grid):
    from ..controlled.trajectory import slowdown_search
    return slowdown_search(points, limits, time_scale, factors, period, reach_bound_m, probe_grid)


class PreparePool:
    def __init__(self, processes):
        if type(processes) is not int or not 1 <= processes <= 8:
            raise ValueError('PREPARE_PROCESSES_INVALID')
        self.processes = processes
        self.executor = ProcessPoolExecutor(max_workers=processes, mp_context=multiprocessing.get_context('spawn'))
        self.pids = sorted(set(f.result(timeout=120) for f in [self.executor.submit(_warm) for _ in range(4*processes)]))

    def prepare(self, points, limits, time_scale, period, reach_bound_m, probe_grid='SENT_GRID'):
        return self.executor.submit(_prepare, points, limits, time_scale, period, reach_bound_m, probe_grid).result()

    def search(self, points, limits, time_scale, factors, period, reach_bound_m, probe_grid='SENT_GRID'):
        """analytic_send_slowdown: the whole factor search in one child call (see trajectory.slowdown_search)."""
        return self.executor.submit(_search, points, limits, time_scale, tuple(factors), period, reach_bound_m, probe_grid).result()

    def close(self):
        self.executor.shutdown(wait=True, cancel_futures=True)
