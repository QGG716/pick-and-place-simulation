"""Optional aggregate counters for one synchronous validation scope.

No per-state JSON or per-pair clock reads. Query families are separate: a
surface-distance query may internally call SAT, so their counts are not added.
"""
from contextlib import contextmanager
from contextvars import ContextVar
import os

_current = ContextVar('validation_work_counts', default=None)
STRATEGY_VERSION = 'stage_invariants_and_aabb_v1_same_strict_grid'


def validation_mode():
    mode = os.environ.get('M710_STAGE_VALIDATION_MODE', 'optimized')
    if mode not in {'reference', 'optimized'}:
        raise ValueError('M710_STAGE_VALIDATION_MODE must be reference or optimized')
    return mode


def count(name, amount=1):
    counters = _current.get()
    if counters is not None:
        counters[name] = counters.get(name, 0) + amount


@contextmanager
def work_counts():
    counters = {}
    token = _current.set(counters)
    try:
        yield counters
    finally:
        _current.reset(token)
