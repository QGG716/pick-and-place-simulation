"""Small public cache/lease primitives reused from feasibility-core.

Source: aa525e873c6a6c8618355923d01f05c9162e87cb, motion_validation.py.
No search framework, interval skipping, or adaptive sampling is imported.
"""
from collections import OrderedDict
from functools import lru_cache
import hashlib
from pathlib import Path
from time import perf_counter
import numpy as np


def exact_snapshot(value):
    if isinstance(value, np.ndarray):
        return value.dtype.str, value.shape, value.tobytes()
    if isinstance(value, dict):
        return tuple((k, exact_snapshot(v)) for k, v in sorted(value.items()))
    if isinstance(value, (list, tuple)):
        return tuple(exact_snapshot(v) for v in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(exact_snapshot(v) for v in value)
    return value


class ContextLease:
    """Sticky invalidation: restoring mutated data cannot revive this lease."""
    def __init__(self, readers, statistics, invalidate=lambda: None):
        self.readers = readers
        self.snapshot = tuple(exact_snapshot(read()) for read in readers)
        self.statistics, self.invalidate = statistics, invalidate
        self.generation = 0

    def current(self):
        started = perf_counter()
        self.statistics['guard_calls'] = self.statistics.get('guard_calls', 0) + 1
        try:
            if not self.generation and any(exact_snapshot(read()) != old
                    for read, old in zip(self.readers, self.snapshot)):
                self.generation += 1
                self.invalidate()
            return not self.generation
        finally:
            self.statistics['guard_seconds'] = self.statistics.get('guard_seconds', 0.) + perf_counter()-started


class LRU(OrderedDict):
    def __init__(self, capacity=4096):
        super().__init__()
        self.capacity = capacity
        self.hits = self.misses = self.evictions = 0

    def lookup(self, key):
        if key not in self:
            self.misses += 1
            return False, None
        self.hits += 1
        self.move_to_end(key)
        return True, self[key]

    def put(self, key, value):
        if key in self:
            self.move_to_end(key)
        elif len(self) >= self.capacity:
            self.popitem(last=False)
            self.evictions += 1
        self[key] = value


@lru_cache(maxsize=1)
def authority_version():
    """Bind evidence to the loaded validation implementation, once per process."""
    root=Path(__file__).parent
    names=('layout_trajectory.py','validation_context.py','pinocchio_backend.py',
           'collision_policy.py','pair_clearance.py','validation_physics.py','geometry.py','robot.py')
    return hashlib.sha256(b''.join(hashlib.sha256((root/name).read_bytes()).digest() for name in names)).hexdigest()
