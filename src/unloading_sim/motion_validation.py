"""One immutable geometric contract for search, optimization and final replay paths.

DISCRETE is the explicitly retained legacy grid, not a continuous certificate.
An interval certificate may replace only the checks it actually covers.
"""
from collections import OrderedDict
from dataclasses import dataclass, field, replace
from enum import Enum
import hashlib
import json
from time import perf_counter
import numpy as np


class Status(str, Enum):
    VALID = "VALID"
    INVALID = "INVALID"
    INDETERMINATE = "INDETERMINATE"
    CANCELLED = "CANCELLED"


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

    def get(self, key, default=None):
        hit,value=self.lookup(key)
        return value if hit else default


@dataclass(frozen=True)
class ValidationContext:
    binding_json: str
    resolution_rad: float
    poc_dense_grid: bool = False
    interpolation: str = "linear_joint"
    guarantee: str = "DISCRETE_LEGACY_STRICT"
    strategy_version: str = "shared_motion_contract_v1"
    max_depth: int = 12
    context_id: str = field(init=False)

    def __post_init__(self):
        if not np.isfinite(self.resolution_rad) or self.resolution_rad <= 0:
            raise ValueError("positive finite resolution required")
        if self.guarantee not in {"DISCRETE_LEGACY_STRICT", "CONTINUOUS"} or self.max_depth < 0:
            raise ValueError("invalid validation contract")
        # Canonical JSON owns a detached immutable snapshot of all caller data.
        binding = json.dumps(json.loads(self.binding_json), sort_keys=True, separators=(",", ":"), allow_nan=False)
        object.__setattr__(self, "binding_json", binding)
        identity = (binding, self.resolution_rad, self.poc_dense_grid, self.interpolation,
                    self.guarantee, self.strategy_version, self.max_depth)
        object.__setattr__(self, "context_id", hashlib.sha256(repr(identity).encode()).hexdigest())

    @classmethod
    def create(cls, binding, **kwargs):
        return cls(json.dumps(binding, sort_keys=True, allow_nan=False), **kwargs)

    def samples(self, a, b):
        n = max(1, int(np.ceil(np.max(np.abs(b-a))/self.resolution_rad)))
        if self.poc_dense_grid:
            n = max(n, int(np.ceil(4.*np.sum(np.abs(b-a))/.0025)))
        return 2*n  # Exactly the previous strict final grid, including both ends.


@dataclass(frozen=True)
class ValidationResult:
    status: Status
    context_id: str
    guarantee: str = "NONE"
    failure: dict | None = None
    interval: tuple | None = None
    checked: tuple = ()
    statistics: dict = field(default_factory=dict)
    cache_hit: bool = False
    interval_certificate: bool = False

    @property
    def valid(self):
        return self.status == Status.VALID

    def evidence(self):
        return dict(status=self.status.value, context_id=self.context_id, guarantee=self.guarantee,
                    failure=self.failure, interval=self.interval, checked=self.checked,
                    statistics=self.statistics, cache_hit=self.cache_hit,
                    interval_certificate=self.interval_certificate)


@dataclass
class RequestBudget:
    """One monotonic deadline/work counter; shared by repair and optimization."""
    deadline: float | None = None
    max_checks: int | None = None
    cancelled: object = lambda: False
    checks: int = 0
    parent: object = None

    def consume(self, count):
        self.checks += count
        if self.parent is not None: self.parent.consume(count)

    def available(self):
        own = None if self.max_checks is None else max(0,self.max_checks-self.checks)
        other = None if self.parent is None else self.parent.available()
        values = [v for v in (own,other) if v is not None]
        return min(values) if values else None

    def status(self):
        if self.parent is not None and self.parent.status(): return self.parent.status()
        if self.cancelled():
            return Status.CANCELLED
        if ((self.deadline is not None and perf_counter() >= self.deadline)
                or (self.max_checks is not None and self.checks >= self.max_checks)):
            return Status.INDETERMINATE
        return None

    def interrupted(self):
        """Completion may consume the last work unit; deadlines/cancellation still bind."""
        if self.parent is not None and self.parent.interrupted(): return self.parent.interrupted()
        if self.cancelled(): return Status.CANCELLED
        if self.deadline is not None and perf_counter() >= self.deadline: return Status.INDETERMINATE
        return None


class MotionValidator:
    def __init__(self, context, check_state, *, check_states=None, context_current=None,
                 interval_proof=None, cache_capacity=4096, cache_states=True):
        self.context = context
        self.scalar = check_state
        self.batch = check_states
        self.context_current = context_current
        self.interval_proof = interval_proof
        self.states, self.edges = LRU(cache_capacity), LRU(cache_capacity)
        self.cache_states = cache_states
        self.statistics = dict(state_samples=0, edge_calls=0, subdivisions=0,
            certified_intervals=0, pair_certificates=0, state_seconds=0., edge_seconds=0.,
            repeated_failed_edges=0, batch_calls=0)

    def _result(self, status, failure=None, **kwargs):
        if failure is not None and 'type' not in failure:
            reason=str(failure.get('reason',''))
            kind=('EVIDENCE' if status == Status.INDETERMINATE else 'STAGE_CONSTRAINT'
                  if any(s in reason for s in ('JOINT','SINGULAR','RADIAL','PROXIMITY','PROGRESS')) else 'GEOMETRY')
            failure=dict(failure,type=kind)
        return ValidationResult(status, self.context.context_id,
            guarantee=self.context.guarantee if status == Status.VALID else "NONE",
            failure=failure, **kwargs)

    def _guard(self, budget):
        if self.context_current is not None and self.context_current() != self.context.context_id:
            return self._result(Status.INDETERMINATE, dict(reason="VALIDATION_CONTEXT_CHANGED", type="CONTEXT"))
        status = budget.status()
        if status:
            return self._result(status, dict(reason=status.value, type="BUDGET_OR_CANCEL"))
        return None

    def check_states(self, states, budget=None, *, proof=None):
        budget = budget or RequestBudget()
        started = perf_counter()
        output = []
        # Native kernels accept bounded chunks; cancellation is checked between chunks.
        for first in range(0, len(states), 32):
            stop = self._guard(budget)
            if stop:
                output.extend([stop]*(len(states)-first)); break
            chunk = states[first:first+32]
            results, missing, indices, keys = [None]*len(chunk), [], [], []
            for i, q in enumerate(chunk):
                q = np.asarray(q, float)
                key = (self.context.context_id, q.shape, q.tobytes())
                hit, old = self.states.lookup(key) if self.cache_states else (False, None)
                if hit:
                    results[i] = replace(old, cache_hit=True)
                else:
                    missing.append(q); indices.append(i); keys.append(key)
            if missing:
                remaining = budget.available()
                if remaining is not None and len(missing) > remaining:
                    missing, indices, keys = missing[:remaining], indices[:remaining], keys[:remaining]
                self.statistics['batch_calls'] += int(self.batch is not None)
                failures = (self.batch(missing, proof=proof) if self.batch is not None else
                            [self.scalar(q) for q in missing])
                if len(failures) != len(missing):
                    raise RuntimeError("validation batch output dimension mismatch")
                budget.consume(len(missing))
                self.statistics['state_samples'] += len(missing)
                for i, key, failure in zip(indices, keys, failures):
                    if isinstance(failure, ValidationResult):
                        result = failure
                    else:
                        uncertain = failure and (failure.get('classification') == 'UNKNOWN' or
                            any(s in str(failure.get('reason', '')) for s in ['DEADLINE', 'TIMEOUT', 'UNKNOWN']))
                        result = self._result(Status.INDETERMINATE if uncertain else
                            Status.INVALID if failure else Status.VALID, failure)
                    results[i] = result
                    if self.cache_states and result.status in {Status.VALID, Status.INVALID}:
                        self.states.put(key, result)
            # A call finishing after its deadline does not certify the remaining motion.
            if budget.interrupted():
                stop = self._result(budget.interrupted(),
                                    dict(reason="VALIDATION_INTERRUPTED", type="BUDGET_OR_CANCEL"))
                results = [r if r is not None and r.status == Status.INVALID else stop for r in results]
            output.extend(r or self._result(Status.INDETERMINATE, dict(reason="VALIDATION_WORK_BUDGET")) for r in results)
        self.statistics['state_seconds'] += perf_counter()-started
        return output

    def check_motion(self, a, b, budget=None, *, parameters=None):
        budget = budget or RequestBudget()
        started = perf_counter()
        self.statistics['edge_calls'] += 1
        a, b = np.asarray(a, float), np.asarray(b, float)
        stop = self._guard(budget)
        if stop: return stop
        if a.ndim != 1 or a.shape != b.shape or not len(a) or not np.isfinite([a,b]).all():
            return self._result(Status.INVALID, dict(reason="JOINT_VECTOR_INVALID", type="INPUT"))
        if self.context.interpolation != 'linear_joint':
            return self._result(Status.INDETERMINATE, dict(reason="UNSUPPORTED_INTERPOLATION", type="CONTRACT"))
        # Direction, exact doubles, parameterization and time are part of the key.
        key = (self.context.context_id, a.tobytes(), b.tobytes(), json.dumps(parameters, sort_keys=True, allow_nan=False))
        hit, old = self.edges.lookup(key) if self.cache_states else (False, None)
        if hit:
            self.statistics['repeated_failed_edges'] += int(old.status == Status.INVALID)
            return replace(old, cache_hit=True)
        n = self.context.samples(a,b)
        grid = np.linspace(0.,1.,n+1)
        checked = []
        before = dict(self.statistics)

        def interval(lo, hi, depth, inherited=None):
            stop = self._guard(budget)
            if stop: return stop
            left, right = a+grid[lo]*(b-a), a+grid[hi]*(b-a)
            # Tiny discrete edges cannot amortize preparing an interval scene.
            # This is a work strategy only: every original strict sample remains.
            proof = (self.interval_proof(left, right) if self.interval_proof and
                     (hi-lo >= 32 or self.context.guarantee == 'CONTINUOUS') else inherited)
            if proof:
                if proof.get('context_id') != self.context.context_id:
                    raise RuntimeError("interval evidence context mismatch")
                self.statistics['pair_certificates'] += len(proof.get('pairs', ()))
                if proof.get('complete_contract'):
                    self.statistics['certified_intervals'] += 1
                    checked.append((lo/n,hi/n,'CONTINUOUS_CERTIFICATE'))
                    return self._result(Status.VALID, interval_certificate=True)
            continuous = self.context.guarantee == 'CONTINUOUS'
            if (continuous or proof is not None) and hi-lo > 32 and depth < self.context.max_depth:
                self.statistics['subdivisions'] += 1
                mid = (lo+hi)//2
                result = interval(lo, mid, depth+1,proof)
                return interval(mid, hi, depth+1,proof) if result.valid else result
            fractions = grid[lo:hi+1]
            values = a[None,:]+fractions[:,None]*(b-a)[None,:]
            results = self.check_states(values, budget, proof=proof)
            for fraction, q, result in zip(fractions, values, results):
                if not result.valid:
                    checked.append((lo/n,float(fraction),'OBSERVED_THROUGH_FAILURE'))
                    failure = dict(result.failure or {}, fraction=float(fraction), q_rad=q.tolist(),
                                   motion_start=a.tolist(), motion_end=b.tolist())
                    return replace(result, failure=failure, interval=(lo/n,hi/n))
            checked.append((lo/n,hi/n,'DISCRETE_LEGACY_STRICT'))
            if continuous:
                return self._result(Status.INDETERMINATE, dict(reason='CONTINUOUS_PROOF_INCOMPLETE', type='CONTRACT'), interval=(lo/n,hi/n))
            return self._result(Status.VALID)

        result = interval(0,n,0)
        stop = self._guard(RequestBudget(cancelled=lambda: budget.interrupted() == Status.CANCELLED))
        if budget.interrupted(): stop = self._result(budget.interrupted(),dict(reason='VALIDATION_INTERRUPTED'))
        if stop and result.valid: result = stop
        self.statistics['edge_seconds'] += perf_counter()-started
        stats={k:self.statistics[k]-before[k] for k in before}
        stats.update(state_cache_hits=self.states.hits,state_cache_misses=self.states.misses,
            state_cache_evictions=self.states.evictions,edge_cache_hits=self.edges.hits,
            edge_cache_evictions=self.edges.evictions)
        result = replace(result, checked=tuple(checked), statistics=stats)
        if self.cache_states and result.status in {Status.VALID, Status.INVALID}:
            self.edges.put(key,result)
        return result

    def check_path(self, path, budget=None):
        budget = budget or RequestBudget()
        if not len(path): return self._result(Status.INVALID, dict(reason='EMPTY_PATH'))
        if len(path) == 1: return self.check_states(path,budget)[0]
        checked = []
        for index,(a,b) in enumerate(zip(path[:-1],path[1:])):
            result = self.check_motion(a,b,budget)
            if not result.valid:
                return replace(result,failure=dict(result.failure or {},edge=index),checked=tuple(checked)+result.checked)
            checked.append((index,result.guarantee))
        return self._result(Status.VALID,checked=tuple(checked))

    def feedback_failure(self, a, b, result, *, parameters=None):
        """Exact failed motion only; a contradictory completed check is a defect."""
        if result.context_id != self.context.context_id:
            raise RuntimeError('VALIDATION_CONTEXT_CHANGED: feedback rejected')
        key=(self.context.context_id,np.asarray(a,float).tobytes(),np.asarray(b,float).tobytes(),json.dumps(parameters,sort_keys=True,allow_nan=False))
        old=self.edges.get(key)
        if old is not None and old.valid and result.status == Status.INVALID:
            raise RuntimeError('VALIDATION_CONSISTENCY_DEFECT: same context and motion changed verdict')
        if result.status == Status.INVALID: self.edges.put(key,result)
