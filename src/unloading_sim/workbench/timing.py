"""Development latency measurement for the ECO65 real-site chain.

Disabled unless the process starts with ``ECO65_TIMING=1``. While disabled
every helper here performs exactly the wrapped operation and records nothing.
While enabled, spans are appended to an in-memory list (never written on the
measured path) and each task phase hands its list to a background thread that
writes one ``timing-<phase>.json``. Enabling also sets ``M710_NATIVE_TIMING=1``
so resident native workers started afterwards report their own segments.

No helper changes a decision, a threshold, a check, a record file or a reply
seen by the workbench. Native ``native_timing`` reply fields are removed before
any caller sees the reply. Measurement overhead (extra serialization done only
to measure it) is accumulated separately in ``overhead_s``.

High-rate spans (every supervisor tick, RPC and file rehash) are folded into
fixed-size aggregates instead of retained events: retaining thousands of
small objects in a process that also holds the source snapshot measurably
lengthened Python's full garbage collections and moved them onto the 20 ms
supervisor tick. ``gc_pauses`` reports every collection's pause while enabled.
"""
from collections import deque
from contextlib import contextmanager, nullcontext
import gc
import json
import math
import os
from pathlib import Path
import threading
import time

ENABLED = os.environ.get('ECO65_TIMING') == '1'
if ENABLED:
    os.environ['M710_NATIVE_TIMING'] = '1'

_local = threading.local()

# Log histogram: 10 bins per decade from 1 us to 100 s, plus under/overflow.
_HIST_MIN, _HIST_PER_DECADE, _HIST_BINS = 1e-6, 10, 82


def _bin(value):
    if not value > _HIST_MIN:
        return 0
    return min(_HIST_BINS-1, int(math.ceil(math.log10(value/_HIST_MIN)*_HIST_PER_DECADE)))


def _upper(index):
    return _HIST_MIN*10**(index/_HIST_PER_DECADE) if index < _HIST_BINS-1 else math.inf


class Recorder:
    def __init__(self):
        self._lock = threading.Lock()
        self.events = []
        self.aggregates = {}
        self.overhead_s = 0.
        self.gc_events = deque(maxlen=8192)
        self.gc_overflow = 0

    def add(self, name, start, duration, **attrs):
        if getattr(_local, 'suppressed', 0):
            return
        event = dict(name=name, start_monotonic_s=start, duration_s=duration,
                     thread=threading.current_thread().name, **getattr(_local, 'tags', {}), **attrs)
        with self._lock:
            self.events.append(event)

    def aggregate(self, name, key, value, **attrs):
        """Fold one value into a fixed-size counter/histogram (no retained event)."""
        if getattr(_local, 'suppressed', 0):
            return
        with self._lock:
            if (name,key) not in self.aggregates and len(self.aggregates)>=4096:
                name,key,attrs='timing.aggregate_overflow','distinct_keys',{}
            a = self.aggregates.get((name, key))
            if a is None:
                a = self.aggregates[(name, key)] = [0, 0., 0., [0]*_HIST_BINS, dict(attrs)]
            a[0] += 1; a[1] += value
            if value > a[2]: a[2] = value
            a[3][_bin(value)] += 1

    def gc_pause(self, generation, duration):
        # GC may interrupt any recorder operation while its lock is held.
        # Only atomic deque append here; folding happens outside the callback.
        if len(self.gc_events)==self.gc_events.maxlen:self.gc_overflow+=1
        self.gc_events.append((generation,duration))

    def overhead(self, seconds):
        with self._lock:
            self.overhead_s += seconds

    def take(self, since=None):
        """Remove all events and aggregates; return events started at or after ``since``."""
        with self._lock:
            taken, self.events = self.events, []
            aggregates, self.aggregates = self.aggregates, {}
            overhead, self.overhead_s = self.overhead_s, 0.
            pauses, self.gc_events = self.gc_events, deque(maxlen=8192)
            dropped, self.gc_overflow = self.gc_overflow, 0
        for generation,duration in pauses:
            key=('python.gc_pause',generation)
            a=aggregates.setdefault(key,[0,0.,0.,[0]*_HIST_BINS,{}])
            a[0]+=1;a[1]+=duration;a[2]=max(a[2],duration);a[3][_bin(duration)]+=1
        if dropped:
            taken.append(dict(name='python.gc_pause_overflow',count=dropped,duration_s=0.,start_monotonic_s=None))
        if since is not None:
            taken = [e for e in taken if e['start_monotonic_s'] >= since]
        taken.extend(_aggregate_rows(aggregates))
        return taken, overhead


def _aggregate_rows(aggregates):
    rows = []
    for (name, key), (count, total, maximum, bins, attrs) in aggregates.items():
        def quantile(q):
            target, seen = q*count, 0
            for i, n in enumerate(bins):
                seen += n
                if seen >= target: return min(_upper(i), maximum)
            return maximum
        rows.append(dict(name=name, key=key, aggregate=True, count=count, sum=total, max=maximum,
                         mean=total/count if count else None, p50_upper=quantile(.5), p90_upper=quantile(.9),
                         p99_upper=quantile(.99), duration_s=total, start_monotonic_s=None,
                         histogram_upper_count=[[_upper(i), n] for i, n in enumerate(bins) if n], **attrs))
    return rows


recorder = Recorder()


def aggregate(name, key, value, **attrs):
    if ENABLED:
        recorder.aggregate(name, key, value, **attrs)


@contextmanager
def aggregated(name, key):
    """Time a high-rate scope into an aggregate; a no-op when disabled."""
    if not ENABLED:
        yield
        return
    start = time.monotonic()
    try:
        yield
    finally:
        recorder.aggregate(name, key, time.monotonic()-start)


if ENABLED:
    # Measurement only: the pause of every Python collection in this process.
    _gc_start = [0.]

    def _gc_callback(phase, info):
        if phase == 'start':
            _gc_start[0] = time.monotonic()
        else:
            recorder.gc_pause(info['generation'], time.monotonic()-_gc_start[0])
    gc.callbacks.append(_gc_callback)


class _Span:
    __slots__ = ('name', 'attrs', 'start')

    def __init__(self, name, attrs):
        self.name, self.attrs = name, attrs

    def __enter__(self):
        self.start = time.monotonic()
        return self

    def __exit__(self, kind, value, traceback):
        attrs = dict(self.attrs)
        if kind is not None:
            attrs['error'] = kind.__name__
        recorder.add(self.name, self.start, time.monotonic()-self.start, **attrs)
        return False


def span(name, **attrs):
    """Context manager timing one scope; a no-op when disabled."""
    return _Span(name, attrs) if ENABLED else nullcontext()


def add(name, start, duration, **attrs):
    if ENABLED:
        recorder.add(name, start, duration, **attrs)


def now():
    return time.monotonic()


class Laps:
    """Consecutive segments: ``lap('name')`` records the time since the previous lap."""
    __slots__ = ('prefix', 't', 'attrs')

    def __init__(self, prefix, **attrs):
        self.prefix, self.attrs = prefix, attrs
        self.t = time.monotonic() if ENABLED else 0.

    def __call__(self, name, **attrs):
        if not ENABLED:
            return
        t = time.monotonic()
        recorder.add(self.prefix+name, self.t, t-self.t, **self.attrs, **attrs)
        self.t = t


@contextmanager
def suppressed():
    """Do not record spans started in this thread (e.g. continuous live vision)."""
    if not ENABLED:
        yield
        return
    _local.suppressed = getattr(_local, 'suppressed', 0)+1
    try:
        yield
    finally:
        _local.suppressed -= 1


@contextmanager
def tags(**values):
    """Attach attributes to every span recorded by this thread inside the block."""
    if not ENABLED:
        yield
        return
    previous = getattr(_local, 'tags', {})
    _local.tags = dict(previous, **values)
    try:
        yield
    finally:
        _local.tags = previous


def write_text(path, text, name, **attrs):
    """``Path(path).write_text(text)``, timed."""
    if not ENABLED:
        return Path(path).write_text(text)
    start = time.monotonic()
    try:
        return Path(path).write_text(text)
    finally:
        recorder.add('disk.write', start, time.monotonic()-start, file=name, bytes=len(text), **attrs)


def write_json(path, value, name, **dumps_options):
    """``Path(path).write_text(json.dumps(value, **dumps_options))``, serialization and write timed apart."""
    if not ENABLED:
        return Path(path).write_text(json.dumps(value, **dumps_options))
    start = time.monotonic()
    text = json.dumps(value, **dumps_options)
    recorder.add('json.dumps', start, time.monotonic()-start, file=name, bytes=len(text))
    return write_text(path, text, name)


def native_request(client, payload, *, record=None, **kwargs):
    """``client.request(payload, **kwargs)`` with round trip and native segments recorded.

    Exactly the caller's keyword arguments are forwarded. ``record`` only
    labels the measurement. The request is serialized once more, only to
    measure JSON serialization and size; that time is booked as overhead.
    """
    if not ENABLED:
        return client.request(payload, **kwargs)
    attrs = dict(record=record)
    started = time.monotonic()
    try:
        # Same serializer call as the client; ASCII output, so length == bytes.
        size = len(json.dumps(payload, allow_nan=False))
    except Exception:
        size = None  # The client raises its own error unchanged below.
    dumps_s = time.monotonic()-started
    recorder.overhead(dumps_s)
    meta = dict(op=payload.get('op'), stage=payload.get('stage'), pipeline=payload.get('pipeline_id'),
                planner=payload.get('planner_id'), request_bytes=size,
                request_dumps_s=dumps_s if size is not None else None, **attrs)
    start = time.monotonic()
    try:
        result = client.request(payload, **kwargs)
    except Exception as error:
        response = getattr(error, 'response', None)
        native = response.pop('native_timing', None) if isinstance(response, dict) else None
        recorder.add('native.request', start, time.monotonic()-start, status='EXCEPTION',
                     error=str(error)[:200], native=native, **meta)
        raise
    elapsed = time.monotonic()-start
    native = result.pop('native_timing', None) if isinstance(result, dict) else None
    recorder.add('native.request', start, elapsed, status=result.get('status') if isinstance(result, dict) else None,
                 native_output_status=result.get('native_output_status') if isinstance(result, dict) else None,
                 native=native, transport_and_worker_s=result.get('transport_and_worker_s') if isinstance(result, dict) else None,
                 **meta)
    return result


def write_later(path, phase, extra=None, since=None):
    """Hand this phase's events to a background writer; returns immediately."""
    if not ENABLED:
        return None
    events, overhead = recorder.take(since)
    payload = dict(schema='eco65_timing_v1', phase=phase, clock='time.monotonic',
                   written_after_phase=True, measurement_overhead_s=overhead,
                   events=events, **(extra or {}))

    def run():
        try:
            Path(path).write_text(json.dumps(payload, default=str))
        except Exception:
            pass
    thread = threading.Thread(target=run, daemon=True, name='timing-writer')
    thread.start()
    return thread
