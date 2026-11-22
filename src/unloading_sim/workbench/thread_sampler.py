"""Development measurement only: where the planning process's Python threads spend wall time.

Enabled by ``ECO65_PYSAMPLE=1`` together with ``ECO65_TIMING=1``; otherwise
``start`` returns None and nothing runs. A daemon thread samples every
thread's Python stack every ``period_s`` (the interpreter's switch interval
bounds the real rate when the GIL is contended) and reads each thread's CPU
time from ``/proc/self/task/<tid>/stat`` every ``cpu_period_s``. It also
samples the length of a queue given by the caller (the evidence writer).

Each stack sample is classified by its innermost frames: ``WAIT`` (blocked in
a lock, event, queue, join or sleep; the GIL is released), ``NATIVE_IPC``
(a worker request waiting for its reply), ``IO`` (file writes) or ``PYTHON``
(running Python code, or waiting for the GIL to run it). The ``PYTHON``
samples of a thread are attributed to the innermost project function
(``unloading_sim``) and to the innermost frame of any module.

Nothing here changes a decision, a record or a reply; it only reads frames.
"""
from collections import Counter, defaultdict
import os
import sys
import threading
import time

WAIT_FUNCTIONS = {('threading.py', 'wait'), ('threading.py', 'join'), ('threading.py', '_wait_for_tstate_lock'),
                  ('queue.py', 'get'), ('threading.py', 'acquire'), ('concurrent/futures/_base.py', 'result'),
                  ('concurrent/futures/_base.py', 'exception'), ('concurrent/futures/thread.py', '_worker'),
                  ('selectors.py', 'select')}
IPC_FUNCTIONS = {('binary_frames.py', 'replies'), ('binary_frames.py', '_read_exact'), ('moveit2_backend.py', 'read'),
                 ('moveit2_backend.py', 'lines')}


def _group(name):
    """Thread name without its trailing index (lanes of one kind share a group)."""
    base = name.rstrip('0123456789').rstrip('-_')
    return base or name


def _tail(path):
    parts = path.replace('\\', '/').split('/')
    return '/'.join(parts[-3:]) if 'concurrent' in parts else parts[-1]


class ThreadSampler:
    def __init__(self, period_s=.002, cpu_period_s=.05, queue_of=None):
        self.period_s, self.cpu_period_s, self.queue_of = period_s, cpu_period_s, queue_of
        self.stop_event = threading.Event(); self.thread = None
        self.samples = defaultdict(Counter)          # group -> class -> count
        self.project = defaultdict(Counter)          # group -> innermost project function -> PYTHON count
        self.innermost = defaultdict(Counter)        # group -> innermost frame -> PYTHON count
        self.cpu = {}                                # native tid -> (name, first ticks, last ticks)
        self.queue_series = []; self.timeline = []
        self.started = None; self.ticks = os.sysconf('SC_CLK_TCK'); self.sample_count = 0

    def _classify(self, frame):
        inner = frame; project = None; depth = 0; kind = 'PYTHON'
        f = frame
        while f is not None and depth < 60:
            code = f.f_code; tail = _tail(code.co_filename)
            if depth < 4:
                if (tail, code.co_name) in WAIT_FUNCTIONS or (tail.endswith('threading.py') and code.co_name in ('wait', 'join')):
                    kind = 'WAIT'
                elif (tail, code.co_name) in IPC_FUNCTIONS:
                    kind = 'NATIVE_IPC'
                elif code.co_name in ('request',) and tail == 'moveit2_backend.py' and depth == 0:
                    kind = 'NATIVE_IPC'
                elif code.co_name in ('write_text', 'write_bytes', 'savez_compressed') and depth == 0:
                    kind = 'IO'
            if project is None and 'unloading_sim' in code.co_filename:
                project = f'{tail}:{code.co_name}'
            f = f.f_back; depth += 1
        top = f'{_tail(inner.f_code.co_filename)}:{inner.f_code.co_name}'
        return kind, project, top

    def _cpu(self):
        out = {}
        for thread in threading.enumerate():
            tid = getattr(thread, 'native_id', None)
            if tid is None:
                continue
            try:
                with open(f'/proc/self/task/{tid}/stat') as f:
                    fields = f.read().rsplit(')', 1)[1].split()
                out[tid] = (thread.name, int(fields[11])+int(fields[12]))
            except (OSError, IndexError, ValueError):
                continue
        return out

    def _run(self):
        me = threading.get_ident(); next_cpu = 0.
        while not self.stop_event.is_set():
            now = time.monotonic()
            names = {t.ident: t.name for t in threading.enumerate()}
            frames = sys._current_frames()
            running = Counter()
            for ident, frame in frames.items():
                if ident == me:
                    continue
                group = _group(names.get(ident, str(ident)))
                kind, project, top = self._classify(frame)
                self.samples[group][kind] += 1
                if kind == 'PYTHON':
                    running[group] += 1
                    self.project[group][project or '(outside project)'] += 1
                    self.innermost[group][top] += 1
            self.sample_count += 1
            if now >= next_cpu:
                next_cpu = now+self.cpu_period_s
                for tid, (name, ticks) in self._cpu().items():
                    first = self.cpu.get(tid, (name, ticks, ticks))[1]
                    self.cpu[tid] = (name, first, ticks)
                if self.queue_of is not None:
                    try:
                        q = self.queue_of()
                        self.queue_series.append((round(now-self.started, 3), None if q is None else q.qsize()))
                    except Exception:
                        pass
                self.timeline.append((round(now-self.started, 3), dict(running)))
            time.sleep(self.period_s)

    def start(self):
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self._run, daemon=True, name='measurement-thread-sampler')
        self.thread.start(); return self

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(5.)
        wall = time.monotonic()-self.started
        cpu = defaultdict(float)
        for tid, (name, first, last) in self.cpu.items():
            cpu[_group(name)] += (last-first)/self.ticks
        groups = {}
        for group, counts in self.samples.items():
            total = sum(counts.values())
            groups[group] = dict(samples=total, fraction={k: round(v/total, 4) for k, v in counts.items()},
                                 cpu_s=round(cpu.get(group, 0.), 3),
                                 python_project_top=self.project[group].most_common(12),
                                 python_innermost_top=self.innermost[group].most_common(12))
        process_cpu = os.times()
        return dict(schema='eco65_python_thread_sample_v1', scope='DEVELOPMENT_MEASUREMENT_NO_DECISION_EFFECT',
                    wall_s=round(wall, 3), sample_rounds=self.sample_count, nominal_period_s=self.period_s,
                    switch_interval_s=sys.getswitchinterval(), python_thread_cpu_s=round(sum(cpu.values()), 3),
                    process_user_s=process_cpu.user, process_system_s=process_cpu.system,
                    groups=dict(sorted(groups.items(), key=lambda kv: -kv[1]['cpu_s'])),
                    evidence_queue_series=self.queue_series, running_timeline=self.timeline)


def start(queue_of=None):
    """A running sampler when ECO65_PYSAMPLE=1 and ECO65_TIMING=1, else None."""
    if os.environ.get('ECO65_PYSAMPLE') != '1' or os.environ.get('ECO65_TIMING') != '1':
        return None
    return ThreadSampler(queue_of=queue_of).start()
