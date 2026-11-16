"""Resident native workers for the ECO65 physical single-carton chain (realtime step 4).

Selected only by ``resident_workers: true`` in the single-carton config; the
default keeps the per-plan ResidentMoveItClient start and close unchanged.

* ``CancellableWorkerClient``: the existing JSONL stream plus out-of-band
  cancellation by request id (worker started with M710_WORKER_CANCEL=1). A
  cancelled or timed-out request no longer kills the process: the worker
  expires that request's NativeDeadline, replies, and the reply is consumed;
  the client stays usable. A worker that does not reply within
  CANCEL_GRACE_S is declared stuck, killed, and never given another request.
* ``WorkerPool``: the main worker and the checker pool start and initialise
  once in the background and are kept between tasks. Every task starts from
  ``sc_task_reset`` (fresh scene, counters, OMPL pipeline and RNG seed): each
  worker is reset right after its init and again after every task, so the
  state at every task start is the same post-reset state. A worker that fails
  its init or reset is retired and replaced in the background.

No hardware is touched by any of this; the workers only plan and check.
"""
from concurrent.futures import ThreadPoolExecutor
import itertools
import json
import os
import queue
import shlex
import subprocess
import threading
import time

from ...moveit2_backend import MoveItUnavailable

CANCEL_GRACE_S = 5.
TASK_SEED = 6518


class WorkerRetired(MoveItUnavailable):
    """This worker was closed and is never given another request."""


class CancellableWorkerClient:
    """One serialized request stream with request-id cancellation."""

    def __init__(self, command, *, log_path=None, name='worker'):
        self.name = name; self._ids = itertools.count(1); self._lock = threading.Lock()
        self._replies = queue.Queue(); self.retired = None; self.cancelled_requests = 0; self.clean = False
        self._log = open(log_path, 'a', encoding='utf-8') if log_path else None
        env = dict(os.environ, M710_WORKER_CANCEL='1')
        try:
            self.process = subprocess.Popen(shlex.split(command) if isinstance(command, str) else command,
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._log,
                                            text=True, encoding='utf-8', bufsize=1, env=env)
        except OSError as exc:
            if self._log: self._log.close()
            raise MoveItUnavailable(f'MOVEIT2_UNAVAILABLE: {exc}') from exc

        def read():
            try:
                for line in self.process.stdout:
                    self._replies.put(json.loads(line))
            except Exception as exc:
                self._replies.put(exc)
            finally:
                self._replies.put(MoveItUnavailable('MOVEIT2_WORKER_EXITED'))
        threading.Thread(target=read, daemon=True, name=f'{name}-reader').start()

    def alive(self):
        return self.retired is None and self.process.poll() is None

    def retire(self, reason):
        if self.retired is None:
            self.retired = reason
        threading.Thread(target=self._kill, daemon=True, name=f'{self.name}-kill').start()

    def _kill(self):
        if self.process.poll() is None:
            self.process.kill(); self.process.wait()
        if self._log and not self._log.closed: self._log.close()

    def _send(self, payload):
        self.process.stdin.write(json.dumps(payload, allow_nan=False)+'\n'); self.process.stdin.flush()

    def request(self, payload, *, timeout=120., cancelled=None):
        with self._lock:
            if not self.alive():
                raise WorkerRetired(f'WORKER_RETIRED:{self.retired or "WORKER_EXITED"}')
            self.clean = False
            request = {**payload, 'request_id': f'{self.name}-{next(self._ids)}'}
            started = time.perf_counter(); cancel_sent = None; why = None
            try:
                self._send(request)
            except (BrokenPipeError, OSError) as exc:
                self.retire('WORKER_EXITED'); raise WorkerRetired(f'WORKER_RETIRED:WORKER_EXITED:{exc}') from exc
            while True:
                now = time.perf_counter()
                if cancel_sent is None:
                    if cancelled is not None and cancelled(): why = 'CANCELLED'
                    elif now-started >= timeout: why = 'MOVEIT2_REQUEST_TIMEOUT'
                    if why:
                        cancel_sent = now
                        try: self._send(dict(op='cancel', target_request_id=request['request_id']))
                        except (BrokenPipeError, OSError):
                            self.retire('WORKER_EXITED'); raise WorkerRetired('WORKER_RETIRED:WORKER_EXITED')
                elif now-cancel_sent >= CANCEL_GRACE_S:
                    self.retire('CANCEL_NOT_ACKNOWLEDGED')
                    raise MoveItUnavailable(why)
                try:
                    result = self._replies.get(timeout=.02)
                except queue.Empty:
                    continue
                if isinstance(result, Exception):
                    self.retire('WORKER_EXITED'); raise WorkerRetired(f'WORKER_RETIRED:WORKER_EXITED:{result}')
                if result.get('request_id') != request['request_id']:
                    self.retire('PROTOCOL_ERROR'); raise WorkerRetired('WORKER_RETIRED:PROTOCOL_ERROR')
                result['transport_and_worker_s'] = time.perf_counter()-started
                if cancel_sent is not None:
                    # The worker stopped this request and replied: the process stays usable.
                    self.cancelled_requests += 1
                    error = MoveItUnavailable(why); error.response = result
                    raise error
                if result.get('status') == 'ERROR':
                    error = MoveItUnavailable(result.get('detail', 'MOVEIT2_ERROR')); error.response = result
                    raise error
                return result

    def reset(self, identity, seed=TASK_SEED):
        reply = self.request(dict(op='sc_task_reset', identity=identity, seed=seed), timeout=60)
        if reply.get('status') != 'RESET':
            raise MoveItUnavailable('TASK_RESET_FAILED:'+str(reply)[:200])
        self.clean = True
        return reply

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.close()
            except OSError:
                pass
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait()
        if self._log and not self._log.closed: self._log.close()
        if self.retired is None: self.retired = 'CLOSED'


REQUIRED_CAPABILITIES = ('bounded_single_carton_planning', 'resident_task_reset', 'request_cancel')


class WorkerPool:
    """Main worker plus checker pool, started once and kept between tasks."""

    def __init__(self, command_for, init, checkers, log_dir):
        self.command_for, self.init, self.count, self.log_dir = command_for, init, checkers, log_dir
        self.lock = threading.Lock(); self.main = None; self.main_ready = None; self.pool = []
        self.status = []; self.ready = threading.Event(); self.closed = False; self.thread = None
        self.background = None; self.serial = itertools.count()

    # -- startup ---------------------------------------------------------------
    def start(self):
        """Start, initialise and reset every worker in the background."""
        with self.lock:
            if self.thread is not None:
                return
            self.thread = threading.Thread(target=self._start_all, daemon=True, name='resident-pool-start')
        self.thread.start()

    def _start_one(self, role):
        name = f'{role}-{next(self.serial)}'
        status = dict(name=name, role=role, state='STARTING', started_monotonic_s=time.monotonic())
        with self.lock: self.status.append(status)
        client = None
        try:
            client = CancellableWorkerClient(self.command_for(name), log_path=self.log_dir/f'resident-{name}.log', name=name)
            ready = client.request(self.init, timeout=180)
            capabilities = ready.get('capabilities', {})
            missing = [c for c in REQUIRED_CAPABILITIES if not capabilities.get(c)]
            if missing:
                raise RuntimeError('NATIVE_WORKER_CAPABILITY_MISSING:'+','.join(missing))
            client.reset(self.init['identity'])
            status.update(state='READY', init_and_reset_s=time.monotonic()-status['started_monotonic_s'])
            return client, ready
        except Exception as error:
            status.update(state='FAILED', error=str(error)[:300])
            if client is not None: client.retire('INIT_FAILED')
            return None, None

    def _start_all(self):
        with ThreadPoolExecutor(max_workers=1+self.count, thread_name_prefix='resident-start') as executor:
            main = executor.submit(self._start_one, 'main')
            checks = [executor.submit(self._start_one, 'check') for _ in range(self.count)]
            self.main, self.main_ready = main.result()
            started = [f.result()[0] for f in checks]
        with self.lock:
            if self.closed:
                for c in [self.main]+started:
                    if c is not None: c.close()
                return
            self.pool = [c for c in started if c is not None]
        self.ready.set()

    def wait_ready(self, timeout):
        if not self.ready.wait(timeout):
            raise MoveItUnavailable('RESIDENT_POOL_NOT_READY_WITHIN_TIMEOUT')
        if self.main is None:
            raise MoveItUnavailable('RESIDENT_MAIN_WORKER_UNAVAILABLE: '+json.dumps(self.status, ensure_ascii=False, default=str)[:600])
        return self.main_ready

    # -- task boundaries ---------------------------------------------------------
    def _reset_all(self, clients):
        def one(client):
            started = time.monotonic()
            try:
                reply = client.reset(self.init['identity'])
                return dict(worker=client.name, state='RESET', reset_s=time.monotonic()-started, ompl_seed=reply.get('ompl_seed_reapplied'))
            except Exception as error:
                client.retire('RESET_FAILED')
                return dict(worker=client.name, state='RETIRED', reason='RESET_FAILED', error=str(error)[:300])
        if not clients:
            return []
        with ThreadPoolExecutor(max_workers=len(clients), thread_name_prefix='resident-reset') as executor:
            return list(executor.map(one, clients))

    def begin_task(self):
        """Every live worker in the post-reset state; returns the evidence of this task start."""
        background = self.background
        if background is not None:
            background.join()
        clients = self.live()
        stale = [c for c in clients if not c.clean]
        report = dict(already_reset=[c.name for c in clients if c.clean], reset_now=self._reset_all(stale),
                      retired=[dict(worker=c.name, reason=c.retired) for c in self.all() if not c.alive()])
        if self.main is None or not self.main.alive():
            raise MoveItUnavailable('RESIDENT_MAIN_WORKER_RETIRED: '+json.dumps(report, ensure_ascii=False, default=str)[:600])
        return report

    def end_task(self):
        """After a task: reset every live worker and replace retired ones, in the background."""
        def run():
            self._reset_all([c for c in self.live() if not c.clean])
            self._replenish()
        self.background = threading.Thread(target=run, daemon=True, name='resident-end-task')
        self.background.start()

    def _replenish(self):
        with self.lock:
            if self.closed:
                return
            main_dead = self.main is None or not self.main.alive()
            dead = [c for c in self.pool if not c.alive()]
            self.pool = [c for c in self.pool if c.alive()]
        if main_dead:
            client, ready = self._start_one('main')
            if client is not None:
                with self.lock: self.main, self.main_ready = client, ready
        for _ in dead:
            client, _ready = self._start_one('check')
            if client is not None:
                with self.lock: self.pool.append(client)

    # -- access --------------------------------------------------------------------
    def all(self):
        with self.lock:
            return [c for c in [self.main]+list(self.pool) if c is not None]

    def live(self):
        return [c for c in self.all() if c.alive()]

    def checkers(self):
        main = self.main
        return [c for c in self.live() if c is not main]

    def snapshot(self):
        with self.lock:
            return [dict(s) for s in self.status]

    def close(self):
        with self.lock:
            self.closed = True; clients = [c for c in [self.main]+list(self.pool) if c is not None]; self.pool = []
        for client in clients:
            threading.Thread(target=client.close, daemon=True, name='resident-close').start()
