"""pipelined_audits (2-second planning step 1, 2026-11-22): scheduling only.

The candidates keep their priority order and every lane plans its candidates
exactly as in ``parallel_candidates`` (rank % lanes, fixed worker groups, so
each planning worker's request history and therefore every OMPL result is
reproducible). What changes is when the unchanged sent-path audits run:

* A fixed set of workers is reserved for audits; planning lanes never send
  them a request and audits never use a planning worker.
* Every segment the head candidate (the highest-priority one not yet
  concluded) prepares is split into chunk requests and queued on the audit
  workers at once, instead of after all six segments are planned. When the
  head candidate fails, the next one becomes head and its segments that are
  already prepared are queued.
* The decision is taken exactly as before, once the candidate's planning has
  ended: a planning failure is that candidate's failure (any audit results
  are discarded); otherwise the segments are examined cheapest first and the
  first rejected audit decides (detach/transfer: next detach profile; any
  other stage: the candidate fails), then the holding-capacity post-check.
  The accepted candidate is still the highest-priority one whose plan and
  final audit both pass.

Chunks of one segment are contiguous and share their boundary state, as in
``sent_check``; how many there are only changes scheduling, never what is
checked (the same requests per chunk, the same combination of replies).
"""
import itertools
import math
import queue
import threading
import time

from .. import timing

# Scheduling estimate only (never affects a result): seconds per probe state
# on one worker, measured on the 2026-11-22 base replays (check_path seconds /
# probe states), and the target wall time of one chunk request.
AUDIT_STATE_S = dict(SAMPLED=4.5e-3, ADAPTIVE=4e-5, ADAPTIVE_CONTACT=2.2e-4)
TARGET_CHUNK_S = .25


class _Closed:
    """Queue sentinel that sorts after every task."""


class SegmentAudit:
    """One segment's audit: chunk requests on the audit workers, combined once all replied."""

    def __init__(self, pool, rank, segment, sub, progress):
        from .planner import CHECK_CHUNK_MIN_STATES
        planner = pool.planner
        self.pool, self.rank, self.segment, self.sub = pool, rank, segment, sub
        path = segment['prepared']['collision_path']
        method = planner.audit_method(segment['context'])
        estimate = AUDIT_STATE_S.get(method, AUDIT_STATE_S['SAMPLED'])*len(path)
        most = max(1, min(len(pool.workers), (len(path)-1)//CHECK_CHUNK_MIN_STATES))
        count = max(1, min(most, math.ceil(estimate/TARGET_CHUNK_S)))
        self.method, self.chunks = planner.sent_check_chunks(segment, sub, count)
        self.parts = [None]*count; self.remaining = count; self.running = 0
        self.lock = threading.Lock(); self.done = threading.Event(); self.idle = threading.Event(); self.idle.set()
        self.cancelled = False; self.error = None; self.exception = None; self.started = time.monotonic()
        self.key = (len(path), segment['stage'])
        progress(f"检查 {segment['stage']} 最终发送轨迹：{len(segment['prepared']['points'])} 点，{len(path)} 个检查状态，"
                 f"{count} 段排队于 {len(pool.workers)} 个审计进程（规划完成即开始）")

    def cancel(self):
        """Chunks not yet started are skipped; running ones finish (see wait_idle)."""
        with self.lock:
            self.cancelled = True

    def wait_idle(self):
        self.idle.wait()

    def run(self, k, worker):
        with self.lock:
            skip = self.cancelled or self.pool.stopped()
            if not skip:
                self.running += 1; self.idle.clear()
        if not skip:
            a, b, name, check, extra = self.chunks[k]
            try:
                with timing.span('pipeline.audit_chunk', stage=self.segment['stage'], record=self.segment['record'],
                                 chunk=k, rank=self.rank, states=b-a+1):
                    reply = self.pool.planner.native_call(check, self.sub, name, 600, self.pool.stopped, worker, **extra)
                self.parts[k] = (a, b, reply)
            except Exception as error:
                with self.lock:
                    if self.exception is None:
                        self.exception = error
        with self.lock:
            if not skip:
                self.running -= 1
                if self.running == 0:
                    self.idle.set()
            self.remaining -= 1
            last = self.remaining == 0
        if last:
            self._finish()

    def _finish(self):
        from .planner import SegmentRejected
        try:
            if self.cancelled or self.exception is not None or any(p is None for p in self.parts):
                return
            with timing.span('pipeline.audit_finish', stage=self.segment['stage'], rank=self.rank):
                self.pool.planner.sent_check_finish(self.segment, self.sub, self.method, self.parts, self.started)
        except SegmentRejected as error:
            self.error = error
        except Exception as error:
            self.exception = error
        finally:
            self.done.set()

    def result(self, cancelled):
        """None when the audit passed (segment evidence set); raises its SegmentRejected or failure."""
        while not self.done.wait(.02):
            if cancelled():
                raise ValueError('CANCELLED')
        if self.exception is not None:
            raise self.exception
        if self.cancelled:
            raise RuntimeError('AUDIT_CANCELLED')
        if self.error is not None:
            raise self.error
        return None


class AuditPool:
    """Reserved audit workers, one serving thread each; chunk tasks of the head candidate first."""

    def __init__(self, planner, workers, cancelled):
        self.planner, self.workers, self.outer = planner, list(workers), cancelled
        self.tasks = queue.PriorityQueue(); self.sequence = itertools.count()
        self.stop_event = threading.Event(); self.lock = threading.Lock()
        self.by_record = {}
        # Chunk set-up (probe file, requests) runs here, off the planning lane threads.
        from concurrent.futures import ThreadPoolExecutor
        self.setup = ThreadPoolExecutor(max_workers=1, thread_name_prefix='audit-setup')
        self.threads = [threading.Thread(target=self._serve, args=(w,), daemon=True, name=f'audit-worker-{k}')
                        for k, w in enumerate(self.workers)]
        for thread in self.threads:
            thread.start()

    def stopped(self):
        return self.stop_event.is_set() or bool(self.outer and self.outer())

    def submit(self, rank, segment, sub, progress):
        """Future of the SegmentAudit; set-up and queueing run on the set-up thread."""
        return self.setup.submit(self._submit, rank, segment, sub, progress)

    def _submit(self, rank, segment, sub, progress):
        """Queue the audit of a prepared segment; a newer segment with the same record replaces an older one."""
        key = (str(sub), segment['record'])
        with self.lock:
            previous = self.by_record.get(key)
        if previous is not None and previous.segment is not segment:
            # The same record file names: let the older audit's running chunks end first.
            previous.cancel(); previous.wait_idle()
        audit = SegmentAudit(self, rank, segment, sub, progress)
        with self.lock:
            self.by_record[key] = audit
        for k in range(len(audit.chunks)):
            self.tasks.put(((rank, next(self.sequence)), audit, k))
        return audit

    def _serve(self, worker):
        while True:
            _, audit, k = self.tasks.get()
            if audit is _Closed:
                return
            audit.run(k, worker)

    def close(self):
        self.stop_event.set()
        self.setup.shutdown(wait=True)
        for _ in self.threads:
            self.tasks.put(((math.inf, next(self.sequence)), _Closed, None))
        for thread in self.threads:
            thread.join()


def pipelined_candidates(planner, q, world, box, order, progress, cancelled, failures):
    """``parallel_candidates`` with reserved audit workers and audits queued as segments are prepared."""
    from .planner import SegmentRejected, PlanningBudgetExhausted
    workers = planner.checkers()
    reserved = int(planner.site['pipelined_audit_workers'])
    if reserved >= len(workers):
        raise ValueError(f'PIPELINED_AUDIT_WORKERS_EXCEED_POOL: {reserved} of {len(workers)}')
    planning, auditing = workers[:len(workers)-reserved], workers[len(workers)-reserved:]
    group = int(planner.site.get('lane_clients', 1)) if planner.cycle_policy is not None else 1
    count = max(1, min(int(planner.site.get('parallel_candidate_lanes', 1)), len(planning)//group))
    lanes = [planning[k*group:(k+1)*group] for k in range(count)]
    stop = threading.Event()
    lane_cancelled = lambda: stop.is_set() or bool(cancelled())
    results = {rank: queue.Queue() for rank in range(len(order))}
    commands = {rank: queue.Queue() for rank in range(len(order))}
    pool = AuditPool(planner, auditing, lambda: stop.is_set() or bool(cancelled()))
    state = dict(head=0); lock = threading.Lock()
    prepared = {rank: [] for rank in range(len(order))}      # segments in production order
    audits = {}                                                # id(segment) -> SegmentAudit
    rank_of_sub = {str(order[rank][2]): rank for rank in range(len(order))}
    progress(f'{len(order)} 个候选分配到 {count} 条车道（每条 {group} 个进程）同时规划；'
             f'{len(auditing)} 个进程专用于审计，首位候选每段规划完成即审计')

    def say(rank):
        i = order[rank][0]
        return lambda text: progress(f'检查候选 {i+1}（{rank+1}/{len(order)}）：{text}')

    def on_prepared(sub, segment):
        rank = rank_of_sub.get(str(sub))
        if rank is None:
            return
        with lock:
            prepared[rank].append(segment)
            head = state['head'] == rank
        if head:
            submit(rank, segment)

    def submit(rank, segment):
        with lock:
            if id(segment) not in audits:
                audits[id(segment)] = pool.submit(rank, segment, order[rank][2], say(rank))

    def become_head(rank):
        with lock:
            state['head'] = rank; ready = list(prepared[rank])
        for segment in ready:
            submit(rank, segment)

    def verify(planned, rank):
        """verify_candidate on pipelined audit results: same order, same first rejection, same post-check."""
        segments = [planned[k] for k in planner.segment_keys()]
        pending = [s for s in sorted(segments, key=lambda s: len(s['prepared']['collision_path'])) if s.get('evidence') is None]
        for segment in pending:
            # Normally queued already. An audit cancelled by an earlier rejection
            # of this candidate (a more expensive segment) is queued again.
            with lock:
                future = audits.get(id(segment))
            if future is not None and future.result().cancelled:
                with lock:
                    audits.pop(id(segment), None)
            submit(rank, segment)
        with timing.span('pipeline.wait_audits', rank=rank, pending=len(pending)):
            for segment in pending:
                with lock:
                    future = audits[id(segment)]
                audit = future.result()
                try:
                    audit.result(cancelled)
                except SegmentRejected:
                    # Later (more expensive) segments cannot change this outcome.
                    with lock:
                        futures = [audits[id(x)] for x in pending[pending.index(segment)+1:]]
                    later = [f.result() for f in futures]
                    for other in later:
                        other.cancel()
                    for other in later:
                        other.wait_idle()
                    raise
        capacity = planner.holding_postcheck(planned, order[rank][2])
        for segment in segments:
            segment['evidence']['holding_capacity'] = capacity
        planned['extraction']['evidence'].update(detach_profile=planned['profile'], rejected_detach_profiles=planned['rejected'])
        for segment in segments:
            if segment.get('j1_direction') is not None:
                segment['evidence']['j1_direction'] = segment['j1_direction']
        if planner.cycle_policy is not None:
            planned['empty_return']['evidence'].update(released_carton=planned['empty']['released_carton'],
                                                       withdraw_distance_m=planned['empty']['withdraw_distance_m'])
        return dict(segments=[dict(stage=s['stage'], prepared=s['prepared'], evidence=s['evidence']) for s in segments],
                    attachment=planned['attachment'], release=planned['release'], seal=planned['seal'])

    def lane(index):
        clients = lanes[index]
        for rank in range(index, len(order), count):
            if lane_cancelled():
                results[rank].put(('STOPPED', None)); continue
            i, candidate, sub, screened = order[rank]
            say_lane = lambda text, i=i, rank=rank: progress(f'规划候选 {i+1}（{rank+1}/{len(order)}，车道 {index}）：{text}')
            reuse = None
            while True:
                try:
                    with timing.span('lane.plan_candidate', lane=index, rank=rank, candidate=i, replan=reuse is not None):
                        planned = planner.plan_candidate_segments(q, world, box, candidate, sub, screened, say_lane,
                                                                  lane_cancelled, list(clients), reuse)
                    results[rank].put(('PLANNED', planned))
                except Exception as error:
                    from .planner import TargetBlocked
                    if isinstance(error, TargetBlocked):
                        results[rank].put(('TARGET_BLOCKED', error)); break
                    if isinstance(error, ValueError):
                        results[rank].put(('FAILED', error)); break
                    results[rank].put(('ERROR', error))
                    for later in range(rank+count, len(order), count):
                        results[later].put(('ERROR', error))
                    return
                command = commands[rank].get()
                if command[0] != 'REPLAN':
                    break
                reuse = command[1]

    def receive(rank):
        while True:
            if cancelled():
                raise ValueError('CANCELLED')
            wait = .1
            if planner.deadline is not None:
                left = planner.deadline-time.monotonic()
                if left <= 0:
                    raise PlanningBudgetExhausted('WAITING_FOR_CANDIDATE_LANE', f'rank {rank}')
                wait = min(wait, left)
            try:
                return results[rank].get(timeout=wait)
            except queue.Empty:
                continue

    planner.segment_ready_hook = on_prepared
    threads = [threading.Thread(target=lane, args=(k,), daemon=True, name=f'candidate-lane-{k}') for k in range(count)]
    for thread in threads:
        thread.start()
    try:
        for rank, (i, candidate, sub, screened) in enumerate(order):
            become_head(rank)
            with timing.span('main.wait_planned_candidate', rank=rank, candidate=i):
                kind, value = receive(rank)
            while kind == 'PLANNED':
                try:
                    with timing.span('main.verify_candidate', rank=rank, candidate=i, pipelined=True):
                        result = verify(value, rank)
                except SegmentRejected as error:
                    try:
                        reuse = planner.replan_after(value, error)
                    except SegmentRejected as fatal:
                        kind, value = 'FAILED', fatal; commands[rank].put(('DONE',)); break
                    commands[rank].put(('REPLAN', reuse))
                    with timing.span('main.wait_replanned_candidate', rank=rank, candidate=i):
                        kind, value = receive(rank)
                    continue
                commands[rank].put(('DONE',))
                return dict(result, index=i, candidate=candidate, screen=screened['screen'], lanes=count,
                            audit_workers=len(auditing))
            if kind == 'TARGET_BLOCKED':
                raise value
            if kind == 'ERROR':
                raise value
            if kind == 'FAILED':
                failures.append(dict(target=box['id'], candidate=i, reason=str(value)))
        return None
    finally:
        stop.set()
        for rank in commands:
            commands[rank].put(('STOP',))
        for thread in threads:
            thread.join()
        planner.segment_ready_hook = None
        pool.close()
