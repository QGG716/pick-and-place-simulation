"""planning_module NATIVE (realtime step 7, docs/native_boundary.md contract A).

Endpoint screening of all candidates is one composite request
(``sc_endpoint_screen``) and every candidate's four segments are two
(``sc_plan_candidate`` CONTACT, then DETACH_AND_VERIFY with the in-worker
sent-path checks), instead of hundreds of round trips. The worker runs the
same branching as the Python flow (ros2/.../sc_composite.h) with request
templates that Python builds with its own functions, prepares the sent
trajectories in C++ (native_prepare.h) and records every sub-request and
result. Python then runs its original, unchanged code (``screen_candidate``,
``plan_candidate``) against a ``ReplayClient`` that serves those records:

* every request the Python code issues must equal a recorded one (volatile
  budget/deadline fields and the world excepted; the scene fingerprint is
  compared), otherwise NATIVE_FLOW_DIVERGED is raised; nothing falls back;
* every decision, failure message and evidence file is produced by the
  unchanged Python code, so the outcome equals planning_module PYTHON;
* ``prepare`` results come from the worker (C++), not from Python.

Composite requests are persisted before dispatch; each worker appends every
internal attempt to its own journal (path recorded in the composite record).
"""
from collections import defaultdict, deque
import copy
from dataclasses import asdict
import json
import queue
import threading
import time

import numpy as np

from ... import moveit2_backend
from .. import timing
from .geometry import candidate_contact_check, corners, rigid

VOLATILE = ('request_id', 'request_budget_s', 'absolute_deadline_monotonic_s', 'deadline_clock_reference_monotonic_s', 'world')
COMPOSITE_TIMEOUT_S = 3600.


class NativeFlowDiverged(RuntimeError):
    """The Python flow asked for a native request the composite did not issue (or vice versa)."""


def _default(x):
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.floating):
        return float(x)
    raise TypeError(type(x).__name__)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False, default=_default)


def replay_key(request, deadline):
    """Canonical request identity; a probe path by its length and end states (the worker echoes the same)."""
    drop = set(VOLATILE) | ({'allowed_planning_time_s'} if deadline else set())
    key = {k: v for k, v in request.items() if k not in drop}
    path = key.get('probe_path')
    if isinstance(path, list):
        key['probe_path'] = dict(length=len(path), first=list(path[0]), last=list(path[-1]))
    return canonical(key)


def prepare_key(points, time_scale, period, reach):
    return canonical([points, float(time_scale), float(period), float(reach)])


class ReplayClient:
    """Serves one composite's recorded native results to the unchanged Python flow."""

    def __init__(self, target, deadline):
        self.target = target; self.deadline = deadline; self.lock = threading.Lock()
        self.calls = defaultdict(deque); self.prepares = {}; self.stopped = None; self.served = 0; self.recorded = 0

    def add(self, reply):
        for call in reply.get('calls', []):
            self.calls[replay_key(call['request'], self.deadline)].append(call); self.recorded += 1
        for p in reply.get('prepares', []):
            self.prepares[prepare_key(p['points'], p['time_scale'], p['period_s'], p['reach_bound_m'])] = p
        if reply.get('stopped'):
            self.stopped = reply['stopped']

    def request(self, payload, *, timeout=120., cancelled=None):
        key = replay_key(payload, self.deadline)
        with self.lock:
            pending = self.calls.get(key)
            call = pending.popleft() if pending else None
            if call is not None:
                self.served += 1
        if call is None:
            if self.stopped and 'BUDGET' in self.stopped:
                raise moveit2_backend.MoveItUnavailable('NATIVE_ABSOLUTE_DEADLINE_EXPIRED')
            if self.stopped == 'CANCELLED':
                raise moveit2_backend.MoveItUnavailable('CANCELLED')
            raise NativeFlowDiverged(f"NATIVE_FLOW_DIVERGED: no recorded native result for {payload.get('op')} "
                                     f"{payload.get('stage')} (composite stopped: {self.stopped})")
        result = copy.deepcopy(call['result'])
        result['transport_and_worker_s'] = call.get('elapsed_s')
        if result.get('status') == 'ERROR':
            error = moveit2_backend.MoveItUnavailable(result.get('detail', 'MOVEIT2_ERROR')); error.response = result
            raise error
        return result

    def prepare_segment(self, points, limits, time_scale, period, reach):
        entry = self.prepares.get(prepare_key(points, time_scale, period, reach))
        if entry is None:
            raise NativeFlowDiverged('NATIVE_FLOW_DIVERGED: no native prepare result for this segment')
        if entry['status'] != 'PREPARED':
            raise ValueError(entry['error'])
        p = entry['prepared']
        return dict(schema='eco65_timed_send_v1', period_s=period, time_scale=time_scale, source_points=points,
                    source_duration_s=p['source_duration_s'], scaled_duration_s=p['scaled_duration_s'],
                    send_duration_s=p['send_duration_s'],
                    points=[dict(t=t, q=q, v=v, a=a) for t, q, v, a in zip(p['t'], p['q'], p['v'], p['a'])],
                    collision_path=p['collision_path'], limits=asdict(limits),
                    collision_sampling=dict(reach_bound_m=reach, analytic_speed_bounds_rad_s=p['analytic_speed_bounds_rad_s'],
                                            maximum_pair_displacement_between_checks_m=.001,
                                            subdivisions_per_send=p['subdivisions_per_send']),
                    interpolation='quintic_q_v_a_nonuniform_knots', terminal_hold_s=p['terminal_hold_s'])

    def unused(self):
        with self.lock:
            return sum(len(v) for v in self.calls.values())

    def close(self):
        pass


class NativeFlow:
    def __init__(self, planner):
        self.p = planner

    # -- shared pieces ---------------------------------------------------------------
    def common(self):
        p = self.p
        deadline = None if p.deadline is None else dict(deadline_monotonic_s=p.deadline, reply_reserve_s=.1,
                                                        reference_monotonic_s=p.deadline_reference)
        from .planner import SEND_SLOWDOWN_STEPS
        return dict(identity=p.init['identity'], deadline=deadline, check_workers=len(p.checkers()),
                    sent_path_audit=p.site.get('sent_path_audit', 'SAMPLED'), path_endpoint_reuse=p.enabled('path_endpoint_reuse'),
                    scale=p.site['time_scale'],
                    prepare=dict(limits=asdict(p.limits), period_s=p.config['period_s'], reach_bound_m=p.reach_bound,
                                 slowdown_steps=list(SEND_SLOWDOWN_STEPS)))

    def handle(self, request, client, folder):
        return self.p.with_scene_handle(request, client, folder) if self.p.enabled('scene_handles') else request

    def send(self, client, request, folder, name, cancelled=None):
        """Persist, send and record one composite request (its sub-calls are replayed later)."""
        p = self.p
        request = dict(request, request_budget_s=COMPOSITE_TIMEOUT_S if p.deadline is None else
                       max(1e-3, p.deadline-.1-time.monotonic()))
        path = folder/(name+'-composite.json')
        timing.write_json(path, dict(status='PENDING', request=request), 'composite-record:pending')
        started = time.monotonic()
        try:
            reply = timing.native_request(client, request, timeout=COMPOSITE_TIMEOUT_S, cancelled=cancelled, record=name)
        except Exception as error:
            p.write_evidence(path, dict(status='REQUEST_FAILED', request=request, error=str(error),
                                        elapsed_s=time.monotonic()-started), 'composite-record:final')
            raise
        summary = {k: v for k, v in reply.items() if k not in ('calls', 'prepares')}
        summary.update(calls=[dict(index=c['index'], name=c['name'], op=c['request'].get('op'), stage=c['request'].get('stage'),
                                   status=c['result'].get('status'), elapsed_s=c['elapsed_s']) for c in reply.get('calls', [])],
                       prepares=len(reply.get('prepares', [])), binary_frames=getattr(client, 'binary_active', False))
        p.write_evidence(path, dict(status='REPLIED', request=request, reply=summary, elapsed_s=time.monotonic()-started),
                         'composite-record:final')
        return reply

    # -- endpoint screening ----------------------------------------------------------
    def screen_request(self, q, world, box, candidates, considered, client, folder):
        p = self.p
        tool = sorted(p.clearance.get('tool_links', [])); ids = sorted({o['id'] for o in world})
        supports = p.detach_supports(world, box)
        rows = []
        for i in considered:
            candidate = candidates[i]
            tcp = rigid(candidate['tcp_pose']); flange = tcp@np.linalg.inv(p.coordinate_model.tool.matrix)
            attachment = np.linalg.inv(flange)@rigid(box['pose'])
            try:
                release = p.release_check(q, attachment, box, world)
            except ValueError:
                rows.append(dict(index=i, skip='RELEASE_CHECK_REJECTED')); continue
            release_supports = [dict(id=o['id'], top_world_m=p.object_top(o)) for o in world if o['id'] in release['region']['ids']]
            loaded = dict(p.context(q, world, box, candidate, 'return', attachment, release_supports), op='validate')
            checks = {stage: dict(p.context(q, world, box, candidate, stage, attached, stage_supports), op='validate')
                      for stage, attached, stage_supports in (('approach', None, ()), ('contact', None, ()),
                                                               ('detach', attachment, supports))}
            template = dict(op='ik', identity=p.init['identity'], seed=6518, planning_only=False,
                            position_tolerance_m=2e-5, orientation_tolerance_rad=2e-4, request_budget_s=110.)
            rows.append(dict(index=i, loaded_start=self.handle(loaded, client, folder),
                             approach_check=self.handle(checks['approach'], client, folder),
                             contact_check=self.handle(checks['contact'], client, folder),
                             detach_check=self.handle(checks['detach'], client, folder),
                             approach_ik=dict(template, goal_pose=np.asarray(candidate['precontact_pose']).tolist()),
                             contact_ik=dict(template, goal_pose=np.asarray(candidate['tcp_pose']).tolist()),
                             tool_links=tool, box_id=box['id'], world_ids=ids))
        lower, upper, _, _ = p.limits.arrays()
        return dict(self.common(), op='sc_endpoint_screen', q=list(map(float, q)), candidates=rows,
                    limits_lower=lower.tolist(), limits_upper=upper.tolist(), spread_seeds=p.spread_seeds())

    def screen(self, q, world, box, candidates, considered, folder):
        """One composite for every candidate; returns the replay client for the unchanged screening loop."""
        p = self.p; client = p.client
        with timing.span('native.composite_screen', candidates=len(considered)):
            reply = self.send(client, self.screen_request(q, world, box, candidates, considered, client, folder), folder,
                              box['id']+'-endpoint-screen')
        replay = ReplayClient(client, p.deadline); replay.add(reply)
        return replay

    # -- one candidate ---------------------------------------------------------------
    def plan_one(self, q, world, box, candidate, sub, screened, progress, cancelled, client, checkers):
        """Both composites for one candidate on ``client``, then the unchanged plan_candidate on their records."""
        p = self.p; scale = p.site['time_scale']; tcp = rigid(candidate['tcp_pose'])
        replay = ReplayClient(client, p.deadline)
        known = screened['screen'].get('approach_q')
        approach_context = p.context(q, world, box, candidate, 'approach')
        contact_context = p.context(q if known is None else list(known), world, box, candidate, 'contact')
        phase = dict(contact=dict(request=self.handle(p.plan_request(contact_context, tcp, True), client, sub),
                                  context=self.handle(contact_context, client, sub)))
        if known is None:
            phase['approach'] = dict(request=self.handle(p.plan_request(approach_context, candidate['precontact_pose'], False), client, sub),
                                     context=self.handle(approach_context, client, sub))
        else:
            phase['known_approach_q'] = list(known)
        progress('原生复合请求：接近 / 接触')
        with timing.span('native.composite_contact'):
            first = self.send(client, dict(self.common(), op='sc_plan_candidate', phase='CONTACT', contact_phase=phase), sub, 'contact-phase',
                              cancelled)
        replay.add(first)
        detach = self.detach_request(q, world, box, candidate, screened, first, approach_context, client, sub) \
            if first.get('outcome') == 'PLANNED' and not first.get('stopped') else None
        if detach is not None:
            progress('原生复合请求：脱离 / 带载返回 / 发送路径检查')
            with timing.span('native.composite_detach_verify'):
                replay.add(self.send(client, detach, sub, 'detach-phase', cancelled))
        with timing.span('native.replay_plan_candidate'):
            result = p.plan_candidate(q, world, box, candidate, sub, screened, progress, cancelled,
                                      clients=[replay], workers=[replay]*max(1, len(checkers)))
        if replay.unused():
            raise NativeFlowDiverged(f'NATIVE_FLOW_DIVERGED: {replay.unused()} recorded native results not requested by the Python flow')
        return result

    def detach_request(self, q, world, box, candidate, screened, first, approach_context, client, sub):
        """DETACH_AND_VERIFY inputs, computed exactly as plan_candidate_segments does (None when it would stop)."""
        p = self.p
        known = screened['screen'].get('approach_q')
        # The accepted plan of each contact-phase segment (one SUCCESS per stage)
        # and the worker's prepared trajectory of exactly its points.
        segments = {}
        accepted = [c for c in first['calls'] if c['request'].get('op') == 'plan' and c['result'].get('status') == 'SUCCESS']
        for name in (['approach', 'contact'] if known is None else ['contact']):
            plan = next(c for c in accepted if c['request'].get('stage') == name)
            segments[name] = next(e for e in first['prepares']
                                  if e['status'] == 'PREPARED' and e['points'] == plan['result']['points'])['prepared']
        qpre = list(known) if known is not None else segments['approach']['q'][-1]
        qc = segments['contact']['q'][-1]
        # Exactly plan_candidate_segments: seal on the actual contact FK, attachment, release.
        actual = p.coordinate_model.tcp(qc)
        if not candidate_contact_check(actual, candidate)['valid']:
            return None  # the replay raises ACTUAL_NATIVE_CONTACT_FK_SEAL_REJECTED
        attachment = np.linalg.inv(actual@np.linalg.inv(p.coordinate_model.tool.matrix))@rigid(box['pose'])
        try:
            p.release_check(q, attachment, box, world)
        except ValueError:
            return None  # the replay raises the same error
        original = p.detach_supports(world, box)
        profiles = []
        for name, goal in p.detach_goals(actual, box, candidate):
            context = p.context(qc, world, box, candidate, 'detach', attachment, original)
            profiles.append(dict(name=name, record='detach' if name == 'OUTWARD' else 'detach-'+name.lower(),
                                 request=self.handle(p.plan_request(context, goal, True), client, sub),
                                 context=self.handle(context, client, sub)))
        return_context = p.context(q, world, box, candidate, 'return', attachment, screened['release_supports'])
        a = return_context['attachment']
        phase = {'q': list(map(float, q)), 'payload_reach_m': float(np.linalg.norm(corners(a['pose'], a['size']), axis=1).max()),
                 'box_id': box['id'], 'world_ids': sorted({o['id'] for o in world}), 'profiles': profiles,
                 'return': dict(request=self.handle(p.plan_request(return_context, q, False), client, sub),
                                context=self.handle(return_context, client, sub)),
                 'contact_planned': dict(context=self.handle(p.context(qpre, world, box, candidate, 'contact'), client, sub),
                                         prepared=segments['contact'])}
        if known is None:
            phase['approach_planned'] = dict(context=self.handle(approach_context, client, sub), prepared=segments['approach'])
        else:
            phase['approach'] = dict(request=self.handle(p.plan_request(approach_context, list(known), False), client, sub),
                                     context=self.handle(approach_context, client, sub))
            phase['qpre'] = qpre
        return dict(self.common(), op='sc_plan_candidate', phase='DETACH_AND_VERIFY', detach_phase=phase)

    # -- candidate lanes ---------------------------------------------------------------
    def plan_ranks(self, q, world, box, order, progress, cancelled, failures):
        """Candidates on worker lanes (rank % lanes), accepted strictly in priority order."""
        p = self.p
        lanes = p.checkers()[:max(1, int(p.site.get('parallel_candidate_lanes', 1)))] or [p.client]
        count = len(lanes); stop = threading.Event(); checkers = p.checkers()
        results = {rank: queue.Queue() for rank in range(len(order))}
        lane_cancelled = lambda: stop.is_set() or bool(cancelled())
        progress(f'{len(order)} 个候选分配到 {count} 个原生进程，每个候选两个复合请求；按优先级取第一个完整通过的候选')
        from .planner import TargetBlocked, PlanningBudgetExhausted

        def lane(index):
            client = lanes[index]
            for rank in range(index, len(order), count):
                if lane_cancelled():
                    results[rank].put(('STOPPED', None)); continue
                i, candidate, sub, screened = order[rank]
                say = lambda text, i=i, rank=rank: progress(f'规划候选 {i+1}（{rank+1}/{len(order)}，进程 {index}）：{text}')
                try:
                    with timing.span('lane.native_plan_candidate', lane=index, rank=rank, candidate=i):
                        results[rank].put(('VERIFIED', self.plan_one(q, world, box, candidate, sub, screened, say, lane_cancelled,
                                                                     client, checkers)))
                except TargetBlocked as e:
                    results[rank].put(('TARGET_BLOCKED', e))
                except ValueError as e:
                    results[rank].put(('FAILED', e))
                except Exception as e:
                    results[rank].put(('ERROR', e))
                    for later in range(rank+count, len(order), count):
                        results[later].put(('ERROR', e))
                    return
        threads = [threading.Thread(target=lane, args=(k,), daemon=True, name=f'native-lane-{k}') for k in range(count)]
        for t in threads:
            t.start()
        try:
            for rank, (i, candidate, sub, screened) in enumerate(order):
                while True:
                    if cancelled():
                        raise ValueError('CANCELLED')
                    if p.deadline is not None and time.monotonic() >= p.deadline:
                        raise PlanningBudgetExhausted('WAITING_FOR_CANDIDATE_LANE', f'rank {rank}')
                    try:
                        kind, value = results[rank].get(timeout=.1); break
                    except queue.Empty:
                        continue
                if kind == 'VERIFIED':
                    return dict(value, index=i, candidate=candidate, screen=screened['screen'], lanes=count)
                if kind in ('TARGET_BLOCKED', 'ERROR'):
                    raise value
                if kind == 'FAILED':
                    failures.append(dict(target=box['id'], candidate=i, reason=str(value)))
            return None
        finally:
            stop.set()
            for t in threads:
                t.join()
