"""Continuous multi-carton job (AGENTS November 16 section 6; user decisions 2026-11-26).

One operator click starts one job. Each round is the unchanged single-carton
cycle of ``single_carton.runtime.TaskManager`` (its own task record, frozen
observation, plan, final audits, pressure gates and stop semantics). The job
only decides when the next round may start, which targets it searches and in
which order, and when the job ends. Nothing is carried from one round to the
next: every round takes a new stationary frozen RGB-D after the previous
release and plans from it.

User decisions of 2026-11-26 (with the request "当前的流程要改造成多箱连续抓放"):
- ``receiver_clearing``: "落箱默认会被自动清空" -> USER_CONFIRMED_SELF_CLEARING;
  ``clearing_time_s`` is the implementer's value pending the user's figure.
- ``plan_validity_s``: "30 s" (frozen exposure receipt to motion start; the
  section 6 default of 5 s is superseded for this entry).
- ``target_order``: "先高后低，同一层里先中间后两边" -> TOP_LAYER_FIRST_CENTRE_OUT.

The job never moves anything by itself: motion starts only through
``TaskManager.execute`` with every check that the single-carton execute button
has. Tests run with fakes; physical acceptance stays NOT_RUN.
"""
from __future__ import annotations

import json
import math
import threading
import time
import uuid
from pathlib import Path

import numpy as np


RECEIVER_MODES = ('USER_CONFIRMED_SELF_CLEARING', 'OPERATOR_CONFIRM_EACH', 'SENSOR_OBSERVED_CLEAR')
ORDER_MODES = ('TOP_LAYER_FIRST_CENTRE_OUT',)
DEFAULT_ORDER_BASIS = 'User decision 2026-11-26 "先高后低，同一层里先中间后两边"'
DEFAULT_VALIDITY_BASIS = 'User decision 2026-11-26 "30 s": continuous job, frozen exposure receipt to motion start'
# A carton is at least 100 mm high on this site (configured prior); half of
# it separates layers and decides "resting on top" (implementer's value).
DEFAULT_LAYER_TOLERANCE_M = 0.05
# Footprint overlap below this is treated as touching, not resting on (implementer's value).
FOOTPRINT_OVERLAP_MIN_M = 0.01
# Hidden extent behind an unresolved face, as the scene's neighbour extrusion (AGENTS 11-19: 0.40 m).
DEFAULT_HIDDEN_EXTENT_M = 0.40

END_REASONS = {
    'OPERATOR_STOP': '操作员停止',
    'CARTON_COUNT_REACHED': '已达到本次作业箱数',
    'NO_TARGET_DETECTED': '未检测到箱子（不代表现场已空）',
    'NO_PLANNABLE_TARGET': '检测到箱子，但没有可完整测量的待抓箱',
    'NO_ELIGIBLE_TARGET': '可测箱都被上方箱子压住，不能先抓',
    'NO_COMPLETE_PLAN': '所有候选箱都没有完整计划',
    'PLAN_EXPIRED_REPEATEDLY': '计划多次在运动前过期',
    'CYCLE_NOT_COMPLETE': '本轮循环未完成',
    'RELEASE_NOT_CONFIRMED': '本轮释放未经压力确认',
    'LOAD_MAY_BE_HELD': '吸具可能仍持箱',
    'SESSION_CHANGED': '设备会话已变化',
    'DEVICE_DATA_INVALID': '设备数据无效',
    'START_MOVED': '本轮起点偏离作业首轮起点',
    'OBSERVATION_FAILED': '观测失败',
    'OBSERVATION_TIMEOUT': '等待新观测超时',
    'EXECUTION_REFUSED': '执行前检查拒绝',
    'RECORD_WRITE_FAILED': '作业记录写入失败',
    'DRY_RUN_NO_RELEASE': '空跑模式不吸箱，箱子仍在原处，不进行下一轮',
    'JOB_ERROR': '作业线程异常',
}


def settings(config):
    """Validated ``continuous_job`` block, or a ValueError naming what is missing."""
    value = (config or {}).get('continuous_job')
    if not isinstance(value, dict):
        raise ValueError('CONTINUOUS_JOB_REQUIRED: 配置缺少 continuous_job（receiver_clearing 及依据为必填）')
    unknown = set(value) - {'max_cartons', 'receiver_clearing', 'plan_validity_s', 'plan_validity_basis',
                            'target_order', 'max_plan_expiries', 'observation_timeout_s'}
    if unknown:
        raise ValueError('CONTINUOUS_JOB_UNKNOWN_KEYS:' + ','.join(sorted(unknown)))
    count = value.get('max_cartons', 1)
    if type(count) is not int or not 1 <= count <= 200:
        raise ValueError('CONTINUOUS_JOB_MAX_CARTONS_INVALID: 1-200')
    receiver = value.get('receiver_clearing')
    if not isinstance(receiver, dict) or receiver.get('mode') not in RECEIVER_MODES or not str(receiver.get('basis', '')).strip():
        raise ValueError('RECEIVER_CLEARING_POLICY_REQUIRED: receiver_clearing={mode: ' + '|'.join(RECEIVER_MODES) + ', basis}')
    if receiver['mode'] == 'SENSOR_OBSERVED_CLEAR':
        # Needs a measured receiver whose region is in view; none is configured on this site.
        raise ValueError('SENSOR_OBSERVED_CLEAR_UNAVAILABLE: 现场没有在视野内的实测承接区')
    clearing = receiver.get('clearing_time_s')
    if receiver['mode'] == 'USER_CONFIRMED_SELF_CLEARING':
        if type(clearing) not in (int, float) or not math.isfinite(clearing) or not 0 <= clearing <= 120:
            raise ValueError('RECEIVER_CLEARING_TIME_INVALID: 0-120 s')
    validity = value.get('plan_validity_s', 30.)
    validity_basis = value.get('plan_validity_basis', DEFAULT_VALIDITY_BASIS)
    if type(validity) not in (int, float) or not math.isfinite(validity) or not 0 < validity <= 30:
        raise ValueError('CONTINUOUS_PLAN_VALIDITY_INVALID: 0-30 s (user decision 2026-11-26)')
    if not str(validity_basis).strip():
        raise ValueError('CONTINUOUS_PLAN_VALIDITY_BASIS_REQUIRED')
    order = value.get('target_order') or dict(mode='TOP_LAYER_FIRST_CENTRE_OUT')
    if not isinstance(order, dict) or order.get('mode') not in ORDER_MODES:
        raise ValueError('TARGET_ORDER_INVALID: ' + '|'.join(ORDER_MODES))
    tolerance = order.get('layer_tolerance_m', DEFAULT_LAYER_TOLERANCE_M)
    if type(tolerance) not in (int, float) or not 0.01 <= tolerance <= 0.1:
        raise ValueError('TARGET_ORDER_LAYER_TOLERANCE_INVALID: 0.01-0.1 m')
    expiries = value.get('max_plan_expiries', 2)
    if type(expiries) is not int or not 0 <= expiries <= 5:
        raise ValueError('CONTINUOUS_MAX_PLAN_EXPIRIES_INVALID: 0-5')
    timeout = value.get('observation_timeout_s', 90.)
    if type(timeout) not in (int, float) or not 10 <= timeout <= 600:
        raise ValueError('CONTINUOUS_OBSERVATION_TIMEOUT_INVALID: 10-600 s')
    return dict(max_cartons=count,
                receiver_clearing=dict(mode=receiver['mode'], basis=str(receiver['basis']),
                                       clearing_time_s=None if clearing is None else float(clearing)),
                plan_validity_s=float(validity), plan_validity_basis=str(validity_basis),
                target_order=dict(mode=order['mode'], layer_tolerance_m=float(tolerance),
                                  basis=str(order.get('basis') or DEFAULT_ORDER_BASIS)),
                max_plan_expiries=expiries, observation_timeout_s=float(timeout))


def apply_to_config(config):
    """Bind the job's plan validity into the entry configuration (tighten only)."""
    job = settings(config)
    current = config.get('plan_validity_s', 120.)
    if job['plan_validity_s'] < current:
        config['plan_validity_s'] = job['plan_validity_s']
        config['plan_validity_basis'] = job['plan_validity_basis']
    return job


def config_issues(config):
    """Every reason the continuous job cannot be started with this configuration."""
    errors = []
    try:
        settings(config)
    except ValueError as e:
        errors.append(str(e))
    from ..single_carton.config import start_joint_check
    if (config or {}).get('start_joint_check') == 'DISABLED_BY_USER':
        errors.append('START_JOINT_CHECK_MUST_BE_ENABLED: 连续作业要求起点关节核对 ENABLED（AGENTS 11-16 第 6 节）')
    else:
        try:
            start_joint_check(config)
        except ValueError as e:
            errors.append('start_joint_check：' + str(e))
    return errors


# -- target order ---------------------------------------------------------------

def _item_from_box(box):
    measured = box.get('measured_box') if isinstance(box.get('measured_box'), dict) else None
    corners = np.asarray((measured or {}).get('corners_m') or box['corners_m'], float).reshape(-1, 3)
    return dict(detection_index=int(box['detection_index']), resolved=True, top=float(corners[:, 2].max()),
                bottom=float(corners[:, 2].min()), xy=corners[:, :2], centre=corners[:, :2].mean(axis=0))


def _item_from_rejected(item, camera, hidden_extent):
    evidence = item.get('evidence') or {}
    face = evidence.get('observed_face')
    if not face or item.get('reason') == 'OUTSIDE_WORKSPACE_IGNORED':
        return None
    c = np.asarray(face['center_m'], float)
    u = np.asarray(face['u'], float) * float(face['size_m'][0]) / 2
    v = np.asarray(face['v'], float) * float(face['size_m'][1]) / 2
    corners = np.array([c + su * u + sv * v for su in (-1, 1) for sv in (-1, 1)])
    normal = np.asarray(face['normal'], float)
    away = -normal if camera is not None and np.dot(normal, np.asarray(camera) - c) > 0 else normal
    if camera is None:
        away = np.zeros(3)  # without a camera position no extrusion direction: face footprint only
    horizontal = np.array([away[0], away[1], 0.])
    if np.linalg.norm(horizontal) > 1e-6:
        horizontal = horizontal / np.linalg.norm(horizontal) * hidden_extent
        corners = np.vstack([corners, corners + horizontal])
    return dict(detection_index=int(item['detection_index']), resolved=False, top=float(corners[:, 2].max()),
                bottom=float(corners[:, 2].min()), xy=corners[:, :2], centre=corners[:4, :2].mean(axis=0))


def _overlap(a, b):
    lo = np.maximum(a.min(axis=0), b.min(axis=0))
    hi = np.minimum(a.max(axis=0), b.max(axis=0))
    return bool(np.all(hi - lo > FOOTPRINT_OVERLAP_MIN_M))


def target_order(boxes, rejected, order_settings, camera_position=None, hidden_extent_m=DEFAULT_HIDDEN_EXTENT_M):
    """Deterministic search order of the measured boxes, and the ones not searched.

    TOP_LAYER_FIRST_CENTRE_OUT: boxes are grouped into layers by their top
    height (a new layer starts more than ``layer_tolerance_m`` below the
    first top of the current one), highest layer first; inside a layer the box
    nearest the row's lateral middle comes first, then outwards (lateral =
    horizontal, perpendicular to the base -> layer direction; the middle is
    the midpoint of every detected carton in that height band, measured or
    not). Ties go to the lower detection index.

    A measured box with another detected carton resting on it (that carton's
    bottom at or above this box's top minus the tolerance, footprints
    overlapping) is not searched: picking it first could drop the upper one.
    Unresolved cartons count with their visible face plus the hidden extent
    behind it (away from the camera). Nothing here changes any obstacle.
    """
    tolerance = order_settings['layer_tolerance_m']
    items = [_item_from_box(b) for b in boxes]
    others = [x for x in (_item_from_rejected(r, camera_position, hidden_extent_m) for r in rejected or ()) if x]
    every = items + others
    skipped = []
    eligible = []
    for item in items:
        above = [o['detection_index'] for o in every if o is not item and o['bottom'] >= item['top'] - tolerance
                 and _overlap(item['xy'], o['xy'])]
        if above:
            skipped.append(dict(detection_index=item['detection_index'], reason='CARTON_RESTING_ON_TOP', above=sorted(above)))
        else:
            eligible.append(item)
    eligible.sort(key=lambda x: (-x['top'], x['detection_index']))
    layers = []
    for item in eligible:
        if layers and item['top'] >= layers[-1]['top'] - tolerance:
            layers[-1]['items'].append(item)
        else:
            layers.append(dict(top=item['top'], items=[item]))
    order = []
    layer_records = []
    for layer in layers:
        band = [o for o in every if o['top'] >= layer['top'] - tolerance and o['top'] <= layer['top'] + tolerance]
        centre = np.mean([o['centre'] for o in band], axis=0)
        radial = centre / np.linalg.norm(centre) if np.linalg.norm(centre) > 1e-6 else np.array([1., 0.])
        lateral_axis = np.array([-radial[1], radial[0]])
        lateral = [float(np.dot(o['centre'], lateral_axis)) for o in band]
        middle = (min(lateral) + max(lateral)) / 2
        ranked = sorted(layer['items'], key=lambda x: (round(abs(float(np.dot(x['centre'], lateral_axis)) - middle), 4),
                                                       x['detection_index']))
        order += [x['detection_index'] for x in ranked]
        layer_records.append(dict(top_m=round(layer['top'], 4), lateral_middle_m=round(middle, 4),
                                  band_detections=sorted(o['detection_index'] for o in band),
                                  order=[x['detection_index'] for x in ranked],
                                  lateral_offset_m={x['detection_index']: round(float(np.dot(x['centre'], lateral_axis)) - middle, 4)
                                                    for x in ranked}))
    return dict(mode=order_settings['mode'], basis=order_settings['basis'], layer_tolerance_m=tolerance,
                order=order, skipped=skipped, layers=layer_records,
                unresolved_counted=sorted(o['detection_index'] for o in others))


# -- the job --------------------------------------------------------------------

class JobEnded(Exception):
    def __init__(self, reason, detail=''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason
        self.detail = detail


class ContinuousJob:
    """Background driver of consecutive single-carton cycles on one TaskManager.

    ``manager`` is the entry's TaskManager (or a fake with the same members).
    ``notify(kind, value)`` reports to the UI: 'job' with the job status after
    every change. The job thread never touches Tk.
    """
    POLL_S = 0.05

    def __init__(self, manager, config, notify=lambda *_: None, clock=time.monotonic, record_root=None,
                 execute_gate=lambda: []):
        self.manager = manager
        # The UI's execute-button device blockers (enabled, alarm, data valid,
        # controller payload, execution lock), minus the manual-only ones.
        self.execute_gate = execute_gate
        self.config = config
        self.notify = notify
        self.clock = clock
        self.settings = settings(config)
        self.job_id = uuid.uuid4().hex
        root = Path(record_root or config['record_root'])
        self.folder = root / 'jobs' / self.job_id
        self.stop_requested = threading.Event()
        self.operator_clear = threading.Event()
        self.state = 'IDLE'
        self.detail = ''
        self.rounds = []
        self.end = None
        self.thread = None
        self.first_q_start = None
        self.generation = None
        self.release_s = None
        self.lock = threading.Lock()

    # -- public -------------------------------------------------------------
    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self):
        m = self.manager
        problems = config_issues(self.config)
        if problems:
            raise ValueError('连续作业配置不完整：\n' + '\n'.join(problems))
        if m.active:
            raise ValueError('已有活动任务或操作，不能开始连续作业')
        if m.vacuum.holding:
            raise ValueError('吸具可能仍持箱：不能开始连续作业，请现场处置')
        if m.cycle is not None and m.cycle.state == 'STOP_UNCONFIRMED':
            raise ValueError('停止未确认：不能开始连续作业')
        self.folder.mkdir(parents=True, exist_ok=False)
        self.generation = m.backend.observation_generation
        self.started_utc_ns = time.time_ns()
        self._event(kind='job_started', settings=self.settings, generation=self.generation,
                    physical_result='NOT_RUN')
        self._set('STARTING', '连续作业已开始')
        self.thread = threading.Thread(target=self._run, daemon=True, name='continuous-job')
        self.thread.start()

    def stop(self, reason='OPERATOR_STOP'):
        """Operator stop: stops the current cycle too (never releases a held load)."""
        self.stop_requested.set()
        self.stop_reason = reason
        self.operator_clear.set()
        self.manager.stop()

    def confirm_receiver_clear(self):
        """OPERATOR_CONFIRM_EACH: the operator confirms the release region is clear."""
        self._event(kind='operator_confirmed_receiver_clear')
        self.operator_clear.set()

    def status(self):
        with self.lock:
            return dict(job_id=self.job_id, state=self.state, detail=self.detail,
                        completed=sum(1 for r in self.rounds if r.get('cycle_state') == 'COMPLETE'),
                        max_cartons=self.settings['max_cartons'], rounds=[dict(r) for r in self.rounds],
                        end=None if self.end is None else dict(self.end), folder=str(self.folder))

    # -- records --------------------------------------------------------------
    def _event(self, **event):
        event = dict(job_id=self.job_id, host_monotonic_s=self.clock(), host_utc_ns=time.time_ns(), **event)
        try:
            with open(self.folder / 'events.jsonl', 'a', encoding='utf-8') as f:
                f.write(json.dumps(event, ensure_ascii=False, default=_json_default) + '\n')
                f.flush()
        except OSError as e:
            raise JobEnded('RECORD_WRITE_FAILED', str(e)) from e

    def _write_job(self):
        status = self.status()
        cycles = [r.get('physical_result') for r in status['rounds'] if r.get('cycle_state')]
        if cycles and status['end'] is not None and all(p == 'PHYSICAL_CYCLE_COMPLETE' for p in cycles):
            physical_result = 'PHYSICAL_JOB_COMPLETE'
        elif any(p == 'PHYSICAL_CYCLE_COMPLETE' for p in cycles):
            physical_result = 'NOT_COMPLETE'
        else:
            physical_result = 'NOT_RUN'
        record = dict(status, schema='eco65_continuous_job_v1', task_ids=[r['task_id'] for r in status['rounds'] if r.get('task_id')],
                      settings=self.settings, config_path=self.config.get('config_path'), physical_result=physical_result)
        try:
            path = self.folder / 'job.json'
            path.with_suffix('.tmp').write_text(json.dumps(record, ensure_ascii=False, indent=2, default=_json_default))
            path.with_suffix('.tmp').replace(path)
        except OSError as e:
            raise JobEnded('RECORD_WRITE_FAILED', str(e)) from e

    def _set(self, state, detail=''):
        with self.lock:
            self.state = state
            self.detail = detail
        try:
            self.notify('job', self.status())
        except Exception:
            pass

    # -- the loop -------------------------------------------------------------
    def _run(self):
        try:
            self._write_job()
            expiries = 0
            while True:
                self._check_stop()
                done = sum(1 for r in self.rounds if r.get('cycle_state') == 'COMPLETE')
                if done >= self.settings['max_cartons']:
                    raise JobEnded('CARTON_COUNT_REACHED', f'{done} 箱')
                if self.rounds:
                    self._between_rounds()
                round_record = dict(round=len(self.rounds) + 1)
                with self.lock:
                    self.rounds.append(round_record)
                outcome = self._round(round_record)
                self._write_job()
                if outcome == 'PLAN_EXPIRED':
                    expiries += 1
                    with self.lock:
                        self.rounds.remove(round_record)
                    self._event(kind='plan_expired_reobserve', count=expiries, task_id=round_record.get('task_id'))
                    if expiries > self.settings['max_plan_expiries']:
                        raise JobEnded('PLAN_EXPIRED_REPEATEDLY', f'{expiries} 次')
                    continue
                expiries = 0
        except JobEnded as e:
            self._finish(e.reason, e.detail)
        except Exception as e:  # never leave the job silently running
            self._finish('JOB_ERROR', f'{type(e).__name__}: {e}')

    def _finish(self, reason, detail):
        if self.stop_requested.is_set() and reason not in ('OPERATOR_STOP',):
            detail = f'{reason} {detail}'.strip()
            reason = 'OPERATOR_STOP'
        with self.lock:
            self.end = dict(reason=reason, label=END_REASONS.get(reason, reason), detail=detail, utc_ns=time.time_ns())
        try:
            self._event(kind='job_ended', reason=reason, detail=detail, physical_result='NOT_RUN')
            self._write_job()
        except JobEnded as e:
            with self.lock:
                self.end['record_error'] = e.detail
        self._set('ENDED', END_REASONS.get(reason, reason) + (f'：{detail}' if detail else ''))

    def _check_stop(self):
        if self.stop_requested.is_set():
            raise JobEnded('OPERATOR_STOP')

    def _wait(self, condition, timeout_s, label):
        deadline = self.clock() + timeout_s
        while not condition():
            self._check_stop()
            if self.clock() > deadline:
                raise JobEnded('OBSERVATION_TIMEOUT', label)
            time.sleep(self.POLL_S)

    def _check_session(self):
        m = self.manager
        if m.backend.observation_generation != self.generation:
            raise JobEnded('SESSION_CHANGED', f'generation {self.generation} -> {m.backend.observation_generation}')
        states = {k: m.backend.device_state(k) for k in ('camera', 'robot', 'vacuum')}
        from ..single_carton.cycle import dry_run_policy
        needed = ('camera', 'robot') if dry_run_policy(self.config) is not None else ('camera', 'robot', 'vacuum')
        bad = {k: states[k] for k in needed if states[k] != '数据有效'}
        if bad:
            raise JobEnded('DEVICE_DATA_INVALID', json.dumps(bad, ensure_ascii=False))

    def _between_rounds(self):
        """Preconditions of AGENTS 11-16 section 6 before the next round starts."""
        m = self.manager
        c = m.cycle
        if c is None or c.state != 'COMPLETE':
            raise JobEnded('CYCLE_NOT_COMPLETE', getattr(c, 'state', None) or 'NO_CYCLE')
        if getattr(c, 'dry_run', None):
            raise JobEnded('DRY_RUN_NO_RELEASE')
        # Both cycle structures reach COMPLETE only after the release pressure
        # rule held (four-segment: at q_start; separated: at q_release, which
        # also sets release_confirmed before the empty return).
        if not c.status().get('cycle_release_command_sent'):
            raise JobEnded('RELEASE_NOT_CONFIRMED')
        if m.vacuum.holding:
            raise JobEnded('LOAD_MAY_BE_HELD')
        self._check_session()
        receiver = self.settings['receiver_clearing']
        if receiver['mode'] == 'USER_CONFIRMED_SELF_CLEARING':
            wait = receiver['clearing_time_s']
            self._set('RECEIVER_CLEARING', f'等待落箱自动清空 {wait:.1f} s（用户确认，非传感器证据）')
            end = self.release_s + wait
            while self.clock() < end:
                self._check_stop()
                time.sleep(self.POLL_S)
            self._event(kind='receiver_clearing', mode=receiver['mode'], clearing_time_s=wait, basis=receiver['basis'],
                        evidence='USER_FIELD_CONFIRMATION_NOT_SENSOR_EVIDENCE')
        elif receiver['mode'] == 'OPERATOR_CONFIRM_EACH':
            self.operator_clear.clear()
            self._set('WAIT_OPERATOR_CLEAR', '请确认落箱位已清空，然后点“落箱位已清空”')
            while not self.operator_clear.is_set():
                self._check_stop()
                time.sleep(self.POLL_S)
            self._check_stop()
        self.clear_s = self.clock()

    def _observe(self, round_record):
        m = self.manager
        not_before = self.clear_s if self.rounds[:-1] else None
        self._set('OBSERVING', f'第 {round_record["round"]} 轮：停稳取景，检测 / 分割 / 测量')
        from ..single_carton.config import observation_mode
        if observation_mode(self.config)['mode'] == 'CONTINUOUS_STANDING':
            requested = self.clock()
            m.request_standing_refresh()
            def ready():
                s = m.standing
                if s is None or m.observation is not s.observation:
                    return False
                return s.observation.frame.stamp.received_monotonic_ns / 1e9 >= (not_before or requested)
            self._wait(ready, self.settings['observation_timeout_s'], '常驻观测刷新')
        else:
            try:
                m.observe()
            except Exception as e:
                raise JobEnded('OBSERVATION_FAILED', str(e)) from e
        observation = m.observation
        if observation is None:
            raise JobEnded('OBSERVATION_FAILED', m.reason)
        exposure_s = observation.frame.stamp.received_monotonic_ns / 1e9
        if not_before is not None and exposure_s < not_before:
            raise JobEnded('OBSERVATION_FAILED', 'FRAME_NOT_AFTER_RELEASE')
        q = np.asarray(observation.q_start, float)
        if self.first_q_start is None:
            self.first_q_start = q
        else:
            from ..single_carton.cycle import AT_START_RAD
            offset = float(np.max(np.abs(q - self.first_q_start)))
            if offset > AT_START_RAD:
                raise JobEnded('START_MOVED', f'{np.rad2deg(offset):.3f}° > {np.rad2deg(AT_START_RAD):.3f}°')
        detections = len(getattr(m.result, 'detections', None) or ())
        round_record.update(observation_id=observation.frame.observation_id, detections=detections,
                            measured_boxes=[b['detection_index'] for b in m.boxes],
                            unresolved=[dict(detection_index=r.get('detection_index'), reason=r.get('reason')) for r in m.rejected])
        if detections == 0:
            raise JobEnded('NO_TARGET_DETECTED', '本帧无检测')
        if m.phase != 'TARGET_READY' or not m.boxes:
            raise JobEnded('NO_PLANNABLE_TARGET', (m.reason or '')[:300])
        return observation

    def _round(self, round_record):
        m = self.manager
        observation = self._observe(round_record)
        camera = np.asarray(observation.world_camera, float)[:3, 3]
        order = target_order(m.boxes, m.rejected, self.settings['target_order'], camera)
        round_record['target_order'] = order
        self._event(kind='round_observed', round=round_record['round'], observation_id=round_record['observation_id'],
                    detections=round_record['detections'], target_order=order)
        if not order['order']:
            raise JobEnded('NO_ELIGIBLE_TARGET', json.dumps(order['skipped'], ensure_ascii=False))
        self._check_stop()
        self._set('PLANNING', f'第 {round_record["round"]} 轮：按顺序 {order["order"]} 规划')
        try:
            m.prepare_plan(list(order['order']))
        except Exception as e:
            round_record.update(task_id=m.task_id, planning='FAILED', reason=str(e)[:500])
            self._event(kind='round_planning_failed', round=round_record['round'], task_id=m.task_id, reason=str(e)[:2000],
                        evidence=getattr(e, 'evidence', None))
            if self.stop_requested.is_set():
                raise JobEnded('OPERATOR_STOP')
            raise JobEnded('NO_COMPLETE_PLAN', str(e)[:300]) from e
        plan = m.plan
        round_record.update(task_id=m.task_id, planning='PLAN_READY', target=plan['target']['id'],
                            target_detection_index=plan['target']['detection_index'])
        self._event(kind='round_plan_ready', round=round_record['round'], task_id=m.task_id, target=plan['target']['id'],
                    target_detection_index=plan['target']['detection_index'], plan_remaining_s=m.plan_remaining_s())
        self._check_stop()
        self._check_session()
        blockers = list(self.execute_gate() or ())
        if blockers:
            round_record.update(execution='REFUSED', reason='、'.join(blockers))
            self._event(kind='round_execution_refused', round=round_record['round'], task_id=m.task_id, blockers=blockers)
            raise JobEnded('EXECUTION_REFUSED', '、'.join(blockers))
        self._set('EXECUTING', f'第 {round_record["round"]} 轮：执行 {plan["target"]["id"]}')
        try:
            m.execute()
        except Exception as e:
            text = str(e)
            round_record.update(execution='REFUSED', reason=text[:500])
            self._event(kind='round_execution_refused', round=round_record['round'], task_id=m.task_id, reason=text[:2000])
            if 'PLAN_EXPIRED_REPLAN' in text and not self.stop_requested.is_set():
                return 'PLAN_EXPIRED'
            raise JobEnded('EXECUTION_REFUSED', text[:300]) from e
        self._wait_cycle(round_record)
        return 'COMPLETE'

    def _wait_cycle(self, round_record):
        from ..single_carton.cycle import TERMINAL
        m = self.manager
        c = m.cycle
        last = None
        while c.state not in TERMINAL:
            if c.state != last:
                last = c.state
                self._set('EXECUTING', f'第 {round_record["round"]} 轮：{c.state}')
            time.sleep(self.POLL_S)
        status = c.status()
        round_record.update(cycle_state=c.state, cycle_reason=c.reason, physical_result=status.get('physical_result'),
                            release_confirmed=status.get('release_confirmed'))
        self.release_s = self.clock()
        self._event(kind='round_cycle_terminal', round=round_record['round'], task_id=m.task_id, cycle=status)
        if c.state != 'COMPLETE':
            raise JobEnded('CYCLE_NOT_COMPLETE', f'{c.state} {c.reason}')


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return str(value)
