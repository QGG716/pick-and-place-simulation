"""observation_mode=CONTINUOUS_STANDING (user decision 2026-11-25).

From start-up, the continuous detection/segmentation loop serves the task
observation: a frame that passes the identical stationary/pairing gate
(perception.FreezeGate: 0.4 s window, 0.02 deg range, 0.5 deg/s, frame age
<= 100 ms, joint pairing <= 30 ms, same session) is segmented by the same
resident model (attached to the live job of that very frame when one exists),
its boxes and site scene are built by TaskManager.perceive (scene_build may run
the mask-independent part in parallel), and the result is installed as ONE
immutable standing observation. Nothing here commands the robot or a valve.

Records: the standing observation is kept in memory. Planning binds it and only
then gets a task id and writes exactly the files the button path writes
(observation, detections, segmentation, geometry, site-scene, layout
registration, journal). A failed or unresolved standing build writes a
diagnostic task record when its reason changes, or at most every 300 s.
"""
from dataclasses import dataclass, field
import threading
import time
import numpy as np
from .. import timing
from .perception import FreezeGate, validate_robot

DIAGNOSTIC_RECORD_INTERVAL_S = 300.
DRIFT_CHECK_INTERVAL_S = .2
MOVING = ('关节位置变化超过 0.02°', '关节速度超过 0.5°/s')


class RecordBuffer:
    """Records produced while building a standing observation, written at bind."""
    def __init__(self):
        self.entries = []; self.tracked = []
    def json(self, name, value, **options):
        self.entries.append(('json', name, value, options))
    def npz(self, name, arrays):
        self.entries.append(('npz', name, arrays, {}))
    def track(self, path):
        self.tracked.append(path)
    def write(self, write_json, write_npz):
        for kind, name, value, options in self.entries:
            if kind == 'json': write_json(name, value, **options)
            else: write_npz(name, value)


@dataclass
class Standing:
    observation: object
    result: object
    boxes: list
    rejected: list
    scene: object
    phase: str
    reason: str
    records: RecordBuffer
    started_timing_s: float
    build_s: float
    installed_monotonic_s: float = 0.
    standing_id: int = 0
    extra: dict = field(default_factory=dict)

    def receipt_age_s(self, now=None):
        now = time.monotonic() if now is None else now
        return now - self.observation.frame.stamp.received_monotonic_ns/1e9


def joint_offset_deg(q, q_start):
    return float(np.rad2deg(np.max(abs(np.asarray(q, float) - np.asarray(q_start, float)))))


class StandingObserver:
    """The one background thread that keeps the standing observation current."""
    def __init__(self, manager, settings):
        self.manager = manager; self.settings = settings
        self.closing = threading.Event(); self.force = threading.Event()
        self.status = '常驻观测：等待启动'; self.gate = None; self.cutoff_ns = None
        self.failure = None; self.failure_at = None; self.last_diagnostic = (None, -1e9)
        self.thread = None; self.count = 0; self.drift_checked = -1e9
    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._run, daemon=True, name='standing-observation')
            self.thread.start()
    def close(self):
        self.closing.set()
    def request_refresh(self):
        self.force.set(); self.failure = None
    # -- loop ---------------------------------------------------------------
    def _run(self):
        while not self.closing.wait(.01):
            try: self._step()
            except Exception as e:
                self.gate = None
                self.status = f'常驻观测：等待（{type(e).__name__}: {e}）'
                self.closing.wait(.2)
    def _pause_reason(self):
        m = self.manager
        reason = m.standing_pause_reason(force=self.force.is_set())
        if reason is None and m.vision() is None: reason = '持续视觉未启动'
        return reason
    def _needs_refresh(self, now):
        m = self.manager; s = m.standing
        if self.force.is_set(): return '操作员要求重新观测'
        if s is None:
            if self.failure is not None and now - self.failure_at < self.settings['refresh_s']: return None
            return '尚无常驻观测'
        if s.observation.generation != m.backend.observation_generation: return '设备会话已变化'
        if s.receipt_age_s(now) > self.settings['refresh_s']: return '常驻观测已到刷新周期'
        # The execution service shares one IPC lock: check the drift at 5 Hz only.
        if now - self.drift_checked < DRIFT_CHECK_INTERVAL_S: return None
        self.drift_checked = now
        sample = m.service.snapshot()
        try: q = validate_robot(sample, time.monotonic())
        except ValueError: return None
        if joint_offset_deg(q, s.observation.q_start) > self.settings['joint_drift_deg']:
            m.drop_standing('机械臂已离开常驻观测起点（>0.02°），等待停稳后重新观测')
            return '机械臂已移动'
        return None
    def _step(self):
        m = self.manager
        reason = self._pause_reason()
        if reason:
            self.status = '常驻观测暂停：'+reason; self.gate = None; self.cutoff_ns = None
            return
        if self.cutoff_ns is None:
            # Frames and joint samples older than this (re)start are never used:
            # after a completed cycle the next standing observation is a new exposure.
            self.cutoff_ns = time.monotonic_ns()
        now = time.monotonic()
        why = self._needs_refresh(now)
        if why is None:
            s = m.standing
            self.status = ('常驻观测就绪 · 年龄 %.1f s · %d 箱' % (s.receipt_age_s(now), len(s.boxes)) if s is not None
                           else '常驻观测失败，%.0f s 后重试：%s' % (self.settings['refresh_s']-(now-self.failure_at), self.failure))
            return
        missing = m.standing_config_issues()
        if missing:
            self.status = '常驻观测：缺少现场参数'; return
        if self.gate is None or self.gate.generation != m.backend.observation_generation:
            self.gate = FreezeGate(m.backend, m.service, m.model, m.calibration, not_before_ns=self.cutoff_ns)
        observation = self.gate.poll()
        if observation is None:
            waiting = self.gate.diagnostic.get('waiting', '')
            self.status = '常驻观测（%s）：%s' % (why, waiting)
            if waiting in MOVING:
                # The arm is moving: restart the window later instead of polling the IPC at 100 Hz.
                self.gate = None; self.closing.wait(.1)
            return
        self.gate = None; forced = self.force.is_set(); self.force.clear()
        self._build(observation, forced)
    def _cancelled(self, generation):
        m = self.manager
        return (self.closing.is_set() or generation != m.backend.observation_generation or
                self._pause_reason() is not None)
    def _build(self, observation, forced):
        m = self.manager; generation = observation.generation
        started = timing.now(); t0 = time.monotonic()
        cancelled = lambda: self._cancelled(generation)
        self.status = '常驻观测：已取景，同帧检测 / 分割与场景预计算进行中'
        records = RecordBuffer(); result = None
        target = type('Target', (), {})(); target.boxes = []; target.scene = None
        try:
            precomputed = m.submit_scene_precompute(observation)
            with timing.span('standing.vision_infer'):
                result = m.vision().infer_standing(observation.frame, generation, cancelled)
            self.status = '常驻观测：恢复箱体与现场碰撞场景'
            rejected = m.perceive(observation, result, target, records.json, records.npz, records.track,
                                  cancelled, lambda _: None, precomputed)
            phase, reason = m.classify(target.boxes, rejected, target.scene)
        except Exception as error:
            if cancelled(): self.status = '常驻观测：本次构建已作废（%s）' % error; return
            self._failed(observation, result, records, target, error)
            return
        standing = Standing(observation, result, target.boxes, rejected, target.scene, phase, reason, records,
                            started, time.monotonic()-t0, extra=dict(forced=forced))
        if phase != 'TARGET_READY':
            self._diagnostic(standing, reason)
        if m.install_standing(standing, forced=forced):
            self.count += 1; self.failure = None
            self.status = '常驻观测已更新（构建 %.1f s）' % standing.build_s
        else:
            self.status = '常驻观测：本次构建已作废（任务状态已变化）'
    def _failed(self, observation, result, records, target, error):
        self.failure = f'{type(error).__name__}: {error}'; self.failure_at = time.monotonic()
        self.manager.drop_standing('常驻观测失败：'+str(error))
        standing = Standing(observation, result, target.boxes, [], target.scene, 'FAILED', str(error), records,
                            timing.now(), 0.)
        self._diagnostic(standing, str(error), error)
    def _diagnostic(self, standing, reason, error=None):
        """Rate-limited task record of a standing build that cannot be planned."""
        previous, at = self.last_diagnostic; now = time.monotonic()
        if reason == previous and now - at < DIAGNOSTIC_RECORD_INTERVAL_S: return
        self.last_diagnostic = (reason, now)
        try: self.manager.write_standing_diagnostic(standing, reason, error)
        except Exception as e: self.status = '常驻观测诊断记录写入失败：'+str(e)
