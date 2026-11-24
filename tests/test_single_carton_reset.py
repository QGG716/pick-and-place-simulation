"""Operator reset after a stop (user decision 2026-11-24): fake devices only."""
import threading
import time
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.runtime import TaskManager


class Service:
    def __init__(self, q):
        self.q = list(q); self.state = 'CANCELLED'; self.task = None; self.submitted = []; self.fail_on = None
    def feedback(self, task_id=None):
        return dict(task_id=self.task, state=self.state, arrival_confirmed=self.state == 'SUCCEEDED', reason='x')
    def snapshot(self):
        return dict(source='LIVE', q=self.q, velocity=[0.]*6, received_s=time.monotonic(), alarm=False, enabled=True)
    def submission_ticket(self): return 0
    def submit(self, package, cancel_ticket=None):
        self.submitted.append(package['stage']); self.task = str(len(self.submitted))
        if self.fail_on == len(self.submitted): self.state = 'CANCELLED'
        else: self.state = 'SUCCEEDED'; self.q = list(package['end'])
        return self.task
    def request_stop(self): pass


class Authority:
    invalidated = 0
    def invalidate(self): self.invalidated += 1


class Planner:
    def __init__(self): self.authority = Authority(); self.calls = []
    def binding(self): return 'b'
    def plan_reset(self, q, plan, folder, progress, cancelled):
        self.calls.append(list(q))
        segs = [dict(stage='withdraw', package=dict(stage='withdraw', end=[.05]*6)),
                dict(stage='return_empty', package=dict(stage='return_empty', end=plan['q_start']))]
        return dict(segments=segs, binding='b', q_now=list(q), q_start=plan['q_start'], duration_s=3., stages=['withdraw', 'return_empty'])


class Cycle:
    def __init__(self, state): self.state = state; self.active = False; self.stop_requested = threading.Event()


class Vacuum:
    holding = False
    def inhibit(self): pass


def manager(state='CANCELLED', q=(.1,)*6):
    m = TaskManager.__new__(TaskManager)
    m.service = Service(q); m.planner = Planner(); m.vacuum = Vacuum(); m.cycle = Cycle(state)
    m.plan = dict(q_start=[0.]*6); m.folder = 'f'; m.journal = None; m.phase = 'FAILED'; m.reason = ''; m.reset = None
    m.busy = False; m.lock = threading.Lock(); m.stop_event = threading.Event(); m.notify = lambda *a: None
    return m


def test_reset_plans_then_runs_each_segment_to_q_start():
    m = manager()
    summary = m.reset_plan()
    assert summary['stages'] == ['withdraw', 'return_empty'] and m.service.submitted == []
    out = m.reset_execute()
    assert m.service.submitted == ['withdraw', 'return_empty'] and out['state'] == 'RESET_AT_START'
    assert out['max_error_deg'] == 0 and m.reset is None and m.planner.authority.invalidated == 1


@pytest.mark.parametrize('state', ['APPROACH', 'STOP_UNCONFIRMED', 'COMPLETE', 'PLAN_READY'])
def test_reset_refused_unless_stop_confirmed(state):
    m = manager(state)
    with pytest.raises(ValueError):
        m.reset_plan()
    assert m.planner.calls == []


def test_reset_refused_with_held_load_or_at_start():
    m = manager(); m.vacuum.holding = True
    with pytest.raises(ValueError):
        m.reset_plan()
    m = manager(q=(0.,)*6)
    with pytest.raises(ValueError):
        m.reset_plan()
    assert m.planner.calls == []


def test_reset_stops_when_a_segment_is_stopped_and_never_sends_the_next():
    m = manager(); m.reset_plan(); m.service.fail_on = 1
    with pytest.raises(ValueError, match='RESET_STOPPED'):
        m.reset_execute()
    assert m.service.submitted == ['withdraw'] and m.phase == 'FAILED'
    with pytest.raises(ValueError, match='RESET_PLAN_REQUIRED'):
        m.reset_execute()
