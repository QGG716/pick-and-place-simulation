"""Deadline evidence must not start another thread on the expired caller path."""
import json
import threading
import time
import types

import pytest

from test_single_carton_planner import case
from unloading_sim.workbench import timing
from unloading_sim.workbench.single_carton.planner import PlanningBudgetExhausted


@pytest.mark.parametrize('write_failure', [False, True])
def test_deadline_record_start_cannot_delay_caller(case, monkeypatch, write_failure):
    p, obs, box, folder, calls, opts = case
    entered = threading.Event()
    release = threading.Event()
    caller = threading.current_thread()
    caller_starts = []
    original_start = threading.Thread.start
    original_write = timing.write_json

    def blocked_plan(self, *args):
        entered.set()
        release.wait(2)
        raise RuntimeError('LATE_TASK_MUST_NOT_PUBLISH')

    def slow_record_start(thread):
        if getattr(thread, '_target', None) is timing.write_json and threading.current_thread() is caller:
            caller_starts.append(thread.name)
            time.sleep(.25)
        return original_start(thread)

    def write(path, *args, **kwargs):
        if write_failure and path.name == 'planning-deadline-return.json':
            raise OSError('DEADLINE_RECORD_WRITE_INJECTED')
        return original_write(path, *args, **kwargs)

    monkeypatch.setattr(p, '_plan_cycle', types.MethodType(blocked_plan, p))
    monkeypatch.setattr(threading.Thread, 'start', slow_record_start)
    monkeypatch.setattr(timing, 'write_json', write)
    try:
        started = time.monotonic()
        with pytest.raises(PlanningBudgetExhausted) as caught:
            p.plan_cycle(obs, [box], folder, deadline=started+.1)
        elapsed = time.monotonic()-started
        assert entered.is_set()
        assert elapsed < .15
        assert not caller_starts
        assert not p.authority._plans
        assert p.last_task['status'] == 'INVALIDATED'
    finally:
        release.set()
    caught.value.record_thread.join(2)
    assert not caught.value.record_thread.is_alive()
    assert p.last_task['cleanup_finished']
    assert not p.authority._plans and not (folder/'complete-plan.json').exists()
    if write_failure:
        assert p.last_task['deadline_record_error'] == 'DEADLINE_RECORD_WRITE_INJECTED'
    else:
        evidence = json.loads((folder/'planning-deadline-return.json').read_text())
        assert evidence['planning_budget']['status'] == 'PLANNING_BUDGET_EXHAUSTED'
        assert p.last_task['caller_returned_s'] <= p.last_task['deadline_record_written_s']


def test_prepare_plan_rejection_and_timing_reuse_task_writer(case, monkeypatch):
    from types import SimpleNamespace
    from unloading_sim.workbench.single_carton.runtime import TaskManager, Journal
    p, obs, box, folder, calls, opts = case
    release = threading.Event()
    def blocked_plan(self, *args):
        release.wait(2)
        raise RuntimeError('LATE_TASK_MUST_NOT_PUBLISH')
    monkeypatch.setattr(p, '_plan_cycle', types.MethodType(blocked_plan, p))
    monkeypatch.setattr(timing, 'ENABLED', True)
    monkeypatch.setattr(timing, 'recorder', timing.Recorder())
    m = TaskManager.__new__(TaskManager)
    m.lock = threading.RLock();m.busy = False;m.phase = 'TARGET_READY';m.cycle = None;m.plan = None;m.reason = ''
    m.vacuum = SimpleNamespace(holding=False);m.service = None;m.planner = p;m.observation = obs;obs.generation = 1
    m.backend = SimpleNamespace(observation_generation=1);m.boxes = [box];m.scene = None;m.stop_event = threading.Event()
    m.folder = folder;m.task_id = folder.name;m.evidence = None
    m.journal = Journal(folder, m.task_id);m.config = dict(planning_deadline_s=.1, vacuum={});m.notify = lambda *a: None
    m.check_contact = lambda *a: None;m.check_release = lambda *a: None
    original_start = threading.Thread.start
    caller = threading.current_thread();late_starts = []
    def slow(thread):
        if threading.current_thread() is caller and thread.name in ('planning-rejection-record', 'timing-writer'):
            late_starts.append(thread.name);time.sleep(.25)
        return original_start(thread)
    monkeypatch.setattr(threading.Thread, 'start', slow)
    try:
        started = time.monotonic()
        with pytest.raises(PlanningBudgetExhausted):m.prepare_plan()
        elapsed = time.monotonic()-started
        assert elapsed < .15
        assert not late_starts and not m.busy and m.phase == 'FAILED' and m.cycle is None
        assert not p.authority._plans
        m.journal.flush(timeout=2)
        record = json.loads((folder/'result.json').read_text())
        assert record['state'] == 'REJECTED' and 'PLANNING_BUDGET_EXHAUSTED' in record['reason']
        measured = json.loads((folder/'timing-prepare_plan.json').read_text())
        assert measured['outcome'] == 'PLANNING_BUDGET_EXHAUSTED'
    finally:
        release.set()
        m.journal.close()


def test_queued_timing_failure_is_observable_without_breaking_journal(tmp_path, monkeypatch):
    from unloading_sim.workbench.single_carton.runtime import Journal
    monkeypatch.setattr(timing, 'ENABLED', True)
    monkeypatch.setattr(timing, 'recorder', timing.Recorder())
    journal = Journal(tmp_path, 'test')
    try:
        receipt = timing.write_later(tmp_path/'missing'/'timing.json', 'prepare_plan', dispatch=journal.defer)
        journal(kind='still_running')
        journal.flush(timeout=2)
        assert not receipt.is_alive() and isinstance(receipt.error, FileNotFoundError)
        assert journal.error is None
        assert json.loads((tmp_path/'events.jsonl').read_text())['kind'] == 'still_running'
    finally:journal.close()
    rejected = timing.write_later(tmp_path/'after-close.json', 'prepare_plan', dispatch=journal.defer)
    assert not rejected.is_alive() and 'TASK_RECORD_UNAVAILABLE' in str(rejected.error)
    assert not (tmp_path/'after-close.json').exists()


def test_queued_write_finishes_with_error_if_journal_dies(tmp_path, monkeypatch):
    from pathlib import Path
    from unloading_sim.workbench.single_carton.runtime import Journal
    entered = threading.Event();release = threading.Event();writes = []
    original = Path.open
    def fail_open(path, *args, **kwargs):
        if path.name == 'events.jsonl':
            entered.set();release.wait(2)
            raise OSError('JOURNAL_OPEN_INJECTED')
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', fail_open)
    journal = Journal(tmp_path, 'test')
    try:
        assert entered.wait(1)
        receipt = timing.dispatch_write(lambda: writes.append(True), journal.defer)
        release.set();receipt.join(2)
        assert not receipt.is_alive() and 'JOURNAL_OPEN_INJECTED' in str(receipt.error)
        assert not writes
    finally:release.set();journal.close()
