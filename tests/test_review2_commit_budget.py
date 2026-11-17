"""Final persistence and lock waits remain inside the caller's absolute budget."""
import json
from pathlib import Path
import threading
import time
import types

import pytest

from test_single_carton_planner import case
from unloading_sim.workbench.single_carton.planner import PlanningBudgetExhausted


def fake_plan(self, observation, boxes, folder, *args):
    package = self.authority.register({}, {}, dict(native_plan='PASS', sent_path_collision='PASS'))
    result = dict(package=package)
    self.complete_plan_path.write_text(json.dumps(result))
    return result


@pytest.mark.parametrize('failure', [False, True])
def test_slow_commit_io_cannot_delay_return_or_publish_late(case, monkeypatch, failure):
    p, obs, box, folder, calls, opts = case
    monkeypatch.setattr(p, '_plan_cycle', types.MethodType(fake_plan, p))
    original = Path.replace
    entered = threading.Event()
    def slow(path, target):
        if path.parent == folder and path.name.startswith('complete-plan.pending'):
            entered.set()
            time.sleep(.25)
            if failure: raise OSError('COMMIT_RENAME_INJECTED')
        return original(path, target)
    monkeypatch.setattr(Path, 'replace', slow)
    started = time.monotonic()
    with pytest.raises(PlanningBudgetExhausted):
        p.plan_cycle(obs, [box], folder, deadline=started+.1)
    elapsed = time.monotonic()-started
    assert entered.is_set() and elapsed < .15
    assert not p.authority._plans and not (folder/'complete-plan.json').exists()
    old_status = p.last_task
    new = folder.parent/'new';new.mkdir()
    plan = p.plan_cycle(obs, [box], new, deadline=time.monotonic()+2)
    ids = set(p.authority._plans)
    deadline = time.monotonic()+2
    while not old_status.get('cleanup_completed_s') and time.monotonic() < deadline:time.sleep(.005)
    assert old_status['status'] == 'INVALIDATED'
    assert old_status['cleanup_finished']
    assert old_status['caller_returned_s'] < old_status['cleanup_completed_s']
    assert not (folder/'complete-plan.json').exists()
    assert set(p.authority._plans) == ids and len(ids) == 1
    p.authority.verify(plan['package'], {})
    assert (new/'complete-plan.json').is_file()


@pytest.mark.parametrize('lock_name', ['task_lock', 'authority'])
def test_initial_commit_lock_wait_is_bounded(case, monkeypatch, lock_name):
    p, obs, box, folder, calls, opts = case
    monkeypatch.setattr(p, '_plan_cycle', types.MethodType(fake_plan, p))
    lock = p.task_lock if lock_name == 'task_lock' else p.authority._lock
    held = threading.Event();release = threading.Event()
    def hold():
        with lock:held.set();release.wait(2)
    thread = threading.Thread(target=hold);thread.start();assert held.wait(1)
    try:
        started = time.monotonic()
        with pytest.raises(PlanningBudgetExhausted):
            p.plan_cycle(obs, [box], folder, deadline=started+.1)
        assert time.monotonic()-started < .15
        assert not p.authority._plans and not (folder/'complete-plan.json').exists()
    finally:release.set();thread.join(2)


def test_finishing_timeout_counts_the_winning_candidate():
    from types import SimpleNamespace
    from unloading_sim.workbench.single_carton.budget import snapshot
    planner = SimpleNamespace(deadline=1., search_counts={'box':dict(candidates=3,
        considered=[0, 1, 2], screened=3, phase='SEARCH_SUCCEEDED', accepted=1)})
    result = snapshot(planner, [dict(id='box', detection_index=0)],
        [dict(target='box', candidate=0, reason='REJECTED')], None, PlanningBudgetExhausted('COMMIT_RECORD_RENAME'))
    target = result['targets'][0]
    assert target['searched'] == 2 and target['unsearched'] == 1
    assert target['infeasible_candidates'] == 1 and target['accepted_candidate'] == 1
    assert target['status'] == 'SEARCH_SUCCEEDED_FINALIZATION_BUDGET_EXHAUSTED'
