"""Deterministic creation/close races with resources held at publication."""
import threading
from types import SimpleNamespace

import pytest

from test_single_carton_planner import case
from unloading_sim.workbench.single_carton.resident import WorkerPool


def test_close_rejects_late_prewarm_and_repeated_close(case, monkeypatch):
    p, *_ = case;p.site['resident_workers'] = True
    entered, release = threading.Event(), threading.Event();closed=[]
    pool = SimpleNamespace(close=lambda:closed.append(1))
    def warm(self):
        entered.set();assert release.wait(3);return pool
    monkeypatch.setattr(type(p), 'resident_pool', warm)
    thread = p.prewarm();assert entered.wait(1)
    try:
        p.close();p.close()
    finally:release.set();thread.join(3)
    assert not thread.is_alive() and p.resident is None and closed == [1]
    assert p.prewarm() is None


@pytest.mark.parametrize('role', ['main', 'check', 'startup'])
def test_close_rejects_workers_created_during_close(tmp_path, monkeypatch, role):
    pool = WorkerPool(None, {}, 0 if role != 'check' else 1, tmp_path)
    entered, release = threading.Event(), threading.Event();closed=[]
    old = SimpleNamespace(alive=lambda:False, close=lambda:None)
    live = SimpleNamespace(alive=lambda:True, close=lambda:closed.append('old'))
    new = SimpleNamespace(alive=lambda:True, close=lambda:closed.append('new'))
    if role == 'check':pool.main=live;pool.pool=[old]
    def start(name):
        entered.set();assert release.wait(3);return new, {}
    monkeypatch.setattr(pool, '_start_one', start)
    target = pool._start_all if role == 'startup' else pool._replenish
    thread=threading.Thread(target=target);thread.start();assert entered.wait(1)
    try:pool.close();pool.close()
    finally:release.set();thread.join(3)
    assert not thread.is_alive()
    assert not pool.live() and closed.count('new') == 1
    assert closed.count('old') <= 1


def test_late_resource_cleanup_failure_is_recorded(tmp_path, monkeypatch):
    pool = WorkerPool(None, {}, 0, tmp_path)
    entered, release = threading.Event(), threading.Event()
    def fail():raise OSError('CLOSE_INJECTED')
    new=SimpleNamespace(name='late',alive=lambda:True,close=fail)
    def start(name):entered.set();release.wait(3);return new, {}
    monkeypatch.setattr(pool, '_start_one', start)
    thread=threading.Thread(target=pool._replenish);thread.start();assert entered.wait(1)
    pool.close();release.set();thread.join(3)
    assert not pool.live()
    assert any('CLOSE_INJECTED' in e['error'] for e in pool.cleanup_errors)


def test_late_prewarm_close_failure_is_observable(case, monkeypatch):
    p, *_ = case;p.site['resident_workers'] = True
    entered, release = threading.Event(), threading.Event()
    def fail():raise OSError('PREWARM_CLOSE_INJECTED')
    def warm(self):entered.set();release.wait(3);return SimpleNamespace(close=fail)
    monkeypatch.setattr(type(p), 'resident_pool', warm)
    thread=p.prewarm();assert entered.wait(1)
    p.close();release.set();thread.join(3)
    assert p.resident is None and p.prewarm_errors == ['PREWARM_CLOSE_INJECTED']


def test_close_cleanup_barrier_reports_failure(tmp_path):
    pool=WorkerPool(None, {}, 0, tmp_path)
    def fail():raise OSError('CLOSE_INJECTED')
    pool.main=SimpleNamespace(name='main',close=fail)
    pool.close();pool.close()
    assert not pool.wait_closed(1)
    assert pool.cleanup_errors == [dict(worker='main', error='CLOSE_INJECTED')]
    assert pool.all() == []
