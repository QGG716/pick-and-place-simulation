"""Real evidence barrier and authority; fake cycle and no hardware transports."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from test_single_carton_pre_execute import manager
from unloading_sim.workbench.single_carton import runtime
from unloading_sim.workbench.single_carton.evidence import EvidenceWriter


@pytest.mark.parametrize('transition,reason', [
    ('observation', 'OBSERVATION_INVALIDATED'),
    ('session', 'UNTRUSTED_STALE_OR_PREVIEW_TRAJECTORY'),
    ('permit', 'UNTRUSTED_STALE_OR_PREVIEW_TRAJECTORY'),
    ('cancel', 'CANCELLED'),
])
def test_invalidation_during_final_npz_barrier(tmp_path, monkeypatch, transition, reason):
    m, events, started, saved = manager(tmp_path, {})
    m.folder = tmp_path
    m.evidence = EvidenceWriter()
    m.stop_event = threading.Event()
    m.save_observation = lambda o, n: runtime.TaskManager.save_observation(m, o, n)
    observation = SimpleNamespace(metadata=lambda: dict(generation=1),
        frame=SimpleNamespace(rgb=np.zeros((2, 2, 3)), depth=np.zeros((2, 2))))
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: observation)
    monkeypatch.setattr(runtime, 'unchanged', lambda *a, **kwargs: dict(status='PASS'))
    monkeypatch.setattr(runtime, 'target_unchanged', lambda *a: None)
    entered, release = threading.Event(), threading.Event()
    original = np.savez_compressed
    def blocked(*a, **k):
        entered.set()
        assert release.wait(3)
        return original(*a, **k)
    monkeypatch.setattr(np, 'savez_compressed', blocked)
    errors = []
    def execute():
        try: runtime.TaskManager.execute(m)
        except Exception as error: errors.append(error)
    thread = threading.Thread(target=execute)
    try:
        thread.start()
        assert entered.wait(2)
        if transition == 'observation': m.backend.observation_generation += 1
        elif transition == 'session': m.planner.authority.invalidate()
        elif transition == 'permit': m.planner.authority._plans.clear()
        else: m.stop_event.set()
        release.set()
        thread.join(3)
        assert not thread.is_alive()
        assert started == []
        assert len(errors) == 1 and reason in str(errors[0])
        assert not m.planner.authority._plans
    finally:
        release.set()
        thread.join(3)
        m.evidence.close()


def test_backend_transition_is_ordered_with_start(tmp_path):
    from unloading_sim.workbench.backend import WorkbenchBackend
    from test_single_carton_pre_execute import DISABLED
    m, events, started, saved = manager(tmp_path, DISABLED)
    backend = WorkbenchBackend.__new__(WorkbenchBackend)
    backend.observation_lock = threading.RLock()
    backend.observation_generation = 1
    m.backend = backend
    entered, release, transitioned = (threading.Event() for _ in range(3))
    def start():
        entered.set()
        assert release.wait(3)
        assert backend.observation_generation == 1
        started.append(1)
    m.cycle.start = start
    errors = []
    def execute():
        try: runtime.TaskManager.execute(m)
        except Exception as e: errors.append(e)
    def transition():
        backend.observation_generation += 1
        transitioned.set()
    execution = threading.Thread(target=execute)
    change = threading.Thread(target=transition)
    try:
        execution.start()
        assert entered.wait(2)
        change.start()
        assert not transitioned.wait(.03)
    finally:
        release.set()
        execution.join(3)
        change.join(3)
    assert not errors and started == [1] and transitioned.is_set()
    assert backend.observation_generation == 2


def test_two_device_transitions_cannot_lose_a_generation():
    from unloading_sim.workbench.backend import WorkbenchBackend
    barrier = threading.Barrier(2)
    class InterleavedBackend(WorkbenchBackend):
        @property
        def observation_generation(self):
            value = WorkbenchBackend.observation_generation.fget(self)
            if threading.current_thread() is not threading.main_thread():barrier.wait(2)
            return value
    backend = InterleavedBackend.__new__(InterleavedBackend)
    backend.observation_lock = threading.RLock()
    backend._observation_generation = 0
    backend.sample_cutoff = {}
    threads = [threading.Thread(target=backend.invalidate_device, args=('camera',)) for _ in range(2)]
    for thread in threads:thread.start()
    for thread in threads:thread.join(3)
    assert all(not t.is_alive() for t in threads)
    assert backend.observation_generation == 2
