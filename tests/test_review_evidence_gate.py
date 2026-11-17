"""Review regression: real execute/save_observation/writers, fake IO only."""
import json
import threading
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import runtime
from unloading_sim.workbench.single_carton.evidence import EvidenceWriter
from test_single_carton_pre_execute import manager, DISABLED

@pytest.mark.parametrize('async_write,broken,disabled', [(True,True,False),(True,False,False),(False,False,False),(True,False,True)])
def test_new_observation_must_be_written_before_start(tmp_path,monkeypatch,async_write,broken,disabled):
    m,events,started,saved=manager(tmp_path,DISABLED if disabled else {})
    m.folder=tmp_path;m.evidence=EvidenceWriter() if async_write else None
    m.save_observation=lambda o,n:runtime.TaskManager.save_observation(m,o,n)
    permits={'valid':True}
    m.planner.authority.invalidate=lambda:permits.update(valid=False)
    o=SimpleNamespace(metadata=lambda:dict(generation=1),frame=SimpleNamespace(rgb=np.zeros((2,2,3)),depth=np.zeros((2,2))))
    monkeypatch.setattr(runtime,'freeze',lambda *a,**k:o)
    monkeypatch.setattr(runtime,'unchanged',lambda *a:dict(status='PASS'))
    monkeypatch.setattr(runtime,'target_unchanged',lambda *a:None)
    if broken:
        def fail(*a,**k):raise OSError('DISK_FULL_INJECTED')
        monkeypatch.setattr(np,'savez_compressed',fail)
    try:
        if broken:
            with pytest.raises(ValueError,match='DISK_FULL_INJECTED'):runtime.TaskManager.execute(m)
            assert started==[] and not permits['valid'] and m.rejected
        else:
            runtime.TaskManager.execute(m)
            assert started==[1]
            if not disabled:assert (tmp_path/'pre_execute_observation.npz').exists()
    finally:
        if m.evidence:m.evidence.close()

@pytest.mark.parametrize('failure',['cancel','timeout','dead'])
def test_writer_barrier_fails_closed(tmp_path,monkeypatch,failure):
    from unloading_sim.workbench.single_carton.evidence import EvidenceWriteFailed
    entered=threading.Event();release=threading.Event();cancel=threading.Event()
    w=EvidenceWriter()
    def blocked(*a,**k):entered.set();release.wait(2)
    monkeypatch.setattr(np,'savez_compressed',blocked)
    try:
        if failure=='dead':w.close()
        else:
            w.write_npz(tmp_path/'blocked.npz',{},'blocked');assert entered.wait(1)
        if failure=='cancel':cancel.set()
        with pytest.raises(EvidenceWriteFailed):w.flush(.03,cancelled=cancel.is_set)
    finally:release.set();w.close()


def test_real_journal_barrier_writes_prior_events(tmp_path):
    j=runtime.Journal(tmp_path,'task')
    try:
        j(kind='pre_execute_recheck');j.flush(timeout=1)
        assert json.loads((tmp_path/'events.jsonl').read_text())['kind']=='pre_execute_recheck'
    finally:j.close()
