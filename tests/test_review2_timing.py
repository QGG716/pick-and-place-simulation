"""Instrumentation overflow and full-sample coverage cannot change outcomes."""
import json
import time

from unloading_sim.workbench import timing


def test_gc_overflow_with_since_remains_bounded_and_writes(tmp_path, monkeypatch):
    recorder = timing.Recorder()
    monkeypatch.setattr(timing, 'recorder', recorder)
    monkeypatch.setattr(timing, 'ENABLED', True)
    since = time.monotonic()
    for _ in range(8193):recorder.gc_pause(0, .001)
    assert len(recorder.gc_events) == 8192 and recorder.gc_overflow == 1
    path = tmp_path/'timing.json'
    thread = timing.write_later(path, 'prepare_plan', since=since)
    thread.join(2)
    assert not thread.is_alive() and thread.error is None
    rows = json.loads(path.read_text())['events']
    overflow = [r for r in rows if r['name'] == 'python.gc_pause_overflow']
    assert len(overflow) == 1 and overflow[0]['count'] == 1
    assert sum(r['count'] for r in rows if r['name'] == 'python.gc_pause') == 8192
    assert recorder.take(since)[0] == []

