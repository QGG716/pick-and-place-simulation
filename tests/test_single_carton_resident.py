"""Step-4 mechanisms with fakes only: no ROS worker, device, SDK or valve."""
import json
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import pytest
from test_single_carton_planner import case  # noqa: F401  (fixture reuse)
from unloading_sim.workbench.single_carton.resident import CancellableWorkerClient, WorkerPool, WorkerRetired
from unloading_sim.workbench.single_carton.evidence import EvidenceWriter, EvidenceWriteFailed
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.config import issues
from unloading_sim.moveit2_backend import MoveItUnavailable

FAKE_WORKER = r'''
import json, sys, threading, time
running = {}; lock = threading.Lock(); mode = sys.argv[1]
def reply(obj): sys.stdout.write(json.dumps(obj)+"\n"); sys.stdout.flush()
def handle(req):
    op = req.get("op"); rid = req.get("request_id")
    if op == "init":
        reply(dict(status="READY", request_id=rid, capabilities=dict(bounded_single_carton_planning=True,
                   resident_task_reset=True, request_cancel=True, scene_handles=True, path_endpoint_reuse=True))); return
    if op == "sc_task_reset":
        if mode == "bad_reset": reply(dict(status="ERROR", detail="RESET_BROKEN", request_id=rid)); return
        reply(dict(status="RESET", request_id=rid, ompl_seed_reapplied=6518)); return
    if op == "slow":
        event = threading.Event()
        with lock: running[rid] = event
        if mode == "ignore_cancel": time.sleep(30)
        event.wait(30)
        reply(dict(status="SEARCH_EXHAUSTED", request_id=rid, cancelled_by_request=True)); return
    reply(dict(status="SUCCESS", request_id=rid, echo=req.get("value")))
pending = []
def work():
    while True:
        while not pending: time.sleep(.001)
        handle(pending.pop(0))
threading.Thread(target=work, daemon=True).start()
for line in sys.stdin:
    req = json.loads(line)
    if req.get("op") == "cancel":
        with lock:
            event = running.get(req["target_request_id"])
        if event: event.set()
        continue
    pending.append(req)
'''


@pytest.fixture
def fake_worker(tmp_path):
    script = tmp_path/'fake_worker.py'; script.write_text(FAKE_WORKER)
    return lambda mode='normal': [sys.executable, str(script), mode]


def test_cancel_by_request_id_keeps_the_worker_usable(fake_worker, tmp_path):
    client = CancellableWorkerClient(fake_worker(), log_path=tmp_path/'w.log', name='w')
    try:
        started = time.monotonic(); flag = threading.Event()
        threading.Timer(.2, flag.set).start()
        with pytest.raises(MoveItUnavailable, match='CANCELLED') as raised:
            client.request(dict(op='slow'), timeout=30, cancelled=flag.is_set)
        assert time.monotonic()-started < 2 and raised.value.response['cancelled_by_request']
        assert client.alive() and client.cancelled_requests == 1
        assert client.request(dict(op='echo', value=7), timeout=5)['echo'] == 7  # same process, next request
    finally:
        client.close()


def test_request_timeout_is_a_cancel_not_a_kill(fake_worker, tmp_path):
    client = CancellableWorkerClient(fake_worker(), name='w')
    try:
        with pytest.raises(MoveItUnavailable, match='MOVEIT2_REQUEST_TIMEOUT'):
            client.request(dict(op='slow'), timeout=.2)
        assert client.alive()
    finally:
        client.close()


def test_worker_that_ignores_cancel_is_retired_and_never_used_again(fake_worker, monkeypatch):
    import unloading_sim.workbench.single_carton.resident as resident
    monkeypatch.setattr(resident, 'CANCEL_GRACE_S', .3)
    client = CancellableWorkerClient(fake_worker('ignore_cancel'), name='w')
    with pytest.raises(MoveItUnavailable, match='CANCELLED'):
        client.request(dict(op='slow'), timeout=30, cancelled=lambda: True)
    assert client.retired == 'CANCEL_NOT_ACKNOWLEDGED' and not client.alive()
    with pytest.raises(WorkerRetired):
        client.request(dict(op='echo'), timeout=5)


def init_payload():
    return dict(op='init', identity=dict(policy_scope='PHYSICAL_SINGLE_CARTON'))


def test_pool_resets_after_init_and_after_every_task(fake_worker, tmp_path):
    pool = WorkerPool(lambda name: fake_worker(), init_payload(), 2, tmp_path)
    try:
        pool.start(); pool.wait_ready(20)
        assert len(pool.live()) == 3 and all(c.clean for c in pool.live())
        report = pool.begin_task()
        assert sorted(report['already_reset']) == sorted(c.name for c in pool.live()) and report['reset_now'] == []
        pool.main.request(dict(op='echo', value=1))  # the task uses the main worker
        assert not pool.main.clean
        pool.end_task(); pool.background.join(10)
        assert all(c.clean for c in pool.live())
        assert pool.begin_task()['reset_now'] == []
    finally:
        pool.close()


def test_failed_reset_retires_the_worker_and_is_reported(fake_worker, tmp_path):
    pool = WorkerPool(lambda name: fake_worker('bad_reset') if name.startswith('check') else fake_worker(),
                      init_payload(), 1, tmp_path)
    try:
        pool.start(); pool.wait_ready(20)
        assert [s['state'] for s in pool.snapshot() if s['role'] == 'check'] == ['FAILED']
        assert pool.checkers() == [] and pool.main.alive()
    finally:
        pool.close()


def test_missing_worker_capability_fails_init(tmp_path):
    script = tmp_path/'old.py'
    script.write_text('import json,sys\nfor l in sys.stdin:\n r=json.loads(l);print(json.dumps(dict(status="READY",request_id=r["request_id"],capabilities=dict(bounded_single_carton_planning=True))),flush=True)\n')
    pool = WorkerPool(lambda name: [sys.executable, str(script)], init_payload(), 0, tmp_path)
    pool.start()
    with pytest.raises(MoveItUnavailable, match='RESIDENT_MAIN_WORKER_UNAVAILABLE'):
        pool.wait_ready(20)
    assert 'NATIVE_WORKER_CAPABILITY_MISSING:resident_task_reset,request_cancel' in pool.snapshot()[0]['error']
    pool.close()


def test_evidence_writer_keeps_order_and_reports_failure(tmp_path):
    writer = EvidenceWriter()
    try:
        for i in range(5):
            writer.write_json(tmp_path/'a.json', dict(i=i), 'a.json')
        writer.flush(5)
        assert json.loads((tmp_path/'a.json').read_text()) == dict(i=4) and writer.written == 5
        writer.write_json(tmp_path/'missing'/'b.json', dict(x=1), 'b.json')
        with pytest.raises(EvidenceWriteFailed, match='b.json'):
            writer.flush(5)
    finally:
        writer.close()


def test_async_evidence_plan_is_on_disk_before_plan_ready(case):
    p, obs, box, folder, calls, opts = case
    p.site['async_evidence'] = True
    plan = p.plan_cycle(obs, [box], folder)
    assert plan['full_path_validated']
    # Written by the background writer, flushed before plan_cycle returned.
    assert json.loads((folder/'complete-plan.json').read_text())['candidate_index'] == plan['candidate_index']
    assert all((Path(r).exists()) for r in folder.rglob('*-sent-check.json'))


def test_async_evidence_write_failure_prevents_plan_ready(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    p.site['async_evidence'] = True
    import unloading_sim.workbench.single_carton.evidence as evidence
    real = evidence.timing.write_json

    def broken(path, value, name, **options):
        if name == 'sent-check.json':
            raise OSError('DISK_FULL_SIMULATED')
        return real(path, value, name, **options)
    monkeypatch.setattr(evidence.timing, 'write_json', broken)
    with pytest.raises(EvidenceWriteFailed, match='DISK_FULL_SIMULATED'):
        p.plan_cycle(obs, [box], folder)
    assert not (folder/'complete-plan.json').exists()


def test_without_switches_the_planner_writes_records_synchronously(case):
    p, obs, box, folder, calls, opts = case
    p.plan_cycle(obs, [box], folder)
    assert p.evidence is None and p.resident is None and p.binding_evidence is None and p.scene_world is None
    assert not any('scene_handle' in r or 'path_endpoint_reuse' in r for r in calls)


def test_scene_handle_replaces_only_the_registered_world(case):
    p, obs, box, folder, calls, opts = case
    p.site['scene_handles'] = True
    uploads = []

    class Client:
        name = 'fake'

        def request(self, r, **kwargs):
            uploads.append(r)
            return dict(status='SCENE_READY', scene_handle='scene-1', scene_fingerprint=r['scene_fingerprint'], objects=len(r['world']))
    world = [dict(id='x', pose=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], size=[.1, .1, .1], category='observed')]
    fingerprint = p.register_scene(world, folder)
    client = Client()
    request = dict(op='validate', world=world, scene_fingerprint=fingerprint, stage='approach')
    replaced = p.with_scene_handle(request, client, folder)
    assert 'world' not in replaced and replaced['scene_handle'] == 'scene-1' and request['world'] is world
    p.with_scene_handle(request, client, folder)
    assert len(uploads) == 1  # one upload per worker per task
    other = dict(request, world=list(world))  # a different world object is never replaced
    assert p.with_scene_handle(other, client, folder) is other
    assert json.loads((folder/'native-scene.json').read_text())['scene_fingerprint'] == fingerprint


def test_endpoint_reuse_flag_is_sent_only_with_the_switch(case):
    p, obs, box, folder, calls, opts = case
    p.site['path_endpoint_reuse'] = True
    opts['capabilities'] = dict(path_endpoint_reuse=True)
    p.plan_cycle(obs, [box], folder)
    probes = [r for r in calls if r.get('op') == 'validate' and 'probe_path' in r]
    assert probes and all(r.get('path_endpoint_reuse') is True for r in probes)
    assert not any('path_endpoint_reuse' in r for r in calls if 'probe_path' not in r)


def test_endpoint_reuse_needs_the_worker_capability(case):
    p, obs, box, folder, calls, opts = case
    p.site['path_endpoint_reuse'] = True  # the fake worker does not declare it
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_PATH_ENDPOINT_REUSE'):
        p.plan_cycle(obs, [box], folder)
    assert not [r for r in calls if r.get('op') in ('plan', 'validate', 'ik')]


def test_binding_hash_cache_reuses_only_while_identity_is_unchanged(tmp_path):
    p = SinglePlanner.__new__(SinglePlanner)
    p.site = dict(binding_hash_cache=True); p.files = {}; p.hash_cache = {}; p.binding_evidence = None
    a = tmp_path/'a.bin'; b = tmp_path/'b.bin'
    a.write_bytes(b'1'*1000); b.write_bytes(b'2'*1000)
    p.track(a); p.track(b)
    # Files changed within the racy window are rehashed, never served from cache.
    p.current_file_hashes()
    assert p.binding_evidence['rehashed'] == 2
    time.sleep(.15)
    p.current_file_hashes()
    first = p.current_file_hashes()
    assert first == p.files and p.binding_evidence['reused'] == 2 and p.binding_evidence['rehashed'] == 0
    a.write_bytes(b'3'*1000)  # same size, new content: mtime and ctime change
    second = p.current_file_hashes()
    rows = {Path(r['file']).name: r['status'] for r in p.binding_evidence['files']}
    assert rows == {'a.bin': 'REHASHED', 'b.bin': 'REUSED'} and second[str(a.resolve())] != first[str(a.resolve())]
    time.sleep(.15); p.current_file_hashes()  # rehashed once more, now outside the racy window
    p.current_file_hashes()
    assert p.binding_evidence['reused'] == 2
    os.utime(a, ns=(a.stat().st_atime_ns, a.stat().st_mtime_ns))  # metadata write changes ctime only
    p.current_file_hashes()
    assert {Path(r['file']).name: r['status'] for r in p.binding_evidence['files']}['a.bin'] == 'REHASHED'
    with pytest.raises(ValueError, match='INPUT_CHANGED_REBUILD_PLAN'):
        # A content change is never hidden: binding() still refuses changed inputs.
        p.model_hash = p.scene_hash = p.commit = None
        p.binding()


def test_binding_without_cache_hashes_every_file_every_time(tmp_path, monkeypatch):
    import unloading_sim.workbench.controlled.planner as controlled
    p = SinglePlanner.__new__(SinglePlanner)
    p.site = {}; p.files = {}; p.hash_cache = {}; p.binding_evidence = None
    f = tmp_path/'c.bin'; f.write_bytes(b'x'); p.track(f)
    seen = []
    monkeypatch.setattr(controlled, 'file_hash', lambda path: seen.append(path) or 'h')
    assert p.current_file_hashes() == {str(f.resolve()): 'h'} and len(seen) == 1 and p.binding_evidence is None


@pytest.mark.parametrize('key', ['resident_workers', 'scene_handles', 'async_evidence', 'path_endpoint_reuse', 'binding_hash_cache'])
@pytest.mark.parametrize('value,ok', [(True, True), (False, True), (1, False), ('true', False)])
def test_step4_switch_configuration(key, value, ok):
    from unloading_sim.workbench.single_carton.config import load
    config = load(Path(__file__).resolve().parents[1]/'configs/workbench/single_carton.example.json', code_defaults=False)
    config[key] = value
    assert (not [e for e in issues(config) if e.startswith(key)]) == ok


def test_example_configuration_keeps_every_step4_switch_off():
    data = json.loads((Path(__file__).resolve().parents[1]/'configs/workbench/single_carton.example.json').read_text())
    assert not any(data.get(k) for k in ('resident_workers', 'scene_handles', 'async_evidence', 'path_endpoint_reuse',
                                         'binding_hash_cache', 'planning_deadline_s'))
