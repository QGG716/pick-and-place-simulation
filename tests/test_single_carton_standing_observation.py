"""observation_mode=CONTINUOUS_STANDING and scene_build=SPLIT_PARALLEL (2026-11-25).

Fake devices only: no SDK, serial, camera, motion or valve command.
"""
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.contracts import DetectionResult, Stamp
from unloading_sim.workbench.single_carton.config import load, issues, observation_mode, scene_build
from unloading_sim.workbench.single_carton.live import LiveVision
from unloading_sim.workbench.single_carton.site_scene import build_scene, precompute_scene
from test_single_carton_observation import frame, observation
from test_single_carton_live import Model, exposure, until
from test_single_carton_partial_site import small_site

EXAMPLE = Path('configs/workbench/single_carton.example.json')
STANDING = dict(observation_mode='CONTINUOUS_STANDING',
                standing_observation=dict(refresh_s=30, basis='User decision 2026-11-25 (test)'))


def test_modes_default_unchanged_and_are_validated():
    c = load(EXAMPLE)
    assert observation_mode(c) == dict(mode='OPERATOR_BUTTON') and scene_build(c) == 'SEQUENTIAL'
    assert not [e for e in issues(c) if e.startswith(('observation_mode', 'scene_build'))]
    assert observation_mode(dict(c, **STANDING))['refresh_s'] == 30.
    assert observation_mode(dict(c, **STANDING))['joint_drift_deg'] == .02
    bad = [dict(STANDING, standing_observation=dict(refresh_s=120, basis='x')),   # not below plan validity
           dict(STANDING, standing_observation=dict(refresh_s=.5, basis='x')),
           dict(STANDING, standing_observation=dict(refresh_s=30, basis=' ')),
           dict(observation_mode='CONTINUOUS_STANDING'),
           dict(observation_mode='AUTO'),
           dict(standing_observation=dict(refresh_s=30, basis='x'))]          # settings without the mode
    for extra in bad:
        assert [e for e in issues(dict(c, **extra)) if e.startswith('observation_mode')], extra
    assert scene_build(dict(c, scene_build='SPLIT_PARALLEL')) == 'SPLIT_PARALLEL'
    assert [e for e in issues(dict(c, scene_build='PARALLEL')) if e.startswith('scene_build')]


def gate_case(cutoff_s):
    from unloading_sim.workbench.single_carton import perception
    class Clock:
        t = 10.
        def __call__(self): return self.t
    clock = Clock()
    class Backend:
        observation_generation = 3
        def sample_state(self, *a): return 'LIVE_VALID'
        @property
        def frame(self):
            return replace(frame(), stamp=replace(Stamp.receive(), received_monotonic_ns=int((clock()-.005)*1e9)))
    class Service:
        def snapshot(self):
            clock.t += .01
            return dict(source='LIVE', q=[0.]*6, received_s=clock(), sequence=clock(), alarm=False, enabled=True)
    class Model:
        def camera(self, q, c): return {'T_world_camera': np.eye(4)}
        def tcp(self, q): return np.eye(4)
    gate = perception.FreezeGate(Backend(), Service(), Model(), SimpleNamespace(parent_frame='link_1'),
                                 not_before_ns=int(cutoff_s*1e9))
    return gate, clock


def test_gate_is_the_freeze_criterion_and_ignores_samples_before_the_cutoff(monkeypatch):
    from unloading_sim.workbench.single_carton import perception
    gate, clock = gate_case(cutoff_s=10.3)
    monkeypatch.setattr(perception, 'monotonic_ns', lambda: int(clock()*1e9))
    found = None
    while found is None and clock() < 12:
        found = gate.poll(clock)
    # The 0.4 s stationary window may only start at the cutoff.
    assert found is not None and found.generation == 3
    assert min(h['received_s'] for h in found.stationary_samples) >= 10.3
    assert found.frame.stamp.received_monotonic_ns >= 10.3e9
    assert found.capture_checks['stationary'] and found.capture_checks['window_s'] >= .4


def test_standing_request_joins_the_running_or_queued_job_of_the_same_frame():
    model = Model(); model.gate.clear(); v = LiveVision({}, factory=lambda **_: model); v.start()
    out = {}
    try:
        first = exposure(1); v.offer(first, None, 0)
        until(lambda: v.current is not None and v.current[0].rgb_number == 1)
        t = threading.Thread(target=lambda: out.setdefault(1, v.infer_standing(first, 0)), daemon=True); t.start()
        until(lambda: 'frame-1' in v.attached)
        second = exposure(2); v.offer(second, None, 0)
        t2 = threading.Thread(target=lambda: out.setdefault(2, v.infer_standing(second, 0)), daemon=True); t2.start()
        until(lambda: 'frame-2' in v.attached)
        # A newer live frame never displaces the attached standing frame.
        v.offer(exposure(3), None, 0)
        assert v.pending[0].rgb_number == 2
        model.gate.set(); t.join(3); t2.join(3)
        assert model.seen == [1, 2]                 # no frame inferred twice
        assert out[1].observation_id == 'frame-1' and out[2].observation_id == 'frame-2'
        # A finished frame is answered from its result.
        assert v.infer_standing(second, 0) is out[2] and model.seen == [1, 2]
    finally:
        v.close(); model.gate.set()


def test_standing_request_is_invalidated_with_the_session():
    model = Model(); model.gate.clear(); v = LiveVision({}, factory=lambda **_: model); v.start()
    errors = []
    try:
        f = exposure(1); v.offer(f, None, 0); until(lambda: v.current is not None)
        def run():
            try: v.infer_standing(f, 0)
            except ValueError as e: errors.append(str(e))
        t = threading.Thread(target=run, daemon=True); t.start(); until(lambda: 'frame-1' in v.attached)
        v.invalidate(1); t.join(3)
        assert errors == ['OBSERVATION_INVALIDATED']
        with pytest.raises(ValueError, match='OBSERVATION_INVALIDATED'): v.infer_standing(f, 0)
    finally:
        v.close(); model.gate.set()


class BallFilter:
    """Pointwise synthetic self filter (a sphere), like RobotSelfFilter.contains."""
    def __init__(self, centre, radius): self.centre = np.asarray(centre); self.radius = radius
    def contains(self, points, q, group_size=1):
        inside = np.linalg.norm(np.asarray(points)-self.centre, axis=1) <= self.radius
        return inside.reshape(-1, group_size).all(1)
    def world_solids(self, q): return []


@pytest.mark.parametrize('with_filter', [False, True])
def test_split_scene_equals_sequential_scene(with_filter):
    obs = observation()
    result = DetectionResult(obs.frame.observation_id, obs.frame.stamp.source, 1, ())
    site = small_site(); sf = BallFilter([0, 0, .5], .03) if with_filter else None
    sequential = build_scene(obs, result, [], site, sf)
    pre = precompute_scene(obs.frame, obs.world_camera, obs.q_capture, site, sf)
    split = build_scene(obs, result, [], site, sf, precomputed=pre)
    assert json.dumps(sequential, sort_keys=True) == json.dumps(split, sort_keys=True)
    if with_filter: assert sequential['self_returns_removed'] > 0
    other = replace(obs, q_capture=(1.,)*6)
    with pytest.raises(ValueError, match='SCENE_PRECOMPUTE_MISMATCH'):
        build_scene(other, result, [], site, sf, precomputed=pre)
    with pytest.raises(ValueError, match='SCENE_PRECOMPUTE_SELF_FILTER_MISMATCH'):
        build_scene(obs, result, [], site, None if with_filter else BallFilter([0, 0, .5], .03), precomputed=pre)


def test_child_process_precompute_matches_in_process():
    from unloading_sim.workbench.single_carton.scene_precompute import ScenePrecompute
    obs = observation(); site = small_site()
    pool = ScenePrecompute(None, None)
    try:
        child = pool.submit(obs.frame, obs.world_camera, obs.q_capture, site).result(timeout=120)
    finally:
        pool.close()
    local = precompute_scene(obs.frame, obs.world_camera, obs.q_capture, site, None)
    assert child['identity'] == local['identity'] and child['self_filter_applied'] is False
    for key in ('sensor_free', 'self_pixels', 'self_cells'):
        np.testing.assert_array_equal(child[key], local[key])


# -- standing observation in the task manager -----------------------------------

def standing_manager(monkeypatch, tmp_path, q_offset_deg=0.):
    from unloading_sim.workbench.single_carton import runtime, site_scene, self_filter
    from unloading_sim.workbench.controlled.trajectory import Authority
    from eco65_test_inputs import use_synthetic_remote_detector
    obs = observation()
    c = use_synthetic_remote_detector(load(EXAMPLE), tmp_path)
    c.update(record_root=str(tmp_path/'records'), **STANDING)
    result = SimpleNamespace(detections=[dict(label='box')], masks=[np.ones(obs.frame.rgb.shape[:2], bool)],
                             observation_id=obs.frame.observation_id)
    vision = SimpleNamespace(infer_standing=lambda *a, **k: result)
    backend = SimpleNamespace(vision=vision, observation_generation=obs.generation)
    planner = SimpleNamespace(authority=Authority('eco65_single_carton_segment_v1'), build=lambda: None,
                              init=dict(parameters=dict(robot_description='fake')), tracked=[])
    planner.track = planner.tracked.append
    measured = [dict(id='measured-target', detection_index=0)]
    monkeypatch.setattr(runtime, 'reconstruct', lambda *a, **k: (measured, []))
    monkeypatch.setattr(site_scene, 'build_scene', lambda *a, **k: dict(world=[dict(id='observed-obstacle')]))
    monkeypatch.setattr(self_filter, 'RobotSelfFilter', lambda *a: None)
    from unloading_sim.workbench.single_carton import site_layout
    def layout(scene, observation, result, config, planner, self_filter, record):
        planner.track('layout.step'); record('layout-registration.json', dict(status='PASS')); return scene
    monkeypatch.setattr(site_layout, 'merge_registered_layout', layout)
    service = SimpleNamespace(snapshot=lambda: dict(source='LIVE', q=list(np.deg2rad([q_offset_deg]+[0]*5)),
                                                    received_s=time.monotonic(), sequence=1, alarm=False, enabled=True))
    manager = runtime.TaskManager(backend, service, SimpleNamespace(holding=False), planner, None, None, c)
    return manager, obs, planner


def test_standing_build_installs_without_records_and_binding_writes_them(monkeypatch, tmp_path):
    manager, obs, planner = standing_manager(monkeypatch, tmp_path)
    try:
        observer = manager.standing_observer
        observer._build(obs, forced=False)
        assert manager.phase == 'TARGET_READY' and manager.observation is obs and manager.standing is not None
        assert manager.folder is None and not (tmp_path/'records').exists()     # nothing written before binding
        assert planner.tracked == []                                            # tracking deferred to the binding
        s = manager._bind_standing()
        # async_evidence (a code default since 2026-11-25): prepare_plan flushes before PLAN_READY.
        manager.evidence_flush(); manager.journal.flush()
        files = {p.name for p in manager.folder.iterdir()}
        assert {'observation.json', 'observation.npz', 'detections.json', 'segmentation.npz', 'site-scene.json',
                'geometry.json', 'layout-registration.json', 'result.json'} <= files
        assert planner.tracked == ['layout.step'] and manager.standing is None and s.observation is obs
        events = [json.loads(line) for line in (manager.folder/'events.jsonl').read_text().splitlines()] \
            if (manager.folder/'events.jsonl').exists() else None
        if events is not None:
            kinds = [e.get('kind') for e in events]
            assert 'standing_observation' in kinds and 'perception' in kinds
    finally:
        manager.closing.set()


def test_binding_refuses_a_moved_robot_and_drops_the_standing(monkeypatch, tmp_path):
    manager, obs, _ = standing_manager(monkeypatch, tmp_path, q_offset_deg=.03)
    try:
        manager.standing_observer._build(obs, forced=False)
        with pytest.raises(ValueError, match='STANDING_OBSERVATION_ROBOT_MOVED'): manager._bind_standing()
        assert manager.standing is None and manager.folder is None
    finally:
        manager.closing.set()


def test_drift_drops_the_standing_and_requests_a_new_one(monkeypatch, tmp_path):
    manager, obs, _ = standing_manager(monkeypatch, tmp_path, q_offset_deg=.03)
    try:
        manager.standing_observer._build(obs, forced=False)
        assert manager.standing is not None
        assert manager.standing_observer._needs_refresh(time.monotonic()) == '机械臂已移动'
        assert manager.standing is None and manager.observation is None and manager.phase == 'IDLE'
    finally:
        manager.closing.set()


def test_pause_rules_protect_tasks_loads_stops_and_the_reset_start(monkeypatch, tmp_path):
    manager, obs, _ = standing_manager(monkeypatch, tmp_path)
    try:
        assert manager.standing_pause_reason() is None
        manager.busy = True; assert manager.standing_pause_reason(); manager.busy = False
        manager.vacuum.holding = True; assert manager.standing_pause_reason(force=True); manager.vacuum.holding = False
        manager.cycle = SimpleNamespace(active=False, state='STOP_UNCONFIRMED')
        assert manager.standing_pause_reason(force=True)
        manager.cycle = SimpleNamespace(active=False, state='FAILED')
        assert manager.standing_pause_reason() and manager.standing_pause_reason(force=True) is None
        manager.cycle = SimpleNamespace(active=False, state='COMPLETE')
        assert manager.standing_pause_reason() is None
        # A ready plan is kept while it is still valid.
        manager.cycle = SimpleNamespace(active=False, state='PLAN_READY'); manager.observation = obs
        assert manager.standing_pause_reason() == '计划就绪，等待执行'
        # Installing is refused while paused; nothing of the task changes.
        manager.cycle = SimpleNamespace(active=True, state='APPROACH')
        observer = manager.standing_observer
        observer._build(obs, forced=False)
        assert manager.standing is None and manager.cycle.state == 'APPROACH'
    finally:
        manager.closing.set()


def test_operator_button_mode_never_starts_a_standing_observer(monkeypatch, tmp_path):
    manager, _, _ = standing_manager(monkeypatch, tmp_path)
    manager.closing.set()
    from unloading_sim.workbench.single_carton import runtime
    c = dict(manager.config); c.pop('observation_mode'); c.pop('standing_observation')
    plain = runtime.TaskManager(manager.backend, manager.service, SimpleNamespace(holding=False), manager.planner, None, None, c)
    try:
        assert plain.standing_observer is None and plain.standing_status() is None
        with pytest.raises(ValueError, match='STANDING_OBSERVATION_NOT_CONFIGURED'): plain.request_standing_refresh()
    finally:
        plain.closing.set()
