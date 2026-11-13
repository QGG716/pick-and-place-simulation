"""Execute-time visual recheck policy; fake objects only, no device or motion."""
from pathlib import Path
from types import SimpleNamespace
import pytest
from unloading_sim.workbench.single_carton import runtime
from unloading_sim.workbench.single_carton.config import issues, load, pre_execute_visual_recheck
from unloading_sim.workbench.single_carton.runtime import TaskManager

ROOT = Path(__file__).resolve().parents[1]


def manager(tmp_path, policy, *, generation=1, binding_error=None):
    events = []; started = []; saved = []
    class Journal:
        error = None; thread = SimpleNamespace(is_alive=lambda: True)
        def __call__(self, **e): events.append(e)
    def binding():
        if binding_error: raise ValueError(binding_error)
        return {}
    m = SimpleNamespace(
        cycle=SimpleNamespace(state='PLAN_READY', active=False, start=lambda: started.append(1)),
        journal=Journal(), _enter=lambda: None, busy=True, phase='PLAN_READY',
        planner=SimpleNamespace(authority=SimpleNamespace(invalidate=lambda: None), binding=binding),
        observation=SimpleNamespace(generation=1), backend=SimpleNamespace(observation_generation=generation),
        stop_event=SimpleNamespace(is_set=lambda: False), config=dict(policy), progress=lambda text: events.append(dict(progress=text)),
        save_observation=lambda o, name: saved.append(name), result=SimpleNamespace(masks=[None]), plan=dict(target=dict(detection_index=0)),
        service=None, model=None, calibration=None, rejected=[])
    m.reject = lambda e: m.rejected.append(str(e))
    return m, events, started, saved


DISABLED = dict(pre_execute_visual_recheck='DISABLED_BY_USER', pre_execute_visual_recheck_basis='SYNTHETIC_USER_INSTRUCTION')


def test_disabled_recheck_never_recaptures_and_starts_the_cycle(tmp_path, monkeypatch):
    m, events, started, saved = manager(tmp_path, DISABLED)
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: pytest.fail('no visual re-capture when disabled'))
    TaskManager.execute(m)
    assert started == [1] and saved == [] and m.busy is False
    record = next(e for e in events if e.get('kind') == 'pre_execute_recheck')['evidence']
    assert record['status'] == 'VISUAL_RECHECK_DISABLED_BY_USER' and record['basis'] == 'SYNTHETIC_USER_INSTRUCTION'
    assert 'FRESH_LIVE_JOINTS_WITHIN_0_01_DEG_OF_Q_START_ENABLED_NO_ALARM' in record['retained']


@pytest.mark.parametrize('generation,binding_error,reason', [(2, None, 'OBSERVATION_INVALIDATED'), (1, 'FILE_CHANGED_REPLAN', 'FILE_CHANGED_REPLAN')])
def test_disabled_recheck_keeps_session_and_file_binding_interlocks(tmp_path, monkeypatch, generation, binding_error, reason):
    m, events, started, saved = manager(tmp_path, DISABLED, generation=generation, binding_error=binding_error)
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: pytest.fail('no visual re-capture when disabled'))
    with pytest.raises(ValueError, match=reason): TaskManager.execute(m)
    assert started == [] and m.rejected


def test_enabled_recheck_saves_the_new_frame_even_when_the_scene_check_rejects(tmp_path, monkeypatch):
    m, events, started, saved = manager(tmp_path, {})
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: SimpleNamespace(frame='new'))
    def changed(a, b): raise ValueError('TARGET_OR_SCENE_CHANGED_REPLAN')
    monkeypatch.setattr(runtime, 'unchanged', changed)
    with pytest.raises(ValueError, match='TARGET_OR_SCENE_CHANGED_REPLAN'): TaskManager.execute(m)
    assert saved == ['pre_execute_observation'] and started == [] and m.rejected


def test_enabled_recheck_compares_scene_and_target_before_start(tmp_path, monkeypatch):
    m, events, started, saved = manager(tmp_path, dict(pre_execute_visual_recheck='ENABLED'))
    calls = []
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: SimpleNamespace(frame='new'))
    monkeypatch.setattr(runtime, 'unchanged', lambda a, b: calls.append('scene') or dict(status='PASS'))
    monkeypatch.setattr(runtime, 'target_unchanged', lambda a, b, mask: calls.append('target'))
    TaskManager.execute(m)
    assert calls == ['scene', 'target'] and started == [1]
    assert next(e for e in events if e.get('kind') == 'pre_execute_recheck')['evidence'] == dict(status='PASS')


@pytest.mark.parametrize('value,basis,ok', [(None, None, True), ('ENABLED', None, True), ('DISABLED_BY_USER', 'x', True),
                                            ('DISABLED_BY_USER', '', False), ('DISABLED_BY_USER', None, False), ('OFF', 'x', False),
                                            ('TARGET_ONLY', 'x', True), ('TARGET_ONLY', None, False)])
def test_policy_values_are_explicit(value, basis, ok):
    c = {} if value is None else dict(pre_execute_visual_recheck=value)
    if basis is not None: c['pre_execute_visual_recheck_basis'] = basis
    if ok: assert pre_execute_visual_recheck(c)['enabled'] == (value != 'DISABLED_BY_USER')
    else:
        with pytest.raises(ValueError, match='PRE_EXECUTE_RECHECK'): pre_execute_visual_recheck(c)


def test_field_config_disables_visual_recheck_with_the_user_instruction_on_record():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    policy = pre_execute_visual_recheck(c)
    assert policy['enabled'] is False and '6b648ffcdb35403cb9d992e2ed8554be' in policy['basis']
    assert not [e for e in issues(c) if e.startswith('pre_execute_visual_recheck')]


def test_target_only_skips_the_whole_scene_comparison_but_keeps_target_and_start(tmp_path, monkeypatch):
    m, events, started, saved = manager(tmp_path, dict(pre_execute_visual_recheck='TARGET_ONLY', pre_execute_visual_recheck_basis='SYNTHETIC'))
    calls = []
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: SimpleNamespace(frame='new'))
    monkeypatch.setattr(runtime, 'unchanged', lambda a, b: pytest.fail('whole-scene comparison must be skipped'))
    monkeypatch.setattr(runtime, 'same_session_start_and_camera', lambda a, b: calls.append('start'))
    monkeypatch.setattr(runtime, 'target_unchanged', lambda a, b, mask: calls.append('target'))
    TaskManager.execute(m)
    assert calls == ['start', 'target'] and started == [1] and saved == ['pre_execute_observation']
    record = next(e for e in events if e.get('kind') == 'pre_execute_recheck')['evidence']
    assert record['scope'] == 'TARGET_ONLY' and record['whole_scene_comparison'] == 'SKIPPED_BY_CONFIGURATION'


@pytest.mark.parametrize('failure', ['START_CHANGED_REPLAN', 'LOCKED_TARGET_CHANGED_REPLAN'])
def test_target_only_still_rejects_a_moved_start_or_target(tmp_path, monkeypatch, failure):
    m, events, started, saved = manager(tmp_path, dict(pre_execute_visual_recheck='TARGET_ONLY', pre_execute_visual_recheck_basis='SYNTHETIC'))
    def fail(*a): raise ValueError(failure)
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: SimpleNamespace(frame='new'))
    monkeypatch.setattr(runtime, 'same_session_start_and_camera', fail if failure.startswith('START') else (lambda a, b: None))
    monkeypatch.setattr(runtime, 'target_unchanged', fail if failure.startswith('LOCKED') else (lambda a, b, mask: None))
    with pytest.raises(ValueError, match=failure): TaskManager.execute(m)
    assert started == [] and m.rejected


# --- Start joint check ---------------------------------------------------------
import numpy as np
from unloading_sim.workbench.controlled.engine import MonitorConfig
from unloading_sim.workbench.single_carton.config import start_joint_check
from test_single_carton_cycle import make
from test_single_carton_review_fixes import segment
from test_controlled_execution import Harness

WAIVER = dict(enabled=False, status='START_JOINT_CHECK_DISABLED_BY_USER', basis='SYNTHETIC_USER_INSTRUCTION')


def test_cycle_start_check_is_enforced_by_default_and_recorded():
    c, t, r, v, records, plan = make(); r.q = [np.deg2rad(.02)]+[0.]*5
    with pytest.raises(ValueError, match='START_CHANGED_REPLAN'): c.start()
    assert not r.calls
    check = next(e for e in records if e['kind'] == 'start_joint_check')
    assert check['limit_deg'] == .01 and check['max_difference_deg'] == pytest.approx(.02)


def test_cycle_start_check_disabled_by_user_starts_and_records_the_difference():
    c, t, r, v, records, plan = make(); plan['start_joint_check'] = WAIVER; c.ready('one-box', plan)
    r.q = [np.deg2rad(.02)]+[0.]*5
    c.start(); assert len(r.calls) == 1 and c.state == 'APPROACH'
    check = next(e for e in records if e['kind'] == 'start_joint_check')
    assert check['limit_deg'] is None and check['policy']['basis'] == 'SYNTHETIC_USER_INSTRUCTION'


@pytest.mark.parametrize('field', ['alarm', 'stale'])
def test_disabled_start_check_still_needs_fresh_alarm_free_feedback(field):
    c, t, r, v, records, plan = make(); plan['start_joint_check'] = WAIVER; c.ready('one-box', plan)
    if field == 'alarm': r.alarm = True
    else: r.age = 1.
    with pytest.raises(ValueError, match='LIVE_ROBOT_STALE_DISCONNECTED_OR_FAULT'): c.start()
    assert not r.calls


def test_engine_waives_only_a_package_bound_user_disabled_start_check():
    h = Harness(); first = 0.; rest = [first+np.deg2rad(.02)]*6
    with pytest.raises(ValueError, match='START_CHANGED_REPLAN_REQUIRED'): h.engine.submit(segment(first), h.sample(rest))
    for bad in (dict(WAIVER, basis=''), dict(WAIVER, status='SOMETHING_ELSE'), 'DISABLED'):
        prepared = segment(first); prepared['start_joint_check'] = bad
        with pytest.raises(ValueError, match='START_CHANGED_REPLAN_REQUIRED'): h.engine.submit(prepared, h.sample(rest))
    prepared = segment(first); prepared['start_joint_check'] = WAIVER
    assert h.engine.submit(prepared, h.sample(rest))
    accepted = [e for e in h.logs if e['kind'] == 'accepted'][-1]
    assert accepted['start_rule'] == 'USER_DISABLED_START_JOINT_CHECK' and accepted['start_joint_check'] == WAIVER
    assert accepted['start_error_rad'] == pytest.approx(np.deg2rad(.02))


def test_waived_start_still_requires_a_stationary_robot():
    h = Harness(); prepared = segment(0.); prepared['start_joint_check'] = WAIVER
    sample = h.sample([np.deg2rad(.02)]*6); sample['velocity'] = [np.deg2rad(1.)]*6
    with pytest.raises(ValueError, match='START_NOT_STATIONARY'): h.engine.submit(prepared, sample)


@pytest.mark.parametrize('value,basis,ok', [(None, None, True), ('ENABLED', None, True), ('DISABLED_BY_USER', 'x', True),
                                            ('DISABLED_BY_USER', '', False), ('OFF', 'x', False)])
def test_start_check_policy_values_are_explicit(value, basis, ok):
    c = {} if value is None else dict(start_joint_check=value)
    if basis is not None: c['start_joint_check_basis'] = basis
    if ok: assert start_joint_check(c)['enabled'] == (value != 'DISABLED_BY_USER')
    else:
        with pytest.raises(ValueError, match='START_JOINT_CHECK'): start_joint_check(c)


def test_field_config_disables_start_check_with_the_user_instruction_on_record():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    policy = start_joint_check(c)
    assert policy['enabled'] is False and '71ce26a3e9cf458e906b065bcb8275e7' in policy['basis']
    assert MonitorConfig().start_rad == pytest.approx(np.deg2rad(.01))  # controlled-empty default unchanged
