"""Fixes after the 2026-11-14 field incident (tool struck J6 during approach).

Execution lock, larger tool-to-robot clearance and wrist-branch limits.
Fake native responses and fake devices only; nothing moves.
"""
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import runtime
from unloading_sim.workbench.single_carton.config import (load, issues, execution_lock, tool_robot_clearance,
                                                          wrist_branch_limits, wrist_branch_violation)
from unloading_sim.workbench.single_carton.geometry import contact_candidates
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.runtime import TaskManager
from test_controlled_execution import Harness
from test_single_carton_planner import case  # noqa: F401  (fixture)
from test_single_carton_pre_execute import manager
from test_single_carton_review_fixes import segment

ROOT = Path(__file__).resolve().parents[1]
LOCK = dict(locked=True, basis='SYNTHETIC_INCIDENT')
LIMITS = dict(keep_j5_sign=True, j5_min_abs_deg=10., max_j4_travel_deg=90., max_j6_travel_deg=180., basis='SYNTHETIC')


# --- configuration ----------------------------------------------------------------

def test_field_config_is_locked_with_40_mm_tool_clearance_and_wrist_limits():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert not [e for e in issues(c) if e.split('：')[0] in ('execution_lock', 'tool_robot_clearance', 'wrist_branch_limits')]
    assert execution_lock(c)['locked'] and 'd99b1a8f5d71445395c158b3c353ca42' in execution_lock(c)['basis']
    t = tool_robot_clearance(c)
    assert t['clearance_m'] == .04 and t['exempt_robot_links'] == ['link_6']
    assert t['exempt_pairs'] == [['part_041_region_1', 'link_5'], ['part_041_region_2', 'link_5']]
    w = wrist_branch_limits(c)
    assert not w['keep_j5_sign'] and w['j5_min_abs_deg'] == 0.
    assert w['max_j4_travel_deg'] is None and w['max_j6_travel_deg'] == 180.
    assert 'USER_REQUEST_2026_11_18' in w['basis']


@pytest.mark.parametrize('lock,ok,locked', [(None, True, False), (dict(locked=False), True, False), (LOCK, True, True),
                                            (dict(locked=True), False, None), (dict(locked='yes', basis='x'), False, None), ('ON', False, None)])
def test_execution_lock_values(lock, ok, locked):
    c = {} if lock is None else dict(execution_lock=lock)
    if ok: assert bool(execution_lock(c)) == locked
    else:
        with pytest.raises(ValueError, match='EXECUTION_LOCK'): execution_lock(c)


@pytest.mark.parametrize('value', [dict(clearance_m=.001, basis='x'), dict(clearance_m=.2, basis='x'), dict(clearance_m=.04),
                                   dict(clearance_m=.04, basis='x', exempt_pairs=[['link_5', 'part_041_region_1']])])
def test_tool_robot_clearance_rejects_bad_values_and_misordered_exemptions(value):
    with pytest.raises(ValueError, match='TOOL_ROBOT_CLEARANCE'):
        tool_robot_clearance(dict(tool_robot_clearance=value), tool_links=['part_041_region_1', 'part_000'])


def test_tool_robot_clearance_exempt_link_must_be_a_robot_link():
    with pytest.raises(ValueError, match='NOT_TOOL_ROBOT'):
        tool_robot_clearance(dict(tool_robot_clearance=dict(clearance_m=.04, basis='x', exempt_robot_links=['part_000'])), tool_links=['part_000'])


# --- wrist branch -------------------------------------------------------------------

def deg(*q): return np.deg2rad(q).tolist()


START = deg(87, 16, -69, -31, 87, 77)


@pytest.mark.parametrize('states,reason', [
    ([deg(20, 3, -65, -12, 58, -90)], None),                       # earlier non-flipping approach
    ([deg(55, 9, -67, -71, 19, 42), deg(20, 3, -65, -117, -58, -1)], 'J5_WRIST_FLIP_OR_SINGULAR'),  # the incident
    ([deg(50, 9, -67, -60, 8, 40)], 'J5_WRIST_FLIP_OR_SINGULAR'),  # inside the singular band without a sign change
    ([deg(20, 3, -65, -125, 58, 77)], 'J4_TRAVEL_EXCEEDED'),
    ([deg(20, 3, -65, -31, 58, -110)], 'J6_TRAVEL_EXCEEDED')])
def test_wrist_branch_violation_literal_cases(states, reason):
    v = wrist_branch_violation(LIMITS, START, states)
    assert (v or {}).get('reason') == reason


def test_no_wrist_limits_configured_means_no_restriction():
    assert wrist_branch_violation(None, START, [deg(0, 0, 0, 0, -50, 0)]) is None


def test_segment_whose_sent_path_flips_the_wrist_is_rejected(case):
    p, obs, box, folder, calls, opts = case
    from unloading_sim.workbench.single_carton.planner import SegmentRejected
    p.wrist_limits = LIMITS; p.wrist_reference = [0., 0., 0., 0., np.deg2rad(20), 0.]
    class Native:
        def request(self, req, **kwargs):
            return dict(status='SUCCESS', native_output_status='PASS',
                        points=[dict(t=0., q=req['q_start'], v=[0.]*6, a=[0.]*6), dict(t=8., q=deg(0, 0, 0, 0, -20, 0), v=[0.]*6, a=[0.]*6)])
    p.client = Native()
    with pytest.raises(SegmentRejected, match='WRIST_BRANCH_REJECTED') as error:
        p.plan_segment({'q_start': [0., 0., 0., 0., np.deg2rad(20), 0.]}, deg(0, 0, 0, 0, -20, 0), 'approach', 10., folder, lambda _: None)
    assert error.value.native_result['failure']['reason'] == 'J5_WRIST_FLIP_OR_SINGULAR'


def test_endpoint_ik_drops_branches_on_the_other_wrist_side(case):
    p, obs, box, folder, calls, opts = case
    p.wrist_limits = LIMITS; p.wrist_reference = START
    same, flipped = deg(20, 3, -65, -12, 58, -90), deg(20, 3, -65, -117, -58, -1)
    class Native:
        def request(self, req, **kwargs): return dict(status='SUCCESS', q=flipped if req['q_start'][0] > 1 else same)
    p.client = Native()
    solutions, _ = p.endpoint_ik(np.eye(4), [START, [0.]*6], folder, 'approach-endpoint-ik', q_reference=START)
    assert solutions == [same]
    record = json.loads((folder/'approach-endpoint-ik-native.json').read_text())
    assert record['wrist_branch_rejected'] == [flipped]


# --- planner binding ------------------------------------------------------------------

def test_lock_is_bound_into_every_segment_and_recorded_in_the_plan(case, monkeypatch):
    p, obs, box, folder, calls, opts = case
    real = contact_candidates
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates', lambda *a: real(*a)[:1])
    p.site['execution_lock'] = LOCK
    plan = p.plan_cycle(obs, [box], folder)
    assert plan['execution_lock']['locked'] and all(s['package']['prepared']['execution_lock'] == plan['execution_lock'] for s in plan['segments'])


def test_worker_without_tool_robot_clearance_support_is_refused():
    p = SinglePlanner.__new__(SinglePlanner); p.clearance = dict(tool_robot_clearance=dict(clearance_m=.04))
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_TOOL_ROBOT_CLEARANCE'):
        p.worker_ready(dict(capabilities=dict(bounded_single_carton_planning=True)))


# --- engine and runtime ------------------------------------------------------------------

@pytest.mark.parametrize('marker', [LOCK, dict(locked=False), 'anything'])
def test_engine_refuses_any_package_carrying_a_lock_marker_and_stays_usable(marker):
    h = Harness(); prepared = segment(0.); prepared['execution_lock'] = marker
    with pytest.raises(ValueError, match='EXECUTION_LOCKED_BY_USER'): h.engine.submit(prepared, h.sample())
    assert h.engine.state == 'READY' and h.io.sent == []
    assert any(e['kind'] == 'submit_refused' for e in h.logs)
    # Stop paths are untouched and an unlocked package still runs normally.
    assert h.engine.submit(segment(0.), h.sample()) and h.engine.state == 'EXECUTING'
    h.engine.cancel(); assert h.engine.state == 'STOPPING'


def test_runtime_execute_refuses_a_locked_configuration_before_any_capture(tmp_path, monkeypatch):
    m, events, started, saved = manager(tmp_path, dict(execution_lock=LOCK))
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: pytest.fail('no capture when locked'))
    with pytest.raises(ValueError, match='EXECUTION_LOCKED_BY_USER'): TaskManager.execute(m)
    assert started == [] and m.rejected and m.busy is False
    assert next(e for e in events if e.get('kind') == 'execution_refused')['execution_lock']['basis'] == 'SYNTHETIC_INCIDENT'


def test_runtime_execute_refuses_a_plan_made_while_locked_even_if_config_is_unlocked(tmp_path, monkeypatch):
    m, events, started, saved = manager(tmp_path, {}); m.plan = dict(m.plan, execution_lock=dict(locked=True, basis='PLANNED_WHILE_LOCKED'))
    monkeypatch.setattr(runtime, 'freeze', lambda *a, **k: pytest.fail('no capture when locked'))
    with pytest.raises(ValueError, match='EXECUTION_LOCKED_BY_USER'): TaskManager.execute(m)
    assert started == []
