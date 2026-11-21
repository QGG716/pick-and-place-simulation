"""User decision 2026-11-21: OMPL fallback budget 5 s; the Pilz direct plan keeps its budget."""
from copy import deepcopy
import pytest
from test_single_carton_planner import case  # noqa: F401
from unloading_sim.workbench.single_carton.config import load, issues
from unloading_sim.workbench.ui import ROOT


def run_failing_approach(p, folder):
    seen = []
    class Native:
        def request(self, req, **kwargs):
            seen.append(dict(pipeline=req['pipeline_id'], budget=req['allowed_planning_time_s']))
            return dict(status='NATIVE_PLANNING_FAILED' if req['pipeline_id'] == 'pilz_industrial_motion_planner'
                        else 'NATIVE_SOLVER_BUDGET_EXHAUSTED')
    p.client = Native(); progress = []
    with pytest.raises(ValueError, match='NATIVE_SOLVER_BUDGET_EXHAUSTED'):
        p.checked_segment({'q_start': [0.]*6}, [.1]*6, 'approach', 10., folder, progress.append)
    return seen, progress


def test_ompl_budget_applies_only_to_the_ompl_fallback(case):
    p, obs, box, folder, calls, opts = case
    p.site['native_solver_budget_s'] = 25.; p.site['native_ompl_budget_s'] = 5.
    seen, progress = run_failing_approach(p, folder)
    assert seen == [dict(pipeline='pilz_industrial_motion_planner', budget=25.), dict(pipeline='ompl', budget=5.)]
    assert any('5 秒预算' in m for m in progress) and p.authority._plans == {}


def test_absent_ompl_budget_keeps_the_solver_budget(case):
    p, obs, box, folder, calls, opts = case
    p.site.pop('native_ompl_budget_s', None); p.site['native_solver_budget_s'] = 25.
    seen, _ = run_failing_approach(p, folder)
    assert [s['budget'] for s in seen] == [25., 25.]


def test_ompl_budget_configuration_is_validated():
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    ok = dict(deepcopy(c), native_ompl_budget_s=5., native_ompl_budget_basis='USER_DECISION_2026_11_21')
    assert not any('native_ompl_budget' in e for e in issues(ok))
    for bad in (0., -1., 101., float('nan'), True, '5'):
        assert any('native_ompl_budget_s' in e for e in issues(dict(ok, native_ompl_budget_s=bad)))
    assert any('native_ompl_budget_basis' in e for e in issues(dict(ok, native_ompl_budget_basis=' ')))
    assert any('native_ompl_budget_s' in e and 'NATIVE' in e for e in issues(dict(ok, planning_module='NATIVE')))
