"""Step-5 Python side of collision_check_mode; fake native workers only."""
from pathlib import Path
import pytest
from test_single_carton_planner import case  # noqa: F401  (fixture reuse)
from unloading_sim.workbench.single_carton.config import issues, load

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('value,ok', [(None, True), ('EXACT', True), ('TWO_LEVEL', True), ('two_level', False), ('FAST', False)])
def test_collision_check_mode_configuration(value, ok):
    config = load(ROOT/'configs/workbench/single_carton.example.json')
    if value is None:
        config.pop('collision_check_mode', None)
    else:
        config['collision_check_mode'] = value
    assert (not [e for e in issues(config) if e.startswith('collision_check_mode')]) == ok


def test_exact_default_sends_no_mode_field(case):
    p, obs, box, folder, calls, opts = case
    p.plan_cycle(obs, [box], folder)
    assert not any('collision_check_mode' in r for r in calls)


def test_two_level_is_requested_on_every_collision_check_with_the_capability(case):
    p, obs, box, folder, calls, opts = case
    p.site['collision_check_mode'] = 'TWO_LEVEL'
    opts['capabilities'] = dict(two_level_collision=True)
    p.plan_cycle(obs, [box], folder)
    checks = [r for r in calls if r.get('op') in ('validate', 'plan')]
    assert checks and all(r.get('collision_check_mode') == 'TWO_LEVEL' for r in checks)
    assert not any('collision_check_mode' in r for r in calls if r.get('op') == 'ik')


def test_two_level_without_worker_capability_is_refused_before_any_check(case):
    p, obs, box, folder, calls, opts = case
    p.site['collision_check_mode'] = 'TWO_LEVEL'
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_TWO_LEVEL_COLLISION'):
        p.plan_cycle(obs, [box], folder)
    assert not [r for r in calls if r.get('op') in ('validate', 'plan', 'ik')]


def test_example_configuration_keeps_exact():
    import json
    data = json.loads((ROOT/'configs/workbench/single_carton.example.json').read_text())
    assert data.get('collision_check_mode', 'EXACT') == 'EXACT'
