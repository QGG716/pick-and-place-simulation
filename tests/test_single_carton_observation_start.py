"""observation_start (user decision 2026-11-25): the "移到观测起点" goal, validated only."""
from pathlib import Path
import pytest
from unloading_sim.workbench.single_carton.config import load, issues, observation_start

EXAMPLE = Path('configs/workbench/single_carton.example.json')
GOAL = dict(q_deg=[68.7, 8.6, -93.4, 78.3, 95.8, -8.9], basis='User decision 2026-11-25 (test)')


def test_absent_is_none_and_valid_goal_is_returned():
    c = load(EXAMPLE, code_defaults=False)
    assert observation_start(c) is None
    assert observation_start(dict(c, observation_start=GOAL)) == GOAL
    assert not [e for e in issues(dict(c, observation_start=GOAL)) if e.startswith('observation_start')]


@pytest.mark.parametrize('bad', [dict(GOAL, basis=''), dict(GOAL, q_deg=GOAL['q_deg'][:5]),
                                 dict(GOAL, q_deg=[68.7, 8.6, -93.4, 78.3, 95.8, 400.]),
                                 dict(GOAL, q_deg=[68.7, 150., -93.4, 78.3, 95.8, -8.9]), dict(GOAL, extra=1)])
def test_invalid_goal_is_a_configuration_error(bad):
    c = load(EXAMPLE, code_defaults=False)
    with pytest.raises(ValueError, match='OBSERVATION_START_INVALID'):observation_start(dict(c, observation_start=bad))
    assert [e for e in issues(dict(c, observation_start=bad)) if e.startswith('observation_start')]
