"""User decisions 2026-11-21: J6 travel gate may be disabled; soft J4->J5 toward -Z preference."""
from copy import deepcopy
import math
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import posture
from unloading_sim.workbench.single_carton.config import load, issues, wrist_branch_limits, wrist_branch_violation
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.workbench.ui import ROOT, URDF, URDF_SHA

# Saved IK branches of one approach pose (field frame cda4e336, candidate 14):
# identical TCP pose, J5 negative (J4->J5 down) and J5 positive (J4->J5 up).
WRIST_DOWN = np.deg2rad([17.4, -0.08, -82.76, -99.90, -62.00, -88.91])
WRIST_UP = np.deg2rad([17.4, -15.53, -123.18, 135.98, 62.00, 91.09])
START = np.deg2rad([81.43, -13.06, -55.02, -15.76, 85.99, -5.15])


@pytest.fixture(scope='module')
def setup():
    c = load(ROOT/'configs/workbench/single_carton.example.json'); i = Installation.load(c['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, i.world_from_base, i.tool)
    return c, model


def with_weight(c, w=2.):
    c = deepcopy(c); c['posture_preference'] = dict(c['posture_preference'], wrist_down_weight=w); return c


def test_j6_travel_gate_can_be_disabled_alone(setup):
    c, _ = setup
    limits = wrist_branch_limits(dict(wrist_branch_limits=dict(c['wrist_branch_limits'], max_j6_travel_deg=None)))
    states = [START.copy() for _ in range(2)]; states[-1][5] += np.deg2rad(300.)
    assert wrist_branch_violation(limits, START, states) is None
    # The other gates stay in force when configured.
    strict = dict(limits, keep_j5_sign=True, j5_min_abs_deg=10., max_j4_travel_deg=90.)
    flip = [START.copy()]; flip[0][4] = -START[4]
    assert wrist_branch_violation(strict, START, flip)['reason'] == 'J5_WRIST_FLIP_OR_SINGULAR'
    turn = [START.copy()]; turn[0][3] += np.deg2rad(95.)
    assert wrist_branch_violation(strict, START, turn)['reason'] == 'J4_TRAVEL_EXCEEDED'
    for bad in (-1., float('nan'), 'none'):
        with pytest.raises(ValueError):
            wrist_branch_limits(dict(wrist_branch_limits=dict(c['wrist_branch_limits'], max_j6_travel_deg=bad)))


def test_native_composite_refuses_python_only_options(setup):
    c, _ = setup
    j6 = deepcopy(c); j6['wrist_branch_limits'] = dict(c['wrist_branch_limits'], max_j6_travel_deg=None)
    assert not any('max_j6_travel_deg' in e for e in issues(j6))
    assert any('max_j6_travel_deg' in e for e in issues(dict(j6, planning_module='NATIVE')))
    w = with_weight(c)
    assert not any('wrist_down_weight' in e for e in issues(w))
    assert any('wrist_down_weight' in e for e in issues(dict(w, planning_module='NATIVE')))


def test_wrist_down_weight_is_optional_and_validated(setup):
    c, model = setup
    base = posture.bind(c, model)
    assert 'wrist_links' not in base and 'wrist_from_down_deg' not in posture.metrics(base, WRIST_UP, START)
    p = posture.bind(with_weight(c), model)
    assert p['wrist_links'] == ['link_4', 'link_5'] and p['hard_constraint'] is False
    for bad in (0., -1., float('nan'), 101., True):
        with pytest.raises(ValueError):
            posture.settings(with_weight(c, bad))
    with pytest.raises(ValueError):
        posture.settings(dict(posture_preference=dict(c['posture_preference'], wrist_up_weight=1.)))


def test_wrist_down_term_matches_fk_and_only_adds_to_the_score(setup):
    c, model = setup
    base = posture.bind(c, model); p = posture.bind(with_weight(c, 2.), model)
    for q in (WRIST_DOWN, WRIST_UP):
        f = model.frames(q); v = f['link_5'][:3, 3]-f['link_4'][:3, 3]
        assert np.linalg.norm(v) == pytest.approx(.11, abs=1e-6)
        expected = math.degrees(math.acos(-v[2]/np.linalg.norm(v)))
        m = posture.metrics(p, q, START); m0 = posture.metrics(base, q, START)
        assert m['wrist_from_down_deg'] == pytest.approx(expected, abs=1e-9)
        assert m['score'] == pytest.approx(m0['score']+2.*(1-math.cos(math.radians(expected))), abs=1e-12)
        assert {k: m[k] for k in m0 if k != 'score'} == {k: m0[k] for k in m0 if k != 'score'}
    assert posture.metrics(p, WRIST_DOWN, START)['wrist_from_down_deg'] < 5.
    assert posture.metrics(p, WRIST_UP, START)['wrist_from_down_deg'] > 170.
    assert posture.key(p, WRIST_DOWN, START) < posture.key(p, WRIST_UP, START)


def test_wrist_down_preference_reorders_but_never_rejects(setup):
    c, model = setup
    p = posture.bind(with_weight(c, 2.), model)
    # A branch with a worse arm posture but the wrist down can win; every branch
    # still gets a finite score (soft preference, not a state gate).
    relaxed = WRIST_DOWN.copy(); relaxed[1] += np.deg2rad(12.)
    keys = [posture.key(p, q, START) for q in (WRIST_UP, relaxed, WRIST_DOWN)]
    assert all(np.isfinite(k[0]) for k in keys)
    assert keys[2] < keys[1] < keys[0]
