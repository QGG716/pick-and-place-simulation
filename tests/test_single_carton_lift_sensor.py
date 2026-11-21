"""Read-only draw-wire lift sensor: protocol-agnostic calibration and freeze.
Fake samples only; no port is opened and nothing is written."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.workbench.single_carton.lift_sensor import (CHECKED, LiftSample, freeze_lift, overall_height,
                                                               sensor_config)

ROOT = Path(__file__).resolve().parents[1]
REAL = ROOT / 'assets/layout/derived/a4b6ed1298f7491b/site-layout.json'
ROLES = ROOT / 'configs/workcells/eco65_site_layout_roles_v1.yaml'


class Mechanism:
    """CAD overall height 346 mm, machine range 160-300 mm -> h -0.186..-0.046 m."""
    overall_height_cad_m = 0.346
    range_h_m = [-0.186, -0.046]

    def state(self, h):
        if not -0.346 < h < 0.258:
            raise ValueError('SITE_MECHANISM_LIFT_OUT_OF_CHAIN_RANGE')
        return {}


SENSOR = dict(max_age_s=0.5, max_age_s_basis='test', stationary_window_s=1.0, stationary_window_s_basis='test',
              stationary_tolerance_m=0.0005, stationary_tolerance_m_basis='test', uncertainty_m=0.002,
              uncertainty_m_basis='test',
              calibration=dict(mode='TWO_POINT', basis='test two-point',
                               points=[dict(reading_m=0.100, overall_height_m=0.160),
                                       dict(reading_m=0.240, overall_height_m=0.300)]))


def samples(readings, start_s=0., step_s=0.1, integrity=CHECKED):
    return [LiftSample(float(r), int(round((start_s + i * step_s) * 1e9)), integrity, '00') for i, r in enumerate(readings)]


def test_every_threshold_and_basis_is_required():
    sensor_config(dict(sensor=SENSOR))
    for key in ('max_age_s', 'stationary_window_s', 'stationary_tolerance_m', 'uncertainty_m'):
        for missing in (key, key + '_basis'):
            bad = deepcopy(SENSOR)
            del bad[missing]
            with pytest.raises(ValueError, match='LIFT_SENSOR_(THRESHOLD_INVALID|BASIS_REQUIRED)'):
                sensor_config(dict(sensor=bad))
    with pytest.raises(ValueError, match='LIFT_SENSOR_CONFIG_REQUIRED'):
        sensor_config({})
    bad = deepcopy(SENSOR)
    bad['calibration']['points'][1]['reading_m'] = 0.100
    with pytest.raises(ValueError, match='LIFT_SENSOR_TWO_POINT_CALIBRATION_INVALID'):
        sensor_config(dict(sensor=bad))


def test_calibration_literal_values_and_no_extrapolation():
    cal = SENSOR['calibration']
    assert overall_height(cal, 0.200) == pytest.approx(0.260)
    assert overall_height(cal, 0.100) == pytest.approx(0.160)
    with pytest.raises(ValueError, match='LIFT_SENSOR_OUTSIDE_CALIBRATED_SPAN'):
        overall_height(cal, 0.250)
    # Cable shortening as the lift rises: one point plus a signed scale.
    one = dict(mode='ONE_POINT_SCALE', basis='x', scale=-1.0, scale_basis='x',
               points=[dict(reading_m=0.500, overall_height_m=0.300)])
    assert overall_height(one, 0.550) == pytest.approx(0.250)


def test_freeze_gives_h_and_conservative_interval():
    rows = samples([0.2000, 0.2002, 0.2001, 0.1999, 0.2000, 0.2003, 0.2001, 0.2000, 0.2002, 0.2001, 0.2001, 0.2001, 0.2002])
    state = freeze_lift(rows, SENSOR, Mechanism(), freeze_monotonic_ns=int(1.3e9))
    # Latest reading 0.2002 -> overall 0.2602 -> h = 0.2602 - 0.346 = -0.0858.
    assert state['h_m'] == pytest.approx(-0.0858, abs=1e-9)
    # Window (1.0 s back from 1.2 s, plus the sample at 0.2 s) spans 0.1999..0.2003, then +-2 mm.
    assert state['h_interval_m'] == pytest.approx([-0.0881, -0.0837], abs=1e-9)
    assert state['window_spread_m'] == pytest.approx(0.0004, abs=1e-12)
    assert state['latest_age_s'] == pytest.approx(0.1) and state['source'] == 'DRAW_WIRE_SENSOR'


@pytest.mark.parametrize('rows,freeze_s,code', [
    (samples([0.2] * 13), 1.8, 'LIFT_SENSOR_STALE'),
    (samples([0.2] * 13), 1.1, 'LIFT_SENSOR_SAMPLE_AFTER_FREEZE'),
    (samples([0.2] * 6), 0.6, 'LIFT_SENSOR_STATIONARY_WINDOW_NOT_COVERED'),
    (samples([0.2] * 12 + [0.201]), 1.3, 'LIFT_SENSOR_NOT_STATIONARY'),
    (samples([0.2] * 13, integrity='CRC_FAILED'), 1.3, 'LIFT_SENSOR_SAMPLE_UNCHECKED'),
    ([], 1.3, 'LIFT_SENSOR_NO_SAMPLES'),
])
def test_freeze_rejects_unusable_samples(rows, freeze_s, code):
    with pytest.raises(ValueError, match=code):
        freeze_lift(rows, SENSOR, Mechanism(), freeze_monotonic_ns=int(round(freeze_s * 1e9)))


def test_h_outside_a_tightened_field_range_is_rejected():
    rows = samples([0.239] * 13)  # overall 0.299 m -> h -0.047 m: inside the machine range
    assert freeze_lift(rows, SENSOR, Mechanism(), int(1.3e9))['h_m'] == pytest.approx(-0.047)
    with pytest.raises(ValueError, match='LIFT_SENSOR_H_OUTSIDE_RANGE'):
        freeze_lift(rows, SENSOR, Mechanism(), int(1.3e9), effective_range_m=[-0.15, -0.05])


@pytest.mark.skipif(not REAL.exists(), reason='private site layout manifest not converted here')
def test_loader_requires_the_frozen_sensor_state_and_sweeps_its_interval():
    from unloading_sim.eco65.site_mechanism import LiftMechanism, cad_robot_base, load_manifest, load_roles
    from unloading_sim.workbench.single_carton.site_layout import load_layout
    roles = load_roles(ROLES)
    mechanism = LiftMechanism(load_manifest(REAL), roles)
    lift = dict(source='DRAW_WIRE_SENSOR', sensor=SENSOR)
    c = dict(site_layout=dict(manifest=str(REAL.relative_to(ROOT)), roles=str(ROLES.relative_to(ROOT)),
                              T_base_layout=cad_robot_base(roles).tolist(), transform_basis='TEST', excluded_part_ids=[],
                              exclusion_basis='TEST', association_distance_m=.05, association_basis='TEST', lift=lift))
    with pytest.raises(ValueError, match='SITE_LIFT_FROZEN_SENSOR_STATE_REQUIRED'):
        load_layout(c, ROOT)
    frozen = freeze_lift(samples([0.2] * 13), sensor_config(lift), mechanism, freeze_monotonic_ns=int(1.3e9))
    loaded = load_layout(c, ROOT, frozen)
    top = next(o for o in loaded['world'] if o['name'] == 'YLYJ-JC00-07_顶板')
    # Top plate 10 mm thick (CAD Y), swept over the 4 mm interval: base-frame z size 14 mm.
    assert top['size'][1] == pytest.approx(0.014, abs=1e-6)
    assert loaded['identity']['mechanism']['lift']['h_interval_m'] == pytest.approx(frozen['h_interval_m'])
    changed = deepcopy(c)
    changed['site_layout']['lift']['sensor']['uncertainty_m'] = 0.003
    with pytest.raises(ValueError, match='SITE_LIFT_SENSOR_CONFIG_CHANGED'):
        load_layout(changed, ROOT, frozen)
