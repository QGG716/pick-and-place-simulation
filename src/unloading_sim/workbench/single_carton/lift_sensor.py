"""Read-only draw-wire lift sensor: protocol-agnostic samples, calibration and freeze.

The protocol adapter (not implemented here: the sensor's communication manual
is still pending) must only ever read. It hands over ``LiftSample`` values:
cable extension in metres after the datasheet unit scaling, the raw bytes,
an integrity flag and the local monotonic receive time.

At the frozen RGB-D moment the lift must be stationary and the latest sample
fresh. The frozen state carries a conservative h interval: the window's
reading spread plus the configured sensor uncertainty. Nothing here defaults
a threshold; every value comes from the field configuration with a basis.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json

import numpy as np

from ...eco65.site_mechanism import RANGE_TOLERANCE_M

CHECKED = 'CHECKED'
SOURCE = 'DRAW_WIRE_SENSOR'
THRESHOLDS = ('max_age_s', 'stationary_window_s', 'stationary_tolerance_m', 'uncertainty_m')


@dataclass(frozen=True)
class LiftSample:
    reading_m: float
    received_monotonic_ns: int
    integrity: str
    raw_hex: str


def _number(value, positive=True):
    return type(value) in (int, float) and np.isfinite(value) and (value > 0 if positive else value >= 0)


def sensor_config(lift):
    """Validated ``site_layout.lift.sensor`` section; a missing value is an error."""
    c = lift.get('sensor') if isinstance(lift, dict) else None
    if not isinstance(c, dict):
        raise ValueError('LIFT_SENSOR_CONFIG_REQUIRED')
    for key in THRESHOLDS:
        if not _number(c.get(key), positive=key != 'uncertainty_m'):
            raise ValueError('LIFT_SENSOR_THRESHOLD_INVALID:' + key)
        if not isinstance(c.get(key + '_basis'), str) or not c[key + '_basis'].strip():
            raise ValueError('LIFT_SENSOR_BASIS_REQUIRED:' + key)
    cal = c.get('calibration')
    if not isinstance(cal, dict) or not isinstance(cal.get('basis'), str) or not cal['basis'].strip():
        raise ValueError('LIFT_SENSOR_CALIBRATION_REQUIRED')
    points = cal.get('points')
    if (not isinstance(points, list) or not points
            or any(not isinstance(p, dict) or not _number(p.get('reading_m'), False) or not _number(p.get('overall_height_m'))
                   for p in points)):
        raise ValueError('LIFT_SENSOR_CALIBRATION_POINTS_INVALID')
    if cal.get('mode') == 'TWO_POINT':
        if len(points) != 2 or points[0]['reading_m'] == points[1]['reading_m']:
            raise ValueError('LIFT_SENSOR_TWO_POINT_CALIBRATION_INVALID')
    elif cal.get('mode') == 'ONE_POINT_SCALE':
        scale = cal.get('scale')
        if len(points) != 1 or type(scale) not in (int, float) or not np.isfinite(scale) or scale == 0:
            raise ValueError('LIFT_SENSOR_ONE_POINT_CALIBRATION_INVALID')
        if not isinstance(cal.get('scale_basis'), str) or not cal['scale_basis'].strip():
            raise ValueError('LIFT_SENSOR_BASIS_REQUIRED:scale')
    else:
        raise ValueError('LIFT_SENSOR_CALIBRATION_MODE_INVALID')
    identity = hashlib.sha256(json.dumps(c, sort_keys=True, ensure_ascii=False).encode()).hexdigest()
    return dict(c, identity_sha256=identity)


def overall_height(calibration, reading_m):
    """Overall lift height (base plate bottom to top plate top) from a reading."""
    p = calibration['points']
    if calibration['mode'] == 'TWO_POINT':
        (r0, h0), (r1, h1) = ((q['reading_m'], q['overall_height_m']) for q in p)
        if not min(r0, r1) - RANGE_TOLERANCE_M <= reading_m <= max(r0, r1) + RANGE_TOLERANCE_M:
            raise ValueError('LIFT_SENSOR_OUTSIDE_CALIBRATED_SPAN')
        return h0 + (reading_m - r0) * (h1 - h0) / (r1 - r0)
    return p[0]['overall_height_m'] + calibration['scale'] * (reading_m - p[0]['reading_m'])


def freeze_lift(samples, config, mechanism, freeze_monotonic_ns, effective_range_m=None):
    """Lift state for one frozen observation, or a ValueError naming the reason."""
    c = config if 'identity_sha256' in config else sensor_config(dict(sensor=config))
    rows = sorted(samples, key=lambda s: s.received_monotonic_ns)
    if not rows:
        raise ValueError('LIFT_SENSOR_NO_SAMPLES')
    if any(s.integrity != CHECKED or not np.isfinite(s.reading_m) for s in rows):
        raise ValueError('LIFT_SENSOR_SAMPLE_UNCHECKED')
    latest = rows[-1]
    age_s = (freeze_monotonic_ns - latest.received_monotonic_ns) / 1e9
    if age_s < 0:
        raise ValueError('LIFT_SENSOR_SAMPLE_AFTER_FREEZE')
    if age_s > c['max_age_s']:
        raise ValueError('LIFT_SENSOR_STALE')
    start = latest.received_monotonic_ns - int(round(c['stationary_window_s'] * 1e9))
    before = [s for s in rows if s.received_monotonic_ns <= start]
    if not before:
        raise ValueError('LIFT_SENSOR_STATIONARY_WINDOW_NOT_COVERED')
    window = [before[-1]] + [s for s in rows if s.received_monotonic_ns > start]
    readings = np.array([s.reading_m for s in window])
    spread = float(readings.max() - readings.min())
    if spread > c['stationary_tolerance_m']:
        raise ValueError('LIFT_SENSOR_NOT_STATIONARY')
    overall = overall_height(c['calibration'], latest.reading_m)
    ends = sorted(overall_height(c['calibration'], r) for r in (readings.min(), readings.max()))
    h = overall - mechanism.overall_height_cad_m
    u = c['uncertainty_m']
    interval = [ends[0] - mechanism.overall_height_cad_m - u, ends[1] - mechanism.overall_height_cad_m + u]
    lo, hi = effective_range_m or mechanism.range_h_m
    if not lo - RANGE_TOLERANCE_M <= h <= hi + RANGE_TOLERANCE_M:
        raise ValueError('LIFT_SENSOR_H_OUTSIDE_RANGE')
    for value in interval:
        mechanism.state(value)
    return dict(source=SOURCE, h_m=float(h), h_interval_m=[float(v) for v in interval], overall_height_m=float(overall),
                reading_m=float(latest.reading_m), window_spread_m=spread, latest_age_s=float(age_s),
                window_samples=[dict(reading_m=s.reading_m, received_monotonic_ns=s.received_monotonic_ns,
                                     raw_hex=s.raw_hex) for s in window],
                freeze_monotonic_ns=int(freeze_monotonic_ns), config_sha256=c['identity_sha256'],
                h_basis='FROZEN_READ_ONLY_DRAW_WIRE_SAMPLE_WITH_STATIONARY_WINDOW',
                calibration_basis=c['calibration']['basis'])
