"""Directed regression for the observed top-middle native IK failure."""
from copy import deepcopy
import numpy as np
import pytest

import inspect_eco65_native_reach as reach


def fixture_snapshot():
    # A centred, planar 12-loop test geometry. The bound deliberately admits
    # the entire box face, so footprint dimensions do not tighten its result.
    contours = [dict(points_m=[[x, y, 0] for x, y in
                [(-.06, -.04), (.06, -.04), (.06, .04), (-.06, .04)]]) for _ in range(12)]
    tcp = np.eye(4); tcp[2, 3] = .14275043380451888
    return dict(fingerprint='TEST_ONLY',
        native_model=dict(init=dict(process_geometry=dict(contours=contours, flange_from_contact_frame=tcp.tolist()))),
        tool=dict(T_flange_tcp=tcp.tolist(), contact=dict(normal_tolerance_rad=.01)),
        scene=dict(base_position_m=[-.36, .1, .18], contact_tolerance_m=.0004,
            target_box_id='carton_r00_l03_c01', box=dict(pose=[.3334045931051881, 0, .53], size_m=[.25, .21, .1])))


def test_actual_joint_lengths_rule_out_even_unrestricted_top_face(monkeypatch):
    path = reach.ROOT/reach.load_robot_config()['urdf']
    if not path.is_file(): pytest.skip('Pinned official ECO65 URDF required')
    monkeypatch.setattr(reach, 'verify_snapshot', lambda s: s)
    snapshot = fixture_snapshot(); report = reach.inspect(snapshot)
    assert report['status'] == 'TOP_CONTACT_GEOMETRICALLY_UNREACHABLE'
    assert report['maximum_joint2_to_joint5_distance_m'] == pytest.approx(.617117086418564, abs=1e-12)
    assert report['necessary_joint2_to_joint5_distance_lower_bound_m'] == pytest.approx(.6645050318769182, abs=1e-12)
    # A closer test-only box must not be declared unreachable by this bound.
    closer = deepcopy(snapshot); closer['scene']['box']['pose'][0] = 0.
    assert reach.inspect(closer)['status'] == 'BOUND_INCONCLUSIVE'


def test_bound_rejects_tcp_outside_seal_hull(monkeypatch):
    monkeypatch.setattr(reach, 'verify_snapshot', lambda s: s)
    snapshot = fixture_snapshot()
    for row in snapshot['native_model']['init']['process_geometry']['contours']:
        for point in row['points_m']: point[0] += 1.
    with pytest.raises(ValueError, match='TCP_INSIDE_TWELVE_SEAL'):
        reach.inspect(snapshot)
