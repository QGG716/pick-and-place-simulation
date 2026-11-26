"""Under-the-upper footprint along the unobserved depth (user decision
2026-11-26 "按此规则来改"): with the front face alone the measured box is ~1 mm
deep; the lower's interface returns behind the front edge still count, up to
the upper's hidden-depth bound (field d5090aff). Fake data only."""
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.stack_support import infer_supports, under_upper_footprint, _measured_frame
from unloading_sim.workbench.single_carton.geometry import corners
from test_single_carton_stack_support import fixture

GENERAL = dict(mode='GENERAL_RESTING_CONTACT', basis='SYNTHETIC_GENERAL_RESTING_CONTACT')


def front_face_only(args, behind_m=None, front_x=.425, hidden_depth=.2):
    """The upper seen from +X only: measured box 1 mm deep at its front face,
    envelope reaching hidden_depth inward (-X). Lower returns moved to
    front_x-behind_m when given."""
    obs, result, boxes, rejected, site, geometry, pts, b = args
    upper = boxes[0]
    T = np.asarray(upper['pose'], float).copy(); T[0, 3] = front_x - .0005
    measured = dict(pose=T.tolist(), size_m=[.001, .1, .1])
    E = np.asarray(upper['pose'], float).copy(); E[0, 3] = front_x - hidden_depth / 2
    upper.update(measured_box=measured, measured_size_m=[.001, .1, .1], pose=E.tolist(), size_m=[hidden_depth, .1, .1],
                 corners_m=corners(E, [hidden_depth, .1, .1]).tolist(), faces=[dict(normal=[1., 0, 0])],
                 quality=dict(all_dimensions_source='RGBD_VISIBLE_FACE_BOUNDARIES_DEPTH_MAX_EXTENT'))
    if behind_m is not None:
        pts = pts.copy(); pts[..., 0] = front_x - behind_m
        face = rejected[0]['evidence']['observed_face']; face['center_m'][0] = front_x - behind_m
    return obs, result, boxes, rejected, site, geometry, pts, b


@pytest.mark.parametrize('rule', ['CONFIRMED', 'GENERAL'])
def test_front_face_only_upper_finds_lower_behind_its_front_edge(rule):
    args = list(front_face_only(fixture(), behind_m=.013))  # field: median 13-15 mm behind
    if rule == 'GENERAL': args[5] = dict(args[5], resting_contact_policy=GENERAL)
    bodies, records = infer_supports(*args)
    applied = [r for r in records if r.get('status') == 'APPLIED']
    assert applied and applied[0]['lower_index'] == 1, records
    if rule == 'GENERAL':
        assert applied[0]['footprint']['rule'] == 'LATERAL_MEASURED_DEPTH_FRONT_FACE_TO_ENVELOPE_HIDDEN_EXTENT'
        # measured from the measured box centre, 0.5 mm behind the front face
        assert abs(applied[0]['footprint']['inward_extent_m'] - (.2 - .0005)) < 1e-9


@pytest.mark.parametrize('rule', ['CONFIRMED', 'GENERAL'])
def test_returns_beyond_the_hidden_depth_bound_are_not_under_the_upper(rule):
    args = list(front_face_only(fixture(), behind_m=.25))   # 0.2 m bound + 3 mm pad < 0.25 m
    if rule == 'GENERAL': args[5] = dict(args[5], resting_contact_policy=GENERAL)
    bodies, records = infer_supports(*args)
    assert not bodies and not any(r.get('status') == 'APPLIED' for r in records)
    if rule == 'GENERAL':
        missed = [r for r in records if r['reason'] == 'INTERFACE_POINTS_BELOW_MINIMUM']
        assert missed and missed[0]['interface_points'] == 0 and missed[0]['required_points'] == 30


def test_footprint_keeps_lateral_bound_and_front_pad():
    _, _, boxes, _, _, _, _, _ = front_face_only(fixture())
    upper = boxes[0]; T, R, half, axis, _, _ = _measured_frame(upper); planar = [i for i in range(3) if i != axis]
    front = .425
    probe = np.array([[front - .1, 0, .3],                 # 10 cm behind the front face: under
                      [front + .002, 0, .3],               # inside the 3 mm front pad: under
                      [front + .01, 0, .3],                # 1 cm in front of the face: not under
                      [front - .1, .05 + .002, .3],        # lateral pad: under
                      [front - .1, .05 + .01, .3],         # 1 cm beyond the measured width: not under
                      [front - .21, 0, .3]])               # beyond 0.2 m + pad: not under
    under, record = under_upper_footprint(upper, (probe - T[:3, 3]) @ R, R, half, planar)
    assert under.tolist() == [True, True, False, True, False, False]
    assert record['depth_axis'] == 0


def test_upper_without_face_normal_keeps_the_measured_footprint():
    _, _, boxes, _, _, _, _, _ = fixture()
    upper = boxes[0]; T, R, half, axis, _, _ = _measured_frame(upper); planar = [i for i in range(3) if i != axis]
    under, record = under_upper_footprint(upper, (np.array([[.35 - .2, 0, .3]]) - T[:3, 3]) @ R, R, half, planar)
    assert record['rule'] == 'MEASURED_FOOTPRINT_NO_OBSERVED_FACE_NORMAL' and not under.any()
