"""User decisions 2026-11-23 on the vision module (offline, synthetic data only).

1. single_face_depth_policy + max_carton_extent_m 0.20: a lone front face is
   bounded by the maximum extent instead of being rejected;
2. coplanar_continuation_policy: a face continuing past its mask is not
   rejected, those edges stay missing;
3. non_interpenetration_policy: other instances' surfaces cut the envelope on
   a single axis only;
4. resting_contact_policy: general resting contact replaces the confirmed
   two-carton stack; measured cartons under another one are demoted;
plus nested-duplicate removal and the own-mask claim policy validation.
"""
from copy import deepcopy
import numpy as np
import pytest
from test_single_carton_variable_geometry import measured_box
from test_single_carton_stack_support import fixture as stack_fixture
from unloading_sim.workbench.single_carton.geometry import recover, corners
from unloading_sim.workbench.single_carton.measured_envelope import single_face_policy, continuation_policy
from unloading_sim.workbench.single_carton import non_interpenetration as ni
from unloading_sim.workbench.single_carton.perception import drop_nested_duplicates
from unloading_sim.workbench.single_carton.stack_support import (resting_policy, demote_resting_lowers, infer_supports)
from unloading_sim.workbench.single_carton.site_scene import own_mask_claim_policy

SINGLE = dict(mode='BOUND_BY_MAX_CARTON_EXTENT', basis='SYNTHETIC')
CONTINUE = dict(mode='EXTEND_ENVELOPE_TO_NEXT_BOUNDARY', basis='SYNTHETIC')
NI = dict(mode='OTHER_INSTANCE_SURFACES_CUT_ENVELOPE', basis='SYNTHETIC')
RESTING = dict(mode='GENERAL_RESTING_CONTACT', basis='SYNTHETIC')


@pytest.mark.parametrize('check,key,mode', [
    (single_face_policy, 'single_face_depth_policy', 'BOUND_BY_MAX_CARTON_EXTENT'),
    (continuation_policy, 'coplanar_continuation_policy', 'EXTEND_ENVELOPE_TO_NEXT_BOUNDARY'),
    (ni.policy, 'non_interpenetration_policy', 'OTHER_INSTANCE_SURFACES_CUT_ENVELOPE'),
    (resting_policy, 'resting_contact_policy', 'GENERAL_RESTING_CONTACT'),
    (own_mask_claim_policy, 'own_mask_claim_policy', 'OWN_MASK_FIRST_BRIDGE_NO_RETURN')])
def test_every_new_switch_is_off_by_default_and_needs_mode_and_basis(check, key, mode):
    assert check(None) is None and check({}) is None and check({key: dict(mode='DISABLED')}) is None
    for bad in (dict(mode='GUESS', basis='X'), dict(mode=mode), dict(mode=mode, basis=' '), 'ON'):
        with pytest.raises(ValueError):
            check({key: bad})
    assert check({key: dict(mode=mode, basis='X')})['mode'] == mode


def contains(box, true_pose, true_size, tol=1e-9):
    T = np.asarray(box['pose'], float)
    local = (corners(true_pose, true_size)-T[:3, 3])@T[:3, :3]
    return bool(np.all(abs(local) <= np.asarray(box['size_m'])/2+tol))


def test_single_face_is_rejected_by_default_and_bounded_by_the_maximum_extent():
    size = (.18, .12, .10)
    p, m, c, R = measured_box(size, angles=(0, 0, 0))
    with pytest.raises(ValueError, match='BOX_THICKNESS_UNOBSERVED_SHOW_SIDE'):
        recover(p, m, np.eye(4), conservative_measured=True, max_carton_extent_m=.2)
    box = recover(p, m, np.eye(4), conservative_measured=True, max_carton_extent_m=.2, single_face=SINGLE)
    q = box['quality']
    assert q['depth_measured'] is False and q['measured_thickness_m'] is None and q['hidden_extent_measured'] is False
    assert q['all_dimensions_source'] == 'RGBD_VISIBLE_FACE_BOUNDARIES_DEPTH_MAX_EXTENT'
    back = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (2, -1))
    assert back['method'] != 'DIRECT_DEPTH_BOUNDARY'       # the lone face never certifies a back
    pose = np.eye(4); pose[:3, :3] = R; pose[:3, 3] = c
    assert contains(box, pose, size)                         # true 0.10 m depth inside the 0.20 m bound
    depth = box['size_m'][2]
    assert .2 <= depth <= .2+.02


def test_coplanar_continuation_extends_instead_of_rejecting():
    p, m, c, R = measured_box((.24, .18, .12))
    xs = np.nonzero(m)[1]; cut = m.copy(); cut[:, :int(xs.mean())] = False
    flat = np.full((*m.shape, 3), 110, np.uint8)
    with pytest.raises(ValueError, match='COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED'):
        recover(p, cut, np.eye(4), rgb=flat, conservative_measured=True)
    box = recover(p, cut, np.eye(4), rgb=flat, conservative_measured=True, continuation_policy=CONTINUE)
    record = box['quality']['coplanar_continuation']
    assert record['continuation_pixels'] > 0 and record['rgb_edge']['fraction'] < .7
    cut_side = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert cut_side['method'] != 'DIRECT_DEPTH_BOUNDARY'     # extended to the next real boundary (free rays here)
    pose = np.eye(4); pose[:3, :3] = R; pose[:3, 3] = c
    assert contains(box, pose, (.24, .18, .12), tol=.004)   # the whole carton, not only the cut half


def synthetic_box(lo, hi):
    T = np.eye(4); T[:3, 3] = [0., 0., 1.]
    intervals = []
    for a in range(3):
        for s, outer in ((-1, lo[a]), (1, hi[a])):
            intervals.append(dict(axis=a, side=s, measured_m=s*.05, outer_m=outer, method='MISSING_BOUNDARY_MAXIMUM_EXTENT',
                                  pixel_footprint_m=.001))
    lo = np.asarray(lo, float); hi = np.asarray(hi, float)
    pose = T.copy(); pose[:3, 3] += (lo+hi)/2
    return dict(id='target', detection_index=0, measured_box=dict(pose=T.tolist(), size_m=[.1, .1, .1]),
                pose=pose.tolist(), size_m=(hi-lo).tolist(), corners_m=corners(pose, hi-lo).tolist(),
                uncertainty=dict(kind='MEASURED_BOUNDARY_INTERVALS', lower_m=lo.tolist(), upper_m=hi.tolist(),
                                 intervals=intervals))


def patch_image(patches):
    """Camera = world; each patch fills a 12x12 pixel block with one 3-D point."""
    points = np.zeros((120, 160, 3)); masks = [np.zeros((120, 160), bool), np.zeros((120, 160), bool)]
    for k, (row, col, xyz, owner) in enumerate(patches):
        points[row:row+12, col:col+12] = xyz; masks[owner][row:row+12, col:col+12] = True
    return points, masks


def test_other_instance_surface_cuts_only_a_single_axis_side_and_never_the_measured_box():
    box = synthetic_box([-.05, -.05, -.25], [.25, .15, .05])
    points, masks = patch_image([
        (2, 2, [0., 0., 1.], 0),            # own face sample
        (40, 40, [.12, 0., 1.], 1),         # beyond +x only -> cuts +x
        (40, 100, [.2, .1, 1.], 1),         # beyond +x and +y -> cuts nothing
    ])
    records = ni.cut_by_other_instances([box], points, masks, np.eye(4), NI)
    cuts = {(c['axis'], c['side']): c['outer_m'] for c in records[0]['cuts']}
    pad = .001+.003
    assert set(cuts) == {(0, 1)} and cuts[(0, 1)] == pytest.approx(.12-pad)
    assert box['uncertainty']['upper_m'][1] == pytest.approx(.15)          # two-axis return: no cut
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, 1))
    assert interval['method'] == 'NON_INTERPENETRATION_OTHER_INSTANCE_SURFACE'
    # A return inside the measured box (+pad) never cuts into it.
    box = synthetic_box([-.05, -.05, -.25], [.25, .15, .05])
    points, masks = patch_image([(2, 2, [0., 0., 1.], 0), (40, 40, [.052, 0., 1.], 1)])
    ni.cut_by_other_instances([box], points, masks, np.eye(4), NI)
    assert box['uncertainty']['upper_m'][0] >= .05+pad-1e-12


def test_fewer_than_the_robust_rank_returns_do_not_cut():
    box = synthetic_box([-.05, -.05, -.25], [.25, .15, .05])
    points, masks = patch_image([(2, 2, [0., 0., 1.], 0)])
    points[60:63, 60:63] = [.12, 0., 1.]; masks[1][60:63, 60:63] = True      # 9 pixels, interior only 1
    ni.cut_by_other_instances([box], points, masks, np.eye(4), NI)
    assert box['uncertainty']['upper_m'][0] == pytest.approx(.25)


def measured(name, index, center, size, envelope=None, faces=None):
    T = np.eye(4); T[:3, 3] = center
    e = envelope or (center, size)
    E = np.eye(4); E[:3, 3] = e[0]
    return dict(id=name, detection_index=index, measured_box=dict(pose=T.tolist(), size_m=list(size)),
                pose=E.tolist(), size_m=list(e[1]), corners_m=corners(E, e[1]).tolist(),
                faces=faces or [dict(center_m=list(center), normal=[1., 0., 0.], u=[0., 1., 0.], v=[0., 0., 1.],
                                     size_m=[size[1], size[2]], support_points_uv_m=[[0., 0.]])],
                quality=dict(measured_face_size_m=[size[1], size[2]]))


def test_nested_duplicate_parent_is_dropped_only_when_both_measured_boxes_coincide():
    child = measured('child', 1, [0., 0., .3], [.1, .17, .11])
    parent = measured('parent', 0, [.002, 0., .301], [.1, .168, .112])
    parent['instance_fit'] = dict(used='ORIGINAL_MASK_AFTER_DIFFERENCE_FAILED', excluded=[dict(detection_index=1)])
    boxes, rejected = drop_nested_duplicates([parent, child], [])
    assert [b['id'] for b in boxes] == ['child']
    assert rejected[0]['reason'] == 'DUPLICATE_OF_NESTED_INSTANCE_SAME_MEASURED_BOX'
    assert rejected[0]['evidence']['duplicate_of_detection_index'] == 1
    far = measured('parent', 0, [0., 0., .25], [.1, .17, .2])
    far['instance_fit'] = parent['instance_fit']
    assert len(drop_nested_duplicates([far, deepcopy(child)], [])[0]) == 2


def test_measured_carton_under_another_is_demoted_only_with_resting_contact():
    upper = measured('upper', 0, [-.6, 0., .37], [.1, .17, .11])
    lower = measured('lower', 1, [-.6, 0., .27], [.1, .15, .08], envelope=([-.6, 0., .3], [.12, .2, .2]))
    geometry = dict(resting_contact_policy=RESTING)
    assert demote_resting_lowers([upper, lower], [], {}) == ([upper, lower], [])
    boxes, rejected = demote_resting_lowers([upper, lower], [], geometry)
    assert [b['id'] for b in boxes] == ['upper']
    assert rejected[0]['reason'] == 'RESTING_UNDER_RESOLVED_UPPER'
    assert rejected[0]['evidence']['observed_face'] == lower['faces'][0]
    beside = measured('beside', 1, [-.6, .4, .27], [.1, .15, .08])
    assert len(demote_resting_lowers([upper, beside], [], geometry)[0]) == 2


def test_general_resting_contact_needs_no_two_carton_confirmation_or_spread_limit():
    args = list(stack_fixture())
    args[5] = dict(size_policy='MEASURED_PER_INSTANCE', size_policy_basis='SYNTHETIC', resting_contact_policy=RESTING)
    bodies, records = infer_supports(*args)
    record = next(r for r in records if r['status'] == 'APPLIED')
    assert record['rule'] == 'GENERAL_RESTING_CONTACT' and record['spread_limit_m'] is None
    body, _ = bodies[record['lower_index']]
    assert body['category'] == 'confirmed_stack_support' and body['supports_target_id'] == 'upper'
    assert np.max(corners(body['pose'], body['size'])@np.asarray(body['support_plane']['normal_world'])) == \
        pytest.approx(body['support_plane']['offset_m'], abs=1e-9)
    # Lower top occluded 21 mm under the upper (overhang): the legacy rule
    # rejects (15 mm spread), the general rule applies.
    def overhang(geometry):
        a = list(stack_fixture()); pts = a[6].copy(); pts[30:35, :, 0] = .39; a[6] = pts; a[5] = geometry; return a
    legacy = overhang(dict(size_policy='MEASURED_PER_INSTANCE', size_policy_basis='SYNTHETIC',
                           confirmed_stack_support=dict(mode='USER_CONFIRMED_TWO_CARTON_STACK', basis='SYNTHETIC')))
    assert infer_supports(*legacy)[1][0]['reason'] == 'STACK_SUPPORT_HEIGHT_ESTIMATES_DISAGREE'
    general = overhang(dict(size_policy='MEASURED_PER_INSTANCE', size_policy_basis='SYNTHETIC', resting_contact_policy=RESTING))
    assert any(r['status'] == 'APPLIED' for r in infer_supports(*general)[1])


def test_lower_seen_from_above_gets_a_body_bounded_by_its_frontmost_return():
    """A lower whose observed face is its top: the body is closed toward the
    camera by the frontmost lower return along the upper face direction, so it
    never reaches the robot side (replay 8af1e4ef: an unbounded body hit base_link)."""
    from types import SimpleNamespace
    from unloading_sim.workbench.single_carton.geometry import corners as box_corners
    obs, result, boxes, rejected, site, _, pts, lower_mask = stack_fixture()
    upper = boxes[0]
    # Measured frame convention of the field data: local +Z = front-face normal (+x here).
    R = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]); T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [.35, 0, .35]
    upper.update(pose=T.tolist(), size_m=[.1, .1, .15], corners_m=box_corners(T, [.1, .1, .15]).tolist())
    face = rejected[0]['evidence']['observed_face']
    face.update(normal=[0., 0., 1.], u=[1., 0., 0.], v=[0., 1., 0.])        # the lower's top face
    geometry = dict(size_policy='MEASURED_PER_INSTANCE', size_policy_basis='SYNTHETIC', resting_contact_policy=RESTING)
    bodies, records = infer_supports(obs, result, boxes, rejected, site, geometry, pts, lower_mask | True)
    applied = [r for r in records if r['status'] == 'APPLIED']
    if not applied:
        pytest.skip('synthetic top-face lower has no interface in this fixture: ' + str([r['reason'] for r in records]))
    body, record = bodies[applied[0]['lower_index']]
    assert record['front_bound']['kind'] == 'FRONTMOST_OBSERVED_LOWER_RETURN_ALONG_UPPER_FACE_DIRECTION'
    toward = np.asarray(record['front_bound']['normal_world'])
    assert np.max(box_corners(body['pose'], body['size'])@toward) <= record['front_bound']['offset_m']+1e-9


def test_lower_top_beside_the_upper_becomes_support_at_the_plane_only_behind_the_front_face():
    """User decision 2026-11-24: the supporting lower's top surface continuing
    beside the upper (behind its front face, inside the interface band) is the
    support surface, represented with its top exactly on the support plane."""
    from unloading_sim.workbench.single_carton.site_scene import _beside_support_pieces
    from unloading_sim.workbench.single_carton.stack_support import resting_policy as rp
    with pytest.raises(ValueError):
        rp(dict(resting_contact_policy=dict(RESTING, lower_top_beside='SUPPORT_SURFACE')))   # basis required
    assert rp(dict(resting_contact_policy=dict(RESTING, lower_top_beside='SUPPORT_SURFACE',
                                               lower_top_beside_basis='X')))['lower_top_beside'] == 'SUPPORT_SURFACE'
    # Upper measured box: +Z = front face (+x world), vertical = world z, bottom at z = 0.30.
    R = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]); T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [-.6, 0., .355]
    upper = dict(id='upper', measured_box=dict(pose=T.tolist(), size_m=[.17, .11, .13]), pose=T.tolist(), size_m=[.18, .12, .14])
    frame = np.eye(4); frame[:3, 3] = [-.7, 0., .2]
    body = dict(id='support', pose=frame.tolist(), supports_target_id='upper', detection_index=1,
                support_plane=dict(normal_world=[0., 0., 1.], offset_m=.296), support_basis='SYNTHETIC', observation_id='o')
    record = dict(upper_id='upper', seam_band_ownership=dict(normal_world=[0., 0., 1.], plane_m=.296, band_top_m=.304))
    observed = np.array([
        [-.62, .11, .300],    # beside (y beyond 0.085), behind the face, in band -> support
        [-.62, .12, .302],    # beside, in band -> support
        [-.50, .11, .300],    # in FRONT of the front face (x > -0.535) -> stays an obstacle
        [-.62, .11, .330],    # above the band -> stays an obstacle
        [-.62, .30, .300],    # farther than 60 mm laterally -> stays an obstacle
    ])
    lower = np.array([True, True, True, True, True])
    pieces, info = _beside_support_pieces(body, record, [upper], observed, np.ones(5, bool), lower)
    members = np.any([m for _, m in pieces], axis=0)
    assert members.tolist() == [True, True, False, False, False]
    for piece, _ in pieces:
        top = np.max(corners(piece['pose'], piece['size'])[:, 2])
        assert top == pytest.approx(.296, abs=1e-9) and piece['category'] == 'confirmed_stack_support'
    assert info['points'] == 2 and info['max_represented_lower_by_m'] == pytest.approx(.006, abs=1e-9)
    # Points labelled by another instance are never taken.
    pieces, info = _beside_support_pieces(body, record, [upper], observed, np.ones(5, bool), np.zeros(5, bool))
    assert not pieces


ADJ = dict(mode='LATERAL_CONTACT_PLANES', max_gap_m=.01, basis='SYNTHETIC')


def adjacent_box():
    """Measured frame as in the field: +Z = front face (world +x), axis 1 = world z (vertical), axis 0 = world y."""
    from unloading_sim.workbench.single_carton.geometry import corners as box_corners
    R = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]); T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [-.6, 0., .35]
    lo = np.array([-.095, -.06, -.2]); hi = np.array([.095, .06, .055])
    intervals = [dict(axis=a, side=s, measured_m=s*h, outer_m=(lo if s < 0 else hi)[a], method='SENSOR_FREE_RECTANGLE_SPAN_BOUND',
                      pixel_footprint_m=.0006) for a, h in enumerate([.085, .055, .05]) for s in (-1, 1)]
    E = T.copy(); E[:3, 3] += R@((lo+hi)/2)
    return dict(id='t', detection_index=0, measured_box=dict(pose=T.tolist(), size_m=[.17, .11, .1]),
                pose=E.tolist(), size_m=(hi-lo).tolist(), corners_m=box_corners(E, hi-lo).tolist(),
                uncertainty=dict(kind='MEASURED_BOUNDARY_INTERVALS', lower_m=lo.tolist(), upper_m=hi.tolist(), intervals=intervals))


def test_adjacent_contact_plane_trims_only_lateral_sides_from_instance_returns():
    from unloading_sim.workbench.single_carton import adjacent_contact as ac
    assert ac.policy(None) is None
    for bad in (dict(ADJ, mode='X'), dict(ADJ, basis=''), dict(ADJ, max_gap_m=.05), dict(ADJ, max_gap_m=.001)):
        with pytest.raises(ValueError):
            ac.policy(dict(adjacent_contact_policy=bad))
    box = adjacent_box()
    # Neighbour returns 4 mm beyond the +y measured side (world y = 0.089), at the front face, inside the vertical range.
    pts = np.zeros((20, 20, 3)); pts[:, :] = [-.55, .089, .35]
    pts[:10, :, 0] = -.70                      # behind the measured depth: never bounds the side
    cand = np.zeros((20, 20), bool); cand[:, :] = True
    planes, record = ac.contact_planes(box, pts, cand, ac.policy(dict(adjacent_contact_policy=ADJ)))
    assert len(planes) == 1 and planes[0]['axis'] == 0 and planes[0]['side'] == 1
    assert planes[0]['gap_to_measured_m'] == pytest.approx(.004, abs=1e-9)
    assert box['uncertainty']['upper_m'][0] == pytest.approx(.089, abs=1e-9)
    assert box['uncertainty']['lower_m'][0] == pytest.approx(-.095)           # other side untouched
    assert box['uncertainty']['lower_m'][2] == pytest.approx(-.2)             # no back plane
    n = np.asarray(planes[0]['normal_world'])
    assert float(np.min(np.asarray(box['corners_m'])@n)) == pytest.approx(planes[0]['offset_m'], abs=1e-9)
    # Farther than max_gap: no plane.
    box = adjacent_box(); far = pts.copy(); far[..., 1] = .085+.012
    assert ac.contact_planes(box, far, cand, ac.policy(dict(adjacent_contact_policy=ADJ)))[0] == []


def test_neighbour_returns_never_include_the_target_or_unlabelled_pixels():
    from unloading_sim.workbench.single_carton.adjacent_contact import neighbour_returns
    box = adjacent_box(); own = np.zeros((40, 40), bool); own[5:20, 5:20] = True
    other = np.zeros((40, 40), bool); other[5:20, 22:38] = True
    pts = np.ones((40, 40, 3)); valid = np.ones((40, 40), bool)
    mask = neighbour_returns(box, pts, valid, [own, other], [box])
    assert not (mask & own).any() and not mask[30:, :].any()                  # own and unlabelled excluded
    assert mask[10, 30] and not mask[10, 22]                                    # within 6 px of the target excluded


def test_neighbour_obstacles_are_clipped_or_extended_to_the_plane_and_bound_to_the_target():
    from unloading_sim.workbench.single_carton import adjacent_contact as ac
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    from types import SimpleNamespace
    setting = ac.policy(dict(adjacent_contact_policy=ADJ))
    box = adjacent_box(); pts = np.zeros((20, 20, 3)); pts[:, :] = [-.55, .089, .35]
    planes, _ = ac.contact_planes(box, pts, np.ones((20, 20), bool), setting)
    def cube(c, size=.006):
        T = np.eye(4); T[:3, 3] = c; return dict(id=f'v{c[1]:.4f}', pose=T.tolist(), size=[size]*3, category='observed')
    crossing = cube([-.56, .088, .35])        # crosses the plane (y = 0.089) into the target side
    close = cube([-.56, .0935, .35])          # 1.5 mm short of the plane: extended to it
    far = cube([-.56, .12, .35])              # beyond the band: untouched
    world = [crossing, close, far]
    records = ac.tag_neighbours(world, [box], planes, setting)
    tagged = [o for o in world if o.get('category') == 'adjacent_contact']
    assert {o['original_id'] for o in tagged} == {crossing['id'], close['id']} and far in world
    n = np.asarray(planes[0]['normal_world']); d = planes[0]['offset_m']
    for o in tagged:
        v = np.asarray(o['convex_mesh']['vertices'])
        assert float((v@n).max()) == pytest.approx(d, abs=1e-9) and o['supports_target_id'] == 't'
    assert len(records) == 2
    supports = SinglePlanner.detach_supports(SimpleNamespace(object_top=SinglePlanner.object_top), world, box)
    assert {s['kind'] for s in supports} == {'ADJACENT_CONTACT'} and len(supports) == 2
    bad = dict(tagged[0], support_plane=dict(planes[0], offset_m=d+.002))
    with pytest.raises(ValueError, match='ADJACENT_CONTACT_NOT_BOUND'):
        SinglePlanner.detach_supports(SimpleNamespace(object_top=SinglePlanner.object_top), [bad], box)
