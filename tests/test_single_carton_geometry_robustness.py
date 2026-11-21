"""2026-11-19 geometry robustness: field-like scenes, truth containment and the
user-selected coplanar seam policy. Synthetic RGB-D only; no device IO."""
from copy import deepcopy
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
import field_like_scene as fl
from test_single_carton_variable_geometry import measured_box
from test_single_carton_measured_envelope import stacked_fixture
from unloading_sim.workbench.single_carton import coplanar_seam
from unloading_sim.workbench.single_carton.geometry import recover, corners, _visible_planes
from unloading_sim.workbench.single_carton.instance_fit import measured_instance
from unloading_sim.workbench.single_carton.measured_envelope import (conservative_envelope, support_clip,
    edge_summary, SUPPORT_TILT_LIMIT_RAD)
from unloading_sim.workbench.single_carton.hf_detector_core import carton_nms
from unloading_sim.workbench.single_carton.planner import seam_band_contacts
from unloading_sim.workbench.single_carton.stack_support import infer_supports, effective_instances

SEAM = dict(mode='MASK_RGB_EDGE_COINCIDENT', uncertainty_m=.005,
            basis='USER_DECISION_2026_11_19_COPLANAR_SEAM_TRUST_MASK_BOUNDARY_COINCIDENT_WITH_STRONG_RGB_EDGE_PLUS_MINUS_5_MM')
NAMES = ['ideal_stack', 'noisy_visible_support', 'flush_stack', 'flush_stack_mask_+1px', 'stack_mask_+1px',
         'stack_mask_+3px', 'stack_mask_-2px', 'stack_samlike_0.35', 'frontal_yaw_0', 'yaw_-12', 'yaw_+20',
         'coplanar_touching_left', 'left_gap_30mm', 'parent_mask_with_child', 'packed_wall']
CACHE = {}


def outside_mm(box, pose, size):
    T = np.asarray(box['pose'])
    return 1000*max(0., float(np.max(abs((fl.truth_corners(pose, size)-T[:3, 3])@T[:3, :3])-np.asarray(box['size_m'])/2)))


def methods(box):
    return {'wht'[i['axis']]+('-' if i['side'] < 0 else '+'): i['method'] for i in box['uncertainty']['intervals']}


def run(name, seam=None, bbox_slack=6, rgb=None, only_target_mask=False, shadow=None):
    key = (name, None if seam is None else seam['uncertainty_m'], bbox_slack, rgb is None, only_target_mask, shadow is None)
    if key not in CACHE:
        p, masks, image, cam, (pose, size) = fl.frame(name)
        ys, xs = np.nonzero(masks[0])
        bbox = [xs.min()-bbox_slack, ys.min()-bbox_slack, xs.max()+bbox_slack, ys.max()+bbox_slack]
        box = measured_instance(p, masks[:1] if only_target_mask else masks, 0, cam, rgb=image if rgb is None else rgb,
                                maximum=.4, seam_policy=seam, bbox=bbox, shadow_policy=shadow)
        CACHE[key] = (box, pose, size)
    return CACHE[key]


# --- field-like scenes (all 15): recovered and truth contained ---------------

@pytest.mark.parametrize('name', NAMES)
def test_field_like_scene_recovers_and_contains_truth(name):
    box, pose, size = run(name)
    assert outside_mm(box, pose, size) == 0.


@pytest.mark.parametrize('name', ['noisy_visible_support', 'flush_stack'])
def test_noisy_support_plane_is_applied_not_rejected(name):
    box, _, _ = run(name)
    assert 'MEASURED_SUPPORT_PLANE' in methods(box).values()


@pytest.mark.parametrize('name', ['frontal_yaw_0', 'yaw_-12', 'yaw_+20', 'stack_mask_+1px', 'stack_mask_+3px'])
def test_thickness_from_second_plane_within_3mm(name):
    box, _, size = run(name)
    assert abs(box['measured_size_m'][2]-size[2]) <= .003


@pytest.mark.parametrize('name', ['stack_mask_+1px', 'flush_stack_mask_+1px'])
def test_thickness_back_edge_bounded_by_free_rays(name):
    box, _, size = run(name)
    assert methods(box)['t-'] != 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    assert box['size_m'][2] <= size[2]+.025


def test_parent_mask_with_child_uses_difference_first():
    box, pose, size = run('parent_mask_with_child')
    assert box['instance_fit']['used'] == 'DIFFERENCE_MASK'
    assert outside_mm(box, pose, size) == 0.


# --- P0-A tilted support -----------------------------------------------------

@pytest.mark.parametrize('noise_mm', [0., .1, .5, 1.])
def test_noisy_stack_fixture_recovers_with_support(noise_mm):
    p, m, W, height = stacked_fixture()
    rng = np.random.default_rng(6518)
    p = p+p/p[..., 2, None]*rng.normal(0, noise_mm/1000, m.shape)[..., None]
    b = recover(p, m, W, conservative_measured=True)
    plane = b['quality']['observed_support_plane']
    assert plane['tilt_rad'] <= SUPPORT_TILT_LIMIT_RAD
    # highest point of the new bottom is at the support top, lowest within the window
    bottom = np.asarray(b['corners_m'])[:, 2]
    assert bottom.min() >= plane['top_world_m']-plane['support_noise_window_m']-1e-9


def test_support_clip_keeps_everything_above_the_plane():
    tilt = np.deg2rad(.5); T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.array([tilt, 0, 0]))[0]
    lo, hi = np.array([-.08, -.05, -.4]), np.array([.08, .05, .05])
    clip = support_clip(T, lo, hi, dict(top_world_m=-.03, noise_m=.0005))
    assert clip['applied'] and clip['axis'] == 2 and clip['side'] == -1
    lo2 = lo.copy(); lo2[2] = clip['cut_m']
    box = np.array([[x, y, lo2[2]] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])])@T[:3, :3].T
    # highest bottom point one support-height sigma below the support top
    assert abs(box[:, 2].max()-(-.03-.0005)) < 1e-12
    assert box[:, 2].min() >= -.03-clip['support_noise_window_m']-1e-12
    steep = np.eye(4); steep[:3, :3] = cv2.Rodrigues(np.array([np.deg2rad(6), 0, 0]))[0]
    far = support_clip(steep, lo, hi, dict(top_world_m=-.03, noise_m=.0005))
    assert not far['applied'] and far['reason'] == 'MEASURED_SUPPORT_TILT_EXCEEDS_5_DEG'
    wide = support_clip(T, np.array([-.05, -.4, -.4]), np.array([.05, .4, .05]), dict(top_world_m=-.03, noise_m=.0005))
    assert not wide['applied'] and wide['reason'] == 'MEASURED_SUPPORT_TILT_DROP_EXCEEDS_SUPPORT_NOISE_WINDOW'


# --- P0-B corner pixels never certify an edge ---------------------------------

def test_mask_missing_one_side_is_never_certified_by_corner_pixels():
    p, m, c, R = measured_box((.24, .18, .12)); xs = np.nonzero(m)[1]
    cut = m.copy(); cut[:, :xs.min()+20] = False
    b = recover(p, cut, np.eye(4), conservative_measured=True)
    truth = np.eye(4); truth[:3, :3] = R; truth[:3, 3] = c
    T = np.asarray(b['pose'])
    assert np.max(abs((corners(truth, (.24, .18, .12))-T[:3, 3])@T[:3, :3])-np.asarray(b['size_m'])/2) <= 1e-7
    for i in b['uncertainty']['intervals']:
        if i['method'] == 'DIRECT_DEPTH_BOUNDARY' and not (i['axis'] == 2 and i['side'] == 1):
            assert i['direct_coverage']['accepted']


@pytest.mark.parametrize('defect', ['left_0', 'left_10', 'left_40', 'left_60', 'top_30', 'right_20', 'bottom_60',
                                    'sam_0.35', 'sam_0.5', 'sam_0.65', 'shift_0_2', 'shift_0_-2', 'shift_2_0', 'shift_-2_0'])
@pytest.mark.parametrize('noise_mm', [0., 1.])
def test_truth_contained_under_mask_defects(defect, noise_mm):
    size = (.24, .18, .12); p, m, c, R = measured_box(size)
    rng = np.random.default_rng(6518)
    p = p+p/p[..., 2, None]*rng.uniform(-noise_mm/1000, noise_mm/1000, m.shape)[..., None]
    ys, xs = np.nonzero(m); kind, *value = defect.split('_')
    if kind in ('left', 'right', 'top', 'bottom'):
        n = int(value[0]); m = m.copy()
        if kind == 'left': m[:, :xs.min()+n] = False
        if kind == 'right': m[:, xs.max()+1-n:] = False
        if kind == 'top': m[:ys.min()+n] = False
        if kind == 'bottom': m[ys.max()+1-n:] = False
    elif kind == 'sam':m = fl.samlike(m, float(value[0]))
    else:m = np.roll(m, (int(value[0]), int(value[1])), axis=(0, 1))
    try:b = recover(p, m, np.eye(4), conservative_measured=True)
    except ValueError:return   # a rejection is safe
    truth = np.eye(4); truth[:3, :3] = R; truth[:3, 3] = c; T = np.asarray(b['pose'])
    assert np.max(abs((corners(truth, size)-T[:3, 3])@T[:3, :3])-np.asarray(b['size_m'])/2) <= 1e-7


# --- P1-C a non-adjacent plane is never a side --------------------------------

def test_transverse_plane_not_touching_the_face_is_not_accepted_as_side():
    """A perpendicular plane elsewhere in the mask (lower top, wall) has no
    supported common edge with the front face and must never be the side."""
    size = (.24, .18, .12); p, m, c, R = measured_box(size)
    ring = cv2.dilate(m.astype('uint8'), np.ones((61, 61), 'uint8')).astype(bool) & ~cv2.dilate(
        m.astype('uint8'), np.ones((31, 31), 'uint8')).astype(bool)
    rays = p/p[..., 2, None]
    normal = R[:, 0]; point = c+normal*(size[0]/2+.12)          # 12 cm beyond the box, parallel to its x side
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = (point@normal)/(rays@normal)
    use = ring & (depth > .1) & (depth < 1.4)
    q = p.copy(); q[use] = rays[use]*depth[use, None]
    diagnostics = []
    _, planes = _visible_planes(q, m | use, 6518, 120, .002, diagnostics)
    # The far parallel plane is rejected; any accepted side with this normal
    # is the carton's own side (centre within the carton), never the far one.
    assert any(not d['accepted'] and str(d['reason']).startswith('NOT_ADJACENT_FACE') for d in diagnostics
               if abs(np.asarray(d['normal_camera'])@normal) > .99)
    for pl in planes[1:]:
        if abs(pl['normal']@normal) > .99:assert abs((pl['center']-c)@normal) <= size[0]/2+.005


# --- P1-E no witness from occluded or missing depth ---------------------------

def test_no_free_witness_without_farther_valid_depth():
    p, masks, image, cam, _ = fl.frame('flush_stack')        # exact mask: no background inside it
    p = p.copy(); p[~masks[0]] = np.nan
    b = measured_instance(p, masks, 0, cam, rgb=image, maximum=.4)
    by = {(i['axis'], i['side']): i for i in b['uncertainty']['intervals']}
    # Only the carton's own visible side can witness (w+/h+ here); the other
    # edges have no farther valid return and stay missing.
    for key in ((0, -1), (1, -1), (2, -1)):
        assert by[key]['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    for i in by.values():
        if i['method'] == 'SENSOR_FREE_RECTANGLE_SPAN_BOUND':
            x, y = i['free_space_witness']['pixel_xy']
            assert np.isfinite(p[y, x]).all() and masks[0][y, x]


# --- P1-F coplanar cut without an RGB edge is rejected -------------------------

def test_mask_cut_through_the_same_plane_without_rgb_edge_is_rejected():
    p, m, _, _ = measured_box((.24, .18, .12)); xs = np.nonzero(m)[1]
    cut = m.copy(); cut[:, :int(xs.mean())] = False
    flat = np.full((*m.shape, 3), 110, np.uint8)
    with pytest.raises(ValueError, match='COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED'):
        recover(p, cut, np.eye(4), rgb=flat, conservative_measured=True)


def test_touching_neighbour_without_rgb_edge_never_gets_a_seam():
    p, masks, image, cam, _ = fl.frame('coplanar_touching_left')
    flat = np.full_like(image, 110); ys, xs = np.nonzero(masks[0])
    try:box = measured_instance(p, masks, 0, cam, rgb=flat, maximum=.4, seam_policy=SEAM,
                                bbox=[xs.min()-6, ys.min()-6, xs.max()+6, ys.max()+6])
    except ValueError as e:
        assert 'COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED' in str(e); return
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT' and interval['missing_boundary_class'] == 'NO_RGB_EDGE'


# --- P1-G detector containment --------------------------------------------------

def test_stack_box_with_two_children_is_dropped_single_child_kept():
    record = []
    keep = carton_nms([[0, 0, 100, 100], [2, 2, 98, 50], [2, 52, 98, 98]], [.9, .8, .7], record=record)
    assert keep == [1, 2] and record[0]['index'] == 0 and record[0]['reason'] == 'STACK_BOX_CONTAINS_TWO_OR_MORE_CARTONS'
    record = []
    keep = carton_nms([[0, 0, 100, 100], [2, 2, 98, 50]], [.9, .8], record=record)
    assert keep == [0, 1] and not record


# --- P1-H stack support with an interval envelope -------------------------------

def interval_stack():
    from test_single_carton_stack_support import fixture
    obs, result, boxes, rejected, site, geometry, pts, valid = fixture()
    box = boxes[0]; T = np.asarray(box['pose'])
    box['measured_box'] = {k: deepcopy(box[k]) for k in ('pose', 'size_m', 'corners_m')}
    lo = np.array([-.075, -.05, -.40]); hi = np.array([.075, .05, .05])
    pose = T.copy(); pose[:3, 3] += (lo+hi)/2; size = hi-lo
    box.update(pose=pose.tolist(), size_m=size.tolist(), corners_m=corners(pose, size).tolist())
    box['uncertainty'] = dict(kind='MEASURED_BOUNDARY_INTERVALS', lower_m=lo.tolist(), upper_m=hi.tolist(),
        intervals=[dict(axis=a, side=s, measured_m=float((-.075, -.05, -.05)[a] if s < 0 else (.075, .05, .05)[a]),
                        outer_m=float(lo[a] if s < 0 else hi[a]),
                        method='MISSING_BOUNDARY_MAXIMUM_EXTENT' if (a, s) == (2, -1) else 'DIRECT_DEPTH_BOUNDARY')
                   for a in range(3) for s in (-1, 1)])
    return obs, result, boxes, rejected, site, geometry, pts, valid


def test_interval_upper_envelope_bottom_set_to_upper_visible_face_bound():
    # User decision 2026-11-20: the upper visible face decides; the lower
    # visible top (occlusion-limited) only checks consistency.
    args = interval_stack(); box = args[2][0]
    bodies, records = infer_supports(*args)
    assert records[0]['status'] == 'APPLIED' and records[0]['upper_envelope_bottom_set_to_plane']
    d = records[0]['plane']['offset_m']; n = np.asarray(records[0]['plane']['normal_world'])
    e = records[0]['estimates']
    assert d == e['upper_visible_face_bottom_bound'] and records[0]['selected_from'] == ['upper_visible_face_bottom_bound']
    assert d <= e['upper_measured_bottom']-.003 and records[0]['seam_band_ownership']['plane_m'] == d
    assert abs(float(np.min(np.asarray(box['corners_m'])@n))-d) <= 1e-9   # binds planner detach rule
    assert box['size_m'][2] < .2 and box['quality']['confirmed_stack_support_clip']['outer_m'] < -.05
    assert any(i['method'] == 'CONFIRMED_STACK_SUPPORT_PLANE' for i in box['uncertainty']['intervals'])


def test_third_detection_outside_workspace_or_whole_stack_mask_is_not_counted():
    args = list(interval_stack()); masks = list(args[1].masks)
    group = masks[0] | masks[1]; outside = np.zeros_like(masks[0]); outside[0, 0] = True
    args[1] = SimpleNamespace(masks=masks+[group, outside])
    bodies, records = infer_supports(*args, outside_ignored=(3,))
    assert records[0]['status'] == 'APPLIED'
    effective, counting = effective_instances(args[1].masks, (3,))
    assert effective == [0, 1] and counting['group_masks'] == [2]


# --- coplanar seam policy -------------------------------------------------------

def test_seam_policy_configuration():
    assert coplanar_seam.policy({}) is None and coplanar_seam.policy({'coplanar_seam_policy': {'mode': 'DISABLED'}}) is None
    assert coplanar_seam.policy({'coplanar_seam_policy': SEAM})['uncertainty_m'] == .005
    for bad in (dict(SEAM, uncertainty_m=.004), dict(SEAM, uncertainty_m=.021), dict(SEAM, basis=' '),
                dict(SEAM, mode='ON'), {k: v for k, v in SEAM.items() if k != 'basis'}):
        with pytest.raises(ValueError):coplanar_seam.policy({'coplanar_seam_policy': bad})


def test_seam_applies_on_touching_coplanar_neighbour_and_contains_truth():
    off, pose, size = run('coplanar_touching_left')
    on, _, _ = run('coplanar_touching_left', SEAM)
    assert methods(off)['w-'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    interval = next(i for i in off['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['missing_boundary_class'] == 'COPLANAR_CONTINUATION'
    seam = next(i for i in on['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert seam['method'] == 'COPLANAR_SEAM_MASK_RGB_EDGE_POLICY'
    assert seam['evidence_kind'] == 'USER_POLICY_RGB_SEAM_NOT_DEPTH_BOUNDARY' and seam['basis'] == SEAM['basis']
    w_plus = next(i for i in on['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, 1))
    pad = w_plus['outer_m']-w_plus['measured_m']
    assert on['size_m'][0] <= size[0]+2*.005+pad+1e-9
    assert outside_mm(on, pose, size) == 0.
    assert on['measured_size_m'] == off['measured_size_m']


def test_seam_policy_leaves_gap_scene_unchanged():
    off, _, _ = run('left_gap_30mm'); on, _, _ = run('left_gap_30mm', SEAM)
    assert off['size_m'] == on['size_m'] and methods(off) == methods(on)


def test_seam_not_applied_at_detection_box_edge_without_adjacent_instance():
    # A box-truncated mask has no detected instance across the cut.
    box, _, _ = run('coplanar_touching_left', SEAM, bbox_slack=0, only_target_mask=True)
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    assert interval['missing_boundary_class'] == 'DETECTION_BOX_EDGE'
    assert interval['missing_boundary']['detection_box_edge_adjacent_instance_fraction'] == 0.


def test_detection_box_edge_explained_by_adjacent_instance_keeps_seam_and_truth():
    # User decision 2026-11-20 (implementer's reading): SAM masks touch tight
    # DINO boxes; a detected neighbour across the edge explains the contact.
    box, pose, size = run('coplanar_touching_left', SEAM, bbox_slack=0)
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] == 'COPLANAR_SEAM_MASK_RGB_EDGE_POLICY'
    assert interval['seam']['detection_box_edge_explained_by_adjacent_instance'] is True
    assert interval['seam']['detection_box_edge_adjacent_instance_fraction'] >= coplanar_seam.ADJACENT_INSTANCE_FRACTION
    assert outside_mm(box, pose, size) == 0.


def test_seam_not_applied_when_edge_is_a_foreground_occluder():
    up = fl.upper_box(); neighbour = fl.side_neighbour(-1, 0., 0.); cam = fl.T_base_cam()
    t = up[0][:3, 0]; edge = np.asarray(fl.UPPER_FRONT)-t*fl.UPPER[0]/2
    ray = cam[:3, 3]-edge; ray /= np.linalg.norm(ray)
    pole = (fl.box_pose(edge+ray*.08, (.02, .30, .02)), (.02, .30, .02))   # on the line of sight to the edge
    p, ident, cam = fl.render([up, neighbour, pole]); p = fl.stereo_shadow(fl.add_noise(p))
    masks = [ident == 0, ident == 1, ident == 2]; ys, xs = np.nonzero(masks[0])
    box = measured_instance(p, masks, 0, cam, rgb=fl.gray(ident), maximum=.4, seam_policy=SEAM,
                            bbox=[xs.min()-6, ys.min()-6, xs.max()+6, ys.max()+6])
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    assert interval['missing_boundary_class'] == 'FOREGROUND_OCCLUDER'


def test_protruding_neighbour_side_is_not_a_seam():
    up = fl.upper_box(); neighbour = fl.side_neighbour(-1, 0., -.02)
    p, ident, cam = fl.render([up, neighbour]); p = fl.stereo_shadow(fl.add_noise(p))
    masks = [ident == 0, ident == 1]; ys, xs = np.nonzero(masks[0])
    box = measured_instance(p, masks, 0, cam, rgb=fl.gray(ident), maximum=.4, seam_policy=SEAM,
                            bbox=[xs.min()-6, ys.min()-6, xs.max()+6, ys.max()+6])
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] != 'COPLANAR_SEAM_MASK_RGB_EDGE_POLICY'


def test_seam_not_applied_when_rgb_edge_is_not_stronger_than_texture():
    p, masks, image, cam, _ = fl.frame('coplanar_touching_left')
    rng = np.random.default_rng(3)
    textured = np.clip(image.astype(int)+rng.integers(-60, 61, image.shape[:2])[..., None], 0, 255).astype(np.uint8)
    ys, xs = np.nonzero(masks[0])
    try:box = measured_instance(p, masks, 0, cam, rgb=textured, maximum=.4, seam_policy=SEAM,
                                bbox=[xs.min()-6, ys.min()-6, xs.max()+6, ys.max()+6])
    except ValueError:return   # rejected outright is also safe
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    assert interval['missing_boundary_class'] == 'NO_RGB_EDGE'


def test_seam_not_applied_at_image_border():
    p, masks, image, cam, _ = fl.frame('coplanar_touching_left')
    raw = recover(p, masks[0], cam, rgb=image)
    x0 = int(np.nonzero(masks[0])[1].min())-1
    b = conservative_envelope(raw, p[:, x0:], masks[0][:, x0:], cam, rgb=image[:, x0:], seam_policy=SEAM)
    interval = next(i for i in b['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    assert interval['missing_boundary_class'] == 'IMAGE_BORDER'


def test_missing_edges_are_classified_and_displayed():
    box, _, _ = run('coplanar_touching_left')
    text = edge_summary(box)
    assert '缺边（共面延续）' in text and '宽-' in text


def test_seam_band_contact_is_reported_separately_without_permission():
    on, _, _ = run('coplanar_touching_left', SEAM)
    seam = next(i for i in on['uncertainty']['intervals'] if i['method'] == 'COPLANAR_SEAM_MASK_RGB_EDGE_POLICY')
    M = np.asarray(on['measured_box']['pose']); axis = M[:3, 0]
    lo = np.asarray(on['uncertainty']['lower_m']); hi = np.asarray(on['uncertainty']['upper_m'])
    # neighbour face 2 mm inside the seam position: 3 mm into the 5 mm band
    seam_position = seam['outer_m']+seam['uncertainty_m']
    centre = M[:3, 3]+M[:3, :3]@np.array([seam_position-.002-.1, (lo[1]+hi[1])/2, (lo[2]+hi[2])/2])
    pose = np.eye(4); pose[:3, :3] = M[:3, :3]; pose[:3, 3] = centre
    world = [dict(id='neighbour', pose=pose.tolist(), size=[.2, .1, .1])]
    rest = dict(intersections=[dict(id='neighbour', category='observed', overlap_depth_m=.003)])
    result = seam_band_contacts(world, on, rest)
    assert result['all_within_seam_band'] and not result['permission_granted']
    pose2 = pose.copy(); pose2[:3, 3] += axis*.02
    world2 = [dict(id='neighbour', pose=pose2.tolist(), size=[.2, .1, .1])]
    assert not seam_band_contacts(world2, on, rest)['all_within_seam_band']


def test_depth_fattening_at_a_silhouette_is_not_a_seam():
    """Field frame d6d985f1: stereo matching extends the carton's depth ~4 px
    over the wall beyond its silhouette. That is not a coplanar neighbour."""
    up = fl.upper_box(); p, ident, cam = fl.render([up]); p = fl.add_noise(p)
    masks = [ident == 0]; ys, xs = np.nonzero(masks[0])
    plane_point = np.linalg.inv(cam)@np.r_[fl.UPPER_FRONT, 1.]; n_cam = (np.linalg.inv(cam)[:3, :3]@up[0][:3, 2])
    rays = p/p[..., 2, None]
    band = cv2.dilate(masks[0].astype('uint8'), np.ones((1, 9), 'uint8')).astype(bool) & ~masks[0]
    band &= np.indices(masks[0].shape)[1] < int(xs.mean())          # left silhouette only
    left = np.indices(masks[0].shape)[1] < int(xs.mean())
    p[left & ~masks[0] & ~band] = np.nan              # invalid (occlusion band) beyond the fattening, as in the field
    depth = (plane_point[:3]@n_cam)/(rays[band]@n_cam); p[band] = rays[band]*depth[:, None]
    box = measured_instance(p, masks, 0, cam, rgb=fl.gray(ident), maximum=.4, seam_policy=SEAM,
                            bbox=[xs.min()-6, ys.min()-6, xs.max()+6, ys.max()+6])
    interval = next(i for i in box['uncertainty']['intervals'] if (i['axis'], i['side']) == (0, -1))
    assert interval['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    assert interval['missing_boundary_class'] in ('INVALID_DEPTH_BAND', 'OTHER')


def test_check_config_reports_invalid_seam_policy_and_accepts_field_value():
    import json
    from pathlib import Path
    from unloading_sim.workbench.single_carton.config import issues, load
    root = Path(__file__).resolve().parents[1]
    base = json.loads((root/'configs/workbench/single_carton.example.json').read_text())
    assert base['geometry']['coplanar_seam_policy'] == {'mode': 'DISABLED'}
    config = load(root/'configs/workbench/single_carton.example.json')
    assert not any('coplanar_seam_policy' in i for i in issues(config))
    for bad in (dict(SEAM, uncertainty_m=.004), dict(SEAM, basis='')):
        config['geometry']['coplanar_seam_policy'] = bad
        assert any('coplanar_seam_policy' in i for i in issues(config))
    config['geometry']['coplanar_seam_policy'] = SEAM
    assert not any('coplanar_seam_policy' in i for i in issues(config))


# --- 2026-11-20 user decisions ----------------------------------------------------

from unloading_sim.workbench.single_carton import stereo_shadow
from unloading_sim.workbench.single_carton.measured_envelope import own_connected_surface, depth_jump
from unloading_sim.workbench.single_carton.site_scene import stack_seam_band_claims

SHADOW = dict(mode='INVALID_BAND_BEFORE_FARTHER_RETURN_IS_BACKGROUND',
              basis='USER_DECISION_2026_11_20_D435_LEFT_NO_RETURN_BAND_IS_THE_BACKGROUND_SCREEN_SEPARATED_BY_DEPTH_DIFFERENCE')


def interval(box, name):
    return next(i for i in box['uncertainty']['intervals'] if 'wht'[i['axis']]+('-' if i['side'] < 0 else '+') == name)


def test_stereo_shadow_policy_configuration():
    assert stereo_shadow.policy({}) is None
    assert stereo_shadow.policy({'stereo_shadow_boundary_policy': {'mode': 'DISABLED'}}) is None
    assert stereo_shadow.policy({'stereo_shadow_boundary_policy': SHADOW})['basis'] == SHADOW['basis']
    for bad in (dict(SHADOW, basis=' '), dict(SHADOW, mode='ON'), {'mode': SHADOW['mode']}, dict(SHADOW, extra=1)):
        with pytest.raises(ValueError):stereo_shadow.policy({'stereo_shadow_boundary_policy': bad})


@pytest.mark.parametrize('name', ['noisy_visible_support', 'left_gap_30mm'])
def test_stereo_shadow_policy_bounds_left_edge_and_contains_truth(name):
    off, pose, size = run(name)
    on, _, _ = run(name, shadow=SHADOW)
    assert interval(off, 'w-')['method'] == 'SENSOR_FREE_RECTANGLE_SPAN_BOUND'
    w = interval(on, 'w-')
    assert w['method'] == 'STEREO_SHADOW_DEPTH_STEP_POLICY' and w['basis'] == SHADOW['basis']
    assert w['evidence_kind'] == 'USER_POLICY_INVALID_BAND_IS_BACKGROUND_BEYOND_DEPTH_STEP'
    assert w['stereo_shadow']['coverage']['accepted'] and w['stereo_shadow']['no_return_band_px']['max'] > 0
    assert on['size_m'][0] < off['size_m'][0]-.005
    assert outside_mm(on, pose, size) == 0. and on['measured_size_m'] == off['measured_size_m']
    # Only the stereo-shadow (image-left) side is eligible.
    assert interval(on, 'h+')['stereo_shadow']['reason'] == 'EDGE_NOT_ON_STEREO_SHADOW_SIDE'


def test_stereo_shadow_never_applies_to_a_coplanar_continuation():
    box, pose, size = run('coplanar_touching_left', shadow=SHADOW)
    w = interval(box, 'w-')
    assert w['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT' and w['stereo_shadow']['reason'] == 'NO_SHADOW_WITNESS'
    assert outside_mm(box, pose, size) == 0.


def test_stereo_shadow_needs_a_farther_return_beyond_the_band():
    p, masks, image, cam, (pose, size) = fl.frame('left_gap_30mm')
    target = masks[0]; left = np.nonzero(target.any(0))[0].min()
    q = p.copy(); beyond = ~target; beyond[:, left:] = False
    q[beyond] *= .7                       # everything left of the carton is now NEARER
    box = measured_instance(q, masks, 0, cam, rgb=image, maximum=.4, shadow_policy=SHADOW)
    w = interval(box, 'w-')
    assert w['method'] != 'STEREO_SHADOW_DEPTH_STEP_POLICY'
    assert w['stereo_shadow']['rejected_rows'].get('NO_FARTHER_RETURN', 0) > 0


def test_own_mask_returns_off_the_planes_grow_the_envelope():
    # User confirmation 2026-11-20: a bulging/skewed side (own-mask returns
    # depth-connected to the measured faces) is the carton; it must be inside.
    up = fl.upper_box(); t = up[0][:3, 0]; n = up[0][:3, 2]
    slab = (fl.box_pose(np.asarray(fl.UPPER_FRONT)+t*(fl.UPPER[0]/2+.003)-n*.07, (.006, .08, .03)), (.006, .08, .03))
    p, ident, cam = fl.render([up, slab]); p = fl.stereo_shadow(fl.add_noise(p))
    box = measured_instance(p, [(ident == 0) | (ident == 1)], 0, cam, rgb=fl.gray(ident), maximum=.4)
    grown = interval(box, 'w+')['own_surface_extension']
    assert grown['outer_m'] > grown['previous_outer_m']+.002 and grown['basis'].startswith('USER_CONFIRMATION_2026_11_20')
    assert outside_mm(box, *slab) == 0. and outside_mm(box, *up) == 0.


def test_own_connected_surface_stops_at_depth_jumps_and_other_instances():
    z = np.full((6, 8), .40); z[:, 5:] = .80        # mask bleeds onto the far background at column 5
    valid = np.ones(z.shape, bool); mask = np.ones(z.shape, bool); seed = np.zeros(z.shape, bool); seed[2, 1] = True
    own = own_connected_surface(mask & valid & ~depth_jump(z, valid), seed)
    assert own[:, :4].all() and not own[:, 4:].any()


def test_stack_seam_band_points_follow_the_upper_visible_face():
    # Rows are heights 0.10 .. 0.01 m; the plane is 0.05 m, the upper measured bottom 0.066 m.
    h, w = 10, 8; z = .10-np.arange(h)*.01
    world = np.zeros((h, w, 3)); world[..., 2] = z[:, None]; world[..., 0] = np.arange(w)[None, :]*.001
    upper = np.zeros((h, w), bool); upper[:4] = True; lower = np.zeros((h, w), bool); lower[4:] = True
    third = np.zeros((h, w), bool); third[3:6, 7] = True; lower &= ~third; upper &= ~third
    box = dict(id='up', pose=np.eye(4).tolist(), size_m=[.2, .2, .2], uncertainty=dict(kind='MEASURED_BOUNDARY_INTERVALS'))
    box['pose'][2][3] = .15                         # envelope bottom at 0.05 m
    record = dict(status='APPLIED', upper_id='up', seam_band_ownership=dict(upper_index=0, lower_index=1,
        plane_m=.05, band_top_m=.07, normal_world=[0, 0, 1.], image_band_px=15))
    evidence = [dict(target_id='up')]
    owned = stack_seam_band_claims([box], [record], [upper, lower, third], np.ones((h, w), bool), world, evidence)
    assert owned[3:6, :7].all() and not owned[:3].any() and not owned[6:].any()   # band only, never below the plane
    assert not owned[:, 7].any()                                                  # another instance is never claimed
    assert record['seam_band_points_assigned_to_upper'] == dict(upper_instance=7, lower_instance=14, unlabelled=0)
    assert evidence[0]['stack_seam_band_points_claimed'] == record['seam_band_points_assigned_to_upper']


@pytest.mark.parametrize('name', ['ideal_stack', 'noisy_visible_support', 'yaw_+20'])
def test_front_face_bound_is_fitted_plane_plus_depth_noise(name):
    # User request 2026-11-20 (field 6e13d059): no pixel-footprint pad or
    # extreme-sample double count along the face normal; truth stays inside.
    box, pose, size = run(name)
    front = interval(box, 't+')
    assert front['rule'] == 'FITTED_FRONT_PLANE_PLUS_DEPTH_NOISE_ALLOWANCE_NOT_EXTREME_SAMPLE_PLUS_PIXEL_PAD'
    assert front['outer_m'] == pytest.approx(max(front['fitted_plane_m']+.003, front['extreme_accepted_sample_m']))
    assert front['outer_m']-front['fitted_plane_m'] <= .0035 and outside_mm(box, pose, size) == 0.
    assert box['uncertainty']['upper_m'][2] == pytest.approx(front['outer_m'])


def _front_bulge(protrusion):
    up = fl.upper_box(); n = up[0][:3, 2]
    centre = np.asarray(fl.UPPER_FRONT)+n*protrusion+np.array([0., 0., -(fl.UPPER[1]/2-.008)])
    depth = protrusion+.003
    bulge = (fl.box_pose(centre, (.10, .012, depth)), (.10, .012, depth))
    p, ident, cam = fl.render([up, bulge]); p = fl.stereo_shadow(fl.add_noise(p))
    box = measured_instance(p, [(ident == 0) | (ident == 1)], 0, cam, rgb=fl.gray(ident), maximum=.4)
    return box, up, bulge


def test_front_bulge_within_face_band_is_contained():
    box, up, bulge = _front_bulge(.0025)
    front = interval(box, 't+')
    assert outside_mm(box, *up) == 0. and outside_mm(box, *bulge) == 0.
    assert front['outer_m']-front['fitted_plane_m'] <= .004+1e-9


def test_front_returns_beyond_face_band_are_recorded_not_grown():
    box, up, bulge = _front_bulge(.009)
    front = interval(box, 't+')
    assert front['outer_m']-front['fitted_plane_m'] <= .004+1e-9
    assert box['quality']['own_surface_points_in_front_beyond_face_band'] > 0
    assert outside_mm(box, *up) == 0.


# --- RGB colour boundary policy (user decision 2026-11-20) ---------------------

from unloading_sim.workbench.single_carton import rgb_color_boundary

COLOUR = dict(mode='CARTON_COLOUR_DIFFERS_FROM_SURROUNDINGS', uncertainty_m=.005,
              basis='USER_DECISION_2026_11_20_RGB_COLOUR_EASILY_SEPARATES_THE_CARTON_FROM_ITS_SURROUNDINGS')


def _coloured(ident, colours, seed=3):
    rgb = np.full((*ident.shape, 3), 230, np.uint8)
    for k, c in colours.items():rgb[ident == k] = c
    noise = np.random.default_rng(seed).normal(0, 3, rgb.shape)
    return np.clip(rgb+noise, 0, 255).astype(np.uint8)


def _left_band_without_returns(objects, colours, foreground=None):
    """Target whose left edge has no depth to the image border (as on the field's left)."""
    p, ident, cam = fl.render(objects); p = fl.add_noise(p)
    target = ident == 0; left = int(np.nonzero(target.any(0))[0].min())
    p = p.copy(); columns = np.indices(target.shape)[1]
    p[~target & (columns < left+25)] = np.nan              # no return left of the carton, to the border
    if foreground is not None:p[foreground] = p[foreground]*.8   # nearer than the face
    masks = [ident == k for k in range(len(objects))]
    return p, masks, _coloured(ident, colours), cam


def test_rgb_colour_policy_configuration():
    assert rgb_color_boundary.policy({}) is None
    assert rgb_color_boundary.policy({'rgb_color_boundary_policy': {'mode': 'DISABLED'}}) is None
    assert rgb_color_boundary.policy({'rgb_color_boundary_policy': COLOUR})['uncertainty_m'] == .005
    for bad in (dict(COLOUR, uncertainty_m=.004), dict(COLOUR, uncertainty_m=.03), dict(COLOUR, basis=''),
                dict(COLOUR, mode='ON'), {k: v for k, v in COLOUR.items() if k != 'basis'}):
        with pytest.raises(ValueError):rgb_color_boundary.policy({'rgb_color_boundary_policy': bad})


def test_rgb_colour_boundary_bounds_an_edge_without_depth_and_contains_truth():
    up = fl.upper_box()
    p, masks, rgb, cam = _left_band_without_returns([up], {0: (150, 110, 70)})
    off = measured_instance(p, masks, 0, cam, rgb=rgb, maximum=.4)
    on = measured_instance(p, masks, 0, cam, rgb=rgb, maximum=.4, colour_policy=COLOUR)
    assert interval(off, 'w-')['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    w = interval(on, 'w-')
    assert w['method'] == 'RGB_COLOR_BOUNDARY_POLICY' and w['basis'] == COLOUR['basis']
    assert w['evidence_kind'] == 'USER_POLICY_RGB_COLOUR_DIFFERS_FROM_SURROUNDINGS_NOT_DEPTH_BOUNDARY'
    assert on['size_m'][0] < off['size_m'][0]-.1 and outside_mm(on, *up) == 0.
    assert on['measured_size_m'] == off['measured_size_m']


def test_rgb_colour_boundary_not_applied_when_surroundings_share_the_carton_colour():
    up = fl.upper_box()
    p, masks, rgb, cam = _left_band_without_returns([up], {0: (150, 110, 70), -1: (150, 110, 70), -2: (150, 110, 70)})
    box = measured_instance(p, masks, 0, cam, rgb=rgb, maximum=.4, colour_policy=COLOUR)
    w = interval(box, 'w-')
    assert w['method'] == 'MISSING_BOUNDARY_MAXIMUM_EXTENT'
    assert w['missing_boundary']['rgb_color_boundary']['reason'] == 'OUTSIDE_HAS_CARTON_COLOUR'


def test_rgb_colour_boundary_not_applied_to_a_coplanar_object_of_another_colour():
    up = fl.upper_box(); neighbour = fl.side_neighbour(-1, 0., 0.)
    p, ident, cam = fl.render([up, neighbour]); p = fl.add_noise(p)
    rgb = _coloured(ident, {0: (150, 110, 70), 1: (60, 90, 160)})
    box = measured_instance(p, [ident == 0], 0, cam, rgb=rgb, maximum=.4, colour_policy=COLOUR)
    w = interval(box, 'w-')
    assert w['method'] != 'RGB_COLOR_BOUNDARY_POLICY'
    assert outside_mm(box, *up) == 0.


def test_rgb_colour_boundary_not_applied_behind_a_foreground_occluder():
    up = fl.upper_box()
    p0, ident, cam = fl.render([up]); target = ident == 0; left = int(np.nonzero(target.any(0))[0].min())
    occluder = np.zeros(target.shape, bool); occluder[:, max(0, left-12):left-2] = True
    p, masks, rgb, cam = _left_band_without_returns([up], {0: (150, 110, 70)})
    p[occluder] = p0[occluder]; p[occluder] *= .5            # a dark pole well in front of the face
    rgb[occluder] = (40, 40, 40)
    box = measured_instance(p, masks, 0, cam, rgb=rgb, maximum=.4, colour_policy=COLOUR)
    assert interval(box, 'w-')['method'] != 'RGB_COLOR_BOUNDARY_POLICY'


def test_precontact_standoff_is_configurable_and_bounded():
    from unloading_sim.workbench.single_carton.geometry import contact_candidates, contact_search_settings
    import json as json_
    from pathlib import Path as Path_
    tool = json_.loads(Path_('configs/local/eco65_desktop/tool.json').read_text(encoding='utf-8'))
    config = json_.loads(Path_('configs/workbench/single_carton.example.json').read_text(encoding='utf-8'))
    box, _, _ = run('ideal_stack')
    search = dict(reference_face_roll_deg=0, j6_rotation_deg=180, lift_offsets_m=[0.], max_candidates=4, basis='TEST')
    near = contact_candidates(box, tool, config['topology'], search)[0]
    far = contact_candidates(box, tool, config['topology'], dict(search, precontact_m=.06))[0]
    gap = lambda c: float(np.linalg.norm(np.asarray(c['precontact_pose'])[:3, 3]-np.asarray(c['tcp_pose'])[:3, 3]))
    assert gap(near) == pytest.approx(.03) and gap(far) == pytest.approx(.06)
    for bad in (.01, .2, '0.06'):
        with pytest.raises(ValueError):contact_search_settings(dict(search, precontact_m=bad))


def test_carton_colour_model_is_disabled_for_a_neutral_face():
    from unloading_sim.workbench.single_carton.rgb_color_boundary import carton_coloured
    mask = np.zeros((80, 80), bool); mask[10:70, 10:70] = True
    brown = np.full((80, 80, 3), 235, np.uint8); brown[mask] = (150, 110, 70)
    coloured = carton_coloured(brown, mask)
    assert coloured[40, 40] and not coloured[2, 2]
    grey = np.full((80, 80, 3), 235, np.uint8); grey[mask] = (120, 120, 120)
    assert carton_coloured(grey, mask) is None
