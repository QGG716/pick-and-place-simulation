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
from unloading_sim.workbench.single_carton.geometry import recover, corners, _visible_planes
from unloading_sim.workbench.single_carton.instance_fit import measured_instance
from unloading_sim.workbench.single_carton.measured_envelope import conservative_envelope, support_clip, SUPPORT_TILT_LIMIT_RAD
from unloading_sim.workbench.single_carton.hf_detector_core import carton_nms
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


def run(name, seam=None, bbox_slack=6, rgb=None):
    key = (name, None if seam is None else seam['uncertainty_m'], bbox_slack, rgb is None)
    if key not in CACHE:
        p, masks, image, cam, (pose, size) = fl.frame(name)
        ys, xs = np.nonzero(masks[0])
        bbox = [xs.min()-bbox_slack, ys.min()-bbox_slack, xs.max()+bbox_slack, ys.max()+bbox_slack]
        box = measured_instance(p, masks, 0, cam, rgb=image if rgb is None else rgb, maximum=.4)
        CACHE[key] = (box, pose, size)
    return CACHE[key]


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


def test_mask_cut_through_the_same_plane_without_rgb_edge_is_rejected():
    p, m, _, _ = measured_box((.24, .18, .12)); xs = np.nonzero(m)[1]
    cut = m.copy(); cut[:, :int(xs.mean())] = False
    flat = np.full((*m.shape, 3), 110, np.uint8)
    with pytest.raises(ValueError, match='COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED'):
        recover(p, cut, np.eye(4), rgb=flat, conservative_measured=True)


def test_stack_box_with_two_children_is_dropped_single_child_kept():
    record = []
    keep = carton_nms([[0, 0, 100, 100], [2, 2, 98, 50], [2, 52, 98, 98]], [.9, .8, .7], record=record)
    assert keep == [1, 2] and record[0]['index'] == 0 and record[0]['reason'] == 'STACK_BOX_CONTAINS_TWO_OR_MORE_CARTONS'
    record = []
    keep = carton_nms([[0, 0, 100, 100], [2, 2, 98, 50]], [.9, .8], record=record)
    assert keep == [0, 1] and not record


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


def test_interval_upper_envelope_bottom_set_to_lowest_support_estimate():
    args = interval_stack(); box = args[2][0]
    bodies, records = infer_supports(*args)
    assert records[0]['status'] == 'APPLIED' and records[0]['upper_envelope_bottom_set_to_plane']
    d = records[0]['plane']['offset_m']; n = np.asarray(records[0]['plane']['normal_world'])
    assert d == min(records[0]['estimates'].values())
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
