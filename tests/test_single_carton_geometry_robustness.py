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
from unloading_sim.workbench.single_carton.measured_envelope import conservative_envelope, support_clip, SUPPORT_TILT_LIMIT_RAD

SEAM = dict(mode='MASK_RGB_EDGE_COINCIDENT', uncertainty_m=.005,
            basis='USER_DECISION_2026_11_19_COPLANAR_SEAM_TRUST_MASK_BOUNDARY_COINCIDENT_WITH_STRONG_RGB_EDGE_PLUS_MINUS_5_MM')
NAMES = ['ideal_stack', 'noisy_visible_support', 'flush_stack', 'flush_stack_mask_+1px', 'stack_mask_+1px',
         'stack_mask_+3px', 'stack_mask_-2px', 'stack_samlike_0.35', 'frontal_yaw_0', 'yaw_-12', 'yaw_+20',
         'coplanar_touching_left', 'left_gap_30mm', 'parent_mask_with_child', 'packed_wall']
CACHE = {}


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
