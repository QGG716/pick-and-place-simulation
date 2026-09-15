import numpy as np
import pytest

from unloading_perception.final_geometry import project
from unloading_perception.metric_support import backproject_pixels, native_pixel_map, check_metric_plane


K = np.array([[110., 0, 39.5], [0, 175., 29.5], [0, 0, 1]])


def test_optical_z_range_units_and_anisotropic_pixel_roundtrip():
    pixels = np.array([[0., 0.], [79., 59.], [39.5, 29.5], [15.3, 23.7]])
    points = backproject_pixels(pixels, np.full(4, 2.), K)
    assert np.allclose(project(points, K), pixels)
    assert np.allclose(points[:, 2], 2.)
    ranges = np.linalg.norm(points, axis=1)
    assert np.allclose(backproject_pixels(pixels, ranges, K, semantics='euclidean_range_m'), points)
    assert not np.allclose(backproject_pixels(pixels, ranges, K), points)
    with pytest.raises(ValueError):
        backproject_pixels(pixels, ranges, K, semantics='millimetres')


def test_native_final_map_preserves_rays_and_pixel_centres():
    native = K.copy(); native[1, 1] = native[0, 0]
    x, y = native_pixel_map((60, 80), native, K)
    final_y, final_x = np.indices((60, 80))
    a = backproject_pixels(np.column_stack((x.ravel(), y.ravel())), np.ones(x.size), native)
    b = backproject_pixels(np.column_stack((final_x.ravel(), final_y.ravel())), np.ones(x.size), K)
    assert np.allclose(a, b)
    assert y[0, 0] == pytest.approx((0-29.5)*110/175+29.5)


def test_frozen_face_support_accepts_plane_rejects_translation_and_rotation():
    depth = np.full((60, 80), 2.)
    instance = np.ones(depth.shape, bool)
    face = instance.copy(); face[:, 40:] = False
    depth[:, 40:] = 2.3  # another observed face, not this plane's support
    good = check_metric_plane([0, 0, 1], -2., depth, instance, face, K)
    translated = check_metric_plane([0, 0, 1], -2.02, depth, instance, face, K)
    rotated = check_metric_plane([np.sin(.1), 0, np.cos(.1)], -2*np.cos(.1), depth, instance, face, K)
    assert good['status'] == 'PASS'
    assert translated['status'] == rotated['status'] == 'REJECTED'
    assert good['selected_pixels'] == translated['selected_pixels'] == rotated['selected_pixels']
    whole = check_metric_plane([0, 0, 1], -2., depth, instance, instance, K)
    assert whole['status'] == 'REJECTED'
    assert whole['interior']['mean_m'] > .1


def test_boundary_contamination_reported_not_silently_trimmed_by_plane():
    depth = np.full((60, 80), 2.); face = np.zeros(depth.shape, bool); face[10:50, 10:70] = True
    depth[10, 10:70] = 3.
    result = check_metric_plane([0, 0, 1], -2., depth, face, face, K)
    assert result['status'] == 'PASS'
    assert result['raw']['mean_m'] > .003
    assert result['excluded']['mean_m'] > 0
    assert result['excluded_boundary'] > 0
    assert not result['candidate_plane_used_for_selection']
    assert check_metric_plane([0, 0, 1], -2.02, depth, face, face, K)['status'] == 'REJECTED'


def test_three_orthogonal_planes_intersections_and_per_module_transform():
    angle = .37
    R = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    corner = np.array([.2, -.1, 2.])
    normals = R.T; offsets = -normals @ corner
    assert np.allclose(np.linalg.solve(normals, -offsets), corner)
    for i, j in ((0, 1), (1, 2), (2, 0)):
        direction = np.cross(normals[i], normals[j])
        assert np.allclose((corner + .4*direction) @ normals[[i, j]].T + offsets[[i, j]], 0)
    # Deliberately reversed list order and distinct K/T; identity selects module.
    modules = [{'id': 'lower', 'K': K*1., 'R': R, 't': np.array([1., 2., 3.])},
               {'id': 'upper', 'K': K+np.diag([5., 9., 0]), 'R': R.T, 't': np.zeros(3)}]
    for identity in ('upper', 'lower'):
        camera = next(c for c in modules if c['id'] == identity)
        pixels = project(corner[None], camera['K'])
        p = backproject_pixels(pixels, [corner[2]], camera['K'])[0]
        world = camera['R'] @ p + camera['t']
        nw = normals @ camera['R'].T
        dw = offsets - nw @ camera['t']
        assert np.allclose(nw @ world + dw, 0)
