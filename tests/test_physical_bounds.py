"""Finite-bound math tests; these are not model accuracy evidence."""
from copy import deepcopy

import numpy as np
import pytest

from unloading_perception import metric_faces as mf


def measured_box(dimensions, angles, shape, noise=0.):
    from scipy.spatial.transform import Rotation
    h, w = shape
    focal = w*.9
    K = np.array([[focal, 0, (w-1)/2], [0, focal, (h-1)/2], [0, 0, 1.]])
    y, x = np.indices(shape)
    rays = np.stack(((x-K[0, 2])/focal, (y-K[1, 2])/focal, np.ones(shape)), -1)
    axes = Rotation.from_euler('xyz', angles, degrees=True).as_matrix().T
    center = np.array([.02, -.01, 2.])
    dimensions = np.array(dimensions)
    origin, direction = -center @ axes.T, rays @ axes.T
    with np.errstate(divide='ignore'):
        a = (-dimensions/2-origin)/direction
        b = (dimensions/2-origin)/direction
    enter, leave = np.minimum(a, b), np.maximum(a, b)
    z = enter.max(-1)
    mask = (z < leave.min(-1)) & (z > 0)
    depth = np.where(mask, z, 4.)
    depth[mask] += np.random.default_rng(17).normal(0, noise, mask.sum())
    hit = enter.argmax(-1)
    labels = np.full(shape, -1, np.int16)
    seeds = []
    for axis in np.unique(hit[mask]):
        region = mask & (hit == axis)
        labels[region] = len(seeds)
        p = rays[region][0]*z[region][0]
        seeds.append(dict(normal=axes[axis].tolist(), offset_m=float(-axes[axis] @ p),
                          initial_plane_index=int(axis), component=1))
    return depth, mask, K, labels, seeds, axes, center, dimensions


@pytest.mark.parametrize('dimensions,angles,shape,noise', [
    ((.55, .48, .42), (0, 30, 0), (300, 400), 0.),
    ((.7, .55, .45), (40, 30, 8), (360, 480), .0002),
    ((.5, .42, .35), (40, 30, -12), (480, 640), .0001),
])
def test_finite_two_and_three_faces_constrain_actual_dimensions(dimensions, angles, shape, noise):
    pytest.importorskip('cv2')
    d, m, k, labels, seeds, axes, center, dimensions = measured_box(dimensions, angles, shape, noise)
    out = mf.fit_metric_faces(d, m, k, labels, seeds, mask_id=9)
    assert out['accepted'], out.get('completion_diagnostics')
    assert out['boundary_constraints']['complete']
    assert out['finite_boundary_validation']['status'] == 'PASS'
    points = np.array(out['corners_3d']) @ axes.T
    expected = center @ axes.T
    assert np.max(abs(points.min(0)-(expected-dimensions/2))) < .01
    assert np.max(abs(points.max(0)-(expected+dimensions/2))) < .01
    from scipy.optimize import linear_sum_assignment
    from tools.build_notch_geometry_review import acceptance_config
    from unloading_perception.final_geometry import SIGNS
    thresholds=acceptance_config()
    reference=center+(SIGNS*dimensions/2)@axes
    distances=np.linalg.norm(np.array(out['corners_3d'])[:,None]-reference[None],axis=-1)
    a,b=linear_sum_assignment(distances)
    assert distances[a,b].max()<=thresholds['gt_certified_corner_m']
    assert np.linalg.norm(np.array(out['center_3d'])-center)<=thresholds['gt_complete_center_m']
    assert np.max(abs(np.ptp(points,axis=0)-dimensions))<=thresholds['gt_complete_dimension_m']


def test_correct_planes_do_not_validate_wrong_tangential_dimensions():
    pytest.importorskip('cv2')
    d, m, k, labels, seeds, *_ = measured_box((.55, .48, .42), (0, 30, 0), (300, 400))
    out = mf.fit_metric_faces(d, m, k, labels, seeds, mask_id=9)
    assert out['accepted']
    bad = deepcopy(out)
    axes = np.array(out['boundary_constraints']['axes_camera'])
    q = np.array(out['corners_3d']) @ axes.T
    free = next(b for b in out['boundary_constraints']['bounds'] if b['sources'][0]['source'] != 'MEASURED_PLANE')
    axis = free['axis']
    target = q[:, axis].min() if free['side'] == 'low' else q[:, axis].max()
    q[np.isclose(q[:, axis], target), axis] += -.04 if free['side'] == 'low' else .04
    bad['corners_3d'] = (q @ axes).tolist()
    checked = mf.validate_metric_record(bad, d, m, k)
    assert not checked['accepted']
    assert 'FINAL_TANGENTIAL_BOUND' in checked['complete_cuboid_consistency']['reason']
    assert not any(name in checked for name in mf._COMPLETE_FIELDS)


def test_partial_three_faces_and_eroded_mask_do_not_invent_hidden_bounds():
    cv2 = pytest.importorskip('cv2')
    d, m, k, labels, seeds, *_ = measured_box((.7, .55, .45), (40, 30, 8), (360, 480))
    # A severe segmentation truncation still leaves three interior planes. The
    # original RGB-D remains continuous beyond the mask; it supplies no edge.
    partial = cv2.erode(m.astype(np.uint8), np.ones((31, 31), np.uint8)).astype(bool)
    labels[~partial] = -1
    out = mf.fit_metric_faces(d, partial, k, labels, seeds, mask_id=9)
    assert out['reliable_plane_count'] == 3
    assert not out['accepted'] and out['boundary_constraints']['missing_bounds']
    assert not any(name in out for name in mf._COMPLETE_FIELDS)
    reasons = {s['kind'] for edge in out['boundary_constraints']['edges'] for s in edge['samples']}
    assert 'MASK_OR_LABEL_LIMIT' in reasons


def test_nearer_occlusion_is_not_an_outer_silhouette():
    pytest.importorskip('cv2')
    d, m, k, labels, seeds, *_ = measured_box((.7, .55, .45), (40, 30, 8), (360, 480))
    d[~m] = 1.
    out = mf.fit_metric_faces(d, m, k, labels, seeds, mask_id=9)
    assert not out['accepted']
    assert all(b['value_m'] is None for b in out['boundary_constraints']['bounds'] if b['status'] == 'UNRESOLVED')
    assert any(s['kind'] == 'OCCLUSION_BOUNDARY' for e in out['boundary_constraints']['edges'] for s in e['samples'])


def test_nearby_planes_do_not_certify_unobserved_or_bevelled_crease():
    k=np.array([[4000.,0,100.],[0,4000.,100.],[0,0,1.]])
    yy,xx=np.indices((201,201));depth=2/(1-abs((xx-100)/4000))
    n1=np.array([-1.,0,-1])/np.sqrt(2);n2=np.array([1.,0,-1])/np.sqrt(2)
    edge=np.array([[0,-.045,2.],[0,.045,2.]])
    groups=[dict(label=0,region=xx<99,normal=n1,offset=np.sqrt(2)),
            dict(label=1,region=xx>101,normal=n2,offset=np.sqrt(2))]
    sharp=mf._shared_edge_support(edge,*groups,depth,k)
    assert sharp['supported']
    depth[:,99:102]+=.02
    rounded=mf._shared_edge_support(edge,*groups,depth,k)
    assert rounded['supported_samples']==15
    assert not rounded['supported']
    assert rounded['reason']=='THEORETICAL_CREASE_NOT_ON_MEASURED_SURFACE'
    assert rounded['crease_depth_check']['p95_max_plane_distance_m']>.006


def test_review_acceptance_uses_the_shared_config(monkeypatch):
    pytest.importorskip('scipy')
    from tools import build_notch_geometry_review as review
    from unloading_perception.final_geometry import SIGNS
    reference=(SIGNS+1)*.2
    moved=reference+np.array([.02,0,0])
    strict=review.box_errors(moved,np.eye(4),reference)
    assert not strict['accuracy_checks']['center']
    config=review.acceptance_config()
    monkeypatch.setattr(review,'acceptance_config',lambda:{**config,'gt_complete_center_m':.03})
    changed=review.box_errors(moved,np.eye(4),reference)
    assert changed['accuracy_checks']['center']
    assert changed['accuracy_checks']['corners']==strict['accuracy_checks']['corners']
