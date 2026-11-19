"""Nested instance fitting retains both raw masks and conservative geometry."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from unloading_sim.workbench.single_carton.geometry import recover,corners
from unloading_sim.workbench.single_carton.instance_fit import measured_instance
from test_single_carton_variable_geometry import measured_box


def two_cartons():
    yy,xx=np.indices((400,450));rays=np.stack(((xx-225)/450,(yy-200)/450,np.ones(xx.shape)),-1)
    R=Rotation.from_euler('y',25,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
    definitions=[([-.02,-.09,.7],[.24,.14,.12]),([0.,.065,.73],[.18,.14,.10])]
    depths=[]
    for center,size in definitions:
        origin=-np.asarray(center)@R;directions=rays@R;half=np.asarray(size)/2
        with np.errstate(divide='ignore',invalid='ignore'):
            a=(-half-origin)/directions;b=(half-origin)/directions
            near=np.minimum(a,b).max(-1);far=np.maximum(a,b).min(-1)
        depths.append(np.where((near<far)&(near>.1),near,np.inf))
    stack=np.asarray(depths);which=stack.argmin(0);depth=stack.min(0);visible=np.isfinite(depth)
    masks=[visible&(which==i) for i in range(2)]
    points=rays*np.minimum(depth,1.4)[...,None]
    truth=np.eye(4);truth[:3,:3]=R;truth[:3,3]=definitions[0][0]
    return points,masks,corners(truth,definitions[0][1])


def test_nested_lower_detection_repairs_mixed_upper_fit_without_deleting_raw_evidence():
    p,m,truth=two_cartons();raw=[m[0]|m[1],m[1].copy()];before=[x.copy() for x in raw]
    with pytest.raises(ValueError,match='MASK_MIXED_NEIGHBOR'):recover(p,raw[0],np.eye(4),conservative_measured=True)
    box=measured_instance(p,raw,0,np.eye(4),rgb=None,maximum=.4)
    fit=box['instance_fit'];assert fit['excluded'][0]['detection_index']==1
    assert fit['original_mask_retained_for_envelope'] and fit['original_scene_evidence_retained']
    assert not fit['excluded_pixels_are_free_space']
    for a,b in zip(before,raw):assert np.array_equal(a,b)
    T=np.asarray(box['pose']);assert np.max(abs((truth-T[:3,3])@T[:3,:3])-np.asarray(box['size_m'])/2)<1e-7
    from unloading_sim.workbench.single_carton.site_scene import _target_claims
    claimed,_,_=_target_claims(box,raw[0],np.isfinite(p).all(-1),p,raw[1])
    assert not claimed[raw[1]].any()


def test_non_nested_detection_does_not_remove_part_of_a_mixed_mask():
    p,m,_=two_cartons();raw=m[0]|m[1];unrelated=np.roll(m[1],150,axis=1)
    with pytest.raises(ValueError,match='MASK_MIXED_NEIGHBOR'):
        measured_instance(p,[raw,unrelated],0,np.eye(4),rgb=None,maximum=.4)


def test_duplicate_or_partial_same_object_does_not_change_a_valid_fit():
    p,m,_,_=measured_box();partial=m.copy();partial[:,:200]=False
    b=measured_instance(p,[m,m.copy(),partial],0,np.eye(4),rgb=None,maximum=.4)
    assert 'instance_fit' not in b


def test_envelope_evidence_must_contain_fit_mask_and_remain_measured_only():
    p,m,_,_=measured_box();smaller=m.copy();smaller[:,:200]=False
    with pytest.raises(ValueError,match='INVALID_ENVELOPE_EVIDENCE_MASK'):
        recover(p,m,np.eye(4),conservative_measured=True,envelope_mask=smaller)
    with pytest.raises(ValueError,match='INVALID_ENVELOPE_EVIDENCE_MASK'):
        recover(p,m,np.eye(4),prior=(.24,.18,.12),conservative_measured=True,envelope_mask=m)


def test_raw_points_outside_fit_patch_still_bound_complete_payload():
    p,m,c,R=measured_box();fit=m.copy();cut=np.nonzero(m)[1].min()+12;fit[:,:cut]=False
    b=recover(p,fit,np.eye(4),conservative_measured=True,envelope_mask=m)
    T=np.asarray(b['pose']);truth=np.eye(4);truth[:3,:3]=R;truth[:3,3]=c
    assert np.max(abs((corners(truth,[.24,.18,.12])-T[:3,3])@T[:3,:3])-np.asarray(b['size_m'])/2)<1e-7
