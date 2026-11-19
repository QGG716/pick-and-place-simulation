"""Literal raycast truth, independent of reconstruction bounds; no device IO."""
from itertools import product
import cv2
import numpy as np
import pytest
from test_single_carton_variable_geometry import measured_box
from unloading_sim.workbench.single_carton.geometry import recover,corners
from unloading_sim.workbench.single_carton.measured_envelope import conservative_envelope,max_carton_extent

SIZES=[(.24,.18,.12),(.32,.21,.16),(.12,.08,.06)]
ANGLES=[(10,30,15),(-15,25,0),(20,-30,25)]
CASES=list(product(SIZES,ANGLES,(0,1,2),(0,1,2,3)))


def scan_case(size,angles,noise_mm,gap_pixels):
    p,m,c,R=measured_box(size,angles);ray=p/p[...,2,None]
    rng=np.random.default_rng(6518)
    p=p+ray*rng.uniform(-noise_mm/1000,noise_mm/1000,m.shape)[...,None]
    if gap_pixels:
        interior=cv2.erode(m.astype('uint8'),np.ones((2*gap_pixels+1,)*2,'uint8')).astype(bool)
        p[m&~interior]=np.nan
    b=conservative_envelope(recover(p,m,np.eye(4)),p,m,np.eye(4))
    truth_pose=np.eye(4);truth_pose[:3,:3]=R;truth_pose[:3,3]=c
    truth=corners(truth_pose,size);T=np.asarray(b['pose'])
    violation=float(np.max(abs((truth-T[:3,3])@T[:3,:3])-np.asarray(b['size_m'])/2))
    M=np.asarray(b['measured_box']['pose']);truth_local=(truth-M[:3,3])@M[:3,:3]
    expansion=np.r_[truth_local.min(0)-b['uncertainty']['lower_m'],np.asarray(b['uncertainty']['upper_m'])-truth_local.max(0)]
    return dict(size_m=size,angles_deg=angles,noise_mm=noise_mm,missing_edge_pixels=gap_pixels,
        contained=violation<=1e-7,max_containment_violation_m=violation,
        outward_expansion_each_boundary_m=expansion.tolist(),max_outward_expansion_m=float(expansion.max()),
        boundary_methods=[x['method'] for x in b['uncertainty']['intervals']],
        expansion_over_6mm_basis=('FORESHORTENED_SURFACE_PIXEL_FOOTPRINT_PLUS_DEPTH_NOISE; '
            'MISSING_DIRECT_BOUNDARY_USES_MAXIMUM_EXTENT' if expansion.max()>.006 else None))


@pytest.mark.parametrize('size,angles,noise_mm,gap_pixels',CASES)
def test_full_true_cuboid_is_contained(size,angles,noise_mm,gap_pixels):
    result=scan_case(size,angles,noise_mm,gap_pixels)
    assert result['contained'],result
    if gap_pixels==0 and result['max_outward_expansion_m']>.006:
        assert result['expansion_over_6mm_basis']


def stacked_fixture():
    p,m,c,R=measured_box(angles=(0,30,0));W=np.eye(4)
    W[:3,:3]=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    rays=p/p[...,2,None];height=-.09
    with np.errstate(divide='ignore',invalid='ignore'):depth=height/rays[...,1]
    use=~m&(depth>.1)&(depth<1.4);p[use]=rays[use]*depth[use,None]
    return p,m,W,height


def test_stacked_envelope_stops_at_measured_support():
    p,m,W,height=stacked_fixture()
    b=recover(p,m,W,conservative_measured=True)
    assert abs(np.asarray(b['corners_m'])[:,2].min()-height)<1e-9
    assert any(i['method']=='MEASURED_SUPPORT_PLANE' for i in b['uncertainty']['intervals'])
    assert b['measured_box']['faces']==b['faces']


@pytest.mark.parametrize('g',[dict(max_carton_extent_m=.41,max_carton_extent_basis='x'),dict(max_carton_extent_m=.4)])
def test_carton_extent_requires_basis_and_cannot_be_relaxed(g):
    with pytest.raises(ValueError):max_carton_extent(g)


def test_enlarged_volume_cannot_claim_neighbor_depth():
    from unloading_sim.workbench.single_carton.site_scene import _target_claims
    p,m,_,_=measured_box();b=conservative_envelope(recover(p,m,np.eye(4)),p,m,np.eye(4))
    M=np.asarray(b['measured_box']['pose']);half=np.asarray(b['measured_size_m'])/2
    b['size_m']=[.8,.8,.8]
    local=np.array([[[half[0]+.04,0,half[2]]]])
    world=local@M[:3,:3].T+M[:3,3]
    yes=np.ones((1,1),bool)
    claimed,_,_=_target_claims(b,yes,yes,world,np.zeros_like(yes))
    assert not claimed.any()


def test_mass_estimate_uses_measured_center_and_dimensions():
    from unloading_sim.workbench.single_carton.measured_envelope import payload_mass_properties
    M=np.eye(4);M[:3,3]=[.01,-.02,.03]
    b=dict(pose=np.eye(4).tolist(),size_m=[.4,.4,.4],measured_box=dict(pose=M.tolist(),size_m=[.24,.18,.12]))
    props=payload_mass_properties(b,2.)
    np.testing.assert_allclose(props['com_box_m'],[.01,-.02,.03])
    np.testing.assert_allclose(np.diag(props['inertia_box_kg_m2']),[.0078,.012,.015])


def test_background_stereo_returns_inside_mask_do_not_become_carton_size():
    p,m,c,R=measured_box();interior=cv2.erode(m.astype('uint8'),np.ones((21,21),'uint8')).astype(bool)
    pixels=np.flatnonzero(interior)[::97];corrupted=p.copy().reshape(-1,3)
    corrupted[pixels]*=3.;corrupted=corrupted.reshape(p.shape)
    box=recover(corrupted,m,np.eye(4),conservative_measured=True)
    assert box['quality']['envelope_rejected_off_plane_depth_points']>=len(pixels)
    assert max(box['size_m'])<.45  # Far returns must not create a metre-scale box.
    truth=np.eye(4);truth[:3,:3]=R;truth[:3,3]=c;T=np.asarray(box['pose'])
    local=(corners(truth,[.24,.18,.12])-T[:3,3])@T[:3,:3]
    assert np.max(abs(local)-np.asarray(box['size_m'])/2)<1e-7
    from unloading_sim.workbench.single_carton.site_scene import _target_claims
    claimed,_,_=_target_claims(box,m,np.isfinite(corrupted).all(-1),corrupted,np.zeros_like(m))
    assert not claimed.ravel()[pixels].any()
