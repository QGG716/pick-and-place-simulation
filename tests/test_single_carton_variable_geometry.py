"""Variable measured cartons; no controller/camera/serial writes."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from unloading_sim.workbench.single_carton.geometry import recover,GeometryRejected,contact_candidates,seal_check
from unloading_sim.workbench.single_carton.perception import carton_nms,reconstruct
from unloading_sim.workbench.contracts import RGBDFrame,Stamp,DeviceTime,DetectionResult

ROOT=Path(__file__).resolve().parents[1]


def measured_box(size=(.24,.18,.12),angles=(10,30,15)):
    """Pinhole rays intersect literal oriented solid, with original depth grid."""
    shape=(350,400);fy=fx=450.;y,x=np.indices(shape)
    ray=np.stack(((x-200)/fx,(y-175)/fy,np.ones(shape)),-1)
    R=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
    center=np.array([0.,0.,.7]);origin=-center@R;directions=ray@R;half=np.asarray(size)/2
    with np.errstate(divide='ignore',invalid='ignore'):
        a=(-half-origin)/directions;b=(half-origin)/directions
        near=np.minimum(a,b).max(-1);far=np.maximum(a,b).min(-1)
    mask=(near<far)&(near>.1);depth=np.where(mask,near,1.4)
    points=ray*depth[...,None]
    return points,mask,center,R


@pytest.mark.parametrize('size',[(.24,.18,.12),(.32,.21,.16),(.12,.08,.06)])
def test_two_visible_faces_measure_different_cartons_without_any_size_prior(size):
    p,m,c,R=measured_box(size)
    b=recover(p,m,np.eye(4))
    np.testing.assert_allclose(sorted(b['size_m']),sorted(size),atol=.006)
    np.testing.assert_allclose(np.asarray(b['pose'])[:3,3],c,atol=.004)
    np.testing.assert_allclose(b['faces'][0]['normal'],R[:,2],atol=1e-5)
    assert b['dimension_source']=='RGBD_VISIBLE_FACE_AND_SIDE_BOUNDARIES_NO_SIZE_PRIOR'
    assert b['quality']['hidden_extent_measured'] and not b['quality']['unseen_surfaces_observed']
    assert len(b['corners_m'])==8


def test_single_face_exposes_measured_patch_but_never_invents_thickness():
    p,m,_,_=measured_box(angles=(0,0,0))
    with pytest.raises(GeometryRejected,match='THICKNESS_UNOBSERVED') as e:recover(p,m,np.eye(4))
    face=e.value.evidence['observed_face']
    np.testing.assert_allclose(sorted(face['size_m']),[.18,.24],atol=.006)
    assert not e.value.evidence['hidden_extent_measured']


def test_variable_large_face_accepts_all_twelve_common_valve_cups():
    p,m,_,_=measured_box((.30,.22,.14));b=recover(p,m,np.eye(4))
    tool=json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())
    c=json.loads((ROOT/'configs/workbench/single_carton.example.json').read_text())
    candidate=contact_candidates(b,tool,c['topology'])[0]
    assert len(candidate['active_ids'])==12
    assert candidate['seal_check']['observed_surface_covered']


def test_seal_rejects_unobserved_hole_even_when_rectangle_edges_fit():
    face=dict(center_m=[0,0,0],normal=[0,0,1],u=[1,0,0],v=[0,1,0],size_m=[.3,.2],
              support_points_uv_m=[[-.1,0],[.1,0]],support_radius_m=.004)
    T=np.diag([1.,-1.,-1.,1.]);ring={'cup':[[.01,0,0],[-.01,0,0],[0,.01,0],[0,-.01,0]]}
    check=seal_check(T,face,ring,[0,0,1])
    assert not check['valid'] and not check['observed_surface_covered']


def test_synonymous_dino_labels_do_not_duplicate_physical_box():
    assert carton_nms([[10,10,100,100],[10.1,10,99.9,100],[120,10,200,100]],[.5,.4,.3])==[0,2]


def test_saved_duplicate_masks_are_not_duplicate_collision_objects(monkeypatch):
    p,m,_,_=measured_box();rgb=np.zeros((*m.shape,3),np.uint8)
    intr=dict(width=400,height=350,fx=450,fy=450,ppx=200,ppy=175,model='SYNTHETIC_PINHOLE',coeffs=[0]*5)
    frame=RGBDFrame('fixture','synthetic-camera',{},1,1,DeviceTime(1,'synthetic'),DeviceTime(1,'synthetic'),Stamp.receive(),intr,.001,rgb,(p[...,2]*1000).astype('uint16'))
    result=DetectionResult('fixture',frame.stamp.source,1,({'label':'box','frame_id':'camera_optical','score':.8},{'label':'carton','frame_id':'camera_optical','score':.8}),(m,m.copy()))
    monkeypatch.setattr('unloading_sim.workbench.single_carton.perception.metric_points',lambda f:p)
    boxes,rejected=reconstruct(SimpleNamespace(frame=frame,world_camera=np.eye(4)),result)
    assert len(boxes)==1 and rejected==[] and boxes[0]['detection_index']==0


def test_clipped_full_envelope_does_not_become_an_executable_small_box():
    p,m,_,_=measured_box();ys,xs=np.nonzero(m);cut=xs.min()+20
    with pytest.raises(GeometryRejected,match='CLIPPED'):
        recover(p[:,cut:],m[:,cut:],np.eye(4))


def test_overlapping_distinct_detections_are_retained_for_neighbor_collision():
    assert carton_nms([[10,10,100,100],[12,10,102,100]],[.5,.4])==[0,1]


def test_bowed_visible_side_is_in_payload_envelope_not_a_neighbor_obstacle():
    # Literal six half-spaces: the measured right wall leans out by 7.2 mm
    # toward the back. This is visible geometry, not a hidden-size prior.
    shape=(350,400);yy,xx=np.indices(shape)
    rays=np.stack(((xx-200)/450.,(yy-175)/450.,np.ones(shape)),-1)
    rotation=Rotation.from_euler('xyz',(10,30,15),degrees=True).as_matrix()@np.diag([1.,-1.,-1.])
    center=np.array([0.,0.,.7]);origin=-center@rotation;directions=rays@rotation
    normals=np.array([[1,0,.06],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1.]])
    offsets=np.array([.12+.06*.06,.12,.09,.09,.06,.06])
    denominator=directions@normals.T
    with np.errstate(divide='ignore',invalid='ignore'):
        hits=(offsets-origin@normals.T)/denominator
    near=np.where(denominator<0,hits,-np.inf).max(-1)
    far=np.where(denominator>0,hits,np.inf).min(-1)
    mask=(near<far)&(near>.1);points=rays*np.where(mask,near,1.4)[...,None]
    box=recover(points,mask,np.eye(4),conservative_measured=True)
    face=box['faces'][0];pose=np.asarray(box['pose']);half=np.asarray(box['size_m'])/2
    actual_local=(points-center)@rotation
    side=mask&(abs(actual_local@normals[0]-offsets[0])<1e-8)
    payload_local=(points[side]-pose[:3,3])@pose[:3,:3]
    # Side-normal extent is conservative for every measured side sample.
    assert np.max(abs(payload_local[:,0]))<=half[0]+1e-8
    assert box['measured_box']['size_m'][0]<box['size_m'][0]
    from unloading_sim.workbench.single_carton.site_scene import _target_claims
    owned,_,_=_target_claims(box,mask,mask,points,np.zeros(shape,bool))
    assert np.all(owned[side])  # Own observed side, not arbitrary envelope volume.
    assert max(box['quality']['observed_side_envelope_expansion_m'])>.005
    # No seal is gained by enlarging the collision envelope: its observed
    # plane and finite contact rectangle stay at the actual front.
    np.testing.assert_allclose(face['normal'],rotation[:,2],atol=1e-5)
    np.testing.assert_allclose(np.asarray(face['size_m']),[.24,.18],atol=.004)
    assert abs((np.asarray(face['center_m'])-center)@rotation[:,2]-.06)<1e-6
    assert not box['quality']['unseen_surfaces_observed']
