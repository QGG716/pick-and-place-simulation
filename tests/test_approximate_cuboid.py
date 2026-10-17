"""CPU geometry contracts; synthetic solids are not real-capture evidence."""
from copy import deepcopy
from dataclasses import replace
import numpy as np
import pytest

from unloading_perception import metric_faces as mf
from unloading_perception.final_geometry import SIGNS,validate_final_record


def rounded_box(*,shape=(300,400),dimensions=(.62,.49,.41),angles=(40,30,8),radii=(.018,.016,.022),chamfer=False,center=(.02,-.01,2.)):
    """Raycast a true convex box + ellipsoid (or planar bevel) surface.

    Labels cover only its three planar interiors. All transition faces remain
    present in raw depth and the instance mask; no pixels are patched by hand.
    """
    pytest.importorskip('cv2');Rotation=pytest.importorskip('scipy.spatial.transform').Rotation
    h,w=shape;K=np.array([[w,0,(w-1)/2],[0,w,(h-1)/2],[0,0,1.]])
    y,x=np.indices(shape);rays=np.stack(((x-K[0,2])/w,(y-K[1,2])/w,np.ones(shape)),-1)
    axes=Rotation.from_euler('xyz',angles,degrees=True).as_matrix().T
    center=np.array(center);half=np.array(dimensions)/2;r=np.array(radii)
    origin=-center@axes.T;directions=rays@axes.T
    with np.errstate(divide='ignore'):
        a=(-half-origin)/directions;b=(half-origin)/directions
    enter=np.minimum(a,b).max(-1);leave=np.maximum(a,b).min(-1);possible=(enter<leave)&(enter>0)
    def field(t):
        q=origin+directions*t[...,None];v=np.maximum(abs(q)-(half-r),0)/r
        return (v.sum(-1) if chamfer else np.linalg.norm(v,axis=-1))-1
    # Convex line minimisation brackets the true front intersection, including
    # rays grazing the curved silhouette, followed by deterministic bisection.
    left=enter.copy();right=np.where(possible,leave,enter)
    for _ in range(35):
        u=(2*left+right)/3;v=(left+2*right)/3;first=field(u)<field(v)
        right=np.where(first,v,right);left=np.where(first,left,u)
    middle=(left+right)/2;mask=possible&(field(middle)<=0)
    left=enter.copy();right=middle
    for _ in range(40):
        t=(left+right)/2;outside=field(t)>0;left=np.where(outside,t,left);right=np.where(outside,right,t)
    depth=np.where(mask,(left+right)/2,4.)
    q=origin+directions*depth[...,None];labels=np.full(shape,-1,np.int16);seeds=[]
    for axis in range(3):
        side=np.sign(origin[axis]);region=mask&(abs(q[...,axis]-side*half[axis])<1e-6)
        if region.sum()<50:continue
        labels[region]=len(seeds);n=axes[axis]*side;point=center+axes[axis]*side*half[axis]
        seeds.append(dict(normal=n.tolist(),offset_m=float(-n@point),initial_plane_index=axis,component=1))
    return depth,mask,K,labels,seeds,axes,center,np.array(dimensions)


@pytest.mark.parametrize('shape,dimensions,angles',[
    ((300,400),(.62,.49,.41),(40,30,8)),
    ((360,480),(.72,.52,.45),(32,38,-10)),
])
def test_curved_three_face_model_survives_failed_sharp_diagnostic(shape,dimensions,angles):
    d,m,k,l,s,axes,center,dims=rounded_box(shape=shape,dimensions=dimensions,angles=angles)
    out=mf.fit_metric_faces(d,m,k,l,s,mask_id=87)
    assert out['reliable_plane_count']==3
    assert any(not edge['supported'] for edge in out['boundary_constraints']['shared_edges'])
    model=out['approximate_cuboid']
    assert model['status']=='VALIDATED_OBSERVATION_MODEL',model.get('raw_boundary_fit',model)
    assert not model['physical_corners_certified'] and not model['conservative_collision_envelope']
    from scipy.optimize import linear_sum_assignment
    from tools.build_notch_geometry_review import acceptance_config
    limits=acceptance_config();reference=center+(SIGNS*dims/2)@axes
    error=np.linalg.norm(np.array(model['corners_3d'])[:,None]-reference[None],axis=-1);a,b=linear_sum_assignment(error)
    assert error[a,b].max()<=limits['gt_certified_corner_m']
    assert np.linalg.norm(np.array(model['center_3d'])-center)<=limits['gt_complete_center_m']
    assert np.max(abs(np.sort(model['shape_dimensions'])-np.sort(dims)))<=limits['gt_complete_dimension_m']
    strict=mf.fit_metric_faces(d,m,k,l,s,mask_id=87,config=replace(mf.MetricFitConfig(),approximate_cuboid_mode='off'))
    assert not strict['accepted'] and 'approximate_cuboid' not in strict
    assert strict['camera_facing_faces']==out['camera_facing_faces']
    assert not out['accepted']
    from unloading_perception.rgbd import hypotheses_from_geometry_record,MetricPointMapSource
    assert hypotheses_from_geometry_record(out,source_instance_id='test',pointmap_source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH,presence_score=.9)==()
    # The production observation adapter supplies the flat calibration tuple.
    checked=validate_final_record(out,d,m,tuple(k.ravel()))
    assert checked['approximate_cuboid']['status']=='VALIDATED_OBSERVATION_MODEL'


def test_missing_far_extent_is_not_filled_after_removing_crease_gate():
    cv2=pytest.importorskip('cv2');d,m,k,l,s,*_=rounded_box()
    partial=cv2.erode(m.astype(np.uint8),np.ones((31,31),np.uint8)).astype(bool);l[~partial]=-1
    out=mf.fit_metric_faces(d,partial,k,l,s,mask_id=33)
    assert out['reliable_plane_count']==3 and not out['accepted']
    assert out['approximate_cuboid']['status']=='UNCONSTRAINED'
    assert 'corners_3d' not in out['approximate_cuboid']


def test_disconnected_face_bridge_and_depth_occlusion_are_rejected():
    from unloading_perception.approximate_cuboid import transition_consistency
    depth=np.full((30,30),2.);mask=np.ones_like(depth,bool);k=np.diag([30.,30.,1.])
    groups=[dict(label=i,normal=np.array([0,0,1.]),offset=-2.) for i in (0,1)]
    joint=dict(support_labels=[0,1],samples=[dict(surfaces=[dict(support_label=0,point_pixels=[[10,y]]),
        dict(support_label=1,point_pixels=[[18,y]])]) for y in range(7,22)])
    assert transition_consistency([joint],depth,mask,k,groups)['status']=='PASS'
    depth[:,14]=1.
    assert transition_consistency([joint],depth,mask,k,groups)['status']!='PASS'
    depth[:]=2.;mask[:,14]=False
    assert transition_consistency([joint],depth,mask,k,groups)['status']!='PASS'


def test_final_approximate_validation_rejects_wrong_tangential_size():
    d,m,k,l,s,*_=rounded_box();out=mf.fit_metric_faces(d,m,k,l,s,mask_id=91)
    assert out['approximate_cuboid']['status']=='VALIDATED_OBSERVATION_MODEL'
    wrong=deepcopy(out);model=wrong['approximate_cuboid'];axes=np.array(model['axes_camera']);q=np.array(model['corners_3d'])@axes.T
    q[np.isclose(q[:,0],q[:,0].min()),0]-=.04;model['corners_3d']=(q@axes).tolist()
    checked=validate_final_record(wrong,d,m,k)
    assert checked['approximate_cuboid']['status']=='REVOKED_FINAL_VALIDATION'
    assert 'TANGENTIAL' in checked['approximate_cuboid']['reason']
    assert 'corners_3d' not in checked['approximate_cuboid']
    ambiguous=deepcopy(out);ambiguous['source_ambiguous']=True
    checked=validate_final_record(ambiguous,d,m,k)
    assert checked['approximate_cuboid']['status']=='REVOKED_FINAL_VALIDATION'
    assert not checked['accepted'] and 'corners_3d' not in checked['approximate_cuboid']


def test_actual_planar_bevel_is_a_model_not_certified_corners():
    d,m,k,l,s,*_=rounded_box(chamfer=True)
    out=mf.fit_metric_faces(d,m,k,l,s,mask_id=23)
    model=out['approximate_cuboid']
    assert model['status']=='VALIDATED_OBSERVATION_MODEL'
    assert model['raw_boundary_fit']['residuals']['transition_pixel_count']>0
    assert model['physical_corners_certified'] is False
    assert any(not row['supported'] for row in model['sharp_corner_diagnostic'])


def test_two_actual_solids_merged_into_one_mask_are_not_one_approximate_box():
    options=dict(shape=(300,600),dimensions=(.42,.35,.3))
    a=rounded_box(**options,center=(-.38,-.01,2.));b=rounded_box(**options,center=(.38,-.01,2.))
    da,ma,k,la,sa,*_=a;db,mb,_,lb,sb,*_=b
    mask=ma|mb;depth=np.minimum(da,db);labels=la.copy();labels[mb&(db<da)]=np.where(lb[mb&(db<da)]>=0,lb[mb&(db<da)]+len(sa),-1)
    out=mf.fit_metric_faces(depth,mask,k,labels,sa+sb,mask_id=12)
    assert out.get('approximate_cuboid',{}).get('status')!='VALIDATED_OBSERVATION_MODEL'
    assert not out['accepted']
