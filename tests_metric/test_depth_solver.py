"""Optional CPU metric solver regression; no GPU, ROS or upstream checkout."""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation

from unloading_perception.metric_faces import fit_metric_faces,MetricFitConfig,_boundary_kinds
from unloading_perception.final_geometry import validate_final_record
from tools.validate_metric_analytic import render


def analytic_box_fixture():
    K=np.array([[700.,0,249.5],[0,800.,249.5],[0,0,1.]])
    T=np.eye(4); T[:3,:3]=Rotation.from_euler('xyz',[15,25,10],degrees=True).as_matrix(); T[:3,3]=[.1,.1,2.5]
    dims=np.array([.6,.4,.3]); box={'T_W_object':T.tolist(),'full_dimensions_m':dims.tolist()}
    depth,identity=render([box],K,(500,500)); mask=identity==0
    # These oracle labels belong only to the explicit analytic fixture. The
    # production extractor is tested separately on actual fixed RGB-D inputs.
    from unloading_perception.metric_support import backproject_pixels
    y,x=np.nonzero(mask); points=backproject_pixels(np.c_[x,y],depth[mask],K)
    local=(points-T[:3,3])@T[:3,:3]
    axis=np.argmin(np.abs(np.abs(local)-dims/2),axis=1)
    labels=np.full(mask.shape,-1,np.int16); labels[y,x]=axis
    seeds=[{'normal':T[:3,i].tolist(),'offset_m':0.,'initial_plane_index':i} for i in range(3)]
    return depth, mask, K, labels, seeds, T


def test_analytic_three_faces_joint_so3_and_independent_rejection():
    depth, mask, K, labels, seeds, T = analytic_box_fixture()
    result=fit_metric_faces(depth,mask,K,labels,seeds,mask_id=1,config=MetricFitConfig(positive_infinity_is_no_hit=True))
    assert len(result['camera_facing_faces'])==3
    assert result['accepted']
    assert abs(np.linalg.det(result['orthogonal_axes_3d'])-1)<1e-9
    assert np.linalg.norm(np.asarray(result['center_3d'])-T[:3,3])<.015
    bad=deepcopy(result)
    bad['camera_facing_faces'][0]['corners_3d_m']=(np.asarray(bad['camera_facing_faces'][0]['corners_3d_m'])+[0,0,.03]).tolist()
    checked=validate_final_record(bad,depth,mask,K)
    assert not checked['accepted']
    assert len(checked['camera_facing_faces'])==2
    moved=deepcopy(result)
    moved['corners_3d']=(np.asarray(moved['corners_3d'])+[0,0,.03]).tolist()
    checked=validate_final_record(moved,depth,mask,K)
    assert not checked['accepted']
    assert checked['complete_cuboid_consistency']['reason']=='COMPLETE_CUBOID_MOVED_FROM_OBSERVED_PLANES'


def test_boundary_semantics_require_explicit_no_hit_and_keep_occlusion():
    K=[100,0,39.5,0,120,39.5,0,0,1]
    polygon=np.array([[20.,20.],[60.,20.],[60.,60.],[20.,60.]])
    depth=np.full((80,80),np.inf)
    assert all(b['kind']=='UNCLASSIFIED_BOUNDARY' for b in _boundary_kinds(polygon,np.array([0,0,1]),-2,depth,K))
    assert all(b['kind']=='PHYSICAL_EDGE_SUPPORTED' for b in _boundary_kinds(polygon,np.array([0,0,1]),-2,depth,K,positive_infinity_is_no_hit=True))
    depth[:]=1.
    assert all(b['kind']=='OCCLUSION_BOUNDARY' for b in _boundary_kinds(polygon,np.array([0,0,1]),-2,depth,K))
    cropped=polygon-[22,22]
    assert any(b['kind']=='IMAGE_CROP_BOUNDARY' for b in _boundary_kinds(cropped,np.array([0,0,1]),-2,depth,K))
