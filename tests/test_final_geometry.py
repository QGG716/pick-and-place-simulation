import numpy as np
import pytest
from unloading_perception.final_geometry import coherent_cuboid,validate_final_record,polygon_pixels
from unloading_perception.upstream_v4 import prepare_metric_multiplane_input,prepare_registered_depth_baseline


def record():
    return {'corners_3d':[[-.3,-.3,2],[.3,-.3,2],[.3,.3,2],[-.3,.3,2],[-.3,-.3,2.5],[.3,-.3,2.5],[.3,.3,2.5],[-.3,.3,2.5]],'camera_facing_faces':[{'corner_indices':[0,1,2,3],'evidence':'registered_metric_depth_plane','axis_index':99}],'plane_residual_mean':0}


def test_final_plane_recomputed_not_inherited_or_indexed_by_axis():
    r=record(); K=[100,0,20,0,100,20,0,0,1]; mask=np.ones((40,40),bool)
    good=validate_final_record(r,np.full(mask.shape,2.001),mask,K)
    face=good['camera_facing_faces'][0]
    assert face['final_support']['plane_residual_m']==pytest.approx(.001)
    assert face['final_support']['point_support_count']>50
    bad=validate_final_record(r,np.full(mask.shape,2.1),mask,K)
    assert not bad['camera_facing_faces']
    assert bad['final_face_validation'][0]['reason']=='FINAL_METRIC_PLANE_RESIDUAL'


@pytest.mark.parametrize('damage',['nonfinite','shear','corner','zero'])
def test_cuboid_consistency(damage):
    p=np.array(record()['corners_3d']); coherent_cuboid(p)
    if damage=='nonfinite': p[0,0]=np.nan
    elif damage=='shear': p[:,0]+=.2*p[:,1]
    elif damage=='corner': p[6,0]+=.1
    else: p[1]=p[0]
    with pytest.raises(ValueError): coherent_cuboid(p)


def test_multiplane_version_selects_all_fields_and_requires_provenance():
    r=record(); r['corners_2d']=[[0,0]]*8
    for k in ('corners_3d','corners_2d','camera_facing_faces'): r['unanchored_'+k]=r[k]
    r['corners_3d']=[[9,9,9]]*8
    out=prepare_metric_multiplane_input({'instances':[r]},verified_metric_source=True)['instances'][0]
    assert out['corners_3d']==record()['corners_3d']
    assert out['shape_dimensions']==pytest.approx([.6,.6,.5])
    with pytest.raises(ValueError): prepare_metric_multiplane_input({'instances':[r]},verified_metric_source=False)
    legacy={'instances':[{'camera_facing_faces':[{'evidence':'monocular_depth_plane'}]}]}
    assert 'adapter_source_evidence' not in prepare_registered_depth_baseline(legacy)['instances'][0]['camera_facing_faces'][0]


def test_crossed_or_diagonal_face_topology_is_rejected():
    with pytest.raises(ValueError,match='NONCONVEX'):
        polygon_pixels([[0,0],[10,10],[10,0],[0,10]],(20,20))
    r=record(); r['camera_facing_faces'][0]['corner_indices']=[0,1,6,7]
    checked=validate_final_record(r,np.full((40,40),2.),np.ones((40,40),bool),[100,0,20,0,100,20,0,0,1])
    assert not checked['camera_facing_faces']
    assert checked['final_face_validation'][0]['reason']=='INVALID_CUBOID_FACE_TOPOLOGY'
