from copy import deepcopy
import numpy as np
import pytest

from unloading_perception.metric_faces import _binding, _support_runs
from unloading_perception.final_geometry import validate_final_record
from unloading_perception.observed_faces import observed_faces_from_geometry_record
from unloading_perception.rgbd import hypotheses_from_geometry_record, MetricPointMapSource


def fixture():
    depth=np.full((80,100),2.); mask=np.zeros(depth.shape,bool); mask[10:70,10:90]=True
    K=np.array([[100.,0,49.5],[0,140.,39.5],[0,0,1.]])
    polygon=np.array([[15,15],[85,15],[85,65],[15,65]],float)
    from unloading_perception.metric_support import backproject_pixels
    points=backproject_pixels(polygon,np.full(4,2.),K)
    record={'mask_id':1,'accepted':False,'geometry_version':'DEPTH_METRIC_PATCHES_V1',
        'complete_observability':'UNRESOLVED_PHYSICAL_BOUNDARIES',
        'support_capture_binding':_binding(depth,mask,K),'frozen_support_regions':{'0':_support_runs(mask)},
        'camera_facing_faces':[{'corner_indices':[],'corners_3d_m':points.tolist(),'corners_2d':polygon.tolist(),
                                'evidence':'registered_metric_depth_plane','support_label':0}]}
    return record,depth,mask,K


def test_direct_patch_survives_without_inventing_eight_corners_or_thickness():
    record,depth,mask,K=fixture(); checked=validate_final_record(record,depth,mask,K)
    assert len(checked['camera_facing_faces'])==1
    faces=observed_faces_from_geometry_record(checked,source_instance_id='source',module_id='lower',capture_id='capture',capture_time=3.,frame_id='camera')
    assert len(faces.faces)==1 and not faces.faces[0].corner_indices
    assert faces.complete_cuboid_status=='INSUFFICIENT_SINGLE_FACE_WITHOUT_SIZE_PRIOR'
    assert hypotheses_from_geometry_record(checked,source_instance_id='source',pointmap_source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH,presence_score=.8)==()


def test_moving_final_patch_does_not_move_observation_support():
    record,depth,mask,K=fixture(); bad=deepcopy(record)
    bad['camera_facing_faces'][0]['corners_3d_m']=(np.asarray(bad['camera_facing_faces'][0]['corners_3d_m'])+[0,0,.02]).tolist()
    out=validate_final_record(bad,depth,mask,K)
    assert not out['camera_facing_faces']
    assert out['final_face_validation'][0]['reason']=='FINAL_METRIC_PATCH_RESIDUAL'
    assert out['frozen_support_regions']==record['frozen_support_regions']


def test_support_is_bound_to_exact_instance_depth_and_calibration():
    record,depth,mask,K=fixture()
    with pytest.raises(ValueError,match='BINDING'):
        validate_final_record(record,depth+.1,mask,K)
    altered=K.copy(); altered[0,0]+=1
    with pytest.raises(ValueError,match='BINDING'):
        validate_final_record(record,depth,mask,altered)
