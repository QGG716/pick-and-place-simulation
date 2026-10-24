from copy import deepcopy
from types import SimpleNamespace
import pytest
from test_l01_selected_routes import setup
from unloading_sim.l01_hmax import layer_permission,c04_face_permission,options,validate_option


@pytest.mark.parametrize('face,face_id,legs',[
    ('top','z+',[[0.,0.,1.],[-1.,0.,0.]]),
    ('side','y-',[[0.,-1.,0.],[-1.,0.,0.]]),
    ('front','x-',[[-1.,0.,0.]])])
def test_three_separate_c04_faces_keep_hmax_and_exact_routes(face,face_id,legs):
    boxes,frozen,world,height=setup();p=layer_permission(frozen);grant=c04_face_permission(frozen,face)
    b=boxes['carton_l01_c04']
    opts=options(b,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p,c04_face=grant)
    assert len(opts)==1
    o=opts[0]
    assert o['face']['geometric_face_id']==face_id and o['segment_directions_world']==legs
    assert o['height_m']==o['pre_approach_height_m']==pytest.approx(.25)
    assert grant['tool_connector_extension_m']==.2 and grant['dispatch_order']==['carton_l01_c04']
    t=SimpleNamespace(box=b,target_id=b.name,geometric_face_id=face_id,world_y_offset_m=0,eccentric_authority=None)
    validate_option(t,o,p,height,c04_face=grant)
    with pytest.raises(ValueError):validate_option(t,o,p,height)
    bad=deepcopy(o);bad['segment_directions_world']=[[0.,1.,0.],[-1.,0.,0.]]
    with pytest.raises(ValueError):validate_option(t,bad,p,height,c04_face=grant)
    for name in ('carton_l01_c01','carton_l00_c04'):
        with pytest.raises(ValueError):options(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p,c04_face=grant)


def test_wrong_face_or_modified_frozen_identity_does_not_authorize():
    boxes,frozen,world,h=setup();p=layer_permission(frozen)
    with pytest.raises(ValueError):c04_face_permission(frozen,'y+')
    grant=c04_face_permission(frozen,'side');grant['start_box_identity']='changed'
    with pytest.raises(ValueError):options(boxes['carton_l01_c04'],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,h,p,c04_face=grant)
