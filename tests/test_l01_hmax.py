from copy import deepcopy
from types import SimpleNamespace
import pytest
from test_l01_selected_routes import setup
from unloading_sim.l01_hmax import layer_permission, options, validate_option, validate_height_permission, ORDER
from unloading_sim.junction_reception import validate_junction_release
from unloading_sim.lifted_reception import translated


def test_frozen_five_box_grant_three_ordered_face_trials():
    boxes,frozen,world,height=setup();p=layer_permission(frozen)
    assert p['dispatch_order']==list(ORDER)
    assert p['allowed_target_ids']==['carton_l01_c00','carton_l01_c01','carton_l01_c02','carton_l01_c03','carton_l01_c04']
    expected=[['x-'],['x-','y+'],['x-','z+']]
    for name,faces in zip(ORDER,expected):
        opts=options(boxes[name],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p)
        assert [o['face']['geometric_face_id'] for o in opts]==faces
        for o in opts:
            assert o['height_m']==pytest.approx(.25) and o['height_m']==o['pre_approach_height_m']
            face=o['face']['geometric_face_id']
            assert o['segment_directions_world']==([[-1.,0.,0.]] if face=='x-' else
                [[0.,1.,0.],[-1.,0.,0.]] if face=='y+' else [[0.,0.,1.],[-1.,0.,0.]])
            target=SimpleNamespace(target_id=name,box=boxes[name],geometric_face_id=face,world_y_offset_m=0.,eccentric_authority=None)
            validate_option(target,o,p,height)
            bad=deepcopy(o);bad['height_m']+=.01
            with pytest.raises(ValueError):validate_option(target,bad,p,height)
        assert opts[0]['reception']['junction']==(name!='carton_l01_c04')
    for name in p['allowed_target_ids']:validate_height_permission(boxes[name],p,height)
    for name in ('carton_l00_c01','carton_l02_c01'):
        with pytest.raises(ValueError):validate_height_permission(boxes[name],p,height)
    with pytest.raises(ValueError):options(boxes['carton_l01_c02'],world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,height,p)


def test_junction_hmax_explicit_scope_retains_actual_y_and_mechanism_height():
    boxes,frozen,world,height=setup();p=layer_permission(frozen);original=boxes[ORDER[0]]
    released=translated(original,[world['conveyor_transverse'].center[0]-original.center[0],0,0])
    with pytest.raises(ValueError,match='JUNCTION_FRONT_REQUIRES_HMIN'):
        validate_junction_release(released,original,world.values(),[1,0,0],height)
    check=validate_junction_release(released,original,world.values(),[1,0,0],height,l01_hmax_permission=p)
    assert check['passed'] and check['junction'] and check['receive_rule']=='USER_FROZEN_L01_HMAX'
    with pytest.raises(ValueError,match='Y_OR_ORIENTATION'):
        validate_junction_release(translated(released,[0,.01,0]),original,world.values(),[1,0,0],height,l01_hmax_permission=p)
    with pytest.raises(ValueError,match='REQUIRED_RECEIVE_HEIGHT'):
        validate_junction_release(released,original,world.values(),[1,0,0],.3,l01_hmax_permission=p)
    changed=translated(original,[0,.001,0])
    with pytest.raises(ValueError):validate_height_permission(changed,p,height)
    bad=deepcopy(p);bad['allowed_target_ids'].append('carton_l00_c01')
    with pytest.raises(ValueError):validate_height_permission(original,bad,height)
    bad=deepcopy(p);bad['original_cartons'].pop()
    with pytest.raises(ValueError):validate_height_permission(original,bad,height)
