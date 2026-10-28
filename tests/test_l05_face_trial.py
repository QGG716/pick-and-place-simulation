from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.l05_face_trial import TARGET,FACES,permission,option_for,validate_option
from unloading_sim.current_population_recipe import LAYOUT
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.low_top_receiver import LowTopReceiverConnector


def setup(face):
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells'/LAYOUT)
    boxes=layout.cartons();frozen=FrozenCartonPopulation.capture(boxes)
    target=next(b for b in boxes if b.name==TARGET)
    world={b.name:b for b in coupled_conveyor_state(layout,1.45,boxes).boxes}
    grant=permission(frozen,face)
    option,layer=option_for(target,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,1.45,grant)
    contact=SimpleNamespace(target_id=TARGET,box=target,geometric_face_id=FACES[face],
        world_y_offset_m=0.,eccentric_authority=None,contact_pose=np.asarray(option['face']['contact_pose']),
        face_role='TOP' if face=='top' else 'SIDE',outward_normal=np.asarray(option['face']['outward_normal']),validate=lambda:None)
    return target,frozen,world,grant,option,layer,contact


@pytest.mark.parametrize('face',list(FACES))
def test_one_frozen_centred_face_and_finite_height(face):
    target,frozen,world,grant,option,layer,contact=setup(face)
    validate_option(contact,option,grant,1.45)
    assert option['height_m']==option['pre_approach_height_m']==1.45
    assert option['original_role']=='MIDDLE'
    assert option['face']['geometric_face_id']==FACES[face]
    for height in (.25,1.15,1.4501):
        with pytest.raises(ValueError):validate_option(contact,option,grant,height)
    bad=deepcopy(grant);bad['target_id']='carton_l05_c04'
    with pytest.raises(ValueError):option_for(target,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,1.45,bad)
    bad=deepcopy(option);bad['segment_directions_world']=[[-1.,0.,0.]]
    with pytest.raises(ValueError):validate_option(contact,bad,grant,1.45)
    bad=deepcopy(contact);bad.contact_pose[0,3]+=.01
    with pytest.raises(ValueError):validate_option(bad,option,grant,1.45)


def test_top_uses_existing_checked_vertical_separation_contract():
    _,_,_,grant,option,layer,contact=setup('top')
    c=object.__new__(LowTopReceiverConnector)
    c.option=option;c.layer_top_permission=layer;c.l05_face_permission=grant;c.batch_root={'height_m':1.45}
    c.validate_template(contact)
    assert option['release_bottom_world_m']==1.55
    assert option['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]]
    c.l05_face_permission=None
    with pytest.raises(ValueError):c.validate_template(contact)


def test_terminal_side_requires_checked_raise_before_approach():
    from unloading_sim.l05_face_trial import validate_binding
    from unloading_sim.coupled_batch import coupled_binding
    target,frozen,world,_,_,_,contact=setup('side_positive_y')
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells'/LAYOUT)
    grant=permission(frozen,'side_positive_y',.25)
    option,_=option_for(target,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,.25,grant)
    validate_option(contact,option,grant,.25)
    binding=coupled_binding(layout,layout.cartons(),1.45,1.45,TARGET,[])
    with pytest.raises(ValueError):validate_binding(option,{'height_m':.25},binding)
    binding['pre_approach_transition']=coupled_binding(layout,layout.cartons(),.25,1.45,TARGET,[])
    validate_binding(option,{'height_m':.25},binding)
    binding['pre_approach_transition']['samples'][0]['height_m']=1.45
    with pytest.raises(ValueError):validate_binding(option,{'height_m':.25},binding)
    with pytest.raises(ValueError):permission(frozen,'top',.25)


def test_requested_negative_y_hmin_approaches_before_checked_raise():
    from unloading_sim.l05_face_trial import validate_binding
    from unloading_sim.coupled_batch import coupled_binding
    target,frozen,_,_,_,_,contact=setup('side_negative_y')
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells'/LAYOUT)
    world={b.name:b for b in coupled_conveyor_state(layout,1.15,layout.cartons()).boxes}
    grant=permission(frozen,'side_negative_y',.25,'Hmin')
    option,_=option_for(target,world['conveyor_transverse'],world['conveyor_longitudinal'],frozen,.25,grant)
    assert option['height_m']==1.15 and option['pre_approach_height_m']==.25
    assert option['segment_directions_world']==[[0.,-1.,0.],[-1.,0.,0.]]
    assert option['synchronization']=='PREGRASP_OR_CONTACT'
    validate_option(contact,option,grant,.25)
    binding=coupled_binding(layout,layout.cartons(),.25,1.15,TARGET,[])
    binding['pre_approach_transition']=coupled_binding(layout,layout.cartons(),.25,.25,TARGET,[])
    validate_binding(option,{'height_m':.25},binding)
    for field,value in [('height_m',1.45),('pre_approach_height_m',1.15),('synchronization','ALREADY_AT_RECEIVE'),
                        ('segment_directions_world',[[0.,1.,0.],[-1.,0.,0.]])]:
        bad=deepcopy(option);bad[field]=value
        with pytest.raises(ValueError):validate_option(contact,bad,grant,.25)
    binding['samples'][-1]['height_m']=1.45
    with pytest.raises(ValueError):validate_binding(option,{'height_m':.25},binding)
    for face in ('top','side_positive_y'):
        with pytest.raises(ValueError):permission(frozen,face,.25,'Hmin')
