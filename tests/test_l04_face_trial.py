from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest
from test_assumed_removal import fixture
from unloading_sim.assumed_removal import assume_l05_removed
from unloading_sim.l04_face_trial import TARGET,FACES,permission,option_for,validate_option,validate_binding
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_batch import coupled_binding
from unloading_sim.low_top_receiver import LowTopReceiverConnector


@pytest.mark.parametrize('face',list(FACES))
def test_requested_face_has_one_pose_and_checked_hmax_sync(face):
    layout,_,state=fixture();assume_l05_removed(state)
    target=next(b for b in state.cartons if b.name==TARGET)
    world={b.name:b for b in coupled_conveyor_state(layout,1.15,state.cartons).boxes}
    grant=permission(state.frozen,face)
    option,layer=option_for(target,world['conveyor_transverse'],world['conveyor_longitudinal'],state.frozen,.25,grant)
    contact=SimpleNamespace(target_id=TARGET,box=target,geometric_face_id=FACES[face],world_y_offset_m=0.,
        eccentric_authority=None,contact_pose=np.asarray(option['face']['contact_pose']),
        face_role='TOP' if face=='top' else 'SIDE',outward_normal=np.asarray(option['face']['outward_normal']),validate=lambda:None)
    validate_option(contact,option,grant,.25)
    assert option['height_m']==1.15 and option['pre_approach_height_m']==.25
    if face=='top':
        assert option['release_bottom_world_m']==1.25 and option['synchronization']=='PREGRASP'
        assert option['segment_directions_world']==[[0.,0.,1.],[-1.,0.,0.]]
        c=object.__new__(LowTopReceiverConnector)
        c.option=option;c.layer_top_permission=layer;c.l04_face_permission=grant;c.batch_root=state.root()
        c.validate_template(contact)
    else:
        assert option['synchronization']=='PREGRASP_OR_CONTACT'
        assert option['segment_directions_world']==[[0.,1. if face=='side_positive_y' else -1.,0.],[-1.,0.,0.]]
    for field,value in [('height_m',.25),('pre_approach_height_m',1.15),('stationary_receiver_during_target',True)]:
        bad=deepcopy(option);bad[field]=value
        with pytest.raises(ValueError):validate_option(contact,bad,grant,.25)
    binding=coupled_binding(layout,state.cartons,.25,1.15,TARGET,[])
    binding['pre_approach_transition']=coupled_binding(layout,state.cartons,.25,.25,TARGET,[])
    validate_binding(option,state.root(),binding)
    wrong=state.root();wrong['assumed_removed']=[]
    with pytest.raises(ValueError):validate_binding(option,wrong,binding)
    binding['samples'][-1]['height_m']=.25
    with pytest.raises(ValueError):validate_binding(option,state.root(),binding)
