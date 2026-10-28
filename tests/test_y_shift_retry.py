from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.y_shift_retry import BASE_LAYOUT,LAYOUT,mount_evidence
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.moveit2_backend import box_message


def layouts():
    root=Path(__file__).resolve().parents[1]/'configs/workcells'
    return load_workcell_layout(root/BASE_LAYOUT),load_workcell_layout(root/LAYOUT)


def test_exact_new_mount_and_frozen_equipment():
    old,new=layouts();e=mount_evidence(old,new)
    np.testing.assert_allclose(new.robot_base_transform()[:3,3],[-1.625,.15,.60],rtol=0,atol=1e-12)
    assert new.data['robot']['base_origin_xy_a_m']==[.325,-.2]
    assert new.data['tool']['connector_extension_m']==.2
    assert new.data['assembly']['world_xyz_m']==[-1.95,.35,0.]
    assert new.data['conveyors']['coupling']['minimum_surface_height_world_m']==.25
    assert audit_layout_constraints(new)['overall_status']=='PASS'
    assert e['translation_world_m']==[0.,-.1,0.] and e['fixed_equipment_and_cartons_unchanged']
    assert len(new.cartons())==40
    for height in (.25,1.15,1.45):
        assert [box_message(b) for b in coupled_conveyor_state(old,height,old.cartons()).boxes]==[
            box_message(b) for b in coupled_conveyor_state(new,height,new.cartons()).boxes]


@pytest.mark.parametrize('change',['x','belt'])
def test_mount_change_rejects_unapproved_extra_geometry(change):
    old,new=layouts();data=deepcopy(new.data)
    if change=='x':data['robot']['base_origin_xy_a_m'][0]+=.1
    else:data['conveyors']['transverse']['size_xy_m'][0]+=.1
    with pytest.raises(ValueError):mount_evidence(old,replace(new,data=data))
