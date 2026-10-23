"""Literal geometry, strict top permission and exact mounting-pair scope."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.conveyor_lift import coupled_conveyor_state,ScissorGeometryRangeError
from unloading_sim.coupled_lift_checks import check_coupled_state,assembly_mates
from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.isaac_collision_policy import classify_robot_mount_contact
from unloading_sim.low_top_receiver import uses_low_top,BOX_TOP_LOW_DEFINITION
from unloading_sim.carton_population import FrozenCartonPopulation
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints

ROOT=Path(__file__).resolve().parents[1]
PATH=ROOT/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile.yaml'

def test_literal_minimum_geometry_and_historical_profile_retained():
    layout=load_workcell_layout(PATH)
    assert audit_layout_constraints(layout)['overall_status']=='PASS'
    altered=deepcopy(layout.data);altered['robot']['base_origin_xy_a_m'][0]+=.01
    assert audit_layout_constraints(replace(layout,data=altered))['checks']['robot_base_origin_world']['status']=='FAIL'
    old=load_workcell_layout(PATH.with_name(PATH.name.replace('_lowprofile','')))
    new=coupled_conveyor_state(layout,.25)
    b={b.name:b for b in new.boxes}
    np.testing.assert_allclose(layout.robot_base_transform()[:3,3],[-1.825,.25,.6],atol=1e-12)
    assert b['conveyor_transverse'].center[2]+b['conveyor_transverse'].half_extents[2]==pytest.approx(.25)
    assert b['lift_transverse_left_end_pin_0_0'].center[2]==pytest.approx(.040)
    assert b['lift_transverse_left_end_pin_2_0'].center[2]==pytest.approx(.094)
    assert new.mechanism['stacks'][0]['stage_rise_m']==pytest.approx(.027)
    np.testing.assert_allclose(2*b['lift_transverse_left_arm_0_0'].half_extents,[1.25,.020,.020])
    np.testing.assert_allclose(2*b['lift_transverse_left_base_rail'].half_extents,[1.33,.080,.020])
    np.testing.assert_allclose(2*b['lift_transverse_left_foot_0'].half_extents,[.1,.1,.01])
    assert b['lift_longitudinal_near_end_pin_0_0'].center[2]==pytest.approx(.040)
    assert b['lift_lower_rail_0'].center[2]==pytest.approx(.340)
    assert 2*b['lift_lower_carriage_0'].half_extents[2]==pytest.approx(.150)
    assert new.mechanism['transverse_support_spacing_m']==.26
    assert 2*b['conveyor_longitudinal'].half_extents[1]==.6
    assert set(b)=={x.name for x in coupled_conveyor_state(old,.55).boxes}
    assert assembly_mates(new)==assembly_mates(coupled_conveyor_state(old,.55))
    with pytest.raises(ScissorGeometryRangeError):coupled_conveyor_state(old,.25)

@pytest.mark.parametrize('height',[.25,.26,.35,.55,.6,1.,1.65,2.1])
def test_complete_mechanism_keeps_all_other_pair_gates(height):
    check=check_coupled_state(load_workcell_layout(PATH),height,designed_contacts=[['chassis','conveyor_transverse']])
    assert check['passed'],check['failures']

def test_top_rule_strictly_uses_frozen_top():
    frozen=FrozenCartonPopulation.capture(load_workcell_layout(PATH).cartons())
    assert not any(uses_low_top(SimpleNamespace(z_b=h['z_b_m'],h=h['h_m']),.25,BOX_TOP_LOW_DEFINITION)
                   for h in frozen.evidence()['heights'].values())
    for top,allowed in ((.249999,True),(.25,False),(.250001,False),(.3,False),(.9,False)):
        assert uses_low_top(SimpleNamespace(z_b=0.,h=top),.25,BOX_TOP_LOW_DEFINITION)==allowed

def test_mount_rule_exact_and_backward_compatible():
    old=SimulationCollisionPolicy();new=SimulationCollisionPolicy(robot_mount_contact_exemption=True)
    assert 'robot_mount_contact_exemption' not in old.to_mapping()
    assert old.robot_mount_pairs()=={('base_link','chassis')}
    assert len(new.robot_mount_pairs())==7 and new.fingerprint!=old.fingerprint
    assert SimulationCollisionPolicy.from_mapping(new.to_mapping())==new
    for link in ('base_link','J1_link','J2_link','J3_link','J4_link','J5_link','J6_link'):
        assert new.allows_robot_mount_contact(link,'chassis')
    for link,other in (('cup_00','chassis'),('flange','chassis'),('tool','chassis'),('payload','chassis'),
            ('J2_link','trailer_floor'),('J2_link','conveyor_transverse'),('J2_link','chassis_clone'),('J2_link','J3_link')):
        assert not new.allows_robot_mount_contact(link,other)
    with pytest.raises(ValueError):new.robot_mount_pairs('conveyor_transverse')
    with pytest.raises(ValueError):SimulationCollisionPolicy(robot_mount_contact_exemption='true')
    kwargs=dict(robot_link_by_collider={'robot-shape':'J2_link'},world_object_by_collider={'mount-shape':'chassis'},policy=new)
    assert classify_robot_mount_contact('robot-shape','mount-shape',**kwargs)
    assert classify_robot_mount_contact('mount-shape','robot-shape',**kwargs)
    assert classify_robot_mount_contact('tool-shape','mount-shape',**kwargs) is None
    assert classify_robot_mount_contact('robot-shape','path-containing-chassis',**kwargs) is None
