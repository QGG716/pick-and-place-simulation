from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
import yaml
from unloading_sim.joint_search_layout import (BASELINE,LayoutCandidate,candidate_data,support_domain,
    mounting_geometry,qualified_mount_pairs)
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints,compute_layout_fingerprint
from unloading_sim.conveyor_lift import coupled_conveyor_state

ROOT=Path(__file__).resolve().parents[1]

def layout(c):
    old=load_workcell_layout(ROOT/'configs/workcells'/BASELINE)
    d=candidate_data(old.data,c)
    return replace(old,data=d,layout_fingerprint=compute_layout_fingerprint(d,old.assets))

def test_full_support_domain_and_frozen_identity():
    old=load_workcell_layout(ROOT/'configs/workcells'/BASELINE)
    np.testing.assert_allclose(support_domain(old.data),[[-.7115,.825],[-.475,.475]],atol=1e-12)
    moved=layout(LayoutCandidate(.625,-.35,.4))
    np.testing.assert_allclose(moved.robot_base_transform()[:3,3],[-1.325,0.,.4],atol=1e-12)
    assert audit_layout_constraints(moved)['overall_status']=='PASS'
    assert all(np.array_equal(a.world_from_local,b.world_from_local) for a,b in zip(old.cartons(),moved.cartons()))
    assert moved.data['tool']==old.data['tool'] and moved.data['trailer']==old.data['trailer']

def test_lowering_has_explicit_recess_with_retained_side_solids():
    moved=layout(LayoutCandidate(.325,-.2,.4));fixed={b.name:b for b in moved.fixed_components()}
    assert fixed['chassis'].half_extents[2]==.2
    assert len([n for n in fixed if n.startswith('mount_recess_')])==4
    for n,b in fixed.items():
        if n.startswith('mount_recess_'):
            assert b.category=='mount_structure'
            assert abs(b.center[2]+b.half_extents[2]-.6)<1e-12
            assert abs(b.center[2]-b.half_extents[2]-.4)<1e-12
    assert audit_layout_constraints(moved)['overall_status']=='PASS'

def test_raised_mount_only_qualifies_actual_base_support_plane():
    moved=layout(LayoutCandidate(.325,-.2,.8));robot=moved.robot()
    assert qualified_mount_pairs(robot,moved.fixed_components())=={('base_link','mount_plinth')}
    parts=list(moved.fixed_components());i=next(i for i,b in enumerate(parts) if b.name=='mount_plinth')
    parts[i]=replace(parts[i],center=parts[i].center+[0,0,.001])
    with pytest.raises(ValueError,match='GEOMETRY_CHANGED'):qualified_mount_pairs(robot,parts)

def test_height_and_geometry_are_distinct_and_frozen():
    with pytest.raises(ValueError):layout(LayoutCandidate(.325,-.2,.6,.20))
    moved=layout(LayoutCandidate(.325,-.2,.6,.20,profile='low_profile_200_v1'))
    s=coupled_conveyor_state(moved,.20,())
    belt=next(b for b in s.boxes if b.name=='conveyor_transverse')
    assert abs(2*belt.half_extents[2]-.12)<1e-12
    assert s.mechanism['low_profile_dimensions_m']['transverse_arm_depth']==.012
    with pytest.raises(ValueError):coupled_conveyor_state(moved,.199,())
    bad=deepcopy(moved.data);bad['tool']['connector_extension_m']=.3
    with pytest.raises(ValueError,match='FROZEN_FIELDS'):audit_layout_constraints(replace(moved,data=bad))


def test_numpy_refinement_is_yaml_serializable_without_value_changes():
    c=LayoutCandidate(*np.array([.35,-.2,.6]))
    d=candidate_data(load_workcell_layout(ROOT/'configs/workcells'/BASELINE).data,c)
    assert yaml.safe_load(yaml.safe_dump(d))==d
    assert c.evidence()['base_xyz_a_m']==[.35,-.2,.6]


def test_retained_exterior_chassis_seam_is_not_a_robot_exemption():
    from unloading_sim.joint_search_layout import retained_chassis_contacts
    from unloading_sim.coupled_lift_checks import check_coupled_state
    moved=layout(LayoutCandidate(.625,-.25,.5))
    contacts=[['chassis','conveyor_transverse'],['chassis','conveyor_longitudinal'],
              ['conveyor_transverse','conveyor_longitudinal']]
    inherited=retained_chassis_contacts(moved,contacts)
    assert ['mount_recess_xplus','conveyor_transverse'] in inherited
    assert all('robot' not in str(p) and 'tool' not in str(p) for p in inherited)
    assert not qualified_mount_pairs(moved.robot(),moved.fixed_components())
    assert check_coupled_state(moved,.6,(),designed_contacts=inherited)['passed']


def test_low200_v2_preserves_belt_and_connects_mount_with_static_clearance():
    from unloading_sim.coupled_lift_checks import check_coupled_state
    moved=layout(LayoutCandidate(.325,-.2,.6,.2,profile='low_profile_200_v2'))
    contacts=[['chassis','conveyor_transverse'],['chassis','conveyor_longitudinal'],
              ['conveyor_transverse','conveyor_longitudinal']]
    for height in (.2,.25,.6,1.15,2.1):
        checked=check_coupled_state(moved,height,(),designed_contacts=contacts)
        assert checked['passed'],[(x['pair'],x['surface_distance_m']) for x in checked['failures']]
        world={b.name:b for b in coupled_conveyor_state(moved,height,()).boxes}
        rail=world['lift_transverse_left_head_rail'];mount=world['lift_transverse_left_mount']
        assert rail.signed_distance_obb(mount)<=0
        assert abs(world['conveyor_transverse'].half_extents[2]*2-.12)<1e-12
        assert abs(mount.half_extents[2]*2-.016)<1e-12


def test_outfeed20_support_is_lowered_at_full_size_and_still_stands_on_floor():
    from unloading_sim.coupled_lift_checks import check_coupled_state
    moved=layout(LayoutCandidate(0.,-.2,.9,.2,profile='low_profile_200_outfeed20_v3'))
    original=layout(LayoutCandidate(0.,-.2,.9,.2,profile='low_profile_200_v2'))
    contacts=[['chassis','conveyor_transverse'],['chassis','conveyor_longitudinal'],
              ['conveyor_transverse','conveyor_longitudinal']]
    assert audit_layout_constraints(moved)['overall_status']=='PASS'
    for height in (.2,.25,.55,.6,.85,2.1):
        checked=check_coupled_state(moved,height,(),designed_contacts=contacts)
        assert checked['passed'],[(x['pair'],x['surface_distance_m']) for x in checked['failures']]
        world={b.name:b for b in coupled_conveyor_state(moved,height,()).boxes}
        old={b.name:b for b in coupled_conveyor_state(original,height,()).boxes}
        np.testing.assert_allclose(world['lift_lower_bearing'].center[2],.530,atol=1e-12)
        np.testing.assert_allclose(2*world['lift_lower_bearing'].half_extents,[.20,.56,.10],atol=1e-12)
        np.testing.assert_allclose(world['lift_lower_carriage_0'].center[2],.405,atol=1e-12)
        np.testing.assert_allclose(2*world['lift_lower_carriage_0'].half_extents,[.16,.10,.15],atol=1e-12)
        np.testing.assert_allclose(world['lift_lower_rail_0'].center[2],.320,atol=1e-12)
        stand=world['lift_lower_rail_0_stand_0']
        np.testing.assert_allclose([stand.center[2],2*stand.half_extents[2]],[.150,.300],atol=1e-12)
        for name in ('lift_lower_bearing','lift_lower_carriage_0','lift_lower_carriage_2',
                     'lift_lower_rail_0','lift_lower_rail_2'):
            np.testing.assert_array_equal(world[name].half_extents,old[name].half_extents)
            np.testing.assert_allclose(world[name].center-old[name].center,[0,0,-.020],atol=1e-12)
        for name in ('conveyor_transverse','conveyor_longitudinal'):
            np.testing.assert_array_equal(world[name].world_from_local,old[name].world_from_local)
            np.testing.assert_array_equal(world[name].half_extents,old[name].half_extents)
