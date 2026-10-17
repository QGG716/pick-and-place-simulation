"""Geometry-only regressions; no worker, IK, trajectory or physical execution."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest

from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.lifted_reception import (StartHeight, LiftedConveyorScene, layer_role,
    directional_contact, receiving_region, drop_geometry, process_scheme, bounds,
    translated, side_turn_regions, DEFERRED)

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def layout():
    return load_workcell_layout(ROOT/"configs/workcells/m710id70_unloading_layout_v1.yaml")


def box(center=(0,0,2.25),half=(.3,.2,.15),name="target",rotation=None):
    return OBB(center,half,np.eye(3) if rotation is None else rotation,name,"carton")


def test_literal_heights_and_frozen_snapshot():
    b=box()
    h=StartHeight.capture(b)
    assert (h.z_b,h.h,h.hmax,h.hmin)==pytest.approx((2.10,.30,2.05,1.75))
    b.center[2]=17
    assert h.hmax==pytest.approx(2.05)
    assert h.evidence()["low_drop_m"]==pytest.approx(.35)
    assert h.evidence()["low_first_touch_top_world_m"]==pytest.approx(2.05)


def test_nondefault_and_rotated_height():
    assert StartHeight.capture(box(half=(.3,.2,.225))).h==pytest.approx(.45)
    b=box(rotation=rotation_matrix_from_rpy(0,np.pi/2,0))
    h=StartHeight.capture(b)
    assert h.h==pytest.approx(.6)
    assert h.z_b==pytest.approx(1.95)


def test_transverse_only_and_same_native_render_source(layout):
    old=LiftedConveyorScene.build(layout)
    new=LiftedConveyorScene.build(layout,surface_z_world_m=2.05)
    assert len(old.boxes)==48 and len(new.boxes)==48
    assert old.scene_fingerprint!=new.scene_fingerprint
    assert new.height["body_center_z_world_m"]==pytest.approx(1.99)
    assert new.height["body_thickness_m"]==pytest.approx(.12)
    assert new.height["lift_displacement_m"]==pytest.approx(1.45)
    for original,actual in zip(layout.fixed_components()+layout.cartons(),old.boxes):
        np.testing.assert_array_equal(original.world_from_local,actual.world_from_local)
    for a,b in zip(old.boxes,new.boxes):
        if a.name!="conveyor_transverse":
            np.testing.assert_array_equal(a.world_from_local,b.world_from_local)
        else:
            np.testing.assert_array_equal(a.center[:2],b.center[:2])
            np.testing.assert_array_equal(a.rotation,b.rotation)
    world=new.evidence()["world"]
    belt=next(b for b in world if b["id"]=="conveyor_transverse")
    assert belt["pose"][2][3]+belt["size"][2]/2==pytest.approx(2.05)
    assert new.evidence()["render_geometry_source"]=="world"
    assert sum(b["id"]==belt["id"] for b in world)==1


def test_assembly_world_offset_respected(layout):
    data=deepcopy(layout.data)
    data["assembly"]["world_xyz_m"][2]=.4
    lifted=replace(layout,data=data)
    scene=LiftedConveyorScene.build(lifted,surface_z_world_m=2.05)
    assert scene.height["original_surface_z_world_m"]==pytest.approx(1.)
    assert scene.height["lift_displacement_m"]==pytest.approx(1.05)
    assert scene.belt.center[2]==pytest.approx(1.99)
    assert next(b for b in scene.boxes if b.name=="conveyor_longitudinal").center[2]==pytest.approx(.94)


def test_unknown_overtravel_and_negative_height_not_clamped(layout):
    unknown=LiftedConveyorScene.build(layout,surface_z_world_m=-.05)
    assert unknown.height["mechanical_travel_status"]=="UNKNOWN"
    assert unknown.height["surface_z_world_m"]==-.05
    assert "BELT_BODY_BELOW_FLOOR" in unknown.height["issues"]
    limited=LiftedConveyorScene.build(layout,surface_z_world_m=2.05,mechanical_surface_limits_m=[.2,1.])
    assert "KNOWN_MECHANICAL_OVERTRAVEL" in limited.height["issues"]
    assert limited.height["surface_z_world_m"]==2.05 and not limited.height["clamped"]


def test_roles_from_geometry_not_names_and_removal_provenance():
    row=[box((.3,y,2.25),name=n) for y,n in [(-.84,"left_label"),(0,"edge_label"),(.84,"right_label")]]
    assert layer_role(row[0],row,[row[0]])["role"]=="RIGHT_END"
    assert layer_role(row[-1],row,[row[-1]])["role"]=="LEFT_END"
    assert layer_role(row[1],row,[row[1]])["role"]=="MIDDLE"
    assert layer_role(row[1],[row[1]],[row[1]])["role"]=="SINGLE_ORIGINAL"


@pytest.mark.parametrize("normal,face",[([0,-1,0],"y-"),([0,1,0],"y+"),([-1,0,0],"x-")])
def test_physical_face_and_inward_axis(normal,face):
    contact=directional_contact(box(),normal)
    assert contact["geometric_face_id"]==face
    np.testing.assert_allclose(np.array(contact["contact_pose"])[:3,2],-np.array(normal))


def test_rotated_face_resolved_or_inapplicable():
    b=box(rotation=rotation_matrix_from_rpy(0,0,np.pi/2))
    assert directional_contact(b,[-1,0,0])["geometric_face_id"]=="y+"
    assert directional_contact(box(rotation=rotation_matrix_from_rpy(0,0,.3)),[-1,0,0]) is None


def test_complete_projection_and_occupancy(layout):
    belt=LiftedConveyorScene.build(layout,surface_z_world_m=2.05).belt
    r=receiving_region(box(),belt)
    np.testing.assert_allclose(r["rectangles_belt_local_m"],[[-.04,.04,-.54,.54]],atol=1e-12)
    assert not receiving_region(box(half=(.36,.2,.15)),belt)["exists"]
    # Center fits; a rotated long corner does not fit the 0.70m belt depth.
    assert not receiving_region(box(half=(.3,.3,.15),rotation=rotation_matrix_from_rpy(0,0,np.pi/4)),belt)["exists"]
    occupant=box((-.55,.35,2.2),(.34,.74,.15),"occupied")
    blocked=receiving_region(box(),belt,[occupant])
    assert not blocked["exists"] and blocked["occupied_exclusions"][0]["id"]=="occupied"


def test_front_region_and_old_distance_are_distinct(layout):
    cartons=layout.cartons()
    target=next(b for b in cartons if b.name=="carton_l07_c02")
    result=process_scheme(layout,target,cartons,cartons,"middle_front")
    assert result["template_region_world_rectangles"][0][:2]==pytest.approx([-.59,-.51])
    assert result["display_extraction_length_m"]==pytest.approx(.85)
    assert result["stack_only_normal_separation_m"]<.7
    assert result["old_planner_bound"]["sample_exceeds_old_bound"]
    assert result["display_sample"]["marker"]=="DISPLAY_SAMPLE_NOT_UNIQUE_TARGET"
    outside=next(b for b in cartons if b.name=="carton_l07_c00")
    # A front-only world-X template cannot fix an out-of-range Y by teleporting.
    original=[box((b.center[0],-b.center[1],b.center[2]),b.half_extents,b.name) for b in cartons]
    left=next(b for b in original if b.name==outside.name)
    shifted=translated(left,[0,1.,0])
    orig=tuple(shifted if b.name==left.name else b for b in original)
    r=process_scheme(layout,shifted,orig,orig,"left_front")
    assert r["receiving_region"]["exists"] and r["applicability"]=="TEMPLATE_NOT_APPLICABLE"
    assert "NOT_CARTON_UNLOADABILITY" in r["rejection_scope"]


def test_blocked_channel_and_dynamic_turn_interval():
    target=box((.3,.84,2.25))
    region=[[-.59,-.51,-.19,.83]]
    clear,_=side_turn_regions(region,target,[],[], -1)
    assert clear==region
    blocker=box((.3,.42,2.25),name="neighbor")
    blocked,evidence=side_turn_regions(region,target,[],[blocker],-1)
    assert blocked and blocked[0][2]>=.825-1e-10
    assert evidence[0]["pair"]==["target","neighbor"]
    # A small lateral shift does not clear the full subsequent X sweep of tool.
    tool=box((.3,.48,2.25),(.3,.15,.12),"rigid_tool")
    blocked,evidence=side_turn_regions(region,target,[tool],[blocker],-1)
    assert not blocked and evidence


def test_hmin_drop_tool_static_and_snag_retained(layout):
    belt=LiftedConveyorScene.build(layout,surface_z_world_m=1.75).belt
    release=box((-.55,.35,2.25))
    tool=box((-.55,.7,2.25),(.1,.1,.12),"tool")
    drop=drop_geometry(release,[tool],belt,[])
    assert drop["drop_m"]==pytest.approx(.35)
    assert drop["first_touch_top_world_m"]==pytest.approx(2.05)
    assert drop["tool_motion_world_m"]==[0,0,0]
    assert tool.center[2]==2.25
    assert drop["full_bottom_support"]["supported"]
    snag=box((-.55,.35,1.99),(.05,.05,.01),"snag")
    obstructed=drop_geometry(release,[tool],belt,[snag])
    assert obstructed["status"]=="POSSIBLE_INTERFERENCE"
    assert ["target","snag"] in [i["pair"] for i in obstructed["screen"]["issues"]]


def test_initial_support_is_not_blanket_neighbor_exemption():
    from unloading_sim.lifted_reception import screen_sweep
    target=box((.3,0,2.25))
    support=box((.3,0,1.95),name="support")
    neighbor=box((-.4,0,2.25),name="blocker")
    r=screen_sweep([target],[-.8,0,0],[support,neighbor],support_ids=["support"],target_id="target")
    assert r["named_initial_contacts"] and r["issues"][0]["pair"]==["target","blocker"]
    # Final and drop phases do not inherit the intermediate support permission.
    r=screen_sweep([target],[0,0,-.1],[support],target_id="target")
    assert r["issues"]


def test_deferred_is_neither_rejection_nor_pass_and_old_policy_unchanged():
    from unloading_sim.release_motion import ReleasePolicy
    assert all(value in {"DEFERRED_BY_SCOPE","NOT_MODELED"} for value in DEFERRED.values())
    assert ReleasePolicy().maximum_drop_m==.05
    # No lift integration is silently applied to existing motion entries.
    for name in ("single_contact.py","single_extraction.py","single_preplace.py","preplace_target.py"):
        assert "lifted_reception" not in (ROOT/"src/unloading_sim"/name).read_text(encoding="utf-8")


def test_planar_support_transfer_is_stage_scoped(layout):
    cartons=layout.cartons()
    target=next(b for b in cartons if b.name=="carton_l07_c00")
    remaining=[b for b in cartons if b.name not in {"carton_l07_c01","carton_l07_c02","carton_l07_c03"}]
    allowed=process_scheme(layout,target,cartons,remaining,"right_inner",planar_stack_contact_allowed=True)
    assert allowed["turn_y_intervals_world_m"]
    assert len(allowed["intermediate_planar_contact_ids"])==5
    assert len(allowed["initial_support_ids"])==1
    assert allowed["final_stack_separation"]
    assert allowed["drop"]["tool_motion_world_m"]==[0,0,0]
    # Fully occupied top row stays a channel counterexample, even with support permission.
    blocked=process_scheme(layout,target,cartons,cartons,"right_inner",planar_stack_contact_allowed=True)
    assert not blocked["turn_y_intervals_world_m"]
    assert blocked["channel_screen_blockers"]


def test_actual_intermediate_rigid_tool_collision_is_not_cup_exempt():
    from unloading_sim.lifted_reception import screen_sweep
    rigid=box((0,0,2),(.1,.1,.1),"rigid")
    obstacle=box((-.5,0,2),(.1,.1,.1),"neighbor")
    r=screen_sweep([rigid],[-1,0,0],[obstacle],compliant_ids={"flexible"},
        stack_ids={"neighbor"},ignore_flexible_neighbors=True)
    assert r["issues"][0]["pair"]==["rigid","neighbor"]


def test_flexible_initial_contact_does_not_hide_new_rigid_drop_snag(layout):
    belt=LiftedConveyorScene.build(layout,surface_z_world_m=1.75).belt
    release=box((-.55,.35,2.25))
    lip=box((-.55,.55,2.25),(.05,.002,.05),"cup")
    hook=box((-.55,.35,1.99),(.05,.05,.01),"insert")
    r=drop_geometry(release,[lip,hook],belt,[],compliant_ids={"cup"})
    assert r["screen"]["named_initial_contacts"]
    assert ["target","insert"] in [i["pair"] for i in r["screen"]["issues"]]


def test_no_robot_geometry_cannot_be_reported_clear(layout):
    cartons=layout.cartons()
    target=next(b for b in cartons if b.name=="carton_l07_c02")
    r=process_scheme(layout,target,cartons,cartons,"middle_front")
    assert r["initial_robot_belt_screen"]["status"]=="NOT_EVALUATED"
    assert r["applicability"]!="DISPLAY_SAMPLE_GEOMETRIC_SCREEN_CLEAR"
    assert not r["continuous_swept_proof"] and not r["execution_ready"]


def test_gross_initial_flexible_overlap_is_not_a_contact_permission(layout):
    belt=LiftedConveyorScene.build(layout,surface_z_world_m=1.75).belt
    target=box((-.55,.35,2.25))
    cup=box((-.55,.35,2.25),(.1,.1,.1),"cup")
    r=drop_geometry(target,[cup],belt,[],compliant_ids={"cup"})
    assert r["status"]=="POSSIBLE_INTERFERENCE"
    assert not r["screen"]["named_initial_contacts"]


def test_invalid_height_input_rejected():
    with pytest.raises(ValueError,match="INVALID_HEIGHT_INPUT"):
        StartHeight.capture(box(rotation=np.ones((3,3))))
    with pytest.raises(ValueError,match="INVALID_HEIGHT_INPUT"):
        StartHeight.capture(box(),d=float("nan"))
