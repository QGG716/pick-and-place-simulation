from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.conveyor_placement import (
    ConveyorSupport, PlacementPolicy, conveyor_support_surfaces,
    generate_conveyor_placements, support_union_audit,
)
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy


def box(name, center, half, yaw=0, category="conveyor"):
    return OBB(center, half, rotation_matrix_from_rpy(0, 0, yaw), name, category)


def belts():
    # Literal confirmed shape: X transverse width .7, Y length 1.5;
    # longitudinal X length 2.8, Y width .7. Both top faces at .6.
    return (box("arbitrary-cross", [-.55, .35, .54], [.35, .75, .06]),
            box("arbitrary-long", [-1.6, -.75, .54], [1.4, .35, .06]))


def test_discovery_uses_role_and_both_belts_get_noncentral_candidates():
    supports = belts()
    chassis = box("not-a-belt", [-1.95, .35, .3], [1.05, .75, .3], category="chassis")
    assert len(conveyor_support_surfaces([chassis, *supports])) == 2
    target = box("payload", [.5, .3, 2.25], [.3, .2, .15], category="carton")
    candidates = generate_conveyor_placements(target, supports, policy=PlacementPolicy(maximum_candidates=12))
    assert candidates[0].receiver_names == (supports[0].name,)
    assert candidates[1].receiver_names == (supports[1].name,)
    assert any(np.linalg.norm(item.payload.center[:2] - supports[0].center[:2]) > 0.1
               for item in candidates if item.receiver_names == (supports[0].name,))
    assert all(item.support["supported"] for item in candidates)
    assert all(item.support["bottom_z_range_m"] == pytest.approx([.6, .6]) for item in candidates)
    assert any(abs(item.yaw_rad) > 0.1 for item in candidates[:6])


def test_complete_footprint_rejects_overhang_and_excessive_penetration():
    support = belts()[0]
    target = box("payload", [-.55, .35, .75], [.3, .2, .15], category="carton")
    assert support_union_audit(target, [support])["supported"]
    overhang = box("payload", [-.49, .35, .75], [.3, .2, .15], category="carton")
    assert support_union_audit(overhang, [support])["reason"] == "UNSUPPORTED_FOOTPRINT"
    penetrating = box("payload", [-.55, .35, .746], [.3, .2, .15], category="carton")
    assert support_union_audit(penetrating, [support])["reason"] == "SUPPORT_PENETRATION"


def test_union_rejects_interior_notch_even_when_every_corner_is_supported():
    # L union x<=0 OR y<=0. Rotated square corners lie on/in the L,
    # but its top-right triangular interior passes across the missing quadrant.
    a = box("vertical", [-.5, 0, .5], [.5, 1, .1])
    b = box("horizontal", [0, -.5, .5], [1, .5, .1])
    target = box("diamond", [0, 0, .75], [.5, .5, .15], np.pi / 4, "carton")
    bottom = target.corners()[::2]
    assert all(a.contains(point, 1e-10) or b.contains(point, 1e-10) for point in bottom)
    audit = support_union_audit(target, [a, b])
    assert not audit["supported"]
    assert audit["unsupported_area_m2"] == pytest.approx(.25, abs=2e-6)


def test_two_contiguous_supports_and_actual_yaw_are_fully_audited():
    supports = [box("west", [-.25, 0, .5], [.25, .5, .1]),
                box("east", [.25, 0, .5], [.25, .5, .1])]
    target = box("carton", [0, 0, .75], [.3, .2, .15], .25, "carton")
    audit = support_union_audit(target, supports)
    assert audit["supported"]
    assert set(audit["receiver_names"]) == {"west", "east"}
    tilted = OBB(target.center, target.half_extents, rotation_matrix_from_rpy(.03, 0, .25), target.name, "carton")
    assert not support_union_audit(tilted, supports)["supported"]


def test_measured_submilliradian_tilt_uses_same_support_band_as_outer_gate():
    support = box("belt", [0, 0, .54], [1.4, .35, .06])
    rotation = rotation_matrix_from_rpy(.0002, -.0002, 0)
    vertical_half_extent = float(np.sum(np.abs(rotation[2]) * np.array([.3, .2, .15])))
    payload = OBB([0, 0, .6 + vertical_half_extent + .0003], [.3, .2, .15], rotation,
                  "payload", "carton")
    audit = support_union_audit(
        payload, [support], contact_tolerance_m=.003, max_support_tilt_rad=np.deg2rad(5)
    )
    assert audit["supported"]
    assert audit["support_face_local_axis"] == 2
    assert audit["bottom_z_range_m"][0] >= .6


def test_failed_place_has_other_locations_and_occupied_belt_is_excluded():
    supports = belts()
    target = box("new", [.5, 0, 2.25], [.3, .2, .15], category="carton")
    # Occupy the full transverse surface with a distinct real carton OBB.
    occupied = box("already-placed", [-.55, .35, .75], [.35, .75, .15], category="carton")
    candidates = generate_conveyor_placements(target, supports, occupied=[occupied],
                                               policy=PlacementPolicy(maximum_candidates=8))
    assert len(candidates) >= 2
    assert all(supports[0].name not in item.receiver_names for item in candidates)
    assert not np.array_equal(candidates[0].payload.center, candidates[1].payload.center)
    assert all(item.payload.signed_distance_obb(occupied) >= .02 - 1e-10 for item in candidates)


def test_extra_engineering_margin_is_explicit_and_not_default():
    support = box("exact", [0, 0, .5], [.3, .2, .1])
    target = box("payload", [0, 0, .75], [.3, .2, .15], category="carton")
    assert support_union_audit(target, [support])["supported"]
    assert not support_union_audit(target, [support], engineering_edge_margin_m=.01)["supported"]


def test_valid_lower_surface_cannot_hide_overlap_with_higher_rigid_receiver():
    lower = box("lower", [0, 0, .5], [.6, .5, .1])
    higher = box("higher", [.25, 0, .503], [.1, .2, .1])
    target = box("payload", [0, 0, .75], [.3, .2, .15], category="carton")
    assert support_union_audit(target, [lower])["supported"]
    assert support_union_audit(target, [lower, higher])["reason"] == "SUPPORT_PENETRATION"


def test_rotated_carton_uses_actual_support_face_and_height():
    support = box("belt", [0.0, 0.0, 0.54], [0.8, 0.6, 0.06])
    rotated = OBB(
        [0.0, 0.0, 0.90], [0.30, 0.20, 0.15],
        rotation_matrix_from_rpy(0.0, np.pi / 2.0, 0.0), "box", "carton",
    )
    audit = support_union_audit(rotated, [support])
    assert audit["supported"]
    assert audit["support_face_local_axis"] == 0
    assert audit["support_face_local_sign"] == 1

    candidates = generate_conveyor_placements(
        box("box", [0.0, 0.0, 1.5], [0.30, 0.20, 0.15], category="carton"),
        [support],
        policy=PlacementPolicy(
            yaw_offsets_rad=(0.0,),
            orientation_rpy_offsets_rad=((0.0, np.pi / 2.0, 0.0),),
            maximum_candidates=2,
        ),
    )
    assert candidates
    assert candidates[0].payload.center[2] == pytest.approx(0.90)
    assert candidates[0].support["support_face_local_axis"] == 0
