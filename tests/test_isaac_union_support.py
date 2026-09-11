"""Export consumes independently recomputable complete-bottom union evidence."""

import copy

import numpy as np
import pytest

from unloading_sim.conveyor_placement import support_union_audit
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.isaac_bridge import _serialize_obb, _validated_place_evidence


def fixture(*, notch=False, penetration=0.):
    if notch:
        supports = [OBB([-.5, 0., .5], [.5, 1., .1], np.eye(3), "west", "conveyor"),
                    OBB([0., -.5, .5], [1., .5, .1], np.eye(3), "south", "conveyor")]
        payload = OBB([0., 0., .75 - penetration], [.5, .5, .15],
                      rotation_matrix_from_rpy(0., 0., np.pi / 4), "actual-carton", "carton")
    else:
        supports = [OBB([-.25, 0., .5], [.25, .5, .1], np.eye(3), "west", "conveyor"),
                    OBB([.25, 0., .5], [.25, .5, .1], np.eye(3), "east", "conveyor")]
        payload = OBB([0., 0., .75 - penetration], [.3, .2, .15],
                      rotation_matrix_from_rpy(0., 0., .25), "actual-carton", "carton")
    audit = support_union_audit(payload, supports, contact_tolerance_m=.0002)
    segment = {"target": payload.name, "validation": {"contact_tolerance_m": .0002}, "place": {
        "receiver": "west", "place_surface": "west",
        "actual_box_pose_world": payload.world_from_local.tolist(),
        "release_center_world_m": payload.center.tolist(),
        "support_names": [box.name for box in supports],
        "load_bearing_support_names": ["west"], "support": audit}}
    primitives = [_serialize_obb(box) for box in supports]
    primitives.append(_serialize_obb(payload, dynamic=True, mass_kg=42.5))
    return segment, primitives


def test_actual_yawed_union_exports_without_fabricated_single_deck_clearance():
    segment, primitives = fixture()
    result = _validated_place_evidence(segment, scene_primitives=primitives)
    audit = result["planned_support_audit"]
    assert audit["supported"]
    assert set(audit["receiver_names"]) == {"west", "east"}
    assert audit["unsupported_area_m2"] == pytest.approx(0., abs=1e-12)
    assert "edge_clearance_xy_m" not in audit
    assert audit == segment["place"]["support"]


@pytest.mark.parametrize("case", ["notch", "penetration", "pose", "area", "receiver", "tolerance", "scene"])
def test_export_rejects_unproven_or_falsified_union_geometry(case):
    segment, primitives = fixture(notch=case == "notch", penetration=.0004 if case == "penetration" else 0.)
    audit = segment["place"]["support"]
    if case in {"notch", "penetration"}:
        assert not audit["supported"]
        audit["supported"] = True  # A declared boolean cannot override geometry.
    elif case == "pose":
        audit["actual_box_pose"][2][3] += .001
    elif case == "area":
        audit["footprint_area_m2"] += .01
    elif case == "receiver":
        segment["place"]["support_names"] = ["west"]
    elif case == "tolerance":
        audit["tolerance_m"] = .01
    elif case == "scene":
        primitives[0]["size_m"][0] += .01
    with pytest.raises(ValueError):
        _validated_place_evidence(segment, scene_primitives=primitives)


def test_support_record_geometry_is_bound_to_frozen_scene_even_if_recomputed_audit_matches():
    segment, primitives = fixture()
    altered = copy.deepcopy(segment)
    support = altered["place"]["support"]["support_obbs"][0]
    support["pose_world"][0][3] -= .1
    with pytest.raises(ValueError, match="frozen scene"):
        _validated_place_evidence(altered, scene_primitives=primitives)
