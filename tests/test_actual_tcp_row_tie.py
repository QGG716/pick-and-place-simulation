from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.geometry import OBB, make_transform, rotation_matrix_from_rpy
from unloading_sim.unloading_sequence import (
    RowSequencePolicy, RowUnloadingState, actual_tcp_approach_costs,
)


def _boxes():
    return [OBB([.5, y, .45], [.3, .2, .15], np.eye(3), name, "carton")
            for name, y in (("negative-side", -.6), ("centre", 0), ("positive-side", .6))]


def _costs(boxes, pose, faces=None):
    return actual_tcp_approach_costs(
        boxes, pose, faces or {box.name: ("front",) for box in boxes},
        pregrasp_standoff_m=.1, virtual_contact_offset_m=.0325)


def _rank(boxes, evidence, *, weight=.15):
    return RowUnloadingState(RowSequencePolicy(actual_cost_weight=weight)).rank(
        boxes, candidate_costs={name: item["normalized_cost"]
                               for name, item in evidence["candidates"].items()})


def test_current_tcp_changes_left_right_tie_without_overriding_row_centre():
    boxes = _boxes()
    negative = _costs(boxes, make_transform(translation=[-.5, -.75, .45]))
    positive = _costs(boxes, make_transform(translation=[-.5, .75, .45]))
    assert [item.name for item in _rank(boxes, negative).candidates] == [
        "centre", "negative-side", "positive-side"]
    assert [item.name for item in _rank(boxes, positive).candidates] == [
        "centre", "positive-side", "negative-side"]
    assert negative["candidates"]["negative-side"]["translation_lower_bound_m"] == pytest.approx(.6325)
    assert negative["candidates"]["positive-side"]["normalized_cost"] == 1
    assert negative["feasibility_claim"] == "NONE_NO_IK_OR_COLLISION_QUERY"


def test_only_available_faces_participate_and_complete_face_not_centre_is_used():
    box = _boxes()[1]
    current = make_transform(translation=[.5, 0, 2])
    front = _costs([box], current, {box.name: ("front",)})
    both = _costs([box], current, {box.name: ("front", "top")})
    assert list(front["candidates"][box.name]["face_lower_bounds_m"]) == ["front"]
    assert both["candidates"][box.name]["translation_lower_bound_m"] == pytest.approx(1.3325)
    assert both["candidates"][box.name]["translation_lower_bound_m"] < front["candidates"][box.name]["translation_lower_bound_m"]


def test_rotated_actual_carton_frames_preserve_same_geometric_distance():
    boxes = _boxes()
    current = make_transform(translation=[-.5, -.75, .45])
    transform = make_transform(rotation_matrix_from_rpy(.2, -.3, .7), [1, -2, .4])
    original = _costs(boxes, current)
    rotated = _costs([box.transformed(transform) for box in boxes], transform @ current)
    for name in original["candidates"]:
        assert rotated["candidates"][name]["translation_lower_bound_m"] == pytest.approx(
            original["candidates"][name]["translation_lower_bound_m"])


def test_zero_weight_disables_actual_tie_and_zero_distance_normalizes_safely():
    boxes = _boxes()
    evidence = _costs(boxes, make_transform(translation=[-.5, -.75, .45]))
    assert [item.name for item in _rank(boxes, evidence, weight=0).candidates] == [
        "centre", "positive-side", "negative-side"]
    at_face = _costs([boxes[1]], make_transform(translation=[.2, 0, .45]))
    assert at_face["normalization_distance_m"] == 0
    assert at_face["candidates"]["centre"]["normalized_cost"] == 0


def test_current_pose_and_unknown_faces_fail_closed():
    with pytest.raises(ValueError, match="world pose"):
        _costs(_boxes(), np.full((4, 4), np.nan))
    with pytest.raises(ValueError, match="unknown exposed face"):
        _costs(_boxes(), np.eye(4), {"centre": ("not-a-face",)})

