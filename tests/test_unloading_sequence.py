from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.unloading_sequence import (
    RowUnloadingState, cluster_carton_rows, fair_face_candidates, height_face_prior,
)


def carton(name, y, z, x=0.5):
    return OBB([x, y, z], [0.3, 0.2, 0.15], np.eye(3), name, "carton")


def test_highest_row_center_priority_and_geometric_id_independence():
    boxes = [carton(f"random-{i}", y, 2.25) for i, y in enumerate([-0.84, -0.42, 0, 0.42, 0.84])]
    boxes += [carton("actually-lower-but-name-top", 0, 1.95)]
    selection = RowUnloadingState().rank(boxes, candidate_costs={"random-1": 0.1, "random-3": 0.9})
    assert selection.status == "READY"
    assert [item.name for item in selection.candidates][:3] == ["random-2", "random-1", "random-3"]
    assert {item.name for item in selection.candidates} == {f"random-{i}" for i in range(5)}
    assert selection.row_center_y_m == pytest.approx(0)


def test_stable_row_center_missing_cartons_and_no_noise_retries():
    boxes = [carton("west", -0.6, 0.45), carton("middle", 0, 0.45), carton("east", 0.6, 0.45)]
    state = RowUnloadingState()
    first = state.rank(boxes)
    state.record_failure("middle", first.scene_fingerprint, "all faces tried")
    noisy = [carton(box.name, box.center[1], box.center[2] + 0.0001) for box in boxes]
    next_selection = state.rank(noisy)
    assert next_selection.scene_fingerprint == first.scene_fingerprint
    assert "middle" not in [item.name for item in next_selection.candidates]
    # A real removal changes space and re-enables the prior failure.  The
    # original row centre remains zero, although remaining extent centre is .3.
    reduced = state.rank([boxes[1], boxes[2]])
    assert reduced.scene_fingerprint != first.scene_fingerprint
    assert reduced.row_center_y_m == pytest.approx(0)
    assert reduced.candidates[0].name == "middle"


def test_all_failed_top_row_blocks_instead_of_removing_lower_support():
    boxes = [carton("upper-a", -0.21, 0.45), carton("upper-b", 0.21, 0.45),
             carton("lower-a", -0.21, 0.15), carton("lower-b", 0.21, 0.15)]
    state = RowUnloadingState()
    first = state.rank(boxes)
    for item in first.candidates:
        state.record_failure(item.name, first.scene_fingerprint, "NO_CONNECTION")
    blocked = state.rank(boxes)
    assert blocked.status == "ROW_BLOCKED"
    assert blocked.candidates == ()
    assert set(blocked.row_remaining_names) == {"upper-a", "upper-b"}
    lower = state.rank(boxes[2:])
    assert lower.status == "READY"
    assert {item.name for item in lower.candidates} == {"lower-a", "lower-b"}


def test_geometric_cluster_ignores_submillimeter_settling():
    boxes = [carton("a", -.42, .451), carton("b", 0, .45), carton("c", .42, .449),
             carton("d", 0, .15)]
    assert [len(row) for row in cluster_carton_rows(boxes)] == [3, 1]


def test_height_prior_uses_fixed_shoulder_and_preserves_real_face_budget():
    upper = height_face_prior(carton("a", 0, 2.25), ["front", "top", "left"], shoulder_height_m=1.2)
    lower = height_face_prior(carton("b", 0, .45), ["front", "top", "left"], shoulder_height_m=1.2)
    assert upper["front"] > upper["top"] > 0
    assert lower["top"] > lower["front"] > 0
    choices = list(fair_face_candidates({"front": range(20), "top": [42], "left": [99]}, upper, 4))
    assert choices == [("front", 0), ("left", 99), ("top", 42), ("front", 1)]
    with pytest.raises(ValueError, match="reserve"):
        list(fair_face_candidates({"front": [1], "top": [2]}, upper, 1))


def test_state_context_and_real_motion_reenable_failed_candidate():
    state = RowUnloadingState()
    boxes = [carton("any-id", 0, .45)]
    first = state.rank(boxes, scene_context={"receiver_revision": 1})
    state.record_failure("any-id", first.scene_fingerprint, "RECEIVER_OCCUPIED")
    assert state.rank(boxes, scene_context={"receiver_revision": 1}).status == "ROW_BLOCKED"
    assert state.rank(boxes, scene_context={"receiver_revision": 2}).status == "READY"
