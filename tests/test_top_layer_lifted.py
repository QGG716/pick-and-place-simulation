from pathlib import Path
import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.top_layer_lifted import FrozenTopLayer, process_options
from unloading_sim.workcell_layout import load_workcell_layout


@pytest.fixture(scope="module")
def scene():
    layout = load_workcell_layout(Path(__file__).resolve().parents[1]/
                                 "configs/workcells/m710id70_unloading_layout_v1.yaml")
    cartons = layout.cartons()
    belt = next(b for b in layout.fixed_components() if b.name == "conveyor_transverse")
    return cartons, belt, FrozenTopLayer.capture(cartons)


def test_frozen_highest_layer_and_lower_population(scene):
    cartons, _, frozen = scene
    assert len(frozen.target_ids) == 5
    assert frozen.evidence()["original_carton_count"] == 40
    assert len([b for b in cartons if b.name not in frozen.target_ids]) == 35
    assert frozen.remaining_ids(frozen.target_ids) == ()
    lower = next(b for b in cartons if b.name not in frozen.target_ids)
    with pytest.raises(ValueError):
        frozen.remaining_ids([lower.name])


def test_identity_not_carton_spelling_or_fixed_count(scene):
    cartons, _, _ = scene
    renamed = [OBB(b.center, b.half_extents, b.rotation, f"arbitrary-{i}", b.category)
               for i, b in enumerate(cartons) if b.center[1] >= 0]
    frozen = FrozenTopLayer.capture(renamed)
    assert len(frozen.target_ids) == 3
    assert all(name.startswith("arbitrary-") for name in frozen.target_ids)


def test_geometry_selects_front_direct_and_lateral_including_middle(scene):
    cartons, belt, frozen = scene
    by_name = {b.name: b for b in cartons}
    options = {n: process_options(by_name[n], belt, frozen, cartons) for n in frozen.target_ids}
    ordered = sorted(frozen.target_ids, key=lambda n: by_name[n].center[1])
    for n in ordered:
        assert options[n][0]["template"] == "FRONT_DIRECT_HMAX"
        assert options[n][0]["height_m"] == pytest.approx(2.05)
        assert options[n][1]["template"] == "FRONT_DIRECT_HMIN"
        assert options[n][1]["height_m"] == pytest.approx(1.75)
    assert options[ordered[0]][-1]["template"] == "RIGHT_INNER_HMAX"
    assert options[ordered[-1]][-1]["template"] == "LEFT_INNER_HMIN"
    assert options[ordered[-1]][-1]["segment_directions_world"][0] == [0., -1., 0.]


def test_last_remaining_keeps_original_role_and_height(scene):
    cartons, belt, frozen = scene
    right = next(b for b in cartons if b.name == frozen.target_ids[0])
    remaining = [b for b in cartons if b.name not in frozen.target_ids or b.name == right.name]
    options = process_options(right, belt, frozen, remaining)
    assert options[0]["original_role"] == "RIGHT_END"
    assert options[-1]["template"] == "RIGHT_INNER_HMAX"
    moved = OBB(right.center+[0, 0, -.1], right.half_extents, right.rotation, right.name, right.category)
    with pytest.raises(ValueError, match="FROZEN_TARGET_GEOMETRY_CHANGED"):
        process_options(moved, belt, frozen, [moved])


def test_lower_target_not_added_after_top_removed(scene):
    cartons, belt, frozen = scene
    lower = next(b for b in cartons if b.name not in frozen.target_ids)
    with pytest.raises(ValueError, match="TARGET_OUTSIDE_FROZEN_TOP_LAYER"):
        process_options(lower, belt, frozen, [lower])
