"""Narrow the support spacing, preserving every solid and the belt geometry."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_lift_checks import assembly_mates, check_coupled_state
from unloading_sim.workcell_layout import audit_layout_constraints, load_workcell_layout

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def layouts():
    return tuple(load_workcell_layout(ROOT/"configs/workcells"/name) for name in (
        "m710id70_unloading_layout_v1_longitudinal_060.yaml",
        "m710id70_unloading_layout_v1_transverse_scissor_030.yaml"))


@pytest.mark.parametrize("height", [.60, 1.65, 1.70, 1.75, 2.05, 2.10])
def test_literal_widths_symmetric_offset_and_no_removed_solids(layouts, height):
    old, new = layouts
    assert audit_layout_constraints(new)["overall_status"] == "PASS"
    a, b = (coupled_conveyor_state(x, height) for x in layouts)
    assert set(x.name for x in a.boxes) == set(x.name for x in b.boxes)
    old_boxes = {x.name: x for x in a.boxes}
    assert len(new.cartons()) == 40 and new.assets == old.assets
    assert assembly_mates(a) == assembly_mates(b)
    for box in b.boxes:
        prior = old_boxes[box.name]
        np.testing.assert_array_equal(prior.half_extents, box.half_extents)
        np.testing.assert_array_equal(prior.rotation, box.rotation)
        dx = .08 if box.name.startswith("lift_transverse_left_") else (
            -.08 if box.name.startswith("lift_transverse_right_") else 0.)
        np.testing.assert_allclose(box.center-prior.center, [dx, 0, 0], atol=1e-12, rtol=0)
    for state, expected in ((a, [-.83, -.27]), (b, [-.75, -.35])):
        corners = np.vstack([x.corners() for x in state.boxes if x.name.startswith("lift_transverse_")])
        assert [corners[:, 0].min(), corners[:, 0].max()] == pytest.approx(expected)
    assert b.mechanism["transverse_support_spacing_m"] == .30
    assert b.mechanism["transverse_support_outer_width_m"] == .40
    assert b.scene_fingerprint != a.scene_fingerprint
    # Exact same source supplies display and planning; permitted assembly
    # contacts are unchanged, and all other mechanism/world pairs still checked.
    check = check_coupled_state(new, height, designed_contacts=[
        ["chassis", "conveyor_transverse"], ["chassis", "conveyor_longitudinal"],
        ["conveyor_transverse", "conveyor_longitudinal"]])
    assert check["passed"], check["failures"]
    assert check["physical_execution"] == "NOT_RUN"


@pytest.mark.parametrize("spacing", [.10, .61, 0., float("nan"), float("inf")])
def test_invalid_spacing_fails_closed(layouts, spacing):
    data = deepcopy(layouts[1].data)
    data["conveyors"]["coupling"]["transverse_support_spacing_m"] = spacing
    with pytest.raises(ValueError, match="TRANSVERSE_SUPPORT_SPACING"):
        coupled_conveyor_state(replace(layouts[1], data=data), 1.75)


def test_historical_default_keeps_geometry_and_identity(layouts):
    # The explicit baseline is geometrically equal, but its layout identity
    # correctly changes. The omitted parameter remains the historical 460 mm.
    old = layouts[0]
    data = deepcopy(old.data)
    data["conveyors"]["coupling"]["transverse_support_spacing_m"] = .46
    a = coupled_conveyor_state(old, 1.75)
    b = coupled_conveyor_state(replace(old, data=data), 1.75)
    for prior, box in zip(a.boxes, b.boxes):
        np.testing.assert_allclose(prior.world_from_local, box.world_from_local, atol=1e-12, rtol=0)
        np.testing.assert_array_equal(prior.half_extents, box.half_extents)
    assert "transverse_support_spacing_m" not in a.mechanism
