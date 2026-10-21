"""Literal mounting dimensions and paired-world isolation regressions."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"tools"))
from plan_m710_mount_comparison import LAYOUTS, load_cases, make_state
from unloading_sim.workcell_layout import load_workcell_layout, audit_layout_constraints
from unloading_sim.moveit2_backend import box_message
from unloading_sim.top_layer_lifted import FrozenTopLayer


@pytest.mark.parametrize("name,expected,front", [
    ("baseline", [-1.325, .350, .600], -1.100),
    ("y_minus_100", [-1.325, .250, .600], -1.100),
    ("x_minus_100", [-1.425, .350, .600], -1.200),
    ("xy_minus_100", [-1.425, .250, .600], -1.200),
])
def test_only_approved_robot_mount_moves(name, expected, front):
    base = load_workcell_layout(ROOT/"configs/workcells"/LAYOUTS["baseline"])
    layout = load_workcell_layout(ROOT/"configs/workcells"/LAYOUTS[name])
    np.testing.assert_allclose(layout.robot_base_transform()[:3, 3], expected, atol=1e-12)
    assert layout.data["robot"]["base_front_edge_x_world_m"] == front
    assert audit_layout_constraints(layout)["overall_status"] == "PASS"
    assert [box_message(b) for b in layout.fixed_components()] == [box_message(b) for b in base.fixed_components()]
    assert [box_message(b) for b in layout.cartons()] == [box_message(b) for b in base.cartons()]
    assert layout.assets == base.assets
    # Authorized label cannot bless a different displacement.
    if name != "baseline":
        changed = deepcopy(layout.data)
        changed["robot"]["base_origin_xy_a_m"][1] += .01
        assert audit_layout_constraints(replace(layout, data=changed))["overall_status"] == "FAIL"


def test_paired_inputs_rebuild_only_initial_world_without_native_receipts():
    from types import SimpleNamespace
    data = load_cases(ROOT/"docs/experiments/m710_mount_comparison_20261021/paired-inputs.json")
    layout = load_workcell_layout(ROOT/"configs/workcells"/LAYOUTS["baseline"])
    for case in data["cases"]:
        state = make_state(case, SimpleNamespace(cartons=layout.cartons()))
        assert state.root() == case["root"] and state.cycles == []
        assert state.root()["predecessor_final_state_receipt"] is None
        assert type(state.frozen) is FrozenTopLayer if case["right_front_eccentric"] else type(state.frozen) is not FrozenTopLayer
        assert len(state.cartons)+len(state.ledger) == 40
        corrupt = deepcopy(case)
        corrupt["root"]["world_cartons"][0]["pose"][0][3] += .01
        with pytest.raises(ValueError, match="INITIAL_CARTONS_CHANGED"):
            make_state(corrupt, SimpleNamespace(cartons=layout.cartons()))


def test_additional_x_200_retreat_keeps_y_and_all_other_geometry():
    old = load_workcell_layout(ROOT/"configs/workcells"/LAYOUTS["xy_minus_100"])
    layout = load_workcell_layout(ROOT/"configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_300_y_minus_100.yaml")
    np.testing.assert_allclose(layout.robot_base_transform()[:3, 3], [-1.625, .250, .600], atol=1e-12)
    assert layout.data["robot"]["base_front_edge_x_world_m"] == -1.400
    assert audit_layout_constraints(layout)["overall_status"] == "PASS"
    assert layout.assets == old.assets
    for key in ("assembly", "chassis", "conveyors", "tool", "trailer", "carton_stack"):
        assert layout.data[key] == old.data[key]
    assert [box_message(b) for b in layout.fixed_components()] == [box_message(b) for b in old.fixed_components()]
    for axis in (0, 1):
        changed = deepcopy(layout.data)
        changed["robot"]["base_origin_xy_a_m"][axis] += .01
        assert audit_layout_constraints(replace(layout, data=changed))["overall_status"] == "FAIL"


def test_population_evidence_rejects_mixed_mount_labels():
    from verify_m710_population import expected_population_mount
    protocol = dict(mounting_variant='x_minus_300_y_minus_100',
        active_layout_file='configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_300_y_minus_100.yaml')
    assert expected_population_mount(protocol) == [-1.625, .250, .600]
    protocol['mounting_variant'] = 'xy_minus_100'
    with pytest.raises(ValueError, match='LAYOUT_VARIANT_MISMATCH'):
        expected_population_mount(protocol)
    protocol['mounting_variant'] = 'unreviewed'
    with pytest.raises(ValueError, match='UNKNOWN_POPULATION_MOUNT'):
        expected_population_mount(protocol)
