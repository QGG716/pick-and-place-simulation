"""Kinematic geometry tests; no physics or planning qualification."""
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state, ScissorSpec, scissor_stack
from unloading_sim.lifted_reception import LiftedConveyorScene


@pytest.fixture(scope="module")
def layout():
    return load_workcell_layout(Path(__file__).resolve().parents[1]/
        "configs/workcells/m710id70_unloading_layout_v1.yaml")


def test_literal_dimensions_and_fixed_rear(layout):
    low = coupled_conveyor_state(layout, .6)
    high = coupled_conveyor_state(layout, 2.05)
    np.testing.assert_allclose(high.mechanism["rear_hinge_world"], [-3.,-.75,.6], atol=1e-12)
    np.testing.assert_allclose(high.mechanism["near_tip_world"], [-.6046920861,-.75,2.05], atol=1e-9)
    assert high.mechanism["longitudinal_length_m"] == pytest.approx(2.8)
    assert high.mechanism["tilt_rad"] == pytest.approx(.54434414888, abs=1e-10)
    assert high.mechanism["near_tip_within_transverse_x"]
    for state in (low, high):
        by_id = {b.name:b for b in state.boxes}
        long = by_id["conveyor_longitudinal"]
        np.testing.assert_allclose(long.center+long.rotation@[-1.4,0,.06], [-3,-.75,.6], atol=1e-12)
        np.testing.assert_allclose(long.center+long.rotation@[1.4,0,.06], state.mechanism["near_tip_world"])
        assert np.linalg.det(long.rotation) == pytest.approx(1)
        assert by_id["conveyor_transverse"].center[2] == pytest.approx(state.mechanism["height_m"]-.06)
    assert low.scene_fingerprint != high.scene_fingerprint
    assert coupled_conveyor_state(layout,2.05).scene_fingerprint == high.scene_fingerprint


def test_all_heights_fixed_lengths_unique_colliders_and_unchanged_world(layout):
    original = {b.name:b for b in (*layout.fixed_components(), *layout.cartons())}
    identities=[]
    for height in np.linspace(.6,2.05,31):
        state = coupled_conveyor_state(layout,height)
        identities.append(state.scene_fingerprint)
        assert len(state.boxes)==len({b.name for b in state.boxes})
        assert sum(b.category=="carton" for b in state.boxes)==40
        for b in state.boxes:
            if b.name in original and not b.name.startswith("conveyor_"):
                np.testing.assert_array_equal(b.world_from_local, original[b.name].world_from_local)
            assert np.isfinite(b.world_from_local).all()
            assert np.linalg.det(b.rotation)==pytest.approx(1)
            if b.category=="lift_mechanism":
                assert b.corners()[:,2].min() >= 0
        for stack in state.mechanism["stacks"]:
            for bar in stack["bars"]:
                a,b=np.asarray(bar["endpoints"])
                assert np.linalg.norm(b-a)==pytest.approx(bar["length_m"], abs=1e-12)
        assert state.evidence()["physical_execution"]=="NOT_RUN"
        assert not state.evidence()["continuous_swept_proof"]
    assert len(set(identities))==31


@pytest.mark.parametrize("height", [float("nan"),float("inf"),.2,4.])
def test_invalid_height_not_clamped(layout,height):
    with pytest.raises(ValueError):
        coupled_conveyor_state(layout,height)


def test_scissor_singularities_and_nonfinite():
    for top in (0.,2.,float("nan")):
        with pytest.raises(ValueError):
            scissor_stack("test",[0,0,0],[0,1,0],top,ScissorSpec(2,1.))
    for spec in (ScissorSpec(0,1),ScissorSpec(1,float("nan")),ScissorSpec(2.5,1)):
        with pytest.raises(ValueError):
            spec.validate()


def test_legacy_pure_z_contract_and_remaining_cartons_unchanged(layout):
    legacy=LiftedConveyorScene.build(layout,surface_z_world_m=2.05)
    assert len(legacy.boxes)==48
    assert legacy.height["moving_object_ids"]==["conveyor_transverse"]
    assert next(b for b in legacy.boxes if b.name=="conveyor_longitudinal").center[2]==pytest.approx(.54)
    coupled=coupled_conveyor_state(layout,2.05,cartons=layout.cartons()[1:])
    assert sum(b.category=="carton" for b in coupled.boxes)==39
    assert coupled.mechanism["native_receiver_transition"].startswith("UNSUPPORTED")


def test_explicit_scene_adapter_includes_mechanisms_with_new_identity(layout):
    legacy=LiftedConveyorScene.build(layout,surface_z_world_m=2.05)
    coupled=LiftedConveyorScene.build(layout,surface_z_world_m=2.05,coupled_lift=True)
    assert coupled.scene_fingerprint!=legacy.scene_fingerprint
    assert coupled.height["native_transition_compatible"] is False
    assert coupled.height["lift_mechanism"]=="SCISSOR_KINEMATIC_CONCEPT"
    assert len(coupled.boxes)>len(legacy.boxes)
    expected=coupled_conveyor_state(layout,2.05)
    for a,b in zip(coupled.boxes,expected.boxes):
        np.testing.assert_array_equal(a.world_from_local,b.world_from_local)
    assert coupled.evidence()["native_world_sent"] is False
