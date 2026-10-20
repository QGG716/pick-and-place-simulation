from pathlib import Path
import numpy as np
import pytest

from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_lift_checks import assembly_mates, check_coupled_state, check_coupled_interval
from unloading_sim.geometry import OBB
from unloading_sim.workcell_layout import load_workcell_layout


@pytest.fixture(scope="module")
def layout():
    return load_workcell_layout(Path(__file__).resolve().parents[1]/
                                "configs/workcells/m710id70_unloading_layout_v1_rear_fixed_20261019.yaml")


def test_existing_rear_stand_solid_overlap_has_literal_numeric_witness(layout):
    result = check_coupled_state(layout, .6, legacy_geometry=True)
    assert not result["passed"]
    assert result["object_count"] == 139
    assert len(result["carton_ids"]) == 40
    for stand, y in (("lift_rear_stand_0", -1.04), ("lift_rear_stand_2", -.46)):
        failure = next(f for f in result["failures"] if set(f["pair"]) == {stand, "conveyor_longitudinal"})
        assert failure["classification"] == "SOLID_OBB_INTERSECTION"
        intersection = failure["exact_axis_aligned_intersection"]
        np.testing.assert_allclose(intersection["min_world_m"], [-3., y-.04, .48], atol=1e-12)
        np.testing.assert_allclose(intersection["max_world_m"], [-2.96, y+.04, .55], atol=1e-12)
        np.testing.assert_allclose(intersection["size_m"], [.04, .08, .07], atol=1e-12)
        assert intersection["volume_m3"] == pytest.approx(.000224)
        assert failure["original_cad_penetration_proven"] is False


def test_exact_topology_does_not_exempt_entire_mechanism(layout):
    state = coupled_conveyor_state(layout, .6)
    pairs = assembly_mates(state)
    assert tuple(sorted(("lift_rear_bearing", "conveyor_longitudinal"))) in pairs
    assert tuple(sorted(("lift_rear_stand_0", "conveyor_longitudinal"))) not in pairs
    assert tuple(sorted(("lift_longitudinal_rear_foot_1", "chassis"))) not in pairs
    assert all(not any(n.startswith("carton_") or n == "J5_link" for n in p) for p in pairs)


def test_carton_named_like_mechanism_is_still_checked(layout):
    state = coupled_conveyor_state(layout, .6)
    arm = next(b for b in state.boxes if b.name == "lift_transverse_left_arm_0_0")
    obstacle = OBB(arm.center, [.02, .02, .02], np.eye(3), "lift_unknown_carton", "carton")
    result = check_coupled_state(layout, .6, [*layout.cartons(), obstacle])
    assert any("lift_unknown_carton" in f["pair"] for f in result["failures"])


@pytest.mark.parametrize("start,end,direction", [(.6, .62, "UP"), (.62, .6, "DOWN"), (.6, .6, "HOLD")])
def test_bidirectional_interval_and_hold_preserve_identity(layout, start, end, direction):
    result = check_coupled_interval(layout, start, end, height_step_m=.01)
    assert result["direction"] == direction
    assert result["samples"][0]["height_m"] == start
    assert result["samples"][-1]["height_m"] == end
    assert result["maximum_height_step_m"] <= .01+1e-12
    assert all(len(s["carton_ids"]) == 40 for s in result["samples"])
    assert not result["continuous_swept_proof"]
    assert result["execution_duration_s"] is None
    if start == end:
        assert result["sample_count"] == 1


def test_interval_invalid_inputs_and_resources(layout):
    for start, end, step in ((.6, 2.05, 0.), (.6, float("nan"), .01), (.6, 2.05, 1e-6)):
        with pytest.raises(ValueError):
            check_coupled_interval(layout, start, end, height_step_m=step)


def test_broadphase_is_not_a_collision_verdict(layout):
    # Each rejected pair carries a refined SAT/surface query. AABB intersects
    # alone cannot supply an exact volume for tilted solids.
    result = check_coupled_state(layout, 2.05, legacy_geometry=True)
    assert all(f["query_method"].startswith("OBB_SAT") for f in result["failures"])
    tilted = next(f for f in result["failures"] if
                  set(f["pair"]) == {"lift_rear_stand_0", "conveyor_longitudinal"})
    assert "exact_axis_aligned_intersection" not in tilted


@pytest.mark.parametrize('start,end',[(.6,2.05),(2.05,.6),(.6,1.75),(1.75,.6),(1.75,2.05),(2.05,1.75),(2.05,2.05)])
def test_corrected_synchronized_working_interval(layout,start,end):
    contacts=[['chassis','conveyor_transverse'],['chassis','conveyor_longitudinal'],
              ['conveyor_transverse','conveyor_longitudinal']]
    result=check_coupled_interval(layout,start,end,height_step_m=.01,designed_contacts=contacts)
    assert all(s['passed'] for s in result['samples']), [(s['height_m'],s['failures']) for s in result['samples'] if not s['passed']]
    assert not result['continuous_swept_proof']


def test_corrected_support_connectivity_and_independent_chassis(layout):
    from unloading_sim.pair_clearance import obb_surface_distance
    for h in [.6,1.75,2.05]:
        state=coupled_conveyor_state(layout,h)
        b={x.name:x for x in state.boxes}
        for stand in ('lift_rear_stand_0','lift_rear_stand_2'):
            assert obb_surface_distance(b[stand],b['lift_rear_bearing']) < 1e-9
            assert obb_surface_distance(b[stand],b['conveyor_longitudinal']) >= .005
        for key in ('lift_longitudinal_near','lift_longitudinal_rear'):
            assert obb_surface_distance(b[key+'_neck'],b[key+'_head_rail']) < 1e-9
            assert obb_surface_distance(b[key+'_neck'],b[key+'_swivel_head']) < 1e-9
            assert obb_surface_distance(b[key+'_foot_1'],b['chassis']) >= .019999
        assert obb_surface_distance(b['lift_rear_bearing'],b['chassis']) >= .009999
