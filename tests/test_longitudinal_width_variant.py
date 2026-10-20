"""Literal dimensions for the user-authorized 0.60 m longitudinal experiment."""
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_lift_checks import check_coupled_state
from unloading_sim.junction_reception import reception_classification
from unloading_sim.lifted_reception import translated

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def layouts():
    return tuple(load_workcell_layout(ROOT/"configs/workcells"/name) for name in (
        "m710id70_unloading_layout_v1.yaml","m710id70_unloading_layout_v1_longitudinal_060.yaml"))


def test_literal_width_outer_alignment_and_new_chassis_gap(layouts):
    old,new=layouts
    assert audit_layout_constraints(new)["overall_status"]=="PASS"
    by={b.name:b for b in new.fixed_components()}
    belt=by["conveyor_longitudinal"]
    assert 2*belt.half_extents==pytest.approx([2.8,.6,.12])
    assert belt.center==pytest.approx([-2.3,-.8,.54])
    assert (belt.corners()[:,1].min(),belt.corners()[:,1].max())==pytest.approx([-1.1,-.5])
    assert by["chassis"].corners()[:,1].min()-belt.corners()[:,1].max()==pytest.approx(.1)
    assert new.layout_fingerprint!=old.layout_fingerprint
    assert new.assets==old.assets
    assert [b.name for b in new.cartons()]==[b.name for b in old.cartons()]


@pytest.mark.parametrize("height",[.6,1.65,1.70,1.75,2.05])
def test_same_source_variant_preserves_transverse_and_legal_supports(layouts,height):
    old,new=layouts
    a={b.name:b for b in coupled_conveyor_state(old,height,()).boxes}
    b={b.name:b for b in coupled_conveyor_state(new,height,()).boxes}
    for name in a:
        if name=="conveyor_transverse" or name.startswith("lift_transverse"):
            np.testing.assert_array_equal(a[name].world_from_local,b[name].world_from_local)
            np.testing.assert_array_equal(a[name].half_extents,b[name].half_extents)
    # Narrowing the belt must not translate the longitudinal support feet
    # through the trailer wall. Their floor stations remain fixed.
    for name in a:
        if name.startswith("lift_longitudinal") and "_foot_" in name:
            np.testing.assert_allclose(a[name].world_from_local,b[name].world_from_local,atol=1e-12,rtol=0)
    check=check_coupled_state(new,height,(),designed_contacts=[
        ["chassis","conveyor_transverse"],["chassis","conveyor_longitudinal"],["conveyor_transverse","conveyor_longitudinal"]])
    assert check["passed"],check["failures"]
    target=next(x for x in new.cartons() if x.name=="carton_l07_c00")
    region=reception_classification(translated(target,[-.85,0,0]),b["conveyor_transverse"],b["conveyor_longitudinal"])
    assert region["junction"] and region["outfeed_direction_world"]==[-1.,0.,0.]
