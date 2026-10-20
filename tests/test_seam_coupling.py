from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
from unloading_sim.workcell_layout import load_workcell_layout, audit_layout_constraints
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_lift_checks import check_coupled_interval, assembly_mates
from unloading_sim.concept_solid import solid_signed_gap, ConceptSolid
from unloading_sim.coupled_batch import message_box, coupled_binding
from unloading_sim.moveit2_backend import box_message
from unloading_sim.top_layer_lifted import FrozenTopLayer, process_options

ROOT=Path(__file__).resolve().parents[1]
CONTACTS=[["chassis","conveyor_transverse"],["chassis","conveyor_longitudinal"],["conveyor_transverse","conveyor_longitudinal"]]

@pytest.fixture(scope="module")
def layout():
    return load_workcell_layout(ROOT/"configs/workcells/m710id70_unloading_layout_v1.yaml")


def test_literal_new_low_layout(layout):
    boxes={b.name:b for b in layout.fixed_components()}
    for name,lo,hi in [
        ("conveyor_transverse",[-.90,-1.10,.48],[-.20,1.10,.60]),
        ("conveyor_longitudinal",[-3.70,-1.10,.48],[-.90,-.40,.60])]:
        np.testing.assert_allclose(boxes[name].corners().min(axis=0),lo,atol=1e-12)
        np.testing.assert_allclose(boxes[name].corners().max(axis=0),hi,atol=1e-12)
    assert all(c["status"]=="PASS" for c in audit_layout_constraints(layout)["checks"].values())
    np.testing.assert_allclose(layout.robot_base_transform()[:3,3],[-1.325,.35,.6],atol=1e-12)


def test_rigid_length_seam_and_fixed_rail(layout):
    initial=coupled_conveyor_state(layout,.6)
    fixed={b.name:box_message(b) for b in initial.boxes if b.name in initial.mechanism["fixed_structure_ids"]}
    for h,lower_x in [(.6,-3.7),(1.75,-3.452939482),(2.05,-3.2953079139)]:
        state=coupled_conveyor_state(layout,h);m=state.mechanism
        np.testing.assert_allclose(m["near_tip_world"],[-.9,-.75,h],atol=1e-12)
        np.testing.assert_allclose(m["rear_hinge_world"],[lower_x,-.75,.6],atol=1e-8)
        assert not m["rear_fixed"]
        assert np.linalg.norm(np.array(m["near_tip_world"])-m["rear_hinge_world"])==pytest.approx(2.8)
        boxes={b.name:b for b in state.boxes}
        long=boxes["conveyor_longitudinal"]
        np.testing.assert_allclose(long.center+long.rotation@[1.4,0,.06],m["near_tip_world"])
        for name,value in fixed.items():assert box_message(boxes[name])==value
        assert len(boxes)==len(state.boxes)
        for b in state.boxes:assert box_message(message_box(box_message(b)))==box_message(b)
        assert solid_signed_gap(boxes["conveyor_transverse"],long)>=-1e-9
        # Every lower support foot is inside the finite original floor.
        for b in state.boxes:
            if b.name.startswith("lift_lower_rail_") and "_stand_" in b.name:
                assert b.corners()[:,0].min()>=-3.2
        assert tuple(sorted(("conveyor_transverse","conveyor_longitudinal"))) not in assembly_mates(state)


@pytest.mark.parametrize("a,b",[(.6,2.05),(2.05,.6),(.6,1.75),(1.75,.6),(1.75,2.05),(2.05,1.75),(.6,.6),(1.75,1.75),(2.05,2.05)])
def test_full_discrete_stroke(layout,a,b):
    r=check_coupled_interval(layout,a,b,designed_contacts=CONTACTS)
    assert all(s["passed"] for s in r["samples"]), [(s["height_m"],[(f["pair"],f["signed_sat_separation_m"]) for f in s["failures"]]) for s in r["samples"] if not s["passed"]]
    assert r["maximum_height_step_m"]<=.010000000001
    assert not r["continuous_swept_proof"]


def test_relief_is_actual_solid_and_penetration_is_detected(layout):
    boxes={b.name:b for b in coupled_conveyor_state(layout,2.05).boxes}
    a,b=boxes["conveyor_transverse"],boxes["conveyor_longitudinal"]
    assert isinstance(a,ConceptSolid)
    assert a.signed_distance_obb(b)<-.01  # the old full box would penetrate
    assert solid_signed_gap(a,b)>=-1e-9
    assert solid_signed_gap(a,replace(b,center=b.center+np.array([.02,0,0])))<-.001
    assert box_message(a)["solid"]["vertices_m"]==a.solid["vertices_m"]


@pytest.mark.parametrize("h",[float("nan"),float("inf"),.59,2.1001,3.4])
def test_range_fails_without_clamp(layout,h):
    with pytest.raises(ValueError):coupled_conveyor_state(layout,h)


def test_new_region_recomputes_all_front_options(layout):
    cartons=layout.cartons();frozen=FrozenTopLayer.capture(cartons)
    belt=next(b for b in layout.fixed_components() if b.name=="conveyor_transverse")
    for b in cartons:
        if b.name not in frozen.target_ids:continue
        options=process_options(b,belt,frozen,cartons)
        assert options[0]["template"]=="FRONT_DIRECT_HMAX"
        assert options[0]["segment_directions_world"]==[[-1.,0.,0.]]
