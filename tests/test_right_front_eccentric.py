"""Authorization is frozen geometry, independent of names and receiving regions."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.geometry import OBB
from unloading_sim.support import SupportRelationGraph
from unloading_sim.top_layer_lifted import FrozenTopLayer
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.right_front_eccentric import eccentric_target, offset_intervals, candidate_deltas


def fixture(names=("right", "junction", "middle", "left")):
    boxes = tuple(OBB([1., y, 2.], [.3, .25, .15], np.eye(3), n, "carton")
                  for n, y in zip(names, (-.9, -.3, .3, .9)))
    frozen = FrozenTopLayer.capture(boxes)
    scene = SimpleNamespace(support_graph=SupportRelationGraph.build(boxes),
        snapshot={"scene_fingerprint": "frozen-scene"},
        policy=SimpleNamespace(data={"task_population": {"face_modes": ["front","top","side"]}}))
    targets = [simulation_contact_targets(scene, b, [0,0,0], .1)[0][0] for b in boxes]
    return frozen, targets


def test_original_right_only_and_rename():
    for names in (("right","junction","middle","left"), ("renamed","c00","c01","c04")):
        frozen, targets = fixture(names)
        assert eccentric_target(targets[0], frozen, .1, {}).validate()
        for target in targets[1:]:
            with pytest.raises(ValueError, match="ORIGINAL_RIGHT"):
                eccentric_target(target, frozen, .1, {})


def test_removed_right_does_not_reauthorize_or_trust_label():
    frozen, targets = fixture()
    # The surviving minimum-Y carton remains the original middle role.
    for target in targets[1:]:
        with pytest.raises(ValueError, match="ORIGINAL_RIGHT"):
            eccentric_target(target, frozen, .1, {})
    import json
    data = frozen.evidence()
    data["roles"]["junction"]["role"] = "RIGHT_END"
    forged = replace(frozen, evidence_json=json.dumps(data))
    with pytest.raises(ValueError):
        eccentric_target(targets[1], forged, .1, {})


@pytest.mark.parametrize("delta", [-.01, float("nan"), float("inf")])
def test_invalid_delta(delta):
    frozen, targets = fixture()
    with pytest.raises(ValueError):
        eccentric_target(targets[0], frozen, delta, {})


def test_real_face_center_and_authorized_outside_reference():
    frozen, targets = fixture()
    centered = targets[0]
    shifted = eccentric_target(centered, frozen, .35, {})
    assert shifted.contact_pose[1,3] > centered.box.center[1]+.25
    np.testing.assert_array_equal(shifted.face_center, centered.face_center)
    np.testing.assert_allclose(shifted.contact_pose[:3,3]-centered.contact_pose[:3,3], [0,.35,0])
    np.testing.assert_allclose(shifted.approach_pose[:3,3]-centered.approach_pose[:3,3], [0,.35,0])
    assert 0 < sum(shifted.predicted_eligible_mask) < 72
    assert len(shifted.predicted_eligible_mask) == 72
    assert shifted.candidate_id != centered.candidate_id
    # The normal unprivileged validator still rejects outside references.
    with pytest.raises(ValueError):
        replace(centered, contact_pose=shifted.contact_pose, approach_pose=shifted.approach_pose).validate()
    with pytest.raises(ValueError):
        eccentric_target(centered, frozen, 1., {})


@pytest.mark.parametrize("change", ["x", "z", "roll", "top", "missing_authority"])
def test_candidate_cannot_change_authorized_degree_of_freedom(change):
    frozen, targets = fixture()
    target = eccentric_target(targets[0], frozen, .1, {})
    pose = target.contact_pose.copy(); approach = target.approach_pose.copy()
    kwargs = {}
    if change in ("x","z"):
        index = 0 if change == "x" else 2
        pose[index,3] += .01; approach[index,3] += .01
    elif change == "roll":
        pose[:3,:2] *= -1; approach[:3,:2] *= -1
        kwargs["in_plane_x"] = pose[:3,0]
    elif change == "top":
        kwargs["face_role"] = "TOP"
    else:
        kwargs["eccentric_authority"] = None
    with pytest.raises(ValueError):
        replace(target, contact_pose=pose, approach_pose=approach, **kwargs).validate()


def test_finite_geometry_domain_center_first_and_nonconstant_candidates():
    frozen, targets = fixture()
    intervals = offset_intervals(targets[0], frozen, {})
    assert intervals and max(b for a,b in intervals) == pytest.approx(.4925)
    deltas = candidate_deltas(intervals)
    assert next(deltas) == 0.
    values = list(deltas)
    assert len(set(values)) > 3 and all(0 < d <= .4925 for d in values)


def test_incremental_opportunities_after_center_or_suffix_failure():
    from unloading_sim.right_front_eccentric import candidate_visits
    options=[dict(height_m=1.75,pre_approach_height_m=h) for h in (1.75,1.70,1.65)]
    visits=candidate_visits(options,((0.,.4925),))
    first=[next(visits) for _ in range(6)]
    assert first[0]["world_y_offset_m"]==0.
    assert first[1]["world_y_offset_m"]>0.
    assert first[2]["pre_approach_height_m"]==1.70
    assert len({(r["world_y_offset_m"],r["pre_approach_height_m"]) for r in first})==6
    # Rejection doesn't consume the iterator or mutate previous candidates.
    assert next(visits)["world_y_offset_m"] != first[1]["world_y_offset_m"]
    assert options[0]==dict(height_m=1.75,pre_approach_height_m=1.75)


def test_actual_masks_and_eccentric_attachment_keep_original_box_pose():
    from unloading_sim.independent_cups import (audit_actual_independent_cup_contacts,
        m710_cup_array_from_mapping, evaluate_independent_cup_geometry)
    from unloading_sim.validation_physics import RigidAttachment
    frozen,targets=fixture();target=eccentric_target(targets[0],frozen,.35,{})
    array=m710_cup_array_from_mapping({})
    ids=[c.cup_id for c,enabled in zip(array.cups,target.predicted_eligible_mask) if enabled]
    def audit(pose):
        g=evaluate_independent_cup_geometry(pose,target.box,"front",array,
            max_attachment_gap_m=.002,maximum_penetration_m=.002,
            max_normal_misalignment_rad=np.deg2rad(5.))
        return audit_actual_independent_cup_contacts(g,ids,pose_source="TEST_ACTUAL_FK")
    assert audit(target.contact_pose).actual_contact_mask==target.predicted_eligible_mask
    miss=target.contact_pose.copy();miss[0,3]-=.01
    assert audit(miss).actual_contact_mask != target.predicted_eligible_mask
    assert audit(miss).commanded_active_mask == target.predicted_eligible_mask
    rigid=RigidAttachment.capture(target.contact_pose,target.box)
    np.testing.assert_allclose(target.contact_pose@rigid.tcp_from_box,target.box.world_from_local)
    withdrawn=target.contact_pose.copy();withdrawn[0,3]-=.8
    pose=withdrawn@rigid.tcp_from_box
    assert pose[1,3]==pytest.approx(target.box.center[1])
    np.testing.assert_allclose(pose[:3,:3],target.box.rotation)


def test_acceptance_revalidates_frozen_authority_geometry_and_masks():
    from copy import deepcopy
    from unloading_sim.right_front_eccentric import validate_accepted_target
    frozen,targets=fixture();t=eccentric_target(targets[0],frozen,.35,{})
    mask=list(t.predicted_eligible_mask)
    plan=dict(target=t.evidence(),option=dict(segment_directions_world=[[-1.,0.,0.]]),
        eccentric_contact_validation=dict(suction={},expected=dict(geometrically_eligible_mask=mask,
            commanded_active_mask=mask),actual=dict(commanded_active_mask=mask,actual_contact_mask=mask)))
    boxes=[x.box for x in targets]
    validate_accepted_target(plan,frozen,boxes)
    for change in ("authority","mask","delta","face","lateral"):
        bad=deepcopy(plan)
        if change=="authority":bad["target"]["eccentric_authorization"]=None
        if change=="mask":bad["eccentric_contact_validation"]["actual"]["actual_contact_mask"]=[False]*72
        if change=="delta":bad["target"]["world_y_offset_m"]=-.1
        if change=="face":bad["target"]["face_center_m"][1]+=.1
        if change=="lateral":bad["option"]["segment_directions_world"]=[[0.,1.,0.],[-1.,0.,0.]]
        with pytest.raises(ValueError):validate_accepted_target(bad,frozen,boxes)


def test_eccentric_does_not_change_full_box_junction_or_hmin():
    from pathlib import Path
    from unloading_sim.workcell_layout import load_workcell_layout
    from unloading_sim.conveyor_lift import coupled_conveyor_state
    from unloading_sim.lifted_reception import translated, StartHeight
    from unloading_sim.junction_reception import reception_classification, require_receive_height
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/
        "configs/workcells/m710id70_unloading_layout_v1_transverse_scissor_026.yaml")
    boxes=layout.cartons();frozen=FrozenTopLayer.capture(boxes)
    box=next(b for b in boxes if b.name==frozen.target_ids[0])
    scene=SimpleNamespace(support_graph=SupportRelationGraph.build(boxes),
        snapshot={"scene_fingerprint":"s"},policy=SimpleNamespace(data={"task_population":{"face_modes":["front"]}}))
    target=simulation_contact_targets(scene,box,[-1.3,0,.6],.1)[0][0]
    shifted=eccentric_target(target,frozen,.35,{})
    assert StartHeight.capture(shifted.box).hmin == pytest.approx(1.75)
    world=coupled_conveyor_state(layout,1.75,boxes).boxes;by={b.name:b for b in world}
    received=translated(shifted.box,[-.85,0,0])
    assert reception_classification(received,by["conveyor_transverse"],by["conveyor_longitudinal"])["junction"]
    require_receive_height(world,1.75)
    with pytest.raises(ValueError):
        require_receive_height(coupled_conveyor_state(layout,1.70,boxes).boxes,1.75)
