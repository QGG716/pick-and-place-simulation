import numpy as np
import pytest

from unloading_sim.collision_policy import SimulationCollisionPolicy, PhysicsCheckedStackTracker
from unloading_sim.geometry import OBB


def test_exemption_is_owned_tool_pair_only_and_identity_changes():
    policy = SimulationCollisionPolicy(wrist_tool_exempt_links=("J5_link", "J6_link"),
                                      stack_contact_mode="planner_relaxed_physics_checked")
    pairs = policy.wrist_tool_pairs(["plate", "insert"])
    assert ("J5_link", "plate") in pairs
    assert ("J6_link", "insert") in pairs
    assert ("J5_link", "carton") not in pairs
    assert ("J4_link", "plate") not in pairs
    assert SimulationCollisionPolicy.from_mapping(policy.to_mapping()) == policy
    assert policy.to_mapping()["inactive_compliant_cup_stack_contact_mode"] == (
        "physical_contact_within_compression"
    )
    assert policy.to_mapping()["maximum_compliant_cup_additional_compression_m"] == pytest.approx(.005)
    assert policy.fingerprint != SimulationCollisionPolicy().fingerprint
    with pytest.raises(ValueError):
        SimulationCollisionPolicy(wrist_tool_exempt_links=("J4_link",))
    with pytest.raises(ValueError, match="inactive compliant"):
        SimulationCollisionPolicy(inactive_compliant_cup_stack_contact_mode="ignore_all")


def test_stack_touch_slide_minor_disturbance_and_gross_crossing():
    policy = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")
    neighbor = OBB([0.3, 0, 0.15], [0.3, 0.2, 0.15], np.eye(3), name="support", category="carton")
    target = OBB([0.3, 0, 0.45], [0.3, 0.2, 0.15], np.eye(3), name="target", category="carton")
    tracker = PhysicsCheckedStackTracker(target, [neighbor], policy, .01, .0002)
    assert tracker.state_failure(target, [neighbor]) is None
    sliding = OBB([.2, 0, .449], target.half_extents, np.eye(3), name=target.name)
    assert tracker.state_failure(sliding, [neighbor]) is None
    crossing = OBB([.2, 0, .40], target.half_extents, np.eye(3), name=target.name)
    assert tracker.state_failure(crossing, [neighbor])["reason"] == "GROSS_PLANNED_STACK_PENETRATION"
    released = OBB([-.33, 0, .45], target.half_extents, np.eye(3), name=target.name)
    assert tracker.state_failure(released, [neighbor]) is None
    assert tracker.fully_released
    assert policy.allows_stack_planning_contact("extraction")
    assert not policy.allows_stack_planning_contact("transit")
    with pytest.raises(ValueError):
        SimulationCollisionPolicy(stack_contact_stages=("transit",))


def test_non_stack_obstacle_keeps_normal_margin_during_extraction():
    policy = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")
    target = OBB([0, 0, .6], [.3, .2, .15], np.eye(3), name="target")
    fixture = OBB([.61, 0, .6], [.3, .2, .15], np.eye(3), name="belt_side", category="conveyor")
    tracker = PhysicsCheckedStackTracker(target, [], policy, .01, .0002)
    assert tracker.state_failure(target, [fixture])["reason"] == "PAYLOAD_COLLISION"
