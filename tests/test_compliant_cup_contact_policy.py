from __future__ import annotations

from dataclasses import replace

import pytest

from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.isaac_collision_policy import classify_compliant_cup_contact


CUP = "/Robot/J6/Tool/SoftCup17"
ROBOT = "/Robot/J6"
TARGET = "/Scene/target"
NEIGHBOR = "/Scene/neighbor"
TARGET_REASON = "EXPECTED_TARGET_COMPLIANT_CUP_CONTACT"
STACK_REASON = "ALLOWED_INACTIVE_COMPLIANT_CUP_STACK_CONTACT"
POLICY = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")


def classify(**overrides):
    values = dict(collider0=CUP, collider1=TARGET, actor0=ROBOT, actor1=TARGET,
                  compliant_cup_index_by_path={CUP: 17}, commanded_mask=[False] * 72,
                  target_path=TARGET, stack_paths={NEIGHBOR}, stage="contact", attached=False,
                  actual_free_space=False, minimum_separation_m=-.005,
                  policy=POLICY, physical_compression_m=.010)
    values.update(overrides)
    return classify_compliant_cup_contact(**values)


@pytest.mark.parametrize("active", [False, True])
@pytest.mark.parametrize("separation", [-.005, -.001, 0, .02])
def test_known_target_cup_contact_and_proximity_use_actual_compression(active, separation):
    mask = [False] * 72
    mask[17] = active
    assert classify(commanded_mask=mask, minimum_separation_m=separation) == TARGET_REASON


def test_pair_order_is_symmetric_but_shape_and_actor_ownership_are_exact():
    assert classify(collider0=TARGET, actor0=TARGET, collider1=CUP, actor1=ROBOT) == TARGET_REASON
    for unknown in (CUP + "/fake", "/Robot/J6/Tool/RigidInsert17", "/Robot/J5/collision", "/Scene/Tool/SoftCup17"):
        assert classify(collider0=unknown) is None
    assert classify(actor0="/Robot/J6_wrong") is None
    assert classify(collider1="/Scene/target_fake/shape") is None
    assert classify(collider1=CUP, actor1=ROBOT) is None
    assert classify(collider1="/Scene/wall", actor1="/Scene/wall") is None


@pytest.mark.parametrize("stage", ["support-release", "extraction", "transit", "preplace", "place"])
def test_target_contact_continues_only_during_actual_attached_lifecycle(stage):
    assert classify(stage=stage, attached=True, actual_free_space=True) == TARGET_REASON
    assert classify(stage=stage, attached=False) is None


@pytest.mark.parametrize("stage", ["release", "released", "withdrawal", "post-release"])
def test_release_stage_cannot_reuse_target_contact_permission(stage):
    assert classify(stage=stage, attached=False) is None
    assert classify(stage=stage, attached=True) is None


@pytest.mark.parametrize("stage,attached", [("contact", False), ("support-release", True), ("extraction", True)])
def test_inactive_neighbor_cup_contact_is_stage_scoped_without_physics_filter(stage, attached):
    for separation in (-.005, 0, .02):
        assert classify(collider1=NEIGHBOR, actor1=NEIGHBOR, stage=stage,
                        attached=attached, minimum_separation_m=separation) == STACK_REASON


def test_active_neighbor_free_space_wrong_mode_and_unattached_extraction_are_rejected():
    neighbor = dict(collider1=NEIGHBOR, actor1=NEIGHBOR)
    active = [False] * 72
    active[17] = True
    assert classify(**neighbor, commanded_mask=active) is None
    assert classify(**neighbor, actual_free_space=True) is None
    assert classify(**neighbor, stage="transit", attached=True) is None
    assert classify(**neighbor, stage="extraction", attached=False) is None
    assert classify(**neighbor, policy=replace(POLICY, inactive_compliant_cup_stack_contact_mode="reject")) is None
    assert classify(**neighbor, stack_paths={TARGET}) is None


@pytest.mark.parametrize("separation", [-.00500001, None, float("nan"), float("inf"), -float("inf")])
def test_missing_nonfinite_or_excess_compression_never_grants_contact(separation):
    assert classify(minimum_separation_m=separation) is None
    assert classify(collider1=NEIGHBOR, actor1=NEIGHBOR, minimum_separation_m=separation) is None


def test_neighbor_uses_smaller_policy_and_physical_compression_bounds():
    neighbor = dict(collider1=NEIGHBOR, actor1=NEIGHBOR)
    tighter = replace(POLICY, maximum_compliant_cup_additional_compression_m=.004)
    assert classify(**neighbor, policy=tighter, minimum_separation_m=-.004) == STACK_REASON
    assert classify(**neighbor, policy=tighter, minimum_separation_m=-.00400001) is None
    assert classify(**neighbor, physical_compression_m=.003, minimum_separation_m=-.004) is None
    assert classify(physical_compression_m=.003, minimum_separation_m=-.004) is None
    assert classify(physical_compression_m=float("nan")) is None


def test_invalid_cup_index_or_command_cannot_be_interpreted_as_an_inactive_cup():
    for index in (-1, 72, 17.0, True):
        assert classify(compliant_cup_index_by_path={CUP: index}) is None
    wrong = [False] * 72
    wrong[17] = "false"
    assert classify(commanded_mask=wrong) is None


@pytest.mark.parametrize("stage", ["place", "release", "withdrawal"])
def test_removed_constraint_release_clearance_window_is_target_only(stage):
    release = dict(stage=stage, attached=False, release_validation_pending=True)
    assert classify(**release) == "EXPECTED_TARGET_COMPLIANT_CUP_RELEASE_CLEARANCE"
    assert classify(**release, minimum_separation_m=.02) == "EXPECTED_TARGET_COMPLIANT_CUP_RELEASE_CLEARANCE"
    assert classify(**release, collider1=NEIGHBOR, actor1=NEIGHBOR) is None
    assert classify(**release, collider0="/Robot/J6/Tool/RigidInsert17") is None
    assert classify(**release, minimum_separation_m=-.00500001) is None
    assert classify(**release, minimum_separation_m=None) is None
    # The caller closes this latch only after target/cup proximity is clear
    # (or terminates on its independent timeout), not on independence alone.
    assert classify(stage=stage, attached=False, release_validation_pending=False) is None


def test_release_validation_requires_deleted_constraint_and_a_real_boolean():
    assert classify(attached=True, release_validation_pending=True) is None
    assert classify(release_validation_pending="true") is None


@pytest.mark.parametrize("active", [False, True])
def test_target_and_release_share_five_mm_additional_compression_budget(active):
    mask = [False] * 72
    mask[17] = active
    assert classify(commanded_mask=mask, minimum_separation_m=-.005) == TARGET_REASON
    assert classify(commanded_mask=mask, minimum_separation_m=-.00500001) is None
    release = dict(commanded_mask=mask, stage="withdrawal", attached=False,
                   release_validation_pending=True)
    assert classify(**release, minimum_separation_m=-.005) == "EXPECTED_TARGET_COMPLIANT_CUP_RELEASE_CLEARANCE"
    assert classify(**release, minimum_separation_m=-.00500001) is None
