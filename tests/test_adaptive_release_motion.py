import copy

import numpy as np
import pytest

from unloading_sim.conveyor_placement import placement_working_normal, support_union_audit
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.release_motion import (MOTION_SEMANTICS, ReleasePolicy, SHORT_DROP_RELEASE,
    SUPPORTED_RELEASE, departure_sweep, flight_box, predict_release, rigid_com_velocity)
from unloading_sim.layout_trajectory import LayoutTrajectoryBudget, validate_layout_trajectory_stage_contract
from unloading_sim.collision_policy import PhysicsCheckedStackTracker, SimulationCollisionPolicy


def deck():
    return OBB([0, 0, .3], [1, 1, .3], np.eye(3), "belt", "conveyor")


def carton(height=.025, rotation=None):
    rotation = np.eye(3) if rotation is None else rotation
    half = np.array([.3, .2, .15])
    extent = float(np.abs(rotation[2]) @ half)
    return OBB([0, 0, .6 + height + extent], half, rotation, "box", "carton")


def test_working_normals_are_literal_world_semantics_and_cannot_be_mutated():
    assert np.array_equal(placement_working_normal("TRANSVERSE_SIDE"), [1, 0, 0])
    assert np.array_equal(placement_working_normal("RIGHT_WALL_FACING"), [0, -1, 0])
    assert np.array_equal(placement_working_normal("TOP_DOWN"), [0, 0, -1])
    placement_working_normal("TRANSVERSE_SIDE")[0] = -1
    assert placement_working_normal("TRANSVERSE_SIDE")[0] == 1


def test_release_keeps_predicted_landing_separate_from_actual_support():
    result = predict_release(carton(), [deck()], mode=SHORT_DROP_RELEASE)
    assert result["accepted"]
    assert not result["actual_support"]["supported"]
    assert result["actual_landing_state"] is None
    assert result["flight_time_s"] == pytest.approx(.071392156, abs=1e-8)
    assert result["height_m"] == pytest.approx(.025)
    assert result["predicted_landing_pose_world"][2][3] == pytest.approx(.75)
    assert not predict_release(carton(), [deck()], mode=SUPPORTED_RELEASE)["accepted"]
    assert predict_release(carton(0), [deck()], mode=SUPPORTED_RELEASE)["accepted"]


def test_rigid_com_velocity_includes_angular_offset_and_flight_does_not_take_belt_speed():
    v = rigid_com_velocity([.1, 0, 0], [0, 0, 2], [0, 0, 0], [0, .3, 0])
    assert np.allclose(v, [-.5, 0, 0])
    initial = carton()
    after = flight_box(initial, [.1, 0, 0], [0, 0, .05], .02)
    assert np.allclose(after.center, [.002, 0, .773038])
    assert np.array_equal(after.half_extents, initial.half_extents)
    assert after.name == initial.name
    assert not np.allclose(after.rotation, initial.rotation)


def test_rotated_carton_uses_actual_bottom_face():
    result = predict_release(carton(rotation=rotation_matrix_from_rpy(0, np.pi/2, 0)),
                             [deck()], mode=SHORT_DROP_RELEASE)
    assert result["accepted"]
    assert result["landing_support"]["support_face_local_axis"] == 0
    assert result["predicted_landing_pose_world"][2][3] == pytest.approx(.9)


@pytest.mark.parametrize("height,omega,reason", [(.051, 0, "RELEASE_HEIGHT_OUT_OF_BOUNDS"),
    (.025, .2, "RELEASE_ANGULAR_SPEED_OUT_OF_MODEL")])
def test_short_drop_bounds_fail_closed(height, omega, reason):
    result = predict_release(carton(height), [deck()], mode=SHORT_DROP_RELEASE,
                             angular_velocity=[0, 0, omega])
    assert not result["accepted"] and result["reason"] == reason


def test_drop_rejects_full_footprint_overhang_and_moving_landing_obstacle():
    box = carton()
    box = OBB([.69, 0, box.center[2]], box.half_extents, box.rotation, box.name, box.category)
    result = predict_release(box, [deck()], mode=SHORT_DROP_RELEASE, linear_velocity=[.3, 0, 0])
    assert not result["accepted"]
    occupied = OBB([0, 0, .76], [.1, .1, .1], np.eye(3), "occupied", "carton")
    result = predict_release(carton(), [deck()], mode=SHORT_DROP_RELEASE, obstacles=[occupied])
    assert result["reason"] == "PREDICTED_FLIGHT_COLLISION"


def test_departure_sweep_covers_stationary_and_between_sample_occupancy():
    box = carton(0)
    sweep = departure_sweep(box, [0, -1, 0], distance_m=.11, resolution_m=.04)
    assert np.allclose(sweep[0].center, box.center)
    assert np.allclose(sweep[-1].center, [0, -.11, .75])
    for y in np.linspace(-.11, 0, 51):
        assert any(abs(y-b.center[1]) <= b.half_extents[1] for b in sweep)


def test_no_mandatory_withdrawal_or_vertical_lift():
    budget = LayoutTrajectoryBudget()
    assert budget.withdrawal_distance_m == budget.post_release_vertical_lift_m == 0.
    with pytest.raises(ValueError):
        LayoutTrajectoryBudget(approach_mode="teleport")


def test_stack_contact_permission_follows_physical_release_and_latches():
    policy = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")
    target = OBB([0, 0, .5], [.3, .2, .15], np.eye(3), "target", "carton")
    neighbor = OBB([0, .4, .5], [.3, .2, .15], np.eye(3), "neighbor", "carton")
    tracker = PhysicsCheckedStackTracker(target, [neighbor], policy, .01, .0002)
    turned = OBB([-.02, -.01, .5], target.half_extents, rotation_matrix_from_rpy(0, 0, .03), "target", "carton")
    assert tracker.state_failure(turned, [neighbor]) is None
    assert not tracker.fully_released
    free = OBB([0, -.1, .5], target.half_extents, target.rotation, "target", "carton")
    assert tracker.state_failure(free, [neighbor]) is None and tracker.fully_released
    assert tracker.clone().state_failure(target, [neighbor])["reason"] == "PAYLOAD_COLLISION"


def test_drop_export_and_runtime_keep_release_and_landing_centers_distinct():
    from unloading_sim.isaac_bridge import _validated_place_evidence
    from unloading_sim.m710_replay_physics import audit_runtime_short_drop
    box, belt = carton(), deck()
    prediction = predict_release(box, [belt], mode=SHORT_DROP_RELEASE)
    segment = {"motion_semantics": MOTION_SEMANTICS, "target": "box",
        "place": {"receiver": "belt", "place_surface": "belt", "support_names": ["belt"],
            "load_bearing_support_names": ["belt"], "release_mode": SHORT_DROP_RELEASE,
            "release_prediction": prediction, "actual_box_pose_world": box.world_from_local.tolist(),
            "release_center_world_m": box.center.tolist(), "support": prediction["actual_support"]}}
    primitive = lambda b, dynamic: {"name": b.name, "size_m": (b.half_extents*2).tolist(),
        "center_m": b.center.tolist(), "rotation_matrix": b.rotation.tolist(),
        "dynamic": dynamic, "category": b.category}
    primitives = [primitive(box, True), primitive(belt, False)]
    with pytest.raises(ValueError, match="current scene is required"):
        _validated_place_evidence(segment)
    evidence = _validated_place_evidence(segment, scene_primitives=primitives)
    assert evidence["release_center_m"][2] == pytest.approx(.775)
    assert evidence["place_center_m"][2] == pytest.approx(.75)
    metadata = {"target": "box", "release_prediction": prediction, "selected_place_support_names": ["belt"],
        "scene_primitives": [primitive(box, True), primitive(belt, False)]}
    state = dict(position=box.center, rotation=box.rotation, linear_velocity=[0, 0, 0],
                 angular_velocity=[0, 0, 0], current_cartons=[])
    assert audit_runtime_short_drop(metadata, **state)["accepted"]
    state["linear_velocity"] = [4, 0, 0]
    assert not audit_runtime_short_drop(metadata, **state)["accepted"]
    assert state["linear_velocity"] == [4, 0, 0]  # audit cannot zero actual velocity
    state["position"] = [0, 0, .8]
    assert audit_runtime_short_drop(metadata, **state)["reason"] == "ACTUAL_RELEASE_REGION_MISMATCH"


def test_controller_export_retains_checked_joint_corners_without_inserting_dwell():
    from unloading_sim.isaac_bridge import _sample_trajectory
    times = np.array([0., .033, .067])
    points = np.array([[0, 0], [.1, 0], [.1, .1]])
    sample_t, sample_q = _sample_trajectory(times, points, .02)
    assert .033 in sample_t
    assert np.array_equal(sample_q[np.flatnonzero(sample_t == .033)[0]], [.1, 0])
    assert np.all(np.diff(sample_t) > 0)
    assert not np.any(np.all(np.diff(sample_q, axis=0) == 0, axis=1))


def test_retained_carton_sweep_reaches_bounded_outlet_without_changing_its_body():
    from unloading_sim.release_motion import retained_receiver_envelope, receiver_outlet_clearance
    receiver = OBB([-1.6, -.75, .54], [1.4, .35, .06], np.eye(3), "long", "conveyor")
    box = OBB([-.5, -.75, .8], [.3, .2, .2], np.eye(3), "opaque-id", "carton")
    envelope = retained_receiver_envelope(box, receiver, [-1, 0, 0])
    assert receiver_outlet_clearance(box, receiver, [-1, 0, 0]) == pytest.approx(2.2)
    assert np.allclose(envelope.center, [-1.55, -.75, .8])
    assert np.allclose(envelope.half_extents, [1.36, .2, .2])
    assert np.array_equal(box.half_extents, [.3, .2, .2])
    assert np.array_equal(retained_receiver_envelope(box, receiver, [-1, 0, 0], held=True).center, box.center)


def test_union_landing_does_not_prove_full_support_along_the_selected_belt():
    from unloading_sim.release_motion import receiver_transport_support
    long = OBB([-1.6, -.75, .54], [1.4, .35, .06], np.eye(3), "long", "conveyor")
    cross = OBB([-.55, .35, .54], [.35, .75, .06], np.eye(3), "cross", "conveyor")
    seam_box = OBB([-.55, -.59, .8], [.15, .2, .2], np.eye(3), "payload", "carton")
    assert support_union_audit(seam_box, [long, cross])["supported"]
    assert not receiver_transport_support(seam_box, long, [long, cross], [-1, 0, 0])["accepted"]
    inside = OBB([-.55, -.75, .8], seam_box.half_extents, seam_box.rotation, "payload", "carton")
    assert receiver_transport_support(inside, long, [long, cross], [-1, 0, 0])["accepted"]


def test_receiver_reserve_checks_union_outer_boundary_without_inventing_a_seam_gap():
    from unloading_sim.release_motion import receiver_footprint_reserve
    left = OBB([-.5, 0, .3], [.5, 1, .3], np.eye(3), "left", "conveyor")
    right = OBB([.5, 0, .3], [.5, 1, .3], np.eye(3), "right", "conveyor")
    box = carton(0)
    assert receiver_footprint_reserve(box, [left, right], .01)["supported"]
    rim = OBB([.7, 0, .75], box.half_extents, box.rotation, box.name, box.category)
    assert support_union_audit(rim, [left, right])["supported"]
    assert not receiver_footprint_reserve(rim, [left, right], .01)["supported"]


def test_sampling_generates_an_interior_candidate_before_spending_ik_budget():
    from unloading_sim.conveyor_placement import PlacementPolicy, generate_conveyor_placements
    policy = PlacementPolicy(yaw_offsets_rad=(0.,), orientation_rpy_offsets_rad=((0., 0., 0.),),
                             sampling_edge_reserve_m=.01)
    candidate = generate_conveyor_placements(carton(0), [deck()],
        preferred_point_world=[9., 9., .75], policy=policy)[0]
    assert np.allclose(candidate.payload.center, [.69, .79, .75])
    assert np.array_equal(candidate.payload.half_extents, [.3, .2, .15])


def test_already_received_carton_cannot_inherit_stack_contact_relaxation():
    from types import SimpleNamespace
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
    policy = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")
    robot = SimpleNamespace(dof=6, joint_limits=np.tile([-np.pi, np.pi], (6, 1)),
                            tool_collision_obbs=lambda q: [])
    connector = LayoutTrajectoryConnector(robot, lambda *a, **kw: None,
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4), ik_policy={},
        collision_margin_m=.01, contact_tolerance_m=.0002, joint_margin_rad=.01,
        maximum_jacobian_condition=1e4, validator_identity="scope_regression", execution_qualified=True,
        collision_policy=policy.to_mapping())
    target = OBB([0, 0, .5], [.3, .2, .15], np.eye(3), "target", "carton")
    received = OBB([0, .401, .5], [.3, .2, .15], np.eye(3), "received", "carton")
    connector.stack_carton_names = {target.name}
    tracker, failure = connector._initial_proximity(target, [received], ())
    assert failure is None and tracker.fully_released
    assert tracker.state_failure(target, [received])["reason"] == "PAYLOAD_COLLISION"
