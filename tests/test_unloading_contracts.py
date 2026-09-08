from dataclasses import FrozenInstanceError

import pytest

from unloading_contracts import (
    CameraIntrinsics, EvidenceKind, PlanArtifactKind, Pose3D, ResourceReference,
    RobotStateRevision, SceneRevision, SensorFrame, TimedJointPoint,
    TimedJointTrajectory, Validity, canonical_fingerprint, dumps, loads,
)


def make_frame() -> SensorFrame:
    return SensorFrame(
        "camera", "rgb", "epoch-a", 3, 1.0, 1.1, "ros", "camera_optical",
        640, 480, "rgb8", ResourceReference("file:///frame.png"),
        intrinsics=CameraIntrinsics(0.8, 0.9, 0.5, 0.5, True, "model", "cal-1"),
    )


def test_contract_round_trip_preserves_type_and_enum_identity():
    original = make_frame()
    restored = loads(dumps(original))
    assert type(restored) is SensorFrame
    assert restored == original
    assert restored.depth_validity is Validity.UNKNOWN


def test_nested_caller_data_is_deeply_frozen_and_detached():
    state = {"joints": [1.0, 2.0], "nested": {"flag": True}}
    revision = RobotStateRevision(1, (1.0, 2.0), state)
    state["joints"][0] = 99.0
    assert revision.robot_state["joints"] == (1.0, 2.0)
    with pytest.raises(TypeError):
        revision.robot_state["new"] = 1


def test_missing_values_are_not_invented():
    frame = make_frame()
    assert frame.depth is None
    assert frame.intrinsics.pixels(640, 480).fx == pytest.approx(512.0)
    with pytest.raises(ValueError):
        Pose3D((0, 0, 0), (0, 0, 0, 0), "world", "columns", EvidenceKind.OBSERVED)


def test_timed_trajectory_requires_strict_time_and_explicit_provenance():
    points = (TimedJointPoint((0.0, 0.0), 0.0), TimedJointPoint((1.0, 2.0), 1.0))
    trajectory = TimedJointTrajectory(("j1", "j2"), points, PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, "fixture", "robot-sha", "config-sha")
    assert trajectory.points[-1].time_from_start == 1.0
    with pytest.raises(ValueError, match="strictly increasing"):
        TimedJointTrajectory(("j1", "j2"), (points[0], TimedJointPoint((1, 2), 0.0)), PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, "fixture", "robot", "config")


def test_scene_fingerprint_ignores_mapping_insertion_order_but_not_geometry():
    assert canonical_fingerprint({"a": 1, "b": [2, 3]}) == canonical_fingerprint({"b": [2, 3], "a": 1})
    assert canonical_fingerprint({"a": 1}) != canonical_fingerprint({"a": 2})
