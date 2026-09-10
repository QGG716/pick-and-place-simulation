import os
import sys


def test_humble_python_and_generated_interfaces_import():
    assert os.environ.get("ROS_DISTRO") == "humble"
    assert sys.version_info[:2] == (3, 10)
    import rclpy  # noqa: F401
    from unloading_interfaces.msg import (  # noqa: F401
        CargoObservation, ControllerStopFact, ExecutionAuthorization, ExecutionCancel,
        ExecutionContext, ExecutionEvent, ExecutionGrant, PerceptionObservation,
        PlanningWorldSnapshot, StopAcknowledgement,
    )


def test_cpu_bridge_import_does_not_load_cuda_frameworks():
    import unloading_ros_bridge.common  # noqa: F401
    assert "torch" not in sys.modules
    assert "ultralytics" not in sys.modules


def test_domain_ros_domain_round_trip_preserves_fingerprint():
    from unloading_contracts import canonical_fingerprint
    from unloading_perception.demo import _assemble, _synthetic_observation
    from unloading_ros_bridge.mapping import observation_from_msg, observation_to_msg, snapshot_from_msg, snapshot_to_msg

    observation = _synthetic_observation()
    restored = observation_from_msg(observation_to_msg(observation))
    assert canonical_fingerprint(restored) == canonical_fingerprint(observation)
    update, assembly = _assemble(observation)
    message = snapshot_to_msg(assembly.snapshot, source_epoch=observation.source_epoch, source_capture_time=observation.capture_time, blocking_reasons=update.blocking_reasons, obstacles=update.accepted_obstacles, unknown_regions=update.unknown_regions)
    assert snapshot_from_msg(message).fingerprint == assembly.snapshot.fingerprint
