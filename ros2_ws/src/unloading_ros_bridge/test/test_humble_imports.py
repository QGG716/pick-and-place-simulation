import os
import sys


def test_humble_python_and_generated_interfaces_import():
    assert os.environ.get("ROS_DISTRO") == "humble"
    assert sys.version_info[:2] == (3, 10)
    import rclpy  # noqa: F401
    from unloading_interfaces.msg import (  # noqa: F401
        CargoObservation, ControllerStopFact, ExecutionAuthorization, ExecutionCancel,
        ExecutionEvent, PerceptionObservation, PlanningWorldSnapshot, StopAcknowledgement,
    )


def test_cpu_bridge_import_does_not_load_cuda_frameworks():
    import unloading_ros_bridge.common  # noqa: F401
    assert "torch" not in sys.modules
    assert "ultralytics" not in sys.modules
