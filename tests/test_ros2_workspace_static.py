import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[1]
ROS = ROOT / "ros2_ws" / "src"


def test_ros_packages_are_humble_compatible_and_parseable():
    packages = ("unloading_interfaces", "unloading_ros_bridge", "unloading_bringup")
    for package in packages:
        root = ET.parse(ROS / package / "package.xml").getroot()
        assert root.findtext("name") == package
    for path in (ROS / "unloading_ros_bridge" / "unloading_ros_bridge").glob("*.py"):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path), feature_version=(3, 10))
    ast.parse((ROS / "unloading_bringup" / "launch" / "replay_mock.launch.py").read_text(encoding="utf-8"), feature_version=(3, 10))


def test_ros_process_has_no_heavy_vision_or_cuda_imports():
    source = "\n".join(path.read_text(encoding="utf-8") for path in (ROS / "unloading_ros_bridge" / "unloading_ros_bridge").glob("*.py"))
    for forbidden in ("import torch", "import cv2", "import ultralytics", "import segment_anything", "import pybullet"):
        assert forbidden not in source
    assert "ROS_DISTRO" in source
    assert "Python 3.10" in source
    perception = (ROS / "unloading_ros_bridge" / "unloading_ros_bridge" / "perception_node.py").read_text(encoding="utf-8")
    assert "self.worker_pool = ThreadPoolExecutor" in perception
    assert "self.executor = ThreadPoolExecutor" not in perception


def test_optional_ros_fields_have_explicit_presence_flags():
    cargo = (ROS / "unloading_interfaces" / "msg" / "CargoObservation.msg").read_text(encoding="utf-8")
    for flag in ("has_detection_score", "has_contour_score", "has_reprojection_score", "has_geometry_error", "has_pose", "has_pose_covariance", "has_full_dimensions"):
        assert flag in cargo
    assert "trajectory_msgs/JointTrajectory trajectory" in (ROS / "unloading_interfaces" / "msg" / "ExecutionAuthorization.msg").read_text(encoding="utf-8")
    assert "planning_generation" in (ROS / "unloading_interfaces" / "msg" / "ExecutionCancel.msg").read_text(encoding="utf-8")
    bridge = (ROS / "unloading_ros_bridge" / "unloading_ros_bridge" / "execution_bridge_node.py").read_text(encoding="utf-8")
    assert '"/unloading/controller_stop_facts"' in bridge
    assert "acknowledgement.command_id" in bridge
    assert "ExecutionGate" in bridge
    world_bridge = (ROS / "unloading_ros_bridge" / "unloading_ros_bridge" / "world_bridge_node.py").read_text(encoding="utf-8")
    assert "transform_cache" in world_bridge
