"""External ROS 2 Humble adapter for saved Isaac RGB-D/GT keyframes.

Isaac Python 3.12 writes standard artifacts.  This Python 3.10 node validates
their hashes and publishes standard ROS types plus the domain observation, so
the two Python ABIs never import each other's extension modules.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray

from unloading_interfaces.msg import PerceptionObservation
from unloading_perception.geometry import quaternion_from_rotation
from unloading_perception.isaac_validation import (
    IsaacCaptureBinding,
    IsaacSceneManifest,
    ground_truth_observation,
    sha256_file,
)

from .common import float_to_time, require_humble_python310
from .mapping import observation_to_msg


def _stamp(value: float) -> Time:
    return float_to_time(float(value))


class IsaacSensorAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("isaac_sensor_adapter")
        self.declare_parameter("capture_directory", "")
        self.declare_parameter("scene_manifest", "")
        self.declare_parameter("publish_period_seconds", 1.0)
        capture = Path(str(self.get_parameter("capture_directory").value)).resolve()
        manifest_path = Path(str(self.get_parameter("scene_manifest").value)).resolve()
        if not capture.is_dir() or not manifest_path.is_file():
            raise RuntimeError("capture_directory and scene_manifest must exist")
        self.capture = capture
        self.manifest = IsaacSceneManifest.from_dict(json.loads(manifest_path.read_text(encoding="utf-8")))
        self.binding = IsaacCaptureBinding.from_dict(json.loads((capture / "capture_binding.json").read_text(encoding="utf-8")))
        self.annotations = json.loads((capture / "gt_annotations.json").read_text(encoding="utf-8"))
        if sha256_file(capture / "sensor_rgb.png") != self.binding.rgb_sha256:
            raise RuntimeError("Isaac RGB hash differs from capture binding")
        if sha256_file(capture / "gt_annotations.json") != self.binding.gt_snapshot_sha256:
            raise RuntimeError("Isaac GT hash differs from capture binding")
        if (self.binding.simulation_epoch, self.binding.frame_sequence) != (
            self.manifest.timing["simulation_epoch"], self.manifest.timing["simulation_frame"],
        ):
            raise RuntimeError("capture and scene manifest timing identities differ")
        camera = self.manifest.cameras[0]
        self.frame_id = str(camera["frame_id"])
        self.stamp = _stamp(float(self.binding.simulation_time))
        self.rgb = np.load(capture / "sensor_rgb.npy", allow_pickle=False).astype(np.uint8)
        self.depth = np.load(capture / "metric_depth_m.npy", allow_pickle=False).astype(np.float32)
        self.points = np.load(capture / "pointcloud_world_m.npz", allow_pickle=False)["xyz_m"].astype(np.float32)
        if self.rgb.ndim != 3 or self.rgb.shape[2] != 3 or self.rgb.shape[:2] != self.depth.shape:
            raise RuntimeError("RGB and depth dimensions differ")
        self.observation = ground_truth_observation(self.manifest, self.annotations["objects"])

        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub = self.create_publisher(Clock, "/clock", sensor_qos)
        self.rgb_pub = self.create_publisher(Image, "/isaac/front_camera/rgb", sensor_qos)
        self.depth_pub = self.create_publisher(Image, "/isaac/front_camera/depth", sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, "/isaac/front_camera/camera_info", sensor_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/isaac/front_camera/pointcloud", sensor_qos)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", sensor_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.joints_pub = self.create_publisher(JointState, "/joint_states", sensor_qos)
        self.gt2_pub = self.create_publisher(Detection2DArray, "/isaac/ground_truth/detections_2d", sensor_qos)
        self.gt3_pub = self.create_publisher(Detection3DArray, "/isaac/ground_truth/detections_3d", sensor_qos)
        self.domain_pub = self.create_publisher(PerceptionObservation, "/unloading/perception", sensor_qos)
        period = float(self.get_parameter("publish_period_seconds").value)
        if period <= 0.0:
            raise RuntimeError("publish_period_seconds must be positive")
        self.timer = self.create_timer(period, self.publish_capture)

    def _header(self, message, frame_id: str) -> None:
        message.header.stamp = self.stamp
        message.header.frame_id = frame_id

    def publish_capture(self) -> None:
        clock = Clock(clock=self.stamp)
        self.clock_pub.publish(clock)

        rgb = Image(height=self.rgb.shape[0], width=self.rgb.shape[1], encoding="rgb8", is_bigendian=0, step=self.rgb.shape[1] * 3, data=self.rgb.tobytes())
        depth = Image(height=self.depth.shape[0], width=self.depth.shape[1], encoding="32FC1", is_bigendian=0, step=self.depth.shape[1] * 4, data=self.depth.astype("<f4", copy=False).tobytes())
        self._header(rgb, self.frame_id)
        self._header(depth, self.frame_id)
        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)

        camera = self.manifest.cameras[0]
        info = CameraInfo(height=self.depth.shape[0], width=self.depth.shape[1], distortion_model=str(camera["distortion_model"]), d=list(camera["distortion"]), k=list(camera["K"]), r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        info.p = [camera["K"][0], 0.0, camera["K"][2], 0.0, 0.0, camera["K"][4], camera["K"][5], 0.0, 0.0, 0.0, 1.0, 0.0]
        self._header(info, self.frame_id)
        self.info_pub.publish(info)

        cloud = PointCloud2(height=1, width=len(self.points), is_bigendian=False, point_step=12, row_step=12 * len(self.points), is_dense=bool(np.isfinite(self.points).all()), data=self.points.astype("<f4", copy=False).tobytes())
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self._header(cloud, "world")
        self.cloud_pub.publish(cloud)

        transform = np.asarray(camera["T_W_C"], dtype=float)
        quaternion = quaternion_from_rotation(transform[:3, :3])
        tf = TransformStamped()
        self._header(tf, "world")
        tf.child_frame_id = self.frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = transform[:3, 3].tolist()
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quaternion
        message = TFMessage(transforms=[tf])
        self.tf_pub.publish(message)
        self.tf_static_pub.publish(message)

        joints = JointState(name=list(self.manifest.robot["joint_names"]), position=list(self.manifest.robot["q_rad"]))
        self._header(joints, "world")
        self.joints_pub.publish(joints)

        detections_2d = Detection2DArray()
        detections_3d = Detection3DArray()
        self._header(detections_2d, self.frame_id)
        self._header(detections_3d, "world")
        by_id = {item["simulation_object_id"]: item for item in self.manifest.objects}
        for annotation in self.annotations["objects"]:
            x1, y1, x2, y2 = (float(value) for value in annotation["bbox_xyxy"])
            detection_2d = Detection2D(id=str(annotation["simulation_object_id"]))
            detection_2d.bbox.center.position.x = 0.5 * (x1 + x2)
            detection_2d.bbox.center.position.y = 0.5 * (y1 + y2)
            detection_2d.bbox.size_x = x2 - x1
            detection_2d.bbox.size_y = y2 - y1
            detections_2d.detections.append(detection_2d)
            state = by_id[annotation["simulation_object_id"]]
            pose = np.asarray(state["T_W_object"], dtype=float)
            detection_3d = Detection3D(id=str(annotation["simulation_object_id"]))
            detection_3d.bbox.center.position.x, detection_3d.bbox.center.position.y, detection_3d.bbox.center.position.z = pose[:3, 3].tolist()
            q = quaternion_from_rotation(pose[:3, :3])
            detection_3d.bbox.center.orientation.x, detection_3d.bbox.center.orientation.y, detection_3d.bbox.center.orientation.z, detection_3d.bbox.center.orientation.w = q
            detection_3d.bbox.size.x, detection_3d.bbox.size.y, detection_3d.bbox.size.z = state["full_dimensions_m"]
            detections_3d.detections.append(detection_3d)
        self.gt2_pub.publish(detections_2d)
        self.gt3_pub.publish(detections_3d)
        self.domain_pub.publish(observation_to_msg(self.observation))


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = IsaacSensorAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
