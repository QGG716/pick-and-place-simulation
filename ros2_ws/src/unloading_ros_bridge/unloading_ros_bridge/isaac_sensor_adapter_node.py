"""External ROS 2 Humble adapter for saved Isaac RGB-D/GT keyframes.

Isaac Python 3.12 writes standard artifacts.  This Python 3.10 node validates
their hashes and publishes standard ROS types plus the domain observation, so
the two Python ABIs never import each other's extension modules.
"""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray

from unloading_interfaces.msg import PerceptionObservation, RgbdCaptureMetadata
from unloading_perception.geometry import quaternion_from_rotation
from unloading_perception.isaac_validation import (
    ground_truth_observation,
    validate_capture_robot_state,
)
from unloading_perception.isaac_payload import CapturePayloadError, load_capture_payload

from .common import float_to_time, require_humble_python310
from .mapping import capture_metadata_to_msg, observation_to_msg


def _stamp(value: float) -> Time:
    return float_to_time(float(value))


class IsaacSensorAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("isaac_sensor_adapter")
        self.declare_parameter("capture_directory", "")
        self.declare_parameter("scene_manifest", "")
        self.declare_parameter("sequence_index", "")
        self.declare_parameter("sequence_hold_cycles", 1)
        self.declare_parameter("publish_period_seconds", 1.0)
        self.declare_parameter("publish_pointcloud_on_demand", False)
        capture = Path(str(self.get_parameter("capture_directory").value)).resolve()
        manifest_value = str(self.get_parameter("scene_manifest").value)
        sequence_value = str(self.get_parameter("sequence_index").value)
        if sequence_value:
            sequence_path = Path(sequence_value).resolve()
            if not capture.is_dir() or not sequence_path.is_file():
                raise RuntimeError("capture_directory and sequence_index must exist")
            index = json.loads(sequence_path.read_text(encoding="utf-8"))
            self.samples = tuple(
                (capture / record["scene"], sequence_path.parent / record["path"])
                for record in index["scenes"]
            )
        else:
            manifest_path = Path(manifest_value).resolve()
            if not capture.is_dir() or not manifest_path.is_file():
                raise RuntimeError("capture_directory and scene_manifest must exist")
            self.samples = ((capture, manifest_path),)
        if not self.samples:
            raise RuntimeError("Isaac capture sequence cannot be empty")
        self.sample_index = 0
        self.sequence_hold_cycles = int(self.get_parameter("sequence_hold_cycles").value)
        if self.sequence_hold_cycles <= 0:
            raise RuntimeError("sequence_hold_cycles must be positive")
        self.sequence_hold_count = 0
        self.publish_pointcloud_on_demand = bool(self.get_parameter("publish_pointcloud_on_demand").value)
        self._load_capture(*self.samples[0])

        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, "/clock", sensor_qos)
        self.rgb_pub = self.create_publisher(Image, "/isaac/vision_mast/module_0_main/rgb", sensor_qos)
        self.depth_pub = self.create_publisher(Image, "/isaac/vision_mast/module_0_main/depth", sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, "/isaac/vision_mast/module_0_main/camera_info", sensor_qos)
        self.metadata_pub = self.create_publisher(RgbdCaptureMetadata, "/isaac/vision_mast/module_0_main/capture_metadata", sensor_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/isaac/vision_mast/module_0_main/pointcloud_on_demand", sensor_qos)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", sensor_qos)
        self.joints_pub = self.create_publisher(JointState, "/joint_states", sensor_qos)
        self.gt2_pub = self.create_publisher(Detection2DArray, "/isaac/ground_truth/detections_2d", sensor_qos)
        self.gt3_pub = self.create_publisher(Detection3DArray, "/isaac/ground_truth/detections_3d", sensor_qos)
        self.domain_pub = self.create_publisher(PerceptionObservation, "/unloading/perception", sensor_qos)
        period = float(self.get_parameter("publish_period_seconds").value)
        if period <= 0.0:
            raise RuntimeError("publish_period_seconds must be positive")
        self.timer = self.create_timer(period, self.publish_capture)

    def _load_capture(self, capture: Path, manifest_path: Path) -> None:
        # Old arrays may remain for diagnosis, but cannot publish after failure.
        self.sample_ready = False
        self.capture_rejection = None
        try:
            sample = load_capture_payload(capture, manifest_path,
                with_pointcloud=self.publish_pointcloud_on_demand)
            robot_state, robot_rejection = None, None
            try:
                robot_state = validate_capture_robot_state(sample.binding.extensions.get('robot_state'),
                    sample.manifest, sample.binding)
            except (ValueError, TypeError, KeyError) as exc:
                robot_rejection = str(exc)
                self.get_logger().error(f'robot JointState suppressed: {exc}; verified image replay remains available')
            observation = ground_truth_observation(sample.manifest, sample.annotations['objects'],
                camera_frame_id=sample.camera['frame_id'])
            # Commit one fully verified sample only after every load succeeds.
            self.__dict__.update(capture=capture, manifest=sample.manifest, binding=sample.binding,
                capture_metadata=sample.metadata, annotations=sample.annotations, camera=sample.camera,
                frame_id=sample.camera['frame_id'], depth_frame_id=sample.camera['depth_frame_id'],
                stamp=_stamp(sample.binding.simulation_time), rgb=sample.rgb, depth=sample.depth,
                points=sample.points, robot_state=robot_state, robot_state_rejection=robot_rejection,
                observation=observation, sample_ready=True)
        except (CapturePayloadError, ValueError, TypeError, KeyError) as exc:
            self.capture_rejection = str(exc)
            self.get_logger().error(f'capture suppressed: {exc}')
            raise

    def _advance_sample(self) -> None:
        if self.sample_index + 1 < len(self.samples):
            self.sample_index += 1
            self.sequence_hold_count = 0
            try:
                self._load_capture(*self.samples[self.sample_index])
            except (CapturePayloadError, ValueError, TypeError, KeyError):
                pass  # diagnostic already recorded; no publication from this sample

    def _header(self, message, frame_id: str) -> None:
        message.header.stamp = self.stamp
        message.header.frame_id = frame_id

    def publish_capture(self) -> None:
        if not self.sample_ready:
            self._advance_sample()
            return
        clock = Clock(clock=self.stamp)
        self.clock_pub.publish(clock)

        rgb = Image(height=self.rgb.shape[0], width=self.rgb.shape[1], encoding="rgb8", is_bigendian=0, step=self.rgb.shape[1] * 3, data=self.rgb.tobytes())
        depth = Image(height=self.depth.shape[0], width=self.depth.shape[1], encoding="32FC1", is_bigendian=0, step=self.depth.shape[1] * 4, data=self.depth.astype("<f4", copy=False).tobytes())
        self._header(rgb, self.frame_id)
        self._header(depth, self.depth_frame_id)
        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)

        camera = self.camera
        info = CameraInfo(height=self.depth.shape[0], width=self.depth.shape[1], distortion_model=str(camera["distortion_model"]), d=list(camera["distortion"]), k=list(camera["K"]), r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        info.p = [camera["K"][0], 0.0, camera["K"][2], 0.0, 0.0, camera["K"][4], camera["K"][5], 0.0, 0.0, 0.0, 1.0, 0.0]
        self._header(info, self.frame_id)
        self.info_pub.publish(info)
        self.metadata_pub.publish(capture_metadata_to_msg(self.capture_metadata))

        if self.publish_pointcloud_on_demand:
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
        depth_tf = deepcopy(tf)
        depth_tf.child_frame_id = self.depth_frame_id  # verified coaxial registered depth
        message = TFMessage(transforms=[tf, depth_tf])
        self.tf_pub.publish(message)

        if self.robot_state is not None:
            joints = JointState(name=self.robot_state['joint_names'],
                position=self.robot_state['position_rad'], velocity=self.robot_state['velocity_rad_s'])
            self._header(joints, "world")  # bound capture time; replay never refreshes it
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
        self.sequence_hold_count += 1
        if self.sample_index + 1 < len(self.samples) and self.sequence_hold_count >= self.sequence_hold_cycles:
            self._advance_sample()


def main(args=None) -> None:
    require_humble_python310()
    rclpy.init(args=args)
    node = IsaacSensorAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
