"""Real Isaac adapter transport, using tiny synthetic capture files only."""
import json
import time
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from tf2_msgs.msg import TFMessage
from unloading_interfaces.msg import RgbdCaptureMetadata
from unloading_perception.isaac_validation import sha256_file
from unloading_ros_bridge.common import float_to_time
from unloading_ros_bridge.isaac_sensor_adapter_node import IsaacSensorAdapterNode
from isaac_joint_fixture import capture_fixture


def test_original_depth_replacement_is_rejected_by_production_loader(tmp_path):
    _, path, _ = capture_fixture(tmp_path)
    rclpy.init(args=['--ros-args', '-p', f'capture_directory:={tmp_path}',
        '-p', f'scene_manifest:={path}', '-p', 'publish_period_seconds:=3600.0'])
    adapter = None
    try:
        adapter = IsaacSensorAdapterNode()
        depth = np.load(tmp_path/'metric_depth_m.npy', allow_pickle=False)
        depth[0, 0] += .125
        np.save(tmp_path/'metric_depth_m.npy', depth)
        # PNG, GT, metadata and binding remain byte-for-byte unchanged.
        with pytest.raises(RuntimeError, match='HASH_MISMATCH.*metric_depth'):
            adapter._load_capture(tmp_path, path)
    finally:
        if adapter is not None: adapter.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('cloud_enabled,bad_file', [(False, 'metric_depth_m.npy'),
    (True, 'metric_depth_m.npy'), (True, 'pointcloud_world_m.npz')])
def test_real_ros_sequence_a_bad_b_c_and_verified_cache(tmp_path, cloud_enabled, bad_file):
    samples = []
    for i, name in enumerate(('A', 'B', 'C')):
        directory = tmp_path/name
        manifest, path, binding = capture_fixture(directory, stamp=10.125+i,
            frame=7+i, pixel=32+20*i, camera_offset=.125*i)
        # New synthetic capture values, bound by the fixture author before use.
        np.save(directory/'metric_depth_m.npy', np.full((2, 3), i+1., dtype=np.float32))
        np.savez(directory/'pointcloud_world_m.npz', xyz_m=np.full((2, 3), i+.25, dtype=np.float32))
        binding['metric_depth_sha256'] = sha256_file(directory/'metric_depth_m.npy')
        binding['pointcloud_sha256'] = sha256_file(directory/'pointcloud_world_m.npz')
        (directory/'capture_binding.json').write_text(json.dumps(binding))
        if not cloud_enabled:
            (directory/'pointcloud_world_m.npz').unlink()
        samples.append((manifest, binding))
    if bad_file.endswith('.npy'):
        np.save(tmp_path/'B'/bad_file, np.full((2, 3), 99., dtype=np.float32))
    else:
        np.savez(tmp_path/'B'/bad_file, xyz_m=np.full((2, 3), 99., dtype=np.float32))
    # PNG is the sole RGB authority; this obsolete cache must never be read.
    (tmp_path/'C'/'sensor_rgb.npy').write_bytes(b'not a numpy file')
    index = tmp_path/'index.json'
    index.write_text(json.dumps({'scenes': [{'scene': name, 'path': f'{name}/manifest.json'} for name in ('A', 'B', 'C')]}))
    rclpy.init(args=['--ros-args', '-p', f'capture_directory:={tmp_path}',
        '-p', f'sequence_index:={index}', '-p', 'publish_period_seconds:=3600.0',
        '-p', f'publish_pointcloud_on_demand:={str(cloud_enabled).lower()}'])
    executor = SingleThreadedExecutor()
    adapter = node = None
    try:
        adapter = IsaacSensorAdapterNode()
        node = rclpy.create_node('synthetic_payload_receiver')
        executor.add_node(adapter)
        executor.add_node(node)
        received = {key: [] for key in ('rgb', 'depth', 'info', 'metadata', 'cloud', 'clock', 'tf', 'joints')}
        prefix = '/isaac/vision_mast/module_0_main/'
        for kind, message, topic in (
            ('rgb', Image, prefix+'rgb'), ('depth', Image, prefix+'depth'),
            ('info', CameraInfo, prefix+'camera_info'), ('metadata', RgbdCaptureMetadata, prefix+'capture_metadata'),
            ('cloud', PointCloud2, prefix+'pointcloud_on_demand'), ('clock', Clock, '/clock'),
            ('tf', TFMessage, '/tf'), ('joints', JointState, '/joint_states')):
            node.create_subscription(message, topic, received[kind].append, 10)

        def wait(predicate, timeout=5):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.02)
                if predicate(): return
            raise AssertionError('verified payload ROS transport timed out')

        def drain():
            end = time.monotonic()+.15
            while time.monotonic() < end: executor.spin_once(timeout_sec=.01)

        pubs = (adapter.rgb_pub, adapter.depth_pub, adapter.info_pub, adapter.metadata_pub,
                adapter.cloud_pub, adapter.clock_pub, adapter.tf_pub, adapter.joints_pub)
        wait(lambda: all(p.get_subscription_count() > 0 for p in pubs))
        required = set(received) - (set() if cloud_enabled else {'cloud'})

        def check(which, count):
            manifest, binding = samples[which]
            camera = manifest.cameras[0]
            stamp = float_to_time(binding['simulation_time'])
            wait(lambda: all(len(received[kind]) == count for kind in required))
            rgb, depth = received['rgb'][-1], received['depth'][-1]
            expected_rgb = np.full((2, 3, 3), 32+20*which, dtype=np.uint8)
            expected_rgb[..., 0] += 1
            assert bytes(rgb.data) == expected_rgb.tobytes()
            assert (rgb.width, rgb.height, rgb.encoding, rgb.step, rgb.is_bigendian) == (3, 2, 'rgb8', 9, 0)
            assert bytes(depth.data) == np.full((2, 3), which+1., dtype='<f4').tobytes()
            assert (depth.width, depth.height, depth.encoding, depth.step, depth.is_bigendian) == (3, 2, '32FC1', 12, 0)
            assert rgb.header.stamp == depth.header.stamp == stamp
            assert rgb.header.frame_id == camera['frame_id'] and depth.header.frame_id == camera['depth_frame_id']
            info = received['info'][-1]
            assert list(info.k) == camera['K'] and list(info.d) == camera['distortion']
            assert (info.width, info.height, info.header.frame_id) == (3, 2, camera['frame_id'])
            assert info.header.stamp == stamp
            metadata = received['metadata'][-1]
            assert metadata.capture_id == binding['capture_id'] and metadata.frame_sequence == 7+which
            assert metadata.sensor_epoch == binding['simulation_epoch'] and metadata.clock_domain == 'ros_sim_time'
            assert metadata.calibration_identity == binding['camera_calibration_identity']
            assert metadata.capture_center_time == stamp
            assert (metadata.rgb_frame_id, metadata.depth_frame_id) == (camera['frame_id'], camera['depth_frame_id'])
            transforms = received['tf'][-1].transforms
            assert {tf.child_frame_id for tf in transforms} == {camera['frame_id'], camera['depth_frame_id']}
            for tf in transforms:
                assert tf.header.frame_id == 'world' and tf.header.stamp == stamp
                assert tf.transform.translation.x == camera['T_W_C'][0][3]
            assert metadata.t_w_c_at_capture.translation.x == camera['T_W_C'][0][3]
            assert received['clock'][-1].clock == stamp
            assert received['joints'][-1].header.stamp == stamp
            assert list(received['joints'][-1].velocity) == [.25, -.5]
            if cloud_enabled:
                cloud = received['cloud'][-1]
                assert bytes(cloud.data) == np.full((2, 3), which+.25, dtype='<f4').tobytes()
                assert (cloud.width, cloud.height, cloud.point_step, cloud.row_step) == (2, 1, 12, 24)
                assert cloud.header.stamp == stamp and cloud.header.frame_id == 'world'
                assert [(f.name, f.offset, f.datatype) for f in cloud.fields] == [('x', 0, 7), ('y', 4, 7), ('z', 8, 7)]
            else:
                assert not received['cloud'] and adapter.points is None

        adapter.publish_capture()  # A publishes; attempting B then fails atomically
        check(0, 1)
        assert not adapter.sample_ready and 'HASH_MISMATCH' in adapter.capture_rejection
        assert bad_file in adapter.capture_rejection
        assert adapter.stamp == float_to_time(10.125)  # no B time or TF committed
        counts = {k: len(v) for k, v in received.items()}
        adapter.publish_capture()  # B emits nothing, next valid C is loaded
        drain()
        assert counts == {k: len(v) for k, v in received.items()}
        assert adapter.sample_ready and adapter.sample_index == 2
        adapter.publish_capture()
        check(2, 2)
        # Repeated static publication uses verified buffers, with original time.
        for filename in ('sensor_rgb.png', 'metric_depth_m.npy', 'capture_metadata.json', 'camera_info.json'):
            (tmp_path/'C'/filename).unlink()
        if cloud_enabled: (tmp_path/'C'/'pointcloud_world_m.npz').unlink()
        adapter.publish_capture()
        check(2, 3)
        assert all(m.clock != float_to_time(11.125) for m in received['clock'])
    finally:
        executor.shutdown()
        if node is not None: node.destroy_node()
        if adapter is not None: adapter.destroy_node()
        rclpy.shutdown()
