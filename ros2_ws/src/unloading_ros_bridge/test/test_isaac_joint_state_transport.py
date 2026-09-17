"""Real adapter -> DDS JointState -> real world bridge, synthetic inputs only.

No execution node, execution authorization, action client or trajectory exists
in this test. The test publisher supplies only perception/mechanisms (and
explicitly malformed receiver-negative JointStates).
"""
from dataclasses import replace
import copy
import json
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, JointState
from unloading_contracts import UnknownRegion
from unloading_interfaces.msg import MechanismState, PerceptionObservation, PlanningWorldSnapshot
from unloading_perception.demo import _synthetic_observation
from unloading_ros_bridge.common import float_to_time
from unloading_ros_bridge.isaac_sensor_adapter_node import IsaacSensorAdapterNode
from unloading_ros_bridge.mapping import observation_to_msg, snapshot_from_msg
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode

from isaac_joint_fixture import capture_fixture


@pytest.mark.parametrize('static', [False, True], ids=['nonzero-readback', 'static-hold-readback'])
def test_adapter_joint_state_reaches_world_without_opening_planning_gate(tmp_path, static):
    manifest, path, binding = capture_fixture(tmp_path, velocity=(0., 0.) if static else (.25, -.5))
    expected = binding['robot_state']
    if static:
        expected.update(source='KINEMATIC_HOLD_READBACK', kinematic_hold={
            'render_without_physics_step': True, 'position_command_rad': list(expected['position_rad']),
            'velocity_command_rad_s': [0., 0.]})
        (tmp_path/'capture_binding.json').write_text(json.dumps(binding))
    rclpy.init(args=['--ros-args', '-p', f'capture_directory:={tmp_path}',
        '-p', f'scene_manifest:={path}', '-p', 'publish_period_seconds:=3600.0',
        '-p', 'use_sim_time:=true', '-p', 'expected_joint_names:=[fixture_a, fixture_b]',
        '-r', 'isaac_sensor_adapter:/unloading/perception:=/fixture/isaac_gt'])
    adapter = bridge = node = None
    executor = SingleThreadedExecutor()
    try:
        adapter = IsaacSensorAdapterNode()
        bridge = WorldBridgeNode()
        node = rclpy.create_node('synthetic_joint_contract_inputs')
        for item in (adapter, bridge, node):
            executor.add_node(item)
        joints, worlds, images = [], [], []
        node.create_subscription(JointState, '/joint_states', joints.append, 10)
        node.create_subscription(Image, '/isaac/vision_mast/module_0_main/rgb', images.append, 10)
        node.create_subscription(PlanningWorldSnapshot, '/unloading/world_snapshot', worlds.append, 10)
        perception = node.create_publisher(PerceptionObservation, '/unloading/perception', 10)
        mechanisms = node.create_publisher(MechanismState, '/unloading/mechanism_state', 10)

        def wait(predicate, timeout=8):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
                if predicate():
                    return
            raise AssertionError('real adapter/world ROS transport timed out')

        wait(lambda: adapter.joints_pub.get_subscription_count() >= 2
             and adapter.clock_pub.get_subscription_count() >= 2
             and perception.get_subscription_count() > 0 and mechanisms.get_subscription_count() > 0)
        adapter.publish_capture()  # unmodified production publisher, reading fixture
        wait(lambda: bool(joints))
        message = joints[-1]
        assert list(message.name) == expected['joint_names']
        assert list(message.velocity) == expected['velocity_rad_s']  # baseline fails here/path above
        assert list(message.position) == expected['position_rad']
        assert not message.effort
        assert message.header.stamp == float_to_time(expected['sample_time'])
        wait(lambda: bridge.assembler.robot_state is not None)
        state = bridge.assembler.robot_state
        assert state.current_q == tuple(expected['position_rad'])
        assert state.robot_state['actual_velocities'] == tuple(expected['velocity_rad_s'])
        assert state.sample_time == expected['sample_time']
        assert adapter.robot_state['source'] == expected['source']
        assert adapter.robot_state['synthetic_fixture'] is True

        t = expected['sample_time']
        observation = replace(_synthetic_observation(), capture_time=t, processed_time=t,
            clock_domain='ros_sim_time', unknown_regions=(UnknownRegion('fixture-unknown', 'world', 'SYNTHETIC_UNOBSERVED_VOLUME'),))
        perception.publish(observation_to_msg(observation))
        mechanisms.publish(MechanismState(schema_version='1.1.0', source_epoch='synthetic-mechanisms',
            sequence=0, source_restart=True, observed_time=float_to_time(t), clock_domain='ros_sim_time',
            tool_state_identity='fixture-tool', payload_state_identity='fixture-empty',
            base_state_identity='fixture-base', conveyor_state_identity='fixture-stopped',
            config_identity='fixture-config', robot_model_fingerprint=manifest.robot['robot_asset_hash'],
            world_model_fingerprint=manifest.world_fingerprint, tool_state_json='{"verified":true}',
            payload_state_json='{"object_id":null}', base_state_json='{"position_m":[0,0,0]}',
            conveyor_state_json='{"running":false}'))
        wait(lambda: bool(worlds))
        snapshot = snapshot_from_msg(worlds[-1])
        assert snapshot.robot_state_revision.current_q == tuple(expected['position_rad'])
        assert snapshot.robot_state_revision.robot_state['actual_velocities'] == tuple(expected['velocity_rad_s'])
        assert snapshot.robot_state_revision.sample_time == t
        assert not worlds[-1].planning_admissible
        assert worlds[-1].unknown_regions

        # A duplicate from the actual adapter must not refresh sample time.
        adapter.publish_capture()
        wait(lambda: len(joints) >= 2)
        for _ in range(10): executor.spin_once(timeout_sec=.02)
        assert bridge.assembler.robot_state is state
        assert bridge.last_joint_stamp == t

        # Receiver negatives are explicitly injected, never used as positives.
        invalid_publisher = node.create_publisher(JointState, '/joint_states', 10)
        wait(lambda: invalid_publisher.get_subscription_count() >= 2)
        bad_fields = [dict(velocity=[]), dict(position=[0.]), dict(velocity=[0.]),
            dict(name=['fixture_a']), dict(name=['fixture_a', 'fixture_a']),
            dict(name=['fixture_b', 'fixture_a']), dict(position=[float('nan'), 0.]),
            dict(velocity=[0., float('inf')]), dict(effort=[float('inf'), 0.])]
        for fields in bad_fields + [dict(sample_time=t), dict(sample_time=t-1.)]:
            bad = JointState(name=list(message.name), position=list(message.position), velocity=list(message.velocity))
            bad.header.stamp = float_to_time(fields.get('sample_time', t+.1))
            for key, value in fields.items():
                if key != 'sample_time': setattr(bad, key, value)
            count = len(joints)
            invalid_publisher.publish(bad)
            wait(lambda: len(joints) > count)
            for _ in range(5): executor.spin_once(timeout_sec=.02)
            assert bridge.assembler.robot_state is state
            assert bridge.last_joint_stamp == t

        # Exercise real adapter loading invalid files after a valid sample.
        # Missing evidence must clear the adapter cache, not republish old joints.
        invalid_records = [None, {'static': True}]
        for patch in ({'velocity_rad_s': []}, {'joint_names': ['fixture_b', 'fixture_a']},
                      {'position_rad': [float('nan'), 0.]}, {'sample_time': t-1.},
                      {'source': 'KINEMATIC_HOLD_READBACK', 'kinematic_hold': {}}):
            invalid_records.append({**expected, **patch})
        for record in invalid_records:
            bad_binding = copy.deepcopy(binding)
            if record is None: bad_binding.pop('robot_state')
            else: bad_binding['robot_state'] = record
            (tmp_path/'capture_binding.json').write_text(json.dumps(bad_binding))
            adapter._load_capture(tmp_path, path)
            assert adapter.robot_state is None
            assert adapter.robot_state_rejection
            count = len(joints)
            image_count = len(images)
            adapter.publish_capture()
            wait(lambda: len(images) > image_count)
            for _ in range(10): executor.spin_once(timeout_sec=.02)
            assert len(joints) == count
            assert bridge.assembler.robot_state is state
            assert bridge.last_joint_stamp == t
    finally:
        executor.shutdown()
        for item in (adapter, bridge, node):
            if item is not None: item.destroy_node()
        rclpy.shutdown()
