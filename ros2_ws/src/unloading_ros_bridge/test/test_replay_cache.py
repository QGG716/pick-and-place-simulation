"""Synthetic small records; real validation, source admission, DDS and heartbeat paths."""
from copy import deepcopy
import json
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from unloading_interfaces.msg import PlanningWorldSnapshot
from unloading_ros_bridge.common import time_to_float
from unloading_ros_bridge.mapping import observation_to_msg, snapshot_from_msg
from unloading_ros_bridge.marker_display import marker_qos
from unloading_ros_bridge.mock_state_publisher import MockStatePublisher
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode
from test_replay_mode import publication
from visualization_msgs.msg import Marker, MarkerArray


def test_verified_record_cache_compares_content_and_rechecks_transport():
    rclpy.init(args=['--ros-args', '-p', 'observation_mode:=replay_display_only'])
    node = WorldBridgeNode()
    try:
        first = observation_to_msg(publication())
        node.on_observation(first)
        saved = node.last_observation
        later = observation_to_msg(publication(1))
        bad = []
        for field, value in (('coverage_json', '[]'), ('coverage_json', '{'),
                             ('schema_version', 'invalid'), ('provider', 'changed-source'),
                             ('exact_processed_time_s', -1.)):
            message = deepcopy(later)
            setattr(message, field, value)
            bad.append(message)
        changed = deepcopy(later)
        changed.cargo[0].raw_result_json = '{'
        bad.append(changed)
        changed = deepcopy(later)
        changed.cargo[0].source_instance_id += '-changed'
        bad.append(changed)
        for message in bad:
            node.on_observation(message)
            assert node.replay_guard.sequence == 0
            assert node.last_observation is saved
        # Mutating the caller's old ROS object cannot poison cached ownership.
        first.cargo.clear()
        node.on_observation(later)
        assert node.replay_guard.sequence == 1
        assert node.last_observation is saved  # pinned record, separate publication guard
        node.on_observation(later)  # duplicate transport is still rejected
        assert node.replay_guard.sequence == 1
        restarted = deepcopy(observation_to_msg(publication()))
        restarted.source_epoch = 'synthetic-new-session'
        coverage = json.loads(restarted.coverage_json)
        coverage['source_restart'] = True
        coverage['replay']['session_id'] = restarted.source_epoch
        restarted.coverage_json = json.dumps(coverage)
        node.on_observation(restarted)
        assert node.replay_guard.current_epoch == 'synthetic-new-session'
        assert node.last_observation is not saved
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_replay_heartbeat_reuse_preserves_sampling_faults_and_content():
    rclpy.init(args=['--ros-args', '-p', 'observation_mode:=replay_display_only'],
               domain_id=130+os.getpid()%30)
    world, source = WorldBridgeNode(), MockStatePublisher()
    observer = rclpy.create_node('synthetic_cache_observer')
    messages = []
    markers = []
    observer.create_subscription(PlanningWorldSnapshot, '/unloading/world_snapshot', messages.append, 10)
    observer.create_subscription(MarkerArray, '/unloading/markers', markers.append, marker_qos())
    executor = SingleThreadedExecutor()
    for node in (world, source, observer): executor.add_node(node)
    def wait(predicate, timeout=5.):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
            if predicate(): return
        raise AssertionError('replay heartbeat condition timed out')
    def state_faults(message):
        return [reason for reason in message.blocking_reasons if 'STATE_' in reason]
    def legend():
        return '\n'.join(m.text for m in markers[-1].markers if m.action==Marker.ADD)
    try:
        wait(lambda: world.publisher.get_subscription_count()>0 and source.joints.get_subscription_count()>0)
        world.on_observation(observation_to_msg(publication()))
        wait(lambda: len(messages)>=4 and not state_faults(messages[-1]))
        initial = messages[-1]
        templates = tuple(id(t) for _,t in world.heartbeat_templates)
        count = len(messages)
        wait(lambda: len(messages)>=count+4)
        assert tuple(id(t) for _,t in world.heartbeat_templates)==templates
        for message in messages[count:]:
            domain = snapshot_from_msg(message)
            assert time_to_float(message.robot_sample_time) == pytest.approx(domain.robot_state_revision.sample_time)
            assert domain.fingerprint == initial.world_fingerprint
            assert not message.planning_admissible
            assert 'HISTORICAL_REPLAY_DISPLAY_ONLY' in message.blocking_reasons
            assert message.source_capture_time == initial.source_capture_time
        assert messages[-1].robot_sample_time != initial.robot_sample_time
        source.timer.cancel()  # both auxiliary sources stop; no freshness monkeypatch
        wait(lambda: 'MECHANISM_STATE_STALE_OR_TIME_JUMP' in messages[-1].blocking_reasons)
        frozen = messages[-1]
        count = len(messages)
        wait(lambda: len(messages)>=count+3)
        wait(lambda: markers and 'MECHANISM_STATE_STALE_OR_TIME_JUMP' in legend())
        assert 'ROBOT_STATE_STALE_OR_TIME_JUMP' in legend()
        for message in messages[count:]:
            assert message.robot_sample_time == frozen.robot_sample_time
            assert message.mechanism_sample_time == frozen.mechanism_sample_time
            assert len(state_faults(message)) == 2
            assert snapshot_from_msg(message).scene_snapshot['blocking_reasons'] == tuple(message.blocking_reasons)
        source.timer.reset()  # same epoch, strictly new source samples
        wait(lambda: not state_faults(messages[-1]))
        wait(lambda: 'STATE_STALE_OR_TIME_JUMP' not in legend())
        assert 'HISTORICAL_REPLAY_DISPLAY_ONLY' in legend()
        assert messages[-1].world_fingerprint == initial.world_fingerprint
        assert messages[-1].robot_sample_time != frozen.robot_sample_time
        assert not messages[-1].planning_admissible
        assert world.assembler.update.blocking_reasons == tuple(messages[-1].blocking_reasons)
        # Actual mechanism change must invalidate the template, even with same source.
        from rclpy.parameter import Parameter
        source.set_parameters([Parameter('config_identity', value='synthetic-changed-config')])
        wait(lambda: messages[-1].mechanism_fingerprint != initial.mechanism_fingerprint)
        assert snapshot_from_msg(messages[-1]).config_identity['identity'] == 'synthetic-changed-config'
        assert len(world.heartbeat_templates)<=8
    finally:
        executor.shutdown()
        for node in (world, source, observer): node.destroy_node()
        rclpy.shutdown()
