"""Real Humble DDS, world snapshots, /clock watchdog and late subscription.

All inputs are synthetic. No model, Isaac, execution node or authorization.
"""
from dataclasses import replace
import json
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray

from test_perception_time_transport import transport, sample
from test_marker_display import geometry, labels, rgba
from unloading_ros_bridge.common import float_to_time
from unloading_ros_bridge.mapping import observation_to_msg
from unloading_ros_bridge.marker_demo import demo_snapshot
from unloading_ros_bridge.marker_display import marker_qos, BLOCKED, HISTORY
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode


def observed_sample(t, seq, two=False):
    base = sample(t, seq, 'registered-rgbd-fused-algorithm', unknown=True)
    surface = json.loads(demo_snapshot(1).obstacles[-1].observed_surfaces_json)[0]
    surface.update(capture_time=t, sensor_epoch=base.source_epoch)
    first = replace(base.cargo[0], source_instance_id='synthetic-fusion-A',
        object_id=None, track_id=None, pose=None, full_dimensions_m=None,
        corners_3d_m=None, axes_3d_rows=None, candidate_eligible=False,
        eligibility_reasons=('OBSERVED_SURFACES_WITH_UNKNOWN_VOLUME',),
        association_status='MULTIFACE_OBSERVED_VOLUME_UNRESOLVED',
        observed_surfaces=(surface,), raw_result={'fusion_id': 'synthetic-fusion-A'})
    other_surface = dict(surface, source_instance_id='synthetic-other-source')
    second = replace(first, source_instance_id='synthetic-fusion-B', observed_surfaces=(other_surface,),
        raw_result={'fusion_id': 'synthetic-fusion-B'})
    return replace(base, cargo=(first, second) if two else (second,),
        coverage={'expected_modules': ['synthetic-module'], 'received_modules': ['synthetic-module'],
                  'module_bindings': {'synthetic-module': {k: surface[k] for k in
                      ('capture_id', 'calibration_identity', 'T_W_C_at_capture')}}})


def test_actual_world_markers_watchdog_deletion_and_late_subscriber(transport):
    bridge, executor, worlds, observations, clock, states, wait = transport
    receiver = rclpy.create_node('synthetic_marker_receiver')
    executor.add_node(receiver)
    received = []
    subscription = receiver.create_subscription(MarkerArray, '/unloading/markers', received.append, marker_qos())
    try:
        wait(lambda: bridge.marker_publisher.get_subscription_count() == 1)
        clock(100.)
        states(100., 0)
        observations.publish(observation_to_msg(observed_sample(100., 1, two=True)))
        wait(lambda: received and len(geometry(received[-1].markers)) == 2 and worlds)
        assert not worlds[-1].planning_admissible
        assert all(not c.candidate_eligible and not c.has_pose for c in worlds[-1].obstacles)
        first_keys = {(m.ns, m.id) for m in geometry(received[-1].markers)}
        assert 'UNKNOWN' in labels(received[-1].markers)
        wait(lambda: bridge.stale_key is not None)
        receiver.destroy_subscription(subscription)
        subscription = None
        observations.publish(observation_to_msg(observed_sample(100., 2)))
        wait(lambda: bridge.last_observation.source_sequence == 2 and len(worlds[-1].obstacles) == 1)
        # Join after publication; no additional perception or geometry change.
        # Pause executor callbacks while retaining the real DDS publisher. This
        # isolates cached delivery from the new periodic world heartbeat.
        executor.remove_node(bridge)
        sequence = bridge.publisher_sequence
        late = []
        subscription = receiver.create_subscription(MarkerArray, '/unloading/markers', late.append, marker_qos())
        wait(lambda: bool(late))
        assert bridge.publisher_sequence == sequence
        executor.add_node(bridge)
        current = geometry(late[-1].markers)
        assert len(current) == 1
        assert first_keys - {(m.ns, m.id) for m in current} <= {
            (m.ns, m.id) for m in late[-1].markers if m.action == Marker.DELETE}
        before_source = bridge.last_observation
        before_geometry = list(worlds[-1].obstacles)
        # Advance /clock only: the production steady watchdog publishes history.
        clock(103.)
        # DDS does not promise callback ordering across two separate topics.
        wait(lambda: 'HISTORY / TIME INVALID' in labels(late[-1].markers)
             and 'OBSERVATION_STALE' in worlds[-1].blocking_reasons)
        assert 'OBSERVATION_STALE' in worlds[-1].blocking_reasons
        assert rgba(geometry(late[-1].markers)[0]) == pytest.approx(HISTORY)
        assert bridge.last_observation is before_source
        assert list(worlds[-1].obstacles) == before_geometry
        assert worlds[-1].source_capture_time == float_to_time(100.)
        assert geometry(late[-1].markers)[0].header.stamp == float_to_time(100.)
        assert 'surface.capture_time=100.0' in labels(late[-1].markers)
        states(103., 1)
        observations.publish(observation_to_msg(observed_sample(103., 3)))
        wait(lambda: bridge.last_observation.source_sequence == 3 and
             'HISTORY / TIME INVALID' not in labels(late[-1].markers)
             and geometry(late[-1].markers)[0].header.stamp == float_to_time(103.)
             and worlds[-1].source_capture_time == float_to_time(103.)
             and 'OBSERVATION_STALE' not in worlds[-1].blocking_reasons)
        assert not worlds[-1].planning_admissible
        assert 'UNKNOWN_OR_UNTRANSFORMED_REGIONS' in worlds[-1].blocking_reasons
        assert rgba(geometry(late[-1].markers)[0]) == pytest.approx(BLOCKED)
        assert 'SYNTHETIC_UNKNOWN_VOLUME' in labels(late[-1].markers)
        # The rejected future sample must gray the retained valid source too.
        observations.publish(observation_to_msg(observed_sample(110., 999)))
        wait(lambda: 'OBSERVATION_TIME_IN_FUTURE' in labels(late[-1].markers)
             and 'OBSERVATION_TIME_IN_FUTURE' in worlds[-1].blocking_reasons)
        assert bridge.last_observation.source_sequence == 3
        assert geometry(late[-1].markers)[0].header.stamp == float_to_time(103.)
        assert rgba(geometry(late[-1].markers)[0]) == pytest.approx(HISTORY)
    finally:
        if subscription is not None: receiver.destroy_subscription(subscription)
        executor.remove_node(receiver)
        receiver.destroy_node()


def test_normal_destroy_clears_owned_markers_and_restart_republishes():
    rclpy.init()
    executor = SingleThreadedExecutor()
    receiver = rclpy.create_node('synthetic_restart_receiver')
    executor.add_node(receiver)
    received = []
    receiver.create_subscription(MarkerArray, '/unloading/markers', received.append, marker_qos())
    bridge = None
    def wait(predicate):
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate(): return
        raise AssertionError('marker restart DDS condition not reached')
    try:
        bridge = WorldBridgeNode()
        bridge.publish_markers(demo_snapshot(0), 'world')
        wait(lambda: received and len(geometry(received[-1].markers)) == 2)
        owned = {(m.ns, m.id) for m in received[-1].markers}
        bridge.destroy_node()
        bridge = None
        wait(lambda: received and all(m.action == Marker.DELETE for m in received[-1].markers))
        assert owned == {(m.ns, m.id) for m in received[-1].markers}
        bridge = WorldBridgeNode()
        bridge.publish_markers(demo_snapshot(1), 'world')
        wait(lambda: 'OBSERVED PATCH' in labels(received[-1].markers))
        assert len(geometry(received[-1].markers)) == 2
    finally:
        if bridge is not None: bridge.destroy_node()
        executor.shutdown()
        receiver.destroy_node()
        rclpy.shutdown()
