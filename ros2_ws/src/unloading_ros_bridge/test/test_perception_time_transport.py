"""Real DDS + /clock + world bridge; all geometry/states are synthetic.

No Isaac, model inference, execution node, authorization or trajectory.
"""
from dataclasses import replace
from contextlib import contextmanager
import math
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from unloading_contracts import UnknownRegion
from unloading_interfaces.msg import MechanismState, PerceptionObservation, PlanningWorldSnapshot
from unloading_perception.demo import _synthetic_observation
from unloading_ros_bridge.common import float_to_time
from unloading_ros_bridge.mapping import observation_to_msg
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode


@contextmanager
def time_transport(*, domain_id=None):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'],
               domain_id=120 + os.getpid() % 30 if domain_id is None else domain_id)
    bridge = WorldBridgeNode()
    node = rclpy.create_node('synthetic_time_admission_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(bridge)
    worlds = []
    node.create_subscription(PlanningWorldSnapshot, '/unloading/world_snapshot', worlds.append, 10)
    clocks = node.create_publisher(Clock, '/clock', 10)
    joints = node.create_publisher(JointState, '/joint_states', 10)
    mechanisms = node.create_publisher(MechanismState, '/unloading/mechanism_state', 10)
    observations = node.create_publisher(PerceptionObservation, '/unloading/perception', 10)

    def wait(predicate, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
            if predicate(): return
        raise AssertionError('ROS time admission condition not reached')

    def clock(t):
        clocks.publish(Clock(clock=float_to_time(t)))
        wait(lambda: bridge.get_clock().now().nanoseconds == round(t*1e9))

    def states(t, seq):
        state = JointState(name=[f'joint_{i}' for i in range(1, 7)],
                           position=[seq*.01]*6, velocity=[.1]*6)
        state.header.stamp = float_to_time(t)
        joints.publish(state)
        mechanisms.publish(MechanismState(schema_version='1.1.0', source_epoch='synthetic-mechanism',
            sequence=seq, source_restart=seq == 0, observed_time=float_to_time(t), clock_domain='ros_sim_time',
            tool_state_identity='synthetic-tool', payload_state_identity='synthetic-empty',
            base_state_identity='synthetic-base', conveyor_state_identity='synthetic-stopped',
            config_identity='synthetic-config', robot_model_fingerprint='synthetic-robot',
            world_model_fingerprint='synthetic-world', tool_state_json='{"verified":true}',
            payload_state_json='{"object_id":null}', base_state_json='{"position_m":[0,0,0]}',
            conveyor_state_json='{"running":false}'))
        wait(lambda: bridge.last_joint_stamp == t and bridge.mechanism_stamp == t)

    try:
        wait(lambda: all(p.get_subscription_count() > 0 for p in (clocks, joints, mechanisms, observations))
             and bridge.publisher.get_subscription_count() > 0)
        yield bridge, executor, worlds, observations, clock, states, wait
    finally:
        executor.shutdown()
        node.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def transport():
    with time_transport() as graph:
        yield graph


def sample(t, seq, provider, *, unknown=False):
    return replace(_synthetic_observation(), capture_time=t, processed_time=t+.01,
        clock_domain='ros_sim_time', source_epoch='synthetic-time-source', source_sequence=seq,
        provider=provider, coverage={'expected_modules': ['synthetic-module'],
            'received_modules': ['synthetic-module'], 'module_bindings': {'synthetic-module': {}}},
        unknown_regions=(UnknownRegion('synthetic-unknown', 'world', 'SYNTHETIC_UNKNOWN_VOLUME'),) if unknown else ())


@pytest.mark.parametrize('provider', ['synthetic-metric-demo', 'registered-rgbd-fused-algorithm'])
def test_time_anomalies_publish_block_and_preserve_valid_source(transport, provider):
    bridge, executor, worlds, observations, clock, states, wait = transport
    guard = bridge.algorithm_observation_guard if provider.startswith('registered') else bridge.tracker

    def send(t, seq, *, unknown=False):
        observations.publish(observation_to_msg(sample(t, seq, provider, unknown=unknown)))
        wait(lambda: bridge.last_observation is not None and bridge.last_observation.source_sequence == seq)
        published_sequence = bridge.publisher_sequence - 1
        wait(lambda: worlds and worlds[-1].source_capture_time == float_to_time(t)
             and worlds[-1].publisher_sequence >= published_sequence
             and worlds[-1].planning_admissible == (not unknown))
        assert bridge.last_observation.processed_time == t+.01

    def rejected(message, reason):
        previous, sequence, count = bridge.last_observation, guard.sequence, len(worlds)
        tracked, tracks = bridge.last_tracked, bridge.tracker._tracks
        obstacles = list(worlds[-1].obstacles)
        epoch = getattr(guard, 'current_epoch', getattr(guard, 'epoch', None))
        observations.publish(message)
        wait(lambda: len(worlds) > count and reason in worlds[-1].blocking_reasons)
        assert not worlds[-1].planning_admissible
        assert worlds[-1].source_capture_time == float_to_time(previous.capture_time)
        assert bridge.last_observation is previous
        assert bridge.last_tracked is tracked and bridge.tracker._tracks is tracks
        assert list(worlds[-1].obstacles) == obstacles
        assert guard.sequence == sequence
        assert getattr(guard, 'current_epoch', getattr(guard, 'epoch', None)) == epoch
        assert reason in bridge.last_time_rejection['blocking_reasons']

    # Sim time zero must not admit even an otherwise valid sample or poison sequence.
    observations.publish(observation_to_msg(sample(100, 999999, provider)))
    wait(lambda: bridge.last_time_rejection is not None)
    assert 'CLOCK_NOT_INITIALIZED' in bridge.last_time_rejection['blocking_reasons']
    assert bridge.last_observation is None and guard.sequence == -1 and not worlds
    clock(100.)
    states(100., 0)
    send(100., 1)
    assert not worlds[-1].blocking_reasons

    bad = observation_to_msg(replace(sample(110., 999999, provider), source_epoch='invalid-future-epoch'))
    rejected(bad, 'OBSERVATION_TIME_IN_FUTURE')
    observations.publish(observation_to_msg(sample(100., 1, provider)))
    end = time.monotonic()+.15
    while time.monotonic() < end: executor.spin_once(timeout_sec=.02)
    assert not bridge.last_snapshot.scene_snapshot['planning_admissible']
    send(100., 2)
    for seq, domain in [(3, 'ros'), (4, '')]:
        bad = observation_to_msg(sample(100., 999999, provider))
        bad.clock_domain = domain
        rejected(bad, 'OBSERVATION_CLOCK_DOMAIN_MISMATCH')
        send(100., seq)
    bad = observation_to_msg(sample(100., 999999, provider))
    bad.exact_capture_time_s = math.nan
    rejected(bad, 'OBSERVATION_TIME_INVALID')
    send(100., 5)

    # Advance and then freeze /clock: the actual watchdog must wake itself.
    count = len(worlds)
    clock(103.)
    wait(lambda: len(worlds) > count and 'OBSERVATION_STALE' in worlds[-1].blocking_reasons)
    assert not worlds[-1].planning_admissible
    assert worlds[-1].source_capture_time == float_to_time(100.)
    clock(104.)
    states(104., 1)  # robot/mechanism callbacks must not bypass the latch
    assert not bridge.last_snapshot.scene_snapshot['planning_admissible']
    send(104., 6)

    count = len(worlds)
    clock(103.75)
    wait(lambda: len(worlds) > count and 'OBSERVATION_TIME_IN_FUTURE' in worlds[-1].blocking_reasons)
    assert not worlds[-1].planning_admissible
    assert bridge.last_observation.source_sequence == 6 and guard.sequence == 6
    assert worlds[-1].source_capture_time == float_to_time(104.)
    # Clock alone recovering cannot erase a latched anomaly.
    clock(104.125)
    states(104.125, 2)
    assert not bridge.last_snapshot.scene_snapshot['planning_admissible']
    send(104.125, 7)
    send(104.125, 8, unknown=True)
    assert 'UNKNOWN_OR_UNTRANSFORMED_REGIONS' in worlds[-1].blocking_reasons
    rejected(observation_to_msg(sample(110., 999999, provider)), 'OBSERVATION_TIME_IN_FUTURE')
    send(104.125, 9, unknown=True)
    wait(lambda: 'OBSERVATION_TIME_IN_FUTURE' not in worlds[-1].blocking_reasons)
    assert 'UNKNOWN_OR_UNTRANSFORMED_REGIONS' in worlds[-1].blocking_reasons
    count = len(worlds)
    clock(0.)
    wait(lambda: len(worlds) > count and 'CLOCK_NOT_INITIALIZED' in worlds[-1].blocking_reasons)
    assert not worlds[-1].planning_admissible and guard.sequence == 9


def test_ros_parameter_validation_and_tolerance_shared_with_watchdog(transport):
    bridge, executor, worlds, observations, clock, states, wait = transport
    for name, value in [('observation_future_tolerance_seconds', -.1),
                        ('observation_future_tolerance_seconds', math.nan),
                        ('snapshot_freshness_seconds', math.inf), ('snapshot_freshness_seconds', 0.)]:
        result = bridge.set_parameters([Parameter(name, value=value)])
        assert not result[0].successful
    assert bridge.get_parameter('snapshot_freshness_seconds').value == 2.
    assert bridge.set_parameters([Parameter('observation_future_tolerance_seconds', value=.125)])[0].successful
    clock(100.)
    states(100., 0)
    observations.publish(observation_to_msg(sample(100.125, 1, 'synthetic-metric-demo')))
    wait(lambda: worlds and worlds[-1].planning_admissible)
    end = time.monotonic()+.3
    while time.monotonic() < end: executor.spin_once(timeout_sec=.02)
    assert all(w.planning_admissible for w in worlds)
    assert worlds[-1].source_capture_time == float_to_time(100.125)
