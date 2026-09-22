"""Real Humble messages, DDS and /clock; no hardware, GPU or perception model."""
from copy import deepcopy
from itertools import count
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Log
from rosgraph_msgs.msg import Clock
from unloading_interfaces.msg import ExecutionContext
from unloading_ros_bridge.execution_bridge_node import ExecutionBridgeNode
from unloading_ros_bridge.common import float_to_time


_graphs = count()


@pytest.fixture
def transport():
    graph_id = next(_graphs)
    rclpy.init(args=['--ros-args', '-p', 'controller_epoch:=context-test-controller',
                    '-p', 'use_sim_time:=true', '-r',
                    f'unloading_execution_bridge:__node:=context_execution_bridge_{graph_id}'],
               domain_id=110 + os.getpid() % 40)
    bridge = ExecutionBridgeNode()
    source = rclpy.create_node(f'context_test_source_{graph_id}')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(source)
    clocks = source.create_publisher(Clock, '/clock', 10)
    contexts = source.create_publisher(ExecutionContext, '/unloading/execution_context', 10)
    errors = []
    source.create_subscription(Log, '/rosout', lambda log: errors.append(log.msg)
                               if log.name == bridge.get_name() and log.msg.startswith('rejecting execution context:') else None, 100)

    def wait(predicate):
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(f'DDS condition not reached: clock_subscribers={clocks.get_subscription_count()}, '
                             f'context_subscribers={contexts.get_subscription_count()}, '
                             f'rosout_publishers={[p.node_name for p in source.get_publishers_info_by_topic("/rosout")]}')

    def advance(now):
        clocks.publish(Clock(clock=float_to_time(now)))
        wait(lambda: bridge._now() == now)

    def send(message, accepted):
        previous_errors = len(errors)
        contexts.publish(message)
        if accepted:
            wait(lambda: bridge.current_context == message)
        else:
            wait(lambda: len(errors) > previous_errors)

    try:
        wait(lambda: clocks.get_subscription_count() >= 2 and contexts.get_subscription_count() == 1
             and source.count_publishers('/rosout') >= 2)
        advance(100.)
        yield bridge, advance, send, errors
    finally:
        executor.shutdown()
        bridge.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def context(stamp=100., **fields):
    values = dict(schema_version='1.2.0', session_id='context-session', epoch='execution-a',
                  planning_generation=2, allowed_plan_id='plan', predecessor_plan_id='',
                  observed_time=float_to_time(stamp), clock_domain='ros',
                  publisher_epoch='publisher-a', publisher_sequence=0, publisher_restart=True)
    values.update(fields)
    return ExecutionContext(**values)


def test_dds_rejects_old_source_then_accepts_fresh_context(transport):
    bridge, advance, send, errors = transport
    send(context(1.), False)
    assert bridge.current_context is None
    assert 'SOURCE_STALE' in errors[-1]
    first = context(99.)  # inclusive source-age boundary
    send(first, True)
    advance(100.25)
    bridge.current_world_received_at = bridge.current_robot_sample_time = bridge.current_mechanism_sample_time = 100.25
    assert bridge.current_context_received_at == 100.
    assert bridge._authoritative_state_error() == 'EXECUTION_CONTEXT_SOURCE_STALE'
    heartbeat = context(100.25, publisher_sequence=1, publisher_restart=False)
    send(heartbeat, True)
    assert bridge._authoritative_state_error() is None
    advance(100.5)
    send(deepcopy(heartbeat), False)
    assert 'SEQUENCE_NOT_INCREASING' in errors[-1]
    assert bridge.current_context_received_at == 100.25
    repeated_sample = deepcopy(heartbeat)
    repeated_sample.publisher_sequence += 1
    send(repeated_sample, False)
    assert 'TIME_NOT_INCREASING' in errors[-1]
    assert bridge.current_context_received_at == 100.25


def test_dds_takeover_is_atomic_and_cannot_restore_retired_identity(transport):
    bridge, advance, send, errors = transport
    first = context()
    send(first, True)
    advance(100.25)
    send(context(1., publisher_epoch='publisher-b', epoch='execution-b'), False)
    assert bridge.context_guard.current.publisher == 'publisher-a'
    second = context(100.25, publisher_epoch='publisher-b', epoch='execution-b')
    send(second, True)
    advance(100.5)
    send(context(100.5), False)
    assert 'RETIRED_PUBLISHER' in errors[-1]
    send(context(100.5, publisher_epoch='publisher-c'), False)
    assert 'RETIRED_EXECUTION' in errors[-1]
    assert bridge.current_context == second
    assert bridge.current_context_received_at == 100.25
    send(context(100.5, publisher_epoch='publisher-b', epoch='execution-b',
                 publisher_sequence=1, publisher_restart=False), True)
