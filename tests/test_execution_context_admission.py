"""CPU callback tests of the production bridge; ROS transport is stubbed here."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock
from dataclasses import replace

import pytest

from unloading_perception.execution import ExecutionGate
from unloading_perception.execution_context import ExecutionContextGuard, positive_age_limit


@pytest.fixture
def bridge(monkeypatch):
    root = Path(__file__).parents[1] / 'ros2_ws/src/unloading_ros_bridge/unloading_ros_bridge'
    for name, attrs in {
        'rclpy': {}, 'action_msgs.msg': {'GoalStatus': Mock()},
        'control_msgs.action': {'FollowJointTrajectory': Mock()},
        'rclpy.action': {'ActionClient': Mock()},
        'rclpy.callback_groups': {'MutuallyExclusiveCallbackGroup': Mock()},
        'rclpy.executors': {'SingleThreadedExecutor': Mock()},
        'rclpy.node': {'Node': object},
        'rclpy.qos': {'QoSProfile': Mock(), 'ReliabilityPolicy': Mock()},
        'unloading_interfaces.msg': {name: Mock() for name in (
            'ControllerStopFact', 'ExecutionAuthorization', 'ExecutionCancel',
            'ExecutionContext', 'ExecutionEvent', 'ExecutionGrant',
            'PlanningWorldSnapshot', 'StopAcknowledgement')},
        '_context_test': {'__path__': [str(root)]},
        '_context_test.mapping': {'snapshot_from_msg': Mock()},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location('_context_test.execution_bridge_node', root / 'execution_bridge_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.ExecutionBridgeNode.__new__(module.ExecutionBridgeNode)
    node.now = 100.
    node._now = lambda: node.now
    node.params = dict(context_source_max_age_seconds=1., context_receive_max_age_seconds=1.,
                       world_receive_max_age_seconds=1., robot_state_max_age_seconds=.5,
                       mechanism_state_max_age_seconds=2.)
    node.get_parameter = lambda name: NS(value=node.params[name])
    node.get_logger = Mock(return_value=Mock())
    node.gate = ExecutionGate()
    node.current_context = node.current_context_received_at = None
    node.current_world = object()
    node.current_world_received_at = node.current_robot_sample_time = node.current_mechanism_sample_time = 100.
    node.client = Mock()
    node.reject = Mock()
    if hasattr(module, 'ExecutionContextGuard'):
        node.context_guard = module.ExecutionContextGuard()
    return node


def context(stamp=100., **changes):
    value = NS(schema_version='1.2.0', session_id='session', epoch='execution-a',
               planning_generation=2, allowed_plan_id='plan', predecessor_plan_id='',
               observed_time=NS(sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9)),
               clock_domain='ros', publisher_epoch='publisher-a', publisher_sequence=0,
               publisher_restart=True)
    value.__dict__.update(changes)
    return value


def authorized_command(bridge):
    """Valid world, matching grant and real command mapping; only ROS is stubbed."""
    from test_perception_integration import admissible_snapshot, command, grant
    bridge.current_world = admissible_snapshot()
    value = command(bridge.current_world, epoch='execution-a')
    value = replace(value, session_id='session', plan_id='plan')
    value = replace(value, trajectory=replace(value.trajectory, source=value.validation_reference))
    bridge.gate.register_grant(replace(grant(value, expires_at=101.), clock_domain='ros'))
    fields = {name: getattr(value, name) for name in (
        'command_id', 'plan_id', 'request_id', 'session_id', 'epoch', 'planning_generation',
        'predecessor_plan_id', 'world_fingerprint', 'robot_model_fingerprint', 'config_identity',
        'validation_reference', 'validation_generation')}
    fields['trajectory'] = NS(joint_names=list(value.trajectory.joint_names), points=[
        NS(positions=p.positions, velocities=p.velocities, accelerations=[],
           time_from_start=NS(sec=int(p.time_from_start), nanosec=0)) for p in value.trajectory.points])
    bridge.publish_event = Mock()
    bridge.command_messages = {}
    return NS(**fields)


def test_old_source_just_received_is_rejected(bridge):
    command = authorized_command(bridge)
    bridge.on_context(context(1.))
    assert bridge.current_context is None
    bridge.on_command(command)
    bridge.client.send_goal_async.assert_not_called()


def test_fresh_context_sends_valid_authorized_command(bridge):
    command = authorized_command(bridge)
    bridge.on_context(context())
    bridge.on_command(command)
    bridge.reject.assert_not_called()
    assert bridge.gate.send_committed, bridge.publish_event.call_args
    bridge.client.send_goal_async.assert_called_once()


def test_retired_execution_cannot_return_with_new_publisher(bridge):
    bridge.on_context(context())
    bridge.now = 100.1
    second = context(100.1, epoch='execution-b', publisher_epoch='publisher-b')
    bridge.on_context(second)
    assert bridge.current_context is second
    bridge.now = 100.2
    bridge.on_context(context(100.2, publisher_epoch='publisher-c'))
    assert bridge.current_context is second


@pytest.mark.parametrize('stamp,accepted', [(99., True), (98.999999999, False), (100., True)])
def test_source_age_inclusive_boundary(bridge, stamp, accepted):
    bridge.on_context(context(stamp))
    assert (bridge.current_context is not None) == accepted


def test_source_expires_before_receive_age_and_blocks_send(bridge):
    command = authorized_command(bridge)
    bridge.on_context(context(99.25))
    bridge.now = 100.3
    assert bridge.now - bridge.current_context_received_at < 1.
    bridge.on_command(command)
    assert bridge.reject.call_args.args[1] == 'EXECUTION_CONTEXT_SOURCE_STALE'
    bridge.client.send_goal_async.assert_not_called()


def test_receive_age_is_independent(bridge):
    bridge.params['context_receive_max_age_seconds'] = .1
    bridge.on_context(context())
    bridge.now = 100.2
    assert bridge._authoritative_state_error() == 'EXECUTION_CONTEXT_STALE_OR_TIME_JUMP'


@pytest.mark.parametrize('sequence', [1, 2])
def test_late_join_then_duplicate_or_backward_sequence(bridge, sequence):
    first = context(publisher_sequence=2, publisher_restart=False)
    bridge.on_context(first)
    assert bridge.current_context is first
    bridge.now = 100.1
    bridge.on_context(context(100.1, publisher_sequence=sequence, publisher_restart=False))
    assert bridge.current_context is first
    assert bridge.current_context_received_at == 100.
    assert bridge.context_guard.current.sequence == 2


@pytest.mark.parametrize('changes,stamp,reason', [
    ({'publisher_restart': False}, 100., 'SEQUENCE_NOT_INCREASING'),
    ({'publisher_restart': False, 'publisher_sequence': 1}, 100., 'TIME_NOT_INCREASING'),
    ({'publisher_restart': False, 'publisher_sequence': 1, 'planning_generation': 1}, 100.1, 'GENERATION_ROLLBACK'),
    ({'publisher_epoch': 'publisher-b', 'publisher_restart': False}, 100.1, 'EXPLICIT_RESTART'),
    ({'publisher_epoch': 'publisher-b', 'planning_generation': 1}, 100.1, 'GENERATION_ROLLBACK'),
    ({'publisher_epoch': 'publisher-b'}, 1., 'SOURCE_STALE'),
    ({'publisher_epoch': 'publisher-b', 'session_id': ''}, 100.1, 'INVALID_SESSION'),
    ({'publisher_epoch': ''}, 100.1, 'INVALID_PUBLISHER_EPOCH'),
    ({'publisher_sequence': -1}, 100.1, 'INVALID_PUBLISHER_SEQUENCE'),
    ({'publisher_sequence': 2}, 100.1, 'RESTART_REQUIRES_SEQUENCE_ZERO'),
    ({'clock_domain': 'ros_sim_time'}, 100.1, 'CLOCK_DOMAIN_MISMATCH'),
    ({'schema_version': '1.1.0'}, 100.1, 'SCHEMA_MISMATCH'),
    ({}, 0., 'SOURCE_TIME_INVALID'),
    ({}, 101., 'SOURCE_TIME_IN_FUTURE'),
    ({'observed_time': NS(sec=100, nanosec=10**9)}, 100.1, 'encoding'),
    ({'observed_time': NS(sec=100, nanosec=-1)}, 100.1, 'encoding'),
    ({'observed_time': NS(sec=2**31, nanosec=0)}, 100.1, 'encoding'),
])
def test_rejection_is_atomic_and_legal_heartbeat_recovers(bridge, changes, stamp, reason):
    first = context()
    bridge.on_context(first)
    snapshot = bridge.context_guard.current
    grant = NS(expires_at=101.)
    bridge.gate._grants['command'] = grant
    bridge.now = 100.1
    bridge.on_context(context(stamp, **changes))
    assert bridge.current_context is first
    assert bridge.current_context_received_at == 100.
    assert bridge.context_guard.current == snapshot
    assert not bridge.context_guard._retired_publishers
    assert not bridge.context_guard._retired_business
    assert reason in bridge.get_logger().error.call_args.args[0]
    heartbeat = context(100.1, publisher_sequence=1, publisher_restart=False)
    bridge.on_context(heartbeat)
    assert bridge.current_context is heartbeat
    assert bridge.current_context_received_at == 100.1
    assert bridge.gate._grants['command'] is grant
    assert grant.expires_at == 101.


@pytest.mark.parametrize('changes', [
    {'predecessor_plan_id': 'previous'}, {'allowed_plan_id': 'new-plan'},
    {'planning_generation': 3}, {'epoch': 'execution-b'}, {'session_id': 'session-b'},
    {'publisher_epoch': 'publisher-b', 'publisher_sequence': 0, 'publisher_restart': True},
])
@pytest.mark.parametrize('committed', [False, True])
def test_authority_changes_revoke_pending_but_preserve_sent_command(bridge, changes, committed):
    from unloading_contracts import ExecutionCommand
    from test_perception_integration import command, complete_assembler, observation
    from unloading_perception.scene import build_scene_update
    world = complete_assembler(build_scene_update(observation())).assemble().snapshot
    active = command(world)
    assert isinstance(active, ExecutionCommand)
    bridge.on_context(context())
    bridge.gate._grants['pending'] = NS(expires_at=101.)
    bridge.gate._active = active
    bridge.gate._reservation = object()
    bridge.gate._send_committed = committed
    bridge.now = 100.1
    fields = dict(publisher_sequence=1, publisher_restart=False)
    fields.update(changes)
    bridge.on_context(context(100.1, **fields))
    assert not bridge.gate._grants
    assert (bridge.gate.active_command is active) == committed
    if not committed:
        assert bridge.gate._reservation is None


def test_publisher_and_business_retirement_are_independent(bridge):
    bridge.on_context(context())
    bridge.now = 100.1
    # Same publisher switches the business identity, without a restart.
    second = context(100.1, epoch='execution-b', publisher_sequence=1, publisher_restart=False)
    bridge.on_context(second)
    assert bridge.current_context is second
    bridge.now = 100.2
    # A restart cannot reactivate the retired business.
    bridge.on_context(context(100.2, publisher_epoch='publisher-b'))
    assert bridge.current_context is second
    # Legal publisher restart retains business generation constraints.
    third = context(100.2, epoch='execution-b', publisher_epoch='publisher-b')
    bridge.on_context(third)
    assert bridge.current_context is third
    bridge.now = 100.3
    bridge.on_context(context(100.3, epoch='execution-b'))
    assert bridge.current_context is third
    assert 'RETIRED_PUBLISHER' in bridge.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('kind', ['publisher', 'business'])
def test_capacity_refuses_transitions_without_forgetting_history(bridge, kind):
    bridge.context_guard = ExecutionContextGuard(retired_capacity=1)
    bridge.on_context(context())
    bridge.now = 100.1
    changes = (dict(publisher_epoch='publisher-b') if kind == 'publisher' else
               dict(epoch='execution-b', publisher_sequence=1, publisher_restart=False))
    second = context(100.1, **changes)
    bridge.on_context(second)
    assert bridge.current_context is second
    bridge.now = 100.2
    changes = (dict(publisher_epoch='publisher-c') if kind == 'publisher' else
               dict(epoch='execution-c', publisher_sequence=2, publisher_restart=False))
    bridge.on_context(context(100.2, **changes))
    assert bridge.current_context is second
    assert 'HISTORY_FULL' in bridge.get_logger().error.call_args.args[0]
    fields = vars(second).copy()
    fields.pop('observed_time')
    fields.update(publisher_sequence=second.publisher_sequence+1, publisher_restart=False)
    bridge.on_context(context(100.2, **fields))
    assert bridge.current_context_received_at == 100.2
    bridge.now = 100.3
    bridge.on_context(context(100.3))
    assert bridge.current_context_received_at == 100.2


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), -float('inf')])
@pytest.mark.parametrize('parameter', ['context_source_max_age_seconds', 'context_receive_max_age_seconds'])
def test_invalid_age_limits_fail_closed(bridge, value, parameter):
    with pytest.raises(ValueError, match='finite and positive'):
        positive_age_limit(value, parameter)
    bridge.on_context(context())
    previous = bridge.context_guard.current
    bridge.params[parameter] = value
    bridge.now = 100.1
    bridge.on_context(context(100.1, publisher_sequence=1, publisher_restart=False))
    assert bridge.context_guard.current == previous
    bridge.on_command(NS(command_id='command'))
    assert 'EXECUTION_CONTEXT_INVALID' in bridge.reject.call_args.args[1]
    bridge.client.send_goal_async.assert_not_called()
