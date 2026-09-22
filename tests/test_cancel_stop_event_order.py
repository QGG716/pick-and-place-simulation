"""Production callbacks/gate with ROS imports stubbed and controlled CPU time.

These are synthetic callback tests, not DDS or hardware acceptance.
"""
from dataclasses import replace
from itertools import permutations
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_execution_context_admission import bridge, authorized_command


GOAL = bytes(range(16))


def stamp(value):
    return NS(sec=int(value), nanosec=round((value-int(value))*1e9))


def response(goal=GOAL, code=0):
    return NS(result=lambda: NS(return_code=code, goals_canceling=[NS(goal_id=NS(uuid=list(goal)))]))


def fact(**changes):
    values = dict(controller_id='mock', controller_epoch='controller-a', goal_id=GOAL.hex(),
                  sequence=1, cancel_accepted_time=stamp(100.), stopped_time=stamp(100.01),
                  clock_domain='ros', joint_names=['j1', 'j2'], actual_positions=[.2, .3],
                  actual_velocities=[0., 0.], evidence_reference='measured-stop')
    values.update(changes)
    return NS(**values)


@pytest.fixture
def cancel_bridge(bridge):
    # The callback fixture loads a real module without retaining it in sys.modules.
    globals_ = bridge.on_result.__func__.__globals__
    globals_['GoalStatus'] = NS(STATUS_CANCELED=5, STATUS_SUCCEEDED=4)
    globals_['FollowJointTrajectory'] = NS(Result=NS(SUCCESSFUL=0))
    globals_['StopAcknowledgement'] = NS
    globals_['float_to_time'] = stamp
    message = authorized_command(bridge)
    command = bridge._command(message)
    bridge.gate.authorize(command, bridge.current_world, epoch=command.epoch,
                          generation=command.planning_generation, now=99.9, clock_domain='ros')
    bridge.gate.bind_goal(command.command_id, controller_id='mock', controller_epoch='controller-a',
                          goal_id=GOAL.hex())
    bridge.handles = {command.command_id: Mock(goal_id=NS(uuid=list(GOAL)))}
    bridge.goal_to_command = {GOAL.hex(): command.command_id}
    bridge.command_messages = {command.command_id: message}
    bridge.pending_cancel = set()
    bridge.cancel_requested = set()
    bridge.buffered_stop_facts = {}
    bridge.params['stop_fact_max_age_seconds'] = 1.
    bridge.stop_acknowledgements = Mock()
    bridge.command = command
    bridge.now = 99.99
    bridge._request_controller_cancel(command, bridge.handles[command.command_id])
    return bridge


def deliver(node, kind):
    if kind == 'response':
        node.on_cancel_response(node.command, response())
    elif kind == 'fact':
        node.on_stop_fact(fact())
    else:
        node.on_result(node.command, NS(result=lambda: NS(status=5,
                       result=NS(error_code=0, error_string='canceled'))))


def test_response_receipt_is_not_physical_stop_lower_bound(cancel_bridge):
    node = cancel_bridge
    node.now = 100.03
    deliver(node, 'response')
    node.now = 100.04
    deliver(node, 'fact')
    node.stop_acknowledgements.publish.assert_called_once()
    assert node.gate.active_command is None


@pytest.mark.parametrize('order', list(permutations(('response', 'result', 'fact'))))
def test_all_delivery_orders_produce_the_same_measured_stop(cancel_bridge, order):
    node = cancel_bridge
    for index, kind in enumerate(order):
        node.now = 100.03 + index*.01
        deliver(node, kind)
        if not {'response', 'fact'}.issubset(set(order[:index+1])):
            node.stop_acknowledgements.publish.assert_not_called()
            assert node.gate.active_command == node.command
    node.stop_acknowledgements.publish.assert_called_once()
    ack = node.stop_acknowledgements.publish.call_args.args[0]
    assert ack.command_id == node.command.command_id
    assert ack.plan_id == node.command.plan_id
    assert ack.stopped_time == stamp(100.01)
    assert ack.goal_id == GOAL.hex() and ack.stop_sequence == 1
    assert node.gate.active_command is None
    assert not node.handles and not node.goal_to_command and not node.buffered_stop_facts


@pytest.mark.parametrize('changes,reason', [
    ({'goal_id': 'other-goal'}, 'identity mismatch'),
    ({'controller_id': 'other-controller'}, 'identity mismatch'),
    ({'controller_epoch': 'old-controller'}, 'identity mismatch'),
    ({'clock_domain': 'monotonic'}, 'clock domain'),
    ({'cancel_accepted_time': stamp(98.), 'stopped_time': stamp(98.01)}, 'stale'),
    ({'cancel_accepted_time': stamp(101.), 'stopped_time': stamp(101.01)}, 'future'),
    ({'cancel_accepted_time': stamp(100.02)}, 'sequence/time'),
    ({'cancel_accepted_time': stamp(99.98)}, 'predates'),
    ({'actual_velocities': [0., .002]}, 'near-zero'),
    ({'joint_names': ['j2', 'j1']}, 'joint order'),
    ({'stopped_time': NS(sec=100, nanosec=10**9)}, 'encoding'),
    ({'stopped_time': NS(sec=100, nanosec=-1)}, 'encoding'),
])
@pytest.mark.parametrize('response_first', [False, True])
def test_bad_facts_do_not_mutate_or_poison_later_valid_fact(cancel_bridge, changes, reason, response_first):
    node = cancel_bridge
    node.now = 100.03
    if response_first:
        deliver(node, 'response')
    previous = (node.gate._cancel_requested_at, node.gate._cancel_event)
    node.on_stop_fact(fact(**changes))
    node.stop_acknowledgements.publish.assert_not_called()
    assert node.gate.active_command == node.command
    assert (node.gate._cancel_requested_at, node.gate._cancel_event) == previous
    assert not node.buffered_stop_facts and not node.gate._last_stop_sequence
    assert reason in node.get_logger().error.call_args.args[0]
    deliver(node, 'fact')
    if not response_first:
        deliver(node, 'response')
    node.stop_acknowledgements.publish.assert_called_once()


@pytest.mark.parametrize('invalid', [response(bytes(reversed(GOAL))), response(code=1),
                                  response(code=2), response(code=3),
                                  NS(result=lambda: NS(return_code=0, goals_canceling=[])),
                                  NS(result=lambda: NS(goals_canceling=[True]))])
def test_response_requires_success_and_exact_goal(cancel_bridge, invalid):
    node = cancel_bridge
    node.now = 100.03
    deliver(node, 'fact')
    node.on_cancel_response(node.command, invalid)
    assert node.gate._cancel_event is None
    node.stop_acknowledgements.publish.assert_not_called()
    assert node.gate.active_command == node.command
    deliver(node, 'response')
    node.stop_acknowledgements.publish.assert_called_once()


def test_unsolicited_response_and_fact_cannot_prove_a_local_request(cancel_bridge):
    node = cancel_bridge
    node.cancel_requested.clear()
    node.gate._cancel_requested_at = None  # fixture: no local request was sent
    node.gate._cancel_clock_domain = None
    node.now = 100.03
    deliver(node, 'response')
    deliver(node, 'fact')
    assert node.gate._cancel_event is None and not node.buffered_stop_facts
    node.stop_acknowledgements.publish.assert_not_called()
    with pytest.raises(ValueError, match='local request'):
        node.gate.accept_cancel(node.command.command_id, goal_id=GOAL.hex(), event_time=node.now, clock_domain='ros')


def test_response_cannot_change_another_command_with_the_same_id(cancel_bridge):
    node = cancel_bridge
    node.now = 100.03
    node.on_cancel_response(replace(node.command, plan_id='wrong-plan'), response())
    assert node.gate._cancel_event is None
    node.on_cancel(NS(command_id=node.command.command_id, plan_id='wrong-plan',
                      epoch=node.command.epoch, planning_generation=node.command.planning_generation))
    assert node.reject.call_args.args[1] == 'CANCEL_IDENTITY_MISMATCH'
    deliver(node, 'response')
    deliver(node, 'fact')
    node.stop_acknowledgements.publish.assert_called_once()


def test_duplicate_callbacks_and_facts_are_idempotent(cancel_bridge):
    node = cancel_bridge
    node.now = 100.03
    for kind in ('fact', 'fact', 'result', 'result', 'response', 'response', 'fact', 'result'):
        deliver(node, kind)
    acks = [call.args[0] for call in node.stop_acknowledgements.publish.call_args_list]
    assert len(acks) == 2 and acks[0] == acks[1]
    assert node.gate._last_stop_sequence == {('mock', 'controller-a'): 1}
    assert node.gate.active_command is None
    assert not node.goal_to_command and not node.buffered_stop_facts


def test_conflicting_duplicate_cannot_reuse_cached_acknowledgement(cancel_bridge):
    node = cancel_bridge
    node.now = 100.03
    deliver(node, 'response')
    deliver(node, 'fact')
    node.on_stop_fact(fact(actual_velocities=[1., 1.]))
    node.stop_acknowledgements.publish.assert_called_once()
    assert 'conflicting duplicate' in node.get_logger().error.call_args.args[0]
    node.now = 101.010000001
    deliver(node, 'fact')
    node.stop_acknowledgements.publish.assert_called_once()
    assert 'stale' in node.get_logger().error.call_args.args[0]


def test_only_canceled_result_keeps_active_goal_and_correlation(cancel_bridge):
    node = cancel_bridge
    node.now = 100.03
    deliver(node, 'result')
    deliver(node, 'response')
    node.stop_acknowledgements.publish.assert_not_called()
    assert node.gate.active_command == node.command
    assert node.goal_to_command[GOAL.hex()] == node.command.command_id


def test_normal_success_still_cleans_up_without_a_stop_ack(cancel_bridge):
    node = cancel_bridge
    node.gate._cancel_requested_at = None  # fixture: uncanceled normal success
    node.cancel_requested.clear()
    node.now = 100.03
    node.on_result(node.command, NS(result=lambda: NS(status=4, result=NS(error_code=0, error_string='done'))))
    assert node.gate.active_command is None
    assert not node.goal_to_command and not node.handles
    node.stop_acknowledgements.publish.assert_not_called()


def test_expired_buffer_record_does_not_block_new_valid_record(cancel_bridge):
    node = cancel_bridge
    node.now = 100.02
    deliver(node, 'fact')
    node.now = 101.1
    node.on_stop_fact(fact(sequence=2, stopped_time=stamp(101.09)))
    deliver(node, 'response')
    assert 'stale' in node.get_logger().error.call_args.args[0]
    node.stop_acknowledgements.publish.assert_called_once()
    assert node.stop_acknowledgements.publish.call_args.args[0].stop_sequence == 2


def test_pending_buffer_deduplicates_and_has_a_hard_capacity(cancel_bridge):
    node = cancel_bridge
    node.now = 100.03
    for _ in range(12):
        deliver(node, 'fact')
    assert len(node.buffered_stop_facts[GOAL.hex()]) == 1
    for sequence in range(2, 12):
        node.on_stop_fact(fact(sequence=sequence))
    assert len(node.buffered_stop_facts[GOAL.hex()]) == 8
    assert 'buffer is full' in node.get_logger().error.call_args.args[0]
    deliver(node, 'response')
    node.stop_acknowledgements.publish.assert_called_once()
    assert not node.buffered_stop_facts


def test_out_of_order_sequence_is_rejected_before_caching(cancel_bridge):
    node = cancel_bridge
    node.gate._last_stop_sequence[('mock', 'controller-a')] = 2
    node.now = 100.03
    deliver(node, 'fact')
    assert 'out-of-order' in node.get_logger().error.call_args.args[0]
    assert not node.buffered_stop_facts
    node.on_stop_fact(fact(sequence=3))
    deliver(node, 'response')
    node.stop_acknowledgements.publish.assert_called_once()


def test_source_cancel_must_not_be_later_than_response_receipt(cancel_bridge):
    node = cancel_bridge
    node.now = 100.03
    deliver(node, 'response')
    node.now = 100.05
    node.on_stop_fact(fact(cancel_accepted_time=stamp(100.04), stopped_time=stamp(100.05)))
    assert 'after response receipt' in node.get_logger().error.call_args.args[0]
    node.stop_acknowledgements.publish.assert_not_called()
    deliver(node, 'fact')
    node.stop_acknowledgements.publish.assert_called_once()


@pytest.mark.parametrize('invalid_time', [0., float('nan'), float('inf')])
def test_invalid_request_time_does_not_consume_pending_cancel(cancel_bridge, invalid_time):
    node = cancel_bridge
    # Re-establish a bound goal awaiting its first local request.
    node.cancel_requested.clear()
    node.gate._cancel_requested_at = node.gate._cancel_clock_domain = None
    node.pending_cancel.add(node.command.command_id)
    handle = node.handles[node.command.command_id]
    handle.cancel_goal_async.reset_mock()
    node.now = invalid_time
    node._request_controller_cancel(node.command, handle)
    assert node.command.command_id in node.pending_cancel
    assert not node.cancel_requested and node.gate._cancel_requested_at is None
    handle.cancel_goal_async.assert_not_called()


@pytest.mark.parametrize('age,accepted', [(1., True), (1.000000001, False)])
def test_stop_freshness_boundary_is_unchanged(cancel_bridge, age, accepted):
    node = cancel_bridge
    node.now = 100.03
    deliver(node, 'response')
    node.now = 100.01 + age
    deliver(node, 'fact')
    assert bool(node.stop_acknowledgements.publish.call_count) == accepted


def test_retry_does_not_move_request_time_or_send_twice(cancel_bridge):
    node = cancel_bridge
    handle = node.handles[node.command.command_id]
    node.now = 100.03
    node._request_controller_cancel(node.command, handle)
    handle.cancel_goal_async.assert_called_once()
    assert node.gate._cancel_requested_at == 99.99
    with pytest.raises(ValueError, match='goal identity'):
        node.gate.accept_cancel(node.command.command_id, goal_id='wrong', event_time=node.now, clock_domain='ros')
    assert node.gate._cancel_event is None
