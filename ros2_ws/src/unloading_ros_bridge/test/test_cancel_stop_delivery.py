"""Real mock action/DDS; test receiver defers callbacks into controlled orders.

The production controller, source timestamps and stop criteria are unchanged.
This controls application delivery, not the network's arrival order.
"""
from itertools import count, permutations
import os
from threading import Thread

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from unloading_contracts import PlanArtifactKind, TimedJointPoint, TimedJointTrajectory, canonical_fingerprint
from unloading_interfaces.msg import ExecutionCancel, ExecutionGrant, StopAcknowledgement
from unloading_ros_bridge.common import float_to_time, time_to_float
from unloading_ros_bridge.execution_bridge_node import ExecutionBridgeNode
import test_world_heartbeat as heartbeat


_domains = count(50 + os.getpid() % 20)


class DeferredBridge(ExecutionBridgeNode):
    def __init__(self):
        self.deliveries = {}
        super().__init__()

    def on_cancel_response(self, command, future):
        self.deliveries['response'] = (command, future)

    def on_result(self, command, future):
        self.deliveries['result'] = (command, future)

    def on_stop_fact(self, message):
        self.deliveries['fact'] = (message,)

    def deliver(self, event):
        method = {'response': ExecutionBridgeNode.on_cancel_response,
                  'result': ExecutionBridgeNode.on_result, 'fact': ExecutionBridgeNode.on_stop_fact}[event]
        method(self, *self.deliveries[event])


@pytest.fixture
def graph(monkeypatch):
    monkeypatch.setattr(heartbeat, 'ExecutionBridgeNode', DeferredBridge)
    domain = next(_domains)
    # rclpy rosout registration is keyed by logger name across contexts. Keep
    # these deferred, Future-owning nodes distinct from other test graphs.
    rclpy.init(args=['--ros-args', '-p', 'controller_epoch:=synthetic-controller',
                    '-r', f'unloading_execution_bridge:__node:=ordered_cancel_bridge_{domain}'],
               domain_id=domain)
    graph = heartbeat.SyntheticInputs()
    # Match the controller executable's executor: its synchronous execute loop
    # must not block its cancel service. The execution bridge stays single-threaded.
    graph.executor.remove_node(graph.controller)
    controller_executor = MultiThreadedExecutor(num_threads=2)
    controller_executor.add_node(graph.controller)
    worker = Thread(target=controller_executor.spin, daemon=True)
    worker.start()
    try:
        yield graph
    finally:
        controller_executor.shutdown()
        worker.join(timeout=5.)
        graph.execution.deliveries.clear()
        graph.execution.client.destroy()
        graph.controller.server.destroy()
        graph.close()
        rclpy.shutdown()


@pytest.mark.parametrize('order', list(permutations(('response', 'result', 'fact'))))
def test_real_action_callbacks_confirm_stop_in_every_delivery_order(graph, order):
    graph.ready()
    acks = []
    graph.node.create_subscription(StopAcknowledgement, '/unloading/stop_acknowledgements', acks.append, 10)
    cancels = graph.node.create_publisher(ExecutionCancel, '/unloading/execution_cancel', 10)
    graph.wait(lambda: cancels.get_subscription_count() == 1
               and graph.execution.stop_acknowledgements.get_subscription_count() > 0)
    command = graph.command('ordered-cancel', grant=False)
    # Enough trajectory points to cancel while active; no controller sleep or
    # production threshold changes. All motion here is mock and holds position.
    q = graph.execution.current_world.current_q
    points = tuple(TimedJointPoint(q, index / 100., (0.,)*6) for index in range(100))
    trajectory = TimedJointTrajectory(tuple(command.trajectory.joint_names), points,
        PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, command.validation_reference,
        command.robot_model_fingerprint, command.config_identity)
    command.trajectory.points = [JointTrajectoryPoint(positions=list(p.positions), velocities=list(p.velocities),
        time_from_start=Duration(sec=0, nanosec=index*10_000_000)) for index, p in enumerate(points)]
    graph.grants.publish(ExecutionGrant(schema_version='1.1.0', grant_id=command.command_id,
        command_id=command.command_id, plan_id=command.plan_id, request_id=command.request_id,
        session_id=command.session_id, epoch=command.epoch, planning_generation=command.planning_generation,
        world_fingerprint=command.world_fingerprint, robot_model_fingerprint=command.robot_model_fingerprint,
        config_identity=command.config_identity, validation_reference=command.validation_reference,
        validation_generation=command.validation_generation, trajectory_fingerprint=canonical_fingerprint(trajectory),
        expires_at=float_to_time(graph.now()+10.), clock_domain='ros', mock_only=True))
    graph.wait(lambda: command.command_id in graph.execution.gate._grants)
    graph.send(command, 'STARTED')
    cancels.publish(ExecutionCancel(schema_version='1.1.0', command_id=command.command_id,
        plan_id=command.plan_id, epoch=command.epoch, planning_generation=command.planning_generation,
        reason='controlled callback-order regression'))
    graph.wait(lambda: set(graph.execution.deliveries) == {'response', 'result', 'fact'})
    measured = graph.execution.deliveries['fact'][0]
    received_response = graph.execution.deliveries['response'][1].result()
    assert received_response.return_code == 0
    assert measured.goal_id in {bytes(info.goal_id.uuid).hex() for info in received_response.goals_canceling}
    assert graph.execution.gate._cancel_requested_at <= time_to_float(measured.cancel_accepted_time)
    assert time_to_float(measured.cancel_accepted_time) <= time_to_float(measured.stopped_time) <= graph.now()
    assert not acks and graph.execution.gate.active_command is not None
    for index, event in enumerate(order):
        graph.execution.deliver(event)
        if not {'response', 'fact'}.issubset(set(order[:index+1])):
            assert graph.execution.gate.active_command is not None
            assert not acks
    graph.wait(lambda: len(acks) == 1)
    ack = acks[0]
    assert ack.command_id == command.command_id and ack.plan_id == command.plan_id
    assert ack.goal_id == measured.goal_id
    # The domain contract represents seconds as float; compare that exact
    # representation, without a tolerance or replacing either source timestamp.
    assert time_to_float(ack.stopped_time) == time_to_float(measured.stopped_time)
    assert ack.actual_velocities == measured.actual_velocities
    assert graph.execution.gate.active_command is None
    assert not graph.execution.handles and not graph.execution.goal_to_command
    assert not graph.execution.buffered_stop_facts
    assert graph.execution.gate._last_stop_sequence[('mock-follow-joint-trajectory', 'synthetic-controller')] == measured.sequence
    assert command.command_id in graph.execution.gate._result_seen
