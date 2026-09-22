"""Synthetic online DDS states + real world/execution bridges and mock action.

No model, Isaac, hardware or planning. Production thresholds/gates are unchanged.
"""
from copy import deepcopy
from dataclasses import replace
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from unloading_contracts import PlanArtifactKind, TimedJointPoint, TimedJointTrajectory, canonical_fingerprint
from unloading_interfaces.msg import (ExecutionAuthorization, ExecutionContext, ExecutionEvent,
    ExecutionGrant, MechanismState, PerceptionObservation, PlanningWorldSnapshot)
from unloading_perception.demo import _synthetic_observation
from unloading_ros_bridge.common import float_to_time, time_to_float
from unloading_ros_bridge.mapping import observation_to_msg, snapshot_from_msg
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode
from unloading_ros_bridge.execution_bridge_node import ExecutionBridgeNode
from unloading_ros_bridge.mock_follow_joint_trajectory import MockTrajectoryServer
from test_perception_time_transport import transport, sample


class SyntheticInputs:
    def __init__(self):
        self.world = WorldBridgeNode()
        self.execution = ExecutionBridgeNode()
        self.controller = MockTrajectoryServer()
        self.node = rclpy.create_node('synthetic_heartbeat_inputs')
        self.executor = SingleThreadedExecutor()
        for node in (self.world, self.execution, self.controller, self.node): self.executor.add_node(node)
        self.worlds, self.events = [], []
        self.node.create_subscription(PlanningWorldSnapshot, '/unloading/world_snapshot', self.worlds.append, 10)
        self.node.create_subscription(ExecutionEvent, '/unloading/execution_events', self.events.append, 10)
        self.joints = self.node.create_publisher(JointState, '/joint_states', 10)
        self.mechanisms = self.node.create_publisher(MechanismState, '/unloading/mechanism_state', 10)
        self.perceptions = self.node.create_publisher(PerceptionObservation, '/unloading/perception', 10)
        self.contexts = self.node.create_publisher(ExecutionContext, '/unloading/execution_context', 10)
        self.grants = self.node.create_publisher(ExecutionGrant, '/unloading/execution_grant', 10)
        self.commands = self.node.create_publisher(ExecutionAuthorization, '/unloading/execution_authorization', 10)
        self.robot_mode = self.mechanism_mode = 'live'
        self.position, self.tool = 0., 'synthetic-tool'
        self.mechanism_sequence = self.perception_sequence = 0
        self.last_robot = self.last_mechanism = None
        self.refresh_perception = False
        self.last_capture = None
        self.sample_cycle = 0
        self.timer = self.node.create_timer(.02, self.tick)

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def wait(self, predicate, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)
            if predicate(): return
        raise AssertionError('DDS heartbeat condition not reached')

    def discover(self):
        pubs = (self.joints, self.mechanisms, self.perceptions, self.contexts, self.grants, self.commands)
        self.wait(lambda: all(p.get_subscription_count() for p in pubs)
                  and self.world.publisher.get_subscription_count() >= 2
                  and self.execution.events.get_subscription_count() > 0
                  and self.execution.client.server_is_ready())

    def observe(self):
        self.last_capture = self.now()
        observation = replace(_synthetic_observation(), source_epoch='synthetic-online-heartbeat',
            source_sequence=self.perception_sequence, capture_time=self.last_capture,
            processed_time=self.last_capture, clock_domain='ros')
        self.perception_sequence += 1
        self.perceptions.publish(observation_to_msg(observation))

    def tick(self):
        self.sample_cycle += 1
        stamp = self.node.get_clock().now().to_msg()
        if self.robot_mode == 'live':
            self.last_robot = JointState(name=[f'joint_{i}' for i in range(1, 7)],
                position=[self.position]*6, velocity=[0.]*6, effort=[0.]*6)
            self.last_robot.header.stamp = stamp
        if self.robot_mode != 'off' and self.last_robot is not None:
            message = deepcopy(self.last_robot)
            if self.robot_mode == 'repeat' and self.sample_cycle % 2:
                message.header.stamp = float_to_time(time_to_float(message.header.stamp) - .1)
            self.joints.publish(message)
        if self.mechanism_mode == 'live':
            self.last_mechanism = MechanismState(schema_version='1.1.0', source_epoch='synthetic-mechanism',
                sequence=self.mechanism_sequence, source_restart=self.mechanism_sequence == 0,
                observed_time=stamp, clock_domain='ros', tool_state_identity=self.tool,
                payload_state_identity='synthetic-empty', base_state_identity='synthetic-base',
                conveyor_state_identity='synthetic-stopped', config_identity='synthetic-config',
                robot_model_fingerprint='synthetic-robot', world_model_fingerprint='synthetic-world',
                tool_state_json='{"verified":"synthetic-test"}', payload_state_json='{"object_id":null}',
                base_state_json='{"position_m":[0,0,0]}', conveyor_state_json='{"running":false}')
            self.mechanism_sequence += 1
        if self.mechanism_mode != 'off' and self.last_mechanism is not None:
            message = deepcopy(self.last_mechanism)
            if self.mechanism_mode == 'repeat' and self.sample_cycle % 2:
                message.observed_time = float_to_time(time_to_float(message.observed_time) - .1)
                message.sequence += 100  # newer transport sequence cannot repair an old sample
                message.source_restart = False
            self.mechanisms.publish(message)
        self.contexts.publish(ExecutionContext(schema_version='1.2.0', publisher_epoch='synthetic-context',
            publisher_sequence=self.sample_cycle, publisher_restart=False, session_id='synthetic-session',
            epoch='synthetic-execution', planning_generation=1, allowed_plan_id='synthetic-plan',
            observed_time=stamp, clock_domain='ros'))
        if self.refresh_perception and (self.last_capture is None or self.now() - self.last_capture >= .3):
            self.observe()

    def ready(self):
        self.discover()
        self.observe()  # exactly one perception for the pure-heartbeat window
        self.wait(lambda: self.worlds and self.worlds[-1].planning_admissible and
                  self.execution.current_world is not None and self.execution.current_context is not None)

    def command(self, identity, *, grant=True, expiry=None):
        world = self.execution.current_world
        trajectory = TimedJointTrajectory(tuple(f'joint_{i}' for i in range(1, 7)),
            (TimedJointPoint(world.current_q, 0., (0.,)*6), TimedJointPoint(world.current_q, .02, (0.,)*6)),
            PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, 'synthetic-validator', 'synthetic-robot', 'synthetic-config')
        command = ExecutionAuthorization(schema_version='1.1.0', command_id=identity,
            plan_id='synthetic-plan', request_id=identity, session_id='synthetic-session', epoch='synthetic-execution',
            planning_generation=1, world_fingerprint=world.fingerprint, robot_model_fingerprint='synthetic-robot',
            config_identity='synthetic-config', validation_reference='synthetic-validator', validation_generation=1)
        command.trajectory.joint_names = list(trajectory.joint_names)
        command.trajectory.points = [JointTrajectoryPoint(positions=list(p.positions), velocities=list(p.velocities),
            time_from_start=Duration(sec=0, nanosec=round(p.time_from_start*1e9))) for p in trajectory.points]
        if grant:
            self.grants.publish(ExecutionGrant(schema_version='1.1.0', grant_id=identity, command_id=identity,
                plan_id=command.plan_id, request_id=identity, session_id=command.session_id, epoch=command.epoch,
                planning_generation=1, world_fingerprint=world.fingerprint, robot_model_fingerprint='synthetic-robot',
                config_identity='synthetic-config', validation_reference='synthetic-validator', validation_generation=1,
                trajectory_fingerprint=canonical_fingerprint(trajectory), expires_at=float_to_time(self.now()+10 if expiry is None else expiry),
                clock_domain='ros', mock_only=True))
            self.wait(lambda: identity in self.execution.gate._grants)
        return command

    def send(self, command, expected):
        self.commands.publish(command)
        self.wait(lambda: any(e.command_id == command.command_id and e.kind == expected for e in self.events))
        return next(e for e in reversed(self.events) if e.command_id == command.command_id and e.kind == expected)

    def close(self):
        self.executor.shutdown()
        for node in (self.world, self.execution, self.controller, self.node):
            if node is not None: node.destroy_node()


@pytest.fixture
def graph():
    rclpy.init(args=['--ros-args', '-p', 'controller_epoch:=synthetic-controller'], domain_id=160 + os.getpid() % 40)
    graph = SyntheticInputs()
    try:
        yield graph
    finally:
        graph.close()
        rclpy.shutdown()


def test_static_samples_reach_execution_without_changing_grant(graph):
    graph.ready()
    initial = graph.worlds[-1]
    command = graph.command('static-heartbeat')
    grant = graph.execution.gate._grants[command.command_id]
    start = graph.now()
    graph.wait(lambda: graph.now() - start >= 1.2)
    assert graph.perception_sequence == 1
    assert graph.now() - graph.world.last_joint_stamp < .1
    assert graph.now() - graph.execution.current_robot_sample_time < .5, (
        graph.world.last_joint_stamp, graph.execution.current_robot_sample_time,
        graph.execution._authoritative_state_error())
    assert graph.execution._authoritative_state_error() is None
    assert graph.execution.gate._grants[command.command_id] == grant
    assert all(w.world_fingerprint == initial.world_fingerprint for w in graph.worlds)
    versions = lambda w: (w.scene_revision_sequence, w.scene_fingerprint, w.robot_state_revision_sequence,
                         w.robot_state_fingerprint, w.mechanism_revision_sequence, w.mechanism_fingerprint)
    assert all(versions(w) == versions(initial) for w in graph.worlds)
    assert len(graph.worlds) >= 8  # controlled 10 Hz propagation over the >1 s window
    assert len(graph.worlds) < 35  # not one full world per 50 Hz source callback
    assert len({w.publisher_epoch for w in graph.worlds}) == 1
    assert all(b.publisher_sequence > a.publisher_sequence for a, b in zip(graph.worlds, graph.worlds[1:]))
    assert all(not w.publisher_restart for w in graph.worlds if w.publisher_sequence > 0)
    assert time_to_float(graph.worlds[-1].robot_sample_time) > time_to_float(initial.robot_sample_time) + 1.
    assert time_to_float(graph.worlds[-1].mechanism_sample_time) > time_to_float(initial.mechanism_sample_time) + 1.
    for world in graph.worlds:
        domain = snapshot_from_msg(world)
        assert float_to_time(domain.robot_state_revision.sample_time) == world.robot_sample_time
        assert domain.robot_state_revision.source == 'joint_states'
        assert domain.tool_attachment['source_epoch'] == 'synthetic-mechanism'
        assert float_to_time(domain.scene_snapshot['perception_source']['capture_time']) == world.source_capture_time
        assert tuple(world.blocking_reasons) == domain.scene_snapshot['blocking_reasons']
        assert world.source_capture_time == initial.source_capture_time
        assert time_to_float(world.published_time) >= time_to_float(world.robot_sample_time)
        assert time_to_float(world.published_time) >= time_to_float(world.mechanism_sample_time)
    graph.send(command, 'STARTED')
    graph.wait(lambda: any(e.command_id == command.command_id and e.kind == 'SUCCEEDED' for e in graph.events))


@pytest.mark.parametrize('source,mode', [('robot', 'off'), ('mechanism', 'off'),
                                        ('robot', 'repeat'), ('mechanism', 'repeat')])
def test_source_loss_old_samples_and_recovery_do_not_revive_grants(graph, source, mode):
    graph.ready()
    initial = graph.worlds[-1]
    # A real new synthetic observation is permitted here to isolate a >2 s
    # mechanism timeout from perception expiry. Not used in the red/normal case.
    if source == 'mechanism': graph.refresh_perception = True
    command = graph.command('before-source-loss')
    setattr(graph, source + '_mode', mode)
    field = 'robot_sample_time' if source == 'robot' else 'mechanism_sample_time'
    frozen = time_to_float(graph.last_robot.header.stamp if source == 'robot' else graph.last_mechanism.observed_time)
    reason = source.upper() + '_STATE_STALE_OR_TIME_JUMP'
    graph.wait(lambda: reason in graph.worlds[-1].blocking_reasons and
               reason in graph.execution.current_world.scene_snapshot['blocking_reasons'])
    stale = graph.worlds[-1]
    assert not stale.planning_admissible and stale.world_fingerprint != initial.world_fingerprint
    assert command.command_id not in graph.execution.gate._grants
    count = len(graph.worlds)
    graph.wait(lambda: len(graph.worlds) >= count + 3)
    assert all(time_to_float(getattr(w, field)) == frozen for w in graph.worlds[count:])
    rejection = graph.send(graph.command('stale-source', grant=False), 'REJECTED')
    assert rejection.message == reason
    graph.refresh_perception = False
    graph.wait(lambda: graph.world.last_observation.source_sequence == graph.perception_sequence - 1
               and graph.worlds[-1].source_capture_time == float_to_time(graph.last_capture))
    # Drain the latest source observation before comparing recovery content.
    before = graph.worlds[-1]
    setattr(graph, source + '_mode', 'live')
    graph.wait(lambda: reason not in graph.worlds[-1].blocking_reasons and
               graph.execution._authoritative_state_error() is None)
    recovered = graph.worlds[-1]
    assert recovered.robot_state_revision_sequence == initial.robot_state_revision_sequence
    assert recovered.mechanism_revision_sequence == initial.mechanism_revision_sequence
    assert recovered.robot_state_fingerprint == initial.robot_state_fingerprint
    assert recovered.mechanism_fingerprint == initial.mechanism_fingerprint
    assert recovered.source_capture_time == before.source_capture_time
    assert time_to_float(getattr(recovered, field)) > frozen
    assert command.command_id not in graph.execution.gate._grants
    # Bind the current world and matching command identity, without any new grant.
    command.world_fingerprint = graph.execution.current_world.fingerprint
    assert graph.send(command, 'REJECTED').message == 'AUTHORIZATION_GRANT_MISSING'


def test_perception_expiry_latch_and_partial_recovery(graph):
    graph.ready()
    command = graph.command('before-perception-expiry')
    initial = graph.worlds[-1]
    graph.wait(lambda: 'OBSERVATION_STALE' in graph.worlds[-1].blocking_reasons and
               'OBSERVATION_STALE' in graph.execution.current_world.scene_snapshot['blocking_reasons'])
    assert command.command_id not in graph.execution.gate._grants
    assert graph.worlds[-1].world_fingerprint != initial.world_fingerprint
    assert graph.worlds[-1].source_capture_time == initial.source_capture_time
    assert graph.execution._authoritative_state_error() is None  # state sources/context/world still live
    assert graph.send(graph.command('perception-stale', grant=False), 'REJECTED').message == 'WORLD_NOT_PLANNING_ADMISSIBLE'
    graph.robot_mode = 'off'
    graph.wait(lambda: 'ROBOT_STATE_STALE_OR_TIME_JUMP' in graph.worlds[-1].blocking_reasons)
    graph.robot_mode = 'live'
    graph.wait(lambda: 'ROBOT_STATE_STALE_OR_TIME_JUMP' not in graph.worlds[-1].blocking_reasons)
    assert 'OBSERVATION_STALE' in graph.worlds[-1].blocking_reasons
    assert graph.worlds[-1].source_capture_time == initial.source_capture_time
    graph.observe()
    graph.wait(lambda: graph.worlds[-1].planning_admissible and graph.execution.current_world.scene_snapshot['planning_admissible'])
    assert graph.worlds[-1].source_capture_time != initial.source_capture_time
    assert command.command_id not in graph.execution.gate._grants


@pytest.mark.parametrize('source', ['robot', 'mechanism'])
def test_actual_content_change_revises_world_and_revokes_grant(graph, source):
    graph.ready()
    initial = graph.worlds[-1]
    command = graph.command('before-content-change')
    if source == 'robot': graph.position = .02
    else: graph.tool = 'synthetic-new-tool'
    graph.wait(lambda: graph.execution.current_world.fingerprint != initial.world_fingerprint
               and graph.worlds[-1].world_fingerprint != initial.world_fingerprint)
    changed = graph.worlds[-1]
    assert changed.scene_fingerprint == initial.scene_fingerprint
    assert changed.source_capture_time == initial.source_capture_time
    if source == 'robot':
        assert changed.robot_state_revision_sequence > initial.robot_state_revision_sequence
        assert changed.robot_state_fingerprint != initial.robot_state_fingerprint
        assert changed.mechanism_revision_sequence == initial.mechanism_revision_sequence
    else:
        assert changed.mechanism_revision_sequence > initial.mechanism_revision_sequence
        assert changed.mechanism_fingerprint != initial.mechanism_fingerprint
        assert changed.robot_state_revision_sequence == initial.robot_state_revision_sequence
    assert command.command_id not in graph.execution.gate._grants
    assert graph.send(command, 'REJECTED').message == 'WORLD_CHANGED_BEFORE_SEND'


def test_world_publisher_stopping_still_rejects_new_command(graph):
    graph.ready()
    command = graph.command('publisher-stopped')
    # Actually stop the publisher node; test sources and context keep sampling.
    graph.executor.remove_node(graph.world)
    graph.world.destroy_node()
    graph.world = None
    graph.wait(lambda: graph.execution._authoritative_state_error() == 'WORLD_PUBLISHER_STALE_OR_TIME_JUMP')
    assert graph.now() - time_to_float(graph.last_robot.header.stamp) < .1
    assert graph.now() - time_to_float(graph.last_mechanism.observed_time) < .1
    assert graph.now() - graph.execution.current_context_received_at < .1
    assert graph.send(command, 'REJECTED').message == 'WORLD_PUBLISHER_STALE_OR_TIME_JUMP'


def test_heartbeat_never_extends_grant_expiry(graph):
    graph.ready()
    expiry = graph.now() + .25
    command = graph.command('expired-grant', expiry=expiry)
    graph.wait(lambda: graph.now() > expiry + .1)
    assert graph.execution._authoritative_state_error() is None
    assert float_to_time(graph.execution.gate._grants[command.command_id].expires_at) == float_to_time(expiry)
    assert graph.send(command, 'REJECTED').message == 'AUTHORIZATION_GRANT_EXPIRED_OR_CLOCK_INVALID'


def test_missing_states_never_produce_fabricated_heartbeat(graph):
    graph.robot_mode = graph.mechanism_mode = 'off'
    graph.discover()
    graph.observe()
    graph.wait(lambda: graph.world.last_observation is not None)
    start = graph.now()
    graph.wait(lambda: graph.now() - start > .3)
    assert not graph.worlds and graph.execution.current_world is None
    assert graph.world.assembler.robot_state is None
    graph.robot_mode = 'live'
    graph.wait(lambda: graph.world.assembler.robot_state is not None)
    start = graph.now()
    graph.wait(lambda: graph.now() - start > .3)
    assert not graph.worlds and graph.world.assembler.tool_attachment is None
    graph.mechanism_mode = 'live'
    graph.wait(lambda: graph.worlds and graph.worlds[-1].planning_admissible)


def test_paused_sim_time_heartbeats_do_not_age_or_refresh_sources(transport):
    bridge, executor, worlds, perceptions, clock, states, wait = transport
    clock(100.)
    states(100., 0)
    perceptions.publish(observation_to_msg(sample(100., 1, 'synthetic-metric-demo')))
    wait(lambda: worlds and worlds[-1].planning_admissible)
    initial = worlds[-1]
    count = len(worlds)
    wait(lambda: len(worlds) >= count + 4)
    for message in worlds[count:]:
        assert message.planning_admissible and not message.blocking_reasons
        assert message.world_fingerprint == initial.world_fingerprint
        assert message.robot_sample_time == message.mechanism_sample_time == message.source_capture_time == float_to_time(100.)
        assert message.published_time == float_to_time(100.)
        assert message.publisher_sequence > initial.publisher_sequence
    assert bridge.last_observation.source_sequence == 1
