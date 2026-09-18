"""Synthetic state messages, real WorldBridgeNode/DDS and controlled /clock."""
from copy import deepcopy
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from unloading_interfaces.msg import MechanismState
from unloading_ros_bridge.common import float_to_time
from unloading_ros_bridge.mapping import observation_to_msg, snapshot_from_msg
from test_perception_time_transport import transport, time_transport, sample
from test_world_heartbeat import graph


def publishers(executor):
    node = next(n for n in executor.get_nodes() if n.get_name() == 'synthetic_time_admission_inputs')
    return (node.create_publisher(JointState, '/joint_states', 10),
            node.create_publisher(MechanismState, '/unloading/mechanism_state', 10))


def joint(t, position=0.):
    message = JointState(name=[f'joint_{i}' for i in range(1, 7)], position=[position]*6, velocity=[.1]*6)
    message.header.stamp = float_to_time(t)
    return message


def mechanism(t, sequence, *, epoch='synthetic-mechanism', restart=False):
    return MechanismState(schema_version='1.1.0', source_epoch=epoch, sequence=sequence,
        source_restart=restart, observed_time=float_to_time(t), clock_domain='ros_sim_time',
        tool_state_identity='synthetic-tool', payload_state_identity='synthetic-empty',
        base_state_identity='synthetic-base', conveyor_state_identity='synthetic-stopped',
        config_identity='synthetic-config', robot_model_fingerprint='synthetic-robot',
        world_model_fingerprint='synthetic-world', tool_state_json='{"verified":true}',
        payload_state_json='{"object_id":null}', base_state_json='{"position_m":[0,0,0]}',
        conveyor_state_json='{"running":false}')


def accepted_state(bridge):
    return (bridge.last_joint_stamp, bridge.mechanism_stamp,
        bridge.last_robot_content, bridge.last_mechanism_content,
        bridge.robot_sequence, bridge.mechanism_sequence,
        bridge.assembler.robot_state, bridge.assembler.tool_attachment,
        bridge.assembler.payload_attachment, bridge.assembler.base_state,
        bridge.assembler.conveyor_state, bridge.assembler.config_identity,
        bridge.mechanism_guard.current_epoch, bridge.mechanism_guard.sequence,
        tuple(bridge.mechanism_guard._retired))


@pytest.mark.parametrize('source,new_epoch', [('robot', False), ('mechanism', False), ('mechanism', True)])
def test_future_state_does_not_poison_and_next_valid_sample_recovers(transport, source, new_epoch):
    bridge, executor, worlds, observations, clock, states, wait = transport
    joints, mechanisms = publishers(executor)
    wait(lambda: joints.get_subscription_count() and mechanisms.get_subscription_count())
    clock(100.)
    states(100., 10)
    observations.publish(observation_to_msg(sample(100., 1, 'synthetic-metric-demo')))
    wait(lambda: worlds and worlds[-1].planning_admissible)
    before = accepted_state(bridge)
    initial = worlds[-1]
    reason = source.upper() + '_STATE_TIME_IN_FUTURE'
    if source == 'robot':
        joints.publish(joint(1000., 9.))
    else:
        bad = mechanism(1000., 0 if new_epoch else 999999,
                        epoch='invalid-B' if new_epoch else 'synthetic-mechanism', restart=new_epoch)
        bad.tool_state_identity = 'bad-tool'
        mechanisms.publish(bad)
    # Old production code commits 1000; fixed code records a separate rejection.
    wait(lambda: bridge.last_joint_stamp == 1000. or bridge.mechanism_stamp == 1000.
         or source in getattr(bridge, 'last_state_time_rejection', {}))
    assert accepted_state(bridge) == before, (
        bridge.last_joint_stamp, bridge.mechanism_stamp,
        bridge.mechanism_guard.current_epoch, bridge.mechanism_guard.sequence,
        tuple(bridge.mechanism_guard._retired))
    wait(lambda: reason in worlds[-1].blocking_reasons)
    blocked = worlds[-1]
    domain = snapshot_from_msg(blocked)
    assert not blocked.planning_admissible and not domain.scene_snapshot['planning_admissible']
    assert reason in domain.scene_snapshot['blocking_reasons']
    assert blocked.robot_sample_time == initial.robot_sample_time
    assert blocked.mechanism_sample_time == initial.mechanism_sample_time
    assert blocked.source_capture_time == initial.source_capture_time
    assert blocked.robot_state_fingerprint == initial.robot_state_fingerprint
    assert blocked.mechanism_fingerprint == initial.mechanism_fingerprint
    assert blocked.world_fingerprint != initial.world_fingerprint
    assert list(blocked.obstacles) == list(initial.obstacles)
    clock(100.1)
    states(100.1, 11)
    wait(lambda: worlds[-1].planning_admissible)
    assert bridge.mechanism_guard.current_epoch == 'synthetic-mechanism'
    assert bridge.mechanism_guard.sequence == 11 and not bridge.mechanism_guard._retired
    assert reason not in worlds[-1].blocking_reasons


@pytest.mark.parametrize('source', ['robot', 'mechanism'])
def test_first_bad_time_and_encoding_cannot_create_state(transport, source):
    bridge, executor, worlds, observations, clock, states, wait = transport
    joints, mechanisms = publishers(executor)
    publisher = joints if source == 'robot' else mechanisms
    wait(lambda: publisher.get_subscription_count())
    def rejected(message, suffix):
        previous = bridge.last_state_time_rejection.get(source)
        publisher.publish(message)
        wait(lambda: bridge.last_state_time_rejection.get(source) is not previous)
        assert bridge.state_time_blocking_reasons[source] == (source.upper() + '_STATE_' + suffix,)
        assert bridge.last_joint_stamp is None and bridge.mechanism_stamp is None
        assert bridge.robot_sequence == bridge.mechanism_sequence == 0
        assert bridge.assembler.robot_state is None and bridge.assembler.tool_attachment is None
        assert bridge.mechanism_guard.current_epoch is None and bridge.mechanism_guard.sequence == -1
        assert not worlds
    make = (lambda t: joint(t)) if source == 'robot' else (lambda t: mechanism(t, 999999))
    rejected(make(100.), 'CLOCK_NOT_INITIALIZED')
    clock(100.)
    for stamp, reason in [(Time(sec=1000), 'TIME_IN_FUTURE'), (Time(), 'TIME_INVALID'),
                          (Time(sec=-1), 'TIME_INVALID'), (Time(sec=100, nanosec=1000000000), 'TIME_INVALID'),
                          (Time(sec=97), 'STALE')]:
        message = make(100.)
        if source == 'robot': message.header.stamp = stamp
        else: message.observed_time = stamp
        rejected(message, reason)
    if source == 'mechanism':
        message = make(100.)
        message.clock_domain = 'ros'
        rejected(message, 'CLOCK_DOMAIN_MISMATCH')
    states(100., 10)
    observations.publish(observation_to_msg(sample(100., 1, 'synthetic-metric-demo')))
    wait(lambda: worlds and worlds[-1].planning_admissible)
    assert not bridge.state_time_blocking_reasons[source]


def test_faults_are_source_local_and_only_new_admitted_samples_clear_them(transport):
    bridge, executor, worlds, observations, clock, states, wait = transport
    joints, mechanisms = publishers(executor)
    wait(lambda: joints.get_subscription_count() and mechanisms.get_subscription_count())
    clock(100.)
    states(100., 10)
    observations.publish(observation_to_msg(sample(100., 1, 'synthetic-metric-demo')))
    wait(lambda: worlds and worlds[-1].planning_admissible)
    initial = accepted_state(bridge)
    joints.publish(joint(100.25, 9.))
    mechanisms.publish(mechanism(100.25, 999999))
    wait(lambda: all(bridge.state_time_blocking_reasons.values()))
    # Even when time catches up, duplicates, reordered stamps and malformed
    # content cannot clear either fault or update admitted samples/guards.
    clock(100.25)
    for t in (100., 99.875):
        joints.publish(joint(t, 9.))
        mechanisms.publish(mechanism(t, 999998))
        count = len(worlds)
        wait(lambda: len(worlds) >= count + 2)
        assert accepted_state(bridge) == initial
        assert all(bridge.state_time_blocking_reasons.values())
    mechanisms.publish(mechanism(100.25, 10))  # fresh time cannot repair a duplicate source sequence
    count = len(worlds)
    wait(lambda: len(worlds) >= count + 2)
    assert accepted_state(bridge) == initial and bridge.state_time_blocking_reasons['mechanism']
    # A domain construction failure (empty q under an explicitly configured
    # empty names list) must not consume timestamp/version or clear a fault.
    assert bridge.set_parameters([Parameter('expected_joint_names', Parameter.Type.STRING_ARRAY, [])])[0].successful
    empty = JointState()
    empty.header.stamp = float_to_time(100.25)
    joints.publish(empty)
    count = len(worlds)
    wait(lambda: len(worlds) >= count + 2)
    assert accepted_state(bridge) == initial and bridge.state_time_blocking_reasons['robot']
    assert bridge.set_parameters([Parameter('expected_joint_names', value=[f'joint_{i}' for i in range(1, 7)])])[0].successful
    # Nested non-finite JSON is rejected before source B can retire A.
    bad = mechanism(100.25, 0, epoch='invalid-B', restart=True)
    bad.tool_state_json = '{"verified":true,"bad":NaN}'
    mechanisms.publish(bad)
    count = len(worlds)
    wait(lambda: len(worlds) >= count + 2)
    assert accepted_state(bridge) == initial
    observations.publish(observation_to_msg(sample(101., 999999, 'synthetic-metric-demo')))
    wait(lambda: bool(bridge.time_admission_blocking_reasons))
    joints.publish(joint(100.25, .1))
    wait(lambda: bridge.last_joint_stamp == 100.25)
    assert not bridge.state_time_blocking_reasons['robot']
    assert bridge.state_time_blocking_reasons['mechanism'] and bridge.time_admission_blocking_reasons
    assert bridge.robot_sequence == initial[4]  # same content as original states(100, 10)
    mechanisms.publish(mechanism(100.25, 11))
    wait(lambda: bridge.mechanism_guard.sequence == 11)
    assert not bridge.state_time_blocking_reasons['mechanism']
    assert bridge.mechanism_sequence == initial[5]
    assert bridge.time_admission_blocking_reasons
    observations.publish(observation_to_msg(sample(100.25, 2, 'synthetic-metric-demo')))
    wait(lambda: worlds[-1].planning_admissible)
    # Real, valid source restart still takes ownership using existing rules.
    mechanisms.publish(mechanism(100.25, 0, epoch='valid-B', restart=True))
    wait(lambda: bridge.mechanism_guard.current_epoch == 'valid-B')
    assert tuple(bridge.mechanism_guard._retired) == ('synthetic-mechanism',)


@pytest.mark.parametrize('source,age', [('robot', .5), ('mechanism', 2.)])
def test_inclusive_source_age_boundary_matches_watchdog(transport, source, age):
    bridge, executor, worlds, observations, clock, states, wait = transport
    joints, mechanisms = publishers(executor)
    wait(lambda: joints.get_subscription_count() and mechanisms.get_subscription_count())
    clock(100.)
    joints.publish(joint(100. - age if source == 'robot' else 100.))
    mechanisms.publish(mechanism(100. - age if source == 'mechanism' else 100., 10))
    wait(lambda: bridge.last_joint_stamp is not None and bridge.mechanism_stamp is not None)
    observations.publish(observation_to_msg(sample(100., 1, 'synthetic-metric-demo')))
    wait(lambda: worlds and worlds[-1].planning_admissible)
    count = len(worlds)
    wait(lambda: len(worlds) >= count + 2)
    assert all(w.planning_admissible for w in worlds[count:])
    clock(100.001)
    wait(lambda: source.upper() + '_STATE_STALE_OR_TIME_JUMP' in worlds[-1].blocking_reasons)


@pytest.mark.parametrize('source', ['robot', 'mechanism'])
def test_system_clock_fault_revokes_grant_and_recovery_cannot_revive_it(graph, source):
    # Existing system-ROS-time mock path; this is not sim-time execution support.
    graph.ready()
    command = graph.command('before-bad-time')
    setattr(graph, source + '_mode', 'off')
    if source == 'robot':
        bad = deepcopy(graph.last_robot)
        bad.header.stamp = float_to_time(graph.now() + 100.)
        graph.joints.publish(bad)
    else:
        bad = deepcopy(graph.last_mechanism)
        bad.observed_time = float_to_time(graph.now() + 100.)
        bad.sequence += 999999
        graph.mechanisms.publish(bad)
    reason = source.upper() + '_STATE_TIME_IN_FUTURE'
    graph.wait(lambda: reason in graph.execution.current_world.scene_snapshot['blocking_reasons'])
    assert command.command_id not in graph.execution.gate._grants
    assert graph.send(command, 'REJECTED').message == 'WORLD_CHANGED_BEFORE_SEND'
    setattr(graph, source + '_mode', 'live')
    graph.wait(lambda: graph.execution.current_world.scene_snapshot['planning_admissible'])
    assert command.command_id not in graph.execution.gate._grants
    command.world_fingerprint = graph.execution.current_world.fingerprint
    graph.events.clear()  # await the second command's event, not the earlier rejection
    assert graph.send(command, 'REJECTED').message == 'AUTHORIZATION_GRANT_MISSING'


def session_probe(kind, domain_id, output):
    """Subprocess entry: all old nodes/context exit before the next process starts."""
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    import rclpy

    with time_transport(domain_id=domain_id) as transport_graph:
        bridge, executor, worlds, observations, clock, states, wait = transport_graph
        node = next(n for n in executor.get_nodes() if n.get_name() == 'synthetic_time_admission_inputs')
        joints, mechanisms = publishers(executor)
        wait(lambda: joints.get_subscription_count() and mechanisms.get_subscription_count())
        t = 100. if kind == 'old' else 1.
        clock(t)
        broadcaster = StaticTransformBroadcaster(node)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'world', 'synthetic-session-camera'
        tf.header.stamp = float_to_time(t)
        tf.transform.rotation.w = 1.
        broadcaster.sendTransform(tf)
        wait(lambda: bridge.buffer.can_transform('world', 'synthetic-session-camera', rclpy.time.Time()))
        observed = sample(t, 1, 'synthetic-metric-demo')
        observed = replace(observed, source_epoch='synthetic-perception-' + kind,
            cargo=tuple(replace(c, pose=replace(c.pose, frame_id='synthetic-session-camera'))
                        if c.pose is not None else c for c in observed.cargo))
        observations.publish(observation_to_msg(observed))
        wait(lambda: bridge.last_observation is not None)
        assert not worlds and bridge.last_snapshot is None
        joints.publish(joint(t))
        wait(lambda: bridge.last_joint_stamp == t)
        # No mechanism is fabricated while the steady watchdog continues.
        start = bridge.watchdog_clock.now().nanoseconds
        wait(lambda: bridge.watchdog_clock.now().nanoseconds - start > 200_000_000)
        assert not worlds and bridge.assembler.tool_attachment is None
        mechanisms.publish(mechanism(t, 10 if kind == 'old' else 0,
            epoch='synthetic-mechanism-' + kind, restart=kind == 'new'))
        wait(lambda: worlds and worlds[-1].planning_admissible)
        first = worlds[-1]
        if kind == 'old':
            saved = accepted_state(bridge)
            clock(0.)
            wait(lambda: 'CLOCK_NOT_INITIALIZED' in worlds[-1].blocking_reasons)
            clock(1.)
            joints.publish(joint(1., 9.))
            mechanisms.publish(mechanism(1., 11, epoch='synthetic-mechanism-old'))
            count = len(worlds)
            wait(lambda: len(worlds) >= count + 3)
            assert accepted_state(bridge) == saved
            assert not worlds[-1].planning_admissible
            assert worlds[-1].source_capture_time == float_to_time(100.)
        result = dict(domain_id=domain_id, publisher_epoch=first.publisher_epoch,
            perception_epoch=first.source_epoch, mechanism_epoch=bridge.mechanism_guard.current_epoch,
            robot_sample_time=bridge.last_joint_stamp, mechanism_sample_time=bridge.mechanism_stamp,
            capture_time=bridge.last_observation.capture_time,
            planning_admissible=worlds[-1].planning_admissible)
    # time_transport has shut down its executor, every node and the DDS context.
    Path(output).write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')


def test_reset_requires_complete_exit_and_new_isolated_session(tmp_path):
    results = []
    base_domain = 30 + os.getpid() % 30
    for index, kind in enumerate(('old', 'new')):
        output = tmp_path / (kind + '.json')
        process = subprocess.run([sys.executable, str(Path(__file__).resolve()), kind,
            str(base_domain + index), str(output)], capture_output=True, text=True,
            encoding='utf-8', timeout=20)
        assert process.returncode == 0, process.stdout + process.stderr
        results.append(json.loads(output.read_text(encoding='utf-8')))
    old, new = results
    assert old['domain_id'] != new['domain_id'] and old['publisher_epoch'] != new['publisher_epoch']
    assert old['perception_epoch'] != new['perception_epoch'] and old['mechanism_epoch'] != new['mechanism_epoch']
    assert old['robot_sample_time'] == old['mechanism_sample_time'] == old['capture_time'] == 100.
    assert not old['planning_admissible']
    assert new['robot_sample_time'] == new['mechanism_sample_time'] == new['capture_time'] == 1.
    assert new['planning_admissible']


if __name__ == '__main__':
    session_probe(sys.argv[1], int(sys.argv[2]), sys.argv[3])
