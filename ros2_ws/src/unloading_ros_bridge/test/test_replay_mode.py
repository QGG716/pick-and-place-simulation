"""Mode and source-state boundaries using production callbacks and disk backend."""
from dataclasses import replace
from pathlib import Path

import pytest
import rclpy
from unloading_perception.backends import CargoJsonReplayBackend
from unloading_perception.demo import _synthetic_observation
from unloading_perception.replay import replay_publication
from unloading_ros_bridge.mapping import observation_to_msg
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode

FIXTURE = Path(__file__).resolve().parents[4] / 'tests/fixtures/vision_upstream/cargo7_minimal.json'


def publication(sequence=0):
    record = CargoJsonReplayBackend().read(FIXTURE, replay_epoch='synthetic-replay-session')
    return replay_publication(record, sequence=sequence, elapsed=sequence + .1,
                              published_time=100. + sequence, clock_domain='ros')


def test_default_online_rejects_replay_even_with_provider_or_synthetic_claims():
    rclpy.init()
    node = WorldBridgeNode()
    try:
        assert node.observation_mode == 'online'
        assert node.describe_parameters(['observation_mode'])[0].read_only
        now = node.get_clock().now().nanoseconds / 1e9
        valid = replace(_synthetic_observation(), capture_time=now, processed_time=now, clock_domain='ros')
        node.on_observation(observation_to_msg(valid))
        saved, sequence = node.last_observation, node.tracker.sequence
        for provider in ('cargo-json-replay', 'registered-rgbd-fused-algorithm', 'synthetic-metric-demo'):
            replay = replace(publication(), provider=provider, synthetic=True)
            node.on_observation(observation_to_msg(replay))
            assert 'OBSERVATION_CLOCK_DOMAIN_MISMATCH' in node.last_time_rejection['blocking_reasons']
            assert node.last_observation is saved and node.tracker.sequence == sequence
        assert node.replay_guard.sequence == -1
        rewritten = replace(publication(), clock_domain='ros', capture_time=now, processed_time=now)
        node.on_observation(observation_to_msg(rewritten))
        assert 'OBSERVATION_REPLAY_TIME_NOT_ONLINE' in node.last_time_rejection['blocking_reasons']
        assert node.last_observation is saved and node.tracker.sequence == sequence
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_replay_mode_rejects_inconsistent_records_before_source_guard():
    rclpy.init(args=['--ros-args', '-p', 'observation_mode:=replay_display_only'])
    node = WorldBridgeNode()
    try:
        first = publication()
        node.on_observation(observation_to_msg(first))
        saved = node.last_observation
        assert saved is not None and node.replay_guard.sequence == 0
        assert node.last_snapshot is None and node.assembler.robot_state is None
        later = publication(1)
        meta = dict(later.coverage['replay'])
        changed = {**meta, 'record_id': 'different-record-in-same-session'}
        missing = dict(meta)
        del missing['source_uri']
        invalid = [replace(later, coverage={}), replace(later, clock_domain='ros'),
            replace(later, coverage={**later.coverage, 'replay': changed}),
            replace(later, coverage={**later.coverage, 'replay': missing}), first]
        for record in invalid:
            node.on_observation(observation_to_msg(record))
            assert node.last_observation is saved and node.replay_guard.sequence == 0
        node.on_observation(observation_to_msg(later))
        assert node.replay_guard.sequence == 1
        assert node.tracker.sequence == -1 and node.algorithm_observation_guard.sequence == -1
        assert node.last_observation.capture_time == 0.
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_mode_configuration_fails_explicitly():
    rclpy.init(args=['--ros-args', '-p', 'observation_mode:=implicit_replay'])
    try:
        with pytest.raises(ValueError, match='observation_mode'):
            WorldBridgeNode()
    finally:
        rclpy.shutdown()
