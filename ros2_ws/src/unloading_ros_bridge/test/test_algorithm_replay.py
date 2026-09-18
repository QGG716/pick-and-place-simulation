"""Synthetic domain artifacts through the real read-only launch and DDS."""
from dataclasses import replace
import os
from pathlib import Path
import subprocess
import sys

import rclpy
from unloading_perception.algorithm_artifact import algorithm_replay_record
from unloading_perception.replay import replay_publication
from unloading_ros_bridge.mapping import observation_to_msg
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'tools')]
from test_algorithm_artifact import artifact_fixture
from probe_algorithm_replay import probe


def test_algorithm_artifact_actual_dds_and_markers(tmp_path):
    ref = artifact_fixture(tmp_path/'artifacts')
    report = probe(Path(ref['path']), ref['sha256'], tmp_path/'dds', 180+os.getpid()%30)
    assert report['status'] == 'ACTUAL_DDS_READ_ONLY_HANDOFF_PASS'
    assert report['raw_surface_count'] > 0 and report['marker_representative_count'] > 0
    assert report['execution_authorizations'] == 0


def test_algorithm_file_node_rejects_hash_before_publication(tmp_path):
    ref = artifact_fixture(tmp_path)
    result = subprocess.run(['ros2', 'run', 'unloading_ros_bridge', 'perception_node', '--ros-args',
        '-p', 'backend:=algorithm_replay', '-p', 'replay_path:='+ref['path'],
        '-p', 'artifact_sha256:="'+'0'*64+'"'], capture_output=True, text=True, encoding='utf-8', timeout=15,
        env={**os.environ, 'ROS_DOMAIN_ID': str(180+os.getpid()%30)})
    assert result.returncode != 0 and 'HASH_MISMATCH' in result.stderr


def test_algorithm_replay_original_identity_validated_without_online_admission(tmp_path):
    ref = artifact_fixture(tmp_path)
    record = algorithm_replay_record(ref['path'], ref['sha256'], 'synthetic-display-session')
    first = replay_publication(record, sequence=0, elapsed=.1, published_time=100., clock_domain='ros')
    rclpy.init()
    online = WorldBridgeNode()
    try:
        online.on_observation(observation_to_msg(first))
        assert online.last_observation is None and online.algorithm_observation_guard.sequence == -1
        assert 'OBSERVATION_CLOCK_DOMAIN_MISMATCH' in online.last_time_rejection['blocking_reasons']
    finally:
        online.destroy_node()
        rclpy.shutdown()
    rclpy.init(args=['--ros-args', '-p', 'observation_mode:=replay_display_only'])
    display = WorldBridgeNode()
    try:
        display.on_observation(observation_to_msg(first))
        saved = display.last_observation
        assert saved is not None
        # Valid envelope, altered inner algorithm evidence: reject before guard.
        later = replay_publication(record, sequence=1, elapsed=1., published_time=101., clock_domain='ros')
        display.on_observation(observation_to_msg(replace(later, cargo=())))
        assert display.last_observation is saved and display.replay_guard.sequence == 0
        display.on_observation(observation_to_msg(later))
        assert display.replay_guard.sequence == 1
    finally:
        display.destroy_node()
        rclpy.shutdown()
