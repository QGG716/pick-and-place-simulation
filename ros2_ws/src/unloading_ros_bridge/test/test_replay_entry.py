"""Exercise the actual replay launch, file reader and world node over DDS."""
import os
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import pytest
from unloading_interfaces.msg import ExecutionAuthorization, PerceptionObservation, PlanningWorldSnapshot
from unloading_ros_bridge.mapping import snapshot_from_msg
from visualization_msgs.msg import MarkerArray
from unloading_ros_bridge.marker_display import marker_qos

ROOT = Path(__file__).resolve().parents[4]
FIXTURE = ROOT / 'tests/fixtures/vision_upstream/cargo7_minimal.json'


@pytest.mark.parametrize('known_time', [False, True])
def test_real_replay_launch_produces_read_only_world(tmp_path, known_time):
    source = FIXTURE
    if known_time:
        data = json.loads(FIXTURE.read_text(encoding='utf-8'))
        data['source_record'] = dict(record_id='synthetic-historical-record',
            capture_time=42., clock_domain='historical-camera', source_sequence=7)
        source = tmp_path / 'synthetic-known-time.json'
        source.write_text(json.dumps(data), encoding='utf-8')
    domain = 180 + os.getpid() % 30
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('replay_entry_observer')
    worlds, observations, markers, authorizations = [], [], [], []
    node.create_subscription(PlanningWorldSnapshot, '/unloading/world_snapshot', worlds.append, 10)
    node.create_subscription(PerceptionObservation, '/unloading/perception', observations.append, 10)
    node.create_subscription(MarkerArray, '/unloading/markers', markers.append, marker_qos())
    node.create_subscription(ExecutionAuthorization, '/unloading/execution_authorization', authorizations.append, 10)
    path = tmp_path / 'launch.log'
    with path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(['ros2', 'launch', str(ROOT / 'ros2_ws/src/unloading_bringup/launch/replay_mock.launch.py'),
            f'replay_path:={source}'], env={**os.environ, 'ROS_DOMAIN_ID': str(domain)},
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 12.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if len(worlds) >= 3 and len(observations) >= 3 and markers:
                    break
            assert len(observations) >= 3, path.read_text(encoding='utf-8')
            assert len(worlds) >= 3, path.read_text(encoding='utf-8')
            assert all(not w.planning_admissible for w in worlds)
            assert all('HISTORICAL_REPLAY_DISPLAY_ONLY' in w.blocking_reasons for w in worlds)
            assert len({o.source_sequence for o in observations}) >= 3
            assert len({o.observation_id for o in observations}) == 1
            assert len({o.source_epoch for o in observations}) == 1
            assert len({w.scene_fingerprint for w in worlds}) == 1
            assert all(o.exact_capture_time_s == 0. and o.clock_domain == 'replay' for o in observations)
            for observation in observations:
                meta = json.loads(observation.coverage_json)['replay']
                assert meta['source_uri'] == source.as_uri()
                assert meta['source_sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
                assert meta['original_capture_time'] == (42. if known_time else None)
                assert meta['original_time_status'] == ('PROVIDED' if known_time else 'NOT_PROVIDED')
                assert meta['publication_sequence'] == observation.source_sequence
                assert meta['session_elapsed_seconds'] == observation.exact_processed_time_s
                assert meta['published_time'] > observation.exact_processed_time_s
            for world in worlds:
                domain_snapshot = snapshot_from_msg(world)  # verifies the actual fingerprint envelope
                scene = domain_snapshot.scene_snapshot
                assert not scene['planning_admissible']
                assert tuple(world.blocking_reasons) == scene['blocking_reasons']
                assert world.source_capture_time.sec == world.source_capture_time.nanosec == 0
                assert world.published_time.sec > 0 and world.obstacles and world.unknown_regions
                assert 'UNKNOWN_OR_UNTRANSFORMED_REGIONS' in world.blocking_reasons
                assert any(r.reason == 'WORLD_TRANSFORM_MISSING' for r in world.unknown_regions)
                assert any('MONOCULAR_SCALE_UNVERIFIED' in c.eligibility_reasons for c in world.obstacles)
                assert scene['perception_source']['coverage']['replay']['original_capture_time'] == (42. if known_time else None)
                assert domain_snapshot.tool_attachment['details']['verified'] == 'synthetic_fixture'
                assert world.robot_model_fingerprint == 'synthetic-robot-v1'
            assert any('HISTORICAL REPLAY / READ ONLY' in m.text for a in markers for m in a.markers)
            assert not authorizations
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()


@pytest.mark.parametrize('failure', ['missing', 'corrupt'])
def test_actual_entry_bad_file_never_publishes_success(tmp_path, failure):
    source = tmp_path / 'invalid.json'
    if failure == 'corrupt': source.write_text('{broken', encoding='utf-8')
    domain = 180 + os.getpid() % 30
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('invalid_replay_observer')
    worlds, observations = [], []
    node.create_subscription(PlanningWorldSnapshot, '/unloading/world_snapshot', worlds.append, 10)
    node.create_subscription(PerceptionObservation, '/unloading/perception', observations.append, 10)
    path = tmp_path / 'failed-launch.log'
    with path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(['ros2', 'launch', str(ROOT / 'ros2_ws/src/unloading_bringup/launch/replay_mock.launch.py'),
            f'replay_path:={source}'], env={**os.environ, 'ROS_DOMAIN_ID': str(domain)},
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 12.
            evidence = ''
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                evidence = path.read_text(encoding='utf-8')
                if 'process has died' in evidence and ('JSONDecodeError' in evidence or 'replay_path does not exist' in evidence):
                    break
            assert 'process has died' in evidence, evidence
            assert ('JSONDecodeError' if failure == 'corrupt' else 'replay_path does not exist') in evidence
            assert not worlds and not observations
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()
