"""Bounded real DDS acceptance of one pinned algorithm artifact; no inference."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from urllib.parse import unquote

import rclpy
from unloading_contracts import canonical_fingerprint, dumps
from unloading_interfaces.msg import ExecutionAuthorization, PerceptionObservation, PlanningWorldSnapshot
from unloading_perception.algorithm_artifact import load_algorithm_artifact, validate_algorithm_replay
from unloading_ros_bridge.mapping import observation_from_msg, snapshot_from_msg
from unloading_ros_bridge.marker_display import marker_qos
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]


def probe(artifact, digest, output, domain):
    original, index = load_algorithm_artifact(artifact, digest)
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('algorithm_replay_acceptance_observer')
    worlds, observations, markers, authorizations = [], [], [], []
    node.create_subscription(PlanningWorldSnapshot, '/unloading/world_snapshot', worlds.append, 10)
    node.create_subscription(PerceptionObservation, '/unloading/perception', observations.append, 10)
    node.create_subscription(MarkerArray, '/unloading/markers', markers.append, marker_qos())
    node.create_subscription(ExecutionAuthorization, '/unloading/execution_authorization', authorizations.append, 10)
    log_path = output/'launch.log'
    process = None
    try:
        if node.count_publishers('/unloading/perception'):
            raise RuntimeError('PERCEPTION_DOMAIN_ALREADY_HAS_PUBLISHER')
        with log_path.open('w', encoding='utf-8') as log:
            process = subprocess.Popen(['ros2', 'launch', str(ROOT/'ros2_ws/src/unloading_bringup/launch/algorithm_replay.launch.py'),
                f'artifact:={artifact}', f'artifact_sha256:={digest}'],
                env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+25.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if len(worlds) >= 3 and len(observations) >= 3 and markers: break
            assert len(worlds) >= 3 and len(observations) >= 3 and markers, log_path.read_text(encoding='utf-8')
            publishers = node.get_publishers_info_by_topic('/unloading/perception')
            assert len(publishers) == 1 and publishers[0].node_name == 'unloading_perception'
            assert not authorizations and node.count_publishers('/unloading/execution_authorization') == 0
            expected = canonical_fingerprint(original)
            for message in observations:
                restored = validate_algorithm_replay(observation_from_msg(message))
                assert canonical_fingerprint(restored) == expected
            assert len({m.source_sequence for m in observations}) >= 3
            assert len({m.source_epoch for m in observations}) == 1
            assert len({m.observation_id for m in observations}) == 1
            for world in worlds:
                snapshot = snapshot_from_msg(world)
                scene = snapshot.scene_snapshot
                assert not world.planning_admissible and not scene['planning_admissible']
                assert 'HISTORICAL_REPLAY_DISPLAY_ONLY' in world.blocking_reasons
                assert tuple(world.blocking_reasons) == scene['blocking_reasons']
                assert scene['perception_source']['coverage']['algorithm_run']['run_id'] == index['run_id']
                source = {c.source_instance_id: c for c in original.cargo}
                assert len(scene['obstacles']) == len(source)
                for c in scene['obstacles']:
                    before = source[c['source_instance_id']]
                    assert canonical_fingerprint(c['observed_surfaces']) == canonical_fingerprint(before.observed_surfaces)
                    assert canonical_fingerprint(c['raw_result']) == canonical_fingerprint(before.raw_result)
                    assert not c['candidate_eligible'] and c['full_dimensions_m'] is None
                assert {r.region_id for r in original.unknown_regions}.issubset(r['region_id'] for r in scene['unknown_regions'])
                assert snapshot.tool_attachment['details']['verified'] == 'synthetic_fixture'
            assert len({w.scene_fingerprint for w in worlds}) == 1
            # A late subscriber gets current geometry without restarting inference.
            late = []
            node.create_subscription(MarkerArray, '/unloading/markers', late.append, marker_qos())
            deadline = time.monotonic()+3.
            while not late and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.05)
            assert late
            surfaces = {(s['module_id'], s['capture_id'], s['source_instance_id'], s['face_id']): s
                        for c in original.cargo for s in c.observed_surfaces}
            representatives = [tuple(ref[k] for k in ('module_id','capture_id','source_instance_id','face_id'))
                for c in original.cargo for ref in c.raw_result.get('fusion_diagnostics', {}).get('face_reduction', {}).get('representatives', ())]
            patch_count = 0
            for marker in late[-1].markers:
                if marker.action != Marker.ADD or marker.type != Marker.LINE_STRIP: continue
                key = json.loads(unquote(marker.ns.split('/evidence/', 1)[1]))
                if 'patch' not in key: continue
                surface = surfaces[tuple(key[-4:])]
                points = [tuple(p) for p in surface['corners_3d_m']]
                assert [(p.x,p.y,p.z) for p in marker.points] == points+[points[0]]
                assert marker.header.frame_id == 'world'
                patch_count += 1
            assert patch_count == len(representatives)
            assert any('HISTORICAL REPLAY / READ ONLY' in m.text for m in late[-1].markers)
            world = worlds[-1]
            (output/'world_snapshot.json').write_text(dumps(snapshot_from_msg(world)), encoding='utf-8')
            (output/'received_observation.json').write_text(dumps(observation_from_msg(observations[-1])), encoding='utf-8')
            from rosidl_runtime_py.convert import message_to_ordereddict
            (output/'markers.json').write_text(json.dumps(message_to_ordereddict(late[-1]), indent=2), encoding='utf-8')
            report = {'status': 'ACTUAL_DDS_READ_ONLY_HANDOFF_PASS', 'algorithm_run_id': index['run_id'],
                'artifact': str(artifact), 'artifact_sha256': digest, 'original_observation': index['observation'],
                'original_epoch': original.source_epoch, 'original_sequence': original.source_sequence,
                'original_capture_time': original.capture_time, 'original_clock_domain': original.clock_domain,
                'replay_session': world.source_epoch, 'planning_admissible': world.planning_admissible,
                'blocking_reasons': list(world.blocking_reasons), 'cargo_count': len(original.cargo),
                'raw_surface_count': len(surfaces), 'marker_representative_count': patch_count,
                'module_surface_counts': dict(Counter(s['module_id'] for s in surfaces.values())),
                'unknown_region_count': len(world.unknown_regions), 'perception_publishers': [p.node_name for p in publishers],
                'execution_authorizations': 0, 'late_subscriber_pass': True,
                'auxiliary_state': 'DISPLAY_ONLY_MOCK_NOT_MEASURED_AT_CAPTURE', 'rviz_visual_check': False}
            (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
            return report
    except Exception as exc:
        (output/'failure.json').write_text(json.dumps({'error_type': type(exc).__name__, 'error': str(exc)}), encoding='utf-8')
        raise
    finally:
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--artifact-sha256', required=True)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=171)
    args = parser.parse_args()
    print(json.dumps(probe(args.artifact.resolve(), args.artifact_sha256, args.output_directory, args.domain_id), indent=2))
