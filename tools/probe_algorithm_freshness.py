"""Bounded real-artifact DDS timing: startup, steady state, source stop and recovery."""
import argparse
from collections import Counter, deque
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from unloading_interfaces.msg import MechanismState, PerceptionObservation, PlanningWorldSnapshot, ExecutionAuthorization
from visualization_msgs.msg import MarkerArray, Marker
from unloading_contracts import canonical_fingerprint
from unloading_perception.algorithm_artifact import load_algorithm_artifact, validate_algorithm_replay
from unloading_ros_bridge.mapping import observation_from_msg, snapshot_from_msg
from unloading_ros_bridge.marker_display import marker_qos

ROOT = Path(__file__).resolve().parents[1]


def stamp(value):
    return value.sec+value.nanosec/1e9


def stats(values):
    values = sorted(values)
    if not values: return {'count': 0}
    return {'count': len(values), 'min': values[0], 'max': values[-1],
            **{f'p{n}': values[round((len(values)-1)*n/100)] for n in (50,95,99)}}


def timing_summary(output, phase_times):
    """Post-process bounded node buffers after measurement/exit, on one host clock."""
    result = {}
    for path in sorted((output/'timing').glob('*.json')):
        data = json.loads(path.read_text(encoding='utf-8'))
        windows = {}
        for phase, (start, end) in phase_times.items():
            events = [e for e in data['events'] if start <= e['monotonic'] <= end]
            by_kind = {}
            for kind in sorted({e['kind'] for e in events}):
                selected = [e for e in events if e['kind']==kind]
                by_kind[kind] = {'count': len(selected),
                    'interval_s': stats(b['monotonic']-a['monotonic'] for a,b in zip(selected,selected[1:])),
                    **{field: stats(e[field] for e in selected if field in e)
                       for field in ('duration','entry_age')}}
            windows[phase] = by_kind
        result[path.name] = {'dropped': data['dropped'], 'windows': windows}
    return result


def probe(summary_path, output, domain, steady_seconds=12.):
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    ref = summary['algorithm_artifact']  # Original expected hash, never recomputed to repair input.
    original, index = load_algorithm_artifact(ref['path'], ref['sha256'])
    output.mkdir(parents=True, exist_ok=False)
    (output/'timing').mkdir()
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('bounded_freshness_observer')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    rows, retained = deque(maxlen=8000), {}
    phase, phase_times = 'discovery', {}
    total = 0
    def callback(kind):
        def receive(message):
            nonlocal total
            started = time.perf_counter()
            now = node.get_clock().now().nanoseconds/1e9
            row = dict(kind=kind, phase=phase, monotonic=started, ros_time=now)
            if kind == 'world':
                row.update(robot_sample=stamp(message.robot_sample_time), mechanism_sample=stamp(message.mechanism_sample_time),
                    evaluated=stamp(message.published_time), reasons=list(message.blocking_reasons),
                    publisher_sequence=message.publisher_sequence, scene_revision=message.scene_revision_sequence,
                    robot_revision=message.robot_state_revision_sequence, mechanism_revision=message.mechanism_revision_sequence,
                    planning_admissible=message.planning_admissible)
            elif kind in ('joints', 'mechanism'):
                row['sample_time'] = stamp(message.header.stamp if kind == 'joints' else message.observed_time)
            retained[(phase, kind)] = message  # At most one large message per phase/topic.
            row['callback_seconds'] = time.perf_counter()-started
            rows.append(row)
            total += 1
        return receive
    for typ, topic, kind, profile in (
        (JointState,'/joint_states','joints',qos_profile_sensor_data),
        (MechanismState,'/unloading/mechanism_state','mechanism',qos),
        (PlanningWorldSnapshot,'/unloading/world_snapshot','world',qos),
        (PerceptionObservation,'/unloading/perception','perception',qos),
        (MarkerArray,'/unloading/markers','markers',marker_qos()),
        (ExecutionAuthorization,'/unloading/execution_authorization','authorization',qos)):
        node.create_subscription(typ,topic,callback(kind),profile)
    def spin_until(predicate, timeout):
        deadline = time.perf_counter()+timeout
        while time.perf_counter() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate(): return True
        return False
    def window(name, duration):
        nonlocal phase
        phase = name
        phase_times[name] = [time.perf_counter()]
        spin_until(lambda: False, duration)
        phase_times[name].append(time.perf_counter())
    process, source_pid, paused = None, None, False
    report = {'algorithm_run_id': index['run_id'], 'artifact': ref, 'phase_times': phase_times,
              'state_control': 'SIGSTOP/SIGCONT same mock process: BOTH robot and mechanism sources',
              'steady_seconds': steady_seconds}
    started = time.perf_counter()
    try:
        with (output/'launch.log').open('w', encoding='utf-8') as log:
            process = subprocess.Popen(['ros2','launch',str(ROOT/'ros2_ws/src/unloading_bringup/launch/algorithm_replay.launch.py'),
                'artifact:='+ref['path'],'artifact_sha256:='+ref['sha256']],
                env={**os.environ,'ROS_DOMAIN_ID':str(domain),'UNLOADING_TIMING_DIRECTORY':str(output/'timing')},
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            discovered = spin_until(lambda: all(node.count_publishers(t)==1 for t in
                ('/joint_states','/unloading/mechanism_state','/unloading/perception','/unloading/world_snapshot','/unloading/markers')),10.)
            report['discovery_seconds'] = time.perf_counter()-started
            if not discovered: raise RuntimeError('ENDPOINT_DISCOVERY_TIMEOUT_OR_UNEXPECTED_PUBLISHER')
            phase = 'initialization'
            initialized = spin_until(lambda: all(any(k[1]==topic for k in retained) for topic in
                                                  ('joints','mechanism','world','perception','markers')),15.)
            report['first_complete_seconds'] = time.perf_counter()-started
            if not initialized: raise RuntimeError('FIRST_COMPLETE_STATE_TIMEOUT')
            # Measured cold admission variants take ~1.2 s each on this artifact.
            # Fixed five-second bound after first complete state; retain every
            # startup sample, never wait until a coincidental fresh message.
            window('warmup',5.)
            window('steady',steady_seconds)
            matches = re.findall(r'\[mock_state_publisher-\d+\]: process started with pid \[(\d+)\]',
                                 (output/'launch.log').read_text(encoding='utf-8'))
            if len(matches)!=1: raise RuntimeError('AMBIGUOUS_OWNED_STATE_PROCESS')
            source_pid = int(matches[0])
            os.kill(source_pid,signal.SIGSTOP); paused = True
            window('stopped',4.)
            os.kill(source_pid,signal.SIGCONT); paused = False
            window('recovery_initialization',2.)
            window('recovered',5.)
            assert node.count_publishers('/unloading/execution_authorization') == 0
            assert node.count_publishers('/unloading/perception') == 1
            late = []
            node.create_subscription(MarkerArray,'/unloading/markers',late.append,marker_qos())
            report['late_subscriber'] = spin_until(lambda: bool(late),3.)
    except Exception as exc:
        report['measurement_error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if paused: os.kill(source_pid,signal.SIGCONT)
        if process is not None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL); process.wait(timeout=3)
        node.destroy_node(); rclpy.shutdown()

    # No JSON parsing, serialization, fingerprints or disk writes in the measurement window.
    analysis_started = time.perf_counter()
    report['dropped_receiver_events'] = total-len(rows)
    report['node_timing'] = timing_summary(output, phase_times)
    report['windows'] = {}
    for name in ('discovery','initialization','warmup','steady','stopped','recovery_initialization','recovered'):
        world = [r for r in rows if r['phase']==name and r['kind']=='world']
        selected = [r for r in rows if r['phase']==name]
        reasons = {reason: {'count':sum(reason in r['reasons'] for r in world),
            'first':min(r['monotonic'] for r in world if reason in r['reasons']),
            'last':max(r['monotonic'] for r in world if reason in r['reasons'])}
            for reason in set(reason for r in world for reason in r['reasons'])}
        report['windows'][name] = {'world_count':len(world),'reasons':reasons,
            'world_interval_s':stats(b['monotonic']-a['monotonic'] for a,b in zip(world,world[1:])),
            **{f'{source}_{where}_age_s':stats(r[clock]-r[source+'_sample'] for r in world)
               for source in ('robot','mechanism') for where,clock in (('evaluation','evaluated'),('receive','ros_time'))},
            'source_counts':dict(Counter(r['kind'] for r in selected)),
            'receiver_callback_s':stats(r['callback_seconds'] for r in selected)}
    try:
        original_fp = canonical_fingerprint(original)
        sizes, durations = {}, {}
        for kind in ('perception','world','markers'):
            message = retained[('recovered',kind)]
            t = time.perf_counter(); sizes[kind] = len(serialize_message(message)); durations[kind+'_serialize_s'] = time.perf_counter()-t
        t = time.perf_counter()
        received = observation_from_msg(retained[('recovered','perception')])
        assert canonical_fingerprint(validate_algorithm_replay(received)) == original_fp
        world = snapshot_from_msg(retained[('recovered','world')])
        source = {c.source_instance_id:c for c in original.cargo}
        assert len(world.scene_snapshot['obstacles']) == len(source)
        for c in world.scene_snapshot['obstacles']:
            assert canonical_fingerprint(c['observed_surfaces']) == canonical_fingerprint(source[c['source_instance_id']].observed_surfaces)
            assert canonical_fingerprint(c['raw_result']) == canonical_fingerprint(source[c['source_instance_id']].raw_result)
        assert {r.region_id for r in original.unknown_regions}.issubset(r['region_id'] for r in world.scene_snapshot['unknown_regions'])
        durations['parse_and_fingerprint_s'] = time.perf_counter()-t
        from urllib.parse import unquote
        surfaces = {(s['module_id'],s['capture_id'],s['source_instance_id'],s['face_id']):s for c in original.cargo for s in c.observed_surfaces}
        patches = 0
        for marker in retained[('recovered','markers')].markers:
            if marker.action != Marker.ADD or marker.type != Marker.LINE_STRIP: continue
            key = json.loads(unquote(marker.ns.split('/evidence/',1)[1]))
            if 'patch' not in key: continue
            points = [tuple(p) for p in surfaces[tuple(key[-4:])]['corners_3d_m']]
            assert [(p.x,p.y,p.z) for p in marker.points] == points+[points[0]]
            patches += 1
        assert patches == sum(len(c.raw_result.get('fusion_diagnostics',{}).get('face_reduction',{}).get('representatives',())) for c in original.cargo)
        report.update(message_bytes=sizes,receiver_postprocessing_s=durations,content_unchanged=True,
                      objects=len(source),surfaces=len(surfaces),marker_representatives=patches,
                      world_unknown_regions=len(world.scene_snapshot['unknown_regions']))
        load_algorithm_artifact(ref['path'],ref['sha256'])
        assert not any(r['kind']=='authorization' for r in rows)
        assert not any(r['planning_admissible'] for r in rows if r['kind']=='world')
        assert not report.get('measurement_error')
        assert len(report['node_timing']) == 3 and all(v['dropped']==0 for v in report['node_timing'].values())
        for name in ('steady','recovered'):
            window_rows = [r for r in rows if r['kind']=='world' and r['phase']==name]
            assert len(window_rows)>=3, name+': too few world heartbeats'
            assert all('HISTORICAL_REPLAY_DISPLAY_ONLY' in r['reasons'] and 'UNKNOWN_OR_UNTRANSFORMED_REGIONS' in r['reasons'] for r in window_rows)
            assert not any('STATE_' in reason for r in window_rows for reason in r['reasons']), name+': state freshness failure'
            assert len({r['robot_sample'] for r in window_rows})>=3
            assert len({r['mechanism_sample'] for r in window_rows})>=3
            assert len({r['scene_revision'] for r in window_rows}) == 1
            assert len({r['robot_revision'] for r in window_rows}) == 1
            assert len({r['mechanism_revision'] for r in window_rows}) == 1
            for source, threshold in (('robot',.5),('mechanism',2.)):
                assert all(0 <= r['evaluated']-r[source+'_sample'] <= threshold for r in window_rows)
                assert all(0 <= r['ros_time']-r[source+'_sample'] <= threshold for r in window_rows)
        stopped = [r for r in rows if r['kind']=='world' and r['phase']=='stopped' and r['monotonic']>phase_times['stopped'][0]+1.]
        assert len(stopped)>=3 and len({r['robot_sample'] for r in stopped})==1 and len({r['mechanism_sample'] for r in stopped})==1
        for source in ('ROBOT','MECHANISM'):
            assert any(source+'_STATE_STALE_OR_TIME_JUMP' in r['reasons'] for r in stopped)
        # Check the actual original thresholds against every stopped-world decision.
        for r in (r for r in rows if r['kind']=='world' and r['phase']=='stopped'):
            for source, threshold in (('robot',.5),('mechanism',2.)):
                stale = r['evaluated']-r[source+'_sample'] > threshold
                assert (source.upper()+'_STATE_STALE_OR_TIME_JUMP' in r['reasons']) == stale
        assert report['late_subscriber'] and total==len(rows)
        report['status']='PASS'
    except Exception as exc:
        report['status']='FAIL'; report['acceptance_error']=f'{type(exc).__name__}: {exc}'
    report['postprocessing_seconds'] = time.perf_counter()-analysis_started
    t=time.perf_counter()
    (output/'receiver_events.json').write_text(json.dumps(list(rows),indent=2),encoding='utf-8')
    report['receiver_file_write_seconds']=time.perf_counter()-t
    (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--summary',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,default=172)
    args=p.parse_args()
    result=probe(args.summary,args.output.resolve(),args.domain_id)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['status']=='PASS' else 1)
