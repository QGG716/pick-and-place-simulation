"""Observation acceptance with fake transport spies, no physical devices."""
from dataclasses import replace
import json
from pathlib import Path
import struct
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from unloading_sim.workbench.backend import WorkbenchBackend
from unloading_sim.workbench.contracts import DeviceTime, Source, Stamp, static_pair
from unloading_sim.workbench.coordinates import Extrinsics, JointMapping, rigid
from unloading_sim.workbench.devices import D435iObserver, RobotObserver, VacuumObserver, StreamProfile
from unloading_sim.workbench.policy import DisabledExecution, ReadOnlyViolation
from unloading_sim.workbench.replay import Replay, generate_demo
from unloading_sim.workbench.reference.vacuum_receive import POSITIVE_THRESHOLD, NEGATIVE_THRESHOLD

ROOT = Path(__file__).resolve().parents[1]


def mapping():
    raw = json.loads((ROOT/'configs/workbench/joint_mapping.json').read_text())
    for key in ('source_names', 'target_names', 'source_indices', 'signs', 'zero_offsets_rad'):
        raw[key] = tuple(raw[key])
    return JointMapping(**raw)


class RobotSpy:
    def __init__(self): self.calls = []
    def rm_create_robot_arm(self, host, port):
        self.calls.append('connect'); return NS(id=1)
    def rm_get_current_arm_state(self):
        self.calls.append('query'); return 0, {'joint': [10, -20, 30, -40, 50, -60], 'arm_err': 0}
    def rm_delete_robot_arm(self): self.calls.append('disconnect')
    def __getattr__(self, name):
        raise AssertionError('Device control API reached: '+name)


def binary(command, payload=b''):
    body = bytes([3, command])+payload
    return b'\xee\x16'+bytes([len(body)])+body+bytes([sum(body) & 255])


class SerialSpy:
    def __init__(self):
        self.data = binary(0x61, struct.pack('<f', -56.25))+binary(0xF1)
        self.closed = False
        self.writes = []
    def read(self, count):
        data, self.data = self.data[:count], self.data[count:]; return data
    def write(self, data):
        self.writes.append(data); raise AssertionError('Serial write is prohibited')
    def close(self): self.closed = True


def wait_until(test, timeout=3):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if test(): return
        time.sleep(.01)
    raise AssertionError('Timed out')


def test_complete_lifecycle_old_config_alarm_and_exit_are_read_only(tmp_path):
    sdk, serial = RobotSpy(), SerialSpy()
    factory_calls = []
    def factory(): factory_calls.append(1); return sdk
    backend = WorkbenchBackend(D435iObserver(), RobotObserver(mapping(), factory), VacuumObserver(lambda _: serial))
    try:
        assert not factory_calls and not sdk.calls
        config = tmp_path/'legacy.json'
        config.write_text(json.dumps({'enable_hardware': True, 'auto_connect': True, 'mode': 'EXECUTE',
                                     'alarm_callback': 'emergency_stop', 'valve': 0x51}))
        assert 'enable_hardware' in backend.import_config(config)
        assert not factory_calls and not sdk.calls
        for device, command in [('robot', 'movej'), ('robot', 'emergency_stop'), ('vacuum', 'send'), ('conveyor', 'start')]:
            with pytest.raises(ReadOnlyViolation): backend.command(device, command)
        backend.command('robot', 'connect', 'fake-device', 8080)
        backend.command('vacuum', 'connect', 'fake-serial')
        wait_until(lambda: sdk.calls.count('query') >= 1 and backend.workers['vacuum'].device.alarms)
        backend.poll()
        assert any('display only' in line for line in backend.logs)
        assert np.allclose(backend.joint.q_rad, np.deg2rad([10,-20,30,-40,50,-60]))
        assert backend.joint.stamp.device.domain == 'UNKNOWN'
        for op in ('rm_movej', 'rm_set_arm_stop', 'rm_set_tool_frame', 'rm_set_arm_power'):
            with pytest.raises(ReadOnlyViolation): backend.workers['robot'].device.request(op)
        with pytest.raises(ReadOnlyViolation): backend.workers['vacuum'].device.send(0x71)
    finally:
        backend.close()
    assert set(sdk.calls) <= {'connect', 'query', 'disconnect'}
    assert sdk.calls[-1] == 'disconnect' and serial.closed and serial.writes == []
    assert all(not w.thread.is_alive() for w in backend.workers.values())


@pytest.mark.parametrize('method,args', [('submit', ({},)), ('feedback', ('id',)), ('cancel', ('id',))])
def test_execution_port_unavailable(method, args):
    with pytest.raises(ReadOnlyViolation): getattr(DisabledExecution(), method)(*args)


def test_failed_robot_connection_and_query_never_become_simulated():
    sdk = RobotSpy()
    sdk.rm_create_robot_arm = lambda *a: NS(id=-1)
    robot = RobotObserver(mapping(), lambda: sdk)
    with pytest.raises(RuntimeError, match='connection failed'): robot.connect('bad', 8080)
    assert not robot.connected
    sdk = RobotSpy()
    sdk.rm_get_current_arm_state = lambda: (1, {})
    robot = RobotObserver(mapping(), lambda: sdk)
    robot.connect('fake', 8080)
    with pytest.raises(RuntimeError, match='query failed'): robot.read()
    assert not robot.connected and sdk.calls == ['connect', 'disconnect']


def test_receive_timestamp_fragment_checksum_and_debug_integrity():
    v = VacuumObserver()
    receipt = replace(Stamp.receive(), received_monotonic_ns=100)
    packet = binary(0x61, struct.pack('<f', -52.5))
    assert v.feed(packet[:4], receipt) == []
    parsed = v.feed(packet[4:], receipt)
    assert parsed[0][1].stamp is receipt and parsed[0][1].kpa == -52.5
    v.feed(binary(0x62, struct.pack('<f', -72.0)), replace(receipt, received_monotonic_ns=300))
    assert v.gauges()[1].stamp.received_monotonic_ns == 100
    assert v.gauges()[2].stamp.received_monotonic_ns == 300
    corrupt = bytearray(binary(0x63, struct.pack('<f', -80.0))); corrupt[-1] ^= 1
    v.feed(corrupt, receipt)
    assert v.gauges()[3] is None and v.bad_frames > 0
    v.feed(b'AI:\r\n 1[-49.0] 2[-70.0] 3[-79.0] 4[241.5]\r\n', receipt)
    assert v.debug[1].integrity == 'ASCII_DEBUG_UNCHECKED'
    assert v.gauges()[4].kpa == 241.5 and v.gauges()[4].integrity == 'ASCII_DEBUG_UNCHECKED'
    assert v.gauges()[1].integrity == 'CHECKED_BINARY'
    assert (POSITIVE_THRESHOLD, NEGATIVE_THRESHOLD) == (0x71, 0x72)
    with pytest.raises(ReadOnlyViolation): v.send(b'anything')


def test_freshness_clock_disconnect_and_replay_preserve_observations(tmp_path):
    path = generate_demo(tmp_path/'fixture')
    frame, joint, pressure, detections = Replay(path).step()
    assert frame.stamp.source == Source.REPLAY and frame.stamp.original_source == 'SIMULATED'
    assert frame.stamp.received_monotonic_ns == 1_000_000_000
    assert frame.rgb_device_time == DeviceTime()
    assert frame.stamp.state(True, 1, frame.stamp.presented_monotonic_ns+2_000_000_000) == 'STALE'
    assert frame.stamp.state(False, 1) == 'DISCONNECTED'
    assert frame.stamp.state(True, 1) == 'REPLAY'
    assert not frame.rgb.flags.writeable and not frame.depth.flags.writeable
    assert detections.observation_id == frame.observation_id
    assert pressure.integrity == 'REPLAY_FIXTURE'
    assert replace(Stamp.receive(), host_session='other-process').state(True, 1) == 'CLOCK_UNKNOWN'
    assert static_pair(frame, joint, camera_connected=True, robot_connected=True).status == 'NO_STATIONARY_EVIDENCE'
    pair = static_pair(frame, joint, camera_connected=True, robot_connected=True, manual_stationary=True)
    assert pair.q_capture_rad == joint.q_rad and not pair.reliable_capture_pair
    assert pair.stationary_evidence == 'MANUAL_CONFIRMATION'
    assert static_pair(frame, joint, camera_connected=False, robot_connected=True, manual_stationary=True).q_capture_rad is None


def test_replay_rejects_detection_from_another_frame_and_path_escape(tmp_path):
    path = generate_demo(tmp_path/'fixture')
    manifest = path/'manifest.json'; data = json.loads(manifest.read_text())
    data['records'][0]['detections']['observation_id'] = 'tracking-overlay'
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='another original'): Replay(path).step()
    data['records'][0]['frame']['arrays'] = '../outside.npz'
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError): Replay(path).step()


@pytest.mark.parametrize('matrix', [np.diag([1,1,-1,1]), [[1,.5,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], np.eye(3), np.full((4,4), np.nan)])
def test_invalid_extrinsics_rejected_without_repair(tmp_path, matrix):
    p = tmp_path/'hand_eye.json'; p.write_text(json.dumps({'matrix': np.asarray(matrix).tolist()}))
    with pytest.raises(ValueError, match='INVALID_RIGID'): Extrinsics.load(p)


def test_training_extrinsics_keeps_status_and_missing_session():
    e = Extrinsics.load(ROOT/'configs/workbench/training_extrinsics.example.json')
    assert e.status == 'SOLVED_REQUIRES_VALIDATION' and e.usage == 'TRAINING_ONLY_TEST'
    assert e.session_status == 'MISSING' and '待独立验证' in e.label


def test_unreviewed_model_identity_is_not_applied(tmp_path):
    data = json.loads((ROOT/'configs/workbench/training_extrinsics.example.json').read_text())
    data['model_sha256'] = 'another-robot'
    path = tmp_path/'external.json'; path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='model identity'): Extrinsics.load(path)


def test_capture_pair_has_no_cross_source_or_clock_promise(tmp_path):
    frame, joint, *_ = Replay(generate_demo(tmp_path/'fixture')).step()
    for stamp in (replace(joint.stamp, host_session='different-recording'),
                  replace(joint.stamp, received_monotonic_ns=4_000_000_000), Stamp.receive()):
        pair = static_pair(frame, replace(joint, stamp=stamp), camera_connected=True, robot_connected=True, manual_stationary=True)
        assert pair.q_capture_rad is None and not pair.reliable_capture_pair


def test_rgbd_metric_geometry_uses_scale_and_original_image(tmp_path):
    from unloading_sim.workbench.detection import metric_points
    frame, *_ = Replay(generate_demo(tmp_path/'fixture')).step()
    points = metric_points(frame)
    np.testing.assert_allclose(points[120,160], [0,0,.65], atol=1e-7)
    np.testing.assert_allclose(points[0,0], [-160/300*.8,-120/300*.8,.8], atol=1e-7)


def test_existing_detection_result_keeps_original_observation_and_mask(tmp_path):
    from unloading_sim.workbench.detection import ExistingGroundingSAM
    frame, *_ = Replay(generate_demo(tmp_path/'fixture')).step()
    class Boxes:
        def detach(self): return self
        def cpu(self): return self
        def numpy(self): return np.array([[85,65,234,184]])
    adapter = ExistingGroundingSAM.__new__(ExistingGroundingSAM)
    adapter.prompt = 'box'
    adapter.dino = NS(run_inference=lambda bgr,prompt: NS(boxes=Boxes(),scores=[.9],labels=('box',)))
    adapter.sam = NS(run_inference=lambda bgr,boxes: [np.array([[85,65],[234,65],[234,184],[85,184]])])
    result = adapter.detect(frame)
    assert result.observation_id == frame.observation_id and result.source == Source.REPLAY
    assert result.detections[0]['geometry_kind'] == 'MASK_MEDIAN_OBSERVATION_NOT_GRASP_POSE'
    assert result.detections[0]['center_m'][2] == pytest.approx(.65)
    assert result.masks[0].sum() == 150*120
    assert frame.rgb[80,100].tolist() == [188,140,83]


def test_history_selects_capture_candidate_not_inference_completion(tmp_path):
    path = generate_demo(tmp_path/'fixture')
    b = WorkbenchBackend(D435iObserver(), RobotObserver(mapping(), lambda: RobotSpy()), VacuumObserver())
    try:
        b.load_replay(path); b.step_replay()
        frame, first = b.frame, b.joint
        b.step_replay()
        assert b.joint is not first and b.joint_for_frame(frame) is first
        assert b.joint_for_frame(b.frame) is b.joint
    finally: b.close()


def test_pending_connection_cannot_race_with_replay(tmp_path):
    entered, release = threading.Event(), threading.Event()
    sdk = RobotSpy()
    def connect(*args): entered.set(); release.wait(3); return NS(id=1)
    sdk.rm_create_robot_arm = connect
    b = WorkbenchBackend(D435iObserver(), RobotObserver(mapping(), lambda: sdk), VacuumObserver())
    try:
        b.command('robot', 'connect', 'fake', 8080)
        assert entered.wait(1)
        with pytest.raises(RuntimeError, match='Disconnect'): b.load_replay(generate_demo(tmp_path/'fixture'))
    finally:
        release.set(); b.close()


def test_fake_realsense_profile_alignment_scale_and_separate_device_clocks():
    calls = []
    intr = NS(width=32, height=24, fx=30., fy=30., ppx=16., ppy=12., model='distortion.none', coeffs=[0.]*5)
    def profile(kind):
        return NS(is_video_stream_profile=lambda: True, stream_type=lambda: kind,
                  format=lambda: 'bgr8' if kind == 'color' else 'z16', fps=lambda: 30,
                  as_video_stream_profile=lambda: NS(width=lambda: 32, height=lambda: 24))
    device = NS(get_info=lambda key: {'name':'Intel RealSense D435I','serial':'fake-123'}[key],
                query_sensors=lambda: [NS(get_stream_profiles=lambda: [profile('color'),profile('depth')])],
                first_depth_sensor=lambda: NS(get_depth_scale=lambda: .002))
    def frame(depth=False):
        return NS(get_timestamp=lambda: 111.5 if depth else 110.25,
                  get_frame_timestamp_domain=lambda: 'hardware_clock', get_frame_number=lambda: 8 if depth else 7,
                  get_data=lambda: np.full((24,32), 500, np.uint16) if depth else np.full((24,32,3), [3,2,1], np.uint8),
                  profile=NS(as_video_stream_profile=lambda: NS(intrinsics=intr)))
    frames = NS(get_color_frame=lambda: frame(), get_depth_frame=lambda: frame(True))
    class Pipeline:
        def start(self, config): calls.append('start'); return NS(get_device=lambda: device)
        def stop(self): calls.append('stop')
        def wait_for_frames(self, timeout): calls.append('receive'); return frames
    class Config:
        def enable_device(self, serial): assert serial == 'fake-123'
        def enable_stream(self, *args): pass
        def can_resolve(self, pipeline): return True
    rs = NS(context=lambda: NS(query_devices=lambda: [device]), camera_info=NS(name='name',serial_number='serial'),
            stream=NS(color='color',depth='depth'), format=NS(bgr8='bgr8',z16='z16'), config=Config,
            pipeline=Pipeline, pipeline_wrapper=lambda p:p,
            align=lambda color: NS(process=lambda raw: (calls.append('align') or raw)))
    camera = D435iObserver(rs)
    assert calls == []
    profiles = camera.connect()
    assert calls == [] and profiles == (StreamProfile(32,24,30),)
    with pytest.raises(ValueError): camera.start(StreamProfile(640,480,99))
    camera.start(profiles[0]); got = camera.read(); camera.disconnect()
    assert calls == ['start','receive','align','stop']
    assert got.rgb_number == 7 and got.depth_number == 8
    assert got.rgb_device_time.value_ms == 110.25 and got.depth_device_time.value_ms == 111.5
    assert got.rgb_device_time.domain == 'hardware_clock'
    assert got.depth_scale_m == .002 and got.aligned_to == 'color'
    assert got.stamp.processed_monotonic_ns >= got.stamp.received_monotonic_ns
    assert got.rgb[0,0].tolist() == [1,2,3] and not camera.connected
