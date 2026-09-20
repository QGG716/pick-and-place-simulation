"""CPU-only orchestration checks; all model objects below are explicit substitutes."""
import json
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import pytest

from unloading_perception.finite_sequence import atomic_json, verify_capture
from unloading_perception.isaac_validation import sha256_file
from unloading_perception.video_demo import LatestFrameSlot, recording_frames, worker_failure
from workcell_once_fakes import capture_fixture, install

ROOT = Path(__file__).resolve().parents[1]


def recording_fixture(tmp_path):
    frames = []
    for index in range(2):
        parent = tmp_path / str(index)
        parent.mkdir()
        capture, _, _ = capture_fixture(parent, ('normal', 'normal'), frame=7+index, stamp=1.+index)
        frames.append(dict(path=str(capture.relative_to(tmp_path)), frame_sequence=7+index,
                           source_time=1.+index, manifest_sha256=sha256_file(capture/'manifest.json')))
    record = dict(schema_version='continuous_rgbd_recording_v1', sensor_epoch='synthetic-joint-capture', frames=frames)
    path = tmp_path/'sequence.json'
    atomic_json(path, record)
    return path, record


def test_latest_complete_pair_is_bounded_and_active_input_does_not_change():
    slot = LatestFrameSlot()
    first = {'frame': 1, 'pair': ('upper-1', 'lower-1')}
    slot.offer(first)
    active = slot.take()
    for frame in range(2, 102):
        slot.offer({'frame': frame, 'pair': (f'upper-{frame}', f'lower-{frame}')})
    assert active == first
    assert slot.dropped == 99
    assert slot.take() == {'frame': 101, 'pair': ('upper-101', 'lower-101')}
    assert slot.pending is None and slot.take() is None


def test_recording_keeps_original_frame_depth_calibration_identity(tmp_path):
    path, record = recording_fixture(tmp_path)
    _, frames = recording_frames(path)
    assert [r['frame_sequence'] for r in frames] == [7, 8]
    for frame in frames:
        inputs = verify_capture(frame['capture'])
        assert len(inputs) == 2
        assert all(p['frame_sequence'] == frame['frame_sequence'] for p in inputs.values())
    assert json.loads(path.read_text(encoding='utf-8')) == record


@pytest.mark.parametrize('fault', ['hash', 'epoch', 'time', 'unpaired', 'escape'])
def test_bad_recording_is_rejected_before_algorithm(tmp_path, fault):
    path, record = recording_fixture(tmp_path)
    if fault == 'hash': record['frames'][1]['manifest_sha256'] = '0'*64
    elif fault == 'epoch': record['sensor_epoch'] = 'other-acquisition'
    elif fault == 'time': record['frames'][1]['source_time'] = 1.
    elif fault == 'escape': record['frames'][1]['path'] = '../foreign'
    else:
        manifest = tmp_path/record['frames'][1]['path']/'manifest.json'
        payload = json.loads(manifest.read_text(encoding='utf-8'))
        payload['cameras'] = payload['cameras'][:1]
        atomic_json(manifest, payload)
        record['frames'][1]['manifest_sha256'] = sha256_file(manifest)
    atomic_json(path, record)
    with pytest.raises(ValueError): recording_frames(path)


def test_failed_worker_details_include_shared_and_module_errors():
    message = worker_failure({'summary': {'errors': [dict(stage='runtime_initialize',
        error_type='ModuleNotFoundError', error='CPU_TEST_MISSING_TORCH')], 'runs': [
            dict(status='TECHNICAL_FAILURE', error='CPU_TEST_GEOMETRY_ERROR'),
            dict(status='COMPLETED_WITH_ALGORITHM_RESULTS', complete_cuboids_accepted=0)]}})
    assert 'runtime_initialize: ModuleNotFoundError: CPU_TEST_MISSING_TORCH' in message
    assert 'CPU_TEST_GEOMETRY_ERROR' in message


def test_resident_model_reused_across_two_real_orchestrations(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'tools'))
    from workcell_video_worker import RuntimeCache
    cache = RuntimeCache()
    captures = []
    for index in range(2):
        parent = tmp_path/str(index)
        parent.mkdir()
        capture, models, _ = capture_fixture(parent, ('normal', 'normal'), frame=7+index, stamp=1.+index)
        with monkeypatch.context() as scope:
            calls = install(scope, capture)  # Explicit CPU substitute for models/geometry only.
            code = runpy.run_path(str(ROOT/'tools/run_workcell_perception_once.py'))['main'](
                ['--capture', str(capture), '--vision', str(tmp_path), '--models', str(models)], runtime_factory=cache)
        assert code == 0
        assert sum(stage == 'sam' for stage, _ in calls) == 2
        report = json.loads((capture/'perception-once/summary.json').read_text(encoding='utf-8'))
        assert all(row['sam_attempts'] == row['metric_attempts'] == 1 for row in report['runs'])
        captures.append(capture)
    assert cache.loads == 1
    assert cache.runtime.output_root == captures[1]/'perception-once/sam-runs'
    with pytest.raises(ValueError, match='identity changed'):
        cache(SimpleNamespace(upstream_root=tmp_path, sam_model='OTHER_MODEL', output_root=tmp_path/'unused'))


def test_launcher_preserves_venv_interpreter_and_does_not_launch_truth_adapter(tmp_path, monkeypatch):
    launcher = runpy.run_path(str(ROOT/'tools/run_rgbd_video_demo.py'))
    calls = []
    output = tmp_path/'demo'
    interpreter = tmp_path/'venv/bin/python'
    original_resolve = Path.resolve
    def resolve(path, *args, **kwargs):
        if path == interpreter: return tmp_path/'base/python'  # Synthetic symlink target, CPU test only.
        return original_resolve(path, *args, **kwargs)
    class Process:
        """No actual ROS/model process is started in this CPU launcher test."""
        pid = 999999
        def poll(self): return 0
        def wait(self, **kwargs): return 0
    def popen(command, **kwargs):
        calls.append(command)
        return Process()
    monkeypatch.setattr(Path, 'resolve', resolve)
    monkeypatch.setattr(launcher['subprocess'], 'Popen', popen)
    launcher['main'].__globals__['recording_frames'] = lambda _: None
    monkeypatch.setattr(sys, 'argv', ['demo', '--recording', str(tmp_path/'source/index.json'),
        '--output', str(output), '--models', str(tmp_path/'models'), '--vision', str(tmp_path/'vision'),
        '--algorithm-python', str(interpreter)])
    with pytest.raises(RuntimeError, match='process exited'): launcher['main']()
    video = next(c for c in calls if 'video_demo_node' in c)
    assert 'algorithm_python:='+str(interpreter.absolute()) in video
    assert not any('isaac_sensor_adapter_node' in c for c in calls)
    assert any('observation_mode:=replay_display_only' in c for c in calls)
