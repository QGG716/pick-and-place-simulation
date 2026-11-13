"""Remote GPU detector protocol with the real server script and a fake core.

No network, no models, no hardware: the server runs as a local subprocess and
the inference core is injected, so protocol, hash binding, timeouts, crashes,
reconnects and malformed replies are exercised deterministically.
"""
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace
import uuid
import numpy as np
import pytest
from unloading_sim.workbench.contracts import Source
from unloading_sim.workbench.single_carton.remote_detector import RemoteGroundingSAM, RemoteDetectorError, remote_config_issues
from unloading_sim.workbench.single_carton.perception import make_detector

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'src/unloading_sim/workbench/single_carton'

FAKE = '''
import os, sys, time
import numpy as np
sys.path.insert(0, {package!r})
import remote_detector_server as server
class FakeCore:
    def __init__(self, dino, sam, device, prompt, box, text, threads=2):
        assert device == 'cuda:0' and prompt == 'carton . box .'
    def detect(self, rgb):
        mode = int(rgb[0, 0, 0])
        if mode == 1: time.sleep(30)
        if mode == 2: os._exit(3)
        if mode == 3: return [dict(label='carton', score=2.0, bbox_xyxy=[1., 2., 3., 4.])]
        return [dict(label='carton', score=.5, bbox_xyxy=[2., 1., 6., 4.]), dict(label='box', score=.4, bbox_xyxy=[0., 0., 3., 3.])]
    def segment(self, rgb, boxes):
        masks = []
        for x0, y0, x1, y1 in boxes:
            m = np.zeros(rgb.shape[:2], bool); m[int(y0):int(y1), int(x0):int(x1)] = True; masks.append(m)
        return masks, [.9]*len(masks)
sys.stdout = sys.stderr
sys.exit(server.serve(sys.stdin.buffer, sys.__stdout__.buffer, {root!r}, core_factory=FakeCore))
'''


@pytest.fixture
def models():
    base = ROOT/'.test-tmp'/uuid.uuid4().hex; dirs = {}
    for key in ('dino', 'sam'):
        d = base/key; d.mkdir(parents=True); (d/'model.safetensors').write_bytes(key.encode()*100); (d/'config.json').write_text('{}')
        dirs[key] = d
    yield base, dirs
    shutil.rmtree(base)


def detector(dirs, root=ROOT, **remote):
    command = [sys.executable, '-c', FAKE.format(package=str(PACKAGE), root=str(root))]
    return RemoteGroundingSAM(dirs['dino'], dirs['sam'], remote={'device': 'cuda:0', 'request_timeout_s': 5., 'connect_timeout_s': 30., **remote},
                              command=command)


def frame(mode=0, ident='obs-1'):
    rgb = np.full((6, 8, 3), 7, np.uint8); rgb[0, 0, 0] = mode
    return SimpleNamespace(observation_id=ident, rgb=rgb, stamp=SimpleNamespace(source=Source.LIVE))


def test_remote_detection_and_segmentation_bind_same_frame_with_provenance(models):
    d = detector(models[1])
    try:
        f = frame(); detection = d.detect_boxes(f)
        assert detection.observation_id == 'obs-1' and detection.provenance == 'HF_GROUNDING_DINO_ACTUAL_INFERENCE_REMOTE_SSH_GPU'
        assert [x['bbox_xyxy'] for x in detection.detections] == [[2., 1., 6., 4.], [0., 0., 3., 3.]]
        assert all(x['geometry_kind'] == 'DETECTION_ONLY' and x['inference']['backend'] == 'REMOTE_SSH_GPU' for x in detection.detections)
        assert detection.detections[0]['inference']['model_files_verified'] is True
        result = d.segment(f, detection)
        assert result.provenance == 'HF_GROUNDING_DINO_SAM_ACTUAL_INFERENCE_REMOTE_SSH_GPU' and len(result.masks) == 2
        expected = np.zeros((6, 8), bool); expected[1:4, 2:6] = True
        np.testing.assert_array_equal(result.masks[0], expected)
        assert all(m.dtype == bool and m.shape == (6, 8) for m in result.masks)
        assert [x['sam_predicted_iou'] for x in result.detections] == [.9, .9]
        # A frame the server never saw is resent losslessly on the cache miss.
        other = frame(ident='obs-2'); other.rgb[5, 7] = [9, 9, 9]
        with pytest.raises(ValueError, match='SAM_FRAME_MISMATCH'): d.segment(other, detection)
        from unloading_sim.workbench.single_carton.perception import detection_result
        again = d.segment(other, detection_result(other, [dict(label='carton', score=.5, bbox_xyxy=[0., 0., 8., 6.])]))
        assert again.masks[0].all() and again.observation_id == 'obs-2'
    finally: d.close()


def test_server_rejects_model_files_that_differ_from_field_pc(models):
    base, dirs = models
    other = base/'other'; copy = other/dirs['dino'].relative_to(ROOT); copy.mkdir(parents=True)
    shutil.copytree(dirs['sam'], other/dirs['sam'].relative_to(ROOT))
    (copy/'model.safetensors').write_bytes(b'tampered'); (copy/'config.json').write_text('{}')
    with pytest.raises(RemoteDetectorError, match='REMOTE_MODEL_HASH_MISMATCH:dino'):detector(dirs, root=other)


def test_reported_code_hash_must_match_field_pc(models):
    d = detector(models[1])
    try:
        d.code['hf_detector_core.py'] = '0'*64
        with pytest.raises(RemoteDetectorError, match='REMOTE_DETECTOR_CODE_MISMATCH'): d.connect()
        assert d.process is None
    finally: d.close()


def test_timeout_and_crash_fail_explicitly_then_reconnect(models):
    d = detector(models[1], request_timeout_s=1.)
    try:
        with pytest.raises(RemoteDetectorError, match='REMOTE_DETECTOR_TIMEOUT'): d.detect_boxes(frame(1))
        assert d.process is None
        assert len(d.detect_boxes(frame()).detections) == 2  # reconnected, re-verified
        with pytest.raises(RemoteDetectorError, match='CONNECTION_LOST'): d.detect_boxes(frame(2))
        assert len(d.detect_boxes(frame()).detections) == 2
    finally: d.close()


def test_invalid_remote_result_is_rejected_not_displayed(models):
    d = detector(models[1])
    try:
        with pytest.raises(RemoteDetectorError, match='REMOTE_DETECTION_RESULT_INVALID'): d.detect_boxes(frame(3))
    finally: d.close()


def test_remote_config_requires_explicit_fields_and_private_key(tmp_path):
    issues = remote_config_issues(dict(device='cuda:0'))
    for key in ('host', 'port', 'user', 'identity_file', 'python', 'workdir', 'model_root'):
        assert f'detector.remote.{key}' in issues
    key = tmp_path/'id'; key.write_text('x'); key.chmod(0o644)
    remote = dict(host='h', port=22, user='u', identity_file=str(key), python='p', workdir='w', model_root='m', device='cpu')
    issues = remote_config_issues(remote)
    assert any('权限须为 600' in x for x in issues) and any('cuda' in x for x in issues)
    key.chmod(0o600); remote['device'] = 'cuda:0'
    assert remote_config_issues(remote) == []
    assert any('request_timeout_s' in x for x in remote_config_issues(dict(remote, request_timeout_s=0)))
    with pytest.raises(ValueError, match='UNKNOWN_DETECTOR_BACKEND'): make_detector(backend='cloud')


def test_example_config_selects_remote_gpu_with_host_details_outside_repository(tmp_path):
    from unloading_sim.workbench.single_carton.config import load, issues
    c = load(ROOT/'configs/workbench/single_carton.example.json')
    assert c['detector']['backend'] == 'remote_ssh' and c['detector']['remote']['device'] == 'cuda:0'
    text = (ROOT/'configs/workbench/single_carton.example.json').read_text()
    assert 'seetacloud' not in text and 'identity_file' not in text and '"host"' not in text
    raw = json.loads(text); raw['detector']['remote_config'] = str(tmp_path/'missing.json')
    (tmp_path/'c.json').write_text(json.dumps(raw))
    missing = load(tmp_path/'c.json')
    assert 'detector.remote.host' in issues(missing)
