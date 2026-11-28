"""Remote Qwen VLM protocol with the real server script and a fake core (no network, no GPU, no model)."""
import hashlib
import os
from pathlib import Path
import sys

import numpy as np
import pytest

from unloading_sim.workbench.brain.vlm_client import RemoteQwenVL, VLMError, encode_jpeg, load_model_hashes, remote_config_issues
from unloading_sim.workbench.brain.vlm_server import verify_model_files

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/unloading_sim/workbench/brain'

FAKE = '''
import os, sys, time
sys.path.insert(0, {package!r})
import vlm_server as server
class FakeCore:
    info = dict(gpu='FAKE', torch='0', transformers='0')
    def __init__(self, directory, device, max_pixels):
        assert device == 'cuda:0' and max_pixels == 640 * 480
    def generate(self, messages, images, max_new_tokens, budget_s=None):
        text = messages[-1]['content'][-1]['text']
        if 'BUDGET' in text: return 'partial', dict(input_tokens=7, output_tokens=1, budget_exceeded=True)
        if 'SLEEP' in text: time.sleep(30)
        if 'CRASH' in text: os._exit(3)
        return 'images=%d tokens=%d last=%s' % (len(images), max_new_tokens, text[-5:]), dict(input_tokens=7, output_tokens=3)
sys.stdout = sys.stderr
sys.exit(server.serve(sys.stdin.buffer, sys.__stdout__.buffer, {root!r}, core_factory=FakeCore))
'''


@pytest.fixture
def model(tmp_path):
    d = tmp_path / 'Qwen'
    d.mkdir()
    (d / 'model.safetensors').write_bytes(b'w' * 1000)
    (d / 'config.json').write_text('{}')
    files = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in d.iterdir()}
    return tmp_path, dict(relative_directory='Qwen', files=files)


def client(root, model, tmp_path, **remote):
    command = [sys.executable, '-c', FAKE.format(package=str(PACKAGE), root=str(root))]
    return RemoteQwenVL({'device': 'cuda:0', 'request_timeout_s': 5., 'connect_timeout_s': 30., **remote}, model,
                        command=command, log_dir=str(tmp_path / 'logs'))


def text(t):
    return [dict(role='user', content=[dict(type='image'), dict(type='text', text=t)])]


def test_connect_verifies_model_files_and_code_then_generates(model, tmp_path):
    root, m = model
    vlm = client(root, m, tmp_path)
    info = vlm.connect()
    assert info['model_files_verified'] and info['gpu'] == 'FAKE'
    assert set(info['file_hash_states'].values()) == {'REHASHED'}
    image, digest = encode_jpeg(np.full((48, 64, 3), 90, np.uint8))
    reply = vlm.generate(text('hello world'), [image], 99)
    assert reply['text'] == 'images=1 tokens=99 last=world' and reply['image_sha256'] == [digest]
    vlm.close()
    again = client(root, m, tmp_path).connect()
    assert set(again['file_hash_states'].values()) == {'REUSED'}


def test_server_refuses_model_files_that_differ(model, tmp_path):
    root, m = model
    bad = dict(m, files=dict(m['files'], **{'config.json': '0' * 64}))
    with pytest.raises(VLMError, match='MODEL_FILES_DIFFER'):
        client(root, bad, tmp_path).connect()
    missing = dict(m, files=dict(m['files'], **{'extra.bin': '0' * 64}))
    with pytest.raises(VLMError, match='MISSING'):
        client(root, missing, tmp_path).connect()


def test_model_directory_may_not_leave_the_model_root(model, tmp_path):
    root, m = model
    with pytest.raises(VLMError, match='OUTSIDE_ROOT'):
        client(root, dict(m, relative_directory='../x'), tmp_path).connect()


def test_reported_code_hash_must_match(model, tmp_path):
    root, m = model
    vlm = client(root, m, tmp_path)
    vlm.code = dict(vlm.code, **{'vlm_server.py': '0' * 64})
    with pytest.raises(VLMError, match='CODE_MISMATCH'):
        vlm.connect()


def test_image_count_must_match_messages(model, tmp_path):
    root, m = model
    vlm = client(root, m, tmp_path)
    vlm.connect()
    with pytest.raises(VLMError, match='INVALID_MESSAGES'):
        vlm.generate(text('no image sent'), [], 10)


def test_timeout_and_crash_fail_explicitly_then_reconnect(model, tmp_path):
    root, m = model
    vlm = client(root, m, tmp_path, request_timeout_s=2.)
    vlm.connect()
    image, _ = encode_jpeg(np.zeros((8, 8, 3), np.uint8))
    with pytest.raises(VLMError, match='TIMEOUT'):
        vlm.generate(text('SLEEP'), [image])
    assert vlm.process is None
    with pytest.raises(VLMError, match='CONNECTION_LOST'):
        vlm.generate(text('CRASH'), [image])
    assert vlm.generate(text('fine again'), [image])['text'].endswith('again')


def test_remote_config_requires_explicit_fields(tmp_path):
    assert remote_config_issues(None)
    key = tmp_path / 'k'
    key.write_text('x')
    key.chmod(0o644)
    remote = dict(host='h', port=22, user='root', identity_file=str(key), python='p', workdir='w', model_root='m', device='cuda:0')
    assert any('600' in e for e in remote_config_issues(remote))
    key.chmod(0o600)
    assert remote_config_issues(remote) == []
    assert remote_config_issues(dict(remote, device='cpu'))
    assert 'password' not in str(remote_config_issues(remote))


def test_hash_cache_rehashes_after_change(tmp_path):
    f = tmp_path / 'a.bin'
    f.write_bytes(b'1')
    cache = tmp_path / 'cache.json'
    digest = hashlib.sha256(b'1').hexdigest()
    assert verify_model_files(tmp_path, {'a.bin': digest}, cache) == {'a.bin': 'REHASHED'}
    assert verify_model_files(tmp_path, {'a.bin': digest}, cache) == {'a.bin': 'REUSED'}
    f.write_bytes(b'2')
    st = f.stat()
    os.utime(f, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000))  # same-tick rewrite: stamp must differ
    with pytest.raises(ValueError, match='SHA256_MISMATCH'):
        verify_model_files(tmp_path, {'a.bin': digest}, cache)


def test_committed_hash_lists_parse():
    qwen = load_model_hashes(ROOT / 'configs/brain/qwen2.5-vl-7b-instruct.sha256')
    assert len(qwen) == 15 and sum(k.endswith('.safetensors') for k in qwen) == 5
    voice = load_model_hashes(ROOT / 'configs/brain/voice_models.sha256')
    assert 'silero_vad.onnx' in voice and 'vits-melo-tts-zh_en/model.onnx' in voice
    assert not any(k.endswith('model.int8.onnx') and 'melo' in k for k in voice)  # release ships an LFS pointer there


def test_qwen35_hash_list_is_the_configured_model():
    from unloading_sim.workbench.brain import settings as brain_settings
    files = load_model_hashes(ROOT / 'configs/brain/qwen3.5-9b.sha256')
    assert len(files) == 14 and sum(k.endswith('.safetensors') for k in files) == 4
    s, _ = brain_settings.load(ROOT / 'configs/brain/eco65_brain.json')
    assert s['vlm']['model']['name'] == 'Qwen/Qwen3.5-9B' and s['vlm']['model_files'] == files


def test_thinking_block_is_removed_before_parsing():
    from unloading_sim.workbench.brain.vlm_server import strip_thinking
    assert strip_thinking('<think>\n先看箱子\n</think>\n\n{"a": 1}') == '{"a": 1}'
    assert strip_thinking('{"a": 1}') == '{"a": 1}'


def test_generation_over_budget_is_reported_not_returned_as_an_answer(model, tmp_path):
    root, m = model
    vlm = client(root, m, tmp_path)
    vlm.connect()
    image, _ = encode_jpeg(np.zeros((8, 8, 3), np.uint8))
    with pytest.raises(VLMError, match='BUDGET_EXCEEDED'):
        vlm.generate(text('BUDGET'), [image])
    assert vlm.generate(text('still fine'), [image])['text'].endswith('fine')


def test_budget_stopping_criterion():
    from unloading_sim.workbench.brain.vlm_server import Budget
    assert Budget(None)(None, None) is False
    b = Budget(1e-9)
    import time as _t
    _t.sleep(.01)
    assert b(None, None) is True and b.exceeded
