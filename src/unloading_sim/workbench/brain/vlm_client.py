"""Field-PC client of the remote Qwen VLM (same SSH transport as the remote detector).

Each connection uploads vlm_server.py and vlm_protocol.py; the server must
report identical SHA-256 for both and verify every model file against the
SHA-256 list the field PC sends before it answers anything. Failures raise;
there is no silent fallback to another model. Nothing here authorizes motion.
"""
import hashlib
import io
import math
import os
from pathlib import Path
import queue
import shlex
import subprocess
import tarfile
import threading
import time

from .vlm_protocol import PROTOCOL, read_frame, write_frame

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
CODE_FILES = ('vlm_server.py', 'vlm_protocol.py')
REQUIRED = ('host', 'port', 'user', 'identity_file', 'python', 'workdir', 'model_root', 'device')


class VLMError(RuntimeError):
    pass


def remote_config_issues(remote):
    if not isinstance(remote, dict):
        return ['brain.remote：远程 VLM 连接参数（主机、端口、用户、密钥、Python、目录）']
    errors = [f'brain.remote.{k}' for k in REQUIRED if remote.get(k) in (None, '')]
    if not errors:
        if type(remote['port']) != int or not 0 < remote['port'] < 65536:
            errors.append('brain.remote.port：1–65535')
        key = Path(os.path.expanduser(remote['identity_file']))
        if not key.is_file():
            errors.append(f'brain.remote.identity_file：密钥文件不存在 {key}')
        elif key.stat().st_mode & 0o077:
            errors.append(f'brain.remote.identity_file：密钥权限须为 600 {key}')
        if not str(remote['device']).startswith('cuda'):
            errors.append('brain.remote.device：须使用 cuda 设备')
    for k, default, low, high in (('request_timeout_s', 30., 2., 120.), ('connect_timeout_s', 600., 10., 1800.)):
        value = remote.get(k, default)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            errors.append(f'brain.remote.{k}：{low:g}–{high:g} s')
    return errors


def load_model_hashes(path):
    """`<sha256>  <relative file>` lines (sha256sum format)."""
    files = {}
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        digest, name = line.split(None, 1)
        name = name.lstrip('*').strip()
        if len(digest) != 64 or Path(name).is_absolute() or '..' in Path(name).parts:
            raise ValueError(f'MODEL_HASH_LINE_INVALID: {line}')
        files[name] = digest.lower()
    if not files:
        raise ValueError('MODEL_HASH_LIST_EMPTY')
    return files


def encode_jpeg(rgb, quality=90):
    import numpy as np
    from PIL import Image
    rgb = np.ascontiguousarray(rgb)
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('RGB_UINT8_FRAME_REQUIRED')
    buffer = io.BytesIO()
    Image.fromarray(rgb, 'RGB').save(buffer, format='JPEG', quality=quality)
    data = buffer.getvalue()
    return data, hashlib.sha256(data).hexdigest()


class RemoteQwenVL:
    loading_status = '正在连接 GPU 服务器、校验并加载 Qwen VLM'

    def __init__(self, remote, model, *, max_pixels=640 * 480, command=None, deploy=None, log_dir='outputs/brain/remote_vlm'):
        issues = remote_config_issues(remote) if command is None else []
        if issues:
            raise VLMError('REMOTE_VLM_CONFIG: ' + '；'.join(issues))
        self.remote = dict(remote or {})
        self.model = dict(model)
        self.command = command
        self.deploy = command is None if deploy is None else deploy
        self.request_timeout = float(self.remote.get('request_timeout_s', 30.))
        self.connect_timeout = float(self.remote.get('connect_timeout_s', 600.))
        self.options = dict(device=self.remote.get('device', 'cuda:0'), max_pixels=int(max_pixels))
        self.code = {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest() for name in CODE_FILES}
        self.log_dir = ROOT / log_dir
        self.lock = threading.Lock()
        self.process = None
        self.replies = None
        self.counter = 0
        self.info = None
        self.log = None

    def ssh(self, remote_command):
        r = self.remote
        return ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=15',
                '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3', '-i', os.path.expanduser(r['identity_file']),
                '-p', str(r['port']), f"{r['user']}@{r['host']}", remote_command]

    def upload_code(self):
        buffer = io.BytesIO()
        with tarfile.open(fileobj=buffer, mode='w') as tar:
            for name in CODE_FILES:
                tar.add(HERE / name, arcname=name)
        workdir = shlex.quote(self.remote['workdir'])
        done = subprocess.run(self.ssh(f'mkdir -p {workdir} && tar -x -m -C {workdir}'), input=buffer.getvalue(),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
        if done.returncode:
            raise VLMError('REMOTE_VLM_CODE_UPLOAD_FAILED: ' + done.stderr.decode(errors='replace')[-400:])

    def connect(self):
        self.close()
        if self.deploy:
            self.upload_code()
        command = self.command or self.ssh(
            f"cd {shlex.quote(self.remote['workdir'])} && exec {shlex.quote(self.remote['python'])} "
            f"-u vlm_server.py --model-root {shlex.quote(self.remote['model_root'])}")
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log = open(self.log_dir / f'{time.strftime("%Y%m%dT%H%M%S")}-{os.getpid()}.log', 'ab')
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
        replies, process = queue.Queue(), self.process
        self.replies = replies

        def pump():
            try:
                while True:
                    frame = read_frame(process.stdout)
                    if frame is None:
                        replies.put(('EOF', None))
                        return
                    replies.put(('FRAME', frame))
            except Exception as error:
                replies.put(('EOF', error))
        threading.Thread(target=pump, daemon=True, name='remote-vlm-reader').start()
        started = time.monotonic()
        reply, _ = self.exchange(dict(op='init', protocol=PROTOCOL, model=self.model, options=self.options), b'', self.connect_timeout)
        if reply.get('status') != 'READY':
            raise self.fail('REMOTE_VLM_INIT_FAILED: ' + str(reply.get('error', reply.get('status'))))
        if reply.get('server_sha256') != self.code['vlm_server.py'] or reply.get('protocol_sha256') != self.code['vlm_protocol.py']:
            raise self.fail('REMOTE_VLM_CODE_MISMATCH')
        self.info = dict(reply, connect_s=time.monotonic() - started)
        self.description = f"远程 GPU {self.info.get('gpu') or self.options['device']}"
        return self.info

    def fail(self, reason):
        self.close()
        return VLMError(reason)

    def exchange(self, header, payload, timeout):
        if self.process is None or self.process.poll() is not None:
            raise self.fail('REMOTE_VLM_NOT_CONNECTED')
        self.counter += 1
        header = dict(header, id=self.counter)
        try:
            write_frame(self.process.stdin, header, payload)
        except (BrokenPipeError, OSError) as error:
            raise self.fail(f'REMOTE_VLM_CONNECTION_LOST: {error}')
        try:
            kind, frame = self.replies.get(timeout=timeout)
        except queue.Empty:
            raise self.fail(f'REMOTE_VLM_TIMEOUT: {header["op"]} 超过 {timeout:g} s 无应答')
        if kind != 'FRAME':
            raise self.fail(f'REMOTE_VLM_CONNECTION_LOST: {frame}')
        reply, data = frame
        if reply.get('id') != header['id'] or reply.get('op') != header['op']:
            raise self.fail('REMOTE_VLM_PROTOCOL_MISMATCH')
        return reply, data

    def generate(self, messages, images=(), max_new_tokens=256):
        """messages: [{role, content: [{type: text, text} | {type: image}]}]; images: encoded JPEG/PNG bytes."""
        images = list(images)
        meta = [dict(size=len(d), sha256=hashlib.sha256(d).hexdigest()) for d in images]
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                self.connect()
            started = time.monotonic()
            # The server stops generating a little before the client gives up, so a slow reply never keeps the GPU busy.
            budget = max(1., self.request_timeout - 3.)
            reply, _ = self.exchange(dict(op='generate', messages=messages, images=meta, max_new_tokens=int(max_new_tokens), budget_s=budget),
                                     b''.join(images), self.request_timeout)
            if reply.get('status') != 'OK' or not isinstance(reply.get('text'), str):
                raise VLMError('REMOTE_VLM_GENERATE_FAILED: ' + str(reply.get('error', reply.get('status'))))
            return dict(text=reply['text'], round_trip_s=time.monotonic() - started, inference_s=reply.get('inference_s'),
                        input_tokens=reply.get('input_tokens'), output_tokens=reply.get('output_tokens'),
                        image_sha256=[m['sha256'] for m in meta])

    def close(self):
        process, self.process = self.process, None
        if process is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
            try:
                process.wait(3)
            except subprocess.TimeoutExpired:
                process.kill()
        if self.log is not None:
            self.log.close()
            self.log = None
