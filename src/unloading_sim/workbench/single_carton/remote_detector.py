"""Field-PC client: Grounding DINO + SAM on the GPU server through SSH.

Same interface as the local detector (detect_boxes / segment), so LiveVision's
single model owner and latest-frame scheduling are unchanged. Each connection
uploads hf_detector_core.py and remote_detector_server.py, and the server must
report identical SHA-256 for both and for every local model file before any
inference. Images go up as lossless PNG; masks come back bit-packed in an npz
loaded without pickle. Failures raise and surface in the UI; there is no
silent fallback to another backend. Inference never authorizes motion.
"""
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
import hashlib
import numpy as np
from .hf_detector_core import directory_hashes
from .remote_detector_server import PROTOCOL, read_frame, write_frame

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
CODE_FILES = ('hf_detector_core.py', 'remote_detector_server.py')
REQUIRED = ('host', 'port', 'user', 'identity_file', 'python', 'workdir', 'model_root', 'device')


class RemoteDetectorError(RuntimeError):
    pass


def remote_config_issues(remote):
    """Explicit connection fields; host details live in the ignored device config."""
    if not isinstance(remote, dict): return ['detector.remote：远程 GPU 检测连接参数（主机、端口、用户、密钥、Python、目录）']
    errors = [f'detector.remote.{k}' for k in REQUIRED if remote.get(k) in (None, '')]
    if not errors:
        if type(remote['port']) != int or not 0 < remote['port'] < 65536: errors.append('detector.remote.port：1–65535')
        key = Path(os.path.expanduser(remote['identity_file']))
        if not key.is_file(): errors.append(f'detector.remote.identity_file：密钥文件不存在 {key}')
        elif key.stat().st_mode & 0o077: errors.append(f'detector.remote.identity_file：密钥权限须为 600 {key}')
        if not str(remote['device']).startswith('cuda'): errors.append('detector.remote.device：远程检测须使用 cuda 设备')
    for k, default, low, high in (('request_timeout_s', 20., 1., 120.), ('connect_timeout_s', 240., 10., 900.)):
        value = remote.get(k, default) if isinstance(remote, dict) else default
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            errors.append(f'detector.remote.{k}：{low:g}–{high:g} s')
    return errors


def encode_png(rgb):
    from PIL import Image
    rgb = np.ascontiguousarray(rgb)
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3: raise ValueError('RGB_UINT8_FRAME_REQUIRED')
    buffer = io.BytesIO(); Image.fromarray(rgb, 'RGB').save(buffer, format='PNG', compress_level=1)
    data = buffer.getvalue(); return data, hashlib.sha256(data).hexdigest()


def _finite_box(box):
    return (isinstance(box, dict) and isinstance(box.get('label'), str) and isinstance(box.get('score'), (int, float))
            and 0 <= box['score'] <= 1 and isinstance(box.get('bbox_xyxy'), list) and len(box['bbox_xyxy']) == 4
            and all(isinstance(x, (int, float)) and math.isfinite(x) for x in box['bbox_xyxy']))


class RemoteGroundingSAM:
    loading_status = '正在连接 GPU 服务器、校验模型并加载'

    def __init__(self, dino_directory, sam_directory, prompt='carton . box .', box_threshold=.35, text_threshold=.30,
                 remote=None, command=None, deploy=None, **_):
        issues = remote_config_issues(remote) if command is None else []
        if issues: raise RemoteDetectorError('REMOTE_DETECTOR_CONFIG: '+'；'.join(issues))
        self.remote = dict(remote or {}); self.command = command; self.deploy = command is None if deploy is None else deploy
        self.request_timeout = float(self.remote.get('request_timeout_s', 20.)); self.connect_timeout = float(self.remote.get('connect_timeout_s', 240.))
        self.models = {key: dict(relative_directory=os.path.relpath(Path(path).resolve(), ROOT), files=directory_hashes(path))
                       for key, path in (('dino', dino_directory), ('sam', sam_directory))}
        if any(m['relative_directory'].startswith('..') for m in self.models.values()): raise RemoteDetectorError('MODEL_DIRECTORY_OUTSIDE_REPOSITORY')
        self.options = dict(device=self.remote.get('device', 'cuda:0'), prompt=prompt, box_threshold=box_threshold, text_threshold=text_threshold)
        self.code = {name: hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in CODE_FILES}
        self.lock = threading.Lock(); self.process = None; self.replies = None; self.counter = 0; self.info = None; self.log = None
        self.connect()

    # -- transport -----------------------------------------------------------------
    def ssh(self, remote_command):
        r = self.remote
        return ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=15',
                '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3', '-i', os.path.expanduser(r['identity_file']),
                '-p', str(r['port']), f"{r['user']}@{r['host']}", remote_command]

    def upload_code(self):
        buffer = io.BytesIO()
        with tarfile.open(fileobj=buffer, mode='w') as tar:
            for name in CODE_FILES: tar.add(HERE/name, arcname=name)
        workdir = shlex.quote(self.remote['workdir'])
        done = subprocess.run(self.ssh(f'mkdir -p {workdir} && tar -x -m -C {workdir}'), input=buffer.getvalue(),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
        if done.returncode: raise RemoteDetectorError('REMOTE_CODE_UPLOAD_FAILED: '+done.stderr.decode(errors='replace')[-400:])

    def connect(self):
        self.close()
        if self.deploy: self.upload_code()
        command = self.command or self.ssh(f"cd {shlex.quote(self.remote['workdir'])} && exec {shlex.quote(self.remote['python'])} "
                                           f"-u remote_detector_server.py --model-root {shlex.quote(self.remote['model_root'])}")
        logs = ROOT/self.remote.get('log_dir', 'outputs/workbench/remote_detector'); logs.mkdir(parents=True, exist_ok=True)
        self.log = open(logs/f'{time.strftime("%Y%m%dT%H%M%S")}-{os.getpid()}.log', 'ab')
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
        replies = queue.Queue(); self.replies = replies; process = self.process
        def pump():
            try:
                while True:
                    frame = read_frame(process.stdout)
                    if frame is None: replies.put(('EOF', None)); return
                    replies.put(('FRAME', frame))
            except Exception as error: replies.put(('EOF', error))
        threading.Thread(target=pump, daemon=True, name='remote-detector-reader').start()
        started = time.monotonic()
        reply, _ = self.exchange(dict(op='init', protocol=PROTOCOL, models=self.models, options=self.options), b'', self.connect_timeout)
        if reply.get('status') != 'READY': raise self.fail('REMOTE_DETECTOR_INIT_FAILED: '+str(reply.get('error', reply.get('status'))))
        if reply.get('core_sha256') != self.code['hf_detector_core.py'] or reply.get('server_sha256') != self.code['remote_detector_server.py']:
            raise self.fail('REMOTE_DETECTOR_CODE_MISMATCH')
        self.info = dict(reply, connect_s=time.monotonic()-started)
        self.description = f"远程 GPU {self.info.get('gpu') or self.options['device']}"

    def fail(self, reason):
        self.close(); return RemoteDetectorError(reason)

    def exchange(self, header, payload, timeout):
        if self.process is None or self.process.poll() is not None: raise self.fail('REMOTE_DETECTOR_NOT_CONNECTED')
        self.counter += 1; header = dict(header, id=self.counter)
        try: write_frame(self.process.stdin, header, payload)
        except (BrokenPipeError, OSError) as error: raise self.fail(f'REMOTE_DETECTOR_CONNECTION_LOST: {error}')
        try: kind, frame = self.replies.get(timeout=timeout)
        except queue.Empty: raise self.fail(f'REMOTE_DETECTOR_TIMEOUT: {header["op"]} 超过 {timeout:g} s 无应答')
        if kind != 'FRAME': raise self.fail(f'REMOTE_DETECTOR_CONNECTION_LOST: {frame}')
        reply, data = frame
        if reply.get('id') != header['id'] or reply.get('op') != header['op']: raise self.fail('REMOTE_DETECTOR_PROTOCOL_MISMATCH')
        return reply, data

    def request(self, header, payload=b''):
        with self.lock:
            if self.process is None or self.process.poll() is not None: self.connect()
            started = time.monotonic(); reply, data = self.exchange(header, payload, self.request_timeout)
            if header['op'] == 'segment' and reply.get('status') == 'IMAGE_NOT_CACHED' and not payload:
                return None, None, time.monotonic()-started
            if reply.get('status') != 'OK': raise RemoteDetectorError('REMOTE_INFERENCE_FAILED: '+str(reply.get('error', reply.get('status'))))
            return reply, data, time.monotonic()-started

    def inference(self, reply, round_trip, image_sha256):
        info = self.info or {}
        return dict(backend='REMOTE_SSH_GPU', host=self.remote.get('host', 'TEST_COMMAND'), gpu=info.get('gpu'), device=self.options['device'],
                    torch=info.get('torch'), transformers=info.get('transformers'), server_inference_s=reply.get('inference_s'),
                    round_trip_s=round_trip, image_sha256=image_sha256, model_files_verified=bool(info.get('model_files_verified')))

    # -- detector interface ----------------------------------------------------------
    def detect_boxes(self, frame):
        from .perception import detection_result
        png, digest = encode_png(frame.rgb)
        reply, _, elapsed = self.request(dict(op='detect', image_sha256=digest), png)
        boxes = reply.get('detections')
        if (reply.get('image_sha256') != digest or reply.get('shape') != list(frame.rgb.shape) or
                not isinstance(boxes, list) or not all(_finite_box(b) for b in boxes)):
            raise RemoteDetectorError('REMOTE_DETECTION_RESULT_INVALID')
        return detection_result(frame, boxes, self.inference(reply, elapsed, digest))

    def segment(self, frame, detection):
        from .perception import segmentation_result
        if detection.observation_id != frame.observation_id: raise ValueError('SAM_FRAME_MISMATCH')
        png, digest = encode_png(frame.rgb); boxes = [item['bbox_xyxy'] for item in detection.detections]
        if not boxes: return segmentation_result(frame, detection, [], [])
        header = dict(op='segment', image_sha256=digest, boxes=boxes)
        reply, data, elapsed = self.request(header)
        if reply is None: reply, data, more = self.request(header, png); elapsed += more
        shape = tuple(frame.rgb.shape[:2]); ious = reply.get('ious')
        if (reply.get('image_sha256') != digest or tuple(reply.get('shape', ())) != shape or reply.get('count') != len(boxes)
                or not isinstance(ious, list) or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in ious)):
            raise RemoteDetectorError('REMOTE_SEGMENTATION_RESULT_INVALID')
        with np.load(io.BytesIO(data), allow_pickle=False) as packed:
            if sorted(packed.files) != sorted(f'mask_{i}' for i in range(len(boxes))): raise RemoteDetectorError('REMOTE_SEGMENTATION_RESULT_INVALID')
            masks = [np.unpackbits(packed[f'mask_{i}'], count=shape[0]*shape[1]).reshape(shape).astype(bool) for i in range(len(boxes))]
        return segmentation_result(frame, detection, masks, ious, self.inference(reply, elapsed, digest))

    def detect(self, frame):
        return self.segment(frame, self.detect_boxes(frame))

    def close(self):
        process, self.process = self.process, None
        if process is not None:
            try: process.stdin.close()
            except OSError: pass
            try: process.wait(3)
            except subprocess.TimeoutExpired: process.kill()
        if self.log is not None: self.log.close(); self.log = None
