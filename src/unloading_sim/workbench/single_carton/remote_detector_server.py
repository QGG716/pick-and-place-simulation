"""GPU-server side of the remote detector: framed messages over stdin/stdout.

Started by the field PC through SSH (no listening port). Runs next to a copy of
hf_detector_core.py; both files are verified by SHA-256 against the field PC.
stdout carries protocol frames only; diagnostics go to stderr.

Frame: b'E65D' | uint32 header length | uint64 payload length | JSON | payload.
Requests: init (expected model file hashes), detect (PNG image), segment
(boxes + image id, PNG resent on cache miss), ping. Masks are returned as an
npz of bit-packed arrays; nothing is ever pickled in either direction.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import struct
import sys
import time
import traceback

MAGIC = b'E65D'
PROTOCOL = 1
HEADER = struct.Struct('>4sIQ')
MAX_HEADER = 1 << 20
MAX_PAYLOAD = 64 << 20


def read_frame(stream):
    raw = stream.read(HEADER.size)
    if not raw: return None
    if len(raw) != HEADER.size: raise EOFError('TRUNCATED_FRAME_HEADER')
    magic, header_len, payload_len = HEADER.unpack(raw)
    if magic != MAGIC or header_len > MAX_HEADER or payload_len > MAX_PAYLOAD: raise ValueError('INVALID_FRAME')
    header = stream.read(header_len); payload = stream.read(payload_len)
    if len(header) != header_len or len(payload) != payload_len: raise EOFError('TRUNCATED_FRAME_BODY')
    return json.loads(header.decode('utf-8')), payload


def write_frame(stream, header, payload=b''):
    data = json.dumps(header, allow_nan=False).encode('utf-8')
    stream.write(HEADER.pack(MAGIC, len(data), len(payload))+data+payload); stream.flush()


def decode_png(payload):
    import numpy as np
    from PIL import Image
    image = Image.open(io.BytesIO(payload))
    if image.format != 'PNG' or image.mode != 'RGB': raise ValueError('RGB_PNG_REQUIRED')
    return np.asarray(image).copy()


def encode_masks(masks):
    import numpy as np
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **{f'mask_{i}': np.packbits(np.asarray(m, bool), axis=None) for i, m in enumerate(masks)})
    return buffer.getvalue()


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def serve(stdin, stdout, model_root, core_factory=None, log=sys.stderr):
    here = Path(__file__).resolve().parent
    sys.path.insert(0, str(here))
    import hf_detector_core as corelib
    core = None; images = {}; order = []
    while True:
        try: frame = read_frame(stdin)
        except Exception as error:
            print('FRAME_ERROR', error, file=log, flush=True); return 2
        if frame is None: return 0
        header, payload = frame; op = header.get('op'); rid = header.get('id')
        started = time.monotonic()
        try:
            if op == 'ping':
                write_frame(stdout, dict(id=rid, status='OK', op=op)); continue
            if op == 'init':
                if header.get('protocol') != PROTOCOL: raise ValueError('PROTOCOL_MISMATCH')
                verified = {}
                for key, spec in header['models'].items():
                    directory = (Path(model_root)/spec['relative_directory']).resolve()
                    if Path(model_root).resolve() not in directory.parents: raise ValueError('MODEL_PATH_OUTSIDE_ROOT')
                    actual = corelib.directory_hashes(directory)
                    if actual != spec['files']:
                        raise ValueError('REMOTE_MODEL_HASH_MISMATCH:'+key+':'+json.dumps(
                            {n: [spec['files'].get(n), actual.get(n)] for n in set(actual) | set(spec['files']) if spec['files'].get(n) != actual.get(n)}))
                    verified[key] = str(directory)
                options = header['options']
                factory = core_factory or corelib.GroundingSamCore
                core = factory(verified['dino'], verified['sam'], options['device'], options['prompt'],
                               options['box_threshold'], options['text_threshold'], options.get('cpu_threads', 2))
                info = dict(core_sha256=file_sha256(here/'hf_detector_core.py'), server_sha256=file_sha256(Path(__file__)),
                            python=sys.version.split()[0])
                try:
                    import torch, transformers
                    info.update(torch=torch.__version__, cuda=torch.version.cuda, transformers=transformers.__version__,
                                gpu=torch.cuda.get_device_name(0) if torch.cuda.is_available() else None)
                except Exception: pass
                write_frame(stdout, dict(id=rid, status='READY', op=op, protocol=PROTOCOL, model_files_verified=True,
                                         load_s=time.monotonic()-started, **info)); continue
            if core is None: raise ValueError('INIT_REQUIRED')
            if op == 'detect':
                rgb = decode_png(payload); key = hashlib.sha256(payload).hexdigest()
                if key != header.get('image_sha256'): raise ValueError('IMAGE_DIGEST_MISMATCH')
                images[key] = rgb; order.append(key)
                while len(order) > 4: images.pop(order.pop(0), None)
                boxes = core.detect(rgb)
                write_frame(stdout, dict(id=rid, status='OK', op=op, image_sha256=key, shape=list(rgb.shape),
                                         detections=boxes, inference_s=time.monotonic()-started)); continue
            if op == 'segment':
                key = header['image_sha256']
                if payload:
                    if hashlib.sha256(payload).hexdigest() != key: raise ValueError('IMAGE_DIGEST_MISMATCH')
                    images[key] = decode_png(payload); order.append(key)
                if key not in images:
                    write_frame(stdout, dict(id=rid, status='IMAGE_NOT_CACHED', op=op)); continue
                masks, ious = core.segment(images[key], header['boxes'])
                write_frame(stdout, dict(id=rid, status='OK', op=op, image_sha256=key, shape=list(images[key].shape[:2]),
                                         ious=ious, count=len(masks), inference_s=time.monotonic()-started), encode_masks(masks)); continue
            raise ValueError('UNKNOWN_OP:'+str(op))
        except Exception as error:
            traceback.print_exc(file=log); log.flush()
            write_frame(stdout, dict(id=rid, status='ERROR', op=op, error=f'{type(error).__name__}: {error}'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-root', required=True)
    args = parser.parse_args()
    out = sys.stdout.buffer; sys.stdout = sys.stderr  # keep stray prints off the protocol stream
    sys.exit(serve(sys.stdin.buffer, out, args.model_root))


if __name__ == '__main__':
    main()
