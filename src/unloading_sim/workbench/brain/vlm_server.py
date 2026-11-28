"""GPU-server side of the Qwen VLM advisor: framed messages over stdin/stdout.

Started by the field PC through SSH (no listening port), next to an uploaded
copy of vlm_protocol.py; the field PC checks the SHA-256 this server reports
for both files. stdout carries protocol frames only; diagnostics go to stderr.

Requests: init (model directory + expected SHA-256 of every model file),
generate (chat messages + JPEG/PNG images back to back in the payload), ping.
Model file hashes are reused only while device, inode, size, mtime_ns and
ctime_ns are unchanged (November 16 section 3f); otherwise they are rehashed.
Generation is greedy (do_sample=False) so a reply depends only on its input.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import time
import traceback

try:
    from .vlm_protocol import PROTOCOL, read_frame, write_frame, split_images
except ImportError:  # uploaded next to this file on the server
    from vlm_protocol import PROTOCOL, read_frame, write_frame, split_images

HERE = Path(__file__).resolve().parent


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(8 << 20), b''):
            h.update(block)
    return h.hexdigest()


def verify_model_files(directory, expected, cache_path):
    """Return {name: REUSED|REHASHED}; raise on any missing or different file."""
    try:
        cache = json.loads(Path(cache_path).read_text())
    except (OSError, ValueError):
        cache = {}
    states, mismatched = {}, []
    for name, digest in sorted(expected.items()):
        path = Path(directory) / name
        if not path.is_file():
            mismatched.append(f'{name}: MISSING')
            continue
        st = path.stat()
        key = str(path.resolve())
        stamp = [st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns]
        entry = cache.get(key)
        if entry and entry.get('stamp') == stamp:
            actual, states[name] = entry['sha256'], 'REUSED'
        else:
            actual, states[name] = file_sha256(path), 'REHASHED'
            cache[key] = dict(stamp=stamp, sha256=actual)
        if actual != digest:
            mismatched.append(f'{name}: SHA256_MISMATCH')
    try:
        Path(cache_path).write_text(json.dumps(cache))
    except OSError:
        pass
    if mismatched:
        raise ValueError('MODEL_FILES_DIFFER: ' + '; '.join(mismatched))
    return states


class QwenCore:
    """Qwen VL models through transformers (AutoModelForImageTextToText), bf16 on one CUDA device.

    Used for Qwen3.5-9B (user choice 2026-11-28) and works for Qwen2.5-VL/Qwen3-VL. Thinking mode is
    switched off in the chat template (real-time replies); a stray <think> block is removed.
    """

    def __init__(self, directory, device, max_pixels):
        import torch
        import transformers
        from transformers import AutoModelForImageTextToText, AutoProcessor
        self.torch = torch
        self.model = AutoModelForImageTextToText.from_pretrained(directory, dtype=torch.bfloat16, device_map=device).eval()
        self.processor = AutoProcessor.from_pretrained(directory)
        self.image_size = dict(shortest_edge=64 * 32 * 32, longest_edge=int(max_pixels))
        self.info = dict(torch=torch.__version__, transformers=transformers.__version__,
                         model_type=getattr(self.model.config, 'model_type', None),
                         gpu=torch.cuda.get_device_name(torch.device(device)) if torch.cuda.is_available() else None)

    def warm_up(self):
        """One small generation so the Triton kernels (linear attention) compile during connect, not on the first request."""
        from PIL import Image
        buffer = io.BytesIO()
        Image.new('RGB', (640, 480), (128, 128, 128)).save(buffer, format='JPEG')
        self.generate([dict(role='user', content=[dict(type='image'), dict(type='text', text='颜色？')])], [buffer.getvalue()], 8)

    def generate(self, messages, images, max_new_tokens, budget_s=None):
        from PIL import Image
        pil = [Image.open(io.BytesIO(data)).convert('RGB') for data in images]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True, enable_thinking=False)
        kwargs = dict(images=pil, images_kwargs=dict(size=self.image_size)) if pil else {}
        inputs = self.processor(text=[text], return_tensors='pt', **kwargs).to(self.model.device)
        stop = Budget(budget_s)
        with self.torch.inference_mode():
            out = self.model.generate(**inputs, max_new_tokens=int(max_new_tokens), do_sample=False, stopping_criteria=[stop])
        new = out[:, inputs['input_ids'].shape[1]:]
        reply = strip_thinking(self.processor.batch_decode(new, skip_special_tokens=True)[0])
        return reply, dict(input_tokens=int(inputs['input_ids'].shape[1]), output_tokens=int(new.shape[1]), budget_exceeded=stop.exceeded)


class Budget:
    """Stopping criterion: the client gives up after its request timeout, so the GPU stops too."""

    def __init__(self, budget_s):
        self.deadline = None if not budget_s else time.monotonic() + float(budget_s)
        self.exceeded = False

    def __call__(self, input_ids, scores, **kwargs):
        if self.deadline is not None and time.monotonic() > self.deadline:
            self.exceeded = True
            return True
        return False


def strip_thinking(text):
    if '</think>' in text:
        text = text.split('</think>', 1)[1]
    return text.replace('<think>', '').strip()


def valid_messages(messages, image_count):
    if not isinstance(messages, list) or not messages:
        return False
    images = 0
    for m in messages:
        if not isinstance(m, dict) or m.get('role') not in ('system', 'user', 'assistant') or not isinstance(m.get('content'), list):
            return False
        for part in m['content']:
            if not isinstance(part, dict) or part.get('type') not in ('text', 'image'):
                return False
            if part['type'] == 'text' and not isinstance(part.get('text'), str):
                return False
            images += part['type'] == 'image'
    return images == image_count


def serve(stdin, stdout, model_root, core_factory=QwenCore):
    core, info = None, {}
    code = {name: file_sha256(HERE / name) for name in ('vlm_server.py', 'vlm_protocol.py')}
    while True:
        frame = read_frame(stdin)
        if frame is None:
            return 0
        header, payload = frame
        reply = dict(id=header.get('id'), op=header.get('op'))
        try:
            op = header.get('op')
            if op == 'init':
                if header.get('protocol') != PROTOCOL:
                    raise ValueError('PROTOCOL_MISMATCH')
                model = header['model']
                rel = Path(model['relative_directory'])
                if rel.is_absolute() or '..' in rel.parts:
                    raise ValueError('MODEL_DIRECTORY_OUTSIDE_ROOT')
                directory = Path(model_root) / rel
                started = time.monotonic()
                states = verify_model_files(directory, model['files'], Path(model_root) / '.eco65_vlm_hash_cache.json')
                verify_s = time.monotonic() - started
                options = header.get('options') or {}
                if core is None:
                    core = core_factory(str(directory), options.get('device', 'cuda:0'), options.get('max_pixels', 640 * 480))
                    info = dict(getattr(core, 'info', {}) or {})
                    if hasattr(core, 'warm_up'):
                        warm = time.monotonic()
                        core.warm_up()
                        info['warm_up_s'] = time.monotonic() - warm
                reply.update(status='READY', server_sha256=code['vlm_server.py'], protocol_sha256=code['vlm_protocol.py'],
                             model_files_verified=True, file_hash_states=states, verify_s=verify_s,
                             load_s=time.monotonic() - started - verify_s, **info)
            elif op == 'ping':
                reply.update(status='OK')
            elif op == 'generate':
                if core is None:
                    raise ValueError('NOT_INITIALIZED')
                images = split_images(payload, header.get('images') or [])
                if not valid_messages(header.get('messages'), len(images)):
                    raise ValueError('INVALID_MESSAGES')
                started = time.monotonic()
                text, tokens = core.generate(header['messages'], images, header.get('max_new_tokens', 256), header.get('budget_s'))
                exceeded = tokens.pop('budget_exceeded', False)
                reply.update(status='BUDGET_EXCEEDED' if exceeded else 'OK', text=text, inference_s=time.monotonic() - started, **tokens)
            else:
                raise ValueError('UNKNOWN_OP')
        except Exception as error:
            traceback.print_exc(file=sys.stderr)
            reply.update(status='ERROR', error=f'{type(error).__name__}: {error}'[:600])
        write_frame(stdout, reply)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model-root', required=True)
    args = p.parse_args()
    stdout = sys.stdout.buffer
    sys.stdout = sys.stderr  # library prints must never corrupt the frame stream
    return serve(sys.stdin.buffer, stdout, args.model_root)


if __name__ == '__main__':
    sys.exit(main())
