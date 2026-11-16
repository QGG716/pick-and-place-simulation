"""Binary worker frames, protocol version 2 (docs/native_boundary.md section 0).

A frame carries one JSON header and float64 data blocks:

  magic b'E65B' | version u16 | kind u16 | request_id u64 | header_length u32 | block_count u32
  | header (canonical JSON) | per block: dtype u8 (1=float64) | ndim u8 | shape u32 x ndim | payload (little endian)

``split`` moves every list of Python floats (1-D) and every list of equal-length
lists of Python floats (2-D) with at least MIN_BLOCK_VALUES values into a block,
replacing it with ``{"$block": i}``; blocks are numbered in canonical order
(object keys sorted, depth first). ``merge`` restores exactly the same values
(float64 bit patterns), so a frame carries the same content as a JSON line.

``canonical_digest`` is SHA-256 over the canonical header JSON followed by every
block's dtype, ndim, shape and payload; Authority uses it for packages when
``planning_module`` is NATIVE (AGENTS 2026-11-16 3g: signer and verifier use
this one function).
"""
import hashlib
import json
import struct

import numpy as np

MAGIC = b'E65B'
VERSION = 2
REQUEST, REPLY, EVENT, CANCEL = 1, 2, 3, 4
FLOAT64 = 1
MIN_BLOCK_VALUES = 64
_FIXED = struct.Struct('<4sHHQII')


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _float_vector(value):
    return isinstance(value, list) and len(value) > 0 and all(type(x) is float for x in value)


def _float_matrix(value):
    if not isinstance(value, list) or not value or not isinstance(value[0], list) or not value[0]:
        return False
    width = len(value[0])
    return all(isinstance(r, list) and len(r) == width and all(type(x) is float for x in r) for r in value)


def split(value):
    """(header, blocks) with float arrays replaced by {"$block": i}; blocks are float64 ndarrays."""
    blocks = []

    def walk(v):
        if isinstance(v, dict):
            if '$block' in v:
                raise ValueError('BINARY_FRAME_RESERVED_KEY')
            return {k: walk(v[k]) for k in sorted(v)}
        if isinstance(v, list):
            if _float_vector(v) and len(v) >= MIN_BLOCK_VALUES:
                blocks.append(np.asarray(v, dtype='<f8')); return {'$block': len(blocks)-1}
            if _float_matrix(v) and len(v)*len(v[0]) >= MIN_BLOCK_VALUES:
                blocks.append(np.asarray(v, dtype='<f8')); return {'$block': len(blocks)-1}
            return [walk(x) for x in v]
        return v
    return walk(value), blocks


def merge(header, blocks):
    def walk(v):
        if isinstance(v, dict):
            if set(v) == {'$block'}:
                return blocks[v['$block']].tolist()
            return {k: walk(x) for k, x in v.items()}
        if isinstance(v, list):
            return [walk(x) for x in v]
        return v
    return walk(header)


def _block_bytes(block):
    shape = block.shape
    return struct.pack('<BB', FLOAT64, len(shape))+struct.pack(f'<{len(shape)}I', *shape)+np.ascontiguousarray(block, dtype='<f8').tobytes()


def encode(kind, request_id, value):
    header, blocks = split(value)
    text = canonical_json(header).encode()
    return b''.join([_FIXED.pack(MAGIC, VERSION, kind, int(request_id), len(text), len(blocks)), text]+[_block_bytes(b) for b in blocks])


def read_frame(read_exact, prefix=b''):
    """Decode one frame; ``read_exact(n)`` returns exactly n bytes; ``prefix`` = bytes of it already read."""
    fixed = prefix+read_exact(_FIXED.size-len(prefix))
    magic, version, kind, request_id, header_length, block_count = _FIXED.unpack(fixed)
    if magic != MAGIC or version != VERSION:
        raise ValueError('BINARY_FRAME_INVALID')
    header = json.loads(read_exact(header_length))
    blocks = []
    for _ in range(block_count):
        dtype, ndim = struct.unpack('<BB', read_exact(2))
        if dtype != FLOAT64 or ndim not in (1, 2):
            raise ValueError('BINARY_FRAME_BLOCK_INVALID')
        shape = struct.unpack(f'<{ndim}I', read_exact(4*ndim))
        count = int(np.prod(shape))
        blocks.append(np.frombuffer(read_exact(8*count), dtype='<f8').reshape(shape))
    return kind, request_id, merge(header, blocks)


class Wire:
    """Byte pipes to one worker: JSON lines until the worker declares binary_protocol=2, then frames."""

    def __init__(self, process):
        self.process = process; self.binary = False; self.sent_bytes = 0; self.received_bytes = 0
        self._sequence = 0

    def activate(self, ready):
        self.binary = isinstance(ready, dict) and (ready.get('capabilities') or {}).get('binary_protocol') == VERSION
        return self.binary

    def send(self, payload, kind=REQUEST):
        self._sequence += 1
        data = encode(kind, self._sequence, payload) if self.binary else (json.dumps(payload, allow_nan=False)+'\n').encode()
        self.process.stdin.write(data); self.process.stdin.flush(); self.sent_bytes += len(data)

    def cancel(self, target):
        self.send(dict(op='cancel', target_request_id=target), CANCEL)

    def replies(self):
        """Yields decoded replies (JSON line or frame) until the worker's stdout closes."""
        out = self.process.stdout

        def exact(n):
            data = out.read(n)
            if len(data) != n:
                raise EOFError('BINARY_FRAME_TRUNCATED')
            self.received_bytes += n
            return data
        while True:
            head = out.read(4)
            if not head:
                return
            if head == MAGIC:
                self.received_bytes += 4
                yield read_frame(exact, head)[2]
            else:
                line = head+out.readline(); self.received_bytes += len(line)
                yield json.loads(line)


def canonical_digest(value):
    header, blocks = split(value)
    h = hashlib.sha256(canonical_json(header).encode())
    for b in blocks:
        h.update(_block_bytes(b))
    return h.hexdigest()
