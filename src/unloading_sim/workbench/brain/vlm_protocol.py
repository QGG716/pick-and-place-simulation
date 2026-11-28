"""Framed messages between the field PC and the remote VLM server (stdin/stdout over SSH).

Frame: b'E65V' | uint32 header length | uint64 payload length | JSON | payload.
Uploaded next to vlm_server.py; nothing is ever pickled in either direction.
"""
import json
import struct

MAGIC = b'E65V'
PROTOCOL = 1
HEADER = struct.Struct('>4sIQ')
MAX_HEADER = 4 << 20
MAX_PAYLOAD = 32 << 20


def read_frame(stream):
    raw = stream.read(HEADER.size)
    if not raw:
        return None
    if len(raw) != HEADER.size:
        raise EOFError('TRUNCATED_FRAME_HEADER')
    magic, header_len, payload_len = HEADER.unpack(raw)
    if magic != MAGIC or header_len > MAX_HEADER or payload_len > MAX_PAYLOAD:
        raise ValueError('INVALID_FRAME')
    header = stream.read(header_len)
    payload = stream.read(payload_len)
    if len(header) != header_len or len(payload) != payload_len:
        raise EOFError('TRUNCATED_FRAME_BODY')
    return json.loads(header.decode('utf-8')), payload


def write_frame(stream, header, payload=b''):
    data = json.dumps(header, allow_nan=False, ensure_ascii=False).encode('utf-8')
    stream.write(HEADER.pack(MAGIC, len(data), len(payload)) + data + payload)
    stream.flush()


def split_images(payload, images):
    """The payload carries the images back to back; each listed with size and SHA-256."""
    import hashlib
    out, offset = [], 0
    for item in images:
        size = int(item['size'])
        if size <= 0 or offset + size > len(payload):
            raise ValueError('IMAGE_PAYLOAD_SIZE_MISMATCH')
        data = payload[offset:offset + size]
        if hashlib.sha256(data).hexdigest() != item['sha256']:
            raise ValueError('IMAGE_PAYLOAD_SHA256_MISMATCH')
        out.append(data)
        offset += size
    if offset != len(payload):
        raise ValueError('IMAGE_PAYLOAD_SIZE_MISMATCH')
    return out
