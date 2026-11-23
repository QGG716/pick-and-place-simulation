"""Canonical tree digest of plan packages (AGENTS 2026-11-16 3g, 2-second planning 2026-11-22).

``tree_digest`` is SHA-256 over a canonical binary form of a JSON-like value,
composed as a tree, so that the float tables of a trajectory are hashed as
float64 bytes instead of being printed as decimal text:

* dict: b'D', then for each key in sorted order its UTF-8 length and bytes
  and the 32-byte digest of its value;
* list of at least ``TABLE_MIN`` dicts with one identical key set whose
  columns are all floats, or all float lists of one length (a trajectory
  table such as ``points``): b'T', the row count, then per key in sorted
  order the key and its column as a float64 little-endian array with shape;
* list of floats (``len >= ARRAY_MIN``) or equal-length lists of floats:
  b'A', shape, float64 little-endian bytes;
* other list: b'L', length, then every element's entry;
* the entry of a dict value or list element is its 32-byte digest for a dict
  or list, and an inline tagged encoding for a scalar: b'f' + float64,
  b'i' + decimal text, b's' + UTF-8 bytes (both length-prefixed), b't'/b'F'
  for true/false, b'n' for null. NaN and infinities are rejected, as by the
  canonical JSON digest (allow_nan=False).

Within one call a dict or list object seen again (the shared binding of every
package) reuses its digest; nothing is cached between calls, so every call
re-reads the whole content. Signer and verifier use this one function. A
float-table column holds Python floats only (``type(x) is float``), so a table
digest never conflates an int with a float. Every value that changes changes
the digest, exactly as with the canonical JSON digest.

A float subclass (``numpy.float64``) is a float here exactly as in
``json.dumps``, so ``tree_digest(v) == tree_digest(json.loads(json.dumps(v)))``
for every value the canonical JSON digest accepts (2026-11-23: a plan field
holding ``numpy.float64`` was rejected before).
"""
import hashlib
from itertools import chain
import json
import struct

import numpy as np

TABLE_MIN = 16
ARRAY_MIN = 2
_LEN = struct.Struct('<Q')
_FLOAT = {float}
_LIST = {list}


def _key(k):
    data = k.encode() if isinstance(k, str) else json.dumps(k).encode()
    return _LEN.pack(len(data))+data


def _is_float_type(kind):
    # bool is not a float subclass; numpy.float64 is (json.dumps prints it as a float).
    return kind is float or issubclass(kind, float)


def _floats(values):
    kinds = set(map(type, values))
    return kinds == _FLOAT or all(map(_is_float_type, kinds))


def _column(values):
    """float64 array of a column (floats, or equal-length float lists), else None."""
    first = values[0]
    if _is_float_type(type(first)):
        return np.asarray(values, dtype='<f8') if _floats(values) else None
    if type(first) is list and first and _is_float_type(type(first[0])):
        if set(map(type, values)) != _LIST or set(map(len, values)) != {len(first)}:
            return None
        if not _floats(chain.from_iterable(values)):
            return None
        return np.asarray(values, dtype='<f8')
    return None


def _array_bytes(tag, array):
    if not np.isfinite(array).all():
        raise ValueError('Out of range float values are not JSON compliant')
    shape = array.shape
    return tag+struct.pack('<B', len(shape))+struct.pack(f'<{len(shape)}Q', *shape)+np.ascontiguousarray(array, dtype='<f8').tobytes()


def _table(rows):
    keys = rows[0].keys()
    if not keys:
        return None
    names = sorted(keys)
    for row in rows:
        if type(row) is not dict or row.keys() != keys:
            return None
    columns = []
    for name in names:
        column = _column([row[name] for row in rows])
        if column is None:
            return None
        columns.append((name, column))
    h = hashlib.sha256(b'T'+_LEN.pack(len(rows)))
    for name, column in columns:
        h.update(_key(name)); h.update(_array_bytes(b'C', column))
    return h.digest()


def _scalar(value):
    kind = type(value)
    if kind is float or (kind is not bool and isinstance(value, float)):
        if value != value or value in (float('inf'), float('-inf')):
            raise ValueError('Out of range float values are not JSON compliant')
        return b'f'+struct.pack('<d', value)
    if kind is str:
        data = value.encode()
        return b's'+_LEN.pack(len(data))+data
    if kind is bool:
        return b't' if value else b'F'
    if kind is int:
        data = str(value).encode()
        return b'i'+_LEN.pack(len(data))+data
    if value is None:
        return b'n'
    raise TypeError(f'Object of type {kind.__name__} is not JSON serializable')


def _entry(value, memo):
    kind = type(value)
    if kind is dict or kind is list or kind is tuple:
        return b'#'+_digest(value, memo)
    return _scalar(value)


def _digest(value, memo):
    kind = type(value)
    if kind is tuple:
        value = list(value); kind = list
    cached = memo.get(id(value))
    if cached is not None and cached[0] is value:
        return cached[1]
    if kind is dict:
        h = hashlib.sha256(b'D')
        for k in sorted(value):
            h.update(_key(k)); h.update(_entry(value[k], memo))
        out = h.digest()
    elif kind is list:
        out = None
        if len(value) >= TABLE_MIN and type(value[0]) is dict:
            out = _table(value)
        elif len(value) >= ARRAY_MIN and (type(value[0]) is list or _is_float_type(type(value[0]))):
            column = _column(value)
            if column is not None:
                out = hashlib.sha256(_array_bytes(b'A', column)).digest()
        if out is None:
            h = hashlib.sha256(b'L'+_LEN.pack(len(value)))
            for x in value:
                h.update(_entry(x, memo))
            out = h.digest()
    else:
        out = hashlib.sha256(b'S'+_scalar(value)).digest()
    memo[id(value)] = (value, out)
    return out


def tree_digest(value, memo=None):
    """Hex SHA-256 of the canonical tree form of ``value`` (see the module docstring).

    ``memo`` (digest_memo, 2026-11-23; signing side only): a dict kept by the
    caller across calls, mapping id(object) to (object, digest). An object
    already in it is not re-read, so a list mutated in place after it was
    memoized keeps its old digest. Only the issuing side may pass one: every
    check (Authority.verify, Cycle.start) re-reads the whole content without
    a memo, so such a change makes the check fail instead of passing.
    """
    return _digest(value, {} if memo is None else memo).hex()
