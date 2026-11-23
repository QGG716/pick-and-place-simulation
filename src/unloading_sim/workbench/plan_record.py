"""binary_plan_record (2-second planning, 2026-11-22): the complete-plan record with binary float tables.

The record is the same content as ``json.dumps(plan)``, written as a JSON
skeleton plus content-addressed float64 ``.npy`` files next to it
(``complete-plan-arrays/<sha256>.npy``):

* a list of at least ``TABLE_MIN`` dicts with one key set whose columns are
  all floats or all equal-length float lists (a trajectory table such as
  ``points``) becomes ``{"$table": {"rows": n, "keys": [...], "columns":
  {key: array reference}}}``;
* a list of at least ``ARRAY_MIN`` floats or of equal-length float lists
  (``collision_path``) becomes ``{"$array": array reference}``;
* a dict or list object that already appeared earlier in the record (the
  binding shared by every package) becomes ``{"$same_as": JSON pointer}``.

An array reference names the file, its SHA-256, shape and dtype. Every file
is written before the skeleton, so a skeleton on disk never names a missing
file; ``load`` restores exactly the original values (float64 bit patterns,
key order). Only Python floats enter an array, so ints and bools stay JSON.
"""
import hashlib
import io
import json
from pathlib import Path

import numpy as np

from .plan_digest import _column

TABLE_MIN = 16
ARRAY_MIN = 256
ARRAYS = 'complete-plan-arrays'


def _escape(key):
    return str(key).replace('~', '~0').replace('/', '~1')


class _Store:
    def __init__(self, folder):
        self.folder = Path(folder)/ARRAYS; self.folder.mkdir(parents=True, exist_ok=True)
        self.files = 0; self.bytes = 0

    def __call__(self, array):
        buffer = io.BytesIO(); np.save(buffer, np.ascontiguousarray(array, dtype='<f8'), allow_pickle=False)
        data = buffer.getvalue(); sha = hashlib.sha256(data).hexdigest()
        target = self.folder/f'{sha}.npy'
        if not target.is_file():
            partial = target.with_suffix('.npy.partial'); partial.write_bytes(data); partial.replace(target)
            self.files += 1; self.bytes += len(data)
        return {'file': f'{ARRAYS}/{sha}.npy', 'sha256': sha, 'shape': list(array.shape), 'dtype': '<f8'}


def _table(rows, store):
    keys = list(rows[0].keys())
    if not keys:
        return None
    key_set = rows[0].keys()
    for row in rows:
        if type(row) is not dict or row.keys() != key_set:
            return None
    columns = {}
    for key in keys:
        column = _column([row[key] for row in rows])
        if column is None:
            return None
        columns[key] = column
    if any(not np.isfinite(c).all() for c in columns.values()):
        raise ValueError('Out of range float values are not JSON compliant')
    return {'$table': {'rows': len(rows), 'keys': keys, 'columns': {k: store(c) for k, c in columns.items()}}}


def _skeleton(value, store, memo, pointer):
    kind = type(value)
    if kind is dict or kind is list:
        first = memo.get(id(value))
        if first is not None and first[0] is value:
            return {'$same_as': first[1]}
        memo[id(value)] = (value, pointer)
    if kind is dict:
        for key in ('$table', '$array', '$same_as'):
            if key in value:
                raise ValueError('BINARY_PLAN_RECORD_RESERVED_KEY')
        return {k: _skeleton(v, store, memo, f'{pointer}/{_escape(k)}') for k, v in value.items()}
    if kind is list:
        if len(value) >= TABLE_MIN and type(value[0]) is dict:
            table = _table(value, store)
            if table is not None:
                return table
        elif len(value) >= ARRAY_MIN and type(value[0]) in (float, list):
            column = _column(value)
            if column is not None and column.size >= ARRAY_MIN:
                if not np.isfinite(column).all():
                    raise ValueError('Out of range float values are not JSON compliant')
                return {'$array': store(column)}
        return [_skeleton(x, store, memo, f'{pointer}/{i}') for i, x in enumerate(value)]
    if kind is tuple:
        return _skeleton(list(value), store, memo, pointer)
    return value


def write(path, plan, **dumps_options):
    """Arrays first, then the skeleton; returns (skeleton bytes, array files written, array bytes)."""
    path = Path(path)
    store = _Store(path.parent)
    skeleton = _skeleton(plan, store, {}, '')
    text = json.dumps(dict(schema='eco65_binary_plan_record_v1', arrays=ARRAYS, plan=skeleton), allow_nan=False, **dumps_options)
    path.write_text(text)
    return len(text), store.files, store.bytes


def load(path):
    """The original plan from a binary plan record (every array checked against its SHA-256)."""
    path = Path(path)
    record = json.loads(path.read_text())
    if record.get('schema') != 'eco65_binary_plan_record_v1':
        raise ValueError('NOT_A_BINARY_PLAN_RECORD')
    root = path.parent

    def array(ref):
        data = (root/ref['file']).read_bytes()
        if hashlib.sha256(data).hexdigest() != ref['sha256']:
            raise ValueError('BINARY_PLAN_RECORD_ARRAY_HASH_MISMATCH')
        return np.load(io.BytesIO(data), allow_pickle=False)

    resolved = {}

    def walk(v, pointer):
        if type(v) is dict:
            if '$same_as' in v:
                return resolved[v['$same_as']]
            if '$table' in v:
                t = v['$table']; columns = {k: array(t['columns'][k]).tolist() for k in t['keys']}
                out = [{k: columns[k][i] for k in t['keys']} for i in range(t['rows'])]
            elif '$array' in v:
                out = array(v['$array']).tolist()
            else:
                out = {}
                resolved[pointer] = out
                for k, x in v.items():
                    out[k] = walk(x, f'{pointer}/{_escape(k)}')
                return out
            resolved[pointer] = out
            return out
        if type(v) is list:
            out = []
            resolved[pointer] = out
            for i, x in enumerate(v):
                out.append(walk(x, f'{pointer}/{i}'))
            return out
        return v
    return walk(record['plan'], '')
