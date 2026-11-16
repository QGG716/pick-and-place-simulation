"""Asynchronous evidence writer (realtime step 4, ``async_evidence: true``).

Requests are still persisted synchronously before dispatch by their callers.
Replies, segment evidence, observation arrays, the site scene and the
complete plan are serialised and written by one background thread, in
submission order. ``flush`` waits until everything submitted so far is on
disk and raises when any write failed; PLAN_READY and execution both require
a successful flush (AGENTS 2026-11-16 3e). Values handed to the writer must
not be mutated afterwards; every caller hands over finished records.
"""
import json
from pathlib import Path
import queue
import threading

import numpy as np

from .. import timing


class EvidenceWriteFailed(RuntimeError):
    pass


class EvidenceWriter:
    def __init__(self, name='evidence-writer'):
        self._queue = queue.Queue(); self.error = None; self.written = 0; self._closed = False
        self._thread = threading.Thread(target=self._run, daemon=True, name=name); self._thread.start()

    def _run(self):
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                kind, path, value, name, options = item
                if kind == 'flush':
                    value.set(); continue
                if self.error is not None:
                    continue  # after a failure nothing more is written; flush reports it
                try:
                    if kind == 'json':
                        timing.write_json(path, value, name, **options)
                    elif kind == 'npz':
                        with timing.span('disk.write', file=name):
                            np.savez_compressed(path, **value)
                    self.written += 1
                except Exception as error:
                    self.error = f'{name}: {type(error).__name__}: {error}'
            finally:
                self._queue.task_done()

    def _put(self, item):
        if self._closed:
            raise EvidenceWriteFailed('EVIDENCE_WRITER_CLOSED')
        self._queue.put(item)

    def write_json(self, path, value, name, **options):
        self._put(('json', Path(path), value, name, options))

    def write_npz(self, path, arrays, name):
        self._put(('npz', Path(path), arrays, name, {}))

    def flush(self, timeout=None):
        """Block until every earlier submission is written; raise on any failure."""
        done = threading.Event()
        self._put(('flush', None, done, 'flush', {}))
        if not done.wait(timeout):
            raise EvidenceWriteFailed('EVIDENCE_FLUSH_TIMEOUT')
        if self.error is not None:
            raise EvidenceWriteFailed('EVIDENCE_WRITE_FAILED: '+self.error)

    @property
    def alive(self):
        return self._thread.is_alive() and self.error is None

    def close(self, timeout=5.):
        if self._closed:
            return
        self._closed = True; self._queue.put(None); self._thread.join(timeout)


def write_json(writer, path, value, name, **options):
    """Through ``writer`` when given, else synchronously (the original behaviour)."""
    if writer is None:
        return timing.write_json(path, value, name, **options)
    writer.write_json(path, value, name, **options)
