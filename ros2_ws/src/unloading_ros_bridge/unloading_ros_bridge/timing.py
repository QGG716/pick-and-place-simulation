"""Opt-in, bounded callback timing; no I/O in measurement callbacks."""
import atexit
from collections import deque
from functools import wraps
import json
import os
from pathlib import Path
from time import perf_counter


class Timing:
    def __init__(self, node):
        directory = os.environ.get('UNLOADING_TIMING_DIRECTORY')
        self.path = Path(directory)/f'{node.get_name()}-{os.getpid()}.json' if directory else None
        self.events = deque(maxlen=16000)
        self.total = 0
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            atexit.register(self.flush)

    def start(self):
        return perf_counter() if self.path else 0.

    def event(self, kind, **fields):
        if self.path:
            self.total += 1
            self.events.append({'kind': kind, 'monotonic': perf_counter(), **fields})

    def finish(self, kind, started, **fields):
        if self.path:
            self.event(kind, duration=perf_counter()-started, **fields)

    def flush(self):
        if self.path:
            self.path.write_text(json.dumps({'events': list(self.events),
                'dropped': self.total-len(self.events)}, indent=2), encoding='utf-8')


def timed_callback(kind):
    def decorate(method):
        @wraps(method)
        def call(self, *args, **kwargs):
            timer = self.timing
            if not timer.path:
                return method(self, *args, **kwargs)
            started = timer.start()
            fields = {'ros_time': self.get_clock().now().nanoseconds/1e9}
            if kind in ('joints', 'mechanism'):
                stamp = args[0].header.stamp if kind == 'joints' else args[0].observed_time
                fields['sample_time'] = stamp.sec+stamp.nanosec/1e9
                fields['entry_age'] = fields['ros_time']-fields['sample_time']
            timer.event(kind+'_entry', **fields)
            try:
                return method(self, *args, **kwargs)
            finally:
                timer.finish(kind, started, robot_sample=getattr(self, 'last_joint_stamp', None),
                             mechanism_sample=getattr(self, 'mechanism_stamp', None))
        return call
    return decorate
