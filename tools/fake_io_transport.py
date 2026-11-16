"""In-memory fakes for the scheduling measurement. Never opens hardware.

``FakeTransport`` replaces the vendor SDK transport inside the real spawned
execution process: feedback is generated at 200 Hz from the last commanded
joint vector (the ideal follower), stop requests settle at once, and the
controller tool payload reads back the configured value. ``FakeVacuum``
replaces the serial owner: valve commands only change an in-memory pressure
whose heartbeats arrive every 0.5 s (field heartbeat cadence is not known to
this tool; see the report). Only ``FAKE_DEVICE`` results can follow.
"""
import json
import multiprocessing as mp
import os
from pathlib import Path
import threading
import time

_CONFIG = {}


def configure(folder, q_start, execution):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    _CONFIG.update(folder=str(folder), q_start=list(q_start),
                   payload=float(execution.get('controller_payload_kg') or execution.get('tool_mass_kg', 1.5)))
    (folder/'fake-config.json').write_text(json.dumps(_CONFIG))


class FakeTransport:
    """Duck-typed RealmanTransport for the spawned execution process."""
    stop_api = 'FAKE_TRANSPORT_NO_HARDWARE'

    def __init__(self, mapping, config):
        cfg = json.loads(Path(os.environ['ECO65_FAKE_IO_DIR'], 'fake-config.json').read_text())
        self.folder = Path(cfg['folder']); self.q = list(cfg['q_start']); self.payload = cfg['payload']
        self.connected = False; self.capabilities = {}; self.sender = self; self.sequence = 0
        self._stop = None; self._lock = threading.Lock(); self.latest = None; self.halt = threading.Event()
        self.first_send = {}; self.task = 0; self.sends = 0

    def connect(self, host, port):
        self.connected = True; self.halt.clear()
        self.capabilities = dict(transport='FAKE_IN_MEMORY_NO_HARDWARE', host=host, port=port,
                                 **{name: dict(value=value) for name, value in (
                                     ('rm_get_joint_min_pos', [-360.]*6), ('rm_get_joint_max_pos', [360.]*6),
                                     ('rm_get_joint_max_speed', [180.]*6), ('rm_get_joint_max_acc', [600.]*6))})
        threading.Thread(target=self._feedback, daemon=True, name='fake-udp').start()
        deadline = time.monotonic()+1
        while self.latest is None and time.monotonic() < deadline: time.sleep(.005)
        return self.capabilities

    def _feedback(self):
        previous = None
        while not self.halt.is_set():
            with self._lock: q = list(self.q)
            now = time.monotonic()
            velocity = [0.]*6 if previous is None else [(a-b)/max(1e-3, now-previous[1]) for a, b in zip(q, previous[0])]
            self.sequence += 1
            self.latest = dict(source='LIVE', sequence=self.sequence, received_s=now, q=q, velocity=velocity,
                               enabled=True, alarm=False, velocity_source='FAKE_FINITE_DIFFERENCE')
            previous = (q, now); time.sleep(.005)

    def rm_get_current_tool_frame(self):
        time.sleep(.002)  # stands in for a bounded TCP query; real latency is not measured here
        return 0, dict(name='FakeLoad', payload=self.payload)

    def arm(self): self.task += 1; self.sends = 0

    def check_limits(self, prepared): pass

    def send(self, q):
        if self.sends == 0:
            line = json.dumps(dict(task=self.task, monotonic_s=time.monotonic()))
            with open(self.folder/'first-sends.jsonl', 'a') as f: f.write(line+'\n')
        self.sends += 1
        with self._lock: self.q = list(q)
        return 0

    def request_stop(self): self._stop = 0

    def stop_result(self): return self._stop

    def disconnect(self): self.connected = False; self.halt.set()


def _child(pipe, mapping, config, stop_event, cancel_epoch, folder):
    os.environ['ECO65_FAKE_IO_DIR'] = folder
    os.environ['ECO65_TIMING'] = '1'
    from unloading_sim.workbench import timing  # noqa: F401  (enables child spans)
    from unloading_sim.workbench.controlled import service
    service.RealmanTransport = FakeTransport
    service.worker(pipe, mapping, config, stop_event, cancel_epoch)


class SpawnedService:
    """The real ExecutionService, with its child started through ``_child``."""
    def __new__(cls, config, mapping, authority, binding):
        from unloading_sim.workbench.controlled.service import ExecutionService
        service = ExecutionService(mapping, config, authority, binding)
        folder = _CONFIG['folder']
        original = service._call

        def call(op, arg=None):
            if service.process is None and op == 'connect':
                ctx = mp.get_context('spawn'); service.pipe, child = ctx.Pipe()
                service.stop_event = ctx.Event(); service.cancel_epoch = ctx.Value('Q', 0)
                service.process = ctx.Process(target=_child, args=(child, mapping, config, service.stop_event,
                                                                   service.cancel_epoch, folder), daemon=False)
                service.process.start(); child.close()
            return original(op, arg)
        service._call = call
        return service


def wait_feedback(service, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        sample = service.snapshot()
        if sample and time.monotonic()-sample['received_s'] < .05: return sample
        time.sleep(.01)
    raise RuntimeError('FAKE_FEEDBACK_NOT_LIVE')


def first_sends(folder):
    path = Path(folder)/'first-sends.jsonl'
    return [json.loads(x) for x in path.read_text().splitlines()] if path.is_file() else []


class FakeVacuum:
    """In-memory common valve; pressure heartbeats every ``period_s``."""
    def __init__(self, config, folder, period_s=.5):
        self.config = config; self.period = period_s; self.holding = False; self.monitor_holding = False
        self.task_id = None; self.inhibited = True; self.target = 0.; self.kpa = 0.; self.received = time.monotonic()
        self.commands = []; self.halt = threading.Event(); self.heartbeats = []; self.folder = Path(folder)
        threading.Thread(target=self._beat, daemon=True, name='fake-vacuum').start()

    def _beat(self):
        while not self.halt.wait(self.period):
            self.kpa = self.target; self.received = time.monotonic(); self.heartbeats.append((self.received, self.kpa))

    def dump(self):
        (self.folder/'vacuum.json').write_text(json.dumps(dict(
            heartbeat_period_s=self.period, heartbeats=self.heartbeats, commands=self.commands,
            basis='FAKE_IN_MEMORY_HEARTBEAT_PERIOD_NOT_FIELD_MEASURED')))

    def pressure(self):
        return dict(kpa=self.kpa, received_s=self.received, source='LIVE', integrity='CHECKED_BINARY', channel='gripper')

    def claim(self, task_id, record, on_fault):
        if self.task_id: raise RuntimeError('VACUUM_TASK_ACTIVE')
        self.task_id = task_id; self.inhibited = False

    def establish(self, task_id):
        assert not self.inhibited; self.holding = True; self.target = -70.; self.commands.append(('establish', time.monotonic()))

    def release(self, task_id):
        assert not self.inhibited; self.monitor_holding = False; self.target = 0.; self.commands.append(('release', time.monotonic()))

    def inhibit(self): self.inhibited = True

    def unclaim(self, released=False):
        if released: self.holding = False
        if not self.holding: self.inhibited = True; self.task_id = None
