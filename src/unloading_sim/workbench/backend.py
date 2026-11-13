"""Serialized observation workers, bounded GUI handoff and explicit lifecycle."""
from __future__ import annotations

from collections import deque
from queue import Queue, Empty, Full
import threading
from dataclasses import dataclass
from time import monotonic, monotonic_ns

from .policy import ReadOnlyViolation, DisabledExecution, ObservationConfig, load_config
from .contracts import Source, compatible_observation_stamps


class DeviceRequestRejected(RuntimeError):
    """Invalid user request; the existing connection remains healthy."""


@dataclass(frozen=True)
class WorkerSample:
    generation: int
    value: object


class SampleHistory(deque):
    """Bounded history appended by the device pump and read by task threads.

    Iteration returns a snapshot taken under the same lock as every mutation,
    so a concurrent append can no longer raise "deque mutated during iteration".
    """
    def __init__(self, maxlen):
        super().__init__(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, value):
        with self._lock:
            super().append(value)

    def clear(self):
        with self._lock:
            super().clear()

    def __iter__(self):
        with self._lock:
            items = list(deque.__iter__(self))
        return iter(items)


class DeviceWorker:
    def __init__(self, name, device, interval=.1):
        self.name, self.device, self.interval = name, device, interval
        self.commands = Queue(maxsize=16)
        self.events = Queue(maxsize=32)
        self.stop = threading.Event()
        self._pending = 0
        self.generation = 0
        self._lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, name="observe-"+name, daemon=True)
        self.thread.start()

    def command(self, action, *args):
        allowed = {"connect", "disconnect"} | ({"start", "stop"} if self.name == "camera" else set())
        if action not in allowed:
            raise ReadOnlyViolation(f"Unavailable stage-one device action: {action}")
        if self.stop.is_set():
            raise RuntimeError("Observation worker is closed")
        with self._lock:
            self.generation += 1
            self._pending += 1
            try:
                self.commands.put_nowait((action, args))
            except Full:
                self._pending -= 1
                raise

    @property
    def busy(self):
        with self._lock:
            return self._pending > 0

    def emit(self, kind, value):
        if kind == 'sample' and not isinstance(value, WorkerSample):
            value = WorkerSample(self.generation, value)
        try:
            self.events.put_nowait((kind, value))
        except Full:
            # Prefer the newest sample; receipt timestamps were already recorded.
            try:
                self.events.get_nowait()
            except Empty:
                pass
            self.events.put_nowait((kind, value))

    def _run(self):
        try:
            while not self.stop.is_set():
                started = monotonic()
                try:
                    action, args = self.commands.get(timeout=.02)
                except Empty:
                    action = None
                try:
                    if action:
                        # Fixed method map, not user-selected getattr / arbitrary SDK dispatch.
                        methods = {"connect": self.device.connect, "disconnect": self.device.disconnect}
                        if self.name == "camera":
                            methods.update(start=self.device.start, stop=self.device.stop)
                        value = methods[action](*args)
                        self.emit("state", {"action": action, "value": value})
                    active = self.device.connected and (self.name != "camera" or self.device.streaming)
                    if active and not self.stop.is_set():
                        generation = self.generation
                        sample = self.device.read()
                        if generation == self.generation:
                            self.emit("sample", WorkerSample(generation, sample))
                except DeviceRequestRejected as exc:
                    # A request error is not evidence of a broken transport.
                    self.emit("error", f"{type(exc).__name__}: {exc}")
                except Exception as exc:
                    try:
                        self.device.disconnect()
                    except Exception:
                        pass
                    self.emit("error", f"{type(exc).__name__}: {exc}")
                finally:
                    if action:
                        with self._lock:
                            self._pending -= 1
                self.stop.wait(max(0, self.interval-(monotonic()-started)))
        finally:
            try:
                self.device.disconnect()
            except Exception as exc:
                self.emit("error", f"Disconnect failed: {type(exc).__name__}")

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.0)
        return not self.thread.is_alive()


class WorkbenchBackend:
    def __init__(self, camera, robot, vacuum):
        self.config = ObservationConfig()
        self.execution = DisabledExecution()
        self.workers = {"camera": DeviceWorker("camera", camera, .01),
                        "robot": DeviceWorker("robot", robot, .25),
                        "vacuum": DeviceWorker("vacuum", vacuum, .01)}
        self.logs = deque(maxlen=300)
        self.frame = self.joint = self.pressure = self.detections = None
        self.joint_history = SampleHistory(maxlen=128)
        self.replay = None
        self.replay_active = False
        self.replay_exhausted = False
        self.observation_generation = 0
        self.manual_stationary = False
        self.sample_cutoff = {name: monotonic_ns() for name in self.workers}
        self.log("READ_ONLY_STAGE_1; all devices disconnected; physical execution unavailable")

    def log(self, message):
        from datetime import datetime
        self.logs.append(datetime.now().strftime("%H:%M:%S") + "  " + str(message))

    def import_config(self, path):
        self.config, ignored = load_config(path)
        self.log("Configuration loaded; ignored fields: " + ", ".join(ignored))
        return ignored

    def command(self, device, action, *args):
        if device not in self.workers:
            raise ReadOnlyViolation("Unknown observation device")
        if self.replay is not None:
            raise ReadOnlyViolation("Stop replay before connecting live devices")
        self.invalidate_device(device)
        self.workers[device].command(action, *args)

    def invalidate_device(self, device):
        self.observation_generation += 1
        self.sample_cutoff[device] = monotonic_ns()
        self.manual_stationary = False
        self.detections = None
        if device == 'camera': self.frame = None
        if device == 'robot':
            self.joint = None
            self.joint_history.clear()
        if device == 'vacuum': self.pressure = None

    def poll(self):
        changes = []
        for name, worker in self.workers.items():
            while True:
                try:
                    kind, value = worker.events.get_nowait()
                except Empty:
                    break
                changes.append((name, kind, value))
                if kind == "sample":
                    if isinstance(value, WorkerSample):
                        if value.generation != worker.generation:continue
                        value = value.value
                    if self.replay is not None or not worker.device.connected:
                        continue
                    if name in {'camera', 'robot'} and (value is None or
                            value.stamp.received_monotonic_ns < self.sample_cutoff[name]):
                        continue
                    if name == 'camera' and not worker.device.streaming:
                        continue
                    if name == "camera": self.frame = value
                    elif name == "robot":
                        self.joint = value
                        self.joint_history.append(value)
                    elif name == "vacuum":
                        for event, sample in value:
                            if event == "alarm":
                                self.log("Vacuum alarm (display only): " + sample["name"])
                else:
                    self.log(name + " " + kind + ": " + str(value))
                    if kind == "error" or value.get("action") in {"disconnect", "stop"}:
                        if not (kind == 'error' and str(value).startswith('DeviceRequestRejected:')):
                            self.invalidate_device(name)
        return changes

    def joint_for_frame(self, frame, max_skew_s=.1):
        candidates = [q for q in self.joint_history if compatible_observation_stamps(frame.stamp, q.stamp)
                      and abs(q.stamp.received_monotonic_ns-frame.stamp.received_monotonic_ns) <= max_skew_s*1e9]
        return min(candidates, key=lambda q: abs(q.stamp.received_monotonic_ns-frame.stamp.received_monotonic_ns)) if candidates else None

    def sample_active(self, stamp, device):
        if stamp.source == Source.REPLAY:
            return self.replay_active
        if stamp.source == Source.LIVE:
            observed = self.workers[device].device
            return observed.connected and (device != 'camera' or observed.streaming)
        # No live simulator producer is registered in this workbench.
        return False

    def sample_state(self, stamp, device):
        if stamp.source == Source.REPLAY and self.replay_exhausted:
            return "HISTORICAL_END_OF_REPLAY"
        return stamp.state(self.sample_active(stamp, device), self.config.max_age_s)

    def _clear_observations(self):
        self.observation_generation += 1
        self.replay = None
        self.replay_active = self.replay_exhausted = self.manual_stationary = False
        self.frame = self.joint = self.pressure = self.detections = None
        self.joint_history.clear()

    def _publish_replay(self, observation):
        self.frame, self.joint, self.pressure, self.detections = observation
        self.joint_history.append(self.joint)
        self.manual_stationary = False
        self.log("Replay observation " + self.frame.observation_id + " (not live hardware)")

    def load_replay(self, directory):
        if any(w.device.connected or w.busy for w in self.workers.values()):
            raise RuntimeError("Disconnect live devices before loading replay")
        from .replay import Replay
        # Consume completed disconnect events before entering a different source domain.
        self.poll()
        self._clear_observations()
        try:
            candidate = Replay(directory)
            first = candidate.step()  # Validate every first-record component before publication.
        except Exception as exc:
            self.log(f"Replay load failed: {type(exc).__name__}: {exc}")
            raise
        self.replay = candidate
        self._publish_replay(first)
        self.replay_active = True
        self.log("Replay loaded: " + self.replay.manifest["provenance"])

    def step_replay(self):
        if self.replay is None:
            raise RuntimeError("No active replay")
        if self.replay_exhausted:
            return False
        try:
            observation = self.replay.step()
        except StopIteration:
            self.replay_active = False
            self.replay_exhausted = True
            self.manual_stationary = False
            self.joint_history.clear()
            self.observation_generation += 1
            self.log("Replay ended; retained last observation is HISTORICAL_END_OF_REPLAY")
            return False
        except Exception as exc:
            self._clear_observations()
            self.log(f"Replay read failed: {type(exc).__name__}: {exc}")
            raise
        self._publish_replay(observation)
        return True

    def stop_replay(self):
        self._clear_observations()
        self.log("Replay stopped")

    def close(self):
        self.manual_stationary = False
        self.replay_active = False
        results = [w.close() for w in self.workers.values()]
        self.log("Observation workers closed" if all(results) else "Device cleanup pending; no control command sent")
