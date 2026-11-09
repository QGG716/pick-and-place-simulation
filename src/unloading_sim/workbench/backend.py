"""Serialized observation workers, bounded GUI handoff and explicit lifecycle."""
from __future__ import annotations

from collections import deque
from queue import Queue, Empty, Full
import threading
from time import monotonic

from .policy import ReadOnlyViolation, DisabledExecution, ObservationConfig, load_config


class DeviceWorker:
    def __init__(self, name, device, interval=.1):
        self.name, self.device, self.interval = name, device, interval
        self.commands = Queue(maxsize=16)
        self.events = Queue(maxsize=32)
        self.stop = threading.Event()
        self._pending = 0
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
                        self.emit("sample", self.device.read())
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
        self.joint_history = deque(maxlen=128)
        self.replay = None
        self.replay_active = False
        self.manual_stationary = False
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
        if self.replay_active:
            raise ReadOnlyViolation("Stop replay before connecting live devices")
        self.manual_stationary = False
        self.workers[device].command(action, *args)

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
                        self.manual_stationary = False
        return changes

    def joint_for_frame(self, frame):
        candidates = [q for q in self.joint_history if q.stamp.source == frame.stamp.source
                      and q.stamp.host_session == frame.stamp.host_session]
        return min(candidates, key=lambda q: abs(q.stamp.received_monotonic_ns-frame.stamp.received_monotonic_ns)) if candidates else None

    def load_replay(self, directory):
        if any(w.device.connected or w.busy for w in self.workers.values()):
            raise RuntimeError("Disconnect live devices before loading replay")
        from .replay import Replay
        self.replay = Replay(directory)
        # Consume completed disconnect events before entering a different source domain.
        self.poll()
        self.joint_history.clear()
        self.replay_active = True
        self.manual_stationary = False
        self.log("Replay loaded: " + self.replay.manifest["provenance"])

    def step_replay(self):
        if not self.replay_active:
            raise RuntimeError("No active replay")
        self.frame, self.joint, self.pressure, self.detections = self.replay.step()
        self.joint_history.append(self.joint)
        self.manual_stationary = False
        self.log("Replay observation " + self.frame.observation_id + " (not live hardware)")

    def stop_replay(self):
        self.replay_active = False
        self.manual_stationary = False
        self.frame = self.joint = self.pressure = self.detections = None
        self.joint_history.clear()
        self.log("Replay stopped")

    def close(self):
        self.manual_stationary = False
        self.replay_active = False
        results = [w.close() for w in self.workers.values()]
        self.log("Observation workers closed" if all(results) else "Device cleanup pending; no control command sent")
