"""Explicitly connected observation adapters; no device control API is reachable."""
from __future__ import annotations

from dataclasses import dataclass, replace
from time import monotonic_ns
import threading
import uuid

import numpy as np

from .contracts import (DeviceTime, Stamp, RGBDFrame, JointSample, finite_vector)
from .coordinates import JointMapping
from .policy import ReadOnlyViolation
from .reference.vacuum_receive import (FrameParser, TextReportParser, parse_heartbeat,
                                     ALARMS, COMMANDS, BAUD)


_sdk_lock = threading.Lock()
_sdk_initialized = False


def realman_sdk():
    """Same lazy process SDK initialization as reference controller_arm, no executor."""
    global _sdk_initialized
    with _sdk_lock:
        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
        sdk = RoboticArm() if _sdk_initialized else RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        _sdk_initialized = True
        return sdk


class RobotObserver:
    ALLOWED = frozenset({"rm_create_robot_arm", "rm_get_current_arm_state", "rm_delete_robot_arm"})

    def __init__(self, mapping: JointMapping, sdk_factory=realman_sdk):
        self.mapping, self._factory = mapping, sdk_factory
        self._sdk = None
        self.connected = False

    def request(self, operation, *args):
        if operation not in self.ALLOWED:
            raise ReadOnlyViolation(f"Robot operation {operation} is not a state query or connection lifecycle")
        if self._sdk is None:
            raise RuntimeError("Robot not connected")
        return getattr(self._sdk, operation)(*args)

    def connect(self, host, port):
        if self._sdk is not None:
            raise RuntimeError("Disconnect the current robot first")
        if not host or not 1 <= int(port) <= 65535:
            raise ValueError("Robot host and port are required")
        self._sdk = self._factory()
        try:
            handle = self.request("rm_create_robot_arm", host, int(port))
            if handle is None or handle.id <= 0:
                raise RuntimeError("Robot connection failed")
            self.connected = True
        except Exception:
            self.disconnect()
            raise

    def read(self):
        if not self.connected:
            raise RuntimeError("Robot not connected")
        try:
            code, state = self.request("rm_get_current_arm_state")
            stamp = Stamp.receive()  # Receipt time, not exposure/controller sample time.
            if code != 0:
                raise RuntimeError(f"Robot state query failed: {code}")
            raw_joints = finite_vector(state["joint"], 6, "controller degrees")
            q = self.mapping.apply(raw_joints, self.mapping.source_names)
            return JointSample(str(uuid.uuid4()), tuple(q), stamp,
                {"joint_deg": list(raw_joints), "arm_error": state.get("arm_err"),
                 "system_error": state.get("sys_err"), "controller_sample_time": "UNKNOWN"},
                self.mapping.mapping_id)
        except Exception:
            self.disconnect()
            raise

    def disconnect(self):
        sdk, self._sdk = self._sdk, None
        self.connected = False
        if sdk is not None:
            sdk.rm_delete_robot_arm()  # Closes transport; no stop, enable or frame setting.


def _device_time(frame):
    try:
        return DeviceTime(float(frame.get_timestamp()), str(frame.get_frame_timestamp_domain()))
    except (AttributeError, RuntimeError):
        return DeviceTime()


@dataclass(frozen=True, order=True)
class StreamProfile:
    width: int
    height: int
    fps: int


class D435iObserver:
    """Adapted reference camera.py profile resolution, alignment and SDK scale.

    Construction is inert. connect enumerates; start alone enables acquisition.
    Frame numbers and device clocks are captured before processing/alignment.
    """
    def __init__(self, rs=None):
        self._rs = rs
        self.connected = False
        self.streaming = False
        self.serial = ""
        self.name = ""
        self.profiles = ()
        self._pipeline = None

    def _config(self, profile):
        rs = self._rs
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, profile.width, profile.height, rs.format.bgr8, profile.fps)
        config.enable_stream(rs.stream.depth, profile.width, profile.height, rs.format.z16, profile.fps)
        return config

    def connect(self, serial=""):
        if self.connected:
            raise RuntimeError("Disconnect camera first")
        if self._rs is None:
            import pyrealsense2 as rs
            self._rs = rs
        rs = self._rs
        devices = list(rs.context().query_devices())
        matches = [d for d in devices if "d435i" in d.get_info(rs.camera_info.name).casefold()
                   and (not serial or d.get_info(rs.camera_info.serial_number) == serial)]
        if len(matches) != 1:
            available=[d.get_info(rs.camera_info.serial_number) for d in devices if 'd435i' in d.get_info(rs.camera_info.name).casefold()]
            raise RuntimeError('未找到指定 D435i' if not available else '多台 D435i 或序列号不匹配；请配置选择：'+', '.join(available))
        d = matches[0]
        self.serial, self.name = d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.name)
        color, depth = set(), set()
        for sensor in d.query_sensors():
            for p in sensor.get_stream_profiles():
                if not p.is_video_stream_profile():
                    continue
                dest = color if p.stream_type() == rs.stream.color and p.format() == rs.format.bgr8 else (
                    depth if p.stream_type() == rs.stream.depth and p.format() == rs.format.z16 else None)
                if dest is not None:
                    v = p.as_video_stream_profile()
                    dest.add(StreamProfile(v.width(), v.height(), p.fps()))
        self._pipeline = rs.pipeline()
        self.profiles = tuple(p for p in sorted(color & depth)
                              if self._config(p).can_resolve(rs.pipeline_wrapper(self._pipeline)))
        if not self.profiles:
            self._pipeline = None
            raise RuntimeError("No resolvable same-size BGR8/Z16 stream pair")
        self.connected = True
        return self.profiles

    def start(self, profile):
        if not self.connected or self.streaming:
            raise RuntimeError("Camera must be connected and stopped")
        if profile not in self.profiles:
            raise ValueError("Selected stream profile was not resolved for this D435i")
        try:
            active = self._pipeline.start(self._config(profile))
            self.streaming = True
            self._scale = float(active.get_device().first_depth_sensor().get_depth_scale())
            if not np.isfinite(self._scale) or self._scale <= 0:
                raise ValueError("Invalid SDK depth scale")
            self._align = self._rs.align(self._rs.stream.color)
            self.profile = profile
        except Exception:
            self.stop()
            raise

    def read(self):
        if not self.streaming:
            raise RuntimeError("Camera acquisition is stopped")
        try:
            raw = self._pipeline.wait_for_frames(1000)
            receipt = Stamp.receive()
            color_raw, depth_raw = raw.get_color_frame(), raw.get_depth_frame()
            if not color_raw or not depth_raw:
                raise RuntimeError("Incomplete RGB-D frameset")
            color_time, depth_time = _device_time(color_raw), _device_time(depth_raw)
            numbers = (int(color_raw.get_frame_number()), int(depth_raw.get_frame_number()))
            aligned = self._align.process(raw)
            color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color or not depth:
                raise RuntimeError("Alignment returned incomplete RGB-D")
            rgb = np.asanyarray(color.get_data())[:, :, ::-1].copy()
            depth_data = np.asanyarray(depth.get_data()).copy()
            k = depth.profile.as_video_stream_profile().intrinsics
            ck = color.profile.as_video_stream_profile().intrinsics
            intrinsics = {n: getattr(k, n) for n in ("width", "height", "fx", "fy", "ppx", "ppy")}
            intrinsics.update(model=str(k.model), coeffs=list(k.coeffs),
                              color_model=str(ck.model), color_coeffs=list(ck.coeffs))
            stamp = replace(receipt, device=color_time, processed_monotonic_ns=monotonic_ns())
            stream = {**vars(self.profile), "color_capture_format": "BGR8", "color_output_format": "RGB8",
                      "depth_format": "Z16", "alignment_target": "color"}
            return RGBDFrame(str(uuid.uuid4()), self.serial, stream, *numbers,
                             color_time, depth_time, stamp, intrinsics, self._scale, rgb, depth_data)
        except Exception:
            self.disconnect()
            raise

    def stop(self):
        if self.streaming:
            self.streaming = False
            self._pipeline.stop()

    def disconnect(self):
        try:
            self.stop()
        finally:
            self.connected = False
            self._pipeline = None


@dataclass(frozen=True)
class GaugeReading:
    channel: int
    kpa: float
    stamp: Stamp
    integrity: str
    raw_hex: str


def serial_reader(device):
    import serial
    port = serial.Serial(port=None, baudrate=BAUD, bytesize=8, parity="N", stopbits=1, timeout=.05, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = device
    port.open()
    return port


class VacuumObserver:
    """Receive-only EE16/ASCII parsing; separate clocks for each asynchronous gauge."""
    def __init__(self, opener=serial_reader):
        self._opener = opener
        self._port = None
        self.connected = False
        self.binary, self.debug = {}, {}
        self.alarms = {}
        self.bad_frames = 0
        self._parser, self._text = FrameParser(), TextReportParser()

    def connect(self, device):
        if self.connected:
            raise RuntimeError("Disconnect serial first")
        self._port = self._opener(device)
        self.binary, self.debug, self.alarms = {}, {}, {}
        self._parser, self._text = FrameParser(), TextReportParser()
        self.connected = True

    def send(self, *args, **kwargs):
        raise ReadOnlyViolation("Stage one never writes serial bytes, including pressure queries")

    def read(self):
        if not self.connected:
            raise RuntimeError("Serial not connected")
        try:
            data = self._port.read(64)
            receipt = Stamp.receive()  # Stamp immediately on receive, before any GUI queue.
            return self.feed(data, receipt)
        except Exception:
            self.disconnect()
            raise

    def feed(self, data, receipt=None):
        receipt = receipt or Stamp.receive()
        events = []
        for command, parameters in self._parser.feed(data):
            reading = parse_heartbeat(command, parameters)
            if reading is not None:
                gauge, value = reading
                sample = GaugeReading(gauge, value, receipt, "CHECKED_BINARY", parameters.hex())
                self.binary[gauge] = sample
                events.append(("pressure", sample))
            if command in ALARMS:
                self.alarms[command] = {"name": COMMANDS[command], "parameters": parameters.hex(), "stamp": receipt}
                events.append(("alarm", self.alarms[command]))
        self.bad_frames = self._parser.rejected
        self._parser.bad.clear()
        for gauge, value in self._text.feed(data):
            if 1 <= gauge <= 4 and np.isfinite(value) and abs(value) <= 2000:
                sample = GaugeReading(gauge, value, receipt, "ASCII_DEBUG_UNCHECKED", "")
                self.debug[gauge] = sample
                events.append(("debug_pressure", sample))
        self._text.messages.clear()
        return events

    def gauges(self):
        # A stale heartbeat is not silently replaced by a fresh unchecked debug value.
        return {i: self.binary.get(i, self.debug.get(i)) for i in range(1, 5)}

    def disconnect(self):
        port, self._port = self._port, None
        self.connected = False
        self.binary, self.debug, self.alarms = {}, {}, {}
        if port is not None:
            port.close()
