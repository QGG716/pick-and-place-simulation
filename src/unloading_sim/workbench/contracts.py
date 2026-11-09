"""Observation identity and freshness, independent of GUI and hardware SDKs.

Device timestamps are never synthesized. Replay preserves the recorded clock
and adds a separate presentation timestamp in the current host clock domain.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from time import monotonic_ns, time_ns
from typing import Any
import math
import uuid

import numpy as np


class Source(str, Enum):
    LIVE = "LIVE"
    REPLAY = "REPLAY"
    SIMULATED = "SIMULATED"


HOST_SESSION = str(uuid.uuid4())


@dataclass(frozen=True)
class DeviceTime:
    value_ms: float | None = None
    domain: str = "UNKNOWN"

    def __post_init__(self):
        if self.value_ms is not None and not math.isfinite(self.value_ms):
            raise ValueError("Nonfinite device timestamp")


@dataclass(frozen=True)
class Stamp:
    source: Source
    received_monotonic_ns: int
    received_utc_ns: int
    host_session: str
    device: DeviceTime = field(default_factory=DeviceTime)
    processed_monotonic_ns: int | None = None
    presented_monotonic_ns: int | None = None
    original_source: str | None = None

    @classmethod
    def receive(cls, source=Source.LIVE, device=None):
        return cls(Source(source), monotonic_ns(), time_ns(), HOST_SESSION,
                   device or DeviceTime())

    def age_s(self, now_ns=None):
        now = monotonic_ns() if now_ns is None else now_ns
        if self.source == Source.REPLAY:
            origin = self.presented_monotonic_ns
        else:
            origin = self.received_monotonic_ns if self.host_session == HOST_SESSION else None
        if origin is None or now < origin:
            return None
        return (now - origin) / 1e9

    def state(self, connected, max_age_s, now_ns=None):
        age = self.age_s(now_ns)
        if not connected:
            return "DISCONNECTED"
        if age is None:
            return "CLOCK_UNKNOWN"
        if age > max_age_s:
            return "STALE"
        return "LIVE_VALID" if self.source == Source.LIVE else self.source.value


def finite_vector(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return tuple(float(v) for v in result)


@dataclass(frozen=True)
class JointSample:
    sample_id: str
    q_rad: tuple[float, ...]
    stamp: Stamp
    raw: dict[str, Any] = field(default_factory=dict)
    mapping_id: str = "UNVERIFIED"

    def __post_init__(self):
        object.__setattr__(self, "q_rad", finite_vector(self.q_rad, 6, "q_rad"))


PRESSURE_CHANNELS = ("gripper", "reservoir", "vacuum_pump", "compressor")


@dataclass(frozen=True)
class PressureSample:
    sample_id: str
    kpa: tuple[float, ...]
    alarms: tuple[str, ...]
    stamp: Stamp
    integrity: str
    raw_hex: str = ""

    def __post_init__(self):
        object.__setattr__(self, "kpa", finite_vector(self.kpa, 4, "pressure kPa"))
        if self.integrity not in {"CHECKED_BINARY", "ASCII_DEBUG_UNCHECKED", "REPLAY_FIXTURE"}:
            raise ValueError("Unknown pressure integrity")


@dataclass(frozen=True)
class RGBDFrame:
    observation_id: str
    device_id: str
    stream: dict[str, Any]
    rgb_number: int | None
    depth_number: int | None
    rgb_device_time: DeviceTime
    depth_device_time: DeviceTime
    stamp: Stamp
    intrinsics: dict[str, Any]
    depth_scale_m: float
    rgb: np.ndarray
    depth: np.ndarray
    frame_id: str = "camera_optical"
    aligned_to: str = "color"

    def __post_init__(self):
        if self.rgb.ndim != 3 or self.rgb.shape[2] != 3 or self.rgb.dtype != np.uint8:
            raise ValueError("RGB must be HxWx3 uint8")
        if self.depth.shape != self.rgb.shape[:2] or self.depth.dtype != np.uint16:
            raise ValueError("Aligned depth must be HxW uint16")
        if self.aligned_to != "color" or self.frame_id != "camera_optical":
            raise ValueError("Expected RGB-aligned optical depth")
        if not math.isfinite(self.depth_scale_m) or self.depth_scale_m <= 0:
            raise ValueError("SDK depth scale must be finite and positive")
        k = self.intrinsics
        if (k.get("width"), k.get("height")) != (self.rgb.shape[1], self.rgb.shape[0]):
            raise ValueError("Intrinsics resolution differs from aligned images")
        if not all(math.isfinite(float(k[n])) for n in ("fx", "fy", "ppx", "ppy")):
            raise ValueError("Invalid intrinsics")
        if min(k["fx"], k["fy"]) <= 0:
            raise ValueError("Invalid focal length")
        # Producers transfer ownership. Neither overlays nor consumers may edit observations.
        self.rgb.setflags(write=False)
        self.depth.setflags(write=False)


@dataclass(frozen=True)
class DetectionResult:
    observation_id: str
    source: Source
    completed_monotonic_ns: int
    detections: tuple[dict[str, Any], ...]
    masks: tuple[np.ndarray, ...] = ()
    provenance: str = "EXISTING_DETECTOR_OUTPUT"

    def __post_init__(self):
        if not self.observation_id or not self.provenance:
            raise ValueError("Detection requires original observation identity and provenance")
        if self.masks and len(self.masks) != len(self.detections):
            raise ValueError("Detection and segmentation counts differ")
        for item in self.detections:
            if item.get("frame_id") != "camera_optical":
                raise ValueError("Original detection coordinates must explicitly use camera_optical")
            if item.get("center_m") is not None:
                finite_vector(item["center_m"], 3, "detection center")
            score = float(item["score"])
            if not math.isfinite(score) or not 0 <= score <= 1:
                raise ValueError("Invalid detection confidence")
        for mask in self.masks:
            if mask.ndim != 2 or mask.dtype != np.bool_:
                raise ValueError("Segmentation must be a two-dimensional boolean mask")
            mask.setflags(write=False)


@dataclass(frozen=True)
class Pairing:
    status: str
    frame_id: str
    joint_id: str | None
    q_capture_rad: tuple[float, ...] | None
    receipt_delta_s: float | None
    reliable_capture_pair: bool = False
    stationary_evidence: str = "NONE"


def static_pair(frame, joint, *, camera_connected, robot_connected,
                manual_stationary=False, max_age_s=1.0, max_skew_s=0.1, now_ns=None):
    """Receipt-time proximity is only a visualization approximation, never sync proof."""
    def result(status, q=None, delta=None, evidence="NONE"):
        return Pairing(status, frame.observation_id, joint.sample_id if joint else None,
                       q, delta, False, evidence)
    if joint is None:
        return result("NO_JOINT_SAMPLE")
    for sample, connected in ((frame, camera_connected), (joint, robot_connected)):
        if sample.stamp.state(connected, max_age_s, now_ns) not in {"LIVE_VALID", "REPLAY", "SIMULATED"}:
            return result("STALE_OR_DISCONNECTED")
    if frame.stamp.source != joint.stamp.source or frame.stamp.host_session != joint.stamp.host_session:
        return result("INCOMPATIBLE_CLOCK_OR_SOURCE")
    delta = abs(frame.stamp.received_monotonic_ns-joint.stamp.received_monotonic_ns)/1e9
    if delta > max_skew_s:
        return result("RECEIPT_TIME_MISMATCH", delta=delta)
    if not manual_stationary:
        return result("NO_STATIONARY_EVIDENCE", delta=delta)
    return result("MANUAL_STATIC_APPROXIMATION_NOT_SYNCHRONIZED", joint.q_rad,
                  delta, "MANUAL_CONFIRMATION")
