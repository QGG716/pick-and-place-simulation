"""Portable RGB-D/joint/pressure replay. Original observations remain immutable."""
from __future__ import annotations

from dataclasses import asdict, replace
import json
from pathlib import Path
from time import monotonic_ns

import numpy as np

from .contracts import (Source, Stamp, DeviceTime, RGBDFrame, JointSample,
                        PressureSample, DetectionResult)


def _path(root, name):
    path = (root / name).resolve()
    path.relative_to(root.resolve())
    return path


def _stamp(raw, presented):
    data = dict(raw)
    # A captured LIVE/SIMULATED stamp names its origin in source. A replay of a
    # replay must supply original_source; missing/unknown origins stay unknown.
    original_source = data.get("original_source")
    if data.get("source") != Source.REPLAY:
        if original_source is not None and original_source != data.get("source"):
            raise ValueError("Conflicting recorded source and original_source")
        original_source = data.get("source")
    data.update(source=Source.REPLAY, original_source=original_source,
                presented_monotonic_ns=presented, device=DeviceTime(**data.get("device", {})))
    return Stamp(**data)


class Replay:
    def __init__(self, directory):
        self.root = Path(directory).resolve()
        self.manifest = json.loads((self.root / "manifest.json").read_text(encoding="utf-8"))
        if self.manifest.get("schema") != "eco65_observation_replay_v1":
            raise ValueError("Unknown observation replay schema")
        if not self.manifest.get("provenance"):
            raise ValueError("Replay requires explicit data provenance")
        self.records = self.manifest["records"]
        if not self.records:
            raise ValueError("Replay has no observations")
        self.index = 0

    def step(self):
        if self.index >= len(self.records):
            raise StopIteration
        record = self.records[self.index]
        now = monotonic_ns()
        meta = dict(record["frame"])
        with np.load(_path(self.root, meta.pop("arrays")), allow_pickle=False) as arrays:
            rgb, depth = arrays["rgb"].copy(), arrays["depth"].copy()
        meta["stamp"] = _stamp(meta["stamp"], now)
        meta["rgb_device_time"] = DeviceTime(**meta["rgb_device_time"])
        meta["depth_device_time"] = DeviceTime(**meta["depth_device_time"])
        frame = RGBDFrame(**meta, rgb=rgb, depth=depth)
        q = dict(record["joint"])
        q["stamp"] = _stamp(q["stamp"], now)
        joint = JointSample(**q)
        p = dict(record["pressure"])
        p["stamp"] = _stamp(p["stamp"], now)
        pressure = PressureSample(**p)
        detections = record.get("detections")
        result = None
        if detections:
            if detections["observation_id"] != frame.observation_id:
                raise ValueError("Detection belongs to another original observation")
            masks = []
            for filename in detections.get("masks", []):
                mask = np.load(_path(self.root, filename), allow_pickle=False)
                if mask.shape != depth.shape or mask.dtype != np.bool_:
                    raise ValueError("Invalid original segmentation mask")
                mask.setflags(write=False)
                masks.append(mask)
            result = DetectionResult(frame.observation_id, Source.REPLAY,
                detections["completed_monotonic_ns"], tuple(detections["items"]), tuple(masks),
                detections["provenance"])
        self.index += 1
        return frame, joint, pressure, result

    def rewind(self):
        self.index = 0


def generate_demo(directory):
    """Authored synthetic fixture, never described as a captured hardware session."""
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    if (root / "manifest.json").exists():
        raise FileExistsError("Use a new replay fixture directory")
    h, w = 240, 320
    rgb = np.zeros((h, w, 3), np.uint8)
    rgb[:] = [31, 43, 54]
    rgb[65:185, 85:235] = [188, 140, 83]
    depth = np.full((h, w), 800, np.uint16)
    depth[65:185, 85:235] = 650
    mask = np.zeros((h, w), bool)
    mask[65:185, 85:235] = True
    np.savez_compressed(root / "rgbd.npz", rgb=rgb, depth=depth)
    np.save(root / "mask.npy", mask, allow_pickle=False)
    records = []
    for i, q_deg in enumerate(([0, 0, 0, 0, 0, 0], [20, -30, 45, 10, 25, -15], [-35, 15, -40, 20, -30, 45])):
        # Stable fixture clock, explicitly not a real device clock or capture time.
        stamp = asdict(Stamp(Source.SIMULATED, 1_000_000_000+i*1_000_000_000, 0,
                             "AUTHORED_FIXTURE_CLOCK", DeviceTime()))
        observation_id = f"synthetic_rgbd_{i}"
        records.append({
            "frame": {"observation_id": observation_id, "device_id": "NO_PHYSICAL_DEVICE",
                "stream": {"width": w, "height": h, "fps": 1, "format": "RGB8/Z16"},
                "rgb_number": i, "depth_number": i, "rgb_device_time": asdict(DeviceTime()),
                "depth_device_time": asdict(DeviceTime()), "stamp": stamp,
                "intrinsics": {"width": w, "height": h, "fx": 300., "fy": 300., "ppx": 160., "ppy": 120., "model": "SYNTHETIC_PINHOLE", "coeffs": [0.]*5},
                "depth_scale_m": .001, "arrays": "rgbd.npz"},
            "joint": {"sample_id": f"synthetic_joint_{i}", "q_rad": np.deg2rad(q_deg).tolist(),
                "stamp": stamp, "raw": {"q_deg": q_deg}, "mapping_id": "SIMULATOR_JOINT_ORDER"},
            "pressure": {"sample_id": f"synthetic_pressure_{i}", "kpa": [-55.-i, -70., -80., 200.],
                "alarms": ["SYNTHETIC_ALARM"] if i == 2 else [], "stamp": stamp,
                "integrity": "REPLAY_FIXTURE", "raw_hex": ""},
            "detections": {"observation_id": observation_id,
                "completed_monotonic_ns": stamp["received_monotonic_ns"]+20_000_000,
                "provenance": "AUTHORED_RECTANGLE_MASK_NOT_MODEL_INFERENCE", "masks": ["mask.npy"],
                "items": [{"label": "synthetic carton", "score": 1., "bbox_xyxy": [85, 65, 235, 185],
                           "size_m": [.15, .1, .1], "frame_id": "camera_optical", "center_m": [0., 0.0108333333, .65]}]}})
    manifest = {"schema": "eco65_observation_replay_v1",
        "provenance": "AUTHORED_SYNTHETIC_FIXTURE; no camera, robot or pressure hardware was measured",
        "records": records}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return root
