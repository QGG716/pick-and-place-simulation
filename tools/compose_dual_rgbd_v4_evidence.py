#!/usr/bin/env python3
"""Compose same-frame geometry comparisons and a concise Isaac demo video."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def _read(path: Path) -> np.ndarray:
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(path)
    return image


def comparison(left_path: Path, right_path: Path, output: Path, title: str) -> None:
    left, right = _read(left_path), _read(right_path)
    height = min(left.shape[0], right.shape[0])
    width = int(round(left.shape[1] * height / left.shape[0]))
    left = cv2.resize(left, (width, height), interpolation=cv2.INTER_AREA)
    right = cv2.resize(right, (width, height), interpolation=cv2.INTER_AREA)
    image = np.hstack((left, right))
    cv2.rectangle(image, (0, 0), (image.shape[1], 48), (15, 15, 15), -1)
    cv2.putText(image, title, (18, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), image):
        raise RuntimeError(f"failed to write {output}")


def _video_frames(path: Path, count: int, size: tuple[int, int]) -> list[np.ndarray]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise FileNotFoundError(path)
    result = []
    try:
        while len(result) < count:
            ok, frame = capture.read()
            if not ok:
                break
            if (frame.shape[1], frame.shape[0]) != size:
                frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            result.append(frame)
    finally:
        capture.release()
    if len(result) < count:
        raise ValueError(f"{path} provides {len(result)} frames, expected at least {count}")
    return result


def compose_video(first: Path, second: Path, output: Path, *, fps: float = 20.0, seconds_each: float = 8.0) -> dict:
    probe = cv2.VideoCapture(str(first))
    if not probe.isOpened():
        raise FileNotFoundError(first)
    width = int(probe.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(probe.get(cv2.CAP_PROP_FRAME_HEIGHT))
    probe.release()
    count = int(round(fps * seconds_each))
    frames = _video_frames(first, count, (width, height)) + _video_frames(second, count, (width, height))
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"failed to open video writer for {output}")
    try:
        for frame in frames:
            writer.write(frame)
    finally:
        writer.release()
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    return {
        "output": str(output.resolve()), "sha256": digest, "fps": fps,
        "frame_count": len(frames), "duration_seconds": len(frames) / fps,
        "segments": [
            {"source": str(first.resolve()), "duration_seconds": seconds_each},
            {"source": str(second.resolve()), "duration_seconds": seconds_each},
        ],
        "note": "sequential excerpts from real Isaac capture and Mode B1 validation; no synthetic interpolation",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-directory", required=True, type=Path)
    args = parser.parse_args()
    root = args.capture_directory.resolve()
    full = root / "FULL_STACK_NOMINAL"
    lower = full / "modules" / "module_1_lower"
    comparison(full / "rgbd_cuboids_baseline.png", full / "rgbd_cuboids.png", root / "16_upper_baseline_vs_v4.png", "UPPER | BASE GEOMETRY (left) | V4 OBSERVED FACES (right)")
    comparison(lower / "rgbd_cuboids_baseline.png", lower / "rgbd_cuboids.png", root / "17_lower_baseline_vs_v4.png", "LOWER | BASE GEOMETRY (left) | V4 OBSERVED FACES (right)")
    metadata = compose_video(
        root / "isaac_j1_mast_rgbd_validation.mp4",
        root / "isaac_perception_mode_b1_validation.mp4",
        root / "dual_rgbd_v4_isaac_demo_16s.mp4",
    )
    (root / "dual_rgbd_v4_isaac_demo_16s.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
