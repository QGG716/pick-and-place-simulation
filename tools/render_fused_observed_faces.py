#!/usr/bin/env python3
"""Render fused observed faces in world-frame 3D, top, and side views."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


MODULE_COLORS = {
    "module_0_upper": "#1565c0",
    "module_1_lower": "#ef6c00",
}


def _faces(payload: dict) -> list[dict]:
    result = []
    for obj in payload.get("objects", []):
        modules = tuple(obj.get("contributing_modules", ()))
        color = "#7b1fa2" if len(modules) > 1 else MODULE_COLORS.get(modules[0] if modules else "", "#546e7a")
        for face in obj.get("observed_faces", []):
            points = np.asarray(face.get("corners_3d_m", ()), dtype=float)
            if points.shape == (4, 3) and np.isfinite(points).all():
                result.append({"points": points, "color": color, "modules": modules})
    return result


def _equal_3d_axes(axis, points: np.ndarray) -> None:
    low = points.min(axis=0)
    high = points.max(axis=0)
    center = (low + high) * 0.5
    radius = max(float(np.max(high - low)) * 0.55, 0.1)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(max(0.0, center[2] - radius), center[2] + radius)


def render(input_path: Path, output_path: Path) -> dict:
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    faces = _faces(payload)
    if not faces:
        raise ValueError("fused result contains no finite observed faces")
    all_points = np.vstack([item["points"] for item in faces])
    figure = plt.figure(figsize=(18, 6.3), constrained_layout=True)

    axis_3d = figure.add_subplot(1, 3, 1, projection="3d")
    for item in faces:
        axis_3d.add_collection3d(Poly3DCollection(
            [item["points"]], facecolors=item["color"], edgecolors="#202020",
            linewidths=0.55, alpha=0.40,
        ))
    _equal_3d_axes(axis_3d, all_points)
    axis_3d.view_init(elev=24, azim=-58)
    axis_3d.set_xlabel("world X [m]")
    axis_3d.set_ylabel("world Y [m]")
    axis_3d.set_zlabel("world Z [m]")
    axis_3d.set_title("Fused directly observed faces (3D)")

    for position, columns, labels, title in (
        (2, (0, 1), ("world X [m]", "world Y [m]"), "True top view (XY)"),
        (3, (0, 2), ("world X [m]", "world Z [m]"), "True side view (XZ)"),
    ):
        axis = figure.add_subplot(1, 3, position)
        polygons = [item["points"][:, columns] for item in faces]
        axis.add_collection(PolyCollection(
            polygons, facecolors=[item["color"] for item in faces],
            edgecolors="#202020", linewidths=0.55, alpha=0.40,
        ))
        projected = all_points[:, columns]
        padding = np.maximum(np.ptp(projected, axis=0) * 0.06, 0.05)
        axis.set_xlim(projected[:, 0].min() - padding[0], projected[:, 0].max() + padding[0])
        axis.set_ylim(projected[:, 1].min() - padding[1], projected[:, 1].max() + padding[1])
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.25)
        axis.set_xlabel(labels[0])
        axis.set_ylabel(labels[1])
        axis.set_title(title)

    figure.suptitle(
        f"Dual RGB-D world-frame face fusion | objects={len(payload.get('objects', []))} | "
        f"faces={len(faces)} | oracle identity used: false",
        fontsize=13,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return {
        "object_count": len(payload.get("objects", [])),
        "observed_face_count": len(faces),
        "coverage_status": payload.get("coverage_status"),
        "output": str(output_path.resolve()),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(render(args.input, args.output), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
