"""Matplotlib visualization for scene and planned robot trajectory."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from .geometry import OBB
from .robot import RobotBackend
from .scene import TrailerScene


_BOX_EDGES = [
    (0, 1), (0, 2), (0, 4),
    (1, 3), (1, 5),
    (2, 3), (2, 6),
    (3, 7),
    (4, 5), (4, 6),
    (5, 7), (6, 7),
]


def _plot_obb(ax, box: OBB, linewidth: float = 0.8, alpha: float = 0.6) -> None:
    corners = box.corners()
    for i, j in _BOX_EDGES:
        p, q = corners[i], corners[j]
        ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], linewidth=linewidth, alpha=alpha)
    ax.text(box.center[0], box.center[1], box.center[2], box.name, fontsize=6)


def _plot_robot(ax, robot: RobotBackend, q: np.ndarray, linewidth: float = 2.4, alpha: float = 1.0) -> None:
    frames = robot.frames(q, include_tool=True)
    points = np.array([f[:3, 3] for f in frames])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], marker="o", linewidth=linewidth, alpha=alpha)


def save_plan_figure(
    path: str | Path,
    robot: RobotBackend,
    scene: TrailerScene,
    trajectory: Sequence[np.ndarray],
    target_name: str | None = None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")

    for obstacle in scene.obstacles:
        _plot_obb(ax, obstacle, linewidth=0.65, alpha=0.35)
    for carton in scene.cartons:
        _plot_obb(ax, carton, linewidth=1.2 if carton.name == target_name else 0.9, alpha=0.95)

    if trajectory:
        sample_indices = sorted(set(np.linspace(0, len(trajectory) - 1, min(8, len(trajectory)), dtype=int).tolist()))
        for idx in sample_indices[:-1]:
            _plot_robot(ax, robot, np.asarray(trajectory[idx]), linewidth=1.0, alpha=0.22)
        _plot_robot(ax, robot, np.asarray(trajectory[-1]), linewidth=3.0, alpha=1.0)
        tool_points = np.array([robot.fk(np.asarray(q))[:3, 3] for q in trajectory])
        ax.plot(tool_points[:, 0], tool_points[:, 1], tool_points[:, 2], linestyle="--", linewidth=1.3)

    ax.set_xlabel("X into trailer [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_xlim(-1.1, 1.5)
    ax.set_ylim(-1.35, 1.35)
    ax.set_zlim(0.0, 2.5)
    ax.set_title("Layer-1 unloading geometry and motion plan")
    ax.view_init(elev=24, azim=-58)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
