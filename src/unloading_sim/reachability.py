"""Deterministic batch pregrasp reachability and collision heatmaps."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from .demo import build_robot
from .geometry import make_tool_rotation, make_transform
from .ik import solve_ik_multistart
from .online_unload import amr_dock_positions, docked_robot_and_scene
from .scene import TrailerScene, load_scene_config


UNREACHABLE = 0
COLLISION_BLOCKED = 1
COLLISION_FREE = 2


@dataclass(frozen=True)
class PregraspHeatmap:
    y_values: np.ndarray
    z_values: np.ndarray
    status: np.ndarray
    joint_solutions: np.ndarray
    position_error: np.ndarray
    robot_base_position: np.ndarray
    pregrasp_x: float

    def nearest_seed(self, position: Sequence[float], robot_base_position: Sequence[float]) -> np.ndarray | None:
        if not np.allclose(robot_base_position, self.robot_base_position, atol=1e-6):
            return None
        point = np.asarray(position, dtype=float)
        yi = int(np.argmin(np.abs(self.y_values - point[1])))
        zi = int(np.argmin(np.abs(self.z_values - point[2])))
        if self.status[zi, yi] != COLLISION_FREE:
            return None
        return self.joint_solutions[zi, yi].copy()


def compute_pregrasp_heatmap(
    robot,
    scene: TrailerScene,
    home_q: np.ndarray,
    pregrasp_x: float,
    y_values: np.ndarray,
    z_values: np.ndarray,
    seed: int = 11,
    max_iterations: int = 80,
    position_tolerance: float = 0.025,
    orientation_tolerance: float = 0.20,
) -> PregraspHeatmap:
    """Classify a trailer-door Y/Z grid as unreachable, blocked, or free."""
    y_values = np.asarray(y_values, dtype=float)
    z_values = np.asarray(z_values, dtype=float)
    status = np.full((len(z_values), len(y_values)), UNREACHABLE, dtype=np.uint8)
    joint_solutions = np.full((len(z_values), len(y_values), 6), np.nan, dtype=float)
    position_error = np.full((len(z_values), len(y_values)), np.nan, dtype=float)
    rotation = make_tool_rotation(np.array([1.0, 0.0, 0.0]))
    rng = np.random.default_rng(int(seed))
    previous_seed = np.asarray(home_q, dtype=float)

    for zi, z in enumerate(z_values):
        y_indices = range(len(y_values)) if zi % 2 == 0 else range(len(y_values) - 1, -1, -1)
        for yi in y_indices:
            pose = make_transform(rotation, [pregrasp_x, y_values[yi], z])
            geometric = solve_ik_multistart(
                robot,
                pose,
                seeds=[previous_seed, home_q],
                obstacles=(),
                random_restarts=0,
                rng=rng,
                max_iterations=max_iterations,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
            )
            position_error[zi, yi] = geometric.position_error
            if not geometric.success:
                continue
            if robot.is_collision_free(geometric.q, scene.all_obstacles):
                result = geometric
            else:
                status[zi, yi] = COLLISION_BLOCKED
                result = solve_ik_multistart(
                    robot,
                    pose,
                    seeds=[geometric.q, previous_seed, home_q],
                    obstacles=scene.all_obstacles,
                    random_restarts=1,
                    rng=rng,
                    max_iterations=max_iterations,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                )
                if not result.success:
                    continue
            status[zi, yi] = COLLISION_FREE
            joint_solutions[zi, yi] = result.q
            position_error[zi, yi] = result.position_error
            previous_seed = result.q

    return PregraspHeatmap(
        y_values,
        z_values,
        status,
        joint_solutions,
        position_error,
        robot.base_transform[:3, 3].copy(),
        float(pregrasp_x),
    )


def save_pregrasp_heatmap(result: PregraspHeatmap, output_prefix: str | Path) -> tuple[Path, Path, Path]:
    """Write a reusable NPZ seed cache, PNG heatmap, and JSON summary."""
    from matplotlib import pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch

    prefix = Path(output_prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    npz_path = prefix.with_suffix(".npz")
    png_path = prefix.with_suffix(".png")
    json_path = prefix.with_suffix(".json")
    np.savez_compressed(
        npz_path,
        y_values=result.y_values,
        z_values=result.z_values,
        status=result.status,
        joint_solutions=result.joint_solutions,
        position_error=result.position_error,
        robot_base_position=result.robot_base_position,
        pregrasp_x=result.pregrasp_x,
    )

    cmap = ListedColormap(["#6b7280", "#dc2626", "#16a34a"])
    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], cmap.N)
    fig, ax = plt.subplots(figsize=(8.0, 5.2), constrained_layout=True)
    extent = [result.y_values[0], result.y_values[-1], result.z_values[0], result.z_values[-1]]
    ax.imshow(result.status, origin="lower", aspect="auto", extent=extent, cmap=cmap, norm=norm, interpolation="nearest")
    ax.set_title(f"FANUC pregrasp heatmap at X={result.pregrasp_x:.2f} m")
    ax.set_xlabel("Y — left positive (m)")
    ax.set_ylabel("Z — height (m)")
    ax.grid(color="white", alpha=0.18, linewidth=0.5)
    ax.legend(
        handles=[
            Patch(color="#16a34a", label="collision-free"),
            Patch(color="#dc2626", label="IK reachable, collision blocked"),
            Patch(color="#6b7280", label="unreachable"),
        ],
        loc="lower right",
        framealpha=0.9,
    )
    fig.savefig(png_path, dpi=150)
    plt.close(fig)

    counts = {str(code): int(np.count_nonzero(result.status == code)) for code in (UNREACHABLE, COLLISION_BLOCKED, COLLISION_FREE)}
    json_path.write_text(
        json.dumps(
            {
                "pregrasp_x_m": result.pregrasp_x,
                "robot_base_position_m": result.robot_base_position.tolist(),
                "grid_shape": list(result.status.shape),
                "status_counts": {"unreachable": counts["0"], "collision_blocked": counts["1"], "collision_free": counts["2"]},
                "npz_cache": str(npz_path),
                "png": str(png_path),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    return npz_path, png_path, json_path


def load_pregrasp_heatmap(path: str | Path) -> PregraspHeatmap:
    data = np.load(path)
    return PregraspHeatmap(
        data["y_values"],
        data["z_values"],
        data["status"],
        data["joint_solutions"],
        data["position_error"],
        data["robot_base_position"],
        float(data["pregrasp_x"]),
    )


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Generate a deterministic pregrasp reachability/collision heatmap")
    parser.add_argument("--config", default="config/fanuc_m20id35.yaml")
    parser.add_argument("--output-prefix", default="outputs/fanuc_m20id35/pregrasp_heatmap")
    parser.add_argument("--dock-index", type=int, default=0)
    parser.add_argument("--y-samples", type=int, default=21)
    parser.add_argument("--z-samples", type=int, default=17)
    args = parser.parse_args(argv)

    scene, cfg = load_scene_config(args.config)
    dock = amr_dock_positions(cfg)[args.dock_index]
    robot, docked_scene, _ = docked_robot_and_scene(scene, cfg, dock)
    carton_front_x = min(carton.center[0] - carton.half_extents[0] for carton in scene.cartons)
    result = compute_pregrasp_heatmap(
        robot,
        docked_scene,
        np.asarray(cfg["robot"]["home_joints"], dtype=float),
        pregrasp_x=carton_front_x - 0.18,
        y_values=np.linspace(-0.98, 0.98, args.y_samples),
        z_values=np.linspace(0.30, 1.65, args.z_samples),
        seed=int(cfg.get("planning", {}).get("seed", 11)),
    )
    paths = save_pregrasp_heatmap(result, args.output_prefix)
    print(json.dumps({"npz": str(paths[0]), "png": str(paths[1]), "summary": str(paths[2])}, indent=2))


if __name__ == "__main__":
    main()
