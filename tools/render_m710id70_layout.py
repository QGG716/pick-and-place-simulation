"""Render the confirmed M-710 layout from one frozen scene snapshot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.workcell_layout import load_layout_validation_config, verify_scene_snapshot


COLORS = {
    "chassis": "#455A64",
    "conveyor_transverse": "#1976A3",
    "conveyor_longitudinal": "#2A83B8",
    "carton": "#C8843E",
    "robot": "#E6B422",
    "tool": "#00A6A6",
}


def _hull(points: np.ndarray) -> np.ndarray:
    unique = sorted(set(map(tuple, np.asarray(points, dtype=float))))
    if len(unique) <= 2:
        return np.asarray(unique)
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1])


def _corners(record: dict) -> np.ndarray:
    pose = np.asarray(record["pose_world"], dtype=float)
    half = np.asarray(record["half_extents_m"], dtype=float)
    local = np.asarray([[x, y, z] for x in (-half[0], half[0]) for y in (-half[1], half[1]) for z in (-half[2], half[2])])
    return (pose[:3, :3] @ local.T).T + pose[:3, 3]


def _draw_record(ax, record: dict, axes: tuple[int, int], color: str, alpha: float, zorder: int) -> None:
    polygon = _hull(_corners(record)[:, axes])
    ax.add_patch(Polygon(polygon, closed=True, facecolor=color, edgecolor="#263238", linewidth=0.8, alpha=alpha, zorder=zorder))


def _draw_scene(ax, snapshot: dict, axes: tuple[int, int], *, include_stack: bool = True) -> None:
    for item in snapshot["assembly"]["fixed_components"]:
        _draw_record(ax, item, axes, COLORS[item["name"]], 0.82, 2)
    if include_stack:
        for carton in snapshot["cartons"]:
            _draw_record(ax, carton, axes, COLORS["carton"], 0.50, 3)
    for link in snapshot["robot"]["link_collision_obbs"]:
        _draw_record(ax, link, axes, COLORS["robot"], 0.72, 5)
    _draw_record(ax, snapshot["tool"]["collision_obb"], axes, COLORS["tool"], 0.85, 6)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#CFD8DC", linewidth=0.5, alpha=0.7)


def _dimension(ax, start, end, text, offset=(0.0, 0.0), color="#B71C1C") -> None:
    start = np.asarray(start, dtype=float) + offset
    end = np.asarray(end, dtype=float) + offset
    ax.annotate("", xy=end, xytext=start, arrowprops={"arrowstyle": "<->", "color": color, "lw": 1.2})
    center = 0.5 * (start + end)
    vertical = abs(end[1] - start[1]) > abs(end[0] - start[0])
    ax.text(
        center[0], center[1], text, color=color, fontsize=8.5,
        ha="center", va="center" if vertical else "bottom",
        rotation=90 if vertical else 0,
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none", "pad": 1.0},
    )


def _top(ax, snapshot: dict) -> None:
    _draw_scene(ax, snapshot, (0, 1))
    trailer = snapshot["trailer"]
    ax.axhline(trailer["left_wall_y_m"], color="#5D4037", lw=1.5)
    ax.axhline(trailer["right_wall_y_m"], color="#5D4037", lw=1.5)
    ax.text(-3.0, trailer["left_wall_y_m"] + 0.03, "left inner wall Y=+1150 mm", fontsize=8, ha="left")
    ax.text(-3.0, trailer["right_wall_y_m"] - 0.08, "right inner wall Y=-1150 mm", fontsize=8, ha="left")
    _dimension(ax, (-3.0, 1.10), (-3.0, 1.15), "50 mm", offset=(-0.10, 0))
    _dimension(ax, (-3.0, -1.15), (-3.0, -1.10), "50 mm", offset=(-0.10, 0))
    _dimension(ax, (-0.2, 1.02), (0.0, 1.02), "200 mm stack gap")
    _dimension(ax, (-1.10, 0.72), (-0.90, 0.72), "200 mm base-front setback")
    _dimension(ax, (-3.0, -1.23), (-0.2, -1.23), "2800 mm fixed assembly X envelope")
    _dimension(ax, (-3.52, -1.10), (-3.52, 1.10), "2200 mm fixed assembly Y envelope")
    ax.plot([-1.10, -1.10], [0.04, 0.66], color="#B71C1C", ls="--", lw=0.9)
    ax.text(-1.10, 0.69, "mount proxy front edge", fontsize=7.5, ha="center", color="#B71C1C")
    ax.text(-1.95, 0.92, "CHASSIS\n2100 × 1500 × 600 mm", ha="center", va="center", fontsize=8, color="white")
    ax.text(-0.55, 0.35, "TRANSVERSE BELT\n700 × 1500 mm", ha="center", va="center", fontsize=8, color="white", rotation=90)
    ax.text(-1.60, -0.75, "LONGITUDINAL BELT\n2800 × 700 mm", ha="center", va="center", fontsize=8, color="white")
    ax.text(0.30, -1.02, "1 × 5 stack top projection\n600 × 400 mm; 20 mm column gaps", ha="center", va="center", fontsize=7.5)
    base = np.asarray(snapshot["robot"]["world_from_mount"], dtype=float)[:3, 3]
    ax.plot(base[0], base[1], marker="+", ms=9, mew=1.5, color="#B71C1C", zorder=9)
    ax.text(
        base[0], base[1] - 0.08, "base origin (-1410, +350, +600) mm",
        fontsize=7.5, ha="center", va="top", color="#B71C1C", zorder=10,
        bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "none", "pad": 1.0},
    )
    ax.annotate("+X", xy=(-3.02, -0.98), xytext=(-3.42, -0.98), arrowprops={"arrowstyle": "->"}, fontsize=8, va="center")
    ax.annotate("+Y", xy=(-3.42, -0.58), xytext=(-3.42, -0.98), arrowprops={"arrowstyle": "->"}, fontsize=8, ha="center")
    ax.set_xlim(-3.65, 0.75)
    ax.set_ylim(-1.38, 1.38)
    ax.set_xlabel("World X into trailer (m)")
    ax.set_ylabel("World Y left (m)")
    ax.set_title("Top view — fixed assembly and 1×5 stack projection")


def _side(ax, snapshot: dict) -> None:
    _draw_scene(ax, snapshot, (0, 2))
    ax.axhline(0.0, color="#5D4037", lw=1.5)
    ax.axhline(0.6, color="#B71C1C", lw=0.9, ls="--")
    _dimension(ax, (-3.12, 0.0), (-3.12, 0.6), "600 mm common top/surface")
    _dimension(ax, (-0.2, 2.52), (0.0, 2.52), "200 mm")
    ax.text(-2.95, 2.63, "Trailer height is NOT DEFINED", fontsize=8, color="#6A1B9A")
    ax.text(-2.30, 0.24, "CHASSIS", ha="center", va="center", fontsize=8, color="white")
    ax.text(-0.52, 0.63, "belt surface Z=600 mm", ha="center", va="bottom", fontsize=7.5, color="#B71C1C")
    ax.text(0.30, 1.23, "8 layers × 300 mm\nno vertical gap", ha="center", va="center", fontsize=7.5)
    ax.set_xlim(-3.65, 0.75)
    ax.set_ylim(-0.05, 2.85)
    ax.set_xlabel("World X into trailer (m)")
    ax.set_ylabel("World Z up (m)")
    ax.set_title("Side view — 1×8 stack projection; longitudinal belt shown by full projection")


def _front_stack(ax, snapshot: dict) -> None:
    for carton in snapshot["cartons"]:
        _draw_record(ax, carton, (1, 2), COLORS["carton"], 0.68, 3)
    _dimension(ax, (-1.04, 2.52), (1.04, 2.52), "2080 mm overall width")
    _dimension(ax, (1.12, 0.0), (1.12, 2.4), "2400 mm overall height")
    ax.set_xlim(-1.25, 1.35)
    ax.set_ylim(-0.05, 2.65)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("World Y left (m)")
    ax.set_ylabel("World Z up (m)")
    ax.set_title("Independent stack front view — 5 columns × 8 layers")
    ax.grid(True, color="#CFD8DC", linewidth=0.5, alpha=0.7)


def _footer(fig, snapshot: dict) -> None:
    fig.text(
        0.5,
        0.012,
        f"layout={snapshot['layout_id']}  layout fingerprint={snapshot['layout_fingerprint'][:16]}…  "
        "M-710 URDF collision proxies at one validated initial q; tool is a STEP-derived equal-scale engineering envelope. "
        "No motion, load, or complete-cell clearance qualification is claimed.",
        ha="center",
        fontsize=8,
    )


def render(snapshot_path: Path, config_path: Path, output_dir: Path) -> dict:
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    verify_scene_snapshot(snapshot)
    render_cfg = load_layout_validation_config(config_path).data["render"]
    output_dir.mkdir(parents=True, exist_ok=True)
    dpi = int(render_cfg["dpi"])

    fig_a = plt.figure(figsize=render_cfg["figure_a_size_in"], dpi=dpi, constrained_layout=True)
    grid = fig_a.add_gridspec(2, 2, width_ratios=(1.55, 1.0))
    _top(fig_a.add_subplot(grid[:, 0]), snapshot)
    _side(fig_a.add_subplot(grid[0, 1]), snapshot)
    _front_stack(fig_a.add_subplot(grid[1, 1]), snapshot)
    fig_a.suptitle("M-710iD/70 confirmed unloading layout — dimensional constraint sheet", fontsize=15)
    _footer(fig_a, snapshot)
    a_png = output_dir / "m710id70_layout_v1_dimensions.png"
    a_svg = output_dir / "m710id70_layout_v1_dimensions.svg"
    fig_a.savefig(a_png, dpi=dpi)
    fig_a.savefig(a_svg)
    plt.close(fig_a)

    fig_b, axes = plt.subplots(1, 2, figsize=render_cfg["figure_b_size_in"], dpi=dpi, constrained_layout=True)
    _top(axes[0], snapshot)
    _side(axes[1], snapshot)
    fig_b.suptitle("M-710iD/70 confirmed unloading layout — corrected two-view sheet", fontsize=15)
    _footer(fig_b, snapshot)
    b_png = output_dir / "m710id70_layout_v1_two_view.png"
    b_svg = output_dir / "m710id70_layout_v1_two_view.svg"
    fig_b.savefig(b_png, dpi=dpi)
    fig_b.savefig(b_svg)
    plt.close(fig_b)
    result = {
        "status": "PASS",
        "scene_fingerprint": snapshot["scene_fingerprint"],
        "dpi": dpi,
        "png_long_edge_px": int(render_cfg["png_long_edge_px"]),
        "files": [str(path.resolve()) for path in (a_png, a_svg, b_png, b_svg)],
    }
    (output_dir / "layout_render_manifest.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/validation/m710id70_layout_v1.yaml")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/m710id70_layout_v1/figures")
    args = parser.parse_args()
    print(json.dumps(render(args.snapshot, args.config, args.output_dir), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
