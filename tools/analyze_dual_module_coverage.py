"""Quantify and visualize dual-module front-face frustum coverage."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from math import pi, tan
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Polygon, Rectangle


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/unloading_contracts/src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_perception.coverage import (  # noqa: E402
    carton_front_face,
    derive_stack_bounds,
    evaluate_front_face_coverage,
    project_world_point,
)
from unloading_perception.isaac_validation import IsaacSceneManifest  # noqa: E402


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _camera_with_hfov(camera: dict, hfov_deg: float) -> dict:
    result = deepcopy(camera)
    width, height = result["resolution"]
    fx = width / (2.0 * tan(hfov_deg * pi / 360.0))
    result["K"][0] = fx
    result["K"][2] = (width - 1.0) / 2.0
    result["K"][5] = (height - 1.0) / 2.0
    result["hfov_deg"] = hfov_deg
    return result


def _projected_size(camera: dict, corners: tuple[tuple[float, float, float], ...]) -> tuple[float, float] | None:
    projected = [project_world_point(camera, point) for point in corners]
    pixels = [item["pixel_xy"] for item in projected if item["pixel_xy"] is not None]
    if len(pixels) != 4:
        return None
    return max(item[0] for item in pixels) - min(item[0] for item in pixels), max(item[1] for item in pixels) - min(item[1] for item in pixels)


def _supplement(report: dict, cameras: tuple[dict, ...]) -> dict:
    all_points = [
        point
        for carton in report["cartons"]
        for point in carton["front_face_world_m"]
    ]
    union_margins = []
    for point in all_points:
        margins = [project_world_point(camera, point)["edge_margin_px"] for camera in cameras]
        union_margins.append(max(value for value in margins if value is not None))
    minimum_sizes = []
    for carton in report["cartons"]:
        corners = tuple(tuple(point) for point in carton["front_face_world_m"])
        for camera in cameras:
            projections = [project_world_point(camera, point) for point in corners]
            if all(item["inside"] for item in projections):
                size = _projected_size(camera, corners)
                if size is not None:
                    minimum_sizes.append((camera["module_id"], carton["simulation_object_id"], *size))
    report["minimum_union_image_edge_margin_px"] = min(union_margins)
    if minimum_sizes:
        shortest = min(min(width, height) for _, _, width, height in minimum_sizes)
        report["minimum_fully_observed_face_short_edge_px"] = shortest
    report["uncovered_polygons"] = [
        {
            "simulation_object_id": carton["simulation_object_id"],
            "conservative_front_face_polygon_world_m": carton["front_face_world_m"],
            "uncovered_sample_count": carton["uncovered_sample_count"],
            "note": "polygon marks a face containing uncovered samples; it is not an exact clipped polygon",
        }
        for carton in report["cartons"] if carton["uncovered_sample_count"]
    ]
    return report


def _vertical_overlap(cameras: tuple[dict, ...], stack: dict) -> dict:
    x = sum(stack["front_x_range_m"]) / 2.0
    y = sum(stack["y_range_m"]) / 2.0
    z0, z1 = stack["z_range_m"]
    samples = [z0 + (z1 - z0) * index / 2400.0 for index in range(2401)]
    per_module = {
        camera["module_id"]: [z for z in samples if project_world_point(camera, (x, y, z))["inside"]]
        for camera in cameras
    }
    overlap = [z for z in samples if all(z in values for values in per_module.values())]
    return {
        "probe_world_xy_m": [x, y],
        "per_module_visible_z_m": {
            key: ([min(values), max(values)] if values else None) for key, values in per_module.items()
        },
        "overlap_z_m": [min(overlap), max(overlap)] if overlap else None,
        "overlap_height_m": (max(overlap) - min(overlap)) if overlap else 0.0,
        "sample_step_m": (z1 - z0) / 2400.0,
    }


def _draw_top(path: Path, manifest: IsaacSceneManifest, snapshot: dict) -> None:
    rig = manifest.mechanisms["vision_rig"]
    j1 = rig["T_W_J1"]
    flange = rig["T_W_vision_flange"]
    fig, ax = plt.subplots(figsize=(10, 6), dpi=160)
    ax.scatter([j1[0][3]], [j1[1][3]], s=90, label="J1", color="#1f77b4")
    ax.scatter([flange[0][3]], [flange[1][3]], s=90, label="mast (+Y 0.400 m)", color="#d62728")
    ax.plot([j1[0][3], flange[0][3]], [j1[1][3], flange[1][3]], color="black", linewidth=2)
    stack = derive_stack_bounds(manifest.objects)
    x = sum(stack["front_x_range_m"]) / 2.0
    ax.plot([x, x], stack["y_range_m"], color="#8c564b", linewidth=7, label="R_stack front")
    for y, label in ((snapshot["trailer"]["right_wall_y_m"], "right wall"), (snapshot["trailer"]["left_wall_y_m"], "left wall")):
        ax.axhline(y, linestyle="--", linewidth=1.5, label=label)
    colors = ("#2ca02c", "#ff7f0e")
    for camera, color in zip(manifest.cameras, colors):
        cx, cy = camera["T_W_C"][0][3], camera["T_W_C"][1][3]
        forward = (camera["T_W_C"][0][2], camera["T_W_C"][1][2])
        target_x = x
        scale = (target_x - cx) / forward[0]
        center_y = cy + scale * forward[1]
        hfov = 2.0 * __import__("math").atan(camera["resolution"][0] / (2.0 * camera["K"][0]))
        half = (target_x - cx) * tan(hfov / 2.0)
        ax.add_patch(Polygon([(cx, cy), (target_x, center_y - half), (target_x, center_y + half)], closed=True, alpha=0.14, color=color))
        ax.plot([cx, target_x], [cy, center_y], color=color, label=camera["module_id"])
    ax.set(xlabel="world X into trailer (m)", ylabel="world Y left (m)", title="Dual J1-coupled vision rig — capture-transform top view")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _draw_side(path: Path, manifest: IsaacSceneManifest) -> None:
    rig = manifest.mechanisms["vision_rig"]
    flange = rig["T_W_vision_flange"]
    top = rig["T_W_mast_top"]
    stack = derive_stack_bounds(manifest.objects)
    front_x = sum(stack["front_x_range_m"]) / 2.0
    fig, ax = plt.subplots(figsize=(10, 6), dpi=160)
    ax.plot([flange[0][3], top[0][3]], [flange[2][3], top[2][3]], linewidth=6, color="#7f7f7f", label="1.500 m mast")
    ax.add_patch(Rectangle((front_x - 0.015, stack["z_range_m"][0]), 0.03, stack["z_range_m"][1] - stack["z_range_m"][0], color="#8c564b", alpha=0.55, label="R_stack"))
    colors = ("#2ca02c", "#ff7f0e")
    for camera, color in zip(manifest.cameras, colors):
        cx, cz = camera["T_W_C"][0][3], camera["T_W_C"][2][3]
        fx, fz = camera["T_W_C"][0][2], camera["T_W_C"][2][2]
        scale = (front_x - cx) / fx
        center_z = cz + scale * fz
        vfov = 2.0 * __import__("math").atan(camera["resolution"][1] / (2.0 * camera["K"][4]))
        distance = ((front_x - cx) ** 2 + (center_z - cz) ** 2) ** 0.5
        half = distance * tan(vfov / 2.0)
        ax.add_patch(Polygon([(cx, cz), (front_x, center_z - half), (front_x, center_z + half)], closed=True, alpha=0.14, color=color))
        ax.plot([cx, front_x], [cz, center_z], color=color, label=f"{camera['module_id']} ({camera['optical_depression_rad'] * 180 / pi:.0f}° down)")
        ax.scatter([cx], [cz], color=color, s=60)
    ax.set(xlabel="world X into trailer (m)", ylabel="world Z up (m)", title="Dual module heights and physical optical depression")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _draw_coverage(path: Path, report: dict, cameras: tuple[dict, ...]) -> None:
    fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
    colors = {"upper": "#2ca02c", "lower": "#ff7f0e", "overlap": "#9467bd", "uncovered": "#d62728"}
    stack = report["stack_region"]
    ymin, ymax = stack["y_range_m"]
    zmin, zmax = stack["z_range_m"]
    x = sum(stack["front_x_range_m"]) / 2.0
    ys_grid = [ymin + (ymax - ymin) * index / 320.0 for index in range(321)]
    zs_grid = [zmin + (zmax - zmin) * index / 360.0 for index in range(361)]
    raster = []
    for z in zs_grid:
        row = []
        for y in ys_grid:
            upper, lower = (project_world_point(camera, (x, y, z))["inside"] for camera in cameras)
            row.append(3 if upper and lower else 1 if upper else 2 if lower else 0)
        raster.append(row)
    ax.imshow(
        raster,
        extent=(ymin, ymax, zmin, zmax),
        origin="lower",
        aspect="equal",
        cmap=ListedColormap([colors["uncovered"], colors["upper"], colors["lower"], colors["overlap"]]),
        vmin=0,
        vmax=3,
        interpolation="nearest",
        alpha=0.82,
    )
    for carton in report["cartons"]:
        face = carton["front_face_world_m"]
        ys, zs = [point[1] for point in face], [point[2] for point in face]
        ax.add_patch(Rectangle((min(ys), min(zs)), max(ys) - min(ys), max(zs) - min(zs), fill=False, edgecolor="white", linewidth=1.0))
        ax.text(sum(ys) / 4.0, sum(zs) / 4.0, carton["simulation_object_id"].replace("carton_", ""), ha="center", va="center", fontsize=6)
    handles = [Rectangle((0, 0), 1, 1, color=color, label=label) for label, color in colors.items()]
    ax.legend(handles=handles, loc="upper right")
    ax.set(xlabel="world Y left (m)", ylabel="world Z up (m)", title="Upper/lower full-face coverage from manifest geometry")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _draw_matrix(path: Path, report: dict) -> None:
    ys = sorted({round(item["center_world_m"][1], 9) for item in report["cartons"]})
    zs = sorted({round(item["center_world_m"][2], 9) for item in report["cartons"]}, reverse=True)
    lookup = {(round(item["center_world_m"][1], 9), round(item["center_world_m"][2], 9)): item for item in report["cartons"]}
    matrix = []
    for z in zs:
        row = []
        for y in ys:
            item = lookup[(y, z)]
            upper = item["per_module_sample_coverage"]["module_0_upper"] > 0.999999
            lower = item["per_module_sample_coverage"]["module_1_lower"] > 0.999999
            row.append(3 if upper and lower else 1 if upper else 2 if lower else 0)
        matrix.append(row)
    fig, ax = plt.subplots(figsize=(8, 8), dpi=160)
    image = ax.imshow(matrix, cmap=ListedColormap(["#d62728", "#2ca02c", "#ff7f0e", "#9467bd"]), vmin=0, vmax=3)
    del image
    for row, z in enumerate(zs):
        for column, y in enumerate(ys):
            ax.text(column, row, lookup[(y, z)]["simulation_object_id"].replace("carton_", ""), ha="center", va="center", color="white", fontsize=8)
    ax.set_xticks(range(len(ys)), [f"Y={value:.2f}" for value in ys], rotation=35, ha="right")
    ax.set_yticks(range(len(zs)), [f"Z={value:.2f}" for value in zs])
    ax.set(xlabel="columns derived from world Y", ylabel="layers derived from world Z", title="Per-layer / per-column complete-face coverage")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle-directory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest = IsaacSceneManifest.from_dict(_load(args.bundle_directory / "FULL_STACK_NOMINAL.manifest.json"))
    snapshot = _load(ROOT / "integration/isaac_scene_contract/m710id70_layout_v1/scene_snapshot.json")
    args.output.mkdir(parents=True, exist_ok=True)

    profile_reports = {}
    for hfov in (90.0, 110.0, 115.0, 120.0):
        cameras = tuple(_camera_with_hfov(dict(camera), hfov) for camera in manifest.cameras)
        report = evaluate_front_face_coverage(cameras, manifest.objects, density=33)
        profile_reports[f"dual_{int(hfov)}deg"] = _supplement(report, cameras)
    active = profile_reports["dual_120deg"]
    overlap = _vertical_overlap(tuple(dict(camera) for camera in manifest.cameras), active["stack_region"])
    payload = {
        "schema_version": "dual_module_front_face_coverage_v1",
        "run_id": args.bundle_directory.name,
        "manifest_fingerprint": manifest.manifest_fingerprint,
        "active_profile": "dual_module_full_face_candidate",
        "design_change_from_baseline": "both modules HFOV 90 deg -> 120 deg; VFOV remains 65 deg",
        "profiles": profile_reports,
        "active_vertical_overlap": overlap,
        "R_context": {
            "y_range_m": [snapshot["trailer"]["right_wall_y_m"], snapshot["trailer"]["left_wall_y_m"]],
            "z_range_m": active["stack_region"]["z_range_m"],
            "definition": "finite context at the front plane spanning stack height and both wall boundaries",
        },
        "rendered_visibility_status": "PENDING_ISAAC_SEMANTIC_CAPTURE",
        "status": "FRUSTUM_PASS" if active["union_sample_coverage"] == 1.0 else "FRUSTUM_FAIL",
    }
    (args.output / "dual_module_coverage.json").write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    _draw_top(args.output / "01_dual_rig_top_view.png", manifest, snapshot)
    _draw_side(args.output / "02_dual_rig_side_view.png", manifest)
    _draw_coverage(args.output / "05_upper_lower_coverage.png", active, tuple(dict(camera) for camera in manifest.cameras))
    _draw_matrix(args.output / "06_layer_column_coverage.png", active)
    print(json.dumps({
        "status": payload["status"],
        "baseline_union": profile_reports["dual_90deg"]["union_sample_coverage"],
        "active_union": active["union_sample_coverage"],
        "active_full_cartons": active["fully_covered_carton_count"],
        "vertical_overlap_m": overlap["overlap_height_m"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
