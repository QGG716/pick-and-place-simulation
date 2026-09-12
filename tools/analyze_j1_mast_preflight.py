"""Audit J1-coupled mast kinematics and analytic carton FOV coverage.

This is a deterministic CPU preflight.  It does not claim occlusion results;
Isaac semantic renders remain the authority for robot/mast/trailer occlusion.
"""

from __future__ import annotations

import argparse
import json
from math import pi
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_perception.isaac_validation import IsaacSceneManifest, write_json  # noqa: E402
from unloading_perception.vision_rig import audit_j1_sweep, load_vision_rig_spec  # noqa: E402


def _corners(item: dict) -> np.ndarray:
    transform = np.asarray(item["T_W_object"], dtype=float)
    half = np.asarray(item["full_dimensions_m"], dtype=float) / 2.0
    local = np.asarray([
        [sx * half[0], sy * half[1], sz * half[2], 1.0]
        for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)
    ])
    return (transform @ local.T).T[:, :3]


def _coverage(manifest: IsaacSceneManifest, *, camera_height_delta_m: float = 0.0) -> dict:
    camera = manifest.cameras[0]
    T_W_C = np.asarray(camera["T_W_C"], dtype=float).copy()
    T_W_C[2, 3] += camera_height_delta_m
    T_C_W = np.linalg.inv(T_W_C)
    K = np.asarray(camera["K"], dtype=float).reshape(3, 3)
    width, height = (int(value) for value in camera["resolution"])
    rows = []
    for item in manifest.objects:
        points_w = np.vstack((_corners(item), np.asarray(item["T_W_object"], dtype=float)[:3, 3]))
        points_c = (T_C_W[:3, :3] @ points_w.T).T + T_C_W[:3, 3]
        positive = points_c[:, 2] > 1e-9
        pixels = np.full((len(points_c), 2), np.nan)
        pixels[positive, 0] = K[0, 0] * points_c[positive, 0] / points_c[positive, 2] + K[0, 2]
        pixels[positive, 1] = K[1, 1] * points_c[positive, 1] / points_c[positive, 2] + K[1, 2]
        inside = positive & (pixels[:, 0] >= 0.0) & (pixels[:, 0] < width) & (pixels[:, 1] >= 0.0) & (pixels[:, 1] < height)
        rows.append({
            "simulation_object_id": item["simulation_object_id"],
            "center_world_z_m": float(points_w[-1, 2]),
            "center_in_fov": bool(inside[-1]),
            "corner_count_in_fov": int(np.count_nonzero(inside[:-1])),
            "projection_intersects_image": bool(np.any(inside)),
            "fully_inside_fov": bool(np.all(inside)),
            "occlusion_status": "NOT_EVALUATED_BY_ANALYTIC_PREFLIGHT",
        })
    visible = sum(item["projection_intersects_image"] for item in rows)
    return {
        "camera_height_delta_m": camera_height_delta_m,
        "carton_count": len(rows),
        "projection_intersection_count": visible,
        "projection_coverage_fraction": visible / max(1, len(rows)),
        "fully_inside_count": sum(item["fully_inside_fov"] for item in rows),
        "cartons": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle-directory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    index = json.loads((args.bundle_directory / "index.json").read_text(encoding="utf-8"))
    manifests = {
        record["scene"]: IsaacSceneManifest.from_dict(
            json.loads((args.bundle_directory / record["path"]).read_text(encoding="utf-8"))
        ) for record in index["scenes"]
    }
    nominal = manifests["FULL_STACK_NOMINAL"]
    spec = load_vision_rig_spec(ROOT / "configs/isaac/perception_sensing_pose.yaml")
    T_W_robot = nominal.robot["T_W_robot"]
    sweep = audit_j1_sweep(T_W_robot, (-pi / 2.0, -pi / 4.0, 0.0, pi / 4.0, pi / 2.0), spec)
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output / "j1_mast_camera_kinematics.json", {
        **sweep,
        "nominal_q1_rad": spec.nominal_q1_rad,
        "physical_relation": "camera optical forward and J1 arm heading differ by +90 degrees",
    })
    baseline = _coverage(nominal)
    alternatives = [_coverage(nominal, camera_height_delta_m=delta) for delta in (-0.85, -0.60, -0.35)]
    baseline_bottom = sum(
        item["projection_intersects_image"] for item in baseline["cartons"]
        if item["center_world_z_m"] <= np.percentile([row["center_world_z_m"] for row in baseline["cartons"]], 33.0)
    )
    best = max(alternatives, key=lambda item: item["projection_intersection_count"])
    write_json(args.output / "fov_carton_coverage_analytic.json", {
        "schema_version": "analytic_carton_fov_v1",
        "status": "PASS",
        "claim_boundary": "frustum projection only; Isaac semantic renders determine occlusion",
        **baseline,
    })
    write_json(args.output / "second_module_need_study.json", {
        "schema_version": "second_module_need_study_v1",
        "status": "STUDY_ONLY_NO_SECOND_MODULE_INSTALLED",
        "baseline_module_height_m": spec.module_height_m,
        "baseline_bottom_tertile_projection_count": baseline_bottom,
        "candidate_lower_module_results": alternatives,
        "best_candidate_height_m": spec.module_height_m + best["camera_height_delta_m"],
        "recommendation": "DEFER_HARDWARE_DECISION_UNTIL_ISAAC_OCCLUSION_AND_REAL_SENSOR_TRIAL",
    })
    print(json.dumps({"status": "PASS", "coverage": baseline["projection_coverage_fraction"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
