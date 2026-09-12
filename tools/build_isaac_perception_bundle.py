"""Build deterministic Isaac perception scene manifests from a frozen layout."""

from __future__ import annotations

import argparse
import json
from math import pi
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/unloading_contracts/src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_perception.isaac_validation import (  # noqa: E402
    build_scene_manifest,
    load_validation_config,
    write_json,
)


def _git_head() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True,
        text=True, capture_output=True, timeout=10,
    ).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/isaac/perception_validation.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--perception-commit", default=None)
    parser.add_argument("--isaac-version", default="6.0.1.0")
    parser.add_argument("--run-id", default="isaac-perception-validation")
    args = parser.parse_args()

    config = load_validation_config(args.config)
    snapshot_path = ROOT / config["layout_bundle"]["snapshot"]
    contract_path = ROOT / config["layout_bundle"]["isaac_contract"]
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    target = str(config["target_selection"]["simulation_object_id"])
    single_box = {
        item["name"]: ({"visible": True, "state": "RGBD_CALIBRATION_TARGET"} if item["name"] == target else {"visible": False, "state": "CALIBRATION_SCENE_HIDDEN"})
        for item in snapshot["cartons"]
    }
    scene_specs = {
        "MECHANICAL_TOP_VIEW": ({}, 0, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_ON_NOMINAL"),
        "J1_ROTATION_SWEEP": ({}, 50, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_ON_NOMINAL"),
        "FULL_STACK_NOMINAL": ({}, 100, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_ON_NOMINAL"),
        "DARK_LIGHT_OFF": ({}, 150, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_OFF"),
        "DARK_LIGHT_ON": ({}, 200, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_ON_NOMINAL"),
        "RGBD_CALIBRATION_BOX": (single_box, 250, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_ON_NOMINAL"),
        "PARTIAL_OCCLUSION": ({target: {"occluded": True, "state": "PARTIAL_OCCLUSION"}}, 300, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_ON_NOMINAL"),
        "MOTION_TIMESTAMP_TEST": ({}, 350, "sim-j1-mast-v2", -pi / 2.0, "LIGHT_ON_NOMINAL"),
    }

    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    perception_commit = args.perception_commit or _git_head()
    for scene_name in config["scenes"]:
        overrides, frame, epoch, q1, illumination = scene_specs[scene_name]
        manifest = build_scene_manifest(
            snapshot, contract, config,
            run_id=f"{args.run_id}-{scene_name}",
            perception_commit=perception_commit,
            feasibility_reference_commit=config["layout_bundle"]["feasibility_reference_commit"],
            isaac_version=args.isaac_version,
            simulation_epoch=epoch,
            simulation_frame=frame,
            simulation_time=frame / float(config["rendering"]["physics_hz"]),
            q1_at_capture_rad=q1,
            object_overrides=overrides,
        )
        path = args.output / f"{scene_name}.manifest.json"
        write_json(path, manifest.to_dict())
        records.append({
            "scene": scene_name,
            "path": path.name,
            "manifest_fingerprint": manifest.manifest_fingerprint,
            "dynamic_scene_fingerprint": manifest.dynamic_scene_fingerprint,
            "world_fingerprint": manifest.world_fingerprint,
            "simulation_epoch": epoch,
            "simulation_frame": frame,
            "q1_at_capture_rad": q1,
            "illumination_state": illumination,
            "inference_time_q1_rad": 0.0 if scene_name == "MOTION_TIMESTAMP_TEST" else q1,
        })
    index = {
        "schema_version": "isaac_perception_scene_bundle_v2",
        "run_id": args.run_id,
        "perception_commit": perception_commit,
        "feasibility_reference_commit": config["layout_bundle"]["feasibility_reference_commit"],
        "layout_id": snapshot["layout_id"],
        "layout_fingerprint": snapshot["layout_fingerprint"],
        "source_scene_fingerprint": snapshot["scene_fingerprint"],
        "raw_image_automatic": False,
        "primary_mode": "STAGED_RGBD",
        "comparison_mode": "STAGED_MONOCULAR_MOGE",
        "rig_id": config["rig_id"],
        "scenes": records,
    }
    write_json(args.output / "index.json", index)
    print(json.dumps(index, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
