"""Build deterministic Isaac perception scene manifests from a frozen layout."""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from math import cos, sin
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


def _translated(pose, offset):
    result = deepcopy(pose)
    for index in range(3):
        result[index][3] = float(result[index][3]) + float(offset[index])
    return result


def _rotated_z(pose, radians):
    result = deepcopy(pose)
    rotation = [[cos(radians), -sin(radians), 0.0], [sin(radians), cos(radians), 0.0], [0.0, 0.0, 1.0]]
    source = [row[:3] for row in pose[:3]]
    combined = [[sum(source[row][k] * rotation[k][column] for k in range(3)) for column in range(3)] for row in range(3)]
    for row in range(3):
        result[row][:3] = combined[row]
    return result


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
    source_pose = next(item["pose_world"] for item in snapshot["cartons"] if item["name"] == target)
    perturbations = config["perturbations"]
    scene_specs = {
        "static_full_stack": ({}, config, 0, "sim-epoch-a"),
        "single_target_focus": ({}, config, 50, "sim-epoch-a"),
        "known_translation": ({target: {"T_W_object": _translated(source_pose, perturbations["known_translation_m"]), "state": "KNOWN_TRANSLATION"}}, config, 100, "sim-epoch-a"),
        "known_rotation": ({target: {"T_W_object": _rotated_z(source_pose, float(perturbations["known_rotation_rpy_rad"][2])), "state": "KNOWN_ROTATION"}}, config, 150, "sim-epoch-a"),
        "partial_occlusion": ({target: {"occluded": True, "state": "PARTIAL_OCCLUSION"}}, config, 200, "sim-epoch-a"),
    }
    changed_camera = deepcopy(config)
    camera = changed_camera["cameras"][0]
    camera["T_W_C"] = _translated(camera["T_W_C"], perturbations["camera_translation_m"])
    camera["look_at_world_m"] = [float(value) + float(offset) for value, offset in zip(camera["look_at_world_m"], perturbations["camera_translation_m"])]
    scene_specs["camera_transform_change"] = ({}, changed_camera, 0, "sim-epoch-camera-b")

    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    perception_commit = args.perception_commit or _git_head()
    for scene_name in config["scenes"]:
        overrides, scene_config, frame, epoch = scene_specs[scene_name]
        manifest = build_scene_manifest(
            snapshot, contract, scene_config,
            run_id=f"{args.run_id}-{scene_name}",
            perception_commit=perception_commit,
            feasibility_reference_commit=config["layout_bundle"]["feasibility_reference_commit"],
            isaac_version=args.isaac_version,
            simulation_epoch=epoch,
            simulation_frame=frame,
            simulation_time=frame / float(config["rendering"]["physics_hz"]),
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
        })
    index = {
        "schema_version": "isaac_perception_scene_bundle_v1",
        "run_id": args.run_id,
        "perception_commit": perception_commit,
        "feasibility_reference_commit": config["layout_bundle"]["feasibility_reference_commit"],
        "layout_id": snapshot["layout_id"],
        "layout_fingerprint": snapshot["layout_fingerprint"],
        "source_scene_fingerprint": snapshot["scene_fingerprint"],
        "raw_image_automatic": False,
        "scenes": records,
    }
    write_json(args.output / "index.json", index)
    print(json.dumps(index, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
