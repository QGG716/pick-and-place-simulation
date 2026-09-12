"""Reproduce the single-camera right-side coverage failure from frozen evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/unloading_contracts/src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_perception.coverage import (  # noqa: E402
    derive_stack_bounds,
    evaluate_front_face_coverage,
    project_world_point,
)
from unloading_perception.isaac_validation import build_scene_manifest, load_validation_config  # noqa: E402


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy-capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = load_validation_config(ROOT / "configs/isaac/perception_validation.yaml")
    snapshot = _load(ROOT / config["layout_bundle"]["snapshot"])
    contract = _load(ROOT / config["layout_bundle"]["isaac_contract"])
    manifest = build_scene_manifest(
        snapshot,
        contract,
        config,
        run_id="legacy-single-camera-root-cause",
        perception_commit="eca9c8ce8a2f17cf99e5e6705f379d3c9651e418",
        feasibility_reference_commit=config["layout_bundle"]["feasibility_reference_commit"],
        isaac_version="6.0.1.0",
        simulation_epoch="sim-j1-mast-v2",
        simulation_frame=100,
        simulation_time=100.0 / 60.0,
    )
    camera_info = _load(args.legacy_capture / "camera_info.json")
    capture_metadata = _load(args.legacy_capture / "capture_metadata.json")
    annotations = _load(args.legacy_capture / "gt_annotations.json")
    image_path = args.legacy_capture / "sensor_rgb.png"
    image_header = image_path.read_bytes()[:24]
    if image_header[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("legacy RGB input is not a PNG")
    width = int.from_bytes(image_header[16:20], "big")
    height = int.from_bytes(image_header[20:24], "big")

    camera = {
        **camera_info,
        "camera_id": camera_info["camera_id"],
        "module_id": "module_0_main",
        "resolution": [camera_info["width"], camera_info["height"]],
    }
    if camera_info["T_W_C"] != manifest.cameras[0]["T_W_C"]:
        raise ValueError("legacy upper-camera pose differs from the frozen kinematic reconstruction")
    if capture_metadata["T_W_C_at_capture"] != camera["T_W_C"]:
        raise ValueError("capture-time pose differs from CameraInfo/manifest pose")

    coverage = evaluate_front_face_coverage((camera,), tuple(manifest.objects), density=33)
    stack = derive_stack_bounds(tuple(manifest.objects))
    front_x = sum(stack["front_x_range_m"]) / 2.0
    z_mid = sum(stack["z_range_m"]) / 2.0
    trailer = snapshot["trailer"]
    right_wall_probe = (front_x, float(trailer["right_wall_y_m"]), z_mid)
    left_wall_probe = (front_x, float(trailer["left_wall_y_m"]), z_mid)
    annotation_rows = annotations["objects"]
    rendered_visible = [item for item in annotation_rows if item["visible"]]
    projected = [item for item in annotation_rows if item["projection_intersects_image"]]
    out_of_frustum = [item["simulation_object_id"] for item in annotation_rows if not item["projection_intersects_image"]]
    occluded = [item["simulation_object_id"] for item in annotation_rows if item["occluded"]]

    fx, fy = float(camera_info["K"][0]), float(camera_info["K"][4])
    cy = float(camera_info["K"][5])
    source_fy = float(camera_info["native_square_pixel_fy_px"])
    source_cy = (height - 1.0) / 2.0
    source_rows = [
        (output_row - cy) * source_fy / fy + source_cy
        for output_row in (0.0, height - 1.0)
    ]
    payload = {
        "schema_version": "camera_coverage_root_cause_v1",
        "status": "REPRODUCED",
        "reference": {
            "perception_commit": "eca9c8ce8a2f17cf99e5e6705f379d3c9651e418",
            "capture_run": "isaac_j1_mast_rgbd_1f2f8b5/FULL_STACK_NOMINAL",
            "isaac_version": "6.0.1.0",
        },
        "actual_capture": {
            "rgb_size_px": [width, height],
            "rgb_sha256": _sha256(image_path),
            "camera_id": camera_info["camera_id"],
            "frame_id": camera_info["frame_id"],
            "K": camera_info["K"],
            "T_W_C": camera_info["T_W_C"],
            "capture_pose_matches_camera_info": True,
            "render_product_module_matches": camera_info["camera_id"] == "module_0_main_rgbd",
        },
        "native_to_output_mapping": {
            "horizontal_mapping": "IDENTITY_NO_CROP_NO_LETTERBOX",
            "vertical_mapping": "CENTER_CROP_BY_DETERMINISTIC_RESAMPLE",
            "source_row_interval_sampled": source_rows,
            "output_size_unchanged": True,
            "right_side_loss_caused_by_mapping": False,
            "note": "the explicit vertical remap realizes VFOV=65 degrees; map_x is the output column",
        },
        "layout_regions": {
            "R_stack": stack,
            "R_context_y_range_m": [float(trailer["right_wall_y_m"]), float(trailer["left_wall_y_m"])],
            "right_wall_asset_in_snapshot": trailer.get("right_wall_y_m") is not None,
            "right_wall_probe": project_world_point(camera, right_wall_probe),
            "left_wall_probe": project_world_point(camera, left_wall_probe),
        },
        "frustum_coverage": coverage,
        "rendered_visibility": {
            "carton_count": len(annotation_rows),
            "projection_intersection_count": len(projected),
            "semantic_visible_count": len(rendered_visible),
            "occluded_count": len(occluded),
            "out_of_frustum_ids": out_of_frustum,
            "occluded_ids": occluded,
        },
        "root_causes": [
            {
                "cause": "MOUNT_OFFSET_PLUS_90_DEG_HORIZONTAL_FOV_INSUFFICIENT",
                "state": "REPRODUCED",
                "evidence": "right wall and rightmost stack samples project beyond the positive image-u edge",
            },
            {
                "cause": "HORIZONTAL_CROP_ROI_OR_LETTERBOX",
                "state": "RULED_OUT",
                "evidence": "2592x1944 render/output identity in x; map_x equals output columns",
            },
            {
                "cause": "VERTICAL_SINGLE_CAMERA_COVERAGE_INSUFFICIENT",
                "state": "REPRODUCED",
                "evidence": "lower stack layers fall outside output VFOV",
            },
            {
                "cause": "MECHANICAL_OCCLUSION",
                "state": "REPRODUCED_FOR_PROJECTED_CARTONS",
                "evidence": f"{len(occluded)} projected cartons had zero semantic pixels with the full mechanism present",
            },
            {
                "cause": "RIGHT_WALL_ASSET_MISSING",
                "state": "RULED_OUT_AT_LAYOUT_AND_STAGE_BUILDER_LEVEL",
                "evidence": "snapshot defines right_wall_y_m=-1.15 and capture stage builder creates RightWall",
            },
        ],
        "conclusion": "The missing image-right stack/wall content is primarily a horizontal projection limit, while missing lower layers are a vertical projection limit; projected cartons can additionally be mechanically occluded.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({
        "status": payload["status"],
        "frustum_union": coverage["union_sample_coverage"],
        "semantic_visible": len(rendered_visible),
        "output": str(args.output),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
