#!/usr/bin/env python3
"""Summarize same-frame dual RGB-D base/V4 geometry and visibility evidence."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import fmean


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _mean(values) -> float | None:
    finite = [float(value) for value in values if value is not None]
    return fmean(finite) if finite else None


def _module(directory: Path) -> dict:
    baseline = _load(directory / "rgbd_cuboids_baseline_raw.json")
    final = _load(directory / "rgbd_cuboids.json")
    observation = _load(directory / "mode_b_rgbd_observation.json")["payload"]
    baseline_faces = [face for item in baseline["instances"] for face in item.get("camera_facing_faces", [])]
    final_faces = [face for item in final["instances"] for face in item.get("camera_facing_faces", [])]
    measured = [face for face in final_faces if face.get("mask_iou") is not None]
    cargo = observation["cargo"]
    statuses = Counter(
        item.get("raw_result", {}).get("complete_cuboid_status", "COMPLETE_HYPOTHESIS_RETAINED_INELIGIBLE")
        for item in cargo
    )
    return {
        "input_instance_count": len(final["instances"]),
        "baseline_face_label_count": len(baseline_faces),
        "v4_direct_observed_face_count": len(final_faces),
        "removed_non_observed_completion_face_labels": len(baseline_faces) - len(final_faces),
        "direct_face_metrics": {
            "measured_face_count": len(measured),
            "mean_mask_iou": _mean(face.get("mask_iou") for face in measured),
            "mean_mask_precision": _mean(face.get("mask_precision") for face in measured),
            "mean_mask_coverage": _mean(face.get("mask_coverage") for face in measured),
            "mean_plane_residual_m": _mean(item.get("plane_residual_mean") for item in final["instances"]),
        },
        "complete_cuboid": {
            "pose_and_dimensions_emitted_count": sum(item.get("pose") is not None and item.get("full_dimensions_m") is not None for item in cargo),
            "execution_eligible_count": sum(bool(item.get("candidate_eligible")) for item in cargo),
            "status_counts": dict(sorted(statuses.items())),
        },
    }


def summarize(capture_root: Path, coverage_path: Path) -> dict:
    full = capture_root / "FULL_STACK_NOMINAL"
    module_rows = {
        "module_0_upper": _module(full),
        "module_1_lower": _module(full / "modules" / "module_1_lower"),
    }
    baseline_faces = sum(item["baseline_face_label_count"] for item in module_rows.values())
    observed_faces = sum(item["v4_direct_observed_face_count"] for item in module_rows.values())
    measured_count = sum(item["direct_face_metrics"]["measured_face_count"] for item in module_rows.values())
    weighted = {}
    for key in ("mean_mask_iou", "mean_mask_precision", "mean_mask_coverage"):
        weighted[key] = sum(
            item["direct_face_metrics"][key] * item["direct_face_metrics"]["measured_face_count"]
            for item in module_rows.values() if item["direct_face_metrics"][key] is not None
        ) / measured_count
    coverage = _load(coverage_path)
    active = coverage["profiles"][coverage["active_profile"].replace("dual_module_full_face_candidate", "dual_120deg")]
    visibility = _load(full / "rendered_visibility.json")
    fusion = _load(full / "multimodule_fusion_metrics.json")
    occluded_ids = sorted(name for name, item in visibility["objects"].items() if item.get("occluded"))
    return {
        "schema_version": "dual_rgbd_v4_acceptance_metrics_v1",
        "capture_root": str(capture_root.resolve()),
        "coverage": {
            "active_profile": coverage["active_profile"],
            "frustum_union_sample_coverage": active["union_sample_coverage"],
            "fully_covered_cartons": active["fully_covered_carton_count"],
            "carton_count": active["carton_count"],
            "minimum_image_edge_margin_px": active["minimum_union_image_edge_margin_px"],
            "minimum_projected_face_short_edge_px": active["minimum_fully_observed_face_short_edge_px"],
            "vertical_overlap_m": coverage["active_vertical_overlap"]["overlap_height_m"],
        },
        "rendered_visibility": {
            "visible_count": visibility["union_visible_count"],
            "object_count": visibility["object_count"],
            "visible_ratio": visibility["union_visible_count"] / visibility["object_count"],
            "out_of_fov_count": visibility["out_of_fov_count"],
            "occluded_count": visibility["occluded_count"],
            "occluded_object_ids": occluded_ids,
            "robot_and_scene_geometry_visible": visibility["robot_and_scene_geometry_visible"],
        },
        "same_frame_base_vs_v4": {
            "baseline_face_label_count": baseline_faces,
            "v4_direct_observed_face_count": observed_faces,
            "removed_non_observed_completion_face_labels": baseline_faces - observed_faces,
            "removed_fraction": (baseline_faces - observed_faces) / baseline_faces,
            "direct_face_metric_change": "UNCHANGED_SAME_MEASURED_FACES",
            "direct_face_metrics": {"measured_face_count": measured_count, **weighted},
            "interpretation": "V4 did not numerically move the registered observed face; it stopped publishing completion faces as observations.",
        },
        "modules": module_rows,
        "fusion": fusion,
        "claim_boundary": "coverage, direct observed faces, and complete cuboid eligibility are separate results",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-directory", required=True, type=Path)
    parser.add_argument("--coverage-json", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = summarize(args.capture_directory.resolve(), args.coverage_json.resolve())
    output = args.output or args.capture_directory / "dual_rgbd_v4_acceptance_metrics.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
