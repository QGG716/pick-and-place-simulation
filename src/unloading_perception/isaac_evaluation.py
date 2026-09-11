"""Offline Isaac ground-truth versus vision evaluation.

The evaluator consumes saved artifacts only and never imports Isaac Sim.  It
keeps scale-aligned monocular depth error separate from absolute metric error
and never substitutes oracle geometry for a failed prediction.
"""

from __future__ import annotations

from math import acos, degrees, sqrt
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from unloading_contracts import CargoObservation, PerceptionObservation

from .geometry import transform_pose
from .isaac_validation import canonical_digest, write_json


def bbox_iou(first: Sequence[float], second: Sequence[float]) -> float:
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, float(first[2]) - float(first[0])) * max(0.0, float(first[3]) - float(first[1]))
    second_area = max(0.0, float(second[2]) - float(second[0])) * max(0.0, float(second[3]) - float(second[1]))
    union = first_area + second_area - intersection
    return 0.0 if union <= 0.0 else intersection / union


def mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    left = np.asarray(first, dtype=bool)
    right = np.asarray(second, dtype=bool)
    if left.shape != right.shape or left.ndim != 2:
        raise ValueError("mask IoU requires equal two-dimensional masks")
    union = np.logical_or(left, right)
    return 1.0 if not np.any(union) else float(np.logical_and(left, right).sum() / union.sum())


def mask_tight_bbox(mask: np.ndarray) -> tuple[float, float, float, float] | None:
    value = np.asarray(mask, dtype=bool)
    if value.ndim != 2:
        raise ValueError("mask bbox requires a two-dimensional mask")
    rows, columns = np.nonzero(value)
    if not len(rows):
        return None
    return float(columns.min()), float(rows.min()), float(columns.max() + 1), float(rows.max() + 1)


def _orientation_error_degrees(first: Sequence[float], second: Sequence[float]) -> float:
    left = np.asarray(first, dtype=float)
    right = np.asarray(second, dtype=float)
    if left.shape != (4,) or right.shape != (4,):
        raise ValueError("orientation comparison requires xyzw quaternions")
    dot = float(np.clip(abs(np.dot(left, right)), 0.0, 1.0))
    return degrees(2.0 * acos(dot))


def _proposal_identity(item: CargoObservation) -> str | None:
    raw = item.raw_result
    for name in ("oracle_proposal_source_id", "simulation_object_id", "source_instance_id"):
        value = raw.get(name) if hasattr(raw, "get") else None
        if value:
            return str(value)
    return item.object_id


def match_observations(
    ground_truth: Sequence[CargoObservation],
    predictions: Sequence[CargoObservation],
    *,
    minimum_bbox_iou: float = 0.1,
) -> tuple[tuple[int, int, str], ...]:
    """Match by oracle proposal identity first, then explicit 2D IoU."""

    if not 0.0 <= minimum_bbox_iou <= 1.0:
        raise ValueError("minimum bbox IoU must be in [0, 1]")
    gt_identity = {item.source_instance_id: index for index, item in enumerate(ground_truth)}
    used_gt: set[int] = set()
    used_predictions: set[int] = set()
    matches: list[tuple[int, int, str]] = []
    for prediction_index, prediction in enumerate(predictions):
        identity = _proposal_identity(prediction)
        if identity in gt_identity and gt_identity[identity] not in used_gt:
            ground_truth_index = gt_identity[identity]
            matches.append((ground_truth_index, prediction_index, "ORACLE_PROPOSAL_IDENTITY"))
            used_gt.add(ground_truth_index)
            used_predictions.add(prediction_index)
    candidates = []
    for ground_truth_index, truth in enumerate(ground_truth):
        if ground_truth_index in used_gt:
            continue
        for prediction_index, prediction in enumerate(predictions):
            if prediction_index in used_predictions:
                continue
            score = bbox_iou(truth.bbox_xyxy, prediction.bbox_xyxy)
            if score >= minimum_bbox_iou:
                candidates.append((score, ground_truth_index, prediction_index))
    for score, ground_truth_index, prediction_index in sorted(candidates, reverse=True):
        if ground_truth_index in used_gt or prediction_index in used_predictions:
            continue
        matches.append((ground_truth_index, prediction_index, f"BBOX_IOU:{score:.6f}"))
        used_gt.add(ground_truth_index)
        used_predictions.add(prediction_index)
    return tuple(sorted(matches))


def evaluate_depth(
    ground_truth_depth_m: np.ndarray,
    predicted_depth: np.ndarray,
    *,
    valid_mask: np.ndarray | None = None,
) -> dict[str, Any]:
    """Report both scale-aligned and absolute mismatch without conflation."""

    truth = np.asarray(ground_truth_depth_m, dtype=float)
    prediction = np.asarray(predicted_depth, dtype=float)
    if truth.shape != prediction.shape or truth.ndim != 2:
        raise ValueError("depth arrays must have equal two-dimensional shapes")
    valid = np.isfinite(truth) & np.isfinite(prediction) & (truth > 0.0) & (prediction > 0.0)
    if valid_mask is not None:
        mask = np.asarray(valid_mask, dtype=bool)
        if mask.shape != truth.shape:
            raise ValueError("depth validity mask shape mismatch")
        valid &= mask
    if not np.any(valid):
        return {"status": "NOT_EVALUATED", "valid_pixel_count": 0}
    truth_values = truth[valid]
    prediction_values = prediction[valid]
    denominator = float(np.dot(prediction_values, prediction_values))
    scale = 1.0 if denominator <= 0.0 else float(np.dot(prediction_values, truth_values) / denominator)
    absolute_error = prediction_values - truth_values
    aligned_error = scale * prediction_values - truth_values
    return {
        "status": "PASS",
        "valid_pixel_count": int(valid.sum()),
        "valid_fraction": float(valid.mean()),
        "absolute_metric_mae_m": float(np.mean(np.abs(absolute_error))),
        "absolute_metric_rmse_m": float(sqrt(np.mean(absolute_error ** 2))),
        "scale_alignment_factor": scale,
        "scale_aligned_mae_m": float(np.mean(np.abs(aligned_error))),
        "scale_aligned_rmse_m": float(sqrt(np.mean(aligned_error ** 2))),
        "claim_boundary": "scale-aligned error is not absolute metric accuracy",
    }


def evaluate_observations(
    ground_truth: PerceptionObservation,
    prediction: PerceptionObservation,
    *,
    ground_truth_masks: Mapping[str, np.ndarray] | None = None,
    prediction_masks: Mapping[str, np.ndarray] | None = None,
    depth_evaluation: Mapping[str, Any] | None = None,
    T_W_C: Sequence[Sequence[float]] | None = None,
) -> dict[str, Any]:
    if not ground_truth.synthetic or ground_truth.provider != "isaac-sim-ground-truth":
        raise ValueError("evaluation ground truth must be explicit Isaac synthetic truth")
    if prediction.synthetic:
        raise ValueError("Mode B prediction cannot be replaced by synthetic ground truth")
    if (ground_truth.source_epoch, ground_truth.source_sequence) != (prediction.source_epoch, prediction.source_sequence):
        raise ValueError("prediction and GT must belong to the same historical frame")
    matches = match_observations(ground_truth.cargo, prediction.cargo)
    matched_gt = {first for first, _, _ in matches}
    matched_predictions = {second for _, second, _ in matches}
    per_object = []
    for gt_index, prediction_index, method in matches:
        truth = ground_truth.cargo[gt_index]
        estimate = prediction.cargo[prediction_index]
        center_error = orientation_error = None
        estimate_world_pose = estimate.pose
        transform_applied = False
        if estimate_world_pose is not None and estimate_world_pose.frame_id == "camera_optical_model" and T_W_C is not None:
            estimate_world_pose = transform_pose(T_W_C, estimate_world_pose, "world")
            transform_applied = True
        if truth.pose is not None and estimate_world_pose is not None and truth.pose.frame_id == estimate_world_pose.frame_id == "world":
            center_error = sqrt(sum((left - right) ** 2 for left, right in zip(truth.pose.position_m, estimate_world_pose.position_m)))
            orientation_error = _orientation_error_degrees(truth.pose.orientation_xyzw, estimate_world_pose.orientation_xyzw)
        dimension_error = relative_error = None
        if truth.full_dimensions_m is not None and estimate.full_dimensions_m is not None:
            dimension_error = [abs(left - right) for left, right in zip(truth.full_dimensions_m, estimate.full_dimensions_m)]
            relative_error = [error / left for error, left in zip(dimension_error, truth.full_dimensions_m)]
        masks_status: float | None = None
        estimated_mask_bbox_iou: float | None = None
        if ground_truth_masks is not None and prediction_masks is not None:
            left = ground_truth_masks.get(truth.source_instance_id)
            right = prediction_masks.get(estimate.source_instance_id)
            if left is not None and right is not None:
                masks_status = mask_iou(left, right)
                left_bbox, right_bbox = mask_tight_bbox(left), mask_tight_bbox(right)
                if left_bbox is not None and right_bbox is not None:
                    estimated_mask_bbox_iou = bbox_iou(left_bbox, right_bbox)
        proposal_bbox_iou = bbox_iou(truth.bbox_xyxy, estimate.bbox_xyxy)
        per_object.append({
            "simulation_object_id": truth.source_instance_id,
            "prediction_source_instance_id": estimate.source_instance_id,
            "match_method": method,
            "visibility": "partially_occluded" if truth.occluded else "visible",
            "bbox_iou": proposal_bbox_iou,
            "proposal_bbox_iou": proposal_bbox_iou,
            "estimated_mask_bbox_iou": estimated_mask_bbox_iou,
            "mask_iou": masks_status,
            "center_translation_error_m": center_error,
            "orientation_angular_error_deg": orientation_error,
            "world_transform_applied": transform_applied,
            "full_dimension_abs_error_m": dimension_error,
            "per_axis_dimension_relative_error": relative_error,
            "candidate_eligible": estimate.candidate_eligible,
            "eligibility_reasons": list(estimate.eligibility_reasons),
            "pose_evidence": None if estimate.pose_evidence is None else estimate.pose_evidence.value,
            "size_evidence": None if estimate.size_evidence is None else estimate.size_evidence.value,
            "depth_evidence": None if estimate.depth_evidence is None else estimate.depth_evidence.value,
            "scale_evidence": None if estimate.scale_evidence is None else estimate.scale_evidence.value,
        })

    def mean(field: str) -> float | None:
        values = [float(item[field]) for item in per_object if item[field] is not None]
        return None if not values else float(np.mean(values))

    def mean_vector(field: str) -> list[float] | None:
        vectors = [item[field] for item in per_object if item[field] is not None]
        return None if not vectors else [float(np.mean([vector[axis] for vector in vectors])) for axis in range(3)]

    report = {
        "schema_version": "isaac_perception_evaluation_v1",
        "mode": "ISAAC_SENSOR_WITH_ORACLE_PROPOSALS",
        "proposal_source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
        "raw_image_automatic": False,
        "source_epoch": ground_truth.source_epoch,
        "frame_sequence": ground_truth.source_sequence,
        "capture_time": ground_truth.capture_time,
        "prediction_processed_time": prediction.processed_time,
        "sensor_to_result_latency_s": prediction.processed_time - prediction.capture_time,
        "ground_truth_object_count": len(ground_truth.cargo),
        "prediction_object_count": len(prediction.cargo),
        "processed_object_count": len(matches),
        "missed_object_ids": [item.source_instance_id for index, item in enumerate(ground_truth.cargo) if index not in matched_gt],
        "unmatched_prediction_ids": [item.source_instance_id for index, item in enumerate(prediction.cargo) if index not in matched_predictions],
        "unknown_region_count": len(prediction.unknown_regions),
        "mean_bbox_iou": mean("bbox_iou"),
        "mean_proposal_bbox_iou": mean("proposal_bbox_iou"),
        "mean_estimated_mask_bbox_iou": mean("estimated_mask_bbox_iou"),
        "mean_mask_iou": mean("mask_iou"),
        "mean_center_translation_error_m": mean("center_translation_error_m"),
        "mean_orientation_angular_error_deg": mean("orientation_angular_error_deg"),
        "mean_full_dimension_abs_error_m": mean_vector("full_dimension_abs_error_m"),
        "mean_per_axis_dimension_relative_error": mean_vector("per_axis_dimension_relative_error"),
        "per_object": per_object,
        "depth": dict(depth_evaluation or {"status": "NOT_EVALUATED"}),
        "bbox_claim_boundary": "proposal bbox IoU evaluates injected oracle proposals; estimated mask bbox IoU evaluates the vision output",
        "world_transform_evaluation": {
            "transform": "T_W_object_prediction = T_W_C @ T_C_object_prediction",
            "T_W_C_supplied": T_W_C is not None,
            "prediction_camera_frame": "camera_optical_model",
            "metric_scale_valid": False,
            "claim_boundary": "world-frame errors are absolute mismatches of monocular pseudo-3D and are not metric accuracy",
        },
        "claim_boundary": "Isaac rendered-image evaluation is not real-camera accuracy",
    }
    report["evaluation_fingerprint"] = canonical_digest(report)
    return report


def write_evaluation(path: str | Path, report: Mapping[str, Any]) -> None:
    write_json(path, dict(report))
