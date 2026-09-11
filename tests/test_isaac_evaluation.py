from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from unloading_contracts import EvidenceKind, Pose3D, Validity
from unloading_perception.isaac_evaluation import bbox_iou, evaluate_depth, evaluate_observations, mask_iou, mask_tight_bbox
from unloading_perception.isaac_validation import ground_truth_observation

from test_isaac_perception_contract import IDENTITY, _manifest, _gt


def test_bbox_and_mask_iou_are_exact():
    assert bbox_iou((0, 0, 2, 2), (1, 1, 3, 3)) == pytest.approx(1 / 7)
    first = np.array([[1, 0], [1, 0]], dtype=bool)
    second = np.array([[1, 1], [0, 0]], dtype=bool)
    assert mask_iou(first, second) == pytest.approx(1 / 3)
    assert mask_tight_bbox(first) == (0.0, 0.0, 1.0, 2.0)
    assert mask_tight_bbox(np.zeros((2, 2), dtype=bool)) is None


def test_oracle_proposal_bbox_and_estimated_mask_bbox_are_not_conflated():
    manifest, _ = _manifest()
    truth = _gt(manifest)
    estimate = replace(
        truth.cargo[0], source_instance_id="vision-1", object_id=None, track_id=None,
        raw_result={"oracle_proposal_source_id": truth.cargo[0].source_instance_id},
    )
    prediction = replace(truth, provider="worker", synthetic=False, cargo=(estimate,))
    gt_mask = np.zeros((8, 8), dtype=bool)
    gt_mask[1:5, 1:5] = True
    predicted_mask = np.zeros((8, 8), dtype=bool)
    predicted_mask[2:6, 2:6] = True
    report = evaluate_observations(
        truth, prediction,
        ground_truth_masks={truth.cargo[0].source_instance_id: gt_mask},
        prediction_masks={estimate.source_instance_id: predicted_mask},
    )
    assert report["mean_proposal_bbox_iou"] == 1.0
    assert report["mean_estimated_mask_bbox_iou"] == pytest.approx(9 / 23)
    assert "oracle proposals" in report["bbox_claim_boundary"]


def test_depth_reports_scale_aligned_and_absolute_error_separately():
    truth = np.array([[1.0, 2.0], [3.0, np.inf]])
    prediction = np.array([[0.5, 1.0], [1.5, 4.0]])
    result = evaluate_depth(truth, prediction)
    assert result["absolute_metric_mae_m"] == pytest.approx(1.0)
    assert result["scale_alignment_factor"] == pytest.approx(2.0)
    assert result["scale_aligned_rmse_m"] == pytest.approx(0.0)
    assert "not absolute" in result["claim_boundary"]


def test_evaluation_matches_oracle_proposal_without_upgrading_evidence():
    manifest, _ = _manifest(simulation_frame=50, simulation_time=5.0)
    truth = _gt(manifest)
    source = truth.cargo[0]
    predicted_pose = replace(
        source.pose,
        position_m=(source.pose.position_m[0] + 0.01, *source.pose.position_m[1:]),
        evidence=EvidenceKind.MODEL_ESTIMATED,
    )
    estimate = replace(
        source,
        source_instance_id="vision-1",
        object_id=None,
        track_id=None,
        bbox_xyxy=(102.0, 99.0, 302.0, 299.0),
        pose=predicted_pose,
        pose_evidence=EvidenceKind.MODEL_ESTIMATED,
        size_evidence=EvidenceKind.CONSTRAINT_COMPLETED,
        depth_evidence=EvidenceKind.MODEL_ESTIMATED,
        scale_evidence=EvidenceKind.MODEL_ESTIMATED,
        metric_scale_validity=Validity.VALID,
        geometry_validity=Validity.VALID,
        raw_result={"oracle_proposal_source_id": source.source_instance_id},
    )
    prediction = replace(
        truth,
        observation_id="vision-result",
        provider="cargo-pipeline-worker",
        model_identity="actual-vision-worker",
        synthetic=False,
        processed_time=33.0,
        cargo=(estimate,),
    )
    report = evaluate_observations(truth, prediction)
    assert report["processed_object_count"] == 1
    assert report["per_object"][0]["match_method"] == "ORACLE_PROPOSAL_IDENTITY"
    assert report["per_object"][0]["center_translation_error_m"] == pytest.approx(0.01)
    assert report["per_object"][0]["pose_evidence"] == "MODEL_ESTIMATED"
    assert report["sensor_to_result_latency_s"] == pytest.approx(28.0)
    assert report["raw_image_automatic"] is False


def test_evaluation_rejects_gt_disguised_as_prediction():
    manifest, _ = _manifest()
    truth = _gt(manifest)
    with pytest.raises(ValueError, match="cannot be replaced"):
        evaluate_observations(truth, truth)


def test_evaluation_applies_explicit_camera_to_world_transform_without_validating_scale():
    manifest, _ = _manifest()
    truth = _gt(manifest)
    source = truth.cargo[0]
    camera_pose = Pose3D(
        source.pose.position_m, source.pose.orientation_xyzw,
        "camera_optical_model", source.pose.axis_convention, EvidenceKind.MODEL_ESTIMATED,
    )
    estimate = replace(
        source, source_instance_id="vision-1", object_id=None, track_id=None,
        pose=camera_pose, pose_evidence=EvidenceKind.MODEL_ESTIMATED,
        metric_scale_validity=Validity.UNKNOWN, candidate_eligible=False,
        eligibility_reasons=("MONOCULAR_SCALE_UNVERIFIED",),
        raw_result={"oracle_proposal_source_id": source.source_instance_id},
    )
    prediction = replace(truth, provider="worker", synthetic=False, cargo=(estimate,))
    report = evaluate_observations(truth, prediction, T_W_C=IDENTITY)
    assert report["mean_center_translation_error_m"] == pytest.approx(0.0)
    assert report["mean_orientation_angular_error_deg"] == pytest.approx(0.0)
    assert report["per_object"][0]["world_transform_applied"] is True
    assert report["world_transform_evaluation"]["metric_scale_valid"] is False
