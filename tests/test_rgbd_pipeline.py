from __future__ import annotations

from math import pi

import numpy as np
import pytest

from unloading_contracts import Validity
from unloading_perception.rgbd import (
    CaptureManager,
    CaptureRequest,
    IlluminationState,
    MetricPointMap,
    MetricPointMapSource,
    PointCloudFilterConfig,
    hypotheses_from_geometry_record,
    masked_metric_pointmap,
    register_rgbd,
    transform_hypothesis_to_world,
)
from unloading_perception.vision_rig import evaluate_vision_rig_pose, load_vision_rig_spec


IDENTITY = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
K = (100.0, 0.0, 2.5, 0.0, 100.0, 2.5, 0.0, 0.0, 1.0)


def _metadata(q1=-pi / 2.0):
    spec = load_vision_rig_spec("configs/isaac/perception_sensing_pose.yaml")
    base = ((1.0, 0.0, 0.0, -1.325), (0.0, 1.0, 0.0, 0.35), (0.0, 0.0, 1.0, 0.6), (0.0, 0.0, 0.0, 1.0))
    pose = evaluate_vision_rig_pose(base, q1, spec)
    request = CaptureRequest("request-1", "module_0_main", 1.0, "ros_sim_time", IlluminationState.LIGHT_ON_NOMINAL)
    return CaptureManager("epoch-a").bind(
        request, capture_start=1.001, capture_center_time=1.002, capture_end=1.003,
        rgb_frame_id="module_0_main_rgb_optical", depth_frame_id="module_0_main_depth_optical",
        calibration_identity="calib-a", j1_state_identity=f"j1:{q1}", q1_at_capture_rad=q1,
        T_W_C_at_capture=pose.T_W_camera_optical,
    )


def _frame(metadata=None):
    metadata = metadata or _metadata()
    rgb = np.zeros((6, 6, 3), dtype=np.uint8)
    depth = np.full((6, 6), 2.0, dtype=np.float32)
    return register_rgbd(
        metadata=metadata, rgb=rgb, depth_optical_z_m=depth, rgb_K=K, depth_K=K,
        T_rgb_depth=IDENTITY, rgb_calibration_identity="calib-a", depth_calibration_identity="calib-a",
        registration_mode="SIMULATION_IDEAL_REGISTERED_DEPTH",
    )


def test_rgb_depth_lights_and_j1_pose_share_one_capture_identity():
    metadata = _metadata()
    assert metadata.capture_id == "epoch-a:module_0_main:0"
    assert metadata.rgb_frame_id and metadata.depth_frame_id
    assert metadata.illumination_state is IlluminationState.LIGHT_ON_NOMINAL
    assert metadata.q1_at_capture_rad == pytest.approx(-pi / 2.0)
    assert metadata.capture_start <= metadata.capture_center_time <= metadata.capture_end


def test_registration_fails_closed_on_sync_calibration_resolution_or_K():
    metadata = _metadata()
    with pytest.raises(ValueError, match="calibration identities"):
        register_rgbd(metadata=metadata, rgb=np.zeros((6, 6, 3)), depth_optical_z_m=np.ones((6, 6)), rgb_K=K, depth_K=K, T_rgb_depth=IDENTITY, rgb_calibration_identity="a", depth_calibration_identity="b", registration_mode="SIMULATION_IDEAL_REGISTERED_DEPTH")
    with pytest.raises(ValueError, match="resolutions"):
        register_rgbd(metadata=metadata, rgb=np.zeros((6, 6, 3)), depth_optical_z_m=np.ones((5, 6)), rgb_K=K, depth_K=K, T_rgb_depth=IDENTITY, rgb_calibration_identity="calib-a", depth_calibration_identity="calib-a", registration_mode="SIMULATION_IDEAL_REGISTERED_DEPTH")
    bad_k = list(K)
    bad_k[0] += 1.0
    with pytest.raises(ValueError, match="intrinsics"):
        register_rgbd(metadata=metadata, rgb=np.zeros((6, 6, 3)), depth_optical_z_m=np.ones((6, 6)), rgb_K=K, depth_K=bad_k, T_rgb_depth=IDENTITY, rgb_calibration_identity="calib-a", depth_calibration_identity="calib-a", registration_mode="SIMULATION_IDEAL_REGISTERED_DEPTH")


def test_registered_depth_lifts_optical_z_to_metric_xyz_without_gt_cleanup():
    mask = np.ones((6, 6), dtype=bool)
    pointmap = masked_metric_pointmap(
        _frame(), mask, depth_identity="depth-a",
        config=PointCloudFilterConfig(boundary_erosion_px=1, minimum_points=10),
    )
    assert pointmap.source is MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH
    assert pointmap.filter_evidence["depth_semantics"] == "optical_z_m"
    assert pointmap.valid_mask.sum() == 16
    assert pointmap.points_camera_xyz_m[3, 3] == pytest.approx((0.01, 0.01, 2.0))
    assert pointmap.filter_evidence["metric_scale_validity"] == Validity.VALID.value


def test_metric_pointmap_keeps_registered_depth_provenance(tmp_path):
    pointmap = masked_metric_pointmap(
        _frame(), np.ones((6, 6), dtype=bool), depth_identity="depth-a",
        config=PointCloudFilterConfig(boundary_erosion_px=0, minimum_points=10),
    )
    destination = tmp_path / "pointmap.npz"
    pointmap.write_npz(destination)
    payload = np.load(destination)
    metadata = __import__("json").loads(str(payload["metadata_json"]))
    assert metadata["source"] == "ISAAC_IDEAL_REGISTERED_DEPTH"
    assert "moge" not in metadata["source"].lower()
    assert payload["K"].shape == (3, 3)
    assert payload["intrinsics"].shape == (3, 3)


def test_monocular_source_cannot_claim_verified_metric_scale():
    template = masked_metric_pointmap(
        _frame(), np.ones((6, 6), dtype=bool), depth_identity="depth-a",
        config=PointCloudFilterConfig(boundary_erosion_px=0, minimum_points=10),
    )
    with pytest.raises(ValueError, match="cannot claim"):
        MetricPointMap(**{**template.__dict__, "source": MetricPointMapSource.MOGE_MONOCULAR_ESTIMATE})


def test_single_visible_face_emits_multiple_ineligible_hypotheses():
    record = {
        "accepted": True,
        "orthogonal_axes_3d": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "corners_3d": [[-0.3, -0.2, 1.8], [0.3, -0.2, 1.8], [0.3, 0.2, 1.8], [-0.3, 0.2, 1.8], [-0.3, -0.2, 2.1], [0.3, -0.2, 2.1], [0.3, 0.2, 2.1], [-0.3, 0.2, 2.1]],
        "shape_dimensions": [0.6, 0.4, 0.3],
        "depth_supported_face_count": 1,
        "plane_inlier_ratio": 0.8,
        "plane_residual_mean": 0.002,
        "completion_mode": "single_visible_plane_cuboid_hypothesis",
        "camera_facing_faces": [{"face_id": "front"}],
        "method": "legacy_name_is_provenance_only",
    }
    hypotheses = hypotheses_from_geometry_record(
        record, source_instance_id="box-1",
        pointmap_source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH, presence_score=0.9,
    )
    assert len(hypotheses) == 2
    assert [item.hypothesis_rank for item in hypotheses] == [0, 1]
    assert all(not item.candidate_eligible for item in hypotheses)
    assert all(item.evidence["pointmap_source"] == "ISAAC_IDEAL_REGISTERED_DEPTH" for item in hypotheses)


def test_world_transform_uses_capture_time_pose_not_inference_time_j1():
    record = {
        "accepted": True,
        "orthogonal_axes_3d": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "corners_3d": [[-0.1, -0.1, 1.9], [0.1, -0.1, 1.9], [0.1, 0.1, 1.9], [-0.1, 0.1, 1.9], [-0.1, -0.1, 2.1], [0.1, -0.1, 2.1], [0.1, 0.1, 2.1], [-0.1, 0.1, 2.1]],
        "shape_dimensions": [0.2, 0.2, 0.2], "depth_supported_face_count": 3,
        "plane_inlier_ratio": 0.9, "plane_residual_mean": 0.001,
        "completion_mode": "orthogonality_selected_multi_plane_cuboid", "uncertainty": {"sigma_m": 0.01},
    }
    hypothesis = hypotheses_from_geometry_record(record, source_instance_id="box-1", pointmap_source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH, presence_score=0.9)[0]
    captured = transform_hypothesis_to_world(hypothesis, _metadata(-pi / 2.0))
    inference_pose = _metadata(0.0).T_W_C_at_capture
    assert captured.pose_world is not None
    assert captured.pose_world.position_m != pytest.approx(tuple(inference_pose[row][3] for row in range(3)))


def test_registered_three_plane_pose_prefers_metric_cuboid_over_2d_face_anchor():
    record = {
        "accepted": True,
        "orthogonal_axes_3d": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "corners_3d": [[-0.1, -0.1, 1.0]] * 8,
        "unanchored_corners_3d": [
            [-0.3, -0.2, 1.8], [0.3, -0.2, 1.8], [0.3, 0.2, 1.8], [-0.3, 0.2, 1.8],
            [-0.3, -0.2, 2.1], [0.3, -0.2, 2.1], [0.3, 0.2, 2.1], [-0.3, 0.2, 2.1],
        ],
        "shape_dimensions": [0.6, 0.4, 0.3],
        "depth_supported_face_count": 3,
        "orthogonal_plane_pair_diagnostics": {"reliable": True},
        "plane_inlier_ratio": 0.9,
        "plane_residual_mean": 0.001,
        "completion_mode": "orthogonality_selected_multi_plane_cuboid",
        "uncertainty": {"sigma_m": 0.01},
    }
    metric = hypotheses_from_geometry_record(
        record, source_instance_id="box-1",
        pointmap_source=MetricPointMapSource.ISAAC_IDEAL_REGISTERED_DEPTH,
        presence_score=0.9,
    )[0]
    monocular = hypotheses_from_geometry_record(
        record, source_instance_id="box-1",
        pointmap_source=MetricPointMapSource.MOGE_MONOCULAR_ESTIMATE,
        presence_score=0.9,
    )[0]
    assert metric.pose_camera.position_m == pytest.approx((0.0, 0.0, 1.95))
    assert metric.evidence["pose_corner_source"] == "unanchored_corners_3d"
    assert monocular.pose_camera.position_m == pytest.approx((-0.1, -0.1, 1.0))
    assert monocular.evidence["pose_corner_source"] == "corners_3d"
