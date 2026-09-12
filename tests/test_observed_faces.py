from __future__ import annotations

import pytest

from unloading_perception.observed_faces import (
    observed_faces_from_geometry_record, transform_observed_face_set,
)


IDENTITY = (
    (1.0, 0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0, 2.0),
    (0.0, 0.0, 1.0, 3.0),
    (0.0, 0.0, 0.0, 1.0),
)


def _record():
    return {
        "corners_3d": [
            [-0.3, -0.2, 2.0], [0.3, -0.2, 2.0], [0.3, 0.2, 2.0], [-0.3, 0.2, 2.0],
            [-0.3, -0.2, 2.3], [0.3, -0.2, 2.3], [0.3, 0.2, 2.3], [-0.3, 0.2, 2.3],
        ],
        "plane_inlier_counts": [120, 45],
        "plane_inlier_ratio": 0.88,
        "plane_residual_mean": 0.002,
        "camera_facing_faces": [
            {
                "evidence": "registered_metric_depth_plane", "axis_index": 0,
                "corner_indices": [0, 1, 2, 3],
                "corners_2d": [[10, 10], [30, 10], [30, 30], [10, 30]],
                "mask_precision": 0.95,
            },
            {
                "evidence": "joint_sam_registered_depth_multiplane_pnp", "axis_index": 1,
                "corner_indices": [0, 1, 5, 4],
                "corners_2d": [[10, 10], [30, 10], [28, 8], [12, 8]],
            },
            {
                "evidence": "cuboid_constraint_completion", "axis_index": 2,
                "corner_indices": [4, 5, 6, 7], "corners_2d": [[0, 0]] * 4,
            },
        ],
    }


def test_observed_face_set_excludes_completion_and_preserves_shared_edge():
    result = observed_faces_from_geometry_record(
        _record(), source_instance_id="box-7", module_id="module_1_lower",
        capture_id="cap-1", capture_time=2.0, frame_id="module_1_lower_rgb_optical",
    )
    assert len(result.faces) == 2
    assert result.complete_cuboid_status == "SUFFICIENT_MULTIFACE_EVIDENCE"
    assert result.shared_edges == (("box-7:face:0", "box-7:face:1", (0, 1)),)
    assert all(face.evidence != "cuboid_constraint_completion" for face in result.faces)


def test_single_observed_face_keeps_plane_but_marks_complete_cuboid_unknown():
    record = _record()
    record["camera_facing_faces"] = record["camera_facing_faces"][:1]
    result = observed_faces_from_geometry_record(
        record, source_instance_id="box-7", module_id="module_0_upper",
        capture_id="cap-1", capture_time=2.0, frame_id="camera",
    )
    assert len(result.faces) == 1
    assert result.complete_cuboid_status == "INSUFFICIENT_SINGLE_FACE_WITHOUT_SIZE_PRIOR"
    assert result.faces[0].plane_offset_m == pytest.approx(2.0)


def test_observed_faces_transform_with_capture_time_extrinsics():
    result = observed_faces_from_geometry_record(
        _record(), source_instance_id="box-7", module_id="module_1_lower",
        capture_id="cap-1", capture_time=2.0, frame_id="camera",
    )
    world = transform_observed_face_set(result, IDENTITY, "world")
    assert world.frame_id == "world"
    assert world.faces[0].corners_3d_m[0] == pytest.approx((0.7, 1.8, 5.0))
    assert world.faces[0].plane_offset_m == pytest.approx(5.0)
