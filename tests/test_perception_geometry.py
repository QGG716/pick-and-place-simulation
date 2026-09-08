import math

import pytest

from unloading_contracts import EvidenceKind, Pose3D
from unloading_perception.geometry import pose_from_axes_rows, transform_pose, validate_transform_parent_child


def test_t_parent_child_rotates_then_translates_child_pose():
    transform = (
        (0.0, -1.0, 0.0, 2.0),
        (1.0, 0.0, 0.0, 3.0),
        (0.0, 0.0, 1.0, 4.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    pose = Pose3D((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "camera_optical", "columns", EvidenceKind.OBSERVED)
    world = transform_pose(transform, pose, "world")
    assert world.position_m == pytest.approx((2.0, 4.0, 4.0))
    assert world.orientation_xyzw == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))


def test_axis_rows_are_explicitly_transposed_to_rotation_columns():
    pose = pose_from_axes_rows((0, 0, 1), ((0, 1, 0), (-1, 0, 0), (0, 0, 1)), "camera", EvidenceKind.MODEL_ESTIMATED)
    assert pose.axis_convention == "right_handed_local_axes_stored_as_rows"
    assert sum(value * value for value in pose.orientation_xyzw) == pytest.approx(1.0)


@pytest.mark.parametrize("bad", [
    ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 0)),
    ((1, 0, 0, 0), (0, 2, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
    ((float("nan"), 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
])
def test_invalid_transform_fails_closed(bad):
    with pytest.raises(ValueError):
        validate_transform_parent_child(bad)
