from __future__ import annotations

import pytest

from unloading_perception.fusion import ModuleFaceBatch, fuse_module_face_batches
from unloading_perception.observed_faces import ObservedFace, ObservedFaceSet


def _face_set(module: str, source: str, *, time: float = 1.0, shift_x: float = 0.0, support: int = 100):
    corners = (
        (-0.3 + shift_x, -0.2, 2.0), (0.3 + shift_x, -0.2, 2.0),
        (0.3 + shift_x, 0.2, 2.0), (-0.3 + shift_x, 0.2, 2.0),
    )
    face = ObservedFace(
        f"{source}:front", (0, 1, 2, 3), ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)),
        corners, (0.0, 0.0, -1.0), 2.0, support, 0.9, 0.002,
        "registered_metric_depth_plane", "world",
    )
    return ObservedFaceSet(source, module, f"cap:{module}", time, "world", (face,), (),
                           "INSUFFICIENT_SINGLE_FACE_WITHOUT_SIZE_PRIOR")


def _batch(module: str, source: str, *, epoch: str = "epoch-1", time: float = 1.0,
           shift_x: float = 0.0, support: int = 100):
    face_set = _face_set(module, source, time=time, shift_x=shift_x, support=support)
    return ModuleFaceBatch(module, epoch, 10, time, (face_set,))


def test_overlap_object_is_fused_without_oracle_identity():
    result = fuse_module_face_batches(
        (_batch("module_0_upper", "sam-12", support=80),
         _batch("module_1_lower", "sam-93", shift_x=0.01, support=120)),
        expected_modules=("module_0_upper", "module_1_lower"),
    )
    assert len(result.objects) == 1
    assert result.objects[0].source_members == (("module_0_upper", "sam-12"), ("module_1_lower", "sam-93"))
    assert len(result.objects[0].observed_faces) == 1
    assert result.objects[0].observed_faces[0].point_support_count == 120
    assert result.objects[0].diagnostics["oracle_identity_used"] is False


def test_inconsistent_overlap_is_retained_and_never_averaged():
    result = fuse_module_face_batches(
        (_batch("module_0_upper", "a"), _batch("module_1_lower", "b", shift_x=0.08)),
        expected_modules=("module_0_upper", "module_1_lower"),
    )
    assert len(result.objects) == 1
    assert result.objects[0].association_status == "CONFLICT_RETAINED_NO_AVERAGE"
    assert len(result.objects[0].observed_faces) == 2
    assert sorted(face.corners_3d_m[0][0] for face in result.objects[0].observed_faces) == pytest.approx([-0.3, -0.22])


def test_missing_module_retains_observation_but_degrades_coverage():
    result = fuse_module_face_batches(
        (_batch("module_0_upper", "a"),),
        expected_modules=("module_0_upper", "module_1_lower"),
    )
    assert len(result.objects) == 1
    assert result.coverage_status == "DEGRADED_MISSING_MODULE"


def test_mixed_epoch_and_non_simultaneous_captures_fail_closed():
    with pytest.raises(ValueError, match="STALE_OR_MIXED"):
        fuse_module_face_batches(
            (_batch("module_0_upper", "a"), _batch("module_1_lower", "b", epoch="epoch-2")),
            expected_modules=("module_0_upper", "module_1_lower"),
        )
    with pytest.raises(ValueError, match="NON_SIMULTANEOUS"):
        fuse_module_face_batches(
            (_batch("module_0_upper", "a", time=1.0), _batch("module_1_lower", "b", time=1.04)),
            expected_modules=("module_0_upper", "module_1_lower"),
        )
