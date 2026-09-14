from __future__ import annotations

import pytest
from dataclasses import replace

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
    lower = _batch("module_1_lower", "b", shift_x=0.08)
    face = lower.face_sets[0].faces[0]
    face = replace(face, corners_3d_m=tuple((x,y,z+.03) for x,y,z in face.corners_3d_m), plane_offset_m=2.03)
    lower = replace(lower, face_sets=(replace(lower.face_sets[0],faces=(face,)),))
    result = fuse_module_face_batches(
        (_batch("module_0_upper", "a"), lower),
        expected_modules=("module_0_upper", "module_1_lower"),
    )
    assert len(result.objects) == 1
    assert result.objects[0].association_status == "CONFLICT_RETAINED_NO_AVERAGE"
    assert len(result.objects[0].observed_faces) == 2
    assert sorted(face.corners_3d_m[0][0] for face in result.objects[0].observed_faces) == pytest.approx([-0.3, -0.22])


def test_partial_boundaries_are_not_a_geometric_conflict():
    result=fuse_module_face_batches((_batch('upper','1'),_batch('lower','1',shift_x=.08)),expected_modules=('upper','lower'))
    assert len(result.objects)==1
    assert result.objects[0].association_status=='ASSOCIATED_BY_WORLD_FACE_GEOMETRY'
    assert len(result.objects[0].observed_faces)==2


def test_nonzero_offset_opposite_normal_is_same_plane():
    a=_batch('upper','1'); b=_batch('lower','1'); f=b.face_sets[0].faces[0]
    b=replace(b,face_sets=(replace(b.face_sets[0],faces=(replace(f,plane_normal=(0.,0.,1.),plane_offset_m=-2.),)),))
    assert len(fuse_module_face_batches((a,b),expected_modules=('upper','lower')).objects)==1


def test_adjacent_coplanar_boxes_are_not_merged():
    result=fuse_module_face_batches((_batch('upper','1'),_batch('lower','2',shift_x=.6)),expected_modules=('upper','lower'))
    assert len(result.objects)==2


def test_full_patch_and_narrow_strip_associate_without_inventing_corners():
    a=_batch('upper','1'); b=_batch('lower','1'); face=b.face_sets[0].faces[0]
    strip=replace(face,corners_3d_m=tuple((x/20,y,z) for x,y,z in face.corners_3d_m))
    b=replace(b,face_sets=(replace(b.face_sets[0],faces=(strip,)),))
    result=fuse_module_face_batches((a,b),expected_modules=('upper','lower'))
    assert len(result.objects)==1
    assert len(result.objects[0].observed_faces)==2
    assert result.objects[0].association_status=='ASSOCIATED_BY_WORLD_FACE_GEOMETRY'


def test_adjacent_face_alone_is_insufficient_to_certify_same_entity():
    a=_batch('upper','1'); b=_batch('lower','1'); f=b.face_sets[0].faces[0]
    side=replace(f,corners_3d_m=((.3,-.2,2),(.3,-.2,2.5),(.3,.2,2.5),(.3,.2,2)),plane_normal=(1.,0.,0.),plane_offset_m=-.3)
    b=replace(b,face_sets=(replace(b.face_sets[0],faces=(side,)),))
    assert len(fuse_module_face_batches((a,b),expected_modules=('upper','lower')).objects)==2


def test_unknown_module_and_mixed_sequence_fail_closed():
    with pytest.raises(ValueError,match='UNKNOWN_MODULE'):
        fuse_module_face_batches((_batch('unknown','1'),),expected_modules=('upper','lower'))
    with pytest.raises(ValueError,match='MIXED_CAPTURE_GROUP'):
        fuse_module_face_batches((_batch('upper','1'),replace(_batch('lower','1'),frame_sequence=11)),expected_modules=('upper','lower'))


def test_close_candidates_remain_ambiguous_and_order_independent():
    a=_batch('upper','1'); b=_batch('lower','2'); c=_face_set('lower','3',shift_x=.001)
    b=replace(b,face_sets=(b.face_sets[0],c))
    r=fuse_module_face_batches((a,b),expected_modules=('upper','lower'))
    s=fuse_module_face_batches((replace(b,face_sets=b.face_sets[::-1]),a),expected_modules=('upper','lower'))
    assert r==s
    assert len(r.objects)==3
    assert all(o.association_status=='AMBIGUOUS_RETAINED' for o in r.objects)


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
