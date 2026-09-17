"""Small measured-patch fixtures, not multi-box detection acceptance."""
from copy import deepcopy
from dataclasses import asdict, replace
from itertools import permutations, product

import pytest

from unloading_perception.fusion import ModuleFaceBatch, _face_distance, _merge_faces, fuse_module_face_batches
from unloading_perception.observed_faces import ObservedFace, ObservedFaceSet


def patch(name, *, x=0., z=2., width=.4, support=100):
    return ObservedFace(
        name, (), ((0., 0.), (1., 0.), (1., 1.), (0., 1.)),
        ((x, 0., z), (x + width, 0., z), (x + width, .4, z), (x, .4, z)),
        (0., 0., -1.), z, support, .9, .002,
        "registered_metric_depth_plane", "world",
        {"boundary_evidence": [{"kind": "UNCLASSIFIED_BOUNDARY", "source": name}],
         "final_support": {"status": "PASS", "point_support_count": support}},
    )


def face_set(module, faces, source=None):
    return ObservedFaceSet(source or module + "/instance", module, "cap:" + module,
                           1., "world", tuple(faces), (), "MULTIFACE_OBSERVED_VOLUME_UNRESOLVED")


@pytest.mark.parametrize("conflicting", [False, True])
def test_unrelated_coplanar_patch_does_not_hide_later_match(conflicting):
    a, b = patch("A", x=-1.), patch("B")
    other = patch("B-prime", z=2.03 if conflicting else 2., support=120)
    outputs = [_merge_faces((face_set("m0", faces), face_set("m1", (other,))), .04)
               for faces in ((a, b), (b, a))]
    actual = [(tuple(face.face_id for face in faces), conflict) for faces, conflict in outputs]
    print("conflicting=", conflicting, "actual=", actual)
    expected_ids = {"A", "B", "B-prime"} if conflicting else {"A", "B-prime"}
    assert all({face.face_id for face in faces} == expected_ids and conflict == conflicting
               for faces, conflict in outputs), actual
    assert outputs[0] == outputs[1]


@pytest.mark.parametrize("x,width,count", [
    (0., .4, 1),       # genuine duplicate
    (.02, .4, 1),      # tolerated displacement with positive area overlap
    (1., .4, 2),       # disjoint coplanar patches
    (.4, .4, 2),       # touching along one boundary
    (.08, .4, 2),      # partially overlapping, different extent
    (0., .1, 2),       # contained strip is not the whole observed face
])
def test_patch_extent_matters(x, width, count):
    first, second = patch("first"), patch("second", x=x, width=width)
    for order in permutations((first, second)):
        faces, conflict = _merge_faces((face_set("m", order),), .04)
        assert len(faces) == count
        assert not conflict
        assert all(any(face is original for original in order) for face in faces)


@pytest.mark.parametrize("x", [.02, .03])
def test_small_touching_or_disjoint_patches_are_not_duplicates(x):
    # Corner distance alone passes 4 cm despite zero intersection area.
    first, second = patch("first", width=.02), patch("second", x=x, width=.02)
    assert _face_distance(first, second)[2] <= .04
    for order in permutations((first, second)):
        faces, conflict = _merge_faces((face_set("m", order),), .04)
        assert len(faces) == 2 and not conflict


@pytest.mark.parametrize("supports,winner", [((80, 120), "m1"), ((100, 100), "m0")])
def test_support_then_complete_source_identity_selects_actual_record(supports, winner):
    # face_id alone does not identify the source; both modules may use face:0.
    sources = tuple(face_set(module, (patch("face:0", support=support),))
                    for module, support in zip(("m0", "m1"), supports))
    expected = next(source.faces[0] for source in sources if source.module_id == winner)
    before = deepcopy(sources)
    for order in permutations(sources):
        faces, conflict = _merge_faces(order, .04)
        assert len(faces) == 1 and faces[0] is expected and not conflict
        assert _merge_faces(order, .04) == (faces, conflict)
    assert sources == before


def test_reused_full_source_identity_is_rejected_instead_of_first_wins():
    for faces in permutations((patch("same"), patch("same", x=1.))):
        with pytest.raises(ValueError, match="DUPLICATE_FACE_SOURCE_IDENTITY"):
            _merge_faces((face_set("m", faces),), .04)


def test_non_transitive_chain_uses_only_surviving_representatives():
    a, b, c = patch("A", support=300), patch("B", x=.03, support=200), patch("C", x=.06)
    assert _face_distance(a, b)[2] < .04 and _face_distance(b, c)[2] < .04
    assert _face_distance(a, c)[2] > .04
    before = deepcopy((a, b, c))
    for faces in permutations((a, b, c)):
        result = fuse_module_face_batches((ModuleFaceBatch("m", "epoch", 10, 1., (face_set("m", faces),)),),
                                          expected_modules=("m",))
        obj = result.objects[0]
        assert obj.observed_faces == (a, c)
        reduction = obj.diagnostics["face_reduction"]
        assert [(item["member"]["face_id"], item["representative"]["face_id"])
                for item in reduction["duplicate_members"]] == [("B", "A")]
        assert not reduction["conflicts"]
    assert (a, b, c) == before


def test_raw_conflict_survives_even_when_both_endpoints_are_removed():
    # The strongest middle plane is duplicate-compatible with both endpoints;
    # those raw endpoints conflict with each other (> 1 cm).
    low = patch("low", z=2., support=100)
    middle = patch("middle", z=2.009, support=300)
    high = patch("high", z=2.018, support=200)
    for faces in permutations((low, middle, high)):
        merged, conflict = _merge_faces((face_set("m", faces),), .04)
        assert merged == (middle,) and merged[0] is middle
        assert conflict
        result = fuse_module_face_batches((ModuleFaceBatch("m", "epoch", 10, 1., (face_set("m", faces),)),),
                                          expected_modules=("m",))
        obj = result.objects[0]
        assert obj.association_status == "CONFLICT_RETAINED_NO_AVERAGE"
        reduction = obj.diagnostics["face_reduction"]
        assert reduction["representative_count"] == 1
        assert len(reduction["duplicate_members"]) == 2
        record, = reduction["conflicts"]
        assert {record["first"]["face_id"], record["second"]["face_id"]} == {"low", "high"}
        assert record["plane_distance_m"] == pytest.approx(.018)
        assert record["intersection_over_smaller_patch"] == pytest.approx(1.)
        assert len(record["overlap_polygon_on_first_plane_m"]) == 4
        assert {point[2] for point in record["overlap_polygon_on_first_plane_m"]} == {high.plane_offset_m}


def batch_fixture(conflicting):
    first = face_set("m0", (patch("A", x=-1.), patch("B")))
    second = face_set("m1", (patch("B-prime", support=120),
                              patch("C", z=2.03 if conflicting else 2., support=80)))
    return (
        ModuleFaceBatch("m0", "epoch", 10, 1., (first, face_set("m0", (patch("far-0", x=10.),), "m0/far"))),
        ModuleFaceBatch("m1", "epoch", 10, 1., (second, face_set("m1", (patch("far-1", x=20.),), "m1/far"))),
    )


def batch_permutations(batches):
    variants = []
    for batch in batches:
        choices = []
        for faces in product(*(tuple(permutations(item.faces)) for item in batch.face_sets)):
            sets = tuple(replace(item, faces=order) for item, order in zip(batch.face_sets, faces))
            choices.extend(replace(batch, face_sets=order) for order in permutations(sets))
        variants.append(choices)
    for choices in product(*variants):
        yield from permutations(choices)


@pytest.mark.parametrize("conflicting", [False, True])
def test_public_fusion_all_input_permutations_preserve_complete_semantics(conflicting):
    batches = batch_fixture(conflicting)
    before = deepcopy(batches)
    expected = None
    count = 0
    for order in batch_permutations(batches):
        result = fuse_module_face_batches(order, expected_modules=("m0", "m1"))
        # No field removal or sorting of coordinate arrays: full exact equality.
        semantics = asdict(result)
        if expected is None:
            expected = semantics
        assert semantics == expected
        assert len(result.objects) == 3  # Two spatially separate controls stay separate.
        obj = next(item for item in result.objects if len(item.source_members) == 2)
        assert obj.source_members == (("m0", "m0/instance"), ("m1", "m1/instance"))
        assert {face.face_id for face in obj.observed_faces} == ({"A", "B-prime", "C"} if conflicting else {"A", "B-prime"})
        assert obj.association_status == ("CONFLICT_RETAINED_NO_AVERAGE" if conflicting else "ASSOCIATED_BY_WORLD_FACE_GEOMETRY")
        reduction = obj.diagnostics["face_reduction"]
        assert reduction["input_face_count"] == 4
        assert len(reduction["conflicts"]) == (2 if conflicting else 0)
        count += 1
    assert count == 32
    assert batches == before


def module_observations(batches):
    """Build valid source contracts from each measured CPU fixture record."""
    from unloading_perception.demo import _synthetic_observation
    base = _synthetic_observation()
    transform = ((1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.))
    observations = []
    for batch in batches:
        binding = {"module_id": batch.module_id, "capture_id": "cap:" + batch.module_id,
                   "calibration_identity": "cal:" + batch.module_id, "T_W_C_at_capture": transform}
        cargo = []
        for item in batch.face_sets:
            surfaces = tuple({
                **face.to_dict(), "schema_version": "observed_surface_v1", "volume_status": "UNKNOWN",
                **binding, "source_instance_id": item.source_instance_id, "sensor_epoch": batch.sensor_epoch,
                "clock_domain": base.clock_domain, "capture_time": item.capture_time,
            } for face in item.faces)
            cargo.append(replace(base.cargo[0], source_instance_id=item.source_instance_id,
                                 pose=None, full_dimensions_m=None, axes_3d_rows=None,
                                 candidate_eligible=False, eligibility_reasons=("UNKNOWN_VOLUME",),
                                 observed_surfaces=surfaces))
        observations.append(replace(base, observation_id="obs:" + batch.module_id,
                                    source_epoch=batch.sensor_epoch, source_sequence=batch.frame_sequence,
                                    capture_time=batch.capture_time, cargo=tuple(cargo),
                                    coverage={"module_binding": binding}))
    return tuple(observations)


@pytest.mark.parametrize("conflicting", [False, True])
def test_public_fusion_handoff_wire_roundtrip_preserves_all_raw_evidence(conflicting):
    from unloading_contracts import dumps, loads, canonical_fingerprint
    from unloading_perception.algorithm_handoff import fused_algorithm_observation, validate_algorithm_capture
    from unloading_perception.scene import build_scene_update

    expected = None
    for batches in batch_permutations(batch_fixture(conflicting)):
        fusion = fuse_module_face_batches(batches, expected_modules=("m0", "m1"))
        observations = module_observations(batches)
        before = tuple(dumps(item) for item in observations)
        handoff = fused_algorithm_observation(observations, fusion)
        restored = loads(dumps(handoff))
        validate_algorithm_capture(restored)
        assert canonical_fingerprint(restored) == canonical_fingerprint(handoff)
        # Formal wire representation is already stable across all permutations.
        if expected is None:
            expected = dumps(restored)
        assert dumps(restored) == expected
        assert dumps(fused_algorithm_observation(observations[::-1], fusion)) == expected
        assert tuple(dumps(item) for item in observations) == before
        obj = next(item for item in fusion.objects if len(item.source_members) == 2)
        cargo = next(item for item in restored.cargo if item.source_instance_id == obj.fusion_id)
        assert cargo.association_status == obj.association_status
        assert len(cargo.observed_surfaces) == 4
        originals = {surface["face_id"]: surface for obs in observations for item in obs.cargo
                     for surface in item.observed_surfaces}
        for surface in cargo.observed_surfaces:
            assert surface == originals[surface["face_id"]]  # geometry, support, binding AND boundary diagnostics
        assert canonical_fingerprint(cargo.raw_result["fusion_diagnostics"]) == canonical_fingerprint(obj.diagnostics)
        raw_ids = {surface["face_id"] for surface in cargo.observed_surfaces}
        for record in cargo.raw_result["fusion_diagnostics"]["face_reduction"]["conflicts"]:
            assert record["first"]["face_id"] in raw_ids and record["second"]["face_id"] in raw_ids
        assert len(restored.unknown_regions) == 3
        assert restored.coverage["absence_means_free_space"] is False
        for item in restored.cargo:
            assert not item.candidate_eligible
            assert item.pose is item.full_dimensions_m is item.corners_3d_m is item.axes_3d_rows is None
            assert item.track_id is item.object_id is None
        update = build_scene_update(restored)
        assert not update.planning_admissible and not update.candidate_objects
