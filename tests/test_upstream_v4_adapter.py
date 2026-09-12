from __future__ import annotations

from pathlib import Path

from unloading_perception.upstream_v4 import (
    REGISTERED_DEPTH_FACE,
    REGISTERED_JOINT_FACE,
    UPSTREAM_SELECTOR_FACE,
    build_upstream_v4_command,
    finalize_registered_depth_result,
    prepare_registered_depth_baseline,
    validate_upstream_v4_command,
)


def test_registered_depth_uses_legacy_selector_only_inside_worker_payload():
    source = {
        "instances": [{
            "mask_id": 7,
            "camera_facing_faces": [{"evidence": REGISTERED_DEPTH_FACE, "corner_indices": [0, 1, 2, 3]}],
        }]
    }
    prepared = prepare_registered_depth_baseline(source)
    face = prepared["instances"][0]["camera_facing_faces"][0]
    assert source["instances"][0]["camera_facing_faces"][0]["evidence"] == REGISTERED_DEPTH_FACE
    assert face["evidence"] == UPSTREAM_SELECTOR_FACE
    assert face["adapter_source_evidence"] == REGISTERED_DEPTH_FACE


def test_v4_output_restores_metric_provenance_and_never_publishes_hidden_faces():
    worker = {
        "instances": [{
            "mask_id": 7,
            "multiplane_refinement_status": "joint_sam_silhouette+depth_faces+fixed_intrinsics_pnp",
            "camera_facing_faces": [
                {"evidence": "joint_sam_depth_multiplane_pnp"},
                {"evidence": UPSTREAM_SELECTOR_FACE, "adapter_source_evidence": REGISTERED_DEPTH_FACE},
                {"evidence": "cuboid_constraint_completion"},
            ],
        }]
    }
    result = finalize_registered_depth_result(worker)
    faces = result["instances"][0]["camera_facing_faces"]
    assert [face["evidence"] for face in faces] == [REGISTERED_JOINT_FACE, REGISTERED_DEPTH_FACE]
    assert result["pointmap_source"] == "REGISTERED_METRIC_DEPTH"
    assert result["v4_adapter"]["refined_instance_count"] == 1
    assert all(face["evidence"] != "bounded_cuboid_completion" for face in faces)
    assert result["v4_adapter"]["suppressed_completion_face_count"] == 1


def test_command_invokes_actual_v4_worker_with_observed_face_safeguards():
    vision_root = Path("tmp") / "vision"
    command = build_upstream_v4_command(
        "python", vision_root, source="rgb.png", masks="masks.npz", pointmap="points.npz",
        baseline="baseline.json", fallback="fallback.json", json_output="v4.json",
        image_output="v4.png",
    )
    validate_upstream_v4_command(command, vision_root)
    assert Path(command[1]).as_posix().endswith("pipeline/geometry/refine_multiplane_cuboids.py")
    assert "--suppress-hidden-fallback" in command
    assert command[command.index("--min-face-precision") + 1] == "0.72"
