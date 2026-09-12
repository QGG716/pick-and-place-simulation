from __future__ import annotations

import json
from pathlib import Path

import pytest

from unloading_perception.coverage import (
    carton_front_face,
    derive_stack_bounds,
    evaluate_front_face_coverage,
    project_world_point,
)
from unloading_perception.isaac_validation import build_scene_manifest, load_validation_config


ROOT = Path(__file__).resolve().parents[1]


def _manifest():
    config = load_validation_config(ROOT / "configs/isaac/perception_validation.yaml")
    snapshot = json.loads((ROOT / config["layout_bundle"]["snapshot"]).read_text(encoding="utf-8"))
    contract = json.loads((ROOT / config["layout_bundle"]["isaac_contract"]).read_text(encoding="utf-8"))
    return build_scene_manifest(
        snapshot,
        contract,
        config,
        run_id="coverage-test",
        perception_commit="a" * 40,
        feasibility_reference_commit=config["layout_bundle"]["feasibility_reference_commit"],
        isaac_version="test",
        simulation_epoch="test",
    ), snapshot


def test_front_surface_is_derived_from_manifest_geometry_not_name_indices():
    manifest, _ = _manifest()
    bounds = derive_stack_bounds(tuple(reversed(manifest.objects)))
    assert bounds["carton_count"] == 40
    assert bounds["front_x_range_m"] == pytest.approx([0.0, 0.0])
    assert bounds["y_range_m"] == pytest.approx([-1.04, 1.04])
    assert bounds["z_range_m"] == pytest.approx([0.0, 2.4])
    assert carton_front_face(manifest.objects[0])[0][0] == pytest.approx(0.0)


def test_legacy_90_degree_camera_reproduces_right_and_lower_projection_loss():
    manifest, snapshot = _manifest()
    camera = dict(manifest.cameras[0])
    width, height = camera["resolution"]
    camera["module_id"] = "module_0_main"
    camera["K"] = [
        width / 2.0, 0.0, (width - 1.0) / 2.0,
        0.0, height / (2.0 * __import__("math").tan(65.0 * __import__("math").pi / 360.0)), (height - 1.0) / 2.0,
        0.0, 0.0, 1.0,
    ]
    report = evaluate_front_face_coverage((camera,), manifest.objects, density=9)
    right_wall = project_world_point(camera, (0.0, snapshot["trailer"]["right_wall_y_m"], 1.2))
    left_wall = project_world_point(camera, (0.0, snapshot["trailer"]["left_wall_y_m"], 1.2))
    assert report["union_sample_coverage"] < 0.5
    assert report["fully_covered_carton_count"] < report["carton_count"]
    assert right_wall["inside"] is False
    assert right_wall["pixel_xy"][0] > camera["resolution"][0]
    assert 0.0 <= left_wall["pixel_xy"][0] < camera["resolution"][0]
    assert left_wall["inside"] is False  # the upper camera also misses this low-Z probe


def test_projection_uses_capture_transform_and_output_intrinsics():
    manifest, _ = _manifest()
    camera = manifest.cameras[0]
    forward = [camera["T_W_C"][axis][2] for axis in range(3)]
    center = [camera["T_W_C"][axis][3] + forward[axis] for axis in range(3)]
    projected = project_world_point(camera, center)
    assert projected["inside"] is True
    assert projected["pixel_xy"] == pytest.approx([camera["K"][2], camera["K"][5]])


def test_dual_120x65_candidate_covers_every_front_face_sample():
    manifest, _ = _manifest()
    assert [camera["module_id"] for camera in manifest.cameras] == ["module_0_upper", "module_1_lower"]
    report = evaluate_front_face_coverage(manifest.cameras, manifest.objects, density=17)
    assert report["union_sample_coverage"] == pytest.approx(1.0)
    assert report["fully_covered_carton_count"] == 40
    assert report["per_module_sample_coverage"]["module_0_upper"] < 0.3
    assert report["per_module_sample_coverage"]["module_1_lower"] > 0.8
