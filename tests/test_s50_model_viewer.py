"""Viewer asset integrity and truthful readiness, independent of a browser."""
import base64
from copy import deepcopy
import hashlib
import gzip
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("s50_viewer_exporter", ROOT / "tools/export_s50_model_viewer.py")
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


@pytest.fixture
def tiny_cad(tmp_path):
    content = "v 0 0 0\nv 0.5 0 0\nv 0 1 0\nf 1 2 3\n"
    parts = []
    for i in range(7):
        path = tmp_path / f"part_{i}.obj"
        path.write_text(content, encoding="utf-8")
        parts.append(dict(id=f"part_{i}", name=f"Synthetic CAD parser fixture {i}", mesh=path.name,
            mesh_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), mesh_vertices=3, mesh_triangles=1,
            geometry_bounds_m=[[0, 0, 0], [.5, 1, 0]], mesh_bounds_m=[[0, 0, 0], [.5, 1, 0]],
            topology_valid=True, mesh_complete=True, missing_tessellation_faces=[],
            # Deliberately huge: exported vertices must never receive this twice.
            cad_placement=[[1, 0, 0, 800], [0, 1, 0, 900], [0, 0, 1, 700], [0, 0, 0, 1]]))
    audit = dict(source="synthetic_fixture_not_s50.step", source_sha256="synthetic", source_units="millimetres",
        normalization_scale_to_m=.001, coordinate_semantics="CAD assembly metres", parts=parts, limitations=[])
    path = tmp_path / "cad_audit.json"
    path.write_text(json.dumps(audit), encoding="utf-8")
    return path, audit


def test_cad_assembly_vertices_never_reapply_instance_placement(tiny_cad):
    path, _ = tiny_cad
    data = exporter.export_cad(path, path.parent)
    first = data["parts"][0]
    vertices = np.frombuffer(base64.b64decode(first["positions_b64"]), dtype="<f4").reshape(-1, 3)
    triangles = np.frombuffer(base64.b64decode(first["indices_b64"]), dtype="<u4").reshape(-1, 3)
    np.testing.assert_array_equal(vertices, [[0, 0, 0], [.5, 0, 0], [0, 1, 0]])
    np.testing.assert_array_equal(triangles, [[0, 1, 2]])
    assert data["instance_placements_applied_again"] is False
    assert data["mounting_alignment_verified"] is False
    assert data["articulated_model_ready"] is False
    assert data["triangle_count"] == 7


def test_gzip_only_checkout_exports_identical_cad_payload(tiny_cad):
    path, audit = tiny_cad
    expected = exporter.export_cad(path, path.parent)
    for part in audit["parts"]:
        raw = path.parent / part["mesh"]
        raw.with_name(raw.name + ".gz").write_bytes(gzip.compress(raw.read_bytes(), mtime=0))
        raw.unlink()
    actual = exporter.export_cad(path, path.parent)
    assert actual == expected
    assert [part["mesh"] for part in actual["parts"]] == [part["mesh"] for part in audit["parts"]]


@pytest.mark.parametrize("corruption,reason", [("invalid_gzip", "GZIP_INVALID"), ("wrong_decompressed_hash", "SHA256_MISMATCH")])
def test_gzip_fallback_rejects_corruption_and_wrong_content(tiny_cad, corruption, reason):
    path, audit = tiny_cad
    raw = path.parent / audit["parts"][0]["mesh"]
    compressed = raw.with_name(raw.name + ".gz")
    compressed.write_bytes(b"not a gzip stream" if corruption == "invalid_gzip" else gzip.compress(b"wrong CAD bytes", mtime=0))
    raw.unlink()
    with pytest.raises(ValueError, match=reason):
        exporter.export_cad(path, path.parent)


@pytest.mark.parametrize("changed,reason", [("hash", "SHA256"), ("count", "COUNT"), ("bounds", "BOUNDS")])
def test_corrupt_cad_manifest_fails_closed(tiny_cad, changed, reason):
    path, audit = tiny_cad
    if changed == "hash":
        audit["parts"][0]["mesh_sha256"] = "incorrect"
    elif changed == "count":
        audit["parts"][0]["mesh_vertices"] = 4
    else:
        audit["parts"][0]["mesh_bounds_m"][1][1] = 1000
    path.write_text(json.dumps(audit), encoding="utf-8")
    with pytest.raises(ValueError, match=reason):
        exporter.export_cad(path, path.parent)


def test_invalid_mesh_index_is_not_drawn(tmp_path):
    path = tmp_path / "bad.obj"
    path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 99\n", encoding="utf-8")
    with pytest.raises(ValueError, match="INVALID_CAD_OBJ_MESH"):
        exporter.read_cad_obj(path)


def test_cad_defects_are_preserved_in_viewer_metadata(tiny_cad):
    path, audit = tiny_cad
    audit["parts"][0].update(topology_valid=False, mesh_complete=False, missing_tessellation_faces=[{"face_index": 74}])
    path.write_text(json.dumps(audit), encoding="utf-8")
    data = exporter.export_cad(path, path.parent)
    assert data["parts"][0]["topology_valid"] is False
    assert data["parts"][0]["mesh_complete"] is False
    assert data["parts"][0]["missing_tessellation_faces"] == [{"face_index": 74}]
    assert data["display_only"] is True


def test_actual_frozen_reference_does_not_claim_robot_fk(monkeypatch):
    monkeypatch.setattr(exporter, "export_cad", lambda *args: {"parts": [], "triangle_count": 0})
    data = exporter.build_data(root=ROOT)
    assert data["model_status"] == "MODEL_NOT_READY"
    assert data["articulated_model"] is None
    assert data["full_scene_fingerprint"] is None
    assert not data["replacement_complete"]
    assert not data["joint_controls_enabled"]
    assert data["trajectory"] == [] and data["trajectory_status"] == "NOT_RUN"
    assert data["layout_difference"]["passed"] is True
    assert len(data["environment"]["initial_cartons"]) == 40
    assert data["tool_reference"]["collision_checked"] is False
    target = np.asarray(data["environment"]["initial_physical_contact_target"])
    frames = data["environment"]["tool_intrinsic"]["intrinsic_frames"]
    reference = np.asarray(data["tool_reference"]["world_from_virtual_task_tcp"])
    actual_contact = reference @ np.linalg.inv(frames["T_tool_mount_reference_virtual_task_tcp"]) @ frames["T_tool_mount_reference_nominal_compressed_contact"]
    np.testing.assert_allclose(actual_contact, target, atol=1e-12)


def test_single_file_output_uses_offline_vendor_and_nonexecutable_compressed_data(tmp_path):
    original = {"untrusted": "</script><script>alert('injection')</script>", "value": .12345678901234568, "label": "真实几何"}
    output = exporter.write_viewer(tmp_path / "viewer.html", original)
    html = output.read_text(encoding="utf-8")
    assert "/*__VENDOR__*/" not in html and "/*__DATA__*/" not in html
    assert "<script src=" not in html
    assert "OrbitControls" in html and "Permission is hereby granted" in html
    encoded = json.loads(html.split("const encoded=", 1)[1].split(";", 1)[0])
    compressed = base64.b64decode(encoded)
    assert gzip.decompress(compressed) == exporter.serialize_payload(original)
    assert json.loads(gzip.decompress(compressed)) == original
    assert compressed[4:8] == b"\0\0\0\0"  # gzip mtime=0
    assert encoded == exporter.encode_payload(original)
    assert original["untrusted"] not in html
    assert "new DecompressionStream('gzip')" in html
    assert "typeof DecompressionStream!=='function'" in html
    assert "查看器加载失败" in html
    assert "slider.disabled=true" in html
    assert "MODEL_NOT_READY" in html
    assert "没有套用作业世界安装变换" in html
