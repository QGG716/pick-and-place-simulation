from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from unloading_sim.asset_audit import (
    AssetAuditError,
    audit_m710id70_asset_set,
    load_and_audit_asset_manifest,
    load_and_audit_link_mapping,
)


ROOT = Path(__file__).resolve().parents[1]
ROBOT_MANIFEST = ROOT / "assets/robots/fanuc_m710id_70/cad_asset_manifest.yaml"
LINK_MAPPING = ROOT / "assets/robots/fanuc_m710id_70/link_mapping.yaml"
TOOL_MANIFEST = ROOT / "assets/grippers/shanghai_wantai_three_zone/asset_manifest.yaml"


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_complete_asset_set_is_hash_verified_but_not_execution_qualified() -> None:
    report = audit_m710id70_asset_set(ROOT)

    assert report["status"] == "SOURCE_INTEGRITY_PASS_CONVERSION_REQUIRED"
    assert report["execution_qualified"] is False
    assert report["robot"]["source_integrity"] is True
    assert report["robot"]["verified_file_count"] == 17
    assert report["robot"]["verified_source_bytes"] == 29_450_224
    assert report["robot"]["conversion_status"] == "SOURCE_AVAILABLE_CONVERSION_REQUIRED"
    assert report["robot"]["execution_qualified"] is False
    assert len(report["robot"]["missing_converted_outputs"]) == 14
    assert report["tool"]["source_integrity"] is True
    assert report["tool"]["verified_file_count"] == 5
    assert report["tool"]["execution_qualified"] is True


def test_robot_manifest_records_literal_vendor_files_and_conversion_gap() -> None:
    report = load_and_audit_asset_manifest(ROBOT_MANIFEST, repository_root=ROOT)
    manifest = _yaml(ROBOT_MANIFEST)
    by_role = {item["role"]: item for item in manifest["source_files"]}

    assert report.manifest_path == "assets/robots/fanuc_m710id_70/cad_asset_manifest.yaml"
    assert by_role["two_dimensional_reference"] == {
        "role": "two_dimensional_reference",
        "path": "res/M-710iD_70/2D_M-710iD_70_v01.dxf",
        "bytes": 12_512_058,
        "sha256": "0facfbf8dfbea236cf157d0fd405e33504bd022e6fff01143ce6d0f42c6003e9",
        "format": "ascii_dxf_ac1021",
    }
    assert by_role["zero_pose_exact_cad"]["bytes"] == 6_335_776
    assert by_role["zero_pose_exact_cad"]["sha256"] == (
        "9642c0336519ced0f8e8bbab43b8463ae2aacde6db9ca93ee4fbed2f611fd118"
    )
    assert by_role["robcad_archive"]["sha256"] == (
        "8b1074538a993c53ce3e9f006434b03cc9c3f50c63f9609152c6f9c4fab0e09c"
    )
    assert manifest["units"]["dxf_insunits_code"] == 4
    assert manifest["units"]["parasolid_header_unit_factor_per_metre"] == 1000.0
    assert manifest["execution_qualified"] is False
    assert "PER_LINK_VISUAL_MESHES_MISSING" in report.unresolved
    assert "PER_LINK_COLLISION_MESHES_MISSING" in report.unresolved
    assert all(not (ROOT / path).exists() for path in report.missing_converted_outputs)
    assert not list((ROOT / "res/M-710iD_70").rglob("Thumbs.db"))


def test_link_mapping_preserves_axes_bodies_and_unresolved_semantics() -> None:
    mapping = load_and_audit_link_mapping(LINK_MAPPING, repository_root=ROOT)

    assert [joint["axis_direction"] for joint in mapping["joints"]] == [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]
    assert [link["zero_origin_from_base_m"] for link in mapping["links"]] == [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.150, 0.0, 0.565],
        [0.150, 0.0, 1.460],
        [0.150, 0.0, 1.630],
        [1.195, 0.0, 1.630],
        [1.195, 0.0, 1.630],
    ]
    assert sum(len(link["bodies"]) for link in mapping["links"]) == 21
    assert mapping["joints"][2]["follows"] == {
        "joint": "J2",
        "ratio": 1.0,
        "source_literal": "follows j2 by 1",
        "interpretation_status": "UNRESOLVED_CONTROLLER_TO_KINEMATIC_MAPPING",
    }
    zero = mapping["zero_pose"]
    assert zero["flange_translation_from_base_mm"] == [1370.0, 0.000246, 1630.0]
    assert zero["current_urdf_to_source_tcp_relative_rotation"] == [
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    assert zero["current_urdf_to_source_tcp_relative_angle_deg"] == 180.0
    assert zero["clocking_status"] == "UNRESOLVED_180_DEG_ABOUT_COMMON_TOOL_Z"


def test_wantai_manifest_keeps_physical_contact_distinct_from_virtual_tcp() -> None:
    report = load_and_audit_asset_manifest(TOOL_MANIFEST, repository_root=ROOT)
    manifest = _yaml(TOOL_MANIFEST)
    by_role = {item["role"]: item for item in manifest["source_files"]}
    contact = manifest["contact_geometry"]

    assert report.source_integrity is True
    assert report.execution_qualified is True
    assert by_role["authoritative_geometry"]["sha256"] == (
        "d54ec5818a772877162e7d8eded484c67920de148037373d9d7d8e35cfa0694b"
    )
    assert by_role["derived_visual_mesh"]["sha256"] == (
        "a3c966bf676da0ac410f099225d2682ef00c56d8e2d9cc3184609630308b733a"
    )
    assert by_role["derived_full_shape_triangle_mesh"]["sha256"] == (
        "4113746138501efde8748f8c77d82612590fba1b6baca7c7e24127c66b2c5ce8"
    )
    assert by_role["derived_mass_and_contact_analysis"]["sha256"] == (
        "2596f20270848f8874cf8e766602be7985da8dd047d769889b83f690009f13f0"
    )
    assert manifest["frames"]["flange_origin_step_mm"] == [-48.561732, -162.659188, 0.0]
    assert manifest["frames"]["rotation_step_from_tool"] == [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    assert contact["flange_to_uncompressed_cup_extreme_m"] == 0.2275
    assert contact["flange_to_nominal_compressed_contact_plane_m"] == 0.2125
    assert contact["active_task_tcp_from_flange_m"] == 0.250
    assert contact["task_tcp_beyond_uncompressed_extreme_m"] == 0.0225
    assert contact["task_tcp_beyond_nominal_compressed_contact_plane_m"] == 0.0375
    assert manifest["collision_geometry"]["rigid_solid_count"] == 58
    assert manifest["collision_geometry"]["final_dynamic_collision_status"] == "NOT_QUALIFIED"


def test_asset_audit_rejects_source_hash_drift(tmp_path: Path) -> None:
    manifest = _yaml(ROBOT_MANIFEST)
    manifest["source_files"][0]["sha256"] = "0" * 64
    changed = tmp_path / "changed.yaml"
    changed.write_text(yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False), encoding="utf-8")

    with pytest.raises(AssetAuditError, match="SHA-256 mismatch"):
        load_and_audit_asset_manifest(changed, repository_root=ROOT)


def test_asset_audit_rejects_repository_escape(tmp_path: Path) -> None:
    manifest = _yaml(ROBOT_MANIFEST)
    manifest["source_files"][0]["path"] = "../outside.dxf"
    changed = tmp_path / "escaped.yaml"
    changed.write_text(yaml.safe_dump(manifest, allow_unicode=True, sort_keys=False), encoding="utf-8")

    with pytest.raises(AssetAuditError, match="must remain within the repository"):
        load_and_audit_asset_manifest(changed, repository_root=ROOT)
