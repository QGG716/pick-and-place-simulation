from __future__ import annotations

import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from unloading_sim.asset_audit import audit_m710id70_official_model


ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / "assets/robots/fanuc_m710id_70/official"
MANIFEST = OFFICIAL / "provenance.yaml"
URDF = OFFICIAL / "fanuc_m710_description/urdf/m710id_70_official.urdf"


def _manifest() -> dict:
    return yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))


def test_pinned_official_model_asset_is_fully_hash_and_semantic_audited() -> None:
    report = audit_m710id70_official_model(ROOT)

    assert report.upstream_commit == "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2"
    assert report.package_version == "2.3.0"
    assert report.source_integrity is True
    assert report.static_urdf_integrity is True
    assert report.model_semantics is True
    assert report.verified_source_file_count == 18
    assert report.verified_source_bytes == 8_657_343
    assert report.visual_mesh_count == 7
    assert report.collision_mesh_count == 7
    assert report.execution_qualified is True


def test_upstream_identity_and_representative_blob_hashes_are_literal() -> None:
    manifest = _manifest()
    upstream = manifest["upstream"]
    records = {record["upstream_path"]: record for record in manifest["source_files"]}

    assert upstream == {
        "repository_url": "https://github.com/FANUC-CORPORATION/fanuc_description.git",
        "commit": "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2",
        "tag": "v2.3.0",
        "package_name": "fanuc_m710_description",
        "package_version": "2.3.0",
        "license_spdx": "Apache-2.0",
        "license_path": "assets/robots/fanuc_m710id_70/official/LICENSES/Apache-2.0.txt",
    }
    assert records["fanuc_m710_description/meshes/m710id_70/visual/base.dae"]["bytes"] == 675_313
    assert records["fanuc_m710_description/meshes/m710id_70/visual/base.dae"]["sha256"] == (
        "35ee039f1d466ea7f0229f34edeb6b47bc81768d4b4013e724edd26297c0b537"
    )
    assert records["fanuc_m710_description/meshes/m710id_70/collision/j3.stl"]["sha256"] == (
        "49116b78b394c8e0b60ca6b23f4391438ab04e702854fe4a4468b4c132c4c080"
    )
    assert records["fanuc_m710_description/urdf/m710id_70_urdf_macro.xacro"]["sha256"] == (
        "ede96c55fd7ec665a259d6f09422e37d8b69a2d27e3adc412f313930f37ed28d"
    )
    for record in records.values():
        path = ROOT / record["path"]
        assert len(path.read_bytes()) == record["bytes"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"]


def test_official_joint_chain_is_independent_and_preserves_literal_limits() -> None:
    model = _manifest()["model"]
    joints = {joint["name"]: joint for joint in model["joints"]}
    robot = ET.parse(URDF).getroot()

    assert model["actuated_joint_order"] == ["J1", "J2", "J3", "J4", "J5", "J6"]
    assert joints["J2"]["axis"] == [0.0, 1.0, 0.0]
    assert joints["J3"]["axis"] == [0.0, -1.0, 0.0]
    assert joints["J3"]["origin_xyz_m"] == [0.0, 0.0, 0.895]
    assert joints["J3"]["limit"] == {
        "lower_rad": -1.4049900478554354,
        "upper_rad": 3.5779249665883754,
        "velocity_rad_s": math.pi,
        "effort_nm": 5000.0,
    }
    j3 = robot.find("joint[@name='J3']")
    assert j3 is not None and j3.attrib["type"] == "revolute"
    assert j3.find("mimic") is None
    assert not robot.findall(".//mimic")


def test_official_inertials_are_complete_and_not_engineering_estimates() -> None:
    model = _manifest()["model"]
    inertials = {record["link"]: record for record in model["inertials"]}

    assert list(inertials) == [
        "base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link",
    ]
    assert model["total_mass_kg"] == 580.347
    assert inertials["J2_link"]["mass_kg"] == 137.0
    assert inertials["J2_link"]["origin_xyz_m"] == [-0.0291, 0.17, 0.4]
    assert inertials["J2_link"]["inertia_at_com_kg_m2"] == [
        [13.3, 0.0925, -0.0231],
        [0.0925, 14.2, -1.14],
        [-0.0231, -1.14, 1.9],
    ]
    assert inertials["J6_link"]["mass_kg"] == 0.447
    for record in inertials.values():
        matrix = record["inertia_at_com_kg_m2"]
        assert len(matrix) == 3 and all(len(row) == 3 for row in matrix)
        assert matrix[0][1] == matrix[1][0]
        assert matrix[0][2] == matrix[2][0]
        assert matrix[1][2] == matrix[2][1]


def test_fanuc_flange_and_planner_tool0_are_distinct_audited_frames() -> None:
    model = _manifest()["model"]
    fixed = {record["name"]: record for record in model["fixed_frames"]}

    assert fixed["J6-flange"] == {
        "name": "J6-flange",
        "parent": "J6_link",
        "child": "flange",
        "origin_xyz_m": [0.175, 0.0, 0.0],
        "origin_rpy_rad": [0.0, 0.0, 0.0],
    }
    assert fixed["flange-fanuc_flange"]["origin_rpy_rad"] == [
        math.pi, -math.pi / 2.0, 0.0,
    ]
    assert model["task_tool_frame"] == {
        "name": "flange-tool0",
        "parent": "flange",
        "child": "tool0",
        "origin_xyz_m": [0.0, 0.0, 0.0],
        "origin_rpy_rad": [0.0, math.pi / 2.0, 0.0],
        "source": "project_integration_wrapper",
    }
    assert model["fanuc_flange_link"] == "fanuc_flange"
    assert model["task_tool_link"] == "tool0"


def test_static_urdf_references_all_official_meshes_and_no_proxy_geometry() -> None:
    robot = ET.parse(URDF).getroot()
    meshes = [mesh.attrib["filename"] for mesh in robot.findall(".//mesh")]
    geometry = _manifest()["geometry"]

    assert len(meshes) == 14
    assert len(set(meshes)) == 14
    assert all(path.startswith("package://fanuc_m710_description/meshes/m710id_70/") for path in meshes)
    assert len(robot.findall(".//visual/geometry/mesh")) == 7
    assert len(robot.findall(".//collision/geometry/mesh")) == 7
    assert not robot.findall(".//collision/geometry/box")
    assert not robot.findall(".//collision/geometry/cylinder")
    assert not robot.findall(".//collision/geometry/sphere")
    assert geometry["base_collision_bounds_m"] == {
        "min_xyz_m": [-0.3385, -0.275, 0.0],
        "max_xyz_m": [0.225, 0.275, 0.245],
    }
    assert geometry["collision_triangle_count_total"] == 17_634
    assert geometry["collision_topology"] == "manufacturer_per_link_triangle_mesh_not_asserted_convex"
    assert _manifest()["qualification"] == {
        "scope": "robot_description_asset_only",
        "execution_qualified": True,
        "system_execution_qualified": False,
        "required_checks": [
            "pinned_source_hashes",
            "package_identity",
            "unmodified_official_macro",
            "seven_visual_meshes",
            "seven_collision_meshes",
            "joint_and_frame_semantics",
            "complete_official_inertials",
            "deterministic_static_urdf",
            "no_proxy_collision",
        ],
        "policy": "robot_asset_qualification_does_not_qualify_tool_path_workcell_or_replay",
    }
