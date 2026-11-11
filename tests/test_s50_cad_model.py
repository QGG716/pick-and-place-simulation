"""Actual S50 measured CAD preview tests, not a substitute robot fixture."""
from copy import deepcopy
import gzip
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from unloading_sim import s50_cad_model as cad
from unloading_sim.geometry import make_transform, rotation_matrix_from_rpy

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def definition():
    return cad.load_definition(ROOT)


@pytest.fixture(scope="module")
def validation():
    return cad.validate_preview_model(ROOT)


def test_true_s50_sources_and_independent_six_revolute_tree(definition):
    assert definition["source_step_sha256"] == "7296244abc02bca236a7ed26a0f10f5cc56fc63b7381fc1293b2edbb92509bc3"
    assert [link["cad_body"] for link in definition["links"]] == ["base_s50", "j1_s50", "j2_s50", "j3_s50", "j4_s50", "j5_s50", "j6_s50"]
    xml = ET.parse(ROOT / definition["urdf_path"]).getroot()
    revolute = [joint for joint in xml.findall("joint") if joint.get("type") == "revolute"]
    assert [joint.get("name") for joint in revolute] == [f"s50_joint_{i}" for i in range(1, 7)]
    assert not xml.findall(".//mimic")
    for i, joint in enumerate(revolute):
        assert joint.find("parent").get("link") == ("s50_base" if i == 0 else f"s50_link_{i}")
        assert joint.find("child").get("link") == f"s50_link_{i+1}"
    assert len(xml.findall(".//visual")) == 7
    assert all(link["visual_mesh"]["vertices"] > 10000 for link in definition["links"])


def test_literal_support_flange_and_measured_axis_geometry(definition):
    mount = np.asarray(definition["frames"]["mount"]["CAD_from_frame_zero"])
    np.testing.assert_allclose(mount[:3, 3], [0., -.003, 0.], atol=1e-14)
    np.testing.assert_array_equal(mount[:3, :3], [[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    axes = [joint["axis_CAD"] for joint in definition["joints"]]
    np.testing.assert_allclose(axes, [[0, 1, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]], atol=1e-13)
    points = [np.asarray(joint["measured_line_point_CAD_m"]) for joint in definition["joints"]]
    assert points[1][1] - mount[1, 3] == pytest.approx(.2833, abs=1e-12)
    assert points[2][1] - points[1][1] == pytest.approx(.9, abs=1e-12)
    assert points[3][1] - points[2][1] == pytest.approx(.9415, abs=1e-12)
    flange = cad.chain_fk(definition, np.zeros(6))["s50_flange"]
    np.testing.assert_allclose(flange[:3, 3], [-.3884998016477048, 0., 2.2833], atol=1e-12)
    np.testing.assert_allclose(flange[:3, 2], [-1., 0., 0.], atol=1e-12)


def test_q0_restores_every_original_cad_vertex_and_triangle(validation):
    assert validation["status"] == "PASS"
    assert len(validation["q0_geometry"]) == 7
    assert sum(row["triangles"] for row in validation["q0_geometry"]) == 1312676
    assert validation["maximum_q0_coordinate_error_m"] < 2e-12


def test_local_collision_patches_keep_physical_ownership_and_source_frames(definition, validation):
    assert definition["collision_coverage_status"] == "NOT_QUALIFIED"
    assert definition["collision_repair"]["whole_robot_coverage_qualified"] is False
    assert len(validation["collision_patch_geometry"]) == 2
    assert [(row["link"], row["source_face_index"]) for row in validation["collision_patch_geometry"]] == [
        ("s50_link_1", 74), ("s50_link_2", 75)]
    for row in validation["collision_patch_geometry"]:
        assert row["vertices"] == 1025 and row["triangles"] == 1023
        assert row["maximum_coordinate_error_m"] < 2e-12
        assert row["physical_thickness_added"] is False
    for link in definition["links"]:
        assert link["visual_mesh"]["sha256"] == link["collision_meshes"][0]["sha256"]
        expected = 2 if link["cad_body"] in ("j1_s50", "j2_s50") else 1
        assert len(link["collision_meshes"]) == expected
    assert not ET.parse(ROOT / definition["srdf_path"]).getroot().findall("disable_collisions")


def test_each_signed_joint_moves_only_real_descendant_bodies(validation):
    assert len(validation["single_axis_checks"]) == 12
    for i in range(6):
        rows = [row for row in validation["single_axis_checks"] if row["joint"] == f"s50_joint_{i+1}"]
        assert [row["angle_rad"] for row in rows] == [-.35, .35]
        for row in rows:
            assert row["moved_links"] == [f"s50_link_{n}" for n in range(i+1, 7)]
            assert row["unchanged_links"] == ["s50_base", *[f"s50_link_{n}" for n in range(1, i+1)]]
            assert row["maximum_coordinate_error_m"] < 2e-11


def test_core_recursive_and_spatial_poe_match_at_many_poses(validation):
    assert len(validation["fk_crosschecks"]) == 25
    assert validation["maximum_fk_matrix_error"] < 2e-11
    assert validation["deterministic_rng_seed"] == 20261110
    assert validation["controller_zero_verified"] is False
    assert validation["controller_signs_verified"] is False
    assert validation["collision_clearance_checked"] is False
    assert validation["motion_planning_run"] is False


@pytest.mark.parametrize("index", range(6))
def test_positive_velocity_obeys_right_hand_rule_about_measured_axis(definition, index):
    robot = cad.load_preview_robot(ROOT)
    joint = definition["joints"][index]
    link = definition["links"][index+1]
    points, _ = cad.read_obj(ROOT / link["visual_mesh"]["path"], link["visual_mesh"]["sha256"])
    point = points[len(points)//3:len(points)//3+1]
    CAD_from_mount = np.asarray(definition["frames"]["mount"]["CAD_from_frame_zero"])
    zero_point = cad.transform_vertices(link["CAD_from_link_zero"], point)[0]
    epsilon = 1e-6
    qp = np.zeros(6); qp[index] = epsilon
    qm = -qp
    plus = cad.transform_vertices(CAD_from_mount @ robot.named_link_frames(qp)[link["name"]], point)[0]
    minus = cad.transform_vertices(CAD_from_mount @ robot.named_link_frames(qm)[link["name"]], point)[0]
    derivative = (plus-minus)/(2*epsilon)
    expected = np.cross(joint["axis_CAD"], zero_point-np.asarray(joint["measured_line_point_CAD_m"]))
    np.testing.assert_allclose(derivative, expected, atol=3e-10, rtol=1e-7)


def test_explicit_world_mount_transform_preserves_fk_and_no_tool(definition):
    world = make_transform(rotation_matrix_from_rpy(.2, -.3, .4), [-1.85, .15, .9])
    robot = cad.load_preview_robot(ROOT, world_from_mount=world)
    q = [.2, -.4, .6, -.8, 1., -.3]
    actual, expected = robot.named_link_frames(q), cad.poe_fk(definition, q, world)
    for name in actual:
        np.testing.assert_allclose(actual[name], expected[name], atol=2e-11, rtol=0)
    np.testing.assert_array_equal(robot.tip_from_tcp, np.eye(4))
    np.testing.assert_allclose(robot.fk(q), actual["s50_flange"], atol=1e-12)
    assert definition["tool_attached"] is False
    assert definition["frames"]["physical_tool_mount"] is None
    assert definition["frames"]["controller_tool0"] is None
    assert definition["frames"]["virtual_tcp"] is None


def test_preview_constraints_are_not_hardware_or_planning_qualification(definition):
    config = yaml.safe_load((ROOT / cad.PREVIEW_CONFIG).read_text(encoding="utf-8"))
    assert config["model_usage"] == "cad_preview_fk_only"
    assert config["preview_only"]["mechanical_limits"] is False
    assert config["readiness"]["operational_planning_ready"] is False
    assert definition["simulation_convention"]["q0_rad"] == [0.]*6
    assert definition["simulation_convention"]["controller_zero_verified"] is False
    assert definition["preview_constraints"]["position_range_semantics"] == "DISPLAY_ONLY_NOT_MECHANICAL_LIMITS"
    xml = ET.parse(ROOT / definition["urdf_path"]).getroot()
    assert not xml.findall(".//inertial")
    assert all(float(limit.get("effort")) == 0. for limit in xml.findall(".//limit"))


def test_link_origin_choice_stays_on_each_measured_infinite_axis(definition):
    for joint in definition["joints"]:
        delta = np.asarray(joint["point_CAD_m"])-joint["measured_line_point_CAD_m"]
        np.testing.assert_allclose(np.cross(delta, joint["axis_CAD"]), 0., atol=1e-14)
        np.testing.assert_allclose(np.dot(delta, joint["axis_CAD"]), joint["line_parameter_shift_m"], atol=1e-14)


def test_missing_raw_link_mesh_is_verified_and_materialized(tmp_path, monkeypatch):
    relative = cad.ASSET_DIRECTORY + "/meshes/link-local/test_only.obj"
    path = tmp_path / relative
    path.parent.mkdir(parents=True)
    raw = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    path.with_name(path.name+".gz").write_bytes(gzip.compress(raw, mtime=0))
    mesh = dict(path=relative, sha256=cad.sha256(raw))
    fixture = dict(source_files={}, definition_sha256="test_only", links=[dict(visual_mesh=mesh, collision_meshes=[mesh])])
    for kind in ("urdf", "srdf"):
        name = cad.ASSET_DIRECTORY + "/urdf/test_only." + kind
        target = tmp_path / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(b"<robot name='test_only' />")
        fixture[kind+"_path"] = name
        fixture[kind+"_sha256"] = cad.sha256(target.read_bytes())
    monkeypatch.setattr(cad, "load_definition", lambda root: fixture)
    result = cad.ensure_preview_assets(tmp_path)
    assert result["materialized"] == [relative]
    assert path.read_bytes() == raw
    path.write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="LINK_MESH_CHANGED"):
        cad.ensure_preview_assets(tmp_path)


def test_changed_urdf_is_rejected_before_preview_is_loaded(tmp_path, monkeypatch):
    relative = cad.ASSET_DIRECTORY + "/urdf/test_only.urdf"
    path = tmp_path / relative
    path.parent.mkdir(parents=True)
    original = b"<robot name='original' />"
    path.write_bytes(b"<robot name='changed_axis_at_same_q0' />")
    fixture = dict(urdf_path=relative, urdf_sha256=cad.sha256(original))
    monkeypatch.setattr(cad, "load_definition", lambda root: fixture)
    with pytest.raises(ValueError, match="URDF_IDENTITY_CHANGED"):
        cad.ensure_preview_assets(tmp_path)


def test_invalid_configuration_and_wrong_chain_source_fail_closed(definition):
    with pytest.raises(ValueError, match="SIX_FINITE"):
        cad.chain_fk(definition, [0.]*5)
    with pytest.raises(ValueError, match="SIX_FINITE"):
        cad.poe_fk(definition, [float("nan")]*6)
    measurements = json.loads((ROOT / cad.ASSET_DIRECTORY / "derived/interface_measurements.json").read_text(encoding="utf-8"))
    audit = json.loads((ROOT / cad.ASSET_DIRECTORY / "derived/cad_audit.json").read_text(encoding="utf-8"))
    wrong = deepcopy(measurements); wrong["axes"][2]["parent_cad_body"] = "j1_s50"
    with pytest.raises(ValueError, match="CHAIN_ORDER_MISMATCH"):
        cad.build_definition(wrong, audit)
    wrong = deepcopy(measurements); wrong["source_sha256"] = "not-s50"
    with pytest.raises(ValueError, match="SOURCE_MISMATCH"):
        cad.build_definition(wrong, audit)
