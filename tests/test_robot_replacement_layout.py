"""Literal frozen-layout contracts; no fixture here qualifies an S50 model."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from unloading_sim.robot_model_binding import RobotModelBinding
from unloading_sim.robot_replacement_layout import (
    BASELINE_COMMIT, build_replacement_layout, compare_non_robot_scenes,
    freeze_non_robot_scene, frozen_baseline_layout, pending_s50_definition,
)
from unloading_sim.workcell_layout import canonical_digest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def frozen():
    return freeze_non_robot_scene(frozen_baseline_layout(ROOT))


def _rehash(snapshot):
    snapshot.pop("environment_fingerprint", None)
    snapshot["environment_fingerprint"] = canonical_digest(snapshot)
    return snapshot


@pytest.fixture
def ready_test_definition(tmp_path):
    """Synthetic chain for adapter tests, explicitly never named S50."""
    mesh = tmp_path / "test_geometry.stl"
    mesh.write_text("solid test_geometry\nendsolid test_geometry\n", encoding="utf-8")
    links = ['<link name="base"><visual><geometry><mesh filename="test_geometry.stl"/></geometry></visual></link>']
    joints = []
    for index in range(1, 7):
        links.append(f'<link name="link_{index}"/>')
        parent = "base" if index == 1 else f"link_{index-1}"
        joints.append(f'<joint name="axis_{index}" type="revolute">'
            f'<parent link="{parent}"/><child link="link_{index}"/>'
            '<origin xyz="0.1 0 0.1"/><axis xyz="0 0 1"/>'
            '<limit lower="-2" upper="2" velocity="0.5" effort="5"/></joint>')
    (tmp_path / "test.urdf").write_text('<robot name="test_only_serial_fixture">' + ''.join(links+joints) + '</robot>', encoding="utf-8")
    (tmp_path / "test.srdf").write_text('<robot name="test_only_serial_fixture"><group name="manipulator">'
        '<chain base_link="base" tip_link="link_6"/></group></robot>', encoding="utf-8")
    mount = np.eye(4); mount[2, 3] = .1
    config = dict(model_id="test_only_serial_fixture", kinematics=dict(
        urdf_path="test.urdf", srdf_path="test.srdf", active_joint_names=[f"axis_{i}" for i in range(1, 7)],
        base_link="base", flange_link="link_6", tip_link="link_6", mounting_from_base=mount.tolist()),
        tool_interface=dict(flange_from_project_tool_mount=np.eye(4).tolist()))
    path = tmp_path / "test.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    binding = RobotModelBinding.from_config(path)
    return dict(model_id=binding.model_id, dof=6, model_config=str(path), binding=binding.identity(),
        model_asset_identity={"source": "SYNTHETIC_ADAPTER_TEST_NOT_S50"},
        mounting_from_urdf_base=mount.tolist(), base_support_bounds=[[-.12, -.14, -.1], [.12, .14, .1]],
        planning_ready=True)


def test_confirmed_layout_numeric_contract(frozen):
    p = frozen["parameters"]
    assert frozen["baseline_commit"] == BASELINE_COMMIT
    assert p["gap_m"] == pytest.approx(.7)
    assert p["mounting_surface_height_m"] == pytest.approx(.9)
    np.testing.assert_allclose(np.array(p["world_from_physical_mount"])[:3, 3], [-1.85, .15, .9], atol=1e-12)
    np.testing.assert_allclose(np.array(p["world_from_physical_mount"])[:3, :3], np.eye(3), atol=1e-12)
    assert p["mount_axis_to_chassis_front_m"] == pytest.approx(.45)
    assert p["minimum_belt_height_m"] == .55
    assert p["maximum_belt_height_m"] == 1.5
    assert p["initial_belt_height_m"] == .6
    assert p["tool_extension_m"] == .2
    fields = frozen["layout_fields"]
    assert fields["world"]["axes"] == dict(x="into_trailer", y="left_when_facing_into_trailer", z="up")
    assert fields["chassis"]["size_xyz_m"] == [2.1, 1.5, .6]
    assert fields["conveyors"]["transverse"]["size_xy_m"] == [.7, 2.2]
    assert fields["conveyors"]["longitudinal"]["size_xy_m"] == [2.8, .6]
    assert fields["conveyors"]["body_thickness_m"] == .12
    assert fields["trailer"]["inner_width_m"] == 2.3
    assert fields["trailer"]["height_m"] == 2.7
    assert fields["trailer"]["opening_x_m"] == -3.2
    assert fields["trailer"]["closed_end_wall_x_m"] == 3.2


def test_fresh_carton_identities_and_tool_target(frozen):
    assert frozen["initial_carton_ids"] == [f"carton_l{layer:02}_c{column:02}" for layer in range(8) for column in range(5)]
    assert frozen["imported_completed_count"] == 0
    assert frozen["assumed_removed"] == []
    assert frozen["initial_robot_q"] is None
    assert all(box["size_m"] == [.6, .4, .3] for box in frozen["initial_cartons"])
    np.testing.assert_array_equal(frozen["initial_physical_contact_target"],
        [[0., 0., 1., -.2], [0., -1., 0., 0.], [1., 0., 0., 1.5], [0., 0., 0., 1.]])
    tool = frozen["tool_intrinsic"]
    assert tool["tool_mass_kg"] == 20.
    assert tool["carton_mass_kg"] == 42.5
    assert np.asarray(tool["rigid_local_boxes_virtual_tcp_m"]).shape == (130, 6)
    assert np.asarray(tool["compliant_local_boxes_virtual_tcp_m"]).shape == (72, 6)
    frames = tool["intrinsic_frames"]
    assert np.array(frames["T_tool_mount_reference_virtual_task_tcp"])[0, 3] == pytest.approx(.45)
    assert np.array(frames["T_tool_mount_reference_uncompressed_cup_plane"])[0, 3] == pytest.approx(.4275)
    assert np.array(frames["T_tool_mount_reference_nominal_compressed_contact"])[0, 3] == pytest.approx(.4175)
    assert tool["robot_dependent_world_tool_pose_excluded"] is True


def test_pending_model_freezes_environment_without_model_acceptance(frozen):
    replacement = build_replacement_layout(root=ROOT)
    comparison = compare_non_robot_scenes(frozen, freeze_non_robot_scene(replacement))
    assert comparison["passed"] is True
    assert comparison["sampled_heights_m"] == [.55, .6, 1., 1.5]
    assert comparison["checked_entities_per_height"] == [148, 148, 148, 148]
    assert comparison["initial_carton_count"] == 40
    assert comparison["model_readiness_evaluated"] is False
    assert replacement.full_scene_fingerprint is None
    assert replacement.layout_fingerprint is None
    assert replacement.configuration_identity != replacement.reference_layout.layout_fingerprint
    assert replacement.data["robot"]["base_support_bounds"] is None
    assert "base_support_bbox_min_xyz_m" not in replacement.data["robot"]
    assert "base_front_edge_x_world_m" not in replacement.data["robot"]
    assert "positioning_basis" not in replacement.data["robot"]
    with pytest.raises(ValueError, match="MISSING_MOUNT_FRAME_ALIGNMENT"):
        replacement.robot_base_transform()
    with pytest.raises(ValueError, match="MODEL_BINDING_NOT_QUALIFIED"):
        replacement.robot()


def test_urdf_origin_alignment_never_moves_installation_or_plinth(frozen):
    definition = pending_s50_definition()
    transform = np.eye(4); transform[2, 3] = .1
    definition.update(mounting_from_urdf_base=transform.tolist(),
        base_support_bounds=[[-.12, -.14, -.1], [.12, .14, .1]])
    replacement = build_replacement_layout(definition, root=ROOT)
    np.testing.assert_allclose(replacement.robot_base_transform()[:3, 3], [-1.85, .15, 1.])
    np.testing.assert_allclose(replacement.physical_mount_transform()[:3, 3], [-1.85, .15, .9])
    assert replacement.data["robot"]["derived_base_front_edge_x_world_m"] == pytest.approx(-1.73)
    assert compare_non_robot_scenes(frozen, freeze_non_robot_scene(replacement))["passed"]
    plinth = next(b for b in replacement.fixed_components() if b.name == "mount_plinth")
    np.testing.assert_allclose(2*plinth.half_extents, [.5635, .55, .3], atol=1e-12)
    np.testing.assert_allclose(plinth.center, [-1.90675, .15, .75], atol=1e-12)
    assert replacement.full_scene_fingerprint is None


@pytest.mark.parametrize("location", ["entity_size", "entity_pose", "stroke", "mechanism", "tool"])
def test_detailed_diff_detects_non_robot_drift_even_with_recomputed_hash(frozen, location):
    changed = deepcopy(frozen)
    if location == "entity_size":
        changed["height_samples"][1]["entities"][0]["size_m"][0] += .001
    elif location == "entity_pose":
        changed["height_samples"][1]["entities"][0]["world_from_entity"][1][3] += .001
    elif location == "stroke":
        changed["layout_fields"]["conveyors"]["coupling"]["maximum_surface_height_world_m"] = 1.6
    elif location == "mechanism":
        changed["height_samples"][2]["mechanism"]["unapproved_motion_change"] = True
    else:
        changed["tool_intrinsic"]["rigid_local_boxes_virtual_tcp_m"][0][3] += .001
    result = compare_non_robot_scenes(frozen, _rehash(changed))
    assert result["status"] == "NON_ROBOT_LAYOUT_CHANGED"
    assert not result["passed"]
    assert result["differences"]


def test_tampered_snapshot_is_rejected(frozen):
    changed = deepcopy(frozen)
    changed["initial_cartons"].pop()
    with pytest.raises(ValueError, match="FROZEN_ENVIRONMENT_FINGERPRINT_MISMATCH"):
        compare_non_robot_scenes(frozen, changed)


def test_readiness_boolean_cannot_manufacture_complete_scene():
    definition = pending_s50_definition(); definition["planning_ready"] = True
    with pytest.raises(ValueError, match="INCOMPLETE_GEOMETRY_BINDING"):
        build_replacement_layout(definition, root=ROOT)


def test_real_ready_adapter_path_uses_new_binding_and_tool(frozen, ready_test_definition):
    layout = build_replacement_layout(ready_test_definition, root=ROOT)
    robot = layout.robot()
    assert robot.name == "test_only_serial_fixture"
    assert robot.active_joint_names == [f"axis_{i}" for i in range(1, 7)]
    np.testing.assert_allclose(robot.base_transform[:3, 3], [-1.85, .15, 1.])
    np.testing.assert_allclose(robot.tip_from_tcp[0, 3], .45)
    assert layout.full_scene_fingerprint and layout.full_scene_fingerprint != layout.reference_layout.layout_fingerprint
    assert compare_non_robot_scenes(frozen, freeze_non_robot_scene(layout))["passed"]
    assert robot.tool_compliant_collision_local_boxes.shape == (72, 6)
    assert "mesh:test_geometry.stl" in layout.definition["verified_runtime_asset_sha256"]


def test_s50_carton_mass_enters_scene_identity_and_ready_scene_requires_it(ready_test_definition, monkeypatch):
    import unloading_sim.s50_payload as payload
    pending = build_replacement_layout(root=ROOT)
    ready = build_replacement_layout(ready_test_definition, root=ROOT)
    def missing(data):
        raise ValueError("S50_CARTON_MASS_NOT_CONFIGURED")
    monkeypatch.setattr(payload, "configured_carton_mass_kg", missing)
    # A pending model stays reportable without the mass, under another identity.
    assert build_replacement_layout(root=ROOT).configuration_identity != pending.configuration_identity
    with pytest.raises(ValueError, match="S50_CARTON_MASS_NOT_CONFIGURED"):
        build_replacement_layout(ready_test_definition, root=ROOT)
    assert ready.full_scene_fingerprint


def test_runtime_mesh_change_invalidates_full_scene_and_loaded_robot(ready_test_definition):
    before = build_replacement_layout(ready_test_definition, root=ROOT)
    path = Path(ready_test_definition["model_config"]).parent / "test_geometry.stl"
    path.write_text("solid changed\nendsolid changed\n", encoding="utf-8")
    with pytest.raises(ValueError, match="REPLACEMENT_MODEL_BINDING_IDENTITY_CHANGED"):
        build_replacement_layout(ready_test_definition, root=ROOT)
    from unloading_sim.robot_model_binding import RobotModelBinding
    ready_test_definition["binding"] = RobotModelBinding.from_config(ready_test_definition["model_config"]).identity()
    after = build_replacement_layout(ready_test_definition, root=ROOT)
    assert before.full_scene_fingerprint != after.full_scene_fingerprint
    with pytest.raises(ValueError, match="REPLACEMENT_MODEL_BINDING_IDENTITY_CHANGED"):
        before.robot()


def test_current_binding_must_match_recorded_binding(ready_test_definition):
    ready_test_definition["binding"]["urdf_sha256"] = "stale"
    with pytest.raises(ValueError, match="REPLACEMENT_MODEL_BINDING_IDENTITY_CHANGED"):
        build_replacement_layout(ready_test_definition, root=ROOT)
