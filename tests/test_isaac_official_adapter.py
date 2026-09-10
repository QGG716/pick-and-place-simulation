from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import pytest
import yaml

from unloading_sim.independent_cups import build_m710_independent_cup_array
from unloading_sim.isaac_bridge import (
    _transform_mass_properties,
    _validated_ideal_cup_selection,
    _validated_official_model_manifest,
    _validated_place_evidence,
)


ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_MANIFEST = (
    ROOT / "assets/robots/fanuc_m710id_70/official/provenance.yaml"
)


def test_official_manifest_drives_joint_limits_and_full_link_inertials():
    manifest = yaml.safe_load(OFFICIAL_MANIFEST.read_text(encoding="utf-8"))

    model = _validated_official_model_manifest(manifest)

    assert model["expanded_urdf"].endswith("m710id_70_official.urdf")
    assert model["joint_names"] == ("J1", "J2", "J3", "J4", "J5", "J6")
    assert model["joint_velocity_rad_s"] == pytest.approx(
        [np.pi, np.pi, np.pi, np.deg2rad(260), np.deg2rad(260), np.deg2rad(370)]
    )
    assert model["joint_effort_nm"] == pytest.approx(
        [8000.0, 10000.0, 5000.0, 2000.0, 1000.0, 900.0]
    )
    assert {item["link"] for item in model["link_dynamics"]} == {
        "base_link",
        "J1_link",
        "J2_link",
        "J3_link",
        "J4_link",
        "J5_link",
        "J6_link",
    }
    assert sum(item["mass_kg"] for item in model["link_dynamics"]) == pytest.approx(
        580.347
    )
    j2 = next(item for item in model["link_dynamics"] if item["link"] == "J2_link")
    assert j2["com_xyz_m"] == pytest.approx([-0.0291, 0.17, 0.4])
    np.testing.assert_allclose(
        np.asarray(j2["inertia_at_com_kg_m2"]),
        [[13.3, 0.0925, -0.0231], [0.0925, 14.2, -1.14], [-0.0231, -1.14, 1.9]],
        rtol=0.0,
        atol=1e-12,
    )


def test_official_manifest_fails_closed_on_changed_commit_or_source_hash():
    manifest = yaml.safe_load(OFFICIAL_MANIFEST.read_text(encoding="utf-8"))
    wrong_commit = copy.deepcopy(manifest)
    wrong_commit["upstream"]["commit"] = "0" * 40
    with pytest.raises(ValueError, match="fixed commit"):
        _validated_official_model_manifest(wrong_commit)

    wrong_hash = copy.deepcopy(manifest)
    wrong_hash["source_files"][0]["sha256"] = "not-a-hash"
    with pytest.raises(ValueError, match="source-file sha256"):
        _validated_official_model_manifest(wrong_hash)


def test_ideal_cup_bundle_keeps_three_masks_and_requires_actual_fk_endpoint():
    cup_ids = list(build_m710_independent_cup_array().cup_ids)
    eligible = [False] * 72
    commanded = [False] * 72
    contact = [False] * 72
    eligible[7] = True
    commanded[7] = True
    contact[7] = True
    path = np.asarray([[0.0] * 6, [0.1] * 6, [0.2] * 6])
    cup_selection = {
        "target_id": "carton_r00_c00",
        "target_face": "front",
        "pose_source": "actual_fk_virtual_tcp_converted_to_physical_contact",
        "mask_bit_order_cup_ids": cup_ids,
        "geometrically_eligible_mask": eligible,
        "commanded_active_mask": commanded,
        "actual_contact_mask": contact,
        "actual_q_rad": path[1].tolist(),
    }
    segment = {
        "target": "carton_r00_c00",
        "grasp_index": 1,
        "contact": {"cup_selection": cup_selection},
    }

    selection = _validated_ideal_cup_selection(
        segment, physical_cup_count=72, path=path
    )

    assert selection["commanded_active_mask"] == commanded
    assert selection["planned_fk_contact_mask"] == contact
    assert selection["actual_contact_mask"] == [False] * 72
    assert selection["actual_contact_mask_source"] == "ISAAC_RUNTIME_ACTUAL_STATE_REQUIRED"
    assert selection["load_bearing_minimum_cup_count"] is None

    stale = copy.deepcopy(segment)
    stale["contact"]["cup_selection"]["actual_q_rad"] = [0.0] * 6
    with pytest.raises(ValueError, match="selected grasp path endpoint"):
        _validated_ideal_cup_selection(stale, physical_cup_count=72, path=path)

    lost_commanded_contact = copy.deepcopy(segment)
    lost_commanded_contact["contact"]["cup_selection"]["commanded_active_mask"][8] = True
    lost_commanded_contact["contact"]["cup_selection"]["geometrically_eligible_mask"][8] = True
    with pytest.raises(ValueError, match="every commanded cup"):
        _validated_ideal_cup_selection(
            lost_commanded_contact, physical_cup_count=72, path=path
        )

    conflicting_alias = copy.deepcopy(segment)
    conflicting_alias["ideal_independent_cups"] = copy.deepcopy(cup_selection)
    conflicting_alias["ideal_independent_cups"]["target_face"] = "top"
    with pytest.raises(ValueError, match="exactly equal"):
        _validated_ideal_cup_selection(
            conflicting_alias, physical_cup_count=72, path=path
        )


def test_fixed_tool_mass_properties_rotate_and_translate_into_last_moving_link():
    quarter_turn_z = np.asarray(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    transform = np.eye(4)
    transform[:3, :3] = quarter_turn_z
    transform[:3, 3] = [0.2, 0.0, 0.0]

    center, inertia = _transform_mass_properties(
        [0.1, 0.0, 0.0], np.diag([1.0, 2.0, 3.0]), transform
    )

    assert center == pytest.approx([0.2, 0.1, 0.0])
    np.testing.assert_allclose(inertia, np.diag([2.0, 1.0, 3.0]), rtol=0.0, atol=1e-12)


def test_structured_place_evidence_is_authoritative_over_legacy_aliases():
    pose = np.eye(4)
    pose[:3, 3] = [0.8, -0.4, 0.75]
    support = {
        "supported": True,
        "bottom_gap_m": 0.0,
        "penetration_m": 0.0,
        "edge_clearance_xy_m": [0.1, 0.1],
        "bottom_face_coplanar": True,
        "actual_box_pose": pose.tolist(),
    }
    segment = {
        "place": {
            "receiver": "conveyor_transverse",
            "place_surface": "conveyor_transverse",
            "actual_box_pose_world": pose.tolist(),
            "release_center_world_m": pose[:3, 3].tolist(),
            "support": support,
        },
        "place_surface": "conveyor_transverse",
        "place_center": pose[:3, 3].tolist(),
        "release_center": pose[:3, 3].tolist(),
    }

    result = _validated_place_evidence(segment)

    assert result["place_surface"] == "conveyor_transverse"
    assert result["release_center_m"] == pytest.approx(pose[:3, 3])
    assert result["planned_support_audit"] == support

    stale = copy.deepcopy(segment)
    stale["release_center"] = [0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="legacy release_center alias"):
        _validated_place_evidence(stale)


def test_replay_source_uses_drive_targets_and_actual_contact_release_gates():
    source = (ROOT / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8")
    loop = source[source.index("for step in range(physics_steps):") :]

    assert "articulation.set_dof_position_targets(command[None, :])" in loop
    assert "articulation.set_dof_positions(" not in loop
    assert "audit_actual_independent_cup_contacts(" in loop
    assert "actual_contact_mask == list(commanded_cup_mask)" in loop
    assert "CreateBody1Rel().SetTargets([Sdf.Path(target_carton_path)])" in loop
    assert "support_contact_report_observed" in loop
    assert loop.index("support_release_accepted") < loop.index("stage.RemovePrim(grasp_joint_path)")
    assert "carton_states.json" in loop
    assert "execution_events.json" in loop
    assert 'os.environ["ROS_PACKAGE_PATH"]' in source
    assert "official expanded URDF bytes differ from the pinned manifest" in source
    assert "official-model SRDF bytes differ from the replay contract" in source
    assert "get_dof_coriolis_and_centrifugal_compensation_forces()" in source
