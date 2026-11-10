"""Missing real model data must stop before any native search or checkpoint use."""
import importlib.util
from pathlib import Path
from copy import deepcopy
import runpy
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from unloading_sim.robot_model_binding import RobotModelBinding

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("s50_admission", ROOT / "tools/check_s50_integration.py")
admission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(admission)


@pytest.mark.skipif(
    not (ROOT / 'assets/robots/huayan_s50_2000/source/S50.STEP').is_file(),
    reason='Requires the real S50 STEP asset; missing assets have a separate fail-closed regression',
)
def test_supplied_cad_is_not_a_qualified_motion_model():
    report = admission.audit(ROOT / "configs/robots/huayan_s50_2000.yaml")
    assert report["cad"]["source_hash_verified"]
    assert report["layout_difference"]["passed"]
    assert len(report["initial_carton_ids"]) == 40
    assert report["imported_completed_count"] == 0
    assert report["initial_q_rad"] is None
    assert report["full_scene_fingerprint"] is None
    assert report["status"] == "MODEL_NOT_READY"
    assert "kinematics.axes_zero_offsets_signs_source" in report["missing_critical_data"]
    assert not report["native_worker_launched"]
    assert report["native_single_carton"]["planning_attempts"] == 0
    assert report["native_single_carton"]["global_unreachable"] is False
    assert report["native_single_carton"]["target"] in report["initial_carton_ids"]
    assert report["payload"]["tool_mass_kg"] == 20.0
    assert report["payload"]["carton_mass_kg"] == 42.5
    assert report["payload"]["rated_robot_payload_kg"] == 50.0
    assert report["payload"]["nominal_mass_exceeds_rating"]
    assert not report["payload"]["blocks_geometric_search"]
    assert not report["dynamics"]["missing_dynamic_data_blocks_geometric_search"]


def test_missing_cad_fails_closed_before_any_native_search(monkeypatch):
    cad = ROOT / 'assets/robots/huayan_s50_2000/source/S50.STEP'
    original_is_file = Path.is_file
    monkeypatch.setattr(Path, 'is_file', lambda path: False if path == cad else original_is_file(path))
    report = admission.audit(ROOT / 'configs/robots/huayan_s50_2000.yaml')
    assert report['status'] == 'MODEL_NOT_READY'
    assert report['cad']['source_hash_verified'] is False
    assert 'source.STEP_SHA256_NOT_VERIFIED' in report['missing_critical_data']
    assert report['native_worker_launched'] is False
    assert report['native_single_carton']['planning_attempts'] == 0


def test_admission_entry_returns_nonzero_for_incomplete_model(tmp_path):
    assert admission.main(["--output", str(tmp_path / "admission.json")]) == 2
    assert (tmp_path / "admission.json").is_file()


def test_empty_evidence_and_invalid_tool_frame_do_not_qualify():
    invalid = np.eye(4); invalid[0, 3] = float('nan')
    data = dict(kinematics=dict(zero_pose_reference='', axes_zero_offsets_signs_source={'source': ''}),
        collision=dict(per_link_visual_collision_sources={}, reviewed_self_collision_pairs=[], owned_tool_exempt_robot_links=[]),
        tool_interface=dict(flange_from_project_tool_mount=invalid.tolist(), mounting_geometry_source='', additional_physical_adapter=False),
        joint_velocity_version_conflict={'resolution': 'Unresolved'})
    errors = admission.qualification_errors(data, None)
    assert 'kinematics.zero_pose_reference' in errors
    assert 'kinematics.axes_zero_offsets_signs_source' in errors
    assert 'collision.per_link_visual_collision_sources' in errors
    assert 'tool_interface.INVALID_FLANGE_FROM_PROJECT_TOOL_MOUNT' in errors
    assert 'joint_velocity_version_conflict.verified_resolution_source' in errors
    assert 'collision.reviewed_self_collision_pairs' not in errors  # Explicit empty exclusions are valid.


def test_provenance_labels_cannot_replace_per_link_geometry(tmp_path):
    # The analytical fixture remains named as a fixture, never S50.
    fixture = runpy.run_path(str(ROOT/'tests/test_robot_model_binding.py'))['fixture_config']
    config = fixture.__wrapped__(tmp_path)
    binding = RobotModelBinding.from_config(config)
    data = deepcopy(binding.data)
    data['kinematics'].update(zero_pose_reference='analytical fixture only', axes_zero_offsets_signs_source='analytical fixture only')
    names = [binding.base_link, *binding.chain_child_links]
    data['collision'].update(per_link_visual_collision_sources={name: 'fixture geometry' for name in names}, reviewed_self_collision_pairs=[])
    data['tool_interface'] = dict(flange_from_project_tool_mount=np.eye(4).tolist(), mounting_geometry_source='fixture only', additional_physical_adapter=False)
    errors = admission.qualification_errors(data, binding)
    assert len([error for error in errors if 'LINK_GEOMETRY_MISSING' in error]) == 14
    xml = ET.parse(binding.urdf_path).getroot()
    for name in names:
        for role in ('visual', 'collision'):
            shape = ET.SubElement(xml.find(f"link[@name='{name}']"), role)
            ET.SubElement(ET.SubElement(shape, 'geometry'), 'box', size='0.1 0.1 0.1')
    binding.urdf_path.write_text(ET.tostring(xml, encoding='unicode'))
    binding = RobotModelBinding.from_config(config)
    assert admission.qualification_errors(data, binding) == []
    data['collision']['reviewed_self_collision_pairs'] = [[names[0], names[1]]]
    assert 'collision.REVIEWED_SRDF_PAIR_MISMATCH' in admission.qualification_errors(data, binding)
