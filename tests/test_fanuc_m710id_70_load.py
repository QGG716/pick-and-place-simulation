from pathlib import Path

import numpy as np
import yaml

from unloading_sim.fanuc_m710id70 import (
    HOME_JOINTS_RAD,
    build_robot,
    combined_payload_properties,
    wrist_load_at_pose,
)
from unloading_sim.identity import load_tool_config, normalize_robot_model_id
from unloading_sim.robot_load import load_robot_limits
from unloading_sim.robot_load.vendor import load_fanuc_wrist_evidence


ROOT = Path(__file__).resolve().parents[1]


def test_m710_identity_and_official_limits_load():
    assert normalize_robot_model_id("fanuc_m710id_70") == "fanuc_m710id_70"
    limits = load_robot_limits(ROOT / "configs/robots/fanuc_m710id_70.yaml")
    assert limits.rated_payload_kg == 70.0
    assert limits.reach_m == 2.104
    assert limits.allowable_moment_nm == (300.0, 300.0, 150.0)
    assert limits.allowable_inertia_kg_m2 == (30.0, 30.0, 15.0)


def test_20kg_tool_is_separate_and_combined_com_fails_70kg_curve():
    tool = load_tool_config(ROOT / "configs/tools/unloading_gripper_20kg.yaml")
    assert tool.mass_kg == 20.0
    result = combined_payload_properties(tool, 42.5, [0.6, 0.4, 0.3])
    assert result["total_external_mass_kg"] == 62.5
    assert np.isclose(result["payload_utilization"], 62.5 / 70.0)
    assert np.isclose(result["combined_com_flange_xyz_m"][0], 0.42193923136)
    assert result["vendor_com_diagram_status"] == "FAIL"
    assert result["vendor_diagram_coordinates_m"]["axial_z"] > result["conservative_70kg_limits_m"]["axial_z"]


def test_vendor_evidence_chain_is_verified_but_inverse_dynamics_is_blocked():
    evidence = load_fanuc_wrist_evidence(ROOT / "configs/vendor/fanuc_m710id_70_load_evidence.yaml")
    assert evidence.reference.evidence_status.value == "VERIFIED"
    assert evidence.load_diagram_status.value == "VERIFIED"
    robot_document = yaml.safe_load((ROOT / "configs/robots/fanuc_m710id_70.yaml").read_text(encoding="utf-8"))
    assert robot_document["joint_effort_limits_nm"]["values"] is None
    assert robot_document["source_metadata"]["dynamics_inertial_status"] == "BLOCKED_BY_MISSING_INERTIAL_DATA"


def test_payload_only_wrist_screen_uses_real_pose_and_dynamic_state():
    robot, _tool = build_robot(-0.70)
    result = wrist_load_at_pose(
        robot,
        HOME_JOINTS_RAD,
        ROOT / "configs/tools/unloading_gripper_20kg.yaml",
        qd_rad_s=np.full(6, 0.10),
        qdd_rad_s2=np.full(6, 0.20),
    )
    assert len(result["dynamic_moment_utilization_J4_J5_J6"]) == 3
    assert len(result["inertia_utilization_J4_J5_J6"]) == 3
    assert all(value >= 0.0 for value in result["dynamic_moment_utilization_J4_J5_J6"])
    assert result["method"].startswith("payload_only")


def test_com_offset_sensitivity_is_limited_to_requested_ten_percent():
    tool = load_tool_config(ROOT / "configs/tools/unloading_gripper_20kg.yaml")
    nominal = combined_payload_properties(tool, 42.5, [0.6, 0.4, 0.3])
    shifted = combined_payload_properties(tool, 42.5, [0.6, 0.4, 0.3], [0.10, 0.0, 0.0])
    assert shifted["combined_com_flange_xyz_m"][0] > nominal["combined_com_flange_xyz_m"][0]
