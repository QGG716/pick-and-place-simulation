import copy
import importlib
from pathlib import Path

import pytest
import yaml

from unloading_sim.identity import (
    CACHE_IDENTITY_FIELDS,
    build_scene_cache_identity,
    load_tool_config,
    normalize_robot_model_id,
    validate_cache_identity,
)
from unloading_sim.reporting import build_report_context, qualification_decision, qualification_decision_reason
from unloading_sim.robot_load import load_qualification_case
from unloading_sim.scene import load_scene_config


ROOT = Path(__file__).resolve().parents[1]


def test_studies_package_imports_in_default_pytest_environment():
    module = importlib.import_module("studies.fanuc_m20id35_cross_section.model")
    assert module.__name__.startswith("studies.")


@pytest.mark.parametrize(
    ("model_id", "canonical"),
    [
        ("fanuc_m20id35", "fanuc_m20id_35"),
        ("fanuc_m20id_35", "fanuc_m20id_35"),
        ("kuka_kr50_r2500", "kuka_kr50_r2500"),
        ("ur5e_like", "ur5e_like"),
    ],
)
def test_robot_model_normalization(model_id, canonical):
    assert normalize_robot_model_id(model_id) == canonical


def test_unknown_robot_model_is_rejected():
    with pytest.raises(ValueError, match="unknown robot model ID"):
        normalize_robot_model_id("fanuc_typo")


def test_default_tool_alias_resolves_to_single_10kg_master():
    alias_path = ROOT / "configs/tools/unloading_gripper.yaml"
    alias = yaml.safe_load(alias_path.read_text(encoding="utf-8"))
    tool = load_tool_config(alias_path)
    assert set(alias) == {"schema_version", "extends", "description"}
    assert tool.config_path == (ROOT / "configs/tools/unloading_gripper_10kg.yaml").resolve()
    assert tool.mass_kg == 10.0
    assert tool.mass_properties_source == "ENGINEERING_MODEL"
    assert tool.source["measured_status"] == "NOT_MEASURED"
    assert tool.source["manufacturer_load_certificate_status"] == "NOT_EVALUATED"


def test_8kg_tool_remains_explicit_historical_variant():
    tool = load_tool_config(ROOT / "configs/tools/unloading_gripper_8kg.yaml")
    assert tool.mass_kg == 8.0
    assert tool.name.endswith("_8kg")
    assert tool.source["type"] == "scaled_supplied_STEP_engineering_estimate"


def test_runtime_and_qualification_share_physical_10kg_tool():
    _, config = load_scene_config(ROOT / "config/fanuc_m20id35.yaml")
    case = load_qualification_case(ROOT / "configs/qualification/fanuc_m20id35_25kg_front.yaml")
    assert config["robot"]["model"] == "fanuc_m20id_35"
    assert config["tool"]["resolved_config_path"] == case.tool.source["resolved_config_path"]
    assert config["tool"]["config_hash"] == case.tool.source["config_sha256"]
    assert config["tool"]["mass_kg"] == case.tool.mass_kg == 10.0


def test_cache_identity_has_robot_tool_and_tcp_and_rejects_mismatch():
    _, config = load_scene_config(ROOT / "config/fanuc_m20id35.yaml")
    identity = build_scene_cache_identity(config)
    assert set(CACHE_IDENTITY_FIELDS).issubset(identity)
    assert identity["robot_model_id"] == "fanuc_m20id_35"
    assert identity["tool_mass_kg"] == 10.0
    validate_cache_identity(identity, identity)

    wrong_tool = copy.deepcopy(identity)
    wrong_tool["tool_mass_kg"] = 8.0
    with pytest.raises(ValueError, match="tool_mass_kg"):
        validate_cache_identity(wrong_tool, identity)

    wrong_robot = copy.deepcopy(identity)
    wrong_robot["robot_kinematic_hash"] = "0" * 64
    with pytest.raises(ValueError, match="robot_kinematic_hash"):
        validate_cache_identity(wrong_robot, identity)


def test_report_context_uses_resolved_inputs_and_dynamic_counts():
    context = build_report_context(
        robot="fanuc_m20id35",
        tool="configs/tools/unloading_gripper.yaml",
        root=ROOT,
    )
    assert context["robot_model_id"] == "fanuc_m20id_35"
    assert context["tool_name"].endswith("_10kg")
    assert Path(context["resolved_paths"]["tool_resolved_config"]).name == "unloading_gripper_10kg.yaml"
    assert len(context["input_hashes"]["tool_resolved_config_sha256"]) == 64
    reason = qualification_decision_reason(
        {"required_25kg_known_failures": 2, "required_25kg_cases": 9, "official_com_curve_status": "NOT_EVALUATED"},
        {"task_reachable_rate": 0.25},
        {"boxes_per_hour": 812.5},
    )
    assert "2/9" in reason
    assert "0.25" in reason
    assert "812.5" in reason
    assert qualification_decision(
        {"required_25kg_known_failures": 2, "required_25kg_cases": 9, "official_com_curve_status": "PASS"},
        {"task_reachable_rate": 1.0},
    ) == "NOT_QUALIFIED"
