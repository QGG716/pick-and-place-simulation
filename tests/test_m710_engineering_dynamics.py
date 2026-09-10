from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from unloading_sim.m710_dynamics import (
    ACTIVE_JOINT_NAMES,
    CONVEYOR_SURFACE_NAMES,
    DEFAULT_CONFIG_PATH,
    ENGINEERING_STATUS,
    FIXED_TOOL_MASS_POLICY,
    ROBOT_LINK_NAMES,
    load_m710id70_dynamics,
    load_m710id70_engineering_dynamics,
)
from unloading_sim.m710_official_dynamics import OFFICIAL_STATUS


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/simulation/m710id70_engineering_dynamics_v1.yaml"


def _editable_document(tmp_path: Path) -> dict:
    """Bind the retained v1 estimate to explicit legacy parser fixtures.

    The active layout/model now use official link frames. The proxy estimate
    must never be rebound to that URDF merely to exercise its old validation.
    These minimal fixtures are dynamics inputs, not executable scene snapshots.
    """
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    for name, value in tuple(data["sources"].items()):
        data["sources"][name] = str((CONFIG.parent / value).resolve())
    legacy_urdf = str(ROOT / "assets/robots/fanuc_m710id_70/m710id_70.urdf")
    legacy_model = _write(
        tmp_path,
        {
            "model": "fanuc_m710id_70",
            "kinematics": {
                "active_joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
                "urdf_path": legacy_urdf,
            },
            "joint_velocity_limits_rad_s": [
                3.141592654, 3.141592654, 3.141592654,
                4.537856055, 4.537856055, 6.457718232,
            ],
        },
        "legacy_proxy_model.yaml",
    )
    legacy_layout = _write(
        tmp_path,
        {
            "schema": "m710id70_unloading_layout_v1",
            "layout_id": "m710id70_unloading_layout_v1",
            "robot": {"model_config": str(legacy_model), "urdf_path": legacy_urdf},
            "tool": {"load_config": data["sources"]["tool_config"]},
            "world": {"floor_z_m": 0.0},
            "trailer": {
                "right_wall_y_m": -1.15, "left_wall_y_m": 1.15,
                "length_m": None, "height_m": None,
            },
            "carton_stack": {
                "depth_rows": 1, "width_columns": 5, "height_layers": 8,
                "carton_size_xyz_m": [0.6, 0.4, 0.3],
            },
        },
        "legacy_proxy_layout.yaml",
    )
    data["sources"].update(
        robot_model_config=str(legacy_model),
        robot_urdf=legacy_urdf,
        workcell_layout=str(legacy_layout),
    )
    return data


def _write(tmp_path: Path, data: dict, name: str = "dynamics.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def test_default_contract_selects_official_sources_without_machine_certificate_gate():
    config = load_m710id70_dynamics()
    assert config.config_path == DEFAULT_CONFIG_PATH
    assert DEFAULT_CONFIG_PATH.name == "m710id70_official_dynamics_v2.yaml"
    assert config.qualification_status == OFFICIAL_STATUS
    assert config.data["qualification"]["simulation_input_accepted"] is True
    assert config.machine_qualified is False
    assert config.robot_mass_kg == pytest.approx(580.347, abs=1e-12)
    assert config.robot_with_fixed_tool_mass_kg == pytest.approx(600.347, abs=1e-12)
    assert config.cache_identity["schema_version"] == "m710id70_official_dynamics_v2"
    assert load_m710id70_engineering_dynamics().fingerprint == config.fingerprint
    model_path = (config.config_path.parent / config.data["sources"]["robot_model_config"]).resolve()
    layout_path = (config.config_path.parent / config.data["sources"]["workcell_layout"]).resolve()
    model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    urdf = (config.config_path.parent / config.data["sources"]["robot_urdf"]).resolve()
    assert urdf == (model_path.parent / model["kinematics"]["urdf_path"]).resolve()
    assert urdf == (layout_path.parent / layout["robot"]["urdf_path"]).resolve()


def test_official_dynamics_rejects_mixed_proxy_urdf_source(tmp_path):
    data = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    data["sources"] = {
        name: str((DEFAULT_CONFIG_PATH.parent / value).resolve())
        for name, value in data["sources"].items()
    }
    data["sources"]["robot_urdf"] = str(
        ROOT / "assets/robots/fanuc_m710id_70/m710id_70.urdf"
    )
    with pytest.raises(ValueError, match="sources.robot_urdf does not match robot model config"):
        load_m710id70_dynamics(_write(tmp_path, data, "mixed_sources.yaml"))


def test_legacy_contract_is_complete_immutable_and_mass_is_counted_once(tmp_path):
    config = load_m710id70_engineering_dynamics(_write(tmp_path, _editable_document(tmp_path)))

    assert config.qualification_status == ENGINEERING_STATUS
    assert config.machine_qualified is False
    assert tuple(config.robot_links) == ROBOT_LINK_NAMES
    assert tuple(config.joint_drives) == ACTIVE_JOINT_NAMES
    assert config.robot_mass_kg == pytest.approx(580.0)
    assert config.mechanical_weight_kg == 580.0
    assert config.manufacturer_evidence["link_distribution_status"] == "NOT_PROVIDED_BY_MANUFACTURER"
    assert config.tool.mass_kg == 20.0
    assert config.tool.attach_to_link == "J6_link"
    assert config.tool.body_mode == FIXED_TOOL_MASS_POLICY
    assert config.data["tool"]["mass_accounting"] == FIXED_TOOL_MASS_POLICY
    assert config.data["mass_accounting"]["tool_body"] == FIXED_TOOL_MASS_POLICY
    assert config.tool.source_status == ENGINEERING_STATUS
    assert config.tool.mass_properties_source == "ENGINEERING_MODEL"
    assert config.tool.machine_qualified is False
    assert config.cartons.count == 40
    assert config.cartons.mass_kg_each == 42.5
    assert config.cartons.inventory_mass_kg == 1700.0
    assert config.robot_with_fixed_tool_mass_kg == 600.0
    assert config.total_configured_mass_kg == 2300.0
    assert np.diag(config.cartons.inertia_tensor_com_kg_m2).tolist() == pytest.approx(
        [0.885416666666667, 1.59375, 1.841666666666667]
    )
    assert config.source_file_hashes["tool_config"] == config.tool.config_hash
    assert len(config.fingerprint) == 64
    assert config.cache_identity["fingerprint"] == config.fingerprint
    with pytest.raises(TypeError):
        config.robot_links["extra"] = config.robot_links["base_link"]
    with pytest.raises(TypeError):
        config.data["model_id"] = "other"


def test_legacy_link_and_tool_inertias_are_strictly_physical_and_drives_are_finite(tmp_path):
    config = load_m710id70_engineering_dynamics(_write(tmp_path, _editable_document(tmp_path)))
    inertials = [*config.robot_links.values(), config.tool]
    for body in inertials:
        tensor = np.asarray(body.inertia_tensor_com_kg_m2)
        principal = np.linalg.eigvalsh(tensor)
        assert body.mass_kg > 0.0
        assert np.all(np.isfinite(body.com_xyz_m))
        assert np.all(principal > 0.0)
        assert principal[-1] <= principal[0] + principal[1] + 1e-12
    for drive in config.joint_drives.values():
        assert np.all(
            np.isfinite(
                [
                    drive.effort_limit_nm,
                    drive.velocity_limit_rad_s,
                    drive.stiffness_nm_rad,
                    drive.damping_nm_s_rad,
                ]
            )
        )
        assert min(
            drive.effort_limit_nm,
            drive.velocity_limit_rad_s,
            drive.stiffness_nm_rad,
            drive.damping_nm_s_rad,
        ) > 0.0


def test_legacy_environment_contacts_settling_and_belt_directions_are_explicit(tmp_path):
    config = load_m710id70_engineering_dynamics(_write(tmp_path, _editable_document(tmp_path)))

    assert config.environment.floor_z_m == 0.0
    assert (config.environment.right_wall_y_m, config.environment.left_wall_y_m) == (-1.15, 1.15)
    assert config.environment.side_wall_extent_status == "TRAILER_LENGTH_AND_HEIGHT_UNDEFINED_NO_EXTENT_CLAIM"
    assert set(config.contacts) == {"carton", "painted_steel", "conveyor_belt", "robot_coating"}
    for material in config.contacts.values():
        assert material.static_friction >= material.dynamic_friction >= 0.0
        assert 0.0 <= material.restitution <= 1.0
    assert set(config.damping) == {"robot_links", "tool", "cartons"}
    assert config.settling.required_stable_duration_s <= config.settling.maximum_settle_time_s
    assert tuple(config.conveyors) == CONVEYOR_SURFACE_NAMES
    assert config.conveyors["conveyor_transverse"].direction_world == (0.0, -1.0, 0.0)
    assert config.conveyors["conveyor_longitudinal"].direction_world == (-1.0, 0.0, 0.0)
    assert config.exclusive_surface_drive_at_transfer is True
    assert config.vacuum_attachment.constraint_type == "finite_breakable_6dof"
    assert config.vacuum_attachment.product_model == "上海皖泰真空吸盘三分区"
    assert config.vacuum_attachment.cup_model == "FG42"
    assert config.vacuum_attachment.physical_cup_count == 72
    assert (config.vacuum_attachment.cup_rows, config.vacuum_attachment.cup_columns) == (6, 12)
    assert config.vacuum_attachment.cup_pitch_m == (0.048, 0.048)
    assert config.vacuum_attachment.cup_radius_m == 0.0215
    assert config.vacuum_attachment.zone_count == 3
    assert config.vacuum_attachment.cups_per_zone == (24, 24, 24)
    assert config.vacuum_attachment.cup_compression_m == 0.015
    assert config.vacuum_attachment.pull_off_force_per_cup_n == 59.0
    assert config.vacuum_attachment.shear_force_per_cup_n == 43.0
    assert config.vacuum_attachment.hardware_maximum_pull_off_force_n == 59.0 * 72
    assert config.vacuum_attachment.hardware_maximum_shear_force_n == 43.0 * 72
    assert config.vacuum_attachment.break_torque_nm == 180.0
    assert np.all(
        np.isfinite(
            [
                config.vacuum_attachment.break_force_n,
                config.vacuum_attachment.break_torque_nm,
            ]
        )
    )
    assert config.vacuum_attachment.force_input_source == "PROJECT_INPUT_NOT_MEASURED_OR_VENDOR_QUALIFIED"
    assert config.vacuum_attachment.break_force_n > 42.5 * 9.81


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda data: data["robot"]["links"]["J3_link"].__setitem__("mass_kg", 0.0), "J3_link.mass_kg"),
        (
            lambda data: data["robot"]["links"]["J3_link"].__setitem__(
                "inertia_tensor_com_kg_m2", [[3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            ),
            "triangle inequality",
        ),
        (
            lambda data: data["robot"]["links"]["J3_link"].__setitem__(
                "inertia_tensor_com_kg_m2", [[1.0, 0.0, 0.0], [0.0, -0.1, 0.0], [0.0, 0.0, 1.0]]
            ),
            "positive definite",
        ),
        (lambda data: data["robot"]["joint_drives"]["J2"].__setitem__("effort_limit_nm", float("inf")), "J2.effort_limit_nm"),
        (lambda data: data["robot"]["joint_drives"]["J6"].__setitem__("velocity_limit_rad_s", 1.0), "does not match"),
    ],
)
def test_invalid_inertial_or_drive_value_fails_closed(tmp_path, mutate, message):
    data = _editable_document(tmp_path)
    mutate(data)
    with pytest.raises(ValueError, match=message):
        load_m710id70_engineering_dynamics(_write(tmp_path, data))


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda data: data["contacts"]["materials"]["carton"].update(
                {"static_friction": 0.4, "dynamic_friction": 0.5}
            ),
            "dynamic_friction",
        ),
        (lambda data: data["environment"]["floor"].__setitem__("collision_enabled", False), "must remain true"),
        (
            lambda data: data["conveyors"]["surfaces"]["conveyor_transverse"].__setitem__(
                "direction_world", [0.0, 1.0, 0.0]
            ),
            "wrong unloading direction",
        ),
        (lambda data: data["settling"].__setitem__("max_penetration_m", 0.0), "max_penetration_m"),
        (
            lambda data: data["damping"]["rigid_bodies"]["tool"].__setitem__(
                "linear_damping_s_inv", 0.06
            ),
            "must match robot_links",
        ),
        (lambda data: data["vacuum_attachment"].__setitem__("unbreakable", True), "cannot be unbreakable"),
    ],
)
def test_invalid_contact_environment_or_execution_semantics_fail_closed(tmp_path, mutate, message):
    data = _editable_document(tmp_path)
    mutate(data)
    with pytest.raises(ValueError, match=message):
        load_m710id70_engineering_dynamics(_write(tmp_path, data))


def test_tool_and_carton_mass_cannot_be_relabelled_or_double_counted(tmp_path):
    data = _editable_document(tmp_path)
    data["tool"]["expected_mass_kg"] = 19.0
    with pytest.raises(ValueError, match="20 kg tool"):
        load_m710id70_engineering_dynamics(_write(tmp_path, data, "tool.yaml"))

    data = _editable_document(tmp_path)
    data["cartons"]["count"] = 39
    with pytest.raises(ValueError, match="all 40 bodies"):
        load_m710id70_engineering_dynamics(_write(tmp_path, data, "cartons.yaml"))

    data = _editable_document(tmp_path)
    data["mass_accounting"]["tool_body"] = "separate_fixed_body_exactly_once"
    with pytest.raises(ValueError, match="exactly once"):
        load_m710id70_engineering_dynamics(_write(tmp_path, data, "accounting.yaml"))

    for field, replacement, message in (
        ("attach_to_link", "flange", "J6_link"),
        ("body_mode", "separate_fixed_rigid_body", "FIXED_TOOL_COMBINED"),
        ("mass_accounting", "exactly_once", "FIXED_TOOL_COMBINED"),
    ):
        data = _editable_document(tmp_path)
        data["tool"][field] = replacement
        with pytest.raises(ValueError, match=message):
            load_m710id70_engineering_dynamics(_write(tmp_path, data, f"tool_{field}.yaml"))


def test_valid_parameter_change_changes_content_addressed_fingerprint(tmp_path):
    before_data = _editable_document(tmp_path)
    after_data = _editable_document(tmp_path)
    after_data["damping"]["rigid_bodies"]["cartons"]["linear_damping_s_inv"] = 0.16
    before = load_m710id70_engineering_dynamics(_write(tmp_path, before_data, "before.yaml"))
    after = load_m710id70_engineering_dynamics(_write(tmp_path, after_data, "after.yaml"))
    assert before.fingerprint != after.fingerprint
    assert before.source_file_hashes["dynamics_config"] != after.source_file_hashes["dynamics_config"]
