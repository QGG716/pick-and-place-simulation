from __future__ import annotations

import copy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
import yaml

from unloading_sim.geometry import OBB
from unloading_sim.m710_initialization_diagnostic import (
    SCOPE as INITIALIZATION_SCOPE,
    build_initialization_diagnostic_contract,
    verify_initialization_diagnostic_contract,
)
from unloading_sim.workcell_layout import (
    LAYOUT_SCHEMA,
    audit_initial_state,
    audit_layout_constraints,
    build_scene_snapshot,
    compute_layout_fingerprint,
    find_initial_state_witness,
    load_layout_validation_config,
    load_workcell_layout,
    robot_chassis_support_contact_allowed,
    verify_scene_snapshot,
    world_state_fingerprint,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/validation/m710id70_layout_v1.yaml"


def _config():
    return load_layout_validation_config(CONFIG)


def _bounds(box: OBB) -> tuple[np.ndarray, np.ndarray]:
    corners = box.corners()
    return np.min(corners, axis=0), np.max(corners, axis=0)


def _mutated_layout(layout, update):
    data = copy.deepcopy(layout.data)
    update(data)
    return replace(
        layout,
        data=data,
        layout_fingerprint=compute_layout_fingerprint(data, layout.assets),
    )


def test_confirmed_dimensions_and_world_bounds_are_literal_contracts():
    layout = _config().layout
    boxes = {box.name: box for box in layout.fixed_components()}

    np.testing.assert_allclose(2 * boxes["chassis"].half_extents, [2.1, 1.5, 0.6])
    np.testing.assert_allclose(_bounds(boxes["chassis"]), ([-3.0, -0.4, 0.0], [-0.9, 1.1, 0.6]))
    np.testing.assert_allclose(2 * boxes["conveyor_transverse"].half_extents[:2], [0.7, 1.5])
    np.testing.assert_allclose(_bounds(boxes["conveyor_transverse"])[0][:2], [-0.9, -0.4])
    np.testing.assert_allclose(_bounds(boxes["conveyor_transverse"])[1][:2], [-0.2, 1.1])
    np.testing.assert_allclose(2 * boxes["conveyor_longitudinal"].half_extents[:2], [2.8, 0.7])
    np.testing.assert_allclose(_bounds(boxes["conveyor_longitudinal"])[0][:2], [-3.0, -1.1])
    np.testing.assert_allclose(_bounds(boxes["conveyor_longitudinal"])[1][:2], [-0.2, -0.4])
    assert _bounds(boxes["conveyor_transverse"])[1][2] == pytest.approx(0.6)
    assert _bounds(boxes["conveyor_longitudinal"])[1][2] == pytest.approx(0.6)

    audit = audit_layout_constraints(layout)
    assert audit["overall_status"] == "PASS"
    np.testing.assert_allclose(audit["assembly_world_bounds_m"]["x"], [-3.0, -0.2])
    np.testing.assert_allclose(audit["assembly_world_bounds_m"]["y"], [-1.1, 1.1])
    np.testing.assert_allclose(audit["assembly_world_bounds_m"]["z"], [0.0, 0.6])
    assert audit["checks"]["left_side_clearance"]["clearance_m"] == pytest.approx(0.05)
    assert audit["checks"]["right_side_clearance"]["clearance_m"] == pytest.approx(0.05)
    assert audit["checks"]["stack_to_conveyor_clearance"]["clearance_m"] == pytest.approx(0.2)


def test_named_development_trailer_is_closed_behind_stack_and_open_at_negative_x():
    layout = _config().layout
    trailer = layout.data["trailer"]
    assert trailer["length_status"] == "DEVELOPMENT_SCENE_ASSUMPTION_NOT_MEASURED"
    assert trailer["height_status"] == "DEVELOPMENT_SCENE_ASSUMPTION_NOT_MEASURED"
    assert trailer["opening_x_m"] == pytest.approx(-3.2)
    assert trailer["closed_end_wall_x_m"] == pytest.approx(3.2)
    assert trailer["length_m"] == pytest.approx(6.4)
    assert trailer["height_m"] == pytest.approx(2.7)
    boundaries = {box.name: box for box in layout.trailer_boundary_boxes()}
    assert set(boundaries) == {
        "trailer_floor", "trailer_left_wall", "trailer_right_wall",
        "trailer_ceiling", "trailer_closed_end_wall",
    }
    np.testing.assert_allclose(_bounds(boundaries["trailer_floor"]),
                               ([-3.2, -1.15, -0.05], [3.2, 1.15, 0.0]))
    np.testing.assert_allclose(_bounds(boundaries["trailer_ceiling"]),
                               ([-3.2, -1.15, 2.7], [3.2, 1.15, 2.75]))
    assert _bounds(boundaries["trailer_closed_end_wall"])[0][0] == pytest.approx(3.2)


def test_robot_mount_derivation_is_explicit_and_supported():
    config = _config()
    layout = config.layout
    origin = layout.robot_base_transform()[:3, 3]
    np.testing.assert_allclose(origin, [-1.325, 0.35, 0.6])
    np.testing.assert_allclose(layout.data["robot"]["base_support_bbox_min_xyz_m"], [-0.3385, -0.275, 0.0])
    np.testing.assert_allclose(layout.data["robot"]["base_support_bbox_max_xyz_m"], [0.225, 0.275, 0.245])
    assert origin[0] + 0.225 == pytest.approx(-1.1)
    assert layout.data["robot"]["positioning_status"] == "EXECUTION_QUALIFIED_CAD_COLLISION"

    chassis = next(box for box in layout.fixed_components() if box.name == "chassis")
    touching = OBB(origin + [0, 0, 0.1], [0.2, 0.2, 0.1], np.eye(3), "mount_proxy")
    penetrating = replace(touching, center=touching.center - [0, 0, 0.001])
    unrelated = replace(touching, center=touching.center + [0.95, 0, -0.05], name="moving_link")
    assert robot_chassis_support_contact_allowed(touching, chassis, config.contact_tolerance_m)
    assert not robot_chassis_support_contact_allowed(penetrating, chassis, config.contact_tolerance_m)
    assert not robot_chassis_support_contact_allowed(unrelated, chassis, config.contact_tolerance_m)


def test_stack_is_exact_one_by_five_by_eight_without_deleted_support_boxes():
    cartons = _config().layout.cartons()
    assert len(cartons) == len({carton.name for carton in cartons}) == 40
    for carton in cartons:
        np.testing.assert_allclose(2 * carton.half_extents, [0.6, 0.4, 0.3])
        np.testing.assert_allclose(carton.rotation, np.eye(3))
        assert carton.center[0] == pytest.approx(0.3)
    assert sorted({round(float(box.center[1]), 8) for box in cartons}) == [-0.84, -0.42, 0.0, 0.42, 0.84]
    assert sorted({round(float(box.center[2]), 8) for box in cartons}) == [0.15, 0.45, 0.75, 1.05, 1.35, 1.65, 1.95, 2.25]
    assert len({(round(float(box.center[0]), 8), round(float(box.center[1]), 8)) for box in cartons}) == 5
    assert len({(round(float(box.center[0]), 8), round(float(box.center[2]), 8)) for box in cartons}) == 8
    assert len({(round(float(box.center[1]), 8), round(float(box.center[2]), 8)) for box in cartons}) == 40
    all_corners = np.concatenate([box.corners() for box in cartons])
    assert np.ptp(all_corners[:, 1]) == pytest.approx(2.08)
    assert np.ptp(all_corners[:, 2]) == pytest.approx(2.4)
    for layer in range(1, 8):
        lower_top = max(_bounds(box)[1][2] for box in cartons if box.name.startswith(f"carton_l{layer - 1:02d}"))
        upper_bottom = min(_bounds(box)[0][2] for box in cartons if box.name.startswith(f"carton_l{layer:02d}"))
        assert upper_bottom - lower_top == pytest.approx(0.0)


def test_assembly_is_a_rigid_transform_and_fixed_identity_ignores_placement():
    layout = _config().layout
    moved = _mutated_layout(
        layout,
        lambda data: data["assembly"].update(
            {"world_xyz_m": [-0.3, 0.1, 0.2], "world_rpy_rad": [0.0, 0.0, 0.37]}
        ),
    )
    original_relative = [layout.assembly_from_world @ box.world_from_local for box in layout.fixed_components()]
    moved_relative = [moved.assembly_from_world @ box.world_from_local for box in moved.fixed_components()]
    for before, after in zip(original_relative, moved_relative):
        np.testing.assert_allclose(after, before, atol=1e-12)
    for before, after in zip(layout.cartons(), moved.cartons()):
        np.testing.assert_allclose(after.world_from_local, before.world_from_local)
    assert moved.layout_fingerprint == layout.layout_fingerprint
    assert world_state_fingerprint(moved, _config().initial_q) != world_state_fingerprint(layout, _config().initial_q)
    assert audit_layout_constraints(moved)["overall_status"] == "FAIL"


@pytest.mark.parametrize("delta", [-0.001, 0.001])
def test_designed_assembly_contact_rejects_gap_or_penetration(delta):
    config = _config()
    components = list(config.layout.component_local_boxes())
    components[1] = replace(components[1], center=components[1].center + [delta, 0, 0])
    audit = audit_layout_constraints(
        config.layout,
        local_components=components,
        tolerance=config.contact_tolerance_m,
    )
    assert audit["overall_status"] == "FAIL"
    assert audit["checks"]["chassis_transverse_join"]["status"] == "FAIL"


def test_robot_mount_cannot_be_sunk_into_chassis():
    config = _config()
    moved = _mutated_layout(
        config.layout,
        lambda data: data["robot"].update({"mounting_surface_z_a_m": 0.599}),
    )
    audit = audit_layout_constraints(moved, tolerance=config.contact_tolerance_m)
    assert audit["overall_status"] == "FAIL"
    assert audit["checks"]["robot_mount_surface"]["status"] == "FAIL"


def test_initial_state_is_collision_checked_and_seeded_witness_is_reproducible():
    config = _config()
    previous = audit_initial_state(config, np.asarray(config.data["initial_state"]["previous_v3_q_rad"]))
    assert previous["status"] == "FAIL"
    assert any(failure["reason"] == "TRAILER_SIDE_CLEARANCE" for failure in previous["failures"])
    initial = audit_initial_state(config)
    assert initial["status"] == "FAIL"
    assert any(failure["reason"] == "TOOL_SELF_COLLISION" and failure["pair"] == ["tool_rigid_13", "J5_link"]
               for failure in initial["failures"])
    # Test deterministic failure accounting with a bounded search; the real
    # 3000-draw diagnostic remains evidence, not a test-time success fixture.
    witness = find_initial_state_witness(config, 1)
    assert witness == find_initial_state_witness(config, 1)
    assert witness["status"] == "FAIL"
    assert witness["seed"] == 71070
    assert witness["maximum_random_draws"] == 1
    assert witness["draw"] is None
    assert witness["last_audit"]["status"] == "FAIL"


def test_invalid_home_refuses_snapshot_and_freezes_only_diagnostic_geometry():
    config = _config()
    with pytest.raises(ValueError, match="refusing to snapshot invalid initial state"):
        build_scene_snapshot(config)
    diagnostic = build_initialization_diagnostic_contract(
        CONFIG, ROOT / "configs/simulation/m710id70_official_dynamics_v2.yaml", ROOT,
    )
    verify_initialization_diagnostic_contract(diagnostic)
    assert diagnostic["scope"] == INITIALIZATION_SCOPE
    assert diagnostic["initial_state_audit"]["status"] == "FAIL"
    assert diagnostic["layout_fingerprint"] == config.layout.layout_fingerprint
    assert not diagnostic["motion_execution_permitted"]
    assert not diagnostic["attachment_permitted"]
    expected = {b.name: b for b in [*config.layout.fixed_components(), *config.layout.cartons(),
                                  *config.layout.robot().tool_collision_obbs(config.initial_q)]}
    assert len(diagnostic["primitives"]) == 101
    for record in diagnostic["primitives"]:
        box = expected[record["name"]]
        np.testing.assert_allclose(record["pose_world"], box.world_from_local, atol=1e-12)
        np.testing.assert_allclose(record["size_xyz_m"], 2 * box.half_extents, atol=1e-12)
    tampered = copy.deepcopy(diagnostic)
    tampered["primitives"][0]["pose_world"][0][3] += 0.001
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_initialization_diagnostic_contract(tampered)
    missing = copy.deepcopy(diagnostic)
    missing.pop("contract_fingerprint")
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_initialization_diagnostic_contract(missing)
    with pytest.raises(ValueError, match="schema"):
        verify_scene_snapshot(diagnostic)


def test_layout_fingerprint_tracks_geometry_but_not_validation_render_settings():
    config = _config()
    data = copy.deepcopy(config.layout.data)
    data["chassis"]["size_xyz_m"][0] += 0.001
    assert compute_layout_fingerprint(data, config.layout.assets) != config.layout.layout_fingerprint
    validation_copy = copy.deepcopy(config.data)
    validation_copy["render"]["dpi"] += 1
    assert config.layout.layout_fingerprint == load_workcell_layout(config.layout.config_path).layout_fingerprint
    assert validation_copy["render"] != config.data["render"]


def test_layout_schema_is_independent_from_legacy_v3_validation_schema():
    assert _config().layout.data["schema"] == LAYOUT_SCHEMA
    with pytest.raises(ValueError):
        load_workcell_layout(ROOT / "configs/validation/m710id70_v3.yaml")


def test_feasibility_entry_routes_layout_phase_without_loading_legacy_defaults(tmp_path):
    validation = copy.deepcopy(_config().data)
    validation["layout_config"] = str(_config().layout.config_path)
    validation["initial_state"]["search_maximum_random_draws"] = 1
    config_path = tmp_path / "layout_validation.yaml"
    config_path.write_text(yaml.safe_dump(validation), encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            "tools/run_m710id70_v3.py",
            "--phase",
            "layout",
            "--config",
            str(config_path),
            "--output-dir",
            str(tmp_path / "layout"),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    numeric = json.loads((tmp_path / "layout/numeric_audit.json").read_text(encoding="utf-8"))
    initial = json.loads((tmp_path / "layout/initial_state_audit.json").read_text(encoding="utf-8"))
    search = json.loads((tmp_path / "layout/initial_state_search.json").read_text(encoding="utf-8"))
    assert numeric["overall_status"] == "PASS"
    assert initial["status"] == "FAIL" and initial["failures"]
    assert search["status"] == "FAIL"
    assert not (tmp_path / "layout/scene_snapshot.json").exists()
