from __future__ import annotations

import copy
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.workcell_layout import (
    LAYOUT_SCHEMA,
    SNAPSHOT_SCHEMA,
    audit_initial_state,
    audit_layout_constraints,
    audit_snapshot_consistency,
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


def test_robot_mount_derivation_is_explicit_and_supported():
    config = _config()
    layout = config.layout
    origin = layout.robot_base_transform()[:3, 3]
    np.testing.assert_allclose(origin, [-1.41, 0.35, 0.6])
    assert origin[0] + layout.data["robot"]["base_proxy_radius_m"] == pytest.approx(-1.1)
    assert layout.data["robot"]["positioning_status"].startswith("ENGINEERING_PROXY")

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
    assert audit_initial_state(config)["status"] == "PASS"
    witness = find_initial_state_witness(config, int(config.data["initial_state"]["search_maximum_random_draws"]))
    assert witness["status"] == "PASS"
    assert witness["random_draw"] == config.data["initial_state"]["selected_random_draw_1_based"]
    np.testing.assert_allclose(witness["q_rad"], config.initial_q, atol=1e-12, rtol=0)
    assert witness["path_from_previous_state"] == "NOT_EVALUATED"


def test_snapshot_is_frozen_content_addressed_and_replay_consistent():
    config = _config()
    snapshot = build_scene_snapshot(config)
    assert snapshot["schema"] == SNAPSHOT_SCHEMA
    assert snapshot["layout_fingerprint"] == config.layout.layout_fingerprint
    assert len(snapshot["cartons"]) == 40
    assert len(snapshot["robot"]["link_collision_obbs"]) == 7
    assert snapshot["receiver"]["transport_capability"] == "NOT_IMPLEMENTED_FOR_LAYOUT_V1"
    assert snapshot["attachments"] == []
    np.testing.assert_allclose(snapshot["robot"]["tcp_pose_world"], snapshot["tool"]["task_tcp_pose_world"])
    np.testing.assert_allclose(
        2 * np.asarray(snapshot["tool"]["collision_obb"]["half_extents_m"]),
        [0.288, 0.576, 0.2275],
    )
    verification = verify_scene_snapshot(snapshot, ROOT)
    assert verification["status"] == "PASS"
    assert len(verification["checked_assets"]) == 7
    consistency = audit_snapshot_consistency(config, snapshot)
    assert consistency["status"] == "PASS"
    assert max(consistency["maximum_absolute_errors"].values()) <= 1e-12

    tampered = copy.deepcopy(snapshot)
    tampered["cartons"][0]["center_m"][0] += 0.001
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_scene_snapshot(tampered)
    missing = copy.deepcopy(snapshot)
    missing.pop("scene_fingerprint")
    with pytest.raises(ValueError, match="fingerprint mismatch"):
        verify_scene_snapshot(missing)


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
