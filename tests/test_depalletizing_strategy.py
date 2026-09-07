from pathlib import Path

import numpy as np
import pytest

from unloading_sim.depalletizing import (
    HandoffProtocol,
    LConveyorGeometry,
    analyze_box_neighborhood,
    dynamic_conveyor_z_limits,
    generate_extraction_candidates,
    minimum_clearance_extraction_distance,
    plan_conveyor_preposition,
)
from unloading_sim.geometry import OBB


def _box(name, center, size=(0.6, 0.4, 0.3)):
    return OBB(np.asarray(center, dtype=float), 0.5 * np.asarray(size), np.eye(3), name, "carton")


def test_both_side_neighbors_require_geometry_derived_front_extraction():
    target = _box("target", [1.3, 0.0, 0.45])
    left = _box("left", [1.3, 0.41, 0.45])
    right = _box("right", [1.3, -0.41, 0.45])
    state = analyze_box_neighborhood(target, [left, right])
    candidates = generate_extraction_candidates(state)
    assert state.constrained_both_sides
    assert candidates[0].grasp_face == "front"
    assert candidates[0].requires_local_linear_extraction
    distance = minimum_clearance_extraction_distance(target, [-1, 0, 0], [left, right])
    assert distance == pytest.approx(0.62, abs=1e-6)


def test_left_blocked_right_open_prefers_direct_right_side_and_is_symmetric():
    target = _box("target", [1.3, 0.0, 0.45])
    left = _box("left", [1.3, 0.41, 0.45])
    right = _box("right", [1.3, -0.41, 0.45])
    left_blocked = generate_extraction_candidates(analyze_box_neighborhood(target, [left]))
    right_blocked = generate_extraction_candidates(analyze_box_neighborhood(target, [right]))
    assert (left_blocked[0].grasp_face, left_blocked[0].requires_local_linear_extraction) == ("right", False)
    assert (right_blocked[0].grasp_face, right_blocked[0].requires_local_linear_extraction) == ("left", False)


def test_both_sides_open_generate_and_score_all_four_faces():
    target = _box("target", [1.3, 0.0, 0.45])
    candidates = generate_extraction_candidates(analyze_box_neighborhood(target, []))
    assert {candidate.grasp_face for candidate in candidates} == {"front", "left", "right", "top"}


def test_minimum_extraction_depends_on_carton_geometry_not_a_fixed_distance():
    deep = _box("deep", [1.3, 0.0, 0.45], (0.6, 0.4, 0.3))
    deep_left = _box("deep_left", [1.3, 0.41, 0.45], (0.6, 0.4, 0.3))
    shallow = _box("shallow", [1.2, 0.0, 0.45], (0.4, 0.4, 0.3))
    shallow_left = _box("shallow_left", [1.2, 0.41, 0.45], (0.4, 0.4, 0.3))
    deep_distance = minimum_clearance_extraction_distance(deep, [-1, 0, 0], [deep_left])
    shallow_distance = minimum_clearance_extraction_distance(shallow, [-1, 0, 0], [shallow_left])
    assert deep_distance == pytest.approx(0.62, abs=1e-6)
    assert shallow_distance == pytest.approx(0.42, abs=1e-6)
    assert deep_distance != shallow_distance


def test_independent_l_conveyor_transform_and_dynamic_limits():
    geometry = LConveyorGeometry()
    base = np.array([-0.70, 0.0, 0.60])
    cross, longitudinal = geometry.obstacles(base, 0.30, 0.50)
    chassis_center = base + np.asarray(geometry.chassis_center_from_robot_base_xyz_m)
    chassis_front = chassis_center[0] + 0.5 * geometry.chassis_size_xyz_m[0]
    chassis_right = chassis_center[1] - 0.5 * geometry.chassis_size_xyz_m[1]
    assert cross.center[0] - cross.half_extents[0] == pytest.approx(chassis_front)
    assert longitudinal.center[1] - longitudinal.half_extents[1] == pytest.approx(chassis_right)
    assert cross.category == longitudinal.category == "conveyor"

    lower = _box("lower", [1.3, -0.8, 0.35])
    upper = _box("upper", [1.3, -0.8, 1.35])
    minimum, maximum = dynamic_conveyor_z_limits([lower, upper], geometry)
    assert minimum == pytest.approx(0.20)
    assert maximum == pytest.approx(1.185)


def test_conveyor_reaches_twenty_mm_surface_gap_and_rejects_collision():
    geometry = LConveyorGeometry()
    base = np.array([-0.70, 0.0, 0.60])
    target = _box("target", [1.3, -0.75, 0.75])
    upper = _box("upper", [1.3, -0.75, 1.35])
    pose = plan_conveyor_preposition(geometry, base, target, [target, upper], [])
    assert pose is not None
    assert pose.target_surface_clearance_m == pytest.approx(0.02, abs=1e-9)
    assert 0.20 <= pose.conveyor_z_m <= 1.185

    blocker = OBB(
        pose.obstacles[1].center,
        pose.obstacles[1].half_extents + [2.0, 2.0, 2.0],
        np.eye(3),
        "blocker",
    )
    assert plan_conveyor_preposition(geometry, base, target, [target, upper], [blocker]) is None


def test_vacuum_release_is_gated_by_completed_handoff():
    protocol = HandoffProtocol()
    with pytest.raises(RuntimeError):
        protocol.release_vacuum()
    protocol.confirm_supported_handoff(box_supported=True, collision_free=True)
    protocol.release_vacuum()
    assert protocol.vacuum_released


def test_fixed_250mm_retreat_is_absent_from_m710_runtime_config():
    text = (Path(__file__).parents[1] / "config/fanuc_m710id_70.yaml").read_text(encoding="utf-8")
    assert "extraction_distance_m" not in text
    assert "0.25" not in text.split("planning:", 1)[1].split("outputs:", 1)[0]
