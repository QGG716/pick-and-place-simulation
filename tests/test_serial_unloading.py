from __future__ import annotations

import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.geometry import OBB, make_transform
from unloading_sim.layout_single_carton import FrozenLayoutMotionInput, LayoutMotionPolicy
from unloading_sim.serial_unloading import (
    ACTUAL_MOTION_STATE_SCHEMA, SerialUnloadingSession, apply_actual_motion_state,
    rotation_from_actual_quaternion, state_from_isaac_record,
)
from unloading_sim.support import SupportRelationGraph
from unloading_sim.workcell_layout import LayoutValidationConfig, SNAPSHOT_SCHEMA, canonical_digest, verify_scene_snapshot


class GeometryOnlyRobot:
    def within_limits(self, q):
        return np.all(np.abs(q) <= 3.14)

    def fk(self, q):
        return make_transform(translation=q[:3])

    def named_link_frames(self, q):
        return {"flange": self.fk(q)}

    def tool_collision_obbs(self, q):
        return [OBB(np.asarray(q[:3]) + [0, 0, .1], [.1, .1, .1], np.eye(3), "rigid-tool", "robot")]


def record(box):
    return {"name": box.name, "category": box.category, "shape": "box",
            "pose_world": box.world_from_local.tolist(), "center_m": box.center.tolist(),
            "half_extents_m": box.half_extents.tolist()}


@pytest.fixture
def scene(monkeypatch):
    # Only robot geometry generation is substituted: identity/state validation,
    # full actual carton OBBs, support graph and row ordering run normally.
    monkeypatch.setattr("unloading_sim.serial_unloading.urdf_collision_shapes", lambda robot: ())
    monkeypatch.setattr("unloading_sim.serial_unloading.world_link_boxes", lambda robot, q, shapes: ())
    cartons = tuple(OBB([.3, -.84 + .42*column, .15 + .3*layer], [.3, .2, .15], np.eye(3),
                       f"opaque-{(layer*5+column)*37%97}", "carton")
                    for layer in range(8) for column in range(5))
    receiver = OBB([-.55, .35, .54], [.35, .75, .06], np.eye(3), "receiver", "conveyor")
    layout = SimpleNamespace(robot=lambda: GeometryOnlyRobot())
    validation = LayoutValidationConfig(Path("validation.yaml"), {"initial_state": {"q_rad": [0]*6}}, layout)
    policy = LayoutMotionPolicy(Path("config.yaml"), {"task_population": {
        "support_contact_tolerance_m": .008, "support_minimum_overlap_ratio": .08,
        "face_modes": ["front", "top", "side"]}}, validation, None)
    snapshot = {"schema": SNAPSHOT_SCHEMA, "layout_fingerprint": "fixed-physical-layout",
                "cartons": [record(box) for box in cartons], "assembly": {"fixed_components": [record(receiver)]},
                "robot": {"joint_names": [f"J{i}" for i in range(1, 7)], "q_rad": [0]*6},
                "tool": {}, "attachments": [], "initial_state_audit": {"status": "PASS"}}
    snapshot["scene_fingerprint"] = canonical_digest(snapshot)
    graph = SupportRelationGraph.build(cartons)
    return FrozenLayoutMotionInput(policy, snapshot, {"status": "PASS"}, {"status": "PASS"},
                                   (receiver,), cartons, receiver, graph, tuple(graph.removable_cartons()))


def actual(scene, completed=(), handed=()):
    return {"schema": ACTUAL_MOTION_STATE_SCHEMA, "time_s": 2., "world_session_id": "one-world",
            "q_rad": [.1]*6, "attached": False,
            "cartons": [{"name": box.name, "position_m": (box.center + [.001, -.0002, .0001]).tolist(),
                         "orientation_wxyz": [1, 0, 0, 0], "linear_velocity_m_s": [.001, 0, 0],
                         "angular_velocity_rad_s": [0, 0, .0001]}
                        for box in scene.cartons if box.name not in handed],
            "completed_carton_ids": list(completed), "handed_off_ids": list(handed)}


def test_actual_update_keeps_all_40_identities_and_completed_receiver_occupancy(scene):
    original_fingerprint = scene.snapshot["scene_fingerprint"]
    completed = scene.cartons[-3].name
    state = actual(scene, [completed])
    target = next(item for item in state["cartons"] if item["name"] == completed)
    target["position_m"] = [-.55, .35, .75]
    target["orientation_wxyz"] = [np.cos(.1), 0, 0, np.sin(.1)]
    changed = apply_actual_motion_state(scene, state)
    assert len(changed.cartons) == 40
    assert len(changed.remaining_stack_names) == 39
    assert [box.name for box in changed.occupied] == [completed]
    assert completed in {box.name for box in changed.all_obstacles}
    assert completed not in changed.support_graph.cartons
    np.testing.assert_allclose(changed.occupied[0].center, [-.55, .35, .75])
    np.testing.assert_allclose(changed.occupied[0].rotation[:2, :2],
                               [[np.cos(.2), -np.sin(.2)], [np.sin(.2), np.cos(.2)]])
    np.testing.assert_allclose(changed.policy.layout_validation.initial_q, [.1]*6)
    assert changed.snapshot["layout_fingerprint"] == scene.snapshot["layout_fingerprint"]
    assert changed.snapshot["scene_fingerprint"] != original_fingerprint
    assert scene.snapshot["scene_fingerprint"] == original_fingerprint
    assert verify_scene_snapshot(changed.snapshot)["status"] == "PASS"


def test_identity_loss_duplication_and_fake_handoff_are_rejected(scene):
    missing = actual(scene)
    missing["cartons"].pop()
    with pytest.raises(ValueError, match="identity mismatch"):
        apply_actual_motion_state(scene, missing)
    duplicate = actual(scene)
    duplicate["cartons"].append(copy.deepcopy(duplicate["cartons"][0]))
    with pytest.raises(ValueError, match="duplicate"):
        apply_actual_motion_state(scene, duplicate)
    with pytest.raises(ValueError, match="subset"):
        apply_actual_motion_state(scene, actual(scene, handed=[scene.cartons[0].name]))


def test_explicit_handoff_reduces_active_population_without_resurrecting_objects(scene):
    name = scene.cartons[-3].name
    changed = apply_actual_motion_state(scene, actual(scene, [name], [name]))
    assert len(changed.cartons) == 39
    assert len(changed.occupied) == 0
    assert changed.snapshot["actual_state_context"]["initial_carton_count"] == 40
    with pytest.raises(ValueError, match="regress"):
        apply_actual_motion_state(changed, actual(changed))
    continued = actual(changed, [name], [name])
    continued["time_s"] = 3.
    again = apply_actual_motion_state(changed, continued)
    assert len(again.cartons) == 39
    assert again.snapshot["actual_state_context"]["initial_carton_count"] == 40


def test_actual_pose_alone_never_completes_or_removes_a_box_and_sessions_do_not_reset(scene):
    state = actual(scene)
    state["cartons"][0]["position_m"] = [-10, 0, .15]
    changed = apply_actual_motion_state(scene, state)
    assert len(changed.remaining_stack_names) == 40
    wrong = actual(changed)
    wrong["world_session_id"] = "reset-world"
    with pytest.raises(ValueError, match="same physical world"):
        apply_actual_motion_state(changed, wrong)
    attached = actual(scene)
    attached["attached"] = True
    with pytest.raises(ValueError, match="attachment release"):
        apply_actual_motion_state(scene, attached)


def test_array_record_conversion_preserves_measured_velocity_and_requires_channels(scene):
    names = [box.name for box in scene.cartons]
    raw = {"q_rad": [0]*6, "carton_names": names,
           "carton_positions_m": [box.center.tolist() for box in scene.cartons],
           "carton_orientations_wxyz": [[1, 0, 0, 0]]*40,
           "carton_linear_velocities_m_s": [[.02, 0, 0]]*40,
           "carton_angular_velocities_rad_s": [[0, .03, 0]]*40}
    state = state_from_isaac_record(raw)
    assert state["cartons"][0]["linear_velocity_m_s"] == [.02, 0, 0]
    del raw["carton_linear_velocities_m_s"]
    with pytest.raises(ValueError, match="missing"):
        state_from_isaac_record(raw)
    with pytest.raises(ValueError, match="unit length"):
        rotation_from_actual_quaternion([0, 0, 0, 0])


def test_serial_entry_preserves_row_state_and_stops_at_completed_budget(scene):
    session = SerialUnloadingSession(scene, maximum_completed_tasks=1)
    calls = []
    result = session.plan_next(planner=lambda policy, **kwargs: calls.append(kwargs) or "planned")
    assert result == "planned"
    assert calls[0]["row_state"] is session.row_state
    assert calls[0]["motion_input"] is scene
    state = actual(scene, [scene.cartons[-3].name])
    session.update_actual_state(state)
    assert session.plan_next(planner=lambda *a, **kw: pytest.fail("budget must stop before planner"))["status"] == "EXECUTION_BUDGET_EXHAUSTED"
