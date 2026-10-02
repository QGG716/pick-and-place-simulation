"""Small isolation and continuity regressions; not planning run evidence."""
from copy import deepcopy
from types import SimpleNamespace
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
from unloading_sim.moveit2_backend import MoveItLayoutConnector, MoveItUnavailable
from unloading_sim.planning_only import PlanningOnlyConnector, PlanningInputGuard, MARKER, SKIPPED
from test_moveit2_native_cold import receipt_connector, accepted_state


def experiment():
    c = receipt_connector()
    c.__class__ = PlanningOnlyConnector
    c.native_generated = []
    c.skipped_calls = {}
    c._native_cancelled = lambda: False
    c.robot.within_limits = lambda q: np.max(np.abs(q)) <= 3
    # receipt_connector's per-instance mock would hide the actual overrides.
    del c._path_failure
    return c


def test_generated_lineage_is_separate_from_validation_and_not_record_order():
    c = experiment()
    root = c.native_root_state(np.zeros(6), [])
    # The established receipt helper builds a validated record, then we move
    # it into the experiment's generated registry with its real weaker status.
    c.__class__ = MoveItLayoutConnector
    first = accepted_state(c, root, np.ones(6)*.1, "first")
    second = accepted_state(c, root, np.ones(6)*.1, "second")
    c.__class__ = PlanningOnlyConnector
    c.native_generated = c.native_verified
    c.native_verified = []
    for r in c.native_generated:
        r.update(authoritative_status=SKIPPED, planning_only_status=MARKER)
    assert c._native_parent_stage_id(first) == "first"
    assert c._native_parent_stage_id(second) == "second"
    assert not c.native_verified
    c.native_task_id = "next-box-task"
    with pytest.raises(MoveItUnavailable, match="TASK_MISMATCH"):
        c._native_parent_stage_id(first)


def test_experiment_does_not_call_dense_validator(monkeypatch):
    c = experiment()
    def forbidden(*args, **kwargs):
        raise AssertionError("heavy validation entered")
    monkeypatch.setattr(LayoutTrajectoryConnector, "_path_failure", forbidden)
    monkeypatch.setattr(LayoutTrajectoryConnector, "_state_failure", forbidden)
    monkeypatch.setattr(LayoutTrajectoryConnector, "_motion_validator", forbidden)
    assert c._path_failure([np.zeros(6), np.ones(6)*.1], [], stage="transit") is None
    assert c._state_failure(np.full(6, np.nan), [], stage="transit") is not None
    assert c._state_failure(np.ones(5), [], stage="transit") is not None
    c._context_identity = lambda *args, **kwargs: "context"
    guard = c._motion_validator([], stage="transit")
    result = guard.check_states([np.zeros(6)], None)[0]
    assert isinstance(guard, PlanningInputGuard)
    assert result.valid and result.evidence()["status"] == SKIPPED
    assert not result.evidence()["collision_validated"]
    assert not hasattr(guard, "check_motion")


def test_experiment_cannot_accept_normal_worker_output():
    c = experiment()
    with pytest.raises(MoveItUnavailable, match="OUTPUT_CHECK_NOT_SKIPPED"):
        c._accept_generated_stage([], {"native_output_status": "PASS"}, [], {},
            attachment=None, initial_proximity=None, started=0.)
    assert MoveItLayoutConnector.planning_only is False


def test_next_scene_uses_previous_generated_endpoint_and_only_removes_completed():
    from unloading_sim.geometry import OBB
    from unloading_sim.support import SupportRelationGraph
    from unloading_sim.layout_single_carton import FrozenLayoutMotionInput
    path = Path(__file__).resolve().parents[1] / "tools/plan_m710_top_row_only.py"
    spec = importlib.util.spec_from_file_location("planning_only_entry", path)
    entry = importlib.util.module_from_spec(spec); spec.loader.exec_module(entry)
    boxes = tuple(OBB([0, i, 1], [.2]*3, np.eye(3), f"box{i}", "carton") for i in range(3))
    snapshot = dict(robot={"q_rad": [0.]*6}, tool={}, cartons=[dict(name=b.name) for b in boxes])
    scene = FrozenLayoutMotionInput(SimpleNamespace(data={"task_population": {
        "support_contact_tolerance_m": .0002, "support_minimum_overlap_ratio": .2}}),
        snapshot, {}, {}, (), boxes, boxes[0], SupportRelationGraph.build(boxes), ())
    endpoint = np.array([.1, .2, -.3, .4, .5, -.6])
    robot = SimpleNamespace(fk=lambda q: np.eye(4), named_link_frames=lambda q: {"flange": np.eye(4)})
    changed = entry.next_scene(scene, ["box1"], endpoint, robot)
    assert np.array_equal(changed.snapshot["robot"]["q_rad"], endpoint)
    assert [b.name for b in changed.cartons] == ["box0", "box2"]
    assert len(scene.cartons) == 3 and scene.snapshot == snapshot
    assert all(np.array_equal(b.world_from_local, boxes[i].world_from_local)
               for b, i in zip(changed.cartons, [0, 2]))
