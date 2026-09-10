from __future__ import annotations

import copy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.layout_trajectory import (
    ExactM710LayoutStateValidator,
    LayoutTrajectoryConnector,
    _audit_official_srdf_policy,
    validate_layout_trajectory_stage_contract,
)
from unloading_sim.robot import CollisionResult


def _segment() -> dict:
    path = [[0.01 * index, 0.0, 0.0, 0.0, 0.0, 0.0] for index in range(8)]
    identity = np.eye(4)
    mask = [True] + [False] * 71
    return {
        "schema": "m710id70_layout_complete_trajectory_segment_v1",
        "target": "carton_l07_c02",
        "face": "top",
        "path": path,
        "grasp_index": 2,
        "release_index": 6,
        "release_retreat_index": 7,
        "stage_ranges": {
            "home": [0, 0],
            "pregrasp": [0, 1],
            "contact": [1, 2],
            "support-release": [2, 3],
            "extraction": [3, 4],
            "transit": [4, 5],
            "place": [5, 6],
            "withdrawal": [6, 7],
        },
        "events": [
            {"index": 2, "event": "ATTACH"},
            {"index": 6, "event": "RELEASE"},
            {"index": 7, "event": "RELEASE_RETREAT_COMPLETE"},
        ],
        "contact": {
            "requested_virtual_task_tcp_pose_world": identity.tolist(),
            "requested_physical_contact_pose_world": identity.tolist(),
            "actual_q_rad": path[2].copy(),
            "actual_virtual_task_tcp_pose_world": identity.tolist(),
            "actual_physical_contact_pose_world": identity.tolist(),
            "physical_contact_from_box": identity.tolist(),
            "cup_selection": {
                "suction_mode": "ideal_independent_cups",
                "load_bearing_minimum_cup_count": None,
                "actual_contact_count": 1,
                "commanded_active_count": 1,
                "target_id": "carton_l07_c02",
                "target_face": "top",
                "geometrically_eligible_mask": mask.copy(),
                "commanded_active_mask": mask.copy(),
                "actual_contact_mask": mask.copy(),
                "actual_q_rad": path[2].copy(),
                "actual_virtual_task_tcp_pose_world": identity.tolist(),
                "actual_physical_contact_pose_world": identity.tolist(),
                "enforce_vacuum_force_capacity": False,
                "enforce_vacuum_break_force": False,
                "enforce_vacuum_break_torque": False,
            },
        },
        "place": {
            "receiver": "conveyor_transverse",
            "place_surface": "conveyor_transverse",
            "release_center_world_m": [0.0, 0.0, 0.0],
            "actual_box_pose_world": identity.tolist(),
            "support": {"supported": True},
        },
        "validation": {"execution_qualified": True},
    }


def test_complete_segment_requires_all_contiguous_stages_and_exact_events():
    segment = _segment()
    assert validate_layout_trajectory_stage_contract(segment) is segment

    missing = copy.deepcopy(segment)
    del missing["stage_ranges"]["support-release"]
    with pytest.raises(ValueError, match="eight required stages"):
        validate_layout_trajectory_stage_contract(missing)

    branch_switch = copy.deepcopy(segment)
    branch_switch["stage_ranges"]["transit"][0] -= 1
    with pytest.raises(ValueError, match="not contiguous"):
        validate_layout_trajectory_stage_contract(branch_switch)

    reordered = copy.deepcopy(segment)
    reordered["events"].reverse()
    with pytest.raises(ValueError, match="three ordered"):
        validate_layout_trajectory_stage_contract(reordered)


def test_complete_segment_rejects_empty_cup_attachment_and_wrong_actual_q():
    segment = _segment()
    segment["contact"]["cup_selection"]["actual_contact_count"] = 0
    with pytest.raises(ValueError, match="non-empty ideal cup"):
        validate_layout_trajectory_stage_contract(segment)

    segment = _segment()
    segment["contact"]["actual_q_rad"][0] += 0.1
    with pytest.raises(ValueError, match="must equal the grasp"):
        validate_layout_trajectory_stage_contract(segment)


def test_complete_segment_rejects_cup_mask_or_nested_actual_pose_tampering():
    segment = _segment()
    segment["contact"]["cup_selection"]["actual_contact_mask"][1] = True
    with pytest.raises(ValueError, match="counts, target, or enforcement"):
        validate_layout_trajectory_stage_contract(segment)

    segment = _segment()
    segment["contact"]["cup_selection"][
        "actual_physical_contact_pose_world"
    ][0][3] = 0.001
    with pytest.raises(ValueError, match="disagrees with contact evidence"):
        validate_layout_trajectory_stage_contract(segment)


class _Robot:
    dof = 6
    joint_limits = np.tile([-2.0, 2.0], (6, 1))
    base_transform = np.eye(4)

    def within_limits(self, q, tolerance=1e-9):
        q = np.asarray(q)
        return q.shape == (6,) and np.all(np.abs(q) <= 2.0 + tolerance)

    def fk(self, q):
        result = np.eye(4)
        result[:3, 3] = np.asarray(q)[:3]
        return result

    def geometric_jacobian(self, q):
        return np.eye(6)

    def named_link_frames(self, q):
        return {"flange": self.fk(q)}

    def tool_collision_obbs(self, q):
        return []


class _ScriptedConnector(LayoutTrajectoryConnector):
    def _plan_branch(self, **kwargs):
        q = np.asarray(kwargs["grasp_q"])
        if q[0] < 0.5:
            return None, {"reason": "FIRST_BRANCH_DISCONNECTED", "stage": "transit"}, {
                "seed": kwargs["seed"]
            }
        return _segment(), None, {"seed": kwargs["seed"]}


def test_grasp_branch_search_is_lazy_bounded_and_backtracks_to_alternative():
    connector = _ScriptedConnector(
        _Robot(),
        lambda *args, **kwargs: None,
        flange_from_virtual_task_tcp=np.eye(4),
        flange_from_physical_contact=np.eye(4),
        ik_policy={},
        collision_margin_m=0.01,
        contact_tolerance_m=0.0002,
        joint_margin_rad=0.01,
        maximum_jacobian_condition=1e4,
        validator_identity="synthetic_exact_validator",
        execution_qualified=True,
    )
    candidates = [
        {"candidate_id": index, "q_rad": [float(index), 0, 0, 0, 0, 0]}
        for index in range(5)
    ]
    outcome = connector.plan(
        target=OBB([0, 0, 1], [0.3, 0.2, 0.15], np.eye(3), "target", "carton"),
        face="top",
        requested_virtual_contact=np.eye(4),
        grasp_candidates=candidates,
        home_q=np.zeros(6),
        all_obstacles=[],
        receiver=OBB([1, 0, 0.5], [0.5, 0.5, 0.1], np.eye(3), "receiver", "conveyor"),
        support_names=[],
        suction={},
        seed=10,
    )
    assert outcome.success
    assert len(outcome.attempts) == 2
    assert [attempt["success"] for attempt in outcome.attempts] == [False, True]
    assert outcome.attempts[0]["trace"]["seed"] == 10
    assert outcome.attempts[1]["trace"]["seed"] == 10010


class _CountingCandidateStream:
    def __init__(self):
        self.next_calls = 0

    def __iter__(self):
        return self

    def __next__(self):
        self.next_calls += 1
        return SimpleNamespace(
            q=np.full(6, 0.1 * self.next_calls),
            position_error=0.0,
            orientation_error=0.0,
            search_evidence={"candidate_id": f"candidate_{self.next_calls}"},
        )

    def evidence(self):
        return {
            "seeds_attempted": self.next_calls,
            "iterations_consumed": self.next_calls,
        }


class _LazyConnectionConnector(LayoutTrajectoryConnector):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stream = _CountingCandidateStream()

    def _ik_stream(self, *args, **kwargs):
        return self.stream

    def _transit(self, start, goal, obstacles, **kwargs):
        return [], {"reason": "DISCONNECTED", "stage": kwargs["stage"]}, {
            "planning_iterations_consumed": 1,
        }


def test_stage_connection_limit_does_not_prefetch_an_unused_ik_candidate():
    connector = _LazyConnectionConnector(
        _Robot(),
        lambda *args, **kwargs: None,
        flange_from_virtual_task_tcp=np.eye(4),
        flange_from_physical_contact=np.eye(4),
        ik_policy={},
        collision_margin_m=0.01,
        contact_tolerance_m=0.0002,
        joint_margin_rad=0.01,
        maximum_jacobian_condition=1e4,
        validator_identity="synthetic_exact_validator",
        execution_qualified=True,
    )
    selected, path, failure, evidence = connector._connect_pose(
        np.eye(4),
        [np.zeros(6)],
        np.zeros(6),
        [],
        ik_seed=1,
        connection_seed=2,
        stage="transit",
    )
    assert selected is None and path == []
    assert failure["reason"] == "DISCONNECTED"
    assert connector.stream.next_calls == connector.budget.stage_connection_attempts
    assert len(evidence["attempts"]) == connector.budget.stage_connection_attempts


def test_official_srdf_policy_has_only_six_adjacent_exclusions(tmp_path):
    root = Path(__file__).resolve().parents[1]
    source = (
        root
        / "assets/robots/fanuc_m710id_70/official/m710id_70_official.srdf"
    )
    evidence = _audit_official_srdf_policy(source)
    assert len(evidence["disabled_self_collision_pairs"]) == 6
    assert evidence["non_adjacent_exclusions_allowed"] is False

    tampered = tmp_path / "tampered.srdf"
    tampered.write_text(
        source.read_text(encoding="utf-8").replace(
            "link1=\"base_link\" link2=\"J1_link\"",
            "link1=\"base_link\" link2=\"J6_link\"",
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="fixed adjacent pairs"):
        _audit_official_srdf_policy(tampered)


class _RecordingMeshRobot:
    dof = 6

    def __init__(self):
        self.calls = []

    def collision_result(self, q, obstacles, **kwargs):
        self.calls.append((tuple(box.name for box in obstacles), kwargs))
        return CollisionResult(False)


class _CompoundToolRobot:
    def tool_collision_obbs(self, q):
        return [
            OBB([0.01 * index, 0, 1], [0.001, 0.001, 0.001], np.eye(3), f"tool_rigid_{index}", "robot")
            for index in range(58)
        ]


def test_exact_validator_retains_target_and_limits_exceptions_to_mount_pairs():
    mesh = _RecordingMeshRobot()
    validator = ExactM710LayoutStateValidator(
        mesh,
        _CompoundToolRobot(),
        collision_margin_m=0.01,
        floor_z_m=0.0,
        right_wall_y_m=-2.0,
        left_wall_y_m=2.0,
        robot_world_boxes=lambda q: [],
    )
    target = OBB([3, 0, 1], [0.2, 0.2, 0.2], np.eye(3), "target", "carton")
    assert validator(
        np.zeros(6), [target], payload=None, target_contact=target, stage="contact"
    ) is None
    assert mesh.calls[0][0] == ("target",)
    assert mesh.calls[0][1]["ignored_geometry_obstacle_pairs"] == {
        ("base_link", "chassis")
    }
    assert len(mesh.calls[1][0]) == 58
    assert mesh.calls[1][1]["ignored_geometry_obstacle_pairs"] == {
        ("J6_link", f"tool_rigid_{index}") for index in range(58)
    }


def test_exact_validator_never_uses_contact_as_a_rigid_tool_collision_waiver():
    validator = ExactM710LayoutStateValidator(
        _RecordingMeshRobot(),
        _CompoundToolRobot(),
        collision_margin_m=0.01,
        floor_z_m=0.0,
        right_wall_y_m=-2.0,
        left_wall_y_m=2.0,
        robot_world_boxes=lambda q: [],
    )
    target = OBB([0, 0, 1], [0.02, 0.02, 0.02], np.eye(3), "target", "carton")
    failure = validator(
        np.zeros(6), [target], payload=None, target_contact=target, stage="contact"
    )
    assert failure == {
        "reason": "RIGID_TOOL_COLLISION",
        "pair": ["tool_rigid_0", "target"],
    }
