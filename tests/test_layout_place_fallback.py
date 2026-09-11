"""Exercise placement fallback through the actual complete connector call chain.

The synthetic robot has analytic Cartesian FK/IK and scripted edge blockage;
attachment transforms, support footprints, branch assembly and final segment
contracts are the production implementations.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from unloading_sim.geometry import OBB, make_transform, rotation_matrix_from_rpy
from unloading_sim.layout_trajectory import LayoutTrajectoryBudget, LayoutTrajectoryConnector


class AnalyticRobot:
    dof = 6
    joint_limits = np.tile([-10., 10.], (6, 1))
    base_transform = np.eye(4)

    def within_limits(self, q, tolerance=1e-9):
        return np.all(np.abs(q) <= 10 + tolerance)

    def fk(self, q):
        return make_transform(rotation_matrix_from_rpy(*np.asarray(q)[3:]), np.asarray(q)[:3])

    def geometric_jacobian(self, q):
        return np.eye(6)

    def named_link_frames(self, q):
        return {"flange": self.fk(q)}

    def tool_collision_obbs(self, q):
        return []

    def audited_tool_collision_obbs(self, q):
        return [OBB(np.asarray(q)[:3], [.10, .30, .05], np.eye(3),
                    "audited-tool-envelope", "tool")]

    @staticmethod
    def exact_q(pose):
        r = np.asarray(pose)[:3, :3]
        return np.r_[pose[:3, 3], np.arctan2(r[2, 1], r[2, 2]),
                     np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0])), np.arctan2(r[1, 0], r[0, 0])]


class ExactCandidateStream:
    def __init__(self, q):
        self.q = q
        self.count = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.count >= 3:
            raise StopIteration
        q = self.q.copy()
        q[3] += (0., 2*np.pi, -2*np.pi)[self.count]
        self.count += 1
        return SimpleNamespace(q=q, position_error=0., orientation_error=0.,
                               search_evidence={"candidate_id": self.count})

    def evidence(self):
        return {"seeds_attempted": self.count, "iterations_consumed": self.count}


class PlacementFallbackConnector(LayoutTrajectoryConnector):
    def __init__(self, failure_mode):
        robot = AnalyticRobot()
        super().__init__(robot, lambda *a, **kw: None,
            flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4),
            ik_policy={"position_tolerance_m": 1e-6, "orientation_tolerance_rad": 1e-6},
            collision_margin_m=.01, contact_tolerance_m=.0002, joint_margin_rad=.01,
            maximum_jacobian_condition=1e4, validator_identity="analytic_geometry_connector",
            execution_qualified=True, budget=LayoutTrajectoryBudget(stage_connection_iterations=24,
                local_transit_cartesian_sample_budget=0),
            surface_directions_world={
                "cross-support": [0., -1., 0.],
                "long-support": [-1., 0., 0.],
            }, tool_collision_obbs_provider=robot.audited_tool_collision_obbs)
        self.failure_mode = failure_mode
        self.transit_calls = []
        self.place_calls = []

    def _ik_stream(self, pose, *args, **kwargs):
        return ExactCandidateStream(self.robot.exact_q(pose))

    def _transit(self, start, goal, obstacles, *, iteration_budget, stage, **kwargs):
        if stage == "transit":
            self.transit_calls.append({"start": start.copy(), "goal": goal.copy(), "budget": iteration_budget})
            # Every first-receiver configuration exhausts its own allocation.
            # An alternative receiver is connected at a real subsequent call.
            if self.failure_mode == "connection" and goal[1] > -.4:
                return [], {"reason": "FIRST_RECEIVER_DISCONNECTED", "stage": stage}, {
                    "planning_iterations_consumed": iteration_budget}
        return [start.copy(), goal.copy()], None, {"planning_iterations_consumed": 1}

    def _cartesian(self, start, destination, obstacles, *, stage, **kwargs):
        goal = self.robot.exact_q(destination)
        if stage == "place":
            self.place_calls.append({"start": start.copy(), "goal": goal.copy(),
                                     "support_names": tuple(kwargs["support_names"])})
            if self.failure_mode == "place" and goal[1] > -.4:
                return [], {"reason": "FIRST_PLACE_PATH_BLOCKED", "stage": stage}, {}
        return [start.copy(), goal], None, {}

    def _extraction(self, start, attachment, obstacles, tracker, outward, *, seed):
        end = start.copy()
        end[0] -= .8
        end[2] += .05
        return [start.copy(), end], tracker, None, {"synthetic_extraction": True}

    def _contact_selection(self, q, target, face, suction):
        mask = [True] + [False]*71
        return {"suction_mode": "ideal_independent_cups", "load_bearing_minimum_cup_count": None,
            "actual_contact_count": 1, "commanded_active_count": 1,
            "geometrically_eligible_mask": mask.copy(), "commanded_active_mask": mask.copy(),
            "actual_contact_mask": mask.copy(), "target_id": target.name, "target_face": face,
            "enforce_vacuum_force_capacity": False, "enforce_vacuum_break_force": False,
            "enforce_vacuum_break_torque": False, "actual_q_rad": q.tolist(),
            "actual_virtual_task_tcp_pose_world": self.robot.fk(q).tolist(),
            "actual_physical_contact_pose_world": self.physical_from_virtual(self.robot.fk(q)).tolist()}


def plan(connector):
    target = OBB([.3, 0, 1.35], [.3, .2, .15], np.eye(3), "opaque-target", "carton")
    cross = OBB([-.55, .35, .54], [.35, .75, .06], np.eye(3), "cross-support", "conveyor")
    longitudinal = OBB([-1.6, -.75, .54], [1.4, .35, .06], np.eye(3), "long-support", "conveyor")
    grasp = np.array([.3, 0, 1.5, np.pi, 0, 0])
    return connector.plan(target=target, face="top", requested_virtual_contact=connector.robot.fk(grasp),
        grasp_candidates=[{"q_rad": grasp.tolist()}], home_q=[-1.2, 0, 1.2, np.pi, 0, 0],
        all_obstacles=[cross, longitudinal, target], receiver=cross, support_names=[], suction={}, seed=71070)


def test_failed_first_place_path_retries_second_receiver_from_same_extraction_endpoint():
    connector = PlacementFallbackConnector("place")
    result = plan(connector)
    assert result.success
    attempts = result.attempts[0]["trace"]["placement_attempts"]
    assert len(attempts) == 2
    assert attempts[0]["failure"]["reason"] == "FIRST_PLACE_PATH_BLOCKED"
    assert attempts[1]["failure"] is None
    assert result.segment["place"]["support"]["supported"]
    assert result.segment["place"]["load_bearing_support_names"] == ["long-support"]
    assert set(result.segment["place"]["support_names"]) == {"cross-support", "long-support"}
    assert len(connector.place_calls) == 2
    np.testing.assert_allclose(connector.transit_calls[0]["start"], connector.transit_calls[1]["start"])
    assert set(connector.place_calls[1]["support_names"]) == {"cross-support", "long-support"}


def test_first_receiver_exhaustion_preserves_shared_budget_for_alternative():
    connector = PlacementFallbackConnector("connection")
    result = plan(connector)
    assert result.success
    assert result.segment["place"]["load_bearing_support_names"] == ["long-support"]
    calls = connector.transit_calls
    assert any(item["goal"][1] > -.4 for item in calls)
    assert any(item["goal"][1] < -.4 for item in calls)
    # All failed paths used exactly their allowance; no per-placement copy of
    # the original shared total was created to obtain the second route.
    consumed = sum(item["budget"] if item["goal"][1] > -.4 else 1 for item in calls)
    assert consumed <= connector.budget.stage_connection_iterations
    assert all(item["budget"] > 0 for item in calls)


def test_withdrawal_ends_at_belt_aware_safe_residence_before_conveyor_start():
    connector = PlacementFallbackConnector(None)
    result = plan(connector)
    assert result.success
    residence = result.segment["post_release_safe_residence"]
    assert residence["conveyor_escape_enabled"] is True
    assert residence["conveyor_surface"] == "cross-support"
    np.testing.assert_allclose(residence["conveyor_direction_world"], [0., -1., 0.])
    assert residence["tool_solid_count"] == 1
    np.testing.assert_allclose(
        residence["unchanged_pair_clearance_m"], .0202, atol=1e-12
    )
    release_q = np.asarray(result.segment["path"])[result.segment["release_index"]]
    residence_q = np.asarray(result.segment["path"])[result.segment["release_retreat_index"]]
    # The empty tool first disengages normally, then ends above the released
    # carton by a full 3-D pair-clearance certificate. Horizontal belt travel
    # can only preserve that separation. This is part of the real segment,
    # not a replay-only pose edit.
    np.testing.assert_allclose(residence_q[1], release_q[1], atol=1e-12)
    assert residence_q[2] > release_q[2] + .39
