"""Exercise extraction, real DLS Cartesian continuation, and actual-box release.

The analytic Cartesian robot isolates finite IK residuals from reachability.
No IK result, path-valid boolean, or tracker release result is mocked.
"""

import numpy as np

from unloading_sim.collision_policy import PhysicsCheckedStackTracker, SimulationCollisionPolicy
from unloading_sim.geometry import OBB, make_transform, rotation_matrix_from_rpy
from unloading_sim.layout_trajectory import (
    LayoutTrajectoryBudget,
    LayoutTrajectoryConnector,
    PhysicalContactAttachment,
)
from unloading_sim.validation_physics import RigidAttachment
import unloading_sim.layout_trajectory as trajectory_module


class CartesianRobot:
    dof = 6
    joint_limits = np.tile([-4.0, 4.0], (6, 1))
    base_transform = np.eye(4)

    def clamp(self, q):
        return np.clip(np.asarray(q), self.joint_limits[:, 0], self.joint_limits[:, 1])

    def within_limits(self, q, tolerance=1e-9):
        q = np.asarray(q)
        return q.shape == (6,) and np.all(np.isfinite(q)) and np.all(np.abs(q) <= 4.0 + tolerance)

    def fk(self, q):
        q = np.asarray(q)
        return make_transform(rotation_matrix_from_rpy(*q[3:]), q[:3])

    def geometric_jacobian(self, q):
        # Test motions translate only.  At zero orientation the geometric
        # rotational Jacobian of these three Euler coordinates is identity.
        return np.eye(6)

    def named_link_frames(self, q):
        return {"flange": self.fk(q)}

    def tool_collision_obbs(self, q):
        return []


def fixture():
    robot = CartesianRobot()
    policy = SimulationCollisionPolicy(stack_contact_mode="planner_relaxed_physics_checked")

    def carriage_geometry(q, obstacles, **kwargs):
        pose = robot.fk(q)
        carriage = OBB(pose[:3, 3], [.005, .005, .005], pose[:3, :3], "carriage", "robot")
        for obstacle in obstacles:
            if carriage.intersects_obb(obstacle, margin=.01):
                return {"reason": "ROBOT_COLLISION", "pair": [carriage.name, obstacle.name]}
        return None

    connector = LayoutTrajectoryConnector(
        robot, carriage_geometry,
        flange_from_virtual_task_tcp=np.eye(4), flange_from_physical_contact=np.eye(4),
        ik_policy={"position_tolerance_m": 1e-4, "orientation_tolerance_rad": 2e-4,
                   "max_iterations": 60, "damping": .015, "max_step_rad": .2,
                   "orientation_weight": .55},
        collision_margin_m=.01, contact_tolerance_m=.0002,
        joint_margin_rad=.01, maximum_jacobian_condition=1e4,
        validator_identity="analytic_cartesian_robot_actual_payload_geometry",
        execution_qualified=True,
        budget=LayoutTrajectoryBudget(cartesian_step_m=.04, extraction_direction_attempts=2),
        collision_policy=policy.to_mapping(),
    )
    start = np.zeros(6)
    target = OBB([0, 0, 1], [.3, .2, .15], np.eye(3), "target", "carton")
    neighbor = OBB([0, .42, 1], [.3, .2, .15], np.eye(3), "neighbor", "carton")
    attachment = PhysicalContactAttachment(robot, RigidAttachment.capture(robot.fk(start), target), np.eye(4), np.eye(4))
    tracker = PhysicsCheckedStackTracker(target, [neighbor], policy, .01, .0002)
    assert not tracker.fully_released
    return connector, start, attachment, tracker, neighbor


def test_guard_reaches_actual_free_space_with_nonzero_strict_fk_residual():
    connector, start, attachment, tracker, neighbor = fixture()
    path, final_tracker, failure, evidence = connector._extraction(
        start, attachment, [neighbor], tracker, np.array([-1., 0., 0.]), seed=71070)
    assert failure is None
    assert final_tracker.fully_released
    chosen = evidence["attempts"][evidence["selected_attempt"]]
    # Actual IK retains a small admissible residual.  The added goal guard,
    # rather than a relaxed release predicate, leaves the actual box clear.
    residual = chosen["search"]["samples"][-1]["position_error_m"]
    assert 0.0 < residual < 1e-4
    assert chosen["fk_boundary_guard_m"] > residual
    actual_distance = attachment.box_at(path[-1]).signed_distance_obb(neighbor)
    assert actual_distance >= .0202
    assert chosen["free_clearance_m"] == .0202
    assert chosen["failure"] is None
    assert not tracker.fully_released  # failed/unused branch state stays local


def test_unbuffered_boundary_goal_cannot_falsely_complete_or_report_null_failure(monkeypatch):
    connector, start, attachment, tracker, neighbor = fixture()
    original = trajectory_module.minimum_clearance_extraction_distance

    def old_boundary_goal(*args, **kwargs):
        # Reproduce the old target-generation rule, while retaining today's
        # real Cartesian solver, every path state, and exact actual-box audit.
        kwargs["free_space_clearance_m"] = .0202
        return original(*args, **kwargs)

    monkeypatch.setattr(trajectory_module, "minimum_clearance_extraction_distance", old_boundary_goal)
    path, final_tracker, failure, evidence = connector._extraction(
        start, attachment, [neighbor], tracker, np.array([-1., 0., 0.]), seed=71070)
    assert path == []
    assert not final_tracker.fully_released
    assert failure["reason"] == "NO_BOUNDED_EXTRACTION_PATH"
    assert evidence["selected_attempt"] is None
    assert len(evidence["attempts"]) == 2
    for attempt in evidence["attempts"]:
        assert attempt["status"] == "REJECTED"
        assert 0.0 < attempt["search"]["samples"][-1]["position_error_m"] < 1e-4
        assert attempt["failure"]["reason"] == "ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED"
        assert not attempt["initial_proximity"]["fully_released"]
