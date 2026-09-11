"""Real payload descent at the two-deck seam through the task connector.

Only the upstream analytic arm routing is synthetic. The production attachment,
place-branch support discovery, path/state collision checks, final support-union
audit and segment assembly are consumed rather than replacing a contact verdict.
"""

from __future__ import annotations

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.layout_trajectory import ExactM710LayoutStateValidator
from unloading_sim.robot import CollisionResult
from test_layout_place_fallback import PlacementFallbackConnector, plan


class DescendingSeamConnector(PlacementFallbackConnector):
    def __init__(self, endpoint_penetration_m=0.):
        super().__init__("connection")
        self.endpoint_penetration_m = endpoint_penetration_m
        self.checked_descents = []

    def _cartesian(self, start, destination, obstacles, *, stage, **kwargs):
        if stage != "place":
            return super()._cartesian(start, destination, obstacles, stage=stage, **kwargs)
        goal = self.robot.exact_q(destination)
        goal[2] -= self.endpoint_penetration_m
        path = [start + fraction * (goal - start) for fraction in np.linspace(0., 1., 21)]
        attachment = kwargs["attachment"]
        support_names = kwargs["support_names"]
        failure = self._path_failure(path, obstacles, attachment=attachment,
            support_names=support_names, stage="place")
        self.checked_descents.append({"path": path, "attachment": attachment,
            "obstacles": tuple(obstacles), "support_names": tuple(support_names),
            "failure": failure})
        return ([] if failure else path), failure, {"checked_descent_samples": len(path)}


@pytest.mark.parametrize("penetration_m", [0., .0001])
def test_same_height_seam_descent_uses_both_real_decks_and_full_bottom_support(penetration_m):
    connector = DescendingSeamConnector(penetration_m)
    result = plan(connector)
    assert result.success
    descent = connector.checked_descents[-1]
    assert descent["failure"] is None
    assert set(descent["support_names"]) == {"cross-support", "long-support"}
    assert {box.name for box in descent["obstacles"]} == {"cross-support", "long-support"}
    payload = descent["attachment"].box_at(descent["path"][-1])
    # Literal seam at Y=-0.4 m: longitudinal carries all area, transverse
    # merely touches the footprint edge and is not deleted from the world.
    assert payload.corners()[:, 1].max() == pytest.approx(-.4)
    assert payload.corners()[:, 2].min() == pytest.approx(.6 - penetration_m)
    assert result.segment["place"]["load_bearing_support_names"] == ["long-support"]
    assert result.segment["place"]["support"]["supported"]
    assert connector._statistics["edge_state_samples"] >= 21
    # Recreate the former selected-only interpretation on the exact same
    # physical path: the neighbouring deck's normal margin rejects it.
    old_failure = connector._path_failure(descent["path"], descent["obstacles"],
        attachment=descent["attachment"], support_names=("long-support",), stage="place")
    assert old_failure["reason"] == "PAYLOAD_COLLISION"
    assert old_failure["pair"] == ["opaque-target", "cross-support"]


def test_actual_descent_beyond_existing_contact_tolerance_cannot_release():
    connector = DescendingSeamConnector(.0004)
    assert connector.endpoint_penetration_m > connector.contact_tolerance_m
    result = plan(connector)
    assert not result.success
    assert result.segment is None
    assert connector.checked_descents
    failures = [descent["failure"] for descent in connector.checked_descents]
    assert all(failure["reason"] == "PAYLOAD_COLLISION" for failure in failures)
    assert any(failure["pair"][1] == "long-support" for failure in failures)


class BoxCollisionBackend:
    """Tiny actual OBB backend for testing validator composition, not FANUC qualification."""

    dof = 6

    def __init__(self, box):
        self.box = box
        self.requested_margins = []

    def collision_result(self, q, obstacles, *, margin, ignored_geometry_obstacle_pairs, **kwargs):
        self.requested_margins.append(margin)
        for obstacle in obstacles:
            if (self.box.name, obstacle.name) not in ignored_geometry_obstacle_pairs \
                    and self.box.intersects_obb(obstacle, margin=margin):
                return CollisionResult(True, "OBB", self.box.name, obstacle.name)
        return CollisionResult(False)


@pytest.mark.parametrize("protected_body,expected_reason", [
    ("tool", "RIGID_TOOL_COLLISION"), ("robot", "ROBOT_MESH_COLLISION")])
def test_seam_support_does_not_waive_robot_or_rigid_tool_engineering_margin(protected_body, expected_reason):
    connector = DescendingSeamConnector()
    # This body has a real 10 mm deck gap, with neither penetration nor contact.
    # The unchanged 10 mm per-body inflation still requires 20 mm clearance.
    near = OBB([-.85, -.6, .615], [.02, .02, .005], np.eye(3), "guard", "robot")
    far_robot = OBB([-2., 0., 2.], [.02]*3, np.eye(3), "J2_link", "robot")
    far_tool = OBB([-2., .3, 2.], [.02]*3, np.eye(3), "tool_rigid_0", "tool")
    robot_box = OBB(near.center, near.half_extents, near.rotation, "J2_link", "robot") \
        if protected_body == "robot" else far_robot
    tool_box = OBB(near.center, near.half_extents, near.rotation, "tool_rigid_0", "tool") \
        if protected_body == "tool" else far_tool
    backend = BoxCollisionBackend(robot_box)
    tool_provider = type("ToolProvider", (), {"tool_collision_obbs": lambda self, q: [tool_box]})()
    validator = ExactM710LayoutStateValidator(backend, tool_provider,
        collision_margin_m=connector.collision_margin_m, floor_z_m=-1.,
        right_wall_y_m=-10., left_wall_y_m=10., robot_world_boxes=lambda q: [robot_box])
    connector.robot_state_validator = lambda q, obstacles, **kwargs: \
        validator(q, obstacles, **kwargs) if kwargs["stage"] == "place" else None
    result = plan(connector)
    assert not result.success
    assert connector.checked_descents
    first = connector.checked_descents[0]
    assert set(first["support_names"]) == {"cross-support", "long-support"}
    assert first["failure"]["reason"] == expected_reason
    longitudinal = next(box for box in first["obstacles"] if box.name == "long-support")
    assert not near.intersects_obb(longitudinal, margin=0.)
    assert near.intersects_obb(longitudinal, margin=.01)
    assert connector.collision_margin_m == .01
    assert backend.requested_margins and set(backend.requested_margins) == {.01}
