"""Actual-state handoff between tasks in one continuously running physical world.

This module never resets Isaac, places a carton, or declares physical success.
Only explicit completion and handoff events classify objects; coordinates alone
cannot remove one.  The initial identity registry is derived from the verified
scene, while every subsequent pose and planner start comes from measured state.
"""

from __future__ import annotations

import copy
from dataclasses import replace
from typing import Any, Callable, Iterable, Mapping

import numpy as np

from .geometry import OBB
from .layout_single_carton import FrozenLayoutMotionInput
from .support import SupportRelationGraph
from .unloading_sequence import RowUnloadingState
from .validation_physics import urdf_collision_shapes, world_link_boxes
from .workcell_layout import canonical_digest, verify_scene_snapshot


ACTUAL_MOTION_STATE_SCHEMA = "m710id70_actual_motion_state_v1"


def _vector(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite vector of length {size}")
    return result


def _identity_set(value: Any, name: str) -> set[str]:
    if not isinstance(value, (list, tuple)) or any(not isinstance(item, str) or not item for item in value):
        raise ValueError(f"{name} must be a list of nonempty carton identities")
    result = set(value)
    if len(result) != len(value):
        raise ValueError(f"{name} contains duplicate identities")
    return result


def rotation_from_actual_quaternion(quaternion_wxyz: Any) -> np.ndarray:
    """Normalize floating-point quaternion drift, reject malformed rotations."""
    q = _vector(quaternion_wxyz, 4, "orientation_wxyz")
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("actual quaternion must be unit length within numerical tolerance")
    w, x, y, z = q / norm
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def state_from_isaac_record(record: Mapping[str, Any], *, completed_carton_ids: Iterable[str] = (),
                            handed_off_ids: Iterable[str] = ()) -> dict[str, Any]:
    """Convert an actual-state array record without inventing missing channels."""
    names = list(record["carton_names"])
    channels = {
        "position_m": "carton_positions_m",
        "orientation_wxyz": "carton_orientations_wxyz",
        "linear_velocity_m_s": "carton_linear_velocities_m_s",
        "angular_velocity_rad_s": "carton_angular_velocities_rad_s",
    }
    for source in channels.values():
        if source not in record or len(record[source]) != len(names):
            raise ValueError(f"actual record missing or inconsistent channel: {source}")
    cartons = [{"name": name, **{destination: list(record[source][index])
                                for destination, source in channels.items()}}
               for index, name in enumerate(names)]
    state = {"schema": ACTUAL_MOTION_STATE_SCHEMA, "q_rad": list(record["q_rad"]),
             "cartons": cartons, "completed_carton_ids": sorted(completed_carton_ids),
             "handed_off_ids": sorted(handed_off_ids)}
    for optional in ("time_s", "joint_names", "qd_rad_s", "world_session_id", "attached"):
        if optional in record:
            state[optional] = copy.deepcopy(record[optional])
    return state


def _record(box: OBB) -> dict[str, Any]:
    return {"name": box.name, "category": box.category, "shape": "box",
            "pose_world": box.world_from_local.tolist(), "center_m": box.center.tolist(),
            "half_extents_m": box.half_extents.tolist()}


def apply_actual_motion_state(scene: FrozenLayoutMotionInput, state: Mapping[str, Any], *,
                               row_state: RowUnloadingState | None = None) -> FrozenLayoutMotionInput:
    """Create the next immutable CPU scene from actual surviving rigid bodies.

    ``completed_carton_ids`` and ``handed_off_ids`` are cumulative trusted
    execution events, not classifier guesses.  Handed-off objects must no longer
    occur in the active body record; all other original objects must occur once.
    New body names, omitted neighbors and reverted completion events fail closed.
    """
    if state.get("schema", ACTUAL_MOTION_STATE_SCHEMA) != ACTUAL_MOTION_STATE_SCHEMA:
        raise ValueError("unsupported actual motion state schema")
    if state.get("attached") or state.get("attachment_target") or state.get("attachments"):
        raise ValueError("between-task scene update requires actual attachment release")
    snapshot = copy.deepcopy(dict(scene.snapshot))
    # Verify the parent's content fingerprint before deriving a child identity.
    verify_scene_snapshot(snapshot)
    previous = snapshot.get("actual_state_context", {})
    registry = copy.deepcopy(previous.get("initial_carton_registry", snapshot["cartons"]))
    initial_names = _identity_set([record["name"] for record in registry], "initial carton registry")
    definitions = {record["name"]: record for record in registry}
    completed = _identity_set(state.get("completed_carton_ids", []), "completed_carton_ids")
    handed_off = _identity_set(state.get("handed_off_ids", []), "handed_off_ids")
    if not handed_off <= completed <= initial_names:
        raise ValueError("handoff must be a subset of completed original carton identities")
    if not set(previous.get("completed_carton_ids", [])) <= completed:
        raise ValueError("completed carton events must not regress")
    if not set(previous.get("handed_off_ids", [])) <= handed_off:
        raise ValueError("handoff events must not regress or resurrect a carton")
    expected_active = initial_names - handed_off
    records = state.get("cartons")
    if not isinstance(records, (list, tuple)) or any(not isinstance(item, Mapping) for item in records):
        raise ValueError("actual cartons must be a sequence of body records")
    observed = _identity_set([item.get("name") for item in records], "actual carton records")
    if observed != expected_active:
        raise ValueError(f"actual carton identity mismatch: missing={sorted(expected_active-observed)}, "
                         f"unknown_or_handed_off={sorted(observed-expected_active)}")
    joint_names = tuple(snapshot["robot"]["joint_names"])
    if "joint_names" in state and tuple(state["joint_names"]) != joint_names:
        raise ValueError("actual joint order must match the verified official model")
    q = _vector(state["q_rad"], len(joint_names), "actual q_rad")
    if "qd_rad_s" in state:
        _vector(state["qd_rad_s"], len(joint_names), "actual qd_rad_s")
    world_session = state.get("world_session_id", previous.get("world_session_id"))
    if previous.get("world_session_id") is not None and world_session != previous["world_session_id"]:
        raise ValueError("serial task state must come from the same physical world session")
    time_s = state.get("time_s")
    if time_s is not None:
        if not np.isfinite(float(time_s)) or float(time_s) < 0:
            raise ValueError("actual time_s must be finite and nonnegative")
        if previous.get("time_s") is not None and float(time_s) < float(previous["time_s"]) - 1e-12:
            raise ValueError("physical simulation time must not move backwards")
    cartons = []
    velocities = {}
    for item in sorted(records, key=lambda record: record["name"]):
        name = item["name"]
        position = _vector(item["position_m"], 3, f"{name}.position_m")
        rotation = rotation_from_actual_quaternion(item["orientation_wxyz"])
        linear = _vector(item["linear_velocity_m_s"], 3, f"{name}.linear_velocity_m_s")
        angular = _vector(item["angular_velocity_rad_s"], 3, f"{name}.angular_velocity_rad_s")
        definition = definitions[name]
        if "half_extents_m" in item and not np.array_equal(np.asarray(item["half_extents_m"]), definition["half_extents_m"]):
            raise ValueError("actual state cannot change a carton's fixed geometry")
        cartons.append(OBB(position, definition["half_extents_m"], rotation, name, definition["category"]))
        velocities[name] = {"linear_velocity_m_s": linear.tolist(), "angular_velocity_rad_s": angular.tolist()}
    actual_cartons = tuple(cartons)
    remaining_names = tuple(sorted(initial_names - completed))
    remaining = tuple(box for box in actual_cartons if box.name in remaining_names)
    population = scene.policy.data["task_population"]
    graph = SupportRelationGraph.build(remaining,
        contact_tolerance_m=float(population["support_contact_tolerance_m"]),
        minimum_overlap_ratio=float(population["support_minimum_overlap_ratio"]))
    selector = row_state or RowUnloadingState()
    selection = selector.rank(remaining, support_graph=graph,
        scene_context={"receiver_occupancy": [_record(box) for box in actual_cartons if box.name in completed],
                       "q_rad": q.tolist()})
    # Only the mutable initial-state section changes. Fixed layout, model and
    # collision policy data continue to identify the same approved assembly.
    validation_data = copy.deepcopy(dict(scene.policy.layout_validation.data))
    validation_data["initial_state"] = {**validation_data["initial_state"], "q_rad": q.tolist()}
    validation = replace(scene.policy.layout_validation, data=validation_data)
    policy = replace(scene.policy, layout_validation=validation)
    robot = policy.layout_validation.layout.robot()
    if not robot.within_limits(q):
        raise ValueError("actual planner start violates official joint limits")
    snapshot["robot"]["q_rad"] = q.tolist()
    if "qd_rad_s" in state:
        snapshot["robot"]["qd_rad_s"] = list(state["qd_rad_s"])
    snapshot["robot"]["flange_pose_world"] = robot.named_link_frames(q)["flange"].tolist()
    snapshot["robot"]["tcp_pose_world"] = robot.fk(q).tolist()
    snapshot["robot"]["link_collision_obbs"] = [_record(box) for box in world_link_boxes(robot, q, urdf_collision_shapes(robot))]
    snapshot["tool"]["task_tcp_pose_world"] = robot.fk(q).tolist()
    tool_boxes = robot.tool_collision_obbs(q)
    snapshot["tool"]["rigid_collision_obbs"] = [_record(box) for box in tool_boxes]
    if tool_boxes:
        frame = robot.fk(q)
        local = np.concatenate([(box.corners() - frame[:3, 3]) @ frame[:3, :3] for box in tool_boxes])
        lo, hi = local.min(axis=0), local.max(axis=0)
        envelope = OBB(frame[:3, :3] @ ((lo + hi) / 2) + frame[:3, 3], (hi - lo) / 2,
                       frame[:3, :3], "tool_compound_display_envelope", "robot")
        snapshot["tool"]["collision_obb"] = {**_record(envelope),
            "role": "DISPLAY_AND_SNAPSHOT_ENVELOPE_NOT_EXECUTION_COLLISION"}
    snapshot["cartons"] = [_record(box) for box in actual_cartons]
    snapshot["attachments"] = []
    snapshot["receiver"] = {**snapshot.get("receiver", {}),
        "state": "OCCUPIED" if completed - handed_off else "EMPTY",
        "occupied_carton_ids": sorted(completed - handed_off)}
    snapshot["actual_state_context"] = {
        "schema": ACTUAL_MOTION_STATE_SCHEMA, "source": "ACTUAL_RIGID_BODY_STATE",
        "parent_scene_fingerprint": scene.snapshot["scene_fingerprint"],
        "initial_carton_registry": registry, "initial_carton_count": len(initial_names),
        "completed_carton_ids": sorted(completed), "handed_off_ids": sorted(handed_off),
        "remaining_stack_names": list(remaining_names), "active_body_count": len(actual_cartons),
        "world_session_id": world_session, "time_s": time_s, "carton_velocities": velocities,
        "row_selection": selection.as_dict(), "actual_state_fingerprint": canonical_digest(state),
        "completion_source": "EXPLICIT_EXECUTION_EVENTS_NOT_POSITION_CLASSIFICATION",
    }
    # Historical initial audit is retained with its original scope; it cannot
    # be mistaken for validation of the newly observed planner start.
    snapshot.setdefault("initial_scene_audit", copy.deepcopy(snapshot.get("initial_state_audit", {})))
    snapshot["initial_state_audit"] = {"status": "REQUIRES_ACTUAL_START_VALIDATION",
        "source": "ACTUAL_RIGID_BODY_STATE", "q_rad": q.tolist()}
    snapshot.pop("scene_fingerprint", None)
    snapshot["scene_fingerprint"] = canonical_digest(snapshot)
    verification = verify_scene_snapshot(snapshot)
    consistency = {"status": "PASS", "scope": "ACTUAL_STATE_IDENTITIES_FIXED_GEOMETRY_AND_LAYOUT",
        "collision_validity": "CHECKED_BY_NEXT_TASK_START_VALIDATOR",
        "layout_fingerprint": snapshot["layout_fingerprint"],
        "scene_fingerprint": snapshot["scene_fingerprint"], "initial_carton_count": len(initial_names),
        "active_carton_count": len(actual_cartons), "remaining_stack_count": len(remaining_names),
        "receiver_occupancy_count": len(completed - handed_off)}
    return replace(scene, policy=policy, snapshot=snapshot, snapshot_verification=verification,
        snapshot_consistency=consistency, cartons=actual_cartons, support_graph=graph,
        removable_cartons=tuple(item.name for item in selection.candidates),
        remaining_stack_names=remaining_names)


class SerialUnloadingSession:
    """CPU entry point held by the adapter for bounded between-task planning."""

    def __init__(self, scene: FrozenLayoutMotionInput, *, maximum_completed_tasks: int = 5,
                 row_state: RowUnloadingState | None = None) -> None:
        if maximum_completed_tasks < 1:
            raise ValueError("serial execution budget must be positive")
        self.scene = scene
        self.row_state = row_state or RowUnloadingState()
        self.maximum_completed_tasks = int(maximum_completed_tasks)
        self.completed_at_start = len(scene.snapshot.get("actual_state_context", {}).get("completed_carton_ids", []))
        remaining = scene.cartons if scene.remaining_stack_names is None else tuple(
            box for box in scene.cartons if box.name in scene.remaining_stack_names)
        self.row_state.rank(remaining, support_graph=scene.support_graph)

    def update_actual_state(self, state: Mapping[str, Any]) -> FrozenLayoutMotionInput:
        self.scene = apply_actual_motion_state(self.scene, state, row_state=self.row_state)
        return self.scene

    def plan_next(self, *, progress_callback: Callable[..., Any] | None = None,
                  planner: Callable[..., Any] | None = None, **planner_kwargs: Any) -> Any:
        completed = len(self.scene.snapshot.get("actual_state_context", {}).get("completed_carton_ids", []))
        if completed - self.completed_at_start >= self.maximum_completed_tasks:
            return {"status": "EXECUTION_BUDGET_EXHAUSTED", "completed_carton_count": completed,
                    "remaining_stack_names": list(self.scene.remaining_stack_names or ())}
        remaining = self.scene.cartons if self.scene.remaining_stack_names is None else tuple(
            box for box in self.scene.cartons if box.name in self.scene.remaining_stack_names)
        selection = self.row_state.rank(remaining, support_graph=self.scene.support_graph)
        if selection.status != "READY":
            return selection.as_dict()
        if planner is None:
            from .layout_single_carton import run_layout_single_carton_audit
            planner = run_layout_single_carton_audit
        return planner(self.scene.policy, motion_input=self.scene, row_state=self.row_state,
                       progress_callback=progress_callback, **planner_kwargs)
