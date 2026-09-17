"""Shared, explicit object-pair assumptions for the offline M710 simulation."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from typing import Mapping, Sequence

import numpy as np

from .geometry import OBB
from .pair_clearance import obb_pair_failure, obb_surface_distance
from .validation_physics import contact_separated


@dataclass(frozen=True)
class SimulationCollisionPolicy:
    schema: str = "m710_simulation_collision_policy_v3"
    wrist_tool_exempt_links: tuple[str, ...] = ()
    stack_contact_mode: str = "strict_initial_proximity"
    stack_contact_stages: tuple[str, ...] = ("support-release", "extraction")
    maximum_planned_stack_penetration_m: float = 0.010
    free_space_clearance_m: float = 0.0202
    free_space_clearance_loss_tolerance_m: float = 0.0002
    maximum_actual_penetration_m: float = 0.010
    maximum_neighbor_displacement_m: float = 0.060
    maximum_neighbor_tilt_rad: float = 0.20
    progress_timeout_s: float = 3.0
    minimum_progress_m: float = 0.002
    inactive_compliant_cup_stack_contact_mode: str = "physical_contact_within_compression"
    compliant_cup_neighbor_contact_mode: str = "check"
    maximum_compliant_cup_additional_compression_m: float = 0.005
    required_pair_clearance_m: float | None = None
    self_collision_clearance_m: float = 0.0
    boundary_mode: str = "legacy_infinite_side_planes"

    def __post_init__(self):
        if self.schema not in {"m710_simulation_collision_policy_v3", "m710_poc_pair_collision_policy_v4"}:
            raise ValueError("unsupported collision policy")
        if self.poc_pair_clearance:
            if (self.required_pair_clearance_m != .005 or self.self_collision_clearance_m != 0.
                    or self.boundary_mode != "finite_frozen_scene_walls"
                    or abs(self.free_space_clearance_m-(.005+self.free_space_clearance_loss_tolerance_m)) > 1e-12):
                raise ValueError("POC pair policy requires 5 mm total gap, zero self gap and finite walls")
        elif self.required_pair_clearance_m is not None or self.boundary_mode != "legacy_infinite_side_planes":
            raise ValueError("pair clearance must be explicitly bound to the POC schema")
        if not set(self.wrist_tool_exempt_links) <= {"J5_link", "J6_link"}:
            raise ValueError("only the explicitly authorized wrist/tool pairs may be exempt")
        if self.stack_contact_mode not in {"strict_initial_proximity", "planner_relaxed_physics_checked"}:
            raise ValueError("unsupported stack contact mode")
        if not set(self.stack_contact_stages) <= {"support-release", "extraction"}:
            raise ValueError("stack planning relaxation must end before free transit")
        if self.inactive_compliant_cup_stack_contact_mode not in {
            "reject", "physical_contact_within_compression"
        }:
            raise ValueError("unsupported inactive compliant cup contact mode")
        if self.compliant_cup_neighbor_contact_mode not in {"check", "ignore"}:
            raise ValueError("unsupported compliant cup neighbor contact mode")
        for name, value in asdict(self).items():
            if isinstance(value, (int, float)) and (not np.isfinite(value) or value < 0):
                raise ValueError(f"invalid collision policy threshold: {name}")
        if self.free_space_clearance_loss_tolerance_m > self.free_space_clearance_m:
            raise ValueError("free-space loss tolerance cannot exceed its entry clearance")

    @classmethod
    def from_mapping(cls, value: Mapping | None):
        data = dict(value or {})
        recorded = data.pop("fingerprint", None)
        data.pop("wrist_tool_exemption_status", None)
        data.pop("box_box_physics_enabled", None)
        for name in ("wrist_tool_exempt_links", "stack_contact_stages"):
            if name in data:
                data[name] = tuple(data[name])
        result = cls(**data)
        if recorded is not None and recorded != result.fingerprint:
            raise ValueError("collision policy fingerprint mismatch")
        return result

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(self._data(), sort_keys=True, separators=(",", ":")).encode()).hexdigest()

    @property
    def poc_pair_clearance(self):
        return self.schema == "m710_poc_pair_collision_policy_v4"

    def pair_clearance(self, pair_kind, legacy_margin):
        if not self.poc_pair_clearance:
            return 2.0 * legacy_margin
        return self.self_collision_clearance_m if pair_kind == "robot_self" else self.required_pair_clearance_m

    def _data(self):
        data = asdict(self)
        if not self.poc_pair_clearance:
            for key in ("required_pair_clearance_m", "self_collision_clearance_m", "boundary_mode"):
                data.pop(key)
        return data

    def to_mapping(self):
        return {**self._data(), "fingerprint": self.fingerprint,
                "wrist_tool_exemption_status": "USER_APPROVED_SIMULATION_EXEMPTION" if self.wrist_tool_exempt_links else "NONE",
                "box_box_physics_enabled": True}

    def wrist_tool_pairs(self, owned_tool_names: Sequence[str]):
        return {(link, name) for link in self.wrist_tool_exempt_links for name in owned_tool_names}

    def allows_stack_planning_contact(self, stage: str):
        return self.stack_contact_mode == "planner_relaxed_physics_checked" and stage in self.stack_contact_stages


class PhysicsCheckedStackTracker:
    """Branch-local payload relaxation; non-stack objects keep their margin.

    The planner rejects gross box crossings, while Isaac handles touch/friction.
    A branch is released only when its actual predicted box clears ALL original
    stack boxes. Transit never consumes this relaxation.
    """
    def __init__(self, target: OBB, neighbors: Sequence[OBB], policy: SimulationCollisionPolicy,
                 margin_m: float, tolerance_m: float):
        self.target_id = target.name
        self.neighbors = {b.name: b for b in neighbors if b.name != target.name}
        self.policy = policy
        self.margin_m = margin_m
        self.tolerance_m = tolerance_m
        self.fully_released = not self.neighbors
        self.last_box = target
        self._update_released(target)

    def _update_released(self, box):
        self.last_box = box
        self.fully_released = self.fully_released or all(
            (obb_surface_distance(box, other) if self.policy.poc_pair_clearance else box.signed_distance_obb(other)) >= self.policy.free_space_clearance_m
            for other in self.neighbors.values())

    def clone(self):
        result = PhysicsCheckedStackTracker(self.last_box, tuple(self.neighbors.values()),
                                           self.policy, self.margin_m, self.tolerance_m)
        result.fully_released = self.fully_released
        return result

    def state_failure(self, box, obstacles, support_names=()):
        supports = set(support_names)
        for obstacle in obstacles:
            if obstacle.name in self.neighbors:
                distance = box.signed_distance_obb(obstacle)
                if self.fully_released:
                    failure = obb_pair_failure(box, obstacle, self.policy, self.margin_m, reason="PAYLOAD_COLLISION")
                    if failure is not None:
                        return {**failure, "contact_state": "FREE_SPACE_RULES_RESTORED"}
                if distance < -self.policy.maximum_planned_stack_penetration_m:
                    return {"reason": "GROSS_PLANNED_STACK_PENETRATION", "pair": [box.name, obstacle.name],
                            "signed_distance_m": distance}
                continue
            if obstacle.name in supports and contact_separated(box, obstacle, self.tolerance_m):
                continue
            failure = obb_pair_failure(box, obstacle, self.policy, self.margin_m, reason="PAYLOAD_COLLISION")
            if failure is not None:
                return failure
        self._update_released(box)
        return None

    def evidence(self):
        return {"mode": self.policy.stack_contact_mode, "target": self.target_id,
                "stack_carton_names": sorted(self.neighbors), "fully_released": self.fully_released,
                "policy_fingerprint": self.policy.fingerprint, "physics_collision_enabled": True}
