"""Auditable rigid attachment, primitive collision and external-load screens."""
from __future__ import annotations

from dataclasses import dataclass
import copy
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import OBB, make_transform, rotation_matrix_from_rpy
from .robot_load.model import cuboid_inertia_at_com
from .robot_load.spatial import spatial_inertia_at_point


@dataclass(frozen=True)
class RigidAttachment:
    tcp_from_box: np.ndarray
    half_extents: np.ndarray
    name: str

    @classmethod
    def capture(cls, actual_tcp: np.ndarray, box: OBB) -> "RigidAttachment":
        return cls(np.linalg.inv(actual_tcp) @ box.world_from_local, box.half_extents.copy(), box.name)

    def box_at(self, actual_tcp: np.ndarray) -> OBB:
        pose = actual_tcp @ self.tcp_from_box
        return OBB(pose[:3, 3], self.half_extents, pose[:3, :3], self.name, "payload")


def suction_coverage(tcp: np.ndarray, box: OBB, face: str, tool: dict, tolerance: float) -> dict:
    """Check complete circular seals on the actual box face, including roll."""
    axis, sign = {"front": (0, -1), "left": (1, 1), "right": (1, -1), "top": (2, 1)}[face]
    box_from_tcp = box.local_from_world @ tcp
    inward = np.eye(3)[:, axis] * -sign
    alignment = float(np.dot(box_from_tcp[:3, 2], inward))
    tangent = [i for i in range(3) if i != axis]
    centers = []
    sealed = []
    for row in range(tool["cup_rows"]):
        for col in range(tool["cup_columns"]):
            local = np.array([(row-(tool["cup_rows"]-1)/2)*tool["cup_pitch_m"][0],
                              (col-(tool["cup_columns"]-1)/2)*tool["cup_pitch_m"][1], 0.0])
            point = box_from_tcp[:3, :3] @ local + box_from_tcp[:3, 3]
            radius = tool["cup_radius_m"] + tool["suction_edge_margin_m"]
            covered = (abs(point[axis] - sign*box.half_extents[axis]) <= tolerance
                       and alignment >= 1.0 - 1e-6
                       and all(abs(point[j]) + radius <= box.half_extents[j] + 1e-12 for j in tangent))
            centers.append(point.tolist())
            sealed.append(bool(covered))
    return {"sealed_cups": sum(sealed), "required_cups": tool["minimum_sealed_cups"],
            "geometric_coverage": sum(sealed) >= tool["minimum_sealed_cups"],
            "cup_centers_box_m": centers, "sealed_mask": sealed,
            "suction_force_status": "NOT_EVALUATED", "normal_alignment": alignment}


def support_audit(actual_box: OBB, deck: OBB, tolerance: float, edge_clearance: float) -> dict:
    corners = (actual_box.corners() - deck.center) @ deck.rotation
    bottom = float(np.min(corners[:, 2]))
    gap = bottom - deck.half_extents[2]
    footprint = np.max(np.abs(corners[:, :2]), axis=0)
    # All four lowest face vertices must contact a horizontal support plane.
    low = np.sort(corners[:, 2])[:4]
    coplanar = float(np.ptp(low)) <= tolerance
    supported = (coplanar and abs(gap) <= tolerance
                 and bool(np.all(footprint + edge_clearance <= deck.half_extents[:2] + 1e-12)))
    return {"supported": supported, "bottom_gap_m": gap, "penetration_m": max(0.0, -gap),
            "edge_clearance_xy_m": (deck.half_extents[:2] - footprint).tolist(),
            "bottom_face_coplanar": coplanar, "actual_box_pose": actual_box.world_from_local.tolist()}


def contact_separated(box: OBB, support: OBB, tolerance: float) -> bool:
    """Only waive a positive separation margin at a designated support face.

    Reject side penetration and penetration deeper than numerical contact
    tolerance. This is not a blanket ignored obstacle.
    """
    points = (box.corners() - support.center) @ support.rotation
    return bool(np.min(points[:, 2]) >= support.half_extents[2] - tolerance)


@dataclass
class InitialProximityPair:
    """Path-dependent state for one pre-existing payload-neighbor proximity."""

    obstacle_name: str
    obstacle_pose_world: np.ndarray
    obstacle_half_extents_m: np.ndarray
    initial_signed_distance_m: float
    last_signed_distance_m: float
    minimum_signed_distance_m: float
    maximum_signed_distance_m: float
    worst_step_change_m: float = 0.0
    samples: int = 0
    released: bool = False
    release_sample: int | None = None


@dataclass
class InitialProximityTracker:
    """Allow only non-worsening separation from explicitly registered neighbors.

    This contract is deliberately scoped to payload-vs-stationary-neighbor
    checks.  Obstacles not captured here retain the ordinary two-body OBB
    margin, and robot/tool collision checks never consult this object.
    """

    payload_name: str
    collision_margin_m: float
    penetration_tolerance_m: float
    monotonic_tolerance_m: float
    pairs: dict[str, InitialProximityPair]

    @classmethod
    def capture(
        cls,
        payload: OBB,
        eligible_neighbors: list[OBB],
        collision_margin_m: float,
        penetration_tolerance_m: float,
        monotonic_tolerance_m: float,
    ) -> tuple["InitialProximityTracker", dict | None]:
        """Register close, non-penetrating pairs from the actual initial pose."""
        names = [box.name for box in eligible_neighbors]
        if len(names) != len(set(names)):
            raise ValueError("initial-proximity obstacle names must be unique")
        pairs: dict[str, InitialProximityPair] = {}
        tracker = cls(
            payload.name,
            float(collision_margin_m),
            float(penetration_tolerance_m),
            float(monotonic_tolerance_m),
            pairs,
        )
        for obstacle in eligible_neighbors:
            if not payload.intersects_obb(obstacle, margin=collision_margin_m):
                continue
            distance = payload.signed_distance_obb(obstacle)
            if distance < -penetration_tolerance_m:
                return tracker, {
                    "reason": "PAYLOAD_INITIAL_PENETRATION",
                    "pair": [payload.name, obstacle.name],
                    "initial_signed_distance_m": distance,
                    "penetration_tolerance_m": float(penetration_tolerance_m),
                }
            pairs[obstacle.name] = InitialProximityPair(
                obstacle.name,
                obstacle.world_from_local.copy(),
                obstacle.half_extents.copy(),
                distance,
                distance,
                distance,
                distance,
            )
        return tracker, None

    def clone(self) -> "InitialProximityTracker":
        """Return independent path history for another deterministic attempt."""
        return copy.deepcopy(self)

    @property
    def fully_released(self) -> bool:
        return all(pair.released for pair in self.pairs.values())

    def state_failure(self, payload: OBB, obstacles: list[OBB], support_names=()) -> dict | None:
        """Validate and advance one ordered payload-path sample."""
        if payload.name != self.payload_name:
            raise ValueError("initial-proximity tracker used with a different payload")
        by_name = {box.name: box for box in obstacles}
        if len(by_name) != len(obstacles):
            raise ValueError("payload obstacle names must be unique")
        missing = sorted(set(self.pairs) - set(by_name))
        if missing:
            return {"reason": "INITIAL_PROXIMITY_OBSTACLE_MISSING", "obstacles": missing}

        support_names = set(support_names)
        for obstacle in obstacles:
            if obstacle.name in support_names and contact_separated(
                payload, obstacle, self.penetration_tolerance_m
            ):
                continue
            pair = self.pairs.get(obstacle.name)
            if pair is None:
                if payload.intersects_obb(obstacle, margin=self.collision_margin_m):
                    return {
                        "reason": "PAYLOAD_COLLISION",
                        "pair": [payload.name, obstacle.name],
                        "initial_proximity_registered": False,
                    }
                continue

            if not np.allclose(obstacle.world_from_local, pair.obstacle_pose_world, atol=1e-12, rtol=0) \
                    or not np.array_equal(obstacle.half_extents, pair.obstacle_half_extents_m):
                return {
                    "reason": "INITIAL_PROXIMITY_OBSTACLE_MOVED",
                    "pair": [payload.name, obstacle.name],
                }

            distance = payload.signed_distance_obb(obstacle)
            common = {
                "pair": [payload.name, obstacle.name],
                "initial_signed_distance_m": pair.initial_signed_distance_m,
                "previous_signed_distance_m": pair.last_signed_distance_m,
                "current_signed_distance_m": distance,
                "penetration_tolerance_m": self.penetration_tolerance_m,
                "monotonic_tolerance_m": self.monotonic_tolerance_m,
            }
            if distance < -self.penetration_tolerance_m:
                return {"reason": "PAYLOAD_PROXIMITY_PENETRATION", **common}
            if pair.released:
                if payload.intersects_obb(obstacle, margin=self.collision_margin_m):
                    return {"reason": "PAYLOAD_PROXIMITY_REENTRY", **common}
            elif (distance < pair.initial_signed_distance_m - self.monotonic_tolerance_m
                  or distance < pair.last_signed_distance_m - self.monotonic_tolerance_m):
                return {"reason": "PAYLOAD_PROXIMITY_WORSENED", **common}

            change = distance - pair.last_signed_distance_m
            pair.samples += 1
            pair.last_signed_distance_m = distance
            pair.minimum_signed_distance_m = min(pair.minimum_signed_distance_m, distance)
            pair.maximum_signed_distance_m = max(pair.maximum_signed_distance_m, distance)
            pair.worst_step_change_m = min(pair.worst_step_change_m, change)
            if not pair.released and not payload.intersects_obb(
                obstacle, margin=self.collision_margin_m
            ):
                pair.released = True
                pair.release_sample = pair.samples
        return None

    def evidence(self) -> dict:
        return {
            "payload": self.payload_name,
            "collision_margin_per_body_m": self.collision_margin_m,
            "penetration_tolerance_m": self.penetration_tolerance_m,
            "monotonic_tolerance_m": self.monotonic_tolerance_m,
            "fully_released": self.fully_released,
            "pairs": [
                {
                    "obstacle": pair.obstacle_name,
                    "initial_signed_distance_m": pair.initial_signed_distance_m,
                    "last_signed_distance_m": pair.last_signed_distance_m,
                    "minimum_signed_distance_m": pair.minimum_signed_distance_m,
                    "maximum_signed_distance_m": pair.maximum_signed_distance_m,
                    "worst_step_change_m": pair.worst_step_change_m,
                    "samples": pair.samples,
                    "released": pair.released,
                    "release_sample": pair.release_sample,
                }
                for pair in self.pairs.values()
            ],
        }


def urdf_collision_shapes(robot) -> list[tuple[str, np.ndarray, np.ndarray]]:
    """Read every URDF collision primitive; cylinders/spheres use enclosing OBBs.

    Previously only centreline capsules were read, missing box corners.
    Enclosing OBBs may reject valid motion but cannot drop a primitive corner.
    """
    result = []
    for link in ET.parse(robot.urdf_path).getroot().findall("link"):
        for element in link.findall("collision"):
            origin = element.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            geometry = element.find("geometry")
            if geometry.find("box") is not None:
                size = np.fromstring(geometry.find("box").get("size"), sep=" ")
            elif geometry.find("cylinder") is not None:
                cylinder = geometry.find("cylinder")
                size = np.array([2*float(cylinder.get("radius"))]*2 + [float(cylinder.get("length"))])
            elif geometry.find("sphere") is not None:
                size = np.full(3, 2*float(geometry.find("sphere").get("radius")))
            else:
                raise ValueError("unhandled URDF collision geometry; cannot silently omit")
            result.append((link.get("name"), make_transform(rotation_matrix_from_rpy(*rpy), xyz), size/2))
    return result


def world_link_boxes(robot, q, shapes) -> list[OBB]:
    frames = robot.named_link_frames(q)
    result = []
    for name, offset, half in shapes:
        pose = frames[name] @ offset
        result.append(OBB(pose[:3, 3], half, pose[:3, :3], name, "robot"))
    return result


def aggregate_status(statuses) -> str:
    values = list(statuses)
    if "FAIL" in values:
        return "FAIL"
    return "PASS" if values and all(value == "PASS" for value in values) else "NOT_EVALUATED"


def external_load(robot, tool, q, attachment: RigidAttachment | None, box_mass: float,
                  com_fraction, model: dict, qd=None, qdd=None) -> dict:
    """Rigid tool + same attached box used in collision checks, in world space.

    Computes external Newton-Euler wrench about actual wrist axes. No arm-link
    mass or drive torque is inferred from allowable external wrist moments.
    """
    q = np.asarray(q, float)
    qd = np.zeros(robot.dof) if qd is None else np.asarray(qd, float)
    qdd = np.zeros(robot.dof) if qdd is None else np.asarray(qdd, float)
    h = 1e-4
    def bodies(qi):
        flange = robot.named_link_frames(qi)["flange"]
        specs = [(tool.mass_kg, np.asarray(tool.com_xyz_m), np.asarray(tool.inertia_tensor_com_kg_m2), flange)]
        if attachment is not None:
            box = attachment.box_at(robot.fk(qi))
            size = 2*box.half_extents
            specs.append((box_mass, np.asarray(com_fraction)*size,
                          cuboid_inertia_at_com(box_mass, size), box.world_from_local))
        return [(m, pose[:3, :3] @ com + pose[:3, 3], pose[:3, :3] @ inertia @ pose[:3, :3].T)
                for m, com, inertia, pose in specs]
    bodies0 = bodies(q)
    plus = bodies(q + h*qd + h*h*qdd/2)
    minus = bodies(q - h*qd + h*h*qdd/2)
    jac = robot.geometric_jacobian(q)
    jplus = robot.geometric_jacobian(q+h*qd)
    jminus = robot.geometric_jacobian(q-h*qd)
    omega = jac[3:] @ qd
    alpha = jac[3:] @ qdd + ((jplus[3:]-jminus[3:])/(2*h)) @ qd
    gravity = np.array([0, 0, -9.80665])
    mass = sum(body[0] for body in bodies0)
    com = sum(m*c for m,c,_ in bodies0)/mass
    flange = robot.named_link_frames(q)["flange"]
    com_flange = flange[:3,:3].T @ (com-flange[:3,3])
    axes = robot.joint_axis_frames(q)
    moments, static, inertias = [], [], []
    for name in ("J4", "J5", "J6"):
        origin, axis = axes[name]
        moment, grav, inertia_axis = 0.0, 0.0, 0.0
        for (m,c,I), (_,cp,_), (_,cm,_) in zip(bodies0, plus, minus):
            acceleration = (cp-2*c+cm)/(h*h)
            moment += float(axis @ (np.cross(c-origin, m*(acceleration-gravity)) + I@alpha + np.cross(omega,I@omega)))
            grav += float(axis @ np.cross(c-origin, -m*gravity))
            inertia_axis += float(axis @ spatial_inertia_at_point(I,m,c,origin) @ axis)
        moments.append(moment); static.append(grav); inertias.append(inertia_axis)
    # Only axis intercepts are transcribed. Outside is a definite failure;
    # inside their rectangle does not establish membership in the full curve.
    curves = model["resolved_payload_evidence"]["public_numeric_values"]["diagram_axis_intercepts_m"]
    lower = sorted((float(key.removesuffix("kg")), value) for key,value in curves.items() if float(key.removesuffix("kg")) <= mass)
    radial = float(np.linalg.norm(com_flange[1:]))
    axial = abs(float(com_flange[0]))
    # Exceeding even the next-LOWER mass curve's intercept proves failure
    # under the documented monotone payload envelope. Exceeding only the
    # next-higher conservative screen must not fabricate a physical failure.
    boundary = None if not lower else lower[-1][1]
    cg = "FAIL" if boundary and (axial > boundary["axial_z"] or radial > boundary["radial_xy"]) else "NOT_EVALUATED"
    allow_m = np.array([model["wrist_limits"]["allowable_moment_nm"][k] for k in ("J4","J5","J6")])
    allow_i = np.array([model["wrist_limits"]["allowable_inertia_kg_m2"][k] for k in ("J4","J5","J6")])
    statuses = {"rated_payload": "PASS" if mass <= model["rated_payload_kg"] else "FAIL",
                "payload_cg": cg,
                "wrist_moment": "FAIL" if np.any(np.abs(moments)>allow_m) else "NOT_EVALUATED",
                "wrist_inertia": "FAIL" if np.any(np.asarray(inertias)>allow_i) else "NOT_EVALUATED",
                "joint_drive": "NOT_EVALUATED", "suction_evidence": "NOT_EVALUATED",
                "full_robot_dynamics": "NOT_EVALUATED"}
    return {"statuses": statuses, "qualification": aggregate_status(statuses.values()),
            "combined_mass_kg": mass, "combined_com_flange_m": com_flange.tolist(),
            "box_pose_world": None if attachment is None else attachment.box_at(robot.fk(q)).world_from_local.tolist(),
            "gravity_moment_nm": static, "external_dynamic_moment_nm": moments,
            "wrist_axis_inertia_kg_m2": inertias,
            "moment_utilization": (np.abs(moments)/allow_m).tolist(),
            "inertia_utilization": (np.asarray(inertias)/allow_i).tolist(),
            "method": "same_rigid_attachment_Newton_Euler_external_load_only",
            "cg_failure_reference": None if not lower else {"payload_kg":lower[-1][0],"axis_intercepts_m":boundary},
            "evidence_scope": "engineering_screen_missing_certified_drive_and_complete_curve_data"}
