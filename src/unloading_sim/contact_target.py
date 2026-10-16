"""One deterministic physical contact target per exposed box face (SI, world)."""
from dataclasses import dataclass
import numpy as np

from .geometry import OBB
from .independent_cups import _finite_se3
from .support import SupportRelationGraph

# These are geometric legacy labels, never semantic priority labels.
GEOMETRIC_FACES = {(0, -1): "front", (1, 1): "left", (1, -1): "right", (2, 1): "top"}


@dataclass(frozen=True)
class ContactTarget:
    target_id: str
    frame_id: str
    scene_revision: str
    source: str
    face_role: str
    geometric_face_id: str
    box: OBB
    axis: int
    sign: int
    face_center: np.ndarray
    face_boundary: np.ndarray
    outward_normal: np.ndarray
    in_plane_x: np.ndarray
    contact_pose: np.ndarray
    approach_pose: np.ndarray
    standoff_m: float

    @property
    def legacy_face(self):
        if (self.axis, self.sign) not in GEOMETRIC_FACES:
            raise ValueError("UNSUPPORTED_GEOMETRIC_FACE_MAPPING")
        return GEOMETRIC_FACES[self.axis, self.sign]

    def validate(self):
        if (self.frame_id != "world" or not self.scene_revision or not self.source
                or self.target_id != self.box.name or self.axis not in range(3)
                or self.sign not in (-1, 1) or self.face_role not in ("FRONT", "TOP", "SIDE")):
            raise ValueError("INVALID_CONTACT_TARGET_IDENTITY")
        if self.geometric_face_id != f"{'xyz'[self.axis]}{'+' if self.sign > 0 else '-'}":
            raise ValueError("INVALID_GEOMETRIC_FACE_ID")
        if not np.isfinite(self.standoff_m) or self.standoff_m <= 0:
            raise ValueError("INVALID_STANDOFF")
        contact = _finite_se3(self.contact_pose, "contact_pose")
        approach = _finite_se3(self.approach_pose, "approach_pose")
        n = self.box.rotation[:, self.axis] * self.sign
        center = self.box.center + self.box.half_extents[self.axis] * n
        if (not np.allclose(self.outward_normal, n, atol=1e-12, rtol=0)
                or not np.allclose(self.face_center, center, atol=1e-12, rtol=0)
                or not np.allclose(contact[:3, 2], -n, atol=1e-12, rtol=0)
                or not np.allclose(contact[:3, 0], self.in_plane_x, atol=1e-12, rtol=0)
                or not np.allclose(approach[:3, :3], contact[:3, :3], atol=1e-12, rtol=0)
                or not np.allclose(approach[:3, 3], contact[:3, 3]+self.standoff_m*n, atol=1e-12, rtol=0)):
            raise ValueError("CONTACT_NORMAL_OR_FRAME_MISMATCH")
        local = self.box.rotation.T @ (contact[:3, 3]-self.box.center)
        tangents = [i for i in range(3) if i != self.axis]
        if (abs(local[self.axis]-self.sign*self.box.half_extents[self.axis]) > 1e-12
                or any(abs(local[i]) > self.box.half_extents[i]+1e-12 for i in tangents)):
            raise ValueError("CONTACT_POINT_OUTSIDE_FACE")
        boundary = np.asarray(self.face_boundary)
        if boundary.shape != (4, 3) or not np.isfinite(boundary).all():
            raise ValueError("INVALID_FACE_BOUNDARY")
        expected = []
        for a, b in ((-1,-1),(-1,1),(1,1),(1,-1)):
            expected.append(center + a*self.box.half_extents[tangents[0]]*self.box.rotation[:,tangents[0]]
                + b*self.box.half_extents[tangents[1]]*self.box.rotation[:,tangents[1]])
        if not np.allclose(boundary, expected, atol=1e-12, rtol=0):
            raise ValueError("FACE_BOUNDARY_MISMATCH")
        return self

    def evidence(self):
        return dict(target_id=self.target_id, frame_id=self.frame_id,
            scene_revision=self.scene_revision, source=self.source, face_role=self.face_role,
            geometric_face_id=self.geometric_face_id, legacy_geometric_face=self.legacy_face,
            face_center_m=self.face_center.tolist(), face_boundary_m=self.face_boundary.tolist(),
            outward_normal=self.outward_normal.tolist(), in_plane_x=self.in_plane_x.tolist(),
            contact_pose=self.contact_pose.tolist(), approach_pose=self.approach_pose.tolist(),
            standoff_m=self.standoff_m,
            tool_axes="X=6 rows (short); Y=12 columns (long); Z=inward",
            contact_frame="nominal_compressed_physical_contact")


def simulation_contact_targets(scene, target, robot_base_position, standoff_m):
    """Fixed FRONT/TOP/SIDE order; one face center, no pose/IK/history search.

    Exposure uses the existing conservative face-blocker test in box-local
    coordinates. Unsupported legacy face mappings are explicitly unavailable.
    """
    base = np.asarray(robot_base_position, dtype=float)
    if base.shape != (3,) or not np.isfinite(base).all():
        raise ValueError("INVALID_ROBOT_BASE_POSITION")
    if not np.isfinite(standoff_m) or standoff_m <= 0:
        raise ValueError("INVALID_STANDOFF")
    active = set(scene.support_graph.cartons)
    if target.name not in active:
        raise ValueError("TARGET_NOT_IN_REMAINING_WORLD")
    up_local = target.rotation.T @ np.array([0., 0., 1.])
    top_axis = int(np.argmax(np.abs(up_local)))
    top_sign = 1 if up_local[top_axis] >= 0 else -1
    vertical = target.rotation[:, top_axis] * top_sign
    horizontal = [i for i in range(3) if i != top_axis]
    toward_base = target.rotation.T @ (base-target.center)
    front_axis = max(horizontal, key=lambda i: (abs(toward_base[i]), -i))
    if abs(toward_base[front_axis]) <= 1e-12:
        raise ValueError("FRONT_FACE_DIRECTION_UNDEFINED")
    front_sign = 1 if toward_base[front_axis] > 0 else -1
    side_axis = next(i for i in horizontal if i != front_axis)
    sides = sorted(((side_axis, s) for s in (-1, 1)),
                   key=lambda a: (-a[1]*toward_base[a[0]], f"{'xyz'[a[0]]}{a[1]}"))
    ordered = [("FRONT", front_axis, front_sign), ("TOP", top_axis, top_sign),
               *(("SIDE", axis, sign) for axis, sign in sides)]
    local_boxes = [OBB(target.rotation.T@(b.center-target.center), b.half_extents,
                       target.rotation.T@b.rotation, b.name, b.category)
                   for b in scene.support_graph.cartons.values()]
    local_graph = SupportRelationGraph(local_boxes, [])
    blocked_supports = sorted(scene.support_graph.supports[target.name] & active)
    modes = scene.policy.data["task_population"]["face_modes"]
    targets, screening = [], []
    for role, axis, sign in ordered:
        face_id = f"{'xyz'[axis]}{'+' if sign > 0 else '-'}"
        legacy = GEOMETRIC_FACES.get((axis, sign))
        reason = None
        blockers = []
        if blocked_supports:
            reason = "TARGET_SUPPORTS_REMAINING_CARTONS"
        elif {"FRONT":"front", "TOP":"top", "SIDE":"side"}[role] not in modes:
            reason = "FACE_ROLE_DISABLED"
        elif legacy is None:
            reason = "UNSUPPORTED_GEOMETRIC_FACE_MAPPING"
        else:
            blockers = local_graph.face_blockers(target.name, legacy, active)
            if blockers:
                reason = "FACE_OCCLUDED"
        screening.append(dict(face_role=role, geometric_face_id=face_id,
            available=reason is None, reason=reason, blockers=blockers,
            supported_cartons=blocked_supports))
        if reason:
            continue
        n = target.rotation[:, axis]*sign
        # X is the short 6-row axis. On vertical faces it follows the box's
        # upward edge. On top it follows the inward front edge. Y is Z cross X.
        x = (-target.rotation[:, front_axis]*front_sign if role == "TOP" else vertical)
        z = -n
        y = np.cross(z, x)
        center = target.center+target.half_extents[axis]*n
        contact = np.eye(4); contact[:3,:3] = np.column_stack((x,y,z)); contact[:3,3] = center
        approach = contact.copy(); approach[:3,3] += standoff_m*n
        tangent_axes = [i for i in range(3) if i != axis]
        boundary = np.array([center+a*target.half_extents[tangent_axes[0]]*target.rotation[:,tangent_axes[0]]
                            +b*target.half_extents[tangent_axes[1]]*target.rotation[:,tangent_axes[1]]
                            for a,b in ((-1,-1),(-1,1),(1,1),(1,-1))])
        targets.append(ContactTarget(target.name,"world",scene.snapshot["scene_fingerprint"],
            "SIM_GROUND_TRUTH",role,face_id,target,axis,sign,center,boundary,n,x,
            contact,approach,float(standoff_m)).validate())
    return targets, screening
