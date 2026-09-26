"""Export the existing authority's frozen geometry, without importing Tesseract."""
from __future__ import annotations

import hashlib
from itertools import combinations
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import OBB
from .planning_contract import fingerprint, subdivision_rule, POINT_MOTION_BOUND_M, LEVER_ARM_M


def _numbers(values):
    return " ".join(format(float(v), ".17g") for v in values)


def _origin(parent, transform):
    # URDF fixed-axis Rz(yaw) Ry(pitch) Rx(roll).
    r = np.asarray(transform)[:3, :3]
    pitch = np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))
    if abs(np.cos(pitch)) > 1e-10:
        roll, yaw = np.arctan2(r[2, 1], r[2, 2]), np.arctan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = 0., np.arctan2(-r[0, 1], r[1, 1])
    old = parent.find("origin")
    if old is not None:
        parent.remove(old)
    ET.SubElement(parent, "origin", xyz=_numbers(transform[:3, 3]), rpy=_numbers([roll, pitch, yaw]))


def export_scene(connector, obstacles, *, stage, attachment=None, support_names=(), target_contact=None):
    """Only explicit free pregrasp/transit domains; process contacts fail closed.

    Each audited tool OBB is a separate fixed link, preserving collider-level
    permissions. The payload's contact relationship is constant within a request.
    """
    if stage not in {"pregrasp", "transit"} or support_names or target_contact is not None:
        raise ValueError("UNSUPPORTED_CONSTRAINT: process contact or departure domain")
    if (stage == "pregrasp" and attachment is not None) or (stage == "transit" and attachment is None):
        raise ValueError("UNSUPPORTED_CONSTRAINT: attachment does not match free stage")
    validator = connector.robot_state_validator
    if not connector.execution_qualified or not hasattr(validator, "mesh_robot"):
        raise ValueError("UNSUPPORTED_CONSTRAINT: exact audited model required")
    policy = connector.collision_policy
    if not policy.poc_pair_clearance:
        raise ValueError("UNSUPPORTED_CONSTRAINT: only explicit uninflated POC pair distances")
    robot = validator.tool_transform_robot
    urdf_path = Path(validator.mesh_robot.urdf_path)
    srdf_path = urdf_path.parents[2] / "m710id_70_official.srdf"
    root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    root.set("xmlns:tesseract", "https://tesseract-robotics.github.io")
    root.set("tesseract:make_convex", "false")
    srdf = ET.fromstring(srdf_path.read_text(encoding="utf-8"))
    original_links = {item.attrib["name"] for item in root.findall("link")}
    collision_links = [item.attrib["name"] for item in root.findall("link") if item.find("collision") is not None]
    files = {str(urdf_path): hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
             str(srdf_path): hashlib.sha256(srdf_path.read_bytes()).hexdigest()}
    # Resolve every original visual/collision asset; never silently discard one.
    package_root = urdf_path.parents[1]
    for mesh in root.iter("mesh"):
        uri = mesh.attrib["filename"]
        if not uri.startswith("package://fanuc_m710_description/"):
            raise ValueError(f"unsupported mesh URI: {uri}")
        path = package_root / uri.split("package://fanuc_m710_description/", 1)[1]
        files[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        mesh.set("filename", str(path.resolve()))
    base_joint = root.find("joint[@name='base_joint']")
    if base_joint is None:
        raise ValueError("official base joint missing")
    _origin(base_joint, np.asarray(robot.base_transform))
    names = set(original_links)
    boxes = []

    def add_link(name, parent, transform, half=None, category=None):
        if name in names:
            raise ValueError(f"duplicate collider/link: {name}")
        names.add(name)
        link = ET.SubElement(root, "link", name=name)
        if half is not None:
            geom = ET.SubElement(ET.SubElement(link, "collision"), "geometry")
            ET.SubElement(geom, "box", size=_numbers(2 * np.asarray(half)))
            boxes.append(dict(name=name, parent=parent, transform=np.asarray(transform).tolist(),
                              half_extents=np.asarray(half).tolist(), category=category))
        joint = ET.SubElement(root, "joint", name="backend_fixed_" + name, type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name)
        _origin(joint, np.asarray(transform))

    add_link("backend_tcp", "tool0", np.asarray(robot.tip_from_tcp))
    zero = np.zeros(robot.dof)
    tcp = robot.fk(zero)
    tool = list(robot.tool_collision_obbs(zero)) + list(validator._compliant_boxes(zero))
    if {b.name for b in tool} != set(validator.required_tool_names):
        raise ValueError("incomplete audited tool compound")
    compliant = {b.name for b in validator._compliant_boxes(zero)}
    for box in tool:
        add_link(box.name, "backend_tcp", np.linalg.inv(tcp) @ box.world_from_local,
                 box.half_extents, "compliant" if box.name in compliant else "rigid_tool")
    permitted = []

    def allow(a, b, reason):
        permitted.append([a, b, reason])
        ET.SubElement(srdf, "disable_collisions", link1=a, link2=b, reason=reason)

    # The common authority regards the assembled tool as one rigid compound.
    for a, b in combinations(tool, 2):
        allow(a.name, b.name, "Same audited rigid assembly")
    for a, b in sorted(policy.wrist_tool_pairs([b.name for b in tool])):
        allow(a, b, "Physical ownership simulation policy")
    target = attachment.rigid.target_name if attachment is not None and hasattr(attachment.rigid, "target_name") else None
    payload = None
    if attachment is not None:
        payload = attachment.box_at(zero)
        target = payload.name
        add_link(payload.name, "backend_tcp", np.linalg.inv(tcp) @ payload.world_from_local,
                 payload.half_extents, "payload")
        # Cup/payload compression is invariant under their common rigid motion.
        # Prove precisely the authority's SAT predicate once, without dropping it.
        for cup in tool:
            if cup.name in compliant and cup.signed_distance_obb(payload) >= -policy.maximum_compliant_cup_additional_compression_m:
                allow(cup.name, payload.name, "Fixed attachment satisfies common cup compression predicate")
    else:
        target = validator.contact_target_name
    obstacle_names = [box.name for box in obstacles]
    if len(set(obstacle_names)) != len(obstacle_names):
        raise ValueError("duplicate frozen obstacle")
    if payload is not None and payload.name in obstacle_names:
        raise ValueError("attached target still present in frozen world obstacles")
    for box in obstacles:
        add_link(box.name, "world", box.world_from_local, box.half_extents, box.category)
        if box.name == validator.base_support_obstacle_name:
            allow("base_link", box.name, "Common base mounting pair")
        if policy.compliant_cup_neighbor_contact_mode == "ignore" and target is not None and box.name != target and box.name in validator.stack_carton_names:
            for name in sorted(compliant):
                allow(name, box.name, "Named non-target stack flexible cup policy")
    # The floor plane is represented over a proven enclosing workspace, not by
    # the finite trailer floor alone. A 1 km enclosure exceeds this fixed arm's
    # sum of joint offsets and every exported attached-body radius by >100x.
    reach_bound = sum(np.linalg.norm(np.fromstring(j.find("origin").get("xyz", "0 0 0"), sep=" "))
                      for j in root.findall("joint") if j.get("type") != "fixed" and j.find("origin") is not None)
    if reach_bound > 10 or np.linalg.norm(robot.base_transform[:3, 3]) > 100 or any(
            np.linalg.norm(b["transform"]) + np.linalg.norm(b["half_extents"]) > 100
            for b in boxes if b["parent"] == "backend_tcp"):
        raise ValueError("floor enclosure proof failed")
    floor = np.eye(4); floor[2, 3] = validator.floor_z_m - 500
    add_link("backend_floor_plane", "world", floor, [500., 500., 500.], "floor")
    gap = policy.pair_clearance("external", connector.collision_margin_m)
    margins = [[a, b, policy.pair_clearance("robot_self", connector.collision_margin_m)]
               for a, b in combinations(collision_links, 2)]
    if payload is not None:
        for box in obstacles:
            if box.category == "conveyor":
                margins.append([payload.name, box.name, gap + connector.budget.receiver_runtime_clearance_reserve_m])
    joints = []
    for name in robot.active_joint_names:
        j = root.find(f"joint[@name='{name}']")
        joints.append(dict(name=name, type=j.get("type"), link=j.find("child").get("link"),
                           axis=np.fromstring(j.find("axis").get("xyz"), sep=" ").tolist()))
    scene = dict(schema="tesseract_frozen_scene_v1", urdf=ET.tostring(root, encoding="unicode"),
                 srdf=ET.tostring(srdf, encoding="unicode"), mesh_files=files,
                 joint_names=list(robot.active_joint_names), joint_limits=np.asarray(robot.joint_limits).tolist(),
                 joints=joints, active_links=collision_links + [b.name for b in tool] + ([] if payload is None else [payload.name]),
                 collision_objects=collision_links + [b["name"] for b in boxes],
                 default_margin=gap, pair_margins=margins, boxes=boxes, permitted_pairs=permitted,
                 constraints=dict(motion="free_joint_space", events=[], joint_margin=connector.joint_margin_rad,
                     maximum_jacobian_condition=connector.maximum_jacobian_condition,
                     radial_limit=None if connector.official_radial_reach_m is None else connector.official_radial_reach_m + connector.radial_guard_tolerance_m,
                     edge_resolution_rad=connector.budget.edge_resolution_rad,
                     point_motion_bound_m=POINT_MOTION_BOUND_M, lever_arm_m=LEVER_ARM_M),
                 frames=dict(world="+X into trailer,+Y left,+Z up; SI m,rad,s", base="base_link", flange="flange",
                     tcp="backend_tcp", base_transform=np.asarray(robot.base_transform).tolist(),
                     tip_from_tcp=np.asarray(robot.tip_from_tcp).tolist()),
                 attachment=None if payload is None else boxes[[b["name"] for b in boxes].index(payload.name)],
                 model_fingerprint=fingerprint(files), tool_fingerprint=fingerprint([b for b in boxes if b["category"] in {"rigid_tool", "compliant"}]),
                 policy_fingerprint=policy.fingerprint, stage=stage)
    subdivision_rule(scene["constraints"])
    scene["fingerprint"] = fingerprint(scene)
    return scene
