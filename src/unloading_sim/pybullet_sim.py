"""PyBullet visualization backend for the unloading geometry demo.

The core simulator remains deterministic and geometry-based.  This module is a
thin optional viewer: it loads the RM65 URDF, mirrors the YAML OBB scene as
PyBullet boxes, and replays an existing joint trajectory CSV.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from unloading_sim.grasp import generate_suction_candidates
    from unloading_sim.geometry import OBB
    from unloading_sim.perception import detect_carton_obbs, fixed_conveyor_place_pose, select_detection
    from unloading_sim.scene import load_scene_config
else:
    from .dynamics import CameraFrame, CameraSpec, TimedJointCommands, tracking_error_audit
    from .grasp import generate_suction_candidates
    from .geometry import OBB
    from .perception import detect_carton_obbs, fixed_conveyor_place_pose, select_detection
    from .scene import load_scene_config

if __package__ in (None, ""):
    from unloading_sim.dynamics import CameraFrame, CameraSpec, TimedJointCommands, tracking_error_audit


ROS_WS = Path("/home/zy0004-lr/下载/code/manipulation-main/ros_ws")
DEFAULT_RM65_URDF = ROS_WS / "src/models/RM65/urdf/RM65-B/urdf/RM65-B.urdf"
DEFAULT_KUKA_KR50_URDF = Path("assets/robots/kuka_kr50_r2500/kr_50_r2500.urdf")
DEFAULT_KUKA_KR50_PACKAGE_ROOT = Path("third_party/kr_50_r2500/kr_50_r2500_description")
DEFAULT_PACKAGE_ROOTS = [
    ROS_WS / "src/models/RM65/urdf/RM65-B",
    ROS_WS / "src/models/Piper/urdf/Piper",
]

BOX_EDGES = [
    (0, 1), (0, 2), (0, 4),
    (1, 3), (1, 5),
    (2, 3), (2, 6),
    (3, 7),
    (4, 5), (4, 6),
    (5, 7), (6, 7),
]


@dataclass
class LoadedModel:
    body_id: int
    joint_indices: list[int]
    joint_names: list[str]
    link_names: dict[str, int]
    temp_dirs: list[tempfile.TemporaryDirectory]


@dataclass
class SimpleTool:
    body_ids: list[int]
    local_offsets: list[tuple[Sequence[float], Sequence[float]]]
    mount_xyz: Sequence[float]
    mount_quat: Sequence[float]


def _import_pybullet():
    try:
        import pybullet as p
    except ImportError as exc:
        raise RuntimeError(
            "PyBullet is required for this viewer. Install with: "
            "conda run -n madpro python -m pip install -e '.[viz]'"
        ) from exc
    return p


def _package_name(package_root: Path) -> str:
    package_xml = package_root / "package.xml"
    if package_xml.exists():
        try:
            root = ET.parse(package_xml).getroot()
            name = root.findtext("name")
            if name:
                return name.strip()
        except ET.ParseError:
            pass
    if package_root.name == "Piper":
        return "piper_description"
    return package_root.name


def _package_root_from_urdf(urdf_path: Path) -> Path:
    for parent in urdf_path.resolve().parents:
        if (parent / "package.xml").exists():
            return parent
    if urdf_path.parent.name == "urdf":
        return urdf_path.parent.parent
    return urdf_path.parent


def _build_package_map(urdf_paths: Iterable[Path], package_roots: Iterable[Path]) -> dict[str, Path]:
    mapping: dict[str, Path] = {}
    for path in urdf_paths:
        if path:
            root = _package_root_from_urdf(Path(path))
            mapping[_package_name(root)] = root
            mapping[root.name] = root
    for root in package_roots:
        root = Path(root).expanduser().resolve()
        if root.exists():
            mapping[_package_name(root)] = root
            mapping[root.name] = root
    return mapping


def resolve_package_uri(uri: str, package_map: dict[str, Path]) -> Path:
    match = re.fullmatch(r"package://([^/]+)/(.+)", uri)
    if not match:
        return Path(uri)
    package, relative = match.groups()
    if package not in package_map:
        known = ", ".join(sorted(package_map))
        raise FileNotFoundError(f"Cannot resolve {uri}; known ROS packages: {known}")
    return package_map[package] / relative


def materialize_urdf(urdf_path: Path, package_map: dict[str, Path]) -> tuple[Path, tempfile.TemporaryDirectory]:
    urdf_path = urdf_path.expanduser().resolve()
    tree = ET.parse(urdf_path)
    for elem in tree.iter():
        filename = elem.attrib.get("filename")
        if filename and filename.startswith("package://"):
            elem.set("filename", str(resolve_package_uri(filename, package_map)))
        elif filename and not Path(filename).is_absolute():
            elem.set("filename", str((urdf_path.parent / filename).resolve()))

    temp_dir = tempfile.TemporaryDirectory(prefix="unloading_pybullet_urdf_")
    out_path = Path(temp_dir.name) / urdf_path.name
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path, temp_dir


def quaternion_from_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    r = np.asarray(rotation, dtype=float)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    idx = int(np.argmax(np.diag(r)))
    if idx == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    if idx == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)


def load_trajectory_csv(path: Path) -> list[np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        joint_columns = [name for name in reader.fieldnames or [] if re.fullmatch(r"q\d+", name)]
        joint_columns.sort(key=lambda name: int(name[1:]))
        if not joint_columns:
            raise ValueError(f"Trajectory CSV has no q1..qN columns: {path}")
        return [np.array([float(row[name]) for name in joint_columns], dtype=float) for row in reader]


def load_timed_joint_commands(path: Path) -> TimedJointCommands:
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        time_columns = [name for name in fields if name in {"time_from_start_s", "time_from_pick_start_s"}]
        joint_columns = sorted(
            (name for name in fields if re.fullmatch(r"q\d+", name)), key=lambda name: int(name[1:])
        )
        if len(time_columns) != 1 or not joint_columns:
            raise ValueError(f"Timed trajectory CSV requires one time column and q1..qN columns: {path}")
        rows = list(reader)
    return TimedJointCommands(
        np.asarray([float(row[time_columns[0]]) for row in rows], dtype=float),
        np.asarray([[float(row[name]) for name in joint_columns] for row in rows], dtype=float),
    )


def replay_with_position_control(
    p,
    robot: LoadedModel,
    commands: TimedJointCommands,
    *,
    physics_dt: float = 1.0 / 240.0,
    max_joint_force: float = 500.0,
    position_gain: float = 0.25,
    velocity_gain: float = 1.0,
    realtime: bool = True,
    simple_tool: SimpleTool | None = None,
    ee_link: int | None = None,
) -> dict:
    """Execute timed commands through PyBullet motors and audit tracking."""
    if physics_dt <= 0.0 or max_joint_force <= 0.0:
        raise ValueError("physics_dt and max_joint_force must be positive")
    if commands.positions.shape[1] != len(robot.joint_indices):
        raise ValueError("timed command DOF does not match the loaded robot")
    p.setTimeStep(float(physics_dt))
    set_robot_joints(p, robot, commands.positions[0])
    commanded_samples: list[np.ndarray] = []
    measured_samples: list[np.ndarray] = []
    sample_times = np.arange(0.0, commands.duration_seconds + 0.5 * physics_dt, physics_dt)
    for timestamp in sample_times:
        target = commands.sample(float(timestamp))
        p.setJointMotorControlArray(
            robot.body_id,
            robot.joint_indices,
            p.POSITION_CONTROL,
            targetPositions=target.tolist(),
            forces=[float(max_joint_force)] * len(robot.joint_indices),
            positionGains=[float(position_gain)] * len(robot.joint_indices),
            velocityGains=[float(velocity_gain)] * len(robot.joint_indices),
        )
        p.stepSimulation()
        measured = np.asarray([state[0] for state in p.getJointStates(robot.body_id, robot.joint_indices)])
        commanded_samples.append(target)
        measured_samples.append(measured)
        if simple_tool is not None and ee_link is not None:
            update_simple_tool(p, robot, ee_link, simple_tool)
        if realtime:
            time.sleep(float(physics_dt))
    return {
        "backend": "pybullet_position_control",
        "physics_dt_seconds": float(physics_dt),
        "max_joint_force": float(max_joint_force),
        **tracking_error_audit(np.asarray(commanded_samples), np.asarray(measured_samples), sample_times),
    }


def add_obb(p, box: OBB, rgba: Sequence[float]) -> int:
    half = box.half_extents.tolist()
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=box.center.tolist(),
        baseOrientation=quaternion_from_matrix(box.rotation),
    )


def set_obb_body_pose(p, body_id: int, box: OBB, center: Sequence[float] | None = None) -> None:
    p.resetBasePositionAndOrientation(
        body_id,
        np.asarray(box.center if center is None else center, dtype=float).tolist(),
        quaternion_from_matrix(box.rotation),
    )


def _quat_from_z_axis(p, direction: np.ndarray) -> tuple[float, float, float, float]:
    direction = np.asarray(direction, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    direction = direction / norm
    z_axis = np.array([0.0, 0.0, 1.0])
    dot = float(np.clip(z_axis @ direction, -1.0, 1.0))
    if dot > 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1e-9:
        return p.getQuaternionFromEuler((np.pi, 0.0, 0.0))
    axis = np.cross(z_axis, direction)
    axis /= np.linalg.norm(axis)
    half_angle = 0.5 * float(np.arccos(dot))
    return (
        float(axis[0] * np.sin(half_angle)),
        float(axis[1] * np.sin(half_angle)),
        float(axis[2] * np.sin(half_angle)),
        float(np.cos(half_angle)),
    )


def add_segment_visual(p, start: np.ndarray, end: np.ndarray, radius: float, rgba: Sequence[float]) -> int | None:
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length < 1e-9:
        return None
    visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=rgba)
    return p.createMultiBody(
        baseMass=0.0,
        baseVisualShapeIndex=visual,
        basePosition=((start + end) * 0.5).tolist(),
        baseOrientation=_quat_from_z_axis(p, delta),
    )


def add_obb_wireframe(p, box: OBB, rgba: Sequence[float], radius: float = 0.006) -> list[int]:
    body_ids: list[int] = []
    corners = box.corners()
    for i, j in BOX_EDGES:
        body_id = add_segment_visual(p, corners[i], corners[j], radius, rgba)
        if body_id is not None:
            body_ids.append(body_id)
    return body_ids


def add_scene_boxes(
    p,
    scene,
    target_name: str | None = None,
    body_groups: dict[str, list[int]] | None = None,
) -> dict[str, int]:
    body_ids: list[int] = []
    named_bodies: dict[str, int] = {}
    colors = {
        "trailer": (0.55, 0.60, 0.66, 0.24),
        "static": (0.25, 0.45, 0.78, 0.55),
        "carton": (0.72, 0.48, 0.25, 0.52),
    }
    for box in scene.obstacles + scene.cartons:
        rgba = colors.get(box.category, (0.7, 0.7, 0.7, 0.45))
        if box.name == target_name:
            rgba = (0.95, 0.58, 0.12, 0.72)
        body_id = add_obb(p, box, rgba)
        body_ids.append(body_id)
        named_bodies[box.name] = body_id
        group = [body_id]
        if box.category == "carton":
            edge_color = (1.0, 0.86, 0.20, 1.0) if box.name == target_name else (0.28, 0.16, 0.06, 1.0)
            edge_radius = 0.008 if box.name == target_name else 0.0045
            wireframe_ids = add_obb_wireframe(p, box, edge_color, radius=edge_radius)
            body_ids.extend(wireframe_ids)
            group.extend(wireframe_ids)
        elif box.category == "static":
            wireframe_ids = add_obb_wireframe(p, box, (0.05, 0.18, 0.42, 1.0), radius=0.006)
            body_ids.extend(wireframe_ids)
            group.extend(wireframe_ids)
        if body_groups is not None:
            body_groups[box.name] = group
    return named_bodies


def load_robot(
    p,
    urdf_path: Path,
    package_roots: Iterable[Path],
    base_position: Sequence[float],
    base_rpy: Sequence[float],
    extra_urdfs: Iterable[Path] = (),
) -> LoadedModel:
    package_map = _build_package_map([urdf_path, *extra_urdfs], package_roots)
    resolved_urdf, temp_dir = materialize_urdf(urdf_path, package_map)
    robot_id = p.loadURDF(
        str(resolved_urdf),
        basePosition=base_position,
        baseOrientation=p.getQuaternionFromEuler(base_rpy),
        useFixedBase=True,
        flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
    )
    joint_indices: list[int] = []
    joint_names: list[str] = []
    link_names: dict[str, int] = {}
    for joint_index in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_index)
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        link_names[link_name] = joint_index
        if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            joint_indices.append(joint_index)
            joint_names.append(joint_name)
    return LoadedModel(robot_id, joint_indices, joint_names, link_names, [temp_dir])


def set_robot_joints(p, model: LoadedModel, q: Sequence[float]) -> None:
    for joint_index, value in zip(model.joint_indices, q):
        p.resetJointState(model.body_id, joint_index, float(value))


def add_debug_axes(p, length: float = 0.35) -> None:
    p.addUserDebugLine((0, 0, 0), (length, 0, 0), (1, 0, 0), 2)
    p.addUserDebugLine((0, 0, 0), (0, length, 0), (0, 0.8, 0), 2)
    p.addUserDebugLine((0, 0, 0), (0, 0, length), (0, 0.2, 1), 2)


def add_perception_markers(p, scene, cfg: dict, max_candidates: int = 60) -> None:
    detections = detect_carton_obbs(scene)
    planning = cfg.get("planning", {})
    target_name = planning.get("target_carton")
    target_detection = select_detection(detections, target_name) if target_name else detections[0]
    robot_base = np.asarray(cfg.get("robot", {}).get("base_position", [0.0, 0.0, 0.0]), dtype=float)
    candidates = generate_suction_candidates(
        target_detection.obb,
        robot_base,
        face_modes=planning.get("grasp_face_modes", ["front", "side", "top"]),
    )
    colors = {
        "front": (0.95, 0.10, 0.08, 1.0),
        "side": (0.10, 0.35, 0.95, 1.0),
        "top": (0.95, 0.80, 0.08, 1.0),
    }
    for candidate in candidates[:max_candidates]:
        visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=0.026,
            rgbaColor=colors.get(candidate.face_mode, (1.0, 1.0, 1.0, 1.0)),
        )
        p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=visual, basePosition=candidate.contact_point.tolist())
        normal_end = candidate.contact_point + candidate.outward_normal * 0.12
        p.addUserDebugLine(candidate.contact_point.tolist(), normal_end.tolist(), colors.get(candidate.face_mode, (1, 1, 1, 1))[:3], 2)

    conveyor_name = planning.get("place_obstacle", "conveyor_deck")
    conveyor = next((obstacle for obstacle in scene.obstacles if obstacle.name == conveyor_name), None)
    if conveyor is not None:
        place_pose = fixed_conveyor_place_pose(conveyor, target_detection.obb)
        visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.045, rgbaColor=(0.0, 0.80, 0.25, 1.0))
        p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=visual, basePosition=place_pose[:3, 3].tolist())


def create_simple_suction_tool(
    p,
    mount_xyz: Sequence[float],
    mount_rpy: Sequence[float],
    cup_radius: float,
) -> SimpleTool:
    mount_quat = p.getQuaternionFromEuler(mount_rpy)
    body_ids: list[int] = []
    offsets: list[tuple[Sequence[float], Sequence[float]]] = []

    specs = [
        (0.030, 0.045, (0.0, 0.0, 0.030), (0.92, 0.92, 0.88, 1.0)),
        (0.018, 0.080, (0.0, 0.0, 0.085), (0.28, 0.30, 0.32, 1.0)),
        (cup_radius, 0.032, (0.0, 0.0, 0.142), (0.03, 0.03, 0.03, 1.0)),
        (cup_radius * 1.08, 0.006, (0.0, 0.0, 0.163), (0.02, 0.02, 0.02, 0.72)),
    ]
    for radius, length, xyz, rgba in specs:
        visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=rgba)
        collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=length)
        body_ids.append(p.createMultiBody(0.0, collision, visual, basePosition=(0, 0, -10)))
        offsets.append((xyz, (0.0, 0.0, 0.0, 1.0)))
    return SimpleTool(body_ids, offsets, mount_xyz, mount_quat)


def update_simple_tool(p, robot: LoadedModel, ee_link: int, tool: SimpleTool) -> None:
    state = p.getLinkState(robot.body_id, ee_link, computeForwardKinematics=True)
    link_pos, link_quat = state[4], state[5]
    mount_pos, mount_quat = p.multiplyTransforms(link_pos, link_quat, tool.mount_xyz, tool.mount_quat)
    for body_id, (local_pos, local_quat) in zip(tool.body_ids, tool.local_offsets):
        pos, quat = p.multiplyTransforms(mount_pos, mount_quat, local_pos, local_quat)
        p.resetBasePositionAndOrientation(body_id, pos, quat)


def add_tool_trace(
    p,
    robot: LoadedModel,
    ee_link: int,
    trajectory: Sequence[np.ndarray],
    tool_offset: float = 0.0,
    sample_count: int = 80,
) -> list[int]:
    if not trajectory:
        return []
    sample_count = min(int(sample_count), len(trajectory))
    sample_indices = np.linspace(0, len(trajectory) - 1, sample_count, dtype=int)
    visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.012, rgbaColor=(0.95, 0.12, 0.08, 0.82))
    body_ids: list[int] = []
    for index in sample_indices:
        set_robot_joints(p, robot, trajectory[int(index)])
        p.stepSimulation()
        state = p.getLinkState(robot.body_id, ee_link, computeForwardKinematics=True)
        position = state[4]
        if tool_offset:
            position, _ = p.multiplyTransforms(
                state[4], state[5], (0.0, 0.0, float(tool_offset)), (0.0, 0.0, 0.0, 1.0)
            )
        body_ids.append(p.createMultiBody(baseMass=0.0, baseVisualShapeIndex=visual, basePosition=position))
    return body_ids


def replay_with_carried_carton(
    p,
    robot: LoadedModel,
    ee_link: int,
    trajectory: Sequence[np.ndarray],
    scene,
    target_name: str,
    carton_body_id: int | None,
    place_body_center: Sequence[float] | None,
    simple_tool: SimpleTool | None,
    dt: float,
    direct: bool,
) -> None:
    if not trajectory:
        return
    target_carton = scene.carton(target_name)
    grasp_index = max(0, int(len(trajectory) * 0.58))
    release_index = max(grasp_index + 1, int(len(trajectory) * 0.94))
    ee_from_carton: tuple[Sequence[float], Sequence[float]] | None = None
    target_center = np.asarray(target_carton.center, dtype=float)
    for index, q in enumerate(trajectory):
        set_robot_joints(p, robot, q)
        p.stepSimulation()
        if simple_tool is not None:
            update_simple_tool(p, robot, ee_link, simple_tool)
        if carton_body_id is not None:
            state = p.getLinkState(robot.body_id, ee_link, computeForwardKinematics=True)
            ee_pos, ee_quat = state[4], state[5]
            if index == grasp_index:
                inv_pos, inv_quat = p.invertTransform(ee_pos, ee_quat)
                ee_from_carton = p.multiplyTransforms(
                    inv_pos,
                    inv_quat,
                    target_carton.center.tolist(),
                    quaternion_from_matrix(target_carton.rotation),
                )
            if ee_from_carton is not None and index < release_index:
                pos, quat = p.multiplyTransforms(ee_pos, ee_quat, ee_from_carton[0], ee_from_carton[1])
                p.resetBasePositionAndOrientation(carton_body_id, pos, quat)
            elif place_body_center is not None and index >= release_index:
                target_center = np.asarray(place_body_center, dtype=float)
                set_obb_body_pose(p, carton_body_id, target_carton, center=target_center)
        if not direct:
            time.sleep(max(dt, 0.0))


def save_camera_snapshot(
    p,
    path: str | Path,
    width: int,
    height: int,
    camera_target: Sequence[float] = (0.20, 0.0, 1.05),
    distance: float = 4.15,
    camera_eye: Sequence[float] | None = None,
) -> None:
    from PIL import Image

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if camera_eye is None:
        view = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=camera_target,
            distance=distance,
            yaw=-50.0,
            pitch=-24.0,
            roll=0.0,
            upAxisIndex=2,
        )
    else:
        view = p.computeViewMatrix(cameraEyePosition=camera_eye, cameraTargetPosition=camera_target, cameraUpVector=(0.0, 0.0, 1.0))
    projection = p.computeProjectionMatrixFOV(
        fov=52.0,
        aspect=float(width) / float(height),
        nearVal=0.03,
        farVal=8.0,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
    )
    image = np.asarray(rgba, dtype=np.uint8).reshape((height, width, 4))
    Image.fromarray(image, mode="RGBA").save(path)


def capture_rgbd_frame(
    p,
    spec: CameraSpec,
    *,
    timestamp_seconds: float,
    camera_eye: Sequence[float],
    camera_target: Sequence[float],
    camera_up: Sequence[float] = (0.0, 0.0, 1.0),
) -> CameraFrame:
    """Capture metric RGB-D and instance segmentation from a calibrated camera."""
    view = p.computeViewMatrix(camera_eye, camera_target, camera_up)
    aspect = float(spec.width) / float(spec.height)
    vertical_fov = 2.0 * np.arctan(np.tan(0.5 * spec.horizontal_fov_rad) / aspect)
    projection = p.computeProjectionMatrixFOV(
        fov=float(np.degrees(vertical_fov)),
        aspect=aspect,
        nearVal=spec.near_m,
        farVal=spec.far_m,
    )
    _, _, rgba, depth_buffer, segmentation = p.getCameraImage(
        spec.width,
        spec.height,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER,
    )
    depth_buffer = np.asarray(depth_buffer, dtype=float).reshape(spec.height, spec.width)
    depth_m = spec.far_m * spec.near_m / (
        spec.far_m - (spec.far_m - spec.near_m) * depth_buffer
    )
    return CameraFrame(
        float(timestamp_seconds),
        np.asarray(rgba, dtype=np.uint8).reshape(spec.height, spec.width, 4),
        depth_m,
        np.asarray(segmentation, dtype=np.int32).reshape(spec.height, spec.width),
        spec.intrinsic_matrix,
    )


def attach_gripper_urdf(
    p,
    robot: LoadedModel,
    ee_link: int,
    gripper_urdf: Path,
    package_roots: Iterable[Path],
    mount_xyz: Sequence[float],
    mount_rpy: Sequence[float],
) -> tuple[int, tempfile.TemporaryDirectory]:
    package_map = _build_package_map([gripper_urdf], package_roots)
    resolved_urdf, temp_dir = materialize_urdf(gripper_urdf, package_map)
    gripper_id = p.loadURDF(str(resolved_urdf), useFixedBase=False)
    p.createConstraint(
        parentBodyUniqueId=robot.body_id,
        parentLinkIndex=ee_link,
        childBodyUniqueId=gripper_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=(0, 0, 0),
        parentFramePosition=mount_xyz,
        parentFrameOrientation=p.getQuaternionFromEuler(mount_rpy),
        childFramePosition=(0, 0, 0),
        childFrameOrientation=(0, 0, 0, 1),
    )
    return gripper_id, temp_dir


def run_viewer(args: argparse.Namespace) -> None:
    if not args.direct and args.software_gl:
        os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "mesa")
        os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    p = _import_pybullet()
    mode = p.DIRECT if args.direct else p.GUI
    client = p.connect(mode)
    if client < 0:
        raise RuntimeError("Failed to connect to PyBullet")
    try:
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        scene, cfg = load_scene_config(args.config)
        planning_cfg = cfg.get("planning", {})
        scene_bodies = add_scene_boxes(p, scene, target_name=planning_cfg.get("target_carton"))
        add_debug_axes(p)
        if args.show_perception:
            add_perception_markers(p, scene, cfg)

        robot_cfg = cfg["robot"]
        base_position = args.base_position or robot_cfg.get("base_position", [0.0, 0.0, 0.0])
        base_rpy = args.base_rpy or robot_cfg.get("base_rpy", [0.0, 0.0, 0.0])
        package_roots = [Path(path) for path in args.package_root]
        gripper_urdf = Path(args.gripper_urdf).expanduser() if args.gripper_urdf else None
        extra_urdfs = [gripper_urdf] if gripper_urdf else []
        robot = load_robot(p, Path(args.robot_urdf), package_roots, base_position, base_rpy, extra_urdfs)

        if args.end_effector_link not in robot.link_names:
            known = ", ".join(sorted(robot.link_names))
            raise KeyError(f"Unknown end-effector link {args.end_effector_link!r}; known links: {known}")
        ee_link = robot.link_names[args.end_effector_link]

        attached_temp: tempfile.TemporaryDirectory | None = None
        simple_tool: SimpleTool | None = None
        if gripper_urdf:
            _, attached_temp = attach_gripper_urdf(
                p, robot, ee_link, gripper_urdf, package_roots, args.gripper_xyz, args.gripper_rpy
            )
        elif args.tool == "suction" and not args.no_gripper:
            simple_tool = create_simple_suction_tool(p, args.gripper_xyz, args.gripper_rpy, args.suction_radius)

        timed_commands = None
        if args.trajectory and Path(args.trajectory).exists():
            if args.dynamic:
                timed_commands = load_timed_joint_commands(Path(args.trajectory))
                trajectory = [row.copy() for row in timed_commands.positions]
            else:
                trajectory = load_trajectory_csv(Path(args.trajectory))
        else:
            trajectory = [np.asarray(robot_cfg.get("home_joints", [0.0] * len(robot.joint_indices)), dtype=float)]

        add_tool_trace(p, robot, ee_link, trajectory)
        p.resetDebugVisualizerCamera(
            cameraDistance=3.2,
            cameraYaw=-42.0,
            cameraPitch=-25.0,
            cameraTargetPosition=(0.45, 0.0, 0.8),
        )
        place_center = None
        target_name = planning_cfg.get("target_carton")
        conveyor_name = planning_cfg.get("place_obstacle", "conveyor_deck")
        if target_name:
            conveyor = next((obstacle for obstacle in scene.obstacles if obstacle.name == conveyor_name), None)
            if conveyor is not None:
                place_pose = fixed_conveyor_place_pose(conveyor, scene.carton(target_name))
                place_center = place_pose[:3, 3].copy()
                place_center[2] = conveyor.center[2] + conveyor.half_extents[2] + scene.carton(target_name).half_extents[2]
        for _ in range(max(args.loops, 1)):
            if timed_commands is not None:
                dynamics_audit = replay_with_position_control(
                    p,
                    robot,
                    timed_commands,
                    physics_dt=args.physics_dt,
                    max_joint_force=args.max_joint_force,
                    position_gain=args.position_gain,
                    velocity_gain=args.velocity_gain,
                    realtime=not args.direct,
                    simple_tool=simple_tool,
                    ee_link=ee_link,
                )
                print(json.dumps(dynamics_audit, ensure_ascii=False, indent=2))
            else:
                replay_with_carried_carton(
                    p,
                    robot,
                    ee_link,
                    trajectory,
                    scene,
                    target_name,
                    scene_bodies.get(target_name) if target_name else None,
                    place_center,
                    simple_tool,
                    args.dt,
                    args.direct,
                )
        if args.snapshot:
            save_camera_snapshot(p, args.snapshot, args.width, args.height)
        if args.hold and not args.direct:
            while True:
                p.stepSimulation()
                time.sleep(1.0 / 60.0)
        if attached_temp is not None:
            attached_temp.cleanup()
    finally:
        p.disconnect()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize the unloading scene in PyBullet")
    parser.add_argument("--config", default="config/demo.yaml", help="YAML scene configuration")
    parser.add_argument("--trajectory", default="outputs/demo/trajectory.csv", help="Joint trajectory CSV from demo.py")
    parser.add_argument("--robot-urdf", default=str(DEFAULT_RM65_URDF), help="RM65 URDF path")
    parser.add_argument("--gripper-urdf", default=None, help="Optional Inspire two-finger gripper URDF path")
    parser.add_argument("--package-root", action="append", default=[str(path) for path in DEFAULT_PACKAGE_ROOTS])
    parser.add_argument("--end-effector-link", default="link_6", help="RM65 link used to mount the gripper")
    parser.add_argument("--base-position", nargs=3, type=float, default=None)
    parser.add_argument("--base-rpy", nargs=3, type=float, default=None)
    parser.add_argument("--gripper-xyz", nargs=3, type=float, default=[0.0, 0.0, 0.035])
    parser.add_argument("--gripper-rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--gripper-opening", type=float, default=0.055, help="Opening for the simple EG2-style gripper")
    parser.add_argument("--tool", choices=["suction", "none"], default="suction", help="Simple visual tool to attach")
    parser.add_argument("--suction-radius", type=float, default=0.045, help="Radius of the simple suction cup")
    parser.add_argument("--loops", type=int, default=1)
    parser.add_argument("--dt", type=float, default=1.0 / 60.0)
    parser.add_argument("--dynamic", action="store_true", help="Use timed CSV commands and motor-driven physics")
    parser.add_argument("--physics-dt", type=float, default=1.0 / 240.0)
    parser.add_argument("--max-joint-force", type=float, default=500.0)
    parser.add_argument("--position-gain", type=float, default=0.25)
    parser.add_argument("--velocity-gain", type=float, default=1.0)
    parser.add_argument("--direct", action="store_true", help="Run without opening a GUI window")
    parser.add_argument("--native-gl", dest="software_gl", action="store_false", help="Do not force Mesa software GL for GUI")
    parser.add_argument("--snapshot", default=None, help="Write an offscreen PNG snapshot and exit")
    parser.add_argument("--show-perception", action="store_true", default=True, help="Show detected OBB grasp candidates and place marker")
    parser.add_argument("--hide-perception", dest="show_perception", action="store_false", help="Hide perception markers")
    parser.add_argument("--width", type=int, default=1280, help="Snapshot width in pixels")
    parser.add_argument("--height", type=int, default=800, help="Snapshot height in pixels")
    parser.add_argument("--hold", action="store_true", help="Keep the GUI open after playback")
    parser.add_argument("--no-gripper", action="store_true", help="Do not add the fallback simple gripper")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    run_viewer(parse_args(argv))


if __name__ == "__main__":
    main()
