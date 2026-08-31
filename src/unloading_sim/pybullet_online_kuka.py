"""Continuous PyBullet replay for online KUKA unloading plans."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from unloading_sim.pybullet_kuka_kr50 import main as _unused_main
    from unloading_sim.pybullet_sim import (
        DEFAULT_KUKA_KR50_PACKAGE_ROOT,
        DEFAULT_KUKA_KR50_URDF,
        _import_pybullet,
        add_obb,
        add_debug_axes,
        add_perception_markers,
        add_tool_trace,
        add_scene_boxes,
        create_simple_suction_tool,
        load_robot,
        save_camera_snapshot,
        set_obb_body_pose,
        set_robot_joints,
        update_simple_tool,
        quaternion_from_matrix,
    )
    from unloading_sim.geometry import OBB
    from unloading_sim.scene import load_scene_config
else:
    from .pybullet_sim import (
        DEFAULT_KUKA_KR50_PACKAGE_ROOT,
        DEFAULT_KUKA_KR50_URDF,
        _import_pybullet,
        add_obb,
        add_debug_axes,
        add_perception_markers,
        add_tool_trace,
        add_scene_boxes,
        create_simple_suction_tool,
        load_robot,
        save_camera_snapshot,
        set_obb_body_pose,
        set_robot_joints,
        update_simple_tool,
        quaternion_from_matrix,
    )
    from .geometry import OBB
    from .scene import load_scene_config


def suction_replay_mount(robot_cfg: dict, link_names: dict[str, int]) -> tuple[str, float]:
    """Return the replay link and Z offset whose cup face matches the tool tip."""
    configured_tip = str(robot_cfg.get("tip_link", "flange"))
    ee_link_name = configured_tip if configured_tip in link_names else "flange"
    return ee_link_name, float(robot_cfg.get("tool_length", 0.20)) - 0.166


def remove_conveyed_carton(
    p,
    scene_bodies: dict[str, int],
    target: str,
    body_groups: dict[str, list[int]] | None = None,
) -> None:
    """Remove a released carton once the conveyor takes it out of the cell."""
    body_id = scene_bodies.pop(target)
    ids = [body_id] if body_groups is None else body_groups.pop(target, [body_id])
    for removable_id in ids:
        p.removeBody(removable_id)


def replay_online(args: argparse.Namespace) -> None:
    if not args.direct and args.software_gl:
        os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "mesa")
        os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
    p = _import_pybullet()
    mode = p.DIRECT if args.direct else p.GUI
    client = p.connect(mode)
    if client < 0:
        raise RuntimeError("Failed to connect to PyBullet")
    try:
        with Path(args.plan).open("r", encoding="utf-8") as f:
            plan = json.load(f)
        scene, cfg = load_scene_config(plan["config"])
        planning_cfg = cfg.get("planning", {})
        robot_cfg = cfg["robot"]
        amr_cfg = cfg.get("amr", {})
        default_dock = amr_cfg.get("dock_positions", [robot_cfg.get("base_position", [-1.15, 0.0, 0.0])])[0]
        conveyor_name = amr_cfg.get("conveyor_name", "conveyor_deck")
        mounted_centers = dict(amr_cfg.get("mounted_surface_centers", {}))
        mounted_centers.setdefault(conveyor_name, amr_cfg.get("conveyor_mount_center", [0.0, 0.0, 0.0]))
        for obstacle_index, obstacle in enumerate(scene.obstacles):
            if obstacle.name in mounted_centers:
                scene.obstacles[obstacle_index] = OBB(
                    center=np.asarray(default_dock, dtype=float)
                    + np.asarray(mounted_centers[obstacle.name], dtype=float),
                    half_extents=obstacle.half_extents,
                    rotation=obstacle.rotation,
                    name=obstacle.name,
                    category=obstacle.category,
                )
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        scene_body_groups: dict[str, list[int]] = {}
        scene_bodies = add_scene_boxes(
            p,
            scene,
            target_name=planning_cfg.get("target_carton"),
            body_groups=scene_body_groups,
        )
        add_debug_axes(p)
        if args.show_perception:
            add_perception_markers(p, scene, cfg)

        platform_size = np.asarray(amr_cfg.get("footprint_size", [1.60, 1.20, 0.30]), dtype=float)
        platform_offset = np.asarray(amr_cfg.get("platform_center_offset", [0.0, 0.0, platform_size[2] / 2.0]), dtype=float)
        amr_platform = OBB(
            center=np.asarray(default_dock, dtype=float) + platform_offset,
            half_extents=platform_size / 2.0,
            rotation=np.eye(3),
            name="amr_platform",
            category="amr",
        )
        amr_body = add_obb(p, amr_platform, (0.12, 0.14, 0.18, 1.0))
        robot_mount = np.asarray(amr_cfg.get("robot_mount_position", [0.0, 0.0, 0.0]), dtype=float)
        default_robot_base = np.asarray(default_dock, dtype=float) + robot_mount

        def load_robot_at(base_position: np.ndarray):
            robot_urdf = Path(robot_cfg.get("urdf_path", DEFAULT_KUKA_KR50_URDF))
            package_roots = [robot_urdf.parent, Path(DEFAULT_KUKA_KR50_PACKAGE_ROOT)]
            model = load_robot(
                p,
                robot_urdf,
                package_roots,
                base_position.tolist(),
                robot_cfg.get("base_rpy", [0.0, 0.0, 0.0]),
            )
            ee_link_name, suction_mount_z = suction_replay_mount(robot_cfg, model.link_names)
            # The last suction-disc face coincides with the kinematic tool tip.
            # Mounting it on FANUC's flange used the flange X axis and made the
            # cylinder side appear to contact the carton.
            return model, model.link_names[ee_link_name], create_simple_suction_tool(
                p, [0.0, 0.0, suction_mount_z], [0.0, 0.0, 0.0], 0.065
            )

        robot, ee_link, tool = load_robot_at(default_robot_base)
        loaded_robot_base = default_robot_base.copy()
        display_q = robot_cfg.get("home_joints", [0.0] * len(robot.joint_indices))
        set_robot_joints(p, robot, display_q)
        update_simple_tool(p, robot, ee_link, tool)
        for surface_name, mount_center in mounted_centers.items():
            if surface_name in scene_bodies:
                surface = next(obstacle for obstacle in scene.obstacles if obstacle.name == surface_name)
                mounted_center = np.asarray(default_dock, dtype=float) + np.asarray(mount_center, dtype=float)
                set_obb_body_pose(p, scene_bodies[surface_name], surface, center=mounted_center)

        replay_camera_target = tuple(args.camera_target)
        replay_camera_distance = float(np.linalg.norm(np.asarray(args.camera_eye) - np.asarray(args.camera_target)))
        replay_camera_eye = tuple(args.camera_eye)
        p.resetDebugVisualizerCamera(replay_camera_distance, 35.0, -32.0, replay_camera_target)
        gif_frames = []
        frame_index = 0

        def capture_frame() -> None:
            nonlocal frame_index
            if args.frames_dir and frame_index % args.frame_stride == 0:
                save_camera_snapshot(
                    p,
                    Path(args.frames_dir) / f"frame_{frame_index:05d}.png",
                    args.width,
                    args.height,
                    replay_camera_target,
                    replay_camera_distance,
                    replay_camera_eye,
                )
            if args.gif and frame_index % args.frame_stride == 0:
                from PIL import Image

                view = p.computeViewMatrix(
                    cameraEyePosition=replay_camera_eye,
                    cameraTargetPosition=replay_camera_target,
                    cameraUpVector=(0.0, 0.0, 1.0),
                )
                projection = p.computeProjectionMatrixFOV(
                    fov=52.0,
                    aspect=float(args.width) / float(args.height),
                    nearVal=0.03,
                    farVal=8.0,
                )
                _, _, rgba, _, _ = p.getCameraImage(
                    width=args.width,
                    height=args.height,
                    viewMatrix=view,
                    projectionMatrix=projection,
                    renderer=p.ER_TINY_RENDERER,
                )
                image = np.asarray(rgba, dtype=np.uint8).reshape((args.height, args.width, 4))
                gif_frames.append(Image.fromarray(image, mode="RGBA").convert("P", palette=Image.ADAPTIVE))
            frame_index += 1
        for segment in plan["segments"]:
            target = segment["target"]
            carton = scene.carton(target)
            body_id = scene_bodies[target]
            dock_position = segment.get("amr_dock_position")
            amr_cfg = cfg.get("amr", {})
            conveyor_name = amr_cfg.get("conveyor_name", "conveyor_deck")
            if dock_position is not None:
                desired_robot_base = np.asarray(dock_position, dtype=float) + robot_mount
                if not np.allclose(desired_robot_base, loaded_robot_base):
                    for body_id_to_remove in tool.body_ids:
                        p.removeBody(body_id_to_remove)
                    p.removeBody(robot.body_id)
                    for temp_dir in robot.temp_dirs:
                        temp_dir.cleanup()
                    robot, ee_link, tool = load_robot_at(desired_robot_base)
                    loaded_robot_base = desired_robot_base.copy()
                for surface_name, mount_center in mounted_centers.items():
                    if surface_name in scene_bodies:
                        surface = next(obstacle for obstacle in scene.obstacles if obstacle.name == surface_name)
                        mounted_center = np.asarray(dock_position, dtype=float) + np.asarray(mount_center, dtype=float)
                        set_obb_body_pose(p, scene_bodies[surface_name], surface, center=mounted_center)
                set_obb_body_pose(
                    p,
                    amr_body,
                    amr_platform,
                    center=np.asarray(dock_position, dtype=float) + platform_offset,
                )
            path = [np.asarray(q, dtype=float) for q in segment["path"]]
            trace_ids = add_tool_trace(
                p,
                robot,
                ee_link,
                path,
                tool_offset=float(robot_cfg.get("tool_length", 0.20)),
                sample_count=args.trace_samples,
            ) if args.show_trajectory else []
            grasp_index = int(segment.get("grasp_index", max(0, int(len(path) * 0.58))))
            release_index = int(segment.get("release_index", max(grasp_index + 1, int(len(path) * 0.94))))
            ee_from_carton = None
            place_center = segment.get("place_center")
            release_center = segment.get("release_center", place_center)
            place_rotation = segment.get("place_rotation")
            for local_index, q in enumerate(path):
                set_robot_joints(p, robot, q)
                p.stepSimulation()
                update_simple_tool(p, robot, ee_link, tool)
                state = p.getLinkState(robot.body_id, ee_link, computeForwardKinematics=True)
                ee_pos, ee_quat = state[4], state[5]
                if local_index == grasp_index:
                    inv_pos, inv_quat = p.invertTransform(ee_pos, ee_quat)
                    ee_from_carton = p.multiplyTransforms(
                        inv_pos,
                        inv_quat,
                        carton.center.tolist(),
                        quaternion_from_matrix(carton.rotation),
                    )
                if ee_from_carton is not None and local_index < release_index:
                    pos, quat = p.multiplyTransforms(ee_pos, ee_quat, ee_from_carton[0], ee_from_carton[1])
                    p.resetBasePositionAndOrientation(body_id, pos, quat)
                elif release_center is not None and local_index >= release_index:
                    if place_rotation is None:
                        set_obb_body_pose(p, body_id, carton, center=release_center)
                    else:
                        p.resetBasePositionAndOrientation(
                            body_id,
                            release_center,
                            quaternion_from_matrix(np.asarray(place_rotation, dtype=float)),
                        )
                capture_frame()
                if not args.direct:
                    time.sleep(args.dt)
            if place_center is not None and release_center is not None:
                release = np.asarray(release_center, dtype=float)
                settled = np.asarray(place_center, dtype=float)
                drop_distance = float(np.linalg.norm(release - settled))
                drop_duration = np.sqrt(2.0 * drop_distance / 9.81) if drop_distance > 0.0 else 0.0
                drop_steps = max(1, int(np.ceil(drop_duration / max(args.dt, 1e-6))))
                for drop_index in range(1, drop_steps + 1):
                    elapsed = min(drop_duration, drop_index * drop_duration / drop_steps)
                    fraction = 1.0 if drop_duration == 0.0 else min(1.0, 0.5 * 9.81 * elapsed * elapsed / drop_distance)
                    center = (1.0 - fraction) * release + fraction * settled
                    p.resetBasePositionAndOrientation(
                        body_id,
                        center.tolist(),
                        quaternion_from_matrix(np.asarray(place_rotation, dtype=float)) if place_rotation is not None else quaternion_from_matrix(carton.rotation),
                    )
                    p.stepSimulation()
                    capture_frame()
            # A downstream conveyor owns the carton after it lands. Removing
            # the rigid body here models transport out of this robot cell and
            # prevents already placed cartons accumulating on the belt.
            remove_conveyed_carton(p, scene_bodies, target, scene_body_groups)
            capture_frame()
            if target in [carton.name for carton in scene.cartons]:
                scene.cartons = [carton for carton in scene.cartons if carton.name != target]
            for trace_id in trace_ids:
                p.removeBody(trace_id)
        if args.snapshot:
            save_camera_snapshot(
                p,
                args.snapshot,
                args.width,
                args.height,
                replay_camera_target,
                replay_camera_distance,
                replay_camera_eye,
            )
        if args.gif and not gif_frames:
            from PIL import Image

            view = p.computeViewMatrix(
                cameraEyePosition=replay_camera_eye,
                cameraTargetPosition=replay_camera_target,
                cameraUpVector=(0.0, 0.0, 1.0),
            )
            projection = p.computeProjectionMatrixFOV(
                fov=52.0,
                aspect=float(args.width) / float(args.height),
                nearVal=0.03,
                farVal=8.0,
            )
            _, _, rgba, _, _ = p.getCameraImage(
                width=args.width,
                height=args.height,
                viewMatrix=view,
                projectionMatrix=projection,
                renderer=p.ER_TINY_RENDERER,
            )
            image = np.asarray(rgba, dtype=np.uint8).reshape((args.height, args.width, 4))
            gif_frames = [Image.fromarray(image, mode="RGBA").convert("P", palette=Image.ADAPTIVE)]
        if args.gif and gif_frames:
            gif_path = Path(args.gif)
            gif_path.parent.mkdir(parents=True, exist_ok=True)
            gif_frames[0].save(
                gif_path,
                save_all=True,
                append_images=gif_frames[1:],
                duration=max(20, int(args.dt * 1000 * args.frame_stride)),
                loop=0,
            )
        if args.hold and not args.direct:
            while True:
                p.stepSimulation()
                time.sleep(1.0 / 60.0)
    finally:
        p.disconnect()


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Replay online KUKA unloading in PyBullet")
    parser.add_argument("--plan", default="outputs/kuka_kr50/online_plan.json")
    parser.add_argument("--direct", action="store_true")
    parser.add_argument("--snapshot", default=None)
    parser.add_argument("--gif", default=None)
    parser.add_argument("--frames-dir", default=None)
    parser.add_argument("--frame-stride", type=int, default=10)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=500)
    parser.add_argument("--camera-eye", nargs=3, type=float, default=[-3.0, -0.55, 3.5])
    parser.add_argument("--camera-target", nargs=3, type=float, default=[1.15, 0.05, 1.0])
    parser.add_argument("--trace-samples", type=int, default=24)
    parser.add_argument("--show-trajectory", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--show-perception", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--dt", type=float, default=1.0 / 60.0)
    parser.add_argument("--hold", action="store_true")
    parser.add_argument("--native-gl", dest="software_gl", action="store_false")
    parser.set_defaults(software_gl=True)
    args = parser.parse_args(argv)
    replay_online(args)


if __name__ == "__main__":
    main()
