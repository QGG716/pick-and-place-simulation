"""Capture RGB-D and exact scene truth for the M-710 perception contract.

This is a perception/ROS validation entry point.  It does not execute a grasp,
planner or unloading trajectory, and it never imports custom ROS messages into
Isaac's Python environment.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import traceback

from isaacsim import SimulationApp


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--bundle-directory", required=True, type=Path)
    result.add_argument("--project-root", required=True, type=Path)
    result.add_argument("--usd-directory", required=True, type=Path)
    result.add_argument("--output", required=True, type=Path)
    result.add_argument("--width", type=int, default=640)
    result.add_argument("--height", type=int, default=480)
    result.add_argument("--fps", type=int, default=20)
    result.add_argument("--seconds", type=float, default=12.0)
    return result


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def rpy_degrees(rotation) -> list[float]:
    import numpy as np

    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
    return np.degrees([roll, pitch, yaw]).tolist()


def rgb_data(value):
    return value["data"] if isinstance(value, dict) and "data" in value else value


args = parser().parse_args()
if min(args.width, args.height, args.fps) <= 0 or args.seconds <= 0.0:
    raise ValueError("render dimensions, FPS and duration must be positive")
args.output.mkdir(parents=True, exist_ok=True)
args.usd_directory.mkdir(parents=True, exist_ok=True)
status_path = args.output / "run_status.json"
status_path.write_text(json.dumps({"status": "started", "time_unix_s": time.time()}, indent=2), encoding="utf-8")

simulation_app = SimulationApp({
    "headless": True,
    "renderer": "RaytracedLighting",
    "width": args.width,
    "height": args.height,
})

try:
    import cv2
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.experimental.utils.semantics import add_labels
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    project_root = args.project_root.resolve()
    sys.path.insert(0, str(project_root / "packages/unloading_contracts/src"))
    sys.path.insert(0, str(project_root / "src"))
    from unloading_perception.isaac_validation import (
        IsaacSceneManifest,
        feasibility_digest,
        verify_text_asset_identity,
    )

    bundle_index = json.loads((args.bundle_directory / "index.json").read_text(encoding="utf-8"))
    manifests = []
    for record in bundle_index["scenes"]:
        value = json.loads((args.bundle_directory / record["path"]).read_text(encoding="utf-8"))
        manifest = IsaacSceneManifest.from_dict(value)
        if manifest.manifest_fingerprint != record["manifest_fingerprint"]:
            raise ValueError("bundle index/manifest identity mismatch")
        manifests.append((record["scene"], manifest))
    contract_path = project_root / "integration/isaac_scene_contract/m710id70_layout_v1/isaac_layout_contract.json"
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    contract_payload = dict(contract)
    if contract_payload.pop("contract_fingerprint") != feasibility_digest(contract_payload):
        raise ValueError("frozen feasibility Isaac layout contract fingerprint mismatch")
    urdf_record = contract["robot"]["urdf"]
    urdf_path = project_root / urdf_record["repository_path"]
    if not urdf_path.is_file():
        raise ValueError("robot URDF differs from the frozen scene contract")
    try:
        urdf_identity = verify_text_asset_identity(urdf_path, urdf_record["sha256"])
    except ValueError as exc:
        raise ValueError("robot URDF differs from the frozen scene contract") from exc

    import_manifest_path = args.usd_directory / "m710id70_perception_import.json"
    settings = {
        "merge_fixed_joints": False,
        "merge_mesh": False,
        "collision_from_visuals": False,
        "collision_type": "Convex Decomposition",
        "allow_self_collision": True,
        "fix_base": True,
    }
    cached = json.loads(import_manifest_path.read_text(encoding="utf-8")) if import_manifest_path.is_file() else None
    imported_now = False
    if cached and cached.get("urdf_contract_sha256") == urdf_record["sha256"] and cached.get("urdf_checkout_sha256") == urdf_identity["checkout_sha256"] and cached.get("settings") == settings and Path(cached["usd_path"]).is_file():
        usd_path = Path(cached["usd_path"])
        root_prim_path = str(cached["root_prim_path"])
    else:
        importer = URDFImporter(config=URDFImporterConfig(
            urdf_path=str(urdf_path), usd_path=str(args.usd_directory.resolve()),
            merge_fixed_joints=False, merge_mesh=False, collision_from_visuals=False,
            collision_type="Convex Decomposition", allow_self_collision=True, fix_base=True,
            joint_drive_type="force", joint_target_type="position",
            override_joint_stiffness=1000.0, override_joint_damping=100.0,
            run_asset_transformer=True, run_multi_physics_conversion=True,
        ))
        usd_path = Path(importer.import_urdf()).resolve()
        root_prim_path = f"/{urdf_path.stem}"
        import_manifest_path.write_text(json.dumps({
            "urdf_contract_sha256": urdf_record["sha256"],
            "urdf_checkout_sha256": urdf_identity["checkout_sha256"],
            "urdf_identity": urdf_identity, "usd_path": str(usd_path),
            "root_prim_path": root_prim_path, "settings": settings,
        }, indent=2), encoding="utf-8")
        imported_now = True

    context = omni.usd.get_context()
    if not context.open_stage(str(usd_path)):
        raise RuntimeError(f"could not open imported M-710 USD: {usd_path}")
    for _ in range(5):
        simulation_app.update()
    stage = context.get_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim.IsValid():
        root_prim = stage.GetDefaultPrim()
        root_prim_path = str(root_prim.GetPath())
    if not root_prim.IsValid():
        raise RuntimeError("imported M-710 root prim is missing")

    first_manifest = manifests[0][1]
    mount = np.asarray(first_manifest.robot["T_W_robot"], dtype=float)
    root_xform = UsdGeom.XformCommonAPI(root_prim)
    root_xform.SetTranslate(Gf.Vec3d(*mount[:3, 3].tolist()))
    root_xform.SetRotate(Gf.Vec3f(*rpy_degrees(mount[:3, :3])), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    physics_variant = root_prim.GetVariantSet("Physics")
    if physics_variant.IsValid():
        physics_variant.SetVariantSelection("physx")

    for path in ("/PerceptionValidation", "/PerceptionValidation/Primitives", "/PerceptionValidation/Materials"):
        UsdGeom.Xform.Define(stage, path)

    def material(name: str, rgb, *, emissive=False, opacity=1.0):
        path = f"/PerceptionValidation/Materials/{name}"
        result = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        color = Gf.Vec3f(*[float(value) for value in rgb])
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.62)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.05)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        if emissive:
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(color)
        result.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return result

    materials = {
        "chassis": material("Chassis", (0.10, 0.18, 0.24)),
        "conveyor": material("Conveyor", (0.03, 0.34, 0.62)),
        "carton": material("Carton", (0.63, 0.34, 0.13)),
        "target": material("Target", (0.98, 0.16, 0.03), emissive=True),
        "tool": material("Tool", (0.02, 0.68, 0.69), emissive=True),
        "floor": material("Floor", (0.16, 0.17, 0.18)),
        "wall": material("TrailerWall", (0.43, 0.45, 0.48)),
        "occluder": material("UnknownOccluder", (0.92, 0.78, 0.05), emissive=True, opacity=0.82),
    }

    def safe_name(index, name):
        return f"p_{index:03d}_" + "".join(character if character.isascii() and (character.isalnum() or character == "_") else "_" for character in str(name))

    prim_paths = {}
    for index, item in enumerate(contract["primitives"]):
        path = f"/PerceptionValidation/Primitives/{safe_name(index, item['name'])}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        pose = np.asarray(item["pose_world"], dtype=float)
        size = np.asarray(item["size_xyz_m"], dtype=float)
        xform = UsdGeom.XformCommonAPI(cube.GetPrim())
        xform.SetTranslate(Gf.Vec3d(*pose[:3, 3].tolist()))
        xform.SetRotate(Gf.Vec3f(*rpy_degrees(pose[:3, :3])), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        xform.SetScale(Gf.Vec3f(*size.tolist()))
        role = item["role"]
        key = "target" if role == "selected_carton" else "tool" if role == "tool_equal_scale_collision_proxy" else "carton" if "carton" in role else "chassis" if item["name"] == "chassis" else "conveyor"
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(materials[key])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if "carton" in role or role == "selected_carton":
            add_labels(cube.GetPrim(), labels="carton", taxonomy="class")
            add_labels(cube.GetPrim(), labels=item["name"], taxonomy="simulation_object_id")
        prim_paths[item["name"]] = path

    # Finite trailer surfaces are visualization-only because layout v1 leaves
    # real trailer length and height unconfirmed.
    visual_surfaces = [
        ("Floor", (1.0, 0.0, -0.03), (6.0, 2.3, 0.06), "floor"),
        ("LeftWall", (1.0, 1.18, 1.35), (6.0, 0.06, 2.7), "wall"),
        ("RightWall", (1.0, -1.18, 1.35), (6.0, 0.06, 2.7), "wall"),
        ("RoofRim", (-0.02, 0.0, 2.68), (0.08, 2.42, 0.08), "wall"),
    ]
    for name, position, scale, key in visual_surfaces:
        cube = UsdGeom.Cube.Define(stage, f"/PerceptionValidation/{name}")
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.XformCommonAPI(cube.GetPrim())
        xform.SetTranslate(Gf.Vec3d(*position))
        xform.SetScale(Gf.Vec3f(*scale))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(materials[key])

    occluder = UsdGeom.Cube.Define(stage, "/PerceptionValidation/UnknownOccluder")
    occluder.CreateSizeAttr(1.0)
    occluder_xform = UsdGeom.XformCommonAPI(occluder.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(occluder.GetPrim()).Bind(materials["occluder"])

    rep.create.light(light_type="distant", intensity=950.0, rotation=(305.0, 0.0, 20.0))
    rep.create.light(light_type="sphere", position=(-0.7, -1.8, 3.3), intensity=6500.0, scale=0.7)
    rep.create.light(light_type="sphere", position=(-0.7, 1.8, 3.0), intensity=5000.0, scale=0.7)
    overview_camera = rep.create.camera(position=(-4.8, -3.8, 3.4), look_at=(-0.7, 0.0, 1.2), focal_length=24.0, clipping_range=(0.05, 20.0))
    overview_product = rep.create.render_product(overview_camera, (args.width, args.height))
    overview_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    overview_annotator.attach(overview_product)

    cameras = []
    for scene_name, manifest in manifests:
        camera_config = manifest.cameras[0]
        t_w_c = np.asarray(camera_config["T_W_C"], dtype=float)
        position = t_w_c[:3, 3]
        look_at = np.asarray(camera_config["look_at_world_m"], dtype=float)
        declared_forward = t_w_c[:3, 2]
        actual_forward = (look_at - position) / np.linalg.norm(look_at - position)
        if np.max(np.abs(declared_forward - actual_forward)) > 1e-6:
            raise ValueError(f"{scene_name} look_at is inconsistent with declared T_W_C")
        camera = rep.create.camera(
            position=tuple(position.tolist()), look_at=tuple(look_at.tolist()),
            focal_length=float(camera_config["focal_length_mm"]),
            horizontal_aperture=float(camera_config["horizontal_aperture_mm"]),
            clipping_range=(float(camera_config["near_clip_m"]), float(camera_config["far_clip_m"])),
        )
        product = rep.create.render_product(camera, (args.width, args.height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        instance = rep.AnnotatorRegistry.get_annotator(
            "instance_segmentation",
            init_params={"colorize": False, "semanticTypes": ["class", "simulation_object_id"]},
        )
        rgb.attach(product)
        depth.attach(product)
        instance.attach(product)
        cameras.append((rgb, depth, instance))

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)
    articulation = Articulation(root_prim_path)
    discovered_names = list(articulation.dof_names)
    expected_names = list(first_manifest.robot["joint_names"])
    if set(discovered_names) != set(expected_names) or len(discovered_names) != len(expected_names):
        raise RuntimeError(f"M-710 articulation DOFs differ from manifest: {discovered_names}")
    source_index = {name: index for index, name in enumerate(expected_names)}
    q = np.asarray(first_manifest.robot["q_rad"], dtype=np.float32)
    command = q[[source_index[name] for name in discovered_names]]
    world.reset()
    articulation.switch_dof_control_mode("position")
    articulation.set_dof_positions(command[None, :])
    articulation.set_dof_position_targets(command[None, :])
    for _ in range(12):
        world.step(render=True)

    target_id = contract["target"]
    target_path = prim_paths[target_id]

    def apply_manifest(manifest, scene_name):
        by_id = {item["simulation_object_id"]: item for item in manifest.objects}
        for object_id, state in by_id.items():
            prim = stage.GetPrimAtPath(prim_paths[object_id])
            pose = np.asarray(state["T_W_object"], dtype=float)
            xform = UsdGeom.XformCommonAPI(prim)
            xform.SetTranslate(Gf.Vec3d(*pose[:3, 3].tolist()))
            xform.SetRotate(Gf.Vec3f(*rpy_degrees(pose[:3, :3])), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        if scene_name == "partial_occlusion":
            target = by_id[target_id]
            camera_position = np.asarray(manifest.cameras[0]["T_W_C"], dtype=float)[:3, 3]
            target_position = np.asarray(target["T_W_object"], dtype=float)[:3, 3]
            position = 0.72 * target_position + 0.28 * camera_position
            occluder_xform.SetTranslate(Gf.Vec3d(*position.tolist()))
            occluder_xform.SetScale(Gf.Vec3f(0.18, 0.70, 0.90))
        else:
            occluder_xform.SetTranslate(Gf.Vec3d(0.0, 0.0, -10.0))
            occluder_xform.SetScale(Gf.Vec3f(0.01, 0.01, 0.01))

    def project_annotations(manifest):
        camera = manifest.cameras[0]
        t_w_c = np.asarray(camera["T_W_C"], dtype=float)
        t_c_w = np.linalg.inv(t_w_c)
        k = np.asarray(camera["K"], dtype=float).reshape(3, 3)
        result = []
        for item in manifest.objects:
            pose = np.asarray(item["T_W_object"], dtype=float)
            size = np.asarray(item["full_dimensions_m"], dtype=float)
            signs = np.asarray([(x, y, z) for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
            local = signs * size
            world_points = (pose[:3, :3] @ local.T).T + pose[:3, 3]
            camera_points = (t_c_w[:3, :3] @ world_points.T).T + t_c_w[:3, 3]
            pixels = (k @ camera_points.T).T
            valid = camera_points[:, 2] > float(camera["near_clip_m"])
            if not np.any(valid):
                continue
            pixels = pixels[valid, :2] / pixels[valid, 2:3]
            bbox = [
                float(np.clip(np.min(pixels[:, 0]), 0.0, args.width - 1.0)),
                float(np.clip(np.min(pixels[:, 1]), 0.0, args.height - 1.0)),
                float(np.clip(np.max(pixels[:, 0]), 0.0, args.width - 1.0)),
                float(np.clip(np.max(pixels[:, 1]), 0.0, args.height - 1.0)),
            ]
            visible = bbox[2] - bbox[0] >= 1.0 and bbox[3] - bbox[1] >= 1.0
            result.append({
                "simulation_object_id": item["simulation_object_id"],
                "semantic_id": item["semantic_id"],
                "category": item["category"],
                "prim_path": prim_paths[item["simulation_object_id"]],
                "bbox_xyxy": bbox,
                "T_W_object": item["T_W_object"],
                "full_dimensions_m": item["full_dimensions_m"],
                "visible": visible,
                "occluded": bool(item["occluded"]),
                "source": "ISAAC_GROUND_TRUTH",
            })
        return result

    scene_records = []
    video_frames = []
    for scene_index, ((scene_name, manifest), (rgb_annotator, depth_annotator, instance_annotator)) in enumerate(zip(manifests, cameras), start=1):
        scene_dir = args.output / scene_name
        scene_dir.mkdir(parents=True, exist_ok=True)
        apply_manifest(manifest, scene_name)
        articulation.set_dof_positions(command[None, :])
        for _ in range(8):
            world.step(render=True)
        rgba = np.asarray(rgb_data(rgb_annotator.get_data()))
        depth = np.asarray(rgb_data(depth_annotator.get_data()), dtype=np.float32)
        instance_result = instance_annotator.get_data()
        instance_ids = np.asarray(instance_result["data"], dtype=np.uint32)
        overview_rgba = np.asarray(rgb_data(overview_annotator.get_data()))
        if rgba.shape != (args.height, args.width, 4) or depth.shape != (args.height, args.width) or instance_ids.shape != (args.height, args.width):
            raise RuntimeError(f"invalid sensor shapes for {scene_name}: {rgba.shape}, {depth.shape}, {instance_ids.shape}")
        rgb = rgba[:, :, :3].astype(np.uint8)
        overview = overview_rgba[:, :, :3].astype(np.uint8)
        rgb_path = scene_dir / "sensor_rgb.png"
        cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        np.save(scene_dir / "sensor_rgb.npy", rgb)
        cv2.imwrite(str(scene_dir / "isaac_overview.png"), cv2.cvtColor(overview, cv2.COLOR_RGB2BGR))
        np.save(scene_dir / "metric_depth_m.npy", depth)
        finite = np.isfinite(depth) & (depth > 0.0)
        depth_visual = np.zeros_like(depth, dtype=np.uint8)
        if np.any(finite):
            low, high = np.percentile(depth[finite], [2.0, 98.0])
            depth_visual[finite] = np.clip(255.0 * (depth[finite] - low) / max(high - low, 1e-6), 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(255 - depth_visual, cv2.COLORMAP_TURBO)
        cv2.imwrite(str(scene_dir / "metric_depth_visualization.png"), depth_color)

        camera = manifest.cameras[0]
        k = np.asarray(camera["K"], dtype=float).reshape(3, 3)
        rows, columns = np.indices(depth.shape)
        stride = 2
        selected = finite & ((rows % stride) == 0) & ((columns % stride) == 0)
        z = depth[selected]
        camera_points = np.column_stack(((columns[selected] - k[0, 2]) * z / k[0, 0], (rows[selected] - k[1, 2]) * z / k[1, 1], z))
        t_w_c = np.asarray(camera["T_W_C"], dtype=float)
        world_points = (t_w_c[:3, :3] @ camera_points.T).T + t_w_c[:3, 3]
        np.savez_compressed(scene_dir / "pointcloud_world_m.npz", xyz_m=world_points.astype(np.float32))

        camera_info = {
            "camera_id": camera["camera_id"], "frame_id": camera["frame_id"],
            "width": args.width, "height": args.height, "K": camera["K"],
            "distortion_model": camera["distortion_model"], "D": camera["distortion"],
            "T_W_C": camera["T_W_C"], "near_clip_m": camera["near_clip_m"],
            "far_clip_m": camera["far_clip_m"],
        }
        camera_info_path = scene_dir / "camera_info.json"
        camera_info_path.write_text(json.dumps(camera_info, indent=2), encoding="utf-8")
        masks_by_object = {}
        instance_identity = {}
        for numeric_id, labels in instance_result.get("info", {}).get("idToLabels", {}).items():
            label = labels.get("simulation_object_id") if isinstance(labels, dict) else None
            if isinstance(label, list):
                label = label[0] if len(label) == 1 else None
            if label in prim_paths:
                mask = instance_ids == int(numeric_id)
                if np.any(mask):
                    masks_by_object[str(label)] = mask
                    instance_identity[str(label)] = int(numeric_id)
        masks_path = scene_dir / "gt_instance_masks.npz"
        np.savez_compressed(masks_path, **masks_by_object)
        masks_sha256 = sha256(masks_path)
        annotations = project_annotations(manifest)
        for annotation in annotations:
            object_id = annotation["simulation_object_id"]
            mask = masks_by_object.get(object_id)
            annotation["mask_key"] = object_id if mask is not None else None
            annotation["mask_pixel_count"] = 0 if mask is None else int(mask.sum())
            annotation["isaac_instance_id"] = instance_identity.get(object_id)
            annotation["mask_reference"] = None if mask is None else {
                "uri": masks_path.resolve().as_uri() + f"#{object_id}",
                "sha256": masks_sha256,
                "media_type": "application/x-npz; array=bool",
            }
            if annotation["visible"] and mask is None:
                raise RuntimeError(f"visible GT object lacks an Isaac instance mask: {object_id}")
        annotations_path = scene_dir / "gt_annotations.json"
        annotations_payload = {
            "schema_version": "isaac_ground_truth_annotations_v1",
            "simulation_epoch": manifest.timing["simulation_epoch"],
            "simulation_frame": manifest.timing["simulation_frame"],
            "simulation_time": manifest.timing["simulation_time"],
            "manifest_fingerprint": manifest.manifest_fingerprint,
            "instance_masks_sha256": masks_sha256,
            "objects": annotations,
        }
        annotations_path.write_text(json.dumps(annotations_payload, indent=2), encoding="utf-8")
        binding = {
            "schema_version": "isaac_capture_binding_v1",
            "simulation_epoch": manifest.timing["simulation_epoch"],
            "frame_sequence": manifest.timing["simulation_frame"],
            "simulation_time": manifest.timing["simulation_time"],
            "rgb_sha256": sha256(rgb_path),
            "camera_calibration_identity": hashlib.sha256(json.dumps(camera_info, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            "gt_snapshot_sha256": sha256(annotations_path),
        }
        (scene_dir / "capture_binding.json").write_text(json.dumps(binding, indent=2), encoding="utf-8")

        gt_overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for annotation in annotations:
            if not annotation["visible"]:
                continue
            x1, y1, x2, y2 = [int(round(value)) for value in annotation["bbox_xyxy"]]
            color = (0, 220, 255) if annotation["simulation_object_id"] != target_id else (0, 70, 255)
            cv2.rectangle(gt_overlay, (x1, y1), (x2, y2), color, 1 if annotation["simulation_object_id"] != target_id else 3)
            if annotation["simulation_object_id"] == target_id:
                cv2.putText(gt_overlay, f"GT {target_id}", (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)
        cv2.rectangle(gt_overlay, (0, 0), (args.width, 34), (15, 15, 15), -1)
        cv2.putText(gt_overlay, f"GT | {scene_name} | objects={len(annotations)}", (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 230, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(scene_dir / "gt_overlay.png"), gt_overlay)
        prediction_overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.rectangle(prediction_overlay, (0, 0), (args.width, 56), (15, 15, 15), -1)
        cv2.putText(prediction_overlay, "PRED | awaiting Mode B1 worker", (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 180, 30), 2, cv2.LINE_AA)
        cv2.putText(prediction_overlay, "RAW_IMAGE_AUTOMATIC=false", (12, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 180, 30), 1, cv2.LINE_AA)
        cv2.imwrite(str(scene_dir / "prediction_overlay.png"), prediction_overlay)

        top_left = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        top_right = gt_overlay
        bottom_left = prediction_overlay
        bottom_right = cv2.cvtColor(overview, cv2.COLOR_RGB2BGR)
        panel = np.vstack((np.hstack((top_left, top_right)), np.hstack((bottom_left, bottom_right))))
        cv2.putText(panel, f"Isaac perception validation {scene_index}/6: {scene_name}", (18, panel.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2, cv2.LINE_AA)
        comparison_path = scene_dir / "comparison.png"
        cv2.imwrite(str(comparison_path), panel)
        video_frames.append(panel)
        scene_records.append({
            "scene": scene_name,
            "status": "PASS",
            "manifest_fingerprint": manifest.manifest_fingerprint,
            "dynamic_scene_fingerprint": manifest.dynamic_scene_fingerprint,
            "world_fingerprint": manifest.world_fingerprint,
            "gt_object_count": len(annotations),
            "finite_depth_fraction": float(finite.mean()),
            "rgb_sha256": binding["rgb_sha256"],
            "gt_sha256": binding["gt_snapshot_sha256"],
            "artifacts": {name: str((scene_dir / name).resolve()) for name in (
                "isaac_overview.png", "sensor_rgb.png", "sensor_rgb.npy", "metric_depth_m.npy",
                "metric_depth_visualization.png", "pointcloud_world_m.npz", "camera_info.json",
                "gt_annotations.json", "gt_instance_masks.npz", "capture_binding.json", "gt_overlay.png",
                "prediction_overlay.png", "comparison.png",
            )},
        })

    video_path = args.output / "isaac_perception_validation.mp4"
    frame_size = (args.width * 2, args.height * 2)
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), float(args.fps), frame_size)
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the perception MP4 writer")
    repeats = max(1, int(round(args.fps * args.seconds / len(video_frames))))
    for frame in video_frames:
        for _ in range(repeats):
            writer.write(frame)
    writer.release()
    if not video_path.is_file() or video_path.stat().st_size == 0:
        raise RuntimeError("perception validation MP4 is empty")

    measured_q = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0]
    q_by_name = {name: measured_q[index] for index, name in enumerate(discovered_names)}
    q_error = max(abs(float(q_by_name[name]) - float(first_manifest.robot["q_rad"][expected_names.index(name)])) for name in expected_names)
    try:
        gpu_name = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            check=True, text=True, capture_output=True, timeout=5,
        ).stdout.strip().splitlines()[0]
    except Exception:
        gpu_name = "UNKNOWN"
    result = {
        "status": "PASS",
        "scope": "Isaac sensor, exact-state and visualization validation; no planning or execution",
        "isaac_version": package_version("isaacsim"),
        "gpu": gpu_name,
        "layout_id": bundle_index["layout_id"],
        "layout_fingerprint": bundle_index["layout_fingerprint"],
        "feasibility_reference_commit": bundle_index["feasibility_reference_commit"],
        "perception_commit": bundle_index["perception_commit"],
        "raw_image_automatic": False,
        "mode_a_capture": "PASS",
        "mode_b1_capture": "READY_FOR_EXTERNAL_GPU_WORKER",
        "ros_bridge": "NOT_RUN_BY_ISAAC_PROCESS",
        "robot_q_max_abs_rad": q_error,
        "urdf_imported_this_run": imported_now,
        "urdf_sha256": urdf_record["sha256"],
        "urdf_checkout_identity": urdf_identity,
        "scene_count": len(scene_records),
        "scenes": scene_records,
        "video": str(video_path.resolve()),
        "video_sha256": sha256(video_path),
        "visualization_only_trailer_surfaces": "UNCONFIRMED_LENGTH_HEIGHT_NOT_PART_OF_LAYOUT_FINGERPRINT",
    }
    (args.output / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    status_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
except Exception as exc:
    failure = {"status": "FAIL", "reason": str(exc), "traceback": traceback.format_exc()}
    status_path.write_text(json.dumps(failure, indent=2), encoding="utf-8")
    print(json.dumps(failure, indent=2), flush=True)
    raise
finally:
    simulation_app.close()
