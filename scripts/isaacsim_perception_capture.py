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
    result.add_argument("--feasibility-root", required=True, type=Path)
    result.add_argument("--usd-directory", required=True, type=Path)
    result.add_argument("--output", required=True, type=Path)
    result.add_argument("--width", type=int, default=2592)
    result.add_argument("--height", type=int, default=1944)
    result.add_argument("--video-width", type=int, default=1280)
    result.add_argument("--video-height", type=int, default=720)
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
if min(args.width, args.height, args.video_width, args.video_height, args.fps) <= 0 or args.seconds <= 0.0:
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
    import Semantics
    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.experimental.utils.semantics import add_labels
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    project_root = args.project_root.resolve()
    sys.path.insert(0, str(project_root / "packages/unloading_contracts/src"))
    sys.path.insert(0, str(project_root / "src"))
    from unloading_perception.isaac_validation import (
        IsaacSceneManifest,
        feasibility_digest,
        verify_text_asset_identity,
    )
    from unloading_perception.rgbd import CaptureMetadata, IlluminationState
    from unloading_perception.vision_rig import evaluate_vision_rig_pose, load_vision_rig_spec

    bundle_index = json.loads((args.bundle_directory / "index.json").read_text(encoding="utf-8"))
    bundle_scene_records = {str(item["scene"]): item for item in bundle_index["scenes"]}
    manifests = []
    for record in bundle_index["scenes"]:
        value = json.loads((args.bundle_directory / record["path"]).read_text(encoding="utf-8"))
        manifest = IsaacSceneManifest.from_dict(value)
        if manifest.manifest_fingerprint != record["manifest_fingerprint"]:
            raise ValueError("bundle index/manifest identity mismatch")
        manifests.append((record["scene"], manifest))
    expected_resolution = tuple(int(value) for value in manifests[0][1].cameras[0]["resolution"])
    if (args.width, args.height) != expected_resolution:
        raise ValueError(f"sensor render must use declared full resolution {expected_resolution}")
    contract_path = project_root / "integration/isaac_scene_contract/m710id70_layout_v1/isaac_layout_contract.json"
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    contract_payload = dict(contract)
    if contract_payload.pop("contract_fingerprint") != feasibility_digest(contract_payload):
        raise ValueError("frozen feasibility Isaac layout contract fingerprint mismatch")
    feasibility_root = args.feasibility_root.resolve()
    expected_feasibility_commit = str(bundle_index["feasibility_reference_commit"])
    actual_feasibility_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=feasibility_root, check=True,
        text=True, capture_output=True, timeout=10,
    ).stdout.strip()
    if actual_feasibility_commit != expected_feasibility_commit:
        raise ValueError(
            f"feasibility checkout must be exactly {expected_feasibility_commit}; got {actual_feasibility_commit}"
        )
    urdf_record = manifests[0][1].robot
    urdf_path = feasibility_root / str(urdf_record["robot_asset_path"])
    if not urdf_path.is_file():
        raise ValueError("latest feasibility official robot URDF is missing")
    try:
        urdf_identity = verify_text_asset_identity(urdf_path, urdf_record["robot_asset_hash"])
    except ValueError as exc:
        raise ValueError("latest feasibility official robot URDF failed identity verification") from exc

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
    if cached and cached.get("urdf_contract_sha256") == urdf_record["robot_asset_hash"] and cached.get("urdf_checkout_sha256") == urdf_identity["checkout_sha256"] and cached.get("feasibility_commit") == actual_feasibility_commit and cached.get("settings") == settings and Path(cached["usd_path"]).is_file():
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
            "urdf_contract_sha256": urdf_record["robot_asset_hash"],
            "urdf_checkout_sha256": urdf_identity["checkout_sha256"],
            "feasibility_commit": actual_feasibility_commit,
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
        "mast": material("VisionMast", (0.86, 0.88, 0.91)),
        "module": material("PerceptionModule", (0.04, 0.05, 0.07)),
        "lens": material("SensorLens", (0.04, 0.34, 0.52), emissive=True),
    }

    # A modular visual proxy with the same hierarchy as the handoff contract.
    # Its root is evaluated from J1 for every scene; no static world camera pose
    # or independent mast yaw joint exists.
    vision_root = UsdGeom.Xform.Define(stage, "/PerceptionValidation/VisionRig")
    vision_root.GetPrim().CreateAttribute("rigId", Sdf.ValueTypeNames.String).Set("m710id70_j1_perception_mast_v2")
    vision_root.GetPrim().CreateAttribute("kinematicParentFrame", Sdf.ValueTypeNames.String).Set("J1_link")
    flange_xform = UsdGeom.Xform.Define(stage, "/PerceptionValidation/VisionRig/VisionFlange")
    flange_proxy = UsdGeom.Cube.Define(stage, "/PerceptionValidation/VisionRig/VisionFlange/Proxy")
    flange_proxy.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(flange_proxy.GetPrim()).SetScale(Gf.Vec3f(0.18, 0.18, 0.05))
    UsdShade.MaterialBindingAPI.Apply(flange_proxy.GetPrim()).Bind(materials["mast"])
    mast_xform = UsdGeom.Xform.Define(stage, "/PerceptionValidation/VisionRig/VisionFlange/Mast")
    mast_proxy = UsdGeom.Cube.Define(stage, "/PerceptionValidation/VisionRig/VisionFlange/Mast/Body")
    mast_proxy.CreateSizeAttr(1.0)
    mast_body_xform = UsdGeom.XformCommonAPI(mast_proxy.GetPrim())
    mast_body_xform.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.75))
    mast_body_xform.SetScale(Gf.Vec3f(0.07, 0.07, 1.5))
    UsdShade.MaterialBindingAPI.Apply(mast_proxy.GetPrim()).Bind(materials["mast"])
    module_xform = UsdGeom.Xform.Define(stage, "/PerceptionValidation/VisionRig/VisionFlange/Mast/PerceptionModule0")
    module_api = UsdGeom.XformCommonAPI(module_xform.GetPrim())
    module_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 1.3))
    module_api.SetRotate(Gf.Vec3f(0.0, 0.0, 90.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    module_proxy = UsdGeom.Cube.Define(stage, "/PerceptionValidation/VisionRig/VisionFlange/Mast/PerceptionModule0/Housing")
    module_proxy.CreateSizeAttr(1.0)
    module_proxy_api = UsdGeom.XformCommonAPI(module_proxy.GetPrim())
    module_proxy_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    module_proxy_api.SetScale(Gf.Vec3f(0.12, 0.34, 0.11))
    UsdShade.MaterialBindingAPI.Apply(module_proxy.GetPrim()).Bind(materials["module"])
    for sensor_name, lateral in (("RGBCamera", 0.035), ("DepthCamera", -0.035)):
        sensor = UsdGeom.Cube.Define(stage, f"/PerceptionValidation/VisionRig/VisionFlange/Mast/PerceptionModule0/{sensor_name}")
        sensor.CreateSizeAttr(1.0)
        sensor_api = UsdGeom.XformCommonAPI(sensor.GetPrim())
        sensor_api.SetTranslate(Gf.Vec3d(0.065, lateral, 0.0))
        sensor_api.SetScale(Gf.Vec3f(0.025, 0.045, 0.045))
        UsdShade.MaterialBindingAPI.Apply(sensor.GetPrim()).Bind(materials["lens"])
    fill_lights = []
    for light_name, lateral in (("FillLightLeft", 0.12), ("FillLightRight", -0.12)):
        light = UsdLux.DiskLight.Define(
            stage, f"/PerceptionValidation/VisionRig/VisionFlange/Mast/PerceptionModule0/{light_name}"
        )
        light.CreateRadiusAttr(0.035)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.93, 0.82))
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(5000.0)
        light_api = UsdGeom.XformCommonAPI(light.GetPrim())
        light_api.SetTranslate(Gf.Vec3d(0.07, lateral, 0.0))
        light_api.SetRotate(Gf.Vec3f(0.0, -90.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        fill_lights.append(light)

    environment_distant = UsdLux.DistantLight.Define(stage, "/PerceptionValidation/EnvironmentDistant")
    environment_distant.CreateAngleAttr(4.0)
    UsdGeom.XformCommonAPI(environment_distant.GetPrim()).SetRotate(
        Gf.Vec3f(305.0, 0.0, 20.0), UsdGeom.XformCommonAPI.RotationOrderXYZ
    )
    environment_sphere = UsdLux.SphereLight.Define(stage, "/PerceptionValidation/EnvironmentSphere")
    environment_sphere.CreateRadiusAttr(0.7)
    UsdGeom.XformCommonAPI(environment_sphere.GetPrim()).SetTranslate(Gf.Vec3d(-0.7, -1.8, 3.3))

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
            # Isaac 6.0.1's Replicator annotator still consumes the legacy
            # SemanticsAPI even though the public utility writes
            # UsdSemantics.LabelsAPI.  Author both forms for this runtime.
            for taxonomy, value in (("class", "carton"), ("simulation_object_id", item["name"])):
                legacy = Semantics.SemanticsAPI.Apply(cube.GetPrim(), taxonomy)
                legacy.CreateSemanticTypeAttr().Set(taxonomy)
                legacy.CreateSemanticDataAttr().Set(value)
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

    overview_camera = rep.create.camera(position=(-4.8, -3.8, 3.4), look_at=(-0.7, 0.0, 1.2), focal_length=24.0, clipping_range=(0.05, 20.0))
    overview_product = rep.create.render_product(overview_camera, (args.width, args.height))
    overview_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    overview_annotator.attach(overview_product)
    nominal_manifest = manifests[0][1]
    nominal_j1 = np.asarray(nominal_manifest.mechanisms["vision_rig"]["T_W_J1"], dtype=float)[:3, 3]
    nominal_module = np.asarray(nominal_manifest.mechanisms["vision_rig"]["T_W_module_0_main"], dtype=float)[:3, 3]
    top_camera = rep.create.camera(
        position=(float(nominal_j1[0] - 0.4), float(nominal_j1[1]), 5.4),
        look_at=(float(nominal_j1[0] + 0.2), float(nominal_j1[1]), 0.7),
        focal_length=35.0, clipping_range=(0.05, 20.0),
    )
    top_product = rep.create.render_product(top_camera, (args.video_width, args.video_height))
    top_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    top_annotator.attach(top_product)
    side_camera = rep.create.camera(
        position=(float(nominal_j1[0] - 2.1), float(nominal_j1[1] - 2.6), 2.3),
        look_at=tuple(nominal_module.tolist()), focal_length=42.0, clipping_range=(0.05, 12.0),
    )
    side_product = rep.create.render_product(side_camera, (args.video_width, args.video_height))
    side_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    side_annotator.attach(side_product)
    closeup_camera = rep.create.camera(
        position=(float(nominal_module[0] + 0.8), float(nominal_module[1] - 0.55), float(nominal_module[2] + 0.25)),
        look_at=tuple(nominal_module.tolist()), focal_length=55.0, clipping_range=(0.02, 5.0),
    )
    closeup_product = rep.create.render_product(closeup_camera, (args.video_width, args.video_height))
    closeup_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    closeup_annotator.attach(closeup_product)

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

    rig_spec = load_vision_rig_spec(project_root / "configs/isaac/perception_sensing_pose.yaml")
    sweep_manifest = next(manifest for name, manifest in manifests if name == "J1_ROTATION_SWEEP")
    sweep_sensors = []
    for angle_deg in (-90.0, -45.0, 0.0, 45.0, 90.0):
        angle_rad = math.radians(angle_deg)
        rig_pose = evaluate_vision_rig_pose(sweep_manifest.robot["T_W_robot"], angle_rad, rig_spec)
        position = np.asarray(rig_pose.camera_center_world_m, dtype=float)
        forward = np.asarray([rig_pose.T_W_camera_optical[row][2] for row in range(3)], dtype=float)
        sweep_camera = rep.create.camera(
            position=tuple(position.tolist()), look_at=tuple((position + forward).tolist()),
            focal_length=float(sweep_manifest.cameras[0]["focal_length_mm"]),
            horizontal_aperture=float(sweep_manifest.cameras[0]["horizontal_aperture_mm"]),
            clipping_range=(float(sweep_manifest.cameras[0]["near_clip_m"]), float(sweep_manifest.cameras[0]["far_clip_m"])),
        )
        sweep_product = rep.create.render_product(sweep_camera, (args.video_width, args.video_height))
        sweep_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        sweep_instance = rep.AnnotatorRegistry.get_annotator(
            "instance_segmentation", init_params={"colorize": False, "semanticTypes": ["class", "simulation_object_id"]}
        )
        sweep_rgb.attach(sweep_product)
        sweep_instance.attach(sweep_product)
        sweep_sensors.append((angle_deg, angle_rad, rig_pose, sweep_rgb, sweep_instance))

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

    def semantic_value(labels, taxonomy):
        value = labels.get(taxonomy) if isinstance(labels, dict) else None
        if isinstance(value, list):
            return value[0] if len(value) == 1 else None
        return value

    def semantic_ids(result, taxonomy, expected_value=None):
        values = set()
        for numeric_id, labels in result.get("info", {}).get("idToSemantics", {}).items():
            value = semantic_value(labels, taxonomy)
            if value is not None and (expected_value is None or value == expected_value):
                values.add(int(numeric_id))
        return values

    def carton_mask(result):
        ids = semantic_ids(result, "class", "carton")
        data = np.asarray(result["data"], dtype=np.uint32)
        return np.isin(data, tuple(ids)) if ids else np.zeros(data.shape, dtype=bool)

    def apply_manifest(manifest, scene_name):
        by_id = {item["simulation_object_id"]: item for item in manifest.objects}
        for object_id, state in by_id.items():
            prim = stage.GetPrimAtPath(prim_paths[object_id])
            imageable = UsdGeom.Imageable(prim)
            imageable.MakeVisible() if state.get("visible", True) else imageable.MakeInvisible()
            pose = np.asarray(state["T_W_object"], dtype=float)
            xform = UsdGeom.XformCommonAPI(prim)
            xform.SetTranslate(Gf.Vec3d(*pose[:3, 3].tolist()))
            xform.SetRotate(Gf.Vec3f(*rpy_degrees(pose[:3, :3])), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        rig_pose = np.asarray(manifest.mechanisms["vision_rig"]["T_W_vision_flange"], dtype=float)
        vision_api = UsdGeom.XformCommonAPI(vision_root.GetPrim())
        vision_api.SetTranslate(Gf.Vec3d(*rig_pose[:3, 3].tolist()))
        vision_api.SetRotate(Gf.Vec3f(*rpy_degrees(rig_pose[:3, :3])), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        record = bundle_scene_records[scene_name]
        dark = scene_name in {"DARK_LIGHT_OFF", "DARK_LIGHT_ON"}
        environment_distant.CreateIntensityAttr(35.0 if dark else 950.0)
        environment_sphere.CreateIntensityAttr(80.0 if dark else 6500.0)
        fill_intensity = 0.0 if record["illumination_state"] == "LIGHT_OFF" else 2500.0
        for light in fill_lights:
            light.CreateIntensityAttr(fill_intensity)
        if scene_name == "PARTIAL_OCCLUSION":
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
    lighting_samples = {}
    for scene_index, ((scene_name, manifest), (rgb_annotator, depth_annotator, instance_annotator)) in enumerate(zip(manifests, cameras), start=1):
        scene_dir = args.output / scene_name
        scene_dir.mkdir(parents=True, exist_ok=True)
        apply_manifest(manifest, scene_name)
        articulation.set_dof_positions(command[None, :])
        articulation.set_dof_position_targets(command[None, :])
        for _ in range(8):
            world.step(render=True)
        if scene_name == "MECHANICAL_TOP_VIEW":
            top_image = cv2.cvtColor(np.asarray(rgb_data(top_annotator.get_data()))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
            side_image = cv2.cvtColor(np.asarray(rgb_data(side_annotator.get_data()))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
            closeup_image = cv2.cvtColor(np.asarray(rgb_data(closeup_annotator.get_data()))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
            cv2.rectangle(top_image, (0, 0), (top_image.shape[1], 112), (10, 10, 10), -1)
            cv2.putText(top_image, "W +X: INTO TRAILER    W +Y: LEFT", (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 240, 255), 2, cv2.LINE_AA)
            cv2.putText(top_image, "MAST = J1 + 0.400 m LEFT (+Y)", (16, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (80, 255, 100), 2, cv2.LINE_AA)
            cv2.putText(top_image, "camera +X / arm -Y / phase = +90 deg", (16, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 210, 60), 2, cv2.LINE_AA)
            cv2.arrowedLine(top_image, (top_image.shape[1] - 180, 82), (top_image.shape[1] - 60, 82), (0, 240, 255), 3, tipLength=0.15)
            cv2.arrowedLine(top_image, (top_image.shape[1] - 180, 82), (top_image.shape[1] - 180, 20), (80, 255, 100), 3, tipLength=0.18)
            cv2.imwrite(str(args.output / "01_mast_left_top_view.png"), top_image)
            cv2.imwrite(str(args.output / "02_mast_j1_90deg_relation.png"), top_image)
            cv2.imwrite(str(args.output / "02_side_view_mast_camera.png"), side_image)
            cv2.imwrite(str(args.output / "04_rgbd_module_closeup.png"), closeup_image)
            cv2.imwrite(str(args.output / "04_isaac_vision_rig.png"), closeup_image)
        if scene_name == "J1_ROTATION_SWEEP":
            sweep_tiles = []
            occlusion_samples = []
            for angle_deg, angle_rad, rig_pose, sweep_rgb, sweep_instance in sweep_sensors:
                sample_command = command.copy()
                sample_command[discovered_names.index("J1")] = angle_rad
                articulation.set_dof_positions(sample_command[None, :])
                articulation.set_dof_position_targets(sample_command[None, :])
                rig_matrix = np.asarray(rig_pose.T_W_vision_flange, dtype=float)
                vision_api = UsdGeom.XformCommonAPI(vision_root.GetPrim())
                vision_api.SetTranslate(Gf.Vec3d(*rig_matrix[:3, 3].tolist()))
                vision_api.SetRotate(Gf.Vec3f(*rpy_degrees(rig_matrix[:3, :3])), UsdGeom.XformCommonAPI.RotationOrderXYZ)
                for _ in range(4):
                    world.step(render=True)
                with_robot_result = sweep_instance.get_data()
                with_robot_pixels = int(np.count_nonzero(carton_mask(with_robot_result)))
                sweep_image = np.asarray(rgb_data(sweep_rgb.get_data()))[:, :, :3].astype(np.uint8)
                UsdGeom.Imageable(root_prim).MakeInvisible()
                for _ in range(2):
                    world.step(render=True)
                without_robot_result = sweep_instance.get_data()
                without_robot_pixels = int(np.count_nonzero(carton_mask(without_robot_result)))
                UsdGeom.Imageable(root_prim).MakeVisible()
                robot_occlusion_fraction = max(0.0, 1.0 - with_robot_pixels / max(1, without_robot_pixels))
                tile = cv2.cvtColor(sweep_image, cv2.COLOR_RGB2BGR)
                cv2.rectangle(tile, (0, 0), (tile.shape[1], 66), (12, 12, 12), -1)
                cv2.putText(tile, f"q1={angle_deg:+.0f} deg", (14, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 230, 255), 2, cv2.LINE_AA)
                cv2.putText(tile, f"robot occlusion={robot_occlusion_fraction:.3f}", (14, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 210, 50), 1, cv2.LINE_AA)
                sweep_tiles.append(tile)
                j1_center = np.asarray(rig_pose.j1_center_world_m)
                mast_center = np.asarray(rig_pose.mast_center_world_m)
                occlusion_samples.append({
                    "q1_deg": angle_deg,
                    "q1_rad": angle_rad,
                    "j1_center_world_m": j1_center.tolist(),
                    "mast_center_world_m": mast_center.tolist(),
                    "mast_j1_radius_m": float(np.linalg.norm((mast_center - j1_center)[:2])),
                    "camera_j1_yaw_offset_deg": 90.0,
                    "carton_pixels_with_robot": with_robot_pixels,
                    "carton_pixels_without_robot": without_robot_pixels,
                    "robot_occlusion_fraction": robot_occlusion_fraction,
                })
            blank = np.zeros_like(sweep_tiles[0])
            montage = np.vstack((np.hstack(sweep_tiles[:3]), np.hstack((sweep_tiles[3], sweep_tiles[4], blank))))
            cv2.imwrite(str(scene_dir / "03_j1_sweep_montage.png"), montage)
            cv2.imwrite(str(args.output / "03_j1_sweep_montage.png"), montage)
            (scene_dir / "occlusion_by_robot_vs_q1.json").write_text(json.dumps({
                "schema_version": "occlusion_by_robot_vs_q1_v1",
                "rig_id": "m710id70_j1_perception_mast_v2",
                "method": "Isaac rendered semantic carton pixels with robot visible versus hidden",
                "samples": occlusion_samples,
                "status": "PASS",
            }, indent=2), encoding="utf-8")
            apply_manifest(manifest, scene_name)
            articulation.set_dof_positions(command[None, :])
            articulation.set_dof_position_targets(command[None, :])
            for _ in range(4):
                world.step(render=True)
        rgba = np.asarray(rgb_data(rgb_annotator.get_data()))
        depth = np.asarray(rgb_data(depth_annotator.get_data()), dtype=np.float32)
        instance_result = instance_annotator.get_data()
        instance_ids = np.asarray(instance_result["data"], dtype=np.uint32)
        overview_rgba = np.asarray(rgb_data(overview_annotator.get_data()))
        if rgba.shape != (args.height, args.width, 4) or depth.shape != (args.height, args.width) or instance_ids.shape != (args.height, args.width):
            raise RuntimeError(f"invalid sensor shapes for {scene_name}: {rgba.shape}, {depth.shape}, {instance_ids.shape}")
        camera = manifest.cameras[0]
        if camera["intrinsics_mode"] == "EXACT_USER_SPEC_RESAMPLED":
            # Isaac renders square-pixel HFOV=90 input.  A deterministic
            # vertical resample realizes the independently specified VFOV=65
            # while retaining the requested 2592x1944 output and exact K.
            target_k = np.asarray(camera["K"], dtype=np.float64).reshape(3, 3)
            source_fy = float(target_k[0, 0])
            source_cy = (args.height - 1.0) / 2.0
            output_rows, output_columns = np.indices((args.height, args.width), dtype=np.float32)
            map_x = output_columns
            map_y = ((output_rows - float(target_k[1, 2])) * source_fy / float(target_k[1, 1]) + source_cy).astype(np.float32)
            rgba = cv2.remap(rgba, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            depth = cv2.remap(depth, map_x, map_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=float("nan"))
            # OpenCV geometric transforms reject CV_32U/CV_32S.  Replicator
            # IDs are exactly representable at these magnitudes in float32;
            # nearest-neighbour interpolation preserves their integer labels.
            instance_ids = cv2.remap(
                instance_ids.astype(np.float32), map_x, map_y, cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
            ).astype(np.uint32)
        rgb = rgba[:, :, :3].astype(np.uint8)
        overview = overview_rgba[:, :, :3].astype(np.uint8)
        frame_instance_result = {"data": instance_ids, "info": instance_result.get("info", {})}
        if scene_name in {"DARK_LIGHT_OFF", "DARK_LIGHT_ON"}:
            luminance = (0.2126 * rgb[:, :, 0] + 0.7152 * rgb[:, :, 1] + 0.0722 * rgb[:, :, 2]) / 255.0
            lighting_samples[scene_name] = {
                "mean_luminance": float(np.mean(luminance)),
                "shadow_fraction": float(np.mean(luminance < 0.08)),
                "underexposed_fraction": float(np.mean(luminance < 0.04)),
                "saturated_pixel_ratio": float(np.mean(luminance > 0.98)),
                "semantic_carton_pixel_count": int(np.count_nonzero(carton_mask(frame_instance_result))),
                "semantic_mask": carton_mask(frame_instance_result),
            }
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
        if scene_name == "DARK_LIGHT_OFF":
            cv2.imwrite(str(args.output / "05_dark_light_off.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        elif scene_name == "DARK_LIGHT_ON":
            cv2.imwrite(str(args.output / "06_dark_light_on.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        elif scene_name == "FULL_STACK_NOMINAL":
            cv2.imwrite(str(args.output / "07_fullres_rgb_preview.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(args.output / "08_metric_depth_preview.png"), depth_color)
            preview_size = (args.video_width, args.video_height)
            registered_preview = np.hstack((
                cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), preview_size, interpolation=cv2.INTER_AREA),
                cv2.resize(depth_color, preview_size, interpolation=cv2.INTER_AREA),
            ))
            cv2.putText(registered_preview, "REGISTERED RGB | OPTICAL_Z_M", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(str(args.output / "09_registered_rgbd.png"), registered_preview)

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
            "intrinsics_mode": camera["intrinsics_mode"],
            "registration_mode": camera["registration_mode"],
            "depth_semantics": camera["depth_semantics"],
            "native_square_pixel_fy_px": float(k[0, 0]),
            "vertical_resampling_applied": camera["intrinsics_mode"] == "EXACT_USER_SPEC_RESAMPLED",
        }
        camera_info_path = scene_dir / "camera_info.json"
        camera_info_path.write_text(json.dumps(camera_info, indent=2), encoding="utf-8")
        masks_by_object = {}
        instance_identity = {}
        segmentation_info = instance_result.get("info", {})
        (scene_dir / "instance_segmentation_info.json").write_text(
            json.dumps(segmentation_info, indent=2, default=lambda value: value.item() if hasattr(value, "item") else str(value)),
            encoding="utf-8",
        )
        for numeric_id, labels in segmentation_info.get("idToSemantics", {}).items():
            label = semantic_value(labels, "simulation_object_id")
            if label in prim_paths:
                mask = instance_ids == int(numeric_id)
                if np.any(mask):
                    masks_by_object[str(label)] = mask
                    instance_identity[str(label)] = int(numeric_id)
        if not masks_by_object:
            raise RuntimeError(f"Isaac emitted no identified carton masks for {scene_name}")
        masks_path = scene_dir / "gt_instance_masks.npz"
        np.savez_compressed(masks_path, **masks_by_object)
        masks_sha256 = sha256(masks_path)
        annotations = project_annotations(manifest)
        for annotation in annotations:
            object_id = annotation["simulation_object_id"]
            mask = masks_by_object.get(object_id)
            projection_intersects_image = bool(annotation["visible"])
            annotation["projection_intersects_image"] = projection_intersects_image
            annotation["visible"] = mask is not None
            annotation["occluded"] = bool(annotation["occluded"] or (projection_intersects_image and mask is None))
            if mask is not None:
                mask_rows, mask_columns = np.nonzero(mask)
                annotation["bbox_xyxy"] = [
                    float(mask_columns.min()), float(mask_rows.min()),
                    float(mask_columns.max() + 1), float(mask_rows.max() + 1),
                ]
            annotation["mask_key"] = object_id if mask is not None else None
            annotation["mask_pixel_count"] = 0 if mask is None else int(mask.sum())
            annotation["isaac_instance_id"] = instance_identity.get(object_id)
            annotation["mask_reference"] = None if mask is None else {
                "uri": masks_path.resolve().as_uri() + f"#{object_id}",
                "sha256": masks_sha256,
                "media_type": "application/x-npz; array=bool",
            }
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
        scene_record = next(item for item in bundle_index["scenes"] if item["scene"] == scene_name)
        capture_time = float(manifest.timing["simulation_time"])
        capture_metadata = CaptureMetadata(
            capture_id=f"{manifest.timing['simulation_epoch']}:module_0_main:{manifest.timing['simulation_frame']}",
            sensor_epoch=str(manifest.timing["simulation_epoch"]),
            frame_sequence=int(manifest.timing["simulation_frame"]),
            requested_time=capture_time,
            capture_start=capture_time,
            capture_center_time=capture_time,
            capture_end=capture_time,
            clock_domain="ros_sim_time",
            rgb_frame_id=str(camera["frame_id"]),
            depth_frame_id=str(camera["depth_frame_id"]),
            calibration_identity=binding["camera_calibration_identity"],
            illumination_state=IlluminationState(scene_record["illumination_state"]),
            j1_state_identity=hashlib.sha256(json.dumps({
                "joint_names": manifest.robot["joint_names"],
                "q_rad": manifest.robot["q_rad"],
                "simulation_epoch": manifest.timing["simulation_epoch"],
                "simulation_frame": manifest.timing["simulation_frame"],
            }, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            q1_at_capture_rad=float(camera["q1_at_capture_rad"]),
            T_W_C_at_capture=tuple(tuple(float(value) for value in row) for row in camera["T_W_C"]),
        )
        (scene_dir / "capture_metadata.json").write_text(
            json.dumps(capture_metadata.to_dict(), indent=2), encoding="utf-8"
        )

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

        video_size = (args.video_width, args.video_height)
        top_left = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), video_size, interpolation=cv2.INTER_AREA)
        top_right = cv2.resize(gt_overlay, video_size, interpolation=cv2.INTER_AREA)
        bottom_left = cv2.resize(prediction_overlay, video_size, interpolation=cv2.INTER_AREA)
        bottom_right = cv2.resize(cv2.cvtColor(overview, cv2.COLOR_RGB2BGR), video_size, interpolation=cv2.INTER_AREA)
        panel = np.vstack((np.hstack((top_left, top_right)), np.hstack((bottom_left, bottom_right))))
        cv2.putText(panel, f"Isaac perception validation {scene_index}/{len(manifests)}: {scene_name}", (18, panel.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2, cv2.LINE_AA)
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
                "gt_annotations.json", "gt_instance_masks.npz", "instance_segmentation_info.json",
                "capture_binding.json", "capture_metadata.json", "gt_overlay.png",
                "prediction_overlay.png", "comparison.png",
            )},
        })

    if set(lighting_samples) != {"DARK_LIGHT_OFF", "DARK_LIGHT_ON"}:
        raise RuntimeError("controlled LIGHT_OFF/LIGHT_ON captures are incomplete")
    off_mask = lighting_samples["DARK_LIGHT_OFF"].pop("semantic_mask")
    on_mask = lighting_samples["DARK_LIGHT_ON"].pop("semantic_mask")
    intersection = int(np.logical_and(off_mask, on_mask).sum())
    union = int(np.logical_or(off_mask, on_mask).sum())
    lighting_report = {
        "schema_version": "simulation_illumination_validation_v1",
        "status": "PASS",
        "geometry_status": "SIMULATION_LIGHT_PROXY",
        "claim_boundary": "simulation illumination validation; no real lux claim",
        "controlled_variables": ["scene", "camera_pose", "exposure", "proposal", "depth"],
        "samples": lighting_samples,
        "mask_iou_light_off_vs_on": float(intersection / max(1, union)),
        "segmentation_retention_ratio": float(
            lighting_samples["DARK_LIGHT_ON"]["semantic_carton_pixel_count"]
            / max(1, lighting_samples["DARK_LIGHT_OFF"]["semantic_carton_pixel_count"])
        ),
    }
    (args.output / "illumination_assumption.json").write_text(json.dumps({
        "geometry_status": "SIMULATION_LIGHT_PROXY",
        "light_type": "disk_proxy",
        "color_temperature_k": 5000.0,
        "intensity": 2500.0,
        "beam_spread_rad": 1.0471975511965976,
        "trigger_mode": "CAPTURE_SYNCHRONIZED",
        "real_luminaire_cad_available": False,
    }, indent=2), encoding="utf-8")
    (args.output / "lighting_comparison.json").write_text(json.dumps(lighting_report, indent=2), encoding="utf-8")

    video_path = args.output / "isaac_j1_mast_rgbd_validation.mp4"
    frame_size = (args.video_width * 2, args.video_height * 2)
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
    q_errors = {
        name: abs(float(q_by_name[name]) - float(first_manifest.robot["q_rad"][expected_names.index(name)]))
        for name in expected_names
    }
    q_error = max(q_errors.values())
    if q_errors["J1"] > 1e-3:
        raise RuntimeError(f"captured official-URDF J1 differs from commanded sensing pose: {q_errors['J1']}")
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
        "robot_joint_abs_error_rad": q_errors,
        "robot_j1_capture_pose_status": "PASS",
        "other_joint_dynamic_state_status": "NOT_AN_EXECUTION_QUALIFICATION",
        "urdf_imported_this_run": imported_now,
        "urdf_sha256": urdf_record["robot_asset_hash"],
        "feasibility_commit": actual_feasibility_commit,
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
