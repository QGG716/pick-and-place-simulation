"""Render the frozen M-710 layout in Isaac Sim with one carton highlighted.

This is an initialization replay, not a grasp or unloading qualification.  All
40 cartons remain in the scene.  The selected top-centre carton is highlighted
consistently in three Isaac-rendered views while the validated initial robot
configuration is held fixed.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time
import traceback

from isaacsim import SimulationApp


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--project-root", required=True, type=Path)
    parser.add_argument("--usd-directory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--width", default=1280, type=int)
    parser.add_argument("--height", default=720, type=int)
    parser.add_argument("--fps", default=30, type=int)
    parser.add_argument("--seconds-per-view", default=2.0, type=float)
    return parser


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _rpy_degrees(rotation) -> list[float]:
    import numpy as np

    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
    return np.degrees([roll, pitch, yaw]).tolist()


def _rotation_matrix_from_rpy_degrees(value):
    import numpy as np

    roll, pitch, yaw = np.radians(np.asarray(value, dtype=float))
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


args = _parser().parse_args()
if args.width <= 0 or args.height <= 0 or args.fps <= 0 or args.seconds_per_view <= 0.0:
    raise ValueError("render dimensions, FPS, and seconds-per-view must be positive")
args.output.mkdir(parents=True, exist_ok=True)
args.usd_directory.mkdir(parents=True, exist_ok=True)
run_status_path = args.output / "run_status.json"
run_status_path.write_text(json.dumps({"status": "started", "time_unix_s": time.time()}, indent=2), encoding="utf-8")

simulation_app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "width": args.width,
        "height": args.height,
    }
)

try:
    # SimulationApp must exist before importing NumPy and Omniverse modules.
    import cv2
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import Articulation
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    project_root = args.project_root.resolve()
    sys.path.insert(0, str(project_root / "src"))
    from unloading_sim.isaac_layout_replay import (
        ISAAC_LAYOUT_DUMP_SCHEMA,
        audit_isaac_layout_backend_dump,
        usd_safe_prim_segment,
        verify_isaac_layout_contract,
    )

    contract_path = args.contract.resolve()
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    contract_verification = verify_isaac_layout_contract(contract)
    urdf_record = contract["robot"]["urdf"]
    urdf_path = project_root / urdf_record["repository_path"]
    if not urdf_path.is_file() or _sha256(urdf_path) != urdf_record["sha256"]:
        raise ValueError("robot URDF does not match the frozen replay contract")

    import_manifest_path = args.usd_directory / "m710id70_import_manifest.json"
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
    if cached and cached.get("urdf_sha256") == urdf_record["sha256"] and cached.get("settings") == settings and Path(cached["usd_path"]).is_file():
        import_manifest = cached
        usd_path = Path(cached["usd_path"])
        root_prim_path = str(cached["root_prim_path"])
    else:
        importer_config = URDFImporterConfig(
            urdf_path=str(urdf_path),
            usd_path=str(args.usd_directory.resolve()),
            merge_fixed_joints=False,
            merge_mesh=False,
            collision_from_visuals=False,
            collision_type="Convex Decomposition",
            allow_self_collision=True,
            fix_base=True,
            joint_drive_type="force",
            joint_target_type="position",
            override_joint_stiffness=1000.0,
            override_joint_damping=100.0,
            run_asset_transformer=True,
            run_multi_physics_conversion=True,
        )
        importer = URDFImporter(config=importer_config)
        usd_path = Path(importer.import_urdf()).resolve()
        root_prim_path = f"/{urdf_path.stem}"
        import_manifest = {
            "urdf_sha256": urdf_record["sha256"],
            "usd_path": str(usd_path),
            "root_prim_path": root_prim_path,
            "settings": settings,
        }
        import_manifest_path.write_text(json.dumps(import_manifest, indent=2), encoding="utf-8")
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
        if not root_prim.IsValid():
            raise RuntimeError("imported M-710 root prim is missing")
        root_prim_path = str(root_prim.GetPath())
    physics_variant = root_prim.GetVariantSet("Physics")
    if physics_variant.IsValid():
        physics_variant.SetVariantSelection("physx")

    mount = np.asarray(contract["robot"]["world_from_mount"], dtype=float)
    root_xform = UsdGeom.XformCommonAPI(root_prim)
    root_xform.SetTranslate(Gf.Vec3d(*mount[:3, 3].tolist()))
    root_xform.SetRotate(
        Gf.Vec3f(*_rpy_degrees(mount[:3, :3])),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )

    UsdGeom.Xform.Define(stage, "/Layout")
    UsdGeom.Xform.Define(stage, "/Layout/Primitives")
    UsdGeom.Xform.Define(stage, "/Layout/Materials")

    def material(name: str, rgb, *, emissive=False):
        path = f"/Layout/Materials/{name}"
        result = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        color = Gf.Vec3f(*[float(x) for x in rgb])
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.58)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.05)
        if emissive:
            shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(color)
        result.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return result

    materials = {
        "chassis": material("Chassis", (0.17, 0.23, 0.27)),
        "conveyor": material("Conveyor", (0.03, 0.39, 0.63)),
        "supporting_carton": material("Cartons", (0.61, 0.31, 0.10)),
        "selected_carton": material("SelectedCarton", (1.0, 0.18, 0.03), emissive=True),
        "tool_equal_scale_collision_proxy": material("ToolProxy", (0.0, 0.68, 0.68), emissive=True),
    }

    prim_paths: dict[str, str] = {}
    for index, item in enumerate(contract["primitives"]):
        # Keep the source name in the dump, but use a deterministic legal USD
        # identifier whose first character cannot be numeric.
        path = f"/Layout/Primitives/{usd_safe_prim_segment(index, item['name'])}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        pose = np.asarray(item["pose_world"], dtype=float)
        size = np.asarray(item["size_xyz_m"], dtype=float)
        xform = UsdGeom.XformCommonAPI(cube.GetPrim())
        xform.SetTranslate(Gf.Vec3d(*pose[:3, 3].tolist()))
        xform.SetRotate(Gf.Vec3f(*_rpy_degrees(pose[:3, :3])), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        xform.SetScale(Gf.Vec3f(*size.tolist()))
        role = item["role"]
        material_key = role
        if role == "fixed_assembly":
            material_key = "chassis" if item["name"] == "chassis" else "conveyor"
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(materials[material_key])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        prim_paths[item["name"]] = path

    # Render-only ground patch visualizes the known Z=0 plane; its finite X/Y
    # extent is deliberately excluded from the geometry contract because the
    # trailer length remains NOT_DEFINED.
    floor = UsdGeom.Cube.Define(stage, "/Layout/RenderOnlyFloorPatch")
    floor.CreateSizeAttr(1.0)
    floor_xform = UsdGeom.XformCommonAPI(floor.GetPrim())
    floor_xform.SetTranslate(Gf.Vec3d(-1.5, 0.0, -0.015))
    floor_xform.SetScale(Gf.Vec3f(7.0, 2.3, 0.03))
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material("Floor", (0.21, 0.22, 0.23)))

    rep.create.light(light_type="distant", intensity=750.0, rotation=(310.0, 0.0, 25.0))
    rep.create.light(light_type="sphere", position=(-1.0, -2.0, 4.0), intensity=5500.0, scale=0.8)
    rep.create.light(light_type="sphere", position=(-1.0, 2.0, 3.2), intensity=4000.0, scale=0.7)

    views = [
        ("overview", (-5.3, -4.1, 3.4), (-1.25, 0.0, 1.15)),
        ("opposite", (-5.0, 4.2, 3.1), (-1.20, 0.0, 1.10)),
        ("target_focus", (-2.3, -2.8, 3.0), (-0.25, 0.0, 1.55)),
    ]
    annotators = []
    render_products = []
    for _, eye, look_at in views:
        camera = rep.create.camera(position=eye, look_at=look_at, focal_length=24.0, clipping_range=(0.03, 20.0))
        render_product = rep.create.render_product(camera, (args.width, args.height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach(render_product)
        annotators.append(annotator)
        render_products.append(render_product)

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0)
    articulation = Articulation(root_prim_path)
    discovered_names = list(articulation.dof_names)
    expected_names = list(contract["robot"]["joint_names"])
    if set(discovered_names) != set(expected_names) or len(discovered_names) != len(expected_names):
        raise RuntimeError(f"M-710 articulation DOFs differ from snapshot: {discovered_names}")
    source_index = {name: index for index, name in enumerate(expected_names)}
    q = np.asarray(contract["robot"]["q_rad"], dtype=np.float32)
    command = q[[source_index[name] for name in discovered_names]]
    world.reset()
    articulation.switch_dof_control_mode("position")
    articulation.set_dof_positions(command[None, :])
    articulation.set_dof_position_targets(command[None, :])
    for _ in range(12):
        world.step(render=True)
    articulation.set_dof_positions(command[None, :])

    backend_primitives = []
    for item in contract["primitives"]:
        prim = stage.GetPrimAtPath(prim_paths[item["name"]])
        translate = prim.GetAttribute("xformOp:translate").Get()
        rotate = prim.GetAttribute("xformOp:rotateXYZ").Get()
        scale = prim.GetAttribute("xformOp:scale").Get()
        pose = np.eye(4)
        pose[:3, :3] = _rotation_matrix_from_rpy_degrees(list(rotate))
        pose[:3, 3] = np.asarray(list(translate), dtype=float)
        backend_primitives.append(
            {
                "name": item["name"],
                "prim_path": prim_paths[item["name"]],
                "pose_world": pose.tolist(),
                "size_xyz_m": np.asarray(list(scale), dtype=float).tolist(),
            }
        )
    measured_q = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0]
    q_by_name = {name: measured_q[index] for index, name in enumerate(discovered_names)}
    backend_dump = {
        "schema": ISAAC_LAYOUT_DUMP_SCHEMA,
        "contract_fingerprint": contract["contract_fingerprint"],
        "backend": {"name": "NVIDIA Isaac Sim", "version": _package_version("isaacsim")},
        "primitives": backend_primitives,
        "robot": {
            "root_prim_path": root_prim_path,
            "world_from_mount": mount.tolist(),
            "joint_names": expected_names,
            "q_rad": [float(q_by_name[name]) for name in expected_names],
        },
    }
    backend_audit = audit_isaac_layout_backend_dump(
        contract, backend_dump, pose_tolerance=1e-6, joint_tolerance=1e-5
    )
    if backend_audit["status"] != "PASS":
        raise RuntimeError(f"Isaac scene dump differs from frozen contract: {backend_audit}")
    (args.output / "backend_scene_dump.json").write_text(json.dumps(backend_dump, indent=2), encoding="utf-8")
    (args.output / "backend_scene_audit.json").write_text(json.dumps(backend_audit, indent=2), encoding="utf-8")

    video_path = args.output / "m710id70_layout_one_carton_focus.mp4"
    writer = cv2.VideoWriter(
        str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), float(args.fps), (args.width, args.height)
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the MP4 writer")
    frame_files = []
    held_frames = max(1, int(round(args.fps * args.seconds_per_view)))
    for view_index, ((view_name, _, _), annotator) in enumerate(zip(views, annotators), start=1):
        for _ in range(8):
            world.step(render=True)
        rgba = np.asarray(annotator.get_data())
        if rgba.ndim != 3 or rgba.shape[2] < 3:
            raise RuntimeError(f"Isaac camera returned an invalid image for {view_name}: {rgba.shape}")
        bgr = cv2.cvtColor(rgba[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
        cv2.rectangle(bgr, (0, 0), (args.width, 62), (18, 18, 18), -1)
        cv2.putText(
            bgr,
            f"Isaac Sim initialization replay {view_index}/3: {view_name}",
            (24, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (242, 242, 242), 2, cv2.LINE_AA,
        )
        cv2.putText(
            bgr,
            f"selected={contract['target']}  full stack=40  physical grasp=NOT EVALUATED",
            (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (80, 210, 255), 1, cv2.LINE_AA,
        )
        frame_path = args.output / f"{view_index:02d}_{view_name}.png"
        if not cv2.imwrite(str(frame_path), bgr):
            raise RuntimeError(f"could not write {frame_path}")
        frame_files.append(str(frame_path.resolve()))
        for _ in range(held_frames):
            writer.write(bgr)
    writer.release()
    if not video_path.is_file() or video_path.stat().st_size == 0:
        raise RuntimeError("Isaac replay MP4 is empty")

    result = {
        "status": "PASS",
        "scope": contract["scope"],
        "target": contract["target"],
        "carton_count": contract["claims"]["carton_count"],
        "physical_grasp": "NOT_EVALUATED",
        "payload_dynamics": "NOT_EVALUATED",
        "complete_workcell_clearance": contract["claims"]["complete_workcell_clearance"],
        "contract_verification": contract_verification,
        "backend_scene_audit": backend_audit,
        "isaacsim_version": _package_version("isaacsim"),
        "urdf_imported_this_run": imported_now,
        "urdf_sha256": urdf_record["sha256"],
        "adapter_sha256": _sha256(Path(__file__).resolve()),
        "contract_sha256": _sha256(contract_path),
        "render_only_floor_patch": "KNOWN_Z0_PLANE_WITH_NON_DIMENSIONAL_FINITE_DISPLAY_EXTENT",
        "video": str(video_path.resolve()),
        "frames": frame_files,
    }
    run_status_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
except Exception as exc:
    failure = {"status": "FAIL", "reason": str(exc), "traceback": traceback.format_exc()}
    run_status_path.write_text(json.dumps(failure, indent=2), encoding="utf-8")
    print(json.dumps(failure, indent=2), flush=True)
    raise
finally:
    simulation_app.close()
