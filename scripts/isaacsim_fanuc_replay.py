"""Import the FANUC URDF into Isaac Sim and replay a timed command bundle."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

def _sha256_path(path: Path) -> str:
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


def _git_evidence(project_root: Path) -> dict[str, object]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty_lines = subprocess.run(
            ["git", "status", "--short"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()
        return {"commit": commit, "dirty": bool(dirty_lines), "changed_path_count": len(dirty_lines)}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None, "changed_path_count": None}


def _load_workspace_module(path: Path, module_name: str):
    """Load a lightweight gate by path without importing unloading_sim."""
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load workspace gate module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--project-root", required=True, type=Path)
    parser.add_argument("--usd-directory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--physics-hz",
        default=None,
        type=float,
        help="optional assertion; M-710 uses the bundle physics time step",
    )
    parser.add_argument(
        "--render-every",
        default=None,
        type=int,
        help="optional assertion; M-710 derives the render stride from bundle physics/FPS",
    )
    parser.add_argument("--width", default=None, type=int)
    parser.add_argument("--height", default=None, type=int)
    parser.add_argument("--output-fps", default=None, type=float)
    parser.add_argument("--camera-mode", default=None)
    parser.add_argument(
        "--record-replay",
        action="store_true",
        help="save the rendered qualification camera stream as replay.gif",
    )
    parser.add_argument(
        "--record-video",
        action="store_true",
        help="save the rendered qualification camera stream as replay.mp4",
    )
    parser.add_argument(
        "--video-preview-speed",
        default=4.0,
        type=float,
        help="also write replay_<N>x.mp4 at this viewing-only speed; use 1 to disable",
    )
    parser.add_argument("--max-sim-seconds", type=float)
    parser.add_argument("--post-release-seconds", default=None, type=float)
    parser.add_argument("--disable-gripper-break-limits", action="store_true")
    parser.add_argument("--gripper-force-limit", type=float)
    parser.add_argument("--gripper-shear-force-limit", type=float)
    parser.add_argument("--gripper-torque-limit", type=float)
    parser.add_argument("--gripper-attachment-point-count", type=int)
    parser.add_argument(
        "--disable-gripper-collision",
        action="store_true",
        help="diagnostic only: render the STEP mesh without applying its collision shape",
    )
    parser.add_argument(
        "--gripper-model",
        choices=("surface_gripper", "fixed_joint_diagnostic"),
        default="surface_gripper",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.physics_hz is not None and (
        args.physics_hz <= 0.0 or not math.isfinite(args.physics_hz)
    ):
        raise ValueError("physics-hz must be finite and positive")
    if args.render_every is not None and args.render_every <= 0:
        raise ValueError("render-every must be positive")
    if (args.width is not None and args.width <= 0) or (
        args.height is not None and args.height <= 0
    ):
        raise ValueError("camera dimensions must be positive")
    if args.output_fps is not None and (
        not math.isfinite(args.output_fps) or args.output_fps <= 0.0
    ):
        raise ValueError("output-fps must be finite and positive")
    if args.camera_mode is not None and not args.camera_mode.strip():
        raise ValueError("camera-mode must be non-empty")
    if args.gripper_attachment_point_count is not None and not (
        1 <= args.gripper_attachment_point_count
    ):
        raise ValueError("gripper-attachment-point-count must be positive")
    if args.max_sim_seconds is not None and args.max_sim_seconds <= 0.0:
        raise ValueError("max-sim-seconds must be positive")
    if args.post_release_seconds is not None and (
        not math.isfinite(args.post_release_seconds) or args.post_release_seconds < 0.0
    ):
        raise ValueError("post-release-seconds must be finite and non-negative")
    if not math.isfinite(args.video_preview_speed) or args.video_preview_speed < 1.0:
        raise ValueError("video-preview-speed must be finite and at least one")
    for name, value in (
        ("gripper-force-limit", args.gripper_force_limit),
        ("gripper-shear-force-limit", args.gripper_shear_force_limit),
        ("gripper-torque-limit", args.gripper_torque_limit),
    ):
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            raise ValueError(f"{name} must be finite and positive")
    if args.gripper_model == "surface_gripper" and (
        args.disable_gripper_break_limits or args.gripper_torque_limit is not None
    ):
        raise ValueError(
            "FixedJoint break-limit options require --gripper-model fixed_joint_diagnostic"
        )


def _sample(commands_t, commands_q, timestamp: float):
    clamped = float(np.clip(timestamp, 0.0, commands_t[-1]))
    return np.asarray(
        [np.interp(clamped, commands_t, commands_q[:, joint]) for joint in range(commands_q.shape[1])],
        dtype=np.float32,
    )


def _rpy_degrees_from_rotation_matrix(rotation):
    """Return XYZ Euler angles for Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
    return np.degrees(np.asarray([roll, pitch, yaw], dtype=float))


def _safe_prim_name(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return sanitized if sanitized and not sanitized[0].isdigit() else f"item_{sanitized}"


def _quaternion_wxyz_from_matrix(rotation):
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale,
             (matrix[0, 2] - matrix[2, 0]) / scale, (matrix[1, 0] - matrix[0, 1]) / scale]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                 (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [(matrix[0, 2] - matrix[2, 0]) / scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale]
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                [(matrix[1, 0] - matrix[0, 1]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale]
            )
    return quaternion / np.linalg.norm(quaternion)


def _rotation_matrix_from_quaternion_wxyz(quaternion):
    value = np.asarray(quaternion, dtype=float)
    value /= np.linalg.norm(value)
    w, x, y, z = value
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _quaternion_multiply_wxyz(left, right):
    lw, lx, ly, lz = np.asarray(left, dtype=float)
    rw, rx, ry, rz = np.asarray(right, dtype=float)
    return np.asarray(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )


def _quaternion_conjugate_wxyz(quaternion):
    value = np.asarray(quaternion, dtype=float)
    return np.asarray([value[0], -value[1], -value[2], -value[3]], dtype=float)


def _quaternion_angle_wxyz(quaternion):
    value = np.asarray(quaternion, dtype=float)
    value /= np.linalg.norm(value)
    return 2.0 * math.acos(float(np.clip(abs(value[0]), 0.0, 1.0)))


args = _parser().parse_args()
_validate_args(args)
bundle = json.loads(args.bundle.resolve().read_text(encoding="utf-8"))
if bundle.get("format") != "isaacsim_fanuc_replay_v1":
    raise ValueError("unsupported replay bundle format")
metadata = bundle["metadata"]
expected_joint_names = list(metadata["joint_names"])
pre_simulation_integrity_gate = None
if metadata.get("robot_model") == "fanuc_m710id_70":
    project_root = args.project_root.resolve()
    contract_module = _load_workspace_module(
        project_root / "src" / "unloading_sim" / "m710_replay_contract.py",
        "_m710_replay_contract_pre_simulation_gate",
    )
    asset_module = _load_workspace_module(
        project_root / "src" / "unloading_sim" / "asset_audit.py",
        "_m710_asset_audit_pre_simulation_gate",
    )
    current_asset_audit = asset_module.audit_m710id70_asset_set(project_root)
    pre_simulation_integrity_gate = contract_module.verify_m710_replay_bundle(
        bundle,
        project_root=project_root,
        current_asset_audit=current_asset_audit,
    )
    execution_blockers = metadata.get("execution_blockers")
    execution_fingerprint = metadata.get("execution_asset_fingerprint_sha256")
    if not isinstance(execution_blockers, list) or any(
        not isinstance(item, str) or not item.strip() for item in execution_blockers
    ):
        raise ValueError("M-710 execution blockers must be a list of non-empty strings")
    if not isinstance(execution_fingerprint, str) or re.fullmatch(
        r"[0-9a-f]{64}", execution_fingerprint
    ) is None:
        raise ValueError("M-710 dynamic replay requires a valid execution asset fingerprint")
    if metadata.get("simulation_execution_ready") is not True or execution_blockers:
        raise ValueError(
            "M-710 dynamic replay is fail-closed until simulation readiness passes: "
            + ", ".join(execution_blockers)
        )
    if not isinstance(metadata.get("machine_qualified"), bool):
        raise ValueError("M-710 machine_qualified must be a boolean")
    machine_warnings = metadata.get("machine_qualification_warnings")
    if not isinstance(machine_warnings, list) or any(
        not isinstance(item, str) or not item.strip() for item in machine_warnings
    ):
        raise ValueError(
            "M-710 machine qualification warnings must be a list of non-empty strings"
        )
    expected_overall_qualification = bool(
        metadata["simulation_execution_ready"] and metadata["machine_qualified"]
    )
    if metadata.get("execution_qualified") is not expected_overall_qualification:
        raise ValueError(
            "M-710 execution_qualified must combine simulation readiness and machine qualification"
        )

physics_contract = metadata.get("physics", {})
if metadata.get("robot_model") == "fanuc_m710id_70":
    if not isinstance(physics_contract, dict):
        raise ValueError("M-710 replay requires a physics contract")
    contract_time_step = float(physics_contract.get("physics_time_step_s", float("nan")))
    if not math.isfinite(contract_time_step) or contract_time_step <= 0.0:
        raise ValueError("M-710 replay requires a finite positive physics time step")
    contract_physics_hz = 1.0 / contract_time_step
    if args.physics_hz is not None and not math.isclose(
        args.physics_hz, contract_physics_hz, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError(
            "--physics-hz conflicts with the content-addressed M-710 physics contract"
        )
    physics_hz = contract_physics_hz
    rendering_contract = metadata.get("rendering")
    if not isinstance(rendering_contract, dict) or set(rendering_contract) != {
        "required_output"
    }:
        raise ValueError("M-710 replay requires a content-addressed rendering contract")
    required_output = rendering_contract["required_output"]
    required_output_keys = {
        "width_px",
        "height_px",
        "fps",
        "camera_mode",
        "render_every_physics_steps",
    }
    if not isinstance(required_output, dict) or set(required_output) != required_output_keys:
        raise ValueError("M-710 replay required-output contract is incomplete")
    required_width = required_output["width_px"]
    required_height = required_output["height_px"]
    required_fps = required_output["fps"]
    required_camera_mode = required_output["camera_mode"]
    required_render_every = required_output["render_every_physics_steps"]
    if (
        any(isinstance(item, bool) or not isinstance(item, int) or item <= 0 for item in (
            required_width,
            required_height,
            required_fps,
            required_render_every,
        ))
        or required_camera_mode
        != "fixed_overview_with_contact_and_place_keyframes"
        or not math.isclose(
            physics_hz / required_render_every,
            float(required_fps),
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    ):
        raise ValueError("M-710 rendering contract is inconsistent with its physics rate")
    for option, actual, required in (
        ("--width", args.width, required_width),
        ("--height", args.height, required_height),
        ("--render-every", args.render_every, required_render_every),
        ("--output-fps", args.output_fps, required_fps),
        ("--camera-mode", args.camera_mode, required_camera_mode),
    ):
        if actual is not None and actual != required:
            raise ValueError(f"{option} conflicts with the content-addressed M-710 output contract")
    args.width = required_width
    args.height = required_height
    args.render_every = required_render_every
    args.output_fps = float(required_fps)
    args.camera_mode = required_camera_mode
    required_post_release_seconds = metadata.get("required_post_release_settle_seconds")
    if not isinstance(required_post_release_seconds, (int, float)) or isinstance(
        required_post_release_seconds, bool
    ):
        raise ValueError("M-710 replay requires a post-release settling duration")
    required_post_release_seconds = float(required_post_release_seconds)
    if not math.isfinite(required_post_release_seconds) or required_post_release_seconds < 0.0:
        raise ValueError("M-710 post-release settling duration is invalid")
    if args.post_release_seconds is not None and not math.isclose(
        args.post_release_seconds,
        required_post_release_seconds,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError(
            "--post-release-seconds conflicts with the content-addressed M-710 replay contract"
        )
    args.post_release_seconds = required_post_release_seconds
else:
    physics_hz = 60.0 if args.physics_hz is None else args.physics_hz
    args.width = 640 if args.width is None else args.width
    args.height = 360 if args.height is None else args.height
    args.render_every = 6 if args.render_every is None else args.render_every
    args.output_fps = (
        physics_hz / args.render_every if args.output_fps is None else args.output_fps
    )
    args.camera_mode = "legacy_fixed_overview" if args.camera_mode is None else args.camera_mode
    args.post_release_seconds = (
        1.0 if args.post_release_seconds is None else args.post_release_seconds
    )
args.output.mkdir(parents=True, exist_ok=True)
run_status_path = args.output / "run_status.json"
run_started_unix_s = time.time()
run_status_path.write_text(
    json.dumps({"status": "started", "started_unix_s": run_started_unix_s}, indent=2),
    encoding="utf-8",
)

from isaacsim import SimulationApp


app_started_at = time.perf_counter()
simulation_app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "width": args.width,
        "height": args.height,
    }
)
app_startup_wall_s = time.perf_counter() - app_started_at
print("FANUC_REPLAY_STAGE=simulation_app_ready", flush=True)

try:
    # Isaac Sim requires SimulationApp to exist before importing NumPy and any
    # Omniverse modules. Keeping this ordering also avoids Kit bootstrap issues
    # when the host has a separate Conda installation.
    import numpy as np
    print("FANUC_REPLAY_STAGE=numpy_imported", flush=True)
    from unloading_sim.qualification import (
        ReplayQualificationPolicy,
        evaluate_replay_qualification,
    )
    from unloading_sim.m710_replay_physics import (
        audit_surface_attachment_contact,
        select_active_conveyor_surfaces,
    )
    import omni.replicator.core as rep
    print("FANUC_REPLAY_STAGE=replicator_imported", flush=True)
    import omni.usd
    print("FANUC_REPLAY_STAGE=usd_imported", flush=True)
    from PIL import Image
    print("FANUC_REPLAY_STAGE=pillow_imported", flush=True)
    if args.record_video:
        import cv2
        print("FANUC_REPLAY_STAGE=opencv_imported", flush=True)
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    print("FANUC_REPLAY_STAGE=urdf_importer_imported", flush=True)
    from isaacsim.core.api import World
    print("FANUC_REPLAY_STAGE=world_imported", flush=True)
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    print("FANUC_REPLAY_STAGE=articulation_imported", flush=True)
    from isaacsim.robot.surface_gripper.bindings._surface_gripper import (
        Closed as SurfaceGripperClosed,
        Open as SurfaceGripperOpen,
        acquire_surface_gripper_interface,
    )
    from usd.schema.isaac import robot_schema
    print("FANUC_REPLAY_STAGE=surface_gripper_imported", flush=True)
    from omni.physx import get_physx_simulation_interface
    from pxr import (
        Gf,
        PhysicsSchemaTools,
        PhysxSchema,
        Sdf,
        Usd,
        UsdGeom,
        UsdPhysics,
        UsdShade,
    )
    print("FANUC_REPLAY_STAGE=pxr_imported", flush=True)

    def _obb_penetration_depth(center_a, half_a, rotation_a, center_b, half_b, rotation_b):
        """Return the minimum SAT overlap, or zero for separated/touching OBBs."""

        center_a = np.asarray(center_a, dtype=float)
        center_b = np.asarray(center_b, dtype=float)
        half_a = np.asarray(half_a, dtype=float)
        half_b = np.asarray(half_b, dtype=float)
        rotation_a = np.asarray(rotation_a, dtype=float)
        rotation_b = np.asarray(rotation_b, dtype=float)
        axes = [*rotation_a.T, *rotation_b.T]
        axes.extend(
            np.cross(first, second)
            for first in rotation_a.T
            for second in rotation_b.T
        )
        center_delta = center_b - center_a
        minimum_overlap = float("inf")
        for raw_axis in axes:
            magnitude = float(np.linalg.norm(raw_axis))
            if magnitude <= 1e-10:
                continue
            axis = raw_axis / magnitude
            radius_a = float(np.sum(half_a * np.abs(rotation_a.T @ axis)))
            radius_b = float(np.sum(half_b * np.abs(rotation_b.T @ axis)))
            overlap = radius_a + radius_b - abs(float(center_delta @ axis))
            if overlap <= 0.0:
                return 0.0
            minimum_overlap = min(minimum_overlap, overlap)
        return 0.0 if not np.isfinite(minimum_overlap) else minimum_overlap

    def _load_binary_stl(path: Path):
        triangle_dtype = np.dtype(
            [
                ("normal", "<f4", (3,)),
                ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ]
        )
        with path.open("rb") as stream:
            stream.seek(80)
            triangle_count = int(np.fromfile(stream, dtype="<u4", count=1)[0])
            triangles = np.fromfile(stream, dtype=triangle_dtype, count=triangle_count)
        if len(triangles) != triangle_count:
            raise ValueError(f"truncated binary STL: {path}")
        raw_vertices = triangles["vertices"].reshape(-1, 3)
        vertices, indices = np.unique(raw_vertices, axis=0, return_inverse=True)
        return vertices.astype(float), indices.astype(np.int32)

    def _resolve_visual_asset(relative_path: str) -> Path:
        asset_path = (args.project_root.resolve() / relative_path).resolve()
        if not asset_path.is_file():
            raise FileNotFoundError(f"rendering asset not found: {asset_path}")
        return asset_path

    def _preview_material(material_path: str, spec: dict[str, object]):
        """Create one USD Preview Surface material from audited local assets."""
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
            float(spec.get("roughness", 0.55))
        )
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
            float(spec.get("metallic", 0.0))
        )
        fallback = np.asarray(spec.get("fallback_rgb", [0.5, 0.5, 0.5]), dtype=float)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*np.clip(fallback, 0.0, 1.0).tolist())
        )

        diffuse_path = spec.get("diffuse_path")
        if diffuse_path:
            primvar = UsdShade.Shader.Define(stage, f"{material_path}/Texcoord")
            primvar.CreateIdAttr("UsdPrimvarReader_float2")
            primvar.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            texture = UsdShade.Shader.Define(stage, f"{material_path}/DiffuseTexture")
            texture.CreateIdAttr("UsdUVTexture")
            texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                str(_resolve_visual_asset(str(diffuse_path)))
            )
            texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            texture_scale = np.asarray(
                spec.get("texture_scale_rgba", [1.0, 1.0, 1.0, 1.0]), dtype=float
            )
            if texture_scale.shape != (4,) or not np.all(np.isfinite(texture_scale)):
                raise ValueError("texture_scale_rgba must contain four finite numbers")
            texture.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(
                Gf.Vec4f(*texture_scale.tolist())
            )
            texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                primvar.ConnectableAPI(), "result"
            )
            shader.GetInput("diffuseColor").ConnectToSource(texture.ConnectableAPI(), "rgb")

        arm_path = spec.get("arm_path")
        if arm_path:
            arm_texture = UsdShade.Shader.Define(stage, f"{material_path}/ArmTexture")
            arm_texture.CreateIdAttr("UsdUVTexture")
            arm_texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                str(_resolve_visual_asset(str(arm_path)))
            )
            arm_texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
            arm_texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                primvar.ConnectableAPI(), "result"
            )
            shader.GetInput("roughness").ConnectToSource(arm_texture.ConnectableAPI(), "g")
            if bool(spec.get("use_arm_metallic", False)):
                shader.GetInput("metallic").ConnectToSource(
                    arm_texture.ConnectableAPI(), "b"
                )

        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    def _unit_box_visual(prim_path: str, material) -> None:
        """Add a UV-mapped visual box while retaining the parent cube for physics."""
        mesh = UsdGeom.Mesh.Define(stage, f"{prim_path}/TexturedVisual")
        faces = (
            ((-0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, 0.5, 0.5), (-0.5, -0.5, 0.5)),
            ((0.5, 0.5, -0.5), (0.5, -0.5, -0.5), (0.5, -0.5, 0.5), (0.5, 0.5, 0.5)),
            ((-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, -0.5, -0.5)),
            ((-0.5, 0.5, 0.5), (-0.5, 0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5)),
            ((-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5)),
            ((-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5)),
        )
        mesh.CreatePointsAttr([Gf.Vec3f(*point) for face in faces for point in face])
        mesh.CreateFaceVertexCountsAttr([4] * 6)
        mesh.CreateFaceVertexIndicesAttr(list(range(24)))
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        )
        st.Set(
            [Gf.Vec2f(*uv) for _ in faces for uv in ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))]
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    def _decal_plane(
        prim_path: str,
        *,
        image_path: str,
        y_half: float,
        z_min: float,
        z_max: float,
    ) -> None:
        material = _preview_material(
            f"{prim_path}/DecalMaterial",
            {"diffuse_path": image_path, "roughness": 0.6, "metallic": 0.0},
        )
        mesh = UsdGeom.Mesh.Define(stage, f"{prim_path}/Decal")
        mesh.CreatePointsAttr(
            [
                Gf.Vec3f(-0.501, -y_half, z_min),
                Gf.Vec3f(-0.501, y_half, z_min),
                Gf.Vec3f(-0.501, y_half, z_max),
                Gf.Vec3f(-0.501, -y_half, z_max),
            ]
        )
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        )
        st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    timestamps = np.asarray(bundle["timestamps_seconds"], dtype=float)
    positions = np.asarray(bundle["positions_rad"], dtype=float)
    if timestamps.ndim != 1 or positions.shape != (len(timestamps), len(expected_joint_names)):
        raise ValueError("replay bundle command dimensions are invalid")

    args.usd_directory.mkdir(parents=True, exist_ok=True)
    urdf_path = (args.project_root.resolve() / metadata["urdf_path"]).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"FANUC URDF not found: {urdf_path}")
    print("FANUC_REPLAY_STAGE=inputs_validated", flush=True)

    import_manifest_path = args.usd_directory / "import_manifest.json"
    urdf_sha256 = _sha256_path(urdf_path)
    requested_import_settings = {
        "collision_source": str(metadata.get("collision_geometry", "urdf_collision_mesh")),
        "dynamic_collision_approximation": "Convex Decomposition",
        "fix_base": True,
        "allow_self_collision": True,
        "merge_fixed_joints": False,
        "merge_mesh": False,
        "execution_asset_fingerprint_sha256": metadata.get(
            "execution_asset_fingerprint_sha256"
        ),
    }
    import_started_at = time.perf_counter()
    imported_now = False
    cached_import_manifest = (
        json.loads(import_manifest_path.read_text(encoding="utf-8"))
        if import_manifest_path.is_file()
        else None
    )
    cache_matches = bool(
        cached_import_manifest is not None
        and cached_import_manifest.get("urdf_sha256") == urdf_sha256
        and cached_import_manifest.get("import_settings") == requested_import_settings
    )
    if cache_matches:
        import_manifest = cached_import_manifest
        usd_path = Path(import_manifest["usd_path"])
        if not usd_path.is_file():
            raise FileNotFoundError(f"cached FANUC USD is missing: {usd_path}")
        root_prim_path = str(import_manifest["root_prim_path"])
    else:
        config = URDFImporterConfig(
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
        importer = URDFImporter(config=config)
        usd_path = Path(importer.import_urdf()).resolve()
        root_prim_path = f"/{urdf_path.stem}"
        import_manifest = {
            "urdf_path": str(urdf_path),
            "urdf_sha256": urdf_sha256,
            "usd_path": str(usd_path),
            "root_prim_path": root_prim_path,
            "collision_source": requested_import_settings["collision_source"],
            "dynamic_collision_approximation": requested_import_settings[
                "dynamic_collision_approximation"
            ],
            "fix_base": requested_import_settings["fix_base"],
            "import_settings": requested_import_settings,
        }
        import_manifest_path.write_text(json.dumps(import_manifest, indent=2), encoding="utf-8")
        imported_now = True
    import_wall_s = time.perf_counter() - import_started_at
    print("FANUC_REPLAY_STAGE=usd_resolved", flush=True)

    context = omni.usd.get_context()
    if not context.open_stage(str(usd_path)):
        raise RuntimeError(f"Isaac Sim could not open imported FANUC USD: {usd_path}")
    for _ in range(5):
        simulation_app.update()
    print("FANUC_REPLAY_STAGE=stage_opened", flush=True)
    stage = context.get_stage()
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim.IsValid():
        default_prim = stage.GetDefaultPrim()
        if not default_prim.IsValid():
            raise RuntimeError(f"imported FANUC root prim is missing: {root_prim_path}")
        root_prim = default_prim
        root_prim_path = str(root_prim.GetPath())
        import_manifest["root_prim_path"] = root_prim_path
        import_manifest_path.write_text(json.dumps(import_manifest, indent=2), encoding="utf-8")
    physics_variant = root_prim.GetVariantSet("Physics")
    if physics_variant.IsValid():
        physics_variant.SetVariantSelection("physx")

    # The source M-710 URDF intentionally contains no estimated inertials.
    # Apply the separately versioned engineering model in the adapter and
    # fail if an imported link cannot be bound, so estimates never masquerade
    # as manufacturer URDF data or silently fall back to importer density.
    link_dynamics = list(metadata.get("robot_link_dynamics", []))
    tool_mass_accounting = metadata.get("robot_tool_mass_accounting")
    if metadata.get("robot_model") == "fanuc_m710id_70":
        if not isinstance(tool_mass_accounting, dict) or tool_mass_accounting.get(
            "policy"
        ) != "FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE":
            raise ValueError("M-710 replay requires explicit exactly-once tool mass accounting")
        applied_mass = float(sum(float(item["mass_kg"]) for item in link_dynamics))
        expected_mass = float(tool_mass_accounting["source_robot_mass_kg"]) + float(
            tool_mass_accounting["tool_mass_kg"]
        )
        if (
            tool_mass_accounting.get("grasp_body_link") != "J6_link"
            or tool_mass_accounting.get("declared_attachment_link") != "J6_link"
            or tool_mass_accounting.get("declared_policy")
            != tool_mass_accounting.get("applied_policy")
            or tool_mass_accounting.get("independent_tool_rigid_body_created") is not False
            or not np.isclose(applied_mass, expected_mass, atol=1e-9, rtol=0.0)
            or not np.isclose(
                applied_mass,
                float(tool_mass_accounting["applied_articulation_mass_kg"]),
                atol=1e-9,
                rtol=0.0,
            )
        ):
            raise ValueError("M-710 exactly-once tool mass accounting is inconsistent")
    if link_dynamics:
        imported_link_prims = {
            prim.GetName(): prim
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(root_prim_path)
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        }
        missing_dynamic_links = sorted(
            str(item["link"])
            for item in link_dynamics
            if str(item["link"]) not in imported_link_prims
        )
        if missing_dynamic_links:
            raise RuntimeError(
                f"imported robot is missing engineering dynamics links: {missing_dynamic_links}"
            )
        for item in link_dynamics:
            link_name = str(item["link"])
            mass = float(item["mass_kg"])
            center = np.asarray(item["com_xyz_m"], dtype=float)
            inertia = np.asarray(item["inertia_at_com_kg_m2"], dtype=float)
            moments, axes = np.linalg.eigh(inertia)
            if np.linalg.det(axes) < 0.0:
                axes[:, 0] *= -1.0
            principal_quaternion = _quaternion_wxyz_from_matrix(axes)
            mass_api = UsdPhysics.MassAPI.Apply(imported_link_prims[link_name])
            mass_api.CreateMassAttr(mass)
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*center.tolist()))
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*moments.tolist()))
            mass_api.CreatePrincipalAxesAttr(
                Gf.Quatf(
                    float(principal_quaternion[0]),
                    Gf.Vec3f(*principal_quaternion[1:].tolist()),
                )
            )

    srdf_path = urdf_path.with_suffix(".srdf")
    allowed_self_collision_pairs: list[dict[str, str]] = []
    srdf_filter_complete = False
    if srdf_path.is_file():
        link_paths = {
            prim.GetName(): str(prim.GetPath())
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(root_prim_path)
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        }
        missing_srdf_links: set[str] = set()
        for element in ET.parse(srdf_path).getroot().findall("disable_collisions"):
            link1 = str(element.attrib["link1"])
            link2 = str(element.attrib["link2"])
            if link1 not in link_paths or link2 not in link_paths:
                missing_srdf_links.update(
                    link for link in (link1, link2) if link not in link_paths
                )
                continue
            prim1 = stage.GetPrimAtPath(link_paths[link1])
            UsdPhysics.FilteredPairsAPI.Apply(prim1).CreateFilteredPairsRel().AddTarget(
                Sdf.Path(link_paths[link2])
            )
            allowed_self_collision_pairs.append(
                {
                    "link1": link1,
                    "link2": link2,
                    "actor1": link_paths[link1],
                    "actor2": link_paths[link2],
                    "reason": str(element.attrib.get("reason", "SRDF")),
                }
            )
        if missing_srdf_links:
            raise RuntimeError(
                f"SRDF collision links are absent from imported articulation: {sorted(missing_srdf_links)}"
            )
        srdf_filter_complete = True

    base_position = np.asarray(metadata.get("base_position_m", [0.0, 0.0, 0.0]), dtype=float)
    base_rpy = list(metadata.get("base_rpy_rad", [0.0, 0.0, 0.0]))
    base_xform = UsdGeom.XformCommonAPI(root_prim)
    base_xform.SetTranslate(Gf.Vec3d(*base_position.tolist()))
    base_xform.SetRotate(
        Gf.Vec3f(*(np.degrees(np.asarray(base_rpy, dtype=float))).tolist()),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )

    scene_primitives = list(metadata.get("scene_primitives", []))
    explicit_floor_declared = any(
        str(item.get("category", "")) == "floor"
        or str(item.get("name", "")) in {"trailer_floor", "physical_floor_reachable_patch"}
        for item in scene_primitives
    )
    synthetic_ground_created = not explicit_floor_declared
    if synthetic_ground_created:
        ground = UsdGeom.Cube.Define(stage, "/Validation/Ground")
        ground.CreateSizeAttr(1.0)
        ground_xform = UsdGeom.XformCommonAPI(ground.GetPrim())
        ground_xform.SetScale(Gf.Vec3f(20.0, 20.0, 0.1))
        ground_xform.SetTranslate(Gf.Vec3d(0.0, 0.0, -0.05))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    UsdGeom.Xform.Define(stage, "/Validation/Scene")
    target_carton_path = None
    target_primitive = None
    rendering_cfg = dict(metadata.get("rendering", {}))
    conveyor_cfg = dict(metadata.get("conveyor", {}))
    conveyor_enabled = bool(conveyor_cfg.get("enabled", False))
    conveyor_speed_m_s = float(conveyor_cfg.get("speed_m_s", 0.0))
    conveyor_start_policy = str(conveyor_cfg.get("start_policy", "immediate"))
    if conveyor_start_policy not in {"immediate", "after_release", "after_release_retreat"}:
        raise ValueError(
            "conveyor start_policy must be immediate, after_release, or after_release_retreat"
        )
    if conveyor_enabled and (not math.isfinite(conveyor_speed_m_s) or conveyor_speed_m_s <= 0.0):
        raise ValueError("enabled conveyor speed_m_s must be finite and positive")
    conveyor_directions_world = {}
    for name, value in dict(conveyor_cfg.get("surface_directions_world", {})).items():
        direction = np.asarray(value, dtype=float)
        if direction.shape != (3,) or not np.all(np.isfinite(direction)):
            raise ValueError(f"conveyor direction for {name} must contain three finite values")
        magnitude = float(np.linalg.norm(direction))
        if magnitude <= 1e-12:
            raise ValueError(f"conveyor direction for {name} must be nonzero")
        conveyor_directions_world[str(name)] = direction / magnitude
    if conveyor_enabled and not conveyor_directions_world:
        raise ValueError("enabled conveyor requires surface_directions_world")
    conveyor_surface_paths = {}
    conveyor_surface_enabled_attrs = {}
    conveyor_primitives = {}
    palette = dict(rendering_cfg.get("material_palette", {}))
    pbr_specs = dict(rendering_cfg.get("pbr_materials", {}))

    def _color(name, fallback):
        value = np.asarray(palette.get(f"{name}_rgb", fallback), dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"rendering material color for {name} must contain three numbers")
        return Gf.Vec3f(*np.clip(value, 0.0, 1.0).tolist())

    category_colors = {
        "trailer": _color("trailer", (0.20, 0.23, 0.27)),
        "static": _color("static", (0.08, 0.22, 0.36)),
        "amr": _color("amr", (0.055, 0.065, 0.075)),
        "carton": _color("carton", (0.47, 0.25, 0.095)),
    }
    UsdGeom.Xform.Define(stage, "/Validation/Materials")
    category_materials = {}
    for category in ("trailer", "carton"):
        spec = dict(pbr_specs.get(category, {}))
        if spec:
            spec.setdefault("fallback_rgb", list(category_colors[category]))
            category_materials[category] = _preview_material(
                f"/Validation/Materials/{category.title()}", spec
            )
    physics_materials = {}
    physics_material_specs = dict(physics_contract.get("materials", {}))
    physics_material_by_category = dict(
        physics_contract.get("material_by_category", {})
    )
    if metadata.get("robot_model") == "fanuc_m710id_70":
        if not physics_material_specs or not physics_material_by_category:
            raise ValueError("M-710 replay requires physics materials and category bindings")
        UsdGeom.Xform.Define(stage, "/Validation/PhysicsMaterials")
        for material_name, coefficients in physics_material_specs.items():
            material = UsdShade.Material.Define(
                stage,
                f"/Validation/PhysicsMaterials/{_safe_prim_name(str(material_name))}",
            )
            material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            material_api.CreateStaticFrictionAttr().Set(
                float(coefficients["static_friction"])
            )
            material_api.CreateDynamicFrictionAttr().Set(
                float(coefficients["dynamic_friction"])
            )
            material_api.CreateRestitutionAttr().Set(
                float(coefficients["restitution"])
            )
            physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
            physx_material_api.CreateFrictionCombineModeAttr().Set(
                PhysxSchema.Tokens.min
            )
            physx_material_api.CreateRestitutionCombineModeAttr().Set(
                PhysxSchema.Tokens.min
            )
            physics_materials[str(material_name)] = material

        robot_material = physics_materials.get("robot_coating")
        if robot_material is None:
            raise ValueError("M-710 physics contract is missing robot_coating")
        for prim in imported_link_prims.values():
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                robot_material,
                UsdShade.Tokens.weakerThanDescendants,
                "physics",
            )
            damping_cfg = physics_contract["damping"]["robot_links"]
            body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            body_api.CreateLinearDampingAttr().Set(
                float(damping_cfg["linear_damping_s_inv"])
            )
            body_api.CreateAngularDampingAttr().Set(
                float(damping_cfg["angular_damping_s_inv"])
            )
    dynamic_scene_prim_paths: list[str] = []
    dynamic_scene_records: list[dict[str, object]] = []
    static_scene_records: list[dict[str, object]] = []
    for primitive in scene_primitives:
        prim_path = f"/Validation/Scene/{_safe_prim_name(str(primitive['name']))}"
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr(
            [category_colors.get(str(primitive.get("category")), Gf.Vec3f(0.5, 0.5, 0.5))]
        )
        category = str(primitive.get("category"))
        if metadata.get("robot_model") == "fanuc_m710id_70":
            physics_material_name = str(
                primitive.get(
                    "material", physics_material_by_category.get(category, "")
                )
            )
            physics_material = physics_materials.get(physics_material_name)
            if physics_material is None:
                raise ValueError(
                    f"scene primitive {primitive['name']} has no audited physics material"
                )
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
                physics_material,
                UsdShade.Tokens.weakerThanDescendants,
                "physics",
            )
        if category in category_materials:
            cube.CreateDisplayOpacityAttr([0.0])
            _unit_box_visual(prim_path, category_materials[category])
            if category == "carton":
                _decal_plane(
                    prim_path,
                    image_path=str(
                        rendering_cfg.get(
                            "carton_brand_decal_path",
                            "assets/materials/industrial/decals/hxpp_logo.jpg",
                        )
                    ),
                    y_half=0.28,
                    z_min=-0.11,
                    z_max=0.11,
                )
            elif str(primitive["name"]) == "trailer_front_wall":
                _decal_plane(
                    prim_path,
                    image_path=str(
                        rendering_cfg.get(
                            "trailer_brand_decal_path",
                            "assets/materials/industrial/decals/foton_ollin_logo.png",
                        )
                    ),
                    y_half=0.18,
                    z_min=0.19,
                    z_max=0.31,
                )
        xform = UsdGeom.XformCommonAPI(cube.GetPrim())
        size = np.asarray(primitive["size_m"], dtype=float)
        center = np.asarray(primitive["center_m"], dtype=float)
        rotation = np.asarray(primitive["rotation_matrix"], dtype=float)
        xform.SetScale(Gf.Vec3f(*size.tolist()))
        xform.SetTranslate(Gf.Vec3d(*center.tolist()))
        xform.SetRotate(
            Gf.Vec3f(*_rpy_degrees_from_rotation_matrix(rotation).tolist()),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )
        primitive_name = str(primitive["name"])
        if conveyor_enabled and primitive_name in conveyor_directions_world:
            # Render the conveyor as a normal deck, but collide only against
            # its upper belt surface. A closed collision cube exposes a
            # vertical end face; cartons then snag at that artificial wall
            # instead of crossing the full-width L-junction seam.
            top_collision = UsdGeom.Mesh.Define(stage, f"{prim_path}/TopCollision")
            top_collision.CreatePointsAttr(
                [
                    Gf.Vec3f(-0.5, -0.5, 0.5),
                    Gf.Vec3f(0.5, -0.5, 0.5),
                    Gf.Vec3f(0.5, 0.5, 0.5),
                    Gf.Vec3f(-0.5, 0.5, 0.5),
                ]
            )
            top_collision.CreateFaceVertexCountsAttr([3, 3])
            top_collision.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 2, 3])
            top_collision.CreateSubdivisionSchemeAttr().Set("none")
            UsdPhysics.CollisionAPI.Apply(top_collision.GetPrim())
            # PhysX surface velocity drives contact friction while the belt
            # body itself remains kinematic. The carton remains a fully
            # dynamic rigid body; its pose is never overwritten after release.
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rigid_body_api.CreateKinematicEnabledAttr().Set(True)
            surface_velocity_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(cube.GetPrim())
            enabled_attr = surface_velocity_api.CreateSurfaceVelocityEnabledAttr()
            # Runtime ownership is selected from the payload footprint.  Start
            # every drive disabled so an exclusive L-transfer can never have
            # both orthogonal surface velocities live for even one step.
            enabled_attr.Set(False)
            surface_velocity_api.CreateSurfaceVelocityLocalSpaceAttr().Set(False)
            surface_velocity = conveyor_directions_world[primitive_name] * conveyor_speed_m_s
            surface_velocity_api.CreateSurfaceVelocityAttr().Set(
                Gf.Vec3f(*surface_velocity.tolist())
            )
            conveyor_surface_paths[primitive_name] = prim_path
            conveyor_surface_enabled_attrs[primitive_name] = enabled_attr
            conveyor_primitives[primitive_name] = primitive
        else:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if bool(primitive.get("dynamic", False)):
            mass = primitive.get("mass_kg")
            inertia = np.asarray(primitive.get("inertia_at_com_kg_m2", []), dtype=float)
            if mass is None or not np.isfinite(float(mass)) or float(mass) <= 0.0:
                raise ValueError(f"dynamic primitive {primitive_name} has no explicit positive mass")
            if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)):
                raise ValueError(f"dynamic primitive {primitive_name} has no explicit inertia tensor")
            if not np.allclose(inertia, np.diag(np.diag(inertia)), atol=1e-10, rtol=0.0):
                raise ValueError(
                    f"dynamic primitive {primitive_name} inertia requires principal-axis metadata"
                )
            UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            if metadata.get("robot_model") == "fanuc_m710id_70":
                damping_cfg = physics_contract["damping"]["cartons"]
                rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim())
                rigid_api.CreateLinearDampingAttr().Set(
                    float(damping_cfg["linear_damping_s_inv"])
                )
                rigid_api.CreateAngularDampingAttr().Set(
                    float(damping_cfg["angular_damping_s_inv"])
                )
            mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
            mass_api.CreateMassAttr(float(mass))
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.diag(inertia).tolist()))
            dynamic_scene_prim_paths.append(prim_path)
            dynamic_scene_records.append(dict(primitive))
        else:
            static_scene_records.append(dict(primitive))
        if str(primitive["name"]) == str(metadata["target"]):
            target_carton_path = prim_path
            target_primitive = primitive
    grasp_suffix = str(metadata.get("isaac_grasp_body_path_suffix", "")).strip("/")
    if not grasp_suffix:
        raise ValueError("replay metadata requires an Isaac grasp-body path suffix")
    grasp_body_path = f"{root_prim_path}/{grasp_suffix}"
    gripper_cfg = metadata.get("gripper", {})
    footprint_size = np.asarray(gripper_cfg.get("footprint_size_m", [0.30, 0.40]), dtype=float)
    flange_offset = np.asarray(
        metadata.get("flange_offset_from_grasp_body_m", []), dtype=float
    )
    if flange_offset.shape != (3,) or not np.all(np.isfinite(flange_offset)):
        raise ValueError("replay metadata requires a finite flange offset from the grasp body")
    flange_offset_x = float(flange_offset[0])
    tool_uncompressed_face_x = flange_offset_x + float(
        gripper_cfg.get("physical_uncompressed_face_from_flange_m", 0.0)
    )
    tool_contact_plane_x = flange_offset_x + float(
        gripper_cfg.get("compressed_contact_plane_from_flange_m", 0.0)
    )
    if not tool_uncompressed_face_x > tool_contact_plane_x > flange_offset_x:
        raise ValueError("physical suction face must lie beyond the flange along tool +X")
    suction_tool_path = f"{grasp_body_path}/SuctionTool"
    gripper_collision_mesh = gripper_cfg.get("collision_mesh_path")
    gripper_mesh_loaded = False
    if gripper_collision_mesh:
        gripper_mesh_path = (args.project_root.resolve() / gripper_collision_mesh).resolve()
        if not gripper_mesh_path.is_file():
            raise FileNotFoundError(f"gripper collision mesh not found: {gripper_mesh_path}")
        step_vertices_mm, triangle_indices = _load_binary_stl(gripper_mesh_path)
        flange_origin_step_mm = np.asarray(
            gripper_cfg.get("flange_origin_step_mm", []), dtype=float
        )
        step_from_tool_rotation = np.asarray(
            gripper_cfg.get("step_from_tool_rotation_matrix", []), dtype=float
        )
        if flange_origin_step_mm.shape != (3,) or step_from_tool_rotation.shape != (3, 3):
            raise ValueError("gripper mesh requires the audited STEP-to-tool transform")
        relative_step = step_vertices_mm - flange_origin_step_mm
        tool_vertices_m = (relative_step @ step_from_tool_rotation) * 1e-3
        tool_vertices_m[:, 0] += flange_offset_x
        suction_tool = UsdGeom.Mesh.Define(stage, suction_tool_path)
        suction_tool.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        suction_tool.CreatePointsAttr(
            [Gf.Vec3f(*point.tolist()) for point in tool_vertices_m]
        )
        suction_tool.CreateFaceVertexCountsAttr([3] * (len(triangle_indices) // 3))
        suction_tool.CreateFaceVertexIndicesAttr(triangle_indices.tolist())
        suction_tool.CreateDisplayColorAttr([_color("gripper", (0.16, 0.18, 0.21))])
        gripper_material = _preview_material(
            "/Validation/Materials/WantaiBrushedMetal",
            dict(
                pbr_specs.get(
                    "gripper_metal",
                    {"fallback_rgb": [0.43, 0.47, 0.50], "roughness": 0.34, "metallic": 0.82},
                )
            ),
        )
        UsdShade.MaterialBindingAPI.Apply(suction_tool.GetPrim()).Bind(gripper_material)

        # A single convex hull/envelope bridges the real gaps between the 202
        # disconnected STEP solids.  Load the offline-audited per-solid boxes
        # instead, excluding the 144 compliant cup/insert solids: those are
        # represented by SurfaceGripper points and their 15 mm compression.
        mass_properties_path = gripper_cfg.get("mass_properties_path")
        if not mass_properties_path:
            raise ValueError("gripper mass properties are required for collision proxies")
        audited_mass_path = (args.project_root.resolve() / mass_properties_path).resolve()
        with audited_mass_path.open("r", encoding="utf-8") as stream:
            audited_mass = json.load(stream)
        rigid_step_bounds = np.asarray(
            audited_mass.get("rigid_collision_bounding_boxes_step_mm", []), dtype=float
        )
        if rigid_step_bounds.ndim != 2 or rigid_step_bounds.shape[1] != 6:
            raise ValueError("audited rigid STEP solid bounds are required")
        for proxy_index, step_bounds in enumerate(rigid_step_bounds):
            step_min = step_bounds[:3]
            step_max = step_bounds[3:]
            step_corners = np.asarray(
                [
                    [x, y, z]
                    for x in (step_min[0], step_max[0])
                    for y in (step_min[1], step_max[1])
                    for z in (step_min[2], step_max[2])
                ],
                dtype=float,
            )
            tool_corners = (
                (step_corners - flange_origin_step_mm) @ step_from_tool_rotation * 1e-3
            )
            tool_corners[:, 0] += flange_offset_x
            tool_min = np.min(tool_corners, axis=0)
            tool_max = np.max(tool_corners, axis=0)
            proxy_size = tool_max - tool_min
            collision_proxy = UsdGeom.Cube.Define(
                stage,
                f"{grasp_body_path}/SuctionToolRigidCollision_{proxy_index:03d}",
            )
            collision_proxy.CreateSizeAttr(1.0)
            UsdGeom.Imageable(collision_proxy.GetPrim()).MakeInvisible()
            collision_proxy_xform = UsdGeom.XformCommonAPI(collision_proxy.GetPrim())
            collision_proxy_xform.SetScale(Gf.Vec3f(*proxy_size.tolist()))
            collision_proxy_xform.SetTranslate(
                Gf.Vec3d(*(0.5 * (tool_min + tool_max)).tolist())
            )
            if not args.disable_gripper_collision:
                UsdPhysics.CollisionAPI.Apply(collision_proxy.GetPrim())

        if metadata.get("robot_model") != "fanuc_m710id_70":
            # Legacy bundles did not provide complete link inertials. Preserve
            # their existing tool-mass authoring path; M-710 combines the fixed
            # tool into J6 before these link properties are applied, because a
            # parent rigid body's explicit mass overrides descendant masses.
            mass_api = UsdPhysics.MassAPI.Apply(suction_tool.GetPrim())
            mass_api.CreateMassAttr(float(gripper_cfg.get("gripper_mass_kg", 10.0)))
            gripper_com = np.asarray(
                gripper_cfg.get("center_of_mass_from_flange_m", [0.0, 0.0, 0.0]), dtype=float
            )
            gripper_com_mesh_frame = gripper_com + np.asarray([flange_offset_x, 0.0, 0.0])
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*gripper_com_mesh_frame.tolist()))
            gripper_inertia = np.asarray(gripper_cfg.get("inertia_at_com_kg_m2"), dtype=float)
            principal_moments, principal_axes = np.linalg.eigh(gripper_inertia)
            if np.linalg.det(principal_axes) < 0.0:
                principal_axes[:, -1] *= -1.0
            principal_quaternion = _quaternion_wxyz_from_matrix(principal_axes)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*principal_moments.tolist()))
            mass_api.CreatePrincipalAxesAttr(
                Gf.Quatf(
                    float(principal_quaternion[0]),
                    Gf.Vec3f(*principal_quaternion[1:].tolist()),
                )
            )
        gripper_mesh_loaded = True
    else:
        suction_tool = UsdGeom.Cube.Define(stage, suction_tool_path)
        suction_tool.CreateSizeAttr(1.0)
        suction_tool.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.10, 0.12)])
        suction_tool_xform = UsdGeom.XformCommonAPI(suction_tool.GetPrim())
        suction_tool_xform.SetScale(
            Gf.Vec3f(0.02, float(footprint_size[0]), float(footprint_size[1]))
        )
        suction_tool_xform.SetTranslate(
            Gf.Vec3d(tool_uncompressed_face_x - 0.01, 0.0, 0.0)
        )
        UsdPhysics.CollisionAPI.Apply(suction_tool.GetPrim())

    # The supplied STEP is the authoritative assembly geometry. These thin,
    # non-colliding cylinders only separate the 72 FG42 rubber lips visually;
    # all contact and seal mechanics remain in SurfaceGripper attachment data.
    physical_cup_centers = np.asarray(
        gripper_cfg.get("cup_centers_tool_yz_m", []), dtype=float
    )
    physical_cup_count = int(gripper_cfg.get("physical_cup_count", 0))
    if physical_cup_centers.shape == (physical_cup_count, 2):
        rubber_material = _preview_material(
            "/Validation/Materials/FG42Rubber",
            dict(
                pbr_specs.get(
                    "gripper_rubber",
                    {"fallback_rgb": [0.025, 0.028, 0.030], "roughness": 0.78},
                )
            ),
        )
        cup_radius = float(gripper_cfg.get("cup_radius_m", 0.0215))
        visual_cup_height = min(
            0.018, float(gripper_cfg.get("physical_cup_compression_m", 0.015)) + 0.003
        )
        for cup_index, (cup_y, cup_z) in enumerate(physical_cup_centers):
            cup = UsdGeom.Cylinder.Define(
                stage, f"{grasp_body_path}/FG42CupVisual_{cup_index:02d}"
            )
            cup.CreateAxisAttr(UsdGeom.Tokens.x)
            cup.CreateRadiusAttr(cup_radius)
            cup.CreateHeightAttr(visual_cup_height)
            cup_xform = UsdGeom.XformCommonAPI(cup.GetPrim())
            cup_xform.SetTranslate(
                Gf.Vec3d(
                    tool_uncompressed_face_x - 0.5 * visual_cup_height,
                    float(cup_y),
                    float(cup_z),
                )
            )
            UsdShade.MaterialBindingAPI.Apply(cup.GetPrim()).Bind(rubber_material)

    surface_gripper_paths: list[str] = []
    surface_gripper_interface = None
    surface_attachment_paths: list[str] = []
    surface_force = None
    surface_force_per_point = None
    point_force_limits = None
    surface_shear_force = None
    surface_shear_force_per_point = None
    point_shear_limits = None
    surface_shear_solver_fallback_used = False
    fixed_joint_torque_solver_fallback_used = False
    active_cup_indices = list(gripper_cfg.get("active_sealed_cup_indices", []))
    if (
        physical_cup_centers.shape != (physical_cup_count, 2)
        or not active_cup_indices
        or len(set(active_cup_indices)) != len(active_cup_indices)
        or any(index < 0 or index >= physical_cup_count for index in active_cup_indices)
    ):
        raise ValueError("physical attachment audit requires valid active cup centers")
    physical_contact_offsets = physical_cup_centers[np.asarray(active_cup_indices, dtype=int)]
    configured_shear_force = gripper_cfg.get(
        "shear_force_total_n", gripper_cfg.get("shear_force_n")
    )
    configured_holding_torque = gripper_cfg.get("holding_torque_nm")
    gripper_wrench_envelope_complete = bool(
        args.gripper_model == "fixed_joint_diagnostic"
        and configured_shear_force is not None
        and configured_holding_torque is not None
    )
    if target_carton_path is not None and args.gripper_model == "surface_gripper":
        physical_active_cup_count = int(gripper_cfg.get("active_sealed_cup_count", 0))
        point_count = int(
            args.gripper_attachment_point_count
            if args.gripper_attachment_point_count is not None
            else gripper_cfg.get("simulation_attachment_point_count", 4)
        )
        surface_force = float(
            args.gripper_force_limit
            if args.gripper_force_limit is not None
            else gripper_cfg.get("holding_force_total_n", gripper_cfg.get("holding_force_n", 1800.0))
        )
        if not np.isfinite(surface_force) or surface_force <= 0.0:
            raise ValueError("surface gripper total coaxial force must be finite and positive")
        surface_force_per_point = None
        if args.gripper_shear_force_limit is not None:
            surface_shear_force = float(args.gripper_shear_force_limit)
        elif configured_shear_force is not None:
            surface_shear_force = float(configured_shear_force)
        else:
            # PhysX requires a numeric shear limit. The product data does not
            # provide one, so use an effectively unbounded diagnostic value
            # and fail qualification through the incomplete wrench envelope.
            surface_shear_force = 3.4028235e38
            surface_shear_solver_fallback_used = True
        if not np.isfinite(surface_shear_force) or surface_shear_force <= 0.0:
            raise ValueError("surface gripper total shear force must be finite and positive")
        surface_shear_force_per_point = None
        inset = float(gripper_cfg.get("attachment_point_inset_m", 0.03))
        cup_centers = np.asarray(gripper_cfg.get("cup_centers_tool_yz_m", []), dtype=float)
        solver_offsets = np.asarray(
            gripper_cfg.get("solver_attachment_offsets_tool_yz_m", []), dtype=float
        )
        if args.gripper_attachment_point_count is None and solver_offsets.shape == (point_count, 2):
            point_offsets = [tuple(offset) for offset in solver_offsets]
        elif (
            gripper_cfg.get("attachment_model") == "per_sealed_cup"
            and cup_centers.shape == (int(gripper_cfg.get("physical_cup_count", 0)), 2)
            and active_cup_indices
            and args.gripper_attachment_point_count is None
        ):
            point_offsets = [tuple(cup_centers[index]) for index in active_cup_indices]
            point_count = len(point_offsets)
        elif point_count == 1:
            point_offsets = [(0.0, 0.0)]
        elif point_count == 4:
            half_y = 0.5 * float(footprint_size[0]) - inset
            half_z = 0.5 * float(footprint_size[1]) - inset
            point_offsets = [
                (-half_y, -half_z),
                (-half_y, half_z),
                (half_y, -half_z),
                (half_y, half_z),
            ]
        else:
            raise ValueError(
                "non-legacy attachment point counts require per-sealed-cup layout metadata"
            )
        represented_cup_counts = np.asarray(
            gripper_cfg.get("solver_attachment_cup_counts", []), dtype=float
        )
        if represented_cup_counts.shape != (len(point_offsets),):
            raise ValueError("solver attachment points require represented cup counts")
        if (
            not np.all(np.isfinite(represented_cup_counts))
            or np.any(represented_cup_counts <= 0.0)
            or not np.array_equal(represented_cup_counts, np.rint(represented_cup_counts))
            or not np.isclose(
                np.sum(represented_cup_counts),
                physical_active_cup_count,
                atol=0.0,
                rtol=0.0,
            )
        ):
            raise ValueError("solver attachment cup counts must partition the active sealed cups")
        point_force_limits = surface_force * represented_cup_counts / np.sum(represented_cup_counts)
        point_shear_limits = surface_shear_force * represented_cup_counts / np.sum(represented_cup_counts)
        surface_force_per_point = float(np.min(point_force_limits))
        surface_shear_force_per_point = float(np.min(point_shear_limits))
        for index, (offset_y, offset_z) in enumerate(point_offsets):
            surface_gripper_path = f"/Validation/SurfaceGrippers/Gripper_{index}"
            surface_gripper_prim = robot_schema.CreateSurfaceGripper(
                stage, surface_gripper_path
            )
            surface_gripper_prim.GetAttribute(
                robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name
            ).Set(float(point_force_limits[index]))
            surface_gripper_prim.GetAttribute(
                robot_schema.Attributes.SHEAR_FORCE_LIMIT.name
            ).Set(float(point_shear_limits[index]))
            surface_gripper_prim.GetAttribute(
                robot_schema.Attributes.MAX_GRIP_DISTANCE.name
            ).Set(float(gripper_cfg.get("max_grip_distance_m", 0.03)))
            surface_gripper_prim.GetAttribute(
                robot_schema.Attributes.RETRY_INTERVAL.name
            ).Set(0.0)
            attachment_path = f"/Validation/SurfaceAttachmentPoints/Point_{index}"
            attachment = UsdPhysics.Joint.Define(stage, attachment_path)
            attachment_prim = attachment.GetPrim()
            robot_schema.ApplyAttachmentPointAPI(attachment_prim)
            attachment_prim.GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name).Set(
                UsdPhysics.Tokens.x
            )
            attachment_prim.GetAttribute(robot_schema.Attributes.CLEARANCE_OFFSET.name).Set(0.0)
            for limit_name in ("rotX", "rotY", "rotZ", "transX", "transY", "transZ"):
                limit_api = UsdPhysics.LimitAPI.Apply(attachment_prim, limit_name)
                limit_api.CreateHighAttr().Set(-1.0)
                limit_api.CreateLowAttr().Set(1.0)
            attachment.CreateBody0Rel().SetTargets([Sdf.Path(grasp_body_path)])
            attachment.CreateLocalPos0Attr().Set(
                Gf.Vec3f(tool_contact_plane_x, float(offset_y), float(offset_z))
            )
            attachment.CreateLocalRot0Attr().Set(
                Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
            )
            attachment.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            attachment.CreateLocalRot1Attr().Set(
                Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
            )
            attachment.CreateExcludeFromArticulationAttr(True)
            surface_attachment_paths.append(attachment_path)
            surface_gripper_prim.GetRelationship(
                robot_schema.Relations.ATTACHMENT_POINTS.name
            ).SetTargets([Sdf.Path(attachment_path)])
            surface_gripper_paths.append(surface_gripper_path)
        surface_gripper_interface = acquire_surface_gripper_interface()

    released_payload_path = None
    released_payload_physx_api = None
    if (
        target_carton_path is not None
        and target_primitive is not None
        and args.gripper_model == "fixed_joint_diagnostic"
    ):
        released_payload_path = "/Validation/ReleasedPayload"
        released_cube = UsdGeom.Cube.Define(stage, released_payload_path)
        released_cube.CreateSizeAttr(1.0)
        released_cube.CreateDisplayColorAttr([category_colors["carton"]])
        released_xform = UsdGeom.XformCommonAPI(released_cube.GetPrim())
        released_size = np.asarray(target_primitive["size_m"], dtype=float)
        released_rotation = np.asarray(target_primitive["rotation_matrix"], dtype=float)
        released_xform.SetScale(Gf.Vec3f(*released_size.tolist()))
        released_xform.SetTranslate(Gf.Vec3d(100.0, 100.0, 0.0))
        released_xform.SetRotate(
            Gf.Vec3f(*_rpy_degrees_from_rotation_matrix(released_rotation).tolist()),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )
        UsdPhysics.CollisionAPI.Apply(released_cube.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(released_cube.GetPrim())
        UsdPhysics.MassAPI.Apply(released_cube.GetPrim()).CreateMassAttr(
            float(target_primitive.get("mass_kg", 7.0))
        )
        released_payload_physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(released_cube.GetPrim())
        released_payload_physx_api.CreateDisableGravityAttr(True)
        source_payload_prim = stage.GetPrimAtPath(target_carton_path)
        UsdPhysics.FilteredPairsAPI.Apply(source_payload_prim).CreateFilteredPairsRel().AddTarget(
            Sdf.Path(released_payload_path)
        )
    print(
        f"FANUC_REPLAY_STAGE=scene_created primitives={len(scene_primitives)} ",
        f"target_carton={target_carton_path}",
        flush=True,
    )

    grasp_joint = None
    grasp_joint_path = "/Validation/GraspJoint"

    contact_pairs: dict[tuple[str, str], dict[str, float | int]] = {}
    contact_clock_s = [0.0]
    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        monitor_payload = prim_path in {target_carton_path, released_payload_path}
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and (
            prim_path.startswith(root_prim_path) or monitor_payload
        ):
            contact_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contact_api.CreateThresholdAttr(0.0)

    def _on_contact_report(headers, _contact_data):
        for header in headers:
            actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            robot_involved = actor0.startswith(root_prim_path) or actor1.startswith(root_prim_path)
            payload_paths = {path for path in (target_carton_path, released_payload_path) if path}
            payload_involved = actor0 in payload_paths or actor1 in payload_paths
            if not robot_involved and not payload_involved:
                continue
            pair = tuple(sorted((actor0, actor1)))
            record = contact_pairs.setdefault(
                pair,
                {
                    "event_count": 0,
                    "peak_impulse_ns": 0.0,
                    "first_contact_time_s": contact_clock_s[0],
                    "last_contact_time_s": contact_clock_s[0],
                },
            )
            record["event_count"] = int(record["event_count"]) + 1
            record["last_contact_time_s"] = contact_clock_s[0]
            try:
                impulse = float(np.linalg.norm(np.asarray(header.total_impulse, dtype=float)))
            except (AttributeError, TypeError, ValueError):
                impulse = 0.0
            record["peak_impulse_ns"] = max(float(record["peak_impulse_ns"]), impulse)

    contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(
        _on_contact_report
    )

    camera_cfg = metadata.get("camera", {})
    camera_target = list(
        camera_cfg.get("target_m", (base_position + np.array([0.5, 0.0, 0.9])).tolist())
    )
    camera_position = list(
        camera_cfg.get("eye_m", (base_position + np.array([3.8, 3.2, 2.4])).tolist())
    )
    horizontal_fov = float(camera_cfg.get("horizontal_fov_rad", np.deg2rad(60.0)))
    horizontal_aperture_mm = float(camera_cfg.get("horizontal_aperture_mm", 20.955))
    if not (0.0 < horizontal_fov < np.pi) or horizontal_aperture_mm <= 0.0:
        raise ValueError("camera horizontal FOV and aperture must be positive and finite")
    focal_length_mm = horizontal_aperture_mm / (2.0 * np.tan(0.5 * horizontal_fov))
    camera = rep.create.camera(
        position=tuple(camera_position),
        look_at=tuple(camera_target),
        focal_length=float(focal_length_mm),
        horizontal_aperture=float(horizontal_aperture_mm),
        clipping_range=(
            float(camera_cfg.get("near_m", 0.03)),
            float(camera_cfg.get("far_m", 10.0)),
        ),
    )
    def _render_vector(name, fallback):
        value = np.asarray(rendering_cfg.get(name, fallback), dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"rendering.{name} must contain three finite numbers")
        return tuple(value.tolist())

    rep.create.light(
        light_type="distant",
        intensity=float(rendering_cfg.get("distant_intensity", 700.0)),
        rotation=(315.0, 0.0, 20.0),
    )
    # Two broad work lights illuminate the trailer interior and the transfer
    # area.  Their positions are fixed in world coordinates for deterministic
    # replay and do not affect physics.
    rep.create.light(
        light_type="sphere",
        position=_render_vector("door_work_light_position_m", (-0.60, -0.20, 2.35)),
        intensity=float(rendering_cfg.get("door_work_light_intensity", 8000.0)),
        color=_render_vector("door_work_light_color_rgb", (1.0, 0.91, 0.80)),
        scale=0.8,
    )
    rep.create.light(
        light_type="sphere",
        position=_render_vector("trailer_work_light_position_m", (1.35, 0.20, 2.30)),
        intensity=float(rendering_cfg.get("trailer_work_light_intensity", 5500.0)),
        color=_render_vector("trailer_work_light_color_rgb", (0.78, 0.88, 1.0)),
        scale=0.65,
    )
    render_product = rep.create.render_product(camera, (args.width, args.height))
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    rgb_annotator.attach(render_product)
    depth_annotator.attach(render_product)

    physics_dt = 1.0 / physics_hz
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt * args.render_every)
    if metadata.get("robot_model") == "fanuc_m710id_70":
        gravity = np.asarray(physics_contract.get("gravity_world_m_s2", []), dtype=float)
        if gravity.shape != (3,) or not np.all(np.isfinite(gravity)):
            raise ValueError("M-710 physics contract requires a finite gravity vector")
        gravity_magnitude = float(np.linalg.norm(gravity))
        if gravity_magnitude <= 0.0:
            raise ValueError("M-710 gravity magnitude must be positive")
        physics_scenes = [
            UsdPhysics.Scene(prim)
            for prim in stage.Traverse()
            if prim.IsA(UsdPhysics.Scene)
        ]
        if len(physics_scenes) != 1:
            raise RuntimeError(
                f"M-710 replay requires exactly one USD physics scene, got {len(physics_scenes)}"
            )
        physics_scenes[0].CreateGravityDirectionAttr().Set(
            Gf.Vec3f(*(gravity / gravity_magnitude).tolist())
        )
        physics_scenes[0].CreateGravityMagnitudeAttr().Set(gravity_magnitude)
    articulation = Articulation(root_prim_path)
    grasp_body = RigidPrim(grasp_body_path) if target_carton_path is not None else None
    target_body = RigidPrim(target_carton_path) if target_carton_path is not None else None
    dynamic_scene_bodies = [RigidPrim(path) for path in dynamic_scene_prim_paths]
    released_payload_body = (
        RigidPrim(released_payload_path) if released_payload_path is not None else None
    )
    discovered_joint_names = list(articulation.dof_names)
    if set(discovered_joint_names) != set(expected_joint_names) or len(discovered_joint_names) != len(expected_joint_names):
        raise RuntimeError(
            f"FANUC articulation DOFs differ from replay contract: {discovered_joint_names}"
        )
    source_index = {name: index for index, name in enumerate(expected_joint_names)}
    command_order = np.asarray([source_index[name] for name in discovered_joint_names], dtype=int)

    solver_position_iterations = int(
        physics_contract.get(
            "solver_position_iterations",
            gripper_cfg.get("solver_position_iterations", 32),
        )
    )
    solver_velocity_iterations = int(
        physics_contract.get(
            "solver_velocity_iterations",
            gripper_cfg.get("solver_velocity_iterations", 8),
        )
    )
    if solver_position_iterations <= 0 or solver_velocity_iterations <= 0:
        raise ValueError("PhysX solver iteration counts must be positive")
    if metadata.get("robot_model") == "fanuc_m710id_70" and (
        solver_position_iterations != int(gripper_cfg.get("solver_position_iterations", -1))
        or solver_velocity_iterations != int(gripper_cfg.get("solver_velocity_iterations", -1))
    ):
        raise ValueError("M-710 rigid-body and attachment solver iterations disagree")
    # SurfaceGripper couples the last articulation link to a dynamic rigid
    # carton. Raise both sides of that constraint above PhysX defaults so the
    # rigid transform converges without adding redundant attachment joints.
    # These are numerical accuracy settings, not hardware capability limits.
    solver_prims = [
        (
            root_prim,
            "physxArticulation:solverPositionIterationCount",
            "physxArticulation:solverVelocityIterationCount",
        ),
        *[
            (
                stage.GetPrimAtPath(path),
                "physxRigidBody:solverPositionIterationCount",
                "physxRigidBody:solverVelocityIterationCount",
            )
            for path in dynamic_scene_prim_paths
        ],
    ]
    if metadata.get("robot_model") != "fanuc_m710id_70" and target_carton_path:
        solver_prims.append(
            (
                stage.GetPrimAtPath(target_carton_path),
                "physxRigidBody:solverPositionIterationCount",
                "physxRigidBody:solverVelocityIterationCount",
            )
        )
    for prim, position_attribute, velocity_attribute in solver_prims:
        if prim is None or not prim.IsValid():
            continue
        position_attr = prim.GetAttribute(position_attribute)
        if not position_attr.IsValid():
            position_attr = prim.CreateAttribute(
                position_attribute, Sdf.ValueTypeNames.Int, custom=False
            )
        velocity_attr = prim.GetAttribute(velocity_attribute)
        if not velocity_attr.IsValid():
            velocity_attr = prim.CreateAttribute(
                velocity_attribute, Sdf.ValueTypeNames.Int, custom=False
            )
        position_attr.Set(solver_position_iterations)
        velocity_attr.Set(solver_velocity_iterations)

    world.reset()
    velocity_limits = np.asarray(metadata["joint_velocity_limits_rad_s"], dtype=np.float32)[command_order]
    effort_limits = np.asarray(metadata.get("joint_effort_limits_nm", []), dtype=np.float32)
    if effort_limits.size == len(expected_joint_names):
        effort_limits = effort_limits[command_order]
        articulation.set_dof_max_efforts(effort_limits[None, :])
    articulation.set_dof_max_velocities(velocity_limits[None, :])
    # These well-damped gains avoid payload shake.  Contact preload belongs
    # in the geometric grasp plan; increasing J3 stiffness to manufacture
    # contact clearance changed the post-release posture enough for a carton
    # on the cross conveyor to graze J2.
    stiffness = np.asarray(metadata.get("joint_drive_stiffness_nm_rad", []), dtype=np.float32)
    damping = np.asarray(metadata.get("joint_drive_damping_nm_s_rad", []), dtype=np.float32)
    if stiffness.shape != (len(expected_joint_names),) or damping.shape != (len(expected_joint_names),):
        raise ValueError("replay metadata requires one finite drive stiffness and damping per joint")
    if not np.all(np.isfinite(stiffness)) or not np.all(np.isfinite(damping)) or np.any(stiffness <= 0.0) or np.any(damping <= 0.0):
        raise ValueError("joint drive gains must be finite and positive")
    stiffness = stiffness[command_order]
    damping = damping[command_order]
    articulation.set_dof_gains(stiffness[None, :], damping[None, :])
    articulation.switch_dof_control_mode("position")

    initial = positions[0, command_order].astype(np.float32)
    articulation.set_dof_positions(initial[None, :])
    articulation.set_dof_position_targets(initial[None, :])
    settling_audit = {
        "status": "NOT_REQUIRED_LEGACY",
        "dynamic_body_count": len(dynamic_scene_bodies),
        "elapsed_s": 0.0,
    }
    if metadata.get("robot_model") == "fanuc_m710id_70":
        if len(dynamic_scene_bodies) != 40 or len(dynamic_scene_records) != 40:
            raise RuntimeError("M-710 initial-state settling requires all 40 dynamic cartons")
        settling_cfg = dict(physics_contract.get("settling", {}))
        maximum_settle_time = float(settling_cfg["maximum_settle_time_s"])
        required_stable_duration = float(settling_cfg["required_stable_duration_s"])
        max_linear_speed = float(settling_cfg["max_linear_speed_m_s"])
        max_angular_speed = float(settling_cfg["max_angular_speed_rad_s"])
        max_position_drift = float(settling_cfg["max_position_drift_m"])
        max_penetration = float(settling_cfg["max_penetration_m"])
        maximum_steps = int(math.ceil(maximum_settle_time / physics_dt))
        required_stable_steps = int(math.ceil(required_stable_duration / physics_dt))
        stable_steps = 0
        stable_reference_positions = None
        configured_positions = np.asarray(
            [record["center_m"] for record in dynamic_scene_records], dtype=float
        )
        final_linear_speed = float("inf")
        final_angular_speed = float("inf")
        final_position_drift = float("inf")
        final_stable_window_drift = float("inf")
        final_penetration = float("inf")
        peak_penetration = 0.0
        peak_position_drift = 0.0
        settled_step = None
        for settle_step in range(maximum_steps):
            articulation.set_dof_position_targets(initial[None, :])
            world.step(render=settle_step % args.render_every == 0)
            carton_states = []
            linear_speeds = []
            angular_speeds = []
            for body, record in zip(dynamic_scene_bodies, dynamic_scene_records):
                body_positions, body_orientations = body.get_world_poses()
                linear_velocities, angular_velocities = body.get_velocities()
                center = np.asarray(body_positions.numpy(), dtype=float)[0]
                quaternion = np.asarray(body_orientations.numpy(), dtype=float)[0]
                rotation = _rotation_matrix_from_quaternion_wxyz(quaternion)
                half_extents = 0.5 * np.asarray(record["size_m"], dtype=float)
                carton_states.append((center, half_extents, rotation))
                linear_speeds.append(
                    float(np.linalg.norm(np.asarray(linear_velocities.numpy(), dtype=float)[0]))
                )
                angular_speeds.append(
                    float(np.linalg.norm(np.asarray(angular_velocities.numpy(), dtype=float)[0]))
                )
            current_positions = np.asarray([item[0] for item in carton_states], dtype=float)
            current_penetration = 0.0
            for first_index, first in enumerate(carton_states):
                for second in carton_states[first_index + 1 :]:
                    current_penetration = max(
                        current_penetration,
                        _obb_penetration_depth(*first, *second),
                    )
                for static_record in static_scene_records:
                    current_penetration = max(
                        current_penetration,
                        _obb_penetration_depth(
                            *first,
                            np.asarray(static_record["center_m"], dtype=float),
                            0.5 * np.asarray(static_record["size_m"], dtype=float),
                            np.asarray(static_record["rotation_matrix"], dtype=float),
                        ),
                    )
            final_linear_speed = max(linear_speeds, default=0.0)
            final_angular_speed = max(angular_speeds, default=0.0)
            final_penetration = current_penetration
            peak_penetration = max(peak_penetration, current_penetration)
            final_position_drift = float(
                np.max(np.linalg.norm(current_positions - configured_positions, axis=1))
            )
            peak_position_drift = max(peak_position_drift, final_position_drift)
            velocities_stable = (
                final_linear_speed <= max_linear_speed
                and final_angular_speed <= max_angular_speed
            )
            if (
                not velocities_stable
                or current_penetration > max_penetration
                or final_position_drift > max_position_drift
            ):
                stable_steps = 0
                stable_reference_positions = None
                final_stable_window_drift = float("inf")
                continue
            if stable_reference_positions is None:
                stable_reference_positions = current_positions.copy()
            final_stable_window_drift = float(
                np.max(np.linalg.norm(current_positions - stable_reference_positions, axis=1))
            )
            if final_stable_window_drift > max_position_drift:
                stable_steps = 0
                stable_reference_positions = current_positions.copy()
                continue
            stable_steps += 1
            if stable_steps >= required_stable_steps:
                settled_step = settle_step
                break
        settling_passed = bool(
            settled_step is not None
            and peak_penetration <= max_penetration
            and peak_position_drift <= max_position_drift
        )
        settling_audit = {
            "status": "PASS" if settling_passed else "FAIL",
            "dynamic_body_count": len(dynamic_scene_bodies),
            "elapsed_s": (
                maximum_steps * physics_dt
                if settled_step is None
                else (settled_step + 1) * physics_dt
            ),
            "stable_duration_s": stable_steps * physics_dt,
            "max_linear_speed_m_s": final_linear_speed,
            "max_angular_speed_rad_s": final_angular_speed,
            "position_drift_from_configured_m": final_position_drift,
            "stable_window_position_drift_m": final_stable_window_drift,
            "peak_position_drift_from_configured_m": peak_position_drift,
            "current_max_penetration_m": final_penetration,
            "peak_settling_penetration_m": peak_penetration,
            "thresholds": settling_cfg,
        }
        print(
            "FANUC_REPLAY_STAGE=initial_cartons_settled "
            + json.dumps(settling_audit, sort_keys=True),
            flush=True,
        )
        if not settling_passed:
            raise RuntimeError(
                "M-710 initial 40-carton state failed the bounded settling/penetration gate"
            )
    else:
        for _ in range(10):
            world.step(render=True)
    initial_target_center = None
    if target_body is not None:
        target_positions, _ = target_body.get_world_poses()
        initial_target_center = np.asarray(target_positions.numpy(), dtype=float)[0]

    requested_duration = float(timestamps[-1])
    replay_duration = requested_duration
    if target_carton_path is not None and metadata.get("release_time_seconds") is not None:
        replay_duration += float(args.post_release_seconds)
    if args.max_sim_seconds is not None:
        replay_duration = min(replay_duration, float(args.max_sim_seconds))
    physics_steps = int(math.ceil(replay_duration / physics_dt)) + 1
    measured_rows: list[np.ndarray] = []
    commanded_rows: list[np.ndarray] = []
    projected_force_rows: list[np.ndarray] = []
    gravity_force_rows: list[np.ndarray] = []
    replay_frames: list[Image.Image] = []
    replay_video_path = args.output / "replay.mp4"
    replay_video_writer = None
    replay_video_frame_count = 0
    preview_video_path = args.output / f"replay_{args.video_preview_speed:g}x.mp4"
    preview_video_writer = None
    if args.record_video:
        replay_video_writer = cv2.VideoWriter(
            str(replay_video_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(physics_hz / args.render_every),
            (args.width, args.height),
        )
        if not replay_video_writer.isOpened():
            raise RuntimeError("OpenCV could not open the MP4 replay writer")
        if args.video_preview_speed > 1.0:
            preview_video_writer = cv2.VideoWriter(
                str(preview_video_path),
                cv2.VideoWriter_fourcc(*"mp4v"),
                float(physics_hz / args.render_every * args.video_preview_speed),
                (args.width, args.height),
            )
            if not preview_video_writer.isOpened():
                raise RuntimeError("OpenCV could not open the accelerated MP4 preview writer")
    measured_times: list[float] = []
    replay_started_at = time.perf_counter()
    grasp_commanded = False
    grasp_command_succeeded = False
    grasp_enabled = False
    grasp_closed_time = None
    surface_grip_lost_time = None
    minimum_active_gripper_count = None
    release_commanded = False
    release_executed = False
    release_command_succeeded = False
    release_open_confirmed = False
    release_velocity_sample_pending = False
    release_linear_velocity_before_m_s = None
    release_angular_velocity_before_rad_s = None
    release_linear_velocity_after_m_s = None
    release_angular_velocity_after_rad_s = None
    break_force = None
    break_torque = None
    target_center_at_release = None
    grasp_frame_position_error_m = None
    grasp_frame_rotation_error_rad = None
    attachment_raycast_distances_m: list[float | None] = []
    physical_contact_audit = None
    grasp_local_position = None
    grasp_local_quaternion = None
    peak_payload_attachment_position_error_m = 0.0
    peak_payload_attachment_rotation_error_rad = 0.0
    grasp_event_time = metadata.get("grasp_time_seconds")
    release_event_time = metadata.get("release_time_seconds")
    expected_place_center = np.asarray(metadata.get("place_center_m", []), dtype=float)
    place_surface = str(metadata.get("place_surface") or "")
    conveyor_initial_direction_world = conveyor_directions_world.get(place_surface)
    conveyor_exclusive = bool(
        conveyor_cfg.get("exclusive_surface_drive_at_transfer", False)
    )
    conveyor_landing_capture_delay_s = float(
        conveyor_cfg.get("landing_capture_delay_s", 0.08)
    )
    conveyor_landing_height_tolerance_m = float(
        conveyor_cfg.get("landing_height_tolerance_m", 0.04)
    )
    conveyor_transport_minimum_distance_m = float(
        conveyor_cfg.get("transport_minimum_distance_m", 0.10)
    )
    conveyor_transport_speed_tolerance_m_s = float(
        conveyor_cfg.get("transport_speed_tolerance_m_s", 0.15)
    )
    conveyor_transport_audit_window_s = float(
        conveyor_cfg.get("transport_audit_window_seconds", 1.0)
    )
    if not np.isfinite(conveyor_transport_audit_window_s) or conveyor_transport_audit_window_s <= 0.0:
        raise ValueError("conveyor transport audit window must be finite and positive")
    target_landing_center = None
    target_landing_time_s = None
    conveyor_transport_samples: list[
        tuple[float, np.ndarray, np.ndarray, str | None]
    ] = []
    active_conveyor_surfaces: tuple[str, ...] = ()
    active_conveyor_surface: str | None = None
    conveyor_surface_history: list[dict[str, object]] = []
    conveyor_selection_initialized = False
    conveyor_start_event_time = (
        0.0
        if conveyor_start_policy == "immediate"
        else metadata.get("release_time_seconds")
        if conveyor_start_policy == "after_release"
        else metadata.get("release_retreat_time_seconds")
    )
    if conveyor_enabled and conveyor_start_event_time is None:
        raise ValueError(
            f"{conveyor_start_policy} conveyor requires its corresponding release event time"
        )
    conveyor_started = bool(conveyor_enabled and conveyor_start_policy == "immediate")
    conveyor_started_time_s = 0.0 if conveyor_started else None

    def _apply_conveyor_surface_selection(
        desired: tuple[str, ...], simulation_time_s: float
    ) -> None:
        global active_conveyor_surfaces, active_conveyor_surface
        if desired == active_conveyor_surfaces:
            return
        unknown = set(desired) - set(conveyor_surface_enabled_attrs)
        if unknown:
            raise ValueError(f"selected unknown conveyor surfaces: {sorted(unknown)}")
        if conveyor_exclusive and len(desired) > 1:
            raise RuntimeError("exclusive conveyor policy selected more than one surface")
        # A transfer is explicitly break-before-make: every drive is disabled
        # before the next owner is enabled, so both orthogonal directions are
        # never active in the same physics step.
        for enabled_attr in conveyor_surface_enabled_attrs.values():
            enabled_attr.Set(False)
        for surface_name in desired:
            conveyor_surface_enabled_attrs[surface_name].Set(True)
        active_conveyor_surfaces = desired
        active_conveyor_surface = desired[0] if len(desired) == 1 else None
        conveyor_surface_history.append(
            {
                "time_s": float(simulation_time_s),
                "active_surfaces": list(desired),
            }
        )

    for step in range(physics_steps):
        simulation_time = min(step * physics_dt, replay_duration)
        contact_clock_s[0] = simulation_time
        if (
            conveyor_enabled
            and not conveyor_started
            and simulation_time >= float(conveyor_start_event_time)
        ):
            conveyor_started = True
            conveyor_started_time_s = simulation_time
            print(
                f"FANUC_REPLAY_EVENT=conveyor_started time_s={simulation_time:.6f} "
                f"speed_m_s={conveyor_speed_m_s:.6f}",
                flush=True,
            )
        if conveyor_enabled:
            payload_center_for_drive = None
            if conveyor_started and target_body is not None:
                drive_positions, _ = target_body.get_world_poses()
                payload_center_for_drive = np.asarray(
                    drive_positions.numpy(), dtype=float
                )[0]
            desired_surfaces = select_active_conveyor_surfaces(
                payload_center_m=payload_center_for_drive,
                conveyor_primitives=conveyor_primitives,
                started=conveyor_started,
                exclusive=conveyor_exclusive,
                current_surface=active_conveyor_surface,
                preferred_initial_surface=(
                    place_surface if not conveyor_selection_initialized else None
                ),
            )
            _apply_conveyor_surface_selection(desired_surfaces, simulation_time)
            if conveyor_started:
                conveyor_selection_initialized = True
        if (
            target_body is not None
            and not grasp_commanded
            and grasp_event_time is not None
            and simulation_time >= float(grasp_event_time)
        ):
            body_positions, body_orientations = grasp_body.get_world_poses()
            carton_positions, carton_orientations = target_body.get_world_poses()
            body_position = np.asarray(body_positions.numpy(), dtype=float)[0]
            carton_position = np.asarray(carton_positions.numpy(), dtype=float)[0]
            body_quaternion = np.asarray(body_orientations.numpy(), dtype=float)[0]
            carton_quaternion = np.asarray(carton_orientations.numpy(), dtype=float)[0]
            body_rotation = Gf.Rotation(
                Gf.Quatd(float(body_quaternion[0]), Gf.Vec3d(*body_quaternion[1:].tolist()))
            )
            carton_rotation = Gf.Rotation(
                Gf.Quatd(float(carton_quaternion[0]), Gf.Vec3d(*carton_quaternion[1:].tolist()))
            )
            local_position = body_rotation.GetInverse().TransformDir(
                Gf.Vec3d(*(carton_position - body_position).tolist())
            )
            body_quaternion /= np.linalg.norm(body_quaternion)
            carton_quaternion /= np.linalg.norm(carton_quaternion)
            local_quaternion_array = _quaternion_multiply_wxyz(
                _quaternion_conjugate_wxyz(body_quaternion), carton_quaternion
            )
            local_quaternion_array /= np.linalg.norm(local_quaternion_array)
            grasp_local_position = local_position
            grasp_local_quaternion = local_quaternion_array
            reconstructed_position = body_position + np.asarray(
                body_rotation.TransformDir(local_position), dtype=float
            )
            grasp_frame_position_error_m = float(
                np.linalg.norm(np.asarray(reconstructed_position, dtype=float) - carton_position)
            )
            reconstructed_quaternion = _quaternion_multiply_wxyz(
                body_quaternion, local_quaternion_array
            )
            rotation_delta = _quaternion_multiply_wxyz(
                _quaternion_conjugate_wxyz(reconstructed_quaternion), carton_quaternion
            )
            grasp_frame_rotation_error_rad = _quaternion_angle_wxyz(rotation_delta)
            # Audit the real, nominally compressed cup plane before asking
            # PhysX to close anything.  Reaching the virtual task TCP alone is
            # never authority to create an attachment.
            body_rotation_matrix = _rotation_matrix_from_quaternion_wxyz(body_quaternion)
            carton_rotation_matrix = _rotation_matrix_from_quaternion_wxyz(carton_quaternion)
            physical_contact_audit = audit_surface_attachment_contact(
                grasp_body_position_m=body_position,
                grasp_body_rotation=body_rotation_matrix,
                contact_plane_from_grasp_body_m=tool_contact_plane_x,
                active_cup_offsets_yz_m=physical_contact_offsets,
                target_center_m=carton_position,
                target_rotation=carton_rotation_matrix,
                target_size_m=target_primitive["size_m"],
                max_attachment_gap_m=float(gripper_cfg["max_grip_distance_m"]),
                max_normal_misalignment_rad=float(
                    gripper_cfg["max_normal_misalignment_rad"]
                ),
                maximum_penetration_m=float(
                    gripper_cfg["maximum_contact_penetration_m"]
                ),
            )
            attachment_raycast_distances_m = list(physical_contact_audit.signed_gaps_m)
            if not physical_contact_audit.accepted:
                grasp_command_succeeded = False
            elif args.gripper_model == "surface_gripper":
                close_results = [
                    bool(surface_gripper_interface.close_gripper(path))
                    for path in surface_gripper_paths
                ]
                grasp_command_succeeded = bool(close_results and all(close_results))
            else:
                # Legacy diagnostic adapter retained only for controlled
                # comparison with earlier FixedJoint evidence.
                break_force = (
                    3.4028235e38
                    if args.disable_gripper_break_limits
                    else float(
                        args.gripper_force_limit
                        if args.gripper_force_limit is not None
                        else gripper_cfg.get("holding_force_n", 1800.0)
                    )
                )
                break_torque = (
                    3.4028235e38
                    if args.disable_gripper_break_limits
                    else (
                        float(args.gripper_torque_limit)
                        if args.gripper_torque_limit is not None
                        else (
                            float(configured_holding_torque)
                            if configured_holding_torque is not None
                            else 3.4028235e38
                        )
                    )
                )
                fixed_joint_torque_solver_fallback_used = bool(
                    not args.disable_gripper_break_limits
                    and args.gripper_torque_limit is None
                    and configured_holding_torque is None
                )
                grasp_joint = UsdPhysics.FixedJoint.Define(stage, grasp_joint_path)
                with Sdf.ChangeBlock():
                    grasp_joint.CreateBody0Rel().SetTargets([Sdf.Path(grasp_body_path)])
                    grasp_joint.CreateBody1Rel().SetTargets([Sdf.Path(target_carton_path)])
                    grasp_joint.CreateLocalPos0Attr(Gf.Vec3f(*local_position))
                    grasp_joint.CreateLocalRot0Attr(
                        Gf.Quatf(
                            float(local_quaternion_array[0]),
                            Gf.Vec3f(*local_quaternion_array[1:].tolist()),
                        )
                    )
                    grasp_joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
                    grasp_joint.CreateLocalRot1Attr(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
                    grasp_joint.CreateCollisionEnabledAttr(False)
                    grasp_joint.CreateExcludeFromArticulationAttr(True)
                    grasp_joint.CreateBreakForceAttr(break_force)
                    grasp_joint.CreateBreakTorqueAttr(break_torque)
                    grasp_joint.CreateJointEnabledAttr(True)
                grasp_command_succeeded = True
                grasp_enabled = True
                grasp_closed_time = simulation_time
            grasp_commanded = True
            print(
                f"FANUC_REPLAY_EVENT=grasp_command time_s={simulation_time:.6f} "
                f"model={args.gripper_model} accepted={grasp_command_succeeded} "
                f"contact_audit={physical_contact_audit.reason or 'PASS'} "
                f"frame_position_error_m={grasp_frame_position_error_m:.9g} "
                f"frame_rotation_error_rad={grasp_frame_rotation_error_rad:.9g}",
                flush=True,
            )
        if (
            grasp_enabled
            and not release_commanded
            and release_event_time is not None
            and simulation_time >= float(release_event_time)
        ):
            release_positions, release_orientations = target_body.get_world_poses()
            target_center_at_release = np.asarray(release_positions.numpy(), dtype=float)[0]
            linear_velocity, angular_velocity = target_body.get_velocities()
            release_linear_velocity_before_m_s = np.asarray(
                linear_velocity.numpy(), dtype=float
            )[0]
            release_angular_velocity_before_rad_s = np.asarray(
                angular_velocity.numpy(), dtype=float
            )[0]
            if args.gripper_model == "surface_gripper":
                open_results = [
                    bool(surface_gripper_interface.open_gripper(path))
                    for path in surface_gripper_paths
                ]
                release_command_succeeded = bool(open_results and all(open_results))
                release_executed = release_command_succeeded
                release_velocity_sample_pending = release_executed
            else:
                # FixedJoint cannot be hot-opened reliably in Isaac Sim 6.0.
                # Keep this state handoff only in the explicitly diagnostic
                # adapter; production qualification always rejects it.
                released_payload_body.set_world_poses(release_positions, release_orientations)
                released_payload_body.set_velocities(
                    release_linear_velocity_before_m_s[None, :].astype(np.float32),
                    release_angular_velocity_before_rad_s[None, :].astype(np.float32),
                )
                released_payload_physx_api.GetDisableGravityAttr().Set(False)
                UsdGeom.Imageable(stage.GetPrimAtPath(target_carton_path)).MakeInvisible()
                target_body = released_payload_body
                release_command_succeeded = True
                release_executed = True
                release_velocity_sample_pending = True
            release_commanded = True
            print(
                f"FANUC_REPLAY_EVENT=release_command time_s={simulation_time:.6f} "
                f"model={args.gripper_model} accepted={release_command_succeeded}",
                flush=True,
            )
        command_source_order = _sample(timestamps, positions, simulation_time)
        command = command_source_order[command_order]
        articulation.set_dof_position_targets(command[None, :])
        render = step % args.render_every == 0 or step == physics_steps - 1
        world.step(render=render)
        if render and (args.record_replay or args.record_video):
            rendered_rgba = np.asarray(rgb_annotator.get_data())
            if rendered_rgba.ndim == 3 and rendered_rgba.shape[-1] >= 3:
                rendered_rgb = rendered_rgba[..., :3].astype(np.uint8).copy()
                if args.record_replay:
                    replay_frames.append(Image.fromarray(rendered_rgb))
                if replay_video_writer is not None:
                    video_frame = cv2.cvtColor(rendered_rgb, cv2.COLOR_RGB2BGR)
                    replay_video_writer.write(video_frame)
                    if preview_video_writer is not None:
                        preview_video_writer.write(video_frame)
                    replay_video_frame_count += 1
        if (
            args.gripper_model == "surface_gripper"
            and grasp_commanded
            and not release_executed
        ):
            active_gripper_paths = []
            for surface_gripper_path in surface_gripper_paths:
                gripped_objects = list(
                    surface_gripper_interface.get_gripped_objects(surface_gripper_path)
                )
                status = surface_gripper_interface.get_gripper_status(surface_gripper_path)
                if status == SurfaceGripperClosed and target_carton_path in gripped_objects:
                    active_gripper_paths.append(surface_gripper_path)
            active_gripper_count = len(active_gripper_paths)
            if not grasp_enabled and active_gripper_count > 0:
                grasp_enabled = True
                grasp_closed_time = simulation_time
                minimum_active_gripper_count = active_gripper_count
                print(
                    f"FANUC_REPLAY_EVENT=grasp_closed time_s={simulation_time:.6f} "
                    f"active_cups={active_gripper_count}",
                    flush=True,
                )
            elif grasp_enabled:
                minimum_active_gripper_count = min(
                    int(minimum_active_gripper_count), active_gripper_count
                )
            if grasp_enabled and active_gripper_count == 0 and surface_grip_lost_time is None:
                surface_grip_lost_time = simulation_time
                print(
                    f"FANUC_REPLAY_EVENT=grasp_lost time_s={simulation_time:.6f}",
                    flush=True,
                )
        if release_velocity_sample_pending:
            linear_velocity, angular_velocity = target_body.get_velocities()
            release_linear_velocity_after_m_s = np.asarray(
                linear_velocity.numpy(), dtype=float
            )[0]
            release_angular_velocity_after_rad_s = np.asarray(
                angular_velocity.numpy(), dtype=float
            )[0]
            release_velocity_sample_pending = False
        if (
            args.gripper_model == "surface_gripper"
            and release_executed
            and not release_open_confirmed
        ):
            release_open_confirmed = all(
                surface_gripper_interface.get_gripper_status(path) == SurfaceGripperOpen
                and target_carton_path
                not in list(surface_gripper_interface.get_gripped_objects(path))
                for path in surface_gripper_paths
            )
        if (
            grasp_enabled
            and not release_executed
            and grasp_local_position is not None
            and grasp_local_quaternion is not None
        ):
            body_positions, body_orientations = grasp_body.get_world_poses()
            carton_positions, carton_orientations = target_body.get_world_poses()
            body_position = np.asarray(body_positions.numpy(), dtype=float)[0]
            carton_position = np.asarray(carton_positions.numpy(), dtype=float)[0]
            body_quaternion = np.asarray(body_orientations.numpy(), dtype=float)[0]
            carton_quaternion = np.asarray(carton_orientations.numpy(), dtype=float)[0]
            body_rotation = Gf.Rotation(
                Gf.Quatd(float(body_quaternion[0]), Gf.Vec3d(*body_quaternion[1:].tolist()))
            )
            carton_rotation = Gf.Rotation(
                Gf.Quatd(float(carton_quaternion[0]), Gf.Vec3d(*carton_quaternion[1:].tolist()))
            )
            expected_carton_position = body_position + np.asarray(
                body_rotation.TransformDir(grasp_local_position), dtype=float
            )
            attachment_position_error = float(
                np.linalg.norm(expected_carton_position - carton_position)
            )
            body_quaternion /= np.linalg.norm(body_quaternion)
            carton_quaternion /= np.linalg.norm(carton_quaternion)
            expected_carton_quaternion = _quaternion_multiply_wxyz(
                body_quaternion, grasp_local_quaternion
            )
            attachment_rotation_delta = _quaternion_multiply_wxyz(
                _quaternion_conjugate_wxyz(expected_carton_quaternion), carton_quaternion
            )
            attachment_rotation_error = _quaternion_angle_wxyz(attachment_rotation_delta)
            peak_payload_attachment_position_error_m = max(
                peak_payload_attachment_position_error_m, attachment_position_error
            )
            peak_payload_attachment_rotation_error_rad = max(
                peak_payload_attachment_rotation_error_rad, attachment_rotation_error
            )
        if (
            target_body is not None
            and release_executed
            and release_event_time is not None
            and conveyor_initial_direction_world is not None
        ):
            payload_positions, _ = target_body.get_world_poses()
            payload_linear_velocities, _ = target_body.get_velocities()
            payload_center = np.asarray(payload_positions.numpy(), dtype=float)[0]
            payload_linear_velocity = np.asarray(
                payload_linear_velocities.numpy(), dtype=float
            )[0]
            elapsed_after_release = simulation_time - float(release_event_time)
            if (
                target_landing_center is None
                and expected_place_center.shape == (3,)
                and elapsed_after_release >= conveyor_landing_capture_delay_s
                and abs(float(payload_center[2] - expected_place_center[2]))
                <= conveyor_landing_height_tolerance_m
            ):
                target_landing_center = payload_center.copy()
                target_landing_time_s = simulation_time
            if (
                target_landing_center is not None
                and conveyor_started
                and conveyor_started_time_s is not None
            ):
                # Restrict the speed audit to the configured interval after
                # landing. The former implementation compared simulation time
                # with the belt start time (normally zero), so every carton
                # landing later than one second silently produced no samples.
                # An L-shaped conveyor can subsequently transfer the carton
                # to a different surface with a different travel direction.
                elapsed_after_landing = simulation_time - float(target_landing_time_s)
                if elapsed_after_landing <= conveyor_transport_audit_window_s + 1e-9:
                    conveyor_transport_samples.append(
                        (
                            simulation_time,
                            payload_center.copy(),
                            payload_linear_velocity.copy(),
                            active_conveyor_surface,
                        )
                    )
        measured = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0]
        # get_dof_efforts() reports explicitly commanded effort inputs and is
        # zero for position drives. Projected joint forces are the physically
        # meaningful loads transmitted along each revolute DOF.
        projected_forces = np.asarray(
            articulation.get_dof_projected_joint_forces().numpy(), dtype=float
        )[0]
        gravity_forces = np.asarray(
            articulation.get_dof_gravity_compensation_forces().numpy(), dtype=float
        )[0]
        measured_times.append(simulation_time)
        commanded_rows.append(command.astype(float))
        measured_rows.append(measured)
        projected_force_rows.append(projected_forces)
        gravity_force_rows.append(gravity_forces)
    replay_wall_s = time.perf_counter() - replay_started_at
    if replay_video_writer is not None:
        replay_video_writer.release()
        replay_video_writer = None
        if not replay_video_path.is_file() or replay_video_path.stat().st_size == 0:
            raise RuntimeError("MP4 replay writer produced no output")
    if preview_video_writer is not None:
        preview_video_writer.release()
        preview_video_writer = None
        if not preview_video_path.is_file() or preview_video_path.stat().st_size == 0:
            raise RuntimeError("accelerated MP4 preview writer produced no output")

    measured_array = np.asarray(measured_rows)
    commanded_array = np.asarray(commanded_rows)
    projected_force_array = np.asarray(projected_force_rows)
    gravity_force_array = np.asarray(gravity_force_rows)
    errors = measured_array - commanded_array
    rms_error = np.sqrt(np.mean(errors**2, axis=0))
    peak_error = np.max(np.abs(errors), axis=0)
    peak_projected_force = np.max(np.abs(projected_force_array), axis=0)
    peak_gravity_force = np.max(np.abs(gravity_force_array), axis=0)
    effort_ratios = peak_projected_force / effort_limits if effort_limits.size else np.asarray([])

    contact_records = [
        {"actor0": pair[0], "actor1": pair[1], **values}
        for pair, values in sorted(contact_pairs.items())
    ]
    target_name = str(metadata["target"])
    grasp_time = metadata.get("grasp_time_seconds")
    release_time = metadata.get("release_time_seconds")

    def _expected_target_contact(record) -> bool:
        actors = (str(record["actor0"]), str(record["actor1"]))
        target_match = any(actor.endswith(f"/{_safe_prim_name(target_name)}") for actor in actors)
        wrist_match = any(
            actor == grasp_body_path or actor.startswith(f"{grasp_body_path}/")
            for actor in actors
        )
        if not target_match or not wrist_match or grasp_time is None:
            return False
        # The collision geometry normally contacts the carton shortly before
        # the commanded vacuum event. Keep this narrow and explicit so an
        # arbitrary earlier wrist/carton collision cannot be hidden.
        lower = float(grasp_time) - 0.25
        upper = float("inf") if release_time is None else float(release_time) + 2.0 * physics_dt
        return float(record["first_contact_time_s"]) >= lower and float(record["last_contact_time_s"]) <= upper

    robot_contact_records = [
        record
        for record in contact_records
        if str(record["actor0"]).startswith(root_prim_path)
        or str(record["actor1"]).startswith(root_prim_path)
    ]
    payload_paths = {path for path in (target_carton_path, released_payload_path) if path}
    payload_contact_records = [
        record
        for record in contact_records
        if str(record["actor0"]) in payload_paths or str(record["actor1"]) in payload_paths
    ]
    conveyor_paths = set(conveyor_surface_paths.values())
    payload_conveyor_contact_records = [
        record
        for record in payload_contact_records
        if str(record["actor0"]) in conveyor_paths
        or str(record["actor1"]) in conveyor_paths
    ]
    premature_payload_conveyor_contacts = None
    if release_time is not None:
        # Allow two physics ticks around the discrete release command, but fail
        # any contact that begins while the payload is still attached. Contact
        # reports were already collected in previous replays; the missing part
        # was this release-relative classification.
        release_contact_tolerance_s = 2.0 * physics_dt
        premature_payload_conveyor_contacts = [
            record
            for record in payload_conveyor_contact_records
            if float(record["first_contact_time_s"])
            < float(release_time) - release_contact_tolerance_s
        ]
    unexpected_contacts = [
        record for record in robot_contact_records if not _expected_target_contact(record)
    ]
    tracking_error_limit_rad = 0.05
    full_schedule_replayed = replay_duration >= requested_duration - 1e-9
    target_final_center = None
    payload_displacement_m = None
    if target_body is not None:
        target_positions, _ = target_body.get_world_poses()
        target_final_center_array = np.asarray(target_positions.numpy(), dtype=float)[0]
        target_final_center = target_final_center_array.tolist()
        if initial_target_center is not None:
            payload_displacement_m = float(
                np.linalg.norm(target_final_center_array - initial_target_center)
            )
    qualification_policy = ReplayQualificationPolicy()
    tracking_error_limit_rad = qualification_policy.tracking_error_limit_rad
    attachment_position_tolerance_m = qualification_policy.attachment_position_tolerance_m
    attachment_rotation_tolerance_rad = qualification_policy.attachment_rotation_tolerance_rad
    payload_attachment_intact = bool(
        grasp_enabled
        and surface_grip_lost_time is None
        and peak_payload_attachment_position_error_m <= attachment_position_tolerance_m
        and peak_payload_attachment_rotation_error_rad <= attachment_rotation_tolerance_rad
    )
    payload_motion_verified = bool(
        payload_attachment_intact
        and payload_displacement_m is not None
        and payload_displacement_m > 0.02
    )

    placement_center_error_m = None
    release_center_error_m = None
    expected_release_center = np.asarray(metadata.get("release_center_m", []), dtype=float)
    if target_center_at_release is not None and expected_release_center.shape == (3,):
        release_center_error_m = float(
            np.linalg.norm(target_center_at_release - expected_release_center)
        )
    if target_landing_center is not None and expected_place_center.shape == (3,):
        placement_center_error_m = float(
            np.linalg.norm(target_landing_center - expected_place_center)
        )
    conveyor_transport_distance_m = None
    conveyor_transport_projected_speed_m_s = None
    conveyor_transport_lateral_drift_m = None
    conveyor_transport_engaged = None
    conveyor_transport_speed_within_tolerance = None
    if conveyor_initial_direction_world is not None and conveyor_transport_samples:
        transport_positions = np.asarray(
            [sample[1] for sample in conveyor_transport_samples], dtype=float
        )
        transport_velocities = np.asarray(
            [sample[2] for sample in conveyor_transport_samples], dtype=float
        )
        transport_surface_names = [sample[3] for sample in conveyor_transport_samples]
        transport_offsets = transport_positions - transport_positions[0]
        transport_deltas = np.diff(transport_positions, axis=0)
        if transport_deltas.size:
            # Attribute progress only to the surface that was actually enabled
            # over each interval. Taking the maximum over every configured
            # direction can hide simultaneous/diagonal belt driving.
            step_progress = []
            for delta, surface_name in zip(
                transport_deltas, transport_surface_names[:-1], strict=True
            ):
                direction = conveyor_directions_world.get(surface_name)
                step_progress.append(
                    0.0 if direction is None else max(0.0, float(delta @ direction))
                )
            conveyor_transport_distance_m = float(np.sum(step_progress))
        else:
            conveyor_transport_distance_m = 0.0
        projected_distances = transport_offsets @ conveyor_initial_direction_world
        final_offset = transport_offsets[int(np.argmax(projected_distances))]
        lateral_offset = final_offset - float(
            final_offset @ conveyor_initial_direction_world
        ) * conveyor_initial_direction_world
        conveyor_transport_lateral_drift_m = float(np.linalg.norm(lateral_offset))
        projected_speeds = [
            float(velocity @ conveyor_directions_world[surface_name])
            for velocity, surface_name in zip(
                transport_velocities, transport_surface_names, strict=True
            )
            if surface_name in conveyor_directions_world
        ]
        if projected_speeds:
            steady_start = max(0, len(projected_speeds) // 2)
            conveyor_transport_projected_speed_m_s = float(
                np.median(projected_speeds[steady_start:])
            )
        conveyor_transport_engaged = bool(
            conveyor_transport_distance_m >= conveyor_transport_minimum_distance_m
        )
        conveyor_transport_speed_within_tolerance = bool(
            conveyor_transport_projected_speed_m_s is not None
            and abs(conveyor_transport_projected_speed_m_s - conveyor_speed_m_s)
            <= conveyor_transport_speed_tolerance_m_s
        )
    placement_tolerance_m = qualification_policy.placement_tolerance_m
    production_release_adapter = bool(
        args.gripper_model == "surface_gripper"
        and release_open_confirmed
        and released_payload_path is None
    )
    release_completed = bool(
        release_open_confirmed
        if args.gripper_model == "surface_gripper"
        else release_executed
    )
    qualification = evaluate_replay_qualification(
        policy=qualification_policy,
        full_schedule_replayed=full_schedule_replayed,
        collision_scope_complete=bool(
            import_manifest.get("import_settings", {}).get("allow_self_collision", False)
            and srdf_filter_complete
        ),
        unexpected_contact_count=len(unexpected_contacts),
        premature_payload_conveyor_contact_count=(
            None
            if premature_payload_conveyor_contacts is None
            else len(premature_payload_conveyor_contacts)
        ),
        peak_joint_error_rad=float(np.max(peak_error)),
        effort_limit_ratios=effort_ratios,
        grasp_expected=grasp_event_time is not None,
        grasp_enabled=grasp_enabled,
        attachment_lost=surface_grip_lost_time is not None,
        peak_attachment_position_error_m=(
            peak_payload_attachment_position_error_m if grasp_enabled else None
        ),
        peak_attachment_rotation_error_rad=(
            peak_payload_attachment_rotation_error_rad if grasp_enabled else None
        ),
        payload_displacement_m=payload_displacement_m,
        release_expected=release_event_time is not None,
        release_executed=release_completed,
        placement_expected=expected_place_center.shape == (3,),
        placement_center_error_m=placement_center_error_m,
        gripper_wrench_envelope_complete=gripper_wrench_envelope_complete,
        gripper_limits_from_configuration=bool(
            not args.disable_gripper_break_limits
            and args.gripper_force_limit is None
            and args.gripper_shear_force_limit is None
            and args.gripper_torque_limit is None
            and args.gripper_attachment_point_count is None
            and not surface_shear_solver_fallback_used
            and not fixed_joint_torque_solver_fallback_used
        ),
        gripper_limits_calibrated=bool(metadata.get("gripper", {}).get("limits_calibrated", False)),
        production_release_adapter=production_release_adapter,
        conveyor_transport_expected=bool(conveyor_enabled and place_surface),
        conveyor_transport_engaged=conveyor_transport_engaged,
        conveyor_transport_speed_within_tolerance=conveyor_transport_speed_within_tolerance,
    )
    qualification_checks = qualification["qualification_checks"]
    qualification_failures = qualification["qualification_failures"]
    qualification_passed = qualification["qualification_passed"]

    rgba = np.asarray(rgb_annotator.get_data())
    depth = np.asarray(depth_annotator.get_data())
    if isinstance(rgba, np.ndarray) and rgba.ndim == 3:
        Image.fromarray(rgba[..., :3].astype(np.uint8)).save(args.output / "rgb.png")
    replay_path = args.output / "replay.gif"
    if replay_frames:
        replay_frames[0].save(
            replay_path,
            save_all=True,
            append_images=replay_frames[1:],
            duration=max(1, int(round(1000.0 * args.render_every / physics_hz))),
            loop=0,
            optimize=False,
        )
    np.save(args.output / "depth_m.npy", depth)
    with (args.output / "joint_tracking.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["time_s"]
            + [f"command_{name}_rad" for name in discovered_joint_names]
            + [f"measured_{name}_rad" for name in discovered_joint_names]
            + [f"projected_force_{name}_nm" for name in discovered_joint_names]
            + [f"gravity_compensation_{name}_nm" for name in discovered_joint_names]
        )
        for timestamp, command, measured, projected_force, gravity_force in zip(
            measured_times,
            commanded_array,
            measured_array,
            projected_force_array,
            gravity_force_array,
        ):
            writer.writerow(
                [
                    timestamp,
                    *command.tolist(),
                    *measured.tolist(),
                    *projected_force.tolist(),
                    *gravity_force.tolist(),
                ]
            )

    result = {
        "format": "isaacsim_fanuc_replay_result_v2",
        "robot_model": metadata["robot_model"],
        "target": metadata["target"],
        "simulation_execution_ready": metadata.get("simulation_execution_ready"),
        "execution_qualified": metadata.get("execution_qualified"),
        "machine_qualified": metadata.get("machine_qualified"),
        "machine_qualification_warnings": metadata.get(
            "machine_qualification_warnings", []
        ),
        "pre_simulation_integrity_gate": pre_simulation_integrity_gate,
        "usd_path": str(usd_path),
        "root_prim_path": root_prim_path,
        "urdf_imported_this_run": imported_now,
        "urdf_import_wall_seconds": import_wall_s,
        "app_startup_wall_seconds": app_startup_wall_s,
        "offline_planning_time_seconds": metadata.get("offline_planning_time_seconds", 0.0),
        "command_schedule_duration_seconds": requested_duration,
        "post_release_settle_seconds": float(max(0.0, replay_duration - requested_duration)),
        "replayed_simulation_seconds": replay_duration,
        "replay_wall_seconds": replay_wall_s,
        "simulation_realtime_factor": replay_duration / replay_wall_s,
        "physics_steps": physics_steps,
        "physics_hz": physics_hz,
        "physics_contract": physics_contract,
        "initial_state_settling": settling_audit,
        "render_every_physics_steps": args.render_every,
        "effective_render_rate_hz": physics_hz / args.render_every,
        "replay_recorded": bool(replay_frames or replay_video_frame_count),
        "replay_frame_count": max(len(replay_frames), replay_video_frame_count),
        "replay_path": (
            str(replay_video_path)
            if replay_video_frame_count
            else str(replay_path) if replay_frames else None
        ),
        "replay_video_recorded": bool(replay_video_frame_count),
        "replay_video_frame_count": replay_video_frame_count,
        "replay_video_path": str(replay_video_path) if replay_video_frame_count else None,
        "replay_video_physical_time_scale": 1.0,
        "replay_preview_speed": (
            float(args.video_preview_speed) if replay_video_frame_count and args.video_preview_speed > 1.0 else None
        ),
        "replay_preview_video_path": (
            str(preview_video_path)
            if replay_video_frame_count and args.video_preview_speed > 1.0
            else None
        ),
        "joint_names": discovered_joint_names,
        "rms_joint_error_rad": rms_error.tolist(),
        "peak_joint_error_rad": peak_error.tolist(),
        "peak_error_rad": float(np.max(peak_error)),
        "joint_load_metric": "physx_projected_joint_force",
        "peak_projected_joint_force_nm": peak_projected_force.tolist(),
        "peak_gravity_compensation_nm": peak_gravity_force.tolist(),
        "effort_limit_ratio": (
            effort_ratios.tolist() if effort_limits.size else []
        ),
        "camera_rgba_shape": list(rgba.shape),
        "camera_depth_shape": list(depth.shape),
        "finite_depth_fraction": float(np.mean(np.isfinite(depth))),
        "collision_source": import_manifest["collision_source"],
        "dynamic_collision_approximation": import_manifest["dynamic_collision_approximation"],
        "self_collision_monitoring_enabled": bool(
            import_manifest.get("import_settings", {}).get("allow_self_collision", False)
        ),
        "srdf_collision_filter_complete": srdf_filter_complete,
        "srdf_allowed_self_collision_pairs": allowed_self_collision_pairs,
        "scene_primitive_count": len(scene_primitives),
        "synthetic_ground_created": synthetic_ground_created,
        "explicit_floor_declared": explicit_floor_declared,
        "robot_tool_mass_accounting": tool_mass_accounting,
        "conveyor_enabled": conveyor_enabled,
        "conveyor_surface_velocity_api": (
            "PhysxSchema.PhysxSurfaceVelocityAPI" if conveyor_enabled else None
        ),
        "conveyor_surface_paths": conveyor_surface_paths,
        "conveyor_speed_command_m_s": conveyor_speed_m_s if conveyor_enabled else None,
        "conveyor_start_policy": conveyor_start_policy if conveyor_enabled else None,
        "conveyor_started_time_s": conveyor_started_time_s,
        "conveyor_exclusive_surface_drive_at_transfer": conveyor_exclusive,
        "conveyor_active_surface_history": conveyor_surface_history,
        "conveyor_maximum_simultaneously_active_surfaces": max(
            (len(item["active_surfaces"]) for item in conveyor_surface_history),
            default=0,
        ),
        "conveyor_initial_surface": place_surface if conveyor_initial_direction_world is not None else None,
        "conveyor_initial_direction_world": (
            conveyor_initial_direction_world.tolist()
            if conveyor_initial_direction_world is not None
            else None
        ),
        "target_landing_center_m": (
            target_landing_center.tolist() if target_landing_center is not None else None
        ),
        "target_landing_time_s": target_landing_time_s,
        "conveyor_transport_distance_m": conveyor_transport_distance_m,
        "conveyor_transport_projected_speed_m_s": conveyor_transport_projected_speed_m_s,
        "conveyor_transport_lateral_drift_m": conveyor_transport_lateral_drift_m,
        "conveyor_transport_engaged": conveyor_transport_engaged,
        "conveyor_transport_speed_within_tolerance": conveyor_transport_speed_within_tolerance,
        "conveyor_transport_speed_audit_model": "piecewise_selected_active_surface_direction",
        "conveyor_transport_direction_count": len(conveyor_directions_world),
        "target_carton_dynamic": target_carton_path is not None,
        "payload_constraint_commanded": grasp_commanded,
        "payload_grasp_command_succeeded": grasp_command_succeeded,
        "payload_grasp_closed_time_s": grasp_closed_time,
        "payload_grip_lost_time_s": surface_grip_lost_time,
        "payload_minimum_active_cup_count": minimum_active_gripper_count,
        "payload_coupled_to_robot": payload_motion_verified,
        "payload_attachment_intact": payload_attachment_intact,
        "peak_payload_attachment_position_error_m": peak_payload_attachment_position_error_m,
        "peak_payload_attachment_rotation_error_rad": peak_payload_attachment_rotation_error_rad,
        "attachment_position_tolerance_m": attachment_position_tolerance_m,
        "attachment_rotation_tolerance_rad": attachment_rotation_tolerance_rad,
        "payload_displacement_m": payload_displacement_m,
        "payload_release_command_succeeded": release_command_succeeded,
        "payload_release_executed": release_completed,
        "payload_release_open_confirmed": release_open_confirmed,
        "payload_release_model": (
            "isaac_surface_gripper_open_same_rigid_body"
            if args.gripper_model == "surface_gripper"
            else "physx_free_body_state_handoff_diagnostic"
        ),
        "release_linear_velocity_before_m_s": (
            None
            if release_linear_velocity_before_m_s is None
            else release_linear_velocity_before_m_s.tolist()
        ),
        "release_angular_velocity_before_rad_s": (
            None
            if release_angular_velocity_before_rad_s is None
            else release_angular_velocity_before_rad_s.tolist()
        ),
        "release_linear_velocity_after_one_step_m_s": (
            None
            if release_linear_velocity_after_m_s is None
            else release_linear_velocity_after_m_s.tolist()
        ),
        "release_angular_velocity_after_one_step_rad_s": (
            None
            if release_angular_velocity_after_rad_s is None
            else release_angular_velocity_after_rad_s.tolist()
        ),
        "target_center_at_release_m": (
            target_center_at_release.tolist() if target_center_at_release is not None else None
        ),
        "release_center_error_m": release_center_error_m,
        "target_final_center_m": target_final_center,
        "expected_place_center_m": metadata.get("place_center_m", []),
        "placement_center_error_m": placement_center_error_m,
        "placement_tolerance_m": placement_tolerance_m,
        "qualification_checks": qualification_checks,
        "qualification_check_details": qualification["qualification_check_details"],
        "qualification_model": qualification["model"],
        "qualification_failures": qualification_failures,
        "grasp_frame_position_error_m": grasp_frame_position_error_m,
        "grasp_frame_rotation_error_rad": grasp_frame_rotation_error_rad,
        "gripper_attachment_raycast_distances_m": attachment_raycast_distances_m,
        "gripper_attachment_raycast_hit_count": sum(
            distance is not None for distance in attachment_raycast_distances_m
        ),
        "gripper_attachment_raycast_within_capture_count": sum(
            distance is not None
            and -float(gripper_cfg.get("maximum_contact_penetration_m", 1e-5)) - 1e-12
            <= distance
            <= float(gripper_cfg.get("max_grip_distance_m", 0.03)) + 1e-12
            for distance in attachment_raycast_distances_m
        ),
        "gripper_physical_contact_audit": (
            None if physical_contact_audit is None else physical_contact_audit.to_dict()
        ),
        "gripper_model_source": metadata.get("gripper", {}).get("model_source"),
        "gripper_adapter": args.gripper_model,
        "gripper_collision_enabled": not args.disable_gripper_collision,
        "gripper_collision_representation": (
            "step_per_rigid_solid_axis_aligned_boxes_compliant_cups_excluded"
            if gripper_mesh_loaded
            else "legacy_placeholder_box"
        ),
        "gripper_product_model": gripper_cfg.get("product_model"),
        "gripper_physical_cup_model": gripper_cfg.get("cup_model"),
        "gripper_physical_cup_count": gripper_cfg.get("physical_cup_count"),
        "gripper_active_sealed_cup_count": gripper_cfg.get("active_sealed_cup_count"),
        "gripper_pull_off_force_per_cup_n": gripper_cfg.get("pull_off_force_per_cup_n"),
        "gripper_shear_force_per_cup_n": gripper_cfg.get("shear_force_per_cup_n"),
        "gripper_catalogue_theoretical_total_force_n_at_minus_60_kpa": gripper_cfg.get(
            "catalogue_theoretical_total_force_n_at_minus_60_kpa"
        ),
        "gripper_footprint_size_m": footprint_size.tolist(),
        "gripper_simulation_attachment_point_count": len(surface_attachment_paths),
        "gripper_attachment_point_count": len(surface_attachment_paths),
        "gripper_solver_attachment_model": gripper_cfg.get("solver_attachment_model"),
        "physx_solver_position_iterations": solver_position_iterations,
        "physx_solver_velocity_iterations": solver_velocity_iterations,
        "gripper_wrench_envelope_complete": gripper_wrench_envelope_complete,
        "surface_gripper_torque_limit_applied": False,
        "gripper_limits_calibrated": bool(metadata.get("gripper", {}).get("limits_calibrated", False)),
        "gripper_break_limits_enabled": (
            None
            if args.gripper_model == "surface_gripper"
            else not args.disable_gripper_break_limits
        ),
        "gripper_force_limit_n": (
            float(surface_force_per_point)
            if args.gripper_model == "surface_gripper" and surface_gripper_paths
            else break_force
        ),
        "gripper_active_total_force_n": (
            float(surface_force)
            if args.gripper_model == "surface_gripper" and surface_gripper_paths
            else None
        ),
        "gripper_hardware_maximum_total_force_n": gripper_cfg.get(
            "hardware_maximum_holding_force_total_n"
        ),
        "gripper_torque_limit_nm": (
            None if args.gripper_model == "surface_gripper" else break_torque
        ),
        "gripper_configured_total_shear_force_n": configured_shear_force,
        "gripper_configured_holding_torque_nm": configured_holding_torque,
        "gripper_shear_force_limit_n": (
            float(surface_shear_force_per_point)
            if args.gripper_model == "surface_gripper" and surface_gripper_paths
            else None
        ),
        "gripper_diagnostic_total_shear_force_n": (
            float(surface_shear_force)
            if args.gripper_model == "surface_gripper" and surface_gripper_paths
            else None
        ),
        "gripper_shear_solver_fallback_used": surface_shear_solver_fallback_used,
        "surface_gripper_solver_coaxial_limit_per_constraint_n": (
            point_force_limits.tolist()
            if args.gripper_model == "surface_gripper" and surface_gripper_paths
            else None
        ),
        "surface_gripper_solver_shear_limit_per_constraint_n": (
            point_shear_limits.tolist()
            if args.gripper_model == "surface_gripper" and surface_gripper_paths
            else None
        ),
        "surface_gripper_solver_limits_partition_active_head_capacity": True,
        "gripper_fixed_joint_torque_solver_fallback_used": (
            fixed_joint_torque_solver_fallback_used
        ),
        "gripper_limits_source": (
            "disabled_diagnostic"
            if args.disable_gripper_break_limits
            else "cli_override_diagnostic"
            if any(
                value is not None
                for value in (
                    args.gripper_force_limit,
                    args.gripper_shear_force_limit,
                    args.gripper_torque_limit,
                    args.gripper_attachment_point_count,
                )
            )
            else "unbounded_shear_diagnostic_fallback"
            if surface_shear_solver_fallback_used
            else "replay_bundle_uncalibrated_hardware_maximum"
            if args.gripper_model == "surface_gripper"
            else "replay_bundle"
        ),
        "robot_scene_contact_pairs": robot_contact_records,
        "robot_scene_contact_pair_count": len(robot_contact_records),
        "unexpected_robot_scene_contacts": unexpected_contacts,
        "payload_contact_pairs": payload_contact_records,
        "payload_conveyor_contact_pairs": payload_conveyor_contact_records,
        "premature_payload_conveyor_contacts": premature_payload_conveyor_contacts,
        "premature_payload_conveyor_contact_count": (
            None
            if premature_payload_conveyor_contacts is None
            else len(premature_payload_conveyor_contacts)
        ),
        "grasp_time_seconds": grasp_time,
        "release_time_seconds": release_time,
        "release_contact_tolerance_seconds": 2.0 * physics_dt,
        "tracking_error_limit_rad": tracking_error_limit_rad,
        "qualification_passed": qualification_passed,
        "drive_gain_source": "simulation_assumption_pending_controller_log_calibration",
        "evidence_manifest": {
            "bundle_sha256": _sha256_path(args.bundle.resolve()),
            "robot_urdf_sha256": _sha256_path(urdf_path),
            "robot_srdf_sha256": _sha256_path(srdf_path) if srdf_path.is_file() else None,
            "imported_usd_sha256": _sha256_path(usd_path),
            "replay_adapter_sha256": _sha256_path(Path(__file__).resolve()),
            "source_plan_sha256": metadata.get("source_plan_sha256"),
            "merged_configuration_sha256": metadata.get("merged_configuration_sha256"),
            "isaacsim_package_version": _package_version("isaacsim"),
            "git": _git_evidence(args.project_root.resolve()),
            "physics": {
                "physics_hz": physics_hz,
                "render_every_physics_steps": args.render_every,
                "post_release_seconds": args.post_release_seconds,
                "conveyor_contact_model": (
                    "physx_surface_velocity_dynamic_payload"
                    if conveyor_enabled
                    else "disabled"
                ),
                "conveyor_speed_command_m_s": (
                    conveyor_speed_m_s if conveyor_enabled else None
                ),
                "surface_gripper_force_distribution": (
                    f"sealed_cup_wrench_aggregated_at_{gripper_cfg.get('solver_attachment_model')};_"
                    "not_a_compliant_per_cup_load_distribution"
                ),
                "physical_cup_count_is_not_simulation_attachment_point_count": True,
                "missing_shear_limit_solver_fallback": surface_shear_solver_fallback_used,
            },
            "output_sha256": {
                "rgb_png": (
                    _sha256_path(args.output / "rgb.png")
                    if (args.output / "rgb.png").is_file()
                    else None
                ),
                "replay_gif": _sha256_path(replay_path) if replay_path.is_file() else None,
                "replay_mp4": (
                    _sha256_path(replay_video_path) if replay_video_path.is_file() else None
                ),
                "replay_preview_mp4": (
                    _sha256_path(preview_video_path) if preview_video_path.is_file() else None
                ),
                "depth_m_npy": _sha256_path(args.output / "depth_m.npy"),
                "joint_tracking_csv": _sha256_path(args.output / "joint_tracking.csv"),
            },
        },
    }
    (args.output / "evidence_manifest.json").write_text(
        json.dumps(result["evidence_manifest"], indent=2), encoding="utf-8"
    )
    result_path = args.output / "result.json"
    result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    run_status_path.write_text(
        json.dumps(
            {
                "status": "complete",
                "started_unix_s": run_started_unix_s,
                "completed_unix_s": time.time(),
                "result_sha256": _sha256_path(result_path),
                "result_format": result["format"],
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print("ISAACSIM_FANUC_REPLAY_RESULT=" + json.dumps(result, sort_keys=True), flush=True)
except BaseException as exc:
    # SimulationApp.close() may terminate Kit before Python reports an uncaught
    # exception, so emit the traceback explicitly for unattended server runs.
    run_status_path.write_text(
        json.dumps(
            {
                "status": "failed",
                "started_unix_s": run_started_unix_s,
                "failed_unix_s": time.time(),
                "exception_type": type(exc).__name__,
                "exception": str(exc),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    traceback.print_exc()
    raise
finally:
    pending_video_writer = locals().get("replay_video_writer")
    if pending_video_writer is not None:
        pending_video_writer.release()
    pending_preview_writer = locals().get("preview_video_writer")
    if pending_preview_writer is not None:
        pending_preview_writer.release()
    simulation_app.close()
