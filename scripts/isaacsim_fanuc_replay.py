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


class DiagnosticSettlingComplete(Exception):
    """A requested short dynamics diagnostic is not a qualified pick cycle."""

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
    parser.add_argument("--reuse-usd-entrypoint", type=Path)
    parser.add_argument("--reuse-usd-run-evidence", type=Path)
    parser.add_argument("--reuse-usd-source-contract", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--continuation-dir", type=Path,
                        help="retain this World and accept numbered offline next-bundle requests")
    parser.add_argument("--maximum-segments", type=int, default=1)
    parser.add_argument("--diagnostic-only", action="store_true")
    parser.add_argument("--diagnostic-settling-steps", type=int,
                        help="short initialization diagnostic only; never a qualified replay override")
    parser.add_argument("--continuation-wait-seconds", type=float, default=600.0)
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
    if args.diagnostic_only != (args.diagnostic_settling_steps is not None):
        raise ValueError("diagnostic-only and diagnostic-settling-steps must be specified together")
    if args.diagnostic_settling_steps is not None and args.diagnostic_settling_steps <= 0:
        raise ValueError("diagnostic settling steps must be positive")
    if args.diagnostic_only and (args.maximum_segments != 1 or args.continuation_dir is not None):
        raise ValueError("a diagnostic cannot enter task continuation")
    reuse_args = (args.reuse_usd_entrypoint, args.reuse_usd_run_evidence, args.reuse_usd_source_contract)
    if any(value is not None for value in reuse_args) and not all(value is not None for value in reuse_args):
        raise ValueError("official USD reuse requires entrypoint, recorded run evidence and source contract")
    if args.maximum_segments < 1 or not math.isfinite(args.continuation_wait_seconds) or args.continuation_wait_seconds <= 0:
        raise ValueError("continuation budgets must be finite and positive")
    if args.maximum_segments > 1 and args.continuation_dir is None:
        raise ValueError("multiple same-world segments require continuation-dir")
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


def _rotation_matrix_from_rpy_xyz(rpy):
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


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
    # The standard-library gate checks bundle, implementation and manifest
    # identities before Kit startup. The full CAD audit now imports NumPy;
    # run that by its real package name after SimulationApp, before any World.
    pre_simulation_integrity_gate = contract_module.verify_m710_replay_bundle(
        bundle,
        project_root=project_root,
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
    simulation_execution_qualified = metadata.get(
        "simulation_execution_qualified",
        metadata.get("simulation_execution_ready") is True and not execution_blockers,
    )
    if (
        metadata.get("simulation_execution_ready") is not True
        or execution_blockers
        or simulation_execution_qualified is not True
    ):
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
    if not isinstance(metadata.get("execution_qualified"), bool):
        raise ValueError("M-710 legacy execution_qualified must be a boolean")
    if args.disable_gripper_collision:
        raise ValueError(
            "M-710 qualified replay forbids disabling rigid gripper collision"
        )
    # A sourced rigid-body simulation is allowed to run while real-machine
    # controller/safety qualification remains false.  That evidence is kept in
    # the result, but it is no longer a hidden execution gate.

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
        "required_output",
        "material_palette",
        "conveyor_visual_motion",
        "pbr_materials",
    }:
        raise ValueError("M-710 replay requires a content-addressed rendering contract")
    if rendering_contract["material_palette"] != {
        "chassis_rgb": [0.08, 0.09, 0.11],
        "conveyor_rgb": [0.035, 0.04, 0.045],
        "conveyor_frame_rgb": [0.32, 0.36, 0.40],
        "conveyor_motion_marker_rgb": [0.62, 0.67, 0.70],
        "conveyor_roller_rgb": [0.18, 0.20, 0.22],
        "trailer_rgb": [0.56, 0.60, 0.64],
    }:
        raise ValueError("M-710 replay rendering palette differs from the bound contract")
    if rendering_contract["conveyor_visual_motion"] != {
        "model": "industrial_belt_surface_and_roller_phase_v2",
        "markers_have_collision": False,
        "markers_follow_active_physx_surface_velocity": True,
        "rollers_follow_active_physx_surface_velocity": True,
        "independent_phase_accumulators": True,
        "stopped_surface_phase_is_frozen": True,
    }:
        raise ValueError("M-710 replay conveyor visual-motion contract is invalid")
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
    if metadata.get("robot_model") == "fanuc_m710id_70":
        from unloading_sim.m710_execution import audit_m710_replay_assets
        from unloading_sim import m710_replay_contract as contract_module
        current_asset_audit = audit_m710_replay_assets(project_root, metadata)
        pre_simulation_integrity_gate = contract_module.verify_m710_replay_bundle(
            bundle, project_root=project_root, current_asset_audit=current_asset_audit)
        print("FANUC_REPLAY_STAGE=current_assets_verified", flush=True)
    from unloading_sim.qualification import (
        ReplayQualificationPolicy,
        evaluate_replay_qualification,
    )
    from unloading_sim.m710_replay_physics import (
        audit_payload_support_contact,
        audit_surface_attachment_contact,
        select_active_conveyor_surfaces,
        replay_command_arrays,
        swept_payload_tool_clearance,
        validate_continuation_request,
        obb_penetration_depth,
        verify_physics_backend_readback,
    )
    from unloading_sim.independent_cups import (
        IDEAL_INDEPENDENT_CUPS_MODE,
        audit_actual_independent_cup_contacts,
        build_m710_independent_cup_array,
        evaluate_independent_cup_geometry,
    )
    from unloading_sim.geometry import OBB
    import omni.replicator.core as rep
    print("FANUC_REPLAY_STAGE=replicator_imported", flush=True)
    import omni.usd
    print("FANUC_REPLAY_STAGE=usd_imported", flush=True)
    from PIL import Image, ImageDraw, ImageFont
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
    from omni.physx.bindings._physx import ContactEventType
    from pxr import (
        Gf,
        PhysicsSchemaTools,
        PhysxSchema,
        Sdf,
        Usd,
        UsdGeom,
        UsdLux,
        UsdPhysics,
        UsdShade,
    )
    print("FANUC_REPLAY_STAGE=pxr_imported", flush=True)

    def _obb_penetration_depth(center_a, half_a, rotation_a, center_b, half_b, rotation_b):
        """Return the minimum SAT overlap, or zero for separated/touching OBBs."""
        return obb_penetration_depth(center_a, half_a, rotation_a, center_b, half_b, rotation_b)

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

    def _load_hud_fonts(frame_height: int):
        body_size = max(26, int(round(28.0 * frame_height / 1080.0)))
        small_size = max(18, int(round(20.0 * frame_height / 1080.0)))
        candidates = (
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
            "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        )
        for candidate in candidates:
            if Path(candidate).is_file():
                return (
                    ImageFont.truetype(candidate, body_size),
                    ImageFont.truetype(candidate, small_size),
                    "NotoSansCJK" in candidate or "wqy" in candidate,
                    candidate,
                )
        return ImageFont.load_default(), ImageFont.load_default(), False, "PIL_DEFAULT"

    def _draw_runtime_hud(rendered_rgb, lines, assumption, status_color):
        image = Image.fromarray(rendered_rgb).convert("RGBA")
        overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        body_font, small_font, _, _ = hud_fonts
        scale = image.height / 1080.0
        left, top = int(22 * scale), int(20 * scale)
        pad_x, pad_y = int(18 * scale), int(14 * scale)
        line_gap = int(7 * scale)
        measured = [draw.textbbox((0, 0), line, font=body_font) for line in lines]
        widths = [box[2] - box[0] for box in measured]
        heights = [box[3] - box[1] for box in measured]
        assumption_box = draw.textbbox((0, 0), assumption, font=small_font)
        panel_width = max(max(widths, default=0), assumption_box[2] - assumption_box[0]) + 2 * pad_x
        panel_height = sum(heights) + line_gap * max(0, len(lines) - 1) + 2 * pad_y
        footer_gap = int(10 * scale)
        footer_height = assumption_box[3] - assumption_box[1]
        panel_height += footer_gap + footer_height
        draw.rounded_rectangle(
            [left, top, left + panel_width, top + panel_height],
            radius=max(8, int(12 * scale)), fill=(8, 12, 18, 224),
            outline=(120, 132, 145, 190), width=max(1, int(2 * scale)),
        )
        y = top + pad_y
        for index, (line, height) in enumerate(zip(lines, heights, strict=True)):
            color = status_color if index == len(lines) - 1 else (248, 250, 252, 255)
            draw.text((left + pad_x, y), line, font=body_font, fill=color)
            y += height + line_gap
        y += footer_gap - line_gap
        draw.text((left + pad_x, y), assumption, font=small_font, fill=(190, 202, 214, 255))
        return np.asarray(Image.alpha_composite(image, overlay).convert("RGB"), dtype=np.uint8)

    timestamps, positions = replay_command_arrays(bundle, expected_joint_names)

    args.usd_directory.mkdir(parents=True, exist_ok=True)
    urdf_path = (args.project_root.resolve() / metadata["urdf_path"]).resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"FANUC URDF not found: {urdf_path}")
    urdf_package_search_root = None
    official_manifest = metadata.get("official_model_manifest")
    if isinstance(official_manifest, dict):
        project_root_resolved = args.project_root.resolve()
        source_records = official_manifest.get("source_files", [])
        for source_record in source_records:
            source_path = (project_root_resolved / source_record["path"]).resolve()
            try:
                source_path.relative_to(project_root_resolved)
            except ValueError as exc:
                raise ValueError("official source path escapes the project root") from exc
            if (
                not source_path.is_file()
                or source_path.stat().st_size != int(source_record["bytes"])
                or _sha256_path(source_path) != source_record["sha256"]
            ):
                raise ValueError(f"official source asset mismatch: {source_record['path']}")
        expanded_urdf_record = (official_manifest.get("integration") or {}).get(
            "expanded_urdf"
        )
        if (
            not isinstance(expanded_urdf_record, dict)
            or urdf_path.stat().st_size != int(expanded_urdf_record.get("bytes", -1))
            or _sha256_path(urdf_path) != expanded_urdf_record.get("sha256")
        ):
            raise ValueError("official expanded URDF bytes differ from the pinned manifest")
        package_name = str(
            (official_manifest.get("upstream") or {}).get("package_name", "")
        ).strip()
        package_manifest_records = [
            item
            for item in source_records
            if isinstance(item, dict) and item.get("role") == "package_manifest"
        ]
        if len(package_manifest_records) != 1 or not package_name:
            raise ValueError("official URDF requires one sourced ROS package manifest")
        package_manifest_path = (
            args.project_root.resolve() / package_manifest_records[0]["path"]
        ).resolve()
        if (
            not package_manifest_path.is_file()
            or package_manifest_path.parent.name != package_name
            or urdf_path != package_manifest_path.parent / "urdf" / urdf_path.name
        ):
            raise ValueError("official URDF is outside its pinned ROS package")
        urdf_package_search_root = package_manifest_path.parent.parent
        existing_package_path = os.environ.get("ROS_PACKAGE_PATH", "")
        search_entries = [str(urdf_package_search_root)]
        if existing_package_path:
            search_entries.append(existing_package_path)
        os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(search_entries)
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
        # The expanded URDF references external DAE/STL files.  Binding only
        # the XML hash would permit a stale cached USD after a mesh changed.
        "official_model_manifest_sha256": metadata.get(
            "official_model_manifest_sha256"
        ),
        "official_source_files": (
            metadata.get("official_model_manifest") or {}
        ).get("source_files", []),
        "ros_package_search_root": (
            None
            if urdf_package_search_root is None
            else str(urdf_package_search_root)
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
    if args.reuse_usd_entrypoint is not None:
        from unloading_sim.isaac_usd_cache import verify_recorded_official_usd
        usd_reuse = verify_recorded_official_usd(
            args.reuse_usd_entrypoint, args.reuse_usd_run_evidence,
            args.reuse_usd_source_contract, metadata)
        if usd_reuse["source_urdf_sha256"] != urdf_sha256:
            raise ValueError("current official URDF differs from the evidenced cached USD")
        usd_path = Path(usd_reuse["usd_path"])
        root_prim_path = "/fanuc_m710id_70"
        import_manifest = {"urdf_path": str(urdf_path), "urdf_sha256": urdf_sha256,
                           "usd_path": str(usd_path), "root_prim_path": root_prim_path,
                           "import_settings": requested_import_settings,
                           "reuse_evidence": usd_reuse,
                           "collision_source": requested_import_settings["collision_source"],
                           "dynamic_collision_approximation": "Convex Decomposition",
                           "fix_base": True}
        args.usd_directory.mkdir(parents=True, exist_ok=True)
        import_manifest_path.write_text(json.dumps(import_manifest, indent=2), encoding="utf-8")
    elif cache_matches:
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
    interior_light = UsdLux.RectLight.Define(stage, "/Validation/Lighting/TrailerCeiling")
    interior_light.CreateWidthAttr(4.8)
    interior_light.CreateHeightAttr(1.6)
    interior_light.CreateIntensityAttr(1050.0)
    interior_light.CreateColorAttr(Gf.Vec3f(0.92, 0.95, 1.0))
    UsdGeom.XformCommonAPI(interior_light.GetPrim()).SetTranslate(Gf.Vec3d(0.0, 0.0, 2.62))
    entrance_fill = UsdLux.DistantLight.Define(stage, "/Validation/Lighting/EntranceFill")
    entrance_fill.CreateIntensityAttr(420.0)
    entrance_fill.CreateAngleAttr(2.0)
    UsdGeom.XformCommonAPI(entrance_fill.GetPrim()).SetRotate(
        Gf.Vec3f(-38.0, -24.0, -18.0), UsdGeom.XformCommonAPI.RotationOrderXYZ
    )
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
    self_collision_roots = []
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(root_prim_path) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(True)
            self_collision_roots.append(str(prim.GetPath()))
    if metadata.get("robot_model") == "fanuc_m710id_70" and not self_collision_roots:
        raise RuntimeError("official cached/imported USD has no articulation root for self collision")

    # Official bundles carry the exact per-link inertials from the fixed FANUC
    # description commit.  Legacy bundles may still carry the separately
    # versioned engineering model; both are explicit and never fall back to an
    # importer density estimate.
    link_dynamics = list(metadata.get("robot_link_dynamics", []))
    tool_mass_accounting = metadata.get("robot_tool_mass_accounting")
    if metadata.get("robot_model") == "fanuc_m710id_70":
        if not isinstance(tool_mass_accounting, dict) or not str(
            tool_mass_accounting.get("policy", "")
        ).startswith("FIXED_TOOL_COMBINED_INTO_"):
            raise ValueError("M-710 replay requires explicit exactly-once tool mass accounting")
        applied_mass = float(sum(float(item["mass_kg"]) for item in link_dynamics))
        expected_mass = float(tool_mass_accounting["source_robot_mass_kg"]) + float(
            tool_mass_accounting["tool_mass_kg"]
        )
        if (
            not str(tool_mass_accounting.get("grasp_body_link", ""))
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
        imported_link_prim_lists = {}
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(root_prim_path) and prim.HasAPI(
                UsdPhysics.RigidBodyAPI
            ):
                imported_link_prim_lists.setdefault(prim.GetName(), []).append(prim)
        ambiguous_dynamic_links = sorted(
            name for name, prims in imported_link_prim_lists.items() if len(prims) != 1
        )
        if ambiguous_dynamic_links:
            raise RuntimeError(
                f"imported robot has ambiguous physical link names: {ambiguous_dynamic_links}"
            )
        imported_link_prims = {
            name: prims[0] for name, prims in imported_link_prim_lists.items()
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
            inertial_rotation = _rotation_matrix_from_rpy_xyz(
                item.get("inertial_origin_rpy_rad", [0.0, 0.0, 0.0])
            )
            inertia = inertial_rotation @ inertia @ inertial_rotation.T
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

    declared_srdf_path = metadata.get("robot_srdf_path")
    if declared_srdf_path:
        srdf_path = (args.project_root.resolve() / str(declared_srdf_path)).resolve()
        try:
            srdf_path.relative_to(args.project_root.resolve())
        except ValueError as exc:
            raise ValueError("robot SRDF path escapes the project root") from exc
    else:
        # Legacy M-20 bundles placed the SRDF beside the URDF.  Official M-710
        # bundles must carry the explicit repository path and content hash.
        srdf_path = urdf_path.with_suffix(".srdf")
    declared_srdf_sha256 = metadata.get("robot_srdf_sha256")
    if official_manifest is not None:
        if (
            not srdf_path.is_file()
            or not isinstance(declared_srdf_sha256, str)
            or _sha256_path(srdf_path) != declared_srdf_sha256
        ):
            raise ValueError("official-model SRDF bytes differ from the replay contract")
    allowed_self_collision_pairs: list[dict[str, str]] = []
    srdf_filter_complete = False
    owned_tool_collider_paths = {}
    owned_tool_collider_corners_body_m = []
    compliant_cup_collider_paths = set()
    compliant_cup_index_by_path = {}
    official_robot_link_colliders = {}
    wrist_tool_exemption_records = []
    if srdf_path.is_file() and metadata.get("robot_model") == "fanuc_m710id_70":
        from unloading_sim.isaac_collision_policy import apply_robot_only_srdf_filters, colliders_by_physical_body

        # J6 also owns the 58 mounted tool collision shapes. Body-level J5/J6
        # filtering would silently exclude those tool/J5 pairs. Capture and
        # filter only the official robot meshes before adding the tool.
        allowed_self_collision_pairs = apply_robot_only_srdf_filters(
            stage, root_prim_path, srdf_path
        )
        srdf_filter_complete = True
        official_robot_link_colliders = colliders_by_physical_body(
            {name: str(prim.GetPath()) for name, prim in imported_link_prims.items()},
            [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)],
        )
        if any(len(paths) != 1 for paths in official_robot_link_colliders.values()):
            raise ValueError("official import must have one collision mesh per nearest physical link")
    elif srdf_path.is_file():
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
        "trailer": _color("trailer", (0.56, 0.60, 0.64)),
        "floor": _color("trailer", (0.56, 0.60, 0.64)),
        "static": _color("static", (0.08, 0.22, 0.36)),
        "amr": _color("amr", (0.055, 0.065, 0.075)),
        "chassis": _color("chassis", (0.08, 0.09, 0.11)),
        "conveyor": _color("conveyor", (0.035, 0.04, 0.045)),
        "carton": _color("carton", (0.47, 0.25, 0.095)),
    }
    UsdGeom.Xform.Define(stage, "/Validation/Materials")
    category_materials = {}
    for category in ("trailer", "floor", "carton", "conveyor"):
        spec_name = "trailer" if category == "floor" else (
            "conveyor_belt" if category == "conveyor" else category
        )
        spec = dict(pbr_specs.get(spec_name, {}))
        if spec:
            spec.setdefault("fallback_rgb", spec.pop("rgb", list(category_colors[category])))
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
    conveyor_visual_markers: dict[str, list[dict[str, object]]] = {}
    conveyor_visual_rollers: dict[str, list[dict[str, object]]] = {}
    if conveyor_enabled:
        UsdGeom.Xform.Define(stage, "/Validation/ConveyorMotionMarkers")
        UsdGeom.Xform.Define(stage, "/Validation/ConveyorEquipment")
        def _named_pbr_material(name, fallback):
            spec = dict(pbr_specs.get(name, {}))
            spec.setdefault("fallback_rgb", spec.pop("rgb", list(fallback)))
            return _preview_material(
                f"/Validation/Materials/{''.join(part.title() for part in name.split('_'))}",
                spec,
            )
        conveyor_frame_material = _named_pbr_material(
            "conveyor_frame", _color("conveyor_frame", (0.32, 0.36, 0.40))
        )
        conveyor_roller_material = _named_pbr_material(
            "conveyor_roller", _color("conveyor_roller", (0.18, 0.20, 0.22))
        )
        conveyor_marker_material = _preview_material(
            "/Validation/Materials/ConveyorMotionMarker",
            {
                "fallback_rgb": list(
                    _color("conveyor_motion_marker", (0.62, 0.67, 0.70))
                ),
                "roughness": 0.62,
                "metallic": 0.18,
            },
        )
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
            if str(primitive["name"]) in {"trailer_left_wall", "trailer_right_wall"}:
                inner_sign = -1.0 if str(primitive["name"]) == "trailer_left_wall" else 1.0
                for rib_index, local_x in enumerate(np.linspace(-0.44, 0.44, 7)):
                    rib = UsdGeom.Cube.Define(stage, f"{prim_path}/InteriorRib_{rib_index:02d}")
                    rib.CreateSizeAttr(1.0)
                    UsdShade.MaterialBindingAPI.Apply(rib.GetPrim()).Bind(category_materials[category])
                    rib_xform = UsdGeom.XformCommonAPI(rib.GetPrim())
                    # The rib remains inside the wall's physical OBB; it adds
                    # visible trailer structure without an unmodelled protrusion.
                    rib_xform.SetScale(Gf.Vec3f(0.018, 0.16, 0.94))
                    rib_xform.SetTranslate(Gf.Vec3d(float(local_x), 0.40 * inner_sign, 0.0))
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
            # The stationary side/bottom shell preserves the exact CPU deck
            # volume. Only the coplanar upper face has moving surface friction.
            # Splitting shapes avoids imparting belt velocity to its structure.
            shell = UsdGeom.Mesh.Define(stage, f"{prim_path}/StructureCollision")
            shell.CreatePointsAttr([
                Gf.Vec3f(x, y, z) for z in (-0.5, 0.5)
                for y in (-0.5, 0.5) for x in (-0.5, 0.5)
            ])
            shell.CreateFaceVertexCountsAttr([4] * 5)
            shell.CreateFaceVertexIndicesAttr([
                0, 2, 3, 1, 0, 1, 5, 4, 2, 6, 7, 3,
                0, 4, 6, 2, 1, 3, 7, 5,
            ])
            shell.CreateSubdivisionSchemeAttr().Set("none")
            UsdPhysics.CollisionAPI.Apply(shell.GetPrim())
            UsdGeom.Imageable(shell.GetPrim()).MakeInvisible()
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
            UsdGeom.Imageable(top_collision.GetPrim()).MakeInvisible()
            # PhysX surface velocity drives contact friction while the belt
            # body itself remains kinematic. The carton remains a fully
            # dynamic rigid body; its pose is never overwritten after release.
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(top_collision.GetPrim())
            rigid_body_api.CreateKinematicEnabledAttr().Set(True)
            surface_velocity_api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(top_collision.GetPrim())
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
            # Subtle belt seams and end rollers share the exact physical drive
            # state.  Their apparent motion is derived from per-surface travel
            # phase; all visual hard structure remains inside the existing
            # collision shell, so appearance adds no unmodelled obstruction.
            direction = conveyor_directions_world[primitive_name]
            if abs(float(direction[2])) > 1e-12:
                raise ValueError("conveyor motion markers require horizontal surface velocity")
            lateral = np.cross(np.array([0.0, 0.0, 1.0]), direction)
            local_direction = rotation.T @ direction
            local_lateral = rotation.T @ lateral
            half_size = 0.5 * size
            travel_half_extent = float(np.sum(np.abs(local_direction) * half_size))
            lateral_half_extent = float(np.sum(np.abs(local_lateral) * half_size))
            marker_count = max(5, int(math.ceil(2.0 * travel_half_extent / 0.20)))
            marker_width = max(0.04, 2.0 * lateral_half_extent - 0.08)
            top_center = center + rotation @ np.array([0.0, 0.0, half_size[2] + 0.003])
            yaw_deg = math.degrees(math.atan2(float(direction[1]), float(direction[0])))
            marker_records = []
            for marker_index, initial_offset in enumerate(
                np.linspace(-travel_half_extent, travel_half_extent, marker_count, endpoint=False)
            ):
                marker = UsdGeom.Cube.Define(
                    stage,
                    "/Validation/ConveyorMotionMarkers/"
                    f"{_safe_prim_name(primitive_name)}/Marker_{marker_index:02d}",
                )
                marker.CreateSizeAttr(1.0)
                marker.CreateDisplayColorAttr(
                    [_color("conveyor_motion_marker", (0.62, 0.67, 0.70))]
                )
                UsdShade.MaterialBindingAPI.Apply(marker.GetPrim()).Bind(
                    conveyor_marker_material
                )
                marker_xform = UsdGeom.XformCommonAPI(marker.GetPrim())
                marker_xform.SetScale(Gf.Vec3f(0.014, marker_width, 0.003))
                marker_xform.SetRotate(
                    Gf.Vec3f(0.0, 0.0, yaw_deg),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                marker_xform.SetTranslate(
                    Gf.Vec3d(*(top_center + direction * float(initial_offset)).tolist())
                )
                marker_records.append(
                    {
                        "xform": marker_xform,
                        "top_center": top_center.copy(),
                        "direction": direction.copy(),
                        "initial_offset_m": float(initial_offset),
                        "travel_half_extent_m": travel_half_extent,
                    }
                )
            conveyor_visual_markers[primitive_name] = marker_records
            equipment_root = (
                "/Validation/ConveyorEquipment/" + _safe_prim_name(primitive_name)
            )
            UsdGeom.Xform.Define(stage, equipment_root)
            for side_index, side_sign in enumerate((-1.0, 1.0)):
                rail = UsdGeom.Cube.Define(stage, f"{equipment_root}/FrameRail_{side_index}")
                rail.CreateSizeAttr(1.0)
                UsdShade.MaterialBindingAPI.Apply(rail.GetPrim()).Bind(conveyor_frame_material)
                rail_xform = UsdGeom.XformCommonAPI(rail.GetPrim())
                rail_center = (center + lateral * side_sign * max(0.0, lateral_half_extent - 0.025)
                               + np.array([0.0, 0.0, -0.045]))
                rail_xform.SetScale(Gf.Vec3f(2.0 * travel_half_extent, 0.035, 0.055))
                rail_xform.SetRotate(Gf.Vec3f(0.0, 0.0, yaw_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
                rail_xform.SetTranslate(Gf.Vec3d(*rail_center.tolist()))
            roller_records = []
            roller_axis = "Y" if abs(float(direction[0])) > 0.5 else "X"
            for end_index, end_sign in enumerate((-1.0, 1.0)):
                roller = UsdGeom.Cylinder.Define(stage, f"{equipment_root}/EndRoller_{end_index}")
                roller.CreateAxisAttr(roller_axis)
                roller.CreateRadiusAttr(0.032)
                roller.CreateHeightAttr(max(0.05, 2.0 * lateral_half_extent - 0.07))
                UsdShade.MaterialBindingAPI.Apply(roller.GetPrim()).Bind(conveyor_roller_material)
                roller_xform = UsdGeom.XformCommonAPI(roller.GetPrim())
                roller_center = (center + direction * end_sign * max(0.0, travel_half_extent - 0.035)
                                 + np.array([0.0, 0.0, half_size[2] - 0.036]))
                roller_xform.SetTranslate(Gf.Vec3d(*roller_center.tolist()))
                roller_records.append({"xform": roller_xform, "axis": roller_axis})
            conveyor_visual_rollers[primitive_name] = roller_records
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
    if not stage.GetPrimAtPath(grasp_body_path).IsValid():
        grasp_link_name = str(
            (metadata.get("robot_tool_mass_accounting") or {}).get(
                "grasp_body_link", grasp_suffix.rsplit("/", 1)[-1]
            )
        )
        grasp_candidates = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(root_prim_path)
            and prim.GetName() == grasp_link_name
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        if len(grasp_candidates) != 1:
            raise RuntimeError(
                f"cannot resolve unique imported grasp body {grasp_link_name!r}: {grasp_candidates}"
            )
        grasp_body_path = grasp_candidates[0]
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

        # Per-source-solid boxes preserve gaps in the Wantai assembly. The
        # verified representation retains rigid cup inserts as well as frame
        # solids; only explicitly classified compliant bellows use compliance.
        mass_properties_path = gripper_cfg.get("mass_properties_path")
        if not mass_properties_path:
            raise ValueError("gripper mass properties are required for collision proxies")
        audited_mass_path = (args.project_root.resolve() / mass_properties_path).resolve()
        with audited_mass_path.open("r", encoding="utf-8") as stream:
            audited_mass = json.load(stream)
        rigid_step_bounds = np.asarray(
            audited_mass.get("rigid_collision_bounding_boxes_step_mm", []), dtype=float
        )
        if metadata.get("robot_model") == "fanuc_m710id_70":
            from unloading_sim.tool_geometry import audit_tool_geometry
            qualified_tool_geometry = audit_tool_geometry(args.project_root.resolve())
            rigid_step_bounds = np.asarray(qualified_tool_geometry["rigid_collision_bounding_boxes_step_mm"], dtype=float)
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
                owned_tool_collider_paths[str(collision_proxy.GetPath())] = suction_tool_path
                owned_tool_collider_corners_body_m.append(
                    np.asarray(
                        [[x, y, z] for x in (tool_min[0], tool_max[0])
                         for y in (tool_min[1], tool_max[1])
                         for z in (tool_min[2], tool_max[2])],
                        dtype=float,
                    )
                )

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
        owned_tool_collider_paths[str(suction_tool.GetPath())] = suction_tool_path
        fallback_half = np.asarray(
            [0.01, 0.5 * float(footprint_size[0]), 0.5 * float(footprint_size[1])],
            dtype=float,
        )
        fallback_center = np.asarray([tool_uncompressed_face_x - 0.01, 0.0, 0.0])
        owned_tool_collider_corners_body_m.append(
            np.asarray(
                [fallback_center + [x, y, z] for x in (-fallback_half[0], fallback_half[0])
                 for y in (-fallback_half[1], fallback_half[1])
                 for z in (-fallback_half[2], fallback_half[2])],
                dtype=float,
            )
        )

    if metadata.get("robot_model") == "fanuc_m710id_70":
        from unloading_sim.collision_policy import SimulationCollisionPolicy
        from unloading_sim.isaac_collision_policy import apply_owned_tool_wrist_filters
        effective_collision_policy = SimulationCollisionPolicy.from_mapping(metadata.get("collision_policy"))

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
        commanded_visual_mask = list(gripper_cfg.get("commanded_active_mask", []))
        active_rubber_material = _preview_material(
            "/Validation/Materials/FG42RubberCommanded",
            {"fallback_rgb": [0.05, 0.65, 0.18], "roughness": 0.62},
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
            commanded_visual = (
                len(commanded_visual_mask) == physical_cup_count
                and commanded_visual_mask[cup_index] is True
            )
            UsdShade.MaterialBindingAPI.Apply(cup.GetPrim()).Bind(
                active_rubber_material if commanded_visual else rubber_material
            )
            if metadata.get("robot_model") == "fanuc_m710id_70":
                # Every cup remains physical independently of its vacuum bit.
                # The bellows is represented at the explicitly commanded
                # compression, while CAD inserts retain their rigid boxes.
                # This bounded compliance representation cannot hide insert
                # or frame penetration.
                compression = float(gripper_cfg["physical_cup_compression_m"])
                if not 0.0 <= compression <= 0.015:
                    raise ValueError("FG42 bellows compression exceeds the explicit 0-15 mm range")
                source_bounds = np.asarray(qualified_tool_geometry["compliant_bellows_bounds_step_mm"][cup_index])
                source_corners = np.array([[x, y, z] for x in (source_bounds[0], source_bounds[3])
                                           for y in (source_bounds[1], source_bounds[4])
                                           for z in (source_bounds[2], source_bounds[5])])
                bellows_corners = (source_corners - flange_origin_step_mm) @ step_from_tool_rotation * 1e-3
                bellows_lower, bellows_upper = bellows_corners.min(axis=0), bellows_corners.max(axis=0)
                collision_height = float(bellows_upper[0] - bellows_lower[0] - compression)
                collider_radius = float(np.max(bellows_upper[1:] - bellows_lower[1:]) / 2)
                collider = UsdGeom.Cylinder.Define(stage, f"{grasp_body_path}/CupCompressedCollision_{cup_index:02d}")
                collider.CreateAxisAttr(UsdGeom.Tokens.x)
                collider.CreateRadiusAttr(collider_radius)
                collider.CreateHeightAttr(collision_height)
                UsdGeom.XformCommonAPI(collider.GetPrim()).SetTranslate(Gf.Vec3d(
                    tool_contact_plane_x - 0.5 * collision_height, float(cup_y), float(cup_z)))
                UsdGeom.Imageable(collider.GetPrim()).MakeInvisible()
                UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
                owned_tool_collider_paths[str(collider.GetPath())] = suction_tool_path
                collider_center = np.asarray(
                    [tool_contact_plane_x - 0.5 * collision_height, float(cup_y), float(cup_z)],
                    dtype=float,
                )
                collider_half = np.asarray(
                    [0.5 * collision_height, collider_radius, collider_radius], dtype=float
                )
                owned_tool_collider_corners_body_m.append(
                    np.asarray(
                        [collider_center + [x, y, z]
                         for x in (-collider_half[0], collider_half[0])
                         for y in (-collider_half[1], collider_half[1])
                         for z in (-collider_half[2], collider_half[2])],
                        dtype=float,
                    )
                )
                compliant_cup_collider_paths.add(str(collider.GetPath()))
                compliant_cup_index_by_path[str(collider.GetPath())] = cup_index

    if (metadata.get("robot_model") == "fanuc_m710id_70"
            and effective_collision_policy.wrist_tool_exempt_links):
        wrist_tool_exemption_records = apply_owned_tool_wrist_filters(
            stage, official_robot_link_colliders, owned_tool_collider_paths,
            tool_owner=suction_tool_path,
            exempt_links=effective_collision_policy.wrist_tool_exempt_links,
        )

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
    ideal_independent_mode = (
        gripper_cfg.get("suction_mode") == IDEAL_INDEPENDENT_CUPS_MODE
    )
    cup_bit_order = list(gripper_cfg.get("mask_bit_order_cup_ids", []))
    eligible_cup_mask = list(gripper_cfg.get("geometrically_eligible_mask", []))
    commanded_cup_mask = list(gripper_cfg.get("commanded_active_mask", []))
    planned_fk_contact_mask = list(gripper_cfg.get("planned_fk_contact_mask", []))
    actual_contact_mask = [False] * physical_cup_count
    independent_cup_array = None
    if ideal_independent_mode:
        for name, mask in (
            ("geometrically_eligible_mask", eligible_cup_mask),
            ("commanded_active_mask", commanded_cup_mask),
            ("planned_fk_contact_mask", planned_fk_contact_mask),
        ):
            if len(mask) != physical_cup_count or any(type(value) is not bool for value in mask):
                raise ValueError(f"ideal independent-cup {name} must contain 72 booleans")
        if (
            len(cup_bit_order) != physical_cup_count
            or len(set(cup_bit_order)) != physical_cup_count
            or not any(commanded_cup_mask)
            or any(active and not eligible for active, eligible in zip(
                commanded_cup_mask, eligible_cup_mask, strict=True
            ))
        ):
            raise ValueError("ideal independent-cup masks and stable cup IDs are inconsistent")
        active_cup_indices = [
            index for index, active in enumerate(commanded_cup_mask) if active
        ]
        independent_cup_array = build_m710_independent_cup_array(
            rows=int(gripper_cfg.get("cup_rows", 0)),
            columns=int(gripper_cfg.get("cup_columns", 0)),
            pitch_m=gripper_cfg.get("cup_pitch_m", []),
            cup_radius_m=float(gripper_cfg.get("cup_radius_m", 0.0)),
            zone_count=int(gripper_cfg.get("zone_count", 0)),
        )
        if tuple(cup_bit_order) != independent_cup_array.cup_ids:
            raise ValueError(
                "ideal independent-cup bit order differs from the canonical stable IDs"
            )
        if planned_fk_contact_mask != commanded_cup_mask:
            raise ValueError(
                "planned actual-FK contact must include every commanded cup"
            )
    else:
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
        ideal_independent_mode
        or (
            args.gripper_model == "fixed_joint_diagnostic"
            and configured_shear_force is not None
            and configured_holding_torque is not None
        )
    )
    if (
        target_carton_path is not None
        and args.gripper_model == "surface_gripper"
        and not ideal_independent_mode
    ):
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

    contact_pairs: dict[tuple[str, str], dict[str, float | int | bool | str]] = {}
    active_contact_headers: set[tuple[str, str, str, str]] = set()
    from unloading_sim.isaac_collision_policy import (
        ActiveContactPairIndex, ContactPathCache, ContactReportProbe, read_effective_collision_offsets,
        PhysicalContactLedger, physical_support_contact_observed, robot_proximity_is_safety_relevant,
        premature_physical_conveyor_contacts, placement_support_window_start,
        classify_compliant_cup_contact, ZeroPointContactResolver,
    )
    active_contacts = ActiveContactPairIndex(active_contact_headers)
    contact_path_cache = ContactPathCache(PhysicsSchemaTools.intToSdfPath)
    contact_probe = ContactReportProbe() if args.diagnostic_only else None
    physical_contact_ledger = PhysicalContactLedger(
        metadata["actual_state_gates"]["support_max_gap_m"])
    contact_clock_s = [0.0]
    contact_trajectory_clock_s = [0.0]
    contact_callback_wall_s = [0.0]
    contact_callback_header_count = [0]
    contact_runtime_context = {"stage": "settling", "attached": False, "actual_free_space": False,
                               "release_validation_pending": False}
    unexpected_robot_contact_events = []
    zero_point_contact_resolver = ZeroPointContactResolver()

    def _contact_scope_token():
        return (contact_runtime_context["stage"], target_carton_path,
                contact_runtime_context["attached"], contact_runtime_context["actual_free_space"],
                contact_runtime_context["release_validation_pending"])

    def _classify_runtime_contact(record, actor0, actor1, collider0, collider1, separations, *, lost):
        key = (*tuple(sorted((actor0, actor1))), *tuple(sorted((collider0, collider1))))
        resolution = zero_point_contact_resolver.observe(key, separations, lost=lost, scope_token=_contact_scope_token())
        resolution_counts = record.setdefault("contact_data_resolution_counts", {})
        resolution_counts[resolution] = resolution_counts.get(resolution, 0) + 1
        if resolution in {"lost", "pending"}:
            return
        finite_separations = [float(value) for value in separations if math.isfinite(float(value))]
        lower = min(finite_separations) if finite_separations and len(finite_separations) == len(separations) else None
        reason = classify_compliant_cup_contact(
            collider0=collider0, collider1=collider1, actor0=actor0, actor1=actor1,
            compliant_cup_index_by_path=compliant_cup_index_by_path,
            commanded_mask=commanded_cup_mask, target_path=target_carton_path,
            stack_paths={f"/Validation/Scene/{_safe_prim_name(name)}" for name in metadata.get("stack_carton_names", [])},
            minimum_separation_m=lower, policy=effective_collision_policy,
            # Bellows collision shapes already encode nominal compression.
            # Only the manifest-derived remaining 15-10 mm travel is available
            # to a reported penetration; this never filters the physical pair.
            physical_compression_m=effective_collision_policy.maximum_compliant_cup_additional_compression_m,
            **contact_runtime_context,
        )
        classification = reason or "UNEXPECTED_ROBOT_OR_RIGID_TOOL_PROXIMITY"
        counts = record.setdefault("runtime_classification_event_counts", {})
        counts[classification] = counts.get(classification, 0) + 1
        per_shape = record.setdefault("runtime_collider_classifications", {})
        shape_key = " | ".join(sorted((collider0, collider1)))
        item = per_shape.setdefault(shape_key, {"classifications": [], "minimum_separation_m": None,
                                               "first_time_s": contact_clock_s[0], "last_time_s": contact_clock_s[0]})
        if classification not in item["classifications"]:
            item["classifications"].append(classification)
        item["last_time_s"] = contact_clock_s[0]
        if lower is not None:
            item["minimum_separation_m"] = lower if item["minimum_separation_m"] is None else min(item["minimum_separation_m"], lower)
        if reason is None:
            record["unexpected_runtime_event_count"] = record.get("unexpected_runtime_event_count", 0) + 1
            if not unexpected_robot_contact_events:
                unexpected_robot_contact_events.append({"time_s": contact_clock_s[0], "actors": [actor0, actor1],
                    "colliders": [collider0, collider1], "minimum_separation_m": lower,
                    "reason": classification, **contact_runtime_context})
    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        monitor_payload = prim_path in {*dynamic_scene_prim_paths, target_carton_path, released_payload_path}
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and (
            prim_path.startswith(root_prim_path) or monitor_payload
        ):
            contact_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            contact_api.CreateThresholdAttr(0.0)

    def _on_contact_report(headers, contact_data):
        callback_started = time.perf_counter()
        contact_callback_header_count[0] += len(headers)
        if contact_probe is not None:
            contact_probe.observe(headers, contact_data, contact_path_cache.resolve)
        for header in headers:
            actor0 = contact_path_cache.resolve(header.actor0)
            actor1 = contact_path_cache.resolve(header.actor1)
            robot_involved = actor0.startswith(root_prim_path) or actor1.startswith(root_prim_path)
            payload_paths = {path for path in (target_carton_path, released_payload_path) if path}
            payload_involved = actor0 in payload_paths or actor1 in payload_paths
            if not robot_involved and not payload_involved:
                continue
            pair = tuple(sorted((actor0, actor1)))
            collider0 = contact_path_cache.resolve(header.collider0)
            collider1 = contact_path_cache.resolve(header.collider1)
            contact_key = (*pair, *tuple(sorted((collider0, collider1))))
            event_type = header.type
            event_type_value = int(event_type)
            event_name = str(getattr(event_type, "name", event_type))
            record = contact_pairs.setdefault(
                pair,
                {
                    "event_count": 0,
                    "found_event_count": 0,
                    "persist_event_count": 0,
                    "lost_event_count": 0,
                    "peak_impulse_ns": 0.0,
                    "impulse_semantics": "peak_norm_of_per_event_vector_sum_over_contact_points",
                    "first_contact_time_s": contact_clock_s[0],
                    "last_contact_time_s": contact_clock_s[0],
                    "last_event_type": event_name,
                    "active": False,
                    "collider_pairs": [],
                },
            )
            record["event_count"] = int(record["event_count"]) + 1
            collider_pair = sorted((collider0, collider1))
            if collider_pair not in record["collider_pairs"]:
                record["collider_pairs"].append(collider_pair)
            record["last_contact_time_s"] = contact_clock_s[0]
            record["last_event_type"] = event_name
            if event_type_value == int(ContactEventType.CONTACT_LOST):
                record["lost_event_count"] = int(record["lost_event_count"]) + 1
            else:
                # subscribe_contact_report_events emits only FOUND, PERSIST and
                # LOST headers.  FOUND/PERSIST both prove a currently active
                # collider pair; LOST explicitly removes that exact pair.
                if event_type_value == int(ContactEventType.CONTACT_FOUND):
                    record["found_event_count"] = int(record["found_event_count"]) + 1
                else:
                    record["persist_event_count"] = int(record["persist_event_count"]) + 1
            active_count = active_contacts.update(
                contact_key, lost=event_type_value == int(ContactEventType.CONTACT_LOST)
            )
            record["active"] = active_count > 0
            record["active_collider_pair_count"] = active_count
            event_impulses = []
            event_separations = []
            for contact_index in range(
                int(header.contact_data_offset),
                int(header.contact_data_offset) + int(header.num_contact_data),
            ):
                try:
                    event_impulses.append(
                        np.asarray(contact_data[contact_index].impulse, dtype=float)
                    )
                    event_separations.append(float(contact_data[contact_index].separation))
                except (AttributeError, IndexError, TypeError, ValueError):
                    continue
            impulse = float(
                np.linalg.norm(np.sum(event_impulses, axis=0))
                if event_impulses
                else 0.0
            )
            record["peak_impulse_ns"] = max(float(record["peak_impulse_ns"]), impulse)
            physical_contact_ledger.observe(
                contact_key, event_separations,
                lost=event_type_value == int(ContactEventType.CONTACT_LOST),
                time_s=contact_clock_s[0], record=record,
                trajectory_time_s=contact_trajectory_clock_s[0],
            )
            if robot_involved:
                _classify_runtime_contact(record, actor0, actor1, collider0, collider1, event_separations,
                                          lost=event_type_value == int(ContactEventType.CONTACT_LOST))
        contact_callback_wall_s[0] += time.perf_counter() - callback_started

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
    # Author the exact initial joint state before PhysX first constructs the
    # articulation. The zero-delta bootstrap cannot sweep a default pose
    # through the stack before the first recorded sample.
    initial_by_name = dict(zip(expected_joint_names, positions[0], strict=True))
    for prim in stage.Traverse():
        if (str(prim.GetPath()).startswith(root_prim_path + "/")
                and prim.IsA(UsdPhysics.RevoluteJoint) and prim.GetName() in initial_by_name):
            q_degrees = float(np.degrees(initial_by_name[prim.GetName()]))
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTargetPositionAttr(q_degrees)
            drive.CreateTargetVelocityAttr(0.0)
            joint_state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
            joint_state.CreatePositionAttr(q_degrees)
            joint_state.CreateVelocityAttr(0.0)
    runtime_collision_offset_evidence = None
    if metadata.get("robot_model") == "fanuc_m710id_70":
        from unloading_sim.isaac_collision_policy import (
            author_explicit_collision_offsets, verify_authored_collision_offsets,
            verify_effective_collision_offsets,
        )
        explicit_collision_offsets = author_explicit_collision_offsets(
            stage, contact_offset_m=physics_contract["contact_offset_m"],
            rest_offset_m=physics_contract["rest_offset_m"],
        )
    world = World(stage_units_in_meters=1.0, physics_dt=0.0, rendering_dt=physics_dt * args.render_every)
    runtime_backend_evidence = None
    if metadata.get("robot_model") == "fanuc_m710id_70":
        from isaacsim.core.simulation_manager import SimulationManager
        requested_backend = physics_contract["execution_backend"]
        SimulationManager.set_physics_sim_device(requested_backend["device"])
        physics_context = world.get_physics_context()
        physics_context.set_broadphase_type(requested_backend["broadphase_type"])
        physics_context.enable_gpu_dynamics(requested_backend["gpu_dynamics_enabled"])
        physics_context.enable_fabric(requested_backend["fabric_enabled"])
        physics_context.enable_ccd(requested_backend["ccd_enabled"])

        def _read_physics_backend():
            gpu_enabled = bool(physics_context.is_gpu_dynamics_enabled())
            return {"mode": "physx_gpu" if gpu_enabled else "physx_cpu",
                    "device": SimulationManager.get_physics_sim_device(),
                    "broadphase_type": physics_context.get_broadphase_type(),
                    "gpu_dynamics_enabled": gpu_enabled,
                    "fabric_enabled": bool(SimulationManager.is_fabric_enabled()),
                    "ccd_enabled": bool(physics_context.is_ccd_enabled())}

        runtime_backend_evidence = verify_physics_backend_readback(requested_backend, _read_physics_backend())
        (args.output / "physics_backend_pre_reset.json").write_text(
            json.dumps(runtime_backend_evidence, indent=2), encoding="utf-8")
        print("FANUC_REPLAY_STAGE=physics_backend_verified " + json.dumps(runtime_backend_evidence), flush=True)
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
    all_carton_bodies = RigidPrim(dynamic_scene_prim_paths) if dynamic_scene_prim_paths else None
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
    from unloading_sim.robot import URDFRobot
    telemetry_robot = URDFRobot.from_urdf(
        urdf_path, active_joint_names=discovered_joint_names,
        tip_link=grasp_body_path.rsplit("/", 1)[-1],
        base_position=base_position, base_rpy=base_rpy, tool_length=0.0,
    )
    telemetry_robot.tip_from_tcp = np.eye(4)
    telemetry_robot.tip_from_tcp[:3, 3] = flange_offset
    telemetry_robot.tip_from_tcp[0, 3] += float(metadata["tool_length_m"])

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

    print("FANUC_REPLAY_STAGE=world_reset_started", flush=True)
    world.reset()
    print("FANUC_REPLAY_STAGE=world_reset_completed", flush=True)
    if metadata.get("robot_model") == "fanuc_m710id_70":
        runtime_backend_evidence = verify_physics_backend_readback(requested_backend, _read_physics_backend())
        (args.output / "physics_backend_post_reset.json").write_text(
            json.dumps(runtime_backend_evidence, indent=2), encoding="utf-8")
        offset_policy = {"contact_offset_m": physics_contract["contact_offset_m"],
                         "rest_offset_m": physics_contract["rest_offset_m"]}
        effective_offset_arrays = {
            "articulation": read_effective_collision_offsets(articulation._physics_articulation_view),
            "cartons": read_effective_collision_offsets(all_carton_bodies._physics_rigid_body_view),
        }
        (args.output / "collision_offset_effective_before_gate.json").write_text(
            json.dumps(effective_offset_arrays, indent=2), encoding="utf-8")
        runtime_collision_offset_evidence = {
            "schema": "m710_explicit_collision_offset_readback_v1", "policy": offset_policy,
            "authored_all_colliders": verify_authored_collision_offsets(stage, explicit_collision_offsets),
            "articulation": verify_effective_collision_offsets(
                effective_offset_arrays["articulation"],
                expected_shape_count=sum(len(paths) for paths in official_robot_link_colliders.values()) + len(owned_tool_collider_paths), **offset_policy),
            "cartons": verify_effective_collision_offsets(
                effective_offset_arrays["cartons"],
                expected_shape_count=len(dynamic_scene_prim_paths), **offset_policy),
        }
        (args.output / "collision_offset_readback.json").write_text(
            json.dumps(runtime_collision_offset_evidence, indent=2), encoding="utf-8")
        print("FANUC_REPLAY_STAGE=collision_offsets_verified " + json.dumps({
            **offset_policy, "all_collider_count": len(explicit_collision_offsets),
            "articulation_shape_count": runtime_collision_offset_evidence["articulation"]["shape_count"],
            "carton_shape_count": len(dynamic_scene_prim_paths)}), flush=True)
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
    articulation.set_dof_position_targets(initial[None, :])
    reset_q = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0]
    reset_q_error_rad = float(np.max(np.abs(reset_q - initial)))
    if reset_q_error_rad > 1.0e-5:
        raise RuntimeError(f"pre-reset authored joint state mismatch: {reset_q_error_rad:.9g} rad")
    world.set_simulation_dt(physics_dt=physics_dt, rendering_dt=physics_dt * args.render_every)
    gravity_feedforward_enabled = bool(metadata.get("joint_gravity_feedforward_enabled", False))
    velocity_feedforward_enabled = bool(metadata.get("joint_velocity_feedforward_enabled", False))
    payload_gravity_feedforward_enabled = bool(metadata.get("attached_payload_gravity_feedforward_enabled", False))
    from unloading_sim.m710_replay_physics import (
        finite_gravity_compensated_drive_target, payload_gravity_compensation,
        sample_joint_reference, BoundedFreeTransitGate, BoundedTargetCupReleaseClearance,
        resolve_actual_task_stage,
    )
    last_drive_feedforward = {"robot_gravity_nm": np.zeros(len(initial)),
                              "payload_gravity_nm": np.zeros(len(initial))}
    articulation.set_dof_velocity_targets(np.zeros_like(initial)[None, :])

    def _drive_target(reference):
        robot_gravity = (np.asarray(articulation.get_dof_gravity_compensation_forces().numpy(), dtype=float)[0]
                         if gravity_feedforward_enabled else np.zeros(len(reference)))
        payload_gravity = np.zeros(len(reference))
        if payload_gravity_feedforward_enabled and grasp_joint is not None:
            # The fixed joint carries an external body excluded from the robot
            # articulation model. Use the actual body COM (authored at its origin),
            # actual q and actual TCP; do not substitute the planned box pose.
            payload_mass_api = UsdPhysics.MassAPI(stage.GetPrimAtPath(target_carton_path))
            local_com = np.asarray(payload_mass_api.GetCenterOfMassAttr().Get(), dtype=float)
            mass_kg = float(payload_mass_api.GetMassAttr().Get())
            payload_positions, payload_quaternions = target_body.get_world_poses()
            payload_com = (np.asarray(payload_positions.numpy(), dtype=float)[0]
                           + _rotation_matrix_from_quaternion_wxyz(np.asarray(payload_quaternions.numpy())[0]) @ local_com)
            measured_q = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0]
            body_positions, body_quaternions = grasp_body.get_world_poses()
            actual_tcp = (np.asarray(body_positions.numpy(), dtype=float)[0]
                          + _rotation_matrix_from_quaternion_wxyz(np.asarray(body_quaternions.numpy())[0])
                          @ telemetry_robot.tip_from_tcp[:3, 3])
            jacobian = telemetry_robot.geometric_jacobian(measured_q)
            # Shift the FK Jacobian origin to the actual TCP before adding the
            # actual COM lever arm. This avoids mixing two origin conventions.
            jacobian[:3] += np.cross(jacobian[3:].T, actual_tcp - telemetry_robot.fk(measured_q)[:3, 3]).T
            payload_gravity = payload_gravity_compensation(
                jacobian, actual_tcp, payload_com, mass_kg, physics_contract["gravity_world_m_s2"])
        last_drive_feedforward.update(robot_gravity_nm=robot_gravity, payload_gravity_nm=payload_gravity)
        return finite_gravity_compensated_drive_target(
            reference, robot_gravity + payload_gravity, stiffness, effort_limits).astype(np.float32)
    settling_audit = {
        "status": "NOT_REQUIRED_LEGACY",
        "dynamic_body_count": len(dynamic_scene_bodies),
        "elapsed_s": 0.0,
    }
    if metadata.get("robot_model") == "fanuc_m710id_70":
        if not dynamic_scene_records or len(dynamic_scene_bodies) != len(dynamic_scene_records):
            raise RuntimeError("M-710 initial-state settling requires all snapshot dynamic cartons")
        settling_cfg = dict(physics_contract.get("settling", {}))
        maximum_settle_time = float(settling_cfg["maximum_settle_time_s"])
        required_stable_duration = float(settling_cfg["required_stable_duration_s"])
        max_linear_speed = float(settling_cfg["max_linear_speed_m_s"])
        max_angular_speed = float(settling_cfg["max_angular_speed_rad_s"])
        max_position_drift = float(settling_cfg["max_position_drift_m"])
        max_penetration = float(settling_cfg["max_penetration_m"])
        maximum_steps = int(math.ceil(maximum_settle_time / physics_dt))
        if args.diagnostic_only:
            maximum_steps = min(maximum_steps, args.diagnostic_settling_steps)
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
        checkpoint_stride = max(1, int(round(1.0 / physics_dt)))
        print("FANUC_REPLAY_STAGE=settling_started " + json.dumps({
            "maximum_steps": maximum_steps, "physics_dt_s": physics_dt,
            "reset_q_error_rad": reset_q_error_rad}), flush=True)
        for settle_step in range(maximum_steps):
            step_wall_started = time.perf_counter()
            contact_wall_before = contact_callback_wall_s[0]
            contact_headers_before = contact_callback_header_count[0]
            articulation.set_dof_position_targets(_drive_target(initial)[None, :])
            drive_wall_finished = time.perf_counter()
            world.step(render=False, update_fabric=True)
            physics_wall_finished = time.perf_counter()
            carton_states = []
            linear_speeds = []
            angular_speeds = []
            batch_positions, batch_orientations = all_carton_bodies.get_world_poses()
            batch_linear, batch_angular = all_carton_bodies.get_velocities()
            batch_positions, batch_orientations = np.asarray(batch_positions.numpy()), np.asarray(batch_orientations.numpy())
            batch_linear, batch_angular = np.asarray(batch_linear.numpy()), np.asarray(batch_angular.numpy())
            for body_index, record in enumerate(dynamic_scene_records):
                center = np.asarray(batch_positions[body_index], dtype=float)
                quaternion = np.asarray(batch_orientations[body_index], dtype=float)
                rotation = _rotation_matrix_from_quaternion_wxyz(quaternion)
                half_extents = 0.5 * np.asarray(record["size_m"], dtype=float)
                carton_states.append((center, half_extents, rotation))
                linear_speeds.append(
                    float(np.linalg.norm(batch_linear[body_index]))
                )
                angular_speeds.append(
                    float(np.linalg.norm(batch_angular[body_index]))
                )
            current_positions = np.asarray([item[0] for item in carton_states], dtype=float)
            tensor_wall_finished = time.perf_counter()
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
            sat_wall_finished = time.perf_counter()
            if settle_step < 10:
                timing = {
                    "schema": "m710_settling_step_wall_timing_v1", "step_index": settle_step,
                    "physical_time_s": (settle_step + 1) * physics_dt,
                    "drive_target_wall_s": drive_wall_finished - step_wall_started,
                    "physx_and_fabric_wall_s": physics_wall_finished - drive_wall_finished,
                    "actual_tensor_read_and_poses_wall_s": tensor_wall_finished - physics_wall_finished,
                    "full_penetration_audit_wall_s": sat_wall_finished - tensor_wall_finished,
                    "contact_callback_wall_s": contact_callback_wall_s[0] - contact_wall_before,
                    "contact_callback_headers": contact_callback_header_count[0] - contact_headers_before,
                    "total_step_wall_s": sat_wall_finished - step_wall_started,
                }
                (args.output / f"settling_step_timing_{settle_step + 1:06d}.json").write_text(
                    json.dumps(timing, indent=2), encoding="utf-8")
                print("FANUC_REPLAY_STAGE=settling_step_wall_timing " + json.dumps(timing), flush=True)
            final_linear_speed = max(linear_speeds, default=0.0)
            final_angular_speed = max(angular_speeds, default=0.0)
            final_penetration = current_penetration
            peak_penetration = max(peak_penetration, current_penetration)
            final_position_drift = float(
                np.max(np.linalg.norm(current_positions - configured_positions, axis=1))
            )
            peak_position_drift = max(peak_position_drift, final_position_drift)
            if settle_step == 0 or (settle_step + 1) % checkpoint_stride == 0 or settle_step + 1 == maximum_steps:
                checkpoint = {
                    "schema": "m710_actual_settling_checkpoint_v1",
                    "step_index": settle_step, "elapsed_s": (settle_step + 1) * physics_dt,
                    "maximum_steps": maximum_steps, "thresholds": settling_cfg,
                    "max_linear_speed_m_s": final_linear_speed,
                    "max_angular_speed_rad_s": final_angular_speed,
                    "current_max_penetration_m": final_penetration,
                    "peak_settling_penetration_m": peak_penetration,
                    "position_drift_from_configured_m": final_position_drift,
                    "peak_position_drift_from_configured_m": peak_position_drift,
                    "joint_names": discovered_joint_names,
                    "q_rad": np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0].tolist(),
                    "cartons": [{"name": record["name"],
                                 "position_m": batch_positions[index].tolist(),
                                 "orientation_wxyz": batch_orientations[index].tolist(),
                                 "linear_velocity_m_s": batch_linear[index].tolist(),
                                 "angular_velocity_rad_s": batch_angular[index].tolist()}
                                for index, record in enumerate(dynamic_scene_records)],
                }
                checkpoint_path = args.output / f"settling_checkpoint_{settle_step + 1:06d}.json"
                checkpoint_path.write_text(json.dumps(checkpoint, indent=2), encoding="utf-8")
                print("FANUC_REPLAY_STAGE=settling_checkpoint " + json.dumps({
                    key: value for key, value in checkpoint.items() if key != "cartons"}), flush=True)
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
            "status": "NOT_EVALUATED_DIAGNOSTIC_STEP_LIMIT" if args.diagnostic_only else ("PASS" if settling_passed else "FAIL"),
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
        if args.diagnostic_only:
            (args.output / "diagnostic_raw_contact_probe.json").write_text(
                json.dumps(contact_probe.as_dict(), indent=2), encoding="utf-8")
            (args.output / "diagnostic_effective_collision_offsets.json").write_text(json.dumps({
                "schema": "m710_effective_collision_offsets_v1",
                "articulation": read_effective_collision_offsets(articulation._physics_articulation_view),
                "cartons": read_effective_collision_offsets(all_carton_bodies._physics_rigid_body_view),
            }, indent=2), encoding="utf-8")
            # Read actual tensor body poses, never stale USD/Fabric world poses.
            actor_poses = {}
            link_transforms = np.asarray(articulation._physics_articulation_view.get_link_transforms().numpy())[0]
            for name, pose in zip(articulation.link_names, link_transforms, strict=True):
                path = next(str(prim.GetPath()) for prim in stage.Traverse()
                            if prim.GetName() == name and prim.HasAPI(UsdPhysics.RigidBodyAPI))
                actor_poses[path] = (pose[:3], _rotation_matrix_from_quaternion_wxyz(pose[[6, 3, 4, 5]]))
            carton_positions, carton_orientations = all_carton_bodies.get_world_poses()
            for path, position, orientation in zip(dynamic_scene_prim_paths, carton_positions.numpy(), carton_orientations.numpy(), strict=True):
                actor_poses[path] = (position, _rotation_matrix_from_quaternion_wxyz(orientation))
            bounds_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"], useExtentsHint=False, ignoreVisibility=True)
            static_xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            collider_rows = []
            for prim in stage.Traverse():
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                ancestor = prim
                while ancestor.IsValid() and not ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                    ancestor = ancestor.GetParent()
                actor_path = str(ancestor.GetPath()) if ancestor.IsValid() else None
                if actor_path in actor_poses:
                    if prim.IsA(UsdGeom.Mesh):
                        source_points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
                    elif prim.IsA(UsdGeom.Cube):
                        half = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2
                        source_points = np.array([[x, y, z] for x in (-half, half)
                                                  for y in (-half, half) for z in (-half, half)])
                    elif prim.IsA(UsdGeom.Cylinder):
                        shape = UsdGeom.Cylinder(prim)
                        halves = np.full(3, float(shape.GetRadiusAttr().Get()))
                        halves[{"X": 0, "Y": 1, "Z": 2}[str(shape.GetAxisAttr().Get()).upper()]] = float(shape.GetHeightAttr().Get()) / 2
                        source_points = np.array([[x, y, z] for x in (-halves[0], halves[0])
                                                  for y in (-halves[1], halves[1]) for z in (-halves[2], halves[2])])
                    else:
                        raise ValueError(f"diagnostic needs explicit geometry for dynamic collider {prim.GetPath()}")
                    # Remove only body rigid motion, preserving the body's own
                    # scale (carton Cube scale is its real physical size).
                    rigid_body_world = static_xform_cache.GetLocalToWorldTransform(ancestor).RemoveScaleShear()
                    local_to_body = static_xform_cache.GetLocalToWorldTransform(prim) * rigid_body_world.GetInverse()
                    corners = np.array([local_to_body.Transform(Gf.Vec3d(*point.tolist())) for point in source_points])
                    position, rotation = actor_poses[actor_path]
                    world_corners = corners @ rotation.T + position
                    world_lower, world_upper = world_corners.min(axis=0), world_corners.max(axis=0)
                    pose_source = "actual_physx_tensor_pose_with_static_body_relative_geometry_bounds"
                else:
                    world_box = bounds_cache.ComputeWorldBound(prim).ComputeAlignedBox()
                    world_lower, world_upper = np.asarray(world_box.GetMin()), np.asarray(world_box.GetMax())
                    pose_source = "static_usd_world_geometry" if actor_path is None else "unavailable_dynamic_pose_usd_not_actual"
                attributes = {}
                for attribute in prim.GetAttributes():
                    name = attribute.GetName()
                    if any(token in name.lower() for token in ("contact", "offset", "torsional", "margin", "gap")):
                        value = attribute.Get()
                        attributes[name] = {"value": str(value), "authored": attribute.HasAuthoredValueOpinion()}
                for name, declared_default in {
                    "physxCollision:contactOffset": "-inf (SDK sentinel; not a measured effective offset)",
                    "physxCollision:restOffset": "-inf (SDK sentinel; not a measured effective offset)",
                    "physxCollision:torsionalPatchRadius": "0",
                    "physxCollision:minTorsionalPatchRadius": "0",
                    "newton:contactGap": "-inf (SDK builder-default sentinel)",
                    "newton:contactMargin": "0",
                }.items():
                    attribute = prim.GetAttribute(name)
                    attributes.setdefault(name, {"value": str(attribute.Get()) if attribute else None,
                                                 "authored": bool(attribute and attribute.HasAuthoredValueOpinion()),
                                                 "declared_sdk_default": declared_default})
                collider_rows.append({"collider": str(prim.GetPath()), "actor": actor_path,
                                      "type": prim.GetTypeName(), "applied_schemas": prim.GetAppliedSchemas(),
                                      "world_aabb_lower_m": world_lower.tolist(), "world_aabb_upper_m": world_upper.tolist(),
                                      "world_aabb_semantics": "source_mesh_vertices_or_exact_primitive_bounds_with_actual_rigid_pose",
                                      "pose_source": pose_source, "collision_attributes": attributes})
            (args.output / "diagnostic_collider_geometry.json").write_text(json.dumps({
                "schema": "m710_actual_collider_geometry_probe_v1", "colliders": collider_rows,
                "read_only": True,
            }, indent=2), encoding="utf-8")
            (args.output / "diagnostic_contact_pairs.json").write_text(json.dumps({
                "schema": "m710_diagnostic_contact_pairs_v1",
                "raw_header_count": contact_callback_header_count[0],
                "processed_event_count": sum(int(record["event_count"]) for record in contact_pairs.values()),
                "active_contact_key_count": len(active_contact_headers),
                "active_physical_contact_key_count": len(physical_contact_ledger.active_headers),
                "physical_contact_tolerance_m": physical_contact_ledger.contact_tolerance_m,
                "physical_contact_tolerance_source": "metadata.actual_state_gates.support_max_gap_m",
                "contact_pairs": [{"actors": list(pair), **record}
                                  for pair, record in sorted(contact_pairs.items())],
                "scope": "unchanged robot and selected/released payload callback records",
            }, indent=2), encoding="utf-8")
            (args.output / "diagnostic_settling_result.json").write_text(json.dumps({
                "status": "DIAGNOSTIC_ONLY_NOT_PHYSICAL_QUALIFICATION",
                "requested_step_limit": args.diagnostic_settling_steps,
                "actual_settling_audit": settling_audit,
                "physics_execution_backend": runtime_backend_evidence,
                "collision_offset_readback": runtime_collision_offset_evidence,
                "bundle_payload_sha256": pre_simulation_integrity_gate["bundle_payload_sha256"],
                "physical_cycle_completed": False, "simulation_qualification_passed": False,
                "no_pick_or_attachment_attempted": True,
            }, indent=2), encoding="utf-8")
            raise DiagnosticSettlingComplete("diagnostic settling step budget completed; no pick qualification")
        if not settling_passed:
            raise RuntimeError(
                "M-710 initial snapshot cartons failed the bounded settling/penetration gate"
            )
    else:
        for _ in range(10):
            world.step(render=False, update_fabric=True)

    rep.orchestrator.step(rt_subframes=4, pause_timeline=False, delta_time=0.0, wait_for_render=True)
    rgb_annotator.get_data()
    capture_max_joint_delta_rad = 0.0
    capture_max_carton_delta_m = 0.0
    actual_frame_states = []

    def _capture_carton_states() -> list[dict[str, object]]:
        states: list[dict[str, object]] = []
        if all_carton_bodies is None:
            return states
        batch_positions, batch_orientations = all_carton_bodies.get_world_poses()
        batch_linear, batch_angular = all_carton_bodies.get_velocities()
        batch_positions, batch_orientations = np.asarray(batch_positions.numpy()), np.asarray(batch_orientations.numpy())
        batch_linear, batch_angular = np.asarray(batch_linear.numpy()), np.asarray(batch_angular.numpy())
        for body_index, (record, prim_path) in enumerate(zip(
            dynamic_scene_records,
            dynamic_scene_prim_paths,
            strict=True,
        )):
            states.append(
                {
                    "name": str(record["name"]),
                    "prim_path": str(prim_path),
                    "center_m": batch_positions[body_index].tolist(),
                    "quaternion_wxyz": batch_orientations[body_index].tolist(),
                    "linear_velocity_m_s": batch_linear[body_index].tolist(),
                    "angular_velocity_rad_s": batch_angular[body_index].tolist(),
                    "size_m": list(record["size_m"]),
                    "mass_kg": float(record["mass_kg"]),
                }
            )
        return states

    session_output_root = args.output
    session_segment_index = 0
    session_time_offset_s = 0.0
    hud_fonts = _load_hud_fonts(args.height)
    hud_chinese_enabled = bool(hud_fonts[2])
    hud_font_path = str(hud_fonts[3])
    if args.continuation_dir is not None:
        args.continuation_dir.mkdir(parents=True, exist_ok=True)
        if not ideal_independent_mode:
            raise ValueError("same-world continuation currently requires ideal_independent_cups")
    while True:
        actual_frame_states = []
        capture_max_joint_delta_rad = 0.0
        capture_max_carton_delta_m = 0.0
        settled_carton_states = _capture_carton_states()
        stack_monitor = None
        runtime_stop_reason = None
        stack_monitor_history = []
        def _state_obb(item):
            return OBB(np.asarray(item["center_m"], dtype=float),
                       0.5 * np.asarray(item["size_m"], dtype=float),
                       _rotation_matrix_from_quaternion_wxyz(item["quaternion_wxyz"]),
                       str(item["name"]), "carton")
        if metadata.get("robot_model") == "fanuc_m710id_70":
            from unloading_sim.m710_replay_physics import ActualStackContactMonitor
            initial_actual_boxes = {_state_obb(item).name: _state_obb(item) for item in settled_carton_states}
            stack_names = set(metadata.get("stack_carton_names") or initial_actual_boxes)
            stack_monitor = ActualStackContactMonitor(
                initial_actual_boxes[str(metadata["target"])],
                [box for name, box in initial_actual_boxes.items() if name in stack_names],
                effective_collision_policy,
            )
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
        actual_state_gates = dict(metadata.get("actual_state_gates", {}))
        maximum_contact_wait_s = float(actual_state_gates.get("maximum_contact_wait_s", 0.5))
        maximum_support_wait_s = float(actual_state_gates.get("maximum_support_wait_s", 0.75))
        maximum_free_transit_wait_s = float(actual_state_gates.get("maximum_free_transit_wait_s", 0.0))
        maximum_release_clearance_wait_s = float(
            actual_state_gates.get("maximum_release_clearance_wait_s", 1.0)
        )
        free_transit_gate = (BoundedFreeTransitGate(metadata["free_transit_start_time_seconds"], maximum_free_transit_wait_s)
                             if metadata.get("free_transit_start_time_seconds") is not None else None)
        target_cup_release_gate = BoundedTargetCupReleaseClearance(
            maximum_release_clearance_wait_s
        )
        target_cup_release_logged_events = 0
        physical_runtime_limit = (replay_duration + maximum_contact_wait_s + maximum_support_wait_s
                                  + maximum_free_transit_wait_s + target_cup_release_gate.maximum_wait_s)
        if args.max_sim_seconds is not None:
            # A short diagnostic's wall-independent physical duration cap is
            # not extended by unused wait budgets. Full runs retain them all.
            physical_runtime_limit = min(physical_runtime_limit, float(args.max_sim_seconds))
        physics_steps = int(math.ceil(physical_runtime_limit / physics_dt)) + 1
        measured_rows: list[np.ndarray] = []
        commanded_rows: list[np.ndarray] = []
        commanded_velocity_rows: list[np.ndarray] = []
        payload_gravity_feedforward_rows: list[np.ndarray] = []
        projected_force_rows: list[np.ndarray] = []
        gravity_force_rows: list[np.ndarray] = []
        measured_velocity_rows: list[np.ndarray] = []
        drive_effort_rows: list[np.ndarray] = []
        model_inverse_dynamics_rows: list[np.ndarray] = []
        external_joint_load_rows: list[np.ndarray] = []
        replay_frames: list[Image.Image] = []
        replay_video_path = args.output / "replay.mp4"
        replay_video_writer = None
        replay_video_frame_count = 0
        last_video_frame = None
        saved_phase_keyframes = set()
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
        ideal_actual_contact_count_at_attach = None
        release_commanded = False
        release_executed = False
        release_command_succeeded = False
        release_open_confirmed = False
        release_executed_time_s = None
        ideal_release_request_step = None
        ideal_release_request_time_s = None
        ideal_release_relative_position_at_request = None
        ideal_release_relative_quaternion_at_request = None
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
        independent_contact_audit = None
        support_contact_audit = None
        support_contact_report_observed = False
        contact_wait_started_s = None
        support_wait_started_s = None
        trajectory_time = 0.0
        event_log: list[dict[str, object]] = []
        cup_mask_change_log: list[dict[str, object]] = []
        previous_measured_velocity = None
        inverse_dynamics_available_all_steps = True
        inverse_dynamics_available_after_first_difference = True
        drive_effort_source = None
        drive_effort_output_qualified = False
        grasp_local_position = None
        grasp_local_quaternion = None
        peak_payload_attachment_position_error_m = 0.0
        peak_payload_attachment_rotation_error_rad = 0.0
        grasp_event_time = metadata.get("grasp_time_seconds")
        release_event_time = metadata.get("release_time_seconds")
        expected_place_center = np.asarray(metadata.get("place_center_m", []), dtype=float)
        place_surface = str(metadata.get("place_surface") or "")
        release_support_primitive = next(
            (
                primitive
                for primitive in scene_primitives
                if str(primitive.get("name", "")) == place_surface
            ),
            None,
        )
        if release_event_time is not None and release_support_primitive is None:
            raise ValueError("release requires one declared receiving support primitive")
        selected_support_names = tuple(metadata.get("selected_place_support_names")
                                       or metadata.get("place_support_names") or [place_surface])
        release_support_primitives = [primitive for primitive in scene_primitives
                                      if str(primitive.get("name", "")) in selected_support_names]
        if release_event_time is not None and len(release_support_primitives) != len(selected_support_names):
            raise ValueError("every declared receiving support must remain in the physical scene")
        release_support_paths = {f"/Validation/Scene/{_safe_prim_name(name)}" for name in selected_support_names}

        def _actual_support_contact_observed():
            return physical_support_contact_observed(
                physical_contact_ledger.active_headers, target_carton_path, release_support_paths)
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
        transport_cfg = dict(conveyor_cfg.get("transport", {}))
        conveyor_transport_minimum_distance_m = float(
            transport_cfg.get("minimum_visible_distance_m", 0.20)
        )
        conveyor_transport_target_distance_m = float(
            transport_cfg.get("target_distance_m", 0.60)
        )
        conveyor_safe_tail_margin_m = float(
            transport_cfg.get("safe_tail_margin_m", 0.10)
        )
        conveyor_transport_speed_tolerance_m_s = float(
            transport_cfg.get("speed_tolerance_m_s", 0.15)
        )
        conveyor_transport_audit_window_s = float(
            transport_cfg.get("audit_window_seconds", 3.0)
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
        conveyor_start_event_time = 0.0 if conveyor_start_policy == "immediate" else None
        if (
            conveyor_enabled
            and conveyor_start_policy == "after_release_retreat"
            and metadata.get("release_retreat_time_seconds") is None
        ):
            raise ValueError(
                f"{conveyor_start_policy} conveyor requires its corresponding release event time"
            )
        conveyor_started = bool(conveyor_enabled and conveyor_start_policy == "immediate")
        conveyor_started_time_s = 0.0 if conveyor_started else None
        conveyor_running = conveyor_started
        conveyor_stopped_time_s = None
        conveyor_stop_reason = None
        conveyor_visual_phase_m = {
            name: 0.0 for name in conveyor_visual_markers
        }
        conveyor_start_interlock_history = []
        target_center_at_conveyor_start = None

        def _actual_conveyor_start_interlock() -> dict[str, object]:
            required = float(dict(conveyor_cfg.get("actual_start_interlock", {})).get(
                "require_tool_target_clearance_m", 0.0202
            ))
            result = {
                "attachment_released": bool(release_open_confirmed),
                "target_independent": bool(
                    release_open_confirmed and not target_cup_release_gate.pending
                ),
                "tool_target_clearance_m": None,
                "required_clearance_m": required,
                "transport_swept_conflict_clear": False,
                "ready": False,
            }
            if target_carton_path is None or target_body is None:
                return result
            if len(owned_tool_collider_corners_body_m) != len(owned_tool_collider_paths):
                raise RuntimeError("live conveyor interlock is missing audited tool collider geometry")
            direction = conveyor_initial_direction_world
            if direction is None:
                return result
            target_positions, target_orientations = target_body.get_world_poses()
            body_positions, body_orientations = grasp_body.get_world_poses()
            clearance = swept_payload_tool_clearance(
                payload_center_m=np.asarray(target_positions.numpy(), dtype=float)[0],
                payload_half_extents_m=0.5 * np.asarray(target_primitive["size_m"], dtype=float),
                payload_rotation=_rotation_matrix_from_quaternion_wxyz(
                    np.asarray(target_orientations.numpy(), dtype=float)[0]
                ),
                transport_direction_world=direction,
                transport_distance_m=conveyor_transport_minimum_distance_m,
                tool_body_center_m=np.asarray(body_positions.numpy(), dtype=float)[0],
                tool_body_rotation=_rotation_matrix_from_quaternion_wxyz(
                    np.asarray(body_orientations.numpy(), dtype=float)[0]
                ),
                tool_collider_corners_body_m=owned_tool_collider_corners_body_m,
            )
            result["tool_target_clearance_m"] = clearance
            result["transport_swept_conflict_clear"] = clearance >= required
            result["ready"] = bool(
                result["attachment_released"]
                and result["target_independent"]
                and result["transport_swept_conflict_clear"]
            )
            return result

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

        def _update_conveyor_visual_markers(time_step_s: float) -> None:
            for surface_name, records in conveyor_visual_markers.items():
                moving = surface_name in active_conveyor_surfaces
                if moving:
                    conveyor_visual_phase_m[surface_name] += conveyor_speed_m_s * time_step_s
                phase_m = conveyor_visual_phase_m[surface_name]
                for record in records:
                    half = float(record["travel_half_extent_m"])
                    span = 2.0 * half
                    offset = float(record["initial_offset_m"])
                    if span > 0.0:
                        offset = (offset + phase_m + half) % span - half
                    position = (
                        np.asarray(record["top_center"], dtype=float)
                        + np.asarray(record["direction"], dtype=float) * offset
                    )
                    record["xform"].SetTranslate(Gf.Vec3d(*position.tolist()))
                roller_angle_deg = math.degrees(phase_m / 0.032)
                for record in conveyor_visual_rollers.get(surface_name, []):
                    rotation = (Gf.Vec3f(roller_angle_deg, 0.0, 0.0)
                                if record["axis"] == "X"
                                else Gf.Vec3f(0.0, roller_angle_deg, 0.0))
                    record["xform"].SetRotate(
                        rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ
                    )

        for step in range(physics_steps):
            simulation_time = step * physics_dt
            hold_trajectory = False
            if stack_monitor is not None and grasp_enabled and not release_commanded and free_transit_gate is not None:
                previous_event_count = len(free_transit_gate.events)
                gate_result = free_transit_gate.evaluate(trajectory_time, simulation_time, stack_monitor.free_space_reached)
                hold_trajectory = gate_result["hold"]
                for event in free_transit_gate.events[previous_event_count:]:
                    event_log.append(event)
                    print("FANUC_REPLAY_EVENT=" + json.dumps(event), flush=True)
                if gate_result["reason"]:
                    runtime_stop_reason = gate_result["reason"]
                    break
            # Commands/events above world.step use the pre-step time.  Contact
            # callbacks and measured state are produced by the completed step.
            contact_clock_s[0] = simulation_time + physics_dt
            conveyor_start_due = bool(
                conveyor_start_policy == "immediate"
                or conveyor_start_policy == "after_release" and release_executed
                or conveyor_start_policy == "after_release_retreat"
                and release_executed
                and trajectory_time
                >= float(metadata.get("release_retreat_time_seconds", float("inf")))
            )
            if conveyor_enabled and not conveyor_started and conveyor_start_due:
                interlock = _actual_conveyor_start_interlock()
                conveyor_start_interlock_history.append({
                    "time_s": float(simulation_time), **interlock,
                })
                if interlock["ready"]:
                    conveyor_started = True
                    conveyor_running = True
                    conveyor_started_time_s = simulation_time
                    positions_at_start, _ = target_body.get_world_poses()
                    target_center_at_conveyor_start = np.asarray(
                        positions_at_start.numpy(), dtype=float
                    )[0].copy()
                    target_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(
                        stage.GetPrimAtPath(target_carton_path)
                    )
                    target_rigid_api.CreateSleepThresholdAttr().Set(0.0)
                    print(
                        f"FANUC_REPLAY_EVENT=conveyor_started time_s={simulation_time:.6f} "
                        f"speed_m_s={conveyor_speed_m_s:.6f} actual_interlock=true",
                        flush=True,
                    )
            if conveyor_enabled:
                payload_center_for_drive = None
                if conveyor_running and target_body is not None:
                    drive_positions, _ = target_body.get_world_poses()
                    payload_center_for_drive = np.asarray(
                        drive_positions.numpy(), dtype=float
                    )[0]
                desired_surfaces = select_active_conveyor_surfaces(
                    payload_center_m=payload_center_for_drive,
                    conveyor_primitives=conveyor_primitives,
                    started=conveyor_running,
                    exclusive=conveyor_exclusive,
                    current_surface=active_conveyor_surface,
                    preferred_initial_surface=(
                        place_surface if not conveyor_selection_initialized else None
                    ),
                )
                _apply_conveyor_surface_selection(desired_surfaces, simulation_time)
                if conveyor_running:
                    conveyor_selection_initialized = True
            if (
                target_body is not None
                and not grasp_commanded
                and grasp_event_time is not None
                and trajectory_time >= float(grasp_event_time)
            ):
                if contact_wait_started_s is None:
                    contact_wait_started_s = simulation_time
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
                actual_contact_accepted = physical_contact_audit.accepted
                if not physical_contact_audit.accepted:
                    actual_contact_accepted = False
                if ideal_independent_mode:
                    world_from_grasp = np.eye(4)
                    world_from_grasp[:3, :3] = body_rotation_matrix
                    world_from_grasp[:3, 3] = body_position
                    grasp_from_contact = np.eye(4)
                    grasp_from_contact[:3, :3] = np.asarray(
                        [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
                        dtype=float,
                    )
                    grasp_from_contact[:3, 3] = [tool_contact_plane_x, 0.0, 0.0]
                    target_obb = OBB(
                        center=carton_position,
                        half_extents=0.5 * np.asarray(target_primitive["size_m"], dtype=float),
                        rotation=carton_rotation_matrix,
                        name=str(metadata["target"]),
                        category="carton",
                    )
                    current_geometry = evaluate_independent_cup_geometry(
                        world_from_grasp @ grasp_from_contact,
                        target_obb,
                        str(gripper_cfg["target_face"]),
                        independent_cup_array,
                        max_attachment_gap_m=float(gripper_cfg["max_grip_distance_m"]),
                        maximum_penetration_m=float(
                            gripper_cfg["maximum_contact_penetration_m"]
                        ),
                        max_normal_misalignment_rad=float(
                            gripper_cfg["max_normal_misalignment_rad"]
                        ),
                    )
                    # Re-command the nonempty, actually sealed set explicitly.
                    # The mask change is recorded before constraint creation;
                    # no original cup is silently counted as still in contact.
                    runtime_mask = list(current_geometry.geometrically_eligible_mask)
                    if any(runtime_mask) and runtime_mask != commanded_cup_mask:
                        cup_mask_change_log.append({
                            "simulation_time_s": simulation_time,
                            "reason": "RESELECT_FROM_ACTUAL_FULL_RING_CONTACT",
                            "previous_commanded_mask": list(commanded_cup_mask),
                            "commanded_mask": runtime_mask,
                            "commanded_ids": [cup_id for cup_id, active in zip(cup_bit_order, runtime_mask, strict=True) if active],
                        })
                        commanded_cup_mask = runtime_mask
                        eligible_cup_mask = list(runtime_mask)
                    commanded_ids = [
                        cup_id
                        for cup_id, active in zip(
                            cup_bit_order, commanded_cup_mask, strict=True
                        )
                        if active
                    ]
                    independent_contact_audit = audit_actual_independent_cup_contacts(
                        current_geometry,
                        commanded_ids,
                        pose_source="isaac_actual_grasp_body_and_target_state",
                    )
                    actual_contact_mask = list(
                        independent_contact_audit.actual_contact_mask
                    )
                    commanded_penetration = any(
                        commanded_cup_mask[contact.index]
                        and contact.reason == "CUP_RING_PENETRATES_TARGET"
                        for contact in current_geometry.contacts
                    )
                    actual_contact_accepted = bool(
                        any(commanded_cup_mask)
                        and actual_contact_mask == list(commanded_cup_mask)
                        and not commanded_penetration
                    )
                if not actual_contact_accepted:
                    grasp_command_succeeded = False
                elif ideal_independent_mode:
                    # Ideal holding capacity is an explicit assumption, but the
                    # joint is created only after actual target contact.  Body1 is
                    # the original target carton and is never cloned or teleported.
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
                        grasp_joint.CreateLocalRot1Attr(
                            Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
                        )
                        # The ideal holding assumption grants no body collision
                        # bypass. Rigid inserts and the real carton remain solid.
                        grasp_joint.CreateCollisionEnabledAttr(True)
                        grasp_joint.CreateExcludeFromArticulationAttr(True)
                        grasp_joint.CreateJointEnabledAttr(True)
                    grasp_command_succeeded = True
                    grasp_enabled = True
                    grasp_closed_time = simulation_time
                    ideal_actual_contact_count_at_attach = int(sum(actual_contact_mask))
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
                grasp_commanded = bool(grasp_command_succeeded)
                if not grasp_commanded:
                    hold_trajectory = True
                    if simulation_time - float(contact_wait_started_s) > maximum_contact_wait_s:
                        raise RuntimeError(
                            "bounded actual-contact wait expired before a valid target attachment"
                        )
                event_log.append(
                    {
                        "event": "grasp_contact_attempt",
                        "simulation_time_s": simulation_time,
                        "trajectory_time_s": trajectory_time,
                        "accepted": grasp_commanded,
                        "target": str(metadata["target"]),
                        "actual_contact_count": int(sum(actual_contact_mask))
                        if ideal_independent_mode
                        else physical_contact_audit.within_gap_count,
                    }
                )
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
                and trajectory_time >= float(release_event_time)
            ):
                if support_wait_started_s is None:
                    support_wait_started_s = simulation_time
                release_positions, release_orientations = target_body.get_world_poses()
                target_center_at_release = np.asarray(release_positions.numpy(), dtype=float)[0]
                target_orientation_at_release = np.asarray(
                    release_orientations.numpy(), dtype=float
                )[0]
                linear_velocity, angular_velocity = target_body.get_velocities()
                release_linear_velocity_before_m_s = np.asarray(
                    linear_velocity.numpy(), dtype=float
                )[0]
                release_angular_velocity_before_rad_s = np.asarray(
                    angular_velocity.numpy(), dtype=float
                )[0]
                support_contact_audit = audit_payload_support_contact(
                    payload_center_m=target_center_at_release,
                    payload_rotation=_rotation_matrix_from_quaternion_wxyz(
                        target_orientation_at_release
                    ),
                    payload_size_m=target_primitive["size_m"],
                    payload_linear_velocity_m_s=release_linear_velocity_before_m_s,
                    payload_angular_velocity_rad_s=release_angular_velocity_before_rad_s,
                    support=release_support_primitive,
                    supports=release_support_primitives,
                    max_support_gap_m=float(
                        actual_state_gates.get("support_max_gap_m", 0.003)
                    ),
                    maximum_penetration_m=float(
                        actual_state_gates.get("support_maximum_penetration_m", 0.001)
                    ),
                    minimum_footprint_overlap_ratio=float(
                        actual_state_gates.get(
                            "support_minimum_footprint_overlap_ratio", 0.90
                        )
                    ),
                    max_support_tilt_rad=float(
                        actual_state_gates.get("support_max_tilt_rad", np.deg2rad(5.0))
                    ),
                    max_linear_speed_m_s=float(
                        actual_state_gates.get("support_max_linear_speed_m_s", 0.03)
                    ),
                    max_angular_speed_rad_s=float(
                        actual_state_gates.get("support_max_angular_speed_rad_s", 0.08)
                    ),
                )
                support_prim_path = conveyor_surface_paths.get(
                    place_surface,
                    f"/Validation/Scene/{_safe_prim_name(place_surface)}",
                )
                support_contact_report_observed = _actual_support_contact_observed()
                support_release_accepted = bool(
                    support_contact_audit.accepted and support_contact_report_observed
                )
                if not support_release_accepted:
                    hold_trajectory = True
                    if simulation_time - float(support_wait_started_s) > maximum_support_wait_s:
                        raise RuntimeError(
                            "bounded support wait expired before the target reached its declared receiver"
                        )
                elif ideal_independent_mode:
                    enabled_attr = grasp_joint.GetPrim().GetAttribute("physics:jointEnabled")
                    if enabled_attr.IsValid():
                        enabled_attr.Set(False)
                    stage.RemovePrim(grasp_joint_path)
                    grasp_joint = None
                    release_command_succeeded = True
                    ideal_release_request_step = step
                    ideal_release_request_time_s = simulation_time
                    target_cup_release_gate.begin(simulation_time)
                    body_positions, body_orientations = grasp_body.get_world_poses()
                    body_position = np.asarray(body_positions.numpy(), dtype=float)[0]
                    body_quaternion = np.asarray(body_orientations.numpy(), dtype=float)[0]
                    body_rotation = _rotation_matrix_from_quaternion_wxyz(body_quaternion)
                    ideal_release_relative_position_at_request = body_rotation.T @ (
                        target_center_at_release - body_position
                    )
                    ideal_release_relative_quaternion_at_request = _quaternion_multiply_wxyz(
                        _quaternion_conjugate_wxyz(body_quaternion),
                        target_orientation_at_release,
                    )
                elif args.gripper_model == "surface_gripper":
                    open_results = [
                        bool(surface_gripper_interface.open_gripper(path))
                        for path in surface_gripper_paths
                    ]
                    release_command_succeeded = bool(open_results and all(open_results))
                    release_executed = release_command_succeeded
                    release_velocity_sample_pending = release_executed
                elif support_contact_audit.accepted:
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
                if release_executed and release_executed_time_s is None:
                    release_executed_time_s = simulation_time
                release_commanded = bool(
                    support_release_accepted and release_command_succeeded
                )
                event_log.append(
                    {
                        "event": "release_support_attempt",
                        "simulation_time_s": simulation_time,
                        "trajectory_time_s": trajectory_time,
                        "support": place_surface,
                        "support_geometry_accepted": support_contact_audit.accepted,
                        "support_contact_report_observed": support_contact_report_observed,
                        "support_release_accepted": support_release_accepted,
                        "support_reason": support_contact_audit.reason,
                        "release_executed": release_executed,
                    }
                )
                print(
                    f"FANUC_REPLAY_EVENT=release_command time_s={simulation_time:.6f} "
                    f"model={'ideal_fixed_constraint' if ideal_independent_mode else args.gripper_model} "
                    f"support={support_contact_audit.reason or 'PASS'} "
                    f"accepted={release_command_succeeded}",
                    flush=True,
                )
            sampled_position, sampled_velocity = sample_joint_reference(
                timestamps, positions, trajectory_time, held=hold_trajectory or not velocity_feedforward_enabled)
            command_source_order = sampled_position.astype(np.float32)
            command = command_source_order[command_order]
            command_velocity = sampled_velocity[command_order]
            if np.any(np.abs(command_velocity) > velocity_limits.astype(float)):
                runtime_stop_reason = "REFERENCE_JOINT_VELOCITY_EXCEEDS_OFFICIAL_LIMIT"
                break
            drive_position_target = _drive_target(command)
            articulation.set_dof_position_targets(drive_position_target[None, :])
            articulation.set_dof_velocity_targets(command_velocity.astype(np.float32)[None, :])
            contact_runtime_context.update(
                stage=resolve_actual_task_stage(metadata.get("stage_windows", []), trajectory_time,
                    grasp_commanded=grasp_commanded, grasp_event_time_s=grasp_event_time,
                    contact_wait_started_s=contact_wait_started_s,
                    attached=grasp_joint is not None,
                    release_commanded=release_commanded,
                    release_event_time_s=release_event_time,
                    support_wait_started_s=support_wait_started_s),
                attached=grasp_joint is not None,
                actual_free_space=bool(stack_monitor is not None and stack_monitor.free_space_reached),
                release_validation_pending=target_cup_release_gate.pending,
            )
            # Contact reports are delivered by the following physics step.
            # Preserve their command-clock timestamp in the same trajectory
            # basis as stage_windows; bounded physical waits can make the
            # simulation clock differ from the trajectory clock.
            contact_trajectory_clock_s[0] = trajectory_time
            if zero_point_contact_resolver.unresolved_outside(_contact_scope_token()):
                runtime_stop_reason = "UNRESOLVED_ZERO_POINT_ROBOT_CONTACT_HEADER"
                (args.output / "unresolved_robot_contact_headers.json").write_text(
                    json.dumps(zero_point_contact_resolver.snapshot(), indent=2), encoding="utf-8")
                break
            render = (step + 1) % args.render_every == 0 or step == physics_steps - 1
            world.step(render=False, update_fabric=True)
            simulation_time = (step + 1) * physics_dt
            if conveyor_enabled:
                _update_conveyor_visual_markers(physics_dt)
            release_clearance_failure = target_cup_release_gate.observe(
                simulation_time, active_contact_headers, target_path=target_carton_path,
                compliant_paths=compliant_cup_collider_paths)
            for event in target_cup_release_gate.events[target_cup_release_logged_events:]:
                event_log.append(event)
                print("FANUC_REPLAY_EVENT=" + json.dumps(event), flush=True)
            target_cup_release_logged_events = len(target_cup_release_gate.events)
            if stack_monitor is not None and grasp_enabled and not release_commanded:
                actual_stack_states = _capture_carton_states()
                actual_boxes = {_state_obb(item).name: _state_obb(item) for item in actual_stack_states}
                next_command = _sample(timestamps, positions, min(requested_duration, trajectory_time + physics_dt))
                stack_observation = stack_monitor.observe(
                    simulation_time, actual_boxes[str(metadata["target"])], list(actual_boxes.values()),
                    commanded_motion=bool(not hold_trajectory and np.linalg.norm(next_command - command_source_order) > 1e-8),
                )
                if render or not stack_observation["accepted"]:
                    stack_monitor_history.append({"time_s": simulation_time, **stack_observation})
                if not stack_observation["accepted"]:
                    runtime_stop_reason = stack_observation["reason"]
                    break
            if render and (args.record_replay or args.record_video):
                capture_q = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0].copy()
                capture_qd = np.asarray(articulation.get_dof_velocities().numpy(), dtype=float)[0].copy()
                tcp_target = telemetry_robot.fk(command)
                body_pose_position, body_pose_quaternion = grasp_body.get_world_poses()
                actual_world_from_body = np.eye(4)
                actual_world_from_body[:3, 3] = np.asarray(body_pose_position.numpy())[0]
                actual_world_from_body[:3, :3] = _rotation_matrix_from_quaternion_wxyz(np.asarray(body_pose_quaternion.numpy())[0])
                tcp_actual = actual_world_from_body @ telemetry_robot.tip_from_tcp
                tcp_translation_error = float(np.linalg.norm(tcp_actual[:3, 3] - tcp_target[:3, 3]))
                tcp_rotation_error = float(np.arccos(np.clip((np.trace(tcp_target[:3, :3].T @ tcp_actual[:3, :3]) - 1) / 2, -1, 1)))
                capture_cartons = _capture_carton_states()
                rep.orchestrator.step(rt_subframes=1, pause_timeline=False, delta_time=0.0, wait_for_render=True)
                after_capture_q = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0]
                capture_max_joint_delta_rad = max(capture_max_joint_delta_rad, float(np.max(np.abs(after_capture_q - capture_q))))
                after_capture_cartons = _capture_carton_states()
                capture_max_carton_delta_m = max(capture_max_carton_delta_m, max(
                    (float(np.linalg.norm(np.asarray(a["center_m"]) - b["center_m"]))
                     for a, b in zip(capture_cartons, after_capture_cartons, strict=True)), default=0.0))
                actual_frame_states.append({"time_s": simulation_time, "trajectory_time_s": trajectory_time,
                                            "stage": contact_runtime_context["stage"],
                                            "q_target_rad": command.tolist(), "q_rad": capture_q.tolist(),
                                            "qd_rad_s": capture_qd.tolist(),
                                            "qd_target_rad_s": command_velocity.tolist(),
                                            "payload_gravity_feedforward_nm": last_drive_feedforward["payload_gravity_nm"].tolist(),
                                            "drive_position_target_rad": drive_position_target.tolist(),
                                            "tcp_target_world": tcp_target.tolist(), "tcp_actual_world": tcp_actual.tolist(),
                                            "tcp_translation_error_m": tcp_translation_error, "tcp_rotation_error_rad": tcp_rotation_error,
                                            "cartons": capture_cartons,
                                            "attached": bool(grasp_enabled and not release_commanded)})
                rendered_rgba = np.asarray(rgb_annotator.get_data())
                if rendered_rgba.ndim == 3 and rendered_rgba.shape[-1] >= 3:
                    rendered_rgb = rendered_rgba[..., :3].astype(np.uint8).copy()
                    if args.record_replay:
                        replay_frames.append(Image.fromarray(rendered_rgb))
                    if replay_video_writer is not None:
                        phase = contact_runtime_context["stage"]
                        display_phase = "conveyor_transport" if conveyor_running else phase
                        actual_cups = int(
                            ideal_actual_contact_count_at_attach
                            if ideal_actual_contact_count_at_attach is not None
                            else sum(commanded_cup_mask)
                        )
                        if conveyor_running:
                            state_key = "belt_running"
                        elif conveyor_started:
                            state_key = "belt_stopped"
                        elif conveyor_start_due and release_open_confirmed:
                            state_key = "waiting_clearance"
                        elif release_open_confirmed:
                            state_key = "released"
                        elif grasp_joint is not None:
                            state_key = "attached"
                        else:
                            state_key = "approaching"
                        if hud_chinese_enabled:
                            phase_names = {
                                "home": "初始", "pregrasp": "预抓取", "contact": "接触",
                                "support-release": "解除支撑", "extraction": "脱垛",
                                "transit": "搬运", "place": "放置", "withdrawal": "撤离",
                                "conveyor_transport": "输送",
                            }
                            state_names = {
                                "belt_running": "皮带运行", "belt_stopped": "带内安全位停止",
                                "waiting_clearance": "等待工具退出输送冲突区",
                                "released": "已解除吸附", "attached": "箱体已吸附",
                                "approaching": "接近目标",
                            }
                            face_names = {"front": "正面", "side": "侧面", "top": "顶面"}
                            belt_names = {
                                "conveyor_transverse": "横向传送带",
                                "conveyor_longitudinal": "纵向传送带",
                            }
                            lines = [
                                f"首箱 {metadata.get('target')} · 行 {metadata.get('row_selection', {}).get('row_id', '派生')} · {face_names.get(str(gripper_cfg.get('target_face')), gripper_cfg.get('target_face'))}",
                                f"阶段 {phase_names.get(display_phase, display_phase)} · 仿真 {simulation_time:.2f} s",
                                f"吸盘 {actual_cups}/72 · {'已释放' if release_open_confirmed else '已吸附' if grasp_joint is not None else '待吸附'}",
                                f"接收 {belt_names.get(place_surface, place_surface)} · 实际带速 {conveyor_speed_m_s if conveyor_running else 0.0:.2f} m/s",
                                f"状态 {state_names[state_key]}",
                            ]
                            assumption = "物理：保留箱间接触；仅免除 J5/J6 与自有吸具内部碰撞"
                        else:
                            lines = [
                                f"First carton {metadata.get('target')} · row {metadata.get('row_selection', {}).get('row_id', 'derived')} · {gripper_cfg.get('target_face')}",
                                f"Phase {display_phase} · simulation {simulation_time:.2f} s",
                                f"Cups {actual_cups}/72 · {'released' if release_open_confirmed else 'attached' if grasp_joint is not None else 'open'}",
                                f"Receiver {place_surface} · actual belt {conveyor_speed_m_s if conveyor_running else 0.0:.2f} m/s",
                                f"State {state_key.replace('_', ' ')}",
                            ]
                            assumption = "Physics: stack contact kept; only J5/J6-owned-tool internal pairs exempt"
                        status_color = ((72, 232, 150, 255) if conveyor_running
                                        else (255, 205, 92, 255) if state_key == "waiting_clearance"
                                        else (210, 220, 232, 255))
                        rendered_rgb = _draw_runtime_hud(
                            rendered_rgb, lines, assumption, status_color
                        )
                        video_frame = cv2.cvtColor(rendered_rgb, cv2.COLOR_RGB2BGR)
                        if replay_video_frame_count == 0:
                            cv2.imwrite(str(args.output / "initial.png"), video_frame)
                        if display_phase not in saved_phase_keyframes:
                            cv2.imwrite(str(args.output / f"phase_{_safe_prim_name(display_phase)}.png"), video_frame)
                            saved_phase_keyframes.add(display_phase)
                        last_video_frame = video_frame.copy()
                        replay_video_writer.write(video_frame)
                        if preview_video_writer is not None:
                            preview_video_writer.write(video_frame)
                        replay_video_frame_count += 1
            if (
                ideal_independent_mode
                and ideal_release_request_step is not None
                and not release_open_confirmed
                and step > ideal_release_request_step
            ):
                body_positions, body_orientations = grasp_body.get_world_poses()
                payload_positions, payload_orientations = target_body.get_world_poses()
                body_position = np.asarray(body_positions.numpy(), dtype=float)[0]
                body_quaternion = np.asarray(body_orientations.numpy(), dtype=float)[0]
                payload_position = np.asarray(payload_positions.numpy(), dtype=float)[0]
                payload_quaternion = np.asarray(payload_orientations.numpy(), dtype=float)[0]
                body_rotation = _rotation_matrix_from_quaternion_wxyz(body_quaternion)
                current_relative_position = body_rotation.T @ (
                    payload_position - body_position
                )
                current_relative_quaternion = _quaternion_multiply_wxyz(
                    _quaternion_conjugate_wxyz(body_quaternion), payload_quaternion
                )
                released_relative_position_delta_m = float(
                    np.linalg.norm(
                        current_relative_position
                        - ideal_release_relative_position_at_request
                    )
                )
                released_relative_rotation_delta_rad = _quaternion_angle_wxyz(
                    _quaternion_multiply_wxyz(
                        _quaternion_conjugate_wxyz(
                            ideal_release_relative_quaternion_at_request
                        ),
                        current_relative_quaternion,
                    )
                )
                support_contact_still_active = _actual_support_contact_observed()
                release_open_confirmed = bool(
                    not stage.GetPrimAtPath(grasp_joint_path).IsValid()
                    and support_contact_still_active
                    and (
                        released_relative_position_delta_m > float(actual_state_gates.get("release_independence_translation_m", 0.002))
                        or released_relative_rotation_delta_rad > float(actual_state_gates.get("release_independence_rotation_rad", 0.01))
                    )
                )
                if release_open_confirmed:
                    release_executed = True
                    release_executed_time_s = simulation_time
                    release_velocity_sample_pending = True
                    event_log.append(
                        {
                            "event": "release_constraint_removal_confirmed",
                            "simulation_time_s": simulation_time,
                            "request_time_s": ideal_release_request_time_s,
                            "relative_position_delta_m": released_relative_position_delta_m,
                            "relative_rotation_delta_rad": released_relative_rotation_delta_rad,
                            "support_contact_still_active": support_contact_still_active,
                        }
                    )
                elif (
                    simulation_time - float(ideal_release_request_time_s)
                    > maximum_support_wait_s
                ):
                    raise RuntimeError(
                        "bounded release confirmation expired before payload/tool independence"
                    )
            if (
                args.gripper_model == "surface_gripper"
                and not ideal_independent_mode
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
                if not grasp_enabled:
                    hold_trajectory = True
                    if simulation_time - float(contact_wait_started_s) > maximum_contact_wait_s:
                        raise RuntimeError(
                            "bounded actual-contact wait expired before SurfaceGripper confirmed the target"
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
                and not ideal_independent_mode
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
                and not release_commanded
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
                if (
                    conveyor_running
                    and target_center_at_conveyor_start is not None
                    and conveyor_initial_direction_world is not None
                ):
                    actual_progress = float(
                        (payload_center - target_center_at_conveyor_start)
                        @ conveyor_initial_direction_world
                    )
                    surface = conveyor_primitives.get(active_conveyor_surface or place_surface)
                    safe_progress = conveyor_transport_target_distance_m
                    if surface is not None:
                        surface_center = np.asarray(surface["center_m"], dtype=float)
                        surface_size = np.asarray(surface["size_m"], dtype=float)
                        surface_rotation = np.asarray(surface["rotation_matrix"], dtype=float)
                        surface_half_along = float(
                            np.sum(np.abs(surface_rotation.T @ conveyor_initial_direction_world)
                                   * 0.5 * surface_size)
                        )
                        target_half_along = 0.5 * float(np.max(target_primitive["size_m"][:2]))
                        surface_outlet_projection = float(
                            surface_center @ conveyor_initial_direction_world + surface_half_along
                        )
                        safe_progress = min(
                            safe_progress,
                            surface_outlet_projection
                            - float(target_center_at_conveyor_start @ conveyor_initial_direction_world)
                            - target_half_along
                            - conveyor_safe_tail_margin_m,
                        )
                    if actual_progress >= max(conveyor_transport_minimum_distance_m, safe_progress):
                        conveyor_running = False
                        conveyor_stopped_time_s = simulation_time
                        conveyor_stop_reason = "SAFE_IN_BELT_WAIT_POSITION_REACHED"
                        _apply_conveyor_surface_selection((), simulation_time)
                        print(
                            f"FANUC_REPLAY_EVENT=conveyor_stopped time_s={simulation_time:.6f} "
                            f"distance_m={actual_progress:.6f} reason={conveyor_stop_reason}",
                            flush=True,
                        )
                elapsed_after_release = simulation_time - float(release_executed_time_s)
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
                    and conveyor_running
                    and conveyor_started_time_s is not None
                ):
                    # Start transport evidence at the actual interlocked belt
                    # start and stop sampling when the physical drive stops.
                    # An L-shaped conveyor can subsequently transfer the carton
                    # to a different surface with a different travel direction.
                    elapsed_after_belt_start = simulation_time - float(conveyor_started_time_s)
                    if elapsed_after_belt_start <= conveyor_transport_audit_window_s + 1e-9:
                        conveyor_transport_samples.append(
                            (
                                simulation_time,
                                payload_center.copy(),
                                payload_linear_velocity.copy(),
                                active_conveyor_surface,
                            )
                        )
            measured = np.asarray(articulation.get_dof_positions().numpy(), dtype=float)[0]
            measured_velocity = np.asarray(
                articulation.get_dof_velocities().numpy(), dtype=float
            )[0]
            # Isaac 6 experimental Articulation exposes get_dof_efforts as the
            # explicit effort-control input channel, not the realized output of an
            # implicit position drive.  Keep it as telemetry, but never pass zeros
            # from that channel as proof that the position drive respected effort.
            drive_effort_tensor = articulation.get_dof_efforts()
            drive_effort_source = (
                "isaac_articulation_commanded_effort_channel_"
                "not_position_drive_output"
            )
            drive_effort = np.asarray(drive_effort_tensor.numpy(), dtype=float)[0]
            # Projected joint forces are reactions transmitted along each revolute
            # DOF.  They are logged independently and are not compared with the
            # official actuator effort limit.
            projected_forces = np.asarray(
                articulation.get_dof_projected_joint_forces().numpy(), dtype=float
            )[0]
            gravity_forces = np.asarray(
                articulation.get_dof_gravity_compensation_forces().numpy(), dtype=float
            )[0]
            measured_acceleration = (
                None
                if previous_measured_velocity is None
                else (measured_velocity - previous_measured_velocity) / physics_dt
            )
            previous_measured_velocity = measured_velocity.copy()
            if measured_acceleration is None:
                inverse_dynamics_available_all_steps = False
                model_inverse_dynamics = np.full_like(measured, np.nan, dtype=float)
            else:
                try:
                    mass_matrix = np.asarray(
                        articulation.get_mass_matrices().numpy(), dtype=float
                    )[0]
                    coriolis = np.asarray(
                        articulation.get_dof_coriolis_and_centrifugal_compensation_forces().numpy(),
                        dtype=float,
                    )[0]
                    model_inverse_dynamics = (
                        mass_matrix @ measured_acceleration + coriolis + gravity_forces
                    )
                    if model_inverse_dynamics.shape != measured.shape or not np.all(
                        np.isfinite(model_inverse_dynamics)
                    ):
                        raise ValueError("invalid inverse-dynamics tensor result")
                except (AttributeError, RuntimeError, ValueError):
                    inverse_dynamics_available_all_steps = False
                    inverse_dynamics_available_after_first_difference = False
                    model_inverse_dynamics = np.full_like(measured, np.nan, dtype=float)
            # Projected constraint reaction and actuator generalized torque are
            # different quantities. Their subtraction is not an identified
            # external-load estimator, so leave this unavailable channel empty.
            external_joint_load = np.full_like(projected_forces, np.nan, dtype=float)
            measured_times.append(simulation_time)
            commanded_rows.append(command.astype(float))
            commanded_velocity_rows.append(command_velocity.copy())
            payload_gravity_feedforward_rows.append(last_drive_feedforward["payload_gravity_nm"].copy())
            measured_rows.append(measured)
            measured_velocity_rows.append(measured_velocity)
            drive_effort_rows.append(drive_effort)
            projected_force_rows.append(projected_forces)
            gravity_force_rows.append(gravity_forces)
            model_inverse_dynamics_rows.append(model_inverse_dynamics)
            external_joint_load_rows.append(external_joint_load)
            if unexpected_robot_contact_events:
                runtime_stop_reason = "UNEXPECTED_ROBOT_OR_RIGID_TOOL_PROXIMITY"
                event_log.append(unexpected_robot_contact_events[0])
                break
            if release_clearance_failure:
                runtime_stop_reason = release_clearance_failure
                break
            if not hold_trajectory:
                trajectory_time = (free_transit_gate.advance(trajectory_time, physics_dt, requested_duration)
                                   if free_transit_gate is not None else min(requested_duration, trajectory_time + physics_dt))
            if (
                trajectory_time >= requested_duration - 1e-12
                and (
                    release_event_time is None
                    or release_executed_time_s is not None
                    and simulation_time - release_executed_time_s
                    >= float(args.post_release_seconds)
                )
                and (
                    not conveyor_enabled
                    or conveyor_started_time_s is not None
                    and simulation_time - conveyor_started_time_s >= float(args.post_release_seconds)
                )
            ):
                break
        replay_wall_s = time.perf_counter() - replay_started_at
        if last_video_frame is not None:
            cv2.imwrite(str(args.output / "final.png"), last_video_frame)
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

        final_carton_states = _capture_carton_states()
        (args.output / "actual_frame_states.json").write_text(json.dumps({
            "format": "isaac_actual_frame_states_v2", "joint_names": discovered_joint_names,
            "states": actual_frame_states, "capture_max_joint_delta_rad": capture_max_joint_delta_rad,
            "capture_max_carton_delta_m": capture_max_carton_delta_m,
        }, indent=2), encoding="utf-8")
        (args.output / "cup_mask_changes.json").write_text(json.dumps(cup_mask_change_log, indent=2), encoding="utf-8")
        (args.output / "stack_contact_monitor.json").write_text(json.dumps({
            "summary": None if stack_monitor is None else stack_monitor.summary(),
            "observations": stack_monitor_history, "stop_reason": runtime_stop_reason,
        }, indent=2), encoding="utf-8")

        measured_array = np.asarray(measured_rows)
        commanded_array = np.asarray(commanded_rows)
        measured_velocity_array = np.asarray(measured_velocity_rows)
        drive_effort_array = np.asarray(drive_effort_rows)
        projected_force_array = np.asarray(projected_force_rows)
        gravity_force_array = np.asarray(gravity_force_rows)
        model_inverse_dynamics_array = np.asarray(model_inverse_dynamics_rows)
        external_joint_load_array = np.asarray(external_joint_load_rows)
        errors = measured_array - commanded_array
        rms_error = np.sqrt(np.mean(errors**2, axis=0))
        peak_error = np.max(np.abs(errors), axis=0)
        peak_projected_force = np.max(np.abs(projected_force_array), axis=0)
        peak_gravity_force = np.max(np.abs(gravity_force_array), axis=0)
        peak_drive_effort = np.max(np.abs(drive_effort_array), axis=0)
        peak_model_inverse_dynamics = (
            np.nanmax(np.abs(model_inverse_dynamics_array), axis=0)
            if np.any(np.isfinite(model_inverse_dynamics_array))
            else np.full(len(discovered_joint_names), np.nan)
        )
        peak_external_joint_load = (
            np.nanmax(np.abs(external_joint_load_array), axis=0)
            if np.any(np.isfinite(external_joint_load_array))
            else np.full(len(discovered_joint_names), np.nan)
        )
        effort_ratios = (
            peak_drive_effort / effort_limits
            if effort_limits.size and drive_effort_output_qualified
            else np.asarray([])
        )
        model_inverse_dynamics_effort_ratios = (
            peak_model_inverse_dynamics / effort_limits
            if effort_limits.size
            and np.all(np.isfinite(peak_model_inverse_dynamics))
            else np.asarray([])
        )
        lower_position_limits = np.asarray(
            metadata.get("joint_position_lower_limits_rad", []), dtype=float
        )
        upper_position_limits = np.asarray(
            metadata.get("joint_position_upper_limits_rad", []), dtype=float
        )
        joint_positions_within_limits = None
        minimum_joint_position_margin_rad = None
        if (
            lower_position_limits.shape == (len(discovered_joint_names),)
            and upper_position_limits.shape == (len(discovered_joint_names),)
        ):
            lower_position_limits = lower_position_limits[command_order]
            upper_position_limits = upper_position_limits[command_order]
            lower_margin = measured_array - lower_position_limits[None, :]
            upper_margin = upper_position_limits[None, :] - measured_array
            minimum_joint_position_margin_rad = np.min(
                np.minimum(lower_margin, upper_margin), axis=0
            )
            joint_positions_within_limits = bool(
                np.all(minimum_joint_position_margin_rad >= -1e-9)
            )

        if zero_point_contact_resolver.pending_keys and runtime_stop_reason is None:
            runtime_stop_reason = "UNRESOLVED_ZERO_POINT_ROBOT_CONTACT_HEADER"
        contact_records = [
            {"actor0": pair[0], "actor1": pair[1], **values}
            for pair, values in sorted(contact_pairs.items())
        ]
        target_name = str(metadata["target"])
        grasp_time = metadata.get("grasp_time_seconds")
        release_time = metadata.get("release_time_seconds")

        robot_contact_records = [
            record
            for record in contact_records
            if robot_proximity_is_safety_relevant(record, root_prim_path)
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
            if any(str(record["actor0"]) == path or str(record["actor0"]).startswith(path + "/")
                   or str(record["actor1"]) == path or str(record["actor1"]).startswith(path + "/")
                   for path in conveyor_paths)
        ]
        premature_payload_conveyor_contacts = None
        if release_time is not None:
            # Receiver contact is required *before* release during the PLACE dwell.
            # Only contact that starts before that declared placement phase is
            # premature; legal non-penetrating support may persist while attached.
            release_contact_tolerance_s = 2.0 * physics_dt
            place_support_window_start = placement_support_window_start(
                metadata, release_time
            )
            premature_payload_conveyor_contacts = premature_physical_conveyor_contacts(
                payload_conveyor_contact_records, place_start_s=place_support_window_start,
                time_tolerance_s=release_contact_tolerance_s,
            )
        unexpected_contacts = [
            record for record in robot_contact_records
            if record.get("unexpected_runtime_event_count", 0) > 0
        ]
        tracking_error_limit_rad = 0.05
        full_schedule_replayed = trajectory_time >= requested_duration - 1e-9
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
            (ideal_independent_mode or args.gripper_model == "surface_gripper")
            and release_open_confirmed
            and released_payload_path is None
        )
        release_completed = bool(
            release_open_confirmed
            if ideal_independent_mode or args.gripper_model == "surface_gripper"
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
            gripper_wrench_envelope_complete=bool(
                not ideal_independent_mode and gripper_wrench_envelope_complete
            ),
            gripper_limits_from_configuration=bool(
                not ideal_independent_mode
                and (
                    not args.disable_gripper_break_limits
                    and args.gripper_force_limit is None
                    and args.gripper_shear_force_limit is None
                    and args.gripper_torque_limit is None
                    and args.gripper_attachment_point_count is None
                    and not surface_shear_solver_fallback_used
                    and not fixed_joint_torque_solver_fallback_used
                )
            ),
            gripper_limits_calibrated=bool(
                not ideal_independent_mode
                and metadata.get("gripper", {}).get("limits_calibrated", False)
            ),
            production_release_adapter=production_release_adapter,
            ideal_holding_capacity_assumption=ideal_independent_mode,
            joint_positions_within_limits=joint_positions_within_limits,
            conveyor_transport_expected=bool(conveyor_enabled and place_surface),
            conveyor_transport_engaged=conveyor_transport_engaged,
            conveyor_transport_speed_within_tolerance=conveyor_transport_speed_within_tolerance,
        )
        qualification_checks = qualification["qualification_checks"]
        qualification_failures = qualification["qualification_failures"]
        qualification_passed = qualification["qualification_passed"]
        completed_carton_ids = list(metadata.get("completed_carton_ids", []))
        physical_cycle_completed = bool(full_schedule_replayed and release_open_confirmed
                                        and not target_cup_release_gate.pending
                                        and payload_motion_verified and not unexpected_contacts
                                        and runtime_stop_reason is None
                                        and support_contact_audit is not None and support_contact_audit.accepted
                                        and joint_positions_within_limits is True
                                        and float(np.max(peak_error)) <= tracking_error_limit_rad
                                        and np.all(np.max(np.abs(measured_velocity_array), axis=0) <= velocity_limits + 1e-5))
        if physical_cycle_completed and str(metadata["target"]) not in completed_carton_ids:
            completed_carton_ids.append(str(metadata["target"]))
        (args.output / "actual_remaining_state.json").write_text(json.dumps({
            "schema": "m710id70_actual_motion_state_v1",
            "q_rad": np.asarray(articulation.get_dof_positions().numpy())[0].tolist(),
            "joint_names": discovered_joint_names,
            "world_session_id": str(run_started_unix_s), "time_s": session_time_offset_s + simulation_time,
            "attached": bool(grasp_enabled and not release_open_confirmed),
            "attachment_target": str(metadata["target"]) if grasp_enabled and not release_open_confirmed else None,
            "cartons": [{"name": item["name"], "position_m": item["center_m"],
                         "orientation_wxyz": item["quaternion_wxyz"],
                         "linear_velocity_m_s": item["linear_velocity_m_s"],
                         "angular_velocity_rad_s": item["angular_velocity_rad_s"]} for item in final_carton_states],
            "completed_carton_ids": completed_carton_ids,
            "handed_off_ids": list(metadata.get("handed_off_ids", [])),
        }, indent=2), encoding="utf-8")

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
                + [f"measured_velocity_{name}_rad_s" for name in discovered_joint_names]
                + [f"drive_input_effort_{name}_nm" for name in discovered_joint_names]
                + [f"model_inverse_dynamics_{name}_nm" for name in discovered_joint_names]
                + [
                    f"projected_constraint_reaction_{name}_nm"
                    for name in discovered_joint_names
                ]
                + [f"external_joint_load_residual_{name}_nm" for name in discovered_joint_names]
                + [f"gravity_compensation_{name}_nm" for name in discovered_joint_names]
                + [f"command_velocity_{name}_rad_s" for name in discovered_joint_names]
                + [f"attached_payload_gravity_feedforward_{name}_nm" for name in discovered_joint_names]
            )
            for (
                timestamp,
                command,
                measured,
                measured_velocity,
                drive_effort,
                model_inverse_dynamics,
                projected_force,
                external_joint_load,
                gravity_force,
                command_velocity,
                payload_gravity_feedforward,
            ) in zip(
                measured_times,
                commanded_array,
                measured_array,
                measured_velocity_array,
                drive_effort_array,
                model_inverse_dynamics_array,
                projected_force_array,
                external_joint_load_array,
                gravity_force_array,
                commanded_velocity_rows,
                payload_gravity_feedforward_rows,
                strict=True,
            ):
                writer.writerow(
                    [
                        timestamp,
                        *command.tolist(),
                        *measured.tolist(),
                        *measured_velocity.tolist(),
                        *drive_effort.tolist(),
                        *model_inverse_dynamics.tolist(),
                        *projected_force.tolist(),
                        *external_joint_load.tolist(),
                        *gravity_force.tolist(),
                        *command_velocity.tolist(),
                        *payload_gravity_feedforward.tolist(),
                    ]
                )

        carton_state_path = args.output / "carton_states.json"
        carton_state_path.write_text(
            json.dumps(
                {
                    "format": "isaacsim_dynamic_carton_states_v1",
                    "dynamic_carton_count": len(dynamic_scene_bodies),
                    "target": str(metadata["target"]),
                    "settled_before_replay": settled_carton_states,
                    "final_after_replay": final_carton_states,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        event_log_path = args.output / "execution_events.json"
        event_log_path.write_text(
            json.dumps(
                {
                    "format": "isaacsim_fanuc_execution_events_v1",
                    "events": event_log,
                },
                indent=2,
            ),
            encoding="utf-8",
        )

        actual_replayed_simulation_seconds = (
            float(measured_times[-1]) if measured_times else 0.0
        )

        def _finite_or_none(values) -> list[float | None]:
            return [float(value) if np.isfinite(value) else None for value in values]

        result = {
            "format": "isaacsim_fanuc_replay_result_v2",
            "robot_model": metadata["robot_model"],
            "target": metadata["target"],
            "simulation_execution_ready": metadata.get("simulation_execution_ready"),
            "physics_execution_backend": runtime_backend_evidence,
            "collision_offset_readback": runtime_collision_offset_evidence,
            "simulation_execution_qualified": metadata.get(
                "simulation_execution_qualified"
            ),
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
            "replayed_simulation_seconds": actual_replayed_simulation_seconds,
            "nominal_replay_duration_seconds": replay_duration,
            "maximum_physics_runtime_seconds": physical_runtime_limit,
            "replay_wall_seconds": replay_wall_s,
            "simulation_realtime_factor": actual_replayed_simulation_seconds / replay_wall_s,
            "physics_steps": len(measured_times),
            "maximum_physics_steps": physics_steps,
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
            "joint_load_metric": "explicit_multi_channel_joint_dynamics_telemetry",
            "joint_telemetry_csv": str(args.output / "joint_tracking.csv"),
            "joint_telemetry_semantics": {
                "q_target": "position-drive target sampled from the frozen trajectory clock",
                "q": "measured articulation joint position",
                "qd": "measured articulation joint velocity",
                "model_inverse_dynamics": "M(q)*finite_difference(qd)+c(q,qd)+g(q)",
                "model_inverse_dynamics_includes_external_carton": False,
                "drive_input_effort": (
                    "Isaac explicit effort-control input channel; not the realized "
                    "position-drive output torque"
                ),
                "projected_constraint_reaction": "Isaac PhysX projected DOF force",
                "external_joint_load_residual": (
                    "UNAVAILABLE: projected reaction is not calibrated actuator torque"
                ),
            },
            "model_inverse_dynamics_available_all_steps": (
                inverse_dynamics_available_all_steps
            ),
            "model_inverse_dynamics_available_after_first_difference": (
                inverse_dynamics_available_after_first_difference and len(measured_rows) > 1
            ),
            "drive_input_effort_source": drive_effort_source,
            "initial_state_method": "USD_JOINT_STATE_BEFORE_ZERO_DELTA_RESET",
            "initial_max_joint_error_rad": reset_q_error_rad,
            "render_capture_method": "FABRIC_SYNC_ZERO_DELTA_REPLICATOR",
            "render_capture_max_joint_delta_rad": capture_max_joint_delta_rad,
            "render_capture_max_carton_delta_m": capture_max_carton_delta_m,
            "wrist_tool_collision_exemptions": wrist_tool_exemption_records,
            "collision_policy": metadata.get("collision_policy"),
            "first_unexpected_runtime_robot_contact": unexpected_robot_contact_events[0] if unexpected_robot_contact_events else None,
            "zero_point_contact_resolution": zero_point_contact_resolver.snapshot(),
            "cup_mask_change_log": cup_mask_change_log,
            "runtime_stop_reason": runtime_stop_reason,
            "actual_stack_contact_monitor": None if stack_monitor is None else stack_monitor.summary(),
            "actual_free_transit_gate": None if free_transit_gate is None else {
                "maximum_wait_s": maximum_free_transit_wait_s,
                "trajectory_boundary_s": free_transit_gate.boundary_time_s,
                "passed": free_transit_gate.passed, "events": free_transit_gate.events,
                "clearance_policy_changed": False},
            "target_cup_release_clearance_gate": {
                "policy_source": "actual_state_gates.maximum_release_clearance_wait_s; independent bounded timer",
                "maximum_wait_s": target_cup_release_gate.maximum_wait_s,
                "pending": target_cup_release_gate.pending, "events": target_cup_release_gate.events,
                "constraint_independence_is_separate": True,
                "normal_rule_resumes_after_all_known_target_cup_proximity_lost": True},
            "physical_cycle_completed": physical_cycle_completed,
            "peak_actual_tcp_translation_error_m": max((item["tcp_translation_error_m"] for item in actual_frame_states), default=None),
            "peak_actual_tcp_rotation_error_rad": max((item["tcp_rotation_error_rad"] for item in actual_frame_states), default=None),
            "cup_collision_representation": "ALL_CUPS_COMPRESSED_BELLOWS_ENVELOPES_PLUS_VERIFIED_RIGID_INSERTS",
            "gravity_feedforward": {"enabled": gravity_feedforward_enabled,
                                    "method": "g_over_Kp_position_bias_inside_official_finite_drive_force_limit",
                                    "model_includes_tool": True, "model_includes_external_carton": False,
                                    "external_attached_payload_feedforward_enabled": payload_gravity_feedforward_enabled,
                                    "external_method": "J_world_transpose_compensating_gravity_wrench_at_actual_COM_inside_same_finite_drive",
                                    "external_term_is_measured_actuator_effort": False},
            "velocity_feedforward": {"enabled": velocity_feedforward_enabled,
                                      "method": "validated_position_path_piecewise_linear_derivative_right_segment_at_internal_knots",
                                      "endpoint_and_held_velocity_rad_s": 0.0},
            "drive_effort_output_qualified": drive_effort_output_qualified,
            "joint_positions_within_official_limits": joint_positions_within_limits,
            "minimum_joint_position_margin_rad": (
                None
                if minimum_joint_position_margin_rad is None
                else minimum_joint_position_margin_rad.tolist()
            ),
            "peak_measured_joint_velocity_rad_s": np.max(
                np.abs(measured_velocity_array), axis=0
            ).tolist(),
            "peak_drive_input_effort_nm": peak_drive_effort.tolist(),
            "peak_model_inverse_dynamics_nm": _finite_or_none(
                peak_model_inverse_dynamics
            ),
            "peak_external_joint_load_residual_nm": _finite_or_none(
                peak_external_joint_load
            ),
            "peak_projected_joint_force_nm": peak_projected_force.tolist(),
            "peak_gravity_compensation_nm": peak_gravity_force.tolist(),
            "effort_limit_ratio": (
                effort_ratios.tolist() if effort_limits.size else []
            ),
            "model_inverse_dynamics_effort_limit_ratio": (
                model_inverse_dynamics_effort_ratios.tolist()
                if model_inverse_dynamics_effort_ratios.size
                else []
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
            "conveyor_stopped_time_s": conveyor_stopped_time_s,
            "conveyor_stop_reason": conveyor_stop_reason,
            "conveyor_running_at_end": conveyor_running,
            "conveyor_start_interlock_history": conveyor_start_interlock_history,
            "target_center_at_conveyor_start_m": (
                None if target_center_at_conveyor_start is None
                else target_center_at_conveyor_start.tolist()
            ),
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
            "conveyor_visual_motion": {
                "model": "industrial_belt_surface_and_roller_phase_v2",
                "markers_have_collision": False,
                "marker_counts": {
                    name: len(records)
                    for name, records in conveyor_visual_markers.items()
                },
                "markers_follow_active_physx_surface_velocity": True,
                "roller_counts": {
                    name: len(records)
                    for name, records in conveyor_visual_rollers.items()
                },
                "rollers_follow_active_physx_surface_velocity": True,
                "independent_phase_accumulators_m": dict(conveyor_visual_phase_m),
                "stopped_surface_phase_is_frozen": True,
            },
            "hud": {
                "model": "measured_state_dark_panel_v1",
                "font_path": hud_font_path,
                "chinese_enabled": hud_chinese_enabled,
                "body_pixels_at_1080p": 28,
                "line_count": 5,
            },
            "target_carton_dynamic": target_carton_path is not None,
            "payload_constraint_commanded": grasp_commanded,
            "payload_grasp_command_succeeded": grasp_command_succeeded,
            "payload_grasp_closed_time_s": grasp_closed_time,
            "payload_grip_lost_time_s": surface_grip_lost_time,
            "payload_minimum_active_cup_count": minimum_active_gripper_count,
            "payload_actual_contact_cup_count_at_attach": (
                ideal_actual_contact_count_at_attach
            ),
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
                "usd_fixed_joint_removed_after_actual_receiver_support_same_rigid_body"
                if ideal_independent_mode
                else "isaac_surface_gripper_open_same_rigid_body"
                if args.gripper_model == "surface_gripper"
                else "physx_free_body_state_handoff_diagnostic"
            ),
            "payload_release_actual_support_audit": (
                None if support_contact_audit is None else support_contact_audit.to_dict()
            ),
            "payload_release_support_contact_report_observed": (
                support_contact_report_observed
            ),
            "release_requires_actual_receiver_support": True,
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
            "gripper_independent_cup_actual_contact_audit": (
                None
                if independent_contact_audit is None
                else independent_contact_audit.to_dict()
            ),
            "gripper_suction_mode": gripper_cfg.get("suction_mode"),
            "gripper_holding_capacity_assumption": gripper_cfg.get(
                "holding_capacity_assumption"
            ),
            "gripper_mask_bit_order_cup_ids": cup_bit_order,
            "gripper_geometrically_eligible_mask": eligible_cup_mask,
            "gripper_commanded_active_mask": commanded_cup_mask,
            "gripper_planned_fk_contact_mask": planned_fk_contact_mask,
            "gripper_actual_contact_mask": actual_contact_mask,
            "gripper_actual_contact_mask_source": (
                "isaac_actual_grasp_body_and_target_state"
                if independent_contact_audit is not None
                else None
            ),
            "gripper_actual_contact_gates_attachment": ideal_independent_mode,
            "gripper_attachment_uses_original_target_body": ideal_independent_mode,
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
            "gripper_simulation_constraint_count": (
                int(grasp_command_succeeded) if ideal_independent_mode else len(surface_attachment_paths)
            ),
            "gripper_simulation_attachment_point_count": (
                None if ideal_independent_mode else len(surface_attachment_paths)
            ),
            "gripper_attachment_point_count": (
                None if ideal_independent_mode else len(surface_attachment_paths)
            ),
            "gripper_solver_attachment_model": gripper_cfg.get("solver_attachment_model"),
            "physx_solver_position_iterations": solver_position_iterations,
            "physx_solver_velocity_iterations": solver_velocity_iterations,
            "gripper_wrench_envelope_complete": gripper_wrench_envelope_complete,
            "gripper_capacity_qualification": (
                "NOT_APPLICABLE_IDEAL_HOLDING_CAPACITY_ASSUMPTION"
                if ideal_independent_mode
                else "PHYSICAL_LIMITS_EVALUATED"
            ),
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
            "contact_report_semantics": {
                "legacy_contact_pairs": "all reported proximity headers; robot margin safety remains on this layer",
                "physical_contact": "finite separation <= bound existing support_max_gap_m; impulse is not an acceptance bypass",
                "physical_contact_tolerance_m": physical_contact_ledger.contact_tolerance_m,
                "tolerance_source": "metadata.actual_state_gates.support_max_gap_m",
                "support_and_premature_conveyor_layer": "physical_contact",
            },
            "robot_scene_proximity_pairs": robot_contact_records,
            "payload_physical_contact_pairs": [record for record in payload_contact_records
                                               if record.get("physical_contact_observed", False)],
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
            "dynamic_carton_state_log": str(carton_state_path),
            "dynamic_carton_state_count": len(final_carton_states),
            "execution_event_log": str(event_log_path),
            "execution_event_count": len(event_log),
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
                    "carton_states_json": _sha256_path(carton_state_path),
                    "execution_events_json": _sha256_path(event_log_path),
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

        session_time_offset_s += simulation_time
        session_segment_index += 1
        if (args.continuation_dir is None or session_segment_index >= args.maximum_segments
                or not physical_cycle_completed):
            break
        request_path = args.continuation_dir / f"segment_{session_segment_index + 1:03d}_request.json"
        ready_path = args.continuation_dir / f"segment_{session_segment_index + 1:03d}_ready.json"
        actual_state_path = args.output / "actual_remaining_state.json"
        actual_state = json.loads(actual_state_path.read_text(encoding="utf-8"))
        actual_state_sha256 = _sha256_path(actual_state_path)
        ready_path.write_text(json.dumps({
            "status": "WORLD_RETAINED_AWAITING_OFFLINE_NEXT_PLAN",
            "world_session_id": str(run_started_unix_s),
            "completed_segments": session_segment_index,
            "actual_state_path": str(actual_state_path),
            "actual_state_sha256": actual_state_sha256,
            "request_path": str(request_path),
            "physics_time_paused_for_offline_planning": True,
            "no_reset_no_body_replacement": True,
        }, indent=2), encoding="utf-8")
        print("FANUC_REPLAY_STAGE=awaiting_same_world_continuation " + str(ready_path), flush=True)
        wait_started = time.monotonic()
        while not request_path.is_file() and time.monotonic() - wait_started < args.continuation_wait_seconds:
            time.sleep(0.2)
        if not request_path.is_file():
            ready_path.write_text(json.dumps({"status": "CONTINUATION_WAIT_BUDGET_ENDED",
                                             "world_session_id": str(run_started_unix_s),
                                             "completed_segments": session_segment_index}, indent=2), encoding="utf-8")
            break
        request = json.loads(request_path.read_text(encoding="utf-8"))
        if _sha256_path(actual_state_path) != actual_state_sha256:
            raise ValueError("saved actual state changed while the physical World was paused")
        validate_continuation_request(request, world_session_id=str(run_started_unix_s),
                                      actual_state_sha256=actual_state_sha256)
        if request.get("stop"):
            ready_path.write_text(json.dumps({"status": str(request.get("reason", "OFFLINE_TASK_BUDGET_ENDED")),
                                             "completed_segments": session_segment_index}, indent=2), encoding="utf-8")
            break
        next_bundle_path = Path(request["bundle_path"]).resolve()
        if not next_bundle_path.is_relative_to(args.project_root.resolve()):
            raise ValueError("continuation bundle must remain inside the selected project")
        next_bundle = json.loads(next_bundle_path.read_text(encoding="utf-8"))
        from unloading_sim.m710_replay_physics import validate_same_world_continuation
        continuation_identity = validate_same_world_continuation(metadata, next_bundle, actual_state)
        next_integrity = contract_module.verify_m710_replay_bundle(
            next_bundle, project_root=args.project_root.resolve(),
            current_asset_audit=audit_m710_replay_assets(args.project_root.resolve(), next_bundle["metadata"]))
        metadata = next_bundle["metadata"]
        bundle = next_bundle
        args.bundle = next_bundle_path
        pre_simulation_integrity_gate = next_integrity
        timestamps, positions = replay_command_arrays(bundle, expected_joint_names)
        scene_primitives = list(metadata["scene_primitives"])
        target_name = str(metadata["target"])
        target_index = next(index for index, item in enumerate(dynamic_scene_records) if item["name"] == target_name)
        target_carton_path = dynamic_scene_prim_paths[target_index]
        target_body = dynamic_scene_bodies[target_index]
        target_primitive = next(item for item in scene_primitives if item["name"] == target_name)
        gripper_cfg = metadata["gripper"]
        cup_bit_order = list(gripper_cfg["mask_bit_order_cup_ids"])
        eligible_cup_mask = list(gripper_cfg["geometrically_eligible_mask"])
        commanded_cup_mask = list(gripper_cfg["commanded_active_mask"])
        planned_fk_contact_mask = list(gripper_cfg["planned_fk_contact_mask"])
        actual_contact_mask = [False] * physical_cup_count
        active_cup_indices = [index for index, active in enumerate(commanded_cup_mask) if active]
        physical_contact_offsets = physical_cup_centers[np.asarray(active_cup_indices, dtype=int)]
        for enabled_attr in conveyor_surface_enabled_attrs.values():
            enabled_attr.Set(False)
        for cup_index, active in enumerate(commanded_cup_mask):
            cup_prim = stage.GetPrimAtPath(f"{grasp_body_path}/FG42CupVisual_{cup_index:02d}")
            UsdShade.MaterialBindingAPI.Apply(cup_prim).Bind(active_rubber_material if active else rubber_material)
        contact_pairs = {}
        unexpected_robot_contact_events = []
        zero_point_contact_resolver = ZeroPointContactResolver()
        args.output = session_output_root / f"segment_{session_segment_index + 1:03d}"
        args.output.mkdir(parents=True, exist_ok=False)
        run_status_path = args.output / "run_status.json"
        run_status_path.write_text(json.dumps({"status": "same_world_segment_started",
                                             "continuation_identity": continuation_identity}, indent=2), encoding="utf-8")
except BaseException as exc:
    # SimulationApp.close() may terminate Kit before Python reports an uncaught
    # exception, so emit the traceback explicitly for unattended server runs.
    run_status_path.write_text(
        json.dumps(
            {
                "status": "diagnostic_complete" if isinstance(exc, DiagnosticSettlingComplete) else "failed",
                "started_unix_s": run_started_unix_s,
                ("completed_unix_s" if isinstance(exc, DiagnosticSettlingComplete) else "failed_unix_s"): time.time(),
                "exception_type": type(exc).__name__,
                "exception": str(exc),
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    # Preserve the physical failure, masks and recorded frames so the next
    # candidate can be chosen from evidence rather than an opaque timeout.
    try:
        if locals().get("last_video_frame") is not None:
            cv2.imwrite(str(args.output / "failure.png"), last_video_frame)
        if "actual_frame_states" in locals():
            (args.output / "actual_frame_states.json").write_text(
                json.dumps({"format": "isaac_actual_frame_states_v2", "states": actual_frame_states,
                            "stop_reason": str(exc)}, indent=2), encoding="utf-8")
        if "event_log" in locals():
            (args.output / "execution_events.json").write_text(json.dumps({"events": event_log}, indent=2), encoding="utf-8")
        if "cup_mask_change_log" in locals():
            (args.output / "cup_mask_changes.json").write_text(json.dumps(cup_mask_change_log, indent=2), encoding="utf-8")
        if "_capture_carton_states" in locals():
            (args.output / "failed_actual_carton_states.json").write_text(
                json.dumps(_capture_carton_states(), indent=2), encoding="utf-8")
    except BaseException as checkpoint_error:
        print(f"FANUC_REPLAY_FAILURE_CHECKPOINT_ERROR={checkpoint_error}", flush=True)
    if not isinstance(exc, DiagnosticSettlingComplete):
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
