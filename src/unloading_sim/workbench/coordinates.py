"""Read-only frame composition using the simulator's existing URDF FK."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import numpy as np

from unloading_sim.robot import URDFRobot
from .contracts import finite_vector, Pairing


def rigid(matrix):
    t = np.asarray(matrix, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-7, rtol=0)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-7, rtol=0)):
        raise ValueError("INVALID_RIGID_TRANSFORM: no orthogonalization or rotation repair is allowed")
    return t.copy()


@dataclass(frozen=True)
class Extrinsics:
    parent_frame: str
    child_frame: str
    matrix: np.ndarray
    status: str
    usage: str
    sha256: str
    raw: dict
    session_status: str

    @classmethod
    def load(cls, path):
        path = Path(path)
        data = path.read_bytes()
        raw = json.loads(data.decode("utf-8-sig"))
        matrix = rigid(raw["matrix"])
        if raw.get("unit") != "m":
            raise ValueError(f"EXTRINSICS_UNIT_M_REQUIRED: unit must explicitly be 'm'; got {raw.get('unit', '<missing>')!r}. No unit inference or conversion is allowed")
        if raw.get("parent_frame") not in {"link_1", "base_link"} or raw.get("child_frame") != "camera_optical":
            raise ValueError("Expected camera_optical relative to link_1 or base_link")
        if raw["parent_frame"] == "base_link" and raw.get("camera_mount") != "external_fixed":
            raise ValueError("BASE_CAMERA_REQUIRES_EXTERNAL_FIXED_MOUNT")
        if raw.get("model_sha256") not in {
            "f842fc3e6973e37d622553e8cc614e8eb2e133a955e8648541ac4ea854ae6faf",
            "08b33a0588839f9fef4f08e13bb04ff27071c332f5ac40e688949a1ea5e9970b",
        }:
            raise ValueError("Calibration model identity has no reviewed ECO65-B FK mapping")
        # Preserve source status verbatim; fitting error is never validation evidence.
        if not raw.get("status") or not raw.get("usage"):
            raise ValueError("Calibration status and usage are required")
        session = raw.get("session_file", raw.get("source_session_path"))
        session_path = (path.parent / session) if isinstance(session, str) else None
        session_status = "PRESENT_NOT_REVALIDATED" if session_path and session_path.is_file() else "MISSING"
        matrix.setflags(write=False)
        return cls(raw["parent_frame"], raw["child_frame"], matrix, raw["status"],
                   raw["usage"], hashlib.sha256(data).hexdigest(), raw, session_status)

    @property
    def label(self):
        return f"待独立验证 · {self.status} · {self.usage} · 原始会话 {self.session_status}"


@dataclass(frozen=True)
class JointMapping:
    mapping_id: str
    source_names: tuple[str, ...]
    target_names: tuple[str, ...]
    source_indices: tuple[int, ...]
    signs: tuple[int, ...]
    zero_offsets_rad: tuple[float, ...]
    source_unit: str
    evidence: str

    def __post_init__(self):
        if (len(set(self.source_names)) != 6 or len(self.source_names) != 6
                or self.target_names != tuple(f"joint_{i}" for i in range(1, 7))
                or sorted(self.source_indices) != list(range(6))
                or len(self.signs) != 6 or any(s not in (-1, 1) for s in self.signs)
                or self.source_unit not in ("degree", "radian")):
            raise ValueError("Invalid explicit joint mapping")
        finite_vector(self.zero_offsets_rad, 6, "joint zero offsets")
        if not self.evidence:
            raise ValueError("Joint mapping requires recorded comparison evidence")

    def apply(self, values, names):
        if tuple(names) != self.source_names:
            raise ValueError("Source joint order differs from audited mapping")
        q = np.asarray(finite_vector(values, 6, "source joints"))
        if self.source_unit == "degree":
            q = np.deg2rad(q)
        return q[list(self.source_indices)] * self.signs + self.zero_offsets_rad


@dataclass(frozen=True)
class ToolFrame:
    name: str
    parent_frame: str
    matrix: np.ndarray
    source: str
    status: str

    @classmethod
    def from_dict(cls, raw):
        if raw["parent_frame"] != "link_6":
            raise ValueError("Tool must explicitly use link_6")
        return cls(raw["name"], raw["parent_frame"], rigid(raw["matrix"]),
                   raw["source"], raw["status"])


@dataclass(frozen=True)
class Installation:
    """Explicit physical mounting choices, independent from historical scenes."""
    world_from_base: np.ndarray
    camera_parent_frame: str
    tool_name: str
    tool: ToolFrame | None

    @classmethod
    def load(cls, path):
        raw = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if raw.get("schema") != "eco65_workbench_installation_v1":
            raise ValueError("INVALID_INSTALLATION_SCHEMA")
        world, camera, tool = raw["world"], raw["camera"], raw["tool"]
        if (world.get("parent_frame") != "world" or world.get("child_frame") != "base_link"
                or world.get("unit") != "m" or world.get("definition") != "world_equals_robot_base"):
            raise ValueError("INSTALLATION_REQUIRES_EXPLICIT_WORLD_EQUALS_BASE")
        matrix = rigid(world["matrix"])
        if not np.allclose(matrix, np.eye(4), atol=1e-12, rtol=0):
            raise ValueError("WORLD_EQUALS_BASE_REQUIRES_IDENTITY")
        camera_parents = {"external_fixed": "base_link", "link_1": "link_1"}
        if (camera.get("mount") not in camera_parents
                or camera.get("parent_frame") != camera_parents[camera["mount"]]
                or camera.get("child_frame") != "camera_optical"):
            raise ValueError("INSTALLATION_CAMERA_MOUNT_PARENT_MISMATCH")
        if (tool.get("parent_frame") != "link_6" or tool.get("unit") != "m"
                or not tool.get("name") or not tool.get("source") or not tool.get("status")):
            raise ValueError("INVALID_INSTALLATION_TOOL")
        if tool.get("matrix") is None and tool["status"] != "AWAITING_TCP_MEASUREMENT":
            raise ValueError("MISSING_TOOL_MATRIX_REQUIRES_PENDING_STATUS")
        return cls(matrix, camera["parent_frame"], tool["name"],
                   ToolFrame.from_dict(tool) if tool.get("matrix") is not None else None)

    def check_camera(self, extrinsics):
        if extrinsics.parent_frame != self.camera_parent_frame:
            raise ValueError(f"CAMERA_MOUNT_MISMATCH: installation requires {self.camera_parent_frame}, got {extrinsics.parent_frame}")


class CoordinateModel:
    def __init__(self, urdf_path, expected_sha256, world_from_base, tool):
        data = Path(urdf_path).read_bytes()
        if hashlib.sha256(data).hexdigest() != expected_sha256:
            raise ValueError("Pinned ECO65-B URDF hash mismatch")
        self.robot = URDFRobot.from_urdf(
            urdf_path, active_joint_names=[f"joint_{i}" for i in range(1, 7)],
            base_link="base_link", tip_link="link_6", tool_length=0.)
        self.world_from_base = rigid(world_from_base)
        self.tool = tool

    def frames(self, q):
        q = np.asarray(finite_vector(q, 6, "q_rad"))
        if not self.robot.within_limits(q):
            raise ValueError("Measured or replay joints exceed official URDF limits")
        return self.robot.named_link_frames(q)

    def camera(self, q_capture, extrinsics):
        if extrinsics.parent_frame == "base_link":
            base_from_camera = extrinsics.matrix
        elif extrinsics.parent_frame == "link_1":
            base_from_camera = self.frames(q_capture)["link_1"] @ extrinsics.matrix
        else:
            raise ValueError("UNSUPPORTED_CAMERA_PARENT_FRAME")
        return {"T_base_camera": base_from_camera,
                "T_world_camera": self.world_from_base @ base_from_camera}

    def paired_camera(self, pairing: Pairing, extrinsics):
        if pairing.q_capture_rad is None:
            raise ValueError("No capture joint association: " + pairing.status)
        return self.camera(pairing.q_capture_rad, extrinsics)

    def tcp(self, q):
        if self.tool is None:
            raise ValueError("TOOL_TCP_NOT_CALIBRATED: no measured tool transform loaded")
        return self.world_from_base @ self.frames(q)["link_6"] @ self.tool.matrix

    def installation_difference(self, q, extrinsics, nominal_link1_camera, nominal_flange_tcp):
        """Report explicit deltas; do not modify source calibration or add legacy yaw."""
        if extrinsics.parent_frame != "link_1":
            raise ValueError("CANNOT_COMPARE_DIFFERENT_CAMERA_MOUNTING_FRAMES")
        if self.tool is None:
            raise ValueError("TOOL_TCP_NOT_CALIBRATED")
        nominal_camera = rigid(nominal_link1_camera)
        nominal_tcp = rigid(nominal_flange_tcp)
        return {"T_nominal_camera_calibrated_camera": (np.linalg.inv(nominal_camera) @ extrinsics.matrix).tolist(),
                "T_nominal_tcp_configured_tcp": (np.linalg.inv(nominal_tcp) @ self.tool.matrix).tolist(),
                "T_world_camera": self.camera(q, extrinsics)["T_world_camera"].tolist(),
                "calibration_modified": False, "implicit_yaw_compensation": False}
