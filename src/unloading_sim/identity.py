"""Canonical robot, tool, and cache identities.

This module is deliberately independent from planners and simulator backends.
It provides the fail-closed identity checks needed when a trajectory or report
is reused with a different robot or end effector.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


_ROBOT_ALIASES = {
    "ur5e_like": "ur5e_like",
    "kuka_kr50_r2500": "kuka_kr50_r2500",
    "fanuc_m20id_35": "fanuc_m20id_35",
    "fanuc_m710id_70": "fanuc_m710id_70",
    # v0.3 and earlier spelling. Accept on input, never emit it.
    "fanuc_m20id35": "fanuc_m20id_35",
}

CACHE_IDENTITY_FIELDS = (
    "robot_model_id",
    "robot_kinematic_hash",
    "tool_name",
    "tool_mass_kg",
    "tool_config_hash",
    "tcp_transform",
)


def normalize_robot_model_id(value: object) -> str:
    """Return the canonical model ID and reject unknown identifiers."""
    model_id = str(value).strip() if value is not None else ""
    try:
        return _ROBOT_ALIASES[model_id]
    except KeyError as exc:
        known = ", ".join(sorted(set(_ROBOT_ALIASES.values())))
        raise ValueError(f"unknown robot model ID {model_id!r}; expected one of: {known}") from exc


def sha256_file(path: str | Path) -> str:
    resolved = Path(path).resolve()
    return hashlib.sha256(resolved.read_bytes()).hexdigest()


def _canonical_hash(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _read_yaml(path: str | Path) -> tuple[dict[str, Any], Path]:
    resolved = Path(path).resolve()
    data = yaml.safe_load(resolved.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{resolved} must contain a YAML mapping")
    return data, resolved


def _vector3(value: Sequence[float], name: str) -> tuple[float, float, float]:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite SI values")
    return tuple(float(v) for v in result)


def _matrix3(value: Sequence[Sequence[float]], name: str) -> tuple[tuple[float, ...], ...]:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    return tuple(tuple(float(v) for v in row) for row in result)


def _inertia3(value: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError("tool inertia must be a finite 3x3 tensor in kg*m^2")
    if not np.allclose(result, result.T, atol=1e-10):
        raise ValueError("tool inertia must be symmetric")
    if np.min(np.linalg.eigvalsh(result)) < -1e-10:
        raise ValueError("tool inertia must be positive semidefinite")
    return tuple(tuple(float(v) for v in row) for row in result)


@dataclass(frozen=True)
class ToolConfig:
    name: str
    mass_kg: float
    com_xyz_m: tuple[float, float, float]
    inertia_tensor_com_kg_m2: tuple[tuple[float, ...], ...]
    tcp_translation_xyz_m: tuple[float, float, float]
    tcp_rotation_matrix: tuple[tuple[float, ...], ...]
    mass_properties_source: str
    mass_properties_reference_frame: str
    source: Mapping[str, Any]
    config_path: Path
    config_hash: str
    data: Mapping[str, Any]

    @property
    def tcp_transform(self) -> dict[str, Any]:
        return {
            "translation_xyz_m": list(self.tcp_translation_xyz_m),
            "rotation_matrix": [list(row) for row in self.tcp_rotation_matrix],
        }

    def to_mapping(self) -> dict[str, Any]:
        return {
            **dict(self.data),
            "name": self.name,
            "mass_kg": self.mass_kg,
            "com_xyz_m": list(self.com_xyz_m),
            "inertia_tensor_com_kg_m2": [list(row) for row in self.inertia_tensor_com_kg_m2],
            "tcp_xyz_m": list(self.tcp_translation_xyz_m),
            "tcp_transform": self.tcp_transform,
            "resolved_config_path": str(self.config_path),
            "config_hash": self.config_hash,
        }


def _resolve_tool_document(path: str | Path) -> tuple[dict[str, Any], Path]:
    data, resolved = _read_yaml(path)
    if data.get("schema_version") != "tool_load_alias_v1":
        return data, resolved
    unexpected = set(data) - {"schema_version", "extends", "description"}
    if unexpected:
        raise ValueError(f"tool alias {resolved} cannot override physical fields: {sorted(unexpected)}")
    target_value = data.get("extends")
    if not isinstance(target_value, str) or not target_value.strip():
        raise ValueError(f"tool alias {resolved} requires a non-empty extends path")
    target = Path(target_value)
    if not target.is_absolute():
        target = resolved.parent / target
    target_data, target_path = _read_yaml(target)
    if target_data.get("schema_version") == "tool_load_alias_v1":
        raise ValueError("tool aliases must point directly to a physical tool config")
    return target_data, target_path


def load_tool_config(path: str | Path) -> ToolConfig:
    """Load a physical tool document, resolving a metadata-only alias."""
    data, resolved = _resolve_tool_document(path)
    if data.get("schema_version") not in {"tool_load_v1", "tool_load_v2"}:
        raise ValueError(f"unsupported tool config schema in {resolved}")
    mass = float(data["mass_kg"])
    if not np.isfinite(mass) or mass <= 0.0:
        raise ValueError("tool.mass_kg must be finite and positive")
    inertia_value = data.get("inertia_tensor_com_kg_m2", data.get("inertia_at_com_kg_m2"))
    if inertia_value is None:
        raise ValueError("tool config requires inertia_tensor_com_kg_m2")
    tcp = data.get("tcp_transform", {})
    if tcp and not isinstance(tcp, Mapping):
        raise ValueError("tool.tcp_transform must be a mapping")
    translation = tcp.get("translation_xyz_m", data.get("tcp_xyz_m")) if isinstance(tcp, Mapping) else None
    if translation is None:
        raise ValueError("tool config requires tcp_transform.translation_xyz_m")
    rotation = tcp.get("rotation_matrix", np.eye(3).tolist()) if isinstance(tcp, Mapping) else np.eye(3).tolist()
    rotation_matrix = np.asarray(rotation, dtype=float)
    if rotation_matrix.shape != (3, 3) or not np.all(np.isfinite(rotation_matrix)):
        raise ValueError("tool TCP rotation must be a finite 3x3 matrix")
    if not np.allclose(rotation_matrix.T @ rotation_matrix, np.eye(3), atol=1e-8) or not np.isclose(
        np.linalg.det(rotation_matrix), 1.0, atol=1e-8
    ):
        raise ValueError("tool TCP rotation must be a proper rotation matrix")
    name = str(data.get("name", "")).strip()
    if not name:
        raise ValueError("tool.name must be non-empty")
    source = data.get("source", {})
    if not isinstance(source, Mapping):
        raise ValueError("tool.source must be a mapping")
    return ToolConfig(
        name=name,
        mass_kg=mass,
        com_xyz_m=_vector3(data["com_xyz_m"], "tool.com_xyz_m"),
        inertia_tensor_com_kg_m2=_inertia3(inertia_value),
        tcp_translation_xyz_m=_vector3(translation, "tool.tcp_transform.translation_xyz_m"),
        tcp_rotation_matrix=_matrix3(rotation_matrix, "tool.tcp_transform.rotation_matrix"),
        mass_properties_source=str(data.get("tool_mass_properties_source", "ENGINEERING_MODEL")),
        mass_properties_reference_frame=str(data.get("mass_properties_reference_frame", "flange")),
        source=dict(source),
        config_path=resolved,
        config_hash=sha256_file(resolved),
        data=dict(data),
    )


def _resolve_relative(path_value: object, base_directory: Path) -> Path | None:
    if path_value is None:
        return None
    path = Path(str(path_value))
    if path.is_absolute():
        return path.resolve()
    candidate = (base_directory / path).resolve()
    if candidate.exists():
        return candidate
    repository_relative = (base_directory.parent / path).resolve()
    return repository_relative if repository_relative.exists() else candidate


def build_cache_identity(
    robot_config: Mapping[str, Any],
    tool: ToolConfig,
    *,
    base_directory: str | Path = ".",
) -> dict[str, Any]:
    """Build the minimum identity required for safe trajectory-cache reuse."""
    base = Path(base_directory).resolve()
    model_id = normalize_robot_model_id(robot_config.get("model"))
    model_document: Mapping[str, Any] = {}
    model_config_path = _resolve_relative(robot_config.get("model_config"), base)
    if model_config_path is not None:
        model_document, _ = _read_yaml(model_config_path)
        configured_model = normalize_robot_model_id(model_document.get("model"))
        if configured_model != model_id:
            raise ValueError("runtime robot model does not match model_config")
    kinematics = model_document.get("kinematics", {}) if isinstance(model_document, Mapping) else {}
    if not isinstance(kinematics, Mapping):
        raise ValueError("robot kinematics must be a mapping")
    urdf_value = robot_config.get("urdf_path", kinematics.get("urdf_path"))
    urdf_base = model_config_path.parent if model_config_path is not None and "urdf_path" not in robot_config else base
    urdf_path = _resolve_relative(urdf_value, urdf_base)
    kinematic_document: dict[str, Any] = {
        "robot_model_id": model_id,
        "base_link": str(kinematics.get("base_link", robot_config.get("base_link", "base_link"))),
        "flange_link": str(kinematics.get("flange_link", robot_config.get("flange_link", "flange"))),
        "tip_link": str(robot_config.get("tip_link", kinematics.get("tip_link", "tool0"))),
        "active_joint_names": list(kinematics.get("active_joint_names", robot_config.get("active_joint_names", ()))),
        "urdf_sha256": sha256_file(urdf_path) if urdf_path is not None and urdf_path.is_file() else None,
    }
    if model_config_path is not None:
        kinematic_document["model_config_sha256"] = sha256_file(model_config_path)
    return {
        "schema_version": "robot_tool_cache_identity_v1",
        "robot_model_id": model_id,
        "robot_kinematic_hash": _canonical_hash(kinematic_document),
        "tool_name": tool.name,
        "tool_mass_kg": tool.mass_kg,
        "tool_config_hash": tool.config_hash,
        "tcp_transform": tool.tcp_transform,
    }


def build_scene_cache_identity(config: Mapping[str, Any]) -> dict[str, Any]:
    """Build an identity from a mapping returned by ``load_scene_config``."""
    robot = config.get("robot")
    tool = config.get("tool")
    if not isinstance(robot, Mapping):
        raise ValueError("scene config requires a robot mapping")
    if not isinstance(tool, Mapping) or not tool.get("resolved_config_path"):
        raise ValueError("scene config requires a resolved physical tool config")
    scene_path = Path(str(config.get("_resolved_config_path", "."))).resolve()
    return build_cache_identity(
        robot,
        load_tool_config(str(tool["resolved_config_path"])),
        base_directory=scene_path.parent,
    )


def validate_cache_identity(stored: Mapping[str, Any], expected: Mapping[str, Any]) -> None:
    """Reject missing or mismatched robot/tool cache identities."""
    missing = [field for field in CACHE_IDENTITY_FIELDS if field not in stored]
    if missing:
        raise ValueError(f"cache identity is missing required fields: {', '.join(missing)}")
    normalized_stored = dict(stored)
    normalized_stored["robot_model_id"] = normalize_robot_model_id(stored["robot_model_id"])
    normalized_expected = dict(expected)
    normalized_expected["robot_model_id"] = normalize_robot_model_id(expected["robot_model_id"])
    differences = [
        field
        for field in CACHE_IDENTITY_FIELDS
        if normalized_stored.get(field) != normalized_expected.get(field)
    ]
    if differences:
        raise ValueError(f"cache identity mismatch: {', '.join(differences)}")
