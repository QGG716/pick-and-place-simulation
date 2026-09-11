"""Strict, lightweight audits for supplied CAD and derived mesh assets.

The loader intentionally distinguishes source integrity from conversion and
execution qualification.  It uses only the Python standard library and
PyYAML; importing it never imports a CAD kernel or a simulator backend.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import io
import json
import math
from pathlib import Path, PurePosixPath
import re
import struct
from typing import Any, Mapping, Sequence
import zipfile
import xml.etree.ElementTree as ET

import yaml


ROBOT_SCHEMA = "m710id70_cad_asset_manifest_v1"
TOOL_SCHEMA = "shanghai_wantai_three_zone_asset_manifest_v1"
LINK_MAPPING_SCHEMA = "m710id70_link_mapping_v1"
OFFICIAL_M710ID70_SCHEMA = "fanuc_official_description_provenance_v1"

OFFICIAL_M710ID70_REPOSITORY = "https://github.com/FANUC-CORPORATION/fanuc_description.git"
OFFICIAL_M710ID70_COMMIT = "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2"
OFFICIAL_M710ID70_PACKAGE_VERSION = "2.3.0"
OFFICIAL_M710ID70_SOURCE_FILES: Mapping[str, tuple[int, str]] = {
    "LICENSES/Apache-2.0.txt": (
        10_280,
        "074e6e32c86a4c0ef8b3ed25b721ca23aca83df277cd88106ef7177c354615ff",
    ),
    "fanuc_m710_description/package.xml": (
        986,
        "3e172da46846c21ad46eecb86de0e1bcb09397a47b6d79c98c6103cf6fad8a71",
    ),
    "fanuc_m710_description/robot/m710id_70.urdf.xacro": (
        371,
        "d74ba41dd6f9bc97f4f7745e9bee0c0096d6880b12191e3ef6eebd5c70b835b6",
    ),
    "fanuc_m710_description/urdf/m710id_70_urdf_macro.xacro": (
        11_581,
        "ede96c55fd7ec665a259d6f09422e37d8b69a2d27e3adc412f313930f37ed28d",
    ),
    "fanuc_m710_description/meshes/m710id_70/collision/base.stl": (
        14_884,
        "7335c642059e09de7f58d5f186b4dd5d9fb5e3ac22584434716f6b1eff3b8469",
    ),
    "fanuc_m710_description/meshes/m710id_70/collision/j1.stl": (
        316_484,
        "af6c8411af44c282174a4ebcdd90d1ee2d2e716deff7f12ebba0466805fef7a1",
    ),
    "fanuc_m710_description/meshes/m710id_70/collision/j2.stl": (
        150_684,
        "cde344ebf927e39110800b3e54e90f85826cbc21452e55d94abd5b22ba61db15",
    ),
    "fanuc_m710_description/meshes/m710id_70/collision/j3.stl": (
        262_984,
        "49116b78b394c8e0b60ca6b23f4391438ab04e702854fe4a4468b4c132c4c080",
    ),
    "fanuc_m710_description/meshes/m710id_70/collision/j4.stl": (
        30_084,
        "6b03fcd7ee9832f48e96570d10b30ae2dd9a9ccec4c0389c6308eeef86446d25",
    ),
    "fanuc_m710_description/meshes/m710id_70/collision/j5.stl": (
        96_084,
        "516e34e9174b61464cb8a494d2c1ce5498e48acea8aaecadd41977d1e6d6e15a",
    ),
    "fanuc_m710_description/meshes/m710id_70/collision/j6.stl": (
        11_084,
        "ed4fbf513eab9c8a8869a53e89f3d4f649ea586a3833d8e8c8e9096e4e0bbcfd",
    ),
    "fanuc_m710_description/meshes/m710id_70/visual/base.dae": (
        675_313,
        "35ee039f1d466ea7f0229f34edeb6b47bc81768d4b4013e724edd26297c0b537",
    ),
    "fanuc_m710_description/meshes/m710id_70/visual/j1.dae": (
        965_072,
        "4591de5f409f675f7e10736c6b6cf5bf6a7bc8bad9bb9856c5e1e11233c3fca7",
    ),
    "fanuc_m710_description/meshes/m710id_70/visual/j2.dae": (
        1_438_918,
        "e899cbe3a1082e6a8c85dbac13711bba23ab158eb310126d914958bcc3713928",
    ),
    "fanuc_m710_description/meshes/m710id_70/visual/j3.dae": (
        1_311_883,
        "ff35b1faba0b498915f01890887895fa7824dda213d3a60c6cef0c91ae344c88",
    ),
    "fanuc_m710_description/meshes/m710id_70/visual/j4.dae": (
        2_917_370,
        "5bcefe962dbd6c7625a6b1b5be28442881e61745838d7865f3204d1d7923b80b",
    ),
    "fanuc_m710_description/meshes/m710id_70/visual/j5.dae": (
        329_365,
        "1be4482d6f520c7babee25984e50cc2b8049360aa0013d61805b098a0546324a",
    ),
    "fanuc_m710_description/meshes/m710id_70/visual/j6.dae": (
        113_916,
        "d86c8fa173db6e4225d22b6be04c9a5082534b7ef30905c9c942d7772e159c3b",
    ),
}


class AssetAuditError(ValueError):
    """Raised when an asset contract is malformed or does not match disk."""


@dataclass(frozen=True)
class AssetAuditReport:
    manifest_path: str
    schema_version: str
    manifest_sha256: str
    source_integrity: bool
    conversion_status: str
    execution_qualified: bool
    verified_file_count: int
    verified_source_bytes: int
    missing_converted_outputs: tuple[str, ...]
    unresolved: tuple[str, ...]

    def to_mapping(self) -> dict[str, Any]:
        result = asdict(self)
        result["missing_converted_outputs"] = list(self.missing_converted_outputs)
        result["unresolved"] = list(self.unresolved)
        return result


@dataclass(frozen=True)
class OfficialModelAuditReport:
    """Fail-closed qualification report for the pinned official robot model."""

    manifest_path: str
    manifest_sha256: str
    upstream_commit: str
    package_version: str
    source_integrity: bool
    static_urdf_integrity: bool
    model_semantics: bool
    verified_source_file_count: int
    verified_source_bytes: int
    visual_mesh_count: int
    collision_mesh_count: int
    execution_qualified: bool

    def to_mapping(self) -> dict[str, Any]:
        return asdict(self)


def _expect_mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AssetAuditError(f"{context} must be a mapping")
    return value


def _expect_exact_keys(value: Mapping[str, Any], keys: set[str], context: str) -> None:
    actual = set(value)
    if actual != keys:
        missing = sorted(keys - actual)
        extra = sorted(actual - keys)
        raise AssetAuditError(f"{context} keys mismatch: missing={missing}, extra={extra}")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise AssetAuditError(f"cannot load YAML {path}: {exc}") from exc
    return _expect_mapping(document, str(path))


def _repository_root(manifest_path: Path, repository_root: str | Path | None) -> Path:
    if repository_root is not None:
        root = Path(repository_root).resolve()
        if not root.is_dir():
            raise AssetAuditError(f"repository root is not a directory: {root}")
        return root
    for candidate in manifest_path.resolve().parents:
        if (candidate / "pyproject.toml").is_file() and (candidate / "src" / "unloading_sim").is_dir():
            return candidate
    raise AssetAuditError("repository root could not be inferred; pass repository_root explicitly")


def _resolve_repository_path(root: Path, raw: Any, context: str) -> Path:
    if not isinstance(raw, str) or not raw or "\\" in raw:
        raise AssetAuditError(f"{context} must be a non-empty canonical POSIX repository path")
    posix = PurePosixPath(raw)
    if posix.is_absolute() or ".." in posix.parts or Path(raw).is_absolute():
        raise AssetAuditError(f"{context} must remain within the repository: {raw!r}")
    resolved = (root / Path(*posix.parts)).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise AssetAuditError(f"{context} escaped the repository: {raw!r}") from exc
    return resolved


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _validate_format(data: bytes, declared: str, path: Path) -> None:
    def require(condition: bool, detail: str) -> None:
        if not condition:
            raise AssetAuditError(f"format check failed for {path}: {detail}")

    if declared == "ascii_dxf_ac1021":
        require(data.startswith(b"  0\r\nSECTION"), "not the expected ASCII DXF header")
        require(b"$ACADVER\r\n  1\r\nAC1021" in data, "AC1021 marker missing")
        require(b"$INSUNITS" in data and b"$MEASUREMENT" in data, "unit markers missing")
    elif declared == "parasolid_text_v29":
        require(data.startswith(b"**ABCDEFGHIJKLMNOPQRSTUVWXYZ"), "Parasolid text magic missing")
        require(b"**PARASOLID" in data and b"SCH_2900201" in data, "Parasolid v29 header missing")
    elif declared == "zip":
        require(zipfile.is_zipfile(io.BytesIO(data)), "invalid ZIP archive")
    elif declared == "jt_9_5_dm_8_3_7_1":
        require(data.startswith(b"Version 9.5 JT  DM 8.3.7.1"), "JT version header mismatch")
    elif declared == "process_simulate_motion_text":
        require(b"config_family" in data and b"wrist_joint j5" in data, "motion metadata missing")
    elif declared == "robcad_configuration_text":
        require(b"CONFIGURATION" in data and b"default_turns" in data, "ROBCAD config markers missing")
    elif declared == "robcad_information_text":
        require(b"Robot Name" in data and b"M-710iD/70" in data, "ROBCAD identity missing")
    elif declared == "robcad_mechanism_text":
        require(b"begin COMPONENT m710id_70_v01" in data, "mechanism component missing")
        require(data.count(b"axis_points") == 6, "mechanism must contain six joint axes")
        require(b"follows j2 by 1" in data, "J3 coupling declaration missing")
    elif declared == "robcad_binary":
        require(len(data) > 0, "empty ROBCAD binary")
        if path.name == ".atr":
            require(b"JOINT_AXIS_LINE" in data, "attribute marker missing")
        elif path.name == ".geo":
            require(data.startswith(b"##150004"), "geometry header mismatch")
        elif path.name == ".r":
            require(data.startswith(b"##050005"), "runtime header mismatch")
        elif path.name == ".wm":
            require(data.startswith(b"##030061"), "world-model header mismatch")
    elif declared == "robcad_mixed":
        require(len(data) > 0 and data.startswith(b"P"), "ROBCAD mixed-data header mismatch")
    elif declared == "step_ap214_solidworks_2023":
        require(data.startswith(b"ISO-10303-21;"), "STEP exchange magic missing")
        require(b"STEP AP214" in data and b"SolidWorks 2023" in data, "STEP provenance mismatch")
        require(b"SI_UNIT ( .MILLI., .METRE. )" in data, "STEP millimetre declaration missing")
    elif declared == "png_rgb_957x754":
        require(data.startswith(b"\x89PNG\r\n\x1a\n") and len(data) >= 26, "invalid PNG")
        width, height = struct.unpack(">II", data[16:24])
        require((width, height, data[25]) == (957, 754, 2), "PNG dimensions or RGB type mismatch")
    elif declared == "glb_2_0":
        require(len(data) >= 20, "truncated GLB")
        magic, version, declared_size = struct.unpack_from("<4sII", data)
        require(magic == b"glTF" and version == 2 and declared_size == len(data), "GLB v2 header mismatch")
    elif declared == "binary_stl":
        require(len(data) >= 84, "truncated binary STL")
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        require(len(data) == 84 + 50 * triangle_count, "binary STL length/count mismatch")
    elif declared == "json":
        try:
            parsed = json.loads(data.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise AssetAuditError(f"format check failed for {path}: invalid JSON") from exc
        require(isinstance(parsed, Mapping), "JSON root must be a mapping")
    else:
        raise AssetAuditError(f"unsupported declared asset format {declared!r} for {path}")


def _audit_source_files(
    document: Mapping[str, Any], root: Path
) -> tuple[dict[str, Path], int, int]:
    records = document.get("source_files")
    if not isinstance(records, list) or not records:
        raise AssetAuditError("source_files must be a non-empty list")
    seen_roles: set[str] = set()
    seen_paths: set[str] = set()
    resolved_by_path: dict[str, Path] = {}
    total_bytes = 0
    for index, raw_record in enumerate(records):
        record = _expect_mapping(raw_record, f"source_files[{index}]")
        _expect_exact_keys(record, {"role", "path", "bytes", "sha256", "format"}, f"source_files[{index}]")
        role = record["role"]
        raw_path = record["path"]
        expected_bytes = record["bytes"]
        expected_hash = record["sha256"]
        if not isinstance(role, str) or not role or role in seen_roles:
            raise AssetAuditError(f"source_files[{index}].role must be unique")
        if not isinstance(raw_path, str) or raw_path in seen_paths:
            raise AssetAuditError(f"source_files[{index}].path must be unique")
        if not isinstance(expected_bytes, int) or isinstance(expected_bytes, bool) or expected_bytes <= 0:
            raise AssetAuditError(f"source_files[{index}].bytes must be a positive integer")
        if not isinstance(expected_hash, str) or len(expected_hash) != 64 or expected_hash != expected_hash.lower():
            raise AssetAuditError(f"source_files[{index}].sha256 must be lowercase SHA-256")
        path = _resolve_repository_path(root, raw_path, f"source_files[{index}].path")
        if not path.is_file():
            raise AssetAuditError(f"required source asset is missing: {raw_path}")
        data = path.read_bytes()
        if len(data) != expected_bytes:
            raise AssetAuditError(f"byte count mismatch for {raw_path}: {len(data)} != {expected_bytes}")
        actual_hash = _sha256(data)
        if actual_hash != expected_hash:
            raise AssetAuditError(f"SHA-256 mismatch for {raw_path}: {actual_hash} != {expected_hash}")
        _validate_format(data, record["format"], path)
        seen_roles.add(role)
        seen_paths.add(raw_path)
        resolved_by_path[raw_path] = path
        total_bytes += expected_bytes
    return resolved_by_path, len(records), total_bytes


def _audit_archives(document: Mapping[str, Any], root: Path, source_paths: Mapping[str, Path]) -> None:
    archives = document.get("archives", [])
    if not isinstance(archives, list):
        raise AssetAuditError("archives must be a list")
    for index, raw_record in enumerate(archives):
        record = _expect_mapping(raw_record, f"archives[{index}]")
        _expect_exact_keys(
            record,
            {"path", "file_member_count", "integrity", "expanded_base_path", "ignored_archive_members"},
            f"archives[{index}]",
        )
        raw_path = record["path"]
        if raw_path not in source_paths:
            raise AssetAuditError(f"archive is not declared in source_files: {raw_path}")
        if record["integrity"] != "CRC_VERIFIED_ALL_MEMBERS":
            raise AssetAuditError(f"archive integrity policy is not fail-closed: {raw_path}")
        ignored = record["ignored_archive_members"]
        if not isinstance(ignored, list) or any(not isinstance(item, str) for item in ignored):
            raise AssetAuditError(f"archives[{index}].ignored_archive_members must be a string list")
        expanded_base = _resolve_repository_path(root, record["expanded_base_path"], f"archives[{index}].expanded_base_path")
        with zipfile.ZipFile(source_paths[raw_path]) as archive:
            bad_member = archive.testzip()
            if bad_member is not None:
                raise AssetAuditError(f"ZIP CRC failure in {raw_path}: {bad_member}")
            members = [item for item in archive.infolist() if not item.is_dir()]
            if len(members) != record["file_member_count"]:
                raise AssetAuditError(f"ZIP member count mismatch for {raw_path}")
            member_names = {item.filename for item in members}
            if not set(ignored).issubset(member_names):
                raise AssetAuditError(f"ignored ZIP member not present in {raw_path}")
            for member in members:
                if member.filename in ignored:
                    continue
                member_path = _resolve_repository_path(
                    expanded_base,
                    member.filename,
                    f"archive member {member.filename}",
                )
                if not member_path.is_file() or member_path.read_bytes() != archive.read(member):
                    raise AssetAuditError(f"expanded ZIP member mismatch: {member.filename}")


def _audit_reference(root: Path, reference: Any, context: str) -> Path:
    record = _expect_mapping(reference, context)
    keys = set(record)
    if keys not in ({"path", "sha256"}, {"path", "bytes", "sha256"}):
        raise AssetAuditError(
            f"{context} keys mismatch: expected path/sha256 with optional bytes, got={sorted(keys)}"
        )
    path = _resolve_repository_path(root, record["path"], f"{context}.path")
    if not path.is_file():
        raise AssetAuditError(f"referenced file is missing: {record['path']}")
    data = path.read_bytes()
    if "bytes" in record:
        expected_bytes = record["bytes"]
        if not isinstance(expected_bytes, int) or isinstance(expected_bytes, bool) or expected_bytes <= 0:
            raise AssetAuditError(f"{context}.bytes must be a positive integer")
        if len(data) != expected_bytes:
            raise AssetAuditError(
                f"referenced-file byte count mismatch for {record['path']}: "
                f"{len(data)} != {expected_bytes}"
            )
    actual_hash = _sha256(data)
    if actual_hash != record["sha256"]:
        raise AssetAuditError(f"referenced-file SHA-256 mismatch for {record['path']}")
    return path


def _normalised_axis(point_1: Sequence[Any], point_2: Sequence[Any]) -> list[float]:
    if len(point_1) != 3 or len(point_2) != 3:
        raise AssetAuditError("joint axis points must contain three values")
    vector = [float(b) - float(a) for a, b in zip(point_1, point_2)]
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 0.0 or not math.isfinite(norm):
        raise AssetAuditError("joint axis points must define a finite nonzero direction")
    return [value / norm for value in vector]


def load_and_audit_link_mapping(
    mapping_path: str | Path,
    *,
    repository_root: str | Path | None = None,
) -> Mapping[str, Any]:
    path = Path(mapping_path).resolve()
    root = _repository_root(path, repository_root)
    document = _load_yaml(path)
    _expect_exact_keys(
        document,
        {
            "schema_version", "mapping_id", "model", "source_mechanism", "source_world_model",
            "units", "source_component", "zero_pose", "joints", "links", "mesh_conversion_rule", "unresolved",
        },
        "link mapping",
    )
    if document["schema_version"] != LINK_MAPPING_SCHEMA:
        raise AssetAuditError(f"unsupported link mapping schema: {document['schema_version']!r}")
    mechanism_path = _audit_reference(root, document["source_mechanism"], "source_mechanism")
    world_path = _audit_reference(root, document["source_world_model"], "source_world_model")
    mechanism = mechanism_path.read_bytes()
    world = world_path.read_bytes()
    joints = document["joints"]
    if not isinstance(joints, list) or [joint.get("name") for joint in joints] != [f"J{i}" for i in range(1, 7)]:
        raise AssetAuditError("link mapping must contain ordered joints J1 through J6")
    expected_axes = {
        "J1": [0.0, 0.0, 1.0], "J2": [0.0, 1.0, 0.0], "J3": [0.0, -1.0, 0.0],
        "J4": [-1.0, 0.0, 0.0], "J5": [0.0, -1.0, 0.0], "J6": [-1.0, 0.0, 0.0],
    }
    for joint in joints:
        axis = _normalised_axis(joint["axis_point_1_mm"], joint["axis_point_2_mm"])
        declared = [float(value) for value in joint["axis_direction"]]
        if any(abs(a - b) > 1e-12 for a, b in zip(axis, declared)) or declared != expected_axes[joint["name"]]:
            raise AssetAuditError(f"axis mismatch for {joint['name']}")
        literal = " ".join(str(int(value)) if float(value).is_integer() else str(value) for value in joint["axis_point_1_mm"])
        if literal.encode("ascii") not in mechanism:
            raise AssetAuditError(f"first axis point for {joint['name']} not found in ROBCAD mechanism")
    j3 = joints[2]
    if j3.get("follows", {}).get("source_literal") != "follows j2 by 1" or j3["follows"]["interpretation_status"] != "UNRESOLVED_CONTROLLER_TO_KINEMATIC_MAPPING":
        raise AssetAuditError("J3 follows-J2 coupling must remain explicitly unresolved")
    links = document["links"]
    expected_pairs = [
        ("k1", "base_link"), ("k2", "J1_link"), ("k3", "J2_link"), ("k4", "J3_link"),
        ("k5", "J4_link"), ("k6", "J5_link"), ("k7", "J6_link"),
    ]
    if not isinstance(links, list) or [(item.get("robcad_link"), item.get("urdf_link")) for item in links] != expected_pairs:
        raise AssetAuditError("ROBCAD-to-URDF link order mismatch")
    bodies = [body for link in links for body in link.get("bodies", [])]
    if len(bodies) != 21 or len(set(bodies)) != 21:
        raise AssetAuditError("link mapping must contain 21 unique ROBCAD bodies")
    for body in bodies:
        if not isinstance(body, str) or body.encode("ascii") not in world:
            raise AssetAuditError(f"mapped body not found in ROBCAD world model: {body!r}")
    zero = _expect_mapping(document["zero_pose"], "zero_pose")
    if zero.get("current_urdf_to_source_tcp_relative_angle_deg") != 180.0 or zero.get("clocking_status") != "UNRESOLVED_180_DEG_ABOUT_COMMON_TOOL_Z":
        raise AssetAuditError("tool0 clocking mismatch must remain explicitly unresolved")
    required_unresolved = {
        "J3_FOLLOWS_J2_CONTROLLER_TO_KINEMATIC_MAPPING",
        "TOOL0_CLOCKING_180_DEG_ABOUT_COMMON_TOOL_Z",
    }
    if not required_unresolved.issubset(set(document["unresolved"])):
        raise AssetAuditError("required link-mapping uncertainties were removed")
    if document["mesh_conversion_rule"].get("status") != "SOURCE_MAPPING_AVAILABLE_CONVERSION_REQUIRED":
        raise AssetAuditError("link mesh conversion must remain pending until outputs exist")
    return document


def _audit_expected_outputs(document: Mapping[str, Any], root: Path) -> tuple[str, ...]:
    outputs = _expect_mapping(document["expected_converted_outputs"], "expected_converted_outputs")
    _expect_exact_keys(outputs, {"status", "visual_meshes", "collision_meshes"}, "expected_converted_outputs")
    if outputs["status"] != "SOURCE_AVAILABLE_CONVERSION_REQUIRED":
        raise AssetAuditError("converted-output status must describe the current source-only state")
    missing: list[str] = []
    for category in ("visual_meshes", "collision_meshes"):
        records = outputs[category]
        if not isinstance(records, list) or len(records) != 7 or len(set(records)) != 7:
            raise AssetAuditError(f"expected_converted_outputs.{category} must list seven unique link meshes")
        for raw_path in records:
            if not _resolve_repository_path(root, raw_path, f"expected_converted_outputs.{category}").is_file():
                missing.append(raw_path)
    return tuple(missing)


def _audit_robot_manifest(document: Mapping[str, Any], root: Path) -> tuple[str, ...]:
    _expect_exact_keys(
        document,
        {
            "schema_version", "asset_id", "manufacturer", "model", "source_status", "conversion_status",
            "execution_qualified", "source_file_total_count", "source_file_total_bytes", "link_mapping",
            "units", "source_files", "archives", "expected_converted_outputs", "unresolved", "qualification_policy",
        },
        "robot asset manifest",
    )
    if document["source_status"] != "AVAILABLE_HASH_VERIFIED" or document["conversion_status"] != "SOURCE_AVAILABLE_CONVERSION_REQUIRED":
        raise AssetAuditError("robot CAD source/conversion status is inconsistent")
    mapping_path = _audit_reference(root, document["link_mapping"], "link_mapping")
    load_and_audit_link_mapping(mapping_path, repository_root=root)
    missing = _audit_expected_outputs(document, root)
    if document["execution_qualified"] is not False:
        raise AssetAuditError("robot execution_qualified must remain false while converted link meshes are missing")
    if len(missing) != 14:
        raise AssetAuditError("source-only robot manifest must expose all 14 missing per-link mesh outputs")
    required = {
        "J3_FOLLOWS_J2_CONTROLLER_TO_KINEMATIC_MAPPING",
        "TOOL0_CLOCKING_180_DEG_ABOUT_COMMON_TOOL_Z",
        "PER_LINK_VISUAL_MESHES_MISSING",
        "PER_LINK_COLLISION_MESHES_MISSING",
    }
    if not required.issubset(set(document["unresolved"])):
        raise AssetAuditError("required robot conversion blockers were removed")
    return missing


def _audit_tool_manifest(document: Mapping[str, Any], root: Path, source_paths: Mapping[str, Path]) -> tuple[str, ...]:
    _expect_exact_keys(
        document,
        {
            "schema_version", "asset_id", "manufacturer", "product_description", "source_status",
            "conversion_status", "execution_qualified", "source_files", "units", "step_assembly", "frames",
            "contact_geometry", "collision_geometry", "visual_geometry", "mass_properties", "zones",
            "conversion_provenance", "unresolved", "qualification_policy",
        },
        "tool asset manifest",
    )
    if document["source_status"] != "AVAILABLE_HASH_VERIFIED" or document["conversion_status"] != "DERIVED_ASSETS_PRESENT_ENGINEERING_ONLY":
        raise AssetAuditError("tool source/conversion status is inconsistent")
    if not isinstance(document["execution_qualified"], bool):
        raise AssetAuditError("historical tool execution_qualified must be a boolean")
    contact = _expect_mapping(document["contact_geometry"], "contact_geometry")
    physical = float(contact["flange_to_uncompressed_cup_extreme_m"])
    compression = float(contact["configured_cup_compression_m"])
    task_tcp = float(contact["active_task_tcp_from_flange_m"])
    checks = (
        (physical - compression, float(contact["flange_to_nominal_compressed_contact_plane_m"])),
        (task_tcp - physical, float(contact["task_tcp_beyond_uncompressed_extreme_m"])),
        (task_tcp - (physical - compression), float(contact["task_tcp_beyond_nominal_compressed_contact_plane_m"])),
    )
    if any(abs(left - right) > 1e-12 for left, right in checks):
        raise AssetAuditError("tool contact-plane/TCP offsets are internally inconsistent")
    if contact["attachment_policy"] != "validate_actual_cup_working_plane_contact_never_attach_at_virtual_task_tcp":
        raise AssetAuditError("tool attachment policy must use the physical cup plane")
    collision = _expect_mapping(document["collision_geometry"], "collision_geometry")
    if collision["solid_occurrence_count"] != 202 or collision["deformable_cup_and_insert_solid_count"] != 144 or collision["rigid_solid_count"] != 58:
        raise AssetAuditError("tool solid classification mismatch")
    if collision["final_dynamic_collision_status"] != "NOT_QUALIFIED" or not collision["full_shape_stl_contains_compliant_cup_geometry"]:
        raise AssetAuditError("full-shape STL must not be treated as qualified rigid collision geometry")
    analysis = _audit_reference(root, document["conversion_provenance"]["analysis_script"], "analysis_script")
    if analysis.name != "analyze_gripper_step.py":
        raise AssetAuditError("unexpected gripper analysis script")
    mass_path = next(
        (source_paths[item["path"]] for item in document["source_files"] if item["role"] == "derived_mass_and_contact_analysis"),
        None,
    )
    if mass_path is None:
        raise AssetAuditError("derived mass/contact analysis record missing")
    mass = json.loads(mass_path.read_text(encoding="utf-8"))
    if mass.get("solid_occurrence_count") != 202 or len(mass.get("deformable_cup_solid_indices", [])) != 144 or len(mass.get("rigid_collision_bounding_boxes_step_mm", [])) != 58:
        raise AssetAuditError("mass-properties JSON does not match the declared solid classification")
    if mass.get("flange_origin_step_mm") != document["frames"]["flange_origin_step_mm"]:
        raise AssetAuditError("tool flange origin does not match mass-properties JSON")
    if mass.get("rotation_step_from_tool") != document["frames"]["rotation_step_from_tool"]:
        raise AssetAuditError("tool frame rotation does not match mass-properties JSON")
    return ()


def load_and_audit_asset_manifest(
    manifest_path: str | Path,
    *,
    repository_root: str | Path | None = None,
) -> AssetAuditReport:
    """Load one supported manifest and fail on any source or semantic drift."""

    path = Path(manifest_path).resolve()
    root = _repository_root(path, repository_root)
    try:
        repository_manifest_path = path.relative_to(root).as_posix()
    except ValueError:
        # Auditing a copied manifest is useful for tamper regressions.  Its
        # referenced source paths are still resolved fail-closed against
        # ``root`` by _audit_source_files(); only the report label is external.
        repository_manifest_path = path.as_posix()
    document = _load_yaml(path)
    schema = document.get("schema_version")
    if schema not in {ROBOT_SCHEMA, TOOL_SCHEMA}:
        raise AssetAuditError(f"unsupported asset manifest schema: {schema!r}")
    source_paths, count, total_bytes = _audit_source_files(document, root)
    if schema == ROBOT_SCHEMA:
        if count != document["source_file_total_count"] or total_bytes != document["source_file_total_bytes"]:
            raise AssetAuditError("robot source-file totals do not match audited records")
        _audit_archives(document, root, source_paths)
        thumbs = list((root / "res" / "M-710iD_70").rglob("Thumbs.db"))
        if thumbs:
            raise AssetAuditError(f"expanded source tree contains forbidden Thumbs.db: {thumbs}")
        missing = _audit_robot_manifest(document, root)
    else:
        missing = _audit_tool_manifest(document, root, source_paths)
    # The manifest preserves the historical conversion decision.  Current
    # simulation geometry readiness is derived from the independently
    # regenerated STEP coverage, including the restored rigid cup inserts.
    # Vacuum/material certification remains separate evidence and does not
    # replace these geometry checks or veto an explicit ideal-cup simulation.
    unresolved = tuple(str(item) for item in document["unresolved"])
    if schema == TOOL_SCHEMA:
        from .tool_geometry import audit_tool_geometry

        geometry = audit_tool_geometry(root)
        execution_qualified = bool(geometry["simulation_geometry_qualified"])
        if execution_qualified:
            unresolved = tuple(
                item for item in unresolved
                if item != "DYNAMIC_COLLISION_REPRESENTATION_NOT_QUALIFIED"
            )
    else:
        execution_qualified = not missing
    return AssetAuditReport(
        manifest_path=repository_manifest_path,
        schema_version=str(schema),
        manifest_sha256=_sha256(path.read_bytes()),
        source_integrity=True,
        conversion_status=str(document["conversion_status"]),
        execution_qualified=execution_qualified,
        verified_file_count=count,
        verified_source_bytes=total_bytes,
        missing_converted_outputs=missing,
        unresolved=unresolved,
    )


def audit_m710id70_asset_set(repository_root: str | Path) -> dict[str, Any]:
    """Audit the robot source bundle, link mapping and Wantai tool together."""

    root = Path(repository_root).resolve()
    robot = load_and_audit_asset_manifest(
        root / "assets" / "robots" / "fanuc_m710id_70" / "cad_asset_manifest.yaml",
        repository_root=root,
    )
    tool = load_and_audit_asset_manifest(
        root / "assets" / "grippers" / "shanghai_wantai_three_zone" / "asset_manifest.yaml",
        repository_root=root,
    )
    return {
        "status": "SOURCE_INTEGRITY_PASS_CONVERSION_REQUIRED",
        "execution_qualified": robot.execution_qualified and tool.execution_qualified,
        "robot": robot.to_mapping(),
        "tool": tool.to_mapping(),
    }


_OFFICIAL_ROOT = PurePosixPath("assets/robots/fanuc_m710id_70/official")
_XACRO_NAMESPACE = "http://wiki.ros.org/xacro"
_XACRO = f"{{{_XACRO_NAMESPACE}}}"


def _official_expected_format(upstream_path: str) -> str:
    if upstream_path == "LICENSES/Apache-2.0.txt":
        return "apache_2_0_text"
    if upstream_path.endswith("package.xml"):
        return "ros_package_xml"
    if upstream_path.endswith(".xacro"):
        return "xacro_xml"
    if upstream_path.endswith(".dae"):
        return "collada_metre_z_up"
    if upstream_path.endswith(".stl"):
        return "binary_stl"
    raise AssetAuditError(f"unsupported official FANUC source format: {upstream_path}")


def _validate_official_format(data: bytes, declared: str, path: Path) -> None:
    if declared == "binary_stl":
        _validate_format(data, declared, path)
        return
    if declared == "apache_2_0_text":
        if not data.startswith(b"Apache License\nVersion 2.0, January 2004\n"):
            raise AssetAuditError(f"official Apache-2.0 license text mismatch: {path}")
        return
    if declared in {"ros_package_xml", "xacro_xml", "collada_metre_z_up"}:
        try:
            root = ET.fromstring(data)
        except ET.ParseError as exc:
            raise AssetAuditError(f"invalid official XML asset {path}: {exc}") from exc
        if declared == "ros_package_xml" and root.tag != "package":
            raise AssetAuditError(f"official package.xml root mismatch: {path}")
        if declared == "xacro_xml" and root.tag != "robot":
            raise AssetAuditError(f"official xacro root mismatch: {path}")
        if declared == "collada_metre_z_up":
            if not root.tag.endswith("COLLADA"):
                raise AssetAuditError(f"official DAE is not COLLADA: {path}")
            asset = next((item for item in root if item.tag.endswith("asset")), None)
            if asset is None:
                raise AssetAuditError(f"official DAE has no asset metadata: {path}")
            unit = next((item for item in asset if item.tag.endswith("unit")), None)
            up_axis = next((item for item in asset if item.tag.endswith("up_axis")), None)
            if unit is None or float(unit.attrib.get("meter", "nan")) != 1.0:
                raise AssetAuditError(f"official DAE unit is not one metre: {path}")
            if up_axis is None or (up_axis.text or "").strip() != "Z_UP":
                raise AssetAuditError(f"official DAE is not Z-up: {path}")
        return
    raise AssetAuditError(f"unsupported official FANUC format declaration: {declared!r}")


def _audit_official_source_files(
    document: Mapping[str, Any], root: Path
) -> tuple[Mapping[str, Path], int]:
    records = document.get("source_files")
    if not isinstance(records, list):
        raise AssetAuditError("official source_files must be a list")
    if len(records) != len(OFFICIAL_M710ID70_SOURCE_FILES):
        raise AssetAuditError("official source_files must contain the pinned 18-file asset set")
    by_upstream: dict[str, Path] = {}
    seen_roles: set[str] = set()
    total_bytes = 0
    for index, raw_record in enumerate(records):
        record = _expect_mapping(raw_record, f"official source_files[{index}]")
        _expect_exact_keys(
            record,
            {"role", "path", "upstream_path", "bytes", "sha256", "format"},
            f"official source_files[{index}]",
        )
        role = record["role"]
        upstream_path = record["upstream_path"]
        if not isinstance(role, str) or not role or role in seen_roles:
            raise AssetAuditError(f"official source_files[{index}].role must be unique")
        if not isinstance(upstream_path, str) or upstream_path in by_upstream:
            raise AssetAuditError(f"official source_files[{index}].upstream_path must be unique")
        if upstream_path not in OFFICIAL_M710ID70_SOURCE_FILES:
            raise AssetAuditError(f"unrecognised pinned upstream file: {upstream_path!r}")
        expected_bytes, expected_sha256 = OFFICIAL_M710ID70_SOURCE_FILES[upstream_path]
        expected_path = (_OFFICIAL_ROOT / PurePosixPath(upstream_path)).as_posix()
        if record["path"] != expected_path:
            raise AssetAuditError(f"official source path is not canonical: {record['path']!r}")
        if record["bytes"] != expected_bytes or record["sha256"] != expected_sha256:
            raise AssetAuditError(f"pinned byte identity mismatch for {upstream_path}")
        expected_format = _official_expected_format(upstream_path)
        if record["format"] != expected_format:
            raise AssetAuditError(f"format declaration mismatch for {upstream_path}")
        source_path = _resolve_repository_path(root, record["path"], f"official source_files[{index}].path")
        if not source_path.is_file():
            raise AssetAuditError(f"required official source asset is missing: {record['path']}")
        data = source_path.read_bytes()
        if len(data) != expected_bytes:
            raise AssetAuditError(f"byte count mismatch for official source {record['path']}")
        if _sha256(data) != expected_sha256:
            raise AssetAuditError(f"SHA-256 mismatch for official source {record['path']}")
        _validate_official_format(data, expected_format, source_path)
        seen_roles.add(role)
        by_upstream[upstream_path] = source_path
        total_bytes += expected_bytes
    if set(by_upstream) != set(OFFICIAL_M710ID70_SOURCE_FILES):
        raise AssetAuditError("official source asset set differs from the pinned commit")
    if document.get("source_file_total_count") != len(by_upstream):
        raise AssetAuditError("official source-file count does not match audited records")
    if document.get("source_file_total_bytes") != total_bytes:
        raise AssetAuditError("official source byte total does not match audited records")
    return by_upstream, total_bytes


def _float_vector(raw: Any, length: int, context: str) -> tuple[float, ...]:
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)) or len(raw) != length:
        raise AssetAuditError(f"{context} must contain {length} numeric values")
    result = tuple(float(value) for value in raw)
    if not all(math.isfinite(value) for value in result):
        raise AssetAuditError(f"{context} must contain finite values")
    return result


def _xml_vector(element: ET.Element | None, attribute: str, context: str) -> tuple[float, ...]:
    if element is None or attribute not in element.attrib:
        raise AssetAuditError(f"{context}.{attribute} is missing")
    values = element.attrib[attribute].split()
    return _float_vector(values, 3, f"{context}.{attribute}")


def _assert_vector_close(left: Sequence[Any], right: Sequence[Any], context: str) -> None:
    lhs = _float_vector(left, len(right), context)
    rhs = tuple(float(value) for value in right)
    if any(abs(a - b) > 1e-12 for a, b in zip(lhs, rhs)):
        raise AssetAuditError(f"{context} mismatch: {lhs!r} != {rhs!r}")


def _unprefixed(raw: str, context: str) -> str:
    prefix = "${prefix}"
    if not raw.startswith(prefix):
        raise AssetAuditError(f"{context} does not use the official prefix parameter")
    return raw[len(prefix):]


_RADIANS_EXPRESSION = re.compile(
    r"^\$\{radians\(\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))\s*\)\}$"
)
_RADIANS_TOKEN = re.compile(
    r"\$\{radians\(\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))\s*\)\}"
)


def _xacro_property_number(raw: str, context: str) -> float:
    match = _RADIANS_EXPRESSION.fullmatch(raw.strip())
    if match is not None:
        return math.radians(float(match.group(1)))
    try:
        result = float(raw)
    except ValueError as exc:
        raise AssetAuditError(f"unsupported numeric xacro expression for {context}: {raw!r}") from exc
    if not math.isfinite(result):
        raise AssetAuditError(f"non-finite numeric xacro property for {context}")
    return result


def _manifest_by_name(raw: Any, expected_names: Sequence[str], context: str) -> Mapping[str, Mapping[str, Any]]:
    if not isinstance(raw, list):
        raise AssetAuditError(f"{context} must be a list")
    records = [_expect_mapping(item, f"{context}[{index}]") for index, item in enumerate(raw)]
    names = [record.get("name", record.get("link")) for record in records]
    if names != list(expected_names):
        raise AssetAuditError(f"{context} order mismatch: {names!r}")
    return {str(name): record for name, record in zip(names, records)}


def _inertia_matrix(element: ET.Element, context: str) -> tuple[tuple[float, ...], ...]:
    try:
        ixx = float(element.attrib["ixx"])
        ixy = float(element.attrib["ixy"])
        ixz = float(element.attrib["ixz"])
        iyy = float(element.attrib["iyy"])
        iyz = float(element.attrib["iyz"])
        izz = float(element.attrib["izz"])
    except (KeyError, ValueError) as exc:
        raise AssetAuditError(f"invalid inertia tensor for {context}") from exc
    return ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))


def _audit_official_macro(model: Mapping[str, Any], macro_path: Path) -> None:
    root = ET.parse(macro_path).getroot()
    macro = root.find(f"{_XACRO}macro")
    if macro is None or macro.attrib.get("name") != "m710id_70":
        raise AssetAuditError("official M-710iD/70 macro declaration is missing")
    params = " ".join(macro.attrib.get("params", "").split())
    if params != "prefix='' parent *origin child":
        raise AssetAuditError("official macro parameter contract mismatch")
    properties = {
        item.attrib["name"]: _xacro_property_number(item.attrib["value"], item.attrib["name"])
        for item in macro.findall(f"{_XACRO}property")
    }

    expected_links = ["base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link"]
    links = {_unprefixed(item.attrib["name"], "official link"): item for item in macro.findall("link")}
    if list(links) != [*expected_links, "wbase", "flange", "fanuc_flange"]:
        raise AssetAuditError("official macro link order or names changed")
    if model.get("base_link") != "base_link" or model.get("moving_links") != expected_links[1:]:
        raise AssetAuditError("official manifest link contract mismatch")
    if model.get("wbase_link") != "wbase" or model.get("flange_link") != "flange":
        raise AssetAuditError("official manifest flange/base frame contract mismatch")
    if model.get("fanuc_flange_link") != "fanuc_flange" or model.get("sample_child_link") != "ee_link":
        raise AssetAuditError("official manifest auxiliary-frame contract mismatch")

    joint_names = [f"J{index}" for index in range(1, 7)]
    manifest_joints = _manifest_by_name(model.get("joints"), joint_names, "official model.joints")
    joints = {
        _unprefixed(item.attrib["name"], "official joint"): item
        for item in macro.findall("joint")
        if item.attrib.get("type") == "revolute"
    }
    if list(joints) != joint_names or model.get("actuated_joint_order") != joint_names:
        raise AssetAuditError("official actuated-joint order mismatch")
    if macro.findall(".//mimic"):
        raise AssetAuditError("official J3 must remain an independent revolute joint without mimic")
    for name in joint_names:
        element = joints[name]
        record = manifest_joints[name]
        expected_keys = {
            "name", "type", "parent", "child", "origin_xyz_m", "origin_rpy_rad", "axis", "limit",
        }
        _expect_exact_keys(record, expected_keys, f"official model.joints.{name}")
        if record["type"] != "revolute" or element.attrib.get("type") != "revolute":
            raise AssetAuditError(f"official joint type mismatch for {name}")
        parent = element.find("parent")
        child = element.find("child")
        if parent is None or child is None:
            raise AssetAuditError(f"official joint link declaration missing for {name}")
        if record["parent"] != _unprefixed(parent.attrib["link"], f"{name}.parent"):
            raise AssetAuditError(f"official parent link mismatch for {name}")
        if record["child"] != _unprefixed(child.attrib["link"], f"{name}.child"):
            raise AssetAuditError(f"official child link mismatch for {name}")
        origin = element.find("origin")
        _assert_vector_close(record["origin_xyz_m"], _xml_vector(origin, "xyz", name), f"{name}.origin_xyz_m")
        _assert_vector_close(record["origin_rpy_rad"], _xml_vector(origin, "rpy", name), f"{name}.origin_rpy_rad")
        _assert_vector_close(record["axis"], _xml_vector(element.find("axis"), "xyz", name), f"{name}.axis")
        limit = _expect_mapping(record["limit"], f"official model.joints.{name}.limit")
        _expect_exact_keys(limit, {"lower_rad", "upper_rad", "velocity_rad_s", "effort_nm"}, f"{name}.limit")
        source_limit = element.find("limit")
        if source_limit is None:
            raise AssetAuditError(f"official joint limit is missing for {name}")
        property_names = {
            "lower_rad": source_limit.attrib["lower"].strip("${}"),
            "upper_rad": source_limit.attrib["upper"].strip("${}"),
            "velocity_rad_s": source_limit.attrib["velocity"].strip("${}"),
            "effort_nm": source_limit.attrib["effort"].strip("${}"),
        }
        for field, property_name in property_names.items():
            actual = properties.get(property_name)
            if actual is None or abs(float(limit[field]) - actual) > 1e-12:
                raise AssetAuditError(f"official {name} {field} mismatch")

    manifest_inertials = _manifest_by_name(model.get("inertials"), expected_links, "official model.inertials")
    total_mass = 0.0
    for link_name in expected_links:
        inertial = links[link_name].find("inertial")
        if inertial is None:
            raise AssetAuditError(f"official inertial missing for {link_name}")
        record = manifest_inertials[link_name]
        _expect_exact_keys(
            record,
            {"link", "mass_kg", "origin_xyz_m", "origin_rpy_rad", "inertia_at_com_kg_m2"},
            f"official model.inertials.{link_name}",
        )
        mass_element = inertial.find("mass")
        origin = inertial.find("origin")
        tensor = inertial.find("inertia")
        if mass_element is None or origin is None or tensor is None:
            raise AssetAuditError(f"official inertial fields missing for {link_name}")
        mass = float(mass_element.attrib["value"])
        if abs(float(record["mass_kg"]) - mass) > 1e-12:
            raise AssetAuditError(f"official mass mismatch for {link_name}")
        total_mass += mass
        _assert_vector_close(record["origin_xyz_m"], _xml_vector(origin, "xyz", link_name), f"{link_name}.origin")
        _assert_vector_close(record["origin_rpy_rad"], (0.0, 0.0, 0.0), f"{link_name}.origin_rpy_rad")
        matrix = record["inertia_at_com_kg_m2"]
        if not isinstance(matrix, list) or len(matrix) != 3:
            raise AssetAuditError(f"official inertia matrix shape mismatch for {link_name}")
        source_matrix = _inertia_matrix(tensor, link_name)
        for row_index in range(3):
            _assert_vector_close(matrix[row_index], source_matrix[row_index], f"{link_name}.inertia[{row_index}]")
    if abs(float(model.get("total_mass_kg", math.nan)) - total_mass) > 1e-12:
        raise AssetAuditError("official total link mass mismatch")

    fixed_names = ["base_link-wbase", "J6-flange", "flange-fanuc_flange"]
    manifest_fixed = _manifest_by_name(model.get("fixed_frames"), fixed_names, "official model.fixed_frames")
    fixed = {
        _unprefixed(item.attrib["name"], "official fixed joint"): item
        for item in macro.findall("joint")
        if item.attrib.get("type") == "fixed" and item.attrib.get("name") not in {"${prefix}base_joint", "${prefix}child_joint"}
    }
    if list(fixed) != fixed_names:
        raise AssetAuditError("official internal fixed-frame set mismatch")
    for name in fixed_names:
        element = fixed[name]
        record = manifest_fixed[name]
        _expect_exact_keys(
            record,
            {"name", "parent", "child", "origin_xyz_m", "origin_rpy_rad"},
            f"official model.fixed_frames.{name}",
        )
        parent = element.find("parent")
        child = element.find("child")
        origin = element.find("origin")
        if parent is None or child is None or origin is None:
            raise AssetAuditError(f"official fixed frame is incomplete: {name}")
        if record["parent"] != _unprefixed(parent.attrib["link"], f"{name}.parent"):
            raise AssetAuditError(f"official fixed-frame parent mismatch for {name}")
        if record["child"] != _unprefixed(child.attrib["link"], f"{name}.child"):
            raise AssetAuditError(f"official fixed-frame child mismatch for {name}")
        _assert_vector_close(record["origin_xyz_m"], _xml_vector(origin, "xyz", name), f"{name}.origin_xyz_m")
        raw_rpy = origin.attrib.get("rpy", "")
        if name == "flange-fanuc_flange":
            matches = list(_RADIANS_TOKEN.finditer(raw_rpy))
            residue = _RADIANS_TOKEN.sub("", raw_rpy)
            if len(matches) != 3 or residue.strip():
                raise AssetAuditError("official FANUC flange rpy expression mismatch")
            source_rpy = tuple(math.radians(float(match.group(1))) for match in matches)
        else:
            source_rpy = _xml_vector(origin, "rpy", name)
        _assert_vector_close(record["origin_rpy_rad"], source_rpy, f"{name}.origin_rpy_rad")


def _audit_integration_wrapper(integration: Mapping[str, Any], root: Path) -> Path:
    wrapper = _audit_reference(root, integration.get("standalone_xacro"), "official integration.standalone_xacro")
    wrapper_root = ET.parse(wrapper).getroot()
    if wrapper_root.tag != "robot" or wrapper_root.attrib.get("name") != "fanuc_m710id_70":
        raise AssetAuditError("official standalone wrapper robot identity mismatch")
    include = wrapper_root.find(f"{_XACRO}include")
    if include is None or include.attrib.get("filename") != "../urdf/m710id_70_urdf_macro.xacro":
        raise AssetAuditError("official standalone wrapper must use the local pinned macro")
    invocation = wrapper_root.find(f"{_XACRO}m710id_70")
    if invocation is None or invocation.attrib != {"parent": "world", "child": "ee_link"}:
        raise AssetAuditError("official standalone wrapper invocation mismatch")
    invocation_origin = invocation.find("origin")
    if _xml_vector(invocation_origin, "xyz", "wrapper origin") != (0.0, 0.0, 0.0):
        raise AssetAuditError("official standalone wrapper base translation must remain zero")
    if _xml_vector(invocation_origin, "rpy", "wrapper origin") != (0.0, 0.0, 0.0):
        raise AssetAuditError("official standalone wrapper base rotation must remain zero")
    direct_links = [element.attrib.get("name") for element in wrapper_root.findall("link")]
    if direct_links != ["world", "ee_link", "tool0"]:
        raise AssetAuditError("official standalone wrapper direct-link contract mismatch")
    tool_joint = wrapper_root.find("joint[@name='flange-tool0']")
    if tool_joint is None or tool_joint.attrib.get("type") != "fixed":
        raise AssetAuditError("planner tool0 fixed adapter is missing")
    if tool_joint.find("parent").attrib.get("link") != "flange" or tool_joint.find("child").attrib.get("link") != "tool0":
        raise AssetAuditError("planner tool0 fixed adapter link mapping mismatch")
    tool_origin = tool_joint.find("origin")
    if _xml_vector(tool_origin, "xyz", "flange-tool0") != (0.0, 0.0, 0.0):
        raise AssetAuditError("planner tool0 translation mismatch")
    _assert_vector_close(
        _xml_vector(tool_origin, "rpy", "flange-tool0"),
        (0.0, math.pi / 2.0, 0.0),
        "planner tool0 rotation",
    )
    return wrapper


def _audit_static_official_urdf(model: Mapping[str, Any], integration: Mapping[str, Any], root: Path) -> Path:
    urdf = _audit_reference(root, integration.get("expanded_urdf"), "official integration.expanded_urdf")
    data = urdf.read_bytes()
    if b"xacro:" in data or b"${" in data or b"$(" in data:
        raise AssetAuditError("static official URDF contains unresolved xacro syntax")
    robot = ET.fromstring(data)
    if robot.tag != "robot" or robot.attrib.get("name") != "fanuc_m710id_70":
        raise AssetAuditError("static official URDF identity mismatch")
    link_names = [link.attrib["name"] for link in robot.findall("link")]
    expected_links = [
        "world", "ee_link", "tool0", "base_link", "J1_link", "J2_link", "J3_link", "J4_link",
        "J5_link", "J6_link", "wbase", "flange", "fanuc_flange",
    ]
    if set(link_names) != set(expected_links) or len(link_names) != len(expected_links):
        raise AssetAuditError("static official URDF link set mismatch")
    if robot.findall(".//mimic"):
        raise AssetAuditError("static official J3 must not acquire a mimic relation")
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    joint_names = [f"J{index}" for index in range(1, 7)]
    manifest_joints = _manifest_by_name(model.get("joints"), joint_names, "official model.joints")
    for name in joint_names:
        element = joints.get(name)
        if element is None or element.attrib.get("type") != "revolute":
            raise AssetAuditError(f"static official revolute joint missing: {name}")
        record = manifest_joints[name]
        limit = element.find("limit")
        if limit is None:
            raise AssetAuditError(f"static official limit missing: {name}")
        expected_limit = _expect_mapping(record["limit"], f"official model.joints.{name}.limit")
        comparisons = {
            "lower": "lower_rad", "upper": "upper_rad", "velocity": "velocity_rad_s", "effort": "effort_nm",
        }
        for xml_name, manifest_name in comparisons.items():
            if abs(float(limit.attrib[xml_name]) - float(expected_limit[manifest_name])) > 1e-12:
                raise AssetAuditError(f"static official {name} {manifest_name} mismatch")
        _assert_vector_close(record["origin_xyz_m"], _xml_vector(element.find("origin"), "xyz", name), f"static {name}.origin")
        _assert_vector_close(record["axis"], _xml_vector(element.find("axis"), "xyz", name), f"static {name}.axis")
    manifest_inertials = _manifest_by_name(
        model.get("inertials"),
        ["base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link"],
        "official model.inertials",
    )
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    for link_name, record in manifest_inertials.items():
        inertial = links[link_name].find("inertial")
        if inertial is None:
            raise AssetAuditError(f"static official inertial missing: {link_name}")
        mass = inertial.find("mass")
        origin = inertial.find("origin")
        tensor = inertial.find("inertia")
        if mass is None or origin is None or tensor is None:
            raise AssetAuditError(f"static official inertial fields missing: {link_name}")
        if abs(float(mass.attrib["value"]) - float(record["mass_kg"])) > 1e-12:
            raise AssetAuditError(f"static official mass mismatch: {link_name}")
        _assert_vector_close(
            _xml_vector(origin, "xyz", f"static {link_name}.inertial"),
            record["origin_xyz_m"],
            f"static {link_name}.inertial origin",
        )
        source_matrix = _inertia_matrix(tensor, f"static {link_name}")
        for row_index in range(3):
            _assert_vector_close(
                source_matrix[row_index],
                record["inertia_at_com_kg_m2"][row_index],
                f"static {link_name}.inertia[{row_index}]",
            )
    expected_fixed = {
        "base_link-wbase": ("base_link", "wbase", (0.0, 0.0, 0.565), (0.0, 0.0, 0.0)),
        "J6-flange": ("J6_link", "flange", (0.175, 0.0, 0.0), (0.0, 0.0, 0.0)),
        "flange-fanuc_flange": (
            "flange", "fanuc_flange", (0.0, 0.0, 0.0), (math.pi, -math.pi / 2.0, 0.0),
        ),
        "base_joint": ("world", "base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        "child_joint": ("flange", "ee_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        "flange-tool0": ("flange", "tool0", (0.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0)),
    }
    for name, (parent, child, xyz, rpy) in expected_fixed.items():
        joint = joints.get(name)
        if joint is None or joint.attrib.get("type") != "fixed":
            raise AssetAuditError(f"static official fixed joint missing: {name}")
        if joint.find("parent").attrib.get("link") != parent or joint.find("child").attrib.get("link") != child:
            raise AssetAuditError(f"static official fixed joint link mismatch: {name}")
        _assert_vector_close(_xml_vector(joint.find("origin"), "xyz", name), xyz, f"static {name}.xyz")
        _assert_vector_close(_xml_vector(joint.find("origin"), "rpy", name), rpy, f"static {name}.rpy")
    mesh_uris = [mesh.attrib.get("filename") for mesh in robot.findall(".//mesh")]
    expected_uris = {
        f"package://fanuc_m710_description/meshes/m710id_70/{kind}/{link}.{suffix}"
        for kind, suffix in (("visual", "dae"), ("collision", "stl"))
        for link in ("base", "j1", "j2", "j3", "j4", "j5", "j6")
    }
    if set(mesh_uris) != expected_uris or len(mesh_uris) != 14:
        raise AssetAuditError("static official URDF mesh references changed or use a proxy")
    return urdf


def _binary_stl_bounds(path: Path) -> tuple[tuple[float, ...], tuple[float, ...], int]:
    data = path.read_bytes()
    if len(data) < 84:
        raise AssetAuditError(f"truncated official collision STL: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    for triangle_index in range(triangle_count):
        values = struct.unpack_from("<12f", data, 84 + 50 * triangle_index)
        for vertex_index in range(3):
            vertex = values[3 + 3 * vertex_index : 6 + 3 * vertex_index]
            for axis, value in enumerate(vertex):
                minimum[axis] = min(minimum[axis], value)
                maximum[axis] = max(maximum[axis], value)
    return tuple(minimum), tuple(maximum), triangle_count


def audit_m710id70_official_model(
    repository_root: str | Path,
    manifest_path: str | Path | None = None,
) -> OfficialModelAuditReport:
    """Audit pinned official source, meshes, dynamics and deterministic URDF.

    ``execution_qualified`` is scoped to the robot-description asset only.  It
    does not qualify a gripper, path, workcell, controller, or physics replay.
    """

    root = Path(repository_root).resolve()
    if manifest_path is None:
        path = root / Path(*(_OFFICIAL_ROOT / "provenance.yaml").parts)
    else:
        candidate = Path(manifest_path)
        path = (candidate if candidate.is_absolute() else root / candidate).resolve()
    try:
        repository_manifest_path = path.relative_to(root).as_posix()
    except ValueError as exc:
        raise AssetAuditError(f"official model manifest must remain within the repository: {path}") from exc
    document = _load_yaml(path)
    _expect_exact_keys(
        document,
        {
            "schema_version", "asset_id", "manufacturer", "model_name", "upstream",
            "source_file_total_count", "source_file_total_bytes", "source_files", "geometry",
            "model", "integration", "qualification",
        },
        "official model manifest",
    )
    if document["schema_version"] != OFFICIAL_M710ID70_SCHEMA:
        raise AssetAuditError(f"unsupported official model manifest schema: {document['schema_version']!r}")
    if document["manufacturer"] != "FANUC" or document["model_name"] != "M-710iD/70":
        raise AssetAuditError("official model manufacturer or name mismatch")
    upstream = _expect_mapping(document["upstream"], "official upstream")
    _expect_exact_keys(
        upstream,
        {"repository_url", "commit", "tag", "package_name", "package_version", "license_spdx", "license_path"},
        "official upstream",
    )
    if upstream != {
        "repository_url": OFFICIAL_M710ID70_REPOSITORY,
        "commit": OFFICIAL_M710ID70_COMMIT,
        "tag": "v2.3.0",
        "package_name": "fanuc_m710_description",
        "package_version": OFFICIAL_M710ID70_PACKAGE_VERSION,
        "license_spdx": "Apache-2.0",
        "license_path": "assets/robots/fanuc_m710id_70/official/LICENSES/Apache-2.0.txt",
    }:
        raise AssetAuditError("official upstream provenance is not the pinned FANUC release")
    sources, total_bytes = _audit_official_source_files(document, root)
    package = ET.parse(sources["fanuc_m710_description/package.xml"]).getroot()
    package_name = package.findtext("name")
    package_version = package.findtext("version")
    package_license = package.findtext("license")
    if (package_name, package_version, package_license) != (
        "fanuc_m710_description", OFFICIAL_M710ID70_PACKAGE_VERSION, "Apache-2.0",
    ):
        raise AssetAuditError("official package identity/version/license mismatch")

    geometry = _expect_mapping(document["geometry"], "official geometry")
    _expect_exact_keys(
        geometry,
        {
            "visual_meshes", "collision_meshes", "dae_unit_m", "dae_up_axis",
            "base_collision_bounds_m", "collision_triangle_count_total", "collision_topology",
            "collision_policy",
        },
        "official geometry",
    )
    expected_links = ["base_link", "J1_link", "J2_link", "J3_link", "J4_link", "J5_link", "J6_link"]
    visual = _manifest_by_name(geometry["visual_meshes"], expected_links, "official visual_meshes")
    collision = _manifest_by_name(geometry["collision_meshes"], expected_links, "official collision_meshes")
    for kind, suffix, records in (("visual", "dae", visual), ("collision", "stl", collision)):
        for link_name, record in records.items():
            _expect_exact_keys(record, {"link", "path"}, f"official {kind} mesh {link_name}")
            stem = "base" if link_name == "base_link" else link_name[:-5].lower()
            expected_path = (
                _OFFICIAL_ROOT
                / "fanuc_m710_description"
                / "meshes"
                / "m710id_70"
                / kind
                / f"{stem}.{suffix}"
            ).as_posix()
            if record["path"] != expected_path:
                raise AssetAuditError(f"official {kind} mesh mapping mismatch for {link_name}")
    if geometry["dae_unit_m"] != 1.0 or geometry["dae_up_axis"] != "Z_UP":
        raise AssetAuditError("official visual-mesh unit or up-axis mismatch")
    bounds = _expect_mapping(geometry["base_collision_bounds_m"], "official base collision bounds")
    _expect_exact_keys(bounds, {"min_xyz_m", "max_xyz_m"}, "official base collision bounds")
    base_collision = root / collision["base_link"]["path"]
    actual_minimum, actual_maximum, _ = _binary_stl_bounds(base_collision)
    declared_minimum = _float_vector(bounds["min_xyz_m"], 3, "official base minimum")
    declared_maximum = _float_vector(bounds["max_xyz_m"], 3, "official base maximum")
    if any(abs(a - b) > 1e-6 for a, b in zip(actual_minimum, declared_minimum)):
        raise AssetAuditError("official base collision minimum bound mismatch")
    if any(abs(a - b) > 1e-6 for a, b in zip(actual_maximum, declared_maximum)):
        raise AssetAuditError("official base collision maximum bound mismatch")
    collision_triangles = sum(
        _binary_stl_bounds(root / record["path"])[2] for record in collision.values()
    )
    if geometry["collision_triangle_count_total"] != collision_triangles or collision_triangles != 17_634:
        raise AssetAuditError("official collision-mesh triangle total mismatch")
    if geometry["collision_topology"] != "manufacturer_per_link_triangle_mesh_not_asserted_convex":
        raise AssetAuditError("official collision topology must not be misrepresented as convex")
    if geometry["collision_policy"] != "official_per_link_meshes_no_proxy_substitution":
        raise AssetAuditError("official collision policy must forbid proxy substitution")

    model = _expect_mapping(document["model"], "official model")
    _expect_exact_keys(
        model,
        {
            "base_link", "wbase_link", "flange_link", "fanuc_flange_link", "sample_child_link",
            "task_tool_link", "task_tool_frame", "moving_links", "massless_fixed_links", "actuated_joint_order", "joints",
            "fixed_frames", "inertials", "total_mass_kg", "zero_pose",
        },
        "official model",
    )
    if model["task_tool_link"] != "tool0":
        raise AssetAuditError("planner task tool link must remain explicit and separate from fanuc_flange")
    task_tool_frame = _expect_mapping(model["task_tool_frame"], "official model.task_tool_frame")
    if task_tool_frame != {
        "name": "flange-tool0",
        "parent": "flange",
        "child": "tool0",
        "origin_xyz_m": [0.0, 0.0, 0.0],
        "origin_rpy_rad": [0.0, math.pi / 2.0, 0.0],
        "source": "project_integration_wrapper",
    }:
        raise AssetAuditError("planner task tool frame contract mismatch")
    if model["massless_fixed_links"] != ["wbase", "flange", "fanuc_flange", "ee_link", "tool0"]:
        raise AssetAuditError("official massless fixed-link policy mismatch")
    zero_pose = _expect_mapping(model["zero_pose"], "official model.zero_pose")
    _expect_exact_keys(
        zero_pose,
        {"flange_xyz_m", "flange_rpy_rad", "fanuc_flange_xyz_m", "fanuc_flange_rpy_rad", "tool0_xyz_m", "tool0_rpy_rad"},
        "official model.zero_pose",
    )
    _assert_vector_close(zero_pose["flange_xyz_m"], (1.370, 0.0, 1.630), "zero flange xyz")
    _assert_vector_close(zero_pose["flange_rpy_rad"], (0.0, 0.0, 0.0), "zero flange rpy")
    _assert_vector_close(zero_pose["fanuc_flange_xyz_m"], (1.370, 0.0, 1.630), "zero FANUC flange xyz")
    _assert_vector_close(
        zero_pose["fanuc_flange_rpy_rad"], (math.pi, -math.pi / 2.0, 0.0), "zero FANUC flange rpy",
    )
    _assert_vector_close(zero_pose["tool0_xyz_m"], (1.370, 0.0, 1.630), "zero tool0 xyz")
    _assert_vector_close(zero_pose["tool0_rpy_rad"], (0.0, math.pi / 2.0, 0.0), "zero tool0 rpy")
    _audit_official_macro(model, sources["fanuc_m710_description/urdf/m710id_70_urdf_macro.xacro"])

    integration = _expect_mapping(document["integration"], "official integration")
    _expect_exact_keys(
        integration,
        {"official_sample_xacro", "standalone_xacro", "expanded_urdf", "expansion_script", "generator"},
        "official integration",
    )
    if integration["official_sample_xacro"] != (
        "assets/robots/fanuc_m710id_70/official/fanuc_m710_description/robot/m710id_70.urdf.xacro"
    ):
        raise AssetAuditError("official sample xacro path mismatch")
    _audit_integration_wrapper(integration, root)
    _audit_reference(root, integration["expansion_script"], "official integration.expansion_script")
    generator = _expect_mapping(integration["generator"], "official integration.generator")
    _expect_exact_keys(generator, {"name", "version", "canonicalization"}, "official integration.generator")
    if generator["name"] != "xacro" or generator["canonicalization"] != (
        "xml.etree.ElementTree.canonicalize(strip_text=true,with_comments=false)"
    ):
        raise AssetAuditError("official static-URDF generator contract mismatch")
    if not isinstance(generator["version"], str) or not generator["version"]:
        raise AssetAuditError("official xacro generator version must be recorded")
    _audit_static_official_urdf(model, integration, root)

    qualification = _expect_mapping(document["qualification"], "official qualification")
    _expect_exact_keys(
        qualification,
        {"scope", "execution_qualified", "system_execution_qualified", "required_checks", "policy"},
        "official qualification",
    )
    required_checks = {
        "pinned_source_hashes", "package_identity", "unmodified_official_macro", "seven_visual_meshes",
        "seven_collision_meshes", "joint_and_frame_semantics", "complete_official_inertials",
        "deterministic_static_urdf", "no_proxy_collision",
    }
    if set(qualification["required_checks"]) != required_checks:
        raise AssetAuditError("official qualification check set mismatch")
    if qualification["scope"] != "robot_description_asset_only":
        raise AssetAuditError("official qualification scope must remain asset-only")
    if qualification["execution_qualified"] is not True or qualification["system_execution_qualified"] is not False:
        raise AssetAuditError("official asset/system qualification flags are inconsistent")
    if qualification["policy"] != "robot_asset_qualification_does_not_qualify_tool_path_workcell_or_replay":
        raise AssetAuditError("official qualification policy mismatch")
    return OfficialModelAuditReport(
        manifest_path=repository_manifest_path,
        manifest_sha256=_sha256(path.read_bytes()),
        upstream_commit=OFFICIAL_M710ID70_COMMIT,
        package_version=OFFICIAL_M710ID70_PACKAGE_VERSION,
        source_integrity=True,
        static_urdf_integrity=True,
        model_semantics=True,
        verified_source_file_count=len(sources),
        verified_source_bytes=total_bytes,
        visual_mesh_count=len(visual),
        collision_mesh_count=len(collision),
        execution_qualified=True,
    )
