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
import struct
from typing import Any, Mapping, Sequence
import zipfile

import yaml


ROBOT_SCHEMA = "m710id70_cad_asset_manifest_v1"
TOOL_SCHEMA = "shanghai_wantai_three_zone_asset_manifest_v1"
LINK_MAPPING_SCHEMA = "m710id70_link_mapping_v1"


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
    if document["execution_qualified"] is not False:
        raise AssetAuditError("tool execution_qualified must remain false for the current engineering assets")
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
    except ValueError as exc:
        raise AssetAuditError(f"asset manifest must remain within the repository: {path}") from exc
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
    return AssetAuditReport(
        manifest_path=repository_manifest_path,
        schema_version=str(schema),
        manifest_sha256=_sha256(path.read_bytes()),
        source_integrity=True,
        conversion_status=str(document["conversion_status"]),
        execution_qualified=bool(document["execution_qualified"]),
        verified_file_count=count,
        verified_source_bytes=total_bytes,
        missing_converted_outputs=missing,
        unresolved=tuple(str(item) for item in document["unresolved"]),
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
