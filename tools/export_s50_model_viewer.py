"""Embed real CAD and the independently frozen environment in an offline viewer.

CAD vertices already use metres in the CAD assembly frame. No inferred mounting
transform, joint articulation, initial state or successful path is introduced.
"""
from __future__ import annotations

import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path
import sys
import zlib

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.robot_replacement_layout import (
    build_replacement_layout, compare_non_robot_scenes, freeze_non_robot_scene,
    frozen_baseline_layout,
)


def _read_cad_bytes(path):
    """Allow lossless repository storage without changing the CAD identity."""
    path = Path(path)
    if path.is_file():
        return path.read_bytes()
    compressed_path = path.with_name(path.name + ".gz")
    if compressed_path.is_file():
        try:
            return gzip.decompress(compressed_path.read_bytes())
        except (OSError, EOFError, zlib.error) as exc:
            raise ValueError("CAD_MESH_GZIP_INVALID:" + str(compressed_path)) from exc
    raise FileNotFoundError("CAD_MESH_FILE_MISSING:" + str(path))


def read_cad_obj(path, *, raw_bytes=None):
    """Read the triangulated CAD export without re-applying instance placement."""
    vertices, triangles = [], []
    content = _read_cad_bytes(path) if raw_bytes is None else raw_bytes
    for line in content.decode("utf-8").splitlines():
        values = line.split()
        if not values:
            continue
        if values[0] == "v":
            if len(values) != 4:
                raise ValueError("CAD_OBJ_REQUIRES_XYZ_VERTICES")
            vertices.append([float(v) for v in values[1:]])
        elif values[0] == "f":
            if len(values) != 4:
                raise ValueError("CAD_OBJ_REQUIRES_TRIANGULATED_FACES")
            indices = [int(v.split("/")[0]) for v in values[1:]]
            triangles.append([v-1 if v > 0 else len(vertices)+v for v in indices])
    xyz = np.asarray(vertices, dtype="<f4")
    indices = np.asarray(triangles, dtype=np.int64)
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all()
            or indices.ndim != 2 or indices.shape[1] != 3 or not len(indices)
            or np.min(indices) < 0 or np.max(indices) >= len(xyz)):
        raise ValueError("INVALID_CAD_OBJ_MESH")
    return xyz, indices.astype("<u4")


def export_cad(audit_path, root=ROOT):
    audit_path, root = Path(audit_path), Path(root).resolve()
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    if audit["normalization_scale_to_m"] != .001 or audit["source_units"] != "millimetres":
        raise ValueError("UNEXPECTED_S50_CAD_UNIT_CONTRACT")
    parts = []
    ids = set()
    for part in audit["parts"]:
        path = (root / part["mesh"]).resolve()
        if not path.is_relative_to(root):
            raise ValueError("CAD_MESH_OUTSIDE_REPOSITORY")
        if part["id"] in ids:
            raise ValueError("CAD_PART_ID_NOT_UNIQUE")
        ids.add(part["id"])
        raw_mesh = _read_cad_bytes(path)
        if hashlib.sha256(raw_mesh).hexdigest() != part["mesh_sha256"]:
            raise ValueError("CAD_MESH_SHA256_MISMATCH:" + part["id"])
        xyz, triangles = read_cad_obj(path, raw_bytes=raw_mesh)
        if len(xyz) != part["mesh_vertices"] or len(triangles) != part["mesh_triangles"]:
            raise ValueError("CAD_MESH_COUNT_MISMATCH:" + part["id"])
        bounds = np.asarray([xyz.min(axis=0), xyz.max(axis=0)])
        if not np.allclose(bounds, part["mesh_bounds_m"], atol=2e-6, rtol=0):
            raise ValueError("CAD_MESH_BOUNDS_MISMATCH:" + part["id"])
        fields = ("id", "name", "mesh", "mesh_sha256", "mesh_vertices", "mesh_triangles", "geometry_bounds_m",
                  "mesh_bounds_m", "topology_valid", "mesh_complete", "missing_tessellation_faces")
        parts.append({**{key: part[key] for key in fields},
            "positions_b64": base64.b64encode(xyz.tobytes()).decode("ascii"),
            "indices_b64": base64.b64encode(triangles.tobytes()).decode("ascii")})
    if len(parts) != 7:
        raise ValueError("S50_CAD_EXPECTS_SEVEN_DISTINCT_ASSEMBLY_PARTS")
    return dict(source=audit["source"], source_sha256=audit["source_sha256"],
        audit_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest(),
        coordinate_semantics=audit["coordinate_semantics"], units="metres", cad_vertical_axis="+Y",
        instance_placements_applied_again=False, parts=parts, limitations=audit["limitations"],
        triangle_count=sum(p["mesh_triangles"] for p in parts),
        display_only=True, articulated_model_ready=False, mounting_alignment_verified=False)


def build_data(audit_path=None, root=ROOT):
    root = Path(root).resolve()
    before = freeze_non_robot_scene(frozen_baseline_layout(root))
    replacement = build_replacement_layout(root=root)
    frozen = freeze_non_robot_scene(replacement)
    # Compare shared geometry first, then show the S50 carton mass.
    difference = compare_non_robot_scenes(before, frozen)
    from unloading_sim.s50_payload import apply_s50_carton_mass
    frozen = apply_s50_carton_mass(frozen, root)
    tool = frozen["tool_intrinsic"]
    frames = tool["intrinsic_frames"]
    target = np.asarray(frozen["initial_physical_contact_target"])
    world_from_virtual = (target @ np.linalg.inv(frames["T_tool_mount_reference_nominal_compressed_contact"])
                          @ frames["T_tool_mount_reference_virtual_task_tcp"])
    cup_boxes = np.asarray(tool["compliant_local_boxes_virtual_tcp_m"])
    plane_size = ((cup_boxes[:, :2] + cup_boxes[:, 3:5]/2).max(axis=0)
                  - (cup_boxes[:, :2] - cup_boxes[:, 3:5]/2).min(axis=0))
    cad = export_cad(audit_path or root / "assets/robots/huayan_s50_2000/derived/cad_audit.json", root)
    return dict(schema="s50_offline_cad_layout_display_v1", model_status="MODEL_NOT_READY",
        replacement_complete=False, full_scene_fingerprint=None, articulated_model=None,
        joint_controls_enabled=False,
        joint_controls_disabled_reason="缺少已核验的安装朝向、关节零位/正负号、限位及法兰定义；CAD 装配不等于可运动模型。",
        trajectory=[], trajectory_status="NOT_RUN", initial_state_status="NOT_SOLVED",
        environment=frozen, layout_difference=difference, cad=cad,
        tool_reference=dict(world_from_virtual_task_tcp=world_from_virtual.tolist(),
            contact_plane_size_m=plane_size.tolist(),
            status="REQUESTED_CONTACT_POSE_REFERENCE_ONLY_NOT_ROBOT_FK", collision_checked=False),
        display_contract=dict(cad_and_environment_views_separate=True, cad_is_not_planning_initial_state=True,
            tool_target_is_not_accepted_initial_state=True, no_fanuc_joint_values_reused=True,
            physical_execution="NOT_RUN", collision_qualification="NOT_EVALUATED"))


def serialize_payload(data):
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")


def encode_payload(data):
    """Losslessly compress JSON with no timestamp or executable data content."""
    return base64.b64encode(gzip.compress(serialize_payload(data), compresslevel=9, mtime=0)).decode("ascii")


def write_viewer(output, data, root=ROOT):
    root = Path(root)
    vendor = root / "tools/search_replay/vendor"
    # Preserve the existing offline libraries and their license in the artifact.
    license_text = (vendor / "LICENSE.three").read_text(encoding="utf-8").replace("*/", "* /")
    scripts = "/*\n" + license_text + "\n*/\n" + "\n".join(
        (vendor / name).read_text(encoding="utf-8") for name in ("three.min.js", "OrbitControls.js"))
    template = (root / "tools/s50_model_viewer.html").read_text(encoding="utf-8")
    text = template.replace("/*__VENDOR__*/", scripts).replace("/*__DATA__*/",
        json.dumps(encode_payload(data)))
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
        default=ROOT / "docs/experiments/s50_model_integration_20261110/viewer.html")
    parser.add_argument("--cad-audit", type=Path)
    args = parser.parse_args(argv)
    data = build_data(args.cad_audit)
    write_viewer(args.output, data)
    checks = dict(model_status=data["model_status"], layout_difference=data["layout_difference"],
        cad_source_sha256=data["cad"]["source_sha256"], audit_sha256=data["cad"]["audit_sha256"],
        cad_triangles=data["cad"]["triangle_count"], cad_parts=len(data["cad"]["parts"]),
        joint_controls_enabled=False, trajectory_status="NOT_RUN", display_contract=data["display_contract"],
        offline_payload=dict(format="gzip_base64_json_utf8", gzip_mtime=0,
            uncompressed_sha256=hashlib.sha256(serialize_payload(data)).hexdigest(),
            uncompressed_bytes=len(serialize_payload(data)), lossless=True,
            required_browser_capabilities=["DecompressionStream(gzip)", "Blob.stream", "Response.text", "WebGL"],
            unsupported_browser_behavior="VISIBLE_LOAD_ERROR_NO_MODEL_OR_TRAJECTORY"))
    args.output.with_name(args.output.stem + "-checks.json").write_text(
        json.dumps(checks, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(dict(html=str(args.output), bytes=args.output.stat().st_size, **checks), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
