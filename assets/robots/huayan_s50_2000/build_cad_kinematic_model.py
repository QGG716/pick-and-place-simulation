"""Generate the actual S50 measured 6R CAD-preview model and check its FK.

This creates no operational/controller model and adds no tool or virtual TCP.
All positions use metres; q0 is the user-authorized supplied CAD assembly.
"""
from __future__ import annotations

import argparse
import gzip
import io
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.s50_cad_model import (
    ASSET_DIRECTORY, DEFINITION_PATH, PREVIEW_CONFIG, MODEL_ID, MODEL_USAGE,
    build_definition, read_obj, sha256, transform_vertices, validate_preview_model,
)


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def _source_record(path):
    raw = path.read_bytes()
    record = dict(sha256=sha256(raw))
    if path.suffix == ".json":
        canonical = json.dumps(json.loads(raw.decode("utf-8")), sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
        record["canonical_json_sha256"] = sha256(canonical)
        record["text_checkout_policy"] = "Accept unchanged JSON values across Git newline conversion; retain observed byte hash separately."
    return record


def _gzip(content):
    target = io.BytesIO()
    with gzip.GzipFile(filename="", mode="wb", fileobj=target, compresslevel=9, mtime=0) as stream:
        stream.write(content)
    return target.getvalue()


def _obj_bytes(vertices, faces):
    lines = ["# Link-local S50 CAD geometry; metres; deterministic float64 transform\n"]
    lines.extend("v " + " ".join(format(float(v), ".17g") for v in point) + "\n" for point in vertices)
    lines.extend("f " + " ".join(str(int(v)+1) for v in face) + "\n" for face in faces)
    return "".join(lines).encode("utf-8")


def _export_local_mesh(root, mesh, target_stem, CAD_from_link, asset_directory=ASSET_DIRECTORY):
    vertices, triangles = read_obj(root / mesh["path"], mesh["sha256"])
    local = transform_vertices(np.linalg.inv(CAD_from_link), vertices)
    content = _obj_bytes(local, triangles)
    relative = f"{asset_directory}/meshes/link-local/{target_stem}.obj"
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    packed = _gzip(content)
    path.with_name(path.name + ".gz").write_bytes(packed)
    return dict(path=relative, gzip_path=relative+".gz", sha256=sha256(content), gzip_sha256=sha256(packed),
        vertices=len(local), triangles=len(triangles), local_bounds_m=[local.min(axis=0).tolist(), local.max(axis=0).tolist()],
        coordinate_frame="LINK_LOCAL_METRES", source_CAD_mesh_path=mesh["path"], source_CAD_mesh_sha256=mesh["sha256"])


def _vector(values):
    return " ".join(format(float(value), ".17g") for value in values)


def _rpy(rotation):
    rotation = np.asarray(rotation)
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(np.cos(pitch)) > 1e-10:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.
        yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
    return [roll, pitch, yaw]


def _add_origin(element, transform):
    transform = np.asarray(transform)
    ET.SubElement(element, "origin", xyz=_vector(transform[:3, 3]), rpy=_vector(_rpy(transform[:3, :3])))


def write_urdf(root, definition, asset_directory=ASSET_DIRECTORY):
    path = root / asset_directory / "urdf/s50_cad_preview.urdf"
    robot = ET.Element("robot", name=MODEL_ID)
    robot.append(ET.Comment("CAD preview only: q0 is supplied CAD assembly; right-hand simulation axes; no controller mapping, hard limits, dynamics or tool qualification."))
    for link in definition["links"]:
        element = ET.SubElement(robot, "link", name=link["name"])
        visual = ET.SubElement(element, "visual", name=link["cad_body"]+"_source_visual")
        ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename="../meshes/link-local/"+Path(link["visual_mesh"]["path"]).name, scale="1 1 1")
        for index, collider in enumerate(link["collision_meshes"]):
            collision = ET.SubElement(element, "collision", name=link["cad_body"]+f"_collision_{index}")
            ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(geometry, "mesh", filename="../meshes/link-local/"+Path(collider["path"]).name, scale="1 1 1")
    ET.SubElement(robot, "link", name=definition["flange_link"])
    for item in [*definition["joints"], *definition["fixed_joints"]]:
        joint = ET.SubElement(robot, "joint", name=item["name"], type=item["type"])
        ET.SubElement(joint, "parent", link=item["parent_link"])
        ET.SubElement(joint, "child", link=item["child_link"])
        _add_origin(joint, item["origin"])
        if item["type"] == "revolute":
            ET.SubElement(joint, "axis", xyz=_vector(item["axis"]))
            joint.append(ET.Comment("These finite ranges, velocity and zero effort are preview/parser fields, NOT physical limits or dynamics parameters."))
            ET.SubElement(joint, "limit", lower=format(-float(np.pi), ".17g"), upper=format(float(np.pi), ".17g"), velocity="1", effort="0")
    ET.indent(robot)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(ET.tostring(robot, encoding="utf-8", xml_declaration=True)+b"\n")
    srdf = ET.Element("robot", name=MODEL_ID)
    group = ET.SubElement(srdf, "group", name="manipulator")
    ET.SubElement(group, "chain", base_link=definition["base_link"], tip_link=definition["tip_link"])
    srdf.append(ET.Comment("No broad self-collision exclusions inferred from display motion or missing controller metadata."))
    ET.indent(srdf)
    srdf_path = path.with_suffix(".srdf")
    srdf_path.write_bytes(ET.tostring(srdf, encoding="utf-8", xml_declaration=True)+b"\n")
    return path, srdf_path


def build_model(root=ROOT, repair_report=None):
    root = Path(root).resolve()
    asset = root / ASSET_DIRECTORY
    measurements_path, audit_path = asset / "derived/interface_measurements.json", asset / "derived/cad_audit.json"
    measurements = json.loads(measurements_path.read_text(encoding="utf-8"))
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    definition = build_definition(measurements, audit)
    definition["source_files"] = {str(path.relative_to(root)).replace("\\", "/"): _source_record(path)
        for path in (measurements_path, audit_path, asset / "source/S50.STEP")}
    parts = {part["id"]: part for part in audit["parts"]}
    repair_path = Path(repair_report) if repair_report else asset / "collision_repair/coverage_report.json"
    repair = None
    if repair_path.is_file():
        repair = json.loads(repair_path.read_text(encoding="utf-8"))
        if repair["source_sha256"] != audit["source_sha256"]:
            raise ValueError("S50_REPAIR_PATCH_SOURCE_IDENTITY_MISMATCH")
        identities = [(patch["cad_body"], patch["source_face_index"]) for patch in repair["patches"]]
        if len(set(identities)) != len(identities) or any(body not in parts for body, _ in identities):
            raise ValueError("S50_REPAIR_PATCH_OWNERSHIP_INVALID")
        definition["source_files"][repair_path.relative_to(root).as_posix()] = _source_record(repair_path)
        definition["collision_repair"] = dict(path=repair_path.relative_to(root).as_posix(),
            report_sha256=sha256(repair_path.read_bytes()), qualification=repair.get("qualification"),
            local_patch_count=len(repair["patches"]), whole_robot_coverage_qualified=False,
            interpretation="Repair report claims retained; attaching its meshes alone does not certify collision clearance.")
    for link in definition["links"]:
        body = parts[link["cad_body"]]
        visual = _export_local_mesh(root, dict(path=body["mesh"], sha256=body["mesh_sha256"]),
            link["name"], np.asarray(link["CAD_from_link_zero"]))
        link["visual_mesh"] = visual
        link["collision_meshes"] = [{**visual, "role": "ORIGINAL_CAD_SURFACE", "source_mesh_complete": body["mesh_complete"],
            "source_topology_valid": body["topology_valid"]}]
        for index, patch in enumerate((repair or {}).get("patches", [])):
            if patch["cad_body"] != link["cad_body"]:
                continue
            if patch["coordinate_frame"] != "CAD_METRES":
                raise ValueError("S50_REPAIR_PATCH_REQUIRES_CAD_METRES")
            target = link["name"]+f"_collision_patch_{index}"
            collision = _export_local_mesh(root, dict(path=patch["mesh"], sha256=patch["mesh_sha256"]),
                target, np.asarray(link["CAD_from_link_zero"]))
            link["collision_meshes"].append({**collision, "role": "INDEPENDENT_SOURCE_FACE_REPAIR", "source_patch": patch})
    urdf_path, srdf_path = write_urdf(root, definition)
    definition["urdf_path"] = urdf_path.relative_to(root).as_posix()
    definition["urdf_sha256"] = sha256(urdf_path.read_bytes())
    definition["srdf_path"] = srdf_path.relative_to(root).as_posix()
    definition["srdf_sha256"] = sha256(srdf_path.read_bytes())
    definition["config_path"] = PREVIEW_CONFIG
    _write_json(root / DEFINITION_PATH, definition)
    config = dict(schema_version="cad_preview_robot_binding_v1", model=MODEL_ID, model_id=MODEL_ID, model_usage=MODEL_USAGE,
        model_revision="measured_S50_CAD_user_authorized_simulation_zero_v1",
        kinematics=dict(urdf_path="../../"+definition["urdf_path"], srdf_path="../../"+definition["srdf_path"],
            active_joint_names=definition["joint_names"], base_link=definition["base_link"], flange_link=definition["flange_link"],
            tip_link=definition["tip_link"], mounting_from_base=np.eye(4).tolist(), planning_group="manipulator",
            zero_pose_reference="USER_AUTHORIZED_SUPPLIED_CAD_ASSEMBLY_NOT_CONTROLLER_ZERO",
            axes_zero_offsets_signs_source="../../"+DEFINITION_PATH, package_map={}),
        preview_only=dict(display_range_rad=[-float(np.pi), float(np.pi)], mechanical_limits=False,
            hardware_velocity_limits=False, urdf_effort_is_placeholder=True,
            controller_mapping_verified=False, installation_clocking_verified=False),
        collision=dict(capsule_radii_m=None, capsule_self_exclusions=[], reviewed_self_collision_pairs=[]),
        tool_interface=dict(flange_from_project_tool_mount=None, additional_physical_adapter=False),
        readiness=dict(cad_articulation_defined=True, operational_planning_ready=False, dynamics_verified=False),
        generated_definition="../../"+DEFINITION_PATH,
        source_step_sha256=definition["source_step_sha256"])
    path = root / PREVIEW_CONFIG
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8", newline="\n")
    result = validate_preview_model(root)
    _write_json(asset / "generated/kinematics_validation.json", result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repair-report", type=Path)
    args = parser.parse_args(argv)
    result = build_model(repair_report=args.repair_report)
    print(json.dumps(dict(status=result["status"], model_usage=result["model_usage"],
        definition=DEFINITION_PATH, definition_sha256=result["definition_sha256"],
        q0_max_error_m=result["maximum_q0_coordinate_error_m"],
        fk_max_matrix_error=result["maximum_fk_matrix_error"], fk_configurations=len(result["fk_crosschecks"]),
        signed_axis_checks=len(result["single_axis_checks"]))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
