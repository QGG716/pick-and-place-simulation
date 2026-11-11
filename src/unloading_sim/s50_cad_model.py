"""Measured S50 six-revolute-joint CAD preview, independent of controller zero.

The user-authorized simulation convention is q=0 at the supplied CAD assembly
and right-hand-positive rotations about the measured canonical axis directions.
No physical joint limits, controller signs, clocking or tool transform are
inferred. This module contains no ROS, GPU or dynamics dependency.
"""
from __future__ import annotations

from copy import deepcopy
import gzip
import hashlib
import json
from pathlib import Path
import zlib

import numpy as np

ASSET_DIRECTORY = "assets/robots/huayan_s50_2000"
DEFINITION_PATH = ASSET_DIRECTORY + "/generated/cad_kinematic_model.json"
PREVIEW_CONFIG = "configs/robots/huayan_s50_2000_cad_preview.yaml"
MODEL_ID = "huayan_s50_2000_cad_preview"
MODEL_USAGE = "cad_preview_fk_only"
SCHEMA = "s50_cad_simulation_6r_v1"
JOINT_NAMES = tuple(f"s50_joint_{i}" for i in range(1, 7))
BODY_ORDER = ("base_s50", *(f"j{i}_s50" for i in range(1, 7)))
LINK_NAMES = ("s50_base", *(f"s50_link_{i}" for i in range(1, 7)))


def project_root(root=None):
    return Path(root).resolve() if root is not None else Path(__file__).resolve().parents[2]


def sha256(content):
    return hashlib.sha256(content).hexdigest()


def _json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def rigid_transform(rotation=None, translation=None):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = np.asarray(rotation, dtype=float)
    if translation is not None:
        result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def _unit(vector):
    result = np.asarray(vector, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all() or np.linalg.norm(result) < 1e-12:
        raise ValueError("S50_CAD_INVALID_AXIS")
    return result / np.linalg.norm(result)


def read_mesh_bytes(path):
    path = Path(path)
    if path.is_file():
        return path.read_bytes()
    packed = path.with_name(path.name + ".gz")
    if not packed.is_file():
        raise FileNotFoundError("S50_CAD_MESH_MISSING:" + str(path))
    try:
        return gzip.decompress(packed.read_bytes())
    except (OSError, EOFError, zlib.error) as exc:
        raise ValueError("S50_CAD_INVALID_COMPRESSED_MESH:" + str(packed)) from exc


def read_obj(path, expected_sha256=None):
    """Keep source float64 coordinates; display Float32 conversion is separate."""
    content = read_mesh_bytes(path)
    if expected_sha256 is not None and sha256(content) != expected_sha256:
        raise ValueError("S50_CAD_MESH_SHA256_MISMATCH:" + str(path))
    vertices, triangles = [], []
    for line in content.decode("utf-8").splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            if len(fields) != 4:
                raise ValueError("S50_CAD_OBJ_REQUIRES_XYZ")
            vertices.append([float(v) for v in fields[1:]])
        elif fields[0] == "f":
            if len(fields) != 4:
                raise ValueError("S50_CAD_OBJ_REQUIRES_TRIANGLES")
            indices = [int(v.split("/")[0]) for v in fields[1:]]
            triangles.append([v-1 if v > 0 else len(vertices)+v for v in indices])
    vertices, triangles = np.asarray(vertices, dtype=float), np.asarray(triangles, dtype=np.int64)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices) or not np.isfinite(vertices).all()
            or triangles.ndim != 2 or triangles.shape[1] != 3 or not len(triangles)
            or triangles.min() < 0 or triangles.max() >= len(vertices)):
        raise ValueError("S50_CAD_INVALID_OBJ")
    return vertices, triangles


def transform_vertices(transform, vertices):
    transform = np.asarray(transform, dtype=float)
    return np.asarray(vertices) @ transform[:3, :3].T + transform[:3, 3]


def rotation_about_axis(axis, angle):
    x, y, z = _unit(axis)
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle)*skew + (1.-np.cos(angle))*(skew @ skew)


def rotation_about_line(axis, point, angle):
    rotation = rotation_about_axis(axis, angle)
    point = np.asarray(point, dtype=float)
    return rigid_transform(rotation, point-rotation @ point)


def _configuration(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError("S50_CAD_REQUIRES_SIX_FINITE_ANGLES")
    return q


def build_definition(measurements, audit):
    if measurements["source_sha256"] != audit["source_sha256"]:
        raise ValueError("S50_CAD_MEASUREMENT_SOURCE_MISMATCH")
    parts = {part["id"]: part for part in audit["parts"]}
    axes = measurements["axes"]
    if set(parts) != set(BODY_ORDER) or len(axes) != 6:
        raise ValueError("S50_CAD_REQUIRES_REAL_SEVEN_BODY_SIX_AXIS_CHAIN")
    # User-authorized display clocking, not verified controller/base clocking:
    # preview X=CAD +X, preview Y=CAD -Z, preview Z=CAD +Y.
    rotation = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])
    support = np.asarray(measurements["support_plane"]["axis_intersection_CAD_m"])
    mount = rigid_transform(rotation, support)
    transforms = {"s50_base": mount}
    links = [dict(name="s50_base", cad_body="base_s50", CAD_from_link_zero=mount.tolist())]
    joints = []
    for index, axis_record in enumerate(axes):
        if (axis_record["joint_label"] != f"J{index+1}"
                or axis_record["parent_cad_body"] != BODY_ORDER[index]
                or axis_record["child_cad_body"] != BODY_ORDER[index+1]
                or axis_record["candidate_line_groups"] != 1):
            raise ValueError("S50_CAD_MEASURED_CHAIN_ORDER_MISMATCH")
        axis = _unit(axis_record["unoriented_axis_CAD"])
        measured_point = np.asarray(axis_record["point_CAD_m"])
        # Picking a different point on the measured infinite line changes only
        # coordinates, not the revolute motion. Use the child's documented CAD
        # instance origin to avoid remote frame origins along J1/J5 axis lines.
        instance_origin = np.asarray(parts[BODY_ORDER[index+1]]["cad_placement"])[:3, 3]
        shift = float(np.dot(instance_origin-measured_point, axis))
        point = measured_point + shift*axis
        child = rigid_transform(rotation, point)
        parent_name, child_name = LINK_NAMES[index:index+2]
        transforms[child_name] = child
        joint = dict(name=JOINT_NAMES[index], type="revolute", parent_link=parent_name, child_link=child_name,
            origin=(np.linalg.inv(transforms[parent_name]) @ child).tolist(),
            axis=(rotation.T @ axis).tolist(), point_CAD_m=point.tolist(), axis_CAD=axis.tolist(),
            measured_line_point_CAD_m=measured_point.tolist(),
            frame_origin_selection="PROJECT_CHILD_CAD_INSTANCE_ORIGIN_ON_MEASURED_AXIS_LINE",
            child_CAD_instance_origin_m=instance_origin.tolist(), line_parameter_shift_m=shift,
            simulation_positive="RIGHT_HAND_RULE_ABOUT_RECORDED_CANONICAL_AXIS",
            simulation_zero="SUPPLIED_CAD_ASSEMBLY", controller_mapping_verified=False,
            controller_zero_offset_rad=None, controller_positive_sign=None,
            preview_range_rad=[-float(np.pi), float(np.pi)],
            range_semantics="DISPLAY_ONLY_NOT_MECHANICAL_LIMITS",
            axis_measurement_evidence_count=len(axis_record["evidence"]))
        joints.append(joint)
        links.append(dict(name=child_name, cad_body=BODY_ORDER[index+1], CAD_from_link_zero=child.tolist()))
    flange_record = measurements["flange_distal_face"]
    flange_z = _unit(flange_record["normal"])
    reference_x = np.array([0., 1., 0.])
    flange_x = _unit(reference_x - np.dot(reference_x, flange_z)*flange_z)
    flange_y = _unit(np.cross(flange_z, flange_x))
    flange = rigid_transform(np.column_stack((flange_x, flange_y, flange_z)), flange_record["origin_m"])
    fixed = dict(name="s50_link_6_to_flange", type="fixed", parent_link="s50_link_6", child_link="s50_flange",
        origin=(np.linalg.inv(transforms["s50_link_6"]) @ flange).tolist())
    return dict(schema=SCHEMA, model_id=MODEL_ID, model_usage=MODEL_USAGE,
        units=dict(length="metre", angle="radian"), source_step_sha256=audit["source_sha256"],
        simulation_convention=dict(q0_rad=[0.]*6, zero="SUPPLIED_CAD_ASSEMBLY_USER_AUTHORIZED_SIMULATION_ZERO",
            positive="RIGHT_HAND_RULE_ABOUT_MEASURED_CANONICAL_AXIS", controller_zero_verified=False,
            controller_signs_verified=False, controller_reference_pose_verified=False),
        joint_names=list(JOINT_NAMES), base_link="s50_base", flange_link="s50_flange", tip_link="s50_flange",
        links=links, joints=joints, fixed_joints=[fixed],
        frames=dict(mount=dict(link="s50_base", CAD_from_frame_zero=mount.tolist(),
            origin="MEASURED_SUPPORT_PLANE_AND_J1_AXIS_INTERSECTION",
            axes="X=CAD+X; Y=CAD-Z; Z=CAD+Y", clocking_status="SIMULATION_CONVENTION_NOT_MACHINE_VERIFIED",
            support_plane_source=deepcopy(measurements["support_plane"])),
            flange=dict(link="s50_flange", CAD_from_frame_zero=flange.tolist(),
                origin="MEASURED_DISTAL_J6_FACE_ORIGIN", z_axis="MEASURED_OUTWARD_FACE_NORMAL",
                x_axis="PROJECTION_OF_CAD+Y_ON_FLANGE_PLANE_SIMULATION_CLOCKING",
                source_face_index=flange_record["face_index"], controller_tool0_verified=False),
            controller_tool0=None, physical_tool_mount=None, virtual_tcp=None),
        preview_constraints=dict(position_range_semantics="DISPLAY_ONLY_NOT_MECHANICAL_LIMITS",
            velocity_rad_s=1., velocity_semantics="URDF_PARSER_AND_DISPLAY_PLACEHOLDER_NOT_HARDWARE_LIMIT",
            effort_nm=0., effort_semantics="URDF_REQUIRED_FIELD_PLACEHOLDER_NOT_DYNAMICS_INPUT"),
        collision_coverage_status="NOT_QUALIFIED", collision_repair=None,
        operational_planning_ready=False, dynamics_verified=False,
        tool_attached=False, physical_adapter_added=False,
        claims=dict(real_measured_six_revolute_chain=True, kinematics_checks="RUN_SEPARATELY",
            official_controller_model=False, physical_joint_limits_verified=False,
            environment_collision_clearance_verified=False, motion_plan_generated=False))


def chain_fk(definition, q, world_from_mount=None):
    """Recursive URDF-style FK; returns world/base-mount-from-link matrices."""
    q = _configuration(q)
    frames = {definition["base_link"]: np.eye(4) if world_from_mount is None else np.asarray(world_from_mount, dtype=float)}
    for joint, angle in zip(definition["joints"], q):
        frames[joint["child_link"]] = (frames[joint["parent_link"]] @ np.asarray(joint["origin"])
                                       @ rigid_transform(rotation_about_axis(joint["axis"], angle)))
    for joint in definition["fixed_joints"]:
        frames[joint["child_link"]] = frames[joint["parent_link"]] @ np.asarray(joint["origin"])
    return frames


def poe_fk(definition, q, world_from_mount=None):
    """Independent spatial screw/axis-line product in the original CAD frame."""
    q = _configuration(q)
    world = np.eye(4) if world_from_mount is None else np.asarray(world_from_mount, dtype=float)
    world_from_CAD = world @ np.linalg.inv(definition["frames"]["mount"]["CAD_from_frame_zero"])
    frames = {definition["base_link"]: world.copy()}
    prefix = np.eye(4)
    links = {link["name"]: link for link in definition["links"]}
    for joint, angle in zip(definition["joints"], q):
        prefix = prefix @ rotation_about_line(joint["axis_CAD"], joint["measured_line_point_CAD_m"], angle)
        frames[joint["child_link"]] = world_from_CAD @ prefix @ np.asarray(links[joint["child_link"]]["CAD_from_link_zero"])
    frames[definition["flange_link"]] = world_from_CAD @ prefix @ np.asarray(definition["frames"]["flange"]["CAD_from_frame_zero"])
    return frames


def load_definition(root=None):
    path = project_root(root) / DEFINITION_PATH
    definition = _json(path)
    if definition.get("schema") != SCHEMA or definition.get("model_usage") != MODEL_USAGE:
        raise ValueError("S50_CAD_PREVIEW_DEFINITION_SCOPE_MISMATCH")
    definition["definition_sha256"] = sha256(path.read_bytes())
    return definition


def ensure_preview_assets(root=None):
    """Verify all generated meshes, materializing raw OBJ for URDF readers."""
    root = project_root(root)
    definition = load_definition(root)
    verified, materialized = [], []
    for kind in ("urdf", "srdf"):
        name = definition[kind+"_path"]
        path = (root / name).resolve()
        if (not path.is_relative_to(root / ASSET_DIRECTORY / "urdf")
                or sha256(path.read_bytes()) != definition[kind+"_sha256"]):
            raise ValueError("S50_CAD_PREVIEW_" + kind.upper() + "_IDENTITY_CHANGED")
    sources = definition.get("source_files", {})
    for name, expected in sources.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root):
            raise ValueError("S50_CAD_PREVIEW_SOURCE_CHANGED:" + name)
        content = path.read_bytes()
        valid = sha256(content) == expected["sha256"]
        if not valid and "canonical_json_sha256" in expected:
            canonical = json.dumps(json.loads(content.decode("utf-8")), sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            valid = sha256(canonical) == expected["canonical_json_sha256"]
        if not valid:
            raise ValueError("S50_CAD_PREVIEW_SOURCE_CHANGED:" + name)
    unique = {}
    for link in definition["links"]:
        for mesh in [link["visual_mesh"], *link["collision_meshes"]]:
            unique[mesh["path"]] = mesh
    for name, record in unique.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root / ASSET_DIRECTORY / "meshes/link-local"):
            raise ValueError("S50_CAD_PREVIEW_MESH_OUTSIDE_OWN_ASSET_DIRECTORY")
        existed = path.is_file()
        content = read_mesh_bytes(path)
        if sha256(content) != record["sha256"]:
            raise ValueError("S50_CAD_PREVIEW_LINK_MESH_CHANGED:" + name)
        if not existed:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
            materialized.append(name)
        verified.append(name)
    return dict(status="PASS", verified=verified, materialized=materialized,
        definition_sha256=definition["definition_sha256"])


def load_preview_robot(root=None, *, world_from_mount=None):
    root = project_root(root)
    ensure_preview_assets(root)
    from .robot import URDFRobot
    robot = URDFRobot.from_config(root / PREVIEW_CONFIG, mounting_world_transform=world_from_mount)
    if robot.model_binding.data.get("model_usage") != MODEL_USAGE:
        raise ValueError("S50_CAD_PREVIEW_CONFIG_SCOPE_MISMATCH")
    definition = load_definition(root)
    binding = robot.model_binding
    if (binding.model_id != definition["model_id"] or list(binding.active_joint_names) != definition["joint_names"]
            or any(getattr(binding, field) != definition[field] for field in ("base_link", "flange_link", "tip_link"))
            or not np.allclose(binding.mounting_from_base, np.eye(4), atol=1e-14, rtol=0)
            or not np.allclose(binding.joint_limits, [joint["preview_range_rad"] for joint in definition["joints"]], atol=1e-14, rtol=0)):
        raise ValueError("S50_CAD_PREVIEW_CONFIG_DEFINITION_BINDING_CHANGED")
    robot.cad_preview_definition = definition
    return robot


def validate_preview_model(root=None):
    """Actually check geometry recovery, all descendants and three FK paths."""
    root = project_root(root)
    definition = load_definition(root)
    robot = load_preview_robot(root)
    audit = _json(root / ASSET_DIRECTORY / "derived/cad_audit.json")
    parts = {part["id"]: part for part in audit["parts"]}
    zero = robot.named_link_frames(np.zeros(6))
    CAD_from_mount = np.asarray(definition["frames"]["mount"]["CAD_from_frame_zero"])
    geometries, recovery, patch_recovery = {}, [], []
    for link in definition["links"]:
        source, faces = read_obj(root / parts[link["cad_body"]]["mesh"], parts[link["cad_body"]]["mesh_sha256"])
        local, local_faces = read_obj(root / link["visual_mesh"]["path"], link["visual_mesh"]["sha256"])
        if not np.array_equal(faces, local_faces):
            raise ValueError("S50_CAD_PREVIEW_TRIANGLE_TOPOLOGY_CHANGED")
        reconstructed = transform_vertices(CAD_from_mount @ zero[link["name"]], local)
        error = float(np.max(np.abs(reconstructed-source)))
        if error > 2e-12:
            raise ValueError("S50_CAD_PREVIEW_Q0_MESH_RECOVERY_FAILED:" + link["name"])
        recovery.append(dict(link=link["name"], vertices=len(local), triangles=len(faces), maximum_coordinate_error_m=error))
        for collider in link["collision_meshes"]:
            if collider.get("role") != "INDEPENDENT_SOURCE_FACE_REPAIR":
                continue
            patch = collider["source_patch"]
            original, original_faces = read_obj(root / patch["mesh"], patch["mesh_sha256"])
            local_patch, local_patch_faces = read_obj(root / collider["path"], collider["sha256"])
            if not np.array_equal(original_faces, local_patch_faces):
                raise ValueError("S50_CAD_PREVIEW_COLLISION_PATCH_TOPOLOGY_CHANGED")
            restored = transform_vertices(CAD_from_mount @ zero[link["name"]], local_patch)
            patch_error = float(np.max(np.abs(restored-original)))
            if patch_error > 2e-12:
                raise ValueError("S50_CAD_PREVIEW_COLLISION_PATCH_FRAME_MISMATCH")
            patch_recovery.append(dict(link=link["name"], source_face_index=patch["source_face_index"],
                vertices=len(local_patch), triangles=len(local_patch_faces), maximum_coordinate_error_m=patch_error,
                source_patch_sha256=patch["mesh_sha256"], physical_thickness_added=False))
        # Probe non-collinear mesh samples for every body, not just frame origins.
        sample = np.unique(np.linspace(0, len(local)-1, min(97, len(local)), dtype=int))
        geometries[link["name"]] = (source[sample], local[sample])
    cases = [np.zeros(6)]
    single_axis = []
    for axis_index, joint in enumerate(definition["joints"]):
        for angle in (-.35, .35):
            q = np.zeros(6); q[axis_index] = angle
            cases.append(q)
            frames = robot.named_link_frames(q)
            motion = rotation_about_line(joint["axis_CAD"], joint["measured_line_point_CAD_m"], angle)
            errors, moved = {}, []
            for link_index, link in enumerate(definition["links"]):
                source, local = geometries[link["name"]]
                actual = transform_vertices(CAD_from_mount @ frames[link["name"]], local)
                descendant = link_index > axis_index
                expected = transform_vertices(motion, source) if descendant else source
                errors[link["name"]] = float(np.max(np.abs(actual-expected)))
                if float(np.max(np.abs(actual-source))) > 1e-8:
                    moved.append(link["name"])
                if errors[link["name"]] > 2e-11:
                    raise ValueError("S50_CAD_PREVIEW_WRONG_DESCENDANT_MOTION")
            expected_moved = list(LINK_NAMES[axis_index+1:])
            if moved != expected_moved:
                raise ValueError("S50_CAD_PREVIEW_JOINT_DID_NOT_MOVE_EXPECTED_BODIES")
            single_axis.append(dict(joint=joint["name"], angle_rad=angle, moved_links=moved,
                unchanged_links=list(LINK_NAMES[:axis_index+1]), maximum_coordinate_error_m=max(errors.values())))
    cases.extend(np.random.default_rng(20261110).uniform(-1.2, 1.2, (12, 6)))
    comparisons = []
    for q in cases:
        actual, recursive, independent = robot.named_link_frames(q), chain_fk(definition, q), poe_fk(definition, q)
        recursive_error = max(float(np.max(np.abs(actual[name]-recursive[name]))) for name in actual)
        poe_error = max(float(np.max(np.abs(actual[name]-independent[name]))) for name in actual)
        if max(recursive_error, poe_error) > 2e-11:
            raise ValueError("S50_CAD_PREVIEW_FK_CROSSCHECK_FAILED")
        comparisons.append(dict(q_rad=np.asarray(q).tolist(), recursive_max_matrix_error=recursive_error,
            spatial_poe_max_matrix_error=poe_error,
            mount_from_flange=actual[definition["flange_link"]].tolist()))
    return dict(schema="s50_cad_preview_kinematics_validation_v1", status="PASS",
        definition_sha256=definition["definition_sha256"], model_usage=MODEL_USAGE,
        q0_geometry=recovery, collision_patch_geometry=patch_recovery, single_axis_checks=single_axis, fk_crosschecks=comparisons,
        deterministic_rng_seed=20261110, maximum_q0_coordinate_error_m=max(r["maximum_coordinate_error_m"] for r in recovery),
        maximum_fk_matrix_error=max(max(r["recursive_max_matrix_error"], r["spatial_poe_max_matrix_error"]) for r in comparisons),
        controller_zero_verified=False, controller_signs_verified=False,
        official_mechanical_limits_verified=False, collision_clearance_checked=False,
        dynamics_verified=False, motion_planning_run=False)
