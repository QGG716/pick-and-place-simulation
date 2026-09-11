"""Versioned, immutable workcell layout and scene snapshots for M-710 V3.

This module contains only CPU geometry and serialization.  Optional viewers and
Isaac Sim consume the frozen snapshot instead of reopening mutable YAML files.
"""

from __future__ import annotations

from dataclasses import dataclass
import copy
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from .geometry import OBB, make_transform, rotation_matrix_from_rpy
from .collision_policy import SimulationCollisionPolicy
from .identity import load_tool_config
from .robot import URDFRobot
from .validation_physics import contact_separated, urdf_collision_shapes, world_link_boxes


LAYOUT_SCHEMA = "m710id70_unloading_layout_v1"
VALIDATION_SCHEMA = "m710id70_layout_validation_v1"
SNAPSHOT_SCHEMA = "m710id70_workcell_snapshot_v1"


def canonical_digest(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def compute_layout_fingerprint(data: Mapping[str, Any], assets: Mapping[str, Mapping[str, str]]) -> str:
    """Hash fixed assembly semantics while excluding its current world pose."""
    semantic = copy.deepcopy(dict(data))
    semantic.pop("assembly", None)
    return canonical_digest({"schema": semantic.get("schema"), "fixed_assembly": semantic, "assets": assets})


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _keys(value: Mapping[str, Any], expected: set[str], name: str) -> None:
    unknown = set(value) - expected
    missing = expected - set(value)
    if unknown or missing:
        raise ValueError(f"{name} unknown={sorted(unknown)} missing={sorted(missing)}")


def _vector(value: Any, length: int, name: str, *, positive: bool = False) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite vector of length {length}")
    if positive and np.any(result <= 0.0):
        raise ValueError(f"{name} values must be positive")
    return result


def _positive(value: Any, name: str, *, allow_zero: bool = False) -> float:
    result = float(value)
    if not np.isfinite(result) or (result < 0.0 if allow_zero else result <= 0.0):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
    return result


def _resolve(config_path: Path, declared: str) -> Path:
    path = Path(declared)
    result = path.resolve() if path.is_absolute() else (config_path.parent / path).resolve()
    if not result.is_file():
        raise FileNotFoundError(f"declared layout asset is missing: {declared}")
    return result


def _pose(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    return make_transform(rotation_matrix_from_rpy(*np.asarray(rpy, dtype=float)), xyz)


def _tool_compound_boxes(
    geometry_config: Mapping[str, Any], project_root: Path, bounds_key: str
) -> np.ndarray:
    """Transform one audited STEP-bound group into virtual-TCP coordinates.

    The STEP assembly coordinates are an axis permutation/reflection of the
    flange-aligned tool coordinates, so transforming all eight corners keeps
    each component an exact OBB (represented as an axis-aligned box in the
    virtual TCP frame). Only the source-identified flexible bellows use the
    separate compliant contact representation; rigid inserts remain present.
    """

    geometry = _mapping(geometry_config.get("geometry"), "tool geometry")
    analysis_path = (project_root / str(geometry["mass_properties_path"])).resolve()
    if not analysis_path.is_file():
        raise FileNotFoundError(f"tool mass/contact analysis is missing: {analysis_path}")
    analysis = _mapping(json.loads(analysis_path.read_text(encoding="utf-8")), "tool analysis")
    from .tool_geometry import audit_tool_geometry
    coverage = audit_tool_geometry(project_root)
    bounds = np.asarray(coverage[bounds_key], dtype=float)
    rotation_step_from_tool = np.asarray(analysis.get("rotation_step_from_tool"), dtype=float)
    flange_step_mm = np.asarray(analysis.get("flange_origin_step_mm"), dtype=float)
    if (
        rotation_step_from_tool.shape != (3, 3)
        or flange_step_mm.shape != (3,)
        or not np.allclose(rotation_step_from_tool.T @ rotation_step_from_tool, np.eye(3), atol=1e-12)
    ):
        raise ValueError("tool STEP-to-flange transform is invalid")
    flange_from_virtual = np.eye(4)
    flange_from_virtual[:3, :3] = rotation_matrix_from_rpy(0.0, np.pi / 2.0, 0.0)
    flange_from_virtual[0, 3] = 0.250
    virtual_from_flange = np.linalg.inv(flange_from_virtual)
    rows: list[np.ndarray] = []
    for lower_x, lower_y, lower_z, upper_x, upper_y, upper_z in bounds:
        corners_step = np.array(
            [
                [x, y, z]
                for x in (lower_x, upper_x)
                for y in (lower_y, upper_y)
                for z in (lower_z, upper_z)
            ],
            dtype=float,
        )
        corners_flange = (corners_step - flange_step_mm) @ rotation_step_from_tool * 0.001
        corners_virtual = (
            corners_flange @ virtual_from_flange[:3, :3].T
            + virtual_from_flange[:3, 3]
        )
        low = np.min(corners_virtual, axis=0)
        high = np.max(corners_virtual, axis=0)
        rows.append(np.concatenate((0.5 * (low + high), high - low)))
    return np.asarray(rows, dtype=float)


def _rigid_tool_compound_boxes(
    geometry_config: Mapping[str, Any], project_root: Path
) -> np.ndarray:
    """Return all audited rigid structures and inserts in virtual-TCP coordinates."""

    return _tool_compound_boxes(
        geometry_config, project_root, "rigid_collision_bounding_boxes_step_mm"
    )


def _compliant_tool_compound_boxes(
    geometry_config: Mapping[str, Any], project_root: Path
) -> np.ndarray:
    """Return conservative uncompressed FG42 bounds for swept-path audits."""

    return _tool_compound_boxes(
        geometry_config, project_root, "compliant_bellows_bounds_step_mm"
    )


def _obb_record(box: OBB, *, shape: str = "box", **extra: Any) -> dict[str, Any]:
    return {
        "name": box.name,
        "category": box.category,
        "shape": shape,
        "pose_world": box.world_from_local.tolist(),
        "center_m": box.center.tolist(),
        "half_extents_m": box.half_extents.tolist(),
        **extra,
    }


def _tool_compound_envelope(robot: URDFRobot, q: np.ndarray) -> OBB:
    """Return a display/snapshot envelope, never an execution collision proxy."""

    components = robot.tool_collision_obbs(q)
    if not components:
        legacy = robot.tool_collision_obb(q)
        if legacy is None:
            raise ValueError("robot has no configured tool collision geometry")
        return legacy
    frame = robot.fk(q)
    local_corners = np.concatenate(
        [(component.corners() - frame[:3, 3]) @ frame[:3, :3] for component in components]
    )
    lower = np.min(local_corners, axis=0)
    upper = np.max(local_corners, axis=0)
    center_local = 0.5 * (lower + upper)
    return OBB(
        frame[:3, :3] @ center_local + frame[:3, 3],
        0.5 * (upper - lower),
        frame[:3, :3],
        "tool_compound_display_envelope",
        "robot",
    )


@dataclass(frozen=True)
class WorkcellLayout:
    config_path: Path
    data: Mapping[str, Any]
    assets: Mapping[str, Mapping[str, str]]
    layout_fingerprint: str

    @property
    def world_from_assembly(self) -> np.ndarray:
        assembly = self.data["assembly"]
        return _pose(assembly["world_xyz_m"], assembly["world_rpy_rad"])

    @property
    def assembly_from_world(self) -> np.ndarray:
        return np.linalg.inv(self.world_from_assembly)

    def component_local_boxes(self) -> tuple[OBB, OBB, OBB]:
        chassis = self.data["chassis"]
        conveyors = self.data["conveyors"]
        thickness = float(conveyors["body_thickness_m"])
        surface_z = float(conveyors["surface_z_a_m"])
        transverse = conveyors["transverse"]
        longitudinal = conveyors["longitudinal"]
        return (
            OBB(chassis["center_a_m"], 0.5 * np.asarray(chassis["size_xyz_m"], float), np.eye(3), "chassis", "chassis"),
            OBB(
                [*transverse["center_xy_a_m"], surface_z - 0.5 * thickness],
                0.5 * np.asarray([*transverse["size_xy_m"], thickness], float),
                np.eye(3),
                "conveyor_transverse",
                "conveyor",
            ),
            OBB(
                [*longitudinal["center_xy_a_m"], surface_z - 0.5 * thickness],
                0.5 * np.asarray([*longitudinal["size_xy_m"], thickness], float),
                np.eye(3),
                "conveyor_longitudinal",
                "conveyor",
            ),
        )

    def fixed_components(self) -> tuple[OBB, OBB, OBB]:
        transform = self.world_from_assembly
        return tuple(box.transformed(transform) for box in self.component_local_boxes())

    def cartons(self) -> tuple[OBB, ...]:
        stack = self.data["carton_stack"]
        size = np.asarray(stack["carton_size_xyz_m"], dtype=float)
        x = float(stack["front_face_x_m"]) + 0.5 * size[0]
        result = []
        for layer in range(int(stack["height_layers"])):
            z = 0.5 * size[2] + layer * (size[2] + float(stack["layer_gap_m"]))
            for column, y in enumerate(stack["center_y_m"]):
                result.append(
                    OBB([x, y, z], 0.5 * size, np.eye(3), f"carton_l{layer:02d}_c{column:02d}", "carton")
                )
        return tuple(result)

    def robot_base_transform(self) -> np.ndarray:
        robot = self.data["robot"]
        local = make_transform(
            translation=[
                *robot["base_origin_xy_a_m"],
                float(robot["mounting_surface_z_a_m"]),
            ]
        )
        return self.world_from_assembly @ local

    def robot(self) -> URDFRobot:
        robot_data = self.data["robot"]
        tool_data = self.data["tool"]
        tool = load_tool_config(_resolve(self.config_path, str(tool_data["load_config"])))
        geometry = load_tool_config(_resolve(self.config_path, str(tool_data["geometry_config"])))
        rigid_tool_boxes = _rigid_tool_compound_boxes(geometry.data, self.config_path.parents[2])
        compliant_tool_boxes = _compliant_tool_compound_boxes(
            geometry.data, self.config_path.parents[2]
        )
        base = self.robot_base_transform()
        robot = URDFRobot.fanuc_m710id_70(
            urdf_path=_resolve(self.config_path, str(robot_data["urdf_path"])),
            base_position=base[:3, 3],
            base_rpy=self.data["assembly"]["world_rpy_rad"],
            tool_length=float(tool.tcp_translation_xyz_m[0]),
            tool_collision_size=None,
            tool_collision_center_offset=None,
            tool_collision_local_boxes=rigid_tool_boxes,
        )
        mechanical_tcp = make_transform(tool.tcp_rotation_matrix, tool.tcp_translation_xyz_m)
        task_axes = make_transform(rotation_matrix_from_rpy(0.0, np.pi / 2.0, 0.0))
        frames = robot.named_link_frames(np.zeros(6))
        flange_from_tip = np.linalg.inv(frames["flange"]) @ frames["tool0"]
        robot.tip_from_tcp = np.linalg.inv(flange_from_tip) @ mechanical_tcp @ task_axes
        # These conservative, source-audited uncompressed bellows bounds are
        # not promoted into rigid planning collisions. They are consumed by
        # the post-release swept-path certificate so all 202 physical tool
        # entities, including inactive compliant cups, clear the moving box.
        robot.tool_compliant_collision_local_boxes = compliant_tool_boxes
        return robot


@dataclass(frozen=True)
class LayoutValidationConfig:
    config_path: Path
    data: Mapping[str, Any]
    layout: WorkcellLayout

    @property
    def initial_q(self) -> np.ndarray:
        return np.asarray(self.data["initial_state"]["q_rad"], dtype=float)

    @property
    def collision_margin_m(self) -> float:
        return float(self.data["collision"]["margin_m"])

    @property
    def contact_tolerance_m(self) -> float:
        return float(self.data["collision"]["assembly_contact_tolerance_m"])


def _asset_manifest(config_path: Path, data: Mapping[str, Any]) -> dict[str, dict[str, str]]:
    project_root = config_path.parents[2]
    robot = data["robot"]
    tool = data["tool"]
    declared = {
        "robot_model_config": str(robot["model_config"]),
        "robot_urdf": str(robot["urdf_path"]),
        "tool_load_config": str(tool["load_config"]),
        "tool_geometry_config": str(tool["geometry_config"]),
    }
    paths = {name: _resolve(config_path, value) for name, value in declared.items()}
    geometry_config = yaml.safe_load(paths["tool_geometry_config"].read_text(encoding="utf-8"))
    for name, key in (
        ("tool_visual_mesh", "visual_mesh_path"),
        ("tool_collision_mesh", "collision_mesh_path"),
        ("tool_step_analysis", "mass_properties_path"),
    ):
        value = geometry_config["geometry"][key]
        path = (project_root / value).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"declared layout asset is missing: {value}")
        declared[name] = str(value)
        paths[name] = path
    return {
        name: {
            "declared_path": declared[name],
            "repository_path": path.relative_to(project_root).as_posix(),
            "sha256": sha256_file(path),
        }
        for name, path in paths.items()
    }


def load_workcell_layout(path: str | Path) -> WorkcellLayout:
    config_path = Path(path).resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    data = _mapping(data, "layout")
    _keys(
        data,
        {"schema", "layout_id", "units", "world", "assembly", "chassis", "conveyors", "robot", "tool", "trailer", "carton_stack", "evidence"},
        "layout",
    )
    if data["schema"] != LAYOUT_SCHEMA or data["layout_id"] != LAYOUT_SCHEMA:
        raise ValueError("unsupported or inconsistent layout schema/id")
    _keys(_mapping(data["units"], "units"), {"length", "angle", "mass"}, "units")
    if data["units"] != {"length": "metre", "angle": "radian", "mass": "kilogram"}:
        raise ValueError("layout must use SI units")
    _keys(_mapping(data["world"], "world"), {"origin", "axes", "floor_z_m"}, "world")
    _keys(_mapping(data["assembly"], "assembly"), {"world_xyz_m", "world_rpy_rad"}, "assembly")
    _vector(data["assembly"]["world_xyz_m"], 3, "assembly.world_xyz_m")
    _vector(data["assembly"]["world_rpy_rad"], 3, "assembly.world_rpy_rad")
    _keys(_mapping(data["chassis"], "chassis"), {"size_xyz_m", "center_a_m"}, "chassis")
    _vector(data["chassis"]["size_xyz_m"], 3, "chassis.size_xyz_m", positive=True)
    _vector(data["chassis"]["center_a_m"], 3, "chassis.center_a_m")
    conveyors = _mapping(data["conveyors"], "conveyors")
    _keys(conveyors, {"body_thickness_m", "body_thickness_source", "surface_z_a_m", "transverse", "longitudinal"}, "conveyors")
    _positive(conveyors["body_thickness_m"], "conveyors.body_thickness_m")
    _positive(conveyors["surface_z_a_m"], "conveyors.surface_z_a_m", allow_zero=True)
    for name in ("transverse", "longitudinal"):
        entry = _mapping(conveyors[name], f"conveyors.{name}")
        _keys(entry, {"size_xy_m", "center_xy_a_m"}, f"conveyors.{name}")
        _vector(entry["size_xy_m"], 2, f"conveyors.{name}.size_xy_m", positive=True)
        _vector(entry["center_xy_a_m"], 2, f"conveyors.{name}.center_xy_a_m")
    robot = _mapping(data["robot"], "robot")
    _keys(robot, {"model", "model_config", "urdf_path", "mounting_surface_z_a_m", "base_origin_xy_a_m", "base_support_bbox_min_xyz_m", "base_support_bbox_max_xyz_m", "base_front_edge_x_world_m", "positioning_basis", "positioning_status"}, "robot")
    if robot["model"] != "fanuc_m710id_70":
        raise ValueError("layout robot must be fanuc_m710id_70")
    _vector(robot["base_origin_xy_a_m"], 2, "robot.base_origin_xy_a_m")
    base_min = _vector(robot["base_support_bbox_min_xyz_m"], 3, "robot.base_support_bbox_min_xyz_m")
    base_max = _vector(robot["base_support_bbox_max_xyz_m"], 3, "robot.base_support_bbox_max_xyz_m")
    if np.any(base_max <= base_min) or abs(float(base_min[2])) > 1e-12:
        raise ValueError("official base support bounds must be ordered and start at mounting Z=0")
    tool = _mapping(data["tool"], "tool")
    _keys(tool, {"load_config", "geometry_config", "geometry_status", "physical_step_length_m", "planning_tcp_status"}, "tool")
    _positive(tool["physical_step_length_m"], "tool.physical_step_length_m")
    trailer = _mapping(data["trailer"], "trailer")
    _keys(trailer, {"inner_width_m", "right_wall_y_m", "left_wall_y_m", "length_m", "height_m", "length_status", "height_status"}, "trailer")
    _positive(trailer["inner_width_m"], "trailer.inner_width_m")
    if trailer["length_m"] is not None or trailer["height_m"] is not None:
        raise ValueError("layout v1 trailer length and height must remain undefined")
    if trailer["length_status"] != "NOT_DEFINED_BY_CONFIRMED_LAYOUT" or trailer["height_status"] != "NOT_DEFINED_BY_CONFIRMED_LAYOUT":
        raise ValueError("layout v1 must explicitly preserve unknown trailer length and height")
    stack = _mapping(data["carton_stack"], "carton_stack")
    _keys(stack, {"carton_size_xyz_m", "front_face_x_m", "depth_rows", "width_columns", "height_layers", "column_gap_m", "layer_gap_m", "center_y_m", "no_pallet"}, "carton_stack")
    _vector(stack["carton_size_xyz_m"], 3, "carton_stack.carton_size_xyz_m", positive=True)
    _vector(stack["center_y_m"], int(stack["width_columns"]), "carton_stack.center_y_m")
    if int(stack["depth_rows"]) != 1 or int(stack["width_columns"]) <= 0 or int(stack["height_layers"]) <= 0:
        raise ValueError("layout v1 requires one positive-depth row and positive stack counts")
    _positive(stack["column_gap_m"], "carton_stack.column_gap_m", allow_zero=True)
    _positive(stack["layer_gap_m"], "carton_stack.layer_gap_m", allow_zero=True)
    assets = _asset_manifest(config_path, data)
    return WorkcellLayout(config_path, data, assets, compute_layout_fingerprint(data, assets))


def load_layout_validation_config(path: str | Path) -> LayoutValidationConfig:
    config_path = Path(path).resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    data = _mapping(data, "layout validation config")
    _keys(data, {"schema", "layout_config", "initial_state", "collision", "render", "collision_policy"}, "layout validation config")
    SimulationCollisionPolicy.from_mapping(data["collision_policy"])
    if data["schema"] != VALIDATION_SCHEMA:
        raise ValueError("unsupported layout validation schema")
    initial = _mapping(data["initial_state"], "initial_state")
    _keys(initial, {"seed", "search_maximum_random_draws", "selected_random_draw_1_based", "previous_v3_q_rad", "q_rad"}, "initial_state")
    _vector(initial["q_rad"], 6, "initial_state.q_rad")
    _vector(initial["previous_v3_q_rad"], 6, "initial_state.previous_v3_q_rad")
    if isinstance(initial["seed"], bool) or int(initial["seed"]) != initial["seed"]:
        raise ValueError("initial_state.seed must be an integer")
    for key in ("search_maximum_random_draws", "selected_random_draw_1_based"):
        if key == "selected_random_draw_1_based" and initial[key] is None:
            continue  # A task-derived validated home is not a random draw.
        if isinstance(initial[key], bool) or int(initial[key]) != initial[key] or int(initial[key]) <= 0:
            raise ValueError(f"initial_state.{key} must be a positive integer")
    collision = _mapping(data["collision"], "collision")
    _keys(collision, {"margin_m", "assembly_contact_tolerance_m", "joint_margin_rad"}, "collision")
    for key in collision:
        _positive(collision[key], f"collision.{key}")
    render = _mapping(data["render"], "render")
    _keys(render, {"png_long_edge_px", "dpi", "figure_a_size_in", "figure_b_size_in"}, "render")
    _vector(render["figure_a_size_in"], 2, "render.figure_a_size_in", positive=True)
    _vector(render["figure_b_size_in"], 2, "render.figure_b_size_in", positive=True)
    layout = load_workcell_layout(_resolve(config_path, str(data["layout_config"])))
    return LayoutValidationConfig(config_path, data, layout)


def _interval(box: OBB, axis: int) -> tuple[float, float]:
    return float(box.center[axis] - box.half_extents[axis]), float(box.center[axis] + box.half_extents[axis])


def _contact_check(a: OBB, b: OBB, axis: int, a_side: int, tolerance: float) -> dict[str, Any]:
    a_interval = _interval(a, axis)
    b_interval = _interval(b, axis)
    a_face = a_interval[1] if a_side > 0 else a_interval[0]
    b_face = b_interval[0] if a_side > 0 else b_interval[1]
    gap = b_face - a_face if a_side > 0 else a_face - b_face
    tangential = [index for index in range(3) if index != axis]
    overlaps = [min(_interval(a, i)[1], _interval(b, i)[1]) - max(_interval(a, i)[0], _interval(b, i)[0]) for i in tangential]
    penetration = max(0.0, -gap)
    passed = abs(gap) <= tolerance and all(value > tolerance for value in overlaps) and penetration <= tolerance
    return {"status": "PASS" if passed else "FAIL", "gap_m": gap, "penetration_m": penetration, "tangential_overlap_m": overlaps}


def audit_layout_constraints(layout: WorkcellLayout, *, local_components: Iterable[OBB] | None = None, tolerance: float = 2e-4) -> dict[str, Any]:
    boxes = {box.name: box for box in (tuple(local_components) if local_components is not None else layout.component_local_boxes())}
    chassis = boxes["chassis"]
    transverse = boxes["conveyor_transverse"]
    longitudinal = boxes["conveyor_longitudinal"]
    transform = layout.world_from_assembly
    world = {name: box.transformed(transform) for name, box in boxes.items()}
    robot = layout.data["robot"]
    trailer = layout.data["trailer"]
    stack = layout.cartons()
    assembly_bounds = {
        axis: [min(_interval(box, index)[0] for box in world.values()), max(_interval(box, index)[1] for box in world.values())]
        for axis, index in (("x", 0), ("y", 1), ("z", 2))
    }
    base = layout.robot_base_transform()[:3, 3]
    base_min = np.asarray(robot["base_support_bbox_min_xyz_m"], dtype=float)
    base_max = np.asarray(robot["base_support_bbox_max_xyz_m"], dtype=float)
    base_front = float(base[0] + base_max[0])
    stack_corners = np.concatenate([box.corners() for box in stack])

    def scalar(actual: float, expected: float) -> dict[str, Any]:
        return {
            "status": "PASS" if abs(float(actual) - float(expected)) <= tolerance else "FAIL",
            "actual": float(actual),
            "expected": float(expected),
        }

    def vector(actual: Iterable[float], expected: Iterable[float]) -> dict[str, Any]:
        measured = np.asarray(actual, dtype=float)
        reference = np.asarray(expected, dtype=float)
        return {
            "status": "PASS" if measured.shape == reference.shape and np.all(np.abs(measured - reference) <= tolerance) else "FAIL",
            "actual": measured.tolist(),
            "expected": reference.tolist(),
        }

    checks = {
        "chassis_size_xyz": vector(2.0 * chassis.half_extents, [2.1, 1.5, 0.6]),
        "transverse_size_xy": vector(2.0 * transverse.half_extents[:2], [0.7, 1.5]),
        "longitudinal_size_xy": vector(2.0 * longitudinal.half_extents[:2], [2.8, 0.7]),
        "transverse_surface_z": scalar(_interval(world["conveyor_transverse"], 2)[1], 0.6),
        "longitudinal_surface_z": scalar(_interval(world["conveyor_longitudinal"], 2)[1], 0.6),
        "transverse_front_x": scalar(_interval(world["conveyor_transverse"], 0)[1], -0.2),
        "longitudinal_front_x": scalar(_interval(world["conveyor_longitudinal"], 0)[1], -0.2),
        "fixed_assembly_x_extent": scalar(assembly_bounds["x"][1] - assembly_bounds["x"][0], 2.8),
        "fixed_assembly_y_extent": scalar(assembly_bounds["y"][1] - assembly_bounds["y"][0], 2.2),
        "chassis_transverse_join": _contact_check(chassis, transverse, 0, 1, tolerance),
        "chassis_longitudinal_join": _contact_check(chassis, longitudinal, 1, -1, tolerance),
        "transverse_longitudinal_join": _contact_check(transverse, longitudinal, 1, -1, tolerance),
        "chassis_floor_contact": {"status": "PASS" if abs(_interval(world["chassis"], 2)[0]) <= tolerance else "FAIL", "gap_m": _interval(world["chassis"], 2)[0]},
        "robot_mount_surface": {"status": "PASS" if abs(base[2] - _interval(world["chassis"], 2)[1]) <= tolerance else "FAIL", "gap_m": float(base[2] - _interval(world["chassis"], 2)[1])},
        "robot_base_origin_world": vector(base, [-1.325, 0.35, 0.6]),
        "robot_base_official_bbox_min": vector(base_min, [-0.3385, -0.275, 0.0]),
        "robot_base_official_bbox_max": vector(base_max, [0.225, 0.275, 0.245]),
        "robot_base_front_edge": {"status": "PASS" if abs(base_front - float(robot["base_front_edge_x_world_m"])) <= tolerance else "FAIL", "actual_x_m": base_front},
        "robot_base_support_footprint": {"status": "PASS" if all(base[i] + base_min[i] >= world["chassis"].center[i] - world["chassis"].half_extents[i] - tolerance and base[i] + base_max[i] <= world["chassis"].center[i] + world["chassis"].half_extents[i] + tolerance for i in (0, 1)) else "FAIL"},
        "left_side_clearance": {"status": "PASS" if abs(float(trailer["left_wall_y_m"]) - assembly_bounds["y"][1] - 0.05) <= tolerance else "FAIL", "clearance_m": float(trailer["left_wall_y_m"]) - assembly_bounds["y"][1]},
        "right_side_clearance": {"status": "PASS" if abs(assembly_bounds["y"][0] - float(trailer["right_wall_y_m"]) - 0.05) <= tolerance else "FAIL", "clearance_m": assembly_bounds["y"][0] - float(trailer["right_wall_y_m"])},
        "trailer_inner_width": scalar(trailer["inner_width_m"], 2.3),
        "trailer_wall_coordinates": vector([trailer["right_wall_y_m"], trailer["left_wall_y_m"]], [-1.15, 1.15]),
        "trailer_length_height_undefined": {
            "status": "PASS" if trailer["length_m"] is None and trailer["height_m"] is None else "FAIL",
            "length_m": trailer["length_m"],
            "height_m": trailer["height_m"],
        },
        "stack_to_conveyor_clearance": {"status": "PASS" if abs(float(layout.data["carton_stack"]["front_face_x_m"]) - assembly_bounds["x"][1] - 0.2) <= tolerance else "FAIL", "clearance_m": float(layout.data["carton_stack"]["front_face_x_m"]) - assembly_bounds["x"][1]},
        "stack_count": {"status": "PASS" if len(stack) == int(layout.data["carton_stack"]["width_columns"]) * int(layout.data["carton_stack"]["height_layers"]) else "FAIL", "count": len(stack)},
        "carton_size_xyz": vector(2.0 * stack[0].half_extents, [0.6, 0.4, 0.3]),
        "stack_width": scalar(float(np.ptp(stack_corners[:, 1])), 2.08),
        "stack_height": scalar(float(np.ptp(stack_corners[:, 2])), 2.4),
    }
    return {
        "schema": "m710id70_layout_numeric_audit_v1",
        "layout_id": layout.data["layout_id"],
        "layout_fingerprint": layout.layout_fingerprint,
        "assembly_world_bounds_m": assembly_bounds,
        "checks": checks,
        "overall_status": "PASS" if all(item["status"] == "PASS" for item in checks.values()) else "FAIL",
    }


def robot_chassis_support_contact_allowed(link: OBB, chassis: OBB, tolerance: float) -> bool:
    """Allow only a nonpenetrating mounting-plane contact inside the chassis."""
    bottom = float(np.min(link.corners()[:, 2]))
    top = float(np.max(chassis.corners()[:, 2]))
    footprint_inside = all(
        np.min(link.corners()[:, axis]) >= np.min(chassis.corners()[:, axis]) - tolerance
        and np.max(link.corners()[:, axis]) <= np.max(chassis.corners()[:, axis]) + tolerance
        for axis in (0, 1)
    )
    return footprint_inside and bottom >= top - tolerance and not link.intersects_obb(chassis, margin=-tolerance)


def _audit_initial_state_prepared(
    config: LayoutValidationConfig,
    robot: URDFRobot,
    shapes: list[tuple[str, np.ndarray, np.ndarray]],
    fixed: list[OBB],
    cartons: tuple[OBB, ...],
    q: np.ndarray,
) -> dict[str, Any]:
    layout = config.layout
    failures: list[dict[str, Any]] = []
    effective = SimulationCollisionPolicy.from_mapping(config.data.get("collision_policy"))
    if q.shape != (6,) or not np.all(np.isfinite(q)) or not robot.within_limits(q):
        failures.append({"reason": "JOINT_LIMIT_OR_NONFINITE"})
    else:
        margin = config.collision_margin_m
        tolerance = config.contact_tolerance_m
        joint_margin = float(config.data["collision"]["joint_margin_rad"])
        actual_joint_margin = float(np.min(np.minimum(q - robot.joint_limits[:, 0], robot.joint_limits[:, 1] - q)))
        if actual_joint_margin < joint_margin:
            failures.append({"reason": "JOINT_MARGIN", "actual_margin_rad": actual_joint_margin, "required_margin_rad": joint_margin})
        self_result = robot.collision_result(q, [], check_self=True)
        if self_result.in_collision:
            failures.append({"reason": "SELF_COLLISION", "detail": str(self_result)})
        links = world_link_boxes(robot, q, shapes)
        obstacles = [*fixed, *cartons]
        chassis = next(box for box in fixed if box.name == "chassis")
        for link in links:
            for obstacle in obstacles:
                if not link.intersects_obb(obstacle, margin=margin):
                    continue
                if link.name in {"base_link", "J1_link"} and obstacle.name == "chassis" and robot_chassis_support_contact_allowed(link, chassis, tolerance):
                    continue
                failures.append({"reason": "ROBOT_COLLISION", "pair": [link.name, obstacle.name]})
        tools = robot.tool_collision_obbs(q)
        legacy_tool = robot.tool_collision_obb(q)
        if not tools and legacy_tool is not None:
            tools = [legacy_tool]
        for tool in tools:
            for obstacle in obstacles:
                if tool.intersects_obb(obstacle, margin=margin):
                    failures.append({"reason": "TOOL_COLLISION", "pair": [tool.name, obstacle.name]})
            for link in links:
                if link.name not in effective.wrist_tool_exempt_links and tool.intersects_obb(link, margin=margin):
                    failures.append({"reason": "TOOL_SELF_COLLISION", "pair": [tool.name, link.name]})
        y_min = float(layout.data["trailer"]["right_wall_y_m"])
        y_max = float(layout.data["trailer"]["left_wall_y_m"])
        for body in [*links, *tools]:
            corners = body.corners()
            if float(np.min(corners[:, 1])) < y_min + margin or float(np.max(corners[:, 1])) > y_max - margin:
                failures.append({"reason": "TRAILER_SIDE_CLEARANCE", "body": body.name, "y_bounds_m": [float(np.min(corners[:, 1])), float(np.max(corners[:, 1]))]})
    unique = list({json.dumps(item, sort_keys=True): item for item in failures}.values())
    return {
        "schema": "m710id70_initial_state_audit_v1",
        "layout_fingerprint": layout.layout_fingerprint,
        "q_rad": q.tolist(),
        "collision_policy": effective.to_mapping(),
        "status": "PASS" if not unique else "FAIL",
        "failures": unique,
        "known_geometry_scope": "side_wall_planes_floor_fixed_assembly_carton_stack_official_robot_mesh_broadphase_and_source_audited_rigid_tool_solids",
        "complete_workcell_clearance": "KNOWN_GEOMETRY_EVALUATED_TRAILER_LENGTH_AND_HEIGHT_UNDEFINED",
    }


def audit_initial_state(config: LayoutValidationConfig, q: np.ndarray | None = None) -> dict[str, Any]:
    robot = config.layout.robot()
    return _audit_initial_state_prepared(
        config,
        robot,
        urdf_collision_shapes(robot),
        list(config.layout.fixed_components()),
        config.layout.cartons(),
        config.initial_q if q is None else np.asarray(q, dtype=float),
    )


def find_initial_state_witness(config: LayoutValidationConfig, maximum_draws: int = 3000) -> dict[str, Any]:
    """Find the first deterministic valid initial state without moving the layout."""
    if isinstance(maximum_draws, bool) or maximum_draws <= 0:
        raise ValueError("maximum_draws must be a positive integer")
    layout = config.layout
    robot = layout.robot()
    shapes = urdf_collision_shapes(robot)
    fixed = list(layout.fixed_components())
    cartons = layout.cartons()
    seed = int(config.data["initial_state"]["seed"])
    rng = np.random.default_rng(seed)
    joint_margin = float(config.data["collision"]["joint_margin_rad"])
    lower = robot.joint_limits[:, 0] + joint_margin
    upper = robot.joint_limits[:, 1] - joint_margin
    candidates = [np.asarray(config.data["initial_state"]["previous_v3_q_rad"], dtype=float), np.zeros(6), 0.5 * (lower + upper)]
    candidates.extend(rng.uniform(lower, upper) for _ in range(maximum_draws))
    last = None
    for draw, q in enumerate(candidates):
        audit = _audit_initial_state_prepared(config, robot, shapes, fixed, cartons, np.asarray(q, dtype=float))
        last = audit
        if audit["status"] == "PASS":
            return {
                "status": "PASS",
                "seed": seed,
                "draw": draw,
                "random_draw": max(0, draw - 2),
                "maximum_random_draws": maximum_draws,
                "q_rad": audit["q_rad"],
                "audit": audit,
                "path_from_previous_state": "NOT_EVALUATED",
            }
    return {
        "status": "FAIL",
        "seed": seed,
        "draw": None,
        "maximum_random_draws": maximum_draws,
        "last_audit": last,
    }


def build_scene_snapshot(config: LayoutValidationConfig, q: np.ndarray | None = None) -> dict[str, Any]:
    layout = config.layout
    q = config.initial_q if q is None else np.asarray(q, dtype=float)
    initial = audit_initial_state(config, q)
    if initial["status"] != "PASS":
        raise ValueError(f"refusing to snapshot invalid initial state: {initial['failures']}")
    robot = layout.robot()
    robot_link_obbs = world_link_boxes(robot, q, urdf_collision_shapes(robot))
    rigid_tool_obbs = robot.tool_collision_obbs(q)
    tool_display_envelope = _tool_compound_envelope(robot, q)
    fixed = [_obb_record(box) for box in layout.fixed_components()]
    cartons = [_obb_record(box) for box in layout.cartons()]
    mounting = layout.data["robot"]
    tool_load = load_tool_config(_resolve(layout.config_path, str(layout.data["tool"]["load_config"])))
    tool_geometry = load_tool_config(_resolve(layout.config_path, str(layout.data["tool"]["geometry_config"])))
    from .tool_geometry import audit_tool_geometry
    tool_coverage = audit_tool_geometry(layout.config_path.parents[2])
    snapshot = {
        "schema": SNAPSHOT_SCHEMA,
        "layout_id": layout.data["layout_id"],
        "layout_fingerprint": layout.layout_fingerprint,
        "units": layout.data["units"],
        "world": layout.data["world"],
        "assembly": {
            "pose_world": layout.world_from_assembly.tolist(),
            "fixed_components": fixed,
            "designed_contacts": [
                ["chassis", "conveyor_transverse"],
                ["chassis", "conveyor_longitudinal"],
                ["conveyor_transverse", "conveyor_longitudinal"],
                ["chassis", "floor"],
                ["robot_mount", "chassis"],
            ],
        },
        "trailer": layout.data["trailer"],
        "cartons": cartons,
        "robot": {
            "model": "fanuc_m710id_70",
            "joint_names": list(robot.active_joint_names),
            "q_rad": q.tolist(),
            "world_from_mount": layout.robot_base_transform().tolist(),
            "mounting_reference": {
                "base_support_bbox_min_xyz_m": mounting["base_support_bbox_min_xyz_m"],
                "base_support_bbox_max_xyz_m": mounting["base_support_bbox_max_xyz_m"],
                "base_front_edge_x_world_m": mounting["base_front_edge_x_world_m"],
                "positioning_basis": mounting["positioning_basis"],
                "status": mounting["positioning_status"],
            },
            "urdf": layout.assets["robot_urdf"],
            "model_config": layout.assets["robot_model_config"],
            "flange_pose_world": robot.named_link_frames(q)["flange"].tolist(),
            "tcp_pose_world": robot.fk(q).tolist(),
            "link_collision_obbs": [_obb_record(box) for box in robot_link_obbs],
        },
        "tool": {
            "name": tool_load.name,
            "mass_kg": tool_load.mass_kg,
            "tcp_transform": tool_load.tcp_transform,
            "task_tcp_pose_world": robot.fk(q).tolist(),
            "collision_obb": _obb_record(
                tool_display_envelope,
                role="DISPLAY_AND_SNAPSHOT_ENVELOPE_NOT_EXECUTION_COLLISION",
            ),
            "rigid_collision_obbs": [_obb_record(box) for box in rigid_tool_obbs],
            "execution_collision_representation": "CAD_RIGID_STRUCTURES_AND_INSERTS_WITH_SEPARATE_FLEXIBLE_BELLOWS",
            "geometry": tool_geometry.data["geometry"],
            "geometry_status": "CONSERVATIVE_RIGID_SOLID_COVERAGE_PROVEN" if tool_coverage["coverage_verified"] else "COVERAGE_FAILED",
            "geometry_coverage": tool_coverage,
            "planning_tcp_status": layout.data["tool"]["planning_tcp_status"],
            "assets": {name: value for name, value in layout.assets.items() if name.startswith("tool_")},
        },
        "receiver": {"state": "EMPTY", "transport_capability": "PHYSICAL_CONVEYOR_SURFACES"},
        "attachments": [],
        "initial_state_audit": initial,
        "evidence": {
            "layout_source": layout.config_path.name,
            "approval_basis": layout.data["evidence"]["approval_basis"],
            "unconfirmed": layout.data["evidence"]["unconfirmed"],
            "legacy_104_task_population": "NOT_REINTERPRETED_WITH_LAYOUT_V1",
        },
    }
    snapshot["scene_fingerprint"] = canonical_digest(snapshot)
    return snapshot


def world_state_fingerprint(layout: WorkcellLayout, q: Iterable[float]) -> str:
    """Identity of the placed assembly and robot state, excluding render settings."""
    return canonical_digest(
        {
            "layout_fingerprint": layout.layout_fingerprint,
            "world_from_assembly": layout.world_from_assembly.tolist(),
            "fixed_component_poses": [box.world_from_local.tolist() for box in layout.fixed_components()],
            "q_rad": np.asarray(q, dtype=float).tolist(),
        }
    )


def audit_snapshot_consistency(config: LayoutValidationConfig, snapshot: Mapping[str, Any]) -> dict[str, Any]:
    """Compare a frozen snapshot to the same CPU geometry used by planning."""
    verification = verify_scene_snapshot(snapshot)
    layout = config.layout
    robot = layout.robot()
    q = np.asarray(snapshot["robot"]["q_rad"], dtype=float)
    fixed_expected = {box.name: box.world_from_local for box in layout.fixed_components()}
    fixed_actual = {item["name"]: np.asarray(item["pose_world"], dtype=float) for item in snapshot["assembly"]["fixed_components"]}
    carton_expected = {box.name: box.world_from_local for box in layout.cartons()}
    carton_actual = {item["name"]: np.asarray(item["pose_world"], dtype=float) for item in snapshot["cartons"]}
    link_expected = {
        box.name: box.world_from_local
        for box in world_link_boxes(robot, q, urdf_collision_shapes(robot))
    }
    link_actual = {
        item["name"]: np.asarray(item["pose_world"], dtype=float)
        for item in snapshot["robot"]["link_collision_obbs"]
    }
    errors = {
        "fixed_component_pose_max_abs": max(float(np.max(np.abs(fixed_actual[name] - pose))) for name, pose in fixed_expected.items()),
        "carton_pose_max_abs": max(float(np.max(np.abs(carton_actual[name] - pose))) for name, pose in carton_expected.items()),
        "robot_mount_pose_max_abs": float(np.max(np.abs(np.asarray(snapshot["robot"]["world_from_mount"]) - layout.robot_base_transform()))),
        "flange_pose_max_abs": float(np.max(np.abs(np.asarray(snapshot["robot"]["flange_pose_world"]) - robot.named_link_frames(q)["flange"]))),
        "tcp_pose_max_abs": float(np.max(np.abs(np.asarray(snapshot["robot"]["tcp_pose_world"]) - robot.fk(q)))),
        "tool_collision_pose_max_abs": float(
            np.max(np.abs(np.asarray(snapshot["tool"]["collision_obb"]["pose_world"]) - _tool_compound_envelope(robot, q).world_from_local))
        ),
        "tool_rigid_compound_pose_max_abs": max(
            float(np.max(np.abs(np.asarray(actual["pose_world"]) - expected.world_from_local)))
            for actual, expected in zip(snapshot["tool"]["rigid_collision_obbs"], robot.tool_collision_obbs(q))
        ),
        "robot_link_pose_max_abs": max(
            float(np.max(np.abs(link_actual[name] - pose)))
            for name, pose in link_expected.items()
        ),
    }
    passed = (
        snapshot["layout_fingerprint"] == layout.layout_fingerprint
        and set(fixed_actual) == set(fixed_expected)
        and set(carton_actual) == set(carton_expected)
        and set(link_actual) == set(link_expected)
        and max(errors.values()) <= 1e-12
    )
    return {
        "schema": "m710id70_snapshot_consistency_v1",
        "status": "PASS" if passed else "FAIL",
        "snapshot_verification": verification,
        "maximum_absolute_errors": errors,
        "cpu_geometry_source": "workcell_layout_snapshot_v1",
        "replay_geometry_source": "same_frozen_snapshot",
        "receiver_transport": "NOT_IMPLEMENTED_FOR_LAYOUT_V1",
    }


def verify_scene_snapshot(snapshot: Mapping[str, Any], project_root: str | Path | None = None) -> dict[str, Any]:
    snapshot = dict(snapshot)
    if snapshot.get("schema") != SNAPSHOT_SCHEMA:
        raise ValueError("unsupported scene snapshot schema")
    recorded = snapshot.pop("scene_fingerprint", None)
    actual = canonical_digest(snapshot)
    if recorded != actual:
        raise ValueError("scene snapshot fingerprint mismatch")
    checked_assets = []
    if project_root is not None:
        root = Path(project_root).resolve()
        asset_records = [snapshot["robot"]["urdf"], snapshot["robot"]["model_config"], *snapshot["tool"]["assets"].values()]
        for record in asset_records:
            declared = Path(record["repository_path"])
            path = root / declared
            if not path.is_file() or sha256_file(path) != record["sha256"]:
                raise ValueError(f"scene snapshot asset mismatch: {record['repository_path']}")
            checked_assets.append(record["repository_path"])
    return {"status": "PASS", "scene_fingerprint": recorded, "checked_assets": checked_assets}


def write_scene_snapshot(config: LayoutValidationConfig, path: str | Path) -> dict[str, Any]:
    snapshot = build_scene_snapshot(config)
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(snapshot, indent=2, ensure_ascii=False), encoding="utf-8")
    return snapshot
