"""Freeze the actual g700 environment independently of a replacement robot.

An unknown robot mount transform is not identity. Pending model definitions can
carry a configuration identity, but cannot carry an accepted full-scene hash or
load the FANUC robot as a substitute.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, replace
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .conveyor_lift import coupled_conveyor_state
from .gap_search_layout import BASELINE, GapCandidate, candidate_data
from .identity import load_tool_config
from .tool_connector_extension import connector_extension_m, extended_connector_boxes
from .workcell_layout import (
    WorkcellLayout, canonical_digest, compute_layout_fingerprint, load_workcell_layout,
    _rigid_tool_compound_boxes, _compliant_tool_compound_boxes,
)

BASELINE_COMMIT = "a3f08cc2283138e5905b4cc9829cefd7b4aac6b7"
BASELINE_REPORT = "docs/experiments/m710_gap700_front_working_20261110"
MODEL_ID = "huayan_s50_2000"
HEIGHT_SAMPLES_M = (.55, .6, 1., 1.5)
SOURCE_FILES = (
    f"{BASELINE_REPORT}/README.md",
    f"{BASELINE_REPORT}/front-working-search-launch.json",
    f"{BASELINE_REPORT}/front-working-home-checked.json",
    "src/unloading_sim/gap_search_layout.py",
)


def _root(root=None):
    return Path(root).resolve() if root else Path(__file__).resolve().parents[2]


def _json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _transform(value, name):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0., 0., 0., 1.], atol=1e-12, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-10, rtol=0)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-10, rtol=0)):
        raise ValueError(f"INVALID_RIGID_TRANSFORM:{name}")
    return t


def frozen_baseline_layout(root=None):
    """Reconstruct the checked g700 runtime, not the historical YAML default."""
    root = _root(root)
    for name in SOURCE_FILES:
        if not (root / name).is_file():
            raise ValueError(f"FROZEN_BASELINE_SOURCE_MISSING:{name}")
    launch = _json(root / SOURCE_FILES[1]);home = _json(root / SOURCE_FILES[2])
    if (launch["gap_m"] != .7 or launch["mount_height_m"] != .9
            or launch["fresh_initial_carton_count"] != 40 or launch["imported_completed_count"] != 0
            or launch["assumed_removed"] or home["initial_carton_count"] != 40
            or not home["accepted_initial_state"] or home["status"] != "PASS"):
        raise ValueError("FROZEN_BASELINE_RUN_BINDING_CHANGED")
    raw = (root / SOURCE_FILES[2]).read_bytes()
    # Git text checkouts may normalize line endings; do not normalize content.
    if launch["initial_home_sha256"] not in {
            hashlib.sha256(raw).hexdigest(), hashlib.sha256(raw.replace(b"\r\n", b"\n")).hexdigest()}:
        raise ValueError("FROZEN_HOME_HASH_MISMATCH")
    if launch["initial_q_rad"] != home["selected"]["q_rad"]:
        raise ValueError("FROZEN_HOME_JOINT_IDENTITY_MISMATCH")
    original = load_workcell_layout(root / "configs/workcells" / BASELINE)
    data = candidate_data(original.data, GapCandidate(.7, .9))
    fingerprint = compute_layout_fingerprint(data, original.assets)
    if fingerprint != home["layout_fingerprint"]:
        # Asset digests include file bytes, so a Git CRLF checkout is accepted
        # only when every normalized text asset recovers the recorded identity.
        assets = deepcopy(original.assets)
        for item in assets.values():
            raw = (root / item["repository_path"]).read_bytes()
            if b"\0" not in raw:
                item["sha256"] = hashlib.sha256(raw.replace(b"\r\n", b"\n")).hexdigest()
        fingerprint = compute_layout_fingerprint(data, assets)
        if fingerprint != home["layout_fingerprint"]:
            raise ValueError("FROZEN_BASELINE_LAYOUT_FINGERPRINT_MISMATCH")
        original = replace(original, assets=assets)
    return replace(original, data=data, layout_fingerprint=fingerprint)


def _box_record(box):
    return dict(id=box.name, category=box.category, size_m=(2 * box.half_extents).tolist(),
                world_from_entity=box.world_from_local.tolist(),
                solid=deepcopy(getattr(box, "solid", None)))


def _tool_intrinsic(layout):
    root = layout.config_path.parents[2]
    tool = layout.data["tool"]
    load_path = (layout.config_path.parent / tool["load_config"]).resolve()
    geometry_path = (layout.config_path.parent / tool["geometry_config"]).resolve()
    load = load_tool_config(load_path);geometry = load_tool_config(geometry_path)
    extension = connector_extension_m(tool)
    rigid = extended_connector_boxes(_rigid_tool_compound_boxes(geometry.data, root), extension)
    compliant = _compliant_tool_compound_boxes(geometry.data, root)
    frame_contract = yaml.safe_load((root / "configs/validation/m710id70_proof_of_concept.yaml").read_text(encoding="utf-8"))["tool_frame_contract"]
    frames = {}
    for key in ("T_flange_virtual_task_tcp", "T_flange_uncompressed_cup_plane", "T_flange_nominal_compressed_contact"):
        t = _transform(frame_contract[key], key).copy();t[0, 3] += extension
        frames[key.replace("T_flange", "T_tool_mount_reference")] = t.tolist()
    masses = yaml.safe_load((root / "configs/simulation/m710id70_official_dynamics_v2.yaml").read_text(encoding="utf-8"))["cartons"]
    return dict(definition=deepcopy(tool), load=deepcopy(load.data), geometry=deepcopy(geometry.data),
        assets={k: deepcopy(v) for k, v in layout.assets.items() if k.startswith("tool_")},
        rigid_local_boxes_virtual_tcp_m=rigid.tolist(), compliant_local_boxes_virtual_tcp_m=compliant.tolist(),
        intrinsic_frames=frames, frame_convention="INHERITED_PHYSICAL_TOOL_MOUNT_REFERENCE_NOT_S50_URDF_FLANGE",
        new_robot_flange_adapter_required=True, connector_extension_m=extension,
        tool_mass_kg=float(load.mass_kg), carton_mass_kg=float(masses["mass_kg_each"]),
        robot_dependent_world_tool_pose_excluded=True)


def freeze_non_robot_scene(layout):
    """Serialize every environment collider and mechanism at fixed stroke samples.

    Configuration dimensions and mechanism definitions are also retained, so
    equal samples alone cannot hide changed strokes or dimensions.
    """
    root = layout.config_path.parents[2]
    cartons = tuple(layout.cartons())
    ids = sorted(b.name for b in cartons)
    if len(ids) != 40 or len(set(ids)) != 40:
        raise ValueError("REPLACEMENT_REQUIRES_FRESH_40_CARTONS")
    mount = (layout.physical_mount_transform() if hasattr(layout, "physical_mount_transform")
             else layout.robot_base_transform())
    fixed = {b.name: b for b in layout.fixed_components()}
    chassis_front = float(fixed["chassis"].corners()[:, 0].max())
    stack_front = min(float(b.corners()[:, 0].min()) for b in cartons)
    transverse_front = float(fixed["conveyor_transverse"].corners()[:, 0].max())
    home = _json(root / SOURCE_FILES[2])
    result = dict(schema="robot_replacement_frozen_environment_v1", baseline_commit=BASELINE_COMMIT,
        parameters=dict(gap_m=stack_front-transverse_front, mounting_surface_height_m=float(mount[2, 3]),
            world_from_physical_mount=mount.tolist(), mount_axis_to_chassis_front_m=chassis_front-float(mount[0, 3]),
            minimum_belt_height_m=float(layout.data["conveyors"]["coupling"]["minimum_surface_height_world_m"]),
            maximum_belt_height_m=float(layout.data["conveyors"]["coupling"]["maximum_surface_height_world_m"]),
            initial_belt_height_m=float(layout.data["assembly"]["world_xyz_m"][2]+layout.data["conveyors"]["surface_z_a_m"]),
            tool_extension_m=connector_extension_m(layout.data["tool"])),
        layout_fields={k: deepcopy(layout.data[k]) for k in ("units", "world", "assembly", "chassis", "conveyors", "trailer", "carton_stack", "tool")},
        initial_carton_ids=ids, initial_cartons=[_box_record(b) for b in sorted(cartons, key=lambda b: b.name)],
        imported_completed_count=0, assumed_removed=[], initial_robot_q=None,
        initial_physical_contact_target=deepcopy(home["requested_physical_pose_world"]),
        target_orientation_source=SOURCE_FILES[2]+"#requested_physical_pose_world",
        initial_reference_fk_contact_pose=deepcopy(home["selected"]["stack_facing"]["physical_pose_world"]),
        tool_intrinsic=_tool_intrinsic(layout), height_samples=[])
    for height in HEIGHT_SAMPLES_M:
        state = coupled_conveyor_state(layout, height, cartons)
        result["height_samples"].append(dict(height_m=height,
            entities=[_box_record(b) for b in sorted(state.boxes, key=lambda b: b.name)],
            mechanism=deepcopy(state.mechanism)))
    result["environment_fingerprint"] = canonical_digest(result)
    return result


def _differences(a, b, path=""):
    if isinstance(a, Mapping) and isinstance(b, Mapping):
        for key in sorted(set(a) | set(b)):
            item = f"{path}/{key}"
            if key not in a or key not in b:
                yield dict(path=item, reason="MISSING_FIELD")
            else:
                yield from _differences(a[key], b[key], item)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            yield dict(path=path, reason="LENGTH_CHANGED", before=len(a), after=len(b))
        for i, (x, y) in enumerate(zip(a, b)):
            yield from _differences(x, y, f"{path}/{i}")
    elif a != b:
        yield dict(path=path, reason="VALUE_CHANGED", before=a, after=b)


def compare_non_robot_scenes(before, after):
    a = deepcopy(before);b = deepcopy(after)
    expected_a = a.pop("environment_fingerprint", None);expected_b = b.pop("environment_fingerprint", None)
    if canonical_digest(a) != expected_a or canonical_digest(b) != expected_b:
        raise ValueError("FROZEN_ENVIRONMENT_FINGERPRINT_MISMATCH")
    differences = list(_differences(a, b))
    return dict(status="PASS" if not differences else "NON_ROBOT_LAYOUT_CHANGED", passed=not differences,
        differences=differences, before_environment_fingerprint=expected_a, after_environment_fingerprint=expected_b,
        sampled_heights_m=[s["height_m"] for s in a["height_samples"]],
        checked_entities_per_height=[len(s["entities"]) for s in a["height_samples"]],
        initial_carton_count=len(a["initial_carton_ids"]), model_readiness_evaluated=False,
        tool_world_pose_compared=False, scope="NON_ROBOT_GEOMETRY_STROKE_AND_INTRINSIC_TOOL_ONLY")


def pending_s50_definition():
    return dict(model_id=MODEL_ID, dof=6, binding=None, model_asset_identity={},
        mounting_from_urdf_base=None, base_support_bounds=None, planning_ready=False)


def _validated_ready_model(definition, root):
    """Bind readiness to the current model files, not just a caller's boolean."""
    from .robot_model_binding import RobotModelBinding

    model_config = definition.get("model_config")
    if not model_config:
        raise ValueError("S50_MODEL_NOT_READY:MISSING_MODEL_CONFIG")
    path = Path(model_config)
    if not path.is_absolute():
        path = root / path
    binding = RobotModelBinding.from_config(path)
    if binding.model_id != definition["model_id"]:
        raise ValueError("REPLACEMENT_MODEL_BINDING_ID_MISMATCH")
    if binding.identity() != definition["binding"]:
        raise ValueError("REPLACEMENT_MODEL_BINDING_IDENTITY_CHANGED")
    if not np.allclose(binding.mounting_from_base, definition["mounting_from_urdf_base"], atol=1e-12, rtol=0):
        raise ValueError("REPLACEMENT_MOUNT_ALIGNMENT_CHANGED")
    adapter = (binding.data.get("tool_interface") or {}).get("flange_from_project_tool_mount")
    if adapter is None:
        raise ValueError("S50_MODEL_NOT_READY:MISSING_PHYSICAL_TOOL_MOUNT_ADAPTER")
    _transform(adapter, "flange_from_project_tool_mount")
    # Include actual visual and collision assets. Metadata about a STEP source
    # is insufficient to invalidate plans after a converted mesh changes.
    files = {"config": binding.config_path, "urdf": binding.urdf_path, "srdf": binding.srdf_path}
    for mesh in ET.parse(binding.urdf_path).getroot().iter("mesh"):
        uri = mesh.get("filename", "")
        files["mesh:" + uri] = binding.resolve_mesh(uri)
    return {name: hashlib.sha256(file.read_bytes()).hexdigest() for name, file in sorted(files.items())}


@dataclass(frozen=True)
class RobotReplacementLayout:
    reference_layout: WorkcellLayout
    definition: Mapping[str, Any]
    data: Mapping[str, Any]
    configuration_identity: str
    full_scene_fingerprint: str | None

    @property
    def config_path(self):return self.reference_layout.config_path

    @property
    def world_from_assembly(self):return self.reference_layout.world_from_assembly

    @property
    def assembly_from_world(self):return self.reference_layout.assembly_from_world

    @property
    def layout_fingerprint(self):return self.full_scene_fingerprint

    @property
    def assets(self):
        return {**{k: deepcopy(v) for k, v in self.reference_layout.assets.items() if not k.startswith("robot_")},
                "replacement_robot": deepcopy(self.definition["model_asset_identity"])}

    def physical_mount_transform(self):return self.reference_layout.robot_base_transform().copy()

    def robot_base_transform(self):
        if self.definition.get("mounting_from_urdf_base") is None:
            raise ValueError("S50_MODEL_NOT_READY:MISSING_MOUNT_FRAME_ALIGNMENT")
        return self.physical_mount_transform() @ _transform(self.definition["mounting_from_urdf_base"], "mounting_from_urdf_base")

    def fixed_components(self):return self.reference_layout.fixed_components()

    def cartons(self):return self.reference_layout.cartons()

    def component_local_boxes(self):return self.reference_layout.component_local_boxes()

    def trailer_boundary_boxes(self):return self.reference_layout.trailer_boundary_boxes()

    def robot(self):
        if not self.definition.get("planning_ready") or self.full_scene_fingerprint is None:
            raise ValueError("S50_MODEL_NOT_READY:MODEL_BINDING_NOT_QUALIFIED")
        model_config = self.definition.get("model_config")
        if not model_config:
            raise ValueError("S50_MODEL_NOT_READY:MISSING_MODEL_CONFIG")
        path = Path(model_config)
        if not path.is_absolute():path = self.config_path.parents[2] / path
        from .robot_model_binding import RobotModelBinding
        from .robot import URDFRobot
        binding = RobotModelBinding.from_config(path)
        if binding.model_id != self.definition["model_id"]:
            raise ValueError("REPLACEMENT_MODEL_BINDING_ID_MISMATCH")
        if not np.allclose(binding.mounting_from_base, self.definition["mounting_from_urdf_base"], atol=1e-12, rtol=0):
            raise ValueError("REPLACEMENT_MOUNT_ALIGNMENT_CHANGED")
        current_files = _validated_ready_model(self.definition, self.config_path.parents[2])
        if current_files != self.definition["verified_runtime_asset_sha256"]:
            raise ValueError("REPLACEMENT_RUNTIME_MODEL_ASSET_CHANGED")
        config = yaml.safe_load(path.read_text(encoding="utf-8"))
        adapter = config.get("tool_interface", {}).get("flange_from_project_tool_mount")
        if adapter is None:
            raise ValueError("S50_MODEL_NOT_READY:MISSING_PHYSICAL_TOOL_MOUNT_ADAPTER")
        adapter = _transform(adapter, "flange_from_project_tool_mount")
        tool = _tool_intrinsic(self)
        bare = URDFRobot.from_config(path, mounting_world_transform=self.physical_mount_transform())
        # This derives the fixed flange-to-tip relation; zero is not an
        # accepted initial state and is never exported as an initial q.
        frames = bare.named_link_frames(np.zeros(6))
        tip_from_tcp = (np.linalg.inv(frames[binding.tip_link]) @ frames[binding.flange_link]
                        @ adapter @ np.asarray(tool["intrinsic_frames"]["T_tool_mount_reference_virtual_task_tcp"]))
        robot = URDFRobot.from_config(path, mounting_world_transform=self.physical_mount_transform(),
            tip_from_tcp=tip_from_tcp, tool_collision_local_boxes=tool["rigid_local_boxes_virtual_tcp_m"])
        robot.tool_compliant_collision_local_boxes = np.asarray(tool["compliant_local_boxes_virtual_tcp_m"])
        robot.tool_connector_extension_m = tool["connector_extension_m"]
        robot.joint_search_mount = deepcopy(self.data["gap_search"])
        return robot


def build_replacement_layout(definition=None, *, root=None):
    reference = frozen_baseline_layout(root)
    definition = deepcopy(pending_s50_definition() if definition is None else dict(definition))
    required = set(pending_s50_definition())
    if required-set(definition) or definition["dof"] != 6 or not definition["model_id"]:
        raise ValueError("INCOMPLETE_REPLACEMENT_MODEL_DEFINITION")
    if definition["mounting_from_urdf_base"] is not None:
        definition["mounting_from_urdf_base"] = _transform(definition["mounting_from_urdf_base"], "mounting_from_urdf_base").tolist()
    data = deepcopy(reference.data)
    # Preserve physical installation coordinates only. Never retain the FANUC
    # shell footprint, front extent, model path, positioning basis or inertials.
    data["robot"] = dict(model=definition["model_id"], dof=6,
        base_origin_xy_a_m=[.6, -.2], mounting_surface_z_a_m=.9,
        mounting_from_urdf_base=deepcopy(definition["mounting_from_urdf_base"]),
        base_support_bounds=deepcopy(definition["base_support_bounds"]),
        planning_ready=bool(definition["planning_ready"]))
    ready = bool(definition["planning_ready"])
    if ready and (not definition["binding"] or not definition["model_asset_identity"]
            or definition["mounting_from_urdf_base"] is None or definition["base_support_bounds"] is None):
        raise ValueError("S50_MODEL_NOT_READY:INCOMPLETE_GEOMETRY_BINDING")
    if ready:
        definition["verified_runtime_asset_sha256"] = _validated_ready_model(definition, _root(root))
    if definition["base_support_bounds"] is not None:
        bounds = np.asarray(definition["base_support_bounds"], dtype=float)
        if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[1] <= bounds[0]):
            raise ValueError("INVALID_REPLACEMENT_BASE_BOUNDS")
        if definition["mounting_from_urdf_base"] is None:
            raise ValueError("BASE_BOUNDS_REQUIRE_MOUNT_FRAME_ALIGNMENT")
        t = reference.robot_base_transform() @ np.asarray(definition["mounting_from_urdf_base"])
        corners = np.array([[x, y, z] for x in bounds[:, 0] for y in bounds[:, 1] for z in bounds[:, 2]])
        world = corners @ t[:3, :3].T + t[:3, 3]
        data["robot"]["derived_base_front_edge_x_world_m"] = float(world[:, 0].max())
    environment = freeze_non_robot_scene(reference)
    identity = canonical_digest(dict(schema="robot_replacement_configuration_v1",
        environment_fingerprint=environment["environment_fingerprint"], definition=definition))
    full = canonical_digest(dict(schema="robot_replacement_complete_model_scene_v1", configuration=identity)) if ready else None
    if full == reference.layout_fingerprint:
        raise ValueError("REPLACEMENT_MUST_CHANGE_FULL_SCENE_FINGERPRINT")
    return RobotReplacementLayout(reference, definition, data, identity, full)
