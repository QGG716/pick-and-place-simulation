"""Backend-neutral command bundles for Isaac Sim trajectory validation.

This module deliberately imports no Isaac Sim, USD, CUDA, or ROS packages.
It converts a collision-checked geometric plan into a deterministic controller
command schedule that a heavyweight simulator adapter can consume.
"""

from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass, replace
from typing import Any, Mapping, Sequence

import numpy as np

from .geometry import OBB, rotation_matrix_from_rpy
from .identity import normalize_robot_model_id
from .independent_cups import (
    HOLDING_CAPACITY_ASSUMPTION,
    IDEAL_INDEPENDENT_CUPS_MODE,
    M710_CUP_COUNT,
    build_m710_independent_cup_array,
)
from .m710_replay_contract import (
    add_bundle_payload_sha256,
    build_m710_replay_contract,
)
from .scene import build_trailer_walls
from .timing import (
    motion_limits_from_config,
    scale_timed_trajectory_window,
    time_parameterize_joint_path,
)


FANUC_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
SUPPORTED_FANUC_REPLAY_MODELS = frozenset({"fanuc_m20id_35", "fanuc_m710id_70"})
OFFICIAL_M710_DESCRIPTION_REPOSITORY = "https://github.com/FANUC-CORPORATION/fanuc_description"
OFFICIAL_M710_DESCRIPTION_COMMIT = "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2"


def _lowercase_sha256(value: Any, name: str) -> str:
    result = str(value or "")
    if len(result) != 64 or any(character not in "0123456789abcdef" for character in result):
        raise ValueError(f"{name} must be a lowercase SHA-256")
    return result


def _validated_bool_mask(value: Any, name: str, count: int) -> tuple[bool, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != count or any(
        type(item) is not bool for item in value
    ):
        raise ValueError(f"{name} must contain exactly {count} booleans")
    return tuple(value)


def _validated_ideal_cup_selection(
    segment: Mapping[str, Any], *, physical_cup_count: int, path: np.ndarray
) -> dict[str, Any]:
    """Validate planner/FK cup evidence without calling it runtime contact.

    The planner's third mask proves contact at its strict FK endpoint.  Isaac
    must recompute a distinct ``actual_contact_mask`` from the simulated state
    before creating any constraint, so this function deliberately renames the
    incoming value to ``planned_fk_contact_mask``.
    """

    contact = segment.get("contact")
    authoritative = (
        contact.get("cup_selection") if isinstance(contact, Mapping) else None
    )
    aliases = [
        value
        for value in (
            segment.get("ideal_independent_cups"),
            segment.get("independent_cup_selection"),
        )
        if value is not None
    ]
    if authoritative is not None and any(alias != authoritative for alias in aliases):
        raise ValueError(
            "legacy independent-cup aliases must exactly equal contact.cup_selection"
        )
    raw = authoritative if authoritative is not None else (aliases[0] if aliases else None)
    if not isinstance(raw, Mapping):
        raise ValueError(
            "ideal_independent_cups requires planner evidence from the strict actual-FK contact pose"
        )
    if physical_cup_count != M710_CUP_COUNT:
        raise ValueError("ideal_independent_cups requires the fixed 72-cup physical array")
    target_id = str(raw.get("target_id", "")).strip()
    if target_id != str(segment.get("target", "")).strip():
        raise ValueError("independent-cup evidence target differs from the trajectory target")
    target_face = str(raw.get("target_face", "")).strip()
    if target_face not in {"front", "left", "right", "top"}:
        raise ValueError("independent-cup evidence requires a supported target face")
    pose_source = str(raw.get("pose_source", "")).strip()
    if "actual_fk" not in pose_source:
        raise ValueError("independent-cup evidence must be recomputed from the accepted actual FK")
    bit_order = raw.get("mask_bit_order_cup_ids")
    if not isinstance(bit_order, list) or len(bit_order) != physical_cup_count or any(
        not isinstance(item, str) or not item for item in bit_order
    ) or tuple(bit_order) != build_m710_independent_cup_array().cup_ids:
        raise ValueError(
            "independent-cup evidence requires the canonical 72 stable cup IDs"
        )
    eligible = _validated_bool_mask(
        raw.get("geometrically_eligible_mask"),
        "geometrically_eligible_mask",
        physical_cup_count,
    )
    commanded = _validated_bool_mask(
        raw.get("commanded_active_mask"),
        "commanded_active_mask",
        physical_cup_count,
    )
    planned_contact = _validated_bool_mask(
        raw.get("actual_contact_mask"),
        "planned actual_contact_mask",
        physical_cup_count,
    )
    if not any(commanded):
        raise ValueError("ideal_independent_cups requires a non-empty commanded cup mask")
    if any(active and not allowed for active, allowed in zip(commanded, eligible, strict=True)):
        raise ValueError("commanded cup mask must be a subset of geometrically eligible cups")
    if planned_contact != commanded:
        raise ValueError(
            "strict actual-FK contact mask must include every commanded cup"
        )
    actual_q = np.asarray(raw.get("actual_q_rad", []), dtype=float)
    grasp_index = int(segment.get("grasp_index", -1))
    if (
        actual_q.shape != (path.shape[1],)
        or not np.all(np.isfinite(actual_q))
        or grasp_index < 0
        or grasp_index >= len(path)
        or not np.allclose(actual_q, path[grasp_index], atol=1e-9, rtol=0.0)
    ):
        raise ValueError("independent-cup actual_q_rad must equal the selected grasp path endpoint")
    return {
        "suction_mode": IDEAL_INDEPENDENT_CUPS_MODE,
        "holding_capacity_assumption": HOLDING_CAPACITY_ASSUMPTION,
        "enforce_vacuum_force_capacity": False,
        "enforce_vacuum_break_force": False,
        "enforce_vacuum_break_torque": False,
        "load_bearing_minimum_cup_count": None,
        "target_id": target_id,
        "target_face": target_face,
        "pose_source": pose_source,
        "mask_bit_order_cup_ids": list(bit_order),
        "geometrically_eligible_mask": list(eligible),
        "commanded_active_mask": list(commanded),
        "planned_fk_contact_mask": list(planned_contact),
        "actual_contact_mask": [False] * physical_cup_count,
        "actual_contact_mask_source": "ISAAC_RUNTIME_ACTUAL_STATE_REQUIRED",
        "commanded_active_indices": [index for index, active in enumerate(commanded) if active],
        "actual_q_rad": actual_q.tolist(),
        "actual_virtual_task_tcp_pose_world": copy.deepcopy(
            raw.get("actual_virtual_task_tcp_pose_world")
        ),
    }


def _validated_union_support_evidence(support: Mapping[str, Any], place: Mapping[str, Any],
                                      pose: np.ndarray, segment: Mapping[str, Any],
                                      scene_primitives: Sequence[Mapping[str, Any]] | None) -> None:
    """Recompute a full footprint union, without inventing single-deck clearance."""
    from .conveyor_placement import support_union_audit

    actual_pose = np.asarray(support.get("actual_box_pose", []), dtype=float)
    half = np.asarray(support.get("payload_half_extents_m", []), dtype=float)
    if (actual_pose.shape != (4, 4) or not np.allclose(actual_pose, pose, atol=1e-12, rtol=0.)
            or half.shape != (3,) or not np.all(np.isfinite(half)) or np.any(half <= 0.)):
        raise ValueError("union support audit does not prove the selected actual box pose")
    records = support.get("support_obbs")
    if not isinstance(records, list) or not records:
        raise ValueError("union support audit requires its real support geometry")
    bodies = []
    for record in records:
        if not isinstance(record, Mapping):
            raise ValueError("union support geometry must be mappings")
        support_pose = np.asarray(record.get("pose_world", []), dtype=float)
        extent = np.asarray(record.get("half_extents_m", []), dtype=float)
        name = record.get("name")
        if (not isinstance(name, str) or not name or support_pose.shape != (4, 4)
                or not np.all(np.isfinite(support_pose))
                or not np.allclose(support_pose[3], [0., 0., 0., 1.], atol=1e-12, rtol=0.)
                or not np.allclose(support_pose[:3, :3].T @ support_pose[:3, :3], np.eye(3), atol=1e-12, rtol=0.)
                or not np.isclose(np.linalg.det(support_pose[:3, :3]), 1., atol=1e-12, rtol=0.)
                or extent.shape != (3,) or not np.all(np.isfinite(extent)) or np.any(extent <= 0.)):
            raise ValueError("union support geometry must contain finite rigid OBBs")
        bodies.append(OBB(support_pose[:3, 3], extent, support_pose[:3, :3], name,
                          str(record.get("category", "conveyor"))))
    names = [body.name for body in bodies]
    declared_names = place.get("support_names")
    if (len(names) != len(set(names)) or not isinstance(declared_names, list)
            or len(declared_names) != len(set(declared_names)) or set(names) != set(declared_names)):
        raise ValueError("union support geometry must match the retained contact-support identities")
    tolerance = float(support.get("tolerance_m", float("nan")))
    policy_tolerance = segment.get("validation", {}).get("contact_tolerance_m")
    if policy_tolerance is not None and not np.isclose(tolerance, float(policy_tolerance), atol=1e-12, rtol=0.):
        raise ValueError("union support tolerance differs from trajectory validation policy")
    payload = OBB(pose[:3, 3], half, pose[:3, :3], str(segment.get("target", "payload")), "carton")
    if scene_primitives is not None:
        scene = {item["name"]: item for item in scene_primitives}
        original_target = scene.get(payload.name)
        if original_target is None or not np.allclose(
                np.asarray(original_target["size_m"]), 2. * half, atol=1e-12, rtol=0.):
            raise ValueError("union support payload dimensions differ from frozen scene")
        for body in bodies:
            original = scene.get(body.name)
            if (original is None or original.get("dynamic") is not False
                    or not np.allclose(original["center_m"], body.center, atol=1e-12, rtol=0.)
                    or not np.allclose(original["size_m"], 2. * body.half_extents, atol=1e-12, rtol=0.)
                    or not np.allclose(original["rotation_matrix"], body.rotation, atol=1e-12, rtol=0.)):
                raise ValueError("union support geometry differs from frozen scene")
    recomputed = support_union_audit(payload, bodies, contact_tolerance_m=tolerance,
        edge_tolerance_m=float(support.get("edge_tolerance_m", float("nan"))),
        engineering_edge_margin_m=float(support.get("engineering_edge_margin_m", float("nan"))))
    if support.get("supported") is not True or recomputed["supported"] is not True:
        raise ValueError("union support geometry does not cover the actual full bottom footprint")
    for key in ("schema", "reason", "receiver_names", "coverage_method"):
        if support.get(key) != recomputed[key]:
            raise ValueError(f"union support audit {key} differs from recomputed geometry")
    for key in ("footprint_area_m2", "unsupported_area_m2", "bottom_z_range_m", "support_z_m", "plane_error_m"):
        value = np.asarray(support.get(key), dtype=float)
        expected = np.asarray(recomputed[key], dtype=float)
        if value.shape != expected.shape or not np.allclose(value, expected, atol=1e-12, rtol=0.):
            raise ValueError(f"union support audit {key} differs from recomputed geometry")
    bearing = place.get("load_bearing_support_names", [place["receiver"]])
    if (place["receiver"] not in bearing or not set(bearing).issubset(recomputed["receiver_names"])):
        raise ValueError("selected load-bearing receivers lack actual support area")


def _validated_place_evidence(segment: Mapping[str, Any], *,
                              scene_primitives: Sequence[Mapping[str, Any]] | None = None) -> dict[str, Any]:
    """Prefer the stage contract's real box pose and support evidence.

    Older replay fixtures expose three flat aliases.  When the structured
    contract exists those aliases may remain only as byte-for-byte-equivalent
    compatibility fields, so the adapter cannot silently replay a different
    receiving surface or release pose.
    """

    raw = segment.get("place")
    if raw is None:
        return {
            "place_surface": segment.get("place_surface"),
            "place_center_m": list(segment.get("place_center", [])),
            "release_center_m": list(segment.get("release_center", [])),
            "planned_support_audit": None,
            "actual_box_pose_world": None,
        }
    if not isinstance(raw, Mapping):
        raise ValueError("trajectory place evidence must be a mapping")
    receiver = str(raw.get("receiver", "")).strip()
    surface = str(raw.get("place_surface", "")).strip()
    if not receiver or surface != receiver:
        raise ValueError("place receiver and place_surface must name the same support")
    pose = np.asarray(raw.get("actual_box_pose_world", []), dtype=float)
    release_center = np.asarray(raw.get("release_center_world_m", []), dtype=float)
    if (
        pose.shape != (4, 4)
        or release_center.shape != (3,)
        or not np.all(np.isfinite(pose))
        or not np.all(np.isfinite(release_center))
        or not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0)
        or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-12, rtol=0.0)
        or not np.isclose(np.linalg.det(pose[:3, :3]), 1.0, atol=1e-12, rtol=0.0)
        or not np.allclose(release_center, pose[:3, 3], atol=1e-12, rtol=0.0)
    ):
        raise ValueError("place evidence requires one finite rigid actual box pose")
    support = raw.get("support")
    if not isinstance(support, Mapping):
        raise ValueError("place evidence requires a support audit mapping")
    if support.get("schema") == "complete_bottom_support_union_v1":
        _validated_union_support_evidence(support, raw, pose, segment, scene_primitives)
    else:
        # Historical single-deck evidence keeps its exact old contract. A union
        # is deliberately not forced into fictional per-axis edge clearances.
        edge_clearance = np.asarray(support.get("edge_clearance_xy_m", []), dtype=float)
        support_pose = np.asarray(support.get("actual_box_pose", []), dtype=float)
        bottom_gap = float(support.get("bottom_gap_m", float("nan")))
        penetration = float(support.get("penetration_m", float("nan")))
        if (
            support.get("supported") is not True
            or support.get("bottom_face_coplanar") is not True
            or edge_clearance.shape != (2,)
            or np.any(edge_clearance < 0.0)
            or not np.all(np.isfinite(edge_clearance))
            or not np.isfinite(bottom_gap)
            or not np.isfinite(penetration)
            or penetration < 0.0
            or support_pose.shape != (4, 4)
            or not np.allclose(support_pose, pose, atol=1e-12, rtol=0.0)
        ):
            raise ValueError("place support audit does not prove the selected actual box pose")
    for alias, expected in (
        ("place_surface", surface),
        ("place_center", release_center.tolist()),
        ("release_center", release_center.tolist()),
    ):
        if alias in segment and segment.get(alias) != expected:
            raise ValueError(f"legacy {alias} alias differs from structured place evidence")
    return {
        "place_surface": surface,
        "place_center_m": release_center.tolist(),
        "release_center_m": release_center.tolist(),
        "planned_support_audit": copy.deepcopy(dict(support)),
        "actual_box_pose_world": pose.tolist(),
    }


def _validated_official_model_manifest(value: Any) -> dict[str, Any]:
    """Validate the inline, fixed-commit FANUC model data consumed by Isaac."""

    if not isinstance(value, Mapping):
        raise ValueError("official FANUC model manifest must be an inline mapping")
    manifest = copy.deepcopy(dict(value))
    if manifest.get("schema_version") != "fanuc_official_description_provenance_v1":
        raise ValueError("unsupported official FANUC model manifest schema")
    upstream = manifest.get("upstream")
    if not isinstance(upstream, Mapping):
        raise ValueError("official FANUC manifest requires upstream provenance")
    repository_url = str(upstream.get("repository_url", "")).rstrip("/")
    if repository_url.removesuffix(".git") != OFFICIAL_M710_DESCRIPTION_REPOSITORY:
        raise ValueError("official FANUC manifest repository is not authoritative")
    if upstream.get("commit") != OFFICIAL_M710_DESCRIPTION_COMMIT:
        raise ValueError("official FANUC manifest must use the reviewed fixed commit")
    if str(upstream.get("license_spdx", "")) != "Apache-2.0":
        raise ValueError("official FANUC manifest must preserve its Apache-2.0 license")
    model = manifest.get("model")
    integration = manifest.get("integration")
    geometry = manifest.get("geometry")
    if not isinstance(model, Mapping) or not isinstance(integration, Mapping) or not isinstance(
        geometry, Mapping
    ):
        raise ValueError("official FANUC manifest model, geometry, and integration are required")
    expanded_urdf_record = integration.get("expanded_urdf")
    if not isinstance(expanded_urdf_record, Mapping):
        raise ValueError("official FANUC manifest requires an expanded URDF record")
    expanded_urdf = str(expanded_urdf_record.get("path", "")).strip()
    if (
        not expanded_urdf
        or int(expanded_urdf_record.get("bytes", 0)) <= 0
        or not _lowercase_sha256(
            expanded_urdf_record.get("sha256"), "official expanded URDF sha256"
        )
    ):
        raise ValueError("official FANUC manifest requires a repository-relative expanded URDF")
    visual_meshes = geometry.get("visual_meshes")
    collision_meshes = geometry.get("collision_meshes")
    if not isinstance(visual_meshes, list) or not isinstance(collision_meshes, list):
        raise ValueError("official FANUC manifest requires visual and collision mesh inventories")

    source_files = manifest.get("source_files")
    if (
        not isinstance(source_files, list)
        or len(source_files) != int(manifest.get("source_file_total_count", -1))
        or any(
            not isinstance(item, Mapping)
            or not str(item.get("path", "")).strip()
            or not str(item.get("upstream_path", "")).strip()
            or int(item.get("bytes", 0)) <= 0
            or not _lowercase_sha256(
                item.get("sha256"), "official source-file sha256"
            )
            for item in source_files
        )
        or sum(int(item["bytes"]) for item in source_files)
        != int(manifest.get("source_file_total_bytes", -1))
    ):
        raise ValueError("official FANUC source-file inventory is incomplete")

    actuated_order = model.get("actuated_joint_order")
    joints = model.get("joints")
    moving_links = model.get("moving_links")
    inertials = model.get("inertials")
    if (
        not isinstance(actuated_order, list)
        or len(actuated_order) != 6
        or len(set(actuated_order)) != 6
        or not isinstance(joints, list)
        or not isinstance(moving_links, list)
        or len(moving_links) != 6
        or not isinstance(inertials, list)
    ):
        raise ValueError("official FANUC manifest must expose six ordered joints and moving links")
    physical_links = {str(model.get("base_link", "")), *map(str, moving_links)}
    for name, inventory in (
        ("visual", visual_meshes),
        ("collision", collision_meshes),
    ):
        if (
            len(inventory) != len(physical_links)
            or {
                str(item.get("link", ""))
                for item in inventory
                if isinstance(item, Mapping)
            }
            != physical_links
            or any(
                not isinstance(item, Mapping) or not str(item.get("path", "")).strip()
                for item in inventory
            )
        ):
            raise ValueError(
                f"official FANUC {name} inventory must cover each physical link exactly once"
            )
    joints_by_name = {
        str(item.get("name", "")): item for item in joints if isinstance(item, Mapping)
    }
    if set(joints_by_name) != set(actuated_order):
        raise ValueError("official FANUC joint records differ from the actuated joint order")
    lower: list[float] = []
    upper: list[float] = []
    velocity: list[float] = []
    effort: list[float] = []
    for name in actuated_order:
        joint = joints_by_name[name]
        limit = joint.get("limit")
        if not isinstance(limit, Mapping):
            raise ValueError(f"official joint {name} has no complete limit record")
        values = np.asarray(
            [
                limit.get("lower_rad"),
                limit.get("upper_rad"),
                limit.get("velocity_rad_s"),
                limit.get("effort_nm"),
            ],
            dtype=float,
        )
        if (
            values.shape != (4,)
            or not np.all(np.isfinite(values))
            or values[0] >= values[1]
            or values[2] <= 0.0
            or values[3] <= 0.0
        ):
            raise ValueError(f"official joint {name} limits are invalid")
        lower.append(float(values[0]))
        upper.append(float(values[1]))
        velocity.append(float(values[2]))
        effort.append(float(values[3]))

    link_dynamics = []
    for raw in inertials:
        if not isinstance(raw, Mapping):
            raise ValueError("official link inertials must be mappings")
        item = {
            "link": raw.get("link"),
            "mass_kg": raw.get("mass_kg"),
            "com_xyz_m": raw.get("origin_xyz_m"),
            "inertia_at_com_kg_m2": raw.get("inertia_at_com_kg_m2"),
            "inertial_origin_rpy_rad": raw.get("origin_rpy_rad", [0.0, 0.0, 0.0]),
            "source_status": "OFFICIAL_FANUC_DESCRIPTION_FIXED_COMMIT",
        }
        link_dynamics.append(item)
    audited_dynamics = _validated_link_dynamics(link_dynamics)
    if set(item["link"] for item in audited_dynamics) != physical_links:
        raise ValueError("official FANUC inertials must cover the base and all six moving links")
    total_mass = float(sum(item["mass_kg"] for item in audited_dynamics))
    declared_total = float(model.get("total_mass_kg", float("nan")))
    if not np.isclose(total_mass, declared_total, atol=1e-9, rtol=0.0):
        raise ValueError("official FANUC total mass disagrees with its link inertials")
    return {
        "manifest": manifest,
        "expanded_urdf": expanded_urdf,
        "joint_names": tuple(str(name) for name in actuated_order),
        "joint_lower_rad": np.asarray(lower, dtype=float),
        "joint_upper_rad": np.asarray(upper, dtype=float),
        "joint_velocity_rad_s": np.asarray(velocity, dtype=float),
        "joint_effort_nm": np.asarray(effort, dtype=float),
        "link_dynamics": audited_dynamics,
        "moving_links": tuple(str(name) for name in moving_links),
        "mass_accounting_link": str(moving_links[-1]),
        "flange_link": str(model.get("flange_link", "")),
        "fanuc_flange_link": str(model.get("fanuc_flange_link", "")),
        "manifest_sha256": _canonical_digest(manifest),
    }


def _canonical_digest(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _serialize_obb(obb: OBB, *, dynamic: bool = False, mass_kg: float | None = None) -> dict[str, Any]:
    primitive = {
        "name": obb.name,
        "category": obb.category,
        "center_m": obb.center.tolist(),
        "size_m": (2.0 * obb.half_extents).tolist(),
        "rotation_matrix": obb.rotation.tolist(),
        "dynamic": bool(dynamic),
    }
    if mass_kg is not None:
        primitive["mass_kg"] = float(mass_kg)
    return primitive


def _declared_scene_primitives(plan: dict[str, Any]) -> list[dict[str, Any]] | None:
    """Validate a frozen, layout-bound scene supplied by a planning adapter.

    Legacy M-20 plans still use ``_build_scene_primitives`` below.  New workcell
    adapters must serialize the already verified frozen snapshot and place it
    in the plan; rebuilding that scene from mutable legacy configuration would
    silently change carton identities and known/unknown trailer boundaries.
    """
    declared = plan.get("scene_primitives")
    if declared is None:
        return None
    if not isinstance(declared, list) or not declared:
        raise ValueError("plan.scene_primitives must be a non-empty list")
    result: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, source in enumerate(declared):
        if not isinstance(source, dict):
            raise ValueError(f"plan.scene_primitives[{index}] must be a mapping")
        name = str(source.get("name", "")).strip()
        category = str(source.get("category", "")).strip()
        if not name or not category or name in names:
            raise ValueError("layout-bound scene primitive names/categories must be nonempty and unique")
        center = np.asarray(source.get("center_m", []), dtype=float)
        size = np.asarray(source.get("size_m", []), dtype=float)
        rotation = np.asarray(source.get("rotation_matrix", []), dtype=float)
        if (
            center.shape != (3,)
            or size.shape != (3,)
            or rotation.shape != (3, 3)
            or not np.all(np.isfinite(center))
            or not np.all(np.isfinite(size))
            or not np.all(np.isfinite(rotation))
            or np.any(size <= 0.0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0.0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-9, rtol=0.0)
        ):
            raise ValueError(f"invalid layout-bound scene primitive geometry: {name}")
        dynamic = bool(source.get("dynamic", False))
        item = {
            "name": name,
            "category": category,
            "center_m": center.tolist(),
            "size_m": size.tolist(),
            "rotation_matrix": rotation.tolist(),
            "dynamic": dynamic,
        }
        if dynamic:
            mass = float(source.get("mass_kg", float("nan")))
            inertia = np.asarray(source.get("inertia_at_com_kg_m2", []), dtype=float)
            if (
                not np.isfinite(mass)
                or mass <= 0.0
                or inertia.shape != (3, 3)
                or not np.all(np.isfinite(inertia))
                or not np.allclose(inertia, inertia.T, atol=1e-10, rtol=0.0)
                or np.min(np.linalg.eigvalsh(inertia)) <= 0.0
            ):
                raise ValueError(f"dynamic primitive requires positive mass and inertia: {name}")
            item["mass_kg"] = mass
            item["inertia_at_com_kg_m2"] = inertia.tolist()
        material = source.get("material")
        if material is not None:
            if not isinstance(material, str) or not material.strip():
                raise ValueError(f"invalid physics material name for scene primitive: {name}")
            item["material"] = material
        boundary = source.get("boundary")
        if boundary is not None:
            if not isinstance(boundary, dict) or set(boundary) != {
                "axis",
                "value_m",
                "inside",
                "extent_status",
            }:
                raise ValueError(f"invalid boundary contract for scene primitive: {name}")
            axis = boundary["axis"]
            inside = boundary["inside"]
            value = float(boundary["value_m"])
            if (
                axis not in {"x", "y", "z"}
                or inside not in {"+", "-"}
                or not np.isfinite(value)
                or not isinstance(boundary["extent_status"], str)
                or not boundary["extent_status"].strip()
            ):
                raise ValueError(f"invalid boundary contract for scene primitive: {name}")
            item["boundary"] = {
                "axis": axis,
                "value_m": value,
                "inside": inside,
                "extent_status": boundary["extent_status"],
            }
        result.append(item)
        names.add(name)
    return result


def _validated_link_dynamics(value: Any) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        raise ValueError("robot_link_dynamics must be a list")
    result: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, source in enumerate(value):
        if not isinstance(source, dict):
            raise ValueError(f"robot_link_dynamics[{index}] must be a mapping")
        name = str(source.get("link", "")).strip()
        mass = float(source.get("mass_kg", float("nan")))
        center = np.asarray(source.get("com_xyz_m", []), dtype=float)
        inertia = np.asarray(source.get("inertia_at_com_kg_m2", []), dtype=float)
        inertial_origin_rpy = np.asarray(
            source.get("inertial_origin_rpy_rad", [0.0, 0.0, 0.0]), dtype=float
        )
        if not name or name in names:
            raise ValueError("robot dynamic link names must be nonempty and unique")
        eigenvalues = np.linalg.eigvalsh(inertia) if inertia.shape == (3, 3) else np.asarray([])
        if (
            not np.isfinite(mass)
            or mass <= 0.0
            or center.shape != (3,)
            or not np.all(np.isfinite(center))
            or inertia.shape != (3, 3)
            or not np.all(np.isfinite(inertia))
            or inertial_origin_rpy.shape != (3,)
            or not np.all(np.isfinite(inertial_origin_rpy))
            or not np.allclose(inertia, inertia.T, atol=1e-10, rtol=0.0)
            or eigenvalues.shape != (3,)
            or np.any(eigenvalues <= 0.0)
            or 2.0 * np.max(eigenvalues) > np.sum(eigenvalues) + 1e-10
        ):
            raise ValueError(f"invalid physical mass properties for robot link {name!r}")
        result.append(
            {
                "link": name,
                "mass_kg": mass,
                "com_xyz_m": center.tolist(),
                "inertia_at_com_kg_m2": inertia.tolist(),
                "inertial_origin_rpy_rad": inertial_origin_rpy.tolist(),
                "source_status": str(source.get("source_status", "UNSPECIFIED")),
            }
        )
        names.add(name)
    return result


def _combine_fixed_mass_properties(
    first_mass_kg: float,
    first_com_m: np.ndarray,
    first_inertia_at_com_kg_m2: np.ndarray,
    second_mass_kg: float,
    second_com_m: np.ndarray,
    second_inertia_at_com_kg_m2: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Combine two fixed bodies expressed in one common body frame."""

    first_mass = float(first_mass_kg)
    second_mass = float(second_mass_kg)
    first_com = np.asarray(first_com_m, dtype=float)
    second_com = np.asarray(second_com_m, dtype=float)
    first_inertia = np.asarray(first_inertia_at_com_kg_m2, dtype=float)
    second_inertia = np.asarray(second_inertia_at_com_kg_m2, dtype=float)
    total_mass = first_mass + second_mass
    if (
        not np.isfinite(first_mass)
        or not np.isfinite(second_mass)
        or first_mass <= 0.0
        or second_mass <= 0.0
        or first_com.shape != (3,)
        or second_com.shape != (3,)
        or first_inertia.shape != (3, 3)
        or second_inertia.shape != (3, 3)
        or not np.all(np.isfinite(first_com))
        or not np.all(np.isfinite(second_com))
        or not np.all(np.isfinite(first_inertia))
        or not np.all(np.isfinite(second_inertia))
    ):
        raise ValueError("fixed-body mass properties must be finite and positive")
    combined_com = (first_mass * first_com + second_mass * second_com) / total_mass

    def shifted(inertia: np.ndarray, mass: float, source_com: np.ndarray) -> np.ndarray:
        offset = source_com - combined_com
        return inertia + mass * (
            float(offset @ offset) * np.eye(3) - np.outer(offset, offset)
        )

    combined_inertia = shifted(first_inertia, first_mass, first_com) + shifted(
        second_inertia, second_mass, second_com
    )
    return total_mass, combined_com, combined_inertia


def _transform_mass_properties(
    center_child_m: Sequence[float],
    inertia_child_at_com_kg_m2: Sequence[Sequence[float]],
    parent_from_child: Sequence[Sequence[float]],
) -> tuple[np.ndarray, np.ndarray]:
    """Express a body's COM and full inertia tensor in its fixed parent frame."""

    center = np.asarray(center_child_m, dtype=float)
    inertia = np.asarray(inertia_child_at_com_kg_m2, dtype=float)
    transform = np.asarray(parent_from_child, dtype=float)
    if (
        center.shape != (3,)
        or inertia.shape != (3, 3)
        or transform.shape != (4, 4)
        or not np.all(np.isfinite(center))
        or not np.all(np.isfinite(inertia))
        or not np.all(np.isfinite(transform))
        or not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0)
        or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-10, rtol=0.0)
        or not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-10, rtol=0.0)
    ):
        raise ValueError("fixed-body frame transform and mass properties must be finite SE(3)")
    rotation = transform[:3, :3]
    return (
        rotation @ center + transform[:3, 3],
        rotation @ inertia @ rotation.T,
    )


def _validated_physics_contract(
    value: Any,
    *,
    solver_position_iterations: int,
    solver_velocity_iterations: int,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("M-710 physics contract must be a mapping")
    required = {
        "parameter_status",
        "gravity_world_m_s2",
        "physics_time_step_s",
        "execution_backend",
        "contact_offset_m",
        "rest_offset_m",
        "materials",
        "material_by_category",
        "damping",
        "settling",
    }
    if set(value) != required:
        raise ValueError("M-710 physics contract keys are incomplete")
    gravity = np.asarray(value["gravity_world_m_s2"], dtype=float)
    time_step = float(value["physics_time_step_s"])
    if (
        not isinstance(value["parameter_status"], str)
        or not value["parameter_status"].strip()
        or gravity.shape != (3,)
        or not np.all(np.isfinite(gravity))
        or np.linalg.norm(gravity) <= 0.0
        or not np.isfinite(time_step)
        or time_step <= 0.0
    ):
        raise ValueError("M-710 gravity and physics time step must be explicit and finite")

    backend = value["execution_backend"]
    backend_keys = {
        "mode", "device", "broadphase_type", "gpu_dynamics_enabled", "fabric_enabled",
        "ccd_enabled",
    }
    if not isinstance(backend, dict) or set(backend) != backend_keys:
        raise ValueError("M-710 physics execution backend contract is incomplete")
    backend_tuple = (
        backend["mode"], backend["device"], backend["broadphase_type"],
        backend["gpu_dynamics_enabled"], backend["fabric_enabled"], backend["ccd_enabled"],
    )
    if backend_tuple != ("physx_cpu", "cpu", "MBP", False, True, True):
        raise ValueError("M-710 physics execution backend contract is inconsistent")

    contact_offset_m = float(value["contact_offset_m"])
    rest_offset_m = float(value["rest_offset_m"])
    if (
        not np.isfinite(contact_offset_m)
        or not np.isfinite(rest_offset_m)
        or contact_offset_m != 0.010
        or rest_offset_m != 0.0
    ):
        raise ValueError(
            "M-710 physics requires an explicit 10 mm contact offset and zero rest offset"
        )

    materials = value["materials"]
    if not isinstance(materials, dict) or not materials:
        raise ValueError("M-710 physics materials must be a non-empty mapping")
    audited_materials: dict[str, dict[str, float]] = {}
    for name, raw in materials.items():
        if not isinstance(name, str) or not name or not isinstance(raw, dict) or set(raw) != {
            "static_friction",
            "dynamic_friction",
            "restitution",
        }:
            raise ValueError("invalid M-710 physics material contract")
        static = float(raw["static_friction"])
        dynamic = float(raw["dynamic_friction"])
        restitution = float(raw["restitution"])
        if (
            not all(np.isfinite(item) for item in (static, dynamic, restitution))
            or static < 0.0
            or dynamic < 0.0
            or dynamic > static
            or not 0.0 <= restitution <= 1.0
        ):
            raise ValueError(f"invalid M-710 physics material coefficients: {name}")
        audited_materials[name] = {
            "static_friction": static,
            "dynamic_friction": dynamic,
            "restitution": restitution,
        }

    material_by_category = value["material_by_category"]
    if not isinstance(material_by_category, dict) or any(
        not isinstance(category, str)
        or not category
        or material not in audited_materials
        for category, material in material_by_category.items()
    ):
        raise ValueError("M-710 scene material mapping references an unknown material")

    damping = value["damping"]
    if not isinstance(damping, dict) or not damping:
        raise ValueError("M-710 rigid-body damping must be a non-empty mapping")
    audited_damping: dict[str, dict[str, float]] = {}
    for name, raw in damping.items():
        if not isinstance(name, str) or not name or not isinstance(raw, dict) or set(raw) != {
            "linear_damping_s_inv",
            "angular_damping_s_inv",
        }:
            raise ValueError("invalid M-710 rigid-body damping contract")
        linear = float(raw["linear_damping_s_inv"])
        angular = float(raw["angular_damping_s_inv"])
        if not all(np.isfinite(item) and item >= 0.0 for item in (linear, angular)):
            raise ValueError(f"invalid M-710 rigid-body damping coefficients: {name}")
        audited_damping[name] = {
            "linear_damping_s_inv": linear,
            "angular_damping_s_inv": angular,
        }
    if set(audited_damping) != {"robot_links", "tool", "cartons"}:
        raise ValueError("M-710 damping must cover robot_links, tool, and cartons")
    if audited_damping["tool"] != audited_damping["robot_links"]:
        raise ValueError(
            "fixed M-710 tool/J6 mass combination requires identical tool and robot-link damping"
        )

    settling = value["settling"]
    settling_keys = {
        "maximum_settle_time_s",
        "required_stable_duration_s",
        "max_linear_speed_m_s",
        "max_angular_speed_rad_s",
        "max_position_drift_m",
        "max_penetration_m",
    }
    if not isinstance(settling, dict) or set(settling) != settling_keys:
        raise ValueError("M-710 settling contract is incomplete")
    audited_settling = {name: float(settling[name]) for name in settling_keys}
    if any(not np.isfinite(item) or item <= 0.0 for item in audited_settling.values()):
        raise ValueError("M-710 settling thresholds must be finite and positive")
    if audited_settling["required_stable_duration_s"] > audited_settling["maximum_settle_time_s"]:
        raise ValueError("M-710 stable duration exceeds the maximum settle time")
    if solver_position_iterations <= 0 or solver_velocity_iterations <= 0:
        raise ValueError("M-710 solver iteration counts must be positive")
    return {
        "parameter_status": value["parameter_status"],
        "gravity_world_m_s2": gravity.tolist(),
        "physics_time_step_s": time_step,
        "execution_backend": dict(backend),
        "contact_offset_m": contact_offset_m,
        "rest_offset_m": rest_offset_m,
        "solver_position_iterations": int(solver_position_iterations),
        "solver_velocity_iterations": int(solver_velocity_iterations),
        "friction_combine_mode": "min",
        "restitution_combine_mode": "min",
        "materials": audited_materials,
        "material_by_category": dict(material_by_category),
        "damping": audited_damping,
        "settling": audited_settling,
    }


def _validated_rendering_contract(value: Any, *, physics_time_step_s: float) -> dict[str, Any]:
    """Validate the content-addressed recording contract used by M-710 replay."""

    expected_contract_keys = {
        "required_output",
        "material_palette",
        "conveyor_visual_motion",
    }
    if not isinstance(value, dict) or set(value) != expected_contract_keys:
        raise ValueError(
            "M-710 rendering contract must contain output, palette, and conveyor motion"
        )
    required = value["required_output"]
    expected_keys = {"width_px", "height_px", "fps", "camera_mode"}
    if not isinstance(required, dict) or set(required) != expected_keys:
        raise ValueError("M-710 required output contract is incomplete")
    width = required["width_px"]
    height = required["height_px"]
    fps = required["fps"]
    camera_mode = required["camera_mode"]
    if (
        isinstance(width, bool)
        or isinstance(height, bool)
        or isinstance(fps, bool)
        or not isinstance(width, int)
        or not isinstance(height, int)
        or not isinstance(fps, int)
        or width <= 0
        or height <= 0
        or fps <= 0
        or not isinstance(camera_mode, str)
        or not camera_mode.strip()
    ):
        raise ValueError("M-710 required output values must be positive integers and a camera mode")
    physics_hz = 1.0 / float(physics_time_step_s)
    render_stride = physics_hz / fps
    if not np.isclose(render_stride, round(render_stride), atol=1e-9, rtol=0.0):
        raise ValueError("M-710 output FPS must divide the physics rate into an integer render stride")
    palette = value["material_palette"]
    expected_palette = {
        "chassis_rgb": [0.10, 0.12, 0.16],
        "conveyor_rgb": [0.035, 0.22, 0.62],
        "conveyor_motion_marker_rgb": [1.0, 0.58, 0.03],
    }
    if not isinstance(palette, dict) or set(palette) != set(expected_palette):
        raise ValueError("M-710 rendering palette is incomplete")
    audited_palette = {}
    for name, expected in expected_palette.items():
        color = np.asarray(palette[name], dtype=float)
        if color.shape != (3,) or not np.all(np.isfinite(color)) or not np.allclose(
            color, expected, atol=1e-12, rtol=0.0
        ):
            raise ValueError(f"M-710 rendering color {name} changed from the reviewed palette")
        audited_palette[name] = color.tolist()
    visual_motion = value["conveyor_visual_motion"]
    expected_visual_motion = {
        "model": "collision_free_wrapped_surface_markers_v1",
        "markers_have_collision": False,
        "markers_follow_active_physx_surface_velocity": True,
    }
    if visual_motion != expected_visual_motion:
        raise ValueError("M-710 conveyor visual motion contract is invalid")
    return {
        "required_output": {
            "width_px": width,
            "height_px": height,
            "fps": fps,
            "camera_mode": camera_mode,
            "render_every_physics_steps": int(round(render_stride)),
        },
        "material_palette": audited_palette,
        "conveyor_visual_motion": dict(expected_visual_motion),
    }


def _build_scene_primitives(
    plan: dict[str, Any], cfg: dict[str, Any], segment_index: int
) -> list[dict[str, Any]]:
    """Serialize the selected docked scene without importing a physics backend."""
    declared = _declared_scene_primitives(plan)
    if declared is not None:
        return declared
    scene_cfg = cfg.get("scene")
    if not isinstance(scene_cfg, dict):
        return []

    primitives: list[dict[str, Any]] = []
    trailer = scene_cfg.get("trailer", {})
    required_trailer_keys = {"length", "width", "height"}
    if required_trailer_keys.issubset(trailer):
        walls = build_trailer_walls(
            length=float(trailer["length"]),
            width=float(trailer["width"]),
            height=float(trailer["height"]),
            wall_thickness=float(trailer.get("wall_thickness", 0.05)),
            floor_thickness=float(trailer.get("floor_thickness", 0.08)),
        )
        primitives.extend(_serialize_obb(wall) for wall in walls)

    segment = plan["segments"][segment_index]
    dock_value = segment.get("amr_dock_position")
    dock = np.asarray(dock_value, dtype=float) if dock_value is not None else None
    amr_cfg = cfg.get("amr", {})
    mounted_centers = dict(amr_cfg.get("mounted_surface_centers", {}))
    conveyor_name = str(amr_cfg.get("conveyor_name", "conveyor_deck"))
    if "conveyor_mount_center" in amr_cfg:
        mounted_centers.setdefault(conveyor_name, amr_cfg["conveyor_mount_center"])

    for item in scene_cfg.get("static_obstacles", []):
        center = np.asarray(item["center"], dtype=float)
        if dock is not None and item["name"] in mounted_centers:
            center = dock + np.asarray(mounted_centers[item["name"]], dtype=float)
        obb = OBB(
            center=center,
            half_extents=0.5 * np.asarray(item["size"], dtype=float),
            rotation=rotation_matrix_from_rpy(*item.get("rpy", [0.0, 0.0, 0.0])),
            name=str(item["name"]),
            category="static",
        )
        primitives.append(_serialize_obb(obb))

    validation_cfg = cfg.get("simulation_validation", {})
    conveyor_cfg = validation_cfg.get("conveyor", {})
    if bool(conveyor_cfg.get("enabled", False)):
        if dock is None:
            raise ValueError("an enabled AMR conveyor requires a segment dock position")
        outfeed_name = str(conveyor_cfg.get("outfeed_name", "conveyor_outfeed"))
        outfeed_offset = np.asarray(
            conveyor_cfg.get("outfeed_mount_center", []), dtype=float
        )
        outfeed_size = np.asarray(conveyor_cfg.get("outfeed_size_m", []), dtype=float)
        if outfeed_offset.shape != (3,) or outfeed_size.shape != (3,):
            raise ValueError("conveyor outfeed center and size must contain three values")
        if not np.all(np.isfinite(outfeed_offset)) or not np.all(np.isfinite(outfeed_size)):
            raise ValueError("conveyor outfeed geometry must be finite")
        if np.any(outfeed_size <= 0.0):
            raise ValueError("conveyor outfeed size must be positive")
        primitives.append(
            _serialize_obb(
                OBB(
                    center=dock + outfeed_offset,
                    half_extents=0.5 * outfeed_size,
                    rotation=np.eye(3),
                    name=outfeed_name,
                    category="static",
                )
            )
        )

    if dock is not None and "footprint_size" in amr_cfg:
        size = np.asarray(amr_cfg["footprint_size"], dtype=float)
        offset = np.asarray(amr_cfg.get("platform_center_offset", [0.0, 0.0, size[2] / 2.0]))
        primitives.append(
            _serialize_obb(
                OBB(
                    center=dock + offset,
                    half_extents=0.5 * size,
                    rotation=np.eye(3),
                    name="amr_base",
                    category="amr",
                )
            )
        )

    removed_names = {
        str(item.get("target")) for item in plan["segments"][:segment_index] if item.get("target")
    }
    target_name = str(segment.get("target", ""))
    carton_mass = float(cfg.get("simulation_validation", {}).get("carton_mass_kg", 7.0))
    if not np.isfinite(carton_mass) or carton_mass <= 0.0:
        raise ValueError("simulation_validation.carton_mass_kg must be finite and positive")
    for item in scene_cfg.get("cartons", []):
        name = str(item["name"])
        if name in removed_names:
            continue
        carton = OBB(
            center=np.asarray(item["center"], dtype=float),
            half_extents=0.5 * np.asarray(item["size"], dtype=float),
            rotation=rotation_matrix_from_rpy(*item.get("rpy", [0.0, 0.0, 0.0])),
            name=name,
            category="carton",
        )
        # Only the picked carton needs to be dynamic in a single-segment
        # qualification run. Its neighbours remain exact collision geometry.
        primitives.append(
            _serialize_obb(carton, dynamic=name == target_name, mass_kg=carton_mass)
        )
    return primitives


@dataclass(frozen=True)
class IsaacReplayBundle:
    timestamps_seconds: np.ndarray
    positions_rad: np.ndarray
    metadata: dict[str, Any]

    def __post_init__(self) -> None:
        timestamps = np.asarray(self.timestamps_seconds, dtype=float)
        positions = np.asarray(self.positions_rad, dtype=float)
        if timestamps.ndim != 1 or positions.ndim != 2 or len(timestamps) != len(positions):
            raise ValueError("Isaac replay commands require (N,) timestamps and (N, dof) positions")
        if len(timestamps) < 2 or timestamps[0] != 0.0 or np.any(np.diff(timestamps) <= 0.0):
            raise ValueError("Isaac replay timestamps must start at zero and increase strictly")
        if not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(positions)):
            raise ValueError("Isaac replay commands must be finite")
        object.__setattr__(self, "timestamps_seconds", timestamps.copy())
        object.__setattr__(self, "positions_rad", positions.copy())

    def to_dict(self) -> dict[str, Any]:
        return add_bundle_payload_sha256({
            "format": "isaacsim_fanuc_replay_v1",
            "timestamps_seconds": self.timestamps_seconds.tolist(),
            "positions_rad": self.positions_rad.tolist(),
            "metadata": copy.deepcopy(self.metadata),
        })


def _sample_trajectory(
    timestamps: np.ndarray,
    positions: np.ndarray,
    period_seconds: float,
) -> tuple[np.ndarray, np.ndarray]:
    if not np.isfinite(period_seconds) or period_seconds <= 0.0:
        raise ValueError("controller period must be finite and positive")
    duration = float(timestamps[-1])
    command_times = np.arange(0.0, duration, period_seconds, dtype=float)
    if command_times.size == 0 or duration - command_times[-1] > 1e-12:
        command_times = np.append(command_times, duration)
    else:
        command_times[-1] = duration
    commands = np.column_stack(
        [np.interp(command_times, timestamps, positions[:, joint]) for joint in range(positions.shape[1])]
    )
    return command_times, commands


def _insert_command_hold(
    timestamps: np.ndarray,
    positions: np.ndarray,
    event_time_seconds: float,
    hold_seconds: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Insert an exact, stationary controller hold at a trajectory event."""
    event_time = float(event_time_seconds)
    hold = float(hold_seconds)
    if not np.isfinite(event_time) or event_time < 0.0 or event_time > timestamps[-1]:
        raise ValueError("controller hold event time is outside the command schedule")
    if not np.isfinite(hold) or hold < 0.0:
        raise ValueError("controller hold duration must be finite and non-negative")
    if hold == 0.0:
        return timestamps, positions

    event_position = np.asarray(
        [np.interp(event_time, timestamps, positions[:, joint]) for joint in range(positions.shape[1])]
    )
    tolerance = 1e-12
    before = timestamps < event_time - tolerance
    after = timestamps > event_time + tolerance
    held_times = np.concatenate(
        (timestamps[before], [event_time, event_time + hold], timestamps[after] + hold)
    )
    held_positions = np.vstack(
        (positions[before], event_position, event_position, positions[after])
    )
    return held_times, held_positions


def build_fanuc_isaac_replay_bundle(
    plan: dict[str, Any],
    cfg: dict[str, Any],
    *,
    segment_index: int = 0,
    controller_period_seconds: float | None = None,
    preflight: dict[str, Any] | None = None,
) -> IsaacReplayBundle:
    """Build a limit-audited FANUC command stream from one planned segment."""
    robot = plan.get("robot")
    if not isinstance(robot, dict):
        raise ValueError("Isaac replay requires a supported six-axis FANUC plan")
    try:
        robot_model_id = normalize_robot_model_id(robot.get("model"))
    except ValueError as exc:
        raise ValueError("Isaac replay requires a supported six-axis FANUC plan") from exc
    if robot_model_id not in SUPPORTED_FANUC_REPLAY_MODELS:
        raise ValueError("Isaac replay requires a supported six-axis FANUC plan")
    raw_official_manifest = robot.get("official_model_manifest")
    if raw_official_manifest is None:
        raw_official_manifest = plan.get("official_model_manifest")
    if raw_official_manifest is None:
        raw_official_manifest = cfg.get("official_model_manifest")
    official_model = (
        None
        if raw_official_manifest is None
        else _validated_official_model_manifest(raw_official_manifest)
    )
    official_model_required = plan.get("official_model_required", False)
    if not isinstance(official_model_required, bool):
        raise ValueError("official_model_required must be a boolean")
    if robot_model_id == "fanuc_m710id_70" and official_model_required and official_model is None:
        raise ValueError("qualified M-710 replay requires the inline official model manifest")
    declared_official_manifest_sha256 = robot.get("official_model_manifest_sha256")
    if declared_official_manifest_sha256 is None:
        declared_official_manifest_sha256 = plan.get("official_model_manifest_sha256")
    if declared_official_manifest_sha256 is None:
        declared_official_manifest_sha256 = cfg.get("official_model_manifest_sha256")
    official_manifest_file_sha256 = (
        None
        if official_model is None
        else _lowercase_sha256(
            declared_official_manifest_sha256,
            "official model manifest file sha256",
        )
    )
    if official_model is None and declared_official_manifest_sha256 is not None:
        raise ValueError("official model manifest sha256 requires the inline manifest")
    declared_srdf_path = str(robot.get("srdf_path", "")).strip()
    declared_srdf_sha256 = robot.get("srdf_sha256")
    if official_model is not None:
        if not declared_srdf_path:
            raise ValueError("official M-710 replay requires an explicit SRDF path")
        declared_srdf_sha256 = _lowercase_sha256(
            declared_srdf_sha256, "official-model SRDF sha256"
        )
    joint_names = (
        FANUC_JOINT_NAMES
        if official_model is None
        else official_model["joint_names"]
    )
    segments = plan.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("plan contains no trajectory segments")
    if segment_index < 0 or segment_index >= len(segments):
        raise IndexError("segment index is outside the plan")

    segment = segments[segment_index]
    m710_replay_contract = None
    if robot_model_id == "fanuc_m710id_70":
        if preflight is None:
            raise ValueError("M-710 replay requires a full verified execution preflight")
        if segment_index != 0 or len(segments) != 1:
            raise ValueError("M-710 replay preflight binds exactly one selected trajectory segment")
        m710_replay_contract = build_m710_replay_contract(
            preflight,
            plan,
            cfg,
            segment,
        )
    path = np.asarray(segment.get("path", []), dtype=float)
    if path.ndim != 2 or path.shape[1] != len(joint_names) or len(path) < 2:
        raise ValueError("FANUC segment path must have shape (N, 6) with at least two waypoints")
    if official_model is not None and (
        np.any(path < official_model["joint_lower_rad"][None, :] - 1e-12)
        or np.any(path > official_model["joint_upper_rad"][None, :] + 1e-12)
    ):
        raise ValueError("trajectory exceeds a fixed official FANUC joint position limit")
    execution = cfg.get("execution", {})
    effective_cfg = cfg
    if official_model is not None:
        effective_cfg = copy.deepcopy(cfg)
        effective_execution = effective_cfg.setdefault("execution", {})
        declared_velocity = np.asarray(
            effective_execution.get("joint_velocity_limits_rad_s", []), dtype=float
        )
        official_velocity = official_model["joint_velocity_rad_s"]
        if declared_velocity.size and (
            declared_velocity.shape != official_velocity.shape
            or not np.allclose(declared_velocity, official_velocity, atol=1e-12, rtol=0.0)
        ):
            raise ValueError("execution velocity limits differ from the fixed official FANUC manifest")
        effective_execution["joint_velocity_limits_rad_s"] = official_velocity.tolist()
        effective_execution["limits_source"] = (
            f"{OFFICIAL_M710_DESCRIPTION_REPOSITORY}@{OFFICIAL_M710_DESCRIPTION_COMMIT}"
        )
        execution = effective_execution
    limits = motion_limits_from_config(effective_cfg, path.shape[1])
    loaded_motion_time_scale = float(execution.get("loaded_motion_time_scale", 1.0))
    release_index = int(segment.get("release_index", len(path) - 1))
    if not 0 <= release_index < len(path):
        raise ValueError("release_index is outside the source path")
    event_indices: dict[str, int] = {}
    for index_key in ("grasp_index", "release_index", "release_retreat_index"):
        raw_index = segment.get(index_key)
        if raw_index is None:
            continue
        index = int(raw_index)
        if index < 0 or index >= len(path):
            raise ValueError(f"{index_key} is outside the source path")
        event_indices[index_key] = index
    if (
        "grasp_index" in event_indices
        and "release_index" in event_indices
        and event_indices["grasp_index"] > event_indices["release_index"]
    ):
        raise ValueError("grasp_index must not follow release_index")
    if (
        "release_index" in event_indices
        and "release_retreat_index" in event_indices
        and event_indices["release_index"] > event_indices["release_retreat_index"]
    ):
        raise ValueError("release_index must not follow release_retreat_index")
    post_release_motion_limit_scale = float(
        execution.get("post_release_motion_limit_scale", limits.limit_scale)
    )
    if (
        not np.isfinite(post_release_motion_limit_scale)
        or not 0.0 < post_release_motion_limit_scale <= 1.0
    ):
        raise ValueError("post_release_motion_limit_scale must be in (0, 1]")

    # The release dwell is a commanded stop, so it is a genuine trajectory
    # phase boundary. Time the loaded prefix conservatively and the empty-tool
    # escape independently; otherwise the loaded motion scale needlessly keeps
    # the gripper beside a carton that is already moving on a live conveyor.
    prefix = time_parameterize_joint_path(path[: release_index + 1], limits)
    prefix = scale_timed_trajectory_window(
        prefix,
        int(segment.get("grasp_index", 0)),
        release_index,
        loaded_motion_time_scale,
    )
    prefix_audit = prefix.audit(limits)
    empty_limits = replace(
        limits,
        limit_scale=post_release_motion_limit_scale,
        source=f"{limits.source}; empty-tool post-release phase",
    )
    empty = None
    empty_audit = None
    if release_index < len(path) - 1:
        empty = time_parameterize_joint_path(path[release_index:], empty_limits)
        empty_audit = empty.audit(empty_limits)
        source_motion_times = np.concatenate(
            (
                prefix.time_from_start,
                prefix.duration_seconds + empty.time_from_start[1:],
            )
        )
    else:
        source_motion_times = prefix.time_from_start
    audit = {
        **prefix_audit,
        "duration_seconds": float(source_motion_times[-1]),
        "within_limits": bool(
            prefix_audit["within_limits"]
            and (empty_audit is None or empty_audit["within_limits"])
        ),
        "phase_model": "stopped_release_then_empty_tool",
        "pre_release": prefix_audit,
        "post_release": empty_audit,
    }
    if not audit["within_limits"]:
        raise RuntimeError("trajectory cannot be exported because its timing audit failed")
    replay_time_scale = float(cfg.get("execution", {}).get("isaac_replay_time_scale", 1.0))
    if not np.isfinite(replay_time_scale) or replay_time_scale < 1.0:
        raise ValueError("execution.isaac_replay_time_scale must be finite and at least one")
    replay_motion_times = source_motion_times * replay_time_scale

    if controller_period_seconds is None:
        controller_period_seconds = float(
            cfg.get("planning", {}).get("trajectory_waypoint_period_seconds", 0.02)
        )
    command_times, commands = _sample_trajectory(
        replay_motion_times, path, float(controller_period_seconds)
    )

    def event_time(index_key: str) -> float | None:
        value = segment.get(index_key)
        if value is None:
            return None
        index = int(value)
        if index < 0 or index >= len(replay_motion_times):
            raise ValueError(f"{index_key} is outside the source path")
        return float(replay_motion_times[index])

    pre_grasp_settle_seconds = float(
        execution.get("pre_grasp_controller_settle_seconds", 0.0)
    )
    vacuum_establish_seconds = float(execution.get("vacuum_establish_seconds", 0.0))
    release_seconds = float(execution.get("release_seconds", 0.0))
    if not np.isfinite(pre_grasp_settle_seconds) or pre_grasp_settle_seconds < 0.0:
        raise ValueError(
            "pre_grasp_controller_settle_seconds must be finite and non-negative"
        )
    if not np.isfinite(vacuum_establish_seconds) or vacuum_establish_seconds < 0.0:
        raise ValueError("vacuum_establish_seconds must be finite and non-negative")
    if not np.isfinite(release_seconds) or release_seconds < 0.0:
        raise ValueError("release_seconds must be finite and non-negative")

    source_grasp_time = event_time("grasp_index")
    source_release_time = event_time("release_index")
    grasp_arrival_time = source_grasp_time
    grasp_time = source_grasp_time
    if source_grasp_time is not None:
        command_times, commands = _insert_command_hold(
            command_times, commands, source_grasp_time, pre_grasp_settle_seconds
        )
        grasp_time += pre_grasp_settle_seconds
        command_times, commands = _insert_command_hold(
            command_times, commands, grasp_time, vacuum_establish_seconds
        )
    release_arrival_time = source_release_time
    if (
        release_arrival_time is not None
        and source_grasp_time is not None
        and source_release_time >= source_grasp_time
    ):
        release_arrival_time += pre_grasp_settle_seconds + vacuum_establish_seconds
    release_time = release_arrival_time
    if release_arrival_time is not None:
        command_times, commands = _insert_command_hold(
            command_times, commands, release_arrival_time, release_seconds
        )
        release_time += release_seconds
    source_release_retreat_time = event_time("release_retreat_index")
    release_retreat_time = source_release_retreat_time
    if release_retreat_time is not None:
        if source_grasp_time is not None and source_release_retreat_time >= source_grasp_time:
            release_retreat_time += pre_grasp_settle_seconds + vacuum_establish_seconds
        if source_release_time is not None and source_release_retreat_time >= source_release_time:
            release_retreat_time += release_seconds
    effort_limits = np.asarray(execution.get("joint_effort_limits_nm", []), dtype=float)
    if official_model is not None:
        official_effort = official_model["joint_effort_nm"]
        if effort_limits.size and (
            effort_limits.shape != official_effort.shape
            or not np.allclose(effort_limits, official_effort, atol=1e-12, rtol=0.0)
        ):
            raise ValueError("execution effort limits differ from the fixed official FANUC manifest")
        effort_limits = official_effort.copy()
    if effort_limits.shape not in {(0,), (len(joint_names),)}:
        raise ValueError("joint_effort_limits_nm must contain one value per FANUC joint")
    if effort_limits.size and (not np.all(np.isfinite(effort_limits)) or np.any(effort_limits <= 0.0)):
        raise ValueError("joint effort limits must be finite and positive")
    stiffness = np.asarray(execution.get("joint_drive_stiffness_nm_rad", []), dtype=float)
    damping = np.asarray(execution.get("joint_drive_damping_nm_s_rad", []), dtype=float)
    if robot_model_id == "fanuc_m20id_35" and not stiffness.size and not damping.size:
        # Preserve the legacy replay controller as explicit metadata.  New
        # M-710 contracts are never allowed to inherit these M-20 values.
        stiffness = np.asarray([80000.0, 80000.0, 60000.0, 12000.0, 8000.0, 5000.0])
        damping = np.asarray([5000.0, 5000.0, 4000.0, 800.0, 500.0, 300.0])
    raw_simulation_execution_ready = plan.get(
        "simulation_execution_ready", robot_model_id == "fanuc_m20id_35"
    )
    raw_execution_blockers = plan.get("execution_blockers", [])
    if not isinstance(raw_simulation_execution_ready, bool):
        raise ValueError("simulation_execution_ready must be a boolean")
    if not isinstance(raw_execution_blockers, list) or any(
        not isinstance(item, str) or not item.strip() for item in raw_execution_blockers
    ):
        raise ValueError("execution_blockers must be a list of non-empty strings")
    simulation_execution_ready = raw_simulation_execution_ready
    execution_blockers = list(raw_execution_blockers)
    machine_qualified = plan.get(
        "machine_qualified", robot_model_id == "fanuc_m20id_35"
    )
    machine_qualification_warnings = plan.get("machine_qualification_warnings", [])
    if not isinstance(machine_qualified, bool):
        raise ValueError("machine_qualified must be a boolean")
    if not isinstance(machine_qualification_warnings, list) or any(
        not isinstance(item, str) or not item.strip()
        for item in machine_qualification_warnings
    ):
        raise ValueError("machine_qualification_warnings must be a list of non-empty strings")
    # Machine/controller certification is evidence for a real installation,
    # not a prerequisite for executing the sourced rigid-body model in Isaac.
    # ``execution_qualified`` is retained as a legacy reporting field only.
    execution_qualified = plan.get("execution_qualified", False)
    if not isinstance(execution_qualified, bool):
        raise ValueError("legacy execution_qualified must be a boolean")
    simulation_execution_qualified = plan.get(
        "simulation_execution_qualified",
        simulation_execution_ready and not execution_blockers,
    )
    if not isinstance(simulation_execution_qualified, bool) or (
        simulation_execution_qualified
        != (simulation_execution_ready and not execution_blockers)
    ):
        raise ValueError(
            "simulation_execution_qualified must equal simulation readiness with no blockers"
        )
    if robot_model_id == "fanuc_m710id_70":
        if effort_limits.shape != (len(joint_names),):
            raise ValueError("M-710 engineering replay requires six explicit finite effort limits")
        if stiffness.shape != (len(joint_names),) or damping.shape != (len(joint_names),):
            raise ValueError("M-710 engineering replay requires six explicit drive gains")
        if (
            not np.all(np.isfinite(stiffness))
            or not np.all(np.isfinite(damping))
            or np.any(stiffness <= 0.0)
            or np.any(damping <= 0.0)
        ):
            raise ValueError("M-710 drive stiffness and damping must be finite and positive")
        execution_identity = str(plan.get("execution_asset_fingerprint_sha256", "")).strip()
        if len(execution_identity) != 64 or any(
            character not in "0123456789abcdef" for character in execution_identity
        ):
            raise ValueError("M-710 plan requires a complete execution asset fingerprint")
        if simulation_execution_ready == bool(execution_blockers):
            raise ValueError(
                "M-710 simulation readiness and blocker list are inconsistent"
            )
        scene_primitives = _declared_scene_primitives(plan)
        if scene_primitives is None:
            raise ValueError("M-710 replay requires frozen layout-bound scene primitives")
        cartons = [item for item in scene_primitives if item["category"] == "carton"]
        if not cartons or not all(item["dynamic"] for item in cartons):
            raise ValueError("M-710 replay requires all current cartons as dynamic rigid bodies")
        if sum(item["name"] == str(segment.get("target", "")) for item in cartons) != 1:
            raise ValueError("M-710 replay target must identify exactly one dynamic carton")

    validation_cfg = cfg.get("simulation_validation", {})
    robot_link_dynamics = (
        copy.deepcopy(official_model["link_dynamics"])
        if official_model is not None
        else _validated_link_dynamics(validation_cfg.get("robot_link_dynamics", []))
    )
    if robot_model_id == "fanuc_m710id_70":
        expected_links = (
            {"base_link", *(f"J{index}_link" for index in range(1, 7))}
            if official_model is None
            else {item["link"] for item in official_model["link_dynamics"]}
        )
        actual_links = {item["link"] for item in robot_link_dynamics}
        if actual_links != expected_links:
            raise ValueError(
                "M-710 replay requires mass properties for every declared physical robot link"
            )
    footprint_size = np.asarray(
        validation_cfg.get("vacuum_footprint_size_m", [0.30, 0.40]), dtype=float
    )
    if footprint_size.shape != (2,) or not np.all(np.isfinite(footprint_size)) or np.any(footprint_size <= 0.0):
        raise ValueError("vacuum footprint must contain two finite positive dimensions")
    effective_seal_area = validation_cfg.get("vacuum_effective_seal_area_m2")
    if effective_seal_area is not None:
        effective_seal_area = float(effective_seal_area)
        geometric_area = float(np.prod(footprint_size))
        if not np.isfinite(effective_seal_area) or not 0.0 < effective_seal_area <= geometric_area:
            raise ValueError("effective vacuum seal area must be positive and no larger than the footprint")
    def optional_positive(name: str):
        value = validation_cfg.get(name)
        if value is None:
            return None
        result = float(value)
        if not np.isfinite(result) or result <= 0.0:
            raise ValueError(f"{name} must be null or finite and positive")
        return result

    product_model = str(validation_cfg.get("vacuum_product_model", "")).strip()
    cup_model = str(validation_cfg.get("vacuum_cup_model", "")).strip()
    physical_cup_count = int(validation_cfg.get("vacuum_cup_count", 0))
    if not product_model or not cup_model or physical_cup_count <= 0:
        raise ValueError("vacuum product model, cup model, and physical cup count are required")
    suction_mode = str(
        validation_cfg.get(
            "vacuum_suction_mode",
            validation_cfg.get("vacuum_attachment_model", "all_cups"),
        )
    )
    attachment_model = str(validation_cfg.get("vacuum_attachment_model", "all_cups"))
    ideal_independent_mode = suction_mode == IDEAL_INDEPENDENT_CUPS_MODE or (
        attachment_model == IDEAL_INDEPENDENT_CUPS_MODE
    )
    if ideal_independent_mode:
        suction_mode = IDEAL_INDEPENDENT_CUPS_MODE
        attachment_model = IDEAL_INDEPENDENT_CUPS_MODE
    holding_torque = optional_positive("vacuum_holding_torque_nm")
    pull_off_force_per_cup = optional_positive("vacuum_pull_off_force_per_cup_n")
    shear_force_per_cup = optional_positive("vacuum_shear_force_per_cup_n")
    if not ideal_independent_mode and (
        pull_off_force_per_cup is None or shear_force_per_cup is None
    ):
        raise ValueError("per-cup axial pull-off and shear forces are required")
    if (
        robot_model_id == "fanuc_m710id_70"
        and not ideal_independent_mode
        and holding_torque is None
    ):
        raise ValueError("M-710 engineering replay requires a finite vacuum holding torque")
    cup_rows = int(validation_cfg.get("vacuum_cup_rows", 0))
    cup_columns = int(validation_cfg.get("vacuum_cup_columns", 0))
    cup_pitch = np.asarray(validation_cfg.get("vacuum_cup_pitch_m", []), dtype=float)
    cup_radius = float(validation_cfg.get("vacuum_cup_radius_m", 0.0))
    zone_count = int(validation_cfg.get("vacuum_zone_count", 1))
    if (
        cup_rows <= 0
        or cup_columns <= 0
        or cup_rows * cup_columns != physical_cup_count
        or cup_pitch.shape != (2,)
        or not np.all(np.isfinite(cup_pitch))
        or np.any(cup_pitch <= 0.0)
        or not np.isfinite(cup_radius)
        or cup_radius <= 0.0
        or zone_count <= 0
        or cup_columns % zone_count != 0
    ):
        raise ValueError("vacuum cup grid, radius, count, and zones are inconsistent")
    width_offsets = (np.arange(cup_rows) - 0.5 * (cup_rows - 1)) * cup_pitch[0]
    length_offsets = (np.arange(cup_columns) - 0.5 * (cup_columns - 1)) * cup_pitch[1]
    cup_centers_tool_yz = np.asarray(
        [(length, -width) for length in length_offsets for width in width_offsets],
        dtype=float,
    )
    ideal_cup_selection = None
    if ideal_independent_mode:
        ideal_cup_selection = _validated_ideal_cup_selection(
            segment,
            physical_cup_count=physical_cup_count,
            path=path,
        )
        active_cup_indices = tuple(ideal_cup_selection["commanded_active_indices"])
    else:
        active_cup_indices = tuple(int(index) for index in segment.get("sealed_cup_indices", []))
        if attachment_model == "per_sealed_cup" and not active_cup_indices:
            raise ValueError("per-sealed-cup attachment requires sealed cup evidence in the plan")
        if not active_cup_indices:
            active_cup_indices = tuple(range(physical_cup_count))
    if len(set(active_cup_indices)) != len(active_cup_indices) or any(
        index < 0 or index >= physical_cup_count for index in active_cup_indices
    ):
        raise ValueError("sealed cup indices must be unique and inside the physical cup grid")
    active_cup_count = len(active_cup_indices)
    solver_attachment_model = (
        "single_same_body_fixed_constraint_after_actual_contact"
        if ideal_independent_mode
        else str(validation_cfg.get("vacuum_solver_attachment_model", "per_sealed_cup"))
    )
    solver_position_iterations = int(
        validation_cfg.get("vacuum_solver_position_iterations", 32)
    )
    solver_velocity_iterations = int(
        validation_cfg.get("vacuum_solver_velocity_iterations", 8)
    )
    if solver_position_iterations <= 0 or solver_velocity_iterations <= 0:
        raise ValueError("vacuum solver iteration counts must be positive")
    if solver_attachment_model not in {
        "per_sealed_cup",
        "equivalent_center_of_pressure",
        "equivalent_zone_row_band_centers",
        "single_same_body_fixed_constraint_after_actual_contact",
    }:
        raise ValueError("vacuum solver attachment model is unsupported")
    active_cup_centers = cup_centers_tool_yz[np.asarray(active_cup_indices, dtype=int)]
    if solver_attachment_model in {
        "equivalent_center_of_pressure",
        "single_same_body_fixed_constraint_after_actual_contact",
    }:
        solver_attachment_offsets = np.mean(active_cup_centers, axis=0, keepdims=True)
        solver_attachment_cup_counts = [active_cup_count]
    elif solver_attachment_model == "equivalent_zone_row_band_centers":
        # Six non-collinear solver points retain the known 3-zone x 2-row-band
        # spatial support without creating 72 nearly coincident rigid joints.
        # This is a numerical attachment representation; physical force and
        # moment qualification continues to use the full sealed-cup evidence.
        columns_per_zone = cup_columns // zone_count
        row_split = cup_rows // 2
        grouped_centers: list[np.ndarray] = []
        solver_attachment_cup_counts = []
        for zone_index in range(zone_count):
            for row_band in range(2):
                group_indices = [
                    index
                    for index in active_cup_indices
                    if (index // cup_rows) // columns_per_zone == zone_index
                    and (0 if (index % cup_rows) < row_split else 1) == row_band
                ]
                if group_indices:
                    grouped_centers.append(
                        np.mean(cup_centers_tool_yz[np.asarray(group_indices, dtype=int)], axis=0)
                    )
                    solver_attachment_cup_counts.append(len(group_indices))
        if len(grouped_centers) < 3:
            raise ValueError("zone-row-band solver model requires at least three populated groups")
        solver_attachment_offsets = np.asarray(grouped_centers, dtype=float)
    else:
        solver_attachment_offsets = active_cup_centers
        solver_attachment_cup_counts = [1] * active_cup_count
    holding_force = (
        None if ideal_independent_mode else pull_off_force_per_cup * active_cup_count
    )
    shear_force = (
        None if ideal_independent_mode else shear_force_per_cup * active_cup_count
    )
    hardware_maximum_holding_force = (
        None
        if pull_off_force_per_cup is None
        else pull_off_force_per_cup * physical_cup_count
    )
    hardware_maximum_shear_force = (
        None
        if shear_force_per_cup is None
        else shear_force_per_cup * physical_cup_count
    )
    catalogue_theoretical_force = optional_positive(
        "vacuum_catalog_theoretical_total_force_n_at_minus_60_kpa"
    )
    max_grip_distance = float(validation_cfg.get("vacuum_max_grip_distance_m", 0.03))
    cup_compression = float(validation_cfg.get("vacuum_cup_compression_m", 0.0))
    capture_tolerance = float(
        validation_cfg.get("vacuum_surface_gripper_capture_tolerance_m", 0.0)
    )
    physical_positive_values = [max_grip_distance, cup_compression]
    if holding_force is not None:
        physical_positive_values.append(holding_force)
    if any(
        not np.isfinite(value) or value <= 0.0 for value in physical_positive_values
    ) or not np.isfinite(capture_tolerance) or capture_tolerance < 0.0:
        raise ValueError("vacuum total force and grip distance must be finite and positive")
    if robot_model_id == "fanuc_m20id_35" and not np.isclose(
        max_grip_distance, cup_compression + capture_tolerance
    ):
        raise ValueError("legacy surface gripper distance must equal cup compression plus solver tolerance")
    max_normal_misalignment = float(
        validation_cfg.get("vacuum_max_normal_misalignment_rad", np.deg2rad(5.0))
    )
    maximum_contact_penetration = float(
        validation_cfg.get("vacuum_maximum_contact_penetration_m", 1e-5)
    )
    if (
        not np.isfinite(max_normal_misalignment)
        or not 0.0 <= max_normal_misalignment < 0.5 * np.pi
        or not np.isfinite(maximum_contact_penetration)
        or maximum_contact_penetration < 0.0
    ):
        raise ValueError("vacuum contact angle and penetration tolerances are invalid")
    tool_cfg = cfg.get("tool", {})
    geometry_cfg = tool_cfg.get("geometry", {}) if isinstance(tool_cfg, dict) else {}
    declared_tool_mass_policy = str(tool_cfg.get("fixed_mass_accounting_policy", ""))
    declared_tool_attachment_link = str(tool_cfg.get("attach_to_link", ""))
    independent_tool_rigid_body_required = tool_cfg.get(
        "independent_tool_rigid_body_required"
    )
    tool_frame_contract = tool_cfg.get("frame_contract")
    validation_frame_contract = validation_cfg.get("vacuum_frame_contract")
    mass_accounting_link = (
        "J6_link"
        if official_model is None
        else official_model["mass_accounting_link"]
    )
    if robot_model_id == "fanuc_m710id_70":
        allowed_attachment_links = {mass_accounting_link}
        if official_model is not None:
            allowed_attachment_links.update(
                link
                for link in (
                    official_model["flange_link"],
                    official_model["fanuc_flange_link"],
                )
                if link
            )
        if (
            "FIXED_TOOL_COMBINED_INTO_" not in declared_tool_mass_policy
            or not declared_tool_mass_policy.endswith("_RIGID_BODY_EXACTLY_ONCE")
            or declared_tool_attachment_link not in allowed_attachment_links
            or independent_tool_rigid_body_required is not False
        ):
            raise ValueError(
                "M-710 tool topology must declare one fixed mass contribution combined exactly once"
            )
        if (
            not isinstance(tool_frame_contract, dict)
            or tool_frame_contract != validation_frame_contract
        ):
            raise ValueError("M-710 tool frame contract must be explicit and consistent")
    gripper_mass = float(tool_cfg.get("mass_kg", validation_cfg.get("vacuum_gripper_mass_kg", 0.0)))
    gripper_com = np.asarray(
        tool_cfg.get("com_xyz_m", validation_cfg.get("vacuum_center_of_mass_from_flange_m", [])), dtype=float
    )
    gripper_inertia = np.asarray(
        tool_cfg.get("inertia_tensor_com_kg_m2", validation_cfg.get("vacuum_inertia_at_com_kg_m2", [])), dtype=float
    )
    gripper_inertia_eigenvalues = (
        np.linalg.eigvalsh(gripper_inertia)
        if gripper_inertia.shape == (3, 3)
        else np.asarray([])
    )
    flange_origin_step = np.asarray(
        geometry_cfg.get("flange_origin_step_mm", validation_cfg.get("vacuum_flange_origin_step_mm", [])), dtype=float
    )
    step_from_tool_rotation = np.asarray(
        geometry_cfg.get("step_from_tool_rotation_matrix", validation_cfg.get("vacuum_step_from_tool_rotation_matrix", [])), dtype=float
    )
    if (
        not np.isfinite(gripper_mass)
        or gripper_mass <= 0.0
        or gripper_com.shape != (3,)
        or not np.all(np.isfinite(gripper_com))
        or gripper_inertia.shape != (3, 3)
        or not np.all(np.isfinite(gripper_inertia))
        or not np.allclose(gripper_inertia, gripper_inertia.T, atol=1e-10, rtol=0.0)
        or gripper_inertia_eigenvalues.shape != (3,)
        or np.any(gripper_inertia_eigenvalues <= 0.0)
        or 2.0 * np.max(gripper_inertia_eigenvalues)
        > np.sum(gripper_inertia_eigenvalues) + 1e-10
        or flange_origin_step.shape != (3,)
        or not np.all(np.isfinite(flange_origin_step))
        or step_from_tool_rotation.shape != (3, 3)
        or not np.all(np.isfinite(step_from_tool_rotation))
        or not np.allclose(step_from_tool_rotation.T @ step_from_tool_rotation, np.eye(3))
        or not np.isclose(np.linalg.det(step_from_tool_rotation), 1.0)
    ):
        raise ValueError("gripper mass properties and STEP-to-tool transform are required")
    task_tcp_from_flange = float(
        validation_cfg.get(
            "vacuum_task_tcp_from_flange_m",
            tool_cfg.get(
                "planner_tool_length_m",
                robot.get("tool_length", 0.20 if robot_model_id == "fanuc_m20id_35" else 0.0),
            ),
        )
    )
    physical_face_from_flange = float(
        validation_cfg.get(
            "vacuum_physical_uncompressed_face_from_flange_m",
            task_tcp_from_flange,
        )
    )
    compressed_contact_from_flange = float(
        validation_cfg.get(
            "vacuum_compressed_contact_plane_from_flange_m",
            physical_face_from_flange - cup_compression,
        )
    )
    if (
        not np.isfinite(task_tcp_from_flange)
        or not np.isfinite(physical_face_from_flange)
        or not np.isfinite(compressed_contact_from_flange)
        or min(task_tcp_from_flange, physical_face_from_flange, compressed_contact_from_flange) <= 0.0
    ):
        raise ValueError("task TCP and physical suction contact planes must be finite and positive")
    if robot_model_id == "fanuc_m710id_70" and np.isclose(
        task_tcp_from_flange, physical_face_from_flange, atol=1e-12, rtol=0.0
    ):
        raise ValueError("M-710 replay must distinguish the virtual task TCP from the physical cup face")
    if robot_model_id == "fanuc_m710id_70" and not (
        task_tcp_from_flange > physical_face_from_flange > compressed_contact_from_flange
    ):
        raise ValueError(
            "M-710 task TCP, uncompressed cup face, and compressed contact plane are out of order"
        )
    if robot_model_id == "fanuc_m710id_70" and not np.isclose(
        compressed_contact_from_flange,
        physical_face_from_flange - cup_compression,
        atol=1e-12,
        rtol=0.0,
    ):
        raise ValueError("M-710 compressed contact plane must equal cup face minus compression")

    grasp_body_path_suffix = str(
        robot.get(
            "isaac_grasp_body_path_suffix",
            (
                "Geometry/base_link/J1_link/J2_link/J3_link/J4_link/J5_link/J6_link"
                if official_model is None
                else mass_accounting_link
            ),
        )
    ).strip("/")
    flange_offset_from_grasp_body = np.asarray(
        robot.get("flange_offset_from_grasp_body_m", [0.09, 0.0, 0.0]),
        dtype=float,
    )
    if (
        not grasp_body_path_suffix
        or flange_offset_from_grasp_body.shape != (3,)
        or not np.all(np.isfinite(flange_offset_from_grasp_body))
    ):
        raise ValueError("Isaac grasp body and finite flange offset are required")

    mass_accounting: dict[str, Any] | None = None
    if robot_model_id == "fanuc_m710id_70":
        grasp_body_link = str(robot.get("isaac_grasp_body_link", mass_accounting_link))
        if grasp_body_link != mass_accounting_link:
            raise ValueError("M-710 fixed-tool mass accounting requires the final moving link")
        original_robot_mass = float(sum(item["mass_kg"] for item in robot_link_dynamics))
        grasp_record = next(item for item in robot_link_dynamics if item["link"] == grasp_body_link)
        grasp_body_from_tool = np.asarray(
            tool_cfg.get("T_mass_accounting_link_tool", []), dtype=float
        )
        if grasp_body_from_tool.shape != (4, 4):
            if official_model is not None:
                raise ValueError(
                    "official M-710 mass accounting requires T_mass_accounting_link_tool"
                )
            grasp_body_from_tool = np.eye(4)
            grasp_body_from_tool[:3, 3] = flange_offset_from_grasp_body
        if (
            not np.all(np.isfinite(grasp_body_from_tool))
            or not np.allclose(
                grasp_body_from_tool[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12, rtol=0.0
            )
            or not np.allclose(
                grasp_body_from_tool[:3, :3].T @ grasp_body_from_tool[:3, :3],
                np.eye(3),
                atol=1e-12,
                rtol=0.0,
            )
            or not np.isclose(
                np.linalg.det(grasp_body_from_tool[:3, :3]), 1.0, atol=1e-12, rtol=0.0
            )
        ):
            raise ValueError("T_mass_accounting_link_tool must be a finite proper SE(3) transform")
        tool_com_in_grasp_body, tool_inertia_in_grasp_body = _transform_mass_properties(
            gripper_com,
            gripper_inertia,
            grasp_body_from_tool,
        )
        combined_mass, combined_com, combined_inertia = _combine_fixed_mass_properties(
            grasp_record["mass_kg"],
            np.asarray(grasp_record["com_xyz_m"], dtype=float),
            np.asarray(grasp_record["inertia_at_com_kg_m2"], dtype=float),
            gripper_mass,
            tool_com_in_grasp_body,
            tool_inertia_in_grasp_body,
        )
        applied_link_dynamics: list[dict[str, Any]] = []
        for item in robot_link_dynamics:
            applied = dict(item)
            if item["link"] == grasp_body_link:
                applied.update(
                    mass_kg=combined_mass,
                    com_xyz_m=combined_com.tolist(),
                    inertia_at_com_kg_m2=combined_inertia.tolist(),
                    source_status=(
                        f"{item['source_status']}; FIXED_TOOL_MASS_COMBINED_EXACTLY_ONCE"
                    ),
                )
            applied_link_dynamics.append(applied)
        robot_link_dynamics = applied_link_dynamics
        applied_mass = float(sum(item["mass_kg"] for item in robot_link_dynamics))
        if not np.isclose(applied_mass, original_robot_mass + gripper_mass, atol=1e-10, rtol=0.0):
            raise ValueError("M-710 robot/tool mass accounting is inconsistent")
        mass_accounting = {
            "policy": declared_tool_mass_policy,
            "declared_policy": declared_tool_mass_policy,
            "applied_policy": declared_tool_mass_policy,
            "grasp_body_link": grasp_body_link,
            "declared_attachment_link": declared_tool_attachment_link,
            "independent_tool_rigid_body_created": False,
            "source_robot_mass_kg": original_robot_mass,
            "tool_mass_kg": gripper_mass,
            "applied_articulation_mass_kg": applied_mass,
            "tool_com_in_grasp_body_m": tool_com_in_grasp_body.tolist(),
            "tool_inertia_at_com_in_grasp_body_kg_m2": tool_inertia_in_grasp_body.tolist(),
            "T_grasp_body_tool": grasp_body_from_tool.tolist(),
        }
    physics_contract = dict(validation_cfg.get("physics", {}))
    rendering_contract = dict(validation_cfg.get("rendering", {}))
    required_post_release_settle_seconds = execution.get("post_release_settle_seconds")
    if robot_model_id == "fanuc_m710id_70":
        physics_contract = _validated_physics_contract(
            physics_contract,
            solver_position_iterations=solver_position_iterations,
            solver_velocity_iterations=solver_velocity_iterations,
        )
        rendering_contract = _validated_rendering_contract(
            rendering_contract,
            physics_time_step_s=physics_contract["physics_time_step_s"],
        )
        if required_post_release_settle_seconds is None:
            raise ValueError("M-710 replay requires a post-release settling duration")
        required_post_release_settle_seconds = float(required_post_release_settle_seconds)
        if (
            not np.isfinite(required_post_release_settle_seconds)
            or required_post_release_settle_seconds < 0.0
        ):
            raise ValueError("M-710 post-release settling duration must be finite and non-negative")

    base_position = np.asarray(robot.get("base_position", [0.0, 0.0, 0.0]), dtype=float)
    dock_value = segment.get("amr_dock_position")
    robot_mount = cfg.get("amr", {}).get("robot_mount_position")
    if dock_value is not None and robot_mount is not None:
        base_position = np.asarray(dock_value, dtype=float) + np.asarray(robot_mount, dtype=float)

    conveyor_contract = dict(cfg.get("simulation_validation", {}).get("conveyor", {}))
    if (
        robot_model_id == "fanuc_m710id_70"
        and conveyor_contract.get("enabled") is True
        and conveyor_contract.get("start_policy") == "after_release_retreat"
        and source_release_retreat_time is None
    ):
        raise ValueError("M-710 after_release_retreat conveyor requires release_retreat_index")

    actual_state_gates = dict(validation_cfg.get("actual_state_gates", {}))
    actual_state_gate_defaults = {
        "maximum_contact_wait_s": 0.50,
        "maximum_support_wait_s": 0.75,
        "maximum_free_transit_wait_s": 1.0,
        "maximum_release_clearance_wait_s": 1.0,
        "support_max_gap_m": 0.003,
        "support_maximum_penetration_m": 0.001,
        "support_minimum_footprint_overlap_ratio": 0.90,
        "support_max_tilt_rad": float(np.deg2rad(5.0)),
        "support_max_linear_speed_m_s": 0.03,
        "support_max_angular_speed_rad_s": 0.08,
    }
    for name, default in actual_state_gate_defaults.items():
        value = float(actual_state_gates.get(name, default))
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"actual-state gate {name} must be finite and non-negative")
        actual_state_gates[name] = value
    if not 0.0 < actual_state_gates["support_minimum_footprint_overlap_ratio"] <= 1.0:
        raise ValueError("support footprint overlap ratio must be in (0, 1]")

    place_evidence = _validated_place_evidence(segment,
        scene_primitives=scene_primitives if robot_model_id == "fanuc_m710id_70" else None)

    metadata = {
        "robot_model": robot_model_id,
        "source_plan_sha256": _canonical_digest(plan),
        "merged_configuration_sha256": _canonical_digest(cfg),
        "joint_names": list(joint_names),
        "urdf_path": (
            str(robot["urdf_path"])
            if official_model is None
            else official_model["expanded_urdf"]
        ),
        "base_position_m": base_position.tolist(),
        "base_rpy_rad": list(robot.get("base_rpy", [0.0, 0.0, 0.0])),
        "tool_length_m": task_tcp_from_flange,
        "isaac_grasp_body_path_suffix": grasp_body_path_suffix,
        "flange_offset_from_grasp_body_m": flange_offset_from_grasp_body.tolist(),
        "segment_index": int(segment_index),
        "pick_index": int(segment.get("pick_index", segment_index)),
        "target": str(segment.get("target", "unknown")),
        "offline_planning_time_seconds": float(plan.get("planning_time_seconds", 0.0)),
        "source_waypoint_count": int(len(path)),
        "controller_period_seconds": float(controller_period_seconds),
        "command_count": int(len(command_times)),
        "duration_seconds": float(command_times[-1]),
        "motion_duration_seconds": float(replay_motion_times[-1]),
        "source_motion_duration_seconds": float(source_motion_times[-1]),
        "isaac_replay_time_scale": replay_time_scale,
        "timing_audit": {**audit, "isaac_replay_time_scale": replay_time_scale},
        "limits_source": limits.source,
        "joint_velocity_limits_rad_s": limits.velocity.tolist(),
        "joint_effort_limits_nm": effort_limits.tolist(),
        "joint_drive_stiffness_nm_rad": stiffness.tolist(),
        "joint_drive_damping_nm_s_rad": damping.tolist(),
        "execution_asset_fingerprint_sha256": plan.get("execution_asset_fingerprint_sha256"),
        "execution_qualified": execution_qualified,
        "simulation_execution_qualified": simulation_execution_qualified,
        "simulation_execution_ready": simulation_execution_ready,
        "execution_blockers": execution_blockers,
        "machine_qualified": machine_qualified,
        "machine_qualification_warnings": list(machine_qualification_warnings),
        "robot_link_dynamics": robot_link_dynamics,
        "robot_dynamics_source": (
            "legacy_engineering_configuration"
            if official_model is None
            else "official_fanuc_description_fixed_commit"
        ),
        "official_model_manifest": (
            None if official_model is None else official_model["manifest"]
        ),
        "official_model_manifest_sha256": (
            official_manifest_file_sha256
        ),
        "official_model_manifest_canonical_sha256": (
            None if official_model is None else official_model["manifest_sha256"]
        ),
        "robot_srdf_path": declared_srdf_path or None,
        "robot_srdf_sha256": declared_srdf_sha256,
        "joint_position_lower_limits_rad": (
            [] if official_model is None else official_model["joint_lower_rad"].tolist()
        ),
        "joint_position_upper_limits_rad": (
            [] if official_model is None else official_model["joint_upper_rad"].tolist()
        ),
        "robot_tool_mass_accounting": mass_accounting,
        "physics": physics_contract,
        "required_post_release_settle_seconds": required_post_release_settle_seconds,
        "grasp_arrival_time_seconds": grasp_arrival_time,
        "pre_grasp_controller_settle_seconds": pre_grasp_settle_seconds,
        "grasp_time_seconds": grasp_time,
        "vacuum_establish_seconds": vacuum_establish_seconds,
        "release_arrival_time_seconds": release_arrival_time,
        "release_time_seconds": release_time,
        "release_hold_seconds": release_seconds,
        "loaded_motion_time_scale": loaded_motion_time_scale,
        "post_release_motion_limit_scale": post_release_motion_limit_scale,
        "release_retreat_time_seconds": release_retreat_time,
        "place_center_m": place_evidence["place_center_m"],
        "release_center_m": place_evidence["release_center_m"],
        "place_surface": place_evidence["place_surface"],
        "selected_place_support_names": list(segment.get("place", {}).get("support_names", [place_evidence["place_surface"]])),
        "collision_policy": dict(plan.get("collision_policy", {})),
        "stack_carton_names": list(plan.get("stack_carton_names", [])),
        "row_selection": dict(plan.get("row_selection", {})),
        "completed_carton_ids": list(plan.get("completed_carton_ids", [])),
        "handed_off_ids": list(plan.get("handed_off_ids", [])),
        "trajectory_stage_ranges": dict(segment.get("stage_ranges", {})),
        "free_transit_start_time_seconds": (
            float(replay_motion_times[segment["stage_ranges"]["extraction"][1]])
            + pre_grasp_settle_seconds + vacuum_establish_seconds
            if "extraction" in segment.get("stage_ranges", {}) else None),
        "stage_windows": [
            {"stage": name,
             "start_time_s": float(replay_motion_times[interval[0]])
                + (pre_grasp_settle_seconds + vacuum_establish_seconds if interval[0] > segment.get("grasp_index", len(path)) else 0)
                + (release_seconds if interval[0] > segment.get("release_index", len(path)) else 0),
             "end_time_s": float(replay_motion_times[interval[1]])
                + (pre_grasp_settle_seconds + vacuum_establish_seconds if interval[1] > segment.get("grasp_index", len(path)) else 0)
                + (release_seconds if interval[1] > segment.get("release_index", len(path)) else 0)}
            for name, interval in segment.get("stage_ranges", {}).items()],
        "joint_gravity_feedforward_enabled": bool(cfg.get("execution", {}).get("joint_gravity_feedforward_enabled", False)),
        "joint_velocity_feedforward_enabled": bool(cfg.get("execution", {}).get("joint_velocity_feedforward_enabled", False)),
        "attached_payload_gravity_feedforward_enabled": bool(cfg.get("execution", {}).get("attached_payload_gravity_feedforward_enabled", False)),
        "planned_place_support_audit": place_evidence["planned_support_audit"],
        "planned_actual_box_pose_world": place_evidence["actual_box_pose_world"],
        "free_fall_height_m": float(segment.get("free_fall_height_m", 0.0)),
        "collision_geometry": str(plan.get("collision_geometry", "urdf_collision_mesh")),
        "scene_primitives": _build_scene_primitives(plan, cfg, segment_index),
        "camera": dict(cfg.get("simulation_validation", {}).get("camera", {})),
        "rendering": rendering_contract,
        "conveyor": conveyor_contract,
        "actual_state_gates": actual_state_gates,
        "gripper": {
            "qualified_rigid_collision_boxes_tool_frame": list(cfg.get("tool", {}).get("qualified_rigid_collision_boxes_tool_frame", [])),
            "suction_mode": suction_mode,
            "holding_capacity_assumption": (
                HOLDING_CAPACITY_ASSUMPTION if ideal_independent_mode else None
            ),
            "enforce_vacuum_force_capacity": not ideal_independent_mode,
            "enforce_vacuum_break_force": not ideal_independent_mode,
            "enforce_vacuum_break_torque": not ideal_independent_mode,
            "load_bearing_minimum_cup_count": None,
            "product_model": product_model,
            "cup_model": cup_model,
            "physical_cup_count": physical_cup_count,
            "active_sealed_cup_count": active_cup_count,
            "active_sealed_cup_indices": list(active_cup_indices),
            "mask_bit_order_cup_ids": (
                []
                if ideal_cup_selection is None
                else ideal_cup_selection["mask_bit_order_cup_ids"]
            ),
            "geometrically_eligible_mask": (
                []
                if ideal_cup_selection is None
                else ideal_cup_selection["geometrically_eligible_mask"]
            ),
            "commanded_active_mask": (
                []
                if ideal_cup_selection is None
                else ideal_cup_selection["commanded_active_mask"]
            ),
            "planned_fk_contact_mask": (
                []
                if ideal_cup_selection is None
                else ideal_cup_selection["planned_fk_contact_mask"]
            ),
            "actual_contact_mask": (
                []
                if ideal_cup_selection is None
                else ideal_cup_selection["actual_contact_mask"]
            ),
            "actual_contact_mask_source": (
                None
                if ideal_cup_selection is None
                else ideal_cup_selection["actual_contact_mask_source"]
            ),
            "target_id": (
                None if ideal_cup_selection is None else ideal_cup_selection["target_id"]
            ),
            "target_face": (
                None if ideal_cup_selection is None else ideal_cup_selection["target_face"]
            ),
            "planner_contact_pose_source": (
                None if ideal_cup_selection is None else ideal_cup_selection["pose_source"]
            ),
            "sealed_cups_per_zone": list(segment.get("sealed_cups_per_zone", [])),
            "cup_rows": cup_rows,
            "cup_columns": cup_columns,
            "cup_pitch_m": cup_pitch.tolist(),
            "cup_radius_m": cup_radius,
            "cup_centers_tool_yz_m": cup_centers_tool_yz.tolist(),
            "holding_force_n": holding_force,
            "holding_force_total_n": holding_force,
            "hardware_maximum_holding_force_total_n": hardware_maximum_holding_force,
            "holding_force_derivation": (
                "NOT_APPLICABLE_IDEAL_HOLDING_CAPACITY_ASSUMPTION"
                if ideal_independent_mode
                else "pull_off_force_per_cup_n_times_active_sealed_cup_count"
            ),
            "holding_torque_nm": holding_torque,
            "pull_off_force_per_cup_n": pull_off_force_per_cup,
            "shear_force_per_cup_n": shear_force_per_cup,
            "catalogue_theoretical_total_force_n_at_minus_60_kpa": (
                catalogue_theoretical_force
            ),
            "footprint_size_m": footprint_size.tolist(),
            "effective_seal_area_m2": effective_seal_area,
            "limits_calibrated": bool(
                cfg.get("simulation_validation", {}).get("vacuum_limits_calibrated", False)
            ),
            "shear_force_n": shear_force,
            "shear_force_total_n": shear_force,
            "hardware_maximum_shear_force_total_n": hardware_maximum_shear_force,
            "max_grip_distance_m": max_grip_distance,
            "max_normal_misalignment_rad": max_normal_misalignment,
            "maximum_contact_penetration_m": maximum_contact_penetration,
            "physical_cup_compression_m": cup_compression,
            "task_tcp_from_flange_m": task_tcp_from_flange,
            "physical_uncompressed_face_from_flange_m": physical_face_from_flange,
            "compressed_contact_plane_from_flange_m": compressed_contact_from_flange,
            "frame_contract": copy.deepcopy(tool_frame_contract),
            "virtual_tcp_beyond_uncompressed_face_m": task_tcp_from_flange - physical_face_from_flange,
            "surface_gripper_capture_tolerance_m": capture_tolerance,
            "attachment_model": attachment_model,
            "attachment_point_count": active_cup_count,
            "solver_attachment_model": solver_attachment_model,
            "solver_attachment_offsets_tool_yz_m": solver_attachment_offsets.tolist(),
            "solver_attachment_cup_counts": solver_attachment_cup_counts,
            "simulation_attachment_point_count": int(len(solver_attachment_offsets)),
            "solver_position_iterations": solver_position_iterations,
            "solver_velocity_iterations": solver_velocity_iterations,
            "gripper_mass_kg": gripper_mass,
            "center_of_mass_from_flange_m": gripper_com.tolist(),
            "inertia_at_com_kg_m2": gripper_inertia.tolist(),
            "flange_origin_step_mm": flange_origin_step.tolist(),
            "step_from_tool_rotation_matrix": step_from_tool_rotation.tolist(),
            "outer_size_m": list(geometry_cfg.get("outer_size_m", validation_cfg.get("vacuum_outer_size_m", []))),
            "step_path": geometry_cfg.get("step_path", validation_cfg.get("vacuum_step_path")),
            "visual_mesh_path": geometry_cfg.get("visual_mesh_path", validation_cfg.get("vacuum_visual_mesh_path")),
            "collision_mesh_path": geometry_cfg.get("collision_mesh_path", validation_cfg.get("vacuum_collision_mesh_path")),
            "mass_properties_path": geometry_cfg.get("mass_properties_path", validation_cfg.get("vacuum_mass_properties_path")),
            "flange_transform_confidence": geometry_cfg.get(
                "flange_transform_confidence", validation_cfg.get("vacuum_flange_transform_confidence")
            ),
            "zone_count": zone_count,
            "zone_assignment_confirmed": False,
            "model_source": (
                "step_geometry_with_ideal_independent_cups_and_actual_contact_gate"
                if ideal_independent_mode
                else "step_geometry_with_per_cup_forces_and_provisional_zone_mapping"
            ),
        },
    }
    if robot_model_id == "fanuc_m710id_70":
        metadata["m710_execution_preflight"] = copy.deepcopy(preflight)
        metadata["m710_replay_contract"] = m710_replay_contract
    return IsaacReplayBundle(command_times, commands, metadata)
