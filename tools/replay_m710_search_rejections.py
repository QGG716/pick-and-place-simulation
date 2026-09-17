"""Replay bounded saved rejection examples through the production connector."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.geometry import OBB
from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.collision_policy import PhysicsCheckedStackTracker
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.planning_profile import DEFAULT_MOTION
from unloading_sim.unloading_sequence import RowSequencePolicy, RowUnloadingState


def box(record):
    pose = np.asarray(record["pose_world"])
    return OBB(pose[:3, 3], record["half_extents_m"], pose[:3, :3], record["name"], record["category"])


def probe_source_part(result, part_record):
    """Optional offline tessellated CAD check; never substitutes runtime geometry."""
    import coal
    from unloading_sim.tool_geometry import audit_tool_geometry
    from unloading_sim.pinocchio_backend import _transform
    metadata = json.loads(part_record.read_text())
    geometry = audit_tool_geometry(ROOT)
    mesh_path = Path(metadata["mesh_path"])
    coverage_bytes = (ROOT / "assets/grippers/shanghai_wantai_three_zone/geometry_coverage.json").read_bytes()
    coverage_hashes = {hashlib.sha256(coverage_bytes).hexdigest(),
                       hashlib.sha256(coverage_bytes.replace(b"\r\n", b"\n")).hexdigest()}
    if (hashlib.sha256(mesh_path.read_bytes()).hexdigest() != metadata["mesh_sha256"]
            or metadata["source_step_sha256"] != geometry["source_step_sha256"]
            or metadata["coverage_sha256"] not in coverage_hashes
            or metadata["mesh_units"] != "millimetre"
            or geometry["rigid_solid_indices"][metadata["tool_part_index"]] != metadata["source_solid_index"]):
        raise ValueError("source part mesh provenance mismatch")
    mesh = coal.MeshLoader().load(str(mesh_path), np.full(3, .001))
    robot = load_layout_motion_policy(DEFAULT_MOTION).layout_validation.layout.robot()
    cases = []
    for row in result["rows"]:
        e = row["example"]
        pair = e["failure"].get("pair", [])
        if not pair or pair[0] != f"tool_rigid_{metadata['tool_part_index']}":
            continue
        other = next((b for b in row["pair_obb_geometry"] if b["name"] == pair[1]), None)
        if other is None:
            continue
        pose = np.asarray(other["pose_world"])
        flange = robot.named_link_frames(e["q_rad"])["flange"]
        rotation = flange[:3, :3] @ np.asarray(geometry["rotation_step_from_tool"]).T
        translation = flange[:3, 3] - rotation @ (np.asarray(geometry["flange_origin_step_mm"]) * .001)
        transform = _transform(coal, rotation, translation)
        obstacle = coal.Box(*(2*np.asarray(other["half_extents_m"])))
        obstacle_transform = _transform(coal, pose[:3, :3], pose[:3, 3])
        distance = float(coal.distance(mesh, transform, obstacle, obstacle_transform,
                                      coal.DistanceRequest(), coal.DistanceResult()))
        contacts = int(coal.collide(mesh, transform, obstacle, obstacle_transform,
                                   coal.CollisionRequest(), coal.CollisionResult()))
        required = e["failure"]["required_pair_clearance_m"]
        cases.append(dict(q_rad=e["q_rad"], pair=pair, stage=e["failure"]["stage"],
            mesh_distance_m=distance, mesh_contacts=contacts, required_pair_clearance_m=required,
            classification="TESSELLATED_SOURCE_CONTACT" if contacts else
                "TESSELLATED_SOURCE_CLEARANCE_SHORTFALL" if distance < required else
                "PROXY_REJECTION_NOT_CONFIRMED_BY_SOURCE_MESH"))
    return dict(part=metadata, cases=cases, current_coverage_sha256=geometry["coverage_evidence_sha256"],
        coverage_binding="EXACT_BYTES_OR_LF_NORMALIZATION_ONLY", runtime_geometry_changed=False,
        scope="OFFLINE_TESSELLATED_SOURCE_SOLID_NOT_ANALYTIC_BREP_PROOF")


def replay(diagnostic, actual_state, *, limit=8, config=DEFAULT_MOTION):
    identity = diagnostic["identity"]
    for name, expected in identity["implementation"]["source_sha256"].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"implementation changed: {name}")
    initial = build_verified_motion_input(load_layout_motion_policy(config))
    rows_state = RowUnloadingState(RowSequencePolicy(row_height_fraction=float(
        initial.policy.data["search_strategy"].get("row_height_fraction", .05))))
    rows_state.rank(initial.cartons, support_graph=initial.support_graph)
    scene = apply_actual_motion_state(initial, actual_state, row_state=rows_state)
    if scene.policy.policy_fingerprint != identity["policy"] or scene.snapshot["scene_fingerprint"] != identity["scene"]:
        raise ValueError("replay input scene/policy identity differs")
    c = _build_automatic_trajectory_connector(scene, scene.policy.layout_validation.layout.robot()).connector
    c.start_planning_request()
    rows = []
    for key, group in sorted(diagnostic["groups"].items(), key=lambda item: -item[1]["count"])[:limit]:
        example = group["examples"][0]
        context = diagnostic["contexts"][example["context"]]
        if hashlib.sha256(json.dumps(context, sort_keys=True).encode()).hexdigest() != example["context"]:
            raise ValueError("saved validation context fingerprint mismatch")
        v = c.robot_state_validator
        v.commanded_cup_mask = context["commanded_cup_mask"]
        v.contact_target_name = context["contact_target_name"]
        v.stack_carton_names = set(context["stack_carton_names"])
        c._state_cache.clear()
        obstacles = [box(item) for item in context["obstacles"]]
        target = None if context["target_contact"] is None else box(context["target_contact"])
        attachment = None
        if context["attachment"] is not None:
            a = context["attachment"]
            payload = box(a["payload"])
            attachment = PhysicalContactAttachment(c.robot,
                RigidAttachment(np.asarray(a["tcp_from_box"]), payload.half_extents, payload.name),
                c.flange_from_virtual_task_tcp, c.flange_from_physical_contact)
        tracker = None
        history = context["initial_proximity"]
        if history is not None:
            if history.get("mode") != "planner_relaxed_physics_checked" or attachment is None:
                raise ValueError("this replayer requires explicit POC stack tracker evidence")
            tracker = PhysicsCheckedStackTracker(attachment.box_at(example["q_rad"]),
                [b for b in obstacles if b.name in history["stack_carton_names"]],
                c.collision_policy, c.collision_margin_m, c.contact_tolerance_m)
            tracker.fully_released = history["fully_released"]
        q = np.asarray(example["q_rad"])
        failure = c._state_failure(q, obstacles, attachment=attachment,
            target_contact=target, support_names=context["support_names"],
            initial_proximity=tracker, stage=context["stage"])
        expected = example["failure"]
        matched = failure is not None and all(failure.get(k) == expected.get(k)
            for k in ("reason", "pair", "classification"))
        shapes = [*obstacles, *v.tool_transform_robot.tool_collision_obbs(q), *v._compliant_boxes(q)]
        if attachment is not None:
            shapes.append(attachment.box_at(q))
        pair = expected.get("pair", [])
        geometry = [dict(name=b.name, pose_world=b.world_from_local.tolist(),
                        half_extents_m=b.half_extents.tolist(),
                        world_aabb_m=[b.corners().min(axis=0).tolist(), b.corners().max(axis=0).tolist()])
                    for b in shapes if b.name in pair]
        limits = np.asarray(c.robot.joint_limits)
        rows.append(dict(group=key, observed_count=group["count"], example=example,
            replay_failure=failure, matched=matched, pair_obb_geometry=geometry,
            joint_margin_rad=np.minimum(q-limits[:, 0], limits[:, 1]-q).tolist(),
            jacobian_condition=float(np.linalg.cond(c.robot.geometric_jacobian(q))),
            original_cad_intersection_proven=False))
    return dict(schema="production_rejection_replay_v1", rows=rows,
                all_matched=bool(rows) and all(r["matched"] for r in rows))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--diagnostics", type=Path, required=True)
    p.add_argument("--actual-state", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--limit", type=int, default=8)
    p.add_argument("--config", default=DEFAULT_MOTION)
    p.add_argument("--part-record", type=Path, help="optional existing source mesh metadata for offline CAD check")
    a = p.parse_args()
    result = replay(json.loads(a.diagnostics.read_text()), json.loads(a.actual_state.read_text()), limit=a.limit, config=a.config)
    if a.part_record is not None:
        result["source_part_probe"] = probe_source_part(result, a.part_record)
    a.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"all_matched": result["all_matched"], "examples": len(result["rows"])}))
    return 0 if result["all_matched"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
