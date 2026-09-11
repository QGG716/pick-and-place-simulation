"""Read-only, bounded endpoint diagnosis of the live layout planning inputs."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.conveyor_placement import PlacementPolicy, generate_conveyor_placements
from unloading_sim.fanuc_m710id70 import target_pose
from unloading_sim.layout_single_carton import (
    _audit_pose, _build_automatic_trajectory_connector, _independent_cup_selection_at_pose,
    build_verified_motion_input, load_layout_motion_policy, virtual_tcp_from_physical_contact,
)
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.workcell_layout import canonical_digest


class PrefixCaptured(Exception):
    pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/validation/m710id70_layout_v1_single_carton.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-placements", type=int, default=3)
    parser.add_argument("--cartesian", action="store_true")
    parser.add_argument("--roll", type=int, default=0)
    parser.add_argument("--contact-z-offset", type=float, default=0.0)
    parser.add_argument("--buffer-check", action="store_true")
    parser.add_argument("--alternate-extraction", action="store_true")
    parser.add_argument("--complete-cartesian", action="store_true")
    parser.add_argument("--avoid-other-receiver-margin", action="store_true")
    parser.add_argument("--summarize", action="store_true")
    args = parser.parse_args()
    if args.summarize:
        data = json.loads(args.output.read_text(encoding="utf-8"))
        for item in data.get("geometry_cases", []):
            print(json.dumps({"key": item["key"], "cups": item["eligible_cup_count"],
                "places": [{"index": p["placement_index"], "min_z_m": p["rigid_tool_min_z_m"],
                    "collisions": p["tool_fixed_or_payload_collision_count"], "first": p["first_collisions"][:1]}
                    for p in item["placements"]]}))
        for item in data.get("endpoint_checks", []):
            print(json.dumps({"receiver": item["placement"]["receiver_names"],
                "failures": item["failure_counts"], "valid": len(item["valid_solutions"]),
                "cartesian": item.get("direct_cartesian", {}).get("failure")}))
        for item in data.get("outward_buffer_checks", []):
            print(json.dumps(item))
        for item in data.get("alternate_extraction_checks", []):
            print(json.dumps(item))
        return 0
    started = time.monotonic()
    policy = load_layout_motion_policy(args.config)
    scene = build_verified_motion_input(policy)
    light = policy.layout_validation.layout.robot()
    target = next(box for box in scene.cartons if box.name == scene.removable_cartons[0])
    supports = tuple(box for box in scene.fixed_components if box.category == "conveyor")
    report = {"scope": "DIAGNOSTIC_ONLY_NO_SCENE_OR_RUNNING_SOURCE_MUTATIONS",
              "config": str(Path(args.config).resolve()), "target": target.name,
              "home_q_rad": policy.layout_validation.initial_q.tolist(),
              "scene_fingerprint": scene.snapshot["scene_fingerprint"],
              "source_hashes": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                for name in ("src/unloading_sim/layout_trajectory.py", "src/unloading_sim/layout_single_carton.py",
                             "src/unloading_sim/conveyor_placement.py")}, "geometry_cases": []}
    def save(event):
        report["elapsed_s"] = time.monotonic() - started
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps({"event": event, "elapsed_s": report["elapsed_s"]}), flush=True)
    placements = generate_conveyor_placements(target, supports,
        preferred_point_world=target.center + [-.8, 0, 0],
        policy=PlacementPolicy(maximum_candidates=36 if args.avoid_other_receiver_margin else 12,
                               contact_tolerance_m=.0002))
    if args.avoid_other_receiver_margin:
        rejected = []
        kept = []
        for placement in placements:
            blocked = [surface.name for surface in scene.fixed_components
                       if surface.name not in placement.receiver_names
                       and placement.payload.intersects_obb(surface, margin=.01)]
            if blocked:
                rejected.append({"candidate": placement.as_dict(), "other_body_margin": blocked})
            else:
                kept.append(placement)
        report["diagnostic_receiver_margin_prefilter"] = rejected
        placements = tuple(kept)
    q0 = np.zeros(light.dof)
    tool0 = light.tool_collision_obbs(q0)
    virtual0 = light.fk(q0)
    physical_poses = {}
    # These offsets change only the candidate contact point within the same
    # actual face. Geometry eligibility is measured, not assumed.
    for face, roll in (("front", 0), ("front", 90), ("front", 180), ("front", 270), ("top", 0), ("top", 90)):
        for offset_z in ([0., .024, .06, .10, .14] if face == "front" else [0.]):
            nominal, _ = target_pose(-float(target.half_extents[0]), 0, 0,
                                     2 * target.half_extents, face, roll)
            physical = target.world_from_local @ nominal
            physical[2, 3] += offset_z
            key = f"{face}_r{roll}_z{offset_z:.3f}"
            physical_poses[key] = physical
            try:
                selection, geometry = _independent_cup_selection_at_pose(
                    physical, target, face, policy.data["suction"], .0002,
                    pose_source="diagnostic_requested_actual_face")
                eligible = None if selection is None else int(sum(selection.commanded_active_mask))
                if eligible is None:
                    eligible = int(np.count_nonzero(geometry.geometrically_eligible_mask))
            except Exception as exc:
                eligible = None
                report.setdefault("geometry_selection_errors", {})[key] = str(exc)
            rigid = RigidAttachment.capture(physical, target)
            variants = []
            for index, placement in enumerate(placements[:args.max_placements]):
                desired = placement.payload.world_from_local @ np.linalg.inv(rigid.tcp_from_box)
                virtual = virtual_tcp_from_physical_contact(desired,
                    policy.tool_frames.flange_from_virtual_task_tcp, policy.tool_frames.flange_from_physical_contact)
                delta = virtual @ np.linalg.inv(virtual0)
                boxes = [box.transformed(delta) for box in tool0]
                collisions = [{"tool": box.name, "obstacle": obstacle.name,
                               "signed_sat_distance_m": box.signed_distance_obb(obstacle)}
                    for box in boxes for obstacle in (*scene.fixed_components, placement.payload)
                    if box.intersects_obb(obstacle, margin=.01)]
                variants.append({"placement_index": index, "receiver_names": list(placement.receiver_names),
                    "box_center_m": placement.payload.center.tolist(),
                    "physical_contact_center_m": desired[:3, 3].tolist(),
                    "rigid_tool_min_z_m": float(min(box.corners()[:, 2].min() for box in boxes)),
                    "tool_rigid_0_bounds_m": [boxes[0].corners().min(axis=0).tolist(), boxes[0].corners().max(axis=0).tolist()],
                    "tool_rigid_0_preplace_bounds_m": [(boxes[0].corners().min(axis=0)+[0,0,.1]).tolist(),
                                                       (boxes[0].corners().max(axis=0)+[0,0,.1]).tolist()],
                    "tool_fixed_or_payload_collision_count": len(collisions), "first_collisions": collisions[:8]})
            report["geometry_cases"].append({"key": key, "eligible_cup_count": eligible, "placements": variants})
    save("geometry_complete")

    build = _build_automatic_trajectory_connector(scene, light)
    if build.connector is None:
        report["connector_failure"] = build.evidence
        save("connector_unavailable")
        return 1
    connector = build.connector
    actual_finish_place = connector._finish_place_branch
    physical = physical_poses[f"front_r{args.roll}_z{args.contact_z_offset:.3f}"]
    virtual = connector.virtual_from_physical(physical)
    seed = int(policy.data["ik"]["seed"])
    attempt = _audit_pose(scene, target, "front", args.roll, physical, virtual, {}, seed,
        True, connector.robot, [], lambda q: connector.validate_unloaded_state(
            q, scene.all_obstacles, target_contact=target, stage="grasp_contact_endpoint"))
    report["grasp_attempt"] = attempt
    save("strict_front0_grasp_complete")
    if not attempt["strict_grasp_candidates"]:
        return 2
    captured = {}
    def capture(**kwargs):
        captured.update(kwargs)
        raise PrefixCaptured
    connector._finish_place_branch = capture
    try:
        prefix_result = connector._plan_branch(target=target, face="front", requested_virtual_contact=virtual,
            grasp_q=np.asarray(attempt["strict_grasp_candidates"][0]["q_rad"]),
            home_q=policy.layout_validation.initial_q, all_obstacles=scene.all_obstacles,
            receiver=scene.receiver, support_names=scene.support_graph.supported_by[target.name],
            suction=policy.data["suction"], seed=seed+50000)
        report["prefix_failed"] = prefix_result[1:]
        save("prefix_failed")
        return 3
    except PrefixCaptured:
        pass
    report["actual_prefix"] = {"contact_q_rad": captured["contact_q"].tolist(),
        "extraction_end_q_rad": captured["extraction"][-1].tolist(),
        "extraction_path": [q.tolist() for q in captured["extraction"]], "trace": captured["trace"]}
    save("actual_extraction_prefix_captured")
    if args.complete_cartesian:
        # Use the production finisher, real contact attachment, actual support
        # audit and unloaded withdrawal. Only its transit strategy is replaced
        # by the existing checked Cartesian continuation for this diagnosis.
        connector.budget = replace(connector.budget, cartesian_max_samples_per_stage=160)
        real_connect = connector._connect_pose
        def cartesian_connect(pose, seeds, start, obstacles, *, stage, connection_seed, **kwargs):
            if stage != "transit":
                return real_connect(pose, seeds, start, obstacles, stage=stage,
                                    connection_seed=connection_seed, **kwargs)
            path, failure, evidence = connector._cartesian(start, pose, obstacles,
                seed=connection_seed, attachment=kwargs.get("attachment"),
                support_names=kwargs.get("support_names", ()), stage=stage)
            return (None if failure is not None else path[-1]), path, failure, {
                "diagnostic_strategy": "EXISTING_CHECKED_CARTESIAN_CONTINUATION",
                "cartesian_max_samples_per_stage": 160, **evidence}
        connector._connect_pose = cartesian_connect
        variants = [("original_extraction", captured["extraction"], captured["released_tracker"])]
        report["complete_cartesian_attempts"] = []
        for variant_index in range(3):
            if variant_index == 1:
                destination = connector.robot.fk(captured["extraction"][-1]).copy()
                destination[:3,3] -= .03 * captured["physical_contact"][:3,2]
                extra, failure, evidence = connector._cartesian(captured["extraction"][-1], destination,
                    captured["payload_obstacles"], seed=seed+81000,
                    attachment=captured["attachment"], stage="transit")
                if failure is not None:
                    report["complete_cartesian_attempts"].append({"variant": "outward_30mm_buffer", "failure": failure})
                    continue
                variants.append(("outward_30mm_buffer", captured["extraction"] + extra[1:], captured["released_tracker"]))
            elif variant_index == 2:
                tracker, failure = connector._initial_proximity(target, captured["payload_obstacles"],
                    scene.support_graph.supported_by[target.name])
                destination = connector.robot.fk(captured["contact_q"]).copy()
                destination[:3,3] += [-.65,0,.10]
                extra, failure, evidence = connector._cartesian(captured["contact_q"], destination,
                    captured["payload_obstacles"], seed=seed+82000, attachment=captured["attachment"],
                    initial_proximity=tracker, support_names=scene.support_graph.supported_by[target.name], stage="extraction")
                if failure is not None or not tracker.fully_released:
                    report["complete_cartesian_attempts"].append({"variant": "outward_650mm_up100mm", "failure": failure,
                                                                "fully_released": bool(tracker.fully_released)})
                    continue
                variants.append(("outward_650mm_up100mm", extra, tracker))
            name, extraction, tracker = variants[-1]
            for index, placement in enumerate(placements[:args.max_placements]):
                arguments = {**captured, "extraction": extraction, "released_tracker": tracker,
                    "placement": placement, "selected_supports": [item for item in supports if item.name in placement.receiver_names],
                    "trace": {"diagnostic_extraction_variant": name, "stages": dict(captured["trace"]["stages"])},
                    "seed": seed+83000+variant_index*1000+index}
                segment, failure, trace = actual_finish_place(**arguments)
                report["complete_cartesian_attempts"].append({"variant": name,
                    "receiver_names": list(placement.receiver_names), "failure": failure, "trace": trace})
                print(json.dumps({"event": "full_cartesian_finisher", "variant": name,
                    "receiver_names": list(placement.receiver_names), "success": segment is not None,
                    "failure": failure}), flush=True)
                if segment is not None:
                    segment["validation"]["diagnostic_strategy"] = "EXISTING_CARTESIAN_TRANSIT_MAX_160_SAMPLES"
                    report["complete_segment"] = segment
                    report["complete_segment_fingerprint"] = canonical_digest(segment)
                    destination = args.output.with_name(args.output.stem + "_segment.json")
                    destination.write_text(json.dumps(segment, indent=2), encoding="utf-8")
                    report["complete_segment_file"] = str(destination)
                    save("complete_checked_cartesian_cycle")
                    return 0
                save("complete_cartesian_attempt_finished")
        save("complete_cartesian_attempts_exhausted")
        return 4
    if args.alternate_extraction:
        report["alternate_extraction_checks"] = []
        connector.budget = replace(connector.budget, cartesian_max_samples_per_stage=160)
        for vector in ([-.65,0,.05], [-.65,0,.10], [-.65,0,.15], [-.65,-.05,0], [-.65,.05,0]):
            tracker, initial_failure = connector._initial_proximity(target, captured["payload_obstacles"],
                scene.support_graph.supported_by[target.name])
            destination = connector.robot.fk(captured["contact_q"]).copy()
            destination[:3,3] += vector
            path, failure, evidence = connector._cartesian(captured["contact_q"], destination,
                captured["payload_obstacles"], seed=seed+73000,
                attachment=captured["attachment"], initial_proximity=tracker,
                support_names=scene.support_graph.supported_by[target.name], stage="extraction")
            check = {"translation_m": vector, "failure": failure, "samples": len(path),
                     "fully_released": bool(tracker.fully_released), "place_connections": []}
            if failure is None and tracker.fully_released:
                check["free_end_failure"] = connector._state_failure(path[-1], captured["payload_obstacles"],
                    attachment=captured["attachment"], stage="transit")
                for index, placement in enumerate(placements[:min(2,args.max_placements)]):
                    goal_physical = placement.payload.world_from_local @ np.linalg.inv(captured["rigid"].tcp_from_box)
                    goal_physical[2,3] += connector.budget.preplace_standoff_m
                    goal = connector.virtual_from_physical(goal_physical)
                    connection, connection_failure, _ = connector._cartesian(path[-1], goal,
                        captured["payload_obstacles"], seed=seed+74000+index,
                        attachment=captured["attachment"], stage="transit")
                    check["place_connections"].append({"index": index,
                        "receiver_names": list(placement.receiver_names), "failure": connection_failure,
                        "path_samples": len(connection)})
            report["alternate_extraction_checks"].append(check)
            save(f"alternate_extraction_{vector}_complete")
    if args.buffer_check:
        report["outward_buffer_checks"] = []
        connector.budget = replace(connector.budget, cartesian_max_samples_per_stage=160)
        outward = -captured["physical_contact"][:3, 2]
        extraction_q = captured["extraction"][-1]
        for buffer_m in (.03, .06, .10):
            buffer_pose = connector.robot.fk(extraction_q).copy()
            buffer_pose[:3, 3] += buffer_m * outward
            buffer_path, failure, evidence = connector._cartesian(extraction_q, buffer_pose,
                captured["payload_obstacles"], seed=seed+71000,
                attachment=captured["attachment"], stage="transit")
            check = {"buffer_m": buffer_m, "buffer_failure": failure, "buffer_samples": len(buffer_path),
                     "place_connections": []}
            if failure is None:
                for index, placement in enumerate(placements[:min(2,args.max_placements)]):
                    goal_physical = placement.payload.world_from_local @ np.linalg.inv(captured["rigid"].tcp_from_box)
                    goal_physical[2,3] += connector.budget.preplace_standoff_m
                    goal = connector.virtual_from_physical(goal_physical)
                    path, connection_failure, connection_evidence = connector._cartesian(buffer_path[-1], goal,
                        captured["payload_obstacles"], seed=seed+72000+index,
                        attachment=captured["attachment"], stage="transit")
                    check["place_connections"].append({"index": index, "receiver_names": list(placement.receiver_names),
                        "failure": connection_failure, "path_samples": len(path),
                        "last_box_center_m": captured["attachment"].box_at(path[-1]).center.tolist() if path else None})
            report["outward_buffer_checks"].append(check)
            save(f"outward_buffer_{buffer_m}_complete")
    real_state_failure = connector._state_failure
    report["endpoint_checks"] = []
    for index, placement in enumerate(placements[:args.max_placements]):
        physical = placement.payload.world_from_local @ np.linalg.inv(captured["rigid"].tcp_from_box)
        physical[2, 3] += connector.budget.preplace_standoff_m
        destination = connector.virtual_from_physical(physical)
        classified = []
        def classify(q, obstacles, **kwargs):
            failure = real_state_failure(q, obstacles, **kwargs)
            classified.append({"q_rad": np.asarray(q).tolist(), "failure": failure})
            return failure
        connector._state_failure = classify
        stream = connector._ik_stream(destination,
            [captured["extraction"][-1], captured["contact_q"], policy.layout_validation.initial_q],
            captured["payload_obstacles"], seed=seed+50060+index*1000,
            attachment=captured["attachment"], support_names=(), target_contact=None, stage="transit")
        solutions = list(stream)
        item = {"placement": placement.as_dict(), "preplace_virtual": destination.tolist(),
                "valid_solutions": [solution.q.tolist() for solution in solutions],
                "classification": classified, "ik_evidence": stream.evidence(),
                "failure_counts": dict(Counter("PASS" if value["failure"] is None else
                    value["failure"]["reason"] for value in classified))}
        report["endpoint_checks"].append(item)
        connector._state_failure = real_state_failure
        if args.cartesian:
            connector.budget = replace(connector.budget, cartesian_max_samples_per_stage=160)
            path, failure, evidence = connector._cartesian(captured["extraction"][-1], destination,
                captured["payload_obstacles"], seed=seed+61000+index,
                attachment=captured["attachment"], stage="transit")
            item["direct_cartesian"] = {"failure": failure, "evidence": evidence,
                                        "path": [q.tolist() for q in path]}
        save(f"endpoint_{index}_complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
