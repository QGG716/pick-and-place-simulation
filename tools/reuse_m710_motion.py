"""Recheck a historical path against current exact geometry, then bind new evidence."""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.layout_single_carton import (load_layout_motion_policy, build_verified_motion_input,
    _build_automatic_trajectory_connector, motion_implementation_identity)
from unloading_sim.layout_trajectory import PhysicalContactAttachment, validate_layout_trajectory_stage_contract
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState
from unloading_sim.workcell_layout import canonical_digest, sha256_file
from unloading_sim.m710_execution import build_m710_execution_preflight


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--actual-state", type=Path)
    parser.add_argument("--config", default="configs/validation/m710id70_layout_v1_single_carton.yaml")
    parser.add_argument("--execution-config", default="configs/simulation/m710id70_first_row_recording_v1.yaml")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    old = json.loads(args.motion.read_text())
    check = copy.deepcopy(old)
    if check.pop("evidence_fingerprint") != canonical_digest(check):
        raise ValueError("historical motion fingerprint mismatch")
    policy = load_layout_motion_policy(args.config)
    scene = build_verified_motion_input(policy)
    if args.actual_state:
        rows = RowUnloadingState()
        rows.rank(scene.cartons, support_graph=scene.support_graph)
        scene = apply_actual_motion_state(scene, json.loads(args.actual_state.read_text()), row_state=rows)
        policy = scene.policy
    evidence = {"source_motion_sha256": sha256_file(args.motion), "source_evidence_fingerprint": old["evidence_fingerprint"],
                "source_policy_fingerprint": old["policy_fingerprint"], "status": "RECHECKING"}
    try:
        if old["layout_fingerprint"] != scene.snapshot["layout_fingerprint"]:
            raise ValueError("cached path uses a different fixed layout")
        segment = copy.deepcopy(old["selected_trajectory_segment"])
        if segment["target"] not in scene.removable_cartons:
            raise ValueError("cached target is not in current highest removable row")
        target = next(box for box in scene.cartons if box.name == segment["target"])
        old_target = next(task for task in old["tasks"] if task["task_id"] == target.name)
        if not np.allclose(old_target["target_pose_world"], target.world_from_local, atol=1e-9, rtol=0):
            raise ValueError("cached target pose changed; actual-state replanning required")
        build = _build_automatic_trajectory_connector(scene, policy.layout_validation.layout.robot())
        connector = build.connector
        if connector is None:
            raise ValueError(str(build.evidence))
        path = np.asarray(segment["path"])
        q = np.asarray(scene.snapshot["robot"]["q_rad"])
        if np.max(np.abs(q - path[0])) > 0.001:
            raise ValueError("cached start differs from current measured joints")
        path[0] = q
        connector.start_planning_request()
        connector.stack_carton_names = set(scene.remaining_stack_names or [box.name for box in scene.cartons])
        grasp_q = path[segment["grasp_index"]]
        selection = connector._contact_selection(grasp_q, target, segment["face"], policy.data["suction"])
        if selection["commanded_active_mask"] != segment["contact"]["cup_selection"]["commanded_active_mask"]:
            raise ValueError("cached cup selection no longer seals the selected target")
        rigid = RigidAttachment.capture(connector.physical_from_virtual(connector.robot.fk(grasp_q)), target)
        attachment = PhysicalContactAttachment(connector.robot, rigid, connector.flange_from_virtual_task_tcp,
                                               connector.flange_from_physical_contact)
        all_obstacles = scene.all_obstacles
        payload_obstacles = [box for box in all_obstacles if box.name != target.name]
        stack_supports = tuple(scene.support_graph.supported_by.get(target.name, ()))
        tracker, failure = connector._initial_proximity(target, payload_obstacles, stack_supports)
        if failure:
            raise ValueError(str(failure))
        checked = []
        placed = attachment.box_at(path[segment["release_index"]])
        for stage, (first, last) in segment["stage_ranges"].items():
            points = path[first:last + 1]
            if stage in {"home", "pregrasp", "contact"}:
                failure = connector._path_failure(points, all_obstacles, stage=stage,
                    target_contact=target if stage == "contact" else None)
            elif stage in {"support-release", "extraction"}:
                failure = connector._path_failure(points, payload_obstacles, stage=stage,
                    attachment=attachment, initial_proximity=tracker, support_names=stack_supports)
            elif stage in {"transit", "place"}:
                failure = connector._path_failure(points, payload_obstacles, stage=stage,
                    attachment=attachment, support_names=segment["place"]["support_names"] if stage == "place" else ())
            elif stage == "withdrawal":
                failure = connector._path_failure(points, [*payload_obstacles, placed], stage=stage,
                                                  target_contact=placed)
            else:
                raise ValueError(f"unsupported cached stage {stage}")
            checked.append({"stage": stage, "failure": failure})
            if failure:
                raise ValueError(str(checked[-1]))
        # The historical ballistic landing and full departure sweep were
        # checked with the identical target and path. New runtime still gates
        # takeover on real contact; no exemption is applied at RELEASE.
        segment["path"] = path.tolist()
        segment["validation"]["validator_identity"] = connector.validator_identity
        segment["validation"]["current_full_tool_recheck"] = checked
        validate_layout_trajectory_stage_contract(segment)
        result = copy.deepcopy(old)
        result.update(policy_fingerprint=policy.policy_fingerprint,
            scene_fingerprint=scene.snapshot["scene_fingerprint"], implementation_identity=motion_implementation_identity(ROOT),
            selected_trajectory_segment=segment, snapshot_verification=scene.snapshot_verification,
            trajectory_backend=dict(build.evidence))
        result["task_population"]["carton_ids"] = list(scene.removable_cartons)
        result["tasks"] = [task for task in result["tasks"] if task["task_id"] in scene.removable_cartons]
        result["statistics"]["task_count"] = len(scene.removable_cartons)
        result["planning_performance"] = {"planning_total_wall_seconds": time.monotonic() - start,
            "historical_search_seconds": old["planning_performance"]["planning_total_wall_seconds"],
            "search_repeated": False, "current_path_recheck": checked}
        evidence.update(status="CURRENT_PATH_RECHECK_PASS", stages=checked)
        result["historical_path_reuse"] = evidence
        result.pop("evidence_fingerprint", None)
        result["evidence_fingerprint"] = canonical_digest(result)
        (args.output / "motion.json").write_text(json.dumps(result, indent=2))
        preflight = build_m710_execution_preflight(args.execution_config, motion_result=result, motion_input=scene)
        (args.output / "preflight.json").write_text(json.dumps(preflight, indent=2))
        (args.output / "actual_scene_snapshot.json").write_text(json.dumps(scene.snapshot, indent=2))
        if not preflight["simulation_execution_ready"]:
            raise ValueError(str(preflight["blockers"]))
    except (ValueError, KeyError) as exc:
        evidence.update(status="REUSE_REJECTED", reason=str(exc))
    evidence["wall_seconds"] = time.monotonic() - start
    (args.output / "reuse_evidence.json").write_text(json.dumps(evidence, indent=2))
    print(json.dumps(evidence), flush=True)
    return 0 if evidence["status"] == "CURRENT_PATH_RECHECK_PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
