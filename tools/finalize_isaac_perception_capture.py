"""Convert saved Isaac captures to domain snapshots and feasibility handoffs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/unloading_contracts/src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_contracts import RobotStateRevision, dumps, to_wire  # noqa: E402
from unloading_perception.isaac_validation import (  # noqa: E402
    IsaacCaptureBinding,
    IsaacSceneManifest,
    build_feasibility_handoff,
    check_feasibility_handoff,
    ground_truth_observation,
    write_json,
)
from unloading_perception.scene import SnapshotAssembler, build_scene_update  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle-directory", required=True, type=Path)
    parser.add_argument("--capture-directory", required=True, type=Path)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    args = parser.parse_args()

    index = json.loads((args.bundle_directory / "index.json").read_text(encoding="utf-8"))
    contract = json.loads((args.project_root / "integration/isaac_scene_contract/m710id70_layout_v1/isaac_layout_contract.json").read_text(encoding="utf-8"))
    results = []
    # One assembler spans the ordered keyframes so a geometry change advances
    # SceneRevision instead of making every independently serialized snapshot
    # look like revision zero.
    assembler = SnapshotAssembler()
    for record in index["scenes"]:
        scene_name = record["scene"]
        scene_dir = args.capture_directory / scene_name
        manifest = IsaacSceneManifest.from_dict(json.loads((args.bundle_directory / record["path"]).read_text(encoding="utf-8")))
        binding = IsaacCaptureBinding.from_dict(json.loads((scene_dir / "capture_binding.json").read_text(encoding="utf-8")))
        if (binding.simulation_epoch, binding.frame_sequence) != (
            manifest.timing["simulation_epoch"], manifest.timing["simulation_frame"],
        ):
            raise ValueError(f"capture binding differs from manifest for {scene_name}")
        annotation_payload = json.loads((scene_dir / "gt_annotations.json").read_text(encoding="utf-8"))
        observation = ground_truth_observation(manifest, annotation_payload["objects"])
        assembler.update = build_scene_update(observation)
        assembler.robot_state = RobotStateRevision(
            int(manifest.timing["simulation_frame"]), tuple(manifest.robot["q_rad"]),
            {"joint_names": tuple(manifest.robot["joint_names"]), "robot_model_identity": manifest.robot["robot_model_identity"]},
            sample_time=float(manifest.timing["simulation_time"]), clock_domain="ros_sim_time", source="isaac_joint_state",
        )
        assembler.tool_attachment = manifest.mechanisms["tool_state"]
        assembler.payload_attachment = manifest.mechanisms["payload_state"]
        assembler.base_state = manifest.mechanisms["base_state"]
        assembler.conveyor_state = manifest.mechanisms["conveyor_state"]
        assembler.config_identity = {
            "identity": manifest.manifest_fingerprint,
            "robot_model_fingerprint": manifest.robot["robot_asset_hash"],
            "world_model_fingerprint": manifest.world_fingerprint,
            "layout_fingerprint": manifest.layout["layout_fingerprint"],
        }
        assembled = assembler.assemble()
        if assembled.snapshot is None:
            raise RuntimeError(f"Mode A snapshot incomplete for {scene_name}: {assembled.missing}")
        snapshot = assembled.snapshot
        handoff = build_feasibility_handoff(snapshot, manifest, evidence_mode="ISAAC_GT")
        compatibility = check_feasibility_handoff(handoff, manifest, contract)
        if compatibility["status"] != "PASS":
            raise RuntimeError(f"feasibility handoff compatibility failed for {scene_name}: {compatibility['differences']}")
        (scene_dir / "mode_a_perception_observation.json").write_text(dumps(observation), encoding="utf-8")
        write_json(scene_dir / "planning_world_snapshot.json", to_wire(snapshot))
        write_json(scene_dir / "feasibility_handoff.json", handoff)
        write_json(scene_dir / "feasibility_compatibility_report.json", compatibility)
        proposals = {
            "schema_version": "isaac_oracle_proposals_v1",
            "coordinate_space": "source_image",
            "source_size": [int(manifest.cameras[0]["resolution"][0]), int(manifest.cameras[0]["resolution"][1])],
            "source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
            "raw_image_automatic": False,
            "simulation_epoch": binding.simulation_epoch,
            "frame_sequence": binding.frame_sequence,
            "rgb_sha256": binding.rgb_sha256,
            "instances": [{
                "id": proposal_index,
                "instance_id": proposal_index,
                "simulation_object_id": item["simulation_object_id"],
                "oracle_proposal_source_id": item["simulation_object_id"],
                "label": "box",
                "bbox": item["bbox_xyxy"],
                "visible": item["visible"],
                "occluded": item["occluded"],
                "proposal_source": "ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL",
            } for proposal_index, item in enumerate(
                (item for item in annotation_payload["objects"] if item["visible"]), start=1
            )],
        }
        write_json(scene_dir / "oracle_proposals.json", proposals)
        results.append({
            "scene": scene_name,
            "mode_a_status": "PASS",
            "planning_world_snapshot": str((scene_dir / "planning_world_snapshot.json").resolve()),
            "world_fingerprint": snapshot.fingerprint,
            "scene_revision": snapshot.scene_revision.sequence,
            "candidate_count": len(assembled.snapshot.scene_snapshot["obstacles"]),
            "unknown_region_count": len(observation.unknown_regions),
            "feasibility_handoff_status": compatibility["status"],
        })
    summary = {
        "schema_version": "isaac_perception_domain_finalize_v1",
        "status": "PASS",
        "mode_a": "PASS",
        "mode_b1": "NOT_RUN",
        "raw_image_automatic": False,
        "feasibility_reference_commit": index["feasibility_reference_commit"],
        "layout_fingerprint": index["layout_fingerprint"],
        "scenes": results,
    }
    write_json(args.capture_directory / "domain_summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
