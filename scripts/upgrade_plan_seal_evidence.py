"""Add deterministic per-cup geometric seal evidence to a legacy unload plan."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from unloading_sim.demo import build_robot
from unloading_sim.grasp import SuctionGraspCandidate, suction_cup_seal_indices
from unloading_sim.scene import load_scene_config


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    source_plan = json.loads(args.plan.read_text(encoding="utf-8"))
    upgraded = copy.deepcopy(source_plan)
    scene, cfg = load_scene_config(args.config)
    planning = cfg["planning"]
    layout = planning["suction_cup_layout"]
    rows = int(layout["rows"])
    columns = int(layout["columns"])
    pitch = np.asarray(layout["pitch_m"], dtype=float)
    radius = float(layout["cup_radius_m"])
    zone_count = int(layout["zone_count"])
    minimum = int(layout["minimum_sealed_cups"])
    edge_margin = float(planning.get("suction_edge_margin_m", 0.0))
    if rows * columns != int(cfg["simulation_validation"]["vacuum_cup_count"]):
        raise ValueError("planner and validation cup counts differ")
    width_offsets = (np.arange(rows) - 0.5 * (rows - 1)) * pitch[0]
    length_offsets = (np.arange(columns) - 0.5 * (columns - 1)) * pitch[1]
    cup_centers = np.asarray(
        [(width, length) for length in length_offsets for width in width_offsets], dtype=float
    )
    columns_per_zone = columns // zone_count

    evidence: list[dict[str, object]] = []
    for index, segment in enumerate(upgraded["segments"]):
        target = str(segment["target"])
        carton = scene.carton(target)
        grasp_index = int(segment["grasp_index"])
        path = np.asarray(segment["path"], dtype=float)
        base_path = np.asarray(segment["base_path"], dtype=float)
        if path.ndim != 2 or not 0 <= grasp_index < len(path):
            raise ValueError(f"segment {index} has an invalid grasp index")
        if base_path.shape != (len(path), 3):
            raise ValueError(f"segment {index} has an invalid base path")
        segment_cfg = copy.deepcopy(cfg)
        segment_cfg["robot"]["base_position"] = base_path[grasp_index].tolist()
        robot = build_robot(segment_cfg)
        grasp_pose = robot.fk(path[grasp_index])

        contact = np.asarray(segment["contact_point"], dtype=float)
        contact_local = carton.to_local(contact)
        normalized = np.abs(contact_local) / np.maximum(carton.half_extents, 1e-12)
        face_axis = int(np.argmax(normalized))
        sign = 1.0 if contact_local[face_axis] >= 0.0 else -1.0
        outward_normal = carton.rotation[:, face_axis] * sign
        candidate = SuctionGraspCandidate(
            carton_name=target,
            contact_point=contact,
            outward_normal=outward_normal,
            face_mode=str(segment["face_mode"]),
            pregrasp_pose=grasp_pose.copy(),
            grasp_pose=grasp_pose,
            score=0.0,
        )
        sealed = suction_cup_seal_indices(
            candidate,
            carton,
            cup_centers,
            radius,
            edge_margin_m=edge_margin,
        )
        zone_counts = [0] * zone_count
        for cup_index in sealed:
            column = cup_index // rows
            zone_counts[min(column // columns_per_zone, zone_count - 1)] += 1
        if not sealed:
            raise ValueError(f"segment {index} {target} has no geometrically sealed cups")
        segment["sealed_cup_indices"] = list(sealed)
        segment["sealed_cups_per_zone"] = zone_counts
        segment["sealed_cup_evidence"] = {
            "kind": "deterministic_geometric_reconstruction",
            "source_fields": ["target", "contact_point", "path[grasp_index]", "base_path[grasp_index]"],
            "cup_count": len(sealed),
            "meets_current_planner_minimum": len(sealed) >= minimum,
            "fk_contact_offset_m": float(np.linalg.norm(grasp_pose[:3, 3] - contact)),
            "tool_normal_alignment": abs(float(grasp_pose[:3, 2] @ outward_normal)),
        }
        evidence.append(
            {
                "segment_index": index,
                "target": target,
                "sealed_cup_count": len(sealed),
                "sealed_cups_per_zone": zone_counts,
                "meets_current_planner_minimum": len(sealed) >= minimum,
            }
        )

    upgraded["seal_evidence_upgrade"] = {
        "source_plan_path": str(args.plan),
        "source_plan_sha256": _sha256(args.plan),
        "config_path": str(args.config),
        "config_sha256": _sha256(args.config),
        "method": "robot_fk_at_grasp_plus_complete_circular_lip_inside_target_face",
        "measurement_status": "geometric_digital_twin_evidence_not_vacuum_sensor_measurement",
        "cup_layout": {
            "rows": rows,
            "columns": columns,
            "pitch_m": pitch.tolist(),
            "cup_radius_m": radius,
            "edge_margin_m": edge_margin,
            "minimum_sealed_cups": minimum,
        },
        "segments": evidence,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(upgraded, ensure_ascii=False, indent=2), encoding="utf-8")
    counts = [int(item["sealed_cup_count"]) for item in evidence]
    print(
        json.dumps(
            {
                "output": str(args.output),
                "segments": len(evidence),
                "minimum_sealed_cups": min(counts),
                "maximum_sealed_cups": max(counts),
                "output_sha256": _sha256(args.output),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
