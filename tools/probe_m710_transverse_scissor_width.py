"""Offline paired J2 mesh diagnostics on saved failed goals; never plans."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.layout_single_carton import (
    load_layout_motion_policy, build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.moveit2_backend import box_message
from unloading_sim.pinocchio_backend import _transform
from unloading_sim.workcell_layout import load_workcell_layout


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("source", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    layouts = [load_workcell_layout(ROOT/"configs/workcells"/name) for name in (
        "m710id70_unloading_layout_v1_longitudinal_060.yaml",
        "m710id70_unloading_layout_v1_transverse_scissor_030.yaml")]
    policy = load_layout_motion_policy(ROOT/"configs/validation/m710id70_proof_of_concept.yaml")
    policy = replace(policy, layout_validation=replace(policy.layout_validation, layout=layouts[0]))
    scene = build_verified_motion_input(policy, ROOT)
    built = _build_automatic_trajectory_connector(scene, layouts[0].robot())
    if built.connector is None:
        raise ValueError(built.failure_reason)
    mesh = built.connector.robot
    j2 = [(i, g) for i, g in enumerate(mesh.geometry_model.geometryObjects)
          if mesh.model.frames[int(g.parentFrame)].name == "J2_link"]
    if not j2:
        raise ValueError("OFFICIAL_J2_MESH_MISSING")
    rows, inputs, cache = [], {}, {}

    def read(path):
        raw = path.read_bytes()
        inputs[str(path.relative_to(args.source))] = hashlib.sha256(raw).hexdigest()
        return json.loads(raw)

    protocol = read(args.source/"protocol.json")
    if protocol["active_layout_file"] != "configs/workcells/m710id70_unloading_layout_v1_longitudinal_060.yaml":
        raise ValueError("PAIRED_PROBE_REQUIRES_PREVIOUS_060_LAYOUT")
    result = read(args.source/"seed-71070/result.json")
    for attempt in result["attempts"]:
        if not attempt.get("folder"):
            continue
        folder = args.source/"seed-71070"/attempt["folder"]
        calls = {c["stage_id"]: c for c in read(folder/"requests.json") if c.get("stage_id")}
        for entry in read(folder/"search-trace.json")["attempts"]:
            req = entry["request"]
            if (req.get("op") != "plan" or req.get("stage") != "pregrasp" or
                    calls[req["stage_id"]]["status"] != "INVALID_GOAL_CLEARANCE"):
                continue
            world = {b["id"]: b for b in req["world"]}
            height = world["conveyor_transverse"]["pose"][2][3] + world["conveyor_transverse"]["size"][2]/2
            states = [coupled_conveyor_state(layout, height, ()) for layout in layouts]
            parts = [{b.name: b for b in state.boxes if b.name.startswith("lift_transverse_")}
                     for state in states]
            for name, box in parts[0].items():
                expected = box_message(box)
                if not (np.allclose(expected["pose"], world[name]["pose"], atol=1e-12, rtol=0) and
                        np.array_equal(expected["size"], world[name]["size"])):
                    raise ValueError("RECORDED_BASELINE_SCISSOR_GEOMETRY_MISMATCH")
            q = req["q_goal"]
            key = (height, *q)
            if key not in cache:
                mesh._update_geometry(np.asarray(q))
                compared = []
                for variant in parts:
                    pairs = []
                    for name, box in variant.items():
                        obstacle = mesh.coal.Box(*(2*box.half_extents).tolist())
                        transform = _transform(mesh.coal, box.rotation, box.center)
                        distances, contacts = [], []
                        for i, geometry in j2:
                            pose = mesh.base_transform @ mesh._matrix(mesh.geometry_data.oMg[i])
                            robot_tf = _transform(mesh.coal, pose[:3, :3], pose[:3, 3])
                            distances.append(mesh._distance(geometry.geometry, robot_tf, obstacle, transform))
                            contacts.append(int(mesh.coal.collide(geometry.geometry, robot_tf, obstacle, transform,
                                mesh.coal.CollisionRequest(), mesh.coal.CollisionResult())))
                        distance = float(min(distances))
                        if not np.isfinite(distance):
                            raise ValueError("PAIR_DISTANCE_UNKNOWN")
                        pairs.append(dict(pair=["J2_link", name], surface_distance_m=distance,
                                          collision=any(contacts), passed=not any(contacts) and distance+1e-9 >= .005))
                    compared.append(dict(passed=all(p["passed"] for p in pairs), pairs=pairs))
                cache[key] = compared
            rows.append(dict(attempt=attempt["folder"], stage_id=req["stage_id"], height_m=height, q_rad=q,
                             previous_native_failure=calls[req["stage_id"]]["response"].get("failure"),
                             baseline=cache[key][0], narrowed=cache[key][1]))
    if not rows:
        raise ValueError("NO_FAILED_NATIVE_GOALS")
    previous_witness = [r for r in rows if set(r["previous_native_failure"].get("pair", [])) ==
                        {"J2_link", "lift_transverse_left_arm_1_0"}]
    report = dict(schema="paired_failed_goal_j2_scissor_mesh_probe_v1",
        scope="OFFLINE_FAILED_GOALS_ONLY_NOT_PLANNER_INPUT_OR_PATH_ACCEPTANCE",
        source_commit=protocol["source_commit"], source_files_sha256=inputs,
        layouts=[dict(file=str(x.config_path.relative_to(ROOT)), fingerprint=x.layout_fingerprint) for x in layouts],
        mesh_geometry_source="COMPLETE_OFFICIAL_URDF_COLLISION_MESH_COAL",
        required_clearance_m=.005, rows=rows, request_count=len(rows), unique_q_height_count=len(cache),
        old_native_left_arm_witness_count=len(previous_witness),
        old_native_left_arm_witness_still_fails_scissor_count=sum(not r["narrowed"]["passed"] for r in previous_witness),
        baseline_j2_scissor_failed_count=sum(not r["baseline"]["passed"] for r in rows),
        narrowed_j2_scissor_failed_count=sum(not r["narrowed"]["passed"] for r in rows),
        physical_execution="NOT_RUN", execution_ready=False, continuous_swept_proof=False)
    args.output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k.endswith("count")}))


if __name__ == "__main__":
    main()
