"""Export one source STEP solid and diagnose its exact mesh/fixture clearance.

The export runs in an isolated OCP CPU environment; checking uses the existing
CPU planning environment.  Neither command changes runtime collision policy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def export_part(root, part_index, output):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.StlAPI import StlAPI_Writer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    source = root / "res/上海皖泰真空吸盘三分区.STEP"
    analysis_path = root / "assets/grippers/shanghai_wantai_three_zone/mass_properties.json"
    coverage_path = root / "assets/grippers/shanghai_wantai_three_zone/geometry_coverage.json"
    analysis = json.loads(analysis_path.read_text())
    coverage = json.loads(coverage_path.read_text())
    if digest(source) != coverage["source_step_sha256"] or digest(analysis_path) != coverage["analysis_sha256"]:
        raise ValueError("source STEP or per-solid analysis hash mismatch")
    old_removed = set(analysis["deformable_cup_solid_indices"])
    rigid_indices = [i for i in range(analysis["solid_occurrence_count"]) if i not in old_removed] + coverage["rigid_insert_solid_indices"]
    solid_index = rigid_indices[part_index]
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone or reader.TransferRoots() <= 0:
        raise RuntimeError("STEP transfer failed")
    explorer = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID)
    for _ in range(solid_index):
        explorer.Next()
    solid = explorer.Current()
    bounds = Bnd_Box()
    BRepBndLib.AddOptimal_s(solid, bounds, False, False)
    actual_bounds = list(bounds.Get())
    expected = np.asarray(analysis["solid_bounding_boxes_step_mm"][solid_index])
    if np.max(np.abs(np.asarray(actual_bounds) - expected)) > float(coverage["outward_pad_mm"]):
        raise ValueError("selected solid no longer matches source coverage bounds")
    mesher = BRepMesh_IncrementalMesh(solid, .05, False, .05, False)
    mesher.Perform()
    if not mesher.IsDone():
        raise RuntimeError("selected solid triangulation failed")
    output.mkdir(parents=True, exist_ok=True)
    mesh_path = output / f"tool_rigid_{part_index:03d}_source_solid_{solid_index:03d}.stl"
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(solid, str(mesh_path)):
        raise RuntimeError("selected solid STL export failed")
    record = {"schema": "wantai_single_part_mesh_probe_v1", "source_step_sha256": digest(source),
        "coverage_sha256": digest(coverage_path), "tool_part_index": part_index,
        "source_solid_index": solid_index, "source_bounds_step_mm": actual_bounds,
        "mesh_path": str(mesh_path), "mesh_sha256": digest(mesh_path), "mesh_units": "millimetre",
        "linear_deflection_mm": .05, "angular_deflection_rad": .05,
        "generator_sha256": digest(__file__), "runtime_collision_policy_changed": False}
    (output / "part_mesh.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


def raw_cases(value, path="root"):
    if isinstance(value, dict):
        failure = value.get("failure")
        if "q_rad" in value and isinstance(failure, dict):
            pair = failure.get("pair", [])
            if len(pair) == 2 and pair[0] == "tool_rigid_0":
                yield path, value["q_rad"], pair[1]
        for key, child in value.items():
            yield from raw_cases(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from raw_cases(child, f"{path}[{index}]")


def check_part(root, diagnostic, part_record, output):
    sys.path.insert(0, str(root / "src"))
    import coal
    from unloading_sim.layout_single_carton import load_layout_motion_policy
    from unloading_sim.pinocchio_backend import _transform
    from unloading_sim.tool_geometry import audit_tool_geometry
    metadata = json.loads(part_record.read_text())
    mesh_path = Path(metadata["mesh_path"])
    if digest(mesh_path) != metadata["mesh_sha256"]:
        raise ValueError("part mesh differs from exported source")
    policy = load_layout_motion_policy(root / "configs/validation/m710id70_layout_v1_single_carton.yaml")
    robot = policy.layout_validation.layout.robot()
    fixtures = {box.name: box for box in policy.layout_validation.layout.fixed_components()}
    geometry = audit_tool_geometry(root)
    step_rotation = np.asarray(geometry["rotation_step_from_tool"])
    flange_step_m = np.asarray(geometry["flange_origin_step_mm"]) * .001
    mesh = coal.MeshLoader().load(str(mesh_path), np.full(3, .001))
    margin = float(policy.data["state_validity"]["collision_margin_m"])
    rows = []
    for case_path, q, fixture_name in raw_cases(json.loads(diagnostic.read_text())):
        fixture = fixtures[fixture_name]
        flange = robot.named_link_frames(q)["flange"]
        world_rotation = flange[:3, :3] @ step_rotation.T
        world_translation = flange[:3, 3] - world_rotation @ flange_step_m
        mesh_tf = _transform(coal, world_rotation, world_translation)
        box = coal.Box(*(2 * fixture.half_extents))
        box_tf = _transform(coal, fixture.rotation, fixture.center)
        request, result = coal.DistanceRequest(), coal.DistanceResult()
        distance = float(coal.distance(mesh, mesh_tf, box, box_tf, request, result))
        distance = float(getattr(result, "min_distance", distance))
        collision_result = coal.CollisionResult()
        collision_count = int(coal.collide(mesh, mesh_tf, box, box_tf, coal.CollisionRequest(), collision_result))
        tool_obb = next(b for b in robot.tool_collision_obbs(q) if b.name == f"tool_rigid_{metadata['tool_part_index']}")
        rows.append({"case_path": case_path, "q_rad": q, "fixture": fixture_name,
            "obb_signed_sat_distance_m": tool_obb.signed_distance_obb(fixture),
            "obb_rejected_at_same_margin": tool_obb.intersects_obb(fixture, margin=margin),
            "mesh_distance_m": distance, "mesh_actual_collision": collision_count > 0,
            "mesh_contact_count": collision_count, "required_distance_m": 2 * margin,
            "classification": "REAL_PART_COLLISION" if collision_count else
                "REAL_PART_MARGIN_SHORTFALL" if distance <= 2 * margin else "OBB_FALSE_POSITIVE",
            "mesh_world_from_step": np.block([[world_rotation, world_translation.reshape(3, 1)], [np.array([[0., 0., 0., 1.]])]]).tolist()})
    result = {"schema": "wantai_single_part_clearance_probe_v1", "diagnostic_sha256": digest(diagnostic),
        "part": metadata, "robot_model": robot.name, "collision_margin_m": margin,
        "source_hashes": {str(root / "src/unloading_sim/workcell_layout.py"): digest(root / "src/unloading_sim/workcell_layout.py")},
        "scope": "diagnostic_only_no_geometry_or_policy_substitution", "cases": rows,
        "summary": {name: sum(r["classification"] == name for r in rows)
                    for name in ("REAL_PART_COLLISION", "REAL_PART_MARGIN_SHORTFALL", "OBB_FALSE_POSITIVE")}}
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["summary"], indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["export", "check"])
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--part-index", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--diagnostic", type=Path)
    parser.add_argument("--part-record", type=Path)
    args = parser.parse_args()
    if args.mode == "export":
        export_part(args.project_root.resolve(), args.part_index, args.output)
    else:
        check_part(args.project_root.resolve(), args.diagnostic, args.part_record, args.output)


if __name__ == "__main__":
    main()
