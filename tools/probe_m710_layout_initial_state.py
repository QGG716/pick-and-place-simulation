"""Diagnose fixed-layout home clearance without changing layout or policy."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.layout_single_carton import (  # noqa: E402
    _build_automatic_trajectory_connector,
    load_layout_motion_policy,
)
from unloading_sim.pinocchio_backend import _transform  # noqa: E402
from unloading_sim.workcell_layout import audit_initial_state  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/validation/m710id70_layout_v1_single_carton.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wrist-samples", type=int, default=25)
    parser.add_argument("--random-draws", type=int, default=0)
    parser.add_argument(
        "--full-tool-mesh",
        action="store_true",
        help="also measure official J5 against the hash-bound complete tool STL",
    )
    args = parser.parse_args()
    if args.wrist_samples < 2 or args.random_draws < 0:
        parser.error("wrist-samples must be >= 2 and random-draws must be >= 0")
    policy = load_layout_motion_policy(args.config)
    config = policy.layout_validation
    layout = config.layout
    lightweight = layout.robot()
    build = _build_automatic_trajectory_connector(policy, lightweight)
    result = {
        "schema": "m710id70_layout_initial_state_probe_v1",
        "layout_fingerprint": layout.layout_fingerprint,
        "scope": "read_only_diagnostic_no_state_or_layout_changes",
        "backend": build.evidence,
        "broadphase_initial": audit_initial_state(config),
    }
    if build.connector is not None:
        connector = build.connector
        mesh = connector.robot
        obstacles = [*layout.fixed_components(), *layout.cartons()]
        margin = connector.joint_margin_rad
        lower = mesh.joint_limits[:, 0] + margin
        upper = mesh.joint_limits[:, 1] - margin
        seed = int(config.data["initial_state"]["seed"])
        result.update(seed=seed, collision_margin_m=connector.collision_margin_m,
                      initial_failure=connector.validate_unloaded_state(config.initial_q, obstacles, stage="home_probe"))
        j5 = [
            (index, geometry) for index, geometry in enumerate(mesh.geometry_model.geometryObjects)
            if mesh.model.frames[int(geometry.parentFrame)].name == "J5_link"
        ]
        if not j5:
            raise ValueError("official J5 collision geometry is absent")
        wrist_rows = []
        for angle in np.linspace(max(lower[5], -np.pi), min(upper[5], np.pi), args.wrist_samples):
            q = config.initial_q.copy()
            q[5] = angle
            # Update the official geometry placements through its normal API.
            mesh.collision_result(q, [], check_self=False)
            plate = next(box for box in lightweight.tool_collision_obbs(q) if box.name == "tool_rigid_13")
            distances = []
            for index, geometry in j5:
                placement = mesh.base_transform @ mesh._matrix(mesh.geometry_data.oMg[index])
                distances.append(mesh._distance(
                    geometry.geometry,
                    _transform(mesh.coal, placement[:3, :3], placement[:3, 3]),
                    mesh.coal.Box(*(2.0 * plate.half_extents).tolist()),
                    _transform(mesh.coal, plate.rotation, plate.center),
                ))
            wrist_rows.append({"j6_rad": float(angle), "j5_plate_distance_m": min(distances)})
        result["wrist_sweep"] = wrist_rows
        if args.full_tool_mesh:
            analysis_record = layout.assets["tool_step_analysis"]
            mesh_record = layout.assets["tool_collision_mesh"]
            analysis_path = ROOT / analysis_record["repository_path"]
            mesh_path = ROOT / mesh_record["repository_path"]
            if hashlib.sha256(mesh_path.read_bytes()).hexdigest() != mesh_record["sha256"]:
                raise ValueError("tool collision mesh differs from the layout asset identity")
            analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
            flange_step_m = np.asarray(
                analysis["flange_origin_step_mm"], dtype=float
            ) * 0.001
            rotation_step_from_tool = np.asarray(
                analysis["rotation_step_from_tool"], dtype=float
            )
            tool_mesh = mesh.coal.MeshLoader().load(
                str(mesh_path), np.full(3, 0.001, dtype=float)
            )

            def full_mesh_clearance(configuration):
                configuration = np.asarray(configuration, dtype=float)
                mesh.collision_result(configuration, [], check_self=False)
                world_from_flange = lightweight.named_link_frames(configuration)[
                    "flange"
                ]
                world_from_step_rotation = (
                    world_from_flange[:3, :3] @ rotation_step_from_tool.T
                )
                world_from_step_translation = (
                    world_from_flange[:3, 3]
                    - world_from_step_rotation @ flange_step_m
                )
                tool_tf = _transform(
                    mesh.coal,
                    world_from_step_rotation,
                    world_from_step_translation,
                )
                best = None
                for index, geometry in j5:
                    placement = mesh.base_transform @ mesh._matrix(
                        mesh.geometry_data.oMg[index]
                    )
                    request = mesh.coal.DistanceRequest()
                    request.enable_signed_distance = True
                    distance_result = mesh.coal.DistanceResult()
                    value = mesh.coal.distance(
                        geometry.geometry,
                        _transform(
                            mesh.coal,
                            placement[:3, :3],
                            placement[:3, 3],
                        ),
                        tool_mesh,
                        tool_tf,
                        request,
                        distance_result,
                    )
                    distance = float(
                        getattr(distance_result, "min_distance", value)
                    )
                    row = {
                        "distance_m": distance,
                        "j5_nearest_point_world_m": np.asarray(
                            distance_result.getNearestPoint1(), dtype=float
                        ).tolist(),
                        "tool_nearest_point_world_m": np.asarray(
                            distance_result.getNearestPoint2(), dtype=float
                        ).tolist(),
                    }
                    if best is None or distance < best["distance_m"]:
                        best = row
                if best is None:
                    raise ValueError("official J5 collision geometry is absent")
                return best

            mesh_rows = []
            for angle in np.linspace(
                max(lower[5], -np.pi),
                min(upper[5], np.pi),
                args.wrist_samples,
            ):
                configuration = config.initial_q.copy()
                configuration[5] = angle
                mesh_rows.append(
                    {
                        "j6_rad": float(angle),
                        **full_mesh_clearance(configuration),
                    }
                )
            distances = [row["distance_m"] for row in mesh_rows]
            required = 2.0 * connector.collision_margin_m
            result["full_tool_mesh_clearance"] = {
                "geometry": "official_J5_mesh_vs_complete_hash_bound_tool_STL",
                "source": mesh_record,
                "complete_tool_stl_includes_compliant_cups": True,
                "step_to_flange_transform_source": analysis_record,
                "samples": mesh_rows,
                "sample_count": len(mesh_rows),
                "j6_sample_range_rad": [mesh_rows[0]["j6_rad"], mesh_rows[-1]["j6_rad"]],
                "minimum_distance_m": min(distances),
                "maximum_distance_m": max(distances),
                "required_pairwise_clearance_m": required,
                "maximum_clearance_shortfall_m": required - max(distances),
                "continuous_exhaustiveness": False,
                "changes_collision_policy": False,
            }
        failures = Counter()
        witness = None
        draws_used = 0
        rng = np.random.default_rng(seed)
        for index in range(args.random_draws):
            q = rng.uniform(lower, upper)
            failure = connector.validate_unloaded_state(q, obstacles, stage="home_probe")
            draws_used = index + 1
            if failure is None:
                # A witness is diagnostic only.  Preserve the current core
                # snapshot gate; do not quietly promote a new initial state.
                witness = {"draw_1_based": draws_used, "q_rad": q.tolist(),
                           "broadphase_audit": audit_initial_state(config, q)}
                break
            failures[json.dumps(failure, sort_keys=True)] += 1
        result.update(random_draws_requested=args.random_draws, random_draws_used=draws_used,
                      failure_counts=dict(failures), witness=witness)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"output": str(args.output), "backend_status": build.status,
                      "initial_failure": result.get("initial_failure"), "witness": result.get("witness")}))
    return 0 if build.connector is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
