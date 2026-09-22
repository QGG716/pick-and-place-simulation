"""Three small frozen FANUC segments; no IK replacement or physical execution."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.layout_single_carton import (load_layout_motion_policy, build_verified_motion_input,
                                                _build_automatic_trajectory_connector)
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.planning_contract import FreeMotionRequest, PlanningBudget
from unloading_sim.tesseract_scene import export_scene
from unloading_sim.tesseract_ompl_backend import create_free_motion_backend


def fixture_context(state_path, segment_path):
    policy = load_layout_motion_policy(ROOT / "configs/validation/m710id70_proof_of_concept.yaml")
    scene = apply_actual_motion_state(build_verified_motion_input(policy), json.loads(Path(state_path).read_text()))
    segment = json.loads(Path(segment_path).read_text())
    built = _build_automatic_trajectory_connector(scene, scene.policy.layout_validation.layout.robot())
    if built.connector is None:
        raise RuntimeError(built.evidence)
    connector = built.connector
    connector.robot_state_validator.stack_carton_names = set(scene.remaining_stack_names)
    connector.stack_carton_names = set(scene.remaining_stack_names)
    connector.robot_state_validator.contact_target_name = segment["target"]
    target = next(b for b in scene.cartons if b.name == segment["target"])
    attachment = PhysicalContactAttachment(connector.robot,
        RigidAttachment(np.asarray(segment["contact"]["physical_contact_from_box"]), target.half_extents, target.name),
        connector.flange_from_virtual_task_tcp, connector.flange_from_physical_contact)
    return scene, segment, connector, attachment


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state", type=Path, required=True)
    ap.add_argument("--segment", type=Path, required=True)
    ap.add_argument("--worker", required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--case", choices=["empty_direct", "empty_detour", "loaded_free", "all"], default="all")
    ap.add_argument("--backend", choices=["legacy", "tesseract_ompl", "both"], default="both")
    ap.add_argument("--max-state-checks", type=int, default=100000)
    args = ap.parse_args()
    start = perf_counter()
    world, segment, connector, attachment = fixture_context(args.state, args.segment)
    setup = perf_counter()-start
    q = np.asarray(segment["path"])
    lo, hi = segment["stage_ranges"]["transit"]
    definitions = [("empty_direct", q[lo], q[lo+1], None),
                   ("empty_detour", q[lo], q[hi], None),
                   ("loaded_free", q[lo], q[lo+1], attachment)]
    output = dict(schema="tesseract_ompl_directed_comparison_v1", cases=[],
        setup_s=setup, hardware=dict(platform=platform.platform(), processor=platform.processor(),
                                    worker_threads=1, authority_threads=1),
        sources={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.state, args.segment]},
        scene_snapshot_fingerprint=world.snapshot["scene_fingerprint"],
        population=dict(active_cartons=len(world.cartons), original_cartons=40,
                        source="historical actual surviving world; no extra obstacle removal"),
        isaac_run=False, p50=None, p95=None)
    native = create_free_motion_backend("tesseract_ompl", executable=args.worker)
    try:
        for name, first, last, attached in definitions:
            if args.case not in {"all", name}:
                continue
            stage = "transit" if attached else "pregrasp"
            obstacles = [b for b in world.all_obstacles if attached is None or b.name != segment["target"]]
            began = perf_counter()
            scene = export_scene(connector, obstacles, stage=stage, attachment=attached)
            export_s = perf_counter()-began
            revision = world.snapshot["scene_fingerprint"]
            request = FreeMotionRequest(name, revision, scene["fingerprint"], scene["model_fingerprint"],
                scene["tool_fingerprint"], scene["policy_fingerprint"], stage, tuple(scene["joint_names"]),
                tuple(first), tuple(last), scene["constraints"], scene["frames"], scene["attachment"], 71070,
                budget=PlanningBudget(max_state_checks=args.max_state_checks, legacy_iterations=600))
            case = dict(name=name, stage=stage, q_start=first.tolist(), q_goal=last.tolist(),
                scene_fingerprint=scene["fingerprint"], model_fingerprint=scene["model_fingerprint"],
                tool_fingerprint=scene["tool_fingerprint"], policy_fingerprint=scene["policy_fingerprint"],
                source_path_indices=[lo, hi if name == "empty_detour" else lo+1],
                attachment=scene["attachment"], seed=request.seed, budgets=asdict(request.budget),
                export_s=export_s, results=[])
            output["cases"].append(case)
            authority = lambda path: connector._path_failure(path, obstacles, attachment=attached, stage=stage,
                                                             diagnostic_origin="backend_comparison")
            for backend in [native, create_free_motion_backend("legacy")]:
                if args.backend not in {"both", backend.name}:
                    continue
                # Keep the geometry loaded but clear authority state caches for
                # each solver's comparison; report geometry warm reuse explicitly.
                connector._state_cache.clear()
                validator = connector.robot_state_validator
                validator._static_cache.clear(); validator._geometry_cache.clear()
                before = perf_counter()
                print(json.dumps(dict(event="case_start", case=name, backend=backend.name)), flush=True)
                result = backend.plan(request, scene, authority)
                record = result.to_mapping()
                record["outer_wall_s"] = perf_counter()-before
                record["request_plus_scene_export_s"] = record["outer_wall_s"] + export_s
                if result.path:
                    points = np.asarray(result.path)
                    record["path_length_joint_l2_rad"] = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
                    record["waypoints"] = len(points)
                case["results"].append(record)
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(output, indent=2, allow_nan=False))
                print(json.dumps(dict(event="case_finish", case=name, backend=backend.name,
                                      status=result.status.value, timings=result.timings)), flush=True)
    finally:
        native.worker.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
