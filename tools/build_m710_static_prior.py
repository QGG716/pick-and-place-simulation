"""Build a bounded generic joint graph in the existing Linux planning worker.

Output is a JSON database file plus .build.json and .worker.log sidecars.
No historical trajectory, successful endpoint, or carton ID is an input.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.layout_single_carton import (load_layout_motion_policy,
    build_verified_motion_input, _build_automatic_trajectory_connector)
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.planning_only import PlanningOnlyConnector
from unloading_sim.static_prior import (MARKER, fingerprint, load_prior,
    make_database, nominal_front_attachment, request_context, validate_mode,
    placement_policy_fingerprint, graph_id, layout_portal_poses, portal_ik_seeds,
    diverse_portal_branches)
from unloading_sim.validation_physics import RigidAttachment


def build_request(connector, scene, *, mode, seed, node_limit, attempt_limit,
                  neighbors, placement_policy_sha256):
    if mode not in {"empty", "loaded"}:
        raise ValueError("STATIC_PRIOR_INVALID_MODE")
    q = np.asarray(scene.policy.layout_validation.initial_q, dtype=float)
    obstacles = tuple(b for b in scene.all_obstacles if b.category not in {"carton", "payload"})
    # Empty and loaded populations are independently checked by the worker.
    # The start state serves only to express a fixed flange attachment, not as
    # a solved query endpoint or a required roadmap vertex.
    attachment = None
    if mode == "loaded":
        layout = scene.policy.layout_validation.layout
        size = np.asarray(layout.data["carton_stack"]["carton_size_xyz_m"], dtype=float)
        rigid = RigidAttachment(np.asarray(nominal_front_attachment(size.tolist())),
            size/2., "static_prior_nominal_payload")
        attachment = PhysicalContactAttachment(connector.robot, rigid,
            connector.flange_from_virtual_task_tcp, connector.flange_from_physical_contact)
    connector.stack_carton_names = set()
    connector._native_contact_context = None
    connector.robot_state_validator.contact_target_name = None
    stage = "pregrasp" if mode == "empty" else "transit"
    request = connector._build_native_request(q, q, obstacles, seed=seed,
        attachment=attachment, stage=stage)
    # The offline graph has no stack, contact transition or task target.
    # Its full clearance checker still includes robot, complete tool, fixed
    # equipment, and the actual nominal attached body for the loaded mode.
    for key in ("process_policy", "task_id", "require_native_motion"):
        request.pop(key, None)
    if mode == "empty":
        request["clearance_policy"]["target_id"] = ""
    request["clearance_policy"]["schema"] = "m710_native_free_clearance_v1"
    request["clearance_policy"]["stack_carton_ids"] = []
    request.update(op="build_static_prior", prior_build=dict(
        node_limit=node_limit, attempt_limit=attempt_limit, neighbors=neighbors, seed=seed))
    request["prior_context"] = request_context(request,
        placement_policy_sha256=placement_policy_sha256,
        joint_limits_rad=connector.robot.joint_limits.tolist())
    return request




def generate_layout_portals(connector, scene, *, seed, ipc_timeout_s, build_budget_s,
                            placement_policy_sha256):
    """Offline IK only; every returned branch still needs native geometry screening."""
    from unloading_sim.ik import pose_error
    started = perf_counter()
    poses, sources = layout_portal_poses(scene)
    initial_q = np.asarray(scene.policy.layout_validation.initial_q, dtype=float).tolist()
    limits = connector.robot.joint_limits.tolist()
    template = build_request(connector, scene, mode="empty", seed=seed,
        node_limit=160, attempt_limit=1600, neighbors=10,
        placement_policy_sha256=placement_policy_sha256)
    for key in ("q_goal", "prior_build", "prior_context"):
        template.pop(key, None)
    candidates, records = [], []
    native_ik_calls = 0
    for pose_index, portal in enumerate(poses):
        seeds = portal_ik_seeds(limits, initial_q, seed=seed+pose_index*17)
        solutions, calls = [], []
        pose = np.asarray(portal["pose"], dtype=float)
        for offset in range(0, len(seeds), 8):
            submitted = deepcopy(template)
            batch = seeds[offset:offset+8]
            submitted.update(op="ik_batch", q_start=batch[0], goal_pose=portal["pose"],
                ik_seeds=batch, stop_on_first_success=False,
                seed=seed+pose_index*17+offset,
                candidate_id=portal["portal_id"], collision_check_in_native_ik=False,
                allowed_planning_time_s=min(float(build_budget_s), 30.),
                position_tolerance_m=float(connector.ik["position_tolerance_m"]),
                orientation_tolerance_rad=float(connector.ik["orientation_tolerance_rad"]),
                ik_timeout_s=float(connector.native_ik_seconds))
            began = perf_counter()
            raw = connector.native.request(submitted, timeout=ipc_timeout_s)
            elapsed = perf_counter()-began
            rows = raw.get("results", [])
            consumed = raw.get("consume_count")
            if (raw.get("status") not in {"SUCCESS", "NATIVE_IK_BATCH_TIMEOUT"}
                    or type(consumed) is not int or not 0 <= consumed <= len(batch)
                    or len(rows) != consumed or raw.get("native_ik_calls") != consumed
                    or (raw.get("status") == "SUCCESS" and consumed != len(batch))
                    or any(row.get("seed_index") != i or row.get("native_ik_calls") != 1
                           for i, row in enumerate(rows))):
                raise ValueError("STATIC_PRIOR_PORTAL_IK_EVIDENCE_MISSING")
            native_ik_calls += consumed
            calls.append(dict(seed_offset=offset, submitted=len(batch), consumed=consumed,
                status=raw["status"], wall_s=elapsed, native_ik_s=raw.get("ik_s"),
                native_ik_calls=consumed,
                results=[{k: row.get(k) for k in ("status", "error_code", "position_error_m",
                    "orientation_error_rad", "ik_s")} for row in rows]))
            for row in rows:
                if row.get("status") != "SUCCESS":
                    continue
                q = np.asarray(row.get("q"), dtype=float)
                if q.shape != (6,) or not np.isfinite(q).all() or not connector.robot.within_limits(q):
                    raise ValueError("STATIC_PRIOR_PORTAL_IK_INVALID_Q")
                _, position, angle = pose_error(connector.robot.fk(q), pose)
                if (position > connector.ik["position_tolerance_m"]
                        or angle > connector.ik["orientation_tolerance_rad"]):
                    raise ValueError("STATIC_PRIOR_PORTAL_IK_FK_RESIDUAL")
                solutions.append(q.tolist())
        branches = diverse_portal_branches(solutions, initial_q)
        for index, q in enumerate(branches):
            candidates.append(dict(portal_id=portal["portal_id"]+f"/branch-{index}",
                q=q, pose=portal["pose"]))
        records.append(dict(**portal, ik_calls=calls, converged=len(solutions),
            retained_branches=len(branches)))
    manifest = dict(schema="m710_layout_portal_generation_v1",
        coordinate_sources=sources, pose_count=len(poses), candidate_count=len(candidates),
        seeds_per_pose=17, native_batch_cap=8, branches_per_pose_cap=3,
        native_ik_calls=native_ik_calls, native_batch_calls=sum(len(r["ik_calls"]) for r in records),
        wall_s=perf_counter()-started, collision_screening="DEFERRED_TO_EACH_MODE_NATIVE_BUILD",
        no_historical_solutions=True, no_query_endpoints_used=True,
        poses=records, candidates=candidates)
    return candidates, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker-command", required=True)
    parser.add_argument("--worker-binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New database JSON file")
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--seed", type=int, default=71071)
    parser.add_argument("--node-limit", type=int, default=160)
    parser.add_argument("--attempt-limit", type=int, default=1600)
    parser.add_argument("--neighbors", type=int, default=10)
    parser.add_argument("--build-budget-s", type=float, default=300.)
    parser.add_argument("--ipc-timeout-s", type=float, default=360.)
    parser.add_argument("--layout-portals", action="store_true",
        help="Add 30 configuration-derived workspace poses with up to 3 native IK branches each")
    parser.add_argument("--mode", choices=("both", "empty", "loaded"), default="both")
    args = parser.parse_args()
    if (not 2 <= args.node_limit <= 512 or not args.node_limit <= args.attempt_limit <= 10000
            or not 1 <= args.neighbors <= 24):
        parser.error("bounded node/attempt/neighbor limits required")
    if (not np.isfinite(args.build_budget_s) or args.build_budget_s <= 0
            or not np.isfinite(args.ipc_timeout_s) or args.ipc_timeout_s <= args.build_budget_s):
        parser.error("positive finite build budget and IPC > build budget required")
    sidecar = args.output.with_suffix(".build.json")
    log_path = args.output.with_suffix(".worker.log")
    if any(path.exists() for path in (args.output, sidecar, log_path)):
        parser.error("output files already exist; preserve previous build evidence")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ["M710_MOVEIT_SEED"] = str(args.seed)
    os.environ["M710_MOVEIT_LOG"] = str(log_path)
    os.environ.pop("M710_MOVEIT_REQUEST_LOG", None)
    os.environ.pop("M710_MOVEIT_DIAGNOSTICS", None)
    initialized = perf_counter()
    policy_path = ROOT / "configs/validation/m710id70_proof_of_concept.yaml"
    policy = load_layout_motion_policy(policy_path)
    scene = build_verified_motion_input(policy, ROOT)
    built = _build_automatic_trajectory_connector(scene, policy.layout_validation.layout.robot())
    if built.connector is None:
        raise RuntimeError(built.failure_reason)
    connector = PlanningOnlyConnector.from_existing(built.connector, scene, args.worker_command,
        planning_mode="static_prior_fast", native_budget=dict(stage_wall_time_s=args.build_budget_s, ipc_timeout_s=args.ipc_timeout_s))
    report = dict(schema="m710_static_prior_build_report_v1", status=MARKER,
        qualification_status="NOT_EVALUATED", source_commit=args.source_commit,
        command=[sys.executable, *sys.argv], seed=args.seed,
        build_limits=dict(node_limit=args.node_limit, attempt_limit=args.attempt_limit,
            neighbors=args.neighbors, build_budget_s=args.build_budget_s, ipc_timeout_s=args.ipc_timeout_s),
        worker_sha256=hashlib.sha256(args.worker_binary.read_bytes()).hexdigest(),
        python=platform.python_version(), initialization_s=perf_counter()-initialized,
        no_historical_solutions=True, no_query_endpoints_used=True, modes={})
    modes = {}
    generation_started = perf_counter()
    try:
        placement_policy_sha256 = placement_policy_fingerprint(policy)
        portal_candidates = []
        if args.layout_portals:
            portal_candidates, portal_manifest = generate_layout_portals(connector, scene,
                seed=args.seed, ipc_timeout_s=args.ipc_timeout_s,
                build_budget_s=args.build_budget_s,
                placement_policy_sha256=placement_policy_sha256)
            report["layout_portals"] = portal_manifest
            args.output.with_suffix(".portals.json").write_text(
                json.dumps(portal_manifest, indent=2, allow_nan=False), encoding="utf-8")
        for mode in (("empty", "loaded") if args.mode == "both" else (args.mode,)):
            began = perf_counter()
            request = build_request(connector, scene, mode=mode, seed=args.seed,
                node_limit=args.node_limit, attempt_limit=args.attempt_limit,
                neighbors=args.neighbors, placement_policy_sha256=placement_policy_sha256)
            if args.layout_portals:
                request["prior_build"]["portal_candidates"] = deepcopy(portal_candidates)
            returned = connector.native.request(request, timeout=args.ipc_timeout_s)
            graph = deepcopy(returned)
            response_path = args.output.with_name(args.output.stem + "." + mode + ".response.json")
            context_path = args.output.with_name(args.output.stem + "." + mode + ".request-context.json")
            response_path.write_text(json.dumps(graph, indent=2, allow_nan=False), encoding="utf-8")
            context_path.write_text(json.dumps(request["prior_context"], indent=2, allow_nan=False), encoding="utf-8")
            def container_types(value, path=""):
                rows = {path: type(value).__name__} if isinstance(value, (dict, list, tuple)) else {}
                children = value.items() if isinstance(value, dict) else enumerate(value) if isinstance(value, (list, tuple)) else ()
                for key, item in children:
                    rows.update(container_types(item, path + "/" + str(key)))
                return rows
            expected_types = container_types(request["prior_context"])
            returned_types = container_types(graph.get("context"))
            report["modes"][mode] = dict(generation_s=perf_counter()-began,
                raw_response_file=response_path.name, request_context_file=context_path.name,
                context_equal_before_serialization=graph.get("context") == request["prior_context"],
                context_equal_after_serialization=fingerprint(graph.get("context")) == fingerprint(request["prior_context"]),
                context_container_type_differences=[dict(path=key, expected=value, returned=returned_types.get(key))
                    for key, value in expected_types.items() if value != returned_types.get(key)])
            # Context comes back from the worker; never silently replace a
            # missing/mismatched native context with the caller's expectation.
            validate_mode(graph, expected_context=request["prior_context"])
            graph["prior_id"] = graph_id(graph)
            modes[mode] = graph
            report["modes"][mode].update(generation_s=perf_counter()-began,
                nodes=len(graph["nodes"]), edges=len(graph["edges"]),
                context_sha256=fingerprint(graph["context"]),
                native={k: v for k, v in graph.items() if k not in {"nodes", "edges", "context"}})
            print(json.dumps(dict(mode=mode, **report["modes"][mode]), allow_nan=False), flush=True)
        report["generation_s"] = perf_counter()-generation_started
        data = make_database(modes, dict(source_commit=args.source_commit,
            worker_sha256=report["worker_sha256"], build_limits=report["build_limits"],
            seed=args.seed, generation_s=report["generation_s"],
            initialization_s=report["initialization_s"],
            placement_policy_sha256=placement_policy_sha256,
            node_source=("CONFIGURATION_WORKSPACE_PORTALS_THEN_BOUNDED_HALTON" if args.layout_portals else
                "BOUNDED_GENERIC_JOINT_LIMIT_SAMPLING_NO_REQUEST_ENDPOINTS"),
            layout_portals=report.get("layout_portals"),
            coverage="STATIC_LAYOUT_EMPTY_OR_NOMINAL_FRONT_CENTER_PAYLOAD",
            current_geometry_check_required=True, static_clearance_inherited=False,
            attachment_applicability=dict(translation_limit_m=.025, rotation_limit_rad=.05,
                meaning="CANDIDATE_SCOPE_ONLY_ACTUAL_GEOMETRY_RECHECK_REQUIRED"),
            collision_scope="DISCRETE_JOINT_EDGE_CHECK_NOT_CONTINUOUS_CERTIFICATION"))
        write_started = perf_counter()
        args.output.write_text(json.dumps(data, separators=(",", ":"), allow_nan=False), encoding="utf-8")
        report["database_write_s"] = perf_counter()-write_started
        _, report["load"] = load_prior(args.output)
        report["result"] = "BUILT_STATIC_CANDIDATE_GRAPH"
    except Exception as exc:
        report.update(result="BUILD_FAILED", generation_s=perf_counter()-generation_started,
            failure=dict(type=type(exc).__name__, reason=str(exc)))
        raise
    finally:
        connector.native.close()
        sidecar.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("result", "generation_s", "initialization_s", "load")}),
        flush=True)


if __name__ == "__main__":
    main()
