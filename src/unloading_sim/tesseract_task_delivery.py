"""Thin single-candidate delivery adapter; never searches or starts Isaac.

The production bridge consumes a verified preflight, so the dependency order is
geometry -> bound preflight -> timing/export -> controller-reference acceptance
-> workspace/bundle readback. Readiness is published only after every gate.
"""
from copy import deepcopy
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from .isaac_bridge import build_fanuc_isaac_replay_bundle
from .layout_trajectory import validate_layout_trajectory_stage_contract
from .layout_single_carton import RESULT_SCHEMA, motion_implementation_identity
from .workcell_layout import canonical_digest
from .m710_execution import build_m710_execution_preflight, verify_m710_execution_preflight
from .m710_replay_contract import validate_trajectory_segment, verify_m710_replay_bundle
from .m710_replay_physics import replay_command_arrays, sample_joint_reference
from .motion_quality import collinear_indices
from .planning_profile import profile_evidence
from .collision_policy import SimulationCollisionPolicy


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def complete_geometry(result, scene, connector):
    segment = result.get("selected_trajectory_segment")
    validate_trajectory_segment(segment)
    validate_layout_trajectory_stage_contract(segment)
    target = next(box for box in scene.cartons if box.name == segment["target"])
    # Match the production finalizer's ordered obstacle identity exactly.
    obstacles = [box for box in scene.all_obstacles if box.name != target.name] + [target]
    if not connector._task_completed(segment, obstacles, target):
        raise ValueError("complete task proof is missing, stale or belongs to another request")
    return dict(status="PASS", completion=deepcopy(segment["validation"]["completion"]),
                scope="CURRENT_REQUEST_COMPLETE_GEOMETRY_AND_STAGE_EVENT_CONTRACT")


def motion_envelope(result, scene, project_root):
    """Convert an already completed production segment, without inventing assets/contact."""
    motion = dict(schema=RESULT_SCHEMA, simulation_profile=profile_evidence(scene.policy.data),
        collision_policy=SimulationCollisionPolicy.from_mapping(
            scene.policy.layout_validation.data.get("collision_policy")).to_mapping(),
        run_status="COMPLETED", layout_fingerprint=scene.snapshot["layout_fingerprint"],
        scene_fingerprint=scene.snapshot["scene_fingerprint"], policy_fingerprint=scene.policy.policy_fingerprint,
        implementation_identity=motion_implementation_identity(project_root),
        task_population=dict(carton_ids=list(scene.removable_cartons),
            selector="SupportRelationGraph.removable_cartons", searched_candidate_count=1,
            scope="LEGAL_POPULATION_NOT_NUMBER_OF_SEARCHED_OR_EXECUTED_TASKS"),
        statistics={**result["statistics"], "task_count": len(scene.removable_cartons),
                    "actual_connector_plan_calls": result["task_call_count"]},
        planning_performance=dict(planning_total_wall_seconds=result["elapsed_s"]),
        selected_trajectory_segment=deepcopy(result["selected_trajectory_segment"]),
        complete_trajectory_status="PASS", complete_trajectory_failure_reason=None,
        single_candidate_geometry_acceptance=deepcopy(result["geometry_acceptance"]))
    motion["evidence_fingerprint"] = canonical_digest(motion)
    return motion


def accept_controller_reference(bundle, segment):
    """Bind the actual quintic reference to the checked polyline and event states.

    No collision shortcut: acceptance requires the same ordered straight edges,
    with only the production collinear removal and stationary process holds.
    A changed curve is rejected, rather than borrowing the source path's proof.
    """
    data = bundle.to_dict() if hasattr(bundle, "to_dict") else bundle
    metadata = data["metadata"]
    timestamps, commands = replay_command_arrays(data, metadata["joint_names"])
    reference = metadata["joint_reference"]
    if reference["interpolation"] != "C2_piecewise_quintic_rest_to_rest":
        raise ValueError("unsupported controller interpolation")
    if metadata["timing_audit"]["within_limits"] is not True:
        raise ValueError("actual timing audit failed")
    path = np.asarray(segment["path"], dtype=float)
    release = segment["release_index"]
    protected = {segment["grasp_index"], release}
    for first, last in segment.get("stage_ranges", {}).values():
        protected.update((first, last))
    boundary = segment.get("approach", {}).get("free_connection_end_index")
    if boundary is not None:
        protected.add(boundary)
    indices = collinear_indices(path[:release+1], protected)
    if release < len(path)-1:
        indices += [release+i for i in collinear_indices(path[release:],
            [i-release for i in protected if i >= release])[1:]]
    if reference["source_retained_indices"] != indices:
        raise ValueError("controller skipped a protected source corner or event")
    q = np.asarray(reference["positions_rad"], float)
    times = np.asarray(reference["timestamps_seconds"], float)
    if q.ndim != 2 or q.shape[1] != 6 or times.shape != (len(q),) or len(q) < 2:
        raise ValueError("invalid controller reference shape")
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(times)) or times[0] != 0 or np.any(np.diff(times) <= 0):
        raise ValueError("invalid controller reference values")
    def moving_knots(values):
        return values[np.r_[True, np.any(np.diff(values, axis=0) != 0, axis=1)]]
    actual, expected = moving_knots(q), moving_knots(path[indices])
    if actual.shape != expected.shape or not np.allclose(actual, expected, atol=1e-12, rtol=0):
        raise ValueError("controller reference changed the checked joint polyline (including angle wrap)")
    # replay_command_arrays above is the production Isaac command acceptance gate.
    source_progress = np.r_[0., np.abs(np.diff(path, axis=0)).sum(axis=1).cumsum()]
    reference_progress = np.r_[0., np.abs(np.diff(q, axis=0)).sum(axis=1).cumsum()]
    for event, key in (("grasp", "grasp_index"), ("release", "release_index"),
                       ("release_retreat", "release_retreat_index")):
        event_time = metadata[f"{event}_time_seconds"]
        if event_time is None or not times[0] <= event_time <= times[-1]:
            raise ValueError(f"invalid {event} event time")
        state, _ = sample_joint_reference(timestamps, commands, event_time, reference=reference)
        if not np.allclose(state, path[segment[key]], atol=1e-10, rtol=0):
            raise ValueError(f"{event} is no longer at its checked joint state")
        edge = min(int(np.searchsorted(times, event_time, side="right"))-1, len(q)-2)
        progress = reference_progress[edge] + np.abs(state-q[edge]).sum()
        if not np.isclose(progress, source_progress[segment[key]], atol=1e-9, rtol=0):
            raise ValueError(f"{event} moved to another occurrence of its joint state")
    return dict(status="PASS", geometry="SAME_ORDERED_JOINT_POLYLINE_WITH_STATIONARY_HOLDS",
        angle_wrap=False, interpolation=reference["interpolation"],
        controller_command_count=len(commands), source_points=len(path),
        timing_audit=metadata["timing_audit"], duration_s=metadata["duration_seconds"])


def prepare_task_delivery(result, scene, connector, directory, *, execution_config=None, project_root):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    result.update(simulation_execution_ready=False, complete_geometry_status="NOT_RUN",
                  execution_preflight="NOT_RUN", time_parameterization_status="NOT_RUN",
                  execution_trajectory_status="NOT_RUN")
    timings = {}
    result["delivery_timings_s"] = timings
    gate = "complete_geometry_status"
    started = perf_counter()
    try:
        result["geometry_acceptance"] = complete_geometry(result, scene, connector)
        result[gate] = "PASS"
        timings["complete_geometry_gate"] = perf_counter()-started
        write(directory / "geometry.json", result["geometry_acceptance"])
        motion = motion_envelope(result, scene, project_root)
        write(directory / "motion.json", motion)
        gate, started = "execution_preflight", perf_counter()
        preflight = build_m710_execution_preflight(execution_config, motion_result=motion, motion_input=scene)
        write(directory / "preflight.json", preflight)
        verify_m710_execution_preflight(preflight)
        if preflight.get("simulation_execution_ready") is not True:
            raise ValueError(f"preflight blocked: {preflight.get('blockers')}")
        result[gate] = "PASS"
        timings["preflight"] = perf_counter()-started
        gate, started = "time_parameterization_status", perf_counter()
        adapter = preflight["replay_adapter_inputs"]
        plan = {**adapter["plan_common"], "segments": [adapter["trajectory_segment"]]}
        bundle = build_fanuc_isaac_replay_bundle(plan, adapter["configuration"], preflight=preflight)
        write(directory / "replay_bundle.json", bundle.to_dict())
        result[gate] = "PASS"
        timings["timing_and_export_inclusive"] = perf_counter()-started
        timings["time_parameterization_nested"] = bundle.metadata["time_parameterization_wall_seconds"]
        gate, started = "execution_trajectory_status", perf_counter()
        accepted = accept_controller_reference(bundle, adapter["trajectory_segment"])
        write(directory / "controller-reference-acceptance.json", accepted)
        result[gate] = "PASS"
        timings["controller_reference_acceptance"] = perf_counter()-started
        gate, started = "execution_preflight", perf_counter()
        verified = verify_m710_replay_bundle(json.loads((directory / "replay_bundle.json").read_text()),
                                             project_root=project_root)
        write(directory / "bundle-readback.json", verified)
        timings["workspace_and_bundle_readback"] = perf_counter()-started
        result["simulation_execution_ready"] = True
        result["execution_bundle"] = str(directory / "replay_bundle.json")
        result["path_metrics"]["execution_duration_s"] = accepted["duration_s"]
    except Exception as exc:
        result[gate] = "FAIL"
        timings[f"{gate}_failed_inclusive"] = perf_counter()-started
        result["delivery_failure"] = dict(gate=gate, exception_type=type(exc).__name__, reason=str(exc))
    finally:
        write(directory / "delivery.json", result)
    return result
