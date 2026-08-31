"""Certified trajectory-library lookup for bounded-latency online planning.

Cold IK/RRT planning is deliberately excluded from this module.  A slow,
offline compiler validates paths and binds each segment to the scene state in
which it is valid.  Online lookup then performs only deterministic state,
support-order, joint-continuity, and integrity checks.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from typing import Any, Sequence

import numpy as np

from .benchmark import BenchmarkRecorder
from .online_unload import remove_carton
from .scene import TrailerScene, load_scene_config
from .support import SupportRelationGraph
from .timing import motion_limits_from_config, time_parameterize_joint_path
from .trajectory_cache import validate_cached_plan


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def _digest(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def scene_snapshot(scene: TrailerScene) -> list[dict[str, Any]]:
    return [
        {
            "name": carton.name,
            "center_m": carton.center.tolist(),
            "half_extents_m": carton.half_extents.tolist(),
            "rotation": carton.rotation.tolist(),
        }
        for carton in sorted(scene.cartons, key=lambda item: item.name)
    ]


def _rotation_error_rad(a: np.ndarray, b: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(a.T @ b) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(cosine))


def snapshot_matches(
    actual: list[dict[str, Any]],
    expected: list[dict[str, Any]],
    *,
    translation_tolerance_m: float,
    size_tolerance_m: float,
    rotation_tolerance_rad: float,
) -> tuple[bool, str]:
    actual_by_name = {item["name"]: item for item in actual}
    expected_by_name = {item["name"]: item for item in expected}
    if set(actual_by_name) != set(expected_by_name):
        return False, "carton set differs from certified scene"
    for name in sorted(expected_by_name):
        observed = actual_by_name[name]
        nominal = expected_by_name[name]
        center_error = float(
            np.linalg.norm(np.asarray(observed["center_m"]) - np.asarray(nominal["center_m"]))
        )
        if center_error > translation_tolerance_m:
            return False, f"{name} translation error {center_error:.6f} m exceeds certificate"
        size_error = float(
            np.max(np.abs(np.asarray(observed["half_extents_m"]) - np.asarray(nominal["half_extents_m"])))
        )
        if size_error > 0.5 * size_tolerance_m:
            return False, f"{name} size error {2.0 * size_error:.6f} m exceeds certificate"
        rotation_error = _rotation_error_rad(
            np.asarray(nominal["rotation"], dtype=float), np.asarray(observed["rotation"], dtype=float)
        )
        if rotation_error > rotation_tolerance_rad:
            return False, f"{name} rotation error {rotation_error:.6f} rad exceeds certificate"
    return True, "scene is inside certified envelope"


def compile_runtime_library(
    manifest: dict[str, Any],
    *,
    joint_start_tolerance_rad: float = 0.01,
) -> dict[str, Any]:
    """Slow offline compilation; every segment must pass collision validation."""
    if joint_start_tolerance_rad <= 0.0:
        raise ValueError("joint_start_tolerance_rad must be positive")
    validation = validate_cached_plan(manifest)
    segments = manifest.get("segments", [])
    if len(validation) != len(segments) or not segments or not all(item["valid"] for item in validation):
        raise ValueError("source plan is not fully collision-valid and cannot be certified")
    scene, cfg = load_scene_config(manifest["config"])
    limits = motion_limits_from_config(cfg, len(manifest["robot"]["home_joints"]))
    execution = cfg.get("execution", {})
    fixed_process_seconds = sum(
        float(execution.get(key, 0.0))
        for key in (
            "perception_update_seconds",
            "vacuum_establish_seconds",
            "release_seconds",
        )
    )
    if not np.isfinite(fixed_process_seconds) or fixed_process_seconds < 0.0:
        raise ValueError("execution process delays must be finite and non-negative")
    entries = []
    for segment, validation_result in zip(segments, validation):
        path = segment.get("path", [])
        if not path:
            raise ValueError("runtime library segment has an empty path")
        timed = time_parameterize_joint_path(np.asarray(path, dtype=float), limits)
        certified_segment = copy.deepcopy(segment)
        certified_segment["time_from_start_seconds"] = timed.time_from_start.tolist()
        certified_segment["timing"] = {
            "motion_seconds": float(timed.duration_seconds),
            "fixed_process_seconds": fixed_process_seconds,
            "estimated_cycle_seconds": float(timed.duration_seconds + fixed_process_seconds),
            "limits_source": limits.source,
        }
        entry = {
            "pick_index": int(segment["pick_index"]),
            "target": str(segment["target"]),
            "scene": scene_snapshot(scene),
            "start_joints_rad": path[0],
            "end_joints_rad": path[-1],
            "amr_dock_position": segment["amr_dock_position"],
            "segment": certified_segment,
            "path_sha256": _digest(path),
            "offline_validation_seconds": float(validation_result["validation_seconds"]),
        }
        entries.append(entry)
        remove_carton(scene, entry["target"])
    source_payload = {key: manifest.get(key) for key in ("config", "robot", "segments")}
    return {
        "format": "fanuc_certified_runtime_library_v1",
        "robot_model": manifest.get("robot", {}).get("model"),
        "config": manifest["config"],
        "source_plan_sha256": _digest(source_payload),
        "certificate": {
            # Non-zero perception envelopes require separate robust-clearance
            # certification plus visual-servo endpoint correction.  Until that
            # exists, exact nominal geometry is the only safe fast path.
            "translation_tolerance_m": 0.0,
            "size_tolerance_m": 0.0,
            "rotation_tolerance_rad": 0.0,
            "joint_start_tolerance_rad": float(joint_start_tolerance_rad),
            "collision_validation": "continuous_edges_robot_and_carried_carton",
        },
        "entries": entries,
    }


@dataclass(frozen=True)
class RealtimePlanResult:
    success: bool
    path: list[np.ndarray]
    target: str | None
    source: str
    latency_seconds: float
    message: str
    segment: dict[str, Any] | None = None


class CertifiedRuntimePlanner:
    def __init__(self, library: dict[str, Any], *, deadline_seconds: float = 0.05) -> None:
        if library.get("format") != "fanuc_certified_runtime_library_v1":
            raise ValueError("unsupported or uncertified runtime library")
        if library.get("robot_model") != "fanuc_m20id35":
            raise ValueError("runtime planner currently supports FANUC M-20iD/35 only")
        if not np.isfinite(deadline_seconds) or deadline_seconds <= 0.0:
            raise ValueError("deadline_seconds must be finite and positive")
        self.library = library
        self.deadline_seconds = float(deadline_seconds)
        self._prepared_entries: list[tuple[dict[str, Any], list[np.ndarray]]] = []
        for entry in library.get("entries", []):
            if _digest(entry["segment"]["path"]) != entry["path_sha256"]:
                raise RuntimeError("certified trajectory integrity check failed")
            path = [np.asarray(q, dtype=float) for q in entry["segment"]["path"]]
            self._prepared_entries.append((entry, path))

    def plan_next(
        self,
        scene: TrailerScene,
        current_q: np.ndarray,
        current_dock: np.ndarray | None = None,
    ) -> RealtimePlanResult:
        started = perf_counter()

        def deadline_result() -> RealtimePlanResult:
            elapsed = perf_counter() - started
            return RealtimePlanResult(
                False,
                [],
                None,
                "deadline",
                elapsed,
                "certified lookup exceeded its hard deadline",
            )

        current_q = np.asarray(current_q, dtype=float)
        certificate = self.library["certificate"]
        candidates = set(SupportRelationGraph.build(scene.cartons).removable_cartons())
        actual_snapshot = scene_snapshot(scene)
        if perf_counter() - started > self.deadline_seconds:
            return deadline_result()
        rejection = "no certified entry matches the current scene"
        for entry, prepared_path in self._prepared_entries:
            if perf_counter() - started > self.deadline_seconds:
                return deadline_result()
            if entry["target"] not in candidates:
                continue
            matches, reason = snapshot_matches(
                actual_snapshot,
                entry["scene"],
                translation_tolerance_m=float(certificate["translation_tolerance_m"]),
                size_tolerance_m=float(certificate["size_tolerance_m"]),
                rotation_tolerance_rad=float(certificate["rotation_tolerance_rad"]),
            )
            if not matches:
                rejection = reason
                continue
            start_q = np.asarray(entry["start_joints_rad"], dtype=float)
            if current_q.shape != start_q.shape:
                rejection = "current joint vector has the wrong dimension"
                continue
            start_error = float(np.max(np.abs(current_q - start_q)))
            if start_error > float(certificate["joint_start_tolerance_rad"]):
                rejection = f"joint start error {start_error:.6f} rad exceeds certificate"
                continue
            if current_dock is not None:
                dock_error = float(
                    np.max(
                        np.abs(
                            np.asarray(current_dock, dtype=float)
                            - np.asarray(entry["amr_dock_position"], dtype=float)
                        )
                    )
                )
                if dock_error > 1e-6:
                    rejection = f"AMR dock error {dock_error:.6f} m exceeds certificate"
                    continue
            segment = entry["segment"]
            latency = perf_counter() - started
            if latency > self.deadline_seconds:
                return deadline_result()
            return RealtimePlanResult(
                True,
                prepared_path,
                entry["target"],
                "certified_library",
                latency,
                "certified trajectory selected",
                segment,
            )
        return RealtimePlanResult(False, [], None, "cache_miss", perf_counter() - started, rejection)


def benchmark_runtime_library(
    library: dict[str, Any], *, repetitions: int = 100
) -> dict[str, Any]:
    if repetitions <= 0:
        raise ValueError("benchmark repetitions must be positive")
    _, cfg = load_scene_config(library["config"])
    planner = CertifiedRuntimePlanner(library)
    configured_dock = cfg.get("planning", {}).get("initial_dock_position")
    latencies = []
    reference_targets = None
    recorder = BenchmarkRecorder(execution_source="certified_simulation_command_schedule")
    for repetition in range(repetitions):
        scene, _ = load_scene_config(library["config"])
        current_q = np.asarray(cfg["robot"]["home_joints"], dtype=float)
        current_dock = (
            None if configured_dock is None else np.asarray(configured_dock, dtype=float)
        )
        selected = []
        while scene.cartons:
            result = planner.plan_next(scene, current_q, current_dock)
            latencies.append(result.latency_seconds)
            recorder.record_planning(result.latency_seconds)
            if not result.success or result.segment is None:
                return {
                    "success": False,
                    "message": result.message,
                    "failed_repetition": repetition,
                    "latencies_seconds": latencies,
                }
            selected.append(result.target)
            timing = result.segment.get("timing", {})
            cycle_seconds = timing.get("estimated_cycle_seconds")
            if cycle_seconds is None:
                command_times = result.segment.get("time_from_start_seconds", [])
                if command_times:
                    cycle_seconds = command_times[-1]
            if cycle_seconds is not None:
                recorder.record_cycle(float(cycle_seconds))
            current_q = result.path[-1]
            current_dock = np.asarray(result.segment["amr_dock_position"], dtype=float)
            remove_carton(scene, str(result.target))
            if len(selected) >= len(library.get("entries", [])):
                break
        if reference_targets is None:
            reference_targets = selected
        elif selected != reference_targets:
            raise RuntimeError("certified runtime benchmark was not deterministic")
    values = np.asarray(latencies, dtype=float)
    separated_timing = recorder.report(planning_deadline_seconds=0.05)
    return {
        "success": True,
        "repetitions": repetitions,
        "samples": len(latencies),
        "segments_per_repetition": len(reference_targets or []),
        "targets": reference_targets or [],
        "mean_latency_ms": float(np.mean(values) * 1000.0),
        "p95_latency_ms": float(np.percentile(values, 95.0) * 1000.0),
        "max_latency_ms": float(np.max(values) * 1000.0),
        "deadline_ms": 50.0,
        "within_deadline": bool(np.all(values <= 0.05)),
        "separated_timing": separated_timing,
    }


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Compile or benchmark a certified FANUC runtime trajectory library")
    parser.add_argument("--plan", help="Collision-checked online plan to compile")
    parser.add_argument("--library", help="Compiled runtime library to benchmark")
    parser.add_argument("--output")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--repetitions", type=int, default=100)
    args = parser.parse_args(argv)
    if args.plan:
        if not args.output:
            parser.error("--output is required with --plan")
        source = json.loads(Path(args.plan).read_text(encoding="utf-8"))
        compiled = compile_runtime_library(source)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(compiled, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"output": str(output), "entries": len(compiled["entries"])}, ensure_ascii=False, indent=2))
        return
    if args.library and args.benchmark:
        library = json.loads(Path(args.library).read_text(encoding="utf-8"))
        print(
            json.dumps(
                benchmark_runtime_library(library, repetitions=args.repetitions),
                ensure_ascii=False,
                indent=2,
            )
        )
        return
    parser.error("use --plan/--output to compile, or --library --benchmark")


if __name__ == "__main__":
    main()
