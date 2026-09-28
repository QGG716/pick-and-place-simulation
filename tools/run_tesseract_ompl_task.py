"""One real connector.plan call using a frozen previously solved contact candidate.

All contact/extraction/place/release logic is the existing connector. A returned
full segment still requires the independent execution preflight before Isaac.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
from tools.run_tesseract_ompl_comparison import fixture_context
from unloading_sim.tesseract_ompl_backend import create_free_motion_backend, NativeWorker, NativePlanningBlocked
from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
from unloading_sim.tesseract_scene import export_scene


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--state", type=Path, required=True)
    ap.add_argument("--segment", type=Path, required=True)
    ap.add_argument("--worker", required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--ompl-planner", choices=["rrt_connect", "lazy_prm"])
    ap.add_argument("--planner-config", type=Path, help="Strict OMPLPlannerConfig JSON")
    ap.add_argument("--expected-first-request", type=Path,
                    help="Compare identity only, before first native search; never read a saved path")
    ap.add_argument("--execution-config", type=Path)
    ap.add_argument("--native-max-attempts", type=int, default=2)
    ap.add_argument("--stop-on-native-block", action="store_true",
                    help="Stop this experiment before the connector retries an expensive blocked request")
    ap.add_argument("--profile", action="store_true")
    return ap.parse_args(argv)


def planner_configuration(args):
    if args.planner_config is None:
        return OMPLPlannerConfig(name=args.ompl_planner or "rrt_connect")
    def strict_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate configuration key: {key}")
            result[key] = value
        return result
    value = json.loads(args.planner_config.read_text(encoding="utf-8"), object_pairs_hook=strict_object)
    if not isinstance(value, dict):
        raise ValueError("planner configuration must be an object")
    try:
        config = OMPLPlannerConfig(**value)
    except TypeError as exc:
        raise ValueError(f"unsupported planner configuration: {exc}") from exc
    if args.ompl_planner is not None and args.ompl_planner != config.name:
        raise ValueError("--ompl-planner conflicts with --planner-config")
    return config


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


class RecordingWorker(NativeWorker):
    """Persist actual transport inputs/results; no alternate planner or paths."""
    def __init__(self, executable, directory, expected_first_request=None):
        super().__init__(executable)
        self.directory_evidence = directory
        self.call_count = 0
        self.transport_call_count = 0
        self.returned_results = []
        self.expected_first_request = expected_first_request

    def activity(self):
        """Lower bounds from returned worker results, never inferred from a send attempt."""
        plans = [r for r in self.returned_results if r["operation"] == "plan"]
        return dict(requests_prepared=self.call_count, communication_attempts=self.transport_call_count,
            native_results_returned=len(self.returned_results),
            configure_results=sum(r["operation"] == "configure" for r in self.returned_results),
            plan_results=len(plans), requests_with_state_checks=sum(r["actual_state_computations"] > 0 for r in plans),
            actual_state_computations=sum(r["actual_state_computations"] for r in plans),
            direct_motion_checks=sum(r["direct_checked"] for r in plans),
            ompl_searches=sum(r["search_started"] for r in plans),
            pending_or_failed_communication_attempts=self.transport_call_count-len(self.returned_results),
            count_scope="RETURNED_RESULTS_ONLY; pending communication does not prove zero native work",
            results=self.returned_results)

    def call(self, message, cancelled=lambda: False):
        self.call_count += 1
        stem = self.directory_evidence / f"native-{self.call_count:02d}"
        write_json(stem.with_suffix(".request.json"), message)
        write_json(self.directory_evidence / "activity.json", self.activity())
        if self.call_count == 1 and self.expected_first_request is not None:
            expected = json.loads(self.expected_first_request.read_text(encoding="utf-8"))
            # Full scene equality includes assets, TCP, attachment, constraints and fingerprints.
            fields = ("q_start", "q_goal", "seed", "scene", "planner_config")
            # Compare what actually crosses the JSON transport: request q tuples
            # and JSON arrays are the same sequence, not a model difference.
            wire_message = json.loads(json.dumps(message, allow_nan=False))
            differences = [key for key in fields if expected.get(key) != wire_message.get(key)]
            identity = dict(status="FAIL" if differences else "PASS", differences=differences,
                compared_fields=fields, reference_sha256=hashlib.sha256(self.expected_first_request.read_bytes()).hexdigest(),
                historical_path_or_roadmap_read=False)
            write_json(stem.with_suffix(".identity-check.json"), identity)
            if differences:
                result = dict(status="UNSUPPORTED_CONSTRAINT", source="HOST_IDENTITY_GUARD_NO_NATIVE_CALL",
                              diagnostics={"first_request_identity": identity})
                write_json(stem.with_suffix(".result.json"), result)
                raise NativePlanningBlocked(dict(status="UNSUPPORTED_CONSTRAINT", stage="pregrasp", diagnostics=identity))
        self.transport_call_count += 1
        write_json(self.directory_evidence / "activity.json", self.activity())
        try:
            result = super().call(message, cancelled)
        except Exception as exc:
            write_json(stem.with_suffix(".communication-error.json"),
                       dict(source="HOST_TRANSPORT_EXCEPTION_NOT_NATIVE_RESULT", error=str(exc)))
            raise
        write_json(stem.with_suffix(".result.json"), result)
        self.returned_results.append(dict(operation=message.get("operation", "plan"),
            status=result.get("status"), seed=message["seed"], stage=message["scene"].get("stage"),
            actual_state_computations=result.get("counters", {}).get("actual_state_computations", 0),
            direct_checked="direct_valid" in result, search_started=result.get("search_started") is True))
        write_json(self.directory_evidence / "activity.json", self.activity())
        return result


def task_backend(args, worker=None):
    return create_free_motion_backend("tesseract_ompl", executable=args.worker, worker=worker,
        planner_config=planner_configuration(args), max_state_checks=100000,
        max_attempts=args.native_max_attempts, stop_on_native_block=args.stop_on_native_block,
        profile=args.profile)


def run_single_candidate(scene, historical, connector, backend, *, progress=None):
    connector.free_motion_backend = backend
    connector.start_planning_request()
    connector.progress_callback = progress or (lambda event: print(json.dumps(event), flush=True))
    target = next(b for b in scene.cartons if b.name == historical["target"])
    started = perf_counter()
    outcome = None
    block = None
    try:
        outcome = connector.plan(target=target, face=historical["face"],
            requested_virtual_contact=np.asarray(historical["contact"]["requested_virtual_task_tcp_pose_world"]),
            grasp_candidates=[dict(candidate_id="frozen_previously_solved_contact",
                                   q_rad=historical["contact"]["actual_q_rad"])],
            home_q=np.asarray(historical["path"][0]), all_obstacles=scene.all_obstacles,
            receiver=scene.receiver, support_names=sorted(scene.support_graph.supported_by[target.name]),
            suction=scene.policy.data["suction"], seed=71070)
    except NativePlanningBlocked as exc:
        block = dict(reason=exc.evidence["status"], stage=exc.evidence["stage"],
                     detail=exc.evidence.get("diagnostics"), termination="EXPERIMENT_STOP_ON_NATIVE_BLOCK")
    except Exception as exc:
        block = dict(reason="INTERNAL_ERROR", detail=str(exc), exception_type=type(exc).__name__,
                     termination="EXPERIMENT_STOP_NO_RETRY")
    result = dict(schema="tesseract_real_single_candidate_task_v2", task_entry="LayoutTrajectoryConnector.plan",
            target=target.name, scene_fingerprint=scene.snapshot["scene_fingerprint"],
            fixed_contact_candidate=historical["contact"], success=bool(outcome and outcome.success),
            selected_trajectory_segment=outcome.segment if outcome else None,
            failure=outcome.failure if outcome else block,
            attempts=outcome.attempts if outcome else [],
            statistics=outcome.statistics if outcome else dict(connector._statistics),
            free_motion_records=getattr(connector, "free_motion_records", []),
            elapsed_s=perf_counter()-started, execution_preflight="NOT_RUN", isaac_run=False,
            task_call_count=1, historical_path_injected=False, all_candidates_proven_infeasible=False,
            simulation_execution_ready=False, machine_qualified=False,
            complete_geometry_status="PENDING_VALIDATION" if outcome and outcome.success else "NOT_AVAILABLE",
            execution_trajectory_status="NOT_RUN",
            time_parameterization_status="NOT_RUN", physical_execution_status="NOT_RUN")
    segment = result["selected_trajectory_segment"]
    if segment:
        delta = np.diff(np.asarray(segment["path"]), axis=0)
        result["path_metrics"] = dict(points=len(segment["path"]),
            joint_path_length_l2_rad=float(np.linalg.norm(delta, axis=1).sum()),
            per_joint_total_variation_rad=np.abs(delta).sum(axis=0).tolist(), execution_duration_s=None)
    return result


def main(argv=None):
    args = parse_args(argv)
    planner_configuration(args)  # Reject conflicting/unknown configuration before creating evidence.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    evidence_dir = args.output.parent / (args.output.stem + "-native")
    # A rerun must use a new explicit artifact location; never overwrite evidence.
    evidence_dir.mkdir(exist_ok=False)
    if args.output.exists():
        raise FileExistsError(args.output)
    setup = perf_counter()
    scene, historical, connector, _ = fixture_context(args.state, args.segment)
    worker = RecordingWorker(args.worker, evidence_dir, args.expected_first_request)
    backend = task_backend(args, worker)
    try:
        initial = export_scene(connector, scene.all_obstacles, stage="pregrasp")
        identity = dict(schema="tesseract_single_candidate_identity_v1",
            scene_snapshot_fingerprint=scene.snapshot["scene_fingerprint"],
            target=historical["target"], remaining_cartons=sorted(b.name for b in scene.cartons),
            start_q_rad=historical["path"][0], fixed_contact_candidate=historical["contact"],
            face=historical["face"], seed=71070, native_free_stages=["pregrasp", "transit"],
            initial_native_contract={k: initial[k] for k in ("fingerprint", "model_fingerprint",
                "tool_fingerprint", "policy_fingerprint", "frames", "constraints", "attachment")},
            planner_config=backend.planner_config.to_mapping(), max_state_checks=backend.max_state_checks,
            max_attempts=backend.max_attempts, stop_on_native_block=backend.stop_on_native_block,
            profile=backend.profile, connector_budget=asdict(connector.budget),
            candidate_count=1, history_hint_used=False, roadmap_or_solution_injected=False,
            command=[sys.executable, *sys.argv],
            files_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [args.state, args.segment, Path(args.worker),
                                    args.planner_config, args.expected_first_request, args.execution_config] if p is not None},
            source_sha256={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [*sorted((ROOT / "src/unloading_sim").glob("*.py")),
                                    Path(__file__).resolve(), ROOT / "native/tesseract_ompl/worker.cpp",
                                    *sorted((ROOT / "native/tesseract_ompl").glob("*.h"))]})
        write_json(args.output.with_suffix(".identity.json"), identity)
        setup_s = perf_counter()-setup
        from unloading_sim.tesseract_task_delivery import prepare_task_delivery
        completed_dir = args.output.parent / (args.output.stem + "-stages")
        completed_dir.mkdir(exist_ok=False)
        stage_count = 0
        def save_stage(item):
            nonlocal stage_count
            stage_count += 1
            write_json(completed_dir / f"{stage_count:03d}.json", item)
        connector.stage_evidence_callback = save_stage
        def progress(event):
            with args.output.with_suffix(".progress.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event) + "\n")
            print(json.dumps(event), flush=True)
        result = run_single_candidate(scene, historical, connector, backend, progress=progress)
        result.update(setup_and_identity_s=setup_s, native_request_count=worker.call_count,
                      native_call_count=len(worker.returned_results),
                      native_communication_attempts=worker.transport_call_count,
                      native_activity=worker.activity())
        write_json(args.output, result)
        if result["success"]:
            prepare_task_delivery(result, scene, connector, args.output.parent / (args.output.stem + "-delivery"),
                                  execution_config=args.execution_config, project_root=ROOT)
            write_json(args.output, result)
        print(json.dumps({key: result[key] for key in ["task_entry", "success", "failure", "elapsed_s", "isaac_run"]}), flush=True)
    finally:
        backend.worker.close()
    return 0 if result["simulation_execution_ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
