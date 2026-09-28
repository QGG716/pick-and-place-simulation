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
    ap.add_argument("--ompl-planner", choices=["rrt_connect", "lazy_prm"], default="rrt_connect")
    ap.add_argument("--native-max-attempts", type=int, default=2)
    ap.add_argument("--stop-on-native-block", action="store_true",
                    help="Stop this experiment before the connector retries an expensive blocked request")
    ap.add_argument("--profile", action="store_true")
    return ap.parse_args(argv)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


class RecordingWorker(NativeWorker):
    """Persist actual transport inputs/results; no alternate planner or paths."""
    def __init__(self, executable, directory):
        super().__init__(executable)
        self.directory_evidence = directory
        self.call_count = 0

    def call(self, message, cancelled=lambda: False):
        self.call_count += 1
        stem = self.directory_evidence / f"native-{self.call_count:02d}"
        write_json(stem.with_suffix(".request.json"), message)
        result = super().call(message, cancelled)
        write_json(stem.with_suffix(".result.json"), result)
        return result


def task_backend(args, worker=None):
    return create_free_motion_backend("tesseract_ompl", executable=args.worker, worker=worker,
        planner_config=OMPLPlannerConfig(name=args.ompl_planner), max_state_checks=100000,
        max_attempts=args.native_max_attempts, stop_on_native_block=args.stop_on_native_block,
        profile=args.profile)


def run_single_candidate(scene, historical, connector, backend):
    connector.free_motion_backend = backend
    connector.start_planning_request()
    connector.progress_callback = lambda event: print(json.dumps(event), flush=True)
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
            complete_geometry_status="PASS" if outcome and outcome.success else "NOT_AVAILABLE",
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
    args.output.parent.mkdir(parents=True, exist_ok=True)
    evidence_dir = args.output.parent / (args.output.stem + "-native")
    # A rerun must use a new explicit artifact location; never overwrite evidence.
    evidence_dir.mkdir(exist_ok=False)
    if args.output.exists():
        raise FileExistsError(args.output)
    setup = perf_counter()
    scene, historical, connector, _ = fixture_context(args.state, args.segment)
    worker = RecordingWorker(args.worker, evidence_dir)
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
                          for p in [args.state, args.segment, Path(args.worker)]},
            source_sha256={p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [*sorted((ROOT / "src/unloading_sim").glob("*.py")),
                                    Path(__file__).resolve(), ROOT / "native/tesseract_ompl/worker.cpp"]})
        write_json(args.output.with_suffix(".identity.json"), identity)
        setup_s = perf_counter()-setup
        result = run_single_candidate(scene, historical, connector, backend)
        result.update(setup_and_identity_s=setup_s, native_call_count=worker.call_count)
        write_json(args.output, result)
        print(json.dumps({key: result[key] for key in ["task_entry", "success", "failure", "elapsed_s", "isaac_run"]}), flush=True)
    finally:
        backend.worker.close()
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
