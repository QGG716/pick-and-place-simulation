"""Exactly one replay of v0.4's first native request through the production backend.

No connector.plan, IK, historical path injection, full task or Isaac execution.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from tools.run_tesseract_ompl_comparison import fixture_context
from tools.run_tesseract_ompl_task import RecordingWorker, write_json
from unloading_sim.planning_contract import FreeMotionRequest, PlanningBudget, fingerprint
from unloading_sim.tesseract_scene import export_scene
from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
from unloading_sim.tesseract_ompl_backend import TesseractOMPLBackend


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--worker", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--planner-config", type=Path)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "started.json", dict(utc=datetime.now(timezone.utc).isoformat(), command=sys.argv))
    original_path = ROOT / "docs/validation/evidence/backend_tesseract_ompl_v0_4/task-result-native/native-01.request.json"
    original = json.loads(original_path.read_text())
    assert original["seed"] == 71081 and original["max_state_checks"] == 100000
    assert original["scene"]["stage"] == "pregrasp" and original["scene"]["attachment"] is None
    config = OMPLPlannerConfig(**(json.loads(args.planner_config.read_text()) if args.planner_config else original["planner_config"]))
    for key in ("name", "max_samples", "max_roadmap_vertices", "max_roadmap_edges"):
        assert config.to_mapping()[key] == original["planner_config"][key]
    fixtures = ROOT / "tests/fixtures/tesseract_ompl"
    world, _, connector, _ = fixture_context(fixtures / "historical_state.json", fixtures / "historical_segment.json")
    scene = export_scene(connector, world.all_obstacles, stage="pregrasp")
    # Same directory and assets: insist on exact exported content, not a hand-edited hash.
    if scene != original["scene"]:
        raise ValueError("re-export differs from frozen v0.4 scene; no search authorized")
    def revision():
        return fingerprint([connector._context_identity(world.all_obstacles, stage="pregrasp"), None])
    request = FreeMotionRequest("v5_original_pregrasp_71081", revision(), scene["fingerprint"],
        scene["model_fingerprint"], scene["tool_fingerprint"], scene["policy_fingerprint"],
        "pregrasp", tuple(scene["joint_names"]), tuple(original["q_start"]), tuple(original["q_goal"]),
        scene["constraints"], scene["frames"], None, original["seed"],
        budget=PlanningBudget(max_state_checks=original["max_state_checks"], max_attempts=1),
        current_revision=revision)
    worker = RecordingWorker(args.worker, args.output)
    backend = TesseractOMPLBackend(worker=worker, planner_config=config, profile=original["profile"],
        max_attempts=1, stop_on_native_block=True, roadmap_diagnostics=True)
    authority_calls = 0
    def authority(path):
        nonlocal authority_calls
        authority_calls += 1
        assert authority_calls == 1
        began = perf_counter()
        rejection = connector._path_failure(path, world.all_obstacles, stage="pregrasp",
                                            diagnostic_origin="v5_frozen_pregrasp")
        write_json(args.output / "authority.json", dict(accepted=rejection is None, failure=rejection,
            wall_s=perf_counter()-began, statistics=dict(connector._statistics)))
        return rejection
    sources = [*sorted((ROOT / "src/unloading_sim").glob("*.py")),
               *sorted((ROOT / "native/tesseract_ompl").glob("*.h")),
               ROOT / "native/tesseract_ompl/worker.cpp", Path(__file__),
               ROOT / "tools/run_tesseract_ompl_task.py", ROOT / "tools/run_tesseract_ompl_comparison.py"]
    write_json(args.output / "identity.json", dict(review_base="92471392b5d25fa76f2846c361cc7a159ecfa5aa",
        worker_sha256=sha(args.worker), original_request_sha256=sha(original_path),
        scene_exactly_equal_to_original=True, asset_path_relocation=False, seed=request.seed,
        q_start=request.q_start, q_goal=request.q_goal, scene_fingerprint=scene["fingerprint"],
        budget=asdict(request.budget), planner_config=config.to_mapping(),
        config_file_sha256=sha(args.planner_config) if args.planner_config else None,
        sources={p.relative_to(ROOT).as_posix(): sha(p) for p in sources},
        full_task_run=False, isaac_run=False, historical_path_injected=False))
    try:
        began = perf_counter()
        result = backend.plan(request, scene, authority)
        assert worker.call_count == 1
        write_json(args.output / "result.json", result.to_mapping())
        write_json(args.output / "completion.json", dict(status=result.status.value,
            deliverable=result.deliverable, native_requests=worker.call_count, authority_calls=authority_calls,
            adapter_wall_s=perf_counter()-began, full_task_run=False, isaac_run=False))
        print(json.dumps(dict(status=result.status.value, counters=result.counters)), flush=True)
    finally:
        worker.log.flush(); worker.log.seek(0)
        (args.output / "native.log").write_text(worker.log.read(), encoding="utf-8")
        worker.close()


if __name__ == "__main__":
    main()
