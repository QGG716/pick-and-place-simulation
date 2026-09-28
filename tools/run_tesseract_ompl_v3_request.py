"""One empty-roadmap FANUC LazyPRM request; never rerun automatically."""
from __future__ import annotations
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
from time import perf_counter

from run_tesseract_ompl_comparison import fixture_context, ROOT
from run_tesseract_ompl_audit import sha, write
from tesseract_ompl_v3_identity import verify_checkers
from unloading_sim.planning_contract import FreeMotionRequest, PlanningBudget
from unloading_sim.tesseract_scene import export_scene
from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
from unloading_sim.tesseract_ompl_backend import NativeWorker, TesseractOMPLBackend


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--worker", required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Lock before all setup; even a failed preparation is not silently retried.
    with (args.output / "started.json").open("x") as f:
        json.dump(dict(started_utc=datetime.now(timezone.utc).isoformat()), f)
    proof = verify_checkers(ROOT)
    write(args.output / "checker-identity.json", proof)
    fixtures = ROOT / "tests/fixtures/tesseract_ompl"
    world, segment, connector, _ = fixture_context(fixtures / "historical_state.json",
                                                  fixtures / "historical_segment.json")
    scene = export_scene(connector, world.all_obstacles, stage="pregrasp")
    assert scene["fingerprint"] == proof["scene_fingerprint"]
    assert scene["constraints"] == proof["constraints"]
    assert scene["constraints"]["edge_resolution_rad"] == .04
    # Only these two endpoints enter the request. No saved legacy path is read here.
    start, goal = segment["path"][78], segment["path"][208]
    config = OMPLPlannerConfig(name="lazy_prm")
    request = FreeMotionRequest("empty_detour_lazy_v3", world.snapshot["scene_fingerprint"],
        scene["fingerprint"], scene["model_fingerprint"], scene["tool_fingerprint"],
        scene["policy_fingerprint"], "pregrasp", tuple(scene["joint_names"]), tuple(start), tuple(goal),
        scene["constraints"], scene["frames"], scene["attachment"], 71070,
        budget=PlanningBudget(max_state_checks=100000, max_attempts=1))
    class RecordingWorker(NativeWorker):
        plan_calls = 0
        def call(self, message, cancelled=lambda: False):
            raw = super().call(message, cancelled)
            if message.get("operation") == "configure":
                kind = "configuration"
            else:
                self.plan_calls += 1
                kind = "native-result" if self.plan_calls == 1 else f"native-rejection-probe-{self.plan_calls}"
            write(args.output / f"{kind}.json", raw)
            print(json.dumps(dict(event=kind, status=raw["status"], counters=raw.get("counters"))), flush=True)
            return raw
    worker = RecordingWorker(args.worker)
    backend = TesseractOMPLBackend(worker=worker, planner_config=config, profile=True)
    authority_calls = 0
    def authority(points):
        nonlocal authority_calls
        authority_calls += 1
        assert authority_calls == 1
        began = perf_counter()
        rejection = connector._path_failure(points, world.all_obstacles, stage="pregrasp",
                                             diagnostic_origin="v3_fixed_lazy_request")
        write(args.output / "authority.json", dict(accepted=rejection is None, failure=rejection,
            wall_s=perf_counter()-began, statistics=dict(connector._statistics)))
        return rejection
    try:
        configured_request_started = perf_counter()
        # setup() only: no endpoint checking, sampling, edges or search.
        configured = worker.call(dict(operation="configure", scene=scene, q_start=start, q_goal=goal,
            seed=71070, max_state_checks=100000, planner_config=config.to_mapping()))
        assert configured["status"] == "CONFIGURED" and not configured["search_started"]
        assert configured["initial_roadmap_vertices"] == configured["initial_roadmap_edges"] == 0
        manifest = dict(schema="fixed_engineering_request_v3", worker_sha256=sha(args.worker),
            code_base="eab8d9de69786e2069f243fd8b8d994af86e4162",
            sources={str(p.relative_to(ROOT)): sha(p) for p in [ROOT / "native/tesseract_ompl/worker.cpp",
                Path(__file__), ROOT / "tools/tesseract_ompl_v3_identity.py",
                *sorted((ROOT / "src/unloading_sim").glob("*.py"))]},
            seed=71070, source_path_indices=[78, 208], q_start=start, q_goal=goal,
            scene_fingerprint=scene["fingerprint"], model_fingerprint=scene["model_fingerprint"],
            tool_fingerprint=scene["tool_fingerprint"], policy_fingerprint=scene["policy_fingerprint"],
            attachment=scene["attachment"], constraints=scene["constraints"], budget=asdict(request.budget),
            planner_config=config.to_mapping(), effective_planner=configured["effective_planner"],
            profile_enabled=True, first_exact_only=True, historical_path_injected=False,
            initial_roadmap="empty", environment_prewarmed_by_configuration=True,
            isaac_run=False, full_task_run=False)
        # Freeze actual post-setup parameters on disk BEFORE invoking search.
        write(args.output / "request.json", manifest)
        write(args.output / "scene.json", scene)
        result = backend.plan(request, scene, authority)
        write(args.output / "result.json", result.to_mapping())
        write(args.output / "completion.json", dict(authority_calls=authority_calls,
            status=result.status.value, deliverable=result.deliverable,
            configuration_and_request_s=perf_counter()-configured_request_started))
        print(json.dumps(dict(event="finished", status=result.status.value,
                              deliverable=result.deliverable)), flush=True)
    finally:
        worker.log.flush(); worker.log.seek(0)
        (args.output / "native.log").write_text(worker.log.read(), encoding="utf-8")
        worker.close()


if __name__ == "__main__":
    main()
