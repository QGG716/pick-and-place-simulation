"""Small fake transport/task tests. These are not native or physical evidence."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from test_tesseract_ompl_contract import inputs, FakeWorker
from tools.run_tesseract_ompl_task import parse_args, task_backend, run_single_candidate, write_json
from unloading_sim.planning_contract import PlanningStatus
from unloading_sim.tesseract_ompl_backend import NativePlanningBlocked, connect_free_motion


def args(*extra):
    return parse_args(["--state", "state.json", "--segment", "segment.json",
                       "--worker", "worker", "--output", "result.json", *extra])


def test_single_candidate_defaults_and_explicit_transport():
    default = task_backend(args(), FakeWorker())
    assert default.planner_config.name == "rrt_connect"
    assert default.max_attempts == 2 and not default.stop_on_native_block
    worker = FakeWorker()
    backend = task_backend(args("--ompl-planner", "lazy_prm", "--native-max-attempts", "1",
                                "--stop-on-native-block"), worker)
    r, s = inputs()
    result = backend.plan(r, s, lambda path: None)
    assert result.deliverable and result.backend == "tesseract_ompl"
    assert worker.calls[0]["planner_config"]["name"] == "lazy_prm"
    assert result.diagnostics["effective_planner"]["type"] == "ompl::geometric::LazyPRM"
    assert backend.max_attempts == 1 and backend.max_state_checks == 100000


@pytest.mark.parametrize("extra", [["--ompl-planner", "unknown"], ["--unknown", "1"]])
def test_unknown_cli_rejected(extra):
    with pytest.raises(SystemExit): args(*extra)


@pytest.mark.parametrize("effective", [None, "invalid", {"type": "ompl::geometric::RRTConnect"},
    {"type": "ompl::geometric::LazyPRM", "star": True, "max_nearest_neighbors": 5},
    {"type": "ompl::geometric::LazyPRM", "star": False, "max_nearest_neighbors": 10}])
def test_echoed_config_cannot_disguise_wrong_planner(effective):
    class WrongWorker(FakeWorker):
        def call(self, data, cancelled):
            raw = super().call(data, cancelled)
            raw["effective_planner"] = effective
            return raw
    r, s = inputs()
    backend = task_backend(args("--ompl-planner", "lazy_prm"), WrongWorker())
    result = backend.plan(r, s, lambda p: pytest.fail("wrong planner reached authority"))
    assert result.status == PlanningStatus.UNSUPPORTED_CONSTRAINT
    assert result.backend == "tesseract_ompl" and not result.deliverable and not result.path


def connector_stub():
    return SimpleNamespace(_context_identity=lambda *a, **kw: "identity",
        _limit=lambda *a: None, _remaining_wall_time=lambda: None,
        budget=SimpleNamespace(stage_wall_time_s=None),
        _path_failure=lambda *a, **kw: None, _remember_path=lambda *a: None,
        _statistics={"connection_attempts": 0, "path_connection_wall_seconds_inclusive": 0.})


@pytest.mark.parametrize("status", ["BUDGET_EXHAUSTED", "UNSUPPORTED_CONSTRAINT", "INTERNAL_ERROR",
                                  "AUTHORITY_REJECTED", "STALE_SCENE", "CANCELLED", "BACKEND_UNAVAILABLE"])
def test_stop_records_first_block_before_outer_retry(monkeypatch, status):
    from unloading_sim.planning_contract import FreeMotionResult
    r, s = inputs()
    monkeypatch.setattr("unloading_sim.tesseract_scene.export_scene", lambda *a, **kw: s)
    backend = task_backend(args("--stop-on-native-block", "--native-max-attempts", "1"), FakeWorker())
    calls = []
    def plan(request, scene, authority):
        calls.append(request)
        return FreeMotionResult(PlanningStatus(status), "tesseract_ompl")
    monkeypatch.setattr(backend, "plan", plan)
    c = connector_stub()
    with pytest.raises(NativePlanningBlocked):
        for seed in [71081, 72090, 73100]:
            connect_free_motion(c, backend, [0.], [1.], [], seed=seed,
                                iteration_budget=200, stage="pregrasp")
    assert len(calls) == len(c.free_motion_records) == 1
    assert calls[0].budget.max_attempts == 1 and calls[0].budget.wall_time_s is None
    assert c.free_motion_records[0]["request"]["q_goal"] == (1.,)


def test_export_block_is_recorded_and_default_does_not_raise(monkeypatch):
    def fail(*a, **kw): raise ValueError("unsupported process constraint")
    monkeypatch.setattr("unloading_sim.tesseract_scene.export_scene", fail)
    for stop in [False, True]:
        c = connector_stub()
        backend = task_backend(args(*(["--stop-on-native-block"] if stop else [])), FakeWorker())
        if stop:
            with pytest.raises(NativePlanningBlocked):
                connect_free_motion(c, backend, [0.], [1.], [], seed=1, iteration_budget=5, stage="pregrasp")
        else:
            path, failure, evidence = connect_free_motion(c, backend, [0.], [1.], [], seed=1,
                                                         iteration_budget=5, stage="pregrasp")
            assert not path and failure["reason"] == "UNSUPPORTED_CONSTRAINT"
        assert len(c.free_motion_records) == 1 and not backend.worker.calls


def test_experiment_authority_rejection_never_triggers_probe_or_retry():
    r, s = inputs(); worker = FakeWorker()
    backend = task_backend(args("--stop-on-native-block", "--native-max-attempts", "1"), worker)
    result = backend.plan(replace(r, budget=replace(r.budget, max_attempts=1)), s,
                          lambda p: {"reason": "COLLISION", "q_rad": [.5]})
    assert result.status == PlanningStatus.AUTHORITY_REJECTED and not result.deliverable
    assert len(worker.calls) == 1


def test_blocked_single_task_never_exports_history_or_execution_readiness(tmp_path):
    c = connector_stub()
    c.start_planning_request = lambda: None
    called = []
    def plan(**kw):
        called.append(kw)
        assert "history_hint" not in kw and len(kw["grasp_candidates"]) == 1
        raise NativePlanningBlocked({"status": "BUDGET_EXHAUSTED", "stage": "pregrasp"})
    c.plan = plan
    world = SimpleNamespace(cartons=[SimpleNamespace(name="target")], all_obstacles=[], receiver=None,
        support_graph=SimpleNamespace(supported_by={"target": []}), policy=SimpleNamespace(data={"suction": {}}),
        snapshot={"scene_fingerprint": "scene"})
    history = dict(target="target", face="front", path=[[0.], [99.]],
                   contact={"requested_virtual_task_tcp_pose_world": [[1.]], "actual_q_rad": [1.]})
    result = run_single_candidate(world, history, c, task_backend(args(), FakeWorker()))
    assert len(called) == result["task_call_count"] == 1
    assert not result["success"] and result["selected_trajectory_segment"] is None
    assert not result["simulation_execution_ready"] and not result["isaac_run"]
    assert result["execution_preflight"] == result["time_parameterization_status"] == "NOT_RUN"
    assert not result["all_candidates_proven_infeasible"]
    write_json(tmp_path / "result.json", result)


def test_free_path_process_handoff_rejects_discontinuity():
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
    full = [np.zeros(6)]
    stages = {"home": [0, 0]}
    free = [np.zeros(6), np.ones(6)*.1]
    LayoutTrajectoryConnector._append_stage(full, stages, "pregrasp", free)
    with pytest.raises(ValueError, match="preceding endpoint"):
        LayoutTrajectoryConnector._append_stage(full, stages, "contact", [np.ones(6)*.11, np.ones(6)*.2])
    LayoutTrajectoryConnector._append_stage(full, stages, "contact", [free[-1], np.ones(6)*.2])
    assert stages["pregrasp"][1] == stages["contact"][0] == 1
