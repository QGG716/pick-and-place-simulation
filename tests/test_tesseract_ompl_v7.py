"""Host wiring tests; fake communication is not native/configure evidence."""
import json
from copy import deepcopy
import pytest

from test_tesseract_ompl_contract import FakeWorker, inputs
from test_tesseract_ompl_task import args
from tools.run_tesseract_ompl_task import RecordingWorker, task_backend
from unloading_sim.tesseract_ompl_backend import NativeWorker, NativePlanningBlocked
from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig, EndpointSamplingConfig


@pytest.mark.parametrize("change", [None, "q_start", "q_goal", "seed", "model", "policy", "sampling"])
def test_production_request_identity_exact_before_native_send(tmp_path, monkeypatch, change):
    request, scene = inputs()
    config = OMPLPlannerConfig(name="lazy_prm", sampling=EndpointSamplingConfig()).to_mapping()
    config_file = tmp_path / "config.json"; config_file.write_text(json.dumps(config))
    expected = dict(q_start=list(request.q_start), q_goal=list(request.q_goal), scene=deepcopy(scene),
                    seed=request.seed, planner_config=deepcopy(config))
    if change in {"q_start", "q_goal"}: expected[change][0] += 1e-13
    if change == "seed": expected["seed"] += 1
    if change in {"model", "policy"}: expected["scene"][change+"_fingerprint"] = "changed"
    if change == "sampling": expected["planner_config"]["sampling"]["local_probability"] = .4
    reference = tmp_path / "expected.json"; reference.write_text(json.dumps(expected))
    calls = []
    def communicate(self, message, cancelled):
        calls.append(message)
        raw = FakeWorker().call(message, cancelled)
        raw["effective_sampler"] = {"configuration": message["planner_config"]["sampling"]}
        return raw
    monkeypatch.setattr(NativeWorker, "call", communicate)
    worker = RecordingWorker("not-started", tmp_path, reference)
    backend = task_backend(args("--planner-config", str(config_file)), worker)
    try:
        if change:
            with pytest.raises(NativePlanningBlocked): backend.plan(request, scene, lambda p: None)
            assert calls == [] and worker.activity()["communication_attempts"] == 0
        else:
            assert backend.plan(request, scene, lambda p: None).deliverable
            assert isinstance(calls[0]["q_start"], tuple)
            assert worker.activity()["native_results_returned"] == 1
    finally:
        worker.close()


def test_communication_attempt_does_not_prove_state_or_search(tmp_path, monkeypatch):
    request, scene = inputs()
    message = dict(q_start=request.q_start, q_goal=request.q_goal, seed=request.seed, scene=scene)
    worker = RecordingWorker("unused", tmp_path)
    def broken(*args): raise RuntimeError("startup failed")
    monkeypatch.setattr(NativeWorker, "call", broken)
    with pytest.raises(RuntimeError): worker.call(message)
    activity = worker.activity()
    assert activity["communication_attempts"] == 1 and activity["native_results_returned"] == 0
    assert activity["pending_or_failed_communication_attempts"] == 1
    assert activity["requests_with_state_checks"] == activity["ompl_searches"] == 0
    worker.close()


def test_configure_direct_and_ompl_counts_are_distinct(tmp_path, monkeypatch):
    request, scene = inputs()
    message = dict(q_start=request.q_start, q_goal=request.q_goal, seed=request.seed, scene=scene)
    responses = iter([
        dict(status="CONFIGURED", search_started=False),
        dict(status="CANDIDATE", direct_valid=True, search_started=False,
             counters={"actual_state_computations": 10}),
        dict(status="BUDGET_EXHAUSTED", direct_valid=False, search_started=True,
             counters={"actual_state_computations": 100000})])
    monkeypatch.setattr(NativeWorker, "call", lambda *a: next(responses))
    worker = RecordingWorker("unused", tmp_path)
    worker.call(dict(message, operation="configure"))
    worker.call(message); worker.call(message)
    activity = worker.activity()
    assert activity["native_results_returned"] == 3 and activity["configure_results"] == 1
    assert activity["plan_results"] == activity["requests_with_state_checks"] == activity["direct_motion_checks"] == 2
    assert activity["ompl_searches"] == 1 and activity["actual_state_computations"] == 100010
    worker.close()
