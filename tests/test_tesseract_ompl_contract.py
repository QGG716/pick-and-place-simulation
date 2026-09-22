"""Transport/contract regressions; fake workers are NOT native evidence."""
from dataclasses import replace
import subprocess
import sys

import numpy as np
import pytest

from unloading_sim.planning_contract import (
    FreeMotionRequest, PlanningBudget, PlanningStatus, fingerprint)
from unloading_sim.tesseract_ompl_backend import TesseractOMPLBackend, create_free_motion_backend


def inputs():
    scene = dict(joint_limits=[[-2., 2.]], joint_names=["J1"], constraints={"motion": "free_joint_space", "events": [], "edge_resolution_rad": .055},
                 frames={}, attachment=None, stage="pregrasp", model_fingerprint="model",
                 tool_fingerprint="tool", policy_fingerprint="policy", mesh_files={})
    scene["fingerprint"] = fingerprint(scene)
    r = FreeMotionRequest("test", "r1", scene["fingerprint"], "model", "tool", "policy", "pregrasp",
                          ("J1",), (0.,), (1.,), scene["constraints"], {}, None, 4,
                          budget=PlanningBudget(max_state_checks=100, max_attempts=2))
    return r, scene


class FakeWorker:
    def __init__(self, probe_rejects=False):
        self.calls = []
        self.probe_rejects = probe_rejects

    def call(self, data, cancelled):
        self.calls.append(data)
        if self.probe_rejects and data["q_start"] == data["q_goal"]:
            return dict(status="INVALID_START", counters={"state_checks": 1}, timings={})
        return dict(status="CANDIDATE", candidate_found=True, exact_solution=True, native_validated=True,
                    path=[list(data["q_start"]), list(data["q_goal"])],
                    scene_fingerprint=data["scene"]["fingerprint"], counters={"state_checks": 4}, timings={})


def test_core_import_has_no_native_dependency():
    code = "import sys;import unloading_sim.planner,unloading_sim.layout_trajectory;assert not any(n.startswith('tesseract_robotics') for n in sys.modules)"
    subprocess.run([sys.executable, "-c", code], check=True)


def test_explicit_unavailable_is_not_legacy():
    r, s = inputs()
    backend = TesseractOMPLBackend("/nonexistent/tesseract-worker")
    try:
        result = backend.plan(r, s, lambda p: None)
        assert result.status == PlanningStatus.BACKEND_UNAVAILABLE
        assert result.backend == "tesseract_ompl" and not result.deliverable
    finally:
        backend.worker.close()


@pytest.mark.parametrize("q,status", [((3.,), PlanningStatus.INVALID_START),
                                      ((float("nan"),), PlanningStatus.INVALID_START)])
def test_invalid_start_never_clamped(q, status):
    r, s = inputs(); w = FakeWorker()
    result = TesseractOMPLBackend(worker=w).plan(replace(r, q_start=q), s, lambda p: None)
    assert result.status == status and not w.calls


def test_invalid_goal():
    r, s = inputs()
    assert TesseractOMPLBackend(worker=FakeWorker()).plan(replace(r, q_goal=(3.,)), s, lambda p: None).status == PlanningStatus.INVALID_GOAL


def test_cancel_and_stale_before_or_after_authority():
    r, s = inputs(); w = FakeWorker()
    assert TesseractOMPLBackend(worker=w).plan(replace(r, cancelled=lambda: True), s, lambda p: None).status == PlanningStatus.CANCELLED
    revision = ["r1"]
    def authority(path): revision[0] = "r2"
    result = TesseractOMPLBackend(worker=w).plan(replace(r, current_revision=lambda: revision[0]), s, authority)
    assert result.status == PlanningStatus.STALE_SCENE and not result.path


def test_changed_policy_and_scene_fingerprint_rejected():
    r, s = inputs(); s["policy_fingerprint"] = "new"
    assert TesseractOMPLBackend(worker=FakeWorker()).plan(r, s, lambda p: None).status == PlanningStatus.STALE_SCENE


def test_unknown_constraint_is_not_dropped():
    r, s = inputs()
    r = replace(r, constraints={**r.constraints, "upright_payload": True})
    assert TesseractOMPLBackend(worker=FakeWorker()).plan(r, s, lambda p: None).status == PlanningStatus.UNSUPPORTED_CONSTRAINT


def test_authority_disagreement_stops_without_seed_retry():
    r, s = inputs(); w = FakeWorker()
    result = TesseractOMPLBackend(worker=w).plan(r, s, lambda p: {"reason": "COLLISION", "q_rad": [.5], "edge": 0})
    assert result.status == PlanningStatus.AUTHORITY_REJECTED and result.candidate_found
    assert not result.path and not result.deliverable and len(w.calls) == 2
    assert result.diagnostics["attempts"][0]["repair"] == "MODEL_OR_POLICY_MISMATCH_STOP"


def test_grid_repair_changes_checker_and_rebuilds_without_cached_solution():
    r, s = inputs(); w = FakeWorker(probe_rejects=True); count = [0]
    def authority(path):
        count[0] += 1
        return {"reason": "COLLISION", "q_rad": [.5]} if count[0] == 1 else None
    result = TesseractOMPLBackend(worker=w).plan(r, s, authority)
    assert result.deliverable and len(w.calls) == 3
    assert w.calls[2]["l1_resolution_rad"] < w.calls[0]["l1_resolution_rad"]
    assert w.calls[2]["max_state_checks"] == 95
    assert w.calls[0]["seed"] == w.calls[2]["seed"]
    assert result.diagnostics["attempts"][1]["repeated_candidate"]


def test_legacy_uses_same_authority_and_explicit_factory():
    r, s = inputs()
    result = create_free_motion_backend("legacy").plan(r, s, lambda p: None)
    assert result.deliverable and result.backend == "legacy"
    with pytest.raises(ValueError): create_free_motion_backend("typo")
    with pytest.raises(ValueError): PlanningBudget(max_state_checks=0)


def test_explicit_wall_budget_includes_authority(monkeypatch):
    import unloading_sim.tesseract_ompl_backend as module
    clock = [0.]
    monkeypatch.setattr(module, "perf_counter", lambda: clock[0])
    r, s = inputs()
    r = replace(r, budget=PlanningBudget(wall_time_s=1.))
    def authority(path):
        clock[0] = 2.
    result = TesseractOMPLBackend(worker=FakeWorker()).plan(r, s, authority)
    assert result.status == PlanningStatus.BUDGET_EXHAUSTED and not result.deliverable


def test_approximate_candidate_is_recorded_but_not_delivered():
    r, s = inputs()
    class ApproximateWorker:
        def call(self, data, cancelled):
            return dict(status="BUDGET_EXHAUSTED", candidate_found=True, exact_solution=False,
                        native_validated=False, approximate_solution_found=True,
                        counters={"state_checks": 100}, timings={})
    result = TesseractOMPLBackend(worker=ApproximateWorker()).plan(r, s, lambda p: pytest.fail("approximate path reached authority"))
    assert result.candidate_found and not result.exact_solution and not result.deliverable
    assert result.path == [] and result.status == PlanningStatus.BUDGET_EXHAUSTED


def test_legacy_rejects_changed_snapshot_and_request_policy():
    r, s = inputs()
    backend = create_free_motion_backend("legacy")
    result = backend.plan(replace(r, policy_fingerprint="different"), s,
                          lambda p: pytest.fail("mismatched policy reached authority"))
    assert result.status == PlanningStatus.UNSUPPORTED_CONSTRAINT
    s["attachment"] = {"changed": True}
    result = backend.plan(r, s, lambda p: pytest.fail("stale snapshot reached authority"))
    assert result.status == PlanningStatus.STALE_SCENE
