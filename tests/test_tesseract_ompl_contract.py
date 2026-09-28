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
    scene = dict(joint_limits=[[-2., 2.]], joint_names=["J1"], constraints={"motion": "free_joint_space", "events": [], "edge_resolution_rad": .055, "point_motion_bound_m": .00125, "lever_arm_m": 4.},
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
                    planner_config=data.get("planner_config"),
                    effective_planner={"type": "ompl::geometric::LazyPRM" if data.get("planner_config", {}).get("name") == "lazy_prm" else "ompl::geometric::RRTConnect",
                                       "star": False, "max_nearest_neighbors": 5},
                    path=[list(data["q_start"]), list(data["q_goal"])],
                    scene_fingerprint=data["scene"]["fingerprint"], counters={"state_checks": 4}, timings={})


def test_core_import_has_no_native_dependency():
    code = "import sys;import unloading_sim.planner,unloading_sim.layout_trajectory;assert not any(n.startswith('tesseract_robotics') for n in sys.modules)"
    subprocess.run([sys.executable, "-c", code], check=True)


@pytest.mark.parametrize("name", ["rrt_connect", "lazy_prm"])
def test_formal_planner_configuration_reaches_transport_and_result(name):
    from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
    r, s = inputs(); w = FakeWorker()
    config = OMPLPlannerConfig(name=name)
    result = create_free_motion_backend("tesseract_ompl", worker=w,
        planner_config=config).plan(r, s, lambda p: None)
    assert result.deliverable
    assert w.calls[0]["planner_config"] == result.diagnostics["planner_config"] == config.to_mapping()
    assert ("range_rad" in w.calls[0]) == (name == "rrt_connect")


def test_lazy_cannot_silently_run_on_old_rrt_worker():
    from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
    r, s = inputs()
    class OldWorker(FakeWorker):
        def call(self, data, cancelled):
            result = super().call(data, cancelled)
            result.pop("planner_config")
            return result
    result = TesseractOMPLBackend(worker=OldWorker(),
        planner_config=OMPLPlannerConfig(name="lazy_prm")).plan(r, s,
            lambda p: pytest.fail("unacknowledged planner reached authority"))
    assert result.status == PlanningStatus.UNSUPPORTED_CONSTRAINT and not result.deliverable


@pytest.mark.parametrize("kwargs", [{"name": "lazy_prm_star"}, {"max_samples": True},
    {"max_samples": 0}, {"max_roadmap_vertices": 1}, {"max_roadmap_edges": 5},
    {"max_samples": float("inf")}, {"max_samples": 1.5}])
def test_planner_and_resource_contract_reject_invalid_values(kwargs):
    from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
    with pytest.raises(ValueError): OMPLPlannerConfig(**kwargs)


def test_cli_planner_default_and_explicit_selection():
    from tools.run_m710id70_layout_single_carton import parse_args
    assert parse_args([]).free_motion_backend == "legacy"
    assert parse_args([]).ompl_planner == "rrt_connect"
    assert parse_args(["--free-motion-backend", "tesseract_ompl",
                       "--ompl-planner", "lazy_prm"]).ompl_planner == "lazy_prm"


@pytest.mark.parametrize("when", ["before", "after"])
@pytest.mark.parametrize("change,status", [("cancel", PlanningStatus.CANCELLED),
    ("revision", PlanningStatus.STALE_SCENE), ("wall", PlanningStatus.BUDGET_EXHAUSTED)])
def test_lazy_candidate_pre_and_post_authority_guards(when, change, status, monkeypatch):
    from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
    import unloading_sim.tesseract_ompl_backend as module
    clock = [0.]; cancelled = [False]; revision = ["r1"]; calls = []
    monkeypatch.setattr(module, "perf_counter", lambda: clock[0])
    def update():
        if change == "cancel": cancelled[0] = True
        if change == "revision": revision[0] = "r2"
        if change == "wall": clock[0] = 2.
    class Worker(FakeWorker):
        def call(self, data, cancel):
            raw = super().call(data, cancel)
            if when == "before": update()
            return raw
    def authority(path):
        calls.append(path)
        update()
    r, s = inputs()
    r = replace(r, cancelled=lambda: cancelled[0], current_revision=lambda: revision[0],
                budget=PlanningBudget(wall_time_s=1.))
    result = TesseractOMPLBackend(worker=Worker(), planner_config=OMPLPlannerConfig(name="lazy_prm")).plan(r, s, authority)
    assert result.status == status and not result.path and not result.deliverable
    assert len(calls) == (1 if when == "after" else 0)


def test_known_path_checker_identity_reuses_v2_proof_without_search():
    from tools.tesseract_ompl_v3_identity import verify_checkers
    from pathlib import Path
    proof = verify_checkers(Path(__file__).resolve().parents[1])
    assert proof["previous_path_points"] == 29 and proof["previous_same_q_mismatches"] == 0
    assert not proof["new_full_audit_executed"] and not proof["historical_path_injected"]


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


@pytest.mark.parametrize("key,value", [("lever_arm_m", 2.), ("lever_arm_m", 8.),
    ("point_motion_bound_m", .0025), ("point_motion_bound_m", .000625),
    ("lever_arm_m", 0.), ("lever_arm_m", -4.), ("lever_arm_m", float("nan")),
    ("point_motion_bound_m", float("inf")), ("edge_resolution_rad", .11),
    ("edge_resolution_rad", 0.), ("lever_arm_m", True)])
def test_subdivision_changes_and_invalid_values_explicitly_rejected(key, value):
    r, s = inputs(); w = FakeWorker()
    r = replace(r, constraints={**r.constraints, key: value})
    result = TesseractOMPLBackend(worker=w).plan(r, s, lambda p: pytest.fail("unsupported grid reached authority"))
    assert result.status == PlanningStatus.UNSUPPORTED_CONSTRAINT and not w.calls


def test_default_and_repair_rule_numeric_contract():
    from unloading_sim.planning_contract import subdivision_rule
    r, s = inputs()
    assert subdivision_rule(r.constraints)["l1_resolution_rad"] == .0003125
    assert subdivision_rule(r.constraints, 1)["l1_resolution_rad"] == .00015625
    assert subdivision_rule({**r.constraints, "edge_resolution_rad": .025})["edge_resolution_rad"] == .025
    with pytest.raises(ValueError): subdivision_rule(r.constraints, 8)


@pytest.mark.parametrize("change,status", [("cancel", PlanningStatus.CANCELLED),
    ("revision", PlanningStatus.STALE_SCENE), ("wall", PlanningStatus.BUDGET_EXHAUSTED)])
def test_returned_candidate_guarded_before_authority(change, status, monkeypatch):
    import unloading_sim.tesseract_ompl_backend as module
    clock=[0.]; cancelled=[False]; revision=["r1"]
    monkeypatch.setattr(module,"perf_counter",lambda:clock[0])
    class Worker(FakeWorker):
        def call(self,data,cancel):
            result=super().call(data,cancel)
            if change == "cancel": cancelled[0]=True
            if change == "revision": revision[0]="r2"
            if change == "wall": clock[0]=2.
            return result
    r,s=inputs()
    r=replace(r,cancelled=lambda:cancelled[0],current_revision=lambda:revision[0],
              budget=PlanningBudget(wall_time_s=1.))
    result=TesseractOMPLBackend(worker=Worker()).plan(r,s,lambda p:pytest.fail("obsolete candidate reached authority"))
    assert result.status == status and not result.path
