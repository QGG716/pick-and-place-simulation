"""Real OMPL LazyPRM mechanism tests; unit geometry is not FANUC evidence."""
import json
import os
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.tesseract_ompl_backend import NativeWorker
from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig
from tesseract_ompl_unit_scene import unit_scene


@pytest.fixture(scope="module")
def worker():
    executable = os.environ.get("UNLOADING_TESSERACT_WORKER")
    if not executable:
        pytest.skip("real worker required; a skip is not native evidence")
    native = NativeWorker(executable)
    records = []
    def call(**changes):
        message = dict(scene=unit_scene(), q_start=[-.6, 0], q_goal=[.6, 0],
            seed=71070, max_state_checks=100000, profile=True,
            planner_config=OMPLPlannerConfig(name="lazy_prm").to_mapping())
        message.update(changes)
        result = native.call(message)
        records.append(dict(request=message, result=result))
        return result
    yield call
    output = os.environ.get("UNLOADING_LAZY_TEST_EVIDENCE")
    if output:
        Path(output).write_text(json.dumps(records, indent=2), encoding="utf-8")
    native.close()


def test_actual_default_and_explicit_planners(worker):
    rrt = worker(operation="configure", planner_config=OMPLPlannerConfig().to_mapping())
    assert rrt["effective_planner"]["type"] == "ompl::geometric::RRTConnect"
    assert rrt["effective_planner"]["range_rad"] == .18
    lazy = worker(operation="configure")
    p = lazy["effective_planner"]
    assert p["type"] == "ompl::geometric::LazyPRM" and p["star"] is False
    assert p["max_nearest_neighbors"] == 5
    assert p["max_connection_distance"] == pytest.approx(.2*np.sqrt(8))
    assert p["cost_threshold"] == "positive infinity"
    assert lazy["initial_roadmap_vertices"] == lazy["initial_roadmap_edges"] == 0
    assert not lazy["search_started"] and not lazy["native_validated"]
    assert lazy["versions"]["ompl"] == "1.7.0"
    for config in ({"name": "typo"}, {"name": "lazy_prm", "max_samples": True},
                   {"name": "lazy_prm", "unknown": 1}):
        assert worker(planner_config=config)["status"] == "UNSUPPORTED_CONSTRAINT"
    assert worker(range_rad=.18)["status"] == "UNSUPPORTED_CONSTRAINT"


def test_real_rrt_default_and_explicit_selection_match(worker):
    # Empty configuration exercises protocol defaults. Both must actually search.
    default = worker(planner_config={})
    explicit = worker(planner_config=OMPLPlannerConfig().to_mapping())
    assert default["status"] == explicit["status"] == "CANDIDATE"
    assert default["path"] == explicit["path"]
    assert default["search_started"] and explicit["search_started"]
    assert default["effective_planner"]["range_rad"] == .18


def test_real_invalid_connections_feed_back_then_first_valid_solution_stops(worker):
    r = worker()
    assert r["status"] == "CANDIDATE" and r["exact_solution"] and r["native_validated"]
    assert r["ompl_status"] == "Exact solution" and r["direct_valid"] is False
    assert r["initial_roadmap_vertices"] == r["initial_roadmap_edges"] == 0
    assert r["ompl_invalid_motion_checks"] > 0  # excludes the direct edge
    assert r["counters"]["actual_state_computations"] < 100000
    assert r["counters"]["cumulative_samples"] < 10000
    assert r["counters"]["incomplete_edges"] == 0
    assert r["search_progress"]["unknown_nodes"] > 0
    assert r["search_progress"]["known_valid_edges"] >= len(r["path"])-1
    assert r["first_native_valid_path_s"] is not None
    # Independent analytic box separation over every required grid point.
    for a, b in zip(r["path"][:-1], r["path"][1:]):
        a, b = np.asarray(a), np.asarray(b)
        n = max(1, int(np.ceil(np.abs(b-a).sum()/.0003125)),
                2*int(np.ceil(np.abs(b-a).max()/.055)))
        points = a + np.arange(n+1)[:, None]/n*(b-a)
        gaps = np.maximum(np.abs(points)-[.15, .35], 0)
        assert np.linalg.norm(gaps, axis=1).min() >= .005-1e-9
    # Same worker, new request: same local sampler seed, no roadmap or flags reused.
    repeat = worker(max_state_checks=2)
    assert repeat["status"] == "BUDGET_EXHAUSTED" and not repeat["path"]
    assert repeat["counters"]["actual_state_computations"] == 2
    assert repeat["search_progress"]["roadmap_vertices"] == 0
    assert repeat["counters"]["cache_hits"] == 0


@pytest.mark.parametrize("limits,reason", [
    ({"max_samples": 1}, "CUMULATIVE_SAMPLE_LIMIT"),
    ({"max_roadmap_vertices": 2}, "ROADMAP_VERTEX_LIMIT"),
    ({"max_roadmap_edges": 6}, "ROADMAP_EDGE_RESERVATION_LIMIT"),
])
def test_unknown_roadmap_and_resource_guard_do_not_deliver(worker, limits, reason):
    config = OMPLPlannerConfig(name="lazy_prm", **limits).to_mapping()
    r = worker(planner_config=config)
    assert r["status"] == "BUDGET_EXHAUSTED" and r["termination_detail"] == reason
    assert not r["path"] and not r["native_validated"] and not r["exact_solution"]
    assert r["counters"]["actual_state_computations"] < 100000
    assert r["counters"]["cumulative_samples"] <= config["max_samples"]
    assert r["search_progress"]["roadmap_vertices"] <= config["max_roadmap_vertices"]
    assert r["search_progress"]["roadmap_edges"] <= config["max_roadmap_edges"]
    assert r["roadmap_discarded_after_request"]


def test_incomplete_dense_edge_and_cancellation_fail_closed(worker):
    r = worker(max_state_checks=30)
    assert r["status"] == "BUDGET_EXHAUSTED" and not r["native_validated"] and not r["path"]
    assert r["termination_detail"] == "ACTUAL_STATE_COMPUTATION_LIMIT"
    assert r["counters"]["incomplete_edges"] == 1
    assert r["counters"]["actual_state_computations"] == 30
    native = NativeWorker(os.environ["UNLOADING_TESSERACT_WORKER"])
    try:
        r = native.call(dict(scene=unit_scene(), q_start=[-.6, 0], q_goal=[.6, 0],
            seed=71070, max_state_checks=100000,
            planner_config=OMPLPlannerConfig(name="lazy_prm").to_mapping()), lambda: True)
        assert r["status"] == "CANCELLED" and not r["native_validated"] and not r["path"]
    finally:
        native.close()
