"""Small contract and real-native tests; no frozen engineering request here."""
from dataclasses import replace
import json
import os
from pathlib import Path

import pytest

from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig, EndpointSamplingConfig
from unloading_sim.tesseract_ompl_backend import TesseractOMPLBackend, NativeWorker
from unloading_sim.planning_contract import PlanningStatus
from test_tesseract_ompl_contract import inputs, FakeWorker
from tesseract_ompl_unit_scene import unit_scene


def test_default_wire_contract_unchanged_and_optional_sampling_typed():
    default = dict(name="rrt_connect", max_samples=10000, max_roadmap_vertices=10002, max_roadmap_edges=50010)
    assert OMPLPlannerConfig().to_mapping() == default
    config = OMPLPlannerConfig(name="lazy_prm", sampling=EndpointSamplingConfig())
    assert OMPLPlannerConfig(**config.to_mapping()) == config
    assert config.to_mapping()["sampling"] == dict(type="endpoint_mixture", local_probability=.5,
                                                  joint_span_half_width_fraction=.05)


@pytest.mark.parametrize("sampling", [dict(type="unknown"), dict(local_probability=0),
    dict(local_probability=1), dict(local_probability=True), dict(local_probability=float("nan")),
    dict(joint_span_half_width_fraction=0), dict(joint_span_half_width_fraction=.25),
    dict(joint_span_half_width_fraction=float("inf")), dict(unknown=1), "unknown"])
def test_bad_sampling_configuration_rejected(sampling):
    with pytest.raises(ValueError): OMPLPlannerConfig(name="lazy_prm", sampling=sampling)


def test_sampling_cannot_silently_apply_to_rrt():
    with pytest.raises(ValueError): OMPLPlannerConfig(sampling=EndpointSamplingConfig())


@pytest.mark.parametrize("acknowledge", [True, False])
@pytest.mark.parametrize("exact", [True, False])
def test_sampling_transport_still_requires_effective_ack_exact_and_authority(acknowledge, exact):
    class Worker(FakeWorker):
        def call(self, message, cancelled):
            raw = super().call(message, cancelled)
            raw["exact_solution"] = exact
            if acknowledge:
                raw["effective_sampler"] = {"configuration": message["planner_config"]["sampling"]}
            return raw
    r, scene = inputs(); worker = Worker(); authority = []
    backend = TesseractOMPLBackend(worker=worker, stop_on_native_block=True, roadmap_diagnostics=True,
        planner_config=OMPLPlannerConfig(name="lazy_prm", sampling=EndpointSamplingConfig()))
    result = backend.plan(r, scene, lambda path: authority.append(path))
    assert result.deliverable == (acknowledge and exact)
    assert len(authority) == int(acknowledge and exact)
    assert worker.calls[0]["q_start"] == r.q_start and worker.calls[0]["q_goal"] == r.q_goal
    assert worker.calls[0]["roadmap_diagnostics"] is True
    assert len(worker.calls) == 1


@pytest.fixture(scope="module")
def real_worker():
    executable = os.environ.get("UNLOADING_TESSERACT_WORKER")
    if not executable: pytest.skip("real native worker unavailable; not native evidence")
    worker = NativeWorker(executable); records = []
    def call(**changes):
        message = dict(scene=unit_scene(), q_start=[-.6, 0], q_goal=[.6, 0], seed=71081,
            max_state_checks=100000, planner_config=OMPLPlannerConfig(name="lazy_prm").to_mapping(),
            roadmap_diagnostics=True)
        message.update(changes)
        result = worker.call(message); records.append(dict(request=message, result=result))
        return result
    yield call
    if os.environ.get("UNLOADING_V5_TEST_EVIDENCE"):
        Path(os.environ["UNLOADING_V5_TEST_EVIDENCE"]).write_text(json.dumps(records, indent=2))
    worker.close()


def test_real_readonly_diagnostics_preserve_search_and_graph_labels(real_worker):
    plain = real_worker(roadmap_diagnostics=False)
    diagnostic = real_worker()
    assert plain["counters"] == diagnostic["counters"]
    assert plain["path"] == diagnostic["path"]
    assert plain["search_progress"] == diagnostic["search_progress"]
    d = diagnostic["roadmap_diagnostics"]
    assert d["connectivity"]["component_label_audit"]["matches_independent_traversal"]
    assert d["connectivity"]["verified_subgraph"]["connected"] == diagnostic["exact_solution"]
    assert diagnostic["rejected_connections"]["first_16"]


def test_real_mixture_accounting_interruption_and_fresh_roadmap(real_worker):
    config = OMPLPlannerConfig(name="lazy_prm", sampling=EndpointSamplingConfig()).to_mapping()
    first = real_worker(planner_config=config, max_state_checks=30)
    second = real_worker(planner_config=config, max_state_checks=30)
    for raw in (first, second):
        assert raw["status"] == "BUDGET_EXHAUSTED" and not raw["path"]
        assert raw["effective_sampler"]["configuration"] == config["sampling"]
        c = raw["sampling_counts"]
        assert c["global"] + c["start_neighborhood"] + c["goal_neighborhood"] + c["other"] == c["roadmap_total"]
        assert c["setup_projection"] + c["roadmap_total"] == raw["counters"]["cumulative_samples"]
        assert raw["initial_roadmap_vertices"] == raw["initial_roadmap_edges"] == 0
        assert raw["counters"]["actual_state_computations"] <= 30
        assert raw["counters"]["incomplete_edges"] > 0
        interrupted = [e for e in raw["rejected_connections"]["first_16"] if e["classification"] == "C_INCOMPLETE"]
        assert interrupted and all(e["failure"] is None for e in interrupted)
    assert first["counters"] == second["counters"]


@pytest.mark.parametrize("sampling", [dict(type=42, local_probability=.5, joint_span_half_width_fraction=.05),
    dict(type="endpoint_mixture", local_probability=True, joint_span_half_width_fraction=.05),
    dict(type="endpoint_mixture", local_probability=1., joint_span_half_width_fraction=.05)])
def test_real_invalid_sampling_rejected(real_worker, sampling):
    config = OMPLPlannerConfig(name="lazy_prm").to_mapping(); config["sampling"] = sampling
    assert real_worker(planner_config=config)["status"] == "UNSUPPORTED_CONSTRAINT"
