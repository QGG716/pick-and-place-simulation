from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.native_cold_entry import (
    DisabledHistorySource, native_budget_config, validate_native_cold_inputs,
    read_native_initial_state, preserve_native_failure, bind_native_run,
)


@pytest.mark.parametrize("history", [{"source": "absent.json"}, {"register_directory": "old"}])
def test_cold_rejects_configured_history_even_when_not_found(history):
    with pytest.raises(ValueError, match="FORBIDDEN_INPUT"):
        validate_native_cold_inputs({"search_strategy": {"history": history}},
            backend="moveit2", target_id="carton_l07_c02")


def test_cold_cli_rejects_fixture_before_read_or_derivation(tmp_path):
    from tools.run_m710id70_layout_single_carton import main
    output = tmp_path / "failure.json"
    assert main(["--native-cold", "--backend", "moveit2", "--fixed-history-fixture",
                 str(tmp_path / "must_not_read.json"), "--output", str(output)]) == 1
    assert "NATIVE_COLD_FORBIDDEN_HISTORY_FIXTURE" in output.read_text()
    assert not output.with_suffix(".hint.json").exists()


def test_cold_never_enters_history_candidate_loader():
    source = DisabledHistorySource()
    assert source.evidence()["history_inputs_read"] == 0
    with pytest.raises(RuntimeError, match="HISTORY_ENTRY_FORBIDDEN"):
        source.candidates()


@pytest.mark.parametrize("value", [-1, 0, float("nan"), float("inf"), True])
def test_cold_budget_rejects_invalid_limits(value):
    with pytest.raises(ValueError, match="INVALID_BUDGET"):
        native_budget_config({"task_wall_time_s": value})


def test_cold_ipc_cannot_preempt_configured_stage():
    with pytest.raises(ValueError, match="IPC_BUDGET_SHORTER"):
        native_budget_config({"ipc_timeout_s": 12})


def test_outer_contact_ik_uses_native_stream_without_legacy_generator(monkeypatch):
    from unloading_sim import layout_single_carton as entry
    class EmptyNativeStream:
        best_failure = None
        def __iter__(self):
            return iter(())
        def evidence(self):
            return {"native_ik_calls": 1}
    called = []
    def native(*args, **kwargs):
        called.append(kwargs)
        return EmptyNativeStream()
    def forbidden(*args, **kwargs):
        raise AssertionError("legacy motion generator entered")
    monkeypatch.setattr(entry, "iter_ik_solutions", forbidden)
    monkeypatch.setattr(entry, "_independent_cup_selection_at_pose", lambda *a, **k: (object(), object()))
    monkeypatch.setattr(entry, "_compact_coverage", lambda *a, **k: {})
    scene = SimpleNamespace(policy=SimpleNamespace(data={"suction": {}, "ik": {},
        "state_validity": {"contact_tolerance_m": .002}},
        layout_validation=SimpleNamespace(initial_q=np.zeros(6))), all_obstacles=())
    result = entry._audit_pose(scene, object(), "front", 0, np.eye(4), np.eye(4), {},
        71070, True, SimpleNamespace(dof=1), (), native_connector=SimpleNamespace(native_ik_stream=native))
    assert len(called) == 1
    assert called[0]["stage"] == "contact_endpoint"
    assert result["ik_stream"]["native_ik_calls"] == 1


def test_ordinary_api_forbidden_history_fails_before_model_or_history_load(monkeypatch):
    from unloading_sim import layout_single_carton as entry
    def forbidden(*args, **kwargs):
        raise AssertionError("model/history loaded before input guard")
    monkeypatch.setattr(entry, "load_layout_motion_policy", lambda *a: SimpleNamespace(
        data={"search_strategy": {"history": {"source": "old.json"}}}))
    monkeypatch.setattr(entry, "audit_initial_state", forbidden)
    with pytest.raises(ValueError, match="FORBIDDEN_INPUT"):
        entry.run_layout_single_carton_audit("unused", backend="moveit2", native_cold=True,
                                            target_id="carton_l07_c02")


def test_missing_measured_state_cannot_be_substituted_with_configured_home():
    with pytest.raises(ValueError, match="REQUIRES_ACTUAL_STATE"):
        read_native_initial_state(None, None)


def test_failure_keeps_native_records_and_closes_client():
    closed = []
    connector = SimpleNamespace(native_cold_evidence=lambda: {"actual_calls": 3},
        native_evidence=[{"stage": "contact", "status": "FAILED"}],
        native_ik_evidence=[{"native_ik_calls": 1}], authority_path_evidence=[],
        native=SimpleNamespace(close=lambda: closed.append(True)))
    @preserve_native_failure
    def request(*, native_cold):
        bind_native_run(connector)
        raise RuntimeError("ordered-contact-rejection")
    with pytest.raises(RuntimeError) as caught:
        request(native_cold=True)
    assert caught.value.native_cold_failure_evidence["native_stages"][0]["stage"] == "contact"
    assert closed == [True]
