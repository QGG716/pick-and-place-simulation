"""Accounting regressions; synthetic durations are not performance evidence."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest

spec = importlib.util.spec_from_file_location("planning_only_timing_entry",
    Path(__file__).resolve().parents[1] / "tools/plan_m710_top_row_only.py")
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def test_online_delivery_includes_bookkeeping_without_adding_solver_subtotals():
    summary = dict(completed_count=5, batch_wall_s=1.5,
        cumulative_attempted_plan_s=1., T_plan_5_s=1., scene_update_s=.2,
        initialization_s=20., native_solver_s=.8, request_round_trip_s=.95)
    result = entry.online_delivery_timing(summary, .1)
    assert result["T_online_5_s"] == pytest.approx(1.6)
    assert result["attempted_online_s"] == pytest.approx(1.6)
    assert result["batch_bookkeeping_s"] == pytest.approx(.3)
    assert result["qualification_status"] == "NOT_EVALUATED"


def test_incomplete_delivery_has_actual_attempted_time_and_no_five_box_claim():
    result = entry.online_delivery_timing(dict(completed_count=2, batch_wall_s=.9,
        cumulative_attempted_plan_s=.4, T_plan_5_s=None, scene_update_s=.2), .05)
    assert result["T_online_5_s"] is None
    assert result["attempted_online_s"] == pytest.approx(.95)
    assert result["batch_bookkeeping_s"] == pytest.approx(.3)


def test_ik_batch_timing_fields_survive_summary_without_duplicate_ipc():
    records = [
        dict(stage="transit", status="FAILED", native_ik_calls=1, ik_s=.25, elapsed_s=1.,
            request_round_trip_s=1., native_solver_s=.25, planning_phase="FAST",
            fast_phase_overrun_s=.2, batch_status="NATIVE_IK_BATCH_TIMEOUT",
            batch_seed_index=0, batch_consume_count=2),
        dict(stage="transit", status="FAILED", native_ik_calls=1, ik_s=.2, elapsed_s=0.,
            request_round_trip_s=0., native_solver_s=.2, planning_phase="FAST",
            fast_phase_overrun_s=0., batch_status="NATIVE_IK_BATCH_TIMEOUT",
            batch_seed_index=1, batch_consume_count=2)]
    c = SimpleNamespace(native_evidence=[], native_ik_evidence=records, skipped_calls={},
        authority_path_evidence=[], native_verified=[], _cold_counters={})
    result = entry.call_summary(c)
    assert result["ik"] == records
    assert result["counts"]["IK"] == 2
    assert result["ik_solver_s"] == pytest.approx(.45)
    assert result["ik_including_ipc_s"] == 1.
