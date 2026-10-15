"""Focused evidence consumer and no direct per-pair clock regressions."""
from pathlib import Path
import importlib.util
import copy
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("native_checks", ROOT / "tools/benchmark_m710_native_checks.py")
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


def example(enabled):
    counts = {k: 1 for k in ("state_checks", "seal_geometry_evaluations", "compression_sat_evaluations",
                             "ordered_checks", "seal_input_compilations", "fixed_seal_verdict_reuses")}
    phase = dict(aabb_tests=3, aabb_skips=1, narrow_calls=2, fixed_narrow_qualifications=1,
                 fixed_qualification_queries=1, fixed_qualification_declines={})
    phase.update({k: 0.0 if enabled else None for k in benchmark.PAIR_TIME_FIELDS})
    return dict(visited_states=1, process=counts, clearance=dict(workspace=dict(
        pair_profile_enabled=enabled, pair_profile_status="MEASURED" if enabled else "NOT_MEASURED",
        prepared_objects=2, prepare_s=.1, phases={"output": phase})))


def test_unmeasured_null_is_distinct_from_measured_zero():
    a, b = example(True), example(False)
    benchmark.check_pair_profile(a, True)
    benchmark.check_pair_profile(b, False)
    assert benchmark.pair_geometry_counts(a) == benchmark.pair_geometry_counts(b)
    bad = copy.deepcopy(b)
    bad["clearance"]["workspace"]["phases"]["output"]["narrow_s"] = 0
    with pytest.raises(RuntimeError, match="UNMEASURED"):
        benchmark.check_pair_profile(bad, False)
    b["clearance"]["workspace"]["phases"]["output"]["narrow_calls"] += 1
    assert benchmark.pair_geometry_counts(a) != benchmark.pair_geometry_counts(b)


def test_pair_loop_uses_only_optional_clock_and_worker_passes_option():
    workspace = (ROOT / "ros2/m710_moveit_backend/src/clearance_workspace.h").read_text()
    loop = workspace.split("for(auto& p:pairs_) {", 1)[1]
    assert "Clock::now" not in loop
    assert "elapsed(broad)" not in loop and "elapsed(narrow)" not in loop
    assert loop.count("PairProfileTimer<>") == 2
    worker = (ROOT / "ros2/m710_moveit_backend/src/worker.cpp").read_text()
    assert 'true,req.value("pair_profile_enabled",false)' in worker
