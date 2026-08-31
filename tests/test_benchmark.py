import pytest

from unloading_sim.benchmark import BenchmarkRecorder


def test_planning_and_simulated_cycle_time_are_never_combined():
    recorder = BenchmarkRecorder(execution_source="pybullet_simulation")
    recorder.record_planning(0.012)
    recorder.record_planning(0.018)
    recorder.record_cycle(7.0)
    recorder.record_cycle(9.0)
    report = recorder.report(planning_deadline_seconds=0.05)

    assert report["planning_compute"]["max_seconds"] == pytest.approx(0.018)
    assert report["execution_cycle"]["mean_seconds"] == pytest.approx(8.0)
    assert report["theoretical_cases_per_hour"] == pytest.approx(450.0)
    assert report["planning_within_deadline"]
    assert report["execution_source"] == "pybullet_simulation"
