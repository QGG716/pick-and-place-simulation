"""Opt-in small real-worker regression through the ordinary c00 entry."""
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest


def test_production_ik_pool_score_and_native_goal_request(monkeypatch, tmp_path):
    archive = os.environ.get("M710_C00_PREDECESSOR_ARCHIVE")
    worker = os.environ.get("M710_C00_WORKER_COMMAND")
    binary = os.environ.get("M710_C00_WORKER_BINARY")
    if not all((archive, worker, binary)):
        pytest.skip("requires the existing isolated production worker and predecessor archive")
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root / "tools"))
    import plan_m710_junction_single as entry
    from unloading_sim.coupled_batch import CoupledCycleConnector
    from unloading_sim.upper_arm_preference import numerical_seed, upper_arm

    class ProbeComplete(RuntimeError):
        pass

    observed = {}
    original = CoupledCycleConnector._native_plan

    def probe(self, start, goal, obstacles, **kwargs):
        # Observe the real production call, without substituting FK, IK, native
        # responses, collision results, candidate generation or the RobotModel.
        assert type(self.robot).__name__ == "PinocchioHppFclBackend"
        assert kwargs["stage"] == "pregrasp"
        row = self.connection_attempts[-1]
        np.testing.assert_array_equal(row["q_goal"], goal)
        arm = upper_arm(self.robot, goal)
        assert arm["length_m"] == pytest.approx(.895, abs=1e-10)
        assert arm["q2_rad"] == pytest.approx(goal[1])
        assert arm["elevation_rad"] == row["upper_arm_preference"]["elevation_rad"]
        assert row["valid_configuration_retained"]
        assert 0 <= row["upper_arm_preference"]["cost"] <= 1
        assert "mechanism_surface_gap" in row["upper_arm_preference"]
        for index in range(3):
            seed = numerical_seed(self.robot, start, 71070, index)
            assert self.robot.within_limits(seed)
            np.testing.assert_array_equal(seed, numerical_seed(self.robot, start, 71070, index))
        before = len(self.native.records)
        path, failure, evidence = original(self, start, goal, obstacles, **kwargs)
        records = self.native.records[before:]
        motions = [r for r in records if r["op"] == "plan"]
        assert motions, "the qualified numerical IK must reach the full native request"
        assert all(r["pipeline_id"] != "external_axis" for r in motions)
        assert all(r["q_start"] == np.asarray(start).tolist() for r in motions)
        assert all(r["parent_stage_id"] for r in motions)
        iks = [r for r in self.native.records if r["op"] == "ik"]
        assert any(r["status"] == "SUCCESS" for r in iks)
        np.testing.assert_array_equal(iks[0]["response"]["real_start_q_unchanged"], start)
        np.testing.assert_array_equal(iks[0]["response"]["numerical_ik_seed"], start)
        assert all(r["response"].get("request_check_evidence") for r in motions)
        observed.update(backend=type(self.robot).__name__, arm=arm,
            selected_candidate=row, ik_requests=iks, motion_requests=motions,
            native_failure=failure, native_path_returned=bool(path),
            purpose="INTEGRATION_PROBE_ONLY_NOT_A_COMPLETE_C00_ATTEMPT")
        raise ProbeComplete("FIRST_REAL_NATIVE_GOAL_MOTION_REQUEST_RETURNED")

    monkeypatch.setattr(CoupledCycleConnector, "_native_plan", probe)
    output = Path(os.environ.get("M710_C00_CHAIN_OUTPUT", str(tmp_path / "production-chain")))
    monkeypatch.setattr(sys, "argv", [str(root / "tools/plan_m710_junction_single.py"),
        "--archive", archive, "--output", str(output), "--worker-command", worker,
        "--worker-binary", binary, "--source-commit", os.environ.get("M710_C00_TEST_SOURCE", "working-tree"),
        "--development", "--transverse-scissor-026", "--right-front-eccentric"])
    with pytest.raises(ProbeComplete, match="FIRST_REAL_NATIVE"):
        entry.main()
    assert observed
    (output / "integration-observation.json").write_text(json.dumps(observed, indent=2))
    result = json.loads((output / "seed-71070/result.json").read_text())
    assert result["completed_count"] == 0
    assert result["initial"] == result["terminal"]
    assert result["physical_execution"] == "NOT_RUN"
