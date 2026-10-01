"""Strict source gates operate before numerical or simulator initialization."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

from unloading_sim.m710_replay_contract import (
    M710ReplayContractError, _validate_native_mode_binding, canonical_sha256,
    validate_trajectory_segment,
)


def _unproven_segment():
    return dict(
        target="carton_l07_c02", path=[[0.0] * 6, [0.01] * 6],
        grasp_index=0, release_index=1, release_retreat_index=1,
        native_cold=True, require_native_motion=True,
    )


def test_strict_segment_cannot_discard_entire_native_backend():
    with pytest.raises(M710ReplayContractError, match="NATIVE_COLD_SOURCE_REJECTED"):
        validate_trajectory_segment(_unproven_segment())


def test_integrated_segment_gate_recomputes_coverage_from_returned_points():
    # Synthetic source bindings exercise the artifact gate only; this fixture
    # carries no claim of a native worker invocation or robot feasibility.
    segment = _unproven_segment()
    path = segment["path"]
    request = dict(task_id="synthetic-task", stage_id="synthetic-stage", q_start=path[0], q_goal=path[-1])
    record = dict(
        stage_id="synthetic-stage", request_id="synthetic-request", task_id="synthetic-task",
        parent_stage_id="", native_solver_calls=dict(PTP=1), submitted_request=request,
        request_fingerprint=canonical_sha256(request),
        solver="pilz/PTP", input_state_sha256=canonical_sha256(path[0]),
        constraints_sha256=canonical_sha256({k: v for k, v in request.items() if k != "q_start"}),
        authoritative_status="PASS",
        mtc_generation=True, path_sha256=canonical_sha256(path),
        path_range=[0, 1], points=[dict(q=q.copy(), t=float(i)) for i, q in enumerate(path)],
    )
    segment["native_backend"] = dict(
        native_cold=True, require_native_motion=True, task_id="synthetic-task", stages=[record],
        cold_audit=dict(history_enabled=False, history_inputs_read=0, legacy_motion_generator_calls=0),
        mtc_task_audit=dict(status="SUCCESS", task_id="synthetic-task", generated_during_task=True,
            complete_task=True, stage_ids=["synthetic-stage"], stage_sources=[dict(
                stage_id="synthetic-stage", parent_stage_id="", request_id="synthetic-request",
                native_solver_calls=dict(PTP=1))]),
    )
    assert validate_trajectory_segment(segment) == segment
    segment["path"][1][0] += 0.01
    with pytest.raises(M710ReplayContractError, match="NATIVE_COLD_MOTION_SOURCE_GAP"):
        validate_trajectory_segment(segment)


@pytest.mark.parametrize("removed_from", ["plan", "segment"])
def test_request_mode_cannot_be_removed_on_either_side_of_export(removed_from):
    plan = dict(native_cold=True, require_native_motion=True)
    segment = _unproven_segment()
    (plan if removed_from == "plan" else segment).pop("native_cold")
    with pytest.raises(M710ReplayContractError, match="NATIVE_COLD_MODE_BINDING_MISMATCH"):
        _validate_native_mode_binding(plan, segment)


def test_standalone_strict_rejection_does_not_import_numerical_or_simulator_runtime():
    source = Path(__file__).resolve().parents[1] / "src/unloading_sim/m710_replay_contract.py"
    code = """
import importlib.util, json, sys
spec = importlib.util.spec_from_file_location('gate', sys.argv[1])
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)
try:
    gate.validate_trajectory_segment(json.loads(sys.argv[2]))
except ValueError as exc:
    print(str(exc))
else:
    raise RuntimeError('strict unproven motion was accepted')
print('numpy' in sys.modules, 'isaacsim' in sys.modules)
"""
    result = subprocess.run([sys.executable, "-c", code, str(source), json.dumps(_unproven_segment())],
                            capture_output=True, text=True, check=True)
    assert "NATIVE_COLD_SOURCE_REJECTED" in result.stdout
    assert result.stdout.splitlines()[-1] == "False False"
