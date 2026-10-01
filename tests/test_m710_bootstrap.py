"""Initialization is not an execution bundle, and binds its first real plan."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from unloading_sim.m710_bootstrap import (
    _digest, build_bootstrap_contract, measured_initial_state,
    verify_bootstrap_contract, verify_initial_bundle_scope, verify_initial_plan_request,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def contract():
    source = os.environ.get("M710_BOOTSTRAP_TEST_CONTRACT")
    if source:
        return json.loads(Path(source).read_text(encoding="utf-8"))
    return build_bootstrap_contract(ROOT / "configs/simulation/m710id70_proof_of_concept.yaml")


def test_real_bootstrap_is_static_uncommanded_and_not_an_execution_bundle(contract):
    result = verify_bootstrap_contract(contract, project_root=ROOT)
    assert result["status"] == "INITIALIZATION_ONLY"
    assert "format" not in contract and "positions_rad" not in contract
    metadata = contract["metadata"]
    assert metadata["simulation_execution_ready"] is False
    assert metadata["gripper"]["commanded_active_mask"] == [False] * 72
    assert "m710_execution_preflight" not in metadata
    assert sum(p.get("dynamic") is True for p in metadata["scene_primitives"]) == 40


@pytest.mark.parametrize("mutation, reason", [
    ("ready", "BOOTSTRAP_INVALID_SCOPE"),
    ("cup", "BOOTSTRAP_CUPS_MUST_BE_UNCOMMANDED"),
    ("path", "BOOTSTRAP_FORBIDDEN_MOTION_INPUT"),
    ("missing_carton", "BOOTSTRAP_REQUIRES_40"),
])
def test_rehashed_bootstrap_cannot_authorize_motion_or_hide_bodies(contract, mutation, reason):
    changed = deepcopy(contract)
    if mutation == "ready":
        changed["metadata"]["simulation_execution_ready"] = True
    elif mutation == "cup":
        changed["metadata"]["gripper"]["commanded_active_mask"][0] = True
    elif mutation == "path":
        changed["path"] = [[0.] * 6]
    else:
        records = changed["metadata"]["scene_primitives"]
        records.remove(next(p for p in records if p.get("dynamic")))
    changed.pop("contract_sha256")
    changed["contract_sha256"] = _digest(changed)
    with pytest.raises(ValueError, match=reason):
        verify_bootstrap_contract(changed)


def test_first_request_requires_both_live_world_and_actual_state_identity():
    with pytest.raises(ValueError, match="MISSING_REQUEST_IDENTITY"):
        verify_initial_plan_request(dict(bundle_path="bundle.json"),
                                    world_session_id="new-world", actual_state_sha256="measured")
    verify_initial_plan_request(dict(stop=True, world_session_id="new-world", actual_state_sha256="measured"),
                                world_session_id="new-world", actual_state_sha256="measured")


def test_first_bundle_must_bind_measured_state_and_keep_selected_target():
    state = dict(world_session_id="new-world", observed="current physical tensor")
    bundle = dict(format="isaacsim_fanuc_replay_v1", metadata=dict(
        target="carton_l07_c02", native_cold=True, require_native_motion=True,
        initial_actual_state_context=dict(world_session_id="new-world", actual_state_fingerprint=_digest(state))))
    verify_initial_bundle_scope(bundle, state)
    bundle["metadata"]["target"] = "carton_l07_c03"
    with pytest.raises(ValueError, match="NATIVE_COLD_TARGET_BUNDLE"):
        verify_initial_bundle_scope(bundle, state)


def test_initial_snapshot_keeps_raw_velocity_and_rest_samples_without_completion():
    names = [f"carton_{index}" for index in range(40)]
    metadata = dict(stack_carton_names=names, joint_names=[f"J{index}" for index in range(1, 7)],
        simulation_profile=dict(name="proof_of_concept"), post_landing_transport=dict(mode="ideal_outfeed"))
    cartons = [dict(name=name, center_m=[index * .6, 0., .15], quaternion_wxyz=[1., 0., 0., 0.],
        linear_velocity_m_s=[0., 0., .0001], angular_velocity_rad_s=[0., 0., 0.])
        for index, name in enumerate(names)]
    raw_qd = [.0002, -.0003, 0., 0., 0., 0.]
    proof = dict(schema="m710_native_rest_start_evidence_v1", samples=[dict(qd_rad_s=raw_qd)])
    state = measured_initial_state(metadata, q_rad=[0.] * 6, qd_rad_s=raw_qd,
        joint_names=metadata["joint_names"], cartons=cartons, world_session_id="retained-world", time_s=3.,
        native_rest_start_evidence=proof)
    assert state["qd_rad_s"] == raw_qd
    assert state["native_rest_start_evidence"] == proof
    assert state["initialization_provenance"]["historical_motion_inputs_read"] == 0
    assert not state["completed_carton_ids"] and not state["processed_carton_ids"]
    assert not state["attached"] and state["attachment_target"] is None
    assert state["cartons"][0]["linear_velocity_m_s"] == [0., 0., .0001]
    proof["samples"].clear()
    assert state["native_rest_start_evidence"]["samples"]


def test_bootstrap_import_is_standard_library_only():
    code = """
import importlib.util, sys
spec = importlib.util.spec_from_file_location('bootstrap', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
print('numpy' in sys.modules, 'isaacsim' in sys.modules)
"""
    result = subprocess.run([sys.executable, "-c", code, str(ROOT / "src/unloading_sim/m710_bootstrap.py")],
                            check=True, text=True, capture_output=True)
    assert result.stdout.strip() == "False False"
