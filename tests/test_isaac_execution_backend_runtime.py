import argparse
import ast
import copy
import math
from pathlib import Path

import pytest

from unloading_sim.m710_replay_physics import verify_physics_backend_readback

ROOT = Path(__file__).resolve().parents[1]
GPU = {"mode": "physx_gpu", "device": "cuda:0", "broadphase_type": "GPU",
       "gpu_dynamics_enabled": True, "fabric_enabled": True, "ccd_enabled": False}
CPU = {"mode": "physx_cpu", "device": "cpu", "broadphase_type": "MBP",
       "gpu_dynamics_enabled": False, "fabric_enabled": True, "ccd_enabled": True}


@pytest.mark.parametrize("policy", [CPU])
def test_runtime_backend_requires_exact_device_broadphase_and_fabric(policy):
    assert verify_physics_backend_readback(policy, policy)["status"] == "PASS"
    for field in policy:
        actual = copy.deepcopy(policy)
        actual[field] = None
        with pytest.raises(ValueError, match="readback mismatch"):
            verify_physics_backend_readback(policy, actual)


def test_gpu_policy_cannot_be_silently_mixed_with_cpu_broadphase():
    with pytest.raises(ValueError, match="contradictory"):
        verify_physics_backend_readback({**GPU, "broadphase_type": "MBP"}, GPU)
    with pytest.raises(ValueError, match="contradictory"):
        verify_physics_backend_readback(GPU, GPU)
    with pytest.raises(ValueError, match="contradictory"):
        verify_physics_backend_readback({**CPU, "ccd_enabled": False}, CPU)


def test_short_settling_override_is_explicitly_diagnostic_and_never_continues():
    tree = ast.parse((ROOT / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8"))
    nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in {"_parser", "_validate_args"}]
    namespace = {"argparse": argparse, "Path": Path, "math": math, "__doc__": "test"}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "<real runtime CLI>", "exec"), namespace)
    args = ["--bundle", "bundle.json", "--project-root", ".", "--usd-directory", "usd", "--output", "out"]
    parser = namespace["_parser"]()
    namespace["_validate_args"](parser.parse_args(args))
    namespace["_validate_args"](parser.parse_args(args + ["--diagnostic-only", "--diagnostic-settling-steps", "10"]))
    with pytest.raises(ValueError, match="together"):
        namespace["_validate_args"](parser.parse_args(args + ["--diagnostic-settling-steps", "10"]))
    with pytest.raises(ValueError, match="cannot enter task continuation"):
        namespace["_validate_args"](parser.parse_args(args + ["--diagnostic-only", "--diagnostic-settling-steps", "10", "--continuation-dir", "queue"]))
