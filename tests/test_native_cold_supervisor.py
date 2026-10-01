"""Supervisor failures never authorize a retry or stand in for physics success.

Subprocesses below are controlled fakes. These tests start no simulator or planner.
"""
import ast
import hashlib
import json
import os
from pathlib import Path
import re
from types import SimpleNamespace

import pytest

from tools import run_m710_native_cold_once as supervisor


def _json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


@pytest.fixture
def controlled_run(tmp_path, monkeypatch):
    root = tmp_path / "isolated"
    root.mkdir()
    output = root / "run"
    binary = root / "worker"
    binary.write_bytes(b"test worker; never executed")
    calls = []
    scenario = dict(planning_returncode=2, result=None, isaac_exit_code=0)

    class World:
        pid = 1234
        returncode = None
        terminated = False

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            calls.append("world_wait")
            request_path = output / "handshake/initial_request.json"
            if self.terminated:
                self.returncode = -15
            elif request_path.exists() and json.loads(request_path.read_text()).get("stop") is True:
                self.returncode = 1
            else:
                if scenario["result"] is not None:
                    _json(output / "physics/result.json", scenario["result"])
                self.returncode = scenario["isaac_exit_code"]
            return self.returncode

        def terminate(self):
            calls.append("world_terminate")
            self.terminated = True

        def kill(self):
            calls.append("world_kill")
            self.terminated = True

    world = World()

    def popen(command, **kwargs):
        calls.append("world_launch")
        assert command[command.index("--maximum-segments") + 1] == "1"
        assert "--bootstrap-contract" in command and "--bundle" not in command
        assert not any(arg.startswith("--reuse-usd") for arg in command)
        state = dict(schema="m710id70_actual_motion_state_v1", world_session_id="one-new-world",
            attached=False, cartons=[dict(name=f"carton-{i}") for i in range(40)],
            initialization_provenance=dict(world_scope="NEW_WORLD_NATIVE_COLD"))
        actual = output / "physics/initialized_actual_state.json"
        _json(actual, state)
        ready = dict(status="INITIAL_WORLD_RETAINED_AWAITING_NATIVE_PLAN", completed_segments=0,
            target="carton_l07_c02", world_session_id=state["world_session_id"],
            actual_state_path=str(actual), actual_state_sha256=hashlib.sha256(actual.read_bytes()).hexdigest(),
            request_path=str(output / "handshake/initial_request.json"), bootstrap_contract_sha256="a" * 64,
            physics_time_paused_for_offline_planning=True, no_reset_no_body_replacement=True,
            motion_execution_permitted=False, attachment_permitted=False)
        _json(output / "handshake/initial_ready.json", ready)
        return world

    def run(command, **kwargs):
        if Path(command[1]).name == "prepare_m710_native_bootstrap.py":
            calls.append("prepare")
            _json(output / "bootstrap.json", {})
            return SimpleNamespace(returncode=0)
        assert Path(command[1]).name == "run_m710_contact_unloading.py"
        calls.append("plan")
        assert "--native-cold" in command and "--actual-state" in command and "--initial-ready" in command
        assert not any(arg in command for arg in ("--history-source", "--reuse-motion", "--fixed-history-fixture"))
        _json(output / "plan/motion.json", dict(complete_trajectory_status="PASS", native_cold=True,
            require_native_motion=True))
        _json(output / "plan/delivery.json", dict(simulation_execution_ready=True,
            bundle_readback=dict(status="PASS"), bundle_path=str(output / "plan/bundle.json")))
        _json(output / "plan/bundle.json", {})
        return SimpleNamespace(returncode=scenario["planning_returncode"])

    monkeypatch.setattr(supervisor, "ROOT", root)
    monkeypatch.setattr(supervisor, "source_hashes", lambda: {"current.py": "sha"})
    monkeypatch.setattr(supervisor.subprocess, "Popen", popen)
    monkeypatch.setattr(supervisor.subprocess, "run", run)
    monkeypatch.setattr(supervisor.sys, "argv", ["supervisor", "--output", str(output),
        "--isaac-python", str(root / "isaac-python"), "--worker-command", "not-executed",
        "--worker-binary", str(binary), "--native-asset-root", str(root), "--source-commit", "a" * 40])
    return SimpleNamespace(output=output, calls=calls, world=world, scenario=scenario)


def test_planning_failure_stops_exactly_one_world_without_motion_or_retry(controlled_run):
    case = controlled_run
    assert supervisor.main() == 1
    request = json.loads((case.output / "handshake/initial_request.json").read_text())
    ready = json.loads((case.output / "handshake/initial_ready.json").read_text())
    assert request["stop"] is True and "bundle_path" not in request
    assert request["world_session_id"] == ready["world_session_id"]
    assert request["actual_state_sha256"] == ready["actual_state_sha256"]
    status = json.loads((case.output / "run.json").read_text())
    assert (status["world_launches"], status["planning_requests"], status["execution_requests"]) == (1, 1, 0)
    assert case.calls.count("world_launch") == case.calls.count("plan") == 1
    assert case.world.poll() is not None


def _workflow_result():
    return dict(workflow_cycle_completed=True, runtime_stop_reason=None, qualification_passed=False,
        execution_counts=dict(actual_grasp=1, actual_release=1),
        native_cold_initialization=dict(same_world_retained_for_planning_and_execution=True,
            maximum_physical_segments=1, initial_completed_segments=0, historical_motion_inputs_read=0))


def test_one_gated_request_uses_declared_workflow_result_not_physical_qualification(controlled_run):
    case = controlled_run
    case.scenario.update(planning_returncode=0, result=_workflow_result())
    assert supervisor.main() == 0
    request = json.loads((case.output / "handshake/initial_request.json").read_text())
    assert "bundle_path" in request and not request.get("stop")
    status = json.loads((case.output / "run.json").read_text())
    assert (status["world_launches"], status["planning_requests"], status["execution_requests"]) == (1, 1, 1)
    assert status["status"] == "ISAAC_WORKFLOW_COMPLETED_UNDER_DECLARED_ASSUMPTIONS"
    assert status["physical_qualification_passed"] is False
    assert case.calls.count("world_launch") == case.calls.count("plan") == 1
    assert case.calls.index("world_launch") < case.calls.index("plan") < case.calls.index("world_wait")


@pytest.mark.parametrize("mutation", ["boolean_count", "missing_release", "runtime_stop", "multiple_segments", "old_completion"])
def test_completion_claim_cannot_replace_single_current_execution_observations(mutation):
    value = _workflow_result()
    if mutation == "boolean_count":
        value["execution_counts"]["actual_grasp"] = True
    elif mutation == "missing_release":
        value["execution_counts"].pop("actual_release")
    elif mutation == "runtime_stop":
        value["runtime_stop_reason"] = "TRACKING_LIMIT"
    elif mutation == "multiple_segments":
        value["native_cold_initialization"]["maximum_physical_segments"] = 2
    else:
        value["native_cold_initialization"]["initial_completed_segments"] = 1
    assert supervisor.successful_isaac_result(value) is False


def test_stop_write_failure_still_terminates_and_reaps_world(controlled_run, monkeypatch):
    case = controlled_run
    original = supervisor.write

    def fail_request(path, value):
        if path.name == "initial_request.json":
            raise OSError("simulated unavailable handshake destination")
        return original(path, value)

    monkeypatch.setattr(supervisor, "write", fail_request)
    assert supervisor.main() == 1
    assert case.world.terminated and case.world.poll() is not None
    assert case.calls.count("world_launch") == case.calls.count("plan") == 1


@pytest.mark.parametrize("result", [None, dict(workflow_cycle_completed=False, qualification_passed=False)])
def test_exit_zero_is_not_evidence_of_a_completed_cycle(controlled_run, result):
    case = controlled_run
    case.scenario.update(planning_returncode=0, result=result)
    assert supervisor.main() != 0
    assert case.calls.count("world_launch") == case.calls.count("plan") == 1
    status = json.loads((case.output / "run.json").read_text())
    assert status["execution_requests"] == 1


def test_existing_output_is_rejected_before_any_new_process(controlled_run):
    case = controlled_run
    case.output.mkdir()
    with pytest.raises((FileExistsError, SystemExit)):
        supervisor.main()
    assert case.calls == []


@pytest.mark.parametrize("field", ["request_path", "actual_state_path"])
def test_ready_paths_cannot_escape_the_current_run(tmp_path, field):
    output = tmp_path / "current"
    ready = dict(request_path=str(output / "handshake/initial_request.json"),
        actual_state_path=str(output / "physics/initialized_actual_state.json"))
    ready[field] = str(tmp_path / "other-run" / "artifact.json")
    with pytest.raises(ValueError):
        supervisor.validate_world_ready_paths(output, ready)


def test_unplanned_bootstrap_contact_has_no_declared_receiver_and_never_formats_none():
    source = Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    safe_name = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_safe_prim_name")
    declaration = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "receiver_paths" for target in node.targets))
    namespace = dict(re=re, metadata={})
    exec(compile(ast.Module(body=[safe_name], type_ignores=[]), str(source), "exec"), namespace)
    expression = compile(ast.Expression(body=declaration), str(source), "eval")
    assert eval(expression, namespace) == set()
    namespace["metadata"] = dict(selected_place_support_names=["conveyor_transverse"])
    assert eval(expression, namespace) == {"/Validation/Scene/conveyor_transverse"}


def test_bootstrap_supplies_literal_startup_field_reads_without_loading_isaac():
    """AST field presence complements contracts; it is not a simulated boot."""
    contract_path = os.environ.get("M710_BOOTSTRAP_TEST_CONTRACT")
    if not contract_path:
        pytest.skip("provide the source-frozen GPU bootstrap contract for static startup validation")
    metadata = json.loads(Path(contract_path).read_text(encoding="utf-8"))["metadata"]
    source = Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    boundary = min(node.lineno for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        and node.module == "unloading_sim.m710_bootstrap" and any(item.name == "READY" for item in node.names))
    aliases = dict(metadata=(), gripper_cfg=("gripper",), physics_contract=("physics",),
        rendering_contract=("rendering",), rendering_cfg=("rendering",), conveyor_cfg=("conveyor",),
        camera_cfg=("camera",), required_output=("rendering", "required_output"),
        settling_cfg=("physics", "settling"), tool_mass_accounting=("robot_tool_mass_accounting",))

    def key_path(node):
        if isinstance(node, ast.Name):
            return aliases.get(node.id)
        if isinstance(node, ast.Subscript) and isinstance(node.slice, ast.Constant) and isinstance(node.slice.value, str):
            base = key_path(node.value)
            if base is not None:
                return (*base, node.slice.value)
        return None

    missing = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Subscript) or node.lineno >= boundary:
            continue
        path = key_path(node)
        if not path:
            continue
        value = metadata
        try:
            for key in path:
                value = value[key]
        except (KeyError, TypeError):
            missing.append((node.lineno, ".".join(path)))
    assert not missing, f"bootstrap lacks fields read during initialization: {missing}"
