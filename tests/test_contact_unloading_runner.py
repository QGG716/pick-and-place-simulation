from __future__ import annotations

import json
import os
from types import SimpleNamespace

import pytest

from tools import run_m710_contact_unloading as runner


def setup_planner(monkeypatch, tmp_path, *, ready=True):
    calls = {}
    result = {"complete_trajectory_status": "PASS", "statistics": {"task_count": 1},
              "planning_performance": {"planning_total_wall_seconds": .1}}
    def plan(policy, **kwargs):
        calls["plan"] = (policy, kwargs)
        kwargs["progress_callback"]({"target": "opaque-box", "trajectory_success": True})
        return result
    def preflight(config, **kwargs):
        calls["preflight"] = (config, kwargs)
        return {"status": "READY" if ready else "BLOCKED", "simulation_execution_ready": ready,
                "blockers": [] if ready else ["INVALID_CURRENT_STATE"]}
    def write(value, path):
        path.write_text(json.dumps(value), encoding="utf-8")
    monkeypatch.setattr(runner, "run_layout_single_carton_audit", plan)
    monkeypatch.setattr(runner, "load_layout_motion_policy", lambda path: SimpleNamespace(
        data={"search_strategy": {"row_height_fraction": .08}, "suction": {"mode": "ideal_independent_cups"}}))
    monkeypatch.setattr(runner, "write_layout_single_carton_audit", write)
    monkeypatch.setattr("unloading_sim.m710_execution.build_m710_execution_preflight", preflight)
    monkeypatch.setattr("unloading_sim.m710_execution.write_m710_execution_preflight", write)
    def export(command, **kwargs):
        calls.update(export=(command, kwargs))
        from pathlib import Path
        Path(command[-1]).write_text("{}")
    monkeypatch.setattr(runner.subprocess, "run", export)
    monkeypatch.setattr("unloading_sim.m710_replay_contract.verify_m710_replay_bundle",
                        lambda *a, **k: {"status": "TEST_WIRING_ONLY"})
    return calls, result


@pytest.mark.parametrize("pythonpath_case", ["missing", "other", "present", "relative_present"])
def test_actual_state_is_passed_through_to_planning_and_preflight(monkeypatch, tmp_path, pythonpath_case):
    calls, _ = setup_planner(monkeypatch, tmp_path)
    other_root = str(tmp_path / "other_imports")
    source_root = str(runner.ROOT / "src")
    if pythonpath_case == "missing":
        monkeypatch.delenv("PYTHONPATH", raising=False)
    else:
        existing = {"other": other_root, "present": other_root + os.pathsep + source_root,
                    "relative_present": other_root + os.pathsep + "src"}[pythonpath_case]
        monkeypatch.setenv("PYTHONPATH", existing)
    monkeypatch.setenv("M710_RUNNER_ENV_TEST_SENTINEL", "preserve-this-value")
    parent_env = os.environ.copy()
    actual = {"q_rad": [.1]*6, "cartons": [], "completed_carton_ids": [], "handed_off_ids": []}
    source = tmp_path / "actual.json"
    source.write_text(json.dumps(actual), encoding="utf-8")
    initial = SimpleNamespace(cartons=(), support_graph=None)
    actual_scene = SimpleNamespace(policy=SimpleNamespace(data={"search_strategy": {}, "suction": {"mode": "ideal_independent_cups"}}),
                                   snapshot={"scene_fingerprint": "actual-snapshot"})
    monkeypatch.setattr(runner, "build_verified_motion_input", lambda policy: initial)
    def apply(scene, state, **kwargs):
        assert scene is initial
        assert state == actual
        calls["row_state"] = kwargs["row_state"]
        return actual_scene
    monkeypatch.setattr(runner, "apply_actual_motion_state", apply)
    bundle = tmp_path / "next_bundle.json"
    assert runner.main(["--output", str(tmp_path / "run"), "--actual-state", str(source),
                        "--execution-bundle", str(bundle)]) == 0
    assert calls["plan"][0] is actual_scene.policy
    assert calls["plan"][1]["motion_input"] is actual_scene
    assert calls["plan"][1]["row_state"] is calls["row_state"]
    assert calls["row_state"].config.row_height_fraction == .08
    assert calls["preflight"][1]["motion_input"] is actual_scene
    command, options = calls["export"]
    assert command[1].endswith("scripts/export_isaac_fanuc_replay.py") or command[1].endswith("scripts\\export_isaac_fanuc_replay.py")
    assert command[-1] == str(bundle.resolve())
    assert options["check"] is True
    assert options["env"] is not os.environ
    assert options["env"]["M710_RUNNER_ENV_TEST_SENTINEL"] == "preserve-this-value"
    expected_path = parent_env.get("PYTHONPATH", "")
    if pythonpath_case in {"missing", "other"}:
        expected_path = source_root + (os.pathsep + expected_path if expected_path else "")
    assert options["env"]["PYTHONPATH"] == expected_path
    assert os.environ == parent_env
    assert json.loads((tmp_path / "run" / "actual_scene_snapshot.json").read_text())["scene_fingerprint"] == "actual-snapshot"


def test_blocked_preflight_does_not_export_and_geometry_failure_has_nonzero_status(monkeypatch, tmp_path):
    calls, result = setup_planner(monkeypatch, tmp_path, ready=False)
    assert runner.main(["--output", str(tmp_path), "--execution-bundle", str(tmp_path / "bundle.json")]) == 3
    assert "export" not in calls
    result["complete_trajectory_status"] = "FAIL_CLOSED"
    calls.clear()
    assert runner.main(["--output", str(tmp_path / "failed")]) == 2
    assert "preflight" not in calls


def test_missing_or_malformed_actual_state_cannot_fall_back_to_initial_scene(monkeypatch, tmp_path):
    calls, _ = setup_planner(monkeypatch, tmp_path)
    with pytest.raises(FileNotFoundError):
        runner.main(["--output", str(tmp_path), "--actual-state", str(tmp_path / "missing.json")])
    source = tmp_path / "bad.json"
    source.write_text("[]", encoding="utf-8")
    with pytest.raises(ValueError, match="one actual-state JSON"):
        runner.main(["--output", str(tmp_path), "--actual-state", str(source)])
    assert "plan" not in calls


@pytest.mark.parametrize("ready", [False, True])
def test_curobo_cli_installs_actual_scene_adapter_and_closes_it(monkeypatch,tmp_path,ready):
    # Wiring test only. Real complete-task evidence is produced by the CLI on the server.
    calls,_=setup_planner(monkeypatch,tmp_path,ready=ready)
    robot=object(); connector=object()
    policy=SimpleNamespace(data={"search_strategy":{},"suction":{"mode":"ideal_independent_cups"}},
        layout_validation=SimpleNamespace(layout=SimpleNamespace(robot=lambda:robot)))
    scene=SimpleNamespace(snapshot={"scene_fingerprint":"fresh"})
    monkeypatch.setattr(runner,'load_layout_motion_policy',lambda _:policy)
    monkeypatch.setattr(runner,'build_verified_motion_input',lambda _:scene)
    monkeypatch.setattr('unloading_sim.layout_single_carton._build_automatic_trajectory_connector',
        lambda s,r:SimpleNamespace(connector=connector))
    def install(s,c,python,output,**kwargs):
        assert s is scene and c is connector
        calls['install']=kwargs
        return SimpleNamespace(close=lambda:calls.update(closed=True))
    monkeypatch.setattr('unloading_sim.curobo_transit.install_curobo_transit',install)
    def handoff(s,c,result,bundle):
        assert s is scene and c is connector
        calls['handoff']=True
        return dict(accepted=False,failure={'reason':'TEST_CHANGED_EXECUTION_GEOMETRY'})
    monkeypatch.setattr('unloading_sim.curobo_execution.validate_execution_handoff',handoff)
    assert runner.main(['--output',str(tmp_path),'--transit-backend','curobo_v2','--gpu-python','gpu-python'])==(4 if ready else 3)
    assert calls['plan'][1]['trajectory_connector'] is connector
    assert calls['plan'][1]['trajectory_connector_build'].connector is connector
    assert calls['preflight'][1]['motion_input'] is scene
    assert calls['install']['geometry_fit_seed']==716 and calls['closed']
    assert ('handoff' in calls)==ready
    assert ('export' in calls)==ready
    assert not (tmp_path/'first_feasible').exists()


def test_curobo_cli_requires_explicit_gpu_python(tmp_path):
    with pytest.raises(SystemExit):
        runner.main(['--output',str(tmp_path),'--transit-backend','curobo_v2'])


def test_injected_connector_cannot_borrow_other_model_build_evidence():
    from unloading_sim.layout_single_carton import run_layout_single_carton_audit
    with pytest.raises(ValueError, match="must belong"):
        run_layout_single_carton_audit(None, trajectory_connector=object(),
            trajectory_connector_build=SimpleNamespace(connector=object()))


def test_continuation_entry_selects_its_existing_paired_execution_policy(monkeypatch,tmp_path):
    calls,_=setup_planner(monkeypatch,tmp_path,ready=False)
    policy=SimpleNamespace(data=dict(search_strategy=dict(profile='proof_of_concept',
        post_landing_transport=dict(mode='ideal_outfeed',reception_mode='ideal')),
        suction=dict(mode='ideal_independent_cups')))
    monkeypatch.setattr(runner,'load_layout_motion_policy',lambda _:policy)
    assert runner.main(['--output',str(tmp_path),'--config',str(runner.ROOT/
        'configs/validation/m710id70_handoff_continuation.yaml')])==3
    assert calls['preflight'][0]==runner.ROOT/'configs/simulation/m710id70_handoff_continuation.yaml'


def test_preflight_exception_is_recorded_and_cannot_export(monkeypatch,tmp_path):
    calls,_=setup_planner(monkeypatch,tmp_path)
    def reject(*a,**k):raise ValueError('actual motion input policy mismatch')
    monkeypatch.setattr('unloading_sim.m710_execution.build_m710_execution_preflight',reject)
    assert runner.main(['--output',str(tmp_path)])==3
    assert 'export' not in calls
    delivery=json.loads((tmp_path/'delivery.json').read_text())
    assert delivery['first_failure']==dict(stage='preflight',reason='ValueError',
        detail='actual motion input policy mismatch')
    assert not delivery['isaac_executed']
