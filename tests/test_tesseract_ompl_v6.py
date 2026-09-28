"""Directed wiring fixtures only: no engineering search or Isaac evidence."""
from copy import deepcopy
from dataclasses import replace
import json
from types import SimpleNamespace

import numpy as np
import pytest

from test_tesseract_ompl_task import args
from test_tesseract_ompl_contract import FakeWorker, inputs
from test_isaac_bridge import _ready_m710_inputs as old_fixture
from tools.run_tesseract_ompl_task import task_backend, planner_configuration, RecordingWorker
from unloading_sim.tesseract_ompl_backend import NativeWorker, NativePlanningBlocked, _record_free_motion
from unloading_sim.tesseract_ompl_config import OMPLPlannerConfig, EndpointSamplingConfig
from unloading_sim import tesseract_task_delivery as delivery


def _ready_m710_inputs(**kw):
    # Extend the older synthetic fixture with the now-required profile binding.
    from unloading_sim.m710_replay_contract import build_replay_input_binding, canonical_sha256
    plan, cfg, preflight = old_fixture(**kw)
    fields = dict(simulation_profile={}, post_landing_transport={"mode": "strict_physics"}, collision_policy={})
    plan.update(fields)
    adapter = preflight["replay_adapter_inputs"]
    adapter["plan_common"].update(deepcopy(fields))
    adapter["input_binding"] = build_replay_input_binding(plan_common=adapter["plan_common"],
        configuration=cfg, scene_primitives=preflight["scene"]["primitives"],
        trajectory_segment=adapter["trajectory_segment"], trajectory_segment_status="VERIFIED",
        input_identity=preflight["input_identity"],
        execution_asset_fingerprint_sha256=preflight["execution_asset_fingerprint_sha256"])
    preflight.pop("preflight_fingerprint")
    preflight["preflight_fingerprint"] = canonical_sha256(preflight)
    return plan, cfg, preflight


def config_file(tmp_path):
    p = tmp_path / "planner.json"
    p.write_text(json.dumps(OMPLPlannerConfig(name="lazy_prm", sampling=EndpointSamplingConfig()).to_mapping()))
    return p


@pytest.mark.parametrize("ack", [True, False])
def test_formal_task_json_reaches_each_request_and_requires_sampler_ack(tmp_path, ack):
    class Worker(FakeWorker):
        def call(self, message, cancelled):
            raw = super().call(message, cancelled)
            if ack: raw["effective_sampler"] = {"configuration": message["planner_config"]["sampling"]}
            return raw
    worker = Worker()
    backend = task_backend(args("--planner-config", str(config_file(tmp_path)), "--native-max-attempts", "1",
                                "--stop-on-native-block"), worker)
    request, scene = inputs()
    authority = []
    for start, goal in [((0.,), (1.,)), ((.2,), (.9,))]:
        result = backend.plan(replace(request, q_start=start, q_goal=goal), scene, lambda p: authority.append(p))
        assert result.deliverable == ack
        assert worker.calls[-1]["q_start"] == start and worker.calls[-1]["q_goal"] == goal
        assert worker.calls[-1]["planner_config"]["sampling"]["local_probability"] == .5
    assert len(authority) == 2 * int(ack)
    assert planner_configuration(args()).to_mapping() == OMPLPlannerConfig().to_mapping()


def test_cli_json_conflicts_unknown_duplicate_and_invalid_values_rejected(tmp_path):
    p = config_file(tmp_path)
    with pytest.raises(ValueError, match="conflicts"):
        planner_configuration(args("--planner-config", str(p), "--ompl-planner", "rrt_connect"))
    assert planner_configuration(args("--planner-config", str(p), "--ompl-planner", "lazy_prm")).sampling
    for text in ['{"unknown": 1}', '{"name":"lazy_prm","name":"rrt_connect"}',
                 '{"name":"lazy_prm","sampling":{"local_probability":0}}', '[]']:
        p.write_text(text)
        with pytest.raises(ValueError): planner_configuration(args("--planner-config", str(p)))


@pytest.mark.parametrize("field", [None, "q_start", "q_goal", "seed", "scene", "planner_config"])
def test_first_request_identity_blocks_before_transport(tmp_path, monkeypatch, field):
    expected = dict(q_start=[0.], q_goal=[1.], seed=71081, scene={"attachment": None}, planner_config={})
    reference = tmp_path / "expected.json"; reference.write_text(json.dumps(expected))
    calls = []
    monkeypatch.setattr(NativeWorker, "call", lambda self, message, cancelled: calls.append(message) or {"status": "CANDIDATE"})
    worker = RecordingWorker("not-launched", tmp_path, reference)
    request = deepcopy(expected)
    request["q_start"], request["q_goal"] = tuple(request["q_start"]), tuple(request["q_goal"])
    if field: request[field] = "different"
    if field:
        with pytest.raises(NativePlanningBlocked): worker.call(request)
        assert not calls
    else:
        worker.call(request)
        assert json.loads(json.dumps(calls)) == [expected]
    assert worker.transport_call_count == int(field is None)
    check = json.loads((tmp_path / "native-01.identity-check.json").read_text())
    assert check["status"] == ("FAIL" if field else "PASS")


@pytest.mark.parametrize("different", [False, True])
def test_production_backend_to_recording_guard_uses_wire_semantics(tmp_path, monkeypatch, different):
    request, scene = inputs()
    expected = dict(q_start=list(request.q_start), q_goal=list(request.q_goal), seed=request.seed,
                    scene=scene, planner_config=OMPLPlannerConfig().to_mapping())
    if different: expected["q_goal"][0] += .1
    reference = tmp_path / "reference.json"; reference.write_text(json.dumps(expected))
    fake = FakeWorker()
    monkeypatch.setattr(NativeWorker, "call", lambda self, data, cancelled: fake.call(data, cancelled))
    worker = RecordingWorker("unused", tmp_path, reference)
    backend = task_backend(args(), worker)
    if different:
        with pytest.raises(NativePlanningBlocked): backend.plan(request, scene, lambda p: None)
        assert not fake.calls and worker.transport_call_count == 0
    else:
        assert backend.plan(request, scene, lambda p: None).deliverable
        assert len(fake.calls) == worker.transport_call_count == 1


def test_block_is_persisted_before_stop():
    saved = []
    connector = SimpleNamespace(stage_evidence_callback=saved.append)
    with pytest.raises(NativePlanningBlocked):
        _record_free_motion(connector, SimpleNamespace(stop_on_native_block=True),
                            {"status": "BUDGET_EXHAUSTED", "stage": "transit"})
    assert saved[0]["stage"] == "transit"


def test_complete_geometry_requires_current_production_completion_proof():
    from test_layout_trajectory import _segment
    segment = _segment(); segment["validation"]["completion"] = {"fixture": True}
    scene = SimpleNamespace(cartons=[SimpleNamespace(name=segment["target"])], all_obstacles=[])
    connector = SimpleNamespace(_task_completed=lambda *a: False)
    with pytest.raises(ValueError, match="proof"):
        delivery.complete_geometry({"selected_trajectory_segment": segment}, scene, connector)
    connector._task_completed = lambda *a: True
    assert delivery.complete_geometry({"selected_trajectory_segment": segment}, scene, connector)["status"] == "PASS"
    segment["events"].pop()
    with pytest.raises(ValueError): delivery.complete_geometry({"selected_trajectory_segment": segment}, scene, connector)


def test_authority_identity_comparison_detects_a_real_checker_change():
    from pathlib import Path
    from tools.tesseract_ompl_v3_identity import without_evidence_sink
    source = (Path(__file__).resolve().parents[1] / "src/unloading_sim/layout_trajectory.py").read_text()
    assert "def _state_failure(" in source
    changed = source.replace("def _state_failure(", "def _disabled_state_failure(", 1)
    assert without_evidence_sink(source) != without_evidence_sink(changed)


def test_actual_production_export_and_command_acceptance_preserve_large_finite_joint_edge():
    plan, cfg, preflight = _ready_m710_inputs()
    segment = deepcopy(plan["segments"][0])
    # This fixture is not the engineering path. It exercises the >pi failure mode.
    segment["path"][1][0] = segment["path"][0][0] + 3.2545
    plan, cfg, preflight = _ready_m710_inputs(segment=segment)
    bundle = delivery.build_fanuc_isaac_replay_bundle(plan, cfg, preflight=preflight)
    assert delivery.accept_controller_reference(bundle, segment)["status"] == "PASS"
    assert delivery.verify_m710_replay_bundle(bundle.to_dict())["status"] == "PASS"
    broken = bundle.to_dict()
    broken["metadata"]["joint_reference"]["positions_rad"][1][0] -= 2*np.pi
    with pytest.raises(ValueError): delivery.accept_controller_reference(broken, segment)
    broken = bundle.to_dict(); broken["metadata"]["release_time_seconds"] = 0.
    with pytest.raises(ValueError): delivery.accept_controller_reference(broken, segment)


@pytest.mark.parametrize("fail_gate", [None, "geometry", "preflight", "timing", "commands", "readback"])
def test_success_exit_dependency_order_and_failure_status(tmp_path, monkeypatch, fail_gate):
    plan, cfg, preflight = _ready_m710_inputs()
    order = []
    def wrap(name, function):
        def call(*a, **kw):
            order.append(name)
            if fail_gate == name: raise ValueError("fixture injected " + name)
            return function(*a, **kw)
        return call
    monkeypatch.setattr(delivery, "complete_geometry", wrap("geometry", lambda *a: {"status": "PASS", "fixture": True}))
    monkeypatch.setattr(delivery, "motion_envelope", lambda *a: {"fixture": True})
    monkeypatch.setattr(delivery, "build_m710_execution_preflight", wrap("preflight", lambda *a, **kw: preflight))
    monkeypatch.setattr(delivery, "build_fanuc_isaac_replay_bundle", wrap("timing", delivery.build_fanuc_isaac_replay_bundle))
    monkeypatch.setattr(delivery, "accept_controller_reference", wrap("commands", delivery.accept_controller_reference))
    verify = delivery.verify_m710_replay_bundle
    monkeypatch.setattr(delivery, "verify_m710_replay_bundle", wrap("readback", lambda b, **kw: verify(b)))
    result = dict(selected_trajectory_segment=plan["segments"][0], path_metrics={})
    delivery.prepare_task_delivery(result, None, None, tmp_path / "delivery", project_root=tmp_path)
    stages = ["geometry", "preflight", "timing", "commands", "readback"]
    assert order == stages[:stages.index(fail_gate)+1] if fail_gate else order == stages
    assert result["simulation_execution_ready"] == (fail_gate is None)
    if fail_gate:
        assert result[result["delivery_failure"]["gate"]] == "FAIL"
        if fail_gate == "geometry":
            assert result["execution_preflight"] == result["time_parameterization_status"] == "NOT_RUN"
        if fail_gate == "preflight": assert result["time_parameterization_status"] == "NOT_RUN"
    else:
        assert all(result[k] == "PASS" for k in ("complete_geometry_status", "execution_preflight",
            "time_parameterization_status", "execution_trajectory_status"))
        assert (tmp_path / "delivery/replay_bundle.json").is_file()
