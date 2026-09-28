import copy
import math

import pytest

from unloading_sim.joint_effort import (
    CONDITIONS, JointEffortMonitor, projected_source, safe_read_channel,
)
from unloading_sim.qualification import evaluate_replay_qualification
from test_qualification import _passing_inputs


def monitor(**overrides):
    args = dict(joint_names=["J1", "J2"], limits_nm=[10., 20.],
                limit_source="fixture_limits", physics_dt=.01,
                source=projected_source(isaac_version="6.0.1.0", tensor_version="110.1.13", backend="CPU_PhysX"))
    args.update(overrides)
    return JointEffortMonitor(**args)


def sample(m, **overrides):
    args = dict(joint_names=["J1", "J2"], physics_step=1, simulation_time=.01,
                observation_phase="post_physics_step", raw_values=[1., 2.],
                applicability=dict.fromkeys(CONDITIONS, True))
    args.update(overrides)
    return m.observe(**args)


def test_valid_scope_and_complete_coverage_pass():
    m = monitor(configured_readback_nm=[10., 20.])
    sample(m)
    result = m.summary(1)
    assert result["status"] == "PASS"
    assert result["configured_limits_read_back"]


@pytest.mark.parametrize("kind", ["explicit_effort_input", "model_inverse_dynamics", "joint_reaction_wrench", "configured_max_effort"])
def test_other_quantities_cannot_be_renamed_drive_output(kind):
    m = monitor()
    m.source["quantity_kind"] = kind
    sample(m, raw_values=[0., 0.])
    assert m.summary(1)["status"] == "NOT_EVALUATED"


@pytest.mark.parametrize("changes", [
    {"raw_values": None}, {"raw_values": [math.nan, 0.]}, {"raw_values": [1.]},
    {"joint_names": ["J2", "J1"]}, {"simulation_time": None}, {"simulation_time": .02},
    {"observation_phase": "pre_physics_step"}, {"physics_step": 0},
])
def test_invalid_observations_are_not_zero_or_overload(changes):
    m = monitor()
    observation = sample(m, **changes)
    assert observation["stop_reason"] == "JOINT_EFFORT_OBSERVATION_LOST"
    assert m.summary(1)["status"] == "NOT_EVALUATED"


def test_valid_overload_fails_even_with_incomplete_other_steps():
    m = monitor()
    row = sample(m, raw_values=[11., 2.])
    assert row["stop_reason"] == "JOINT_EFFORT_LIMIT_EXCEEDED"
    sample(m, physics_step=2, simulation_time=.02, raw_values=None)
    summary = m.summary(3)
    assert summary["status"] == "FAIL"
    assert summary["first_failure"]["joint"] == "J1"
    assert summary["first_failure"]["physics_step"] == 1


@pytest.mark.parametrize("condition", CONDITIONS)
def test_source_applicability_must_be_proven(condition):
    m = monitor()
    scope = dict.fromkeys(CONDITIONS, True)
    scope[condition] = None
    sample(m, applicability=scope, raw_values=[99., 99.])
    assert m.summary(1)["status"] == "NOT_EVALUATED"


def test_scope_and_version_are_not_inferred_from_nonzero_values():
    m = monitor()
    m.source["backend"] = "Newton"
    sample(m)
    assert m.summary(1)["status"] == "NOT_EVALUATED"


def test_missing_step_and_duplicate_do_not_pass():
    m = monitor()
    sample(m)
    sample(m, physics_step=3, simulation_time=.03)
    assert m.summary(3)["status"] == "NOT_EVALUATED"
    sample(m, physics_step=3, simulation_time=.03)
    assert m.summary(3)["status"] == "NOT_EVALUATED"


def test_limits_readback_alone_cannot_pass():
    m = monitor(configured_readback_nm=[10., 20.])
    assert m.summary(1)["configured_limits_read_back"]
    assert m.summary(1)["status"] == "NOT_EVALUATED"


def test_channel_exception_is_explicit():
    def reader():
        raise RuntimeError("unsupported")
    values, reason = safe_read_channel(reader, 2)
    assert values is None and "unsupported" in reason


def test_qualification_distinguishes_failure_and_missing_and_preserves_assumptions():
    args = _passing_inputs()
    args.update(ideal_holding_capacity_assumption=True, joint_effort_monitor=monitor(), required_effort_steps=1)
    before = copy.deepcopy(args)
    result = evaluate_replay_qualification(**args)
    assert result["qualification_not_evaluated"] == ["joint_efforts_within_limit"]
    assert result["qualification_measured_failures"] == []
    assert not result["qualification_passed"]
    assert result["qualification_failures"] == ["joint_efforts_within_limit"]
    assert result["simulation_assumptions"] == ["ideal_independent_cups_holding_capacity"]
    assert args["full_schedule_replayed"] == before["full_schedule_replayed"]


def test_historical_workflow_and_ideal_downstream_are_preserved():
    import json
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    original = json.loads((root / "docs/evidence/curobo_v2_20260928_isaac/physical_trial_summary.json").read_text())
    assert original["workflow_cycle_completed"] is True
    assert original["physical_cycle_completed"] is False
    assert original["qualification_passed"] is False
    assert original["new_target_execution_counts"]["actual_received"] == 0
    assert original["new_target_execution_counts"]["ideal_received"] == 1
    assert original["drive_effort_output_qualified"] is False


def test_effort_implementation_is_bound_to_preflight_identity():
    from unloading_sim.m710_execution import EXECUTION_IMPLEMENTATION_FILES
    assert "src/unloading_sim/joint_effort.py" in EXECUTION_IMPLEMENTATION_FILES


def test_collector_explicit_channel_loss_is_not_a_valid_zero():
    import numpy as np
    from unloading_sim.joint_effort import collect_isaac_effort_sample
    class Tensor:
        def numpy(self):
            return np.asarray([[1., 2.]])
    class Articulation:
        dof_names = ["J1", "J2"]
        def get_dof_efforts(self):
            raise RuntimeError("lost input channel")
        def get_dof_projected_joint_forces(self):
            return Tensor()
        def get_link_incoming_joint_force(self):
            raise RuntimeError("not available")
    m = monitor()
    row = collect_isaac_effort_sample(Articulation(), m, 1, .01, dict.fromkeys(CONDITIONS, True))
    assert row["explicit_input_nm"] is None
    assert row["raw_values"] == [1., 2.]
    assert row["incoming_wrench_child_joint_frame"] is None
    assert row["stop_reason"] == "JOINT_EFFORT_OBSERVATION_LOST"
    assert m.summary(1)["status"] == "NOT_EVALUATED"


@pytest.mark.parametrize("ratios", [[0., 0.], [.2, .8], [2., 3.], [float("nan")]])
def test_untyped_legacy_ratios_cannot_claim_observed_pass_or_failure(ratios):
    args = _passing_inputs()
    args.update(joint_effort_monitor=None, effort_limit_ratios=ratios)
    result = evaluate_replay_qualification(**args)
    assert result["qualification_check_details"]["joint_efforts_within_limit"]["status"] == "NOT_EVALUATED"
    assert result["qualification_measured_failures"] == []
