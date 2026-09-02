from pathlib import Path

import numpy as np
import pytest

from unloading_sim.cycle import (
    draw_cycle_randomness,
    load_cycle_model,
    required_normal_cycle_seconds,
    simulate_cycles,
)


def _model():
    root = Path(__file__).resolve().parents[1]
    return load_cycle_model(root / "configs/cycle/unloading_900pph.yaml")[0]


def test_cycle_simulation_is_seed_reproducible():
    model = _model()
    first_cycles, first = simulate_cycles(model, 3.2)
    second_cycles, second = simulate_cycles(model, 3.2)
    assert np.array_equal(first_cycles, second_cycles)
    assert first == second
    assert len(first_cycles) == 100000


def test_common_random_numbers_make_candidate_delta_exact():
    model = _model()
    draws = draw_cycle_randomness(model)
    fast, _ = simulate_cycles(model, 3.0, draws)
    slow, _ = simulate_cycles(model, 4.0, draws)
    assert slow - fast == pytest.approx(np.ones(model.sample_count))


def test_required_normal_cycle_hits_target_mean():
    model = _model()
    draws = draw_cycle_randomness(model)
    required = required_normal_cycle_seconds(model, draws)
    _cycles, summary = simulate_cycles(model, required, draws)
    assert summary["boxes_per_hour"] == pytest.approx(model.target_boxes_per_hour)


def test_conditional_recovery_never_exceeds_grasp_retries():
    model = _model()
    draws = draw_cycle_randomness(model)
    recovery = draws.loss_occurrences["RECOVERY"]
    retry = draws.loss_occurrences["GRASP_RETRY"]
    assert np.all(~recovery | retry)
