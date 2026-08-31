import numpy as np

from unloading_sim.robustness import (
    RobustnessNoise,
    TrialOutcome,
    run_monte_carlo_robustness,
)


def _noise():
    return RobustnessNoise(
        perception_position_std_m=[0.01, 0.01, 0.02],
        carton_size_std_m=[0.005, 0.005, 0.005],
        base_position_std_m=[0.003, 0.003],
        base_yaw_std_rad=0.002,
        suction_leak_probability=0.2,
    )


def test_monte_carlo_is_reproducible_and_reports_failure_modes():
    def evaluate(trial):
        if trial.suction_leak:
            return TrialOutcome(False, "suction_leak")
        clearance = 0.04 - float(np.linalg.norm(trial.perception_offset_m))
        return TrialOutcome(clearance >= 0.0, "collision_margin" if clearance < 0 else "success", clearance)

    first = run_monte_carlo_robustness(evaluate, _noise(), trials=200, seed=23)
    second = run_monte_carlo_robustness(evaluate, _noise(), trials=200, seed=23)

    assert first == second
    assert 0.0 < first["success_probability"] < 1.0
    assert first["failure_reasons"]["suction_leak"] > 0
    assert first["success_probability_95pct_wilson"][0] < first["success_probability"]


def test_monte_carlo_boolean_evaluator_is_supported():
    result = run_monte_carlo_robustness(lambda trial: not trial.suction_leak, _noise(), trials=10, seed=1)
    assert result["successes"] <= 10
    assert result["model"].startswith("seeded")
