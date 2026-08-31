import numpy as np
import pytest

from unloading_sim.base_optimization import (
    BasePose,
    geometric_trailer_coverage_objective,
    optimize_base_pose_continuous,
)


def test_continuous_optimizer_is_seeded_and_not_restricted_to_dock_enumeration():
    optimum = np.array([1.137, -0.243, 0.371])

    def objective(pose):
        error = pose.as_array() - optimum
        return -float(error @ error)

    first = optimize_base_pose_continuous(
        objective, [[0.0, 2.0], [-1.0, 1.0], [-0.8, 0.8]], seed=7,
        initial_samples=256, refinement_iterations=30,
    )
    second = optimize_base_pose_continuous(
        objective, [[0.0, 2.0], [-1.0, 1.0], [-0.8, 0.8]], seed=7,
        initial_samples=256, refinement_iterations=30,
    )

    assert np.allclose(first.pose.as_array(), second.pose.as_array())
    assert np.linalg.norm(first.pose.as_array() - optimum) < 0.01
    assert first.evaluations > 4


def test_geometric_coverage_prefers_pose_facing_targets():
    targets = [[2.0, -0.2, 1.0], [2.0, 0.2, 1.5], [2.5, 0.0, 2.0]]
    objective = geometric_trailer_coverage_objective(
        targets,
        minimum_reach_m=0.5,
        maximum_reach_m=3.0,
        minimum_height_m=0.2,
        maximum_height_m=2.5,
    )
    assert objective(BasePose(0.0, 0.0, 0.0)) > objective(BasePose(0.0, 0.0, np.pi))


def test_continuous_optimizer_rejects_nonfinite_objective():
    with pytest.raises(ValueError, match="non-finite"):
        optimize_base_pose_continuous(
            lambda pose: np.nan, [[0, 1], [0, 1], [-1, 1]], seed=1, initial_samples=1
        )
