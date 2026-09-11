from dataclasses import replace
import numpy as np

from test_extraction_boundary_guard import fixture


def test_local_transit_chunks_existing_cartesian_and_accounts_shared_samples():
    connector, start, attachment, _, _ = fixture()
    connector.budget = replace(connector.budget, cartesian_max_samples_per_stage=4)
    connector._local_transit_remaining = 40
    goal = connector.robot.fk(start)
    goal[:3, 3] += [-.3, 0, .3]
    before = connector._statistics["cartesian_samples"]
    path, failure, evidence = connector._local_cartesian_transit(start, goal, [], attachment, seed=7)
    assert failure is None
    assert len(path) > 4
    assert np.linalg.norm(connector.robot.fk(path[-1])[:3, 3] - goal[:3, 3]) < 1e-4
    assert 40 - connector._local_transit_remaining == connector._statistics["cartesian_samples"] - before
    assert all(len(search["samples"]) <= 4 for search in evidence["attempts"][0]["searches"])


def test_exhausted_local_budget_cannot_silently_grant_each_placement_more_samples():
    connector, start, attachment, _, _ = fixture()
    connector._local_transit_remaining = 1
    goal = connector.robot.fk(start)
    goal[:3, 3] += [-1., 0, 1.]
    path, failure, _ = connector._local_cartesian_transit(start, goal, [], attachment, seed=8)
    assert not path and failure is not None
    assert connector._statistics["cartesian_samples"] <= 1
    assert connector._local_transit_remaining >= 0
