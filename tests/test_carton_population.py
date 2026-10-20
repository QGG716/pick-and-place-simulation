"""Population scheduling regressions; no synthetic planning success evidence."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.carton_population import FrozenCartonPopulation, rank_population, blocked_population
from unloading_sim.geometry import OBB
from unloading_sim.top_layer_lifted import FrozenTopLayer, process_options
from unloading_sim.workcell_layout import load_workcell_layout


@pytest.fixture
def scene():
    layout = load_workcell_layout(Path(__file__).resolve().parents[1]/
        "configs/workcells/m710id70_unloading_layout_v1_transverse_scissor_026.yaml")
    cartons = layout.cartons()
    # IDs deliberately carry no layer/column semantics.
    cartons = tuple(OBB(b.center, b.half_extents, b.rotation, f"opaque-{i}", b.category)
                    for i, b in enumerate(cartons))
    bodies = {b.name: b for b in layout.fixed_components()}
    return cartons, bodies["conveyor_transverse"], bodies["conveyor_longitudinal"]


def test_population_freezes_original_per_layer_roles_without_expanding_old_scope(scene):
    cartons, belt, longitudinal = scene
    frozen = FrozenCartonPopulation.capture(cartons)
    assert len(frozen.target_ids) == 40
    assert len(FrozenTopLayer.capture(cartons).target_ids) == 5
    roles = frozen.evidence()["roles"]
    assert sum(r["role"] == "RIGHT_END" for r in roles.values()) == 8
    assert sum(r["role"] == "LEFT_END" for r in roles.values()) == 8
    assert sum(r["role"] == "MIDDLE" for r in roles.values()) == 24
    middle = next(b for b in cartons if roles[b.name]["role"] == "MIDDLE")
    options = process_options(middle, belt, frozen, [middle], longitudinal=longitudinal)
    assert all(o["original_role"] == "MIDDLE" for o in options)
    assert all("INNER" not in o["template"] for o in options)
    assert frozen.evidence()["eccentric_authority"] == "NONE_ADDED_BY_POPULATION_SCOPE"
    with pytest.raises(ValueError):
        frozen.remaining_ids([middle.name, middle.name])


def test_failed_top_remains_support_and_is_not_given_a_new_task_budget(scene):
    cartons, belt, longitudinal = scene
    frozen = FrozenCartonPopulation.capture(cartons)
    top = FrozenTopLayer.capture(cartons).target_ids
    failed = top[-1]
    completed = top[:-1]
    active = tuple(b for b in cartons if b.name not in completed)
    robot = SimpleNamespace(within_limits=lambda q: True, fk=lambda q: np.eye(4))
    ranked = rank_population(frozen, active, completed, {failed}, np.zeros(6), robot, belt,
                             longitudinal=longitudinal)
    assert len(ranked) == 4
    assert failed not in [r["target_id"] for r in ranked]
    assert all(not r["supported_cartons"] for r in ranked)
    blocked = blocked_population(active, {failed})
    assert len(blocked) == 31
    assert sum(failed in r["supported_cartons"] for r in blocked) == 1
    assert len(active) == 36
    assert len(ranked)+len(blocked)+1 == 36


def test_low_layer_frozen_heights_are_not_clamped_to_mechanism_minimum(scene):
    cartons, belt, longitudinal = scene
    frozen = FrozenCartonPopulation.capture(cartons)
    bottom = min(cartons, key=lambda b: b.center[2])
    options = process_options(bottom, belt, frozen, [bottom], longitudinal=longitudinal)
    heights = [o["height_m"] for o in options]
    assert max(heights) == pytest.approx(-.05)
    assert min(heights) == pytest.approx(-.35)
    assert all(h < .6 for h in heights)
