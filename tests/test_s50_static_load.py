"""Literal numeric checks for the S50 quasi-static load screening primitives."""
import numpy as np
import pytest

from unloading_sim.s50_static_load import (GRAVITY_M_S2, base_moment, gravity_torques,
    mesh_mass_properties, payload_curve_limit_kg, scale_to_total_mass)

CUBE_V = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], float)
CUBE_T = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                   [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])


def test_unit_cube_mass_properties():
    p = mesh_mass_properties(CUBE_V, CUBE_T)
    assert p['volume_m3'] == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_allclose(p['centroid_m'], [.5, .5, .5], atol=1e-12)
    np.testing.assert_allclose(p['inertia_unit_density'], np.eye(3) / 6., atol=1e-12)
    assert p['boundary_edges'] == 0 and p['nonmanifold_edges'] == 0


def test_inverted_winding_and_open_mesh_are_reported():
    p = mesh_mass_properties(CUBE_V, CUBE_T[:, ::-1])
    assert p['volume_m3'] == pytest.approx(1.0, abs=1e-12)
    assert mesh_mass_properties(CUBE_V, CUBE_T[:-1])['boundary_edges'] == 3


def test_density_scaling_preserves_published_total():
    p = mesh_mass_properties(CUBE_V, CUBE_T)
    scaled, density = scale_to_total_mass({'a': p, 'b': dict(p, volume_m3=3.)}, 156.)
    assert density == pytest.approx(39.)
    assert scaled['a']['mass_kg'] + scaled['b']['mass_kg'] == pytest.approx(156.)


@pytest.mark.parametrize('offset_m,limit', [(0., 50.), (.04, 50.), (.05, 41.7), (.25, 38.35),
                                            (.405, 35.75375), (.45, 35.), (.8, 21.8)])
def test_digitized_s50_payload_curve(offset_m, limit):
    assert payload_curve_limit_kg(offset_m) == pytest.approx(limit)


def test_payload_curve_outside_published_range():
    assert payload_curve_limit_kg(.81) is None
    with pytest.raises(ValueError):
        payload_curve_limit_kg(-.01)


def test_single_joint_gravity_torque_and_base_moment():
    # Joint at origin about +Y holding 1 kg at +X 1 m: actuator must supply -g.
    torque = gravity_torques([np.zeros(3)], [np.array([0., 1., 0.])], [(0, 1., [1., 0., 0.])])
    np.testing.assert_allclose(torque, [-GRAVITY_M_S2])
    # Mass on the axis line needs no torque; a mass carried only by joint 0 does not load joint 1.
    torque = gravity_torques([np.zeros(3), np.array([1., 0, 0])], [np.array([0, 1., 0]), np.array([0, 1., 0])],
                             [(0, 2., [2., 0., 0.]), (1, 1., [1., 0., 5.])])
    np.testing.assert_allclose(torque, [-2 * 2 * GRAVITY_M_S2 - 1 * GRAVITY_M_S2, 0.])
    moment, weight = base_moment(np.zeros(3), [(0, 2., [2., 0., 0.])])
    np.testing.assert_allclose(moment, [0., 4 * GRAVITY_M_S2, 0.])
    assert weight == pytest.approx(2 * GRAVITY_M_S2)
