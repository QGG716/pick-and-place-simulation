"""Limits and timing must not be confused with the independently usable CAD FK."""
import numpy as np
import pytest

from unloading_sim.s50_qualification import (
    controller_mapping_audit, convert_controller_limits, geometric_definition_errors, timing_audit,
)


def test_controller_sign_offset_conversion_has_literal_endpoints():
    # Synthetic example of a negative axis: [-190, 10] = -q + (-90).
    actual = convert_controller_limits(np.deg2rad([-190., 10.]), -1, np.deg2rad(-90.))
    np.testing.assert_allclose(actual, np.deg2rad([-100., 100.]), atol=1e-14)
    np.testing.assert_allclose(convert_controller_limits([-2., 1.], 1, .5), [-2.5, .5])


@pytest.mark.parametrize('sign,offset', [(0, 0.), (2, 0.), (True, 0.), (1, float('nan')), (-1, None)])
def test_mapping_rejects_unknown_sign_or_offset(sign, offset):
    with pytest.raises(ValueError):
        convert_controller_limits([-1., 1.], sign, offset)


def test_numbers_without_verified_reference_cannot_qualify_limits():
    data = dict(joint_position_limits_rad={f'J{i}': [-1., 1.] for i in range(1,7)},
        controller_mapping={'joints': {f'J{i}': dict(sign=1, offset_rad=0., source=None) for i in range(1,7)}})
    report = controller_mapping_audit(data)
    assert not report['ready']
    assert report['converted_limits_rad'] == {}
    assert len(report['missing']) == 6


def test_speed_revision_is_only_a_timing_gate():
    data = {'joint_velocity_version_conflict': {'manual': [1]*6, 'product_page': [2]*6}}
    assert not any('velocity' in item for item in geometric_definition_errors(data))
    timing = timing_audit(data)
    assert not timing['ready']
    assert not timing['blocks_static_fk'] and not timing['blocks_geometric_search']


def test_temporary_timing_must_name_engineering_assumption():
    data = dict(time_parameterization_assumption=dict(kind='ENGINEERING_ASSUMPTION',
        manufacturer_confirmed=False, source_or_rationale='test-only numerical timing fixture',
        velocity_rad_s=[.1]*6, acceleration_rad_s2=[.2]*6),
        joint_velocity_version_conflict={'a': 1, 'b': 2})
    assert timing_audit(data)['status'] == 'READY_WITH_ENGINEERING_ASSUMPTION'
    data['time_parameterization_assumption']['manufacturer_confirmed'] = True
    assert not timing_audit(data)['ready']


def test_incomplete_published_speed_and_acceleration_cannot_qualify_timing():
    invalid = timing_audit({'joint_acceleration_limits_rad_s2': [0., float('nan')]})
    assert not invalid['ready'] and not invalid['manufacturer_confirmed']
    assert not invalid['blocks_static_fk']
    complete = dict(joint_velocity_limits_rad_s=[1.]*6, joint_velocity_source='test fixture source',
        joint_acceleration_limits_rad_s2=[2.]*6, joint_acceleration_source='test fixture source')
    assert timing_audit(complete)['ready']
    complete['joint_acceleration_limits_rad_s2'][3] = -1
    assert not timing_audit(complete)['ready']
