"""Small regressions for measured interface identity and shared numeric policy."""
import copy
import numpy as np
import pytest
from unloading_sim.eco65.model import distance_accepted, mounting_pairs


@pytest.mark.parametrize('threshold',[.0024,-.0004])
@pytest.mark.parametrize('delta,accepted',[(0.,True),(-.5e-9,True),(-2e-9,False),(2e-9,True)])
def test_numeric_epsilon_preserves_physical_threshold(threshold,delta,accepted):
    assert distance_accepted(threshold+delta,threshold) == accepted


def test_nonfinite_distance_is_rejected():
    assert not any(distance_accepted(x,.0024) for x in (np.nan,np.inf,-np.inf))


def test_measured_mount_is_specific_rigid_asset_not_legacy_name():
    tool=dict(adapter=[],collisions=[dict(id='actual_cap',semantic='conservative_rigid'),
        dict(id='lip',semantic='terminal_contact_band')],mounting_contacts=[dict(robot_link='link_6',
            tool_collision='actual_cap',maximum_penetration_m=.0004)])
    assert mounting_pairs(tool)=={frozenset(['robot_link_6','actual_cap'])}
    for key,value in [('robot_link','link_5'),('tool_collision','lip'),('tool_collision','missing'),('maximum_penetration_m',.005)]:
        broken=copy.deepcopy(tool);broken['mounting_contacts'][0][key]=value
        with pytest.raises(ValueError):mounting_pairs(broken)
    tool['mounting_contacts']*=2
    with pytest.raises(ValueError):mounting_pairs(tool)
