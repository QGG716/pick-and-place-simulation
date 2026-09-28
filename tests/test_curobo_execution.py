"""Controller handoff contract tests; physical success requires real execution."""
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.curobo_execution import execution_knots
from unloading_sim.timing import sample_quintic_knots


def fixture():
    path=np.array([[0.]*6,[.1]*6,[.2]*6,[.3]*6,[.4]*6,[.5]*6])
    segment=dict(path=path.tolist(),grasp_index=2,release_index=4,release_retreat_index=5,
        stage_ranges=dict(home=[0,0],contact=[0,2],extraction=[2,3],transit=[3,4],
                          place=[4,4],withdrawal=[4,5]))
    retained=[0,2,3,4,5]
    # Hold at attachment; actual geometry omits only the first collinear point.
    reference=dict(interpolation='C2_piecewise_quintic_rest_to_rest',
        timestamps_seconds=[0.,1.,2.,3.,4.,5.],positions_rad=path[[0,2,2,3,4,5]].tolist(),
        source_retained_indices=retained)
    times=np.linspace(0,5,51)
    q,_=sample_quintic_knots(reference['timestamps_seconds'],reference['positions_rad'],times)
    from unloading_sim.isaac_bridge import IsaacReplayBundle
    bundle=IsaacReplayBundle(times,q,dict(joint_names=list('abcdef'),joint_reference=reference)).to_dict()
    return segment,bundle


def test_execution_checks_retained_geometry_and_permits_timing_holds():
    segment,bundle=fixture()
    path,indices=execution_knots(segment,bundle)
    assert indices==[0,2,3,4,5] and len(path)==6


@pytest.mark.parametrize('change',['spline','command','unbound_knot','event'])
def test_changed_reference_cannot_reuse_geometric_acceptance(change):
    segment,bundle=fixture();bundle=deepcopy(bundle)
    reference=bundle['metadata']['joint_reference']
    if change=='spline':reference['interpolation']='native_bspline'
    if change=='command':bundle['positions_rad'][10][0]+=.001
    if change=='event':reference['source_retained_indices']=[0,3,4,5]
    if change=='unbound_knot':
        reference['positions_rad'][1][0]+=.001
        q,_=sample_quintic_knots(reference['timestamps_seconds'],reference['positions_rad'],bundle['timestamps_seconds'])
        bundle['positions_rad']=q.tolist()
    with pytest.raises(ValueError):execution_knots(segment,bundle)
