"""Component reporting fixtures; these do not qualify real S50 task poses."""
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.s50_compactness import compactness_sample, group_envelopes, summarize_samples


def fixture():
    geometry=[]; pairs=[]
    for i,link in enumerate(['s50_base',*[f's50_link_{j}' for j in range(1,7)]]):
        world=np.eye(4); world[:3,3]=[i,.25,1.]
        geometry.append(dict(link=link,geometry=link+'_0',vertices=np.array([[0,0,0],[.2,.1,.3]]),world=world))
        pairs.append(dict(kind='robot_environment',first=dict(link=link,geometry=link+'_0'),
            second=dict(object='conveyor_transverse'),status='QUERIED',unsigned_separation_m=.01 if i==0 else i*.1,
            intersects=False))
    return geometry,pairs


def sample(geometry,pairs,**kwargs):
    return compactness_sample(geometry,pairs,np.eye(4),q_rad=[0]*6,pose_id='synthetic',conveyor_height_m=.6,**kwargs)


def test_fixed_base_minimum_does_not_hide_active_elbow_and_wrist():
    geometry,pairs=fixture(); row=sample(geometry,pairs)
    assert [row['groups'][k]['minimum_unsigned_separation_m'] for k in ('all_robot','active_links','elbow','wrist')]==[.01,.1,.2,.4]
    assert row['groups']['elbow']['envelope']['horizontal_bounds_xy_m']==[[2.,.25],[3.2,.35]]
    assert row['replaces_all_robot_safety_checks'] is False
    assert row['mechanical_limit_qualification_implied'] is False


@pytest.mark.parametrize('change',['missing','duplicate','unknown','nonfinite'])
def test_incomplete_component_pairs_never_claim_complete_minimum(change):
    geometry,pairs=fixture()
    if change=='missing':pairs.pop()
    if change=='duplicate':pairs.append(deepcopy(pairs[-1]))
    if change=='unknown':pairs[-1]['status']='FAILED'
    if change=='nonfinite':pairs[-1]['unsigned_separation_m']=float('nan')
    row=sample(geometry,pairs)
    assert row['groups']['wrist']['minimum_unsigned_separation_m'] is None
    assert not row['groups']['wrist']['all_component_pairs_known']
    assert row['groups']['elbow']['minimum_unsigned_separation_m']==.2


def test_envelope_is_measured_in_installation_reference_not_world():
    geometry,_=fixture(); installation=np.array([[0,-1,0,10],[1,0,0,20],[0,0,1,3],[0,0,0,1.]])
    reference=group_envelopes(geometry,np.eye(4))
    for item in geometry:item['world']=installation@item['world']
    actual=group_envelopes(geometry,installation)
    for name in reference:np.testing.assert_allclose(actual[name]['mount_frame_bounds_m'],reference[name]['mount_frame_bounds_m'],atol=1e-12)


def test_missing_or_nonfinite_mesh_is_rejected():
    geometry,_=fixture()
    with pytest.raises(ValueError,match='GEOMETRY_MISSING'):group_envelopes(geometry[:-1],np.eye(4))
    geometry[-1]['vertices'][0,0]=float('nan')
    with pytest.raises(ValueError,match='FINITE_MESH'):group_envelopes(geometry,np.eye(4))


def test_phase_summary_retains_configuration_and_fails_closed_for_missing_pair():
    geometry,pairs=fixture(); a=sample(geometry,pairs,phase='withdraw')
    b=deepcopy(a);b['pose_id']='second';b['q_rad']=[.1]*6
    b['groups']['wrist']['minimum_unsigned_separation_m']=.03
    b['groups']['wrist']['envelope']['mount_frame_bounds_m'][1][0]=9.
    result=summarize_samples([a,b]); wrist=result['phases']['withdraw']['wrist']
    assert wrist['minimum_sampled_distance_m']==.03
    assert wrist['minimum_configuration']['q_rad']==[.1]*6
    assert wrist['horizontal_extrema']['x_max']['value_m']==9.
    assert wrist['horizontal_extrema']['x_max']['pose_id']=='second'
    b['groups']['wrist']['all_component_pairs_known']=False
    assert summarize_samples([a,b])['phases']['withdraw']['wrist']['minimum_sampled_distance_m'] is None
