"""Measured pre-removal -> independence -> first takeover, through production gates."""
from copy import deepcopy
import numpy as np
import pytest
from unloading_sim.geometry import OBB
from unloading_sim.m710_replay_physics import IdealReleaseHandoff
from unloading_sim.post_landing_transport import begin_ideal_transport, advance_ideal_transport
from unloading_sim.release_motion import predict_release, IDEAL_RECEPTION_RELEASE
from test_poc_release_reserves import runtime_metadata, carton, deck
from test_post_landing_transport import POLICY


def setup(nominal=.025, release=.024):
    metadata = runtime_metadata()
    metadata['release_prediction'] = predict_release(carton(nominal), [deck()], mode=IDEAL_RECEPTION_RELEASE)
    context = dict(world_id='new-world', task_id='task-1', receiver='belt',
                   transport_policy={**POLICY, 'reception_mode':'ideal'})
    gate = IdealReleaseHandoff(metadata, **context)
    audit = gate.accept_release(time_s=10., position=carton(release).center, rotation=np.eye(3),
        linear_velocity=[0,0,0], angular_velocity=[0,0,0], current_cartons=[])
    return metadata, context, gate, audit


def transition(metadata, context, gate, release=.024, takeover=.0208, **overrides):
    dt = np.sqrt(2*(release-takeover)/9.81)
    args = dict(**context, target='box', time_s=10.+dt, joint_present=False,
        position=carton(takeover).center, rotation=np.eye(3), linear_velocity=[0,0,-9.81*dt],
        angular_velocity=[0,0,0], current_cartons=[])
    args.update(overrides)
    return gate.audit_takeover(metadata, **args)


def removed(gate):
    gate.confirm_independence(time_s=10.01, joint_present=False, translation_m=.0021,
        rotation_rad=0., minimum_translation_m=.002, minimum_rotation_rad=.01)


def begin(gate, audit, policy):
    pose=np.asarray(audit['current_pose_world'])
    return begin_ideal_transport(OBB(pose[:3,3],carton().half_extents,pose[:3,:3],'box','carton'),
        receiver_name='belt', receivers={'belt':deck()}, directions={'belt':[-1,0,0]},
        time_s=audit['time_s'], policy=policy, attachment_removed=True, top_contact_observed=False,
        support_geometry_accepted=False, expected_target='box', actual_attachment_observed=True,
        released_handoff=gate)


@pytest.mark.parametrize('nominal,release,takeover',[(.022,.0215,.0195),(.025,.024,.0208),(.025,.025,.023)])
def test_first_takeover_after_fall_consumes_actual_release_receipt(nominal,release,takeover):
    m,c,g,pre=setup(nominal,release)
    assert pre['accepted']
    removed(g)
    audit=transition(m,c,g,release,takeover)
    assert audit['accepted'],audit
    before=g.evidence()
    record=begin(g,audit,c['transport_policy'])
    assert record['reception_region']['gap_m']==pytest.approx(takeover)
    assert before['release']['audit']['height_m']==pytest.approx(release)
    assert before['takeover']['first_takeover_height_m']==pytest.approx(takeover)
    assert begin(g,audit,c['transport_policy']) is record
    advance_ideal_transport(record,dt_s=.05,speed_m_s=.3)
    assert record['pose_world'][2][3] < .770
    assert begin(g,audit,c['transport_policy']) is record
    assert g.evidence()['release']==before['release']
    assert not record['actual_top_contact_observed']


@pytest.mark.parametrize('nominal,height',[(.022,.019999),(.049,.050001)])
def test_invalid_pre_release_cannot_create_receipt_or_release(nominal,height):
    m,c,g,audit=setup(nominal,height)
    assert not audit['accepted'] and g.evidence()['release'] is None
    with pytest.raises(ValueError):removed(g)
    assert not transition(m,c,g)['accepted']


def test_no_release_evidence_and_still_attached_fail_closed():
    m,c,g,_=setup()
    empty=IdealReleaseHandoff(m,**c)
    assert not transition(m,c,empty,release=.024,takeover=.0195)['accepted']
    assert not transition(m,c,g)['accepted']
    with pytest.raises(ValueError):
        g.confirm_independence(time_s=10.01,joint_present=True,translation_m=.01,rotation_rad=0.,
            minimum_translation_m=.002,minimum_rotation_rad=.01)
    removed(g)
    assert not transition(m,c,g,joint_present=True)['accepted']
    with pytest.raises(ValueError):
        begin_ideal_transport(carton(.0195),receiver_name='belt',receivers={'belt':deck()},
            directions={'belt':[-1,0,0]},time_s=10.,policy=c['transport_policy'],attachment_removed=True,
            top_contact_observed=False,support_geometry_accepted=False,expected_target='box',actual_attachment_observed=True)


@pytest.mark.parametrize('override',[dict(target='wrong'),dict(receiver='other'),dict(world_id='old-world'),
    dict(task_id='old-task'),dict(transport_policy={**POLICY,'reception_mode':'physical'}),
    dict(time_s=10.3),dict(time_s=9.9),dict(linear_velocity=[0,0,-4]),
    dict(position=[2,0,.77]),dict(position=[float('nan'),0,.77])])
def test_context_expiration_and_abnormal_measurements_rejected(override):
    m,c,g,_=setup();removed(g)
    assert not transition(m,c,g,**override)['accepted']


def test_changed_scene_policy_and_unbound_takeover_pose_rejected():
    m,c,g,_=setup();removed(g)
    altered=deepcopy(m);altered['release_prediction']['policy']['position_uncertainty_m']=.03
    assert not transition(altered,c,g)['accepted']
    good=transition(m,c,g)
    tampered=deepcopy(good);tampered['current_pose_world'][0][3]+=.001
    with pytest.raises(ValueError):begin(g,tampered,c['transport_policy'])
    leaked=g.evidence();leaked['release']['audit']['accepted']=False
    assert g.evidence()['release']['audit']['accepted']


def test_current_obstacle_and_receiver_penetration_rejected():
    m,c,g,_=setup()
    # The obstacle was absent at release but its actual current pose occupies the descent.
    obstacle=dict(name='neighbor',center_m=[3,0,.8],size_m=[.02,.02,.02],rotation_matrix=np.eye(3).tolist(),dynamic=True)
    m['scene_primitives'].append(obstacle)
    g=IdealReleaseHandoff(m,**c)
    current=[dict(name='neighbor',center_m=[3,0,.8],quaternion_wxyz=[1,0,0,0],linear_velocity_m_s=[0,0,0],angular_velocity_rad_s=[0,0,0])]
    assert g.accept_release(time_s=10,position=carton(.024).center,rotation=np.eye(3),linear_velocity=[0,0,0],angular_velocity=[0,0,0],current_cartons=current)['accepted']
    removed(g);current[0]['center_m']=[0,0,.77]
    rejected=transition(m,c,g,current_cartons=current)
    assert rejected['reason']=='IDEAL_HANDOFF_ENVELOPE_COLLISION'
    assert rejected['pair']==['box','neighbor']
    m,c,g,_=setup();removed(g)
    assert transition(m,c,g,takeover=-.001)['reason']=='IDEAL_HANDOFF_REGION_OR_PENETRATION'
