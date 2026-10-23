"""Directed process boundaries; these tests are not native-cycle evidence."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.eco65.transverse_process import (freeze_box,process_options,release_poses,
    junction_projection,channel_blockers,projection_limits)
from unloading_sim.eco65.coupled_scene import make_coupled_spec,coupled_conveyor_state
from unloading_sim.eco65.coupled_state import J1_FREE_POLICY,check_business_direction
from unloading_sim.eco65.coupled_task import CoupledTask
from unloading_sim.eco65.native_contact import FACE_AXES,check_contours
from test_eco65_coupled_scene import source_scene,vertices


def box(bottom=.48,y=0.,name='target'):
    return dict(id=name,pose=[.205,y,bottom+.05],size_m=[.15,.1,.1],supported_by=['floor'],supports=[])


@pytest.fixture
def spec():return make_coupled_spec(source_scene())


@pytest.mark.parametrize('bottom,expected',[(.099,True),(.100,False),(.101,False)])
def test_top_gate_uses_frozen_eight_corner_bottom_not_low_receive(bottom,expected,spec):
    b=box(bottom);f=freeze_box(b,[b])
    assert f['top_allowed']==expected
    result=process_options(b,[b],[b],spec,coupled_conveyor_state(spec,.18)['receiving'])
    assert any(o['face']=='top' for o in result['options'])==expected
    if not expected:
        assert not result['options']  # Neither high nor low may be clamped.
        assert all(r['reason']=='PROCESS_RECEIVE_HEIGHT_OUT_OF_RANGE' for r in result['rejected'])


def test_rotated_box_and_common_ground_datum():
    b=box();p=np.eye(4);p[:3,3]=[0.,0.,.19]
    a=np.pi/4;p[:3,:3]=[[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]]
    b['pose']=p.tolist()
    f=freeze_box(b,[b],.03)
    assert f['z_b_m']==pytest.approx(.08928932188134525)
    assert f['vertical_height_m']==pytest.approx(.1414213562373095)
    assert f['top_allowed']


def test_low_receiver_keeps_original_release_height_and_full_box(spec):
    b=box();receiver=coupled_conveyor_state(spec,.18)['receiving']
    options=process_options(b,[b],[b],spec,receiver)['options']
    assert [o['height_m'] for o in options]==pytest.approx([.43,.33])
    assert [o['expected_drop_m'] for o in options]==pytest.approx([.05,.15])
    for o in options:
        goals=list(release_poses(b,o,[b]));assert len(goals)>3
        assert all(g[2,3]==pytest.approx(.53) for g in goals)
        assert all(-.095-1e-9<=g[0,3]<=-.065+1e-9 for g in goals)
        assert all(g[1,3]==0 and np.array_equal(g[:3,:3],np.eye(3)) for g in goals)


def test_junction_uses_full_projection_and_low_only(spec):
    b=box(y=-.06);receiver=coupled_conveyor_state(spec,.18)['receiving']
    p=np.eye(4);p[:3,3]=[-.08,-.06,.53]
    assert junction_projection(b,p,spec,receiver)['junction']  # Centre lies beyond inlet Y=-.1.
    rows=process_options(b,[b],[b],spec,receiver)['options']
    assert [r['template'] for r in rows]==['FRONT_JUNCTION_LOW']
    assert rows[0]['height_m']==pytest.approx(.33)


def test_original_endpoint_and_closed_lateral_channel(spec):
    right,middle,left=box(y=-.11,name='right'),box(name='middle'),box(y=.11,name='left')
    original=[right,middle,left];receiver=coupled_conveyor_state(spec,.18)['receiving']
    blocked=process_options(left,original,original,spec,receiver)
    assert not blocked['options']
    assert any('middle' in r.get('blocker_ids',[]) for r in blocked['rejected'])
    opened=process_options(left,original,[right,left],spec,receiver)
    assert {'FRONT_LATERAL_LOW','LEFT_INNER_LOW'}<={o['template'] for o in opened['options']}
    assert freeze_box(middle,original)['original_role']=='MIDDLE'
    assert 'RIGHT_INNER_HIGH' in {o['template'] for o in process_options(right,original,[right,left],spec,receiver)['options']}


def test_lowest_mechanism_rigid_parts_ground_and_fixed_feet(spec):
    assert spec['low_height_m']==.100 and spec['high_height_m']==.450
    states=[coupled_conveyor_state(spec,h) for h in (.1,.18,.33,.43,.45)]
    assert states[0]['tilt_rad']<0
    assert states[0]['mechanism']['lower_endpoint_D_m'][2]==.18
    assert states[0]['mechanism']['stacks'][0]['stage_rise_m']==pytest.approx(.013)
    reference={r['id']:r for r in states[0]['world_objects']}
    for state in states:
        m=state['mechanism'];upper,lower=np.asarray(m['upper_endpoint_D_m']),np.asarray(m['lower_endpoint_D_m'])
        assert np.linalg.norm(upper-lower)==pytest.approx(.4443197215708027)
        for row in state['world_objects']:
            assert vertices(row)[:,2].min()>=-1e-9,row['id']
            if row['id'] in m['fixed_structure_ids']:assert row==reference[row['id']]
            assert row.get('size')==reference[row['id']].get('size')
        for stack in m['stacks']:
            for bar in stack['bars']:assert np.linalg.norm(np.diff(bar['endpoints_m'],axis=0))==pytest.approx(bar['length_m'])


@pytest.mark.parametrize('interval',['FIRST_APPROACH','OUTBOUND','RETURN'])
def test_raw_j1_both_directions_and_actual_reversal(interval):
    q=np.zeros((5,6));q[:,0]=[-2.,1.8,-1.9,2.1,.2]
    assert check_business_direction(q,interval,policy=J1_FREE_POLICY)==0


def test_attachment_requires_synced_receiver_and_lift_forbidden_after_attach():
    task=object.__new__(CoupledTask);task.height_m=.18;task.process_option={'height_m':.43}
    with pytest.raises(ValueError,match='EXTRACTION_BEFORE'):task.require_receive_height()
    task.height_m=.43;task.require_receive_height()
    task.attachment=np.eye(4)
    with pytest.raises(ValueError,match='SYNC_AFTER_ATTACHMENT'):task.lift(.33)


def test_contact_budget_does_not_extend_task_or_renew_failed_approach(monkeypatch):
    from unloading_sim.eco65 import coupled_task
    monkeypatch.setattr(coupled_task,'perf_counter',lambda:100.)
    task=object.__new__(CoupledTask);task.connection_deadline=101.
    task.deadline=150.;task.connection_seconds=80.;task.started=0.;task.counter=4
    task.target={'id':'target'};task.phase_budgets={};task.sync_location='CONTACT'
    task.process_option={'height_m':.43};seen=[]
    task.move=lambda *a,**kw:seen.append(('contact',task.connection_deadline))
    task.lift=lambda h:seen.append(('lift',task.connection_deadline))
    task.seal=lambda:{'status':'PASS'}
    task.finish_contact(np.eye(4),None)
    assert seen==[('contact',150.),('lift',150.)]
    assert task.connection_deadline==101. and task.deadline==150.


def test_native_world_avoids_unrelated_scene_copy_without_changing_solids():
    from unloading_sim.eco65.coupled_scene import build_coupled_scene,with_coupled_state,native_coupled_world
    from unloading_sim.eco65.native_scene import world_objects
    scene=build_coupled_scene(source_scene());original=deepcopy(scene)
    for height in (.1,.18,.33,.43,.45):
        native=native_coupled_world(scene,height)
        assert native==world_objects(with_coupled_state(scene,height))
        native[0]['pose'][0][3]+=99.
        assert scene==original


def test_lift_preparation_timeout_does_not_commit_a_sample_height():
    task=object.__new__(CoupledTask);task.height_m=.18;task.attachment=None;task.current_q=np.zeros(6)
    task.checkpoint=lambda:{'height_m':task.height_m}
    task.restore=lambda cp:setattr(task,'height_m',cp['height_m'])
    task.require_connection_budget=lambda:None
    task.request=lambda *a:{}
    def state():
        if task.height_m>.18:raise ValueError('simulated preparation timeout')
        return {}
    task.state=state
    with pytest.raises(ValueError,match='preparation timeout'):task.lift(.23)
    assert task.height_m==.18
