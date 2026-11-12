"""Fake native response contract tests; no SDK, no physical acceptance."""
import json
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.workbench.controlled.trajectory import Authority,Limits
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.single_carton.geometry import corners,contact_candidates
from unloading_sim.workbench.single_carton.config import load

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def case(tmp_path,monkeypatch):
    config=load(ROOT/'configs/workbench/single_carton.example.json')
    # Retained pre-site native chain fixture explicitly uses its historical policy.
    config.pop('site');config['release_policy']='USER_DEFERRED_RECEIVER_QUALIFICATION'
    tool=json.loads((ROOT/'configs/local/eco65_desktop/tool.json').read_text())
    T=np.eye(4);T[:3,3]=[.4,0,.4]
    box=dict(id='real-observation-box-0',detection_index=0,pose=T.tolist(),size_m=[.15,.1,.1],corners_m=corners(T,[.15,.1,.1]).tolist(),
        faces=[dict(observed=True,center_m=[.4,0,.45],normal=[0,0,1],u=[1,0,0],v=[0,1,0],size_m=[.15,.1])])
    candidate=contact_candidates(box,tool,config['topology'])[0]
    class Model:
        world_from_base=np.eye(4)
        def __init__(self):self.tool=SimpleNamespace(matrix=np.asarray(tool['T_flange_tcp']))
        def tcp(self,q):return np.asarray(candidate['tcp_pose']) if q[0]>.01 else np.eye(4)
        def frames(self,q):return dict(link_6=self.tcp(q)@np.linalg.inv(self.tool.matrix))
    class Planner(SinglePlanner):
        def build(self):pass
        def binding(self):return dict(physical_cycle=self.cycle_binding,version='FAKE_NATIVE_TEST')
        def receiver(self):return dict(ids=[],lower=[-2,-2,-2],upper=[2,2,-1.9],margin=.01)
    p=Planner(config,None,Authority('eco65_single_carton_segment_v1'),Model())
    p.source=dict(tool=tool);p.world=[dict(id='wall',pose=np.eye(4).tolist(),size=[.1,.1,.1],category='environment'),dict(id='old-simulation-box',pose=T.tolist(),size=[.15,.1,.1],category='carton')]
    p.contact_geometry=dict(terminal_band_by_contour={s['id']:s['component']+'_lip' for s in tool['seals']})
    p.init=dict(identity=dict(flange_from_task_tcp=tool['T_flange_tcp']))
    p.clearance={};p.limits=Limits((-3.,)*6,(3.,)*6,(.1,)*6,(.2,)*6);p.reach_bound=1.
    calls=[];options=dict(reject_return=False,reject_loaded_start=False,legacy_worker=False)
    class Native:
        def __init__(self,*args,**kwargs):pass
        def request(self,r,**kwargs):
            calls.append(deepcopy(r))
            if r==p.init:return dict(status='READY',capabilities={'bounded_single_carton_planning':not options['legacy_worker']})
            if r['op']=='validate':
                valid=not((options['reject_return'] and r['stage']=='return' and 'probe_path' in r) or
                          (options['reject_loaded_start'] and r['stage']=='return' and 'probe_path' not in r))
                failure=None if valid else {'reason':'PAYLOAD_NEIGHBOR_COLLISION'}
                return dict(native_valid=valid,legacy_intersection_valid=valid,path_failure=failure,failure=failure)
            stage=r['stage'];qgoal=r.get('q_goal',[dict(approach=.01,contact=.02,detach=.03)[stage]]*6) if stage!='return' else r['q_goal']
            return dict(status='SUCCESS',native_output_status='PASS',points=[dict(t=0.,q=r['q_start'],v=[0.]*6,a=[0.]*6),dict(t=1.,q=qgoal,v=[0.]*6,a=[0.]*6)])
        def close(self):pass
    monkeypatch.setattr('unloading_sim.moveit2_backend.ResidentMoveItClient',Native)
    folder=tmp_path/('a'*32);folder.mkdir()
    from unloading_sim.workbench.contracts import Stamp
    observation=SimpleNamespace(q_start=(0.,)*6,world_tcp_start=np.eye(4).tolist(),
        frame=SimpleNamespace(stamp=Stamp.receive()),robot_sample={'source':'LIVE'},
        metadata=lambda:dict(source='FAKE_DEVICE_TEST_NOT_HARDWARE_EVIDENCE'))
    return p,observation,box,folder,calls,options

def test_full_loaded_chain_validated_before_any_permit(case):
    p,obs,box,folder,calls,opts=case
    plan=p.plan_cycle(obs,[box],folder)
    assert plan['q_start']==[0.]*6 and plan['full_path_validated']
    assert [s['stage'] for s in plan['segments']]==['approach','contact','detach','return']
    motions=[c for c in calls if c.get('op')=='plan']
    assert len(motions)==4
    assert motions[0]['attachment'] is None and motions[1]['attachment'] is None
    assert motions[2]['attachment']==motions[3]['attachment'] and motions[3]['q_goal']==[0.]*6
    assert motions[2]['planner_id']=='LIN' and motions[3]['planner_id']=='PTP'
    assert {w['id'] for w in motions[0]['world']}=={'wall',box['id']}
    assert motions[0]['allowed_pairs']==[['base_link','robot_mount_plate']]
    assert all(pair[1].endswith('_lip') for pair in motions[1]['allowed_pairs'][1:])
    assert len(motions[1]['allowed_pairs'])==13  # mount + all 12 terminal lips, never rigid cups
    spec=motions[1]['single_carton_contact']
    assert len(spec['active_ids'])==6 and len(spec['target_lip_contact_ids'])==12
    lips=[p.contact_geometry['terminal_band_by_contour'][i] for i in spec['target_lip_contact_ids']]
    assert motions[1]['allowed_pairs'][1:]==[[box['id'],lip] for lip in lips]
    assert motions[2]['attachment']['touch_links']==lips
    recorded=json.loads(next(folder.glob('*/loaded-start-check.json')).read_text())
    assert recorded['result']['native_valid'] and recorded['request']['stage']=='return'
    assert len(p.authority._plans)==4
    assert len([c for c in calls if c.get('op')=='validate' and 'probe_path' in c])==4
    inertia=np.asarray(plan['binding']['physical_cycle']['payload_mass_properties']['inertia_box_kg_m2'])
    np.testing.assert_allclose(np.diag(inertia),[.0033333333333333,.0054166666666667,.0054166666666667],atol=1e-12)
    assert plan['release']['status']=='RECEIVER_QUALIFICATION_DEFERRED_BY_USER'


def test_current_real_site_scene_flows_through_all_four_native_stages(case):
    p,obs,box,folder,calls,opts=case
    candidate=contact_candidates(box,p.source['tool'],p.site['topology'])[0]
    flange=np.asarray(candidate['tcp_pose'])@np.linalg.inv(p.coordinate_model.tool.matrix)
    attachment=np.linalg.inv(flange)@np.asarray(box['pose'])
    returned=p.coordinate_model.frames(obs.q_start)['link_6']@attachment
    bounds=corners(returned,box['size_m']);lo=bounds.min(0);hi=bounds.max(0)
    top=float(lo[2]);support=np.eye(4);support[:3,3]=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,top-.02]
    table=dict(id='field_measured_table',size=[hi[0]-lo[0]+.1,hi[1]-lo[1]+.1,.04],pose=support.tolist(),category='measured_fixed')
    p.site['release_policy']='MEASURED_SITE_RECEIVER_REQUIRED'
    p.site['site']=dict(workspace_bounds_m=[[-2.,-2.,-2.],[2.,2.,2.]],receiver=dict(
        support_ids=[table['id']],bounds_m=[[lo[0]-.01,lo[1]-.01,top-.04],[hi[0]+.01,hi[1]+.01,top]],
        margin_m=.003,measurement_basis='SYNTHETIC_REAL_SITE_ADAPTER_TEST'))
    p.receiver=lambda:SinglePlanner.receiver(p)
    obs.frame.observation_id='frozen-current-exposure'
    scene=dict(observation_id=obs.frame.observation_id,world=[table],imported_simulation_objects=0)
    plan=p.plan_cycle(obs,[box],folder,scene=scene)
    assert plan['release']['status']=='PASS' and plan['binding']['physical_cycle']['site_scene']==scene
    for r in calls:
        if r.get('op') in {'plan','validate'}:
            assert {o['id'] for o in r['world']}=={box['id'],table['id']}
            assert r['site_workspace_bounds_m']==[[-2.,-2.,-2.],[2.,2.,2.]]
    assert plan['segments'][-1]['package']['prepared']['points'][-1]['q']==pytest.approx(obs.q_start,abs=1e-12)


@pytest.mark.parametrize('failure',[None,'loaded_start','loaded_return'])
def test_user_deferred_real_site_keeps_full_world_and_loaded_checks(case,monkeypatch,failure):
    p,obs,box,folder,calls,opts=case
    p.site['site']=dict(workspace_bounds_m=[[-2.,-2.,-2.],[2.,2.,2.]],receiver=None)
    obs.frame.observation_id='frozen-site-frame'
    world=[dict(id=ident,size=[.1,.2,.3],pose=np.eye(4).tolist(),category=kind)
           for ident,kind in [('neighbor','carton'),('blind-region','unknown'),('actual-table','measured_fixed')]]
    scene=dict(observation_id=obs.frame.observation_id,world=world,imported_simulation_objects=0)
    def no_receiver():raise AssertionError('No receiver measurement may be invented for the deferred policy')
    p.receiver=no_receiver
    real=contact_candidates
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates',lambda *a:real(*a)[:1])
    opts['reject_loaded_start']=failure=='loaded_start';opts['reject_return']=failure=='loaded_return'
    if failure:
        with pytest.raises(ValueError,match='PAYLOAD_NEIGHBOR_COLLISION'):p.plan_cycle(obs,[box],folder,scene=scene)
        assert not p.authority._plans and not (folder/'complete-plan.json').exists()
    else:
        plan=p.plan_cycle(obs,[box],folder,scene=scene)
        assert plan['release']['status']=='RECEIVER_QUALIFICATION_DEFERRED_BY_USER'
        assert plan['release']['support_confirmed'] is False and plan['release']['region']['ids']==[]
        assert plan['release']['qualification']['basis']==p.site['receiver_qualification_basis']
        assert plan['release']['q_return']==list(obs.q_start)
        assert plan['binding']['physical_cycle']['site_scene']==scene
        assert len(p.authority._plans)==4
        assert len([r for r in calls if r.get('op')=='validate' and 'probe_path' in r])==4
    checked=[r for r in calls if r.get('op') in {'plan','validate'}]
    assert checked
    for r in checked:
        assert {o['id'] for o in r['world']}=={'neighbor','blind-region','actual-table',box['id']}
        assert not any(pair[1] in {'neighbor','blind-region','actual-table'} for pair in r['allowed_pairs'])
        if r['stage']=='return':
            assert r['attachment']['size']==box['size_m'] and r['single_carton_contact']['supports']==[]
    assert scene['world']==world
    if failure=='loaded_start':assert not [r for r in calls if r.get('op')=='plan']


def test_explicit_measured_receiver_failure_is_once_before_native_candidates(case):
    p,obs,box,folder,calls,opts=case
    p.site['release_policy']='MEASURED_SITE_RECEIVER_REQUIRED';p.site['site']={'receiver':None}
    p.receiver=lambda:SinglePlanner.receiver(p)
    with pytest.raises(ValueError,match='SITE_RECEIVER_NOT_CONFIGURED') as error:p.plan_cycle(obs,[box],folder)
    assert 'NO_COMPLETE_FEASIBLE' not in str(error.value) and calls==[] and not p.authority._plans


@pytest.mark.parametrize('policy,basis,expected',[
    ('USER_DEFERRED_RECEIVER_QUALIFICATION','', 'RECEIVER_DEFERRAL_BASIS_REQUIRED'),
    ('UNKNOWN',None,'INVALID_RELEASE_POLICY'),(None,None,'INVALID_RELEASE_POLICY')])
def test_receiver_deferral_is_explicit_not_a_missing_configuration_fallback(case,policy,basis,expected):
    from unloading_sim.workbench.single_carton.config import issues
    p,obs,box,folder,calls,opts=case
    p.site['release_policy']=policy;p.site['receiver_qualification_basis']=basis
    assert any(expected in error for error in issues(p.site))
    with pytest.raises(ValueError,match=expected):p.plan_cycle(obs,[box],folder)
    assert calls==[] and not p.authority._plans


@pytest.mark.parametrize('category',['unknown','observed'])
def test_native_unknown_region_is_distinguished_from_observed_collision(category):
    T=np.eye(4);T[:3,3]=[-.06,-.06,.59]
    world=[dict(id='region',category=category,size=[.04,.04,1.48],pose=T.tolist())]
    before=deepcopy(world)
    result={'native_valid':False,'failure':dict(reason='CLEARANCE_INSUFFICIENT',pair=['base_link','region'],
        surface_distance_m=.0016305,required_pair_clearance_m=.0024)}
    text=SinglePlanner.loaded_start_failure(result,world)
    if category=='unknown':
        assert text.startswith('LOADED_Q_START_UNKNOWN_SPACE') and 'base_link' in text
        assert '[[-0.08, -0.08, -0.15], [-0.04, -0.04, 1.33]]' in text
        assert '现场测量' in text and '尚不能确认' in text
    else:assert text.startswith('LOADED_Q_START_COLLISION')
    assert world==before and result['native_valid'] is False

def test_loaded_return_collision_rejects_whole_task_without_any_permit(case,monkeypatch):
    p,obs,box,folder,calls,opts=case;opts['reject_return']=True
    # One concrete candidate keeps this directed failure test bounded.
    real=contact_candidates
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates',lambda *a:real(*a)[:1])
    with pytest.raises(ValueError,match='NO_COMPLETE_FEASIBLE'):p.plan_cycle(obs,[box],folder)
    assert p.authority._plans=={} and not (folder/'complete-plan.json').exists()
    assert any(c.get('stage')=='return' and c.get('attachment') for c in calls)
    failures=json.loads((folder/'planning-failures.json').read_text())
    assert failures[0]['candidate']==0 and 'PAYLOAD_NEIGHBOR_COLLISION' in failures[0]['reason']


def test_unknown_requested_target_is_not_substituted(case):
    p,obs,box,folder,calls,opts=case
    with pytest.raises(ValueError,match='NO_COMPLETE_FEASIBLE'):p.plan_cycle(obs,[box],folder,selected=999)
    assert not [c for c in calls if c.get('op')=='plan']


def test_loaded_start_rejections_are_recorded_and_summarized_without_motion_plans(case):
    p,obs,box,folder,calls,opts=case;opts['reject_loaded_start']=True
    with pytest.raises(ValueError,match='NO_COMPLETE_FEASIBLE') as error:
        p.plan_cycle(obs,[box],folder)
    assert 'LOADED_Q_START_COLLISION × 32' in str(error.value)
    assert len(str(error.value))<1000 and 'PAYLOAD_NEIGHBOR_COLLISION' in str(error.value)
    failures=json.loads((folder/'planning-failures.json').read_text())
    assert len(failures)==32 and {f['candidate'] for f in failures}==set(range(32))
    assert len(list(folder.glob('*/loaded-start-check.json')))==32
    assert not [c for c in calls if c.get('op')=='plan'] and p.authority._plans=={}


def test_final_sample_collision_bound_includes_actual_large_payload_extent(case):
    from unloading_sim.moveit2_backend import ResidentMoveItClient
    p,obs,box,folder,calls,opts=case
    p.client=ResidentMoveItClient()
    pose=np.eye(4);pose[2,3]=.4
    context={'q_start':[0.]*6,'stage':'detach',
             'attachment':dict(pose=pose.tolist(),size=[.06,.08,.1])}
    segment=p.checked_segment(context,[.03]*6,'detach',10.,folder,lambda *a:None)
    assert segment['prepared']['collision_sampling']['reach_bound_m']==pytest.approx(1.452769256906871)
    assert calls[-1]['op']=='validate' and calls[-1]['probe_path']


def test_old_worker_cannot_silently_use_unbounded_physical_planning(case):
    p,obs,box,folder,calls,opts=case;opts['legacy_worker']=True
    with pytest.raises(ValueError,match='NATIVE_WORKER_UPDATE_REQUIRED'):
        p.plan_cycle(obs,[box],folder)
    assert not [r for r in calls if r.get('op')=='plan'] and p.authority._plans=={}


def test_bounded_solver_failure_keeps_both_attempts_and_does_not_issue_permit(case):
    p,obs,box,folder,calls,opts=case
    progress=[];cancel=lambda:False
    class Native:
        def request(self,req,**kwargs):
            name='pilz' if req['pipeline_id']=='pilz_industrial_motion_planner' else 'ompl'
            record=json.loads((folder/f'approach-{name}-native.json').read_text())
            assert record['status']=='PENDING' and record['request']==req
            assert req['request_budget_s']==110. and kwargs['timeout']==120.
            assert kwargs['cancelled'] is cancel
            return dict(status='NATIVE_PLANNING_FAILED' if name=='pilz' else 'NATIVE_SOLVER_BUDGET_EXHAUSTED')
    p.client=Native()
    with pytest.raises(ValueError,match='NATIVE_SOLVER_BUDGET_EXHAUSTED'):
        p.checked_segment({'q_start':[0.]*6},[.1]*6,'approach',10.,folder,progress.append,cancelled=cancel)
    for name in ('pilz','ompl'):
        record=json.loads((folder/f'approach-{name}-native.json').read_text())
        assert record['status']=='REPLIED' and record['elapsed_s']>=0
    assert any('避让路径' in msg for msg in progress) and p.authority._plans=={}


@pytest.mark.parametrize('reason',['MOVEIT2_REQUEST_TIMEOUT','CANCELLED'])
def test_request_failure_retains_input_elapsed_and_stage_without_retry(case,reason):
    p,obs,box,folder,calls,opts=case
    class Native:
        count=0
        def request(self,req,**kwargs):
            self.count+=1
            raise RuntimeError(reason)
    p.client=Native()
    with pytest.raises(RuntimeError,match='approach-pilz:'+reason):
        p.checked_segment({'q_start':[0.]*6},[.1]*6,'approach',10.,folder,lambda _:None)
    record=json.loads((folder/'approach-pilz-native.json').read_text())
    assert record['status']=='REQUEST_FAILED' and record['error']==reason
    assert record['request']['q_goal']==[.1]*6 and record['elapsed_s']>=0
    assert p.client.count==1 and p.authority._plans=={}
