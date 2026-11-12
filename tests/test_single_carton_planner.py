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
            if r['op']=='ik':return options.get('ik',lambda r:dict(status='NATIVE_IK_FAILED'))(r)
            if r['op']=='validate' and 'validate' in options and 'probe_path' not in r:
                hooked=options['validate'](r)
                if hooked is not None:return hooked
            if r['op']=='validate':
                valid=not((options['reject_return'] and r['stage']=='return' and 'probe_path' in r) or
                          (options['reject_loaded_start'] and r['stage']=='return' and 'probe_path' not in r))
                failure=None if valid else {'reason':'PAYLOAD_NEIGHBOR_COLLISION'}
                return dict(native_valid=valid,legacy_intersection_valid=valid,path_failure=failure,failure=failure)
            if 'plan' in options:
                hooked=options['plan'](r)
                if hooked is not None:return hooked
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


def test_lin_failure_retries_slower_native_profiles_and_records_each_attempt(case):
    p,obs,box,folder,calls,opts=case
    attempts=[]
    class Native:
        def request(self,req,**kwargs):
            if req['op']=='validate':return dict(native_valid=True,legacy_intersection_valid=True,path_failure=None,failure=None)
            assert req['pipeline_id']=='pilz_industrial_motion_planner' and req['planner_id']=='LIN'
            attempts.append((req['velocity_scale'],req['acceleration_scale']))
            if req['velocity_scale']>.25:return dict(status='NATIVE_PLANNING_FAILED')
            return dict(status='SUCCESS',native_output_status='PASS',points=[dict(t=0.,q=[0.]*6,v=[0.]*6,a=[0.]*6),dict(t=1.,q=[.02]*6,v=[0.]*6,a=[0.]*6)])
    p.client=Native();progress=[]
    segment=p.checked_segment({'q_start':[0.]*6,'stage':'contact'},np.eye(4),'contact',10.,folder,progress.append,True)
    assert attempts==[(1.,1.),(.5,.5),(.25,.25)]
    assert segment['evidence']['planner']==dict(pipeline_id='pilz_industrial_motion_planner',planner_id='LIN',
        velocity_scale=.25,acceleration_scale=.25,goal='POSE',configured_time_scale=10.,send_time_scale=10.)
    for name in ('contact-pilz','contact-pilz-s0.5','contact-pilz-s0.25'):
        assert json.loads((folder/f'{name}-native.json').read_text())['status']=='REPLIED'
    assert not (folder/'contact-ompl-native.json').exists() and p.authority._plans=={}
    assert sum('重新原生规划' in m for m in progress)==2


def test_lin_never_falls_back_to_free_space_search(case):
    p,obs,box,folder,calls,opts=case
    class Native:
        def request(self,req,**kwargs):
            assert req['planner_id']=='LIN'
            return dict(status='NATIVE_PLANNING_FAILED')
    p.client=Native()
    with pytest.raises(ValueError,match='contact:NATIVE_REJECTED'):
        p.checked_segment({'q_start':[0.]*6,'stage':'contact'},np.eye(4),'contact',10.,folder,lambda _:None,True)
    assert not list(folder.glob('*ompl*'))


def test_endpoint_screen_skips_blocked_tool_and_plans_clear_candidate_first(case,monkeypatch):
    p,obs,box,folder,calls,opts=case
    real=contact_candidates;cands=real(box,p.source['tool'],p.site['topology'])[:3]
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates',lambda *a:deepcopy(cands))
    p.clearance={'tool_links':['part_000']}
    poses={json.dumps(np.round(c[k],9).tolist()):i for i,c in enumerate(cands) for k in ('precontact_pose','tcp_pose')}
    def index(r):return poses[json.dumps(np.round(r['goal_pose'],9).tolist())]
    solution=lambda i:[.011+i/1000]*6  # small moves within the 0.1 rad/s limit at time_scale 1
    def ik(r):
        i=index(r)
        if i==1:return dict(status='NATIVE_IK_FAILED')
        return dict(status='SUCCESS',q=solution(i))
    def validate(r):
        if r['stage']=='return':return None
        if r['q_start']==solution(0):
            failure=dict(reason='INTERSECTION_OR_CONTACT',pair=['part_000','wall'],surface_distance_m=-1.)
            return dict(native_valid=False,legacy_intersection_valid=False,failure=failure,path_failure=None)
        return dict(native_valid=True,legacy_intersection_valid=True,failure=None,path_failure=None)
    opts['ik']=ik;opts['validate']=validate
    p.coordinate_model.tcp=lambda q:np.asarray(cands[2]['tcp_pose']) if q[0]>.01 else np.eye(4)
    progress=[]
    plan=p.plan_cycle(obs,[box],folder,progress.append)
    assert plan['candidate_index']==2 and plan['endpoint_screen']=='CLEAR'
    motions=[c for c in calls if c.get('op')=='plan']
    # Blocked candidate 0 never reaches path search; CLEAR candidate 2 goes
    # before the non-conclusive candidate 1 and uses its checked joint goal.
    assert motions[0]['stage']=='approach' and motions[0]['q_goal']==solution(2) and 'goal_pose' not in motions[0]
    assert len(motions)==4 and len(p.authority._plans)==4
    screen=json.loads((folder/(box['id']+'-0')/'endpoint-precheck.json').read_text())
    assert screen[0]['conclusion']=='TOOL_OR_PAYLOAD_POSE_BLOCKED'
    assert json.loads((folder/(box['id']+'-1')/'endpoint-precheck.json').read_text())[0]['conclusion']=='IK_NOT_FOUND_NOT_CONCLUSIVE'
    assert [s['conclusion'] for s in json.loads((folder/(box['id']+'-2')/'endpoint-precheck.json').read_text())]==['PASS']*3
    assert any('1 个候选终态全部通过，1 个逆解未定论，1 个已排除' in m for m in progress)
    assert plan['segments'][0]['package']['evidence']['planner']['goal']=='JOINT'


def test_ik_seeds_are_deterministic_current_task_solutions_within_limits(case):
    p,obs,box,folder,calls,opts=case
    seeds=p.ik_seeds([0.]*6,[[.1]*6,[.2,.2,.2,.2,.2,2.9]])
    assert seeds[0]==[0.]*6 and seeds[1]==[.2,.2,.2,.2,.2,2.9]
    assert all(-3<=x[5]<=3 for x in seeds) and len(seeds)==len({tuple(np.round(x,12)) for x in seeds})
    assert [.1]*5+[.1+np.pi/2] in [list(np.round(x,12)) for x in seeds] or any(np.allclose(x,[.1]*5+[.1+np.pi/2]) for x in seeds)


def test_detach_falls_back_to_lift_profiles_when_outward_line_is_blocked(case,monkeypatch):
    p,obs,box,folder,calls,opts=case
    real=contact_candidates
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates',lambda *a:real(*a)[:1])
    candidate=real(box,p.source['tool'],p.site['topology'])[0]
    tcp=np.asarray(candidate['tcp_pose']);normal=np.asarray(candidate['face']['normal'])
    seen=[]
    def plan(r):
        if r['stage']!='detach':return None
        goal=np.asarray(r['goal_pose'])[:3,3]-tcp[:3,3];seen.append(goal.round(6).tolist())
        if goal@normal>.1:return dict(status="NATIVE_PLANNING_FAILED")  # straight outward line blocked
        return None
    opts['plan']=plan
    result=p.plan_cycle(obs,[box],folder)
    detach=result['segments'][2]
    assert detach['stage']=='detach' and detach['package']['evidence']['detach_profile']=='DIAGONAL_45'
    rejected=detach['package']['evidence']['rejected_detach_profiles']
    assert [r['profile'] for r in rejected]==['OUTWARD'] and 'NATIVE_REJECTED' in rejected[0]['reason']
    # Outward attempt at full and three slower LIN profiles, then one 45-degree plan.
    assert len(seen)==5
    np.testing.assert_allclose(seen[-1],normal*.04+np.array([0,0,.04]),atol=1e-6)
    sub=next(folder.glob('*-0'))
    assert (sub/'detach-pilz-native.json').is_file() and (sub/'detach-diagonal_45-pilz-native.json').is_file()
    assert (sub/'detach-diagonal_45-sent-check.json').is_file() and len(p.authority._plans)==4


def test_all_detach_profiles_rejected_issues_no_permit(case,monkeypatch):
    p,obs,box,folder,calls,opts=case
    real=contact_candidates
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates',lambda *a:real(*a)[:1])
    opts['plan']=lambda r:dict(status='NATIVE_PLANNING_FAILED') if r['stage']=='detach' else None
    with pytest.raises(ValueError,match='NO_COMPLETE_FEASIBLE_SINGLE_CARTON_PLAN: detach × 1'):p.plan_cycle(obs,[box],folder)
    assert p.authority._plans=={} and not (folder/'complete-plan.json').exists()
    reason=json.loads((folder/'planning-failures.json').read_text())[0]['reason']
    assert 'ALL_PROFILES_REJECTED' in reason and all(name in reason for name in ('OUTWARD','DIAGONAL_45','LIFT_FIRST'))


# Quintic rest-to-rest peaks: speed 1.875*d/T (limit 0.1), acceleration 5.7735*d/T^2 (limit 0.2).
@pytest.mark.parametrize('delta,expected',[(.02,1.),(.05,1.25),(.1,2.),(.2,None)])
def test_send_scale_slows_only_as_needed_to_keep_interpolated_limits(case,delta,expected):
    p,obs,box,folder,calls,opts=case
    class Native:
        def request(self,req,**kwargs):
            if req['op']=='validate':return dict(native_valid=True,legacy_intersection_valid=True,path_failure=None,failure=None)
            return dict(status='SUCCESS',native_output_status='PASS',points=[dict(t=0.,q=[0.]*6,v=[0.]*6,a=[0.]*6),dict(t=1.,q=[delta]*6,v=[0.]*6,a=[0.]*6)])
    p.client=Native();progress=[]
    if expected is None:
        with pytest.raises(ValueError,match='INTERPOLATION_LIMIT'):
            p.checked_segment({'q_start':[0.]*6,'stage':'approach'},[delta]*6,'approach',1.,folder,progress.append)
        assert not (folder/'approach-sent-native.json').exists()
        return
    segment=p.checked_segment({'q_start':[0.]*6,'stage':'approach'},[delta]*6,'approach',1.,folder,progress.append)
    assert segment['prepared']['time_scale']==expected
    assert segment['evidence']['planner']['configured_time_scale']==1. and segment['evidence']['planner']['send_time_scale']==expected
    q=np.asarray([x['q'] for x in segment['prepared']['points']]);dt=segment['prepared']['period_s']
    assert (np.abs(np.diff(q,axis=0))/dt).max()<=.1+1e-9 and (np.abs(np.diff(q,2,axis=0))/dt**2).max()<=.2+1e-6
    from unloading_sim.workbench.single_carton.planner import SEND_SLOWDOWN_STEPS
    assert sum('插值后超出关节限值' in m for m in progress)==SEND_SLOWDOWN_STEPS.index(expected)
