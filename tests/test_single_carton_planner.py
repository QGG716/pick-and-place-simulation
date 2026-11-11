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
    calls=[];options=dict(reject_return=False)
    class Native:
        def __init__(self,*args,**kwargs):pass
        def request(self,r,**kwargs):
            calls.append(deepcopy(r))
            if r==p.init:return dict(status='READY')
            if r['op']=='validate':
                valid=not(options['reject_return'] and r['stage']=='return' and 'probe_path' in r)
                return dict(native_valid=valid,legacy_intersection_valid=valid,path_failure=None if valid else {'reason':'PAYLOAD_NEIGHBOR_COLLISION'})
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
    assert len(motions[1]['allowed_pairs'])==7
    assert len(p.authority._plans)==4
    assert len([c for c in calls if c.get('op')=='validate' and 'probe_path' in c])==4
    inertia=np.asarray(plan['binding']['physical_cycle']['payload_mass_properties']['inertia_box_kg_m2'])
    np.testing.assert_allclose(np.diag(inertia),[.0033333333333333,.0054166666666667,.0054166666666667],atol=1e-12)
    assert plan['release']['status']=='RECEIVER_QUALIFICATION_DEFERRED_BY_USER'

def test_loaded_return_collision_rejects_whole_task_without_any_permit(case,monkeypatch):
    p,obs,box,folder,calls,opts=case;opts['reject_return']=True
    # One concrete candidate keeps this directed failure test bounded.
    real=contact_candidates
    monkeypatch.setattr('unloading_sim.workbench.single_carton.planner.contact_candidates',lambda *a:real(*a)[:1])
    with pytest.raises(ValueError,match='NO_COMPLETE_FEASIBLE'):p.plan_cycle(obs,[box],folder)
    assert p.authority._plans=={} and not (folder/'complete-plan.json').exists()
    assert any(c.get('stage')=='return' and c.get('attachment') for c in calls)


def test_unknown_requested_target_is_not_substituted(case):
    p,obs,box,folder,calls,opts=case
    with pytest.raises(ValueError,match='NO_COMPLETE_FEASIBLE'):p.plan_cycle(obs,[box],folder,selected=999)
    assert not [c for c in calls if c.get('op')=='plan']
