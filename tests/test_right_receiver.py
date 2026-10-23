"""TEST INJECTION: controlled native replies exercise the real two-leg coordinator."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from test_receiver_candidate_acceptance import receiver_fixture
from test_lifted_reception import box
from unloading_sim.single_right_receiver import SingleRightReceiverConnector, right_inner_targets, rectangle_samples
from unloading_sim.side_transfer import SideTransferTracker
from unloading_sim.lifted_reception import translated, layer_role
from unloading_sim.contact_target import simulation_contact_targets
from unloading_sim.support import SupportRelationGraph
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.moveit2_backend import MoveItUnavailable
from unloading_sim.geometry import OBB, rotation_matrix_from_rpy
from unloading_sim.single_contact import SingleContactClient
from test_single_extraction import extraction_request
import unloading_sim.single_right_receiver as side
import unloading_sim.single_lifted_receiver as lifted


def side_fixture():
    c,t,q,world,plan,row,calls,checks,records,_=receiver_fixture()
    c.__class__=SingleRightReceiverConnector
    t=replace(t,outward_normal=np.array([0.,1.,0.]),face_role='SIDE')
    c.region_candidates=lambda *a:iter([np.array([-.5102,.15,t.box.center[2]]),np.array([-.5898,.15,t.box.center[2]])])
    prefix=[dict(stage='pregrasp',stage_id='pregrasp',parent_stage_id='',task_id='task'),
            dict(stage='contact',stage_id='contact',parent_stage_id='pregrasp',task_id='task')]
    def chain(state):
        name=state.native_receipt['stage_id'];tail=[]
        while name!='contact':
            if name not in records:raise MoveItUnavailable('TEST_INJECTION_PARENT_IDENTITY')
            r=records[name];tail.insert(0,r);name=r['parent_stage_id']
        return [*prefix,*tail]
    c._native_selected_records=chain
    class State(np.ndarray):
        @property
        def native_receipt(self):return self.receipt
    def motion(start,goal,obstacles,**kw):
        calls.append(dict(start=start,history=deepcopy(kw['initial_proximity'].evidence()),substage=c._side_substage,**kw))
        assert not kw['allow_ompl'] and not kw['try_ptp'] and not kw['try_prior']
        end=np.r_[kw['goal_pose'][:3,3],np.zeros(3)].view(State)
        stage='extract-'+str(len(calls));end.receipt=dict(task_id='task',stage_id=stage)
        path=[start,(np.asarray(start)+end)/2,end]
        history=kw['initial_proximity'].clone()
        for value in path:assert history.state_failure(kw['attachment'].box_at(value),obstacles,kw['support_names']) is None
        record=dict(stage='extraction',stage_id=stage,parent_stage_id=start.native_receipt['stage_id'],task_id='task',
            status='SUCCESS',request_id=len(calls),points=[dict(q=np.asarray(v).tolist(),t=float(i)) for i,v in enumerate(path)],
            submitted_request=dict(ownership_transition='ATTACH' if start.native_receipt['stage_id']=='contact' else None),
            process_policy=dict(terminal_proximity=history.evidence()))
        records[stage]=record
        return path,None,dict(attempts=[record])
    c._native_plan=motion
    return c,t,q,world,plan,row,calls,checks,records


def test_two_actual_legs_history_and_single_attach():
    c,t,q,w,p,r,calls,checks,records=side_fixture()
    out,r=c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert out and r['complete'] and len(calls)==2 and len(checks)==1
    assert [x['substage'] for x in calls]==['RIGHT_INNER_Y','RIGHT_INNER_X']
    assert calls[0]['start'] is q
    assert calls[1]['start'].native_receipt['stage_id']=='extract-1'
    assert calls[1]['history']==records['extract-1']['process_policy']['terminal_proximity']
    assert not calls[1]['history']['fully_released']  # first leg is legal, still supported
    assert len(out['stages'])==4 and out['stages'][-1]['parent_stage_id']=='extract-1'
    assert [x['substage'] for x in out['extraction_segments']]==['RIGHT_INNER_Y','RIGHT_INNER_X']
    assert not out['extraction_segments'][0]['sample_check']['final_separation_required']
    assert len([e for e in out['events'] if e['event_type']=='ATTACH'])==1
    assert out['events'][-1]['next_stage_id']=='extract-1'


@pytest.mark.parametrize('failure',['separation','region'])
def test_second_success_postcheck_failure_continues_and_reuses_turn(monkeypatch,failure):
    c,t,q,w,p,r,calls,checks,records=side_fixture()
    if failure=='separation':
        endpoint=c.native.request
        def reject(req,**kw):
            result=endpoint(req,**kw)
            if len(checks)==1:result.update(status='EXTRACTION_END_CLEARANCE_REJECTED',failure=dict(reason='TEST_INJECTED_END',pair=['held','wall']))
            return result
        c.native.request=reject
    else:
        fn=lifted.release_ready_geometry;count=[]
        def reject(*a,**kw):
            result=fn(*a,**kw);count.append(1)
            if len(count)==1:result['passed']=False
            return result
        monkeypatch.setattr(lifted,'release_ready_geometry',reject)
    out,r=c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert out and len(calls)==3 and len(checks)==2
    assert calls[1]['start'] is calls[2]['start']
    assert calls[1]['history']==calls[2]['history']
    a,b=r['region_attempts'];assert not a['selected'] and b['selected']
    assert a['native_result']['status']=='SUCCESS' and a['acceptance_status']=='POSTCHECK_REJECTED'
    assert b['segments'][0]['reused'] and out['stages'][-1]['stage_id']=='extract-3'
    assert len(out['events'])==1


def test_first_leg_postcheck_failure_restarts_root_tracker(monkeypatch):
    c,t,q,w,p,r,calls,checks,records=side_fixture();original=side.extraction_sample_check;seen=[]
    def rejected(*args,**kw):
        samples=original(*args,**kw);seen.append(1)
        if len(seen)==1:samples.update(passed=False,failure=dict(reason='EXTRACTION_RETURNED_SAMPLE_PAYLOAD_REJECTED',
            detail=dict(reason='TEST_INJECTED_FIRST_MIDDLE_COLLISION'),sample=1))
        return samples
    monkeypatch.setattr(side,'extraction_sample_check',rejected)
    out,r=c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert out and len(calls)==3 and len(checks)==1
    assert calls[0]['start'] is calls[1]['start'] is q
    assert calls[0]['history']==calls[1]['history']
    assert r['region_attempts'][0]['acceptance_status']=='POSTCHECK_REJECTED'
    assert out['stages'][-2]['stage_id']=='extract-2'


@pytest.mark.parametrize('bad',['history','parent','nonfinite','cancel','deadline'])
def test_request_corruption_or_late_first_leg_stops(bad):
    c,t,q,w,p,r,calls,checks,records=side_fixture();original=c._native_plan
    def corrupt(*a,**kw):
        result=original(*a,**kw)
        if bad=='history':result[2]['attempts'][-1]['process_policy']['terminal_proximity']['fully_released']=True
        elif bad=='parent':result[2]['attempts'][-1]['parent_stage_id']='other'
        elif bad=='nonfinite':result[0][1][0]=np.nan
        elif bad=='cancel':c._native_cancelled=lambda:True
        else:c._single_connection_deadline=0.
        return result
    c._native_plan=corrupt
    if bad=='deadline':
        out,r=c.complete_contact_plan(p,r,q,t,w,{},71070);assert out is None
    else:
        with pytest.raises(MoveItUnavailable):c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert len(calls)==1 and not checks and not r['region_attempts'][0]['selected']


def test_planar_transfer_reentry_and_side_collision():
    c,*_=side_fixture();target=box((.3,-.8,2.25),name='held')
    supports=[box((.3,-.8,1.95),name='under'),box((.3,-.38,1.95),name='next')]
    tracker=SideTransferTracker(target,supports,c.collision_policy,.005,.0002)
    other=translated(target,[0,.42,0]);assert tracker.state_failure(other,supports) is None
    assert not tracker.fully_released and 'under' in tracker.released and 'next' in tracker.touched
    assert tracker.state_failure(target,supports)['reason']=='SIDE_TRANSFER_RELEASED_PAIR_REENTRY'
    assert tracker.state_failure(translated(other,[0,0,-.002]),supports)['reason']=='SIDE_TRANSFER_SUPPORT_PENETRATION'
    obstacle=box((.3,-.38,2.25),name='wall_neighbor')
    t=SideTransferTracker(target,[*supports,obstacle],c.collision_policy,.005,.0002)
    assert t.state_failure(other,[*supports,obstacle])['reason']=='SIDE_TRANSFER_NONPLANAR_CLEARANCE'


def test_original_geometry_role_and_unopened_channel():
    layout=load_workcell_layout(Path(__file__).resolve().parents[1]/'configs/workcells/m710id70_unloading_layout_v1.yaml')
    boxes=layout.cartons();top=max(b.center[2] for b in boxes);row=sorted([b for b in boxes if b.center[2]==top],key=lambda b:b.center[1]);target=row[0]
    def generated(remaining):
        scene=SimpleNamespace(support_graph=SupportRelationGraph(remaining,[]),snapshot={'scene_fingerprint':'test'},
            policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}))
        return simulation_contact_targets(scene,target,[-1,0,0],.1,side_exposure_depth_m=.4)[0]
    selected,proof=right_inner_targets(generated(boxes),target,boxes,boxes)
    assert not selected and proof['availability']=='INNER_FACE_BLOCKED_OR_DISABLED'
    removed={b.name for b in row[1:-1]};remaining=[b for b in boxes if b.name not in removed]
    selected,proof=right_inner_targets(generated(remaining),target,boxes,remaining)
    assert len(selected)==1 and selected[0].geometric_face_id=='y+'
    np.testing.assert_array_equal(selected[0].contact_pose[:3,2],[0,-1,0])
    assert proof['role']['removed_from_original_layer']==[b.name for b in row[1:-1]]
    assert len(remaining)==37 and row[-1].name in {b.name for b in remaining}


def test_region_samples_not_display_midpoint_or_width_quota():
    stream=rectangle_samples([-.59,-.51,-.18,.51],.0001)
    values=[next(stream) for _ in range(30)]
    assert values[0]==pytest.approx((-.5102,-.1798))
    assert len(set(values))==30 and len({y for _,y in values})>2


def test_rotated_box_resolves_actual_inner_face():
    target=box((0,-.8,2.25),name='renamed_right',rotation=rotation_matrix_from_rpy(0,0,np.pi))
    left=box((0,.8,2.25),name='renamed_left');original=[left,target]
    scene=SimpleNamespace(support_graph=SupportRelationGraph(original,[]),snapshot={'scene_fingerprint':'test'},
        policy=SimpleNamespace(data={'task_population':{'face_modes':['front','top','side']}}))
    targets,_=simulation_contact_targets(scene,target,[-1,0,0],.1,side_exposure_depth_m=.4)
    selected,proof=right_inner_targets(targets,target,original,original)
    assert len(selected)==1 and selected[0].geometric_face_id=='y-'
    np.testing.assert_allclose(selected[0].contact_pose[:3,2],[0,-1,0],atol=1e-12)
    assert proof['role']['role']=='RIGHT_END'


@pytest.mark.parametrize('bad',['disabled','root','wrong_transition','wrong_leg','unknown_template'])
def test_side_protocol_scope(bad):
    r=extraction_request();r.update(receiver_template='RIGHT_INNER_HMAX',process_substage='RIGHT_INNER_Y')
    client=SingleContactClient(SimpleNamespace(request=lambda *a,**k:pytest.fail('must not dispatch')),
        lifted_receiver_binding={'target_id':'box'},right_inner_enabled=bad!='disabled')
    if bad=='root':r['parent_stage_id']=''
    elif bad=='wrong_transition':r['ownership_transition']=None
    elif bad=='wrong_leg':r['process_substage']='RIGHT_INNER_X'
    elif bad=='unknown_template':r['receiver_template']='HMIN'
    with pytest.raises(MoveItUnavailable,match='RIGHT_INNER'):client.request(r)


def test_second_solver_incomplete_reuses_valid_first_without_restarting_contact():
    c,t,q,w,p,r,calls,checks,records=side_fixture();original=c._native_plan
    def incomplete(*a,**kw):
        result=original(*a,**kw)
        if len(calls)==2:
            result[2]['attempts'][-1]['status']='NATIVE_PLANNING_TIMEOUT'
            return [],dict(reason='NATIVE_PLANNING_TIMEOUT'),result[2]
        return result
    c._native_plan=incomplete
    out,r=c.complete_contact_plan(p,r,q,t,w,{},71070)
    assert out and len(calls)==3 and len(checks)==1
    assert calls[1]['start'] is calls[2]['start']
    assert r['region_attempts'][0]['acceptance_status']=='NATIVE_METHOD_NOT_CONNECTED'
    assert not r['region_attempts'][0]['selected']
