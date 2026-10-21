"""TEST INJECTION: post-LIN failures are synthetic, never native-run evidence."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest

from test_single_extraction import suffix_fixture
from test_lifted_reception import box
from unloading_sim.single_lifted_receiver import SingleLiftedReceiverConnector
from unloading_sim.moveit2_backend import MoveItUnavailable
from unloading_sim.lifted_reception import translated
import unloading_sim.single_lifted_receiver as lifted
import unloading_sim.single_extraction as extraction


def receiver_fixture():
    c,target,initial,world,plan,row,_,checks=suffix_fixture()
    c.__class__=SingleLiftedReceiverConnector
    c.high_belt=box((-.55,0,target.box.center[2]-target.box.half_extents[2]-.11),
        (.35,.75,.06),'conveyor_transverse')
    world=[*world,c.high_belt]
    c.active_obstacles=world;c.native_scene.all_obstacles=world
    c.receiver_raised=True;c.receiver_events=[];c._single_connection_deadline=None
    c.native_identity={'test_injection':True};c.cancel_requested=False
    class State(np.ndarray):
        @property
        def native_receipt(self):return self.receipt
    def state(value,stage):
        result=np.array(value).view(State)
        result.receipt=dict(task_id='task',stage_id=stage)
        return result
    q=state(initial,'contact')
    plan['final_state_receipt']=q.native_receipt
    c._native_state_receipt=lambda s:SimpleNamespace(stage_id=s.native_receipt['stage_id'],
        evidence=lambda:deepcopy(s.native_receipt))
    prefix=[dict(stage='pregrasp',stage_id='pregrasp',parent_stage_id='',task_id='task'),
        dict(stage='contact',stage_id='contact',parent_stage_id='pregrasp',task_id='task')]
    generated={};calls=[];trackers=[]
    def motion(start,goal,obstacles,**kw):
        assert start is q and np.array_equal(start,initial)
        assert kw['attachment'].box_at(start).name==target.target_id
        assert not kw['initial_proximity'].fully_released
        calls.append(kw);trackers.append(kw['initial_proximity'])
        stage='extract-'+str(len(calls))
        end=state(np.r_[kw['goal_pose'][:3,3],np.zeros(3)],stage)
        path=[start,(start+end)/2,end]
        kw['initial_proximity']._update_released(kw['attachment'].box_at(end))
        record=dict(task_id='task',stage_id=stage,parent_stage_id='contact',stage='extraction',
            submitted_request=dict(ownership_transition='ATTACH'),
            status='SUCCESS',request_id=100+len(calls),points=[dict(q=np.asarray(s).tolist()) for s in path])
        generated[stage]=record
        return path,None,dict(attempts=[record])
    c._native_plan=motion
    c._native_selected_records=lambda s:[*prefix,generated[s.native_receipt['stage_id']]]
    c._native_parent_for_request=lambda s,r:(c._native_state_receipt(s),{},
        'ATTACH' if s.native_receipt['stage_id']=='contact' else None)
    def request(start,goal,obstacles,**kw):
        return dict(op='plan',q_start=np.asarray(start).tolist(),stage=kw['stage'],
            world=[b.name for b in obstacles],attachment=kw['attachment'].rigid.tcp_from_box.tolist(),
            initial_proximity=None if kw['initial_proximity'] is None else kw['initial_proximity'].evidence())
    c._build_native_request=request
    endpoint=c.native.request
    def checked(req,**kw):
        result=endpoint(req,**kw)
        result['parent_stage_id']=req['parent_stage_id']
        return result
    c.native.request=checked
    return c,target,q,world,plan,row,calls,checks,generated,trackers


def test_lin_success_then_end_rejection_continues_same_contact():
    c,t,q,world,plan,row,calls,checks,records,trackers=receiver_fixture()
    original=c.native.request
    def injected(req,**kw):
        result=original(req,**kw)
        if len(checks)==1:
            result.update(status='EXTRACTION_END_CLEARANCE_REJECTED',
                failure=dict(reason='TEST_INJECTED_PAIR_CLEARANCE',pair=['box','support']))
        return result
    c.native.request=injected
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result is not None and row['complete']
    assert len(calls)==len(checks)==2
    assert [a['selected'] for a in row['region_attempts']]==[False,True]
    assert set(records)=={'extract-1','extract-2'}
    assert result['stages'][-1]['stage_id']=='extract-2'
    assert result['stages'][-1]['parent_stage_id']=='contact'
    assert len([e for e in result['events'] if e['event_type']=='ATTACH'])==1
    assert len({id(t) for t in trackers})==2
    a,b=row['region_attempts']
    assert a['native_result']['status']==b['native_result']['status']=='SUCCESS'
    assert a['sample_check']['passed'] and a['terminal_separation']['status']=='EXTRACTION_END_CLEARANCE_REJECTED'
    assert a['release_ready_check']['status']=='NOT_REACHED'
    assert a['acceptance_status']=='POSTCHECK_REJECTED' and b['acceptance_status']=='FINAL_SELECTED'
    assert a['stage_id']!=b['stage_id'] and a['parent_stage_id']==b['parent_stage_id']=='contact'
    assert all(c['attachment'] is calls[0]['attachment'] for c in calls)


@pytest.mark.parametrize('bad',['projection','gap','orientation'])
def test_release_ready_rejection_tries_another_sample(monkeypatch,bad):
    c,t,q,world,plan,row,calls,checks,*_=receiver_fixture()
    original=lifted.release_ready_geometry;seen=[]
    def injected(payload,belt,**kw):
        seen.append(payload)
        if len(seen)==1:
            if bad=='projection':payload=translated(payload,[.1,0,0])
            elif bad=='gap':payload=translated(payload,[0,0,-.01])
            else:kw['expected_rotation']=np.eye(3)*.99
        return original(payload,belt,**kw)
    monkeypatch.setattr(lifted,'release_ready_geometry',injected)
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result and len(calls)==len(checks)==len(seen)==2
    assert [a['selected'] for a in row['region_attempts']]==[False,True]
    assert row['region_attempts'][0]['failure']['reason']=='ACTUAL_RELEASE_REGION_NOT_REACHED'
    assert not row['region_attempts'][0]['release_ready_check']['passed']
    assert row['release_ready_check']['passed']
    assert len([e for e in result['events'] if e['event_type']=='ATTACH'])==1


def test_returned_history_rejection_does_not_pollute_next_tracker(monkeypatch):
    c,t,q,world,plan,row,calls,checks,records,trackers=receiver_fixture()
    sample_trackers=[];root=[]
    proximity=c._initial_proximity
    def capture(*a,**kw):
        value,failure=proximity(*a,**kw);root.append((value,deepcopy(value.evidence())))
        return value,failure
    c._initial_proximity=capture
    samples=extraction.extraction_sample_check
    def counted(c,path,attachment,target,geometry,tracker,*args):
        assert not tracker.fully_released
        sample_trackers.append(tracker)
        result=samples(c,path,attachment,target,geometry,tracker,*args)
        if len(sample_trackers)==1:
            # Explicit test injection after the real checker has evolved this
            # branch's tracker; never copied into a real planning run.
            result.update(passed=False,failure=dict(reason='EXTRACTION_RETURNED_SAMPLE_PAYLOAD_REJECTED',
                detail=dict(reason='TEST_INJECTED_ORDERED_HISTORY_REJECTION'),sample=1))
        return result
    monkeypatch.setattr(extraction,'extraction_sample_check',counted)
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result and len(calls)==len(sample_trackers)==2 and len(checks)==1
    assert not row['region_attempts'][0]['sample_check']['passed']
    assert row['region_attempts'][0]['failure']['reason']=='EXTRACTION_RETURNED_SAMPLE_PAYLOAD_REJECTED'
    assert len({id(x) for x in trackers+sample_trackers})==4
    assert root[0][0].evidence()==root[0][1]
    assert result['extraction'][0]==q.tolist() and result['stages'][-1]['stage_id']=='extract-2'


@pytest.mark.parametrize('bad',['parent','start','nonfinite','attachment','context','policy','cancel','task_deadline','unknown_check','missing_witness'])
def test_request_errors_stop_without_accepting_or_retrying(bad):
    c,t,q,world,plan,row,calls,checks,records,_=receiver_fixture()
    original=c._native_plan;endpoint=c.native.request
    c._native_cancelled=lambda:c.cancel_requested
    def corrupt(*args,**kw):
        path,failure,evidence=original(*args,**kw)
        if bad=='parent':evidence['attempts'][0]['parent_stage_id']='other'
        elif bad=='start':path[0]=np.asarray(q)+.01
        elif bad=='nonfinite':path[1][0]=np.nan
        elif bad=='attachment':kw['attachment'].rigid.tcp_from_box[0,3]+=.01
        elif bad=='policy':c.ik['position_tolerance_m']*=2
        elif bad=='context':c._native_parent_for_request=lambda *a:(_ for _ in ()).throw(MoveItUnavailable('SCENE_CONTEXT_CHANGED'))
        elif bad in {'cancel','task_deadline'}:c.cancel_requested=True
        return path,failure,evidence
    c._native_plan=corrupt
    def unknown(req,**kw):
        result=endpoint(req,**kw)
        if bad=='unknown_check':result['status']='NOT_EVALUATED'
        if bad=='missing_witness':result.update(status='EXTRACTION_END_CLEARANCE_REJECTED',failure=None)
        return result
    c.native.request=unknown
    with pytest.raises(MoveItUnavailable):
        c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert len(calls)==1 and not row['complete']
    assert not any(a['selected'] for a in row['region_attempts'])
    assert row['region_attempts'][0]['acceptance_status']=='REQUEST_STOPPED'
    assert c._single_connection_deadline is None


def test_unavailable_sample_evidence_is_request_error(monkeypatch):
    c,t,q,world,plan,row,calls,checks,*_=receiver_fixture()
    monkeypatch.setattr(extraction,'extraction_sample_check',lambda *a,**kw:dict(passed=None,failure=None))
    with pytest.raises(MoveItUnavailable,match='RESULT_UNKNOWN'):
        c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert len(calls)==1 and not checks and not row['region_attempts'][0]['selected']


def test_cancelled_prefix_never_dispatches_extraction():
    c,t,q,world,plan,row,calls,checks,*_=receiver_fixture()
    c._native_cancelled=lambda:True
    with pytest.raises(MoveItUnavailable,match='CANCELLED_OR_DEADLINE'):
        c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert not calls and not checks and not row['region_attempts']
    assert c._single_connection_deadline is None


@pytest.mark.parametrize('late_at',['native','end','release'])
def test_late_success_is_not_accepted_and_stage_budget_is_shared(monkeypatch,late_at):
    c,t,q,world,plan,row,calls,checks,*_=receiver_fixture()
    clock=[0.];c._single_face_started=0.
    monkeypatch.setattr(lifted,'perf_counter',lambda:clock[0])
    native=c._native_plan;endpoint=c.native.request;release=lifted.release_ready_geometry
    def motion(*args,**kw):
        result=native(*args,**kw)
        if late_at=='native':clock[0]=91.
        return result
    def checked(*args,**kw):
        result=endpoint(*args,**kw)
        if late_at=='end':clock[0]=91.
        return result
    def ready(*args,**kw):
        result=release(*args,**kw)
        if late_at=='release':clock[0]=91.
        return result
    c._native_plan=motion;c.native.request=checked
    monkeypatch.setattr(lifted,'release_ready_geometry',ready)
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result is None and not row['complete'] and len(calls)==1
    assert row['failure']['termination']=='STAGE_BUDGET_EXHAUSTED'
    assert not row['failure']['global_unreachable'] and not row['region_attempts'][0]['selected']
    assert row['region_attempts'][0]['native_result']['status']=='SUCCESS'
    assert c._single_connection_deadline is None


def test_first_candidate_passes_without_duplicate_checks_or_extra_candidates(monkeypatch):
    c,t,q,world,plan,row,calls,checks,*_=receiver_fixture()
    samples=extraction.extraction_sample_check;seen=[]
    def counted(*args,**kw):seen.append(1);return samples(*args,**kw)
    monkeypatch.setattr(extraction,'extraction_sample_check',counted)
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result and len(seen)==len(calls)==len(checks)==1
    assert len(row['region_attempts'])==1 and row['region_attempts'][0]['selected']
    assert len(result['events'])==1 and result['events'][0]['event_type']=='ATTACH'
    assert row['times']['extraction_sample_check_s']==row['region_attempts'][0]['extraction_sample_check_s']
    assert row['times']['region_search_s']>=row['region_attempts'][0]['candidate_total_s']


def test_timeout_is_incomplete_not_cached_unreachability(monkeypatch):
    c,t,q,world,plan,row,calls,checks,*_=receiver_fixture()
    clock=[0.];c._single_face_started=0.
    monkeypatch.setattr(lifted,'perf_counter',lambda:clock[0])
    original=c._native_plan;seen=[]
    def first_timeout(*args,**kw):
        seen.append(c._single_connection_deadline)
        if len(seen)==1:
            clock[0]=40.
            return [],dict(reason='NATIVE_PLANNING_TIMEOUT'),dict(attempts=[dict(status='NATIVE_PLANNING_TIMEOUT')])
        return original(*args,**kw)
    c._native_plan=first_timeout
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result and seen==[90.,90.]
    assert row['region_attempts'][0]['native_result']['status']=='NATIVE_PLANNING_TIMEOUT'
    assert row['region_attempts'][0]['failure']['scope']=='CANDIDATE_METHOD'
    assert [a['selected'] for a in row['region_attempts']]==[False,True]


def test_exploration_exhaustion_is_not_global_unreachability(monkeypatch):
    c,t,q,world,plan,row,calls,checks,*_=receiver_fixture()
    monkeypatch.setattr(lifted,'region_samples',lambda *a:iter([-.5102,-.5898]))
    original=c.native.request
    def reject(req,**kw):
        result=original(req,**kw)
        result.update(status='EXTRACTION_END_CLEARANCE_REJECTED',failure=dict(reason='TEST_INJECTED_CLEARANCE'))
        return result
    c.native.request=reject
    result,row=c.complete_contact_plan(plan,row,q,t,world,{},71070)
    assert result is None and not row['complete'] and len(calls)==2
    assert row['failure']['termination']=='DISCRETE_EXPLORATION_EXHAUSTED'
    assert row['failure']['global_unreachable'] is False
    assert not any(a['selected'] for a in row['region_attempts'])
