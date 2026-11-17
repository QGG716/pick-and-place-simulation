import importlib.util,json,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from test_single_carton_planner import case
from test_single_carton_bench import bench,span


def test_replay_ten_seconds_already_includes_two_second_tail():
    samples=bench.replay_samples('task',0,dict(complete_plan=True),dict(events=[span('replay.planning_entry_to_search_end',10),span('plan_cycle.binding',2,start=8)]),'log')
    assert samples[0][1]['seconds']==10
    assert samples[0][1]['scope']=='PARTIAL_REPLAY_NOT_PREPARE_PLAN'


def test_handoff_delay_increases_scheduling_total_without_double_subtraction():
    from unloading_sim.workbench.single_carton.scheduling_metrics import handoffs
    events=[];motion=[]
    for i in range(4):
        events.append(dict(kind='motion_task',segment=i,motion_task_id=str(i)))
        motion.append([dict(kind='send',sent_s=10*i),dict(kind='send',sent_s=10*i+1),dict(kind='result',state='SUCCEEDED',task_id=str(i),arrival_rule_satisfied_s=10*i+1.4)])
    events.extend([dict(kind='pressure_rule_satisfied',phase='grip',valve_completed_s=11.5,pressure_received_s=12.5),dict(kind='pressure_rule_satisfied',phase='release',valve_completed_s=31.5,pressure_received_s=32.5),dict(kind='stage',state='COMPLETE',host_monotonic_s=33)])
    a=handoffs(events,motion);before=bench.scheduling_values(3.5,a)
    for rows in motion[2:]:
        for row in rows:
            for key in ('sent_s','arrival_rule_satisfied_s'):
                if key in row:row[key]+=2
    for event in events:
        if event.get('phase')=='release':
            event['valve_completed_s']+=2;event['pressure_received_s']+=2
        if event.get('state')=='COMPLETE':event['host_monotonic_s']+=2
    after=bench.scheduling_values(3.5,handoffs(events,motion))
    assert after['seconds']==pytest.approx(before['seconds']+2)
    assert len(a)==4 and a[1]['required_wait_s']==pytest.approx(1.4)


def test_deadline_replay_executes_actual_branch_and_keeps_candidate_filter(case,monkeypatch,tmp_path):
    p,ob,box,folder,calls,opts=case
    spec=importlib.util.spec_from_file_location('replay_review',Path(__file__).resolve().parents[1]/'tools/replay_single_carton_record.py')
    r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
    config=dict(p.site,site={});box['quality']={};p.init['parameters']={'robot_description':'fake'}
    record=tmp_path/'record';record.mkdir();out=tmp_path/'replay'
    (record/'observation.json').write_text(json.dumps(dict(observation_id='r',T_world_camera=np.eye(4).tolist(),q_capture=[0.]*6,q_start=[0.]*6,intrinsics={},depth_scale_m=.001)))
    np.savez(record/'observation.npz',rgb=np.zeros((2,2,3)),depth=np.zeros((2,2)))
    np.savez(record/'segmentation.npz',mask_0=np.ones((2,2),bool))
    monkeypatch.setattr(r,'load',lambda *a:config)
    monkeypatch.setattr(r.Installation,'load',lambda *a:SimpleNamespace(world_from_base=np.eye(4),tool=None))
    monkeypatch.setattr(r,'CoordinateModel',lambda *a:p.coordinate_model)
    monkeypatch.setattr(r,'load_mapping',lambda *a:None)
    monkeypatch.setattr(r,'SinglePlanner',lambda *a:p)
    monkeypatch.setattr(r,'recover_boxes',lambda *a:([box],[]))
    monkeypatch.setattr(r,'RobotSelfFilter',lambda *a:None)
    monkeypatch.setattr(r,'build_scene',lambda *a:dict(world=[o for o in p.world if o.get('category')!='carton']))
    from unloading_sim import moveit2_backend
    monkeypatch.setattr(r,'ResidentMoveItClient',moveit2_backend.ResidentMoveItClient)
    monkeypatch.setattr(sys,'argv',['replay',str(record),'--output',str(out),'--candidates','0','--planning-deadline-s','30'])
    r.main()
    report=json.loads((out/'replay-report.json').read_text())
    assert report['complete_plan']['candidate_index']==0 and report['source_record_unchanged']
    assert not p.authority._plans and report['permits_issued']==0


def test_outcome_counts_keep_failed_cancelled_and_timeout_samples():
    section=dict(metric='planning',stages=[])
    samples=[dict(events=[],seconds=value,success=name=='SUCCESS',outcome=name) for name,value in
             [('SUCCESS',1),('FAILED',10),('CANCELLED',20),('PLANNING_BUDGET_EXHAUSTED',30)]]
    result=bench.summarize('planning',samples,section,1.5)
    assert result['samples']==4 and result['timed_samples']==4 and result['success_rate']==.25
    assert result['max_s']==30 and result['outcomes']==dict(SUCCESS=1,FAILED=1,CANCELLED=1,PLANNING_BUDGET_EXHAUSTED=1)
    empty=bench.summarize('planning',[],section,1.5)
    assert empty['status']=='NO_SAMPLES' and empty['samples']==0 and empty['success_rate'] is None


def test_partial_replay_cannot_claim_full_planning_target():
    sample=dict(events=[],seconds=1.,success=True,scope='PARTIAL_REPLAY_NOT_PREPARE_PLAN')
    result=bench.summarize('planning',[sample],dict(metric='planning',stages=[],total_s=10),1.5)
    assert result['status']=='PARTIAL_SCOPE' and result['meets_total_p95'] is None
