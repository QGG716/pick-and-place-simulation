from copy import deepcopy
import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('exporter',Path(__file__).parents[1]/'tools/export_m710_search_replay.py')
exporter=importlib.util.module_from_spec(spec);spec.loader.exec_module(exporter)

def fixture():
    q=[0.]*6;g=[.1]*6
    identity=dict(task_id='task',stage_id='stage',parent_stage_id='',request_id='3')
    request=dict(identity,q_start=q,q_goal=g,world=[],attachment=None)
    planner=dict(nodes=[dict(id=0,q=q,tree='START',root=True),dict(id=1,q=g,tree='GOAL',root=True)],
        edges=[[0,1]],join=[0,1],planner_data_vertices=2,planner_data_edges=1,
        events=[[0,0,False,q,g,0,1,'REJECTED']],checked_samples=[[g,False,None]],total_extensions=1)
    return dict(request=request,response_identity=identity,trace=dict(input=request,planners=[planner]),
        final_timed_path=[dict(q=q,t=0.),dict(q=g,t=1.)])

def test_export_preserves_rejected_events_and_input():
    a=fixture();before=deepcopy(a);assert exporter.validate_attempt(a)['qualification_status']=='NOT_EVALUATED'
    assert a==before

@pytest.mark.parametrize('bad',['request','count','edge','cycle','event','time','start','attachment'])
def test_export_rejects_mixed_or_corrupt_evidence(bad):
    a=fixture();p=a['trace']['planners'][0]
    if bad=='request':a['response_identity']=dict(a['response_identity'],request_id='old')
    if bad=='count':p['planner_data_vertices']=9
    if bad=='edge':p['edges']=[[0,2]]
    if bad=='cycle':p['edges']=[[0,0]];p['join']=None
    if bad=='event':p['events'][0][2]=True
    if bad=='time':a['final_timed_path'][1]['t']=-1
    if bad=='start':a['final_timed_path'][0]['q']=[2.]*6
    if bad=='attachment':a['request']['attachment']={'id':'box'};a['request']['world']=[{'id':'box'}]
    with pytest.raises(AssertionError):exporter.validate_attempt(a)

def test_missing_search_is_explicit():
    a=fixture();a['trace']=None;a['history_status']='UNAVAILABLE_INTERRUPTED_REQUEST'
    result=exporter.validate_attempt(a)
    assert result['status']!='PASS_TRACE_STRUCTURE_ONLY'
