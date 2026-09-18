"""Finite production orchestration with explicitly substituted CPU-only inference/ROS."""
import json
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import pytest
import yaml
from unloading_perception.finite_sequence import (read_json,atomic_json,manifest_groups,
    verify_capture,verify_result)
from workcell_once_fakes import capture_fixture,install

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from run_finite_workcell_sequence import run_sequence


class SyntheticDelivery:
    """CPU runner test only; actual DDS is separately covered by ROS tests."""
    def __init__(self): self.calls=[]
    def start(self): self.calls.append('start')
    def await_receipt(self,request):
        self.calls.append(request['task_id'])
        return {**request,'status':'ROS_ACCEPTED','test_substitute':'NO_DDS_IN_CPU_TEST'}
    def close(self): self.calls.append('close')


def setup_groups(tmp_path):
    groups=[]
    for i,name in enumerate(('A','B','C')):
        root=tmp_path/name
        root.mkdir()
        capture,models,modules=capture_fixture(root,('normal','normal'),stamp=10.125+i,frame=7+i,pixel=32+i)
        groups.append({'task_id':name,'capture':str(capture),'action':'RUN'})
    return groups,read_json(models),modules


def run(tmp_path,monkeypatch,groups,models):
    config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text(encoding='utf-8'))
    output=tmp_path/'batch'
    output.mkdir()
    atomic_json(output/'models.json',models)
    delivery=SyntheticDelivery()
    calls=[]
    def inference(command,**kwargs):
        # Only the external model/worker implementations are replaced.
        capture=Path(command[command.index('--capture')+1])
        with monkeypatch.context() as scope:
            install(scope,capture)
            code=runpy.run_path(str(ROOT/'tools/run_workcell_perception_once.py'))['main'](command[2:])
        calls.append(capture)
        return SimpleNamespace(returncode=code)
    code=run_sequence(groups,output,models=models,config=config,algorithm_python=sys.executable,
                      vision=tmp_path,delivery=delivery,run_algorithm=inference)
    return code,read_json(output/'progress.json'),calls,delivery


def test_real_orchestration_continues_after_invalid_group_and_counts_calls(tmp_path,monkeypatch):
    groups,models,modules=setup_groups(tmp_path)
    (Path(groups[1]['capture'])/'FULL_STACK_NOMINAL/modules'/modules[1]/'sensor_rgb.png').write_bytes(b'corrupt')
    code,report,calls,delivery=run(tmp_path,monkeypatch,groups,models)
    assert code==report['exit_code']==1
    assert [r['status'] for r in report['groups']]==['ROS_ACCEPTED','FAILED','ROS_ACCEPTED']
    assert report['displayed_task']=='C'
    assert [r['algorithm_attempts'] for r in report['groups']]==[1,0,1]
    assert [r['sam_attempts'] for r in report['groups']]==[2,0,2]
    assert [r['metric_attempts'] for r in report['groups']]==[2,0,2]
    assert report['groups'][1]['failure_stage']=='capture_validation'
    assert len(calls)==2 and delivery.calls==['start','A','C','close']
    assert not (tmp_path/'batch/B/algorithm/algorithm_artifact.json').exists()


def test_verified_reuse_checks_inputs_models_config_without_new_inference(tmp_path,monkeypatch):
    groups,models,_=setup_groups(tmp_path)
    code,report,_,_=run(tmp_path,monkeypatch,groups[:1],models)
    assert code==0
    first=report['groups'][0]
    again=tmp_path/'again'
    again.mkdir()
    reused={**groups[0],'action':'REUSE_VERIFIED','reuse_summary':first['summary_path']}
    code,new,calls,_=run(again,monkeypatch,[reused],models)
    assert code==0 and not calls
    assert new['groups'][0]['result_origin']=='REUSED_VERIFIED'
    assert new['groups'][0]['sam_attempts']==new['groups'][0]['metric_attempts']==0
    assert new['groups'][0]['artifact']==first['artifact']
    inputs=verify_capture(groups[0]['capture'])
    config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text(encoding='utf-8'))
    with pytest.raises(ValueError,match='model/config'):
        verify_result(first['summary_path'],inputs,{'changed':True},config)
    with pytest.raises(ValueError,match='model/config'):
        verify_result(first['summary_path'],inputs,models,{'changed':True})
    with pytest.raises(ValueError,match='input differs'):
        verify_result(first['summary_path'],verify_capture(groups[2]['capture']),models,config)


def test_single_cli_failure_preserves_calls_and_continues_to_next_group(tmp_path,monkeypatch):
    groups,models,modules=setup_groups(tmp_path)
    atomic_json(Path(groups[1]['capture'])/'test-control.json',
                {modules[0]:'sam_error',modules[1]:'normal'})
    code,report,calls,delivery=run(tmp_path,monkeypatch,groups,models)
    assert code==report['exit_code']==1
    assert [r['status'] for r in report['groups']]==['ROS_ACCEPTED','FAILED','ROS_ACCEPTED']
    failed=report['groups'][1]
    assert failed['failure_stage']=='algorithm'
    assert failed['algorithm_attempts']==1 and failed['sam_attempts']==2 and failed['metric_attempts']==1
    assert failed['modules'][0]['status']=='TECHNICAL_FAILURE'
    assert failed['modules'][1]['status']=='COMPLETED_WITH_ALGORITHM_RESULTS'
    assert len(calls)==3 and delivery.calls==['start','A','C','close']
    assert not (tmp_path/'batch/B/algorithm/algorithm_artifact.json').exists()


def test_mismatched_upper_lower_and_duplicate_group_do_not_call_algorithm(tmp_path,monkeypatch):
    import shutil
    groups,models,modules=setup_groups(tmp_path)
    shutil.copyfile(Path(groups[0]['capture'])/'FULL_STACK_NOMINAL/modules'/modules[0]/'capture_binding.json',
                    Path(groups[1]['capture'])/'FULL_STACK_NOMINAL/modules'/modules[1]/'capture_binding.json')
    groups[2]['capture']=groups[0]['capture']
    code,report,calls,_=run(tmp_path,monkeypatch,groups,models)
    assert code==1 and len(calls)==1
    assert [r['algorithm_attempts'] for r in report['groups']]==[1,0,0]
    assert 'DUPLICATE_CAPTURE_GROUP' in report['groups'][2]['error']


@pytest.mark.parametrize('groups',[[],[{'task_id':'../bad','capture':'.','action':'RUN'}],
    [{'task_id':'A','capture':'.','action':'RUN'}]*2])
def test_manifest_is_finite_and_unambiguous(tmp_path,groups):
    path=tmp_path/'manifest.json'
    atomic_json(path,{'schema_version':'finite_capture_sequence_v1','sequence_kind':'INDEPENDENT_CAPTURE_GROUPS','groups':groups})
    with pytest.raises(ValueError): manifest_groups(path)
