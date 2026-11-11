"""Generated-task routing, using synthetic references only for protocol tests.

No fixture is a physical installation qualification or a native planning run.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import check_s50_integration as admission_tool
import plan_s50_single_carton as plan_tool
from unloading_sim import configured_layout_trajectory as entry
from unloading_sim.robot_model_binding import RobotModelBinding
from unloading_sim.s50_task_model import generate_task_model,select_task_configuration,INPUT_CONFIG


@pytest.fixture(scope='module')
def generated_pair(tmp_path_factory):
    folder=tmp_path_factory.mktemp('SYNTHETIC-task-entry-selection')
    reference=folder/'SYNTHETIC-NOT-MANUFACTURER.txt'
    reference.write_text('SYNTHETIC TASK ENTRY ROUTING TEST; NOT S50 ENGINEERING DATA')
    data=yaml.safe_load((ROOT/INPUT_CONFIG).read_text(encoding='utf-8'))
    for index in range(6):
        data['controller_mapping']['joints'][f'J{index+1}']=dict(sign=-1 if index==1 else 1,
            offset_rad=-np.pi/2 if index==1 else .125,verified=True,
            source=dict(path=reference.name,sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),locator='Synthetic fixture only'))
    data['task_generation']['generated_config']='generated/model.yaml'
    data['kinematics']['mounting_from_base']=np.eye(4).tolist()
    source=folder/'input.yaml';source.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    report=generate_task_model(source,root=ROOT)
    assert not report['output_directory_overridden']
    assert Path(report['generated_config'])==folder/'generated/model.yaml'
    return SimpleNamespace(source=source,config=Path(report['generated_config']),report=report)


def test_default_missing_model_uses_input_only_to_report_gaps():
    selected=select_task_configuration(root=ROOT)
    assert selected['requested_config_role']=='generator_input'
    assert selected['selected_task_config'] is None
    assert selected['status']=='GENERATED_TASK_MODEL_NOT_READY'
    assert selected['audit_config']==str((ROOT/INPUT_CONFIG).resolve())
    assert 'task.GENERATED_TASK_CONFIG_FILE_MISSING' in selected['missing']
    assert selected['generator_input_used_for_task_execution'] is False


def test_input_and_explicit_generated_paths_select_the_same_identity(generated_pair):
    pair=generated_pair
    via_input=select_task_configuration(pair.source,root=ROOT)
    direct=select_task_configuration(pair.config,root=ROOT)
    for selected in [via_input,direct]:
        assert selected['selected_task_config']==str(pair.config)
        assert selected['audit_config']==str(pair.config)
        assert selected['input_config']==str(pair.source)
        assert selected['task_definition_sha256']==pair.report['task_definition_sha256']
        assert selected['missing']==[]
    assert via_input['requested_config_role']=='generator_input'
    assert direct['requested_config_role']=='generated_task_model'


def test_admission_loads_generated_binding_even_when_input_path_was_requested(generated_pair):
    pair=generated_pair;report=admission_tool.audit(pair.source)
    assert report['effective_config']==str(pair.config)
    assert report['model_binding']['model_definition_sha256']==pair.report['task_definition_sha256']
    assert report['config_sha256']==hashlib.sha256(pair.config.read_bytes()).hexdigest()
    # The routing works; missing real installation/collision evidence still gates planning.
    assert report['status']=='MODEL_NOT_READY'
    assert not report['native_worker_launched']


def test_viewer_input_and_generated_paths_use_one_task_definition(generated_pair):
    from export_s50_cad_motion_viewer import load_viewer_model
    pair=generated_pair
    via_input,robot_input,binding_input=load_viewer_model(ROOT,pair.source)
    direct,robot_direct,binding_direct=load_viewer_model(ROOT,pair.config)
    assert via_input['definition_sha256']==direct['definition_sha256']==pair.report['task_definition_sha256']
    assert binding_input.identity()==binding_direct.identity()
    assert via_input['task_config_selection']['selected_task_config']==str(pair.config)
    np.testing.assert_allclose(robot_input.fk(np.zeros(6)),robot_direct.fk(np.zeros(6)),atol=1e-12,rtol=0)


def test_viewer_rejects_stale_input_without_loading_preview(generated_pair):
    from export_s50_cad_motion_viewer import load_viewer_model
    pair=generated_pair;original=pair.source.read_bytes()
    try:
        pair.source.write_bytes(original+b'\n# stale viewer source input\n')
        with pytest.raises(ValueError,match='TASK_SOURCE_INPUT_CONFIG_CHANGED'):
            load_viewer_model(ROOT,pair.source)
    finally:
        pair.source.write_bytes(original)


def test_stale_generated_model_is_rejected_without_input_fallback(generated_pair):
    pair=generated_pair;original=pair.source.read_bytes()
    try:
        pair.source.write_bytes(original+b'\n# changed source input after generation\n')
        selected=select_task_configuration(pair.source,root=ROOT)
        assert selected['selected_task_config'] is None
        assert selected['audit_config']==str(pair.config)
        assert any('TASK_SOURCE_INPUT_CONFIG_CHANGED' in problem for problem in selected['missing'])
    finally:
        pair.source.write_bytes(original)


def test_another_input_cannot_adopt_a_generated_model(generated_pair,tmp_path):
    pair=generated_pair;other=tmp_path/'other-input.yaml'
    data=yaml.safe_load(pair.source.read_text())
    data['task_generation']['generated_config']=str(pair.config)
    other.write_text(yaml.safe_dump(data))
    result=select_task_configuration(other,root=ROOT)
    assert result['selected_task_config'] is None
    assert any('BELONGS_TO_ANOTHER_INPUT' in item for item in result['missing'])


def test_generator_rejects_its_generated_config_as_input(generated_pair,tmp_path):
    with pytest.raises(ValueError,match='REQUIRES_INPUT_CONFIG_NOT_GENERATED_MODEL'):
        generate_task_model(generated_pair.config,root=ROOT,output_directory=tmp_path/'not-created')
    assert not (tmp_path/'not-created').exists()


def test_generator_command_chain_uses_generated_config_everywhere(generated_pair):
    chain=generated_pair.report['command_chain']
    assert chain['shared_generated_config']==str(generated_pair.config)
    for command in chain['commands']:
        argv=command['argv']
        key='--task-config' if command['stage'].endswith('viewer') else '--config'
        assert argv[argv.index(key)+1]==str(generated_pair.config)
        assert str(generated_pair.source) not in argv


def test_formal_cli_forwards_effective_generated_config(generated_pair,tmp_path,monkeypatch):
    pair=generated_pair;calls=[]
    def retained(admission,*,config,**kwargs):
        calls.append((Path(config),admission['model_binding']['model_definition_sha256']))
        return dict(status='SYNTHETIC_ROUTING_TEST_NOT_RUN',complete=False,missing=[])
    monkeypatch.setattr(entry,'plan_single_carton_from_admission',retained)
    assert plan_tool.main(['--config',str(pair.source),'--output',str(tmp_path/'cli')])==2
    assert calls==[(pair.config,pair.report['task_definition_sha256'])]
    saved=json.loads((tmp_path/'cli/admission.json').read_text())
    assert saved['effective_config']==str(pair.config)


def test_home_stage_receives_the_same_generated_identity(generated_pair,tmp_path,monkeypatch):
    pair=generated_pair;admission=admission_tool.audit(pair.source)
    # This injected gate is only a routing fixture; the solver is not called.
    admission=deepcopy(admission);admission['geometric_planning']=dict(ready=True,missing=[])
    seen=[]
    def context(config,**kwargs):
        binding=RobotModelBinding.from_config(config)
        seen.append(('factory',str(config),binding.identity()['model_definition_sha256']))
        return SimpleNamespace(binding=binding,layout=SimpleNamespace(layout_fingerprint='synthetic-routing-only'))
    def home(context,**kwargs):
        seen.append(('home',str(context.binding.config_path),context.binding.identity()['model_definition_sha256']))
        return dict(status='SYNTHETIC_NO_IK_RUN',attempts=[],q_rad=None)
    monkeypatch.setattr(entry,'make_configured_context',context)
    monkeypatch.setattr(entry,'solve_configured_initial',home)
    report=entry.plan_single_carton_from_admission(admission,config=pair.source,
        root=ROOT,output=tmp_path/'home-route')
    assert [row[0] for row in seen]==['factory','home']
    assert all(row[1]==str(pair.config) and row[2]==pair.report['task_definition_sha256'] for row in seen)
    assert report['effective_config']==str(pair.config)
    assert not report['native_worker_launched'] and report['planning_attempts']==0
