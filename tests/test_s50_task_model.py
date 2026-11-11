"""Task identity/limit protocol tests; temporary mappings are synthetic fixtures.

No fixture mapping or generated temporary task model is S50 engineering evidence.
The checked-in real configuration remains mapping-incomplete and must not generate.
"""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from unloading_sim.robot import URDFRobot
from unloading_sim.robot_model_binding import ModelNotReady, RobotModelBinding
from unloading_sim.s50_cad_model import load_preview_robot
from unloading_sim.s50_task_model import (generate_task_model, mapping_requirements, task_identity_audit,
    verify_task_definition, INPUT_CONFIG, SCHEMA, _relative)

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def mapping_fixture(tmp_path):
    """Use real geometry to test transport, with explicitly NON-engineering maps."""
    data=yaml.safe_load((ROOT/INPUT_CONFIG).read_text(encoding='utf-8'))
    source=tmp_path/'SYNTHETIC_MAPPING_FIXTURE_ONLY.txt'
    source.write_text('Synthetic mapping for code tests only. Not manufacturer evidence.',encoding='utf-8')
    for index,label in enumerate([f'J{i}' for i in range(1,7)]):
        data['controller_mapping']['joints'][label]=dict(sign=-1 if index==1 else 1,
            offset_rad=-np.pi/2 if index==1 else .125,verified=True,
            source=dict(path=source.name,sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                locator='SYNTHETIC TEST FIXTURE NOT S50 ENGINEERING EVIDENCE'))
    data['kinematics']['mounting_from_base']=np.eye(4).tolist()
    path=tmp_path/'fixture-input.yaml'
    path.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    return path


def _generated(config,tmp_path):
    report=generate_task_model(config,root=ROOT,output_directory=tmp_path/'generated-fixture')
    assert report['task_model_generated'],report
    return report,Path(report['generated_config'])


def test_real_missing_references_never_write_task_urdf(tmp_path):
    destination=tmp_path/'must-remain-absent'
    report=generate_task_model(root=ROOT,output_directory=destination)
    assert report['status']=='MODEL_NOT_READY'
    assert not report['task_model_generated'] and not destination.exists()
    assert report['task_definition_sha256'] is None
    assert any('controller_mapping.J2.' in item for item in report['missing'])
    assert any('controller_mapping.J3.' in item for item in report['missing'])
    derived=report['physical_frame_derivations']
    np.testing.assert_allclose(np.asarray(derived['physical_mount_world'])[:3,3],[-1.85,.15,.9])
    assert derived['tool_connector_extension_m']==.2
    assert derived['flange_from_tool_tcp'] is None and not derived['tool_connected']
    assert not report['native_worker_launched']


def test_published_limits_remain_separate_from_simulation_limits():
    data=yaml.safe_load((ROOT/INPUT_CONFIG).read_text(encoding='utf-8'))
    assert data['joint_position_limits_rad'] is None
    np.testing.assert_allclose(np.rad2deg(data['published_controller_position_limits_rad']['J2']),[-190,10])
    np.testing.assert_allclose(np.rad2deg(data['published_controller_position_limits_rad']['J3']),[-160,160])
    assert 's50_cad_preview' not in data['kinematics']['urdf_path']
    assert data['generated_definition_schema']==SCHEMA


@pytest.mark.skipif(os.name!='nt',reason='Windows drive-root path semantics')
def test_windows_different_drive_reference_is_explicit_absolute_not_rewritten():
    assert _relative(Path('D:/s50-source/assets/mesh.obj'),Path('C:/s50-output/model'))=='D:/s50-source/assets/mesh.obj'
    assert _relative(Path('D:/s50-source/assets/mesh.obj'),Path('D:/s50-source/model'))=='../assets/mesh.obj'


def test_claimed_mapping_requires_hash_bound_reference(mapping_fixture):
    data=yaml.safe_load(mapping_fixture.read_text())
    assert mapping_requirements(data,mapping_fixture)['ready']
    data['controller_mapping']['joints']['J2']['source']='An unbound source label'
    report=mapping_requirements(data,mapping_fixture)
    assert not report['ready'] and any('HASHED_LOCAL_REFERENCE' in item for item in report['missing'])
    data=yaml.safe_load(mapping_fixture.read_text())
    data['controller_mapping']['joints']['J2']['source']['sha256']='0'*64
    assert any('SHA256_MISMATCH' in item for item in mapping_requirements(data,mapping_fixture)['missing'])


def test_generated_task_has_independent_limits_identity_and_same_fk(mapping_fixture,tmp_path):
    report,config=_generated(mapping_fixture,tmp_path)
    assert not report['timing']['ready']  # Missing acceleration/revision does not block geometry generation.
    binding=RobotModelBinding.from_config(config)
    task=verify_task_definition(config,yaml.safe_load(config.read_text()))
    assert task['definition_sha256']==report['task_definition_sha256']
    assert binding.identity()['model_definition_sha256']==task['definition_sha256']
    assert task['definition_sha256']!=task['source_cad_definition_sha256']
    if config.drive.casefold()!=ROOT.drive.casefold():
        assert task['path_transport']['mode']=='LOCAL_ABSOLUTE_REFERENCES_NOT_PORTABLE'
        assert Path(task['source_cad_definition_path']).is_absolute()
        assert task['path_transport']['absolute_local_references']
    else:
        assert task['path_transport']['mode']=='RELATIVE_REFERENCES_PRESERVE_DIRECTORY_STRUCTURE'
        assert not Path(task['source_cad_definition_path']).is_absolute()
    assert not task['path_transport']['asset_files_copied']
    np.testing.assert_allclose(binding.joint_limits[1],np.deg2rad([-100,100]),atol=1e-14)
    assert task['joints'][1]['position_limits_rad']==list(binding.joint_limits[1])
    assert 'preview_range_rad' not in task['joints'][1]
    assert 'preview_constraints' not in task
    assert not task['installed_task_qualified'] and not task['dynamics_qualified']
    # Same physical geometry and coordinates, different limit/model identity.
    robot=URDFRobot.from_config(config);preview=load_preview_robot(ROOT)
    for q in (np.zeros(6),np.array([.2,-.3,.4,-.5,.6,-.7])):
        for name,frame in robot.named_link_frames(q).items():
            np.testing.assert_allclose(frame,preview.named_link_frames(q)[name],atol=1e-14,rtol=0)
    xml=ET.parse(binding.urdf_path)
    assert all('effort' not in limit.attrib for limit in xml.getroot().iter('limit'))
    assert task_identity_audit(config)['ready']


def test_task_json_cannot_bless_changed_urdf_by_rehash(mapping_fixture,tmp_path):
    _,config=_generated(mapping_fixture,tmp_path)
    folder=config.parent;urdf=folder/'s50_task.urdf';definition=folder/'task_model.json'
    xml=ET.parse(urdf);xml.find("joint[@name='s50_joint_2']/axis").set('xyz','0 0 1')
    xml.write(urdf,encoding='utf-8',xml_declaration=True)
    task=json.loads(definition.read_text())
    task['generated_files']['urdf']['sha256']=hashlib.sha256(urdf.read_bytes()).hexdigest()
    definition.write_text(json.dumps(task))
    with pytest.raises(ValueError,match='NOT_REPRODUCIBLE'):
        RobotModelBinding.from_config(config)


def test_preview_definition_cannot_be_used_as_task(mapping_fixture,tmp_path):
    _,config=_generated(mapping_fixture,tmp_path)
    data=yaml.safe_load(config.read_text())
    data['generated_definition']=str(ROOT/'assets/robots/huayan_s50_2000/generated/cad_kinematic_model.json')
    config.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError,match='PREVIEW_OR_MODEL_IDENTITY'):
        RobotModelBinding.from_config(config)


def test_changed_mapping_reference_invalidates_existing_binding(mapping_fixture,tmp_path):
    _,config=_generated(mapping_fixture,tmp_path)
    binding=RobotModelBinding.from_config(config)
    source=mapping_fixture.parent/'SYNTHETIC_MAPPING_FIXTURE_ONLY.txt'
    source.write_text('Changed synthetic reference')
    with pytest.raises(ModelNotReady,match='SOURCE_CHANGED'):
        binding.verify_source_unchanged()
    with pytest.raises(ModelNotReady,match='SHA256_MISMATCH'):
        RobotModelBinding.from_config(config)


def test_no_overwrite_of_existing_different_task_model(mapping_fixture,tmp_path):
    _,config=_generated(mapping_fixture,tmp_path)
    task=config.parent/'s50_task.urdf';task.write_bytes(task.read_bytes()+b'<!-- prior work -->')
    with pytest.raises(ValueError,match='ALREADY_EXISTS_WITH_DIFFERENT_IDENTITY'):
        generate_task_model(mapping_fixture,root=ROOT,output_directory=config.parent)
    assert task.read_bytes().endswith(b'<!-- prior work -->')


def test_fanuc_identity_does_not_gain_task_fields():
    binding=RobotModelBinding.from_config(ROOT/'configs/robots/fanuc_m710id_70.yaml')
    assert 'model_definition_sha256' not in binding.identity()
    assert 'task_definition' not in binding.source_asset_sha256


def test_generation_rejects_a_different_configured_joint_order(mapping_fixture,tmp_path):
    data=yaml.safe_load(mapping_fixture.read_text())
    data['kinematics']['active_joint_names'][1:3]=reversed(data['kinematics']['active_joint_names'][1:3])
    mapping_fixture.write_text(yaml.safe_dump(data))
    destination=tmp_path/'must-remain-absent'
    with pytest.raises(ValueError,match='JOINT_ORDER_MISMATCH'):
        generate_task_model(mapping_fixture,root=ROOT,output_directory=destination)
    assert not destination.exists()
