"""Viewer/task contracts using real CAD geometry and synthetic mapping evidence.

These fixtures never run a native planner, never qualify a real S50 installation,
and never represent an actual task trajectory. Mesh assets are read in place;
only small config/definition/URDF/report files are written to pytest's workspace.
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
import export_s50_cad_motion_viewer as viewer
from s50_viewer_diagnostics import diagnostic_data
from unloading_sim.configured_layout_trajectory import export_native_trajectory
from unloading_sim.robot import URDFRobot
from unloading_sim.robot_model_binding import ModelNotReady
from unloading_sim.s50_cad_model import chain_fk,load_definition
from unloading_sim.s50_task_model import generate_task_model,INPUT_CONFIG
from unloading_sim.robot_replacement_layout import freeze_non_robot_scene,frozen_baseline_layout


@pytest.fixture(scope='module')
def task_contract(tmp_path_factory):
    directory=tmp_path_factory.mktemp('task-viewer-SYNTHETIC-NOT-ENGINEERING')
    evidence=directory/'SYNTHETIC-MAPPING-NOT-MANUFACTURER.txt'
    evidence.write_text('Synthetic protocol test only; no manufacturer semantics asserted.',encoding='utf-8')
    data=yaml.safe_load((ROOT/INPUT_CONFIG).read_text(encoding='utf-8'))
    for index in range(6):
        # J3 synthetic offset intentionally makes CAD q3=0 outside task range.
        data['controller_mapping']['joints'][f'J{index+1}']=dict(sign=-1 if index==1 else 1,
            offset_rad=-np.pi/2 if index==1 else np.pi if index==2 else .125,verified=True,
            source=dict(path=evidence.name,sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
                locator='Synthetic math/transport fixture, not engineering evidence'))
    theta=.37;alignment=np.eye(4)
    alignment[:3,:3]=[[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]
    alignment[:3,3]=[.013,-.021,.009]  # A deliberate nonidentity test, not proposed installation.
    data['kinematics']['mounting_from_base']=alignment.tolist()
    config=directory/'input.yaml';config.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
    report=generate_task_model(config,root=ROOT,output_directory=directory/'generated')
    task_config=Path(report['generated_config'])
    definition,local_robot,binding=viewer.load_viewer_model(ROOT,task_config)
    frozen=freeze_non_robot_scene(frozen_baseline_layout(ROOT))
    scene=viewer.viewer_scene_context(definition,ROOT,environment=frozen,mounting_from_base=binding.mounting_from_base)
    obstacles=next(sample['entities'] for sample in frozen['height_samples'] if sample['height_m']==.6)
    return SimpleNamespace(directory=directory,config=task_config,definition=definition,
        local=local_robot,binding=binding,scene=scene,alignment=alignment,report=report,obstacles=obstacles)


def _trajectory(contract):
    low,high=contract.binding.joint_limits.T
    # The actual export function is exercised, but the upstream plan and PASS
    # marker below are explicitly synthetic test fixtures, never run evidence.
    plan=dict(stages=[dict(stage='SYNTHETIC_TEST_ONLY',stage_id='synthetic-native-stage',
        generation_source='NATIVE_GENERATED',points=[dict(q=low.tolist(),t=0.),dict(q=high.tolist(),t=1.)])])
    verification=dict(status='PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED',
        evidence_scope='SYNTHETIC_TEST_ONLY_NO_NATIVE_PLANNER_RUN')
    return export_native_trajectory(plan,contract.binding,verification,scene_context=contract.scene)


def _write(tmp_path,name,data):
    path=tmp_path/name;path.write_text(json.dumps(data),encoding='utf-8');return path


def test_task_viewer_model_keeps_exact_task_identity_and_limits(task_contract):
    c=task_contract
    assert c.definition['schema']=='s50_task_model_v1'
    assert c.definition['definition_sha256']==c.binding.identity()['model_definition_sha256']
    assert c.definition['definition_sha256']!=load_definition(ROOT)['definition_sha256']
    limits=np.asarray([joint['position_limits_rad'] for joint in c.definition['joints']])
    np.testing.assert_allclose(limits,c.binding.joint_limits,atol=0,rtol=0)
    assert limits[2,1]<0  # The viewer must not initialize/reset every task model to all zero.
    np.testing.assert_allclose(c.local.base_transform,np.eye(4),atol=1e-14,rtol=0)


def test_nonidentity_mount_is_applied_exactly_once(task_contract):
    c=task_contract;mount=np.asarray(c.scene['world_from_physical_mount'])
    world_base=np.asarray(c.scene['world_from_model_base'])
    np.testing.assert_allclose(world_base,mount @ c.alignment,atol=1e-14,rtol=0)
    installed=URDFRobot.from_config(c.config,mounting_world_transform=mount)
    low,high=c.binding.joint_limits.T
    for q in (low+.25*(high-low),low+.75*(high-low)):
        pure=chain_fk(c.definition,q)
        local=c.local.named_link_frames(q);world=installed.named_link_frames(q)
        for link in c.definition['links']:
            name=link['name']
            np.testing.assert_allclose(local[name],pure[name],atol=2e-14,rtol=0)
            np.testing.assert_allclose(world_base @ local[name],world[name],atol=2e-14,rtol=0)
        # This fixture would detect accidentally applying the base transform twice.
        assert np.max(np.abs(world_base @ c.alignment @ local[c.definition['flange_link']]
                             -world[c.definition['flange_link']]))>.01


def test_real_export_function_and_viewer_agree_on_task_sha_and_range_endpoints(task_contract,tmp_path):
    c=task_contract;exported=_trajectory(c)
    assert exported['model_definition_sha256']==c.definition['definition_sha256']
    report=viewer.trajectory_data(_write(tmp_path,'fixture-trajectory.json',exported),c.definition,
        expected_scene_context=c.scene)
    assert report['scene_context_verified']
    np.testing.assert_array_equal(report['points'][0]['q_rad'],c.binding.joint_limits[:,0])
    np.testing.assert_array_equal(report['points'][-1]['q_rad'],c.binding.joint_limits[:,1])
    assert report['playback_scope']=='robot_joint_only'
    assert report['environment_playback']=='initial_scene_static_reference'


@pytest.mark.parametrize('endpoint,delta',[(0,-1e-6),(1,1e-6)])
def test_viewer_rejects_task_points_beyond_either_limit(task_contract,tmp_path,endpoint,delta):
    c=task_contract;data=_trajectory(c);data['points'][endpoint]['q_rad'][1]+=delta
    with pytest.raises(ValueError,match='OUTSIDE_VERIFIED_TASK_LIMITS'):
        viewer.trajectory_data(_write(tmp_path,'outside.json',data),c.definition,expected_scene_context=c.scene)


def test_same_fk_preview_sha_cannot_relabel_task_trajectory(task_contract,tmp_path):
    c=task_contract;data=_trajectory(c);data['model_definition_sha256']=load_definition(ROOT)['definition_sha256']
    with pytest.raises(ValueError,match='MODEL_IDENTITY_MISMATCH'):
        viewer.trajectory_data(_write(tmp_path,'preview-sha.json',data),c.definition,expected_scene_context=c.scene)


def _diagnostics(contract):
    c=contract;q=c.binding.joint_limits.mean(axis=1);base=np.asarray(c.scene['world_from_model_base'])
    frames=c.local.named_link_frames(q)
    return dict(schema='s50_runtime_collision_diagnostics_v1',qualification_scope='CAD_DIAGNOSTIC_NOT_TASK',
        model_definition_sha256=c.definition['definition_sha256'],scene_context=c.scene,
        conveyor_height_m=.6,obstacles=[dict(name=row['id'],size_m=row['size_m'],
            world_from_entity=row['world_from_entity'],solid=row.get('solid')) for row in deepcopy(c.obstacles)],
        fixture_notice='SYNTHETIC FK RECORD ONLY; NO COLLISION QUERY PERFORMED',
        poses=[dict(pose_id='synthetic-FK-only',q_rad=q.tolist(),conveyor_height_m=.6,
            link_frames_world={name:(base @ matrix).tolist() for name,matrix in frames.items()},pairs=[])])


def _read_diagnostic(c,path):
    return diagnostic_data(path,c.definition,c.local,c.scene,viewer.validate_scene_context,
        expected_obstacles=c.obstacles)


def test_diagnostic_fk_checks_use_same_task_mount_once(task_contract,tmp_path):
    c=task_contract;data=_diagnostics(c)
    accepted=_read_diagnostic(c,_write(tmp_path,'fk-only.json',data))
    assert accepted['viewer_fk_crosscheck']=='PASS_ALL_RECORDED_LINK_FRAMES'
    # Deliberately wrong second application must be rejected even if q agrees.
    name=c.definition['links'][-1]['name']
    data['poses'][0]['link_frames_world'][name]=(c.alignment @ np.asarray(data['poses'][0]['link_frames_world'][name])).tolist()
    with pytest.raises(ValueError,match='CORE_FK_MISMATCH'):
        _read_diagnostic(c,_write(tmp_path,'wrong-frame.json',data))


def test_diagnostics_do_not_accept_preview_mount_for_a_different_task_mount(task_contract,tmp_path):
    c=task_contract;data=deepcopy(_diagnostics(c))
    data['scene_context']['world_from_model_base']=data['scene_context']['world_from_physical_mount']
    with pytest.raises(ValueError,match='WORLD_FROM_MODEL_BASE_MISMATCH'):
        _read_diagnostic(c,_write(tmp_path,'wrong-mount.json',data))


@pytest.mark.parametrize('change,error',[
    (lambda d:d.update(conveyor_height_m=.8),'CONVEYOR_HEIGHT_MISMATCH'),
    (lambda d:d['poses'][0].update(conveyor_height_m=.8),'POSE_CONVEYOR_HEIGHT_MISMATCH'),
    (lambda d:d['obstacles'].pop(),'OBSTACLE_SET_MISMATCH'),
    (lambda d:d['obstacles'][0]['size_m'].__setitem__(0,d['obstacles'][0]['size_m'][0]+.01),'OBSTACLE_GEOMETRY_MISMATCH'),
    (lambda d:d['obstacles'][0]['world_from_entity'][2].__setitem__(3,.123456),'OBSTACLE_GEOMETRY_MISMATCH'),
    (lambda d:next(row for row in d['obstacles'] if row.get('solid')).update(solid=None),'OBSTACLE_GEOMETRY_MISMATCH'),
])
def test_diagnostics_reject_other_height_or_environment_geometry(task_contract,tmp_path,change,error):
    c=task_contract;data=_diagnostics(c);change(data)
    with pytest.raises(ValueError,match=error):
        _read_diagnostic(c,_write(tmp_path,'changed-environment.json',data))


def test_diagnostics_reject_pair_with_unknown_environment_object(task_contract,tmp_path):
    c=task_contract;data=_diagnostics(c)
    data['poses'][0]['pairs']=[dict(first={'link':c.definition['base_link']},second={'object':'foreign_object'},
        surface_distance_m=.1,nearest_points_valid=False)]
    with pytest.raises(ValueError,match='UNKNOWN_DIAGNOSTIC_SECOND_OBJECT'):
        _read_diagnostic(c,_write(tmp_path,'foreign-pair.json',data))


def test_export_revalidates_bound_task_definition_before_using_its_sha(task_contract):
    c=task_contract;path=c.config.parent/'task_model.json';original=path.read_bytes()
    try:
        task=json.loads(original);task['fixture_after_binding_mutation']=True
        path.write_text(json.dumps(task),encoding='utf-8')
        with pytest.raises((ModelNotReady,ValueError),match='SOURCE_CHANGED|DEFINITION|IDENTITY'):
            _trajectory(c)
    finally:
        path.write_bytes(original)
