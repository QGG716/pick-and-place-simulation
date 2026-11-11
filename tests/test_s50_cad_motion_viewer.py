import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('cad_motion_exporter', ROOT/'tools/export_s50_cad_motion_viewer.py')
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


def fixture_trajectory():
    return dict(schema='s50_native_joint_trajectory_v1', model_definition_sha256='fixture-definition',
        joint_names=[f's50_joint_{i}' for i in range(1,7)], source='native_worker_current_run',
        native_provenance={'current_run': True}, status='FAILED_WITH_ACTUAL_STATE',
        scene_context=scene_context(), playback_scope='robot_joint_only',
        environment_playback='initial_scene_static_reference',
        points=[], failure_state={'q_rad':[.1]*6, 'stage':'test-fixture', 'collision_objects':['fixture_object']})


def scene_context():
    mount=np.eye(4);mount[:3,3]=[-1.85,.15,.9]
    return dict(environment_fingerprint='fixture-frozen-environment',
        world_from_physical_mount=mount.tolist(), world_from_model_base=mount.tolist(),
        initial_carton_ids=[f'fixture_carton_{i:02d}' for i in range(40)])


def definition():
    return {'definition_sha256':'fixture-definition', 'joints':[{'name':f's50_joint_{i}'} for i in range(1,7)]}


def test_no_trajectory_is_not_fabricated():
    data = exporter.trajectory_data(None, definition())
    assert data['status'] == 'NOT_RUN' and data['points'] == []
    assert not data['scene_context_verified']
    assert data['environment_playback']=='initial_scene_static_reference'


@pytest.mark.parametrize('field,value,error',[
    ('model_definition_sha256','another-model','IDENTITY'),
    ('source','historical_fanuc_solution','SOURCE'),
    ('native_provenance',{'current_run':False},'PROVENANCE'),
])
def test_trajectory_source_and_model_are_bound(tmp_path,field,value,error):
    data=fixture_trajectory();data[field]=value
    path=tmp_path/'trajectory.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match=error):
        exporter.trajectory_data(path, definition(), expected_scene_context=scene_context())


def test_failure_state_is_retained_without_inventing_success_path(tmp_path):
    data=fixture_trajectory();path=tmp_path/'failure.json';path.write_text(json.dumps(data))
    loaded=exporter.trajectory_data(path, definition(), expected_scene_context=scene_context())
    assert loaded['status']=='FAILED_WITH_ACTUAL_STATE'
    assert loaded['points']==[]
    assert loaded['failure_state']['collision_objects']==['fixture_object']
    assert loaded['scene_context_verified']


@pytest.mark.parametrize('change,error', [
    (lambda d:d.pop('scene_context'), 'SCENE_CONTEXT_REQUIRED'),
    (lambda d:d['scene_context'].update(environment_fingerprint='other-layout'), 'ENVIRONMENT_FINGERPRINT'),
    (lambda d:d['scene_context']['world_from_physical_mount'][0].__setitem__(3,-1.75), 'PHYSICAL_MOUNT_MISMATCH'),
    (lambda d:d['scene_context']['world_from_model_base'][1].__setitem__(3,.25), 'MODEL_BASE_MISMATCH'),
    (lambda d:d['scene_context']['world_from_model_base'][0].__setitem__(0,2.), 'INVALID_WORLD_FROM_MODEL_BASE'),
    (lambda d:d['scene_context']['initial_carton_ids'].pop(), 'FRESH40'),
    (lambda d:d['scene_context']['initial_carton_ids'].__setitem__(1,'fixture_carton_00'), 'FRESH40'),
    (lambda d:d['scene_context']['initial_carton_ids'].__setitem__(1,'other_carton'), 'FRESH40'),
    (lambda d:d.update(playback_scope='full_pick_and_place'), 'JOINT_ONLY_STATIC_ENVIRONMENT'),
    (lambda d:d.update(environment_playback='dynamic'), 'JOINT_ONLY_STATIC_ENVIRONMENT'),
])
def test_same_model_cannot_replay_different_world_or_claim_dynamic_environment(tmp_path,change,error):
    data=fixture_trajectory();change(data)
    path=tmp_path/'trajectory.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match=error):
        exporter.trajectory_data(path,definition(),expected_scene_context=scene_context())


def test_expected_model_base_is_derived_from_same_cad_mount_frame():
    cad=np.eye(4);cad[:3,3]=[.1,-.003,.2]
    model=dict(base_link='base',frames={'mount':{'CAD_from_frame_zero':cad.tolist()}},
        links=[dict(name='base',CAD_from_link_zero=cad.tolist())])
    context=scene_context()
    environment=dict(environment_fingerprint=context['environment_fingerprint'],
        parameters={'world_from_physical_mount':context['world_from_physical_mount']},
        initial_carton_ids=context['initial_carton_ids'])
    actual=exporter.viewer_scene_context(model,environment=environment)
    np.testing.assert_allclose(actual['world_from_model_base'],context['world_from_model_base'],atol=1e-12)
    # A different URDF base convention must not silently reuse mount-root FK.
    model['links'][0]['CAD_from_link_zero'][0][3]+=.1
    with pytest.raises(ValueError,match='FK_ROOT_REQUIRES_MODEL_BASE'):
        exporter.viewer_scene_context(model,environment=environment)


@pytest.mark.parametrize('timestamp',[-1.,True,float('nan')])
def test_recorded_time_is_finite_nonnegative_and_not_boolean(tmp_path,timestamp):
    data=fixture_trajectory();data['points']=[dict(q_rad=[0.]*6,time_s=timestamp)]
    path=tmp_path/'trajectory.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='INVALID_NATIVE_TRAJECTORY_POINT'):
        exporter.trajectory_data(path,definition(),expected_scene_context=scene_context())
