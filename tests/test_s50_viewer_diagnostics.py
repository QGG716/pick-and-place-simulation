from copy import deepcopy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from export_s50_cad_motion_viewer import viewer_scene_context, validate_scene_context
from s50_viewer_diagnostics import diagnostic_data
from unloading_sim.s50_cad_model import load_definition, load_preview_robot


@pytest.fixture(scope='module')
def model():
    definition=load_definition(ROOT);robot=load_preview_robot(ROOT)
    return definition,robot,viewer_scene_context(definition,ROOT)


def fixture_record(model):
    definition,robot,context=model
    base=np.asarray(context['world_from_model_base']);q=np.zeros(6)
    return dict(schema='s50_runtime_collision_diagnostics_v1',qualification_scope='CAD_DIAGNOSTIC_NOT_TASK',
        model_definition_sha256=definition['definition_sha256'],scene_context=context,
        conveyor_height_m=.6,obstacles=[{'name':'mount_plinth'}],
        poses=[dict(pose_id='synthetic_contract_fixture_not_runtime_evidence',q_rad=q.tolist(),
            conveyor_height_m=.6,
            link_frames_world={k:(base@v).tolist() for k,v in robot.named_link_frames(q).items()},
            pairs=[dict(first={'link':'s50_base'},second={'object':'mount_plinth'},
                intersects=True,surface_distance_m=-1.,nearest_points_valid=False)])])


def read_record(tmp_path,model,value):
    p=tmp_path/'fixture.json';p.write_text(json.dumps(value))
    return diagnostic_data(p,*model,validate_scene_context)


def test_actual_diagnostic_frames_verified_but_not_task_accepted(tmp_path,model):
    result=read_record(tmp_path,model,fixture_record(model))
    assert result['viewer_fk_crosscheck']=='PASS_ALL_RECORDED_LINK_FRAMES'
    assert result['qualification_scope']=='CAD_DIAGNOSTIC_NOT_TASK'
    assert result['poses'][0]['pairs'][0]['surface_distance_m']==-1.
    assert result['poses'][0]['pairs'][0]['nearest_points_valid'] is False


@pytest.mark.parametrize('kind,error',[
    ('frame','CORE_FK_MISMATCH'),('environment','ENVIRONMENT_FINGERPRINT'),
    ('model','MODEL_IDENTITY'),('nearest','NEAREST_POINTS'),('distance','DISTANCE'),
    ('scope','LEGAL_TASK'),('duplicate','DUPLICATED'),('height','CONVEYOR_HEIGHT'),('object','SECOND_OBJECT'),
])
def test_stale_or_invalid_diagnostics_rejected(tmp_path,model,kind,error):
    value=deepcopy(fixture_record(model));pose=value['poses'][0];pair=pose['pairs'][0]
    if kind=='frame':pose['link_frames_world']['s50_link_2'][0][3]+=.01
    if kind=='environment':value['scene_context']['environment_fingerprint']='another-layout'
    if kind=='model':value['model_definition_sha256']='other-model'
    if kind=='nearest':pair.update(nearest_points_valid=True,nearest_points_world=[[0,0,0],[float('nan'),1,2]])
    if kind=='distance':pair['surface_distance_m']=float('nan')
    if kind=='scope':value['qualification_scope']='TASK_VALIDATED'
    if kind=='duplicate':value['poses'].append(deepcopy(pose))
    if kind=='height':pose['conveyor_height_m']=1.5
    if kind=='object':pair['second']['object']='unknown_floor'
    with pytest.raises(ValueError,match=error):read_record(tmp_path,model,value)
