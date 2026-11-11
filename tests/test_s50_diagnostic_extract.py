"""Retained viewer records must stay actual subsets, including every collision."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('extract_s50_diagnostic_view',ROOT/'tools/extract_s50_diagnostic_view.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def fixture_report(path, **changes):
    data=dict(schema='s50_runtime_collision_diagnostics_v1', qualification_scope='CAD_DIAGNOSTIC_NOT_TASK',
        status='DIAGNOSTIC_QUERIES_COMPLETED_NOT_TASK_QUALIFIED', environment_fingerprint='synthetic-test-only',
        poses=[dict(pose_id='synthetic-test-only', queried_pair_count=51, pairs=[
            dict(id=i, surface_distance_m=float(i), intersects=i==50) for i in range(51)])])
    data.update(changes)
    path.write_text(json.dumps(data))
    return data


def test_every_collision_retained_without_modifying_actual_pair(tmp_path):
    path=tmp_path/'source.json';original=fixture_report(path)
    result=module.extract([path]);pairs=result['poses'][0]['pairs']
    assert [p['id'] for p in pairs]==list(range(40))+[50]
    assert all(pair in original['poses'][0]['pairs'] for pair in pairs)
    assert result['total_original_pair_count']==51 and result['total_retained_pair_count']==41
    assert json.loads(path.read_text())==original
    assert result['record_scope']=='VIEWER_EXTRACT_NOT_COMPLETE_PAIR_EVIDENCE'


def test_reject_incomplete_query_and_cross_scene_merge(tmp_path):
    a=tmp_path/'a.json';b=tmp_path/'b.json'
    fixture_report(a,status='DIAGNOSTIC_FAILURE_RETAINED')
    with pytest.raises(ValueError,match='ONLY_COMPLETED'):module.extract([a])
    fixture_report(a);fixture_report(b,environment_fingerprint='other-scene')
    with pytest.raises(ValueError,match='DIFFERENT_SCENE'):module.extract([a,b])


def test_reject_duplicate_pose_without_counting_twice(tmp_path):
    path=tmp_path/'source.json';fixture_report(path)
    with pytest.raises(ValueError,match='DUPLICATE_DIAGNOSTIC_POSE'):module.extract([path,path])
