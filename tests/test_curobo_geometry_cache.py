"""Directed cache/geometry construction tests; mock caches are not GPU evidence."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from unloading_sim.curobo_v2_backend import geometry_cache_key,prepare_geometry,assemble_runtime,GEOMETRY_SCHEMA,FIT_SETTINGS,local_cover

ROOT=Path(__file__).resolve().parents[1]


def bundle():
    b=json.loads((ROOT/'docs/evidence/curobo_v2_20260922/final_business/bundle.json').read_text())
    b['pair_permissions']=dict(base_mount=[['base_link','chassis']],named_stack_cartons=[x['name'] for x in b['obstacles'] if x['category']=='carton'],target_id=b['request']['payload']['object_id'])
    return b


def geometry(b):
    return dict(schema=GEOMETRY_SCHEMA,key=geometry_cache_key(b),quality={},spheres={'base_link':[dict(center=[0,0,0],radius=.1)]})


def test_policy_change_reuses_only_geometry_and_reassembles_permissions(tmp_path):
    b=bundle();g=geometry(b);(tmp_path/(g['key']+'.json')).write_text(json.dumps(g))
    one,timing=prepare_geometry(b,tmp_path)
    assert timing['cache_hit']
    new=deepcopy(b);new['request']['collision_policy']['wrist_tool_exempt_links']=[]
    new['request']['collision_policy_fingerprint']='revoked'
    assert geometry_cache_key(new)==geometry_cache_key(b)
    two,timing=prepare_geometry(new,tmp_path)
    assert timing['cache_hit'] and two==one
    a=assemble_runtime(b,g);c=assemble_runtime(new,g)
    assert a['runtime_policy']['wrist_tool_exempt_links']==['J5_link','J6_link']
    assert c['runtime_policy']['wrist_tool_exempt_links']==[]
    assert a['robot_cfg']['kinematics']['self_collision_ignore']==c['robot_cfg']['kinematics']['self_collision_ignore']
    assert all('tool_part' not in name for name in a['robot_cfg']['kinematics']['collision_spheres'])


def test_fitting_parameters_and_schema_invalidate_cache(tmp_path):
    b=bundle();key=geometry_cache_key(b)
    settings={**FIT_SETTINGS,'num_spheres':65}
    assert geometry_cache_key(b,settings)!=key
    b['meshes'][0]['sha256']='different'
    assert geometry_cache_key(b)!=key
    key=geometry_cache_key(b)
    (tmp_path/(key+'.json')).write_text(json.dumps(dict(schema='old',key=key,robot_cfg={})))
    with pytest.raises(ValueError,match='cache schema'):prepare_geometry(b,tmp_path)


def test_local_cover_does_not_inflate_unrelated_spheres_or_shrink():
    c=np.array([[0.,0,0],[10.,0,0]]);r=np.array([1.,1.])
    changed,increment=local_cover(c,r,np.array([[1.1,0,0],[10.,.5,0]]),.000002)
    assert changed[0]==pytest.approx(1.100002) and changed[1]==1.
    assert np.all(changed>=r) and increment[1]==0


def test_worker_context_covers_configuration_but_reuses_for_new_endpoints():
    from unloading_sim.stage_export import worker_context_key
    b=bundle();key=worker_context_key(b)
    new=deepcopy(b);new['request']['q_start'][0]+=.001
    assert worker_context_key(new)==key
    for change in ('limits','permissions','tool','self_ignore'):
        new=deepcopy(b)
        if change=='limits':new['request']['limits']['velocity'][0]*=.9
        elif change=='permissions':new['pair_permissions']['base_mount']=[]
        elif change=='tool':new['tool'][0]['dimensions_m'][0]+=.001
        else:new['self_collision_ignore']=[]
        assert worker_context_key(new)!=key
        assert geometry_cache_key(new)==geometry_cache_key(b)
