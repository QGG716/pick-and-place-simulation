"""Actual assets/run evidence only; run ID is explicitly supplied by the caller."""
import os,hashlib
import numpy as np
import pytest
from unloading_sim.eco65.model import ROOT,LOCAL,URDF,load_json,canonical_fingerprint,transform
from unloading_sim.eco65.unloading_layout import seal_check,face_tcp
from unloading_sim.eco65.compact_task import prepare_world

@pytest.fixture(scope='module')
def run():
    if not (LOCAL/'tool.json').exists() or not URDF.exists():pytest.skip('Actual private CAD/official model unavailable; physical adaptation NOT_EVALUATED')
    ident=os.environ.get('ECO65_COMPACT_RUN_ID')
    assert ident,'Set ECO65_COMPACT_RUN_ID to the explicit evidence run'
    out=ROOT/'outputs/eco65_desktop_layout'/ident;snapshot=load_json(out/'scene_snapshot.json');home=load_json(out/'reports/poses.json')['home']['q']
    return out,snapshot,home

def test_all_twelve_actual_rings(run):
    out,snap,home=run
    for b in snap['scene']['boxes']:
        for face in ('top','front'):
            pose=transform(b['pose']);r=seal_check(snap['tool'],face_tcp(pose,b['size_m'],face),pose,b['size_m'],face)
            assert r['fits'] and r['rings']==12

def test_initial_complete_geometry_and_base_footprint(run):
    out,snap,home=run;selection=dict(box_id=snap['scene']['target_box_id'],face='top',region='conveyor_transverse');w=prepare_world(snap,selection,home)
    try:
        assert w.valid(home,'initial');assert len(w.boxes)==4
        for name in ('trailer_floor','trailer_left','trailer_right','trailer_rear','trailer_roof','robot_mount_plate'):
            assert name in w.geoms and any(name in (a['name'],b['name']) for a,b in w.pairs)
        assert load_json(out/'reports/base_footprint.json')['covered']
    finally:w.close()

def test_complete_validated_trajectory_and_actual_same_id_events(run):
    out,snap,home=run;folder=out/'task';a=load_json(folder/'plan.json');v=load_json(folder/'validation.json');e=load_json(folder/'execution.json')
    assert v['status']=='VALID' and v['samples']>100
    assert a['trajectory_fingerprint']==v['trajectory_fingerprint']==e['trajectory_fingerprint']
    assert e['geometric_tasks_completed']==1
    assert [x['event'] for x in e['events']]==['ATTACHED','SUPPORTED_RELEASE','RETREAT_COMPLETE']
    assert {x['box_id'] for x in e['events']}=={a['box_id']}=={x['box_id'] for x in e['frames']}
    assert np.allclose(e['events'][0]['relative_transform'],a['attachment_tcp_to_box'])
    assert np.allclose(e['events'][1]['pose'],a['released_box_pose'])
    assert e['events'][1]['support']['valid']
    for b in snap['scene']['boxes']:
        if b['id']!=a['box_id']:assert np.allclose(e['retained_cartons_actual'][b['id']],b['pose'],atol=1e-10)
    assert any(np.linalg.norm(f['qd'])>0 for f in e['frames'])

def test_ideal_outfeed_geometry_and_separate_count(run):
    out,snap,home=run;o=load_json(out/'task/outfeed.json');e=load_json(out/'task/execution.json')
    assert e['geometric_tasks_completed']==1
    if o['status']!='OUTFED_ASSUMED':
        assert o['outfed_assumed']==0 and o.get('failure');return
    assert o['outfed_assumed']==1 and o['entire_box_outside']
    pose=np.array(o['final_box_pose']);assert np.allclose(pose[:2,3],[-.78,-.22],atol=1e-8)
    assert {f['box_id'] for f in o['frames']}=={e['events'][0]['box_id']}
    assert np.all(np.diff([f['t'] for f in o['frames']])>=0)
    positions=np.array([f['box_pose'] for f in o['frames']])[:,:3,3]
    assert np.max(np.linalg.norm(np.diff(positions,axis=0),axis=1))<=.011


def test_recorded_actual_boundary_satisfies_final_outfeed_guard(run):
    # Revalidate the new admission guard against recorded actual execution, without rerunning motion.
    from types import SimpleNamespace
    from unloading_sim.eco65.task_checks import outfeed_preconditions
    out,snap,home=run;e=load_json(out/'task/execution.json');last=e['frames'][-1];release=e['events'][1]
    state=SimpleNamespace(box_id=last['box_id'],execution_id=e['execution_id'],attached=False,released=True,
        box_states={last['box_id']:last['box_state']},release_support=release['support'],current_q=np.array(last['q']),released_pose=np.array(last['box_pose']))
    assert outfeed_preconditions(state,e)
