"""Directed checks for the 200 mm travel limit; no native execution evidence."""
import numpy as np
import pytest
import yaml
from unloading_sim.eco65.model import ROOT
from unloading_sim.eco65.coupled_scene import make_coupled_spec, coupled_conveyor_state, build_coupled_scene
from unloading_sim.eco65.transverse_process import process_options
from test_eco65_coupled_scene import source_scene, vertices
from test_eco65_transverse_process import box


def config():
    return yaml.safe_load((ROOT/'configs/workcells/eco65_coupled_carton_v3.yaml').read_text())


def test_200mm_limit_keeps_rigid_geometry_fixed_feet_and_actual_height():
    source=source_scene();spec=make_coupled_spec(source,config())
    assert spec['low_height_m']==.200 and spec['high_height_m']==.450
    states=[coupled_conveyor_state(spec,h) for h in (.2,.231182794,.331182794,.431182794,.45)]
    fixed=None
    for state in states:
        m=state['mechanism'];rows=state['world_objects']
        assert state['tilt_rad']>0
        assert m['lower_endpoint_D_m'][2]==.180
        assert np.linalg.norm(np.array(m['upper_endpoint_D_m'])-m['lower_endpoint_D_m'])==pytest.approx(.4443197215708027)
        now={r['id']:r for r in rows if r['id'] in m['fixed_structure_ids']}
        if fixed is None:fixed=now
        assert fixed==now
        assert all(vertices(r)[:,2].min()>=-1e-9 for r in rows)
        for stack in m['stacks']:
            assert all(np.linalg.norm(np.diff(bar['endpoints_m'],axis=0))==pytest.approx(bar['length_m']) for bar in stack['bars'])
    with pytest.raises(ValueError):coupled_conveyor_state(spec,.199)
    source['coupled_state']={'height_m':.3311827942858692}
    scene=build_coupled_scene(source,config())
    assert scene['coupled_state']['height_m']==.3311827942858692
    source['coupled_state']['height_m']=.18
    with pytest.raises(ValueError):build_coupled_scene(source,config())
    assert build_coupled_scene(source,config(),initial_height_m=.3311827942858692)['coupled_state']['height_m']==.3311827942858692


@pytest.mark.parametrize('bottom,heights',[(.181182794,[]),(.281182794,[.231182794]),(.381182794,[.331182794,.231182794]),(.481182794,[.431182794,.331182794])])
def test_actual_layers_do_not_clamp_or_fall_back_to_top(bottom,heights):
    spec=make_coupled_spec(source_scene(),config());b=box(bottom)
    result=process_options(b,[b],[b],spec,coupled_conveyor_state(spec,.331182794)['receiving'])
    assert not result['frozen']['top_allowed']
    assert [o['height_m'] for o in result['options']]==pytest.approx(heights)
    assert all(o['face']=='front' for o in result['options'])
    if not heights:assert all(r['reason']=='PROCESS_RECEIVE_HEIGHT_OUT_OF_RANGE' for r in result['rejected'])


def test_region_suffix_candidates_share_absolute_phase_budget(monkeypatch):
    from unloading_sim.eco65 import coupled_task
    task=object.__new__(coupled_task.CoupledTask)
    now=[100.];monkeypatch.setattr(coupled_task,'perf_counter',lambda:now[0])
    task.started=0.;task.deadline=900.;task.connection_seconds=80.;task.connection_deadline=None
    task.phase_budgets={};task.counter=1;task.target={'id':'target'}
    def candidates():
        for stamp in (100.,150.,179.):
            now[0]=stamp
            with task.connection():assert task.connection_deadline==180.
        now[0]=180.
        task.require_connection_budget()
    task.extract_and_release_in_budget=candidates
    with pytest.raises(ValueError,match='CONNECTION_BUDGET_EXHAUSTED'):task.extract_and_release()
    assert task.deadline==900. and task.connection_deadline is None
    assert task.phase_budgets['suffix-1-target']['deadline_elapsed_s']==180.


def test_batch_candidate_checkpoint_leaves_full_archive_to_committed_boundary(tmp_path):
    import json
    from unloading_sim.eco65.batch_task import BatchTask
    task=object.__new__(BatchTask)
    task.progress_path=tmp_path/'native_progress.jsonl';task.completed_ids=['first']
    task.target={'id':'second'};task.current_q=np.zeros(6);task.height_m=.331
    task.attachment=np.eye(4);task.cycle_records=[{}];task.started=0.
    task.result=lambda:pytest.fail('Full batch serialization inside the motion phase')
    task.save_checkpoint('FIRST_CARTON_EXTRACTED')
    progress=json.loads((tmp_path/'batch_progress.json').read_text())
    assert progress['completed_ids']==['first'] and progress['attached_id']=='second'
    assert not (tmp_path/'latest_plan.json.gz').exists()


def test_single_cycle_also_avoids_full_serialization_before_withdrawal(tmp_path):
    import json
    from unloading_sim.eco65.coupled_task import CoupledTask
    task=object.__new__(CoupledTask)
    task.progress_path=tmp_path/'native_progress.jsonl';task.completed_ids=[]
    task.target={'id':'first'};task.current_q=np.zeros(6);task.height_m=.331
    task.cycle_records=[{}];task.started=0.;task.parent='native-extraction-1'
    task.result=lambda:pytest.fail('Full candidate serialization before withdrawal')
    task.save_checkpoint('FIRST_CARTON_EXTRACTED')
    progress=json.loads((tmp_path/'candidate_progress.json').read_text())
    assert progress['parent_stage_id']=='native-extraction-1' and progress['completed_ids']==[]
    assert not (tmp_path/'first_carton_extracted.json').exists()
