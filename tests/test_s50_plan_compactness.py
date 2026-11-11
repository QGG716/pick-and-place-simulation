"""Synthetic protocol fixtures; no actual S50 task/collision success is claimed."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

from unloading_sim.s50_plan_compactness import coupled_plan_samples,measure_task_plan


def synthetic_plan():
    return dict(batch_root=dict(q_rad=[0.]*6,height_m=.6),height_m=.8,stages=[
        dict(stage='approach',stage_id='a',generation_source='NATIVE_GENERATED',
            points=[dict(q=[0.]*6,t=0.),dict(q=[.1]*6,t=.5),dict(q=[.2]*6,t=1.)]),
        dict(stage='receiver_lift',stage_id='b',generation_source='NATIVE_EXTERNAL_AXIS',
            points=[dict(q=[.2]*6,t=0.)],external_axis=dict(robot_q_constant=True,robot_q=[.2]*6,
                from_height_m=.6,to_height_m=.8,checks=[dict(height_m=h,passed=True) for h in (.6,.7,.8)])),
        dict(stage='release',stage_id='c',generation_source='SEMANTIC_EVENT',points=[],terminal_q=[.2]*6),
        dict(stage='withdrawal',stage_id='d',generation_source='NATIVE_GENERATED',
            points=[dict(q=[.2]*6,t=0.),dict(q=[.3]*6,t=1.)])])


def test_all_original_waypoints_and_axis_checks_keep_recorded_heights():
    samples=coupled_plan_samples(synthetic_plan())
    assert len(samples)==10
    original=[row for row in samples if row['source_kind']=='ORIGINAL_ROBOT_WAYPOINT']
    assert len(original)==6
    assert [row['conveyor_height_m'] for row in original]==[.6,.6,.6,.6,.8,.8]
    axis=[row for row in samples if row['source_kind']=='ORIGINAL_EXTERNAL_AXIS_CHECK']
    assert [row['conveyor_height_m'] for row in axis]==[.6,.7,.8]
    assert original[-1]['height_source']=='b.external_axis.to_height_m'
    assert samples[7]['source_kind']=='RECORDED_SEMANTIC_TERMINAL'


@pytest.mark.parametrize('mutation,reason',[
    (lambda p:p['batch_root'].pop('height_m'),'HEIGHT_MISSING'),
    (lambda p:p['stages'][1]['external_axis'].pop('from_height_m'),'HEIGHT_MISSING'),
    (lambda p:p['stages'][1]['external_axis']['checks'].pop(),'CHECKS_INCOMPLETE'),
    (lambda p:p['stages'][1]['external_axis'].update(from_height_m=.7),'STATE_MISMATCH'),
    (lambda p:p.update(height_m=.6),'FINAL_CONVEYOR_HEIGHT_MISMATCH')])
def test_missing_or_conflicting_height_never_defaults_to_initial(mutation,reason):
    plan=synthetic_plan();mutation(plan)
    with pytest.raises(ValueError,match=reason):coupled_plan_samples(plan)


def test_unavailable_metrics_preserve_original_safety_verification():
    plan=synthetic_plan();plan['batch_root'].pop('height_m')
    verification=dict(status='PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED',native_stages=4)
    report=measure_task_plan(None,plan,verification)
    assert report['status']=='COMPACTNESS_NOT_MEASURED'
    assert report['samples']==[]
    assert report['all_robot_cycle_verification']==verification
    assert report['replaces_all_robot_safety_checks'] is False


def test_measurement_uses_each_original_height_and_true_belt_solid(monkeypatch):
    """Actual frozen belts; synthetic backend/query checks the sampling wiring."""
    import unloading_sim.s50_collision_diagnostics as diagnostic
    import unloading_sim.pinocchio_backend as pin
    from unloading_sim.robot_replacement_layout import frozen_baseline_layout
    root=Path(__file__).resolve().parents[1];layout=frozen_baseline_layout(root)
    binding=SimpleNamespace(verify_source_unchanged=lambda:None,identity=lambda:{'fixture':'SYNTHETIC_NOT_ENGINEERING'})
    backend=SimpleNamespace(model_binding=binding,coal=object())
    context=SimpleNamespace(binding=binding,connector=SimpleNamespace(robot=backend),layout=layout)
    # FANUC baseline layout supplies the exact same physical installation axis.
    monkeypatch.setattr(type(layout),'physical_mount_transform',lambda self:self.robot_base_transform(),raising=False)
    observed=[]
    def obstacle(coal,belt):
        assert belt.solid['kind']=='convex_concept_solid_v1'
        observed.append(float(belt.corners()[:,2].max()))
        return object(),dict(geometry='frozen_convex_solid',is_bounding_proxy=False)
    def geometry(backend,q):
        rows=[]
        for index,link in enumerate(['s50_base',*[f's50_link_{i}' for i in range(1,7)],'s50_link_1','s50_link_2']):
            rows.append(dict(link=link,geometry=f'synthetic-shape-{index}',shape=object(),
                world=np.eye(4),vertices=np.array([[0,0,0],[.01,.01,.01]])))
        return rows
    monkeypatch.setattr(diagnostic,'diagnostic_coal_obstacle',obstacle)
    monkeypatch.setattr(diagnostic,'_world_geometry',geometry)
    monkeypatch.setattr(pin,'_transform',lambda *a:None)
    queries=[]
    def query(*a,**k):
        queries.append(None)
        return dict(status='QUERIED',unsigned_separation_m=.025,intersects=False)
    monkeypatch.setattr(diagnostic,'coal_pair_query',query)
    verification=dict(status='PASS_CYCLE_NATIVE_PARENT_AND_WAIT_OUTFEED')
    report=measure_task_plan(context,synthetic_plan(),verification)
    assert report['status']=='MEASURED_RECORDED_PLAN_SAMPLES',report
    assert observed==pytest.approx([.6,.6,.6,.6,.6,.7,.8,.8,.8,.8])
    assert len(queries)==90
    assert report['original_robot_waypoint_count']==6 and report['external_axis_check_count']==3
    assert report['summary']['sample_count']==10
    assert set(report['samples'][0]['groups'])=={'all_robot','active_links','elbow','wrist'}
    assert report['samples'][0]['groups']['all_robot']['expected_shape_pairs']==9
    assert report['all_robot_cycle_verification']==verification
