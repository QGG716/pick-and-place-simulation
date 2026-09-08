import json
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import (Cell, evaluate_task, grasp_seed_configurations,
                                               grasp_task_set, escape_path_proposals,
                                               support_relations)
from unloading_sim.validation_physics import (InitialProximityTracker, RigidAttachment,
                                               contact_separated, world_link_boxes)
from unloading_sim.validation_scenes import grid_tasks, regular_scene, random_scene
from tools.run_m710id70_acceptance import _scene_regular, _scene_random
from tools.run_m710id70_v3 import (grasp_task_set_recovery_row,
                                   initial_proximity_recovery_row)


def test_original_populations_and_grid_denominator_are_unchanged():
    s=load_validation_config().data['scene']
    for actual,expected in [(regular_scene(s),_scene_regular()),*[(random_scene(s,seed),_scene_random(seed)) for seed in s['random_seeds']]]:
        assert len(actual)==len(expected)
        for a,b in zip(actual,expected):
            assert a.name==b.name
            assert np.array_equal(a.center,b.center)
            assert np.array_equal(a.half_extents,b.half_extents)
    assert sum(valid for *_,valid in grid_tasks(s))==104
    assert [len(regular_scene(s)),*[len(random_scene(s,seed)) for seed in s['random_seeds']]]==[40,27,32,30]


def test_full_path_checks_interior_even_when_both_endpoints_valid(monkeypatch):
    cell=Cell(load_validation_config())
    monkeypatch.setattr(cell,'state_failure',lambda q,*args: {'reason':'INTERIOR_COLLISION'} if .04<q[0]<.06 else None)
    start=np.zeros(6);end=np.array([.1,0,0,0,0,0])
    failure=cell.path_failure([start,end],[])
    assert failure['reason']=='INTERIOR_COLLISION'
    assert 0<failure['fraction']<1


def test_tool_forearm_collision_rejects_old_home_and_new_home_is_explicit():
    cfg=load_validation_config();cell=Cell(cfg)
    original=np.array([-.17301878,-1.13655578,-.74874837,.63151726,-2.12393931,-2.83276516])
    obstacles=[*cell.fixtures(),*cell.decks((0,.2)),*regular_scene(cfg.data['scene'])]
    assert cell.state_failure(original,obstacles)['reason']=='TOOL_SELF_COLLISION'
    assert cell.state_failure(cfg.data['robot']['home_joints'],obstacles) is None


def test_urdf_corner_missing_from_centerline_proxy_is_checked():
    cfg=load_validation_config();cell=Cell(cfg);q=np.asarray(cfg.data['robot']['home_joints'])
    shapes=world_link_boxes(cell.robot,q,cell.shapes)
    base=next(b for b in shapes if b.name=='base_link')
    obstacle=OBB(base.to_world([.29,.29,.2]),np.full(3,.005),np.eye(3),'base_corner')
    # Old base capsule is a sphere at the mounting origin, not this cylinder.
    assert not cell.robot.link_capsules(q)[0].collides_obb(obstacle)
    failure=cell.state_failure(q,[obstacle])
    assert failure['reason']=='ROBOT_COLLISION'


def test_lift_changes_robot_mount_without_moving_chassis_or_belt():
    cfg=load_validation_config();low=Cell(cfg,0);high=Cell(cfg,.4)
    assert np.allclose(high.robot.base_transform[:3,3]-low.robot.base_transform[:3,3],[0,0,.4])
    assert np.array_equal(high.chassis.world_from_local,low.chassis.world_from_local)
    for a,b in zip(low.decks((.3,.5)),high.decks((.3,.5))):
        assert np.array_equal(a.world_from_local,b.world_from_local)


def test_last_bottom_box_retains_mechanical_conveyor_height_conflict():
    cfg=load_validation_config();cell=Cell(cfg)
    target=OBB([1.3,0,.15],[.3,.2,.15],np.eye(3),'last_box')
    assert cell.conveyor_options(target,[target],'dynamic',(0,.2))==[]


def test_support_contact_exception_does_not_allow_penetration():
    deck=OBB([0,0,.1],[.5,.4,.1],np.eye(3))
    assert contact_separated(OBB([0,0,.35],[.3,.2,.15],np.eye(3)),deck,.0002)
    assert not contact_separated(OBB([0,0,.349],[.3,.2,.15],np.eye(3)),deck,.0002)


def _proximity_boxes(gap=0.01):
    target=OBB([0,0,0],[.5,.5,.5],np.eye(3),'target')
    neighbor=OBB([1+gap,0,0],[.5,.5,.5],np.eye(3),'neighbor')
    return target,neighbor


def _moved(box,dx):
    return OBB(box.center+np.asarray(dx),box.half_extents,box.rotation,box.name,box.category)


def test_ten_mm_initial_neighbor_gap_is_registered_and_may_separate():
    target,neighbor=_proximity_boxes()
    tracker,failure=InitialProximityTracker.capture(target,[neighbor],.01,.0002,1e-6)

    assert failure is None
    assert set(tracker.pairs)=={'neighbor'}
    assert tracker.state_failure(target,[neighbor]) is None
    assert tracker.state_failure(_moved(target,[-.005,0,0]),[neighbor]) is None
    assert not tracker.fully_released


def test_motion_toward_initial_neighbor_fails_immediately():
    target,neighbor=_proximity_boxes()
    tracker,failure=InitialProximityTracker.capture(target,[neighbor],.01,.0002,1e-6)
    assert failure is None

    failure=tracker.state_failure(_moved(target,[.002,0,0]),[neighbor])

    assert failure['reason']=='PAYLOAD_PROXIMITY_WORSENED'
    assert failure['current_signed_distance_m'] < failure['initial_signed_distance_m']


def test_small_numeric_jitter_cannot_accumulate_into_motion_toward_neighbor():
    target,neighbor=_proximity_boxes()
    tracker,failure=InitialProximityTracker.capture(target,[neighbor],.01,.0002,.0001)
    assert failure is None
    assert tracker.state_failure(_moved(target,[.00005,0,0]),[neighbor]) is None

    failure=tracker.state_failure(_moved(target,[.00015,0,0]),[neighbor])

    assert failure['reason']=='PAYLOAD_PROXIMITY_WORSENED'
    assert failure['current_signed_distance_m'] < .01-.0001


def test_normal_collision_margin_is_restored_after_free_space():
    target,neighbor=_proximity_boxes()
    tracker,failure=InitialProximityTracker.capture(target,[neighbor],.01,.0002,1e-6)
    assert failure is None
    assert tracker.state_failure(_moved(target,[-.011,0,0]),[neighbor]) is None
    assert tracker.fully_released

    failure=tracker.state_failure(_moved(target,[-.005,0,0]),[neighbor])

    assert failure['reason']=='PAYLOAD_PROXIMITY_REENTRY'


def test_initial_proximity_never_waives_unregistered_pair_or_real_penetration():
    target,neighbor=_proximity_boxes()
    tracker,failure=InitialProximityTracker.capture(target,[],.01,.0002,1e-6)
    assert failure is None
    assert tracker.state_failure(target,[neighbor])['reason']=='PAYLOAD_COLLISION'

    penetrating,neighbor=_proximity_boxes(gap=-.001)
    _,failure=InitialProximityTracker.capture(penetrating,[neighbor],.01,.0002,1e-6)
    assert failure['reason']=='PAYLOAD_INITIAL_PENETRATION'


def test_grid_neighbor_clearance_registers_proximity_instead_of_failing_initial_gate(monkeypatch):
    cfg=load_validation_config();cell=Cell(cfg)
    _,target,neighbors,valid=list(grid_tasks(cfg.data['scene']))[20]
    assert valid
    monkeypatch.setattr(cell,'conveyor_options',lambda *args: [])
    result=evaluate_task(cell,target,[target,*neighbors],np.asarray(cfg.data['robot']['home_joints']),
        (0,.2),seed=cfg.data['planning']['seed']+20,only_face='top')
    assert result['grasp_reachable']
    assert not result['geometric_feasible']
    assert result['failure_reason']=='MINIMUM_BELT_HEIGHT_EXCEEDS_UPPER_STACK_BOUND'
    assert all(a.get('reason')!='PAYLOAD_INITIAL_CLEARANCE_FAILED' for a in result['attempts'])
    assert any('left_neighbor' in
               {p['obstacle'] for p in a.get('initial_proximity',{}).get('pairs',[])}
               for a in result['attempts'])


def test_initial_proximity_reporting_does_not_count_gate_passage_as_success():
    result={'attempts':[{'reason':'CONVEYOR_SWEEP_COLLISION','initial_proximity':{
                'pairs':[{'obstacle':'neighbor'}]},'conveyor_attempts':[]}],
            'geometric_feasible':False,'failure_stage':'conveyor_preposition',
            'failure_reason':'CONVEYOR_SWEEP_COLLISION'}

    row=initial_proximity_recovery_row('grid_020',result)

    assert row['registered_initial_proximity']
    assert row['passed_initial_gate']
    assert not row['complete_geometric_success']
    assert row['classification']=='INITIAL_GATE_PASSED_FAILED_BEFORE_SEPARATION'


def test_grasp_task_set_is_ordered_sparse_and_deterministic():
    nominal=np.eye(4);nominal[:3,3]=[1,2,3]
    args=(nominal,[0,-.025,.025,-.05,.05],[0,-.0005,.0005])

    first=grasp_task_set(*args);second=grasp_task_set(*args)

    assert len(first)==13
    assert first[0][1]['variant']=='nominal'
    assert all(np.array_equal(a[0],b[0]) and a[1]==b[1] for a,b in zip(first,second))
    assert all(np.isclose(pose[2,3],3) for pose,meta in first if meta['face_offset_local_xy_m'] != [0,0])


def test_grasp_wrist_flip_seeds_are_valid_and_do_not_replace_strict_ik():
    cfg=load_validation_config();robot=cfg.robot();home=np.asarray(cfg.data['robot']['home_joints'])

    seeds=grasp_seed_configurations(robot,[home,home.copy()])

    assert len(seeds)==3
    assert np.array_equal(seeds[0],home)
    assert all(robot.within_limits(q) for q in seeds)


def test_grasp_task_set_reporting_counts_only_strict_actual_grasps_as_recovered():
    result={'attempts':[{'task_set':{'variant':'nominal'},'reason':'NO_IK',
                         'failure_taxonomy':{'detail':'POSITION_RESIDUAL_NOT_CONVERGED'}},
                        {'task_set':{'variant':'offset_local_x_+0.025'},'reason':'PATH_SEARCH_EXHAUSTED',
                         'tcp_from_box':np.eye(4).tolist(),'strict_grasp_valid':True}],
            'grasp_reachable':True,'geometric_feasible':False,
            'failure_stage':'extraction','failure_reason':'ROBOT_COLLISION'}

    row=grasp_task_set_recovery_row('grid_032',result)

    assert not row['nominal_strict_grasp_valid']
    assert row['expanded_task_set_strict_grasp_valid']
    assert row['task_set_recovered_grasp']


def test_grasp_task_set_reporting_does_not_count_pre_clearance_tcp_capture():
    result={'attempts':[{'task_set':{'variant':'nominal'},'reason':'PAYLOAD_INITIAL_CLEARANCE_FAILED',
                         'tcp_from_box':np.eye(4).tolist()}],
            'grasp_reachable':False,'geometric_feasible':False,
            'failure_stage':'attachment_clearance','failure_reason':'PAYLOAD_INITIAL_CLEARANCE_FAILED'}

    row=grasp_task_set_recovery_row('grid_020',result)

    assert not row['nominal_strict_grasp_valid']
    assert not row['expanded_task_set_strict_grasp_valid']


def test_grasp_only_stops_before_conveyor_or_path_search(monkeypatch):
    cfg=load_validation_config();cell=Cell(cfg)
    _,target,neighbors,valid=list(grid_tasks(cfg.data['scene']))[20]
    monkeypatch.setattr(cell,'conveyor_sweep',lambda *args: (_ for _ in ()).throw(
        AssertionError('grasp-only qualification must not enter conveyor planning')))

    result=evaluate_task(cell,target,[target,*neighbors],np.asarray(cfg.data['robot']['home_joints']),
        (0,.2),seed=cfg.data['planning']['seed']+20,only_face='top',grasp_only=True)

    assert valid and result['grasp_reachable']
    assert result['failure_stage']=='grasp_task_set_complete'
    assert result['failure_reason']=='GRASP_TASK_SET_VALID'
    assert result['selected'] is None


def test_escape_proposals_find_lift_before_620mm_pure_front_extraction():
    cfg=load_validation_config();p=cfg.data['planning']
    target=OBB([1.3,0,.8],[.3,.2,.15],np.eye(3),'target')
    left=OBB([1.3,.41,.8],[.3,.2,.15],np.eye(3),'left')
    right=OBB([1.3,-.41,.8],[.3,.2,.15],np.eye(3),'right')

    proposals=escape_path_proposals(target,[-1,0,0],[left,right],.62,p)

    lift=next(item for item in proposals if item['escape_direction']=='lift' and
              item['escape_rotation_world_z_rad']==0)
    assert np.isclose(lift['constrained_straight_distance_m'],0)
    assert np.isclose(lift['escape_translation_m'],.32,atol=1e-8)
    assert lift['escape_translation_m'] < .62


def test_support_relation_graph_drives_carton_and_floor_release_names():
    cfg=load_validation_config();cell=Cell(cfg);p=cfg.data['planning']
    lower=OBB([1.3,0,.15],[.3,.2,.15],np.eye(3),'lower')
    upper=OBB([1.3,0,.46],[.3,.2,.15],np.eye(3),'upper')

    upper_names,audit=support_relations(upper,[lower,upper],cell.fixtures(),p)
    lower_names,_=support_relations(lower,[lower,upper],cell.fixtures(),p)

    assert upper_names==['lower']
    assert lower_names==['floor']
    assert audit['support_edges'][0]['supporter']=='lower'


@pytest.mark.slow
def test_original_grid_022_top_escape_is_a_reproducible_full_geometry_witness():
    cfg=load_validation_config();p=cfg.data['planning']
    # Keep the witness focused on the nominal task-set member. This does not
    # alter its scene, strict tolerances, collision margin or original index.
    p['grasp_face_offset_candidates_m']=[0.0,0.025]
    p['grasp_tilt_candidates_rad']=[0.0]
    p['grasp_downstream_candidate_limit_per_strategy']=3
    p['escape_rotation_candidates_rad']=[0.0]
    cell=Cell(cfg);_,target,neighbors,valid=list(grid_tasks(cfg.data['scene']))[22]
    conveyor=cfg.data['conveyor'];belt=(conveyor['fixed_extension_m'],conveyor['fixed_z_m'])

    result=evaluate_task(cell,target,[target,*neighbors],np.asarray(cfg.data['robot']['home_joints']),
        belt,seed=p['seed']+22,mode='fixed',only_face='top')

    assert valid and result['geometric_feasible']
    assert result['selected']['face']=='top'
    assert result['selected']['roll_deg']==90
    assert result['selected']['task_set']['variant']=='offset_local_y_+0.025'
    assert result['selected']['contact_state']['coverage']['geometric_coverage']
    assert result['selected']['contact_state']['collision_validation']['valid']
    assert result['selected']['contact_state']['attachment_pose_continuity_max_abs']<1e-12
    search=result['selected']['escape_search']
    assert search['termination']=='SUCCESS'
    assert search['selected_schedule_index']>0
    assert len(result['selected']['escape_attempts'])<=p['escape_path_attempt_limit']
    assert result['selected']['escape_attempts'][0]['status'].startswith('REJECTED')
    metrics=result['selected']['extraction_metrics']
    assert np.isclose(metrics['pure_straight_clearance_distance'],.32,atol=1e-8)
    assert metrics['escape_path_used']
    assert metrics['distance_until_first_escape_path']==0
    assert metrics['total_stack_release_distance'] < .02
    assert p['collision_margin_m']==.01


@pytest.mark.slow
def test_evaluate_task_rejects_actual_contact_endpoint_outside_coverage(monkeypatch):
    cfg=load_validation_config();p=cfg.data['planning']
    p['grasp_face_offset_candidates_m']=[0.0,0.025]
    p['grasp_tilt_candidates_rad']=[0.0]
    p['grasp_downstream_candidate_limit_per_strategy']=3
    p['escape_rotation_candidates_rad']=[0.0]
    cell=Cell(cfg);_,target,neighbors,valid=list(grid_tasks(cfg.data['scene']))[22]
    conveyor=cfg.data['conveyor'];belt=(conveyor['fixed_extension_m'],conveyor['fixed_z_m'])
    known_contact=np.array([-0.300479359612743,1.2937834890279378,0.4895834197945346,
                            3.666909416623987e-06,-0.7665782546700639,-4.411918905504988])
    outside_pose=cell.robot.fk(known_contact).copy()
    outside_pose[:3,3]+=target.rotation[:,0]*.2
    outside=cell.solve(outside_pose,[known_contact],99122,stage='test_outside_contact')
    assert outside.success
    original_cartesian=cell.cartesian

    def crossed_contact(start,destination,obstacles,seed,attachment=None,support_names=(),
                        target_contact=None,initial_proximity=None,stage='cartesian'):
        if target_contact is target:
            return [np.asarray(start),outside.q.copy()],None
        return original_cartesian(start,destination,obstacles,seed,attachment,support_names,
                                  target_contact,initial_proximity,stage)

    monkeypatch.setattr(cell,'cartesian',crossed_contact)
    result=evaluate_task(cell,target,[target,*neighbors],np.asarray(cfg.data['robot']['home_joints']),
        belt,seed=p['seed']+22,mode='fixed',only_face='top')

    rejected=[sub for attempt in result['attempts'] if attempt.get('strict_grasp_valid')
              for sub in attempt.get('conveyor_attempts',[])
              if sub.get('reason')=='FINAL_CONTACT_COVERAGE_FAILED']
    assert valid and rejected and not result['geometric_feasible']
    assert all(sub['stage']=='contact_validation' for sub in rejected)
    assert all('contact_state' not in sub and 'tcp_from_box' not in sub for sub in rejected)
    assert all(not sub['failure']['coverage']['geometric_coverage'] for sub in rejected)


@pytest.mark.slow
def test_real_handoff_uses_second_strict_valid_ik_branch_after_first_connection_fails():
    """Exercise actual M-710 IK, collision checks and RRT branch backtracking."""
    cfg=load_validation_config();cell=Cell(cfg)
    _,target,neighbors,valid=list(grid_tasks(cfg.data['scene']))[22]
    conveyor=cfg.data['conveyor'];belt=(conveyor['fixed_extension_m'],conveyor['fixed_z_m'])
    decks=cell.decks(belt);deck=decks[1]
    obstacles=[*cell.fixtures(),*neighbors,*decks]
    evidence=Path('docs/validation/evidence/m710id70_v3_hardening_baseline') / \
             'representative_success_grid_022.json'
    selected=json.loads(evidence.read_text(encoding='utf-8'))['task']['selected']
    start=np.asarray(selected['paths']['extraction'][-1])
    attachment=RigidAttachment(np.asarray(selected['tcp_from_box']),target.half_extents,target.name)
    disconnected=np.array([2.3017154929270216,-0.8216171444816843,3.271290421876069,
        3.1417028349347134,-0.6194600865396844,-3.872611302379414])
    connected=np.array([-0.8398753368245535,0.6932950816681044,-0.7987123516487091,
        0.000825148069595617,-0.07877058422026448,-3.8733456706593072])
    pose=cell.robot.fk(connected)
    endpoint_valid=lambda q:cell.validate_handoff_endpoint(q,obstacles,attachment,deck)[1] is None
    cell.p['ik_restarts']=0
    cell.p['stage_ik_candidate_limit']=2
    cell.p['stage_connection_attempt_limit']=2
    cell.p['stage_connection_iteration_budget']=800

    assert endpoint_valid(disconnected) and endpoint_valid(connected)
    candidate,path,failure,search=cell.connect_pose_candidates(
        pose,[disconnected,connected],9022,start,obstacles,10022,'carry_real_multisolution',
        endpoint_valid,attachment,[deck.name],mode='multi_solution')

    assert valid and failure is None
    assert np.allclose(candidate.q,connected,atol=1e-10)
    assert len(search['connection_attempts'])==2
    assert search['connection_attempts'][0]['failure']['reason']=='PATH_SEARCH_EXHAUSTED'
    assert search['connection_attempts'][1]['connection_success']
    assert np.allclose(path[0],start,atol=1e-12)
    assert np.allclose(path[-1],candidate.q,atol=1e-12)
    assert cell.path_failure(path,obstacles,attachment,[deck.name]) is None
    support,endpoint_failure=cell.validate_handoff_endpoint(path[-1],obstacles,attachment,deck)
    assert endpoint_failure is None and support['supported']
    assert search['shared_rrt_iterations_consumed']<=800
