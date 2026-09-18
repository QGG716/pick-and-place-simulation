"""Production pre-step / callback / post-step adjudication, without Isaac."""
from copy import deepcopy
import ast
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import pytest

from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.geometry import OBB
from unloading_sim.isaac_collision_policy import (
    ActiveContactPairIndex, ContactPathCache, PhysicalContactLedger, ZeroPointContactResolver,
    classify_poc_runtime_pair,
)
from unloading_sim.m710_replay_physics import ActualStackContactMonitor
from unloading_sim.stack_clearance import StackClearanceStep, copy_contact_points, paired_contact_key


def policy():
    return SimulationCollisionPolicy(schema='m710_poc_pair_collision_policy_v4',
        required_pair_clearance_m=.005, boundary_mode='finite_frozen_scene_walls',
        free_space_clearance_m=.0052, stack_contact_mode='planner_relaxed_physics_checked')


def states(gap):
    return [dict(name=n,prim_path='/'+n,center_m=c,quaternion_wxyz=[1,0,0,0],
                 linear_velocity_m_s=[0,0,0],angular_velocity_rad_s=[0,0,0],size_m=[.6,.4,.3])
            for n,c in [('target',[.6+gap,0,0]),('neighbor',[0,0,0])]]


def shapes():
    return {n:dict(name=n,actor='/'+n,collider='/'+n,source='VERIFIED_NATIVE_PHYSX_CUBE',
        half_extents_m=[.3,.2,.15],local_center_m=[0,0,0],local_rotation=np.eye(3).tolist(),
        contact_generation_offset_m=.01)
        for n in ['target','neighbor']}


def setup():
    p=policy(); s=states(0)
    monitor=ActualStackContactMonitor(OBB(s[0]['center_m'],[.3,.2,.15],np.eye(3),'target'),
        [OBB(s[1]['center_m'],[.3,.2,.15],np.eye(3),'neighbor')],p)
    gate=StackClearanceStep(shapes=shapes(),target='target',neighbors=['neighbor'],policy=p,
        world_id='world',task_id='task',maximum_wait_s=.1)
    return gate,monitor


def point(separation=.00006643665983574465, impulse=0):
    return dict(contact_point_separation_m=separation,position_m=[.3,0,0],normal=[1,0,0],
                impulse_ns=[impulse,0,0],face_index0=0,face_index1=0)


def start(gate,monitor,gap,step=0):
    gate.begin_step(step=step,time_s=step*.01,trajectory_time_s=step*.01,
        context=dict(stage='extraction',attached=True,actual_free_space=monitor.free_space_reached),
        states=states(gap))


def contact(gate,points=None,**kwargs):
    args=dict(actor0='/target',actor1='/neighbor',collider0='/target',collider1='/neighbor',
              event='CONTACT_PERSIST',points=[point()] if points is None else points)
    args.update(kwargs);return gate.collect(**args)


def finish(gate,monitor,gap):
    return gate.finish_step(states=states(gap),time_s=(gate.step+1)*.01,monitor=monitor,commanded_motion=True)


def test_six_mm_geometry_and_small_positive_contact_point_have_distinct_meanings():
    g,m=setup();start(g,m,.006);contact(g);result=finish(g,m,.006)
    assert not result['hold'] and result['stop_reason'] is None and m.free_space_reached
    assert g.last['contacts'][0]['points'][0]['contact_point_separation_m']==pytest.approx(.00006643666)
    assert g.last['contacts'][0]['geometry']['surface_distance_m']==pytest.approx(.006)
    old=classify_poc_runtime_pair(collider0='/target',collider1='/neighbor',
        minimum_separation_m=.00006643666,policy=policy(),robot_link_by_collider={},
        owned_tool_colliders=set(),stage='extraction')
    assert old['classification']=='CLEARANCE_INSUFFICIENT'


def test_entry_maintain_and_reapproach_share_the_same_measurement():
    g,m=setup();start(g,m,.0051);assert finish(g,m,.0051)['stop_reason'] is None
    assert not m.free_space_reached
    start(g,m,.00520000001,1);finish(g,m,.00520000001);assert m.free_space_reached
    start(g,m,.00500000001,2);assert finish(g,m,.00500000001)['stop_reason'] is None
    start(g,m,.004,3);contact(g)
    result=finish(g,m,.004)
    assert result['stop_reason']=='ACTUAL_FREE_TRANSIT_STACK_CLEARANCE_LOST'
    assert m.free_space_reached and len(g.transitions)==1


def test_intersection_only_has_existing_bounded_stage_permission():
    g,m=setup();start(g,m,-.001);contact(g,[point(-.001,1)])
    assert finish(g,m,-.001)['stop_reason'] is None
    start(g,m,-.011,1);contact(g,[point(-.011,1)])
    assert finish(g,m,-.011)['stop_reason'] is not None
    g,m=setup();start(g,m,.006);finish(g,m,.006)
    start(g,m,-.001,1);contact(g,[point(-.001,1)])
    assert finish(g,m,-.001)['stop_reason'] is not None
    assert m.free_space_reached


@pytest.mark.parametrize('p',[point(-.00001),point(.00001,1),{'invalid_contact_data':True}])
def test_unexplained_or_invalid_contact_is_not_erased_by_clearance_or_lost(p):
    g,m=setup();start(g,m,.006);finish(g,m,.006)
    start(g,m,.006,1);contact(g,[p]);r=finish(g,m,.006)
    assert r['hold'] and g.first_conflict
    for step in range(2,14):
        start(g,m,.006,step);contact(g,[],event='CONTACT_LOST');r=finish(g,m,.006)
    assert r['stop_reason'] and g.first_conflict


def test_pending_zero_points_transfers_explicitly_and_never_synthesizes_lost():
    g,m=setup();resolver=ZeroPointContactResolver();key=paired_contact_key('/target','/neighbor','/target','/neighbor')
    resolver.observe(key,[],False,('old-initialization',))
    resolver.transfer_verified_stack_pending(g)
    assert not resolver.pending_keys and len(g.transferred_pending)==1
    start(g,m,.006);contact(g,[]);assert finish(g,m,.006)['hold']
    assert not m.free_space_reached
    start(g,m,.006,1);contact(g);assert not finish(g,m,.006)['hold']
    assert m.free_space_reached and not g.pending
    assert all(c['event']!='CONTACT_LOST' for r in g.ring for c in r['contacts'])


@pytest.mark.parametrize('change',[lambda s:s.pop(),lambda s:s[0].update(prim_path='/wrong'),
    lambda s:s[0].update(center_m=[float('nan'),0,0]),lambda s:s[0].update(quaternion_wxyz=[0,0,0,0])])
def test_missing_invalid_or_wrong_physical_measurements_fail_closed(change):
    g,m=setup();s=states(.006);change(s)
    with pytest.raises(ValueError):
        g.begin_step(step=0,time_s=0,trajectory_time_s=0,context={},states=s)


def test_shape_mismatch_and_stale_step_do_not_pass():
    bad=shapes();bad['target']['source']='UNVERIFIED_PLAN'
    with pytest.raises(ValueError):StackClearanceStep(shapes=bad,target='target',neighbors=['neighbor'],
        policy=policy(),world_id='w',task_id='t',maximum_wait_s=.1)
    g,m=setup();start(g,m,.006);contact(g,collider0='/wrong')
    assert finish(g,m,.006)['hold']
    with pytest.raises(ValueError):start(g,m,.006,0)
    g,m=setup();start(g,m,.006)
    with pytest.raises(ValueError):g.finish_step(states=states(.006),time_s=0,monitor=m,commanded_motion=True)


def test_contact_point_outside_both_measured_bodies_is_unresolved_not_clear():
    g,m=setup();start(g,m,.006);p=point();p['position_m']=[100,0,0];contact(g,[p])
    assert finish(g,m,.006)['hold']
    assert g.first_conflict['reason']=='STACK_CONTACT_INVALID_MEASUREMENT'


def test_archived_failure_only_proves_saved_geometry_not_old_process_safety():
    import json
    from unloading_sim.serial_unloading import rotation_from_actual_quaternion
    from unloading_sim.pair_clearance import obb_surface_distance
    path=Path('docs/evidence/m710_poc_legal_release_handoff/second_carton_failure_fixture.json')
    f=json.loads(path.read_text())
    boxes=[OBB(x['position_m'],np.asarray(x['size_m'])/2,
        rotation_from_actual_quaternion(x['orientation_wxyz']),x['name']) for x in f['current_pair']]
    assert obb_surface_distance(*boxes)==pytest.approx(.005388677305227511,abs=1e-12)
    assert f['contact']['minimum_separation_m']==pytest.approx(.00006643665983574465,abs=1e-15)
    assert 'points' not in f['contact']  # no reconstructed within-step evidence


def test_pair_ownership_copy_lifetime_and_new_task_state():
    assert paired_contact_key('/z','/a','/first','/second')==('/a','/z','/second','/first')
    raw=SimpleNamespace(separation=.001,position=[1,2,3],normal=[1,0,0],impulse=[0,0,0],face_index0=0,face_index1=0)
    copied=copy_contact_points(SimpleNamespace(contact_data_offset=0,num_contact_data=1),[raw]);raw.position[0]=99
    assert copied[0]['position_m']==[1,2,3]
    g,m=setup();start(g,m,.006);contact(g);finish(g,m,.006)
    h,n=setup();assert not h.pending and not n.free_space_reached and not h.transitions
    assert not h.contains_pair('/target','/robot')


def test_frozen_measurements_are_not_mutated():
    g,m=setup();s=states(.006);before=deepcopy(s)
    g.begin_step(step=0,time_s=0,trajectory_time_s=0,
        context=dict(stage='extraction',attached=True,actual_free_space=False),states=s)
    g.finish_step(states=s,time_s=.01,monitor=m,commanded_motion=True)
    assert s==before


def test_real_runtime_callback_collects_stack_evidence_before_production_adjudication():
    from enum import IntEnum
    class Event(IntEnum):
        CONTACT_FOUND=1
        CONTACT_PERSIST=2
        CONTACT_LOST=3
    source=Path('scripts/isaacsim_fanuc_replay.py').read_text(encoding='utf-8')
    node=next(x for x in ast.walk(ast.parse(source)) if isinstance(x,ast.FunctionDef) and x.name=='_on_contact_report')
    g,m=setup();start(g,m,.006)
    paths={1:'/target',2:'/neighbor'};active=set()
    ns=dict(time=time,np=np,math=__import__('math'),ContactEventType=Event,
        root_prim_path='/robot',target_carton_path='/target',released_payload_path=None,ideal_actor_paths=set(),
        contact_pairs={},active_contacts=ActiveContactPairIndex(active),contact_path_cache=ContactPathCache(paths.__getitem__),
        contact_probe=None,physical_contact_ledger=PhysicalContactLedger(.003),contact_clock_s=[.01],
        contact_callback_wall_s=[0.],contact_trajectory_clock_s=[0.],contact_callback_header_count=[0],
        current_pair_separations={},stack_clearance_step=g,copy_contact_points=copy_contact_points,
        paired_contact_key=paired_contact_key,effective_collision_policy=policy())
    exec(compile(ast.Module(body=[node],type_ignores=[]),'actual_runtime_callback','exec'),ns)
    header=SimpleNamespace(actor0=1,actor1=2,collider0=1,collider1=2,type=Event.CONTACT_PERSIST,contact_data_offset=0,num_contact_data=1)
    p=SimpleNamespace(separation=.00006643666,position=[.3,0,0],normal=[1,0,0],impulse=[0,0,0],face_index0=0,face_index1=0)
    ns['_on_contact_report']([header],[p])
    # No replacement classifier exists in this namespace: the actual callback
    # must route this pair to the same production step adjudicator used in Isaac.
    assert len(g.contacts)==1 and ns['contact_pairs']
    assert not finish(g,m,.006)['hold'] and m.free_space_reached


def test_real_runtime_action_conditions_respect_unresolved_production_hold():
    tree=ast.parse(Path('scripts/isaacsim_fanuc_replay.py').read_text(encoding='utf-8'))
    tests=[n.test for n in ast.walk(tree) if isinstance(n,ast.If)]
    def condition(required):
        found=[t for t in tests if required <= {n.id for n in ast.walk(t) if isinstance(n,ast.Name)}
               and not any(isinstance(n,ast.Name) and n.id=='requested_duration' for n in ast.walk(t))]
        assert len(found)==1
        return compile(ast.Expression(found[0]),'production_action_condition','eval')
    grasp=condition({'grasp_event_time','target_body','grasp_commanded'})
    release=condition({'release_event_time','grasp_enabled','release_commanded'})
    takeover=condition({'ideal_reception_mode','release_open_confirmed','assumed_reception_state'})
    g,m=setup();start(g,m,.006);finish(g,m,.006)
    start(g,m,.006,1);contact(g,[point(-.001)]);assert finish(g,m,.006)['hold']
    ns=dict(stack_clearance_step=g,target_body=object(),grasp_enabled=True,grasp_commanded=False,release_commanded=False,
            grasp_event_time=1.,release_event_time=1.,trajectory_time=2.,ideal_reception_mode=True,
            release_open_confirmed=True,assumed_reception_state=None)
    assert not eval(grasp,ns) and not eval(release,ns) and not eval(takeover,ns)

    g.hold=False  # the same production branches open after a resolved step
    assert eval(grasp,ns) and eval(release,ns) and eval(takeover,ns)


def slide_states(gap):
    data=states(gap)
    data[0].update(center_m=[-.6-gap,0,.44999965],linear_velocity_m_s=[-.022,0,0])
    data[1].update(center_m=[0,0,.15])
    return data


def slide_setup():
    g,_=setup();data=slide_states(-.6)
    m=ActualStackContactMonitor(OBB(data[0]['center_m'],[.3,.2,.15],np.eye(3),'target'),
        [OBB(data[1]['center_m'],[.3,.2,.15],np.eye(3),'neighbor')],policy())
    return g,m


def slide_contact():
    # Vertical support feature while the upper box slides out horizontally.
    # -0.00035 mm = -0.35 micrometres; NOT -0.35 mm.
    return dict(contact_point_separation_m=-.00000035,position_m=[-.3,0,.3],
        normal=[0,0,1],impulse_ns=[0,0,.05],face_index0=0,face_index1=0)


def slide_step(g,m,pre,post,step,points=None,event='CONTACT_PERSIST'):
    g.begin_step(step=step,time_s=step*.01,trajectory_time_s=step*.01,
        context=dict(stage='extraction',attached=True,actual_free_space=m.free_space_reached),
        states=slide_states(pre),world_id='world',task_id='task')
    contact(g,[slide_contact()] if points is None else points,event=event)
    return g.finish_step(states=slide_states(post),time_s=(step+1)*.01,monitor=m,commanded_motion=True)


@pytest.mark.parametrize('gap,free',[(.00519,False),(.00521,True)])
def test_licensed_contact_ending_is_not_made_invalid_by_better_geometry(gap,free):
    g,m=slide_setup();r=slide_step(g,m,.00499,gap,0)
    assert not r['hold'] and r['stop_reason'] is None and m.free_space_reached==free
    assert g.last['contacts'][0]['contact_response_observed']
    assert g.first_conflict is None


def test_persistent_observable_feature_after_exit_requires_origin_and_continuity():
    g,m=slide_setup();slide_step(g,m,.00499,.00521,0)
    r=slide_step(g,m,.00521,.00543,1)
    assert not r['hold'] and m.free_space_reached
    assert g.last['contacts'][0]['phase_contact_permission'] is None
    assert g.last['contacts'][0]['explanation']=='PERSISTING_LICENSED_FEATURE_WITH_CONTINUOUS_CLEAR_SOLIDS'
    # A new response after LOST is not silently classified as the old feature.
    slide_step(g,m,.00543,.00565,2,points=[],event='CONTACT_LOST')
    assert slide_step(g,m,.00565,.00587,3,event='CONTACT_FOUND')['hold']
    assert g.first_conflict and g.pending


def test_missing_features_can_be_completed_without_erasing_history_or_faking_lost():
    g,m=slide_setup();slide_step(g,m,.00499,.00521,0)
    assert slide_step(g,m,.00521,.00543,1,points=[])['hold']
    first=deepcopy(g.first_conflict)
    r=slide_step(g,m,.00543,.00565,2)
    assert not r['hold'] and not g.pending and g.first_conflict==first
    assert g.issue_history[0]['status']=='EXPLAINED'
    assert g.issue_history[0]['resolved_step']==2
    assert g.counts['issues_explained']==1
    assert all(c['event']!='CONTACT_LOST' for row in g.ring for c in row['contacts'])


def test_unresolved_origin_cannot_be_cleared_by_new_geometry_or_time():
    g,m=slide_setup()
    g.begin_step(step=0,time_s=0,trajectory_time_s=0,
        context=dict(stage='extraction',attached=True,actual_free_space=False),states=slide_states(.006))
    g.finish_step(states=slide_states(.006),time_s=.01,monitor=m,commanded_motion=True)
    r=slide_step(g,m,.006,.00622,1)
    assert r['hold']
    for i in range(2,14):
        r=slide_step(g,m,.00622+(i-2)*.00022,.00622+(i-1)*.00022,i)
    assert r['stop_reason']=='STACK_CONTACT_ORIGIN_OR_CONTINUITY_UNRESOLVED'
    assert g.issue_history[0]['status']=='PENDING_REVIEW'


def test_confirmed_geometry_violation_stays_failed_after_lost_and_separation():
    g,m=slide_setup();slide_step(g,m,.00499,.00521,0)
    r=slide_step(g,m,.00521,.004,1)
    reason=r['stop_reason'];assert reason=='ACTUAL_FREE_TRANSIT_STACK_CLEARANCE_LOST'
    r=slide_step(g,m,.004,.006,2,points=[],event='CONTACT_LOST')
    assert r['stop_reason']==reason and g.confirmed_violation and m.free_space_reached


@pytest.mark.parametrize('field,value',[('world_id','wrong'),('task_id','wrong')])
def test_measurement_binding_rejects_wrong_world_or_task(field,value):
    g,m=setup();args=dict(step=0,time_s=0,trajectory_time_s=0,context={},states=states(.006),
        world_id='world',task_id='task');args[field]=value
    with pytest.raises(ValueError,match='world/task'):g.begin_step(**args)


def test_discontinuous_pose_or_changed_negative_feature_is_not_explained_as_old():
    g,m=slide_setup();slide_step(g,m,.00499,.00521,0)
    assert slide_step(g,m,.006,.00622,1)['hold']  # no previous readback continuity
    g,m=slide_setup();slide_step(g,m,.00499,.00521,0)
    p=slide_contact();p['contact_point_separation_m']=-.009
    assert slide_step(g,m,.00521,.00543,1,points=[p])['hold']
