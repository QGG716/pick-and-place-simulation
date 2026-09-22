"""Prefix work accounting and real production context invalidation."""
from dataclasses import replace
import numpy as np
import pytest
from unloading_sim.motion_validation import MotionValidator,ValidationContext,RequestBudget,Status
from test_validation_kernel_official import official_scene
from test_lookahead_contact import START_Q,CONTACT_Q


@pytest.mark.parametrize('failed',[0,4,9,17,None])
def test_ordered_prefix_across_batches_and_no_suffix_cache(failed):
    observed=[];prepared=[]
    def state(q):
        index=int(round(q[0]*2));observed.append(index)
        return {'reason':'COLLISION'} if index==failed else None
    def prefix(qs,proof=None,interrupted=lambda:None):
        prepared.append(len(qs));result=[]
        for q in qs:
            if interrupted():break
            f=state(q);result.append(f)
            if f is not None:break
        return result
    v=MotionValidator(ValidationContext.create({},resolution_rad=1),state,check_prefix=prefix)
    result=v.check_motion([0],[32])
    count=65 if failed is None else failed+1
    assert observed==list(range(count))
    assert v.statistics['state_samples']==count
    assert len(v.states)==count
    assert max(prepared)<=8
    assert sum(prepared)-count<=7
    if failed is None: assert result.valid
    else:
        assert result.status==Status.INVALID
        assert result.failure['first_failure_sample']==failed
        assert result.statistics['skipped_suffix_states']==64-failed
        assert v.check_motion([0],[32]).cache_hit
        assert len(observed)==count
    # All-result API still evaluates and returns every requested result.
    w=MotionValidator(v.context,state)
    assert len(w.check_states([[0],[2],[10]]))==3


@pytest.mark.parametrize('interrupt',['cancel','deadline','context','budget'])
def test_prefix_interruptions_never_cache_unobserved_or_stale(interrupt):
    context=ValidationContext.create({},resolution_rad=1);current=[context.context_id]
    cancelled=[False];seen=[];budget=RequestBudget(max_checks=3 if interrupt=='budget' else None,
        cancelled=lambda:cancelled[0])
    def state(q):
        seen.append(float(q[0]))
        if len(seen)==3:
            if interrupt=='cancel':cancelled[0]=True
            if interrupt=='deadline':budget.deadline=0
            if interrupt=='context':current[0]='changed'
        return None
    v=MotionValidator(context,state,context_current=lambda:current[0])
    result=v.check_motion([0],[32],budget)
    assert result.status==(Status.CANCELLED if interrupt=='cancel' else Status.INDETERMINATE)
    assert len(seen)==3
    assert not v.edges
    assert len(v.states)<=3
    if interrupt=='context':assert not v.states


@pytest.mark.parametrize('change',['scene','attachment','mask','margin','mount','tool','joint_origin','revision','policy'])
def test_real_production_dependencies_invalidate_warm_results(official_scene,change):
    from unloading_sim.layout_trajectory import PhysicalContactAttachment
    from unloading_sim.validation_physics import RigidAttachment
    scene,c=official_scene;c.start_planning_request();c._deadline_monotonic=None;c.validation_budget.deadline=None
    obstacles=list(scene.all_obstacles)
    target=scene.cartons[0]
    rigid=RigidAttachment.capture(c.physical_from_virtual(c.robot.fk(START_Q)),target)
    attachment=PhysicalContactAttachment(c.robot,rigid,c.flange_from_virtual_task_tcp.copy(),c.flange_from_physical_contact.copy())
    options=dict(stage='transit',attachment=attachment)
    v=c._motion_validator(obstacles,**options)
    # INVALID is also a real cached verdict; both statuses must check the lease.
    first=v.check_motion(START_Q,START_Q+.000001)
    assert first.status in (Status.VALID,Status.INVALID)
    assert v.check_motion(START_Q,START_Q+.000001).cache_hit
    snapshot=v.context.binding_json
    tool=c.validation_kernel.urdf;rv=c.robot_state_validator
    if change in {'scene','attachment','mount','tool','joint_origin'}:
        array={'scene':obstacles[-1].center,'attachment':rigid.tcp_from_box,
               'mount':tool.base_transform,'tool':tool.tool_collision_local_boxes,
               'joint_origin':tool.joints[1].origin}[change]
        old=array.copy();array.flat[0]+=.0001
        restore=lambda:np.copyto(array,old)
    else:
        owner,name,value={'mask':(rv,'commanded_cup_mask',[False]*72),
            'margin':(c,'collision_margin_m',c.collision_margin_m+.001),
            'revision':(c.robot,'geometry_revision',getattr(c.robot,'geometry_revision',0)+1),
            'policy':(c,'collision_policy',replace(c.collision_policy,compliant_cup_neighbor_contact_mode='check'))}[change]
        existed=hasattr(owner,name);old=getattr(owner,name,None);setattr(owner,name,value)
        restore=lambda:setattr(owner,name,old) if existed else delattr(owner,name)
    try:
        assert v.check_motion(START_Q,START_Q+.000001).status==Status.INDETERMINATE
        assert v.context.binding_json==snapshot
        assert c._motion_validator(obstacles,**options).context.context_id!=v.context.context_id
    finally:restore()
    assert v.check_motion(START_Q,START_Q+.000001).status==Status.INDETERMINATE


def test_production_guard_avoids_rebuilding_context_and_discards_late_work(official_scene,monkeypatch):
    scene,c=official_scene;c.start_planning_request();c._deadline_monotonic=None;c.validation_budget.deadline=None
    obstacles=list(scene.all_obstacles);v=c._motion_validator(obstacles,stage='pregrasp')
    builds=c.context_statistics['full_context_builds']
    a=START_Q;b=a+.000001
    assert v.check_motion(a,b).valid
    assert c._motion_validator(obstacles,stage='pregrasp') is v
    assert v.check_motion(a,b).cache_hit
    assert c.context_statistics['full_context_builds']==builds
    center=obstacles[-1].center;old=center.copy()
    original=c._state_failure
    def mutate(q,*args,**kwargs):
        result=original(q,*args,**kwargs);center[0]+=.001;return result
    monkeypatch.setattr(c,'_state_failure',mutate)
    try:
        result=v.check_motion(a+.000002,b+.000002)
        assert result.status==Status.INDETERMINATE
        assert not v.states and not v.edges and not c.validation_kernel.cache
    finally:np.copyto(center,old)


@pytest.mark.parametrize('failure_index',[0,4,9])
def test_production_stateful_prefix_does_not_advance_after_failure(official_scene,monkeypatch,failure_index):
    from unloading_sim.validation_physics import InitialProximityTracker
    from unloading_sim.geometry import OBB
    scene,c=official_scene;c.start_planning_request();c._deadline_monotonic=None;c.validation_budget.deadline=None
    payload=OBB(np.zeros(3),np.full(3,.1),np.eye(3),'payload')
    neighbor=OBB(np.array([.205,0,0]),np.full(3,.1),np.eye(3),'neighbor')
    tracker,error=InitialProximityTracker.capture(payload,[neighbor],.005,.0001,.0001)
    assert error is None
    seen=[]
    # Inject at the actual production state dispatch; track ordered side effects
    # across prefix batches. Ordinary runtime has no fault-injection branch.
    def injected(q,*args,**kwargs):
        assert kwargs['initial_proximity'] is tracker
        seen.append(np.asarray(q).copy())
        position=np.array([.01,0,0]) if len(seen)==failure_index+1 else np.zeros(3)
        return tracker.state_failure(OBB(position,payload.half_extents,payload.rotation,'payload'),[neighbor])
    monkeypatch.setattr(c,'_state_failure',injected)
    v=c._motion_validator(scene.all_obstacles,stage='extraction',initial_proximity=tracker)
    b=START_Q.copy();b[0]+=.004
    result=v.check_motion(START_Q,b)
    assert result.status==Status.INVALID and len(seen)==failure_index+1
    assert tracker.pairs['neighbor'].samples==failure_index
    assert not v.states and not v.edges


def test_lazy_geometry_is_readonly_and_matches_plane_boundaries(official_scene):
    scene,c=official_scene;kernel=c.validation_kernel;kernel.context_id='lazy-regression';kernel.cache.clear()
    before=kernel.statistics['tool_obb_constructions']
    prepared=kernel.prepare_states([START_Q,CONTACT_Q])
    assert kernel.statistics['tool_obb_constructions']==before
    first=prepared[START_Q.tobytes()];second=prepared[CONTACT_Q.tobytes()]
    for group in (first.rigid,first.cups,first.compressed_cups):
        low,high=group.plane_bounds()
        for i,box in enumerate(group):
            np.testing.assert_array_equal(low[i],box.corners().min(0))
            np.testing.assert_array_equal(high[i],box.corners().max(0))
            with pytest.raises(ValueError):box.center[0]=0
            with pytest.raises(ValueError):box.rotation[0,0]=0
            assert box.world_from_local is box.world_from_local
            with pytest.raises(ValueError):box.world_from_local[0,3]=0
    original=first.rigid[0].center.copy()
    _=second.rigid[0]
    np.testing.assert_array_equal(first.rigid[0].center,original)
    assert first.rigid[0] is first.rigid[0]
