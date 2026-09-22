from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.motion_validation import ValidationContext,MotionValidator,RequestBudget,Status,ValidationResult,LRU
from unloading_sim.planner import RRTConnectPlanner
from unloading_sim.validation_kernel import batch_kinematics,point_motion_bound,certify_pair
from unloading_sim.robot import URDFRobot,URDFJoint


def fixture():
    # Legacy resolution .5 samples x=0,.5,1, missing the thin x=.25 obstacle.
    def failure(q):
        return {'reason':'THIN_OBSTACLE','pair':['robot','thin']} if abs(q[0]-.25)<.022 and abs(q[1])<.18 else None
    c=ValidationContext.create({'scene':'thin_detour','margin':.005},resolution_rad=.5)
    return MotionValidator(c,failure)


def planner(v,**kw):
    return RRTConnectPlanner(np.array([0.,-.5]),np.array([1.,.5]),lambda q:v.scalar(q) is None,
        edge_resolution=.5,step_size=.15,max_iterations=600,rng=np.random.default_rng(9),motion_validator=v,**kw)


def test_coarse_miss_is_reproduced_then_real_rrt_finds_checked_detour():
    a,b=np.array([0.,0.]),np.array([1.,0.]);v=fixture()
    legacy=RRTConnectPlanner(np.array([0.,-.5]),np.array([1.,.5]),lambda q:v.scalar(q) is None,edge_resolution=.5)
    assert legacy.plan(a,b).message=='direct edge'
    assert v.check_motion(a,b).status==Status.INVALID
    result=planner(v).plan(a,b)
    assert result.success and result.iterations>0 and len(result.path)>2
    assert v.check_path(result.path).valid
    assert result.search_evidence['direct_rejected_continue_search']==1
    assert result.search_evidence['direct_edge_validation']['failure']['pair']==['robot','thin']
    before=v.statistics['state_samples']
    assert not v.check_motion(a,b).valid
    assert v.statistics['state_samples']==before
    assert v.statistics['repeated_failed_edges']>=2


def test_budget_cancel_unknown_never_unreachable_or_cached_invalid():
    v=fixture();p=planner(v,request_budget=RequestBudget(max_checks=2))
    r=p.plan([0,0],[1,0]);assert not r.success
    assert r.search_evidence['validation_status']=='INDETERMINATE'
    assert v.check_motion([0,0],[1,0],RequestBudget(cancelled=lambda:True)).status==Status.CANCELLED
    unknown=MotionValidator(v.context,lambda q:{'reason':'DISTANCE','classification':'UNKNOWN'})
    assert unknown.check_motion([0,0],[1,0]).status==Status.INDETERMINATE
    assert not unknown.edges and not unknown.states


@pytest.mark.parametrize('field',['scene','attachment','stage','mask','margin','time','model'])
def test_context_and_exact_directional_parameter_cache(field):
    binding={field:0};c=ValidationContext.create(binding,resolution_rad=.1)
    binding[field]=1
    other=ValidationContext.create(binding,resolution_rad=.1)
    assert other.context_id!=c.context_id
    v=MotionValidator(c,lambda q:None,context_current=lambda:other.context_id)
    assert v.check_motion([0],[1]).status==Status.INDETERMINATE
    v=MotionValidator(c,lambda q:None)
    assert not v.check_motion([0],[.1],parameters={'duration':1}).cache_hit
    assert not v.check_motion([.1],[0],parameters={'duration':1}).cache_hit
    assert not v.check_motion([0],[.1],parameters={'duration':2}).cache_hit
    assert v.check_motion([0],[.1],parameters={'duration':1}).cache_hit
    assert v.check_motion([0],[.1],parameters={'duration':1}).statistics['state_samples']==0
    assert replace(c,interpolation='spline').context_id!=c.context_id


def test_consistency_defect_not_hidden_by_restart():
    v=MotionValidator(ValidationContext.create({},resolution_rad=.1),lambda q:None)
    assert v.check_motion([0],[1]).valid
    bad=ValidationResult(Status.INVALID,v.context.context_id,failure={'reason':'COLLISION'})
    with pytest.raises(RuntimeError,match='CONSISTENCY_DEFECT'): v.feedback_failure([0],[1],bad)
    with pytest.raises(RuntimeError,match='CONTEXT_CHANGED'): v.feedback_failure([0],[1],replace(bad,context_id='other'))


def test_candidate_rejection_restarts_inside_original_budget():
    v=fixture();budget=RequestBudget(max_checks=4000)
    calls=[]
    def candidate(path,budget):
        calls.append(budget.checks)
        if len(calls)==1:
            return ValidationResult(Status.INVALID,v.context.context_id,failure={
                'reason':'DOWNSTREAM_CURVE_COLLISION','type':'GEOMETRY','edge':0,
                'motion_parameters':{'curve':'distinct_checked_downstream_curve'}})
        return v.check_path(path,budget)
    r=planner(v,request_budget=budget,candidate_check=candidate).plan([0,0],[1,0])
    assert r.success and len(calls)>=2 and calls[1]>calls[0]
    assert r.search_evidence['bounded_tree_restarts']==1
    assert budget.checks<=4000


def test_optional_invalid_shortcut_keeps_original_and_narrow_passage():
    v=fixture();p=planner(v)
    path=[np.array(q,float) for q in ([0,0],[0,.181],[1,.181],[1,0])]
    assert v.check_path(path).valid
    result,e=p.bounded_shortcut(path,deadline=None,attempts=1,state_budget=100)
    assert len(result)==4 and v.check_path(result).valid
    # Failure record does not poison a 1 mm clearance route nearby.
    assert v.check_motion([0,.181],[1,.181]).valid


def test_sweeps_both_bodies_corners_margin_and_unknown_proof():
    # A long rotating tool/payload corner moves far even at a fixed TCP.
    bound=point_motion_bound([np.pi/2],[2.])
    actual=np.linalg.norm(np.array([0,2.])-np.array([2.,0]))
    assert bound>=actual
    assert not certify_pair(3.,bound,0,.005)
    assert not certify_pair(.02,.01,.01,.005)  # both robot self bodies move
    assert not certify_pair(.005,0,0,.005)
    assert not certify_pair(np.nan,0,0,.005)
    assert certify_pair(.02000001,.005,.005,.005)
    c=ValidationContext.create({},resolution_rad=.1,guarantee='CONTINUOUS')
    v=MotionValidator(c,lambda q:None)
    assert v.check_motion([0],[1]).status==Status.INDETERMINATE
    v.interval_proof=lambda a,b:dict(context_id=c.context_id,complete_contract=True)
    assert v.check_motion([0],[1]).valid
    assert v.statistics['certified_intervals']==1


def test_native_batch_fk_matches_scalar_rotations_and_translation():
    joints=[URDFJoint('a','revolute','base','one',np.eye(4),np.array([0.,0,1]),(-3,3)),
            URDFJoint('b','revolute','one','tip',np.array([[1.,0,0,1],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),np.array([0.,1,0]),(-3,3))]
    robot=URDFRobot(joints,['a','b'],'base','tip')
    q=np.array([[0,0],[1,-.2],[-2,1],[3,3]])
    links,_,tcp,jac=batch_kinematics(robot,q)
    for i,state in enumerate(q):
        np.testing.assert_allclose(tcp[i],robot.fk(state),atol=1e-14)
        np.testing.assert_allclose(jac[i],robot.geometric_jacobian(state),atol=1e-14)
        for name,pose in robot.named_link_frames(state).items():
            np.testing.assert_allclose(links[name][i],pose,atol=1e-14)
    with pytest.raises(ValueError):batch_kinematics(robot,[[np.nan,0]])
    with pytest.raises(ValueError):batch_kinematics(robot,[[0]])


def test_lru_is_incremental_and_parent_cancellation_checked_after_native_batch():
    cache=LRU(2);cache.put(1,1);cache.put(2,2);cache.lookup(1);cache.put(3,3)
    assert list(cache)==[1,3] and cache.evictions==1
    cancelled=[False];parent=RequestBudget(cancelled=lambda:cancelled[0])
    def batch(q,proof=None):cancelled[0]=True;return [None]*len(q)
    v=MotionValidator(ValidationContext.create({},resolution_rad=.1),lambda q:None,check_states=batch)
    assert v.check_motion([0],[1],RequestBudget(parent=parent)).status==Status.CANCELLED
