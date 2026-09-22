"""Small official-mesh differential: native batch vs unchanged scalar predicates."""
import numpy as np
import pytest
from test_lookahead_contact import START_Q, CONTACT_Q
from unloading_sim.validation_kernel import batch_kinematics
from unloading_sim.motion_validation import RequestBudget,Status


@pytest.fixture(scope='module')
def official_scene():
    from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
    policy=load_layout_motion_policy('configs/validation/m710id70_handoff_continuation.yaml')
    scene=build_verified_motion_input(policy)
    build=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot())
    assert build.connector is not None,build.evidence
    assert build.connector.collision_policy.poc_pair_clearance
    return scene,build.connector


def test_official_native_fk_jacobian_geometry_and_scalar_verdicts(official_scene):
    scene,c=official_scene
    c.start_planning_request()
    kernel=c.validation_kernel;mesh=c.robot;urdf=kernel.urdf
    qs=np.array([START_Q,CONTACT_Q,np.zeros(6),START_Q+.001])
    kernel.context_id='official-differential'
    original=[]
    tool_original=[]
    for q in qs:
        mesh._update_geometry(q)
        original.append((mesh.fk(q).copy(),mesh.geometric_jacobian(q).copy(),
            np.array([mesh.base_transform@mesh._matrix(p) for p in mesh.geometry_data.oMg])))
        tool_original.append([*urdf.tool_collision_obbs(q),*c.robot_state_validator._compliant_boxes(q)])
    with kernel.scope(qs):
        for q,(tcp,jac,geometry) in zip(qs,original):
            state=kernel.get(q)
            np.testing.assert_allclose(state.tcp,tcp,atol=2e-14)
            np.testing.assert_allclose(state.jacobian,jac,atol=2e-14)
            np.testing.assert_allclose(state.geometry,geometry,atol=2e-14)
        for q,reference in zip(qs,tool_original):
            state=kernel.get(q)
            for actual,expected in zip([*state.rigid,*state.compressed_cups],reference):
                np.testing.assert_allclose(actual.corners(),expected.corners(),atol=2e-14)
                lower,upper=state.obstacle_aabbs[id(actual)]
                assert np.all(lower<=expected.corners().min(0))
                assert np.all(upper>=expected.corners().max(0))
    samples=[*qs,np.full(6,1e4),np.full(6,np.nan),np.zeros(5)]
    def fresh():
        c._state_cache.clear();c.robot_state_validator._static_cache.clear()
        c.robot_state_validator._geometry_cache.clear()
    fresh()
    scalar=[c._state_failure(q,scene.all_obstacles,stage='pregrasp') for q in samples]
    fresh()
    context=c._validation_context(scene.all_obstacles,stage='pregrasp')
    kernel.context_id=context.context_id
    native=kernel.check_states(samples,lambda q:c._state_failure(q,scene.all_obstacles,stage='pregrasp'))
    assert [None if r is None else (r['reason'],r.get('pair')) for r in native]==[
        None if r is None else (r['reason'],r.get('pair')) for r in scalar]
    assert kernel.statistics['native_batches']>0


def test_actual_connector_shared_motion_contract_and_interval_proofs(official_scene):
    scene,c=official_scene;c.start_planning_request()
    c._deadline_monotonic=None;c.validation_budget.deadline=None
    a=START_Q;b=a+np.array([.0001,0,0,0,0,0])
    validator=c._motion_validator(scene.all_obstacles,stage='pregrasp')
    result=validator.check_motion(a,b,RequestBudget())
    assert result.valid,result.evidence()
    assert c._path_failure([a,b],scene.all_obstacles,stage='pregrasp') is None
    assert validator.check_motion(a,b).cache_hit
    # Complete stage constraints are discrete; pair certificates do not inflate guarantee.
    assert result.guarantee=='DISCRETE_LEGACY_STRICT'
    kernel=c.validation_kernel
    if c.collision_policy.poc_pair_clearance:
        proof=kernel.prepare_context(validator.context,scene.all_obstacles,stage='pregrasp')(a,b)
        assert proof['pairs'] and not proof['complete_contract']
        # Every declared pair lower bound must also survive the independent strict scan.
        native=validator.check_motion(a,b)
        scalar=[c._state_failure(q,scene.all_obstacles,stage='pregrasp')
                for q in np.linspace(a,b,validator.context.samples(a,b)+1)]
        assert native.valid and not any(scalar)


def test_certified_diagonal_interval_skips_actual_narrow_phase():
    from test_pinocchio_broadphase import _backend,_check
    from unloading_sim.geometry import OBB
    from unloading_sim.validation_kernel import aabb_distance_lower,certify_pair
    backend=_backend()
    obstacle=OBB([.603,.603,.603],[.1,.1,.1],np.eye(3),'diagonal')
    # Each axis gap < margin, so the original broadphase needs an exact query.
    assert not _check(backend,[obstacle],margin=.005).in_collision
    assert backend.exact_call_count==1
    lower=aabb_distance_lower((np.full(3,-.5),np.full(3,.5)),
        (obstacle.corners().min(0),obstacle.corners().max(0)))
    assert certify_pair(lower,0,0,.005)
    backend._interval_pairs={('mesh','mesh','diagonal')}
    backend.performance_counters={}
    assert not _check(backend,[obstacle],margin=.005).in_collision
    assert backend.exact_call_count==1
    assert backend.performance_counters['interval_pair_skips']==1
