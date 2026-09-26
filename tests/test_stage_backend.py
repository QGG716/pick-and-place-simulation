"""CPU contracts/control-flow tests. Fake candidates are explicitly not GPU evidence."""
from copy import deepcopy
import json
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from unloading_sim.stage_backend import StageRequest,SCHEMA_VERSION,run_stage,trajectory_failure,pose_wxyz
from unloading_sim.stage_export import box_spheres,released_world


def request():
    return StageRequest(dict(request_id='unit',schema_version=SCHEMA_VERSION,stage='TRANSIT',scene_revision=1,
        scene_fingerprint='scene',robot_model_fingerprint='robot',tool_fingerprint='tool',payload_fingerprint='payload',
        collision_policy_fingerprint='policy',joint_names=['a','b'],q_start=[0.,0.],q_goal=[1.,0.],
        limits=dict(lower=[-2,-2],upper=[2,2],velocity=[2,2],effort=[10,10],acceleration=[3,3],jerk=[10,10]),
        transforms={'world_from_base':np.eye(4).tolist()},payload=dict(object_id='box',mass_kg=42.5,
        flange_from_object=np.eye(4).tolist(),dimensions_m=[.6,.4,.3],com_xyz_m=[0,0,0],
        inertia_tensor_com_kg_m2=np.diag([.4,.5,.6]).tolist()),seed=7,resources=dict(attempts=3,num_seeds=4),
        collision_policy={},goal_tolerance_rad=1e-4))


def path(mid=0.):
    return dict(q=[[0.,0.],[.5,mid],[1.,0.]],time_s=[0.,1.,2.],joint_names=['a','b'],
                valid_length=3,interpolation='linear_joint_samples',dq=None,ddq=None)


def native(mid=0.):
    return dict(status='CANDIDATE_GENERATED',backend={'name':'MOCK'},trajectory=path(mid))


def run(candidate=lambda r,a:native(),authority=lambda t:None,**kwargs):
    return run_stage(kwargs.pop('req',request()),candidate,authority,kwargs.pop('state',lambda q:None),
                     kwargs.pop('revision',lambda:(1,'scene')),kwargs.pop('cancelled',lambda:False))


def test_core_import_does_not_load_gpu():
    import subprocess
    subprocess.run([sys.executable,'-c',
        "import sys; import unloading_sim.curobo_v2_backend; assert 'curobo' not in sys.modules; assert 'torch' not in sys.modules"],check=True)


def test_json_roundtrip_and_snapshot_copy():
    req=request(); raw=req.to_dict(); raw['q_start'][0]=1
    assert req.data['q_start'][0]==0
    assert StageRequest(json.loads(json.dumps(req.data))).data==req.data


@pytest.mark.parametrize('key,value',[('q_goal',[1]),('joint_names',['a','a']),('q_start',[float('nan'),0]),
                                      ('schema_version','unknown'),('goal_tolerance_rad',0)])
def test_bad_request(key,value):
    d=request().to_dict();d[key]=value
    with pytest.raises(ValueError):StageRequest(d)


@pytest.mark.parametrize('angle',[0.,.7,np.pi,-np.pi/2])
def test_explicit_wxyz(angle):
    from unloading_sim.serial_unloading import rotation_from_actual_quaternion
    m=np.eye(4);m[:3,:3]=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
    m[:3,3]=[1,2,3];pose=pose_wxyz(m)
    assert pose[:3]==[1,2,3]
    np.testing.assert_allclose(rotation_from_actual_quaternion(pose[3:]),m[:3,:3],atol=1e-12)


def test_box_corner_cover_and_release_identity():
    spheres=box_spheres([.6,.4,.3],.08)
    for x in (-.3,.3):
        for y in (-.2,.2):
            for z in (-.15,.15):
                assert min(np.linalg.norm(np.array([x,y,z])-s['center'])-s['radius'] for s in spheres)<=0
    b={'request':request().to_dict(),'obstacles':[{'name':'neighbor'}]}
    out=released_world(b,np.eye(4))
    assert out['attached_object'] is None
    assert [x['name'] for x in out['obstacles']]==['neighbor','box']


def test_authority_rejection_continues_search():
    calls=[]
    def candidate(req,attempt):calls.append(attempt);return native(attempt*.1)
    result=run(candidate,lambda t: {'reason':'middle_edge'} if t['q'][1][1]==0 else None)
    assert calls==[0,1]
    assert result['authority_accepted']
    assert result['first_failure']['reason']=='middle_edge'
    assert not result['delivered'] and not result['isaac_execution_completed'] and not result['fallback']


def test_duplicate_rejected_path_does_not_repeat_authority():
    calls=[]
    result=run(authority=lambda t:calls.append(1) or {'reason':'collision'})
    assert len(calls)==1 and len(result['attempts'])==3
    assert result['status']=='AUTHORITY_REJECTED'


@pytest.mark.parametrize('mode,expected',[('cancel','CANCELLED'),('stale','SCENE_STALE'),('deadline','TIMEOUT')])
def test_discard_after_synchronous_solve(mode,expected):
    state={'done':False};req=request()
    def candidate(r,a):
        state['done']=True
        if mode=='deadline':r.data['deadline_monotonic']=0
        return native()
    result=run(candidate,req=req,cancelled=lambda: mode=='cancel' and state['done'],
        revision=lambda:(2,'scene') if mode=='stale' and state['done'] else (1,'scene'))
    assert result['status']==expected
    assert not result['authority_accepted'] and not result['delivered']


@pytest.mark.parametrize('change,reason',[
    ({'joint_names':['b','a']},'JOINT_ORDER_MISMATCH'),
    ({'valid_length':2},'INVALID_TRAJECTORY_SHAPE_OR_TIME'),
    ({'time_s':[0,1,1]},'INVALID_TRAJECTORY_SHAPE_OR_TIME'),
    ({'time_s':[0,.01,.02]},'REFERENCE_VELOCITY_LIMIT'),
    ({'q':[[0,0],[.5,0],[1.1,0]]},'ENDPOINT_MISMATCH'),
    ({'interpolation':'bspline_control_points'},'UNSUPPORTED_EXECUTION_INTERPOLATION')])
def test_final_trajectory_contract(change,reason):
    t=path();t.update(change)
    assert trajectory_failure(request(),t)['reason']==reason


def test_missing_backend_no_fallback():
    result=run(lambda r,a:dict(status='DEPENDENCY_UNAVAILABLE',trajectory=None,error='missing',backend={'name':'curobo_v2'}))
    assert result['status']=='DEPENDENCY_UNAVAILABLE'
    assert len(result['attempts'])==1 and not result['fallback']


def test_production_edge_checker_rejects_legal_endpoints_with_colliding_middle():
    from unloading_sim.layout_trajectory import LayoutTrajectoryConnector
    connector=LayoutTrajectoryConnector.__new__(LayoutTrajectoryConnector)
    connector._statistics={'edge_validation_calls':0,'edge_state_samples':0}
    connector.budget=SimpleNamespace(edge_resolution_rad=.1)
    connector.collision_policy=SimpleNamespace(poc_pair_clearance=True)
    connector._state_failure=lambda q,obstacles,**kwargs: {'reason':'PAYLOAD_COLLISION'} if .49<q[0]<.51 else None
    assert connector._state_failure(np.zeros(6),[]) is None
    assert connector._state_failure(np.array([1.,0,0,0,0,0]),[]) is None
    failure=connector._path_failure([np.zeros(6),[1,0,0,0,0,0]],[],stage='transit')
    assert failure['reason']=='PAYLOAD_COLLISION' and 0<failure['fraction']<1


@pytest.mark.parametrize('payload_change',[{'mass_kg':0},{'dimensions_m':[0,.4,.3]},
    {'inertia_tensor_com_kg_m2':[[0,0,0],[0,0,0],[0,0,0]]}])
def test_payload_cannot_lose_mass_geometry_or_inertia(payload_change):
    d=request().to_dict();d['payload'].update(payload_change)
    with pytest.raises(ValueError):StageRequest(d)


def test_worker_launch_missing_dependency_is_structured_and_has_no_fallback():
    def missing(*args):raise FileNotFoundError('isolated python does not exist')
    result=run(missing)
    assert result['status']=='DEPENDENCY_UNAVAILABLE' and len(result['attempts'])==1
    assert not result['fallback'] and not result['delivered']


def test_native_endpoint_rejection_is_model_mismatch_not_physical_start_invalid():
    error={'reason':'NATIVE_COLLISION_REJECTS_AUTHORITY_ENDPOINT','pair':['base_link','chassis']}
    result=run(lambda r,a:dict(status='MODEL_MISMATCH',trajectory=None,error=error))
    assert result['status']=='MODEL_MISMATCH' and result['first_failure']==error
    assert len(result['attempts'])==1


def test_no_candidate_preserves_native_first_cause():
    error={'reason':'NATIVE_OPTIMIZATION_OR_CONVERGENCE_FAILED'}
    result=run(lambda r,a:dict(status='BACKEND_NO_CANDIDATE',trajectory=None,error=error))
    assert result['status']=='RESOURCE_EXHAUSTED' and result['first_failure']==error
