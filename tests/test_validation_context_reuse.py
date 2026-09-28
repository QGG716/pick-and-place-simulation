"""Context/edge regressions; geometry safety is unchanged by evidence reuse."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from unloading_sim.geometry import OBB
from unloading_sim.layout_trajectory import LayoutTrajectoryConnector, PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment


class Robot:
    dof=6
    base_transform=np.eye(4)
    joint_limits=np.array([[-4.,4.]]*6)
    def within_limits(self,q): return bool(np.all(np.abs(q)<=4))
    def geometric_jacobian(self,q): return np.eye(6)
    def fk(self,q):
        pose=self.base_transform.copy();pose[0,3]+=q[0];return pose
    def named_link_frames(self,q): return {'flange':self.fk(q)}


def connector(check=lambda *a,**k:None):
    return LayoutTrajectoryConnector(Robot(),check,flange_from_virtual_task_tcp=np.eye(4),
        flange_from_physical_contact=np.eye(4),ik_policy={},collision_margin_m=.005,
        contact_tolerance_m=.0002,joint_margin_rad=.001,maximum_jacobian_condition=1e4,
        validator_identity='test-context-v1',execution_qualified=True)


def test_attachment_size_and_identity_invalidate_state_cache():
    calls=[];c=connector(lambda *a,**k:calls.append(k['payload']) or None)
    a=PhysicalContactAttachment(c.robot,RigidAttachment(np.eye(4),np.array([.3,.2,.15]),'held'),np.eye(4),np.eye(4))
    q=np.zeros(6)
    for _ in range(2): assert c._state_failure(q,[],attachment=a,stage='transit') is None
    assert len(calls)==1
    b=replace(a,rigid=RigidAttachment(np.eye(4),np.array([.3,.2,.16]),'held'))
    assert c._state_failure(q,[],attachment=b,stage='transit') is None
    d=replace(b,rigid=RigidAttachment(np.eye(4),np.array([.3,.2,.16]),'different'))
    assert c._state_failure(q,[],attachment=d,stage='transit') is None
    assert len(calls)==3


def test_accepted_edge_cache_requires_same_scene_policy_and_interpolation():
    calls=[];c=connector(lambda *a,**k:calls.append(1) or None)
    obstacle=OBB([5,0,0],[.1]*3,np.eye(3),'world','fixture');world=[obstacle]
    path=[np.zeros(6),np.array([.04,0,0,0,0,0])]
    assert c._path_failure(path,world,stage='transit') is None
    count=len(calls)
    assert c._path_failure(path,world,stage='transit') is None and len(calls)==count
    obstacle.center[0]+=1
    assert c._path_failure(path,world,stage='transit') is None and len(calls)>count
    count=len(calls);c.collision_margin_m+=.001
    assert c._path_failure(path,world,stage='transit') is None and len(calls)>count
    failure=c._path_failure(path,world,stage='transit',interpolation='cubic')
    assert failure['reason']=='UNSUPPORTED_VALIDATION_INTERPOLATION'


def test_context_change_inside_block_cannot_publish_acceptance():
    world=[OBB([5,0,0],[.1]*3,np.eye(3),'world','fixture')]
    def change(*args,**kwargs): world[0].center[0]+=1
    c=connector(change)
    failure=c._path_failure([np.zeros(6),np.array([.04,0,0,0,0,0])],world,stage='transit')
    assert failure['reason']=='VALIDATION_CONTEXT_CHANGED'


def test_box_corner_is_checked_even_when_robot_tool_are_safe():
    c=connector()
    a=PhysicalContactAttachment(c.robot,RigidAttachment(np.eye(4),np.array([.3,.2,.15]),'held'),np.eye(4),np.eye(4))
    corner=OBB([.3,.2,.15],[.003]*3,np.eye(3),'corner','fixture')
    f=c._state_failure(np.zeros(6),[corner],attachment=a,stage='transit')
    assert f['reason']=='PAYLOAD_COLLISION' and f['pair']==['held','corner']


def test_blocked_middle_keeps_first_edge_fraction_q_and_pair():
    def collision(q,*a,**k):
        return {'reason':'PAYLOAD_COLLISION','pair':['held','world']} if .49<q[0]<.51 else None
    path=[np.zeros(6),np.array([1.,0,0,0,0,0])]
    c=connector(collision);old=connector(collision)
    failure=c._path_failure(path,[],stage='transit')
    reference=old._legacy_path_failure(path,[],stage='transit')
    assert failure==reference and failure['edge']==0
    assert 0<failure['fraction']<1 and failure['pair']==['held','world']


def test_rigid_pair_cache_preserves_cup_boundary_and_rigid_collision():
    from unloading_sim.layout_trajectory import ExactM710LayoutStateValidator
    from unloading_sim.robot import URDFRobot
    from unloading_sim.collision_policy import SimulationCollisionPolicy
    tool=URDFRobot.__new__(URDFRobot)
    tool.tool_collision_local_boxes=np.array([[0,0,.4,.1,.1,.1], [0,0,0,.1,.1,.1]])
    tool.tool_compliant_collision_local_boxes=np.array([[0,0,.151,.02,.02,.002]])
    mesh=Robot();v=ExactM710LayoutStateValidator.__new__(ExactM710LayoutStateValidator)
    v.mesh_robot=mesh;v.tool_transform_robot=tool;v.nominal_cup_compression_m=0.
    v.collision_margin_m=.01;v.performance_counters={}
    from unloading_sim.layout_single_carton import load_layout_motion_policy
    v.collision_policy=SimulationCollisionPolicy.from_mapping(load_layout_motion_policy(
        'configs/validation/m710id70_handoff_continuation.yaml').layout_validation.data['collision_policy'])
    a=PhysicalContactAttachment(mesh,RigidAttachment(np.eye(4),np.array([.3,.2,.15]),'held'),np.eye(4),np.eye(4))
    v._verified_attachment_frames=v._attachment_frame_binding()
    name,proofs=v.prepare_attached_pair_cache(a,'transit','one')
    assert name=='held' and 'tool_rigid_0' in proofs
    assert v.prepare_attached_pair_cache(replace(a,robot=tool),'transit','lightweight') is not None
    assert v.prepare_attached_pair_cache(replace(a,robot=Robot()),'transit','unknown') is None
    assert 'tool_rigid_1' not in proofs  # colliding rigid insert still takes world exact query
    assert 'tool_compliant_bellows_0' in proofs
    tool.tool_compliant_collision_local_boxes[0,2]=.146  # exactly 5 mm extra compression
    _,proofs=v.prepare_attached_pair_cache(a,'transit','two')
    assert 'tool_compliant_bellows_0' not in proofs  # roundoff-sensitive boundary is not memoized
    assert v.prepare_attached_pair_cache(a,'extraction','three') is None
    mesh.base_transform=mesh.base_transform.copy();mesh.base_transform[0,3]=.01
    assert v.prepare_attached_pair_cache(a,'transit','four') is None


def test_revoked_policy_cannot_reuse_accepted_edges():
    c=connector()
    c.collision_policy=replace(c.collision_policy,wrist_tool_exempt_links=('J5_link','J6_link'))
    def check(q,*a,**k):
        if not c.collision_policy.wrist_tool_exempt_links:
            return dict(reason='ROBOT_TOOL_COLLISION',pair=['J5_link','tool_rigid_0'])
    c.robot_state_validator=check
    path=[np.zeros(6),np.array([.04,0,0,0,0,0])]
    assert c._path_failure(path,[],stage='transit') is None
    assert c._path_failure(path,[],stage='transit') is None
    c.collision_policy=replace(c.collision_policy,wrist_tool_exempt_links=())
    failure=c._path_failure(path,[],stage='transit')
    assert failure['pair']==['J5_link','tool_rigid_0']


def test_changed_execution_geometry_does_not_inherit_endpoint_edge_proof():
    def collision(q,*a,**k):
        if q[0]>.45:return dict(reason='PAYLOAD_COLLISION',pair=['held','obstacle'])
    c=connector(collision);start=np.zeros(6);goal=np.array([.4,0,0,0,0,0])
    assert c._path_failure([start,goal],[],stage='transit') is None
    # Same endpoints, but a smoothed/retimed path with spatial overshoot is new geometry.
    overshoot=np.array([.5,0,0,0,0,0])
    failure=c._path_failure([start,overshoot,goal],[],stage='transit')
    assert failure['edge']==0 and failure['pair']==['held','obstacle']
