"""Scoped planning exception retains other layers and every other pair rule."""
from pathlib import Path
import numpy as np
from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.geometry import OBB
from unloading_sim.layout_trajectory import ExactM710LayoutStateValidator
from unloading_sim.workcell_layout import load_workcell_layout
from test_layout_trajectory import _RecordingMeshRobot, _CompoundToolRobot
from l01_tool_chassis import frozen_l01_scope


def test_scope_uses_original_geometry_not_names():
    root=Path(__file__).resolve().parents[1]
    boxes=load_workcell_layout(root/'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_500_y_minus_100_min250_lowprofile.yaml').cartons()
    permission=frozen_l01_scope(boxes)
    assert permission['allowed_target_ids']==[f'carton_l01_c{i:02}' for i in range(5)]
    renamed=[OBB(b.center,b.half_extents,b.rotation,'renamed_'+str(i),b.category) for i,b in enumerate(boxes)]
    assert len(frozen_l01_scope(renamed)['allowed_target_ids'])==5
    old=SimulationCollisionPolicy()
    policy=SimulationCollisionPolicy(tool_chassis_contact_target_ids=tuple(permission['allowed_target_ids']))
    assert 'tool_chassis_contact_target_ids' not in old.to_mapping()
    assert SimulationCollisionPolicy.from_mapping(policy.to_mapping())==policy
    assert not policy.tool_chassis_pairs('carton_l00_c02',['tool_rigid_11'])


def test_cpu_tool_mount_cache_restores_other_targets_and_obstacles():
    validator=ExactM710LayoutStateValidator(_RecordingMeshRobot(),_CompoundToolRobot(),
        collision_margin_m=.01,floor_z_m=0.,right_wall_y_m=-2.,left_wall_y_m=2.,
        robot_world_boxes=lambda q:[],collision_policy={'tool_chassis_contact_target_ids':['authorized']})
    q=np.zeros(6)
    chassis=OBB([0,0,1],[.02,.02,.02],np.eye(3),'chassis','fixed')
    def check(name,obstacle=chassis):
        validator.contact_target_name=name
        return validator(q,[obstacle],payload=None,target_contact=None,stage='extraction')
    assert check('authorized') is None
    assert check('other')['reason']=='RIGID_TOOL_COLLISION'
    assert check(None)['reason']=='RIGID_TOOL_COLLISION'
    assert check('authorized') is None
    for name in ('chassis_clone','trailer_left_wall','conveyor_transverse','carton_neighbor'):
        other=OBB(chassis.center,chassis.half_extents,np.eye(3),name,'carton' if name=='carton_neighbor' else 'fixed')
        assert check('authorized',other)['reason']=='RIGID_TOOL_COLLISION'
