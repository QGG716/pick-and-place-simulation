import numpy as np
import pytest

from unloading_sim.geometry import OBB
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell
from unloading_sim.validation_physics import world_link_boxes, contact_separated
from unloading_sim.validation_scenes import grid_tasks, regular_scene, random_scene
from tools.run_m710id70_acceptance import _scene_regular, _scene_random


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
