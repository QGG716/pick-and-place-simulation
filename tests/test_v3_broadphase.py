import numpy as np

from unloading_sim.geometry import OBB,rotation_matrix_from_rpy
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell
from unloading_sim.validation_scenes import regular_scene


def test_aabb_filter_never_rejects_any_sat_hit_in_rotated_random_or_touching_cases():
    cell=Cell(load_validation_config());rng=np.random.default_rng(71070)
    for _ in range(150):
        moving=OBB(rng.uniform(-2,2,3),rng.uniform(.01,.8,3),rotation_matrix_from_rpy(*rng.uniform(-3,3,3)))
        obstacles=[OBB(rng.uniform(-2,2,3),rng.uniform(.01,.8,3),rotation_matrix_from_rpy(*rng.uniform(-3,3,3)),str(i)) for i in range(20)]
        selected={id(o) for o in cell.broadphase(moving,obstacles,.01)}
        for obstacle in obstacles:
            if moving.intersects_obb(obstacle,margin=.01):assert id(obstacle) in selected
    a=OBB([0,0,0],[.3,.2,.15],np.eye(3));b=OBB([.62,0,0],[.3,.2,.15],np.eye(3))
    assert cell.broadphase(a,[b],.01)==[b]


def test_complete_state_reasons_match_without_broadphase():
    cfg=load_validation_config();fast=Cell(cfg);reference=Cell(cfg)
    reference.broadphase=lambda moving,obstacles,margin:list(obstacles)
    obstacles=[*fast.fixtures(),*fast.decks((0,.2)),*regular_scene(cfg.data['scene'])]
    rng=np.random.default_rng(71070)
    for _ in range(150):
        q=rng.uniform(fast.robot.joint_limits[:,0],fast.robot.joint_limits[:,1])
        assert fast.state_failure(q,obstacles)==reference.state_failure(q,obstacles)
