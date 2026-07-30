import numpy as np

from unloading_sim.geometry import Capsule, OBB


def test_capsule_obb_collision_and_clearance():
    box = OBB(np.zeros(3), np.ones(3) * 0.5, np.eye(3), name="box")
    hit = Capsule(np.array([-1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), 0.05)
    miss = Capsule(np.array([-1.0, 1.0, 0.0]), np.array([1.0, 1.0, 0.0]), 0.10)
    assert hit.collides_obb(box)
    assert not miss.collides_obb(box)


def test_capsule_obb_bounding_sphere_rejection_keeps_near_miss_clear():
    box = OBB(np.zeros(3), np.ones(3) * 0.5, np.eye(3), name="box")
    far_capsule = Capsule(np.array([3.0, 0.0, 0.0]), np.array([3.2, 0.0, 0.0]), 0.05)

    assert not far_capsule.collides_obb(box)
