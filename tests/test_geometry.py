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


def test_segment_obb_distance_finds_interior_minimum_exactly():
    box = OBB(np.zeros(3), np.ones(3) * 0.5, np.eye(3), name="box")
    p0 = np.array([-2.0, 1.0, 0.0])
    p1 = np.array([2.0, 1.0, 0.0])

    assert np.isclose(box.segment_distance_squared(p0, p1), 0.25)


def test_segment_obb_distance_is_rotation_invariant():
    angle = np.pi / 5.0
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]]
    )
    box = OBB(np.array([0.4, -0.2, 0.3]), np.array([0.5, 0.3, 0.2]), rotation, name="rotated")
    local_p0 = np.array([-1.2, 0.8, 0.1])
    local_p1 = np.array([1.1, 0.8, 0.1])

    distance = box.segment_distance_squared(box.to_world(local_p0), box.to_world(local_p1))

    assert np.isclose(distance, 0.25)
