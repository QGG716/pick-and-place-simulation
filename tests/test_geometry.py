import numpy as np

from unloading_sim.geometry import (Capsule, OBB,
                                    rotation_matrix_from_rotation_vector,
                                    rotation_vector_from_matrix)


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


def test_obb_signed_distance_distinguishes_clearance_contact_and_penetration():
    box = OBB(np.zeros(3), np.full(3, 0.5), np.eye(3), name="target")

    assert np.isclose(box.signed_distance_obb(
        OBB([1.01, 0, 0], np.full(3, 0.5), np.eye(3), name="clear")), 0.01)
    assert np.isclose(box.signed_distance_obb(
        OBB([1.0, 0, 0], np.full(3, 0.5), np.eye(3), name="contact")), 0.0)
    assert np.isclose(box.signed_distance_obb(
        OBB([0.99, 0, 0], np.full(3, 0.5), np.eye(3), name="overlap")), -0.01)


def test_obb_signed_distance_is_rigid_transform_invariant():
    angle = np.pi / 3
    rotation = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1],
    ])
    offset = np.array([0.4, -0.7, 0.2])
    box = OBB(offset, np.full(3, 0.5), rotation, name="target")
    neighbor = OBB(offset + rotation @ np.array([1.013, 0, 0]),
                   np.full(3, 0.5), rotation, name="neighbor")

    assert np.isclose(box.signed_distance_obb(neighbor), 0.013)


def test_rotation_vector_exponential_round_trips_nontrivial_rotation():
    vector=np.array([.2,-.3,.4])

    recovered=rotation_vector_from_matrix(rotation_matrix_from_rotation_vector(vector))

    assert np.allclose(recovered,vector,atol=1e-12,rtol=0)
