from types import SimpleNamespace

import numpy as np

from unloading_sim.geometry import make_transform
from unloading_sim.grasp import (
    SuctionGraspCandidate,
    _expand_candidate_wrist_rolls,
    _nearest_equivalent_joint_vector,
)
from unloading_sim.scene import load_scene_config


def test_wrist_roll_candidates_keep_suction_normal_and_contact():
    pose = make_transform(np.eye(3), [1.0, 2.0, 3.0])
    candidate = SuctionGraspCandidate(
        carton_name="box",
        contact_point=np.array([1.0, 2.0, 3.0]),
        outward_normal=np.array([1.0, 0.0, 0.0]),
        face_mode="front",
        pregrasp_pose=pose.copy(),
        grasp_pose=pose.copy(),
        score=1.0,
    )

    variants = _expand_candidate_wrist_rolls([candidate], [0.0, 90.0, -90.0, 180.0])

    assert len(variants) == 4
    for variant in variants:
        assert np.allclose(variant.grasp_pose[:3, 2], candidate.grasp_pose[:3, 2])
        assert np.allclose(variant.grasp_pose[:3, 3], candidate.grasp_pose[:3, 3])


def test_nearest_equivalent_joint_vector_avoids_full_wrist_turn():
    robot = SimpleNamespace(
        joint_limits=np.array([[-np.pi, np.pi]] * 5 + [[-2.5 * np.pi, 2.5 * np.pi]])
    )
    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.1 * np.pi])
    reference = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.05 * np.pi])

    normalized = _nearest_equivalent_joint_vector(robot, q, reference)

    assert np.isclose(normalized[5], 0.1 * np.pi)
    assert robot.joint_limits[5, 0] <= normalized[5] <= robot.joint_limits[5, 1]


def test_scene_config_supports_recursive_inheritance(tmp_path):
    base = tmp_path / "base.yaml"
    middle = tmp_path / "middle.yaml"
    leaf = tmp_path / "leaf.yaml"
    base.write_text(
        """
scene:
  trailer: {length: 4.0, width: 2.0, height: 3.0}
  cartons: []
robot: {model: base}
planning: {seed: 1, nested: {a: 1}}
""",
        encoding="utf-8",
    )
    middle.write_text(
        "extends: base.yaml\nplanning: {nested: {b: 2}}\n",
        encoding="utf-8",
    )
    leaf.write_text(
        "extends: middle.yaml\nrobot: {model: fanuc}\n",
        encoding="utf-8",
    )

    _, config = load_scene_config(leaf)

    assert config["robot"]["model"] == "fanuc"
    assert config["planning"]["nested"] == {"a": 1, "b": 2}
