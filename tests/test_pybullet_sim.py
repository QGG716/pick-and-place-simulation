from pathlib import Path

import numpy as np

from unloading_sim.pybullet_sim import quaternion_from_matrix, resolve_package_uri
import unloading_sim.pybullet_kuka_kr50 as pybullet_kuka_kr50
from unloading_sim.pybullet_online_kuka import remove_conveyed_carton, suction_replay_mount


def test_resolve_package_uri():
    root = Path("/tmp/example_pkg")
    assert resolve_package_uri("package://example_pkg/meshes/link.stl", {"example_pkg": root}) == root / "meshes/link.stl"


def test_quaternion_from_identity_matrix():
    quat = quaternion_from_matrix(np.eye(3))
    assert np.allclose(quat, (0.0, 0.0, 0.0, 1.0))


def test_kuka_viewer_uses_robot_pose_from_config(monkeypatch):
    captured = {}

    monkeypatch.setattr(pybullet_kuka_kr50, "run_viewer", lambda args: captured.setdefault("args", args))

    pybullet_kuka_kr50.main([])

    assert captured["args"].base_position is None
    assert captured["args"].base_rpy is None


def test_fanuc_suction_face_is_mounted_on_tool_z_at_planned_tip():
    link_name, mount_z = suction_replay_mount(
        {"tip_link": "tool0", "tool_length": 0.20},
        {"flange": 6, "tool0": 7},
    )

    assert link_name == "tool0"
    assert np.isclose(mount_z + 0.166, 0.20)


def test_released_carton_disappears_after_conveyor_handoff():
    class FakePyBullet:
        def __init__(self):
            self.removed = []

        def removeBody(self, body_id):
            self.removed.append(body_id)

    backend = FakePyBullet()
    bodies = {"carton_a": 42, "carton_b": 43}
    groups = {"carton_a": [42, 101, 102], "carton_b": [43, 103]}

    remove_conveyed_carton(backend, bodies, "carton_a", groups)

    assert backend.removed == [42, 101, 102]
    assert bodies == {"carton_b": 43}
    assert groups == {"carton_b": [43, 103]}
