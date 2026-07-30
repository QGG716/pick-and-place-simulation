from pathlib import Path

import numpy as np

from unloading_sim.pybullet_sim import quaternion_from_matrix, resolve_package_uri


def test_resolve_package_uri():
    root = Path("/tmp/example_pkg")
    assert resolve_package_uri("package://example_pkg/meshes/link.stl", {"example_pkg": root}) == root / "meshes/link.stl"


def test_quaternion_from_identity_matrix():
    quat = quaternion_from_matrix(np.eye(3))
    assert np.allclose(quat, (0.0, 0.0, 0.0, 1.0))