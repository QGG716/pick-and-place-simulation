from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from unloading_sim.geometry import make_transform, rotation_matrix_from_rpy
from unloading_sim.robot import URDFRobot


ROOT = Path(os.environ.get("M710_TEST_REPO_ROOT", Path(__file__).resolve().parents[1]))
# Isolated server validation can load the proposed module without replacing a
# source file currently used by the running planning process.
if os.environ.get("M710_MESH_PLANE_BACKEND_CANDIDATE"):
    spec = importlib.util.spec_from_file_location("unloading_sim._mesh_plane_candidate",
                                                os.environ["M710_MESH_PLANE_BACKEND_CANDIDATE"])
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
else:
    from unloading_sim import pinocchio_backend as module


def _backend():
    pytest.importorskip("pinocchio")
    pytest.importorskip("coal")
    package = ROOT / "assets/robots/fanuc_m710id_70/official/fanuc_m710_description"
    urdf = package / "urdf/m710id_70_official.urdf"
    base = make_transform(translation=[-1.325, 0.35, 0.6])
    return module.PinocchioHppFclBackend(urdf, tip_frame="flange", package_dirs=[package.parent],
                                        base_transform=base), package, base


def _source_world_vertices(backend, package, base, q):
    # Independent byte reader and pure-URDF FK: this does not use the provider's
    # cached Coal vertices or its Pinocchio geometry placements.
    robot = URDFRobot.from_urdf(backend.urdf_path, active_joint_names=backend.active_joint_names,
                               tip_link="flange", tool_length=0)
    robot.base_transform = base
    frames = robot.named_link_frames(q)
    results = {}
    for link in ET.parse(backend.urdf_path).getroot().findall("link"):
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            path = package / mesh.attrib["filename"].split("package://fanuc_m710_description/", 1)[1]
            payload = path.read_bytes()
            count = int.from_bytes(payload[80:84], "little")
            dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
            assert len(payload) == 84 + count * 50
            vertices = np.frombuffer(payload, dtype=dtype, count=count, offset=84)["vertices"].reshape(-1, 3).astype(float)
            scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            origin = collision.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            world = frames[link.attrib["name"]] @ make_transform(rotation_matrix_from_rpy(*rpy), xyz)
            points = vertices * scale @ world[:3, :3].T + world[:3, 3]
            results.setdefault(link.attrib["name"], []).append(points)
    return {name: np.concatenate(points) for name, points in results.items()}


def test_rotated_triangle_extrema_do_not_fill_empty_aabb_corners():
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    rotation = rotation_matrix_from_rpy(0, 0, np.pi / 4)
    lower, upper = module._world_vertex_extrema(vertices, rotation, np.zeros(3))
    old_lower, old_upper = module._world_aabb((vertices.min(axis=0), vertices.max(axis=0)), rotation, np.zeros(3))
    assert upper[1] == pytest.approx(np.sqrt(0.5), abs=1e-12)
    assert old_upper[1] == pytest.approx(np.sqrt(2), abs=1e-12)
    assert upper[1] < 0.8 - 0.02 < old_upper[1]
    assert np.all(lower <= (vertices @ rotation.T).min(axis=0))
    assert np.all(upper >= (vertices @ rotation.T).max(axis=0))


def test_official_mesh_extrema_match_all_source_stl_vertices_and_urdf_placements():
    backend, package, base = _backend()
    q_cases = [np.zeros(6), np.array([0.7, -0.3, 0.2, -0.4, 0.5, 0.6]),
               np.array([-0.89575680025, -0.80860228565, -0.50349145903,
                         1.80680454974, 0.93215870722, -5.09587270926])]
    for q in q_cases:
        actual = backend.collision_world_axis_extrema(q)
        independent = _source_world_vertices(backend, package, base, q)
        assert set(actual) == set(independent) == {"base_link", *(f"J{i}_link" for i in range(1, 7))}
        for name, points in independent.items():
            record = actual[name]
            assert record["exact_mesh"] and record["vertex_count"] > 0
            assert record["sources"] == ["SAME_COAL_COLLISION_MESH_VERTICES"]
            assert np.all(record["lower_m"] <= points.min(axis=0) + 1e-12)
            assert np.all(record["upper_m"] >= points.max(axis=0) - 1e-12)
            np.testing.assert_allclose(record["lower_m"], points.min(axis=0), atol=1e-10, rtol=0)
            np.testing.assert_allclose(record["upper_m"], points.max(axis=0), atol=1e-10, rtol=0)
        if q is q_cases[-1]:
            index = next(i for i, item in enumerate(backend.geometry_model.geometryObjects)
                         if backend.model.frames[item.parentFrame].name == "J2_link")
            pose = base @ backend._matrix(backend.geometry_data.oMg[index])
            old = module._world_aabb(backend._geometry_local_aabbs[index], pose[:3, :3], pose[:3, 3])
            print("M710_J2_PLANE_DIAGNOSTIC=" + json.dumps({"q_rad": q.tolist(),
                  "true_mesh_ymax_m": float(actual["J2_link"]["upper_m"][1]),
                  "rotated_local_aabb_ymax_m": float(old[1][1]),
                  "overbound_m": float(old[1][1] - actual["J2_link"]["upper_m"][1])}))
            assert old[1][1] - actual["J2_link"]["upper_m"][1] > 1e-3


def test_repeated_mesh_plane_queries_use_cached_same_coal_vertices_without_file_reads(monkeypatch):
    backend, _, _ = _backend()
    cached = backend._geometry_local_vertices
    assert all(vertices is not None and not vertices.flags.writeable for vertices in cached)
    def forbid_read(*args, **kwargs):
        raise AssertionError("mesh files must not be read during a plane query")
    monkeypatch.setattr(Path, "read_bytes", forbid_read)
    for q in (np.zeros(6), np.full(6, 0.15)):
        assert len(backend.collision_world_axis_extrema(q)) == 7
    assert backend._geometry_local_vertices is cached
