import csv

import numpy as np
import pytest

from unloading_sim.calibration import (
    ControllerLog,
    calibrate_motion_limits,
    calibrate_process_delays_from_csv,
    load_controller_csv,
)
from unloading_sim.collision_backend import CollisionQuery, compare_collision_backends
from unloading_sim.dynamics import CameraSpec, TimedJointCommands, tracking_error_audit
from unloading_sim.geometry import OBB
from unloading_sim.realtime_planner import CertifiedRuntimePlanner, _digest, scene_snapshot
from unloading_sim.scene import TrailerScene
from unloading_sim.support import SupportRelationGraph
from unloading_sim.trajectory import blend_joint_path, simplify_collinear_joint_path


def _box(name, center, size=(1.0, 1.0, 1.0)):
    return OBB(np.asarray(center, dtype=float), np.asarray(size, dtype=float) / 2.0, np.eye(3), name, "carton")


def test_corner_blend_is_deterministic_and_collision_validated():
    path = [np.array([0.0, 0.0]), np.array([1.0, 0.0]), np.array([1.0, 1.0])]
    first = blend_joint_path(path, lambda _q: True, corner_fraction=0.2, samples_per_corner=5)
    second = blend_joint_path(path, lambda _q: True, corner_fraction=0.2, samples_per_corner=5)

    assert first.accepted_corners == 1
    assert np.allclose(first.path, second.path)
    assert np.allclose(first.path[0], path[0])
    assert np.allclose(first.path[-1], path[-1])


def test_corner_blend_keeps_original_corner_when_curve_collides():
    path = [np.array([0.0, 0.0]), np.array([1.0, 0.0]), np.array([1.0, 1.0])]

    def valid(q):
        return not (q[0] < 0.999 and q[1] > 0.001)

    result = blend_joint_path(path, valid, corner_fraction=0.2, samples_per_corner=7)
    assert result.accepted_corners == 0
    assert result.rejected_corners == 1
    assert np.allclose(result.path, path)


def test_collinear_simplification_preserves_real_corner():
    path = [np.array([0.0, 0.0]), np.array([0.5, 0.0]), np.array([1.0, 0.0]), np.array([1.0, 1.0])]
    simplified = simplify_collinear_joint_path(path)
    assert len(simplified) == 3
    assert np.allclose(simplified[1], [1.0, 0.0])


def test_controller_log_calibration_and_csv_import(tmp_path):
    path = tmp_path / "controller.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_from_start_s", "q1", "q2"])
        for timestamp in np.linspace(0.0, 1.0, 21):
            writer.writerow([timestamp, timestamp**3, 0.5 * timestamp**2])

    log = load_controller_csv(path)
    result = calibrate_motion_limits(log, percentile=100.0, headroom=1.1)
    assert result["samples"] == 21
    assert result["duration_seconds"] == pytest.approx(1.0)
    assert len(result["joint_jerk_limits_rad_s3"]) == 2
    assert all(value > 0.0 for value in result["joint_velocity_limits_rad_s"])


def test_controller_log_rejects_non_monotonic_time():
    with pytest.raises(ValueError, match="strictly increasing"):
        ControllerLog(np.array([0.0, 0.1, 0.1, 0.2]), np.zeros((4, 2)), "test")


def test_process_event_log_calibrates_perception_vacuum_and_release(tmp_path):
    path = tmp_path / "events.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["cycle_id", "event", "timestamp_s"])
        for event, timestamp in (
            ("perception_start", 0.0), ("perception_ready", 0.2),
            ("vacuum_on", 1.0), ("vacuum_sealed", 1.3),
            ("release_command", 2.0), ("release_confirmed", 2.1),
        ):
            writer.writerow([1, event, timestamp])
    result = calibrate_process_delays_from_csv(path)
    assert result["perception_update_seconds"] == pytest.approx(0.2)
    assert result["vacuum_establish_seconds"] == pytest.approx(0.3)
    assert result["release_seconds"] == pytest.approx(0.1)


def test_support_graph_defers_supporting_carton_and_updates_order():
    lower = _box("lower", (1.0, 0.0, 0.5))
    upper = _box("upper", (1.0, 0.0, 1.5))
    front = _box("front", (0.0, 1.2, 0.5))
    graph = SupportRelationGraph.build([lower, upper, front])

    assert graph.supports["lower"] == {"upper"}
    assert "lower" not in graph.removable_cartons()
    order = graph.removal_order()
    assert order.index("upper") < order.index("lower")


class _ThresholdBackend:
    def __init__(self, name, threshold):
        self.name = name
        self.threshold = threshold

    def query(self, q, _obstacles):
        return CollisionQuery(bool(q[0] >= self.threshold), "threshold")


def test_collision_backend_comparison_reports_false_positive_and_negative():
    path = [np.array([0.0]), np.array([0.5]), np.array([1.0])]
    report = compare_collision_backends(
        path,
        [],
        _ThresholdBackend("approx", 0.5),
        _ThresholdBackend("exact", 1.0),
    )
    assert report["both_free"] == 1
    assert report["approx_only"] == 1
    assert report["both_collision"] == 1
    assert report["agreement_rate"] == pytest.approx(2.0 / 3.0)


def test_timed_commands_camera_intrinsics_and_tracking_audit():
    commands = TimedJointCommands(np.array([0.0, 1.0]), np.array([[0.0, 1.0], [1.0, 3.0]]))
    assert np.allclose(commands.sample(0.25), [0.25, 1.5])
    camera = CameraSpec("trailer_rgbd", 640, 480, np.deg2rad(60.0))
    assert camera.intrinsic_matrix.shape == (3, 3)
    report = tracking_error_audit(
        np.array([[0.0, 0.0], [1.0, 1.0]]),
        np.array([[0.0, 0.1], [0.8, 1.0]]),
        np.array([0.0, 1.0]),
    )
    assert report["peak_error_rad"] == pytest.approx(0.2)


def test_certified_runtime_lookup_is_bounded_and_respects_support_order():
    lower = _box("lower", (1.0, 0.0, 0.5))
    upper = _box("upper", (1.0, 0.0, 1.5))
    scene = TrailerScene([], [lower, upper], {})
    path = [[0.0] * 6, [0.1] * 6]
    segment = {
        "pick_index": 0,
        "target": "upper",
        "path": path,
        "amr_dock_position": [0.0, 0.0, 0.0],
    }
    library = {
        "format": "fanuc_certified_runtime_library_v1",
        "robot_model": "fanuc_m20id35",
        "certificate": {
            "translation_tolerance_m": 0.0,
            "size_tolerance_m": 0.0,
            "rotation_tolerance_rad": 0.0,
            "joint_start_tolerance_rad": 0.01,
        },
        "entries": [{
            "target": "upper",
            "scene": scene_snapshot(scene),
            "start_joints_rad": path[0],
            "end_joints_rad": path[-1],
            "segment": segment,
            "path_sha256": _digest(path),
        }],
    }
    result = CertifiedRuntimePlanner(library, deadline_seconds=0.05).plan_next(scene, np.zeros(6))
    assert result.success
    assert result.target == "upper"
    assert result.source == "certified_library"
    assert result.latency_seconds < 0.05


def test_certified_runtime_lookup_rejects_perception_drift_without_robust_certificate():
    nominal = _box("carton", (1.0, 0.0, 0.5))
    observed = _box("carton", (1.001, 0.0, 0.5))
    nominal_scene = TrailerScene([], [nominal], {})
    observed_scene = TrailerScene([], [observed], {})
    path = [[0.0] * 6, [0.1] * 6]
    segment = {"target": "carton", "path": path}
    library = {
        "format": "fanuc_certified_runtime_library_v1",
        "robot_model": "fanuc_m20id35",
        "certificate": {
            "translation_tolerance_m": 0.0,
            "size_tolerance_m": 0.0,
            "rotation_tolerance_rad": 0.0,
            "joint_start_tolerance_rad": 0.01,
        },
        "entries": [{
            "target": "carton", "scene": scene_snapshot(nominal_scene),
            "start_joints_rad": path[0], "segment": segment, "path_sha256": _digest(path),
        }],
    }
    result = CertifiedRuntimePlanner(library).plan_next(observed_scene, np.zeros(6))
    assert not result.success
    assert result.source == "cache_miss"
    assert "translation error" in result.message
