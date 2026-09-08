"""Perception and ROS-neutral integration adapters."""

from .backends import CargoJsonReplayBackend, CargoPipelineBackend, SimGroundTruthBackend
from .execution import ExecutionGate
from .scene import ObservationTracker, SnapshotAssembler, build_scene_update

__all__ = [
    "CargoJsonReplayBackend", "CargoPipelineBackend", "SimGroundTruthBackend",
    "ExecutionGate", "ObservationTracker", "SnapshotAssembler", "build_scene_update",
]
