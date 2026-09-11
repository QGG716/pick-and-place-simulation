"""Perception and ROS-neutral integration adapters."""

from .backends import CargoJsonReplayBackend, CargoPipelineBackend, SimGroundTruthBackend
from .execution import ExecutionGate
from .isaac_validation import (
    HistoricalResultGate,
    IsaacCaptureBinding,
    IsaacSceneManifest,
    SimulationClockGuard,
    build_feasibility_handoff,
    build_scene_manifest,
    ground_truth_observation,
)
from .scene import ObservationTracker, SnapshotAssembler, build_scene_update

__all__ = [
    "CargoJsonReplayBackend", "CargoPipelineBackend", "SimGroundTruthBackend",
    "ExecutionGate", "ObservationTracker", "SnapshotAssembler", "build_scene_update",
    "HistoricalResultGate", "IsaacCaptureBinding", "IsaacSceneManifest",
    "SimulationClockGuard", "build_feasibility_handoff", "build_scene_manifest",
    "ground_truth_observation",
]
