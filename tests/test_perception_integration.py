from pathlib import Path
import sys

import pytest

from unloading_contracts import (
    ExecutionCommand, ExecutionEventKind, PlanArtifactKind, ResourceReference,
    RobotStateRevision, SensorFrame, TimedJointPoint, TimedJointTrajectory, Validity,
)
from unloading_perception.backends import CargoJsonReplayBackend, CargoPipelineBackend, resolve_controlled_reference
from unloading_perception.execution import ExecutionGate
from unloading_perception.geometry import transform_pose
from unloading_perception.scene import ObservationTracker, SnapshotAssembler, build_scene_update


FIXTURE = Path(__file__).parent / "fixtures" / "vision_upstream" / "cargo7_minimal.json"


def frame(sequence: int = 1, epoch: str = "epoch-a") -> SensorFrame:
    return SensorFrame("camera", "rgb", epoch, sequence, 10.0 + sequence, 10.1 + sequence, "replay", "camera_optical", 1536, 1024, "json", ResourceReference(FIXTURE.resolve().as_uri()))


def observation(sequence: int = 1, epoch: str = "epoch-a"):
    return CargoJsonReplayBackend().read(FIXTURE, frame=frame(sequence, epoch))


def complete_assembler(update):
    assembler = SnapshotAssembler()
    assembler.update = update
    assembler.robot_state = RobotStateRevision(1, (0.0, 0.0), {"joint_names": ("j1", "j2")})
    assembler.tool_attachment = {"tool_id": "vacuum", "confirmed": True}
    assembler.payload_attachment = {"object_id": None, "confirmed": True}
    assembler.base_state = {"position_m": (0.0, 0.0, 0.0), "confirmed": True}
    assembler.conveyor_state = {"running": False, "confirmed": True}
    assembler.config_identity = {"robot_model_fingerprint": "robot-sha", "world_model_fingerprint": "world-sha"}
    return assembler


def test_real_upstream_fixture_preserves_null_scores_and_non_box_obstacles():
    result = observation()
    assert len(result.cargo) == 4
    bag = result.cargo[0]
    assert bag.detection_score is None
    assert bag.category == "bag"
    assert not bag.candidate_eligible
    assert "NON_BOX_CARGO" in bag.eligibility_reasons
    assert result.cargo[2].geometry_validity is Validity.NOT_EVALUATED
    assert result.cargo[3].geometry_validity is Validity.INVALID


def test_monocular_metric_named_output_is_not_execution_geometry():
    box = observation().cargo[1]
    assert box.pose.frame_id == "camera_optical_model"
    assert box.full_dimensions_m == pytest.approx((0.01293, 0.0909, 0.1454))
    assert box.metric_scale_validity is Validity.UNKNOWN
    assert not box.candidate_eligible
    assert "MONOCULAR_SCALE_UNVERIFIED" in box.eligibility_reasons
    assert any(face.evidence.value == "CONSTRAINT_COMPLETED" for face in box.face_evidence)


def test_top_level_source_image_is_not_misread_as_3d_frame():
    result = observation()
    update = build_scene_update(result)
    assert not update.planning_admissible
    assert any(region.reason == "WORLD_TRANSFORM_MISSING" for region in update.unknown_regions)


def test_reference_resolution_rejects_escape_and_missing_file(tmp_path):
    inside = tmp_path / "inside.txt"
    inside.write_text("ok", encoding="utf-8")
    assert resolve_controlled_reference(tmp_path, "inside.txt") == inside.resolve()
    with pytest.raises(ValueError):
        resolve_controlled_reference(tmp_path, "../outside.txt")
    with pytest.raises(FileNotFoundError):
        resolve_controlled_reference(tmp_path, "missing.txt")


def test_tracker_rejects_duplicate_retains_missed_and_resets_epoch():
    tracker = ObservationTracker()
    first = tracker.update(observation(1))
    with pytest.raises(ValueError, match="out-of-order"):
        tracker.update(observation(1))
    second_observation = observation(2)
    from dataclasses import replace
    second_observation = replace(second_observation, cargo=second_observation.cargo[:1])
    second = tracker.update(second_observation)
    assert any(item.association_status == "STALE_OCCLUDED" for item in second)
    reset = tracker.update(observation(0, "epoch-b"))
    assert all(item.association_status == "NEW" for item in reset)


def test_snapshot_requires_real_mechanism_state_and_is_immutable():
    update = build_scene_update(observation())
    assembler = SnapshotAssembler()
    assembler.update = update
    assert assembler.assemble().status == "INCOMPLETE_STATE"
    assembler = complete_assembler(update)
    first = assembler.assemble()
    second = assembler.assemble()
    assert first.snapshot is not None
    assert first.status == "COMPLETE_BUT_NOT_PLANNABLE"
    assert second.snapshot.scene_revision.sequence == first.snapshot.scene_revision.sequence
    with pytest.raises(TypeError):
        first.snapshot.scene_snapshot["new"] = 1


def test_freshness_and_attachment_changes_invalidate_scene_revision():
    fresh = build_scene_update(observation(), now=11.5, max_age_seconds=2.0)
    stale = build_scene_update(observation(), now=20.0, max_age_seconds=2.0)
    assert "OBSERVATION_STALE" not in fresh.blocking_reasons
    assert "OBSERVATION_STALE" in stale.blocking_reasons
    assembler = complete_assembler(fresh)
    before = assembler.assemble().snapshot
    assembler.payload_attachment = {"object_id": "track-1", "confirmed": True}
    after = assembler.assemble().snapshot
    assert after.scene_revision.sequence == before.scene_revision.sequence + 1
    assert after.fingerprint != before.fingerprint


def trajectory():
    return TimedJointTrajectory(("j1", "j2"), (TimedJointPoint((0, 0), 0.0), TimedJointPoint((1, 1), 1.0)), PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, "mock-fixture", "robot-sha", "config-sha")


def command(snapshot, command_id="cmd-1", epoch="epoch-a", generation=2):
    return ExecutionCommand(command_id, "plan-1", "request-1", "session-1", epoch, generation, None, snapshot.fingerprint, "robot-sha", "config-sha", "validator@1:validation-1", 2, trajectory())


def test_execution_gate_rejects_hardware_duplicate_and_changed_world():
    snapshot = complete_assembler(build_scene_update(observation())).assemble().snapshot
    assert snapshot is not None
    hardware = ExecutionGate(enable_hardware=True, hardware_adapter_verified=False)
    assert hardware.authorize(command(snapshot), snapshot, epoch="epoch-a", generation=2).kind is ExecutionEventKind.REJECTED
    gate = ExecutionGate()
    assert gate.authorize(command(snapshot), snapshot, epoch="epoch-a", generation=2).kind is ExecutionEventKind.ACCEPTED
    assert gate.authorize(command(snapshot), snapshot, epoch="epoch-a", generation=2).message == "DUPLICATE_COMMAND"
    assert ExecutionGate().authorize(command(snapshot, epoch="old"), snapshot, epoch="epoch-a", generation=2).message == "STALE_EPOCH_OR_GENERATION"
    altered_assembler = complete_assembler(build_scene_update(observation()))
    altered_assembler.conveyor_state = {"running": True, "confirmed": True}
    altered = altered_assembler.assemble().snapshot
    assert ExecutionGate().authorize(command(snapshot, command_id="cmd-world"), altered, epoch="epoch-a", generation=2).message == "WORLD_CHANGED_BEFORE_SEND"


def test_cancel_acceptance_is_not_stop_acknowledgement():
    snapshot = complete_assembler(build_scene_update(observation())).assemble().snapshot
    gate = ExecutionGate()
    gate.authorize(command(snapshot), snapshot, epoch="epoch-a", generation=2)
    event = gate.accept_cancel("cmd-1")
    assert event.kind is ExecutionEventKind.CANCEL_ACCEPTED
    with pytest.raises(ValueError, match="near-zero"):
        gate.confirm_stop((0.2, 0.3), (0.0, 0.01), evidence_reference="feedback-1", event_time=5.0)
    acknowledgement = gate.confirm_stop((0.2, 0.3), (0.0, 0.0), evidence_reference="feedback-2", event_time=6.0)
    assert acknowledgement.plan_id == "plan-1"


def test_pipeline_capability_fails_closed_without_interpreter(tmp_path):
    backend = CargoPipelineBackend((str(tmp_path / "missing-python"), "worker.py"), cwd=tmp_path)
    result = backend.infer(frame())
    assert result.failure_code == "BACKEND_UNAVAILABLE"
    assert not result.cargo


def test_pipeline_timeout_crash_and_bad_schema_are_distinct(tmp_path):
    sleeper = tmp_path / "sleep.py"
    sleeper.write_text("import time; time.sleep(2)\n", encoding="utf-8")
    assert CargoPipelineBackend((sys.executable, str(sleeper)), cwd=tmp_path, timeout_seconds=0.05).infer(frame()).failure_code == "WORKER_TIMEOUT"
    crash = tmp_path / "crash.py"
    crash.write_text("raise SystemExit(7)\n", encoding="utf-8")
    assert CargoPipelineBackend((sys.executable, str(crash)), cwd=tmp_path).infer(frame()).failure_code == "WORKER_CRASH"
    malformed = tmp_path / "malformed.py"
    malformed.write_text("print('not-json')\n", encoding="utf-8")
    assert CargoPipelineBackend((sys.executable, str(malformed)), cwd=tmp_path).infer(frame()).failure_code == "WORKER_SCHEMA_ERROR"
