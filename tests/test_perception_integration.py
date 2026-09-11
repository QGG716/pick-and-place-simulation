from dataclasses import replace
import json
from pathlib import Path
import sys

import pytest

from unloading_contracts import (
    ControllerStopFact, ExecutionCommand, ExecutionEventKind, ExecutionGrant,
    PlanArtifactKind, PlanningWorldSnapshot, ResourceReference, RobotStateRevision,
    Pose3D, SceneRevision, SensorFrame, TimedJointPoint, TimedJointTrajectory,
    UnknownRegion, Validity,
)
from unloading_perception.backends import CargoJsonReplayBackend, CargoPipelineBackend, resolve_controlled_reference
from unloading_perception.execution import DuplicateCallbackError, ExecutionGate, StaleCallbackError
from unloading_perception.geometry import transform_pose
from unloading_perception.scene import (
    ObservationTracker, SnapshotAssembler, SourceEpochGuard,
    build_scene_update, parse_mechanism_bundle,
)


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


def test_upstream_quality_head_overshoot_is_explicitly_bounded(tmp_path):
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    payload["instances"][0]["sam_iou_score"] = 1.00387
    source = tmp_path / "overshoot.json"
    source.write_text(json.dumps(payload), encoding="utf-8")
    result = CargoJsonReplayBackend().read(source)
    assert result.cargo[0].contour_score == 1.0
    assert result.cargo[0].raw_result["sam_iou_score_raw"] == 1.00387
    assert result.cargo[0].raw_result["sam_iou_score_clipped_to_contract"] is True


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


def test_ambiguous_association_retains_current_observation_and_old_occupancy():
    from dataclasses import replace
    base = observation()
    cargo = base.cargo[0]
    first = replace(cargo, source_instance_id="a", bbox_xyxy=(0.0, 0.0, 10.0, 10.0))
    second = replace(cargo, source_instance_id="b", bbox_xyxy=(0.2, 0.0, 10.2, 10.0))
    tracker = ObservationTracker(association_iou=0.5)
    tracker.update(replace(base, cargo=(first, second)))
    current = replace(cargo, source_instance_id="c", bbox_xyxy=(0.1, 0.0, 10.1, 10.0))
    result = tracker.update(replace(observation(2), cargo=(current,)))
    ambiguous = [item for item in result if item.association_status == "AMBIGUOUS"]
    assert len(ambiguous) == 1
    assert ambiguous[0].object_id.startswith("ambiguous-")
    assert sum(item.association_status == "STALE_OCCLUDED" for item in result) == 2


def test_scene_fingerprint_changes_for_position_and_rotation_but_not_heartbeat():
    from dataclasses import replace
    original = observation()
    cargo = original.cargo[1]
    update = build_scene_update(original)
    moved_pose = replace(cargo.pose, position_m=(cargo.pose.position_m[0] + 1e-6, *cargo.pose.position_m[1:]))
    moved = build_scene_update(replace(original, source_sequence=2, cargo=(replace(cargo, pose=moved_pose),)))
    rotated_pose = replace(cargo.pose, orientation_xyzw=(0.0, 0.0, 1.0, 0.0))
    rotated = build_scene_update(replace(original, source_sequence=3, cargo=(replace(cargo, pose=rotated_pose),)))
    heartbeat = build_scene_update(replace(original, source_sequence=4))
    assert moved.geometry_fingerprint != update.geometry_fingerprint
    assert rotated.geometry_fingerprint != update.geometry_fingerprint
    assert heartbeat.geometry_fingerprint == update.geometry_fingerprint


def test_unknown_only_and_failed_empty_observations_are_never_free_space():
    from dataclasses import replace
    source = observation()
    unknown_only = replace(source, cargo=(), unknown_regions=(UnknownRegion("u", "coverage", "NO_DEPTH"),))
    assert not build_scene_update(unknown_only).planning_admissible
    failed = replace(source, cargo=(), unknown_regions=(), status="FAILED", failure_code="WORKER_OOM", failure_message="deterministic injection")
    update = build_scene_update(failed)
    assert not update.planning_admissible
    assert update.unknown_regions


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


def test_freshness_and_attachment_changes_only_invalidate_world_context():
    fresh = build_scene_update(observation(), now=11.5, max_age_seconds=2.0)
    stale = build_scene_update(observation(), now=20.0, max_age_seconds=2.0)
    assert "OBSERVATION_STALE" not in fresh.blocking_reasons
    assert "OBSERVATION_STALE" in stale.blocking_reasons
    assembler = complete_assembler(fresh)
    before = assembler.assemble().snapshot
    assembler.payload_attachment = {"object_id": "track-1", "confirmed": True}
    after = assembler.assemble().snapshot
    assert after.scene_revision.sequence == before.scene_revision.sequence
    assert after.fingerprint != before.fingerprint


def admissible_snapshot():
    scene = {"obstacles": (), "unknown_regions": (), "planning_admissible": True, "blocking_reasons": ()}
    return PlanningWorldSnapshot(
        SceneRevision.from_scene(scene, 1), scene,
        RobotStateRevision(1, (0.0, 0.0), {"joint_names": ("j1", "j2"), "actual_velocities": (0.0, 0.0)}),
        {"identity": "tool"}, {"identity": "payload"}, {"identity": "base"},
        {"identity": "conveyor"}, {"identity": "config", "robot_model_fingerprint": "robot-sha", "world_model_fingerprint": "world-sha"},
    )


def trajectory():
    return TimedJointTrajectory(("j1", "j2"), (TimedJointPoint((0, 0), 0.0, (0.0, 0.0)), TimedJointPoint((1, 1), 1.0, (0.0, 0.0))), PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, "mock-fixture", "robot-sha", "config-sha")


def command(snapshot, command_id="cmd-1", epoch="epoch-a", generation=2):
    return ExecutionCommand(command_id, "plan-1", "request-1", "session-1", epoch, generation, None, snapshot.fingerprint, "robot-sha", "config-sha", "validator@1:validation-1", 2, trajectory())


def grant(value, *, expires_at=100.0, trajectory_fingerprint=None):
    return ExecutionGrant(
        "grant-" + value.command_id, value.command_id, value.plan_id, value.request_id,
        value.session_id, value.epoch, value.planning_generation, value.predecessor_plan_id,
        value.world_fingerprint, value.robot_model_fingerprint, value.config_identity,
        value.validation_reference, value.validation_generation,
        trajectory_fingerprint or value.trajectory_fingerprint, expires_at, "monotonic", True,
    )


def test_execution_gate_rejects_hardware_duplicate_and_changed_world():
    snapshot = admissible_snapshot()
    hardware = ExecutionGate(enable_hardware=True, hardware_adapter_verified=False)
    assert hardware.authorize(command(snapshot), snapshot, epoch="epoch-a", generation=2).kind is ExecutionEventKind.REJECTED
    gate = ExecutionGate()
    gate.register_grant(grant(command(snapshot)))
    assert gate.authorize(command(snapshot), snapshot, epoch="epoch-a", generation=2, now=1.0).kind is ExecutionEventKind.ACCEPTED
    assert gate.authorize(command(snapshot), snapshot, epoch="epoch-a", generation=2, now=1.0).message == "DUPLICATE_COMMAND"
    assert ExecutionGate().authorize(command(snapshot, epoch="old"), snapshot, epoch="epoch-a", generation=2).message == "STALE_EPOCH_OR_GENERATION"
    scene = dict(snapshot.scene_snapshot)
    altered = PlanningWorldSnapshot(snapshot.scene_revision, scene, snapshot.robot_state_revision, snapshot.tool_attachment, snapshot.payload_attachment, snapshot.base_state, {"identity": "moving"}, snapshot.config_identity)
    assert ExecutionGate().authorize(command(snapshot, command_id="cmd-world"), altered, epoch="epoch-a", generation=2).message == "WORLD_CHANGED_BEFORE_SEND"


def test_execution_gate_requires_exact_registered_grant():
    snapshot = admissible_snapshot()
    value = command(snapshot)
    gate = ExecutionGate()
    assert gate.authorize(value, snapshot, epoch="epoch-a", generation=2, now=1.0).message == "AUTHORIZATION_GRANT_MISSING"
    gate.register_grant(grant(value, trajectory_fingerprint="wrong"))
    assert gate.authorize(value, snapshot, epoch="epoch-a", generation=2, now=1.0).message == "AUTHORIZATION_GRANT_MISMATCH"


def test_cancel_acceptance_is_not_stop_acknowledgement():
    snapshot = admissible_snapshot()
    gate = ExecutionGate()
    value = command(snapshot)
    gate.register_grant(grant(value))
    gate.authorize(value, snapshot, epoch="epoch-a", generation=2, now=1.0)
    gate.bind_goal("cmd-1", controller_id="mock", controller_epoch="controller-a", goal_id="goal-a")
    event = gate.accept_cancel("cmd-1", event_time=2.0)
    assert event.kind is ExecutionEventKind.CANCEL_ACCEPTED
    with pytest.raises(ValueError, match="identity mismatch"):
        gate.confirm_stop(ControllerStopFact("mock", "controller-a", "wrong", 1, 2.0, 3.0, "monotonic", ("j1", "j2"), (0.2, 0.3), (0.0, 0.0), "feedback-1"), now=3.0, max_age_seconds=1.0)
    with pytest.raises(ValueError, match="near-zero"):
        gate.confirm_stop(ControllerStopFact("mock", "controller-a", "goal-a", 1, 2.0, 3.0, "monotonic", ("j1", "j2"), (0.2, 0.3), (0.0, 0.01), "feedback-1"), now=3.0, max_age_seconds=1.0)
    acknowledgement = gate.confirm_stop(ControllerStopFact("mock", "controller-a", "goal-a", 1, 2.0, 3.0, "monotonic", ("j1", "j2"), (0.2, 0.3), (0.0, 0.0), "feedback-2"), now=3.0, max_age_seconds=1.0)
    assert acknowledgement.plan_id == "plan-1"
    assert acknowledgement.goal_id == "goal-a"


def test_mechanism_bundle_is_all_or_nothing_and_requires_explicit_state():
    valid = dict(
        tool_identity="tool-a", tool_json='{"verified":true}',
        payload_identity="payload-a", payload_json='{"object_id":null}',
        base_identity="base-a", base_json='{"position_m":[0,0,0]}',
        conveyor_identity="conveyor-a", conveyor_json='{"running":false}',
        source_epoch="mechanism-a", source_sequence=1, sample_time=10.0,
    )
    committed = parse_mechanism_bundle(**valid)
    assert committed["tool_attachment"]["details"]["verified"] is True
    malformed = {**valid, "conveyor_json": '{"running":"no"}'}
    with pytest.raises(ValueError, match="boolean running"):
        parse_mechanism_bundle(**malformed)
    assert committed["conveyor_state"]["details"]["running"] is False
    with pytest.raises(ValueError, match="non-empty JSON"):
        parse_mechanism_bundle(**{**valid, "tool_json": "{}"})


def test_source_epoch_restart_rejects_late_retired_epoch():
    guard = SourceEpochGuard(retired_capacity=2)
    guard.accept("epoch-a", 4)
    guard.accept("epoch-a", 5)
    with pytest.raises(ValueError, match="explicit"):
        guard.accept("epoch-b", 0)
    guard.accept("epoch-b", 0, restart=True)
    with pytest.raises(ValueError, match="retired"):
        guard.accept("epoch-a", 6)
    with pytest.raises(ValueError, match="out-of-order"):
        guard.accept("epoch-b", 0)


def test_freshness_changes_snapshot_identity_not_geometry_revision():
    from dataclasses import replace
    source = observation()
    world_cargo = tuple(
        replace(item, pose=None if item.pose is None else replace(item.pose, frame_id="world"))
        for item in source.cargo
    )
    source = replace(source, cargo=world_cargo, unknown_regions=())
    fresh = build_scene_update(source, now=source.capture_time, max_age_seconds=2.0)
    stale = build_scene_update(source, now=source.capture_time + 3.0, max_age_seconds=2.0)
    assembler = complete_assembler(fresh)
    before = assembler.assemble().snapshot
    assembler.update = stale
    after = assembler.assemble().snapshot
    assert before.scene_revision == after.scene_revision
    assert before.fingerprint != after.fingerprint
    assert after.scene_snapshot["planning_admissible"] is False


def test_send_reservation_revocation_and_busy_check_are_atomic():
    snapshot = admissible_snapshot()
    first = command(snapshot, command_id="cmd-first")
    second = command(snapshot, command_id="cmd-second")
    gate = ExecutionGate()
    gate.register_grant(grant(first))
    gate.register_grant(grant(second))
    accepted, reservation = gate.reserve(first, snapshot, epoch="epoch-a", generation=2, now=1.0)
    assert accepted.kind is ExecutionEventKind.ACCEPTED and reservation is not None
    rejected, no_reservation = gate.reserve(second, snapshot, epoch="epoch-a", generation=2, now=1.0)
    assert rejected.message == "CONTROLLER_BUSY" and no_reservation is None
    gate.revoke_all(now=1.1)
    assert gate.commit_send(first, reservation, now=1.2).message == "SEND_RESERVATION_REVOKED"
    assert gate.active_command is None


def test_late_result_and_duplicate_stop_do_not_touch_new_active_command():
    snapshot = admissible_snapshot()
    first = command(snapshot, command_id="cmd-old")
    gate = ExecutionGate(history_capacity=4)
    gate.register_grant(grant(first))
    gate.authorize(first, snapshot, epoch="epoch-a", generation=2, now=1.0)
    gate.bind_goal(first.command_id, controller_id="mock", controller_epoch="controller-a", goal_id="goal-old")
    gate.accept_cancel(first.command_id, event_time=2.0)
    fact = ControllerStopFact("mock", "controller-a", "goal-old", 1, 2.0, 2.1, "monotonic", ("j1", "j2"), (0.0, 0.0), (0.0, 0.0), "feedback-old")
    acknowledgement = gate.confirm_stop(fact, now=2.1, max_age_seconds=1.0)
    second = command(snapshot, command_id="cmd-new")
    gate.register_grant(grant(second))
    gate.authorize(second, snapshot, epoch="epoch-a", generation=2, now=2.2)
    with pytest.raises(StaleCallbackError):
        gate.complete(first.command_id, ExecutionEventKind.SUCCEEDED, "late success", event_time=2.3)
    late = gate.complete(first.command_id, ExecutionEventKind.CANCELED, "late cancel", event_time=2.4)
    assert late.kind is ExecutionEventKind.CANCELED
    with pytest.raises(DuplicateCallbackError):
        gate.complete(first.command_id, ExecutionEventKind.CANCELED, "duplicate", event_time=2.5)
    assert gate.confirm_stop(fact, now=2.3, max_age_seconds=1.0) == acknowledgement
    assert gate.active_command.command_id == second.command_id


def test_terminal_history_is_bounded():
    snapshot = admissible_snapshot()
    gate = ExecutionGate(history_capacity=3, history_ttl_seconds=100.0)
    for index in range(12):
        value = command(snapshot, command_id=f"cmd-{index}")
        gate.register_grant(grant(value))
        gate.authorize(value, snapshot, epoch="epoch-a", generation=2, now=float(index))
        gate.complete(value.command_id, ExecutionEventKind.SUCCEEDED, "done", event_time=float(index) + 0.1)
    assert gate.history_size <= 3


def test_pending_grant_cache_is_bounded():
    snapshot = admissible_snapshot()
    gate = ExecutionGate(history_capacity=3, history_ttl_seconds=100.0)
    for index in range(12):
        gate.register_grant(grant(command(snapshot, command_id=f"cmd-{index}")))
    assert len(gate._grants) == 3


def test_pipeline_capability_fails_closed_without_interpreter(tmp_path):
    backend = CargoPipelineBackend((str(tmp_path / "missing-python"), "worker.py"), cwd=tmp_path)
    result = backend.infer(frame())
    assert result.failure_code == "BACKEND_UNAVAILABLE"
    assert not result.cargo


def test_pipeline_timeout_crash_and_bad_schema_are_distinct(tmp_path):
    digest = __import__("hashlib").sha256(FIXTURE.read_bytes()).hexdigest()
    valid_frame = frame()
    from dataclasses import replace
    valid_frame = replace(valid_frame, rgb=ResourceReference(FIXTURE.resolve().as_uri(), digest))
    sleeper = tmp_path / "sleep.py"
    sleeper.write_text("import time; time.sleep(2)\n", encoding="utf-8")
    assert CargoPipelineBackend((sys.executable, str(sleeper)), cwd=tmp_path, timeout_seconds=0.05, allowed_roots=(FIXTURE.parent,)).infer(valid_frame).failure_code == "WORKER_TIMEOUT"
    crash = tmp_path / "crash.py"
    crash.write_text("raise SystemExit(7)\n", encoding="utf-8")
    assert CargoPipelineBackend((sys.executable, str(crash)), cwd=tmp_path, allowed_roots=(FIXTURE.parent,)).infer(valid_frame).failure_code == "WORKER_CRASH"
    malformed = tmp_path / "malformed.py"
    malformed.write_text("print('not-json')\n", encoding="utf-8")
    assert CargoPipelineBackend((sys.executable, str(malformed)), cwd=tmp_path, allowed_roots=(FIXTURE.parent,)).infer(valid_frame).failure_code == "WORKER_SCHEMA_ERROR"


def test_resident_pipeline_handshake_reuses_process_and_rotates_epoch_after_crash(tmp_path):
    digest = __import__("hashlib").sha256(FIXTURE.read_bytes()).hexdigest()
    worker = tmp_path / "resident.py"
    starts = tmp_path / "starts.txt"
    worker.write_text(
        """import json, os, sys
starts = sys.argv[1]
with open(starts, 'a', encoding='utf-8') as stream:
    stream.write('start\\n')
for line in sys.stdin:
    request = json.loads(line)
    if request['op'] == 'hello':
        response = {'schema_version': '1.1.0', 'op': 'ready', 'worker_epoch': request['worker_epoch']}
    elif request['op'] == 'shutdown':
        break
    elif request['frame']['sequence'] == 3:
        raise SystemExit(7)
    else:
        response = {
            'schema_version': '1.1.0', 'request_id': request['request_id'],
            'worker_epoch': request['worker_epoch'],
            'input_sha256': request['frame']['rgb_sha256'], 'status': 'FAILED',
            'error_code': 'EXPECTED_TEST_FAILURE', 'error_message': 'no GPU in unit test',
        }
    print(json.dumps(response), flush=True)
""",
        encoding="utf-8",
    )
    backend = CargoPipelineBackend(
        (sys.executable, str(worker), str(starts)), cwd=tmp_path,
        timeout_seconds=2.0, allowed_roots=(FIXTURE.parent,), resident=True,
        worker_epoch="resident-epoch-a",
    )
    try:
        first = replace(frame(1), rgb=ResourceReference(FIXTURE.resolve().as_uri(), digest))
        second = replace(frame(2), rgb=ResourceReference(FIXTURE.resolve().as_uri(), digest))
        assert backend.infer(first).failure_code == "EXPECTED_TEST_FAILURE"
        process_id = backend._process.pid
        assert backend.infer(second).failure_code == "EXPECTED_TEST_FAILURE"
        assert backend._process.pid == process_id
        assert starts.read_text(encoding="utf-8").splitlines() == ["start"]

        crashed = replace(frame(3), rgb=ResourceReference(FIXTURE.resolve().as_uri(), digest))
        previous_epoch = backend.worker_epoch
        assert backend.infer(crashed).failure_code == "WORKER_CRASH"
        assert backend.worker_epoch != previous_epoch
        assert backend._process is None

        fourth = replace(frame(4), rgb=ResourceReference(FIXTURE.resolve().as_uri(), digest))
        assert backend.infer(fourth).failure_code == "EXPECTED_TEST_FAILURE"
        assert starts.read_text(encoding="utf-8").splitlines() == ["start", "start"]
    finally:
        backend.shutdown()


def test_pipeline_output_references_are_root_bounded_and_hash_verified(tmp_path):
    artifact = tmp_path / "artifact.json"
    artifact.write_text('{"ok":true}\n', encoding="utf-8")
    digest = __import__("hashlib").sha256(artifact.read_bytes()).hexdigest()
    backend = CargoPipelineBackend((sys.executable, "worker.py"), cwd=tmp_path, allowed_roots=(tmp_path,))
    assert backend._validated_output_reference(
        {"path": str(artifact), "sha256": digest}, name="artifact"
    ) == artifact.resolve()
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        backend._validated_output_reference({"path": str(artifact), "sha256": "0" * 64}, name="artifact")
    with pytest.raises(ValueError, match="escapes configured roots"):
        backend._validated_output_reference(
            {"path": str(FIXTURE.resolve()), "sha256": __import__("hashlib").sha256(FIXTURE.read_bytes()).hexdigest()},
            name="artifact",
        )
