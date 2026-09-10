"""CPU-only replay and explicitly synthetic closed-loop demonstrations."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from unloading_contracts import (
    SCHEMA_VERSION, CargoObservation, ControllerStopFact, EvidenceKind, ExecutionCommand, ExecutionGrant,
    ImageMapping, ObservationStatus, PerceptionObservation, PlanArtifactKind,
    Pose3D, ResourceReference, RobotStateRevision, SensorFrame,
    TimedJointPoint, TimedJointTrajectory, Validity, to_wire,
)

from .backends import CargoJsonReplayBackend, UPSTREAM_COMMIT
from .execution import ExecutionGate
from .scene import ObservationTracker, SnapshotAssembler, build_scene_update


def _frame(mode: str) -> SensorFrame:
    return SensorFrame(
        mode, "rgb", mode + "-epoch", 1, 1.0, 1.01, mode, "world" if mode == "synthetic" else "source_image",
        640, 480, "controlled-reference", ResourceReference(f"{mode}://frame/1"),
        image_mapping=ImageMapping(640, 480),
    )


def _synthetic_observation() -> PerceptionObservation:
    frame = _frame("synthetic")
    cargo = CargoObservation(
        "synthetic-source-1", None, None, "box", (100.0, 80.0, 300.0, 280.0), None,
        1.0, 1.0, 1.0, 0.005,
        Pose3D((2.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0), "world", "rotation_columns_are_local_axes", EvidenceKind.SYNTHETIC, tuple(0.0001 if index % 7 == 0 else 0.0 for index in range(36))),
        (0.4, 0.3, 0.25), None, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        EvidenceKind.SYNTHETIC, EvidenceKind.SYNTHETIC, EvidenceKind.SYNTHETIC, EvidenceKind.SYNTHETIC,
        Validity.VALID, Validity.VALID, True, (), raw_result={"mode": "synthetic_metric_fixture"},
    )
    return PerceptionObservation(
        SCHEMA_VERSION, "synthetic-observation-1", frame.epoch, frame.sequence,
        frame.capture_time, frame.receive_time, frame.clock_domain, "synthetic-metric-demo",
        "not-an-upstream-model", "synthetic-box", "synthetic_metric_v1",
        ObservationStatus.COMPLETE, None, None, (cargo,), (),
        {"frame_id": "world", "bounded_error_m": 0.01, "absence_means_free_space": False}, True,
    )


def _assemble(observation: PerceptionObservation):
    tracked = ObservationTracker().update(observation)
    update = build_scene_update(observation, tracked)
    assembler = SnapshotAssembler()
    assembler.update = update
    assembler.robot_state = RobotStateRevision(1, (0.0,) * 6, {
        "joint_names": tuple(f"joint_{index}" for index in range(1, 7)),
        "actual_velocities": (0.0,) * 6, "source": "mock_joint_state",
    })
    assembler.tool_attachment = {"tool_id": "synthetic-vacuum", "confirmed": True}
    assembler.payload_attachment = {"object_id": None, "confirmed": True}
    assembler.base_state = {"pose": (0.0, 0.0, 0.0), "confirmed": True}
    assembler.conveyor_state = {"running": False, "confirmed": True}
    assembler.config_identity = {"robot_model_fingerprint": "synthetic-robot-v1", "world_model_fingerprint": "synthetic-world-v1", "calibration": "synthetic-calibration-v1"}
    return update, assembler.assemble()


def _mock_execution(snapshot):
    points = (
        TimedJointPoint((0.0,) * 6, 0.0, (0.0,) * 6, (0.0,) * 6),
        TimedJointPoint((0.05,) * 6, 0.5, (0.0,) * 6, (0.0,) * 6),
    )
    trajectory = TimedJointTrajectory(tuple(f"joint_{index}" for index in range(1, 7)), points, PlanArtifactKind.TIME_PARAMETERIZED_TRAJECTORY, "synthetic_test_fixture_not_physical_plan", "synthetic-robot-v1", "synthetic_metric_v1")
    command = ExecutionCommand(
        "synthetic-command-1", "synthetic-plan-1", "synthetic-request-1", "synthetic-session-1",
        "synthetic-epoch", 1, None, snapshot.fingerprint, "synthetic-robot-v1", "synthetic_metric_v1",
        "mock-contract-validator@1:synthetic-validation", 1, trajectory,
    )
    gate = ExecutionGate(enable_hardware=False)
    gate.register_grant(ExecutionGrant(
        "synthetic-grant-1", command.command_id, command.plan_id, command.request_id,
        command.session_id, command.epoch, command.planning_generation,
        command.predecessor_plan_id, command.world_fingerprint,
        command.robot_model_fingerprint, command.config_identity,
        command.validation_reference, command.validation_generation,
        command.trajectory_fingerprint, 10.0, "monotonic", True,
    ))
    accepted = gate.authorize(command, snapshot, epoch="synthetic-epoch", generation=1, now=1.0)
    gate.bind_goal(command.command_id, controller_id="synthetic-mock-controller", controller_epoch="synthetic-controller-epoch", goal_id="synthetic-goal-1")
    cancel = gate.accept_cancel(command.command_id, event_time=1.5)
    stopped = gate.confirm_stop(ControllerStopFact(
        "synthetic-mock-controller", "synthetic-controller-epoch", "synthetic-goal-1", 1,
        1.5, 2.0, "monotonic", trajectory.joint_names, points[0].positions,
        points[0].velocities, "synthetic_mock_feedback",
    ), now=2.0, max_age_seconds=1.0)
    return accepted, cancel, stopped


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("replay", "synthetic"), required=True)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.mode == "replay":
        if args.input is None:
            parser.error("--input is required for replay")
        observation = CargoJsonReplayBackend(UPSTREAM_COMMIT).read(args.input, replay_time=1.0)
    else:
        observation = _synthetic_observation()
    update, assembly = _assemble(observation)
    result = {
        "mode": args.mode,
        "data_claim": "real_upstream_json_replay" if args.mode == "replay" else "explicit_synthetic_metric_fixture",
        "observation": to_wire(observation),
        "scene_update": to_wire(update),
        "snapshot_status": assembly.status,
        "snapshot": None if assembly.snapshot is None else to_wire(assembly.snapshot),
    }
    if args.mode == "synthetic" and assembly.snapshot is not None:
        accepted, cancel, stopped = _mock_execution(assembly.snapshot)
        result["mock_execution"] = {"accepted": to_wire(accepted), "cancel_accepted": to_wire(cancel), "stop_confirmed": to_wire(stopped)}
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
