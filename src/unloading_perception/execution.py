"""Fail-closed execution authorization and stop-fact handling."""

from __future__ import annotations

from time import monotonic

from unloading_contracts import ExecutionCommand, ExecutionEvent, ExecutionEventKind, PlanningWorldSnapshot, StopAcknowledgement


class ExecutionGate:
    def __init__(self, *, enable_hardware: bool = False, hardware_adapter_verified: bool = False) -> None:
        self.enable_hardware = bool(enable_hardware)
        self.hardware_adapter_verified = bool(hardware_adapter_verified)
        self._commands: set[str] = set()
        self._active: ExecutionCommand | None = None
        self._cancel_accepted = False

    def authorize(self, command: ExecutionCommand, current_world: PlanningWorldSnapshot, *, epoch: str, generation: int) -> ExecutionEvent:
        if self.enable_hardware and not self.hardware_adapter_verified:
            return self._event(command, ExecutionEventKind.REJECTED, "HARDWARE_ADAPTER_NOT_VERIFIED")
        if command.command_id in self._commands:
            return self._event(command, ExecutionEventKind.REJECTED, "DUPLICATE_COMMAND")
        if command.epoch != epoch or command.planning_generation != generation:
            return self._event(command, ExecutionEventKind.REJECTED, "STALE_EPOCH_OR_GENERATION")
        if command.world_fingerprint != current_world.fingerprint:
            return self._event(command, ExecutionEventKind.REJECTED, "WORLD_CHANGED_BEFORE_SEND")
        if command.robot_model_fingerprint != current_world.robot_model_fingerprint:
            return self._event(command, ExecutionEventKind.REJECTED, "ROBOT_MODEL_MISMATCH")
        self._commands.add(command.command_id)
        self._active = command
        self._cancel_accepted = False
        return self._event(command, ExecutionEventKind.ACCEPTED, "MOCK_ONLY" if not self.enable_hardware else "AUTHORIZED")

    def accept_cancel(self, command_id: str) -> ExecutionEvent:
        if self._active is None or self._active.command_id != command_id:
            raise ValueError("cancel does not match active command")
        self._cancel_accepted = True
        return self._event(self._active, ExecutionEventKind.CANCEL_ACCEPTED, "controller accepted cancel request; stop not yet confirmed")

    def confirm_stop(self, positions: tuple[float, ...], velocities: tuple[float, ...], *, evidence_reference: str, event_time: float) -> StopAcknowledgement:
        if self._active is None or not self._cancel_accepted:
            raise ValueError("stop cannot be confirmed before a matching accepted cancel")
        acknowledgement = StopAcknowledgement(
            self._active.command_id, self._active.plan_id, self._active.epoch,
            self._active.planning_generation, event_time, "monotonic",
            positions, velocities, "measured_joint_velocity_below_1e-3_rad_s", evidence_reference,
        )
        self._active = None
        return acknowledgement

    @staticmethod
    def _event(command: ExecutionCommand, kind: ExecutionEventKind, message: str) -> ExecutionEvent:
        return ExecutionEvent(command.command_id, command.plan_id, command.epoch, command.planning_generation, kind, monotonic(), "monotonic", message=message)
