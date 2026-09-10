"""Fail-closed execution authorization and correlated stop-fact handling."""

from __future__ import annotations

from time import monotonic

from unloading_contracts import (
    ControllerStopFact,
    ExecutionCommand,
    ExecutionEvent,
    ExecutionEventKind,
    ExecutionGrant,
    PlanningWorldSnapshot,
    StopAcknowledgement,
)


class ExecutionGate:
    """The only domain authorization gate used by CPU and ROS adapters."""

    def __init__(self, *, enable_hardware: bool = False, hardware_adapter_verified: bool = False) -> None:
        self.enable_hardware = bool(enable_hardware)
        self.hardware_adapter_verified = bool(hardware_adapter_verified)
        self._grants: dict[str, ExecutionGrant] = {}
        self._commands: set[str] = set()
        self._active: ExecutionCommand | None = None
        self._goal_binding: tuple[str, str, str] | None = None
        self._cancel_accepted_time: float | None = None
        self._cancel_clock_domain: str | None = None
        self._last_stop_sequence: dict[tuple[str, str], int] = {}

    @property
    def active_command(self) -> ExecutionCommand | None:
        return self._active

    def register_grant(self, grant: ExecutionGrant) -> None:
        self._grants[grant.command_id] = grant

    def revoke_all(self) -> None:
        self._grants.clear()

    def authorize(self, command: ExecutionCommand, current_world: PlanningWorldSnapshot, *, epoch: str, generation: int, now: float | None = None, clock_domain: str = "monotonic") -> ExecutionEvent:
        current_time = monotonic() if now is None else float(now)
        if self.enable_hardware and not self.hardware_adapter_verified:
            return self._event(command, ExecutionEventKind.REJECTED, "HARDWARE_ADAPTER_NOT_VERIFIED", current_time, clock_domain)
        if command.command_id in self._commands:
            return self._event(command, ExecutionEventKind.REJECTED, "DUPLICATE_COMMAND", current_time, clock_domain)
        if self._active is not None:
            return self._event(command, ExecutionEventKind.REJECTED, "CONTROLLER_BUSY", current_time, clock_domain)
        if command.epoch != epoch or command.planning_generation != generation:
            return self._event(command, ExecutionEventKind.REJECTED, "STALE_EPOCH_OR_GENERATION", current_time, clock_domain)
        if command.world_fingerprint != current_world.fingerprint:
            return self._event(command, ExecutionEventKind.REJECTED, "WORLD_CHANGED_BEFORE_SEND", current_time, clock_domain)
        if command.robot_model_fingerprint != current_world.robot_model_fingerprint:
            return self._event(command, ExecutionEventKind.REJECTED, "ROBOT_MODEL_MISMATCH", current_time, clock_domain)
        if not bool(current_world.scene_snapshot.get("planning_admissible", False)):
            return self._event(command, ExecutionEventKind.REJECTED, "WORLD_NOT_PLANNING_ADMISSIBLE", current_time, clock_domain)
        grant = self._grants.get(command.command_id)
        if grant is None:
            return self._event(command, ExecutionEventKind.REJECTED, "AUTHORIZATION_GRANT_MISSING", current_time, clock_domain)
        if not grant.matches(command):
            return self._event(command, ExecutionEventKind.REJECTED, "AUTHORIZATION_GRANT_MISMATCH", current_time, clock_domain)
        if grant.clock_domain != clock_domain or current_time > grant.expires_at:
            return self._event(command, ExecutionEventKind.REJECTED, "AUTHORIZATION_GRANT_EXPIRED_OR_CLOCK_INVALID", current_time, clock_domain)
        if self.enable_hardware and grant.mock_only:
            return self._event(command, ExecutionEventKind.REJECTED, "MOCK_GRANT_FOR_HARDWARE", current_time, clock_domain)
        self._commands.add(command.command_id)
        self._active = command
        self._goal_binding = None
        self._cancel_accepted_time = None
        self._cancel_clock_domain = None
        self._grants.pop(command.command_id, None)
        return self._event(command, ExecutionEventKind.ACCEPTED, "MOCK_ONLY" if not self.enable_hardware else "AUTHORIZED", current_time, clock_domain)

    def bind_goal(self, command_id: str, *, controller_id: str, controller_epoch: str, goal_id: str) -> None:
        if self._active is None or self._active.command_id != command_id:
            raise ValueError("goal does not match active command")
        if not controller_id or not controller_epoch or not goal_id:
            raise ValueError("controller and goal identity are required")
        self._goal_binding = (controller_id, controller_epoch, goal_id)

    def reject_active_goal(self, command_id: str) -> None:
        if self._active is None or self._active.command_id != command_id:
            raise ValueError("rejected goal does not match active command")
        self._active = None
        self._goal_binding = None
        self._cancel_accepted_time = None
        self._cancel_clock_domain = None

    def accept_cancel(self, command_id: str, *, event_time: float | None = None, clock_domain: str = "monotonic") -> ExecutionEvent:
        if self._active is None or self._active.command_id != command_id:
            raise ValueError("cancel does not match active command")
        if self._goal_binding is None:
            raise ValueError("cancel cannot be accepted before goal identity is bound")
        self._cancel_accepted_time = monotonic() if event_time is None else float(event_time)
        self._cancel_clock_domain = clock_domain
        return self._event(self._active, ExecutionEventKind.CANCEL_ACCEPTED, "controller accepted cancel request; stop not yet confirmed", self._cancel_accepted_time, clock_domain)

    def complete(self, command_id: str, kind: ExecutionEventKind, message: str, *, event_time: float | None = None, clock_domain: str = "monotonic") -> ExecutionEvent:
        if self._active is None or self._active.command_id != command_id:
            raise ValueError("terminal event does not match active command")
        if kind not in (ExecutionEventKind.SUCCEEDED, ExecutionEventKind.CANCELED, ExecutionEventKind.FAILED):
            raise ValueError("terminal execution kind required")
        command = self._active
        event = self._event(command, kind, message, monotonic() if event_time is None else float(event_time), clock_domain)
        if kind is not ExecutionEventKind.CANCELED:
            self._active = None
            self._goal_binding = None
            self._cancel_accepted_time = None
            self._cancel_clock_domain = None
        return event

    def confirm_stop(self, fact: ControllerStopFact, *, now: float, max_age_seconds: float) -> StopAcknowledgement:
        if self._active is None or self._cancel_accepted_time is None or self._goal_binding is None:
            raise ValueError("stop cannot be confirmed before a matching accepted cancel")
        if max_age_seconds <= 0.0:
            raise ValueError("stop freshness threshold must be positive")
        controller_id, controller_epoch, goal_id = self._goal_binding
        if (fact.controller_id, fact.controller_epoch, fact.goal_id) != self._goal_binding:
            raise ValueError("stop fact controller/goal identity mismatch")
        if fact.clock_domain != self._cancel_clock_domain:
            raise ValueError("stop fact clock domain does not match cancel response")
        if fact.sample_time < self._cancel_accepted_time or now - fact.sample_time > max_age_seconds or now < fact.sample_time:
            raise ValueError("stop fact is stale, pre-cancel, or from the future")
        if fact.joint_names != self._active.trajectory.joint_names:
            raise ValueError("stop fact joint order mismatch")
        key = (controller_id, controller_epoch)
        if fact.sequence <= self._last_stop_sequence.get(key, -1):
            raise ValueError("duplicate or out-of-order stop fact")
        if any(abs(value) > 1e-3 for value in fact.actual_velocities):
            raise ValueError("stop fact does not meet near-zero measured velocity criterion")
        acknowledgement = StopAcknowledgement(
            self._active.command_id, self._active.plan_id, self._active.epoch,
            self._active.planning_generation, fact.sample_time, fact.clock_domain,
            fact.actual_positions, fact.actual_velocities,
            "measured_joint_velocity_below_1e-3_rad_s", fact.evidence_reference,
            fact.controller_id, fact.controller_epoch, fact.goal_id, fact.sequence,
        )
        self._last_stop_sequence[key] = fact.sequence
        self._active = None
        self._goal_binding = None
        self._cancel_accepted_time = None
        self._cancel_clock_domain = None
        return acknowledgement

    @staticmethod
    def _event(command: ExecutionCommand, kind: ExecutionEventKind, message: str, event_time: float, clock_domain: str) -> ExecutionEvent:
        return ExecutionEvent(command.command_id, command.plan_id, command.epoch, command.planning_generation, kind, event_time, clock_domain, message=message)
