"""Fail-closed execution authorization and correlated stop-fact handling."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from time import monotonic

from unloading_contracts import (
    ControllerStopFact, ExecutionCommand, ExecutionEvent, ExecutionEventKind,
    ExecutionGrant, PlanningWorldSnapshot, StopAcknowledgement,
)


class ExecutionStateError(ValueError):
    """Base error for an explicit execution-state disposition."""


class StopNotReadyError(ExecutionStateError):
    """A validly shaped stop fact arrived before cancel acceptance."""


class StaleCallbackError(ExecutionStateError):
    """A callback belongs to a command that is no longer active."""


class DuplicateCallbackError(ExecutionStateError):
    """A callback or fact was already applied."""


@dataclass(frozen=True)
class SendReservation:
    command_id: str
    token: int


class ExecutionGate:
    """Single-writer domain gate with an explicit send-commit boundary."""

    def __init__(self, *, enable_hardware: bool = False, hardware_adapter_verified: bool = False,
                 history_capacity: int = 128, history_ttl_seconds: float = 300.0) -> None:
        if history_capacity <= 0 or history_ttl_seconds <= 0.0:
            raise ValueError("history capacity and TTL must be positive")
        self.enable_hardware = bool(enable_hardware)
        self.hardware_adapter_verified = bool(hardware_adapter_verified)
        self.history_capacity = int(history_capacity)
        self.history_ttl_seconds = float(history_ttl_seconds)
        self._grants: OrderedDict[str, ExecutionGrant] = OrderedDict()
        self._history: OrderedDict[str, tuple[float, ExecutionEvent | None]] = OrderedDict()
        self._stop_cache: OrderedDict[tuple[str, str, str, int], tuple[float, StopAcknowledgement]] = OrderedDict()
        self._active: ExecutionCommand | None = None
        self._goal_binding: tuple[str, str, str] | None = None
        self._reservation: SendReservation | None = None
        self._send_committed = False
        self._next_token = 1
        self._cancel_event: ExecutionEvent | None = None
        self._cancel_accepted_time: float | None = None
        self._cancel_clock_domain: str | None = None
        self._result_seen: set[str] = set()
        self._last_stop_sequence: dict[tuple[str, str], int] = {}

    @property
    def active_command(self) -> ExecutionCommand | None:
        return self._active

    @property
    def send_committed(self) -> bool:
        return self._send_committed

    @property
    def history_size(self) -> int:
        return len(self._history) + len(self._stop_cache)

    def _prune(self, now: float) -> None:
        cutoff = float(now) - self.history_ttl_seconds
        while self._history and (next(iter(self._history.values()))[0] < cutoff or len(self._history) > self.history_capacity):
            command_id, _ = self._history.popitem(last=False)
            self._result_seen.discard(command_id)
        while self._stop_cache and (next(iter(self._stop_cache.values()))[0] < cutoff or len(self._stop_cache) > self.history_capacity):
            self._stop_cache.popitem(last=False)

    def register_grant(self, grant: ExecutionGrant) -> None:
        self._grants[grant.command_id] = grant
        self._grants.move_to_end(grant.command_id)
        while len(self._grants) > self.history_capacity:
            self._grants.popitem(last=False)

    def revoke_all(self, *, now: float | None = None, clock_domain: str = "monotonic") -> None:
        current_time = monotonic() if now is None else float(now)
        self._grants.clear()
        if self._active is not None and not self._send_committed:
            command = self._active
            event = self._event(command, ExecutionEventKind.REJECTED, "SEND_RESERVATION_REVOKED", current_time, clock_domain)
            self._remember(command.command_id, current_time, event)
            self._clear_active()

    def reserve(self, command: ExecutionCommand, current_world: PlanningWorldSnapshot, *, epoch: str,
                generation: int, now: float | None = None, clock_domain: str = "monotonic") -> tuple[ExecutionEvent, SendReservation | None]:
        current_time = monotonic() if now is None else float(now)
        self._prune(current_time)
        reason = self._rejection_reason(command, current_world, epoch=epoch, generation=generation, now=current_time, clock_domain=clock_domain)
        if reason is not None:
            return self._event(command, ExecutionEventKind.REJECTED, reason, current_time, clock_domain), None
        reservation = SendReservation(command.command_id, self._next_token)
        self._next_token += 1
        self._active, self._reservation, self._send_committed = command, reservation, False
        self._goal_binding = self._cancel_event = None
        self._cancel_accepted_time = self._cancel_clock_domain = None
        self._grants.pop(command.command_id, None)
        return self._event(command, ExecutionEventKind.ACCEPTED, "SEND_RESERVED", current_time, clock_domain), reservation

    def commit_send(self, command: ExecutionCommand, reservation: SendReservation, *, now: float | None = None,
                    clock_domain: str = "monotonic") -> ExecutionEvent:
        current_time = monotonic() if now is None else float(now)
        if self._active is None or self._active.command_id != command.command_id or self._reservation != reservation:
            return self._event(command, ExecutionEventKind.REJECTED, "SEND_RESERVATION_REVOKED", current_time, clock_domain)
        self._reservation, self._send_committed = None, True
        return self._event(command, ExecutionEventKind.ACCEPTED, "MOCK_ONLY" if not self.enable_hardware else "AUTHORIZED", current_time, clock_domain)

    def abort_reservation(self, command_id: str, *, now: float | None = None) -> None:
        if self._active is None or self._active.command_id != command_id or self._send_committed:
            raise StaleCallbackError("send reservation is not active")
        current_time = monotonic() if now is None else float(now)
        self._remember(command_id, current_time, None)
        self._clear_active()

    def authorize(self, command: ExecutionCommand, current_world: PlanningWorldSnapshot, **kwargs) -> ExecutionEvent:
        event, reservation = self.reserve(command, current_world, **kwargs)
        if reservation is None:
            return event
        return self.commit_send(command, reservation, now=kwargs.get("now"), clock_domain=kwargs.get("clock_domain", "monotonic"))

    def _rejection_reason(self, command: ExecutionCommand, current_world: PlanningWorldSnapshot, *, epoch: str,
                          generation: int, now: float, clock_domain: str) -> str | None:
        if self.enable_hardware and not self.hardware_adapter_verified:
            return "HARDWARE_ADAPTER_NOT_VERIFIED"
        if command.command_id in self._history or (self._active is not None and self._active.command_id == command.command_id):
            return "DUPLICATE_COMMAND"
        if self._active is not None:
            return "CONTROLLER_BUSY"
        if command.epoch != epoch or command.planning_generation != generation:
            return "STALE_EPOCH_OR_GENERATION"
        if command.world_fingerprint != current_world.fingerprint:
            return "WORLD_CHANGED_BEFORE_SEND"
        if command.robot_model_fingerprint != current_world.robot_model_fingerprint:
            return "ROBOT_MODEL_MISMATCH"
        if not bool(current_world.scene_snapshot.get("planning_admissible", False)):
            return "WORLD_NOT_PLANNING_ADMISSIBLE"
        first = command.trajectory.points[0]
        if len(first.positions) != len(current_world.current_q) or any(abs(a - b) > 1e-6 for a, b in zip(first.positions, current_world.current_q)):
            return "ROBOT_START_POSITION_MISMATCH"
        actual_velocities = current_world.robot_state_revision.robot_state.get("actual_velocities")
        if first.velocities is None or actual_velocities is None:
            return "ROBOT_START_VELOCITY_MISSING"
        if len(first.velocities) != len(actual_velocities) or any(abs(a - b) > 1e-6 for a, b in zip(first.velocities, actual_velocities)):
            return "ROBOT_START_VELOCITY_MISMATCH"
        grant = self._grants.get(command.command_id)
        if grant is None:
            return "AUTHORIZATION_GRANT_MISSING"
        if not grant.matches(command):
            return "AUTHORIZATION_GRANT_MISMATCH"
        if grant.clock_domain != clock_domain or now > grant.expires_at:
            return "AUTHORIZATION_GRANT_EXPIRED_OR_CLOCK_INVALID"
        if self.enable_hardware and grant.mock_only:
            return "MOCK_GRANT_FOR_HARDWARE"
        return None

    def bind_goal(self, command_id: str, *, controller_id: str, controller_epoch: str, goal_id: str) -> None:
        if self._active is None or self._active.command_id != command_id or not self._send_committed:
            raise StaleCallbackError("goal does not match a committed active command")
        if not controller_id or not controller_epoch or not goal_id:
            raise ValueError("controller and goal identity are required")
        binding = (controller_id, controller_epoch, goal_id)
        if self._goal_binding is not None and self._goal_binding != binding:
            raise StaleCallbackError("active command already has a different goal")
        self._goal_binding = binding

    def reject_active_goal(self, command_id: str, *, now: float | None = None) -> None:
        if self._active is None or self._active.command_id != command_id:
            raise StaleCallbackError("rejected goal does not match active command")
        current_time = monotonic() if now is None else float(now)
        self._remember(command_id, current_time, None)
        self._clear_active()

    def mark_controller_unknown(self, command: ExecutionCommand, message: str, *, event_time: float, clock_domain: str) -> ExecutionEvent:
        if self._active is None or self._active.command_id != command.command_id:
            raise StaleCallbackError("controller uncertainty belongs to a stale command")
        return self._event(command, ExecutionEventKind.FAILED, "CONTROLLER_STATE_UNKNOWN:" + message, event_time, clock_domain)

    def accept_cancel(self, command_id: str, *, event_time: float | None = None, clock_domain: str = "monotonic") -> ExecutionEvent:
        if self._active is None or self._active.command_id != command_id:
            raise StaleCallbackError("cancel does not match active command")
        if self._goal_binding is None:
            raise StopNotReadyError("cancel cannot be accepted before goal identity is bound")
        if self._cancel_event is not None:
            return self._cancel_event
        self._cancel_accepted_time = monotonic() if event_time is None else float(event_time)
        self._cancel_clock_domain = clock_domain
        self._cancel_event = self._event(self._active, ExecutionEventKind.CANCEL_ACCEPTED,
                                         "controller accepted cancel request; stop not yet confirmed",
                                         self._cancel_accepted_time, clock_domain)
        return self._cancel_event

    def complete(self, command_id: str, kind: ExecutionEventKind, message: str, *, event_time: float | None = None,
                 clock_domain: str = "monotonic") -> ExecutionEvent:
        current_time = monotonic() if event_time is None else float(event_time)
        self._prune(current_time)
        if command_id in self._history or command_id in self._result_seen:
            raise DuplicateCallbackError("terminal callback already applied")
        if self._active is None or self._active.command_id != command_id:
            raise StaleCallbackError("terminal event belongs to a stale command")
        if kind not in (ExecutionEventKind.SUCCEEDED, ExecutionEventKind.CANCELED, ExecutionEventKind.FAILED):
            raise ValueError("terminal execution kind required")
        command = self._active
        event = self._event(command, kind, message, current_time, clock_domain)
        self._result_seen.add(command_id)
        if kind is not ExecutionEventKind.CANCELED:
            self._remember(command_id, current_time, event)
            self._clear_active()
        return event

    def confirm_stop(self, fact: ControllerStopFact, *, now: float, max_age_seconds: float) -> StopAcknowledgement:
        key = (fact.controller_id, fact.controller_epoch, fact.goal_id, fact.sequence)
        self._prune(now)
        cached = self._stop_cache.get(key)
        if cached is not None:
            return cached[1]
        if self._active is None or self._cancel_accepted_time is None or self._goal_binding is None:
            raise StopNotReadyError("stop cannot be confirmed before a matching accepted cancel")
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
        sequence_key = (controller_id, controller_epoch)
        if fact.sequence <= self._last_stop_sequence.get(sequence_key, -1):
            raise ValueError("out-of-order stop fact")
        if any(abs(value) > 1e-3 for value in fact.actual_velocities):
            raise ValueError("stop fact does not meet near-zero measured velocity criterion")
        acknowledgement = StopAcknowledgement(self._active.command_id, self._active.plan_id, self._active.epoch,
            self._active.planning_generation, fact.sample_time, fact.clock_domain, fact.actual_positions,
            fact.actual_velocities, "measured_joint_velocity_below_1e-3_rad_s", fact.evidence_reference,
            fact.controller_id, fact.controller_epoch, fact.goal_id, fact.sequence)
        command_id = self._active.command_id
        self._last_stop_sequence[sequence_key] = fact.sequence
        self._stop_cache[key] = (float(now), acknowledgement)
        self._remember(command_id, float(now), None)
        self._clear_active()
        self._prune(now)
        return acknowledgement

    def _remember(self, command_id: str, timestamp: float, event: ExecutionEvent | None) -> None:
        self._history[command_id] = (float(timestamp), event)
        self._history.move_to_end(command_id)
        self._prune(timestamp)

    def _clear_active(self) -> None:
        self._active = self._goal_binding = self._reservation = None
        self._send_committed = False
        self._cancel_event = self._cancel_accepted_time = self._cancel_clock_domain = None

    @staticmethod
    def _event(command: ExecutionCommand, kind: ExecutionEventKind, message: str, event_time: float, clock_domain: str) -> ExecutionEvent:
        return ExecutionEvent(command.command_id, command.plan_id, command.epoch, command.planning_generation, kind, event_time, clock_domain, message=message)
